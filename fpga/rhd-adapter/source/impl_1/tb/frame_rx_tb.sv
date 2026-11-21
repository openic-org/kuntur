/*********************************************************************************
	frame_rx_tb: unit testbench for T5.1 frame_rx (lvds-tunnel.md §9.6, §6.6
	Frame mode, §4.1, §4.7; PHASESTEP rules near §6.6, "Low for at least 2 WCLK").

	What it plays
	  - Kuntur's byte stream, as §4.7: per sampling period one SAMPLE (272 B,
	    frame_index N), then directly an aux frame when due (CONFIG 38 B every
	    CFG_EVERY periods, else STATUS 34 B every STS_EVERY), then idle 0x00 up
	    to 379 / 380 bytes alternately (379.5 B per period). Bytes go out bit 0
	    first; the IDDRX4 word has the first received bit in din[0] (O2b(a)).
	    The stream starts `s` bits late in the word, so frame_rx must lock with
	    bitsel == s.
	  - frame_parse (T5.2), as a mock with the contract frame_rx relies on:
	      * a frame starts in the cycle hdr_start is high: dout = header byte 0
	        (the cycle after the 4th preamble byte was on dout);
	      * header byte 7 done: (type, payload_len) must be (01,256), (02,22) or
	        (03,18); otherwise hdr_bad (R3) and the parser is idle again;
	      * last CRC byte on dout in cycle t: frame_end in cycle t + LAT, with
	        crc_ok, is_sample, frame_index valid in that cycle only. In every
	        other cycle these three carry random values, so frame_rx must sample
	        them only with frame_end. hdr_bad likewise at byte 7 + LAT.
	      LAT = 1 (default) or 3 (the limit: an aux frame follows its SAMPLE
	      directly, so frame_rx must be searching again 4 cycles after t).
	  - pll lock (`lock`, already synchronous to the receive side in the top).

	Checks (each failure prints ERROR and counts; the run ends PASS / FAIL)
	  C1 PHASESTEP: exactly STORED_PHASE falling edges after reset, each low
	     and each high >= 2 wclk; none after start-up (no new steps on a relock).
	     pll_phasedir = 0. stepping high only during the steps.
	  C2 hdr_start: only when the 4 previous dout bytes are the preamble, and
	     never while the parser is inside a frame.
	  C3 lock rule: locked rises only after >= 2 good SAMPLEs with consecutive
	     frame_index seen since the last failure or loss; aux frames between
	     them pass. Never locks on non-consecutive indices.
	  C4 locked: bitsel == s and held while locked; in clean stretches every
	     frame sent is received with a good CRC, at LAT 1 and 3, including
	     each SAMPLE + aux pair sent back to back.
	  C5 loss of lock: 1 bad frame keeps lock; 8 consecutive failures drop it
	     (bad CRC, bad header and a bit slip); lock low drops it; relocks
	     counts each drop of locked; relock without new phase steps, at the
	     new offset after a bit slip.
	  C6 never stuck: idle-only, garbage, or a stream that cannot lock
	     (indices +2): frame_rx keeps hunting and locks once the stream is good.
	  Timing limits: lock within LOCK_LIM periods of a good stream.

	Run: tb/run_frame_rx.sh (Questa). Plusargs: +lat=1|3, +off=0..7 (stream
	offset, default 3), +seed=N; -gSP=n sets STORED_PHASE (default 5).
*********************************************************************************/

`timescale 1ns/1ps

module frame_rx_tb;
	parameter integer SP = 5;			// STORED_PHASE for the run (-gSP=...)
	parameter integer N_FAIL = 8;
	localparam real T_W = 1000.0 / 9.4875;	// wclk, ns
	localparam real T_C = 1000.0 / 12.0;	// clk, ns
	localparam integer CFG_EVERY = 4, STS_EVERY = 3;
	localparam integer LOCK_LIM = 25;		// periods

	// ---------------- clocks, DUT ----------------
	reg wclk = 0, clk = 0, rstb = 0, lock = 0;
	always #(T_W / 2) wclk = ~wclk;
	always #(T_C / 2) clk = ~clk;

	reg  [7:0] din = 0;
	reg  hdr_bad = 0, frame_end = 0, crc_ok = 0, is_sample = 0;
	reg  [31:0] frame_index = 0;
	wire pll_phasestep, pll_phasedir, hdr_start, dout_en, locked, stepping;
	wire [7:0] dout;
	wire [2:0] bitsel;
	wire [31:0] relocks;

	frame_rx #(
		.STORED_PHASE (SP),
		.LOCK_TIMEOUT (9'd100),
		.N_FAIL       (N_FAIL)
	) dut (
		.rstb(rstb), .clk(clk), .wclk(wclk), .lock(lock), .din(din),
		.hdr_bad(hdr_bad), .frame_end(frame_end), .crc_ok(crc_ok),
		.is_sample(is_sample), .frame_index(frame_index),
		.pll_phasestep(pll_phasestep), .pll_phasedir(pll_phasedir),
		.dout(dout), .hdr_start(hdr_start), .dout_en(dout_en),
		.locked(locked), .stepping(stepping), .bitsel(bitsel), .relocks(relocks)
	);

	integer errors = 0;
	integer LAT = 1;
	// each kind of error (its first 24 characters) prints 3 times, then only counts
	int n_kind [string];
	task automatic err(input string s);
		string k;
		begin
			errors = errors + 1;
			k = s.substr(0, (s.len() < 24 ? s.len() : 24) - 1);
			if (!n_kind.exists(k)) n_kind[k] = 0;
			n_kind[k]++;
			if (n_kind[k] <= 3) $display("%t ERROR %s", $realtime, s);
			if (n_kind[k] == 3) $display("%t   (further \"%s...\" errors counted, not printed)", $realtime, k);
		end
	endtask
	task automatic err_summary;
		string k;
		if (n_kind.first(k)) do
			if (n_kind[k] > 3) $display("  %0d x \"%s...\"", n_kind[k], k);
		while (n_kind.next(k));
	endtask

	// ---------------- CRC-32 (§4.6) ----------------
	function automatic [31:0] crc_byte(input [31:0] c, input [7:0] b);
		integer k;
		begin
			c = c ^ b;
			for (k = 0; k < 8; k = k + 1) c = c[0] ? (c >> 1) ^ 32'hEDB88320 : c >> 1;
			crc_byte = c;
		end
	endfunction

	// ---------------- stream generator ----------------
	// tb controls, read when a period is generated (about one period ahead)
	integer s_off = 3;			// bit offset of the stream in the word
	integer bad_crc_n = 0;		// next n frames: one payload bit flipped
	integer bad_hdr_n = 0;		// next n frames: payload_len 255
	integer bad_pre_n = 0;		// next n SAMPLEs: preamble byte 2 corrupted
	integer idx_step = 1;		// frame_index increment per period
	bit     frames_on = 1;		// 0: idle only
	bit     garbage = 0;		// 1: random bytes instead of the stream
	integer slip_req = 0;		// insert this many extra bits at the next period
	bit     slipping = 0;		// tb: from a slip until the relock (bitsel check off)

	bit bits[$];				// bits not yet sent, first = next
	typedef struct { int unsigned bitpos; int kind; bit good; } mark_t;	// kind 0 period start
	mark_t marks[$];
	int unsigned bits_out = 0;	// bits serialised so far (absolute)
	int unsigned bits_in = 0;	// bits pushed so far (absolute)
	reg [31:0] gen_index = 0;
	integer gen_period = 0;

	// counters advanced when the serialiser passes a mark
	integer sent_good = 0;		// frames sent intact
	integer sent_periods = 0;

	task automatic push_byte(input [7:0] b);
		integer k;
		for (k = 0; k < 8; k = k + 1) begin bits.push_back(b[k]); bits_in++; end
	endtask

	task automatic push_frame(input [7:0] typ, input [15:0] len, output integer nbytes);
		reg [7:0] hdr [0:7];
		reg [7:0] pl [0:255];
		reg [31:0] c;
		reg corrupt_crc, corrupt_hdr, corrupt_pre;
		integer k, flip;
		mark_t m;
		begin
			corrupt_crc = 0; corrupt_hdr = 0; corrupt_pre = 0;
			if (bad_crc_n > 0) begin corrupt_crc = 1; bad_crc_n--; end
			if (bad_hdr_n > 0) begin corrupt_hdr = 1; bad_hdr_n--; end
			if (typ == 8'h01 && bad_pre_n > 0) begin corrupt_pre = 1; bad_pre_n--; end
			hdr[0] = typ; hdr[1] = 8'h00;
			{hdr[5], hdr[4], hdr[3], hdr[2]} = gen_index;
			{hdr[7], hdr[6]} = corrupt_hdr ? 16'd255 : len;
			for (k = 0; k < len; k = k + 1) begin
				pl[k] = $urandom;
				// keep random payloads from holding the preamble (an accidental
				// one is legal, but it is never examined and only confuses logs)
				if (k >= 3 && pl[k] == 8'h5A && pl[k-1] == 8'hA5 && pl[k-2] == 8'h5A && pl[k-3] == 8'hA5)
					pl[k] = 8'h00;
			end
			c = 32'hFFFFFFFF;
			for (k = 0; k < 8; k = k + 1) c = crc_byte(c, hdr[k]);
			for (k = 0; k < len; k = k + 1) c = crc_byte(c, pl[k]);
			c = ~c;
			if (corrupt_crc) begin flip = $urandom % len; pl[flip] = pl[flip] ^ (8'h01 << ($urandom % 8)); end
			push_byte(8'hA5); push_byte(8'h5A);
			push_byte(corrupt_pre ? 8'hA4 : 8'hA5); push_byte(8'h5A);
			for (k = 0; k < 8; k = k + 1) push_byte(hdr[k]);
			for (k = 0; k < len; k = k + 1) push_byte(pl[k]);
			push_byte(c[7:0]); push_byte(c[15:8]); push_byte(c[23:16]); push_byte(c[31:24]);
			m.bitpos = bits_in; m.kind = 1; m.good = !(corrupt_crc || corrupt_hdr || corrupt_pre);
			marks.push_back(m);
			nbytes = 4 + 8 + len + 4;
		end
	endtask

	task automatic gen_one_period;
		integer used, n, k, slot;
		mark_t m;
		begin
			slot = (gen_period % 2) ? 380 : 379;
			while (slip_req > 0) begin bits.push_back(1'b0); bits_in++; slip_req--; end
			m.bitpos = bits_in; m.kind = 0; m.good = 1;
			marks.push_back(m);
			used = 0;
			if (garbage) begin
				for (k = 0; k < slot; k = k + 1) push_byte($urandom);
				used = slot;
			end else if (frames_on) begin
				push_frame(8'h01, 16'd256, n); used = used + n;
				if (gen_period % CFG_EVERY == 1) begin push_frame(8'h02, 16'd22, n); used = used + n; end
				else if (gen_period % STS_EVERY == 2) begin push_frame(8'h03, 16'd18, n); used = used + n; end
			end
			for (k = used; k < slot; k = k + 1) push_byte(8'h00);
			gen_index = gen_index + idx_step;
			gen_period++;
		end
	endtask

	// serialiser: one IDDRX4 word per wclk, first received bit in din[0]
	always @(posedge wclk) begin
		reg [7:0] w;
		integer k;
		while (bits.size() < 4000) gen_one_period;
		for (k = 0; k < 8; k = k + 1) w[k] = bits.pop_front();
		bits_out = bits_out + 8;
		while (marks.size() > 0 && marks[0].bitpos <= bits_out) begin
			if (marks[0].kind == 0) sent_periods++;
			else if (marks[0].good) sent_good++;
			void'(marks.pop_front());
		end
		din <= #1 w;
	end

	// ---------------- mock frame_parse ----------------
	reg [7:0] hist [0:3];		// dout of the 4 previous cycles, [0] newest
	bit     busy = 0;
	integer bno = 0;
	reg [7:0] p_type;
	reg [15:0] p_len;
	reg [31:0] p_idx, p_crc, p_rx_crc;
	integer fe_at = -1, hb_at = -1;		// cycle numbers of scheduled pulses
	reg fe_crc, fe_smp;
	reg [31:0] fe_idx;
	integer cyc = 0;

	// parser counters
	integer rx_good = 0, rx_good_smp = 0, rx_crc_fail = 0, rx_hdr_bad = 0;
	integer run_good = 0;		// consecutive good SAMPLEs with index + 1
	reg [31:0] last_idx = 0;
	bit have_idx = 0;

	always @(negedge wclk) begin
		integer k;
		cyc++;
		// outputs for this cycle (seen by frame_rx at the next posedge)
		frame_end = (cyc == fe_at);
		hdr_bad = (cyc == hb_at);
		if (frame_end) begin crc_ok = fe_crc; is_sample = fe_smp; frame_index = fe_idx; end
		else begin crc_ok = $urandom; is_sample = $urandom; frame_index = $urandom; end

		// C2 hdr_start, and frame start
		if (hdr_start) begin
			if (!({hist[3], hist[2], hist[1], hist[0]} == 32'hA55AA55A))
				err($sformatf("C2 hdr_start without the preamble just before it (prev dout %h %h %h %h, dout %h)",
					hist[3], hist[2], hist[1], hist[0], dout));
			if (busy) err("C2 hdr_start while the parser is inside a frame");
			busy = 1; bno = 0;
		end
		if (busy) begin
			if (bno == 0) p_crc = 32'hFFFFFFFF;
			if (bno < 8 + p_len || bno < 8) p_crc = crc_byte(p_crc, dout);
			case (bno)
				0: p_type = dout;
				2: p_idx[7:0] = dout;  3: p_idx[15:8] = dout;
				4: p_idx[23:16] = dout; 5: p_idx[31:24] = dout;
				6: p_len[7:0] = dout;  7: p_len[15:8] = dout;
			endcase
			if (bno == 7) begin
				if (!((p_type == 8'h01 && p_len == 256) || (p_type == 8'h02 && p_len == 22) ||
				      (p_type == 8'h03 && p_len == 18))) begin
					hb_at = cyc + LAT; busy = 0;
					rx_hdr_bad++; run_good = 0;
				end
			end
			if (busy && bno >= 8 + p_len) p_rx_crc[8 * (bno - 8 - p_len) +: 8] = dout;
			if (busy && bno == 8 + p_len + 3) begin
				fe_at = cyc + LAT; busy = 0;
				fe_crc = (~p_crc == p_rx_crc); fe_smp = (p_type == 8'h01); fe_idx = p_idx;
				if (!fe_crc) begin rx_crc_fail++; run_good = 0; end
				else begin
					rx_good++;
					if (fe_smp) begin
						rx_good_smp++;
						if (have_idx && p_idx == last_idx + 1) run_good++; else run_good = 1;
						last_idx = p_idx; have_idx = 1;
					end
				end
			end
			bno++;
		end
		for (k = 3; k > 0; k = k - 1) hist[k] = hist[k-1];
		hist[0] = dout;
	end

	// ---------------- C1 PHASESTEP ----------------
	integer n_steps = 0;
	realtime t_ps_fall = 0, t_ps_rise = -1.0e9;
	bit startup_done = 0;
	always @(negedge pll_phasestep) begin
		if (rstb) begin
			n_steps++;
			if ($realtime - t_ps_rise < 2.0 * T_W - 1.0)
				err($sformatf("C1 PHASESTEP high only %0.1f ns before a step", $realtime - t_ps_rise));
			if (startup_done) err("C1 PHASESTEP after start-up (a relock must not step)");
			else if (stepping !== 1'b1) err("C1 stepping not high at a start-up PHASESTEP");
		end
		t_ps_fall = $realtime;
	end
	always @(posedge pll_phasestep) begin
		if (rstb && $realtime - t_ps_fall < 2.0 * T_W - 1.0)
			err($sformatf("C1 PHASESTEP low only %0.1f ns", $realtime - t_ps_fall));
		t_ps_rise = $realtime;
	end
	always @(posedge wclk) if (rstb && pll_phasedir !== 1'b0) err("C1 pll_phasedir not 0");
	always @(posedge wclk) if (rstb && startup_done && stepping) err("C1 stepping high after start-up");

	// ---------------- C3 / C4 / C5 lock monitor ----------------
	integer n_lock_rise = 0, n_lock_fall = 0;
	reg [31:0] relocks_at_reset = 0;
	reg locked_q = 0;
	always @(posedge wclk) if (rstb) begin
		if (locked && !locked_q) begin
			n_lock_rise++;
			if (run_good < 2)
				err($sformatf("C3 locked after %0d consecutive good SAMPLE(s); needs 2", run_good));
			if (bitsel !== s_off[2:0])
				err($sformatf("C4 locked at bitsel %0d, stream offset %0d", bitsel, s_off));
		end
		if (!locked && locked_q) begin n_lock_fall++; run_good = 0; end
		if (locked && locked_q && bitsel !== s_off[2:0] && !slipping)
			err($sformatf("C4 bitsel %0d while locked, stream offset %0d", bitsel, s_off));
		locked_q <= locked;
	end

	// ---------------- helpers ----------------
	task automatic wait_periods(input integer n);
		integer p0;
		begin p0 = sent_periods; wait (sent_periods >= p0 + n); end
	endtask

	// lock within lim periods, or error
	task automatic expect_lock(input string what, input integer lim);
		integer p0;
		begin
			p0 = sent_periods;
			fork : wl
				wait (locked === 1'b1);
				wait (sent_periods >= p0 + lim);
			join_any
			disable wl;
			if (locked !== 1'b1) err($sformatf("%s: no lock within %0d periods", what, lim));
			else $display("%t   %s: locked after %0d periods, bitsel %0d", $realtime, what,
				sent_periods - p0, bitsel);
		end
	endtask

	// a clean stretch: every frame sent arrives with a good CRC, lock held
	task automatic clean_stretch(input string what, input integer n);
		integer g0, s0, f0, h0, l0;
		begin
			wait_periods(1);		// a period boundary: nothing in flight
			g0 = rx_good; s0 = sent_good; f0 = rx_crc_fail; h0 = rx_hdr_bad; l0 = n_lock_fall;
			wait_periods(n);
			if (rx_good - g0 != sent_good - s0 || rx_crc_fail != f0 || rx_hdr_bad != h0)
				err($sformatf("C4 %s: sent %0d frames, received %0d good, %0d CRC fail, %0d hdr_bad",
					what, sent_good - s0, rx_good - g0, rx_crc_fail - f0, rx_hdr_bad - h0));
			if (n_lock_fall != l0) err($sformatf("C4 %s: lock lost in a clean stretch", what));
			$display("%t   %s: %0d periods, %0d frames received intact", $realtime, what, n, rx_good - g0);
		end
	endtask

	task automatic expect_unlock(input string what, input integer lim);
		integer p0;
		begin
			p0 = sent_periods;
			fork : wu
				wait (locked === 1'b0);
				wait (sent_periods >= p0 + lim);
			join_any
			disable wu;
			if (locked !== 1'b0) err($sformatf("%s: still locked after %0d periods", what, lim));
		end
	endtask

	// ---------------- scenario ----------------
	integer seed = 1, r0, k;
	initial begin
		if ($value$plusargs("lat=%d", LAT)) ;
		if ($value$plusargs("seed=%d", seed)) ;
		if ($value$plusargs("off=%d", s_off)) ;
		for (k = 0; k < s_off; k = k + 1) begin bits.push_back(1'b0); bits_in++; end
		r0 = $urandom(seed);
		$display("frame_rx_tb: STORED_PHASE %0d, N_FAIL %0d, LAT %0d, seed %0d", SP, N_FAIL, LAT, seed);
		// self-test of the tb's CRC: "123456789" -> CBF43926
		begin
			reg [31:0] c; reg [8*9-1:0] s; c = 32'hFFFFFFFF; s = "123456789";
			for (k = 8; k >= 0; k = k - 1) c = crc_byte(c, s[8*k +: 8]);
			if (~c != 32'hCBF43926) err("tb CRC self-test");
		end

		// T1 start-up: steps, hunt at offset 3, lock
		$display("T1 start-up, stream offset %0d", s_off);
		#(10 * T_W) lock = 1;
		#(20 * T_W) rstb = 1;
		wait (n_steps >= SP);
		#(40 * T_W);
		if (n_steps != SP) err($sformatf("C1 %0d PHASESTEP pulses, expected %0d", n_steps, SP));
		startup_done = 1;
		expect_lock("T1", LOCK_LIM);
		relocks_at_reset = relocks;
		clean_stretch("T1 steady (LAT and back-to-back aux)", 2 * CFG_EVERY * STS_EVERY);

		// T2 one bad CRC: lock kept
		$display("T2 one bad CRC");
		bad_crc_n = 1;
		wait_periods(3);
		if (n_lock_fall != 0) err("C5 T2: lock lost on a single bad frame");
		clean_stretch("T2 after", 6);

		// T3 N_FAIL-1 bad frames, then good: lock kept (failures reset by a good frame)
		$display("T3 %0d bad frames", N_FAIL - 1);
		bad_crc_n = N_FAIL - 1;
		wait_periods(N_FAIL + 2);
		if (n_lock_fall != 0) err($sformatf("C5 T3: lock lost on %0d bad frames", N_FAIL - 1));
		bad_crc_n = N_FAIL - 1;			// again: a counter not cleared would reach N_FAIL
		wait_periods(N_FAIL + 2);
		if (n_lock_fall != 0) err("C5 T3: lock lost on two separated runs of N_FAIL-1 (failure count not cleared by a good frame?)");
		clean_stretch("T3 after", 6);

		// T4 N_FAIL bad CRCs: unlock, relock at the same offset, no new steps
		$display("T4 %0d consecutive bad CRCs", N_FAIL);
		bad_crc_n = N_FAIL;
		expect_unlock("C5 T4", N_FAIL + 4);
		expect_lock("T4 relock", LOCK_LIM);
		clean_stretch("T4 after", 6);

		// T5 bad header (R3), single then N_FAIL
		$display("T5 bad header (payload_len 255)");
		begin
			integer f0; f0 = n_lock_fall;
			bad_hdr_n = 1;
			wait_periods(3);
			if (n_lock_fall != f0) err("C5 T5: lock lost on one bad header");
		end
		clean_stretch("T5 after one", 4);
		bad_hdr_n = N_FAIL;
		expect_unlock("C5 T5 (8 bad headers)", N_FAIL + 4);
		bad_hdr_n = 0;
		expect_lock("T5 relock", LOCK_LIM);
		clean_stretch("T5 after", 4);

		// T6 missing preamble: the frame is invisible; frame_rx must catch the next
		$display("T6 one corrupted preamble");
		begin
			integer f0; f0 = n_lock_fall;
			bad_pre_n = 1;
			wait_periods(3);
			if (n_lock_fall != f0) err("C5 T6: lock lost on one missing preamble");
		end
		clean_stretch("T6 after", 4);

		// T7 pll lock low: unlock, relock, no steps
		$display("T7 lock low 20 us");
		lock = 0;
		#(30 * T_W);
		if (locked !== 1'b0) err("C5 T7: still locked 30 wclk after lock fell");
		#20000 lock = 1;
		expect_lock("T7 relock", LOCK_LIM);
		clean_stretch("T7 after", 4);

		// T7b short lock drop just after a good SAMPLE: the next SAMPLE's index is
		// the last one seen + 1, so a stale "previous index" would lock on 1 frame
		$display("T7b lock low 10 wclk, right after a SAMPLE");
		begin
			integer p0; p0 = sent_periods;
			fork : w7b
				@(negedge wclk iff (frame_end && crc_ok && is_sample));
				wait (sent_periods >= p0 + 3);
			join_any
			disable w7b;
			if (sent_periods >= p0 + 3) err("T7b: no good SAMPLE received in 3 periods");
		end
		lock = 0;
		#(10 * T_W) lock = 1;
		expect_lock("T7b relock", LOCK_LIM);
		clean_stretch("T7b after", 4);

		// T8 bit slip: all frames fail, unlock, relock at the new offset
		$display("T8 one-bit slip");
		slipping = 1;
		slip_req = 1;
		wait (slip_req == 0);
		s_off = (s_off + 1) % 8;
		expect_unlock("C5 T8 (no preamble is seen at the old offset: needs a SEEK window without a preamble to count as a failure)", N_FAIL + 6);
		slipping = 0;
		expect_lock("T8 relock", LOCK_LIM);
		clean_stretch("T8 after", 4);

		// T9 indices +2: must never lock; then +1: locks
		$display("T9 frame_index steps of 2 (no lock allowed), then 1");
		lock = 0; idx_step = 2;
		wait_periods(3);			// frames already generated (about 1 period ahead) still step by 1
		lock = 1;
		wait_periods(20);
		if (locked) err("C3 T9: locked on frame_index steps of 2");
		idx_step = 1;
		expect_lock("T9 relock", LOCK_LIM);

		// T10 idle only, then garbage, then frames: keeps hunting, locks
		$display("T10 idle 10 periods, garbage 10, then frames");
		frames_on = 0;
		expect_unlock("C6 T10 idle (needs a SEEK window without a preamble to count as a failure)", 2 * N_FAIL + 4);
		wait_periods(10);
		garbage = 1;
		wait_periods(10);
		if (locked) err("C6 T10: locked on garbage");
		garbage = 0; frames_on = 1;
		expect_lock("T10 relock", 110);	// may need LOCK_TIMEOUT (100 periods) to restart
		clean_stretch("T10 after", 4);

		// relocks counts each drop of locked
		if (relocks - relocks_at_reset != n_lock_fall)
			err($sformatf("C5 relocks moved by %0d, locked fell %0d times", relocks - relocks_at_reset, n_lock_fall));
		$display("lock rises %0d, falls %0d, relocks %0d, steps %0d", n_lock_rise, n_lock_fall, relocks, n_steps);
		$display("parser: %0d good (%0d SAMPLE), %0d CRC fail, %0d hdr_bad", rx_good, rx_good_smp, rx_crc_fail, rx_hdr_bad);
		err_summary;
		if (errors == 0) $display("frame_rx_tb: PASS");
		else $display("frame_rx_tb: FAIL, %0d errors", errors);
		$finish;
	end

	initial begin #200ms; err("timeout"); err_summary; $display("frame_rx_tb: FAIL"); $finish; end
endmodule
