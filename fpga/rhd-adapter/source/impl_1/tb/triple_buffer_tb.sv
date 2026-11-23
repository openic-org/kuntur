/*********************************************************************************
	triple_buffer_tb: unit testbench for T5.3 triple_buffer (lvds-tunnel.md
	§9.6 T5.3, "Added 2026-11-23", Q1-Q5, R2, R5; write port as T5.2's, §9.6
	T5.2 "Added 2026-11-23"; Intan timing as intan_master_bfm.v, §9.1a).

	The tb plays both neighbours.
	  Writer (wclk, Kuntur's clock, +ppm against Intan): T5.2's write port, one
	    Kuntur period of 379/380 wclk each. A SAMPLE's 128 buf_we words at
	    byte 9+2k (+1..3 wclk), then smp_commit (good) or smp_invalid (bad CRC)
	    after the last CRC byte (+1..3 wclk); also header failures (invalid at
	    byte 7), cut frames (k words, then invalid), partial frames with
	    neither, silent losses (nothing), and Kuntur stopped. buf_waddr,
	    buf_wdata, smp_index, smp_flags carry random values outside their
	    strobes. Word k of the frame with index i is wv(i, k), so every read
	    tells which frame and slot it came from.
	  Reader (SCLK, RHX and T5.4): 35 words per sample period (CONVERT(0..31),
	    3 aux), 80 Intan ticks each, SCLK rising at ticks 4i+2. At the 16th
	    falling edge of word N (the slave's data_rx load) the tb drives
	    rd_slot = C of word N's CONVERT(C) (random for aux words), and
	    frame_start = 1 for CONVERT(0). frame_start drops after the 1st
	    rising edge of word N+1 (+fs_level: held through word N+1, as a
	    decode of data_rx would be). rd_*, rd_valid, serve_* are sampled
	    just before the 2nd rising edge of word N+1 (tx_load).

	Timing the tb assumes (to confirm with Manuel; lvds-tunnel.md §9.6 T5.3):
	  - frame_start is acted on at the 1st rising SCLK edge of word N+1, and
	    the reads of that edge already come from the new frame's bank.
	  - An entry (smp_commit or smp_invalid) at least W_VIS (3.5 us, +wvis=ns)
	    before that edge is seen by the decision; a later one by it or the next.
	  - link_down changes only in a CS-high gap (here before words 4..29):
	    rd_valid is 0 from the word after next on (two flops on SCLK); after
	    link_down falls, rd_valid is free for the rest of that frame.
	  - Counters are checked just before the 2nd rising edge of word 3.

	The decision at each frame_start, as the tb reads §9.6 T5.3 (Q1-Q5):
	  link_down: sentinel (rd_valid 0); slip counters unchanged (the valid
	    entries it consumes may count in slip_skip or not).
	  no new entry: held frame valid and not itself a repeat: serve it again,
	    slip_repeat + 1 (Q2, Q4); otherwise sentinel.
	  newest entry invalid: sentinel (R2); slip_skip + the valid entries passed.
	  newest entry valid: take it; slip_skip + the valid entries passed (Q3).
	  A sentinel frame leaves nothing held: the next frame is valid only with
	  a new valid entry. After reset (either side): sentinel until the first
	  valid entry; slip_skip free at the first take.

	Checks (each failure prints ERROR and counts; the run ends PASS / FAIL)
	  D1 data: every read with rd_valid = 1 is slot C of the frame serve_index
	     names (rd_a0, rd_b0, rd_a1, rd_b1 = words 4C..4C+3, §4.3 order),
	     that frame was committed good, serve_flags is its flags, and
	     serve_index stays the same through the Intan frame. This is what
	     catches the writer writing the bank being read (Q1).
	  D2 decision: the outcome at each frame_start is one the rules above
	     allow for some entry the reader may have seen (W_VIS).
	  D3 rd_valid: as decided, through the frame; 0 while link_down (R5).
	  D4 counters: slip_repeat exact, slip_skip within the bounds of D2,
	     frames_sentinel_served + 1 per Intan frame with any read rd_valid 0
	     (Q5); saturating (+define+TB_SAT deposits values near 2^32 into the
	     DUT's registers of those names).
	  D5 resets: rstb_w (cdr_rstb) pulsed while link_down is high; rstb_s
	     (rst_n_emu) while SCLK is stopped. After either, no stale data.
	  D6 RHX stops (SCLK stopped for 3, 64, 256 Kuntur periods while Kuntur
	     runs): the first frame after is the newest entry, or the sentinel,
	     never an older frame.
	  Rates: Kuntur 25k / Intan 25k (bring-up) with +-3000 ppm and +-2 %
	  drifts, and the pairs of §9.5a: Kuntur 28k with Intan 30k (repeats) and
	  25k (skips), and Intan at 20k (an operator mistake). At 25/25 the writer
	  refills a bank faster than the reader reads it and starts >= 11 us after
	  a commit, so a wrong bank choice is masked; with Intan slower the reader
	  holds a bank long enough for the writer to overtake it.

	Run: tb/run_triple_buffer.sh (Questa). Plusargs: +seed=N, +soak=N (random
	frames at the end, default 1500), +fs_level, +wvis=ns.
*********************************************************************************/

`timescale 1ns/1ps

module triple_buffer_tb;
	localparam [31:0] MAXC = 32'hFFFFFFFF;
	localparam longint FREE = 64'h7FFF_FFFF_FFFF;

	integer errors = 0;
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

	function automatic [15:0] wv(input [31:0] idx, input [6:0] a);
		reg [31:0] h;
		begin
			h = (idx * 32'h9E3779B1) ^ ({25'd0, a} * 32'h85EBCA77) ^ {a, 25'd0};
			h = h ^ (h >> 15);
			wv = h[31:16] ^ h[15:0];
		end
	endfunction
	function automatic longint satl(input longint a);
		satl = (a > MAXC) ? MAXC : a;
	endfunction

	// ---------------------------------------------------------------- DUT
	// Rates (§9.5a): Kuntur 25 or 28 kS/s, 379.5 wclk per period (9.4875 / 10.626 MHz);
	// Intan fs, 35 words of 80 ticks per period (SCLK = tick / 4).
	int k_rate = 25000, i_rate = 25000;
	real t_w0 = 1.0e9 / (379.5 * 25000.0);		// wclk, ns, before ppm
	real t_k = 1.0e9 / (2800.0 * 25000.0);		// Intan tick, ns
	real t_wh = 1.0e9 / (379.5 * 25000.0) / 2.0;
	real ppm = 0.0;
	reg wclk = 0, sck = 0;
	always #(t_wh) wclk = ~wclk;
	task automatic set_ppm(input real p);	// > 0: Kuntur fast (skips), < 0: slow (repeats)
		ppm = p; t_wh = t_w0 / (1.0 + p * 1.0e-6) / 2.0;
	endtask
	task automatic set_rates(input int kr, input int ir);
		k_rate = kr; i_rate = ir;
		t_w0 = 1.0e9 / (379.5 * kr); t_k = 1.0e9 / (2800.0 * ir);
		set_ppm(ppm);
	endtask

	reg rstb_w = 0, rstb_s = 0;
	reg buf_we = 0; reg [6:0] buf_waddr = 0; reg [15:0] buf_wdata = 0;
	reg smp_commit = 0, smp_invalid = 0; reg [31:0] smp_index = 0; reg [7:0] smp_flags = 0;
	reg frame_start = 0; reg [4:0] rd_slot = 0; reg link_down = 0;
	wire [15:0] rd_a0, rd_b0, rd_a1, rd_b1;
	wire rd_valid;
	wire [31:0] serve_index; wire [7:0] serve_flags;
	wire [31:0] slip_repeat, slip_skip, frames_sentinel_served;

	triple_buffer dut (
		.rstb_w(rstb_w), .wclk(wclk),
		.buf_we(buf_we), .buf_waddr(buf_waddr), .buf_wdata(buf_wdata),
		.smp_commit(smp_commit), .smp_invalid(smp_invalid), .smp_index(smp_index), .smp_flags(smp_flags),
		.rstb_s(rstb_s), .sck(sck), .frame_start(frame_start), .rd_slot(rd_slot), .link_down(link_down),
		.rd_a0(rd_a0), .rd_b0(rd_b0), .rd_a1(rd_a1), .rd_b1(rd_b1), .rd_valid(rd_valid),
		.serve_index(serve_index), .serve_flags(serve_flags),
		.slip_repeat(slip_repeat), .slip_skip(slip_skip), .frames_sentinel_served(frames_sentinel_served)
	);

	// ---------------------------------------------------------------- entries
	// Every smp_commit / smp_invalid the DUT took, in order; pv[n] = valid ones among the first n.
	real ev_t [$]; bit ev_v [$]; reg [31:0] ev_i [$]; reg [7:0] ev_f [$];
	int pv [$];
	bit good_idx [int];
	initial pv.push_back(0);
	always @(posedge wclk) if (rstb_w && (smp_commit || smp_invalid)) begin
		ev_t.push_back($realtime); ev_v.push_back(smp_commit);
		ev_i.push_back(smp_index); ev_f.push_back(smp_flags);
		pv.push_back(pv[pv.size() - 1] + smp_commit);
		if (smp_commit) good_idx[smp_index] = 1;
	end
	function automatic int nvalid(input int a, input int b);	// valid entries a..b-1
		nvalid = (b > a) ? pv[b] - pv[a] : 0;
	endfunction

	// ---------------------------------------------------------------- writer (T5.2)
	typedef enum int {GOOD, BADCRC, HDRBAD, CUT, PARTIAL, SILENT, STOP} kind_t;
	int n_per_kind [7];
	reg [31:0] kidx = 32'd5000;
	bit wk_on = 0, wk_hold = 0;
	int wk_stop = 0, wk_force = -1, p_fail = 0;
	bit wlen_alt = 0;

	task automatic kperiod(input kind_t kind);
		int len, r, k, lw, lc, kcut;
		reg [7:0] fl; reg [31:0] idx;
		len = wlen_alt ? 380 : 379; wlen_alt = !wlen_alt;
		lw = $urandom_range(1, 3); lc = $urandom_range(1, 3); kcut = $urandom_range(1, 127);
		idx = kidx; fl = $urandom;
		n_per_kind[kind]++;
		for (r = -4; r < len - 4; r++) begin	// r: byte number in the SAMPLE frame (0 = header byte 0)
			@(negedge wclk);
			buf_we <= 0; buf_waddr <= $urandom; buf_wdata <= $urandom;
			smp_commit <= 0; smp_invalid <= 0; smp_index <= $urandom; smp_flags <= $urandom;
			if ((kind == GOOD || kind == BADCRC || kind == CUT || kind == PARTIAL)
					&& r >= 9 + lw && ((r - 9 - lw) % 2) == 0) begin
				k = (r - 9 - lw) / 2;
				if (k < 128 && (kind == GOOD || kind == BADCRC || k < kcut)) begin
					buf_we <= 1; buf_waddr <= k; buf_wdata <= wv(idx, k);
				end
			end
			if (kind == GOOD && r == 267 + lc) begin smp_commit <= 1; smp_index <= idx; smp_flags <= fl; end
			if (kind == BADCRC && r == 267 + lc) smp_invalid <= 1;
			if (kind == HDRBAD && r == 7 + lc) smp_invalid <= 1;
			if (kind == CUT && r == 9 + lw + 2 * kcut + 1) smp_invalid <= 1;
		end
		if (kind != STOP) kidx = kidx + 1;
	endtask

	function automatic kind_t next_kind;
		int r;
		if (wk_hold) return STOP;
		if (wk_stop > 0) begin wk_stop--; return STOP; end
		if (wk_force >= 0) begin next_kind = kind_t'(wk_force); wk_force = -1; return next_kind; end
		r = $urandom_range(0, 99);
		if (r >= p_fail) return GOOD;
		return kind_t'($urandom_range(BADCRC, SILENT));
	endfunction

	initial begin : writer
		wait (wk_on);
		forever kperiod(next_kind());
	end

	// ---------------------------------------------------------------- reader (RHX + T5.4)
	bit rd_on = 0, rd_pause = 0, rd_paused = 0, fs_level = 0;
	int frame_no = -1;		// Intan sample periods started
	int gword = 0;			// words since the start
	real t_fs = 0.0;		// time of the last frame_start edge
	int ld_f [$], ld_w [$]; bit ld_v [$];	// scheduled link_down changes (frame, word, value)
	int ld_g = -100;		// gword of the last link_down change
	event ev_dec;			// after each decision has been checked

	// checking state
	real w_vis = 3500.0;	// an entry this long before frame_start is seen by it
	bit checking = 0;
	int dbg = -1;			// +dbg=frame: print the decisions before it
	int lo = 0, hi = 0;		// the reader has seen entries 0 .. L-1, lo <= L <= hi
	bit hv = 0, rep = 0;	// held frame valid; it was served as a repeat
	int he = 0;				// held frame's entry
	bit skip_free = 1, after_pause = 0, wres = 0;
	longint e_rep = 0, s_lo = 0, s_hi = 0, sn_lo = 0, sn_hi = 0;
	bit cur_open = 0, cur_sent = 0, cur_conv0 = 0, cur_any0 = 0, cur_ld = 0;
	reg [31:0] cur_si;
	int ic = 0, ip = 0;		// entries certainly / possibly seen at t_fs
	int n_take = 0, n_reprt = 0, n_sent = 0, n_race = 0, n_race_new = 0, n_reads = 0;
	int n_ld_dec = 0;

	task automatic decide(input bit v, input [31:0] si, input [7:0] sf, input bit ld_fs);
		int Lp, L, Lc, nlo, nhi, oc, oe, moc, moe;
		longint sk, sk_a, skmin, skmax;
		bit found, m;
		// close the previous frame for frames_sentinel_served
		if (cur_open) begin
			if (!cur_sent) begin
				sn_lo += cur_conv0;
				sn_hi += (cur_any0 || cur_ld);
			end
		end
		while (ic < ev_t.size() && ev_t[ic] <= t_fs - w_vis) ic++;
		while (ip < ev_t.size() && ev_t[ip] < t_fs) ip++;
		Lc = ic; Lp = ip;
		if (Lc < Lp) n_race++;
		if (ld_fs) begin
			n_ld_dec++;
			if (v) err($sformatf("D2 frame %0d: rd_valid 1 at a frame_start with link_down", frame_no));
			s_hi += (lo < Lp) ? nvalid(lo, Lp) : 0;
			if (wres) skip_free = 1;
			hi = (Lp > hi) ? Lp : hi; hv = 0; rep = 0;
			n_sent++;
		end else begin
			found = 0; nlo = 1 << 30; nhi = -1; skmin = 1 << 30; skmax = 0; moc = 0; moe = 0;
			for (int Lprev = lo; Lprev <= hi; Lprev++)
				for (L = (Lprev > Lc ? Lprev : Lc); L <= Lp; L++) begin
					// oc: 0 sentinel, 1 take, 2 repeat
					if (L == Lprev) begin
						if (hv && !rep) begin oc = 2; oe = he; end else begin oc = 0; oe = 0; end
						sk = 0;
					end else if (!ev_v[L - 1]) begin
						oc = 0; oe = 0; sk = nvalid(Lprev, L);
					end else begin
						oc = 1; oe = L - 1; sk = nvalid(Lprev, L - 1);
					end
					m = (oc == 0 && !v) || (oc != 0 && v && ev_i[oe] === si && ev_f[oe] === sf);
					sk_a = sk;
					// after an RHX stop the sentinel is always allowed (D6), passing 0 .. all valid entries
					if (after_pause && !v && oc != 0) begin m = 1; oc = 0; sk_a = 0; sk = nvalid(Lprev, L); end
					if (m) begin
						found = 1; moc = oc; moe = oe;
						if (L < nlo) nlo = L;
						if (L > nhi) nhi = L;
						if (sk_a < skmin) skmin = sk_a;
						if (sk > skmax) skmax = sk;
					end
				end
			if (!found) begin
				err($sformatf("D2 frame %0d: outcome not allowed: rd_valid %0d index %0d flags %02h; seen entries %0d..%0d before, %0d..%0d now (newest %s %0d), held %s",
					frame_no, v, si, sf, lo, hi, Lc, Lp,
					(Lp > 0 && ev_v[Lp - 1]) ? "good" : "invalid", (Lp > 0) ? ev_i[Lp - 1] : 0,
					hv ? (rep ? "repeated" : "valid") : "none"));
				nlo = Lp; nhi = Lp; skmin = 0; skmax = FREE;
				moc = v ? 1 : 0; moe = he;
				if (v) for (int e = Lp - 1; e >= 0; e--) if (ev_i[e] === si) begin moe = e; break; end
			end
			if (Lc < Lp && moc == 1 && moe == Lp - 1) n_race_new++;
			lo = nlo; hi = nhi;
			if (moc == 2) begin
				rep = 1; e_rep = satl(e_rep + 1); n_reprt++;
			end else if (moc == 1) begin
				hv = 1; he = moe; rep = 0; n_take++;
				if (skip_free) begin skmax = FREE; skmin = 0; skip_free = 0; end
				wres = 0;
			end else begin
				hv = 0; rep = 0; n_sent++;
			end
			s_lo += skmin; s_hi = (skmax == FREE || s_hi >= FREE) ? FREE : s_hi + skmax;
		end
		if (dbg >= 0 && frame_no >= dbg - 12 && frame_no <= dbg)
			$display("%t dbg frame %0d: v %0d idx %0d ld %0d pause %0d | seen %0d..%0d, now %0d..%0d | skip bounds %0d..%0d, DUT %0d | rep %0d DUT %0d",
				$realtime, frame_no, v, si, ld_fs, after_pause, lo, hi, Lc, Lp, s_lo, s_hi, slip_skip, e_rep, slip_repeat);
		after_pause = 0;
		cur_open = 1; cur_sent = !v; cur_conv0 = 0; cur_any0 = 0; cur_ld = link_down;
		cur_si = si;
		if (!v) begin sn_lo++; sn_hi++; end
	endtask

	task automatic check_counters;
		if (slip_repeat !== satl(e_rep))
			err($sformatf("D4 frame %0d: slip_repeat %0d, expected %0d", frame_no, slip_repeat, satl(e_rep)));
		if (slip_skip < satl(s_lo) || slip_skip > satl(s_hi))
			err($sformatf("D4 frame %0d: slip_skip %0d, expected %0d..%0d", frame_no, slip_skip, satl(s_lo), satl(s_hi)));
		if (frames_sentinel_served < satl(sn_lo) || frames_sentinel_served > satl(sn_hi))
			err($sformatf("D4 frame %0d: frames_sentinel_served %0d, expected %0d..%0d",
				frame_no, frames_sentinel_served, satl(sn_lo), satl(sn_hi)));
		e_rep = slip_repeat; s_lo = slip_skip; s_hi = slip_skip;
		sn_lo = frames_sentinel_served; sn_hi = frames_sentinel_served;
	endtask

	// One read, sampled just before tx_load (2nd rising edge of the word after the CONVERT).
	task automatic sample(input int pc, input int w);
		bit v, req0, free;
		reg [31:0] si; reg [7:0] sf;
		v = rd_valid; si = serve_index; sf = serve_flags;
		if (!checking) return;
		if (pc == 0 && w == 1) begin decide(v, si, sf, link_down); -> ev_dec; end
		if (!cur_open) return;
		if (link_down) cur_ld = 1;
		if (!v) cur_any0 = 1;
		if (pc < 0) return;
		n_reads++;
		if (!v) cur_conv0 = 1;
		// D3
		req0 = link_down && (gword >= ld_g + 1);
		free = (link_down && gword < ld_g + 1) || (!link_down && cur_ld);
		if (req0 && v) err($sformatf("D3 frame %0d slot %0d: rd_valid 1 while link_down", frame_no, pc));
		if (!free && !link_down && v !== !cur_sent)
			err($sformatf("D3 frame %0d slot %0d: rd_valid %0d, decided %0d at frame_start", frame_no, pc, v, !cur_sent));
		// D1
		if (v) begin
			if (!good_idx.exists(si)) err($sformatf("D1 frame %0d slot %0d: serve_index %0d was never committed good", frame_no, pc, si));
			if (!cur_sent && si !== cur_si) err($sformatf("D1 frame %0d slot %0d: serve_index %0d changed from %0d mid frame", frame_no, pc, si, cur_si));
			if (rd_a0 !== wv(si, 4 * pc) || rd_b0 !== wv(si, 4 * pc + 1) || rd_a1 !== wv(si, 4 * pc + 2) || rd_b1 !== wv(si, 4 * pc + 3))
				err($sformatf("D1 frame %0d slot %0d: words %04h %04h %04h %04h, frame %0d has %04h %04h %04h %04h",
					frame_no, pc, rd_a0, rd_b0, rd_a1, rd_b1, si, wv(si, 4 * pc), wv(si, 4 * pc + 1), wv(si, 4 * pc + 2), wv(si, 4 * pc + 3)));
		end
		if (pc == 2) check_counters;
	endtask

	task automatic word(input int c, input int w, input int pc);
		// tick 0: CS falls (gap since the last word's tick 64): link_down changes here
		if (ld_f.size() > 0 && ld_f[0] == frame_no && ld_w[0] == w) begin
			link_down = ld_v[0]; ld_g = gword;
			void'(ld_f.pop_front()); void'(ld_w.pop_front()); void'(ld_v.pop_front());
		end
		for (int i = 0; i < 16; i++) begin
			#(2 * t_k - 0.5);
			if (i == 1) sample(pc, w);
			#0.5 sck = 1;
			if (i == 0) t_fs = $realtime;
			#(2 * t_k);
			sck = 0;
			if (i == 0 && !fs_level) frame_start = 0;
		end
		// 16th fall = the slave's data_rx load: T5.4's decode of this word
		rd_slot = (c >= 0) ? c : $urandom;
		frame_start = (c == 0);
		#(16 * t_k);
		gword++;
	endtask

	initial begin : reader
		int pc;
		pc = -1;
		wait (rd_on);
		forever begin
			if (rd_pause) begin
				rd_paused = 1; wait (!rd_pause); rd_paused = 0;
				after_pause = 1;
			end
			frame_no++;
			for (int w = 0; w < 35; w++) begin
				word(w < 32 ? w : -1, w, pc);
				pc = (w < 32) ? w : -1;
			end
		end
	end

	// ---------------------------------------------------------------- phases
	task automatic frames(input int n);
		repeat (n) @(ev_dec);
	endtask
	task automatic ld_episode(input int f0, input int w0, input int f1, input int w1);
		ld_f.push_back(frame_no + f0); ld_w.push_back(w0); ld_v.push_back(1);
		ld_f.push_back(frame_no + f1); ld_w.push_back(w1); ld_v.push_back(0);
	endtask
	task automatic pause(input int periods);
		rd_pause = 1; wait (rd_paused);
		#(periods * 40000.0);
		rd_pause = 0;
	endtask
	int r0_rep, r0_skip, r0_sent;
	task automatic mark;
		r0_rep = slip_repeat; r0_skip = slip_skip; r0_sent = frames_sentinel_served;
	endtask
	task automatic report(input string s);
		$display("%t %-34s repeat +%0d  skip +%0d  sentinel frames +%0d", $realtime, s,
			slip_repeat - r0_rep, slip_skip - r0_skip, frames_sentinel_served - r0_sent);
	endtask

	integer seed, soak, k;
	int stops [3] = '{1, 2, 5};
	int pauses [3] = '{3, 64, 256};
	// planned pairs (28/30, 28/25), and Intan at 20k by mistake
	int rates_k [4] = '{28000, 28000, 25000, 28000};
	int rates_i [4] = '{30000, 25000, 20000, 20000};
	int rates_n [4] = '{500, 800, 600, 600};
	initial begin
		if (!$value$plusargs("seed=%d", seed)) seed = 1;
		if (!$value$plusargs("soak=%d", soak)) soak = 1500;
		if ($test$plusargs("fs_level")) fs_level = 1;
		if (!$value$plusargs("wvis=%f", w_vis)) w_vis = 3500.0;
		if (!$value$plusargs("dbg=%d", dbg)) dbg = -1;
		k = $urandom(seed);
		$display("triple_buffer_tb: seed %0d, soak %0d, fs_level %0d, W_VIS %0.0f ns", seed, soak, fs_level, w_vis);

		// P0 resets released in random order; RHX streams before Kuntur sends
		#($urandom_range(100, 3000));
		if ($urandom_range(0, 1)) begin rstb_w = 1; #($urandom_range(10, 3000)); rstb_s = 1; end
		else begin rstb_s = 1; #($urandom_range(10, 3000)); rstb_w = 1; end
		checking = 1; sn_hi = 1;	// the words before the first frame_start may count as one
		rd_on = 1;
		frames(5);
		mark;
		$display("%t P0 no SAMPLE yet: sentinel only", $realtime);
		if (n_take != 0) err("P0 a frame served before any SAMPLE");

		// P1 same rate, commit mid frame: every frame taken, no slip
		@(ev_dec); #(31400.0); wk_on = 1;
		frames(3); mark;
		frames(30);
		report("P1 same rate");
		if (slip_repeat != r0_rep || slip_skip != r0_skip || frames_sentinel_served != r0_sent)
			err("P1 slip or sentinel at the same rate");

		// P2 slow drift through every phase of commit against CONVERT(0) (Q1 race)
		mark; set_ppm(3000.0); frames(800); report("P2 Kuntur +3000 ppm");
		if (slip_skip == r0_skip) err("P2 no skip at +3000 ppm");
		mark; set_ppm(-3000.0); frames(800); report("P2 Kuntur -3000 ppm");
		if (slip_repeat == r0_rep) err("P2 no repeat at -3000 ppm");
		// P3 fast drift
		mark; set_ppm(20000.0); frames(250); report("P3 Kuntur +2 %");
		mark; set_ppm(-20000.0); frames(250); report("P3 Kuntur -2 %");

		// P4 failed and lost frames (R2, Q3, Q4)
		set_ppm(700.0);
		for (int kk = BADCRC; kk <= SILENT; kk++) begin
			mark; frames(4); wk_force = kk; frames(6);
			report($sformatf("P4 one %s", kind_t'(kk)));
		end
		mark; p_fail = 25; frames(600); p_fail = 0; report("P4 25 % failed, +700 ppm");

		// P5 Kuntur stops (Q2: one repeat, then the sentinel)
		set_ppm(0.0);
		foreach (stops[i]) begin
			mark; frames(4); wk_stop = stops[i]; frames(12);
			report($sformatf("P5 Kuntur stops %0d period(s)", stops[i]));
		end

		// P6 link_down (R5), mid frame
		mark; ld_episode(2, 10, 5, 20); frames(10); report("P6 link_down, Kuntur sending");
		mark; frames(2); wk_hold = 1; ld_episode(1, 4, 6, 29); frames(8); wk_hold = 0; frames(6);
		report("P6 link_down, Kuntur stopped");
		mark; frames(2); wk_hold = 1; ld_episode(1, 15, 8, 12); frames(3);
		@(negedge wclk); rstb_w = 0; lo = ev_t.size(); hi = lo; hv = 0; wres = 1; skip_free = 1;
		#($urandom_range(200, 5000)); rstb_w = 1;
		frames(6); wk_hold = 0; frames(8); report("P6 link_down with cdr_rstb");

		// P7 RHX stops while Kuntur runs (D6)
		set_ppm(1000.0);
		foreach (pauses[i]) begin
			mark; frames(3); pause(pauses[i]); frames(6);
			report($sformatf("P7 RHX stops %0d periods", pauses[i]));
		end

		// P8 rst_n_emu while RHX is stopped
		mark; frames(3);
		rd_pause = 1; wait (rd_paused);
		#(3000.0); rstb_s = 0; #(500.0); rstb_s = 1;
		lo = 0; hi = ev_t.size(); hv = 0; rep = 0; skip_free = 1; cur_open = 0;
		e_rep = 0; s_lo = 0; s_hi = 0; sn_lo = 0; sn_hi = 1;
		#(80000.0); rd_pause = 0;
		frames(8);
		$display("%t P8 rst_n_emu: repeat %0d skip %0d sentinel frames %0d", $realtime, slip_repeat, slip_skip, frames_sentinel_served);

		// P10 other rate pairs (§9.5a): Intan slower than Kuntur keeps a bank in the reader for
		// longer than the writer takes to come back to it, so here a wrong bank choice (Q1) shows.
		foreach (rates_k[i]) begin
			mark; set_ppm(0.0); set_rates(rates_k[i], rates_i[i]); frames(rates_n[i]);
			report($sformatf("P10 Kuntur %0dk, Intan %0dk", rates_k[i] / 1000, rates_i[i] / 1000));
			if (rates_k[i] > rates_i[i] && slip_skip == r0_skip) err("P10 no skip with Intan slower");
			if (rates_k[i] < rates_i[i] && slip_repeat == r0_rep) err("P10 no repeat with Intan faster");
		end
		mark; set_rates(28000, 25000); p_fail = 20; frames(400); p_fail = 0;
		report("P10 Kuntur 28k, Intan 25k, 20 % failed");
		set_rates(25000, 25000);

`ifdef TB_SAT
		// P9 counters saturate
		@(negedge sck);
		dut.slip_repeat = MAXC - 1; dut.slip_skip = MAXC - 1; dut.frames_sentinel_served = MAXC - 1;
		e_rep = MAXC - 1; s_lo = MAXC - 1; s_hi = MAXC - 1; sn_lo = MAXC - 1; sn_hi = MAXC - 1;
		set_ppm(-20000.0); frames(120); set_ppm(20000.0); frames(120);
		set_ppm(0.0); wk_stop = 3; frames(10);
		if (slip_repeat !== MAXC || slip_skip !== MAXC || frames_sentinel_served !== MAXC)
			err($sformatf("P9 counters did not saturate: %h %h %h", slip_repeat, slip_skip, frames_sentinel_served));
		else $display("%t P9 counters saturate", $realtime);
`endif

		// Soak: random rate, failures, stops, link_down, RHX stops
		mark;
		for (int f = 0; f < soak; ) begin
			int a, n;
			a = $urandom_range(0, 10); n = $urandom_range(20, 120);
			case (a)
				0, 1, 2, 3: begin set_ppm($urandom_range(0, 8000) - 4000.0); p_fail = $urandom_range(0, 20); frames(n); end
				4: begin set_ppm($urandom_range(0, 1) ? 70.0 : -70.0); frames(n); end
				5: begin wk_stop = $urandom_range(1, 4); frames(n); end
				6: begin ld_episode(1, $urandom_range(4, 29), $urandom_range(2, 6), $urandom_range(4, 29)); frames(n); end
				7: begin pause($urandom_range(1, 30)); frames(n); end
				8: begin set_rates(28000, $urandom_range(0, 1) ? 30000 : 25000); p_fail = $urandom_range(0, 20);
					frames(n); set_rates(25000, 25000); end
				default: begin set_ppm($urandom_range(0, 40000) - 20000.0); frames(n); end
			endcase
			f += n;
		end
		p_fail = 0; set_ppm(0.0); frames(10);
		report("soak");

		$display("frames: %0d taken, %0d repeats, %0d sentinel (%0d with link_down); %0d with an entry in the W_VIS window (%0d of them took it); %0d reads checked",
			n_take, n_reprt, n_sent, n_ld_dec, n_race, n_race_new, n_reads);
		$display("Kuntur periods: good %0d, bad CRC %0d, bad header %0d, cut %0d, partial %0d, silent %0d, stopped %0d",
			n_per_kind[GOOD], n_per_kind[BADCRC], n_per_kind[HDRBAD], n_per_kind[CUT], n_per_kind[PARTIAL], n_per_kind[SILENT], n_per_kind[STOP]);
		$display("DUT: slip_repeat %0d, slip_skip %0d, frames_sentinel_served %0d", slip_repeat, slip_skip, frames_sentinel_served);
		err_summary;
		if (errors == 0) $display("triple_buffer_tb: PASS");
		else $display("triple_buffer_tb: FAIL, %0d errors", errors);
		$finish;
	end

	initial begin #2s; err("timeout"); err_summary; $display("triple_buffer_tb: FAIL"); $finish; end
endmodule
