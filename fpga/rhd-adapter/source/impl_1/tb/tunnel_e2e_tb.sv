//testbench
//
//	tunnel_e2e_tb: end-to-end testbench for the tunnel, lvds-tunnel.md §9.6
//	(T5). Both FPGA top levels, real RTL, pin to pin:
//
//	  rhd2164_bfm x2 ─ spi1 ─► kuntur_fpga ─ tun_data/tun_clk ─► link ─► intan_rec_adapter ─ spi0 ─► intan_master_bfm
//	  (answers name chip,                    (delay, jitter,           (pll0/IDDRX4 models,      (Rhythm: 35 words per
//	   half, channel, frame)                  cable in/out,             adapter_prims_sim.v)      period, §9.1b lists,
//	                                          bit-error windows)                                  cable-delay scan)
//
//	Two stages, one file. The stage is chosen at run time (+stageb) and
//	must match Kuntur's build variant (compile define KUNTUR_VARIANT):
//
//	Stage A (Kuntur variant 3, today's adapter: cdr sweep on the K4.2
//	pattern). Checks the harness and what the adapter does today:
//	  A1  power order (Kuntur first); the adapter's PLL locks, cdr sweeps,
//	      ends valid; eye width >= 84 (G5) and its centre against the
//	      prediction from the link delays; no PHASESTEP pulse violation.
//	  A2  the host's detection scan (findConnectedChips, §9.1): cable-delay
//	      settings 0..7, one register-config run each (bank 0, 37 periods);
//	      a setting is good when every checked answer of that run is right.
//	      Good settings must exist; the middle of the longest good run is
//	      used from then on.
//	  A3  streaming (bank 1): every answer the emulator gives today is
//	      checked against §9.1 / §9.3: ROM reads 40-44, 48-50, 59 (A 0x35,
//	      B 0x3A), 61-63, WRITE acks {FF, D}, CALIBRATE 0, on both chips'
//	      MISO, in word w + 2 (§9.2). READ(0..21) and CONVERT are counted,
//	      not judged (no T5.4 yet).
//	  A4  cable out 200 us, then back: PLL lock falls, the reset chain
//	      reruns, cdr sweeps and ends valid again; the emulator still
//	      answers right (it must not be reset, system-startup.md R-C8).
//
//	Stage B (Kuntur variant 2: frames; adapter with T5). Everything of
//	stage A except the cdr parts, plus, on every CONVERT(c) answer, c < 32,
//	both chips, A and B (decoded on the pins, never from DUT internals):
//	  B1  value: answer ^ 0x8000 (§9.2 XOR) is the sentinel 0x8000 (pins
//	      0x0000) or {chip, half, channel c, frame tag} as rhd2164_bfm
//	      makes it: right chip on the right MISO, A = c, B = c + 32.
//	  B2  not torn: one tag for all 128 words of a sweep (CONVERT(0) to
//	      CONVERT(31)); a sweep may switch to the sentinel part way (R5),
//	      never back.
//	  B3  fresh: the tag served from a CONVERT(0) is the newest SAMPLE that
//	      had left Kuntur by then, or the one before if the newest left
//	      less than LAT_US before (the receive latency). Kuntur's frames are
//	      decoded on its own wclk/tun_data_8b (as K8 does), so the tb knows
//	      each SAMPLE's tag and when its last CRC byte left.
//	  B4  slips (§9.5a): tag steps of 0 (repeat) and k > 1 (skip k - 1)
//	      counted, reported; with +ippm the expected count is printed.
//	  B5  lost frames (R2): a SAMPLE hit by an injected bit error is never
//	      served, and the sweep that would have used it is the sentinel.
//	  B6  link down (R1, R5, §7.1): from 6 us after TUN_CLK stops (cable
//	      out, Kuntur reset) every word is the sentinel, mid sweep included,
//	      until the clock is back; and no sentinel at all when nothing was
//	      done to the link for RECOVER_US.
//	  B7  READ(0..21) (R4): 0 before the first deliverable CONFIG, then the
//	      values in the last deliverable CONFIG (decoded on Kuntur's wire,
//	      so a wrong CONFIG and a wrong replication are told apart). A
//	      CONFIG is deliverable if it left Kuntur while the adapter's T5.1
//	      was locked (adp.frame_rx_0.locked): one sent before the lock never
//	      reaches the adapter, and the next comes with the 40 ms refresh.
//	      (Fixed 2026-11-26: B7 expected the start-up CONFIG, sent 0.5 ms
//	      before the adapter locked.)
//	  Scenario: power-up; scan; stream; 1 bit-error frame; 8 consecutive
//	  bit-error frames (unlock and re-hunt at the same phase, R1); cable
//	  out 200 us; Kuntur rstb 100 us; stream to the end.
//
//	Plusargs
//	  +stageb           stage B (else A)
//	  +ippm=<ppm>       Intan clock offset against Kuntur (default +2000,
//	                    i.e. Intan fast: repeats; -2000 for skips). Large
//	                    on purpose: real crystals give ~1 slip/s, a few ms
//	                    of simulation would see none.
//	  +d_data= +d_clk= +ins= (ps, link and PLL insertion delays)
//	  +jitter=<ps> +win=<ps> (data-edge jitter, IDDRX4 window)
//	  +keep_phase=<0|1> (receive PLL keeps its stepped phase across a lock
//	                    loss: unknown on the real part, run both)
//	  +seed=<n>
//	  +periods=<n>      streaming periods per step (default 400)
//
//	Run (QuestaSim from Radiant; the two tops `include the same common
//	modules, so each goes into its own library; instances resolve in their
//	parent's library first). From a scratch directory, with
//	K = fpga/headstage/source/impl_1, A = fpga/rhd-adapter/source/impl_1:
//	  vlib kun; vlib adp; vlib work
//	  vlog -sv -work kun +incdir+$K +incdir+$K/afe/rhd2164 +incdir+$K/app \
//	       +define+KUNTUR_VARIANT=4\'d3 $K/tb/kuntur_prims_sim.v $K/kuntur_fpga.v
//	  vlog -sv -work adp +incdir+$A $A/tb/adapter_prims_sim.v $A/intan_rec_adapter.v
//	  vlog -sv -work work $K/tb/rhd2164_bfm.v $A/tb/intan_master_bfm.v $A/tb/tunnel_e2e_tb.sv
//	  vsim -c -L kun -L adp tunnel_e2e_tb -do "run -all; quit -f"
//	(4'd2 and +stageb for stage B.) run_e2e.sh does all of it.

`timescale 1ns/1ps

module tunnel_e2e_tb;

	// ---------------------------------------------------------------- setup
	localparam real TK_NOM = 1.0e3 / 37.95;		// Kuntur clk, ns (25,000 x 33 x 46)
	localparam real TH12 = 41.667;				// adapter clkin 12 MHz, half period
	localparam real LAT_US = 3.0;				// B3: receive latency allowance
	localparam real RECOVER_US = 3000.0;		// B6: sentinel allowed this long after an event
	localparam integer NDLY = 8;				// A2: cable-delay settings scanned

	reg stageb = 1'b0;
	reg judge = 1'b0;					// set after the scan: answers are judged from then on
	real ippm, d_data, d_clk, ins, jit, win;
	integer keep_phase, seed, periods;
	integer fails = 0, checks = 0;

	task automatic fail(input string msg);
		fails = fails + 1;
		if (fails <= 40) $display("FAIL  t=%0.3f us: %s", $realtime / 1000.0, msg);
	endtask
	task automatic pass(input string msg);
		checks = checks + 1;
		$display("pass  t=%0.3f us: %s", $realtime / 1000.0, msg);
	endtask

	// ---------------------------------------------------------------- clocks
	reg kclk = 1'b0, clk12 = 1'b0;
	reg kclk_run = 1'b1;
	always #(TK_NOM / 2.0) if (kclk_run) kclk = ~kclk;
	always #(TH12) clk12 = ~clk12;

	// ---------------------------------------------------------------- Kuntur
	reg k_rstb = 1'b0;
	wire spi1_csb, spi1_sck, spi1_mosi, spi1_miso0, spi1_miso1;
	wire tun_data, tun_clk;

	kuntur_fpga kun (
		.rstb		(k_rstb),
		.clkin		(kclk),
		.spi0_csb	(1'b1),
		.spi0_sck	(1'b0),
		.spi0_mosi	(1'b0),
		.spi0_miso	(),
		.spi1_miso1	(spi1_miso1),
		.spi1_miso0	(spi1_miso0),
		.spi1_csb	(spi1_csb),
		.spi1_sck	(spi1_sck),
		.spi1_mosi	(spi1_mosi),
		.spi2_miso0	(1'b0),
		.spi2_csb	(),
		.spi2_sck	(),
		.spi2_mosi0	(),
		.tun_data	(tun_data),
		.tun_clk	(tun_clk),
		.cmd_is_00	()
	);

	rhd2164_bfm #(.CHIP(0)) chip0 (.csb(spi1_csb), .sck(spi1_sck), .mosi(spi1_mosi), .miso(spi1_miso0));
	rhd2164_bfm #(.CHIP(1)) chip1 (.csb(spi1_csb), .sck(spi1_sck), .mosi(spi1_mosi), .miso(spi1_miso1));

	// ---------------------------------------------------------------- link
	// Fixed delays, uniform jitter on data transitions, cable in/out (both
	// pairs low while out), and an error window that inverts the data.
	reg cable = 1'b1, inj = 1'b0;
	reg data_line = 1'b0, clk_line = 1'b0;
	real jd;
	always @(tun_data or inj or cable) begin
		jd = ($urandom % 2001) / 1000.0 - 1.0;
		data_line <= #((d_data + jit * jd) / 1000.0) cable & (tun_data ^ inj);
	end
	always @(tun_clk or cable) clk_line <= #(d_clk / 1000.0) cable & tun_clk;

	// ---------------------------------------------------------------- adapter
	reg a_rstb = 1'b0;
	wire h_csb, h_sck, h_mosi, h_miso0, h_miso1, con_tx;
	wire [4:0] leds;

	intan_rec_adapter adp (
		.rstb		(a_rstb),
		.clkin		(clk12),
		.kuntur_data	(data_line),
		.kuntur_clk	(clk_line),
		.spi0_csb	(h_csb),
		.spi0_sck	(h_sck),
		.spi0_mosi	(h_mosi),
		.spi0_miso1	(h_miso1),
		.spi0_miso0	(h_miso0),
		.con_tx		(con_tx),
		.led_n_4	(leds[4]),
		.led_n_3	(leds[3]),
		.led_n_2	(leds[2]),
		.led_n_1	(leds[1]),
		.led_n_0	(leds[0])
	);
	// Stage A only: the bench build's sweep, shortened (cdr_tb does the same)
	defparam adp.cdr0.checker0.NB_ERRORS = 16'd20;

	// ---------------------------------------------------------------- host
	intan_master_bfm #(.FS(25000.0)) host (
		.csb	(h_csb),
		.sck	(h_sck),
		.mosi	(h_mosi),
		.miso0	(h_miso0),
		.miso1	(h_miso1)
	);

	// ---------------------------------------------------------------- Kuntur's wire, decoded (B3, B5, B7)
	// On Kuntur's own wclk and tun_data_8b, as kuntur_tb's K8 receiver.
	// Per SAMPLE: its tag (slot 0, chip 0 A word, bits 8:0) and the time
	// its last CRC byte left. Per CONFIG: its 22 bytes and that time.
	localparam integer NS = 256;
	integer ns = 0;							// SAMPLEs seen
	reg [8:0] s_tag [0:NS-1];
	realtime s_end [0:NS-1];
	integer nc = 0;							// CONFIGs seen
	reg [7:0] cfg_now [0:21], cfg_rx [0:21], cfg_dl [0:21];
	integer ncd = 0;						// deliverable CONFIGs seen (B7)
	realtime cfg_end = 0.0;
	event sample_payload;					// a SAMPLE's payload starts (B5)
	reg [31:0] ksh = 32'd0;
	integer kst = 0, kn = 0, klen = 0;
	reg [7:0] ktype, kb0;
	reg [8:0] ktag;
	wire [7:0] kbyte = kun.tun_data_8b;
	always @(posedge kun.wclk) if (stageb) begin
		case (kst)
			0: begin
				ksh = {ksh[23:0], kbyte};
				if (ksh == 32'hA55AA55A) begin kst = 1; kn = 0; end
			end
			1: begin						// header, 8 bytes
				if (kn == 0) ktype = kbyte;
				if (kn == 6) klen = kbyte;
				if (kn == 7) begin
					klen = klen + 256 * kbyte;
					kst = 2; kn = -1;
					if (ktype == 8'h01) -> sample_payload;
				end
				kn = kn + 1;
			end
			2: begin						// payload
				if (ktype == 8'h01 && kn == 0) kb0 = kbyte;
				if (ktype == 8'h01 && kn == 1) ktag = {kbyte[0], kb0};
				if (ktype == 8'h02 && kn < 22) cfg_rx[kn] = kbyte;
				kn = kn + 1;
				if (kn == klen) begin kst = 3; kn = 0; end
			end
			3: begin						// CRC, 4 bytes (K8 checks it; trusted here)
				kn = kn + 1;
				if (kn == 4) begin
					if (ktype == 8'h01) begin
						s_tag[ns % NS] = ktag;
						s_end[ns % NS] = $realtime;
						ns = ns + 1;
					end
					else if (ktype == 8'h02) begin
						for (int r = 0; r < 22; r++) cfg_now[r] = cfg_rx[r];
						nc = nc + 1;
						if (adp.frame_rx_0.locked === 1'b1) begin
							for (int r = 0; r < 22; r++) cfg_dl[r] = cfg_rx[r];
							cfg_end = $realtime;
							ncd = ncd + 1;
						end
					end
					kst = 0; ksh = 32'd0;
				end
			end
		endcase
		if (!k_rstb) begin kst = 0; ksh = 32'd0; end
	end

	// newest SAMPLE whose last byte left Kuntur at or before t; -1 if none
	function automatic integer newest_before(input realtime t);
		integer i;
		newest_before = -1;
		for (i = ns - 1; i >= 0 && i >= ns - NS; i = i - 1)
			if (s_end[i % NS] <= t) begin newest_before = i; break; end
	endfunction

	// ---------------------------------------------------------------- link events (B6)
	// Times at which TUN_CLK stopped and restarted (cable out, Kuntur
	// reset), and at which errors were injected.
	realtime ev_last = -1.0e12;			// end of the latest event of any kind
	realtime down_from = 1.0e12, down_to = -1.0e12;		// current/last TUN_CLK stop
	task automatic mark_event; ev_last = $realtime; endtask

	// ---------------------------------------------------------------- answer checker
	// Pipeline: the answer to word w is in word w + 2 (§9.2).
	reg [15:0] c_hist [0:3];
	realtime t_hist [0:3];
	integer n_rom = 0, n_rom_bad = 0, n_wr = 0, n_wr_bad = 0, n_reg = 0, n_reg_bad = 0;
	integer n_reg_cfg = 0;						// B7: READs judged against a delivered CONFIG
	integer n_conv = 0, n_conv_bad = 0, n_const = 0;
	integer run_bad = 0;					// A2: wrong answers in the current run

	function automatic [7:0] rom(input [5:0] r);
		case (r)
			40: rom = 8'h49; 41: rom = 8'h4E; 42: rom = 8'h54; 43: rom = 8'h41; 44: rom = 8'h4E;
			48: rom = 8'h52; 49: rom = 8'h48; 50: rom = 8'h44;
			59: rom = 8'h35; 61: rom = 8'h01; 62: rom = 8'd64; 63: rom = 8'h04;
			default: rom = 8'h00;
		endcase
	endfunction
	function automatic bit rom_checked(input [5:0] r);
		rom_checked = (r >= 40 && r <= 44) || (r >= 48 && r <= 50) || r == 59 || (r >= 61 && r <= 63);
	endfunction

	// sweep state (B2-B4)
	integer sw_tag = -1;					// tag of the sweep being answered; -2 sentinel; -1 none
	bit sw_sent = 0;						// switched to the sentinel in this sweep
	bit sw_bad = 0;
	realtime sw_t0;
	integer last_tag = -1;					// tag of the last valid sweep
	bit sent_since = 0;						// a sentinel sweep since last_tag
	integer n_sweeps = 0, n_sent_sweeps = 0, n_repeat = 0, n_skip = 0, n_gap = 0;
	integer n_stale = 0, n_torn = 0, n_sent_bad = 0, n_sent_missing = 0;
	integer lost_tag [0:15];				// B5: tags hit by injected errors
	realtime lost_t [0:15];					// when; the 9-bit tag comes back after 512 SAMPLEs (~20 ms)
	integer n_lost = 0, n_lost_served = 0;
	localparam real LOST_WIN_US = 5000.0;	// B5 looks this long after the hit (<< one tag wrap)

	function automatic bit lost(input integer tag);
		lost = 0;
		for (int i = 0; i < n_lost; i++)
			if (lost_tag[i] == tag && $realtime - lost_t[i] < LOST_WIN_US * 1000.0) lost = 1;
	endfunction

	task automatic check_conv(input [5:0] c, input realtime tc);
		reg [15:0] w [0:3];
		reg [15:0] v;
		integer k, tag;
		bit sent, any_tag, all_sent;
		begin
			w[0] = host.a0; w[1] = host.b0; w[2] = host.a1; w[3] = host.b1;
			// The sentinel comes on all four words at once (T5.4: one rd_valid).
			// Decide it per command: chip 1's A word of CONVERT(0) at tag 0 is
			// {1, 0, 00000, 0} = 8000 = the sentinel value, and is data.
			all_sent = 1;
			for (k = 0; k < 4; k = k + 1) if ((w[k] ^ 16'h8000) != 16'h8000) all_sent = 0;
			n_conv = n_conv + 1;
			if (c == 0) begin						// a new sweep
				sw_tag = -1; sw_sent = 0; sw_bad = 0; sw_t0 = tc;
			end
			for (k = 0; k < 4; k = k + 1) begin
				v = w[k] ^ 16'h8000;
				sent = all_sent;
				// B6: TUN_CLK stopped for 6 us before this command: sentinel
				if (tc > down_from + 6000.0 && tc < down_to && !sent) begin
					n_sent_missing = n_sent_missing + 1;
					if (n_sent_missing <= 5)
						fail($sformatf("B6: CONVERT(%0d) word %0d = %h while TUN_CLK is stopped", c, k, w[k]));
				end
				if (sent) begin
					if (sw_tag >= 0) sw_sent = 1;			// R5: valid -> sentinel, mid sweep
					else sw_tag = -2;
					continue;
				end
				// B1: names itself
				if (v[15] !== k[1] || v[14] !== k[0] || v[13:9] !== c[4:0]) begin
					n_conv_bad = n_conv_bad + 1; sw_bad = 1;
					if (n_conv_bad <= 5)
						fail($sformatf("B1: CONVERT(%0d) MISO%0d %s = %h: not chip %0d half %0d channel %0d",
							c, k / 2, (k % 2) ? "B" : "A", w[k], k / 2, k % 2, c));
					continue;
				end
				tag = v[8:0];
				// B2: not torn, and never back from the sentinel
				if (sw_sent || sw_tag == -2 || (sw_tag >= 0 && tag != sw_tag)) begin
					n_torn = n_torn + 1; sw_bad = 1;
					if (n_torn <= 5)
						fail($sformatf("B2: torn sweep: CONVERT(%0d) word %0d tag %0d, sweep %0d%s",
							c, k, tag, sw_tag, sw_sent ? " (after the sentinel)" : ""));
				end
				if (sw_tag == -1) sw_tag = tag;
			end
			if (c == 31) end_sweep();
		end
	endtask

	task automatic end_sweep;
		integer i_new, i_old, d;
		begin
			n_sweeps = n_sweeps + 1;
			if (sw_tag == -2 || sw_sent) begin
				n_sent_sweeps = n_sent_sweeps + 1;
				sent_since = 1;
				// B6: no sentinel on an untouched, locked link
				if (stageb && sw_t0 > ev_last + RECOVER_US * 1000.0 && down_to < 1.0e11) begin	// not during an event
					n_sent_bad = n_sent_bad + 1;
					if (n_sent_bad <= 5) fail("B6: sentinel sweep on an untouched link");
				end
			end
			if (sw_tag >= 0) begin
				// B5
				if (lost(sw_tag)) begin
					n_lost_served = n_lost_served + 1;
					fail($sformatf("B5: tag %0d served, but that SAMPLE had an injected error", sw_tag));
				end
				// B3: fresh
				i_new = newest_before(sw_t0);
				i_old = newest_before(sw_t0 - LAT_US * 1000.0);
				if (!((i_new >= 0 && sw_tag == s_tag[i_new % NS]) ||
				      (i_old >= 0 && sw_tag == s_tag[i_old % NS]))) begin
					n_stale = n_stale + 1;
					if (n_stale <= 5) begin
						if (i_new < 0)
							fail($sformatf("B3: sweep at %0.3f us served tag %0d before any SAMPLE left Kuntur",
								sw_t0 / 1000.0, sw_tag));
						else
							fail($sformatf("B3: sweep at %0.3f us served tag %0d; newest left Kuntur: tag %0d, %0.1f us before",
								sw_t0 / 1000.0, sw_tag, s_tag[i_new % NS], (sw_t0 - s_end[i_new % NS]) / 1000.0));
					end
				end
				// B4: slips
				if (last_tag >= 0) begin
					d = (sw_tag - last_tag + 512) % 512;
					if (d == 0) n_repeat = n_repeat + 1;
					else if (d > 1 && !sent_since) n_skip = n_skip + d - 1;
					else if (d > 1) n_gap = n_gap + 1;
				end
				last_tag = sw_tag;
				sent_since = 0;
			end
		end
	endtask

	always @(host.word_done) begin : answer_check
		reg [15:0] c;
		realtime tc;
		reg [15:0] exp_a, exp_b;
		integer w, r;
		bit ok;
		w = host.word_no;
		c_hist[w % 4] = host.done_cmd;
		t_hist[w % 4] = $realtime;
		if (w >= 2) begin
			c = c_hist[(w - 2) % 4];
			tc = t_hist[(w - 2) % 4];
			r = c[13:8];
			case (c[15:14])
				2'b11: begin									// READ
					if (rom_checked(r)) begin
						exp_a = {8'h00, rom(r)};
						exp_b = (r == 59) ? 16'h003A : exp_a;
						ok = host.a0 == exp_a && host.b0 == exp_b && host.a1 == exp_a && host.b1 == exp_b;
						n_rom = n_rom + 1;
						if (!ok) begin
							n_rom_bad = n_rom_bad + 1; run_bad = run_bad + 1;
							if (n_rom_bad <= 5 && host.bank)
								fail($sformatf("READ(%0d): %h %h / %h %h, expected %h %h", r,
									host.a0, host.b0, host.a1, host.b1, exp_a, exp_b));
						end
					end
					else if (r <= 21) begin
						n_reg = n_reg + 1;
						// B7: 0 before any CONFIG has left Kuntur; the last one's
						// value once it has had LAT_US to arrive; not judged in
						// between, nor while the link recovers from an event
						if (stageb && judge && (ncd == 0 || tc > cfg_end + LAT_US * 1000.0)) begin
							exp_a = (ncd == 0) ? 16'h0000 : {8'h00, cfg_dl[r]};
							if (ncd > 0) n_reg_cfg = n_reg_cfg + 1;
							begin
								ok = host.a0 == exp_a && host.b0 == exp_a && host.a1 == exp_a && host.b1 == exp_a;
								if (!ok && tc > ev_last + RECOVER_US * 1000.0) begin
									n_reg_bad = n_reg_bad + 1;
									if (n_reg_bad <= 5)
										fail($sformatf("B7: READ(%0d) = %h %h / %h %h, expected %h (%0d CONFIGs seen, %0d deliverable)",
											r, host.a0, host.b0, host.a1, host.b1, exp_a, nc, ncd));
								end
							end
						end
					end
				end
				2'b10: begin									// WRITE
					exp_a = {8'hFF, c[7:0]};
					ok = host.a0 == exp_a && host.b0 == exp_a && host.a1 == exp_a && host.b1 == exp_a;
					n_wr = n_wr + 1;
					if (!ok) begin
						n_wr_bad = n_wr_bad + 1; run_bad = run_bad + 1;
						if (n_wr_bad <= 5 && host.bank)
							fail($sformatf("WRITE(%0d, %h): %h %h / %h %h, expected %h", r, c[7:0],
								host.a0, host.b0, host.a1, host.b1, exp_a));
					end
				end
				2'b01: begin									// CALIBRATE / CLEAR
					ok = host.a0 == 16'h0 && host.b0 == 16'h0 && host.a1 == 16'h0 && host.b1 == 16'h0;
					n_rom = n_rom + 1;
					if (!ok) begin
						n_rom_bad = n_rom_bad + 1; run_bad = run_bad + 1;
					end
				end
				2'b00: begin									// CONVERT
					if (r < 32 && stageb && judge) check_conv(r, tc);
					else n_const = n_const + 1;
				end
			endcase
		end
	end

	// ---------------------------------------------------------------- scenario steps
	task automatic stream(input integer n);
		host.run_periods(n);
	endtask

	// inject a bit-error window into the next n SAMPLEs' payloads (B5)
	task automatic inject_frames(input integer n);
		integer i;
		for (i = 0; i < n; i = i + 1) begin
			@(sample_payload);
			#2000.0;					// well inside the 256-byte payload (~27 us)
			inj = 1'b1;
			#40.0;
			inj = 1'b0;
			mark_event();
			// the tag of the SAMPLE just hit: the payload's own, decoded at its end
			@(posedge kun.wclk iff kst == 3);
			if (n_lost < 16) begin lost_tag[n_lost] = ktag; lost_t[n_lost] = $realtime; n_lost = n_lost + 1; end
		end
	endtask

	task automatic cable_out(input real us);
		down_from = $realtime;
		down_to = 1.0e12;
		cable = 1'b0;
		#(us * 1000.0);
		cable = 1'b1;
		down_to = $realtime;
		mark_event();
	endtask

	task automatic kuntur_reset(input real us);
		down_from = $realtime + 1000.0;			// TUN_CLK stops once Kuntur's reset reaches gddr_sync
		down_to = 1.0e12;
		k_rstb = 1'b0;
		#(us * 1000.0);
		k_rstb = 1'b1;
		down_to = $realtime;
		mark_event();
	endtask

	// A1/A4: cdr ends valid within a timeout
	task automatic wait_cdr(input string what, input real ms);
		realtime t0;
		t0 = $realtime;
		while (adp.cdr_valid !== 1'b1 && $realtime - t0 < ms * 1.0e6)
			#1000.0;
		if (adp.cdr_valid === 1'b1)
			pass($sformatf("%s: cdr valid after %0.2f ms, eye %0d/%0d (G5 needs width >= 84)",
				what, ($realtime - t0) / 1.0e6, adp.eye_center, adp.eye_width));
		else
			fail($sformatf("%s: cdr not valid after %0.1f ms (rdy %b)", what, ms, adp.cdr_rdy));
		if (adp.cdr_valid === 1'b1 && adp.eye_width < 84)
			fail($sformatf("%s: eye width %0d < 84 (G5)", what, adp.eye_width));
	endtask

	// A2: the host's cable-delay scan
	integer best_dly = 1;
	task automatic delay_scan;
		integer d, run_len, best_len, best_start, start;
		bit good [0:NDLY-1];
		begin
			#100000.0;						// the emulator leaves reset
			host.bank = 1'b0;
			host.build_lists();
			for (d = 0; d < NDLY; d = d + 1) begin
				host.delay = d;
				host.period_no = 0;
				run_bad = 0;
				host.run_periods(37);
				good[d] = (run_bad == 0);
				$display("      delay %0d: %0d wrong answers%s", d, run_bad, good[d] ? " (good)" : "");
			end
			best_len = 0; run_len = 0; start = 0; best_start = 0;
			for (d = 0; d < NDLY; d = d + 1) begin
				if (good[d]) begin
					if (run_len == 0) start = d;
					run_len = run_len + 1;
					if (run_len > best_len) begin best_len = run_len; best_start = start; end
				end
				else run_len = 0;
			end
			if (best_len == 0) begin
				fail("A2: no cable-delay setting answers the detection reads right");
				best_dly = 1;
			end
			else begin
				best_dly = best_start + (best_len - 1) / 2;
				pass($sformatf("A2: good cable-delay settings %0d..%0d, using %0d",
					best_start, best_start + best_len - 1, best_dly));
			end
			host.delay = best_dly;
			host.bank = 1'b1;
			host.build_lists();
			host.run_periods(1);				// the word in flight was sampled at the old delay
			n_rom_bad = 0; n_wr_bad = 0;			// judged from here on (A3)
			judge = 1'b1;
			// the wire decoder against what the chips were sent (stage B)
			if (stageb) begin
				integer bad;
				bad = 0;
				for (int r = 0; r <= 21; r++) if (cfg_now[r] !== chip0.regs[r]) bad++;
				if (nc > 0 && bad == 0)
					pass($sformatf("wire decoder: %0d SAMPLEs, %0d CONFIGs; CONFIG = the chips' registers", ns, nc));
				else
					fail($sformatf("wire decoder: %0d SAMPLEs, %0d CONFIGs, %0d CONFIG bytes differ from the chips'", ns, nc, bad));
			end
		end
	endtask

	// ---------------------------------------------------------------- main
	initial begin
		if ($test$plusargs("stageb")) stageb = 1'b1;
		if (!$value$plusargs("ippm=%f", ippm)) ippm = 2000.0;
		if (!$value$plusargs("d_data=%f", d_data)) d_data = 1000.0;
		if (!$value$plusargs("d_clk=%f", d_clk)) d_clk = 1300.0;
		if (!$value$plusargs("ins=%f", ins)) ins = 2500.0;
		if (!$value$plusargs("jitter=%f", jit)) jit = 300.0;
		if (!$value$plusargs("win=%f", win)) win = 400.0;
		if (!$value$plusargs("keep_phase=%d", keep_phase)) keep_phase = 1;
		if (!$value$plusargs("seed=%d", seed)) seed = 1;
		if (!$value$plusargs("periods=%d", periods)) periods = 400;
		void'($urandom(seed));
		adp.pllcdr.KEEP_PHASE = keep_phase;
		adp.iddrx4_0.WIN_PS = win;
		host.ppm = ippm;
		// the host's register values (RHX's; only their echo is checked here)
		for (int r = 0; r <= 21; r++) host.host_regs[r] = 8'h40 + r;
		host.build_lists();

		$display("tunnel_e2e_tb: stage %0s, Intan %0.0f ppm, d_data %0.0f d_clk %0.0f ins %0.0f ps, jitter ±%0.0f, win %0.0f ps, keep_phase %0d, seed %0d",
			stageb ? "B (frames, T5)" : "A (K4.2 pattern, cdr)", ippm, d_data, d_clk, ins, jit, win, keep_phase, seed);
		begin
			// predicted eye centres (cdr_tb's model): a sample o ps after a data
			// transition is clean for win + jitter < o < UI - jitter
			real t_ps, ui, o0, oc, st;
			t_ps = TK_NOM * 1000.0; ui = t_ps / 2.0;
			o0 = d_clk + ins * 1.0 - d_data;
			oc = (ui + win) / 2.0;
			st = (oc - o0) / (t_ps / 336.0);
			while (st < 0.0) st = st + 168.0;
			while (st >= 168.0) st = st - 168.0;
			$display("      predicted eye: width %0.0f steps, centres at steps %0.0f and %0.0f",
				(ui - win - 2.0 * jit) / (t_ps / 336.0), st, st + 168.0);
		end

		// power order: headstage first, then the adapter (R4 runbook rule)
		#1000.0 k_rstb = 1'b1;
		#200000.0 a_rstb = 1'b1;
		mark_event();				// start-up counts as an event (B6, B7)

		if (!stageb) begin
			// A1: the bench build's sweep, with the host scanning alongside
			fork
				wait_cdr("A1", 30.0);
				delay_scan();
			join
			if (adp.pllcdr.n_step_viol != 0)
				fail($sformatf("A1: %0d PHASESTEP pulse violations", adp.pllcdr.n_step_viol));
			// A3
			stream(periods);
			if (n_rom_bad == 0 && n_wr_bad == 0)
				pass($sformatf("A3: %0d ROM/CALIBRATE and %0d WRITE answers right (READ(0..21) %0d and CONVERT %0d not judged)",
					n_rom, n_wr, n_reg, n_const));
			else
				fail($sformatf("A3: %0d ROM and %0d WRITE answers wrong", n_rom_bad, n_wr_bad));
			// A4
			fork
				cable_out(200.0);
				stream(10);
			join
			fork
				wait_cdr("A4 (cable back)", 15.0);
				stream(400);
			join
			begin
				integer rb, wb;
				rb = n_rom_bad; wb = n_wr_bad;
				stream(periods);
				if (n_rom_bad == rb && n_wr_bad == wb) pass("A4: emulator answers right after the cable event");
				else fail("A4: emulator answers wrong after the cable event");
			end
		end
		else begin
			delay_scan();
			stream(periods);
			$display("      B: %0d SAMPLEs and %0d CONFIGs left Kuntur, %0d sweeps answered", ns, nc, n_sweeps);
			// B5: one lost frame
			fork inject_frames(1); stream(100); join
			// B5/R1: 8 consecutive lost frames: unlock, re-hunt at the same phase
			fork inject_frames(8); stream(300); join
			// B6
			fork cable_out(200.0); stream(300); join
			fork kuntur_reset(100.0); stream(400); join
			stream(periods);
			$display("      B: sweeps %0d (sentinel %0d), repeats %0d, skips %0d, gaps after a sentinel %0d",
				n_sweeps, n_sent_sweeps, n_repeat, n_skip, n_gap);
			$display("      B: expected slips at %0.0f ppm over %0.1f ms: about %0.1f",
				ippm, $realtime / 1.0e6, 25.0 * ippm * 1.0e-6 * $realtime / 1.0e6);	// 25 frames per ms
			if (n_conv_bad == 0 && n_torn == 0 && n_stale == 0 && n_lost_served == 0 &&
			    n_sent_bad == 0 && n_sent_missing == 0 && n_reg_bad == 0 && n_sweeps > 0)
				pass($sformatf("B1-B7: %0d CONVERT answers, %0d sweeps", n_conv, n_sweeps));
			if (n_lost > 0 && n_sent_sweeps == 0)
				fail("B5: errors injected, but no sentinel sweep");
			if (n_reg_cfg == 0)
				fail($sformatf("B7: no READ(0..21) judged against a delivered CONFIG (%0d CONFIGs, %0d deliverable)", nc, ncd));
			else
				$display("      B: READ(0..21) judged against a delivered CONFIG: %0d (%0d CONFIGs, %0d deliverable)", n_reg_cfg, nc, ncd);
		end

		$display("%s", (fails == 0) ? "tunnel_e2e_tb: ALL PASS" : $sformatf("tunnel_e2e_tb: %0d FAIL", fails));
		$finish;
	end

	// hard stop
	initial begin
		#400.0e6;
		fail("timeout");
		$display("tunnel_e2e_tb: %0d FAIL", fails);
		$finish;
	end
endmodule

// Library binding. Both tops define modules with the same name and
// different behaviour (crc32: Kuntur's load resets without the byte, the
// adapter's folds the first byte in), and -L order alone binds every such
// name to one library. This config makes each top's subtree bind from its
// own library. Run "tunnel_e2e_cfg", not "tunnel_e2e_tb" (run_e2e.sh).
config tunnel_e2e_cfg;
	design work.tunnel_e2e_tb;
	default liblist work kun adp;
	instance tunnel_e2e_tb.kun liblist kun;
	instance tunnel_e2e_tb.adp liblist adp;
endconfig
