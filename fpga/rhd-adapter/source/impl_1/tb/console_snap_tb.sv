//testbench
//
//	Testbench for the console's snapshot handshake, console_snap (clkin)
//	and console_shadow (wclk) together (kuntur-notes
//	docs/interfaces/companion-console.md §8.6, §8.7):
//
//	  clkin: tick, t, lock_sync ─► console_snap ─snap_req─► console_shadow ◄─ wclk inputs
//	                              ◄─snap_ack, sh_*─────────┘
//	                              ─► go, h_* (the record)
//
//	clkin is 12 MHz; wclk ~10.6 MHz with random jitter, so its phase
//	drifts against clkin; the tb stops and restarts it (whole cycles, no
//	glitch). The seven wclk inputs all come from one wclk counter k (f(k)
//	below), so a record that mixes two k is caught.
//
//	Checks
//	  shadow side (after each wclk edge):
//	    1. S1/S2: snap_ack toggles only with a request outstanding, on the
//	       3rd or 4th wclk edge after snap_req changed, and sh_* = f(k) of
//	       that edge (the copy is on the same edge as the ack);
//	    2. S3: sh_* never change without an ack toggle.
//	  supervisor side (after each clkin edge):
//	    3. N2: snap_req changes only on a tick edge; at a tick with the
//	       last request answered (ack settled > 4 clocks) it toggles; a
//	       request is never sent while one is outstanding;
//	    4. N6: one go per tick, one clock wide, inside the window, on the
//	       clock after the record changed; the record never changes
//	       without a go on the next clock;
//	    5. N3/N4: fresh (h_snap_ok = 1) only with an ack 3..6 clocks
//	       before the go and not before the tick (-4); stale only at
//	       TIMEOUT..TIMEOUT+3 after the tick, and not when an ack came in
//	       time;
//	    6. record: fresh = f(k) for one k, k at or after the request, newer
//	       than the last fresh one; stale keeps the last fresh fields
//	       (zeros before the first); h_t = t and h_lock = lock_sync.
//
//	Cases: 30 steady ticks; 10 stops from just before a tick to less than
//	TIMEOUT after it (fresh, late in the window); 10 stops from just
//	before a tick to past TIMEOUT within that second (stale, late ack);
//	5 such stops in two consecutive seconds (stale after a late ack);
//	3 stops of several seconds (stale lines, no extra request, recovery);
//	60 random stops; then 20 steady ticks.
//
//	Reset: con_rst_n (rst_n here) resets both blocks, the shadow through a
//	wclk reset synchroniser (§8.7 S4). It is asserted at t = 0 with wclk
//	stopped: both blocks must be 0 at once (asynchronous clear), and stay
//	0 once wclk runs, until the release. No GSR stand-in: there are no
//	reset-less flops.
//
//	Plusargs: +seed=<n>.
//	Defines: +define+REAL runs TIMEOUT = 120,000 (10 ms) with 33 ms ticks
//	  (few seconds of cases); default TIMEOUT = 300, DIV = 2000.
//	Define: +define+REF_MODEL replaces both DUTs by models; +define+MUT=<k>
//	  breaks them (with REF_MODEL), each must FAIL:
//	    1 shadow copies on every wclk clock; 2 copy one wclk edge after the
//	    ack; 3 request resent while pending; 4 late ack captured; 5 go on
//	    the capture clock; 6 no timeout; 7 one synchroniser flop in the
//	    shadow; 8 stale capture overwrites the shadow fields; 9 shadow
//	    with no reset; 10 shadow reset synchronous to wclk.
//
//	Run (from this directory):
//	  tr '\r' '\n' < ../intan_rec_adapter.v | awk '/^module console_(snap|shadow)[ #(]/,/^endmodule/' > /tmp/snap_rtl.v
//	  iverilog -g2012 -Wall -o snap.vvp console_snap_tb.sv /tmp/snap_rtl.v && vvp snap.vvp
//	  iverilog -g2012 -Wall -DREF_MODEL -o snap.vvp console_snap_tb.sv && vvp snap.vvp
//	  for k in 1 2 3 4 5 6 7 8 9 10; do iverilog -g2012 -DREF_MODEL -DMUT=$k -o snap.vvp console_snap_tb.sv && vvp snap.vvp | grep console_snap_tb:; done
//	Expected: "console_snap_tb: ALL PASS" for the RTL and the models; FAIL
//	  for every mutant.

`timescale 1ns/1ps

`ifdef REAL
	`define TO_CYC 120000
	`define DIV_CYC 400000
`else
	`define TO_CYC 300
	`define DIV_CYC 2000
`endif

`ifdef REF_MODEL
`ifndef MUT
	`define MUT 0
`endif
module console_shadow (
	input  wire	wclk,
	input  wire	rst_n,
	input  wire	snap_req,
	input  wire	[8:0]	phase, eye_center, eye_width,
	input  wire	cdr_rdy, cdr_valid,
	input  wire	[31:0]	pat_blocks, pat_errors,
	output reg	[8:0]	sh_phase, sh_eye_center, sh_eye_width,
	output reg	sh_cdr_rdy, sh_cdr_valid,
	output reg	[31:0]	sh_pat_blocks, sh_pat_errors,
	output reg	snap_ack
);
	reg req_s1, req_s2, req_s3, late, rst_w1, rst_w2;
	wire req_t = (`MUT == 7) ? (req_s1 ^ req_s2) : (req_s2 ^ req_s3);
	always @(negedge rst_n or posedge wclk)
		if (!rst_n) begin rst_w1 <= 0; rst_w2 <= 0; end
		else begin rst_w1 <= 1; rst_w2 <= rst_w1; end
	// MUT 9: no reset at all; MUT 10: reset synchronous to wclk
	wire rst_a = (`MUT == 9 || `MUT == 10) ? 1'b1 : rst_w2;
	task clr;
		begin
			req_s1 <= 0; req_s2 <= 0; req_s3 <= 0; late <= 0; snap_ack <= 0;
			sh_phase <= 0; sh_eye_center <= 0; sh_eye_width <= 0;
			sh_cdr_rdy <= 0; sh_cdr_valid <= 0; sh_pat_blocks <= 0; sh_pat_errors <= 0;
		end
	endtask
	always @(negedge rst_a or posedge wclk)
		if (!rst_a) clr;
		else if (`MUT == 10 && !rst_w2) clr;
		else begin
			req_s1 <= snap_req; req_s2 <= req_s1; req_s3 <= req_s2;
			late <= req_t;
			if (req_t) snap_ack <= ~snap_ack;
			if ((`MUT == 1) || (`MUT == 2 ? late : req_t)) begin
				sh_phase <= phase; sh_eye_center <= eye_center; sh_eye_width <= eye_width;
				sh_cdr_rdy <= cdr_rdy; sh_cdr_valid <= cdr_valid;
				sh_pat_blocks <= pat_blocks; sh_pat_errors <= pat_errors;
			end
		end
endmodule

module console_snap #(
	parameter TIMEOUT = 120000
)(
	input  wire	clk, rst_n,
	input  wire	tick,
	input  wire	[31:0]	t,
	input  wire	lock_sync,
	input  wire	snap_ack,
	input  wire	[8:0]	sh_phase, sh_eye_center, sh_eye_width,
	input  wire	sh_cdr_rdy, sh_cdr_valid,
	input  wire	[31:0]	sh_pat_blocks, sh_pat_errors,
	output reg	snap_req, go,
	output reg	[31:0]	h_t,
	output reg	h_lock, h_snap_ok, h_cdr_rdy, h_cdr_valid,
	output reg	[8:0]	h_phase, h_eye_center, h_eye_width,
	output reg	[31:0]	h_pat_blocks, h_pat_errors
);
	reg a1, a2, a3, win, go_d;
	reg [16:0] cnt;
	wire ack_t = a2 ^ a3;
	wire pending = snap_req ^ a2;
	task fresh;
		begin
			h_phase <= sh_phase; h_eye_center <= sh_eye_center; h_eye_width <= sh_eye_width;
			h_cdr_rdy <= sh_cdr_rdy; h_cdr_valid <= sh_cdr_valid;
			h_pat_blocks <= sh_pat_blocks; h_pat_errors <= sh_pat_errors;
			h_t <= t; h_lock <= lock_sync; h_snap_ok <= 1;
			win <= 0;
			if (`MUT == 5) go <= 1; else go_d <= 1;
		end
	endtask
	always @(negedge rst_n or posedge clk)
		if (!rst_n) begin
			a1 <= 0; a2 <= 0; a3 <= 0; win <= 0; go_d <= 0; cnt <= 0;
			snap_req <= 0; go <= 0;
			h_t <= 0; h_lock <= 0; h_snap_ok <= 0; h_cdr_rdy <= 0; h_cdr_valid <= 0;
			h_phase <= 0; h_eye_center <= 0; h_eye_width <= 0; h_pat_blocks <= 0; h_pat_errors <= 0;
		end else begin
			a1 <= snap_ack; a2 <= a1; a3 <= a2;
			go_d <= 0;
			go <= go_d;
			if (tick) begin
				win <= 1; cnt <= 0;
				if (!pending || `MUT == 3) snap_req <= ~snap_req;
			end else if (win) begin
				if (ack_t) fresh;
				else if (cnt == TIMEOUT - 1 && `MUT != 6) begin
					if (`MUT == 8) begin
						h_phase <= sh_phase; h_pat_blocks <= sh_pat_blocks; h_pat_errors <= sh_pat_errors;
						h_eye_center <= sh_eye_center; h_eye_width <= sh_eye_width;
						h_cdr_rdy <= sh_cdr_rdy; h_cdr_valid <= sh_cdr_valid;
					end
					h_t <= t; h_lock <= lock_sync; h_snap_ok <= 0;
					win <= 0;
					if (`MUT == 5) go <= 1; else go_d <= 1;
				end else
					cnt <= cnt + 1;
			end else if (ack_t && `MUT == 4)
				fresh;
		end
endmodule
`endif

module console_snap_tb;
	localparam real T = 1000.0 / 12.0;	// ns
	localparam integer TO = `TO_CYC;
	localparam integer DIV = `DIV_CYC;

	reg clk = 0, wclk = 0, wclk_en = 0, rst_n = 1;
	reg tick = 0, lock_sync = 0;
	reg [31:0] t = 0;
	integer seed = 1, fails = 0;

	always #(T/2) clk = ~clk;
	// wclk: ~94 ns +-4 ns per half-cycle pair; stops only between cycles
	always begin
		if (wclk_en) begin
			#(47.0 + ({$random(seed)} % 400) / 100.0 - 2.0) wclk = 1;
			#(47.0 + ({$random(seed)} % 400) / 100.0 - 2.0) wclk = 0;
		end else
			#7.3;
	end

	// ---- wclk inputs, all from k -----------------------------------------
	reg [31:0] k = 1000;
	function [92:0] f(input [31:0] x);
		reg [8:0] ph, ec, ew;
		begin
			ph = x % 336; ec = (x * 7 + 3) % 336; ew = (x * 13 + 5) % 337;
			f = {ph, ec, ew, x[4], x[5], x, x ^ 32'h5A5A_A5A5};
		end
	endfunction
	wire [92:0] fk = f(k);
	wire [8:0] phase = fk[92:84], eye_center = fk[83:75], eye_width = fk[74:66];
	wire cdr_rdy = fk[65], cdr_valid = fk[64];
	wire [31:0] pat_blocks = fk[63:32], pat_errors = fk[31:0];
	always @(negedge wclk) k = k + 1;	// stable around each rising edge

	// ---- DUTs ------------------------------------------------------------
	wire snap_req, snap_ack, go;
	wire [8:0] sh_phase, sh_eye_center, sh_eye_width, h_phase, h_eye_center, h_eye_width;
	wire sh_cdr_rdy, sh_cdr_valid, h_lock, h_snap_ok, h_cdr_rdy, h_cdr_valid;
	wire [31:0] sh_pat_blocks, sh_pat_errors, h_t, h_pat_blocks, h_pat_errors;

	console_shadow dut_sh (.wclk(wclk), .rst_n(rst_n), .snap_req(snap_req),
		.phase(phase), .eye_center(eye_center), .eye_width(eye_width),
		.cdr_rdy(cdr_rdy), .cdr_valid(cdr_valid), .pat_blocks(pat_blocks), .pat_errors(pat_errors),
		.sh_phase(sh_phase), .sh_eye_center(sh_eye_center), .sh_eye_width(sh_eye_width),
		.sh_cdr_rdy(sh_cdr_rdy), .sh_cdr_valid(sh_cdr_valid),
		.sh_pat_blocks(sh_pat_blocks), .sh_pat_errors(sh_pat_errors), .snap_ack(snap_ack));

	console_snap #(.TIMEOUT(TO)) dut_sn (.clk(clk), .rst_n(rst_n), .tick(tick), .t(t),
		.lock_sync(lock_sync), .snap_ack(snap_ack),
		.sh_phase(sh_phase), .sh_eye_center(sh_eye_center), .sh_eye_width(sh_eye_width),
		.sh_cdr_rdy(sh_cdr_rdy), .sh_cdr_valid(sh_cdr_valid),
		.sh_pat_blocks(sh_pat_blocks), .sh_pat_errors(sh_pat_errors),
		.snap_req(snap_req), .go(go), .h_t(h_t), .h_lock(h_lock), .h_snap_ok(h_snap_ok),
		.h_cdr_rdy(h_cdr_rdy), .h_cdr_valid(h_cdr_valid), .h_phase(h_phase),
		.h_eye_center(h_eye_center), .h_eye_width(h_eye_width),
		.h_pat_blocks(h_pat_blocks), .h_pat_errors(h_pat_errors));


	task fail(input [8*100-1:0] msg);
		begin fails = fails + 1; if (fails <= 25) $display("FAIL  %0s  (t=%0.3f us)", msg, $realtime/1000.0); end
	endtask

	wire [92:0] sh_pack = {sh_phase, sh_eye_center, sh_eye_width, sh_cdr_rdy, sh_cdr_valid, sh_pat_blocks, sh_pat_errors};
	wire [92:0] h_pack  = {h_phase, h_eye_center, h_eye_width, h_cdr_rdy, h_cdr_valid, h_pat_blocks, h_pat_errors};
	wire [126:0] rec = {h_pack, h_t, h_lock, h_snap_ok};

	// ---- event bookkeeping -----------------------------------------------
	integer we = 0, ce = 0;			// wclk / clkin rising edges so far
	integer req_we = 0, req_ce = -100, ack_ce = -100;
	reg [31:0] req_k = 0;
	reg pend_sh = 0;
	integer req_tog = 0, ack_tog = 0;
	always @(snap_req) if ($realtime > 1.0) begin
		if (pend_sh) fail("snap_req changed with a request outstanding (N2)");
		req_we = we; req_ce = ce; req_k = k; pend_sh = 1; req_tog = req_tog + 1;
	end
	always @(snap_ack) if ($realtime > 1.0) begin ack_ce = ce; ack_tog = ack_tog + 1; end

	// ---- shadow side checks ----------------------------------------------
	reg [92:0] sh_prev = 0;
	reg ack_prev = 0;
	reg [31:0] kedge;
	integer n;
	always @(posedge wclk) begin
		we = we + 1;
		kedge = k;
		#0.01;
		if (snap_ack !== ack_prev) begin
			n = we - req_we;
			if (!pend_sh) fail("snap_ack toggled with no request (S2)");
			else if (n < 3 || n > 4) fail("snap_ack not on the 3rd/4th wclk edge after snap_req (S1)");
			if (sh_pack !== f(kedge)) fail("shadow not copied on the ack edge (S2)");
			pend_sh = 0;
		end else if (sh_pack !== sh_prev)
			fail("shadow changed without an ack (S3)");
		sh_prev = sh_pack;
		ack_prev = snap_ack;
	end

	// ---- supervisor side checks ------------------------------------------
	integer ws = -1000000, rec_ce = -100, nticks = 0, ngo = 0, nfresh = 0, nstale = 0;
	reg win_tb = 0, got_go = 0, go_prev = 0, req_prev = 0, tick_s;
	reg [126:0] rec_prev = 0;
	reg [92:0] last_good = 0;
	reg [31:0] last_good_k = 0;
	reg have_good = 0;
	always @(posedge clk) begin
		ce = ce + 1;
		tick_s = tick;
		#0.01;
		if (rst_n) begin
			// N2
			if (req_ce == ce && !tick_s) fail("snap_req changed outside a tick (N2)");
			if (tick_s) begin
				if (win_tb && !got_go) fail("no go for the previous tick (N6)");
				nticks = nticks + 1;
				ws = ce; win_tb = 1; got_go = 0;
				if (ce - ack_ce > 4 && req_prev == ack_prev && req_ce != ce)
					fail("no request at a tick with the last one answered (N2)");
			end
			// record changes only on the clock before a go
			if (rec !== rec_prev) rec_ce = ce;
			if (rec_ce == ce - 1 && !go) fail("record changed without a go on the next clock (N6, N7)");
			// go
			if (go) begin
				ngo = ngo + 1;
				if (go_prev) fail("go wider than one clock (N6)");
				if (!win_tb) fail("go outside a tick's window (N5, N6)");
				else if (got_go) fail("second go for one tick (N6)");
				got_go = 1;
				if (rec_ce != ce - 1) fail("go not on the clock after the capture (N6)");
				if (h_t !== t) fail("h_t not the current t (N3, N4)");
				if (h_lock !== lock_sync) fail("h_lock not the current lock_sync (N3, N4)");
				if (h_snap_ok === 1'b1) begin
					nfresh = nfresh + 1;
					if (!(ack_ce >= ws - 4 && ce - ack_ce >= 3 && ce - ack_ce <= 6))
						fail("fresh record without a fresh ack (N3, N5)");
					if (h_pack !== f(h_pat_blocks)) fail("torn record: fields from different k (S2, N3)");
					if (h_pat_blocks < req_k) fail("fresh record older than the request (N3)");
					if (have_good && h_pat_blocks <= last_good_k) fail("fresh record not newer than the last one (N3)");
					last_good = h_pack; last_good_k = h_pat_blocks; have_good = 1;
				end else if (h_snap_ok === 1'b0) begin
					nstale = nstale + 1;
					if (ce - ws < TO || ce - ws > TO + 3) fail("stale go not at the timeout (N4)");
					if (ack_ce > ws + 1 && ack_ce < ws + TO - 5) fail("stale although the ack came in time (N3)");
					if (h_pack !== last_good) fail("stale record changed the shadow fields (N4)");
				end else
					fail("h_snap_ok is X");
			end
			if (win_tb && !got_go && ce > ws + TO + 4) begin
				fail("no go within TIMEOUT of a tick (N4, N6)");
				win_tb = 0;
			end
			if (got_go) win_tb = 0;
		end
		go_prev = go;
		req_prev = snap_req;
		rec_prev = rec;
	end

	// ---- tick, t and lock_sync (clkin side, change on the falling edge) ---
	integer tcnt = 0;
	reg run_ticks = 0;
	always @(negedge clk) begin
		tick = 0;
		if (run_ticks) begin
			tcnt = tcnt + 1;
			if (tcnt == DIV) begin tcnt = 0; tick = 1; t = t + 1; end
			// lock_sync moves only outside the windows
			if (tcnt > TO + 10 && tcnt < DIV - 10 && ({$random(seed)} % 700) == 0)
				lock_sync = ~lock_sync;
		end
	end

	task wait_tick;
		begin @(posedge clk); while (!tick) @(posedge clk); end
	endtask
	task wait_clk(input integer m);
		integer j;
		begin for (j = 0; j < m; j = j + 1) @(posedge clk); end
	endtask
	// stop wclk at `off` clkin clocks after a tick, for `len` clkin clocks
	task stop_wclk(input integer off, input integer len);
		begin
			wait_tick; wait_clk(off);
			wclk_en = 0; wait_clk(len); wclk_en = 1;
		end
	endtask

	// stop wclk `lead` clkin clocks before a tick, for `len` clkin clocks
	task stop_before_tick(input integer lead, input integer len);
		begin
			wait_tick; wait_clk(DIV - lead);
			wclk_en = 0; wait_clk(len); wclk_en = 1;
		end
	endtask

	// stale, late ack refreshing the shadow, then stale again at the next
	// tick: the second stale line must keep the last fresh record, not the
	// newer shadow (N4)
	task stale_pair;
		integer l1, l2;
		begin
			l1 = 1 + {$random(seed)} % 20; l2 = 1 + {$random(seed)} % 20;
			wait_tick; wait_clk(DIV - l1);
			wclk_en = 0; wait_clk(l1 + TO + 100); wclk_en = 1;
			wait (tcnt == DIV - l2);
			wclk_en = 0; wait_clk(l2 + TO + 100); wclk_en = 1;
		end
	endtask

	integer i, ld;
	initial begin
		if ($value$plusargs("seed=%d", seed)) ;
		// S4/N8: reset with wclk stopped: everything 0 at once (asynchronous)
		#0.05 rst_n = 0;
		#1;
		if (sh_pack !== 93'd0 || snap_ack !== 1'b0) fail("shadow not 0 at once in reset, wclk stopped (S4)");
		if (snap_req !== 1'b0 || go !== 1'b0 || rec !== 127'd0) fail("console_snap not 0 at once in reset (N8)");
		#(T * 2) wclk_en = 1;
		#(T * 5.3);
		if (sh_pack !== 93'd0 || snap_ack !== 1'b0) fail("shadow not 0 in reset, wclk running (S4)");
		rst_n = 1;
		#(T * 3) run_ticks = 1;

		// steady
		for (i = 0; i < 30; i = i + 1) wait_tick;
		$display("      steady: %0d ticks, %0d fresh, %0d stale", nticks, nfresh, nstale);
		// stops shorter than TIMEOUT, starting just before/after the request
		for (i = 0; i < 10; i = i + 1) begin
			ld = 1 + {$random(seed)} % 20;
			stop_before_tick(ld, ld + TO / 3 + {$random(seed)} % (TO / 3));
		end
		$display("      short stops: %0d ticks, %0d fresh, %0d stale", nticks, nfresh, nstale);
		// stops past TIMEOUT within one second: stale, then a late ack
		for (i = 0; i < 10; i = i + 1) begin
			ld = 1 + {$random(seed)} % 20;
			stop_before_tick(ld, ld + TO + 50 + {$random(seed)} % (DIV - TO - 200));
		end
		$display("      long stops: %0d ticks, %0d fresh, %0d stale", nticks, nfresh, nstale);
		for (i = 0; i < 5; i = i + 1) stale_pair;
		$display("      stale pairs: %0d ticks, %0d fresh, %0d stale", nticks, nfresh, nstale);
		// several seconds
		for (i = 0; i < 3; i = i + 1)
			stop_wclk({$random(seed)} % DIV, 3 * DIV + {$random(seed)} % DIV);
		$display("      multi-second stops: %0d ticks, %0d fresh, %0d stale", nticks, nfresh, nstale);
`ifndef REAL
		// random stops
		for (i = 0; i < 60; i = i + 1)
			stop_wclk({$random(seed)} % DIV, {$random(seed)} % (2 * DIV));
		$display("      random stops: %0d ticks, %0d fresh, %0d stale", nticks, nfresh, nstale);
`endif
		for (i = 0; i < 20; i = i + 1) wait_tick;
		wait_clk(TO + 20);

		$display("      %0d ticks, %0d go (%0d fresh, %0d stale); %0d requests, %0d acks",
			nticks, ngo, nfresh, nstale, req_tog, ack_tog);
		if (ngo != nticks) fail("go count differs from tick count");
		if (nfresh == 0 || nstale == 0) fail("a case did not occur (no fresh or no stale line)");
		if (fails == 0) $display("console_snap_tb: ALL PASS");
		else $display("console_snap_tb: %0d FAIL", fails);
		$finish;
	end
endmodule
