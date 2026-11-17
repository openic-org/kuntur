//testbench
//
//	Testbench for the companion's status LEDs (status_leds).
//	Contract: vega docs/interfaces/system-startup.md §2.3 R-C6.
//
//	  clkin (12 MHz) ─┐
//	  rst_n_sup ──────┤
//	  lock_sync ──────┤ status_leds ─► led_n[4:0], active low
//	  gddr_rdy ───────┤
//	  wclk flops ─────┘ cdr_rdy, cdr_valid (11.4 MHz, unrelated phase)
//
//	Checks
//	  1. Reset: led_n = 11111 (all off) while rst_n_sup = 0, set at once
//	     (asynchronously) when it falls, with no clkin edge: at 0.1 ns,
//	     before the first clkin edge, and again in operation. The
//	     power-up value itself (FD1P3BX on led_n) is check_powerup.py's
//	     job, not this tb's: no GSR stand-in.
//	  2. Truth table, all 16 combinations of lock_sync, gddr_rdy, cdr_rdy,
//	     cdr_valid, after settling:
//	       LED1 = lock_sync
//	       LED2 = gddr_rdy && !rdy
//	       LED3 = gddr_rdy && valid
//	       LED4 = gddr_rdy && rdy && !valid
//	  3. Latency, in clkin rising edges after the input changes (strictly
//	     after): lock_sync, gddr_rdy (clkin signals, registered once): 1;
//	     cdr_rdy, cdr_valid (two synchroniser flops, then the LED flop): 3.
//	     The tb changes the clkin inputs just after a clkin edge and the
//	     wclk inputs on a wclk edge, never within 1 ns of a clkin edge, so
//	     each count is exact.
//	  4. Heartbeat (LED0): toggles every HB = 2^23 clkin edges after
//	     rst_n_sup rises (0.70 s on, 0.70 s off), three half-periods
//	     measured; reset clears it, and the count starts again.
//	  5. Glitch-free pins: no led_n bit changes except on a clkin rising
//	     edge or the falling edge of rst_n_sup.
//
//	Plusargs: +seed=<n> (wclk phase, the order of the combinations).
//	Define: +define+REF_MODEL replaces the DUT by a model written from the
//	  spec; +define+MUT=<k> breaks it (with REF_MODEL), each must FAIL:
//	    1 no synchroniser on cdr_rdy/cdr_valid; 2 LED2 not gated by READY;
//	    3 LEDs active high; 4 synchronous reset; 5 heartbeat 2^21;
//	    6 LED4 without !valid.
//
//	Run (from this directory). The RTL is cut out of intan_rec_adapter.v
//	(tr: the file mixes CR and CRLF line endings).
//	  tr '\r' '\n' < ../intan_rec_adapter.v | awk '/^module status_leds /,/^endmodule/' > /tmp/sl_rtl.v
//	  iverilog -g2012 -Wall -o sl.vvp status_leds_tb.sv /tmp/sl_rtl.v && vvp sl.vvp
//	  iverilog -g2012 -Wall -DREF_MODEL -o sl.vvp status_leds_tb.sv && vvp sl.vvp
//	About 1.6 s simulated (the heartbeat), some tens of seconds to run.

`timescale 1ns/1ps

`ifdef REF_MODEL
module status_leds (
	input  wire       clkin,
	input  wire       rst_n_sup,
	input  wire       lock_sync,
	input  wire       gddr_rdy,
	input  wire       cdr_rdy,
	input  wire       cdr_valid,
	output reg  [4:0] led_n
);
`ifndef MUT
`define MUT 0
`endif
	localparam integer HB_BIT = (`MUT == 5) ? 21 : 23;
	reg [HB_BIT:0] cnt;
	reg [1:0] rdy_q, valid_q;
	wire rdy_s   = (`MUT == 1) ? cdr_rdy   : rdy_q[1];
	wire valid_s = (`MUT == 1) ? cdr_valid : valid_q[1];
	reg  [4:0] on;
	always @(*) begin
		on[0] = cnt[HB_BIT];
		on[1] = lock_sync;
		on[2] = ((`MUT == 2) ? 1'b1 : gddr_rdy) && !rdy_s;
		on[3] = gddr_rdy && valid_s;
		on[4] = gddr_rdy && rdy_s && ((`MUT == 6) ? 1'b1 : !valid_s);
	end
	generate if (`MUT == 4) begin : sync_rst
		always @(posedge clkin)
			if (!rst_n_sup) begin cnt <= 0; rdy_q <= 0; valid_q <= 0; led_n <= 5'b11111; end
			else begin
				cnt <= cnt + 1;
				rdy_q <= {rdy_q[0], cdr_rdy}; valid_q <= {valid_q[0], cdr_valid};
				led_n <= ~on;
			end
	end else begin : async_rst
		always @(negedge rst_n_sup or posedge clkin)
			if (!rst_n_sup) begin cnt <= 0; rdy_q <= 0; valid_q <= 0; led_n <= (`MUT == 3) ? 5'b00000 : 5'b11111; end
			else begin
				cnt <= cnt + 1;
				rdy_q <= {rdy_q[0], cdr_rdy}; valid_q <= {valid_q[0], cdr_valid};
				led_n <= (`MUT == 3) ? on : ~on;
			end
	end endgenerate
endmodule
`endif

module status_leds_tb;
	localparam real T_CLKIN = 1.0e3 / 12.0;		// 83.33 ns
	localparam real T_WCLK  = 87.835;			// ECLKDIV ÷4 at 45.54 MHz
	localparam integer HB = 1 << 23;

	reg clkin = 0, wclk = 0;
	reg rst_n_sup = 1, lock_sync = 0, gddr_rdy = 0;
	reg rdy_d = 0, valid_d = 0;		// wanted values, taken by the wclk flops
	reg cdr_rdy = 0, cdr_valid = 0;		// the cdr_fsm output flops (wclk)
	wire [4:0] led_n;
	wire [4:0] on = ~led_n;

	integer fails = 0, seed, i, k, n, e;
	real    wphase;

	status_leds dut (.clkin(clkin), .rst_n_sup(rst_n_sup), .lock_sync(lock_sync), .gddr_rdy(gddr_rdy),
		.cdr_rdy(cdr_rdy), .cdr_valid(cdr_valid), .led_n(led_n));

	always #(T_CLKIN / 2.0) clkin = ~clkin;
	initial begin
		if (!$value$plusargs("seed=%d", seed)) seed = 1;
		i = $urandom(seed);
		wphase = ($urandom % 1000) / 1000.0 * T_WCLK;
		#(wphase);
		forever #(T_WCLK / 2.0) wclk = ~wclk;
	end
	// cdr's outputs change only on a wclk edge. Edges within 1 ns of a clkin
	// rising edge are skipped, so the latency counts are exact.
	real t_clk_rise = -1.0e9;
	always @(posedge clkin) t_clk_rise = $realtime;
	always @(posedge wclk) begin
		if ($realtime - t_clk_rise > 1.0 && t_clk_rise + T_CLKIN - $realtime > 1.0) begin
			cdr_rdy   <= rdy_d;
			cdr_valid <= valid_d;
		end
	end

	// 5. Pins change only on a clkin rising edge or the fall of rst_n_sup.
	integer glitches = 0;
	real    t_rst_fall = -1.0e9;
	always @(negedge rst_n_sup) t_rst_fall = $realtime;
	always @(led_n) if ($realtime > 1.0 && $realtime != t_clk_rise && $realtime != t_rst_fall) begin
		glitches = glitches + 1;
		if (glitches <= 3) $display("  led_n changed to %b at %0.3f ns, not on a clkin edge", led_n, $realtime);
	end

	function [4:0] expect_on(input l, input g, input r, input v);
		expect_on = {g && r && !v, g && v, g && !r, l, 1'b0};
	endfunction

	task check(input cond, input [8*100-1:0] what);
		if (cond) $display("PASS  %0s", what);
		else begin $display("FAIL  %0s", what); fails = fails + 1; end
	endtask

	// clkin rising edges, strictly after now, until LED bit b equals val
	task edges_until(input integer b, input val, output integer cnt);
		begin
			cnt = 0;
			while (on[b] !== val && cnt < 100) begin @(posedge clkin); cnt = cnt + 1; #0.01; end
		end
	endtask

	integer lat_bad, comb_bad, hb_bad;
	integer h0, h1, h2;
	reg [3:0] c;
	reg [4:0] want;
	integer order [0:15];

	initial begin
		// ---- 1. Reset before the first clkin edge ----
		#0.1 rst_n_sup = 0;
		#1;
		check(led_n === 5'b11111, "reset: all LEDs off at once when rst_n_sup falls, before any clkin edge");
		repeat (5) @(posedge clkin);
		check(led_n === 5'b11111, "reset: still all off after 5 clkin edges in reset");

		// ---- 2. Truth table (all 16 combinations, random order) ----
		@(posedge clkin); #1 rst_n_sup = 1;
		for (i = 0; i < 16; i = i + 1) order[i] = i;
		for (i = 15; i > 0; i = i - 1) begin k = $urandom % (i + 1); n = order[i]; order[i] = order[k]; order[k] = n; end
		comb_bad = 0;
		for (i = 0; i < 16; i = i + 1) begin
			c = order[i];
			@(posedge clkin); #1;
			lock_sync = c[3]; gddr_rdy = c[2];
			rdy_d = c[1]; valid_d = c[0];
			repeat (8) @(posedge clkin); #1;
			want = expect_on(c[3], c[2], c[1], c[0]);
			if (on[4:1] !== want[4:1]) begin
				comb_bad = comb_bad + 1;
				$display("  lock %b READY %b rdy %b valid %b: LED4..1 = %b, expected %b", c[3], c[2], c[1], c[0], on[4:1], want[4:1]);
			end
		end
		check(comb_bad == 0, "truth table: LED1..LED4 right for all 16 input combinations");

		// ---- 3. Latency ----
		lat_bad = 0;
		// clkin inputs: 1 edge
		@(posedge clkin); #1 lock_sync = 0; gddr_rdy = 1; rdy_d = 0; valid_d = 0;
		repeat (8) @(posedge clkin);
		@(posedge clkin); #1 lock_sync = 1; edges_until(1, 1, e);
		if (e != 1) begin lat_bad = lat_bad + 1; $display("  lock_sync -> LED1: %0d clkin edges, expected 1", e); end
		@(posedge clkin); #1 gddr_rdy = 0; edges_until(2, 0, e);
		if (e != 1) begin lat_bad = lat_bad + 1; $display("  gddr_rdy -> LED2: %0d clkin edges, expected 1", e); end
		@(posedge clkin); #1 gddr_rdy = 1;
		repeat (8) @(posedge clkin);
		// wclk inputs: 3 edges, several times (different wclk/clkin phases)
		for (k = 0; k < 8; k = k + 1) begin
			rdy_d = 1; @(cdr_rdy); edges_until(2, 0, e);
			if (e != 3) begin lat_bad = lat_bad + 1; $display("  cdr_rdy rise -> LED2 off: %0d clkin edges, expected 3", e); end
			valid_d = 1; @(cdr_valid); edges_until(3, 1, e);
			if (e != 3) begin lat_bad = lat_bad + 1; $display("  cdr_valid rise -> LED3 on: %0d clkin edges, expected 3", e); end
			valid_d = 0; @(cdr_valid); edges_until(4, 1, e);
			if (e != 3) begin lat_bad = lat_bad + 1; $display("  cdr_valid fall -> LED4 on: %0d clkin edges, expected 3", e); end
			rdy_d = 0; @(cdr_rdy); edges_until(2, 1, e);
			if (e != 3) begin lat_bad = lat_bad + 1; $display("  cdr_rdy fall -> LED2 on: %0d clkin edges, expected 3", e); end
			repeat ($urandom % 5) @(posedge wclk);
		end
		check(lat_bad == 0, "latency: clkin inputs 1 clkin edge, cdr_rdy/cdr_valid 3 (2 synchroniser flops)");

		// ---- 1b. Asynchronous reset in operation ----
		@(posedge clkin); #1 lock_sync = 1; gddr_rdy = 1; rdy_d = 1; valid_d = 1;
		repeat (10) @(posedge clkin);
		#(T_CLKIN / 4.0) rst_n_sup = 0;		// between clkin edges
		#1;
		check(led_n === 5'b11111, "reset: all LEDs off at once when rst_n_sup falls, no clkin edge");
		#(T_CLKIN) rst_n_sup = 1;		// mid-period: the release lands between edges

		// ---- 4. Heartbeat, from this release ----
		hb_bad = 0;
		h0 = 0; while (!on[0] && h0 < 2 * HB) begin @(posedge clkin); h0 = h0 + 1; #0.01; end
		h1 = 0; while ( on[0] && h1 < 2 * HB) begin @(posedge clkin); h1 = h1 + 1; #0.01; end
		h2 = 0; while (!on[0] && h2 < 2 * HB) begin @(posedge clkin); h2 = h2 + 1; #0.01; end
		$display("  heartbeat: first on after %0d clkin edges, on %0d, off %0d (expected %0d each, %0.3f s)",
			h0, h1, h2, HB, HB * T_CLKIN / 1.0e9);
		if (h0 < HB - 1 || h0 > HB + 2) hb_bad = hb_bad + 1;
		if (h1 != HB) hb_bad = hb_bad + 1;
		if (h2 != HB) hb_bad = hb_bad + 1;
		check(hb_bad == 0, "heartbeat: LED0 toggles every 2^23 clkin (0.70 s), counted from the reset release");
		// LED1..LED4 kept working meanwhile
		check(on[4:1] === 4'b0101, "LED1..LED4 during the heartbeat: lock and link up");

		check(glitches == 0, "pins: led_n changes only on a clkin edge or when rst_n_sup falls");

		if (fails == 0) $display("status_leds_tb: ALL PASS");
		else            $display("status_leds_tb: %0d FAILED", fails);
		$finish;
	end
endmodule
