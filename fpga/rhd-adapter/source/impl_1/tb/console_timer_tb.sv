//testbench
//
//	Testbench for the console's one-second timer, console_timer
//	(kuntur-notes docs/interfaces/companion-console.md §8.5):
//
//	  clk, rst_n ─► console_timer #(DIV) ─► tick  1-clock pulse every DIV clocks
//	                                      ─► t     seconds, 32-bit saturating
//
//	Four DUTs run side by side, DIV = 7, 100, 1000 and the default
//	(12,000,000), each against a cycle-accurate model of T1-T4:
//	  after each rising edge, with n = edges since rst_n rose:
//	    tick = 1 exactly when n is a multiple of DIV (n > 0), else 0;
//	    t    = min(number of such edges, FFFFFFFF), stepping with tick.
//
//	Checks
//	  1. T1/T2: tick and t match the model after every edge, for > 6
//	     periods of each small DIV and 2 periods of 12,000,000.
//	  2. T3: t preloaded to FFFFFFFD (DIV = 7, through dut.t, T5) climbs
//	     to FFFFFFFF and stays, with tick still coming.
//	  3. T4: rst_n pulled low mid-period (between edges): tick = 0 and
//	     t = 0 at once; counting restarts from the release.
//
//	Define: +define+SHORT skips the 12,000,000 DUT (quick runs).
//	Define: +define+REF_MODEL replaces the DUT by a model; +define+MUT=<k>
//	  breaks it (with REF_MODEL), each must FAIL:
//	    1 period DIV+1; 2 period DIV-1; 3 t wraps to 0; 4 tick two clocks
//	    wide; 5 t one clock after tick; 6 reset synchronous, not async.
//
//	Run (from this directory):
//	  tr '\r' '\n' < ../intan_rec_adapter.v | awk '/^module console_timer[ #(]/,/^endmodule/' > /tmp/tmr_rtl.v
//	  iverilog -g2012 -Wall -o tmr.vvp console_timer_tb.sv /tmp/tmr_rtl.v && vvp tmr.vvp
//	  iverilog -g2012 -Wall -DREF_MODEL -o tmr.vvp console_timer_tb.sv && vvp tmr.vvp
//	  for k in 1 2 3 4 5 6; do iverilog -g2012 -DREF_MODEL -DSHORT -DMUT=$k -o tmr.vvp console_timer_tb.sv && vvp tmr.vvp | grep console_timer_tb:; done
//	Expected: "console_timer_tb: ALL PASS" for the RTL and the model; FAIL
//	  for every mutant.

`timescale 1ns/1ps

`ifdef REF_MODEL
`ifndef MUT
	`define MUT 0
`endif
module console_timer #(
	parameter DIV = 12_000_000
)(
	input  wire	clk,
	input  wire	rst_n,
	output reg	tick,
	output reg	[31:0] t
);
	localparam P = (`MUT == 1) ? DIV + 1 : (`MUT == 2) ? DIV - 1 : DIV;
	reg [23:0] cnt;
	reg tick_d, step;
	generate if (`MUT == 6) begin : sync_rst
		always @(posedge clk)
			if (!rst_n) begin cnt <= 0; tick <= 0; t <= 0; tick_d <= 0; step <= 0; end
			else body;
	end else begin : async_rst
		always @(negedge rst_n or posedge clk)
			if (!rst_n) begin cnt <= 0; tick <= 0; t <= 0; tick_d <= 0; step <= 0; end
			else body;
	end endgenerate
	task body;
		begin
			step <= 0;
			if (cnt == P - 1) begin
				cnt <= 0;
				tick <= 1;
				if (`MUT == 5) step <= 1;
				else if (`MUT == 3 || t != 32'hFFFFFFFF) t <= t + 1;
			end else begin
				cnt <= cnt + 1;
				tick <= (`MUT == 4) ? tick && !tick_d : 0;
			end
			tick_d <= tick;
			if (`MUT == 5 && step && t != 32'hFFFFFFFF) t <= t + 1;
		end
	endtask
endmodule
`endif

// one DUT with its own model and checks
module timer_chk #(parameter integer DIV = 7, parameter SAT = 0) (
	input wire clk, input wire rst_n, output reg [31:0] fails, output reg [31:0] ticks
);
	wire tick;
	wire [31:0] t;
	console_timer #(.DIV(DIV)) dut (.clk(clk), .rst_n(rst_n), .tick(tick), .t(t));

	integer n = 0;			// edges since release, mod DIV
	reg [31:0] mt = 0;
	reg mtick = 0;
	initial begin fails = 0; ticks = 0; end

	task fail(input [8*90-1:0] msg);
		begin
			fails = fails + 1;
			if (fails <= 10) $display("FAIL  DIV=%0d: %0s  (tick %b want %b, t %h want %h, t=%0.3f us)",
				DIV, msg, tick, mtick, t, mt, $realtime/1000.0);
		end
	endtask

	// model, and the check just after each edge
	always @(posedge clk) begin
		#0.01;
		if (!rst_n) begin n = 0; mt = 0; mtick = 0; end
		else begin
			n = n + 1;
			mtick = 0;
			if (n == DIV) begin
				n = 0; mtick = 1; ticks = ticks + 1;
				if (mt != 32'hFFFFFFFF) mt = mt + 1;
			end
		end
		if (tick !== mtick) fail("tick wrong (T1)");
		if (t !== mt) fail("t wrong (T2/T3)");
	end

	// T4: at once on the falling edge of rst_n
	always @(negedge rst_n) begin
		#0.01;
		if (tick !== 1'b0 || t !== 32'd0) fail("not reset at once by rst_n (T4)");
		n = 0; mt = 0; mtick = 0;
	end

	// T3/T5: preload t near the top, between edges, right after a tick
	task preload(input [31:0] v);
		begin
			@(posedge clk); #0.01;
			while (!mtick) begin @(posedge clk); #0.01; end
			#1 dut.t = v; mt = v;
		end
	endtask
endmodule

module console_timer_tb;
	localparam real T = 1000.0 / 12.0;	// ns
	reg clk = 0, rst_n = 1;
	always #(T/2) clk = ~clk;

	wire [31:0] f7, f100, f1000, freal, k7, k100, k1000, kreal;
	timer_chk #(.DIV(7))    c7    (.clk(clk), .rst_n(rst_n), .fails(f7),    .ticks(k7));
	timer_chk #(.DIV(100))  c100  (.clk(clk), .rst_n(rst_n), .fails(f100),  .ticks(k100));
	timer_chk #(.DIV(1000)) c1000 (.clk(clk), .rst_n(rst_n), .fails(f1000), .ticks(k1000));
`ifndef SHORT
	timer_chk #(.DIV(12_000_000)) creal (.clk(clk), .rst_n(rst_n), .fails(freal), .ticks(kreal));
`else
	assign freal = 0, kreal = 2;
`endif

	integer i, total, extra = 0;
	integer sat_ticks;
	initial begin
		#0.05 rst_n = 0;
		#(T * 3.3) rst_n = 1;

		// periods of the small DIVs
		repeat (7000) @(posedge clk);

		// T3: saturation on DIV = 7
		c7.preload(32'hFFFFFFFD);
		sat_ticks = k7;
		repeat (7 * 10) @(posedge clk);
		#0.02;
		if (c7.dut.t !== 32'hFFFFFFFF) begin $display("FAIL  DIV=7: t not at FFFFFFFF after saturation (T3)"); extra = extra + 1; end
		if (k7 - sat_ticks < 9) begin $display("FAIL  DIV=7: ticks stopped at saturation (T3)"); extra = extra + 1; end

		// T4: reset mid-period, between edges
		repeat (450) @(posedge clk);
		#(T / 3) rst_n = 0;
		#(T * 2.6) rst_n = 1;
		repeat (5000) @(posedge clk);

`ifndef SHORT
		// two periods of the real DIV after the reset release
		wait (kreal == 2);
		repeat (10) @(posedge clk);
`endif
		#1;
		total = f7 + f100 + f1000 + freal + extra;
		$display("      ticks: DIV=7 %0d, 100 %0d, 1000 %0d, 12000000 %0d", k7, k100, k1000, kreal);
		if (total == 0) $display("console_timer_tb: ALL PASS");
		else $display("console_timer_tb: %0d FAIL", total);
		$finish;
	end
endmodule
