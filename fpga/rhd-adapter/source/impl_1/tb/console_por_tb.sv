//testbench
//
//	Testbench for the console's power-on reset, console_por (kuntur-notes
//	docs/interfaces/companion-console.md §8.4):
//
//	  clk ─► console_por ─► con_rst_n
//
//	There is no reset input (D8). On the chip the block's flops are
//	instantiated FD1P3DX (REGSET = "RESET") and wake up 0 by the
//	configuration's GSR; LSE removes an RTL version with no reset (tried
//	2026-11-19). This tb supplies an FD1P3DX model whose Q is 0 at t = 0
//	(the GSR stand-in); +define+LATTICE_PRIMS leaves it out.
//
//	Checks
//	  1. P1: con_rst_n = 0, not X, from time 0 until the release.
//	  2. P2: con_rst_n rises on exactly the POR_CYCLES-th rising edge of clk.
//	  3. P3: then stays 1 for 1,100,000 clocks (past any 8-bit to 20-bit
//	     counter wrap), checked on every edge and between edges.
//
//	Define: +define+POR_N=<n> sets the expected POR_CYCLES (default 16;
//	  the tb does not override the DUT's value).
//	Define: +define+REF_MODEL replaces the DUT by a model; +define+MUT=<k>
//	  breaks it (with REF_MODEL), each must FAIL:
//	    1 releases one edge early; 2 one edge late; 3 counter wraps
//	    (8 bits, reset again after 256 edges); 4 no initial value (X).
//
//	Run (from this directory):
//	  tr '\r' '\n' < ../intan_rec_adapter.v | awk '/^module console_por[ #(]/,/^endmodule/' > /tmp/por_rtl.v
//	  iverilog -g2012 -Wall -o por.vvp console_por_tb.sv /tmp/por_rtl.v && vvp por.vvp
//	  iverilog -g2012 -Wall -DREF_MODEL -o por.vvp console_por_tb.sv && vvp por.vvp
//	  for k in 1 2 3 4; do iverilog -g2012 -DREF_MODEL -DMUT=$k -o por.vvp console_por_tb.sv && vvp por.vvp | grep console_por_tb:; done
//	Expected: "console_por_tb: ALL PASS" for the RTL and the model; FAIL for
//	  every mutant.

`timescale 1ns/1ps

`ifndef POR_N
	`define POR_N 16
`endif

`ifdef REF_MODEL
`ifndef MUT
	`define MUT 0
`endif
module console_por #(
	parameter POR_CYCLES = `POR_N
)(
	input  wire	clk,
	output reg	con_rst_n
);
	reg [19:0] cnt;
	reg [7:0] c8, c8n;
	initial begin
		if (`MUT != 4) begin cnt = 0; c8 = 0; con_rst_n = 1'b0; end
	end
	localparam N = (`MUT == 1) ? POR_CYCLES - 1 : (`MUT == 2) ? POR_CYCLES + 1 : POR_CYCLES;
	always @(posedge clk)
		if (`MUT == 3) begin
			c8n = c8 + 8'd1;
			c8 <= c8n;
			con_rst_n <= (c8n >= N);
		end else if (!con_rst_n) begin
			cnt <= cnt + 1;
			if (cnt + 1 == N) con_rst_n <= 1'b1;
		end
endmodule
`endif

`ifndef REF_MODEL
`ifndef LATTICE_PRIMS
// FD1P3DX for simulation (Lattice's own model is encrypted; Icarus cannot
// read it). Pins as the Lattice library: SP clock enable, CD asynchronous
// clear, both active high. GSR stand-in: Q = 0 at t = 0, as REGSET =
// "RESET" with the configuration's wake-up GSR (system-startup.md R-C8).
module FD1P3DX #(parameter GSR = "ENABLED") (input wire D, input wire SP,
		input wire CK, input wire CD, output reg Q);
	initial Q = 1'b0;
	always @(posedge CK or posedge CD)
		if (CD) Q <= 1'b0;
		else if (SP) Q <= D;
endmodule
`endif
`endif

module console_por_tb;
	localparam real T = 1000.0 / 12.0;	// ns
	localparam integer N = `POR_N;
	localparam integer RUN = 1_100_000;
	reg clk = 0;
	wire con_rst_n;
	integer fails = 0, edges = 0, rise_at = -1;

	console_por dut (.clk(clk), .con_rst_n(con_rst_n));
	always #(T/2) clk = ~clk;

	task fail(input [8*100-1:0] msg);
		begin fails = fails + 1; if (fails <= 20) $display("FAIL  %0s  (edge %0d, t=%0.3f us)", msg, edges, $realtime/1000.0); end
	endtask

	// P1 at time 0 (after the initial values settle, before the first edge)
	initial begin
		#0.01;
		if (con_rst_n !== 1'b0) fail("con_rst_n not 0 at time 0 (P1)");
	end

	// count edges; check just after each one
	always @(posedge clk) begin
		edges = edges + 1;
		#0.01;
		if (edges < N) begin
			if (con_rst_n !== 1'b0) fail("con_rst_n not 0 before edge POR_CYCLES (P1, P2)");
		end else begin
			if (con_rst_n !== 1'b1) fail("con_rst_n not 1 on or after edge POR_CYCLES (P2, P3)");
			if (con_rst_n === 1'b1 && rise_at < 0) rise_at = edges;
		end
	end

	// P3 between edges as well: any change after the release is a fail
	always @(con_rst_n)
		if (rise_at > 0 && $realtime > 0.02) fail("con_rst_n changed after the release (P3)");

	initial begin
		wait (edges == RUN);
		#1;
		$display("      POR_CYCLES expected %0d; con_rst_n rose on edge %0d; %0d edges run", N, rise_at, edges);
		if (fails == 0) $display("console_por_tb: ALL PASS");
		else $display("console_por_tb: %0d FAIL", fails);
		$finish;
	end
endmodule
