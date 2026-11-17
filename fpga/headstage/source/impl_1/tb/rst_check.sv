//	Reset check, shared by the reset-generator testbenches of both FPGAs:
//	rhd-adapter/source/impl_1/tb/reset_gen_adapter_tb.sv (rst_n_sup,
//	cdr_rstb) and headstage/source/impl_1/tb/reset_gen_kuntur_tb.sv
//	(rst_n_clkin, rst_n_clk, blk_rstb). Contract: vega
//	docs/interfaces/system-startup.md R-C8 and R-K7.
//	cond is what must hold for rst_n to be released.
//	  - initial 0, never X or Z;
//	  - 0 within 1 ps whenever cond = 0, with no clock edge;
//	  - rises only in the time step of a clk rising edge;
//	  - rises after MIN..MAX clk edges with cond continuously high;
//	  - does rise once cond has held for more than MAX edges.
`timescale 1ns/1ps

module rst_check #(parameter NAME = "rst", parameter MIN = 2, parameter MAX = 3) (
	input wire clk,
	input wire cond,
	input wire rst_n
);
	integer errors = 0, n = 0;
	realtime t_clk = -1.0;
	reg late = 0;

	task fail(input [8*48-1:0] what);
		begin errors = errors + 1; $display("%t  FAIL  %0s: %0s", $realtime, NAME, what); end
	endtask

	initial #0.001 if (rst_n !== 1'b0) fail("not in reset at wake-up");

	always @(rst_n) if (rst_n !== 1'b0 && rst_n !== 1'b1 && $realtime > 0) fail("X or Z");

	always @(cond, rst_n)
		if (cond === 1'b0 && rst_n !== 1'b0)
			#0.001 if (cond === 1'b0 && rst_n !== 1'b0) fail("not asserted at once");

	always @(negedge cond) begin n = 0; late = 0; end

	always @(posedge clk) begin
		t_clk = $realtime;
		n = cond ? n + 1 : 0;
		if (cond && n > MAX && rst_n !== 1'b1 && !late) begin
			late = 1; fail("not released");
		end
	end

	always @(posedge rst_n) begin
		if ($realtime != t_clk) fail("released off a clk edge");
		if (n < MIN || n > MAX) begin
			errors = errors + 1;
			$display("%t  FAIL  %0s: released after %0d edges, want %0d..%0d", $realtime, NAME, n, MIN, MAX);
		end
	end
endmodule
