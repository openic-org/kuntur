/*********************************************************************************
	Author: Manuel Monge
	Description:
		Test-pattern (ramp) generator for the fifo_din source mux. Split out
		of the combined fifo_din_mux_reg module, 2026-08-26 (PLAN.md A.1's
		original structural-fix note: "extract test-pattern generation into
		its own module; mux at the top level behind an obvious named
		signal" — this module is the generator half of that).
*********************************************************************************/

module test_pattern_gen (
	input  wire	rstb,			// Active-low asyncrhonous reset
	input  wire	clk,			// Input clock
	input  wire dout_en_0,
	output wire	[31:0]	dout_test	// Ramp output: {ch0, ch1 = ch0+1000}
);

	reg [15:0] cnt0;
	wire [15:0] ch0, ch1;

	always@(negedge rstb or posedge clk)
		if (!rstb)
			cnt0 <= 0;
		else if (dout_en_0)
			cnt0 <= cnt0 + 1;

	assign ch0 = cnt0;
	assign ch1 = cnt0 + 16'd1000;
	assign dout_test = {ch0, ch1};

endmodule
