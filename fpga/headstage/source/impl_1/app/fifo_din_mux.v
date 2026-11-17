/*********************************************************************************
	Author: Manuel Monge
	Description:
		Selects the FIFO's data source: real RHD2164 data (ch_sel_dout) or
		the test_pattern_gen ramp, per data_source_sel (regbank word 229;
		reset default 0 = real data). Mux half of the split above.
*********************************************************************************/

module fifo_din_mux (
	input  wire	[1:0]	data_source_sel,
	input  wire	[31:0]	ch_sel_dout,	// Real RHD2164 data
	input  wire	[31:0]	dout_test,		// test_pattern_gen ramp
	output wire	[31:0]	fifo_din
);

	assign fifo_din = (data_source_sel == 2'd1) ? dout_test : ch_sel_dout;

endmodule
