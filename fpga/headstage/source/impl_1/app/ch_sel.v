/*********************************************************************************
	Author: Manuel Monge
	Description:
		Picks the two runtime-selected channels (ch_a/ch_b) out of the two
		RHD2164s' four MISO halves (data_a0/b0/a1/b1) as rhd2164_controller's
		ch_cnt walks the sampling frame, latches them at the frame boundary
		(ch_is_0_redge) for a clean simultaneous {ch0,ch1} sample, and gates
		fifo_wen with stream_enable. Ports shaped by "two RHD2164s, two
		outputs each" — PLAN.md B.3 flags this AFE-topology coupling as
		worth decoupling into a generic {channel,sample,valid} source later.
*********************************************************************************/

module ch_sel #(
	// localparam, not a real parameter (2026-08-26 parametrization
	// review): dout (2*n wide) feeds fifo_din_mux.v, whose ch_sel_dout/
	// dout_test/fifo_din ports are all hardcoded [31:0], and
	// test_pattern_gen.v's dout_test is hardcoded [31:0] too — neither
	// has a parameter of its own. fifo itself is also explicitly
	// instantiated with DATA_WIDTH=32 in kuntur_fpga.v. Changing n here
	// alone would silently stop matching every one of those. Kept in the
	// #(...) header (rather than a body-level localparam) only because
	// Verilog requires it declared before use in the port widths below.
	localparam n = 16
	)(
	input  wire	rstb,					// Active-low asyncrhonous reset
	input  wire	clk,					// Input clock
	input  wire [1:0]	mode,			// Input; operation mode
	input  wire [5:0]	ch_cnt,			// Input; channel count
	input  wire [n-1:0]	data_a0,		// Input; 16-bit data coming from RHD2164-0 output a
	input  wire [n-1:0]	data_b0,		// Input; 16-bit data coming from RHD2164-0 output b
	input  wire [n-1:0]	data_a1,		// Input; 16-bit data coming from RHD2164-1 output a
	input  wire [n-1:0]	data_b1,		// Input; 16-bit data coming from RHD2164-1 output b
	output wire	[2*n-1:0]	dout,			// Output Data
	// Channel Config
	input  wire	[7:0]	ch_a,		// ChA
	input  wire	[7:0]	ch_b,		// ChB
	// TX FIFO
	output reg	fifo_wen,				// Output; FIFO write enable
	input wire stream_enable,
	// debug
	output reg	dout_en_0,
	output wire	trig
);
	
	// wire & regs
	reg [n-1:0] data0, data1, data0_synced, data1_synced;
	wire ch_is_a, ch_is_b, ch_is_0, ch_is_16;
	reg ch_is_a_reg, ch_is_b_reg, ch_is_0_reg, ch_is_16_reg;
	wire ch_is_a_redge, ch_is_b_redge, ch_is_0_redge, ch_is_16_redge;
	
	reg [15:0] cnt1;			// debug trigger counter, width independent of n
	reg [n-1:0] ch0, ch1;		// was hardcoded [15:0] regardless of n — bug,
								// silently mismatched dout's [2*n-1:0] for any n != 16
	
	// Comparator
	assign ch_is_a = (ch_cnt == ch_a[5:0]);
	assign ch_is_b = (ch_cnt == ch_b[5:0]);
	assign ch_is_0 = (ch_cnt == 0);
	assign ch_is_16 = (ch_cnt == 6'd16);

	// Rising edge detector sets 'fifo_wen'
	assign ch_is_a_redge = (ch_is_a) & (~ch_is_a_reg);
	assign ch_is_b_redge = (ch_is_b) & (~ch_is_b_reg);
	assign ch_is_0_redge = (ch_is_0) & (~ch_is_0_reg);
	assign ch_is_16_redge = (ch_is_16) & (~ch_is_16_reg);

	always@(negedge rstb or posedge clk)
		if (!rstb) begin
			ch_is_a_reg <= 0;
			ch_is_b_reg <= 0;
			ch_is_0_reg <= 0;
			ch_is_16_reg <= 0;
			fifo_wen <= 0;
			dout_en_0 <= 0;
		end
		else begin
			ch_is_a_reg <= ch_is_a;
			ch_is_b_reg <= ch_is_b;
			ch_is_0_reg <= ch_is_0;
			ch_is_16_reg <= ch_is_16;
			dout_en_0 <= ch_is_0_redge;
			//fifo_wen <= dout_en_0 | dout_en_16;
			fifo_wen <= dout_en_0 & stream_enable;
		end


	// channel selection
	always@(negedge rstb or posedge clk)
		if (!rstb) begin
			data0 <= 0;
			data1 <= 0;
		end
		else begin
			if ( ch_is_a_redge )
				case (ch_a[7:6])
					2'd0: data0 <= data_a0;
					2'd1: data0 <= data_b0;
					2'd2: data0 <= data_a1;
					2'd3: data0 <= data_b1;
					default: data0 <= 0;
				endcase
			if ( ch_is_b_redge )
				case (ch_b[7:6])
					2'd0: data1 <= data_a0;
					2'd1: data1 <= data_b0;
					2'd2: data1 <= data_a1;
					2'd3: data1 <= data_b1;
					default: data1 <= 0;
				endcase
		end
	
	// Frame synchronization
	always@(negedge rstb or posedge clk)
		if (!rstb) begin
			data0_synced <= 0;
			data1_synced <= 0;
		end
		else if (ch_is_0_redge) begin
			data0_synced <= data0;
			data1_synced <= data1;
		end
	
	
	// Adding delay to sample after frame synchronization
	always@(negedge rstb or posedge clk)
		if (!rstb) begin
			ch0 <= 0;
			ch1 <= 0;
		end
		else if (dout_en_0) begin
			ch0 <= data0_synced;
			ch1 <= data1_synced;
		end
	
	assign dout = {ch0, ch1};


	// For debugging - Begin
	always@(negedge rstb or posedge clk)
		if (!rstb)
			cnt1 <= 0;
		else if (dout_en_0)
			cnt1 <= 0;
		else
			cnt1 <= cnt1 + 1;
	
	assign trig = (cnt1 > 16'd2000) ? 1:0;
	// assign trig = cnt1[0];
	// End
			
endmodule
