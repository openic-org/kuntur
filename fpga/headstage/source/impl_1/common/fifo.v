/*********************************************************************************
	Author: Manuel Monge
	Description:
		Generic circular-buffer FIFO, parametric width/depth. Standard
		wrap-around in_ptr/out_ptr with an extra MSB to disambiguate empty
		from full at the same address. Underrun (ren while empty) returns
		the sentinel 0x8000 on both halves rather than stale data — see
		the project-wide underrun-sentinel convention this depends on.
		Used as the main sample FIFO between ch_sel and SPI0; no RHD2164
		or Kuntur-specific knowledge here, genuinely reusable.
*********************************************************************************/

module fifo #(
	parameter DATA_WIDTH = 16,
	parameter ADDR_WIDTH = 16
	)(
	input  wire	rstb,					// Active-low asyncrhonous reset
	input  wire	clk,					// Input clock
	input  wire wen,						// Input; FIFO write enable
	input  wire ren,						// Input; FIFO read enable
	input  wire [DATA_WIDTH-1:0]	DIN,	// Input Data
	output reg	[DATA_WIDTH-1:0] DOUT,	// Output Data
	output wire	empty,					// Output flag indicating empty FIFO
	output wire	full,					// Output flag indicating full FIFO
	output wire dbg0,
	output wire dbg1,
	output wire trig
);
	
	// memory
	reg [DATA_WIDTH-1:0] fifo_ram [0:(2**ADDR_WIDTH)-1];

	// wire & regs
	reg [ADDR_WIDTH:0] in_ptr, out_ptr; // PTRs with additional bit as flag
	reg [15:0] cnt0;
	reg [17:0] cnt1;
	
	
	// for debugging only
	assign dbg0 = in_ptr[0];
	assign dbg1 = out_ptr[0];
	
	always@(negedge rstb or posedge clk)
		if (!rstb)
			cnt1 <= 0;
		else if (ren)
			cnt1 <= 0;
		else
			cnt1 <= cnt1 + 1;
	
	assign trig = (cnt1 > 16'd1000) ? 1:0;
	// end
	
	// Writing
	always@(negedge rstb or posedge clk)
		if (!rstb)
			in_ptr <= 0;
		else if (wen && !full) begin
			fifo_ram[in_ptr[ADDR_WIDTH-1:0]] <= DIN;
			in_ptr <= in_ptr + 1'b1;
		end
		//else if (full) begin // if FIFO full -> drop all content
			//in_ptr <= out_ptr;
		//end
		
	// Reading
	always@(negedge rstb or posedge clk)
		if (!rstb) begin
			out_ptr <= 0;
			DOUT <= 0;
		end
		else if (ren && !empty) begin
			out_ptr <= out_ptr + 1'b1;
			DOUT <= fifo_ram[out_ptr[ADDR_WIDTH-1:0]];
		end
		else if (ren && empty) begin // if FIFO empty -> output 0x8000
			DOUT <= {2{16'h8000}};
		end
	
		
	// Status Flags
	assign empty = (in_ptr == out_ptr) ? 1:0; // both pointers are the same
	assign full = ((in_ptr[ADDR_WIDTH-1:0] == out_ptr[ADDR_WIDTH-1:0]) & (in_ptr[ADDR_WIDTH] != out_ptr[ADDR_WIDTH])) ? 1:0; // pointers have the same [ADDR_WIDTH-1:0] value but the MSBs are different
	
endmodule
