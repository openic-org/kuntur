/*********************************************************************************
	Author: Manuel Monge
	Description:
		n-bits shift register serial to parallel converter with synchronous load and en.
*********************************************************************************/

module sr_s2p #(
	parameter n = 16	// # of bits of shift register
	)(
	input  wire			rstb,	// Active-low asyncrhonous reset
	input  wire			clk,	// Input clock
	input  wire			en,		// Shigt-register enable
	input  wire			load,	// Shigt-register load
	input  wire			a,		// Serial input signal
	output reg	[n-1:0]	data	// n-bit Parallel output data
);
	
	// wire & regs
	reg [n-1:0] areg;
	
	always@(negedge rstb or posedge clk)
		if (!rstb) begin
			areg <= 0;
			data <= 0;
		end
		else if (en)
			areg <= {areg[n-2:0],a};
		else if (load)
			data <= areg;
	
endmodule


/*********************************************************************************
	Author: Manuel Monge
	Description:
		n-bits shift register parallel to serial converter with synchronous load and en.
*********************************************************************************/

module sr_p2s #(
	parameter n = 16	// # of bits of shift register
	)(
	input  wire			rstb,	
	input  wire			clk,	// Input clock
	input  wire			en,		// Shigt-register enable
	input  wire			load,	// Shift-register load signal to parallel register
	input  wire	[n-1:0] data,	// n-bit input signal
	output wire			a		// serial output
);
	
	// wire & regs
	reg [n-1:0] areg;
	
	always@(negedge rstb or posedge clk)
		if (!rstb)
			areg <= 0;
		else if (load)
			areg <= data;
		else if (en)
			areg <= {areg[n-2:0],1'b0};
	
	assign a = areg[n-1];
	
endmodule
