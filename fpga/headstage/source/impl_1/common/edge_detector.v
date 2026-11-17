/*********************************************************************************
	Author: Manuel Monge
	Description:
		Rising and falling edge detector using a[n-1] and a[n-2].
*********************************************************************************/

module edge_detector	(
	input  wire	rstb,	// Active-low asyncrhonous reset
	input  wire	clk,	// Input clock
	input  wire	a,		// Input signal
	output wire	redge,	// Pulses high when there is a rising edge in 'a'.
	output wire	fedge	// Pulses high when there is a falling edge in 'a'.
);
	
	// wire & regs
	reg areg0,areg1;
	
	// rising edge detector
	assign redge = (areg0)&(~areg1);
	
	// falling edge detector
	assign fedge = (~areg0)&(areg1);
	
	// registers
	always@(negedge rstb or posedge clk)
		if (!rstb) begin
			areg1 <= 0;
			areg0 <= 0;
		end
		else begin
			areg1 <= areg0;
			areg0 <= a;
		end
	
endmodule
