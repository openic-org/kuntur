/*********************************************************************************
	Author: Manuel Monge
	Description:
		SPI0 TX-side source mux/register — selects what main_controller's
		mcu_dtx_sel points at (FIFO ChA, FIFO ChB, or regbank readback) and
		latches it on mcu_dtx_en for the MCU to shift out next.
*********************************************************************************/

module dtx_mux_reg (
	input  wire	rstb,			// Active-low asyncrhonous reset
	input  wire	clk,			// Input clock
	input  wire	[1:0]	mcu_dtx_sel,
	input  wire	mcu_dtx_en,
	input  wire	[31:0]	din,	// Data input
	input  wire	[15:0]	ram_din,	// SRAM Data input
	output reg	[15:0]	dout	// Data output
);

	reg [15:0] dtx;
	
	// mux
	always@(*) begin
		case (mcu_dtx_sel)
			2'd0: dtx <= din[31:16];
			2'd1: dtx <= din[15:0];
			2'd2: dtx <= ram_din[15:0];
			default: dtx <= din[31:16];
		endcase
	end
	
	always@(negedge rstb or posedge clk)
		if (!rstb)
			dout <= 0;
		else if (mcu_dtx_en)
			dout <= dtx;
	
endmodule
