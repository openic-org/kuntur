/***********************************************************
	Author: Manuel Monge
	Description:
		SPI-Master core with n-bits tx/rx registers.
***********************************************************/

module spi_master #(
	// n is a localparam, not a real parameter: spi_master_controller_std
	// hand-unrolls exactly 16 SCK states (sck0b..sck15d), so n isn't
	// actually free to change without rewriting that FSM. Kept here
	// (rather than as a body-level localparam) only because Verilog
	// requires it declared before use in the port widths below.
	// m was unused everywhere — deleted rather than documented.
	localparam n = 16
	)(
	input  wire			rstb,		// Active-low asyncrhonous reset
	input  wire			clk,		// Input clock
	input  wire			start,		// Active-high signal that starts SPI cycle
	output wire			csb,		// Active-low chip-select
	output wire			sck,		// SPI Clock
	output wire			mosi,		// Master-Output Slave-Input
	input  wire			miso,		// Master-Input Slave-Output
	input  wire	[n-1:0]	data_tx,	// Data to be transmitted
	output wire	[n-1:0]	data_rx,	// DataA received
	output wire			done		// TX/RX operation completed; no communicacion happening
);
	
	// wire & regs
	wire rx_a_en,rx_a_load,tx_en,tx_load;
	
	// TX and RX shift-registers and registers
	sr_s2p #(.n(n)) rxsr (
		.rstb	(rstb),
		.clk	(clk),
		.en		(rx_a_en),	// RX_b is aligned with SCK rising edge; sampling at falling edge
		.load	(rx_a_load),
		.a		(miso),
		.data	(data_rx)
	);
		
	sr_p2s #(.n(n)) txsr (
		.rstb	(rstb),
		.clk	(clk),
		.en		(tx_en),
		.load	(tx_load),
		.data	(data_tx),
		.a		(mosi)
	);

	
	// FSM Controller
	spi_master_controller_std spi_master_controller0 (
		.rstb		(rstb),
		.clk		(clk),
		.start		(start),
		.csb		(csb),
		.sck		(sck),
		.rx_a_en	(rx_a_en),
		.rx_a_load	(rx_a_load),
		.rx_b_en	(),
		.rx_b_load	(),
		.tx_en		(tx_en),
		.tx_load	(tx_load),
		.done		(done)
	);
	

endmodule


/*********************************************************************************
	Author: Manuel Monge
	Description:
		SPI-Master controller standard.
*********************************************************************************/

module spi_master_controller_std (
	input  wire	rstb,	// Active-low asynchronouse reset
	input  wire	clk,	// Input clock < 48 MHz
	input  wire	start,	// Start signal to begin 1x SPI cycle
	output reg	csb,	// CSb
	output reg	sck,	// SCK
	output reg	rx_a_en,	// rxRegA Enable
	output reg	rx_a_load,	// rxRegA Load signal
	output reg	rx_b_en,	// rxRegB Enable
	output reg	rx_b_load,	// rxRegB Load signal
	output reg	tx_en,		// txReg Enable
	output reg	tx_load,	// txReg Load signal
	output reg	done		// Active high, indicates SPI cycle completed
);
	
	// state
	localparam idle = 6'd0;
	localparam op0 = 6'd1;
	localparam op1 = 6'd2;
	localparam sck0b = 6'd3, sck0d = 6'd4;
	localparam sck1b = 6'd5, sck1d = 6'd6;
	localparam sck2b = 6'd7, sck2d = 6'd8;
	localparam sck3b = 6'd9, sck3d = 6'd10;
	localparam sck4b = 6'd11, sck4d = 6'd12;
	localparam sck5b = 6'd13, sck5d = 6'd14;
	localparam sck6b = 6'd15, sck6d = 6'd16;
	localparam sck7b = 6'd17, sck7d = 6'd18;
	localparam sck8b = 6'd19, sck8d = 6'd20;
	localparam sck9b = 6'd21, sck9d = 6'd22;
	localparam sck10b = 6'd23, sck10d = 6'd24;
	localparam sck11b = 6'd25, sck11d = 6'd26;
	localparam sck12b = 6'd27, sck12d = 6'd28;
	localparam sck13b = 6'd29, sck13d = 6'd30;
	localparam sck14b = 6'd31, sck14d = 6'd32;
	localparam sck15b = 6'd33, sck15d = 6'd34;
	localparam csbend0 = 6'd35;
	localparam csbend1 = 6'd36;
	localparam csbend2 = 6'd37;
	localparam csbend3 = 6'd38;
	localparam csbend4 = 6'd39;
	localparam csbend5 = 6'd40;
	localparam csbend6 = 6'd41;
	localparam csbend7 = 6'd42;
	reg [5:0] current_state,next_state;
		
	
		
	// state register
	always@(negedge rstb or posedge clk)
		if (!rstb)
			current_state <= idle;
		else
			current_state <= next_state;
	
	// next_state logic
	always@(*) begin
		// default values
		rx_a_en = 1'b0;
		rx_a_load = 1'b0;
		rx_b_en = 1'b0;
		rx_b_load = 1'b0;
		tx_en = 1'b0;
		tx_load = 1'b0;
		done = 1'b0;
		csb = 1'b1;
		sck = 1'b0;
		
		case (current_state)
			idle: begin
				if (start)
					next_state = op0;
				else
					next_state = idle;
				done = 1'b1;
			end
			op0: begin
				next_state = op1;
				// load txreg
				tx_load = 1'b1;
				// csb
				csb = 1'b0;
			end
			op1: begin
				next_state = sck0b;
				// csb
				csb = 1'b0;
			end
			
			/*****************************
			** SCK0
			*****************************/
			sck0b: begin
				next_state = sck0d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_a_en
				rx_a_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck0d: begin
				next_state = sck1b;
				// csb
				csb = 1'b0;
				// rx_b_en
				rx_b_en = 1'b1;
			end
			
			/*****************************
			** SCK1
			*****************************/
			sck1b: begin
				next_state = sck1d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_a_en
				rx_a_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck1d: begin
				next_state = sck2b;
				// csb
				csb = 1'b0;
				// rx_b_en
				rx_b_en = 1'b1;
			end
			
			/*****************************
			** SCK2
			*****************************/
			sck2b: begin
				next_state = sck2d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_a_en
				rx_a_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck2d: begin
				next_state = sck3b;
				// csb
				csb = 1'b0;
				// rx_b_en
				rx_b_en = 1'b1;
			end
			
			/*****************************
			** SCK3
			*****************************/
			sck3b: begin
				next_state = sck3d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_a_en
				rx_a_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck3d: begin
				next_state = sck4b;
				// csb
				csb = 1'b0;
				// rx_b_en
				rx_b_en = 1'b1;
			end
			
			/*****************************
			** SCK4
			*****************************/
			sck4b: begin
				next_state = sck4d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_a_en
				rx_a_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck4d: begin
				next_state = sck5b;
				// csb
				csb = 1'b0;
				// rx_b_en
				rx_b_en = 1'b1;
			end
			
			/*****************************
			** SCK5
			*****************************/
			sck5b: begin
				next_state = sck5d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_a_en
				rx_a_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck5d: begin
				next_state = sck6b;
				// csb
				csb = 1'b0;
				// rx_b_en
				rx_b_en = 1'b1;
			end
			
			/*****************************
			** SCK6
			*****************************/
			sck6b: begin
				next_state = sck6d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_a_en
				rx_a_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck6d: begin
				next_state = sck7b;
				// csb
				csb = 1'b0;
				// rx_b_en
				rx_b_en = 1'b1;
			end
			
			/*****************************
			** SCK7
			*****************************/
			sck7b: begin
				next_state = sck7d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_a_en
				rx_a_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck7d: begin
				next_state = sck8b;
				// csb
				csb = 1'b0;
				// rx_b_en
				rx_b_en = 1'b1;
			end
			
			/*****************************
			** SCK8
			*****************************/
			sck8b: begin
				next_state = sck8d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_a_en
				rx_a_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck8d: begin
				next_state = sck9b;
				// csb
				csb = 1'b0;
				// rx_b_en
				rx_b_en = 1'b1;
			end
			
			/*****************************
			** SCK9
			*****************************/
			sck9b: begin
				next_state = sck9d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_a_en
				rx_a_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck9d: begin
				next_state = sck10b;
				// csb
				csb = 1'b0;
				// rx_b_en
				rx_b_en = 1'b1;
			end
			
			/*****************************
			** SCK10
			*****************************/
			sck10b: begin
				next_state = sck10d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_a_en
				rx_a_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck10d: begin
				next_state = sck11b;
				// csb
				csb = 1'b0;
				// rx_b_en
				rx_b_en = 1'b1;
			end
			
			/*****************************
			** SCK11
			*****************************/
			sck11b: begin
				next_state = sck11d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_a_en
				rx_a_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck11d: begin
				next_state = sck12b;
				// csb
				csb = 1'b0;
				// rx_b_en
				rx_b_en = 1'b1;
			end
			
			/*****************************
			** SCK12
			*****************************/
			sck12b: begin
				next_state = sck12d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_a_en
				rx_a_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck12d: begin
				next_state = sck13b;
				// csb
				csb = 1'b0;
				// rx_b_en
				rx_b_en = 1'b1;
			end
			
			/*****************************
			** SCK13
			*****************************/
			sck13b: begin
				next_state = sck13d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_a_en
				rx_a_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck13d: begin
				next_state = sck14b;
				// csb
				csb = 1'b0;
				// rx_b_en
				rx_b_en = 1'b1;
			end
			
			/*****************************
			** SCK14
			*****************************/
			sck14b: begin
				next_state = sck14d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_a_en
				rx_a_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck14d: begin
				next_state = sck15b;
				// csb
				csb = 1'b0;
				// rx_b_en
				rx_b_en = 1'b1;
			end
			
			/*****************************
			** SCK15
			*****************************/
			sck15b: begin
				next_state = sck15d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_a_en
				rx_a_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck15d: begin
				next_state = csbend0;
				// csb
				csb = 1'b0;
				// rx_b_en
				rx_b_en = 1'b1;
			end
			
			/*****************************
			** END-Section
			*****************************/
			//csbend0: next_state = csbend1;
			csbend0: begin
				next_state = csbend2;
				// load rxreg
				rx_a_load = 1'b1;
				rx_b_load = 1'b1;
			end
			csbend2: next_state = csbend3;
			csbend3: next_state = csbend4;
			csbend4: next_state = csbend5;
			csbend5: next_state = csbend6;
			csbend6: next_state = csbend7;
			csbend7: next_state = idle;
			default: begin
				next_state = idle;
			end
		endcase
	end
	
endmodule
