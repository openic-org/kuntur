/***********************************************************
	Author: Manuel Monge
	Description:
		SPI-Master RHD2164x2 core with 16-bits tx/rx registers.
***********************************************************/

module spi_master_rhd2164x2 #(
	// n is a localparam, not a real parameter: spi_master_controller
	// hand-unrolls exactly 16 SCK states (sck0b..sck15d), so n isn't
	// actually free to change without rewriting that FSM — and this is
	// the real, timing-critical, hardware-verified RHD2164 SPI1 link
	// fpga-timing-constraints.md's csb/rx_a_en/rx_b_en analysis was
	// derived against, so that FSM is deliberately not being touched.
	// Kept here (rather than as a body-level localparam) only because
	// Verilog requires it declared before use in the port widths below.
	// m was unused everywhere — deleted rather than documented.
	localparam n = 16
	)(
	input  wire			rstb,		// Active-low asyncrhonous reset
	input  wire			clk,		// Input clock
	input  wire			clk90,		// Input clock
	input  wire			start,		// Active-high signal that starts SPI cycle
	output wire			csb,		// Active-low chip-select
	output wire			sck_reg,		// SPI Clock
	output wire			mosi_reg,		// Master-Output Slave-Input
	input  wire			miso,		// Master-Input Slave-Output
	input  wire			miso1,		// Master-Input Slave-Output 1
	input  wire	[n-1:0]	data_tx,	// Data to be transmitted
	output wire	[n-1:0]	data_rx_a,	// DataA received
	output wire	[n-1:0]	data_rx_b,	// DataB received
	output wire	[n-1:0]	data_rx_a1,	// DataA1 received
	output wire	[n-1:0]	data_rx_b1,	// DataB1 received
	output wire			done,		// TX/RX operation completed; no communicacion happening
	output wire			a_next
);
	
	// wire & regs
	wire rx_a_en,rx_a_load,rx_b_en,rx_b_load,tx_en,tx_load;
	wire sck, mosi;
	
	// TX and RX shift-registers and registers
	sr_s2p #(.n(n)) rxsr_a0 (
		.rstb	(rstb),
		.clk	(clk),
		.en		(rx_a_en),
		.load	(rx_a_load),
		.a		(miso),
		.data	(data_rx_a)
	);
	
	sr_s2p #(.n(n)) rxsr_b0 (
		.rstb	(rstb),
		.clk	(clk),
		.en		(rx_b_en),
		.load	(rx_b_load),
		.a		(miso),
		.data	(data_rx_b)
	);
	
	sr_s2p #(.n(n)) rxsr_a1 (
		.rstb	(rstb),
		.clk	(clk),
		.en		(rx_a_en),
		.load	(rx_a_load),
		.a		(miso1),
		.data	(data_rx_a1)
	);
	
	sr_s2p #(.n(n)) rxsr_b1 (
		.rstb	(rstb),
		.clk	(clk),
		.en		(rx_b_en),
		.load	(rx_b_load),
		.a		(miso1),
		.data	(data_rx_b1)
	);
	
	sr_p2s_1 #(.n(n)) txsr (
		.rstb	(rstb),
		.clk	(clk),
		.en		(tx_en),
		.load	(tx_load),
		.data	(data_tx),
		.a		(mosi),
		.a_next	(a_next)
	);

	
	// FSM Controller
	spi_master_controller spi_master_controller0 (
		.rstb		(rstb),
		.clk		(clk),
		.start		(start),
		.csb_reg		(csb),
		.sck		(sck),
		.rx_a_en_reg	(rx_a_en),
		.rx_a_load	(rx_a_load),
		.rx_b_en_reg	(rx_b_en),
		.rx_b_load	(rx_b_load),
		.tx_en		(tx_en),
		.tx_load	(tx_load),
		.done		(done)
	);
	
	
	// registers
	
	assign sck_reg = sck;
	assign mosi_reg = mosi;
	
	//always @(negedge rstb or posedge clk)
		//if (!rstb)
			//sck_reg <= 1'b0;
		//else
			//sck_reg <= sck;      // existing combinational sck from spi_master_controller

	//always @(negedge rstb or negedge clk90)
		//if (!rstb)
			//mosi_reg <= 1'b0;
		//else
			//mosi_reg <= mosi;    // existing sr_p2s output (areg[n-1])
	

endmodule


/*********************************************************************************
	Author: Manuel Monge
	Description:
		SPI-Master controller for RHD2164. SCK Max. Freq. = 24 MHz.
*********************************************************************************/

module spi_master_controller (
	input  wire	rstb,	// Active-low asynchronouse reset
	input  wire	clk,	// Input clock < 48 MHz
	input  wire	start,	// Start signal to begin 1x SPI cycle
	output reg	csb_reg,	// CSb
	output reg	sck,	// SCK
	output reg	rx_a_en_reg,	// rxRegA Enable
	output reg	rx_a_load,	// rxRegA Load signal
	output reg	rx_b_en_reg,	// rxRegB Enable
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
	
	reg rx_a_en, rx_b_en, csb;
	
		
	// pipeline registers - Enables need to be ready one cycle prior
	always@(negedge rstb or posedge clk)
		if (!rstb) begin
			rx_a_en_reg <= 1'b0;
			rx_b_en_reg <= 1'b0;
			csb_reg <= 1'b0;
		end
		else begin
			rx_a_en_reg <= rx_a_en;
			rx_b_en_reg <= rx_b_en;
			csb_reg <= csb;
		end
	
	
	
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
				if (start) begin
					next_state = op0;
					// csb
					csb = 1'b0;
				end
				else begin
					next_state = idle;
				end
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
				// rx_a_en
				rx_a_en = 1'b1;
			end
			
			/*****************************
			** SCK0
			*****************************/
			sck0b: begin
				next_state = sck0d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_b_en
				rx_b_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck0d: begin
				next_state = sck1b;
				// csb
				csb = 1'b0;
				// rx_a_en
				rx_a_en = 1'b1;
			end
			
			/*****************************
			** SCK1
			*****************************/
			sck1b: begin
				next_state = sck1d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_b_en
				rx_b_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck1d: begin
				next_state = sck2b;
				// csb
				csb = 1'b0;
				// rx_a_en
				rx_a_en = 1'b1;
			end
			
			/*****************************
			** SCK2
			*****************************/
			sck2b: begin
				next_state = sck2d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_b_en
				rx_b_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck2d: begin
				next_state = sck3b;
				// csb
				csb = 1'b0;
				// rx_a_en
				rx_a_en = 1'b1;
			end
			
			/*****************************
			** SCK3
			*****************************/
			sck3b: begin
				next_state = sck3d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_b_en
				rx_b_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck3d: begin
				next_state = sck4b;
				// csb
				csb = 1'b0;
				// rx_a_en
				rx_a_en = 1'b1;
			end
			
			/*****************************
			** SCK4
			*****************************/
			sck4b: begin
				next_state = sck4d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_b_en
				rx_b_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck4d: begin
				next_state = sck5b;
				// csb
				csb = 1'b0;
				// rx_a_en
				rx_a_en = 1'b1;
			end
			
			/*****************************
			** SCK5
			*****************************/
			sck5b: begin
				next_state = sck5d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_b_en
				rx_b_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck5d: begin
				next_state = sck6b;
				// csb
				csb = 1'b0;
				// rx_a_en
				rx_a_en = 1'b1;
			end
			
			/*****************************
			** SCK6
			*****************************/
			sck6b: begin
				next_state = sck6d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_b_en
				rx_b_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck6d: begin
				next_state = sck7b;
				// csb
				csb = 1'b0;
				// rx_a_en
				rx_a_en = 1'b1;
			end
			
			/*****************************
			** SCK7
			*****************************/
			sck7b: begin
				next_state = sck7d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_b_en
				rx_b_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck7d: begin
				next_state = sck8b;
				// csb
				csb = 1'b0;
				// rx_a_en
				rx_a_en = 1'b1;
			end
			
			/*****************************
			** SCK8
			*****************************/
			sck8b: begin
				next_state = sck8d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_b_en
				rx_b_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck8d: begin
				next_state = sck9b;
				// csb
				csb = 1'b0;
				// rx_a_en
				rx_a_en = 1'b1;
			end
			
			/*****************************
			** SCK9
			*****************************/
			sck9b: begin
				next_state = sck9d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_b_en
				rx_b_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck9d: begin
				next_state = sck10b;
				// csb
				csb = 1'b0;
				// rx_a_en
				rx_a_en = 1'b1;
			end
			
			/*****************************
			** SCK10
			*****************************/
			sck10b: begin
				next_state = sck10d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_b_en
				rx_b_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck10d: begin
				next_state = sck11b;
				// csb
				csb = 1'b0;
				// rx_a_en
				rx_a_en = 1'b1;
			end
			
			/*****************************
			** SCK11
			*****************************/
			sck11b: begin
				next_state = sck11d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_b_en
				rx_b_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck11d: begin
				next_state = sck12b;
				// csb
				csb = 1'b0;
				// rx_a_en
				rx_a_en = 1'b1;
			end
			
			/*****************************
			** SCK12
			*****************************/
			sck12b: begin
				next_state = sck12d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_b_en
				rx_b_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck12d: begin
				next_state = sck13b;
				// csb
				csb = 1'b0;
				// rx_a_en
				rx_a_en = 1'b1;
			end
			
			/*****************************
			** SCK13
			*****************************/
			sck13b: begin
				next_state = sck13d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_b_en
				rx_b_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck13d: begin
				next_state = sck14b;
				// csb
				csb = 1'b0;
				// rx_a_en
				rx_a_en = 1'b1;
			end
			
			/*****************************
			** SCK14
			*****************************/
			sck14b: begin
				next_state = sck14d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_b_en
				rx_b_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck14d: begin
				next_state = sck15b;
				// csb
				csb = 1'b0;
				// rx_a_en
				rx_a_en = 1'b1;
			end
			
			/*****************************
			** SCK15
			*****************************/
			sck15b: begin
				next_state = sck15d;
				// csb, sck
				csb = 1'b0;
				sck = 1'b1;
				// rx_b_en
				rx_b_en = 1'b1;
				// tx_en
				tx_en = 1'b1;
			end
			sck15d: begin
				next_state = csbend0;
				// csb
				// csb = 1'b0;
				// rx_a_en
				// rx_a_en = 1'b1;
			end
			
			/*****************************
			** END-Section
			*****************************/
			csbend0: next_state = csbend1;
			csbend1: begin
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


/*********************************************************************************
	Author: Manuel Monge
	Description:
		n-bits shift register parallel to serial converter with synchronous load and en.
		This is "sr_p2s" plus "a_next".
*********************************************************************************/

module sr_p2s_1 #(
	parameter n = 16	// # of bits of shift register
	)(
	input  wire			rstb,	
	input  wire			clk,	// Input clock
	input  wire			en,		// Shigt-register enable
	input  wire			load,	// Shift-register load signal to parallel register
	input  wire	[n-1:0] data,	// n-bit input signal
	output wire			a,		// serial output
	output wire			a_next
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
	
	assign a_next = (load)	? data[n-1] :
					(en)	? areg[n-2] : areg[n-1];
	
endmodule