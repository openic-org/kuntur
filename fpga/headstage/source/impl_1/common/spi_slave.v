/***********************************************************
	Author: Manuel Monge
	Description:
		SPI-Slave core with n-bits tx/rx registers.
***********************************************************/

module spi_slave #(
	parameter n = 16,	// n = 2^m bits to be transmitted/received
	parameter m = 4	// n = 2^m bits to be transmitted/received
	)(
	input  wire			rstb,		// Active-low asyncrhonous reset
	input  wire			clk,		// Input clock
	input  wire			csb,		// Active-low chip-select
	input  wire			sck,		// SPI Clock
	input  wire			mosi,		// Master-Output Slave-Input
	output wire			miso,		// Master-Input Slave-Output
	input  wire [n-1:0]	data_tx,	// Data to be transmitted
	output wire [n-1:0]	data_rx,	// Data received
	output wire			done		// TX/RX operation completed; no communicacion happening
);
		
	// wire & regs
	wire csb_redge,csb_fedge;
	wire sck_redge,sck_fedge;
	wire rx_en,rx_load,tx_en,tx_load;
	
	// TX and RX shift-registers and registers
	sr_s2p #(.n(n)) rxsr (
		.rstb	(rstb),
		.clk	(clk),
		.en		(rx_en),
		.load	(rx_load),
		.a		(mosi),
		.data	(data_rx)
	);
	
	sr_p2s #(.n(n)) txsr (
		.rstb	(rstb),
		.clk	(clk),
		.en		(tx_en),
		.load	(tx_load),
		.data	(data_tx),
		.a		(miso)
	);

	
	// edge detectors
	edge_detector edgedet_csb (
		.rstb	(rstb),
		.clk	(clk),
		.a		(csb),
		.redge	(csb_redge),
		.fedge	(csb_fedge)
		);
		
	edge_detector edgedet_sck (
		.rstb	(rstb),
		.clk	(clk),
		.a		(sck),
		.redge	(sck_redge),
		.fedge	(sck_fedge)
		);
	
	
	// FSM Controller
	spi_slave_controller #(.n(m)) spi_controller0 (
		.rstb		(rstb),
		.clk		(clk),
		.csb_redge	(csb_redge),
		.csb_fedge	(csb_fedge),
		.sck_redge	(sck_redge),
		.sck_fedge	(sck_fedge),
		.rx_en		(rx_en),
		.rx_load	(rx_load),
		.tx_en		(tx_en),
		.tx_load	(tx_load),
		.done		(done)
	);

endmodule


/*********************************************************************************
	Author: Manuel Monge
	Description:
		SPI-Slave controller.
*********************************************************************************/

module spi_slave_controller #(
	parameter n = 4			// Indicates 2**n bits to be transmitted/received
	)(
	input  wire	rstb,			// Active Low, Asynchronous reset
	input  wire	clk,			// input clock
	input  wire	csb_redge,	// Active high rising edge detector
	input  wire	csb_fedge,	// Active high falling edge detector
	input  wire	sck_redge,	// Active high risiing edge detector
	input  wire	sck_fedge,	// Active high falling edge detector
	output reg	rx_en,		// rxReg Enable
	output reg	rx_load,		// rxReg Load signal
	output reg	tx_en,		// txReg Enable
	output reg	tx_load,		// txReg Load signal
	output reg	done			// Active high, indicates SPI cycle completed
);
	
	// state
	localparam idle = 3'd0;
	localparam op0 = 3'd1;
	localparam op1 = 3'd2;
	localparam op2 = 3'd3;
	localparam op3 = 3'd4;
	localparam op4 = 3'd5;
	reg [2:0] current_state,next_state;
	
	// wire & regs
	reg [n-1:0] cnt;
	reg cnt_load,cnt_en;
	wire cnt_is_0;
	
	
	// n-bit counter
	always@(negedge rstb or posedge clk)
		if (!rstb)
			cnt <= 0;
		else if (cnt_load)
			cnt <= {n{1'b1}};
		else if (cnt_en)
			cnt <= cnt - 1'b1;
	
	// comparator to zero
	assign cnt_is_0 = (cnt == 0) ? 1:0;
	
	// state register
	always@(negedge rstb or posedge clk)
		if (!rstb)
			current_state <= idle;
		else
			current_state <= next_state;
	
	// next_state logic
	always@(*) begin
		// default values
		rx_en = 1'b0;
		rx_load = 1'b0;
		tx_en = 1'b0;
		tx_load = 1'b0;
		cnt_load = 1'b0;
		cnt_en = 1'b0;
		done = 1'b0;
		
		case (current_state)
			idle: begin
				if (csb_fedge) begin
					next_state = op1;
					// load txreg
					tx_load = 1'b1;
					// start cnt
					cnt_load = 1'b1;
				end
				else
					next_state = idle;
				done = 1'b1;
			end
//			op0: begin
//				next_state = op1;
//				 //load txreg
//				tx_load = 1'b1;
//				 //start cnt
//				cnt_load = 1'b1;
//			end
			op1: begin
				if (sck_redge) begin
					next_state = op2;
					rx_en = 1'b1;
				end
				else begin
					next_state = op1;
					rx_en = 1'b0;
				end
			end
			op2: begin
				if (sck_fedge) begin
					next_state = op3;
					tx_en = 1'b1;
				end
				else begin
					next_state = op2;
					tx_en = 1'b0;
				end
			end
			op3: begin
				if (cnt_is_0) begin
					next_state = idle;
					// load rxreg
					rx_load = 1'b1;
				end
				else
					next_state = op1;
				// update cnt
				cnt_en = 1'b1;
			end
//			op4: begin
//				next_state = idle;
//				// load rxreg
//				rx_load = 1'b1;
//			end
			default: begin
				next_state = idle;
			end
		endcase
	end
	
	
endmodule
