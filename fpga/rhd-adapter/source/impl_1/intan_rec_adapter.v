/***********************************************************
	Author: Manuel Monge
	Description:
		Adapter for Data/CLK-to-SPI Intan Rec Controller.
***********************************************************/

//`default_nettype none

`include "../../../headstage/source/impl_1/common/edge_detector.v"
`include "../../../headstage/source/impl_1/common/shift_registers.v"
`include "../../../headstage/source/impl_1/common/spi_slave.v"
`include "../../../headstage/source/impl_1/common/spi_master.v"
`include "../../../headstage/source/impl_1/common/fifo.v"

`include "../../../headstage/source/impl_1/afe/rhd2164/spi_master_rhd2164x2.v"
`include "../../../headstage/source/impl_1/afe/rhd2164/rhd2164_controller.v"

`include "../../../headstage/source/impl_1/app/regbank.v"
`include "../../../headstage/source/impl_1/app/ch_sel.v"
`include "../../../headstage/source/impl_1/app/main_controller.v"
`include "../../../headstage/source/impl_1/app/dtx_mux_reg.v"
`include "../../../headstage/source/impl_1/app/test_pattern_gen.v"
`include "../../../headstage/source/impl_1/app/fifo_din_mux.v"


`include "../../../headstage/source/impl_1/afe/rhd2164/rhd2164_defs.vh"
`include "../../../headstage/source/impl_1/common/gddr_sync.v"





module intan_rec_adapter (
	input  wire	rstb,
	input  wire	clkin,			// L13 (12 MHz)
	// Data/Clk from Kuntur
	input  wire	kuntur_data,	// V1 (LVDS)
	input  wire	kuntur_clk,		// Y2 (LVDS)
	// SPI to Intan Recording Controller
	input  wire	spi0_csb,		// Y17 (LVDS)
	input  wire	spi0_sck,		// W14 (LVDS)
	input  wire	spi0_mosi,		// Y15 (LVDS)
	output wire	spi0_miso1,		// Y13 (LVDS)
	output wire	spi0_miso0,		// U15 (LVDS)
	// LED visualizer
	output wire	led_n_4,		// L16 (LVCMOS)
	output wire	led_n_3,		// F14 (LVCMOS)
	output wire	led_n_2,		// G13 (LVCMOS)
	output wire	led_n_1,		// F13 (LVCMOS)
	output wire	led_n_0		// E17 (LVCMOS)
);


	// wire & regs
	wire clk;
	
	reg [15:0] spi0_data_tx_a0, spi0_data_tx_b0, spi0_data_tx_a1, spi0_data_tx_b1;
	wire [15:0] spi0_data_rx;
	wire spi0_done, tx_load;
	wire [1:0] cmd_code;
	wire [5:0] cmd_register;
	wire [7:0] reg_ram0_dout0, reg_ram0_dout1,reg_ram0_dh, cmd_data;
	wire [15:0] reg_ram0_dout0_16b,reg_ram0_dout1_16b;
	
	wire kuntur_clk_0,kuntur_clk_os,eclk,wclk;
	wire [7:0] kuntur_data_8b;
	wire pll_phasedir,pll_phasestep,pll_lock;
	wire cdr_rdy, cdr_valid;
	wire [4:0] led_n;
	
	
	// LED assignment
	assign led_n_4 = led_n[4];
	assign led_n_3 = led_n[3];
	assign led_n_2 = led_n[2];
	assign led_n_1 = led_n[1];
	assign led_n_0 = led_n[0];
	
	
	// Clock directly from on-board oscillator at 12 MHz
	assign clk = clkin;


	wire gddr_rdy, gddr_ddr_rst, gddr_stop;
	wire rst_n_sup, lock_sync, gddr_rst, cdr_rstb, rst_n_emu;
	// Reset generation 
	reset_gen_adapter rst_gen_0 (
		// inputs
		.clkin	(clkin),
		.rstb	(rstb),
		.pll_lock	(pll_lock),
		.wclk	(wclk),
		.gddr_rdy	(gddr_rdy),
		// outputs
		.rst_n_sup	(rst_n_sup),
		.lock_sync	(lock_sync),
		.gddr_rst	(gddr_rst),
		.cdr_rstb	(cdr_rstb),
		.rst_n_emu	(rst_n_emu)
	);
		
	gddr_sync u_gddr_sync (
		.rst       (gddr_rst),    // async, active high      <- reset_gen_adapter
		.sync_clk  (clkin),       // free-running 12 MHz
		.start     (lock_sync),   // begin when the PLL is locked
		.stop      (gddr_stop),   // -> ECLKSYNC.STOP
		.ddr_reset (gddr_ddr_rst),// -> ECLKDIV.DIVRST, IDDRX4.RST
		.ready     (gddr_rdy)     // -> reset_gen_adapter.gddr_rdy
	);
	
	status_leds leds (
		.clkin	(clkin),
		.rst_n_sup	(rst_n_sup),
		.lock_sync	(lock_sync),
		.gddr_rdy	(gddr_rdy),
		.cdr_rdy	(cdr_rdy),
		.cdr_valid	(cdr_valid),
		.led_n	(led_n)
	);



	// **********************************************************
	// CDR for kuntur_data and kuntur_clk - BEGIN
	// **********************************************************
	
	//assign pll_phasedir = 1'b0;
	//assign pll_phasestep = 1'b1;
	
	
	pll0 pllcdr (
		.clki_i	(kuntur_clk),
		.phasedir_i	(pll_phasedir), // '0'=delayed (lagging); '1'=advanced (leading)
		.phasestep_i	(pll_phasestep), // active on falling edge. Each pulse signal changes the phase by phasemin.
										 // Output clock phase change per step = [1/(8 × (DIVO<n>_ACTUAL_STR + 1))] × 360°
										 // DIV0S_ACTUAL_STR = "34" -> phase_step ~ 1.28 deg
										 // Full turn (360 deg) = 280 steps
		.phaseloadreg_i	(1'b0),
		.phasesel_i	(3'b001), // 3-bit input: Select the PLL oputput that can be shifted. '001'=CLKOS
		.clkop_o	(kuntur_clk_0),
		.clkos_o	(kuntur_clk_os),
		.lock_o		(pll_lock)
	);
	
	
	
	
	// Primitive block: Edge-clock synchronizer
	ECLKSYNC #(
		.STOP_EN ("ENABLE")
	) eclksync_0 (
		.ECLKIN  (kuntur_clk_os),  // I
		.STOP    (gddr_stop),  // I
		.ECLKOUT (eclk)   // O
	);

	// Primitive block: Edge-clock divider
	ECLKDIV #(
		.ECLK_DIV ("4"),
		.GSR      ("DISABLED")
	) eclkdiv_0 (
		.DIVRST (gddr_ddr_rst),  // I
		.ECLKIN (eclk),  // I
		.SLIP   (1'b0),  // I
		.DIVOUT (wclk)   // O
	);
	
	// Primitive block: Input, Double-Data Rate 1:8 deserializer
	IDDRX4 iddrx4_0 (
	  .D       (kuntur_data),  // I
	  .SCLK    (wclk),  // I
	  .RST     (gddr_ddr_rst),  // I
	  .ECLK    (eclk),  // I
	  .ALIGNWD (1'b0),  // I
	  .Q0      (kuntur_data_8b[0]),  // O
	  .Q1      (kuntur_data_8b[1]),  // O
	  .Q2      (kuntur_data_8b[2]),  // O
	  .Q3      (kuntur_data_8b[3]),  // O
	  .Q4      (kuntur_data_8b[4]),  // O
	  .Q5      (kuntur_data_8b[5]),  // O
	  .Q6      (kuntur_data_8b[6]),  // O
	  .Q7      (kuntur_data_8b[7])   // O
	);
	
	// CDR
	cdr cdr0 (
		.rstb	(cdr_rstb),
		.clk	(clk),
		.wclk	(wclk),
		.lock	(pll_lock),
		.din	(kuntur_data_8b),
		.pll_phasedir	(pll_phasedir), // output
		.pll_phasestep_sync	(pll_phasestep), // output
		.rdy	(cdr_rdy),
		.valid	(cdr_valid)
	);
	
	// **********************************************************
	// CDR for kuntur_data and kuntur_clk - END
	// **********************************************************
	
	

	// RHD2164 SPI Slave Model	
	
	rhd2164x2_spi_slave_model rhd2164x2_spi0 (
		.rstb	(rst_n_emu),		// Active-low asyncrhonous reset
		.csb	(spi0_csb),		// Active-low chip-select
		.sck	(spi0_sck),		// SPI Clock
		.mosi	(spi0_mosi),		// Master-Output Slave-Input
		.miso0	(spi0_miso0),		// Master-Input Slave-Output
		.miso1	(spi0_miso1),		// Master-Input Slave-Output
		.data_tx_a0	(spi0_data_tx_a0),	// Data to be transmitted
		.data_tx_b0	(spi0_data_tx_b0),	// Data to be transmitted
		.data_tx_a1	(spi0_data_tx_a1),	// Data to be transmitted
		.data_tx_b1	(spi0_data_tx_b1),	// Data to be transmitted
		.data_rx	(spi0_data_rx),	// Data received
		.tx_load	(tx_load),
		.done	(spi0_done)		// TX/RX operation completed; no communicacion happening
	);
	
	assign cmd_code = spi0_data_rx[15:14];
	assign cmd_register = spi0_data_rx[13:8];
	assign cmd_data = spi0_data_rx[7:0];
	
	rhd2164_model_reg_ram reg_ram0 (
		.rstb	(rst_n_emu),
		.sck	(spi0_sck),
		.cmd_code	(cmd_code),
		.cmd_register	(cmd_register),
		.cmd_data	(cmd_data),
		.dout0	(reg_ram0_dout0),
		.dout1	(reg_ram0_dout1),
		.dh		(reg_ram0_dh)
	);
	
	assign reg_ram0_dout0_16b = {reg_ram0_dh,reg_ram0_dout0};
	assign reg_ram0_dout1_16b = {reg_ram0_dh,reg_ram0_dout1};
	
	// data_tx registers
	always@(negedge rst_n_emu or posedge spi0_sck)
		if (!rst_n_emu) begin
			spi0_data_tx_a0 <= 16'd0;
			spi0_data_tx_b0 <= 16'd0;
			spi0_data_tx_a1 <= 16'd0;
			spi0_data_tx_b1 <= 16'd0;
		end
		else if (tx_load) begin
			spi0_data_tx_a0 <= reg_ram0_dout0_16b;
			spi0_data_tx_b0 <= reg_ram0_dout1_16b;
			spi0_data_tx_a1 <= reg_ram0_dout0_16b;
			spi0_data_tx_b1 <= reg_ram0_dout1_16b;
		end

endmodule







	


module status_leds (
	input  wire	clkin,
	input  wire	rst_n_sup,
	input  wire	lock_sync,
	input  wire	gddr_rdy,
	input  wire	cdr_rdy,
	input  wire	cdr_valid,
	output reg	[4:0]	led_n
);

	// wires & regs
	reg [23:0] cnt0;
	reg [1:0] rdy_q, valid_q;

	// Heartbeat
	always@(negedge rst_n_sup or posedge clkin)
		if (!rst_n_sup)
			cnt0 <= 0;
		else
			cnt0 <= cnt0 + 1;
	
	// LED outputs
	always@(negedge rst_n_sup or posedge clkin)
		if (!rst_n_sup) begin
			led_n <= 5'b11111;
			rdy_q <= 2'b00;
			valid_q <= 2'b00;
		end
		else begin
			led_n[0] <= ~cnt0[23];
			led_n[1] <= ~lock_sync;
			led_n[2] <= ~(gddr_rdy && !rdy_q[1]);
			led_n[3] <= ~(gddr_rdy && valid_q[1]);
			led_n[4] <= ~(gddr_rdy && rdy_q[1] && !valid_q[1]);
			rdy_q <= {rdy_q[0], cdr_rdy};
			valid_q <= {valid_q[0], cdr_valid};
		end

endmodule








module reset_gen_adapter (
	// inputs
	input  wire	clkin,	// 12 MHz Adapter board clock (L13)
	input  wire	rstb,	// Async reset_b connected to a button/switch (SW2 on G14). Low resets all three domains
	input  wire	pll_lock,	// Async input from PLL. It is 1 when PLL is locked.
	input  wire	wclk,	// ECLK div by 4, ~10.6 MHz
	input  wire	gddr_rdy,	// GDDR_SYNC block READY output. It is 1 when FPGA start-up sequence is done.
	// outputs
	output reg	rst_n_sup,	// Active low, Supervisor reset. CLKIN domain
	output reg	lock_sync,	// Active high, CLKIN domain
	output reg	gddr_rst,	// Active high, CLKIN domain
	output reg	cdr_rstb,	// Active low, WCLK domain
	output wire	rst_n_emu	// Async active-low reset to SCK domain
);
	
	
	// ***************************************
	// rst_n_sup generation - BEGIN
	// ***************************************
	localparam SUP_CYCLES = 5'd20;
	reg [4:0] cnt0;
	
	always@(negedge rstb or posedge clkin)
		if (!rstb)
			rst_n_sup <= 1'b0;
		else if (cnt0 == SUP_CYCLES)
			rst_n_sup <= 1'b1;
	
	always@(negedge rstb or posedge clkin)
		if (!rstb)
			cnt0 <= 0;
		else
			if (cnt0 < SUP_CYCLES)
				cnt0 <= cnt0 + 1;
	
	
	// ***************************************
	// lock_sync generation - BEGIN
	// ***************************************
	reg pll_lock_reg;
	
	always@(negedge rstb or posedge clkin)
		if (!rstb) begin
			lock_sync <= 1'b0;
			pll_lock_reg <= 1'b0;
		end
		else begin
			lock_sync <= pll_lock_reg;
			pll_lock_reg <= pll_lock;
		end
	
	
	// ***************************************
	// gddr_rst generation - BEGIN
	// ***************************************
	
	always@(negedge rstb or posedge clkin)
		if (!rstb)
			gddr_rst <= 1'b1;
		else
			gddr_rst <= (!rst_n_sup) || (!lock_sync);
	
	
	// ***************************************
	// cdr_rstb generation - BEGIN
	// ***************************************
	reg cdr_rstb_1;
	wire cdr_go;
	
	assign cdr_go = rstb && gddr_rdy;
	
	always@(negedge cdr_go or posedge wclk)
		if (!cdr_go) begin
			cdr_rstb <= 1'b0;
			cdr_rstb_1 <= 1'b0;
		end
		else begin
			cdr_rstb <= cdr_rstb_1;
			cdr_rstb_1 <= 1'b1;
		end


	// ***************************************
	// rst_n_emu generation - BEGIN
	// ***************************************
	
	assign rst_n_emu = rstb;
	

endmodule




/***********************************************************
	Author: Manuel Monge
	Description:
		SPI-Slave core with n-bits tx/rx registers.
***********************************************************/

module rhd2164x2_spi_slave_model #(
	localparam n = 16,	// n = 2^m bits to be transmitted/received
	localparam m = 4	// n = 2^m bits to be transmitted/received
	)(
	input  wire			rstb,		// Active-low asyncrhonous reset
	input  wire			csb,		// Active-low chip-select
	input  wire			sck,		// SPI Clock
	input  wire			mosi,		// Master-Output Slave-Input
	output wire			miso0,		// Master-Input Slave-Output
	output wire			miso1,		// Master-Input Slave-Output
	input  wire [n-1:0]	data_tx_a0,	// Data to be transmitted
	input  wire [n-1:0]	data_tx_b0,	// Data to be transmitted
	input  wire [n-1:0]	data_tx_a1,	// Data to be transmitted
	input  wire [n-1:0]	data_tx_b1,	// Data to be transmitted
	output reg	[n-1:0]	data_rx,	// Data received
	output wire			tx_load,
	output wire			done		// TX/RX operation completed; no communicacion happening
);


	// wires & regs
	reg [n-1:0] rx,tx_a0,tx_b0,tx_a1,tx_b1;
	reg [m:0] cnt0;
	reg miso_a0,miso_b0,miso_a1,miso_b1;
	wire rx_load,cnt0_is_15;

	
	
	// n-counter
	always@(negedge rstb or posedge sck or posedge csb)
		if (!rstb)
			cnt0 <= 0;
		else if (csb)
			cnt0 <= 0;
		else
			cnt0 <= cnt0 + 1;
	
	// RX registers
	always@(negedge rstb or posedge sck)
		if (!rstb)
			rx <= 0;
		else
			rx <= {rx[n-2:0],mosi};

	assign rx_load = (cnt0 == 5'b10000) ? 1:0;
	assign tx_load = (cnt0 == 5'd1) ? 1:0;
	assign cnt0_is_15 = (cnt0 == 5'b01111) ? 1:0;

	always@(negedge rstb or negedge sck)
		if (!rstb)
			data_rx <= 0;
		else if (rx_load)
			data_rx <= rx;
	
	
	// TX registers
	always@(negedge rstb or posedge sck)
		if (!rstb) begin
			tx_a0 <= 0;
			tx_b0 <= 0;
			tx_a1 <= 0;
			tx_b1 <= 0;
		end
		else begin
			if (cnt0_is_15) begin
				tx_a0 <= data_tx_a0;
				tx_b0 <= data_tx_b0;
				tx_a1 <= data_tx_a1;
				tx_b1 <= data_tx_b1;
			end
			else begin
				tx_a0 <= {tx_a0[n-2:0],1'b0};
				tx_b0 <= {tx_b0[n-2:0],1'b0};
				tx_a1 <= {tx_a1[n-2:0],1'b0};
				tx_b1 <= {tx_b1[n-2:0],1'b0};
			end
		end
	
	// DDR mux
	always@(negedge rstb or posedge sck)
		if (!rstb) begin
			miso_a0 <= 1'b0;
			miso_b0 <= 1'b0;
			miso_a1 <= 1'b0;
			miso_b1 <= 1'b0;
		end
		else begin
			miso_a0 <= tx_a0[n-1];
			miso_b0 <= tx_b0[n-1];
			miso_a1 <= tx_a1[n-1];
			miso_b1 <= tx_b1[n-1];
		end
	
	assign miso0 = (sck) ? miso_a0:miso_b0;
	assign miso1 = (sck) ? miso_a1:miso_b1;


endmodule







module rhd2164_model_reg_ram (
	input  wire	rstb,
	input  wire	sck,
	input  wire	[1:0]	cmd_code,
	input  wire	[5:0]	cmd_register,
	input  wire	[7:0]	cmd_data,
	output reg	[7:0]	dout0,
	output reg	[7:0]	dout1,
	output reg	[7:0]	dh
);

	reg [15:0] ram [0:63];

	// Initialize ROM contents
	initial begin
		ram[8'd0] = `RHD_REG0;
		ram[8'd1] = `RHD_REG1;
		ram[8'd2] = `RHD_REG2;
		ram[8'd3] = `RHD_REG3;
		ram[8'd4] = `RHD_REG4;
		ram[8'd5] = `RHD_REG5;
		ram[8'd6] = `RHD_REG6;
		ram[8'd7] = `RHD_REG7;
		ram[8'd8] = `RHD_REG8;
		ram[8'd9] = `RHD_REG9;
		ram[8'd10] = `RHD_REG10;
		ram[8'd11] = `RHD_REG11;
		ram[8'd12] = `RHD_REG12;
		ram[8'd13] = `RHD_REG13;
		ram[8'd14] = `RHD_REG14;
		ram[8'd15] = `RHD_REG15;
		ram[8'd16] = `RHD_REG16;
		ram[8'd17] = `RHD_REG17;
		ram[8'd18] = `RHD_REG18;
		ram[8'd19] = `RHD_REG19;
		ram[8'd20] = `RHD_REG20;
		ram[8'd21] = `RHD_REG21;
		ram[8'd22] = 8'd0;
		ram[8'd23] = 8'd0;
		ram[8'd24] = 8'd0;
		ram[8'd25] = 8'd0;
		ram[8'd26] = 8'd0;
		ram[8'd27] = 8'd0;
		ram[8'd28] = 8'd0;
		ram[8'd29] = 8'd0;
		ram[8'd30] = 8'd0;
		ram[8'd31] = 8'd0;
		ram[8'd32] = 8'd0;
		ram[8'd33] = 8'd0;
		ram[8'd34] = 8'd0;
		ram[8'd35] = 8'd0;
		ram[8'd36] = 8'd0;
		ram[8'd37] = 8'd0;
		ram[8'd38] = 8'd0;
		ram[8'd39] = 8'd0;
		ram[8'd40] = `RHD_REG40_VAL;
		ram[8'd41] = `RHD_REG41_VAL;
		ram[8'd42] = `RHD_REG42_VAL;
		ram[8'd43] = `RHD_REG43_VAL;
		ram[8'd44] = `RHD_REG44_VAL;
		ram[8'd45] = `RHD_REG45_VAL;
		ram[8'd46] = `RHD_REG46_VAL;
		ram[8'd47] = `RHD_REG47_VAL;
		ram[8'd48] = `RHD_REG48_VAL;
		ram[8'd49] = `RHD_REG49_VAL;
		ram[8'd50] = `RHD_REG50_VAL;
		ram[8'd51] = `RHD_REG51_VAL;
		ram[8'd52] = `RHD_REG52_VAL;
		ram[8'd53] = `RHD_REG53_VAL;
		ram[8'd54] = `RHD_REG54_VAL;
		ram[8'd55] = `RHD_REG55_VAL;
		ram[8'd56] = `RHD_REG56_VAL;
		ram[8'd57] = `RHD_REG57_VAL;
		ram[8'd58] = `RHD_REG58_VAL;
		ram[8'd59] = `RHD_REG59_VAL;
		ram[8'd60] = `RHD_REG60_VAL;
		ram[8'd61] = `RHD_REG61_VAL;
		ram[8'd62] = `RHD_REG62_VAL;
		ram[8'd63] = `RHD_REG63_VAL;
	end

	// Writing
	//always@(posedge sck)
		//if (cmd_code == `RHD_WRITE_CODE)
			//ram[cmd_register] <= cmd_data;

	// Reading
	always@(negedge rstb or posedge sck)
		if (!rstb) begin
			dh <= 0;
			dout0 <= 0;
			dout1 <= 0;
		end
		else
			if (cmd_code == `RHD_READ_CODE) begin
				dout0 <= ram[cmd_register];
				dh <= 8'h00;
				if (cmd_register == 6'd59)
					dout1 <= `RHD_2164_MISOB_MARKER;
				else
					dout1 <= ram[cmd_register];
			end
			else if (cmd_code == `RHD_WRITE_CODE) begin
				dout0 <= cmd_data;
				dout1 <= cmd_data;
				dh <= 8'hff;
			end
			else if ({cmd_code,cmd_register,8'd0} == `RHD_CALIBRATE) begin
				dout0 <= 8'h00;
				dout1 <= 8'h00;
				dh <= 8'h00;
			end
			else if ({cmd_code,cmd_register,8'd0} == `RHD_CLEAR) begin
				dout0 <= 8'h00;
				dout1 <= 8'h00;
				dh <= 8'h00;
			end


endmodule





// CDR

module cdr (
	input  wire	rstb,
	input  wire	clk,
	input  wire	wclk,
	input  wire	lock,
	input  wire	[7:0]	din,
	// outputs
	output wire	pll_phasedir,
	output reg	pll_phasestep_sync,
	output wire	rdy,
	output wire	valid,
	output reg	[8:0] eye_center,
	output wire	[8:0] eye_width,
	// meas
	output reg	[8:0] meas_phase,
	output reg	meas_no_lock,
	output reg	[15:0] meas_errors,
	output reg	meas_en
);
	
	// parameters
	localparam PHASE_MAX = 9'd279;

	
	// wire & reg
	wire phase_en;
	reg [8:0] phase;
	reg [8:0] cur_start, cur_len, best_start, best_len, first_len;
	
	wire aligner_rst, aligner_rdy;
	wire [8:0] aligner_pos;
	wire [7:0] aligner_dout;
	wire aligner_valid, aligner_error;
	wire checker_rst, checker_start, checker_no_lock, checker_done;
	wire phase_is_PHASE_MAX;
	wire phase_rst;
	
	wire cdr_rdy, cdr_valid;
	
	wire [15:0] errors;
	
	wire pll_phasestep;
	reg pll_phasestep_reg;
	reg lock_sync, lock_reg;
	
	// 9-bit counter for phase
	always@(posedge wclk)
		if (phase_rst)
			phase <= 0;
		else if (phase_en)
			if ( phase_is_PHASE_MAX )
				phase <= 0;
			else
				phase <= phase + 1;

	assign phase_is_PHASE_MAX = (phase == PHASE_MAX);
	
	assign pll_phasedir = 1'b0; // delayed phase change
	
	// aligner
	frame_aligner aligner0 (
		.rst_sync	(aligner_rst),	// sync reset
		.clk		(wclk),		// wclk
		.din		(din),		// input
		.dout		(aligner_dout),	// output
		.pos		(aligner_pos), // output
		.bitsel		(), // output
		.valid		(aligner_valid),	// output
		.rdy		(aligner_rdy)	// output
	);


	// checker
	frame_checker checker0 (
		.rst_sync	(checker_rst),	// sync reset
		.clk		(wclk),		// wclk
		.start		(checker_start),
		.din		(aligner_dout),	// input [7:0]
		.pos		(aligner_pos),	// input [8:0]
		.no_lock	(checker_no_lock),	// output
		.errors		(errors), // output [15:0]
		.done		(checker_done)	// output
	);
	
	
	wire meas_rst, meas_en_fsm;
	
	// meas_en_out is one cycle delayed from meas_en
	always@(negedge rstb or posedge wclk)
		if (!rstb || meas_rst)
			meas_en <= 0;
		else
			meas_en <= meas_en_fsm; // port meas_en
	
	reg [8:0] len_n, start_n, cand_len;
	
	// registers for measurements at each phase step
	always@(negedge rstb or posedge wclk)
		if (!rstb || meas_rst) begin
			cur_start <= 0;
			cur_len <= 0;
			best_start <= 0;
			best_len <= 0;
			first_len <= 0;
			meas_phase <= 9'd0;
			meas_no_lock <= 1'b0;
			meas_errors <= 16'd0;
			len_n = 0;
			start_n = 0;
			cand_len = 0;
		end
		else if (meas_en_fsm) begin
			meas_phase <= phase;
			meas_no_lock <= checker_no_lock || aligner_error;
			if (checker_no_lock || aligner_error)
				meas_errors <= 16'hffff;
			else
				meas_errors <= errors;
				
			// errors logic
			if ( (errors == 16'd0) && (!checker_no_lock) && (!aligner_error) ) begin
				len_n = cur_len + 1;
				if (cur_len == 0)
					start_n = phase;
				else
					start_n = cur_start;
				
				cur_len <= len_n;
				cur_start <= start_n;
				
				if (start_n == 0)
					first_len <= len_n;

				if ( phase_is_PHASE_MAX && (start_n != 0) )
					cand_len = len_n + first_len;
				else
					cand_len = len_n;
				
				// Update best case
				if (cand_len > best_len) begin
					best_start <= start_n;
					best_len <= cand_len;
				end
				
			end
			
			else
				cur_len <= 0;
			
		end
		
	// Center of the Eye
	wire center_en;
	wire [8:0] center;
	
	assign center = best_start + {1'b0,best_len[8:1]};
	
	always@(negedge rstb or posedge wclk)
		if (!rstb || meas_rst)
			eye_center <= 9'd0;
		else if (center_en)
			if (center >= 280)
				eye_center <= center - 9'd280;
			else
				eye_center <= center;
			
	
	assign rdy = cdr_rdy;
	assign valid = cdr_valid;
	assign eye_width = best_len;
		
	
	// FSM
	cdr_fsm fsm0 (
		.rstb	(rstb),
		.clk	(wclk),
		.lock	(lock_sync),
		.phase_is_PHASE_MAX (phase_is_PHASE_MAX), // input
		.phase	(phase),	// input
		.phase_rst	(phase_rst), // input
		.phase_en	(phase_en),	// output
		.phasestep_reg	(pll_phasestep),	// output
		// aligner
		.aligner_rst	(aligner_rst),	// output
		.aligner_error	(aligner_error),	// output
		.aligner_rdy	(aligner_rdy),	// input
		.aligner_valid	(aligner_valid),	// input
		// checker
		.checker_rst	(checker_rst),	// output
		.checker_start	(checker_start), // output
		.checker_no_lock	(checker_no_lock), // input
		.checker_done	(checker_done), // input
		// phase measurements
		.best_len	(best_len), // input
		.meas_en	(meas_en_fsm), // output
		.meas_rst	(meas_rst), // output
		// center
		.center_en	(center_en), // output
		.eye_center	(eye_center), // input
		// others
		.valid	(cdr_valid), // output
		.rdy	(cdr_rdy) // output
	);
	
	
	// PHASESTEP synchronizer
	always@(negedge rstb or posedge clk)
		if (!rstb) begin
			pll_phasestep_sync <= 1'b1;
			pll_phasestep_reg <= 1'b1;
		end
		else begin
			pll_phasestep_sync <= pll_phasestep_reg;
			pll_phasestep_reg <= pll_phasestep;
		end
	
	// LOCK synchronizer
	always@(negedge rstb or posedge wclk)
		if (!rstb) begin
			lock_sync <= 1'b0;
			lock_reg <= 1'b0;
		end
		else begin
			lock_sync <= lock_reg;
			lock_reg <= lock;
		end
	
endmodule




module cdr_fsm (
	input  wire	rstb,
	input  wire	clk,
	input  wire	lock,
	input  wire phase_is_PHASE_MAX,
	input  wire	[8:0]	phase,
	output reg	phase_rst,
	output reg	phase_en,
	output reg	phasestep_reg,
	// aligner
	output reg	aligner_rst,
	output reg	aligner_error,
	input  wire	aligner_rdy,
	input  wire	aligner_valid,
	
	// checker
	output reg	checker_rst,
	output reg	checker_start,
	input  wire	checker_no_lock,
	input  wire	checker_done,
	// phase measurements
	input  wire	[8:0] best_len,
	output reg	meas_en,
	output reg	meas_rst,
	// center
	output reg	center_en,
	input  wire	[8:0]	eye_center,
	// others
	output reg	valid,
	output reg	rdy
);

	// state
	localparam idle = 5'd0;
	localparam op0 = 5'd1; // ALIGN
	localparam op1a = 5'd2;
	localparam op1b = 5'd3;
	localparam op2a = 5'd4; // CHECK
	localparam op2b = 5'd5;
	localparam op2c = 5'd6;
	localparam op2d = 5'd7;
	localparam op2e = 5'd8;
	localparam op3a = 5'd9;
	localparam op4a = 5'd10;
	localparam FAIL = 5'd12;
	localparam SUCCESS = 5'd13;
	localparam WAIT = 5'd14;
	localparam MOVE_TO_CENTER = 5'd15;
	localparam MTC_LOW = 5'd16;
	localparam RELOCK = 5'd17;
	localparam RELOCK_1 = 5'd18;
	localparam RELOCK_2 = 5'd19;
	localparam RELOCK_3 = 5'd20;
	
	reg [4:0] current_state,next_state;
	reg phasestep, cnt0_en, cnt0_rst;
	reg [7:0] cnt0;
	
	
	// register outputs
	always@(negedge rstb or posedge clk)
		if (!rstb)
			phasestep_reg <= 1'b1;
		else
			phasestep_reg <= phasestep;
	
	// cnt0: auxiliary counter
	always@(posedge clk)
		if (cnt0_rst)
			cnt0 <= 0;
		else if (cnt0_en)
			cnt0 <= cnt0 + 1;
	
	// state register
	always@(negedge rstb or posedge clk)
		if (!rstb)
			current_state <= idle;
		else
			current_state <= next_state;
	
	// next_state logic
	always@(*) begin
		// default values
		rdy = 1'b0;
		valid = 1'b0;
		aligner_rst = 1'b0;
		aligner_error = 1'b0;
		checker_rst = 1'b0;
		checker_start = 1'b0;
		phase_en = 1'b0;
		phasestep = 1'b1;
		cnt0_en = 1'b0;
		cnt0_rst = 1'b0;
		phase_rst = 1'b0;
		meas_en = 1'b0;
		meas_rst = 1'b0;
		center_en = 1'b0;
		
		case (current_state)
			idle: begin
				if (lock)
					next_state = op0;
				else
					next_state = idle;
				aligner_rst = 1'b1;
				checker_rst = 1'b1;
				cnt0_rst = 1'b1;
				phase_rst = 1'b1;
				meas_rst = 1'b1;
			end
			
			// ALIGN
			op0: begin
				if (!lock)
					next_state = idle;
				else begin
					if (aligner_rdy)
						if (aligner_valid)
							next_state = op1a;
						else
							next_state = op1b;
					else
						next_state = op0;
				end
			end
			
			// data is aligned -> we proceed with checker
			op1a: begin
				if (!lock)
					next_state = idle;
				else begin
					next_state = op2a;
					checker_rst = 1'b1;
				end
			end
			
			// data is not aligned -> we indicate aligner error
			op1b: begin
				if (!lock)
					next_state = idle;
				else begin
					next_state = op2c;
					aligner_error = 1'b1;
					phasestep = 1'b0; // send active-low pulse to PLL
					meas_en = 1'b1;
				end
			end
			
			// CHECK
			op2a:begin
				if (!lock)
					next_state = idle;
				else begin
					next_state = op2b;
					checker_start = 1'b1;
				end
			end
			
			op2b:begin
				if (!lock)
					next_state = idle;
				else begin
					if (checker_done)
						if (checker_no_lock) begin
							next_state = op2c;
							phasestep = 1'b0; // send active-low pulse to PLL
							meas_en = 1'b1;
						end
						else begin
							next_state = op3a;
							meas_en = 1'b1;
						end
					else
						next_state = op2b;
				end
			end
			
			// increase phase due to align or check errors or after lock
			op2c: begin
				if (!lock)
					next_state = idle;
				else begin
					if (phase_is_PHASE_MAX)
						next_state = op4a;
					else
						next_state = op2d;
					phase_en = 1'b1;
					phasestep = 1'b0; // send active-low pulse to PLL
					cnt0_en = 1'b1;
				end
			end
			op2d: begin
				if (!lock)
					next_state = idle;
				else begin
					if (cnt0 == 8'd7) begin
						next_state = op2e;
						cnt0_rst = 1'b1;
					end
					else
						next_state = op2d;
					cnt0_en = 1'b1;
					phasestep = 1'b1; // keep it high for 8 cycles
				end
			end
			// ONE PHASE STEP done - get ready for next step
			op2e: begin
				if (!lock)
					next_state = idle;
				else begin
					next_state = op0;
					aligner_rst = 1'b1;
					checker_rst = 1'b1;
					cnt0_rst = 1'b1;
				end
			end
			
			// ALIGN and CHECK GOOD for current phase
			op3a: begin
				if (!lock)
					next_state = idle;
				else begin
					next_state = op2c;
					phasestep = 1'b0; // send active-low pulse to PLL
				end
			end
			
			
			// ALL PHASE STEPS done - DECIDE
			op4a: begin
				if (!lock)
					next_state = idle;
				else begin
					if (best_len == 8'd0)
						next_state = FAIL;
					else begin
						next_state = WAIT;
						center_en = 1'b1;
						phase_rst = 1'b1; // reset phase counter
						cnt0_rst = 1'b1;
					end
				end
			end
			
			FAIL: begin
				if (!lock)
					next_state = idle;
				else begin
					next_state = FAIL;
					valid = 1'b0;
					rdy = 1'b1;
				end
			end
			
			WAIT: begin
				if (!lock)
					next_state = idle;
				else begin
					if (cnt0 == 8'd7)
						next_state = MOVE_TO_CENTER;
					else
						next_state = WAIT;
					cnt0_en = 1'b1;
					phasestep = 1'b1; // keep it high for 8 cycles
				end
			end
			
			MOVE_TO_CENTER: begin
				if (!lock)
					next_state = idle;
				else begin
					if (phase == eye_center) begin
						next_state = RELOCK;
						aligner_rst = 1'b1;
					end
					else begin
						next_state = MTC_LOW;
						phasestep = 1'b0; // send active-low pulse to PLL
					end
				end
			end
			
			MTC_LOW: begin
				if (!lock)
					next_state = idle;
				else begin
					next_state = WAIT;
					cnt0_rst = 1'b1;
					phasestep = 1'b0; // send active-low pulse to PLL
					phase_en = 1'b1;	
				end
			end
			
			RELOCK: begin
				if (!lock)
					next_state = idle;
				else begin
					if (aligner_rdy)
						if (aligner_valid)
							next_state = RELOCK_1;
						else
							next_state = idle;
					else
						next_state = RELOCK;
				end
			end
			
			RELOCK_1: begin
				if (!lock)
					next_state = idle;
				else begin
					next_state = RELOCK_2;
					checker_rst = 1'b1;
				end
			end
			
			RELOCK_2: begin
				if (!lock)
					next_state = idle;
				else begin
					next_state = RELOCK_3;
					checker_start = 1'b1;
				end
			end
			
			RELOCK_3: begin
				if (!lock)
					next_state = idle;
				else begin
					if (checker_done)
						if (!checker_no_lock)
							next_state = SUCCESS;
						else
							next_state = idle;
					else
						next_state = RELOCK_3;
				end
			end
			
			SUCCESS: begin
				if (!lock)
					next_state = idle;
				else begin
					next_state = SUCCESS;
					valid = 1'b1;
					rdy = 1'b1;
				end
			end
			
			// Other States
			default: begin
				next_state = idle;
			end
		endcase
	end
	
endmodule






module frame_aligner (
	input  wire rst_sync,	// sync reset
	input  wire	clk,		// wclk
	input  wire	[7:0]	din,
	output wire	[7:0] dout,
	output reg	[8:0] pos,
	output reg	[2:0] bitsel,
	output reg	valid,
	output reg	rdy
);
	
	// parameters
	localparam PREAMBLE = 32'h5AA55AA5;
	localparam FRAME_LENGTH = 9'd7;
	localparam SEARCH_LENGTH = 9'd14; // frame_length + 2 (bit-shifter) + 4 (preamble)
	
	// state
	localparam idle = 4'd0;
	localparam op0 = 4'd1; // Phase shift
	localparam op1 = 4'd2;
	localparam op2 = 4'd3;
	localparam op3 = 4'd4;
	localparam op4 = 4'd5;
	localparam op5 = 4'd6;
	localparam op6 = 4'd7;
	
	// reg & wire
	reg [3:0] current_state,next_state;
	reg [7:0] din_reg_0,din_reg_1;
	reg [7:0] data_sa, data_sa_reg_0, data_sa_reg_1, data_sa_reg_2, data_sa_reg_3;
	wire data_is_preamble, bitsel_is_max;
	reg bitsel_aligned_en, bitsel_en, rst_sync_2;
	reg [8:0] cnt0;
	
	// cnt0: Count the number of bytes to search
	always@(posedge clk)
		if (rst_sync_2)
			cnt0 <= 9'd0;
		else
			if (cnt0 == SEARCH_LENGTH)
				cnt0 <= 9'd0;
			else
				cnt0 <= cnt0 + 1;
	
	// first register negedge clk
	always@(negedge clk)
		if (rst_sync_2)
			din_reg_0 <= 8'h00;
		else
			din_reg_0 <= din;
	
	// second register posedge
	always@(posedge clk)
		if (rst_sync_2)
			din_reg_1 <= 8'h00;
		else
			din_reg_1 <= din_reg_0;
	
	
	// 3-bit counter for bitsel
	always@(posedge clk)
		if (rst_sync)
			bitsel <= 0;
		else if (bitsel_en)
				bitsel <= bitsel + 1;
				
	assign bitsel_is_max = (bitsel == 3'd7) ? 1:0;
	
	
	// registers after bit-shift
	always@(posedge clk)
		if (rst_sync_2) begin
			data_sa_reg_0 <= 8'h00;
			data_sa_reg_1 <= 8'h00;
			data_sa_reg_2 <= 8'h00;
			data_sa_reg_3 <= 8'h00;
		end
		else begin
			data_sa_reg_0 <= data_sa;
			data_sa_reg_1 <= data_sa_reg_0;
			data_sa_reg_2 <= data_sa_reg_1;
			data_sa_reg_3 <= data_sa_reg_2;
		end
	
	// bit-shift: sample data from bitsel (offset)
	always@(*)
		case (bitsel)
			3'd0: data_sa <= din_reg_1;
			3'd1: data_sa <= {din_reg_0[0],din_reg_1[7:1]};
			3'd2: data_sa <= {din_reg_0[1:0],din_reg_1[7:2]};
			3'd3: data_sa <= {din_reg_0[2:0],din_reg_1[7:3]};
			3'd4: data_sa <= {din_reg_0[3:0],din_reg_1[7:4]};
			3'd5: data_sa <= {din_reg_0[4:0],din_reg_1[7:5]};
			3'd6: data_sa <= {din_reg_0[5:0],din_reg_1[7:6]};
			3'd7: data_sa <= {din_reg_0[6:0],din_reg_1[7]};
			default: data_sa <= din_reg_1;
		endcase
	
	// compare with preamble 
	assign data_is_preamble = ({data_sa_reg_0,data_sa_reg_1,data_sa_reg_2,data_sa_reg_3} == PREAMBLE) ? 1:0;
	
	// position and data_out
	always@(posedge clk)
		if (rst_sync_2)
			pos <= 9'd0;
		else
			if(bitsel_aligned_en)
				pos <= 9'd4;
			else
				if (pos == FRAME_LENGTH)
					pos <= 9'd0;
				else
					pos <= pos + 1;
	
	assign dout = data_sa_reg_0;
	
	// FSM
	
	// state register
	always@(posedge clk)
		if (rst_sync)
			current_state <= idle;
		else
			current_state <= next_state;
	
	// next_state logic
	always@(*) begin
		// default values
		rdy = 1'b0;
		valid = 1'b0;
		bitsel_aligned_en = 1'b0;
		rst_sync_2 = 1'b0;
		bitsel_en = 1'b0;
		
		case (current_state)
			idle: begin
				next_state = op0;
				rst_sync_2 = 1'b1;
			end
			op0: begin
				if (data_is_preamble) begin
					next_state = op1;
					bitsel_aligned_en = 1'b1;
				end
				else if (cnt0 == SEARCH_LENGTH) begin
					if (bitsel_is_max)
						next_state = op2;
					else begin
						next_state = op0;
						bitsel_en = 1'b1;	// next bitsel
						rst_sync_2 = 1'b1;	// clear window and shit registers
					end
				end
				else
					next_state = op0;
			end
			
			op1:begin
				next_state = op1;
				valid = 1'b1;
				rdy = 1'b1;
			end
			
			op2:begin
				next_state = op2;
				rdy = 1'b1;
			end
			// Other States
			default: begin
				next_state = idle;
			end
		endcase
	end
	
endmodule




module frame_checker (
	input  wire rst_sync,	// sync reset
	input  wire	clk,		// wclk
	input  wire	start,
	input  wire	[7:0] din,
	input  wire	[8:0] pos,
	output reg	no_lock,
	output wire	[15:0] errors,
	output reg	done
);
	
	// parameters
	localparam PREAMBLE = 32'h5AA55AA5;
	localparam FRAME_LENGTH = 9'd7;
	parameter NB_ERRORS = 16'd10000; // Number of blocks checked for errors
	parameter NB_CHECK = 16'd100; // Number of blocks checked for check
	localparam FRAME_B0 = 8'hA5;
	localparam FRAME_B1 = 8'h5A;
	localparam FRAME_B2 = 8'hA5;
	localparam FRAME_B3 = 8'h5A;
	localparam FRAME_B6 = 8'h01;
	localparam FRAME_B7 = 8'h80;
	
	// state
	localparam idle = 4'd0;
	localparam op0 = 4'd1;
	localparam op1a = 4'd2;
	localparam op1b = 4'd3;
	localparam op2 = 4'd4;
	localparam op3 = 4'd5;
	localparam op4 = 4'd6;
	localparam op5 = 4'd7;
	localparam op6 = 4'd8;
	
	// reg & wire
	reg [3:0] current_state,next_state;
	
	reg [7:0] fr_pos_0,fr_pos_1,fr_pos_2,fr_pos_3,fr_pos_4,fr_pos_5,fr_pos_6,fr_pos_7;
	reg [7:0] fr_pos_0_reg,fr_pos_1_reg,fr_pos_2_reg,fr_pos_3_reg,fr_pos_4_reg,fr_pos_5_reg,fr_pos_6_reg,fr_pos_7_reg;
	wire [7:0] check8b;
	reg check_is_correct;
	
	reg [2:0] cnt0;
	reg [15:0] frames;
	wire frames_is_NB_CHECK, frames_is_NB_ERRORS;
	reg frame_sa;
	reg frames_cnt_rst;
	
	
	// cnt0: Count the number of blocks and bytes in each block
	always@(posedge clk)
		if (frames_cnt_rst) begin
			cnt0 <= 3'd0;
			frames <= 16'd0;
		end
		else begin
			if (cnt0 == 3'd7)
				frames <= frames + 1;
			cnt0 <= cnt0 + 1;
		end
				
	assign frames_is_NB_CHECK = (frames == NB_CHECK);
	assign frames_is_NB_ERRORS = ( (frames == NB_ERRORS-1 ) && (cnt0 == 3'd7) );

	
	// registers for two frames
	always@(posedge clk)
		if (rst_sync) begin
			fr_pos_0_reg <= 8'h00;
			fr_pos_1_reg <= 8'h00;
			fr_pos_2_reg <= 8'h00;
			fr_pos_3_reg <= 8'h00;
			fr_pos_4_reg <= 8'h00;
			fr_pos_5_reg <= 8'h00;
			fr_pos_6_reg <= 8'h00;
			fr_pos_7_reg <= 8'h00;
			fr_pos_0 <= 8'h00;
			fr_pos_1 <= 8'h00;
			fr_pos_2 <= 8'h00;
			fr_pos_3 <= 8'h00;
			fr_pos_4 <= 8'h00;
			fr_pos_5 <= 8'h00;
			fr_pos_6 <= 8'h00;
			fr_pos_7 <= 8'h00;
		end
		else
			case (pos)
				3'd0: begin
					fr_pos_0_reg <= fr_pos_0;
					fr_pos_0 <= din;
				end
				3'd1: begin
					fr_pos_1_reg <= fr_pos_1;
					fr_pos_1 <= din;
				end
				3'd2: begin
					fr_pos_2_reg <= fr_pos_2;
					fr_pos_2 <= din;
				end
				3'd3: begin
					fr_pos_3_reg <= fr_pos_3;
					fr_pos_3 <= din;
				end
				3'd4: begin
					fr_pos_4_reg <= fr_pos_4;
					fr_pos_4 <= din;
				end
				3'd5: begin
					fr_pos_5_reg <= fr_pos_5;
					fr_pos_5 <= din;
				end
				3'd6: begin
					fr_pos_6_reg <= fr_pos_6;
					fr_pos_6 <= din;
				end
				3'd7: begin
					fr_pos_7_reg <= fr_pos_7;
					fr_pos_7 <= din;
				end
				default: begin
					fr_pos_0_reg <= fr_pos_0;
					fr_pos_0 <= din;
				end
			endcase
	
	// Check if frame is correct.
	assign check8b[0] = (fr_pos_0_reg == FRAME_B0) && (fr_pos_0 == FRAME_B0);
	assign check8b[1] = (fr_pos_1_reg == FRAME_B1) && (fr_pos_1 == FRAME_B1);
	assign check8b[2] = (fr_pos_2_reg == FRAME_B2) && (fr_pos_2 == FRAME_B2);
	assign check8b[3] = (fr_pos_3_reg == FRAME_B3) && (fr_pos_3 == FRAME_B3);
	assign check8b[4] = 1'b1;
	assign check8b[6] = (fr_pos_6_reg == FRAME_B6) && (fr_pos_6 == FRAME_B6);
	assign check8b[7] = (fr_pos_7_reg == FRAME_B7) && (fr_pos_7 == FRAME_B7);
	

	// Bytes 4-5 are a 16-b counter. Consecutive frames should have consecutive counts.
	assign check8b[5] = ( ( {fr_pos_5,fr_pos_4} - {fr_pos_5_reg,fr_pos_4_reg} ) == 16'd1 ) ? 1:0;
	
	// Check if all are correct with bit-reduction operation
	always @(posedge clk)
		if (rst_sync)
			check_is_correct <= 1'b0;
		else if (pos == 3'd0)
			check_is_correct <=  &check8b;
			
	reg [7:0] din_reg, fr_byte;
	wire [7:0] frame_b4, frame_b5;
	wire [7:0] fr_byte_error;
	wire [3:0] fr_byte_error_total;
	reg errors_en;
	reg [16:0] errors17b;
	reg [15:0] frame_cnt_16b;
	
	
	// count bit errors only after lock
	
	// sample the value of FRAME_B4 and FRAME_B5
	always@(posedge clk)
		if (rst_sync)
			frame_cnt_16b <= 16'd0;
		else
			if (frame_sa)
				frame_cnt_16b <= {fr_pos_5, fr_pos_4} + 16'd1;
			else if (pos == 3'd7)
				frame_cnt_16b <= frame_cnt_16b +1;
		
	assign frame_b4 = frame_cnt_16b[7:0];
	assign frame_b5 = frame_cnt_16b[15:8];


	// registers for bytes to compare
	always@(posedge clk)
		if (rst_sync) begin
			din_reg <= 8'h00;
			fr_byte <= 8'h00;
		end
		else begin
			din_reg <= din;
			case (pos)
				3'd0: fr_byte <= FRAME_B0;
				3'd1: fr_byte <= FRAME_B1;
				3'd2: fr_byte <= FRAME_B2;
				3'd3: fr_byte <= FRAME_B3;
				3'd4: fr_byte <= frame_b4;
				3'd5: fr_byte <= frame_b5;
				3'd6: fr_byte <= FRAME_B6;
				3'd7: fr_byte <= FRAME_B7;
				default: fr_byte <= FRAME_B0;
			endcase
		end
	
	// count bit errors per byte
	assign fr_byte_error = (fr_byte ^ din_reg);
	assign fr_byte_error_total = fr_byte_error[0] + fr_byte_error[1] + fr_byte_error[2] + fr_byte_error[3] + fr_byte_error[4] + fr_byte_error[5] + fr_byte_error[6] + fr_byte_error[7];
	
	
	// bit errors accumulator
	always@(posedge clk)
		if (rst_sync)
			errors17b <= 17'd0;
		else if (errors17b < 17'h0ffff)
			if (errors_en)
				errors17b <= errors17b + fr_byte_error_total;
			

	assign errors = ( errors17b[16] ) ? 16'hffff:errors17b[15:0];
	
	// FSM
	
	// state register
	always@(posedge clk)
		if (rst_sync)
			current_state <= idle;
		else
			current_state <= next_state;
	
	// next_state logic
	always@(*) begin
		// default values
		done = 1'b0;
		no_lock = 1'b0;
		frame_sa = 1'b0;
		frames_cnt_rst = 1'b0;
		errors_en = 1'b0;
		
		case (current_state)
			idle: begin
				if (start)
					next_state = op0;
				else
					next_state = idle;
				frames_cnt_rst = 1'b1;
			end
			op0: begin
				if (check_is_correct) begin
					next_state = op1a;
					frame_sa = 1'b1;
					frames_cnt_rst = 1'b1;
				end
				else if (frames_is_NB_CHECK)
					next_state = op2;
				else
					next_state = op0;
			end
			
			// lock
			op1a: begin
				if (frames_is_NB_ERRORS)
					next_state = op1b;
				else
					next_state = op1a;
				errors_en = 1'b1;
			end
			
			op1b:begin
				next_state = op1b;
				done = 1'b1;
			end
			
			// no lock
			op2:begin
				next_state = op2;
				no_lock = 1'b1;
				done = 1'b1;
			end
			// Other States
			default: begin
				next_state = idle;
			end
		endcase
	end
	
endmodule
