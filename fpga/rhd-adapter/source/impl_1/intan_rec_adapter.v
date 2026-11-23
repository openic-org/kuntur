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
`include "../../../headstage/source/impl_1/kuntur_build.vh"





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
	// UART Console
	output wire	con_tx,			// E20 (LVCMOS)
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
	
	
	// PLL values
	// FVCO = 1593.9; DIVOS_ACTUAL_STR = "41"; CLKOS_FREQ_ACTUAL = 37.95;
	
	wire cdr_phasestep, cdr_phasedir;
	wire frx_phasestep, frx_phasedir;
	
	pll0 pllcdr (
		.clki_i	(kuntur_clk),			// 37.95 MHz
		.phasedir_i	(pll_phasedir), 	// '0'=delayed (lagging); '1'=advanced (leading)
		.phasestep_i	(pll_phasestep),	// active on falling edge. Each pulse signal changes the phase by phasemin.
										// Output clock phase change per step = [1/(8 × (DIVO<n>_ACTUAL_STR + 1))] × 360°
										// DIV0S_ACTUAL_STR = "41" -> phase_step ~ 1.071 deg
										// Full turn (360 deg) = 336 steps
		.phaseloadreg_i	(1'b0),
		.phasesel_i	(3'b001), // 3-bit input: Select the PLL oputput that can be shifted. '001'=CLKOS
		.clkop_o	(kuntur_clk_0),
		.clkos_o	(kuntur_clk_os),
		.lock_o		(pll_lock)
	);
	
	
	generate
		if (`KUNTUR_VARIANT == 4'd2) begin: g_frames
			assign pll_phasestep = frx_phasestep;
			assign pll_phasedir = frx_phasedir;
		end
		else if (`KUNTUR_VARIANT == 4'd3) begin: g_pattern
			assign pll_phasestep = cdr_phasestep;
			assign pll_phasedir = cdr_phasedir;
		end
		else begin: g_bad
			KUNTUR_VARIANT_must_be_2_or_3 stop();
		end
	endgenerate
	
	
	
	wire [8:0] phase, eye_center, eye_width;
	wire [7:0] dout;
	wire [2:0] pos;
	wire [31:0] pat_blocks, pat_errors;
	
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
		.pll_phasedir	(cdr_phasedir), // output
		.pll_phasestep_sync	(cdr_phasestep), // output
		.rdy	(cdr_rdy),
		.valid	(cdr_valid),
		.eye_center	(eye_center),
		.eye_width	(eye_width),
	
		// to pat_monitor
		.phase	(phase),
		.dout	(dout),
		.pos	(pos)
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





	wire con_rst_n;

	console_por console_por_0 (
		.clk	(clkin),
		.con_rst_n	(con_rst_n)
	);
	
	
	pat_monitor pat_monitor_0 (
		// inputs
		.rst_n	(con_rst_n),
		.wclk	(wclk),
		.valid	(cdr_valid),
		.din	(dout),
		.pos	(pos),
		// outputs
		.pat_blocks	(pat_blocks),
		.pat_errors	(pat_errors)
	);
	
	
	console #(
		.DIV (24'd12000000),
		.TIMEOUT (24'd120000),
		.MODE_PATTERN (`KUNTUR_VARIANT == 4'd3)
	) console0 (
		// inputs
		.clkin	(clkin),
		.wclk	(wclk),
		.con_rst_n	(con_rst_n),
		.lock_sync	(lock_sync),
		.cdr_rdy	(cdr_rdy),
		.cdr_valid	(cdr_valid),
		.phase	(phase),
		.eye_center	(eye_center),
		.eye_width	(eye_width),
		.pat_blocks	(pat_blocks),
		.pat_errors	(pat_errors),
		// outputs
		.con_tx	(con_tx)
	);
	

	wire [7:0] frx_dout;
	wire frx_hdr_start, frx_hdr_bad, frx_frame_end, frx_crc_ok, frx_is_sample;
	wire [31:0] frx_frame_index, fpa_smp_index;
	wire fpa_buf_we, fpa_smp_commit, fpa_smp_invalid;
	wire [6:0] fpa_buf_waddr;
	wire [7:0]fpa_smp_flags;
	wire [15:0] fpa_buf_wdata;
	wire fpa_cfg_strobe, fpa_sts_strobe;
	wire [175:0] fpa_cfg_regs;
	wire [15:0] fpa_sts_design_id;
	wire [7:0] fpa_sts_valid;
	wire [15:0] fpa_sts_alt_cmd;
	wire [63:0] fpa_sts_answer;
	wire [31:0] fpa_sts_fifo0_overflow, fpa_frames_received, fpa_frames_crc_failed, fpa_frame_index_gaps, fpa_frames_cut;
	reg frx_lock, lock_sync_reg;

	// T5
	
	// synchronizer
	always@(negedge cdr_rstb or posedge wclk)
		if (!cdr_rstb) begin
			lock_sync_reg <= 1'b0;
			frx_lock <= 1'b0;
		end
		else begin
			lock_sync_reg <= lock_sync;
			frx_lock <= lock_sync_reg;
		end
	
	
	frame_rx #(
		.STORED_PHASE	(9'd200),
		.LOCK_TIMEOUT	(9'd100),
		.N_FAIL			(4'd8)
	) frame_rx_0 (
		// inputs
		.rstb	(cdr_rstb),
		.clk	(clkin),
		.wclk	(wclk),
		.lock		(frx_lock),
		.din		(kuntur_data_8b),
		.hdr_bad		(frx_hdr_bad),
		.frame_end		(frx_frame_end),
		.crc_ok			(frx_crc_ok),
		.is_sample		(frx_is_sample),
		.frame_index	(frx_frame_index),
		// outputs
		.pll_phasestep	(frx_phasestep),
		.pll_phasedir	(frx_phasedir),
		.dout		(frx_dout),
		.hdr_start	(frx_hdr_start),
		.dout_en	(),
		.locked		(),
		.stepping	(),
		.bitsel		(),
		.relocks	()		
	);
	
	frame_parse frame_parse_0 (
		// inputs
		.rstb	(cdr_rstb),
		.wclk	(wclk),
		.din	(frx_dout),	// 8b dout
		.hdr_start	(frx_hdr_start),
		// outputs
		.hdr_bad	(frx_hdr_bad),
		.frame_end	(frx_frame_end),
		.crc_ok	(frx_crc_ok),
		.is_sample	(frx_is_sample),
		.frame_index	(frx_frame_index),
		.buf_we		(fpa_buf_we),
		.buf_waddr	(fpa_buf_waddr),	// 7b
		.buf_wdata	(fpa_buf_wdata),	// 16b
		.smp_commit	(fpa_smp_commit),
		.smp_invalid	(fpa_smp_invalid),
		.smp_index	(fpa_smp_index),	//32b
		.smp_flags	(fpa_smp_flags),	// 8b
		.cfg_strobe	(fpa_cfg_strobe),
		.cfg_regs	(fpa_cfg_regs),	// 176b
		.sts_strobe	(fpa_sts_strobe),
		.sts_design_id	(fpa_sts_design_id),	// 16b
		.sts_valid	(fpa_sts_valid),	// 8b
		.sts_alt_cmd	(fpa_sts_alt_cmd),	// 16b
		.sts_alt_answer	(fpa_sts_answer),	// 64b
		.sts_fifo0_overflow	(fpa_sts_fifo0_overflow),	// 32b
		.frames_received	(fpa_frames_received),	// 32b
		.frames_crc_failed	(fpa_frames_crc_failed),	// 32b
		.frame_index_gaps	(fpa_frame_index_gaps),	// 32b
		.frames_cut	(fpa_frames_cut)	// 32b
	);
	
	
	
	




endmodule








module triple_buffer (
	// inputs - wclk
	input  wire	rstb_w,		// cdr_rstb
	input  wire wclk,
	input  wire	buf_we,
	input  wire	[6:0]	buf_waddr,
	input  wire	[15:0]	buf_wdata,
	input  wire	smp_commit,
	input  wire	smp_invalid,
	input  wire	[31:0]	smp_index,
	input  wire	[7:0]	smp_flags,
	// inputs - sclk
	input  wire	rstb_s,		// rst_n_emu
	input  wire	sck,		// spi0_sck
	input  wire	convert_0,
	input  wire	[4:0]	rd_slot,	// CONVERT(C), C= 0...31
	input  wire	link_down,
	// outputs - sclk
	output wire	[15:0]	rd_a0,
	output wire	[15:0]	rd_b0,
	output wire	[15:0]	rd_a1,
	output wire	[15:0]	rd_b1,
	output wire	rd_valid,
	output wire	[31:0]	serve_index,
	output wire	[7:0]	serve_flag,
	output wire	[31:0]	slip_repeat,
	output wire	[31:0]	slip_skip,
	output wire	[31:0]	frames_sentinel_served
);






endmodule












module frame_parse (
	// inputs
	input  wire	rstb,
	input  wire	wclk,
	input  wire	[7:0]	din,	// dout
	input  wire	hdr_start,
	// outputs
	output reg	hdr_bad,
	output reg	frame_end,
	output reg	crc_ok,
	output reg	is_sample,
	output reg	[31:0]	frame_index,
	output reg	buf_we,
	output reg	[6:0]	buf_waddr,
	output reg	[15:0]	buf_wdata,
	output reg	smp_commit,
	output reg	smp_invalid,
	output reg	[31:0]	smp_index,
	output reg	[7:0]	smp_flags,
	output reg	cfg_strobe,
	output reg	[175:0]	cfg_regs,
	output reg	sts_strobe,
	output reg	[15:0]	sts_design_id,
	output reg	[7:0]	sts_valid,
	output reg	[15:0]	sts_alt_cmd,
	output reg	[63:0]	sts_alt_answer,
	output reg	[31:0]	sts_fifo0_overflow,
	output reg	[31:0]	frames_received,
	output reg	[31:0]	frames_crc_failed,
	output reg	[31:0]	frame_index_gaps,
	output reg	[31:0]	frames_cut
);


	localparam HDR_LEN = 16'd8;
	localparam CRC_LEN = 16'd4;

	// state
	localparam IDLE = 4'd0;
	localparam HEADER = 4'd1;
	localparam PAYLOAD = 4'd2;
	localparam CRC32 = 4'd3;
	localparam RX_END_GOOD = 4'd4;
	localparam RX_END_BAD = 4'd5;
	
	// reg & wire
	reg [3:0] current_state,next_state;
	
	
	reg [15:0] byte_index, payload_len_reg;
	reg byte_index_en;
	wire hdr_check, crc_check;
	reg frames_crc_failed_en;
	
	reg [7:0] type, flags;
	reg [15:0] payload_len;
	reg crc_load, crc_en;
	reg [31:0] crc;
	wire [31:0] crc_out;
	reg frames_received_en, frame_index_gaps_en, frames_cut_en;
	reg [31:0] prev_index;
	reg [7:0] buf_wdata_lo;
	
	// staging for CONFIG/STATUS payload
	reg [7:0] stage [0:21];
	
	
	// byte_index counter
	always@(negedge rstb or posedge wclk)
		if (!rstb)
			byte_index <= 9'd0;
		else if (hdr_start)
			byte_index <= 9'd1;
		else if (byte_index_en)
			if ( byte_index != (payload_len_reg + HDR_LEN + CRC_LEN - 1) )
			byte_index <= byte_index + 1;
			
			
	// registers: payload_len_reg
	always@(negedge rstb or posedge wclk)
		if (!rstb)
			payload_len_reg <= 16'hffff;
		else begin
			if ( (current_state == HEADER) && (byte_index == 9'd7) )
				payload_len_reg <= { din, payload_len[7:0] };
			if ( (current_state == RX_END_GOOD) && (type == 8'd1) ) // SAMPLE and GOOD CRC
				prev_index <= frame_index;
		end
			
	
	// Sampling the input
	always@(negedge rstb or posedge wclk)
		if (!rstb) begin
			// header
			type <= 8'd0;
			flags <= 8'd0;
			frame_index <= 32'd0;
			payload_len <= 16'hffff;
			crc <= 32'd0;
		end
		else begin
			
			if (hdr_start)
				type <= din;
				
			case (byte_index)
				16'd1:	flags <= din;
				16'd2:	frame_index[7:0] <= din;
				16'd3:	frame_index[15:8] <= din;
				16'd4:	frame_index[23:16] <= din;
				16'd5:	frame_index[31:24] <= din;
				16'd6:	payload_len[7:0] <= din;
				16'd7:	payload_len[15:8] <= din;
				(payload_len_reg + HDR_LEN + CRC_LEN - 4):	crc[7:0] <= din;
				(payload_len_reg + HDR_LEN + CRC_LEN - 3):	crc[15:8] <= din;
				(payload_len_reg + HDR_LEN + CRC_LEN - 2):	crc[23:16] <= din;
				(payload_len_reg + HDR_LEN + CRC_LEN - 1):	crc[31:24] <= din;
				default:	;
			endcase
			
		end
	
	
	
	assign hdr_check =	( 	( (type == 8'd1) && ( {din, payload_len[7:0]} == 16'd256 )	)	||
							( (type == 8'd2) && ( {din, payload_len[7:0]} == 16'd22 )	)	|| 
							( (type == 8'd3) && ( {din, payload_len[7:0]} == 16'd18 )	) 	) ;
	
	assign crc_check = ( {din,crc[23:0]} == crc_out );
	
	
	// frames_crc_failed counter
	always@(negedge rstb or posedge wclk)
		if (!rstb)
			frames_crc_failed <= 32'd0;
		else if (frames_crc_failed_en && (frames_crc_failed != 32'hffffffff))
			frames_crc_failed <= frames_crc_failed + 1;
	
	// frames_received counter
	always@(negedge rstb or posedge wclk)
		if (!rstb)
			frames_received <= 32'd0;
		else if (frames_received_en && (frames_received != 32'hffffffff))
			frames_received <= frames_received + 1;
	
	// frame_index_gaps counter
	always@(negedge rstb or posedge wclk)
		if (!rstb)
			frame_index_gaps <= 32'd0;
		else if ( frame_index_gaps_en && (frame_index_gaps != 32'hffffffff))
			frame_index_gaps <= frame_index_gaps + 1;
	
	// frames_cut counter
	always@(negedge rstb or posedge wclk)
		if (!rstb)
			frames_cut <= 32'd0;
		else if (frames_cut_en && (frames_cut != 32'hffffffff))
			frames_cut <= frames_cut + 1;
	
	
	
	// CRC
	
	crc32 checksum (
		.rst_n	(rstb),
		.load	(crc_load),
		.en		(crc_en),
		.clk	(wclk),
		.din	(din),
		.crc_out	(crc_out)
	);
	
	
	reg [6:0] stage_i;
	
	// registers: outputs
	always@(negedge rstb or posedge wclk)
		if (!rstb) begin
			buf_we <= 1'b0;
			buf_waddr <= 7'd0;
			buf_wdata <= 16'd0;
			smp_commit <= 1'b0;
			smp_invalid <= 1'b0;
			smp_index <= 32'd0;
			smp_flags <= 8'd0;
			cfg_strobe <= 1'b0;
			cfg_regs <= 176'd0;
			sts_strobe <= 1'b0;
			sts_design_id <= 16'd0;
			sts_valid <= 8'd0;
			sts_alt_cmd <= 16'd0;
			sts_alt_answer <= 64'd0;
			sts_fifo0_overflow <= 32'd0;
			
			buf_wdata_lo <= 8'd0;
		end
		else begin
			
			buf_we <= 1'b0;
			smp_commit <= 1'b0;
			smp_invalid <= 1'b0;
			cfg_strobe <= 1'b0;
			sts_strobe <= 1'b0;
			
			if ( (current_state == PAYLOAD) && (type == 8'd1) ) begin
				buf_wdata_lo <= din;
				buf_we <= byte_index[0] && !hdr_start;
				buf_waddr <= (byte_index - (HDR_LEN[6:0]+1)) >> 1;
				buf_wdata <= {din, buf_wdata_lo};
			end
			
			if ( (current_state == PAYLOAD) && (type != 8'd1) ) begin
				stage_i = byte_index[6:0] - HDR_LEN[6:0];
				stage[stage_i[4:0]] <= din;
			end
			
			if ( (current_state == RX_END_GOOD) && (type == 8'd1) ) begin // SAMPLE and GOOD CRC
				smp_commit <= 1'b1;
				smp_index <= frame_index;
				smp_flags <= flags;
			end
			
			if ( hdr_start && (type == 8'd1) && ( (current_state == HEADER) || (current_state == PAYLOAD) || (current_state == CRC32)  ) ) begin // SAMPLE and Restart
				smp_invalid <= 1'b1;
			end
			
			if ( (current_state == HEADER) && (type == 8'd1) && (byte_index == 16'd7) && (!hdr_check) && (!hdr_start) ) begin // SAMPLE and BAD HEADER
				smp_invalid <= 1'b1;
			end
			
			if ( (current_state == RX_END_BAD) && (type == 8'd1) ) begin // SAMPLE and BAD CRC
				smp_invalid <= 1'b1;
			end

			if ( (current_state == RX_END_GOOD) && (type == 8'd2) ) begin // CONFIG and GOOD CRC
				cfg_strobe <= 1'b1;
				cfg_regs <= {	stage[21], stage[20], stage[19], stage[18], stage[17], stage[16], stage[15], stage[14], stage[13], stage[12], stage[11],
								stage[10],  stage[9],  stage[8],  stage[7],  stage[6],  stage[5],  stage[4],  stage[3],  stage[2],  stage[1],  stage[0]	};
			end
			
			if ( (current_state == RX_END_GOOD) && (type == 8'd3) ) begin // STATUS and GOOD CRC
				sts_strobe <= 1'b1;
				sts_design_id <= { stage[1], stage[0] };
				sts_valid <= stage[2];
				sts_alt_cmd <= { stage[5], stage[4] };
				sts_alt_answer <= { stage[13], stage[12], stage[11], stage[10], stage[9], stage[8], stage[7], stage[6] };
				sts_fifo0_overflow <= { stage[17], stage[16], stage[15], stage[14] };
			end
			
		end
	
	
	

	
	// ******************************************************
	// FSM
	// ******************************************************
	
	// state register
	always@(negedge rstb or posedge wclk)
		if (!rstb)
			current_state <= IDLE;
		else
			current_state <= next_state;
	
	// next_state logic
	always@(*) begin
		// default values
		frames_crc_failed_en = 1'b0;
		byte_index_en = 1'b0;
		crc_load = 1'b0;
		crc_en = 1'b0;
		frame_end = 1'b0;
		crc_ok = 1'b0;
		is_sample = 1'b0;
		frames_received_en = 1'b0;
		frame_index_gaps_en = 1'b0;
		frames_cut_en = 1'b0;
		hdr_bad = 1'b0;
		
		case (current_state)
			IDLE: begin
				if (hdr_start)
					next_state = HEADER;
				else
					next_state = IDLE;
				crc_load = hdr_start;
				crc_en = 1'b1;
			end
			
			HEADER: begin
				if (hdr_start) begin
					next_state = HEADER;
					crc_load = hdr_start;
					crc_en = 1'b1;
					frames_cut_en = 1'b1;
				end
				else if (byte_index == 16'd7)
					if (hdr_check) begin
						// HDR is correct
						next_state = PAYLOAD;
					end
					else begin
						next_state = IDLE;
						frames_crc_failed_en = 1'b1;
						hdr_bad = 1'b1;
					end
				else
					next_state = HEADER;
				crc_en = 1'b1;
				byte_index_en = 1'b1;
			end
			
			PAYLOAD: begin
				if (hdr_start) begin
					next_state = HEADER;
					crc_load = hdr_start;
					crc_en = 1'b1;
					frames_cut_en = 1'b1;
				end
				else if	(byte_index == (payload_len_reg + HDR_LEN - 1))
					next_state = CRC32;
				else
					next_state = PAYLOAD;
				crc_en = 1'b1;
				byte_index_en = 1'b1;
			end
			
			CRC32: begin
				// CRC check
				if (hdr_start) begin
					next_state = HEADER;
					crc_load = hdr_start;
					crc_en = 1'b1;
					frames_cut_en = 1'b1;
				end
				else if	(byte_index == (payload_len_reg + HDR_LEN + CRC_LEN - 1))
					if (crc_check) begin
						next_state = RX_END_GOOD;
						if (type == 8'd1) begin
							frames_received_en = 1'b1;
							if ( (frames_received != 0) && (frame_index != (prev_index + 1)) )
								frame_index_gaps_en = 1'b1;
						end
					end
					else begin
						next_state = RX_END_BAD;
						frames_crc_failed_en = 1'b1;
					end
				else begin
					next_state = CRC32;
					byte_index_en = 1'b1;
				end
			end
			
			RX_END_GOOD: begin
				if (hdr_start) begin
					next_state = HEADER;
					crc_load = hdr_start;
					crc_en = 1'b1;
				end
				else
					next_state = IDLE;
				frame_end = 1'b1;
				crc_ok = 1'b1;
				if (type == 8'd1) is_sample = 1'b1;
				
			end
			
			RX_END_BAD: begin
				if (hdr_start) begin
					next_state = HEADER;
					crc_load = hdr_start;
					crc_en = 1'b1;
				end
				else
					next_state = IDLE;
				frame_end = 1'b1;
				if (type == 8'd1) is_sample = 1'b1;
				
			end
			
			// Other States
			default: begin
				next_state = IDLE;
			end
		endcase
	end

endmodule






module crc32 (
	input  wire	rst_n,
	input  wire load,
	input  wire	en,
	input  wire	clk,
	input  wire	[7:0]	din,
	output wire	[31:0]	crc_out
);

	// CRC TYPE: CRC-32/ISO-HDLC
	// Shift right, poly 0xEDB88320, bit 0 of each byte first, and feedback includes the data bit
	// POLY: 1110 1101 1011 1000 1000 0011 0010 0000 1

	localparam CRC_INIT = 32'hFFFFFFFF;
	localparam CRC_POLY = 32'hEDB88320;
	
	reg [31:0] crc;
	wire [31:0] next_0, next_1, next_2, next_3, next_4, next_5, next_6, next_7, next_8;
	
	// initial value
	assign next_0 = ( load ? CRC_INIT : crc) ^ {24'd0,din};
	
	// 8 steps
	assign next_1 = (next_0[0]) ? ( {1'b0, next_0[31:1]} ^ CRC_POLY ) : ( {1'b0, next_0[31:1]} );
	assign next_2 = (next_1[0]) ? ( {1'b0, next_1[31:1]} ^ CRC_POLY ) : ( {1'b0, next_1[31:1]} );
	assign next_3 = (next_2[0]) ? ( {1'b0, next_2[31:1]} ^ CRC_POLY ) : ( {1'b0, next_2[31:1]} );
	assign next_4 = (next_3[0]) ? ( {1'b0, next_3[31:1]} ^ CRC_POLY ) : ( {1'b0, next_3[31:1]} );
	assign next_5 = (next_4[0]) ? ( {1'b0, next_4[31:1]} ^ CRC_POLY ) : ( {1'b0, next_4[31:1]} );
	assign next_6 = (next_5[0]) ? ( {1'b0, next_5[31:1]} ^ CRC_POLY ) : ( {1'b0, next_5[31:1]} );
	assign next_7 = (next_6[0]) ? ( {1'b0, next_6[31:1]} ^ CRC_POLY ) : ( {1'b0, next_6[31:1]} );
	assign next_8 = (next_7[0]) ? ( {1'b0, next_7[31:1]} ^ CRC_POLY ) : ( {1'b0, next_7[31:1]} );
	
	// CRC regsiter
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			crc <= CRC_INIT;
		else if (en)
			crc <= next_8;
	
	assign crc_out = ~crc;

endmodule












module frame_rx #(
	parameter	STORED_PHASE	=	9'd200,
	parameter	LOCK_TIMEOUT	=	9'd100,
	parameter	N_FAIL			=	4'd8
)(
	// inputs
	input  wire	rstb,
	input  wire	clk,
	input  wire	wclk,
	input  wire	lock,
	input  wire	[7:0]	din,
	input  wire	hdr_bad,
	input  wire	frame_end,
	input  wire	crc_ok,
	input  wire	is_sample,
	input  wire	[31:0]	frame_index,
	// outputs
	output reg	pll_phasestep,
	output wire	pll_phasedir,
	output wire	[7:0]	dout,
	output reg	hdr_start,
	output reg	dout_en,
	output reg	locked,
	output wire	stepping,
	output reg	[2:0]	bitsel,
	output reg	[31:0]	relocks		
);

	
	// parameters
	localparam PREAMBLE = 32'h5AA55AA5;
	localparam FRAME_LENGTH = 9'd380;
	localparam SEARCH_LENGTH = 9'd390; // > frame_length + 2 (bit-shifter) + 4 (preamble)
	
	// state
	localparam IDLE = 4'd0;
	localparam PHASE_STEPS_HI = 4'd1;
	localparam PHASE_STEPS_LO = 4'd2;
	localparam WAIT = 4'd3;
	localparam HUNT = 4'd4;
	localparam FRAME = 4'd5;
	localparam SEEK = 4'd6;
	localparam op6 = 4'd7;
	
	// reg & wire
	reg [3:0] current_state,next_state;
	reg [7:0] din_reg_0,din_reg_1;
	reg [7:0] data_sa, data_sa_reg_0, data_sa_reg_1, data_sa_reg_2, data_sa_reg_3;
	wire data_is_preamble, bitsel_is_max;
	reg bitsel_aligned_en, bitsel_en, rst_sync_2;
	
	reg [8:0] cnt0, cnt1;
	reg cnt0_rst, cnt0_en, cnt1_rst, cnt1_en;
	reg phasestep, phasestep_reg, phasestep_reg_1;
	wire [2:0] bitsel_2;
	
	
	
	// phasestep reg and synchronizer
	always@(negedge rstb or posedge wclk)
		if (!rstb)
			phasestep_reg <= 1'b1;
		else
			phasestep_reg <= phasestep;
	
	always@(negedge rstb or posedge clk)
		if (!rstb) begin
			pll_phasestep <= 1'b1;
			phasestep_reg_1 <= 1'b1;
		end
		else begin
			pll_phasestep <= phasestep_reg_1;
			phasestep_reg_1 <= phasestep_reg;
		end

	
	
	// cnt0: Auxiliary counter
	always@(posedge wclk)
		if (cnt0_rst)
			cnt0 <= 9'd0;
		else if (cnt0_en)
			cnt0 <= cnt0 + 1;
	
	// cnt1: Auxiliary counter
	always@(posedge wclk)
		if (cnt1_rst)
			cnt1 <= 9'd0;
		else if (cnt1_en)
			cnt1 <= cnt1 + 1;
	
	
	// pll_phasedir
	assign pll_phasedir = 1'b0;
	
	// stepping
	assign stepping = ( (current_state == PHASE_STEPS_HI) || (current_state == PHASE_STEPS_LO));
	
	
	// ******************************************************
	// SEARCH FRAME START: OFFSET and PREAMBLE - BEGIN
	// ******************************************************
	
	// first register negedge clk
	always@(negedge wclk)
		if (rst_sync_2)
			din_reg_0 <= 8'h00;
		else
			din_reg_0 <= din;
	
	// second register posedge
	always@(posedge wclk)
		if (rst_sync_2)
			din_reg_1 <= 8'h00;
		else
			din_reg_1 <= din_reg_0;
	
	
	reg search_bits;
	
	assign bitsel_2 = (search_bits) ? cnt1[2:0]:bitsel;	
	
	
	// registers after bit-shift
	always@(posedge wclk)
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
		case (bitsel_2)
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
	
	
	
	
	
	// position, bitsel and data_out
	always@(posedge wclk)
		if (rst_sync_2) begin
			bitsel <= 3'd0;
			dout_en <= 1'b0;
		end
		else
			if(bitsel_aligned_en) begin
				bitsel <= bitsel_2;
			end
			else if (hdr_start)
				dout_en <= 1'b1;
			else if (frame_end || hdr_bad)
				dout_en <= 1'b0;
	
	
	assign dout = data_sa_reg_0;
	
	
	
	// ******************************************************
	// SAVE PREVIOUS FRAME_INDEX & COUNT ERRORS/FAILS
	// ******************************************************
	
	reg [31:0] frame_index_reg;
	reg [7:0] fail_cnt;
	reg fail_cnt_en, fail_cnt_rst, relocks_en, relocks_rst, first_sample;
	reg locked_fsm, locked_en, hdr_start_fsm, first_sample_fsm, first_sample_en;
	
	// frame_index_reg
	always@(negedge rstb or posedge wclk)
		if (!rstb)
			frame_index_reg <= 32'd0;
		else if (frame_end && crc_ok && is_sample)
			frame_index_reg <= frame_index;


	// fail counter
	always@(posedge wclk)
		if (fail_cnt_rst)
			fail_cnt <= 0;
		else if (fail_cnt_en)
			fail_cnt <= fail_cnt + 1;
	
	
	// relocks counter
	always@(posedge wclk)
		if (relocks_rst)
			relocks <= 0;
		else if (relocks_en)
			relocks <= relocks + 1;
	
	// output registers
	always@(negedge rstb or posedge wclk)
		if (!rstb) begin
			hdr_start <= 1'b0;
			locked <= 1'b0;
			first_sample <= 1'b0;
		end
		else begin
			hdr_start <= hdr_start_fsm;
			if (locked_en)
				locked <= locked_fsm;
			if (first_sample_en)
				first_sample <= first_sample_fsm;
		end
	
	
	// ******************************************************
	// FSM
	// ******************************************************
	
	// state register
	always@(posedge wclk)
		if (!rstb)
			current_state <= IDLE;
		else
			current_state <= next_state;
	
	// next_state logic
	always@(*) begin
		// default values
		bitsel_aligned_en = 1'b0;
		rst_sync_2 = 1'b0;		
		
		cnt0_rst = 1'b0;
		cnt0_en = 1'b0;
		cnt1_rst = 1'b0;
		cnt1_en = 1'b0;
		phasestep = 1'b1;
		search_bits = 1'b0;
		fail_cnt_rst = 1'b0;
		fail_cnt_en = 1'b0;
		relocks_rst = 1'b0;
		relocks_en = 1'b0;
		locked_fsm = 1'b0;
		locked_en = 1'b0;
		hdr_start_fsm = 1'b0;
		first_sample_en = 1'b0;
		first_sample_fsm = 1'b0;
		
		case (current_state)
			IDLE: begin
				if (lock) // checks if PLL is locked
					if (STORED_PHASE == 0)
						next_state = WAIT;
					else
						next_state = PHASE_STEPS_HI;
				else
					next_state = IDLE;
				cnt0_rst = 1'b1;
				cnt1_rst = 1'b1;
				fail_cnt_rst = 1'b1;
				relocks_rst = 1'b1;
			end
			
			PHASE_STEPS_HI: begin
				if (cnt0 == 9'd7) begin // cnt0 sets time phasestep = 1
					next_state = PHASE_STEPS_LO;
					cnt0_rst = 1'b1;
				end
				else begin
					next_state = PHASE_STEPS_HI;
					cnt0_en = 1'b1;
				end
				phasestep = 1'b1;
			end
			
			PHASE_STEPS_LO: begin
				if ( (cnt1 == STORED_PHASE-1) && (cnt0 == 9'd7) ) begin // cnt1 sets # of phasesteps
					next_state = WAIT;
					cnt0_rst = 1'b1;
					cnt1_rst = 1'b1;
				end
				else if (cnt0 == 9'd7) begin // cnt0 sets time phasestep = 0
					next_state = PHASE_STEPS_HI;
					cnt0_rst = 1'b1;
					cnt1_en = 1'b1;
				end
				else begin
					next_state = PHASE_STEPS_LO;
					cnt0_en = 1'b1;
				end
				phasestep = 1'b0;
			end
			
			
			WAIT: begin
				if (cnt0 == 9'd7) begin // cnt0 sets waiting time
					next_state = HUNT;
					cnt0_rst = 1'b1;
					cnt1_rst = 1'b1;
					rst_sync_2 = 1'b1;
					fail_cnt_rst = 1'b1;
					first_sample_fsm = 1'b0;
					first_sample_en = 1'b1;
				end
				else begin
					next_state = WAIT;
					cnt0_en = 1'b1;
				end
			end
			
			
			HUNT: begin
				search_bits = 1'b1;
				if (!lock) begin // checks if PLL is locked
					next_state = HUNT;
					locked_fsm = 1'b0; // clears 'locked'
					locked_en = 1'b1;
					cnt0_rst = 1'b1;
					cnt1_rst = 1'b1;
					fail_cnt_rst = 1'b1;
					relocks_en = locked;
				end
				else if (data_is_preamble) begin
					next_state = FRAME; // PREAMBLE found
					bitsel_aligned_en = 1'b1;
					hdr_start_fsm = 1'b1;
				end
				else if (cnt0 == SEARCH_LENGTH) begin // cnt0 is the length of the search window
					if (cnt1 == 9'd7) begin // cnt1 is the 8 different offsets
						next_state = HUNT; // PREAMBLE not found
						cnt0_rst = 1'b1;
						cnt1_rst = 1'b1;
					end
					else begin
						next_state = HUNT;
						cnt1_en = 1'b1;	// next bitsel
						rst_sync_2 = 1'b1;	// clear window and shit registers
						cnt0_rst = 1'b1;// clear window and shit registers
					end
				end
				else begin
					next_state = HUNT;
					cnt0_en = 1'b1;
				end
			end
			
			
			FRAME: begin
				cnt0_en = 1'b1; // cnt0 is counting LOCK_TIMEOUT
				next_state = FRAME;
				
				if (!lock) begin // checks if PLL is locked
					next_state = HUNT;
					locked_fsm = 1'b0; // clears 'locked'
					locked_en = 1'b1;
					cnt0_rst = 1'b1;
					cnt1_rst = 1'b1;
					fail_cnt_rst = 1'b1;
					relocks_en = locked;
					first_sample_fsm = 1'b0;
					first_sample_en = 1'b1;
				end
				else if ( hdr_bad || (frame_end && !crc_ok) ) begin
					next_state = HUNT;
					cnt0_rst = 1'b1;
					cnt1_rst = 1'b1;
					first_sample_fsm = 1'b0;
					first_sample_en = 1'b1;
					if (locked) begin
						if (fail_cnt == N_FAIL-1) begin
							relocks_en = 1'b1;
							next_state = HUNT;
							locked_fsm = 1'b0; // clears 'locked'
							locked_en = 1'b1;
							fail_cnt_rst = 1'b1; // resets fail counter
							first_sample_fsm = 1'b0;
							first_sample_en = 1'b1;
						end
						else begin
							fail_cnt_en = 1'b1;
							next_state = SEEK;
							first_sample_fsm = 1'b0;
							first_sample_en = 1'b0;
						end
					end
				end
				else if ( frame_end && crc_ok && is_sample ) begin
					// GOOD SAMPLE
					if (frame_index == frame_index_reg +1) begin
						if (first_sample) begin
							locked_fsm = 1'b1;
							locked_en = 1'b1;
						end
						next_state = SEEK;
						cnt0_rst = 1'b1;
						cnt1_rst = 1'b1;
					end
					else begin
						next_state = SEEK;
						cnt0_rst = 1'b1;
						cnt1_rst = 1'b1;
					end
					if (!first_sample) begin
						first_sample_fsm = 1'b1;
						first_sample_en = 1'b1;
					end
					if (locked)
						fail_cnt_rst = 1'b1;
				end
				else if ( frame_end && crc_ok && !is_sample ) begin
					next_state = SEEK;
					cnt0_rst = 1'b1;
					cnt1_rst = 1'b1;
				end
			end
			
			
			SEEK: begin
				cnt1_en = 1'b1;
				
				if (!lock) begin // checks if PLL is locked
					next_state = HUNT;
					locked_fsm = 1'b0; // clears 'locked'
					locked_en = 1'b1;
					cnt0_rst = 1'b1;
					cnt1_rst = 1'b1;
					fail_cnt_rst = 1'b1;
					relocks_en = locked;
					first_sample_fsm = 1'b0;
					first_sample_en = 1'b1;
				end
				else if (data_is_preamble) begin
					next_state = FRAME; // PREAMBLE found
					hdr_start_fsm = 1'b1;
				end
				else if (cnt0 == LOCK_TIMEOUT) begin // cnt0 is counting frames
					next_state = HUNT;
					cnt0_rst = 1'b1;// clear window and shit registers
					locked_fsm = 1'b0; // clears 'locked'
					locked_en = 1'b1;
					fail_cnt_rst = 1'b1; // resets fail counter
				end
				else begin
					next_state = SEEK;
					if (cnt1 == FRAME_LENGTH-1) begin // cnt1 is counting frame_length
						cnt0_en = 1'b1;
						cnt1_rst = 1'b1;
						if (locked) begin
							if (fail_cnt == N_FAIL-1) begin
								relocks_en = 1'b1;
								next_state = HUNT;
								locked_fsm = 1'b0; // clears 'locked'
								locked_en = 1'b1;
								fail_cnt_rst = 1'b1; // resets fail counter
							end
							else begin
								fail_cnt_en = 1'b1;
								next_state = SEEK;
							end
						end
						
					end
				end
			end
			
			// Other States
			default: begin
				next_state = IDLE;
			end
		endcase
	end




endmodule


















module console_por (
	input  wire clk,
	output wire	con_rst_n
);
	
	localparam POR_CYCLES = 8'd16;
	localparam CNT0_BITS = 5;
	wire [CNT0_BITS-1:0] cnt0, cnt0_d;
	wire cnt0_en, cnt0_is_max;
	
	genvar i;
	
	// cnt0 starting at 0
	generate
		for ( i=0; i<CNT0_BITS; i = i+1 ) begin: por_cnt0
			FD1P3DX #(.GSR("ENABLED")) ff0 ( .D(cnt0_d[i]), .SP(cnt0_en), .CK(clk), .CD(1'b0), .Q(cnt0[i]) );
		end
	endgenerate
	
	assign cnt0_d = cnt0 + 5'd1;
	assign cnt0_en = ~con_rst_n;


	// con_rst_n starting at 0 for POR_CYCLES 
	assign cnt0_is_max = (cnt0 == POR_CYCLES-1);
	FD1P3DX #(.GSR("ENABLED")) ff1 ( .D(1'b1), .SP(cnt0_is_max), .CK(clk), .CD(1'b0), .Q(con_rst_n) );

endmodule








module console #(
	parameter DIV = 24'd12000000,
	parameter TIMEOUT = 24'd120000,
	parameter MODE_PATTERN = 1'b1
)(
	input  wire	clkin,
	input  wire	wclk,
	input  wire	con_rst_n,
	input  wire	lock_sync,
	input  wire	cdr_rdy,
	input  wire	cdr_valid,
	input  wire	[8:0]	phase,
	input  wire	[8:0]	eye_center,
	input  wire	[8:0]	eye_width,
	input  wire	[31:0]	pat_blocks,
	input  wire	[31:0]	pat_errors,
	output wire	con_tx
);
	
	wire tick;
	wire [31:0] t;
	
	wire snap_req;
	wire sh_cdr_rdy;
	wire sh_cdr_valid;
	wire [8:0] sh_phase;
	wire [8:0] sh_eye_center;
	wire [8:0] sh_eye_width;
	wire [31:0] sh_pat_blocks;
	wire [31:0] sh_pat_errors;
	wire snap_ack;
	
	
	wire [31:0] h_t;
	wire h_lock;
	wire h_snap_ok;
	wire h_cdr_rdy;
	wire h_cdr_valid;
	wire [8:0]	h_phase;
	wire [8:0]	h_eye_center;
	wire [8:0]	h_eye_width;
	wire [31:0]	h_pat_blocks;
	wire [31:0]	h_pat_errors;
	wire go;
	
	
	wire snap_ok;
	wire uart_rdy;
	wire uart_start;
	wire [7:0] uart_data;

	
	
	console_timer #(
		.DIV (DIV)
	) console_timer_0 (
		.clk	(clkin),	// input
		.rst_n	(con_rst_n),	// input
		.tick	(tick),	// output
		.t		(t)	// output [31:0]
	);
	
	
	console_shadow console_shadow_0 (
		// inputs
		.rst_n	(con_rst_n),
		.wclk	(wclk),
		.snap_req	(snap_req),
		.cdr_rdy	(cdr_rdy),
		.cdr_valid	(cdr_valid),
		.phase	(phase),
		.eye_center	(eye_center),
		.eye_width	(eye_width),
		.pat_blocks	(pat_blocks),
		.pat_errors	(pat_errors),
		// outputs
		.sh_cdr_rdy	(sh_cdr_rdy),
		.sh_cdr_valid	(sh_cdr_valid),
		.sh_phase	(sh_phase),
		.sh_eye_center	(sh_eye_center),
		.sh_eye_width	(sh_eye_width),
		.sh_pat_blocks	(sh_pat_blocks),
		.sh_pat_errors	(sh_pat_errors),
		.snap_ack	(snap_ack)
	);
	
	
	
	
	console_snap #(
		.TIMEOUT (TIMEOUT)
	) console_snap_0 (
		// inputs
		.clk	(clkin),
		.rst_n	(con_rst_n),
		.tick	(tick),
		.t		(t),
		.lock_sync	(lock_sync),
		.snap_ack	(snap_ack),
		.sh_cdr_rdy	(sh_cdr_rdy),
		.sh_cdr_valid	(sh_cdr_valid),
		.sh_phase	(sh_phase),
		.sh_eye_center	(sh_eye_center),
		.sh_eye_width	(sh_eye_width),
		.sh_pat_blocks	(sh_pat_blocks),
		.sh_pat_errors	(sh_pat_errors),
		// outputs
		.h_t	(h_t),
		.h_lock	(h_lock),
		.h_snap_ok	(h_snap_ok),
		.h_cdr_rdy	(h_cdr_rdy),
		.h_cdr_valid	(h_cdr_valid),
		.h_phase	(h_phase),
		.h_eye_center	(h_eye_center),
		.h_eye_width	(h_eye_width),
		.h_pat_blocks	(h_pat_blocks),
		.h_pat_errors	(h_pat_errors),
		.snap_req	(snap_req),
		.go	(go)
	);
	
	
	
	console_fmt #(
		.MODE_PATTERN (MODE_PATTERN)
	) console_fmt_0 (
		// inputs
		.clk	(clkin),
		.rst_n	(con_rst_n),
		.go	(go),
		.t	(h_t),
		.lock	(h_lock),
		.snap_ok	(h_snap_ok),
		.cdr_rdy	(h_cdr_rdy),
		.cdr_valid	(h_cdr_valid),
		.phase	(h_phase),
		.eye_center	(h_eye_center),
		.eye_width	(h_eye_width),
		.pat_blocks	(h_pat_blocks),
		.pat_errors	(h_pat_errors),
		.uart_rdy	(uart_rdy),
		// outputs
		.uart_start	(uart_start),
		.uart_data	(uart_data),
		.busy	()
	);
	
	
	
	
	
	console_uart uart0(
		// inputs
		.clk	(clkin),
		.rst_n	(con_rst_n),
		.start	(uart_start),
		.data_tx	(uart_data),
		// outputs
		.con_tx	(con_tx),
		.rdy	(uart_rdy)
	);

	
	

endmodule








module console_timer #(
	parameter DIV = 24'd12000000
)(
	input  wire clk,
	input  wire	rst_n,
	output reg	tick,
	output reg	[31:0] t
);
	
	reg [23:0] cnt0;
	
	
	always@(negedge rst_n or posedge clk)
		if (!rst_n) begin
			cnt0 <= 24'd0;
			tick <= 1'b0;
			t <= 32'd0;
		end
		else if (cnt0 == DIV-1) begin
			cnt0 <= 24'd0;
			tick <= 1'b1;
			if (t < 32'hffffffff)
				t <= t + 32'd1;
		end
		else begin
			cnt0 <= cnt0 + 24'd1;
			tick <= 1'b0;
		end
	
endmodule



module console_shadow (
	input  wire	rst_n,
	input  wire	wclk,
	input  wire	snap_req,
	input  wire	cdr_rdy,
	input  wire	cdr_valid,
	input  wire	[8:0]	phase,
	input  wire	[8:0]	eye_center,
	input  wire	[8:0]	eye_width,
	input  wire	[31:0]	pat_blocks,
	input  wire	[31:0]	pat_errors,
	output reg	sh_cdr_rdy,
	output reg	sh_cdr_valid,
	output reg	[8:0]	sh_phase,
	output reg	[8:0]	sh_eye_center,
	output reg	[8:0]	sh_eye_width,
	output reg	[31:0]	sh_pat_blocks,
	output reg	[31:0]	sh_pat_errors,
	output reg	snap_ack
);

	reg req_s1, req_s2, req_s3;
	wire req_t;
	reg rst_w1, rst_w2;
	
	
	// reset pulse synchronizer - async rst_n
	always@(negedge rst_n or posedge wclk)
		if (!rst_n) begin
			rst_w1 <= 1'b0;
			rst_w2 <= 1'b0;
		end
		else begin
			rst_w1 <= 1'b1;
			rst_w2 <= rst_w1;
		end
	
		
	// synchronizer
	always@(negedge rst_w2 or posedge wclk)
		if (!rst_w2) begin
			req_s1 <= 1'b0;
			req_s2 <= 1'b0;
			req_s3 <= 1'b0;
		end
		else begin
			req_s1 <= snap_req;
			req_s2 <= req_s1;
			req_s3 <= req_s2;
	end
	
	assign req_t = req_s3 ^ req_s2;
	
	
	// toggle
	always@(negedge rst_w2 or posedge wclk)
		if (!rst_w2)
			snap_ack <= 1'b0;
		else if (req_t)
			snap_ack <= ~snap_ack;
	
	
	// copy
	always@(negedge rst_w2 or posedge wclk)
		if (!rst_w2) begin
			sh_cdr_rdy <= 1'b0;
			sh_cdr_valid <= 1'b0;
			sh_phase <= 9'd0;
			sh_eye_center <= 9'd0;
			sh_eye_width <= 9'd0;
			sh_pat_blocks <= 32'd0;
			sh_pat_errors <= 32'd0;
		end
		else if (req_t) begin
			sh_cdr_rdy <= cdr_rdy;
			sh_cdr_valid <= cdr_valid;
			sh_phase <= phase;
			sh_eye_center <= eye_center;
			sh_eye_width <= eye_width;
			sh_pat_blocks <= pat_blocks;
			sh_pat_errors <= pat_errors;
		end
	
endmodule








module console_snap #(
	parameter TIMEOUT = 24'd120000
)(
	// inputs
	input  wire	clk,
	input  wire	rst_n,
	input  wire	tick,
	input  wire	[31:0] t,
	input  wire	lock_sync,
	input  wire	snap_ack,
	input  wire	sh_cdr_rdy,
	input  wire	sh_cdr_valid,
	input  wire	[8:0]	sh_phase,
	input  wire	[8:0]	sh_eye_center,
	input  wire	[8:0]	sh_eye_width,
	input  wire	[31:0]	sh_pat_blocks,
	input  wire	[31:0]	sh_pat_errors,
	// outputs
	output reg	[31:0] h_t,
	output reg	h_lock,
	output reg	h_snap_ok,
	output reg	h_cdr_rdy,
	output reg	h_cdr_valid,
	output reg	[8:0]	h_phase,
	output reg	[8:0]	h_eye_center,
	output reg	[8:0]	h_eye_width,
	output reg	[31:0]	h_pat_blocks,
	output reg	[31:0]	h_pat_errors,
	output reg	snap_req,
	output reg	go
);

	reg [2:0] snap_a;
	wire ack_t, pending;
	reg window, go_capture;
	reg [23:0] cnt0;
	
	// synchronizer
	always@(posedge clk)
		snap_a <= {snap_a[1:0],snap_ack};
	
	assign ack_t = snap_a[2] ^ snap_a[1];
	assign pending = snap_req ^ snap_a[1];
	
	// TIMEOUT Window 
	always@(negedge rst_n or posedge clk)
		if (!rst_n) begin
			cnt0 <= 24'd0;
			snap_req <= 1'b0;
			window <= 1'b0;
			h_t <= 32'd0;
			h_lock <= 1'b0;
			h_snap_ok <= 1'b0;
			h_cdr_rdy <= 1'b0;
			h_cdr_valid <= 1'b0;
			h_phase <= 9'd0;
			h_eye_center <= 9'd0;
			h_eye_width <= 9'd0;
			h_pat_blocks <= 32'd0;
			h_pat_errors <= 32'd0;
			go_capture <= 1'b0;
		end
		else begin
			go_capture <= 1'b0;
			if (tick) begin
				window <= 1'b1;
				cnt0 <= 24'd0;
				if (!pending)
					snap_req <= ~snap_req;
			end
			else if (window) begin
				if (ack_t) begin
					h_t <= t;
					h_lock <= lock_sync;
					h_snap_ok <= 1'b1;
					h_cdr_rdy <= sh_cdr_rdy;
					h_cdr_valid <= sh_cdr_valid;
					h_phase <= sh_phase;
					h_eye_center <= sh_eye_center;
					h_eye_width <= sh_eye_width;
					h_pat_blocks <= sh_pat_blocks;
					h_pat_errors <= sh_pat_errors;
					go_capture <= 1'b1;
					window <= 1'b0;
				end
				else if (cnt0 == TIMEOUT-1) begin
					h_t <= t;
					h_lock <= lock_sync;
					h_snap_ok <= 1'b0;
					go_capture <= 1'b1;
					window <= 1'b0;
				end
				else
					cnt0 <= cnt0 + 1;
			end
		end
	
	
	// go register
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			go <= 1'b0;
		else
			go <= go_capture;


endmodule











module pat_monitor (
	input  wire rst_n,
	input  wire	wclk,
	input  wire	valid,
	input  wire	[7:0] din,
	input  wire	[2:0] pos,
	output reg	[31:0] pat_blocks,
	output reg	[31:0] pat_errors
);
	
	// parameters
	localparam FRAME_B0 = 8'hA5;
	localparam FRAME_B1 = 8'h5A;
	localparam FRAME_B2 = 8'hA5;
	localparam FRAME_B3 = 8'h5A;
	localparam FRAME_B6 = 8'h01;
	localparam FRAME_B7 = 8'h80;
	
	// reg & wire
	reg [2:0] pos_reg;
	reg [15:0] c_reg, cnt_e;
	wire [15:0] c;
	reg inblk, byte_err, c_err, have_e;
	wire blk_err, cnt_bad;
	reg [7:0] c_hi, c_lo;
	reg rst_w2, rst_w1;
	
	
	// reset pulse synchronizer - async rst_n
	always@(negedge rst_n or posedge wclk)
		if (!rst_n) begin
			rst_w1 <= 1'b0;
			rst_w2 <= 1'b0;
		end
		else begin
			rst_w1 <= 1'b1;
			rst_w2 <= rst_w1;
		end
	
	
	always@(negedge rst_w2 or posedge wclk)
		if (!rst_w2) begin
			pat_blocks <= 32'd0;
			pat_errors <= 32'd0;
			inblk <= 1'b0;
			byte_err <= 1'b0;
			c_lo <= 8'd0;
			c_hi <= 8'd0;
			c_err <= 1'b0;
			have_e <= 1'b0;
			pos_reg <= 3'd0;
			c_reg <= 16'd0;
			cnt_e <= 16'd0;
		end
		else if (valid) begin
			pos_reg <= pos;
			if (pos == 3'd7)
				c_reg <= c;
				
			if (pos == 3'd0) begin
				inblk <= 1'b1;
				byte_err <= (FRAME_B0 != din);
			end
			else if (pos == 3'd1) begin
				byte_err <= byte_err | (FRAME_B1 != din);
				if (pos-pos_reg != 3'd1) inblk <= 0;
			end
			else if (pos == 3'd2) begin
				byte_err <= byte_err | (FRAME_B2 != din);
				if (pos-pos_reg != 3'd1) inblk <= 0;
			end
			else if (pos == 3'd3) begin
				byte_err <= byte_err | (FRAME_B3 != din);
				if (pos-pos_reg != 3'd1) inblk <= 0;
			end
			else if (pos == 3'd4) begin
				c_lo <= din;
				if (pos-pos_reg != 3'd1) inblk <= 0;
			end
			else if (pos == 3'd5) begin
				c_hi <= din;
				if (pos-pos_reg != 3'd1) inblk <= 0;
			end
			else if (pos == 3'd6) begin
				byte_err <= byte_err | (FRAME_B6 != din);
				if (pos-pos_reg != 3'd1) inblk <= 0;
			end
			else if ( (pos == 3'd7) && inblk ) begin
				
				if (pat_blocks != 32'hffffffff) begin
					pat_blocks <= pat_blocks + 1;
					if ( blk_err || cnt_bad )
						pat_errors <= pat_errors + 1;
				end
				
				if (!have_e) begin
					cnt_e <= c;
					have_e <= 1'b1;
				end
				else begin
					if ( cnt_bad && !blk_err && c_err && (c == c_reg + 16'd1) ) begin
						cnt_e <= c;
						c_err <= 1'b0;
					end
					else begin
						cnt_e <= cnt_e + 16'd1;
						c_err <= cnt_bad && !blk_err;
					end
				end
				
			end
		end
		else begin
			inblk <= 1'b0;
			pos_reg <= 3'd0;
			have_e <= 1'b0;
			c_err <= 1'b0;
		end
	
	assign c = {c_hi,c_lo};
	assign blk_err = (byte_err | (FRAME_B7 != din));
	assign cnt_bad = have_e && (c != cnt_e + 16'd1);
	
	
endmodule
















module console_fmt #(
	parameter MODE_PATTERN = 1'b1
)(
	input  wire	clk,
	input  wire	rst_n,
	input  wire	go,
	input  wire	[31:0]	t,
	input  wire	lock,
	input  wire	snap_ok,
	input  wire	cdr_rdy,
	input  wire	cdr_valid,
	input  wire	[8:0]	phase,
	input  wire	[8:0]	eye_center,
	input  wire	[8:0]	eye_width,
	input  wire	[31:0]	pat_blocks,
	input  wire	[31:0]	pat_errors,
	input  wire	uart_rdy,
	output reg	uart_start,
	output reg	[7:0]	uart_data,
	output reg	busy
);
	
	localparam IDLE = 3'd0;
	localparam GET_CHAR = 3'd1;
	localparam TRANSMIT = 3'd2;
	localparam DECODE_BIN = 3'd3;
	localparam TRANSMIT_BIN = 3'd4;
	localparam NEXT_CHAR = 3'd5;
	localparam SEND_DIG = 3'd6;
	
	reg [2:0] current_state, next_state;
	
	localparam LINE_CHAR = 8'd128;
	
	localparam [LINE_CHAR*8-1:0] LINE = {"CMP t=", 8'h80, " link=- lock=", 8'h81, " ph=", 8'h82,
          "/336 eye=", 8'h83, "/", 8'h84, " mode=", 8'h89, " rx=", 8'h85, " crc=", 8'h86,
          " sent=- rep=- skip=- gap=- down=- hw=- mm=- design=- flags=- snap=", 8'h87,
          " *", 8'h88, 8'h0D, 8'h0A};
	
	
	reg uart_data_en, cnt0_en, cnt0_rst, cnt1_rst, cnt1_en, rst_sync;
	reg [7:0] cnt0;
	reg [3:0] cnt1;
	
	
	// Line-Char counter
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			cnt0 <= 8'd0;
		else if (cnt0_rst == 1'b1)
			cnt0 <= 8'd0;
		else if (cnt0_en == 1'b1)
			cnt0 <= cnt0 + 8'd1;
	
	wire bin2dec_rdy;
	wire [3:0] bin2dec_len;
	reg bin2dec_start;
	
	reg [31:0] uart_data_bin;
	wire [39:0] uart_data_dec;
	reg [55:0] uart_data_text;
	reg [3:0] uart_data_text_len;
	reg [7:0] uart_data_mux, uart_data_char, chksum;
	wire [15:0] chksum_hexc;
	reg [1:0] uart_data_sel;
	wire [8:0] eye_center_val, eye_width_val;
	
	
	// dec-length counter
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			cnt1 <= 4'd15;
		else if (cnt1_rst == 1'b1)
			if (uart_data_sel == 2'd2)
				cnt1 <= bin2dec_len-1;
			else
				cnt1 <= uart_data_text_len-1;
		else if (cnt1_en == 1'b1)
			cnt1 <= cnt1 - 4'd1;
	
	
	bin2dec dec0 (
		.rst_n	(rst_n),
		.clk	(clk),
		.din	(uart_data_bin),	// input in binary
		.start	(bin2dec_start),
		.dout	(uart_data_dec),		// output in decimal
		.len	(bin2dec_len),		// output: length of decimal
		.rdy	(bin2dec_rdy)
	);
	
	
	
	assign eye_center_val =	(cdr_valid)	?	eye_center : 9'd0;
	assign eye_width_val =	(cdr_valid)	?	eye_width : 9'd0;
	
	
	always@(negedge rst_n or posedge clk)
		if (!rst_n) begin
			uart_data_sel <= 2'd0;
			uart_data_bin <= 32'd0;
			uart_data_text <= 40'd0;
			uart_data_text_len <= 4'd0;
		end
		else if (rst_sync) begin
			uart_data_sel <= 2'd0;
			uart_data_bin <= 32'd0;
			uart_data_text <= 40'd0;
			uart_data_text_len <= 4'd0;
		end
		else if (uart_data_en) begin
			if 		( LINE[LINE_CHAR*8-1 - (cnt0 * 8) -: 8] == "\200" )	begin
				uart_data_bin <= t;
				uart_data_sel <= 2'd2;
			end
			else if	( LINE[LINE_CHAR*8-1 - (cnt0 * 8) -: 8] == "\201" )	begin
				uart_data_bin <= {31'd0, lock};
				uart_data_sel <= 2'd2;
			end
			else if	( LINE[LINE_CHAR*8-1 - (cnt0 * 8) -: 8] == "\202" )	begin
				uart_data_bin <= {23'd0, phase};
				uart_data_sel <= 2'd2;
			end
			else if	( LINE[LINE_CHAR*8-1 - (cnt0 * 8) -: 8] == "\203" )	begin
				if (!cdr_rdy) begin
					uart_data_text <= "---";
					uart_data_text_len <= 4'd3;
					uart_data_sel <= 2'd3;
				end
				else begin
					uart_data_bin <= {23'd0, eye_center_val};
					uart_data_sel <= 2'd2;
				end
			end
			else if	( LINE[LINE_CHAR*8-1 - (cnt0 * 8) -: 8] == "\204" )	begin
				if (!cdr_rdy) begin
					uart_data_text <= "---";
					uart_data_text_len <= 4'd3;
					uart_data_sel <= 2'd3;
				end
				else begin
					uart_data_bin <= {23'd0, eye_width_val};
					uart_data_sel <= 2'd2;
				end
			end
			else if	( LINE[LINE_CHAR*8-1 - (cnt0 * 8) -: 8] == "\205" )	begin
				uart_data_bin <= pat_blocks;
				uart_data_sel <= 2'd2;
			end
			else if	( LINE[LINE_CHAR*8-1 - (cnt0 * 8) -: 8] == "\206" )	begin
				uart_data_bin <= pat_errors;
				uart_data_sel <= 2'd2;
			end
			else if	( LINE[LINE_CHAR*8-1 - (cnt0 * 8) -: 8] == "\207" )	begin
				if (snap_ok) begin
					uart_data_text <= "ok";
					uart_data_text_len <= 4'd2;
				end
				else begin
					uart_data_text <= "stale";
					uart_data_text_len <= 4'd5;
				end
				uart_data_sel <= 2'd3;
			end
			else if	( LINE[LINE_CHAR*8-1 - (cnt0 * 8) -: 8] == 8'h89 )	begin
				if (MODE_PATTERN) begin
					uart_data_text <= "pattern";
					uart_data_text_len <= 4'd7;
				end
				else begin
					uart_data_text <= "frames";
					uart_data_text_len <= 4'd6;
				end
				uart_data_sel <= 2'd3;
			end
			else if	( LINE[LINE_CHAR*8-1 - (cnt0 * 8) -: 8] == "\210" )	begin
				uart_data_text <= { 40'd0, chksum_hexc };
				uart_data_text_len <= 4'd2;
				uart_data_sel <= 2'd3;
			end
			else
				uart_data_sel <= 2'd1;
		end
	
	always@(*) begin
		case (uart_data_sel)
			2'd0: uart_data = 8'd0;
			2'd1: uart_data = LINE[LINE_CHAR*8-1 - (cnt0 * 8) -: 8];
			2'd2: uart_data = uart_data_mux;
			2'd3: uart_data = uart_data_char;
			default: uart_data = 8'd0;
		endcase
	end

	// checksum
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			chksum <= 8'd0;
		else if (rst_sync)
			chksum <= 8'd0;
		else if ( (uart_start) && (cnt0 < LINE_CHAR-4) )
			chksum <= chksum ^ uart_data;
	// hexc: char-to-hex decoder
	assign chksum_hexc[15:8] = (chksum[7:4] < 4'd10) ? 8'h30 + chksum[7:4] : 8'h37 + chksum[7:4];
	assign chksum_hexc[7:0] = (chksum[3:0] < 4'd10) ? 8'h30 + chksum[3:0] : 8'h37 + chksum[3:0];

	
	always@(*) begin
		case (cnt1)
			4'd0: uart_data_mux = 8'h30 + { 4'd0, uart_data_dec[3:0] };
			4'd1: uart_data_mux = 8'h30 + { 4'd0, uart_data_dec[7:4] };
			4'd2: uart_data_mux = 8'h30 + { 4'd0, uart_data_dec[11:8] };
			4'd3: uart_data_mux = 8'h30 + { 4'd0, uart_data_dec[15:12] };
			4'd4: uart_data_mux = 8'h30 + { 4'd0, uart_data_dec[19:16] };
			4'd5: uart_data_mux = 8'h30 + { 4'd0, uart_data_dec[23:20] };
			4'd6: uart_data_mux = 8'h30 + { 4'd0, uart_data_dec[27:24] };
			4'd7: uart_data_mux = 8'h30 + { 4'd0, uart_data_dec[31:28] };
			4'd8: uart_data_mux = 8'h30 + { 4'd0, uart_data_dec[35:32] };
			4'd9: uart_data_mux = 8'h30 + { 4'd0, uart_data_dec[39:36] };
			default: uart_data_mux = 8'd0;
		endcase
	end	
	
	
	always@(*) begin
		case (cnt1)
			4'd0: uart_data_char = uart_data_text[7:0];
			4'd1: uart_data_char = uart_data_text[15:8];
			4'd2: uart_data_char = uart_data_text[23:16];
			4'd3: uart_data_char = uart_data_text[31:24];
			4'd4: uart_data_char = uart_data_text[39:32];
			4'd5: uart_data_char = uart_data_text[47:40];
			4'd6: uart_data_char = uart_data_text[55:48];
			default: uart_data_char = 8'd0;
		endcase
	end	
	
	
	
	
	// FSM
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			current_state <= IDLE;
		else
			current_state <= next_state;
	
	always@(*) begin
		// default
		uart_start = 1'b0;
		busy = 1'b0;
		uart_data_en = 1'b0;
		cnt0_rst = 1'b0;
		cnt0_en = 1'b0;
		bin2dec_start = 1'b0;
		cnt1_rst = 1'b0;
		cnt1_en = 1'b0;
		rst_sync = 1'b0;
		
		case (current_state)
			IDLE: begin
				if (go) begin
					next_state = GET_CHAR;
					uart_data_en = 1'b1;
				end
				else begin
					next_state = IDLE;
					rst_sync = 1'b1;
				end
				cnt0_rst = 1'b1;
				cnt1_rst = 1'b1;
			end
			
			GET_CHAR: begin
				if ( LINE[LINE_CHAR*8-1 - (cnt0 * 8) -: 8] >= "\200" ) begin
					if (uart_data_sel==2'd3) begin
						next_state = SEND_DIG;
						cnt1_rst = 1'b1; // load cnt1
					end
					else begin
						next_state = DECODE_BIN;
						bin2dec_start = 1'b1;
					end
				end
				else begin
					if ( (cnt0 == LINE_CHAR-1) && uart_rdy ) begin
						next_state = IDLE;
						uart_start = 1'b1;
					end
					else if (uart_rdy) begin
						next_state = TRANSMIT;
						uart_start = 1'b1;
					end
					else
						next_state = GET_CHAR;
				end
				busy = 1'b1;
			end
			
			DECODE_BIN: begin
				if (bin2dec_rdy) begin
					next_state = SEND_DIG;
					cnt1_rst = 1'b1; // len is valid while bin2dec_rdy = 1'b1
				end
				else
					next_state = DECODE_BIN;
				busy = 1'b1;
			end
			
			SEND_DIG: begin
				if (uart_rdy) begin
					next_state = TRANSMIT_BIN;
					uart_start = 1'b1;
				end
				else
					next_state = SEND_DIG;
				busy = 1'b1;
			end
			TRANSMIT_BIN: begin
				if ( (cnt1 == 4'd0) && (uart_rdy) ) begin
					next_state = NEXT_CHAR;
					cnt0_en = 1'b1;
				end
				else if (uart_rdy) begin
					next_state = SEND_DIG;
					cnt1_en = 1'b1;
				end
				else
					next_state = TRANSMIT_BIN;
				busy = 1'b1;
			end


			TRANSMIT: begin
				if (uart_rdy) begin
					next_state = NEXT_CHAR;
					cnt0_en = 1'b1;
				end
				else
					next_state = TRANSMIT;
				busy = 1'b1;
			end
			
			NEXT_CHAR: begin
				next_state = GET_CHAR;
				uart_data_en = 1'b1;
				busy = 1'b1;
			end
			
			default: begin
				next_state = IDLE;
			end
		endcase
	end
	
	
	
	
endmodule




module bin2dec (
	input  wire	rst_n,
	input  wire	clk,
	input  wire	[31:0]	din,	// input in binary
	input  wire	start,
	output reg	[39:0]	dout,		// output in decimal
	output reg	[3:0]	len,		// output: length of decimal
	output wire	rdy
);

	localparam IDLE = 2'd0;
	localparam DECODE = 2'd1;
	
	reg [1:0] current_state, next_state;
	
	localparam NBITS = 8'd32;
	
	reg [31:0] din_reg;
	reg [7:0] cnt0;
	reg din_load, din_en, cnt0_en, rst_sync, dout_en;
	
	// input shift register
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			din_reg <= 32'b0;
		else
			if (din_load)
				din_reg <= din;
			else if (din_en)
				din_reg <= { din_reg[30:0], 1'b0 };
	
	reg [3:0] dig0, dig1, dig2, dig3, dig4, dig5, dig6, dig7, dig8, dig9;
	
	// BCD digit check
	always@(*) begin
		if ( dout[3:0] >= 4'd5 ) dig0 = dout[3:0] + 4'd3; else dig0 = dout[3:0];
		if ( dout[7:4] >= 4'd5 ) dig1 = dout[7:4] + 4'd3; else dig1 = dout[7:4];
		if ( dout[11:8] >= 4'd5 ) dig2 = dout[11:8] + 4'd3; else dig2 = dout[11:8];
		if ( dout[15:12] >= 4'd5 ) dig3 = dout[15:12] + 4'd3; else dig3 = dout[15:12];
		if ( dout[19:16] >= 4'd5 ) dig4 = dout[19:16] + 4'd3; else dig4 = dout[19:16];
		if ( dout[23:20] >= 4'd5 ) dig5 = dout[23:20] + 4'd3; else dig5 = dout[23:20];
		if ( dout[27:24] >= 4'd5 ) dig6 = dout[27:24] + 4'd3; else dig6 = dout[27:24];
		if ( dout[31:28] >= 4'd5 ) dig7 = dout[31:28] + 4'd3; else dig7 = dout[31:28];
		if ( dout[35:32] >= 4'd5 ) dig8 = dout[35:32] + 4'd3; else dig8 = dout[35:32];
		if ( dout[39:36] >= 4'd5 ) dig9 = dout[39:36] + 4'd3; else dig9 = dout[39:36];
	end
	
	// output shift register
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			dout <= 40'd0;
		else
			if (rst_sync)
				dout <= 40'd0;
			else if (dout_en)
				dout <= { dig9[2:0], dig8, dig7, dig6, dig5, dig4, dig3, dig2, dig1, dig0, din_reg[31] };
	
	// iteration counter = #bits of din
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			cnt0 <= 8'd0;
		else if (rst_sync == 1'b1)
			cnt0 <= 8'd0;
		else if (cnt0 == NBITS-1)
			cnt0 <= 8'd0;
		else if (cnt0_en)
			cnt0 <= cnt0 + 8'd1;
	
	
	// len register
	always@(*) begin
		len = 4'd1;
		if 		(dout[39:36] != 4'd0)	len = 4'd10;
		else if (dout[35:32] != 4'd0)	len = 4'd9;
		else if (dout[31:28] != 4'd0)	len = 4'd8;
		else if (dout[27:24] != 4'd0)	len = 4'd7;
		else if (dout[23:20] != 4'd0)	len = 4'd6;
		else if (dout[19:16] != 4'd0)	len = 4'd5;
		else if (dout[15:12] != 4'd0)	len = 4'd4;
		else if (dout[11:8] != 4'd0)		len = 4'd3;
		else if (dout[7:4] != 4'd0)		len = 4'd2;
	end
	
	
	// rdy logic
	assign rdy = (current_state == IDLE);
	
	// FSM
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			current_state <= IDLE;
		else
			current_state <= next_state;
	
	always@(*) begin
		// default
		cnt0_en = 1'b0;
		din_load = 1'b0;
		din_en = 1'b0;
		rst_sync = 1'b0;
		dout_en = 1'b0;
		
		case (current_state)
			IDLE: begin
				if (start) begin
					next_state = DECODE;
					din_load = 1'b1;
					rst_sync = 1'b1;
				end
				else
					next_state = IDLE;
			end
			
			DECODE: begin
				if ( (cnt0 == NBITS-1) ) begin
					next_state = IDLE;
				end
				else begin
					next_state = DECODE;
					cnt0_en = 1'b1;
				end	
				din_en = 1'b1;
				dout_en = 1'b1;
			end
			
			default: begin
				next_state = IDLE;
			end
		endcase
	end


endmodule









module console_uart(
	input  wire	clk,				// clkin (12 MHz)
	input  wire	rst_n,
	input  wire	start,
	input  wire	[7:0]	data_tx,
	output reg	con_tx,
	output wire	rdy
);

	// UART: 115,200 8N1. LSB First, Start Bit 0, Stop bit 1
	
	localparam IDLE = 2'd0;
	localparam TRANSMIT = 2'd1;
	
	reg [1:0] current_state, next_state;
	
	localparam BIT_TIMING_MAX = 8'd104;
	localparam BIT_MAX = 4'd10;
	reg d_tx_load, d_tx_en, cnt1_en, rst_sync;
	reg [3:0] cnt1;
	wire [3:0] con_tx_sel;
	reg [7:0] cnt0, d_tx;
	
	// shift register
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			d_tx <= 8'b0;
		else
			if (d_tx_load)
				d_tx <= data_tx;
			else if (d_tx_en)
				d_tx <= { 1'b1, d_tx[7:1] };
		
	// output
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			con_tx <= 1'b1;
		else if (current_state == TRANSMIT) begin
			if (con_tx_sel == 4'd0)
				con_tx <= 1'b0;
			else if ( (con_tx_sel > 4'd0) && (con_tx_sel < 4'd9) )
				con_tx <= d_tx[0];
			else if (con_tx_sel == 4'd9)
				con_tx <= 1'b1;
		end
		else
			con_tx <= 1'b1;
	
	assign con_tx_sel = cnt1;
		
	// bit counter
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			cnt1 <= 4'd0;
		else if (rst_sync == 1'b1)
			cnt1 <= 4'd0;
		else if (cnt1_en == 1'b1)
			if (cnt1 == BIT_MAX-1)
				cnt1 <= 4'd0;
			else
				cnt1 <= cnt1 + 4'd1;
	
	// bit-timing counter
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			cnt0 <= 8'd0;
		else if (rst_sync == 1'b1)
			cnt0 <= 8'd0;
		else if (cnt0 == BIT_TIMING_MAX-1)
			cnt0 <= 8'd0;
		else
			cnt0 <= cnt0 + 8'd1;
	
	
	// rdy logic
	assign rdy = (current_state == IDLE);
	
				
	// FSM
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			current_state <= IDLE;
		else
			current_state <= next_state;
	
	always@(*) begin
		// default
		cnt1_en = 1'b0;
		d_tx_load = 1'b0;
		d_tx_en = 1'b0;
		rst_sync = 1'b0;
		
		case (current_state)
			IDLE: begin
				if (start) begin
					next_state = TRANSMIT;
					d_tx_load = 1'b1;
					
				end
				else
					next_state = IDLE;
				rst_sync = 1'b1;
			end
			
			TRANSMIT: begin
				if ( (cnt1 == BIT_MAX-1) && (cnt0 == BIT_TIMING_MAX-1) ) begin
					next_state = IDLE;
				end
				else if (cnt0 == BIT_TIMING_MAX-1) begin
					next_state = TRANSMIT;
					cnt1_en = 1'b1;
				end
				else
					next_state = TRANSMIT;
				
				if ( (con_tx_sel > 4'd0) && (con_tx_sel < 4'd9) && (cnt0 == BIT_TIMING_MAX-1) )
					d_tx_en = 1'b1;
			end
			
			default: begin
				next_state = IDLE;
			end
		endcase
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
	output reg	meas_en,
	// to pat_monitor
	output reg	[8:0] phase,
	output wire	[7:0] dout,
	output wire	[2:0] pos
);
	
	// parameters
	localparam STEPS = 9'd336;
	localparam PHASE_MAX = STEPS-1;

	
	// wire & reg
	wire phase_en;
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
	
	
	assign dout = aligner_dout;
	assign pos = aligner_pos[2:0]; // specific for FRAME_LENGTH = 7, see frame_aligner
	
	
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
			if (center >= STEPS)
				eye_center <= center - STEPS;
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
