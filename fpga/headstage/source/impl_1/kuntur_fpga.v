/***********************************************************
	Author: Manuel Monge
	Description:
		Kuntur FPGA top level. RHD2164 x2 -> ch_sel -> FIFO -> SPI0 ->
		MCU, plus the MCU-facing regbank/register-console control plane.
		Reorganized into common/afe/app/tb, 2026-08-26 (PLAN.md B.3) —
		see those directories' module headers for the layering rationale.
***********************************************************/

// Applies for the rest of this compilation unit (every `included file
// below) — moved here, 2026-08-26, from its previous home at the top of
// the old components.v, so it now also covers this top-level module's
// own port/wire declarations, which it previously didn't.

//`default_nettype none

`include "common/edge_detector.v"
`include "common/shift_registers.v"
`include "common/spi_slave.v"
`include "common/spi_master.v"
`include "common/fifo.v"
`include "common/gddr_sync.v"

`include "afe/rhd2164/spi_master_rhd2164x2.v"
`include "afe/rhd2164/rhd2164_controller.v"

`include "app/regbank.v"
`include "app/ch_sel.v"
`include "app/main_controller.v"
`include "app/dtx_mux_reg.v"
`include "app/test_pattern_gen.v"
`include "app/fifo_din_mux.v"

`include "kuntur_build.vh"

module kuntur_fpga (
	input  wire	rstb,
	input  wire	clkin,
	// SPI to MCU
	input  wire	spi0_csb,	// F8 (PB20B)
	input  wire	spi0_sck,	// B2 (PT65A)
	input  wire	spi0_mosi,	// B3 (PT63B)
	output wire	spi0_miso,	// E2 (PR9B)
	// SPI to RHD2164
	input  wire	spi1_miso1,	// G4 (LVDS0P)
	input  wire	spi1_miso0,	// G3 (LVDS3P)
	output wire	spi1_csb,	// H1 (LVDS5P)
	output wire	spi1_sck,	// H2 (LVDS4P)
	output wire	spi1_mosi,	// E4 (LVDS2P)
	// SPI to RHS2116
	input  wire	spi2_miso0,	// E7 (LVDS10P)
	output wire	spi2_csb,	// G5 (LVDS11P)
	output wire	spi2_sck,	// F7 (LVDS9P)
	output wire	spi2_mosi0,	// E5 (LVDS1P)
	// LVDS through uHDMI
	output wire tun_data, // G9 (LVDS7P, FPGA_LVDS1P)
	output wire tun_clk, // E9 (LVDS6P, FPGA_LVDS2P)
	// TEMP signals
	output wire	cmd_is_00
);
	
	// parameters
	parameter n = 16;
	parameter data_tx = 16'hbaba;
	parameter spi1_data_tx = 16'h8787;
	parameter cmd_00 = 16'h0004;
	
	
	// wire & regs
	wire [n-1:0] data_rx;
	wire spi0_done;
	
	wire rhd_spi1_start;
	wire [n-1:0] spi1_data_rx_a0,spi1_data_rx_b0;
	wire [n-1:0] spi1_data_rx_a1,spi1_data_rx_b1;
	wire rhd_spi1_done;
	wire rx_flag;
	wire [15:0] rhd_dtx,rhd_dtx0,rhd_dtx1;
	wire rhd_dtx_sel;
	wire [5:0] rhd_addr_cfg,rhd_addr_sampling, ch_cnt;
	
	wire [15:0] spi0_drx, spi0_dtx;
	wire mcu_dtx_en;
	wire [1:0] mcu_dtx_sel;
	
	// regs
	reg clkdiv2;
	wire clk,clk_tx;
	
	// FIFO
	wire fifo_wen, fifo_ren, fifo_empty, fifo_full;
	wire [31:0] fifo_din, fifo_dout;
	
	
	wire [7:0] ch_a, ch_b;
	wire [7:0] mode1_ch_a, mode1_ch_b, mode1_ch_c, mode1_ch_d;
	wire [7:0] mode2_ch_a, mode2_ch_b, mode2_ch_c, mode2_ch_d, mode2_ch_e, mode2_ch_f, mode2_ch_g, mode2_ch_h;
	wire [7:0] mode3_ch_a, mode3_ch_b, mode3_ch_c, mode3_ch_d, mode3_ch_e, mode3_ch_f, mode3_ch_g, mode3_ch_h;
	wire [7:0] mode3_ch_i, mode3_ch_j, mode3_ch_k, mode3_ch_l, mode3_ch_m, mode3_ch_n, mode3_ch_o, mode3_ch_p;
	wire regbank_wen;
	wire [7:0] regbank_addr0, regbank_addr1;
	wire [15:0] regbank_din0, regbank_dout0, regbank_dout1;
	wire [1:0] opcode;
	
	wire trig,dbg0,trig1,dbg1;
	wire stream_enable;
	wire regbank_port_en;
	wire [1:0] regbank_port_sel;
	wire [5:0] regbank_port_addr;
	
	reg [15:0] cnt0;
	wire [15:0] ch0, ch1;
	wire [31:0] dout_test,ch_sel_dout;
	wire dout_en_0;
	wire [1:0] data_source_sel;
	
	wire eclk,wclk,tun_rst;
	wire [7:0] block_gen_dout,tun_data_8b, fb_dout;
	
	wire pll_lock, cfg_done, xfer_done;
	
	// spi1_eye_scan
	wire phase_step_wr;
	wire [15:0] phase_step_cmd;
	wire [15:0] mosi_scan_ctrl;
	wire chk_clear, a_next, spi1_mosi_m0;
	wire [11:0] scan0_spt;

	
	wire gddr_rdy, gddr_ddr_rst, gddr_stop;
	wire rst_n_clkin, lock_sync, gddr_rst, rst_n_clk, blk_rstb;
	
	
	wire	phasestep, phasedir;
	wire	[8:0]	phase_pos;
	wire	[15:0]	phase_info;
	wire	[15:0]	chk_count;
	wire	[15:0]	err_c0a;
	wire	[15:0]	err_c0b;
	wire	[15:0]	err_c1a;
	wire	[15:0]	err_c1b;
	wire	[15:0]	first_err_ans;
	wire	[15:0] first_err_info;
	wire	[15:0] first_err_exp;

	
	
	// TEST ONLY - BEGIN
	assign cmd_is_00 = fifo_full;
	// TEST ONLY - END
	
	
	// RHS2116 is currently disabled.
	assign spi2_csb = 1'b1;
	assign spi2_sck = 1'b0;
	assign spi2_mosi0 = 1'b0;
	
	
	
	clock_gen #(
		.VARIANT	(`KUNTUR_VARIANT)
	) clock_gen_0 (
		.clkin	(clkin),
		.clkop	(clk),
		.clkos	(clk_tx),
		.pll_lock	(pll_lock),
		// phase control
		.phasestep	(phasestep),
		.phasedir	(phasedir),
		.spt	(scan0_spt)
	);

	
	spi1_eye_scan scan0 (
		// inputs
		.rst_n	(rst_n_clk),
		.clk	(clk),
		.clk_tx	(clk_tx),
		.spi1_mosi_in	(spi1_mosi_m0),
		.a_next	(a_next),
		.drx_a0	(spi1_data_rx_a0),
		.drx_b0	(spi1_data_rx_b0),
		.drx_a1	(spi1_data_rx_a1),
		.drx_b1	(spi1_data_rx_b1),
		.rhd_start	(rhd_spi1_start),
		.data_tx	(regbank_dout1),
		.xfer_done	(xfer_done),
		.mosi_scan_ctrl	(mosi_scan_ctrl),
		.phase_step_cmd	(phase_step_cmd),
		.phase_step_wr	(phase_step_wr),
		.chk_clear	(chk_clear),
		.stepper_spt	(scan0_spt),
		// outputs
		.spi1_mosi_out	(spi1_mosi),
		.phasestep	(phasestep),
		.phasedir	(phasedir),
		.phase_pos	(phase_pos),
		.phase_info	(phase_info),
		.chk_count	(chk_count),
		.err_c0a	(err_c0a),
		.err_c0b	(err_c0b),
		.err_c1a	(err_c1a),
		.err_c1b	(err_c1b),
		.first_err_ans	(first_err_ans),
		.first_err_info	(first_err_info),
		.first_err_exp	(first_err_exp)
	);
	
	//`ifdef SIM
		//assign clk = clkin;
	//`else	
		//// Internal PLL
		//pll0 mypll (
			//.clki_i		(clkin),
			//.clkop_o		(clk),
			//.clkos_o		(),
			//.lock_o		(pll_lock)
		//);
	//`endif
	
	
	// Reset generation 
	
	reset_gen_kuntur rst_gen_0 (
		// inputs
		.clkin	(clkin),
		.rstb	(rstb),
		.pll_lock	(pll_lock),
		.clk	(clk),
		.wclk	(wclk),
		.gddr_rdy	(gddr_rdy),
		// outputs
		.lock_sync	(lock_sync),
		.rst_n_clkin	(rst_n_clkin),
		.gddr_rst	(gddr_rst),
		.rst_n_clk	(rst_n_clk),
		.blk_rstb	(blk_rstb)
	);
		
	gddr_sync u_gddr_sync (
		.rst       (gddr_rst),    // async, active high      <- reset_gen_adapter
		.sync_clk  (clkin),       // free-running 32 MHz
		.start     (lock_sync),   // begin when the PLL is locked
		.stop      (gddr_stop),   // -> ECLKSYNC.STOP
		.ddr_reset (gddr_ddr_rst),// -> ECLKDIV.DIVRST, ODDRX4.RST
		.ready     (gddr_rdy)     // -> reset_gen_kuntur.gddr_rdy
	);
	
	
	
	
	// SPI Data Format
	// spi0_drx = {OPCODE[1:0], ADDR[5:0], DATA[7:0]}
	

	// SPI-Master to 2x RHD2164 chips
	spi_master_rhd2164x2 spi1_rhd2164x2 (
		.rstb		(rst_n_clk),				// Active-low asyncrhonous reset
		.clk		(clk),				// Input clock
		.clk90		(clk_tx),				// Input clock
		.start		(rhd_spi1_start),	// Active-high signal that starts SPI cycle
		.csb		(spi1_csb),			// Active-low chip-select
		.sck_reg		(spi1_sck),			// SPI Clock
		.mosi_reg		(spi1_mosi_m0),		// Master-Output Slave-Input
		.miso		(spi1_miso0),		// Master-Input Slave-Output
		.miso1		(spi1_miso1),		// Master-Input Slave-Output 1
		.data_tx	(regbank_dout1),		// Data to be transmitted
		.data_rx_a	(spi1_data_rx_a0),	// DataA received
		.data_rx_b	(spi1_data_rx_b0),	// DataB received
		.data_rx_a1	(spi1_data_rx_a1),	// DataA1 received
		.data_rx_b1	(spi1_data_rx_b1),	// DataB1 received
		.done		(rhd_spi1_done),		// TX/RX operation completed; no communicacion happening
		.a_next	(a_next)
	);


	// FSM
	
	rhd2164_controller controller0 (
		.rstb				(rst_n_clk),				// Active-low asyncrhonous reset
		.clk				(clk),				// Input clock
		.rhd_start			(rhd_spi1_start),	// Active-high output that starts RHD2164x2 SPI cycle
		.rhd_done			(rhd_spi1_done),		// Active-high input that indicates SPI cycle has finished
		.rb_addr1			(regbank_addr1),
		.ch_cnt				(ch_cnt),
		.cfg_done			(cfg_done),		// Active-high; config table played, sampling cycle running
		.xfer_done			(xfer_done)
	);


	
	// RegBank
	// DATA_WIDTH/ADDR_WIDTH are localparams inside regbank now
	// (2026-08-26) — no longer overridable, so no #(...) here.
	regbank regbank0 (
		.rstb		(rstb),			// Active-low asyncrhonous reset
		.clk		(clk),			// Input clock
		.wen		(regbank_wen),	// Input; RegBank write enable
		.addr0		(regbank_addr0),	// Input Address, Port0
		.din0		(regbank_din0),	// Input Data, Port0
		.dout0		(regbank_dout0),	// Output Data, Port0
		.addr1		(regbank_addr1),	// Input Address, Port0
		.dout1		(regbank_dout1),	// Output Data, Port0
		.regbank_port_en		(regbank_port_en),
		.regbank_port_sel	(regbank_port_sel),

		// Config connections
		.ch_a		(ch_a),
		.ch_b		(ch_b),
		.stream_enable	(stream_enable),
		.data_source_sel	(data_source_sel),
		// spi1_eye_scan - outputs
		.phase_step_wr	(phase_step_wr),
		.phase_step_cmd	(phase_step_cmd),
		.mosi_scan_ctrl	(mosi_scan_ctrl),
		.chk_clear	(chk_clear),
		// spi1_eye_scan - inputs
		.phase_pos	(phase_pos),
		.phase_info	(phase_info),
		.chk_count	(chk_count),
		.err_c0a	(err_c0a),
		.err_c0b	(err_c0b),
		.err_c1a	(err_c1a),
		.err_c1b	(err_c1b),
		.first_err_ans	(first_err_ans),
		.first_err_info	(first_err_info),
		.first_err_exp	(first_err_exp)
	);
	
	
	
	
	// n is a localparam inside ch_sel now (2026-08-26) — no longer
	// overridable, so no #(...) here.
	ch_sel ch_sel0 (
		.rstb		(rstb),				// Active-low asyncrhonous reset
		.clk		(clk),				// Input clock
		.mode		(2'b00),			// Input; Operation Mode
		.ch_cnt		(ch_cnt),	// Input; channel count
		.data_a0	(spi1_data_rx_a0),	// Input; 16-bit data coming from RHD2164-0 output a
		.data_b0	(spi1_data_rx_b0),	// Input; 16-bit data coming from RHD2164-0 output b
		.data_a1	(spi1_data_rx_a1),	// Input; 16-bit data coming from RHD2164-1 output a
		.data_b1	(spi1_data_rx_b1),	// Input; 16-bit data coming from RHD2164-1 output b
		.dout		(ch_sel_dout),			// Output Data
		// Channel Config
		.ch_a		(ch_a),
		.ch_b		(ch_b),
        .fifo_wen	(fifo_wen),			    // Output; FIFO write enable
		.stream_enable	(stream_enable),
		// debug
		.dout_en_0	(dout_en_0),
		.trig		(trig)
	);


		

	
	
	// Main Controller
	main_controller controller1 (
		.rstb		(rstb),			// Active-low asyncrhonous reset
		.clk		(clk),			// Input clock
		.fifo_empty	(fifo_empty),	// Active-high Input; indicates if FIFO is empty
		.fifo_ren	(fifo_ren),		// Active-high Output; FIFO read-enable
		.mcu_dtx_sel	(mcu_dtx_sel),
		.mcu_dtx_en		(mcu_dtx_en),
		.mcu_done	(spi0_done),	// Active-high Input; indicates SPI communication with MCU is done
		.opcode		(opcode),				// Operating Code: READ (00)
		.regbank_wen	(regbank_wen),	// Active-high Output; SRAM write enable
		.regbank_port_en		(regbank_port_en),
		.regbank_port_sel	(regbank_port_sel),
		.regbank_port_addr	(regbank_port_addr)
	);
	
	
	// FIFO
	fifo #(
		.DATA_WIDTH	(32),
		.ADDR_WIDTH	(12) // Additional length to withstand blackout-margin in the MCU due to ST closed-source BLE_Stack processing
		)
		fifo0 (
		.rstb	(rstb),			// Active-low asyncrhonous reset
		.clk	(clk),			// Input clock
		.wen	(fifo_wen),		// Input; FIFO write enable
		.ren	(fifo_ren),		// Input; FIFO read enable
		.DIN	(fifo_din),		// Input Data
		.DOUT	(fifo_dout),	// Output Data
		.empty	(fifo_empty),	// Output flag indicating empty FIFO
		.full	(fifo_full),		// Output flag indicating full FIFO
		.dbg0	(dbg0),
		.dbg1	(dbg1),
		.trig	(trig1)
	);
	
	
	// dtx_mux_reg
	dtx_mux_reg muxreg0_spi0 (
		.rstb			(rstb),			// Active-low asyncrhonous reset
		.clk			(clk),			// Input clock
		.mcu_dtx_sel	(mcu_dtx_sel),
		.mcu_dtx_en		(mcu_dtx_en),
		.din			(fifo_dout),	// Data input
		.ram_din		(regbank_dout0),
		.dout			(spi0_dtx)	// Data output
	);
	
	

	test_pattern_gen test_pattern_gen0 (
		.rstb		(rstb),
		.clk		(clk),
		.dout_en_0	(dout_en_0),
		.dout_test	(dout_test)
	);

	fifo_din_mux fifo_din_mux0 (
		.data_source_sel	(data_source_sel),
		.ch_sel_dout	(ch_sel_dout),
		.dout_test		(dout_test),
		.fifo_din		(fifo_din)
	);



	// SPI-Slave to MCU
	spi_slave #(.n(16),.m(4)) spi0 (
		.rstb		(rstb),
		.clk		(clk),
		.csb		(spi0_csb),
		.sck		(spi0_sck),
		.mosi		(spi0_mosi),
		.miso		(spi0_miso),
		.data_tx	(spi0_dtx),
		.data_rx	(spi0_drx),
		.done		(spi0_done)
	);
	
	assign regbank_addr0 = {2'b11, spi0_drx[13:8]};
	assign regbank_port_addr = spi0_drx[13:8];
	assign opcode = spi0_drx[15:14];
	assign regbank_din0 = {8'd0, spi0_drx[7:0]};



	// Tunnel signlas using primitives right at the I/O pin (IOLogic)
	//***********************
	//
	//                 ┌────────────┐  eclk (45.5 MHz)
	//   clk ─────────>│  ECLKSYNC  │──────────┬───────────────────────────────┐
	//                 └────────────┘          │                               │
	//                                         ▼                               ▼
	//                                 ┌────────────┐  wclk = eclk/4   ┌────────────┐
	//                                 │  ECLKDIV   │─────────┬───────>│  ODDRX4    │──> tun_data (G9/F9)
	//                                 │  ÷4        │         │        │ D0..D7=byte│
	//                                 └────────────┘         │        └────────────┘
	//                                                        │        ┌────────────┐
	//                                                        └───────>│  ODDRX4    │──> tun_clk (E9/E8)
	//                                         eclk ──────────────────>│ D0..D7=    │
	//                                                                 │ 1,0,1,0,...│
	//                                                                 └────────────┘
	
	
	// Primitive block: Edge-clock synchronizer
	ECLKSYNC #(
		.STOP_EN ("ENABLE")
	) eclksync_0 (
		.ECLKIN  (clk),  // I
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

	// Primitive block: Output, Double-Data Rate 8:1 Serializer
	ODDRX4 oddrx4_0 (
		.D0   (1'b1),  // I
		.D1   (1'b0),  // I
		.D2   (1'b1),  // I
		.D3   (1'b0),  // I
		.D4   (1'b1),  // I
		.D5   (1'b0),  // I
		.D6   (1'b1),  // I
		.D7   (1'b0),  // I
		.SCLK (wclk),  // I
		.RST  (gddr_ddr_rst),  // I
		.ECLK (eclk),  // I
		.Q    (tun_clk)   // O
	);
	
	// Primitive block: Output, Double-Data Rate 8:1 Serializer
	ODDRX4 oddrx4_1 (
		.D0   (tun_data_8b[0]),  // I
		.D1   (tun_data_8b[1]),  // I
		.D2   (tun_data_8b[2]),  // I
		.D3   (tun_data_8b[3]),  // I
		.D4   (tun_data_8b[4]),  // I
		.D5   (tun_data_8b[5]),  // I
		.D6   (tun_data_8b[6]),  // I
		.D7   (tun_data_8b[7]),  // I
		.SCLK (wclk),  // I
		.RST  (gddr_ddr_rst),  // I
		.ECLK (eclk),  // I
		.Q    (tun_data)   // O
	);
	
	
	
	wire [7:0] fb_rd_addr;
	wire [15:0] fb_rd_data;
	wire frame_rdy, rbank;
	wire [31:0] frame_index;
	wire [7:0] flags;
	wire [4:0] cfg_addr;
	wire [7:0] cfg_byte;
	wire send_config, send_status, announce;
	wire [15:0]	alt_cmd;
	wire [63:0]	alt_answer;

	
	capture_buffer capture_buffer_0 (
		// inputs
		.rst_n	(rst_n_clk), // active-low reset, same as the controller's
		.clk	(clk), // system clock clk
		.xfer_done	(xfer_done), // one-cycle pulse: "a sampling transfer just finished" 
		.t	(ch_cnt), // which transfer just finished, 0-32, valid with xfer_done
		.d_a0	(spi1_data_rx_a0), // answer received in that transfer
		.d_b0	(spi1_data_rx_b0), // answer received in that transfer
		.d_a1	(spi1_data_rx_a1), // answer received in that transfer
		.d_b1	(spi1_data_rx_b1), // answer received in that transfer
		.cmd	(regbank_dout1), // command word being sent in that transfer
		.rd_clk	(wclk), // read clock for the frame builder
		.rd_addr	(fb_rd_addr), // which word to read: {bank, slot[4:0], half[1:0]}
		// outputs
		.rd_data	(fb_rd_data), // the word at rd_addr (fixed read latency)
		.frame_rdy	(frame_rdy), // toggles each time a full frame is ready
		.rbank	(rbank), // which bank holds the ready frame
		.frame_index	(frame_index), // number of the ready frame
		.alt_cmd	(alt_cmd), // the command sent in sampling entry 32
		.alt_answer	(alt_answer), // that command's four answers
		.announce	(announce)
	);
	
	
	
	tun_aux tun_aux_0 (
		// inputs
		.rst_n			(rst_n_clk), // active-low reset, same as the controller's
		.clk			(clk), // PLL
		.rhd_start		(rhd_spi1_start), // a transfer is starting (op0b in the config walk, op1b while sampling)
		.cmd			(regbank_dout1), // command word being sent in that transfer, valid at rhd_start
		.announce		(announce), // one-clk pulse: xfer_done && t == 1 && primed,
		.stream_enable	(stream_enable), // flags bit 0
		.data_source_sel	(data_source_sel), // flags bit 1 = (data_source_sel == 2'd1), matching fifo_din_mux
		.cfg_addr		(cfg_addr), // which CONFIG byte to read, 0-21
		// outputs
		.flags_snap		(flags), // {4'b0, 1'b0, dirty, test_pattern, stream_enable}: byte 5 of the SAMPLE and of the aux frame after it
		.send_config	(send_config), // a CONFIG follows this SAMPLE
		.send_status	(send_status), // a STATUS follows this SAMPLE
		.cfg_byte		(cfg_byte) // shadow_snap[cfg_addr]: a combinational mux of the snapshot	
	);

	
	wire [15:0] design_id;
	
	frame_builder frame_builder_0(
		// inputs - SAMPLE
		.rst_n			(blk_rstb),
		.clk			(wclk), // wclk
		.frame_rdy		(frame_rdy),
		.rbank			(rbank),
		.frame_index	(frame_index),
		.rd_data		(fb_rd_data),
		// inputs - CONFIG / STATUS
		.flags			(flags),
		.send_config	(send_config),
		.send_status	(send_status),
		.cfg_byte	(cfg_byte),
		.alt_cmd	(alt_cmd),
		.alt_answer	(alt_answer),
		.design_id	(design_id),
		// outputs
		.dout_reg		(fb_dout),
		.rd_addr		(fb_rd_addr),
		.cfg_addr	(cfg_addr)
	);


	
	// block-gen
	block_gen block_gen_0(
		.rstb	(blk_rstb),
		.clk	(wclk),
		.dout	(block_gen_dout)
	);
	
	
	// frame selector
	
	frame_selector #(
		.VARIANT	(`KUNTUR_VARIANT),
		.RELEASE	(`KUNTUR_RELEASE)
	) frame_sel_0 (
		.fb_dout	(fb_dout),
		.bg_dout	(block_gen_dout),
		.tun_data	(tun_data_8b),
		.design_id	(design_id)
	);

	
	
endmodule





module spi1_eye_scan (
		// inputs
		input  wire	rst_n,
		input  wire	clk,
		input  wire	clk_tx,
		input  wire spi1_mosi_in,
		input  wire	a_next,
		input  wire	[15:0]	drx_a0,
		input  wire	[15:0]	drx_b0,
		input  wire	[15:0]	drx_a1,
		input  wire	[15:0]	drx_b1,
		input  wire	rhd_start,
		input  wire	[15:0]	data_tx,
		input  wire	xfer_done,
		input  wire	[15:0]	mosi_scan_ctrl,
		input  wire	[15:0]	phase_step_cmd,
		input  wire	phase_step_wr,
		input  wire	chk_clear,
		input  wire	[11:0]	stepper_spt,
		// outputs
		output wire	spi1_mosi_out,
		output wire	phasestep,
		output wire	phasedir,
		output wire	[8:0]	phase_pos,
		output wire	[15:0]	phase_info,
		output wire	[15:0]	chk_count,
		output wire	[15:0]	err_c0a,
		output wire	[15:0]	err_c0b,
		output wire	[15:0]	err_c1a,
		output wire	[15:0]	err_c1b,
		output wire	[15:0]	first_err_ans,
		output wire	[15:0]	first_err_info,
		output wire	[15:0]	first_err_exp
);

	wire [2:0] mosi_sel;
	wire m0, m_early;
	reg mosi_1, mosi_3, mosi_tx;
	wire mosi_0, mosi_2;
	wire chk_en;
	wire stepper_busy;
	
	assign mosi_sel = mosi_scan_ctrl[2:0];
	assign chk_en = mosi_scan_ctrl[3];
	
	assign phase_info = {stepper_busy, 3'b000, stepper_spt};
		
	// MOSI sources
	
	assign m0 = spi1_mosi_in;
	assign m_early = a_next;
	
	assign spi1_mosi_out = ( mosi_sel == 3'd4 ) ? m0 : mosi_tx;

	assign mosi_0 = m_early;
	
	always@(negedge rst_n or negedge clk)
		if (!rst_n)	mosi_1 <= 1'b0;
		else mosi_1 <= m_early;
	
	assign mosi_2 = m0;
	
	always@(negedge rst_n or negedge clk)
		if (!rst_n)	mosi_3 <= 1'b0;
		else mosi_3 <= m0;
	
	always@(negedge rst_n or posedge clk_tx)
		if (!rst_n)	mosi_tx <= 1'b0;
		else begin
			case (mosi_sel[1:0])
				2'd0: mosi_tx <= mosi_0;
				2'd1: mosi_tx <= mosi_1;
				2'd2: mosi_tx <= mosi_2;
				2'd3: mosi_tx <= mosi_3;
				default: mosi_tx <= mosi_0;
			endcase
		end
	
	
	// Phase Stepper
	phase_stepper stepper0 (
		// inputs
		.rst_n	(rst_n),
		.clk	(clk),
		.cmd_wr	(phase_step_wr),
		.cmd	(phase_step_cmd),
		.spt	(stepper_spt),
		// outputs
		.phasestep	(phasestep),
		.phasedir	(phasedir),
		.phase_pos	(phase_pos),
		.busy		(stepper_busy)
	);
	
	//always@(negedge rst_n or posedge clk)
		//if (!rst_n)
			//x <= 1'b0;
		//else
			//x <= x;
	
	// Checker
	mosi_checker mosi_checker0 (
		// inputs
		.rst_n	(rst_n),
		.clk	(clk),
		.rhd_start	(rhd_start),
		.cmd	(data_tx),
		.xfer_done	(xfer_done),
		.drx_a0	(drx_a0),
		.drx_b0	(drx_b0),
		.drx_a1	(drx_a1),
		.drx_b1	(drx_b1),
		.chk_en	(chk_en),
		.chk_clear	(chk_clear),
		// outputs
		.chk_count	(chk_count),
		.err_c0a	(err_c0a),
		.err_c0b	(err_c0b),
		.err_c1a	(err_c1a),
		.err_c1b	(err_c1b),
		.first_err_ans (first_err_ans),
		.first_err_info (first_err_info),
		.first_err_exp (first_err_exp)
	);
		

endmodule






module mosi_checker (
		// inputs
	input  wire	rst_n,
	input  wire	clk,
	input  wire	rhd_start,
	input  wire	[15:0]	cmd,
	input  wire	xfer_done,
	input  wire	[15:0]	drx_a0,
	input  wire	[15:0]	drx_b0,
	input  wire	[15:0]	drx_a1,
	input  wire	[15:0]	drx_b1,
	input  wire	chk_en,
	input  wire	chk_clear,
		// outputs
	output reg	[15:0]	chk_count,
	output reg	[15:0]	err_c0a,
	output reg	[15:0]	err_c0b,
	output reg	[15:0]	err_c1a,
	output reg	[15:0]	err_c1b,
	output reg	[15:0]	first_err_ans,
	output reg	[15:0]	first_err_info,
	output reg	[15:0]	first_err_exp
);

	// states
	localparam IDLE = 2'd0;
	localparam DECODE = 2'd1;
	localparam EVAL = 2'd2;
	
	reg [1:0] current_state,next_state;

	// wires & regs
	reg chk_count_en, err_c0a_en, err_c0b_en, err_c1a_en, err_c1b_en;
	reg first_err_en;
	reg all_rst;
	reg [15:0] h2, h1, h0;
	
	
	// cmd history
	always@(negedge rst_n or posedge clk)
		if (!rst_n) begin
			h2 <= 16'd0;
			h1 <= 16'd0;
			h0 <= 16'd0;
		end
		else if (rhd_start) begin
			h2 <= h1;
			h1 <= h0;
			h0 <= cmd;
		end
	
	wire [1:0] h2_op;
	wire [5:0] h2_r;
	wire [7:0] h2_d;
	reg checked, decode_en;
	reg [15:0] expected;
	
	assign h2_op = h2[15:14];
	assign h2_r = h2[13:8];
	assign h2_d = h2[7:0];
	
	
	// decode registers
	always@(negedge rst_n or posedge clk)
		if (!rst_n) begin
			checked <= 1'b0;
			expected <= 16'd0;
		end
		else if (decode_en) begin
			checked <= 1'b0;	// default: not checked
			expected <= 16'd0;
			if (h2_op == 2'b11) begin
				case (h2_r)
					6'd40: begin
						expected <= 16'h0049; // I
						checked <= 1'b1;
					end
					6'd41: begin
						expected <= 16'h004e; // N
						checked <= 1'b1;
					end
					6'd42: begin
						expected <= 16'h0054; // T
						checked <= 1'b1;
					end
					6'd43: begin
						expected <= 16'h0041; // A
						checked <= 1'b1;
					end
					6'd44: begin
						expected <= 16'h004e; // N
						checked <= 1'b1;
					end
					6'd48: begin
						expected <= 16'h0052; // R
						checked <= 1'b1;
					end
					6'd49: begin
						expected <= 16'h0048; // H
						checked <= 1'b1;
					end
					6'd50: begin
						expected <= 16'h0044; // D
						checked <= 1'b1;
					end
					default: ;
				endcase
			end
			else if (h2_op == 2'b10) begin
				checked <= 1'b1;
				expected <= {8'hff, h2_d};
			end
		end

	
	// counters
	always@(negedge rst_n or posedge clk)
		if (!rst_n) begin
			chk_count <= 16'd0;
			err_c0a <= 16'd0;
			err_c0b <= 16'd0;
			err_c1a <= 16'd0;
			err_c1b <= 16'd0;
		end
		else if (chk_clear) begin
			chk_count <= 16'd0;
			err_c0a <= 16'd0;
			err_c0b <= 16'd0;
			err_c1a <= 16'd0;
			err_c1b <= 16'd0;
		end
		else if (chk_en) begin
			if (chk_count_en && (chk_count != 16'hffff) )
				chk_count <= chk_count + 1;
			if (err_c0a_en && (err_c0a != 16'hffff) )
				err_c0a <= err_c0a + 1;
			if (err_c0b_en && (err_c0b != 16'hffff) )
				err_c0b <= err_c0b + 1;
			if (err_c1a_en && (err_c1a != 16'hffff) )
				err_c1a <= err_c1a + 1;
			if (err_c1b_en && (err_c1b != 16'hffff) )
				err_c1b <= err_c1b + 1;
		end
	
	// first error
	always@(negedge rst_n or posedge clk)
		if (!rst_n) begin
			first_err_ans <= 16'd0;
			first_err_info <= 16'd0;
			first_err_exp <= 16'd0;
		end
		else if (chk_clear) begin
			first_err_ans <= 16'd0;
			first_err_info <= 16'd0;
			first_err_exp <= 16'd0;
		end
		else begin
			if (first_err_en) begin
				if (err_c0a_en) begin
					first_err_ans <= drx_a0;
					first_err_info <= {1'b1, 13'd0, 2'd0};
					first_err_exp <= expected;
				end
				else if (err_c0b_en) begin
					first_err_ans <= drx_b0;
					first_err_info <= {1'b1, 13'd0, 2'd1};
					first_err_exp <= expected;
				end
				else if (err_c1a_en) begin
					first_err_ans <= drx_a1;
					first_err_info <= {1'b1, 13'd0, 2'd2};
					first_err_exp <= expected;
				end
				else if (err_c1b_en) begin
					first_err_ans <= drx_b1;
					first_err_info <= {1'b1, 13'd0, 2'd3};
					first_err_exp <= expected;
				end
			end
		end

	
	
	// FSM
	
	// state register
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			current_state <= IDLE;
		else
			current_state <= next_state;
	
	// next_state logic
	always@(*) begin
		// default values
		chk_count_en = 1'b0;
		err_c0a_en = 1'b0;
		err_c0b_en = 1'b0;
		err_c1a_en = 1'b0;
		err_c1b_en = 1'b0;
		first_err_en = 1'b0;
		decode_en = 1'b0;
		
		case (current_state)
			IDLE: begin
				if (xfer_done) begin
					next_state = DECODE;
					decode_en = 1'b1;
				end
				else
					next_state = IDLE;
				
			end
			
			DECODE: begin
				if ( checked && chk_en )
					next_state = EVAL;
				else
					next_state = IDLE;
			end
			
			EVAL: begin
				next_state = IDLE;
				chk_count_en = 1'b1;
				if (!first_err_info[15])
					first_err_en = 1'b1;
				if (drx_a0 != expected)
					err_c0a_en = 1'b1;
				if (drx_b0 != expected)
					err_c0b_en = 1'b1;
				if (drx_a1 != expected)
					err_c1a_en = 1'b1;
				if (drx_b1 != expected)
					err_c1b_en = 1'b1;
			end
			
			// Other States
			default: begin
				next_state = IDLE;
			end
		endcase
	end
	
	


endmodule





module phase_stepper (
	// inputs
	input  wire	rst_n,
	input  wire	clk,
	input  wire	cmd_wr,
	input  wire	[15:0]	cmd,
	input  wire	[11:0]	spt,
	// outputs
	output reg	phasestep,
	output reg	phasedir,
	output wire	[8:0]	phase_pos,
	output reg	busy
);

	
	// States
	localparam IDLE = 3'd0;
	localparam WAIT = 3'd1;
	localparam PULSE = 3'd2;
	
	reg [2:0] current_state,next_state;
	reg [10:0] cnt0;
	reg [8:0] phase;
	reg [9:0] n;
	reg phase_en, cnt0_rst, cnt0_en, in_reg_en;
	reg phasestep_fsm;
	
	
	// 9-bit counter for phase
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			phase <= 0;
		else if (phase_en)
			if ( !phasedir )
				if ( phase == (spt - 12'd1) )
					phase <= 0;
				else
					phase <= phase + 1;
			else
				if ( phase == 0 )
					phase <= spt - 12'd1;
				else
					phase <= phase - 1;

	assign phase_pos = phase;
		
	
	// register inputs
	always@(negedge rst_n or posedge clk)
		if (!rst_n) begin
			n <= 0;
			phasedir <= 0;
		end
		else if (in_reg_en) begin
			n <= cmd[9:0];
			phasedir <= cmd[15];
		end
	
	
	
	// register outputs
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			phasestep <= 1'b1;
		else
			phasestep <= phasestep_fsm;
	
	// cnt0: auxiliary counter
	always@(posedge clk)
		if (cnt0_rst)
			cnt0 <= 0;
		else if (cnt0_en)
			cnt0 <= cnt0 + 1;
	
	
	// FSM
	
	// state register
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			current_state <= IDLE;
		else
			current_state <= next_state;
	
	// next_state logic
	always@(*) begin
		// default values
		phase_en = 1'b0;
		phasestep_fsm = 1'b1;
		cnt0_en = 1'b0;
		cnt0_rst = 1'b0;
		busy = 1'b0;
		in_reg_en = 1'b0;
		
		case (current_state)
			IDLE: begin
				if ( cmd_wr && (cmd[9:0] != 0)) begin
					next_state = WAIT;
					in_reg_en = 1'b1;
				end
				else
					next_state = IDLE;
				
				cnt0_rst = 1'b1;
			end
			
			WAIT: begin
				next_state = PULSE;
				cnt0_en = 1'b1;
				phasestep_fsm = 1'b0; // send active-low pulse to PLL
				busy = 1'b1;
			end
			
			PULSE: begin
				if (cnt0 < {n,1'b0}) begin // cnt0 < 2*n
					next_state = PULSE;
					if (!phasestep) begin
						phase_en = 1'b1;
						phasestep_fsm = 1'b1;
					end
					else
						phasestep_fsm = 1'b0;
					busy = 1'b1;
				end
				else begin
					next_state = IDLE;
					busy = 1'b0;
				end
				cnt0_en = 1'b1;
			end
			
			// Other States
			default: begin
				next_state = IDLE;
			end
		endcase
	end







endmodule


module clock_gen #(
	parameter [3:0]	VARIANT = 4'd2
)(
	input  wire	clkin,
	output wire	clkop,
	output wire	clkos,
	output wire	pll_lock,
	input  wire	phasestep,
	input  wire	phasedir,
	output wire	[11:0]	spt
);
	
	wire phaseloadreg;
	wire [2:0] phasesel;
	
	generate
		if ( (VARIANT < 4'd1) || (VARIANT > 4'd3) ) begin: bad_variant
			KUNTUR_VARIANT_must_be_1_2_or_3 stop();	// no such module: the build fails here
		end
	endgenerate
	
	assign phaseloadreg = 1'b0;
	assign phasesel = 3'b001;
	
	
	
	generate
		if (VARIANT==4'd1) begin: g_pll			// pll_28k
			// CLKOP_FREQ_ACTUAL(42.504012)
			// DIVOS_ACTUAL_STR("36")
			localparam SPT_VALUE = 12'd296;
			
			assign spt = SPT_VALUE;
			
			pll_28k pll_28k_0 (
				.clki_i			(clkin), 
				.phasedir_i		(phasedir),
				.phasestep_i		(phasestep),
				.phaseloadreg_i	(phaseloadreg),
				.phasesel_i		(phasesel),
				.clkop_o			(clkop),
				.clkos_o			(clkos),
				.lock_o			(pll_lock)
			);
		end
		else begin: g_pll
			// pll_25k
			// CLKOP_FREQ_ACTUAL(37.949963)
			// DIVOS_ACTUAL_STR("41")
			localparam SPT_VALUE = 12'd336;
			
			assign spt = SPT_VALUE;

			pll_25k pll_25k_0 (
				.clki_i			(clkin), 
				.phasedir_i		(phasedir),
				.phasestep_i		(phasestep),
				.phaseloadreg_i	(phaseloadreg),
				.phasesel_i		(phasesel),
				.clkop_o			(clkop),
				.clkos_o			(clkos),
				.lock_o			(pll_lock)
			);
		end
	endgenerate
	
endmodule









module frame_selector #( 
	parameter [3:0]		VARIANT = 4'd2,
	parameter [11:0]	RELEASE = 12'd0
)(
	input  wire	[7:0]	fb_dout,
	input  wire	[7:0]	bg_dout,
	output wire	[7:0]	tun_data,
	output wire	[15:0]	design_id
);

	generate
		if ( (VARIANT < 4'd1) || (VARIANT > 4'd3) ) begin: bad_variant
			KUNTUR_VARIANT_must_be_1_2_or_3 stop();	// no such module: the build fails here
		end
	endgenerate

	assign tun_data =	(VARIANT == 4'd2) ? fb_dout :
						(VARIANT == 4'd3) ? bg_dout : 8'h00;
	
	assign design_id = {VARIANT, RELEASE};


endmodule




module reset_gen_kuntur (
	// inputs
	input  wire	clkin,	// 32 MHz from MCU's MCO (G8)
	input  wire	rstb,	// Async active low reset (E3)
	input  wire	pll_lock,	// Async input from PLL. It is 1 when PLL is locked.
	input  wire	clk,	// PLL0 CLKOP
	input  wire	wclk,	// ECLK div by 4
	input  wire	gddr_rdy,	// GDDR_SYNC block READY output. It is 1 when FPGA start-up sequence is done.
	// outputs
	output reg	lock_sync,	// Active high, CLKIN domain
	output reg	rst_n_clkin,	// Active low, CLKIN domain
	output reg	gddr_rst,	// Active high, CLKIN domain
	output reg	rst_n_clk,	// Active low, CLK domain
	output reg	blk_rstb	// Active low, WCLK domain
);
	
	
	// ***************************************
	// rst_n_clkin generation - BEGIN
	// ***************************************
	reg [15:0] cnt0;
	wire clr;
	
	assign clr = rstb && pll_lock;
	
	always@(negedge clr or posedge clkin)
		if (!clr)
			rst_n_clkin <= 1'b0;
		else if (cnt0 == 16'd32768)
			rst_n_clkin <= 1'b1;
	
	always@(negedge clr or posedge clkin)
		if (!clr)
			cnt0 <= 0;
		else if (!lock_sync)
			cnt0 <= 0;
		else if (cnt0 != 16'd32768)
				cnt0 <= cnt0 + 1;
	
	
	// ***************************************
	// lock_sync generation - BEGIN
	// ***************************************
	reg pll_lock_reg;
	wire lock_sync_rstb;
	
	assign lock_sync_rstb = rstb && pll_lock;
	
	always@(negedge lock_sync_rstb or posedge clkin)
		if (!lock_sync_rstb) begin
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
	
	always@(negedge clr or posedge clkin)
		if (!clr)
			gddr_rst <= 1'b1;
		else
			gddr_rst <= !rst_n_clkin;
	
	
	// ***************************************
	// rst_n_clk generation - BEGIN
	// ***************************************
	reg rst_n_clk_reg;
	
	always@(negedge rst_n_clkin or posedge clk)
		if (!rst_n_clkin) begin
			rst_n_clk <= 1'b0;
			rst_n_clk_reg <= 1'b0;
		end
		else begin
			rst_n_clk <= rst_n_clk_reg;
			rst_n_clk_reg <= 1'b1;
		end
	
	
	// ***************************************
	// blk_rstb generation - BEGIN
	// ***************************************
	reg blk_rstb_1;
	wire blk_go;
	
	assign blk_go = rst_n_clkin && gddr_rdy;
	
	always@(negedge blk_go or posedge wclk)
		if (!blk_go) begin
			blk_rstb <= 1'b0;
			blk_rstb_1 <= 1'b0;
		end
		else begin
			blk_rstb <= blk_rstb_1;
			blk_rstb_1 <= 1'b1;
		end
	

endmodule






module block_gen(
	input  wire	rstb,
	input  wire	clk,
	output reg	[7:0]	dout
);

	localparam byte0 = 8'ha5;
	localparam byte1 = 8'h5a;
	localparam byte2 = 8'ha5;
	localparam byte3 = 8'h5a;
	localparam byte6 = 8'h01;
	localparam byte7 = 8'h80;
	
	reg [2:0] cnt0;
	reg [15:0] cnt1;
	
	// 3-bit cnt0
	always@(negedge rstb or posedge clk)
		if (!rstb)
			cnt0 <= 0;
		else
			cnt0 <= cnt0 + 1;
	
	// 16-bit cnt1
	always@(negedge rstb or posedge clk)
		if (!rstb)
			cnt1 <= 0;
		else if (cnt0==3'd7)
			cnt1 <= cnt1 + 1;
	
	// reg-mux
	always@(negedge rstb or posedge clk)
		if (!rstb)
			dout <= 0;
		else
			case (cnt0)
				3'd0:dout <= byte0;
				3'd1:dout <= byte1;
				3'd2:dout <= byte2;
				3'd3:dout <= byte3;
				3'd4:dout <= cnt1[7:0];
				3'd5:dout <= cnt1[15:8];
				3'd6:dout <= byte6;
				3'd7:dout <= byte7;
				default: dout <= 8'd0;
			endcase
	
endmodule




module frame_builder (
	// inputs - SAMPLE
	input  wire	rst_n,
	input  wire	clk, // wclk
	input  wire	frame_rdy,
	input  wire	rbank,
	input  wire	[31:0]	frame_index,
	input  wire	[15:0]	rd_data,
	// inputs - CONFIG / STATUS
	input  wire	[7:0]	flags,
	input  wire	send_config,
	input  wire	send_status,
	input  wire	[7:0]	cfg_byte,
	input  wire	[15:0]	alt_cmd,
	input  wire	[63:0]	alt_answer,
	input  wire	[15:0]	design_id,
	// outputs
	output reg	[7:0]	dout_reg,
	output reg	[7:0]	rd_addr,
	output reg	[4:0]	cfg_addr
);

	localparam IDLE			= 4'd0;
	localparam PREAMBLE		= 4'd1;
	localparam HEADER		= 4'd2;
	localparam PAYLOAD		= 4'd3;
	localparam CRC			= 4'd4;
	localparam C_PREAMBLE	= 4'd5;
	localparam C_HEADER		= 4'd6;
	localparam C_PAYLOAD		= 4'd7;
	localparam C_CRC			= 4'd8;
	localparam ST_PREAMBLE	= 4'd9;
	localparam ST_HEADER		= 4'd10;
	localparam ST_PAYLOAD	= 4'd11;
	localparam ST_CRC		= 4'd12;
	
	reg [3:0] current_state, next_state;
	
	localparam FRAME_MAX = 9'd272;
	
	reg in_reg_en, cnt0_rst, cnt0_en, rd_addr_en, crc_en, crc_load;
	reg [8:0] cnt0;
	wire [8:0] rd_addr_k, cfg_addr_k;
	reg rbank_reg;
	reg [31:0] frame_index_reg;
	reg frame_rdy_0, frame_rdy_1, frame_rdy_2;
	wire go;
	wire [31:0] crc_out;
	reg [7:0] dout;
	
	reg	[7:0]	flags_reg;
	reg	send_config_reg;
	reg	send_status_reg;
	reg	[15:0]	alt_cmd_reg;
	reg	[63:0]	alt_answer_reg;
	reg cfg_addr_en;
	
	// input registers
	always@(negedge rst_n or posedge clk)
		if (!rst_n) begin
			rbank_reg <= 0;
			frame_index_reg <= 0;
			flags_reg <= 0;
			send_config_reg <= 0;
			send_status_reg <= 0;
			alt_cmd_reg <= 0;
			alt_answer_reg <= 0;
		end
		else if (in_reg_en) begin
			rbank_reg <= rbank;
			frame_index_reg <= frame_index;
			flags_reg <= flags;
			send_config_reg <= send_config;
			send_status_reg <= send_status;
			alt_cmd_reg <= alt_cmd;
			alt_answer_reg <= alt_answer;
		end
		
	// aux counter
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			cnt0 <= 0;
		else if (cnt0_rst) 
			cnt0 <= 0;
		else if (cnt0_en && (cnt0 < FRAME_MAX ))
			cnt0 <= cnt0 + 1;
	
	// rd_addr
	assign rd_addr_k = cnt0 - 9'd10;
	
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			rd_addr <= 0;
		else if (cnt0_rst)
			rd_addr <= 0;
		else if (rd_addr_en)
			rd_addr <= {rbank_reg,rd_addr_k[7:1]};
			
	// cfg_addr
	assign cfg_addr_k = cnt0 - 9'd11;
	
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			cfg_addr <= 0;
		else if (cnt0_rst)
			cfg_addr <= 0;
		else if (cfg_addr_en)
			cfg_addr <= cfg_addr_k[4:0];
	
	// dout_reg
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			dout_reg <= 0;
		else
			dout_reg <= dout;
	
	
	// frame_rdy synchronizer and step detector
	always@(negedge rst_n or posedge clk)
		if (!rst_n) begin
			frame_rdy_0 <= 1'b0;
			frame_rdy_1 <= 1'b0;
			frame_rdy_2 <= 1'd0;
		end
		else begin
			frame_rdy_0 <= frame_rdy;
			frame_rdy_1 <= frame_rdy_0;
			frame_rdy_2 <= frame_rdy_1;
		end
	
	assign go = frame_rdy_2 ^ frame_rdy_1;
	
	
	// CRC
	
	crc32 checksum (
		.rst_n	(rst_n),
		.load	(crc_load),
		.en		(crc_en),
		.clk	(clk),
		.din	(dout),
		.crc_out	(crc_out)
	);
	
	// FSM
	
	// state register
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			current_state <= IDLE;
		else
			current_state <= next_state;
	
	// next_state and output logic
	always@(*) begin
		// default values
		next_state = current_state;
		dout = 8'd0;
		in_reg_en = 1'b0;
		cnt0_rst = 1'b0;
		cnt0_en = 1'b0;
		crc_en = 1'b0;
		crc_load = 1'b0;
		rd_addr_en = 1'b0;
		cfg_addr_en = 1'b0;
		
		case (current_state)
			IDLE: begin
				if (go) begin
					next_state = PREAMBLE;
					in_reg_en = 1'b1;
				end
				else
					next_state = IDLE;
				cnt0_rst = 1'b1;
			end
			
			// **************************************
			// SAMPLE
			// **************************************
			
			PREAMBLE: begin
				if (cnt0 == 9'd3) begin
					next_state = HEADER;
					crc_en = 1'b1;
					crc_load = 1'b1;
				end
				else
					next_state = PREAMBLE;
				// dout logic
				if (cnt0 == 9'd0)
					dout = 8'ha5;
				else if (cnt0 == 9'd1)
					dout = 8'h5a;
				else if (cnt0 == 9'd2)
					dout = 8'ha5;
				else if (cnt0 == 9'd3)
					dout = 8'h5a;
				// increase counter
				cnt0_en = 1'b1;
			end
			
			HEADER: begin
				if (cnt0 == 9'd11) begin
					next_state = PAYLOAD;
				end
				else
					next_state = HEADER;
				// dout logic
				if (cnt0 == 9'd4)
					dout = 8'h01; // SAMPLE
				else if (cnt0 == 9'd5)
					dout = flags_reg;
				else if (cnt0 == 9'd6)
					dout = frame_index_reg[7:0];
				else if (cnt0 == 9'd7)
					dout = frame_index_reg[15:8];
				else if (cnt0 == 9'd8)
					dout = frame_index_reg[23:16];
				else if (cnt0 == 9'd9)
					dout = frame_index_reg[31:24];
				else if (cnt0 == 9'd10) begin
					dout = 8'h00;
					rd_addr_en = 1'b1;
				end
				else if (cnt0 == 9'd11)
					dout = 8'h01;
				
				crc_en = 1'b1;
				// increase counter
				cnt0_en = 1'b1;
			end
			
			PAYLOAD: begin
				if (cnt0 == 9'd267) begin
					next_state = CRC;
				end
				else
					next_state = PAYLOAD;
				// dout logic
				if (cnt0[0])
					dout = rd_data[15:8];
				else
					dout = rd_data[7:0];
				rd_addr_en = 1'b1;
				
				crc_en = 1'b1;
				// increase counter
				cnt0_en = 1'b1;
			end
			
			CRC: begin
				if (cnt0 == 9'd271) begin
					if (send_config_reg) begin
						next_state = C_PREAMBLE;
						cnt0_rst = 1'b1;
					end
					else if (send_status_reg) begin
						next_state = ST_PREAMBLE;
						cnt0_rst = 1'b1;
					end
					else
						next_state = IDLE;
				end
				else
					next_state = CRC;
				// dout logic
				if (cnt0 == 9'd268)
					dout = crc_out[7:0];
				else if (cnt0 == 9'd269)
					dout = crc_out[15:8];
				else if (cnt0 == 9'd270)
					dout = crc_out[23:16];
				else if (cnt0 == 9'd271)
					dout = crc_out[31:24];
				
				// increase counter
				cnt0_en = 1'b1;
			end
			
			// **************************************
			// CONFIG
			// **************************************
			C_PREAMBLE: begin
				if (cnt0 == 9'd3) begin
					next_state = C_HEADER;
					crc_en = 1'b1;
					crc_load = 1'b1;
				end
				else
					next_state = C_PREAMBLE;
				// dout logic
				if (cnt0 == 9'd0)
					dout = 8'ha5;
				else if (cnt0 == 9'd1)
					dout = 8'h5a;
				else if (cnt0 == 9'd2)
					dout = 8'ha5;
				else if (cnt0 == 9'd3)
					dout = 8'h5a;
				// increase counter
				cnt0_en = 1'b1;
			end
			
			C_HEADER: begin
				if (cnt0 == 9'd11) begin
					next_state = C_PAYLOAD;
				end
				else
					next_state = C_HEADER;
				// dout logic
				if (cnt0 == 9'd4)
					dout = 8'h02; // CONFIG
				else if (cnt0 == 9'd5)
					dout = flags_reg;
				else if (cnt0 == 9'd6)
					dout = frame_index_reg[7:0];
				else if (cnt0 == 9'd7)
					dout = frame_index_reg[15:8];
				else if (cnt0 == 9'd8)
					dout = frame_index_reg[23:16];
				else if (cnt0 == 9'd9)
					dout = frame_index_reg[31:24];
				else if (cnt0 == 9'd10)
					dout = 8'h16;
				else if (cnt0 == 9'd11) begin
					dout = 8'h00;
					cfg_addr_en = 1'b1;
				end
				
				crc_en = 1'b1;
				// increase counter
				cnt0_en = 1'b1;
			end
			
			C_PAYLOAD: begin
				if (cnt0 == 9'd33) begin
					next_state = C_CRC;
				end
				else
					next_state = C_PAYLOAD;
				// dout logic
				dout = cfg_byte;
				cfg_addr_en = 1'b1;
				
				crc_en = 1'b1;
				// increase counter
				cnt0_en = 1'b1;
			end
			
			C_CRC: begin
				if (cnt0 == 9'd37) begin
					next_state = IDLE;
				end
				else
					next_state = C_CRC;
				// dout logic
				if (cnt0 == 9'd34)
					dout = crc_out[7:0];
				else if (cnt0 == 9'd35)
					dout = crc_out[15:8];
				else if (cnt0 == 9'd36)
					dout = crc_out[23:16];
				else if (cnt0 == 9'd37)
					dout = crc_out[31:24];
				
				// increase counter
				cnt0_en = 1'b1;
			end
			
			
			// **************************************
			// STATUS
			// **************************************
			ST_PREAMBLE: begin
				if (cnt0 == 9'd3) begin
					next_state = ST_HEADER;
					crc_en = 1'b1;
					crc_load = 1'b1;
				end
				else
					next_state = ST_PREAMBLE;
				// dout logic
				if (cnt0 == 9'd0)
					dout = 8'ha5;
				else if (cnt0 == 9'd1)
					dout = 8'h5a;
				else if (cnt0 == 9'd2)
					dout = 8'ha5;
				else if (cnt0 == 9'd3)
					dout = 8'h5a;
				// increase counter
				cnt0_en = 1'b1;
			end
			
			ST_HEADER: begin
				if (cnt0 == 9'd11) begin
					next_state = ST_PAYLOAD;
				end
				else
					next_state = ST_HEADER;
				// dout logic
				if (cnt0 == 9'd4)
					dout = 8'h03; // STATUS
				else if (cnt0 == 9'd5)
					dout = flags_reg;
				else if (cnt0 == 9'd6)
					dout = frame_index_reg[7:0];
				else if (cnt0 == 9'd7)
					dout = frame_index_reg[15:8];
				else if (cnt0 == 9'd8)
					dout = frame_index_reg[23:16];
				else if (cnt0 == 9'd9)
					dout = frame_index_reg[31:24];
				else if (cnt0 == 9'd10)
					dout = 8'h12;
				else if (cnt0 == 9'd11)
					dout = 8'h00;
				
				crc_en = 1'b1;
				// increase counter
				cnt0_en = 1'b1;
			end
			
			ST_PAYLOAD: begin
				if (cnt0 == 9'd29) begin
					next_state = ST_CRC;
				end
				else
					next_state = ST_PAYLOAD;
				// dout logic
				if (cnt0 == 9'd12)
					dout = design_id[7:0];
				else if (cnt0 == 9'd13)
					dout = design_id[15:8];
				else if (cnt0 == 9'd14)
					dout = 8'h06; // valid
				else if (cnt0 == 9'd15)
					dout = 8'h00;
				else if (cnt0 == 9'd16)
					dout = alt_cmd_reg[7:0];
				else if (cnt0 == 9'd17)
					dout = alt_cmd_reg[15:8];
				else if (cnt0 == 9'd18)
					dout = alt_answer_reg[55:48];
				else if (cnt0 == 9'd19)
					dout = alt_answer_reg[63:56];
				else if (cnt0 == 9'd20)
					dout = alt_answer_reg[39:32];
				else if (cnt0 == 9'd21)
					dout = alt_answer_reg[47:40];
				else if (cnt0 == 9'd22)
					dout = alt_answer_reg[23:16];
				else if (cnt0 == 9'd23)
					dout = alt_answer_reg[31:24];
				else if (cnt0 == 9'd24)
					dout = alt_answer_reg[7:0];
				else if (cnt0 == 9'd25)
					dout = alt_answer_reg[15:8];
				else if (cnt0 == 9'd26)
					dout = 8'h00;
				else if (cnt0 == 9'd27)
					dout = 8'h00;
				else if (cnt0 == 9'd28)
					dout = 8'h00;
				else if (cnt0 == 9'd29)
					dout = 8'h00;
				
				crc_en = 1'b1;
				// increase counter
				cnt0_en = 1'b1;
			end
			
			ST_CRC: begin
				if (cnt0 == 9'd33) begin
					next_state = IDLE;
				end
				else
					next_state = ST_CRC;
				// dout logic
				if (cnt0 == 9'd30)
					dout = crc_out[7:0];
				else if (cnt0 == 9'd31)
					dout = crc_out[15:8];
				else if (cnt0 == 9'd32)
					dout = crc_out[23:16];
				else if (cnt0 == 9'd33)
					dout = crc_out[31:24];
				
				// increase counter
				cnt0_en = 1'b1;
			end
			
			
			// **************************************
			// OTHER STATES
			// **************************************
			default: begin
				next_state = IDLE;
			end
		endcase
	end


endmodule




module tun_aux (
	// inputs
	input  wire	rst_n, // active-low reset, same as the controller's
	input  wire	clk, // PLL
	input  wire	rhd_start, // a transfer is starting (op0b in the config walk, op1b while sampling)
	input  wire [15:0]	cmd, // command word being sent in that transfer, valid at rhd_start
	input  wire	announce, // one-clk pulse: xfer_done && t == 1 && primed,
	input  wire	stream_enable, // flags bit 0
	input  wire	[1:0]	data_source_sel, // flags bit 1 = (data_source_sel == 2'd1), matching fifo_din_mux
	input  wire	[4:0]	cfg_addr, // which CONFIG byte to read, 0-21
	// outputs
	output reg	[7:0]	flags_snap, // {4'b0, 1'b0, dirty, test_pattern, stream_enable}: byte 5 of the SAMPLE and of the aux frame after it
	output reg	send_config, // a CONFIG follows this SAMPLE
	output reg	send_status, // a STATUS follows this SAMPLE
	output wire	[7:0]	cfg_byte // shadow_snap[cfg_addr]: a combinational mux of the snapshot	
);

	reg [7:0] shadow [0:21];
	reg [7:0] shadow_snap [0:21];
	wire [5:0] cmd_r;
	wire [7:0] cmd_d;
	reg dirty, st_pending;
	reg [9:0] cfg_cnt;
	reg [8:0] st_cnt;
	integer i;
	wire cfg_due, st_due;
	
	assign cmd_r = cmd[13:8];
	assign cmd_d = cmd[7:0];
	assign cfg_due = dirty || (cfg_cnt == 10'd999);
	assign st_due = st_pending || (st_cnt == 9'd299);
	
	always@(negedge rst_n or posedge clk)
		if (!rst_n) begin
			dirty <= 1'b1;
			flags_snap <= 0;
			cfg_cnt <= 10'd999;
			st_cnt <= 9'd299;
			send_config <= 1'b0;
			send_status <= 1'b0;
			st_pending <= 1'b0;
		end
		else if (rhd_start) begin
			if ( (cmd[15:14] == 2'b10) && (cmd_r <= 6'd21) ) begin	// WRITE(R,D), RAM register
				if (shadow[cmd_r] != cmd_d)
					dirty <= 1'b1;
			end
		end
		else if (announce) begin
			flags_snap <= {4'd0, 1'b0, dirty, (data_source_sel == 2'd1), stream_enable};
			
			dirty <= 1'b0;
			if (cfg_due) begin
				send_config <= 1'b1;
				cfg_cnt <= 0;
			end
			else begin
				send_config <= 1'b0;
				cfg_cnt <= cfg_cnt + 1;
			end
			if ( st_due && !(cfg_due) ) begin
				send_status <= 1'b1;
				st_pending <= 1'b0;
				st_cnt <= 0;
			end
			else begin
				send_status <= 1'b0;
				st_pending <= st_due;
				if (!st_due)
					st_cnt <= st_cnt + 1;
			end
		end
	
	// writing
	always@(posedge clk) begin
			if ( (rhd_start) && (cmd[15:14] == 2'b10) && (cmd_r <= 6'd21) )	// WRITE(R,D), RAM register
				shadow[cmd_r] <= cmd_d;
			if (announce) begin
				for ( i = 0; i < 22; i = i + 1 ) begin
					shadow_snap[i] <= shadow[i];
				end
			end
		end
	
	// Reading
	assign cfg_byte = shadow_snap[cfg_addr];

endmodule





module capture_buffer (
	// inputs
	input  wire	rst_n, // active-low reset, same as the controller's
	input  wire	clk, // system clock clk
	input  wire	xfer_done, // one-cycle pulse: "a sampling transfer just finished" 
	input  wire	[5:0]	t, // which transfer just finished, 0-32, valid with xfer_done
	input  wire	[15:0]	d_a0, // answer received in that transfer
	input  wire	[15:0]	d_b0, // answer received in that transfer
	input  wire	[15:0]	d_a1, // answer received in that transfer
	input  wire	[15:0]	d_b1, // answer received in that transfer
	input  wire [15:0]	cmd, // command word being sent in that transfer
	input  wire	rd_clk, // read clock for the frame builder
	input  wire	[7:0]	rd_addr, // which word to read: {bank, slot[4:0], half[1:0]}
	// outputs
	output wire	[15:0]	rd_data, // the word at rd_addr (fixed read latency)
	output reg	frame_rdy, // toggles each time a full frame is ready
	output reg	rbank, // which bank holds the ready frame
	output reg	[31:0]	frame_index, // number of the ready frame
	output reg	[15:0]	alt_cmd, // the command sent in sampling entry 32
	output reg	[63:0]	alt_answer, // that command's four answers
	output wire	announce
);

	
	reg wen, bank;
	reg [15:0] wbank_din0;
	wire [7:0] wbank_addr0;
	reg [5:0] wbank_addr0_base;
	
	reg [31:0] cnt0;
	wire [4:0] slot;
	reg [2:0] cnt1;
	wire [1:0] halves;
	reg primed;
	
	// announce
	assign announce = xfer_done && (t == 6'd1) && primed;
	
	// wbank
	wbank wbank0 (
		.clk		(clk),			// Input clock
		.rd_clk		(rd_clk),		// Input Read Clock
		// Write side
		.wen		(wen),	// Input; wbank write enable
		.addr0		(wbank_addr0),	// Input Address, Port0 - 8 bits
		.din0		(wbank_din0),	// Input Data, Port0 - 16 bits
		// Read side
		.addr1		(rd_addr),	// Input Address, Port0 - 8 bits
		.dout1		(rd_data)	// Output Data, Port0 - 16 bits
	);
		
	
	// primed
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			primed <= 1'b0;
		else if ( xfer_done && (t == 6'd2) )
			primed <= 1'b1;
	
	
	// rbank
	always@(negedge rst_n or posedge clk)
		if (!rst_n)
			rbank <= 1'b0;
		else if (xfer_done)
			if (t == 6'd1)
				rbank <= bank;
	
	// frame_index
	always@(negedge rst_n or posedge clk)
		if (!rst_n) begin
			cnt0 <= 0;
			frame_rdy <= 1'b0;
			frame_index <= 32'd0;
		end
		else if (primed) begin
			if ( xfer_done && (t == 6'd1) ) begin
				frame_index <= cnt0;
				cnt0 <= cnt0 + 1;
				frame_rdy <= ~frame_rdy;
			end
		end
			
	
	// alt_cmd and alt_answer
	always@(negedge rst_n or posedge clk)
		if (!rst_n) begin
			alt_cmd <= 16'd0;
			alt_answer <= 64'd0;
		end
		else if (xfer_done)
			if (t == 6'd32)
				alt_cmd <= cmd;
			else if (t == 6'd1)
				alt_answer <= {d_a0, d_b0, d_a1, d_b1};
	
	
	// cnt1 = halves
	always@(negedge rst_n or posedge clk)
		if (!rst_n) begin
			cnt1 <= 3'd4;
			wen <= 1'b0;
		end
		else
			if (xfer_done) begin
				if (t == 6'd1) begin
					cnt1 <= 3'd4;
					wen <= 1'b0;
				end
				else begin
					cnt1 <= 0;
					wen <= 1'b1;
				end
			end
			else
				if (cnt1 < 3'd3) begin
					cnt1 <= cnt1 + 1;
					wen <= 1'b1;
				end
				else
					wen <= 1'b0;
				
	assign halves = cnt1[1:0];
	
	// t=32 gives slot 30 because 5-bit substraction wraps
	assign slot = (t == 6'd0) ? 5'd31:t[4:0] - 5'b00010;
	
	// write addr
	always@(negedge rst_n or posedge clk)
		if (!rst_n) begin
			wbank_addr0_base <= 0;
			bank <= 1'b0;
		end
		else
			if (xfer_done) begin
				if (t == 6'd0) begin
					wbank_addr0_base <= {bank, slot};
				end
				else if (t == 6'd1) begin
					bank <= ~bank;
				end
				else begin
					wbank_addr0_base <= {bank, slot};
				end
			end
	
	assign wbank_addr0 = {wbank_addr0_base, halves};
	
	always@(*)
		case (halves)
			2'd0: wbank_din0 = d_a0;
			2'd1: wbank_din0 = d_b0;
			2'd2: wbank_din0 = d_a1;
			2'd3: wbank_din0 = d_b1;
			default: wbank_din0 = d_a0;
		endcase


endmodule



module wbank (
	input  wire	clk,			// Input clock
	input  wire	rd_clk,			// Input Read Clock
	input  wire	wen,			// Input; RegBank write enable
	input  wire	[7:0] addr0,	// Input Address, Port0
	input  wire [15:0] din0,		// Input Data, Port0
	input  wire	[7:0] addr1,	// Input Address, Port0
	output reg	[15:0] dout1	// Output Data, Port0
);
	
	reg [15:0] ram [0:255];
	
	// writing
	always@(posedge clk)
		if (wen)
			ram[addr0] <= din0;

	// Reading
	always@(posedge rd_clk)
		dout1 <= ram[addr1];
		
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
	assign next_0 = crc ^ {24'd0,din};
	
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
		else if (en) begin
			if (load)
				crc <= CRC_INIT;
			else
				crc <= next_8;
		end
	
	assign crc_out = ~crc;

endmodule
