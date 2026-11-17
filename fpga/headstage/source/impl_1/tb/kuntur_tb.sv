//testbench
//
//	Self-checking testbench for the SPI0 command protocol (A.1.1g).
//	SystemVerilog: compile with -sv, or keep the .sv extension.
//
//	SPI0 word format:  {OPCODE[1:0], TAG[5:0], DATA[7:0]}
//
//	  REG_WRITE  01  3 transfers, tags 1/2/3 = addr / high byte / commit-low
//	  REG_READ   10  1 transfer,  tag 1, carries its own address;
//	                 the value arrives on the FOLLOWING transfer
//	  FIFO_POP   00  2 transfers, both must be POP; ChA arrives on transfer 2,
//	                 ChB on the transfer after the pair
//	  NOP        11  1 transfer,  no side effects, never swallows the next one
//
//	Also checks R-K5 (cfg_done) and the RHD2164 config walk on the SPI1
//	pins, across start-up and PLL lock drops (T0, T8, and the spi1_*
//	monitor below).
//
//	K8: the capture buffer, tun_aux and the frame builder (T9, T10 and the
//	K8 receiver below). Every frame on tun_data_8b is decoded and
//	compared, CRC included, for the whole run: SAMPLE with what the
//	RHD2164 models answered, slot by slot; CONFIG with the WRITEs they
//	executed; STATUS with the build header and the entry-32 answer; and
//	which auxiliary frame follows which SAMPLE.
//
//	#6a (T12, spi1-mosi-eye-scan.md §3.2/§3.4): PHASE_STEP_CMD (233)
//	written through SPI0 steps the PLL model by N, the stepper's
//	PHASE_POS agrees and wraps at clock_gen's SPT; MOSI_SCAN_CTRL (232)
//	gives a one-clk chk_clear and reads bit 4 back as 0.
//	T13 (§3.1): for each MOSI source at allowed phases, every edge on
//	spi1_mosi is an edge of today's MOSI moved by the predicted Delta,
//	the chips still decode every command; the bypass gives Delta = 0.
//	T14 (§3.3, last in the run): the checker (words 242-249) against a
//	reference built from the pins and the models, with errors injected
//	in the models' WRITE answers (rhd2164_bfm inj_*): which commands
//	count, per-half errors, first error and its priority, CHK_EN hold,
//	CHK_CLEAR, saturation at FFFF, readback over SPI0.
//
//	K4.3 (T11): the build variant decides what goes on the tunnel. 2 runs
//	T9/T10 above; 3 checks the K4.2 pattern instead, 1 checks for 00
//	only. Run all three; 0 and 4..F must fail to elaborate
//	(frame_selector's guard).
//
//	Run (QuestaSim from Radiant; no +define+SIM, kuntur_prims_sim.v stands
//	in for the Lattice blocks). From any scratch directory, R = source/impl_1:
//	  vlib work
//	  vlog -sv +incdir+$R +incdir+$R/tb +incdir+$R/afe/rhd2164 +incdir+$R/app $R/tb/kuntur_tb.sv
//	  vsim -c kuntur_tb -do "run -all; quit -f"
//	The variant must be given (kuntur_build.vh defaults to 0, which stops
//	the build): add +define+KUNTUR_VARIANT=4'd2 (or 4'd3, 4'd1) to vlog.
//	About 37 ms of simulated time (T10 waits for 1000 frames), well under
//	a minute of run time.
//	Not Icarus: rhd2164_controller and spi_master_controller each assign
//	a default and then the real value in a combinational block, and each
//	is sensitive to the other's output (rhd_start, rhd_done), so Icarus
//	re-triggers them forever at the first transfer. Logic is fine.

`timescale 1ns/1ps
`include "kuntur_prims_sim.v"
`include "../kuntur_fpga.v"
`include "rhd2164_bfm.v"

module kuntur_tb();
	//parameter
	parameter n=16;
	// inputs
	reg rstb,clk,clk2;

	// outputs
	wire miso;
	wire cmd_is_00,cmd_is_01,cmd_is_02;

	// inputs
	reg spi1_start;
	wire spi1_miso0, spi1_miso1;
	// outputs
	wire spi1_csb,spi1_sck,spi1_mosi;
	wire spi1_done;

	wire csb,sck,mosi;


	kuntur_fpga dut (
		.rstb		(rstb),
		.clkin		(clk),
		// SPI to MCU
		.spi0_csb	(csb),	// F8 (PB20B)
		.spi0_sck	(sck),	// B2 (PT65A)
		.spi0_mosi	(mosi),	// B3 (PT63B)
		.spi0_miso	(miso),	// E2 (PR9B)
		// SPI to RHD2164
		.spi1_miso1	(spi1_miso1),	// G4 (LVDS0P)
		.spi1_miso0	(spi1_miso0),	// G3 (LVDS3P)
		.spi1_csb	(spi1_csb),	// H1 (LVDS5P)
		.spi1_sck	(spi1_sck),	// H2 (LVDS4P)
		.spi1_mosi	(spi1_mosi),	// E4 (LVDS2P)
		// SPI to RHS2116 (disabled in the RTL)
		.spi2_miso0	(1'b0),	// E7 (LVDS10P)
		.spi2_csb	(),		// G5 (LVDS11P)
		.spi2_sck	(),		// F7 (LVDS9P)
		.spi2_mosi0	(),		// E5 (LVDS1P)
		// LVDS through uHDMI
		.tun_data	(),		// G9 (LVDS7P, FPGA_LVDS1P)
		.tun_clk	(),		// E9 (LVDS6P, FPGA_LVDS2P)
		// TEMP signals
		.cmd_is_00	()
	);


	// clk @ 50MHz (target is <48 MHz with PLL)
	always #10	clk = ~clk;

	// clk @ 10/3 MHz (target is <48 MHz with PLL)
	always #150	clk2 = ~clk2;

	// RHD2164 models: two chips on one CS/SCLK/MOSI (rhd2164_bfm.v)
	rhd2164_bfm #(.CHIP(0)) chip0 (
		.csb	(spi1_csb),
		.sck	(spi1_sck),
		.mosi	(spi1_mosi),
		.miso	(spi1_miso0)
	);
	rhd2164_bfm #(.CHIP(1)) chip1 (
		.csb	(spi1_csb),
		.sck	(spi1_sck),
		.mosi	(spi1_mosi),
		.miso	(spi1_miso1)
	);

	reg mcu_start;
	wire mcu_done;
	wire [15:0] mcu_drx;
	reg [15:0] mcu_dtx;


	// MCU SPI Master
	// n is a localparam inside spi_master now (2026-08-26) — no longer
	// overridable, so no #(...) here.
	spi_master mcu_spi (
		.rstb		(rstb),		// Active-low asyncrhonous reset
		.clk		(clk2),		// Input clock
		.start		(mcu_start),		// Active-high signal that starts SPI cycle
		.csb		(csb),		// Active-low chip-select
		.sck		(sck),		// SPI Clock
		.mosi		(mosi),		// Master-Output Slave-Input
		.miso		(miso),		// Master-Input Slave-Output
		.data_tx	(mcu_dtx),	// Data to be transmitted
		.data_rx	(mcu_drx),	// DataA received
		.done		(mcu_done)		// TX/RX operation completed; no communicacion happening
	);


	// ------------------------------------------------------------------
	// Command encoding:  {OPCODE[1:0], TAG[5:0], DATA[7:0]}
	// ------------------------------------------------------------------
	localparam logic [1:0] OP_POP   = 2'b00;
	localparam logic [1:0] OP_WRITE = 2'b01;
	localparam logic [1:0] OP_READ  = 2'b10;
	localparam logic [1:0] OP_NOP   = 2'b11;

	localparam logic [15:0] CMD_POP = {OP_POP, 6'd0, 8'd0};
	localparam logic [15:0] CMD_NOP = {OP_NOP, 6'd0, 8'd0};

	function automatic logic [15:0] cmd_wr_addr(logic [7:0] a); return {OP_WRITE, 6'd1, a}; endfunction
	function automatic logic [15:0] cmd_wr_high(logic [7:0] d); return {OP_WRITE, 6'd2, d}; endfunction
	function automatic logic [15:0] cmd_wr_low (logic [7:0] d); return {OP_WRITE, 6'd3, d}; endfunction
	function automatic logic [15:0] cmd_read   (logic [7:0] a); return {OP_READ,  6'd1, a}; endfunction
	// Deliberately-malformed variant, for the abort tests
	function automatic logic [15:0] cmd_wr_tag(logic [5:0] t, logic [7:0] d);
		return {OP_WRITE, t, d};
	endfunction

	// Regbank addresses (mirrors intan.vh: CONFIG_BASE 0, SAMPLING_BASE 48, CTRL_BASE 192)
	localparam logic [7:0] RB_SAMP0     = 8'd48;	// reset default RHD_CONVERT(0)  = 0x0000
	localparam logic [7:0] RB_SAMP1     = 8'd49;	// reset default RHD_CONVERT(1)  = 0x0100
	localparam logic [7:0] RB_SAMP2     = 8'd50;	// reset default RHD_CONVERT(63) = 0x3F00
	localparam logic [7:0] RB_CH_A      = 8'd196;	// reset default 0x0083
	localparam logic [7:0] RB_CH_B      = 8'd197;	// reset default 0x0082
	localparam logic [7:0] RB_STREAM_EN = 8'd228;	// reset default 0x0001
	localparam logic [7:0] RB_SCAN_CTRL = 8'd232;	// #6a MOSI_SCAN_CTRL, reset 0x0004
	localparam logic [7:0] RB_STEP_CMD  = 8'd233;	// #6a PHASE_STEP_CMD, reset 0x0000
	localparam logic [7:0] RB_PHASE_POS = 8'd240;	// #6a read-only, from the stepper
	localparam logic [7:0] RB_PHASE_INFO = 8'd241;	// #6a read-only: [15] busy, [11:0] SPT

	// Payloads: both have a NON-ZERO high byte, so a staged_h that is never
	// written (or truncated away) reads back as 0x00xx and fails.
	localparam logic [15:0] VAL_A = 16'h95A5;	// = RHD_WRITE(6'd21, 8'hA5)
	localparam logic [15:0] VAL_B = 16'h3C5A;


	// ------------------------------------------------------------------
	// Tasks
	// ------------------------------------------------------------------

	// ---- self-check ----
	int errors = 0, checks = 0;
	logic [15:0] rx;		// MISO word of the transfer that just completed

	// One SPI0 transfer. Handshakes on mcu_done instead of using a fixed
	// settle delay, so rx is valid the moment this returns and there is no
	// delay to re-tune when SPI timing changes.
	//
	// wait() (level) not @(negedge) (edge): mcu_done falls a few clk2 cycles
	// after mcu_start rises, i.e. INSIDE the start pulse itself. An
	// edge-triggered wait misses it and blocks forever.
	task automatic spi_xfer(logic [15:0] word);
		wait (mcu_done === 1'b1);	// interface idle
		#12000 mcu_dtx = word;
		mcu_start = 1'b1;
		wait (mcu_done === 1'b0);	// frame under way
		mcu_start = 1'b0;
		wait (mcu_done === 1'b1);	// frame complete, data_rx settled
		#10;
		rx = mcu_drx;
	endtask

	// REG_WRITE: 3 tagged transfers
	task automatic reg_write16(logic [7:0] addr, logic [15:0] val);
		spi_xfer(cmd_wr_addr(addr));
		spi_xfer(cmd_wr_high(val[15:8]));
		spi_xfer(cmd_wr_low (val[7:0]));
	endtask

	// REG_READ: self-addressing; the value lands on the FOLLOWING transfer,
	// so a NOP is issued as the carrier.
	task automatic reg_read16(logic [7:0] addr, output logic [15:0] val);
		spi_xfer(cmd_read(addr));
		spi_xfer(CMD_NOP);
		val = rx;
	endtask

	// FIFO_POP pair. spi0_dtx is loaded AFTER a frame completes, so ChA lands
	// on transfer 2 and ChB on the transfer after the pair.
	task automatic fifo_pop_pair(output logic [15:0] chA, output logic [15:0] chB);
		spi_xfer(CMD_POP);
		spi_xfer(CMD_POP);	chA = rx;
		spi_xfer(CMD_NOP);	chB = rx;
	endtask

	task automatic check(string name, logic [15:0] got, logic [15:0] exp);
		checks++;
		assert (got === exp)
			$display("  pass  %0s: %04h", name, got);
		else begin
			errors++;
			$error("%0s: got %04h, expected %04h", name, got, exp);
		end
	endtask

	// ch_sel currently assigns dout = {cnt0, cnt0+1000} (the A.1 ramp), so a
	// matched pair satisfies ChB - ChA == 1000 exactly. A phase slip flips
	// that to -1000 (= 64536). 0x8000/0x8000 is the underrun sentinel.
	task automatic check_pair(string name, logic [15:0] chA, logic [15:0] chB);
		checks++;
		assert (((chA === 16'h8000) && (chB === 16'h8000)) ||
		        ((chB - chA) === 16'd1000))
			$display("  pass  %0s: ChA=%04h ChB=%04h", name, chA, chB);
		else begin
			errors++;
			$error("%0s: ChA=%04h ChB=%04h, delta %0d (want 1000)",
			       name, chA, chB, chB - chA);
		end
	endtask

	task automatic banner(string name);
		$display("");
		$display("--- %0s", name);
	endtask


	// ------------------------------------------------------------------
	// R-K5: cfg_done and the config walk, checked on the SPI1 pins
	// ------------------------------------------------------------------
	// Decodes every SPI1 transfer from the pins (MOSI sampled on SCK
	// rising, MSB first) and compares it with the regbank entry the
	// controller should be issuing: config entries 0..CFG_N-1 after every
	// reset, then the sampling table, 48 + (k mod 33). cfg_done must be
	// low for the whole config walk and high from the first sampling
	// transfer on, and may fall only while rst_n_clk is low.
	//
	// Checking the pins, not rhd_start, is the point: a controller reset
	// without its SPI master still pulses rhd_start 34 times, but the
	// commands on the wire then fall out of step with the table.

	localparam int CFG_N  = 34;		// RB_CONFIG_MAX + 1
	localparam int SAMP_N = 33;		// RB_SAMPLING_MAX + 1
	localparam int SAMP_BASE = 48;	// RB_SAMPLING_BASE

	int xfer_idx = 0;		// transfers since rst_n_clk last rose
	int xfer_total = 0;		// transfers checked, whole run
	int cfg_rises = 0;		// cfg_done rising edges, whole run
	int mon_errors = 0;
	int bitcnt = 0;
	logic [15:0] mosi_word;
	logic [15:0] exp_word;
	logic [15:0] exp_word_fall;	// the same word as at CS fall: a write
								// landing during the transfer may give either
	int exp_addr;
	bit cfg_at_csb;
	bit armed = 0;			// a transfer began after reset release
	int csb_rst_errors = 0;

	always @(negedge dut.rst_n_clk) begin
		xfer_idx = 0;
		bitcnt = 0;
		armed = 0;
	end

	always @(negedge spi1_csb)
		if (dut.rst_n_clk === 1'b1) begin
			armed = 1;
			bitcnt = 0;
			mosi_word = 16'd0;
			cfg_at_csb = dut.cfg_done;
			exp_word_fall = dut.regbank0.ram[(xfer_idx < CFG_N) ? xfer_idx
			                               : SAMP_BASE + ((xfer_idx - CFG_N) % SAMP_N)];
		end

	// CS must be deasserted (high) whenever the SPI master is in reset:
	// the RHD2164s must not see a transfer that the master never runs.
	// Checked once the reset has settled and again on every CS change.
	always @(spi1_csb or dut.rst_n_clk) begin
		#1;
		if ((dut.rst_n_clk === 1'b0) && (spi1_csb !== 1'b1) && ($time > 100000)) begin
			csb_rst_errors++;
			if (csb_rst_errors <= 3)
				$error("spi1_csb = %b while rst_n_clk is low (t=%0t)", spi1_csb, $time);
		end
	end

	always @(posedge spi1_sck)
		if (!spi1_csb) begin
			mosi_word = {mosi_word[14:0], spi1_mosi};
			bitcnt++;
		end

	always @(posedge spi1_csb)
		if ((dut.rst_n_clk === 1'b1) && armed) begin
			armed = 0;
			if (bitcnt != 16) begin
				mon_errors++;
				$error("SPI1 transfer %0d: %0d bits, expected 16 (t=%0t)", xfer_idx, bitcnt, $time);
			end
			exp_addr = (xfer_idx < CFG_N) ? xfer_idx
			                              : SAMP_BASE + ((xfer_idx - CFG_N) % SAMP_N);
			exp_word = dut.regbank0.ram[exp_addr];
			if ((mosi_word !== exp_word) && (mosi_word !== exp_word_fall)) begin
				mon_errors++;
				$error("SPI1 transfer %0d: MOSI %04h, expected ram[%0d] = %04h (t=%0t)",
				       xfer_idx, mosi_word, exp_addr, exp_word, $time);
			end
			if (cfg_at_csb !== (xfer_idx >= CFG_N)) begin
				mon_errors++;
				$error("SPI1 transfer %0d: cfg_done = %b at CS fall, expected %b (t=%0t)",
				       xfer_idx, cfg_at_csb, (xfer_idx >= CFG_N), $time);
			end
			xfer_idx++;
			xfer_total++;
		end

	always @(posedge dut.cfg_done) begin
		cfg_rises++;
		if (xfer_idx != CFG_N) begin
			mon_errors++;
			$error("cfg_done rose after %0d transfers, expected %0d (t=%0t)", xfer_idx, CFG_N, $time);
		end
	end

	always @(negedge dut.cfg_done)
		if (dut.rst_n_clk === 1'b1) begin
			mon_errors++;
			$error("cfg_done fell with rst_n_clk high (t=%0t)", $time);
		end

	always @(dut.rst_n_clk or dut.cfg_done) begin
		#1;	// let the asynchronous reset settle
		if ((dut.rst_n_clk === 1'b0) && (dut.cfg_done !== 1'b0)) begin
			mon_errors++;
			$error("cfg_done = %b while rst_n_clk is low (t=%0t)", dut.cfg_done, $time);
		end
	end

	// SPI1 receive: what the master captured equals what the models sent,
	// for every transfer. Compared when the controller has seen the
	// transfer end (op0d / op1d), against the words the models put on
	// MISO for that transfer (latched at CS fall, before the models move
	// on to the next one at CS rise).
	localparam [3:0] ST_OP0D = 4'd4, ST_OP1D = 4'd8;
	logic [15:0] sent_a0, sent_b0, sent_a1, sent_b1;
	int rx_checked = 0, rx_errors = 0;
	bit armed_rx = 0;

	always @(negedge spi1_csb) begin
		sent_a0 = chip0.out_a;  sent_b0 = chip0.out_b;
		sent_a1 = chip1.out_a;  sent_b1 = chip1.out_b;
	end

	always @(posedge dut.clk)
		if ((dut.rst_n_clk === 1'b1) && armed_rx &&
		    ((dut.controller0.current_state == ST_OP0D) ||
		     (dut.controller0.current_state == ST_OP1D))) begin
			armed_rx = 0;
			rx_checked++;
			if ({dut.spi1_data_rx_a0, dut.spi1_data_rx_b0, dut.spi1_data_rx_a1, dut.spi1_data_rx_b1}
			    !== {sent_a0, sent_b0, sent_a1, sent_b1}) begin
				rx_errors++;
				if (rx_errors <= 5)
					$error("SPI1 rx: got %04h %04h %04h %04h, sent %04h %04h %04h %04h (t=%0t)",
					       dut.spi1_data_rx_a0, dut.spi1_data_rx_b0, dut.spi1_data_rx_a1, dut.spi1_data_rx_b1,
					       sent_a0, sent_b0, sent_a1, sent_b1, $time);
			end
		end

	always @(negedge spi1_csb) if (dut.rst_n_clk === 1'b1) armed_rx = 1;


	// ------------------------------------------------------------------
	// K8: capture buffer and frame builder, checked on the wire bytes
	// (lvds-tunnel.md §4.1, §4.3, §4.3.1, §4.4a, §4.6, §4.7)
	// ------------------------------------------------------------------
	// Reference: the answer to every sampling command, as the models
	// computed it (chip0/chip1 hold_*, just after CS rises), stored by
	// sampling frame f and entry e: j = sampling transfers since rst_n_clk
	// rose, f = j / 33, e = j mod 33. The samples name their own channel
	// and frame (rhd2164_bfm), so a slot off by one or a frame late cannot
	// match.
	//
	// Receiver: decodes tun_data_8b, the byte oddrx4_1 serialises, on
	// wclk. Between frames every byte must be 0x00; a frame starts at
	// the first 0xA5, and its length comes from payload_len (256, 22 or
	// 18; anything else is an error). Each SAMPLE must carry
	//   preamble A5 5A A5 5A, frame_type 0x01,
	//   flags = {5'b0, CONFIG_DIRTY, TEST_PATTERN, STREAM_ENABLE} (§4.5),
	//     as tun_aux snapshots them at the announcement,
	//   frame_index f: 0 for the first frame after every reset, then +1,
	//   payload_len 256,
	//   payload slot s, half h = answer to entry s, little-endian
	//     (h = 0..3: chip 0 A, chip 0 B, chip 1 A, chip 1 B),
	//   CRC-32 over bytes 4..267, computed here (k8_crc_step),
	// and consecutive frames start 379 or 380 wclk apart (379.5 byte
	// slots per sampling period, §6.5): one SAMPLE per period.
	//
	// CONFIG_DIRTY's reference is built from the models, not the DUT:
	// a WRITE(R, D), R <= 21, that a chip executes with a D different
	// from the last D the chips took for R sets it; every rst_n_clk reset
	// sets it (the walk that follows rewrites the chips, §4.4); each
	// announcement takes it and clears it. A WRITE in transfers 0/1 of
	// frame f + 1 executes before f's announcement, so counts with f.
	//
	// Auxiliary frames (§4.4, §4.4a, §4.7, §4.8). Which one follows
	// SAMPLE f comes from a reference of the cadence, run at every
	// announcement: CONFIG if CONFIG_DIRTY or 1000 frames since the last
	// one; else STATUS if 300 frames since the last one or one pending;
	// a STATUS due with a CONFIG waits one frame; both start due after
	// every reset. T10 checks the result against the schedule the spec
	// gives, independently of this reference. Each auxiliary frame must
	// start on the wclk after its SAMPLE's last CRC byte, carry the
	// SAMPLE's flags and frame_index, its own type and length, a good
	// CRC, and:
	//   CONFIG: the 22 RAM register values the models last took, as at
	//     the announcement (the same snapshot instant as the flags);
	//   STATUS: design_id = {KUNTUR_VARIANT, KUNTUR_RELEASE}
	//     (kuntur_build.vh), valid 0x06, reserved 0, alt_cmd and
	//     alt_answer[0..3] = entry 32's command and its answer (chip 0 A
	//     first), fifo0_overflow_samples 0. All little-endian.
	// A SAMPLE whose expected auxiliary frame never comes, or an
	// auxiliary frame nobody expected, is an error. A reset may cut one.
	//
	// alt_cmd / alt_answer are also checked at the capture buffer's
	// announcement, as is its placement (only at the end of transfer 1,
	// never before frame 0 is whole).
	//
	// In this tb wclk is clk / 4 (kuntur_prims_sim.v), so the clk -> wclk
	// crossing is checked for logic, not with a drifting phase. The
	// serial line itself (ODDRX4 bit order) is not decoded here.

	localparam int K8_RING = 8;
	localparam int K8_FRAME_B = 272;
	localparam int KV = `KUNTUR_VARIANT;	// +define+KUNTUR_VARIANT=4'd1/2/3

	// clock_gen's PLL (#6): pll_28k for variant 1, pll_25k for 2 and 3.
	// g_pll is clock_gen's variant generate, both branches (S6). Only the
	// elaborated branch is resolved.
	// The model's phase_pos and n_step_viol are read back the same way (T12).
	logic pll_unlock = 1'b0;
	int model_pos, model_viol;
	if (KV == 1) begin : g_pll
		always @(pll_unlock) dut.clock_gen_0.g_pll.pll_28k_0.core.unlock = pll_unlock;
		always @* model_pos  = dut.clock_gen_0.g_pll.pll_28k_0.core.phase_pos;
		always @* model_viol = dut.clock_gen_0.g_pll.pll_28k_0.core.n_step_viol;
	end
	else begin : g_pll
		always @(pll_unlock) dut.clock_gen_0.g_pll.pll_25k_0.core.unlock = pll_unlock;
		always @* model_pos  = dut.clock_gen_0.g_pll.pll_25k_0.core.phase_pos;
		always @* model_viol = dut.clock_gen_0.g_pll.pll_25k_0.core.n_step_viol;
	end
	localparam int SPT_EXP = (KV == 1) ? 296 : 336;	// spi1-mosi-eye-scan.md §3.2

	// #6a strobes (T12): phase_step_wr and chk_clear, counted and timed in
	// clk cycles; every PHASESTEP falling edge out of reset.
	int step_wr_n = 0, step_wr_len = 0, step_wr_maxlen = 0;
	int chk_clr_n = 0, chk_clr_len = 0, chk_clr_maxlen = 0;
	int phasestep_falls = 0;
	always @(posedge dut.clk) begin
		if (dut.phase_step_wr === 1'b1) begin
			if (step_wr_len == 0) step_wr_n++;
			step_wr_len++;
			if (step_wr_len > step_wr_maxlen) step_wr_maxlen = step_wr_len;
		end
		else step_wr_len = 0;
		if (dut.chk_clear === 1'b1) begin
			if (chk_clr_len == 0) chk_clr_n++;
			chk_clr_len++;
			if (chk_clr_len > chk_clr_maxlen) chk_clr_maxlen = chk_clr_len;
		end
		else chk_clr_len = 0;
	end
	always @(negedge dut.phasestep) if (dut.rst_n_clk === 1'b1) phasestep_falls++;

	// One PHASE_STEP_CMD write through SPI0, then: exactly one strobe,
	// exactly N PHASESTEP pulses, and the stepper's PHASE_POS and the
	// model's phase both at exp_pos (updated here: + N, or - N with [15]).
	logic [15:0] last_step_cmd, rd;

	// #6a step 4 (T13): the MOSI launch path, spi1-mosi-eye-scan.md §3.1.
	// Every edge on the spi1_mosi pin must be an edge of today's MOSI
	// (dut.spi1_mosi_m0) with the same new value, Delta ns later. Pin
	// edges are judged 2T after they happen, so an early source
	// (Delta < 0) finds its m0 edge already recorded.
	localparam real TCLK    = 20.0;	// this tb's clk (always #10)
	localparam real CLKOS_D = 5.0;	// pll_sim CLKOS_DELAY default
	bit  meas_on = 0;
	real meas_dexp;
	int  meas_match, meas_miss, meas_m0_edges;
	realtime m0_t[$];
	logic    m0_v[$];
	always @(dut.spi1_mosi_m0) if (meas_on) begin
		meas_m0_edges++;
		m0_t.push_back($realtime);
		m0_v.push_back(dut.spi1_mosi_m0);
		if (m0_t.size() > 16) begin void'(m0_t.pop_front()); void'(m0_v.pop_front()); end
	end
	always @(spi1_mosi) if (meas_on)
		fork
			automatic realtime tp = $realtime;
			automatic logic    vp = spi1_mosi;
			begin
				bit found;
				#(2.0 * TCLK);
				found = 0;
				foreach (m0_t[i])
					if ((m0_v[i] === vp) && ((tp - m0_t[i] - meas_dexp) < 0.05) &&
					    ((tp - m0_t[i] - meas_dexp) > -0.05))
						found = 1;
				if (found) meas_match++;
				else begin
					meas_miss++;
					if (meas_miss <= 3)
						$error("T13 MOSI edge at %0t (%b) is no m0 edge + %0.3f ns", tp, vp, meas_dexp);
				end
			end
		join_none

	// offset o of each source, in ns (§3.1 table)
	function automatic real src_offset(int s);
		case (s)
			0: return -TCLK;
			1: return -TCLK / 2.0;
			2: return 0.0;
			default: return TCLK / 2.0;
		endcase
	endfunction

	// x = (clk_tx delay - o) mod T, the clk_tx edge's distance from the
	// source's last change; Delta = o + x
	function automatic real src_x(int s, int pos);
		real d, x;
		d = CLKOS_D + pos * TCLK / SPT_EXP;
		x = d - src_offset(s);
		while (x >= TCLK) x = x - TCLK;
		while (x < 0.0)   x = x + TCLK;
		return x;
	endfunction

	// Bypass first, then step (never sweep a source through forbidden
	// phases), then select the source.
	task automatic step_to(int pos);
		int n;
		n = (pos - dut.phase_pos + SPT_EXP) % SPT_EXP;
		if (n != 0) begin
			reg_write16(RB_STEP_CMD, 16'(n));
			repeat (8) @(posedge dut.clk);
			wait (dut.scan0.stepper_busy === 1'b0);
			repeat (2) @(posedge dut.clk);
		end
	endtask

	// One scan point: source s at phase pos. Checks Delta on every edge
	// over one sampling frame (33 transfers), and that the SPI1 monitor (MOSI words as sampled
	// on SCK rising) and the rx check (what the models answered) see no
	// error: the chips decode every command.
	task automatic mosi_point(int s, int pos);
		real x, dexp;
		int mon0, rx0, xf0;
		string tag;
		x = src_x(s, pos);
		dexp = (s == 4) ? 0.0 : src_offset(s) + x;
		tag = $sformatf("13 mosi_sel %0d, PHASE_POS %0d (Delta %0.2f ns)", s, pos, dexp);
		if ((s != 4) && ((x < TCLK / 4.0) || (x > 3.0 * TCLK / 4.0) ||
		                 (dexp <= -TCLK + 1.0) || (dexp >= TCLK - 1.0)))
			$fatal(1, "T13 tb bug: %0s is not an allowed, decodable point (x %0.2f)", tag, x);
		reg_write16(RB_SCAN_CTRL, 16'h0004);
		step_to(pos);
		reg_write16(RB_SCAN_CTRL, 16'(s));
		xf0 = xfer_total;
		wait (xfer_total >= xf0 + 2);		// whole transfers on the new source
		mon0 = mon_errors;  rx0 = rx_errors;
		m0_t.delete();  m0_v.delete();
		meas_dexp = dexp;  meas_match = 0;  meas_miss = 0;  meas_m0_edges = 0;
		xf0 = xfer_total;
		meas_on = 1;
		wait (xfer_total >= xf0 + SAMP_N);	// one whole sampling frame
		meas_on = 0;
		#(3.0 * TCLK);						// let the last judgements finish
		checks++;
		assert ((meas_miss == 0) && (meas_match >= 10) &&
		        (meas_match >= meas_m0_edges - 2) && (meas_match <= meas_m0_edges + 2))
			$display("  pass  %0s: %0d edges, all m0 + Delta", tag, meas_match);
		else begin
			errors++;
			$error("%0s: %0d edges at m0 + Delta, %0d not, %0d m0 edges", tag, meas_match, meas_miss, meas_m0_edges);
		end
		check({tag, ": SPI1 monitor errors"}, 16'(mon_errors - mon0), 16'd0);
		check({tag, ": SPI1 rx errors"},      16'(rx_errors - rx0),   16'd0);
	endtask
	task automatic step_and_check(string name, logic [15:0] cmd, inout int exp_pos);
		int n_wr, n_fall, n;
		n = cmd[9:0];
		n_wr = step_wr_n;
		n_fall = phasestep_falls;
		reg_write16(RB_STEP_CMD, cmd);
		last_step_cmd = cmd;
		repeat (8) @(posedge dut.clk);		// the commit lands a few clk after the transfer
		wait (dut.scan0.stepper_busy === 1'b0);
		repeat (2) @(posedge dut.clk);
		exp_pos = cmd[15] ? (exp_pos - n + SPT_EXP) % SPT_EXP : (exp_pos + n) % SPT_EXP;
		check({name, ": one strobe"},        16'(step_wr_n - n_wr),         16'd1);
		check({name, ": N PHASESTEP pulses"}, 16'(phasestep_falls - n_fall), 16'(n));
		check({name, ": stepper PHASE_POS"},  {7'd0, dut.phase_pos},         16'(exp_pos));
		check({name, ": PLL model phase"},    16'(model_pos),                16'(exp_pos));
		// as the MCU sees them (read mux, §3.4)
		reg_read16(RB_PHASE_POS, rd);	check({name, ": 240 PHASE_POS over SPI0"},  rd, 16'(exp_pos));
		reg_read16(RB_PHASE_INFO, rd);	check({name, ": 241 PHASE_INFO over SPI0"}, rd, 16'(SPT_EXP));
	endtask

	// T14 reference checker (spi1-mosi-eye-scan.md §3.3), built from the
	// pins and the models, not from the RTL: the answers sent in transfer
	// t (sent_*, latched at its CS fall) belong to the MOSI word of
	// transfer t - 2. Counts sampling transfers only (cfg_done at CS
	// fall), with CHK_EN as it is at CS rise.
	function automatic bit ref_decode(logic [15:0] c, output logic [15:0] e);
		e = 16'd0;
		if (c[15:14] == 2'b10) begin
			e = {8'hFF, c[7:0]};
			return 1'b1;
		end
		if (c[15:14] == 2'b11)
			case (c[13:8])
				6'd40: begin e = 16'h0049; return 1'b1; end
				6'd41: begin e = 16'h004E; return 1'b1; end
				6'd42: begin e = 16'h0054; return 1'b1; end
				6'd43: begin e = 16'h0041; return 1'b1; end
				6'd44: begin e = 16'h004E; return 1'b1; end
				6'd48: begin e = 16'h0052; return 1'b1; end
				6'd49: begin e = 16'h0048; return 1'b1; end
				6'd50: begin e = 16'h0044; return 1'b1; end
				default: ;
			endcase
		return 1'b0;
	endfunction

	logic [15:0] ref_h0 = 16'd0, ref_h1 = 16'd0;	// MOSI words of transfers t-1, t-2
	logic [15:0] ref_cnt = 16'd0;
	logic [15:0] ref_err [4] = '{default: 16'd0};
	logic [15:0] ref_fans = 16'd0, ref_finfo = 16'd0, ref_fexp = 16'd0;

	task automatic ref_clear();
		ref_cnt = 16'd0;
		ref_err = '{default: 16'd0};
		ref_fans = 16'd0;  ref_finfo = 16'd0;  ref_fexp = 16'd0;
	endtask

	always @(negedge dut.rst_n_clk) begin
		ref_h0 = 16'd0;
		ref_h1 = 16'd0;
		ref_clear();
	end
	always @(posedge dut.clk) if (dut.chk_clear === 1'b1) ref_clear();

	always @(posedge spi1_csb)
		if (dut.rst_n_clk === 1'b1) begin : ref_chk
			logic [15:0] e;
			logic [15:0] ans [4];
			#1;
			ans = '{sent_a0, sent_b0, sent_a1, sent_b1};
			if (cfg_at_csb && (dut.mosi_scan_ctrl[3] === 1'b1) && ref_decode(ref_h1, e)) begin
				if (ref_cnt != 16'hFFFF) ref_cnt++;
				for (int i = 0; i < 4; i++)
					if (ans[i] !== e) begin
						if (ref_err[i] != 16'hFFFF) ref_err[i]++;
						if (!ref_finfo[15]) begin
							ref_fans  = ans[i];
							ref_finfo = {1'b1, 13'd0, 2'(i)};
							ref_fexp  = e;
						end
					end
			end
			ref_h1 = ref_h0;
			ref_h0 = mosi_word;
		end

	// The RTL against the reference. chk_count may differ by one: the
	// reference takes CHK_EN at CS rise, the RTL a few clk after
	// xfer_done, so a CHK_EN or CHK_CLEAR edge in between moves one
	// transfer. Errors are injected away from those edges and must match
	// exactly.
	task automatic chk_vs_ref(string tag);
		checks++;
		assert ((int'(dut.chk_count) >= int'(ref_cnt) - 1) && (int'(dut.chk_count) <= int'(ref_cnt) + 1))
			$display("  pass  %0s: CHK_COUNT %0d (reference %0d)", tag, dut.chk_count, ref_cnt);
		else begin
			errors++;
			$error("%0s: CHK_COUNT %0d, reference %0d", tag, dut.chk_count, ref_cnt);
		end
		check({tag, ": ERR_C0A = reference"}, dut.err_c0a, ref_err[0]);
		check({tag, ": ERR_C0B = reference"}, dut.err_c0b, ref_err[1]);
		check({tag, ": ERR_C1A = reference"}, dut.err_c1a, ref_err[2]);
		check({tag, ": ERR_C1B = reference"}, dut.err_c1b, ref_err[3]);
		check({tag, ": FIRST_ERR_ANS = reference"},  dut.first_err_ans,  ref_fans);
		check({tag, ": FIRST_ERR_INFO = reference"}, dut.first_err_info, ref_finfo);
		check({tag, ": FIRST_ERR_EXP = reference"},  dut.first_err_exp,  ref_fexp);
	endtask

	// Words 242-249 as the MCU reads them, against the live outputs
	// (CHK_EN must be 0, so they hold during the reads).
	task automatic chk_read_back(string tag);
		logic [15:0] live [8];
		logic [15:0] w;
		live = '{dut.chk_count, dut.err_c0a, dut.err_c0b, dut.err_c1a, dut.err_c1b,
		         dut.first_err_ans, dut.first_err_info, dut.first_err_exp};
		for (int i = 0; i < 8; i++) begin
			reg_read16(8'(242 + i), w);
			check($sformatf("%0s: word %0d over SPI0", tag, 242 + i), w, live[i]);
		end
	endtask

	task automatic wait_xfers(int n);
		int x0;
		x0 = xfer_total;
		wait (xfer_total >= x0 + n);
	endtask

	// Error injection: the next n WRITE answers of one chip's half
	task automatic inject(int chip, bit half_b, logic [15:0] x, int n);
		if (chip == 0) begin
			if (half_b) chip0.inj_xor_b = x; else chip0.inj_xor_a = x;
			chip0.inj_n = n;
		end
		else begin
			if (half_b) chip1.inj_xor_b = x; else chip1.inj_xor_a = x;
			chip1.inj_n = n;
		end
	endtask
	task automatic inject_off();
		chip0.inj_n = 0;  chip0.inj_xor_a = 16'd0;  chip0.inj_xor_b = 16'd0;  chip0.inj_seen = 1'b0;
		chip1.inj_n = 0;  chip1.inj_xor_a = 16'd0;  chip1.inj_xor_b = 16'd0;  chip1.inj_seen = 1'b0;
	endtask
	logic [63:0] k8_ans [K8_RING][SAMP_N];	// reference answers, by frame mod 8
	logic [15:0] k8_cmd [K8_RING];			// entry 32's command, by frame mod 8
	int          k8_row_frame [K8_RING];	// frame each ring row holds
	int  k8_j = 0;				// sampling transfers since rst_n_clk rose
	bit  k8_samp = 0;			// this transfer is a sampling one
	int  k8_resets = 0;
	int  k8_errors = 0, k8_frames = 0, k8_words = 0, k8_early = 0;
	int  k8_announced = 0;		// frame_rdy toggles with rst_n_clk high
	int  k8_next_index = 0;
	int  k8_zero = 0;			// frames received with frame_index 0
	int  k8_read40 = 0;			// frames whose slot 31 holds READ(40)'s answer
	int  k8_crc_bad = 0, k8_idle_bad = 0, k8_gap_bad = 0, k8_gaps = 0, k8_cut = 0;
	logic [7:0] k8_buf [K8_FRAME_B];
	int  k8_n = 0;				// bytes of the frame in progress; 0 = between frames
	longint k8_wclk = 0;		// wclk cycles, whole run
	longint k8_start = 0, k8_last_start = -1;
	int  k8_last_fi = -1;
	logic [7:0] k8_shadow [22];		// last D the chips took, per RAM register
	bit  k8_dirty = 1;				// CONFIG_DIRTY reference
	logic [7:0] k8_flags [K8_RING];	// expected flags, by frame mod 8
	int  k8_dirty_frames = 0;		// frames received with CONFIG_DIRTY
	int  k8_xfers_seen = 0;
	// auxiliary frames
	logic [7:0] k8_cfg_ref [K8_RING][22];	// expected CONFIG payload, by frame mod 8
	int  k8_aux_exp [K8_RING];		// 0 none, 2 CONFIG, 3 STATUS, by frame mod 8
	int  k8_cfg_cnt = 999, k8_st_cnt = 299;	// cadence reference, due after reset
	bit  k8_st_pend = 0;
	int  k8_len = K8_FRAME_B;		// length of the frame in progress
	int  k8_aux_pend = 0;			// auxiliary frame expected after the last SAMPLE
	int  k8_smp_fi = -1;			// that SAMPLE's frame_index, flags, last byte
	logic [7:0] k8_smp_flags;
	longint k8_smp_end = -1;
	int  k8_n_config = 0, k8_n_status = 0, k8_aux_bad = 0;
	int  k8_aux_log [$];			// {type, frame_index} since the last reset

	function automatic logic [31:0] k8_crc_step(logic [31:0] c, logic [7:0] b);
		c ^= {24'd0, b};
		for (int i = 0; i < 8; i++)
			c = c[0] ? ((c >> 1) ^ 32'hEDB88320) : (c >> 1);
		return c;
	endfunction

	always @(negedge dut.rst_n_clk) begin
		k8_resets++;
		k8_dirty = 1;
		k8_cfg_cnt = 999;
		k8_st_cnt = 299;
		k8_st_pend = 0;
		k8_aux_pend = 0;		// one a reset cuts is not missing
		k8_aux_log.delete();
		k8_j = 0;
		k8_next_index = 0;
		if (k8_n != 0)
			k8_cut++;			// a frame cut short by the reset
		k8_n = 0;
		k8_last_start = -1;
	end

	always @(negedge spi1_csb)
		k8_samp = (dut.rst_n_clk === 1'b1) && (dut.cfg_done === 1'b1);

	always @(posedge spi1_csb)
		if ((dut.rst_n_clk === 1'b1) && k8_samp) begin : k8_record
			int f, e, r;
			#1;	// the models execute the command at this edge
			f = k8_j / SAMP_N;
			e = k8_j % SAMP_N;
			r = f % K8_RING;
			if (e == 0)
				k8_row_frame[r] = f;
			k8_ans[r][e] = {chip0.hold_a, chip0.hold_b, chip1.hold_a, chip1.hold_b};
			if (e == SAMP_N - 1)
				k8_cmd[r] = chip0.last_cmd;
			k8_j++;
		end

	// CONFIG_DIRTY reference: every WRITE the models execute, walk or
	// sampling
	always @(posedge spi1_csb) begin : k8_writes
		logic [15:0] c;
		#1;
		if (chip0.n_xfers != k8_xfers_seen) begin
			k8_xfers_seen = chip0.n_xfers;
			c = chip0.last_cmd;
			if ((c[15:14] == 2'b10) && (c[13:8] <= 6'd21)) begin
				if (k8_shadow[c[13:8]] !== c[7:0])
					k8_dirty = 1;
				k8_shadow[c[13:8]] = c[7:0];
			end
		end
	end

	// The announcement: only at the end of transfer 1, never before frame
	// 0 is whole (k8_j = 33 (f + 1) + 2 at the toggle), and the alt
	// snapshot belongs to the frame announced.
	always @(dut.capture_buffer_0.frame_rdy)
		if (dut.rst_n_clk === 1'b1) begin : k8_announce
			int fi, r, aux;
			logic [7:0] flags;
			logic [7:0] snap [22];
			bit cfg, st, st_due;
			k8_announced++;
			if ((k8_j < SAMP_N + 2) || ((k8_j % SAMP_N) != 2)) begin
				k8_early++;
				$error("K8: frame_rdy toggled after %0d sampling transfers (t=%0t)", k8_j, $time);
			end
			flags = {5'b0, k8_dirty, dut.data_source_sel == 2'd1, dut.stream_enable};
			snap = k8_shadow;
			// cadence reference
			cfg = k8_dirty || (k8_cfg_cnt == 999);
			k8_cfg_cnt = cfg ? 0 : k8_cfg_cnt + 1;
			st_due = k8_st_pend || (k8_st_cnt == 299);
			st = st_due && !cfg;
			k8_st_pend = st_due && !st;
			if (st)
				k8_st_cnt = 0;
			else if (!st_due)
				k8_st_cnt++;
			aux = cfg ? 2 : (st ? 3 : 0);
			k8_dirty = 0;
			#1;
			fi = dut.capture_buffer_0.frame_index;
			r = fi % K8_RING;
			k8_flags[r] = flags;
			k8_cfg_ref[r] = snap;
			k8_aux_exp[r] = aux;
			if (k8_row_frame[r] != fi) begin
				k8_errors++;
				$error("K8: no reference for announced frame %0d (t=%0t)", fi, $time);
			end
			else begin
				if (dut.capture_buffer_0.alt_answer !== k8_ans[r][SAMP_N - 1]) begin
					k8_errors++;
					$error("K8: frame %0d alt_answer %016h, expected %016h",
					       fi, dut.capture_buffer_0.alt_answer, k8_ans[r][SAMP_N - 1]);
				end
				if (dut.capture_buffer_0.alt_cmd !== k8_cmd[r]) begin
					k8_errors++;
					$error("K8: frame %0d alt_cmd %04h, expected %04h",
					       fi, dut.capture_buffer_0.alt_cmd, k8_cmd[r]);
				end
			end
		end

	// The receiver (variant 2 only: the other variants send no frames)
	always @(posedge dut.wclk) begin : k8_rx
		logic [7:0] b;
		k8_wclk++;
		b = dut.tun_data_8b;
		if ((KV == 2) && (dut.blk_rstb === 1'b1) && (dut.rst_n_clk === 1'b1)) begin
			if (k8_n == 0) begin
				if (b === 8'hA5) begin
					k8_buf[0] = b;
					k8_n = 1;
					k8_start = k8_wclk;
				end
				else if (b !== 8'h00) begin
					k8_idle_bad++;
					if (k8_idle_bad <= 5)
						$error("K8: byte %02h between frames, expected 00 (t=%0t)", b, $time);
				end
			end
			else begin
				k8_buf[k8_n] = b;
				k8_n++;
				if (k8_n == 12) begin
					k8_len = 16 + {k8_buf[11], k8_buf[10]};
					if ((k8_len != K8_FRAME_B) && (k8_len != 38) && (k8_len != 34)) begin
						k8_errors++;
						$error("K8: frame type %02h with payload_len %0d (t=%0t)", k8_buf[4], k8_len - 16, $time);
						k8_n = 0;
					end
				end
				if ((k8_n > 12) && (k8_n == k8_len)) begin
					if (k8_buf[4] === 8'h01)
						k8_check_frame();
					else
						k8_check_aux();
					k8_n = 0;
				end
			end
		end
	end

	// K4.3: the other variants on the wire, from the same byte as the
	// receiver.
	//   3 (bench): the K4.2 link test pattern, 8-byte blocks
	//     A5 5A A5 5A cnt[7:0] cnt[15:8] 01 80, no gaps, cnt 0 for the first
	//     block after every reset and +1 per block. block_gen's register
	//     resets to 00, so one 00 may come first.
	//   1 (28k recording, tunnel unused): 00 only.
	int  kp_errors = 0, kp_blocks = 0, kp_bytes = 0, kp_segs = 0;
	int  kp_k = -1;				// byte of the pattern expected next; -1 = after reset
	int  kp_seg_blocks = 0;		// blocks since the last reset

	function automatic logic [7:0] kp_byte(int k);
		logic [15:0] c = 16'(k / 8);
		case (k % 8)
			0, 2: return 8'hA5;
			1, 3: return 8'h5A;
			4:    return c[7:0];
			5:    return c[15:8];
			6:    return 8'h01;
			default: return 8'h80;
		endcase
	endfunction

	always @(posedge dut.wclk) begin : kp_rx
		logic [7:0] b;
		b = dut.tun_data_8b;
		if ((dut.blk_rstb !== 1'b1) || (dut.rst_n_clk !== 1'b1))
			kp_k = -1;
		else if (KV == 1) begin
			kp_bytes++;
			if (b !== 8'h00) begin
				kp_errors++;
				if (kp_errors <= 5)
					$error("K4.3: variant 1 sent %02h, expected 00 (t=%0t)", b, $time);
			end
		end
		else if (KV == 3) begin
			kp_bytes++;
			if ((kp_k == -1) && (b === 8'h00))
				kp_k = 0;				// block_gen's reset value, once
			else begin
				if (kp_k == -1)
					kp_k = 0;
				if (kp_k == 0) begin
					kp_segs++;
					kp_seg_blocks = 0;
				end
				if (b !== kp_byte(kp_k)) begin
					kp_errors++;
					if (kp_errors <= 5)
						$error("K4.3: pattern byte %0d (block %0d) is %02h, expected %02h (t=%0t)",
						       kp_k % 8, kp_k / 8, b, kp_byte(kp_k), $time);
				end
				kp_k++;
				if (kp_k % 8 == 0) begin
					kp_blocks++;
					kp_seg_blocks++;
				end
			end
		end
	end

	task automatic k8_check_frame();
		int fi, r, s, h;
		logic [31:0] c, crc_rx;
		logic [15:0] got, exp;

		k8_frames++;
		fi = {k8_buf[9], k8_buf[8], k8_buf[7], k8_buf[6]};

		if (k8_aux_pend != 0) begin
			k8_aux_bad++;
			$error("K8: no %s after SAMPLE %0d (t=%0t)", (k8_aux_pend == 2) ? "CONFIG" : "STATUS", k8_smp_fi, $time);
		end
		k8_aux_pend = 0;
		k8_smp_fi = fi;
		k8_smp_flags = k8_buf[5];
		k8_smp_end = k8_wclk;

		if ({k8_buf[0], k8_buf[1], k8_buf[2], k8_buf[3]} !== 32'hA55AA55A) begin
			k8_errors++;
			$error("K8: preamble %02h %02h %02h %02h (t=%0t)", k8_buf[0], k8_buf[1], k8_buf[2], k8_buf[3], $time);
		end
		if ((k8_buf[4] !== 8'h01) || ({k8_buf[11], k8_buf[10]} !== 16'd256)) begin
			k8_errors++;
			$error("K8: frame %0d header: type %02h payload_len %04h (t=%0t)",
			       fi, k8_buf[4], {k8_buf[11], k8_buf[10]}, $time);
		end

		// frame_index: 0 after every reset, then +1
		if (fi == 0)
			k8_zero++;
		if (fi != k8_next_index) begin
			k8_errors++;
			$error("K8: frame_index %0d, expected %0d (t=%0t)", fi, k8_next_index, $time);
		end
		k8_next_index = fi + 1;

		// one SAMPLE per sampling period
		if ((k8_last_start >= 0) && (fi == k8_last_fi + 1)) begin
			k8_gaps++;
			if (((k8_start - k8_last_start) != 379) && ((k8_start - k8_last_start) != 380)) begin
				k8_gap_bad++;
				$error("K8: frame %0d started %0d wclk after the previous one, expected 379 or 380",
				       fi, k8_start - k8_last_start);
			end
		end
		k8_last_start = k8_start;
		k8_last_fi = fi;

		// CRC-32 over header and payload, sent inverted, little-endian
		c = 32'hFFFFFFFF;
		for (int i = 4; i < 268; i++)
			c = k8_crc_step(c, k8_buf[i]);
		crc_rx = {k8_buf[271], k8_buf[270], k8_buf[269], k8_buf[268]};
		if (crc_rx !== ~c) begin
			k8_crc_bad++;
			$error("K8: frame %0d CRC %08h, expected %08h (t=%0t)", fi, crc_rx, ~c, $time);
		end

		// payload against the models
		r = fi % K8_RING;
		if (k8_row_frame[r] != fi) begin
			k8_errors++;
			$error("K8: no reference for frame %0d (ring holds %0d) (t=%0t)", fi, k8_row_frame[r], $time);
			return;
		end
		k8_aux_pend = k8_aux_exp[r];
		if (k8_buf[5] !== k8_flags[r]) begin
			k8_errors++;
			$error("K8: frame %0d flags %02h, expected %02h (t=%0t)", fi, k8_buf[5], k8_flags[r], $time);
		end
		if (k8_buf[5][2] === 1'b1)
			k8_dirty_frames++;
		for (s = 0; s < 32; s++)
			for (h = 0; h < 4; h++) begin
				k8_words++;
				got = {k8_buf[13 + 8*s + 2*h], k8_buf[12 + 8*s + 2*h]};
				exp = k8_ans[r][s][63 - 16*h -: 16];
				if (got !== exp) begin
					k8_errors++;
					if (k8_errors <= 10)
						$error("K8: frame %0d slot %0d half %0d: %04h, expected %04h (t=%0t)",
						       fi, s, h, got, exp, $time);
				end
			end
		if ({k8_buf[260], k8_buf[262], k8_buf[264], k8_buf[266]} === {4{8'h49}} &&
		    {k8_buf[261], k8_buf[263], k8_buf[265], k8_buf[267]} === 32'd0)
			k8_read40++;
	endtask

	task automatic k8_check_aux();
		int fi, r, n, bad;
		logic [7:0] t;
		logic [31:0] c, crc_rx;
		logic [15:0] got, exp;

		bad = 0;
		t = k8_buf[4];
		n = k8_len - 16;
		fi = {k8_buf[9], k8_buf[8], k8_buf[7], k8_buf[6]};
		r = fi % K8_RING;

		if ({k8_buf[0], k8_buf[1], k8_buf[2], k8_buf[3]} !== 32'hA55AA55A) begin
			bad++;
			$error("K8: aux preamble %02h %02h %02h %02h (t=%0t)", k8_buf[0], k8_buf[1], k8_buf[2], k8_buf[3], $time);
		end
		if (!(((t === 8'h02) && (n == 22)) || ((t === 8'h03) && (n == 18)))) begin
			k8_aux_bad++;
			$error("K8: frame type %02h with payload_len %0d after SAMPLE %0d (t=%0t)", t, n, k8_smp_fi, $time);
			return;
		end
		if (t == 8'h02)
			k8_n_config++;
		else
			k8_n_status++;
		k8_aux_log.push_back({t, 24'(fi)});

		// which, where, and the SAMPLE's header
		if (k8_aux_pend != t) begin
			bad++;
			$error("K8: %s after SAMPLE %0d, expected %s (t=%0t)", (t == 2) ? "CONFIG" : "STATUS", k8_smp_fi,
			       (k8_aux_pend == 2) ? "CONFIG" : (k8_aux_pend == 3) ? "STATUS" : "none", $time);
		end
		k8_aux_pend = 0;
		if (k8_start != k8_smp_end + 1) begin
			bad++;
			$error("K8: type %0h frame starts %0d wclk after its SAMPLE's CRC, expected 1", t, k8_start - k8_smp_end);
		end
		if ((fi != k8_smp_fi) || (k8_buf[5] !== k8_smp_flags)) begin
			bad++;
			$error("K8: type %0h frame: frame_index %0d flags %02h, its SAMPLE's %0d %02h (t=%0t)",
			       t, fi, k8_buf[5], k8_smp_fi, k8_smp_flags, $time);
		end

		c = 32'hFFFFFFFF;
		for (int i = 4; i < 12 + n; i++)
			c = k8_crc_step(c, k8_buf[i]);
		crc_rx = {k8_buf[15 + n], k8_buf[14 + n], k8_buf[13 + n], k8_buf[12 + n]};
		if (crc_rx !== ~c) begin
			k8_crc_bad++;
			$error("K8: type %0h frame %0d CRC %08h, expected %08h (t=%0t)", t, fi, crc_rx, ~c, $time);
		end

		if (k8_row_frame[r] != fi) begin
			bad++;
			$error("K8: no reference for type %0h frame %0d (t=%0t)", t, fi, $time);
		end
		else if (t == 8'h02) begin
			for (int i = 0; i < 22; i++)
				if (k8_buf[12 + i] !== k8_cfg_ref[r][i]) begin
					bad++;
					$error("K8: CONFIG %0d reg %0d = %02h, expected %02h (t=%0t)", fi, i, k8_buf[12 + i], k8_cfg_ref[r][i], $time);
				end
		end
		else begin
			got = {k8_buf[13], k8_buf[12]};
			if (got !== 16'({`KUNTUR_VARIANT, `KUNTUR_RELEASE})) begin
				bad++;
				$error("K8: STATUS %0d design_id %04h, expected %04h", fi, got, 16'({`KUNTUR_VARIANT, `KUNTUR_RELEASE}));
			end
			if ((k8_buf[14] !== 8'h06) || (k8_buf[15] !== 8'h00)) begin
				bad++;
				$error("K8: STATUS %0d valid %02h reserved %02h, expected 06 00", fi, k8_buf[14], k8_buf[15]);
			end
			got = {k8_buf[17], k8_buf[16]};
			if (got !== k8_cmd[r]) begin
				bad++;
				$error("K8: STATUS %0d alt_cmd %04h, expected %04h", fi, got, k8_cmd[r]);
			end
			for (int h = 0; h < 4; h++) begin
				got = {k8_buf[19 + 2*h], k8_buf[18 + 2*h]};
				exp = k8_ans[r][SAMP_N - 1][63 - 16*h -: 16];
				if (got !== exp) begin
					bad++;
					$error("K8: STATUS %0d alt_answer[%0d] %04h, expected %04h", fi, h, got, exp);
				end
			end
			if ({k8_buf[29], k8_buf[28], k8_buf[27], k8_buf[26]} !== 32'd0) begin
				bad++;
				$error("K8: STATUS %0d fifo0_overflow_samples not 0", fi);
			end
		end
		k8_aux_bad += bad;
	endtask

	task automatic check_bit(string name, logic got, logic exp);
		checks++;
		assert (got === exp)
			$display("  pass  %0s: %b", name, got);
		else begin
			errors++;
			$error("%0s: got %b, expected %b", name, got, exp);
		end
	endtask

	// Wait for the sampling cycle to be running after a reset: rst_n_clk
	// released (1.024 ms hold-off of clkin after lock) and the config walk
	// done, plus a few sampling transfers.
	task automatic wait_sampling();
		wait (dut.rst_n_clk === 1'b1);
		wait (dut.cfg_done === 1'b1);
		wait (xfer_idx >= CFG_N + 4);
	endtask

	// Drop the PLL lock for `t` ns (R-K2: rst_n_clk falls at once).
	task automatic drop_lock(int t);
		pll_unlock = 1'b1;
		#(t);
		pll_unlock = 1'b0;
	endtask


	// ------------------------------------------------------------------
	// Test target: use MCU-SPI to set channels to be transmitted in SRAM
	// and then verify data.
	// ------------------------------------------------------------------

	logic [15:0] v, a1, b1, a2, b2, a3, b3;

	// test signals
	initial
	begin
		rstb = 1'b0;
		clk = 1'b0;
		clk2 = 1'b0;
		mcu_start = 1'b0;
		#40 rstb = 1'b1;

		$display("");
		$display("========================================================");
		$display(" SPI0 command protocol (A.1.1g)");
		$display("========================================================");

		// --------------------------------------------------------------
		banner("T0  R-K5: cfg_done at start-up");
		// Low through reset and the config walk; the monitor checks the
		// walk transfer by transfer.
		check_bit("cfg_done low at start", dut.cfg_done, 1'b0);
		wait (dut.rst_n_clk === 1'b1);
		check_bit("cfg_done low when rst_n_clk releases", dut.cfg_done, 1'b0);
		// Let the RHD config sequence finish and the sampling cycle start,
		// so the FIFO has entries by the time the POP tests run.
		wait_sampling();
		check_bit("cfg_done high once sampling runs", dut.cfg_done, 1'b1);
		checks++;
		assert (cfg_rises == 1) $display("  pass  cfg_done rose once: %0d", cfg_rises);
		else begin errors++; $error("cfg_done rose %0d times, expected 1", cfg_rises); end
		#40000;


		// --------------------------------------------------------------
		banner("T1  reset defaults readable (read path, no prior write)");
		// As originally built, a read returned ram[addr_reg] left over from
		// the last WRITE, so a word could only be read after writing it.
		// All five of these would have returned stale data. Direct
		// regression test for the 2026-08-10 fix.
		reg_read16(RB_SAMP0,     v);	check("ram[48] default",           v, 16'h0000);
		reg_read16(RB_SAMP1,     v);	check("ram[49] default",           v, 16'h0100);
		reg_read16(RB_CH_A,      v);	check("ram[196] ch_a default",     v, 16'h0083);
		reg_read16(RB_CH_B,      v);	check("ram[197] ch_b default",     v, 16'h0082);
		reg_read16(RB_STREAM_EN, v);	check("ram[228] stream_en default", v, 16'h0001);


		// --------------------------------------------------------------
		banner("T2  16-bit write path");
		// The point of A.1.1g. Both payloads have a non-zero high byte:
		//   95A5 -> staged_h never written => reads 00A5, FAIL
		//   3C5A -> staged_h stuck at 0x95 => reads 955A, FAIL
		// Targets sampling-table slots, where full 16-bit RHD command words
		// are the actual motivation.
		reg_write16(RB_SAMP0, VAL_A);
		reg_read16 (RB_SAMP0, v);	check("ram[48] <- 95A5 (high byte)", v, VAL_A);

		reg_write16(RB_SAMP1, VAL_B);
		reg_read16 (RB_SAMP1, v);	check("ram[49] <- 3C5A (high byte changes)", v, VAL_B);

		// staged_h must not leak between sequences
		reg_read16 (RB_SAMP0, v);	check("ram[48] unchanged by write to 49", v, VAL_A);


		// --------------------------------------------------------------
		banner("T3  read-path settling");
		// addr_reg is written in the port-register always block while
		// dout0 <= ram[addr_reg] reads it in another. The op_read0/op_read1
		// split exists precisely for this. Confirm MISO carries the value
		// for the address just loaded, not the previous one.
		reg_read16(RB_SAMP0, v);	check("settling: 48 after 48",            v, VAL_A);
		reg_read16(RB_SAMP1, v);	check("settling: 49 straight after 48",   v, VAL_B);
		reg_read16(RB_SAMP0, v);	check("settling: 48 straight after 49",   v, VAL_A);

		// Read immediately after a write to a DIFFERENT address: addr_reg
		// was last left pointing at 50 by the write.
		reg_write16(RB_SAMP2, 16'h1234);
		reg_read16 (RB_SAMP1, v);	check("settling: 49 straight after write to 50", v, VAL_B);


		// --------------------------------------------------------------
		banner("T4  control-register taps follow the regbank");
		// Readback proves the RAM word changed; the hierarchical peek
		// proves the combinational tap feeding ch_sel changed with it.
		reg_write16(RB_CH_A, 16'h0004);
		reg_read16 (RB_CH_A, v);	check("ram[196] <- 0004", v, 16'h0004);
		check("ch_a tap follows ram[196]", {8'd0, dut.ch_a}, 16'h0004);

		reg_write16(RB_CH_B, 16'h0084);
		reg_read16 (RB_CH_B, v);	check("ram[197] <- 0084", v, 16'h0084);
		check("ch_b tap follows ram[197]", {8'd0, dut.ch_b}, 16'h0084);


		// --------------------------------------------------------------
		banner("T5  ChA/ChB pairing survives interleaved register traffic");
		// The hazard the 'register writes must be an even-numbered batch'
		// rule existed to prevent. Register traffic BETWEEN pop pairs is the
		// only thing that exposes it, and the three writes below are
		// deliberately an ODD transfer count.
		//
		// If all four pairs report 8000/8000 the FIFO was empty throughout
		// and these passed vacuously -- lengthen the fill delay above.
		fifo_pop_pair(a1, b1);	check_pair("pair 1 (clean)", a1, b1);

		reg_write16(RB_SAMP2, 16'hBEEF);		// 3 transfers -- odd on purpose
		fifo_pop_pair(a2, b2);	check_pair("pair 2 (after 3-transfer write)", a2, b2);

		reg_read16(RB_SAMP2, v);	check("ram[50] <- BEEF", v, 16'hBEEF);
		fifo_pop_pair(a3, b3);	check_pair("pair 3 (after read+nop)", a3, b3);

		spi_xfer(CMD_NOP);						// lone odd transfer
		fifo_pop_pair(a1, b1);	check_pair("pair 4 (after a lone NOP)", a1, b1);


		// --------------------------------------------------------------
		banner("T6  abort paths (tag mismatches)");
		// The tag comparators are the new safety mechanism. Each malformed
		// sequence must abort to op_nop0, leave the target word untouched,
		// and leave the FSM resynchronised for the next command.
		reg_write16(RB_SAMP2, 16'hCAFE);		// known starting value

		// 6a: tag 1 then tag 3 -- the high-byte stage is skipped
		spi_xfer(cmd_wr_addr(RB_SAMP2));
		spi_xfer(cmd_wr_low(8'h11));
		reg_read16(RB_SAMP2, v);	check("6a abort (t1,t3): word untouched", v, 16'hCAFE);

		// 6b: tag 2 with no preceding tag 1
		spi_xfer(cmd_wr_high(8'h22));
		reg_read16(RB_SAMP2, v);	check("6b abort (t2 alone): word untouched", v, 16'hCAFE);

		// 6c: tags 1 and 2 correct, wrong tag at the commit stage
		spi_xfer(cmd_wr_addr(RB_SAMP2));
		spi_xfer(cmd_wr_high(8'h33));
		spi_xfer(cmd_wr_tag(6'd1, 8'h44));
		reg_read16(RB_SAMP2, v);	check("6c abort (bad commit tag): word untouched", v, 16'hCAFE);

		// 6d: the FSM is still usable afterwards
		reg_write16(RB_SAMP2, 16'h5A5A);
		reg_read16 (RB_SAMP2, v);	check("6d write works after three aborts", v, 16'h5A5A);


		// --------------------------------------------------------------
		banner("T7  POP not followed by POP");
		// The pair check exists so a broken pair fails loudly instead of
		// half-consuming a FIFO entry. After the abort the interface must
		// resynchronise.
		spi_xfer(CMD_POP);
		spi_xfer(CMD_NOP);						// breaks the pair
		spi_xfer(CMD_NOP);
		fifo_pop_pair(a1, b1);	check_pair("pair after broken POP pair", a1, b1);

		// A NOP must never swallow the transfer after it
		spi_xfer(CMD_NOP);
		reg_read16(RB_SAMP2, v);	check("read still works after a NOP", v, 16'h5A5A);


		// --------------------------------------------------------------
		banner("T8  R-K5: cfg_done across PLL lock drops");
		// 8a: lock drop while sampling. cfg_done clears with rst_n_clk,
		// and the config walk replays from entry 0 after the hold-off.
		drop_lock(2000);
		check_bit("8a cfg_done cleared by lock drop", dut.cfg_done, 1'b0);
		check_bit("8a rst_n_clk low", dut.rst_n_clk, 1'b0);
		wait_sampling();
		check_bit("8a cfg_done set again after the replayed walk", dut.cfg_done, 1'b1);

		// 8b: lock drop in the middle of the config walk, and in the middle
		// of an SPI1 transfer. Controller and SPI master must restart
		// together: the next walk starts at entry 0 with a whole transfer.
		drop_lock(2000);
		wait (dut.rst_n_clk === 1'b1);
		wait (xfer_idx == 10);
		wait (spi1_csb === 1'b0);
		#300;	// inside the transfer
		drop_lock(2000);
		check_bit("8b cfg_done low after a drop mid-walk", dut.cfg_done, 1'b0);
		wait_sampling();
		check_bit("8b cfg_done set after the restarted walk", dut.cfg_done, 1'b1);

		// The regbank is still on rstb: its contents survive (R-K3 note).
		reg_read16(RB_SAMP2, v);	check("regbank survives lock drops", v, 16'h5A5A);

		checks++;
		assert (cfg_rises == 3) $display("  pass  cfg_done rises, whole run: %0d", cfg_rises);
		else begin errors++; $error("cfg_done rose %0d times, expected 3", cfg_rises); end

		checks++;
		assert (csb_rst_errors == 0)
			$display("  pass  spi1_csb high whenever rst_n_clk is low");
		else begin
			errors++;
			$error("spi1_csb low during reset: %0d times (spi_master_controller resets csb_reg to 0)", csb_rst_errors);
		end

		checks++;
		assert ((rx_errors == 0) && (rx_checked > 1000))
			$display("  pass  SPI1 rx matches the models: %0d transfers", rx_checked);
		else begin
			errors++;
			$error("SPI1 rx: %0d mismatches over %0d transfers", rx_errors, rx_checked);
		end

		checks++;
		assert (mon_errors == 0)
			$display("  pass  SPI1 monitor: %0d transfers checked, 0 errors", xfer_total);
		else begin
			errors++;
			$error("SPI1 monitor: %0d errors over %0d transfers", mon_errors, xfer_total);
		end


		// --------------------------------------------------------------
		banner("T12 #6a step 1: PHASE_STEP_CMD (233) steps the PLL; MOSI_SCAN_CTRL (232)");
		// spi1-mosi-eye-scan.md §3.2, §3.4. Through SPI0, as the MCU does
		// it: a REG_WRITE16 to 233 = one phase_step_wr strobe = N PHASESTEP
		// pulses; the stepper's PHASE_POS and the PLL model's phase agree,
		// accumulate across writes, and wrap at SPT (from clock_gen). MOSI
		// is left at mosi_sel = 4 (bypass), so clk_tx's phase reaches
		// nothing yet and the SPI1 traffic checked above is unaffected.
		begin
			int n0;
			int exp_pos;

			reg_read16(RB_SCAN_CTRL, v);	check("12a ram[232] MOSI_SCAN_CTRL default", v, 16'h0004);
			reg_read16(RB_STEP_CMD,  v);	check("12a ram[233] PHASE_STEP_CMD default", v, 16'h0000);
			check("12a mosi_scan_ctrl live default", dut.mosi_scan_ctrl, 16'h0004);
			check("12b SPT from clock_gen", {4'd0, dut.scan0_spt}, 16'(SPT_EXP));
			check_bit("12c PHASESTEP idle high", dut.phasestep, 1'b1);
			// T8's lock drops reset both (stepper on rst_n_clk, model on
			// unlock), so they start equal; checked rather than assumed.
			check("12c stepper and model agree before stepping", {7'd0, dut.phase_pos}, 16'(model_pos));

			exp_pos = model_pos;

			// 12d: N = 5, then N = 5 again: the same stored value must
			// step again (a strobe, not a change detector), and accumulate.
			step_and_check("12d N=5",        16'd5,                 exp_pos);
			step_and_check("12e N=5 again",  16'd5,                 exp_pos);
			// 12f: N = 0 moves nothing.
			step_and_check("12f N=0",        16'd0,                 exp_pos);
			// 12g: direction bit [15] = 1 (advance): both go back by 3.
			step_and_check("12g N=3 back",   16'h8003,              exp_pos);
			// 12h: up to and across the wrap: SPT - pos + 2 delaying steps
			// ends at 2. Wrong if spt is not SPT (S5).
			step_and_check("12h across wrap", 16'(SPT_EXP - exp_pos + 2), exp_pos);
			check("12h position after wrap", {7'd0, dut.phase_pos}, 16'd2);

			reg_read16(RB_STEP_CMD, v);	check("12i ram[233] reads back the last write", v, last_step_cmd);

			// 12j: a write elsewhere (232, 50) must not strobe the stepper.
			n0 = step_wr_n;
			reg_write16(RB_SCAN_CTRL, 16'h0004);
			reg_write16(RB_SAMP2, 16'h5A5A);
			repeat (8) @(posedge dut.clk);
			check("12j no strobe from writes to 232 / 50", 16'(step_wr_n - n0), 16'd0);
			check("12j strobe always one clk", 16'(step_wr_maxlen), 16'd1);

			// 12k: CHK_CLEAR. Writing 0x0014 (mosi_sel 4 + bit 4) gives one
			// one-clk chk_clear pulse; bit 4 reads back 0, in the RAM and in
			// the live copy (decision 2026-11-09).
			n0 = chk_clr_n;
			reg_write16(RB_SCAN_CTRL, 16'h0014);
			repeat (8) @(posedge dut.clk);
			check("12k one chk_clear pulse", 16'(chk_clr_n - n0), 16'd1);
			check("12k chk_clear one clk", 16'(chk_clr_maxlen), 16'd1);
			reg_read16(RB_SCAN_CTRL, v);	check("12k ram[232] bit 4 reads 0", v, 16'h0004);
			check("12k mosi_scan_ctrl live bit 4 is 0", dut.mosi_scan_ctrl, 16'h0004);
			// a write without bit 4 does not pulse
			n0 = chk_clr_n;
			reg_write16(RB_SCAN_CTRL, 16'h0004);
			repeat (8) @(posedge dut.clk);
			check("12k no chk_clear without bit 4", 16'(chk_clr_n - n0), 16'd0);

			// 12m: 240 is read-only: a write lands in the RAM but the read mux
			// still serves the stepper (§3.4).
			reg_write16(RB_PHASE_POS, 16'h1234);
			reg_read16(RB_PHASE_POS, v);	check("12m write to 240 ignored on read", v, 16'(exp_pos));

			checks++;
			assert (model_viol == 0)
				$display("  pass  12l PHASESTEP/PHASEDIR timing as TN-02095: 0 violations");
			else begin
				errors++;
				$error("12l PLL model: %0d PHASESTEP/PHASEDIR timing violations", model_viol);
			end
		end


		// --------------------------------------------------------------
		banner("T13 #6a step 4: MOSI launch path (sources 0-3, clk_tx, bypass)");
		// spi1-mosi-eye-scan.md §3.1. In this tb the clk_tx edge is
		// CLKOS_D + PHASE_POS * T / SPT after clk, so the allowed phases
		// differ from the hardware's, but Delta follows the same formula.
		// Two points per source, inside its allowed x in [T/4, 3T/4] and
		// where the ideal chip still decodes (|Delta| < T); source 3 has
		// one (its other half is past +T). Then back to the bypass, which
		// must give Delta = 0 exactly.
		mosi_point(0, SPT_EXP * 1 / 10);
		mosi_point(0, SPT_EXP * 4 / 10);
		mosi_point(1, SPT_EXP * 6 / 10);
		mosi_point(1, SPT_EXP * 9 / 10);
		mosi_point(2, SPT_EXP * 1 / 10);
		mosi_point(2, SPT_EXP * 4 / 10);
		mosi_point(3, SPT_EXP * 6 / 10);
		mosi_point(4, SPT_EXP * 6 / 10);	// bypass: phase must not matter
		reg_write16(RB_SCAN_CTRL, 16'h0004);


		// --------------------------------------------------------------
		banner($sformatf("T11 K4.3: build variant %0d", KV));
		checks++;
		assert (dut.design_id === 16'({`KUNTUR_VARIANT, `KUNTUR_RELEASE}))
			$display("  pass  11a design_id = %04h", dut.design_id);
		else begin
			errors++;
			$error("11a design_id %04h, expected %04h", dut.design_id, 16'({`KUNTUR_VARIANT, `KUNTUR_RELEASE}));
		end

		if (KV == 3) begin
			// The three lock drops of T8 are behind us: the pattern has
			// already restarted after each one. Run on to cnt 1000 in this
			// segment.
			wait (kp_seg_blocks >= 1000);
			checks++;
			assert ((kp_errors == 0) && (kp_segs == 4))
				$display("  pass  11b K4.2 pattern only: %0d blocks, no gaps, cnt from 0 after start-up and each of %0d restarts",
				         kp_blocks, kp_segs - 1);
			else begin
				errors++;
				$error("11b %0d pattern errors; %0d segments, expected 4", kp_errors, kp_segs);
			end
		end
		else if (KV == 1) begin
			wait (kp_bytes >= 20000);
			checks++;
			assert (kp_errors == 0)
				$display("  pass  11c 00 only on the tunnel: %0d bytes", kp_bytes);
			else begin
				errors++;
				$error("11c %0d non-zero bytes of %0d", kp_errors, kp_bytes);
			end
		end
		else begin
		// Variant 2 (operating): the frames.

		// --------------------------------------------------------------
		banner("T9  K8: capture buffer and frame builder, on the wire");
		// 9a: a known answer in the last slot. Entry 31 = READ(40) makes
		// both chips answer 'I' (0x0049) on A and B; it arrives in
		// transfer 0 of the next frame and must land in slot 31 of the
		// frame that sent it.
		begin
			int n0, f0;
			reg_write16(8'd79, 16'hE800);		// sampling entry 31 = READ(40)
			n0 = k8_read40;
			f0 = k8_frames;
			wait (k8_frames >= f0 + 3);
			checks++;
			assert (k8_read40 > n0)
				$display("  pass  9a READ(40) in entry 31 answers in slot 31: %0d frames", k8_read40 - n0);
			else begin
				errors++;
				$error("9a no frame with 0049 x4 in slot 31 after entry 31 = READ(40)");
			end
		end

		// 9b: everything the receiver compared, over the whole run (T0-T9,
		// with the sampling table rewritten by T2-T6 and T9, and three
		// lock drops). Every announced frame arrives, except one cut short
		// by each lock drop at most.
		checks++;
		assert ((k8_errors == 0) && (k8_frames > 50) && (k8_announced - k8_frames <= 3))
			$display("  pass  9b %0d of %0d announced frames received (%0d cut by a reset), %0d words: header, flags, slots, alt and frame_index match the models",
			         k8_frames, k8_announced, k8_cut, k8_words);
		else begin
			errors++;
			$error("9b %0d errors; %0d of %0d announced frames received", k8_errors, k8_frames, k8_announced);
		end

		// 9g: flags (in 9b) and CONFIG_DIRTY on the frames that should
		// carry it: frame 0 after start-up and after 8b's restart, and
		// the frame during which T2 put WRITE(21, A5) into sampling entry
		// 0 (the walk wrote FF). The other rewrites of entry 0 (T3-T6)
		// write A5 again or are not WRITEs.
		checks++;
		assert (k8_dirty_frames == 3)
			$display("  pass  9g flags match tun_aux's snapshot; CONFIG_DIRTY on %0d frames", k8_dirty_frames);
		else begin
			errors++;
			$error("9g CONFIG_DIRTY on %0d frames, expected 3", k8_dirty_frames);
		end

		checks++;
		assert (k8_early == 0)
			$display("  pass  9c frame_rdy only at transfer 1, never before frame 0 is whole");
		else begin
			errors++;
			$error("9c frame_rdy toggled out of place %0d times", k8_early);
		end

		// 9d: frame 0 at start-up and after 8b's last restart. 8a's restart
		// never reaches a whole frame: 8b drops the lock again after four
		// sampling transfers.
		checks++;
		assert (k8_zero == 2)
			$display("  pass  9d frame_index restarts at 0: after start-up and after the lock drops");
		else begin
			errors++;
			$error("9d frame_index 0 seen %0d times, expected 2", k8_zero);
		end

		checks++;
		assert ((k8_crc_bad == 0) && (k8_frames > 50))
			$display("  pass  9e CRC-32 correct on all %0d frames", k8_frames);
		else begin
			errors++;
			$error("9e CRC-32 wrong on %0d of %0d frames", k8_crc_bad, k8_frames);
		end

		checks++;
		assert ((k8_gap_bad == 0) && (k8_gaps > 50) && (k8_idle_bad == 0))
			$display("  pass  9f one SAMPLE per period (%0d spacings of 379/380 wclk), only 0x00 between frames", k8_gaps);
		else begin
			errors++;
			$error("9f %0d bad spacings of %0d, %0d non-zero bytes between frames", k8_gap_bad, k8_gaps, k8_idle_bad);
		end

		// 9h: auxiliary frames so far: CONFIG after frame 0 (start-up and
		// 8b's restart) and after frame 11 (T2's WRITE(21, A5)); STATUS
		// after frame 1, twice. Content, position and cadence in the
		// receiver.
		checks++;
		assert ((k8_aux_bad == 0) && (k8_n_config == 3) && (k8_n_status == 2))
			$display("  pass  9h %0d CONFIG and %0d STATUS frames: each where expected, right after its SAMPLE, content as the models",
			         k8_n_config, k8_n_status);
		else begin
			errors++;
			$error("9h %0d aux errors; %0d CONFIG (expected 3), %0d STATUS (expected 2)", k8_aux_bad, k8_n_config, k8_n_status);
		end


		// --------------------------------------------------------------
		banner("T10 K7: CONFIG and STATUS cadence over 1000 frames");
		// Since 8b's restart nothing has changed a register, so the
		// spec's schedule (§4.4, §4.4a, §4.8) is, by frame_index:
		// CONFIG 0 (the walk), STATUS 1 (due with 0, waits a frame),
		// STATUS every 300 after that, CONFIG 1000 (1000 after 0).
		// Entry 32 becomes CONVERT(5) first, so the four alt answers in
		// those STATUS frames differ ({chip, half, channel, frame_tag},
		// rhd2164_bfm) and their order is checked; entry 1 is CONVERT(1),
		// so the channel sequence of the next frame is unaffected.
		begin
			int exp_log [$];
			reg_write16(8'd80, 16'h0500);		// sampling entry 32 = CONVERT(5)
			exp_log = '{{8'h02, 24'd0}, {8'h03, 24'd1}, {8'h03, 24'd301},
			            {8'h03, 24'd601}, {8'h03, 24'd901}, {8'h02, 24'd1000}};
			wait (k8_smp_fi >= 1002);
			checks++;
			assert (k8_aux_log == exp_log)
				$display("  pass  10a aux frames after frames 0 (CONFIG), 1, 301, 601, 901 (STATUS), 1000 (CONFIG), none else");
			else begin
				errors++;
				$error("10a aux frames since the restart:");
				foreach (k8_aux_log[i])
					$display("        type %02h after SAMPLE %0d", k8_aux_log[i][31:24], k8_aux_log[i][23:0]);
			end
			checks++;
			assert ((k8_errors == 0) && (k8_aux_bad == 0) && (k8_crc_bad == 0) && (k8_gap_bad == 0) && (k8_idle_bad == 0))
				$display("  pass  10b %0d SAMPLE, %0d CONFIG, %0d STATUS frames over the whole run, all as the models",
				         k8_frames, k8_n_config, k8_n_status);
			else begin
				errors++;
				$error("10b errors: %0d frame, %0d aux, %0d CRC, %0d spacing, %0d idle",
				       k8_errors, k8_aux_bad, k8_crc_bad, k8_gap_bad, k8_idle_bad);
			end
		end
		end		// variant 2


		// --------------------------------------------------------------
		banner("T14 #6a step 5: the checker (words 242-249)");
		// spi1-mosi-eye-scan.md §3.3. Against a reference built from the
		// pins and the models (ref_chk above), plus fixed numbers where the
		// test sets them. Errors are injected in the models' WRITE
		// answers, so every other check (rx, K8) still sees what was sent.
		// MOSI stays on the bypass. Last in the run: the pattern's
		// WRITE(6, ...) marks CONFIG_DIRTY on every frame, which T9/T10's
		// whole-run counts do not expect.
		begin
			logic [15:0] c, snap [8];
			int n_chk;

			// 14a: CHK_EN has been 0 since T12k's clear: nothing counted
			check("14a CHK_COUNT 0 with CHK_EN 0", dut.chk_count, 16'd0);
			check("14a FIRST_ERR_INFO not valid",  dut.first_err_info, 16'd0);
			chk_read_back("14a");

			// 14b: today's table. 48 = 95A5 = WRITE(21, A5) is checked;
			// 49 CONVERT, 50 5A5A (CALIBRATE/CLEAR class), the CONVERTs
			// and READ(63) at 80 are not. One checked transfer per frame.
			reg_write16(RB_SCAN_CTRL, 16'h001C);		// mosi_sel 4, CHK_EN, CHK_CLEAR
			wait_xfers(4 * SAMP_N);
			reg_write16(RB_SCAN_CTRL, 16'h0004);		// CHK_EN 0: hold
			repeat (8) @(posedge dut.clk);
			chk_vs_ref("14b default table");
			// checked entries in the table as it stands (variant 2: T9 left
			// READ(40) in entry 31, so two); 4 frames waited, plus the writes
			n_chk = 0;
			for (int e = 0; e < SAMP_N; e++)
				if (ref_decode(dut.regbank0.ram[SAMP_BASE + e], c)) n_chk++;
			checks++;
			assert ((n_chk > 0) && (dut.chk_count >= 3 * n_chk) && (dut.chk_count <= 8 * n_chk))
				$display("  pass  14b %0d checked entries: CHK_COUNT %0d over ~5 frames", n_chk, dut.chk_count);
			else begin errors++; $error("14b CHK_COUNT %0d with %0d checked entries", dut.chk_count, n_chk); end
			check("14b no errors", dut.err_c0a | dut.err_c0b | dut.err_c1a | dut.err_c1b, 16'd0);

			// 14c: the §5.1 known-answer pattern, every entry but 32
			// checked. Entry 32 stays READ(63) (not checked): a write to it
			// while sampling can land between capture_buffer's latch (at
			// rhd_start) and the master's (one clk later), and K8 then
			// flags alt_cmd for that frame.
			for (int e = 0; e < SAMP_N - 1; e++) begin
				case (e % 8)
					0: c = 16'h8655;	// WRITE(6, 55)
					1: c = 16'hE800;	// READ(40)
					2: c = 16'h86AA;	// WRITE(6, AA)
					3: c = 16'hE900;	// READ(41)
					4: c = 16'h8600;	// WRITE(6, 00)
					5: c = 16'hEA00;	// READ(42)
					6: c = 16'h86FF;	// WRITE(6, FF)
					default: c = 16'hEB00;	// READ(43)
				endcase
				reg_write16(8'(SAMP_BASE + e), c);
			end
			wait_xfers(SAMP_N + 2);
			reg_write16(RB_SCAN_CTRL, 16'h001C);
			wait_xfers(2 * SAMP_N);
			reg_write16(RB_SCAN_CTRL, 16'h0004);
			repeat (8) @(posedge dut.clk);
			chk_vs_ref("14c pattern, no errors");
			checks++;
			assert (dut.chk_count >= 2 * (SAMP_N - 1))
				$display("  pass  14c 32 of 33 transfers checked: %0d", dut.chk_count);
			else begin errors++; $error("14c CHK_COUNT %0d, expected >= %0d", dut.chk_count, 2 * (SAMP_N - 1)); end
			check("14c no errors", dut.err_c0a | dut.err_c0b | dut.err_c1a | dut.err_c1b, 16'd0);
			check("14c FIRST_ERR_INFO not valid", dut.first_err_info, 16'd0);

			// 14d: three wrong chip1 B answers, then one wrong chip0 A:
			// each half counts its own, the first error stays chip1 B.
			inject_off();
			reg_write16(RB_SCAN_CTRL, 16'h001C);
			wait_xfers(SAMP_N);
			inject(1, 1'b1, 16'h0100, 3);
			wait_xfers(SAMP_N);
			inject(0, 1'b0, 16'h8000, 1);
			wait_xfers(SAMP_N);
			reg_write16(RB_SCAN_CTRL, 16'h0004);
			repeat (8) @(posedge dut.clk);
			chk_vs_ref("14d");
			check("14d ERR_C0A", dut.err_c0a, 16'd1);
			check("14d ERR_C0B", dut.err_c0b, 16'd0);
			check("14d ERR_C1A", dut.err_c1a, 16'd0);
			check("14d ERR_C1B", dut.err_c1b, 16'd3);
			check("14d FIRST_ERR_INFO chip1 B", dut.first_err_info, 16'h8003);
			check("14d FIRST_ERR_EXP",          dut.first_err_exp,  chip1.inj_first);
			check("14d FIRST_ERR_ANS",          dut.first_err_ans,  chip1.inj_first ^ 16'h0100);

			// 14e: CHK_EN 0 holds everything, wrong answers included; the
			// MCU's reads see the held values.
			snap = '{dut.chk_count, dut.err_c0a, dut.err_c0b, dut.err_c1a, dut.err_c1b,
			         dut.first_err_ans, dut.first_err_info, dut.first_err_exp};
			inject(0, 1'b1, 16'h0001, 2);
			wait_xfers(SAMP_N);
			check("14e CHK_EN 0: CHK_COUNT holds", dut.chk_count, snap[0]);
			check("14e CHK_EN 0: ERR_C0B holds",   dut.err_c0b,   snap[2]);
			check("14e CHK_EN 0: FIRST_ERR_INFO holds", dut.first_err_info, snap[6]);
			chk_read_back("14e");

			// 14f: two halves wrong in one transfer (chip0 B, chip1 A):
			// the first error is the lower half, 1.
			inject_off();
			reg_write16(RB_SCAN_CTRL, 16'h001C);
			wait_xfers(SAMP_N);
			chip0.inj_xor_b = 16'h0010;  chip1.inj_xor_a = 16'h0020;
			chip0.inj_n = 1;             chip1.inj_n = 1;
			wait_xfers(SAMP_N);
			reg_write16(RB_SCAN_CTRL, 16'h0004);
			repeat (8) @(posedge dut.clk);
			chk_vs_ref("14f");
			check("14f ERR_C0B", dut.err_c0b, 16'd1);
			check("14f ERR_C1A", dut.err_c1a, 16'd1);
			check("14f ERR_C0A + ERR_C1B", dut.err_c0a + dut.err_c1b, 16'd0);
			check("14f FIRST_ERR_INFO chip0 B", dut.first_err_info, 16'h8001);
			check("14f FIRST_ERR_ANS", dut.first_err_ans, chip0.inj_first ^ 16'h0010);
			check("14f same command on both chips", chip1.inj_first, chip0.inj_first);

			// 14f2: chip0 A and chip1 B in one transfer: half 0 first
			inject_off();
			reg_write16(RB_SCAN_CTRL, 16'h001C);
			wait_xfers(SAMP_N);
			chip0.inj_xor_a = 16'h0002;  chip1.inj_xor_b = 16'h0004;
			chip0.inj_n = 1;             chip1.inj_n = 1;
			wait_xfers(SAMP_N);
			reg_write16(RB_SCAN_CTRL, 16'h0004);
			repeat (8) @(posedge dut.clk);
			chk_vs_ref("14f2");
			check("14f2 ERR_C0A", dut.err_c0a, 16'd1);
			check("14f2 ERR_C1B", dut.err_c1b, 16'd1);
			check("14f2 FIRST_ERR_INFO chip0 A", dut.first_err_info, 16'h8000);
			check("14f2 FIRST_ERR_ANS", dut.first_err_ans, chip0.inj_first ^ 16'h0002);

			// 14g: CHK_CLEAR without CHK_EN clears everything, the valid
			// bit included.
			inject_off();
			reg_write16(RB_SCAN_CTRL, 16'h0014);
			repeat (8) @(posedge dut.clk);
			check("14g clear: CHK_COUNT", dut.chk_count, 16'd0);
			check("14g clear: errors", dut.err_c0a | dut.err_c0b | dut.err_c1a | dut.err_c1b, 16'd0);
			check("14g clear: FIRST_ERR_ANS",  dut.first_err_ans,  16'd0);
			check("14g clear: FIRST_ERR_INFO", dut.first_err_info, 16'd0);
			check("14g clear: FIRST_ERR_EXP",  dut.first_err_exp,  16'd0);
			chk_vs_ref("14g");
			// ... and with CHK_EN 0 and nothing latched, a wrong answer
			// latches no first error either
			inject(1, 1'b0, 16'h0008, 2);
			wait_xfers(SAMP_N);
			check("14g CHK_EN 0: no first error latched", dut.first_err_info, 16'd0);
			check("14g CHK_EN 0: ERR_C1A stays 0", dut.err_c1a, 16'd0);
			inject_off();

			// 14h: saturation at FFFF. 65535 transfers is ~50 ms of
			// simulation, so the counters are preset to FFFE (CHK_EN 0,
			// between clk edges), then run on with errors on chip1 B.
			@(negedge dut.clk);
			dut.scan0.mosi_checker0.chk_count = 16'hFFFE;
			dut.scan0.mosi_checker0.err_c1b   = 16'hFFFE;
			ref_cnt = 16'hFFFE;  ref_err[3] = 16'hFFFE;
			reg_write16(RB_SCAN_CTRL, 16'h000C);		// CHK_EN, no clear
			wait_xfers(4);
			inject(1, 1'b1, 16'h0400, 4);
			wait_xfers(SAMP_N);
			reg_write16(RB_SCAN_CTRL, 16'h0004);
			repeat (8) @(posedge dut.clk);
			check("14h CHK_COUNT saturates", dut.chk_count, 16'hFFFF);
			check("14h ERR_C1B saturates",   dut.err_c1b,   16'hFFFF);
			check("14h ERR_C0A still 0",     dut.err_c0a,   16'd0);
			chk_vs_ref("14h");

			inject_off();
			// nothing after T14 needs the old table back
			check("14 SPI1 rx errors, whole run", 16'(rx_errors), 16'd0);
			check("14 SPI1 monitor errors, whole run", 16'(mon_errors), 16'd0);
		end



		// --------------------------------------------------------------
		$display("");
		$display("========================================================");
		$display(" %0d checks, %0d failures", checks, errors);
		if (errors == 0) $display(" *** PASS ***");
		else             $display(" *** FAIL ***");
		$display("========================================================");
		$display("");
		$finish;
	end


	// Safety net: never let a hung handshake run forever.
	initial begin
		#50000000;
		$display("");
		$display(" *** FAIL *** testbench timeout at %0t (%0d checks done)", $time, checks);
		$fatal(1, "testbench timeout");
	end


endmodule
