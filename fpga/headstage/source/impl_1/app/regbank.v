/*********************************************************************************
	Author: Manuel Monge
	Description:
		Kuntur's regbank: the 256-word memory holding the RHD2164
		config/sampling command tables and the MCU-facing control
		registers (ch_a/ch_b/stream_enable/data_source_sel — those four
		live in dedicated registers below, not this array, since they're
		read combinationally every cycle and can't live inside a hard EBR
		primitive; see the "Writing"/mirror-register comments further
		down). Renamed from `ram` 2026-08-26 (PLAN.md B.3): this was never
		a generic memory primitive — its `initial` block hardcodes the
		actual RHD2164 command sequence, which is why it lives in app/
		rather than common/, alongside the truly-generic `spi_master_rhd2164x2`
		vs. `spi_master` distinction made the same session.
*********************************************************************************/

`include "../afe/rhd2164/rhd2164_defs.vh"
`include "regbank_map.vh"

module regbank #(
	// Both localparams, not real parameters (2026-08-26 parametrization
	// review): neither is actually free to change without a wider
	// redesign. DATA_WIDTH=16 is pinned by main_controller.v's 3-transfer
	// WRITE sequence (address byte / high byte / commit-low byte, each an
	// 8-bit SPI0 transfer) and dtx_mux_reg.v's fixed 16-bit ram_din — both
	// hardcode the 16-bit shape with no parameter of their own.
	// ADDR_WIDTH=8 is pinned the same way: addr_reg/staged_h below are
	// hardcoded [7:0] to match the SPI0 protocol's one-byte-per-transfer
	// address field, not [ADDR_WIDTH-1:0] — growing past 8 needs a
	// wire-protocol change (more than one address-load transfer), not
	// just a wider register. Making either genuinely free would mean
	// redesigning the SPI0 wire protocol across main_controller.v too,
	// not just this module — out of scope here. Kept in the #(...)
	// header (rather than a body-level localparam) only because Verilog
	// requires them declared before use in the port widths below.
	localparam DATA_WIDTH = 16,
	localparam ADDR_WIDTH = 8
	)(
	input  wire	rstb,					// Active-low asyncrhonous reset
	input  wire	clk,					// Input clock
	input  wire	wen,						// Input; RegBank write enable
	input  wire	[ADDR_WIDTH-1:0] addr0,	// Input Address, Port0
	input  wire [DATA_WIDTH-1:0] din0,	// Input Data, Port0
	output reg	[DATA_WIDTH-1:0] dout0,	// Output Data, Port0
	input  wire	[ADDR_WIDTH-1:0] addr1,	// Input Address, Port0
	output reg	[DATA_WIDTH-1:0] dout1,	// Output Data, Port0
	input  wire	regbank_port_en,
	input  wire	[1:0]	regbank_port_sel	,
	// Config connections
	output wire	[7:0]	ch_a,
	output wire	[7:0]	ch_b,
	output wire stream_enable,
	output wire	[1:0]	data_source_sel,
	// spi1_eye_scan outputs
	output reg	phase_step_wr,
	output reg	[15:0] phase_step_cmd,
	output reg	[15:0] mosi_scan_ctrl,
	output reg	chk_clear,
	// spi1_eye_scan inputs
	input  wire	[8:0]	phase_pos,
	input  wire	[15:0]	phase_info,
	input  wire	[15:0]	chk_count,
	input  wire	[15:0]	err_c0a,
	input  wire	[15:0]	err_c0b,
	input  wire	[15:0]	err_c1a,
	input  wire	[15:0]	err_c1b,
	input  wire	[15:0]	first_err_ans,
	input  wire	[15:0]	first_err_info,
	input  wire	[15:0]	first_err_exp
);
	
	// memory
	// No per-word async reset on this array (see the Writing block below) —
	// deliberately, so it can synthesize as EBR instead of ~4096 discrete
	// flip-flops. Same idiom already proven in this file by fifo_ram
	// (components.v ~line 750): an array whose *contents* are never touched
	// in a reset branch, only written by `wen`. Defaults now come from the
	// `initial` block below (bitstream-configuration-time only) instead of
	// active reset logic. PLAN.md A.1.1g "regbank area", 2026-08-26.
	reg [DATA_WIDTH-1:0] ram [0:(2**ADDR_WIDTH)-1];

	// registers
	reg [7:0] addr_reg, staged_h;
	reg	[DATA_WIDTH-1:0] dout0ram;
	wire [DATA_WIDTH-1:0] wdata;

	// ch_a/ch_b/stream_enable/data_source_sel are read *combinationally*,
	// every cycle, by other modules (ch_sel's channel comparators, fifo_wen
	// gating, the fifo_din source mux) — not through the registered
	// dout0/dout1 readback path the rest of the regbank uses. A hard EBR
	// primitive has no combinational output for an arbitrary fixed address,
	// only a registered one, so these four cannot live inside the array
	// above once it's EBR-backed. Pulled out into small dedicated registers
	// instead, mirrored from the same write event (see the always block
	// below) — these keep their real async reset exactly as before; only
	// the bulk config/sampling/free table's reset-restore behaviour changes.
	reg [7:0] ch_a_reg, ch_b_reg;
	reg stream_enable_reg;
	reg [1:0] data_source_sel_reg;
	
	// Config connections
	// rhd2164_sampling_cmd0-3 are dead (A.1.4 — connected to nothing beyond
	// this module's boundary; kuntur_fpga.v doesn't even wire cmd3). Left
	// reading from `ram` as before rather than touching kuntur_fpga.v's
	// port list in this pass; expected to synthesize away as unused, or at
	// worst pull one fixed word out of the EBR-backed table as a small
	// extra tap — watch for this in the next synthesis run. Real fix is
	// still deleting these ports/assigns per A.1.4, deferred so as not to
	// touch kuntur_fpga.v while it's mid-edit for the item-4 placement work.
	assign ch_a = ch_a_reg;
	assign ch_b = ch_b_reg;
	assign stream_enable = stream_enable_reg;
	assign data_source_sel = data_source_sel_reg;
	
	// Port Registers
	always@(negedge rstb or posedge clk)
		if (!rstb) begin
			addr_reg <= 0;
			staged_h <= 0;
		end
		else if (regbank_port_en)
			case (regbank_port_sel)
				2'd1: addr_reg <= din0[7:0];
				2'd2: staged_h <= din0[7:0];
				default: ;
			endcase
	
	
	
	// Defaults, applied at bitstream configuration/power-on only — not
	// re-applied on a bare rstb pulse (see the Writing block below). This is
	// the deliberate behaviour change from 2026-08-26: restoring known-good
	// values without reprogramming is now a firmware (REG_WRITE) job, not
	// an RTL one — see PLAN.md A.1.1g "regbank area" and B.5/B.6 doctor.
	initial begin

		// RHD Config - Starts at 8'd0

		ram[`RB_CONFIG_BASE + 8'd0] = `RHD_READ(6'd63);				// value: 0xFF00
		ram[`RB_CONFIG_BASE + 8'd1] = `RHD_READ(6'd63);				// value: 0xFF00
		ram[`RB_CONFIG_BASE + 8'd2] = `RHD_WRITE(6'd0, `RHD_REG0);	// value: 0x80DE
		ram[`RB_CONFIG_BASE + 8'd3] = `RHD_WRITE(6'd1, `RHD_REG1);	// value: 0x8102
		ram[`RB_CONFIG_BASE + 8'd4] = `RHD_WRITE(6'd2, `RHD_REG2);	// value: 0x8204
		ram[`RB_CONFIG_BASE + 8'd5] = `RHD_WRITE(6'd3, `RHD_REG3);	// value: 0x8302
		ram[`RB_CONFIG_BASE + 8'd6] = `RHD_WRITE(6'd4, `RHD_REG4);	// value: 0x84C0
		ram[`RB_CONFIG_BASE + 8'd7] = `RHD_WRITE(6'd5, `RHD_REG5);	// value: 0x8500
		ram[`RB_CONFIG_BASE + 8'd8] = `RHD_WRITE(6'd6, `RHD_REG6);
		ram[`RB_CONFIG_BASE + 8'd9] = `RHD_WRITE(6'd7, `RHD_REG7);
		ram[`RB_CONFIG_BASE + 8'd10] = `RHD_WRITE(6'd8, `RHD_REG8);
		ram[`RB_CONFIG_BASE + 8'd11] = `RHD_WRITE(6'd9, `RHD_REG9);
		ram[`RB_CONFIG_BASE + 8'd12] = `RHD_WRITE(6'd10, `RHD_REG10);
		ram[`RB_CONFIG_BASE + 8'd13] = `RHD_WRITE(6'd11, `RHD_REG11);
		ram[`RB_CONFIG_BASE + 8'd14] = `RHD_WRITE(6'd12, `RHD_REG12);
		ram[`RB_CONFIG_BASE + 8'd15] = `RHD_WRITE(6'd13, `RHD_REG13);
		ram[`RB_CONFIG_BASE + 8'd16] = `RHD_WRITE(6'd14, `RHD_REG14);
		ram[`RB_CONFIG_BASE + 8'd17] = `RHD_WRITE(6'd15, `RHD_REG15);
		ram[`RB_CONFIG_BASE + 8'd18] = `RHD_WRITE(6'd16, `RHD_REG16);
		ram[`RB_CONFIG_BASE + 8'd19] = `RHD_WRITE(6'd17, `RHD_REG17);
		ram[`RB_CONFIG_BASE + 8'd20] = `RHD_WRITE(6'd18, `RHD_REG18);
		ram[`RB_CONFIG_BASE + 8'd21] = `RHD_WRITE(6'd19, `RHD_REG19);
		ram[`RB_CONFIG_BASE + 8'd22] = `RHD_WRITE(6'd20, `RHD_REG20);
		ram[`RB_CONFIG_BASE + 8'd23] = `RHD_WRITE(6'd21, `RHD_REG21);
		ram[`RB_CONFIG_BASE + 8'd24] = `RHD_CALIBRATE;
		ram[`RB_CONFIG_BASE + 8'd25] = `RHD_READ(6'd63);				// value: 0xFF00
		ram[`RB_CONFIG_BASE + 8'd26] = `RHD_READ(6'd63);				// value: 0xFF00
		ram[`RB_CONFIG_BASE + 8'd27] = `RHD_READ(6'd63);				// value: 0xFF00
		ram[`RB_CONFIG_BASE + 8'd28] = `RHD_READ(6'd63);				// value: 0xFF00
		ram[`RB_CONFIG_BASE + 8'd29] = `RHD_READ(6'd63);				// value: 0xFF00
		ram[`RB_CONFIG_BASE + 8'd30] = `RHD_READ(6'd63);				// value: 0xFF00
		ram[`RB_CONFIG_BASE + 8'd31] = `RHD_READ(6'd63);				// value: 0xFF00
		ram[`RB_CONFIG_BASE + 8'd32] = `RHD_READ(6'd63);				// value: 0xFF00
		ram[`RB_CONFIG_BASE + 8'd33] = `RHD_READ(6'd63);				// value: 0xFF00
		ram[`RB_CONFIG_BASE + 8'd34] = 16'd0;
		ram[`RB_CONFIG_BASE + 8'd35] = 16'd0;
		ram[`RB_CONFIG_BASE + 8'd36] = 16'd0;
		ram[`RB_CONFIG_BASE + 8'd37] = 16'd0;
		ram[`RB_CONFIG_BASE + 8'd38] = 16'd0;
		ram[`RB_CONFIG_BASE + 8'd39] = 16'd0;
		ram[`RB_CONFIG_BASE + 8'd40] = 16'd0;
		ram[`RB_CONFIG_BASE + 8'd41] = 16'd0;
		ram[`RB_CONFIG_BASE + 8'd42] = 16'd0;
		ram[`RB_CONFIG_BASE + 8'd43] = 16'd0;
		ram[`RB_CONFIG_BASE + 8'd44] = 16'd0;
		ram[`RB_CONFIG_BASE + 8'd45] = 16'd0;
		ram[`RB_CONFIG_BASE + 8'd46] = 16'd0;
		ram[`RB_CONFIG_BASE + 8'd47] = 16'd0;

		// RHD Sampling - Starts at 8'd48

		ram[`RB_SAMPLING_BASE + 8'd0] = `RHD_CONVERT(6'd0);
		ram[`RB_SAMPLING_BASE + 8'd1] = `RHD_CONVERT(6'd1);
		ram[`RB_SAMPLING_BASE + 8'd2] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd3] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd4] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd5] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd6] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd7] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd8] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd9] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd10] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd11] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd12] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd13] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd14] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd15] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd16] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd17] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd18] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd19] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd20] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd21] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd22] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd23] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd24] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd25] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd26] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd27] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd28] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd29] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd30] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd31] = `RHD_CONVERT(6'd63);
		ram[`RB_SAMPLING_BASE + 8'd32] = `RHD_READ(6'd63);	// RHD2164 Sampling: cmd0
		ram[`RB_SAMPLING_BASE + 8'd33] = 16'd0;
		ram[`RB_SAMPLING_BASE + 8'd34] = 16'd0;
		ram[`RB_SAMPLING_BASE + 8'd35] = 16'd0;
		ram[`RB_SAMPLING_BASE + 8'd36] = 16'd0;
		ram[`RB_SAMPLING_BASE + 8'd37] = 16'd0;
		ram[`RB_SAMPLING_BASE + 8'd38] = 16'd0;
		ram[`RB_SAMPLING_BASE + 8'd39] = 16'd0;
		ram[`RB_SAMPLING_BASE + 8'd40] = 16'd0;
		ram[`RB_SAMPLING_BASE + 8'd41] = 16'd0;
		ram[`RB_SAMPLING_BASE + 8'd42] = 16'd0;
		ram[`RB_SAMPLING_BASE + 8'd43] = 16'd0;
		ram[`RB_SAMPLING_BASE + 8'd44] = 16'd0;
		ram[`RB_SAMPLING_BASE + 8'd45] = 16'd0;
		ram[`RB_SAMPLING_BASE + 8'd46] = 16'd0;
		ram[`RB_SAMPLING_BASE + 8'd47] = 16'd0;

		// CONFIG (control-region defaults, kept here for readback-map
		// consistency across all 256 words; the four *live* values are
		// separately reset in the ch_a_reg/etc. block below, which is what
		// actually drives them at runtime)
		
		// Starts at 8'd192

		ram[`RB_CTRL_BASE + 8'd0] = `RHD_READ(6'd63);	// RHD2164 Sampling: cmd0
		ram[`RB_CTRL_BASE + 8'd1] = `RHD_READ(6'd63);	// RHD2164 Sampling: cmd1
		ram[`RB_CTRL_BASE + 8'd2] = `RHD_READ(6'd63);	// RHD2164 Sampling: cmd2
		ram[`RB_CTRL_BASE + 8'd3] = `RHD_READ(6'd63);	// RHD2164 Sampling: cmd3
		ram[`RB_CTRL_BASE + 8'd4] = {8'd0,2'd2,6'd3};	// Default: ChA - Remember there is a delay of 2 SPI cycles
		ram[`RB_CTRL_BASE + 8'd5] = {8'd0,2'd2,6'd2};// Default: ChB
		ram[`RB_CTRL_BASE + 8'd6] = 16'd2;			// Default: ChC
		ram[`RB_CTRL_BASE + 8'd7] = 16'd3;			// Default: ChD
		ram[`RB_CTRL_BASE + 8'd8] = 16'd4;			// Default: ChE
		ram[`RB_CTRL_BASE + 8'd9] = 16'd5;			// Default: ChF
		ram[`RB_CTRL_BASE + 8'd10] = 16'd6;			// Default: ChG
		ram[`RB_CTRL_BASE + 8'd11] = 16'd7;			// Default: ChH
		ram[`RB_CTRL_BASE + 8'd12] = 16'd8;			// Default: ChI
		ram[`RB_CTRL_BASE + 8'd13] = 16'd9;			// Default: ChJ
		ram[`RB_CTRL_BASE + 8'd14] = 16'd10;			// Default: ChK
		ram[`RB_CTRL_BASE + 8'd15] = 16'd11;			// Default: ChL
		ram[`RB_CTRL_BASE + 8'd16] = 16'd12;			// Default: ChM
		ram[`RB_CTRL_BASE + 8'd17] = 16'd13;			// Default: ChN
		ram[`RB_CTRL_BASE + 8'd18] = 16'd14;			// Default: ChO
		ram[`RB_CTRL_BASE + 8'd19] = 16'd15;			// Default: ChP
		ram[`RB_CTRL_BASE + 8'd20] = 16'd0;
		ram[`RB_CTRL_BASE + 8'd21] = 16'd0;
		ram[`RB_CTRL_BASE + 8'd22] = 16'd0;
		ram[`RB_CTRL_BASE + 8'd23] = 16'd0;
		ram[`RB_CTRL_BASE + 8'd24] = 16'd0;
		ram[`RB_CTRL_BASE + 8'd25] = 16'd0;
		ram[`RB_CTRL_BASE + 8'd26] = 16'd0;
		ram[`RB_CTRL_BASE + 8'd27] = 16'd0;
		ram[`RB_CTRL_BASE + 8'd28] = 16'd0;
		ram[`RB_CTRL_BASE + 8'd29] = 16'd0;
		ram[`RB_CTRL_BASE + 8'd30] = 16'd0;
		ram[`RB_CTRL_BASE + 8'd31] = 16'd0;
		ram[`RB_CTRL_BASE + 8'd32] = 16'd0;
		ram[`RB_CTRL_BASE + 8'd33] = 16'd0;
		ram[`RB_CTRL_BASE + 8'd34] = 16'd0;
		ram[`RB_CTRL_BASE + 8'd35] = 16'd0;
		ram[`RB_CTRL_BASE + 8'd36] = {15'd0,1'b1}; // bit0: stream_enable
		ram[`RB_CTRL_BASE + 8'd37] = 16'd0; // bits [1:0]: data_source_sel, reset 0 = REAL
		ram[`RB_CTRL_BASE + 8'd38] = 16'd0;
		ram[`RB_CTRL_BASE + 8'd39] = 16'd0;
		ram[`RB_CTRL_BASE + 8'd40] = 16'h0004; // 232: MOSI_SCAN_CTRL
		ram[`RB_CTRL_BASE + 8'd41] = 16'd0; // 233: PHASE_STEP_CMD
		ram[`RB_CTRL_BASE + 8'd42] = 16'd0;
		ram[`RB_CTRL_BASE + 8'd43] = 16'd0;
		ram[`RB_CTRL_BASE + 8'd44] = 16'd0;
		ram[`RB_CTRL_BASE + 8'd45] = 16'd0;
		ram[`RB_CTRL_BASE + 8'd46] = 16'd0;
		ram[`RB_CTRL_BASE + 8'd47] = 16'd0;

		// unused (deliberately reserved headroom, PLAN.md B.1 — free now
		// that it's EBR-backed instead of costing a flip-flop each)

		ram[96] = 16'd0; ram[97] = 16'd0; ram[98] = 16'd0; ram[99] = 16'd0;
		ram[100] = 16'd0; ram[101] = 16'd0; ram[102] = 16'd0; ram[103] = 16'd0; ram[104] = 16'd0;
		ram[105] = 16'd0; ram[106] = 16'd0; ram[107] = 16'd0;	ram[108] = 16'd0; ram[109] = 16'd0;
		ram[110] = 16'd0; ram[111] = 16'd0; ram[112] = 16'd0; ram[113] = 16'd0; ram[114] = 16'd0;
		ram[115] = 16'd0; ram[116] = 16'd0; ram[117] = 16'd0;	ram[118] = 16'd0; ram[119] = 16'd0;
		ram[120] = 16'd0; ram[121] = 16'd0; ram[122] = 16'd0; ram[123] = 16'd0; ram[124] = 16'd0;
		ram[125] = 16'd0; ram[126] = 16'd0; ram[127] = 16'd0;	ram[128] = 16'd0; ram[129] = 16'd0;
		ram[130] = 16'd0; ram[131] = 16'd0; ram[132] = 16'd0; ram[133] = 16'd0; ram[134] = 16'd0;
		ram[135] = 16'd0; ram[136] = 16'd0; ram[137] = 16'd0;	ram[138] = 16'd0; ram[139] = 16'd0;
		ram[140] = 16'd0; ram[141] = 16'd0; ram[142] = 16'd0; ram[143] = 16'd0; ram[144] = 16'd0;
		ram[145] = 16'd0; ram[146] = 16'd0; ram[147] = 16'd0;	ram[148] = 16'd0; ram[149] = 16'd0;
		ram[150] = 16'd0; ram[151] = 16'd0; ram[152] = 16'd0; ram[153] = 16'd0; ram[154] = 16'd0;
		ram[155] = 16'd0; ram[156] = 16'd0; ram[157] = 16'd0;	ram[158] = 16'd0; ram[159] = 16'd0;
		ram[160] = 16'd0; ram[161] = 16'd0; ram[162] = 16'd0; ram[163] = 16'd0; ram[164] = 16'd0;
		ram[165] = 16'd0; ram[166] = 16'd0; ram[167] = 16'd0;	ram[168] = 16'd0; ram[169] = 16'd0;
		ram[170] = 16'd0; ram[171] = 16'd0; ram[172] = 16'd0; ram[173] = 16'd0; ram[174] = 16'd0;
		ram[175] = 16'd0; ram[176] = 16'd0; ram[177] = 16'd0;	ram[178] = 16'd0; ram[179] = 16'd0;
		ram[180] = 16'd0; ram[181] = 16'd0; ram[182] = 16'd0; ram[183] = 16'd0; ram[184] = 16'd0;
		ram[185] = 16'd0; ram[186] = 16'd0; ram[187] = 16'd0;	ram[188] = 16'd0; ram[189] = 16'd0;
		ram[190] = 16'd0; ram[191] = 16'd0;

		ram[240] = 16'd0; ram[241] = 16'd0; ram[242] = 16'd0; ram[243] = 16'd0; ram[244] = 16'd0;
		ram[245] = 16'd0; ram[246] = 16'd0; ram[247] = 16'd0;	ram[248] = 16'd0; ram[249] = 16'd0;
		ram[250] = 16'd0; ram[251] = 16'd0; ram[252] = 16'd0; ram[253] = 16'd0; ram[254] = 16'd0;
		ram[255] = 16'd0;

	end

	// Writing — runtime only. No reset branch touches `ram` (see the
	// initial block above and the comment on the array declaration).
	always@(negedge rstb or posedge clk)
		if (!rstb) begin
			// ram content deliberately not reset here — see above.
		end
		else if (wen) begin
			ram[addr_reg] <= wdata;
		end



	assign wdata = ( addr_reg == (`RB_CTRL_BASE + 8'd40) ) ? {staged_h,din0[7:5], 1'b0, din0[3:0]} : {staged_h,din0[7:0]};
	// Mirror registers off the same write
	// event. Real async reset, unchanged semantics from before this change.
	always@(negedge rstb or posedge clk)
		if (!rstb) begin
			ch_a_reg <= {2'd2,6'd3};
			ch_b_reg <= {2'd2,6'd2};
			stream_enable_reg <= 1'b1;
			data_source_sel_reg <= 2'd0;
			phase_step_cmd <= 16'd0;
			phase_step_wr <= 1'b0;
			mosi_scan_ctrl <= 16'h0004;
			chk_clear <= 1'b0;
		end
		else if (wen) begin
			case (addr_reg)
				(`RB_CTRL_BASE + 8'd4): ch_a_reg <= din0[7:0];
				(`RB_CTRL_BASE + 8'd5): ch_b_reg <= din0[7:0];
				(`RB_CTRL_BASE + 8'd36): stream_enable_reg <= din0[0];
				(`RB_CTRL_BASE + 8'd37): data_source_sel_reg <= din0[1:0];
				(`RB_CTRL_BASE + 8'd40): begin
					mosi_scan_ctrl <= wdata;
					chk_clear <= din0[4];
				end
				(`RB_CTRL_BASE + 8'd41): begin
					phase_step_cmd <= {staged_h, din0[7:0]};
					phase_step_wr <= 1'b1;
				end
				default: ;
			endcase
		end
		else begin
			phase_step_wr <= 1'b0;
			chk_clear <= 1'b0;
		end

	// Reading
	always@(negedge rstb or posedge clk)
		if (!rstb) begin
			dout0ram <= 0;
			dout1 <= 0;
		end
		else begin
			dout0ram <= ram[addr_reg];
			dout1 <= ram[addr1];
		end
	
	
	// Reading mux
	always@(*)
		case (addr_reg)
			8'd240: dout0 = {7'd0, phase_pos};
			8'd241: dout0 = phase_info;
			8'd242: dout0 = chk_count;
			8'd243: dout0 = err_c0a;
			8'd244: dout0 = err_c0b;
			8'd245: dout0 = err_c1a;
			8'd246: dout0 = err_c1b;
			8'd247: dout0 = first_err_ans;
			8'd248: dout0 = first_err_info;
			8'd249: dout0 = first_err_exp;
			default: dout0 = dout0ram;
		endcase

	
		
endmodule
