/*********************************************************************************
	Testbench-only behavioural model of one RHD2164 on spi1, for K8
	(lvds-tunnel.md §4.3.1) and anything else that needs the chip to
	behave like the chip. Instantiate once per chip; both share CS, SCLK
	and MOSI, and each drives its own MISO (chip 0 -> spi1_miso0,
	chip 1 -> spi1_miso1).

	What it models
	  - MOSI decode: 16 bits, MSB first, sampled on SCLK rising; the
	    command executes at CS rising.
	  - MOSI timing at the chip, for the #6a eye scan
	    (spi1-mosi-eye-scan.md §6): the chip sees MOSI MOSI_DELAY ns late
	    (board and chip, relative to SCLK), and needs it stable SETUP ns
	    before and HOLD ns after SCLK rising. A violation in a transfer
	    makes it decode wrong (the word taken, inverted: a real chip takes
	    unknown bits) and counts in n_viol. With
	    the defaults (0, 0, 0) nothing changes.
	  - The answer pipeline: a command's answer is sent two transfers
	    later (rhd2164_defs.vh: "Result is sent to Master two commands
	    later").
	  - DDR MISO with the stub's timing (rhd2164_model.v, which matched the
	    hardware-verified master): the A word is launched on SCLK rising
	    and driven while SCLK is high, the B word on SCLK falling and
	    driven while it is low, each SEL_DELAY ns after the edge, MSB
	    first (lvds-tunnel.md §3.2). Checked against the master in
	    kuntur_tb ("SPI1 rx").
	  - CONVERT(c), c = 0..31: channel c on A, channel c + 32 on B.
	    CONVERT(63) converts the channel after the last one converted
	    (wraps 31 -> 0), so the default sampling table walks 0..31.
	  - READ(r): RAM registers 0..21 as last written, the ROM registers
	    40..44 ("INTAN"), 48..50 ("RHD"), 59 (0x35 on A, 0x3A on B), 60..63.
	    Others read 0.
	  - WRITE(r, d): registers 0..21 take d; the answer is {8'hFF, d}.
	  - CALIBRATE and CLEAR: answer 0x0000 (two's complement mode).
	  - Error injection on WRITE answers (inj_*), for kuntur_tb T14.

	Sample values name themselves, so a misplaced or stale sample is
	visible in the data alone:
	    value = {CHIP[0], half, channel[4:0], frame_tag[8:0]}
	  half = 0 for A (channels 0..31), 1 for B (channels 32..63);
	  frame_tag = number of CONVERT(0) commands decoded so far, mod 512.
	  All 128 samples converted in one sampling frame share a frame_tag.

	Visible to a testbench: regs[0:21], frame_tag, n_xfers, the last
	decoded command (last_cmd) and what this transfer is sending (out_a,
	out_b).
*********************************************************************************/

`timescale 1ns/1ps

module rhd2164_bfm #(
	parameter CHIP = 0,
	parameter real SEL_DELAY = 10.0,
	parameter real MOSI_DELAY = 0.0,
	parameter real SETUP = 0.0,
	parameter real HOLD = 0.0
	)(
	input  wire csb,
	input  wire sck,
	input  wire mosi,
	output wire miso
);
	reg [7:0] regs [0:21];
	reg [15:0] shift_in;
	integer nbits;
	reg [8:0] frame_tag;
	reg [4:0] last_ch;
	integer n_xfers;
	reg [15:0] last_cmd;

	// pipeline: hold_a/b = answer to the previous command; out_a/b = what
	// this transfer sends (answer to the command two transfers back)
	reg [15:0] hold_a, hold_b, out_a, out_b;
	reg [15:0] res_a, res_b;

	integer ia, ib;		// bit index of the A / B word on MISO
	reg sel;			// SCLK, delayed: 1 = drive A, 0 = drive B

	integer i;
	initial begin
		for (i = 0; i <= 21; i = i + 1) regs[i] = 8'd0;
		shift_in = 16'd0;
		nbits = 0;
		frame_tag = 9'd0;
		last_ch = 5'd31;
		n_xfers = 0;
		last_cmd = 16'd0;
		hold_a = 16'd0; hold_b = 16'd0;
		out_a = 16'd0;  out_b = 16'd0;
		ia = 16; ib = 16;
		sel = 1'b0;
	end

	function [7:0] rom(input [5:0] r, input b);
		case (r)
			6'd40: rom = 8'h49;	// I
			6'd41: rom = 8'h4E;	// N
			6'd42: rom = 8'h54;	// T
			6'd43: rom = 8'h41;	// A
			6'd44: rom = 8'h4E;	// N
			6'd48: rom = 8'h52;	// R
			6'd49: rom = 8'h48;	// H
			6'd50: rom = 8'h44;	// D
			6'd59: rom = b ? 8'h3A : 8'h35;
			6'd60: rom = 8'h01;	// die revision
			6'd61: rom = 8'h01;	// unipolar
			6'd62: rom = 8'd64;	// amplifiers
			6'd63: rom = 8'h04;	// RHD2164
			default: rom = 8'h00;
		endcase
	endfunction

	function [15:0] sample(input b, input [4:0] ch);
		sample = {CHIP[0], b, ch, frame_tag};
	endfunction

	// MOSI in, as the chip sees it (MOSI_DELAY), with its setup/hold window
	wire mosi_d;
	assign #(MOSI_DELAY) mosi_d = mosi;
	realtime t_mosi = -1.0e9;	// last change of mosi_d
	realtime t_sck = -1.0e9;	// last SCLK rising while selected
	integer n_viol = 0;
	reg garbled = 1'b0;			// a violation in this transfer
	always @(mosi_d) begin
		t_mosi = $realtime;
		if (!csb && nbits > 0 && $realtime - t_sck < HOLD) begin
			garbled = 1'b1;			// hold violated
			n_viol = n_viol + 1;
		end
	end

	always @(negedge csb) begin
		garbled = 1'b0;
		nbits = 0;
		shift_in = 16'd0;
		ia = 16;
		ib = 16;
	end

	always @(posedge sck)
		if (!csb) begin
			t_sck = $realtime;
			if ($realtime - t_mosi < SETUP) begin
				garbled = 1'b1;			// setup violated
				n_viol = n_viol + 1;
			end
			shift_in = {shift_in[14:0], mosi_d};
			nbits = nbits + 1;
		end

	// Error injection, for a testbench (kuntur_tb T14, the #6a checker):
	// the next inj_n WRITE answers have their A word XORed with inj_xor_a
	// and their B word with inj_xor_b. The model then sends (out_a/out_b,
	// hold_a/hold_b) the corrupted words, so every check built on what
	// the model sent stays consistent. inj_first: the correct answer of
	// the first injected WRITE since the tb cleared inj_seen.
	reg [15:0] inj_xor_a = 16'd0, inj_xor_b = 16'd0;
	integer inj_n = 0;
	reg inj_seen = 1'b0;
	reg [15:0] inj_first = 16'd0;

	// execute at CS rising
	reg [4:0] ch;
	always @(posedge csb)
		if (nbits == 16) begin
			if (garbled)
				shift_in = ~shift_in;
			last_cmd = shift_in;
			case (shift_in[15:14])
				2'b00: begin						// CONVERT
					if (shift_in[13:8] == 6'd63)
						ch = last_ch + 5'd1;
					else
						ch = shift_in[12:8];
					if (shift_in[13:8] == 6'd0)
						frame_tag = frame_tag + 9'd1;
					last_ch = ch;
					res_a = sample(1'b0, ch);
					res_b = sample(1'b1, ch);
				end
				2'b10: begin						// WRITE
					if (shift_in[13:8] <= 6'd21)
						regs[shift_in[13:8]] = shift_in[7:0];
					res_a = {8'hFF, shift_in[7:0]};
					res_b = res_a;
					// tb error injection (kuntur_tb T14)
					if (inj_n > 0) begin
						if (!inj_seen) begin
							inj_first = res_a;
							inj_seen = 1'b1;
						end
						res_a = res_a ^ inj_xor_a;
						res_b = res_b ^ inj_xor_b;
						inj_n = inj_n - 1;
					end
				end
				2'b11: begin						// READ
					if (shift_in[13:8] <= 6'd21) begin
						res_a = {8'h00, regs[shift_in[13:8]]};
						res_b = res_a;
					end
					else begin
						res_a = {8'h00, rom(shift_in[13:8], 1'b0)};
						res_b = {8'h00, rom(shift_in[13:8], 1'b1)};
					end
				end
				default: begin						// CALIBRATE, CLEAR, other
					res_a = 16'h0000;
					res_b = 16'h0000;
				end
			endcase
			// the next transfer sends the previous command's answer
			out_a = hold_a;
			out_b = hold_b;
			hold_a = res_a;
			hold_b = res_b;
			n_xfers = n_xfers + 1;
		end

	// MISO out, DDR (the stub's timing)
	always @(posedge sck) if (!csb) ia = ia - 1;
	always @(negedge sck) if (!csb) ib = ib - 1;
	always @(sck) sel <= #(SEL_DELAY) sck;

	assign miso = csb ? 1'b0 :
	              sel  ? out_a[ia & 15] : out_b[ib & 15];
endmodule
