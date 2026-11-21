/*********************************************************************************
	Testbench-only model of the Intan RHD Recording Controller's SPI port
	(Rhythm on the XEM7310, ControllerRecordUSB3) as lvds-tunnel.md §9.1a and
	§9.1b describe it. One port, two chips' MISO (miso0 = chip 0, miso1 =
	chip 1, as on the adapter).

	Timing
	  - A tick is 1 / (2800 * FS * (1 + ppm 1e-6)): 14.29 ns at 25 kS/s.
	    The tb sets ppm (crystal offset against Kuntur).
	  - One word = 80 ticks, with no gap between sample periods (§9.1a):
	    CS falls at tick 0; bit i (MSB first) is put on MOSI at tick 4i,
	    SCLK is high in ticks 4i+2 .. 4i+3 (rises 1..16, falls 1..16; the
	    16th fall at tick 64); CS rises at tick 66 and stays high to 80.
	    SCLK = tick / 4 (17.5 MHz at 25 kS/s).
	  - DDR MISO: bit i of the A word is sampled `delay` ticks + half a
	    tick after SCLK rise i + 1, bit i of the B word the same after
	    fall i + 1 (MSB first). `delay` is the cable-delay setting, 0..15,
	    as RHX's setCableDelay(); the tb scans it as findConnectedChips().
	  - One sample period = 35 words: CONVERT(0..31), then aux1, aux2,
	    aux3, entry e = period mod 128 of each list (§9.1b).

	Lists (§9.1b, rhxregisters.cpp)
	  aux1[e] = WRITE(3, host_regs[3]).
	  aux2[e] = CONVERT(32), CONVERT(33), CONVERT(34), READ(40), repeating;
	            entry 127 is CONVERT(48) (supply sensor) instead.
	  aux3    = the register-config list, bank 0 (CALIBRATE at 54) or
	            bank 1 (READ(63) there); see build_aux3.
	  host_regs[0..21]: the values RHX writes; set by the tb, then call
	  build_lists.

	Output, per word, for the tb: at the end of each word (CS rising +
	1 tick) the event word_done fires with
	  word_no (from 0 at start), done_cmd (its command), a0, b0 (miso0's
	  A and B words), a1, b1 (miso1's). These hold until the next word
	  ends; cmd, slot and period_no already belong to the next word.
	The answer to word w arrives in word w + 2 (§9.2).
*********************************************************************************/

`timescale 1ns/1ps

module intan_master_bfm #(
	parameter real FS = 25000.0
	)(
	output reg  csb,
	output reg  sck,
	output reg  mosi,
	input  wire miso0,
	input  wire miso1
);
	real ppm = 0.0;				// tb
	integer delay = 1;			// tb: cable-delay setting, 0..15
	reg bank = 1'b0;			// tb: aux3 bank (0 = with CALIBRATE)
	reg [7:0] host_regs [0:21];	// tb
	reg [15:0] aux1 [0:127], aux2 [0:127], aux3 [0:127];

	event word_done;
	integer word_no = -1, period_no = 0, slot = 0;
	reg [15:0] cmd = 16'd0, a0, b0, a1, b1;
	reg [15:0] done_cmd = 16'd0;	// the finished word's command (cmd is already the next one's
									// when a process woken by word_done runs)
	reg running = 1'b0;

	integer i;
	initial begin
		csb = 1'b1; sck = 1'b0; mosi = 1'b0;
		for (i = 0; i <= 21; i = i + 1) host_regs[i] = 8'd0;
	end

	function [15:0] CONVERT(input [5:0] c); CONVERT = {2'b00, c, 8'd0}; endfunction
	function [15:0] READ(input [5:0] r);    READ = {2'b11, r, 8'd0};    endfunction
	function [15:0] WRITE(input [5:0] r, input [7:0] d); WRITE = {2'b10, r, d}; endfunction
	localparam [15:0] CALIBRATE = 16'h5500;

	task build_aux3(input b);
		integer e, k;
		begin
			for (e = 0; e < 128; e = e + 1) aux3[e] = READ(63);
			// 2..17: WRITE 0-2, 4, 5, 7-17 (skips 3 and 6)
			k = 2;
			for (e = 0; e <= 17; e = e + 1)
				if (e != 3 && e != 6) begin aux3[k] = WRITE(e, host_regs[e]); k = k + 1; end
			aux3[18] = READ(63); aux3[19] = READ(62); aux3[20] = READ(61);
			aux3[21] = READ(60); aux3[22] = READ(59);
			for (e = 0; e < 8; e = e + 1) aux3[23 + e] = READ(48 + e);		// 23..30
			for (e = 0; e < 5; e = e + 1) aux3[31 + e] = READ(40 + e);		// 31..35
			for (e = 0; e <= 17; e = e + 1) aux3[36 + e] = READ(e);			// 36..53
			aux3[54] = b ? READ(63) : CALIBRATE;
			for (e = 0; e < 4; e = e + 1) aux3[55 + e] = WRITE(18 + e, host_regs[18 + e]);
		end
	endtask

	task build_lists;
		integer e;
		begin
			for (e = 0; e < 128; e = e + 1) begin
				aux1[e] = WRITE(3, host_regs[3]);
				case (e % 4)
					0: aux2[e] = CONVERT(32);
					1: aux2[e] = CONVERT(33);
					2: aux2[e] = CONVERT(34);
					3: aux2[e] = (e == 127) ? CONVERT(48) : READ(40);
				endcase
			end
			build_aux3(bank);
		end
	endtask

	function real tick;
		tick = 1.0e9 / (2800.0 * FS * (1.0 + ppm * 1.0e-6));
	endfunction

	// one word; the period/slot counters are the caller's
	reg [15:0] ra0, rb0, ra1, rb1;
	task automatic word(input [15:0] c);
		real t;
		integer b;
		begin
			t = tick;
			ra0 = 0; rb0 = 0; ra1 = 0; rb1 = 0;
			cmd = c;
			csb = 1'b0;
			fork
				begin : drive
					integer bd;		// own loop variable: the sample branch runs alongside
					for (bd = 0; bd < 16; bd = bd + 1) begin
						mosi = c[15 - bd];
						#(2.0 * t) sck = 1'b1;
						#(2.0 * t) sck = 1'b0;
					end
					#(2.0 * t) csb = 1'b1;
					#(14.0 * t);
				end
				begin : sample
					for (b = 0; b < 16; b = b + 1) begin
						automatic integer bb = b;
						fork
							begin
								#((4.0 * bb + 2.0 + delay + 0.5) * t);
								ra0[15 - bb] = miso0; ra1[15 - bb] = miso1;
							end
							begin
								#((4.0 * bb + 4.0 + delay + 0.5) * t);
								rb0[15 - bb] = miso0; rb1[15 - bb] = miso1;
							end
						join_none
					end
				end
			join
			a0 = ra0; b0 = rb0; a1 = ra1; b1 = rb1;
			done_cmd = c;
			word_no = word_no + 1;
			-> word_done;
		end
	endtask

	// n sample periods
	task run_periods(input integer n);
		integer p, s, e;
		begin
			running = 1'b1;
			for (p = 0; p < n; p = p + 1) begin
				e = period_no % 128;
				for (s = 0; s < 35; s = s + 1) begin
					slot = s;
					if (s < 32)       word(CONVERT(s));
					else if (s == 32) word(aux1[e]);
					else if (s == 33) word(aux2[e]);
					else              word(aux3[e]);
				end
				period_no = period_no + 1;
			end
			running = 1'b0;
		end
	endtask
endmodule
