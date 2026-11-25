/*********************************************************************************
	emu_answer_tb: unit testbench for T5.4 emu_answer (lvds-tunnel.md §9.6 T5.4,
	"Added 2026-11-24", Q1-Q9; §9.1, §9.1b, §9.2, §9.3).

	DUT: the slave (rhd2164x2_spi_slave_model), rhd2164_model_reg_ram and
	emu_answer, wired as the top level wires them (Q1-Q7, Q10: word_end), driven by
	intan_master_bfm (RHX's timing, §9.1a; 25 kS/s, cable delay 1, also 0).

	The tb plays T5.3 and T5.5.
	  T5.3 stub (sck): at every rising edge, rd_* <= sval(frame, rd_slot, lane);
	    frame + 1 at a rising edge where frame_start is high and was low at the
	    edge before (T5.3 edge-detects), and that edge's read already uses the
	    new frame. sval = {lane[1:0], slot[4:0], frame[8:0]}, lane 0..3 =
	    chip 0 A, chip 0 B, chip 1 A, chip 1 B, so a value names where it came
	    from. rd_valid is set per word, in the CS-high gap.
	  T5.5 stand-in: cfg_regs_s / cfg_valid_s, changed only in a CS-high gap.

	Reference model (independent of the RTL), one entry per word N:
	  CONVERT(C): slot = C for C < 32; C = 63: last + 1, mod 32. last <- C for
	    C < 32, last + 1 for C = 63; other C leave it (Q5). frame_start level
	    for N = C == 0, or C == 63 with last == 31 (before the update). The
	    frame counter steps when N's level is high and N-1's was low.
	  After reset: last = 31; the command before the first word is the slave's
	    reset data_rx, READ(63) (Q1).
	  The answer to word N, in word N + 2 (§9.2), with rd_valid / cfg as they
	    stood during word N + 1 (they are taken at tx_load, Q6):
	    CONVERT(0..31, 63): rd_valid ? sval ^ 16'h8000 : 16'h0000 (§9.2)
	    CONVERT(32/33/34): 0; CONVERT(48): 44100; other C: 0 (Q4)
	    READ(0..21): cfg_valid_s ? {00, cfg[R]} : 0, all four lanes (R4, Q7)
	    READ(22..63): ROM (40-44 "INTAN", 48-50 "RHD", 59 = 0x35 on A and
	      0x3A on B, 61 = 1, 62 = 64, 63 = 4, others 0)
	    WRITE(R, D): {FF, D}; CALIBRATE, CLEAR: 0x0000 (§9.3)

	Checks (each failure prints ERROR and counts; the run ends PASS / FAIL)
	  A  every answer word, all four lanes, against the model, from the
	     word after reset on (A-1 is the answer to the reset READ(63)).
	  F1 frame_start at edges 1 and 2 of word N+1 = N's level; at edges 3..16
	     too, except after a CONVERT(63) (Q8).
	  F2 rd_slot at edges 1 and 2 of word N+1 = N's slot, for CONVERT(0..31,
	     63); at edges 3..16 too for C < 32 (Q8).
	  F3 cmd_strobe high at edge 2 of every word and at no other edge (Q2).
	  T1 frame_start never rises except at edge 1 of a word (T5.3 decides at
	     edge 1, §9.6 T5.3 tb assumptions).
	  R1 frame_start low after reset, before the first word (Q1).

	Phases: RHX lists (§9.1b) bank 0 without CONFIG, bank 0 with CONFIG,
	bank 1 with rd_valid and cfg changing, cable delay 0; random command
	words; CONVERT(63) sweeps alone and mixed; resets mid stream.

	Run: tb/run_emu_answer.sh (Questa). Plusargs: +seed=N, +nrand=N (random
	words, default 20000).
*********************************************************************************/

`timescale 1ns/1ps

module emu_answer_tb;

	integer errors = 0;
	int n_kind [string];
	task automatic err(input string s);
		string k;
		begin
			errors = errors + 1;
			k = s.substr(0, (s.len() < 24 ? s.len() : 24) - 1);
			if (!n_kind.exists(k)) n_kind[k] = 0;
			n_kind[k]++;
			if (n_kind[k] <= 3) $display("%t ERROR %s", $realtime, s);
			if (n_kind[k] == 3) $display("%t   (further \"%s...\" errors counted, not printed)", $realtime, k);
		end
	endtask
	task automatic err_summary;
		string k;
		if (n_kind.first(k)) do
			if (n_kind[k] > 3) $display("  %0d x \"%s...\"", n_kind[k], k);
		while (n_kind.next(k));
	endtask

	function automatic [15:0] sval(input int fr, input [4:0] s, input [1:0] lane);
		sval = {lane, s, fr[8:0]};
	endfunction

	function automatic [7:0] rom(input [5:0] r, input bit b);
		case (r)
			6'd40: rom = 8'h49; 6'd41: rom = 8'h4E; 6'd42: rom = 8'h54; 6'd43: rom = 8'h41; 6'd44: rom = 8'h4E;
			6'd48: rom = 8'h52; 6'd49: rom = 8'h48; 6'd50: rom = 8'h44;
			6'd59: rom = b ? 8'h3A : 8'h35;
			6'd61: rom = 8'd1; 6'd62: rom = 8'd64; 6'd63: rom = 8'd4;
			default: rom = 8'h00;
		endcase
	endfunction

	function [15:0] CONVERT(input [5:0] c); CONVERT = {2'b00, c, 8'd0}; endfunction
	function [15:0] READ(input [5:0] r);    READ = {2'b11, r, 8'd0};    endfunction
	function [15:0] WRITE(input [5:0] r, input [7:0] d); WRITE = {2'b10, r, d}; endfunction
	localparam [15:0] CALIBRATE = 16'h5500, CLEAR = 16'h6A00;

	// ---------------------------------------------------------------- DUT
	wire csb, sck, mosi, miso0, miso1;
	reg  rstb = 1'b0;

	intan_master_bfm #(.FS(25000.0)) bfm (.csb(csb), .sck(sck), .mosi(mosi), .miso0(miso0), .miso1(miso1));

	wire [15:0] data_rx, dtx_a0, dtx_b0, dtx_a1, dtx_b1;
	wire tx_load, word_end;
	wire [7:0] dout0, dout1, dh;
	reg  [15:0] rd_a0 = 0, rd_b0 = 0, rd_a1 = 0, rd_b1 = 0;
	reg  rd_valid = 1'b1;
	reg  [175:0] cfg_regs_s = 176'd0;
	reg  cfg_valid_s = 1'b0;
	wire frame_start, cmd_strobe;
	wire [4:0] rd_slot;

	rhd2164x2_spi_slave_model slave (
		.rstb(rstb), .csb(csb), .sck(sck), .mosi(mosi), .miso0(miso0), .miso1(miso1),
		.data_tx_a0(dtx_a0), .data_tx_b0(dtx_b0), .data_tx_a1(dtx_a1), .data_tx_b1(dtx_b1),
		.data_rx(data_rx), .tx_load(tx_load), .word_end(word_end), .done());

	rhd2164_model_reg_ram reg_ram0 (
		.rstb(rstb), .sck(sck), .cmd_code(data_rx[15:14]), .cmd_register(data_rx[13:8]),
		.cmd_data(data_rx[7:0]), .dout0(dout0), .dout1(dout1), .dh(dh));

	emu_answer dut (
		.rstb(rstb), .sck(sck), .data_rx(data_rx), .tx_load(tx_load), .word_end(word_end),
		.dout0(dout0), .dout1(dout1), .dh(dh),
		.rd_a0(rd_a0), .rd_b0(rd_b0), .rd_a1(rd_a1), .rd_b1(rd_b1), .rd_valid(rd_valid),
		.cfg_regs_s(cfg_regs_s), .cfg_valid_s(cfg_valid_s),
		.data_tx_a0(dtx_a0), .data_tx_b0(dtx_b0), .data_tx_a1(dtx_a1), .data_tx_b1(dtx_b1),
		.frame_start(frame_start), .rd_slot(rd_slot), .cmd_strobe(cmd_strobe));

	// ---------------------------------------------------------------- reference model
	bit [15:0] r_cmd [int];
	bit        r_fs [int];
	bit [4:0]  r_slot [int];
	int        r_fr [int];
	bit        s_valid [int], s_cfgv [int];
	bit [175:0] s_cfg [int];
	bit        skip [int];
	bit [4:0]  m_last = 5'd31;
	bit        m_fsprev = 1'b0;
	int        m_fr = 0;

	task automatic store_settings(input int idx);
		s_valid[idx] = rd_valid; s_cfgv[idx] = cfg_valid_s; s_cfg[idx] = cfg_regs_s;
	endtask

	// reset: idx is the last word before it (-1 at start)
	task automatic model_reset(input int idx);
		r_cmd[idx] = READ(6'd63); r_fs[idx] = 1'b0; r_slot[idx] = 5'd0; r_fr[idx] = 0;
		m_last = 5'd31; m_fsprev = 1'b0; m_fr = 0;
		skip[idx + 1] = 1'b1;
		store_settings(idx + 1);
	endtask

	task automatic model_word(input int w, input [15:0] c);
		bit fs; bit [4:0] slot;
		fs = 1'b0; slot = 5'd0;
		if (c[15:14] == 2'b00) begin
			if (c[13] == 1'b0) slot = c[12:8];
			else if (c[13:8] == 6'd63) slot = m_last + 5'd1;
			fs = (c[13:8] == 6'd0) || (c[13:8] == 6'd63 && m_last == 5'd31);
			if (c[13] == 1'b0) m_last = c[12:8];
			else if (c[13:8] == 6'd63) m_last = m_last + 5'd1;
		end
		if (fs && !m_fsprev) m_fr++;
		m_fsprev = fs;
		r_cmd[w] = c; r_fs[w] = fs; r_slot[w] = slot; r_fr[w] = m_fr;
	endtask

	int n_ans = 0, n_data = 0, n_sent = 0, n_auxc = 0, n_vdd = 0, n_cfg = 0, n_cfg0 = 0,
	    n_rom = 0, n_wr = 0, n_cal = 0, n_63 = 0, n_fs = 0;

	function automatic [15:0] expect_ans(input int n, input [1:0] lane);
		bit [15:0] c; bit [5:0] C;
		c = r_cmd[n]; C = c[13:8];
		case (c[15:14])
			2'b00: if (C < 6'd32 || C == 6'd63)
					expect_ans = s_valid[n + 1] ? (sval(r_fr[n], r_slot[n], lane) ^ 16'h8000) : 16'h0000;
				else if (C == 6'd48) expect_ans = 16'd44100;
				else expect_ans = 16'h0000;
			2'b11: if (C <= 6'd21)
					expect_ans = s_cfgv[n + 1] ? {8'h00, s_cfg[n + 1][8 * C +: 8]} : 16'h0000;
				else expect_ans = {8'h00, rom(C, lane[0])};
			2'b10: expect_ans = {8'hFF, c[7:0]};
			default: expect_ans = 16'h0000;		// CALIBRATE, CLEAR (only these are sent)
		endcase
	endfunction

	task automatic check_answer(input int n, input [15:0] a0, input [15:0] b0, input [15:0] a1, input [15:0] b1);
		bit [15:0] got [4]; bit [15:0] c; bit bad;
		got[0] = a0; got[1] = b0; got[2] = a1; got[3] = b1;
		c = r_cmd[n]; bad = 1'b0;
		for (int l = 0; l < 4; l++)
			if (got[l] !== expect_ans(n, l)) bad = 1'b1;
		if (bad)
			err($sformatf("A answer to word %0d (cmd %h, slot %0d, frame %0d, rd_valid %b, cfg_valid %b): got %h %h %h %h, want %h %h %h %h",
				n, c, r_slot[n], r_fr[n], s_valid[n + 1], s_cfgv[n + 1], a0, b0, a1, b1,
				expect_ans(n, 0), expect_ans(n, 1), expect_ans(n, 2), expect_ans(n, 3)));
		n_ans++;
		case (c[15:14])
			2'b00: if (c[13:8] < 6'd32 || c[13:8] == 6'd63) begin
					if (s_valid[n + 1]) n_data++; else n_sent++;
					if (c[13:8] == 6'd63) n_63++;
				end
				else if (c[13:8] == 6'd48) n_vdd++;
				else n_auxc++;
			2'b11: if (c[13:8] <= 6'd21) begin if (s_cfgv[n + 1]) n_cfg++; else n_cfg0++; end else n_rom++;
			2'b10: n_wr++;
			default: n_cal++;
		endcase
	endtask

	// ---------------------------------------------------------------- settings (T5.3 rd_valid, T5.5 cfg)
	int p_inval = 0;		// % of words with rd_valid = 0
	int p_cfgchg = 0;		// per mille of words where cfg_regs_s changes
	int p_cfgvtog = 0;		// per mille of words where cfg_valid_s toggles
	task automatic choose_settings;
		rd_valid = ($urandom_range(0, 99) >= p_inval);
		if ($urandom_range(0, 999) < p_cfgchg)
			for (int i = 0; i < 22; i++) cfg_regs_s[8 * i +: 8] = $urandom;
		if ($urandom_range(0, 999) < p_cfgvtog) cfg_valid_s = !cfg_valid_s;
	endtask

	// at the end of each word (CS high gap; the next word's edge 1 is 2 ticks later)
	initial forever begin
		int w;
		@(bfm.word_done);
		w = bfm.word_no;
		model_word(w, bfm.done_cmd);
		choose_settings;
		store_settings(w + 1);
		if (r_cmd.exists(w - 2) && !skip.exists(w))
			check_answer(w - 2, bfm.a0, bfm.b0, bfm.a1, bfm.b1);
	end

	// ---------------------------------------------------------------- T5.3 stub and edge checks
	int wn = -1, e = 0, fr_s = 0;
	reg fs_d = 1'b0;
	always @(negedge csb) begin wn = wn + 1; e = 0; end

	always @(negedge rstb or posedge sck)
		if (!rstb) begin
			fs_d <= 1'b0; fr_s = 0;
			rd_a0 <= 0; rd_b0 <= 0; rd_a1 <= 0; rd_b1 <= 0;
		end
		else begin
			int pw; bit [15:0] c; bit isdata, is63;
			e = e + 1;
			pw = wn - 1;			// data_rx holds word pw's command
			if (frame_start && !fs_d) begin
				fr_s++; n_fs++;
				if (e != 1) err($sformatf("T1 frame_start rose at edge %0d of word %0d (after cmd %h)", e, wn,
					r_cmd.exists(pw) ? r_cmd[pw] : 16'hxxxx));
			end
			if (r_cmd.exists(pw)) begin
				c = r_cmd[pw];
				isdata = (c[15:14] == 2'b00) && (c[13] == 1'b0 || c[13:8] == 6'd63);
				is63 = (c[15:14] == 2'b00) && (c[13:8] == 6'd63);
				if ((e <= 2 || !is63) && frame_start !== r_fs[pw])
					err($sformatf("F1 frame_start %b at edge %0d of word %0d, want %b (cmd %h)", frame_start, e, wn, r_fs[pw], c));
				if (isdata && (e <= 2 || !is63) && rd_slot !== r_slot[pw])
					err($sformatf("F2 rd_slot %0d at edge %0d of word %0d, want %0d (cmd %h)", rd_slot, e, wn, r_slot[pw], c));
			end
			if (cmd_strobe !== (e == 2))
				err($sformatf("F3 cmd_strobe %b at edge %0d of word %0d", cmd_strobe, e, wn));
			fs_d <= frame_start;
			rd_a0 <= sval(fr_s, rd_slot, 2'd0);
			rd_b0 <= sval(fr_s, rd_slot, 2'd1);
			rd_a1 <= sval(fr_s, rd_slot, 2'd2);
			rd_b1 <= sval(fr_s, rd_slot, 2'd3);
		end

	// ---------------------------------------------------------------- stimulus
	task automatic do_reset;
		#1;								// let the last word_done handler run first
		model_reset(bfm.word_no);		// the last word before the reset
		rstb = 1'b0;
		#200;
		rstb = 1'b1;
		#20;
		if (frame_start !== 1'b0) err($sformatf("R1 frame_start %b after reset, before any word", frame_start));
		#200;
	endtask

	task automatic rand_words(input int n);
		for (int i = 0; i < n; i++) begin
			int k; bit [15:0] c;
			k = $urandom_range(0, 99);
			if (k < 35)      c = CONVERT($urandom_range(0, 31));
			else if (k < 42) c = CONVERT(0);
			else if (k < 55) c = CONVERT(63);
			else if (k < 65) c = CONVERT($urandom_range(32, 62));
			else if (k < 80) c = READ($urandom_range(0, 63));
			else if (k < 93) c = WRITE($urandom_range(0, 63), $urandom);
			else if (k < 97) c = CALIBRATE;
			else             c = CLEAR;
			bfm.word(c);
		end
	endtask

	task automatic sweeps63(input int n);			// CONVERT(63) only
		for (int i = 0; i < n; i++) bfm.word(CONVERT(63));
	endtask

	task automatic mixed63(input int n);			// 63 runs broken by other words
		for (int i = 0; i < n; i++) begin
			int k;
			k = $urandom_range(0, 9);
			if (k < 6) bfm.word(CONVERT(63));
			else if (k == 6) bfm.word(CONVERT($urandom_range(0, 31)));
			else if (k == 7) bfm.word(CONVERT($urandom_range(32, 34)));
			else if (k == 8) bfm.word(READ($urandom_range(0, 63)));
			else bfm.word(WRITE($urandom_range(0, 21), $urandom));
		end
	endtask

	int seed = 1, nrand = 20000;
	task automatic report(input string s);
		$display("%t %s done: %0d errors so far", $realtime, s, errors);
	endtask

	initial begin
		if ($value$plusargs("seed=%d", seed)) ;
		if ($value$plusargs("nrand=%d", nrand)) ;
		void'($urandom(seed));
		$display("emu_answer_tb: seed %0d, %0d random words", seed, nrand);

		model_reset(-1);
		#100 rstb = 1'b1;
		#20 if (frame_start !== 1'b0) err($sformatf("R1 frame_start %b after reset, before any word", frame_start));
		#200;

		for (int i = 0; i <= 21; i++) bfm.host_regs[i] = $urandom;
		bfm.bank = 1'b0; bfm.build_lists;

		// RHX bank 0 (detection run), no CONFIG yet: READ(0..17) must read 0 (R4)
		bfm.run_periods(130); report("RHX bank 0, no CONFIG");
		// CONFIG arrives
		for (int i = 0; i < 22; i++) cfg_regs_s[8 * i +: 8] = $urandom;
		cfg_valid_s = 1'b1;
		bfm.run_periods(130); report("RHX bank 0, CONFIG");
		// streaming: bank 1, link drops, CONFIG updates
		bfm.bank = 1'b1; bfm.build_lists;
		p_inval = 10; p_cfgchg = 10;
		bfm.run_periods(130); report("RHX bank 1, rd_valid and cfg changing");
		bfm.delay = 0;
		bfm.run_periods(20); report("RHX bank 1, cable delay 0");
		bfm.delay = 1;

		// random command words
		p_inval = 20; p_cfgchg = 20; p_cfgvtog = 5;
		rand_words(nrand); report("random words");

		// CONVERT(63)
		p_inval = 5; p_cfgvtog = 0;
		sweeps63(200); report("CONVERT(63) sweeps");
		mixed63(3000); report("CONVERT(63) mixed");

		// resets mid stream (SCLK stopped)
		do_reset; sweeps63(70); report("reset, then CONVERT(63)");
		do_reset; bfm.run_periods(10); rand_words(500); report("reset, then RHX and random");
		do_reset; rand_words(200); report("reset, then random");

		$display("answers checked: %0d (data %0d, sentinel %0d, of which CONVERT(63) %0d; aux 0 %0d, VDD %0d; READ cfg %0d, READ before CONFIG %0d, ROM %0d; WRITE %0d; CAL/CLEAR %0d); frame_starts %0d",
			n_ans, n_data, n_sent, n_63, n_auxc, n_vdd, n_cfg, n_cfg0, n_rom, n_wr, n_cal, n_fs);
		err_summary;
		if (errors == 0) $display("emu_answer_tb: PASS");
		else $display("emu_answer_tb: FAIL, %0d errors", errors);
		$finish;
	end

	initial begin #500ms; err("timeout"); err_summary; $display("emu_answer_tb: FAIL"); $finish; end
endmodule
