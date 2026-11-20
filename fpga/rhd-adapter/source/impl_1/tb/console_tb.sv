//testbench
//
//	Testbench for the console as the top level builds it (kuntur-notes
//	docs/interfaces/companion-console.md §8.9, requirements U3-U10 of §8):
//
//	  clkin ─► console_por ─► con_rst_n ─┬─► pat_monitor (wclk) ─► pat_blocks, pat_errors ─┐
//	                                     └─► console #(DIV, TIMEOUT) ◄── lock_sync, cdr_rdy, ┘
//	                                           cdr_valid, phase, eye_center, eye_width
//	                                           ─► con_tx ─► UART decoder ─► lines file
//
//	The wiring is the top level's (intan_rec_adapter, W4). DIV = 360,000
//	(30 ms "seconds"; a stale line starts TIMEOUT = 10 ms after its tick
//	and takes up to 15 ms, so 20 ms would overlap); TIMEOUT the real
//	120,000. clkin 12 MHz; wclk 9.4875 MHz (the 25k build) with +-2 ns
//	jitter, stopped and restarted between cycles. The wclk inputs change on
//	the falling edge of wclk: phase a random walk 0..335, eye_center and
//	eye_width new random values on every edge (so a record mixing two wclk
//	edges is caught), cdr_rdy/cdr_valid per scenario; pat_monitor is fed by
//	a K4.2 block source (A5 5A A5 5A c_lo c_hi 01 80) with random block
//	errors, through cdr_valid like the aligner.
//
//	For every snap_req toggle (probed in console) the tb records the seven
//	wclk inputs as sampled by the 1st..6th wclk edge after it (the shadow
//	copies on the 3rd, §8.7 S2).
//
//	Checks in the tb (at the pin)
//	  1. U3 UART: con_tx never X/Z, 1 from time 0 (through the power-on
//	     reset) and between bytes; every byte 8N1 with its stop bit 1; every
//	     edge of con_tx on the bit grid of its byte (104 clkin clocks per
//	     bit from the start edge, +-0.5 ns).
//	  2. Lines: every byte belongs to a line that starts with 'C' and ends
//	     in \r\n, <= 200 bytes (U4).
//	  3. U4 one line per tick: line k starts after tick k (the tb counts
//	     DIV clkin clocks from the con_rst_n release), 3..TIMEOUT+300
//	     clocks after it, and ends before tick k+1; no tick without a line.
//	     The start latency is the same for every line of a kind (spread <= 6
//	     clocks with wclk running, <= 2 with wclk stopped), which catches a
//	     wrong DIV passed down.
//	  4. snap_req toggles only within 3 clocks after a tick.
//	  5. The lines go to a file (+out=<path>, default console_lines.txt),
//	     one row per line with the expected t, lock and snap (ok / stale /
//	     any near the 10 ms boundary) and the recorded candidates, for
//	     check_console.py, which parses each line with
//	     companion_console.parse_line (the Tunki parser) and checks U5-U9:
//	     ok -> the record is one wclk edge's values, the 3rd or 4th edge
//	     after the request; stale -> the last good record (zeros before the
//	     first, N9); t, lock, eye (U7), D6 fields, saturated values printed.
//
//	Scenario (k = tick number; each "second" 30 ms):
//	  1-2  power-up, wclk absent: stale zeros, lock 0 (1 after the
//	       capture of 1, before its lock field); wclk starts
//	       4 ms into second 2 (an earlier request answered, N3)
//	  3-4  sweeping (rdy 0): eye ---/---    5  gave up (rdy 1, valid 0): 0/0
//	  6-   locked: rx rising, crc from random block errors
//	  8    rdy 0, valid 1: ---/---
//	  10   wclk stopped 1 ms before the tick for 4 ms: ok, late copy
//	  12-13 wclk stopped over two ticks: stale, stale, ok
//	  15   wclk back after the 10 ms window: stale, late ack; 16 ok
//	  17   wclk back right at the window end: either
//	  19-20 lock_sync 0 (changed 1 ms after the tick, after the capture)
//	  21-28 random: rdy/valid, lock, wclk stops
//	  29-  pat counters set near FFFFFFFF, every block an error (both
//	       saturate), then t (saturates)
//
//	Plusargs: +seed=<n>, +out=<path>.
//	Mutants: none in the tb; the run line below makes them from the RTL
//	  (wiring faults of console), each must FAIL in the tb or the checker.
//
//	Run (from this directory):
//	  tr '\r' '\n' < ../intan_rec_adapter.v | awk '/^module (console|console_por|console_timer|console_snap|console_shadow|console_fmt|bin2dec|console_uart|pat_monitor)[ #(]/,/^endmodule/' > /tmp/con_rtl.v
//	  iverilog -g2012 -Wall -o con.vvp console_tb.sv /tmp/con_rtl.v && vvp con.vvp && python3 check_console.py console_lines.txt
//	  mutants (k = 1..8; see check_console.py --mutants for the list):
//	  for k in 1 2 3 4 5 6 7 8; do python3 check_console.py --mutate $k /tmp/con_rtl.v /tmp/con_mut.v && iverilog -g2012 -o con.vvp console_tb.sv /tmp/con_mut.v && vvp con.vvp | grep console_tb: ; python3 check_console.py console_lines.txt | tail -1; done
//	Expected: "console_tb: ALL PASS" and "check_console: ALL PASS" for
//	  the RTL; for every mutant at least one FAIL.

`timescale 1ns/1ps

`ifndef LATTICE_PRIMS
// FD1P3DX for simulation (pins as console_por_tb's model). GSR stand-in:
// Q = 0 at 1 ps, not at t = 0, so con_rst_n falls after every RTL process
// is waiting on it and the asynchronous resets act; on the chip the
// configuration's wake-up GSR sets every console flop to that value.
module FD1P3DX #(parameter GSR = "ENABLED") (input wire D, input wire SP,
		input wire CK, input wire CD, output reg Q);
	initial #0.001 Q = 1'b0;
	always @(posedge CK or posedge CD)
		if (CD) Q <= 1'b0;
		else if (SP) Q <= D;
endmodule
`endif

module console_tb;
	localparam real TH = 41.667;			// clkin half period at 1 ps precision
	localparam real T = 2 * TH;			// clkin period, ns (12 MHz)
	localparam real WH = 1000.0 / 9.4875 / 2.0;	// wclk half period, ns
	localparam integer DIV = 360_000;
	localparam integer TO = 120_000;
	localparam integer BIT = 104;
	localparam real TB = BIT * T;			// bit time, ns
	localparam integer NTICKS = 35;

	integer seed = 1;
	reg [8*256-1:0] outname;
	integer fd;
	integer fails = 0;

	task fail(input [8*120-1:0] msg);
		begin
			fails = fails + 1;
			if (fails <= 30) $display("FAIL  %0s  (t=%0.3f ms, tick %0d)", msg, $realtime / 1e6, ntick);
		end
	endtask

	// ---------------- clocks ----------------
	reg clkin = 0;
	always #(TH) clkin = ~clkin;

	reg wclk = 0, wclk_en = 0;
	real wclk_chg_t = 0.0;
	always begin
		if (wclk_en) begin
			#(WH + ({$random(seed)} % 400) / 100.0 - 2.0) wclk = 1;
			#(WH + ({$random(seed)} % 400) / 100.0 - 2.0) wclk = 0;
		end else @(wclk_en);
	end
	always @(wclk_en) wclk_chg_t = $realtime;

	// ---------------- wclk-domain sources ----------------
	reg rdy_s = 0, valid_s = 0;		// scenario requests, applied on wclk
	reg cdr_rdy = 0, cdr_valid = 0;
	reg [8:0] phase = 0, eye_center = 0, eye_width = 0;
	reg [7:0] din = 0;
	reg [2:0] pos = 0;
	reg lock_sync = 0;
	integer err_div = 4096;
	integer sp = 7, eb = 0, pr;
	reg [15:0] sc = 0;
	reg berr = 0;

	function [7:0] k42(input [2:0] k, input [15:0] c);
		case (k)
			0, 2: k42 = 8'hA5;
			1, 3: k42 = 8'h5A;
			4: k42 = c[7:0];
			5: k42 = c[15:8];
			6: k42 = 8'h01;
			default: k42 = 8'h80;
		endcase
	endfunction

	always @(negedge wclk) begin
		cdr_rdy <= rdy_s;
		cdr_valid <= valid_s;
		pr = {$random(seed)} % 8;
		if (pr == 0) phase <= (phase == 335) ? 9'd0 : phase + 9'd1;
		else if (pr == 1) phase <= (phase == 0) ? 9'd335 : phase - 9'd1;
		eye_center <= {$random(seed)} % 336;
		eye_width <= {$random(seed)} % 337;
		sp = (sp + 1) % 8;
		if (sp == 0) begin
			sc = sc + 16'd1;
			berr = ({$random(seed)} % err_div) == 0;
			eb = {$random(seed)} % 8;
			// every block an error (saturation): fixed bytes only; equal
			// count-byte flips in a row form a valid count run, which
			// pat_monitor rightly resyncs to
			if (err_div == 1 && (eb == 4 || eb == 5)) eb = eb - 4;
		end
		din <= k42(sp, sc) ^ ((berr && sp == eb) ? 8'h10 : 8'h00);
		pos <= sp;
	end

	// ---------------- DUT: the top level's console part ----------------
	wire con_rst_n, con_tx;
	wire [31:0] pat_blocks, pat_errors;

	console_por console_por_0 (.clk(clkin), .con_rst_n(con_rst_n));

	pat_monitor pat_monitor_0 (.rst_n(con_rst_n), .wclk(wclk), .valid(cdr_valid),
		.din(din), .pos(pos), .pat_blocks(pat_blocks), .pat_errors(pat_errors));

	console #(.DIV(DIV), .TIMEOUT(TO), .MODE_PATTERN(1'b1)) console0 (
		.clkin(clkin), .wclk(wclk), .con_rst_n(con_rst_n), .lock_sync(lock_sync),
		.cdr_rdy(cdr_rdy), .cdr_valid(cdr_valid), .phase(phase),
		.eye_center(eye_center), .eye_width(eye_width),
		.pat_blocks(pat_blocks), .pat_errors(pat_errors), .con_tx(con_tx));

	// ---------------- tick model ----------------
	integer ce = 0, ntick = 0;
	real tick_t [0:63];
	reg [31:0] t_at [0:63];
	reg lock_at [0:63];
	reg [31:0] mt = 0;
	always @(posedge clkin)
		if (con_rst_n === 1'b1) begin
			ce = ce + 1;
			if (ce % DIV == 0) begin
				ntick = ntick + 1;
				tick_t[ntick] = $realtime;
				if (mt != 32'hFFFFFFFF) mt = mt + 1;
				t_at[ntick] = mt;
				lock_at[ntick] = lock_sync;
			end
		end

	// ---------------- snap_req toggles and the candidate records ----------------
	integer ntog = 0, cap_left = 0;
	reg [92:0] cand [1:63][0:5];
	real cand_t [1:63][0:5];
	integer cand_n [1:63];
	always @(console0.snap_req)
		if (con_rst_n === 1'b1) begin
			ntog = ntog + 1;
			cand_n[ntog] = 0;
			cap_left = 6;
			if (ntick == 0 || $realtime - tick_t[ntick] > 3.5 * T)
				fail("snap_req changed other than just after a tick (N2)");
		end
	// sampled before the edge's own updates: what the shadow's flops take
	always @(posedge wclk)
		if (cap_left > 0) begin
			cand[ntog][cand_n[ntog]] = {cdr_rdy, cdr_valid, phase, eye_center, eye_width, pat_blocks, pat_errors};
			cand_t[ntog][cand_n[ntog]] = $realtime;
			cand_n[ntog] = cand_n[ntog] + 1;
			cap_left = cap_left - 1;
		end

	// ---------------- con_tx level checks ----------------
	initial begin
		#0.01;
		if (con_tx !== 1'b1) fail("con_tx not 1 at time 0 (U3, power-on hold)");
	end
	always @(con_tx)
		if ($realtime > 0.005 && con_tx !== 1'b0 && con_tx !== 1'b1) fail("con_tx X or Z");

	// ---------------- UART decoder ----------------
	real bs = -1.0e9;			// start edge of the byte in progress
	reg inbyte = 0;
	always @(con_tx)
		if (inbyte && $realtime > bs + 0.01) begin
			// edges only on the byte's bit grid
			if ((($realtime - bs) / TB) - $floor(($realtime - bs) / TB + 0.5) > 0.5 / TB
			    || (($realtime - bs) / TB) - $floor(($realtime - bs) / TB + 0.5) < -0.5 / TB)
				fail("con_tx edge off the 104-clock bit grid (U3)");
		end

	reg [7:0] lb [0:255];
	integer ln = 0, nlines = 0, i;
	reg inline_ = 0;
	real ls, le, lprev_end = -1.0;
	integer lk, ltog;
	real L, fast_min = 1e30, fast_max = -1e30, nw_min = 1e30, nw_max = -1e30;
	integer nfast = 0, nnw = 0, nok = 0, nstale = 0, nany = 0;
	reg [7:0] b;
	reg [9:0] fr;
	reg lclass_fast, lclass_nw;

	always begin
		@(negedge con_tx);
		if (con_rst_n !== 1'b1 && $realtime > 0.01) fail("start bit while con_rst_n = 0");
		bs = $realtime;
		inbyte = 1;
		if (!inline_) begin
			// a new line: which tick, which request, what kind
			inline_ = 1;
			ln = 0;
			ls = bs;
			lk = ntick;
			ltog = ntog;
			nlines = nlines + 1;
			if (lk != nlines) fail("line not one per tick (U4): line number != ticks so far");
			if (lk >= 1) begin
				L = (ls - tick_t[lk]) / T;
				if (L < 3 || L > TO + 300) fail("line start not 3..TIMEOUT+300 clocks after its tick (U4)");
				lclass_fast = wclk_en && wclk_chg_t < tick_t[lk] - 10000.0;
				lclass_nw = !wclk_en && wclk_chg_t < tick_t[lk] - 1000.0;
				if (lclass_fast) begin
					nfast = nfast + 1;
					if (L < fast_min) fast_min = L;
					if (L > fast_max) fast_max = L;
				end
				if (lclass_nw) begin
					nnw = nnw + 1;
					if (L < nw_min) nw_min = L;
					if (L > nw_max) nw_max = L;
				end
			end
			if (lprev_end > ls) fail("line starts before the previous one ended (U4)");
		end
		for (i = 0; i < 10; i = i + 1) begin
			#((i == 0 ? 0.5 : 1.0) * TB);
			fr[i] = con_tx;
		end
		inbyte = 0;
		if (fr[0] !== 1'b0) fail("start bit not 0 at its centre (U3)");
		if (fr[9] !== 1'b1) fail("stop bit not 1 (U3)");
		b = fr[8:1];
		if (ln < 256) lb[ln] = b;
		ln = ln + 1;
		if (ln == 1 && b != "C") fail("line does not start with 'C'");
		if (b == 8'h0A) begin
			le = bs + 10 * TB;
			lprev_end = le;
			inline_ = 0;
			if (ln < 2 || lb[ln - 2] != 8'h0D) fail("line does not end in \\r\\n (D7)");
			if (ln > 200) fail("line longer than 200 bytes (U4)");
			if (lk >= 1 && le > tick_t[lk] + DIV * T) fail("line not ended before the next tick (U4)");
			write_row;
		end else if (ln > 250) begin
			fail("no \\n within 250 bytes");
			inline_ = 0;
		end
	end

	// expected snap from the copy (3rd edge) time; "any" within 1 us of the window end
	task write_row;
		reg [8*6-1:0] exp;
		real c3, wend;
		integer j;
		begin
			wend = tick_t[lk] + TO * T;
			if (lk < 1 || ltog < 1) exp = "any";
			else if (cand_n[ltog] < 3) exp = (le > wend + 1000.0) ? "stale" : "any";
			else begin
				c3 = cand_t[ltog][2];
				if (c3 >= tick_t[lk] - 1000.0 && c3 + 1000.0 <= wend) exp = "ok";
				else if (c3 > wend + 1000.0) exp = "stale";
				else exp = "any";
			end
			if (exp == "ok") nok = nok + 1; else if (exp == "stale") nstale = nstale + 1; else nany = nany + 1;
			$fwrite(fd, "%0d %0d %0d %0s", lk, t_at[lk], lock_at[lk], exp);
			for (j = 0; j < 6; j = j + 1)
				if (ltog >= 1 && j < cand_n[ltog])
					$fwrite(fd, " %0d,%0d,%0d,%0d,%0d,%0d,%0d", cand[ltog][j][92], cand[ltog][j][91],
						cand[ltog][j][90:82], cand[ltog][j][81:73], cand[ltog][j][72:64],
						cand[ltog][j][63:32], cand[ltog][j][31:0]);
				else $fwrite(fd, " -");
			$fwrite(fd, " | ");
			for (j = 0; j < ln && j < 256; j = j + 1)
				if (lb[j] == 8'h0D) $fwrite(fd, "\\r");
				else if (lb[j] == 8'h0A) $fwrite(fd, "\\n");
				else if (lb[j] < 8'h20 || lb[j] > 8'h7E) $fwrite(fd, "<%02X>", lb[j]);
				else $fwrite(fd, "%c", lb[j]);
			$fwrite(fd, "\n");
		end
	endtask

	// ---------------- scenario ----------------
	task automatic at(input integer k, input real ms);
		real target;
		begin
			wait (ntick >= k);
			target = tick_t[k] + ms * 1.0e6;
			if ($realtime < target) #(target - $realtime);
		end
	endtask

	integer k, r;
	real off, len;

	// lock_sync changes only after that second's capture, so the line must
	// show the value at the tick: 10.5 ms into second 1 (stale, captured at
	// 10 ms) and 1 ms into 19 and 20 (wclk running, captured within 1 us),
	// both before the line sends its lock field (~1.7 ms after its start),
	// so a console_fmt reading lock_sync live is caught; 11..28 ms in 21-28
	integer kl;
	initial begin
		at(1, 10.5); lock_sync = 1;
		at(19, 1.0); lock_sync = 0;
		at(20, 1.0); lock_sync = 1;
		for (kl = 21; kl <= 28; kl = kl + 1) begin
			at(kl, 11.0 + ({$random(seed)} % 1700) / 100.0);
			if ({$random(seed)} % 2) lock_sync = ~lock_sync;
		end
		at(29, 12); lock_sync = 1;
	end

	initial begin
		if (!$value$plusargs("seed=%d", seed)) seed = 1;
		if (!$value$plusargs("out=%s", outname)) outname = "console_lines.txt";
		fd = $fopen(outname, "w");
		r = $random(seed);

		// 1-2: power-up without wclk; wclk 4 ms into second 2
		at(2, 4); wclk_en = 1;
		// 3-4 sweeping; 5 gave up; 6- locked
		at(4, 12); rdy_s = 1;
		at(5, 12); valid_s = 1;
		// 8: rdy 0, valid 1
		at(7, 12); rdy_s = 0;
		at(8, 12); rdy_s = 1;
		// 10: stopped across the tick, back within the window
		at(9, 29); wclk_en = 0;
		at(10, 3); wclk_en = 1;
		// 12-13: stopped over two ticks, back 3 ms into 13
		at(11, 12); wclk_en = 0;
		at(13, 3); wclk_en = 1;
		// 15: back after the window (late ack), 16 fresh
		at(14, 29.5); wclk_en = 0;
		at(15, 12); wclk_en = 1;
		// 17: back right at the window end
		at(16, 29.9); wclk_en = 0;
		at(17, TO * T / 1.0e6 - 0.0003); wclk_en = 1;
		// 21-28: random
		for (k = 21; k <= 28; k = k + 1) begin
			at(k, 0.5);
			case ({$random(seed)} % 6)
				0: begin rdy_s = 0; valid_s = 0; end
				1: begin rdy_s = 1; valid_s = 0; end
				2: begin rdy_s = 0; valid_s = 1; end
				default: begin rdy_s = 1; valid_s = 1; end
			endcase
			if ({$random(seed)} % 2) begin
				off = ({$random(seed)} % 2900) / 100.0;
				len = ({$random(seed)} % 4000) / 100.0;
				at(k, off); wclk_en = 0;
				#(len * 1.0e6); wclk_en = 1;
			end
		end
		// 29-: saturation
		// (pat_errors counts only with pat_blocks, so errors <= blocks holds
		// from reset and errors needs no saturation of its own: set both
		// equal and make every block an error, so both reach FFFFFFFF)
		at(29, 12); wclk_en = 1; rdy_s = 1; valid_s = 1; err_div = 1;
		repeat (32) @(negedge wclk);	// blocks made before err_div = 1 judged
		#1;
		pat_monitor_0.pat_blocks = 32'hFFFFFFFF - 60000;
		pat_monitor_0.pat_errors = 32'hFFFFFFFF - 60000;
		at(31, 12);
		console0.console_timer_0.t = 32'hFFFFFFFD; mt = 32'hFFFFFFFD;
		at(NTICKS, 29);

		if (nlines != NTICKS) fail("not one line per tick by the end (U4)");
		if (nfast < 5) fail("too few lines with wclk running to judge the latency");
		else if (fast_max - fast_min > 6) fail("line latency varies with wclk running (DIV/TIMEOUT wiring?)");
		if (nnw < 2) fail("too few lines without wclk to judge the latency");
		else if (nw_max - nw_min > 2) fail("stale line latency varies (TIMEOUT wiring?)");
		$fclose(fd);
		$display("      %0d lines (expected ok %0d, stale %0d, either %0d); latency wclk running %0.0f..%0.0f clocks (%0d), stopped %0.0f..%0.0f (%0d)",
			nlines, nok, nstale, nany, fast_min, fast_max, nfast, nw_min, nw_max, nnw);
		$display("      final rx %0d crc %0d, %0d snap_req toggles", pat_blocks, pat_errors, ntog);
		if (fails == 0) $display("console_tb: ALL PASS");
		else $display("console_tb: %0d FAIL", fails);
		$finish;
	end
endmodule
