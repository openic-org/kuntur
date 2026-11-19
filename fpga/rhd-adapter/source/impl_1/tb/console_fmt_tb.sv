//testbench
//
//	Testbench for the console's line formatter (console_fmt, with its
//	bin2dec). Contract: kuntur-notes docs/interfaces/companion-console.md
//	§8.3 (F0-F8) and §3; reference formatter: format_line() in
//	software/tunki/companion_console.py.
//
//	  go, record ─► console_fmt ─► uart_start, uart_data ─► UART model
//	                             ◄─ uart_rdy ◄──────────────┘
//	                             ─► busy
//
//	The UART model behaves like console_uart at byte level (rdy high when
//	idle; a start is taken on a clock edge with rdy; rdy falls on that
//	edge and stays low for the byte time). Byte time: 1,042 clocks for the
//	timed lines (F5), else random 1..40 clocks to keep the run short (any
//	gap between bytes is allowed). The tb drives inputs on the falling
//	edge and samples on the rising edge.
//
//	Checks
//	  1. F0/F1/F3/F4 every line byte for byte against the tb's own
//	     formatter: the §8.3 test vector (144 bytes, *5F); every decimal
//	     digit-count boundary in t, rx, crc (0, 10^k - 1, 10^k, FFFFFFFF);
//	     phase 0/335; the three eye cases (rdy 0 with valid 0 and 1; rdy 1
//	     valid 0; both 1, centre 0..335, width 0..336); lock 0/1; snap
//	     ok/stale; 300 random records; checksums with hex letters.
//	  2. F5 handshake: uart_start never high on an edge where uart_rdy is
//	     low; no byte outside a line.
//	  3. F5 time: two lines at the real byte time end within 50 ms of go.
//	  4. busy: high within 2 clocks of go, low within 2 clocks after the
//	     UART took the last byte (\n), never low while bytes remain.
//	  5. F6: go pulses while busy (several, random times) change nothing:
//	     the line is complete and right, and no second line follows.
//	  6. F7: an asynchronous reset mid-line: uart_start = 0 and busy = 0
//	     at once; no further byte of that line; the next go gives a full,
//	     right line.
//	  7. The lines are written to a file (+out=<path>, default
//	     console_fmt_lines.txt), one "inputs | line" per line, for
//	     check_console_fmt.py, which parses every line with
//	     companion_console.parse_line and compares it with the inputs.
//
//	Plusargs: +seed=<n>, +out=<path>.
//	Defines: +define+MODE_FRAMES builds with MODE_PATTERN = 0 (mode=frames).
//	  +define+REF_MODEL replaces the DUT by a model; +define+MUT=<k> breaks
//	  it (with REF_MODEL), each must FAIL:
//	    1 digits least significant first; 2 checksum includes '*';
//	    3 eye ignores cdr_rdy; 4 go while busy restarts the line;
//	    5 no wait after a start (a start while !rdy); 6 snap inverted;
//	    7 lowercase checksum; 8 leading zeros in t; 9 busy falls one byte
//	    early; 10 reset does not stop the line.
//
//	Run (from this directory):
//	  tr '\r' '\n' < ../intan_rec_adapter.v | awk '/^module (console_fmt|bin2dec)[ #(]/,/^endmodule/' > /tmp/cf_rtl.v
//	  iverilog -g2012 -Wall -o cf.vvp console_fmt_tb.sv /tmp/cf_rtl.v && vvp cf.vvp && python3 check_console_fmt.py console_fmt_lines.txt
//	  iverilog -g2012 -Wall -DREF_MODEL -o cf.vvp console_fmt_tb.sv && vvp cf.vvp && python3 check_console_fmt.py console_fmt_lines.txt
//	  for k in 1 2 3 4 5 6 7 8 9 10; do iverilog -g2012 -DREF_MODEL -DMUT=$k -o cf.vvp console_fmt_tb.sv && vvp cf.vvp | grep console_fmt_tb:; done
//	(console_fmt_ref.vh, the tb's own formatter, must be in this directory.)
//	Expected: "console_fmt_tb: ALL PASS"
//	 and "check_console_fmt: ALL PASS" for the RTL and the model; FAIL for
//	 every mutant.

`timescale 1ns/1ps

`ifdef MODE_FRAMES
	`define MODE_BIT 1'b0
`else
	`define MODE_BIT 1'b1
`endif
`ifndef MUT
	`define MUT 0
`endif

// The formatter (expected lines and the model): console_fmt_ref.vh,
// included in both modules.

`ifdef REF_MODEL
module bin2dec;	// not used by the model; keeps the RTL file list the same
endmodule
module console_fmt #(parameter MODE_PATTERN = 1'b1) (
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
	`include "console_fmt_ref.vh"
	integer idx;
	reg hold;	// one clock after a start, while the UART's rdy falls
	always @(negedge rst_n or posedge clk)
		if (!rst_n) begin
			if (`MUT != 10) begin uart_start <= 0; busy <= 0; idx <= 0; ll = 0; end
			hold <= 0;
		end else begin
			uart_start <= 0;
			hold <= 0;
			if (go && (!busy || `MUT == 4)) begin
				mkline(t, lock, snap_ok, cdr_rdy, cdr_valid, phase, eye_center, eye_width,
					pat_blocks, pat_errors, MODE_PATTERN, `MUT);
				idx <= 0; busy <= 1; hold <= 1;
			end else if (busy) begin
				if (idx < ll) begin
					if (uart_rdy && !uart_start && (!hold || `MUT == 5)) begin
						uart_start <= 1; uart_data <= lb[idx]; idx <= idx + 1; hold <= 1;
					end else if (`MUT == 5 && uart_start) begin
						uart_start <= 1; uart_data <= lb[idx]; idx <= idx + 1;
					end
					if (`MUT == 9 && idx == ll - 1 && uart_start) busy <= 0;
				end else if (!uart_start)
					busy <= 0;
			end
		end
endmodule
`endif

module console_fmt_tb;
	localparam real T = 1000.0 / 12.0;	// ns
	localparam REAL_BYTE = 1042;

	reg clk = 0, rst_n = 1, go = 0;
	reg [31:0] t = 0, rx = 0, crc = 0;
	reg lock = 0, snap_ok = 0, rdy = 0, valid = 0;
	reg [8:0] ph = 0, ec = 0, ew = 0;
	wire uart_start, busy;
	wire [7:0] uart_data;
	reg m_busy = 0;
	wire uart_rdy = !m_busy;
	integer m_cnt = 0, byte_time = 0;	// 0: random 1..40
	integer fails = 0, seed = 1, lines = 0, fd;
	reg [8*256-1:0] outname;

	console_fmt #(.MODE_PATTERN(`MODE_BIT)) dut (
		.clk(clk), .rst_n(rst_n), .go(go), .t(t), .lock(lock), .snap_ok(snap_ok),
		.cdr_rdy(rdy), .cdr_valid(valid), .phase(ph), .eye_center(ec), .eye_width(ew),
		.pat_blocks(rx), .pat_errors(crc), .uart_rdy(uart_rdy), .uart_start(uart_start),
		.uart_data(uart_data), .busy(busy));

	always #(T/2) clk = ~clk;

	`include "console_fmt_ref.vh"	// lb/ll: the expected line

	task fail(input [8*80-1:0] msg);
		begin fails = fails + 1; if (fails <= 25) $display("FAIL  %0s  (t=%0.3f us)", msg, $realtime/1000.0); end
	endtask

	// ---- UART model + byte capture --------------------------------------
	reg [7:0] gb [0:511];		// captured bytes of the current line
	integer gl = 0;
	integer nbytes_total = 0, clk_n = 0, last_take = -1;
	reg expect_bytes = 0;
	always @(posedge clk) begin
		clk_n <= clk_n + 1;
		if (!rst_n) begin
			m_busy <= 0;
		end else if (uart_start) begin
			if (!uart_rdy) fail("uart_start on an edge with uart_rdy low (F5)");
			else begin
				if (!expect_bytes) fail("byte sent outside a line");
				if (gl < 512) gb[gl] = uart_data;
				gl = gl + 1;
				nbytes_total = nbytes_total + 1;
				last_take = clk_n;
				m_busy <= 1;
				m_cnt <= byte_time ? byte_time : 1 + {$random(seed)} % 40;
			end
		end else if (m_busy) begin
			if (m_cnt <= 1) m_busy <= 0;
			m_cnt <= m_cnt - 1;
		end
	end

	// busy never low while the line is incomplete (checked on edges)
	reg checking_busy = 0;
	always @(posedge clk) if (rst_n && checking_busy && !busy && gl < ll && gl > 0)
		fail("busy low before the last byte was taken");

	// a captured line, escaped (\r, \n, <XX> for other non-printables)
	task write_got(input integer f);
		integer i;
		begin
			for (i = 0; i < gl && i < 512; i = i + 1)
				if (gb[i] == 8'h0D) $fwrite(f, "\\r");
				else if (gb[i] == 8'h0A) $fwrite(f, "\\n");
				else if (gb[i] < 8'h20 || gb[i] > 8'h7E) $fwrite(f, "<%02X>", gb[i]);
				else $fwrite(f, "%c", gb[i]);
		end
	endtask
	task write_want(input integer f);
		integer i;
		begin
			for (i = 0; i < ll; i = i + 1)
				if (lb[i] == 8'h0D) $fwrite(f, "\\r");
				else if (lb[i] == 8'h0A) $fwrite(f, "\\n");
				else $fwrite(f, "%c", lb[i]);
		end
	endtask
	function same;
		input dummy;
		integer i;
		begin
			same = (gl == ll);
			for (i = 0; i < ll && same; i = i + 1) if (gb[i] !== lb[i]) same = 0;
		end
	endfunction

	// ---- one line -----------------------------------------------------------
	// extra_go: number of go pulses to issue while busy (F6)
	task send_line(input integer extra_go);
		integer n, i, tgo, tend;
		integer go_at [0:7];
		begin
			mkline(t, lock, snap_ok, rdy, valid, ph, ec, ew, rx, crc, `MODE_BIT, 0);
			gl = 0;
			expect_bytes = 1;
			@(negedge clk) go = 1;
			tgo = clk_n;
			@(negedge clk) go = 0;
			n = 1;
			while (!busy && n < 3) begin @(negedge clk); n = n + 1; end
			if (!busy) fail("busy not high within 2 clocks of go");
			checking_busy = 1;
			for (i = 0; i < extra_go; i = i + 1) go_at[i] = 1 + {$random(seed)} % (ll * (byte_time ? byte_time : 20));
			n = 0;
			while (busy && n < 1_000_000) begin	// 83 ms: > F5 50 ms
				@(negedge clk); n = n + 1;
				go = 0;
				for (i = 0; i < extra_go; i = i + 1) if (n == go_at[i] && busy) go = 1;
			end
			go = 0;
			checking_busy = 0;
			tend = clk_n;
			if (busy) fail("busy never fell");
			if (last_take >= 0 && gl == ll && tend - last_take > 3)
				fail("busy fell more than 2 clocks after the last byte was taken");
			if (byte_time == REAL_BYTE && (tend - tgo) > 600_000) fail("line longer than 50 ms (F5)");
			// nothing more may follow (no second line from a go while busy)
			repeat ((byte_time ? byte_time : 40) * 3) @(negedge clk);
			expect_bytes = 0;
			if (!same(0)) begin
				fail("line differs from the reference");
				if (fails <= 25) begin
					$write("      want %0d: ", ll); write_want(32'h8000_0001); $write("\n");
					$write("      got  %0d: ", gl); write_got(32'h8000_0001); $write("\n");
				end
			end
			$fwrite(fd, "%0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d | ", t, lock, snap_ok, rdy, valid,
				ph, ec, ew, rx, crc, `MODE_BIT);
			write_got(fd); $fwrite(fd, "\n");
			lines = lines + 1;
		end
	endtask

	task rnd_record;
		begin
			t = $random(seed); rx = $random(seed); crc = $random(seed);
			case ({$random(seed)} % 4)	// shorter numbers too
				0: begin t = t >> ({$random(seed)} % 32); rx = rx >> ({$random(seed)} % 32); crc = crc >> ({$random(seed)} % 32); end
				1: begin crc = 0; end
				default: ;
			endcase
			lock = $random(seed); snap_ok = $random(seed);
			rdy = ({$random(seed)} % 4) != 0; valid = rdy ? ({$random(seed)} % 4 != 0) : $random(seed);
			ph = {$random(seed)} % 336; ec = {$random(seed)} % 336; ew = {$random(seed)} % 337;
		end
	endtask

	integer i, k;
	reg [63:0] p, q;
	initial begin
		if ($value$plusargs("seed=%d", seed)) ;
		if (!$value$plusargs("out=%s", outname)) outname = "console_fmt_lines.txt";
		fd = $fopen(outname, "w");
		#0.05 rst_n = 0;
		#(T * 3.3) rst_n = 1;
		#0.1 if (busy !== 1'b0 || uart_start !== 1'b0) fail("busy/uart_start not 0 after reset (F7)");
		repeat (5) @(negedge clk);

		// ---- 1. the §8.3 test vector --------------------------------------
		t = 10; lock = 0; ph = 32; rdy = 1; valid = 1; ec = 11; ew = 23; rx = 3; crc = 43; snap_ok = 0;
		send_line(0);
`ifndef MODE_FRAMES
		// independent of console_fmt_ref.vh: the bytes from format_line()
		if (gl != 144 || gb[0] != "C" || gb[6] != "1" || gb[7] != "0" || gb[138] != " " ||
				gb[139] != "*" || gb[140] != "5" || gb[141] != "F" || gb[142] != 8'h0D || gb[143] != 8'h0A)
			fail("s8.3 test vector (144 bytes, *5F)");
		else $display("      s8.3 test vector: 144 bytes, *5F");
`endif

		// ---- 1. digit-count boundaries ------------------------------------
		p = 1;
		for (k = 0; k <= 10; k = k + 1) begin
			for (i = 0; i <= 1; i = i + 1)	// p - 1, p  (0 for k = 0)
				if (p - i <= 64'hFFFFFFFF && !(k == 0 && i == 0)) begin
					q = p - i;
					t = q; rx = q; crc = q;
					lock = k % 2; snap_ok = !(k % 3 == 0); rdy = 1; valid = 1;
					ph = (k % 2) ? 335 : 0; ec = (k % 2) ? 0 : 335; ew = (k % 2) ? 336 : 0;
					send_line(0);
				end
			p = p * 10;
		end
		t = 32'hFFFFFFFF; rx = 32'hFFFFFFFF; crc = 32'hFFFFFFFF; send_line(0);
		$display("      digit boundaries: %0d lines", lines);

		// ---- 1. eye cases, lock, snap ---------------------------------------
		t = 7; rx = 100; crc = 0; ph = 5; ec = 200; ew = 150;
		for (i = 0; i < 16; i = i + 1) begin
			rdy = i[0]; valid = i[1]; lock = i[2]; snap_ok = i[3];
			send_line(0);
		end
		$display("      eye/lock/snap cases: %0d lines", lines);

		// ---- 1. random records ------------------------------------------------
		for (i = 0; i < 300; i = i + 1) begin rnd_record; send_line(0); end
		$display("      random records: %0d lines", lines);

		// ---- 5. go while busy ---------------------------------------------------
		for (i = 0; i < 10; i = i + 1) begin rnd_record; send_line(1 + i % 4); end
		$display("      go while busy: %0d lines", lines);

		// ---- 3. real byte time ---------------------------------------------------
		byte_time = REAL_BYTE;
		t = 32'hFFFFFFFF; rx = 32'hFFFFFFFF; crc = 32'hFFFFFFFF; ph = 335; ec = 335; ew = 336;
		rdy = 1; valid = 1; lock = 1; snap_ok = 0;
		send_line(0);
		rnd_record; send_line(2);
		byte_time = 0;
		$display("      real byte time: %0d lines", lines);

		// ---- 6. reset mid-line -----------------------------------------------------
		for (i = 0; i < 5; i = i + 1) begin
			rnd_record;
			expect_bytes = 1; gl = 0;
			@(negedge clk) go = 1;
			@(negedge clk) go = 0;
			repeat (20 + {$random(seed)} % 2000) @(negedge clk);
			#(T / 7) rst_n = 0;
			#0.1 if (busy !== 1'b0 || uart_start !== 1'b0) fail("busy/uart_start not 0 at once on reset (F7)");
			expect_bytes = 0;
			#(T * 3) rst_n = 1;
			repeat (300) @(negedge clk);	// any further byte of the old line fails here
			rnd_record; send_line(0);
		end
		$display("      reset mid-line: %0d lines", lines);

		$fclose(fd);
		if (fails == 0) $display("console_fmt_tb: ALL PASS (%0d lines, %0d bytes)", lines, nbytes_total);
		else $display("console_fmt_tb: %0d FAIL", fails);
		$finish;
	end
endmodule
