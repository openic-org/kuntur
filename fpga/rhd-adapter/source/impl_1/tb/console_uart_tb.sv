//testbench
//
//	Testbench for the companion console's byte UART (console_uart).
//	Contract: kuntur-notes docs/interfaces/companion-console.md §3 (D3)
//	and §8 U3: 115,200 8N1 from the 12 MHz clkin, divider 104, LSB first,
//	start bit 0, stop bit 1, the line idles high.
//
//	  clkin (12 MHz) ─┐
//	  rst_n ──────────┤ console_uart ─► con_tx (E20)
//	  start ──────────┤              ─► rdy (level: idle, a start is taken)
//	  data_tx[7:0] ───┘
//
//	Handshake assumed (the line formatter's side): data_tx is sampled on
//	the clkin edge where start && rdy; start while !rdy is ignored.
//
//	Checks
//	  1. Reset: con_tx = 1 and rdy = 1 while rst_n = 0 (asynchronously, no
//	     clkin edge needed) and after it, with no frame until a start.
//	  2. Frame shape, every byte: from the falling edge of the start bit,
//	     sampled on every clkin edge: start 0 for exactly 104 clocks, data
//	     bits LSB first for exactly 104 clocks each, then 1 for at least 104
//	     clocks (the stop bit) before any next start bit.
//	  3. Data: every value 0x00-0xFF, back to back (start on the first
//	     clock rdy is high) and again with random gaps; data_tx is changed
//	     to garbage right after start, so the byte must be the one sampled.
//	  4. Latency and rate: the start bit begins a fixed number of clocks
//	     after the accepting edge (the same for every byte, <= 2), and back
//	     to back a byte takes 10 x 104 clocks plus at most 2.
//	  5. rdy: low from the accepting edge until the stop bit's 104 clocks
//	     are under way (no start can cut a stop bit short; check 2); high
//	     whenever the line is idle.
//	  6. start while busy (any data) changes nothing: no extra frame, the
//	     byte in flight unchanged.
//	  7. Glitch-free: con_tx changes only on a clkin rising edge (or at
//	     rst_n falling).
//
//	Plusargs: +seed=<n>.
//	Define: +define+REF_MODEL replaces the DUT by a model written from the
//	  spec; +define+MUT=<k> breaks it (with REF_MODEL), each must FAIL:
//	    1 line idles low; 2 MSB first; 3 divider 103; 4 data not reloaded
//	    (sends the previous byte); 5 start taken while busy; 6 stop bit
//	    52 clocks.
//
//	Run (from this directory). The RTL is cut out of intan_rec_adapter.v
//	  by its module line:
//	  tr '\r' '\n' < ../intan_rec_adapter.v | awk '/^module console_uart\(/,/^endmodule/' > /tmp/cu_rtl.v
//	  iverilog -g2012 -Wall -o cu.vvp console_uart_tb.sv /tmp/cu_rtl.v && vvp cu.vvp
//	  iverilog -g2012 -Wall -DREF_MODEL -o cu.vvp console_uart_tb.sv && vvp cu.vvp
//	  for k in 1 2 3 4 5 6; do iverilog -g2012 -DREF_MODEL -DMUT=$k -o cu.vvp console_uart_tb.sv && vvp cu.vvp | grep console_uart_tb:; done
//	Expected: "console_uart_tb: ALL PASS" for the RTL and the model; FAIL
//	  for every mutant.

`timescale 1ns/1ps

`ifdef REF_MODEL
module console_uart (
	input  wire	clk,
	input  wire	rst_n,
	input  wire	start,
	input  wire	[7:0]	data_tx,
	output reg	con_tx,
	output wire	rdy
);
`ifndef MUT
	`define MUT 0
`endif
	localparam DIV = (`MUT == 3) ? 103 : 104;
	reg busy;
	reg [3:0] nbit;
	reg [7:0] cnt;
	reg [9:0] sh;
	reg [7:0] last;
	assign rdy = !busy || (`MUT == 5);
	always @(negedge rst_n or posedge clk)
		if (!rst_n) begin
			busy <= 0; con_tx <= (`MUT == 1) ? 1'b0 : 1'b1; nbit <= 0; cnt <= 0; sh <= 10'h3ff; last <= 0;
		end else if (start && rdy) begin
			busy <= 1; nbit <= 0; cnt <= 0;
			sh <= {1'b1, (`MUT == 4) ? last : ((`MUT == 2) ? {data_tx[0],data_tx[1],data_tx[2],data_tx[3],data_tx[4],data_tx[5],data_tx[6],data_tx[7]} : data_tx), 1'b0};
			last <= data_tx;
			con_tx <= (`MUT == 1) ? 1'b0 : 1'b1;
		end else if (busy) begin
			con_tx <= sh[0];
			if (cnt == ((`MUT == 6 && nbit == 9) ? 51 : DIV-1)) begin
				cnt <= 0; sh <= {1'b1, sh[9:1]};
				if (nbit == 9) begin busy <= 0; con_tx <= (`MUT == 1) ? 1'b0 : 1'b1; end
				nbit <= nbit + 1;
			end else
				cnt <= cnt + 1;
		end else
			con_tx <= (`MUT == 1) ? 1'b0 : 1'b1;
endmodule
`endif

module console_uart_tb;
	localparam real T = 1000.0 / 12.0;	// ns, 12 MHz
	localparam DIV = 104;

	reg clk = 0, rst_n = 1, start = 0;	// rst_n falls at 0.05 ns: an edge for the async reset
	reg [7:0] data_tx = 8'h00;
	wire con_tx, rdy;
	integer fails = 0, seed = 1;

	console_uart dut (.clk(clk), .rst_n(rst_n), .start(start), .data_tx(data_tx),
		.con_tx(con_tx), .rdy(rdy));

	always #(T/2) clk = ~clk;

	task fail(input [8*120-1:0] msg);
		begin fails = fails + 1; if (fails <= 20) $display("FAIL  %0s  (t=%0.3f us)", msg, $realtime/1000.0); end
	endtask

	// ---- 7. glitch-free ------------------------------------------------
	realtime last_pos = -1.0e9;
	always @(posedge clk) last_pos = $realtime;
	always @(con_tx) if (rst_n && $realtime - last_pos > 1.0 && $realtime > 0) fail("con_tx changed between clkin edges");

	// ---- the receiver: samples on every clkin edge ----------------------
	reg [7:0] exp_q [0:1023];
	integer exp_wr = 0, exp_rd = 0;
	integer accept_clk [0:1023];
	integer clk_n = 0;			// clkin edges since reset release
	integer frames = 0, lat0 = -1, high_run = 0, in_frame = 0;
	integer pos, nb;
	reg [7:0] got;
	reg bad_shape;
	integer last_frame_start = -1, max_span = 0;

	always @(posedge clk) if (rst_n) begin
		#1;
		clk_n = clk_n + 1;
		if (!in_frame) begin
			if (con_tx === 1'b1) high_run = high_run + 1;
			else begin
				// start bit begins at this sample
				if (frames > 0 && high_run < DIV) fail("stop bit / idle shorter than 104 clocks before a start bit");
				if (exp_rd == exp_wr) fail("frame with no accepted start");
				else begin
					if (lat0 < 0) lat0 = clk_n - accept_clk[exp_rd];
					else if (clk_n - accept_clk[exp_rd] != lat0) fail("start-bit latency not constant");
					if (lat0 > 2) fail("start-bit latency above 2 clocks");
				end
				if (last_frame_start >= 0 && clk_n - last_frame_start > max_span) max_span = clk_n - last_frame_start;
				last_frame_start = clk_n;
				in_frame = 1; pos = 1; got = 0; bad_shape = 0;
			end
		end else begin
			// pos = sample index within the frame, 0-based; bit = pos / DIV
			nb = pos / DIV;
			if (nb == 0 && con_tx !== 1'b0) bad_shape = 1;
			if (nb >= 1 && nb <= 8) begin
				if (pos % DIV == 0) got[nb-1] = con_tx;
				else if (con_tx !== got[nb-1]) bad_shape = 1;
			end
			pos = pos + 1;
			if (pos == 9 * DIV) begin
				in_frame = 0; high_run = 0;
				frames = frames + 1;
				if (bad_shape) fail("start/data bit not exactly 104 clocks of one level");
				if (exp_rd != exp_wr) begin
					if (got !== exp_q[exp_rd]) begin
						fail("wrong byte");
						$display("      expected %h got %h", exp_q[exp_rd], got);
					end
					exp_rd = exp_rd + 1;
				end
			end
		end
	end

	// ---- 5. rdy while the frame is on the line --------------------------
	always @(posedge clk) if (rst_n) begin
		#2;
		if (in_frame && rdy) fail("rdy high during start/data bits");
	end

	// ---- driver -----------------------------------------------------------
	task send(input [7:0] b, input integer gap);
		begin
			repeat (gap) @(posedge clk);
			while (!rdy) @(posedge clk);	// rdy is sampled on the edge
			#1 data_tx = b; start = 1;
			@(posedge clk);
			exp_q[exp_wr] = b; accept_clk[exp_wr] = clk_n + 1; exp_wr = exp_wr + 1;
			#1 start = 0; data_tx = $random(seed);	// garbage after the accept
		end
	endtask

	task wait_idle;
		integer n;
		begin
			n = 0;
			while (!(rdy && !in_frame && high_run > DIV + 4) && n < 20 * DIV) begin @(posedge clk); n = n + 1; end
			if (n == 20 * DIV) fail("line never idle (high, rdy) within 20 bit times");
			#3;
		end
	endtask

	integer i, f0, b;
	reg [7:0] inflight;
	initial begin
		if ($value$plusargs("seed=%d", seed)) ;
		// ---- 1. reset ---------------------------------------------------
		#0.05 rst_n = 0;
		#0.05;
		if (con_tx !== 1'b1) fail("con_tx not 1 in reset (before any clkin edge)");
		if (rdy !== 1'b1) fail("rdy not 1 in reset");
		#(T * 10.3) rst_n = 1;
		repeat (3 * DIV) @(posedge clk);
		#3;
		if (con_tx !== 1'b1 || frames != 0 || in_frame) fail("line not idle high after reset");
		if (rdy !== 1'b1) fail("rdy not 1 when idle after reset");
		$display("      reset: con_tx %b rdy %b; idle %0d clocks", con_tx, rdy, high_run);

		// ---- 3./4. all bytes back to back ---------------------------------
		for (i = 0; i < 256; i = i + 1) send(i[7:0], 0);
		wait_idle;
		if (frames != 256 || exp_rd != 256) fail("back to back: not 256 frames");
		if (max_span > 10 * DIV + 2) fail("back to back: a byte takes more than 10 x 104 + 2 clocks");
		$display("      back to back: %0d frames, latency %0d clock(s), longest start-to-start %0d clocks (min %0d)",
			frames, lat0, max_span, 10 * DIV);

		// ---- 3. random gaps ------------------------------------------------
		for (i = 0; i < 64; i = i + 1) send($random(seed), {$random(seed)} % 300);
		wait_idle;
		if (frames != 320) fail("random gaps: frame count");
		$display("      random gaps: %0d frames in all", frames);

		// ---- 6. start while busy -------------------------------------------
		for (i = 0; i < 8; i = i + 1) begin
			inflight = $random(seed);
			f0 = frames;
			send(inflight, 0);
			repeat ({$random(seed)} % (9 * DIV)) @(posedge clk);
			#1 start = 1; data_tx = ~inflight;
			repeat (1 + {$random(seed)} % 40) @(posedge clk);
			#1 start = 0;
			wait_idle;
			if (frames != f0 + 1) fail("start while busy made an extra frame");
		end
		$display("      start while busy: %0d frames in all", frames);

		// ---- 1. reset in the middle of a byte ------------------------------
		send(8'h00, 0);
		repeat (3 * DIV) @(posedge clk);
		#(T / 3) rst_n = 0;
		#0.1;
		if (con_tx !== 1'b1) fail("con_tx not 1 at once on rst_n falling");
		if (rdy !== 1'b1) fail("rdy not 1 at once on rst_n falling");
		in_frame = 0; exp_rd = exp_wr; high_run = 0;
		#(T * 5) rst_n = 1;
		repeat (3 * DIV) @(posedge clk);
		#3;
		if (con_tx !== 1'b1 || in_frame) fail("line not idle after a reset mid-byte");
		send(8'hA5, 0);
		wait_idle;

		if (fails == 0) $display("console_uart_tb: ALL PASS (%0d frames)", frames);
		else $display("console_uart_tb: %0d FAIL", fails);
		$finish;
	end
endmodule
