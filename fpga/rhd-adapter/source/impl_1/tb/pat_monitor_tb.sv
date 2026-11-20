//testbench
//
//	Testbench for the D9 K4.2 block counter, pat_monitor (kuntur-notes
//	docs/interfaces/companion-console.md §8.8):
//
//	  wclk, rst_n (con_rst_n) ─┐
//	  cdr valid, dout, pos ────┴─► pat_monitor ─► pat_blocks  (rx)
//	                                          ─► pat_errors  (crc)
//
//	The source is a block_gen model (A5 5A A5 5A c_lo c_hi 01 80, c +1 per
//	block) driving din/pos the way cdr's aligner does: byte k with pos = k,
//	both changing just after a wclk edge. A cycle model of M1-M6 runs on
//	the same edges and queues the (pat_blocks, pat_errors) each checked
//	block must produce.
//
//	Checks, after every wclk edge
//	  1. M1-M3: every change of the outputs is the next queued pair, both
//	     outputs on one edge (M4), at one fixed latency L (0-4 edges after
//	     the pos = 7 byte, learned on the first block, then constant).
//	  2. No change without a checked block (M1, M6 hold); no queued block
//	     left uncounted after L edges.
//	  3. pat_errors <= pat_blocks; no X after reset.
//	  4. M5: preloaded near FFFFFFFF (dut.pat_blocks / dut.pat_errors, so
//	     both outputs must be the counter registers themselves), both stop
//	     when pat_blocks reaches FFFFFFFF.
//	  5. M7: rst_n pulled low with wclk stopped: both outputs 0 at once.
//	Cases: valid rising inside a block; clean run; one bit flipped in each
//	bit of each byte; two flips in one block; a lost block, a repeated
//	block and a count restart (2 error blocks each, then clean); valid
//	falling and rising inside blocks, a 1-byte valid gap, a long gap with
//	the source counting on, garbage pos/din while valid = 0; errors in the
//	first block after valid rises; wclk stopped mid-block; reset mid-run;
//	saturation; 20,000 random blocks (+seed=<n>).
//
//	Define: +define+REF_MODEL replaces the DUT by a model; +define+MUT=<k>
//	  breaks it (with REF_MODEL), each must FAIL:
//	    1 partial block counted (valid needed on byte 7 only); 2 no resync;
//	    3 resync on every count error; 4 bit errors counted, not blocks;
//	    5 pat_errors one clock after pat_blocks; 6 no saturation (wraps);
//	    7 pat_errors goes on after pat_blocks saturates; 8 cleared while
//	    valid = 0; 9 reset synchronous to wclk; 10 E not reseeded after
//	    valid = 0; 11 count bytes not checked; 12 byte 6 not checked.
//
//	Run (from this directory):
//	  tr '\r' '\n' < ../intan_rec_adapter.v | awk '/^module pat_monitor[ #(]/,/^endmodule/' > /tmp/pm_rtl.v
//	  iverilog -g2012 -Wall -o pm.vvp pat_monitor_tb.sv /tmp/pm_rtl.v && for s in 1 2 3; do vvp pm.vvp +seed=$s | tail -2; done
//	  iverilog -g2012 -Wall -DREF_MODEL -o pm.vvp pat_monitor_tb.sv && vvp pm.vvp
//	  for k in 1 2 3 4 5 6 7 8 9 10 11 12; do iverilog -g2012 -DREF_MODEL -DMUT=$k -o pm.vvp pat_monitor_tb.sv && vvp pm.vvp | grep pat_monitor_tb:; done
//	Expected: "pat_monitor_tb: ALL PASS" for the RTL and the model; FAIL
//	  for every mutant.

`timescale 1ns/1ps

`ifdef REF_MODEL
`ifndef MUT
	`define MUT 0
`endif
module pat_monitor (
	input  wire	wclk,
	input  wire	rst_n,
	input  wire	valid,
	input  wire	[7:0]	din,
	input  wire	[2:0]	pos,
	output reg	[31:0]	pat_blocks,
	output reg	[31:0]	pat_errors
);
	localparam [31:0] MAX = 32'hFFFFFFFF;
	reg rst_w1, rst_w2;
	always @(negedge rst_n or posedge wclk)
		if (!rst_n) begin rst_w1 <= 0; rst_w2 <= 0; end
		else begin rst_w1 <= 1; rst_w2 <= rst_w1; end

	reg inblk, ferr, have_e, pair, err_d;
	reg [2:0] ppos;
	reg [7:0] c_lo, c_hi;
	reg [15:0] e, pc;
	reg [31:0] bits;

	function [3:0] pop(input [7:0] x);
		pop = x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[7];
	endfunction
	function [7:0] expb(input [2:0] k);
		case (k) 0, 2: expb = 8'hA5; 1, 3: expb = 8'h5A; 6: expb = 8'h01; 7: expb = 8'h80; default: expb = 8'h00; endcase
	endfunction

	task clr;
		begin
			inblk <= 0; ferr <= 0; have_e <= 0; pair <= 0; err_d <= 0; ppos <= 0;
			c_lo <= 0; c_hi <= 0; e <= 0; pc <= 0; bits <= 0;
			pat_blocks <= 0; pat_errors <= 0;
		end
	endtask

	wire v = (`MUT == 1) ? (valid || pos != 3'd7) : valid;
	wire fixed = (pos != 3'd4 && pos != 3'd5 && !(`MUT == 12 && pos == 3'd6));
	wire bad = fixed && (din != expb(pos));

	task body;
		reg f, cerr, pn;
		reg [15:0] c, en;
		begin
			ppos <= pos;
			if (`MUT == 5) begin
				err_d <= 0;
				if (err_d && pat_errors != MAX) pat_errors <= pat_errors + 1;
			end
			if (!v) begin
				inblk <= 0; pair <= 0;
				if (`MUT != 10) have_e <= 0;
				if (`MUT == 8) begin pat_blocks <= 0; pat_errors <= 0; end
			end else if (pos == 3'd0) begin
				inblk <= 1; ferr <= bad; bits <= pop(din ^ expb(0));
			end else if (inblk && pos == ppos + 3'd1) begin
				if (pos == 3'd4) c_lo <= din;
				if (pos == 3'd5) c_hi <= din;
				if (pos != 3'd7) begin
					ferr <= ferr | bad;
					if (fixed) bits <= bits + pop(din ^ expb(pos));
				end else begin
					inblk <= 0;
					f = ferr | bad;
					c = {c_hi, c_lo};
					if (!have_e) begin
						en = c; cerr = 0; pn = 0;
					end else begin
						en = e + 16'd1;
						cerr = (`MUT == 11) ? 1'b0 : (c != en);
						pn = cerr && !f;
						if (`MUT == 3 && cerr) begin en = c; pn = 0; end
						else if (`MUT != 2 && pn && pair && c == pc + 16'd1) begin en = c; pn = 0; end
					end
					have_e <= 1; e <= en; pair <= pn; pc <= c;
					if (pat_blocks != MAX || `MUT == 6) begin
						pat_blocks <= pat_blocks + 1;
						if (f || cerr) begin
							if (`MUT == 4) pat_errors <= pat_errors + bits + pop(din ^ 8'h80) + cerr;
							else if (`MUT == 5) err_d <= 1;
							else pat_errors <= pat_errors + 1;
						end
					end else if (`MUT == 7 && (f || cerr))
						pat_errors <= pat_errors + 1;
				end
			end else
				inblk <= 0;
		end
	endtask

	generate if (`MUT == 9) begin : sync_rst
		always @(posedge wclk) if (!rst_w2) clr; else body;
	end else begin : async_rst
		always @(negedge rst_w2 or posedge wclk) if (!rst_w2) clr; else body;
	end endgenerate
endmodule
`endif

module pat_monitor_tb;
	localparam real T = 1000.0 / 9.4875;	// ns, 25k build
	localparam [31:0] MAX = 32'hFFFFFFFF;
	reg wclk = 0, wclk_en = 0, rst_n = 1;
	reg valid = 0;
	reg [7:0] din = 0;
	reg [2:0] pos = 0;
	wire [31:0] pat_blocks, pat_errors;

	always #(T/2) if (wclk_en || wclk) wclk = ~wclk;	// stops low, glitch-free

	pat_monitor dut (.wclk(wclk), .rst_n(rst_n), .valid(valid), .din(din), .pos(pos),
		.pat_blocks(pat_blocks), .pat_errors(pat_errors));

	integer fails = 0, seed = 1;
	task fail(input [8*100-1:0] msg);
		begin
			fails = fails + 1;
			if (fails <= 20) $display("FAIL  %0s  (blocks %h errors %h, want %h %h; t=%0.3f us)",
				msg, pat_blocks, pat_errors, mb, me, $realtime/1000.0);
		end
	endtask

	// ---------------- cycle model (M1-M6) ----------------
	reg [31:0] mb = 0, me = 0;		// model counters
	reg m_inblk = 0, m_have = 0, m_pair = 0, m_ferr = 0;
	reg [2:0] m_ppos = 0;
	reg [7:0] m_lo = 0, m_hi = 0;
	reg [15:0] m_e = 0, m_pc = 0;
	integer rcnt = 0;				// edges since rst_n rose
	integer edge_n = 0;
	integer qb [0:63], qe [0:63], qt [0:63];
	integer qh = 0, qtl = 0;
	integer L = -1;
	reg [31:0] lastb = 0, laste = 0;
	integer n_checked = 0, n_err = 0;

	function [7:0] expb(input [2:0] k);
		case (k) 0, 2: expb = 8'hA5; 1, 3: expb = 8'h5A; 6: expb = 8'h01; 7: expb = 8'h80; default: expb = 8'h00; endcase
	endfunction

	task model_clear;
		begin
			mb = 0; me = 0; m_inblk = 0; m_have = 0; m_pair = 0; m_ferr = 0;
			qh = 0; qtl = 0; lastb = 0; laste = 0;
		end
	endtask

	task model_step(input sv, input [2:0] sp, input [7:0] sd);
		reg f, cerr, pn;
		reg [15:0] c, en;
		begin
			if (!sv) begin
				m_inblk = 0; m_have = 0; m_pair = 0;
			end else if (sp == 0) begin
				m_inblk = 1; m_ferr = (sd != expb(0));
			end else if (m_inblk && sp == m_ppos + 3'd1) begin
				if (sp == 4) m_lo = sd;
				else if (sp == 5) m_hi = sd;
				else m_ferr = m_ferr | (sd != expb(sp));
				if (sp == 7) begin
					m_inblk = 0;
					f = m_ferr;
					c = {m_hi, m_lo};
					if (!m_have) begin en = c; cerr = 0; pn = 0; end
					else begin
						en = m_e + 16'd1;
						cerr = (c != en);
						pn = cerr && !f;
						if (pn && m_pair && c == m_pc + 16'd1) begin en = c; pn = 0; end
					end
					m_have = 1; m_e = en; m_pair = pn; m_pc = c;
					if (mb != MAX) begin
						mb = mb + 1; n_checked = n_checked + 1;
						if (f || cerr) begin me = me + 1; n_err = n_err + 1; end
						qb[qtl % 64] = mb; qe[qtl % 64] = me; qt[qtl % 64] = edge_n; qtl = qtl + 1;
					end
				end
			end else
				m_inblk = 0;
			m_ppos = sp;
		end
	endtask

	// sample on the edge (inputs change 1 ns after it), model and check just after
	reg sv; reg [2:0] sp; reg [7:0] sd; integer age;
	always @(posedge wclk) begin
		edge_n = edge_n + 1;
		sv = valid; sp = pos; sd = din;
		#0.01;
		if (!rst_n) rcnt = 0;
		else begin
			rcnt = rcnt + 1;
			if (rcnt >= 3) model_step(sv, sp, sd);
		end
		if ((^pat_blocks) === 1'bx || (^pat_errors) === 1'bx) fail("output X (M7)");
		else begin
			if (pat_errors > pat_blocks) fail("pat_errors > pat_blocks (M4)");
			if (pat_blocks !== lastb || pat_errors !== laste) begin
				if (qh == qtl) fail("output changed with no checked block (M1/M6)");
				else begin
					if (pat_blocks !== qb[qh % 64] || pat_errors !== qe[qh % 64])
						fail("wrong pair, or the two outputs not on one edge (M1-M4)");
					age = edge_n - qt[qh % 64];
					if (L < 0) begin
						if (age > 4) fail("latency > 4 edges");
						L = age;
					end else if (age != L) fail("latency changed (M4)");
					qh = qh + 1;
				end
				lastb = pat_blocks; laste = pat_errors;
			end
			if (qh != qtl && edge_n - qt[qh % 64] > ((L < 0) ? 4 : L)) begin
				fail("checked block not counted (M1-M3)");
				lastb = qb[qh % 64]; laste = qe[qh % 64]; qh = qh + 1;
			end
		end
	end

	// M7: 0 at once, with or without wclk
	always @(negedge rst_n) begin
		#1;
		if (pat_blocks !== 0 || pat_errors !== 0) fail("not 0 at once in reset (M7)");
		model_clear;
	end

	// ---------------- source (block_gen + aligner) ----------------
	reg [15:0] c = 16'h1234;

	task put(input v, input [2:0] p, input [7:0] d);
		begin @(posedge wclk); #1 valid = v; pos = p; din = d; end
	endtask

	// one block with count c, bit flips, per-byte valid; c advances
	task blk(input [63:0] flip, input [7:0] vm);
		integer k; reg [7:0] b;
		begin
			for (k = 0; k < 8; k = k + 1) begin
				b = (k == 4) ? c[7:0] : (k == 5) ? c[15:8] : expb(k);
				put(vm[k], k, b ^ flip[8*k +: 8]);
			end
			c = c + 16'd1;
		end
	endtask
	task clean(input integer n);
		integer i; begin for (i = 0; i < n; i = i + 1) blk(0, 8'hFF); end
	endtask
	task gap(input integer n);		// blocks sent with valid = 0, source counting on
		integer i; begin for (i = 0; i < n; i = i + 1) blk(0, 8'h00); end
	endtask
	task garbage(input integer n);	// sweep-like: random pos and din, valid = 0
		integer i; begin for (i = 0; i < n; i = i + 1) put(0, $random(seed), $random(seed)); end
	endtask
	function [63:0] bitflip(input integer byte_i, input integer bit_i);
		bitflip = 64'd1 << (8*byte_i + bit_i);
	endfunction
	task stop_wclk(input real ns);
		begin @(negedge wclk); wclk_en = 0; #(ns); wclk_en = 1; end
	endtask
	task wait_idle;	// queue drained
		begin repeat (6) @(posedge wclk); #0.5; end
	endtask

	integer i, j, r, n;
	initial begin
		if (!$value$plusargs("seed=%d", seed)) seed = 1;

		// M7 at power-up: reset with wclk stopped, outputs 0 at once
		#0.05 rst_n = 0;
		#2;
		if (pat_blocks !== 0 || pat_errors !== 0) fail("not 0 at once in reset, wclk stopped (M7)");
		#(T*3) wclk_en = 1;
		repeat (5) @(posedge wclk);
		if (pat_blocks !== 0 || pat_errors !== 0) fail("not 0 in reset with wclk running (M7)");
		#(T/3) rst_n = 1;
		gap(2);

		// valid rising inside a block: that block is not counted (M1)
		blk(0, 8'hF0);
		clean(2000);

		// one bit in each place, three clean blocks between
		for (i = 0; i < 8; i = i + 1)
			for (j = 0; j < 8; j = j + 1) begin blk(bitflip(i, j), 8'hFF); clean(3); end
		// two flips in one block, fixed and count bytes
		blk(bitflip(0, 1) | bitflip(7, 7), 8'hFF); clean(3);
		blk(bitflip(2, 3) | bitflip(5, 0), 8'hFF); clean(3);
		blk(bitflip(4, 0) | bitflip(5, 7), 8'hFF); clean(3);
		// lost block, repeated block, restart to 0, count jump
		c = c + 1; clean(5);
		c = c - 1; clean(5);
		c = 0; clean(5);
		c = c + 16'h0100; clean(5);
		c = 16'hFFFE; clean(5);				// count wraps FFFF -> 0000
		// a count flip followed by a slip
		blk(bitflip(4, 2), 8'hFF); c = c + 1; clean(5);

		// valid falling and rising inside blocks, 1-byte gaps, long gap
		blk(0, 8'h1F); blk(0, 8'hF8); clean(3);
		blk(0, 8'hEF); clean(3);
		blk(0, 8'h7F); clean(3);
		blk(0, 8'hFE); clean(3);
		gap(1000); clean(10);
		garbage(333); clean(10);
		// errors in the first block after valid rises
		gap(3); blk(bitflip(2, 5), 8'hFF); clean(5);
		gap(3); blk(bitflip(5, 4), 8'hFF); clean(5);
		gap(3); blk(bitflip(4, 0), 8'hFF); clean(5);
		// wclk stopped mid-block, valid stays 1
		blk(0, 8'hFF);
		for (i = 0; i < 4; i = i + 1) put(1, i, expb(i));
		stop_wclk(5000);
		put(1, 4, c[7:0]); put(1, 5, c[15:8]); put(1, 6, 8'h01); put(1, 7, 8'h80); c = c + 1;
		clean(5);

		// reset mid-run, wclk stopped
		gap(2); @(negedge wclk); wclk_en = 0; #(T*2) rst_n = 0;
		#2; if (pat_blocks !== 0 || pat_errors !== 0) fail("not 0 at once in reset mid-run (M7)");
		#(T*4) wclk_en = 1; repeat (3) @(posedge wclk); #(T/4) rst_n = 1;
		gap(2); clean(500);

		// M5 saturation: preload near the top, errors keep coming
		gap(2); wait_idle;
		mb = MAX - 20; me = MAX - 40; lastb = mb; laste = me;
		force dut.pat_blocks = mb; force dut.pat_errors = me;
		#0.5; release dut.pat_blocks; release dut.pat_errors;
		gap(1);
		for (i = 0; i < 40; i = i + 1) blk((i % 3 == 0) ? bitflip(0, 0) : 64'd0, 8'hFF);
		gap(2); wait_idle;
		if (pat_blocks !== MAX) fail("pat_blocks not at FFFFFFFF (M5)");
		if (pat_errors !== MAX - 40 + 7) fail("pat_errors not stopped with pat_blocks (M5)");
		gap(2); @(negedge wclk); wclk_en = 0; #(T) rst_n = 0; #(T*2) wclk_en = 1;
		repeat (3) @(posedge wclk); #(T/4) rst_n = 1; gap(2);

		// random
		for (n = 0; n < 20000; n = n + 1) begin
			r = $unsigned($random(seed)) % 1000;
			if (r < 8) blk(bitflip($unsigned($random(seed)) % 8, $unsigned($random(seed)) % 8), 8'hFF);
			else if (r < 10) blk({$random(seed), $random(seed)} & {$random(seed), $random(seed)}, 8'hFF);
			else if (r < 12) begin c = c + 1 + $unsigned($random(seed)) % 3; blk(0, 8'hFF); end
			else if (r < 14) begin c = c - 1; blk(0, 8'hFF); end
			else if (r < 15) begin c = 0; blk(0, 8'hFF); end
			else if (r < 18) begin
				blk(0, ~(8'hFF << ($unsigned($random(seed)) % 8)));
				if ($random(seed) & 1) gap($unsigned($random(seed)) % 50); else garbage($unsigned($random(seed)) % 300);
				blk(0, 8'hFF << ($unsigned($random(seed)) % 8));
			end else if (r < 20) blk(0, ~(8'd1 << ($unsigned($random(seed)) % 8)));
			else if (r < 21) begin
				for (i = 0; i < 3; i = i + 1) put(1, i, expb(i));
				stop_wclk(200 + $unsigned($random(seed)) % 3000);
				put(1, 3, 8'h5A); put(1, 4, c[7:0]); put(1, 5, c[15:8]); put(1, 6, 8'h01); put(1, 7, 8'h80); c = c + 1;
			end else blk(0, 8'hFF);
		end
		gap(2); wait_idle;

		if (qh != qtl) fail("blocks left uncounted at the end");
		$display("      %0d blocks checked, %0d with errors; latency %0d edges; final %0d / %0d",
			n_checked, n_err, L, pat_blocks, pat_errors);
		if (fails == 0) $display("pat_monitor_tb: ALL PASS");
		else $display("pat_monitor_tb: %0d FAIL", fails);
		$finish;
	end
endmodule
