//testbench
//
//	Self-checking testbench for block_gen, the tunnel link test pattern
//	(K4.2.3). Contract: vega docs/interfaces/lvds-tunnel.md §12 step 4.
//
//	An 8-byte block, repeated with no gaps:
//
//	  byte   0    1    2    3    4          5           6    7
//	  value  A5   5A   A5   5A   cnt[7:0]   cnt[15:8]   01   80
//
//	cnt is a 16-bit block counter, +1 per block. The link is LSB-first:
//	byte bit k drives ODDRX4.Dk, and D0 leaves the pin first.
//
//	What is modelled
//	  - ODDRX4 is replaced by a behavioural 8:1 serialiser (D0 first).
//	    The Lattice model is a wrapper around IOLOGIC_CORE and adds
//	    nothing that this test needs.
//	  - tun_checker models what the companion has to do: hunt for the
//	    preamble in the raw bit stream from an unknown bit phase, confirm
//	    it one block later, then check every block.
//
//	What is checked
//	  - Nominal: the checker locks at each of the 8 bit phases and sees
//	    no error over N blocks. The default N crosses the cnt wrap
//	    FFFF -> 0000.
//	  - Mutants: each of these must be detected (errors > 0), which
//	    proves that the checker can fail:
//	      msb    serialiser sends D7 first. The preamble is
//	             bit-palindromic, so this still locks; only 01/80 and cnt
//	             catch it.
//	      slip   one bit is lost mid-stream (a byte-alignment error).
//	      skip   one whole block is lost: the preamble stays aligned,
//	             and only cnt+1 catches it.
//
//	Run:  +blocks=<n> overrides N (default 65600).
//	      iverilog -g2012 -s block_gen_tb -I .. -I . -o bg.vvp block_gen_tb.sv
//	      vvp bg.vvp +blocks=2000

`timescale 1ns/1ps
`include "../kuntur_fpga.v"

module tun_checker #(
	parameter integer SKIP    = 0,			// bits ignored at the start (bit phase)
	parameter integer DROP_AT = -1,			// first bit index to drop, -1 = none
	parameter integer DROP_N  = 0			// number of bits dropped there
)(
	input  wire clk,
	input  wire en,
	input  wire bit_in
);
	localparam [31:0] PRE  = 32'h5AA55AA5;	// A5 5A A5 5A, oldest byte in [7:0]
	localparam [15:0] TAIL = 16'h8001;		// 01 80

	localparam HUNT = 0, CONFIRM = 1, LOCKED = 2;

	reg  [63:0] sr = 0;						// last 64 bits; oldest byte in [7:0]
	integer idx = 0, nb = 0, state = HUNT;
	integer blocks_ok = 0, errors = 0, prev_cnt = -1;
	integer first_lock_bit = -1;
	reg [63:0] sr_n;

	always @(posedge clk) if (en) begin
		if (idx >= SKIP && !(DROP_AT >= 0 && idx >= DROP_AT && idx < DROP_AT + DROP_N)) begin
			sr_n = {bit_in, sr[63:1]};
			sr  <= sr_n;
			nb   = nb + 1;
			case (state)
			HUNT:
				// The preamble has just been completed.
				if (sr_n[63:32] == PRE) begin
					state = CONFIRM;
					nb    = 0;
				end
			CONFIRM:
				// The next preamble must complete exactly one block later.
				// Then wait 32 bits more, so that each check sees a whole
				// block with its preamble in [31:0].
				if (nb == 64) begin
					if (sr_n[63:32] == PRE) begin
						state = LOCKED;
						first_lock_bit = idx;
						nb = 32;
					end else begin
						state = HUNT;	// false preamble: keep hunting
						nb = 0;
					end
				end
			LOCKED:
				if (nb == 64) begin
					check_block(sr_n);
					nb = 0;
				end
			endcase
		end
		idx = idx + 1;
	end

	task check_block(input [63:0] b);
		reg ok;
		integer cnt;
		begin
			ok  = 1;
			cnt = b[47:32];
			if (b[31:0] != PRE)  ok = 0;
			if (b[63:48] != TAIL) ok = 0;
			if (prev_cnt >= 0 && cnt != ((prev_cnt + 1) % 65536)) ok = 0;
			if (ok)
				blocks_ok = blocks_ok + 1;
			else begin
				errors = errors + 1;
				if (errors <= 3)
					$display("  %m: bad block at bit %0d: %02h %02h %02h %02h %02h %02h %02h %02h (prev cnt %04h)",
						idx, b[7:0], b[15:8], b[23:16], b[31:24], b[39:32], b[47:40], b[55:48], b[63:56],
						prev_cnt[15:0]);
			end
			prev_cnt = cnt;
		end
	endtask
endmodule


module block_gen_tb();
	localparam real T_BIT = 87.82 / 8.0;	// 1 / 91.08 Mbps; wclk = 11.385 MHz

	integer n_blocks;

	reg  rstb = 0;
	reg  bitclk = 0;
	reg  wclk = 0;
	reg  [2:0] ph = 0;
	wire [7:0] dout;

	block_gen dut (
		.rstb	(rstb),
		.clk	(wclk),
		.dout	(dout)
	);

	always #(T_BIT/2) bitclk = ~bitclk;

	// wclk = bitclk / 8, as ECLKDIV makes SCLK from ECLK
	always @(posedge bitclk) begin
		ph   <= ph + 1;
		wclk <= (ph + 3'd1) >= 3'd4;
	end

	// Behavioural ODDRX4: latch the byte once per wclk, send D0 first.
	// The load happens at ph 7, half a wclk period after dout changes.
	reg [7:0] txb = 0;
	reg       ser_lsb = 0, ser_msb = 0, ser_en = 0;
	always @(posedge bitclk) begin
		if (ph == 3'd7) begin
			txb    <= dout;
			ser_en <= rstb;
		end
		ser_lsb <= txb[ph];
		ser_msb <= txb[3'd7 - ph];
	end
	// At the ph 7 edge txb still holds the old byte (bit 7 goes out);
	// from ph 0 the new byte goes out D0..D7. One bitclk of latency.

	// Nominal: all 8 bit phases
	genvar k;
	generate for (k = 0; k < 8; k = k + 1) begin : nom
		tun_checker #(.SKIP(k)) chk (.clk(bitclk), .en(ser_en), .bit_in(ser_lsb));
	end endgenerate

	// Mutants
	tun_checker                                  m_msb  (.clk(bitclk), .en(ser_en), .bit_in(ser_msb));
	tun_checker #(.DROP_AT(5003), .DROP_N(1))   m_slip (.clk(bitclk), .en(ser_en), .bit_in(ser_lsb));
	tun_checker #(.DROP_AT(5003), .DROP_N(64))  m_skip (.clk(bitclk), .en(ser_en), .bit_in(ser_lsb));

	integer fails, i, min_ok;

	task expect_pass(input [8*16-1:0] name, input integer locked, input integer ok, input integer err);
		begin
			if (locked && err == 0 && ok >= min_ok)
				$display("PASS  %0s: %0d blocks, 0 errors", name, ok);
			else begin
				$display("FAIL  %0s: locked=%0d blocks_ok=%0d errors=%0d (need >= %0d blocks)",
					name, locked, ok, err, min_ok);
				fails = fails + 1;
			end
		end
	endtask

	task expect_caught(input [8*16-1:0] name, input integer locked, input integer err);
		begin
			if (locked && err > 0)
				$display("PASS  mutant %0s detected: %0d bad blocks", name, err);
			else begin
				$display("FAIL  mutant %0s NOT detected: locked=%0d errors=%0d", name, locked, err);
				fails = fails + 1;
			end
		end
	endtask

	initial begin
		if (!$value$plusargs("blocks=%d", n_blocks))
			n_blocks = 65600;
		min_ok = n_blocks - 4;
		fails  = 0;

		// Release reset at a time not aligned to wclk, as rstb is on the board.
		#(1000.3) rstb = 1;
		#(n_blocks * 64.0 * T_BIT + 2000.0);

		$display("block_gen_tb: %0d blocks (%0.1f ms)", n_blocks, n_blocks * 64.0 * T_BIT / 1.0e6);
		expect_pass("phase 0", nom[0].chk.state == 2, nom[0].chk.blocks_ok, nom[0].chk.errors);
		expect_pass("phase 1", nom[1].chk.state == 2, nom[1].chk.blocks_ok, nom[1].chk.errors);
		expect_pass("phase 2", nom[2].chk.state == 2, nom[2].chk.blocks_ok, nom[2].chk.errors);
		expect_pass("phase 3", nom[3].chk.state == 2, nom[3].chk.blocks_ok, nom[3].chk.errors);
		expect_pass("phase 4", nom[4].chk.state == 2, nom[4].chk.blocks_ok, nom[4].chk.errors);
		expect_pass("phase 5", nom[5].chk.state == 2, nom[5].chk.blocks_ok, nom[5].chk.errors);
		expect_pass("phase 6", nom[6].chk.state == 2, nom[6].chk.blocks_ok, nom[6].chk.errors);
		expect_pass("phase 7", nom[7].chk.state == 2, nom[7].chk.blocks_ok, nom[7].chk.errors);
		if (n_blocks <= 65536)
			$display("NOTE  %0d blocks does not reach the cnt wrap (65536)", n_blocks);
		expect_caught("msb",  m_msb.state  == 2, m_msb.errors);
		expect_caught("slip", m_slip.state == 2, m_slip.errors);
		expect_caught("skip", m_skip.state == 2, m_skip.errors);

		if (fails == 0) $display("block_gen_tb: ALL PASS");
		else            $display("block_gen_tb: %0d FAILED", fails);
		$finish;
	end
endmodule
