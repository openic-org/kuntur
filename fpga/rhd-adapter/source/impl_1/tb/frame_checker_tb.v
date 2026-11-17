//testbench
//
//	Unit test for frame_checker, the error counter of the tunnel receiver.
//	Contract: vega docs/interfaces/lvds-tunnel.md §6.6 (Checker).
//
//	The checker is driven with an ideal aligned stream: pos counts 0-7 on
//	every clk, din is the K4.2 link test pattern A5 5A A5 5A cnt_lo cnt_hi
//	01 80, with cnt starting at FFF0 so that it wraps through 0000. Each run
//	resets the checker once at each of the 8 byte positions and waits for
//	done. errors is read on the first cycle of done and again 20 clk later.
//
//	Modes (plusargs):
//	  (none)    clean stream: lock, errors = 0.
//	  +inj=<k>  flip bit 7 of cnt_lo in every k-th block: errors = N/k.
//	  +flood    every bit wrong from the first counted byte on: errors =
//	            64 x N, so the window is exactly N blocks. Saturates at
//	            FFFF when 64 x N > 65535.
//	  +dead     din = 0: no_lock after NB_CHECK blocks.
//	Parameter N (default 20) is the checker's NB_ERRORS. N = 20 is below
//	NB_CHECK = 100, which the no-lock timeout must not depend on.
//
//	Run (Icarus, from this directory). frame_checker is cut out of
//	intan_rec_adapter.v, as for cdr_tb.sv:
//	  awk '/^module frame_checker /,/^endmodule/' ../intan_rec_adapter.v > /tmp/chk_rtl.v
//	  iverilog -g2012 -Wall -o chk.vvp frame_checker_tb.v /tmp/chk_rtl.v
//	  vvp chk.vvp; vvp chk.vvp +inj=4; vvp chk.vvp +flood; vvp chk.vvp +dead
//	Saturation, including a crossing on the window's last byte:
//	  iverilog -g2012 -Pframe_checker_tb.N=1100 -o chk_1100.vvp frame_checker_tb.v /tmp/chk_rtl.v
//	  iverilog -g2012 -Pframe_checker_tb.N=1024 -o chk_1024.vvp frame_checker_tb.v /tmp/chk_rtl.v
//	  vvp chk_1100.vvp +flood; vvp chk_1024.vvp +flood
//	Each run prints PASS or FAIL last. Passed all six on 2026-10-27.

`timescale 1ns/1ps

module frame_checker_tb;
	parameter N = 20;

	reg clk = 0; always #5 clk = ~clk;
	reg rst = 0, start = 0;
	reg [8:0] pos = 0;
	reg [7:0] din;
	reg [15:0] cnt = 16'hFFF0;
	wire no_lock, done;
	wire [15:0] errors;
	integer inj, k, ph, blk = 0, fails = 0, flood, dead, expect_e, at_done;

	// flood starts on the cycle the checker seeds its expected cnt, the
	// first byte it counts
	wire flooding = flood && (dut.frame_sa || dut.current_state == 2 || dut.current_state == 3);

	frame_checker #(.NB_ERRORS(N)) dut (
		.rst_sync(rst), .clk(clk), .start(start), .din(din), .pos(pos),
		.no_lock(no_lock), .errors(errors), .done(done));

	always @* begin
		case (pos[2:0])
			0: din = 8'hA5; 1: din = 8'h5A; 2: din = 8'hA5; 3: din = 8'h5A;
			4: din = cnt[7:0]; 5: din = cnt[15:8]; 6: din = 8'h01; 7: din = 8'h80;
		endcase
		if (inj && pos == 4 && blk % inj == 0) din = din ^ 8'h80;
		if (flooding) din = ~din;
		if (dead) din = 0;
	end

	always @(posedge clk) begin
		pos <= (pos == 7) ? 0 : pos + 1;
		if (pos == 7) begin cnt <= cnt + 1; blk <= blk + 1; end
	end

	initial begin
		if (!$value$plusargs("inj=%d", inj)) inj = 0;
		flood = $test$plusargs("flood");
		dead  = $test$plusargs("dead");
		expect_e = flood ? (64 * N > 65535 ? 65535 : 64 * N) : inj ? N / inj : 0;
		for (ph = 0; ph < 8; ph = ph + 1) begin
			while (pos != ph) @(posedge clk);
			rst <= 1; @(posedge clk); rst <= 0; start <= 1; @(posedge clk); start <= 0;
			k = 0;
			while (!done && k < 200000) begin @(posedge clk); k = k + 1; end
			#1 at_done = errors;
			repeat (20) @(posedge clk);
			if (dead) begin
				if (!(done && no_lock)) fails = fails + 1;
				$display("reset at pos %0d: done=%b no_lock=%b after %0d clk (%0d blocks)", ph, done, no_lock, k, k / 8);
			end else begin
				if (!(done && !no_lock && at_done == expect_e && errors == expect_e)) fails = fails + 1;
				$display("reset at pos %0d: done=%b no_lock=%b errors at done=%0d, 20 clk later=%0d (expect %0d) after %0d clk",
				         ph, done, no_lock, at_done, errors, expect_e, k);
			end
		end
		if (fails) $display("FAIL (%0d of 8)", fails); else $display("PASS");
		$finish;
	end
endmodule
