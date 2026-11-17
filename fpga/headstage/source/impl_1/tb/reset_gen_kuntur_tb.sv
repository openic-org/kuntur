//testbench
//
//	Testbench for Kuntur's reset generator.
//	Contract: vega docs/interfaces/system-startup.md §1.2 (decision 10),
//	§1.3 (R-K2, R-K4, R-K7) and lvds-tunnel.md §6.4 (Kuntur start-up).
//
//	DUT: reset_gen_kuntur, the interface of R-K7. The module and port names
//	are placeholders until Manuel's RTL exists; rename here to match it.
//	gddr_sync is Lattice's own (../common/gddr_sync.v), wired as in §6.4:
//	SYNC_CLK = clkin, START = lock_sync, RST = gddr_rst. Its STOP and
//	DDR_RESET gate the modelled wclk, as ECLKSYNC and ECLKDIV do.
//
//	Clocks: clkin is MCO (32 MHz) and can stop; clk is pll0's CLKOP and
//	runs only while clk_en (the PLL needs clkin); wclk = clk / 4.
//
//	Checks (K1-K8 of R-K7), all running for the whole simulation:
//	  K1 power-up: rst_n_clkin, rst_n_clk, blk_rstb, lock_sync 0 and
//	     gddr_rst 1 at wake-up, with no clock running.
//	  K2 lock_sync = pll_lock through exactly 2 clkin flops; 0 at once
//	     (no edge) when rstb or pll_lock is 0.
//	  K3 rst_n_clkin released on a clkin edge after HOLDOFF..HOLDOFF+4
//	     edges of rstb && pll_lock && lock_sync, and does rise.
//	  K4 rst_n_clkin 0 at once when rstb or pll_lock falls, also with
//	     clkin stopped.
//	  K5 gddr_rst = !rst_n_clkin, 1 clkin late; 1 at once by !rstb or
//	     !pll_lock.
//	  K6 rst_n_clk: 0 at once with rst_n_clkin; released after 2..3 clk.
//	  K7 blk_rstb: 0 at once with rst_n_clkin && READY; released after
//	     2..3 wclk.
//	  K8 READY 20..26 clkin edges after rst_n_clkin rose, and does rise.
//	  Each scenario ends with block_gen out of reset (blk_rstb = 1), and
//	  every lock drop and rstb pulse is seen (READY falls).
//
//	Scenarios (one run): wake-up with MCO running but the PLL not locked;
//	LOCK chattering during acquisition; a 30 ns LOCK glitch (must be seen:
//	the clear is asynchronous); LOCK low 50 us; a 20 ns rstb pulse; rstb
//	held 100 us; MCO stopping mid-run (clk stops 1 us later, LOCK falls
//	5 us later, MCO back after 100 us); MCO stopping with LOCK falling
//	first. Variants (plusargs):
//	  +late_clkin    MCO starts 20 us after wake-up
//	  +early_kuntur  PLL already locked at wake-up
//	  +seed=<n>      LOCK chatter pattern (default 1)
//
//	Until the RTL exists, -DREF_MODEL compiles a behavioural model of
//	R-K7 (below, testbench only). -DREF_MODEL -DMUT=<k> breaks it on
//	purpose; every mutant must FAIL:
//	  1 hold-off counter not restarted by a lock drop
//	  2 released after HOLDOFF - 1 edges (32,767)
//	  3 lock drop clears synchronously only (fails with MCO stopped)
//	  4 lock_sync not cleared by pll_lock
//	  5 gddr_rst 0 at wake-up
//	  6 rst_n_clk through 1 clk flop
//	  7 rst_n_clk asserted synchronously
//	  8 blk_rstb through 1 wclk flop
//	  9 gddr_sync RST = !rstb alone (READY never returns after a drop)
//	 10 lock_sync through 1 flop
//
//	Run (Icarus, from this directory):
//	  G=../common/gddr_sync.v
//	  iverilog -g2012 -Wall -DREF_MODEL -o rk.vvp reset_gen_kuntur_tb.sv rst_check.sv $G
//	  vvp rk.vvp; vvp rk.vvp +late_clkin; vvp rk.vvp +early_kuntur
//	Against the RTL, once it is in kuntur_fpga.v:
//	  tr '\r' '\n' < ../kuntur_fpga.v | awk '/^module reset_gen_kuntur /,/^endmodule/' > /tmp/rk_rtl.v
//	  iverilog -g2012 -Wall -o rk.vvp reset_gen_kuntur_tb.sv rst_check.sv /tmp/rk_rtl.v $G
//	Each run prints PASS or FAIL last; a run simulates about 9 ms.

`timescale 1ns/1ps

module reset_gen_kuntur_tb;
	localparam HOLDOFF = 32768;		// R-K2: 1.024 ms of clkin
	localparam real T_CLKIN = 31.25;	// 32 MHz MCO
	localparam real T_CLK   = 21.9587;	// pll0 CLKOP, 45.54 MHz

	reg rstb = 1, pll_lock = 0, clkin_en = 1, clk_en = 1;
	reg clkin = 0, clk = 0, wclk = 0;
	wire lock_sync, rst_n_clkin, gddr_rst, rst_n_clk, blk_rstb;
	wire gddr_stop, gddr_ddr_reset, gddr_ready;
	integer errors = 0, seed = 1, ready_falls = 0;

	always #(T_CLKIN / 2) clkin = clkin_en ? ~clkin : 1'b0;
	always #(T_CLK / 2) clk = clk_en ? ~clk : 1'b0;
	// wclk: ECLKDIV / 4 of ECLK; ECLK stops with STOP, the divider is held
	// by DDR_RESET
	always #(2 * T_CLK)
		if (clk_en && gddr_stop === 1'b0 && gddr_ddr_reset === 1'b0) wclk = ~wclk;
		else if (gddr_ddr_reset !== 1'b0) wclk = 1'b0;

	reset_gen_kuntur dut (
		.clkin(clkin), .rstb(rstb), .pll_lock(pll_lock),
		.clk(clk), .wclk(wclk), .gddr_rdy(gddr_ready),
		.lock_sync(lock_sync), .rst_n_clkin(rst_n_clkin), .gddr_rst(gddr_rst),
		.rst_n_clk(rst_n_clk), .blk_rstb(blk_rstb));

	// 1 ps on gddr_sync's RST stands in for GSR, as in the companion's
	// testbench: at t = 0 gddr_rst is already 1 before the IP's always
	// block is armed, which would leave its state X until the first clkin
	// edge (none for 20 us with +late_clkin). On the FPGA, wake-up sets
	// those flops.
	wire #0.001 gddr_rst_ip = gddr_rst;

	gddr_sync sync0 (
		.rst(gddr_rst_ip), .sync_clk(clkin), .start(lock_sync),
		.stop(gddr_stop), .ddr_reset(gddr_ddr_reset), .ready(gddr_ready));

	// --- GSR stand-in. On the FPGA the wake-up GSR sets every flop to its
	//     REGSET, taken from the flop's reset value (system-startup.md
	//     R-C8, R-K7 K1); the RTL carries no initial values, so the
	//     testbench sets the required power-up values at t = 0: all 0
	//     except gddr_rst = 1. Internal names follow the RTL (Manuel's,
	//     2026-10-29); a rename fails to compile.
`ifndef REF_MODEL
	initial begin
		dut.cnt0          = 0;
		dut.rst_n_clkin   = 1'b0;
		dut.pll_lock_reg  = 1'b0;
		dut.lock_sync     = 1'b0;
		dut.gddr_rst      = 1'b1;
		dut.rst_n_clk_reg = 1'b0;
		dut.rst_n_clk     = 1'b0;
		dut.blk_rstb_1    = 1'b0;
		dut.blk_rstb      = 1'b0;
	end
`endif

	// --- generic reset checks (rst_check.sv): async assert, sync release
	//     after MIN..MAX edges, initial 0
	rst_check #(.NAME("rst_n_clkin"), .MIN(HOLDOFF), .MAX(HOLDOFF + 4))
		chk_clkin (.clk(clkin), .cond(rstb && pll_lock && lock_sync), .rst_n(rst_n_clkin));
	rst_check #(.NAME("rst_n_clk"), .MIN(2), .MAX(3))
		chk_clk (.clk(clk), .cond(rst_n_clkin), .rst_n(rst_n_clk));
	rst_check #(.NAME("blk_rstb"), .MIN(2), .MAX(3))
		chk_blk (.clk(wclk), .cond(rst_n_clkin && gddr_ready), .rst_n(blk_rstb));

	task fail(input [8*64-1:0] what);
		begin errors = errors + 1; $display("%t  FAIL  %0s", $realtime, what); end
	endtask

	initial $timeformat(-9, 1, " ns", 14);

	// gddr_sync's outputs are known from 1 ns on
	always @(gddr_stop, gddr_ddr_reset, gddr_ready)
		if ($realtime > 1.0 && ^{gddr_stop, gddr_ddr_reset, gddr_ready} === 1'bx)
			fail("gddr_sync output X");

	// --- K2 lock_sync: pll_lock through exactly 2 clkin flops, initial 0,
	//     cleared at once by rstb or pll_lock
	reg lk0 = 0, lk1 = 0;
	initial #0.001 if (lock_sync !== 1'b0) fail("lock_sync not 0 at wake-up");
	always @(posedge clkin) begin
		lk1 = lk0;
		lk0 = pll_lock && rstb;
		#0.001 if (lock_sync !== lk1) fail("lock_sync is not pll_lock through 2 flops");
	end
	always @(negedge rstb or negedge pll_lock) begin
		lk0 = 0; lk1 = 0;
		#0.001 if (lock_sync !== 1'b0) fail("lock_sync not cleared at once");
	end

	// --- K5 gddr_rst: !rst_n_clkin registered on clkin (one clkin late),
	//     1 at wake-up, 1 at once by !rstb or !pll_lock
	reg rst_want;
	initial #0.001 if (gddr_rst !== 1'b1) fail("gddr_rst not 1 at wake-up");
	always @(negedge rstb or negedge pll_lock)
		#0.001 if (gddr_rst !== 1'b1) fail("gddr_rst not 1 at once");
	always @(posedge clkin) begin
		rst_want = !rstb || !pll_lock || !rst_n_clkin;
		#0.001 if (gddr_rst !== rst_want) fail("gddr_rst != !rst_n_clkin, 1 clkin late");
	end

	// --- K8 READY timing and liveness, in clkin edges of rst_n_clkin high
	integer up_edges = 0;
	always @(posedge clkin) begin
		up_edges = rst_n_clkin ? up_edges + 1 : 0;
		if (up_edges == 40 && !gddr_ready) fail("READY not up 40 clkin after rst_n_clkin");
	end
	always @(negedge rst_n_clkin) up_edges = 0;
	always @(posedge gddr_ready)
		if (up_edges < 20 || up_edges > 26) fail("READY rose outside 20..26 clkin of rst_n_clkin");
		else $display("%t  READY after %0d clkin edges", $realtime, up_edges);
	always @(negedge gddr_ready) ready_falls = ready_falls + 1;

	// --- stimulus
	integer i, falls0;

	// LOCK chatters while the PLL acquires; highs up to ~3 us, far below
	// the hold-off, so every one restarts it
	task acquire;
		begin
			#2000;
			for (i = 0; i < 8; i = i + 1) begin
				pll_lock = 1; #(30 + ($unsigned($random(seed)) % 3000));
				pll_lock = 0; #(30 + ($unsigned($random(seed)) % 800));
			end
			pll_lock = 1;
		end
	endtask

	// MCO back: clkin, then the PLL output, then lock
	task mco_on;
		begin
			clkin_en = 1; #500; clk_en = 1;
			acquire;
		end
	endtask

	task expect_up(input [8*32-1:0] name);
		begin
			fork : w
				begin wait (blk_rstb === 1'b1); disable w; end
				begin #1500000; disable w; end
			join
			if (blk_rstb !== 1'b1) fail({"block_gen not out of reset: ", name});
			else $display("%t  ok    %0s", $realtime, name);
			#5000;
		end
	endtask

	task expect_seen(input integer falls_before, input [8*32-1:0] name);
		if (ready_falls == falls_before) fail({"not seen: ", name});
	endtask

	task all_in_reset(input [8*32-1:0] name);
		if (rst_n_clkin !== 0 || gddr_rst !== 1 || rst_n_clk !== 0 ||
		    blk_rstb !== 0 || lock_sync !== 0 || gddr_ready !== 0)
			fail({"not all in reset: ", name});
	endtask

	initial begin
		if (!$value$plusargs("seed=%d", seed)) seed = 1;

		if ($test$plusargs("late_clkin")) begin
			clkin_en = 0; clk_en = 0;
			#20000;
			all_in_reset("no MCO yet");
			mco_on;
			expect_up("MCO 20 us late");
		end else if ($test$plusargs("early_kuntur")) begin
			pll_lock = 1;
			expect_up("locked before wake-up");
		end else begin
			#5000;
			all_in_reset("MCO running, PLL not locked");
			$display("%t  ok    wake-up, PLL not locked", $realtime);
			acquire;
			expect_up("LOCK chatter");
		end

		// 30 ns LOCK glitch between clkin edges: the asynchronous clear
		// must catch it (decision 10)
		falls0 = ready_falls;
		@(posedge clkin); #10; pll_lock = 0; #0.1 all_in_reset("30 ns LOCK glitch");
		#29.9 pll_lock = 1;
		expect_seen(falls0, "30 ns LOCK glitch");
		expect_up("30 ns LOCK glitch");

		// LOCK low 50 us, clocks running
		falls0 = ready_falls;
		pll_lock = 0; #50000;
		all_in_reset("LOCK low 50 us");
		acquire;
		expect_seen(falls0, "LOCK low 50 us");
		expect_up("LOCK low 50 us");

		// rstb pulse 20 ns, between clkin edges
		falls0 = ready_falls;
		@(posedge clkin); #5; rstb = 0; #0.1 all_in_reset("20 ns rstb");
		#19.9 rstb = 1;
		expect_seen(falls0, "20 ns rstb");
		expect_up("20 ns rstb pulse");

		// rstb held 100 us
		rstb = 0; #100000; all_in_reset("rstb 100 us"); rstb = 1;
		expect_up("rstb 100 us");

		// MCO stops (MCU reset): clkin stops, the PLL output 1 us later,
		// LOCK falls 5 us later with no clock running anywhere
		falls0 = ready_falls;
		clkin_en = 0; #1000; clk_en = 0; #4000;
		pll_lock = 0; #0.1 all_in_reset("MCO stop, no clocks");
		#100000;
		all_in_reset("MCO stopped 100 us");
		expect_seen(falls0, "MCO stop");
		mco_on;
		expect_up("MCO stop");

		// MCO stops with LOCK falling first, PLL output still running
		falls0 = ready_falls;
		clkin_en = 0; #300; pll_lock = 0; #0.1 all_in_reset("MCO stop, LOCK first");
		#2000; clk_en = 0; #50000;
		expect_seen(falls0, "MCO stop, LOCK first");
		mco_on;
		expect_up("MCO stop, LOCK first");

		if (errors + chk_clkin.errors + chk_clk.errors + chk_blk.errors)
			$display("FAIL (%0d)", errors + chk_clkin.errors + chk_clk.errors + chk_blk.errors);
		else
			$display("PASS");
		$finish;
	end
endmodule


`ifdef REF_MODEL
//	Behavioural model of R-K7, testbench only: it stands in for the RTL
//	until reset_gen_kuntur exists in kuntur_fpga.v, and checks that the
//	testbench passes something that meets the spec. Its initial values
//	stand in for GSR (the RTL has none, R-K7 K1).
`ifndef MUT
`define MUT 0
`endif
module reset_gen_kuntur (
	input  wire clkin,
	input  wire rstb,
	input  wire pll_lock,
	input  wire clk,
	input  wire wclk,
	input  wire gddr_rdy,
	output wire lock_sync,
	output wire rst_n_clkin,
	output wire gddr_rst,
	output wire rst_n_clk,
	output wire blk_rstb
);
	localparam HOLDOFF = 32768;
	// decision 10: a lock drop clears at once, like rstb
	wire clr = (`MUT == 3) ? !rstb : (!rstb || !pll_lock);
	wire clr_lk = (`MUT == 4) ? !rstb : clr;

	reg [1:0] lk = 2'b00;
	always @(posedge clkin or posedge clr_lk)
		if (clr_lk) lk <= 2'b00;
		else lk <= {lk[0], pll_lock};
	assign lock_sync = (`MUT == 10) ? lk[0] : lk[1];

	// hold-off: cleared while lock_sync = 0, saturates at the target
	localparam [15:0] TARGET = (`MUT == 2) ? HOLDOFF - 2 : HOLDOFF;
	reg [15:0] cnt = 0;
	reg rnc = 1'b0;
	wire clr_cnt = (`MUT == 1) ? !rstb : clr;
	always @(posedge clkin or posedge clr_cnt)
		if (clr_cnt) cnt <= 0;
		else if (!lock_sync && `MUT != 1) cnt <= 0;
		else if (lock_sync && cnt != TARGET) cnt <= cnt + 1;
	always @(posedge clkin or posedge clr)
		if (clr) rnc <= 1'b0;
		else rnc <= (cnt == TARGET);
	assign rst_n_clkin = rnc;

	reg g = (`MUT == 5) ? 1'b0 : 1'b1;
	always @(posedge clkin or posedge clr)
		if (clr) g <= 1'b1;
		else g <= !rst_n_clkin;
	assign gddr_rst = (`MUT == 9) ? !rstb : g;

	// clk domain: asserted at once, released through 2 clk flops
	reg [1:0] cs = 2'b00;
	generate if (`MUT == 7) begin
		always @(posedge clk) cs <= rst_n_clkin ? {cs[0], 1'b1} : 2'b00;
	end else begin
		always @(posedge clk or negedge rst_n_clkin)
			if (!rst_n_clkin) cs <= 2'b00;
			else cs <= {cs[0], 1'b1};
	end endgenerate
	assign rst_n_clk = (`MUT == 6) ? cs[0] : cs[1];

	// block_gen: rst_n_clkin && READY, released through 2 wclk flops
	wire go = rst_n_clkin && gddr_rdy;
	reg [1:0] bs = 2'b00;
	always @(posedge wclk or negedge go)
		if (!go) bs <= 2'b00;
		else bs <= {bs[0], 1'b1};
	assign blk_rstb = (`MUT == 8) ? bs[0] : bs[1];
endmodule
`endif
