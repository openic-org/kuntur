//testbench
//
//	Testbench for the companion's reset generator.
//	Contract: vega docs/interfaces/system-startup.md §2.3 (R-C1, R-C2, R-C8)
//	and lvds-tunnel.md §6.4 (gddr_sync connections).
//
//	DUT: reset_gen_adapter (interface in R-C8). gddr_sync is Lattice's
//	own, copied unmodified into headstage/source/impl_1/common/ from
//	Radiant 2025.2.1 ip/lifcl/ddr/rtl/, wired as in §6.4:
//	SYNC_CLK = clkin, START = lock_sync, RST = gddr_rst; its STOP gates the
//	modelled wclk, as ECLKSYNC does.
//
//	Checks, all running for the whole simulation:
//	  rst_n_sup  initial 0; 0 at once whenever rstb = 0; released on a
//	             clkin edge after SUP_CYCLES..SUP_CYCLES+4 edges of rstb
//	             high; released if rstb stays high.          (R-C1, R-C8)
//	  lock_sync  initial 0; pll_lock through exactly 2 clkin flops;
//	             0 at once whenever rstb = 0.
//	  gddr_rst   = !rst_n_sup || !lock_sync, registered (1 clkin late);
//	             initial 1; 1 at once whenever rstb = 0.
//	  READY      rises 20..26 clkin edges after rst_n_sup && lock_sync rose
//	             (23 measured 2026-10-28), and does rise if both hold.
//	             After every lock drop that lock_sync saw, READY returns.
//	  cdr_rstb   initial 0; 0 at once when rst_n_sup or READY falls, with
//	             no wclk edge; released on a wclk edge after 2..3 edges.
//	  rst_n_emu  = rstb at all times, from t = 0, with no clock at all:
//	             never touched by lock, READY or wclk.      (R-C1, R-C2)
//	  Each scenario ends with the receiver out of reset (cdr_rstb = 1).
//
//	Scenarios (one run): wake-up with no Kuntur; Kuntur on with LOCK
//	chattering during acquisition; a sub-cycle LOCK glitch (may or may not
//	be seen); a 2-cycle drop (must be seen); cable out for 50 us; a 20 ns
//	rstb pulse between clkin edges; SW2 held 100 us; SW2 pressed with the
//	cable out. Variants (plusargs):
//	  +late_clkin    clkin starts 20 us after wake-up, Kuntur already on
//	  +early_kuntur  Kuntur locked before wake-up
//	  +seed=<n>      LOCK chatter pattern (default 1)
//
//	Until the RTL exists, -DREF_MODEL compiles a behavioural model of
//	R-C8 (below, testbench only). -DREF_MODEL -DMUT=<k> breaks it on
//	purpose; every mutant must FAIL:
//	  1 supervisor released without the counter
//	  2 cdr_rstb through 1 wclk flop
//	  3 cdr_rstb asserted synchronously
//	  4 gddr_sync RST = !rstb alone (READY never returns after a drop)
//	  9 gddr_rst cleared (not set) by rstb, so 0 at wake-up
//	  5 emulator reset gated by READY
//	  6 lock_sync initial value 1
//	  7 lock_sync through 1 flop
//	  8 lock_sync not cleared by rstb (initial value only)
//
//	Run (Icarus, from this directory):
//	  K=../../../../headstage/source/impl_1
//	  G=$K/common/gddr_sync.v
//	  R=$K/tb/rst_check.sv     (shared with the Kuntur generator's testbench)
//	  iverilog -g2012 -Wall -DREF_MODEL -o rg.vvp reset_gen_adapter_tb.sv $G $R
//	  vvp rg.vvp; vvp rg.vvp +late_clkin; vvp rg.vvp +early_kuntur
//	Against the RTL, once it is in intan_rec_adapter.v:
//	  awk '/^module reset_gen_adapter /,/^endmodule/' ../intan_rec_adapter.v > /tmp/rg_rtl.v
//	  iverilog -g2012 -Wall -o rg.vvp reset_gen_adapter_tb.sv /tmp/rg_rtl.v $G $R
//	Each run prints PASS or FAIL last. The RTL passed all three, and
//	+seed=7, 42, 1234, on 2026-10-29.

`timescale 1ns/1ps

module reset_gen_adapter_tb;
	localparam SUP_CYCLES = 20;		// the RTL's count (R-C1 asks for ~16)
	localparam real T_CLKIN = 83.333;	// 12 MHz
	localparam real T_WCLK  = 105.4;		// ECLK / 4 at 37.95 MHz (87.8 at 45.54 before #6b)

	reg rstb = 1, pll_lock = 0, tun_on = 0, clkin_en = 1;
	reg clkin = 0, wclk = 0;
	wire rst_n_sup, lock_sync, gddr_rst, cdr_rstb, rst_n_emu;
	wire gddr_stop, gddr_ddr_reset, gddr_ready;
	integer errors = 0, seed = 1, ready_falls = 0;

	always #(T_CLKIN / 2) clkin = clkin_en ? ~clkin : 1'b0;
	// wclk exists only with TUN_CLK present and ECLKSYNC not stopped
	always #(T_WCLK / 2) if (tun_on && gddr_stop === 1'b0) wclk = ~wclk;

	reset_gen_adapter dut (
		.clkin(clkin), .rstb(rstb), .pll_lock(pll_lock),
		.wclk(wclk), .gddr_rdy(gddr_ready),
		.rst_n_sup(rst_n_sup), .lock_sync(lock_sync), .gddr_rst(gddr_rst),
		.cdr_rstb(cdr_rstb), .rst_n_emu(rst_n_emu));

	// 1 ps on gddr_sync's RST stands in for GSR: at t = 0 gddr_rst is
	// already 1 before the IP's always block is armed, which would leave
	// its state X until the first clkin edge (none for 20 us with
	// +late_clkin). On the FPGA, wake-up sets those flops.
	wire #0.001 gddr_rst_ip = gddr_rst;

	gddr_sync sync0 (
		.rst(gddr_rst_ip), .sync_clk(clkin), .start(lock_sync),
		.stop(gddr_stop), .ddr_reset(gddr_ddr_reset), .ready(gddr_ready));

	// --- GSR stand-in. On the FPGA the wake-up GSR sets every flop to its
	//     REGSET, which synthesis takes from the flop's reset value
	//     (system-startup.md R-C8); the RTL carries no initial values, so
	//     the testbench sets the same values at t = 0. They are the
	//     required power-up values, not copied from the RTL's resets: a
	//     wrong reset value is caught by the rstb checks, a wrong power-up
	//     value by check_powerup.py on the netlist. Internal names follow
	//     the RTL; a rename fails to compile.
`ifndef REF_MODEL
	initial begin
		dut.cnt0         = 5'd0;
		dut.rst_n_sup    = 1'b0;
		dut.pll_lock_reg = 1'b0;
		dut.lock_sync    = 1'b0;
		dut.gddr_rst     = 1'b1;
		dut.cdr_rstb_1   = 1'b0;
		dut.cdr_rstb     = 1'b0;
	end
`endif

	// --- generic reset checks: async assert, sync release after MIN..MAX edges
	rst_check #(.NAME("rst_n_sup"), .MIN(SUP_CYCLES), .MAX(SUP_CYCLES + 4))
		chk_sup (.clk(clkin), .cond(rstb), .rst_n(rst_n_sup));
	rst_check #(.NAME("cdr_rstb"), .MIN(2), .MAX(3))
		chk_cdr (.clk(wclk), .cond(rst_n_sup && gddr_ready), .rst_n(cdr_rstb));

	task fail(input [8*64-1:0] what);
		begin errors = errors + 1; $display("%t  FAIL  %0s", $realtime, what); end
	endtask

	initial $timeformat(-9, 1, " ns", 14);

	// gddr_sync's outputs are known from 1 ns on
	always @(gddr_stop, gddr_ddr_reset, gddr_ready)
		if ($realtime > 1.0 && ^{gddr_stop, gddr_ddr_reset, gddr_ready} === 1'bx)
			fail("gddr_sync output X");

	// --- lock_sync: pll_lock through exactly 2 clkin flops, initial 0,
	//     cleared at once by rstb (G4)
	reg lock_at_prev_edge = 0, lock_at_edge;
	initial #0.001 if (lock_sync !== 1'b0) fail("lock_sync not 0 at wake-up");
	always @(posedge clkin) begin
		lock_at_edge = pll_lock && rstb;
		#0.001 if (lock_sync !== lock_at_prev_edge) fail("lock_sync is not pll_lock through 2 flops");
		lock_at_prev_edge = lock_at_edge;
	end
	always @(negedge rstb) begin
		lock_at_prev_edge = 0;
		#0.001 if (lock_sync !== 1'b0) fail("lock_sync not cleared by rstb");
	end

	// --- gddr_sync RST: !rst_n_sup || !lock_sync registered on clkin
	//     (one clkin late), 1 at wake-up, 1 at once whenever rstb = 0
	reg rst_want;
	initial #0.001 if (gddr_rst !== 1'b1) fail("gddr_rst not 1 at wake-up");
	always @(negedge rstb) #0.001 if (gddr_rst !== 1'b1) fail("gddr_rst not 1 at once with rstb");
	always @(posedge clkin) begin
		rst_want = !rstb || !rst_n_sup || !lock_sync;
		#0.001 if (gddr_rst !== rst_want) fail("gddr_rst != !rst_n_sup || !lock_sync, 1 clkin late");
	end

	// --- emulator reset: rstb and nothing else, from t = 0
	initial #0.001 if (rst_n_emu !== 1'b1) fail("rst_n_emu not 1 at wake-up (no clock)");
	always @(rst_n_emu, rstb)
		#0.001 if (rst_n_emu !== rstb) fail("rst_n_emu != rstb");

	// --- READY timing and liveness, counted in clkin edges of
	//     rst_n_sup && lock_sync continuously high
	integer up_edges = 0;
	wire sync_go = rst_n_sup && lock_sync;
	always @(posedge clkin) begin
		up_edges = sync_go ? up_edges + 1 : 0;
		if (up_edges == 40 && !gddr_ready) fail("READY not up 40 clkin after lock_sync");
	end
	always @(negedge sync_go) up_edges = 0;
	always @(posedge gddr_ready)
		if (up_edges < 20 || up_edges > 26) fail("READY rose outside 20..26 clkin of lock_sync");
		else $display("%t  READY after %0d clkin edges", $realtime, up_edges);
	always @(negedge gddr_ready) ready_falls = ready_falls + 1;

	// --- stimulus
	integer i, falls0;

	// LOCK chatters while the PLL acquires: highs up to ~36 clkin, so that
	// READY sometimes rises and is then pulled down mid-run
	task kuntur_on;
		begin
			tun_on = 1;
			#2000;
			for (i = 0; i < 8; i = i + 1) begin
				pll_lock = 1; #(30 + ($unsigned($random(seed)) % 3000));
				pll_lock = 0; #(30 + ($unsigned($random(seed)) % 800));
			end
			pll_lock = 1;
		end
	endtask

	task expect_up(input [8*32-1:0] name);
		begin
			fork : w
				begin wait (cdr_rstb === 1'b1); disable w; end
				begin #20000; disable w; end
			join
			if (cdr_rstb !== 1'b1) fail({"receiver not out of reset: ", name});
			else $display("%t  ok    %0s", $realtime, name);
			#5000;
		end
	endtask

	initial begin
		if (!$value$plusargs("seed=%d", seed)) seed = 1;

		if ($test$plusargs("late_clkin")) begin
			clkin_en = 0;
			fork kuntur_on; join_none
			#20000;
			if (rst_n_sup !== 0 || gddr_rst !== 1 || cdr_rstb !== 0) fail("released with no clkin");
			clkin_en = 1;
			expect_up("late clkin, Kuntur already on");
		end else if ($test$plusargs("early_kuntur")) begin
			tun_on = 1; pll_lock = 1;
			expect_up("Kuntur locked before wake-up");
		end else begin
			#5000;
			if (rst_n_sup !== 1) fail("supervisor not released, no Kuntur");
			if (gddr_ready !== 0 || cdr_rstb !== 0) fail("receiver up with no Kuntur");
			$display("%t  ok    wake-up, no Kuntur", $realtime);
			kuntur_on;
			expect_up("Kuntur on, LOCK chatter");
		end

		// sub-cycle LOCK glitch, between clkin edges: may be missed
		@(posedge clkin); #10; pll_lock = 0; #30; pll_lock = 1;
		expect_up("30 ns LOCK glitch");

		// 2-cycle drop: lock_sync must see it, READY must fall and return
		falls0 = ready_falls;
		@(posedge clkin); #10; pll_lock = 0; #(2.5 * T_CLKIN); pll_lock = 1;
		#1000 if (ready_falls == falls0) fail("2-cycle LOCK drop not seen");
		expect_up("2-cycle LOCK drop");

		// cable out 50 us: no TUN_CLK, no wclk, no lock
		tun_on = 0; pll_lock = 0; #50000;
		if (cdr_rstb !== 0 || gddr_ready !== 0) fail("receiver up with the cable out");
		kuntur_on;
		expect_up("cable out 50 us");

		// rstb pulse 20 ns, between clkin edges
		@(posedge clkin); #20; rstb = 0; #20; rstb = 1;
		expect_up("20 ns rstb pulse");

		// SW2 held 100 us mid-session
		rstb = 0; #100000; rstb = 1;
		expect_up("SW2 100 us");

		// SW2 with the cable out, cable back after
		tun_on = 0; pll_lock = 0; #5000;
		rstb = 0; #30000; rstb = 1; #10000;
		if (rst_n_sup !== 1 || cdr_rstb !== 0) fail("after SW2, cable out");
		kuntur_on;
		expect_up("SW2 with cable out");

		if (errors + chk_sup.errors + chk_cdr.errors)
			$display("FAIL (%0d)", errors + chk_sup.errors + chk_cdr.errors);
		else
			$display("PASS");
		$finish;
	end
endmodule


`ifdef REF_MODEL
//	Behavioural model of R-C8, testbench only: it stands in for the RTL
//	until reset_gen_adapter exists in intan_rec_adapter.v, and checks
//	that the testbench passes something that meets the spec.
`ifndef MUT
`define MUT 0
`endif
module reset_gen_adapter (
	input  wire clkin,
	input  wire rstb,
	input  wire pll_lock,
	input  wire wclk,
	input  wire gddr_rdy,
	output wire rst_n_sup,
	output wire lock_sync,
	output wire gddr_rst,
	output wire cdr_rstb,
	output wire rst_n_emu
);
	localparam SUP_CYCLES = 20;
	// supervisor: rstb synchronised, then SUP_CYCLES clkin
	reg [1:0] rstb_s = 2'b00;
	reg [4:0] cnt = 0;
	reg sup = 1'b0;
	always @(posedge clkin or negedge rstb)
		if (!rstb) rstb_s <= 2'b00;
		else rstb_s <= {rstb_s[0], 1'b1};
	always @(posedge clkin or negedge rstb)
		if (!rstb) begin cnt <= 0; sup <= 1'b0; end
		else if (rstb_s[1]) begin
			if (cnt != SUP_CYCLES) cnt <= cnt + 1;
			else sup <= 1'b1;
		end
	assign rst_n_sup = (`MUT == 1) ? rstb_s[1] : sup;

	// lock through 2 clkin flops. The async clear is there for synthesis:
	// it makes them FD1P3DX (REGSET = RESET), so the wake-up GSR sets 0.
	reg [1:0] lk = (`MUT == 6) ? 2'b11 : 2'b00;
	generate if (`MUT == 8) begin
		always @(posedge clkin) lk <= {lk[0], pll_lock};
	end else begin
		always @(posedge clkin or negedge rstb)
			if (!rstb) lk <= 2'b00;
			else lk <= {lk[0], pll_lock};
	end endgenerate
	assign lock_sync = (`MUT == 7) ? lk[0] : lk[1];

	reg rst_q = (`MUT == 9) ? 1'b0 : 1'b1;
	always @(posedge clkin or negedge rstb)
		if (!rstb) rst_q <= (`MUT == 9) ? 1'b0 : 1'b1;
		else rst_q <= !rst_n_sup || !lock_sync;
	assign gddr_rst = (`MUT == 4) ? !rstb : rst_q;

	// receiver: asserted at once, released through 2 wclk flops
	wire cdr_go = rst_n_sup && gddr_rdy;
	reg [1:0] cs = 2'b00;
	generate if (`MUT == 3) begin
		always @(posedge wclk) cs <= cdr_go ? {cs[0], 1'b1} : 2'b00;
	end else begin
		always @(posedge wclk or negedge cdr_go)
			if (!cdr_go) cs <= 2'b00;
			else cs <= {cs[0], 1'b1};
	end endgenerate
	assign cdr_rstb = (`MUT == 2) ? cs[0] : cs[1];

	// emulator: rstb only
	assign rst_n_emu = (`MUT == 5) ? (rstb && gddr_rdy) : rstb;
endmodule
`endif
