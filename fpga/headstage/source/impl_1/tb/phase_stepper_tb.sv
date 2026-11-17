//testbench
//
//	Self-checking testbench for phase_stepper (task #6a,
//	spi1-mosi-eye-scan.md §3.2), driving the pll_sim model of the
//	rate's PLL (kuntur_prims_sim.v), so every command also checks that
//	PHASE_POS and the model's CLKOS phase agree:
//	  - reset values; N = 0 does nothing
//	  - N delaying steps = N PHASESTEP falling edges, PHASE_POS + N mod SPT;
//	    direction 1 = N back, mod SPT; both wraps; a full turn; N = 1023
//	  - pulses one clk low, at least one clk high; PHASEDIR stable 5 ns
//	    before every falling edge (and the model's own TN-02095 checks)
//	  - busy high from the clk after the write to the last pulse's rising
//	    edge (2N or 2N + 1 clk), low after; a write
//	    while busy is ignored; a write the clk after busy falls is taken
//	  - no pulses while idle (2100 clk, past an 11-bit counter's wrap)
//	  - rst_n mid-run: PHASESTEP high at once, PHASE_POS 0, busy 0.
//	    With rst_n alone (the rstb pin) the model PLL keeps its phase:
//	    the counter and CLKOS then disagree, as §3.2 says; reported, not
//	    an error. A lock drop with rst_n (rst_n_clk follows the lock)
//	    returns both to 0 under the model's assumption (§3.2).
//
//	RATE = 25 (pll_25k, SPT 336) or 28 (pll_28k, SPT 296), at that rate's
//	clk period. Run (QuestaSim), R = source/impl_1:
//	  vlib work
//	  vlog -sv +define+KUNTUR_VARIANT=4'd2 +incdir+$R +incdir+$R/tb \
//	       +incdir+$R/afe/rhd2164 +incdir+$R/app $R/tb/phase_stepper_tb.sv
//	  vsim -c phase_stepper_tb -gRATE=28 -do "run -all; quit -f"
//	or fpga/scripts/run_kuntur_tb.sh, which runs both rates.

`timescale 1ns/1ps
`include "kuntur_prims_sim.v"
`include "../kuntur_fpga.v"

module phase_stepper_tb #(
	parameter int RATE = 25
)();
	localparam real T = (RATE == 28) ? 23.5271907979 : 26.3504868239;
	localparam int SPT = (RATE == 28) ? 296 : 336;

	int checks = 0, errors = 0;

	task automatic check(string name, bit ok, string got = "");
		checks++;
		if (ok)
			$display("  pass  %0s %0s", name, got);
		else begin
			errors++;
			$error("%0s %0s", name, got);
		end
	endtask

	// ------------------------------------------------------------------
	// clk, the model PLL, the stepper
	// ------------------------------------------------------------------
	reg clkin = 0;
	always #(T / 2) clkin = ~clkin;

	reg rst_n = 0, cmd_wr = 0;
	reg [15:0] cmd = 0;
	wire clk, clkos, lock;
	wire phasestep, phasedir, busy;
	wire [8:0] phase_pos;

	generate
		if (RATE == 28) begin : g_pll
			pll_28k pll (.clki_i(clkin), .phasedir_i(phasedir), .phasestep_i(phasestep),
				.phaseloadreg_i(1'b0), .phasesel_i(3'b001),
				.clkop_o(clk), .clkos_o(clkos), .lock_o(lock));
		end
		else begin : g_pll
			pll_25k pll (.clki_i(clkin), .phasedir_i(phasedir), .phasestep_i(phasestep),
				.phaseloadreg_i(1'b0), .phasesel_i(3'b001),
				.clkop_o(clk), .clkos_o(clkos), .lock_o(lock));
		end
	endgenerate

	phase_stepper dut (
		.rst_n		(rst_n),
		.clk		(clk),
		.cmd_wr		(cmd_wr),
		.cmd		(cmd),
		.spt		(SPT[11:0]),
		.phasestep	(phasestep),
		.phasedir	(phasedir),
		.phase_pos	(phase_pos),
		.busy		(busy)
	);

	// ------------------------------------------------------------------
	// Monitors: pulses, widths, PHASEDIR set-up, busy at the edges
	// ------------------------------------------------------------------
	int falls = 0, n_width = 0, n_dir_setup = 0, n_not_busy = 0;
	realtime t_fall = 0, t_rise = -1000, t_dir = -1000;
	bit dir_at_fall;

	always @(phasedir) t_dir = $realtime;

	always @(negedge phasestep) if (rst_n) begin
		falls++;
		dir_at_fall = phasedir;
		if ($realtime - t_rise < T - 0.01) n_width++;		// high < one clk
		if ($realtime - t_dir < 5.0) n_dir_setup++;		// TN-02095: 5 ns
		if (!busy) n_not_busy++;
		t_fall = $realtime;
	end

	always @(posedge phasestep) if (rst_n) begin
		if ($realtime - t_fall < T - 0.01) n_width++;		// low < one clk
		t_rise = $realtime;
	end

	// busy rose at all since the last write (for N = 0 and ignored writes)
	bit busy_seen;
	always @(posedge clk) if (busy) busy_seen = 1;

	function automatic int model_pos();
		return g_pll.pll.core.phase_pos;
	endfunction

	// ------------------------------------------------------------------
	// Commands
	// ------------------------------------------------------------------
	// One regbank write of PHASE_STEP_CMD: cmd_wr for one clk.
	task automatic wr(input [9:0] n, input bit dir);
		@(posedge clk);
		cmd <= {dir, 5'd0, n};
		cmd_wr <= 1;
		@(posedge clk);
		cmd_wr <= 0;
	endtask

	// Write, wait 2N + 8 clk, check the steps, the position, the model.
	task automatic do_cmd(string name, input int n, input bit dir);
		int f0, p0, want;
		f0 = falls;
		p0 = phase_pos;
		busy_seen = 0;
		wr(n, dir);
		repeat (2 * n + 8) @(posedge clk);
		want = dir ? (p0 - n % SPT + SPT) % SPT : (p0 + n) % SPT;
		check({name, ": falling edges"}, falls - f0 == n, $sformatf("%0d (want %0d)", falls - f0, n));
		check({name, ": PHASE_POS"}, phase_pos == want, $sformatf("%0d (want %0d)", phase_pos, want));
		check({name, ": model PLL agrees"}, model_pos() == phase_pos,
			$sformatf("model %0d, stepper %0d", model_pos(), phase_pos));
		check({name, ": idle after"}, busy === 1'b0 && phasestep === 1'b1,
			$sformatf("busy %b, phasestep %b", busy, phasestep));
		if (n > 0)
			check({name, ": busy seen"}, busy_seen);
		if (n > 0)
			check({name, ": PHASEDIR at the edges"}, dir_at_fall == dir, $sformatf("%b", dir_at_fall));
	endtask

	int f0, p0, busy_cycles;

	initial begin
		$display("--- phase_stepper, %0dk: T = %.3f ns, SPT %0d", RATE, T, SPT);
		#1;
		check("reset: PHASESTEP high, PHASEDIR 0, PHASE_POS 0, not busy",
			phasestep === 1'b1 && phasedir === 1'b0 && phase_pos === 9'd0 && busy === 1'b0,
			$sformatf("%b %b %0d %b", phasestep, phasedir, phase_pos, busy));
		wait (lock === 1'b1);
		repeat (4) @(posedge clk);
		rst_n = 1;
		repeat (4) @(posedge clk);

		// N = 0
		f0 = falls;
		busy_seen = 0;
		wr(0, 0);
		repeat (10) @(posedge clk);
		check("N = 0: no edge, no busy, PHASE_POS 0", falls == f0 && !busy_seen && phase_pos == 0,
			$sformatf("%0d edges, busy seen %b, pos %0d", falls - f0, busy_seen, phase_pos));

		do_cmd("N = 1 delay", 1, 0);
		do_cmd("N = 3 delay", 3, 0);

		// busy covers the whole command: the set-up clk, then N pulses up to
		// the last rising edge (2N), or also that last high clk (2N + 1)
		busy_cycles = 0;
		fork
			wr(5, 0);
			repeat (30) @(posedge clk) if (busy) busy_cycles++;
		join
		check("N = 5: busy for 2N or 2N + 1 clk", busy_cycles == 10 || busy_cycles == 11,
			$sformatf("%0d", busy_cycles));
		check("N = 5: PHASE_POS 9", phase_pos == 9, $sformatf("%0d", phase_pos));

		// a write while busy is ignored
		f0 = falls;
		wr(5, 0);
		repeat (3) @(posedge clk);
		wr(7, 1);
		repeat (40) @(posedge clk);
		check("write while busy ignored: 5 edges, PHASE_POS 14", falls - f0 == 5 && phase_pos == 14,
			$sformatf("%0d edges, pos %0d", falls - f0, phase_pos));

		// a write the clk after busy falls is taken
		f0 = falls;
		wr(2, 0);
		@(posedge clk);
		while (busy) @(posedge clk);
		wr(3, 0);
		repeat (20) @(posedge clk);
		check("write right after busy: taken (2 + 3 edges)", falls - f0 == 5 && phase_pos == 19,
			$sformatf("%0d edges, pos %0d", falls - f0, phase_pos));

		do_cmd("N = 2 advance (direction 1)", 2, 1);
		do_cmd("delay to SPT - 1", SPT - 1 - phase_pos, 0);
		do_cmd("wrap SPT - 1 -> 0", 1, 0);
		do_cmd("advance wrap 0 -> SPT - 1", 1, 1);
		do_cmd("delay back to 0", 1, 0);
		do_cmd("full turn (N = SPT)", SPT, 0);
		do_cmd("N = 1023 (the most one write takes)", 1023, 0);
		do_cmd("advance across 0", 1023 % SPT + 5, 1);

		// idle: no pulses past an 11-bit counter's wrap
		f0 = falls;
		repeat (2100) @(posedge clk);
		check("idle 2100 clk: no edge", falls == f0, $sformatf("%0d", falls - f0));

		check("PHASESTEP low one clk, high at least one", n_width == 0, $sformatf("%0d short", n_width));
		check("PHASEDIR stable 5 ns before every edge", n_dir_setup == 0, $sformatf("%0d late", n_dir_setup));
		check("busy high at every falling edge", n_not_busy == 0, $sformatf("%0d edges not busy", n_not_busy));
		check("model PLL: no TN-02095 timing breach", g_pll.pll.core.n_step_viol == 0,
			$sformatf("%0d", g_pll.pll.core.n_step_viol));

		// rst_n mid-run (the rstb pin: the PLL is not reset)
		do_cmd("set up: back to 0", (SPT - phase_pos) % SPT, 0);
		wr(50, 0);
		repeat (20) @(posedge clk);
		#(T / 4) rst_n = 0;
		#1;
		check("rst_n mid-run: PHASESTEP high at once, PHASE_POS 0, not busy",
			phasestep === 1'b1 && phase_pos === 9'd0 && busy === 1'b0,
			$sformatf("%b %0d %b", phasestep, phase_pos, busy));
		$display("  note  after rst_n alone, the model PLL is at %0d, the counter at 0 (§3.2: reconfigure)",
			model_pos());
		repeat (4) @(posedge clk);
		rst_n = 1;
		repeat (4) @(posedge clk);
		p0 = model_pos();
		f0 = falls;
		wr(1, 0);
		repeat (10) @(posedge clk);
		check("after rst_n: steps again from 0", falls - f0 == 1 && phase_pos == 1 && model_pos() == (p0 + 1) % SPT,
			$sformatf("%0d edges, pos %0d, model %0d -> %0d", falls - f0, phase_pos, p0, model_pos()));

		// lock drop with rst_n: both return to 0 (the model's assumption)
		g_pll.pll.core.unlock = 1;
		rst_n = 0;
		#100 g_pll.pll.core.unlock = 0;
		wait (lock === 1'b1);
		repeat (4) @(posedge clk);
		rst_n = 1;
		repeat (4) @(posedge clk);
		check("lock drop with rst_n: counter and model both 0", phase_pos == 0 && model_pos() == 0,
			$sformatf("pos %0d, model %0d", phase_pos, model_pos()));
		do_cmd("after the lock drop: N = 4", 4, 0);

		$display("\n  %0d checks, %0d failures", checks, errors);
		if (errors == 0)
			$display("  *** PASS ***");
		else
			$display("  *** FAIL ***");
		$finish;
	end

	initial begin
		#20ms;
		$error("timeout");
		$finish;
	end
endmodule
