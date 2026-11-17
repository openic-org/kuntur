//testbench
//
//	Self-checking testbench for the two #6a eye-scan models
//	(spi1-mosi-eye-scan.md §6), before kuntur_tb relies on them:
//	  - pll_sim's dynamic phase (kuntur_prims_sim.v): one PHASESTEP
//	    falling edge moves clkos by T/SPT, PHASEDIR 1 moves it back, a
//	    full turn of SPT steps comes back to 0, a lock drop resets it,
//	    and pulses that break the TN-02095 timing are counted.
//	  - rhd2164_bfm's MOSI window: a command whose MOSI changes well away
//	    from SCLK rising (after MOSI_DELAY) decodes right; one that
//	    changes inside SETUP before or HOLD after decodes wrong, and
//	    n_viol counts it.
//
//	Run (QuestaSim), R = source/impl_1:
//	  vlib work
//	  vlog -sv +incdir+$R/tb +incdir+$R/afe/rhd2164 $R/tb/eye_models_tb.sv
//	  vsim -c eye_models_tb -do "run -all; quit -f"

`timescale 1ns/1ps
`include "kuntur_prims_sim.v"
`include "rhd2164_bfm.v"

module eye_models_tb();
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
	// pll_sim through pll_25k, at the real 25k clk period
	// ------------------------------------------------------------------
	localparam real T = 26.3504868239;
	localparam int SPT = 336;
	reg clk = 0;
	always #(T / 2) clk = ~clk;

	reg phasedir = 0, phasestep = 1;
	reg [2:0] phasesel = 3'b001;
	wire clkop, clkos, lock;

	pll_25k pll (
		.clki_i         (clk),
		.phasedir_i     (phasedir),
		.phasestep_i    (phasestep),
		.phaseloadreg_i (1'b0),
		.phasesel_i     (phasesel),
		.clkop_o        (clkop),
		.clkos_o        (clkos),
		.lock_o         (lock)
	);

	// clkos rising minus clkop rising, in ns (0 .. T)
	task automatic measure(output real d);
		realtime t0;
		@(posedge clkop);
		t0 = $realtime;
		@(posedge clkos);
		d = $realtime - t0;
		while (d >= T - 0.001)
			d = d - T;
	endtask

	// n steps, each one clk low then one clk high (the RTL's pulse)
	task automatic steps(int n);
		repeat (n) begin
			@(posedge clk) phasestep <= 0;
			@(posedge clk) phasestep <= 1;
		end
		repeat (4) @(posedge clk);
	endtask

	// ------------------------------------------------------------------
	// rhd2164_bfm with a window: MOSI 3 ns late at the chip, 8 ns setup,
	// 6 ns hold
	// ------------------------------------------------------------------
	reg csb = 1, sck = 0, mosi = 0;
	wire miso;
	rhd2164_bfm #(.CHIP(0), .MOSI_DELAY(3.0), .SETUP(8.0), .HOLD(6.0)) chip (
		.csb  (csb),
		.sck  (sck),
		.mosi (mosi),
		.miso (miso)
	);

	// One 16-bit transfer, SCLK period 40 ns (rising at 20, 60, ...); MOSI
	// changes `lead` ns before each rising edge (negative: after it).
	task automatic xfer(input [15:0] w, input real lead);
		real start;
		csb = 0;
		#20;
		start = $realtime;
		for (int i = 15; i >= 0; i--) begin
			// bit i is taken at start + 20 + 40*(15-i)
			fork
				begin
					#(20.0 + 40.0 * (15 - i) - lead - ($realtime - start));
					mosi = w[i];
				end
				begin
					#(20.0 + 40.0 * (15 - i) - ($realtime - start));
					sck = 1;
					#20 sck = 0;
				end
			join
		end
		#30 csb = 1;
		#100;
	endtask

	real d, d0;
	int v0;

	initial begin
		$display("--- pll_sim dynamic phase (pll_25k, SPT %0d)", SPT);
		wait (lock === 1'b1);
		measure(d0);
		check("static clkos delay is CLKOS_DELAY", d0 > 4.99 && d0 < 5.01, $sformatf("%.3f ns", d0));
		steps(1);
		measure(d);
		check("one step = T/SPT later", (d - d0 - T / SPT) < 0.002 && (d - d0 - T / SPT) > -0.002,
			$sformatf("%.4f ns (want %.4f)", d - d0, T / SPT));
		steps(83);
		measure(d);
		check("84 steps = a quarter turn", (d - d0 - T / 4) < 0.002 && (d - d0 - T / 4) > -0.002,
			$sformatf("%.4f ns (want %.4f)", d - d0, T / 4));
		check("phase_pos counts the steps", pll.core.phase_pos == 84, $sformatf("%0d", pll.core.phase_pos));
		phasedir = 1;
		repeat (2) @(posedge clk);
		steps(4);
		check("PHASEDIR 1 steps back", pll.core.phase_pos == 80, $sformatf("%0d", pll.core.phase_pos));
		phasedir = 0;
		repeat (2) @(posedge clk);
		steps(SPT - 80);
		measure(d);
		check("a full turn comes back to 0", pll.core.phase_pos == 0 && (d - d0) < 0.002 && (d - d0) > -0.002,
			$sformatf("pos %0d, %.4f ns", pll.core.phase_pos, d - d0));
		check("RTL-style pulses break no timing", pll.core.n_step_viol == 0, $sformatf("%0d", pll.core.n_step_viol));
		phasesel = 3'b000;
		repeat (2) @(posedge clk);
		steps(3);
		check("PHASESEL 000 does not move CLKOS", pll.core.phase_pos == 0, $sformatf("%0d", pll.core.phase_pos));
		phasesel = 3'b001;
		v0 = pll.core.n_step_viol;
		@(posedge clk);
		phasestep <= 0;		// PHASESEL changed 1 clk ago: fine; now a 1 ns pulse
		#1 phasestep <= 1;
		repeat (4) @(posedge clk);
		check("a 1 ns PHASESTEP pulse is flagged", pll.core.n_step_viol > v0, $sformatf("%0d", pll.core.n_step_viol - v0));
		steps(10);
		pll.core.unlock = 1;
		#100 pll.core.unlock = 0;
		check("a lock drop returns phase_pos to 0", pll.core.phase_pos == 0, $sformatf("%0d", pll.core.phase_pos));

		$display("--- rhd2164_bfm MOSI window (delay 3, setup 8, hold 6 ns)");
		// The chip sees a change at lead - 3 ns before SCLK rising.
		// Clean: 3 + 8 < lead < 40 - 6 + 3 (the next edge's hold).
		xfer(16'hE800, 20.0);		// READ(40), changes 17 ns before: clean
		check("clean READ(40) decodes", chip.last_cmd == 16'hE800 && chip.n_viol == 0,
			$sformatf("%04h, %0d violations", chip.last_cmd, chip.n_viol));
		xfer(16'h8655, 12.0);		// WRITE(6, 55), 9 ns before at the chip: clean
		check("WRITE(6,55) with 1 ns of setup margin decodes", chip.last_cmd == 16'h8655 && chip.n_viol == 0,
			$sformatf("%04h, %0d", chip.last_cmd, chip.n_viol));
		xfer(16'h86AA, 9.0);		// 6 ns before at the chip: setup broken
		check("setup violation corrupts the command", chip.last_cmd != 16'h86AA && chip.n_viol > 0,
			$sformatf("%04h, %0d violations", chip.last_cmd, chip.n_viol));
		v0 = chip.n_viol;
		xfer(16'hE800, 0.0);		// changes 3 ns after SCLK rising at the chip: hold broken
		check("hold violation corrupts the command", chip.last_cmd != 16'hE800 && chip.n_viol > v0,
			$sformatf("%04h, %0d violations", chip.last_cmd, chip.n_viol - v0));
		v0 = chip.n_viol;
		xfer(16'hE800, -4.0);		// 7 ns after at the chip: past hold, so each bit
		// lands on the NEXT edge: the word shifts by one (the scan's far end)
		check("past the hold, the word arrives one bit late", chip.last_cmd == 16'h7400 && chip.n_viol == v0,
			$sformatf("%04h, %0d", chip.last_cmd, chip.n_viol - v0));

		$display("\n  %0d checks, %0d failures", checks, errors);
		if (errors == 0)
			$display("  *** PASS ***");
		else
			$display("  *** FAIL ***");
		$finish;
	end
endmodule
