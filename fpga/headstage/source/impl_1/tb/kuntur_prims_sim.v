/*********************************************************************************
	Testbench-only behavioural models of the Lattice blocks kuntur_fpga.v
	instantiates, so the whole top level elaborates in Icarus without the
	Radiant simulation libraries. Not timing models: each one does just
	enough for the logic around it to run.

	  pll_25k,  the two PLL IPs clock_gen picks from by build variant (#6).
	  pll_28k   Same model for both, pll_sim: clkop = clki (the testbench's
	            clkin stands for the PLL output, so the rate is the
	            testbench's, not 37.95/42.504 MHz); lock_o rises
	            LOCK_CYCLES clki edges after start-up or after `unlock` is
	            released. A testbench drops lock with
	            <instance>.core.unlock = 1 (R-K2: the reset must follow at
	            once).
	            clkos = clki delayed by CLKOS_DELAY ns plus phase_pos / SPT
	            of a measured clki period. Dynamic phase (TN-02095 §14.6.1,
	            spi1-mosi-eye-scan.md §3.2): each PHASESTEP falling edge
	            with PHASESEL = 001 moves phase_pos by one, PHASEDIR 0 =
	            later; SPT 336 (pll_25k) / 296 (pll_28k), as the IPs; a lock
	            drop returns it to 0 (an assumption, unverified on the real
	            PLL: spi1-mosi-eye-scan.md §3.2 Reset). PHASESEL/PHASEDIR must be stable 5 ns
	            before the edge, and the pulse 4 VCO cycles each way;
	            n_step_viol counts breaches (VCO period = T / (SPT / 8)).
	  ECLKSYNC  ECLKOUT = ECLKIN, held low while STOP is high.
	  ECLKDIV   divide by 4 ("4" only), DIVOUT held low by DIVRST.
	  ODDRX4    D0..D7 latched on SCLK, sent D0 first on both ECLK edges.

	Compile it ahead of kuntur_tb.sv.
*********************************************************************************/

`timescale 1ns/1ps

module pll_sim #(
	parameter LOCK_CYCLES = 100,
	parameter real CLKOS_DELAY = 5.0,
	parameter SPT = 336
	)(
	input  wire       clki_i,
	input  wire       phasedir_i,
	input  wire       phasestep_i,
	input  wire [2:0] phasesel_i,
	output wire       clkop_o,
	output reg        clkos_o,
	output wire       lock_o
);
	reg unlock = 1'b0;			// driven by the testbench, hierarchically
	reg locked = 1'b0;
	integer cnt = 0;
	integer phase_pos = 0;
	integer n_step_viol = 0;
	realtime t_rise = 0.0, period = 0.0;
	realtime t_sel = 0.0, t_step = 0.0;

	assign clkop_o = clki_i;
	assign lock_o = locked & ~unlock;

	initial clkos_o = 1'b0;

	always @(posedge clki_i) begin
		if (t_rise > 0.0)
			period = $realtime - t_rise;
		t_rise = $realtime;
	end

	// transport delay, so a step that moves the edge past the next one
	// still gives every edge
	always @(clki_i)
		clkos_o <= #(CLKOS_DELAY + phase_pos * period / SPT) clki_i;

	always @(phasesel_i or phasedir_i)
		t_sel = $realtime;

	always @(phasestep_i)
		if (phasestep_i !== 1'bx) begin
			if (period > 0.0 && $realtime - t_step < 4.0 * period * 8.0 / SPT)
				n_step_viol = n_step_viol + 1;
			t_step = $realtime;
		end

	always @(negedge phasestep_i)
		if (phasesel_i == 3'b001 && !unlock) begin
			if ($realtime - t_sel < 5.0)
				n_step_viol = n_step_viol + 1;
			phase_pos = phasedir_i ? (phase_pos + SPT - 1) % SPT : (phase_pos + 1) % SPT;
		end

	always @(posedge clki_i or posedge unlock)
		if (unlock) begin
			cnt <= 0;
			locked <= 1'b0;
			phase_pos = 0;
		end
		else if (cnt == LOCK_CYCLES)
			locked <= 1'b1;
		else
			cnt <= cnt + 1;
endmodule


module pll_25k (
	input  wire       clki_i,
	input  wire       phasedir_i,
	input  wire       phasestep_i,
	input  wire       phaseloadreg_i,
	input  wire [2:0] phasesel_i,
	output wire       clkop_o,
	output wire       clkos_o,
	output wire       lock_o
);
	pll_sim #(.SPT(336)) core (
		.clki_i      (clki_i),
		.phasedir_i  (phasedir_i),
		.phasestep_i (phasestep_i),
		.phasesel_i  (phasesel_i),
		.clkop_o     (clkop_o),
		.clkos_o     (clkos_o),
		.lock_o      (lock_o)
	);
endmodule


module pll_28k (
	input  wire       clki_i,
	input  wire       phasedir_i,
	input  wire       phasestep_i,
	input  wire       phaseloadreg_i,
	input  wire [2:0] phasesel_i,
	output wire       clkop_o,
	output wire       clkos_o,
	output wire       lock_o
);
	pll_sim #(.SPT(296)) core (
		.clki_i      (clki_i),
		.phasedir_i  (phasedir_i),
		.phasestep_i (phasestep_i),
		.phasesel_i  (phasesel_i),
		.clkop_o     (clkop_o),
		.clkos_o     (clkos_o),
		.lock_o      (lock_o)
	);
endmodule


module ECLKSYNC #(
	parameter STOP_EN = "ENABLE"
	)(
	input  wire ECLKIN,
	input  wire STOP,
	output wire ECLKOUT
);
	assign ECLKOUT = ECLKIN & ~STOP;
endmodule


module ECLKDIV #(
	parameter ECLK_DIV = "4",
	parameter GSR = "DISABLED"
	)(
	input  wire DIVRST,
	input  wire ECLKIN,
	input  wire SLIP,
	output reg  DIVOUT
);
	reg c = 1'b0;

	initial DIVOUT = 1'b0;

	always @(posedge ECLKIN or posedge DIVRST)
		if (DIVRST) begin
			c <= 1'b0;
			DIVOUT <= 1'b0;
		end
		else begin
			c <= ~c;
			if (c)
				DIVOUT <= ~DIVOUT;
		end
endmodule


module ODDRX4 (
	input  wire D0, D1, D2, D3, D4, D5, D6, D7,
	input  wire SCLK,
	input  wire RST,
	input  wire ECLK,
	output reg  Q
);
	reg [7:0] word = 8'd0;
	reg [7:0] sh = 8'd0;
	integer k = 8;

	initial Q = 1'b0;

	always @(posedge SCLK or posedge RST)
		if (RST)
			word <= 8'd0;
		else
			word <= {D7, D6, D5, D4, D3, D2, D1, D0};

	always @(ECLK)
		if (RST)
			Q <= 1'b0;
		else begin
			if (k >= 8) begin
				sh = word;
				k = 0;
			end
			Q <= sh[k];
			k = k + 1;
		end
endmodule
