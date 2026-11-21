/*********************************************************************************
	Testbench-only behavioural models of the Lattice blocks intan_rec_adapter.v
	instantiates, so the whole adapter top level elaborates without the
	Radiant simulation libraries (tunnel_e2e_tb.sv). Not timing models.
	Same conventions as cdr_tb.sv's inline models.

	  pll0      The receive PLL IP (37.95 MHz in, CLKOS dynamic phase).
	            clkop_o = clki_i; clkos_o = clki_i delayed by INS ns plus
	            phase_pos / SPT of the measured period (SPT 336, as the IP,
	            DIVOS_ACTUAL_STR 41). Each PHASESTEP falling edge with
	            PHASESEL = 001 moves phase_pos by one, 2.5 ns (4 VCO cycles)
	            later, PHASEDIR 0 = later (TN-02095 §14.5.9). Pulses
	            shorter than 2.5 ns low or high count in n_step_viol.
	            lock_o: high once clki_i has run LOCK_NS without a gap,
	            low LOSS_NS after its last edge (as cdr_tb part 6).
	            Phase across a lock loss: KEEP_PHASE = 1 keeps phase_pos,
	            0 returns it to 0. Which one the real PLL does is not known
	            (lvds-tunnel.md §9.6 T5.1 / log 2026-11-20); the tb runs
	            both. unlock (hierarchical, from a tb) forces lock low.
	  ECLKSYNC  STOP acts on a falling edge of ECLKIN: ECLK stops low,
	            glitch-free, and restarts with a whole high phase.
	  ECLKDIV   divide by 4; DIVRST holds DIVOUT low, released it counts
	            from the next ECLK edge (a new word boundary).
	  IDDRX4    D sampled on both ECLK edges; at each SCLK rising edge
	            Q <= the last 8 samples, first received in Q0 (O2b(a)),
	            100 ps after the edge. RST: Q = 0. A data transition less
	            than WIN_PS before a sampling edge makes that sample random
	            (a crude setup/hold model); WIN_PS is set by the tb.
	  FD1P3DX   console_por's flop (as console_tb's model). GSR stand-in: Q = 0
	            at 1 ps, so the time-0 reset events are not missed.
*********************************************************************************/

`timescale 1ns/1ps

module pll0 #(
	parameter real INS = 2.5,		// insertion delay, ns
	parameter integer SPT = 336,
	parameter real LOCK_NS = 20000.0,
	parameter real LOSS_NS = 1000.0
	)(
	input  wire       clki_i,
	input  wire       phasedir_i,
	input  wire       phasestep_i,
	input  wire       phaseloadreg_i,
	input  wire [2:0] phasesel_i,
	output wire       clkop_o,
	output reg        clkos_o,
	output reg        lock_o
);
	reg unlock = 1'b0;			// tb: force lock low
	integer KEEP_PHASE = 1;		// tb: set at time 0
	integer phase_pos = 0;
	integer n_steps = 0, n_step_viol = 0;
	realtime t_rise = 0.0, period = 26.3505, t_edge = -1.0e9, t_run = 0.0;
	realtime ps_fall = -1.0e9, ps_rise = -1.0e9;

	assign clkop_o = clki_i;
	initial begin clkos_o = 1'b0; lock_o = 1'b0; end

	always @(posedge clki_i) begin
		if (t_rise > 0.0 && $realtime - t_rise < 2.0 * period + 1.0)
			period = $realtime - t_rise;
		t_rise = $realtime;
	end

	always @(clki_i)
		clkos_o <= #(INS + phase_pos * period / SPT) clki_i;

	// lock: LOCK_NS of edges with no gap longer than LOSS_NS
	always @(clki_i) begin
		if ($realtime - t_edge > LOSS_NS)
			t_run = $realtime;			// first edge after a gap
		t_edge = $realtime;
	end
	always #50.0 begin
		if (unlock || $realtime - t_edge > LOSS_NS) begin
			if (lock_o && !KEEP_PHASE)
				phase_pos = 0;
			lock_o = 1'b0;
		end
		else if ($realtime - t_run >= LOCK_NS)
			lock_o = 1'b1;
	end

	always @(negedge phasestep_i) begin
		if ($realtime - ps_rise < 2.5) n_step_viol = n_step_viol + 1;
		ps_fall = $realtime;
		if (phasesel_i == 3'b001) begin
			#2.5;
			phase_pos = phasedir_i ? (phase_pos + SPT - 1) % SPT : (phase_pos + 1) % SPT;
			n_steps = n_steps + 1;
		end
	end
	always @(posedge phasestep_i) begin
		if ($realtime - ps_fall < 2.5 && ps_fall > 0.0) n_step_viol = n_step_viol + 1;
		ps_rise = $realtime;
	end
endmodule


module ECLKSYNC #(
	parameter STOP_EN = "ENABLE"
	)(
	input  wire ECLKIN,
	input  wire STOP,
	output wire ECLKOUT
);
	reg en = 1'b0;
	always @(negedge ECLKIN) en <= !STOP;
	assign ECLKOUT = ECLKIN & en;
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
	reg [1:0] c = 2'd0;
	initial DIVOUT = 1'b0;
	always @(posedge ECLKIN or posedge DIVRST)
		if (DIVRST) begin
			c <= 2'd0;
			DIVOUT <= 1'b0;
		end
		else begin
			c <= c + 2'd1;
			if (c[0])
				DIVOUT <= ~DIVOUT;
		end
endmodule


module IDDRX4 (
	input  wire D,
	input  wire SCLK,
	input  wire RST,
	input  wire ECLK,
	input  wire ALIGNWD,
	output reg  Q0, Q1, Q2, Q3, Q4, Q5, Q6, Q7
);
	real WIN_PS = 0.0;			// tb
	realtime t_d = -1.0e9;
	reg [7:0] sh = 8'd0;
	reg smp;
	initial {Q7, Q6, Q5, Q4, Q3, Q2, Q1, Q0} = 8'd0;

	always @(D) t_d = $realtime;

	always @(ECLK) if (!RST && ECLK !== 1'bx) begin
		if ($realtime - t_d < WIN_PS / 1000.0) smp = $urandom % 2;
		else smp = D;
		sh = {smp, sh[7:1]};		// first received ends in sh[0]
	end

	always @(posedge SCLK or posedge RST)
		if (RST)
			{Q7, Q6, Q5, Q4, Q3, Q2, Q1, Q0} <= 8'd0;
		else
			{Q7, Q6, Q5, Q4, Q3, Q2, Q1, Q0} <= #0.1 sh;
endmodule


module FD1P3DX #(parameter GSR = "ENABLED") (input wire D, input wire SP,
		input wire CK, input wire CD, output reg Q);
	initial #0.001 Q = 1'b0;
	always @(posedge CK or posedge CD)
		if (CD) Q <= 1'b0;
		else if (SP) Q <= D;
endmodule
