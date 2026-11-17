#create_clock -name {clkin} -period 10 [get_ports clkin]

# ---------------------------------------------------------------------------
# Clocks (PLAN.md Phase B.1, "FPGA timing constraints" — first real STA on
# this design). Before this the only active line here was a stray
# `create_clock -name {clkin} -period 10 [get_ports clkin]`, i.e. clkin
# constrained at 100 MHz — clkin is actually 32 MHz (31.25 ns). That wrong
# clock also fed the `clk` uncertainty line living in impl_1.pdc, but there
# was still no create_clock for `clk` anywhere, so every path in the design —
# including the RHD2164 MISO/MOSI SPI timing — closed on whatever the router
# happened to do.
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# TIMING_MARGIN_PCT — single knob to over-constrain clkin and clk together
# for extra STA guardband, without hand-editing derived numbers.
#
# Found empirically (Manuel, pre-2026-08-24): tightening clkin's constraint
# alone — with no create_clock for clk and no explicit spi1_mosi constraint
# anywhere — made a tMOSI-margin problem go away. clkin fans out to the PLL
# primitive and other blocks, so the only way that can work is if Radiant derives
# clk's STA timing from clkin's constraint using the PLL's real configured
# ratio; tightening the input proportionally tightens clk, which tightens
# every clk-domain register-to-register path system-wide — including
# whatever feeds spi1_mosi's output register — even though nothing named
# spi1_mosi. Below, clk is created explicitly (see the fractional-N note)
# rather than left to auto-derivation, but is scaled by the same
# TIMING_MARGIN_PCT so the two move together and that mechanism is
# reproduced deliberately instead of by accident.
#
# IMPORTANT — this only buys margin on the FPGA-internal portion of the
# round trip (sck decode, capture-flop setup, MOSI clock-to-out). It does
# NOT touch the external, physical part of the budget — T_co_RHD (12 ns),
# tMOSI (10 ns) or the board/cable delays below — those are fixed by the
# chip and the PCB, not by synthesis effort. And leaving this at 0 for the
# first real run matters: the point of this whole constraint set (PLAN.md
# Phase B.1) was to see the HONEST violation at the real 32/44.55 MHz
# operating point and diagnose the structural cause, not to synthesize our
# way past it. Use TIMING_MARGIN_PCT > 0 to guardband a chosen structural
# fix afterward (slower sck, pipelined rx_en, DELAYA, phase-shifted PLL —
# see PLAN.md), not as a substitute for picking one.
# pll0/pll0.cfg gui_clk_op_freq / CLKOP_FREQ_ACTUAL
# clkin: 32 MHz from the STM32WB0 MCO3/PB14.
create_clock -name {clkin} -period 31.25 [get_ports clkin]
# clk: pll0's CLKOP. 2026-08-31: PLL regenerated for the 2026-08-27 retune
# (exact 30,000 SPS) plus a new CLKOS output (see clk90 below) for the
# SCK/MOSI deskew fix — pll0/constraints/pll0.ldc: CLKOP_FREQ_ACTUAL
# 45.539955. The PLL is fractional-N (FRAC_N_EN=1) and 45.54/32 is not an
# integer ratio, so a create_generated_clock -multiply_by/-divide_by off
# clkin can't express the relationship exactly — constraining CLKOP
# directly, as Lattice's own IP constraint templates do for fractional-N
# PLLs.
create_clock -name {clk} -period 21.9587393093 [get_nets clk]
set_clock_groups -asynchronous -group [get_clocks clkin] -group [get_clocks {clk spi1_sck}]
set_clock_uncertainty -setup -hold 0.125 [get_clocks clk]
# clk90: pll0's CLKOS, same PLL, same 45.539955 MHz, phase-shifted +90°
# (pll0.ldc: CLKOS_PHASE_ACTUAL 90.000000) — added 2026-08-31 for the
# SCK/MOSI board-skew deskew fix (PLAN.md B.5, "kuntur chip0/SCK-MOSI
# board-level regression"). Declared as its own create_clock on the net
# (same pattern as clk itself), not as a generated clock off clk, because
# a generated clock's phase is inferred structurally and this is a second,
# independent PLL output — the -waveform below is what actually encodes
# the 90° offset for STA. Positive phase on this Lattice PLL IP means
# clk90's edges trail clk's (confirmed against the regenerated IP config,
# 2026-08-31), so its first rising edge lands one quarter-period after
# clk's: 21.9587393093 / 4 = 5.4896848273 ns, falling at + one half period
# = 16.4690544820 ns. Used to re-launch spi1_mosi (kuntur_fpga.v's new
# mosi_reg) deliberately later than spi1_sck's clk-launched sck_reg, to
# compensate for the RHD2164 pair's board-level SCK/MOSI trace-length
# asymmetry — see the plan entry for the full derivation. Real effect
# on spi1_mosi's existing set_output_delay budget (-max 11.5 ns) must be
# read off the .twr after this lands, not assumed from the phase alone.
#create_clock -name {clk90} -period 21.9587393093 \
#    -waveform {5.4896848273 16.4690544820} [get_nets clk90]
#set_clock_uncertainty -setup -hold 0.125 [get_clocks clk90]
# spi1_sck: registered off clk as of 2026-08-31 (sck_reg, mirroring
# csb_reg's existing pattern) — was combinationally decoded from
# spi_master_controller's current_state before. Still one clk cycle high
# (sckNb), one clk cycle low (sckNd), for as long as a burst is shifting;
# -divide_by 2 is an assertion that holds during the shift states, which
# is the only time it needs to, same as before the register was added.
create_generated_clock -name {spi1_sck} -source [get_nets clk] -divide_by 2 [get_ports spi1_sck]
# ---------------------------------------------------------------------------
# Board/cable delay placeholders (uHDMI link, kuntur144-omnetics). NOT yet
# extracted from real trace lengths or a bench measurement — replace before
# trusting reported slack. A wrong constraint here still reports a clean run.
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# RHD2164 MISO input delays.
#
# Round trip that must fit inside one clk period (22.447 ns): the capture
# flops (sr_s2p in spi_controllers.v) are clocked by clk, not by sck, and
# the FSM samples MISO on the very next clk edge after each sck edge — so
# T_pcb_sck(FPGA->RHD) + T_co_RHD + T_pcb_miso(RHD->FPGA) must clear setup
# with margin inside that one clk period, on BOTH sck edges: the RHD2164's
# MISO is DDR (A data launched off the SCLK rising edge, B off the falling
# edge, RHD2164 datasheet p.9), so a rising-edge-only constraint would leave
# the B half of every sample unconstrained.
#
# T_co_RHD = 12 ns max (RHD2000 series datasheet p.15, SPI BUS TIMING
# SPECIFICATIONS, tMISO: "SCLK or CS Falling Edge to MISO Data Valid", MAX
# column; no MIN is published, so 0 ns is used below rather than assumed).
# ---------------------------------------------------------------------------
# MISO-A: launched off sck's rising edge -> constrained against spi1_sck's
# rising edge (the default).
set_input_delay -clock [get_clocks spi1_sck] -max 15 [get_ports {spi1_miso0 spi1_miso1}]
set_input_delay -clock [get_clocks spi1_sck] -min 3 [get_ports {spi1_miso0 spi1_miso1}]
# MISO-B: launched off sck's falling edge -> constrained against spi1_sck's
# falling edge.
set_input_delay -clock [get_clocks spi1_sck] -clock_fall -max -add_delay 15 [get_ports {spi1_miso0 spi1_miso1}]
set_input_delay -clock [get_clocks spi1_sck] -clock_fall -min -add_delay 3 [get_ports {spi1_miso0 spi1_miso1}]
# ---------------------------------------------------------------------------
# RHD2164 MOSI output delay.
#
# STALE as of 2026-08-31, kept for the history — see clk90's comment above.
# Originally: spi_master_controller asserts tx_en in state sckNb, the SAME
# state that drives sck=1, so sr_p2s's areg shifted on the clk edge that
# ended sckNb — exactly the clk edge where sck's combinational output fell
# from 1 to 0, making the new MOSI bit valid coincident with sck's FALLING
# edge (matching the datasheet's timing diagram, "Data should change on the
# falling edge of SCLK", p.15). That coincidence is now deliberately broken:
# spi1_mosi launches off mosi_reg on clk90 instead, ~5.49 ns after sck_reg's
# own clk-launched transition, specifically to deskew against the RHD2164
# pair's board-level SCK/MOSI trace-length asymmetry. The datasheet
# relationship below (MOSI valid before the RHD's next SCK rising-edge
# sample point) is what's actually being protected — the *launch* timing
# relative to sck's falling edge was never the requirement, just an
# incidental property of the old combinational implementation.
#
# The RHD then samples MOSI on sck's NEXT rising edge, one spi1_sck half
# period later, and needs it stable tMOSI = 10 ns before that edge (RHD2000
# series datasheet p.15, SPI BUS TIMING SPECIFICATIONS, tMOSI: "MOSI Data
# Valid to SCLK High Setup", MIN column — verified directly against the
# datasheet table; the 10.4 ns figure from memory doesn't match what's
# printed there, it's 10 ns flat, no MAX given).
#
# NOTE on -clock_fall (or the deliberate absence of it here): for
# set_output_delay the named clock/edge means "the edge the EXTERNAL device
# uses to CAPTURE" — the opposite convention from set_input_delay, where it
# means "the edge the external device uses to LAUNCH". The RHD captures
# MOSI on sck's RISING edge (datasheet: "the RHD2000 samples MOSI on the
# rising edge of SCLK"), so this is left at spi1_sck's default (rising)
# reference — no -clock_fall. An earlier version of this file used
# -clock_fall here, which told the tool the RHD captures on the FALLING
# edge instead; Radiant's own report confirmed the mistake ("Destination
# Clock: spi1_sck (F)") and it inflated the reported slack to ~12 ns by
# checking against a deadline a full spi1_sck period too late.
# ---------------------------------------------------------------------------
set_output_delay -clock [get_clocks spi1_sck] -max 11.5 [get_ports spi1_mosi]
set_output_delay -clock [get_clocks spi1_sck] -min 1.5 [get_ports spi1_mosi]
# spi1_mosi hold: clk (fast) launches, spi1_sck (2x slower, divide_by 2)
# captures. Setup already finds the correct edge (one full spi1_sck period
# after launch — matches the intended relationship, no exception needed and
# none given). Hold's SDC default is different: absent a multicycle
# exception, it checks against the destination-clock edge immediately
# preceding the setup edge, which for a 2:1 related clock is a FULL
# spi1_sck period too early — a rising edge that has no physical meaning
# here, since MOSI only ever changes in lockstep with sck's own toggling
# and could never actually be sampled there. Confirmed numerically
# (2026-08-24 run): the failing hold path's destination-clock arrival was
# 4.178 ns; 4.178 + one spi1_sck period (44.893 ns) ≈ 49.1 ns, matching
# the setup path's reference-edge ballpark — i.e. the default hold check
# was referencing the edge exactly one period before the real one.
# -hold 1 shifts the hold reference forward to align with the same edge
# setup already uses correctly.
#
# Syntax per Lattice's own app note (FPGA-AN-02059, "Radiant Timing
# Constraints Methodology", section 3.3.4): ncycles is a bare positional
# argument, NOT a value fused onto -hold — the doc's own two worked
# examples disagree on where it sits (`-from [...] 2` in one, `2 -from
# [...] -to [...]` in the other), so Radiant's parser evidently accepts it
# anywhere, but there is no bare trailing object list the way
# set_input_delay/set_output_delay have — every object must sit under
# -from/-to/-through. My first attempt left [get_ports spi1_mosi] bare at
# the end with no flag, which is what Radiant rejected. Fixed: spi1_mosi
# goes under -to (Table 3.14: -to accepts clocks, ports, pins, or cells),
# replacing the [get_clocks spi1_sck] -to target.
set_multicycle_path -hold -end -from [get_clocks clk] -to [get_ports spi1_mosi] 1
# ---------------------------------------------------------------------------
# RHD2164 CSB (chip select) output delay.
#
# csb is combinational in spi_master_controller (spi_controllers.v): it
# defaults high, and every active state (op0 through sck15d) explicitly
# drives it low; csbend0..csbend7 do NOT re-drive it, so it reverts to its
# default (high) starting at csbend0. Two edges to constrain, each against
# a DIFFERENT sck reference edge, so they're modeled as two independent,
# single-sided checks rather than a symmetric max/min pair:
#
# - FALLING edge: csb goes low the same clk edge the FSM leaves idle
#   (entering op0); sck's FIRST rising edge follows exactly 2 clk cycles
#   later (op0 -> op1 -> sck0b). This is a setup-type requirement — csb
#   must be low before that edge — matching the datasheet's tCS1: "CS Low
#   to SCLK High Setup", 20 ns MIN (RHD2000 series datasheet p.15). No
#   published MAX and no meaningful hold-type pairing here (csb falling
#   earlier is harmless), so -max only.
#
# - RISING edge: csb goes high the same clk edge sck's LAST falling edge
#   happens (leaving sck15d, entering csbend0). This is a hold-type
#   requirement — csb must stay low a minimum time AFTER that edge —
#   matching tCS2: "SCLK Low to CS High Setup", 20 ns MIN (same table).
#   Referenced against spi1_sck's FALLING edge (the true last-sck-edge
#   reference for this check, unlike the MOSI case above), -min only.
#
# NOT modeled: tCSOFF ("CS High Duration", 154 ns MIN) — this bounds the
# minimum time between the end of one transaction and the start of the
# next, which isn't a two-signal edge relationship set_output_delay can
# express. It's a protocol/firmware-pacing requirement on how soon `start`
# may re-assert after csb rises, not something Radiant STA checks here.
#
# Budget for both: pcb_csb_ns is NOT netted against pcb_sck_ns even though
# both are the same 1.5 ns placeholder today (which would cancel to zero) —
# same conservative choice as MOSI above: don't assume trace-length
# matching between csb's and sck's uHDMI pairs that hasn't been verified.
# ---------------------------------------------------------------------------
set_output_delay -clock [get_clocks spi1_sck] -max 21.5 [get_ports spi1_csb]
set_output_delay -clock [get_clocks spi1_sck] -clock_fall -min -add_delay 21.5 [get_ports spi1_csb]
ldc_create_macro -name spi1_rhd2164x2_macro [get_cells spi1_rhd2164x2]
ldc_create_macro -name controller0_macro [get_cells controller0]
ldc_create_macro -name ch_sel0_macro [get_cells ch_sel0]
ldc_create_macro -name controller1_macro [get_cells controller1]
ldc_create_macro -name fifo0_macro [get_cells fifo0]
ldc_create_macro -name spi0_macro [get_cells spi0]
# spi0 — asynchronous exception (no sck-domain logic exists anywhere;
# edge_detector's 2-flop synchronizer + one-cycle-late capture is the
# real safety mechanism, not a clock relationship). See
# fpga-timing-constraints.md §10.1.
set_false_path -from [get_ports {spi0_csb spi0_sck spi0_mosi}]
set_false_path -to [get_ports spi0_miso]
# rstb — global async reset, no release synchronizer exists (~2747
# endpoints). Documented risk acceptance, not a fix — see §10.2.
set_false_path -from [get_ports rstb]
ldc_create_macro -name muxreg0_spi0_macro [get_cells muxreg0_spi0]
ldc_create_macro -name regbank0_macro [get_cells regbank0]
ldc_create_macro -name test_pattern_gen0_macro [get_cells test_pattern_gen0]
ldc_create_macro -name fifo_din_mux0_macro [get_cells fifo_din_mux0]
