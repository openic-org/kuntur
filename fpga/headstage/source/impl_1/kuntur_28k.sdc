# ---------------------------------------------------------------------------
# Kuntur timing constraints, 28 kSPS: clk 42.504 MHz (pll_28k).
# Implementation impl_28k (variant 1).
#
# One of two per-rate copies (task #6, log/2026-11-04.md): kuntur_25k.sdc
# is this file with the header and RATE block changed, nothing else. Edit both
# together; `diff kuntur_25k.sdc kuntur_28k.sdc` must show only those.
# Background and history of every constraint below: vega
# docs/interfaces/fpga-timing-constraints.md.
#
# Board/cable delays are placeholders (uHDMI link, kuntur144-omnetics),
# not extracted from trace lengths or measured. A wrong number here still
# reports a clean run.
# ---------------------------------------------------------------------------

# ---- RATE block (the only part that differs from kuntur_25k.sdc) ----------
# clk: CLKOP. pll_28k/constraints/pll_28k.ldc: CLKOP_FREQ_ACTUAL 42.504012,
# period 1000/42.504012 = 23.5271907979 ns. Fractional-N (FRAC_N_EN 1), so
# CLKOP/clkin is not an integer ratio a create_generated_clock could
# express: constrained directly on the net, as Lattice's IP templates do.
create_clock -name {clk} -period 23.5271907979 [get_nets clk]
# clk_tx: CLKOS of the same PLL, same frequency, static phase 0°
# (CLKOS_PHASE_ACTUAL 0.000000), dynamic phase enabled. #6a
# (spi1-mosi-eye-scan.md §3.1) launches spi1_mosi from it, through one
# register, scan0/mosi_tx, when mosi_sel is 0-3.
# The clk -> clk_tx bound on that register (O2) is in impl_1.pdc: it
# names a post-synthesis pin, which this file cannot.
create_clock -name {clk_tx} -period 23.5271907979 [get_nets clk_tx]
# ---- end of RATE block -----------------------------------------------------

# clkin: 32 MHz from the STM32WB0 MCO3/PB14.
create_clock -name {clkin} -period 31.25 [get_ports clkin]
set_clock_groups -asynchronous -group [get_clocks clkin] -group [get_clocks {clk clk_tx spi1_sck}]
set_clock_uncertainty -setup -hold 0.125 [get_clocks {clk clk_tx}]

# spi1_sck: decoded from spi_master_controller's state (one clk high in
# sckNb, one clk low in sckNd) for as long as a burst shifts. -divide_by 2
# holds during the shift states, the only time it needs to.
create_generated_clock -name {spi1_sck} -source [get_nets clk] -divide_by 2 [get_ports spi1_sck]

# ---------------------------------------------------------------------------
# RHD2164 MISO input delays.
#
# The capture flops (sr_s2p) run on clk and sample MISO on the next clk
# edge after each sck edge, so T_pcb_sck + T_co_RHD + T_pcb_miso must fit
# in one clk period, on both sck edges: MISO is DDR (A off SCLK rising,
# B off falling, RHD2164 datasheet p.9). T_co_RHD = tMISO 12 ns max
# (RHD2000 datasheet p.15; no min published, 0 used).
# ---------------------------------------------------------------------------
# MISO-A: launched off sck rising.
set_input_delay -clock [get_clocks spi1_sck] -max 15 [get_ports {spi1_miso0 spi1_miso1}]
set_input_delay -clock [get_clocks spi1_sck] -min 3 [get_ports {spi1_miso0 spi1_miso1}]
# MISO-B: launched off sck falling.
set_input_delay -clock [get_clocks spi1_sck] -clock_fall -max -add_delay 15 [get_ports {spi1_miso0 spi1_miso1}]
set_input_delay -clock [get_clocks spi1_sck] -clock_fall -min -add_delay 3 [get_ports {spi1_miso0 spi1_miso1}]

# ---------------------------------------------------------------------------
# RHD2164 MOSI output delay.
#
# spi1_mosi is sr_p2s's output on clk: it shifts on the clk edge that ends
# sckNb, the edge where sck falls. The RHD samples it on sck's next rising
# edge and needs tMOSI = 10 ns setup (RHD2000 datasheet p.15). For
# set_output_delay the reference edge is the one the external device
# CAPTURES on, so spi1_sck rising (no -clock_fall).
# ---------------------------------------------------------------------------
set_output_delay -clock [get_clocks spi1_sck] -max 11.5 [get_ports spi1_mosi]
set_output_delay -clock [get_clocks spi1_sck] -min 1.5 [get_ports spi1_mosi]
# Hold: clk launches, spi1_sck (clk/2) captures. Setup finds the right
# edge; the default hold check uses the spi1_sck edge one period earlier,
# which has no physical meaning here. -hold 1 moves it onto the setup
# edge. ncycles is positional and every object goes under -from/-to
# (FPGA-AN-02059 §3.3.4).
set_multicycle_path -hold -end -from [get_clocks clk] -to [get_ports spi1_mosi] 1

# ---------------------------------------------------------------------------
# RHD2164 CSB output delay.
#
# Falling: csb goes low entering op0; sck's first rising edge is 2 clk
# later. tCS1 "CS low to SCLK high setup" 20 ns min, -max only.
# Rising: csb goes high entering csbend0, the edge of sck's last fall.
# tCS2 "SCLK low to CS high" 20 ns min, against spi1_sck falling, -min
# only. tCSOFF (154 ns CS high) is protocol pacing, not an edge
# relationship STA can check.
# ---------------------------------------------------------------------------
set_output_delay -clock [get_clocks spi1_sck] -max 21.5 [get_ports spi1_csb]
set_output_delay -clock [get_clocks spi1_sck] -clock_fall -min -add_delay 21.5 [get_ports spi1_csb]

# ---------------------------------------------------------------------------
# Asynchronous inputs (fpga-timing-constraints.md §10).
# ---------------------------------------------------------------------------
# spi0: no sck-domain logic; edge_detector's 2-flop synchroniser is the
# safety mechanism (§10.1).
set_false_path -from [get_ports {spi0_csb spi0_sck spi0_mosi}]
set_false_path -to [get_ports spi0_miso]
# rstb: global async reset, no release synchroniser. Accepted risk (§10.2).
set_false_path -from [get_ports rstb]
# clk -> wclk is cut in impl_1.pdc, not here: wclk only exists after
# synthesis (Radiant derives it from ECLKDIV).

# ---------------------------------------------------------------------------
# Macros.
# ---------------------------------------------------------------------------
ldc_create_macro -name spi1_rhd2164x2_macro [get_cells spi1_rhd2164x2]
ldc_create_macro -name controller0_macro [get_cells controller0]
ldc_create_macro -name ch_sel0_macro [get_cells ch_sel0]
ldc_create_macro -name controller1_macro [get_cells controller1]
ldc_create_macro -name fifo0_macro [get_cells fifo0]
ldc_create_macro -name spi0_macro [get_cells spi0]
ldc_create_macro -name muxreg0_spi0_macro [get_cells muxreg0_spi0]
ldc_create_macro -name regbank0_macro [get_cells regbank0]
ldc_create_macro -name test_pattern_gen0_macro [get_cells test_pattern_gen0]
ldc_create_macro -name fifo_din_mux0_macro [get_cells fifo_din_mux0]
