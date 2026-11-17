create_clock -name {clkin} -period 83.3333333333333 [get_ports clkin]
create_clock -name {spi0_sck} -period 41.6666666666667 [get_ports spi0_sck]
create_clock -name {kuntur_clk} -period 26.3504868239 [get_ports kuntur_clk]
# derived clocks 'kuntur_clk_os' and 'wclk' inherit the group
set_clock_groups -asynchronous -group [get_clocks clkin] -group [get_clocks {kuntur_clk}] -group [get_clocks spi0_sck]