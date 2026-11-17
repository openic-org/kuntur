create_clock -name {clkin} -period 83.3333333333333 [get_ports clkin]
create_clock -name {spi0_sck} -period 41.6666666666667 [get_ports spi0_sck]
create_clock -name {kuntur_clk} -period 21.9587176108915 [get_ports kuntur_clk]
set_clock_groups -asynchronous -group [get_clocks clkin] -group [get_clocks {kuntur_clk kuntur_clk_os wclk}] -group [get_clocks spi0_sck]