onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /kuntur_tb/dut/rstb
add wave -noupdate /kuntur_tb/dut/clk
add wave -noupdate -divider RHD2164
add wave -noupdate /kuntur_tb/dut/spi1_rhd2164x2/start
add wave -noupdate /kuntur_tb/dut/spi1_rhd2164x2/done
add wave -noupdate /kuntur_tb/dut/spi1_miso1
add wave -noupdate /kuntur_tb/dut/spi1_miso0
add wave -noupdate /kuntur_tb/dut/spi1_csb
add wave -noupdate /kuntur_tb/dut/spi1_sck
add wave -noupdate /kuntur_tb/dut/spi1_mosi
add wave -noupdate -radix unsigned /kuntur_tb/dut/rhd_addr_cfg
add wave -noupdate -radix unsigned /kuntur_tb/dut/rhd_addr_sampling
add wave -noupdate /kuntur_tb/dut/rhd_dtx
add wave -noupdate -radix unsigned /kuntur_tb/dut/regbank_addr1
add wave -noupdate /kuntur_tb/dut/spi1_rhd2164x2/data_tx
add wave -noupdate /kuntur_tb/dut/spi1_data_rx_a0
add wave -noupdate /kuntur_tb/dut/spi1_data_rx_b0
add wave -noupdate /kuntur_tb/dut/spi1_data_rx_a1
add wave -noupdate /kuntur_tb/dut/spi1_data_rx_b1
add wave -noupdate -divider CH_SEL
add wave -noupdate -radix unsigned /kuntur_tb/dut/ch_sel0/ch_cnt
add wave -noupdate /kuntur_tb/dut/ch_sel0/data_a0
add wave -noupdate /kuntur_tb/dut/ch_sel0/data_b0
add wave -noupdate /kuntur_tb/dut/ch_sel0/data_a1
add wave -noupdate /kuntur_tb/dut/ch_sel0/data_b1
add wave -noupdate /kuntur_tb/dut/ch_sel0/ch_is_a
add wave -noupdate /kuntur_tb/dut/ch_sel0/ch_is_b
add wave -noupdate /kuntur_tb/dut/ch_sel0/ch_is_a_redge
add wave -noupdate /kuntur_tb/dut/ch_sel0/ch_is_b_redge
add wave -noupdate /kuntur_tb/dut/ch_sel0/data0
add wave -noupdate /kuntur_tb/dut/ch_sel0/data1
add wave -noupdate /kuntur_tb/dut/ch_sel0/data0_synced
add wave -noupdate /kuntur_tb/dut/ch_sel0/data1_synced
add wave -noupdate /kuntur_tb/dut/ch_sel0/ch_is_0
add wave -noupdate /kuntur_tb/dut/ch_sel0/ch_is_0_redge
add wave -noupdate /kuntur_tb/dut/ch_sel0/ch_is_16_redge
add wave -noupdate /kuntur_tb/dut/ch_sel0/dout_en_0
add wave -noupdate /kuntur_tb/dut/ch_sel0/dout_en_16
add wave -noupdate /kuntur_tb/dut/ch_sel0/dout
add wave -noupdate /kuntur_tb/dut/ch_sel0/ch_a
add wave -noupdate /kuntur_tb/dut/ch_sel0/ch_b
add wave -noupdate /kuntur_tb/dut/ch_sel0/fifo_wen
add wave -noupdate -divider FIFO
add wave -noupdate /kuntur_tb/dut/fifo0/wen
add wave -noupdate /kuntur_tb/dut/fifo0/ren
add wave -noupdate /kuntur_tb/dut/fifo0/DIN
add wave -noupdate /kuntur_tb/dut/fifo0/DOUT
add wave -noupdate /kuntur_tb/dut/fifo0/empty
add wave -noupdate /kuntur_tb/dut/fifo0/full
add wave -noupdate /kuntur_tb/dut/fifo0/fifo_ram
add wave -noupdate /kuntur_tb/dut/fifo0/in_ptr
add wave -noupdate /kuntur_tb/dut/fifo0/out_ptr
add wave -noupdate -divider MCU
add wave -noupdate /kuntur_tb/dut/spi0/csb
add wave -noupdate /kuntur_tb/dut/spi0/sck
add wave -noupdate /kuntur_tb/dut/spi0/mosi
add wave -noupdate /kuntur_tb/dut/spi0/miso
add wave -noupdate /kuntur_tb/dut/spi0/data_tx
add wave -noupdate /kuntur_tb/dut/spi0/data_rx
add wave -noupdate /kuntur_tb/mcu_spi/rxsr/en
add wave -noupdate /kuntur_tb/mcu_spi/rxsr/load
add wave -noupdate /kuntur_tb/mcu_spi/rxsr/a
add wave -noupdate /kuntur_tb/mcu_spi/rxsr/data
add wave -noupdate /kuntur_tb/mcu_spi/rxsr/areg
add wave -noupdate /kuntur_tb/dut/spi0/done
add wave -noupdate /kuntur_tb/dut/fifo_ren
add wave -noupdate -radix unsigned /kuntur_tb/dut/controller1/current_state
add wave -noupdate -expand /kuntur_tb/dut/muxreg0/mcu_dtx_sel
add wave -noupdate /kuntur_tb/dut/muxreg0/mcu_dtx_en
add wave -noupdate /kuntur_tb/dut/muxreg0/din
add wave -noupdate /kuntur_tb/dut/muxreg0/dout
add wave -noupdate /kuntur_tb/dut/muxreg0/dtx
add wave -noupdate /kuntur_tb/mcu_start
add wave -noupdate /kuntur_tb/mcu_dtx
add wave -noupdate -divider Serial-LVDS
add wave -noupdate /kuntur_tb/dut/serial_lvds_tx
add wave -noupdate -divider SRAM
add wave -noupdate /kuntur_tb/dut/regbank/regbank_port_en
add wave -noupdate /kuntur_tb/dut/regbank/regbank_port_sel
add wave -noupdate /kuntur_tb/dut/regbank/addr_reg
add wave -noupdate /kuntur_tb/dut/regbank/staged_h
add wave -noupdate /kuntur_tb/dut/regbank/din0
add wave -noupdate /kuntur_tb/dut/ch_a
add wave -noupdate /kuntur_tb/dut/ch_b
add wave -noupdate /kuntur_tb/dut/regbank/rstb
add wave -noupdate /kuntur_tb/dut/regbank/clk
add wave -noupdate /kuntur_tb/dut/regbank/wen
add wave -noupdate /kuntur_tb/dut/regbank/addr0
add wave -noupdate /kuntur_tb/dut/regbank/din0
add wave -noupdate /kuntur_tb/dut/regbank/dout0
add wave -noupdate -radix unsigned /kuntur_tb/dut/regbank/addr1
add wave -noupdate /kuntur_tb/dut/regbank/dout1
add wave -noupdate /kuntur_tb/dut/regbank/rhd2164_sampling_cmd0
add wave -noupdate /kuntur_tb/dut/regbank/rhd2164_sampling_cmd1
add wave -noupdate /kuntur_tb/dut/regbank/rhd2164_sampling_cmd2
add wave -noupdate /kuntur_tb/dut/regbank/rhd2164_sampling_cmd3
add wave -noupdate /kuntur_tb/dut/regbank/ram
add wave -noupdate /kuntur_tb/dut/controller1/rstb
add wave -noupdate /kuntur_tb/dut/controller1/clk
add wave -noupdate /kuntur_tb/dut/controller1/fifo_empty
add wave -noupdate /kuntur_tb/dut/controller1/fifo_ren
add wave -noupdate /kuntur_tb/dut/controller1/mcu_done
add wave -noupdate /kuntur_tb/dut/controller1/opcode
add wave -noupdate /kuntur_tb/dut/spi0/data_rx
add wave -noupdate /kuntur_tb/dut/controller1/regbank_wen
add wave -noupdate /kuntur_tb/dut/controller1/current_state
add wave -noupdate /kuntur_tb/dut/controller1/next_state
add wave -noupdate /kuntur_tb/dut/controller1/redge_mcu_done
add wave -noupdate /kuntur_tb/dut/controller1/mcu_dtx_sel
add wave -noupdate /kuntur_tb/dut/controller1/opcode_is_pop
add wave -noupdate /kuntur_tb/dut/controller1/opcode_is_write
add wave -noupdate /kuntur_tb/dut/controller1/opcode_is_read
add wave -noupdate /kuntur_tb/dut/controller1/opcode_is_nop
add wave -noupdate /kuntur_tb/dut/controller1/mcu_done_reg
add wave -noupdate -divider Controller0
add wave -noupdate /kuntur_tb/dut/rhd_spi1_start
add wave -noupdate /kuntur_tb/dut/rhd_spi1_done
add wave -noupdate /kuntur_tb/dut/controller0/cnt0_en
add wave -noupdate /kuntur_tb/dut/controller0/cnt1_en
add wave -noupdate -radix unsigned /kuntur_tb/dut/rhd_dtx_sel
add wave -noupdate -radix unsigned /kuntur_tb/dut/rhd_addr_cfg
add wave -noupdate -radix unsigned /kuntur_tb/dut/rhd_addr_sampling
add wave -noupdate -radix unsigned /kuntur_tb/dut/regbank_addr1
add wave -noupdate -radix unsigned /kuntur_tb/dut/ch_cnt
add wave -noupdate -divider {Inner Blocks}
add wave -noupdate /kuntur_tb/dut/cmd_is_00
add wave -noupdate /kuntur_tb/dut/data_rx
add wave -noupdate /kuntur_tb/dut/spi0_done
add wave -noupdate /kuntur_tb/dut/rx_flag
add wave -noupdate /kuntur_tb/dut/rhd_dtx
add wave -noupdate /kuntur_tb/dut/rhd_dtx0
add wave -noupdate /kuntur_tb/dut/rhd_dtx1
add wave -noupdate /kuntur_tb/dut/spi0_drx
add wave -noupdate /kuntur_tb/dut/clkdiv2
add wave -noupdate /kuntur_tb/dut/fifo_wen
add wave -noupdate /kuntur_tb/dut/fifo_ren
add wave -noupdate /kuntur_tb/dut/fifo_empty
add wave -noupdate /kuntur_tb/dut/fifo_full
add wave -noupdate /kuntur_tb/dut/fifo_din
add wave -noupdate /kuntur_tb/dut/fifo_dout
add wave -noupdate /kuntur_tb/dut/rhd2164_sampling_cmd0
add wave -noupdate /kuntur_tb/dut/rhd2164_sampling_cmd1
add wave -noupdate /kuntur_tb/dut/rhd2164_sampling_cmd2
add wave -noupdate /kuntur_tb/dut/regbank_wen
add wave -noupdate /kuntur_tb/dut/regbank_addr0
add wave -noupdate /kuntur_tb/dut/regbank_addr1
add wave -noupdate /kuntur_tb/dut/regbank_din0
add wave -noupdate /kuntur_tb/dut/regbank_dout0
add wave -noupdate /kuntur_tb/dut/regbank_dout1
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {10006555 ns} 0} {{Cursor 2} {15770 ns} 0}
quietly wave cursor active 2
configure wave -namecolwidth 266
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {9354 ns} {75606 ns}
