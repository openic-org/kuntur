onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -divider tx_gen
add wave -noupdate /cdr_tb/tx_gen/rstb
add wave -noupdate /cdr_tb/tx_gen/clk
add wave -noupdate /cdr_tb/tx_gen/dout
add wave -noupdate /cdr_tb/tx_gen/cnt0
add wave -noupdate -divider ref0
add wave -noupdate /cdr_tb/tx_gen/cnt1
add wave -noupdate /cdr_tb/ref0/clk
add wave -noupdate /cdr_tb/ref0/rst
add wave -noupdate /cdr_tb/ref0/count_en
add wave -noupdate /cdr_tb/ref0/q
add wave -noupdate /cdr_tb/ref0/locked
add wave -noupdate /cdr_tb/ref0/errors
add wave -noupdate /cdr_tb/ref0/blocks
add wave -noupdate /cdr_tb/ref0/h
add wave -noupdate /cdr_tb/ref0/blk
add wave -noupdate /cdr_tb/ref0/s
add wave -noupdate /cdr_tb/ref0/pos
add wave -noupdate /cdr_tb/ref0/ecnt
add wave -noupdate /cdr_tb/ref0/pcnt
add wave -noupdate /cdr_tb/ref0/state
add wave -noupdate /cdr_tb/ref0/clean
add wave -noupdate /cdr_tb/ref0/k
add wave -noupdate /cdr_tb/ref0/hn
add wave -noupdate /cdr_tb/ref0/ab
add wave -noupdate /cdr_tb/ref0/eb
add wave -noupdate /cdr_tb/ref0/found
add wave -noupdate -divider {New Divider}
add wave -noupdate /cdr_tb/dout
add wave -noupdate /cdr_tb/txb
add wave -noupdate /cdr_tb/tx_eclk
add wave -noupdate /cdr_tb/tx_wclk
add wave -noupdate /cdr_tb/tx_bit
add wave -noupdate /cdr_tb/rx_eclk
add wave -noupdate /cdr_tb/rx_wclk
add wave -noupdate -divider aligner
add wave -noupdate /cdr_tb/al0/rst_sync
add wave -noupdate /cdr_tb/al0/clk
add wave -noupdate /cdr_tb/al0/din
add wave -noupdate /cdr_tb/al0/bitsel
add wave -noupdate /cdr_tb/al0/bitsel_aligned
add wave -noupdate /cdr_tb/al0/valid
add wave -noupdate /cdr_tb/al0/rdy
add wave -noupdate /cdr_tb/al0/current_state
add wave -noupdate /cdr_tb/al0/next_state
add wave -noupdate /cdr_tb/al0/din_reg_0
add wave -noupdate /cdr_tb/al0/din_reg_1
add wave -noupdate /cdr_tb/al0/data_sa
add wave -noupdate /cdr_tb/al0/data_sa_reg_0
add wave -noupdate /cdr_tb/al0/data_sa_reg_1
add wave -noupdate /cdr_tb/al0/data_sa_reg_2
add wave -noupdate /cdr_tb/al0/data_sa_reg_3
add wave -noupdate /cdr_tb/al0/data_is_preamble
add wave -noupdate /cdr_tb/al0/bitsel_aligned_en
add wave -noupdate /cdr_tb/al0/cnt0
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {614824 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 196
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
WaveRestoreZoom {20539128457 ps} {20540231831 ps}
