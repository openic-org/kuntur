/***************************************************************************
Author: Manuel Monge
Description:
	Kuntur's regbank memory map — where the RHD2164 config/sampling command
	tables and the control registers live in the 256-word regbank address
	space. Split out of intan.vh, 2026-08-26 (PLAN.md B.3 "Decouple the
	AFE"): this is an application-level decision about this product's
	memory layout, not a property of the RHD2164 chip itself (see
	afe/rhd2164/rhd2164_defs.vh for that half).
***************************************************************************/

`ifndef REGBANK_MAP_VH
`define REGBANK_MAP_VH

// RegBank RAM

`define RB_CONFIG_BASE		8'b0
`define RB_CONFIG_ALLOC		8'd48
`define RB_SAMPLING_BASE 	(`RB_CONFIG_BASE + `RB_CONFIG_ALLOC)
`define RB_SAMPLING_ALLOC	8'd48
`define RB_CTRL_BASE			8'd192
`define RB_CONFIG_MAX		6'd33
`define RB_SAMPLING_MAX		6'd32

`endif // REGBANK_MAP_VH
