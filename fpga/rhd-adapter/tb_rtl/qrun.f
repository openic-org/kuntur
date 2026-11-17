// From: https://www.latticesemi.com/en/Blog/2026/02/17/07/05/Optimizing-FPGA-Design-Simulation-Workflow-with-QuestaSim-qrun

// Companion RTL in its own library: it and kuntur_fpga.v (pulled in by
// cdr_tb.sv for block_gen) both `include the common modules, so they cannot
// share a compilation unit. cdr_tb takes frame_aligner from here.
-makelib lib_rx ../source/impl_1/intan_rec_adapter.v -end

// Create library lib1 in qrun.out and compile testbench into it
-makelib lib1 +define+SIM ../source/impl_1/tb/cdr_tb.sv -end -top cdr_tb

// Create a PDU (Pre-Optimized Design Unit) from module m1 in lib1
// Useful for Lattice Libraries wich will not change
// -makepdu m1_opt_pdu lib1.m1
// -end
-optionset VOPTDEBUG
+noacc+pmi_work.*
+noacc+ovi_lfcpnx.*
-vsim.options
-end

-do "view wave"
-do "do wave1.do"
-do "run -all"
