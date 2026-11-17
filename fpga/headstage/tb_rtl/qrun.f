// From: https://www.latticesemi.com/en/Blog/2026/02/17/07/05/Optimizing-FPGA-Design-Simulation-Workflow-with-QuestaSim-qrun

// Create library lib1 in qrun.out and compile testbench into it
// The variant must be given (kuntur_build.vh defaults to 0): 4'd1, 4'd2 or 4'd3.
// Quoted, or the option-file parser reads the ' as a string start.
-makelib lib1 "+define+KUNTUR_VARIANT=4'd2" ../source/impl_1/tb/kuntur_tb.sv -end

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
-do "add wave /dut/*"
-do "run 1us"
