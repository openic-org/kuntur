#!/bin/sh
# tunnel_e2e_tb: compile both tops into their own libraries and run.
#   run_e2e.sh a [plusargs...]     stage A (Kuntur variant 3, cdr)
#   run_e2e.sh b [plusargs...]     stage B (Kuntur variant 2, frames, T5)
# Works in $E2E_DIR (default: a new temporary directory). ADAPTER_RTL=<file>
# compiles another copy of intan_rec_adapter.v (its includes resolve through
# +incdir, so the copy may live anywhere).
set -e
Q=${QUESTA:-$HOME/lscc/radiant/2025.2.1/questasim/bin}
HERE=$(cd "$(dirname "$0")" && pwd)
A=$(cd "$HERE/.." && pwd)
K=$(cd "$A/../../../headstage/source/impl_1" && pwd)
STAGE=$1; shift
case $STAGE in
	a) V="4'd3"; PA="" ;;
	b) V="4'd2"; PA="+stageb" ;;
	*) echo "usage: $0 a|b [plusargs]"; exit 2 ;;
esac
D=${E2E_DIR:-$(mktemp -d /tmp/e2e.XXXXXX)}
cd "$D"
rm -rf kun adp work
$Q/vlib kun >/dev/null; $Q/vlib adp >/dev/null; $Q/vlib work >/dev/null
$Q/vlog -quiet -sv -work kun +incdir+$K +incdir+$K/afe/rhd2164 +incdir+$K/app \
	"+define+KUNTUR_VARIANT=$V" $K/tb/kuntur_prims_sim.v $K/kuntur_fpga.v
$Q/vlog -quiet -work adp +incdir+$A +incdir+$K/afe/rhd2164 "+define+KUNTUR_VARIANT=$V" $HERE/adapter_prims_sim.v ${ADAPTER_RTL:-$A/intan_rec_adapter.v}
$Q/vlog -quiet -sv -work work $K/tb/rhd2164_bfm.v $HERE/intan_master_bfm.v $HERE/tunnel_e2e_tb.sv
$Q/vsim -c -quiet -L kun -L adp tunnel_e2e_cfg $PA "$@" -do "run -all; quit -f" | grep -v "^# \(Loading\|//\)"
