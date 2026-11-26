#!/bin/sh
# link_mon (T5.6) unit tb (QuestaSim). Usage: tb/run_link_mon.sh [plusargs] [-g...]
#   TB_SRC="<files>"  compile link_mon from these files instead of intan_rec_adapter.v
#   TB_DIR=<dir>      work directory (default /tmp/link_mon_tb)
set -e
TB=$(cd "$(dirname "$0")" && pwd)
IMPL=$(dirname "$TB")
SRC=${TB_SRC:-$IMPL/intan_rec_adapter.v}
DIR=${TB_DIR:-/tmp/link_mon_tb}
export PATH=${QUESTA_BIN:-$HOME/lscc/radiant/2025.2.1/questasim/bin}:$PATH
D=""; G=""; P=""
for a in "$@"; do case "$a" in +define+*) D="$D $a";; -G*|-g*) G="$G $a";; *) P="$P $a";; esac; done
mkdir -p "$DIR"; cd "$IMPL"		# the adapter's `includes are relative to impl_1
rm -rf "$DIR/work"; vlib "$DIR/work" >/dev/null
vlog -quiet -work "$DIR/work" +define+KUNTUR_VARIANT=2 $SRC
vlog -quiet -sv -work "$DIR/work" $D "$TB/link_mon_tb.sv"
cd "$DIR"
vsim -c -quiet -lib work $G link_mon_tb $P -do "run -all; quit -f" | grep -v "^# //" | sed 's/^# //'
