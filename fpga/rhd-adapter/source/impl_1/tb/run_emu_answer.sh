#!/bin/sh
# emu_answer unit tb (QuestaSim). Usage: tb/run_emu_answer.sh [plusargs] [-g...]
#   TB_SRC="<files>"  compile the slave, reg_ram and emu_answer from these files instead of intan_rec_adapter.v
#   TB_DIR=<dir>      work directory (default /tmp/emu_answer_tb)
set -e
TB=$(cd "$(dirname "$0")" && pwd)
IMPL=$(dirname "$TB")
SRC=${TB_SRC:-$IMPL/intan_rec_adapter.v}
DIR=${TB_DIR:-/tmp/emu_answer_tb}
export PATH=${QUESTA_BIN:-$HOME/lscc/radiant/2025.2.1/questasim/bin}:$PATH
D=""; G=""; P=""
for a in "$@"; do case "$a" in +define+*) D="$D $a";; -G*|-g*) G="$G $a";; *) P="$P $a";; esac; done
mkdir -p "$DIR"; cd "$IMPL"		# the adapter's `includes are relative to impl_1
rm -rf "$DIR/work"; vlib "$DIR/work" >/dev/null
vlog -quiet -work "$DIR/work" +define+KUNTUR_VARIANT=2 $SRC
vlog -quiet -sv -work "$DIR/work" $D "$TB/intan_master_bfm.v" "$TB/emu_answer_tb.sv"
cd "$DIR"
vsim -c -quiet -lib work $G emu_answer_tb $P -do "run -all; quit -f" | grep -v "^# //" | sed 's/^# //'
