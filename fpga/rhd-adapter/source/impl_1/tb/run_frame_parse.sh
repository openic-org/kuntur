#!/bin/sh
# frame_parse unit tb (QuestaSim). Usage: tb/run_frame_parse.sh [plusargs] [+define+FP_SAT] [-g...]
#   FP_SRC="<files>"  compile frame_parse and frame_rx from these files instead of intan_rec_adapter.v
#   FP_DIR=<dir>      work directory (default /tmp/frame_parse_tb)
set -e
TB=$(cd "$(dirname "$0")" && pwd)
IMPL=$(dirname "$TB")
SRC=${FP_SRC:-$IMPL/intan_rec_adapter.v}
DIR=${FP_DIR:-/tmp/frame_parse_tb}
export PATH=${QUESTA_BIN:-$HOME/lscc/radiant/2025.2.1/questasim/bin}:$PATH
D=""; G=""; P=""
for a in "$@"; do case "$a" in +define+*) D="$D $a";; -G*|-g*) G="$G $a";; *) P="$P $a";; esac; done
mkdir -p "$DIR"; cd "$IMPL"		# the adapter's `includes are relative to impl_1
rm -rf "$DIR/work"; vlib "$DIR/work" >/dev/null
vlog -quiet -work "$DIR/work" $SRC
vlog -quiet -sv -work "$DIR/work" $D "$TB/frame_parse_tb.sv"
cd "$DIR"
vsim -c -quiet -lib work $G frame_parse_tb $P -do "run -all; quit -f" | grep -v "^# //" | sed 's/^# //'
