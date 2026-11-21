#!/bin/sh
# frame_rx unit tb (QuestaSim). Usage: tb/run_frame_rx.sh [plusargs] [-gSP=n]
#   FRX_SRC=<file>  compile frame_rx from this file instead of intan_rec_adapter.v
#   FRX_DIR=<dir>   work directory (default /tmp/frame_rx_tb)
set -e
TB=$(cd "$(dirname "$0")" && pwd)
IMPL=$(dirname "$TB")
SRC=${FRX_SRC:-$IMPL/intan_rec_adapter.v}
DIR=${FRX_DIR:-/tmp/frame_rx_tb}
export PATH=${QUESTA_BIN:-$HOME/lscc/radiant/2025.2.1/questasim/bin}:$PATH
mkdir -p "$DIR"; cd "$IMPL"		# the adapter's `includes are relative to impl_1
[ -d "$DIR/work" ] || vlib "$DIR/work" >/dev/null
vlog -quiet -work "$DIR/work" "$SRC"
vlog -quiet -sv -work "$DIR/work" "$TB/frame_rx_tb.sv"
G=""; P=""
for a in "$@"; do case "$a" in -G*|-g*) G="$G $a";; *) P="$P $a";; esac; done
cd "$DIR"
vsim -c -quiet -lib work $G frame_rx_tb $P -do "run -all; quit -f" | grep -v "^# //" | sed 's/^# //'
