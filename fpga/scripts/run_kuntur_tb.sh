#!/usr/bin/env bash
# Run kuntur_tb for every build variant (1, 2, 3) in QuestaSim, in
# parallel, and print each one's summary and failures. Also checks that a
# build with no variant define stops at elaboration (kuntur_build.vh
# defaults KUNTUR_VARIANT to 0, which the variant guards reject), and runs
# eye_models_tb, the self-test of the #6a models kuntur_tb uses (pll_sim's
# dynamic phase, rhd2164_bfm's MOSI window), and phase_stepper_tb (#6a's
# stepper against pll_sim) at both rates.
#
# The three Radiant implementations share the RTL (task #6, vega
# log/2026-11-04.md), and so does the testbench: one RTL change, three
# runs. About a minute each.
#
# Usage, from anywhere:
#   run_kuntur_tb.sh            variants 1 2 3, plus the no-define check
#   run_kuntur_tb.sh 2          only these variants
#   KEEP=1 run_kuntur_tb.sh     keep the work folders (vsim.log per variant)
#   SRC=DIR run_kuntur_tb.sh    test a copy of source/impl_1 instead
#
# QuestaSim from $QUESTA_DIR, else the newest Radiant install's.
# Exit status: 0 every variant ran to its summary with 0 failures and the
# no-define build failed to elaborate; 1 otherwise.

set -u

here=$(cd "$(dirname "$0")" && pwd)
# SRC: another copy of source/impl_1 (e.g. a scratch checkout) instead
R=$(cd "${SRC:-$here/../headstage/source/impl_1}" && pwd)

if [ -z "${QUESTA_DIR:-}" ]; then
	QUESTA_DIR=$(ls -d "$HOME"/lscc/radiant/*/questasim/linux_x86_64 2>/dev/null | sort | tail -1)
fi
if [ ! -x "$QUESTA_DIR/vsim" ]; then
	echo "QuestaSim not found; set QUESTA_DIR" >&2
	exit 1
fi

variants=("$@")
[ ${#variants[@]} -eq 0 ] && variants=(1 2 3)

work=$(mktemp -d "${TMPDIR:-/tmp}/kuntur_tb.XXXXXX")
[ -z "${KEEP:-}" ] && trap 'rm -rf "$work"' EXIT

incs=(+incdir+"$R" +incdir+"$R/tb" +incdir+"$R/afe/rhd2164" +incdir+"$R/app")

# compile DIR [DEFINE...]: vlog into DIR/work, log in DIR/vlog.log
compile() {
	local d=$1; shift
	mkdir -p "$d" && cd "$d" || return 1
	"$QUESTA_DIR/vlib" work > /dev/null
	"$QUESTA_DIR/vlog" -quiet -sv "$@" "${incs[@]}" "$R/tb/kuntur_tb.sv" > vlog.log 2>&1
}

pids=()
for v in "${variants[@]}"; do
	(
		compile "$work/v$v" "+define+KUNTUR_VARIANT=4'd$v" &&
		"$QUESTA_DIR/vsim" -c kuntur_tb -do "run -all; quit -f" > vsim.log 2>&1
	) &
	pids+=($!)
done
(
	compile "$work/none" &&
	"$QUESTA_DIR/vsim" -c kuntur_tb -do "run 1us; quit -f" > vsim.log 2>&1
) &
pids+=($!)
(
	mkdir -p "$work/models" && cd "$work/models" &&
	"$QUESTA_DIR/vlib" work > /dev/null &&
	"$QUESTA_DIR/vlog" -quiet -sv "${incs[@]}" "$R/tb/eye_models_tb.sv" > vlog.log 2>&1 &&
	"$QUESTA_DIR/vsim" -c eye_models_tb -do "run -all; quit -f" > vsim.log 2>&1
) &
pids+=($!)
for rate in 25 28; do
	(
		mkdir -p "$work/stepper$rate" && cd "$work/stepper$rate" &&
		"$QUESTA_DIR/vlib" work > /dev/null &&
		"$QUESTA_DIR/vlog" -quiet -sv "+define+KUNTUR_VARIANT=4'd2" "${incs[@]}" "$R/tb/phase_stepper_tb.sv" > vlog.log 2>&1 &&
		"$QUESTA_DIR/vsim" -c phase_stepper_tb -gRATE=$rate -do "run -all; quit -f" > vsim.log 2>&1
	) &
	pids+=($!)
done
wait "${pids[@]}"

status=0
for v in "${variants[@]}"; do
	d=$work/v$v
	echo "== variant $v"
	if grep -q "Error" "$d/vlog.log"; then
		echo "  compile errors:"
		grep -E "Error|^\*\* at " "$d/vlog.log" | sed 's/^/    /'
		status=1
		continue
	fi
	summary=$(grep -E "^#\s+[0-9]+ checks, [0-9]+ failures" "$d/vsim.log" | tail -1 | sed 's/^#\s*//')
	if [ -z "$summary" ]; then
		echo "  no summary line: the run did not finish"
		grep -E "\*\* (Error|Fatal)" "$d/vsim.log" | head -5 | sed 's/^/    /'
		status=1
		continue
	fi
	echo "  $summary"
	grep "^# \*\* Error" "$d/vsim.log" | sed 's/ *Time:.*//; s/^# \*\* Error: /    /' | sort | uniq -c | sed 's/^ */    /'
	case $summary in
		*" 0 failures") ;;
		*) status=1 ;;
	esac
done

echo "== no variant define"
if grep -qs "KUNTUR_VARIANT_must_be_1_2_or_3" "$work/none/vsim.log" "$work/none/vlog.log"; then
	echo "  stops at elaboration (KUNTUR_VARIANT_must_be_1_2_or_3), as it should"
elif grep -q "Error" "$work/none/vlog.log"; then
	echo "  not checked: the RTL does not compile (see above)"
	status=1
else
	echo "  did NOT stop: the variant guard is not working"
	status=1
fi

echo "== eye_models_tb (#6a models)"
m=$(grep -Es "^#\s+[0-9]+ checks, [0-9]+ failures" "$work/models/vsim.log" | tail -1 | sed 's/^#\s*//')
if [ -z "$m" ]; then
	echo "  did not run:"
	grep -Es "Error|^\*\* at " "$work/models/vlog.log" "$work/models/vsim.log" | head -5 | sed 's/^/    /'
	status=1
else
	echo "  $m"
	grep -s "^# \*\* Error" "$work/models/vsim.log" | sed 's/ *Time:.*//; s/^# \*\* Error: /    /'
	case $m in *" 0 failures") ;; *) status=1 ;; esac
fi

for rate in 25 28; do
	echo "== phase_stepper_tb ${rate}k (#6a)"
	m=$(grep -Es "^#\s+[0-9]+ checks, [0-9]+ failures" "$work/stepper$rate/vsim.log" | tail -1 | sed 's/^#\s*//')
	if [ -z "$m" ]; then
		echo "  did not run:"
		grep -Es "Error|^\*\* at " "$work/stepper$rate/vlog.log" "$work/stepper$rate/vsim.log" | head -5 | sed 's/^/    /'
		status=1
	else
		echo "  $m"
		grep -s "^# \*\* Error" "$work/stepper$rate/vsim.log" | sed 's/ *Time:.*//; s/^# \*\* Error: /    /'
		case $m in *" 0 failures") ;; *) status=1 ;; esac
	fi
done

[ -n "${KEEP:-}" ] && echo "work folders kept in $work"
echo
[ $status -eq 0 ] && echo PASS || echo FAIL
exit $status
