#!/usr/bin/env python3
"""check_console.py — line checker for console_tb (companion-console.md §8.9).

console_tb decodes con_tx and writes one row per line:

    k t lock exp c1 .. c6 | line

k = tick number, t / lock = what the line must show, exp = ok / stale / any
(what snap must be, from the time the shadow copied), c1..c6 = the seven
wclk inputs "rdy,valid,ph,ec,ew,rx,crc" as sampled by the 1st..6th wclk
edge after that second's snap_req toggle ("-" if wclk did not run).

Every line is parsed with companion_console.parse_line (the Tunki parser),
and checked:
  - t, lock as given; mode=pattern; the D6 fields "-";
  - snap=ok: exp is not stale, and ph/eye/rx/crc are the values of the 3rd
    or 4th edge's candidate (one wclk edge, so not torn; §8.7 S2, U8),
    with eye per U7; that record becomes the last good one. The first ok
    record may come from the 3rd..6th edge: at power-up the request waits
    while console_shadow is still in reset (no wclk), and its synchroniser
    lets go on the 2nd edge once wclk starts (S4), then 3 more;
  - snap=stale: exp is not ok, and the fields are the last good record
    (zeros, eye ---/---, before the first, N9) (U9);
  - at least one line each with t, rx and crc = 4294967295 (U10, printed
    saturated), and at least one ok, one stale, the three eye forms.

Usage:
  python3 check_console.py console_lines.txt
  python3 check_console.py --mutants                 list the RTL mutants
  python3 check_console.py --mutate K IN.v OUT.v     write mutant K of IN.v
"""

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "..", "..", "..", "..",
                                                 "software", "tunki")))
from companion_console import COUNTER_MAX, LineError, SWEEPING, parse_line  # noqa: E402

# wiring faults of the console wrapper (and one in the UART), each must FAIL
MUTANTS = {
    1: ("console_fmt reads the live phase, not the record",
        r"\.phase(\s*)\(h_phase\)", r".phase\1(phase)"),
    2: ("snap_ok tied to 1",
        r"\.snap_ok(\s*)\(h_snap_ok\)", r".snap_ok\1(1'b1)"),
    3: ("TIMEOUT not passed down right (3 clocks)",
        r"\.TIMEOUT\s*\(TIMEOUT\)", r".TIMEOUT (24'd3)"),
    4: ("rx shows pat_errors",
        r"\.pat_blocks(\s*)\(h_pat_blocks\)", r".pat_blocks\1(h_pat_errors)"),
    5: ("DIV passed as DIV + 1",
        r"\.DIV\s*\(DIV\)", r".DIV (DIV + 24'd1)"),
    6: ("UART bit time 103 clocks",
        r"BIT_TIMING_MAX = 8'd104", r"BIT_TIMING_MAX = 8'd103"),
    7: ("console_fmt reads the live lock_sync, not the record",
        r"\.lock(\s*)\(h_lock\)", r".lock\1(lock_sync)"),
    8: ("console_shadow never reset",
        r"(console_shadow_0\s*\(\s*(?://[^\n]*\n\s*)?\.rst_n\s*)\(con_rst_n\)", r"\1(1'b1)"),
}


def mutate(k, src, dst):
    _, pat, rep = MUTANTS[k]
    with open(src) as f:
        text = f.read()
    new, n = re.subn(pat, rep, text, count=1)
    if n != 1:
        sys.exit(f"mutant {k}: pattern not found")
    with open(dst, "w") as f:
        f.write(new)


def unescape(s):
    return s.replace("\\r", "\r").replace("\\n", "\n")


def eye_of(rdy, valid, ec, ew):
    return SWEEPING if not rdy else ((0, 0) if not valid else (ec, ew))


def rec_fields(c):
    rdy, valid, ph, ec, ew, rx, crc = c
    return dict(ph=ph, eye=eye_of(rdy, valid, ec, ew), rx=rx, crc=crc)


ZERO = (0, 0, 0, 0, 0, 0, 0)


def main(path):
    fails = n = 0
    last_good = ZERO
    seen = dict(ok=0, stale=0, sweep=0, gaveup=0, eye=0, t_sat=0, rx_sat=0, crc_sat=0)
    offsets = {}

    def fail(no, msg):
        nonlocal fails
        fails += 1
        if fails <= 25:
            print(f"FAIL  line {no}: {msg}")

    with open(path) as f:
        rows = [r.rstrip("\n") for r in f if r.strip()]
    for no, raw in enumerate(rows, 1):
        n += 1
        head, _, line = raw.partition(" | ")
        parts = head.split()
        k, t, lock, exp = int(parts[0]), int(parts[1]), int(parts[2]), parts[3]
        cands = [None if c == "-" else tuple(map(int, c.split(","))) for c in parts[4:10]]
        try:
            got = parse_line(unescape(line))
        except LineError as e:
            fail(no, f"parse_line rejects it ({e}): {line}")
            continue
        want = dict(t=t, link=None, lock=lock, mode="pattern", sent=None, rep=None,
                    skip=None, gap=None, down=None, hw=None, mm=None, design=None, flags=None)
        diff = {key: (want[key], got[key]) for key in want if want[key] != got[key]}
        if diff:
            fail(no, f"fields differ (want, got) {diff}")
        fields = {key: got[key] for key in ("ph", "eye", "rx", "crc")}
        if got["snap"] == "ok":
            seen["ok"] += 1
            if exp == "stale":
                fail(no, "snap=ok, but the shadow was not copied within 10 ms (U9)")
            match = [i for i, c in enumerate(cands) if c is not None and rec_fields(c) == fields]
            if not match:
                fail(no, f"snap=ok record is no single wclk edge's values (U8): {fields}")
            else:
                allowed = (3, 4) if seen["ok"] > 1 else (3, 4, 5, 6)
                good = [i for i in match if i + 1 in allowed]
                i = (good or match)[0]       # equal candidates (e.g. rdy 0): any 3rd/4th will do
                offsets[i + 1] = offsets.get(i + 1, 0) + 1
                if i + 1 not in allowed:
                    fail(no, f"record from wclk edge {i + 1} after the request, not {allowed} (S2, S4)")
                last_good = cands[i]
        else:
            seen["stale"] += 1
            if exp == "ok":
                fail(no, "snap=stale, but the shadow was copied within 10 ms (U8)")
            if fields != rec_fields(last_good):
                fail(no, f"stale line is not the last good record (U9/N9): want "
                         f"{rec_fields(last_good)}, got {fields}")
        e = got["eye"]
        seen["sweep" if e == SWEEPING else "gaveup" if e == (0, 0) else "eye"] += 1
        seen["t_sat"] += got["t"] == COUNTER_MAX
        seen["rx_sat"] += got["rx"] == COUNTER_MAX
        seen["crc_sat"] += got["crc"] == COUNTER_MAX
    if n == 0:
        print("check_console: FAIL (no lines)")
        return 1
    for key, v in seen.items():
        if v == 0:
            fail("-", f"no line with {key}")
    print(f"      {n} lines: {seen}; ok records from wclk edge {dict(sorted(offsets.items()))}")
    if fails:
        print(f"check_console: {fails} FAIL of {n} lines")
        return 1
    print(f"check_console: ALL PASS ({n} lines)")
    return 0


if __name__ == "__main__":
    a = sys.argv[1:]
    if a[:1] == ["--mutants"]:
        for k, (d, _, _) in MUTANTS.items():
            print(f"{k}: {d}")
        sys.exit(0)
    if a[:1] == ["--mutate"]:
        mutate(int(a[1]), a[2], a[3])
        sys.exit(0)
    sys.exit(main(a[0] if a else "console_lines.txt"))
