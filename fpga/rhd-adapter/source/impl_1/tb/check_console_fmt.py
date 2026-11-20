#!/usr/bin/env python3
"""check_console_fmt.py — second check of console_fmt_tb's lines.

console_fmt_tb writes one "inputs | line" per line it captured from the
formatter. This parses every line with companion_console.parse_line (the
host's own parser, software/tunki/companion_console.py) and compares the
fields with the inputs that produced it, so the RTL and the host parser are
checked against each other, independently of the tb's own formatter.

Usage: python3 check_console_fmt.py console_fmt_lines.txt
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "..", "..", "..", "..",
                                                 "software", "tunki")))
from companion_console import LineError, SWEEPING, parse_line  # noqa: E402


def unescape(s):
    return s.replace("\\r", "\r").replace("\\n", "\n")


def main(path):
    fails = n = 0
    with open(path) as f:
        for no, raw in enumerate(f, 1):
            raw = raw.rstrip("\n")
            if not raw:
                continue
            n += 1
            ins, _, line = raw.partition(" | ")
            t, lock, snap_ok, rdy, valid, ph, ec, ew, rx, crc, mode = map(int, ins.split())
            want_eye = SWEEPING if not rdy else ((0, 0) if not valid else (ec, ew))
            want = dict(t=t, link=None, lock=lock, ph=ph, eye=want_eye,
                        mode="pattern" if mode else "frames", rx=rx, crc=crc,
                        sent=None, rep=None, skip=None, gap=None, down=None, hw=None,
                        mm=None, design=None, flags=None,
                        snap="ok" if snap_ok else "stale")
            try:
                got = parse_line(unescape(line))
            except LineError as e:
                fails += 1
                if fails <= 20:
                    print(f"FAIL  line {no}: parse_line rejects it ({e}): {line}")
                continue
            if got != want:
                fails += 1
                if fails <= 20:
                    diff = {k: (want[k], got[k]) for k in want if want[k] != got[k]}
                    print(f"FAIL  line {no}: fields differ (want, got) {diff}")
    if n == 0:
        print("check_console_fmt: FAIL (no lines)")
        return 1
    if fails:
        print(f"check_console_fmt: {fails} FAIL of {n} lines")
        return 1
    print(f"check_console_fmt: ALL PASS ({n} lines)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "console_fmt_lines.txt"))
