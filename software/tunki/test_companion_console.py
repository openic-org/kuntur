#!/usr/bin/env python3
"""Checks for companion_console.py against companion-console.md §3, D6, D7."""

import io
import os
import tempfile

from companion_console import (COUNTER_MAX, LineError, Logger, SWEEPING, checksum,
                               describe_changes, format_line, parse_line, process)


def must_fail(line, why):
    try:
        parse_line(line)
    except LineError:
        return
    raise AssertionError(f"accepted: {why}: {line!r}")


def fix(line):
    """Recompute the checksum of an edited line (keeps the edit, fixes *XX)."""
    body = line[:line.rfind(" *") + 1]
    return f"{body}*{checksum(body):02X}\r\n"


# The §3 example, with its checksum computed (the spec's *5A is illustrative).
EX = ("CMP t=1234 link=up lock=1 ph=218/336 eye=218/156 mode=frames rx=37045123 "
      "crc=0 sent=12 rep=3 skip=2 gap=0 down=1 hw=7 mm=000000 design=2101 "
      "flags=05 snap=ok ")
EX = f"{EX}*{checksum(EX):02X}\r\n"
f = parse_line(EX)
assert f["t"] == 1234 and f["link"] == "up" and f["lock"] == 1
assert f["ph"] == 218 and f["eye"] == (218, 156) and f["mode"] == "frames"
assert f["rx"] == 37045123 and f["mm"] == 0 and f["design"] == 0x2101 and f["flags"] == 5
assert format_line(f) == EX
print("pass  §3 example parses and re-formats byte for byte")

# Checksum: XOR from C through the space before *.
assert checksum("CMP ") == ord("C") ^ ord("M") ^ ord("P") ^ ord(" ")
must_fail(EX.replace("rx=37045123", "rx=37045124"), "checksum not updated")
must_fail(EX[:-4] + "zz\r\n", "non-hex checksum")
rest = EX[EX.index("link="):EX.rfind(" *") + 1]
lo = next(l for l in (f"CMP t={n} {rest}" for n in range(1, 300))
          if not f"{checksum(l):02X}".isdigit())   # a line whose checksum has a letter
must_fail(f"{lo}*{checksum(lo):02x}\r\n", "lowercase checksum")
assert parse_line(f"{lo}*{checksum(lo):02X}\r\n")
print("pass  checksum: one changed digit is caught")

# D7: \r\n only.
must_fail(EX[:-2] + "\n", "LF only")
must_fail(EX[:-2] + "\r", "CR only")
assert parse_line(EX[:-2]) == f      # already stripped by a reader: accepted
print("pass  D7: \\r\\n line ending")

# D6: today's pattern build, fields not built print `-`.
today = dict(t=0, link=None, lock=0, ph=0, eye=SWEEPING, mode="pattern", rx=0, crc=0,
             sent=None, rep=None, skip=None, gap=None, down=None, hw=None,
             mm=None, design=None, flags=None, snap="stale")
line = format_line(today)
assert "link=- " in line and "eye=---/--- " in line and "flags=- " in line
assert parse_line(line) == today
print("pass  D6: `-` fields round-trip as None:", line.rstrip())

# Built-but-unset values (§3): mm ------, design ----, eye 0/0 after giving up.
g = dict(f, mm="------", design="----", eye=(0, 0))
assert parse_line(format_line(g))["mm"] == "------"
assert parse_line(format_line(g))["design"] == "----"
assert parse_line(format_line(g))["eye"] == (0, 0)
print("pass  ------ / ---- / 0/0 kept distinct from `-`")

# Format errors.
must_fail(fix(EX.replace("link=up lock=1", "lock=1 link=up")), "field order")
must_fail(fix(EX.replace(" hw=7", "")), "missing field")
must_fail(fix(EX.replace("crc=0", "crc=00")), "leading zero")
must_fail(fix(EX.replace("rx=37045123", f"rx={COUNTER_MAX + 1}")), "above 32 bits")
must_fail(fix(EX.replace("ph=218/336", "ph=336/336")), "phase out of range")
must_fail(fix(EX.replace("ph=218/336", "ph=218/280")), "old step count")
must_fail(fix(EX.replace("mm=000000", "mm=00000")), "mm width")
must_fail(fix(EX.replace("snap=ok", "snap=OK")), "snap value")
must_fail(fix(EX.replace("rep=3 ", "rep=3  ")), "double space")
must_fail(fix(EX.replace("CMP ", "CMQ ")), "prefix")
assert parse_line(fix(EX.replace("rx=37045123", f"rx={COUNTER_MAX}")))["rx"] == COUNTER_MAX
print("pass  order, count, leading zeros, 32 bits, ranges, widths, spacing")

# Changes worth printing.
a = parse_line(EX)
b = dict(a, t=1235, crc=2, rx=a["rx"] + 25000)
assert describe_changes(a, b) == ["crc +2 (now 2)"]
assert describe_changes(a, dict(a, t=1235)) == ["rx not moving"]
assert describe_changes(a, dict(b, t=3)) == ["adapter restarted (t 1234 -> 3)"]
m = describe_changes(a, dict(b, crc=0, t=1236, eye=(50, 60)))
assert m[0] == "1 line(s) missing" and "BELOW G5" in m[1], m
assert "ok)" in describe_changes(a, dict(b, crc=0, eye=(50, 84)))[0]
# a stale snapshot repeats old counters: no "rx not moving" from it
assert describe_changes(a, dict(a, t=1235, snap="stale")) == ["snap ok -> stale"]
print("pass  changes: crc, rx stall, restart, missing lines, G5 eye, stale")

# process + CSV: one bad line counted, never logged.
with tempfile.TemporaryDirectory() as d:
    path = os.path.join(d, "c.csv")
    lg = Logger(path)
    out = io.StringIO()
    lines = [EX, EX.replace("rx=3", "rx=4"), format_line(dict(a, t=1235, rx=a["rx"] + 1))]
    good, bad = process(lines, lg, out=out, clock=lambda: 1.0e9)
    lg.close()
    rows = open(path).read().splitlines()
    assert (good, bad) == (2, 1) and len(rows) == 3, (good, bad, rows)
    assert rows[0].startswith("host_time,t,link,") and ",218/156," in rows[1]
    assert "bad line" in out.getvalue()
print("pass  process: bad lines counted, not logged; CSV has every field")

print("ALL COMPANION-CONSOLE CHECKS PASSED")
