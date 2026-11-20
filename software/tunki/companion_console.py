#!/usr/bin/env python3
"""
companion_console.py — reader for the RHD adapter's diagnostics console (O8).

Spec: kuntur-notes docs/interfaces/companion-console.md (§3 line format,
D6 `-` for fields not built, D7 `\\r\\n`). The adapter sends one ASCII line
per second on FT2232H port 1, 115,200 8N1:

  CMP t=1234 link=up lock=1 ph=218/336 eye=218/156 mode=frames rx=37045123 ...
      ... snap=ok *5A

This script:
  - checks every line's checksum and field order (bad lines are counted,
    never used);
  - writes bench/companion_YYYYMMDD_HHMMSS.csv (host time + every field);
  - prints the changes worth seeing: a new CRC/pattern error, lock or link
    changes, the eye after a sweep, `snap` going stale, the adapter
    restarting (t going backwards), `rx` stopping;
  - says so when the port is silent ("no bytes for 5 s"), or delivers
    bytes but no line end (wrong baud rate), every 5 s until it recovers.

The parser (`parse_line`) is the one `console_tb`'s checks are written
against, so the bench and the simulation check the same thing.

Usage:
  python companion_console.py                       # default by-id port
  python companion_console.py --port /dev/ttyUSB1 --duration 600
  python companion_console.py --file capture.txt    # parse a saved capture
"""

import argparse
import csv
import datetime as dt
import os
import sys
import time

DEFAULT_PORT = ("/dev/serial/by-id/"
                "usb-Lattice_Lattice_CrossLink-NX_Eval_Board_FT6WVUEH-if01-port0")
BAUD_RATE = 115_200
STEPS = 336                 # phase steps per turn at 37.95 MHz (lvds-tunnel.md §6.6)
EYE_MIN = 84                # G5: >= 25% of a bit each side (companion-console.md §3)
COUNTER_MAX = 0xFFFFFFFF

NOT_BUILT = None            # D6: a `-` value
SWEEPING = "sweeping"       # eye=---/---


class LineError(ValueError):
    """A console line that fails the format or the checksum."""


def checksum(body: str) -> int:
    """XOR of every byte from `C` up to and including the space before `*`."""
    x = 0
    for b in body.encode("ascii"):
        x ^= b
    return x


def _counter(v):
    if not v.isdigit() or (len(v) > 1 and v[0] == "0"):
        raise LineError(f"bad counter {v!r}")
    n = int(v)
    if n > COUNTER_MAX:
        raise LineError(f"counter {v} above 32 bits")
    return n


def _choice(*opts):
    def f(v):
        if v not in opts:
            raise LineError(f"{v!r} not one of {opts}")
        return v
    return f


def _hex(width, unset):
    def f(v):
        if v == unset:
            return unset
        if len(v) != width or any(c not in "0123456789ABCDEFabcdef" for c in v):
            raise LineError(f"bad {width}-digit hex {v!r}")
        return int(v, 16)
    return f


def _small(v):
    if not v.isdigit() or (len(v) > 1 and v[0] == "0"):
        raise LineError(f"bad number {v!r}")
    return int(v)


def _ph(v):
    a, sep, b = v.partition("/")
    if not sep:
        raise LineError(f"bad ph {v!r}")
    p, s = _small(a), _small(b)
    if s != STEPS or p >= s:
        raise LineError(f"ph {v!r} outside 0..{STEPS - 1}/{STEPS}")
    return p


def _eye(v):
    if v == "---/---":
        return SWEEPING
    a, sep, b = v.partition("/")
    if not sep:
        raise LineError(f"bad eye {v!r}")
    c, w = _small(a), _small(b)
    if c >= STEPS or w > STEPS:
        raise LineError(f"eye {v!r} out of range")
    return (c, w)


# Fixed order (§3). Each value is parsed by its function, or is `-` (D6).
FIELDS = [
    ("t", _counter),
    ("link", _choice("up", "down")),
    ("lock", _choice("0", "1")),
    ("ph", _ph),
    ("eye", _eye),
    ("mode", _choice("pattern", "frames")),
    ("rx", _counter),
    ("crc", _counter),
    ("sent", _counter),
    ("rep", _counter),
    ("skip", _counter),
    ("gap", _counter),
    ("down", _counter),
    ("hw", _counter),
    ("mm", _hex(6, "------")),
    ("design", _hex(4, "----")),
    ("flags", _hex(2, None)),
    ("snap", _choice("ok", "stale")),
]
FIELD_NAMES = [k for k, _ in FIELDS]


def parse_line(line: str) -> dict:
    """Parse one console line (with or without its `\\r\\n`) into a dict.

    `-` values become None (D6). `eye` is (centre, width), "sweeping", or
    (0, 0) after a sweep that gave up. Raises LineError on any defect.
    """
    if line.endswith("\r\n"):
        line = line[:-2]
    elif line.endswith("\n") or line.endswith("\r"):
        raise LineError("line ending is not \\r\\n")
    if not line.startswith("CMP "):
        raise LineError("does not start with 'CMP '")
    star = line.rfind(" *")
    if star < 0 or len(line) != star + 4:
        raise LineError("no ' *XX' checksum at the end")
    body, cks = line[:star + 1], line[star + 2:]
    if cks != cks.upper():
        raise LineError("checksum not uppercase hex")
    try:
        want = int(cks, 16)
    except ValueError:
        raise LineError(f"bad checksum {cks!r}")
    got = checksum(body)
    if got != want:
        raise LineError(f"checksum {cks}, computed {got:02X}")
    tokens = body[4:-1].split(" ")
    if len(tokens) != len(FIELDS):
        raise LineError(f"{len(tokens)} fields, expected {len(FIELDS)}")
    out = {}
    for tok, (name, fn) in zip(tokens, FIELDS):
        key, eq, val = tok.partition("=")
        if key != name or not eq:
            raise LineError(f"field {tok!r} where {name}= expected")
        out[name] = NOT_BUILT if val == "-" else fn(val)
    if out["lock"] is not None:
        out["lock"] = int(out["lock"])
    return out


def format_line(fields: dict) -> str:
    """Build a console line from a dict as parse_line returns it (for tests
    and testbench reference models). Includes the `\\r\\n`."""
    def enc(name, v):
        if v is None:
            return "-"
        if name == "ph":
            return f"{v}/{STEPS}"
        if name == "eye":
            return "---/---" if v == SWEEPING else f"{v[0]}/{v[1]}"
        if name == "mm":
            return v if isinstance(v, str) else f"{v:06X}"
        if name == "design":
            return v if isinstance(v, str) else f"{v:04X}"
        if name == "flags":
            return f"{v:02X}"
        return str(v)
    body = "CMP " + " ".join(f"{n}={enc(n, fields.get(n))}" for n in FIELD_NAMES) + " "
    return f"{body}*{checksum(body):02X}\r\n"


def describe_changes(prev, cur):
    """Return the notable changes from one good line to the next, as text."""
    msgs = []
    if prev is None:
        return msgs
    if cur["t"] is not None and prev["t"] is not None:
        if cur["t"] < prev["t"]:
            msgs.append(f"adapter restarted (t {prev['t']} -> {cur['t']})")
            return msgs
        if cur["t"] > prev["t"] + 1:
            msgs.append(f"{cur['t'] - prev['t'] - 1} line(s) missing")
    for k in ("lock", "link", "snap", "mode"):
        if cur[k] != prev[k]:
            msgs.append(f"{k} {prev[k]} -> {cur[k]}")
    if cur["eye"] != prev["eye"]:
        e = cur["eye"]
        if e == SWEEPING:
            msgs.append("sweep started")
        elif e == (0, 0):
            msgs.append("sweep gave up (eye 0/0)")
        elif e is not None:
            c, w = e
            ok = "ok" if w >= EYE_MIN else f"BELOW G5 minimum {EYE_MIN}"
            msgs.append(f"eye centre {c}, width {w} steps ({w * 78.4 / 1000:.1f} ns, {ok})")
    if cur["snap"] == "ok":
        for k in ("crc", "sent", "rep", "skip", "gap", "down", "hw"):
            a, b = prev[k], cur[k]
            if a is not None and b is not None and b > a:
                msgs.append(f"{k} +{b - a} (now {b})")
        if (prev["rx"] is not None and cur["rx"] == prev["rx"]
                and cur["eye"] not in (SWEEPING, (0, 0))):
            msgs.append("rx not moving")
        for k in ("mm", "design", "flags"):
            if cur[k] != prev[k] and prev[k] is not None:
                msgs.append(f"{k} {prev[k]} -> {cur[k]}")
        for k in FIELD_NAMES:
            if cur[k] == COUNTER_MAX and prev[k] != COUNTER_MAX:
                msgs.append(f"{k} saturated")
    return msgs


def _csv_value(v):
    if v is None:
        return "-"
    if isinstance(v, tuple):
        return f"{v[0]}/{v[1]}"
    return v


class Logger:
    def __init__(self, path):
        self.path = path
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["host_time"] + FIELD_NAMES)

    def write(self, host_time, fields):
        self.w.writerow([host_time] + [_csv_value(fields[k]) for k in FIELD_NAMES])
        self.f.flush()

    def close(self):
        self.f.close()


def process(lines, logger=None, out=sys.stdout, clock=time.time, stats=None):
    """Parse an iterable of raw lines; log good ones and print changes.
    Counts go into `stats` ({"good", "bad"}) as they happen, so an
    interrupt keeps them. Returns (good, bad)."""
    if stats is None:
        stats = {}
    stats.setdefault("good", 0)
    stats.setdefault("bad", 0)
    prev = None
    for raw in lines:
        if not raw.strip():
            continue
        now = clock()
        stamp = dt.datetime.fromtimestamp(now).strftime("%H:%M:%S")
        try:
            cur = parse_line(raw)
        except LineError as e:
            stats["bad"] += 1
            print(f"{stamp} bad line ({e}): {raw.rstrip()!r}", file=out)
            continue
        stats["good"] += 1
        if logger:
            logger.write(f"{now:.3f}", cur)
        if prev is None:
            print(f"{stamp} first line: {raw.rstrip()}", file=out)
        for m in describe_changes(prev, cur):
            print(f"{stamp} {m}", file=out)
        prev = cur
    return stats["good"], stats["bad"]


QUIET_S = 5.0                # report a silent port after this long


def _serial_lines(port, deadline, out=sys.stdout, clock=time.time, quiet=QUIET_S):
    """Yield the port's lines. Says so on `out` when nothing arrives for
    `quiet` seconds (and again every `quiet` s), when bytes arrive but no
    line end (a wrong baud rate or a broken line looks like that), and when
    bytes come back after a silence, so a silent port is never a silent
    screen."""
    buf = b""
    last_byte = last_line = clock()     # times of the last byte, the last \n
    warned_at = None                    # time of the last silence report
    nbytes = 0                          # bytes since the last \n
    silent = False                      # a "no bytes" report is standing
    while deadline is None or clock() < deadline:
        chunk = port.read(256)
        now = clock()
        stamp = dt.datetime.fromtimestamp(now).strftime("%H:%M:%S")
        if chunk:
            if silent:
                print(f"{stamp} bytes again after {now - last_byte:.0f} s", file=out)
                silent, warned_at = False, None
            last_byte = now
            nbytes += len(chunk)
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                last_line = now
                nbytes = len(buf)
                warned_at = None
                yield line.decode("ascii", errors="replace") + "\n"
        if warned_at is not None and now - warned_at < quiet:
            continue
        if now - last_byte >= quiet:
            name = getattr(port, "port", None) or "the port"
            print(f"{stamp} no bytes for {now - last_byte:.0f} s on {name} "
                  "(FPGA programmed and sending? companion-console.md §2)", file=out)
            silent, warned_at = True, now
        elif nbytes and now - last_line >= quiet:
            print(f"{stamp} {nbytes} bytes but no line end for {now - last_line:.0f} s "
                  f"(baud rate not {BAUD_RATE}, or a broken line?)", file=out)
            warned_at = now


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--file", help="parse a saved capture instead of a port")
    ap.add_argument("--duration", type=float, help="seconds, default: until Ctrl-C")
    ap.add_argument("--no-csv", action="store_true")
    args = ap.parse_args(argv)

    logger = None
    if not args.no_csv:
        bench = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bench")
        os.makedirs(bench, exist_ok=True)
        name = dt.datetime.now().strftime("companion_%Y%m%d_%H%M%S.csv")
        logger = Logger(os.path.join(bench, name))
        print(f"Logging to {logger.path}")

    stats = {"good": 0, "bad": 0}
    try:
        if args.file:
            with open(args.file, newline="") as f:
                process(f, logger, stats=stats)
        else:
            import serial
            try:
                port = serial.Serial(args.port, baudrate=BAUD_RATE, timeout=0.5)
            except serial.SerialException as e:
                print(f"ERROR: {e}", file=sys.stderr)
                return 1
            print(f"Reading {args.port} at {BAUD_RATE} baud (Ctrl-C to stop)")
            deadline = time.time() + args.duration if args.duration else None
            try:
                process(_serial_lines(port, deadline), logger, stats=stats)
            finally:
                port.close()
    except KeyboardInterrupt:
        pass
    finally:
        if logger:
            logger.close()
    print(f"{stats['good']} good line(s), {stats['bad']} bad")
    return 0 if stats["good"] and not stats["bad"] else 1


if __name__ == "__main__":
    sys.exit(main())
