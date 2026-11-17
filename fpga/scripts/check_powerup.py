#!/usr/bin/env python3
"""Check the power-up value of every flip-flop of one module, in a Radiant
post-synthesis netlist (impl_1/<project>_impl_1.vm).

A Nexus flop wakes up at its REGSET value, which the primitive fixes:
FD1P3DX / FD1P3IX (clear) wake up 0, FD1P3BX / FD1P3JX (preset) wake up 1
(Radiant cae_library/simulation/verilog/uaplatform/FD1P3*.v). The wake-up
GSR sets them only if GSR = "ENABLED". An `initial` value in the RTL plays
no part. Contract: vega docs/interfaces/system-startup.md §2.3 R-C8
(power-up values) and §1.3 R-K2 (the REGSET check).

Every flop in MODULE, and in every module it instantiates, must wake up 0
with GSR enabled, except the ones whose instance name matches --one, which
must wake up 1. Cells that are neither flops nor plain logic (PLLs, DDR,
clock primitives) are listed as not checked.

Usage:
  check_powerup.py NETLIST.vm MODULE [--one REGEX ...]
e.g. from fpga/:
  scripts/check_powerup.py rhd-adapter/impl_1/intan_rec_adapter_impl_1.vm reset_gen_adapter

Exit status: 0 all good, 1 a flop is wrong, 2 MODULE not in the netlist
(synthesis dissolved the hierarchy, or the build predates the module).
"""

import argparse
import os
import re
import sys

FLOP = re.compile(r'^(?:I|O)?FD1P3([BDIJ])X$')
WAKES_UP = {'D': 0, 'I': 0, 'B': 1, 'J': 1}
LOGIC = {'LUT4', 'CCU2', 'WIDEFN9', 'INV', 'VHI', 'VLO', 'IB', 'OB', 'BB'}


def ident(s, i):
    """Read a Verilog identifier at s[i:]; escaped ones run to a space."""
    if s[i] == '\\':
        j = s.index(' ', i)
        return s[i:j], j
    m = re.compile(r'[A-Za-z_][A-Za-z0-9_$]*').match(s, i)
    return (m.group(0), m.end()) if m else (None, i)


def parse(path):
    """Return {module: (cells, gsr)} with cells = [(type, name)] and
    gsr = {instance: value} from defparams, plus the source files list."""
    text = open(path).read()
    files = re.findall(r'^// file \d+ "([^"]+)"', text, re.M)
    text = re.sub(r'//[^\n]*', '', text)
    text = re.sub(r'\(\*.*?\*\)', ' ', text, flags=re.S)
    mods = {}
    for m in re.finditer(r'\bmodule\s+(.*?)\bendmodule\b', text, re.S):
        body = m.group(1)
        name, i = ident(body, 0)
        # the header runs to the first ';' outside the port list
        depth, j = 0, i
        while j < len(body):
            c = body[j]
            depth += (c == '(') - (c == ')')
            if c == ';' and depth == 0:
                break
            j += 1
        cells, gsr = [], {}
        for st in body[j + 1:].split(';'):
            st = st.strip()
            if not st:
                continue
            d = re.match(r'defparam\s+(\S+?)\.GSR\s*=\s*"(\w+)"', st)
            if d:
                gsr[d.group(1)] = d.group(2)
            if st.startswith('defparam'):
                continue
            if re.match(r'(wire|input|output|inout|assign|reg)\b', st):
                continue
            t, k = ident(st, 0)
            if not t:
                continue
            while k < len(st) and st[k] in ' \t\n':
                k += 1
            if k < len(st) and st[k] == '#':
                continue
            n, _ = ident(st, k) if k < len(st) else (None, k)
            if n:
                cells.append((t, n))
        mods[name] = (cells, gsr)
    return mods, files


def find(mods, want):
    """The netlist name of MODULE: plain, or \\MODULE(PARAM=...) when LSE
    uniquifies a parameterised one."""
    return [m for m in mods if re.match(r'\\?' + re.escape(want) + r'(\(|$)', m)]


def walk(mods, top, path, out, skipped):
    cells, gsr = mods[top]
    for t, n in cells:
        where = path + '/' + n
        f = FLOP.match(t)
        if f:
            out.append((where, t, WAKES_UP[f.group(1)], gsr.get(n, 'ENABLED')))
        elif t in mods:
            walk(mods, t, where, out, skipped)
        elif t not in LOGIC:
            skipped.append((where, t))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('netlist')
    ap.add_argument('module')
    ap.add_argument('--one', action='append', default=[], metavar='REGEX',
                    help='instance names that must wake up 1 (repeatable)')
    a = ap.parse_args()

    mods, files = parse(a.netlist)
    hits = find(mods, a.module)
    if not hits:
        print(f'{a.module}: not in {a.netlist}.')
        print('  Either the build predates it, or synthesis dissolved its hierarchy')
        print('  (keep it with /* synthesis syn_hier = "hard" */ on the module).')
        return 2

    t_vm = os.path.getmtime(a.netlist)
    for f in files:
        if os.path.exists(f) and os.path.getmtime(f) > t_vm and not f.startswith('/home'):
            print(f'warning: {f} is newer than the netlist; rebuild before trusting this.')

    ones = [re.compile(r) for r in a.one]
    bad = 0
    for h in hits:
        flops, skipped = [], []
        walk(mods, h, h.split('(')[0].lstrip('\\'), flops, skipped)
        print(f'{h.split("(")[0]}: {len(flops)} flops')
        for where, t, v, g in flops:
            inst = where.rsplit('/', 1)[1]
            want = 1 if any(r.search(inst) for r in ones) else 0
            ok = v == want and g == 'ENABLED'
            bad += not ok
            if not ok or a.one:
                why = [] if ok else ([f'wakes up {v}, want {want}'] if v != want else []) + \
                                    ([f'GSR = "{g}"'] if g != 'ENABLED' else [])
                print(f'  {"ok  " if ok else "FAIL"} {where:<48} {t}  {"; ".join(why)}')
        for where, t in skipped:
            print(f'  not checked  {where} ({t})')
        if not a.one and not bad:
            print(f'  all wake up 0, GSR enabled')
    print('PASS' if not bad else f'FAIL ({bad})')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
