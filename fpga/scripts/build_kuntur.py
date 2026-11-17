#!/usr/bin/env python3
"""Build Kuntur's Radiant implementations from the command line and check
each one: the variant define reached synthesis, timing closed, and nothing
new turned up in the synthesis warnings.

The implementations (impl_28k, impl_25k, impl_25k_bench) share the RTL and
the .pdc; only the variant define and the per-rate .sdc differ (task #6,
vega log/2026-11-04.md). A change that closes at 25k can still fail at
28k, the tightest, so build all of them before a push.

Runs radiantc (Radiant's Tcl shell, no GUI) on kuntur_fpga.rdf through
Export (bitstream). It does not save the project. Save your work in the
Radiant GUI first, and do not build in the GUI while this runs: both
write the same impl_* folders.

Per implementation it reports:
  - define     VERILOG_DIRECTIVES from the .rdf, and whether the
               synthesis project (.synproj) received it as -directive
  - size       register bits, PLLs, the clock nets and their loads
  - timing     errors per corner (.twr 1.3), the worst endpoints,
               constraint coverage
  - warnings   synthesis warnings from our sources; Lattice library
               noise and the known ones (KNOWN_WARNINGS) are counted, not
               listed. "stuck at" (VDB-5040, constant carry bits) is
               counted on its own: compare with the last build, a jump is not normal
  - applied    each REQUIRED_EXCEPTIONS constraint shows up as a Path
               Exception in the .twr (map drops a .pdc constraint whose
               object was renamed with only a map warning)
  - bitstream  .bit written by this run
It also checks that kuntur_25k.sdc and kuntur_28k.sdc differ only in
their header and RATE block.

Usage, from anywhere:
  build_kuntur.py                       build all implementations except impl_1
  build_kuntur.py impl_28k impl_25k     build these
  build_kuntur.py --report-only         check the last builds, build nothing
  build_kuntur.py --project DIR         a copy of the project instead

Exit status: 0 every build ran, carries its define and has 0 timing
errors (and the .sdc pair is consistent); 1 otherwise; 2 Radiant or the
project not found.
"""

import argparse
import glob
import os
import re
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.normpath(os.path.join(HERE, '..', 'headstage'))
RETIRED = {'impl_1'}

# Synthesis warnings that are not ours, or are known and harmless.
NOISE = ('cae_library', 'VERI-1209', 'VERI-1214', 'VDB-5040', '70009502',
         '_macro')
# .pdc constraints on post-synthesis names. map drops one whose object no
# longer matches with only a map warning, so the build would look clean.
REQUIRED_EXCEPTIONS = (
    ('O2 clk -> clk_tx MOSI bound (#6a)',
     r'set_max_delay -from \[get_clocks clk\] -to \[get_pins \{?scan0/mosi_tx_c\.ff_inst/DF'),
)

KNOWN_WARNINGS = (
    'shadow[0][7] does not have a driver',  # before #6 too (impl_1)
)


def radiantc():
    """radiantc from $RADIANT_DIR, else the newest ~/lscc/radiant/*."""
    root = os.environ.get('RADIANT_DIR')
    roots = [root] if root else sorted(glob.glob(os.path.expanduser('~/lscc/radiant/*')))
    for r in reversed(roots):
        p = os.path.join(r, 'bin', 'lin64', 'radiantc')
        if os.access(p, os.X_OK):
            return p
    return None


def set_project(proj):
    global PROJ, RDF, SDC_25K, SDC_28K
    PROJ = os.path.abspath(proj)
    RDF = os.path.join(PROJ, 'kuntur_fpga.rdf')
    SDC_25K = os.path.join(PROJ, 'source', 'impl_1', 'kuntur_25k.sdc')
    SDC_28K = os.path.join(PROJ, 'source', 'impl_1', 'kuntur_28k.sdc')


def implementations():
    """{name: VERILOG_DIRECTIVES} from the .rdf, in file order."""
    impls = {}
    for impl in ET.parse(RDF).getroot().iter('Implementation'):
        opts = impl.find('Options')
        impls[impl.get('title')] = opts.get('VERILOG_DIRECTIVES', '') if opts is not None else ''
    return impls


def build(tool, names):
    """Run radiantc through Export for each name; return its exit status."""
    lines = ['prj_open {%s}' % RDF]
    lines += ['prj_run Export -impl %s' % n for n in names]
    lines += ['prj_close']
    with tempfile.TemporaryDirectory(prefix='build_kuntur_') as tmp:
        tcl = os.path.join(tmp, 'build.tcl')
        with open(tcl, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        # radiantc leaves radiantc.tcl.<pid>/.log.<pid> in its working
        # directory: run it in the temporary one.
        r = subprocess.run([tool, tcl], cwd=tmp, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, text=True)
    errors = [l for l in r.stdout.splitlines() if l.startswith('ERROR')]
    for l in errors:
        print('  radiantc: ' + l)
    return r.returncode if r.returncode else (1 if errors else 0)


def read(path):
    try:
        with open(path, errors='replace') as f:
            return f.read()
    except OSError:
        return None


def check(name, directive, since):
    """Print the report for one implementation; return True if it passes."""
    d = os.path.join(PROJ, name)
    base = os.path.join(d, 'kuntur_fpga_' + name)
    ok = True
    print('\n== %s  (%s)' % (name, directive or 'no VERILOG_DIRECTIVES'))

    synproj = read(base + '_lattice.synproj')
    want = '-directive ' + directive
    if not directive:
        print('  define     none set in the .rdf: the build stops at the variant guard')
        ok = False
    elif synproj is None:
        print('  define     no .synproj: not built')
        return False
    elif want in synproj:
        print('  define     %s reached synthesis' % want)
    else:
        print('  define     %s NOT in the .synproj' % want)
        ok = False

    srp = read(base + '_lattice.srp')
    if srp is None:
        print('  size       no .srp')
        return False
    regs = re.search(r'Number of register bits => (\d+)', srp)
    plls = re.search(r'^PLL => (\d+)', srp, re.M)
    print('  size       %s register bits, %s PLL' % (regs.group(1) if regs else '?',
                                                    plls.group(1) if plls else '0'))
    clocks = re.search(r'Clock Nets\nNumber of Clocks: \d+\n((?:  Net : .*\n)+)', srp)
    for l in (clocks.group(1).splitlines() if clocks else []):
        net, loads = re.match(r'\s+Net : (\S+), loads : (\d+)', l).groups()
        print('             %-6s loads  %s' % (loads, net.split('/')[-1] if '/' in net else net))

    warns = [l for l in srp.splitlines() if l.startswith(('WARNING', 'ERROR'))]
    ours = [l for l in warns if not any(n in l for n in NOISE)]
    known = [l for l in ours if any(k in l for k in KNOWN_WARNINGS)]
    new = [l for l in ours if l not in known]
    stuck = sum('VDB-5040' in l for l in warns)
    print('  warnings   %d new, %d known, %d library/noise (%d of them "stuck at", VDB-5040)'
          % (len(new), len(known), len(warns) - len(ours), stuck))
    for l in sorted(set(new)):
        print('             ' + re.sub(r'\S*/headstage/', '', l)[:150])
    if any(l.startswith('ERROR') for l in warns):
        ok = False

    twr = read(base + '.twr')
    if twr is None:
        print('  timing     no .twr')
        return False
    corners = re.findall(r'^ (Setup|Hold) at .*?Corner at (\S+ Degrees)\s+Timing Errors: (\d+) endpoints;'
                         r'\s+Total Negative Slack: (\S+) ns', twr, re.M)
    for kind, corner, n, tns in corners:
        print('  timing     %-5s %-11s %s errors, TNS %s ns' % (kind, corner, n, tns))
        if n != '0':
            ok = False
    if not corners:
        print('  timing     no summary in the .twr')
        ok = False
    ends = re.search(r'2\.2  Endpoint slacks\n=+\n-+\n.*?\n-+\n(.*?)\n\n', twr, re.S)
    if ends:
        rows = re.findall(r'^(\S[^|]*?)\s*\n?\s*\|\s+(-?[\d.]+) ns', ends.group(1), re.M)
        for ep, slack in rows[:3]:
            print('             %8s ns  %s' % (slack, ep.strip()))
    cov = re.search(r'Constraint Coverage: (\S+)', twr)
    if cov:
        print('             coverage %s' % cov.group(1))
    # Constraints that map can silently drop (a map warning only): each
    # must appear as a Path Exception in the .twr.
    for name, pat in REQUIRED_EXCEPTIONS:
        if re.search(r'^Path Exception\s*:\s*' + pat, twr, re.M):
            print('  applied    %s' % name)
        else:
            print('  MISSING    %s: no Path Exception in the .twr (renamed cell? see impl_1.pdc)' % name)
            ok = False

    bit = base + '.bit'
    if os.path.exists(bit) and os.path.getmtime(bit) >= since:
        print('  bitstream  %s' % os.path.relpath(bit, PROJ))
    elif os.path.exists(bit):
        print('  bitstream  %s is older than this run' % os.path.relpath(bit, PROJ))
        ok = False if since else ok
    else:
        print('  bitstream  none')
        ok = False
    return ok


def sdc_body(path):
    """The .sdc minus its header (up to the second '# ----' rule) and RATE block."""
    text = read(path)
    if text is None:
        return None
    lines = text.splitlines()
    rules = [i for i, l in enumerate(lines) if l.startswith('# ----')]
    lines = lines[rules[1] + 1:] if len(rules) > 1 else lines
    out, skip = [], False
    for l in lines:
        if l.startswith('# ---- RATE block'):
            skip = True
        elif l.startswith('# ---- end of RATE block'):
            skip = False
        elif not skip:
            out.append(l)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('impl', nargs='*', help='implementations (default: all except impl_1)')
    ap.add_argument('--report-only', action='store_true', help='check the last builds, build nothing')
    ap.add_argument('--project', default=PROJ, help='project folder (default: %(default)s)')
    a = ap.parse_args()
    set_project(a.project)

    if not os.path.exists(RDF):
        print('no project at %s' % RDF, file=sys.stderr)
        return 2
    impls = implementations()
    names = a.impl or [n for n in impls if n not in RETIRED]
    unknown = [n for n in names if n not in impls]
    if unknown:
        print('not in the .rdf: %s (have %s)' % (', '.join(unknown), ', '.join(impls)), file=sys.stderr)
        return 2

    since = 0
    if not a.report_only:
        tool = radiantc()
        if tool is None:
            print('radiantc not found; set RADIANT_DIR', file=sys.stderr)
            return 2
        print('building %s with %s' % (', '.join(names), tool))
        since = time.time()
        build(tool, names)
        print('done in %.0f s' % (time.time() - since))

    ok = all([check(n, impls[n], since) for n in names])

    b25, b28 = sdc_body(SDC_25K), sdc_body(SDC_28K)
    if b25 is not None and b28 is not None:
        same = b25 == b28
        print('\n.sdc pair   %s' % ('kuntur_25k.sdc and kuntur_28k.sdc differ only in header and RATE block'
                                    if same else 'kuntur_25k.sdc and kuntur_28k.sdc DIFFER outside the RATE block'))
        ok = ok and same

    print('\n%s' % ('PASS' if ok else 'FAIL'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
