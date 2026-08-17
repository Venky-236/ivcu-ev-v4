#!/usr/bin/env python3
"""
filter_liberty.py  -  remove cells that must not be used for general logic

WHY THIS EXISTS

The first V4 STA run showed this on the critical path into the register map:

    9.967   18.701 ^ _43110_/X (sky130_fd_sc_hd__lpflow_isobufsrc_1)

Ten nanoseconds through one cell, on a 20 ns clock.  The cell is not broken -
it is a low-power isolation buffer, meant to sit on a signal crossing out of a
power-gated domain and hold it at a defined level while that domain is off.
It is deliberately slow and deliberately weak.

ABC does not know that.  It sees a cell in the liberty with an attractive
area, and it uses it.  Every open-source Sky130 flow that produces sensible
timing pre-filters these out; OpenLane ships a dont_use list and this is the
same list.

WHAT GETS REMOVED

  lpflow_*        power-gating isolation and retention.  Slow on purpose.
  probe_p_*       manufacturing probe cells.  Not functional logic.
  probec_p_*      same.
  dlclkp_*        latch-based clock gates.  Not used; this design has no
                  clock gating in RTL and CTS should choose its own.
  dlymetal6s6s_*  metal-only delay cells.  Only meaningful post-layout.
  dlygate4sd*     delay cells.  Same.
  clkdlybuf4s*    clock delay buffers.  CTS territory, not synthesis.
  sdf*, sdl*      scan flops and scan latches.  Scan is inserted after
                  synthesis, and letting ABC pick scan cells now would give
                  us scan-capable flops with nothing stitched to them - the
                  V3 DFT-pins-with-no-chains mistake in a new costume.

USAGE
    python3 scripts_v4/filter_liberty.py \\
        libs/sky130_fd_sc_hd__tt_025C_1v80.lib \\
        libs/sky130_fd_sc_hd__tt_025C_1v80__filtered.lib
"""

import re
import sys

DONT_USE = [
    r"lpflow_",
    r"probe_p_",
    r"probec_p_",
    r"dlclkp_",
    r"dlymetal6s6s",
    r"dlygate4sd",
    r"clkdlybuf4s",
    r"^sky130_fd_sc_hd__sdf",
    r"^sky130_fd_sc_hd__sdl",
]

PAT = [re.compile(p) for p in DONT_USE]


def blocked(name):
    return any(p.search(name) for p in PAT)


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 1

    src, dst = sys.argv[1], sys.argv[2]

    with open(src, "r", errors="replace") as fh:
        text = fh.read()

    out = []
    i = 0
    n = len(text)
    removed = []
    kept = 0

    cell_re = re.compile(r'\bcell\s*\(\s*"?([A-Za-z0-9_]+)"?\s*\)\s*\{')

    while i < n:
        m = cell_re.search(text, i)
        if not m:
            out.append(text[i:])
            break

        # everything before this cell block is copied verbatim
        out.append(text[i:m.start()])

        name = m.group(1)

        # walk to the matching close brace
        depth = 0
        j = m.end() - 1          # position of the opening '{'
        while j < n:
            c = text[j]
            if c == "{":
                depth += 1
            elif c == "}":
                depth -= 1
                if depth == 0:
                    j += 1
                    break
            j += 1

        block = text[m.start():j]

        if blocked(name):
            removed.append(name)
        else:
            out.append(block)
            kept += 1

        i = j

    with open(dst, "w") as fh:
        fh.write("".join(out))

    print("=" * 66)
    print("liberty filter")
    print("  source : %s" % src)
    print("  output : %s" % dst)
    print("=" * 66)
    print("  cells kept    : %d" % kept)
    print("  cells removed : %d" % len(removed))
    print()

    # group the removals so the output is readable rather than a wall of names
    groups = {}
    for r in removed:
        for p in DONT_USE:
            key = p.strip("^").replace(r"sky130_fd_sc_hd__", "")
            if re.search(p, r):
                groups.setdefault(key, []).append(r)
                break
    for k in sorted(groups):
        print("    %-16s %3d" % (k, len(groups[k])))
    print()

    if kept < 100:
        print("  *** only %d cells survived - that is too few." % kept)
        print("  *** Check the patterns before using this liberty.")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
