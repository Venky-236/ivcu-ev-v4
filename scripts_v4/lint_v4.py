#!/usr/bin/env python3
"""
lint_v4.py  -  structural lint for IVCU-EV V4 RTL.

Each rule here encodes a defect family that cost a real synthesis run in V3.
This is not style checking.  Every one of these found something measurable:

  R1  a constant passed through a module port
          V3 did this ten times.  Synthesis cannot see the value, so it builds
          a general comparator or multiplier instead of folding the constant.
          Cost: 575,000 um2 in sensor_interface_fabric_complete alone, plus
          most of a 105,193 um2 grace manager and a 62,111 um2 ADC block.

  R2  a module output left unconnected at instantiation
          V3's u_diagnostic had 386 output bits and not one was connected.
          Verilog allows silent port omission, so nothing complained.
          Cost: 118,908 um2, 18.8% of the standard cell area.

  R3  a signal that is written and never read
          V3's raw_array had 42 writes and 0 reads, which in turn made 384
          top-level pins dead.

  R5  a '/' or '%' operator
          The source of the STA divider problem.  seq_divider.v is deleted from
          V4 and no division is permitted anywhere.

  R8  a process node in a module or file name
          clock_manager_14nm.v and adc_interface_14nm.v in a 130 nm project.

  SV  a SystemVerilog-only construct in a .v file
          V4 is Verilog-2001 throughout, one language mode, no -sv flag.

  SIM a simulation-only construct in synthesizable RTL
          initial blocks, #delays, $display outside `ifndef SYNTHESIS.

USAGE
    python3 scripts_v4/lint_v4.py [rtl_dir]

    Default rtl_dir is RTL_V4 next to this script's parent.
    Exit 0 = clean.  Exit 1 = at least one violation.
"""

import os
import re
import sys

# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def strip_comments(text):
    """Blank out // and /* */ comments, preserving line structure."""
    out = []
    i, n = 0, len(text)
    while i < n:
        if text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i))
            i = j
        elif text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append("".join(c if c == "\n" else " " for c in text[i:j]))
            i = j
        elif text[i] == '"':
            j = i + 1
            while j < n and text[j] != '"':
                j += 2 if text[j] == "\\" else 1
            j = min(j + 1, n)
            out.append("".join(c if c == "\n" else " " for c in text[i:j]))
            i = j
        else:
            out.append(text[i])
            i += 1
    return "".join(out)


def line_of(text, pos):
    return text.count("\n", 0, pos) + 1


SIZED_LIT = re.compile(r"^\s*\d*\s*'\s*[sS]?[bodhBODH][0-9a-fA-FxXzZ_]+\s*$")
PLAIN_NUM = re.compile(r"^\s*\d+\s*$")


class Module(object):
    def __init__(self, name, file, line):
        self.name = name
        self.file = file
        self.line = line
        self.ports = []      # (name, direction)
        self.body = ""
        self.body_off = 0


MODULE_RE = re.compile(r"\bmodule\s+(\w+)")
PORT_DECL_RE = re.compile(
    r"\b(input|output|inout)\b\s*(?:wire|reg|logic)?\s*"
    r"(?:signed\s*)?(?:\[[^\]]*\]\s*)?([\w\s,]+)")


def parse_modules(path):
    """Return a list of Module objects found in one file."""
    raw = open(path, "r", encoding="utf-8", errors="replace").read()
    text = strip_comments(raw)
    mods = []
    for m in MODULE_RE.finditer(text):
        name = m.group(1)
        end = text.find("endmodule", m.end())
        end = len(text) if end < 0 else end
        mod = Module(name, path, line_of(text, m.start()))
        mod.body = text[m.end():end]
        mod.body_off = m.end()
        # header = everything up to the first ';' after the module keyword
        semi = text.find(";", m.end())
        header = text[m.end():semi if semi > 0 else end]
        for pd in PORT_DECL_RE.finditer(header):
            direction = pd.group(1)
            for nm in pd.group(2).split(","):
                nm = nm.strip()
                if nm and re.match(r"^\w+$", nm) and nm not in (
                        "wire", "reg", "logic", "signed"):
                    mod.ports.append((nm, direction))
        # ANSI-less style: port directions declared in the body
        if not mod.ports:
            for pd in PORT_DECL_RE.finditer(mod.body):
                direction = pd.group(1)
                for nm in pd.group(2).split(","):
                    nm = nm.strip()
                    if nm and re.match(r"^\w+$", nm):
                        mod.ports.append((nm, direction))
        mods.append(mod)
    return mods, text


# instantiation:  MODNAME  [#( ... )]  instname ( .port(sig), ... );
INST_RE = re.compile(
    r"\b(\w+)\s*(#\s*\((?:[^()]|\([^()]*\))*\)\s*)?(\w+)\s*\(", re.S)

KEYWORDS = set("""module endmodule input output inout wire reg logic parameter
localparam always assign begin end if else case endcase casez casex for while
generate endgenerate genvar function endfunction task endtask initial posedge
negedge or and not xor nand nor xnor buf integer real time signed unsigned
default repeat forever disable wait fork join specify endspecify table endtable
primitive endprimitive defparam""".split())


def balanced(text, open_pos):
    """Return (inner_text, end_pos) for the paren starting at open_pos."""
    depth = 0
    i = open_pos
    while i < len(text):
        if text[i] == "(":
            depth += 1
        elif text[i] == ")":
            depth -= 1
            if depth == 0:
                return text[open_pos + 1:i], i
        i += 1
    return text[open_pos + 1:], len(text)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    rtl = sys.argv[1] if len(sys.argv) > 1 else os.path.normpath(
        os.path.join(here, "..", "RTL_V4"))

    if not os.path.isdir(rtl):
        print("lint: no such directory: %s" % rtl)
        return 1

    files = sorted(
        os.path.join(rtl, f) for f in os.listdir(rtl)
        if f.endswith(".v") or f.endswith(".vh"))
    if not files:
        print("lint: no .v files in %s" % rtl)
        return 1

    all_mods = {}
    texts = {}
    for f in files:
        if f.endswith(".vh"):
            texts[f] = strip_comments(
                open(f, "r", encoding="utf-8", errors="replace").read())
            continue
        mods, txt = parse_modules(f)
        texts[f] = txt
        for mo in mods:
            all_mods[mo.name] = mo

    violations = []

    def hit(rule, f, ln, msg):
        violations.append((rule, os.path.basename(f), ln, msg))

    # ---------------- R5 : no division -----------------------------------
    # Exemptions, both of which contain a literal '/' that is not an operator:
    #   `timescale 1ns / 1ps
    #   `include "some/path.vh"
    for f, txt in texts.items():
        lines = txt.split("\n")
        for ln_no, line in enumerate(lines, start=1):
            if "`timescale" in line or "`include" in line:
                continue
            for m in re.finditer(r"[/%]", line):
                hit("R5", f, ln_no,
                    "division/modulo operator at column %d - use shifts or a "
                    "constant reciprocal" % (m.start() + 1))

    # ---------------- R8 : no process node in names ----------------------
    for f in files:
        base = os.path.basename(f)
        if re.search(r"\d+\s*nm", base, re.I):
            hit("R8", f, 1, "process node in file name: %s" % base)
    for name, mo in all_mods.items():
        if re.search(r"\d+\s*nm", name, re.I):
            hit("R8", mo.file, mo.line, "process node in module name: %s" % name)

    # ---------------- SV / SIM constructs --------------------------------
    SV_PAT = [(r"\balways_ff\b", "always_ff"), (r"\balways_comb\b", "always_comb"),
              (r"\balways_latch\b", "always_latch"), (r"\blogic\b", "logic"),
              (r"\btypedef\b", "typedef"), (r"\bpackage\b", "package"),
              (r"\bint\s+\w", "int"), (r"\bbit\s+\w", "bit"),
              (r"\benum\b", "enum"), (r"\bstruct\b", "struct"),
              (r"\bunique\b", "unique"), (r"\bpriority\b", "priority"),
              (r"\.\*", ".* port connection")]
    SIM_PAT = [(r"\binitial\b", "initial block"),
               (r"#\s*\d", "delay control"),
               (r"\$display", "$display"), (r"\$finish", "$finish"),
               (r"\$fatal", "$fatal"), (r"\$random", "$random")]
    for f, txt in texts.items():
        for pat, what in SV_PAT:
            for m in re.finditer(pat, txt):
                hit("SV", f, line_of(txt, m.start()),
                    "SystemVerilog construct '%s' in a Verilog-2001 file" % what)
        for pat, what in SIM_PAT:
            for m in re.finditer(pat, txt):
                # '#(' parameter override is not a delay
                if what == "delay control" and txt[m.start():m.start() + 2] == "#(":
                    continue
                hit("SIM", f, line_of(txt, m.start()),
                    "simulation-only construct '%s' in synthesizable RTL" % what)

    # ---------------- R1 / R2 : instantiation checks ---------------------
    for f in files:
        if f.endswith(".vh"):
            continue
        txt = texts[f]
        for m in INST_RE.finditer(txt):
            mod_name, param_blk, inst_name = m.group(1), m.group(2), m.group(3)
            if mod_name in KEYWORDS or inst_name in KEYWORDS:
                continue
            if mod_name not in all_mods:
                continue                      # external macro or not ours
            open_pos = m.end() - 1
            inner, _end = balanced(txt, open_pos)
            ln = line_of(txt, m.start())

            conns = dict()
            for c in re.finditer(r"\.(\w+)\s*\(", inner):
                pinner, _pe = balanced(inner, c.end() - 1)
                conns[c.group(1)] = pinner.strip()

            target = all_mods[mod_name]

            # R2 - every port present, and no output left empty
            for pname, pdir in target.ports:
                if pname not in conns:
                    hit("R2", f, ln,
                        "%s %s: port .%s (%s) omitted" %
                        (mod_name, inst_name, pname, pdir))
                elif conns[pname] == "" and pdir != "input":
                    hit("R2", f, ln,
                        "%s %s: output .%s left open" %
                        (mod_name, inst_name, pname))

            # R1 - a literal driving a port is a constant that should be a
            #      parameter.  Parameter overrides live in param_blk and are
            #      exempt by construction.
            for pname, val in conns.items():
                if SIZED_LIT.match(val) or PLAIN_NUM.match(val):
                    hit("R1", f, ln,
                        "%s %s: constant %s tied to port .%s - make it a "
                        "parameter or localparam" %
                        (mod_name, inst_name, val.strip(), pname))
            del param_blk

    # ---------------- R3 : declared and never read -----------------------
    for name, mo in all_mods.items():
        body = mo.body
        port_names = set(p for p, _d in mo.ports)
        decl = re.finditer(
            r"\b(?:reg|wire)\b\s*(?:signed\s*)?(?:\[[^\]]*\]\s*)?([\w\s,]+?);", body)
        for d in decl:
            for nm in d.group(1).split(","):
                nm = nm.strip()
                if not re.match(r"^\w+$", nm) or nm in port_names:
                    continue
                uses = len(re.findall(r"\b%s\b" % re.escape(nm), body))
                if uses <= 1:
                    hit("R3", mo.file, mo.line + body[:d.start()].count("\n"),
                        "%s: signal '%s' declared and never used" % (name, nm))
                else:
                    # written but never read: appears only on the left of <= or =
                    rd = re.findall(
                        r"(?<![\w.])%s\b(?!\s*(?:<=|=[^=])|\s*\[[^\]]*\]\s*(?:<=|=[^=]))"
                        % re.escape(nm), body)
                    if len(rd) <= 1:
                        hit("R3", mo.file,
                            mo.line + body[:d.start()].count("\n"),
                            "%s: signal '%s' is written but never read" % (name, nm))

    # ---------------- report ---------------------------------------------
    print("=" * 78)
    print("IVCU-EV V4 structural lint   %d file(s), %d module(s)"
          % (len(files), len(all_mods)))
    print("=" * 78)

    if not violations:
        print("\n  CLEAN - no violations.\n")
        for r in ("R1", "R2", "R3", "R5", "R8", "SV", "SIM"):
            print("    %-4s pass" % r)
        print()
        return 0

    by_rule = {}
    for v in violations:
        by_rule.setdefault(v[0], []).append(v)
    for rule in sorted(by_rule):
        print("\n%s  -  %d violation(s)" % (rule, len(by_rule[rule])))
        for _r, f, ln, msg in sorted(by_rule[rule], key=lambda x: (x[1], x[2])):
            print("    %s:%d  %s" % (f, ln, msg))
    print("\n%d violation(s) total\n" % len(violations))
    return 1


if __name__ == "__main__":
    sys.exit(main())
