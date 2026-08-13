#!/usr/bin/env bash
#=============================================================================
# verify_v4.sh  -  the gate suite for IVCU-EV V4
#
# RUN THIS AFTER EVERY SINGLE EDIT.  Not once at the end.
#
# Your own note from the V3 rerun says it: if you batch ten changes and the
# whole-design elaboration fails, you do not know which one did it.  This suite
# takes well under a minute.  A mistake costs you that, instead of five hours
# of synthesis followed by a netlist you have to throw away.
#
# GATES
#   1  iverilog, whole design                    (syntax + hierarchy resolves)
#   2  Yosys, each module as top                 (multiple drivers, widths)
#   3  Yosys, whole design + check -assert       (top-level wiring)
#   4  structural lint  R1 R2 R3 R5 R8 SV SIM    (scripts_v4/lint_v4.py)
#   5  constant table check                      (scripts_v4/check_defs.py)
#
# Gates 1 and 2 were corrected on 13 Aug 2026.  Both originally compiled one
# file at a time, which is impossible for a hierarchical design: eight wrapper
# modules reported "Unknown module type" for children that live in other
# files.  Those were failures of the gate, not of the RTL - the whole-design
# gate 3 passed at the same time.  A per-file check only ever worked for leaf
# modules, and half of this design is structure.
#
# Gates 4 and 5 are new in V4.  Each encodes a defect family that survived V3
# synthesis, V3 STA and V3 floorplan without anyone noticing.
#
# USAGE
#   bash scripts_v4/verify_v4.sh            # all gates
#   bash scripts_v4/verify_v4.sh fast       # gates 4 and 5 only, ~1 second
#=============================================================================

set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${HERE}/.." && pwd)"
RTL="${ROOT}/RTL_V4"
TOP="ivcu_ev_v4_top"
LOG="${ROOT}/qc_v4"
mkdir -p "${LOG}"

MODE="${1:-full}"
FAIL=0
PASS_N=0
FAIL_N=0

c_red()   { printf '\033[31m%s\033[0m\n' "$1"; }
c_green() { printf '\033[32m%s\033[0m\n' "$1"; }
c_bold()  { printf '\033[1m%s\033[0m\n'  "$1"; }

report() {           # report <name> <rc>
  if [ "$2" -eq 0 ]; then
    c_green "  PASS  $1"; PASS_N=$((PASS_N+1))
  else
    c_red   "  FAIL  $1"; FAIL_N=$((FAIL_N+1)); FAIL=1
  fi
}

echo
c_bold "=============================================================="
c_bold " IVCU-EV V4 verification gates"
c_bold " rtl : ${RTL}"
c_bold " log : ${LOG}"
c_bold "=============================================================="

FILES=$(ls "${RTL}"/*.v 2>/dev/null || true)
if [ -z "${FILES}" ]; then
  c_red "no .v files in ${RTL} - nothing to check"
  exit 1
fi
N_FILES=$(echo "${FILES}" | wc -l | tr -d ' ')
echo "  ${N_FILES} verilog file(s)"
echo

#-----------------------------------------------------------------------------
# GATE 5 - constant tables  (cheap, run first: if the roster is wrong,
#                            everything downstream is wrong)
#-----------------------------------------------------------------------------
c_bold "GATE 5  constant table check"
python3 "${HERE}/check_defs.py" > "${LOG}/check_defs.log" 2>&1
RC=$?
[ ${RC} -ne 0 ] && cat "${LOG}/check_defs.log"
report "constant tables (see qc_v4/check_defs.log)" ${RC}
echo

#-----------------------------------------------------------------------------
# GATE 4 - structural lint
#-----------------------------------------------------------------------------
c_bold "GATE 4  structural lint  R1 R2 R3 R5 R8 SV SIM"
python3 "${HERE}/lint_v4.py" "${RTL}" > "${LOG}/lint.log" 2>&1
RC=$?
[ ${RC} -ne 0 ] && cat "${LOG}/lint.log"
report "structural lint (see qc_v4/lint.log)" ${RC}
echo

if [ "${MODE}" = "fast" ]; then
  echo
  c_bold "--------------------------------------------------------------"
  echo "  fast mode: gates 1-3 skipped"
  c_bold "  ${PASS_N} passed, ${FAIL_N} failed"
  c_bold "--------------------------------------------------------------"
  exit ${FAIL}
fi

#-----------------------------------------------------------------------------
# GATE 1 - iverilog: syntax and elaboration, WHOLE DESIGN
#
# CORRECTED 13 Aug 2026.  This gate used to compile each file on its own, and
# every wrapper module failed with "Unknown module type" - ivcu_reset_manager
# cannot elaborate without ivcu_reset_sync, and so on for eight wrappers.
#
# Those were not defects in the RTL.  They were the gate asking a hierarchical
# design to be flat.  A per-file check only works for leaf modules, and half
# this design is structure.
#
# iverilog parses every file it is given, so one whole-design invocation still
# catches a syntax error anywhere - and it additionally proves the hierarchy
# resolves, which the per-file version could never do.
#-----------------------------------------------------------------------------
c_bold "GATE 1  iverilog syntax and elaboration (whole design)"
if command -v iverilog >/dev/null 2>&1; then
  iverilog -g2005 -t null -I "${RTL}" \
      "${RTL}"/*.v "${ROOT}/macros/sram_512x32_2port.v" \
      > "${LOG}/iverilog.log" 2>&1
  RC=$?
  [ ${RC} -ne 0 ] && tail -60 "${LOG}/iverilog.log"
  report "iverilog -g2005, ${N_FILES} file(s) + SRAM model" ${RC}
else
  c_red "  SKIP  iverilog not on PATH"
fi
echo

#-----------------------------------------------------------------------------
# GATE 2 - Yosys: elaborate each module AS A TOP, with the full design loaded
#
# CORRECTED 13 Aug 2026, same reason as gate 1.
#
# The point of a per-module gate is to catch multiple drivers, width mismatches
# and undriven logic inside one module without the surrounding design hiding
# them.  That does not require compiling the file alone - it requires setting
# that module as the top so nothing above it is elaborated.
#
# So: read every file once per iteration, then hierarchy -check -top <module>.
# Leaf modules get checked in isolation exactly as before; wrappers now get
# checked with their children present, which is the only way they can be.
#-----------------------------------------------------------------------------
c_bold "GATE 2  yosys elaborate each module as top"
if command -v yosys >/dev/null 2>&1; then
  : > "${LOG}/elab_each.log"
  G2=0
  MODULES=$(grep -h '^[[:space:]]*module[[:space:]]' "${RTL}"/*.v \
            | sed -E 's/^[[:space:]]*module[[:space:]]+([A-Za-z_0-9]+).*/\1/' \
            | sort -u)
  N_MOD=$(echo "${MODULES}" | wc -l | tr -d ' ')
  for m in ${MODULES}; do
    echo "=== ${m} ===" >> "${LOG}/elab_each.log"
    yosys -q -p "
      read_verilog -I ${RTL} ${RTL}/*.v;
      read_verilog -lib ${ROOT}/macros/sram_512x32_2port.v;
      hierarchy -check -top ${m};
      proc; opt_clean; check" \
      >> "${LOG}/elab_each.log" 2>&1 \
      || { echo "  elaboration failed: ${m}" | tee -a "${LOG}/elab_each.log"; G2=1; }
  done
  [ ${G2} -ne 0 ] && grep -B2 -A6 "^ERROR" "${LOG}/elab_each.log" | tail -60
  report "per-module elaboration (${N_MOD} modules)" ${G2}
else
  c_red "  SKIP  yosys not on PATH"
fi
echo

#-----------------------------------------------------------------------------
# GATE 3 - whole design elaboration
#-----------------------------------------------------------------------------
c_bold "GATE 3  yosys elaborate whole design (top = ${TOP})"
if command -v yosys >/dev/null 2>&1; then
  if grep -qs "module[[:space:]]\+${TOP}\b" "${RTL}"/*.v; then
    # NOTE: no -q here.  Quiet mode suppresses the whole log including the
    # stat output, which is the most useful thing this gate produces - it is
    # the first honest look at whether anything is unexpectedly large, before
    # committing to a multi-hour synthesis run.
    yosys -p "
      read_verilog -I ${RTL} ${RTL}/*.v;
      read_verilog -lib ${ROOT}/macros/sram_512x32_2port.v;
      hierarchy -check -top ${TOP};
      proc; opt_clean; check -assert; stat" \
      > "${LOG}/elab_top.log" 2>&1
    RC=$?
    [ ${RC} -ne 0 ] && tail -60 "${LOG}/elab_top.log"
    report "whole-design elaboration (see qc_v4/elab_top.log)" ${RC}
  else
    echo "  SKIP  ${TOP} not written yet - batches still in progress"
  fi
else
  c_red "  SKIP  yosys not on PATH"
fi

echo
c_bold "--------------------------------------------------------------"
if [ ${FAIL} -eq 0 ]; then
  c_green "  ALL GATES PASS   (${PASS_N} passed)"
else
  c_red   "  ${FAIL_N} GATE(S) FAILED   (${PASS_N} passed)"
fi
c_bold "--------------------------------------------------------------"
echo
exit ${FAIL}
