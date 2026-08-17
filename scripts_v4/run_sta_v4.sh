#!/usr/bin/env bash
#=============================================================================
# run_sta_v4.sh  -  wrapper for OpenSTA on the newest V4 netlist
#
# USAGE
#   bash scripts_v4/run_sta_v4.sh                    # newest netlist
#   bash scripts_v4/run_sta_v4.sh synth_out_v4/x.v   # a specific one
#=============================================================================

set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${HERE}/.." && pwd)"
TOP="ivcu_ev_v4_top"
OUT="${ROOT}/sta_out_v4"
mkdir -p "${OUT}"

cd "${ROOT}" || exit 1

NETLIST="${1:-}"
if [ -z "${NETLIST}" ]; then
  NETLIST=$(ls -t "${ROOT}/synth_out_v4"/${TOP}_netlist_*.v 2>/dev/null | head -1)
fi
if [ -z "${NETLIST}" ] || [ ! -f "${NETLIST}" ]; then
  echo "  no netlist found - run scripts_v4/run_synth_v4.sh first"
  exit 1
fi

if ! command -v sta >/dev/null 2>&1; then
  echo "  OpenSTA not on PATH."
  echo "  It lives in the micromamba 'eda' environment:"
  echo "      micromamba activate eda"
  exit 1
fi

SRAM_LIB="${ROOT}/macros/sram_512x32_2port_TT_1p8V_25C.lib"
if [ ! -f "${SRAM_LIB}" ]; then
  echo "  *** SRAM liberty missing: ${SRAM_LIB}"
  echo "  *** Without it every path through the fault log macro is silently"
  echo "  *** unconstrained, and STA will report clean timing for a block it"
  echo "  *** never looked at."
  exit 1
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
LOGFILE="${OUT}/sta_${STAMP}.txt"

echo "=============================================================="
echo " IVCU-EV V4 static timing analysis"
echo " netlist : $(basename "${NETLIST}")"
echo " sdc     : SDC/ivcu_ev_v4.sdc"
echo "=============================================================="
echo

NETLIST="${NETLIST}" sta -no_splash -exit "${HERE}/run_sta_v4.tcl" \
    2>&1 | tee "${LOGFILE}"

echo
echo "--------------------------------------------------------------"
echo " HEADLINE NUMBERS"
echo "--------------------------------------------------------------"
grep -E "^(wns|tns|worst slack)" "${LOGFILE}" | sed 's/^/  /'
echo
echo "  V3 baseline:  WNS -1.389 ns   TNS -48.323 ns"
echo "                (against 200 MHz MCU / 100 MHz AI)"
echo "  V4 targets:   50 / 50 / 25 / 10 MHz"
echo
echo "  full report: ${LOGFILE}"
echo
