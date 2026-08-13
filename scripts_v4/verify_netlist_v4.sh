#!/usr/bin/env bash
#=============================================================================
# verify_netlist_v4.sh  -  check the mapped netlist, standalone
#
# WHY THIS IS A SEPARATE SCRIPT AND NOT PART OF THE SYNTHESIS RUN
#
# Your V3 notes record it directly: running "check -assert -mapped" at the end
# of a synthesis run reported thousands of FALSE problems and renamed the
# netlist REJECTED_*.  Five hours of work, then a scary filename for reasons
# that turned out not to be real.
#
# The verdict that counts is "Found and reported 0 problems" from a clean,
# standalone pass over the written netlist with the liberty and the macro
# stub loaded.  That is what this does.
#
# USAGE
#   bash scripts_v4/verify_netlist_v4.sh synth_out_v4/ivcu_ev_v4_top_netlist_*.v
#=============================================================================

set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${HERE}/.." && pwd)"
TOP="ivcu_ev_v4_top"
LIB="${ROOT}/libs/sky130_fd_sc_hd__tt_025C_1v80.lib"
SRAM_V="${ROOT}/macros/sram_512x32_2port.v"
OUT="${ROOT}/synth_out_v4"

NETLIST="${1:-}"
if [ -z "${NETLIST}" ]; then
  # newest netlist, if none named
  NETLIST=$(ls -t "${OUT}"/${TOP}_netlist_*.v 2>/dev/null | head -1)
fi

if [ -z "${NETLIST}" ] || [ ! -f "${NETLIST}" ]; then
  echo "  no netlist found.  Pass one explicitly:"
  echo "    bash scripts_v4/verify_netlist_v4.sh synth_out_v4/<file>.v"
  exit 1
fi

echo "=============================================================="
echo " netlist verification"
echo " file : $(basename "${NETLIST}")"
echo "=============================================================="
echo

LOGFILE="${OUT}/verify_netlist_$(date +%H%M%S).log"

yosys -p "
  read_liberty -lib ${LIB};
  read_verilog -lib ${SRAM_V};
  blackbox sram_512x32_2port;
  read_verilog ${NETLIST};
  hierarchy -check -top ${TOP};
  check;
  stat -liberty ${LIB};
" > "${LOGFILE}" 2>&1
RC=$?

echo "--- the verdict ----------------------------------------------"
if grep -q "Found and reported 0 problems" "${LOGFILE}"; then
  echo "  Found and reported 0 problems"
  VERDICT=0
else
  echo "  *** problems reported:"
  grep -A3 -E "Warning|ERROR|problems" "${LOGFILE}" | head -40
  VERDICT=1
fi
echo

echo "--- area -----------------------------------------------------"
grep "Chip area for" "${LOGFILE}" | sed 's/^/  /'
echo

echo "--- top-level ports ------------------------------------------"
echo "  The design boundary should be 237 pins.  V3 had 1,344 sensor"
echo "  pins alone, which is why the floorplan could not place them."
awk '/^module '"${TOP}"'/,/;/' "${NETLIST}" \
  | tr ',' '\n' | grep -c '[a-z]' | sed 's/^/  port names in header: /'
echo

echo "--- pin count sanity -----------------------------------------"
grep -cE '^[[:space:]]*(input|output|inout)' "${NETLIST}" \
  | sed 's/^/  port declarations: /'
echo

echo "  full log: ${LOGFILE}"
echo

if [ ${RC} -ne 0 ]; then
  echo "  yosys exited non-zero - see the log"
  exit 1
fi
exit ${VERDICT}
