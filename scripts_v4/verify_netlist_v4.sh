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

#-----------------------------------------------------------------------------
# PORT COUNT
#
# CORRECTED 13 Aug 2026.  This used to count port DECLARATIONS and compare
# them against 237, which is the BIT count.  "input [11:0] adc_data" is one
# declaration and twelve pins, so the check compared two different quantities
# and looked alarming when nothing was wrong.
#
# What matters for the floorplan is bits - those are the physical pins that
# have to be placed on the die boundary.
#-----------------------------------------------------------------------------
echo "--- top-level ports ------------------------------------------"
awk '
  /^[[:space:]]*(input|output|inout)\b/ {
      # width from a [hi:lo] range, default 1
      w = 1
      if (match($0, /\[[0-9]+:[0-9]+\]/)) {
          r = substr($0, RSTART+1, RLENGTH-2)
          split(r, a, ":")
          hi = a[1] + 0; lo = a[2] + 0
          w = (hi > lo ? hi - lo : lo - hi) + 1
      }
      # a declaration may name several ports: input a, b, c;
      line = $0
      sub(/^[[:space:]]*(input|output|inout)[[:space:]]*/, "", line)
      sub(/\[[0-9]+:[0-9]+\][[:space:]]*/, "", line)
      sub(/;.*$/, "", line)
      n = split(line, names, ",")
      cnt = 0
      for (i = 1; i <= n; i++) if (names[i] ~ /[A-Za-z_]/) cnt++
      decls += cnt
      bits  += cnt * w
  }
  END {
      printf "  port declarations : %d\n", decls
      printf "  PHYSICAL PINS     : %d\n", bits
  }
' "${NETLIST}"
echo
echo "  Target is 237 pins on a 5,840 um perimeter = 24.6 um per pin."
echo "  V3 had 1,344 sensor pins alone = 4.3 um per pin, which is why"
echo "  the floorplan could not place and route them."
echo

echo "  full log: ${LOGFILE}"
echo

if [ ${RC} -ne 0 ]; then
  echo "  yosys exited non-zero - see the log"
  exit 1
fi
exit ${VERDICT}
