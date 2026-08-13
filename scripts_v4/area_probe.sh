#!/usr/bin/env bash
#=============================================================================
# area_probe.sh  -  find out what this design costs BEFORE spending five hours
#
# WHY THIS EXISTS
#
# The V3 synthesis run took 4:53:29 and peaked at 3.17 GB.  At the end of it
# you learned that u_diagnostic was 118,908 um2 of logic connected to nothing.
# That is an expensive way to find out.
#
# This script runs generic synthesis only - no technology mapping, no ABC, no
# liberty - which takes seconds instead of hours and still answers the two
# questions that matter most before a real run:
#
#   1  Is anything unexpectedly large?
#   2  Are the blocks that were supposed to disappear actually gone?
#
# Flip-flop count is the honest pre-synthesis proxy for area in a design like
# this one.  Combinational logic gets restructured heavily by ABC; registers do
# not.  If the flop count is sane, the area will be roughly sane.
#
# USAGE
#     bash scripts_v4/area_probe.sh
#     less qc_v4/area_probe.log
#=============================================================================

set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${HERE}/.." && pwd)"
RTL="${ROOT}/RTL_V4"
TOP="ivcu_ev_v4_top"
LOG="${ROOT}/qc_v4"
mkdir -p "${LOG}"

if ! command -v yosys >/dev/null 2>&1; then
  echo "yosys not on PATH"
  exit 1
fi

echo
echo "=============================================================="
echo " IVCU-EV V4 area probe   (generic synthesis, no tech mapping)"
echo "=============================================================="
echo

yosys -p "
  read_verilog -I ${RTL} ${RTL}/*.v;
  read_verilog -lib ${ROOT}/macros/sram_512x32_2port.v;
  hierarchy -check -top ${TOP};
  proc;
  opt -full;
  memory_collect;
  opt -full;
  stat;
" > "${LOG}/area_probe.log" 2>&1

RC=$?
if [ ${RC} -ne 0 ]; then
  echo "  probe FAILED - see qc_v4/area_probe.log"
  tail -40 "${LOG}/area_probe.log"
  exit 1
fi

#-----------------------------------------------------------------------------
# Per-module cell counts, largest first.
#
# Yosys stat prints one section per module:
#     === module_name ===
#        Number of cells:   123
# so the module name is field 2 of the header line and the count is the last
# field of the "Number of cells" line inside that section.
#-----------------------------------------------------------------------------
# Yosys has used two stat formats.  Older builds print
#     Number of cells:            7295
# and newer ones (0.6x) print
#          7295 cells
# Handle both, because the machine that runs this may not be the one it was
# written on.
PARSED=$(awk '
  /^=== / { mod = $2; next }
  /Number of cells:/          { if (mod != "") printf "%8d  %s\n", $NF, mod }
  /^[[:space:]]*[0-9]+ cells$/{ if (mod != "") printf "%8d  %s\n", $1,  mod }
' "${LOG}/area_probe.log" | sort -rn)

if [ -z "${PARSED}" ]; then
  echo "--- could not parse stat output -----------------------------"
  echo "  Showing the tail of the log instead.  If there is no stat"
  echo "  section below, yosys did not reach the stat command."
  echo
  tail -60 "${LOG}/area_probe.log"
  echo
else
  echo "--- cells per module, largest first -------------------------"
  echo "${PARSED}"
  echo

  # NOTE ON WHAT THIS COUNTS
  # Before technology mapping a $adff cell is one cell of WIDTH bits, so this
  # is a count of register GROUPS, not of flip-flops.  It is still the right
  # thing to sort modules by, but do not read the number as a flop count -
  # the real one only exists after techmap.
  echo "--- register cells per module, largest first ----------------"
  awk '
    /^=== / { mod = $2; next }
    /^[[:space:]]*[0-9]+[[:space:]]+\$(a|s)?dffe?$/ {
        if (mod != "") ff[mod] += $1
    }
    END { for (m in ff) printf "%8d  %s\n", ff[m], m }
  ' "${LOG}/area_probe.log" | sort -rn
  echo

  echo "--- design hierarchy summary --------------------------------"
  sed -n '/=== design hierarchy ===/,$p' "${LOG}/area_probe.log"
  echo
fi

#-----------------------------------------------------------------------------
# The blocks that were supposed to disappear
#-----------------------------------------------------------------------------
echo "--- V3 blocks that must NOT appear --------------------------"
for dead in diagnostic_report_generator sensor_grace_manager_complete \
            seq_divider adc_interface_14nm clock_manager_14nm; do
  if grep -q "${dead}" "${LOG}/area_probe.log"; then
    printf "  *** STILL PRESENT: %s\n" "${dead}"
  else
    printf "  gone: %s\n" "${dead}"
  fi
done
echo

#-----------------------------------------------------------------------------
# Dividers.  RULE R5 says there are none; this is the structural proof.
#-----------------------------------------------------------------------------
#   CORRECTED 13 Aug 2026.  The pattern here was '\\\$(div|mod)', which in a
#   single-quoted shell string is the regex \\\$ - a literal BACKSLASH followed
#   by a literal $.  The log contains "$div" with no backslash, so the pattern
#   could never match and this check reported PASS unconditionally.
#
#   It was a check that could only ever say yes.  Worse than no check, because
#   it looked like coverage.  The multiplier check below had the same bug and
#   hid five real $mul cells.
echo "--- divider check (RULE R5) ---------------------------------"
if grep -qE '\$(div|mod|divfloor|modfloor)\b' "${LOG}/area_probe.log"; then
  echo "  *** A DIVIDER WAS INFERRED - this is what broke V3 STA"
  grep -E '\$(div|mod|divfloor|modfloor)\b' "${LOG}/area_probe.log"
else
  echo "  no \$div / \$mod cells inferred anywhere"
fi
echo

#-----------------------------------------------------------------------------
# Multipliers.  Not forbidden, but every one should be explainable - this is
# the mechanism that cost V3 575,000 um2 in the sensor fabric.
#-----------------------------------------------------------------------------
echo "--- multiplier check ----------------------------------------"
if grep -qE '\$mul\b' "${LOG}/area_probe.log"; then
  echo "  multipliers inferred - each one must be explainable:"
  grep -E '\$mul\b' "${LOG}/area_probe.log"
  echo
  echo "  Most likely source: a part-select index computed with a VARIABLE"
  echo "  times a non-power-of-two, e.g. sensor_status_flat[idx*3 +: 3]."
  echo "  x*16 and x*4 fold to shifts and cost nothing; x*3 does not."
else
  echo "  no \$mul cells inferred anywhere"
fi
echo

echo "full log: qc_v4/area_probe.log"
echo
