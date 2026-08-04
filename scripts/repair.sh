#!/usr/bin/env bash
# ============================================================================
# repair.sh  v2  -- mechanical fixes for GATE 1, 2 and the sim/ directory.
#
#     ~/final_ivcu_project/scripts/repair.sh
#
# v1 tried to recover your fixed battery/perception files from RTL_backup/.
# That directory was never created, so the recovery was done a different way:
# the named-block fix has been reconstructed directly into the two replacement
# files (integer i / j / confidence_scaled / curr_sum_tmp moved back inside
# their named always blocks). Copy those two files in BEFORE running this.
#
# This script now only does the boring stuff that is safe to automate.
# ============================================================================
set -u

PROJ=/home/venky/final_ivcu_project
RTL=$PROJ/RTL
SIM=$PROJ/sim
SCRIPTS=$PROJ/scripts

# ---------------------------------------------------------------------------
echo "=============================================================="
echo " STEP 1 -- confirm the two rebuilt files are actually in place"
echo "=============================================================="
BAD=0
# The fixed version has NO module-scope integer/confidence_scaled/curr_sum_tmp.
# Module scope is exactly 4 spaces of indent; block-local is 8.
for f in battery_predictive_ai_complete.v perception_health_ai_complete.v; do
  N=$(grep -cE "^    (integer |reg .*(confidence_scaled|curr_sum_tmp))" "$RTL/$f" 2>/dev/null)
  D=$(grep -cE "^        (integer |reg \[.*\] (confidence_scaled|curr_sum_tmp))" "$RTL/$f" 2>/dev/null)
  if [ "${N:-1}" -eq 0 ] && [ "${D:-0}" -gt 0 ]; then
    echo "  OK    $f   ($D block-local declarations, 0 at module scope)"
  else
    echo "  WRONG $f   ($N at module scope, $D block-local)"
    echo "        -> this is still the STALE file. Copy the rebuilt one in first."
    BAD=1
  fi
done
if [ $BAD -ne 0 ]; then
  echo
  echo "  Stopping. Copy the two rebuilt files into RTL/ and run this again."
  exit 1
fi

# ---------------------------------------------------------------------------
echo
echo "=============================================================="
echo " STEP 2 -- line endings"
echo "=============================================================="
mkdir -p "$SIM"
dos2unix -q "$RTL"/*.v "$RTL"/*.sv 2>/dev/null
ls "$SIM"/*.v >/dev/null 2>&1 && dos2unix -q "$SIM"/*.v 2>/dev/null
dos2unix -q "$SCRIPTS"/*.sh "$SCRIPTS"/*.tcl 2>/dev/null
echo "  converted RTL/, sim/ and scripts/ to Unix line endings"

# ---------------------------------------------------------------------------
echo
echo "=============================================================="
echo " STEP 3 -- synthesis script must read seq_divider.v"
echo "=============================================================="
S=$SCRIPTS/run_synthesis_v2.sh
if grep -q "seq_divider.v" "$S"; then
  echo "  already present"
else
  cp "$S" "$S.bak"
  sed -i 's|^\(read_verilog -sv -DSYNTHESIS \$RTL/defines_ivcu_ev_v3.sv\)$|\1\n\n# Sequential divider. Used by battery_predictive_ai and perception_health_ai,\n# so it MUST be read before them or hierarchy -check reports an unknown module.\nread_verilog \$RTL/seq_divider.v|' "$S"
  if grep -q "seq_divider.v" "$S"; then
    echo "  added (original saved as run_synthesis_v2.sh.bak)"
    grep -n -A1 "seq_divider.v" "$S" | head -4 | sed 's/^/     /'
  else
    echo "  COULD NOT auto-insert. Add by hand to $S, inside the heredoc,"
    echo "  just after the defines_ivcu_ev_v3.sv line:"
    echo "        read_verilog \$RTL/seq_divider.v"
  fi
fi

# ---------------------------------------------------------------------------
echo
echo "=============================================================="
echo " STEP 4 -- testbench in sim/"
echo "=============================================================="
if [ -f "$SIM/tb_seq_divider.v" ]; then
  echo "  present"
else
  echo "  MISSING: $SIM/tb_seq_divider.v"
  echo "  Copy tb_seq_divider.v into $SIM/ then re-run this script."
fi

# ---------------------------------------------------------------------------
echo
echo "=============================================================="
echo " STEP 5 -- make a real backup, so this cannot happen twice"
echo "=============================================================="
BAK=$PROJ/RTL_backup_$(date +%Y%m%d_%H%M%S)
mkdir -p "$BAK"
cp "$RTL"/*.v "$RTL"/*.sv "$BAK"/
echo "  saved $(ls "$BAK" | wc -l) files to $(basename $BAK)/"

echo
echo "=============================================================="
echo " NEXT:   $SCRIPTS/verify_all.sh"
echo "=============================================================="
echo