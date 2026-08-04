#!/usr/bin/env bash
# ============================================================================
# verify_all.sh  v2  -- run EVERY pre-synthesis gate, in order.
#
#     ~/final_ivcu_project/scripts/verify_all.sh
#
# CHANGES FROM v1, both because v1 got things wrong:
#
#  GATE 3 was too strict. It failed on the four variable divisions in
#  adas_controller_v3.v. Masked STA measured that module and it has ZERO
#  violating endpoints -- the clk_100mhz violator list ran all the way down to
#  -0.600 ns and adas never appeared. A variable divide is a WARNING worth
#  seeing, not a failure. Measurement beats assumption.
#
#  GATE 5 gave a FALSE PASS. It ran `proc; opt; check -assert`, and that `opt`
#  cleaned up multiple-driver conflicts before the check could see them --
#  perception passed GATE 5 and then failed GATE 6 with 72 problems. Both gates
#  now use the SAME pass ordering as run_synthesis_v2.sh, so a pass here means
#  the same thing the real run will mean.
# ============================================================================
set -u

PROJ=/home/venky/final_ivcu_project
RTL=$PROJ/RTL
SIM=$PROJ/sim
LIB=$PROJ/libs/sky130_fd_sc_hd__tt_025C_1v80.lib
TMP=/tmp/ivcu_verify
mkdir -p "$TMP"

PASS=0; FAIL=0; WARN=0
ok()   { echo "  [PASS] $1"; PASS=$((PASS+1)); }
bad()  { echo "  [FAIL] $1"; FAIL=$((FAIL+1)); }
warn() { echo "  [WARN] $1"; WARN=$((WARN+1)); }
hdr()  { echo; echo "=============================================================="; \
         echo "  $1"; echo "=============================================================="; }

# ---------------------------------------------------------------------------
hdr "GATE 0 -- files present"
for f in "$RTL/seq_divider.v" "$RTL/sensor_interface_fabric_complete.sv" \
         "$RTL/battery_predictive_ai_complete.v" "$RTL/perception_health_ai_complete.v" \
         "$RTL/ivcu_ev_v3_hybrid_top.sv" "$RTL/defines_ivcu_ev_v3.sv" "$LIB"; do
  [ -f "$f" ] && ok "$(basename $f)" || bad "MISSING: $f"
done
[ -f "$SIM/tb_seq_divider.v" ] && ok "tb_seq_divider.v" || bad "MISSING: $SIM/tb_seq_divider.v"

# ---------------------------------------------------------------------------
hdr "GATE 1 -- CRLF line endings"
CRLF=0
for f in "$RTL"/*.v "$RTL"/*.sv "$SIM"/*.v; do
  [ -f "$f" ] || continue
  grep -qU $'\r' "$f" 2>/dev/null && { echo "     CRLF in $(basename $f)"; CRLF=1; }
done
[ $CRLF -eq 0 ] && ok "all Unix line endings" \
  || bad "run: dos2unix $RTL/*.v $RTL/*.sv $SIM/*.v"

# ---------------------------------------------------------------------------
hdr "GATE 2 -- synthesis script reads seq_divider.v"
grep -q "seq_divider.v" "$PROJ/scripts/run_synthesis_v2.sh" 2>/dev/null \
  && ok "run_synthesis_v2.sh reads seq_divider.v" \
  || { bad "add to scripts/run_synthesis_v2.sh after the defines line:"; \
       echo "         read_verilog \$RTL/seq_divider.v"; }

# ---------------------------------------------------------------------------
hdr "GATE 3 -- variable divisors  (INFORMATIONAL)"
# A divisor that is a signal, not a literal, becomes a ~130-level combinational
# chain. Whether that MATTERS depends on the clock it sits on -- which only STA
# can tell you. These are listed so you know they exist, not to block the run.
HITS=$(grep -nE "/ *[a-z_][a-z_0-9]*" "$RTL"/*.v "$RTL"/*.sv 2>/dev/null \
       | grep -vE "//|1ns/1ps|/\*" | grep -vE "16'd|32'd|8'd|4'd")
if [ -z "$HITS" ]; then
  ok "no variable divisors anywhere"
else
  echo "$HITS" | sed 's|.*/RTL/|       |'
  warn "$(echo "$HITS" | wc -l) variable divisor(s) remain -- see note below"
  cat <<'EOM'
         adas_controller_v3.v is EXPECTED here. Masked STA measured it: zero
         violating endpoints, and that list went down to -0.600 ns. Leave it.
         Only act on these if STA says a specific endpoint is negative.
EOM
fi

# ---------------------------------------------------------------------------
hdr "GATE 4 -- seq_divider is functionally correct"
if ! command -v iverilog >/dev/null 2>&1; then
  bad "iverilog not installed:   sudo apt install iverilog"
elif [ ! -f "$SIM/tb_seq_divider.v" ]; then
  bad "cannot run -- $SIM/tb_seq_divider.v is missing (see GATE 0)"
else
  if ! iverilog -g2012 -o "$TMP/tb_div" "$RTL/seq_divider.v" "$SIM/tb_seq_divider.v" \
        > "$TMP/iv.log" 2>&1; then
    bad "iverilog COMPILE failed:"; sed 's/^/       /' "$TMP/iv.log" | head -20
  else
    vvp "$TMP/tb_div" > "$TMP/tb.log" 2>&1
    if grep -q "^PASS" "$TMP/tb.log"; then
      ok "$(grep '^PASS' "$TMP/tb.log")"
    else
      bad "divider testbench FAILED -- full output:"
      sed 's/^/       /' "$TMP/tb.log" | head -25
      [ -s "$TMP/tb.log" ] || echo "       (no output at all -- check $TMP/iv.log)"
    fi
  fi
fi

# ---------------------------------------------------------------------------
# Same pass ordering as run_synthesis_v2.sh. NO bare `opt` before the check --
# that is what let perception pass GATE 5 in v1 and then fail GATE 6.
CHK="proc; opt_expr; bmuxmap; demuxmap; check -assert; stat"

hdr "GATE 5 -- each edited module elaborates cleanly"
elab () {
  local top=$1; shift
  local src=""
  for s in "$@"; do src="$src read_verilog -sv -DSYNTHESIS $s;"; done
  yosys -p "read_verilog -sv -DSYNTHESIS $RTL/defines_ivcu_ev_v3.sv; \
            $src hierarchy -check -top $top; $CHK" > "$TMP/$top.log" 2>&1
  local rc=$?
  local prob=$(grep -o "Found and reported [0-9]* problems" "$TMP/$top.log" | tail -1)
  local cells=$(grep -E "Number of cells:" "$TMP/$top.log" | tail -1 | awk '{print $NF}')
  if [ $rc -eq 0 ] && echo "$prob" | grep -q "reported 0 problems"; then
    ok "$top   ($prob, cells=${cells:-n/a})"
  else
    bad "$top -- ${prob:-elaboration error}   log: $TMP/$top.log"
    grep -iE "^ERROR|multiple conflicting drivers|not found" "$TMP/$top.log" \
      | head -6 | sed 's/^/       /'
  fi
}
elab seq_divider                      "$RTL/seq_divider.v"
elab sensor_interface_fabric_complete "$RTL/sensor_interface_fabric_complete.sv"
elab battery_predictive_ai_complete   "$RTL/seq_divider.v" "$RTL/battery_predictive_ai_complete.v"
elab perception_health_ai_complete    "$RTL/seq_divider.v" "$RTL/perception_health_ai_complete.v"

# ---------------------------------------------------------------------------
hdr "GATE 6 -- WHOLE DESIGN elaborates"
SRCS=""
for f in defines_ivcu_ev_v3.sv seq_divider.v ivcu_ev_v3_hybrid_top.sv sync_cell.sv \
         clock_manager_14nm.v reset_sync_v3.v power_domain_controller_v3.v \
         mode_config_enhanced_v3.sv sensor_enable_logic.sv adc_interface_14nm.v \
         sensor_interface_fabric_complete.sv sensor_grace_manager_complete.sv \
         sensor_validation_fsm.sv battery_predictive_ai_complete.v \
         thermal_management_hierarchical_complete.sv motor_condition_enhanced_complete.v \
         vehicle_dynamics_predictive_complete.v perception_health_ai_complete.v \
         crash_predictive_ai_complete.v system_health_ai_complete.sv \
         central_safety_fsm_v3.v adas_controller_v3.v motor_control_hybrid.v \
         emergency_response_system.v fault_logger_sram_32kb.sv \
         diagnostic_report_generator.v mcu_axi_lite_interface.sv; do
  SRCS="$SRCS read_verilog -sv -DSYNTHESIS $RTL/$f;"
done
yosys -p "$SRCS hierarchy -check -top ivcu_ev_v3_hybrid_top; $CHK" \
      > "$TMP/top.log" 2>&1
RC=$?
PROB=$(grep -o "Found and reported [0-9]* problems" "$TMP/top.log" | tail -1)
if [ $RC -eq 0 ] && echo "$PROB" | grep -q "reported 0 problems"; then
  ok "ivcu_ev_v3_hybrid_top   ($PROB)"
  grep -E "Number of cells:|Number of wires:" "$TMP/top.log" | tail -2 | sed 's/^/       /'
else
  bad "whole design -- ${PROB:-error}   log: $TMP/top.log"
  echo "       --- which signals, and in which module ---"
  grep -oE "multiple conflicting drivers for [^ ]*\\\\[a-zA-Z_0-9]+" "$TMP/top.log" \
    | sed 's/multiple conflicting drivers for //' | sed 's/\[.*//' \
    | sort -u | head -15 | sed 's/^/       /'
  grep -iE "^ERROR|not found|referenced in module" "$TMP/top.log" | head -6 | sed 's/^/       /'
fi

# ---------------------------------------------------------------------------
hdr "RESULT"
echo "  passed: $PASS    warnings: $WARN    failed: $FAIL"
echo
if [ $FAIL -eq 0 ]; then
  cat <<'EOM'
  ALL GATES PASSED.  Safe to start the long run:

      ~/final_ivcu_project/scripts/run_synthesis_v2.sh full

  First:
    - quit Docker Desktop entirely (it holds the WSL VM and its RAM)
    - confirm nothing is already running:   pgrep -c yosys     -> must be 0
    - expect ~6.5 hours, ~5.6 GB peak

  OpenRAM does NOT need re-running. The SRAM macro and its wrapper netlist
  are untouched by these edits.
EOM
else
  echo "  DO NOT START SYNTHESIS. Fix the [FAIL] items and re-run."
  echo "  [WARN] items are informational and do not block."
fi
echo
exit $FAIL
