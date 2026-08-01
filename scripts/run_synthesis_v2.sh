#!/usr/bin/env bash
# ============================================================================
# run_synthesis_v2.sh -- IVCU-EV V3 synthesis, Sky130 HD
#
# Place in ~/final_ivcu_project/scripts/ and run from anywhere:
#     chmod +x ~/final_ivcu_project/scripts/run_synthesis_v2.sh
#     ~/final_ivcu_project/scripts/run_synthesis_v2.sh fast     <- middle gate
#     ~/final_ivcu_project/scripts/run_synthesis_v2.sh full     <- signoff run
#
# WHAT CHANGED vs run_real_synthesis.sh
#   1. defines_ivcu_ev_v3.sv is now read FIRST. Its absence was the root cause
#      of the 2327 undriven-wire problems.
#   2. `check -assert` runs BEFORE write_verilog. A broken design now aborts
#      instead of silently writing a 21 MB netlist that has to be debugged
#      backwards.
#   3. Output goes to synth_out/, never into RTL/. A gate netlist sitting in the
#      source directory is read back in as source on the next run.
#   4. `fast` mode uses `abc -fast`, which cuts the dominant runtime cost for
#      iteration. Use `full` only once `fast` is clean.
#   5. Logs are timestamped so runs stop overwriting each other.
# ============================================================================
set -u

MODE="${1:-fast}"
if [ "$MODE" != "fast" ] && [ "$MODE" != "full" ]; then
  echo "usage: $0 [fast|full]"; exit 1
fi

PROJ=/home/venky/final_ivcu_project
RTL=$PROJ/RTL
OUT=$PROJ/synth_out
LIB=$PROJ/libs/sky130_fd_sc_hd__tt_025C_1v80.lib
TOP=ivcu_ev_v3_hybrid_top
STAMP=$(date +%Y%m%d_%H%M%S)
LOG=$OUT/synth_${MODE}_${STAMP}.log
NETLIST=$OUT/${TOP}_gate_${MODE}_${STAMP}.v
YS=$OUT/synth_${MODE}_${STAMP}.ys

mkdir -p "$OUT"

if [ ! -f "$LIB" ]; then echo "missing liberty: $LIB"; exit 1; fi

# ---- abc invocation differs by mode ----------------------------------------
if [ "$MODE" = "fast" ]; then
  ABC_CMD="abc -fast -liberty $LIB"
else
  ABC_CMD="abc -liberty $LIB"
fi

# ---- build the yosys script -------------------------------------------------
cat > "$YS" <<EOF
# ---------- read ----------
# defines MUST come first -- every module \`include's it, and without it the
# guarded logic silently vanishes and outputs end up undriven.
read_verilog -sv -DSYNTHESIS $RTL/defines_ivcu_ev_v3.sv

read_verilog -sv -DSYNTHESIS $RTL/ivcu_ev_v3_hybrid_top.sv
read_verilog -sv -DSYNTHESIS $RTL/sync_cell.sv
read_verilog $RTL/clock_manager_14nm.v
read_verilog $RTL/reset_sync_v3.v
read_verilog $RTL/power_domain_controller_v3.v
read_verilog -sv $RTL/mode_config_enhanced_v3.sv
read_verilog -sv $RTL/sensor_enable_logic.sv
read_verilog $RTL/adc_interface_14nm.v
read_verilog -sv $RTL/sensor_interface_fabric_complete.sv
read_verilog -sv $RTL/sensor_grace_manager_complete.sv
read_verilog -sv $RTL/sensor_validation_fsm.sv
read_verilog $RTL/battery_predictive_ai_complete.v
read_verilog -sv $RTL/thermal_management_hierarchical_complete.sv
read_verilog $RTL/motor_condition_enhanced_complete.v
read_verilog $RTL/vehicle_dynamics_predictive_complete.v
read_verilog $RTL/perception_health_ai_complete.v
read_verilog $RTL/crash_predictive_ai_complete.v
read_verilog -sv $RTL/system_health_ai_complete.sv
read_verilog $RTL/central_safety_fsm_v3.v
read_verilog $RTL/adas_controller_v3.v
read_verilog $RTL/motor_control_hybrid.v
read_verilog $RTL/emergency_response_system.v
read_verilog -sv -DSYNTHESIS $RTL/fault_logger_sram_32kb.sv
read_verilog $RTL/diagnostic_report_generator.v
read_verilog -sv $RTL/mcu_axi_lite_interface.sv

# ---------- elaborate ----------
hierarchy -check -top $TOP

# Early gate: catches undriven / multi-driven / loops before any time is spent
# on mapping. Aborts the run rather than pressing on.
proc
opt_expr
bmuxmap
demuxmap
check -assert

# ---------- generic optimisation ----------
opt
fsm
opt
memory
opt

# ---------- technology mapping ----------
techmap
opt

dfflibmap -liberty $LIB
$ABC_CMD

# ---------- cleanup ----------
# setundef: tie any remaining x to 0 so P&R never sees an undefined value.
# opt_clean WITHOUT -purge: -purge additionally removes unused *public* wires,
# which destroys port connectivity. Plain opt_clean only removes genuinely
# unused private nets and is safe.
#
# splitnets -ports is deliberately NOT used. OpenROAD does not need a
# bit-blasted netlist and it inflates the output enormously.
setundef -zero
opt_clean

# ---------- report ----------
stat -liberty $LIB

# ---------- WRITE FIRST, THEN JUDGE ----------
# The netlist is written BEFORE the assert runs. After a multi-hour run you
# always end up with a file you can inspect, even if it turns out to be bad.
# The wrapper script renames it to *_REJECTED_* if the check fails, so a bad
# netlist can never be mistaken for a good one later.
write_verilog -noattr $NETLIST

# ---------- FINAL GATE ----------
# -assert  : exit non-zero instead of merely printing
# -mapped  : also flag any cell ABC failed to map
#
# Runs on the real mapped netlist, after cleanup but with no port-mangling
# passes in between. An earlier version ran this after "splitnets -ports;
# opt_clean -purge" and produced 2330 false failures -- splitnets bit-blasts
# every port into single wires and -purge then deletes the nets carrying the
# driver connections, so every output looked undriven. The design was fine;
# the check was looking at a mangled view of it.
check -assert -mapped
EOF

# ---- run --------------------------------------------------------------------
echo "mode     : $MODE"
echo "script   : $YS"
echo "log      : $LOG"
echo "netlist  : $NETLIST"
echo "started  : $(date)"
echo

/usr/bin/time -v yosys -l "$LOG" "$YS" 2> "$OUT/time_${MODE}_${STAMP}.txt"
RC=$?

echo
echo "finished : $(date)"
echo "exit code: $RC"

PEAKKB=$(awk '/Maximum resident set size/{print $NF}' "$OUT/time_${MODE}_${STAMP}.txt" 2>/dev/null)
WALL=$(awk -F': ' '/Elapsed \(wall clock\)/{print $NF}' "$OUT/time_${MODE}_${STAMP}.txt" 2>/dev/null)

if [ $RC -ne 0 ]; then
  echo
  # Exit 137 = killed by signal 9 = almost always the Linux OOM killer.
  if [ $RC -eq 137 ] || [ $RC -eq 9 ]; then
    echo "*** KILLED -- almost certainly OUT OF MEMORY, not a design fault ***"
    echo "    Confirm with:  dmesg | grep -iE 'killed process|out of memory' | tail -5"
    echo "    Then rerun in fast mode:  $0 fast"
  else
    echo "*** SYNTHESIS CHECK FAILED ***"
    echo "    The netlist WAS written, but it did not pass check -assert."
    echo "    It is renamed with REJECTED so it can never be mistaken for good."
  fi
  if [ -f "$NETLIST" ]; then
    BAD="$OUT/REJECTED_$(basename "$NETLIST")"
    mv "$NETLIST" "$BAD"
    echo
    echo "    rejected netlist kept for inspection: $BAD"
  fi
  echo "    peak memory: ${PEAKKB:-unknown} kB      wall clock: ${WALL:-unknown}"
  echo
  echo "--- last 30 log lines ---"
  tail -30 "$LOG"
  exit $RC
fi

echo
echo "=== PASSED ==="
echo "=== chip area ==="
grep -i "Chip area" "$LOG"
echo "=== SRAM macro instance (want exactly 1) ==="
grep -c "fault_log_sram_1024x32" "$NETLIST"
echo "=== DFF instances ==="
grep -c "sky130_fd_sc_hd__df" "$NETLIST"
echo "=== resources ==="
echo "peak memory: ${PEAKKB:-unknown} kB"
echo "wall clock : ${WALL:-unknown}"
echo
echo "netlist: $NETLIST"