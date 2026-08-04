# ============================================================================
# run_sta.tcl  --  pre-layout static timing analysis, IVCU-EV V3
#
#   cd ~/final_ivcu_project
#   sta -no_splash -exit scripts/run_sta.tcl 2>&1 | tee sta_out/sta_prelayout.txt
#
# WHAT THIS IS AND IS NOT
#   This is PRE-LAYOUT STA. There are no wires yet, so there are no real
#   parasitics. OpenSTA assumes ideal interconnect: zero wire resistance and
#   only whatever capacitance a wire-load model supplies (none here).
#
#   Therefore:
#     - Real post-route timing will be WORSE than this, often substantially.
#     - Passing here does NOT mean timing closes.
#     - FAILING here is meaningful: if a path is already negative with zero
#       wire delay, routing cannot rescue it. Those are the ones to fix now.
#
#   Same logic for power: what you get is cell internal + leakage, plus
#   switching power estimated from an assumed activity factor. Real switching
#   power scales with wire capacitance, which does not exist yet.
#
#   Area and cell count are NOT computed here -- Yosys `stat -liberty` already
#   gave those and they are authoritative pre-layout.
# ============================================================================

set PROJ    /home/venky/final_ivcu_project
set LIB_STD $PROJ/libs/sky130_fd_sc_hd__tt_025C_1v80.lib
set LIB_RAM $PROJ/macros/sram_512x32_2port_TT_1p8V_25C.lib
set NET_TOP $PROJ/synth_out/ivcu_ev_v3_hybrid_top_gate_full.v
set NET_RAM $PROJ/synth_out/fault_log_sram_1024x32_netlist.v
set SDC     $PROJ/SDC/ivcu_ev_v3.sdc
set TOP     ivcu_ev_v3_hybrid_top

# ---------------------------------------------------------------------------
# Libraries. BOTH are required -- standard cells and the SRAM macro. Without
# the macro liberty, STA sees an unresolved black box where the memory is and
# silently stops timing through it.
# ---------------------------------------------------------------------------
puts "=========== READING LIBERTY ==========="
read_liberty $LIB_STD
read_liberty $LIB_RAM

# ---------------------------------------------------------------------------
# Netlists. Main design plus the SRAM wrapper that glues the two macros.
# ---------------------------------------------------------------------------
puts "=========== READING NETLISTS ==========="
read_verilog $NET_TOP
read_verilog $NET_RAM
link_design $TOP

# ---------------------------------------------------------------------------
# Constraints.
# ---------------------------------------------------------------------------
puts "=========== READING SDC ==========="
read_sdc $SDC

# Assumed switching activity for power estimation. 0.2 means each net is
# assumed to toggle on 20% of clock edges. This is a guess -- for a real
# number you need a VCD from simulation and `read_vcd`.
set_power_activity -global -activity 0.2

# ---------------------------------------------------------------------------
puts ""
puts "=========================================================="
puts "  CLOCKS AS SEEN BY STA"
puts "  If a clock you expect is missing here, its paths are"
puts "  unconstrained and will be silently skipped."
puts "=========================================================="
report_clock_properties

# ---------------------------------------------------------------------------
puts ""
puts "=========================================================="
puts "  DESIGN RULE VIOLATIONS (slew / capacitance / fanout)"
puts "  These are real even pre-layout -- they come from cell"
puts "  drive strength versus load, not from wires."
puts "=========================================================="
report_check_types -max_slew -max_capacitance -max_fanout -violators

# ---------------------------------------------------------------------------
puts ""
puts "=========================================================="
puts "  SETUP TIMING  (max delay)"
puts "  Negative slack here cannot be fixed by routing."
puts "=========================================================="
report_worst_slack -max -digits 3
report_tns -digits 3
report_wns -digits 3
puts ""
puts "---- 10 worst setup paths ----"
report_checks -path_delay max -group_count 10 -slack_max 0.0 -digits 3

# ---------------------------------------------------------------------------
puts ""
puts "=========================================================="
puts "  HOLD TIMING  (min delay)"
puts "  Pre-layout hold is optimistic -- clock tree does not"
puts "  exist yet. Treat gross violations as real, small ones"
puts "  as noise until after CTS."
puts "=========================================================="
report_worst_slack -min -digits 3
puts ""
puts "---- 10 worst hold paths ----"
report_checks -path_delay min -group_count 10 -slack_max 0.0 -digits 3

# ---------------------------------------------------------------------------
puts ""
puts "=========================================================="
puts "  UNCONSTRAINED PATHS"
puts "  Anything listed here is NOT being timed. That is the"
puts "  most dangerous category -- silent, not loud."
puts "=========================================================="
report_checks -unconstrained -digits 3

# ---------------------------------------------------------------------------
puts ""
puts "=========================================================="
puts "  POWER  (estimate, activity = 0.2, no wire capacitance)"
puts "=========================================================="
report_power -digits 3

puts ""
puts "=========================================================="
puts "  STA COMPLETE"
puts "=========================================================="
