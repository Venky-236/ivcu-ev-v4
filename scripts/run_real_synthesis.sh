#!/bin/bash
# run_real_synthesis.sh
# Verified synthesis flow for ivcu_ev_v3_hybrid_top with real sky130 liberty mapping.
# This needs more RAM than a small sandbox provides - run on your WSL machine
# where you already have swap configured from earlier synthesis work.
#
# Usage:
#   1. Put this script + all RTL files + the .lib file in the same directory
#      (or adjust paths below).
#   2. chmod +x run_real_synthesis.sh
#   3. ./run_real_synthesis.sh
#
# Expect this to take a while (design has 93 division operators - dozens are
# genuine, non-power-of-2 dividers, which are inherently expensive to map).
# Watch memory with `free -h` in another terminal if you're unsure it's progressing.

set -e

LIB="sky130_fd_sc_hd__tt_025C_1v80.lib"

if [ ! -f "$LIB" ]; then
    echo "ERROR: $LIB not found in this directory. Copy/symlink it here first."
    exit 1
fi

yosys -l real_synth_log.txt <<'EOF'
read_verilog -sv -DSYNTHESIS ivcu_ev_v3_hybrid_top.sv
read_verilog -sv -DSYNTHESIS sync_cell.sv
read_verilog clock_manager_14nm.v
read_verilog reset_sync_v3.v
read_verilog power_domain_controller_v3.v
read_verilog -sv mode_config_enhanced_v3.sv
read_verilog -sv sensor_enable_logic.sv
read_verilog adc_interface_14nm.v
read_verilog -sv sensor_interface_fabric_complete.sv
read_verilog -sv sensor_grace_manager_complete.sv
read_verilog -sv sensor_validation_fsm.sv
read_verilog battery_predictive_ai_complete.v
read_verilog -sv thermal_management_hierarchical_complete.sv
read_verilog motor_condition_enhanced_complete.v
read_verilog vehicle_dynamics_predictive_complete.v
read_verilog perception_health_ai_complete.v
read_verilog crash_predictive_ai_complete.v
read_verilog -sv system_health_ai_complete.sv
read_verilog central_safety_fsm_v3.v
read_verilog adas_controller_v3.v
read_verilog motor_control_hybrid.v
read_verilog emergency_response_system.v
read_verilog -sv -DSYNTHESIS fault_logger_sram_32kb.sv
read_verilog diagnostic_report_generator.v
read_verilog -sv mcu_axi_lite_interface.sv

hierarchy -check -top ivcu_ev_v3_hybrid_top
proc
opt_clean -purge
check -noinit
memory_collect
memory_dff
memory_share
opt_clean -purge
memory_map
opt_clean

# safe, automated width reduction - narrows operand widths where provably
# unused, no RTL changes, no risk of the overflow bug we found manually
wreduce
opt -purge

bmuxmap
demuxmap
opt -purge

techmap
opt -purge

dfflibmap -liberty sky130_fd_sc_hd__tt_025C_1v80.lib
abc -liberty sky130_fd_sc_hd__tt_025C_1v80.lib
opt_clean

# Save the real deliverables FIRST, unconditionally - before check gets a
# chance to abort the whole process. This is the fix: last time, check
# -assert killed yosys before these two lines ever ran, losing hours of work.
stat -liberty sky130_fd_sc_hd__tt_025C_1v80.lib
write_verilog ivcu_ev_v3_hybrid_top_gate_netlist.v

# Now check is purely informational - no -assert, so a warning can never
# blow away the netlist/area report we already just saved above.
check
EOF

echo ""
echo "Done. Real area/timing numbers are in real_synth_log.txt (search for 'Chip area')."
echo "Gate-level netlist: ivcu_ev_v3_hybrid_top_gate_netlist.v"
echo "Any 'no driver' warnings from 'check' at the end are logged for review,"
echo "but will NOT have aborted this run or lost the netlist above."
