# ============================================================================
# run_sta_mask_reset.tcl  --  DIAGNOSTIC ONLY.  NOT FOR SIGNOFF.
#
#   cd ~/final_ivcu_project
#   sta -no_splash -exit scripts/run_sta_mask_reset.tcl 2>&1 \
#       | tee sta_out/sta_mask.txt
#
# Minutes. Runs on the netlist you already have.
#
# WHAT THIS DOES AND WHY IT IS LEGITIMATE
#   The four reset nets are unbuffered: one minimum-strength gate driving
#   15372 / 1317 / 911 / 347 flop pins. Their delay is pure RC loading and it
#   is almost exactly linear in fanout:
#
#       _169_/Y   15372 loads   1136.681 ns    0.0739 ns/load
#       _188_/Y    1317 loads     96.606 ns    0.0733 ns/load
#       _150_/Y     911 loads     67.269 ns    0.0738 ns/load
#       _130_/Y     347 loads     25.562 ns    0.0737 ns/load
#
#   That linearity is the proof it is loading, not logic. OpenROAD's
#   repair_design inserts buffer trees during placement and these collapse to
#   a few hundred picoseconds. Nothing in the RTL causes it and nothing in the
#   RTL can fix it.
#
#   But while they are present they dominate every path group, so the 40-deep
#   endpoint lists in the previous run were 100% reset in three of five groups.
#   Anything real below about -89 ns was invisible.
#
#   Masking them here lets the REAL logic timing surface. These false paths
#   exist ONLY in this diagnostic script. They are not in SDC/ivcu_ev_v3.sdc
#   and must never be, or signoff would be lying to itself.
#
# WHAT WE ARE LOOKING FOR
#   Six variable-divisor divisions on clk_ai (10 ns period):
#       adas_controller_v3.v:118,119   / valid_count
#       adas_controller_v3.v:145       / vehicle_speed
#       adas_controller_v3.v:158       / object_relative_speed
#       battery_predictive_ai:297      / current_avg
#       motor_condition_enhanced:142   / motor_max_rpm  (tied to 16'd12000)
#   If u_adas, u_battery_ai or u_motor_ai endpoints appear with negative
#   slack, they need fixing in the SAME rtl pass as the sensor fabric, so we
#   only pay for one synthesis run.
# ============================================================================

set PROJ    /home/venky/final_ivcu_project
set LIB_STD $PROJ/libs/sky130_fd_sc_hd__tt_025C_1v80.lib
set LIB_RAM $PROJ/macros/sram_512x32_2port_TT_1p8V_25C.lib
set NET_TOP $PROJ/synth_out/ivcu_ev_v3_hybrid_top_gate_full.v
set NET_RAM $PROJ/synth_out/fault_log_sram_1024x32_netlist.v
set SDC     $PROJ/SDC/ivcu_ev_v3.sdc
set TOP     ivcu_ev_v3_hybrid_top

read_liberty $LIB_STD
read_liberty $LIB_RAM
read_verilog $NET_TOP
read_verilog $NET_RAM
link_design  $TOP
read_sdc     $SDC

# ---------------------------------------------------------------------------
puts ""
puts "############################################################"
puts "#  MASKING THE FOUR UNBUFFERED RESET NETS  (diagnostic only)"
puts "############################################################"
foreach pin {u_reset_sync/_169_/Y u_reset_sync/_188_/Y \
             u_reset_sync/_150_/Y u_reset_sync/_130_/Y} {
    if {[catch {set_false_path -through [get_pins $pin]} msg]} {
        puts "  COULD NOT MASK $pin  --  $msg"
    } else {
        puts "  masked: $pin"
    }
}
puts ""
puts "  NOTE: por_n (fanout 54, approx 4 ns) is deliberately NOT masked."
puts "  It is a real top-level port and its contribution is small."

# ---------------------------------------------------------------------------
puts ""
puts "############################################################"
puts "#  REAL LOGIC TIMING, PER DOMAIN, RESET REMOVED"
puts "#  Anything negative here is genuine and needs an RTL fix."
puts "############################################################"

foreach grp {clk_50mhz_sensor clk_100mhz clk_200mhz_mcu clk_10mhz_aon asynchronous} {
    puts ""
    puts "================ PATH GROUP: $grp ================"
    if {[catch {report_checks -path_delay max -path_group $grp \
                              -group_path_count 25 -slack_max 0.0 \
                              -format end -digits 3} msg]} {
        puts "   (none: $msg)"
    }
}

# ---------------------------------------------------------------------------
puts ""
puts "############################################################"
puts "#  WORST REAL PATH IN THE clk_ai / 100 MHz DOMAIN, IN FULL"
puts "#  This is where the adas / battery / motor dividers live."
puts "#  If the gate list shows a long maj3 / xnor2 / mux2 / o211ai"
puts "#  chain, that is a divider and it needs the same treatment"
puts "#  as the sensor fabric."
puts "############################################################"
catch {report_checks -path_delay max -path_group clk_100mhz \
                     -group_path_count 1 -digits 3}

# ---------------------------------------------------------------------------
puts ""
puts "############################################################"
puts "#  WORST REAL PATH, 200 MHz MCU DOMAIN, IN FULL"
puts "############################################################"
catch {report_checks -path_delay max -path_group clk_200mhz_mcu \
                     -group_path_count 1 -digits 3}

# ---------------------------------------------------------------------------
puts ""
puts "############################################################"
puts "#  TOTAL NEGATIVE SLACK, RESET REMOVED"
puts "#  Compare against tns = -23843076 from the unmasked run."
puts "#  The gap between the two is the size of the reset problem."
puts "############################################################"
report_worst_slack -max -digits 3
report_tns -digits 3
report_wns -digits 3

puts ""
puts "############################################################"
puts "#  MASKED DIAGNOSTIC COMPLETE  --  NOT A SIGNOFF RESULT"
puts "############################################################"
