# ============================================================================
# run_sta_diag.tcl  --  DIAGNOSTIC pre-layout STA, IVCU-EV V3
#
#   cd ~/final_ivcu_project
#   mkdir -p sta_out
#   sta -no_splash -exit scripts/run_sta_diag.tcl 2>&1 | tee sta_out/sta_diag.txt
#
# Runs on the netlist you ALREADY HAVE. Minutes, not hours. No re-synthesis.
#
# WHY THIS EXISTS
#   The first run used `report_checks -group_count 10`, which returns the ten
#   worst paths in the whole design. All ten were consumed by the unbuffered
#   reset nets (fanout 15372 / 1317 / 911 / 347), because a 1136 ns fake gate
#   delay beats every real violation by two orders of magnitude.
#
#   Anything genuinely broken UNDERNEATH those paths was invisible. We already
#   know at least six variable dividers exist on clk_ai (10 ns period, tighter
#   than the sensor domain that was already failing) and none of them appeared.
#
#   This script reports per PATH GROUP, so each clock domain gets its own quota
#   and the reset paths can only crowd out their own domain. It also uses
#   `-format end`, which prints one line per path instead of a full path dump,
#   so 40 paths per group stays readable.
#
# READ THE OUTPUT LIKE THIS
#   Ignore anything whose endpoint is in u_reset_sync or whose slack is worse
#   than about -50 ns -- those are the reset-fanout artifacts and OpenROAD's
#   repair_design fixes them by inserting buffer trees.
#   What matters is everything in the -1 ns to -40 ns band. Those are real.
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
puts "#  1. WORST SLACK PER CLOCK DOMAIN"
puts "#     One number per domain. Tells you which domains are"
puts "#     actually in trouble before you read any path detail."
puts "############################################################"

foreach grp {clk_10mhz_aon clk_50mhz_sensor clk_100mhz clk_200mhz_mcu asynchronous} {
    puts ""
    puts "---- $grp ----"
    if {[catch {report_checks -path_delay max -path_group $grp \
                              -group_path_count 1 -format end -digits 3} msg]} {
        puts "   (no such path group, or none found: $msg)"
    }
}

# ---------------------------------------------------------------------------
puts ""
puts "############################################################"
puts "#  2. VIOLATING ENDPOINTS PER DOMAIN  (up to 40 each)"
puts "#     One line per path. Look at WHICH MODULE the endpoint"
puts "#     is in -- that is what tells you where the work is."
puts "############################################################"

foreach grp {clk_50mhz_sensor clk_100mhz clk_200mhz_mcu clk_10mhz_aon asynchronous} {
    puts ""
    puts "================ PATH GROUP: $grp ================"
    if {[catch {report_checks -path_delay max -path_group $grp \
                              -group_path_count 40 -slack_max 0.0 \
                              -format end -digits 3} msg]} {
        puts "   (none: $msg)"
    }
}

# ---------------------------------------------------------------------------
puts ""
puts "############################################################"
puts "#  3. THE ONE PATH THAT MATTERS PER DOMAIN, IN FULL"
puts "#     Full gate-by-gate detail for the single worst path in"
puts "#     each domain, so we can see what logic is actually on it."
puts "############################################################"

foreach grp {clk_50mhz_sensor clk_100mhz clk_200mhz_mcu} {
    puts ""
    puts "================ WORST PATH, $grp ================"
    catch {report_checks -path_delay max -path_group $grp \
                         -group_path_count 1 -digits 3}
}

# ---------------------------------------------------------------------------
puts ""
puts "############################################################"
puts "#  4. HIGH-FANOUT NETS  (the reset problem, quantified)"
puts "#     Expect exactly the four reset nets plus por_n. If any"
puts "#     OTHER net shows up with fanout in the hundreds, that"
puts "#     is new information and worth knowing about."
puts "############################################################"
report_check_types -max_fanout -violators

# ---------------------------------------------------------------------------
puts ""
puts "############################################################"
puts "#  5. POWER  --  sanity-check the units"
puts "#     The macro row previously read 8.121e+06 W, i.e. 8"
puts "#     megawatts, which is a unit declaration bug in the"
puts "#     OpenRAM liberty, not a real number. Confirm with:"
puts "#        grep -iE 'power_unit|leakage_power_unit' \\"
puts "#          macros/sram_512x32_2port_TT_1p8V_25C.lib"
puts "#     Standard-cell rows are the trustworthy ones for now."
puts "############################################################"
report_power -digits 3

puts ""
puts "############################################################"
puts "#  DIAGNOSTIC STA COMPLETE"
puts "############################################################"
