#=============================================================================
# run_sta_v4.tcl  -  OpenSTA timing analysis for IVCU-EV V4
#
# Invoked by scripts_v4/run_sta_v4.sh, which finds the newest netlist and sets
# the NETLIST environment variable.  Run it directly with:
#     NETLIST=synth_out_v4/<file>.v sta -no_splash -exit scripts_v4/run_sta_v4.tcl
#
#-----------------------------------------------------------------------------
# WHAT TO LOOK FOR
#
# V3 reported WNS -1.389 ns and TNS -48.323 ns against 200 MHz on the MCU
# domain and 100 MHz on the AI domain.  Two things changed:
#
#   1  the targets came down to 50 / 50 / 25 / 10 MHz, because Sky130 HD does
#      not comfortably run general logic at 200 MHz and that number came from
#      a 14 nm mindset
#   2  seq_divider is deleted.  RULE R5 forbids '/' anywhere in the design and
#      the area probe confirms no $div cell was inferred by any other route
#
# If WNS is still negative here, the interesting question is WHICH domain and
# WHICH path - not whether to relax the clock again.  The per-clock breakdown
# below is there to answer that.
#=============================================================================

#-----------------------------------------------------------------------------
# Libraries.  The SRAM macro needs its own liberty or every path into and out
# of it is unconstrained and silently ignored.
#-----------------------------------------------------------------------------
read_liberty libs/sky130_fd_sc_hd__tt_025C_1v80.lib
read_liberty macros/sram_512x32_2port_TT_1p8V_25C.lib

#-----------------------------------------------------------------------------
# Design
#-----------------------------------------------------------------------------
set nl $env(NETLIST)
puts "\n=== netlist: $nl ===\n"
read_verilog $nl
link_design ivcu_ev_v4_top

read_sdc SDC/ivcu_ev_v4.sdc

puts "\n=============================================================="
puts " CLOCKS AS THE TOOL SEES THEM"
puts "=============================================================="
report_clock_properties

puts "\n=============================================================="
puts " SETUP - worst path per clock group"
puts "=============================================================="
report_checks -path_delay max -group_count 5 -endpoint_count 1 \
              -digits 3 -format full_clock_expanded

puts "\n=============================================================="
puts " HOLD - worst path per clock group"
puts "=============================================================="
report_checks -path_delay min -group_count 5 -endpoint_count 1 \
              -digits 3

puts "\n=============================================================="
puts " SLACK SUMMARY"
puts "=============================================================="
puts "\n-- setup --"
report_wns -digits 3
report_tns -digits 3

puts "\n-- hold --"
report_worst_slack -min -digits 3

puts "\n=============================================================="
puts " SLACK PER CLOCK - THE NUMBER THAT ACTUALLY MEANS SOMETHING"
puts "=============================================================="
puts " A single unbuffered high-fanout net can dominate WNS and TNS"
puts " for the whole design and hide the state of every real path."
puts " On the first V4 run one reset net with 3,321 loads produced"
puts " WNS -61.6 ns while clk_ai was sitting at +7.9 ns of margin."
puts ""
puts " Per-clock worst slack, functional paths only:"
foreach clk {clk_aon clk_sensor clk_ai clk_mcu} {
  puts "\n---- $clk ----"
  report_checks -path_delay max -to [get_clocks $clk] \
                -group_count 1 -endpoint_count 1 -digits 3 -slack_max 1e30
}

puts "\n=============================================================="
puts " HIGH FANOUT NETS"
puts "=============================================================="
puts " These are what P&R has to buffer.  Anything over a few hundred"
puts " loads will look catastrophic in this report and be fine after"
puts " repair_design and CTS have built a tree for it."
report_check_types -max_fanout -digits 3

puts "\n=============================================================="
puts " DESIGN RULE VIOLATIONS"
puts "=============================================================="
# OpenSTA calls this -max_slew, not -max_transition.  The SDC command is
# set_max_transition; the report flag is -max_slew.  They are the same check.
puts "\n-- max slew (set_max_transition 0.4 ns) --"
report_check_types -max_slew -digits 3

puts "\n-- max capacitance (limit 0.2 pF) --"
report_check_types -max_capacitance -digits 3

puts "\n-- max fanout (limit 32) --"
report_check_types -max_fanout -digits 3

puts "\n=============================================================="
puts " UNCONSTRAINED PATHS"
puts "=============================================================="
puts " Anything listed here is a path the tool was NOT asked to time."
puts " In V3, unconstrained paths were where the surprises lived - a"
puts " path that is not timed is not fast, it is just unexamined."
report_checks -unconstrained -digits 3 -endpoint_count 20

# report_design_area does not exist in this OpenSTA build - it is an OpenROAD
# command, not an OpenSTA one.  Area comes from the Yosys stat, which is where
# it was measured: 322,160 um2 of standard cells.

puts "\n=== STA complete ===\n"
