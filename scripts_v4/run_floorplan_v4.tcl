#=============================================================================
# run_floorplan_v4.tcl  -  OpenROAD floorplan for IVCU-EV V4
#
#   openroad -gui
#   source scripts_v4/run_floorplan_v4.tcl
#
#-----------------------------------------------------------------------------
# WHAT WENT WRONG IN V3 AND WHAT THIS FILE DOES DIFFERENTLY
#
# Your V3 note: "in floorplan the macro pin and wires are not align and metal
# layer and pin are unconnection situation".
#
# Three separate causes, all addressed here explicitly rather than left to the
# tool's defaults:
#
#  1  PIN COUNT.  V3 had 1,344 sensor pins alone on a 5,840 um perimeter -
#     4.3 um per pin.  V4 has 237 pins.  This is fixed in the RTL, not here,
#     but it is why the rest of this file can be simple.
#
#  2  NO EXPLICIT PIN PLACEMENT.  V3 let place_pins scatter every port around
#     the die. Related signals ended up on opposite edges and the router had to
#     cross the whole core to connect a bus to its own logic.  Here every port
#     group is assigned to a named side, chosen so each group sits next to the
#     block that owns it.
#
#  3  NO MACRO HALO AND NO LAYER PLAN.  A macro with cells packed against its
#     pins has no room for the router to reach them - that is the "pin
#     unconnected" symptom.  The halo below keeps a 40 um keep-out, and the pin
#     layers are stated rather than defaulted.
#
#-----------------------------------------------------------------------------
# DIE SIZING, SHOWN AS ARITHMETIC SO YOU CAN REDO IT
#
#   standard cell area (measured)             322,640 um2
#   target utilisation                             45 %
#   core area needed for cells    322,640 / 0.45 = 717,000 um2
#
#   sram_512x32_2port footprint          ~696 x 411 = 286,000 um2
#   with a 40 um halo on all sides       ~776 x 491 = 381,000 um2
#
#   total core                    717,000 + 381,000 = 1,098,000 um2
#
#   core chosen                              1200 x 950 = 1,140,000 um2
#   die  = core + 40 um margin all round     1280 x 1030
#
#   V3 die was 1520 x 1420 = 2,158,400 um2.
#   V4 die is  1280 x 1030 = 1,318,400 um2  -  39 % smaller.
#
# The width is no longer forced by two macros side by side, because there is
# only one macro now.
#=============================================================================

set TOP      ivcu_ev_v4_top
set NETLIST  [lindex [lsort [glob synth_out_v4/${TOP}_netlist_*.v]] end]

puts "\n=== floorplan: $NETLIST ===\n"

#-----------------------------------------------------------------------------
# 1  TECHNOLOGY AND DESIGN
#-----------------------------------------------------------------------------
read_lef  lef/sky130_fd_sc_hd__nom.tlef
read_lef  lef/sky130_fd_sc_hd.lef
read_lef  macros/sram_512x32_2port.lef

read_liberty libs/sky130_fd_sc_hd__tt_025C_1v80.lib
read_liberty macros/sram_512x32_2port_TT_1p8V_25C.lib

read_verilog $NETLIST
link_design  $TOP

read_sdc SDC/ivcu_ev_v4.sdc

#-----------------------------------------------------------------------------
# 2  FLOORPLAN
#
# site unithd is the Sky130 HD standard cell site.  The die and core numbers
# come from the arithmetic in the header.
#-----------------------------------------------------------------------------
initialize_floorplan \
    -die_area  "0 0 1280 1030" \
    -core_area "40 40 1240 990" \
    -site      unithd

#-----------------------------------------------------------------------------
# 3  THE SRAM MACRO
#
# Placed in the top-right corner, away from the MCU pins on the south edge and
# clear of the sensor pins on the north.  The fault logger that talks to it
# lives in the clk_mcu domain, so the placer will pull that logic toward it.
#
# THE HALO IS THE POINT.  40 um of keep-out on every side means no standard
# cell can be placed against the macro's pin edge, so the detailed router
# always has space to reach those pins.  Without it, cells pack right up
# against the macro and the router cannot get in - which is exactly the
# "macro pin unconnected" state V3 ended in.
#-----------------------------------------------------------------------------
place_macro -macro_name u_sram \
            -location {480 520} \
            -orientation R0

set_macro_halo -macro u_sram -halo_x 40 -halo_y 40

#-----------------------------------------------------------------------------
# 4  PIN PLACEMENT - BY FUNCTION, NOT BY WHATEVER THE TOOL CHOOSES
#
# Each group goes on the edge nearest the block that owns it, so the router is
# never asked to cross the die to connect a bus to its own logic.
#
#   NORTH  sensor acquisition   27 pins over 1200 um  = 44 um pitch
#   SOUTH  MCU / APB            83 pins over 1200 um  = 14 um pitch
#   EAST   HV + powertrain      94 pins over  950 um  = 10 um pitch
#   WEST   clocks, discretes,
#          telematics, status   33 pins over  950 um  = 29 um pitch
#
# The tightest is 10 um per pin on the east edge.  Sky130 met3/met4 pitch is
# under 1 um, so even that is ten times looser than the technology needs.
#-----------------------------------------------------------------------------

# --- NORTH: everything that talks to the ADC and the analog front ends ------
set_io_pin_constraint -region top:* -pin_names {
    adc_chan* adc_req adc_data* adc_valid adc_busy
    afe_sclk afe_sdata afe_latch
}

# --- SOUTH: the MCU register interface --------------------------------------
set_io_pin_constraint -region bottom:* -pin_names {
    paddr* psel penable pwrite pwdata* prdata* pready pslverr
    disp_sclk disp_sdata disp_cs
}

# --- EAST: high voltage, powertrain, safety actuators -----------------------
# These are the highest-current, most safety-critical outputs.  Grouping them
# on one edge keeps their board traces short and keeps the HV drive signals
# physically away from the MCU bus on the opposite side.
set_io_pin_constraint -region right:* -pin_names {
    hv_contactor_pos_en hv_contactor_neg_en
    hv_precharge_en hv_discharge_en
    pyro_fuse_arm pyro_fuse_fire
    hv_isolated hv_state* hv_fault_code*
    torque_cmd* regen_cmd* motor_enable
    power_derate_pct* cooling_pump_pwm* cooling_fan_pwm* charge_enable
    airbag_trigger* belt_pretension* door_unlock
    horn_en headlight_en hazard_lights_en
}

# --- WEST: clocks, reset, the asynchronous discretes, telematics, status ----
set_io_pin_constraint -region left:* -pin_names {
    clk_aon clk_sensor clk_ai clk_mcu por_n ext_rst_n
    crash_trig_front crash_trig_side hvil_raw
    ignition_on permit_ack hazard_button mode_strap*
    gps_rx_data* gps_rx_valid sos_tx_data* sos_tx_valid sos_tx_ready
    system_health_score* vehicle_enable limp_home_active
    speed_limit_kph* safety_state* warn_latched
}

#-----------------------------------------------------------------------------
# 5  PLACE THE PINS
#
# Sky130 layer directions: met1 hor, met2 ver, met3 hor, met4 ver, met5 hor.
# Pins on the left and right edges need a HORIZONTAL layer to escape inward;
# pins on top and bottom need a VERTICAL one.
#
# met3/met4 rather than met1/met2, so the pin escapes do not compete with the
# dense local routing the placer will put on the lower layers.
#-----------------------------------------------------------------------------
place_pins -hor_layers met3 \
           -ver_layers met4 \
           -corner_avoidance 40 \
           -min_distance 4

#-----------------------------------------------------------------------------
# 6  CHECKS BEFORE GOING ANY FURTHER
#-----------------------------------------------------------------------------
puts "\n=============================================================="
puts " FLOORPLAN CHECKS"
puts "=============================================================="

puts "\n-- die and core --"
report_design_area

puts "\n-- unplaced instances (should be only standard cells) --"
set unplaced 0
foreach inst [get_cells *] {
  if {![$inst isPlaced] && [[$inst getMaster] isBlock]} {
    puts "  UNPLACED MACRO: [$inst getName]"
    incr unplaced
  }
}
if {$unplaced == 0} { puts "  all macros placed" }

puts "\n-- pin placement --"
puts "  If any pin failed to place, place_pins would have errored above."
puts "  Open the GUI and look at the four edges: each group should be"
puts "  contiguous on its own side.  V3's failure mode was pins from the"
puts "  same bus scattered across opposite edges."

write_def floorplan_v4/${TOP}_floorplan.def
puts "\n  written: floorplan_v4/${TOP}_floorplan.def"

puts "\n=============================================================="
puts " NEXT"
puts "=============================================================="
puts " global_placement, then:"
puts ""
puts "   estimate_parasitics -placement"
puts "   repair_design"
puts ""
puts " repair_design is what fixes the two remaining STA violations:"
puts "   _70531_/Q  clk_sensor reset, 3321 loads, 73.979 ns"
puts "   paddr\[2\]   APB address decode, 9.708 ns through one nor2"
puts ""
puts " Both are unbuffered high-fanout nets.  repair_design inserts the"
puts " buffer trees.  Re-run STA after it and expect both to close."
puts " If either does NOT close after repair_design, that is the point"
puts " to come back and look at the RTL - not before."
