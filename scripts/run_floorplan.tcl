# =====================================================================
#  IVCU-EV V3  --  FLOORPLAN  (version 2)
# =====================================================================
#
#  Run with:   openroad -gui
#              source scripts/run_floorplan.tcl
#
#  WHAT CHANGED FROM VERSION 1
#  ---------------------------
#  Only one number: the macro y coordinate, 987.360 -> 995.520.
#  That is +8.16 um, which is exactly 3 cell rows (3 x 2.72).
#
#  WHY.  In version 1 the macros sat 10.33 um below the core top.
#  That gap was too small to power but too big to be empty: three
#  full-width cell rows (511, 512, 513) lived in it.  They held only
#  tapcells, yet pdngen still had to feed them, and it could not:
#
#      space above macro halo   1400.63 -> 1408.96  =   8.33 um
#      met5 VDD-to-VSS spacing        27.2 / 2      =  13.60 um
#
#  Only one of the two nets fits.  The other is always orphaned.
#  Proved twice: shifting the met5 offset just swapped which net
#  failed (VDD -> VSS).  Extending the macro halo to cover the rows
#  is refused outright:
#
#      [ERROR PDN-0008] halo overlaps row ROW_511 (and 2 other row(s))
#
#  Moving the macros up 3 rows makes rows 511-513 overlap the macros
#  in y, so they get CUT in x like every other row.  What is left are
#  narrow segments in the left margin, the 60 um channel and the right
#  margin -- and met4 straps run the full core height there, uncut, so
#  those segments connect normally.
#
#  Side effect, all good: the cell band grows 8.13 um and utilisation
#  improves from 44.2% to about 43.8% of a slightly larger band.
#
#  y = 995.520 is the HIGHEST legal position.  One row higher (998.24)
#  puts the macro top at 1409.475, outside the core.
#
# =====================================================================


# ---------------------------------------------------------------------
#  1.  LIBRARIES
# ---------------------------------------------------------------------
#  Tech LEF MUST be first - it defines the layers and sites that the
#  cell LEFs reference.
#
#  NOTE: macros/sram_512x32_2port.lef must say "DATABASE MICRONS 1000".
#  OpenRAM wrote 2000 and OpenROAD silently discards the whole file.
#  Original kept as macros/sram_512x32_2port.lef.units2000

read_lef  lef/sky130_fd_sc_hd__nom.tlef
read_lef  lef/sky130_fd_sc_hd.lef
read_lef  macros/sram_512x32_2port.lef

read_liberty libs/sky130_fd_sc_hd__tt_025C_1v80.lib
read_liberty macros/sram_512x32_2port_TT_1p8V_25C.lib


# ---------------------------------------------------------------------
#  2.  NETLIST
# ---------------------------------------------------------------------
#  Both files are needed.  The second one is the SRAM wrapper glue,
#  synthesised in a separate Yosys run (72 cells, 786 um2).

read_verilog synth_out/ivcu_ev_v3_hybrid_top_gate_full.v
read_verilog synth_out/fault_log_sram_1024x32_netlist.v

link_design ivcu_ev_v3_hybrid_top

read_sdc SDC/ivcu_ev_v3.sdc


# ---------------------------------------------------------------------
#  3.  DIE AND CORE
# ---------------------------------------------------------------------
#  die   1520 x 1420   the silicon you pay for
#  core  1500 x 1400   where cell rows are created
#  the 10 um ring between them holds the I/O pins
#
#  Width was forced by the macros:
#      24.00 margin + 696.02 macro + 60.00 channel
#            + 696.02 macro + 24.00 margin  =  1500.04
#
#  Height came from utilisation:
#      634,059 um2 of cells / 0.44  =  1,441,043 um2
#      1,441,043 / 1500             =    960.7 um cell band
#      + 441.2 macro band           =   1401.9  -> 1400
#
#  The tool will snap the core to (10.120, 10.880) because 10.000 is
#  not a whole number of sites (0.46) or rows (2.72).  Expect
#  [WARNING IFP-0028].  That is the tool correcting an impossible
#  request, not an error.

initialize_floorplan -die_area  {0 0 1520 1420} \
                     -core_area {10 10 1510 1410} \
                     -site      unithd


# ---------------------------------------------------------------------
#  4.  ROUTING TRACKS
# ---------------------------------------------------------------------
#  initialize_floorplan made the ROWS (where cells go).
#  make_tracks makes the TRACKS (where wires go).  Separate grids.
#
#  li1  VERT 0.46   met1 HORIZ 0.34   met2 VERT 0.46
#  met3 HORIZ 0.68  met4 VERT 0.92    met5 HORIZ 3.40
#
#  place_macro fails with [ERROR MPL-0039] without this, because it
#  snaps macros so their pins land on tracks.
#  Verify afterwards: [llength [[ord::get_db_block] getTrackGrids]] = 6

make_tracks


# ---------------------------------------------------------------------
#  5.  MACRO PLACEMENT           <-- THE ONE CHANGE IN VERSION 2
# ---------------------------------------------------------------------
#  X, from the snapped core left edge 10.120:
#      u_bank_lo   10.120 + (52   x 0.46) =  34.040
#      u_bank_hi   10.120 + (1696 x 0.46) = 790.280
#      channel     790.280 - (34.040 + 696.02) = 60.22 um
#
#  Y, both banks:
#      10.880 + (362 x 2.72) = 995.520      <-- was 987.360 in v1
#      macro top = 995.520 + 411.235 = 1406.755
#      gap to core top 1408.96 = 2.205 um   (was 10.33)
#
#  R0 for both.  Checked, not assumed: the pins span the macro's whole
#  face (y 0 to 411.235), so orientation does not change pin access.
#
#  Expect the tool to nudge each by ~0.27 um in x and ~0.035 in y to
#  hit the track grid, and to report both as LOCKED.
#
#  Expect [WARNING MPL-0002] "2 out of 103 pins were aligned" and
#  "1 out of 14".  Only 3 of 117 macro pins land on a routing track.
#  This is permanent: OpenRAM laid the pins on its own grid, which is
#  not a multiple of sky130's met3 (0.68) or met4 (0.92) pitch.  A
#  macro has one position; you can align two pins, never all of them.
#  If detailed routing later reports DRC clustered at the macro edges,
#  THIS is the first thing to come back to.

place_macro -macro_name u_fault_logger/u_fault_log_sram/u_bank_lo \
            -location {34.040 995.520} -orientation R0

place_macro -macro_name u_fault_logger/u_fault_log_sram/u_bank_hi \
            -location {790.280 995.520} -orientation R0


# ---------------------------------------------------------------------
#  6.  PLACEMENT BLOCKAGE
# ---------------------------------------------------------------------
#  A hard keep-out: no standard cell above (macro bottom - 20 um).
#
#  The 20 um strip below the macros is the important part.  That edge
#  faces all 69,771 standard cells, and every one of the macro's 117
#  signal pins must get a wire down into the cell band.  Cells flush
#  against it leave the router no room, and 114 of those pins already
#  need an off-track jog.  Both problems land in the same place.
#
#  Computed from the ACTUAL macro position, not the requested one,
#  because the tool nudges by a few hundredths of a micron.
#
#  We use create_blockage rather than set_macro_halo because
#  set_macro_halo belongs to the macro placer, which never runs here
#  (our macros are LOCKED).  A blockage is unconditional and can be
#  read back and confirmed.

set _blk [[ord::get_db_block] findInst \
          u_fault_logger/u_fault_log_sram/u_bank_lo]
set _bb  [$_blk getBBox]
set _y   [expr {[$_bb yMin] / 1000.0 - 20.0}]
puts "INFO: macro bottom [expr {[$_bb yMin]/1000.0}] um, blockage at $_y um"

create_blockage -region "10.12 $_y 1509.72 1408.96"


# ---------------------------------------------------------------------
#  7.  I/O PIN CONSTRAINTS
# ---------------------------------------------------------------------
#  Region syntax is  edge:from-to  in microns, or  edge:*  for a whole
#  edge.
#
#  Buses grouped by edge so their wires do not cross the whole core.
#  504 ADC + 42 valid on the left, 1344 digital + 42 valid on the
#  bottom, 440 AXI on the right.  The remaining 347 are left free so
#  the tool can minimise wirelength.
#
#  The CLOCK placements are the point of the exercise.  A clock
#  entering at a corner forces its tree to reach diagonally across the
#  whole die; entering at an edge centre halves the worst distance.
#  Each one sits nearest the logic it feeds.

set_io_pin_constraint -pin_names {sensor_adc_in_*}        -region left:*
set_io_pin_constraint -pin_names {sensor_adc_valid_*}     -region left:*
set_io_pin_constraint -pin_names {sensor_digital_in_*}    -region bottom:*
set_io_pin_constraint -pin_names {sensor_digital_valid_*} -region bottom:*
set_io_pin_constraint -pin_names {m_axi_* s_axi_*}        -region right:*

set_io_pin_constraint -pin_names {clk_50mhz_sensor} -region left:600-820
set_io_pin_constraint -pin_names {clk_200mhz_mcu}   -region right:650-770
set_io_pin_constraint -pin_names {clk_100mhz}       -region bottom:700-820
set_io_pin_constraint -pin_names {clk_10mhz_aon}    -region top:700-820


# ---------------------------------------------------------------------
#  8.  PIN PLACEMENT
# ---------------------------------------------------------------------
#  -hor_layers met3   left and right edges.  A left-edge pin must be
#                     reachable by a wire travelling left-to-right,
#                     which needs a HORIZONTAL layer.  met3 is
#                     horizontal.
#  -ver_layers met2   top and bottom edges.  met2 is vertical.
#  -corner_avoidance  keep 50 um clear of each corner.  A corner pin
#                     can only leave in one direction.
#  -annealing         the slower, better optimiser.  Worth it for
#                     2,719 pins.  THIS IS THE SLOW STEP - allow a
#                     few minutes.
#
#  Expect "Number of I/O w/o sink 1394".  Not a new bug: 864 are
#  unused upper bits of sensor_digital_in, 390 are ADC channels 12-41
#  which the RTL never processes, 32 are fault_log_rd_data.  Logged in
#  WHERE_WE_ARE.md, to be fixed in one RTL batch.

place_pins -hor_layers met3 -ver_layers met2 \
           -corner_avoidance 50 -annealing


# ---------------------------------------------------------------------
#  9.  TAP CELLS AND END CAPS
# ---------------------------------------------------------------------
#  Not a quality knob - a DRC requirement.
#
#  CMOS has a parasitic thyristor under every cell, formed by the
#  substrate, the n-well and the diffusions.  If the well or substrate
#  voltage drifts, it can turn on and LATCH: a low-resistance path from
#  VDD to ground that stays open until power is removed.  The chip
#  usually destroys itself.
#
#  Tap cells tie substrate to VGND and n-well to VPWR at regular
#  intervals so neither can drift.  They contain no logic.
#  End caps close off the ends of each row so the wells terminate
#  cleanly.
#
#  -distance 13 is a RADIUS, not a spacing.  Max 13 um from any point
#  to the nearest tap, so taps sit every 26 um:
#      556,417 um of row length / 26 = 21,400   (v1 inserted 21,630)
#  Reading it as a spacing predicts ~42,800, which is wrong.
#
#  Expect the row count to change from v1's 820 segments, because the
#  macros moved up 3 rows.

tapcell -distance 13 \
        -tapcell_master sky130_fd_sc_hd__tapvpwrvgnd_1 \
        -endcap_master  sky130_fd_sc_hd__decap_3


# ---------------------------------------------------------------------
#  10.  POWER PIN CONNECTIONS
# ---------------------------------------------------------------------
#  Gate-level Verilog omits power by convention, so the netlist has no
#  power connections at all.  Every cell in the LEF does have them:
#      VPWR / VGND   the supply rails
#      VPB  / VNB    the well taps (p-well bulk, n-well bulk)
#  and the SRAMs have vccd1 / vssd1.
#
#  Each line means: any pin with this name, on any instance, belongs to
#  this net.  -power and -ground mark which is which.  -inst_pattern
#  restricts a rule, needed for the macros because their pins are
#  lowercase and differently named.
#
#  The arithmetic checks itself:
#      VPWR 93,041   VPB 71,411    difference 21,630 = the tapcells,
#      VGND 93,041   VNB 71,411    which ARE the well tap and so have
#                                  no VPB/VNB of their own.
#      93,041 + 71,411 + 2 = 164,454 on each net.
#  Those numbers will shift slightly in v2 - fewer rows above the
#  macros means fewer tapcells.  The RELATIONSHIP must still hold.
#
#  Without the two macro lines everything routes, everything times,
#  and the design fails LVS at the very end because the memories have
#  no supply.

add_global_connection -net VDD -pin_pattern {^VPWR$} -power
add_global_connection -net VDD -pin_pattern {^VPB$}
add_global_connection -net VSS -pin_pattern {^VGND$} -ground
add_global_connection -net VSS -pin_pattern {^VNB$}

add_global_connection -net VDD -inst_pattern {.*u_bank.*} \
                      -pin_pattern {^vccd1$} -power
add_global_connection -net VSS -inst_pattern {.*u_bank.*} \
                      -pin_pattern {^vssd1$} -ground

global_connect


# ---------------------------------------------------------------------
#  11.  REPORT AND CHECKPOINT
# ---------------------------------------------------------------------

puts "====================== FLOORPLAN v2 ======================"

foreach n {VDD VSS} {
  set net [[ord::get_db_block] findNet $n]
  puts "$n : [expr {$net eq "NULL" ? "MISSING" \
        : "[llength [$net getITerms]] pins"}]"
}

puts "track grids : [llength [[ord::get_db_block] getTrackGrids]]  (want 6)"
puts "rows        : [llength [[ord::get_db_block] getRows]]"
puts "instances   : [llength [[ord::get_db_block] getInsts]]"

# The check that matters for this rebuild: no cell row may sit
# entirely above the macros.  If this prints anything, the macro y is
# still too low and pdngen will fail with PDN-0178 again.
set _top [[[[ord::get_db_block] findInst \
            u_fault_logger/u_fault_log_sram/u_bank_lo] getBBox] yMax]
set _bad 0
foreach r [[ord::get_db_block] getRows] {
  if { [[$r getBBox] yMin] >= $_top } {
    puts "ORPHAN ROW: [$r getName]"
    incr _bad
  }
}
puts "orphan rows above macros : $_bad   (want 0)"
puts "=========================================================="

write_db  pnr_out/ivcu_floorplan_v2.odb
write_def pnr_out/ivcu_floorplan_v2.def

puts "checkpoint written: pnr_out/ivcu_floorplan_v2.odb"
