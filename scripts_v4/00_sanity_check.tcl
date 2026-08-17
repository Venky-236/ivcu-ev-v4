#=============================================================================
# 00_sanity_check.tcl  -  STAGE 0 of physical design
#
#   openroad -exit scripts_v4/00_sanity_check.tcl 2>&1 | tee pd_v4/sanity.log
#
#-----------------------------------------------------------------------------
# WHAT THIS STAGE IS
#
# Physical design has seven stages and every one of them assumes its inputs are
# consistent.  Nothing in the flow checks that for you.  V3 reached floorplan
# with inputs nobody had verified and the symptoms - "pins and wires are not
# align", "macro pin unconnected", "metal layer unconnection" - all appeared at
# the stage that could no longer tolerate them, not the stage that caused them.
#
# This file asks six questions.  It does not fix anything.  It prints numbers
# and lets you judge them.
#
#   A  do the files exist and do the LEFs agree on units
#   B  does the netlist link, with a physical master for every instance
#   C  does every instance also have TIMING, not just geometry
#   D  is the SRAM macro physically usable, and which edge are its pins on
#   E  do the constraints read, and is every clock defined
#   F  is the die size we are about to choose actually feasible
#
# Run it, read it, then go to stage 1.
#=============================================================================

set TOP ivcu_ev_v4_top

file mkdir pd_v4

puts "\n"
puts "=============================================================="
puts " STAGE 0  -  SANITY CHECK"
puts " design: $TOP"
puts "=============================================================="

#=============================================================================
# A.  FILE INVENTORY AND UNIT CONSISTENCY
#
# WHY: LEF coordinates are integers in database units.  DATABASE MICRONS says
# how many of those units make a micron.  Sky130 tech LEF uses 1000.  OpenRAM
# sometimes emits 2000.  Mix them and every macro coordinate is wrong by a
# factor of two - the macro lands in the wrong place, or half outside the die,
# and the error surfaces as unroutable pins rather than as a units message.
#
# The file macros/sram_512x32_2port.lef.units2000 in this project is evidence
# that this already happened once here.
#=============================================================================
puts "\n--------------------------------------------------------------"
puts " A.  FILES AND UNITS"
puts "--------------------------------------------------------------"

set files [list \
    lef/sky130_fd_sc_hd__nom.tlef \
    lef/sky130_fd_sc_hd.lef \
    macros/sram_512x32_2port.lef \
    libs/sky130_fd_sc_hd__tt_025C_1v80.lib \
    macros/sram_512x32_2port_TT_1p8V_25C.lib \
    SDC/ivcu_ev_v4.sdc ]

set missing 0
foreach f $files {
  if {[file exists $f]} {
    puts [format "  OK      %-52s %8d bytes" $f [file size $f]]
  } else {
    puts [format "  MISSING %s" $f]
    incr missing
  }
}

# newest netlist
set nl_list [lsort [glob -nocomplain synth_out_v4/${TOP}_netlist_*.v]]
if {[llength $nl_list] == 0} {
  puts "  MISSING synth_out_v4/${TOP}_netlist_*.v"
  incr missing
  exit 1
}
set NETLIST [lindex $nl_list end]
puts [format "  OK      %-52s %8d bytes" $NETLIST [file size $NETLIST]]

if {$missing > 0} {
  puts "\n  *** $missing input(s) missing.  Stop here."
  exit 1
}

# --- units, read straight out of the LEF headers ---------------------------
puts "\n  DATABASE MICRONS declared in each LEF:"
foreach f {lef/sky130_fd_sc_hd__nom.tlef lef/sky130_fd_sc_hd.lef \
           macros/sram_512x32_2port.lef} {
  set fh [open $f r]
  set units "not declared"
  while {[gets $fh line] >= 0} {
    if {[regexp {DATABASE\s+MICRONS\s+([0-9]+)} $line -> u]} { set units $u ; break }
    if {[regexp {^MACRO }  $line]} { break }
  }
  close $fh
  puts [format "    %-46s %s" [file tail $f] $units]
}
puts "\n  These MUST all be the same number.  If one says 2000 and another"
puts "  says 1000, stop and convert before doing anything else."

#=============================================================================
# B.  LOAD THE TECHNOLOGY AND LINK THE DESIGN
#
# WHY: linking is where every cell name in the netlist is resolved against a
# physical master in the LEF.  An unresolved reference is an instance with no
# size, no pins and no legal location.  Better to see it now than to have the
# placer discover it.
#
# ORDER MATTERS: technology LEF first (it defines layers, vias and sites),
# then cell LEF, then macro LEF.  A macro LEF read before the tech LEF has no
# layers to refer to.
#=============================================================================
puts "\n--------------------------------------------------------------"
puts " B.  LINK"
puts "--------------------------------------------------------------"

read_lef lef/sky130_fd_sc_hd__nom.tlef
read_lef lef/sky130_fd_sc_hd.lef
read_lef macros/sram_512x32_2port.lef

read_liberty libs/sky130_fd_sc_hd__tt_025C_1v80.lib
read_liberty macros/sram_512x32_2port_TT_1p8V_25C.lib

read_verilog $NETLIST
link_design  $TOP

puts "  linked."

set block [ord::get_db_block]
set insts [$block getInsts]
puts [format "  instances in netlist: %d" [llength $insts]]

#=============================================================================
# C.  GEOMETRY WITHOUT TIMING
#
# WHY: LEF gives a cell its shape.  Liberty gives it delay.  A cell that is in
# the LEF but not in the Liberty will place and route perfectly and report
# ZERO delay through it - a hole in static timing that no STA run will flag,
# because from the tool's point of view there is nothing there to time.
#
# This is the physical-design cousin of the V3 defect where sensor_fault could
# never assert: the check exists, it runs, and it cannot fail.
#=============================================================================
puts "\n--------------------------------------------------------------"
puts " C.  CELLS WITH GEOMETRY BUT NO TIMING"
puts "--------------------------------------------------------------"

set no_timing [dict create]
foreach inst $insts {
  set master [$inst getMaster]
  set mname  [$master getName]
  if {[dict exists $no_timing $mname]} { continue }
  # a liberty cell lookup that returns nothing means no timing model
  if {[get_lib_cells -quiet */$mname] eq ""} {
    dict set no_timing $mname 1
  }
}

if {[dict size $no_timing] == 0} {
  puts "  none - every instance has both geometry and timing"
} else {
  puts "  *** these masters have LEF but no Liberty:"
  foreach m [dict keys $no_timing] { puts "        $m" }
  puts "  *** every path through them is untimed."
}

#=============================================================================
# D.  MACRO READINESS
#
# WHY: a hard macro is not just a big rectangle.  Three things decide whether
# it can be used:
#
#   SIZE       drives the die arithmetic in section F
#   CLASS      must be BLOCK, or the placer treats it as a standard cell
#   PIN EDGE   which face the pins are on decides orientation and halo
#
# The last one is the one that gets missed.  This macro has all of its pins on
# ONE edge, on met4.  If that edge is turned toward a die boundary, or toward
# a region the placer has filled solid, the router has no way in and you get
# "macro pin unconnected" - with nothing in the log explaining why.
#=============================================================================
puts "\n--------------------------------------------------------------"
puts " D.  MACRO"
puts "--------------------------------------------------------------"

foreach inst $insts {
  set master [$inst getMaster]
  if {![$master isBlock]} { continue }

  set w [expr {[$master getWidth]  / 1000.0}]
  set h [expr {[$master getHeight] / 1000.0}]

  puts [format "  instance : %s" [$inst getName]]
  puts [format "  master   : %s" [$master getName]]
  puts [format "  size     : %.2f x %.2f um   = %.0f um2" $w $h [expr {$w*$h}]]
  puts [format "  class    : %s" [$master getType]]

  # which edges carry pins, and on what layers
  set edge_s 0 ; set edge_n 0 ; set edge_w 0 ; set edge_e 0
  set layers [dict create]
  set npins 0
  foreach mterm [$master getMTerms] {
    foreach mpin [$mterm getMPins] {
      foreach box [$mpin getGeometry] {
        incr npins
        dict set layers [[$box getTechLayer] getName] 1
        set y1 [expr {[$box yMin]/1000.0}]
        set y2 [expr {[$box yMax]/1000.0}]
        set x1 [expr {[$box xMin]/1000.0}]
        set x2 [expr {[$box xMax]/1000.0}]
        if {$y1 < 1.0}       { incr edge_s }
        if {$y2 > $h - 1.0}  { incr edge_n }
        if {$x1 < 1.0}       { incr edge_w }
        if {$x2 > $w - 1.0}  { incr edge_e }
      }
    }
  }
  puts [format "  pin shapes: %d   on layers: %s" $npins [dict keys $layers]]
  puts [format "  pins touching each edge:  S %d   N %d   W %d   E %d" \
        $edge_s $edge_n $edge_w $edge_e]
  puts ""
  puts "  READ THAT LINE.  The edge with the large number is the face the"
  puts "  router must approach from.  Orient the macro so that face looks"
  puts "  into open core, never at a die boundary, and give it the halo."
}

#=============================================================================
# E.  CONSTRAINTS
#
# WHY: the SDC is the only statement of intent in the whole flow.  If a clock
# is missing, every path on it is unconstrained - and an unconstrained path is
# not fast, it is simply never examined.  V3's surprises lived here.
#=============================================================================
puts "\n--------------------------------------------------------------"
puts " E.  CONSTRAINTS"
puts "--------------------------------------------------------------"

read_sdc SDC/ivcu_ev_v4.sdc
puts "  SDC read without error."

puts "\n  clocks the tool now knows about:"
foreach clk [all_clocks] {
  puts [format "    %-14s period %8.2f ns" [get_name $clk] [get_property $clk period]]
}
puts "\n  Four clocks expected: clk_aon 100, clk_sensor 40, clk_ai 20, clk_mcu 20."
puts "  A missing one here means an entire domain is untimed."

#=============================================================================
# F.  DIE FEASIBILITY - THE ARITHMETIC BEHIND STAGE 1
#
# WHY: this is the calculation that stage 1 turns into a die.  Doing it here,
# before committing, means the floorplan is a consequence of numbers rather
# than a guess you discover was wrong during global placement.
#
# UTILISATION is the fraction of the core occupied by cells at the start.
# It is deliberately NOT 100 %, because placement needs room to:
#   - insert the buffer trees repair_design will add (and this design needs a
#     big one: 3,321 loads on the clk_sensor reset)
#   - insert the clock tree at CTS
#   - let the router find paths without detouring
#
# 40-50 % is the normal band for a Sky130 design containing a macro.  Below
# 35 % you are wasting silicon; above 60 % placement and routing start to
# fight you and you will not know which stage to blame.
#=============================================================================
puts "\n--------------------------------------------------------------"
puts " F.  DIE FEASIBILITY"
puts "--------------------------------------------------------------"

# standard cell area, summed from the actual instances
set cell_area 0.0
set macro_area 0.0
foreach inst $insts {
  set master [$inst getMaster]
  set a [expr {([$master getWidth]/1000.0) * ([$master getHeight]/1000.0)}]
  if {[$master isBlock]} {
    set macro_area [expr {$macro_area + $a}]
  } else {
    set cell_area  [expr {$cell_area + $a}]
  }
}

set HALO 40.0
set UTIL 0.45

puts [format "  standard cell area          %12.0f um2" $cell_area]
puts [format "  macro area                  %12.0f um2" $macro_area]
puts [format "  target utilisation          %12.0f %%"  [expr {$UTIL*100}]]
puts ""
set cells_need [expr {$cell_area / $UTIL}]
puts [format "  core needed for cells       %12.0f um2   (%.0f / %.2f)" \
      $cells_need $cell_area $UTIL]

# macro footprint including halo
set mw 696.02 ; set mh 411.235
set mfoot [expr {($mw + 2*$HALO) * ($mh + 2*$HALO)}]
puts [format "  macro + %.0f um halo         %12.0f um2   (%.0f x %.0f)" \
      $HALO $mfoot [expr {$mw+2*$HALO}] [expr {$mh+2*$HALO}]]

set core_need [expr {$cells_need + $mfoot}]
puts [format "  total core needed           %12.0f um2" $core_need]

# a proposed core, and what utilisation it actually gives
set CORE_W 1200.0
set CORE_H  950.0
set core_have [expr {$CORE_W * $CORE_H}]
set avail     [expr {$core_have - $mfoot}]
set real_util [expr {$cell_area / $avail}]

puts ""
puts [format "  PROPOSED core               %.0f x %.0f = %.0f um2" \
      $CORE_W $CORE_H $core_have]
puts [format "  minus macro footprint       %12.0f um2 available for cells" $avail]
puts [format "  actual utilisation          %12.1f %%" [expr {$real_util*100}]]
puts [format "  aspect ratio (W/H)          %12.2f" [expr {$CORE_W/$CORE_H}]]
puts ""
if {$real_util > 0.60} {
  puts "  *** utilisation above 60 % - placement and routing will fight."
} elseif {$real_util < 0.30} {
  puts "  *** utilisation below 30 % - the die is bigger than it needs to be."
} else {
  puts "  utilisation is in the healthy 30-60 % band."
}

puts ""
puts "  V3 die was 1520 x 1420 = 2,158,400 um2 with TWO macros."
puts [format "  V4 die would be %.0f x %.0f = %.0f um2 with one." \
      [expr {$CORE_W+80}] [expr {$CORE_H+80}] \
      [expr {($CORE_W+80)*($CORE_H+80)}]]

puts "\n=============================================================="
puts " STAGE 0 COMPLETE"
puts "=============================================================="
puts " Read every section above before running stage 1."
puts " The number that decides the floorplan is 'actual utilisation'."
puts " The line that decides macro orientation is 'pins touching each edge'."
puts ""
