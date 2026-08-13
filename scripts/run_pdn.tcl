# =====================================================================
#  IVCU-EV V3  --  POWER DISTRIBUTION NETWORK
# =====================================================================
#
#  Run AFTER run_floorplan.tcl, in the same session:
#      source scripts/run_pdn.tcl
#
#  Or from the checkpoint in a fresh session:
#      read_db pnr_out/ivcu_floorplan_v2.odb        <-- must be FIRST
#      read_liberty libs/sky130_fd_sc_hd__tt_025C_1v80.lib
#      read_liberty macros/sram_512x32_2port_TT_1p8V_25C.lib
#      read_sdc SDC/ivcu_ev_v3.sdc
#      source scripts/run_pdn.tcl
#
#  Kept separate from the floorplan on purpose.  PDN is the part that
#  fails and needs iterating; the floorplan takes minutes to rebuild
#  (place_pins -annealing is slow).  Restarting only the PDN from a
#  checkpoint is much cheaper than restarting everything.
#
#
#  WHY A MESH
#  ----------
#  Resistance x current = voltage drop.  With one path, cells far from
#  the supply see less than 1.8 V and go slow.  That is IR drop.  A
#  mesh gives current many parallel paths, so resistance - and the drop
#  - falls roughly with the number of paths.
#
#  met4 and met5 are used because upper layers are physically thicker,
#  so they have the lowest resistance per micron.  met1 is thin and
#  only carries current a few microns, from a via down to the nearest
#  cells.
#
#  Commands 1-8 write a PLAN and draw nothing.
#  Command 9, pdngen, reads the plan and draws real metal.
#
# =====================================================================


# ---------------------------------------------------------------------
#  1.  VOLTAGE DOMAIN
# ---------------------------------------------------------------------
#  THIS LINE IS NOT OPTIONAL AFTER read_db.
#
#  set_voltage_domain writes a note in the RUNNING PROGRAM'S MEMORY.
#  It is not stored in the .odb.  read_db restores your nets and their
#  labels but not this note, so pdngen wakes up with amnesia.
#
#  When the note is missing, define_pdn_grid falls back to GUESSING:
#  it scans every net for the label POWER and requires exactly one.
#  This design has TWELVE, because OpenROAD's Verilog reader creates a
#  net for every hard-wired logic constant and labels them:
#      logic 1 -> POWER   (nets named one_)
#      logic 0 -> GROUND  (nets named zero_)
#  One pair per module, 21 nets, 6,457 pins on the six live ones.
#  Result:
#      [ERROR PDN-0181] Found multiple possible nets for POWER net
#
#  DO NOT "fix" that by relabelling one_/zero_ to SIGNAL.  insert_tiecells
#  finds constant nets precisely by their POWER/GROUND label on a net
#  that is NOT marked special.  VDD and VSS were marked special by
#  global_connect; the constant nets were not.  That difference is the
#  whole mechanism.  Relabel them and insert_tiecells finds nothing,
#  reports success, and 6,457 constant pins reach silicon undriven.
#
#  -name CORE is rewritten to "Core" internally.  define_pdn_grid
#  -voltage_domains CORE does the same rewrite, so they match.

set_voltage_domain -name CORE -power VDD -ground VSS


# ---------------------------------------------------------------------
#  2.  THE CORE GRID
# ---------------------------------------------------------------------
#  An empty named plan.  "top" is just a handle; every following
#  command refers to it with -grid top.
#
#  Not safe to run twice: [ERROR PDN-1043] Grid named "top" already
#  defined.  (add_pdn_stripe, by contrast, is ADDITIVE and running it
#  twice silently doubles your straps.  Use pdngen -report_only to
#  check before drawing.)

define_pdn_grid -name top -voltage_domains CORE


# ---------------------------------------------------------------------
#  3.  met1 RAILS  --  what actually touches the cells
# ---------------------------------------------------------------------
#  -followpins does NOT use a pitch.  It finds every cell ROW and lays
#  a rail along the power pins in it, reading the row positions out of
#  the database.  You supply no coordinates.
#
#  0.48 um is the rail width already built into every sky130 HD
#  standard cell.  Matching it exactly means the PDN rail sits on top
#  of the cell's own rail.  Wider wastes space and risks spacing
#  violations against met1 signal wires; narrower connects poorly.
#
#  Rows are flipped so neighbours share a rail, so the rails alternate
#  VGND, VPWR, VGND, VPWR every 2.72 um.  The report will show
#  "Pitch: 5.4400" - that is 2 x 2.72, the repeat distance for ONE net.
#
#  These are the only pieces of metal in the whole PDN that make
#  contact with a standard cell.

add_pdn_stripe -grid top -layer met1 -width 0.48 -followpins


# ---------------------------------------------------------------------
#  4.  met4 VERTICAL STRAPS
# ---------------------------------------------------------------------
#  27.14 um is not arbitrary:  59 x 0.46 (the unithd site width) =
#  27.14 exactly.  met4 runs vertically, so it must line up with the
#  column grid the cells sit on.
#
#  offset 13.57 is half the pitch - first strap half a pitch in from
#  the core edge rather than on the boundary.
#
#  READ THE PITCH CORRECTLY.  The pitch is PER NET, not per strap.
#  Each 27.14 um contains one VDD strap AND one VSS strap:
#      1.6 + 11.97 + 1.6 + 11.97 = 27.14
#  So the core's 1499.60 um holds about 110 met4 straps, not 55, and
#  power takes 3.2/27.14 = 11.8% of met4, not 5.9%.  The report's
#  "Spacing" field is what gives this away.
#
#  27.14 / 0.92 (the met4 routing track pitch) = 29.5, so these straps
#  do NOT sit on signal tracks.  Normal for power: straps are wide
#  special-purpose metal and the router treats them as obstacles.
#  There is a -snap_to_grid flag; sky130 does not need it here.

add_pdn_stripe -grid top -layer met4 -width 1.6 -pitch 27.14 -offset 13.57


# ---------------------------------------------------------------------
#  5.  met5 HORIZONTAL STRAPS
# ---------------------------------------------------------------------
#  27.2 hits two grids at once, because met5 runs horizontally and so
#  must line up with things that vary in y:
#      met5 track pitch  3.40 um    8  x 3.40 = 27.2   exact
#      row height        2.72 um    10 x 2.72 = 27.2   exact
#
#  offset 13.6 = half the pitch = 4 met5 tracks.
#
#  DO NOT retune this offset to fix a PDN-0178 channel.  That was tried
#  (13.6 -> 19.04) and it only swapped which net was orphaned, because
#  the failing gap was 8.33 um and one VDD-to-VSS step is 13.6 um.
#  Only one strap ever fits.  If PDN-0178 appears, the floorplan is
#  wrong, not this number.

add_pdn_stripe -grid top -layer met5 -width 1.6 -pitch 27.2 -offset 13.6


# ---------------------------------------------------------------------
#  6.  VIAS BETWEEN LAYERS
# ---------------------------------------------------------------------
#  Without these you have three separate sets of metal that merely
#  overlap, and the cells receive no current at all.
#
#  A via goes at every crossing where BOTH pieces carry the SAME net.
#  VDD crossing VSS gets nothing.
#
#  met1 -> met4 is not one via.  met1 and met4 are not neighbours:
#      met1 -via- met2 -via2- met3 -via3- met4
#  The tool works this out and builds a STACK of three, with small
#  landing pads of met2 and met3 in between.  Each stack punches a
#  column through four layers that the signal router must avoid, so it
#  is not free routing space.  There are roughly 19,000 of them.
#
#  met4 -> met5 ARE neighbours, so a single via4 - but the overlap is
#  1.6 x 1.6 um, which pdngen fills with an ARRAY of cuts.  Cuts in
#  parallel divide resistance, and this crossing sits at the top of the
#  current path where all the current for a region passes through.
#  About 5,600 of these.
#
#  Many small connections at the bottom, fewer large ones at the top.
#  That is the shape of every power network.

add_pdn_connect -grid top -layers {met1 met4}
add_pdn_connect -grid top -layers {met4 met5}


# ---------------------------------------------------------------------
#  7.  THE MACRO GRID
# ---------------------------------------------------------------------
#  The core grid feeds cells through met1 rails that follow rows.  Under
#  the macros there are NO rows - they were cut away.  So met1
#  followpins reaches nothing there.
#
#  The memories have their own pins, vccd1 and vssd1, on met4 and met3,
#  spread across their face.  met4 is chosen because it sits directly
#  under met5: one via, no stack.
#
#  -cells is a pattern matched against CELL TYPE names.  It finds the
#  master, then every instance of it.
#
#  -halo {2.0 2.0} expands to all four sides.  This is a THIRD kind of
#  halo, and it is about METAL, not floor space:
#      create_blockage        reserves floor space from standard cells
#      set_macro_halo         reserves floor space from the macro placer
#      -halo here             reserves metal from the core grid's straps
#
#  The halo may NOT overlap a cell row.  Try it and you get
#      [ERROR PDN-0008] halo overlaps row ROW_511 ...
#  with the largest legal value quoted in the message.  This is what
#  forced the v2 floorplan change.
#
#  WATCH FOR A SILENT FAILURE.  If the cell name does not match you get
#      [WARNING PDN-1031] Unable to find cells: sram_512x32_2port
#      [WARNING PDN-1051] No instances found for grid (sram).
#  and an empty DUMMY grid.  No error.  Everything downstream runs and
#  the memories end up with no power - the LVS failure that
#  global_connect already closed once.  Any warning here means STOP.

define_pdn_grid -macro -name sram -voltage_domains CORE \
                -cells sram_512x32_2port -halo {2.0 2.0}

add_pdn_connect -grid sram -layers {met4 met5}


# ---------------------------------------------------------------------
#  8.  CHECK THE PLAN BEFORE DRAWING
# ---------------------------------------------------------------------
#  Read-only.  Safe to run any number of times.  Confirm:
#      - ONE voltage domain, Core, VDD / VSS
#      - grid "top" with THREE straps, each appearing ONCE
#      - TWO connects under "top"
#      - grid "sram" listing BOTH banks, not zero

pdngen -report_only


# ---------------------------------------------------------------------
#  9.  DRAW THE METAL
# ---------------------------------------------------------------------
#  The first command here that changes the layout.
#
#  If it fails, undo in this order and only this order:
#      pdngen -ripup     removes the metal   (needs the plan to know
#                                             which shapes are its own)
#      pdngen -reset     clears the plan
#  Reset first and the metal is orphaned in the database, untracked and
#  unrecoverable without reloading the .odb.
#
#  PDN-0178 / PDN-0179 "unable to repair all channels" means some met1
#  rails have no strap they can reach.  Do not tune the offsets - check
#  whether a cell row exists somewhere it cannot be powered.

pdngen


# ---------------------------------------------------------------------
#  10.  CHECKPOINT
# ---------------------------------------------------------------------

puts "========================= PDN DONE ========================="
foreach n {VDD VSS} {
  set net [[ord::get_db_block] findNet $n]
  puts "$n : [llength [$net getITerms]] pins , \
        [llength [$net getSWires]] special wires"
}
puts "============================================================"

write_db  pnr_out/ivcu_pdn_v2.odb
write_def pnr_out/ivcu_pdn_v2.def

puts "checkpoint written: pnr_out/ivcu_pdn_v2.odb"
