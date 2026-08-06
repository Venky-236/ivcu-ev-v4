# IVCU-EV V3 — where we are

Last updated: 5 August 2026, end of the floorplan session.

**Read this first when starting a new chat.** Then read
`IVCU_flow_explained.md` (synthesis and STA) and
`IVCU_floorplan_explained.md` (floorplan and power planning).

---

# HOW TO WORK WITH ME ON THIS PROJECT

The person is Venky. He is an EEE graduate doing open-source ASIC design.
He knows circuits. He does **not** know EDA tool vocabulary.

**He asked directly for these things. Please respect them:**

1. **Use simple English.** Short sentences. Explain any new word the first time
   it appears. He said the earlier writing was "too advance" and hard to follow.
2. **Explain the command BEFORE he runs it.** Say what it does, what each
   option means, what number came from where, and what to expect.
3. **Discuss first, then execute.** Ask if he is ready before giving commands.
   Do not run ahead.
4. **He wants to learn, not just finish.** His words: "what is the point here
   without learning". Teaching matters more than speed.
5. **One command at a time** in the OpenROAD prompt. Multi-line pastes break the
   interactive Tcl shell and leave it stuck at a `...>` prompt.

---

# CURRENT STATE

## Done and verified

**Synthesis** — 2 full runs, second one is live.

```
netlist   synth_out/ivcu_ev_v3_hybrid_top_gate_full.v
          -> symlink to ivcu_ev_v3_hybrid_top_gate_full_20260804_160929.v
wrapper   synth_out/fault_log_sram_1024x32_netlist.v
area      633,274 um2 standard cells   (was 1,189,022 before RTL fixes)
cells     69,773
verified  standalone Yosys check: "Found and reported 0 problems"
```

Note: `check -assert -mapped` at the end of every synthesis run reports
thousands of false problems. This is a known Yosys false positive. The
workaround is to verify the WRITTEN netlist in a fresh Yosys session. That
always returns 0. Do not panic at the REJECTED_ prefix.

**STA** — reset nets masked, which is diagnostic only, never in the real SDC.

```
WNS  -1.389 ns      TNS  -48.323 ns
```

Everything still failing is drive strength or measurement artifact, not logic
depth. `repair_design` during placement is expected to fix it. Details and the
reasoning are in `IVCU_flow_explained.md` Part 7.

**Floorplan** — complete.

```
die     1520 x 1420 um
core    10.120 .. 1509.720  x  10.880 .. 1408.960   = 1499.60 x 1398.08
rows    514, cut to 820 segments by the macros
tracks  6 grids (li1 met1 met2 met3 met4 met5)
macros  u_fault_logger/u_fault_log_sram/u_bank_lo at (34.31, 987.395)
        u_fault_logger/u_fault_log_sram/u_bank_hi at (790.55, 987.395)
        both R0, both LOCKED
blockage  {10.12 967.395 1509.72 1408.96}  = no cells above y 967.395
pins    2,719 placed, buses grouped by edge, clocks at edge centres
taps    21,630 tapcells + 1,028 endcaps
util    44.2% of the cell band, 59% of the whole core
```

**Power connections** — done, and they survive the checkpoint.

```
VDD : 164,454 pins       VSS : 164,454 pins
macros: vccd1 -> VDD, vssd1 -> VSS on both banks
voltage domain: CORE
```

## Checkpoint files

```
pnr_out/ivcu_floorplan_taps_pdnconn.odb     48 MB   restore with read_db
pnr_out/ivcu_floorplan_taps_pdnconn.def     18 MB   portable format
```

## How to restore the session

```bash
cd ~/final_ivcu_project
openroad -gui
```

```tcl
read_db pnr_out/ivcu_floorplan_taps_pdnconn.odb
```

`read_db` MUST be the first command. No LEF, no Liberty, no netlist before it.
An `[ERROR STA-2141] No liberty libraries found` appears — that is only the GUI
trying to draw before timing is loaded. Harmless.

Then:

```tcl
read_liberty libs/sky130_fd_sc_hd__tt_025C_1v80.lib
read_liberty macros/sram_512x32_2port_TT_1p8V_25C.lib
read_sdc SDC/ivcu_ev_v3.sdc
```

Confirm power survived:

```tcl
foreach n {VDD VSS} { set net [[ord::get_db_block] findNet $n] ; puts "$n : [expr {$net eq "NULL" ? "MISSING" : "[llength [$net getITerms]] pins"}]" }
```

Want 164,454 on each.

---

# WE ARE STUCK HERE

The next command failed:

```tcl
define_pdn_grid -name top -voltage_domains CORE
```

```
[ERROR PDN-0181] Found multiple possible nets for POWER net for Core domain.
```

**In plain words:** more than one net in the design claims to be POWER. The tool
cannot decide which one is the real supply.

**Most likely cause:** the top-level ports `vdd_core`, `vdd_io` and `vdd_ram`.
These are LOGIC signals — status flags saying whether each supply is healthy.
They are not real supplies. But something may have marked them as POWER nets
because of their names.

## The diagnostic that was NOT yet run

```tcl
foreach n [[ord::get_db_block] getNets] { set t [$n getSigType] ; if {$t eq "POWER" || $t eq "GROUND"} { puts "[$n getName] : $t : [llength [$n getITerms]] pins" } }
```

```tcl
foreach d [[ord::get_db_block] getVoltageDomains] { puts "domain [$d getName]" }
```

Run these two first. Expect to see only VDD and VSS. If `vdd_core`, `vdd_io` or
`vdd_ram` appear as POWER, that is the collision.

Possible fixes once the cause is known:
- change those nets' signal type back to SIGNAL
- or a second voltage domain got created when `set_voltage_domain` ran twice
  across the checkpoint, and the old one needs clearing

---

# THE REMAINING PDN COMMANDS

Once the error is fixed. **Commands 1 to 8 only write a plan and draw nothing.
Command 9, `pdngen`, draws the real metal.**

```tcl
define_pdn_grid -name top -voltage_domains CORE
add_pdn_stripe  -grid top -layer met1 -width 0.48 -followpins
add_pdn_stripe  -grid top -layer met4 -width 1.6 -pitch 27.14 -offset 13.57
add_pdn_stripe  -grid top -layer met5 -width 1.6 -pitch 27.2  -offset 13.6
add_pdn_connect -grid top -layers {met1 met4}
add_pdn_connect -grid top -layers {met4 met5}

define_pdn_grid -macro -name sram -voltage_domains CORE \
                -cells sram_512x32_2port -halo {2.0 2.0}
add_pdn_connect -grid sram -layers {met4 met5}

pdngen
```

What each does, in simple words:

| command | meaning |
|---|---|
| `define_pdn_grid` | start an empty plan named `top` |
| `met1 -followpins` | thin rails along every cell row. 0.48 um matches the rail already inside each sky130 cell. These touch the cells. |
| `met4 -pitch 27.14` | thick vertical straps, 1.6 um wide, one every 27.14 um |
| `met5 -pitch 27.2` | same but horizontal. met4 x met5 makes the mesh. |
| `add_pdn_connect` | put vias where those layers cross |
| `-macro ... -cells sram` | a separate plan for the memories. Their power pins are on met4 and met3. |
| `pdngen` | read the whole plan and draw real metal |

Why a mesh: one wire means cells far from the supply see less than 1.8 V and go
slow. That is IR drop. A mesh gives current many parallel paths, so the drop
stays small. met4 and met5 are used because upper layers are physically thicker
and so have lower resistance.

The pitch is the trade-off: tighter means less voltage drop but fewer tracks
left for signal wires. 27 um is the normal sky130 starting point.

---

# AFTER PDN

1. `insert_tiecells` — about 268 live constants need real `conb_1` cells.
   Silicon cannot hold a logic level with nothing driving it. Note that 504 of
   the 772 constants are dead `chan[].mem[]` entries that drive nothing and
   should NOT get tie cells.
2. `global_placement` — cells get coordinates for the first time.
3. `repair_design` — **watch the buffer count.** It is currently **7** in a
   69,773 cell design. That is why one `o21ai_0` drives 5,962 reset pins. This
   step inserts the buffer trees. The count jumping to hundreds is the proof it
   worked.
4. `detailed_placement` — legalise every cell onto the row grid.
5. CTS — build the four clock trees. Hold timing becomes meaningful here for
   the first time.
6. Routing — real wires, real parasitics.
7. Signoff STA with extracted RC.

---

# KNOWN PROBLEMS — agreed plan is to note them, not fix them yet

Venky's decision: keep going, collect every problem, then fix them all in one
batch and rerun synthesis once.

## Group 1 — not problems, this is normal (950 pins)

| signal | pins | why fine |
|---|---|---|
| `sensor_digital_in` | 864 | RTL uses only bits [15:0] of each 32-bit input |
| `s_axi_rdata` | 32 | bits [63:32] are constant 0 in every RTL branch |
| `s_axi_wdata` | 32 | RTL reads only `[31:0]` of a 64-bit bus |
| `s_axi_wstrb` | 8 | byte strobes unused |
| `*_axi_*prot` | 12 | protection bits tied off, normal everywhere |
| `scan_enable`, `vdd_ram` | 2 | DFT and status |

## Group 2 — very small (18 pins)

`debug_data_out` 13, `sensor_adc_channel` 5.

## Group 3 — real, fix in the batch (422 pins)

| signal | pins | what it means |
|---|---|---|
| `sensor_adc_in` + `sensor_adc_valid` | 390 | **only 12 of 42 ADC channels work.** The top level wires all 42 into `adc_interface_14nm` and that module declares all 42 ports, but its internal logic only processes channels 0-11. |
| `fault_log_rd_data` | 32 | declared an input to silence an elaboration error. The internal SRAM read data goes to a wire connected to nothing. |

## Also open

**Macro pins off-track.** Only 3 of 117 SRAM pins land on a routing track.
OpenRAM laid them out on its own grid, which does not match sky130's track
pitches (met3 0.68, met4 0.92). Each off-track pin needs a small routing jog.
Cannot be fixed without regenerating the memory. **If detailed routing reports
DRC violations near the macros later, this is the first thing to check.**

**battery_ai -0.902 ns** on 3 endpoints, 48 minimum-size gates. Expected to
close with cell resizing. If it does not close after CTS, come back to RTL.

**SRAM LEF units.** OpenRAM wrote `DATABASE MICRONS 2000`; sky130 uses 1000.
OpenROAD discarded the whole file until this was changed. Original kept as
`macros/sram_512x32_2port.lef.units2000`. If the memory is ever regenerated,
this fix must be reapplied.

---

# TRAPS THAT HAVE ALREADY BITTEN US

1. **Never run synthesis twice at once.** `pgrep -c yosys` must print 0 before
   starting. Two runs on an 8 GB machine force swap and both crawl. This
   happened twice.
2. **Multi-line pastes break the OpenROAD Tcl prompt.** One line at a time.
3. **Windows line endings.** Run `dos2unix` on every file copied in.
4. **Verify in-place edits.** A `sed` that matches nothing reports success. The
   SRAM LEF fix failed silently once because the file says `MICRONS` not
   `MICRON`.
5. **Commit to git before anything long.** Git recovered `run_synthesis_v2.sh`
   after it was accidentally truncated to zero bytes.
6. **`pnr_out/` should be in .gitignore.** Those .odb and .def files are 130 MB
   and regenerate in minutes.
