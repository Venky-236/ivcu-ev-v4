# IVCU-EV V3 — floorplan and power planning, explained

Everything done in the OpenROAD session of 5 August 2026, why each command
exists, where every number came from, and what we found along the way.

Companion to `IVCU_flow_explained.md`, which covers synthesis and STA.

---

# PART 0 — what a floorplan is, and why the order is fixed

After synthesis you have a **logical** design: 69,773 instances and 72,283 nets,
all correctly connected, none of them anywhere. No coordinates exist.

Physical design gives every instance an (x, y). The floorplan is the first part
of that — it decides the shape of the space before anything is placed in it.

The order is not arbitrary. Each step needs the one before:

```
  read LEF        geometry: how big is everything
       |
  read Liberty    timing: how fast is everything
       |
  read netlist    what is connected to what
       |
  link_design     resolve every instance against the libraries
       |
  read SDC        clock periods and I/O timing
       |
  initialize_floorplan     create die, core and cell rows
       |
  make_tracks              create the routing grid
       |
  place_macro              put the hard blocks down
       |
  create_blockage          reserve keep-out around them
       |
  place_pins               put the I/O on the boundary
       |
  tapcell                  latch-up prevention (a DRC requirement)
       |
  PDN                      the power grid
       |
  -> placement, CTS, routing
```

**The floorplan is the cheapest thing to redo and the most expensive to get
wrong.** Every later stage inherits it. Synthesis takes 5 hours; a floorplan
takes 90 seconds.

---

# PART 1 — loading, and the bug that stopped us

## 1.1 Why LEF and Liberty are both needed

|  | LEF | Liberty |
|---|---|---|
| answers | how **big**, where the **pins** are | how **fast**, how much **power** |
| used by | placer, router | STA, resizer, CTS |
| if missing | placer has no size → crash | cell places fine, **is never timed** |

The second failure is the dangerous one: a cell in LEF but not Liberty places
and routes perfectly and is silently ignored by timing analysis.

```tcl
read_lef lef/sky130_fd_sc_hd__nom.tlef      13 layers, 25 vias
read_lef lef/sky130_fd_sc_hd.lef            437 library cells
read_lef macros/sram_512x32_2port.lef       1 library cell
read_liberty libs/sky130_fd_sc_hd__tt_025C_1v80.lib
read_liberty macros/sram_512x32_2port_TT_1p8V_25C.lib
```

The **tech LEF must come first** — it defines the layers and sites that the
cell LEFs reference.

`tt_025C_1v80` = typical process, 25 °C, 1.80 V. OpenRAM also gave you SS and
FF corners; typical is enough for floorplanning, all three are needed at signoff.

## 1.2 The LEF units bug

```
[WARNING ODB-0205] The LEF UNITS DATABASE MICRON convert factor (2000) is
                   greater than the database units per micron (1000)
[ERROR   ODB-0292] LEF data from macros/sram_512x32_2port.lef is discarded
```

**The macro did not load at all.** `findMaster` returned `NULL`.

In LEF, coordinates are always written in **microns**. The
`UNITS DATABASE MICRONS n` line declares the precision — how many integer
database units make one micron.

- sky130 tech LEF: **1000** (0.001 µm resolution)
- OpenRAM's macro LEF: **2000** (0.0005 µm resolution)

OpenROAD will not mix precisions in one database, so it threw the file away.

**Why this mattered enormously.** Had it silently accepted the mismatch:

```
factor 1000:  696.02 x 1000 =   696,020 dbu  =  696.02 um    correct
factor 2000:  696.02 x 2000 = 1,392,040 dbu  = 1392.04 um    DOUBLE
```

Your SRAM would have been exactly twice its real size, and the floorplan built
around it would have been nonsense in a way that is very hard to spot visually.

**The fix**, after confirming no coordinate in the file needed finer than
0.001 µm (`grep -oE "[0-9]+\.[0-9]{4,}"` returned zero matches):

```bash
cp macros/sram_512x32_2port.lef macros/sram_512x32_2port.lef.units2000
sed -i 's/DATABASE MICRONS 2000/DATABASE MICRONS 1000/' macros/sram_512x32_2port.lef
```

Note it took two attempts — the first `sed` used `MICRON` and the file says
`MICRONS`. The `grep` afterwards showed the value unchanged, which is why you
always verify an in-place edit.

**The wider lesson.** This is a seam between two tools that were never told
about each other. OpenRAM generated the memory with its own conventions;
OpenROAD consumes it with different ones. Expect friction at every such seam,
and read the warnings there most carefully.

## 1.3 link_design

```
instances 69773    nets 72283    ports 2719
macro instances: 2
  u_fault_logger/u_fault_log_sram/u_bank_hi
  u_fault_logger/u_fault_log_sram/u_bank_lo
```

`link_design` walks every instance and resolves it against the loaded masters.
**Completing without error IS the library consistency check** — all 69,773
instances found a matching LEF entry.

2,719 ports is bit-level: 208 input buses + 51 output buses expand to 2,719
individual pins.

---

# PART 2 — initialize_floorplan

## 2.1 The command

```tcl
initialize_floorplan -die_area  {0 0 1520 1420} \
                     -core_area {10 10 1510 1410} \
                     -site      unithd
```

Both areas are `{lower-left-x lower-left-y upper-right-x upper-right-y}` in
microns.

- **die** — the total silicon you pay for
- **core** — where cell rows are created
- the 10 µm ring between them is where I/O pins live
- **site** `unithd` = 0.46 × 2.72 µm, the tile standard cells are built from

## 2.2 Where 1520 x 1420 came from — the full derivation

### The governing idea

> **The width is forced by the macros. The height is chosen from utilisation.**

### Inputs

```
standard cells                       634,059 um2   (OpenROAD report_cell_usage)
SRAM macro   696.02 x 411.235      = 286,228 um2   (measured from the LEF)
two macros                         = 572,456 um2
```

Note 634,059, not the 633,274 Yosys reported. The difference of 786 µm² is the
SRAM **wrapper glue** — 72 cells synthesised in a separate Yosys run, so
`stat` on the top module never counted them. OpenROAD reads both netlists and
sees everything, so its number is authoritative.

### Utilisation — the concept the whole floorplan rests on

```
utilisation = area occupied by cells / area available for cells
```

You can never reach 100%. Three reasons:

1. cells must legalise onto the row grid — gaps appear
2. the router needs space **between** cells for wires
3. **more cells get added later** — buffer trees, tie cells, tap cells, fillers

Rules of thumb:

| situation | target |
|---|---|
| no macros, relaxed timing | 60–75% |
| **large macros present** | **40–55%** |
| congested or high performance | 30–40% |

Macros push it down because a 696 µm block is a solid wall. Signals crossing it
must detour, so demand for routing space rises exactly where supply fell.

**We chose 44%.**

### Step 1 — width, forced

```
  24.00   margin: core left edge -> macro 1
 696.02   macro 1
  60.00   channel between the macros
 696.02   macro 2
  24.00   margin: macro 2 -> core right edge
--------
1500.04   ->  core width 1500
```

The **60 µm channel** is not decoration. Your wrapper muxes `dout0`/`dout1`
from both banks, so **64 bits cross from one memory to the other**, and power
straps run down that gap. Butt them together and there is nowhere to put any
of it.

The **24 µm side margins** exist because the memories have pins on their edges
that need space for wires to escape sideways.

### Step 2 — how much floor do the cells need

```
available space = cell area / utilisation
                = 634,059 / 0.44
                = 1,441,043 um2
```

**Why divide?** Furniture in a room. 44 m² of furniture, want the room 44%
full → you need 100 m² of room, because 44 / 0.44 = 100.

### Step 3 — turn area into height

Width is already fixed at 1500:

```
height = area / width = 1,441,043 / 1500 = 960.7 um
```

That is the **cell band** — the space below the macros.

### Step 4 — the macro band

```
  10.000   gap: core top edge -> top of macro
 411.235   the macro itself
  20.000   keep-out below the macro
---------
 441.235   macro band
```

The **20 µm halo below** is the important one: that edge faces all 69,771
standard cells. Every one of the macro's 117 signal pins must get its wire down
into the cell band. Cells flush against that edge leave the router no room —
and we already know 114 of those pins are off-track and need a jog too. Both
problems land in the same place.

Above the macro there are no cells, just the core boundary, so 10 µm suffices.

### Step 5 — add them

```
 960.7   cell band
 441.2   macro band
-------
1401.9   -> round to 1400
```

Rounded down. Round numbers are easier to check, and 1.9 µm costs 0.13% of
utilisation.

### Step 6 — die is core plus the pin ring

```
1500 + 10 + 10 = 1520
1400 + 10 + 10 = 1420
```

### Step 7 — verify backwards

```
cell band height = 1400 - 441.235          =   958.8 um
cell band area   = 1500 x 958.8            = 1,438,200 um2
utilisation      = 634,059 / 1,438,200     =    44.1%
aspect ratio     = 1500 / 1400             =    1.07
```

### Why near-square matters

For a fixed area, a square has the shortest average distance between any two
points. Wires are shorter, delay is lower.

The alternative — stacking the macros vertically — was rejected for this:

| | side by side | stacked |
|---|---|---|
| core width | 1500 | ~750 |
| macro band | 441 | ~912 |
| cell band needed | 961 | 1,921 |
| **core** | **1500 × 1400** | **750 × 2834** |
| aspect ratio | **1.07** | 0.26 |

Same cells, same utilisation. But in the stacked version a signal from bottom to
top travels **2834 µm instead of 1400** — twice the wire on every long path.

## 2.3 What the tool reported

```
[WARNING IFP-0028] Core area lower left (10.000, 10.000) snapped to (10.120, 10.880)
[INFO IFP-0001] Added 514 rows of 3260 site unithd
[INFO IFP-0102] Core area:              2,096,560.768 um2
[INFO IFP-0104] Effective utilization:          0.575
```

### The snap

The core edge must land on the site grid so cells tile cleanly:

```
10.120 / 0.46 = 22.0 sites exactly
10.880 / 2.72 =  4.0 rows  exactly
```

You asked for 10.000, which is 21.7 sites and 3.7 rows — not whole numbers of
either. Not an error; the tool correcting an impossible request.

### The rows

```
3260 sites x 0.46 = 1499.60 um   (1509.720 - 10.120)
 514 rows  x 2.72 = 1398.08 um   (1408.960 -  10.880)
```

Your core is a grid of 514 × 3260 = **1.67 million legal cell positions**.

### Row flipping

Adjacent rows are mirrored vertically. Every standard cell has a VPWR rail
along one edge and VGND along the other. Flipping alternate rows makes the VPWR
rail at the top of one row coincide with the VPWR rail at the bottom of the
next, so **two rows share one rail** instead of each needing its own.

### Two different utilisation numbers, both correct

```
tool:   ALL instances / WHOLE core   = 1,206,515 / 2,096,561 = 57.5%
ours:   std cells only / cell band   =   634,059 / 1,434,884 = 44.2%
```

The tool counts macro area as used, which it is. But that says nothing about
whether the **cells** fit, because cells cannot go where macros are. Ours is
the number that governs whether placement succeeds.

---

# PART 3 — make_tracks

```tcl
make_tracks
```

`place_macro` failed before this with:

```
[ERROR MPL-0039] No track-grid found for layer met4
```

**Tracks** are the grid of legal wire centrelines on each routing layer. From
the tech LEF pitches:

```
li1  VERTICAL    0.46      met3 HORIZONTAL  0.68
met1 HORIZONTAL  0.34      met4 VERTICAL    0.92
met2 VERTICAL    0.46      met5 HORIZONTAL  3.40
```

A track grid turns those into actual lines across the die. **Wires can only run
on tracks** — the router works in track units, not free space.

`place_macro` needs them because it snaps the macro so its pins land *on*
tracks. A pin between two tracks is a pin the router cannot cleanly reach.

Note the **alternating directions**. That is what lets the router turn corners:
travel horizontally on met3, via up to met4, travel vertically. Two adjacent
layers sharing a direction would make routing far harder.

`initialize_floorplan` made the rows (where cells go). `make_tracks` makes the
tracks (where wires go). Separate grids, separate purposes, separate commands.

Verify: `[llength [[ord::get_db_block] getTrackGrids]]` → **6**.

---

# PART 4 — macro placement

## 4.1 Deciding orientation first

```tcl
set m [[ord::get_db] findMaster sram_512x32_2port] ; ... pin y range ...
-> pins span y = 0.0 to 411.235   (macro is 411.235 tall)
```

Pins are spread over the **entire face**, not clustered on one edge — normal
for an OpenRAM block, whose pins sit on met3/met4 and are reached from above.

**So orientation does not affect pin access.** `R0` for both. Worth checking
rather than assuming: a macro with its pins facing a wall is a classic
floorplan mistake — everything routes, but every wire takes the long way.

## 4.2 The coordinates

Working from the **snapped** core, 10.120 .. 1509.720 × 10.880 .. 1408.960:

**Y, same for both:**
```
core top                          1408.960
minus 10 gap                      1398.960   wanted macro top
minus macro height 411.235         987.725   wanted macro bottom
snap: (987.725 - 10.880) / 2.72 = 359.13  ->  359 rows
10.880 + (359 x 2.72)              987.360   <- used
```

**X:**
```
u_bank_lo:  10.120 + (52 x 0.46)   =   34.040
u_bank_hi:  10.120 + (1696 x 0.46) =  790.280

channel      790.280 - (34.040 + 696.02)  =  60.22 um
right margin 1509.720 - (790.280 + 696.02) = 23.42 um
```

Everything lands on the site grid so cells can abut the macros without a ragged
gap.

## 4.3 The commands and the result

```tcl
place_macro -macro_name u_fault_logger/u_fault_log_sram/u_bank_lo \
            -location {34.040 987.360} -orientation R0
place_macro -macro_name u_fault_logger/u_fault_log_sram/u_bank_hi \
            -location {790.280 987.360} -orientation R0
```

```
u_bank_lo  ( 34.31, 987.395) to ( 730.33, 1398.63)  R0  LOCKED
u_bank_hi  (790.55, 987.395) to (1486.57, 1398.63)  R0  LOCKED
```

Verification:

```
                    designed     actual
macro width          696.020     730.33 - 34.31    = 696.020
macro height         411.235    1398.63 - 987.395  = 411.235
channel               60.000     790.55 - 730.33   =  60.22
left margin           24.000      34.31 - 10.12    =  24.19
right margin          23.420    1509.72 - 1486.57  =  23.15
gap to core top       10.000    1408.96 - 1398.63  =  10.33
```

The tool nudged by 0.27 µm in x and 0.035 µm in y to hit the track grid.

`LOCKED` means the placer will not move them.

## 4.4 The pin alignment warning — a real, permanent limitation

```
[WARNING MPL-0002] Could not align all pins ... 2 out of 103 pins were aligned
[WARNING MPL-0002] Could not align all pins ... 1 out of 14 pins were aligned
```

103 met3 pins and 14 met4 pins. **Only 3 of 117 landed on a track.**

**Why it cannot do better.** A macro has exactly one position. Its 103 pins sit
at fixed offsets inside it. For all of them to land on tracks, the macro's
internal pin pitch would have to be a whole multiple of the track pitch
(met3 = 0.68 µm, met4 = 0.92 µm). OpenRAM laid those pins out on its own grid
with no knowledge of sky130's routing tracks. Moving the macro shifts all 103
together — you can align two, never all.

**What it costs.** Each off-track pin needs the router to jog — a short
off-grid segment or an extra via. Detailed routing handles it, but it adds
congestion right at the macro edge, already the most congested place on the die.

**Why we are not chasing it.** Fixing it properly means regenerating the memory
with sky130 track pitches. Large job, modest gain.

**But it is written down.** If detailed routing later reports DRC violations
clustered around the macros, this warning is the first thing to come back to.

---

# PART 5 — the blockage

## 5.1 Halo versus blockage

A **halo** is a keep-out margin *attached to a macro*. It moves with the macro.

A **blockage** is a fixed rectangle in absolute coordinates. It knows nothing
about any macro.

Same effect, different ownership. Both existed in this build:

```
set_macro_halo -macro_name macro_name -halo halo
create_blockage -region {x1 y1 x2 y2} [-inst instance] [-max_density d] [-soft]
```

**We chose the blockage** because `set_macro_halo` is an MPL (macro placer)
command, and our macros are already `LOCKED` — the macro placer will never run.
Whether its halo then constrains `global_placement` varies by build. A blockage
is unconditional and you can read it back and confirm.

## 5.2 What we created

```tcl
create_blockage -region {10.12 967.395 1509.72 1408.96}
```

```
x:  10.120 -> 1509.720    full core width
y:  967.395 -> 1408.960   20 um below the macros, up to the core top
```

`967.395 = 987.395 (macro bottom) - 20 (halo)`

One rectangle covering the macros, the 60 µm channel, both side margins and the
20 µm strip below. One concept: **above y = 967.395, no standard cells.**

No `-soft` and no `-max_density` — those allow reduced-density placement. We
want a hard keep-out.

## 5.3 What it buys, and what it costs

```
below the macros   cells stop at 967.4 instead of 987.4
in the channel     60.22 - 20 - 20 = 20.2 um left in the middle
left / right       the 24 and 23 um margins are fully consumed
```

The channel effectively closes to logic — 20 µm is about 43 site widths, enough
for a handful of cells but not enough to be useful. That is fine: the channel
was never for logic, it is for the 64 `dout` wires and the power straps.

The side margins vanish entirely. Also fine — a 24 µm sliver of isolated cells
against the core edge would be badly connected to everything anyway.

**So the halo costs almost nothing you would have wanted, and buys routing room
exactly where it is scarcest.**

After it, the utilisation figure became a **property of the database** rather
than an assumption:

```
usable cell region  10.880 .. 967.395  =  956.52 um tall x 1499.60 wide
                                       = 1,434,400 um2
standard cells                         =   634,059 um2
utilisation                            =     44.2%
```

---

# PART 6 — I/O pin placement

## 6.1 Your five principles, assessed

**1. Timing-critical signals near the logic they drive** — right in principle,
awkward in practice. You do not know where the logic is until placement runs,
and placement runs after pins. In a real flow you substitute **design intent**:
you know the AXI logic belongs near the MCU side, so you put AXI pins there and
the placer follows.

**2. Clock pins positioned for CTS** — right, and the most valuable one on the
list. A clock entering at a corner forces the tree to reach diagonally across
the whole die; entering at an edge centre halves the worst-case distance. You
have four clocks feeding four different regions, so this matters more for you
than for a typical single-clock design.

**3. Power/ground distributed on all sides for IR drop** — right as a
principle, **wrong step**. Your `vdd_core`, `vdd_io`, `vdd_ram` and `pwr_good`
are not supplies; they are **logic signals reporting power status**. Real supply
arrives through the PDN — metal straps across the whole die, not pins on the
boundary. The instinct is correct; it belongs at `pdngen`, and Part 7 is where
it gets applied.

**4. Buses grouped on one side** — right, and very applicable. 42 ADC buses and
42 digital buses. Scattered, they would force wires across the entire core.

**5. High-fanout control near its logic** — same as 1, same caveat.

## 6.2 The chicken-and-egg problem

Ideally each pin sits closest to the logic it connects to. But **no cells are
placed yet**. The flow resolves it in the other direction: place pins first at a
sensible spread, then `global_placement` **pulls cells toward their pins**. The
pins become anchors that shape the placement.

That is why pin placement comes before cell placement even though it looks
backwards.

## 6.3 The counts

```
adc   504 = 42 x 12 bits      adcv   42 = 42 x 1
dig  1344 = 42 x 32 bits      digv   42 = 42 x 1
axi   440
other 347                     total 2,719
```

## 6.4 Edge capacity

```
left / right   1420 / 0.68 (met3 pitch) = 2,088 slots each
top / bottom   1520 / 0.46 (met2 pitch) = 3,304 slots each
```

The tool actually reported **5,024 usable slots**, not the ~10,800 those raw
numbers suggest, because of the line

```
Using 2 tracks default min distance between IO pins
```

Pins are not packed at pitch; there is a two-track gap between neighbours,
which roughly halves capacity. Corner avoidance takes a little more.

2,719 into 5,024 is **54% occupancy**.

## 6.5 A consequence of our own floorplan

The macro band occupies the top 441 µm of the core, and the macros span core
x = 34.31 to 1486.57 — essentially the full width.

**A pin on the top die edge must get from y ≈ 1415 down to cells below y = 967
— 448 µm straight through the macro band.** It can be done on met5, which
routes over macros, but met5's pitch is 3.4 µm: only about 450 tracks across
the whole die.

So: **keep the top edge lightly loaded.** Not a general rule — specific to the
floorplan we built, and only visible once the macros were placed.

## 6.6 The constraints applied

```tcl
set_io_pin_constraint -pin_names {sensor_adc_in_*}        -region left:*
set_io_pin_constraint -pin_names {sensor_adc_valid_*}     -region left:*
set_io_pin_constraint -pin_names {sensor_digital_in_*}    -region bottom:*
set_io_pin_constraint -pin_names {sensor_digital_valid_*} -region bottom:*
set_io_pin_constraint -pin_names {m_axi_* s_axi_*}        -region right:*

set_io_pin_constraint -pin_names {clk_50mhz_sensor} -region left:600-820
set_io_pin_constraint -pin_names {clk_200mhz_mcu}   -region right:650-770
set_io_pin_constraint -pin_names {clk_100mhz}       -region bottom:700-820
set_io_pin_constraint -pin_names {clk_10mhz_aon}    -region top:700-820
```

Region syntax is `edge:from-to` in microns, or `edge:*` for the whole edge.
`-group` keeps pins contiguous; `-order` preserves bus bit order.

**The clock placements are the point of the exercise.** Each sits at the centre
of the edge nearest its own logic — `clk_50mhz_sensor` by the ADC pins that
feed the sensor fabric, `clk_200mhz_mcu` by the AXI pins. When CTS runs, each
tree starts from a sensible root instead of a corner.

The 347 "other" pins were left unconstrained so the tool could place them where
wirelength is best.

## 6.7 The placement command

```tcl
place_pins -hor_layers met3 -ver_layers met2 -corner_avoidance 50 -annealing
```

**`-hor_layers met3`** — pins whose wires run **horizontally** into the design,
i.e. on the **left and right** edges. met3 is HORIZONTAL.

**`-ver_layers met2`** — pins whose wires run **vertically**, on the **top and
bottom** edges. met2 is VERTICAL.

A left-edge pin must be reachable by a wire travelling left-to-right, which
needs a horizontal layer. Put it on a vertical layer and the router needs a via
just to turn, on the most congested part of the die.

met2/met3 is also conventional for sky130 — high enough to stay clear of met1,
which the cells need for their own internal connections.

**`-corner_avoidance 50`** — keep pins 50 µm clear of each die corner. A corner
pin can only leave in one direction; it is the worst real estate on the boundary.

**`-annealing`** — the slower, better optimiser. Worth it for 2,719 pins.

---

# PART 7 — THE PIN CONNECTION PROBLEM

`place_pins` reported:

```
Number of I/O            2719
Number of I/O w/sink     1325
Number of I/O w/o sink   1394      <- 51%
```

## 7.1 The first analysis was wrong, and how

I ran a query counting **instance terminals** on each pin's net, and read zero
ITerms as "disconnected". On that basis I claimed the AXI slave data path was
dead — a serious accusation.

The verification proved it wrong:

```
s_axi_rdata[0] : iterms=2      connected
s_axi_wdata[0] : iterms=1      connected
```

**A net with zero ITerms has two possible causes:**

| net state | ITerms | meaning |
|---|---|---|
| driven by a cell | ≥1 | live |
| **tied to a constant** | **0** | **carries a fixed value, not broken** |
| genuinely dangling | 0 | dead |

The last two are indistinguishable to that query. I asserted the worst reading
without separating them.

**The correct explanation for AXI:**

```verilog
output reg [63:0] s_axi_rdata;
32'h0000_0000: s_axi_rdata <= {56'd0, status_register};
32'h0000_1000: s_axi_rdata <= {32'd0, read_data};
default:       s_axi_rdata <= 64'd0;
```

Bits [63:32] are **zero in every branch**. Yosys correctly deletes those 32
flip-flops and ties the outputs to constant 0 — no driving cell, so no ITerm.

Same on the input side: `write_data <= s_axi_wdata[31:0]` — the bus is declared
64 bits, only the low 32 are ever read.

**So the AXI slave works.** It is a 64-bit bus carrying 32-bit data. Wasteful in
pins, entirely functional.

## 7.2 What IS genuinely disconnected

Single-bit signals have no "upper bits tied to zero" story — either something
reads them or nothing does:

```
sensor_adc_valid_0    iterms=4     live
sensor_adc_valid_11   iterms=20    live
sensor_adc_valid_12   iterms=0     dead
sensor_adc_valid_41   iterms=0     dead

sensor_digital_valid_0   iterms=1  live
sensor_digital_valid_41  iterms=1  live
```

**ADC channels 0–11 are wired. Channels 12–41 are not. Twelve of forty-two.**

The digital path is fine — both ends of that range are live.

The top level *does* connect all 42 channels into `adc_interface_14nm`, and the
module *does* declare 42 ports (85 connections = 42 data + 42 valid + 1
channel select). So the ports are plumbed; the **logic inside the module only
processes 12**. Yosys saw 30 inputs feeding nothing and propagated that outward
until the top-level pins had no loads.

## 7.3 The full unconnected list, categorised

| signal | pins | cause | real problem? |
|---|---|---|---|
| `sensor_digital_in` | 864 | unused upper bits per channel | no — RTL uses `[15:0]` of 32 |
| `sensor_adc_in` | 360 | channels 12–41 unprocessed | **yes** |
| `sensor_adc_valid` | 30 | channels 12–41 unprocessed | **yes** |
| `fault_log_rd_data` | 32 | port made an input to silence an error | **yes** |
| `s_axi_rdata` | 32 | bits [63:32] constant 0 | no — by design |
| `s_axi_wdata` | 32 | bits [63:32] never read | no — by design |
| `s_axi_wstrb` | 8 | byte strobes unused | no |
| `*_axi_*prot` | 12 | protection bits tied off | no — normal |
| `debug_data_out` | 13 | partial | minor |
| `sensor_adc_channel` | 5 | partial | minor |
| `scan_enable`, `vdd_ram` | 2 | DFT / status | no |

## 7.4 What this costs, and whether it matters now

```
30 ADC channels x (12 data + 1 valid) = 390 pins
fault_log_rd_data                     =  32 pins
```

Over 400 package pins accepting data the silicon ignores. Your interface
documentation says 42 sensors; the chip reads 12.

**It does not affect this floorplan.** A pin with no sink generates no wire —
the placer has nothing to pull toward it. Those pins occupy boundary slots, and
you have 5,024 for 2,719.

**It does matter for the project.** Same family as `fault_log_rd_data`: ports
that exist in the interface but connect to nothing. Worth an RTL review before
tapeout; not worth a five-hour re-synthesis today.

## 7.5 HPWL

```
I/O nets HPWL: 1,105,403.75 um
```

Half-perimeter wirelength — the standard cheap wirelength estimate. For each
net, draw the smallest box containing all its pins; HPWL is half that box's
perimeter, summed over all nets.

Right now it is close to meaningless because **no cells are placed** — every
net's box is drawn against logic with no location. Its value is as a
**baseline**: after `global_placement` you see this number again, and the drop
measures how much the placer improved things.

---

# PART 8 — tap cells

## 8.1 What latch-up is, and why this is not optional

CMOS has an unavoidable parasitic structure. The p-substrate, n-well and the
source/drain diffusions form two interleaved bipolar transistors — effectively
a **thyristor under every cell**.

Normally dormant. But if the well or substrate voltage drifts — a noise spike,
an ESD event, accumulated charge with nothing holding it — that thyristor can
**turn on**. Once on, it latches: a low-resistance path opens from VDD straight
to ground and stays open until power is removed. The chip draws huge current and
usually destroys itself.

**Tap cells prevent it** by physically tying substrate to VGND and n-well to
VPWR at regular intervals, so neither can drift. They contain no logic — they
are pure electrical anchors.

**End caps** close off the ends of each cell row so the wells terminate cleanly
at the boundary.

This is a **DRC rule**, not a quality knob.

## 8.2 The command

```tcl
tapcell -distance 13 \
        -tapcell_master sky130_fd_sc_hd__tapvpwrvgnd_1 \
        -endcap_master  sky130_fd_sc_hd__decap_3
```

**`-distance 13` is a radius, not a spacing.** It means *maximum 13 µm from any
point to the nearest tap*, so taps sit every **26 µm**:

```
1,209,602 sites x 0.46 = 556,417 um of total row length
556,417 / 26            =  21,400 taps    matches the 21,630 inserted
556,417 / 13            =  42,800         what a spacing reading predicts
```

I initially predicted ~59,000 by reading it as spacing. Same convention applies
in most tools; worth remembering.

## 8.3 What happened

```
[INFO ODB-0303] The initial 514 rows (1,675,640 sites) were cut with 2 shapes
                for a total of 820 rows (1,209,602 sites)
[INFO TAP-0004] Inserted 1,028 endcaps
[INFO TAP-0005] Inserted 21,630 tapcells
```

### Row cutting — the macros becoming physically real

A row that runs into a macro cannot continue through it. It gets **cut into
segments**: left margin, the 60 µm channel, right margin.

```
514 rows  ->  820 row segments
1,675,640 sites -> 1,209,602 sites
lost:       466,038 sites
466,038 x 1.2512 um2/site = 583,000 um2
two macros                = 572,456 um2      agrees
```

That is the silicon under the memories, gone from cell placement forever.

Note it says **2 shapes**, not 3 — the **macros** cut the rows, the blockage did
not. A blockage is a *placement* rule, not a row-level one. Rows still exist in
the 20 µm halo; the placer simply will not use them. Both mechanisms work, at
different levels.

### The cost

```
instances  69,773 -> 93,043      (+23,270)
area    1,206,515 -> 1,239,734 um2   (+33,219)
utilisation   58% -> 59%
```

Tap cells are one site wide — 1.25 µm² each. Twenty-two thousand of them cost
2.7% of your area and are non-negotiable.

---

# PART 9 — POWER PLANNING

## 9.1 The problem

Every one of your 93,043 cells needs VDD and VSS. But **your netlist contains
no power connections at all** — gate-level Verilog omits them by convention.
`read_verilog` never saw a single power pin.

Meanwhile every cell in the LEF does have them:

- `VPWR` / `VGND` — the supply rails
- `VPB` / `VNB` — the well taps (p-well bulk, n-well bulk)

And the SRAM macros have `vccd1` / `vssd1`, flagged during readiness checks as
"unconnected in the netlist, PDN must handle it."

## 9.2 Step 1 — global connections

```tcl
add_global_connection -net VDD -pin_pattern {^VPWR$} -power      -> 93,041
add_global_connection -net VDD -pin_pattern {^VPB$}              -> 71,411
add_global_connection -net VSS -pin_pattern {^VGND$} -ground     -> 93,041
add_global_connection -net VSS -pin_pattern {^VNB$}              -> 71,411

add_global_connection -net VDD -inst_pattern {.*u_bank.*} \
                      -pin_pattern {^vccd1$} -power              -> 2
add_global_connection -net VSS -inst_pattern {.*u_bank.*} \
                      -pin_pattern {^vssd1$} -ground             -> 2

global_connect
```

Each line says: *any pin with this name, on any instance, belongs to this net.*
`-power` and `-ground` mark which is which so later tools know the difference.
`-inst_pattern` restricts a rule to matching instances — the macros need their
own because their pins are lowercase and differently named.

### The arithmetic confirms itself

```
VPWR  93,041      VPB  71,411
VGND  93,041      VNB  71,411
                        difference = 21,630
```

**21,630 is exactly the tap cell count.** Tap cells have `VPWR`/`VGND` but no
`VPB`/`VNB` — because they *are* the well tap; there is nothing for them to tap
into.

```
93,041 + 71,411 + 2 = 164,454      matches both nets
```

### The macros, confirmed

```
u_bank_lo : vccd1 -> VDD    vssd1 -> VSS
u_bank_hi : vccd1 -> VDD    vssd1 -> VSS
```

**That is the LVS failure identified during readiness checks, closed.** Without
it everything would route, everything would time, and the design would fail LVS
at the very end because the memories had no supply.

## 9.3 Step 2 — voltage domain

```tcl
set_voltage_domain -name CORE -power VDD -ground VSS
```

One domain, as established when checking for UPF. Declares which net is supply
and which is ground for the whole core.

## 9.4 Where the macro power pins are

```tcl
vccd1 on met4 / met3        vssd1 on met4 / met3
```

met4 is one of the intended strap layers, so the macro grid can connect
directly without an awkward via stack.

## 9.5 What comes next — the mesh

**This is where your IR-drop instinct applies.**

```
met1   thin rails along every row, alternating VDD / VSS
       these are the cells' own supply, built into their layout

met4   thick vertical straps every ~27 um
met5   thick horizontal straps every ~27 um

       met4 x met5 crossings get vias  ->  a MESH
```

**Why a mesh.** Current takes a path from the supply to each cell. Resistance ×
current = voltage drop. With a single path, cells far from the source see less
than 1.8 V and slow down — that is IR drop.

A mesh gives current **many parallel paths**. Resistance falls roughly with the
number of paths, and the drop falls with it.

met4 and met5 are used because they are the **thickest** layers, so lowest
resistance per micron. met1 is thin and only carries current a few microns from
a strap to the nearest cell.

The commands still to run:

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

**`-followpins`** is the key word on the met1 line. It does not draw straps at a
pitch — it draws met1 rails **along every cell row**, on top of the VPWR/VGND
pins the cells already have. Width 0.48 µm matches the rail width built into
sky130 HD cells. These are what physically touch your 93,043 cells.

**The strap pitch is the real trade-off.** Tighter pitch means lower IR drop but
more routing tracks consumed by power. 27 µm is the standard sky130 starting
point. If IR analysis shows too much drop you tighten it; if routing is
congested you loosen it.

**`add_pdn_connect`** punches vias where layers cross. Without it you would have
three disconnected sets of metal.

---

# PART 10 — STATE AND WHAT REMAINS

## Completed

| item | result |
|---|---|
| die | 1520 × 1420 µm |
| core | 10.120 .. 1509.720 × 10.880 .. 1408.960 = 1499.60 × 1398.08 |
| rows | 514, cut to 820 segments by the macros |
| tracks | 6 grids, li1 through met5 |
| macros | both placed, `LOCKED`, verified to 0.01 µm |
| blockage | y ≥ 967.395 reserved |
| I/O pins | 2,719 placed, buses grouped, clocks at edge centres |
| tap cells | 21,630 + 1,028 endcaps |
| power nets | VDD/VSS created, 164,454 pins each, macros connected |
| voltage domain | CORE |
| utilisation | 44.2% of the cell band, 59% of the whole core |

## Checkpoints

```
pnr_out/ivcu_floorplan_taps_pdnconn.odb     48 MB, restore with read_db
pnr_out/ivcu_floorplan_taps_pdnconn.def     18 MB, portable
```

To resume: `read_db` as the **first** command in a fresh session, then re-apply
`read_liberty` ×2, `read_sdc`, and check whether the global connections
survived — geometry always does, the power setup may not.

## Remaining

1. **PDN grid and `pdngen`** — the commands in 9.5
2. **`insert_tiecells`** — the ~268 live constants need real `conb_1` cells
3. **`global_placement`** — cells get coordinates for the first time
4. **`repair_design`** — where the 5,962-load reset net finally gets its buffer
   tree. Watch the buffer count: it is currently **7**
5. **`detailed_placement`** — legalise everything onto the row grid
6. **CTS** — build the four clock trees. Hold timing becomes meaningful here for
   the first time
7. **Routing** — real wires, real parasitics
8. **Signoff STA** with extracted RC

## Design issues found, not blocking

| issue | impact |
|---|---|
| ADC channels 12–41 unprocessed | 390 pins accept data the chip ignores |
| `fault_log_rd_data` input drives nothing | 32 pins, and the SRAM read data has no reader |
| `s_axi` 64-bit bus carries 32-bit data | 40 pins constant or unread |
| macro pins off-track (114 of 117) | routing congestion at macro edges |
| `estimated_range_km` chain | fixed, but check for similar width issues elsewhere |

All are interface-cleanliness problems, none affect whether the chip works for
what *is* connected. Worth an RTL review before tapeout.
