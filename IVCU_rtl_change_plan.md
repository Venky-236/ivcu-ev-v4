# IVCU-EV V3 — the RTL problem list and change plan

Everything found in the RTL that is worth changing before the next synthesis
run, with the evidence for each one, what it costs, and what I would do.

Written 6 August 2026. Companion to `IVCU_architecture_explained.md`.

**Nothing here is a decision made for you.** Each item says what is wrong,
how I know, and what I would recommend. You choose.

---

# PART 0 — HOW TO READ THIS

Every problem has the same five fields:

| field | meaning |
|---|---|
| **Evidence** | the file and line, or the log number. You can check it yourself. |
| **Cost now** | what it is costing you today, in µm² or pins |
| **Fix** | what to change |
| **Effort** | small = under an hour. medium = a morning. large = a day or more. |
| **Risk** | what could go wrong when you change it |

Confidence markers:

- **measured** — a number read out of a log or source file
- **derived** — arithmetic from measured numbers
- **estimated** — my judgement, could be wrong, marked as such

---

# PART 1 — THE ONE PIECE OF SEQUENCING THAT MATTERS MOST

You have decided **all 42 sensors must be analog**, so the ADC needs
extending from 12 channels to 42.

**Do not do that first.** Do problem A3 first. Here is why.

The ADC is **62,111 µm² for 12 channels** — measured. That is **5,176 µm²
per channel**. Extend to 42 at that rate and you get:

```
42 x 5,176 = 217,000 um2      (derived)
```

That is +155,000 µm², roughly a quarter of your whole standard-cell area,
for a block that today does almost nothing.

Now look at why one channel costs 5,176 µm²:

```verilog
calibrated[2] = ({5'd0, adc_in_2} + {5'd0, offset_correction}) * gain_correction >> 8;
```

A **16-bit multiplier per channel**. And at the top level:

```verilog
.calibration_enable (1'b0),
.offset_correction  (12'd0),
.gain_correction    (16'd256),
```

`offset_correction` and `gain_correction` are **ports**, not parameters. So
synthesis never sees the constants and builds a general multiplier for every
channel — the same mechanism that cost you 575,000 µm² in the sensor fabric.

And look at the arithmetic those constants actually describe:

```
(adc + 0) * 256 >> 8
= adc * 256 / 256
= adc                    <- the identity function
```

**The calibration path, with the values you are feeding it, does nothing at
all.** Make those two ports parameters and the whole multiplier collapses to a
wire.

```
Order A:  extend to 42 first, then fix parameters
          -> you build 42 general multipliers, then throw them away

Order B:  fix parameters first, then extend to 42          <- DO THIS
          -> each channel is a register and a mux. 42 channels may well
             cost LESS than the 12 you have now.
```

I will not put a number on the "after" figure because I have not measured it.
But the mechanism is identical to one you have already proven twice on this
design.

---

# PART 2 — THE PROBLEM LIST

Sorted by what it costs you.

| # | problem | cost | effort | confidence |
|---|---|---|---|---|
| **A1** | `u_diagnostic` — 386 output bits, none connected | **118,908 µm²** | small | measured |
| **A2** | `grace_timeout` is a port, not a parameter | part of **105,193 µm²** | small | measured |
| **A3** | ADC calibration ports build 42 unused multipliers | part of **62,111 µm²** | small | measured |
| **F1** | ADC converts 12 of 42 channels | 390 pins, **you chose to extend** | large | measured |
| **A4** | `sensor_validation` — 22 constants tied to ports | part of 7,219 µm² | small | measured |
| **A5** | `sensor_fabric` — 2 constants tied to ports | part of 244,062 µm² | small | measured |
| **A6** | `motor_ai` — 3 constants tied to ports | part of 36,857 µm² | small | measured |
| **A7** | fabric declares `mem[0:15]`, uses `mem[0:3]` | 504 dead registers | small | measured |
| **F2** | `raw_array` in the top: 42 writes, 0 reads | 384 dead pins | small | measured |
| **F3** | validation ranges tied 0 / 0xFFFF — no checking happens | function | medium | measured |
| **F4** | fault log `rd_en` tied 0 — SRAM never read | function | small | measured |
| **F5** | `torque_command` / `regen_command` / `steering_assist` go nowhere | function | small | measured |
| **F6** | `fault_log_rd_data` input drives nothing | 32 pins | small | measured |
| **F7** | channels 35–41 read only by the MCU | 7 channels | — | measured |
| **F8** | `s_axi` 64-bit bus carries 32-bit data | 72 pins | medium | measured |
| **F9** | no scan chain despite four DFT pins | testability | large | measured |
| **F10** | `sensor_data[]` crosses 3 clock domains unsynchronised | risk | medium | measured |

---

# PART 3 — EACH PROBLEM IN DETAIL

## A1 — `u_diagnostic`: 118,908 µm² that cannot reach a pin

**Evidence.** `diagnostic_report_generator.v` lines 19–25 declare **seven
outputs, 386 bits total**:

```verilog
output reg [127:0] maintenance_report;
output reg  [63:0] battery_life_prediction;
output reg  [63:0] motor_life_prediction;
output reg  [95:0] component_wear_level;
output reg         report_ready;
output reg  [31:0] report_data;
output reg         report_valid;
```

`ivcu_ev_v3_hybrid_top.sv` lines 1316–1333 instantiate it with **twelve
inputs, three tie-offs, and not one output**. Verilog lets you omit a port
silently, so nothing complained.

It is the **third largest module in the design, 18.8% of standard-cell area**.
`opt_clean` kept it because Yosys treats module outputs as public wires.

**Cost now.** 118,908 µm² — measured. Removing it takes standard cells from
633,274 to **514,366 µm²**, a 18.8% reduction.

**Fix.** Two honest options.

*Remove it:* comment out the instantiation. Keep the `.v` file. One line
change, and it comes back the day you have a register map for its 386 bits.

*Connect it:* route `report_data[31:0]` and `report_valid` into
`mcu_axi_lite_interface` as a new address, so the MCU can poll the
predictions. This keeps the feature and the area. The other 354 bits do not
fit anywhere without a paging scheme.

**Effort.** Small either way. **Risk.** None for removal — nothing reads it.

**What I would do.** Remove it for this run. You are trying to reach a working
tapeout; 19% of your die for predictions no one can read is not the trade to
make now. Note it in `WHERE_WE_ARE.md` as deferred, not deleted.

---

## A2 — `grace_timeout` is a port, not a parameter

**Evidence.** `ivcu_ev_v3_hybrid_top.sv` line 874:

```verilog
.grace_period_count (3'd5),
.grace_timeout      (32'd1000000),
```

`sensor_grace_manager_complete.sv` builds 42 of these:

```verilog
reg [31:0] grace_timer;
if (grace_timer < grace_timeout) grace_timer <= grace_timer + 1;
if (grace_timer == grace_timeout - 1) ...
```

Because the constant is at the top level and synthesis never flattens, each
channel gets **two full 32-bit magnitude comparators** against an unknown
value, instead of a comparison with a fixed number.

This is **exactly** problem 9 from `IVCU_flow_explained.md`, the one that cost
you 575,000 µm² in `sensor_interface_fabric_complete`.

**Cost now.** The module is 105,193 µm², 12,898 cells — measured. How much of
that is the comparators I have not measured, so I will not guess.

**Fix.**

```verilog
module sensor_grace_manager_complete #(
    parameter integer GRACE_TIMEOUT      = 1_000_000,
    parameter integer GRACE_PERIOD_COUNT = 5
) ( ... );      // remove the two ports
```

and at the top:

```verilog
sensor_grace_manager_complete #(
    .GRACE_TIMEOUT(1_000_000), .GRACE_PERIOD_COUNT(5)
) u_sensor_grace ( ... );   // remove the two port connections
```

**Effort.** Small. **Risk.** Low — but the timeout is no longer runtime
adjustable. It never was, in practice; it was hard-tied.

**What I would do.** Change it. Proven mechanism, proven payoff.

---

## A3 — the ADC calibration multipliers (do this before F1)

Covered fully in Part 1. Same fix shape:

```verilog
module adc_interface_14nm #(
    parameter integer OFFSET_CORRECTION = 0,
    parameter integer GAIN_CORRECTION   = 256,
    parameter bit     CALIBRATION_ENABLE = 0
) ( ... );
```

With `CALIBRATION_ENABLE = 0` the whole calibrated branch disappears at
elaboration and each channel becomes a zero-extend and a register.

**Even better:** if calibration is genuinely never needed, delete the branch
entirely rather than parameterising it. Dead code that cannot be reached is
better removed than optimised away.

**What I would do.** Delete the calibration path. If you want calibration
later, it belongs in software reading raw counts, not in 42 copies of a
multiplier on silicon.

---

## F1 — extending the ADC to 42 channels *(your decision)*

**Evidence.** `adc_interface_14nm.v` line 310, the module's own comment:

```verilog
// Note: ADCs 12-41 are ignored because we have only 12 outputs.
```

The module declares `adc_in_0..41` and `adc_valid_0..41` but only
`digital_out_0..11` and `digital_valid_0..11`.

**What extending means, concretely.** Three changes, in this order:

**1. The ADC module.** Add `digital_out_12..41` and `digital_valid_12..41`,
and thirty more copies of the per-channel block. The file is written as 42
hand-copied `if` blocks; **replace it with a generate loop** rather than
pasting thirty more:

```verilog
generate
  for (genvar i = 0; i < 42; i++) begin : ch
    always @(posedge clk_sensor or negedge rst_sensor_n)
      if (!rst_sensor_n)      digital_valid[i] <= 1'b0;
      else if (adc_valid[i]) begin
        digital_out[i]   <= {20'd0, adc_in[i]};
        digital_valid[i] <= 1'b1;
      end else digital_valid[i] <= 1'b0;
  end
endgenerate
```

Forty-two hand-written copies is where transcription bugs live. A loop is one
piece of logic you can read once.

**2. The top level.** Rewire the fabric so all 42 raw inputs come from the
ADC:

```verilog
.sensor_raw_in_12 (digital_out_12),    // was sensor_digital_in_12
...
.sensor_raw_in_41 (digital_out_41),    // was sensor_digital_in_41
```

**3. Decide what happens to `sensor_digital_in_12..41`.** Once the ADC feeds
all 42 channels, those 30 digital ports (960 pins) have no consumer. Either
delete them, or keep them as an alternative input path with a mux — which is
new logic and a new control signal.

**Cost.** Depends entirely on doing A3 first. With the multiplier removed,
each channel is a 32-bit register and a mux — cheap. With it, +155,000 µm²
(derived).

**Effort.** Large. This is the biggest change on the list. **Risk.** Medium —
you are touching the input path of the whole chip. `verify_all.sh` gate 6
(whole-design elaboration) is your safety net.

**What I would do.** Do it, but do A3 first, and rewrite the module as a
generate loop rather than extending the copy-paste. And decide about
`sensor_digital_in_12..41` explicitly — do not leave 960 pins dangling.

---

## A4, A5, A6 — the rest of the constants-tied-to-ports family

Same mechanism as A2 and A3. All measured from the top-level instantiation.

| module | ports that are really constants | value |
|---|---|---|
| `u_sensor_validation` | `validation_timeout` | 32'd1000 |
| | `hysteresis_threshold` | 16'd10 |
| | `expected_ranges_min_0..9` | all 16'd0 |
| | `expected_ranges_max_0..9` | all 16'hFFFF |
| `u_sensor_fabric` | `filter_coefficients` | 16'd256 |
| | `deadband_threshold` | 16'd5 |
| `u_motor_ai` | `motor_max_rpm` | 16'd12000 |
| | `motor_max_torque` | 16'd500 |
| | `motor_nominal_current` | 16'd300 |
| `u_sensor_grace` | `grace_period_count` | 3'd5 |

**Effort.** Small, but there are ten of them — do them in one sitting.
**Risk.** Low. Each one only removes flexibility that was never used.

**What I would do.** All of them, in one pass, same day as A2 and A3. This is
a systematic class of problem in this design, not a set of coincidences.

---

## A7 — the fabric's oversized memory array

**Evidence.** `sensor_interface_fabric_complete.sv` line 215:

```verilog
reg [15:0] mem [0:15];        // 16 entries declared
```

but `AVG_MASK = MOVING_AVG_DEPTH - 1 = 3`, so `ptr` only ever reaches 3 and
**`mem[4]` through `mem[15]` are written at reset and never again.**

```
12 unused entries x 42 channels = 504 dead registers
```

That is the 504 dead `chan[].mem[]` constants in your `WHERE_WE_ARE.md` notes
— exact match, now explained.

**Fix.**

```verilog
reg [15:0] mem [0:MOVING_AVG_DEPTH-1];
reg [$clog2(MOVING_AVG_DEPTH)-1:0] ptr;
```

**Effort.** Small. **Risk.** Low, but re-run the fabric elaboration gate —
changing a `ptr` width touches the mask arithmetic.

**Bonus.** This also removes 504 of the 772 constants that `insert_tiecells`
would otherwise have to handle during physical design.

---

## F2 — `raw_array`: 42 writes, 0 reads

**Evidence.** `ivcu_ev_v3_hybrid_top.sv` lines 394–436 declare and assign it
42 times. Grep finds **43 occurrences total** — one declaration plus 42
assignments, and **not a single read**.

The fabric takes `digital_out_0..11` and `sensor_digital_in_12..41` directly,
never `raw_array`.

**Consequence.** `sensor_digital_in_0..11` are used *only* to build this dead
array, so all 384 of those pins are dead. That accounts exactly for the 864
unconnected `sensor_digital_in` pins you measured:

```
12 ports x 32 bits (channels 0-11, entirely unused)   = 384
30 ports x 16 bits (upper half of channels 12-41)     = 480
                                                        ---
                                                        864   (matches)
```

**Fix.** Delete lines 394–436. If you also do F1 (all 42 analog), then
`sensor_digital_in_0..41` all become redundant and the whole port group goes.

**Effort.** Small. **Risk.** None — deleting something nothing reads.

---

## F3 — no range checking actually happens

**Evidence.** Top level lines 929–948:

```verilog
.expected_ranges_min_0(16'd0),  ...  .expected_ranges_min_9(16'd0),
.expected_ranges_max_0(16'hFFFF), ... .expected_ranges_max_9(16'hFFFF),
```

Every value passes a check of "between 0 and 65535". And only channels 0–9
have range ports at all — the other 32 channels have no range check even in
principle.

**So `sensor_fault` can never be raised by a range violation.** Since
`sensor_fault` drives `sensor_enable_logic`, the grace manager and
`system_health`, a whole safety mechanism is inert.

**Fix.** Decide real per-sensor limits from `defines_ivcu_ev_v3.sv` — you
already have `BATTERY_TEMP_CRITICAL`, `MOTOR_TEMP_CRITICAL`, `RPM_CRITICAL`
and others defined and unused. Make the ranges parameters (see A4) and give
all 42 channels a range.

**Effort.** Medium — it needs real engineering judgement per sensor, not just
editing. **Risk.** Medium: once ranges are real, faults will start firing and
you will find out whether the downstream fault handling works. That is the
point, but expect surprises.

**What I would do.** This is the most *important* functional item on the list
and the one I would not rush. It is fine to defer it past this synthesis run
— but write down that the fault path is currently untested.

---

## F4 — the fault log SRAM is never read

**Evidence.** Top level lines 1298–1299:

```verilog
.rd_en   (1'b0),
.rd_addr (10'd0),
```

You have two OpenRAM 512×32 macros costing **572,456 µm² — 47% of your total
die area** — and nothing ever reads them back.

Writes work: `fault_log_wr_en`, `fault_log_addr` and `fault_log_data` come
from the MCU interface.

**Fix.** Route the MCU's read address and enable through to the logger, and
`fault_log_rd_data_int` back to the AXI read path. Some of this already
exists — `u_mcu_interface.read_data` is already wired to
`fault_log_rd_data_int`.

**Effort.** Small — the wire is already there, `rd_en` and `rd_addr` need
driving from the AXI side.

**What I would do.** Fix it. Half your die is memory you cannot read from.

---

## F5, F6, F7, F8 — smaller interface problems

**F5 — motor control outputs go nowhere.** `torque_command`,
`regen_command`, `steering_assist` are 16-bit internal wires with no
top-level port. The chip computes a torque command and cannot send it. Either
add three output ports, or accept that `u_motor_control` (47,933 µm²) is
another `u_diagnostic`.

**F6 — `fault_log_rd_data`.** A top-level *input* that drives nothing. It was
made an input to silence an elaboration error. Delete the port, or wire it to
a real external log memory.

**F7 — channels 35–41.** Read only by the MCU interface. Seven channels feed
no decision logic. Fine if intended; worth confirming.

**F8 — `s_axi` 64-bit carrying 32-bit.** Bits [63:32] of `s_axi_wdata` are
never read and of `s_axi_rdata` are constant 0. 72 pins. Narrow the bus to 32
bits or accept it as a documented interface choice.

---

## F9 — no scan chain

**Evidence.** `scan_enable` reaches only `clock_manager_14nm`, which has zero
gates and ignores it. `test_done` is tied `1'b1` and `test_fail` to `1'b0` in
the top level.

**You have four DFT pins and no DFT.** After tapeout, a manufacturing defect
in the middle of 69,773 cells is undetectable.

**Effort.** Large — scan insertion, chain stitching, ATPG.

**What I would do.** Not in this batch. But put it in `WHERE_WE_ARE.md` as a
known gap, because it is the kind of thing that is very expensive to discover
late.

---

## F10 — clock domain crossings

**Evidence.** `sensor_data[0..41]` is written on `clk_sensor` and read on
`clk_ai`, `clk_aon` and `clk_mcu`. The only synchronisers in the design are
two `sync_cell` instances carrying 2 bits of mode.

The riskiest specific path: `battery_health_score`, `thermal_score`,
`motor_score`, `dynamics_score`, `perception_score` and the five `*_status`
signals cross `clk_ai -> clk_aon` into **`u_central_fsm`, the block that
decides whether the vehicle may move.**

**Three options, in increasing cost:**

1. Document it as accepted. Sensor values change slowly, each has a valid
   strobe, and `set_clock_groups -asynchronous` tells STA not to time it.
2. Synchronise **only** the safety path — `sync_cell` on the ten signals
   entering `u_central_fsm`. Small area, protects the important block.
3. Full handshake per channel. Correct by construction, significant redesign.

**What I would do.** Option 2, this batch. It is a few `sync_cell` instances
on signals that are already only 8 and 4 bits wide, and it protects the one
block where being wrong matters most. Then document the rest as option 1.

---

# PART 4 — RECOMMENDED ORDER

Do them in this order. The ordering is not arbitrary — later items get
cheaper or safer because of earlier ones.

| step | items | why here |
|---|---|---|
| **1** | A1 — remove `u_diagnostic` | biggest win, zero risk, one line |
| **2** | F2 — delete `raw_array` | zero risk, clears the picture for step 4 |
| **3** | A2 A3 A4 A5 A6 — all ten constants → parameters | one mechanical pass, proven mechanism |
| **4** | A7 — fix the `mem` array depth | small, and removes 504 tie cells later |
| **5** | **F1 — extend the ADC to 42, as a generate loop** | must come after step 3 or it costs 155,000 µm² |
| **6** | F4 — connect the fault log read path | half your die is unreadable memory |
| **7** | F5 F6 — motor control outputs, drop the dead input | small interface cleanup |
| **8** | F10 option 2 — sync the safety path | small, protects the critical block |
| **9** | F3 — real sensor ranges | needs judgement, do it last or defer |
| — | F7 F8 F9 | note in `WHERE_WE_ARE.md`, do not do now |

**Steps 1–4 are all "small" and all low risk. If you only do those, you
should still see a large area reduction** — and you will find out what the
constants class is really worth, which tells you how to think about step 5.

---

# PART 5 — THE RERUN, FROM SCRATCH

## 5.1 Before you touch anything

```bash
cd ~/final_ivcu_project
git add -A && git commit -m "checkpoint before RTL batch v4"
git tag pre-rtl-batch-v4
```

Your notes record that git already saved this project once, when
`run_synthesis_v2.sh` was truncated to zero bytes.

## 5.2 After each edit, not at the end

```bash
scripts/verify_all.sh
```

Seven gates in about a minute. Gate 5 elaborates each module (catches multiple
drivers), gate 6 elaborates the whole design (catches top-level wiring). A
mistake costs you sixty seconds instead of five hours.

**Run it after every single item on the list, not once at the end.** If you
batch ten changes and gate 6 fails, you do not know which one did it.

## 5.3 Synthesis

```bash
pgrep -c yosys          # MUST print 0
free -h                 # check you have headroom
nohup scripts/run_synthesis_v2.sh full > synth_out/run_$(date +%H%M).out 2>&1 &
```

Expect roughly 4–5 hours based on the last run (4:53:29, peak 3.17 GB).
It should be **faster** this time if the area drops.

## 5.4 After synthesis — the checks that matter

```bash
grep "Chip area for top module" synth_out/synth_full_*.log
grep -E "Elapsed \(wall|Maximum resident" synth_out/time_full_*.txt
```

Then the standalone verification, because `check -assert -mapped` at the end
of every run reports thousands of **false** problems and renames the netlist
`REJECTED_*`:

```bash
yosys -p "
read_liberty -lib libs/sky130_fd_sc_hd__tt_025C_1v80.lib;
read_verilog -lib macros/sram_512x32_2port.v;
read_verilog synth_out/fault_log_sram_1024x32_netlist.v;
read_verilog synth_out/<the new netlist>;
hierarchy -check -top ivcu_ev_v3_hybrid_top;
check; stat"
```

`Found and reported 0 problems` is the verdict that counts. Then repoint the
symlink:

```bash
ln -sf <new netlist> synth_out/ivcu_ev_v3_hybrid_top_gate_full.v
```

## 5.5 STA

```bash
sta -no_splash -exit scripts/run_sta_diag.tcl | tee sta_out/diag_v4.txt
```

Compare WNS and TNS against −1.389 / −48.323 ns.

## 5.6 Physical, from the scripts

```bash
openroad -gui
```

```tcl
source scripts/run_floorplan.tcl
```

**Before it runs, update one number.** The die size in section 3 was derived
from 634,059 µm² of cells at 44% utilisation. If your area drops, that
derivation changes — the comments in the script show the arithmetic:

```
new cell band area = new_cell_area / 0.44
new cell band height = that / 1500
new core height = cell band + 441.2 macro band
```

The macros are 572,456 µm² and do not shrink, so the **width stays 1500** —
it is forced by the two 696 µm macros side by side. Only the height moves.

Then:

```tcl
source scripts/run_pdn.tcl
```

Both scripts carry the traps as comments — PDN-0181, PDN-0008, the halo rule,
why not to retune the met5 offset.

---

# PART 6 — MEASURE THE RESULT

Fill this in after the run. The point is to know which change bought what, so
the next design starts from knowledge instead of guesswork.

| | before | after | note |
|---|---|---|---|
| standard cell area | 633,274 µm² | | |
| `u_sensor_fabric` | 244,062 µm² | | A5, A7 |
| `u_diagnostic` | 118,908 µm² | | A1 — expect 0 |
| `u_sensor_grace` | 105,193 µm² | | A2 |
| `u_adc_interface` | 62,111 µm² | | A3 + F1, 12 → 42 channels |
| `u_motor_ai` | 36,857 µm² | | A6 |
| `u_sensor_validation` | 7,219 µm² | | A4 |
| total instances | 69,773 | | |
| buffers in netlist | 7 | | should still be ~7 pre-placement |
| WNS (reset masked) | −1.389 ns | | |
| TNS (reset masked) | −48.323 ns | | |
| synthesis wall clock | 4:53:29 | | |
| peak memory | 3.17 GB | | |
| die | 1520 × 1420 µm | | width fixed by macros |

**The single number to watch** is `u_adc_interface`. If A3 worked, 42 channels
should cost about the same as 12 did, or less. If it went to ~217,000 µm²,
the parameter change did not take effect and you should check it before going
any further.
