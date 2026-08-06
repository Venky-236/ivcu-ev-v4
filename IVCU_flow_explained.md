# IVCU-EV V3 — everything we did, and why

A working reference for the RTL-to-netlist stage of this project.
Written after the second successful synthesis, 4 August 2026.

---

# PART 0 — the shape of the whole flow

You are turning **Verilog text** into a **physical chip layout**. It happens in
stages, and each stage only knows about certain kinds of problem.

```
   RTL (.v / .sv)              your design, written as behaviour
        |
        |  SYNTHESIS  (Yosys)
        v
   GATE NETLIST (.v)           the same design, written as real Sky130 cells
        |                      "connect this NAND to that flip-flop"
        |
        |  STATIC TIMING ANALYSIS  (OpenSTA)     <-- we are here
        v
   answers: is it fast enough? where is it slow?
        |
        |  PHYSICAL DESIGN  (OpenROAD)
        |    floorplan -> placement -> CTS -> routing
        v
   GDSII                       the actual mask layout
```

**The single most important idea in this whole project:**

> Each stage can only fix certain problems. Ask "which stage owns this?"
> before you spend five hours fixing something in the wrong place.

- **Too many gates in series** → only RTL can fix it. Synthesis and placement
  are powerless.
- **A gate too weak for its load** → only physical design can fix it. Rewriting
  RTL does nothing.

Almost every decision we made today came from correctly sorting a problem into
one of those two buckets.

---

# PART 1 — SYNTHESIS

## 1.1 What synthesis actually does

You wrote things like `sum / depth`. There is no "divide" gate in silicon.
Synthesis reads your intent and builds it out of the ~400 real cells in the
Sky130 standard cell library — NAND, NOR, XOR, flip-flops, multiplexers.

It happens in ordered passes. Each pass transforms the design a bit more.

## 1.2 The synthesis script, pass by pass

From `scripts/run_synthesis_v2.sh`, which writes a Yosys script and runs it:

```tcl
read_verilog -sv -DSYNTHESIS $RTL/defines_ivcu_ev_v3.sv
read_verilog $RTL/seq_divider.v
read_verilog -sv -DSYNTHESIS $RTL/ivcu_ev_v3_hybrid_top.sv
...
```
**`read_verilog`** — parse a source file into memory.
- `-sv` = treat as SystemVerilog (needed for `always_ff`, `logic`, `int`)
- `-DSYNTHESIS` = define the `SYNTHESIS` macro, which switches your
  `` `ifdef SYNTHESIS `` blocks to the hardware path instead of the
  simulation path (that is how the SRAM becomes a black box)
- `-lib` = read as a **black box**: take the port list, ignore the body.
  Used for the SRAM macro, where the body is a behavioural model we must not
  synthesise.

**Order matters.** The defines file MUST be first, because every other file
`` `include ``s it. When it was missing, guarded logic silently vanished and we
got 2,327 undriven wires.

```tcl
hierarchy -check -top ivcu_ev_v3_hybrid_top
```
**`hierarchy`** — work out which module is the top and which modules it
instantiates, recursively. `-check` errors if a module is instantiated but was
never read. This is the pass that would have caught a missing
`read_verilog seq_divider.v`.

```tcl
proc
```
**`proc`** — convert `always` blocks into logic. Before `proc`, Yosys just has
a parse tree. After it, it has multiplexers and flip-flops. **This is the pass
that discovers multiple drivers**, because it is where "who assigns this
signal" becomes a real connection.

```tcl
opt_expr        # simplify constant expressions:  a & 1'b1  ->  a
bmuxmap         # convert binary-select muxes to gate form
demuxmap        # convert demuxes to gate form
check -assert   # THE GATE. abort if anything is wrong.
```

```tcl
opt             # general cleanup: dead code, constant folding, dedup
fsm             # detect state machines and re-encode them efficiently
memory          # turn reg arrays into memory structures or flop banks
techmap         # map generic operations onto simple gate primitives
```
**`techmap`** is where `$add`, `$mul`, `$div` become adders built from
full-adder cells. In the log you saw:
```
Using template $paramod\_90_fa\WIDTH=... for cells of type $fa
```
`$fa` = full adder. `$lcu` = look-ahead carry unit. `$alu`, `$pmux` = ALU,
priority mux.

```tcl
dfflibmap -liberty $LIB
```
**`dfflibmap`** — map generic flip-flops onto the *real* flip-flops in the
liberty file. This is why your netlist has `sky130_fd_sc_hd__dfrtp_1`
(D flip-flop, resettable, true output, drive strength 1) rather than a generic
`$dff`.

```tcl
abc -liberty $LIB
```
**`abc`** — the logic optimiser. This is the expensive pass, the one that takes
four of your five hours. It takes the combinational logic between flip-flops
and finds a cheaper way to build it using real cells, balancing area against
delay. `abc -fast` skips the deep search — quicker, worse results.

```tcl
setundef -zero   # tie any remaining 'x' to 0, so P&R never sees undefined
opt_clean        # delete genuinely unused nets and cells
```
**Warning about `opt_clean -purge`:** the `-purge` option also removes unused
*public* (named) wires. On a netlist that destroys port connectivity. We do not
use it.

```tcl
stat -liberty $LIB    # cell counts and area
write_verilog -noattr $NETLIST
check -assert -mapped
```

**Why `write_verilog` comes BEFORE the final check:** after a five-hour run you
always want a file you can inspect, even if the check then fails. The wrapper
renames a failed one to `REJECTED_*` so it can never be confused with a good
one.

## 1.3 Running it

```bash
scripts/run_synthesis_v2.sh full     # full abc, signoff quality
scripts/run_synthesis_v2.sh fast     # abc -fast, for quick iteration
```

Launch detached so a closed terminal cannot kill a five-hour run:

```bash
nohup scripts/run_synthesis_v2.sh full > synth_out/run_$(date +%H%M).out 2>&1 &
```
- `nohup` — ignore hangup signal (terminal closing)
- `>` — send stdout to a file
- `2>&1` — send stderr to the same place
- `&` — run in the background

**Before every launch:**
```bash
pgrep -c yosys      # 0 = safe.  1 = already running, LEAVE IT ALONE.
```
`pgrep -c name` counts running processes with that name. We learned this the
hard way: three concurrent runs on an 8 GB machine forced everything into swap
and all three crawled.

To kill a background job and everything under it:
```bash
kill -- -<PID>      # the leading minus kills the whole process GROUP
```

---

# PART 2 — SIGNALS AND THEIR PROBLEMS

This is the vocabulary. Every error we hit is one of these.

## 2.1 Undriven signal

**"Wire X is used but has no driver."**

Something reads a wire that nothing writes. In hardware the wire floats — its
voltage is undefined, and the gates reading it may output garbage or oscillate.

*Our case:* the `defines` file was not read, so `` `ifdef ``-guarded logic
compiled to nothing, leaving 2,327 wires with readers but no writers.

*Fix:* read the defines file first. 2,327 → 168.

## 2.2 Multiple conflicting drivers

**"multiple conflicting drivers for X."**

Two or more pieces of logic both write the same signal. In silicon that is two
transistor stacks fighting over one wire — one pulling high, one pulling low.
Short circuit, heat, undefined logic value.

*Our case, twice.* Both from the same Verilog subtlety:

```verilog
integer i;                                    // <-- module scope: SHARED

always @(posedge clk) begin : block_a
    for (i = 0; i < 8; i = i + 1) ...         // block_a writes i
end
always @(posedge clk) begin : block_b
    for (i = 0; i < 8; i = i + 1) ...         // block_b ALSO writes i
end
```

A loop counter looks harmless, but `i` is a real register and both blocks drive
it. Same for `confidence_scaled` and `curr_sum_tmp` — scratch variables at
module scope, written by more than one block.

**The fix, and why it works:** Verilog-2001 lets a **named** block hold its own
declarations. Give each block its own copy:

```verilog
always @(posedge clk) begin : block_a
    integer i;                                // <-- block-local, private
    for (i = 0; i < 8; i = i + 1) ...
end
```

An unnamed `begin`/`end` cannot hold declarations — that is why the blocks had
to be labelled first.

**How to spot it:** module-scope declarations sit at exactly 4 spaces of indent;
block-local ones at 8. That is what `verify_all.sh` counts.

## 2.3 The non-blocking assignment trap

Not an error message — a **silent wrong answer**. This one bites hardest.

```verilog
data_filt <= sum / depth;                        // (A)
if (data_filt > threshold)
    data_filt <= data_filt - threshold;          // (B)
```

`<=` means *"at the next clock edge, set this."* It does **not** mean "set it
now."

So:
1. Both A and B get scheduled for the same edge. **The last one wins.** A is
   discarded — the division you paid 130 gate levels for is thrown away.
2. The `data_filt` on the right of B is **last cycle's value**, not A's result.

Your moving-average filter was therefore computing
`previous_output − threshold` every cycle, walking downward with the sensor
data playing no part. A sawtooth, not an average.

**Fix:** one branch, one assignment.
```verilog
data_filt <= (avg > threshold) ? (avg - threshold) : avg;
```

**Rule:** never assign the same signal twice with `<=` in one block unless the
branches are mutually exclusive (`if`/`else if`/`else`).

## 2.4 Width overflow

Also silent. Verilog sizes an expression to the **width of the left-hand side**.

```verilog
reg [15:0] remaining_capacity;
remaining_capacity <= (soc * soh) / 16'd1000;
```

`soc` and `soh` each go to 1000. The true product is 1,000,000. But the whole
expression is evaluated at 16 bits, which hold 65,535 maximum, so it wraps to
16,960 — and you get 16 instead of 1000.

**Fix:** compute the product in a wide enough intermediate.
```verilog
reg [31:0] soc_soh_prod;
soc_soh_prod <= {16'd0, soc} * {16'd0, soh};    // 32-bit, cannot overflow
```

**And the lesson that cost us a second bug:** fixing one overflow can expose
another downstream.

```verilog
estimated_range_km <= (remaining_capacity * 8'd90) / 16'd150;
```
While `remaining_capacity` was wrongly 16, the product 1,440 fit fine. Once it
correctly read 1000, the product became 90,000 — which wraps to 24,464 and
gives 163 km instead of 600.

*Fix:* `90/150` reduces exactly to `3/5`, and `floor(90x/150) == floor(3x/5)`
for all integers. Same answer, dividend stays under 3,000 instead of 90,000.

**Whenever you correct a value's range, check what consumes it.**

## 2.5 High fanout

**"max fanout: limit 32, fanout 15372, slack -15340"**

Fanout = how many gate inputs one output drives. Every input is a small
capacitor. One weak gate charging 15,372 of them is like one person pushing
15,372 shopping trolleys.

We measured it:

| net | fanout | delay | ns per load |
|---|---|---|---|
| `_169_/Y` | 15372 | 1136.681 ns | 0.0739 |
| `_188_/Y` | 1317 | 96.606 ns | 0.0733 |
| `_150_/Y` | 911 | 67.269 ns | 0.0738 |
| `_130_/Y` | 347 | 25.562 ns | 0.0737 |

**Perfectly linear.** Four independent nets, the same constant to three
decimals. That linearity is the proof it is capacitive loading, not logic —
logic does not scale linearly with anything.

**Whose problem is it?** Not yours. Yosys does not build buffer trees; that is
the placer's job. OpenROAD's `repair_design` inserts a tree of buffers so each
one drives a manageable number. **No RTL change can fix this and none should
try.**

## 2.6 Slew, capacitance, and why bad numbers appear

**Slew** (transition time) = how long a signal takes to swing between levels.
A weak gate into a big load has slow slew.

The liberty file characterises each cell over a table of input slews and output
loads. Feed it a value far outside that table and it **extrapolates** — and
extrapolation produces nonsense:

```
library setup time  -41.863     <- a NEGATIVE setup time
library recovery time -421.247
```

Setup time cannot be negative. Those numbers are artifacts of a 1,136 ns slew
on the reset net. **When you see impossible numbers, look for a slew problem
upstream, not a logic problem.**

## 2.7 Clock domain crossing

Your design has four asynchronous clocks: `clk_ai` (100 MHz), `clk_aon`
(10 MHz), `clk_sensor` (50 MHz), `clk_mcu` (200 MHz). A signal crossing between
them can be sampled mid-transition and go **metastable** — settling to an
unpredictable value.

Handled by `sync_cell` (two flip-flops in series) and, in the SDC, by
`set_clock_groups -asynchronous`, which tells STA not to try to time paths
between domains — they are handled by synchronisers, not by timing.

---

# PART 3 — STATIC TIMING ANALYSIS

## 3.1 What STA is

STA checks, without simulating, whether every signal arrives in time. It walks
every path from a flip-flop output to the next flip-flop input, adds the delays,
and compares against the clock period.

```
        SLACK = required time - arrival time

        positive = MET      arrives early enough
        negative = VIOLATED arrives too late
```

**Setup** — data must be stable *before* the clock edge. Failing setup means
the logic is too slow; fix by shortening the path or slowing the clock.

**Hold** — data must stay stable *after* the edge. Failing hold means a path is
too *fast*. Pre-layout hold is meaningless because the clock tree does not exist
yet.

**WNS** (worst negative slack) — the single worst path.
**TNS** (total negative slack) — the sum of all violations. Tells you whether
you have one bad path or thousands.

## 3.2 The critical limitation of PRE-layout STA

There are no wires yet. OpenSTA assumes ideal interconnect: zero resistance,
zero wire capacitance.

- Real post-route timing will be **worse** than this.
- **Passing pre-layout does not mean timing closes.**
- **Failing pre-layout IS meaningful**: if a path is already negative with zero
  wire delay, routing cannot rescue it.

That asymmetry is why pre-layout STA is a screening tool, not a verdict.

## 3.3 Running it

```bash
sta -no_splash -exit scripts/run_sta_diag.tcl 2>&1 | tee sta_out/sta_diag.txt
```
- `-no_splash` — skip the banner
- `-exit` — quit when the script ends instead of dropping to a prompt
- `tee` — show on screen AND save to a file

## 3.4 The STA commands, explained

```tcl
read_liberty  libs/sky130_fd_sc_hd__tt_025C_1v80.lib
read_liberty  macros/sram_512x32_2port_TT_1p8V_25C.lib
```
Cell timing models. `tt_025C_1v80` = typical process, 25 °C, 1.80 V.
**Both** are required. Without the macro liberty, STA sees an unresolved black
box and silently stops timing through the memory.

```tcl
read_verilog synth_out/ivcu_ev_v3_hybrid_top_gate_full.v
read_verilog synth_out/fault_log_sram_1024x32_netlist.v
link_design  ivcu_ev_v3_hybrid_top
```
`link_design` resolves every instance against the libraries. It fails loudly if
a cell is missing.

```tcl
read_sdc SDC/ivcu_ev_v3.sdc
```
The constraints — clock periods, I/O delays, false paths.

```tcl
report_checks -path_delay max -path_group clk_100mhz \
              -group_path_count 25 -slack_max 0.0 -format end
```
- `-path_delay max` — setup (use `min` for hold)
- `-path_group <clk>` — report **per clock domain**. Critical: without it, one
  domain's huge violations crowd out everything else. This is exactly what
  happened with `-group_count 10` in the first run — all ten worst paths were
  reset, and the real problems below were invisible.
- `-slack_max 0.0` — only show violators
- `-format end` — one compact line per path instead of a full gate dump

```tcl
report_check_types -max_slew -max_capacitance -max_fanout -violators
report_worst_slack -max ; report_tns ; report_wns
report_power
```

## 3.5 The masking trick

When the reset nets dominated everything, we could not see the real logic. So:

```tcl
set_false_path -through [get_pins u_reset_sync/_169_/Y]
```
`set_false_path` tells STA "never time paths through here."

**This is diagnostic only and must NEVER go in your real SDC.** It is legitimate
here because we independently proved those nets are pure loading that
`repair_design` will fix. Put it in signoff constraints and you would be lying
to yourself.

Masking revealed the truth immediately: TNS went from −23,843,076 to −6,255,
and underneath sat the real problems in `sensor_fabric`, `battery_ai` and
`perception_ai`.

## 3.6 The SDC bugs we found

**Syntax.** `-max` and `-min` are **flags**, not options that take a value. The
delay is positional.
```tcl
set_input_delay -clock C -max 35.0 -min 0.0 [get_ports ...]   # WRONG, 3 values
set_input_delay -clock C -max 35.0 [get_ports ...]            # RIGHT
set_input_delay -clock C -min  0.0 [get_ports ...]
```

**Direction.** `fault_log_rd_data` was in an input group but is an output in the
netlist. OpenSTA printed 64 warnings and **silently discarded** all of them,
leaving 32 bits untimed through the entire flow. Fixed by moving it to the
output group.

**Lesson: read the warnings.** A discarded constraint does not stop the run — it
just quietly removes coverage.

---

# PART 4 — EVERY PROBLEM WE HIT

| # | Symptom | Root cause | Fix | Owner |
|---|---|---|---|---|
| 1 | 2,327 undriven wires | defines file not read | read it first | script |
| 2 | 168 conflicting drivers | `integer i` etc. at module scope | move into named blocks | RTL |
| 3 | 2,328–2,500 problems at `check -assert -mapped` | Yosys false positive on the in-memory design | verify the **written file** standalone | workaround |
| 4 | Killed, exit 137 | out of memory | `.wslconfig`, quit Docker, one run at a time | environment |
| 5 | Runs crawling, 33% iowait | 3 concurrent synthesis runs, swap thrashing | `pgrep -c yosys` before launching | discipline |
| 6 | OpenRAM assertion at 1024 deep | Sky130 row-mirroring rule | build 2 × 512×32 and wrap | tool limit |
| 7 | `Error 567: requires two positional arguments` | `-max`/`-min` misuse | split into two commands | SDC |
| 8 | 64 × `set_input_delay not allowed on output port` | wrong direction group | move to outputs | SDC |
| 9 | 39.2 ns path, 819,661 µm² | 42 general dividers built because the constant never reached the module | port → **parameter** | RTL |
| 10 | 35.2 / 30.5 ns paths | genuine variable division in one cycle | sequential divider, 32 clocks | RTL |
| 11 | Filter output a sawtooth | double `<=` to `data_filt` | single branch | RTL |
| 12 | Capacity 60× low | 16-bit overflow of `soc*soh` | 32-bit intermediate | RTL |
| 13 | Range 163 km instead of 600 | overflow exposed by fixing #12 | reduce 90/150 to 3/5 | RTL |
| 14 | 11.9 ns path | 32-bit `/1000` — the width cost of fixing #12 | third sequential divider | RTL |
| 15 | Reset paths at −1,539 ns | 15,372 fanout, unbuffered | **none — OpenROAD's job** | placer |

## 4.1 The one that mattered most — problem 9

`ivcu_ev_v3_hybrid_top.sv` line 857 read:

```verilog
.moving_average_depth(4'd4),
```

The divisor was **already the constant 4**. `sum / 4` is `sum >> 2` — free,
pure wiring, zero gates.

But `moving_average_depth` was an **input port**, and the synthesis script never
calls `flatten`. So Yosys built `sensor_interface_fabric_complete` in
isolation, treated the port as an unknown, and constructed the general divider
that handles any value 1–15. The `generate` loop made **42 copies**.

The `4` was sitting right there at the top level. The tool never looked.

**Fix:** make it a `parameter` instead of a port. A parameter is fixed at build
time, so the constant is inside the module before a single gate is made.

**Why not just add `flatten`?** One word, would also work. But your peak memory
was 5.58 GB on an 8 GB machine, and flattening hands `abc` one enormous problem
instead of many small ones. The parameter is deterministic and costs nothing.

Result: 819,661 → 244,062 µm², and the 39.2 ns path vanished.

## 4.2 Problem 3 — the false check failure

Every full run ends with:
```
Found and reported 2500 problems.
ERROR: Found 2500 problems in 'check -assert'.
```
and the wrapper renames the netlist `REJECTED_*`.

**It is a false positive.** The written file is provably clean. The established
workaround is to read the netlist back into a **fresh** Yosys and check it
there:

```bash
yosys -p "
read_liberty -lib libs/sky130_fd_sc_hd__tt_025C_1v80.lib;
read_verilog -lib macros/sram_512x32_2port.v;
read_verilog synth_out/fault_log_sram_1024x32_netlist.v;
read_verilog synth_out/<the netlist>;
hierarchy -check -top ivcu_ev_v3_hybrid_top;
check; stat"
```
`Found and reported 0 problems` — that is the verdict that counts.

The root cause of the in-session false failure is still unknown. The workaround
is reliable and has now been used on three separate netlists.

---

# PART 5 — THE VERIFICATION GATES

`scripts/verify_all.sh` runs seven checks in about a minute, so a mistake costs
you sixty seconds instead of five hours.

| gate | checks | catches |
|---|---|---|
| 0 | files present | forgot to copy something |
| 1 | Unix line endings | Windows CRLF breaking identifiers |
| 2 | script reads `seq_divider.v` | missing module, hours in |
| 3 | variable divisors (warning) | informational only |
| 4 | **simulate the divider, 312 vectors** | wrong arithmetic |
| 5 | each module elaborates | multiple drivers |
| 6 | **whole design elaborates** | top-level wiring |

**A hard lesson about gate 5.** The first version ran `proc; opt; check -assert`
— and that bare `opt` cleaned up the driver conflicts before `check` could see
them. `perception` **passed gate 5 and then failed gate 6 with 72 problems.**

A gate that passes broken code is worse than no gate. Both now use the exact
pass ordering of the real synthesis script.

---

# PART 6 — COMMAND REFERENCE

**Check state**
```bash
pgrep -c yosys                      # 0 = safe to start
free -h                             # memory and swap
synthstat                           # all yosys processes + newest log tail
ps -eo pid,ppid,etime,rss,cmd --sort=start_time | grep [y]osys
```

**Synthesis**
```bash
scripts/verify_all.sh                                   # ALWAYS first
nohup scripts/run_synthesis_v2.sh full > synth_out/run_$(date +%H%M).out 2>&1 &
kill -- -<PID>                                          # kill a job group
```

**After synthesis**
```bash
grep -E "Elapsed \(wall|Maximum resident" synth_out/time_full_*.txt
grep "Chip area for top module"           synth_out/synth_full_*.log
# then the standalone verify from 4.2, then repoint the symlink:
ln -sf <new netlist> synth_out/ivcu_ev_v3_hybrid_top_gate_full.v
```

**STA**
```bash
sta -no_splash -exit scripts/run_sta_diag.tcl       | tee sta_out/diag.txt
sta -no_splash -exit scripts/run_sta_mask_reset.tcl | tee sta_out/mask.txt
```

**Git — this saved the project once**
```bash
git add -A && git commit -m "..."
git show HEAD:scripts/run_synthesis_v2.sh      # view a file as committed
git checkout HEAD -- scripts/run_synthesis_v2.sh  # restore a clobbered file
```
When `run_synthesis_v2.sh` was accidentally truncated to zero bytes, git
restored it in one command. Commit before every long run.

---

# PART 7 — RESULTS

| | before | after |
|---|---|---|
| standard cell area | 1,189,022 µm² | **633,274 µm²** |
| `sensor_interface_fabric` | 819,661 µm² | 244,062 µm² |
| with both SRAM macros | 1.76 mm² | **1.21 mm²** |
| real WNS (reset masked) | −32.391 ns | **−1.389 ns** |
| real TNS (reset masked) | −6,255.832 ns | **−48.323 ns** |
| synthesis wall clock | 6:30:22 | 4:53:29 |
| peak memory | 5.58 GB | 3.17 GB |
| known functional bugs | 4 | 0 |

## What remains, and why we are moving on

| violation | what it really is | who fixes it |
|---|---|---|
| `mcu_interface` −0.654 × ~60 | one `nor2` with **68 loads** — 2.956 of 5.223 ns | `repair_design` |
| `reset_sync` −1.389, −0.818 | required time reads **−0.122 ns**, an impossible number — slew artifact | buffering |
| `battery_ai` −0.902 × 3 | 48 gates, **all minimum size** | resizer |
| `motor_ai` −0.027 × 3 | 27 picoseconds | noise |

**The test we applied:**

```
sensor_fabric was:  39.2 ns / 20 ns  =  96% over, 130 gates  -> RTL problem
battery_ai now:     10.5 ns / 10 ns  = 4.9% over,  48 gates  -> tool problem
```

96% over with 130 gates in series cannot be fixed downstream — no amount of
buffering removes gates from a chain. 4.9% over with 48 minimum-size cells is
routine work for the resizer, which typically recovers 10–15%.

Nothing remaining is "too many logic levels." Every one is drive strength or a
measurement artifact. That is the whole reason we are moving to physical design
rather than doing another five-hour run.

**And to be honest about the risk:** routing adds wire delay, so −0.902 could
get worse before the resizer claws it back. Neither effect is predictable from
here — which is precisely the argument for measuring after placement instead of
guessing. If post-CTS timing still shows `battery_ai` negative, we come back to
the RTL and fix it with real data. Iterating between logical and physical design
is normal practice, not failure.

---

# PART 8 — NEXT

1. **Floorplan** — die size, core area, macro placement. The two SRAMs are
   286,228 µm² each, close to half your total, so their placement drives the
   die shape.
2. **Power distribution network** — the grid that feeds every cell, and the
   macro power connections OpenRAM left for the physical tool.
3. **Placement** — `global_placement`, then `repair_design` (this is where the
   reset fanout gets fixed), then `detailed_placement`.
4. **Clock tree synthesis** — real clock distribution. After this, hold timing
   becomes meaningful for the first time.
5. **Routing** — real wires, real parasitics.
6. **Signoff STA** — with extracted RC. This is the number that counts.
