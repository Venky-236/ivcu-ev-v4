# IVCU-EV V4 — Intelligent Vehicle Control Unit

A dual-mode (car / motorcycle) electric-vehicle control SoC, taken from RTL to layout on a **fully open-source 130 nm flow** — Yosys, OpenRAM, OpenROAD and OpenSTA on the SkyWater SKY130 PDK.

**Status:** RTL through detailed routing complete. **Routed DRC-clean — 0 violations.** Remaining for full signoff: LVS, antenna checking, post-route STA on extracted parasitics, GDS streamout.

**Closes at WNS +7.951 ns / WHS +0.045 ns with zero setup, hold and recovery/removal violators, zero global-routing overflow on all five metal layers, and zero detailed-routing DRC violations.**

---

## What it does

IVCU-EV V4 is a safety-oriented vehicle control unit for an electric platform that runs **two vehicle personalities on the same silicon** — car and motorcycle. It resolves its mode at power-on and inhibits the drivetrain until the mode is proven.

It monitors **64 sensor channels**, scores vehicle health per domain, arbitrates operating mode, controls the high-voltage system, logs faults to on-chip SRAM, and assembles an emergency SOS telemetry frame after a crash.

## Architecture

Four asynchronous clock domains, chosen for safety rather than speed:

| Domain | Frequency | Owns |
|---|---|---|
| `clk_aon` | 10 MHz | Always-on safety island, mode manager, serviceability, HV control |
| `clk_sensor` | 25 MHz | 64-channel ADC acquisition, conditioning, plausibility, fault detection |
| `clk_ai` | 50 MHz | Seven-block health-scoring cluster |
| `clk_mcu` | 50 MHz | APB register file, fault logger + SRAM, service guidance |

Five reset domains. All four clocks arrive as primary inputs — no PLL, no divider, no clock gating in RTL.

### Design decisions worth reading the code for

**The HV safety island is architecturally isolated, not merely careful.** It has its own power domain with no enable pin — it cannot be gated in any sleep state. Its reset comes from power-on reset alone, so a watchdog bite cannot clear a welded-contactor or fired-pyro latch; those are facts about the physical vehicle and survive until a real power cycle plus an authenticated service write. It runs on the *slowest* clock, because that is where the timing margin is. And it accepts no inbound authority: no AI block and no MCU write can command high voltage *on*. The crash disconnect is a combinational override on the drive outputs, deliberately outside any FSM.

**Access control enforced by synthesis, not by a runtime check.** Exactly 8 of 64 sensor channels may ever be service-bypassed. The guard is `bypass_active = permit_grants & BYPASS_ELIGIBLE` against a compile-time constant, so on the other 56 bits the logic is provably constant and Yosys deletes it. Brake pressure is not "protected by a check" — after synthesis there is no gate to attack. The constant is rebuilt from the sensor attribute table and compared at elaboration, so a mismatch fails the build rather than shipping.

**No divider, no multiplier, anywhere.** Health scoring weights are powers of two, so weighting is a shift. The scorer walks one channel per clock — 64 clocks at 50 MHz is 1.28 µs, comfortably inside the 10.24 µs sensor sweep that triggered it — instead of building a 64-term combinational adder tree that would sit on the critical path for a result that changes once per sweep.

**Three CDC patterns, each used on purpose.** A *pulse* synchroniser for a 40 ns strobe crossing into a 100 ns domain, where a level synchroniser would silently drop it. A *bit* synchroniser for every single-bit control. And a documented *quasi-static* pattern for wide buses (1024-bit sensor array, 192-bit status) where only the control pulse crosses synchronised — avoiding roughly 1,200 flip-flops for a hazard those paths do not have. The one place a coherent snapshot **is** paid for is the HV precharge comparison, because a mixed snapshot there welds a contactor.

**A boundary rework that made physical design tractable.** Multiplexing acquisition through an ADC scan sequencer took the top level from 1,344 sensor pins to **237 pins total, every one named, commented and connected** — 24.6 µm per pin on the die perimeter instead of 4.3.

### Enforced structural rules

Nine rules applied across all RTL, checked by `scripts/check_defs.py`:

| | |
|---|---|
| **R1** | No threshold, limit, timeout, coefficient or mask is ever a module port — all are compile-time constants |
| **R2** | Every module output is connected at every instantiation |
| **R3** | Every declared signal has at least one reader |
| **R4** | Array depth derives from its parameter; pointer width via `clog2` |
| **R5** | No `/` and no `%` operator anywhere |
| **R6** | Repetition uses `generate`, never copy-paste |
| **R7** | Every cross-domain signal passes through a CDC synchroniser |
| **R8** | No module name references a process node |
| **R9** | Every top-level port is named, commented and connected |

R1 exists because in an earlier revision ten port-passed constants prevented constant folding and cost **575,000 µm²** in the sensor fabric alone — synthesis built general-purpose comparators and multipliers for every channel. R5 exists because a sequential divider had dominated the STA critical path.

## Flow and tools

```
Verilog-2001 RTL
    │
    ├─ Yosys + ABC ──────────► gate-level netlist    (sky130_fd_sc_hd)
    │
    ├─ OpenSTA ──────────────► constraint validation
    │
    ├─ OpenRAM ──────────────► sram_512x32_2port macro
    │
    └─ OpenROAD
         ├─ floorplan          die 1230 × 1060 µm, macro placed and locked
         ├─ physical cells     cut_rows → endcaps → tapcells
         ├─ PDN                met1 followpins + met4/met5 straps
         ├─ global placement   timing-driven, routability-driven
         ├─ detailed placement legalization
         ├─ CTS                TritonCTS, 5 clock nets
         ├─ hold closure       repair_timing
         └─ global routing     met1–met5 signal, met3–met5 clock
```

| Tool | Version / target |
|---|---|
| PDK | SkyWater SKY130 HD — `sky130_fd_sc_hd`, `tt_025C_1v80` |
| Synthesis | Yosys + ABC |
| Memory compiler | OpenRAM |
| Place & route | OpenROAD `26Q3-850` |
| Timing | OpenSTA |
| Language | Verilog-2001 / IEEE 1364-2005 |

## Results

### Design

| | |
|---|---|
| RTL modules | 41 |
| Sensor channels | 64 |
| Clock domains | 4 asynchronous |
| Top-level pins | 237 (65 port declarations) |
| Post-synthesis area | 322,160 µm² (40 s runtime) |
| Post-synthesis netlist | 38,238 instances · 38,356 nets · 143,284 connections |

### Physical

| | |
|---|---|
| Die | 1230 × 1060 µm |
| Core | 40.02–1189.56 × 40.80–1020.00 µm |
| Site | `unithd`, 0.46 × 2.72 µm |
| SRAM macro | `sram_512x32_2port`, 696.02 × 411.235 µm = 286,228 µm² |
| Tap cells / endcaps | 10,380 / 2,134 |
| Rows after `cut_rows` | 541 |
| Final instances | 55,378 standard cells + 1 macro + 2 tie cells |
| Design area / utilization | 682,435 µm² / 61 % |

### Clock tree

| Clock | Period | Insertion delay | Setup skew | Hold skew |
|---|---|---|---|---|
| `clk_ai` | 20 ns | 1.002 ns | 0.019 ns | 0.016 ns |
| `clk_mcu` | 20 ns | 1.131 ns | 0.024 ns | 0.111 ns |
| `clk_sensor` | 40 ns | 1.284 ns | 0.093 ns | 0.093 ns |
| `clk_aon` | 100 ns | 1.143 ns | 0.145 ns | 0.117 ns |

427 clock buffers · 331 dummy loads · 3 delay buffers · 4,938 sinks across 5 clock nets.

### Timing closure

| Gate | Result |
|---|---|
| Worst setup slack | **+7.951 ns** |
| Worst hold slack | **+0.045 ns** |
| Setup violators | **0** |
| Hold violators | **0** |
| Recovery / removal violators | **0** |
| `check_placement` | pass (silent) |
| Legality idempotence | 0.0 µm displacement on re-run |
| Forbidden-cell census | 0 |

Hold closed with 302 buffers at **+0.2 % area and zero setup cost**. Reports in `reports/sta/`.

### Global routing

| | |
|---|---|
| Overflow (max H / max V / total) | **0 / 0 / 0** |
| Total utilization | 27.86 % |
| Per-layer | met1 39.4 % · met2 38.5 % · met3 14.0 % · met4 9.0 % · met5 1.4 % |
| Wirelength | 2,728,425 µm |
| Vias | 312,923 |
| Nets | 42,650 |
| Runtime | 8 s |

Re-run with **30 % track derating on all five layers as a stress test: still zero overflow**, at +3.3 % wirelength. The router climbed the stack rather than squeezing — met1 demand fell 16 % while met3 rose 78 %, met4 111 % and met5 534 % into layers that were nearly idle.

### Detailed routing

TritonRoute converged to a clean result over successive optimization passes:

```
Completing 100% with 10 violations.
[INFO DRT-0199]   Number of violations = 6.
[INFO DRT-0199]   Number of violations = 6.
[INFO DRT-0199]   Number of violations = 0.
```

**Final: 0 DRC violations.** `pd_v4/stage5_route_drc.rpt` is empty because TritonRoute writes violations into it and there were none.

This clears open issue **O1** — the macro pin/track misalignment carried since floorplan. Only 2 of 103 met4 and 1 of 14 met3 SRAM pins could ever align to routing tracks (pin pitch 5.84 µm against a 0.92 µm track pitch gives a 23-pin alignment period), and the chosen mitigation was to let the router jog rather than move the macro. It worked.

## Three problems worth writing down

**A power grid that reported 100 % of shapes unconnected.** `check_power_grid` returned every shape orphaned on both nets — 1,960 VDD and 2,492 VSS, *exactly* the census counts. That equality is the tell: a graph partition cannot orphan precisely everything, so zero edges had been traversed, which is a statement about the whole net rather than a layer boundary. Root cause: rows were never cut around the macro, so `add_pdn_stripe -followpins` — which follows *rows*, not blockages — drew 151 met1 power rails straight across the SRAM's own met1 obstruction. `cut_rows -halo_width_x 40 -halo_width_y 40` before `place_endcaps` fixes it; both nets then report `PSM-0040 All shapes connected`.

**A −1.094 ns hold failure that was not a clock-tree problem.** Skew could account for at most −0.09 ns, so the number could not have come from CTS. One query settled it — `report_checks -path_delay min -from [all_registers -clock_pins]` returned **"No paths found"**: not a single register-to-register hold violation existed. All 278 violating endpoints launched from input ports carrying `set_input_delay -min 0.0`, and the one domain with no boundary constraint at all had zero violations. `-min 0.0` asserts an external driver changes data at the exact instant the clock edge arrives, which no CMOS part can do. Correcting it to 0.5 ns removed 0.5 ns as a *modelling* fix; `repair_timing -hold` paid the remaining ~0.6 ns with 302 buffers that cost zero setup, because every one landed on an input path holding 13–65 ns of unbudgeted period.

**A detailed-routing blocker that reported success.** `detailed_route` refused to start: `DRT-0305 — Net zero_ of signal type GROUND is not routable`. The netlist had reached routing with constants never tied off — two nets, 22 connections, every one a sink with no driver, including five top-level output ports. `insert_tiecells` fixed it and reported success; nets were correctly retyped, timing was untouched, the instance count was exactly right. But the cells had been created unplaced at (0,0) and the legalizer *silently skipped them*, reporting 0.0 µm displacement against a census two cells short. Only `check_placement` caught it (`DPL-0006`, `DPL-0033`). A tool reporting success is not evidence that it worked.

## Repository layout

```
RTL_V4/          41 Verilog-2001 modules + ivcu_defs.vh  ← the current design
SDC/             timing constraints
scripts_v4/      synthesis, floorplan, STA and verification scripts
                 check_defs.py — rule checker for R1–R9
                 filter_liberty.py — builds the 355-cell filtered library
macros/          OpenRAM sram_512x32_2port views (.v, .lef, TT/SS/FF .lib, .gds)
sta_out_v4/      OpenSTA timing reports
qc_v4/           lint, rule check, elaboration, area probe
synth_out_v4/    Yosys synthesis log and runtime
pd_v4/           physical design reports — DRC, congestion, guide coverage

RTL/             V3 — the previous revision, kept for comparison
scripts/         V3 flow scripts
```

### Why V3 is still here

The V3 sources are retained deliberately. The commit history documents the transition, and the difference is the most instructive part of the project:

| | V3 | V4 |
|---|---|---|
| Post-synthesis area | 1,189,022 µm² → 629,098 µm² | **322,160 µm²** |
| Top-level pins | 1,344 sensor pins (864 unconnected) | **237, all connected** |
| SRAM macros | 2 (unreadable — `rd_en` tied low) | **1, read/write** |
| Dividers | 42 in the sensor fabric + a sequential divider on the critical path | **0** |
| CDC synchronisers | 2 in the entire design | Every crossing (rule R7) |
| HV disconnect | none — `motor_enable` low, pack stays live | **Dedicated safety island** |

V3 reached synthesis and STA. It was not a failed build; it was a build whose problems only became visible under physical implementation.

## Reproducing

You will need the [SkyWater SKY130 PDK](https://github.com/google/skywater-pdk), [Yosys](https://github.com/YosysHQ/yosys), [OpenROAD](https://github.com/The-OpenROAD-Project/OpenROAD) and [OpenSTA](https://github.com/parallaxsw/OpenSTA). Point the scripts in `scripts/` at your PDK install and run synthesis first; the physical flow is executed interactively in OpenROAD, stage by stage, checkpointing with `write_db` at each stage boundary.

`write_db` rather than `write_def` is deliberate — DEF cannot carry voltage domains, PDN grid definitions or global-connect rules, so ODB is the only lossless checkpoint in this flow.

## Not tracked here

Layout databases (`.odb`, DEF) and gate-level netlists are excluded from version control. They are generated output, they regenerate from the scripts in this repository, and together they are roughly 400 MB. The PDK is likewise not vendored — install SkyWater SKY130 separately and point the scripts at it.

The physical-design *reports* are kept, since they are the evidence behind the numbers above.

## Current status and known limitations

Stated plainly, because a repository that lists none tends to have more.

**Remaining for full signoff:**

- LVS, antenna checking, post-route STA on extracted parasitics, and GDS streamout. Detailed routing is DRC-clean; these are the steps after it.

**Known issues in the design:**

- **No functional testbench.** The RTL has been linted, elaborated, rule-checked and physically implemented, but never simulated. Nothing here verifies that HV disconnect fires or that the permit counter counts. This is the top open item, and it is the reason the constant-folding audit below matters.
- **The OpenRAM macro Liberty is uncharacterized.** All 40 delay, transition and constraint tables consist of three identical rows, and six `internal_power` blocks carry values that make `report_power` unusable — which currently blocks IR-drop analysis. Timing arcs are unaffected; the fix is a per-block edit, not a re-characterization.
- **The SRAM is dual-port and one port is dead.** `csb1` is tied high and `addr1` tied low, so the design pays a full two-port footprint to use one port. A single-port macro would be smaller, easier to power and less hostile to routing.
- **One unsynchronised clock crossing.** `ivcu_gps_receiver` samples `gps_rx_data`/`gps_rx_valid` directly, violating rule R7. It is documented in the SDC and is the source of 192 of the 278 boundary hold endpoints.
- **Two chip outputs are provably constant.** The routing-stage tie-off audit showed synthesis had folded `speed_limit_kph[0]` and `power_derate_pct[4]` to zero — meaning the speed limit can only express even values and the derate percentage cannot represent 16–31 or 48–63. Either deliberate quantisation or an RTL defect; unresolved because there is no simulation to distinguish them.
- **Single-corner signoff only.** TT at 25 °C, 1.8 V. SS and FF liberties exist but multi-corner has not been run.
- **No SI or crosstalk analysis.** OpenROAD does not provide it, so clock spacing decisions here are defensive rather than validated.

## License

To be added.

---

*Independent design project, 2026.*
