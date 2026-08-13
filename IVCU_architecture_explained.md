# IVCU-EV V3 — the design itself, explained

What the chip does, what every module is, which signal goes where, and what
all of that means for physical design.

Written 6 August 2026, from the RTL source, the synthesis log and the SDC.
Companion to `IVCU_flow_explained.md` (synthesis and STA) and
`IVCU_floorplan_explained.md` (floorplan and power).

Every number here was read out of a file. Where something is my inference
rather than a fact in the source, it says so.

---

# PART 0 — READ THIS FIRST

## 0.1 What the chip is

An **Integrated Vehicle Control Unit** for an electric vehicle that can be
either a car or a motorbike. It reads 42 sensors, runs six health-monitoring
blocks over them, decides whether the vehicle is safe to operate, and drives
the actuators and an MCU interface.

## 0.2 The one-line summary of the architecture

> 42 sensors come in, get filtered into one 42-entry array called
> `sensor_data[]`, and **every AI block reads from that same array.**

That single array is the spine of the whole design. Understanding it is 80%
of understanding the chip.

```
42 pins  ->  ADC / direct  ->  fabric filter  ->  sensor_data[0..41]  ->  6 AI blocks
                                                          |                    |
                                                          |                    v
                                                          |            central safety FSM
                                                          +-> MCU AXI          |
                                                          +-> fault logger     v
                                                                        actuator outputs
```

## 0.3 The four clocks, in one table

| clock | period | domain name | what runs on it |
|---|---|---|---|
| `clk_200mhz_mcu` | 5 ns | `clk_mcu` | AXI interface only |
| `clk_100mhz` | 10 ns | `clk_ai` | the six AI blocks, ADAS, motor control, diagnostics |
| `clk_50mhz_sensor` | 20 ns | `clk_sensor` | ADC, fabric, validation, enable logic, grace timers |
| `clk_10mhz_aon` | 100 ns | `clk_aon` | power, mode, safety FSM, crash, emergency, fault log |

**AON** means "always on" — the domain that stays powered when everything else
is switched off. That is why crash detection and the safety FSM live there.

All four are **mutually asynchronous**. There is no PLL and no clock division:
`clock_manager_14nm` is a pass-through with **zero gates**.

---

# PART 1 — THE 42 SENSORS

## 1.1 The two paths in

This is the single most misunderstood thing in the design, so it goes first.

**Channels 0–11 come in as analog.** They arrive on `sensor_adc_in_0..11`
(12 bits each) and pass through `adc_interface_14nm`, which converts them to
32-bit digital values.

**Channels 12–41 come in already digital.** They arrive on
`sensor_digital_in_12..41` (32 bits each) and go **straight to the sensor
fabric**, bypassing the ADC entirely.

You can see it in the top-level wiring of `u_sensor_fabric`:

```verilog
.sensor_raw_in_0  (digital_out_0),        // from the ADC
...
.sensor_raw_in_11 (digital_out_11),       // from the ADC
.sensor_raw_in_12 (sensor_digital_in_12), // straight from the pin
...
.sensor_raw_in_41 (sensor_digital_in_41), // straight from the pin
```

**So `sensor_adc_in_12` through `sensor_adc_in_41` connect to nothing that
produces an output.** The ADC module accepts all 42 analog inputs but only
declares 12 digital outputs. Its own source says so:

```verilog
// Note: ADCs 12-41 are ignored because we have only 12 outputs.
```

That is the origin of the 390 unconnected pins in your floorplan notes
(30 channels × 12 data bits + 30 valid bits). It is **not a wiring mistake** —
it is a deliberate 12-output module wired to a 42-input port list. Whether
that was the intent is a design question, not a bug in the connections.

## 1.2 The complete sensor map

Indices come from `defines_ivcu_ev_v3.sv`. "Consumers" is who actually reads
`sensor_data[i]`.

| # | name | in via | consumers |
|---|---|---|---|
| 0 | batt_cell_temp | ADC | battery, thermal, fault log, MCU |
| 1 | batt_pack_temp | ADC | battery, thermal, fault log, MCU |
| 2 | motor_temp | ADC | thermal, motor, motor control, fault log, MCU |
| 3 | inverter_temp | ADC | thermal, motor, fault log, MCU |
| 4 | ambient_temp | ADC | battery, MCU |
| 5 | batt_cell_volt | ADC | battery, MCU |
| 6 | batt_pack_volt | ADC | battery, motor (×2), MCU |
| 7 | charging_volt | ADC | battery, MCU |
| 8 | batt_current | ADC | battery, motor, MCU |
| 9 | charging_current | ADC | battery, MCU |
| 10 | coolant_flow | ADC | thermal, motor (×2), MCU |
| 11 | soc (state of charge) | ADC | battery, motor control, MCU |
| 12 | soh (state of health) | **digital** | battery, MCU |
| 13 | crash_impact | digital | crash AI, MCU |
| 14 | gyroscope | digital | motor, dynamics, crash, MCU |
| 15 | imu | digital | motor, dynamics, crash, MCU |
| 16 | wheel_speed | digital | motor (×3), dynamics (×2), ADAS, motor control, MCU |
| 17 | motor_rpm | digital | motor, motor control, MCU |
| 18 | rotor_position | digital | motor, MCU |
| 19 | throttle_position | digital | motor, ADAS, motor control, MCU |
| 20 | brake_pressure | digital | motor, ADAS, motor control, MCU |
| 21 | brake_switch | digital | motor (**bit [0] only**), MCU |
| 22 | steering_angle | digital | dynamics, ADAS, MCU |
| 23 | side_stand | digital | dynamics, MCU |
| 24 | gear_position | digital | dynamics, MCU |
| 25 | cooling_press | digital | thermal, MCU |
| 26 | humidity | digital | thermal, MCU |
| 27 | enclosure_press | digital | thermal, MCU |
| 28 | ultrasonic | digital | perception, ADAS, MCU |
| 29 | camera | digital | perception, ADAS, MCU |
| 30 | radar | digital | perception, ADAS, MCU |
| 31 | lidar | digital | perception, ADAS, MCU |
| 32 | gps | digital | perception, ADAS, MCU |
| 33 | tpms | digital | perception, motor (×2), MCU |
| 34 | cabin_temp | digital | thermal, MCU |
| 35 | seat_occupancy | digital | **MCU only** |
| 36–41 | *unnamed* | digital | **MCU only** |

**Note channel 12.** `defines` calls it SOH and `u_battery_ai` reads it as
`soh`, but it enters through `sensor_digital_in_12`, not the ADC. The ADC's
analog `sensor_adc_in_12` is discarded. So state-of-health is a digital input
that happens to sit on the boundary of the analog block.

**Channels 35–41 are read by nothing except the MCU interface.** Seven
channels of data land in an AXI register and are never used by any decision
logic. Not broken, but worth knowing.

## 1.3 Sensors reused for different things

Several AI inputs share one channel because a separate sensor does not exist:

| block | port | actually reads |
|---|---|---|
| motor | `wheel_speed_front` and `wheel_speed_rear` | both channel 16 |
| motor | `motor_coolant_temp` and `inverter_coolant_temp` | both channel 10 |
| motor | `motor_voltage` and `inverter_dc_voltage` | both channel 6 |
| motor | `tire_pressure_front` and `tire_pressure_rear` | both channel 33 |
| motor | `vehicle_speed` | channel 16 again |

**This matters for physical design.** Channel 16 alone drives 8 separate
module inputs. `sensor_data[16]` is a 16-bit bus fanning out to five different
blocks — a natural place for congestion and one of the reasons the AI blocks
want to sit close together.

---

# PART 2 — THE JOURNEY OF ONE SENSOR VALUE

Follow channel 2, `motor_temp`, from pin to actuator.

## Stage 1 — the pin

```
sensor_adc_in_2[11:0]      12-bit analog reading
sensor_adc_valid_2         1 = this reading is fresh
```

Constrained in the SDC to `clk_50mhz_sensor`, input delay max 7.0 ns
(35% of the 20 ns period).

## Stage 2 — the ADC, `u_adc_interface`

Runs on `clk_sensor`. Per channel:

```verilog
if (adc_valid_2) begin
    calibrated[2] = {4'b0, adc_in_2};        // calibration_enable is tied 0
    digital_out_2  <= {16'd0, calibrated[2]};
    digital_valid_2 <= 1'b1;
end else
    digital_valid_2 <= 1'b0;
```

Zero-extends 12 bits to 32. The calibration path (offset + gain multiply)
exists but `calibration_enable` is tied to `1'b0` at the top, so **the
multiplier is built and never used.** 7,947 cells, 62,111 µm².

## Stage 3 — the fabric, `u_sensor_fabric`

The biggest module in the design: 23,603 cells, **244,062 µm², 38% of the
standard-cell area.** It runs one identical pipeline for each of the 42
channels via a `generate` loop.

Per channel, a 4-deep moving average:

```verilog
sum      <= sum - mem[ptr] + raw_array[i][15:0];   // running sum
mem[ptr] <= raw_array[i][15:0];                     // ring buffer
ptr      <= (ptr + 4'd1) & AVG_MASK;                // wrap by masking
data_filt <= (avg_now > deadband_threshold)
             ? (avg_now - deadband_threshold) : avg_now;
```

`avg_now` is `sum[AVG_SHIFT +: 16]` — a bit-select, which is **pure wiring,
zero gates.** That is the fix that took this module from 819,661 µm² down to
244,062. It only works because `MOVING_AVG_DEPTH` is a **parameter**, so the
constant 4 is inside the module before any gate is built.

Note the input narrowing: `raw_array[i][15:0]`. **The upper 16 bits of every
32-bit digital input are discarded.** That is why 864 `sensor_digital_in`
pins have no load — 42 channels × 16 unused bits = 672, plus the rest from
channels the fabric gates off.

Output: `sensor_data[2]`, 16 bits, registered on `clk_sensor`.

## Stage 4 — the consumers

`sensor_data[2]` now fans out to four places at once:

```
u_thermal_ai     .motor_temp        clk_ai
u_motor_ai       .motor_temp        clk_ai
u_motor_control  .motor_temp        clk_ai
u_fault_logger   .sensor_data_2     clk_aon
u_mcu_interface  .sensor_data_2     clk_mcu
```

**This is a clock domain crossing and it is not synchronised.** The fabric
writes on `clk_sensor` (50 MHz); the AI blocks read on `clk_ai` (100 MHz).
The SDC declares all four domains asynchronous with `set_clock_groups`, so STA
never times these paths — but there is no `sync_cell` on the data either.
See Part 6.4; this is a real finding.

## Stage 5 — the decision

`u_motor_ai` produces `motor_score` (8 bits). The top level converts it:

```verilog
assign motor_status = (motor_score >= 8'd80) ? `STATUS_OK :
                      (motor_score >= 8'd50) ? `STATUS_WARNING :
                      (motor_score >= 8'd20) ? `STATUS_CRITICAL : `STATUS_FAULT;
```

`motor_ok` and `motor_status` go to `u_central_fsm` on `clk_aon`, which
decides `vehicle_enable`, `motor_enable`, `brake_control`, `throttle_limit`.
Those are chip outputs.

**Pin to pin, channel 2 crosses three clock domains: sensor → AI → AON.**

---

# PART 3 — EVERY MODULE

Areas and cell counts are from `synth_full_20260804_160929.log`, the live
netlist. Sorted by area, because area is what the floorplan cares about.

| # | instance | module | clock | cells | area µm² | % |
|---|---|---|---|---|---|---|
| 1 | `u_sensor_fabric` | sensor_interface_fabric_complete | sensor | 23,603 | 244,062 | 38.5% |
| 2 | `u_sensor_grace` | sensor_grace_manager_complete | sensor | 12,898 | 105,193 | 16.6% |
| 3 | `u_diagnostic` | diagnostic_report_generator | ai | — | 118,908 | 18.8% |
| 4 | `u_adc_interface` | adc_interface_14nm | sensor | 7,947 | 62,111 | 9.8% |
| 5 | `u_motor_control` | motor_control_hybrid | ai | — | 47,933 | 7.6% |
| 6 | `u_battery_ai` | battery_predictive_ai_complete | ai | 4,808 | 40,121 | 6.3% |
| 7 | `u_motor_ai` | motor_condition_enhanced_complete | ai | 5,008 | 36,857 | 5.8% |
| 8 | `u_adas_controller` | adas_controller_v3 | ai | — | 24,719 | 3.9% |
| 9 | `u_dynamics_ai` | vehicle_dynamics_predictive_complete | ai | 2,708 | 20,159 | 3.2% |
| 10 | `u_perception_ai` | perception_health_ai_complete | ai | 1,796 | 18,426 | 2.9% |
| 11 | `u_crash_ai` | crash_predictive_ai_complete | **aon** | 1,468 | 13,830 | 2.2% |
| 12 | `u_mcu_interface` | mcu_axi_lite_interface | **mcu** | 919 | 13,863 | 2.2% |
| 13 | `u_central_fsm` | central_safety_fsm_v3 | **aon** | 1,159 | 12,078 | 1.9% |
| 14 | `u_emergency` | emergency_response_system | **aon** | 985 | 10,260 | 1.6% |
| 15 | `u_sensor_validation` | sensor_validation_fsm | sensor | 1,184 | 7,219 | 1.1% |
| 16 | `u_thermal_ai` | thermal_management_hierarchical_complete | ai | 614 | 5,797 | 0.9% |
| 17 | `u_system_health` | system_health_ai_complete | ai | 392 | 3,406 | 0.5% |
| 18 | `u_power_domain` | power_domain_controller_v3 | **aon** | 337 | 3,406 | 0.5% |
| 19 | `u_fault_logger` | fault_logger_sram_32kb | **aon** | 339 | 3,378 | 0.5% |
| 20 | `u_mode_controller` | mode_config_enhanced_v3 | aon+ai | 253 | 2,402 | 0.4% |
| 21 | `u_reset_sync` | reset_sync_v3 | **all 4** | 163 | 1,986 | 0.3% |
| 22 | `u_sensor_enable_logic` | sensor_enable_logic | sensor | 170 | 1,954 | 0.3% |
| 23 | `u_clock_manager` | clock_manager_14nm | — | **0** | **0** | 0% |
| — | `u_sync_mode_ai`, `u_sync_mode_sensor` | sync_cell ×2 | — | 4 | 100 | ~0% |
| — | top-level glue | — | — | — | 2,182 | 0.3% |

Percentages are of the 633,274 µm² standard-cell total. The two SRAM macros
add 572,456 µm² on top of that.

Cell counts for `u_diagnostic`, `u_motor_control`, `u_adas_controller` are not
broken out separately in the hierarchy dump; their areas are.

## 3.1 What each one does

**`u_clock_manager` — clock_manager_14nm.** Zero gates. Four assign
statements passing the input clocks straight through, plus `pll_locked` tied
to `1'b1`. There is no PLL. It exists as a placeholder so a real clock
generator can be dropped in later without touching the top level.

**`u_reset_sync` — reset_sync_v3.** The reset tree, and the source of your
biggest timing problem. Details in Part 5.

**`u_power_domain` — power_domain_controller_v3.** Reads `pwr_good`,
`vdd_core`, `vdd_io`, `vdd_ram` and produces `pwr_en_ai`, `pwr_en_sensor`,
`pwr_en_mcu`. Its request inputs are all tied to `1'b1`, so it always turns
everything on. Retention, isolation and level shifters are tied off.

**`u_mode_controller` — mode_config_enhanced_v3.** Car / bike / auto / safe.
Produces `current_mode_aon` and the two sensor maps that decide which channels
are relevant in which mode. The only module reading two clocks directly.

**`u_sensor_enable_logic`.** Combines the mode map, fault flags and grace
expiry into the 42-bit `sensor_enable` mask. This mask gates the fabric — a
channel with its bit clear produces zero.

*This module was missing from the instantiation list at one point. With
`sensor_enable` undriven it defaults to all-zero, which gates off all 42
channels and no sensor data reaches anything. The comment in the top level
records the fix.*

**`u_adc_interface`.** 42 analog in, 12 digital out. See Part 1.1.

**`u_sensor_fabric`.** 42 moving-average filters. See Part 2 Stage 3.

**`u_sensor_grace`.** 42 independent 32-bit timers giving a failing sensor a
grace period before it is declared dead. Second largest module — see Part 7.1
for why, and how to shrink it.

**`u_sensor_validation`.** Range-checks each channel against min/max and
produces the 42-bit `sensor_fault`. Only ranges 0–9 have ports; they are all
tied to `0` and `0xFFFF` at the top, i.e. no range is actually enforced.

**The six AI blocks.** `u_battery_ai`, `u_thermal_ai`, `u_motor_ai`,
`u_dynamics_ai`, `u_perception_ai`, `u_crash_ai`. Each reads a handful of
`sensor_data[]` entries plus their valid bits, and produces an `*_ok` flag, an
8-bit `*_score` and a 4-bit `*_status`. Five run on `clk_ai`; **`u_crash_ai`
runs on `clk_aon`** because crash detection must work when the AI domain is
powered down.

**`u_system_health`.** Combines the five scores and five statuses into one
`system_health_score` and `overall_status`.

**`u_central_fsm` — central_safety_fsm_v3.** The safety authority. Reads
every `*_ok` and `*_status`, `crash_latched`, `emergency_stop`,
`manual_override`, and drives the actuator outputs.

**`u_adas_controller`, `u_motor_control`.** Driver assistance and torque /
regen commands. Note `torque_command`, `regen_command` and `steering_assist`
are computed and **go nowhere** — they are internal wires with no top-level
port.

**`u_emergency`.** Independent emergency path. Shares three outputs with the
safety FSM, OR-combined at the top.

**`u_fault_logger` — fault_logger_sram_32kb.** Wraps the two OpenRAM
512×32 SRAM macros. Only 339 cells of glue; the memory is the macro.

**`u_diagnostic`.** 118,908 µm² — the third largest module — and **every one
of its outputs is unconnected.** See Part 7.2.

**`u_mcu_interface`.** AXI-Lite master and slave. Reads all 42 sensor
channels and every status signal into registers.

---

# PART 4 — CLOCK DOMAINS

## 4.1 What runs where

```
clk_aon    10 MHz   ALWAYS ON
  u_power_domain     u_mode_controller   u_central_fsm
  u_crash_ai         u_emergency         u_fault_logger
  u_reset_sync (aon section)

clk_sensor 50 MHz   SENSOR DOMAIN
  u_adc_interface    u_sensor_fabric     u_sensor_validation
  u_sensor_enable_logic                  u_sensor_grace
  u_reset_sync (sensor section)

clk_ai    100 MHz   AI DOMAIN
  u_battery_ai   u_thermal_ai   u_motor_ai
  u_dynamics_ai  u_perception_ai  u_system_health
  u_adas_controller  u_motor_control  u_diagnostic
  u_reset_sync (ai section)

clk_mcu   200 MHz   MCU DOMAIN
  u_mcu_interface only
  u_reset_sync (mcu section)
```

## 4.2 Why the split is sensible

Each domain runs at the speed its job needs, and no faster. Sensors change
slowly, so 50 MHz. AI arithmetic is the heavy work, so 100 MHz. The AXI bus
must keep up with an external MCU, so 200 MHz. Safety and crash detection must
survive power-down, so they sit on the slow always-on clock.

Slower clocks are not a compromise — a 100 ns period gives `u_central_fsm`
ten times the timing margin of the AI blocks, which is exactly what you want
for the block that decides whether the vehicle may move.

## 4.3 The crossings that ARE handled

Two `sync_cell` instances, both 2-flop synchronisers, both 2 bits wide:

```verilog
sync_cell #(.WIDTH(2)) u_sync_mode_ai      current_mode_aon -> clk_ai
sync_cell #(.WIDTH(2)) u_sync_mode_sensor  current_mode_aon -> clk_sensor
```

Four cells and 100 µm² total. These carry the mode selection out of the AON
domain into the AI and sensor domains.

## 4.4 The crossings that are NOT — a real finding

**`sensor_data[0..41]` and `sensor_data_valid[41:0]` cross from `clk_sensor`
into `clk_ai`, `clk_aon` and `clk_mcu` with no synchroniser at all.**

The consumers:

| destination | clock | what it reads |
|---|---|---|
| six AI blocks | `clk_ai` (×5), `clk_aon` (crash) | selected `sensor_data[]` entries |
| `u_fault_logger` | `clk_aon` | `sensor_data[0..3]` |
| `u_mcu_interface` | `clk_mcu` | all 42 |

The SDC's `set_clock_groups -asynchronous` tells STA not to time these paths.
That is correct — you cannot meaningfully time an asynchronous crossing. But
**not timing a path is not the same as making it safe.** A 16-bit value
sampled mid-change on an unrelated clock can be read as a mixture of old and
new bits.

For slowly-changing temperature readings this is usually tolerable in
practice, and it may well have been a deliberate choice. But it is not
recorded anywhere as a decision, and the same designer *did* add
synchronisers for the 2-bit mode signal — which is a much less risky
crossing. Worth a deliberate answer before tapeout.

Similar crossings, same situation:

```
sensor_fault, sensor_grace_active   clk_sensor -> clk_aon (enable logic, MCU)
battery_health_score et al.         clk_ai     -> clk_aon (central FSM, emergency)
control_signals, system_status      clk_aon    -> clk_mcu (MCU interface)
```

The `clk_ai -> clk_aon` group is the one I would look at first: those feed the
**safety FSM**.

---

# PART 5 — RESET

## 5.1 The structure

Two asynchronous inputs:

```
por_n      power-on reset. The true master. Every always block in
           reset_sync_v3 uses "or negedge por_n".
ext_rst_n  external reset request. Combined in logic, not used directly
           as an async reset.
```

`reset_sync_v3` builds one 2-flop synchroniser plus a 16-cycle hold counter
per domain, giving four outputs:

```
rst_ai_n   rst_aon_n   rst_sensor_n   rst_mcu_n
```

Each is released 16 clock cycles after reset deasserts, so every domain comes
out of reset cleanly and at a defined time. The AON section additionally
debounces the combined reset source over 8 cycles.

## 5.2 Why reset is your worst timing problem

`u_reset_sync` is 163 cells and 1,986 µm² — 0.3% of the design. But it drives
a reset into **every flip-flop in the domain**:

```
8,953  sky130_fd_sc_hd__dfrtp_1     resettable D flip-flops
```

Nearly nine thousand flops, and the netlist contains **7 buffers total**.
One `o21ai_0` gate was measured driving 5,962 reset pins; another net measured
15,372 loads at 1,136 ns.

That is not a logic problem and no RTL change fixes it. `repair_design` builds
the buffer tree. It is listed here so that when you see the reset paths at the
top of every timing report, you know exactly which 163 cells are responsible
and why it is expected.

## 5.3 The bug that was fixed

`clock_manager_14nm` has a `clk_valid[3:0]` output. It was originally wired
to `{rst_ai_n, rst_aon_n, rst_sensor_n, rst_mcu_n}` — which put a **second
driver** on all four reset nets, fighting `reset_sync_v3`. It is now left
unconnected. The comment in the top level records it.

---

# PART 6 — POWER DOMAINS

## 6.1 The trap in the names

```verilog
input wire vdd_core;
input wire vdd_io;
input wire vdd_ram;
input wire pwr_good;
```

**These are not power supplies.** They are ordinary logic inputs — status
flags from an external PMIC saying whether each rail is healthy. They are read
by `power_domain_controller_v3` like any other data signal.

The real supply arrives through the PDN as metal, not through pins. The SDC
correctly false-paths all four:

```tcl
set_false_path -from [get_ports {ext_rst_n por_n vdd_core vdd_io vdd_ram pwr_good ...}]
```

## 6.2 What the controller does

```
inputs:   pwr_good, vdd_core, vdd_io, vdd_ram
requests: pd_req_ai, pd_req_sensor, pd_req_mcu   -- all tied 1'b1
outputs:  pwr_en_ai, pwr_en_sensor, pwr_en_mcu
```

Because the requests are hard-tied high, the controller always enables all
three domains. The switch-off capability exists in the RTL but nothing can
ask for it.

`retention_enable`, `iso_enable` and `level_shifter_en` are all tied `1'b0`.

## 6.3 Power domains in silicon — the honest position

**There is one power domain in the physical design: `CORE`, VDD/VSS.**

The RTL describes four *logical* domains and the enables to switch them, but
none of that is implemented physically. There are no power switches, no
isolation cells, no retention flops and no UPF file. `pwr_en_*` go out as
chip pins for an external PMIC to act on.

That is a legitimate design point — external power gating — but it should be
a stated decision, not an accident. If on-chip gating is ever wanted, it means
UPF, isolation cells, a second voltage domain in `pdngen`, and a lot of
floorplan work.

Interestingly, synthesis *did* infer isolation-style cells:

```
1,937  sky130_fd_sc_hd__lpflow_isobufsrc_1
  477  sky130_fd_sc_hd__lpflow_inputiso1p_1
```

2,414 low-power isolation buffers. These come from `abc` choosing them as
ordinary buffers, not from any power-intent file. Harmless, but do not mistake
them for a power-gating implementation.

---

# PART 7 — WHAT THE HIERARCHY MEANS FOR THE FLOORPLAN

This is the part that connects the RTL to the physical work you are doing.

## 7.1 The area is concentrated in three modules

```
u_sensor_fabric    244,062     38.5%
u_diagnostic       118,908     18.8%
u_sensor_grace     105,193     16.6%
                   -------
                   468,163     74% of all standard-cell area
```

**Three modules are three-quarters of your chip.** Everything else is noise
by comparison. If placement or congestion goes wrong, it will be here.

### Why `u_sensor_grace` is 105,193 µm²

42 independent timers, each:

```verilog
reg [31:0] grace_timer;
reg [2:0]  fault_count;
if (grace_timer < grace_timeout) grace_timer <= grace_timer + 1;
if (grace_timer == grace_timeout - 1) ...
```

That is 42 × 32-bit counters (1,344 flops) plus 42 × two 32-bit comparators.

**And here is the finding: `grace_timeout` is a PORT, not a parameter.**

```verilog
.grace_timeout (32'd1000000),      // top level ties it to a constant
```

This is **exactly** the bug that cost you 575,000 µm² in
`sensor_interface_fabric_complete` — problem 9 in your flow document. The
constant sits at the top level, synthesis never flattens, so the module is
built to compare against an arbitrary runtime value. Two full 32-bit magnitude
comparators per channel instead of a constant compare.

Changing `grace_timeout` and `grace_period_count` from ports to parameters
should reduce this module substantially. I have not measured it, so I will not
guess a number — but the mechanism is identical and it is the same fix.

### The same pattern elsewhere

Every one of these is a constant tied to a port:

| module | port | tied to |
|---|---|---|
| `u_sensor_grace` | `grace_timeout`, `grace_period_count` | 1000000, 5 |
| `u_sensor_validation` | `validation_timeout`, `hysteresis_threshold` | 1000, 10 |
| `u_sensor_validation` | `expected_ranges_min/max_0..9` | 0, 0xFFFF |
| `u_sensor_fabric` | `filter_coefficients`, `deadband_threshold` | 256, 5 |
| `u_adc_interface` | `offset_correction`, `gain_correction` | 0, 256 |
| `u_motor_ai` | `motor_max_rpm`, `motor_max_torque`, `motor_nominal_current` | 12000, 500, 300 |

**This is a systematic optimisation opportunity, not a single bug.** Each one
is a place where synthesis builds general-purpose arithmetic for a value that
is fixed at build time. Worth doing all of them in one pass when you next
touch the RTL.

## 7.2 `u_diagnostic` — 118,908 µm² that drives nothing

```verilog
diagnostic_report_generator u_diagnostic (
    .clk_ai(clk_ai), .rst_ai_n(rst_ai_n),
    ... 12 inputs ...
    .generate_report      (1'b0),
    .continuous_monitoring(1'b1),
    .report_detail_level  (debug_mode)
);
```

**Look at the port list: there is not a single output connection.** Every
output of this module is either absent from the instantiation or unconnected.

Yet it survives synthesis at 118,908 µm² — the **third largest module in the
design, 18.8% of your standard-cell area.** `opt_clean` did not remove it,
which means Yosys treated its outputs as public and kept the logic.

I cannot tell you from the RTL alone whether this is intended (a block wired
up later) or dead weight. But **you are currently paying about 119,000 µm² —
roughly 19% of your cell area, and a meaningful slice of your 1500×1400 die —
for a module whose results leave no way out of the chip.**

This is the single biggest area question in the design. Worth answering before
the next synthesis run.

## 7.3 Which modules must be placed close together

Physical placement follows connectivity. Here is where the heavy wiring is.

**Cluster A — the sensor pipeline (`clk_sensor`, ~420,000 µm²)**

```
u_adc_interface -> u_sensor_fabric -> u_sensor_validation
                        |                    |
                        +-> u_sensor_grace <-+
                        +-> u_sensor_enable_logic -+
                              ^                    |
                              +--------------------+ (feedback)
```

These four modules exchange 42-bit and 42×16-bit buses. `sensor_enable` is a
42-bit signal from the enable logic back into the fabric and grace manager —
a genuine feedback loop inside the domain. **Keep them together.** This
cluster is two-thirds of your cell area on its own.

**Cluster B — the AI blocks (`clk_ai`, ~150,000 µm²)**

All six read from `sensor_data[]`, all six feed `u_system_health` and
`u_central_fsm`. The `sensor_data` bus is 42 × 16 = **672 wires** leaving
Cluster A and arriving in Cluster B. That is the widest interface in the chip
and the one to keep short.

**Cluster C — safety and always-on (`clk_aon`, ~43,000 µm²)**

```
u_central_fsm  u_emergency  u_crash_ai  u_power_domain
u_mode_controller  u_fault_logger
```

`u_fault_logger` must sit near the SRAM macros — it is the only thing that
talks to them. Your macros are at the top of the die, so this cluster wants
to be up there too.

**Cluster D — MCU (`clk_mcu`, ~14,000 µm²)**

`u_mcu_interface` alone, but it reads **all 42** `sensor_data[]` entries plus
every status signal. It has the widest fan-in in the design while being only
2.2% of the area. It needs to be near the AXI pins (right edge, per your pin
constraints) *and* near the sensor data — those pull in opposite directions.
Expect this to be a placement compromise.

## 7.4 Where deep combinational logic lives

From your STA results, in order of severity:

| module | what | fixable by |
|---|---|---|
| `u_reset_sync` | 5,962-load reset net, 7 buffers in the design | `repair_design` |
| `u_mcu_interface` | one `nor2` with 68 loads, 2.956 ns of 5.223 | `repair_design` |
| `u_battery_ai` | 48 gates, all minimum size, −0.902 ns | resizer |
| `u_motor_ai` | −0.027 ns | noise |

**Two `seq_divider` instances**, one in `u_battery_ai` and one in
`u_perception_ai`, 865 cells and 7,828 µm² each. These are 32-cycle sequential
dividers added deliberately to break single-cycle division out of the critical
path. They are *supposed* to be there.

`u_mcu_interface` is on the 5 ns clock — the tightest in the design — while
also having the widest fan-in. **It is the module most likely to cause you
trouble after routing.** Small, so give it room and short wires.

## 7.5 Fanout hotspots

| net | fans out to | why |
|---|---|---|
| `rst_*_n` | ~9,000 flops | one per domain |
| `sensor_enable[41:0]` | fabric + grace + validation | 42 bits × 3 destinations |
| `sensor_data[16]` | 8 module inputs | wheel speed reused everywhere |
| `sensor_data_valid[41:0]` | every AI block + MCU | 42 bits, wide fan-out |
| `current_mode_ai_sync[1:0]` | 6 modules on `clk_ai` | mode selection |
| `debug_mode[2:0]` | 5 modules across 3 clock domains | crosses domains unsynchronised |

`debug_mode` is worth noting: it is a 3-bit input read by `u_mode_controller`
(aon), `u_sensor_fabric` (sensor), `u_central_fsm` (aon), `u_fault_logger`
(aon), `u_diagnostic` (ai) and `u_mcu_interface` (mcu). It crosses every clock
domain with no synchroniser — but the SDC false-paths it as static
configuration, which is the right call as long as it is only changed while the
chip is idle.

---

# PART 8 — I/O PINS

## 8.1 The full inventory — 2,719 bit-level pins

| group | ports | bits | direction | clock |
|---|---|---|---|---|
| `sensor_digital_in_0..41` | 42 | 1,344 | in | sensor |
| `sensor_adc_in_0..41` | 42 | 504 | in | sensor |
| `sensor_adc_valid_0..41` | 42 | 42 | in | sensor |
| `sensor_digital_valid_0..41` | 42 | 42 | in | sensor |
| `sensor_adc_channel` | 1 | 5 | in | sensor |
| `sensor_enable/grace_active/fault/valid_out` | 4 | 168 | out | sensor |
| m_axi (master) | 19 | ~220 | both | mcu |
| s_axi (slave) | 19 | ~220 | both | mcu |
| clocks | 4 | 4 | in | — |
| resets | 2 | 2 | in | async |
| power status | 4 | 4 | in | false path |
| power enables + state | 4 | 5 | out | aon |
| mode | 4+2 | 8 | both | aon |
| AI status | 6 | 25 | out | ai |
| control outputs | 8 | 8 | out | aon |
| emergency | 3 | 3 | both | aon |
| fault log | 4 | 74 | both | mcu |
| debug | 3 | 36 | both | mcu |
| test | 4 | 4 | both | false path |

## 8.2 Special pins — treat these differently

**Clocks (4).** `clk_100mhz`, `clk_10mhz_aon`, `clk_50mhz_sensor`,
`clk_200mhz_mcu`. Placed at edge centres nearest their logic. These become
clock tree roots at CTS.

**Asynchronous resets (2).** `por_n` is the true master — every reset always
block keys off it. `ext_rst_n` is a request combined in logic. Both
false-pathed at the boundary because they are captured by synchroniser flops.

**Power status (4).** `vdd_core`, `vdd_io`, `vdd_ram`, `pwr_good`. Logic
inputs despite the names. Do not confuse them with supplies.

**DFT (4).** `scan_enable`, `test_mode`, `test_done`, `test_fail`.
`test_done` is tied `1'b1` and `test_fail` to `1'b0` in the RTL — they are
constants, correctly false-pathed. `scan_enable` reaches only
`clock_manager_14nm`, which ignores it. **There is no scan chain in this
design.**

**`debug_mode[2:0]`.** Static configuration crossing all four clock domains.

## 8.3 Pins that accept data the chip ignores

| pins | signal | reason |
|---|---|---|
| 390 | `sensor_adc_in_12..41` + valids | ADC has only 12 outputs |
| 672 | upper 16 bits of every `sensor_digital_in` | fabric reads `[15:0]` only |
| 32 | `s_axi_wdata[63:32]` | RTL reads `[31:0]` |
| 32 | `s_axi_rdata[63:32]` | constant 0 in every branch |
| 32 | `fault_log_rd_data` | declared input, drives nothing |
| 8 | `s_axi_wstrb` | byte strobes unused |
| 12 | `*_axi_*prot` | tied off — normal for AXI-Lite |

`fault_log_rd_data` deserves a note. The top level says:

```verilog
// fault_logger_sram_32kb.rd_data (an output) was wired directly to the
// top-level fault_log_rd_data INPUT port - illegal.
```

The internal SRAM read data now goes to `fault_log_rd_data_int`, which reaches
`u_mcu_interface.read_data`. The top-level input port remains declared but
unused. Also note `u_fault_logger.rd_en` is tied `1'b0` and `rd_addr` to
`10'd0` — **nothing ever reads the fault log SRAM.** Writes work; reads are
disabled.

---

# PART 9 — FINDINGS

Sorted by how much they cost you.

## 9.1 Area

| # | finding | cost | confidence |
|---|---|---|---|
| 1 | `u_diagnostic` — 118,908 µm², no output connected | ~19% of cell area | **high** — read the port list |
| 2 | `grace_timeout` is a port, not a parameter | part of 105,193 µm² | **high** — same mechanism as problem 9 |
| 3 | six more constants-tied-to-ports (Part 7.1) | unmeasured | high |
| 4 | ADC calibration multiplier built, `calibration_enable` tied 0 | part of 62,111 µm² | high |

## 9.2 Function

| # | finding | impact |
|---|---|---|
| 5 | ADC channels 12–41 accepted and discarded | 30 of 42 analog inputs dead |
| 6 | `sensor_data[]` crosses 3 clock domains unsynchronised | Part 4.4 |
| 7 | validation ranges all tied to 0 / 0xFFFF | no range checking happens |
| 8 | fault log read path disabled (`rd_en` tied 0) | log can be written, never read |
| 9 | `torque_command`, `regen_command`, `steering_assist` go nowhere | motor control output unused |
| 10 | channels 35–41 read only by the MCU | 7 channels feed no decision |
| 11 | no scan chain despite DFT pins | not testable by scan |

## 9.3 Already known, carried from earlier documents

- 114 of 117 macro pins off-track
- `battery_ai` −0.902 ns on 3 endpoints
- reset fanout, 7 buffers, `repair_design`'s job
- SRAM LEF `DATABASE MICRONS` fix must be reapplied if memory regenerated

---

# PART 10 — WHAT I WOULD CHECK NEXT

1. **Decide what `u_diagnostic` is for.** 19% of your area rests on the
   answer. If it is meant to report through AXI, it needs connecting. If it
   is not needed yet, remove it and get the area back.

2. **Change the six constant-ports to parameters.** Same fix as the one that
   already saved you 575,000 µm². Do them together in one RTL pass.

3. **Write down the CDC decision.** Either add synchronisers on the
   `clk_ai -> clk_aon` status paths feeding the safety FSM, or record why
   they are safe. Do not leave it undocumented.

4. **Decide whether 42 analog channels are real.** If yes, the ADC needs 42
   outputs. If no, delete 30 port groups and recover 390 pins.

5. **Then re-synthesise once**, with all of it together — which is the
   batching plan you already chose.
