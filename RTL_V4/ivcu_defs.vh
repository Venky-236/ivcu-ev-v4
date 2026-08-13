//=============================================================================
// ivcu_defs.vh   -   IVCU-EV V4 global definitions
//
// Project : IVCU-EV V4  (Intelligent Vehicle Control Unit, dual mode car/bike)
// PDK     : Sky130 HD (sky130_fd_sc_hd, tt_025C_1v80)
// Flow    : Yosys + OpenROAD
// Language: Verilog-2001 / IEEE 1364-2005 ONLY.
//
//-----------------------------------------------------------------------------
// STRUCTURAL RULE R1 - THE MOST IMPORTANT RULE IN THIS PROJECT
//
//   Every threshold, limit, timeout, coefficient and mask in this design is a
//   compile-time constant declared here or as a localparam inside the module
//   that uses it.  NOT ONE of them is a module port.
//
//   In V3, ten constants were passed in through ports.  Synthesis could not
//   see their values, so it built general-purpose comparators and multipliers
//   for every one of 42 channels.  That single mistake cost 575,000 um2 in the
//   sensor fabric alone, plus most of a 105,193 um2 grace manager and a
//   62,111 um2 ADC block.  Constants belong here.  Never in a port list.
//-----------------------------------------------------------------------------
// Other rules enforced across all V4 RTL:
//   R2  every module output is connected at every instantiation
//   R3  every declared signal has at least one reader
//   R4  array depth derives from its parameter, pointer width via clog2
//   R5  no '/' and no '%' operator anywhere (killed the V3 STA divider path)
//   R6  repetition uses generate, never copy-paste
//   R7  every cross-domain signal passes through a cdc synchroniser
//   R8  no module name references a process node
//   R9  every top-level port is named, commented and connected
//=============================================================================

`ifndef IVCU_DEFS_VH
`define IVCU_DEFS_VH

//-----------------------------------------------------------------------------
// 1. IDENTITY
//-----------------------------------------------------------------------------
`define IVCU_ID_CODE        32'h49564355      // "IVCU" in ASCII
`define IVCU_VER_MAJOR      8'd4
`define IVCU_VER_MINOR      8'd0
`define IVCU_VER_PATCH      8'd0

//-----------------------------------------------------------------------------
// 2. GLOBAL WIDTHS
//-----------------------------------------------------------------------------
`define NUM_SENSORS         64              // channels, exactly fills 6-bit index
`define SIDX_W               6              // sensor index width
`define ADC_W               12              // external ADC resolution
`define SVAL_W              16              // internal sensor value width
`define SSTAT_W              3              // per-sensor status code width
`define SCONF_W              4              // per-sensor confidence width
`define SCORE_W              8              // domain / system health score width
`define DSTAT_W              4              // domain status code width
`define APB_ADDR_W          12              // 4 KB register space
`define APB_DATA_W          32
`define LOG_ADDR_W           9              // sram_512x32_2port address width
`define LOG_DATA_W          32

//-----------------------------------------------------------------------------
// 3. CLOCK DOMAINS
//
// Four primary clock inputs.  No on-chip division, no PLL, no clock gating in
// the RTL.  Deliberate: V3 reached synthesis and STA with this arrangement and
// the open-source CTS flow handles four independent roots without complaint.
// Introducing divided clocks here would add flow risk for no functional gain.
//
// FLAGGED CHANGE vs V3: the targets come down.  V3 asked for 200 MHz on the
// MCU domain and reported WNS -1.389 ns / TNS -48.323 ns.  Sky130 HD standard
// cells do not comfortably run general logic at 200 MHz; that number came from
// a 14 nm mindset, same origin as the "_14nm" module names.  Close timing with
// margin first, raise the targets afterwards.
//-----------------------------------------------------------------------------
`define F_AON_HZ            10_000_000      // 100.0 ns  always-on, safety
`define F_SENSOR_HZ         25_000_000      //  40.0 ns  acquisition
`define F_AI_HZ             50_000_000      //  20.0 ns  AI cluster
`define F_MCU_HZ            50_000_000      //  20.0 ns  APB, logger, guidance

//-----------------------------------------------------------------------------
// 4. VEHICLE MODE
//-----------------------------------------------------------------------------
`define MODE_W               2
`define MODE_CAR            2'b00
`define MODE_BIKE           2'b01
`define MODE_DETECT         2'b10           // resolving, vehicle inhibited
`define MODE_SAFE           2'b11           // unresolvable, vehicle inhibited

//-----------------------------------------------------------------------------
// 5. SENSOR CHANNEL INDICES  -  all 64 named, no gaps, no unnamed slots
//
// V3 stopped at index 35.  Six of its 42 channels had no identity at all,
// which is why nothing downstream could reason about them and why the
// floorplan showed pins nobody could name.
//-----------------------------------------------------------------------------

// --- Group 0 : HV safety island (0-5) ---------------------------------------
`define S_HVIL_LOOP              6'd0    // interlock loop continuity
`define S_ISOLATION_RES          6'd1    // pack-to-chassis insulation
`define S_HV_BUS_VOLT            6'd2    // DC link, downstream of contactors
`define S_PRECHARGE_VOLT         6'd3    // DC link during precharge
`define S_CONTACTOR_FB_POS       6'd4    // positive contactor aux contact
`define S_CONTACTOR_FB_NEG       6'd5    // negative contactor aux contact

// --- Group 1 : Battery / BMS (6-17) -----------------------------------------
`define S_CELL_VOLT_MIN          6'd6
`define S_CELL_VOLT_MAX          6'd7
`define S_PACK_VOLTAGE           6'd8
`define S_PACK_CURRENT           6'd9    // offset binary, 2048 = zero current
`define S_CELL_TEMP_MIN          6'd10
`define S_CELL_TEMP_MAX          6'd11
`define S_PACK_TEMP              6'd12
`define S_SOC                    6'd13   // enabled in BIKE mode (V3 bug)
`define S_SOH                    6'd14
`define S_CHARGE_VOLTAGE         6'd15
`define S_CHARGE_CURRENT         6'd16
`define S_PACK_ENCL_PRESS        6'd17   // venting precursor - safety critical

// --- Group 2 : Motor & inverter (18-25) -------------------------------------
`define S_MOTOR_RPM              6'd18
`define S_ROTOR_POSITION         6'd19
`define S_PHASE_CURRENT_A        6'd20
`define S_PHASE_CURRENT_B        6'd21   // phase C derived as -(A+B)
`define S_MOTOR_TEMP             6'd22
`define S_INVERTER_TEMP          6'd23
`define S_DC_LINK_CURRENT        6'd24
`define S_GEAR_POSITION          6'd25   // conditional bypass

// --- Group 3 : Thermal (26-31) ----------------------------------------------
`define S_COOLANT_TEMP_IN        6'd26   // car only, bike is air cooled
`define S_COOLANT_TEMP_OUT       6'd27   // car only
`define S_COOLANT_FLOW           6'd28   // car only
`define S_COOLANT_PRESSURE       6'd29   // car only
`define S_AMBIENT_TEMP           6'd30   // conditional bypass
`define S_HUMIDITY               6'd31   // conditional bypass

// --- Group 4 : Vehicle dynamics (32-43) -------------------------------------
`define S_WSPD_FRONT_A           6'd32   // car front-left  / bike front
`define S_WSPD_FRONT_B           6'd33   // car front-right / not on bike
`define S_WSPD_REAR_A            6'd34   // car rear-left   / bike rear
`define S_WSPD_REAR_B            6'd35   // car rear-right  / not on bike
`define S_ACCEL_LONG             6'd36   // X, offset binary
`define S_ACCEL_LAT              6'd37   // Y, offset binary
`define S_ACCEL_VERT             6'd38   // Z, offset binary
`define S_YAW_RATE               6'd39   // offset binary
`define S_ROLL_RATE              6'd40   // offset binary, bike lean rate
`define S_PITCH_RATE             6'd41   // offset binary
`define S_STEERING_ANGLE         6'd42   // car only
`define S_RIDE_HEIGHT            6'd43   // car only, conditional bypass

// --- Group 5 : Rider / driver input and braking (44-49) ---------------------
`define S_THROTTLE_POS_1         6'd44   // primary track
`define S_THROTTLE_POS_2         6'd45   // redundant track, inverse slope
`define S_BRAKE_PRESSURE         6'd46
`define S_BRAKE_SWITCH           6'd47   // also has a direct pin
`define S_SIDE_STAND             6'd48   // bike only, conditional bypass
`define S_SEAT_OCCUPANCY         6'd49   // conditional bypass

// --- Group 6 : Crash and emergency (50-52) ----------------------------------
`define S_CRASH_FRONT            6'd50
`define S_CRASH_SIDE             6'd51   // car only
`define S_TIP_OVER               6'd52   // bike only

// --- Group 7 : Perception / ADAS (53-57) ------------------------------------
`define S_CAMERA_STATUS          6'd53
`define S_RADAR_STATUS           6'd54   // car only
`define S_LIDAR_STATUS           6'd55   // car only
`define S_ULTRASONIC_STATUS      6'd56   // car only
`define S_GPS_STATUS             6'd57   // BOTH modes - crash SOS needs it

// --- Group 8 : Tyres, cabin, environment (58-63) ----------------------------
`define S_TPMS_FRONT             6'd58   // conditional bypass
`define S_TPMS_REAR              6'd59   // conditional bypass
`define S_CABIN_TEMP             6'd60   // car only
`define S_AMBIENT_LIGHT          6'd61   // car only
`define S_RAIN_SENSOR            6'd62   // car only
`define S_SUNLOAD                6'd63   // car only

//-----------------------------------------------------------------------------
// 6. SENSOR ATTRIBUTE ROM
//
// Four bits per channel:  { hv_hazard, servicer, class[1:0] }
//
//   class[1:0]   00 SAFETY_CRITICAL     vehicle inhibited, no bypass exists
//                01 POWERTRAIN_DEGRADE  runs power-limited, no bypass
//                10 CONDITIONAL_BYPASS  5-start speed-capped permit offered
//                11 COMFORT_ADAS        feature self-disables, ride continues
//   servicer     0  field replaceable (owner or local mechanic)
//                1  authorised service centre only
//   hv_hazard    0  low voltage side, safe to inspect once HV is isolated
//                1  high voltage side, display shows DO NOT OPEN, no steps ever
//
// Packed MSB-first: nibble [63] is channel 63, nibble [0] is channel 0, so
// SENSOR_ATTR[i*4 +: 4] returns channel i.  Verified in ivcu_defs_check.py.
//-----------------------------------------------------------------------------
`define SENSOR_ATTR_W        4
`define ATTR_CLASS_LSB       0
`define ATTR_SERVICER_BIT    2
`define ATTR_HVHAZARD_BIT    3

`define CLASS_CRITICAL      2'b00
`define CLASS_DEGRADE       2'b01
`define CLASS_CONDITIONAL   2'b10
`define CLASS_COMFORT       2'b11

//        ch: 63 62 61 60 59 58 57 56 | 55 54 53 52 51 50 49 48
//             3  3  3  3  2  2  5  3 |  7  7  3  4  4  4  2  2
//        ch: 47 46 45 44 43 42 41 40 | 39 38 37 36 35 34 33 32
//             4  4  4  4  2  5  5  4 |  4  4  4  4  4  4  4  4
//        ch: 31 30 29 28 27 26 25 24 | 23 22 21 20 19 18 17 16
//             2  2  5  5  5  5  2  D |  D  5  C  C  4  4  C  D
//        ch: 15 14 13 12 11 10  9  8 |  7  6  5  4  3  2  1  0
//             D  D  C  C  C  C  C  C |  C  C  C  C  C  C  C  C
`define SENSOR_ATTR_TABLE   256'h3333_2253_7734_4422_4444_2554_4444_4444_2255_552D_D5CC_44CD_DDCC_CCCC_CCCC_CCCC

//-----------------------------------------------------------------------------
// 7. MODE MASKS
//
// Bit i set means channel i is active in that mode.  These are cross-check
// constants: ivcu_mode_manager builds the live mask from SENSOR_ATTR_TABLE and
// the per-mode presence table, then a generate-time comparison against these
// values catches any edit that makes the two disagree.
//
// V3 hand-wrote its masks and got both wrong: sensor_map_car enabled the
// side-stand sensor on a car, and sensor_map_bike disabled state of charge and
// GPS on the bike, which silently killed the crash-location feature on the
// very vehicle the project was built around.
//-----------------------------------------------------------------------------
//   disabled in CAR : 48 side stand, 52 tip over                    -> 62 active
`define CAR_MASK    64'hFFEE_FFFF_FFFF_FFFF
//   disabled in BIKE: 26-29 coolant loop, 33/35 second and fourth wheel,
//                     42 steering angle, 43 ride height, 51 side impact,
//                     54-56 radar/lidar/ultrasonic, 60-63 cabin comfort
//                                                                   -> 48 active
`define BIKE_MASK   64'h0E37_F3F5_C3FF_FFFF

//-----------------------------------------------------------------------------
// 8. BYPASS ELIGIBILITY  -  THE SECURITY CONSTANT
//
// Exactly eight channels can ever be masked out by request:
//   25 gear position, 30 ambient temp, 31 humidity, 43 ride height,
//   48 side stand, 49 seat occupancy, 58 TPMS front, 59 TPMS rear.
//
// Every bypass request from the outside world is ANDed with this constant
// before it can affect anything.  On the other 56 channels the AND leaves a
// tied-low net that synthesis deletes outright, so there is no gate to attack.
//
// V3 exposed sensor_force_disable as a raw unguarded 42-bit input.  Anything
// on the outside could force-disable brake pressure or battery cell
// temperature.  This constant is the fix.
//-----------------------------------------------------------------------------
`define BYPASS_ELIGIBLE   64'h0C03_0800_C200_0000

//-----------------------------------------------------------------------------
// 9. PER-SENSOR STATUS CODE
//
// Codes OUT_OF_RANGE..IMPLAUSIBLE all suppress the numeric reading on the
// dashboard.  A sensor that is not answering reports NO_RESPONSE, never a
// plausible-looking number and never X.
//
// DISABLED_BY_MODE is deliberately distinct from NO_RESPONSE: a motorcycle
// must never be told its coolant flow sensor has failed, because a motorcycle
// does not have one.
//-----------------------------------------------------------------------------
`define SS_OK               3'd0   // value shown normally
`define SS_DEGRADED         3'd1   // value shown with a caution marker
`define SS_OUT_OF_RANGE     3'd2   // "reading out of range"      value hidden
`define SS_STUCK            3'd3   // "sensor not updating"       value hidden
`define SS_NO_RESPONSE      3'd4   // "SENSOR NOT WORKING"        value hidden
`define SS_IMPLAUSIBLE      3'd5   // "disagrees with others"     value hidden
`define SS_BYPASSED         3'd6   // "operating without this sensor, N left"
`define SS_DISABLED_MODE    3'd7   // "not fitted" - NOT a fault, never scored

//-----------------------------------------------------------------------------
// 10. DOMAIN STATUS AND HEALTH WEIGHTS
//
// Weights are powers of two so every score is computed with shifts and adds.
// RULE R5: there is no divider anywhere in this design.  V3's seq_divider.v
// was the source of the STA divider problem and it is deleted.
//-----------------------------------------------------------------------------
`define DS_OK               4'b0000
`define DS_WARNING          4'b0001
`define DS_CRITICAL         4'b0010
`define DS_FAULT            4'b0100
`define DS_EMERGENCY        4'b1000

`define WEIGHT_CRITICAL     4'd8
`define WEIGHT_DEGRADE      4'd4
`define WEIGHT_CONDITIONAL  4'd2
`define WEIGHT_COMFORT      4'd1

//-----------------------------------------------------------------------------
// 11. HV SAFETY ISLAND
//-----------------------------------------------------------------------------
`define HV_W                 3
`define HV_OFF              3'd0   // both contactors open, DC link bled
`define HV_PRECHARGE        3'd1   // neg contactor + precharge relay closed
`define HV_CLOSING          3'd2   // closing pos contactor
`define HV_ON               3'd3   // normal operation
`define HV_OPENING          3'd4   // ramping torque to zero, then opening
`define HV_DISCHARGE        3'd5   // bleeding the DC link
`define HV_FAULT            3'd6   // latched, cannot reclose without service

`define HVF_NONE            4'd0
`define HVF_PRECHARGE_TO    4'd1   // DC link not rising - downstream short
`define HVF_CONTACTOR_WELD  4'd2   // feedback disagrees with command
`define HVF_DISCHARGE_TO    4'd3   // bleed resistor open - DC LINK MAY BE LIVE
`define HVF_ISOLATION       4'd4   // insulation breakdown
`define HVF_HVIL_OPEN       4'd5   // connector apart with the pack live
`define HVF_CRASH           4'd6   // opened by impact
`define HVF_OVERVOLT        4'd7
`define HVF_OVERCURRENT     4'd8

// Timing, in clk_aon (10 MHz, 100 ns) cycles.  RULE R1: constants, not ports.
`define HVIL_DEBOUNCE_CYC     32'd100_000      //  10 ms, rejects connector chatter
`define PRECHARGE_TIMEOUT_CYC 32'd5_000_000    // 500 ms
`define CONTACTOR_TIMEOUT_CYC 32'd1_000_000    // 100 ms
`define DISCHARGE_TIMEOUT_CYC 32'd20_000_000   //   2 s
`define PYRO_ARM_DELAY_CYC    32'd50_000       //   5 ms
`define PYRO_FIRE_DELAY_CYC   32'd100_000      //  10 ms glitch immunity window
`define TORQUE_RAMP_TIMEOUT   32'd2_000_000    // 200 ms before forcing the open

// Voltage and resistance thresholds, in raw 12-bit ADC counts.
`define HV_TOUCH_SAFE_CNT     16'd600     // ~60 V, discharge complete
`define HV_PRECHARGE_OK_PCT   16'd3891    // 95% of a nominal 4095 full-scale
`define ISO_WARN_CNT          16'd2048    // ~500 ohm/V, warn and keep running
`define ISO_FAULT_CNT         16'd410     // ~100 ohm/V, block HV on
`define I_SAFE_OPEN_CNT       16'd2148    // near-zero current, safe to open

//-----------------------------------------------------------------------------
// 12. CRASH SEVERITY AND SOS ROUTING
//-----------------------------------------------------------------------------
`define CRASH_NONE          3'd0
`define CRASH_MINOR         3'd1   // hazards, owner notified, no emergency call
`define CRASH_MODERATE      3'd2   // + HV disconnect, unlock, POLICE
`define CRASH_SEVERE        3'd3   // + AMBULANCE, occupant count, HV state
`define CRASH_CRITICAL      3'd4   // + FIRE, flagged as a lithium battery event

`define SOS_ROUTE_POLICE    0      // bit position in sos_route[2:0]
`define SOS_ROUTE_AMBULANCE 1
`define SOS_ROUTE_FIRE      2

`define CRASH_THRESH_MINOR    16'd2600   // offset binary about 2048
`define CRASH_THRESH_MODERATE 16'd3000
`define CRASH_THRESH_HARD     16'd3400   // pyro-fuse level impact
`define TIPOVER_SPEED_MIN     8'd30      // km/h, above this a fall is severe

//-----------------------------------------------------------------------------
// 13. SERVICEABILITY PERMIT
//-----------------------------------------------------------------------------
`define NUM_COND_SENSORS     8     // matches BYPASS_ELIGIBLE popcount
`define PERMIT_STARTS        3'd5  // your "5 starts to reach the service centre"
`define PERMIT_W             3

`define PS_NORMAL           2'd0
`define PS_INHIBITED        2'd1   // fault confirmed, permit offered, not taken
`define PS_ACTIVE           2'd2   // permit taken, speed capped, counting down
`define PS_EXPIRED          2'd3   // used up, immobilised until service

// clk_aon cycles
`define IGNITION_DEBOUNCE_CYC 32'd500_000      //  50 ms
`define PERMIT_ACK_HOLD_CYC   32'd30_000_000   //   3 s deliberate hold

`define SPEED_LIMIT_CAR_KPH   8'd40
`define SPEED_LIMIT_BIKE_KPH  8'd30

//-----------------------------------------------------------------------------
// 14. SERVICE GUIDANCE ACTION CODES
//
// The chip emits a code.  The MCU firmware renders the words and pictures.
// V3 put 386 bits of report text and life predictions in silicon, connected
// to nothing - 118,908 um2, 18.8% of the die, that no one could ever read.
// Text does not belong on a die.
//-----------------------------------------------------------------------------
`define GA_NONE             4'd0
`define GA_USER_REPLACE     4'd1   // owner replaceable, show the procedure
`define GA_MECHANIC         4'd2   // local mechanic, show procedure + tools
`define GA_SERVICE_CENTRE   4'd3   // authorised service only, no procedure
`define GA_HV_DANGER        4'd4   // HIGH VOLTAGE, DO NOT OPEN, never any steps
`define GA_PERMIT_OFFER     4'd5   // hold MODE 3 s for 5 limited starts
`define GA_PERMIT_EXPIRED   4'd6   // immobilised, recovery required
`define GA_AUTO_DISABLED    4'd7   // feature switched itself off, no action

//-----------------------------------------------------------------------------
// 15. ACQUISITION TIMING  (clk_sensor, 25 MHz, 40 ns)
//
// A full 64-channel sweep is 64 x 4 cycles / 25 MHz = 10.24 us.
// In bike mode only 48 channels are scanned: 7.68 us.  The mode mask genuinely
// saves scan time and power - it is not just a data mask as it was in V3.
//-----------------------------------------------------------------------------
`define ADC_TIMEOUT_CYC     8'd200    // 8 us with no adc_valid = channel dead
`define ADC_FAIL_COUNT      2'd2      // two consecutive timeouts latch it

//-----------------------------------------------------------------------------
// 16. FAULT LOG EVENT CODES
//-----------------------------------------------------------------------------
`define EV_NONE             4'd0
`define EV_SENSOR_FAULT     4'd1
`define EV_HV_FAULT         4'd2
`define EV_CRASH            4'd3
`define EV_PERMIT_GRANTED   4'd4
`define EV_PERMIT_EXPIRED   4'd5
`define EV_MODE_CHANGE      4'd6
`define EV_PYRO_FIRED       4'd7
`define EV_ISOLATION_WARN   4'd8
`define EV_THERMAL_DERATE   4'd9

`endif // IVCU_DEFS_VH
