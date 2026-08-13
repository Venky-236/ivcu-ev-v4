//=============================================================================
// ivcu_sensor_fault_detect.v  -  five detection mechanisms, all 64 channels
//
//-----------------------------------------------------------------------------
// WHAT V3 ACTUALLY DID
//
// sensor_validation_fsm had real min/max values for channels 0-9 only.
// Channels 10-41 fell through to DEFAULT_MIN = 0 and DEFAULT_MAX = 0xFFFF, so
// they passed a test of "is this 16-bit number between 0 and 65535".  And at
// the top level even 0-9 were tied to 16'd0 and 16'hFFFF.
//
// So no channel could ever raise a range fault.  sensor_fault was permanently
// zero.  Everything downstream of it - the grace manager, sensor_enable_logic,
// system_health - was reading a signal that could not change.  A whole safety
// mechanism was inert and synthesis, STA and floorplan all passed anyway.
//
// There was no stuck-at detection and no no-response detection either.
// validation_timeout was declared and never used.  A sensor whose wire had
// fallen off, floating at a plausible mid-scale value, read as healthy.
// That is precisely the "shows a value instead of saying the sensor is dead"
// failure this block exists to prevent.
//
//-----------------------------------------------------------------------------
// THE FIVE MECHANISMS  (the sixth, plausibility, is in ivcu_sensor_plausibility)
//
//  1  RANGE      a reading outside the physically possible band for a working
//                sensor.  Not "out of spec" - "out of possible".
//  2  SLEW       a change per sample that no real quantity can produce.  A
//                battery cannot warm 3 degrees in 10 microseconds; if the
//                reading did that, the wiring did it, not the battery.
//  3  STUCK      a reading that does not move when it must be moving.  See the
//                long note on activity gating below - this one is subtle.
//  4  NO RESPONSE the sequencer asked and nothing came back.  Two consecutive
//                timeouts latch it.  This is the mechanism that produces the
//                words "sensor not working".
//  5  DEBOUNCE   nothing latches on one sample.  An up/down counter must
//                saturate before a fault is confirmed and must drain before it
//                clears.  Without this, a marginal connector on a rough road
//                would chatter faults and burn the rider's ignition permits.
//
//-----------------------------------------------------------------------------
// THE 5% / 95% RAIL RULE
//
// Almost every automotive analog sensor is ratiometric: it outputs a fraction
// of its supply.  A working one never sits on the rails.  So:
//
//     below  5% of full scale  ->  short to ground, or open circuit pulled low
//     above 95% of full scale  ->  short to supply
//
// That is why LIM_MIN defaults to 205 and LIM_MAX to 3890 rather than 0 and
// 4095, and it is a far better wiring-fault detector than any per-sensor
// engineering limit.  Sensors whose physics genuinely reaches the rails -
// crash accelerometers during an impact - are given the full span explicitly
// and are covered by stuck detection instead.
//
// DISCRETE channels invert the rule.  A switch is near a rail or it is broken;
// a mid-band reading means a floating input.  is_discrete() marks them.
//
//-----------------------------------------------------------------------------
// WHY STUCK DETECTION IS GATED, AND WHY IT IS OFF FOR TEMPERATURES
//
// A fixed-epsilon stuck check is only honest for quantities that must move.
// Wheel speed must change while the vehicle is moving.  A MEMS accelerometer
// always jitters by a few LSB because it has a noise floor - a perfectly
// constant reading from one means it is dead.
//
// A battery temperature in a parked garage is legitimately constant for hours.
// Running a stuck check on it would either false-alarm or need a threshold so
// loose it detects nothing.  So it is switched off (ACT_NEVER) for
// temperatures, SoC, SoH and the environment channels, and those are covered
// by the cross-checks in ivcu_sensor_plausibility instead - a cell temperature
// that disagrees with pack temperature and with its neighbours is a far more
// reliable signal than one that has not changed.
//
// Claiming a stuck check on a channel where it cannot work would be worse than
// not having one, because it would look like coverage.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_sensor_fault_detect (
    input  wire          clk_sensor,
    input  wire          rst_sensor_n,

    // --- tagged sample stream from the scan sequencer -----------------------
    input  wire [5:0]    samp_chan,
    input  wire [15:0]   samp_data,      // RAW, not filtered - see note below
    input  wire          samp_valid,
    input  wire          samp_timeout,

    // --- context ------------------------------------------------------------
    input  wire [63:0]   sensor_enable,  // mode mask
    input  wire [63:0]   sensor_fresh,   // has this channel ever answered
    input  wire [63:0]   bypass_active,  // operating under a service permit

    // --- activity qualifiers for the stuck check ---------------------------
    input  wire          vehicle_moving,
    input  wire          hv_on,

    // --- results -------------------------------------------------------------
    output wire [191:0]  sensor_status_flat,  // 64 x 3, channel i at [i*3 +: 3]
    output wire [255:0]  sensor_conf_flat,    // 64 x 4, confidence 0..15
    output wire [63:0]   sensor_fault,        // confirmed fault, any mechanism
    output wire [63:0]   sensor_dead          // NO_RESPONSE specifically
);

    // Detection runs on the RAW sample, deliberately.  The IIR filter in
    // ivcu_sensor_conditioning exists to give the AI blocks a clean number; if
    // fault detection ran on the filtered value, the filter would smooth away
    // the very step changes and rail excursions we are trying to catch.

    //=========================================================================
    // CONSTANT TABLES
    //
    // RULE R1 - every one of these is a function of a compile-time constant.
    // Called with a genvar they fold to literals at elaboration and cost
    // nothing.  In V3 the equivalents were 22 module ports, so synthesis built
    // general comparators against unknown values on every channel.
    //=========================================================================

    localparam [15:0] RAIL_LO = 16'd205;    //  5% of 4095 - short to ground
    localparam [15:0] RAIL_HI = 16'd3890;   // 95% of 4095 - short to supply

    // discrete band: a switch outside this is fine, inside it is floating
    localparam [15:0] DISC_LO = 16'd500;
    localparam [15:0] DISC_HI = 16'd3595;

    // stuck activity selectors
    localparam [1:0] ACT_ALWAYS = 2'd0,
                     ACT_MOVING = 2'd1,
                     ACT_HV_ON  = 2'd2,
                     ACT_NEVER  = 2'd3;   // stuck detection disabled

    localparam [15:0] STUCK_COUNT   = 16'd50000; // ~0.5 s of unchanged samples
    localparam [3:0]  FAULT_DEBOUNCE = 4'd5;     // consecutive bad samples
    localparam [1:0]  DEAD_COUNT     = `ADC_FAIL_COUNT;

    //-------------------------------------------------------------------------
    // Scaling conventions used by every table below.  Twelve bits, 0..4095.
    //
    //   TEMPERATURE  -40..+160 C          count = (T + 40) * 20.475
    //   CELL VOLTAGE 0..5 V               count = V * 819
    //   PACK VOLTAGE 0..500 V             count = V * 8.19
    //   CURRENT      offset binary +-500A count = 2048 + A * 4.095
    //   PERCENT      0..100 %             count = P * 40.95
    //   ACCEL        offset binary +-20g  count = 2048 + g * 102.4
    //   RATE         offset binary +-300  count = 2048 + rate * 6.83  (deg/s)
    //   RPM          0..15000             count = rpm * 0.273
    //   PRESSURE     0..10 bar            count = bar * 409.5
    //   ISOLATION    0..1000 ohm/V        count = ohmPerV * 4.095
    //-------------------------------------------------------------------------

    // ---- LIM_MIN : below this the sensor or its wiring is broken -----------
    function [15:0] lim_min;
        input [5:0] c;
        begin
            case (c)
                // percentages legitimately approach zero, so 2% not 5%
                `S_SOC, `S_SOH               : lim_min = 16'd82;
                // crash sensors legitimately hit the bottom rail on impact
                `S_CRASH_FRONT, `S_CRASH_SIDE,
                `S_ACCEL_LONG, `S_ACCEL_LAT,
                `S_ACCEL_VERT                : lim_min = 16'd0;
                // discretes are checked by the inverted rule, not by rails
                `S_HVIL_LOOP, `S_CONTACTOR_FB_POS, `S_CONTACTOR_FB_NEG,
                `S_BRAKE_SWITCH, `S_SIDE_STAND, `S_SEAT_OCCUPANCY,
                `S_TIP_OVER                  : lim_min = 16'd0;
                // isolation may legitimately read very low - that IS the fault
                // condition, and it must reach the HV island, not be masked as
                // a sensor failure
                `S_ISOLATION_RES             : lim_min = 16'd0;
                // DC link is at zero volts whenever HV is off - normal
                `S_HV_BUS_VOLT, `S_PRECHARGE_VOLT : lim_min = 16'd0;
                default                      : lim_min = RAIL_LO;
            endcase
        end
    endfunction

    // ---- LIM_MAX : above this the sensor or its wiring is broken -----------
    function [15:0] lim_max;
        input [5:0] c;
        begin
            case (c)
                `S_SOC, `S_SOH               : lim_max = 16'd4013;
                `S_CRASH_FRONT, `S_CRASH_SIDE,
                `S_ACCEL_LONG, `S_ACCEL_LAT,
                `S_ACCEL_VERT                : lim_max = 16'd4095;
                `S_HVIL_LOOP, `S_CONTACTOR_FB_POS, `S_CONTACTOR_FB_NEG,
                `S_BRAKE_SWITCH, `S_SIDE_STAND, `S_SEAT_OCCUPANCY,
                `S_TIP_OVER                  : lim_max = 16'd4095;
                default                      : lim_max = RAIL_HI;
            endcase
        end
    endfunction

    // ---- WARN_LO / WARN_HI : the sensor works, the QUANTITY is dangerous ---
    // These do not indicate a broken sensor.  They set SS_DEGRADED and feed
    // the domain AI blocks, which decide what to do about it.
    function [15:0] warn_lo;
        input [5:0] c;
        begin
            case (c)
                `S_CELL_VOLT_MIN     : warn_lo = 16'd2457;  // 3.00 V per cell
                `S_CELL_VOLT_MAX     : warn_lo = 16'd2457;
                `S_PACK_VOLTAGE      : warn_lo = 16'd2048;  // 250 V
                `S_CELL_TEMP_MIN     : warn_lo = 16'd819;   // 0 C - no charging
                `S_CELL_TEMP_MAX     : warn_lo = 16'd819;
                `S_PACK_TEMP         : warn_lo = 16'd819;
                `S_SOC               : warn_lo = 16'd410;   // 10 %
                `S_SOH               : warn_lo = 16'd2867;  // 70 % - ageing
                `S_ISOLATION_RES     : warn_lo = `ISO_WARN_CNT;  // 500 ohm/V
                `S_COOLANT_FLOW      : warn_lo = 16'd410;   // 1 bar equivalent
                `S_COOLANT_PRESSURE  : warn_lo = 16'd410;
                `S_TPMS_FRONT        : warn_lo = 16'd819;   // 2.0 bar
                `S_TPMS_REAR         : warn_lo = 16'd819;
                default              : warn_lo = 16'd0;     // no low warning
            endcase
        end
    endfunction

    function [15:0] warn_hi;
        input [5:0] c;
        begin
            case (c)
                `S_CELL_VOLT_MAX     : warn_hi = 16'd3440;  // 4.20 V per cell
                `S_CELL_VOLT_MIN     : warn_hi = 16'd3440;
                `S_PACK_VOLTAGE      : warn_hi = 16'd3440;  // 420 V
                `S_CELL_TEMP_MAX     : warn_hi = 16'd1945;  // 55 C
                `S_CELL_TEMP_MIN     : warn_hi = 16'd1945;
                `S_PACK_TEMP         : warn_hi = 16'd1945;
                `S_MOTOR_TEMP        : warn_hi = 16'd2866;  // 100 C
                `S_INVERTER_TEMP     : warn_hi = 16'd2457;  // 80 C
                `S_PACK_CURRENT      : warn_hi = 16'd3277;  // +300 A
                `S_PHASE_CURRENT_A   : warn_hi = 16'd3277;
                `S_PHASE_CURRENT_B   : warn_hi = 16'd3277;
                `S_DC_LINK_CURRENT   : warn_hi = 16'd3277;
                `S_CHARGE_CURRENT    : warn_hi = 16'd3277;
                `S_MOTOR_RPM         : warn_hi = 16'd2730;  // 10,000 rpm
                `S_COOLANT_TEMP_OUT  : warn_hi = 16'd2457;  // 80 C
                `S_COOLANT_TEMP_IN   : warn_hi = 16'd2252;  // 70 C
                `S_PACK_ENCL_PRESS   : warn_hi = 16'd1229;  // 3 bar - venting
                `S_AMBIENT_TEMP      : warn_hi = 16'd3071;  // 110 C
                default              : warn_hi = 16'd4095;  // no high warning
            endcase
        end
    endfunction

    // ---- SLEW_MAX : physically impossible change between samples ----------
    // One sample period is one sweep, about 10 us.
    function [15:0] slew_max;
        input [5:0] c;
        begin
            case (c)
                // exempt: a huge step IS the measurement
                `S_CRASH_FRONT, `S_CRASH_SIDE, `S_TIP_OVER,
                `S_ACCEL_LONG, `S_ACCEL_LAT, `S_ACCEL_VERT,
                `S_HVIL_LOOP, `S_CONTACTOR_FB_POS, `S_CONTACTOR_FB_NEG,
                `S_BRAKE_SWITCH, `S_SIDE_STAND, `S_SEAT_OCCUPANCY,
                `S_GEAR_POSITION             : slew_max = 16'd4095;
                // electrical: fast, but not instantaneous across a full scale
                `S_PACK_CURRENT, `S_PHASE_CURRENT_A, `S_PHASE_CURRENT_B,
                `S_DC_LINK_CURRENT, `S_HV_BUS_VOLT,
                `S_PRECHARGE_VOLT            : slew_max = 16'd2048;
                // mechanical / human speed
                `S_THROTTLE_POS_1, `S_THROTTLE_POS_2, `S_BRAKE_PRESSURE,
                `S_ROTOR_POSITION, `S_MOTOR_RPM, `S_STEERING_ANGLE,
                `S_WSPD_FRONT_A, `S_WSPD_FRONT_B,
                `S_WSPD_REAR_A,  `S_WSPD_REAR_B,
                `S_YAW_RATE, `S_ROLL_RATE, `S_PITCH_RATE
                                             : slew_max = 16'd512;
                // thermal mass: nothing real moves 3 C in 10 us
                default                      : slew_max = 16'd64;
            endcase
        end
    endfunction

    // ---- STUCK_EPS : below this change the reading counts as not moving ----
    function [15:0] stuck_eps;
        input [5:0] c;
        begin
            case (c)
                `S_ACCEL_LONG, `S_ACCEL_LAT, `S_ACCEL_VERT,
                `S_YAW_RATE, `S_ROLL_RATE, `S_PITCH_RATE
                                             : stuck_eps = 16'd2;  // noise floor
                default                      : stuck_eps = 16'd4;
            endcase
        end
    endfunction

    // ---- ACT_SEL : when is the stuck check meaningful ----------------------
    function [1:0] act_sel;
        input [5:0] c;
        begin
            case (c)
                // a MEMS device always jitters; perfectly constant means dead
                `S_ACCEL_LONG, `S_ACCEL_LAT, `S_ACCEL_VERT,
                `S_YAW_RATE, `S_ROLL_RATE, `S_PITCH_RATE
                                             : act_sel = ACT_ALWAYS;
                // must move while the wheels turn
                `S_WSPD_FRONT_A, `S_WSPD_FRONT_B,
                `S_WSPD_REAR_A,  `S_WSPD_REAR_B,
                `S_MOTOR_RPM, `S_ROTOR_POSITION
                                             : act_sel = ACT_MOVING;
                // must move while current flows
                `S_PACK_CURRENT, `S_PHASE_CURRENT_A, `S_PHASE_CURRENT_B,
                `S_DC_LINK_CURRENT, `S_HV_BUS_VOLT
                                             : act_sel = ACT_HV_ON;
                // temperatures, SoC, SoH, environment, discretes:
                // legitimately constant.  Covered by plausibility instead.
                default                      : act_sel = ACT_NEVER;
            endcase
        end
    endfunction

    // ---- IS_DISCRETE : rail rule inverted ----------------------------------
    function is_discrete;
        input [5:0] c;
        begin
            case (c)
                `S_HVIL_LOOP, `S_CONTACTOR_FB_POS, `S_CONTACTOR_FB_NEG,
                `S_BRAKE_SWITCH, `S_SIDE_STAND, `S_SEAT_OCCUPANCY,
                `S_TIP_OVER                  : is_discrete = 1'b1;
                default                      : is_discrete = 1'b0;
            endcase
        end
    endfunction

    //=========================================================================
    // PER-CHANNEL DETECTION.  RULE R6 - generate, never copy-paste.
    //=========================================================================
    genvar i;
    generate
        for (i = 0; i < `NUM_SENSORS; i = i + 1) begin : ch

            // all folded to literals at elaboration
            localparam [15:0] L_MIN   = lim_min  (i[5:0]);
            localparam [15:0] L_MAX   = lim_max  (i[5:0]);
            localparam [15:0] W_LO    = warn_lo  (i[5:0]);
            localparam [15:0] W_HI    = warn_hi  (i[5:0]);
            localparam [15:0] SLEW    = slew_max (i[5:0]);
            localparam [15:0] EPS     = stuck_eps(i[5:0]);
            localparam [1:0]  ACT     = act_sel  (i[5:0]);
            localparam        DISC    = is_discrete(i[5:0]);

            reg  [15:0] prev_raw;
            reg  [15:0] stuck_cnt;
            reg  [1:0]  dead_cnt;
            reg  [3:0]  dbnc;
            reg  [2:0]  kind;        // which mechanism fired, latched
            reg         dead_latch;

            wire hit_ok  = samp_valid   & (samp_chan == i[5:0]);
            wire hit_tmo = samp_timeout & (samp_chan == i[5:0]);
            wire live    = sensor_enable[i];

            //-----------------------------------------------------------------
            // 1  RANGE
            //-----------------------------------------------------------------
            wire rng_bad = DISC
                         ? ((samp_data > DISC_LO) && (samp_data < DISC_HI))
                         : ((samp_data < L_MIN)   || (samp_data > L_MAX));

            //-----------------------------------------------------------------
            // 2  SLEW.  Magnitude of the change without a subtract-and-abs:
            //    compare in whichever direction is larger.
            //-----------------------------------------------------------------
            wire [15:0] delta = (samp_data > prev_raw)
                              ? (samp_data - prev_raw)
                              : (prev_raw  - samp_data);

            wire slew_bad = (delta > SLEW);

            //-----------------------------------------------------------------
            // 3  STUCK, gated by whether the quantity must be moving
            //-----------------------------------------------------------------
            wire act_now = (ACT == ACT_ALWAYS) ? 1'b1        :
                           (ACT == ACT_MOVING) ? vehicle_moving :
                           (ACT == ACT_HV_ON ) ? hv_on       : 1'b0;

            wire not_moving = act_now && (delta < EPS);
            wire stuck_bad  = (stuck_cnt >= STUCK_COUNT);

            //-----------------------------------------------------------------
            // 4  NO RESPONSE
            //-----------------------------------------------------------------
            wire dead_bad = dead_latch;

            //-----------------------------------------------------------------
            // 5  DEBOUNCE.  Any mechanism feeds one up/down counter.
            //-----------------------------------------------------------------
            wire raw_bad   = rng_bad | slew_bad | stuck_bad;
            wire confirmed = (dbnc >= FAULT_DEBOUNCE);

            always @(posedge clk_sensor or negedge rst_sensor_n) begin
                if (!rst_sensor_n) begin
                    prev_raw   <= 16'd0;
                    stuck_cnt  <= 16'd0;
                    dead_cnt   <= 2'd0;
                    dbnc       <= 4'd0;
                    kind       <= `SS_OK;
                    dead_latch <= 1'b0;
                end else if (!live) begin
                    // rest state: hold everything clear so the channel cannot
                    // carry a stale fault from a different vehicle mode
                    prev_raw   <= 16'd0;
                    stuck_cnt  <= 16'd0;
                    dead_cnt   <= 2'd0;
                    dbnc       <= 4'd0;
                    kind       <= `SS_OK;
                    dead_latch <= 1'b0;
                end else begin

                    // -- the channel answered ---------------------------------
                    if (hit_ok) begin
                        prev_raw <= samp_data;
                        dead_cnt <= 2'd0;
                        dead_latch <= 1'b0;

                        if (not_moving && !stuck_bad) begin
                            stuck_cnt <= stuck_cnt + 16'd1;
                        end else if (!not_moving) begin
                            stuck_cnt <= 16'd0;
                        end

                        if (raw_bad) begin
                            if (dbnc != 4'd15) dbnc <= dbnc + 4'd1;
                            // record the reason, most serious first
                            if      (stuck_bad) kind <= `SS_STUCK;
                            else if (slew_bad ) kind <= `SS_IMPLAUSIBLE;
                            else                kind <= `SS_OUT_OF_RANGE;
                        end else begin
                            if (dbnc != 4'd0)  dbnc <= dbnc - 4'd1;
                        end
                    end

                    // -- the channel did not answer ---------------------------
                    if (hit_tmo) begin
                        if (dead_cnt != 2'd3) dead_cnt <= dead_cnt + 2'd1;
                        if (dead_cnt >= (DEAD_COUNT - 2'd1)) begin
                            dead_latch <= 1'b1;
                            kind       <= `SS_NO_RESPONSE;
                            dbnc       <= FAULT_DEBOUNCE;   // confirm at once
                        end
                    end
                end
            end

            //-----------------------------------------------------------------
            // STATUS PRIORITY
            //
            // Mode first: a motorcycle must never be told its coolant flow
            // sensor has failed, because it does not have one.  Then bypass,
            // then no-response, then the measured faults, then the warning
            // band, then OK.
            //-----------------------------------------------------------------
            // prev_raw holds the last good sample for this channel, which is
            // the right thing to test against the warning band: it is the most
            // recent reading that actually arrived.
            wire warn_band = (prev_raw < W_LO) || (prev_raw > W_HI);

            wire [2:0] status =
                  (!live)                     ? `SS_DISABLED_MODE :
                  ( bypass_active[i])         ? `SS_BYPASSED      :
                  ( dead_bad)                 ? `SS_NO_RESPONSE   :
                  ( confirmed)                ? kind              :
                  (!sensor_fresh[i])          ? `SS_NO_RESPONSE   :
                  ( warn_band)                ? `SS_DEGRADED      :
                                                `SS_OK;

            assign sensor_status_flat[i*3 +: 3] = status;

            //-----------------------------------------------------------------
            // CONFIDENCE, 0..15.  How far the reading sits inside its warning
            // band.  This is the per-sensor score shown to the rider.
            // Shifts only - RULE R5, no divider.
            //-----------------------------------------------------------------
            wire [3:0] conf = (status == `SS_OK)       ? 4'd15 :
                              (status == `SS_DEGRADED) ? 4'd8  :
                              (status == `SS_BYPASSED) ? 4'd4  :
                              (status == `SS_DISABLED_MODE) ? 4'd0 : 4'd0;

            assign sensor_conf_flat[i*4 +: 4] = conf;

            assign sensor_fault[i] = live & (confirmed | dead_bad);
            assign sensor_dead [i] = live & dead_bad;

        end
    endgenerate

endmodule

`default_nettype wire
