//=============================================================================
// ivcu_driver_input_ai.v  -  throttle, brake and rider presence, channels 44-49
//
// THIS BLOCK DECIDES HOW MUCH TORQUE THE RIDER IS ASKING FOR.  It is the only
// place in the design that turns a human intention into a number that moves
// the vehicle, so every failure mode here has to resolve to zero torque, not
// to a guess.
//
//-----------------------------------------------------------------------------
// WHY THERE ARE TWO THROTTLE SENSORS
//
// Every production drive-by-wire system uses two throttle tracks with opposite
// slopes, so their readings sum to a constant.  One track alone cannot be
// trusted: a potentiometer wiper that lifts reads a plausible mid-scale value,
// and a vehicle that accelerates because a wiper lifted is a vehicle that
// accelerates for no reason.
//
//     track 1 rises 0 -> 4095 as the throttle opens
//     track 2 falls 4095 -> 0
//     sum stays at 4095 +- tolerance
//
// ivcu_sensor_plausibility does the comparison and raises throttle_disagree.
// When it fires, the pedal position is UNKNOWN - not high, not low, unknown -
// and the only safe interpretation of an unknown throttle is closed.
//
//-----------------------------------------------------------------------------
// BRAKE OVERRIDES THROTTLE, ALWAYS
//
// If the brake is applied and the throttle is open at the same time, something
// is wrong: a stuck pedal, a trapped mat, a failed sensor, or a panicking
// rider using both. In every one of those cases the correct action is the
// same. Brake wins. This is a legal requirement in most markets and it is
// also just right.
//
// Note the asymmetry: brake_switch is a discrete and brake_pressure is analog,
// and EITHER asserting is enough. Requiring both would mean one failed brake
// sensor disables the override.
//
//-----------------------------------------------------------------------------
// TORQUE IS NOT SCALED HERE
//
// torque_request is the rider's demand, 0 to 4095, and nothing else.  Every
// limit in the vehicle - battery derate, motor thermal derate, permit speed
// cap, HV state - is applied downstream in ivcu_actuator_output_mgr, in one
// place, where all the limits can be seen together.  Applying some of them
// here would scatter the authority over the torque path across two files, and
// then nobody could answer "what can make this vehicle move" by reading one.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_driver_input_ai (
    input  wire        clk_ai,
    input  wire        rst_ai_n,

    //--- conditioned values, synchronised into clk_ai -----------------------
    input  wire [15:0] v_throttle_1,
    // Track 2 is deliberately NOT a port here.  Its entire job is to disagree
    // with track 1, and that comparison happens in ivcu_sensor_plausibility
    // where both tracks are available at the same instant from the same sweep.
    // Its verdict arrives as throttle_disagree below.  Carrying the raw value
    // in as well would be a 16-bit input this module never reads.
    input  wire [15:0] v_brake_press,
    input  wire [15:0] v_brake_switch,
    input  wire [15:0] v_side_stand,
    input  wire [15:0] v_seat_occ,
    input  wire [15:0] v_gear_pos,

    //--- trust ----------------------------------------------------------------
    input  wire        throttle_disagree,   // plausibility check 1
    input  wire        dead_throttle,       // either track not answering
    input  wire        dead_brake,

    //--- context ---------------------------------------------------------------
    input  wire        mode_is_bike,
    input  wire [63:0] sensor_enable_s,
    input  wire [63:0] bypass_active_s,     // side stand may be under a permit
    input  wire        update_req,

    //--- results ---------------------------------------------------------------
    output reg  [11:0] torque_request,      // rider demand, unlimited
    output reg  [11:0] regen_request,
    output reg         throttle_fault,
    output reg         brake_override,      // brake beat an open throttle
    output reg         rider_present,
    output reg         stand_down,          // side stand deployed - inhibit
    output reg         gear_engaged
);

    localparam integer C_STAND = `S_SIDE_STAND;

    localparam [15:0] THR_DEADBAND = 16'd120;   // about 3 % - pedal slop
    localparam [15:0] BRAKE_ON     = 16'd400;
    localparam [15:0] SWITCH_ON    = 16'd3595;  // discrete, near the top rail
    localparam [15:0] STAND_DOWN   = 16'd3595;  // discrete: stand is deployed
    localparam [15:0] SEAT_OCCUPIED= 16'd3595;
    localparam [15:0] GEAR_NEUTRAL = 16'd500;   // below this is neutral

    //-------------------------------------------------------------------------
    // Is the brake applied?  Either source is enough.
    //-------------------------------------------------------------------------
    wire brake_applied = (v_brake_press  > BRAKE_ON) |
                         (v_brake_switch > SWITCH_ON);

    //-------------------------------------------------------------------------
    // Is the throttle trustworthy?
    //-------------------------------------------------------------------------
    wire throttle_bad = throttle_disagree | dead_throttle;

    //-------------------------------------------------------------------------
    // Raw demand from the primary track, with a deadband so a pedal resting
    // slightly off zero does not creep the vehicle.
    //-------------------------------------------------------------------------
    wire [15:0] thr_raw = (v_throttle_1 > THR_DEADBAND)
                        ? (v_throttle_1 - THR_DEADBAND)
                        : 16'd0;

    //-------------------------------------------------------------------------
    // Regen demand from brake pressure.  A brake switch alone gives no
    // magnitude, so switch-only braking asks for a fixed modest regen and
    // leaves the rest to the friction brakes.
    //-------------------------------------------------------------------------
    localparam [11:0] REGEN_SWITCH_ONLY = 12'd800;

    wire [11:0] regen_from_press = v_brake_press[11:0];

    //-------------------------------------------------------------------------
    // Side stand.  On a motorcycle this inhibits the vehicle - unless the
    // rider holds a limited-operation permit for that channel, which is the
    // worked example the whole serviceability feature was built around.
    //
    // If the channel is not fitted (car mode) or is under permit, it cannot
    // hold the vehicle back.
    //-------------------------------------------------------------------------
    wire stand_channel_live = sensor_enable_s[C_STAND] & ~bypass_active_s[C_STAND];
    wire stand_deployed     = mode_is_bike & stand_channel_live &
                              (v_side_stand > STAND_DOWN);

    always @(posedge clk_ai or negedge rst_ai_n) begin
        if (!rst_ai_n) begin
            torque_request <= 12'd0;
            regen_request  <= 12'd0;
            throttle_fault <= 1'b0;
            brake_override <= 1'b0;
            rider_present  <= 1'b0;
            stand_down     <= 1'b0;
            gear_engaged   <= 1'b0;
        end else if (update_req) begin

            throttle_fault <= throttle_bad;
            rider_present  <= (v_seat_occ > SEAT_OCCUPIED);
            stand_down     <= stand_deployed;
            gear_engaged   <= (v_gear_pos > GEAR_NEUTRAL);

            //-----------------------------------------------------------------
            // THE TORQUE DECISION.  Three ways to get zero, one way to get a
            // number.  Ordering matters: the faults are checked before the
            // demand is even looked at.
            //-----------------------------------------------------------------
            if (throttle_bad) begin
                // the pedal position is unknown.  Unknown means closed.
                torque_request <= 12'd0;
                brake_override <= 1'b0;
            end else if (brake_applied) begin
                torque_request <= 12'd0;
                // only call it an override if the throttle was actually open -
                // ordinary braking with a closed throttle is not an event
                brake_override <= (thr_raw > 16'd0);
            end else begin
                torque_request <= thr_raw[11:0];
                brake_override <= 1'b0;
            end

            //-----------------------------------------------------------------
            // Regen.  A dead brake sensor asks for none: guessing regen
            // amounts from a sensor that is not answering would decelerate the
            // vehicle by an amount nobody commanded.
            //-----------------------------------------------------------------
            if (dead_brake) begin
                regen_request <= 12'd0;
            end else if (v_brake_press > BRAKE_ON) begin
                regen_request <= regen_from_press;
            end else if (v_brake_switch > SWITCH_ON) begin
                regen_request <= REGEN_SWITCH_ONLY;
            end else begin
                regen_request <= 12'd0;
            end

        end
    end

endmodule

`default_nettype wire
