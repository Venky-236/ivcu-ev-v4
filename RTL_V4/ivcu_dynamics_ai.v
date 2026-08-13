//=============================================================================
// ivcu_dynamics_ai.v  -  vehicle motion, channels 32 to 43
//
// This block produces the signals half the chip depends on:
//   vehicle_speed_kph   the permit speed cap is enforced against it
//   vehicle_moving      the stuck-detection activity gate
//   vehicle_stationary  mode changes and permit grants both require it
//   braking             the front/rear wheel plausibility check needs it
//
// Getting vehicle_stationary wrong in the unsafe direction would let a mode
// change reconfigure the airbags of a moving car.  So it is not simply
// !moving - see below.
//
//-----------------------------------------------------------------------------
// SPEED WITHOUT A DIVIDER
//
// Wheel speed channels are scaled so full scale (4095 counts) is 255 km/h.
//
//     km/h = counts * 255 / 4095  =  counts / 16.06
//
// which is counts >> 4 to within 0.4 %.  One shift.  RULE R5 satisfied not by
// working around a divide but by choosing the sensor scaling so no divide is
// needed - that decision belongs at the sensor spec, not in the RTL.
//
// The speed used is the HIGHEST of the fitted wheels.  Not the average: a
// locked wheel reads zero and would drag an average down, and under-reporting
// speed is the dangerous direction for every consumer of this signal.
//
//-----------------------------------------------------------------------------
// STATIONARY IS NOT "NOT MOVING"
//
//     vehicle_moving     = speed above a threshold
//     vehicle_stationary = speed below a LOWER threshold, AND has been for
//                          a continuous period
//
// The gap between the two thresholds is hysteresis, so a vehicle creeping at
// the boundary is neither, and nothing that requires stationary can happen.
// The dwell requirement stops a car rolling through a dip - momentarily
// reading zero - from being treated as parked.
//
//-----------------------------------------------------------------------------
// LEAN ANGLE: WHAT THIS IS AND WHAT IT IS NOT
//
// lean_estimate is a leaky integration of roll rate.  It is good enough to
// tell a display that a motorcycle is leaning hard, and to qualify a tip-over.
//
// It is NOT good enough for cornering ABS or traction control.  A real lean
// angle needs a complementary filter fusing the gyro with the gravity vector
// from the accelerometers, because a bare integrator drifts and the leak that
// stops it drifting also destroys its accuracy at sustained lean.  Anything
// safety-critical must not consume this signal, and nothing in V4 does.
//
// Saying so in a comment is better than shipping a number that looks like a
// lean angle and is not.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_dynamics_ai (
    input  wire        clk_ai,
    input  wire        rst_ai_n,

    //--- conditioned values, synchronised into clk_ai -----------------------
    input  wire [15:0] v_wspd_fa,
    input  wire [15:0] v_wspd_fb,
    input  wire [15:0] v_wspd_ra,
    input  wire [15:0] v_wspd_rb,
    // The three accelerometer channels are deliberately NOT ports of this
    // block.  Acceleration drives exactly one decision in this design - the HV
    // disconnect - and that decision is made in ivcu_hv_crash_detect, in the
    // always-on domain, without passing through here.  Carrying them into this
    // module so it looks thorough would be three dead 16-bit inputs, which is
    // the V3 pattern this rebuild exists to remove.
    input  wire [15:0] v_yaw_rate,
    input  wire [15:0] v_roll_rate,
    input  wire [15:0] v_brake_press,
    input  wire [15:0] v_brake_switch,

    //--- which wheels are fitted ---------------------------------------------
    input  wire        mode_is_car,
    input  wire [63:0] sensor_enable_s,

    input  wire        update_req,

    //--- results --------------------------------------------------------------
    output reg  [7:0]  vehicle_speed_kph,
    output reg         vehicle_moving,
    output reg         vehicle_stationary,
    output reg         braking,
    output reg  [7:0]  lean_estimate,     // see the header before using this
    output reg         yaw_excessive,
    output reg         wheel_locked
);

    localparam integer C_WFB = `S_WSPD_FRONT_B;
    localparam integer C_WRB = `S_WSPD_REAR_B;

    localparam [7:0]  SPD_MOVING     = 8'd3;    // km/h - above this, moving
    localparam [7:0]  SPD_STOPPED    = 8'd1;    // km/h - below this, stopped
    localparam [19:0] STATIONARY_CYC = 20'd500_000;  // 10 ms at 50 MHz

    localparam [15:0] ZERO_RATE      = 16'd2048;
    localparam [15:0] YAW_LIMIT      = 16'd683;  // about 100 deg/s magnitude
    localparam [15:0] BRAKE_ON       = 16'd400;
    localparam [15:0] SWITCH_ON      = 16'd3595;
    localparam [7:0]  LOCK_DELTA     = 8'd15;    // km/h between wheels

    function [15:0] rmag;
        input [15:0] v;
        begin
            rmag = (v > ZERO_RATE) ? (v - ZERO_RATE) : (ZERO_RATE - v);
        end
    endfunction

    function [15:0] max2;
        input [15:0] a;
        input [15:0] b;
        begin
            max2 = (a > b) ? a : b;
        end
    endfunction

    //-------------------------------------------------------------------------
    // Only the fitted wheels count.  On a motorcycle channels 33 and 35 are
    // not scanned and read zero; including them would make max2 correct but
    // makes the intent unclear, so they are explicitly masked.
    //-------------------------------------------------------------------------
    wire [15:0] w_fb = sensor_enable_s[C_WFB] ? v_wspd_fb : 16'd0;
    wire [15:0] w_rb = sensor_enable_s[C_WRB] ? v_wspd_rb : 16'd0;

    wire [15:0] max_front = max2(v_wspd_fa, w_fb);
    wire [15:0] max_rear  = max2(v_wspd_ra, w_rb);
    wire [15:0] max_all   = max2(max_front, max_rear);
    wire [15:0] min_axle  = (max_front < max_rear) ? max_front : max_rear;

    // counts >> 4 is km/h to within 0.4 %.  One shift, no divide.
    wire [7:0]  speed_now = max_all[11:4];
    wire [7:0]  speed_min = min_axle[11:4];

    wire [7:0]  axle_delta = (speed_now > speed_min)
                           ? (speed_now - speed_min) : 8'd0;

    wire brake_now = (v_brake_press > BRAKE_ON) |
                     (v_brake_switch > SWITCH_ON);

    //-------------------------------------------------------------------------
    // Stationary dwell counter
    //-------------------------------------------------------------------------
    reg [19:0] still_cnt;

    //-------------------------------------------------------------------------
    // Leaky roll integration.  The leak is a shift, so this stays inside
    // RULE R5.  See the header for what this number is worth.
    //-------------------------------------------------------------------------
    reg signed [19:0] lean_acc;

    wire signed [19:0] roll_signed =
        $signed({4'd0, v_roll_rate}) - $signed({4'd0, ZERO_RATE});

    // integrate, then bleed 1/64 of the accumulator away each update
    wire signed [19:0] lean_next = lean_acc + (roll_signed >>> 6)
                                            - (lean_acc   >>> 6);

    wire [19:0] lean_mag = lean_next[19] ? (~lean_next + 20'd1) : lean_next;

    always @(posedge clk_ai or negedge rst_ai_n) begin
        if (!rst_ai_n) begin
            vehicle_speed_kph  <= 8'd0;
            vehicle_moving     <= 1'b0;
            vehicle_stationary <= 1'b0;   // NOT stationary until proven
            braking            <= 1'b0;
            lean_estimate      <= 8'd0;
            yaw_excessive      <= 1'b0;
            wheel_locked       <= 1'b0;
            still_cnt          <= 20'd0;
            lean_acc           <= 20'sd0;
        end else if (update_req) begin

            vehicle_speed_kph <= speed_now;
            braking           <= brake_now;
            yaw_excessive     <= (rmag(v_yaw_rate) > YAW_LIMIT);

            //-----------------------------------------------------------------
            // Hysteresis band.  Between SPD_STOPPED and SPD_MOVING the vehicle
            // is neither moving nor stationary, and anything that needs one of
            // those two answers waits.
            //-----------------------------------------------------------------
            vehicle_moving <= (speed_now > SPD_MOVING);

            if (speed_now <= SPD_STOPPED) begin
                if (still_cnt >= STATIONARY_CYC) begin
                    vehicle_stationary <= 1'b1;
                end else begin
                    still_cnt <= still_cnt + 20'd1;
                end
            end else begin
                still_cnt          <= 20'd0;
                vehicle_stationary <= 1'b0;
            end

            //-----------------------------------------------------------------
            // A locked wheel: one axle reading far below the other while the
            // vehicle is demonstrably moving.  Only meaningful on a car, where
            // there are two axles with independent sensors on both sides.
            //-----------------------------------------------------------------
            wheel_locked <= mode_is_car & (speed_now > SPD_MOVING) &
                            (axle_delta > LOCK_DELTA);

            //-----------------------------------------------------------------
            // Lean estimate - display and tip-over qualification only
            //-----------------------------------------------------------------
            lean_acc      <= lean_next;
            lean_estimate <= (lean_mag > 20'd255) ? 8'd255 : lean_mag[7:0];

        end
    end

endmodule

`default_nettype wire
