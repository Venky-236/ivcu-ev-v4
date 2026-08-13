//=============================================================================
// ivcu_perception_ai.v  -  ADAS feature availability, channels 53 to 57
//
// THE ONLY BLOCK IN THIS DESIGN THAT IS ALLOWED TO FAIL QUIETLY.
//
// Every other domain, on a fault, either stops the vehicle or limits it.  This
// one switches a convenience feature off, tells the driver once, and gets out
// of the way.  A failed parking sensor must not strand anybody.
//
// The rule that matters: a feature is available only when EVERY sensor it
// depends on is healthy.  Adaptive cruise on radar alone, after the camera has
// failed, is worse than no adaptive cruise - the driver keeps trusting a
// system that has lost half its evidence.  So the dependency lists below are
// ANDs, not ORs.
//
//-----------------------------------------------------------------------------
// GPS IS DIFFERENT AND IT IS NOT A COMFORT CHANNEL
//
// Channel 57 is classified POWERTRAIN_DEGRADE, not COMFORT_ADAS, because the
// crash SOS message depends on it.  Losing GPS does not inhibit the vehicle,
// but it does latch a warning, and the SOS message falls back to the last
// valid fix with an explicit staleness count - so emergency services are told
// the position is N seconds old rather than being allowed to believe it is
// current.
//
// V3 disabled GPS entirely in bike mode.  The crash-location-to-emergency-
// services feature, on the vehicle the whole project was described around, was
// dead.  Channel 57 is enabled in both modes here.
//
//-----------------------------------------------------------------------------
// STALENESS IS COUNTED IN SWEEPS, NOT SECONDS
//
// One sweep is about 10 us in car mode.  Counting seconds would need a divide
// by the sweep rate.  The counter here is in sweeps and the MCU converts it -
// the chip supplies a number and a unit, firmware supplies the arithmetic.
// Same principle as the guidance codes: no engineering units on the die where
// a raw count will do.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_perception_ai (
    input  wire        clk_ai,
    input  wire        rst_ai_n,

    //--- per-channel verdicts, synchronised into clk_ai ---------------------
    input  wire [2:0]  st_camera,
    input  wire [2:0]  st_radar,
    input  wire [2:0]  st_lidar,
    input  wire [2:0]  st_ultrasonic,
    input  wire [2:0]  st_gps,
    input  wire [2:0]  st_ambient_light,
    input  wire [2:0]  st_rain,

    input  wire        mode_is_car,
    input  wire        update_req,

    //--- feature availability -------------------------------------------------
    output reg         feat_lane_keep,
    output reg         feat_adaptive_cruise,
    output reg         feat_blind_spot,
    output reg         feat_park_assist,
    output reg         feat_auto_headlight,
    output reg         feat_auto_wiper,
    output reg         feat_collision_warn,

    //--- GPS, for the emergency message ---------------------------------------
    output reg         gps_usable,
    output reg  [15:0] gps_stale_sweeps,   // 0 = fix is current
    output reg         perception_degraded // something switched itself off
);

    //-------------------------------------------------------------------------
    // "healthy" means OK or DEGRADED.  A degraded sensor still produces usable
    // data with a caution flag; anything past that - out of range, stuck, not
    // answering, implausible - is not evidence and must not support a feature.
    //
    // SS_DISABLED_MODE is NOT healthy here, deliberately.  A motorcycle has no
    // radar, so adaptive cruise is unavailable rather than "available with a
    // missing sensor".
    //-------------------------------------------------------------------------
    function healthy;
        input [2:0] s;
        begin
            healthy = (s == `SS_OK) || (s == `SS_DEGRADED);
        end
    endfunction

    wire h_cam   = healthy(st_camera);
    wire h_rad   = healthy(st_radar);
    wire h_lid   = healthy(st_lidar);
    wire h_ultra = healthy(st_ultrasonic);
    wire h_gps   = healthy(st_gps);
    wire h_light = healthy(st_ambient_light);
    wire h_rain  = healthy(st_rain);

    //-------------------------------------------------------------------------
    // Dependency lists.  ANDs, not ORs - see the header.
    //-------------------------------------------------------------------------
    wire avail_lane   = mode_is_car & h_cam;
    wire avail_acc    = mode_is_car & h_rad & h_cam;   // both, or neither
    wire avail_blind  = mode_is_car & h_rad;
    wire avail_park   = mode_is_car & h_ultra;
    wire avail_light  = mode_is_car & h_light;
    wire avail_wiper  = mode_is_car & h_rain;
    wire avail_fcw    = h_cam & (h_rad | h_lid);       // camera plus one range
                                                       // sensor, either will do

    //-------------------------------------------------------------------------
    // Anything that should be available in this mode but is not
    //-------------------------------------------------------------------------
    wire degraded_now = mode_is_car
                      ? ~(h_cam & h_rad & h_lid & h_ultra & h_gps &
                          h_light & h_rain)
                      : ~(h_cam & h_gps);

    always @(posedge clk_ai or negedge rst_ai_n) begin
        if (!rst_ai_n) begin
            feat_lane_keep       <= 1'b0;
            feat_adaptive_cruise <= 1'b0;
            feat_blind_spot      <= 1'b0;
            feat_park_assist     <= 1'b0;
            feat_auto_headlight  <= 1'b0;
            feat_auto_wiper      <= 1'b0;
            feat_collision_warn  <= 1'b0;
            gps_usable           <= 1'b0;
            gps_stale_sweeps     <= 16'd0;
            perception_degraded  <= 1'b0;
        end else if (update_req) begin

            feat_lane_keep       <= avail_lane;
            feat_adaptive_cruise <= avail_acc;
            feat_blind_spot      <= avail_blind;
            feat_park_assist     <= avail_park;
            feat_auto_headlight  <= avail_light;
            feat_auto_wiper      <= avail_wiper;
            feat_collision_warn  <= avail_fcw;

            perception_degraded  <= degraded_now;

            //-----------------------------------------------------------------
            // GPS staleness.  The counter saturates rather than wrapping: a
            // position that is "65535 sweeps old" and one that is "very old"
            // are the same statement to a dispatcher, but a wrapped counter
            // would say "current" and that would be a lie at the worst
            // possible moment.
            //-----------------------------------------------------------------
            gps_usable <= h_gps;

            if (h_gps) begin
                gps_stale_sweeps <= 16'd0;
            end else if (gps_stale_sweeps != 16'hFFFF) begin
                gps_stale_sweeps <= gps_stale_sweeps + 16'd1;
            end

        end
    end

endmodule

`default_nettype wire
