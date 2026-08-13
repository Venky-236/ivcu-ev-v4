//=============================================================================
// ivcu_motor_ai.v  -  motor and inverter decisions, channels 18 to 25
//
// THE ONE HARD RULE IN THIS BLOCK
//
// Without rotor position, a permanent-magnet motor cannot be commutated.  The
// inverter does not know which winding to energise, and guessing produces a
// short across the DC link.  So a dead or implausible rotor position channel
// is not a derate, it is a stop.  There is no degraded mode that makes sense.
//
// Everything else here is a ladder: hot motor, hot inverter, overspeed and
// overcurrent each cap the torque, and the lowest cap wins.
//
//-----------------------------------------------------------------------------
// THERMAL DERATING IS DELIBERATELY COARSE
//
// Four steps, not a continuous curve.  A continuous curve needs either a
// divider or a multiplier and buys nothing: the thermal time constant of a
// motor is tens of seconds, and the rider cannot feel the difference between a
// smooth ramp and a step every 10 C.  What the rider can feel is the vehicle
// cutting out entirely, which is what a design without derating does.
//
//-----------------------------------------------------------------------------
// V3 PASSED motor_max_rpm, motor_max_torque AND motor_nominal_current AS PORTS
//
// Three constants that synthesis could not see, so it built general
// comparators against unknown values.  Part of the 36,857 um2 that block cost.
// They are localparams here - RULE R1.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_motor_ai (
    input  wire        clk_ai,
    input  wire        rst_ai_n,

    //--- conditioned values, synchronised into clk_ai -----------------------
    input  wire [15:0] v_motor_rpm,
    input  wire [15:0] v_rotor_pos,
    input  wire [15:0] v_phase_i_a,
    input  wire [15:0] v_phase_i_b,
    input  wire [15:0] v_motor_temp,
    input  wire [15:0] v_inverter_temp,
    input  wire [15:0] v_dc_link_i,

    //--- can these channels be believed --------------------------------------
    input  wire        dead_rotor,
    input  wire        dead_rpm,
    input  wire        dead_phase_i,
    input  wire        dead_motor_temp,
    input  wire        dead_inv_temp,

    input  wire        update_req,

    //--- decisions ------------------------------------------------------------
    output reg         motor_inhibit,      // commutation impossible - stop
    output reg  [7:0]  torque_limit_pct,   // 0..100
    output reg         regen_inhibit,      // do not push current into the pack
    output reg         motor_hot,
    output reg         inverter_hot,
    output reg         overspeed,
    output reg         overcurrent
);

    //-------------------------------------------------------------------------
    // Scaling: temperature count = (T + 40) * 20.475
    //          rpm count         = rpm * 0.273
    //          current           = offset binary, 2048 = 0 A, 4.095 counts/A
    //-------------------------------------------------------------------------
    localparam [15:0] MT_WARM     = 16'd2457;  // 100 C motor
    localparam [15:0] MT_HOT      = 16'd2866;  // 120 C
    localparam [15:0] MT_CRITICAL = 16'd3276;  // 150 C

    localparam [15:0] IT_WARM     = 16'd2252;  //  70 C inverter
    localparam [15:0] IT_HOT      = 16'd2457;  //  80 C
    localparam [15:0] IT_CRITICAL = 16'd2866;  // 100 C

    localparam [15:0] RPM_WARN    = 16'd2730;  // 10,000 rpm
    localparam [15:0] RPM_MAX     = 16'd3276;  // 12,000 rpm

    localparam [15:0] ZERO_I      = 16'd2048;
    localparam [15:0] I_WARN_MAG  = 16'd1024;  // about 250 A
    localparam [15:0] I_MAX_MAG   = 16'd1229;  // about 300 A

    // Four steps.  A fifth is the most likely change once there is a real
    // motor on a dynamometer, but an unused constant sitting here waiting for
    // that day is exactly the kind of thing that accumulates into dead code.
    localparam [7:0]  TQ_FULL     = 8'd100;
    localparam [7:0]  TQ_LIMITED  = 8'd40;
    localparam [7:0]  TQ_CRAWL    = 8'd15;
    localparam [7:0]  TQ_NONE     = 8'd0;

    function [15:0] imag;
        input [15:0] v;
        begin
            imag = (v > ZERO_I) ? (v - ZERO_I) : (ZERO_I - v);
        end
    endfunction

    wire [15:0] mag_a  = imag(v_phase_i_a);
    wire [15:0] mag_b  = imag(v_phase_i_b);
    wire [15:0] mag_dc = imag(v_dc_link_i);

    // the worst of the three current measurements
    wire [15:0] mag_ab   = (mag_a > mag_b)  ? mag_a  : mag_b;
    wire [15:0] mag_worst= (mag_ab > mag_dc)? mag_ab : mag_dc;

    //-------------------------------------------------------------------------
    // Unknown is treated as bad, consistently with ivcu_battery_ai.
    //-------------------------------------------------------------------------
    wire mt_warm_now = dead_motor_temp | (v_motor_temp    > MT_WARM);
    wire mt_hot_now  = dead_motor_temp | (v_motor_temp    > MT_HOT);
    wire mt_crit_now = dead_motor_temp | (v_motor_temp    > MT_CRITICAL);

    wire it_warm_now = dead_inv_temp   | (v_inverter_temp > IT_WARM);
    wire it_hot_now  = dead_inv_temp   | (v_inverter_temp > IT_HOT);
    wire it_crit_now = dead_inv_temp   | (v_inverter_temp > IT_CRITICAL);

    wire ovspd_warn  = dead_rpm        | (v_motor_rpm > RPM_WARN);
    wire ovspd_now   = dead_rpm        | (v_motor_rpm > RPM_MAX);

    wire ovcur_warn  = dead_phase_i    | (mag_worst > I_WARN_MAG);
    wire ovcur_now   = dead_phase_i    | (mag_worst > I_MAX_MAG);

    //-------------------------------------------------------------------------
    // THE HARD RULE.  No rotor position, no commutation, no torque.
    //
    // v_rotor_pos itself is not compared against anything here - a resolver
    // reads every value in its range legitimately as the shaft turns.  What
    // matters is whether the channel is answering at all, which
    // ivcu_sensor_fault_detect has already determined.
    //-------------------------------------------------------------------------
    wire commutation_lost = dead_rotor;

    //-------------------------------------------------------------------------
    // The derate ladder, worst first
    //-------------------------------------------------------------------------
    function [7:0] derate;
        input lost; input mcrit; input icrit; input ocur;
        input mhot; input ihot;  input ospd;
        input mwarm;input iwarm; input ocurw;
        begin
            if      (lost || mcrit || icrit || ocur) derate = TQ_NONE;
            else if (mhot || ihot  || ospd)          derate = TQ_CRAWL;
            else if (mwarm|| iwarm || ocurw)         derate = TQ_LIMITED;
            else                                     derate = TQ_FULL;
        end
    endfunction

    always @(posedge clk_ai or negedge rst_ai_n) begin
        if (!rst_ai_n) begin
            motor_inhibit    <= 1'b1;        // safe default
            torque_limit_pct <= TQ_NONE;
            regen_inhibit    <= 1'b1;
            motor_hot        <= 1'b0;
            inverter_hot     <= 1'b0;
            overspeed        <= 1'b0;
            overcurrent      <= 1'b0;
        end else if (update_req) begin

            motor_hot    <= mt_hot_now;
            inverter_hot <= it_hot_now;
            overspeed    <= ovspd_now;
            overcurrent  <= ovcur_now;

            motor_inhibit <= commutation_lost | mt_crit_now | it_crit_now;

            torque_limit_pct <= derate(commutation_lost,
                                       mt_crit_now, it_crit_now, ovcur_now,
                                       mt_hot_now,  it_hot_now,  ovspd_now,
                                       mt_warm_now, it_warm_now, ovcur_warn);

            // Regeneration pushes current back into the pack.  Above the
            // overspeed warning the back-EMF is already high, and a hot
            // inverter is the last thing that should be asked to rectify more
            // power.  The battery's own charge_inhibit is ANDed in at the
            // actuator stage, not here - this block only speaks for the motor.
            regen_inhibit <= commutation_lost | it_hot_now | ovspd_warn;

        end
    end

endmodule

`default_nettype wire
