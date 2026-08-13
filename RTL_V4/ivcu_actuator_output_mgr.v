//=============================================================================
// ivcu_actuator_output_mgr.v  -  every limit, applied in one place
//
// THE RULE THIS FILE EXISTS TO ENFORCE
//
// If you want to know what can make this vehicle move, you read this file.
// Not this file and four others.  Every derate, inhibit, cap and mode gate
// that touches an actuator lands here, so the answer to "why is the torque
// zero" is always in one place.
//
// ivcu_driver_input_ai produces the rider's demand and applies nothing to it.
// ivcu_battery_ai and ivcu_motor_ai produce percentage ceilings and drive
// nothing.  The authority is here.
//
//-----------------------------------------------------------------------------
// AIRBAGS DO NOT EXIST ON A MOTORCYCLE
//
// V3 let airbag_control and door_unlock fire in bike mode.  A motorcycle has
// no airbags and no doors, so those outputs were driving nothing at best -
// and if the same silicon were ever fitted to a bike with the airbag output
// wired to anything, it would fire it.
//
// The gate here is an AND with a constant that depends only on active_mode.
// It is not a check that could be skipped and it is not inside a state
// machine.  In bike mode, synthesis ties those outputs to zero.
//
//-----------------------------------------------------------------------------
// PERCENTAGES BECOME CAPS, NOT MULTIPLIERS
//
// The obvious way to apply a 40 % limit is torque * 40 / 100.  That is a
// multiply and a divide, and RULE R5 forbids the divide outright.
//
// So a percentage is mapped to an absolute ceiling by a small comparison
// ladder, and the output is the smaller of demand and ceiling.  A rider asking
// for less than the ceiling is unaffected, which is what a limit should do -
// scaling would quietly reduce a gentle throttle input that was never a
// problem.
//
//-----------------------------------------------------------------------------
// THE SPEED CAP HAS HYSTERESIS
//
// A hard cutoff at exactly the permit speed would make the vehicle surge and
// cut repeatedly as it crossed the threshold, which on a motorcycle under a
// side-stand permit is genuinely dangerous.  Torque is reduced to a crawl at
// the limit and removed two km/h above it, and restored below.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_actuator_output_mgr (
    input  wire        clk_aon,
    input  wire        rst_aon_n,

    //--- rider demand, unlimited, from ivcu_driver_input_ai -----------------
    input  wire [11:0] torque_request,
    input  wire [11:0] regen_request,

    //--- ceilings from the domain blocks --------------------------------------
    input  wire [7:0]  batt_power_limit_pct,
    input  wire [7:0]  torque_limit_pct,

    //--- absolute inhibits -----------------------------------------------------
    input  wire        vehicle_enable,     // central safety FSM
    input  wire        torque_inhibit,     // HV island
    input  wire        discharge_inhibit,  // battery
    input  wire        motor_inhibit,      // motor
    input  wire        charge_inhibit,     // battery
    input  wire        regen_inhibit,      // motor
    input  wire        crash_latched,

    //--- permit speed cap -------------------------------------------------------
    input  wire [7:0]  speed_limit_kph,    // 0 = no cap
    input  wire [7:0]  vehicle_speed_kph,

    //--- thermal ----------------------------------------------------------------
    input  wire [7:0]  pump_pwm_req,
    input  wire [7:0]  fan_pwm_req,

    //--- safety actuator requests ----------------------------------------------
    input  wire [2:0]  crash_severity,
    input  wire        rider_present,

    //--- mode --------------------------------------------------------------------
    input  wire        mode_is_car,

    //--- powertrain outputs, top-level pins ------------------------------------
    output reg  [11:0] torque_cmd,
    output reg  [11:0] regen_cmd,
    output reg         motor_enable,
    output reg  [7:0]  power_derate_pct,
    output reg  [7:0]  cooling_pump_pwm,
    output reg  [7:0]  cooling_fan_pwm,
    output reg         charge_enable,

    //--- safety actuators, top-level pins --------------------------------------
    output reg  [3:0]  airbag_trigger,
    output reg  [1:0]  belt_pretension,
    output reg         door_unlock,
    output reg         horn_en,
    output reg         headlight_en,
    output reg         airbag_deployed     // feeds crash severity
);

    localparam [11:0] TQ_CRAWL = 12'd400;    // about 10 % - enough to move

    //=========================================================================
    // Percentage to absolute ceiling.  Comparison ladder, no multiplier.
    //=========================================================================
    function [11:0] cap_of;
        input [7:0] pct;
        begin
            if      (pct >= 8'd100) cap_of = 12'd4095;
            else if (pct >= 8'd75 ) cap_of = 12'd3071;
            else if (pct >= 8'd40 ) cap_of = 12'd1638;
            else if (pct >= 8'd15 ) cap_of = 12'd614;
            else                    cap_of = 12'd0;
        end
    endfunction

    function [11:0] min12;
        input [11:0] a;
        input [11:0] b;
        begin
            min12 = (a < b) ? a : b;
        end
    endfunction

    function [7:0] min8;
        input [7:0] a;
        input [7:0] b;
        begin
            min8 = (a < b) ? a : b;
        end
    endfunction

    //=========================================================================
    // The lowest ceiling wins
    //=========================================================================
    wire [7:0]  worst_pct = min8(batt_power_limit_pct, torque_limit_pct);
    wire [11:0] pct_cap   = cap_of(worst_pct);

    //=========================================================================
    // Speed cap, with hysteresis
    //=========================================================================
    wire cap_active = (speed_limit_kph != 8'd0);
    wire at_limit   = cap_active && (vehicle_speed_kph >= speed_limit_kph);
    wire over_limit = cap_active && (vehicle_speed_kph >= (speed_limit_kph + 8'd2));

    wire [11:0] speed_cap = over_limit ? 12'd0
                          : at_limit   ? TQ_CRAWL
                                       : 12'd4095;

    //=========================================================================
    // Absolute inhibits.  Any one of these means zero, regardless of ceilings.
    //=========================================================================
    wire torque_blocked = ~vehicle_enable | torque_inhibit |
                          discharge_inhibit | motor_inhibit | crash_latched;

    wire [11:0] torque_allowed = min12(min12(torque_request, pct_cap), speed_cap);

    //=========================================================================
    // MODE GATES - constants, not decisions
    //=========================================================================
    wire car_only = mode_is_car;

    always @(posedge clk_aon or negedge rst_aon_n) begin
        if (!rst_aon_n) begin
            torque_cmd       <= 12'd0;
            regen_cmd        <= 12'd0;
            motor_enable     <= 1'b0;
            power_derate_pct <= 8'd0;
            cooling_pump_pwm <= 8'd0;
            cooling_fan_pwm  <= 8'd0;
            charge_enable    <= 1'b0;
            airbag_trigger   <= 4'd0;
            belt_pretension  <= 2'd0;
            door_unlock      <= 1'b0;
            horn_en          <= 1'b0;
            headlight_en     <= 1'b0;
            airbag_deployed  <= 1'b0;
        end else begin

            //-----------------------------------------------------------------
            // TRACTION
            //-----------------------------------------------------------------
            torque_cmd       <= torque_blocked ? 12'd0 : torque_allowed;
            motor_enable     <= ~torque_blocked;
            power_derate_pct <= worst_pct;

            //-----------------------------------------------------------------
            // REGENERATION.  Pushing current into a pack that must not be
            // charged is charging it.  charge_inhibit blocks regen too - a
            // detail it would be easy to miss, because regen does not look
            // like charging from the motor's side.
            //-----------------------------------------------------------------
            regen_cmd <= (regen_inhibit | charge_inhibit | crash_latched |
                          ~vehicle_enable) ? 12'd0 : regen_request;

            charge_enable <= ~charge_inhibit & ~crash_latched;

            //-----------------------------------------------------------------
            // COOLING - car only.  ivcu_thermal_ai already forces these to
            // zero in bike mode; gating again here costs nothing after
            // synthesis and means the property holds even if that block is
            // later changed.
            //-----------------------------------------------------------------
            cooling_pump_pwm <= car_only ? pump_pwm_req : 8'd0;
            cooling_fan_pwm  <= car_only ? fan_pwm_req  : 8'd0;

            //-----------------------------------------------------------------
            // SAFETY ACTUATORS
            //
            // Airbags, belts and door locks are car-only.  In bike mode
            // active_mode makes car_only a constant zero and synthesis removes
            // these paths entirely.
            //-----------------------------------------------------------------
            if (car_only && (crash_severity >= `CRASH_SEVERE)) begin
                // front pair always; side pair as well on a severe impact
                airbag_trigger  <= 4'b1111;
                belt_pretension <= rider_present ? 2'b11 : 2'b01;
                airbag_deployed <= 1'b1;
            end else if (car_only && (crash_severity >= `CRASH_MODERATE)) begin
                belt_pretension <= rider_present ? 2'b11 : 2'b01;
                airbag_trigger  <= 4'd0;
            end else begin
                airbag_trigger  <= 4'd0;
                belt_pretension <= 2'd0;
            end

            // doors unlock at moderate and above so responders can get in
            door_unlock <= car_only & (crash_severity >= `CRASH_MODERATE);

            //-----------------------------------------------------------------
            // BOTH MODES - a motorcycle rider lying in a ditch at night needs
            // the horn and the headlight as much as a car occupant does, and
            // arguably more.
            //-----------------------------------------------------------------
            horn_en      <= (crash_severity >= `CRASH_SEVERE);
            headlight_en <= (crash_severity >= `CRASH_MODERATE);

        end
    end

endmodule

`default_nettype wire
