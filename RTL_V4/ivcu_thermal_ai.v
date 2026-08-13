//=============================================================================
// ivcu_thermal_ai.v  -  cooling loop decisions, channels 26 to 31
//
// A MOTORCYCLE HAS NO RADIATOR, AND THIS BLOCK KNOWS IT
//
// Channels 26 to 29 - coolant inlet, outlet, flow and pressure - are car-only
// in the mode mask.  In bike mode they are not scanned, their front ends are
// unpowered, and their status reads SS_DISABLED_MODE.
//
// So in bike mode this block forces the pump and fan drives to zero and stops
// reasoning about a loop that does not exist.  It does NOT report a cooling
// fault on a motorcycle.  That distinction is the whole point of having a
// separate SS_DISABLED_MODE code: "not fitted" and "not working" are different
// statements, and only one of them should worry a rider.
//
//-----------------------------------------------------------------------------
// THE MEASUREMENT THAT ACTUALLY DETECTS A DEAD PUMP
//
// Flow sensors fail in the most inconvenient way: they report flow.  A
// paddle-wheel sensor whose bearing has seized reads a constant plausible
// value forever.
//
// Temperature rise across the loop cannot lie in the same way.  If the pump is
// commanded on and the outlet is not hotter than the inlet, either there is no
// flow or there is no heat - and this block knows whether there is heat,
// because it can see the motor and inverter temperatures.  That cross-check
// lives in ivcu_sensor_plausibility (check 7) and its verdict arrives here as
// an implausible flag on the flow channel.
//
//-----------------------------------------------------------------------------
// PWM WITHOUT A MULTIPLIER
//
// Duty is a ladder of constants selected by the hottest thing in the system.
// A proportional controller would need a multiply by an error term; there is
// no evidence a vehicle cooling loop benefits from one, and RULE R5 means the
// obvious implementation is not available anyway.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_thermal_ai (
    input  wire        clk_ai,
    input  wire        rst_ai_n,

    //--- conditioned values, synchronised into clk_ai -----------------------
    input  wire [15:0] v_coolant_in,
    input  wire [15:0] v_coolant_out,
    input  wire [15:0] v_coolant_flow,
    input  wire [15:0] v_coolant_press,
    input  wire [15:0] v_ambient_t,

    //--- the things being cooled ---------------------------------------------
    input  wire [15:0] v_motor_temp,
    input  wire [15:0] v_inverter_temp,
    input  wire [15:0] v_cell_temp_max,

    //--- trust ----------------------------------------------------------------
    input  wire        dead_coolant,       // any coolant channel not answering
    input  wire        implausible_flow,   // plausibility check 7 fired

    //--- mode -----------------------------------------------------------------
    input  wire        mode_is_car,
    input  wire        update_req,

    //--- decisions ------------------------------------------------------------
    output reg  [7:0]  pump_pwm,
    output reg  [7:0]  fan_pwm,
    output reg         pump_running,       // feeds the plausibility check
    output reg         cooling_fault,      // loop is broken - car only
    output reg         coolant_overtemp
);

    //-------------------------------------------------------------------------
    // temperature count = (T + 40) * 20.475
    //-------------------------------------------------------------------------
    localparam [15:0] CT_IDLE     = 16'd1536;  //  35 C  loop can rest
    localparam [15:0] CT_WARM     = 16'd1843;  //  50 C
    localparam [15:0] CT_HOT      = 16'd2252;  //  70 C
    localparam [15:0] CT_CRITICAL = 16'd2457;  //  80 C

    localparam [15:0] MT_WARM     = 16'd2457;  // 100 C motor
    localparam [15:0] IT_WARM     = 16'd2252;  //  70 C inverter
    localparam [15:0] BT_WARM     = 16'd1741;  //  45 C cells

    localparam [15:0] FLOW_MIN    = 16'd410;   // pump commanded, flow expected
    localparam [15:0] PRESS_MIN   = 16'd410;   // below this the loop has a leak

    localparam [7:0]  PWM_OFF     = 8'd0;
    localparam [7:0]  PWM_LOW     = 8'd64;
    localparam [7:0]  PWM_MID     = 8'd150;
    localparam [7:0]  PWM_MAX     = 8'd255;

    //-------------------------------------------------------------------------
    // How hot is the hottest thing that this loop is responsible for
    //-------------------------------------------------------------------------
    wire heat_low  = (v_motor_temp    > MT_WARM) |
                     (v_inverter_temp > IT_WARM) |
                     (v_cell_temp_max > BT_WARM) |
                     (v_coolant_out   > CT_WARM);

    wire heat_high = (v_coolant_out > CT_HOT) |
                     (v_inverter_temp > (IT_WARM + 16'd205));  // +10 C

    wire heat_crit = (v_coolant_out > CT_CRITICAL);

    //-------------------------------------------------------------------------
    // A hot ambient means the loop has less headroom, so it starts earlier.
    //-------------------------------------------------------------------------
    wire hot_day = (v_ambient_t > 16'd2457);   // 80 C sensor reading

    //-------------------------------------------------------------------------
    // Loop integrity.  Only meaningful when the loop exists.
    //-------------------------------------------------------------------------
    wire no_flow  = (v_coolant_flow  < FLOW_MIN);
    wire no_press = (v_coolant_press < PRESS_MIN);

    wire loop_broken = mode_is_car &
                       ( dead_coolant | implausible_flow |
                         (pump_running & no_flow) |
                         no_press );

    always @(posedge clk_ai or negedge rst_ai_n) begin
        if (!rst_ai_n) begin
            pump_pwm         <= PWM_OFF;
            fan_pwm          <= PWM_OFF;
            pump_running     <= 1'b0;
            cooling_fault    <= 1'b0;
            coolant_overtemp <= 1'b0;
        end else if (update_req) begin

            //-----------------------------------------------------------------
            // BIKE MODE: there is no loop.  Drives off, no fault, no opinion.
            //-----------------------------------------------------------------
            if (!mode_is_car) begin
                pump_pwm         <= PWM_OFF;
                fan_pwm          <= PWM_OFF;
                pump_running     <= 1'b0;
                cooling_fault    <= 1'b0;   // NOT a fault - see the header
                coolant_overtemp <= 1'b0;

            end else begin
                //-------------------------------------------------------------
                // CAR MODE
                //
                // The pump runs hard on any critical reading, including when
                // the coolant sensors are dead.  If the loop cannot be
                // measured, running it at full is the only defensible choice -
                // the alternative is switching off cooling on a vehicle whose
                // temperature is unknown.
                //-------------------------------------------------------------
                if (heat_crit || dead_coolant) begin
                    pump_pwm <= PWM_MAX;
                    fan_pwm  <= PWM_MAX;
                end else if (heat_high) begin
                    pump_pwm <= PWM_MAX;
                    fan_pwm  <= PWM_MID;
                end else if (heat_low || hot_day) begin
                    pump_pwm <= PWM_MID;
                    fan_pwm  <= hot_day ? PWM_LOW : PWM_OFF;
                end else if (v_coolant_out > CT_IDLE) begin
                    pump_pwm <= PWM_LOW;
                    fan_pwm  <= PWM_OFF;
                end else begin
                    pump_pwm <= PWM_OFF;
                    fan_pwm  <= PWM_OFF;
                end

                pump_running     <= (pump_pwm != PWM_OFF);
                cooling_fault    <= loop_broken;
                coolant_overtemp <= heat_crit;
            end
        end
    end

endmodule

`default_nettype wire
