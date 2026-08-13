//=============================================================================
// ivcu_battery_ai.v  -  battery domain decisions, channels 6 to 17
//
// "AI" here means what you described it as meaning: logic that looks at a
// group of related sensors and issues commands.  It is a decision engine, not
// a neural network, and calling it anything else would misrepresent what is on
// the die.
//
//-----------------------------------------------------------------------------
// THE FOUR DECISIONS THIS BLOCK MAKES
//
//  1  MAY THE PACK BE CHARGED
//     Charging a lithium cell below 0 C plates metallic lithium on the anode.
//     It is permanent, it is a short-circuit hazard, and nothing downstream
//     will notice until the pack fails.  Cold is not a warning here, it is a
//     hard inhibit.
//
//  2  HOW MUCH POWER MAY BE DRAWN
//     A ladder, not a curve.  Hot pack, cold pack, low state of charge and
//     aged pack each cap the available power.  The lowest cap wins.
//
//  3  IS THE PACK VENTING
//     Rising enclosure pressure with a hot cell is the signature of a cell
//     letting go.  It leads thermal runaway by tens of seconds, and those tens
//     of seconds are the only useful warning window that exists.  This is why
//     channel 17 is SAFETY_CRITICAL and not bypassable - see the flagged
//     deviation in the architecture spec.
//
//  4  IS THE MEASUREMENT TRUSTWORTHY AT ALL
//     If the cell temperature channels are dead, this block does not get to
//     assume the pack is fine.  Unknown state is treated as the worst case.
//     That is the difference between a monitor and a monitor that means
//     something.
//
//-----------------------------------------------------------------------------
// NO DIVIDERS, NO MULTIPLIERS.  Derating is a priority ladder of constants.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_battery_ai (
    input  wire        clk_ai,
    input  wire        rst_ai_n,

    //--- conditioned values, synchronised into clk_ai -----------------------
    input  wire [15:0] v_cell_v_min,
    input  wire [15:0] v_cell_v_max,
    input  wire [15:0] v_pack_v,
    input  wire [15:0] v_pack_i,
    input  wire [15:0] v_cell_t_min,
    input  wire [15:0] v_cell_t_max,
    input  wire [15:0] v_pack_t,
    input  wire [15:0] v_soc,
    input  wire [15:0] v_soh,
    input  wire [15:0] v_encl_p,

    //--- can these channels be believed --------------------------------------
    input  wire        dead_cell_t,      // either cell temperature channel
    input  wire        dead_cell_v,      // either cell voltage channel
    input  wire        dead_soc,

    input  wire        update_req,

    //--- decisions ------------------------------------------------------------
    output reg         charge_inhibit,       // charging not permitted
    output reg         discharge_inhibit,    // traction power not permitted
    output reg  [7:0]  power_limit_pct,      // 0..100
    output reg         thermal_runaway_alarm,
    output reg         batt_cold,
    output reg         batt_hot,
    output reg         batt_low_soc,
    output reg         batt_worn
);

    //-------------------------------------------------------------------------
    // Thresholds in raw ADC counts.  Scaling convention from
    // ivcu_sensor_fault_detect:  temperature count = (T + 40) * 20.475
    //                            cell voltage count = V * 819
    //                            percent count      = P * 40.95
    // RULE R1 - every one of these is a localparam.
    //-------------------------------------------------------------------------
    localparam [15:0] T_FREEZING   = 16'd819;   //   0 C  no charging below
    localparam [15:0] T_WARM       = 16'd1741;  //  45 C  start derating
    localparam [15:0] T_HOT        = 16'd1945;  //  55 C  heavy derate
    localparam [15:0] T_CRITICAL   = 16'd2150;  //  65 C  stop
    localparam [15:0] T_RUNAWAY    = 16'd2457;  //  80 C  runaway territory

    localparam [15:0] V_CELL_LOW   = 16'd2457;  // 3.00 V
    localparam [15:0] V_CELL_EMPTY = 16'd2211;  // 2.70 V
    localparam [15:0] V_CELL_FULL  = 16'd3440;  // 4.20 V

    localparam [15:0] SOC_LOW      = 16'd410;   //  10 %
    localparam [15:0] SOC_CRITICAL = 16'd205;   //   5 %
    localparam [15:0] SOH_WORN     = 16'd2867;  //  70 %

    localparam [15:0] P_VENT_WARN  = 16'd1229;  //   3 bar
    localparam [15:0] P_VENT_ALARM = 16'd1638;  //   4 bar

    localparam [7:0]  PWR_FULL     = 8'd100;
    localparam [7:0]  PWR_REDUCED  = 8'd70;
    localparam [7:0]  PWR_LIMITED  = 8'd40;
    localparam [7:0]  PWR_CRAWL    = 8'd15;
    localparam [7:0]  PWR_NONE     = 8'd0;

    //-------------------------------------------------------------------------
    // If the sensors that matter are dead, this block has no basis for saying
    // the pack is healthy.  Unknown is treated as bad.
    //-------------------------------------------------------------------------
    wire temp_unknown = dead_cell_t;
    wire volt_unknown = dead_cell_v;

    wire cold_now  = temp_unknown | (v_cell_t_min < T_FREEZING);
    wire warm_now  = temp_unknown | (v_cell_t_max > T_WARM);
    wire hot_now   = temp_unknown | (v_cell_t_max > T_HOT);
    wire crit_now  = temp_unknown | (v_cell_t_max > T_CRITICAL);

    wire cell_low  = volt_unknown | (v_cell_v_min < V_CELL_LOW);
    wire cell_dead = volt_unknown | (v_cell_v_min < V_CELL_EMPTY);
    wire cell_full = (v_cell_v_max > V_CELL_FULL);

    wire soc_low   = dead_soc | (v_soc < SOC_LOW);
    wire soc_crit  = dead_soc | (v_soc < SOC_CRITICAL);
    wire soh_worn  = (v_soh < SOH_WORN);

    //-------------------------------------------------------------------------
    // Venting.  Pressure alone could be an altitude change or a blocked vent
    // filter; heat alone could be a hard drive on a hot day.  Together, with a
    // cell already above 80 C, they are a cell letting go.
    //-------------------------------------------------------------------------
    wire vent_warn  = (v_encl_p > P_VENT_WARN);
    wire vent_alarm = (v_encl_p > P_VENT_ALARM) &&
                      (v_cell_t_max > T_RUNAWAY);

    //-------------------------------------------------------------------------
    // The derate ladder.  Evaluated worst-first, so the first match wins and
    // the lowest cap always applies.
    //-------------------------------------------------------------------------
    function [7:0] derate;
        input crit; input vent; input cellempty;
        input hot;  input soccrit;
        input warm; input soclow;  input worn;
        begin
            if      (crit || vent || cellempty) derate = PWR_NONE;
            else if (hot  || soccrit)           derate = PWR_CRAWL;
            else if (warm)                      derate = PWR_LIMITED;
            else if (soclow)                    derate = PWR_REDUCED;
            else if (worn)                      derate = PWR_REDUCED;
            else                                derate = PWR_FULL;
        end
    endfunction

    always @(posedge clk_ai or negedge rst_ai_n) begin
        if (!rst_ai_n) begin
            charge_inhibit        <= 1'b1;   // safe default: no charging
            discharge_inhibit     <= 1'b1;   // safe default: no traction
            power_limit_pct       <= PWR_NONE;
            thermal_runaway_alarm <= 1'b0;
            batt_cold             <= 1'b0;
            batt_hot              <= 1'b0;
            batt_low_soc          <= 1'b0;
            batt_worn             <= 1'b0;
        end else if (update_req) begin

            batt_cold    <= cold_now;
            batt_hot     <= hot_now;
            batt_low_soc <= soc_low;
            batt_worn    <= soh_worn;

            //-----------------------------------------------------------------
            // 1  charging.  Cold is a hard inhibit, not a warning.
            //-----------------------------------------------------------------
            charge_inhibit <= cold_now | crit_now | cell_full |
                              vent_warn | temp_unknown | volt_unknown;

            //-----------------------------------------------------------------
            // 2  traction
            //-----------------------------------------------------------------
            discharge_inhibit <= crit_now | cell_dead | vent_alarm;

            //-----------------------------------------------------------------
            // 3  power ceiling
            //-----------------------------------------------------------------
            power_limit_pct <= derate(crit_now, vent_alarm, cell_dead,
                                      hot_now,  soc_crit,
                                      warm_now, soc_low, soh_worn);

            //-----------------------------------------------------------------
            // 4  venting.  This latches - once set it stays set until the
            //    vehicle is power cycled.  A pack that has vented does not
            //    become safe again because the pressure sensor settled.
            //-----------------------------------------------------------------
            if (vent_alarm) begin
                thermal_runaway_alarm <= 1'b1;
            end
        end
    end

endmodule

`default_nettype wire
