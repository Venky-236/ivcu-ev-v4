//=============================================================================
// ivcu_crash_severity.v  -  how bad was it, and who needs to be told
//
// V3 had one sos_signal bit and a hardcoded crash_speed of 500.  There was no
// severity, no routing, and no way for the message to say anything that would
// change what a dispatcher sends.
//
//-----------------------------------------------------------------------------
// FIVE LEVELS, AND EACH ONE CHANGES WHO IS DISPATCHED
//
//  1 MINOR      hazards, owner's phone.  No emergency call.  A car park bump
//               should not send an ambulance.
//  2 MODERATE   + HV disconnect, doors unlocked, POLICE notified with position
//  3 SEVERE     + AMBULANCE.  Airbag deployed, or a bike down above 30 km/h.
//               Occupant count goes in the message.
//  4 CRITICAL   + FIRE, flagged as a lithium battery incident.
//
//-----------------------------------------------------------------------------
// WHY LEVEL 4 EXISTS AND WHY ONLY THIS CHIP CAN RAISE IT
//
// A lithium fire is fought differently from a petrol fire.  Water on a
// venting pack can make it worse; the pack can reignite hours after it looks
// out; and a crew cutting into a vehicle whose contactors have welded is
// cutting into 400 volts.
//
// Nothing else in the vehicle knows all three of those things at once.  This
// block does: it sees the isolation state, the enclosure pressure, the cell
// temperature and whether the pyro fuse fired.  Level 4 is the difference
// between a crew arriving prepared and a crew arriving surprised, and it costs
// three extra bits in a message that is being sent anyway.
//
//-----------------------------------------------------------------------------
// A MOTORCYCLE FALLING OVER IN A CAR PARK IS NOT A SEVERE CRASH
//
// Below TIPOVER_SPEED_MIN a fall is a minor event: hazards on, owner told, no
// ambulance.  Above it, the rider has hit the ground at speed and it is
// severe.  Getting this wrong in the loud direction would teach riders to
// disable the feature, which is worse than not having it.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_crash_severity (
    input  wire        clk_aon,
    input  wire        rst_aon_n,

    //--- from the HV safety island -------------------------------------------
    input  wire        crash_latched,
    input  wire        crash_pyro,
    input  wire [15:0] crash_peak_long,
    input  wire [15:0] crash_peak_lat,
    input  wire [2:0]  crash_direction,
    input  wire        pyro_fired,
    input  wire        hv_isolated,
    input  wire [3:0]  hv_fault_code,
    input  wire [15:0] hv_cell_tmax,
    input  wire [15:0] hv_encl_press,

    //--- vehicle context -------------------------------------------------------
    input  wire [1:0]  active_mode,
    input  wire [7:0]  speed_at_impact_kph,
    input  wire        airbag_deployed,
    input  wire        rider_present,
    input  wire        thermal_runaway_alarm,
    input  wire        service_clear,

    //--- verdict ----------------------------------------------------------------
    output reg  [2:0]  crash_severity,
    output reg  [2:0]  sos_route,          // bit 0 police, 1 ambulance, 2 fire
    output reg  [2:0]  occupant_count,
    output reg         severity_valid,     // a classification has been made
    output reg         battery_incident    // lithium flag for the fire service
);

    localparam [15:0] ZERO_G      = 16'd2048;
    localparam [15:0] MAG_MINOR   = `CRASH_THRESH_MINOR    - ZERO_G;   //  552
    localparam [15:0] MAG_MODER   = `CRASH_THRESH_MODERATE - ZERO_G;   //  952
    localparam [15:0] MAG_HARD    = `CRASH_THRESH_HARD     - ZERO_G;   // 1352

    localparam [15:0] T_RUNAWAY   = 16'd2457;   //  80 C
    localparam [15:0] P_VENTING   = 16'd1638;   //   4 bar

    localparam [2:0]  DIR_FALL    = 3'd3;

    function [15:0] mag;
        input [15:0] v;
        begin
            mag = (v > ZERO_G) ? (v - ZERO_G) : (ZERO_G - v);
        end
    endfunction

    wire [15:0] m_long = mag(crash_peak_long);
    wire [15:0] m_lat  = mag(crash_peak_lat);
    wire [15:0] m_peak = (m_long > m_lat) ? m_long : m_lat;

    wire is_bike = (active_mode == `MODE_BIKE);

    //-------------------------------------------------------------------------
    // A fall at speed on a motorcycle is severe regardless of peak g, because
    // the rider has left the vehicle.  A fall while parked is not.
    //-------------------------------------------------------------------------
    wire fall_severe = is_bike && (crash_direction == DIR_FALL) &&
                       (speed_at_impact_kph > `TIPOVER_SPEED_MIN);

    wire fall_minor  = is_bike && (crash_direction == DIR_FALL) &&
                       (speed_at_impact_kph <= `TIPOVER_SPEED_MIN);

    //-------------------------------------------------------------------------
    // The battery is a hazard to the responders, not just to the occupants.
    //-------------------------------------------------------------------------
    wire hv_compromised = ~hv_isolated |
                          (hv_fault_code == `HVF_CONTACTOR_WELD) |
                          (hv_fault_code == `HVF_DISCHARGE_TO)   |
                          (hv_fault_code == `HVF_ISOLATION);

    wire pack_hazard = thermal_runaway_alarm |
                       (hv_cell_tmax  > T_RUNAWAY) |
                       (hv_encl_press > P_VENTING) |
                       pyro_fired;

    //-------------------------------------------------------------------------
    // Classification.  Worst first.
    //-------------------------------------------------------------------------
    wire [2:0] sev_now =
          (!crash_latched)                                ? `CRASH_NONE     :
          (pack_hazard || (hv_compromised && crash_pyro)) ? `CRASH_CRITICAL :
          (airbag_deployed || crash_pyro ||
           (m_peak > MAG_HARD) || fall_severe)            ? `CRASH_SEVERE   :
          ((m_peak > MAG_MODER) && !fall_minor)           ? `CRASH_MODERATE :
          ((m_peak > MAG_MINOR) || fall_minor)            ? `CRASH_MINOR    :
                                                            `CRASH_MINOR;

    //-------------------------------------------------------------------------
    // Routing.  Cumulative: level 4 sends all three services.
    //-------------------------------------------------------------------------
    function [2:0] route_of;
        input [2:0] sev;
        begin
            case (sev)
                `CRASH_MINOR    : route_of = 3'b000;  // owner only
                `CRASH_MODERATE : route_of = 3'b001;  // police
                `CRASH_SEVERE   : route_of = 3'b011;  // police + ambulance
                `CRASH_CRITICAL : route_of = 3'b111;  // + fire
                default         : route_of = 3'b000;
            endcase
        end
    endfunction

    always @(posedge clk_aon or negedge rst_aon_n) begin
        if (!rst_aon_n) begin
            crash_severity   <= `CRASH_NONE;
            sos_route        <= 3'b000;
            occupant_count   <= 3'd0;
            severity_valid   <= 1'b0;
            battery_incident <= 1'b0;
        end else if (service_clear && !crash_latched) begin
            crash_severity   <= `CRASH_NONE;
            sos_route        <= 3'b000;
            occupant_count   <= 3'd0;
            severity_valid   <= 1'b0;
            battery_incident <= 1'b0;
        end else if (crash_latched) begin

            //-----------------------------------------------------------------
            // Severity only ever climbs.  A pack that starts venting thirty
            // seconds after the impact escalates the incident; nothing
            // de-escalates it, because the responders are already moving on
            // the information they were given.
            //-----------------------------------------------------------------
            if (sev_now > crash_severity) begin
                crash_severity <= sev_now;
                sos_route      <= route_of(sev_now);
            end

            if (pack_hazard) begin
                battery_incident <= 1'b1;
                sos_route[`SOS_ROUTE_FIRE] <= 1'b1;
            end

            //-----------------------------------------------------------------
            // Occupants.  A car reports what the seat sensors say.  A
            // motorcycle reports at least one, possibly two - there is no
            // pillion sensor, and telling a dispatcher "one" when a passenger
            // may be lying in a ditch is the wrong error to make.
            //-----------------------------------------------------------------
            if (is_bike) begin
                occupant_count <= 3'd2;      // rider, possibly pillion
            end else begin
                occupant_count <= rider_present ? 3'd2 : 3'd1;
            end

            severity_valid <= 1'b1;
        end
    end

endmodule

`default_nettype wire
