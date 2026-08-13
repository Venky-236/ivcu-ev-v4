//=============================================================================
// ivcu_emergency_response.v  -  what happens in the sixty seconds after
//
// Wraps the four blocks that turn an impact into help arriving:
//     ivcu_gps_receiver     real position from a real pin
//     ivcu_crash_severity   five levels, three routing flags
//     ivcu_sos_builder      a 25-byte frame, sent three times
//     ivcu_hazard_blinker   a lamp that actually blinks
//
// Everything here is in clk_aon, in the always-on power domain, for the same
// reason the HV island is: after an impact the AI cluster and the MCU may be
// unpowered, damaged, or held in reset, and none of that may stop the vehicle
// calling for help.
//
//-----------------------------------------------------------------------------
// THE ORDERING, AND WHAT IS NOT IN IT
//
//   t = 0         contactors already open.  That happened in
//                 ivcu_hv_crash_detect, combinationally, and did not pass
//                 through this file or wait for anything in it.
//   t = 0         hazard lamps start blinking
//   t = 0         doors unlock (car only - see ivcu_actuator_output_mgr)
//   t + ~10 us    severity classified from the latched peak values
//   t + ~50 us    SOS frame latched and streaming to the modem
//   t + 5 s       frame sent again
//   t + 10 s      frame sent a third time
//
// If the pack starts venting later - minutes later - severity escalates,
// the payload is re-latched, and the sequence restarts with the fire service
// added to the routing.
//
//-----------------------------------------------------------------------------
// THE HAZARD REQUEST IS AN OR, DELIBERATELY
//
// hazard_req comes from the crash latch OR the driver's button.  A driver who
// presses the button must not have it overridden by a chip that thinks nothing
// has happened, and a crash must not need the driver to press anything.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_emergency_response (
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
    input  wire [2:0]  hv_state,
    input  wire [3:0]  hv_fault_code,
    input  wire [15:0] hv_cell_tmax,
    input  wire [15:0] hv_encl_press,

    //--- vehicle context -------------------------------------------------------
    input  wire [1:0]  active_mode,
    input  wire [7:0]  vehicle_speed_kph,
    input  wire        airbag_deployed,
    input  wire        rider_present,
    input  wire        thermal_runaway_alarm,
    input  wire        hazard_button,      // driver's switch, synchronised
    input  wire        service_clear,

    //--- GNSS module, top-level pins ------------------------------------------
    input  wire [7:0]  gps_rx_data,
    input  wire        gps_rx_valid,

    //--- telematics modem, top-level pins -------------------------------------
    output wire [7:0]  sos_tx_data,
    output wire        sos_tx_valid,
    input  wire        sos_tx_ready,

    //--- lamp driver ------------------------------------------------------------
    output wire        hazard_lights_en,

    //--- status for the register map --------------------------------------------
    output wire [2:0]  crash_severity,
    output wire [2:0]  sos_route,
    output wire [2:0]  occupant_count,
    output wire        battery_incident,
    output wire        sos_sending,
    output wire [1:0]  sos_tx_count,
    output wire        gps_fix_valid,
    output wire        gps_fix_current,
    output wire [15:0] fix_age_ms,
    output wire [3:0]  gps_sats,
    output wire [7:0]  gps_frame_err
);

    //=========================================================================
    // SPEED AT IMPACT
    //
    // Captured on the rising edge of crash_latched.  Reading vehicle_speed_kph
    // live would report the speed of a stationary wreck, which is not the
    // number a dispatcher or an investigator needs.
    //=========================================================================
    reg [7:0] speed_at_impact;
    reg       crash_prev;

    always @(posedge clk_aon or negedge rst_aon_n) begin
        if (!rst_aon_n) begin
            speed_at_impact <= 8'd0;
            crash_prev      <= 1'b0;
        end else begin
            crash_prev <= crash_latched;
            if (crash_latched && !crash_prev) begin
                speed_at_impact <= vehicle_speed_kph;
            end else if (service_clear && !crash_latched) begin
                speed_at_impact <= 8'd0;
            end
        end
    end

    //=========================================================================
    // 1  GNSS
    //=========================================================================
    wire signed [31:0] gps_lat;
    wire signed [31:0] gps_lon;

    ivcu_gps_receiver u_gps (
        .clk_aon        (clk_aon),
        .rst_aon_n      (rst_aon_n),
        .gps_rx_data    (gps_rx_data),
        .gps_rx_valid   (gps_rx_valid),
        .gps_lat        (gps_lat),
        .gps_lon        (gps_lon),
        .gps_sats       (gps_sats),
        .gps_fix_valid  (gps_fix_valid),
        .gps_fix_current(gps_fix_current),
        .fix_age_ms     (fix_age_ms),
        .frame_err_count(gps_frame_err)
    );

    //=========================================================================
    // 2  severity
    //=========================================================================
    wire severity_valid;

    ivcu_crash_severity u_sev (
        .clk_aon              (clk_aon),
        .rst_aon_n            (rst_aon_n),
        .crash_latched        (crash_latched),
        .crash_pyro           (crash_pyro),
        .crash_peak_long      (crash_peak_long),
        .crash_peak_lat       (crash_peak_lat),
        .crash_direction      (crash_direction),
        .pyro_fired           (pyro_fired),
        .hv_isolated          (hv_isolated),
        .hv_fault_code        (hv_fault_code),
        .hv_cell_tmax         (hv_cell_tmax),
        .hv_encl_press        (hv_encl_press),
        .active_mode          (active_mode),
        .speed_at_impact_kph  (speed_at_impact),
        .airbag_deployed      (airbag_deployed),
        .rider_present        (rider_present),
        .thermal_runaway_alarm(thermal_runaway_alarm),
        .service_clear        (service_clear),
        .crash_severity       (crash_severity),
        .sos_route            (sos_route),
        .occupant_count       (occupant_count),
        .severity_valid       (severity_valid),
        .battery_incident     (battery_incident)
    );

    //=========================================================================
    // 3  the message
    //=========================================================================
    ivcu_sos_builder u_sos (
        .clk_aon            (clk_aon),
        .rst_aon_n          (rst_aon_n),
        .crash_severity     (crash_severity),
        .sos_route          (sos_route),
        .occupant_count     (occupant_count),
        .severity_valid     (severity_valid),
        .battery_incident   (battery_incident),
        .gps_lat            (gps_lat),
        .gps_lon            (gps_lon),
        .fix_age_ms         (fix_age_ms),
        .crash_direction    (crash_direction),
        .crash_peak_long    (crash_peak_long),
        .crash_peak_lat     (crash_peak_lat),
        .hv_cell_tmax       (hv_cell_tmax),
        .hv_encl_press      (hv_encl_press),
        .speed_at_impact_kph(speed_at_impact),
        .hv_isolated        (hv_isolated),
        .pyro_fired         (pyro_fired),
        .hv_state           (hv_state),
        .hv_fault_code      (hv_fault_code),
        .active_mode        (active_mode),
        .sos_tx_data        (sos_tx_data),
        .sos_tx_valid       (sos_tx_valid),
        .sos_tx_ready       (sos_tx_ready),
        .sos_sending        (sos_sending),
        .sos_tx_count       (sos_tx_count)
    );

    //=========================================================================
    // 4  the lamps
    //=========================================================================
    ivcu_hazard_blinker u_blink (
        .clk_aon   (clk_aon),
        .rst_aon_n (rst_aon_n),
        .hazard_req(crash_latched | hazard_button),
        .hazard_out(hazard_lights_en)
    );

endmodule

`default_nettype wire
