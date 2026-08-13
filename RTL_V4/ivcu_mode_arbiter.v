//=============================================================================
// ivcu_mode_arbiter.v  -  is this a car or a motorcycle?
//
//-----------------------------------------------------------------------------
// WHAT V3 DID
//
//     sensor_signature <= sensor_signature + 1;
//     // in real design, this would read sensors
//
// and it counted car versus bike off an LSB toggle.  MODE_AUTO was a coin
// flip.  The comment admits it.  Given that the mode decides whether airbags
// exist, whether the side-stand inhibit applies, and which sensors are even
// fitted, a coin flip is not an acceptable answer.
//
//-----------------------------------------------------------------------------
// THREE-STAGE RESOLUTION, AND IT NEVER GUESSES
//
//  1  mode_strap[1:0] - two pins bonded at manufacture.
//        00 = car, 01 = bike.  This is the answer in a real product.  A
//        vehicle does not change from a car into a motorcycle, and asking
//        silicon to work out what chassis it is bolted to when the factory
//        already knows is solving a problem nobody has.
//        10 and 11 request auto-detection.
//
//  2  signature detection - only when the strap asks for it.  All 64 channels
//     are enabled and the arbiter watches which ones actually answer.  A
//     sensor that is not fitted does not respond, and that absence is
//     evidence.
//
//  3  MODE_SAFE - if the evidence is contradictory or never converges.
//     Vehicle inhibited, display says the configuration is not recognised.
//     Refusing to move is a correct answer.  Guessing is not.
//
//-----------------------------------------------------------------------------
// MODE IS LATCHED, AND CHANGING IT HAS PRECONDITIONS
//
// A mid-ride mode change would reconfigure the airbags of a moving vehicle.
// So mode_req from the MCU is honoured only when the vehicle is stationary AND
// the high voltage is off.  Both, not either.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_mode_arbiter (
    input  wire        clk_aon,
    input  wire        rst_aon_n,

    //--- manufacture-time strap, already synchronised -----------------------
    input  wire [1:0]  mode_strap_s,

    //--- evidence from the acquisition path (synchronised into clk_aon) -----
    input  wire [63:0] sensor_fresh_s,   // channel has answered at least once
    input  wire [63:0] sensor_dead_s,    // channel is not answering
    input  wire        sweep_done_s,     // one pulse per completed sweep

    //--- runtime change request from the MCU --------------------------------
    input  wire [1:0]  mode_req,
    input  wire        mode_req_valid,
    input  wire        vehicle_stationary,
    input  wire        hv_on,

    //--- result ---------------------------------------------------------------
    output reg  [1:0]  active_mode,
    output reg         mode_resolved,    // detection has finished
    output reg  [7:0]  detect_sweeps     // how many sweeps it took, for the log
);

    //-------------------------------------------------------------------------
    // Channel indices as integers.  Bit-selects are safe with the sized macros,
    // but naming them here documents what each piece of evidence is.
    //-------------------------------------------------------------------------
    localparam integer C_SIDE_STAND = `S_SIDE_STAND;      // 48  bike only
    localparam integer C_TIP_OVER   = `S_TIP_OVER;        // 52  bike only
    localparam integer C_STEERING   = `S_STEERING_ANGLE;  // 42  car only
    localparam integer C_CRASH_SIDE = `S_CRASH_SIDE;      // 51  car only
    localparam integer C_WSPD_FB    = `S_WSPD_FRONT_B;    // 33  car only
    localparam integer C_WSPD_RB    = `S_WSPD_REAR_B;     // 35  car only
    localparam integer C_CL_IN      = `S_COOLANT_TEMP_IN;
    localparam integer C_CL_OUT     = `S_COOLANT_TEMP_OUT;
    localparam integer C_CL_FLOW    = `S_COOLANT_FLOW;
    localparam integer C_CL_PRESS   = `S_COOLANT_PRESSURE;

    // RULE R1 - constants, never ports
    localparam [3:0] AGREE_NEEDED  = 4'd8;    // consecutive identical verdicts
    localparam [7:0] DETECT_LIMIT  = 8'd200;  // sweeps before giving up
    localparam [4:0] MARGIN        = 5'd4;    // points of separation required

    localparam [1:0] V_UNKNOWN = 2'd0,
                     V_CAR     = 2'd1,
                     V_BIKE    = 2'd2;

    //-------------------------------------------------------------------------
    // "responds" means the channel has answered and is not currently timing
    // out.  A sensor that is not physically fitted times out forever.
    //-------------------------------------------------------------------------
    function resp;
        input fresh;
        input dead;
        begin
            resp = fresh & ~dead;
        end
    endfunction

    wire r_stand  = resp(sensor_fresh_s[C_SIDE_STAND], sensor_dead_s[C_SIDE_STAND]);
    wire r_tip    = resp(sensor_fresh_s[C_TIP_OVER],   sensor_dead_s[C_TIP_OVER]);
    wire r_steer  = resp(sensor_fresh_s[C_STEERING],   sensor_dead_s[C_STEERING]);
    wire r_cside  = resp(sensor_fresh_s[C_CRASH_SIDE], sensor_dead_s[C_CRASH_SIDE]);
    wire r_wfb    = resp(sensor_fresh_s[C_WSPD_FB],    sensor_dead_s[C_WSPD_FB]);
    wire r_wrb    = resp(sensor_fresh_s[C_WSPD_RB],    sensor_dead_s[C_WSPD_RB]);
    wire r_cl1    = resp(sensor_fresh_s[C_CL_IN],      sensor_dead_s[C_CL_IN]);
    wire r_cl2    = resp(sensor_fresh_s[C_CL_OUT],     sensor_dead_s[C_CL_OUT]);
    wire r_cl3    = resp(sensor_fresh_s[C_CL_FLOW],    sensor_dead_s[C_CL_FLOW]);
    wire r_cl4    = resp(sensor_fresh_s[C_CL_PRESS],   sensor_dead_s[C_CL_PRESS]);

    wire four_wheels = r_wfb & r_wrb;      // second and fourth wheel present
    wire two_wheels  = ~r_wfb & ~r_wrb;    // they are not
    wire coolant_all = r_cl1 & r_cl2 & r_cl3 & r_cl4;

    //-------------------------------------------------------------------------
    // Evidence scoring.  Conditional adds only - no multipliers, no dividers.
    //-------------------------------------------------------------------------
    wire [4:0] car_pts  = (r_steer     ? 5'd3 : 5'd0)
                        + (r_cside     ? 5'd2 : 5'd0)
                        + (four_wheels ? 5'd3 : 5'd0)
                        + (coolant_all ? 5'd2 : 5'd0);

    wire [4:0] bike_pts = (r_stand     ? 5'd3 : 5'd0)
                        + (r_tip       ? 5'd3 : 5'd0)
                        + (two_wheels  ? 5'd3 : 5'd0);

    wire [1:0] verdict = (car_pts  >= (bike_pts + MARGIN)) ? V_CAR  :
                         (bike_pts >= (car_pts  + MARGIN)) ? V_BIKE :
                                                             V_UNKNOWN;

    //-------------------------------------------------------------------------
    // Strap decode
    //-------------------------------------------------------------------------
    wire strap_is_car  = (mode_strap_s == 2'b00);
    wire strap_is_bike = (mode_strap_s == 2'b01);
    wire strap_auto    = mode_strap_s[1];       // 10 or 11

    //-------------------------------------------------------------------------
    // Detection state
    //-------------------------------------------------------------------------
    reg [1:0] last_verdict;
    reg [3:0] agree_cnt;

    always @(posedge clk_aon or negedge rst_aon_n) begin
        if (!rst_aon_n) begin
            active_mode   <= `MODE_DETECT;
            mode_resolved <= 1'b0;
            last_verdict  <= V_UNKNOWN;
            agree_cnt     <= 4'd0;
            detect_sweeps <= 8'd0;
        end else begin

            //-----------------------------------------------------------------
            // A strapped vehicle is resolved immediately.  No detection, no
            // sweeps, no opportunity to get it wrong.
            //-----------------------------------------------------------------
            if (!mode_resolved && !strap_auto) begin
                active_mode   <= strap_is_car  ? `MODE_CAR  :
                                 strap_is_bike ? `MODE_BIKE : `MODE_SAFE;
                mode_resolved <= 1'b1;
            end

            //-----------------------------------------------------------------
            // Auto-detection: eight consecutive sweeps must agree
            //-----------------------------------------------------------------
            else if (!mode_resolved && strap_auto && sweep_done_s) begin

                if (detect_sweeps != 8'hFF) detect_sweeps <= detect_sweeps + 8'd1;

                if ((verdict != V_UNKNOWN) && (verdict == last_verdict)) begin
                    if (agree_cnt >= (AGREE_NEEDED - 4'd1)) begin
                        active_mode   <= (verdict == V_CAR) ? `MODE_CAR
                                                            : `MODE_BIKE;
                        mode_resolved <= 1'b1;
                    end else begin
                        agree_cnt <= agree_cnt + 4'd1;
                    end
                end else begin
                    agree_cnt    <= 4'd0;
                    last_verdict <= verdict;
                end

                // the evidence never converged.  Refuse to guess.
                if (detect_sweeps >= DETECT_LIMIT) begin
                    active_mode   <= `MODE_SAFE;
                    mode_resolved <= 1'b1;
                end
            end

            //-----------------------------------------------------------------
            // Runtime change.  Stationary AND high voltage off.  Both.
            //-----------------------------------------------------------------
            else if (mode_resolved && mode_req_valid &&
                     vehicle_stationary && !hv_on) begin
                if ((mode_req == `MODE_CAR) || (mode_req == `MODE_BIKE)) begin
                    active_mode <= mode_req;
                end
            end
        end
    end

endmodule

`default_nettype wire
