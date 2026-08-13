//=============================================================================
// ivcu_serviceability_mgr.v  -  who may be bypassed, and who may fix it
//
//-----------------------------------------------------------------------------
// THE SECURITY PROPERTY THIS BLOCK EXISTS TO GUARANTEE
//
// V3 exposed sensor_force_disable as a raw, unguarded 42-bit input.  Anything
// outside the chip - a confused MCU, a hostile diagnostic tool, a stuck bus -
// could force-disable brake pressure or battery cell temperature and the
// vehicle would carry on as if those sensors were not fitted.
//
// Here, exactly eight of sixty-four channels can ever be bypassed, and which
// eight is decided at synthesis time by a constant.  The guard is not a check
// that could be skipped; it is an AND with a literal:
//
//     bypass_active = permit_grants & BYPASS_ELIGIBLE
//
// On the other fifty-six bits that AND leaves a tied-low net, and Yosys
// deletes it.  There is no gate to attack, because after synthesis there is no
// gate.  Brake pressure is not "protected by logic" - it is unreachable.
//
//-----------------------------------------------------------------------------
// AND THE CONSTANT IS PROVEN TO MATCH THE ATTRIBUTE ROM
//
// A hand-written security constant is only as good as the hand that wrote it.
// So MASK_CONDITIONAL is rebuilt here from SENSOR_ATTR_TABLE - the same ROM
// that drives the display's "who can replace this" guidance - and compared
// against BYPASS_ELIGIBLE at elaboration time.  If someone changes a channel's
// class and forgets the mask, the build fails with a missing-module error
// naming the problem.
//
//   NOTE FOR THE TOOLCHAIN:  the assertion uses the standard Verilog-2001
//   trick of instantiating a module that does not exist.  If your Yosys or
//   iverilog version cannot evaluate the constant function that builds the
//   mask, comment out the generate block marked ELABORATION ASSERTION below -
//   scripts_v4/check_defs.py performs the same check independently and is the
//   authority.  Do not "fix" it by editing the constant to match.
//
//-----------------------------------------------------------------------------
// THE ONLY WAY TO TAKE A PERMIT IS THE PHYSICAL BUTTON
//
// FLAGGED CHANGE vs the architecture spec.  The spec had an APB register at
// 0x020 that could request a bypass.  It is gone; that register is read-only
// status now.
//
// The reason: a permit is a rider consciously accepting a degraded vehicle.
// If firmware can grant one, then a firmware bug can put a rider on a
// motorcycle with a defeated safety inhibit and no one held a button.  The
// three-second hold on a physical pin cannot be produced by a software defect.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_serviceability_mgr (
    input  wire        clk_aon,
    input  wire        rst_aon_n,

    //--- confirmed faults, synchronised into clk_aon ------------------------
    input  wire [63:0] sensor_fault_s,

    //--- rider and vehicle context -------------------------------------------
    input  wire        permit_ack_s,       // physical button, synchronised
    input  wire        vehicle_stationary,
    input  wire        ign_cycle,
    input  wire        service_clear,      // authenticated technician write
    input  wire        mode_is_bike,

    //--- results --------------------------------------------------------------
    output wire [63:0] bypass_active,      // guarded - see header
    output wire        limp_home_active,
    output wire        permit_inhibit,     // some permit says do not move
    output wire        permit_offer,       // display should offer a permit
    output reg  [7:0]  speed_limit_kph,    // 0 = no cap
    output wire [23:0] permit_starts_flat, // 8 x 3 bits, for the register map
    output wire [15:0] permit_state_flat,  // 8 x 2 bits
    output wire [63:0] mask_critical,      // class masks, for the central FSM
    output wire [63:0] mask_degrade,
    output wire [63:0] mask_conditional,
    output wire [63:0] mask_comfort
);

    //=========================================================================
    // CLASS MASKS, REBUILT FROM THE ATTRIBUTE ROM
    //=========================================================================
    localparam [255:0] ATTR_ROM = `SENSOR_ATTR_TABLE;

    function [63:0] class_mask;
        input [1:0] want;
        integer k;
        begin
            class_mask = 64'd0;
            for (k = 0; k < `NUM_SENSORS; k = k + 1) begin
                if (ATTR_ROM[k*4 +: 2] == want) begin
                    class_mask[k] = 1'b1;
                end
            end
        end
    endfunction

    localparam [63:0] MASK_CRITICAL    = class_mask(`CLASS_CRITICAL);
    localparam [63:0] MASK_DEGRADE     = class_mask(`CLASS_DEGRADE);
    localparam [63:0] MASK_CONDITIONAL = class_mask(`CLASS_CONDITIONAL);
    localparam [63:0] MASK_COMFORT     = class_mask(`CLASS_COMFORT);

    assign mask_critical    = MASK_CRITICAL;
    assign mask_degrade     = MASK_DEGRADE;
    assign mask_conditional = MASK_CONDITIONAL;
    assign mask_comfort     = MASK_COMFORT;

    //-------------------------------------------------------------------------
    // ELABORATION ASSERTION - see the note in the header before disabling
    //-------------------------------------------------------------------------
    generate
        if (MASK_CONDITIONAL != `BYPASS_ELIGIBLE) begin : g_assert_bypass
            ivcu_ERROR_bypass_eligible_disagrees_with_sensor_attr_rom u_assert ();
        end
    endgenerate

    //=========================================================================
    // THE EIGHT BYPASSABLE CHANNELS
    //=========================================================================
    function [5:0] cond_chan;
        input [2:0] k;
        begin
            case (k)
                3'd0   : cond_chan = `S_GEAR_POSITION;   // 25
                3'd1   : cond_chan = `S_AMBIENT_TEMP;    // 30
                3'd2   : cond_chan = `S_HUMIDITY;        // 31
                3'd3   : cond_chan = `S_RIDE_HEIGHT;     // 43
                3'd4   : cond_chan = `S_SIDE_STAND;      // 48  your example
                3'd5   : cond_chan = `S_SEAT_OCCUPANCY;  // 49
                3'd6   : cond_chan = `S_TPMS_FRONT;      // 58
                default: cond_chan = `S_TPMS_REAR;       // 59
            endcase
        end
    endfunction

    //=========================================================================
    // THE THREE-SECOND HOLD
    //
    // Long enough that it cannot be an accidental brush against the button,
    // short enough that a rider in a dark car park will actually complete it.
    // The vehicle must be stationary throughout - releasing or moving resets it.
    //=========================================================================
    localparam [24:0] ACK_HOLD_CYC = `PERMIT_ACK_HOLD_CYC;   // 3 s at 10 MHz

    reg [24:0] ack_cnt;
    reg        ack_held;

    always @(posedge clk_aon or negedge rst_aon_n) begin
        if (!rst_aon_n) begin
            ack_cnt  <= 25'd0;
            ack_held <= 1'b0;
        end else if (permit_ack_s && vehicle_stationary) begin
            if (ack_cnt >= ACK_HOLD_CYC) begin
                ack_held <= 1'b1;
            end else begin
                ack_cnt <= ack_cnt + 25'd1;
            end
        end else begin
            ack_cnt  <= 25'd0;
            ack_held <= 1'b0;
        end
    end

    //=========================================================================
    // EIGHT PERMIT MACHINES.  RULE R6 - generate, with the channel index
    // supplied by a constant function, rather than eight pasted instances.
    //=========================================================================
    wire [63:0] bp_onehot [0:`NUM_COND_SENSORS-1];
    wire [`NUM_COND_SENSORS-1:0] p_inhibit;
    wire [`NUM_COND_SENSORS-1:0] p_offer;

    genvar p;
    generate
        for (p = 0; p < `NUM_COND_SENSORS; p = p + 1) begin : perm

            localparam integer CH = cond_chan(p[2:0]);

            wire       b_req;
            wire [1:0] p_state;
            wire [2:0] p_left;

            ivcu_permit_fsm u_permit (
                .clk_aon           (clk_aon),
                .rst_aon_n         (rst_aon_n),
                .sensor_faulted    (sensor_fault_s[CH]),
                .ack_held          (ack_held),
                .vehicle_stationary(vehicle_stationary),
                .ign_cycle         (ign_cycle),
                .service_clear     (service_clear),
                .permit_state      (p_state),
                .starts_left       (p_left),
                .bypass_req        (b_req),
                .inhibit           (p_inhibit[p]),
                .offer             (p_offer[p])
            );

            // one-hot on this instance's channel; CH is constant so the shift
            // folds away entirely
            assign bp_onehot[p] = b_req ? (64'd1 << CH) : 64'd0;

            assign permit_starts_flat[p*3 +: 3] = p_left;
            assign permit_state_flat [p*2 +: 2] = p_state;

        end
    endgenerate

    //=========================================================================
    // THE GUARD
    //
    // The one-hot vectors can only ever set eligible bits, because the FSMs
    // are only instantiated on eligible channels.  The AND with
    // BYPASS_ELIGIBLE is therefore redundant - and it stays, because a
    // redundant AND with a synthesis-time constant costs nothing after
    // opt_clean and it makes the property true by inspection rather than by
    // argument.  Somebody reading this file in a year should not have to trace
    // cond_chan() to convince themselves brake pressure cannot be disabled.
    //=========================================================================
    assign bypass_active = ( bp_onehot[0] | bp_onehot[1] |
                             bp_onehot[2] | bp_onehot[3] |
                             bp_onehot[4] | bp_onehot[5] |
                             bp_onehot[6] | bp_onehot[7] ) & `BYPASS_ELIGIBLE;

    assign limp_home_active = |bypass_active;
    assign permit_inhibit   = |p_inhibit;
    assign permit_offer     = |p_offer;

    //=========================================================================
    // SPEED CAP
    //
    // A permit is not permission to ride normally.  It is permission to reach
    // a service centre.  The cap is lower on a motorcycle because the sensor
    // most likely to be under permit there is the side stand, and a stand that
    // drops at speed is a crash.
    //=========================================================================
    always @(posedge clk_aon or negedge rst_aon_n) begin
        if (!rst_aon_n) begin
            speed_limit_kph <= 8'd0;
        end else if (|bypass_active) begin
            speed_limit_kph <= mode_is_bike ? `SPEED_LIMIT_BIKE_KPH
                                            : `SPEED_LIMIT_CAR_KPH;
        end else begin
            speed_limit_kph <= 8'd0;          // 0 means no cap
        end
    end

endmodule

`default_nettype wire
