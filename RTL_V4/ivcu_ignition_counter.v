//=============================================================================
// ivcu_ignition_counter.v  -  what "one start" actually means
//
// The permit you described counts starts: five of them, enough to reach a
// service centre.  This block defines the unit.
//
//-----------------------------------------------------------------------------
// WHAT V3 COUNTED INSTEAD
//
//   GRACE_PERIOD_COUNT = 5
//   GRACE_TIMEOUT      = 32'd1_000_000   at 50 MHz
//
//     1,000,000 / 50 MHz = 20 ms per period
//     5 periods          = 100 ms total
//
// One hundred milliseconds, and then the sensor was permanently disabled.
// V3 counted fault timeouts, not ignition cycles, and there was no key-on
// detection anywhere in the design.  The feature you described did not exist.
//
//-----------------------------------------------------------------------------
// A START IS A RISING EDGE WITH THE VEHICLE STATIONARY
//
// The stationary qualifier is not decoration.  Without it:
//   - a brownout on the 12 V rail mid-ride looks like a new start
//   - a loose key barrel vibrating on a rough road burns the rider's budget
//   - a rider who stalls in traffic and restarts loses a permit for it
//
// Any of those turns "five starts to reach the service centre" into "some
// number of starts, possibly zero, depending on your wiring".  The rider needs
// to be able to trust the number on the display.
//
// Fifty milliseconds of debounce on top, because a mechanical key switch
// bounces for tens of milliseconds and each bounce would otherwise be a start.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_ignition_counter (
    input  wire        clk_aon,
    input  wire        rst_aon_n,

    //--- direct pin, already through ivcu_cdc_bit_sync ----------------------
    input  wire        ignition_on_s,
    input  wire        vehicle_stationary,

    //--- results --------------------------------------------------------------
    output reg         ignition_stable,   // debounced key state
    output reg         ign_cycle,         // one pulse per qualified start
    output reg  [31:0] ign_lifetime       // total starts, for the service log
);

    // RULE R1 - constant, never a port
    localparam [19:0] DEBOUNCE_CYC = `IGNITION_DEBOUNCE_CYC;  // 50 ms at 10 MHz

    reg [19:0] dbnc;
    reg        ign_prev;

    always @(posedge clk_aon or negedge rst_aon_n) begin
        if (!rst_aon_n) begin
            dbnc            <= 20'd0;
            ignition_stable <= 1'b0;
            ign_prev        <= 1'b0;
            ign_cycle       <= 1'b0;
            ign_lifetime    <= 32'd0;
        end else begin
            ign_cycle <= 1'b0;               // one-cycle strobe

            //-----------------------------------------------------------------
            // Debounce: the input must disagree with the accepted state for
            // the full window before the state moves.
            //-----------------------------------------------------------------
            if (ignition_on_s != ignition_stable) begin
                if (dbnc >= DEBOUNCE_CYC) begin
                    ignition_stable <= ignition_on_s;
                    dbnc            <= 20'd0;
                end else begin
                    dbnc <= dbnc + 20'd1;
                end
            end else begin
                dbnc <= 20'd0;
            end

            //-----------------------------------------------------------------
            // A qualified start: rising edge of the debounced key, vehicle not
            // moving.
            //-----------------------------------------------------------------
            ign_prev <= ignition_stable;

            if (ignition_stable && !ign_prev && vehicle_stationary) begin
                ign_cycle <= 1'b1;
                if (ign_lifetime != 32'hFFFF_FFFF) begin
                    ign_lifetime <= ign_lifetime + 32'd1;
                end
            end
        end
    end

endmodule

`default_nettype wire
