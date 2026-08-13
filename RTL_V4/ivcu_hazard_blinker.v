//=============================================================================
// ivcu_hazard_blinker.v  -  a light that actually blinks
//
// THE V3 BUG, IN ITS OWN WORDS
//
// emergency_response_system.v carried a comment admitting that hazard_timer
// was never reset.  Once the counter passed its threshold it stayed past it,
// so the comparison was true every cycle and the output toggled every cycle.
// At 10 MHz that is a 5 MHz "blink" - which to a human eye, and to a filament
// or LED with any thermal mass at all, is a lamp that is simply on.
//
// A solid hazard lamp after a crash is not a cosmetic defect.  It is the
// difference between "this vehicle has had an accident" and "this vehicle is
// parked", to every driver approaching it.
//
// THE FIX IS ONE LINE: the counter wraps.
//
//     timer <= (timer == HALF-1) ? 0 : timer + 1;
//
// which is the difference between a free-running counter and a saturating one,
// and it is worth writing down because it is the kind of thing that survives
// synthesis, STA and floorplan without complaint and only shows up when
// somebody looks at the actual vehicle.
//
//-----------------------------------------------------------------------------
// 1.5 Hz IS NOT ARBITRARY
//
// Automotive hazard and indicator flash rates are regulated - roughly 60 to
// 120 flashes per minute in most markets.  1.5 Hz is 90 per minute, in the
// middle of the band, and it is the rate drivers already read as "hazard".
//
//     HALF = 10 MHz / (2 x 1.5 Hz) = 3,333,333 clk_aon cycles
//
// RULE R5: that division is done here, by me, at design time, and what goes
// into the RTL is the constant.  There is no divider on the die.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_hazard_blinker (
    input  wire clk_aon,
    input  wire rst_aon_n,

    input  wire hazard_req,      // crash, or the driver pressed the button
    output reg  hazard_out       // to the lamp driver
);

    // 10,000,000 / (2 * 1.5) = 3,333,333.  Computed at design time, not on die.
    localparam [21:0] HALF_PERIOD = 22'd3_333_333;

    reg [21:0] timer;

    always @(posedge clk_aon or negedge rst_aon_n) begin
        if (!rst_aon_n) begin
            timer      <= 22'd0;
            hazard_out <= 1'b0;
        end else if (!hazard_req) begin
            // released: lamp off, and the phase resets so the next assertion
            // starts with a full-length ON period rather than whatever
            // fraction was left over
            timer      <= 22'd0;
            hazard_out <= 1'b0;
        end else begin
            if (timer == (HALF_PERIOD - 22'd1)) begin
                timer      <= 22'd0;          // THE FIX: it wraps
                hazard_out <= ~hazard_out;
            end else begin
                timer <= timer + 22'd1;
            end
        end
    end

endmodule

`default_nettype wire
