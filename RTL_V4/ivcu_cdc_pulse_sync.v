//=============================================================================
// ivcu_cdc_pulse_sync.v  -  a one-cycle pulse, across a clock domain
//
// A level synchroniser cannot carry a pulse.  sweep_done is one clk_sensor
// cycle wide - 40 ns - and clk_aon has a 100 ns period, so a two-flop level
// synchroniser samples it only if the sampling edge happens to land inside
// that 40 ns window.  Most of the time it does not, and the pulse is simply
// lost.  Nothing reports an error; the destination just never updates.
//
// This is the classic fast-to-slow crossing mistake, and it is silent, which
// is what makes it worth its own module rather than a two-flop chain written
// inline and assumed to work.
//
// The fix is to convert the pulse to a level change at the source - a toggle -
// carry the toggle across, and detect its edge at the destination.  A toggle
// is stable until the next pulse, so any sampling edge sees it.
//
// REQUIREMENT: source pulses must be at least three destination clocks apart.
// Every use in IVCU-EV V4 satisfies this with enormous margin - the fastest is
// sweep_done at one pulse per 10.24 us into a 10 MHz domain, about 100
// destination clocks between pulses.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

module ivcu_cdc_pulse_sync (
    //--- source domain ------------------------------------------------------
    input  wire clk_src,
    input  wire rst_src_n,
    input  wire pulse_src,

    //--- destination domain -------------------------------------------------
    input  wire clk_dst,
    input  wire rst_dst_n,
    output reg  pulse_dst
);

    reg toggle_q;

    always @(posedge clk_src or negedge rst_src_n) begin
        if (!rst_src_n) begin
            toggle_q <= 1'b0;
        end else if (pulse_src) begin
            toggle_q <= ~toggle_q;
        end
    end

    reg [2:0] tsync_q;

    always @(posedge clk_dst or negedge rst_dst_n) begin
        if (!rst_dst_n) begin
            tsync_q   <= 3'b000;
            pulse_dst <= 1'b0;
        end else begin
            tsync_q   <= {tsync_q[1:0], toggle_q};
            pulse_dst <= tsync_q[2] ^ tsync_q[1];
        end
    end

endmodule

`default_nettype wire
