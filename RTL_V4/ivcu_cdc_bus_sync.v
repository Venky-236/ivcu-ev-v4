//=============================================================================
// ivcu_cdc_bus_sync.v  -  multi-bit clock domain crossing, valid-toggle
//                         handshake
//
// HOW IT WORKS
//   Source domain : when d_src_valid pulses, the data is captured into a
//                   holding register and a toggle flag flips.
//   Destination   : the toggle flag is synchronised through two flops, the
//                   edge is detected, and the holding register is sampled.
//
//   The holding register is stable for many destination clocks either side of
//   the sampling edge, so the destination never sees a torn value.  Only one
//   bit - the toggle - actually crosses the domain boundary asynchronously.
//
// REQUIREMENT ON THE SOURCE
//   d_src_valid pulses must be at least 4 destination clocks apart.  Every use
//   in IVCU-EV V4 satisfies this by a wide margin: the fastest producer is the
//   AI cluster emitting a health score once per 10.24 us sensor sweep, into a
//   destination running at 10 MHz.  There is no back-pressure mechanism here
//   on purpose - adding one would put a return path across the boundary for a
//   condition that cannot occur.
//
// SDC REQUIREMENT
//   The hold_q -> q_dst path is a genuine asynchronous crossing whose timing is
//   guaranteed by the handshake, not by the clock relationship.  It must be
//   excluded from analysis:
//       set_false_path -through [get_pins */u_*cdc*/hold_q_reg*/Q]
//   set_clock_groups -asynchronous in the SDC already covers this, but the
//   dedicated line is kept for clarity.
//=============================================================================

`timescale 1ns / 1ps
`default_nettype none

module ivcu_cdc_bus_sync #(
    parameter integer W = 8       // bus width
) (
    // --- source domain -------------------------------------------------------
    input  wire         clk_src,
    input  wire         rst_src_n,
    input  wire [W-1:0] d_src,
    input  wire         d_src_valid,    // one-cycle pulse in clk_src

    // --- destination domain --------------------------------------------------
    input  wire         clk_dst,
    input  wire         rst_dst_n,
    output reg  [W-1:0] q_dst,
    output reg          q_dst_valid     // one-cycle pulse in clk_dst
);

    //-------------------------------------------------------------------------
    // Source side: capture and flip the toggle
    //-------------------------------------------------------------------------
    reg [W-1:0] hold_q;
    reg         toggle_q;

    always @(posedge clk_src or negedge rst_src_n) begin
        if (!rst_src_n) begin
            hold_q   <= {W{1'b0}};
            toggle_q <= 1'b0;
        end else if (d_src_valid) begin
            hold_q   <= d_src;
            toggle_q <= ~toggle_q;
        end
    end

    //-------------------------------------------------------------------------
    // Destination side: synchronise the toggle, detect its edge
    //
    // Three flops: [0] and [1] resolve metastability, [2] holds the previous
    // value so [2]^[1] is a clean one-cycle pulse on either edge of the toggle.
    //-------------------------------------------------------------------------
    reg  [2:0] tsync_q;
    wire       take;

    always @(posedge clk_dst or negedge rst_dst_n) begin
        if (!rst_dst_n) begin
            tsync_q <= 3'b000;
        end else begin
            tsync_q <= {tsync_q[1:0], toggle_q};
        end
    end

    assign take = tsync_q[2] ^ tsync_q[1];

    always @(posedge clk_dst or negedge rst_dst_n) begin
        if (!rst_dst_n) begin
            q_dst       <= {W{1'b0}};
            q_dst_valid <= 1'b0;
        end else begin
            q_dst_valid <= take;
            if (take) begin
                q_dst <= hold_q;
            end
        end
    end

endmodule

`default_nettype wire
