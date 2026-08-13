//=============================================================================
// ivcu_cdc_bit_sync.v  -  single-bit clock domain crossing synchroniser
//
// Two (or more) flip-flops in the destination domain.  Use for single-bit
// level signals only.
//
// DO NOT use this on a multi-bit bus.  Synchronising each bit of a bus
// independently lets the destination sample a mixture of old and new bits and
// see a value that never existed at the source - for a health score that is a
// wrong number on a dashboard, but for a fault mask it is a safety failure.
// Buses go through ivcu_cdc_bus_sync.
//
// RULE R7: every signal crossing a clock domain in IVCU-EV V4 passes through
// this module or ivcu_cdc_bus_sync.  No exceptions, and specifically not the
// "it changes slowly so it is fine" exception.  V3 had exactly two sync_cell
// instances in the entire design, carrying two bits of mode, while sensor data
// crossed three domains raw.
//=============================================================================

`timescale 1ns / 1ps
`default_nettype none

module ivcu_cdc_bit_sync #(
    parameter integer STAGES    = 2,      // must be >= 2
    parameter         RESET_VAL = 1'b0    // value held while rst_dst_n is low
) (
    input  wire clk_dst,      // destination domain clock
    input  wire rst_dst_n,    // destination domain reset, active low
    input  wire d_src,        // signal from the source domain (async here)
    output wire q_dst         // synchronised into clk_dst
);

    reg [STAGES-1:0] meta_q;

    always @(posedge clk_dst or negedge rst_dst_n) begin
        if (!rst_dst_n) begin
            meta_q <= {STAGES{RESET_VAL}};
        end else begin
            meta_q <= {meta_q[STAGES-2:0], d_src};
        end
    end

    assign q_dst = meta_q[STAGES-1];

endmodule

`default_nettype wire
