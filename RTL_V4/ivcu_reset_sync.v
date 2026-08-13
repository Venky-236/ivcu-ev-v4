//=============================================================================
// ivcu_reset_sync.v  -  asynchronous assert, synchronous de-assert
//
// The standard reset synchroniser.  Reset asserts the instant async_rst_n
// falls, with no clock required - which matters, because at power-on there may
// be no clock yet.  De-assertion is walked through STAGES flip-flops so that
// every register in the domain leaves reset on the same clock edge and none of
// them sees a recovery/removal violation.
//
// Verilog-2001.  No SystemVerilog, no initial block, no delays.
//=============================================================================

`timescale 1ns / 1ps
`default_nettype none

module ivcu_reset_sync #(
    // Number of synchroniser stages.  Must be >= 2.
    // 3 is used throughout IVCU-EV V4: two stages for metastability plus one
    // for margin, since reset release is a once-per-power-cycle event and the
    // extra flop costs nothing.
    parameter integer STAGES = 3
) (
    input  wire clk,            // destination domain clock
    input  wire async_rst_n,    // asynchronous, active low
    output wire sync_rst_n      // synchronised, active low
);

    reg [STAGES-1:0] sync_q;

    always @(posedge clk or negedge async_rst_n) begin
        if (!async_rst_n) begin
            sync_q <= {STAGES{1'b0}};
        end else begin
            // shift a 1 in from the bottom: the domain leaves reset STAGES
            // clocks after async_rst_n rises
            sync_q <= {sync_q[STAGES-2:0], 1'b1};
        end
    end

    assign sync_rst_n = sync_q[STAGES-1];

endmodule

`default_nettype wire
