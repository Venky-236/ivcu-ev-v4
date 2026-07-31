// sync_cell.sv - Parameterised N-bit 2-flop synchroniser
`timescale 1ns/1ps
`default_nettype none

module sync_cell #(
    parameter integer WIDTH = 1
) (
    input  wire              clk_dst,
    input  wire              rst_dst_n,
    input  wire [WIDTH-1:0]  signal_src,
    output wire [WIDTH-1:0]  signal_dst
);

    reg [WIDTH-1:0] meta, sync_r;
    assign signal_dst = sync_r;

    always @(posedge clk_dst or negedge rst_dst_n) begin
        if (!rst_dst_n) begin
            meta   <= {WIDTH{1'b0}};
            sync_r <= {WIDTH{1'b0}};
        end else begin
            meta   <= signal_src;
            sync_r <= meta;
        end
    end

endmodule
`default_nettype wire