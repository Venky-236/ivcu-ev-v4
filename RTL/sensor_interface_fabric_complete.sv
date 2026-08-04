// sensor_interface_fabric_complete.sv
`timescale 1ns/1ps
`include "defines_ivcu_ev_v3.sv"
`default_nettype none

module sensor_interface_fabric_complete #(
    // Depth of the moving-average window. MUST be a power of two: 1,2,4,8,16.
    // This was previously a PORT tied to the constant 4'd4 at the top level.
    // Because synthesis never flattens, that constant never reached this module
    // and Yosys built 42 general-purpose dividers instead of a shift.
    parameter integer MOVING_AVG_DEPTH = 4
) (
    input  wire        clk_sensor,
    input  wire        rst_sensor_n,
    input  wire [31:0] sensor_raw_in_0,
    input  wire [31:0] sensor_raw_in_1,
    input  wire [31:0] sensor_raw_in_2,
    input  wire [31:0] sensor_raw_in_3,
    input  wire [31:0] sensor_raw_in_4,
    input  wire [31:0] sensor_raw_in_5,
    input  wire [31:0] sensor_raw_in_6,
    input  wire [31:0] sensor_raw_in_7,
    input  wire [31:0] sensor_raw_in_8,
    input  wire [31:0] sensor_raw_in_9,
    input  wire [31:0] sensor_raw_in_10,
    input  wire [31:0] sensor_raw_in_11,
    input  wire [31:0] sensor_raw_in_12,
    input  wire [31:0] sensor_raw_in_13,
    input  wire [31:0] sensor_raw_in_14,
    input  wire [31:0] sensor_raw_in_15,
    input  wire [31:0] sensor_raw_in_16,
    input  wire [31:0] sensor_raw_in_17,
    input  wire [31:0] sensor_raw_in_18,
    input  wire [31:0] sensor_raw_in_19,
    input  wire [31:0] sensor_raw_in_20,
    input  wire [31:0] sensor_raw_in_21,
    input  wire [31:0] sensor_raw_in_22,
    input  wire [31:0] sensor_raw_in_23,
    input  wire [31:0] sensor_raw_in_24,
    input  wire [31:0] sensor_raw_in_25,
    input  wire [31:0] sensor_raw_in_26,
    input  wire [31:0] sensor_raw_in_27,
    input  wire [31:0] sensor_raw_in_28,
    input  wire [31:0] sensor_raw_in_29,
    input  wire [31:0] sensor_raw_in_30,
    input  wire [31:0] sensor_raw_in_31,
    input  wire [31:0] sensor_raw_in_32,
    input  wire [31:0] sensor_raw_in_33,
    input  wire [31:0] sensor_raw_in_34,
    input  wire [31:0] sensor_raw_in_35,
    input  wire [31:0] sensor_raw_in_36,
    input  wire [31:0] sensor_raw_in_37,
    input  wire [31:0] sensor_raw_in_38,
    input  wire [31:0] sensor_raw_in_39,
    input  wire [31:0] sensor_raw_in_40,
    input  wire [31:0] sensor_raw_in_41,
    input  wire [41:0] sensor_raw_valid,
    input  wire [41:0] sensor_enable,
    output reg  [15:0] sensor_data_out_0,
    output reg  [15:0] sensor_data_out_1,
    output reg  [15:0] sensor_data_out_2,
    output reg  [15:0] sensor_data_out_3,
    output reg  [15:0] sensor_data_out_4,
    output reg  [15:0] sensor_data_out_5,
    output reg  [15:0] sensor_data_out_6,
    output reg  [15:0] sensor_data_out_7,
    output reg  [15:0] sensor_data_out_8,
    output reg  [15:0] sensor_data_out_9,
    output reg  [15:0] sensor_data_out_10,
    output reg  [15:0] sensor_data_out_11,
    output reg  [15:0] sensor_data_out_12,
    output reg  [15:0] sensor_data_out_13,
    output reg  [15:0] sensor_data_out_14,
    output reg  [15:0] sensor_data_out_15,
    output reg  [15:0] sensor_data_out_16,
    output reg  [15:0] sensor_data_out_17,
    output reg  [15:0] sensor_data_out_18,
    output reg  [15:0] sensor_data_out_19,
    output reg  [15:0] sensor_data_out_20,
    output reg  [15:0] sensor_data_out_21,
    output reg  [15:0] sensor_data_out_22,
    output reg  [15:0] sensor_data_out_23,
    output reg  [15:0] sensor_data_out_24,
    output reg  [15:0] sensor_data_out_25,
    output reg  [15:0] sensor_data_out_26,
    output reg  [15:0] sensor_data_out_27,
    output reg  [15:0] sensor_data_out_28,
    output reg  [15:0] sensor_data_out_29,
    output reg  [15:0] sensor_data_out_30,
    output reg  [15:0] sensor_data_out_31,
    output reg  [15:0] sensor_data_out_32,
    output reg  [15:0] sensor_data_out_33,
    output reg  [15:0] sensor_data_out_34,
    output reg  [15:0] sensor_data_out_35,
    output reg  [15:0] sensor_data_out_36,
    output reg  [15:0] sensor_data_out_37,
    output reg  [15:0] sensor_data_out_38,
    output reg  [15:0] sensor_data_out_39,
    output reg  [15:0] sensor_data_out_40,
    output reg  [15:0] sensor_data_out_41,
    output reg  [41:0] sensor_data_valid,
    output reg  [41:0] sensor_filtered_valid,
    output reg  [7:0]  sensor_accuracy_0,
    output reg  [7:0]  sensor_accuracy_1,
    output reg  [7:0]  sensor_accuracy_2,
    output reg  [7:0]  sensor_accuracy_3,
    output reg  [7:0]  sensor_accuracy_4,
    output reg  [7:0]  sensor_accuracy_5,
    output reg  [7:0]  sensor_accuracy_6,
    output reg  [7:0]  sensor_accuracy_7,
    output reg  [7:0]  sensor_accuracy_8,
    output reg  [7:0]  sensor_accuracy_9,
    output reg  [7:0]  sensor_accuracy_10,
    output reg  [7:0]  sensor_accuracy_11,
    output reg  [7:0]  sensor_accuracy_12,
    output reg  [7:0]  sensor_accuracy_13,
    output reg  [7:0]  sensor_accuracy_14,
    output reg  [7:0]  sensor_accuracy_15,
    output reg  [7:0]  sensor_accuracy_16,
    output reg  [7:0]  sensor_accuracy_17,
    output reg  [7:0]  sensor_accuracy_18,
    output reg  [7:0]  sensor_accuracy_19,
    output reg  [7:0]  sensor_accuracy_20,
    output reg  [7:0]  sensor_accuracy_21,
    output reg  [7:0]  sensor_accuracy_22,
    output reg  [7:0]  sensor_accuracy_23,
    output reg  [7:0]  sensor_accuracy_24,
    output reg  [7:0]  sensor_accuracy_25,
    output reg  [7:0]  sensor_accuracy_26,
    output reg  [7:0]  sensor_accuracy_27,
    output reg  [7:0]  sensor_accuracy_28,
    output reg  [7:0]  sensor_accuracy_29,
    output reg  [7:0]  sensor_accuracy_30,
    output reg  [7:0]  sensor_accuracy_31,
    output reg  [7:0]  sensor_accuracy_32,
    output reg  [7:0]  sensor_accuracy_33,
    output reg  [7:0]  sensor_accuracy_34,
    output reg  [7:0]  sensor_accuracy_35,
    output reg  [7:0]  sensor_accuracy_36,
    output reg  [7:0]  sensor_accuracy_37,
    output reg  [7:0]  sensor_accuracy_38,
    output reg  [7:0]  sensor_accuracy_39,
    output reg  [7:0]  sensor_accuracy_40,
    output reg  [7:0]  sensor_accuracy_41,
    output wire [41:0] sensor_calibrated,
    input  wire [15:0] filter_coefficients,
    input  wire [15:0] deadband_threshold,
    input  wire [2:0]  debug_mode
);

    // log2 of the window depth -- the shift that replaces the division.
    localparam integer AVG_SHIFT = (MOVING_AVG_DEPTH <= 1) ? 0 :
                                   (MOVING_AVG_DEPTH <= 2) ? 1 :
                                   (MOVING_AVG_DEPTH <= 4) ? 2 :
                                   (MOVING_AVG_DEPTH <= 8) ? 3 : 4;

    // Depth-1 is an all-ones mask because the depth is a power of two, so the
    // ring-buffer pointer wrap becomes an AND instead of a compare-and-select.
    localparam [3:0] AVG_MASK = MOVING_AVG_DEPTH - 1;

    // Internal arrays for channel processing
    wire [15:0] filtered_data [0:41];   // FIXED: was reg
    wire [7:0]  acc [0:41];            // FIXED: was reg
    wire [41:0] valid_int;             // FIXED: was reg
    wire [41:0] filtered_valid_int;    // FIXED: was reg

    // Raw input array
    wire [31:0] raw_array [0:41];
    assign raw_array[0]  = sensor_raw_in_0;
    assign raw_array[1]  = sensor_raw_in_1;
    assign raw_array[2]  = sensor_raw_in_2;
    assign raw_array[3]  = sensor_raw_in_3;
    assign raw_array[4]  = sensor_raw_in_4;
    assign raw_array[5]  = sensor_raw_in_5;
    assign raw_array[6]  = sensor_raw_in_6;
    assign raw_array[7]  = sensor_raw_in_7;
    assign raw_array[8]  = sensor_raw_in_8;
    assign raw_array[9]  = sensor_raw_in_9;
    assign raw_array[10] = sensor_raw_in_10;
    assign raw_array[11] = sensor_raw_in_11;
    assign raw_array[12] = sensor_raw_in_12;
    assign raw_array[13] = sensor_raw_in_13;
    assign raw_array[14] = sensor_raw_in_14;
    assign raw_array[15] = sensor_raw_in_15;
    assign raw_array[16] = sensor_raw_in_16;
    assign raw_array[17] = sensor_raw_in_17;
    assign raw_array[18] = sensor_raw_in_18;
    assign raw_array[19] = sensor_raw_in_19;
    assign raw_array[20] = sensor_raw_in_20;
    assign raw_array[21] = sensor_raw_in_21;
    assign raw_array[22] = sensor_raw_in_22;
    assign raw_array[23] = sensor_raw_in_23;
    assign raw_array[24] = sensor_raw_in_24;
    assign raw_array[25] = sensor_raw_in_25;
    assign raw_array[26] = sensor_raw_in_26;
    assign raw_array[27] = sensor_raw_in_27;
    assign raw_array[28] = sensor_raw_in_28;
    assign raw_array[29] = sensor_raw_in_29;
    assign raw_array[30] = sensor_raw_in_30;
    assign raw_array[31] = sensor_raw_in_31;
    assign raw_array[32] = sensor_raw_in_32;
    assign raw_array[33] = sensor_raw_in_33;
    assign raw_array[34] = sensor_raw_in_34;
    assign raw_array[35] = sensor_raw_in_35;
    assign raw_array[36] = sensor_raw_in_36;
    assign raw_array[37] = sensor_raw_in_37;
    assign raw_array[38] = sensor_raw_in_38;
    assign raw_array[39] = sensor_raw_in_39;
    assign raw_array[40] = sensor_raw_in_40;
    assign raw_array[41] = sensor_raw_in_41;

    generate
        for (genvar i = 0; i < 42; i++) begin : chan
            // Moving average filter
            reg [15:0] mem [0:15];
            reg [3:0]  ptr;
            reg [31:0] sum;
            reg [15:0] data_raw;
            reg [15:0] data_filt;
            reg [7:0]  accuracy;

            // Moving average = sum >> log2(depth), written as a bit-select.
            // Wiring, not logic: zero gates. Replaces the 32-bit variable
            // divider that measured ~130 levels and 39.2 ns.
            wire [15:0] avg_now = sum[AVG_SHIFT +: 16];

            always @(posedge clk_sensor or negedge rst_sensor_n) begin
                if (!rst_sensor_n) begin
                    for (int j = 0; j < 16; j++) mem[j] <= 16'd0;
                    ptr <= 4'd0;
                    sum <= 32'd0;
                    data_raw <= 16'd0;
                    data_filt <= 16'd0;
                    accuracy <= 8'd0;
                end else if (sensor_raw_valid[i] && sensor_enable[i]) begin
                    data_raw <= raw_array[i][15:0];
                    sum      <= sum - mem[ptr] + raw_array[i][15:0];
                    mem[ptr] <= raw_array[i][15:0];

                    // Power-of-two depth, so the wrap is a mask, not a compare.
                    ptr      <= (ptr + 4'd1) & AVG_MASK;

                    // FUNCTIONAL FIX.
                    // The old code assigned data_filt TWICE with non-blocking
                    // assignments in the same block. The second won, so the
                    // average was computed then silently discarded, and the
                    // threshold was subtracted from the PREVIOUS cycle's output.
                    // Whenever the deadband was active the output walked itself
                    // downward with the sensor data playing no part at all.
                    // Now: one branch, one assignment, deadband applied to the
                    // average computed this cycle.
                    data_filt <= (avg_now > deadband_threshold)
                                 ? (avg_now - deadband_threshold)
                                 : avg_now;

                    accuracy  <= 8'd90;
                end else begin
                    data_filt <= 16'd0;
                    accuracy <= 8'd0;
                end
            end

            assign filtered_data[i] = data_filt;
            assign acc[i] = accuracy;
            assign valid_int[i] = sensor_raw_valid[i] & sensor_enable[i];
            assign filtered_valid_int[i] = valid_int[i];
        end
    endgenerate

    // Drive outputs
    always @(posedge clk_sensor) begin
        sensor_data_out_0  <= filtered_data[0];
        sensor_data_out_1  <= filtered_data[1];
        sensor_data_out_2  <= filtered_data[2];
        sensor_data_out_3  <= filtered_data[3];
        sensor_data_out_4  <= filtered_data[4];
        sensor_data_out_5  <= filtered_data[5];
        sensor_data_out_6  <= filtered_data[6];
        sensor_data_out_7  <= filtered_data[7];
        sensor_data_out_8  <= filtered_data[8];
        sensor_data_out_9  <= filtered_data[9];
        sensor_data_out_10 <= filtered_data[10];
        sensor_data_out_11 <= filtered_data[11];
        sensor_data_out_12 <= filtered_data[12];
        sensor_data_out_13 <= filtered_data[13];
        sensor_data_out_14 <= filtered_data[14];
        sensor_data_out_15 <= filtered_data[15];
        sensor_data_out_16 <= filtered_data[16];
        sensor_data_out_17 <= filtered_data[17];
        sensor_data_out_18 <= filtered_data[18];
        sensor_data_out_19 <= filtered_data[19];
        sensor_data_out_20 <= filtered_data[20];
        sensor_data_out_21 <= filtered_data[21];
        sensor_data_out_22 <= filtered_data[22];
        sensor_data_out_23 <= filtered_data[23];
        sensor_data_out_24 <= filtered_data[24];
        sensor_data_out_25 <= filtered_data[25];
        sensor_data_out_26 <= filtered_data[26];
        sensor_data_out_27 <= filtered_data[27];
        sensor_data_out_28 <= filtered_data[28];
        sensor_data_out_29 <= filtered_data[29];
        sensor_data_out_30 <= filtered_data[30];
        sensor_data_out_31 <= filtered_data[31];
        sensor_data_out_32 <= filtered_data[32];
        sensor_data_out_33 <= filtered_data[33];
        sensor_data_out_34 <= filtered_data[34];
        sensor_data_out_35 <= filtered_data[35];
        sensor_data_out_36 <= filtered_data[36];
        sensor_data_out_37 <= filtered_data[37];
        sensor_data_out_38 <= filtered_data[38];
        sensor_data_out_39 <= filtered_data[39];
        sensor_data_out_40 <= filtered_data[40];
        sensor_data_out_41 <= filtered_data[41];

        sensor_accuracy_0  <= acc[0];
        sensor_accuracy_1  <= acc[1];
        sensor_accuracy_2  <= acc[2];
        sensor_accuracy_3  <= acc[3];
        sensor_accuracy_4  <= acc[4];
        sensor_accuracy_5  <= acc[5];
        sensor_accuracy_6  <= acc[6];
        sensor_accuracy_7  <= acc[7];
        sensor_accuracy_8  <= acc[8];
        sensor_accuracy_9  <= acc[9];
        sensor_accuracy_10 <= acc[10];
        sensor_accuracy_11 <= acc[11];
        sensor_accuracy_12 <= acc[12];
        sensor_accuracy_13 <= acc[13];
        sensor_accuracy_14 <= acc[14];
        sensor_accuracy_15 <= acc[15];
        sensor_accuracy_16 <= acc[16];
        sensor_accuracy_17 <= acc[17];
        sensor_accuracy_18 <= acc[18];
        sensor_accuracy_19 <= acc[19];
        sensor_accuracy_20 <= acc[20];
        sensor_accuracy_21 <= acc[21];
        sensor_accuracy_22 <= acc[22];
        sensor_accuracy_23 <= acc[23];
        sensor_accuracy_24 <= acc[24];
        sensor_accuracy_25 <= acc[25];
        sensor_accuracy_26 <= acc[26];
        sensor_accuracy_27 <= acc[27];
        sensor_accuracy_28 <= acc[28];
        sensor_accuracy_29 <= acc[29];
        sensor_accuracy_30 <= acc[30];
        sensor_accuracy_31 <= acc[31];
        sensor_accuracy_32 <= acc[32];
        sensor_accuracy_33 <= acc[33];
        sensor_accuracy_34 <= acc[34];
        sensor_accuracy_35 <= acc[35];
        sensor_accuracy_36 <= acc[36];
        sensor_accuracy_37 <= acc[37];
        sensor_accuracy_38 <= acc[38];
        sensor_accuracy_39 <= acc[39];
        sensor_accuracy_40 <= acc[40];
        sensor_accuracy_41 <= acc[41];

        sensor_data_valid <= valid_int;
        sensor_filtered_valid <= filtered_valid_int;
    end

    assign sensor_calibrated = 42'h3FFFFFFFFFF;   // FIXED: was reg, now wire

endmodule
`default_nettype wire