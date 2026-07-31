// sensor_validation_fsm.sv – proper validation FSM
`timescale 1ns/1ps
`include "defines_ivcu_ev_v3.sv"
`default_nettype none

module sensor_validation_fsm (
    input  wire         clk_sensor,
    input  wire         rst_sensor_n,
    input  wire [15:0]  sensor_data_0,  sensor_data_1,  sensor_data_2,  sensor_data_3,
    input  wire [15:0]  sensor_data_4,  sensor_data_5,  sensor_data_6,  sensor_data_7,
    input  wire [15:0]  sensor_data_8,  sensor_data_9,  sensor_data_10, sensor_data_11,
    input  wire [15:0]  sensor_data_12, sensor_data_13, sensor_data_14, sensor_data_15,
    input  wire [15:0]  sensor_data_16, sensor_data_17, sensor_data_18, sensor_data_19,
    input  wire [15:0]  sensor_data_20, sensor_data_21, sensor_data_22, sensor_data_23,
    input  wire [15:0]  sensor_data_24, sensor_data_25, sensor_data_26, sensor_data_27,
    input  wire [15:0]  sensor_data_28, sensor_data_29, sensor_data_30, sensor_data_31,
    input  wire [15:0]  sensor_data_32, sensor_data_33, sensor_data_34, sensor_data_35,
    input  wire [15:0]  sensor_data_36, sensor_data_37, sensor_data_38, sensor_data_39,
    input  wire [15:0]  sensor_data_40, sensor_data_41,
    input  wire [41:0]  sensor_data_valid,
    input  wire [41:0]  sensor_grace_active,
    input  wire [15:0]  expected_ranges_min_0, expected_ranges_max_0,
    input  wire [15:0]  expected_ranges_min_1, expected_ranges_max_1,
    input  wire [15:0]  expected_ranges_min_2, expected_ranges_max_2,
    input  wire [15:0]  expected_ranges_min_3, expected_ranges_max_3,
    input  wire [15:0]  expected_ranges_min_4, expected_ranges_max_4,
    input  wire [15:0]  expected_ranges_min_5, expected_ranges_max_5,
    input  wire [15:0]  expected_ranges_min_6, expected_ranges_max_6,
    input  wire [15:0]  expected_ranges_min_7, expected_ranges_max_7,
    input  wire [15:0]  expected_ranges_min_8, expected_ranges_max_8,
    input  wire [15:0]  expected_ranges_min_9, expected_ranges_max_9,
    input  wire [31:0]  validation_timeout,
    input  wire [15:0]  hysteresis_threshold,
    output reg  [41:0]  sensor_fault
);

    // Default min/max for sensors without dedicated inputs
    localparam [15:0] DEFAULT_MIN = 16'd0;
    localparam [15:0] DEFAULT_MAX = 16'hFFFF;

    // Helper function to get min/max for a given sensor index
    function [15:0] get_min;
        input [5:0] idx;
        begin
            case (idx)
                0: get_min = expected_ranges_min_0;
                1: get_min = expected_ranges_min_1;
                2: get_min = expected_ranges_min_2;
                3: get_min = expected_ranges_min_3;
                4: get_min = expected_ranges_min_4;
                5: get_min = expected_ranges_min_5;
                6: get_min = expected_ranges_min_6;
                7: get_min = expected_ranges_min_7;
                8: get_min = expected_ranges_min_8;
                9: get_min = expected_ranges_min_9;
                default: get_min = DEFAULT_MIN;
            endcase
        end
    endfunction

    function [15:0] get_max;
        input [5:0] idx;
        begin
            case (idx)
                0: get_max = expected_ranges_max_0;
                1: get_max = expected_ranges_max_1;
                2: get_max = expected_ranges_max_2;
                3: get_max = expected_ranges_max_3;
                4: get_max = expected_ranges_max_4;
                5: get_max = expected_ranges_max_5;
                6: get_max = expected_ranges_max_6;
                7: get_max = expected_ranges_max_7;
                8: get_max = expected_ranges_max_8;
                9: get_max = expected_ranges_max_9;
                default: get_max = DEFAULT_MAX;
            endcase
        end
    endfunction

    // Array of sensor data for easy indexing
    wire [15:0] sensor_data_array [0:41];
    assign sensor_data_array[0]  = sensor_data_0;
    assign sensor_data_array[1]  = sensor_data_1;
    assign sensor_data_array[2]  = sensor_data_2;
    assign sensor_data_array[3]  = sensor_data_3;
    assign sensor_data_array[4]  = sensor_data_4;
    assign sensor_data_array[5]  = sensor_data_5;
    assign sensor_data_array[6]  = sensor_data_6;
    assign sensor_data_array[7]  = sensor_data_7;
    assign sensor_data_array[8]  = sensor_data_8;
    assign sensor_data_array[9]  = sensor_data_9;
    assign sensor_data_array[10] = sensor_data_10;
    assign sensor_data_array[11] = sensor_data_11;
    assign sensor_data_array[12] = sensor_data_12;
    assign sensor_data_array[13] = sensor_data_13;
    assign sensor_data_array[14] = sensor_data_14;
    assign sensor_data_array[15] = sensor_data_15;
    assign sensor_data_array[16] = sensor_data_16;
    assign sensor_data_array[17] = sensor_data_17;
    assign sensor_data_array[18] = sensor_data_18;
    assign sensor_data_array[19] = sensor_data_19;
    assign sensor_data_array[20] = sensor_data_20;
    assign sensor_data_array[21] = sensor_data_21;
    assign sensor_data_array[22] = sensor_data_22;
    assign sensor_data_array[23] = sensor_data_23;
    assign sensor_data_array[24] = sensor_data_24;
    assign sensor_data_array[25] = sensor_data_25;
    assign sensor_data_array[26] = sensor_data_26;
    assign sensor_data_array[27] = sensor_data_27;
    assign sensor_data_array[28] = sensor_data_28;
    assign sensor_data_array[29] = sensor_data_29;
    assign sensor_data_array[30] = sensor_data_30;
    assign sensor_data_array[31] = sensor_data_31;
    assign sensor_data_array[32] = sensor_data_32;
    assign sensor_data_array[33] = sensor_data_33;
    assign sensor_data_array[34] = sensor_data_34;
    assign sensor_data_array[35] = sensor_data_35;
    assign sensor_data_array[36] = sensor_data_36;
    assign sensor_data_array[37] = sensor_data_37;
    assign sensor_data_array[38] = sensor_data_38;
    assign sensor_data_array[39] = sensor_data_39;
    assign sensor_data_array[40] = sensor_data_40;
    assign sensor_data_array[41] = sensor_data_41;

    integer idx;
    always @(posedge clk_sensor or negedge rst_sensor_n) begin
        if (!rst_sensor_n) begin
            sensor_fault <= 42'd0;
        end else begin
            for (idx = 0; idx < 42; idx = idx + 1) begin
                // Default: no fault
                sensor_fault[idx] <= 1'b0;
                // Check if sensor is valid and not in grace period
                if (sensor_data_valid[idx] && !sensor_grace_active[idx]) begin
                    if (sensor_data_array[idx] < get_min(idx) ||
                        sensor_data_array[idx] > get_max(idx)) begin
                        sensor_fault[idx] <= 1'b1;
                    end
                end
            end
        end
    end

endmodule
`default_nettype wire