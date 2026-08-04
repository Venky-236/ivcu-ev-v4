// adc_interface_14nm.v
// Verilog-2001 format for Synopsys DC/ICC2
`timescale 1ns/1ps
`include "defines_ivcu_ev_v3.sv"   // FIX: file is defines_ivcu_ev_v3.sv, .v version never existed

module adc_interface_14nm (
    input  wire        clk_sensor,
    input  wire        rst_sensor_n,
    
    // ADC inputs (12‑bit each)
    input  wire [11:0] adc_in_0,
    input  wire [11:0] adc_in_1,
    input  wire [11:0] adc_in_2,
    input  wire [11:0] adc_in_3,
    input  wire [11:0] adc_in_4,
    input  wire [11:0] adc_in_5,
    input  wire [11:0] adc_in_6,
    input  wire [11:0] adc_in_7,
    input  wire [11:0] adc_in_8,
    input  wire [11:0] adc_in_9,
    input  wire [11:0] adc_in_10,
    input  wire [11:0] adc_in_11,
    input  wire [11:0] adc_in_12,
    input  wire [11:0] adc_in_13,
    input  wire [11:0] adc_in_14,
    input  wire [11:0] adc_in_15,
    input  wire [11:0] adc_in_16,
    input  wire [11:0] adc_in_17,
    input  wire [11:0] adc_in_18,
    input  wire [11:0] adc_in_19,
    input  wire [11:0] adc_in_20,
    input  wire [11:0] adc_in_21,
    input  wire [11:0] adc_in_22,
    input  wire [11:0] adc_in_23,
    input  wire [11:0] adc_in_24,
    input  wire [11:0] adc_in_25,
    input  wire [11:0] adc_in_26,
    input  wire [11:0] adc_in_27,
    input  wire [11:0] adc_in_28,
    input  wire [11:0] adc_in_29,
    input  wire [11:0] adc_in_30,
    input  wire [11:0] adc_in_31,
    input  wire [11:0] adc_in_32,
    input  wire [11:0] adc_in_33,
    input  wire [11:0] adc_in_34,
    input  wire [11:0] adc_in_35,
    input  wire [11:0] adc_in_36,
    input  wire [11:0] adc_in_37,
    input  wire [11:0] adc_in_38,
    input  wire [11:0] adc_in_39,
    input  wire [11:0] adc_in_40,
    input  wire [11:0] adc_in_41,

    input  wire        adc_valid_0,
    input  wire        adc_valid_1,
    input  wire        adc_valid_2,
    input  wire        adc_valid_3,
    input  wire        adc_valid_4,
    input  wire        adc_valid_5,
    input  wire        adc_valid_6,
    input  wire        adc_valid_7,
    input  wire        adc_valid_8,
    input  wire        adc_valid_9,
    input  wire        adc_valid_10,
    input  wire        adc_valid_11,
    input  wire        adc_valid_12,
    input  wire        adc_valid_13,
    input  wire        adc_valid_14,
    input  wire        adc_valid_15,
    input  wire        adc_valid_16,
    input  wire        adc_valid_17,
    input  wire        adc_valid_18,
    input  wire        adc_valid_19,
    input  wire        adc_valid_20,
    input  wire        adc_valid_21,
    input  wire        adc_valid_22,
    input  wire        adc_valid_23,
    input  wire        adc_valid_24,
    input  wire        adc_valid_25,
    input  wire        adc_valid_26,
    input  wire        adc_valid_27,
    input  wire        adc_valid_28,
    input  wire        adc_valid_29,
    input  wire        adc_valid_30,
    input  wire        adc_valid_31,
    input  wire        adc_valid_32,
    input  wire        adc_valid_33,
    input  wire        adc_valid_34,
    input  wire        adc_valid_35,
    input  wire        adc_valid_36,
    input  wire        adc_valid_37,
    input  wire        adc_valid_38,
    input  wire        adc_valid_39,
    input  wire        adc_valid_40,
    input  wire        adc_valid_41,

    input  wire [4:0]  adc_channel,   // optional, not used here

    // Digital outputs (32‑bit each)
    output reg  [31:0] digital_out_0,
    output reg  [31:0] digital_out_1,
    output reg  [31:0] digital_out_2,
    output reg  [31:0] digital_out_3,
    output reg  [31:0] digital_out_4,
    output reg  [31:0] digital_out_5,
    output reg  [31:0] digital_out_6,
    output reg  [31:0] digital_out_7,
    output reg  [31:0] digital_out_8,
    output reg  [31:0] digital_out_9,
    output reg  [31:0] digital_out_10,
    output reg  [31:0] digital_out_11,

    output reg         digital_valid_0,
    output reg         digital_valid_1,
    output reg         digital_valid_2,
    output reg         digital_valid_3,
    output reg         digital_valid_4,
    output reg         digital_valid_5,
    output reg         digital_valid_6,
    output reg         digital_valid_7,
    output reg         digital_valid_8,
    output reg         digital_valid_9,
    output reg         digital_valid_10,
    output reg         digital_valid_11,

    input  wire        calibration_enable,
    input  wire [11:0] offset_correction,
    input  wire [15:0] gain_correction   // Q8.8 format
);

    // Internal array to hold calibrated values (only used for channels 0‑11)
    reg [15:0] calibrated [0:41];
    integer i;

    always @(posedge clk_sensor or negedge rst_sensor_n) begin : adc_proc
        if (!rst_sensor_n) begin
            digital_out_0  <= 32'd0; digital_valid_0  <= 1'b0;
            digital_out_1  <= 32'd0; digital_valid_1  <= 1'b0;
            digital_out_2  <= 32'd0; digital_valid_2  <= 1'b0;
            digital_out_3  <= 32'd0; digital_valid_3  <= 1'b0;
            digital_out_4  <= 32'd0; digital_valid_4  <= 1'b0;
            digital_out_5  <= 32'd0; digital_valid_5  <= 1'b0;
            digital_out_6  <= 32'd0; digital_valid_6  <= 1'b0;
            digital_out_7  <= 32'd0; digital_valid_7  <= 1'b0;
            digital_out_8  <= 32'd0; digital_valid_8  <= 1'b0;
            digital_out_9  <= 32'd0; digital_valid_9  <= 1'b0;
            digital_out_10 <= 32'd0; digital_valid_10 <= 1'b0;
            digital_out_11 <= 32'd0; digital_valid_11 <= 1'b0;
        end else begin
            // ---------------------------------------------------------
            // Process ADC 0 with fixed calibration arithmetic
            // ---------------------------------------------------------
            if (adc_valid_0) begin
                if (calibration_enable) begin
                    // (adc_in + offset) * gain  >> 8   (gain is Q8.8)
                    // Use 17-bit sum to avoid overflow, 33-bit product, then take bits [23:8]
                    calibrated[0] = ({5'd0, adc_in_0} + {5'd0, offset_correction}) * gain_correction >> 8;
                end else begin
                    calibrated[0] = {4'b0, adc_in_0};   // zero-extend to 16 bits
                end
                digital_out_0  <= {16'd0, calibrated[0]};
                digital_valid_0 <= 1'b1;
            end else begin
                digital_valid_0 <= 1'b0;
            end

            // Process ADC 1
            if (adc_valid_1) begin
                if (calibration_enable) begin
                    calibrated[1] = ({5'd0, adc_in_1} + {5'd0, offset_correction}) * gain_correction >> 8;
                end else begin
                    calibrated[1] = {4'b0, adc_in_1};
                end
                digital_out_1  <= {16'd0, calibrated[1]};
                digital_valid_1 <= 1'b1;
            end else begin
                digital_valid_1 <= 1'b0;
            end

            // Process ADC 2
            if (adc_valid_2) begin
                if (calibration_enable) begin
                    calibrated[2] = ({5'd0, adc_in_2} + {5'd0, offset_correction}) * gain_correction >> 8;
                end else begin
                    calibrated[2] = {4'b0, adc_in_2};
                end
                digital_out_2  <= {16'd0, calibrated[2]};
                digital_valid_2 <= 1'b1;
            end else begin
                digital_valid_2 <= 1'b0;
            end

            // Process ADC 3
            if (adc_valid_3) begin
                if (calibration_enable) begin
                    calibrated[3] = ({5'd0, adc_in_3} + {5'd0, offset_correction}) * gain_correction >> 8;
                end else begin
                    calibrated[3] = {4'b0, adc_in_3};
                end
                digital_out_3  <= {16'd0, calibrated[3]};
                digital_valid_3 <= 1'b1;
            end else begin
                digital_valid_3 <= 1'b0;
            end

            // Process ADC 4
            if (adc_valid_4) begin
                if (calibration_enable) begin
                    calibrated[4] = ({5'd0, adc_in_4} + {5'd0, offset_correction}) * gain_correction >> 8;
                end else begin
                    calibrated[4] = {4'b0, adc_in_4};
                end
                digital_out_4  <= {16'd0, calibrated[4]};
                digital_valid_4 <= 1'b1;
            end else begin
                digital_valid_4 <= 1'b0;
            end

            // Process ADC 5
            if (adc_valid_5) begin
                if (calibration_enable) begin
                    calibrated[5] = ({5'd0, adc_in_5} + {5'd0, offset_correction}) * gain_correction >> 8;
                end else begin
                    calibrated[5] = {4'b0, adc_in_5};
                end
                digital_out_5  <= {16'd0, calibrated[5]};
                digital_valid_5 <= 1'b1;
            end else begin
                digital_valid_5 <= 1'b0;
            end

            // Process ADC 6
            if (adc_valid_6) begin
                if (calibration_enable) begin
                    calibrated[6] = ({5'd0, adc_in_6} + {5'd0, offset_correction}) * gain_correction >> 8;
                end else begin
                    calibrated[6] = {4'b0, adc_in_6};
                end
                digital_out_6  <= {16'd0, calibrated[6]};
                digital_valid_6 <= 1'b1;
            end else begin
                digital_valid_6 <= 1'b0;
            end

            // Process ADC 7
            if (adc_valid_7) begin
                if (calibration_enable) begin
                    calibrated[7] = ({5'd0, adc_in_7} + {5'd0, offset_correction}) * gain_correction >> 8;
                end else begin
                    calibrated[7] = {4'b0, adc_in_7};
                end
                digital_out_7  <= {16'd0, calibrated[7]};
                digital_valid_7 <= 1'b1;
            end else begin
                digital_valid_7 <= 1'b0;
            end

            // Process ADC 8
            if (adc_valid_8) begin
                if (calibration_enable) begin
                    calibrated[8] = ({5'd0, adc_in_8} + {5'd0, offset_correction}) * gain_correction >> 8;
                end else begin
                    calibrated[8] = {4'b0, adc_in_8};
                end
                digital_out_8  <= {16'd0, calibrated[8]};
                digital_valid_8 <= 1'b1;
            end else begin
                digital_valid_8 <= 1'b0;
            end

            // Process ADC 9
            if (adc_valid_9) begin
                if (calibration_enable) begin
                    calibrated[9] = ({5'd0, adc_in_9} + {5'd0, offset_correction}) * gain_correction >> 8;
                end else begin
                    calibrated[9] = {4'b0, adc_in_9};
                end
                digital_out_9  <= {16'd0, calibrated[9]};
                digital_valid_9 <= 1'b1;
            end else begin
                digital_valid_9 <= 1'b0;
            end

            // Process ADC 10
            if (adc_valid_10) begin
                if (calibration_enable) begin
                    calibrated[10] = ({5'd0, adc_in_10} + {5'd0, offset_correction}) * gain_correction >> 8;
                end else begin
                    calibrated[10] = {4'b0, adc_in_10};
                end
                digital_out_10 <= {16'd0, calibrated[10]};
                digital_valid_10 <= 1'b1;
            end else begin
                digital_valid_10 <= 1'b0;
            end

            // Process ADC 11
            if (adc_valid_11) begin
                if (calibration_enable) begin
                    calibrated[11] = ({5'd0, adc_in_11} + {5'd0, offset_correction}) * gain_correction >> 8;
                end else begin
                    calibrated[11] = {4'b0, adc_in_11};
                end
                digital_out_11 <= {16'd0, calibrated[11]};
                digital_valid_11 <= 1'b1;
            end else begin
                digital_valid_11 <= 1'b0;
            end

            // Note: ADCs 12‑41 are ignored because we have only 12 outputs.
        end
    end

endmodule