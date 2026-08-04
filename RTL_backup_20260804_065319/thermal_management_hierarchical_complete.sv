// thermal_management_hierarchical_complete.sv
`timescale 1ns/1ps
`include "defines_ivcu_ev_v3.sv"
`default_nettype none

module thermal_management_hierarchical_complete (
    input  wire        clk_ai,
    input  wire        rst_ai_n,
    input  wire [15:0] batt_cell_temp,
    input  wire [15:0] batt_pack_temp,
    input  wire [15:0] motor_temp,
    input  wire [15:0] inverter_temp,
    input  wire [15:0] cabin_temp,
    input  wire [15:0] coolant_flow,
    input  wire [15:0] cooling_press,
    input  wire [15:0] humidity,
    input  wire [15:0] enclosure_press,
    input  wire [8:0]  sensor_valid,
    output reg         thermal_ok,
    output reg [7:0]   thermal_score,
    output reg [3:0]   thermal_status,
    output reg         cooling_required,
    output reg [3:0]   batt_thermal_status,
    output reg [3:0]   motor_thermal_status,
    output reg [3:0]   cabin_thermal_status,
    output reg [15:0]  temp_trend,
    output reg         overheat_prediction
);

    localparam TEMP_BATT_CRIT = 16'd600;
    localparam TEMP_BATT_WARN = 16'd450;
    localparam TEMP_MOTOR_CRIT = 16'd1200;
    localparam TEMP_MOTOR_WARN = 16'd1000;
    localparam TEMP_CABIN_MAX = 16'd400;

    reg [7:0] batt_score, motor_score, cabin_score;
    reg [15:0] prev_batt_temp, prev_motor_temp;
    reg [15:0] batt_trend, motor_trend;

    always_ff @(posedge clk_ai or negedge rst_ai_n) begin
        if (!rst_ai_n) begin
            thermal_score <= 8'd100;
            thermal_ok <= 1'b1;
            thermal_status <= `STATUS_OK;
            cooling_required <= 1'b0;
            batt_thermal_status <= `STATUS_OK;
            motor_thermal_status <= `STATUS_OK;
            cabin_thermal_status <= `STATUS_OK;
            temp_trend <= 16'd0;
            overheat_prediction <= 1'b0;
            prev_batt_temp <= 16'd0;
            prev_motor_temp <= 16'd0;
            batt_trend <= 16'd0;
            motor_trend <= 16'd0;
        end else begin
            // Battery thermal score
            if (sensor_valid[0] && sensor_valid[1]) begin
                if (batt_pack_temp >= TEMP_BATT_CRIT) begin
                    batt_score <= 8'd0;
                    batt_thermal_status <= `STATUS_EMERGENCY;
                end else if (batt_pack_temp >= TEMP_BATT_WARN) begin
                    batt_score <= 8'd30;
                    batt_thermal_status <= `STATUS_CRITICAL;
                end else begin
                    batt_score <= 8'd100;
                    batt_thermal_status <= `STATUS_OK;
                end
                batt_trend <= batt_pack_temp - prev_batt_temp;
                prev_batt_temp <= batt_pack_temp;
            end

            // Motor thermal score
            if (sensor_valid[2]) begin
                if (motor_temp >= TEMP_MOTOR_CRIT) begin
                    motor_score <= 8'd0;
                    motor_thermal_status <= `STATUS_EMERGENCY;
                end else if (motor_temp >= TEMP_MOTOR_WARN) begin
                    motor_score <= 8'd30;
                    motor_thermal_status <= `STATUS_CRITICAL;
                end else begin
                    motor_score <= 8'd100;
                    motor_thermal_status <= `STATUS_OK;
                end
                motor_trend <= motor_temp - prev_motor_temp;
                prev_motor_temp <= motor_temp;
            end

            // Cabin thermal score
            if (sensor_valid[4]) begin
                if (cabin_temp >= TEMP_CABIN_MAX) begin
                    cabin_score <= 8'd40;
                    cabin_thermal_status <= `STATUS_WARNING;
                end else begin
                    cabin_score <= 8'd100;
                    cabin_thermal_status <= `STATUS_OK;
                end
            end

            // Overall thermal score (average)
            thermal_score <= (batt_score + motor_score + cabin_score) / 3;
            thermal_ok <= (thermal_score > 8'd50);
            thermal_status <= (thermal_score > 8'd70) ? `STATUS_OK :
                              (thermal_score > 8'd40) ? `STATUS_WARNING :
                              (thermal_score > 8'd20) ? `STATUS_CRITICAL : `STATUS_EMERGENCY;

            cooling_required <= (batt_pack_temp > TEMP_BATT_WARN) || (motor_temp > TEMP_MOTOR_WARN) ||
                                (inverter_temp > 16'd800);
            temp_trend <= (batt_trend + motor_trend) / 2;
            overheat_prediction <= ((batt_trend > 16'd50) && (batt_pack_temp > TEMP_BATT_WARN - 16'd100)) ||
                                   ((motor_trend > 16'd100) && (motor_temp > TEMP_MOTOR_WARN - 16'd200));
        end
    end

endmodule
`default_nettype wire