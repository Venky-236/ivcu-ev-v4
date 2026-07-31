// system_health_ai_complete.sv
`timescale 1ns/1ps
`include "defines_ivcu_ev_v3.sv"
`default_nettype none

module system_health_ai_complete (
    input  wire        clk_ai,
    input  wire        rst_ai_n,
    input  wire [7:0]  battery_score,
    input  wire [7:0]  thermal_score,
    input  wire [7:0]  motor_score,
    input  wire [7:0]  dynamics_score,
    input  wire [7:0]  perception_score,
    input  wire [3:0]  battery_status,
    input  wire [3:0]  thermal_status,
    input  wire [3:0]  motor_status,
    input  wire [3:0]  dynamics_status,
    input  wire [3:0]  perception_status,
    input  wire [1:0]  current_mode,
    input  wire [7:0]  sensor_fault_count,
    output reg  [7:0]  system_health_score,
    output reg  [3:0]  overall_status,
    output reg         maintenance_required,
    output reg  [7:0]  predicted_failures
);

    reg [7:0] weighted_score;
    reg [3:0] worst_status;

    always_ff @(posedge clk_ai or negedge rst_ai_n) begin
        if (!rst_ai_n) begin
            system_health_score <= 8'd100;
            overall_status <= `STATUS_OK;
            maintenance_required <= 1'b0;
            predicted_failures <= 8'd0;
        end else begin
            // Weighted average (battery and motor have higher weight)
            weighted_score <= (battery_score * 3 + motor_score * 3 + thermal_score * 2 +
                               dynamics_score * 1 + perception_score * 1) / 10;
            system_health_score <= weighted_score;

            // Determine worst status
            worst_status = battery_status;
            if (motor_status > worst_status) worst_status = motor_status;
            if (thermal_status > worst_status) worst_status = thermal_status;
            if (dynamics_status > worst_status) worst_status = dynamics_status;
            if (perception_status > worst_status) worst_status = perception_status;
            overall_status <= worst_status;

            maintenance_required <= (weighted_score < 8'd60) || (sensor_fault_count > 8'd10);
            predicted_failures <= (100 - weighted_score) + sensor_fault_count;
        end
    end

endmodule
`default_nettype wire