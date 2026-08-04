// sensor_enable_logic.sv
`timescale 1ns/1ps
`include "defines_ivcu_ev_v3.sv"
`default_nettype none

module sensor_enable_logic (
    input  wire        clk_sensor,
    input  wire        rst_sensor_n,
    input  wire [1:0]  current_mode,
    input  wire        mode_valid,
    input  wire [41:0] sensor_map_car,
    input  wire [41:0] sensor_map_bike,
    input  wire [41:0] sensor_fault,
    input  wire [41:0] sensor_grace_expired,
    input  wire [41:0] sensor_force_disable,
    output reg  [41:0] sensor_enable,
    input  wire        debug_enable
);

    // FIX: 42-bit all-ones = 42'h3FFFFFFFFFF (not 42'hFFFFFFFFFFFF which is 48-bit)
    always @(posedge clk_sensor or negedge rst_sensor_n) begin
        if (!rst_sensor_n) begin
            sensor_enable <= 42'd0;
        end else begin
            if (mode_valid) begin
                case (current_mode)
                    `MODE_CAR:  sensor_enable <= sensor_map_car  & ~sensor_grace_expired & ~sensor_force_disable;
                    `MODE_BIKE: sensor_enable <= sensor_map_bike & ~sensor_grace_expired & ~sensor_force_disable;
                    default:    sensor_enable <= 42'd0;
                endcase
            end else begin
                sensor_enable <= 42'd0;
            end
            if (debug_enable) sensor_enable <= 42'h3FFFFFFFFFF;  // FIX: was 42'hFFFFFFFFFFFF (48-bit)
        end
    end

endmodule
`default_nettype wire