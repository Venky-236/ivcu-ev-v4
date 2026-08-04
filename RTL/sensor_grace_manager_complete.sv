// sensor_grace_manager_complete.sv – removed unpacked array output
`timescale 1ns/1ps
`include "defines_ivcu_ev_v3.sv"
`default_nettype none

module sensor_grace_manager_complete (
    input  wire        clk_aon,
    input  wire        rst_aon_n,
    input  wire        clk_sensor,
    input  wire        rst_sensor_n,
    input  wire [41:0] sensor_fault,
    input  wire [41:0] sensor_enable,
    input  wire [41:0] sensor_data_valid,
    input  wire [2:0]  grace_period_count,
    input  wire [31:0] grace_timeout,
    output reg  [41:0] sensor_grace_active,
    output reg  [41:0] sensor_grace_expired,
    input  wire        force_disable,
    input  wire        reset_grace
);
    genvar i;
    generate
        for (i = 0; i < 42; i = i + 1) begin : grace
            reg [31:0] grace_timer;
            reg [2:0]  fault_count;

            always @(posedge clk_sensor or negedge rst_sensor_n) begin
                if (!rst_sensor_n) begin
                    grace_timer <= 32'd0;
                    fault_count <= 3'd0;
                end else begin
                    if (reset_grace) begin
                        grace_timer <= 32'd0;
                        fault_count <= 3'd0;
                    end else if (sensor_fault[i] && sensor_enable[i]) begin
                        if (grace_timer < grace_timeout) begin
                            grace_timer <= grace_timer + 1;
                            if (grace_timer == grace_timeout - 1) begin
                                if (fault_count < grace_period_count)
                                    fault_count <= fault_count + 1;
                            end
                        end
                    end else begin
                        grace_timer <= 32'd0;
                        if (!sensor_fault[i]) fault_count <= 3'd0;
                    end
                end
            end

            always @(*) begin
                sensor_grace_active[i] = (grace_timer > 0);
                sensor_grace_expired[i] = (fault_count >= grace_period_count) || force_disable;
            end
        end
    endgenerate
endmodule
`default_nettype wire