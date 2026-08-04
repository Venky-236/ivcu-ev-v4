// clock_manager_14nm.v - Safe for OpenROAD / Yosys
`timescale 1ns/1ps
`include "defines_ivcu_ev_v3.sv"   // FIX: file is defines_ivcu_ev_v3.sv, .v version never existed
`default_nettype none

module clock_manager_14nm (
    input  wire        clk_100mhz_in,
    input  wire        clk_10mhz_in,
    input  wire        clk_50mhz_sensor_in,
    input  wire        clk_200mhz_mcu_in,
    input  wire        por_n,
    input  wire        pll_bypass,
    input  wire [2:0]  clk_gate_enable,
    output wire        clk_ai_out,
    output wire        clk_aon_out,
    output wire        clk_sensor_out,
    output wire        clk_mcu_out,
    output wire        pll_locked,
    output wire [3:0]  clk_valid,
    input  wire        scan_enable,
    input  wire        test_clk
);

    // Avoid unused warnings
    wire _unused = &{por_n, pll_bypass, clk_gate_enable, scan_enable, test_clk};

    assign pll_locked = 1'b1;
    assign clk_valid  = 4'b1111;

    // Direct clock outputs – no gating, no division
    assign clk_ai_out     = clk_100mhz_in;
    assign clk_aon_out    = clk_10mhz_in;
    assign clk_sensor_out = clk_50mhz_sensor_in;
    assign clk_mcu_out    = clk_200mhz_mcu_in;

endmodule
`default_nettype wire