// ivcu_ev_v3_hybrid_top.sv
// Fixed:
//   1. sync_cell #(.WIDTH(2)) now works because sync_cell.sv is parameterised.
//   2. system_status[7:4] = 4'b0; (procedural assign to wire) replaced with
//      a proper 8-bit wire driven by concat of module output + 4'b0.
//   3. Removed unused wire sensor_grace_count.
`timescale 1ns/1ps
`include "defines_ivcu_ev_v3.sv"
`default_nettype none

module ivcu_ev_v3_hybrid_top (
    // Global Clock and Reset
    input  wire        clk_100mhz,
    input  wire        clk_10mhz_aon,
    input  wire        clk_50mhz_sensor,
    input  wire        clk_200mhz_mcu,
    input  wire        ext_rst_n,
    input  wire        por_n,

    // Power Management
    input  wire        vdd_core,
    input  wire        vdd_io,
    input  wire        vdd_ram,
    input  wire        pwr_good,
    output wire        pwr_en_ai,
    output wire        pwr_en_sensor,
    output wire        pwr_en_mcu,
    output wire [1:0]  power_state,

    // Mode Selection
    input  wire        mode_switch_car,
    input  wire        mode_switch_bike,
    input  wire        mode_auto_detect,
    input  wire [1:0]  user_mode_override,
    output wire [1:0]  active_mode,
    output wire        mode_change_ack,

    // Sensor Inputs (Analog) - 42 channels
    input  wire [11:0] sensor_adc_in_0,
    input  wire [11:0] sensor_adc_in_1,
    input  wire [11:0] sensor_adc_in_2,
    input  wire [11:0] sensor_adc_in_3,
    input  wire [11:0] sensor_adc_in_4,
    input  wire [11:0] sensor_adc_in_5,
    input  wire [11:0] sensor_adc_in_6,
    input  wire [11:0] sensor_adc_in_7,
    input  wire [11:0] sensor_adc_in_8,
    input  wire [11:0] sensor_adc_in_9,
    input  wire [11:0] sensor_adc_in_10,
    input  wire [11:0] sensor_adc_in_11,
    input  wire [11:0] sensor_adc_in_12,
    input  wire [11:0] sensor_adc_in_13,
    input  wire [11:0] sensor_adc_in_14,
    input  wire [11:0] sensor_adc_in_15,
    input  wire [11:0] sensor_adc_in_16,
    input  wire [11:0] sensor_adc_in_17,
    input  wire [11:0] sensor_adc_in_18,
    input  wire [11:0] sensor_adc_in_19,
    input  wire [11:0] sensor_adc_in_20,
    input  wire [11:0] sensor_adc_in_21,
    input  wire [11:0] sensor_adc_in_22,
    input  wire [11:0] sensor_adc_in_23,
    input  wire [11:0] sensor_adc_in_24,
    input  wire [11:0] sensor_adc_in_25,
    input  wire [11:0] sensor_adc_in_26,
    input  wire [11:0] sensor_adc_in_27,
    input  wire [11:0] sensor_adc_in_28,
    input  wire [11:0] sensor_adc_in_29,
    input  wire [11:0] sensor_adc_in_30,
    input  wire [11:0] sensor_adc_in_31,
    input  wire [11:0] sensor_adc_in_32,
    input  wire [11:0] sensor_adc_in_33,
    input  wire [11:0] sensor_adc_in_34,
    input  wire [11:0] sensor_adc_in_35,
    input  wire [11:0] sensor_adc_in_36,
    input  wire [11:0] sensor_adc_in_37,
    input  wire [11:0] sensor_adc_in_38,
    input  wire [11:0] sensor_adc_in_39,
    input  wire [11:0] sensor_adc_in_40,
    input  wire [11:0] sensor_adc_in_41,

    input  wire        sensor_adc_valid_0,
    input  wire        sensor_adc_valid_1,
    input  wire        sensor_adc_valid_2,
    input  wire        sensor_adc_valid_3,
    input  wire        sensor_adc_valid_4,
    input  wire        sensor_adc_valid_5,
    input  wire        sensor_adc_valid_6,
    input  wire        sensor_adc_valid_7,
    input  wire        sensor_adc_valid_8,
    input  wire        sensor_adc_valid_9,
    input  wire        sensor_adc_valid_10,
    input  wire        sensor_adc_valid_11,
    input  wire        sensor_adc_valid_12,
    input  wire        sensor_adc_valid_13,
    input  wire        sensor_adc_valid_14,
    input  wire        sensor_adc_valid_15,
    input  wire        sensor_adc_valid_16,
    input  wire        sensor_adc_valid_17,
    input  wire        sensor_adc_valid_18,
    input  wire        sensor_adc_valid_19,
    input  wire        sensor_adc_valid_20,
    input  wire        sensor_adc_valid_21,
    input  wire        sensor_adc_valid_22,
    input  wire        sensor_adc_valid_23,
    input  wire        sensor_adc_valid_24,
    input  wire        sensor_adc_valid_25,
    input  wire        sensor_adc_valid_26,
    input  wire        sensor_adc_valid_27,
    input  wire        sensor_adc_valid_28,
    input  wire        sensor_adc_valid_29,
    input  wire        sensor_adc_valid_30,
    input  wire        sensor_adc_valid_31,
    input  wire        sensor_adc_valid_32,
    input  wire        sensor_adc_valid_33,
    input  wire        sensor_adc_valid_34,
    input  wire        sensor_adc_valid_35,
    input  wire        sensor_adc_valid_36,
    input  wire        sensor_adc_valid_37,
    input  wire        sensor_adc_valid_38,
    input  wire        sensor_adc_valid_39,
    input  wire        sensor_adc_valid_40,
    input  wire        sensor_adc_valid_41,

    input  wire [4:0]  sensor_adc_channel,

    // Sensor Inputs (Digital) - 42 channels
    input  wire [31:0] sensor_digital_in_0,
    input  wire [31:0] sensor_digital_in_1,
    input  wire [31:0] sensor_digital_in_2,
    input  wire [31:0] sensor_digital_in_3,
    input  wire [31:0] sensor_digital_in_4,
    input  wire [31:0] sensor_digital_in_5,
    input  wire [31:0] sensor_digital_in_6,
    input  wire [31:0] sensor_digital_in_7,
    input  wire [31:0] sensor_digital_in_8,
    input  wire [31:0] sensor_digital_in_9,
    input  wire [31:0] sensor_digital_in_10,
    input  wire [31:0] sensor_digital_in_11,
    input  wire [31:0] sensor_digital_in_12,
    input  wire [31:0] sensor_digital_in_13,
    input  wire [31:0] sensor_digital_in_14,
    input  wire [31:0] sensor_digital_in_15,
    input  wire [31:0] sensor_digital_in_16,
    input  wire [31:0] sensor_digital_in_17,
    input  wire [31:0] sensor_digital_in_18,
    input  wire [31:0] sensor_digital_in_19,
    input  wire [31:0] sensor_digital_in_20,
    input  wire [31:0] sensor_digital_in_21,
    input  wire [31:0] sensor_digital_in_22,
    input  wire [31:0] sensor_digital_in_23,
    input  wire [31:0] sensor_digital_in_24,
    input  wire [31:0] sensor_digital_in_25,
    input  wire [31:0] sensor_digital_in_26,
    input  wire [31:0] sensor_digital_in_27,
    input  wire [31:0] sensor_digital_in_28,
    input  wire [31:0] sensor_digital_in_29,
    input  wire [31:0] sensor_digital_in_30,
    input  wire [31:0] sensor_digital_in_31,
    input  wire [31:0] sensor_digital_in_32,
    input  wire [31:0] sensor_digital_in_33,
    input  wire [31:0] sensor_digital_in_34,
    input  wire [31:0] sensor_digital_in_35,
    input  wire [31:0] sensor_digital_in_36,
    input  wire [31:0] sensor_digital_in_37,
    input  wire [31:0] sensor_digital_in_38,
    input  wire [31:0] sensor_digital_in_39,
    input  wire [31:0] sensor_digital_in_40,
    input  wire [31:0] sensor_digital_in_41,

    input  wire        sensor_digital_valid_0,
    input  wire        sensor_digital_valid_1,
    input  wire        sensor_digital_valid_2,
    input  wire        sensor_digital_valid_3,
    input  wire        sensor_digital_valid_4,
    input  wire        sensor_digital_valid_5,
    input  wire        sensor_digital_valid_6,
    input  wire        sensor_digital_valid_7,
    input  wire        sensor_digital_valid_8,
    input  wire        sensor_digital_valid_9,
    input  wire        sensor_digital_valid_10,
    input  wire        sensor_digital_valid_11,
    input  wire        sensor_digital_valid_12,
    input  wire        sensor_digital_valid_13,
    input  wire        sensor_digital_valid_14,
    input  wire        sensor_digital_valid_15,
    input  wire        sensor_digital_valid_16,
    input  wire        sensor_digital_valid_17,
    input  wire        sensor_digital_valid_18,
    input  wire        sensor_digital_valid_19,
    input  wire        sensor_digital_valid_20,
    input  wire        sensor_digital_valid_21,
    input  wire        sensor_digital_valid_22,
    input  wire        sensor_digital_valid_23,
    input  wire        sensor_digital_valid_24,
    input  wire        sensor_digital_valid_25,
    input  wire        sensor_digital_valid_26,
    input  wire        sensor_digital_valid_27,
    input  wire        sensor_digital_valid_28,
    input  wire        sensor_digital_valid_29,
    input  wire        sensor_digital_valid_30,
    input  wire        sensor_digital_valid_31,
    input  wire        sensor_digital_valid_32,
    input  wire        sensor_digital_valid_33,
    input  wire        sensor_digital_valid_34,
    input  wire        sensor_digital_valid_35,
    input  wire        sensor_digital_valid_36,
    input  wire        sensor_digital_valid_37,
    input  wire        sensor_digital_valid_38,
    input  wire        sensor_digital_valid_39,
    input  wire        sensor_digital_valid_40,
    input  wire        sensor_digital_valid_41,

    // Sensor Status Outputs
    output wire [41:0] sensor_enable,
    output wire [41:0] sensor_grace_active,
    output wire [41:0] sensor_fault,
    output wire [41:0] sensor_valid_out,

    // AI Processing Interface
    output wire        ai_processing_active,
    output wire [7:0]  system_health_score,
    output wire [3:0]  battery_health_status,
    output wire [3:0]  motor_health_status,
    output wire [3:0]  thermal_health_status,
    output wire [3:0]  safety_health_status,

    // MCU Interface (AXI-Lite)
    output wire        m_axi_awvalid,
    input  wire        m_axi_awready,
    output wire [31:0] m_axi_awaddr,
    output wire [2:0]  m_axi_awprot,
    output wire        m_axi_wvalid,
    input  wire        m_axi_wready,
    output wire [63:0] m_axi_wdata,
    output wire [7:0]  m_axi_wstrb,
    input  wire        m_axi_bvalid,
    output wire        m_axi_bready,
    input  wire [1:0]  m_axi_bresp,
    output wire        m_axi_arvalid,
    input  wire        m_axi_arready,
    output wire [31:0] m_axi_araddr,
    output wire [2:0]  m_axi_arprot,
    input  wire        m_axi_rvalid,
    output wire        m_axi_rready,
    input  wire [63:0] m_axi_rdata,
    input  wire [1:0]  m_axi_rresp,

    // Slave Interface (AXI-Lite)
    input  wire        s_axi_awvalid,
    output wire        s_axi_awready,
    input  wire [31:0] s_axi_awaddr,
    input  wire [2:0]  s_axi_awprot,
    input  wire        s_axi_wvalid,
    output wire        s_axi_wready,
    input  wire [63:0] s_axi_wdata,
    input  wire [7:0]  s_axi_wstrb,
    output wire        s_axi_bvalid,
    input  wire        s_axi_bready,
    output wire [1:0]  s_axi_bresp,
    input  wire        s_axi_arvalid,
    output wire        s_axi_arready,
    input  wire [31:0] s_axi_araddr,
    input  wire [2:0]  s_axi_arprot,
    output wire        s_axi_rvalid,
    input  wire        s_axi_rready,
    output wire [63:0] s_axi_rdata,
    output wire [1:0]  s_axi_rresp,

    // Control Outputs
    output wire        vehicle_enable,
    output wire        motor_enable,
    output wire        brake_control,
    output wire        throttle_limit,
    output wire        cooling_control,
    output wire        hazard_lights,
    output wire        door_unlock,
    output wire        airbag_control,

    // Emergency Interface
    input  wire        emergency_stop,
    input  wire        manual_override,
    output wire        emergency_ack,

    // Fault Logging
    output wire        fault_log_wr_en,
    output wire [9:0]  fault_log_addr,
    output wire [31:0] fault_log_data,
    output wire [31:0] fault_log_rd_data,

    // Debug Interface
    input  wire [2:0]  debug_mode,
    output wire [31:0] debug_data_out,
    output wire        debug_valid,

    // Test Interface
    input  wire        scan_enable,
    input  wire        test_mode,
    output wire        test_done,
    output wire        test_fail
);

    // ==================== INTERNAL WIRES ====================
    wire clk_ai, clk_aon, clk_sensor, clk_mcu;
    wire rst_ai_n, rst_aon_n, rst_sensor_n, rst_mcu_n;
    wire pll_locked;
    wire [3:0] clk_valid_status;
    wire [1:0] current_mode_aon;
    wire mode_valid, mode_stable;
    wire [41:0] sensor_map_car, sensor_map_bike;

    // Sensor data arrays (unpacked)
    wire [15:0] sensor_data [0:41];
    wire [41:0] sensor_data_valid;
    wire [41:0] sensor_filtered_valid;
    wire [7:0]  sensor_accuracy [0:41];
    wire [41:0] sensor_calibrated;

    wire [41:0] sensor_grace_expired;
    // Removed unused wire sensor_grace_count

    // AI outputs
    wire battery_health_ok, thermal_ok, motor_ok, dynamics_ok, perception_ok, crash_latched;
    wire [7:0] battery_health_score, thermal_score, motor_score, dynamics_score, perception_score;
    wire [3:0] battery_status, thermal_status, motor_status, dynamics_status, perception_status;

    // FIX: system_status is driven by system_health_ai output (lower 4 bits) with upper 4 bits
    //      hardwired to 0.  The original code had:
    //        wire [7:0] system_status;
    //        ...connect [3:0] to module...
    //        system_status[7:4] = 4'b0;   <-- procedural assign to wire - ILLEGAL
    //      Correct approach: declare a 4-bit wire for the module output, then
    //      concatenate with 4'b0 for the full 8-bit bus.
    wire [3:0]  system_status_lo;          // connected to overall_status output
    wire [7:0]  system_status;             // full 8-bit bus consumed by other modules
    assign system_status = {4'b0, system_status_lo};

    wire [31:0] fault_code;
    wire [3:0]  alert_level;
    wire [15:0] control_signals;

    wire adas_active;
    wire [7:0] adas_confidence;
    wire cruise_control, lane_keep, auto_brake;

    wire [15:0] torque_command, regen_command, steering_assist;

    wire emergency_active;
    wire [7:0] emergency_severity;

    // ADC interface outputs (first 12 channels)
    wire [31:0] digital_out_0,  digital_out_1,  digital_out_2,  digital_out_3,  digital_out_4;
    wire [31:0] digital_out_5,  digital_out_6,  digital_out_7,  digital_out_8,  digital_out_9;
    wire [31:0] digital_out_10, digital_out_11;
    wire digital_valid_0, digital_valid_1, digital_valid_2, digital_valid_3, digital_valid_4;
    wire digital_valid_5, digital_valid_6, digital_valid_7, digital_valid_8, digital_valid_9;
    wire digital_valid_10, digital_valid_11;

    // FIX: sync_cell now accepts WIDTH parameter because sync_cell.sv is parameterised.
    //      No defparam is used anywhere - parameter override is done in the #() style.
    // Population count of sensor_fault, used by system_health_ai_complete's
    // maintenance_required (>10 threshold) and predicted_failures. Previously
    // wired to |sensor_fault (a 1-bit OR-reduction, auto zero-extended to 8
    // bits), which could only ever be 0 or 1 and could never cross the >10
    // threshold.
    function [7:0] popcount42;
        input [41:0] bus;
        integer k;
        begin
            popcount42 = 8'd0;
            for (k = 0; k < 42; k = k + 1)
                popcount42 = popcount42 + bus[k];
        end
    endfunction
    wire [1:0] current_mode_ai_sync;
    // hazard_lights / door_unlock / airbag_control are each driven by BOTH
    // central_safety_fsm_v3 and emergency_response_system in the original
    // wiring, which is an electrical short (confirmed by Yosys flatten:
    // "multiple conflicting drivers"). Give each FSM its own wire and
    // OR-combine them onto the real top-level output - either system
    // asserting the signal should activate it (fail-safe).
    wire safety_hazard_lights, safety_door_unlock, safety_airbag_control;
    wire emergency_hazard_lights, emergency_door_unlock, emergency_airbag_control;
    assign hazard_lights  = safety_hazard_lights  | emergency_hazard_lights;
    assign door_unlock    = safety_door_unlock    | emergency_door_unlock;
    assign airbag_control = safety_airbag_control | emergency_airbag_control;
    sync_cell #(.WIDTH(2)) u_sync_mode_ai (
        .clk_dst   (clk_ai),
        .rst_dst_n (rst_ai_n),
        .signal_src(current_mode_aon),
        .signal_dst(current_mode_ai_sync)
    );

    // Sensor-domain synchronizers for mode / mode_valid, needed to drive
    // sensor_enable_logic (which runs on clk_sensor).
    wire [1:0] current_mode_sensor_sync;
    wire       mode_valid_sensor_sync;
    sync_cell #(.WIDTH(2)) u_sync_mode_sensor (
        .clk_dst   (clk_sensor),
        .rst_dst_n (rst_sensor_n),
        .signal_src(current_mode_aon),
        .signal_dst(current_mode_sensor_sync)
    );
    sync_cell #(.WIDTH(1)) u_sync_mode_valid_sensor (
        .clk_dst   (clk_sensor),
        .rst_dst_n (rst_sensor_n),
        .signal_src(mode_valid),
        .signal_dst(mode_valid_sensor_sync)
    );

    // Build raw arrays for sensor interface fabric from digital inputs
    wire [31:0] raw_array [0:41];
    assign raw_array[0]  = sensor_digital_in_0;
    assign raw_array[1]  = sensor_digital_in_1;
    assign raw_array[2]  = sensor_digital_in_2;
    assign raw_array[3]  = sensor_digital_in_3;
    assign raw_array[4]  = sensor_digital_in_4;
    assign raw_array[5]  = sensor_digital_in_5;
    assign raw_array[6]  = sensor_digital_in_6;
    assign raw_array[7]  = sensor_digital_in_7;
    assign raw_array[8]  = sensor_digital_in_8;
    assign raw_array[9]  = sensor_digital_in_9;
    assign raw_array[10] = sensor_digital_in_10;
    assign raw_array[11] = sensor_digital_in_11;
    assign raw_array[12] = sensor_digital_in_12;
    assign raw_array[13] = sensor_digital_in_13;
    assign raw_array[14] = sensor_digital_in_14;
    assign raw_array[15] = sensor_digital_in_15;
    assign raw_array[16] = sensor_digital_in_16;
    assign raw_array[17] = sensor_digital_in_17;
    assign raw_array[18] = sensor_digital_in_18;
    assign raw_array[19] = sensor_digital_in_19;
    assign raw_array[20] = sensor_digital_in_20;
    assign raw_array[21] = sensor_digital_in_21;
    assign raw_array[22] = sensor_digital_in_22;
    assign raw_array[23] = sensor_digital_in_23;
    assign raw_array[24] = sensor_digital_in_24;
    assign raw_array[25] = sensor_digital_in_25;
    assign raw_array[26] = sensor_digital_in_26;
    assign raw_array[27] = sensor_digital_in_27;
    assign raw_array[28] = sensor_digital_in_28;
    assign raw_array[29] = sensor_digital_in_29;
    assign raw_array[30] = sensor_digital_in_30;
    assign raw_array[31] = sensor_digital_in_31;
    assign raw_array[32] = sensor_digital_in_32;
    assign raw_array[33] = sensor_digital_in_33;
    assign raw_array[34] = sensor_digital_in_34;
    assign raw_array[35] = sensor_digital_in_35;
    assign raw_array[36] = sensor_digital_in_36;
    assign raw_array[37] = sensor_digital_in_37;
    assign raw_array[38] = sensor_digital_in_38;
    assign raw_array[39] = sensor_digital_in_39;
    assign raw_array[40] = sensor_digital_in_40;
    assign raw_array[41] = sensor_digital_in_41;

    // Valid flags array
    wire [41:0] valid_array;
    assign valid_array[0]  = sensor_digital_valid_0;
    assign valid_array[1]  = sensor_digital_valid_1;
    assign valid_array[2]  = sensor_digital_valid_2;
    assign valid_array[3]  = sensor_digital_valid_3;
    assign valid_array[4]  = sensor_digital_valid_4;
    assign valid_array[5]  = sensor_digital_valid_5;
    assign valid_array[6]  = sensor_digital_valid_6;
    assign valid_array[7]  = sensor_digital_valid_7;
    assign valid_array[8]  = sensor_digital_valid_8;
    assign valid_array[9]  = sensor_digital_valid_9;
    assign valid_array[10] = sensor_digital_valid_10;
    assign valid_array[11] = sensor_digital_valid_11;
    assign valid_array[12] = sensor_digital_valid_12;
    assign valid_array[13] = sensor_digital_valid_13;
    assign valid_array[14] = sensor_digital_valid_14;
    assign valid_array[15] = sensor_digital_valid_15;
    assign valid_array[16] = sensor_digital_valid_16;
    assign valid_array[17] = sensor_digital_valid_17;
    assign valid_array[18] = sensor_digital_valid_18;
    assign valid_array[19] = sensor_digital_valid_19;
    assign valid_array[20] = sensor_digital_valid_20;
    assign valid_array[21] = sensor_digital_valid_21;
    assign valid_array[22] = sensor_digital_valid_22;
    assign valid_array[23] = sensor_digital_valid_23;
    assign valid_array[24] = sensor_digital_valid_24;
    assign valid_array[25] = sensor_digital_valid_25;
    assign valid_array[26] = sensor_digital_valid_26;
    assign valid_array[27] = sensor_digital_valid_27;
    assign valid_array[28] = sensor_digital_valid_28;
    assign valid_array[29] = sensor_digital_valid_29;
    assign valid_array[30] = sensor_digital_valid_30;
    assign valid_array[31] = sensor_digital_valid_31;
    assign valid_array[32] = sensor_digital_valid_32;
    assign valid_array[33] = sensor_digital_valid_33;
    assign valid_array[34] = sensor_digital_valid_34;
    assign valid_array[35] = sensor_digital_valid_35;
    assign valid_array[36] = sensor_digital_valid_36;
    assign valid_array[37] = sensor_digital_valid_37;
    assign valid_array[38] = sensor_digital_valid_38;
    assign valid_array[39] = sensor_digital_valid_39;
    assign valid_array[40] = sensor_digital_valid_40;
    assign valid_array[41] = sensor_digital_valid_41;

    // ==================== INSTANTIATIONS ====================

    // 1. Clock and Reset Management
    clock_manager_14nm u_clock_manager (
        .clk_100mhz_in        (clk_100mhz),
        .clk_10mhz_in         (clk_10mhz_aon),
        .clk_50mhz_sensor_in  (clk_50mhz_sensor),
        .clk_200mhz_mcu_in    (clk_200mhz_mcu),
        .por_n                (por_n),
        .pll_bypass           (1'b0),
        .clk_gate_enable      ({pwr_en_ai, pwr_en_sensor, pwr_en_mcu}),
        .clk_ai_out           (clk_ai),
        .clk_aon_out          (clk_aon),
        .clk_sensor_out       (clk_sensor),
        .clk_mcu_out          (clk_mcu),
        .pll_locked           (pll_locked),
        .clk_valid            (clk_valid_status),
        .scan_enable          (scan_enable),
        .test_clk             (1'b0)
    );

    // 2. Reset Synchronizer
    reset_sync_v3 u_reset_sync (
        .clk_ai       (clk_ai),
        .clk_aon      (clk_aon),
        .clk_sensor   (clk_sensor),
        .clk_mcu      (clk_mcu),
        .ext_rst_n    (ext_rst_n),
        .por_n        (por_n),
        .pll_locked   (pll_locked),
        .rst_ai_n     (rst_ai_n),
        .rst_aon_n    (rst_aon_n),
        .rst_sensor_n (rst_sensor_n),
        .rst_mcu_n    (rst_mcu_n),
        .soft_reset   (1'b0),
        .watchdog_reset(1'b0)
    );

    // 3. Power Domain Controller
    power_domain_controller_v3 u_power_domain (
        .clk_aon          (clk_aon),
        .rst_aon_n        (rst_aon_n),
        .pwr_good         (pwr_good),
        .vdd_core         (vdd_core),
        .vdd_io           (vdd_io),
        .vdd_ram          (vdd_ram),
        .pd_req_ai        (1'b1),
        .pd_req_sensor    (1'b1),
        .pd_req_mcu       (1'b1),
        .pd_en_ai         (pwr_en_ai),
        .pd_en_sensor     (pwr_en_sensor),
        .pd_en_mcu        (pwr_en_mcu),
        .pd_state_ai      (),
        .pd_state_sensor  (),
        .pd_state_mcu     (),
        .retention_enable (1'b0),
        .iso_enable       (1'b0),
        .level_shifter_en (1'b0)
    );

    // 4. Mode Controller
    mode_config_enhanced_v3 u_mode_controller (
        .clk_aon            (clk_aon),
        .rst_aon_n          (rst_aon_n),
        .clk_ai             (clk_ai),
        .rst_ai_n           (rst_ai_n),
        .mode_switch_car    (mode_switch_car),
        .mode_switch_bike   (mode_switch_bike),
        .mode_auto_detect   (mode_auto_detect),
        .user_mode_override (user_mode_override),
        .active_mode        (current_mode_aon),
        .mode_valid         (mode_valid),
        .mode_stable        (mode_stable),
        .mode_change_ack    (mode_change_ack),
        .sensor_map_car     (sensor_map_car),
        .sensor_map_bike    (sensor_map_bike),
        .debug_mode         (debug_mode),
        .mode_state         (),
        .auto_detect_active ()
    );
    assign active_mode = current_mode_aon;

    // 5. ADC Interface - connects 42 analog inputs to 12 digital outputs
    adc_interface_14nm u_adc_interface (
        .clk_sensor         (clk_sensor),
        .rst_sensor_n       (rst_sensor_n),
        .adc_in_0           (sensor_adc_in_0),
        .adc_in_1           (sensor_adc_in_1),
        .adc_in_2           (sensor_adc_in_2),
        .adc_in_3           (sensor_adc_in_3),
        .adc_in_4           (sensor_adc_in_4),
        .adc_in_5           (sensor_adc_in_5),
        .adc_in_6           (sensor_adc_in_6),
        .adc_in_7           (sensor_adc_in_7),
        .adc_in_8           (sensor_adc_in_8),
        .adc_in_9           (sensor_adc_in_9),
        .adc_in_10          (sensor_adc_in_10),
        .adc_in_11          (sensor_adc_in_11),
        .adc_in_12          (sensor_adc_in_12),
        .adc_in_13          (sensor_adc_in_13),
        .adc_in_14          (sensor_adc_in_14),
        .adc_in_15          (sensor_adc_in_15),
        .adc_in_16          (sensor_adc_in_16),
        .adc_in_17          (sensor_adc_in_17),
        .adc_in_18          (sensor_adc_in_18),
        .adc_in_19          (sensor_adc_in_19),
        .adc_in_20          (sensor_adc_in_20),
        .adc_in_21          (sensor_adc_in_21),
        .adc_in_22          (sensor_adc_in_22),
        .adc_in_23          (sensor_adc_in_23),
        .adc_in_24          (sensor_adc_in_24),
        .adc_in_25          (sensor_adc_in_25),
        .adc_in_26          (sensor_adc_in_26),
        .adc_in_27          (sensor_adc_in_27),
        .adc_in_28          (sensor_adc_in_28),
        .adc_in_29          (sensor_adc_in_29),
        .adc_in_30          (sensor_adc_in_30),
        .adc_in_31          (sensor_adc_in_31),
        .adc_in_32          (sensor_adc_in_32),
        .adc_in_33          (sensor_adc_in_33),
        .adc_in_34          (sensor_adc_in_34),
        .adc_in_35          (sensor_adc_in_35),
        .adc_in_36          (sensor_adc_in_36),
        .adc_in_37          (sensor_adc_in_37),
        .adc_in_38          (sensor_adc_in_38),
        .adc_in_39          (sensor_adc_in_39),
        .adc_in_40          (sensor_adc_in_40),
        .adc_in_41          (sensor_adc_in_41),
        .adc_valid_0        (sensor_adc_valid_0),
        .adc_valid_1        (sensor_adc_valid_1),
        .adc_valid_2        (sensor_adc_valid_2),
        .adc_valid_3        (sensor_adc_valid_3),
        .adc_valid_4        (sensor_adc_valid_4),
        .adc_valid_5        (sensor_adc_valid_5),
        .adc_valid_6        (sensor_adc_valid_6),
        .adc_valid_7        (sensor_adc_valid_7),
        .adc_valid_8        (sensor_adc_valid_8),
        .adc_valid_9        (sensor_adc_valid_9),
        .adc_valid_10       (sensor_adc_valid_10),
        .adc_valid_11       (sensor_adc_valid_11),
        .adc_valid_12       (sensor_adc_valid_12),
        .adc_valid_13       (sensor_adc_valid_13),
        .adc_valid_14       (sensor_adc_valid_14),
        .adc_valid_15       (sensor_adc_valid_15),
        .adc_valid_16       (sensor_adc_valid_16),
        .adc_valid_17       (sensor_adc_valid_17),
        .adc_valid_18       (sensor_adc_valid_18),
        .adc_valid_19       (sensor_adc_valid_19),
        .adc_valid_20       (sensor_adc_valid_20),
        .adc_valid_21       (sensor_adc_valid_21),
        .adc_valid_22       (sensor_adc_valid_22),
        .adc_valid_23       (sensor_adc_valid_23),
        .adc_valid_24       (sensor_adc_valid_24),
        .adc_valid_25       (sensor_adc_valid_25),
        .adc_valid_26       (sensor_adc_valid_26),
        .adc_valid_27       (sensor_adc_valid_27),
        .adc_valid_28       (sensor_adc_valid_28),
        .adc_valid_29       (sensor_adc_valid_29),
        .adc_valid_30       (sensor_adc_valid_30),
        .adc_valid_31       (sensor_adc_valid_31),
        .adc_valid_32       (sensor_adc_valid_32),
        .adc_valid_33       (sensor_adc_valid_33),
        .adc_valid_34       (sensor_adc_valid_34),
        .adc_valid_35       (sensor_adc_valid_35),
        .adc_valid_36       (sensor_adc_valid_36),
        .adc_valid_37       (sensor_adc_valid_37),
        .adc_valid_38       (sensor_adc_valid_38),
        .adc_valid_39       (sensor_adc_valid_39),
        .adc_valid_40       (sensor_adc_valid_40),
        .adc_valid_41       (sensor_adc_valid_41),
        .adc_channel        (sensor_adc_channel),
        .digital_out_0      (digital_out_0),
        .digital_out_1      (digital_out_1),
        .digital_out_2      (digital_out_2),
        .digital_out_3      (digital_out_3),
        .digital_out_4      (digital_out_4),
        .digital_out_5      (digital_out_5),
        .digital_out_6      (digital_out_6),
        .digital_out_7      (digital_out_7),
        .digital_out_8      (digital_out_8),
        .digital_out_9      (digital_out_9),
        .digital_out_10     (digital_out_10),
        .digital_out_11     (digital_out_11),
        .digital_valid_0    (digital_valid_0),
        .digital_valid_1    (digital_valid_1),
        .digital_valid_2    (digital_valid_2),
        .digital_valid_3    (digital_valid_3),
        .digital_valid_4    (digital_valid_4),
        .digital_valid_5    (digital_valid_5),
        .digital_valid_6    (digital_valid_6),
        .digital_valid_7    (digital_valid_7),
        .digital_valid_8    (digital_valid_8),
        .digital_valid_9    (digital_valid_9),
        .digital_valid_10   (digital_valid_10),
        .digital_valid_11   (digital_valid_11),
        .calibration_enable (1'b0),
        .offset_correction  (12'd0),
        .gain_correction    (16'd256)
    );

    // 6. Sensor Interface Fabric
    sensor_interface_fabric_complete u_sensor_fabric (
        .clk_sensor          (clk_sensor),
        .rst_sensor_n        (rst_sensor_n),
        .sensor_raw_in_0     (digital_out_0),
        .sensor_raw_in_1     (digital_out_1),
        .sensor_raw_in_2     (digital_out_2),
        .sensor_raw_in_3     (digital_out_3),
        .sensor_raw_in_4     (digital_out_4),
        .sensor_raw_in_5     (digital_out_5),
        .sensor_raw_in_6     (digital_out_6),
        .sensor_raw_in_7     (digital_out_7),
        .sensor_raw_in_8     (digital_out_8),
        .sensor_raw_in_9     (digital_out_9),
        .sensor_raw_in_10    (digital_out_10),
        .sensor_raw_in_11    (digital_out_11),
        .sensor_raw_in_12    (sensor_digital_in_12),
        .sensor_raw_in_13    (sensor_digital_in_13),
        .sensor_raw_in_14    (sensor_digital_in_14),
        .sensor_raw_in_15    (sensor_digital_in_15),
        .sensor_raw_in_16    (sensor_digital_in_16),
        .sensor_raw_in_17    (sensor_digital_in_17),
        .sensor_raw_in_18    (sensor_digital_in_18),
        .sensor_raw_in_19    (sensor_digital_in_19),
        .sensor_raw_in_20    (sensor_digital_in_20),
        .sensor_raw_in_21    (sensor_digital_in_21),
        .sensor_raw_in_22    (sensor_digital_in_22),
        .sensor_raw_in_23    (sensor_digital_in_23),
        .sensor_raw_in_24    (sensor_digital_in_24),
        .sensor_raw_in_25    (sensor_digital_in_25),
        .sensor_raw_in_26    (sensor_digital_in_26),
        .sensor_raw_in_27    (sensor_digital_in_27),
        .sensor_raw_in_28    (sensor_digital_in_28),
        .sensor_raw_in_29    (sensor_digital_in_29),
        .sensor_raw_in_30    (sensor_digital_in_30),
        .sensor_raw_in_31    (sensor_digital_in_31),
        .sensor_raw_in_32    (sensor_digital_in_32),
        .sensor_raw_in_33    (sensor_digital_in_33),
        .sensor_raw_in_34    (sensor_digital_in_34),
        .sensor_raw_in_35    (sensor_digital_in_35),
        .sensor_raw_in_36    (sensor_digital_in_36),
        .sensor_raw_in_37    (sensor_digital_in_37),
        .sensor_raw_in_38    (sensor_digital_in_38),
        .sensor_raw_in_39    (sensor_digital_in_39),
        .sensor_raw_in_40    (sensor_digital_in_40),
        .sensor_raw_in_41    (sensor_digital_in_41),
        .sensor_raw_valid    (valid_array),
        .sensor_enable       (sensor_enable),
        .sensor_data_out_0   (sensor_data[0]),
        .sensor_data_out_1   (sensor_data[1]),
        .sensor_data_out_2   (sensor_data[2]),
        .sensor_data_out_3   (sensor_data[3]),
        .sensor_data_out_4   (sensor_data[4]),
        .sensor_data_out_5   (sensor_data[5]),
        .sensor_data_out_6   (sensor_data[6]),
        .sensor_data_out_7   (sensor_data[7]),
        .sensor_data_out_8   (sensor_data[8]),
        .sensor_data_out_9   (sensor_data[9]),
        .sensor_data_out_10  (sensor_data[10]),
        .sensor_data_out_11  (sensor_data[11]),
        .sensor_data_out_12  (sensor_data[12]),
        .sensor_data_out_13  (sensor_data[13]),
        .sensor_data_out_14  (sensor_data[14]),
        .sensor_data_out_15  (sensor_data[15]),
        .sensor_data_out_16  (sensor_data[16]),
        .sensor_data_out_17  (sensor_data[17]),
        .sensor_data_out_18  (sensor_data[18]),
        .sensor_data_out_19  (sensor_data[19]),
        .sensor_data_out_20  (sensor_data[20]),
        .sensor_data_out_21  (sensor_data[21]),
        .sensor_data_out_22  (sensor_data[22]),
        .sensor_data_out_23  (sensor_data[23]),
        .sensor_data_out_24  (sensor_data[24]),
        .sensor_data_out_25  (sensor_data[25]),
        .sensor_data_out_26  (sensor_data[26]),
        .sensor_data_out_27  (sensor_data[27]),
        .sensor_data_out_28  (sensor_data[28]),
        .sensor_data_out_29  (sensor_data[29]),
        .sensor_data_out_30  (sensor_data[30]),
        .sensor_data_out_31  (sensor_data[31]),
        .sensor_data_out_32  (sensor_data[32]),
        .sensor_data_out_33  (sensor_data[33]),
        .sensor_data_out_34  (sensor_data[34]),
        .sensor_data_out_35  (sensor_data[35]),
        .sensor_data_out_36  (sensor_data[36]),
        .sensor_data_out_37  (sensor_data[37]),
        .sensor_data_out_38  (sensor_data[38]),
        .sensor_data_out_39  (sensor_data[39]),
        .sensor_data_out_40  (sensor_data[40]),
        .sensor_data_out_41  (sensor_data[41]),
        .sensor_data_valid   (sensor_data_valid),
        .sensor_filtered_valid(sensor_filtered_valid),
        .sensor_accuracy_0   (sensor_accuracy[0]),
        .sensor_accuracy_1   (sensor_accuracy[1]),
        .sensor_accuracy_2   (sensor_accuracy[2]),
        .sensor_accuracy_3   (sensor_accuracy[3]),
        .sensor_accuracy_4   (sensor_accuracy[4]),
        .sensor_accuracy_5   (sensor_accuracy[5]),
        .sensor_accuracy_6   (sensor_accuracy[6]),
        .sensor_accuracy_7   (sensor_accuracy[7]),
        .sensor_accuracy_8   (sensor_accuracy[8]),
        .sensor_accuracy_9   (sensor_accuracy[9]),
        .sensor_accuracy_10  (sensor_accuracy[10]),
        .sensor_accuracy_11  (sensor_accuracy[11]),
        .sensor_accuracy_12  (sensor_accuracy[12]),
        .sensor_accuracy_13  (sensor_accuracy[13]),
        .sensor_accuracy_14  (sensor_accuracy[14]),
        .sensor_accuracy_15  (sensor_accuracy[15]),
        .sensor_accuracy_16  (sensor_accuracy[16]),
        .sensor_accuracy_17  (sensor_accuracy[17]),
        .sensor_accuracy_18  (sensor_accuracy[18]),
        .sensor_accuracy_19  (sensor_accuracy[19]),
        .sensor_accuracy_20  (sensor_accuracy[20]),
        .sensor_accuracy_21  (sensor_accuracy[21]),
        .sensor_accuracy_22  (sensor_accuracy[22]),
        .sensor_accuracy_23  (sensor_accuracy[23]),
        .sensor_accuracy_24  (sensor_accuracy[24]),
        .sensor_accuracy_25  (sensor_accuracy[25]),
        .sensor_accuracy_26  (sensor_accuracy[26]),
        .sensor_accuracy_27  (sensor_accuracy[27]),
        .sensor_accuracy_28  (sensor_accuracy[28]),
        .sensor_accuracy_29  (sensor_accuracy[29]),
        .sensor_accuracy_30  (sensor_accuracy[30]),
        .sensor_accuracy_31  (sensor_accuracy[31]),
        .sensor_accuracy_32  (sensor_accuracy[32]),
        .sensor_accuracy_33  (sensor_accuracy[33]),
        .sensor_accuracy_34  (sensor_accuracy[34]),
        .sensor_accuracy_35  (sensor_accuracy[35]),
        .sensor_accuracy_36  (sensor_accuracy[36]),
        .sensor_accuracy_37  (sensor_accuracy[37]),
        .sensor_accuracy_38  (sensor_accuracy[38]),
        .sensor_accuracy_39  (sensor_accuracy[39]),
        .sensor_accuracy_40  (sensor_accuracy[40]),
        .sensor_accuracy_41  (sensor_accuracy[41]),
        .sensor_calibrated   (sensor_calibrated),
        .filter_coefficients (16'd256),
        .moving_average_depth(4'd4),
        .deadband_threshold  (16'd5),
        .debug_mode          (debug_mode)
    );
    assign sensor_valid_out = sensor_filtered_valid;

    // 7. Sensor Grace Manager
    sensor_grace_manager_complete u_sensor_grace (
        .clk_aon          (clk_aon),
        .rst_aon_n        (rst_aon_n),
        .clk_sensor       (clk_sensor),
        .rst_sensor_n     (rst_sensor_n),
        .sensor_fault     (sensor_fault),
        .sensor_enable    (sensor_enable),
        .sensor_data_valid(sensor_data_valid),
        .grace_period_count(3'd5),
        .grace_timeout    (32'd1000000),
        .sensor_grace_active(sensor_grace_active),
        .sensor_grace_expired(sensor_grace_expired),
        .force_disable    (1'b0),
        .reset_grace      (1'b0)
    );

    // 8. Sensor Validation FSM
    sensor_validation_fsm u_sensor_validation (
        .clk_sensor         (clk_sensor),
        .rst_sensor_n       (rst_sensor_n),
        .sensor_data_0      (sensor_data[0]),
        .sensor_data_1      (sensor_data[1]),
        .sensor_data_2      (sensor_data[2]),
        .sensor_data_3      (sensor_data[3]),
        .sensor_data_4      (sensor_data[4]),
        .sensor_data_5      (sensor_data[5]),
        .sensor_data_6      (sensor_data[6]),
        .sensor_data_7      (sensor_data[7]),
        .sensor_data_8      (sensor_data[8]),
        .sensor_data_9      (sensor_data[9]),
        .sensor_data_10     (sensor_data[10]),
        .sensor_data_11     (sensor_data[11]),
        .sensor_data_12     (sensor_data[12]),
        .sensor_data_13     (sensor_data[13]),
        .sensor_data_14     (sensor_data[14]),
        .sensor_data_15     (sensor_data[15]),
        .sensor_data_16     (sensor_data[16]),
        .sensor_data_17     (sensor_data[17]),
        .sensor_data_18     (sensor_data[18]),
        .sensor_data_19     (sensor_data[19]),
        .sensor_data_20     (sensor_data[20]),
        .sensor_data_21     (sensor_data[21]),
        .sensor_data_22     (sensor_data[22]),
        .sensor_data_23     (sensor_data[23]),
        .sensor_data_24     (sensor_data[24]),
        .sensor_data_25     (sensor_data[25]),
        .sensor_data_26     (sensor_data[26]),
        .sensor_data_27     (sensor_data[27]),
        .sensor_data_28     (sensor_data[28]),
        .sensor_data_29     (sensor_data[29]),
        .sensor_data_30     (sensor_data[30]),
        .sensor_data_31     (sensor_data[31]),
        .sensor_data_32     (sensor_data[32]),
        .sensor_data_33     (sensor_data[33]),
        .sensor_data_34     (sensor_data[34]),
        .sensor_data_35     (sensor_data[35]),
        .sensor_data_36     (sensor_data[36]),
        .sensor_data_37     (sensor_data[37]),
        .sensor_data_38     (sensor_data[38]),
        .sensor_data_39     (sensor_data[39]),
        .sensor_data_40     (sensor_data[40]),
        .sensor_data_41     (sensor_data[41]),
        .sensor_data_valid  (sensor_data_valid),
        .sensor_grace_active(sensor_grace_active),
        .expected_ranges_min_0(16'd0),
        .expected_ranges_min_1(16'd0),
        .expected_ranges_min_2(16'd0),
        .expected_ranges_min_3(16'd0),
        .expected_ranges_min_4(16'd0),
        .expected_ranges_min_5(16'd0),
        .expected_ranges_min_6(16'd0),
        .expected_ranges_min_7(16'd0),
        .expected_ranges_min_8(16'd0),
        .expected_ranges_min_9(16'd0),
        .expected_ranges_max_0(16'hFFFF),
        .expected_ranges_max_1(16'hFFFF),
        .expected_ranges_max_2(16'hFFFF),
        .expected_ranges_max_3(16'hFFFF),
        .expected_ranges_max_4(16'hFFFF),
        .expected_ranges_max_5(16'hFFFF),
        .expected_ranges_max_6(16'hFFFF),
        .expected_ranges_max_7(16'hFFFF),
        .expected_ranges_max_8(16'hFFFF),
        .expected_ranges_max_9(16'hFFFF),
        .sensor_fault       (sensor_fault),
        .validation_timeout (32'd1000),
        .hysteresis_threshold(16'd10)
    );

    // 8b. Sensor Enable Logic - drives the sensor_enable bus consumed by the
    //     fabric and grace manager above (previously omitted, leaving
    //     sensor_enable permanently undriven/floating)
    sensor_enable_logic u_sensor_enable (
        .clk_sensor          (clk_sensor),
        .rst_sensor_n        (rst_sensor_n),
        .current_mode        (current_mode_sensor_sync),
        .mode_valid          (mode_valid_sensor_sync),
        .sensor_map_car      (sensor_map_car),
        .sensor_map_bike     (sensor_map_bike),
        .sensor_fault        (sensor_fault),
        .sensor_grace_expired(sensor_grace_expired),
        .sensor_force_disable(42'd0),
        .sensor_enable       (sensor_enable),
        .debug_enable        (1'b0)
    );

    // 9. AI Modules
    battery_predictive_ai_complete u_battery_ai (
        .clk_ai           (clk_ai),
        .rst_ai_n         (rst_ai_n),
        .batt_cell_temp   (sensor_data[0]),
        .batt_pack_temp   (sensor_data[1]),
        .ambient_temp     (sensor_data[4]),
        .batt_cell_volt   (sensor_data[5]),
        .batt_pack_volt   (sensor_data[6]),
        .batt_current     (sensor_data[8]),
        .charging_volt    (sensor_data[7]),
        .charging_current (sensor_data[9]),
        .soc              (sensor_data[11]),
        .soh              (sensor_data[12]),
        .sensor_valid     ({sensor_data_valid[12], sensor_data_valid[11], sensor_data_valid[9],
                            sensor_data_valid[7], sensor_data_valid[8], sensor_data_valid[6],
                            sensor_data_valid[5], sensor_data_valid[4], sensor_data_valid[1],
                            sensor_data_valid[0]}),
        .battery_health_ok(battery_health_ok),
        .battery_health_score(battery_health_score),
        .battery_status   (battery_status),
        .battery_fault_code(),
        .predictive_alerts(),
        .estimated_range_km(),
        .charging_safety_ok()
    );

    thermal_management_hierarchical_complete u_thermal_ai (
        .clk_ai           (clk_ai),
        .rst_ai_n         (rst_ai_n),
        .batt_cell_temp   (sensor_data[0]),
        .batt_pack_temp   (sensor_data[1]),
        .motor_temp       (sensor_data[2]),
        .inverter_temp    (sensor_data[3]),
        .cabin_temp       (sensor_data[34]),
        .coolant_flow     (sensor_data[10]),
        .cooling_press    (sensor_data[25]),
        .humidity         (sensor_data[26]),
        .enclosure_press  (sensor_data[27]),
        .sensor_valid     ({sensor_data_valid[27], sensor_data_valid[26], sensor_data_valid[25],
                            sensor_data_valid[10], sensor_data_valid[34], sensor_data_valid[3],
                            sensor_data_valid[2], sensor_data_valid[1], sensor_data_valid[0]}),
        .thermal_ok       (thermal_ok),
        .thermal_score    (thermal_score),
        .thermal_status   (thermal_status),
        .cooling_required (cooling_control),
        .batt_thermal_status(),
        .motor_thermal_status(),
        .cabin_thermal_status(),
        .temp_trend(),
        .overheat_prediction()
    );

    motor_condition_enhanced_complete u_motor_ai (
        .clk_ai           (clk_ai),
        .rst_ai_n         (rst_ai_n),
        .motor_temp       (sensor_data[2]),
        .motor_temp_valid (sensor_data_valid[2]),
        .inverter_temp    (sensor_data[3]),
        .inverter_temp_valid(sensor_data_valid[3]),
        .motor_coolant_temp(sensor_data[10]),
        .motor_coolant_valid(sensor_data_valid[10]),
        .inverter_coolant_temp(sensor_data[10]),
        .inverter_coolant_valid(sensor_data_valid[10]),
        .motor_rpm        (sensor_data[17]),
        .motor_rpm_valid  (sensor_data_valid[17]),
        .rotor_position   (sensor_data[18]),
        .rotor_position_valid(sensor_data_valid[18]),
        .wheel_speed_front(sensor_data[16]),
        .wheel_speed_front_valid(sensor_data_valid[16]),
        .wheel_speed_rear (sensor_data[16]),
        .wheel_speed_rear_valid(sensor_data_valid[16]),
        .throttle_position(sensor_data[19]),
        .throttle_valid   (sensor_data_valid[19]),
        .brake_pressure   (sensor_data[20]),
        .brake_pressure_valid(sensor_data_valid[20]),
        .brake_switch     (sensor_digital_in_21[0]),
        .motor_current    (sensor_data[8]),
        .motor_current_valid(sensor_data_valid[8]),
        .motor_voltage    (sensor_data[6]),
        .motor_voltage_valid(sensor_data_valid[6]),
        .inverter_dc_voltage(sensor_data[6]),
        .inverter_voltage_valid(sensor_data_valid[6]),
        .vibration_level  (sensor_data[14]),
        .vibration_valid  (sensor_data_valid[14]),
        .acoustic_noise   (sensor_data[15]),
        .acoustic_valid   (sensor_data_valid[15]),
        .tire_pressure_front(sensor_data[33]),
        .tpms_front_valid (sensor_data_valid[33]),
        .tire_pressure_rear(sensor_data[33]),
        .tpms_rear_valid  (sensor_data_valid[33]),
        .vehicle_on       (pwr_good),
        .vehicle_speed    (sensor_data[16]),
        .vehicle_odometer (32'd0),
        .vehicle_mode     (current_mode_ai_sync),
        .motor_max_rpm    (16'd12000),
        .motor_max_torque (16'd500),
        .motor_nominal_current(16'd300),
        .motor_health_score(motor_score),
        .motor_critical   (),
        .motor_efficiency (),
        .bearing_wear_level(),
        .winding_insulation_health(),
        .torque_ripple_level(),
        .cooling_system_effectiveness(),
        .predicted_motor_failures_30d(),
        .bearing_life_remaining(),
        .estimated_motor_life_km(),
        .optimal_temperature_range(),
        .bearing_wear_detected(),
        .winding_degradation_detected(),
        .cooling_system_issue(),
        .rotor_imbalance_detected(),
        .brake_pad_wear_detected(),
        .tire_wear_detected(),
        .reduce_load_recommended(),
        .service_motor_recommended(),
        .replace_bearings_soon(),
        .check_cooling_system(),
        .motor_ok         (motor_ok),
        .efficiency_map   (),
        .debug_temp_vs_load(),
        .debug_vibration_spectrum(),
        .debug_efficiency_trend()
    );

    assign motor_health_status = (motor_score > 8'd80) ? 4'd0 :
                                 (motor_score > 8'd50) ? 4'd1 :
                                 (motor_score > 8'd20) ? 4'd2 : 4'd3;

    // motor AI has no dedicated status output; derive one from motor_score so
    // central_safety_fsm_v3.motor_status and system_health_ai.motor_status are
    // never left floating.
    assign motor_status = (motor_score >= 8'd80) ? `STATUS_OK :
                          (motor_score >= 8'd50) ? `STATUS_WARNING :
                          (motor_score >= 8'd20) ? `STATUS_CRITICAL : `STATUS_EMERGENCY;

    vehicle_dynamics_predictive_complete u_dynamics_ai (
        .clk_ai           (clk_ai),
        .rst_ai_n         (rst_ai_n),
        .steering_angle   (sensor_data[22]),
        .gear_position    (sensor_data[24]),
        .side_stand       (sensor_data[23]),
        .gyroscope        (sensor_data[14]),
        .imu              (sensor_data[15]),
        .wheel_speed      (sensor_data[16]),
        .sensor_valid     ({sensor_data_valid[16], sensor_data_valid[15], sensor_data_valid[14],
                            sensor_data_valid[23], sensor_data_valid[24], sensor_data_valid[22]}),
        .current_mode     (current_mode_ai_sync),
        .vehicle_speed    (sensor_data[16]),
        .dynamics_ok      (dynamics_ok),
        .dynamics_score   (dynamics_score),
        .dynamics_status  (dynamics_status),
        .stability_control(),
        .rollover_risk   (),
        .skid_detected   ()
    );

    perception_health_ai_complete u_perception_ai (
        .clk_ai           (clk_ai),
        .rst_ai_n         (rst_ai_n),
        .ultrasonic       (sensor_data[28]),
        .camera           (sensor_data[29]),
        .radar            (sensor_data[30]),
        .lidar            (sensor_data[31]),
        .gps              (sensor_data[32]),
        .tpms             (sensor_data[33]),
        .sensor_valid     ({sensor_data_valid[33], sensor_data_valid[32], sensor_data_valid[31],
                            sensor_data_valid[30], sensor_data_valid[29], sensor_data_valid[28]}),
        .current_mode     (current_mode_ai_sync),
        .vehicle_speed    (sensor_data[16]),
        .perception_ok    (perception_ok),
        .perception_score (perception_score),
        .perception_status(perception_status),
        .sensor_confidence_0(),
        .sensor_confidence_1(),
        .sensor_confidence_2(),
        .sensor_confidence_3(),
        .sensor_confidence_4(),
        .sensor_confidence_5(),
        .blind_spot_warning(),
        .collision_warning(),
        .lane_departure   ()
    );

    crash_predictive_ai_complete u_crash_ai (
        .clk_aon          (clk_aon),
        .rst_aon_n        (rst_aon_n),
        .crash_impact     (sensor_data[13]),
        .gyroscope        (sensor_data[14]),
        .imu              (sensor_data[15]),
        .sensor_valid     ({sensor_data_valid[15], sensor_data_valid[14], sensor_data_valid[13]}),
        .crash_latched    (crash_latched),
        .crash_severity   (),
        .crash_direction  (),
        .pre_crash_warning(),
        .multi_impact_detect(),
        .crash_reset      (1'b0),
        .test_mode        (test_mode)
    );

    system_health_ai_complete u_system_health (
        .clk_ai               (clk_ai),
        .rst_ai_n             (rst_ai_n),
        .battery_score        (battery_health_score),
        .thermal_score        (thermal_score),
        .motor_score          (motor_score),
        .dynamics_score       (dynamics_score),
        .perception_score     (perception_score),
        .battery_status       (battery_status),
        .thermal_status       (thermal_status),
        .motor_status         (motor_status),
        .dynamics_status      (dynamics_status),
        .perception_status    (perception_status),
        .current_mode         (current_mode_ai_sync),
        .sensor_fault_count   (popcount42(sensor_fault)),
        .system_health_score  (system_health_score),
        .overall_status       (system_status_lo),
        .maintenance_required(),
        .predicted_failures   ()
    );
    // system_status = {4'b0, system_status_lo} is done via assign above the instantiations

    // 10. Central Safety FSM
    central_safety_fsm_v3 u_central_fsm (
        .clk_aon          (clk_aon),
        .rst_aon_n        (rst_aon_n),
        .battery_health_ok(battery_health_ok),
        .thermal_ok       (thermal_ok),
        .motor_ok         (motor_ok),
        .dynamics_ok      (dynamics_ok),
        .perception_ok    (perception_ok),
        .crash_latched    (crash_latched),
        .battery_score    (battery_health_score),
        .thermal_score    (thermal_score),
        .motor_score      (motor_score),
        .dynamics_score   (dynamics_score),
        .perception_score (perception_score),
        .battery_status   (battery_status),
        .thermal_status   (thermal_status),
        .motor_status     (motor_status),
        .dynamics_status  (dynamics_status),
        .perception_status(perception_status),
        .emergency_stop   (emergency_stop),
        .manual_override  (manual_override),
        .current_mode     (current_mode_aon),
        .vehicle_enable   (vehicle_enable),
        .motor_enable     (motor_enable),
        .brake_control    (brake_control),
        .throttle_limit   (throttle_limit),
        .hazard_lights    (safety_hazard_lights),
        .door_unlock      (safety_door_unlock),
        .airbag_control   (safety_airbag_control),
        .emergency_ack    (emergency_ack),
        .alert_level      (alert_level),
        .control_signals  (control_signals),
        .fault_code       (fault_code[7:0]),
        .fault_timestamp  (),
        .debug_mode       (debug_mode)
    );

    // 11. ADAS Controller
    adas_controller_v3 u_adas_controller (
        .clk_ai           (clk_ai),
        .rst_ai_n         (rst_ai_n),
        .perception_ok    (perception_ok),
        .dynamics_ok      (dynamics_ok),
        .current_mode     (current_mode_ai_sync),
        .ultrasonic       (sensor_data[28]),
        .camera           (sensor_data[29]),
        .radar            (sensor_data[30]),
        .lidar            (sensor_data[31]),
        .gps              (sensor_data[32]),
        .vehicle_speed    (sensor_data[16]),
        .steering_angle   (sensor_data[22]),
        .throttle_position(sensor_data[19]),
        .brake_pressure   (sensor_data[20]),
        .sensor_valid     ({sensor_data_valid[20], sensor_data_valid[19], sensor_data_valid[22],
                            sensor_data_valid[16], sensor_data_valid[32], sensor_data_valid[31],
                            sensor_data_valid[30], sensor_data_valid[29], sensor_data_valid[28]}),
        .adas_active      (adas_active),
        .adas_confidence  (adas_confidence),
        .cruise_control   (cruise_control),
        .lane_keep        (lane_keep),
        .auto_brake       (auto_brake),
        .following_distance(),
        .lane_offset      (),
        .collision_time   ()
    );

    // 12. Motor Control
    motor_control_hybrid u_motor_control (
        .clk_ai               (clk_ai),
        .rst_ai_n             (rst_ai_n),
        .motor_ok             (motor_ok),
        .thermal_ok           (thermal_ok),
        .dynamics_ok          (dynamics_ok),
        .current_mode         (current_mode_ai_sync),
        .throttle_position    (sensor_data[19]),
        .brake_pressure       (sensor_data[20]),
        .vehicle_speed        (sensor_data[16]),
        .motor_rpm            (sensor_data[17]),
        .motor_temp           (sensor_data[2]),
        .battery_soc          (sensor_data[11]),
        .adas_cruise_control  (cruise_control),
        .adas_auto_brake      (auto_brake),
        .torque_command       (torque_command),
        .regen_command        (regen_command),
        .steering_assist      (steering_assist),
        .torque_limit         (),
        .regen_limit          (),
        .power_limit          ()
    );

    // 13. Emergency Response System
    emergency_response_system u_emergency (
        .clk_aon          (clk_aon),
        .rst_aon_n        (rst_aon_n),
        .crash_latched    (crash_latched),
        .emergency_stop   (emergency_stop),
        .system_status    (system_status),
        .alert_level      (alert_level),
        .hazard_lights    (emergency_hazard_lights),
        .door_unlock      (emergency_door_unlock),
        .airbag_control   (emergency_airbag_control),
        .emergency_severity(emergency_severity),
        .sos_signal       (),
        .location_data    (),
        .battery_backup_enable()
    );

    // 14. Fault Logger
    fault_logger_sram_32kb u_fault_logger (
        .clk_aon          (clk_aon),
        .rst_aon_n        (rst_aon_n),
        .wr_en            (fault_log_wr_en),
        .wr_addr          (fault_log_addr),
        .wr_data          (fault_log_data),
        .rd_en            (1'b0),
        .rd_addr          (10'd0),
        .rd_data          (fault_log_rd_data),
        .fault_code       (fault_code[7:0]),
        .timestamp        (32'd0),
        .sensor_data_0    (sensor_data[0]),
        .sensor_data_1    (sensor_data[1]),
        .sensor_data_2    (sensor_data[2]),
        .sensor_data_3    (sensor_data[3]),
        .log_full         (),
        .log_overflow     (),
        .log_count        (),
        .log_clear        (1'b0),
        .log_read_only    (1'b0),
        .debug_mode       (debug_mode)
    );

    // 15. Diagnostic Report Generator
    diagnostic_report_generator u_diagnostic (
        .clk_ai               (clk_ai),
        .rst_ai_n             (rst_ai_n),
        .system_health_score  (system_health_score),
        .battery_score        (battery_health_score),
        .motor_score          (motor_score),
        .thermal_score        (thermal_score),
        .dynamics_score       (dynamics_score),
        .perception_score     (perception_score),
        .sensor_fault         (sensor_fault),
        .ai_status            (6'd0),
        .current_mode         (current_mode_ai_sync),
        .vehicle_odometer     (32'd0),
        .motor_runtime        (32'd0),
        .generate_report      (1'b0),
        .continuous_monitoring(1'b1),
        .report_detail_level  (debug_mode)
    );

    // 16. MCU AXI Interface
    mcu_axi_lite_interface u_mcu_interface (
        .clk_mcu          (clk_mcu),
        .rst_mcu_n        (rst_mcu_n),
        .m_axi_awvalid    (m_axi_awvalid),
        .m_axi_awready    (m_axi_awready),
        .m_axi_awaddr     (m_axi_awaddr),
        .m_axi_awprot     (m_axi_awprot),
        .m_axi_wvalid     (m_axi_wvalid),
        .m_axi_wready     (m_axi_wready),
        .m_axi_wdata      (m_axi_wdata),
        .m_axi_wstrb      (m_axi_wstrb),
        .m_axi_bvalid     (m_axi_bvalid),
        .m_axi_bready     (m_axi_bready),
        .m_axi_bresp      (m_axi_bresp),
        .m_axi_arvalid    (m_axi_arvalid),
        .m_axi_arready    (m_axi_arready),
        .m_axi_araddr     (m_axi_araddr),
        .m_axi_arprot     (m_axi_arprot),
        .m_axi_rvalid     (m_axi_rvalid),
        .m_axi_rready     (m_axi_rready),
        .m_axi_rdata      (m_axi_rdata),
        .m_axi_rresp      (m_axi_rresp),
        .s_axi_awvalid    (s_axi_awvalid),
        .s_axi_awready    (s_axi_awready),
        .s_axi_awaddr     (s_axi_awaddr),
        .s_axi_awprot     (s_axi_awprot),
        .s_axi_wvalid     (s_axi_wvalid),
        .s_axi_wready     (s_axi_wready),
        .s_axi_wdata      (s_axi_wdata),
        .s_axi_wstrb      (s_axi_wstrb),
        .s_axi_bvalid     (s_axi_bvalid),
        .s_axi_bready     (s_axi_bready),
        .s_axi_bresp      (s_axi_bresp),
        .s_axi_arvalid    (s_axi_arvalid),
        .s_axi_arready    (s_axi_arready),
        .s_axi_araddr     (s_axi_araddr),
        .s_axi_arprot     (s_axi_arprot),
        .s_axi_rvalid     (s_axi_rvalid),
        .s_axi_rready     (s_axi_rready),
        .s_axi_rdata      (s_axi_rdata),
        .s_axi_rresp      (s_axi_rresp),
        .sensor_data_0    (sensor_data[0]),
        .sensor_data_1    (sensor_data[1]),
        .sensor_data_2    (sensor_data[2]),
        .sensor_data_3    (sensor_data[3]),
        .sensor_data_4    (sensor_data[4]),
        .sensor_data_5    (sensor_data[5]),
        .sensor_data_6    (sensor_data[6]),
        .sensor_data_7    (sensor_data[7]),
        .sensor_data_8    (sensor_data[8]),
        .sensor_data_9    (sensor_data[9]),
        .sensor_data_10   (sensor_data[10]),
        .sensor_data_11   (sensor_data[11]),
        .sensor_data_12   (sensor_data[12]),
        .sensor_data_13   (sensor_data[13]),
        .sensor_data_14   (sensor_data[14]),
        .sensor_data_15   (sensor_data[15]),
        .sensor_data_16   (sensor_data[16]),
        .sensor_data_17   (sensor_data[17]),
        .sensor_data_18   (sensor_data[18]),
        .sensor_data_19   (sensor_data[19]),
        .sensor_data_20   (sensor_data[20]),
        .sensor_data_21   (sensor_data[21]),
        .sensor_data_22   (sensor_data[22]),
        .sensor_data_23   (sensor_data[23]),
        .sensor_data_24   (sensor_data[24]),
        .sensor_data_25   (sensor_data[25]),
        .sensor_data_26   (sensor_data[26]),
        .sensor_data_27   (sensor_data[27]),
        .sensor_data_28   (sensor_data[28]),
        .sensor_data_29   (sensor_data[29]),
        .sensor_data_30   (sensor_data[30]),
        .sensor_data_31   (sensor_data[31]),
        .sensor_data_32   (sensor_data[32]),
        .sensor_data_33   (sensor_data[33]),
        .sensor_data_34   (sensor_data[34]),
        .sensor_data_35   (sensor_data[35]),
        .sensor_data_36   (sensor_data[36]),
        .sensor_data_37   (sensor_data[37]),
        .sensor_data_38   (sensor_data[38]),
        .sensor_data_39   (sensor_data[39]),
        .sensor_data_40   (sensor_data[40]),
        .sensor_data_41   (sensor_data[41]),
        .sensor_status    ({2'b00, sensor_fault, sensor_grace_active, sensor_filtered_valid}),
        .ai_status        ({75'd0, battery_health_ok, thermal_ok, motor_ok, dynamics_ok, perception_ok,
                            battery_health_score, thermal_score, motor_score, dynamics_score,
                            perception_score, system_health_score}),
        .control_register (control_signals),
        .status_register  (system_status),
        .fault_register   (fault_code[7:0]),
        .write_enable     (fault_log_wr_en),
        .write_addr       (fault_log_addr),
        .write_data       (fault_log_data),
        .read_data        (fault_log_rd_data),
        .debug_mode       (debug_mode),
        .debug_data       (debug_data_out),
        .debug_valid      (debug_valid)
    );

    // Combinational outputs
    assign ai_processing_active = pwr_en_ai & pll_locked & mode_valid;
    assign power_state = {pwr_en_ai, pwr_en_sensor};
    assign battery_health_status = battery_status;
    assign thermal_health_status = thermal_status;
    assign safety_health_status  = (crash_latched) ? `STATUS_EMERGENCY : system_status[3:0];
    assign test_done = 1'b1;
    assign test_fail = 1'b0;

endmodule
`default_nettype wire