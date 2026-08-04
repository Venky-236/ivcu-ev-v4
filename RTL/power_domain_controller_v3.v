// power_domain_controller_v3.v
`timescale 1ns/1ps
`include "defines_ivcu_ev_v3.sv"   // FIX: file is defines_ivcu_ev_v3.sv, .v version never existed

module power_domain_controller_v3 (
    input  wire        clk_aon,
    input  wire        rst_aon_n,
    input  wire        pwr_good,
    input  wire        vdd_core,
    input  wire        vdd_io,
    input  wire        vdd_ram,
    input  wire        pd_req_ai,
    input  wire        pd_req_sensor,
    input  wire        pd_req_mcu,
    output reg         pd_en_ai,
    output reg         pd_en_sensor,
    output reg         pd_en_mcu,
    output reg  [1:0]  pd_state_ai,
    output reg  [1:0]  pd_state_sensor,
    output reg  [1:0]  pd_state_mcu,
    input  wire        retention_enable,
    input  wire        iso_enable,
    input  wire        level_shifter_en,
    output wire        power_sequence_done,
    output wire [2:0]  power_fault,
    output wire        retention_active
);

    // ==================== PARAMETERS ====================
    localparam [1:0] PD_OFF       = 2'b00;
    localparam [1:0] PD_STANDBY   = 2'b01;
    localparam [1:0] PD_RETENTION = 2'b10;
    localparam [1:0] PD_ACTIVE    = 2'b11;

    localparam [7:0] SEQ_DELAY = 8'd10;

    localparam [2:0] SEQ_IDLE     = 3'b000;
    localparam [2:0] SEQ_RAM      = 3'b001;
    localparam [2:0] SEQ_CORE     = 3'b010;
    localparam [2:0] SEQ_IO       = 3'b011;
    localparam [2:0] SEQ_WAIT     = 3'b100;
    localparam [2:0] SEQ_COMPLETE = 3'b101;

    // ==================== INTERNAL REGISTERS ============
    reg [2:0] seq_state;
    reg [2:0] seq_next;
    reg [7:0] seq_delay_counter;
    reg [3:0] power_up_step;

    reg [2:0] power_fault_reg;
    reg [15:0] undervolt_counter_ai;
    reg [15:0] undervolt_counter_sensor;
    reg [15:0] undervolt_counter_mcu;

    reg retention_active_reg;
    reg [1:0] retention_state_ai;
    reg [1:0] retention_state_sensor;
    reg [1:0] retention_state_mcu;

    // ==================== POWER SEQUENCE AND CONTROL FSM (merged) =====
    always @(posedge clk_aon or negedge rst_aon_n) begin : power_control
        if (!rst_aon_n) begin
            seq_state <= SEQ_IDLE;
            seq_delay_counter <= 8'd0;
            power_up_step <= 4'd0;
            pd_en_ai <= 1'b0;
            pd_en_sensor <= 1'b0;
            pd_en_mcu <= 1'b0;
            pd_state_ai <= PD_OFF;
            pd_state_sensor <= PD_OFF;
            pd_state_mcu <= PD_OFF;
            retention_active_reg <= 1'b0;
            retention_state_ai <= PD_OFF;
            retention_state_sensor <= PD_OFF;
            retention_state_mcu <= PD_OFF;
        end else if (pwr_good) begin
            // Next state logic (combinational part evaluated before clock edge)
            seq_next = seq_state;
            case (seq_state)
                SEQ_IDLE: if (pd_req_ai || pd_req_sensor || pd_req_mcu) seq_next = SEQ_RAM;
                SEQ_RAM: if (seq_delay_counter == 8'd0) seq_next = SEQ_CORE;
                SEQ_CORE: if (seq_delay_counter == 8'd0) seq_next = SEQ_IO;
                SEQ_IO: if (seq_delay_counter == 8'd0) seq_next = SEQ_WAIT;
                SEQ_WAIT: if (seq_delay_counter == 8'd0) seq_next = SEQ_COMPLETE;
                SEQ_COMPLETE: if (!pd_req_ai && !pd_req_sensor && !pd_req_mcu) seq_next = SEQ_IDLE;
                default: seq_next = SEQ_IDLE;
            endcase

            // Update state
            seq_state <= seq_next;

            // Delay counter
            if (seq_state != seq_next) begin
                seq_delay_counter <= SEQ_DELAY;
            end else if (seq_delay_counter > 8'd0) begin
                seq_delay_counter <= seq_delay_counter - 8'd1;
            end

            // Power domain control (AI)
            if (pd_req_ai) begin
                if (seq_state == SEQ_COMPLETE) begin
                    pd_state_ai <= PD_ACTIVE;
                    pd_en_ai <= 1'b1;
                end else if (retention_enable) begin
                    pd_state_ai <= PD_RETENTION;
                    retention_state_ai <= PD_RETENTION;
                end else begin
                    pd_state_ai <= PD_STANDBY;
                end
            end else begin
                pd_state_ai <= PD_OFF;
                pd_en_ai <= 1'b0;
            end

            // Power domain control (Sensor)
            if (pd_req_sensor) begin
                if (seq_state == SEQ_COMPLETE) begin
                    pd_state_sensor <= PD_ACTIVE;
                    pd_en_sensor <= 1'b1;
                end else if (retention_enable) begin
                    pd_state_sensor <= PD_RETENTION;
                    retention_state_sensor <= PD_RETENTION;
                end else begin
                    pd_state_sensor <= PD_STANDBY;
                end
            end else begin
                pd_state_sensor <= PD_OFF;
                pd_en_sensor <= 1'b0;
            end

            // Power domain control (MCU)
            if (pd_req_mcu) begin
                if (seq_state == SEQ_COMPLETE) begin
                    pd_state_mcu <= PD_ACTIVE;
                    pd_en_mcu <= 1'b1;
                end else if (retention_enable) begin
                    pd_state_mcu <= PD_RETENTION;
                    retention_state_mcu <= PD_RETENTION;
                end else begin
                    pd_state_mcu <= PD_STANDBY;
                end
            end else begin
                pd_state_mcu <= PD_OFF;
                pd_en_mcu <= 1'b0;
            end

            retention_active_reg <= (retention_state_ai == PD_RETENTION) ||
                                     (retention_state_sensor == PD_RETENTION) ||
                                     (retention_state_mcu == PD_RETENTION);
        end
    end

    // ==================== POWER FAULT DETECTION ========
    always @(posedge clk_aon or negedge rst_aon_n) begin : fault_detect
        if (!rst_aon_n) begin
            power_fault_reg <= 3'b000;
            undervolt_counter_ai <= 16'd0;
            undervolt_counter_sensor <= 16'd0;
            undervolt_counter_mcu <= 16'd0;
        end else begin
            // AI domain  (vdd_core=1 means good; vdd_core=0 means undervolt)
            if (!vdd_core) begin
                if (undervolt_counter_ai < 16'hFFFF) undervolt_counter_ai <= undervolt_counter_ai + 16'd1;
                if (undervolt_counter_ai > 16'd1000) power_fault_reg[0] <= 1'b1;
            end else begin
                undervolt_counter_ai <= 16'd0;
                power_fault_reg[0] <= 1'b0;
            end
            // Sensor domain (also monitored via vdd_core rail)
            if (!vdd_core) begin
                if (undervolt_counter_sensor < 16'hFFFF) undervolt_counter_sensor <= undervolt_counter_sensor + 16'd1;
                if (undervolt_counter_sensor > 16'd1000) power_fault_reg[1] <= 1'b1;
            end else begin
                undervolt_counter_sensor <= 16'd0;
                power_fault_reg[1] <= 1'b0;
            end
            // MCU domain (monitored via vdd_io rail)
            if (!vdd_io) begin
                if (undervolt_counter_mcu < 16'hFFFF) undervolt_counter_mcu <= undervolt_counter_mcu + 16'd1;
                if (undervolt_counter_mcu > 16'd1000) power_fault_reg[2] <= 1'b1;
            end else begin
                undervolt_counter_mcu <= 16'd0;
                power_fault_reg[2] <= 1'b0;
            end
        end
    end

    assign power_sequence_done = (seq_state == SEQ_COMPLETE);
    assign power_fault = power_fault_reg;
    assign retention_active = retention_active_reg;

endmodule