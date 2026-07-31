// reset_sync_v3.v
`timescale 1ns/1ps
`include "defines_ivcu_ev_v3.sv"   // FIX: file is defines_ivcu_ev_v3.sv, .v version never existed

module reset_sync_v3 (
    // Input clocks
    input  wire clk_ai,
    input  wire clk_aon,
    input  wire clk_sensor,
    input  wire clk_mcu,
    
    // Input resets
    input  wire ext_rst_n,
    input  wire por_n,
    input  wire pll_locked,
    
    // Control inputs
    input  wire soft_reset,
    input  wire watchdog_reset,
    
    // Output synchronized resets
    output wire rst_ai_n,
    output wire rst_aon_n,
    output wire rst_sensor_n,
    output wire rst_mcu_n,
    
    // Status outputs
    output wire reset_status_valid,
    output wire [3:0] reset_hold_count
);

    // ==================== PARAMETERS ====================
    localparam RESET_HOLD_CYCLES = 8'd16;  // 16 clock cycles
    localparam DEBOUNCE_CYCLES = 8'd8;     // 8 clock cycles
    
    // ==================== INTERNAL REGISTERS ============
    // Reset synchronizers (2FF)
    reg [1:0] rst_sync_ai_ff;
    reg [1:0] rst_sync_aon_ff;
    reg [1:0] rst_sync_sensor_ff;
    reg [1:0] rst_sync_mcu_ff;
    
    // Reset hold counters
    reg [7:0] hold_counter_ai;
    reg [7:0] hold_counter_aon;
    reg [7:0] hold_counter_sensor;
    reg [7:0] hold_counter_mcu;
    
    // Reset sources
    reg reset_source;
    reg [2:0] reset_source_sync;
    
    // Reset debounce
    reg [7:0] debounce_counter;
    reg debounce_active;
    
    // ==================== RESET SOURCE LOGIC ============
    always @(posedge clk_aon or negedge por_n) begin
        if (!por_n) begin
            reset_source <= 1'b1;  // Active high reset
            reset_source_sync <= 3'b111;
            debounce_counter <= 8'd0;
            debounce_active <= 1'b0;
        end else begin
            // Combine all reset sources with debounce
            reset_source <= !ext_rst_n || soft_reset || watchdog_reset || !pll_locked;
            
            // Synchronize and debounce
            reset_source_sync <= {reset_source_sync[1:0], reset_source};
            
            // Debounce logic
            if (reset_source_sync[2] != reset_source_sync[1]) begin
                debounce_counter <= DEBOUNCE_CYCLES;
                debounce_active <= 1'b1;
            end else if (debounce_counter > 0) begin
                debounce_counter <= debounce_counter - 1;
                if (debounce_counter == 8'd1) begin
                    debounce_active <= 1'b0;
                end
            end
        end
    end
    
    // ==================== AI DOMAIN RESET ==============
    always @(posedge clk_ai or negedge por_n) begin
        if (!por_n) begin
            rst_sync_ai_ff <= 2'b00;
            hold_counter_ai <= RESET_HOLD_CYCLES;
        end else begin
            // Synchronize reset from AON domain
            rst_sync_ai_ff <= {rst_sync_ai_ff[0], reset_source_sync[2] || debounce_active};
            
            // Hold counter logic
            if (rst_sync_ai_ff[1]) begin
                if (hold_counter_ai > 0) begin
                    hold_counter_ai <= hold_counter_ai - 1;
                end
            end else begin
                hold_counter_ai <= RESET_HOLD_CYCLES;
            end
        end
    end
    
    assign rst_ai_n = !(rst_sync_ai_ff[1] && (hold_counter_ai > 0));
    
    // ==================== AON DOMAIN RESET =============
    always @(posedge clk_aon or negedge por_n) begin
        if (!por_n) begin
            rst_sync_aon_ff <= 2'b00;
            hold_counter_aon <= RESET_HOLD_CYCLES;
        end else begin
            // AON reset (no synchronization needed for external reset)
            rst_sync_aon_ff <= {rst_sync_aon_ff[0], reset_source || !por_n};
            
            // Hold counter logic
            if (rst_sync_aon_ff[1]) begin
                if (hold_counter_aon > 0) begin
                    hold_counter_aon <= hold_counter_aon - 1;
                end
            end else begin
                hold_counter_aon <= RESET_HOLD_CYCLES;
            end
        end
    end
    
    assign rst_aon_n = !(rst_sync_aon_ff[1] && (hold_counter_aon > 0));
    
    // ==================== SENSOR DOMAIN RESET ==========
    always @(posedge clk_sensor or negedge por_n) begin
        if (!por_n) begin
            rst_sync_sensor_ff <= 2'b00;
            hold_counter_sensor <= RESET_HOLD_CYCLES;
        end else begin
            // Synchronize reset from AON domain
            rst_sync_sensor_ff <= {rst_sync_sensor_ff[0], reset_source_sync[2] || debounce_active};
            
            // Hold counter logic
            if (rst_sync_sensor_ff[1]) begin
                if (hold_counter_sensor > 0) begin
                    hold_counter_sensor <= hold_counter_sensor - 1;
                end
            end else begin
                hold_counter_sensor <= RESET_HOLD_CYCLES;
            end
        end
    end
    
    assign rst_sensor_n = !(rst_sync_sensor_ff[1] && (hold_counter_sensor > 0));
    
    // ==================== MCU DOMAIN RESET =============
    always @(posedge clk_mcu or negedge por_n) begin
        if (!por_n) begin
            rst_sync_mcu_ff <= 2'b00;
            hold_counter_mcu <= RESET_HOLD_CYCLES;
        end else begin
            // Synchronize reset from AON domain
            rst_sync_mcu_ff <= {rst_sync_mcu_ff[0], reset_source_sync[2] || debounce_active};
            
            // Hold counter logic
            if (rst_sync_mcu_ff[1]) begin
                if (hold_counter_mcu > 0) begin
                    hold_counter_mcu <= hold_counter_mcu - 1;
                end
            end else begin
                hold_counter_mcu <= RESET_HOLD_CYCLES;
            end
        end
    end
    
    assign rst_mcu_n = !(rst_sync_mcu_ff[1] && (hold_counter_mcu > 0));
    
    // ==================== STATUS LOGIC =================
    assign reset_status_valid = (hold_counter_ai == 0) && 
                               (hold_counter_aon == 0) && 
                               (hold_counter_sensor == 0) && 
                               (hold_counter_mcu == 0) &&
                               !debounce_active;
    
    assign reset_hold_count = {hold_counter_ai[0], hold_counter_aon[0], 
                              hold_counter_sensor[0], hold_counter_mcu[0]};

endmodule