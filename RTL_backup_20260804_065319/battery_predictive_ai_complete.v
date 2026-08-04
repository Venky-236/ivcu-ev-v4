// battery_predictive_ai_complete.v
`timescale 1ns/1ps
`include "defines_ivcu_ev_v3.sv"   // FIX: file is defines_ivcu_ev_v3.sv, .v version never existed

module battery_predictive_ai_complete (
    input  wire        clk_ai,
    input  wire        rst_ai_n,
    input  wire [15:0] batt_cell_temp,
    input  wire [15:0] batt_pack_temp,
    input  wire [15:0] ambient_temp,
    input  wire [15:0] batt_cell_volt,
    input  wire [15:0] batt_pack_volt,
    input  wire [15:0] batt_current,
    input  wire [15:0] charging_volt,
    input  wire [15:0] charging_current,
    input  wire [15:0] soc,
    input  wire [15:0] soh,
    input  wire [9:0]  sensor_valid,
    output reg         battery_health_ok,
    output reg  [7:0]  battery_health_score,
    output reg  [3:0]  battery_status,
    output reg  [7:0]  battery_fault_code,
    output reg  [7:0]  predictive_alerts,
    output reg  [15:0] estimated_range_km,
    output reg         charging_safety_ok
);

    // ==================== PARAMETERS ====================
    localparam [15:0] TEMP_CRITICAL_HIGH = 16'd600;   // 60°C
    localparam [15:0] TEMP_WARNING_HIGH  = 16'd450;   // 45°C
    localparam [15:0] TEMP_NORMAL_HIGH   = 16'd350;   // 35°C
    localparam [15:0] TEMP_NORMAL_LOW    = 16'd0;     // 0°C
    localparam [15:0] TEMP_WARNING_LOW   = 16'd65436; // -10°C (2's complement)
    localparam [15:0] TEMP_CRITICAL_LOW  = 16'd65336; // -20°C (2's complement)

    localparam [15:0] VOLT_CELL_MAX    = 16'd420;     // 4.2V
    localparam [15:0] VOLT_CELL_NOM    = 16'd370;     // 3.7V
    localparam [15:0] VOLT_CELL_MIN    = 16'd300;     // 3.0V
    localparam [15:0] VOLT_PACK_MAX    = 16'd4200;    // 42V for 10S
    localparam [15:0] VOLT_PACK_MIN    = 16'd3000;    // 30V for 10S

    localparam [15:0] CURRENT_CHARGE_MAX     = 16'd500;   // 50A
    localparam [15:0] CURRENT_DISCHARGE_MAX  = 16'd1000;  // 100A
    localparam [15:0] CURRENT_SHORT_CIRCUIT  = 16'd5000;  // 500A

    localparam [15:0] SOC_FULL      = 16'd1000;  // 100%
    localparam [15:0] SOC_HIGH      = 16'd800;   // 80%
    localparam [15:0] SOC_MEDIUM    = 16'd500;   // 50%
    localparam [15:0] SOC_LOW       = 16'd200;   // 20%
    localparam [15:0] SOC_CRITICAL  = 16'd50;    // 5%

    localparam [15:0] SOH_NEW     = 16'd1000;  // 100%
    localparam [15:0] SOH_GOOD    = 16'd800;   // 80%
    localparam [15:0] SOH_WARNING = 16'd600;   // 60%
    localparam [15:0] SOH_BAD     = 16'd400;   // 40%

    // FIXED: increased bit widths to hold value 8 and 32
    localparam [3:0] TEMP_HISTORY_SIZE  = 4'd8;
    localparam [3:0] VOLT_HISTORY_SIZE  = 4'd8;
    localparam [3:0] CURR_HISTORY_SIZE  = 4'd8;
    localparam [5:0] SOC_HISTORY_SIZE   = 6'd32;

    // ==================== INTERNAL REGISTERS ============
    // Temperature monitoring
    reg [15:0] temp_history [0:7];
    reg [2:0]  temp_history_ptr;
    reg [15:0] temp_rise_rate;
    reg [7:0]  temp_score;

    // Voltage monitoring
    reg [15:0] volt_history [0:7];
    reg [2:0]  volt_history_ptr;
    reg [15:0] volt_drop_rate;
    reg [7:0]  volt_score;

    // Current monitoring
    reg [15:0] current_history [0:7];
    reg [2:0]  current_history_ptr;
    reg [15:0] current_avg;
    reg [15:0] current_peak;
    reg [7:0]  current_score;

    // SOC/SOH tracking
    reg [15:0] soc_history [0:31];
    reg [4:0]  soc_history_ptr;
    reg [15:0] soc_drop_rate;
    reg [7:0]  soc_score;
    reg [7:0]  soh_score;

    // Predictive algorithms
    reg [15:0] remaining_capacity;
    reg [15:0] internal_resistance;
    reg [15:0] charge_cycles;
    reg [15:0] thermal_cycles;

    // Health scoring
    reg [7:0]  health_score_temp;
    reg [7:0]  health_score_volt;
    reg [7:0]  health_score_current;
    reg [7:0]  health_score_soc;
    reg [7:0]  health_score_soh;

    // Fault detection
    reg [7:0]  fault_counter_temp;
    reg [7:0]  fault_counter_volt;
    reg [7:0]  fault_counter_current;
    reg [7:0]  fault_counter_soc;

    // Previous values for calculations
    reg [15:0] prev_soc;
    reg [15:0] prev_temp;
    reg [15:0] prev_volt;
    reg [15:0] prev_current;

    // Loop counters

    // ==================== TEMPERATURE MONITORING =======
    always @(posedge clk_ai or negedge rst_ai_n) begin : temp_monitor
        integer i;
        if (!rst_ai_n) begin
            for (i = 0; i < 8; i = i + 1) temp_history[i] <= 16'd0;
            temp_history_ptr <= 3'd0;
            temp_rise_rate <= 16'd0;
            temp_score <= 8'd100;
            fault_counter_temp <= 8'd0;
            prev_temp <= 16'd0;
        end else if (sensor_valid[0] && sensor_valid[1] && sensor_valid[2]) begin
            temp_history[temp_history_ptr] <= batt_pack_temp;
            temp_history_ptr <= temp_history_ptr + 3'd1;
            if (temp_history_ptr == 3'd7) begin
                if (temp_history[7] > temp_history[0])
                    temp_rise_rate <= (temp_history[7] - temp_history[0]) / 16'd8;
                else temp_rise_rate <= 16'd0;
            end
            if ((batt_pack_temp >= TEMP_CRITICAL_HIGH) || 
                ($signed(batt_pack_temp) <= $signed(TEMP_CRITICAL_LOW))) begin
                temp_score <= 8'd0;
                if (fault_counter_temp < 8'd255) fault_counter_temp <= fault_counter_temp + 8'd1;
            end else if ((batt_pack_temp >= TEMP_WARNING_HIGH) || 
                        ($signed(batt_pack_temp) <= $signed(TEMP_WARNING_LOW))) begin
                temp_score <= 8'd50;
                if (fault_counter_temp < 8'd255) fault_counter_temp <= fault_counter_temp + 8'd1;
            end else if (batt_pack_temp >= TEMP_NORMAL_HIGH) begin
                temp_score <= 8'd75;
            end else if (batt_pack_temp >= TEMP_NORMAL_LOW) begin
                temp_score <= 8'd100;
            end else begin
                temp_score <= 8'd25;
            end
            if ((batt_cell_temp > batt_pack_temp) && ((batt_cell_temp - batt_pack_temp) > 16'd50)) begin
                if (temp_score > 8'd20) temp_score <= temp_score - 8'd20;
                else temp_score <= 8'd0;
            end else if ((batt_pack_temp > batt_cell_temp) && ((batt_pack_temp - batt_cell_temp) > 16'd50)) begin
                if (temp_score > 8'd20) temp_score <= temp_score - 8'd20;
                else temp_score <= 8'd0;
            end
            prev_temp <= batt_pack_temp;
        end
    end

    // ==================== VOLTAGE MONITORING ===========
    always @(posedge clk_ai or negedge rst_ai_n) begin : volt_monitor
        integer i;
        if (!rst_ai_n) begin
            for (i = 0; i < 8; i = i + 1) volt_history[i] <= 16'd0;
            volt_history_ptr <= 3'd0;
            volt_drop_rate <= 16'd0;
            volt_score <= 8'd100;
            fault_counter_volt <= 8'd0;
            prev_volt <= 16'd0;
        end else if (sensor_valid[3] && sensor_valid[4]) begin
            volt_history[volt_history_ptr] <= batt_pack_volt;
            volt_history_ptr <= volt_history_ptr + 3'd1;
            if (volt_history_ptr == 3'd7) begin
                if (volt_history[7] < volt_history[0])
                    volt_drop_rate <= (volt_history[0] - volt_history[7]) / 16'd8;
                else volt_drop_rate <= 16'd0;
            end
            if ((batt_pack_volt >= VOLT_PACK_MAX) || (batt_pack_volt <= VOLT_PACK_MIN)) begin
                volt_score <= 8'd0;
                if (fault_counter_volt < 8'd255) fault_counter_volt <= fault_counter_volt + 8'd1;
            end else begin
                volt_score <= ((batt_pack_volt - VOLT_PACK_MIN) * 8'd100) / (VOLT_PACK_MAX - VOLT_PACK_MIN);
            end
            if ((batt_cell_volt > VOLT_CELL_MAX) || (batt_cell_volt < VOLT_CELL_MIN)) begin
                if (volt_score > 8'd30) volt_score <= volt_score - 8'd30;
                else volt_score <= 8'd0;
            end
            prev_volt <= batt_pack_volt;
        end
    end

    // ==================== CURRENT MONITORING ===========
    always @(posedge clk_ai or negedge rst_ai_n) begin : current_monitor
        integer i;
        reg [31:0] curr_sum_tmp;
        if (!rst_ai_n) begin
            for (i = 0; i < 8; i = i + 1) current_history[i] <= 16'd0;
            current_history_ptr <= 3'd0;
            current_avg <= 16'd0;
            current_peak <= 16'd0;
            current_score <= 8'd100;
            fault_counter_current <= 8'd0;
            prev_current <= 16'd0;
        end else if (sensor_valid[5]) begin
            current_history[current_history_ptr] <= batt_current;
            current_history_ptr <= current_history_ptr + 3'd1;
            if (current_history_ptr == 3'd7) begin
                curr_sum_tmp = 32'd0;
                for (i = 0; i < 8; i = i + 1) curr_sum_tmp = curr_sum_tmp + current_history[i];
                current_avg <= curr_sum_tmp / 16'd8;
                current_peak <= 16'd0;
                for (i = 0; i < 8; i = i + 1) if (current_history[i] > current_peak) current_peak <= current_history[i];
            end
            if (batt_current >= CURRENT_SHORT_CIRCUIT) begin
                current_score <= 8'd0;
                fault_counter_current <= 8'd255;
            end else if (batt_current >= CURRENT_DISCHARGE_MAX) begin
                current_score <= 8'd25;
                if (fault_counter_current < 8'd255) fault_counter_current <= fault_counter_current + 8'd2;
            end else if ($signed(batt_current) <= $signed(-CURRENT_CHARGE_MAX)) begin
                current_score <= 8'd25;
                if (fault_counter_current < 8'd255) fault_counter_current <= fault_counter_current + 8'd2;
            end else begin
                current_score <= 8'd100;
                if (fault_counter_current > 8'd0) fault_counter_current <= fault_counter_current - 8'd1;
            end
            if (sensor_valid[6] && sensor_valid[7]) begin
                if (charging_current > CURRENT_CHARGE_MAX) current_score <= 8'd50;
                if (charging_volt > VOLT_PACK_MAX) begin
                    if (current_score > 8'd30) current_score <= current_score - 8'd30;
                    else current_score <= 8'd0;
                end
            end
            prev_current <= batt_current;
        end
    end

    // ==================== SOC/SOH MONITORING ===========
    always @(posedge clk_ai or negedge rst_ai_n) begin : soc_monitor
        integer i;
        if (!rst_ai_n) begin
            for (i = 0; i < 32; i = i + 1) soc_history[i] <= 16'd0;
            soc_history_ptr <= 5'd0;
            soc_drop_rate <= 16'd0;
            soc_score <= 8'd100;
            soh_score <= 8'd100;
            fault_counter_soc <= 8'd0;
            prev_soc <= 16'd0;
        end else if (sensor_valid[8]) begin
            soc_history[soc_history_ptr] <= soc;
            soc_history_ptr <= soc_history_ptr + 5'd1;
            if (soc_history_ptr == 5'd31) begin
                if (soc_history[31] < soc_history[0])
                    soc_drop_rate <= (soc_history[0] - soc_history[31]) / 16'd32;
                else soc_drop_rate <= 16'd0;
            end
            if (soc <= SOC_CRITICAL) begin
                soc_score <= 8'd0;
                if (fault_counter_soc < 8'd255) fault_counter_soc <= fault_counter_soc + 8'd1;
            end else if (soc <= SOC_LOW) begin
                soc_score <= 8'd25;
                if (fault_counter_soc < 8'd255) fault_counter_soc <= fault_counter_soc + 8'd1;
            end else if (soc <= SOC_MEDIUM) begin
                soc_score <= 8'd50;
            end else if (soc <= SOC_HIGH) begin
                soc_score <= 8'd75;
            end else begin
                soc_score <= 8'd100;
            end
            if (sensor_valid[9]) begin
                if (soh <= SOH_BAD) soh_score <= 8'd0;
                else if (soh <= SOH_WARNING) soh_score <= 8'd25;
                else if (soh <= SOH_GOOD) soh_score <= 8'd50;
                else soh_score <= 8'd100;
            end
            prev_soc <= soc;
        end
    end

    // ==================== PREDICTIVE ALGORITHMS ========
    always @(posedge clk_ai or negedge rst_ai_n) begin : predictive
        if (!rst_ai_n) begin
            remaining_capacity <= 16'd0;
            charge_cycles <= 16'd0;
            thermal_cycles <= 16'd0;
            predictive_alerts <= 8'd0;
            estimated_range_km <= 16'd0;
            charging_safety_ok <= 1'b1;
        end else begin
            // Estimate remaining capacity.
            // The 16x16 multiply now happens one cycle earlier, in the
            // remaining_capacity_mul block below. Only the divide-by-constant
            // is left here, so this path is roughly a third of its old depth.
            if (soc_soh_valid) begin
                remaining_capacity <= soc_soh_prod / 32'd1000;
            end

            // Internal resistance moved OUT of this block -- see the
            // internal_resistance_calc block near the end of this file. It now
            // runs on a sequential divider. Any assignment to
            // internal_resistance left here would create a second driver.

            // Count charge cycles (simplified)
            if ((soc >= 16'd950) && (prev_soc < 16'd200)) begin
                if (charge_cycles < 16'hFFFF) charge_cycles <= charge_cycles + 16'd1;
            end

            // Count thermal cycles
            if ((batt_pack_temp > (prev_temp + 16'd100)) && (prev_temp != 16'd0)) begin
                if (thermal_cycles < 16'hFFFF) thermal_cycles <= thermal_cycles + 16'd1;
            end

            // Generate predictive alerts
            predictive_alerts <= 8'd0;

            if (temp_rise_rate > 16'd50) predictive_alerts[0] <= 1'b1;
            if (volt_drop_rate > 16'd100) predictive_alerts[1] <= 1'b1;
            if (internal_resistance > 16'd100) predictive_alerts[2] <= 1'b1;
            if (charge_cycles > 16'd1000) predictive_alerts[3] <= 1'b1;
            if (thermal_cycles > 16'd5000) predictive_alerts[4] <= 1'b1;
            if ((batt_cell_volt * 16'd10) < (batt_pack_volt / 16'd10)) predictive_alerts[5] <= 1'b1;

            // Estimate remaining range
            if (remaining_capacity > 16'd0) begin
                estimated_range_km <= (remaining_capacity * 8'd90) / 16'd150;
            end

            // Charging safety check
            charging_safety_ok <= 1'b1;
            if (sensor_valid[6] && sensor_valid[7]) begin
                if ((batt_pack_temp >= TEMP_CRITICAL_HIGH) || 
                    ($signed(batt_pack_temp) <= $signed(TEMP_CRITICAL_LOW))) begin
                    charging_safety_ok <= 1'b0;
                end
                if (batt_pack_volt >= VOLT_PACK_MAX) charging_safety_ok <= 1'b0;
                if ((charging_current > 16'd0) && (charging_volt < batt_pack_volt)) charging_safety_ok <= 1'b0;
            end
        end
    end

    // ==================== HEALTH SCORE CALCULATION =====
    always @(posedge clk_ai or negedge rst_ai_n) begin : health_score
        if (!rst_ai_n) begin
            health_score_temp <= 8'd100;
            health_score_volt <= 8'd100;
            health_score_current <= 8'd100;
            health_score_soc <= 8'd100;
            health_score_soh <= 8'd100;

            battery_health_score <= 8'd100;
            battery_health_ok <= 1'b1;
            battery_status <= `STATUS_OK;
            battery_fault_code <= `FAULT_NONE;
        end else begin
            health_score_temp <= temp_score;
            health_score_volt <= volt_score;
            health_score_current <= current_score;
            health_score_soc <= soc_score;
            health_score_soh <= soh_score;

            battery_health_score <= (health_score_temp * 8'd2 + 
                                    health_score_volt * 8'd2 + 
                                    health_score_current * 8'd2 + 
                                    health_score_soc * 8'd2 + 
                                    health_score_soh * 8'd2) / 8'd10;

            if (battery_health_score >= 8'd80) begin
                battery_status <= `STATUS_OK;
                battery_health_ok <= 1'b1;
            end else if (battery_health_score >= 8'd50) begin
                battery_status <= `STATUS_WARNING;
                battery_health_ok <= 1'b1;
            end else if (battery_health_score >= 8'd20) begin
                battery_status <= `STATUS_CRITICAL;
                battery_health_ok <= 1'b0;
            end else begin
                battery_status <= `STATUS_FAULT;
                battery_health_ok <= 1'b0;
            end

            if (fault_counter_temp >= 8'd200) battery_fault_code <= `FAULT_BATTERY_TEMP;
            else if (fault_counter_volt >= 8'd200) battery_fault_code <= `FAULT_BATTERY_VOLT;
            else if (fault_counter_current >= 8'd200) battery_fault_code <= `FAULT_BATTERY_TEMP;
            else if (fault_counter_soc >= 8'd200) battery_fault_code <= `FAULT_BATTERY_VOLT;
            else battery_fault_code <= `FAULT_NONE;

            if (batt_pack_temp >= TEMP_CRITICAL_HIGH) begin
                battery_status <= `STATUS_EMERGENCY;
                battery_health_ok <= 1'b0;
                battery_fault_code <= `FAULT_BATTERY_TEMP;
            end else if ((batt_pack_volt >= VOLT_PACK_MAX) || (batt_pack_volt <= VOLT_PACK_MIN)) begin
                battery_status <= `STATUS_EMERGENCY;
                battery_health_ok <= 1'b0;
                battery_fault_code <= `FAULT_BATTERY_VOLT;
            end
        end
    end

    // ======================================================================
    // REMAINING CAPACITY, stage 1: the 16x16 variable multiply, registered.
    //
    // WHY: soc * soh is a full 16x16 variable multiplier, roughly 30-40 logic
    // levels, and it sat in the same 10 ns cycle as a divide. Splitting it
    // across two cycles costs one register and one cycle of latency on a
    // quantity that changes over minutes.
    //
    // ALSO A BUG FIX, AND THIS ONE CHANGES NUMBERS:
    //   The original was   remaining_capacity <= (soc * soh) / 16'd1000;
    //   Verilog sizes that expression to the width of the left-hand side, so
    //   soc*soh was computed in 16 bits and OVERFLOWED. With soc = soh = 1000
    //   (both full scale) the true product is 1,000,000, but 16 bits wrap it to
    //   16,960, giving remaining_capacity = 16 instead of 1000.
    //   Computing the product in 32 bits first gives the intended answer.
    // ======================================================================
    reg [31:0] soc_soh_prod;
    reg        soc_soh_valid;

    always @(posedge clk_ai or negedge rst_ai_n) begin : remaining_capacity_mul
        if (!rst_ai_n) begin
            soc_soh_prod  <= 32'd0;
            soc_soh_valid <= 1'b0;
        end else begin
            soc_soh_prod  <= {16'd0, soc} * {16'd0, soh};
            soc_soh_valid <= sensor_valid[8] && sensor_valid[9];
        end
    end

    // ======================================================================
    // INTERNAL RESISTANCE,  R = dV / I,  on a sequential divider.
    //
    // Was a fully combinational 128-gate shift-subtract chain. Masked STA
    // measured 35.170 ns arrival against a 10 ns clock: worst slack -25.548 ns.
    // Cell resizing cannot recover 25 ns and there is no wire delay yet to
    // remove, so the chain had to become sequential.
    //
    // Unlike the sensor fabric, current_avg is a genuine runtime value -- no
    // shift trick applies here.
    //
    // Latency is now 32 clocks (320 ns at 100 MHz) instead of 1. Battery
    // internal resistance changes over minutes, so this is immaterial.
    //
    // seq_divider latches its operands on start, so nothing here has to hold
    // them stable across the 32 cycles.
    // ======================================================================
    wire [15:0] ir_delta = volt_history[0] - batt_pack_volt;

    // Zero-extend before the constant *10 so it cannot overflow 16 bits.
    wire [31:0] ir_num   = {16'd0, ir_delta} * 32'd10;
    wire [31:0] ir_den   = {16'd0, current_avg};

    // Exactly the guard conditions the original code used.
    wire        ir_cond  = (current_avg     > 16'd100)
                        && (volt_history[0] > 16'd0)
                        && (volt_history[0] > batt_pack_volt);

    wire [31:0] ir_quot;
    wire        ir_busy;
    wire        ir_done;

    seq_divider #(.WIDTH(32)) u_div_ir (
        .clk       (clk_ai),
        .rst_n     (rst_ai_n),
        .start     (ir_cond && !ir_busy),
        .dividend  (ir_num),
        .divisor   (ir_den),
        .quotient  (ir_quot),
        .remainder (),
        .busy      (ir_busy),
        .done      (ir_done)
    );

    // SOLE driver of internal_resistance.
    always @(posedge clk_ai or negedge rst_ai_n) begin : internal_resistance_calc
        if (!rst_ai_n)    internal_resistance <= 16'd0;
        else if (ir_done) internal_resistance <= ir_quot[15:0];
    end

endmodule