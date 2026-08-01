// diagnostic_report_generator.v
`timescale 1ns/1ps
`include "defines_ivcu_ev_v3.sv"   // FIX: file is defines_ivcu_ev_v3.sv, .v version never existed

module diagnostic_report_generator (
    input  wire        clk_ai,
    input  wire        rst_ai_n,
    input  wire [7:0]  system_health_score,
    input  wire [7:0]  battery_score,
    input  wire [7:0]  motor_score,
    input  wire [7:0]  thermal_score,
    input  wire [7:0]  dynamics_score,
    input  wire [7:0]  perception_score,
    input  wire [41:0] sensor_fault,
    input  wire [5:0]  ai_status,
    input  wire [1:0]  current_mode,
    input  wire [31:0] vehicle_odometer,
    input  wire [31:0] motor_runtime,
    output reg  [127:0] maintenance_report,
    output reg  [63:0]  battery_life_prediction,
    output reg  [63:0]  motor_life_prediction,
    output reg  [95:0]  component_wear_level,
    output reg         report_ready,
    output reg  [31:0] report_data,
    output reg         report_valid,
    input  wire        generate_report,
    input  wire        continuous_monitoring,
    input  wire [2:0]  report_detail_level
);

    // ==================== PARAMETERS ====================
    localparam [2:0] REPORT_SUMMARY     = 3'b000;
    localparam [2:0] REPORT_DETAILED    = 3'b001;
    localparam [2:0] REPORT_MAINTENANCE = 3'b010;
    localparam [2:0] REPORT_PREDICTIVE  = 3'b011;
    localparam [2:0] REPORT_DEBUG       = 3'b100;

    localparam [7:0] HEALTH_EXCELLENT = 8'd90;
    localparam [7:0] HEALTH_GOOD      = 8'd70;
    localparam [7:0] HEALTH_FAIR      = 8'd50;
    localparam [7:0] HEALTH_POOR      = 8'd30;
    localparam [7:0] HEALTH_CRITICAL  = 8'd10;

    localparam [31:0] MAINT_TIRE_ROTATION = 32'd8000;
    localparam [31:0] MAINT_BRAKE_CHECK   = 32'd15000;
    localparam [31:0] MAINT_BATTERY_CHECK = 32'd20000;
    localparam [31:0] MAINT_MOTOR_SERVICE = 32'd50000;
    localparam [31:0] MAINT_COOLING_FLUSH = 32'd100000;

    localparam [31:0] LIFE_BATTERY_TYPICAL  = 32'd160000;
    localparam [31:0] LIFE_MOTOR_TYPICAL    = 32'd320000;
    localparam [31:0] LIFE_BEARINGS_TYPICAL = 32'd240000;
    localparam [31:0] LIFE_COOLING_TYPICAL  = 32'd200000;

    localparam [7:0] WEAR_NORMAL      = 8'd50;
    localparam [7:0] WEAR_ACCELERATED = 8'd75;
    localparam [7:0] WEAR_SEVERE      = 8'd90;

    localparam [2:0] STATE_IDLE         = 3'b000;
    localparam [2:0] STATE_COLLECT_DATA = 3'b001;
    localparam [2:0] STATE_ANALYZE      = 3'b010;
    localparam [2:0] STATE_GENERATE     = 3'b011;
    localparam [2:0] STATE_OUTPUT       = 3'b100;
    localparam [2:0] STATE_UPDATE       = 3'b101;

    // ==================== INTERNAL REGISTERS ============
    reg [2:0] report_state;

    reg [7:0] health_history [0:31];
    reg [4:0] health_history_ptr;
    reg [7:0] health_trend;

    reg [15:0] fault_history [0:15];
    reg [3:0]  fault_history_ptr;
    reg [7:0]  fault_frequency;

    reg [7:0]  overall_health;
    reg [7:0]  health_stability;
    reg [7:0]  fault_severity;
    reg [15:0] maintenance_priority;

    reg [31:0] battery_life_remaining;
    reg [31:0] motor_life_remaining;
    reg [31:0] predicted_failure_km;
    reg [7:0]  reliability_score;

    reg [7:0]  battery_wear;
    reg [7:0]  motor_wear;
    reg [7:0]  bearing_wear;
    reg [7:0]  cooling_wear;
    reg [7:0]  brake_wear;
    reg [7:0]  tire_wear;

    reg [31:0] next_maintenance_km;
    reg [7:0]  maintenance_urgency;
    reg [15:0] maintenance_items;

    reg [127:0] report_buffer [0:7];
    reg [2:0]   report_buffer_ptr;
    reg [7:0]   report_sequence;

    reg [31:0] report_timer;
    reg [31:0] monitoring_timer;
    reg [7:0]  report_interval;
    reg        continuous_update;

    reg [31:0] total_runtime;
    reg [31:0] total_distance;
    reg [15:0] fault_count_total;
    reg [15:0] fault_count_critical;

    // Temporary variables (declared at module level)
    // Each of these is written by exactly ONE always block, so none of them
    // conflict. Only `i` below was shared across blocks -- see the FIX note.
    reg [5:0] fault_count;                  // data_collection only
    reg [15:0] health_sum;                  // health_analysis only
    reg [31:0] health_sq_sum;               // health_analysis only
    reg [15:0] health_mean;                 // health_analysis only
    reg [15:0] health_variance;             // health_analysis only
    reg [15:0] fault_sum;                   // data_collection only
    reg [31:0] next_km;                     // maintenance_schedule only
    reg [15:0] total_weighted_score;
    reg [7:0]  total_weight;
    reg [15:0] failure_risk;
    reg [31:0] base_life;

    // FIX (multiple-driver): `integer i;` was declared here at module scope and
    // written from data_collection, health_analysis and report_fsm. Each block
    // inferred its own flip-flop and all three drove the same net -- Yosys
    // `check` reported "multiple conflicting drivers". It is now declared inside
    // each of the three named blocks that use it, which Verilog-2001 permits for
    // a labelled begin/end. No other change.

    // ==================== DATA COLLECTION ==============
    always @(posedge clk_ai or negedge rst_ai_n) begin : data_collection
        integer i;
        if (!rst_ai_n) begin
            for (i = 0; i < 32; i = i + 1) health_history[i] <= 8'd100;
            health_history_ptr <= 5'd0;
            health_trend <= 8'd0;

            for (i = 0; i < 16; i = i + 1) fault_history[i] <= 16'd0;
            fault_history_ptr <= 4'd0;
            fault_frequency <= 8'd0;

            total_runtime <= 32'd0;
            total_distance <= 32'd0;
            fault_count_total <= 16'd0;
            fault_count_critical <= 16'd0;
        end else begin
            health_history[health_history_ptr] <= system_health_score;
            health_history_ptr <= health_history_ptr + 5'd1;

            if (health_history_ptr == 5'd31) begin
                if (health_history[31] > health_history[0])
                    health_trend <= ((health_history[31] - health_history[0]) * 8'd100) / 8'd32;
                else health_trend <= 8'd0;
            end

            fault_count = 6'd0;
            for (i = 0; i < 42; i = i + 1) if (sensor_fault[i]) fault_count = fault_count + 6'd1;

            fault_history[fault_history_ptr] <= {10'b0, fault_count};
            fault_history_ptr <= fault_history_ptr + 4'd1;

            if (fault_history_ptr == 4'd15) begin
                fault_sum = 16'd0;
                for (i = 0; i < 16; i = i + 1) fault_sum = fault_sum + fault_history[i];
                fault_frequency <= (fault_sum * 8'd100) / (16 * 42);
            end

            total_runtime <= motor_runtime;
            total_distance <= vehicle_odometer;

            if (fault_count > 6'd0) begin
                fault_count_total <= fault_count_total + fault_count;
                if (system_health_score < HEALTH_POOR) fault_count_critical <= fault_count_critical + 16'd1;
            end
        end
    end

    // ==================== HEALTH ANALYSIS ==============
    always @(posedge clk_ai or negedge rst_ai_n) begin : health_analysis
        integer i;
        if (!rst_ai_n) begin
            overall_health <= 8'd100;
            health_stability <= 8'd100;
            fault_severity <= 8'd0;
            maintenance_priority <= 16'd0;
        end else begin
            overall_health <= system_health_score;

            if (health_history_ptr == 5'd31) begin
                health_sum = 16'd0;
                health_sq_sum = 32'd0;
                for (i = 0; i < 32; i = i + 1) begin
                    health_sum = health_sum + health_history[i];
                    health_sq_sum = health_sq_sum + (health_history[i] * health_history[i]);
                end
                health_mean = health_sum / 16'd32;
                health_variance = (health_sq_sum / 32) - (health_mean * health_mean);

                if (health_variance == 16'd0) health_stability <= 8'd100;
                else if (health_variance < 16'd100) health_stability <= 8'd90;
                else if (health_variance < 16'd400) health_stability <= 8'd70;
                else if (health_variance < 16'd900) health_stability <= 8'd50;
                else health_stability <= 8'd30;
            end

            fault_severity <= fault_frequency;
            if (fault_count_critical > 16'd10) fault_severity <= 8'd100;

            maintenance_priority <= 16'd0;
            if (battery_score < HEALTH_POOR) maintenance_priority[0] <= 1'b1;
            if (motor_score < HEALTH_POOR) maintenance_priority[1] <= 1'b1;
            if (thermal_score < HEALTH_POOR) maintenance_priority[2] <= 1'b1;
            if (dynamics_score < HEALTH_POOR) maintenance_priority[3] <= 1'b1;
            if (perception_score < HEALTH_POOR) maintenance_priority[4] <= 1'b1;
            if (fault_severity > 8'd50) maintenance_priority[5] <= 1'b1;
        end
    end

    // ==================== WEAR CALCULATION =============
    always @(posedge clk_ai or negedge rst_ai_n) begin : wear_calc
        if (!rst_ai_n) begin
            battery_wear <= 8'd0;
            motor_wear <= 8'd0;
            bearing_wear <= 8'd0;
            cooling_wear <= 8'd0;
            brake_wear <= 8'd0;
            tire_wear <= 8'd0;
            component_wear_level <= 96'd0;
        end else begin
            battery_wear <= 8'd100 - battery_score;
            motor_wear <= 8'd100 - motor_score;
            bearing_wear <= motor_runtime[23:16];
            if (bearing_wear > 8'd100) bearing_wear <= 8'd100;
            cooling_wear <= 8'd100 - thermal_score;
            brake_wear <= vehicle_odometer[23:16];
            if (brake_wear > 8'd100) brake_wear <= 8'd100;
            tire_wear <= vehicle_odometer[23:16];
            if (tire_wear > 8'd100) tire_wear <= 8'd100;
            component_wear_level <= {battery_wear, motor_wear, bearing_wear, cooling_wear, brake_wear, tire_wear, 32'd0};
        end
    end

    // ==================== PREDICTIVE MODELS ============
    always @(posedge clk_ai or negedge rst_ai_n) begin : predictive_models
        if (!rst_ai_n) begin
            battery_life_remaining <= LIFE_BATTERY_TYPICAL;
            motor_life_remaining <= LIFE_MOTOR_TYPICAL;
            predicted_failure_km <= 32'd0;
            reliability_score <= 8'd100;
            battery_life_prediction <= 64'd0;
            motor_life_prediction <= 64'd0;
        end else begin
            battery_life_remaining <= (LIFE_BATTERY_TYPICAL * (8'd100 - battery_wear)) / 8'd100;
            if ($signed(health_trend) < 8'sd0) battery_life_remaining <= (battery_life_remaining * 8'd9) / 8'd10;

            motor_life_remaining <= (LIFE_MOTOR_TYPICAL * (8'd100 - motor_wear)) / 8'd100;
            if (thermal_score < HEALTH_FAIR) motor_life_remaining <= (motor_life_remaining * 8'd8) / 8'd10;

            if (battery_life_remaining < motor_life_remaining) predicted_failure_km <= battery_life_remaining;
            else predicted_failure_km <= motor_life_remaining;
            predicted_failure_km <= predicted_failure_km + vehicle_odometer;

            reliability_score <= (overall_health + health_stability) / 8'd2;
            if (fault_severity > 8'd30) begin
                if (reliability_score > 8'd20) reliability_score <= reliability_score - 8'd20;
                else reliability_score <= 8'd0;
            end

            battery_life_prediction <= {vehicle_odometer + battery_life_remaining, battery_life_remaining};
            motor_life_prediction <= {vehicle_odometer + motor_life_remaining, motor_life_remaining};
        end
    end

    // ==================== MAINTENANCE SCHEDULE =========
    always @(posedge clk_ai or negedge rst_ai_n) begin : maintenance_schedule
        if (!rst_ai_n) begin
            next_maintenance_km <= MAINT_TIRE_ROTATION;
            maintenance_urgency <= 8'd0;
            maintenance_items <= 16'd0;
            maintenance_report <= 128'd0;
        end else begin
            // Use blocking assignments for temporary next_km
            next_km = MAINT_TIRE_ROTATION;
            maintenance_items <= 16'd0;

            if (vehicle_odometer >= next_km) begin
                maintenance_items[0] <= 1'b1;
                next_km = next_km + MAINT_TIRE_ROTATION;
            end

            if (vehicle_odometer >= MAINT_BRAKE_CHECK) begin
                maintenance_items[1] <= 1'b1;
                if (next_km < (MAINT_BRAKE_CHECK + MAINT_BRAKE_CHECK))
                    next_km = MAINT_BRAKE_CHECK + MAINT_BRAKE_CHECK;
            end

            if (vehicle_odometer >= MAINT_BATTERY_CHECK) begin
                maintenance_items[2] <= 1'b1;
                if (next_km < (MAINT_BATTERY_CHECK + MAINT_BATTERY_CHECK))
                    next_km = MAINT_BATTERY_CHECK + MAINT_BATTERY_CHECK;
            end

            if (vehicle_odometer >= MAINT_MOTOR_SERVICE) begin
                maintenance_items[3] <= 1'b1;
                if (next_km < (MAINT_MOTOR_SERVICE + MAINT_MOTOR_SERVICE))
                    next_km = MAINT_MOTOR_SERVICE + MAINT_MOTOR_SERVICE;
            end

            if (vehicle_odometer >= MAINT_COOLING_FLUSH) begin
                maintenance_items[4] <= 1'b1;
                if (next_km < (MAINT_COOLING_FLUSH + MAINT_COOLING_FLUSH))
                    next_km = MAINT_COOLING_FLUSH + MAINT_COOLING_FLUSH;
            end

            if (battery_wear > WEAR_SEVERE) begin
                maintenance_items[5] <= 1'b1;
                maintenance_urgency <= 8'd100;
            end else if (motor_wear > WEAR_SEVERE) begin
                maintenance_items[6] <= 1'b1;
                maintenance_urgency <= 8'd100;
            end else if (bearing_wear > WEAR_SEVERE) begin
                maintenance_items[7] <= 1'b1;
                maintenance_urgency <= 8'd100;
            end else if (maintenance_items != 16'd0) begin
                if ((next_km - vehicle_odometer) < 32'd1000) maintenance_urgency <= 8'd90;
                else if ((next_km - vehicle_odometer) < 32'd5000) maintenance_urgency <= 8'd70;
                else if ((next_km - vehicle_odometer) < 32'd10000) maintenance_urgency <= 8'd50;
                else maintenance_urgency <= 8'd30;
            end else maintenance_urgency <= 8'd0;

            next_maintenance_km <= next_km;
            maintenance_report <= {next_maintenance_km, maintenance_urgency, maintenance_items, vehicle_odometer, 16'd0};
        end
    end

    // ==================== REPORT GENERATION FSM ========
    always @(posedge clk_ai or negedge rst_ai_n) begin : report_fsm
        integer i;
        if (!rst_ai_n) begin
            report_state <= STATE_IDLE;
            report_timer <= 32'd0;
            monitoring_timer <= 32'd0;
            report_interval <= 8'd100;
            continuous_update <= 1'b0;
            report_ready <= 1'b0;
            report_data <= 32'd0;
            report_valid <= 1'b0;
            report_sequence <= 8'd0;
            for (i = 0; i < 8; i = i + 1) report_buffer[i] <= 128'd0;
            report_buffer_ptr <= 3'd0;
        end else begin
            if (report_timer < 32'hFFFFFFFF) report_timer <= report_timer + 32'd1;
            if (monitoring_timer < 32'hFFFFFFFF) monitoring_timer <= monitoring_timer + 32'd1;
            continuous_update <= continuous_monitoring;

            case (report_state)
                STATE_IDLE: begin
                    report_ready <= 1'b0;
                    report_valid <= 1'b0;
                    if (generate_report || (continuous_update && (monitoring_timer >= {24'b0, report_interval}))) begin
                        report_state <= STATE_COLLECT_DATA;
                        monitoring_timer <= 32'd0;
                    end
                end

                STATE_COLLECT_DATA: begin
                    report_buffer[0] <= {system_health_score, battery_score, motor_score,
                                        thermal_score, dynamics_score, perception_score,
                                        health_stability, fault_severity};
                    report_buffer[1] <= maintenance_report;
                    report_buffer[2] <= {battery_life_prediction[63:32], 32'd0};
                    report_buffer[3] <= {battery_life_prediction[31:0], motor_life_prediction[63:32]};
                    report_buffer[4] <= {motor_life_prediction[31:0], component_wear_level[95:64]};
                    report_buffer[5] <= {component_wear_level[63:0], 64'd0};
                    report_buffer[6] <= {total_runtime, total_distance};
                    report_buffer[7] <= {fault_count_total, fault_count_critical, reliability_score, 8'd0, 32'd0};
                    report_state <= STATE_ANALYZE;
                end

                STATE_ANALYZE: report_state <= STATE_GENERATE;

                STATE_GENERATE: begin
                    report_buffer_ptr <= 3'd0;
                    report_sequence <= 8'd0;
                    report_state <= STATE_OUTPUT;
                end

                STATE_OUTPUT: begin
                    report_ready <= 1'b1;
                    case (report_detail_level)
                        REPORT_SUMMARY: begin
                            report_data <= {system_health_score, reliability_score, maintenance_urgency, 8'd0};
                            report_valid <= 1'b1;
                            report_state <= STATE_UPDATE;
                        end

                        REPORT_DETAILED: begin
                            if (report_sequence < 8'd8) begin
                                report_data <= report_buffer[report_buffer_ptr][31:0];
                                report_buffer_ptr <= report_buffer_ptr + 3'd1;
                                report_sequence <= report_sequence + 8'd1;
                                report_valid <= 1'b1;
                            end else report_state <= STATE_UPDATE;
                        end

                        REPORT_MAINTENANCE: begin
                            if (report_sequence < 8'd4) begin
                                case (report_sequence)
                                    8'd0: report_data <= maintenance_report[31:0];
                                    8'd1: report_data <= maintenance_report[63:32];
                                    8'd2: report_data <= maintenance_report[95:64];
                                    8'd3: report_data <= maintenance_report[127:96];
                                endcase
                                report_sequence <= report_sequence + 8'd1;
                                report_valid <= 1'b1;
                            end else report_state <= STATE_UPDATE;
                        end

                        REPORT_PREDICTIVE: begin
                            if (report_sequence < 8'd4) begin
                                case (report_sequence)
                                    8'd0: report_data <= battery_life_prediction[31:0];
                                    8'd1: report_data <= battery_life_prediction[63:32];
                                    8'd2: report_data <= motor_life_prediction[31:0];
                                    8'd3: report_data <= motor_life_prediction[63:32];
                                endcase
                                report_sequence <= report_sequence + 8'd1;
                                report_valid <= 1'b1;
                            end else report_state <= STATE_UPDATE;
                        end

                        REPORT_DEBUG: begin
                            if (report_sequence < 8'd16) begin
                                if (report_sequence < 8'd8) report_data <= report_buffer[report_sequence][31:0];
                                else report_data <= report_buffer[report_sequence - 8'd8][63:32];
                                report_sequence <= report_sequence + 8'd1;
                                report_valid <= 1'b1;
                            end else report_state <= STATE_UPDATE;
                        end

                        default: report_state <= STATE_UPDATE;
                    endcase
                end

                STATE_UPDATE: begin
                    report_valid <= 1'b0;
                    report_ready <= 1'b0;
                    report_state <= STATE_IDLE;
                end

                default: report_state <= STATE_IDLE;
            endcase
        end
    end

endmodule
