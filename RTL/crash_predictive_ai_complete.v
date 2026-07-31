// crash_predictive_ai_complete.v
`timescale 1ns/1ps
`include "defines_ivcu_ev_v3.sv"   // FIX: file is defines_ivcu_ev_v3.sv, .v version never existed

module crash_predictive_ai_complete (
    input  wire        clk_aon,
    input  wire        rst_aon_n,
    input  wire [15:0] crash_impact,
    input  wire [15:0] gyroscope,
    input  wire [15:0] imu,
    input  wire [2:0]  sensor_valid,
    output reg         crash_latched,
    output reg  [7:0]  crash_severity,
    output reg  [2:0]  crash_direction,
    output reg         pre_crash_warning,
    output reg         multi_impact_detect,
    input  wire        crash_reset,
    input  wire        test_mode
);

    // ==================== PARAMETERS ====================
    localparam [15:0] IMPACT_MINOR     = 16'd50;   // 5G
    localparam [15:0] IMPACT_MODERATE  = 16'd100;  // 10G
    localparam [15:0] IMPACT_SEVERE    = 16'd200;  // 20G
    localparam [15:0] IMPACT_CRITICAL  = 16'd300;  // 30G

    localparam [15:0] GYRO_ROLLOVER    = 16'd5000;  // 500°/s

    localparam [15:0] ACCEL_CRASH      = 16'd1500;  // 15G
    localparam [15:0] ACCEL_WARNING    = 16'd800;   // 8G

    localparam [31:0] CRASH_WINDOW        = 32'd100000;   // 10ms
    localparam [31:0] RESET_HOLD          = 32'd10000000; // 1 second
    localparam [31:0] MULTI_IMPACT_WINDOW = 32'd5000000;  // 500ms

    localparam [2:0] DIR_FRONT = 3'b001;
    localparam [2:0] DIR_REAR  = 3'b010;
    localparam [2:0] DIR_LEFT  = 3'b011;
    localparam [2:0] DIR_RIGHT = 3'b100;
    localparam [2:0] DIR_ROLL  = 3'b101;

    localparam [2:0] CRASH_IDLE      = 3'b000;
    localparam [2:0] CRASH_DETECT    = 3'b001;
    localparam [2:0] CRASH_CONFIRM   = 3'b010;
    localparam [2:0] CRASH_LATCHED   = 3'b011;
    localparam [2:0] CRASH_RESETTING = 3'b100;

    // ==================== INTERNAL REGISTERS ============
    reg [2:0] crash_state;
    reg [2:0] next_crash_state;

    reg [15:0] impact_history [0:7];
    reg [2:0]  impact_ptr;
    reg [15:0] impact_derivative;

    reg [15:0] gyro_history [0:7];
    reg [2:0]  gyro_ptr;

    reg [15:0] imu_history [0:7];
    reg [2:0]  imu_ptr;

    reg [31:0] crash_timer;
    reg [31:0] reset_timer;
    reg [31:0] multi_impact_timer;
    reg [7:0]  impact_count;

    reg [7:0]  impact_severity;
    reg [7:0]  gyro_severity;
    reg [7:0]  imu_severity;
    reg [7:0]  combined_severity;

    reg [15:0] imu_x;
    reg [15:0] imu_y;
    reg [15:0] imu_z;
    reg [15:0] gyro_roll;
    reg [15:0] gyro_pitch;
    reg [15:0] gyro_yaw;

    reg [15:0] pre_crash_accel;
    reg [15:0] pre_crash_gyro;
    reg [7:0]  crash_probability;

    reg [7:0]  crash_severity_latched;
    reg [2:0]  crash_direction_latched;
    reg        multi_impact_latched;

    integer i;
    // Temporary computation regs (moved from inline block-local declarations)
    reg [15:0] crash_max_accel;
    reg [15:0] crash_accel_change;
    reg [15:0] crash_gyro_change;

    // ==================== SENSOR PARSING ===============
    always @(posedge clk_aon or negedge rst_aon_n) begin : sensor_parse
        if (!rst_aon_n) begin
            for (i = 0; i < 8; i = i + 1) begin
                impact_history[i] <= 16'd0;
                gyro_history[i] <= 16'd0;
                imu_history[i] <= 16'd0;
            end
            impact_ptr <= 3'd0;
            gyro_ptr <= 3'd0;
            imu_ptr <= 3'd0;

            imu_x <= 16'd0;
            imu_y <= 16'd0;
            imu_z <= 16'd0;
            gyro_roll <= 16'd0;
            gyro_pitch <= 16'd0;
            gyro_yaw <= 16'd0;
        end else begin
            // Parse IMU data (assume format: [15:12]=X, [11:8]=Y, [7:4]=Z)
            if (sensor_valid[0]) begin
                imu_x <= {12'b0, imu[15:12], 4'b0};
                imu_y <= {12'b0, imu[11:8], 4'b0};
                imu_z <= {12'b0, imu[7:4], 4'b0};

                imu_history[imu_ptr] <= imu;
                imu_ptr <= imu_ptr + 3'd1;
            end

            // Parse gyroscope data (assume format: [15:12]=roll, [11:8]=pitch, [7:4]=yaw)
            if (sensor_valid[1]) begin
                gyro_roll <= {12'b0, gyroscope[15:12], 4'b0};
                gyro_pitch <= {12'b0, gyroscope[11:8], 4'b0};
                gyro_yaw <= {12'b0, gyroscope[7:4], 4'b0};

                gyro_history[gyro_ptr] <= gyroscope;
                gyro_ptr <= gyro_ptr + 3'd1;
            end

            // Parse impact sensor
            if (sensor_valid[2]) begin
                impact_history[impact_ptr] <= crash_impact;
                impact_ptr <= impact_ptr + 3'd1;

                // Calculate impact derivative (rate of change)
                if (impact_ptr == 3'd7) begin
                    if (crash_impact > impact_history[0]) begin
                        impact_derivative <= ((crash_impact - impact_history[0]) * 16'd10) / 16'd8;
                    end else begin
                        impact_derivative <= ((impact_history[0] - crash_impact) * 16'd10) / 16'd8;
                    end
                end
            end
        end
    end

    // ==================== SEVERITY CALCULATION =========
    always @(posedge clk_aon or negedge rst_aon_n) begin : severity_calc
        if (!rst_aon_n) begin
            impact_severity <= 8'd0;
            gyro_severity <= 8'd0;
            imu_severity <= 8'd0;
            combined_severity <= 8'd0;
        end else begin
            // Impact severity
            if (crash_impact >= IMPACT_CRITICAL) impact_severity <= 8'd100;
            else if (crash_impact >= IMPACT_SEVERE) impact_severity <= 8'd75;
            else if (crash_impact >= IMPACT_MODERATE) impact_severity <= 8'd50;
            else if (crash_impact >= IMPACT_MINOR) impact_severity <= 8'd25;
            else impact_severity <= 8'd0;

            // Gyroscope severity (for rollover)
            if (gyro_roll >= GYRO_ROLLOVER) gyro_severity <= 8'd100;
            else if (gyro_roll >= (GYRO_ROLLOVER * 3 / 4)) gyro_severity <= 8'd75;
            else if (gyro_roll >= (GYRO_ROLLOVER / 2)) gyro_severity <= 8'd50;
            else if (gyro_roll >= (GYRO_ROLLOVER / 4)) gyro_severity <= 8'd25;
            else gyro_severity <= 8'd0;

            // IMU severity
            crash_max_accel = imu_x;
            if (imu_y > crash_max_accel) crash_max_accel = imu_y;
            if (imu_z > crash_max_accel) crash_max_accel = imu_z;

            if (crash_max_accel >= ACCEL_CRASH) imu_severity <= 8'd100;
            else if (crash_max_accel >= ACCEL_WARNING) imu_severity <= 8'd50;
            else imu_severity <= 8'd0;

            // Combined severity (worst of all)
            if ((impact_severity >= gyro_severity) && (impact_severity >= imu_severity))
                combined_severity <= impact_severity;
            else if (gyro_severity >= imu_severity) combined_severity <= gyro_severity;
            else combined_severity <= imu_severity;
        end
    end

    // ==================== DIRECTION DETECTION ==========
    always @(posedge clk_aon or negedge rst_aon_n) begin : direction_detect
        if (!rst_aon_n) begin
            crash_direction_latched <= 3'b000;
        end else if (crash_state == CRASH_CONFIRM) begin
            // Determine crash direction based on sensor data
            if (gyro_roll >= GYRO_ROLLOVER) begin
                crash_direction_latched <= DIR_ROLL;
            end else begin
                // Find maximum acceleration direction
                if ((imu_x >= imu_y) && (imu_x >= imu_z)) begin
                    if (imu_x[15]) crash_direction_latched <= DIR_REAR;  // Negative = rear impact
                    else crash_direction_latched <= DIR_FRONT;
                end else if (imu_y >= imu_z) begin
                    if (imu_y[15]) crash_direction_latched <= DIR_RIGHT; // Negative = right impact
                    else crash_direction_latched <= DIR_LEFT;
                end else begin
                    crash_direction_latched <= DIR_FRONT;  // Default
                end
            end
        end
    end

    // ==================== CRASH DETECTION FSM ==========
    // Sequential always block for state and all registered outputs
    always @(posedge clk_aon or negedge rst_aon_n) begin : crash_fsm_seq
        if (!rst_aon_n) begin
            crash_state <= CRASH_IDLE;
            crash_timer <= 32'd0;
            reset_timer <= 32'd0;
            multi_impact_timer <= 32'd0;
            impact_count <= 8'd0;
            crash_severity_latched <= 8'd0;
            multi_impact_latched <= 1'b0;
            crash_latched <= 1'b0;
            crash_severity <= 8'd0;
            crash_direction <= 3'b000;
            multi_impact_detect <= 1'b0;
        end else begin
            // Update state first
            crash_state <= next_crash_state;

            // Update timers
            if (crash_timer > 32'd0) crash_timer <= crash_timer - 32'd1;
            if (reset_timer > 32'd0) reset_timer <= reset_timer - 32'd1;
            if (multi_impact_timer > 32'd0) multi_impact_timer <= multi_impact_timer - 32'd1;

            // Update impact count based on crash_state
            case (crash_state)
                CRASH_DETECT: begin
                    if ((crash_impact >= IMPACT_MINOR) && (sensor_valid[2])) begin
                        if (impact_count < 8'd255) impact_count <= impact_count + 8'd1;
                    end
                end
                CRASH_LATCHED: begin
                    if (multi_impact_timer == 32'd0) multi_impact_timer <= MULTI_IMPACT_WINDOW;
                    if ((crash_impact >= IMPACT_MINOR) && (sensor_valid[2])) begin
                        if (impact_count < 8'd255) impact_count <= impact_count + 8'd1;
                        if (impact_count >= 8'd3) multi_impact_latched <= 1'b1;
                    end
                end
                default: ;
            endcase

            // Load crash_timer when entering CRASH_DETECT (so timeout works correctly)
            if (next_crash_state == CRASH_DETECT && crash_state != CRASH_DETECT) begin
                crash_timer <= CRASH_WINDOW;
            end

            // Update latched values on entry to CRASH_CONFIRM
            if (next_crash_state == CRASH_CONFIRM && crash_state != CRASH_CONFIRM) begin
                crash_severity_latched <= combined_severity;
                reset_timer <= RESET_HOLD;
            end

            // Update outputs
            crash_latched <= (crash_state == CRASH_LATCHED) || test_mode;
            crash_severity <= test_mode ? 8'd75 : crash_severity_latched;
            crash_direction <= test_mode ? DIR_FRONT : crash_direction_latched;
            multi_impact_detect <= multi_impact_latched;
        end
    end

    // Combinational next-state logic (no assignments to registered outputs)
    always @(*) begin : crash_fsm_next
        next_crash_state = crash_state;

        case (crash_state)
            CRASH_IDLE: begin
                if ((combined_severity >= 8'd50) && !test_mode) next_crash_state = CRASH_DETECT;
            end

            CRASH_DETECT: begin
                if (crash_timer == 32'd0) begin
                    if ((combined_severity >= 8'd50) || (impact_count >= 8'd2)) begin
                        next_crash_state = CRASH_CONFIRM;
                        // crash_severity_latched and reset_timer updated in sequential block
                    end else begin
                        next_crash_state = CRASH_IDLE;
                    end
                end
            end

            CRASH_CONFIRM: begin
                next_crash_state = CRASH_LATCHED;
            end

            CRASH_LATCHED: begin
                if (reset_timer == 32'd0 && crash_reset) next_crash_state = CRASH_RESETTING;
            end

            CRASH_RESETTING: begin
                if (reset_timer < (RESET_HOLD / 32'd10)) next_crash_state = CRASH_IDLE;
            end

            default: next_crash_state = CRASH_IDLE;
        endcase

        // Test mode override
        if (test_mode) next_crash_state = CRASH_LATCHED;
    end

    // ==================== PREDICTIVE CRASH DETECTION ===
    always @(posedge clk_aon or negedge rst_aon_n) begin : predictive
        if (!rst_aon_n) begin
            pre_crash_accel <= 16'd0;
            pre_crash_gyro <= 16'd0;
            crash_probability <= 8'd0;
            pre_crash_warning <= 1'b0;
        end else begin
            // Calculate pre-crash indicators
            if (imu_ptr == 3'd7) begin
                if (imu_history[7] > imu_history[0])
                    crash_accel_change = (imu_history[7] - imu_history[0]) * 16'd10 / 16'd8;
                else
                    crash_accel_change = (imu_history[0] - imu_history[7]) * 16'd10 / 16'd8;
                if (crash_accel_change > pre_crash_accel) pre_crash_accel <= crash_accel_change;
            end

            if (gyro_ptr == 3'd7) begin
                if (gyro_history[7] > gyro_history[0])
                    crash_gyro_change = (gyro_history[7] - gyro_history[0]) * 16'd10 / 16'd8;
                else
                    crash_gyro_change = (gyro_history[0] - gyro_history[7]) * 16'd10 / 16'd8;
                if (crash_gyro_change > pre_crash_gyro) pre_crash_gyro <= crash_gyro_change;
            end

            // Set pre-crash warning
            if ((pre_crash_accel > ACCEL_WARNING) || (pre_crash_gyro > (GYRO_ROLLOVER / 2))) begin
                pre_crash_warning <= 1'b1;
            end else begin
                pre_crash_warning <= 1'b0;
            end

            // Decay pre-crash indicators over time
            if (pre_crash_accel > 16'd0) pre_crash_accel <= pre_crash_accel - 16'd1;
            if (pre_crash_gyro > 16'd0) pre_crash_gyro <= pre_crash_gyro - 16'd1;
        end
    end

endmodule