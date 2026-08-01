// vehicle_dynamics_predictive_complete.v
// Fixed: all reg declarations moved to module level; no inline block-local regs
// FIX (multiple-driver): the note above is what caused the problem. `vi` was
// hoisted to module scope and then written from four separate clocked always
// blocks, so each block inferred its own flip-flop and all four drove the same
// net -- Yosys `check` reported "multiple conflicting drivers for .\vi".
// The eight always blocks are now labelled, and `vi` is declared inside each of
// the four blocks that use it. Labelling is what makes the local declaration
// legal in Verilog-2001; an unnamed begin/end cannot hold declarations, which
// is presumably why everything was hoisted in the first place.
// The tmp_* registers are left at module scope deliberately: each one is used
// by exactly one block, so none of them conflict today. They are latent
// hazards, not current bugs -- see the note at the end of this file.
`timescale 1ns/1ps
`include "defines_ivcu_ev_v3.sv"   // FIX: file is defines_ivcu_ev_v3.sv, .v version never existed

module vehicle_dynamics_predictive_complete (
    input  wire        clk_ai,
    input  wire        rst_ai_n,
    input  wire [15:0] steering_angle,
    input  wire [15:0] gear_position,
    input  wire [15:0] side_stand,
    input  wire [15:0] gyroscope,
    input  wire [15:0] imu,
    input  wire [15:0] wheel_speed,
    input  wire [5:0]  sensor_valid,
    input  wire [1:0]  current_mode,
    input  wire [15:0] vehicle_speed,
    output reg         dynamics_ok,
    output reg  [7:0]  dynamics_score,
    output reg  [3:0]  dynamics_status,
    output reg         stability_control,
    output reg         rollover_risk,
    output reg         skid_detected
);

    // ==================== PARAMETERS ====================
    localparam [15:0] STEERING_MAX             = 16'd4500;
    localparam [15:0] STEERING_WARNING         = 16'd3500;
    localparam [15:0] GYRO_ROLL_WARNING        = 16'd3000;
    localparam [15:0] GYRO_ROLL_CRITICAL       = 16'd5000;
    localparam [15:0] GYRO_YAW_WARNING         = 16'd2000;
    localparam [15:0] GYRO_YAW_CRITICAL        = 16'd4000;
    localparam [15:0] ACCEL_LATERAL_WARNING    = 16'd80;
    localparam [15:0] ACCEL_LATERAL_CRITICAL   = 16'd120;
    localparam [15:0] ACCEL_LONG_WARNING       = 16'd100;
    localparam [15:0] ACCEL_LONG_CRITICAL      = 16'd150;
    localparam [15:0] SPEED_DIFF_WARNING       = 16'd50;
    localparam [15:0] SPEED_DIFF_CRITICAL      = 16'd100;
    localparam [15:0] SIDE_STAND_ENGAGED       = 16'd1000;
    localparam [15:0] GEAR_PARK               = 16'd0;
    localparam [15:0] GEAR_REVERSE            = 16'd1000;
    localparam [15:0] GEAR_NEUTRAL            = 16'd2000;
    localparam [15:0] GEAR_DRIVE              = 16'd3000;

    localparam [2:0] DYN_NORMAL           = 3'b000;
    localparam [2:0] DYN_WARNING          = 3'b001;
    localparam [2:0] DYN_CRITICAL         = 3'b010;
    localparam [2:0] DYN_STABILITY_ACTIVE = 3'b011;
    localparam [2:0] DYN_EMERGENCY        = 3'b100;

    // ==================== INTERNAL REGISTERS ============
    reg [15:0] steering_history [0:7];
    reg [2:0]  steering_ptr;
    reg [15:0] steering_rate;

    reg [15:0] gyro_history [0:7];
    reg [2:0]  gyro_ptr;
    reg [15:0] gyro_rate;

    reg [15:0] imu_history [0:7];
    reg [2:0]  imu_ptr;
    reg [15:0] imu_rate;

    reg [15:0] wheel_speed_history [0:7];
    reg [2:0]  wheel_speed_ptr;

    reg [7:0]  steering_score;
    reg [7:0]  gyro_score;
    reg [7:0]  imu_score;
    reg [7:0]  wheel_score;
    reg [7:0]  gear_score;
    reg [7:0]  stand_score;

    reg [7:0]  steering_fault_counter;
    reg [7:0]  gyro_fault_counter;
    reg [7:0]  imu_fault_counter;
    reg [7:0]  wheel_fault_counter;

    reg [15:0] predicted_trajectory;
    reg [15:0] stability_margin;
    reg [7:0]  rollover_probability;
    reg [7:0]  skid_probability;

    reg [2:0]  dynamics_state;

    // Temporary computation registers (module-level, used as blocking-assigned temporaries)
    // Each is written by exactly ONE always block below -- no driver conflict.
    reg [15:0] tmp_abs_steering;    // steering_analysis only
    reg [15:0] tmp_roll_rate;       // gyro_analysis only
    reg [15:0] tmp_yaw_rate;        // gyro_analysis only
    reg [7:0]  tmp_roll_score;      // gyro_analysis only
    reg [7:0]  tmp_yaw_score;       // gyro_analysis only
    reg [15:0] tmp_lat_accel;       // imu_analysis only
    reg [15:0] tmp_long_accel;      // imu_analysis only
    reg [7:0]  tmp_lat_score;       // imu_analysis only
    reg [7:0]  tmp_long_score;      // imu_analysis only
    reg [7:0]  tmp_combined_score;  // predictive_dynamics only
    reg [15:0] tmp_pred_lat_accel;  // predictive_dynamics only
    reg [15:0] tmp_pred_yaw_rate;   // predictive_dynamics only
    reg [15:0] tmp_expected_yaw;    // predictive_dynamics only

    // NOTE: `integer vi;` used to live here. Removed -- see header comment.

    // ==================== STEERING ANALYSIS ============
    always @(posedge clk_ai or negedge rst_ai_n) begin : steering_analysis
        integer vi;
        if (!rst_ai_n) begin
            for (vi = 0; vi < 8; vi = vi + 1)
                steering_history[vi] <= 16'd0;
            steering_ptr          <= 3'd0;
            steering_rate         <= 16'd0;
            steering_score        <= 8'd100;
            steering_fault_counter<= 8'd0;
        end else if (sensor_valid[0]) begin
            steering_history[steering_ptr] <= steering_angle;
            steering_ptr <= steering_ptr + 3'd1;

            if (steering_ptr == 3'd7) begin
                if (steering_history[7] > steering_history[0])
                    steering_rate <= (steering_history[7] - steering_history[0]) * 10 / 8;
                else
                    steering_rate <= (steering_history[0] - steering_history[7]) * 10 / 8;
            end

            // Use tmp_abs_steering as module-level blocking temp
            if (steering_angle[15])
                tmp_abs_steering = ~steering_angle + 16'd1;
            else
                tmp_abs_steering = steering_angle;

            if (tmp_abs_steering >= STEERING_MAX) begin
                steering_score <= 8'd0;
                if (steering_fault_counter < 8'd255)
                    steering_fault_counter <= steering_fault_counter + 8'd2;
            end else if (tmp_abs_steering >= STEERING_WARNING) begin
                steering_score <= 8'd50;
                if (steering_fault_counter < 8'd255)
                    steering_fault_counter <= steering_fault_counter + 8'd1;
            end else begin
                steering_score <= 8'd100 - (tmp_abs_steering * 8'd100) / STEERING_WARNING;
                if (steering_fault_counter > 8'd0)
                    steering_fault_counter <= steering_fault_counter - 8'd1;
            end

            if (steering_rate > 16'd5000 && vehicle_speed > 16'd500) begin
                if (steering_score > 8'd20)
                    steering_score <= steering_score - 8'd20;
                else
                    steering_score <= 8'd0;
            end
        end
    end

    // ==================== GYROSCOPE ANALYSIS ===========
    always @(posedge clk_ai or negedge rst_ai_n) begin : gyro_analysis
        integer vi;
        if (!rst_ai_n) begin
            for (vi = 0; vi < 8; vi = vi + 1)
                gyro_history[vi] <= 16'd0;
            gyro_ptr          <= 3'd0;
            gyro_rate         <= 16'd0;
            gyro_score        <= 8'd100;
            gyro_fault_counter<= 8'd0;
        end else if (sensor_valid[1]) begin
            gyro_history[gyro_ptr] <= gyroscope;
            gyro_ptr <= gyro_ptr + 3'd1;

            tmp_roll_rate = {gyroscope[15:8], 8'b0};
            tmp_yaw_rate  = {gyroscope[7:0],  8'b0};

            if (tmp_roll_rate >= GYRO_ROLL_CRITICAL || tmp_yaw_rate >= GYRO_YAW_CRITICAL) begin
                gyro_score <= 8'd0;
                if (gyro_fault_counter < 8'd255)
                    gyro_fault_counter <= gyro_fault_counter + 8'd2;
            end else if (tmp_roll_rate >= GYRO_ROLL_WARNING || tmp_yaw_rate >= GYRO_YAW_WARNING) begin
                gyro_score <= 8'd50;
                if (gyro_fault_counter < 8'd255)
                    gyro_fault_counter <= gyro_fault_counter + 8'd1;
            end else begin
                tmp_roll_score = 8'd100 - (tmp_roll_rate * 8'd100) / GYRO_ROLL_WARNING;
                tmp_yaw_score  = 8'd100 - (tmp_yaw_rate  * 8'd100) / GYRO_YAW_WARNING;
                gyro_score <= (tmp_roll_score + tmp_yaw_score) / 8'd2;
                if (gyro_fault_counter > 8'd0)
                    gyro_fault_counter <= gyro_fault_counter - 8'd1;
            end

            if (gyro_ptr == 3'd7) begin
                if (gyro_history[7] > gyro_history[0])
                    gyro_rate <= (gyro_history[7] - gyro_history[0]) * 10 / 8;
                else
                    gyro_rate <= (gyro_history[0] - gyro_history[7]) * 10 / 8;
            end
        end
    end

    // ==================== IMU ANALYSIS =================
    always @(posedge clk_ai or negedge rst_ai_n) begin : imu_analysis
        integer vi;
        if (!rst_ai_n) begin
            for (vi = 0; vi < 8; vi = vi + 1)
                imu_history[vi] <= 16'd0;
            imu_ptr          <= 3'd0;
            imu_rate         <= 16'd0;
            imu_score        <= 8'd100;
            imu_fault_counter<= 8'd0;
        end else if (sensor_valid[2]) begin
            imu_history[imu_ptr] <= imu;
            imu_ptr <= imu_ptr + 3'd1;

            tmp_lat_accel  = {imu[15:8], 8'b0};
            tmp_long_accel = {imu[7:0],  8'b0};

            if (tmp_lat_accel >= ACCEL_LATERAL_CRITICAL || tmp_long_accel >= ACCEL_LONG_CRITICAL) begin
                imu_score <= 8'd0;
                if (imu_fault_counter < 8'd255)
                    imu_fault_counter <= imu_fault_counter + 8'd2;
            end else if (tmp_lat_accel >= ACCEL_LATERAL_WARNING || tmp_long_accel >= ACCEL_LONG_WARNING) begin
                imu_score <= 8'd50;
                if (imu_fault_counter < 8'd255)
                    imu_fault_counter <= imu_fault_counter + 8'd1;
            end else begin
                tmp_lat_score  = 8'd100 - (tmp_lat_accel  * 8'd100) / ACCEL_LATERAL_WARNING;
                tmp_long_score = 8'd100 - (tmp_long_accel * 8'd100) / ACCEL_LONG_WARNING;
                imu_score <= (tmp_lat_score + tmp_long_score) / 8'd2;
                if (imu_fault_counter > 8'd0)
                    imu_fault_counter <= imu_fault_counter - 8'd1;
            end

            if (imu_ptr == 3'd7) begin
                if (imu_history[7] > imu_history[0])
                    imu_rate <= (imu_history[7] - imu_history[0]) * 10 / 8;
                else
                    imu_rate <= (imu_history[0] - imu_history[7]) * 10 / 8;
            end
        end
    end

    // ==================== WHEEL SPEED ANALYSIS =========
    always @(posedge clk_ai or negedge rst_ai_n) begin : wheel_analysis
        integer vi;
        if (!rst_ai_n) begin
            for (vi = 0; vi < 8; vi = vi + 1)
                wheel_speed_history[vi] <= 16'd0;
            wheel_speed_ptr   <= 3'd0;
            wheel_score       <= 8'd100;
            wheel_fault_counter <= 8'd0;
        end else if (sensor_valid[3]) begin
            wheel_speed_history[wheel_speed_ptr] <= wheel_speed;
            wheel_speed_ptr <= wheel_speed_ptr + 3'd1;

            if (wheel_speed > 16'd30000) begin
                wheel_score <= 8'd0;
                if (wheel_fault_counter < 8'd255)
                    wheel_fault_counter <= wheel_fault_counter + 8'd2;
            end else if (wheel_speed > 16'd20000) begin
                wheel_score <= 8'd30;
                if (wheel_fault_counter < 8'd255)
                    wheel_fault_counter <= wheel_fault_counter + 8'd1;
            end else begin
                wheel_score <= 8'd100;
                if (wheel_fault_counter > 8'd0)
                    wheel_fault_counter <= wheel_fault_counter - 8'd1;
            end
        end
    end

    // ==================== GEAR POSITION ANALYSIS =======
    always @(posedge clk_ai or negedge rst_ai_n) begin : gear_analysis
        if (!rst_ai_n) begin
            gear_score <= 8'd100;
        end else if (sensor_valid[4]) begin
            if (current_mode == `MODE_CAR) begin
                if (gear_position == GEAR_PARK && vehicle_speed > 16'd100)
                    gear_score <= 8'd0;
                else if (gear_position == GEAR_REVERSE && vehicle_speed > 16'd1000)
                    gear_score <= 8'd30;
                else if (gear_position == GEAR_NEUTRAL && vehicle_speed > 16'd3000)
                    gear_score <= 8'd50;
                else
                    gear_score <= 8'd100;
            end else
                gear_score <= 8'd100;
        end
    end

    // ==================== SIDE STAND ANALYSIS ==========
    always @(posedge clk_ai or negedge rst_ai_n) begin : stand_analysis
        if (!rst_ai_n) begin
            stand_score <= 8'd100;
        end else if (sensor_valid[5]) begin
            if (current_mode == `MODE_BIKE) begin
                if (side_stand >= SIDE_STAND_ENGAGED && vehicle_speed > 16'd100)
                    stand_score <= 8'd0;
                else if (side_stand >= SIDE_STAND_ENGAGED && vehicle_speed > 16'd0)
                    stand_score <= 8'd30;
                else
                    stand_score <= 8'd100;
            end else
                stand_score <= 8'd100;
        end
    end

    // ==================== PREDICTIVE DYNAMICS ==========
    always @(posedge clk_ai or negedge rst_ai_n) begin : predictive_dynamics
        if (!rst_ai_n) begin
            predicted_trajectory <= 16'd0;
            stability_margin     <= 16'd100;
            rollover_probability <= 8'd0;
            skid_probability     <= 8'd0;
            stability_control    <= 1'b0;
            rollover_risk        <= 1'b0;
            skid_detected        <= 1'b0;
            dynamics_state       <= DYN_NORMAL;
        end else begin
            predicted_trajectory <= (steering_angle * vehicle_speed) / 16'd1000;

            tmp_combined_score = (steering_score + gyro_score + imu_score) / 8'd3;
            stability_margin <= {8'b0, tmp_combined_score};

            // Rollover probability
            tmp_pred_lat_accel = {imu[15:8], 8'b0};
            if (tmp_pred_lat_accel > 16'd0 && vehicle_speed > 16'd3000) begin
                rollover_probability <= (tmp_pred_lat_accel * vehicle_speed) / 16'd10000;
                if (rollover_probability > 8'd100) rollover_probability <= 8'd100;
            end else
                rollover_probability <= 8'd0;

            rollover_risk <= (rollover_probability > 8'd70);

            // Skid probability
            tmp_pred_yaw_rate = {gyroscope[7:0], 8'b0};
            if (steering_angle != 16'd0 && tmp_pred_yaw_rate != 16'd0) begin
                tmp_expected_yaw = (steering_angle * vehicle_speed) / 16'd1000;
                if (tmp_expected_yaw > tmp_pred_yaw_rate)
                    skid_probability <= ((tmp_expected_yaw - tmp_pred_yaw_rate) * 16'd100) / tmp_expected_yaw;
                else if (tmp_pred_yaw_rate > tmp_expected_yaw)
                    skid_probability <= ((tmp_pred_yaw_rate - tmp_expected_yaw) * 16'd100) / tmp_pred_yaw_rate;
                else
                    skid_probability <= 8'd0;
                if (skid_probability > 8'd100) skid_probability <= 8'd100;
            end else
                skid_probability <= 8'd0;

            skid_detected <= (skid_probability > 8'd50);

            stability_control <= rollover_risk | skid_detected | (stability_margin < 16'd50);

            // Dynamics state machine
            case (dynamics_state)
                DYN_NORMAL: begin
                    if (rollover_risk | skid_detected)
                        dynamics_state <= DYN_EMERGENCY;
                    else if (stability_margin < 16'd50)
                        dynamics_state <= DYN_CRITICAL;
                    else if (stability_margin < 16'd70)
                        dynamics_state <= DYN_WARNING;
                end
                DYN_WARNING: begin
                    if (rollover_risk | skid_detected)
                        dynamics_state <= DYN_EMERGENCY;
                    else if (stability_margin < 16'd50)
                        dynamics_state <= DYN_CRITICAL;
                    else if (stability_margin >= 16'd80)
                        dynamics_state <= DYN_NORMAL;
                end
                DYN_CRITICAL: begin
                    if (rollover_risk | skid_detected)
                        dynamics_state <= DYN_EMERGENCY;
                    else if (stability_margin >= 16'd80)
                        dynamics_state <= DYN_NORMAL;
                    else if (stability_margin >= 16'd70)
                        dynamics_state <= DYN_WARNING;
                end
                DYN_STABILITY_ACTIVE: begin
                    if (stability_margin >= 16'd80)
                        dynamics_state <= DYN_NORMAL;
                    else if (stability_margin >= 16'd60)
                        dynamics_state <= DYN_WARNING;
                end
                DYN_EMERGENCY: begin
                    if (!rollover_risk && !skid_detected && stability_margin >= 16'd60)
                        dynamics_state <= DYN_CRITICAL;
                end
                default: dynamics_state <= DYN_NORMAL;
            endcase

            if (stability_control && dynamics_state != DYN_EMERGENCY)
                dynamics_state <= DYN_STABILITY_ACTIVE;
        end
    end

    // ==================== OVERALL DYNAMICS HEALTH ======
    always @(posedge clk_ai or negedge rst_ai_n) begin : overall_health
        if (!rst_ai_n) begin
            dynamics_score  <= 8'd100;
            dynamics_status <= `STATUS_OK;
            dynamics_ok     <= 1'b1;
        end else begin
            dynamics_score <= (steering_score * 8'd2 +
                               gyro_score     * 8'd2 +
                               imu_score      * 8'd2 +
                               wheel_score    * 8'd1 +
                               gear_score     * 8'd1 +
                               stand_score    * 8'd2) / 8'd10;

            case (dynamics_state)
                DYN_NORMAL:           begin dynamics_status <= `STATUS_OK;       dynamics_ok <= 1'b1; end
                DYN_WARNING:          begin dynamics_status <= `STATUS_WARNING;   dynamics_ok <= 1'b1; end
                DYN_CRITICAL:         begin dynamics_status <= `STATUS_CRITICAL;  dynamics_ok <= 1'b0; end
                DYN_STABILITY_ACTIVE: begin dynamics_status <= `STATUS_WARNING;   dynamics_ok <= 1'b1; end
                DYN_EMERGENCY:        begin dynamics_status <= `STATUS_EMERGENCY; dynamics_ok <= 1'b0; end
                default:              begin dynamics_status <= `STATUS_OK;        dynamics_ok <= 1'b1; end
            endcase

            if (steering_fault_counter >= 8'd200 ||
                gyro_fault_counter     >= 8'd200 ||
                imu_fault_counter      >= 8'd200) begin
                dynamics_status <= `STATUS_FAULT;
                dynamics_ok     <= 1'b0;
            end

            if (current_mode == `MODE_BIKE && stand_score == 8'd0) begin
                dynamics_status <= `STATUS_EMERGENCY;
                dynamics_ok     <= 1'b0;
            end
        end
    end

    // LATENT HAZARD (not fixed here, not currently a bug):
    // The thirteen tmp_* registers above are module-scope blocking temporaries.
    // Each is written by exactly one always block today, so `check` is clean.
    // If any future edit assigns one of them from a second block, this same
    // multiple-driver failure returns. Now that every block is labelled, they
    // can be moved into their owning blocks whenever you want to close that off.

endmodule
