// adas_controller_v3.v - static keyword removed
`timescale 1ns/1ps
`include "defines_ivcu_ev_v3.sv"   // FIX: file is defines_ivcu_ev_v3.sv, .v version never existed
`default_nettype none

module adas_controller_v3 (
    input  wire clk_ai,
    input  wire rst_ai_n,
    input  wire perception_ok,
    input  wire dynamics_ok,
    input  wire [1:0] current_mode,
    input  wire [15:0] ultrasonic,
    input  wire [15:0] camera,
    input  wire [15:0] radar,
    input  wire [15:0] lidar,
    input  wire [15:0] gps,
    input  wire [15:0] vehicle_speed,
    input  wire [15:0] steering_angle,
    input  wire [15:0] throttle_position,
    input  wire [15:0] brake_pressure,
    input  wire [8:0] sensor_valid,
    output reg        adas_active,
    output reg [7:0]  adas_confidence,
    output reg        cruise_control,
    output reg        lane_keep,
    output reg        auto_brake,
    output reg [15:0] following_distance,
    output reg [15:0] lane_offset,
    output reg [15:0] collision_time
);

    // ==================== PARAMETERS ====================
    localparam [2:0]
        ADAS_OFF        = 3'b000,
        ADAS_CRUISE     = 3'b001,
        ADAS_LANE_KEEP  = 3'b010,
        ADAS_AUTO_BRAKE = 3'b011,
        ADAS_FULL       = 3'b100,
        ADAS_EMERGENCY  = 3'b101;

    localparam MIN_FOLLOW_DISTANCE = 16'd500;
    localparam WARNING_DISTANCE    = 16'd200;
    localparam EMERGENCY_DISTANCE  = 16'd100;
    localparam CRUISE_MIN_SPEED = 16'd300;
    localparam CRUISE_MAX_SPEED = 16'd1500;
    localparam COLLISION_WARNING_TIME = 16'd30;
    localparam COLLISION_EMERGENCY_TIME = 16'd10;
    localparam LANE_CENTER_OFFSET = 16'd50;
    localparam LANE_WARNING_OFFSET = 16'd100;
    localparam LANE_EMERGENCY_OFFSET = 16'd200;
    localparam CONFIDENCE_HIGH = 8'd80;
    localparam CONFIDENCE_MEDIUM = 8'd50;
    localparam CONFIDENCE_LOW = 8'd30;

    // ==================== INTERNAL REGISTERS ============
    reg [2:0] adas_state, next_adas_state;
    reg [15:0] object_distance, object_relative_speed, object_angle;
    reg [7:0] object_confidence;
    reg [15:0] lane_position, lane_curvature;
    reg [7:0] lane_confidence;
    reg [15:0] target_speed, set_speed, speed_error;
    reg [15:0] acceleration_command, deceleration_command;
    reg [15:0] speed_pid_integral, speed_pid_derivative;
    reg [15:0] lane_pid_integral, lane_pid_derivative;
    reg [31:0] adas_timer;
    reg [15:0] collision_timer;
    reg [7:0] lane_departure_counter;
    reg [15:0] fused_distance, fused_speed;
    reg [7:0] fused_confidence;
    reg [7:0] sensor_fault_counter, control_fault_counter;

    // Previous values for PID (declared at module level)
    reg [15:0] prev_lane_position;
    reg [15:0] prev_speed_error;
    reg [15:0] prev_accel, prev_decel;
    reg [15:0] prev_lane_offset;
    // Temporary computation regs (moved from inline block-local declarations)
    reg [31:0] distance_sum, speed_sum;
    reg [7:0]  valid_count;
    reg [11:0] radar_dist;
    reg [11:0] lidar_dist;
    reg [15:0] steering_correction;

    // ==================== SENSOR FUSION =================
    always @(posedge clk_ai or negedge rst_ai_n) begin
        if (!rst_ai_n) begin
            fused_distance <= 16'd0;
            fused_speed <= 16'd0;
            fused_confidence <= 8'd0;
            object_distance <= 16'd0;
            object_relative_speed <= 16'd0;
            object_confidence <= 8'd0;
        end else if (perception_ok) begin
            distance_sum = 32'd0;
            speed_sum = 32'd0;
            valid_count = 8'd0;

            if (sensor_valid[0] && ultrasonic < 16'd500) begin
                distance_sum = distance_sum + ultrasonic;
                valid_count = valid_count + 1;
            end
            if (sensor_valid[2]) begin
                radar_dist = radar[11:0];
                if (radar_dist > 0) begin
                    distance_sum = distance_sum + {4'b0, radar_dist};
                    speed_sum = speed_sum + radar[15:12];
                    valid_count = valid_count + 1;
                end
            end
            if (sensor_valid[3]) begin
                lidar_dist = lidar[11:0];
                if (lidar_dist > 0) begin
                    distance_sum = distance_sum + {4'b0, lidar_dist};
                    valid_count = valid_count + 1;
                end
            end
            if (valid_count > 0) begin
                fused_distance <= distance_sum / valid_count;
                fused_speed <= speed_sum / valid_count;
                fused_confidence <= (valid_count * 25);
            end else begin
                fused_distance <= 16'd0;
                fused_speed <= 16'd0;
                fused_confidence <= 8'd0;
            end
            object_distance <= fused_distance;
            object_relative_speed <= fused_speed;
            object_confidence <= fused_confidence;
        end
    end

    // ==================== LANE DETECTION =================
    always @(posedge clk_ai or negedge rst_ai_n) begin
        if (!rst_ai_n) begin
            lane_position <= 16'd0;
            lane_curvature <= 16'd0;
            lane_confidence <= 8'd0;
            lane_offset <= 16'd0;
            prev_lane_position <= 16'd0;
        end else if (sensor_valid[1]) begin
            lane_position <= {camera[15:8], 8'b0};
            lane_confidence <= camera[7:0];
            lane_offset <= (lane_position > 16'd0) ? lane_position : (~lane_position + 16'd1);
            if (vehicle_speed > 16'd100) begin
                lane_curvature <= (lane_position - prev_lane_position) * 10 / vehicle_speed;
                prev_lane_position <= lane_position;
            end
        end
    end

    // ==================== COLLISION DETECTION ============
    always @(posedge clk_ai or negedge rst_ai_n) begin
        if (!rst_ai_n) begin
            collision_time <= 16'd0;
            collision_timer <= 16'd0;
        end else if (object_distance > 0 && object_relative_speed > 0) begin
            if (object_relative_speed > 0) begin
                collision_time <= (object_distance * 10) / object_relative_speed;
            end else begin
                collision_time <= 16'd0;
            end
            if (collision_time < COLLISION_EMERGENCY_TIME) begin
                if (collision_timer < 16'hFFFF) collision_timer <= collision_timer + 1;
            end else if (collision_timer > 0) begin
                collision_timer <= collision_timer - 1;
            end
        end
    end

    // ==================== CRUISE CONTROL =================
    always @(posedge clk_ai or negedge rst_ai_n) begin
        if (!rst_ai_n) begin
            target_speed <= 16'd0;
            set_speed <= 16'd0;
            speed_error <= 16'd0;
            acceleration_command <= 16'd0;
            deceleration_command <= 16'd0;
            speed_pid_integral <= 16'd0;
            speed_pid_derivative <= 16'd0;
            following_distance <= MIN_FOLLOW_DISTANCE;
            prev_speed_error <= 16'd0;
        end else if (adas_state == ADAS_CRUISE || adas_state == ADAS_FULL) begin
            following_distance <= (vehicle_speed * 20) / 36;
            if (following_distance < MIN_FOLLOW_DISTANCE)
                following_distance <= MIN_FOLLOW_DISTANCE;

            if (object_distance > 0 && object_distance < (following_distance * 2)) begin
                if (object_distance < following_distance) begin
                    target_speed <= vehicle_speed - 16'd10;
                end else if (object_relative_speed < 0) begin
                    target_speed <= vehicle_speed + object_relative_speed;
                end else begin
                    target_speed <= set_speed;
                end
            end else begin
                target_speed <= set_speed;
            end

            speed_error <= target_speed - vehicle_speed;
            acceleration_command <= speed_error * 2;

            if (speed_pid_integral < 16'd1000 && speed_error > 0)
                speed_pid_integral <= speed_pid_integral + speed_error;
            else if (speed_pid_integral > 0 && speed_error < 0)
                speed_pid_integral <= speed_pid_integral + speed_error;

            acceleration_command <= acceleration_command + (speed_pid_integral / 10);

            speed_pid_derivative <= speed_error - prev_speed_error;
            prev_speed_error <= speed_error;
            acceleration_command <= acceleration_command + speed_pid_derivative;

            if (acceleration_command > 16'd1000) begin
                acceleration_command <= 16'd1000;
            end else if (acceleration_command < -16'd1000) begin
                deceleration_command <= -acceleration_command;
                acceleration_command <= 16'd0;
            end else begin
                deceleration_command <= 16'd0;
            end
        end
    end

    // ==================== LANE KEEPING ===================
    always @(posedge clk_ai or negedge rst_ai_n) begin
        if (!rst_ai_n) begin
            lane_pid_integral <= 16'd0;
            lane_pid_derivative <= 16'd0;
            lane_departure_counter <= 8'd0;
            prev_lane_offset <= 16'd0;
        end else if ((adas_state == ADAS_LANE_KEEP || adas_state == ADAS_FULL) &&
                     lane_confidence > CONFIDENCE_MEDIUM) begin
            steering_correction = lane_offset * 2;
            if (lane_offset > LANE_CENTER_OFFSET) begin
                if (lane_pid_integral < 16'd500) lane_pid_integral <= lane_pid_integral + 1;
            end else if (lane_pid_integral > 0) begin
                lane_pid_integral <= lane_pid_integral - 1;
            end
            steering_correction = steering_correction + lane_pid_integral;
            lane_pid_derivative <= lane_offset - prev_lane_offset;
            prev_lane_offset <= lane_offset;
            steering_correction = steering_correction + lane_pid_derivative;

            if (lane_offset > LANE_WARNING_OFFSET) begin
                if (lane_departure_counter < 8'd255) lane_departure_counter <= lane_departure_counter + 1;
            end else if (lane_departure_counter > 0) begin
                lane_departure_counter <= lane_departure_counter - 1;
            end
        end
    end

    // ==================== FSM SEQUENTIAL =================
    always @(posedge clk_ai or negedge rst_ai_n) begin
        if (!rst_ai_n) begin
            adas_state <= ADAS_OFF;
            adas_timer <= 32'd0;
            sensor_fault_counter <= 8'd0;
            control_fault_counter <= 8'd0;
            prev_accel <= 16'd0;
            prev_decel <= 16'd0;
        end else begin
            adas_state <= next_adas_state;
            if (adas_timer < 32'hFFFFFFFF) adas_timer <= adas_timer + 1;
            if (!perception_ok || !dynamics_ok) begin
                if (sensor_fault_counter < 8'd255) sensor_fault_counter <= sensor_fault_counter + 1;
            end else if (sensor_fault_counter > 0) begin
                sensor_fault_counter <= sensor_fault_counter - 1;
            end
            if (adas_state != ADAS_OFF && vehicle_speed > 16'd100) begin
                if (acceleration_command > (prev_accel * 2) || deceleration_command > (prev_decel * 2)) begin
                    if (control_fault_counter < 8'd255) control_fault_counter <= control_fault_counter + 1;
                end else if (control_fault_counter > 0) begin
                    control_fault_counter <= control_fault_counter - 1;
                end
                prev_accel <= acceleration_command;
                prev_decel <= deceleration_command;
            end
        end
    end

    // ==================== FSM NEXT STATE =================
    always @(*) begin
        next_adas_state = adas_state;
        case (adas_state)
            ADAS_OFF: begin
                if (perception_ok && dynamics_ok && vehicle_speed >= CRUISE_MIN_SPEED && vehicle_speed <= CRUISE_MAX_SPEED)
                    next_adas_state = ADAS_CRUISE;
            end
            ADAS_CRUISE: begin
                if (collision_time < COLLISION_EMERGENCY_TIME)
                    next_adas_state = ADAS_EMERGENCY;
                else if (lane_confidence > CONFIDENCE_HIGH)
                    next_adas_state = ADAS_FULL;
                else if (!perception_ok || vehicle_speed < CRUISE_MIN_SPEED)
                    next_adas_state = ADAS_OFF;
            end
            ADAS_LANE_KEEP: next_adas_state = ADAS_CRUISE;
            ADAS_AUTO_BRAKE: begin
                if (collision_time > COLLISION_WARNING_TIME * 2)
                    next_adas_state = ADAS_CRUISE;
            end
            ADAS_FULL: begin
                if (collision_time < COLLISION_EMERGENCY_TIME)
                    next_adas_state = ADAS_EMERGENCY;
                else if (lane_confidence < CONFIDENCE_MEDIUM)
                    next_adas_state = ADAS_CRUISE;
                else if (!perception_ok || vehicle_speed < CRUISE_MIN_SPEED)
                    next_adas_state = ADAS_OFF;
            end
            ADAS_EMERGENCY: begin
                if (collision_time > COLLISION_WARNING_TIME * 3) begin
                    if (lane_confidence > CONFIDENCE_HIGH) next_adas_state = ADAS_FULL;
                    else next_adas_state = ADAS_CRUISE;
                end
            end
            default: next_adas_state = ADAS_OFF;
        endcase
        if (sensor_fault_counter > 8'd100 || control_fault_counter > 8'd50)
            next_adas_state = ADAS_OFF;
    end

    // ==================== OUTPUTS ========================
    always @(posedge clk_ai or negedge rst_ai_n) begin
        if (!rst_ai_n) begin
            adas_active <= 1'b0;
            adas_confidence <= 8'd0;
            cruise_control <= 1'b0;
            lane_keep <= 1'b0;
            auto_brake <= 1'b0;
        end else begin
            adas_active <= (adas_state != ADAS_OFF);
            adas_confidence <= (object_confidence + lane_confidence) / 2;
            cruise_control <= (adas_state == ADAS_CRUISE || adas_state == ADAS_FULL);
            lane_keep <= (adas_state == ADAS_FULL);
            auto_brake <= (adas_state == ADAS_EMERGENCY || adas_state == ADAS_AUTO_BRAKE);
            if (collision_time < COLLISION_EMERGENCY_TIME && object_confidence > CONFIDENCE_MEDIUM)
                auto_brake <= 1'b1;
        end
    end

endmodule
`default_nettype wire