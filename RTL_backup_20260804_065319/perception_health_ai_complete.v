// perception_health_ai_complete.v
`timescale 1ns/1ps
`include "defines_ivcu_ev_v3.sv"   // FIX: file is defines_ivcu_ev_v3.sv, .v version never existed

module perception_health_ai_complete (
    input  wire        clk_ai,
    input  wire        rst_ai_n,
    input  wire [15:0] ultrasonic,
    input  wire [15:0] camera,
    input  wire [15:0] radar,
    input  wire [15:0] lidar,
    input  wire [15:0] gps,
    input  wire [15:0] tpms,
    input  wire [5:0]  sensor_valid,
    input  wire [1:0]  current_mode,
    input  wire [15:0] vehicle_speed,
    output reg         perception_ok,
    output reg  [7:0]  perception_score,
    output reg  [3:0]  perception_status,
    output reg  [7:0]  sensor_confidence_0,
    output reg  [7:0]  sensor_confidence_1,
    output reg  [7:0]  sensor_confidence_2,
    output reg  [7:0]  sensor_confidence_3,
    output reg  [7:0]  sensor_confidence_4,
    output reg  [7:0]  sensor_confidence_5,
    output reg         blind_spot_warning,
    output reg         collision_warning,
    output reg         lane_departure
);

    // ==================== PARAMETERS ====================
    localparam [15:0] ULTRASONIC_MIN_RANGE = 16'd20;    // 20cm
    localparam [15:0] ULTRASONIC_MAX_RANGE = 16'd500;   // 500cm
    localparam [15:0] ULTRASONIC_WARNING_RANGE = 16'd100; // 100cm

    localparam [15:0] CAMERA_CONFIDENCE_MIN = 16'd500;  // 50%
    localparam [15:0] CAMERA_CONFIDENCE_WARNING = 16'd300; // 30%

    localparam [15:0] RADAR_CONFIDENCE_MIN = 16'd500;   // 50%
    localparam [15:0] RADAR_CONFIDENCE_WARNING = 16'd300; // 30%

    localparam [15:0] LIDAR_CONFIDENCE_MIN = 16'd500;   // 50%
    localparam [15:0] LIDAR_CONFIDENCE_WARNING = 16'd300; // 30%

    localparam [15:0] GPS_SAT_MIN = 16'd6;      // Minimum satellites
    localparam [15:0] GPS_HDOP_MAX = 16'd200;   // Maximum HDOP (2.0)

    localparam [15:0] TPMS_MIN_PRESSURE = 16'd250;   // 25 psi
    localparam [15:0] TPMS_MAX_PRESSURE = 16'd450;   // 45 psi
    localparam [15:0] TPMS_WARNING_DIFF = 16'd50;    // 5 psi difference

    localparam [15:0] COLLISION_IMMINENT = 16'd20;   // 2.0 seconds
    localparam [15:0] COLLISION_WARNING = 16'd50;    // 5.0 seconds

    // FIXED: increased bit widths to hold value 8
    localparam [3:0] ULTRASONIC_HISTORY_SIZE = 4'd8;
    localparam [3:0] CAMERA_HISTORY_SIZE = 4'd8;
    localparam [3:0] RADAR_HISTORY_SIZE = 4'd8;
    localparam [3:0] LIDAR_HISTORY_SIZE = 4'd8;

    // ==================== INTERNAL REGISTERS ============
    reg [7:0]  ultrasonic_score;
    reg [7:0]  camera_score;
    reg [7:0]  radar_score;
    reg [7:0]  lidar_score;
    reg [7:0]  gps_score;
    reg [7:0]  tpms_score;

    reg [7:0]  ultrasonic_fault_counter;
    reg [7:0]  camera_fault_counter;
    reg [7:0]  radar_fault_counter;
    reg [7:0]  lidar_fault_counter;
    reg [7:0]  gps_fault_counter;
    reg [7:0]  tpms_fault_counter;

    reg [15:0] ultrasonic_history [0:7];
    reg [2:0]  ultrasonic_ptr;

    reg [15:0] camera_history [0:7];
    reg [2:0]  camera_ptr;

    reg [15:0] radar_history [0:7];
    reg [2:0]  radar_ptr;

    reg [15:0] lidar_history [0:7];
    reg [2:0]  lidar_ptr;

    reg [15:0] object_distance;
    reg [15:0] object_relative_speed;
    reg [15:0] time_to_collision;
    reg [7:0]  object_confidence;

    reg [15:0] blind_spot_left;
    reg [15:0] blind_spot_right;
    reg        blind_spot_left_active;
    reg        blind_spot_right_active;

    reg [15:0] lane_position;
    reg [15:0] lane_confidence;
    reg        lane_departure_detected;
    reg [15:0] prev_lane_position;

    reg [15:0] gps_satellites;
    reg [15:0] gps_hdop;
    reg [15:0] gps_speed;
    reg [15:0] gps_heading;

    reg [15:0] tire_pressure [0:3];
    reg [15:0] tire_temperature [0:3];
    reg [7:0]  tire_health [0:3];

    // Temporary variables for always blocks (declared at module level)
    reg all_same;
    reg [3:0] radar_confidence_val;
    reg [11:0] radar_distance;
    reg [3:0] lidar_confidence_val;
    reg [11:0] lidar_distance;
    reg [7:0] camera_confidence_val;
    reg [7:0] object_count;
    reg [7:0] individual_scores [0:3];
    reg [7:0] avg_pressure_score;
    reg [7:0] pressure_diff_score;
    reg [15:0] avg_pressure;
    reg [15:0] max_diff;
    reg [15:0] diff;
    reg [15:0] accel_change;
    reg [15:0] gyro_change;

    // ==================== ULTRASONIC SENSOR AI =========
    always @(posedge clk_ai or negedge rst_ai_n) begin : ultrasonic_ai
        integer i;
        if (!rst_ai_n) begin
            ultrasonic_score <= 8'd100;
            ultrasonic_fault_counter <= 8'd0;
            for (i = 0; i < 8; i = i + 1) ultrasonic_history[i] <= 16'd0;
            ultrasonic_ptr <= 3'd0;
            sensor_confidence_0 <= 8'd100;
        end else if (sensor_valid[0]) begin
            ultrasonic_history[ultrasonic_ptr] <= ultrasonic;
            ultrasonic_ptr <= ultrasonic_ptr + 3'd1;
            if ((ultrasonic < ULTRASONIC_MIN_RANGE) || (ultrasonic > ULTRASONIC_MAX_RANGE)) begin
                ultrasonic_score <= 8'd0;
                sensor_confidence_0 <= 8'd0;
                if (ultrasonic_fault_counter < 8'd255) ultrasonic_fault_counter <= ultrasonic_fault_counter + 8'd2;
            end else if (ultrasonic < ULTRASONIC_WARNING_RANGE) begin
                ultrasonic_score <= 8'd50;
                sensor_confidence_0 <= 8'd50;
                if (ultrasonic_fault_counter < 8'd255) ultrasonic_fault_counter <= ultrasonic_fault_counter + 8'd1;
            end else begin
                ultrasonic_score <= 8'd100;
                sensor_confidence_0 <= 8'd100;
                if (ultrasonic_fault_counter > 8'd0) ultrasonic_fault_counter <= ultrasonic_fault_counter - 8'd1;
            end
            if (ultrasonic_ptr == 3'd7) begin
                all_same = 1'b1;
                for (i = 1; i < 8; i = i + 1) if (ultrasonic_history[i] != ultrasonic_history[0]) all_same = 1'b0;
                if (all_same && (ultrasonic_fault_counter < 8'd255)) begin
                    ultrasonic_fault_counter <= ultrasonic_fault_counter + 8'd1;
                    if (ultrasonic_score > 8'd20) ultrasonic_score <= ultrasonic_score - 8'd20;
                    else ultrasonic_score <= 8'd0;
                end
            end
        end else begin
            ultrasonic_score <= 8'd0;
            sensor_confidence_0 <= 8'd0;
        end
    end

    // ==================== CAMERA SENSOR AI =============
    always @(posedge clk_ai or negedge rst_ai_n) begin : camera_ai
        integer i;
        if (!rst_ai_n) begin
            camera_score <= 8'd100;
            camera_fault_counter <= 8'd0;
            for (i = 0; i < 8; i = i + 1) camera_history[i] <= 16'd0;
            camera_ptr <= 3'd0;
            sensor_confidence_1 <= 8'd100;
            prev_lane_position <= 16'd0;
        end else if (sensor_valid[1]) begin
            camera_confidence_val = camera[15:8];
            object_count = camera[7:0];
            camera_history[camera_ptr] <= {8'b0, camera_confidence_val};
            camera_ptr <= camera_ptr + 3'd1;
            if (camera_confidence_val < CAMERA_CONFIDENCE_WARNING[7:0]) begin
                camera_score <= 8'd0;
                sensor_confidence_1 <= camera_confidence_val;
                if (camera_fault_counter < 8'd255) camera_fault_counter <= camera_fault_counter + 8'd2;
            end else if (camera_confidence_val < CAMERA_CONFIDENCE_MIN[7:0]) begin
                camera_score <= 8'd50;
                sensor_confidence_1 <= camera_confidence_val;
                if (camera_fault_counter < 8'd255) camera_fault_counter <= camera_fault_counter + 8'd1;
            end else begin
                camera_score <= 8'd100;
                sensor_confidence_1 <= camera_confidence_val;
                if (camera_fault_counter > 8'd0) camera_fault_counter <= camera_fault_counter - 8'd1;
            end
            lane_position <= {camera[7:0], 8'b0};
            lane_confidence <= {8'b0, camera_confidence_val};
            if ((vehicle_speed > 16'd1000) && (lane_confidence > 8'd50)) begin
                if ((lane_position > 16'd500) || (lane_position < -16'd500)) lane_departure_detected <= 1'b1;
                else lane_departure_detected <= 1'b0;
            end else lane_departure_detected <= 1'b0;
        end else begin
            camera_score <= 8'd0;
            sensor_confidence_1 <= 8'd0;
            lane_departure_detected <= 1'b0;
        end
    end

    // ==================== RADAR SENSOR AI ==============
    always @(posedge clk_ai or negedge rst_ai_n) begin : radar_ai
        integer i;
        reg [7:0] confidence_scaled;
        if (!rst_ai_n) begin
            radar_score <= 8'd100;
            radar_fault_counter <= 8'd0;
            for (i = 0; i < 8; i = i + 1) radar_history[i] <= 16'd0;
            radar_ptr <= 3'd0;
            sensor_confidence_2 <= 8'd100;
            object_distance <= 16'd0;
            object_relative_speed <= 16'd0;
            // FIX: was never driven anywhere (floating reg). This module has only one radar
            // input, so there is no real right-side channel to compute this from yet - tied
            // off to 0 (known coverage gap: only left-side blind-spot detection exists today).
            blind_spot_right_active <= 1'b0;
        end else if (sensor_valid[2]) begin
            radar_confidence_val = radar[15:12];
            radar_distance = radar[11:0];
            radar_history[radar_ptr] <= {4'b0, radar_distance};
            radar_ptr <= radar_ptr + 3'd1;
            confidence_scaled = {radar_confidence_val, 4'b0};
            if (confidence_scaled < RADAR_CONFIDENCE_WARNING[7:0]) begin
                radar_score <= 8'd0;
                sensor_confidence_2 <= confidence_scaled;
                if (radar_fault_counter < 8'd255) radar_fault_counter <= radar_fault_counter + 8'd2;
            end else if (confidence_scaled < RADAR_CONFIDENCE_MIN[7:0]) begin
                radar_score <= 8'd50;
                sensor_confidence_2 <= confidence_scaled;
                if (radar_fault_counter < 8'd255) radar_fault_counter <= radar_fault_counter + 8'd1;
            end else begin
                radar_score <= 8'd100;
                sensor_confidence_2 <= confidence_scaled;
                if (radar_fault_counter > 8'd0) radar_fault_counter <= radar_fault_counter - 8'd1;
            end
            object_distance <= {4'b0, radar_distance};
            if (radar_ptr == 3'd7) begin
                if (radar_history[7] < radar_history[0])
                    object_relative_speed <= (radar_history[0] - radar_history[7]) * 16'd10 / 16'd8;
                else object_relative_speed <= 16'd0;
            end
            if ((radar_distance < 16'd300) && (radar_distance > 16'd50)) begin
                blind_spot_left <= {4'b0, radar_distance};
                blind_spot_left_active <= 1'b1;
            end else blind_spot_left_active <= 1'b0;
        end else begin
            radar_score <= 8'd0;
            sensor_confidence_2 <= 8'd0;
            blind_spot_left_active <= 1'b0;
        end
    end

    // ==================== LIDAR SENSOR AI ==============
    always @(posedge clk_ai or negedge rst_ai_n) begin : lidar_ai
        integer i;
        reg [7:0] confidence_scaled;
        if (!rst_ai_n) begin
            lidar_score <= 8'd100;
            lidar_fault_counter <= 8'd0;
            for (i = 0; i < 8; i = i + 1) lidar_history[i] <= 16'd0;
            lidar_ptr <= 3'd0;
            sensor_confidence_3 <= 8'd100;
        end else if (sensor_valid[3]) begin
            lidar_confidence_val = lidar[15:12];
            lidar_distance = lidar[11:0];
            lidar_history[lidar_ptr] <= {4'b0, lidar_distance};
            lidar_ptr <= lidar_ptr + 3'd1;
            confidence_scaled = {lidar_confidence_val, 4'b0};
            if (confidence_scaled < LIDAR_CONFIDENCE_WARNING[7:0]) begin
                lidar_score <= 8'd0;
                sensor_confidence_3 <= confidence_scaled;
                if (lidar_fault_counter < 8'd255) lidar_fault_counter <= lidar_fault_counter + 8'd2;
            end else if (confidence_scaled < LIDAR_CONFIDENCE_MIN[7:0]) begin
                lidar_score <= 8'd50;
                sensor_confidence_3 <= confidence_scaled;
                if (lidar_fault_counter < 8'd255) lidar_fault_counter <= lidar_fault_counter + 8'd1;
            end else begin
                lidar_score <= 8'd100;
                sensor_confidence_3 <= confidence_scaled;
                if (lidar_fault_counter > 8'd0) lidar_fault_counter <= lidar_fault_counter - 8'd1;
            end
            if (lidar_distance < 16'd500) object_confidence <= confidence_scaled;
        end else begin
            lidar_score <= 8'd0;
            sensor_confidence_3 <= 8'd0;
        end
    end

    // ==================== GPS SENSOR AI ================
    always @(posedge clk_ai or negedge rst_ai_n) begin : gps_ai
        if (!rst_ai_n) begin
            gps_score <= 8'd100;
            gps_fault_counter <= 8'd0;
            gps_satellites <= 16'd0;
            gps_hdop <= 16'd0;
            gps_speed <= 16'd0;
            gps_heading <= 16'd0;
            sensor_confidence_4 <= 8'd100;
        end else if (sensor_valid[4]) begin
            gps_satellites <= {12'b0, gps[15:12]};
            gps_hdop <= {8'b0, gps[11:8], 4'b0};
            gps_speed <= {8'b0, gps[7:0]};
            if ((gps_satellites < GPS_SAT_MIN) || (gps_hdop > GPS_HDOP_MAX)) begin
                gps_score <= 8'd0;
                sensor_confidence_4 <= 8'd0;
                if (gps_fault_counter < 8'd255) gps_fault_counter <= gps_fault_counter + 8'd2;
            end else if ((gps_satellites < 8'd8) || (gps_hdop > 16'd100)) begin
                gps_score <= 8'd50;
                sensor_confidence_4 <= 8'd50;
                if (gps_fault_counter < 8'd255) gps_fault_counter <= gps_fault_counter + 8'd1;
            end else begin
                gps_score <= 8'd100;
                sensor_confidence_4 <= 8'd100;
                if (gps_fault_counter > 8'd0) gps_fault_counter <= gps_fault_counter - 8'd1;
            end
            gps_heading <= gps_speed;
        end else begin
            gps_score <= 8'd0;
            sensor_confidence_4 <= 8'd0;
        end
    end

    // ==================== TPMS SENSOR AI ===============
    always @(posedge clk_ai or negedge rst_ai_n) begin : tpms_ai
        integer i;
        integer j;
        if (!rst_ai_n) begin
            tpms_score <= 8'd100;
            tpms_fault_counter <= 8'd0;
            for (i = 0; i < 4; i = i + 1) begin
                tire_pressure[i] <= 16'd0;
                tire_temperature[i] <= 16'd0;
                tire_health[i] <= 8'd100;
            end
            sensor_confidence_5 <= 8'd100;
        end else if (sensor_valid[5]) begin
            tire_pressure[0] <= {12'b0, tpms[15:12], 4'b0};
            tire_pressure[1] <= {12'b0, tpms[11:8], 4'b0};
            tire_pressure[2] <= {12'b0, tpms[7:4], 4'b0};
            tire_pressure[3] <= {12'b0, tpms[3:0], 4'b0};

            for (j = 0; j < 4; j = j + 1) begin
                if ((tire_pressure[j] < TPMS_MIN_PRESSURE) || (tire_pressure[j] > TPMS_MAX_PRESSURE)) begin
                    individual_scores[j] = 8'd0;
                    tire_health[j] <= 8'd0;
                end else if ((tire_pressure[j] < (TPMS_MIN_PRESSURE + TPMS_WARNING_DIFF)) || 
                           (tire_pressure[j] > (TPMS_MAX_PRESSURE - TPMS_WARNING_DIFF))) begin
                    individual_scores[j] = 8'd50;
                    tire_health[j] <= 8'd50;
                end else begin
                    individual_scores[j] = 8'd100;
                    tire_health[j] <= 8'd100;
                end
            end

            avg_pressure = (tire_pressure[0] + tire_pressure[1] + 
                           tire_pressure[2] + tire_pressure[3]) / 16'd4;

            max_diff = 16'd0;
            for (j = 0; j < 4; j = j + 1) begin
                if (tire_pressure[j] > avg_pressure) diff = tire_pressure[j] - avg_pressure;
                else diff = avg_pressure - tire_pressure[j];
                if (diff > max_diff) max_diff = diff;
            end

            if (max_diff > (TPMS_WARNING_DIFF * 16'd2)) pressure_diff_score = 8'd0;
            else if (max_diff > TPMS_WARNING_DIFF) pressure_diff_score = 8'd50;
            else pressure_diff_score = 8'd100;

            tpms_score <= (individual_scores[0] + individual_scores[1] + 
                          individual_scores[2] + individual_scores[3] + 
                          pressure_diff_score) / 8'd5;

            sensor_confidence_5 <= tpms_score;

            if (tpms_score == 8'd0) begin
                if (tpms_fault_counter < 8'd255) tpms_fault_counter <= tpms_fault_counter + 8'd2;
            end else if (tpms_score < 8'd50) begin
                if (tpms_fault_counter < 8'd255) tpms_fault_counter <= tpms_fault_counter + 8'd1;
            end else begin
                if (tpms_fault_counter > 8'd0) tpms_fault_counter <= tpms_fault_counter - 8'd1;
            end
        end else begin
            tpms_score <= 8'd0;
            sensor_confidence_5 <= 8'd0;
        end
    end

    // ==================== COLLISION DETECTION ==========
    always @(posedge clk_ai or negedge rst_ai_n) begin : collision_detect
        if (!rst_ai_n) begin
            collision_warning <= 1'b0;
            blind_spot_warning <= 1'b0;
            lane_departure <= 1'b0;
        end else begin
            // time_to_collision moved OUT of this block -- see the ttc_calc
            // block near the end of this file. It now runs on a sequential
            // divider. Any assignment left here would create a second driver.
            // collision_warning below still reads time_to_collision exactly as
            // before: non-blocking, so it sees the previous cycle's value.
            if ((time_to_collision > 16'd0) && (time_to_collision < COLLISION_IMMINENT))
                collision_warning <= 1'b1;
            else if ((time_to_collision > 16'd0) && (time_to_collision < COLLISION_WARNING))
                collision_warning <= 1'b1;
            else collision_warning <= 1'b0;
            blind_spot_warning <= blind_spot_left_active || blind_spot_right_active;
            lane_departure <= lane_departure_detected && (vehicle_speed > 16'd1000);
        end
    end

    // ==================== OVERALL PERCEPTION HEALTH ====
    always @(posedge clk_ai or negedge rst_ai_n) begin : overall_perception
        if (!rst_ai_n) begin
            perception_score <= 8'd100;
            perception_status <= `STATUS_OK;
            perception_ok <= 1'b1;
        end else begin
            perception_score <= (ultrasonic_score * 8'd1 + 
                                camera_score * 8'd3 + 
                                radar_score * 8'd3 + 
                                lidar_score * 8'd2 + 
                                gps_score * 8'd1 + 
                                tpms_score * 8'd1) / 8'd11;

            if (perception_score >= 8'd80) begin
                perception_status <= `STATUS_OK;
                perception_ok <= 1'b1;
            end else if (perception_score >= 8'd50) begin
                perception_status <= `STATUS_WARNING;
                perception_ok <= 1'b1;
            end else if (perception_score >= 8'd20) begin
                perception_status <= `STATUS_CRITICAL;
                perception_ok <= 1'b0;
            end else begin
                perception_status <= `STATUS_FAULT;
                perception_ok <= 1'b0;
            end

            if ((camera_fault_counter >= 8'd200) && (radar_fault_counter >= 8'd200)) begin
                perception_status <= `STATUS_EMERGENCY;
                perception_ok <= 1'b0;
            end

            if (collision_warning && (time_to_collision < COLLISION_IMMINENT)) begin
                perception_status <= `STATUS_EMERGENCY;
                perception_ok <= 1'b0;
            end
        end
    end

    // ======================================================================
    // TIME TO COLLISION = distance / closing speed, on a sequential divider.
    //
    // Was fully combinational: 30.542 ns arrival against a 10 ns clock, worst
    // slack -20.936 ns. object_relative_speed is a genuine runtime value, so
    // unlike the sensor fabric there is no shift trick available.
    //
    // Latency is now 32 clocks = 320 ns at 100 MHz. A vehicle closing at
    // 30 m/s travels 0.01 mm in that time.
    //
    // The !ttc_cond branch reproduces the original `else time_to_collision <=
    // 16'd0;` so behaviour with no valid object in front is unchanged.
    // ======================================================================
    wire [31:0] ttc_num  = {16'd0, object_distance} * 32'd10;
    wire [31:0] ttc_den  = {16'd0, object_relative_speed};
    wire        ttc_cond = (object_distance       > 16'd0)
                        && (object_relative_speed > 16'd0);

    wire [31:0] ttc_quot;
    wire        ttc_busy;
    wire        ttc_done;

    seq_divider #(.WIDTH(32)) u_div_ttc (
        .clk       (clk_ai),
        .rst_n     (rst_ai_n),
        .start     (ttc_cond && !ttc_busy),
        .dividend  (ttc_num),
        .divisor   (ttc_den),
        .quotient  (ttc_quot),
        .remainder (),
        .busy      (ttc_busy),
        .done      (ttc_done)
    );

    // SOLE driver of time_to_collision.
    always @(posedge clk_ai or negedge rst_ai_n) begin : ttc_calc
        if (!rst_ai_n)      time_to_collision <= 16'd0;
        else if (!ttc_cond) time_to_collision <= 16'd0;
        else if (ttc_done)  time_to_collision <= ttc_quot[15:0];
    end

endmodule