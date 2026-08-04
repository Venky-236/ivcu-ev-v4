// motor_control_hybrid.v
`timescale 1ns/1ps
`include "defines_ivcu_ev_v3.sv"   // FIX: file is defines_ivcu_ev_v3.sv, .v version never existed

module motor_control_hybrid (
    input  wire        clk_ai,
    input  wire        rst_ai_n,
    input  wire        motor_ok,
    input  wire        thermal_ok,
    input  wire        dynamics_ok,
    input  wire [1:0]  current_mode,
    input  wire [15:0] throttle_position,
    input  wire [15:0] brake_pressure,
    input  wire [15:0] vehicle_speed,
    input  wire [15:0] motor_rpm,
    input  wire [15:0] motor_temp,
    input  wire [15:0] battery_soc,
    input  wire        adas_cruise_control,
    input  wire        adas_auto_brake,
    output reg  [15:0] torque_command,
    output reg  [15:0] regen_command,
    output reg  [15:0] steering_assist,
    output reg  [15:0] torque_limit,
    output reg  [15:0] regen_limit,
    output reg  [15:0] power_limit
);

    // ==================== PARAMETERS ====================
    localparam [2:0] MODE_TORQUE    = 3'b000;
    localparam [2:0] MODE_SPEED     = 3'b001;
    localparam [2:0] MODE_POSITION  = 3'b010;
    localparam [2:0] MODE_RECOVERY  = 3'b011;
    localparam [2:0] MODE_LIMITED   = 3'b100;
    localparam [2:0] MODE_EMERGENCY = 3'b101;

    localparam [15:0] MAX_TORQUE_CAR  = 16'd5000;
    localparam [15:0] MAX_TORQUE_BIKE = 16'd1000;
    localparam [15:0] MAX_TORQUE_SAFE = 16'd500;
    localparam [15:0] MAX_RPM_CAR  = 16'd10000;
    localparam [15:0] MAX_RPM_BIKE = 16'd8000;
    localparam [15:0] MAX_RPM_SAFE = 16'd5000;
    localparam [15:0] TEMP_REDUCE_TORQUE = 16'd800;
    localparam [15:0] TEMP_LIMIT_TORQUE  = 16'd900;
    localparam [15:0] TEMP_CUTOFF        = 16'd1000;
    localparam [15:0] SOC_LOW_LIMIT      = 16'd200;
    localparam [15:0] SOC_CRITICAL_LIMIT = 16'd100;
    localparam [15:0] MAX_REGEN       = 16'd1000;
    localparam [15:0] MIN_REGEN_SPEED = 16'd100;
    localparam [15:0] SPEED_KP = 16'h0100;
    localparam [15:0] SPEED_KI = 16'h0020;
    localparam [15:0] SPEED_KD = 16'h0008;
    localparam [15:0] TORQUE_KP = 16'h0200;
    localparam [15:0] TORQUE_KI = 16'h0010;
    localparam [15:0] TORQUE_KD = 16'h0004;

    // ==================== INTERNAL REGISTERS ============
    reg [2:0] control_state;
    reg [2:0] next_control_state;
    reg [15:0] throttle_filtered;
    reg [15:0] throttle_derivative;
    reg [15:0] throttle_integral;
    reg [7:0]  throttle_smoothness;
    reg [15:0] limited_torque;
    reg [15:0] torque_error;
    reg [31:0] torque_pid_integral;
    reg [15:0] torque_pid_derivative;
    reg [15:0] target_speed;
    reg [15:0] speed_error;
    reg [31:0] speed_pid_integral;
    reg [15:0] speed_pid_derivative;
    reg [15:0] requested_regen;
    reg [15:0] regen_available;
    reg [7:0]  regen_efficiency;
    reg [15:0] steering_torque;
    reg [15:0] steering_damping;
    reg [15:0] thermal_limit;
    reg [15:0] soc_limit;
    reg [15:0] speed_limit;
    reg [15:0] combined_limit;
    reg [7:0]  throttle_fault_counter;
    reg [7:0]  torque_fault_counter;
    reg [7:0]  regen_fault_counter;
    reg [31:0] control_timer;
    reg [15:0] torque_ramp_counter;
    reg [15:0] regen_ramp_counter;
    reg [15:0] prev_throttle;
    reg [15:0] prev_torque_error;
    reg [15:0] prev_speed_error;
    reg [15:0] requested_torque;  // now assigned in a single block
    // Temporary computation regs (moved from inline block-local declarations)
    reg [15:0] torque_from_throttle;
    reg [15:0] torque_from_speed;

    // ==================== THROTTLE PROCESSING ==========
    always @(posedge clk_ai or negedge rst_ai_n) begin : throttle_proc
        if (!rst_ai_n) begin
            throttle_filtered <= 16'd0;
            throttle_derivative <= 16'd0;
            throttle_integral <= 16'd0;
            throttle_smoothness <= 8'd100;
            throttle_fault_counter <= 8'd0;
            prev_throttle <= 16'd0;
        end else begin
            throttle_filtered <= (throttle_filtered * 16'd3 + throttle_position) / 16'd4;
            if (throttle_filtered > prev_throttle)
                throttle_derivative <= throttle_filtered - prev_throttle;
            else throttle_derivative <= prev_throttle - throttle_filtered;
            prev_throttle <= throttle_filtered;
            if (throttle_derivative < 16'd10) begin
                if (throttle_integral < 16'd1000) throttle_integral <= throttle_integral + 16'd1;
            end else if (throttle_integral > 16'd0) throttle_integral <= throttle_integral - 16'd1;
            if (throttle_derivative < 16'd5) throttle_smoothness <= 8'd100;
            else if (throttle_derivative < 16'd20) throttle_smoothness <= 8'd80;
            else if (throttle_derivative < 16'd50) throttle_smoothness <= 8'd50;
            else throttle_smoothness <= 8'd20;
            if ((throttle_position == 16'd0) || (throttle_position == 16'd1000)) begin
                if (throttle_fault_counter < 8'd255) throttle_fault_counter <= throttle_fault_counter + 8'd1;
            end else if (throttle_fault_counter > 8'd0) throttle_fault_counter <= throttle_fault_counter - 8'd1;
        end
    end

    // ==================== COMBINED TORQUE/SPEED CONTROL ==========
    always @(posedge clk_ai or negedge rst_ai_n) begin : torque_speed_control
        if (!rst_ai_n) begin
            requested_torque <= 16'd0;
            limited_torque <= 16'd0;
            torque_error <= 16'd0;
            torque_pid_integral <= 32'd0;
            torque_pid_derivative <= 16'd0;
            torque_fault_counter <= 8'd0;
            prev_torque_error <= 16'd0;
            target_speed <= 16'd0;
            speed_error <= 16'd0;
            speed_pid_integral <= 32'd0;
            speed_pid_derivative <= 16'd0;
            prev_speed_error <= 16'd0;
        end else begin
            // Compute throttle-based torque (for MODE_TORQUE)
            case (current_mode)
                `MODE_CAR: torque_from_throttle = (throttle_filtered * MAX_TORQUE_CAR) / 16'd1000;
                `MODE_BIKE: torque_from_throttle = (throttle_filtered * MAX_TORQUE_BIKE) / 16'd1000;
                `MODE_SAFE: torque_from_throttle = (throttle_filtered * MAX_TORQUE_SAFE) / 16'd1000;
                default: torque_from_throttle = (throttle_filtered * MAX_TORQUE_CAR) / 16'd1000;
            endcase
            if (vehicle_speed > 16'd800) torque_from_throttle = (torque_from_throttle * 8'd8) / 8'd10;
            else if (vehicle_speed > 16'd500) torque_from_throttle = (torque_from_throttle * 8'd9) / 8'd10;

            // Compute speed-based torque (for MODE_SPEED)
            target_speed <= vehicle_speed;  // simplified cruise control
            speed_error <= target_speed - vehicle_speed;
            torque_from_speed = (speed_error * SPEED_KP) / 16'd256;
            if ((speed_pid_integral < 32'd1000000) && (speed_error > 16'd0))
                speed_pid_integral <= speed_pid_integral + speed_error;
            else if ((speed_pid_integral > 32'd0) && (speed_error < 16'd0))
                speed_pid_integral <= speed_pid_integral + speed_error;
            torque_from_speed = torque_from_speed + (speed_pid_integral * SPEED_KI) / 32'd65536;
            speed_pid_derivative <= speed_error - prev_speed_error;
            prev_speed_error <= speed_error;
            torque_from_speed = torque_from_speed + (speed_pid_derivative * SPEED_KD) / 16'd256;

            // Select based on control state
            if (control_state == MODE_SPEED || adas_cruise_control)
                requested_torque <= torque_from_speed;
            else
                requested_torque <= torque_from_throttle;

            // Common torque PID
            torque_error <= requested_torque - limited_torque;
            limited_torque <= (torque_error * TORQUE_KP) / 16'd256;
            if ((torque_pid_integral < 32'd1000000) && (torque_error > 16'd0))
                torque_pid_integral <= torque_pid_integral + torque_error;
            else if ((torque_pid_integral > 32'd0) && (torque_error < 16'd0))
                torque_pid_integral <= torque_pid_integral + torque_error;
            limited_torque <= limited_torque + (torque_pid_integral * TORQUE_KI) / 32'd65536;
            torque_pid_derivative <= torque_error - prev_torque_error;
            prev_torque_error <= torque_error;
            limited_torque <= limited_torque + (torque_pid_derivative * TORQUE_KD) / 16'd256;
            if (limited_torque > ((combined_limit * 16'd12) / 16'd10)) begin
                if (torque_fault_counter < 8'd255) torque_fault_counter <= torque_fault_counter + 8'd1;
            end else if (torque_fault_counter > 8'd0) torque_fault_counter <= torque_fault_counter - 8'd1;
        end
    end

    // ==================== REGEN CALCULATION ============
    always @(posedge clk_ai or negedge rst_ai_n) begin : regen_calc
        if (!rst_ai_n) begin
            requested_regen <= 16'd0;
            regen_available <= 16'd0;
            regen_efficiency <= 8'd0;
            regen_fault_counter <= 8'd0;
        end else begin
            requested_regen <= (brake_pressure * MAX_REGEN) / 16'd1000;
            regen_available <= MAX_REGEN;
            if (vehicle_speed < MIN_REGEN_SPEED)
                regen_available <= (regen_available * vehicle_speed) / MIN_REGEN_SPEED;
            if (battery_soc > 16'd900)
                regen_available <= (regen_available * (16'd1000 - battery_soc)) / 16'd100;
            if (motor_temp > TEMP_REDUCE_TORQUE)
                regen_available <= (regen_available * (TEMP_CUTOFF - motor_temp)) / 
                                  (TEMP_CUTOFF - TEMP_REDUCE_TORQUE);
            if ((vehicle_speed > 16'd300) && (vehicle_speed < 16'd1000)) regen_efficiency <= 8'd80;
            else if (vehicle_speed > 16'd100) regen_efficiency <= 8'd60;
            else regen_efficiency <= 8'd30;
            if (requested_regen > regen_available) requested_regen <= regen_available;
            if ((requested_regen > 16'd0) && (vehicle_speed == 16'd0)) begin
                if (regen_fault_counter < 8'd255) regen_fault_counter <= regen_fault_counter + 8'd1;
            end else if (regen_fault_counter > 8'd0) regen_fault_counter <= regen_fault_counter - 8'd1;
        end
    end

    // ==================== STEERING ASSIST ==============
    always @(posedge clk_ai or negedge rst_ai_n) begin : steering
        if (!rst_ai_n) begin
            steering_torque <= 16'd0;
            steering_damping <= 16'd0;
            steering_assist <= 16'd0;
        end else if ((current_mode == `MODE_CAR) && dynamics_ok) begin
            steering_torque <= 16'd100;
            if (vehicle_speed > 16'd1000) steering_damping <= 16'd100;
            else if (vehicle_speed > 16'd500) steering_damping <= 16'd50;
            else steering_damping <= 16'd20;
            if (steering_torque > steering_damping) steering_assist <= steering_torque - steering_damping;
            else steering_assist <= 16'd0;
            if (steering_assist > 16'd200) steering_assist <= 16'd200;
        end else steering_assist <= 16'd0;
    end

    // ==================== LIMITS CALCULATION ===========
    always @(posedge clk_ai or negedge rst_ai_n) begin : limits
        if (!rst_ai_n) begin
            thermal_limit <= MAX_TORQUE_CAR;
            soc_limit <= MAX_TORQUE_CAR;
            speed_limit <= MAX_TORQUE_CAR;
            combined_limit <= MAX_TORQUE_CAR;
            torque_limit <= MAX_TORQUE_CAR;
            regen_limit <= MAX_REGEN;
            power_limit <= MAX_TORQUE_CAR;
        end else begin
            if (motor_temp >= TEMP_CUTOFF) thermal_limit <= 16'd0;
            else if (motor_temp >= TEMP_LIMIT_TORQUE)
                thermal_limit <= (MAX_TORQUE_CAR * (TEMP_CUTOFF - motor_temp)) / 
                               (TEMP_CUTOFF - TEMP_LIMIT_TORQUE);
            else if (motor_temp >= TEMP_REDUCE_TORQUE) thermal_limit <= (MAX_TORQUE_CAR * 8'd8) / 8'd10;
            else thermal_limit <= MAX_TORQUE_CAR;
            if (battery_soc <= SOC_CRITICAL_LIMIT) soc_limit <= MAX_TORQUE_SAFE;
            else if (battery_soc <= SOC_LOW_LIMIT)
                soc_limit <= (MAX_TORQUE_CAR * (battery_soc - SOC_CRITICAL_LIMIT)) / 
                           (SOC_LOW_LIMIT - SOC_CRITICAL_LIMIT);
            else soc_limit <= MAX_TORQUE_CAR;
            case (current_mode)
                `MODE_CAR: begin
                    if (motor_rpm >= MAX_RPM_CAR) speed_limit <= 16'd0;
                    else if (motor_rpm >= (MAX_RPM_CAR * 9 / 10))
                        speed_limit <= (MAX_TORQUE_CAR * (MAX_RPM_CAR - motor_rpm)) / 
                                     (MAX_RPM_CAR / 10);
                    else speed_limit <= MAX_TORQUE_CAR;
                end
                `MODE_BIKE: begin
                    if (motor_rpm >= MAX_RPM_BIKE) speed_limit <= 16'd0;
                    else if (motor_rpm >= (MAX_RPM_BIKE * 9 / 10))
                        speed_limit <= (MAX_TORQUE_BIKE * (MAX_RPM_BIKE - motor_rpm)) / 
                                     (MAX_RPM_BIKE / 10);
                    else speed_limit <= MAX_TORQUE_BIKE;
                end
                `MODE_SAFE: speed_limit <= MAX_TORQUE_SAFE;
                default: speed_limit <= MAX_TORQUE_CAR;
            endcase
            combined_limit <= thermal_limit;
            if (soc_limit < combined_limit) combined_limit <= soc_limit;
            if (speed_limit < combined_limit) combined_limit <= speed_limit;
            torque_limit <= combined_limit;
            regen_limit <= regen_available;
            power_limit <= combined_limit;
        end
    end

    // ==================== CONTROL STATE MACHINE ========
    always @(posedge clk_ai or negedge rst_ai_n) begin : control_fsm_seq
        if (!rst_ai_n) begin
            control_state <= MODE_TORQUE;
            control_timer <= 32'd0;
            torque_ramp_counter <= 16'd0;
            regen_ramp_counter <= 16'd0;
        end else begin
            control_state <= next_control_state;
            if (control_timer < 32'hFFFFFFFF) control_timer <= control_timer + 32'd1;
            if (torque_ramp_counter < limited_torque) torque_ramp_counter <= torque_ramp_counter + 16'd10;
            else if (torque_ramp_counter > limited_torque) torque_ramp_counter <= torque_ramp_counter - 16'd10;
            if (regen_ramp_counter < requested_regen) regen_ramp_counter <= regen_ramp_counter + 16'd5;
            else if (regen_ramp_counter > requested_regen) regen_ramp_counter <= regen_ramp_counter - 16'd5;
        end
    end

    always @(*) begin : control_fsm_next
        next_control_state = control_state;
        if (!motor_ok) next_control_state = MODE_EMERGENCY;
        else if (!thermal_ok) next_control_state = MODE_LIMITED;
        else if (adas_cruise_control) next_control_state = MODE_SPEED;
        else if (throttle_fault_counter > 8'd100) next_control_state = MODE_RECOVERY;
        else next_control_state = MODE_TORQUE;
    end

    // ==================== OUTPUT LOGIC =================
    always @(posedge clk_ai or negedge rst_ai_n) begin : outputs
        if (!rst_ai_n) begin
            torque_command <= 16'd0;
            regen_command <= 16'd0;
        end else begin
            torque_command <= torque_ramp_counter;
            regen_command <= regen_ramp_counter;
            if (torque_command > combined_limit) torque_command <= combined_limit;
            if (regen_command > regen_available) regen_command <= regen_available;
            if (adas_auto_brake) begin
                torque_command <= 16'd0;
                regen_command <= MAX_REGEN;
            end
            if (control_state == MODE_EMERGENCY) begin
                torque_command <= 16'd0;
                regen_command <= 16'd0;
            end
        end
    end

endmodule