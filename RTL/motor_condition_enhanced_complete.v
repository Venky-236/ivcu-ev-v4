// motor_condition_enhanced_complete.v – synthesizable implementation
`timescale 1ns/1ps
`include "defines_ivcu_ev_v3.sv"   // FIX: file is defines_ivcu_ev_v3.sv, .v version never existed

module motor_condition_enhanced_complete (
    input  wire        clk_ai,
    input  wire        rst_ai_n,
    input  wire [15:0] motor_temp,
    input  wire        motor_temp_valid,
    input  wire [15:0] inverter_temp,
    input  wire        inverter_temp_valid,
    input  wire [15:0] motor_coolant_temp,
    input  wire        motor_coolant_valid,
    input  wire [15:0] inverter_coolant_temp,
    input  wire        inverter_coolant_valid,
    input  wire [15:0] motor_rpm,
    input  wire        motor_rpm_valid,
    input  wire [15:0] rotor_position,
    input  wire        rotor_position_valid,
    input  wire [15:0] wheel_speed_front,
    input  wire        wheel_speed_front_valid,
    input  wire [15:0] wheel_speed_rear,
    input  wire        wheel_speed_rear_valid,
    input  wire [15:0] throttle_position,
    input  wire        throttle_valid,
    input  wire [15:0] brake_pressure,
    input  wire        brake_pressure_valid,
    input  wire        brake_switch,
    input  wire [15:0] motor_current,
    input  wire        motor_current_valid,
    input  wire [15:0] motor_voltage,
    input  wire        motor_voltage_valid,
    input  wire [15:0] inverter_dc_voltage,
    input  wire        inverter_voltage_valid,
    input  wire [15:0] vibration_level,
    input  wire        vibration_valid,
    input  wire [15:0] acoustic_noise,
    input  wire        acoustic_valid,
    input  wire [15:0] tire_pressure_front,
    input  wire        tpms_front_valid,
    input  wire [15:0] tire_pressure_rear,
    input  wire        tpms_rear_valid,
    input  wire        vehicle_on,
    input  wire [15:0] vehicle_speed,
    input  wire [31:0] vehicle_odometer,
    input  wire [1:0]  vehicle_mode,
    input  wire [15:0] motor_max_rpm,
    input  wire [15:0] motor_max_torque,
    input  wire [15:0] motor_nominal_current,
    output reg  [7:0]  motor_health_score,
    output reg         motor_critical,
    output reg  [7:0]  motor_efficiency,
    output reg  [7:0]  bearing_wear_level,
    output reg  [7:0]  winding_insulation_health,
    output reg  [7:0]  torque_ripple_level,
    output reg  [7:0]  cooling_system_effectiveness,
    output reg  [7:0]  predicted_motor_failures_30d,
    output reg  [15:0] bearing_life_remaining,
    output reg  [15:0] estimated_motor_life_km,
    output reg  [7:0]  optimal_temperature_range,
    output reg         bearing_wear_detected,
    output reg         winding_degradation_detected,
    output reg         cooling_system_issue,
    output reg         rotor_imbalance_detected,
    output reg         brake_pad_wear_detected,
    output reg         tire_wear_detected,
    output reg         reduce_load_recommended,
    output reg         service_motor_recommended,
    output reg         replace_bearings_soon,
    output reg         check_cooling_system,
    output reg         motor_ok,
    output reg  [31:0] efficiency_map,
    output reg  [31:0] debug_temp_vs_load,
    output reg  [31:0] debug_vibration_spectrum,
    output reg  [31:0] debug_efficiency_trend
);

    // ---------- Internal registers ----------
    reg [15:0] temp_score, rpm_score, current_score, voltage_score, vibration_score;
    reg [15:0] brake_wear, tire_wear;
    reg [31:0] runtime_counter;
    reg [15:0] prev_motor_temp, prev_vibration;
    reg [7:0]  temp_trend, vibration_trend;

    // ---------- Health scoring ----------
    always @(posedge clk_ai or negedge rst_ai_n) begin
        if (!rst_ai_n) begin
            temp_score <= 16'd100;
            rpm_score <= 16'd100;
            current_score <= 16'd100;
            voltage_score <= 16'd100;
            vibration_score <= 16'd100;
            brake_wear <= 16'd0;
            tire_wear <= 16'd0;
            runtime_counter <= 32'd0;
            prev_motor_temp <= 16'd0;
            prev_vibration <= 16'd0;
            temp_trend <= 8'd0;
            vibration_trend <= 8'd0;
            motor_health_score <= 8'd100;
            motor_critical <= 1'b0;
            motor_efficiency <= 8'd90;
            bearing_wear_level <= 8'd0;
            winding_insulation_health <= 8'd100;
            torque_ripple_level <= 8'd10;
            cooling_system_effectiveness <= 8'd80;
            predicted_motor_failures_30d <= 8'd0;
            bearing_life_remaining <= 16'd10000;
            estimated_motor_life_km <= 16'd50000;
            optimal_temperature_range <= 8'd70;
            bearing_wear_detected <= 1'b0;
            winding_degradation_detected <= 1'b0;
            cooling_system_issue <= 1'b0;
            rotor_imbalance_detected <= 1'b0;
            brake_pad_wear_detected <= 1'b0;
            tire_wear_detected <= 1'b0;
            reduce_load_recommended <= 1'b0;
            service_motor_recommended <= 1'b0;
            replace_bearings_soon <= 1'b0;
            check_cooling_system <= 1'b0;
            motor_ok <= 1'b1;
            efficiency_map <= 32'd0;
            debug_temp_vs_load <= 32'd0;
            debug_vibration_spectrum <= 32'd0;
            debug_efficiency_trend <= 32'd0;
        end else begin
            // Temperature score (motor_temp in 0.1°C, critical >1200, warning >1000)
            if (motor_temp_valid) begin
                if (motor_temp >= 16'd1200) temp_score <= 16'd0;
                else if (motor_temp >= 16'd1000) temp_score <= (16'd1200 - motor_temp) * 16'd100 / 16'd200;
                else if (motor_temp >= 16'd800) temp_score <= 16'd80;
                else temp_score <= 16'd100;

                if (motor_temp > prev_motor_temp) temp_trend <= temp_trend + 8'd1;
                else if (temp_trend > 0) temp_trend <= temp_trend - 1;
                prev_motor_temp <= motor_temp;
            end

            // RPM score
            if (motor_rpm_valid && motor_max_rpm > 0) begin
                if (motor_rpm >= motor_max_rpm) rpm_score <= 16'd0;
                else rpm_score <= (motor_max_rpm - motor_rpm) * 16'd100 / motor_max_rpm;
            end

            // Current score
            if (motor_current_valid && motor_nominal_current > 0) begin
                if (motor_current >= motor_nominal_current * 2) current_score <= 16'd0;
                else if (motor_current > motor_nominal_current)
                    current_score <= (motor_nominal_current * 2 - motor_current) * 16'd100 / motor_nominal_current;
                else current_score <= 16'd100;
            end

            // Voltage score (nominal 12V -> 1200)
            if (motor_voltage_valid) begin
                if (motor_voltage < 16'd900 || motor_voltage > 16'd1500) voltage_score <= 16'd0;
                else if (motor_voltage < 16'd1000 || motor_voltage > 16'd1400) voltage_score <= 16'd50;
                else voltage_score <= 16'd100;
            end

            // Vibration score
            if (vibration_valid) begin
                if (vibration_level >= 16'd1000) vibration_score <= 16'd0;
                else if (vibration_level >= 16'd500) vibration_score <= (16'd1000 - vibration_level) * 16'd100 / 16'd500;
                else vibration_score <= 16'd100;

                if (vibration_level > prev_vibration) vibration_trend <= vibration_trend + 8'd1;
                else if (vibration_trend > 0) vibration_trend <= vibration_trend - 1;
                prev_vibration <= vibration_level;
            end

            // Brake wear estimation from brake pressure and odometer
            if (brake_pressure_valid && vehicle_on) begin
                if (brake_pressure > 16'd800) brake_wear <= brake_wear + 16'd1;
            end
            if (vehicle_odometer[15:0] > 16'd50000) brake_pad_wear_detected <= 1'b1;

            // Tire wear from odometer
            if (vehicle_odometer[15:0] > 16'd40000) tire_wear_detected <= 1'b1;

            // Bearing wear from runtime and vibration
            if (vehicle_on) runtime_counter <= runtime_counter + 1;
            bearing_wear_level <= (runtime_counter[23:16] + vibration_trend) / 2;
            if (bearing_wear_level > 8'd80) bearing_wear_detected <= 1'b1;

            // Combined health score (0-100)
            motor_health_score <= (temp_score[7:0]   * 3 +
                                   rpm_score[7:0]    * 2 +
                                   current_score[7:0]* 2 +
                                   voltage_score[7:0]* 1 +
                                   vibration_score[7:0]*2) / 10;

            // Motor OK threshold
            motor_ok <= (motor_health_score >= 8'd40);
            motor_critical <= (motor_health_score < 8'd20);

            // Derived outputs
            motor_efficiency <= (motor_health_score * 9 / 10);
            cooling_system_effectiveness <= (motor_coolant_valid ? 8'd75 : 8'd50);
            torque_ripple_level <= (motor_current_valid ? (motor_current[7:0] / 4) : 8'd0);
            winding_insulation_health <= (motor_temp_valid && motor_temp < 16'd1000) ? 8'd90 : 8'd60;
            predicted_motor_failures_30d <= (100 - motor_health_score) / 2;
            bearing_life_remaining <= 16'd10000 - (runtime_counter[15:0] / 16'd100);
            estimated_motor_life_km <= 16'd50000 - vehicle_odometer[15:0];
            optimal_temperature_range <= 8'd70;
            cooling_system_issue <= (motor_coolant_valid && motor_coolant_temp > 16'd800) ? 1'b1 : 1'b0;
            rotor_imbalance_detected <= (vibration_trend > 8'd50) ? 1'b1 : 1'b0;
            reduce_load_recommended <= (temp_score < 16'd60 || rpm_score < 16'd60) ? 1'b1 : 1'b0;
            service_motor_recommended <= (motor_health_score < 8'd50) ? 1'b1 : 1'b0;
            replace_bearings_soon <= (bearing_wear_level > 8'd70) ? 1'b1 : 1'b0;
            check_cooling_system <= cooling_system_issue;

            // FIX: winding_degradation_detected and efficiency_map were only ever
            // assigned in the reset branch above - stuck-at-zero for the module's
            // entire operating lifetime (confirmed via simulation: driving
            // motor_temp deep into the critical range for 2000 cycles never moved
            // either signal). Derive them from health metrics already computed
            // this cycle, following the same style as the other debug/status buses.
            winding_degradation_detected <= (winding_insulation_health < 8'd70);
            efficiency_map <= {motor_efficiency, bearing_wear_level,
                               cooling_system_effectiveness, torque_ripple_level};

            // Debug outputs (simplified)
            debug_temp_vs_load <= {motor_temp[15:8], motor_current[15:8], 16'd0};
            debug_vibration_spectrum <= {vibration_level[15:8], vibration_trend, 16'd0};
            debug_efficiency_trend <= {motor_efficiency, motor_health_score, 16'd0};
        end
    end

endmodule
