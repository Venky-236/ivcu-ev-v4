// central_safety_fsm_v3.v
`timescale 1ns/1ps
`include "defines_ivcu_ev_v3.sv"   // FIX: file is defines_ivcu_ev_v3.sv, .v version never existed

module central_safety_fsm_v3 (
    input  wire        clk_aon,
    input  wire        rst_aon_n,
    input  wire        battery_health_ok,
    input  wire        thermal_ok,
    input  wire        motor_ok,
    input  wire        dynamics_ok,
    input  wire        perception_ok,
    input  wire        crash_latched,
    input  wire [7:0]  battery_score,
    input  wire [7:0]  thermal_score,
    input  wire [7:0]  motor_score,
    input  wire [7:0]  dynamics_score,
    input  wire [7:0]  perception_score,
    input  wire [3:0]  battery_status,
    input  wire [3:0]  thermal_status,
    input  wire [3:0]  motor_status,
    input  wire [3:0]  dynamics_status,
    input  wire [3:0]  perception_status,
    input  wire        emergency_stop,
    input  wire        manual_override,
    input  wire [1:0]  current_mode,
    output reg         vehicle_enable,
    output reg         motor_enable,
    output reg         brake_control,
    output reg         throttle_limit,
    output reg         hazard_lights,
    output reg         door_unlock,
    output reg         airbag_control,
    output reg         emergency_ack,
    output reg  [3:0]  alert_level,
    output reg  [15:0] control_signals,
    output reg  [7:0]  fault_code,
    output reg  [31:0] fault_timestamp,
    input  wire [2:0]  debug_mode
);

    // ==================== PARAMETERS ====================
    localparam [3:0] STATE_POWER_OFF = 4'b0000;
    localparam [3:0] STATE_STANDBY   = 4'b0001;
    localparam [3:0] STATE_NORMAL    = 4'b0010;
    localparam [3:0] STATE_WARNING   = 4'b0011;
    localparam [3:0] STATE_CRITICAL  = 4'b0100;
    localparam [3:0] STATE_EMERGENCY = 4'b0101;
    localparam [3:0] STATE_FAULT     = 4'b0110;
    localparam [3:0] STATE_SAFE_MODE = 4'b0111;
    localparam [3:0] STATE_SHUTDOWN  = 4'b1000;
    localparam [3:0] STATE_RECOVERY  = 4'b1001;

    localparam [3:0] ALERT_NONE      = 4'b0000;
    localparam [3:0] ALERT_INFO      = 4'b0001;
    localparam [3:0] ALERT_WARNING   = 4'b0010;
    localparam [3:0] ALERT_CRITICAL  = 4'b0011;
    localparam [3:0] ALERT_EMERGENCY = 4'b0100;
    localparam [3:0] ALERT_FAULT     = 4'b0101;

    localparam CTRL_VEHICLE_EN     = 0;
    localparam CTRL_MOTOR_EN       = 1;
    localparam CTRL_BRAKE_CTRL     = 2;
    localparam CTRL_THROTTLE_LIMIT = 3;
    localparam CTRL_HAZARD_LIGHTS  = 4;
    localparam CTRL_DOOR_UNLOCK    = 5;
    localparam CTRL_AIRBAG_CTRL    = 6;
    localparam CTRL_COOLING_MAX    = 7;
    localparam CTRL_POWER_REDUCE   = 8;
    localparam CTRL_SPEED_LIMIT    = 9;

    localparam PRIO_CRASH     = 5;
    localparam PRIO_EMERGENCY = 4;
    localparam PRIO_THERMAL   = 3;
    localparam PRIO_BATTERY   = 2;
    localparam PRIO_MOTOR     = 1;
    localparam PRIO_OTHER     = 0;

    // ==================== INTERNAL REGISTERS ============
    reg [3:0] current_state;
    reg [3:0] next_state;
    reg [3:0] previous_state;
    reg battery_ok_sync;
    reg thermal_ok_sync;
    reg motor_ok_sync;
    reg dynamics_ok_sync;
    reg perception_ok_sync;
    reg crash_sync;
    reg [31:0] state_timer;
    reg [31:0] warning_timer;
    reg [31:0] emergency_timer;
    reg [7:0]  active_faults [0:7];
    reg [2:0]  fault_ptr;
    reg [31:0] system_uptime;
    reg [2:0]  highest_priority;
    reg [7:0]  priority_score [0:5];
    reg [15:0] car_mode_rules;
    reg [15:0] bike_mode_rules;
    reg [7:0]  recovery_attempts;
    reg [31:0] recovery_timer;

    // FIX (multiple-driver): loop counter `i` and scratch `ctrl_tmp` were
    // declared here at module scope and written from several clocked always
    // blocks. Each block inferred its own flip-flop and all of them drove the
    // same net -- Yosys `check` reported this as "multiple conflicting drivers".
    // Both are now declared inside the named blocks that use them, which
    // Verilog-2001 permits for a labelled begin/end. No other change.

    // ==================== INPUT SYNCHRONIZATION ========
    always @(posedge clk_aon or negedge rst_aon_n) begin : input_sync
        if (!rst_aon_n) begin
            battery_ok_sync <= 1'b1;
            thermal_ok_sync <= 1'b1;
            motor_ok_sync <= 1'b1;
            dynamics_ok_sync <= 1'b1;
            perception_ok_sync <= 1'b1;
            crash_sync <= 1'b0;
        end else begin
            battery_ok_sync <= battery_health_ok;
            thermal_ok_sync <= thermal_ok;
            motor_ok_sync <= motor_ok;
            dynamics_ok_sync <= dynamics_ok;
            perception_ok_sync <= perception_ok;
            crash_sync <= crash_latched;
        end
    end

    // ==================== PRIORITY RESOLUTION ==========
    always @(posedge clk_aon or negedge rst_aon_n) begin : priority_resolution
        integer i;
        if (!rst_aon_n) begin
            highest_priority <= 3'd0;
            for (i = 0; i < 6; i = i + 1) priority_score[i] <= 8'd0;
        end else begin
            if (crash_sync) priority_score[PRIO_CRASH] <= 8'd100;
            else priority_score[PRIO_CRASH] <= 8'd0;
            if (emergency_stop) priority_score[PRIO_EMERGENCY] <= 8'd100;
            else priority_score[PRIO_EMERGENCY] <= 8'd0;
            if (thermal_status == `STATUS_EMERGENCY) priority_score[PRIO_THERMAL] <= 8'd100;
            else if (thermal_status == `STATUS_CRITICAL) priority_score[PRIO_THERMAL] <= 8'd75;
            else if (thermal_status == `STATUS_WARNING) priority_score[PRIO_THERMAL] <= 8'd50;
            else priority_score[PRIO_THERMAL] <= 8'd0;
            if (battery_status == `STATUS_EMERGENCY) priority_score[PRIO_BATTERY] <= 8'd90;
            else if (battery_status == `STATUS_CRITICAL) priority_score[PRIO_BATTERY] <= 8'd70;
            else if (battery_status == `STATUS_WARNING) priority_score[PRIO_BATTERY] <= 8'd40;
            else priority_score[PRIO_BATTERY] <= 8'd0;
            if (motor_status == `STATUS_EMERGENCY) priority_score[PRIO_MOTOR] <= 8'd80;
            else if (motor_status == `STATUS_CRITICAL) priority_score[PRIO_MOTOR] <= 8'd60;
            else if (motor_status == `STATUS_WARNING) priority_score[PRIO_MOTOR] <= 8'd30;
            else priority_score[PRIO_MOTOR] <= 8'd0;
            if ((dynamics_status == `STATUS_EMERGENCY) || (perception_status == `STATUS_EMERGENCY))
                priority_score[PRIO_OTHER] <= 8'd60;
            else if ((dynamics_status == `STATUS_CRITICAL) || (perception_status == `STATUS_CRITICAL))
                priority_score[PRIO_OTHER] <= 8'd40;
            else priority_score[PRIO_OTHER] <= 8'd0;
            highest_priority <= 3'd0;
            for (i = 1; i < 6; i = i + 1) begin
                if (priority_score[i] > priority_score[highest_priority]) highest_priority <= i;
            end
        end
    end

    // ==================== FSM STATE TRANSITION =========
    always @(posedge clk_aon or negedge rst_aon_n) begin : fsm_seq
        if (!rst_aon_n) begin
            current_state <= STATE_POWER_OFF;
            previous_state <= STATE_POWER_OFF;
            state_timer <= 32'd0;
            warning_timer <= 32'd0;
            emergency_timer <= 32'd0;
            system_uptime <= 32'd0;
            recovery_attempts <= 8'd0;
            recovery_timer <= 32'd0;
        end else begin
            current_state <= next_state;
            previous_state <= current_state;
            if (state_timer < 32'hFFFFFFFF) state_timer <= state_timer + 32'd1;
            if (system_uptime < 32'hFFFFFFFF) system_uptime <= system_uptime + 32'd1;
            if (current_state == STATE_WARNING) begin
                if (warning_timer < 32'hFFFFFFFF) warning_timer <= warning_timer + 32'd1;
            end else warning_timer <= 32'd0;
            if (current_state == STATE_EMERGENCY) begin
                if (emergency_timer < 32'hFFFFFFFF) emergency_timer <= emergency_timer + 32'd1;
            end else emergency_timer <= 32'd0;
            if (current_state == STATE_RECOVERY) begin
                if (recovery_timer < 32'hFFFFFFFF) recovery_timer <= recovery_timer + 32'd1;
            end else recovery_timer <= 32'd0;
            if (current_state != next_state) state_timer <= 32'd0;
            // Update recovery attempts
            if ((next_state == STATE_FAULT) && (current_state == STATE_RECOVERY)) begin
                if (recovery_attempts < 8'd255) recovery_attempts <= recovery_attempts + 8'd1;
            end else if (current_state == STATE_NORMAL) begin
                recovery_attempts <= 8'd0;
            end
        end
    end

    // ==================== FSM NEXT STATE LOGIC =========
    always @(*) begin : fsm_next
        next_state = current_state;
        case (current_state)
            STATE_POWER_OFF: begin
                if (battery_ok_sync && thermal_ok_sync && motor_ok_sync &&
                    dynamics_ok_sync && perception_ok_sync) next_state = STATE_STANDBY;
            end
            STATE_STANDBY: begin
                if (crash_sync || emergency_stop) next_state = STATE_EMERGENCY;
                else if (!battery_ok_sync || !thermal_ok_sync || !motor_ok_sync) next_state = STATE_CRITICAL;
                else if (!dynamics_ok_sync || !perception_ok_sync) next_state = STATE_WARNING;
                else if (state_timer > 32'd1000) next_state = STATE_NORMAL;
            end
            STATE_NORMAL: begin
                if (crash_sync || emergency_stop) next_state = STATE_EMERGENCY;
                else if (!battery_ok_sync || !thermal_ok_sync || !motor_ok_sync) next_state = STATE_CRITICAL;
                else if (!dynamics_ok_sync || !perception_ok_sync) next_state = STATE_WARNING;
                else if (manual_override) next_state = STATE_SAFE_MODE;
            end
            STATE_WARNING: begin
                if (crash_sync || emergency_stop) next_state = STATE_EMERGENCY;
                else if (!battery_ok_sync || !thermal_ok_sync || !motor_ok_sync) next_state = STATE_CRITICAL;
                else if (dynamics_ok_sync && perception_ok_sync) begin
                    if (warning_timer > 32'd5000) next_state = STATE_NORMAL;
                end else if (warning_timer > 32'd30000) next_state = STATE_CRITICAL;
            end
            STATE_CRITICAL: begin
                if (crash_sync || emergency_stop) next_state = STATE_EMERGENCY;
                else if (battery_ok_sync && thermal_ok_sync && motor_ok_sync) begin
                    if (state_timer > 32'd10000) next_state = STATE_WARNING;
                end else if (state_timer > 32'd50000) next_state = STATE_FAULT;
            end
            STATE_EMERGENCY: begin
                if (!crash_sync && !emergency_stop) begin
                    if (emergency_timer > 32'd10000) next_state = STATE_RECOVERY;
                end else if (emergency_timer > 32'd30000) next_state = STATE_SHUTDOWN;
            end
            STATE_FAULT: begin
                if (state_timer > 32'd100000) begin
                    if (recovery_attempts < 8'd3) next_state = STATE_RECOVERY;
                    else next_state = STATE_SHUTDOWN;
                end
            end
            STATE_SAFE_MODE: begin
                if (!manual_override) begin
                    if (battery_ok_sync && thermal_ok_sync && motor_ok_sync) next_state = STATE_NORMAL;
                    else next_state = STATE_CRITICAL;
                end
            end
            STATE_SHUTDOWN: next_state = STATE_SHUTDOWN;
            STATE_RECOVERY: begin
                if (recovery_timer > 32'd5000) begin
                    if (battery_ok_sync && thermal_ok_sync && motor_ok_sync) next_state = STATE_NORMAL;
                    else if (recovery_timer > 32'd20000) next_state = STATE_FAULT;
                end
            end
            default: next_state = STATE_POWER_OFF;
        endcase
    end

    // ==================== CONTROL OUTPUT LOGIC =========
    always @(posedge clk_aon or negedge rst_aon_n) begin : control_outputs
        reg [15:0] ctrl_tmp;  // temp for control signal building
        if (!rst_aon_n) begin
            vehicle_enable <= 1'b0;
            motor_enable <= 1'b0;
            brake_control <= 1'b0;
            throttle_limit <= 1'b0;
            hazard_lights <= 1'b0;
            door_unlock <= 1'b0;
            airbag_control <= 1'b0;
            emergency_ack <= 1'b0;
            alert_level <= ALERT_NONE;
            control_signals <= 16'd0;
        end else begin
            ctrl_tmp = 16'd0;
            case (current_state)
                STATE_POWER_OFF: alert_level <= ALERT_NONE;
                STATE_STANDBY: begin
                    vehicle_enable <= 1'b1;
                    alert_level <= ALERT_INFO;
                    ctrl_tmp[CTRL_VEHICLE_EN] = 1'b1;
                end
                STATE_NORMAL: begin
                    vehicle_enable <= 1'b1;
                    motor_enable <= 1'b1;
                    alert_level <= ALERT_NONE;
                    ctrl_tmp[CTRL_VEHICLE_EN] = 1'b1;
                    ctrl_tmp[CTRL_MOTOR_EN] = 1'b1;
                end
                STATE_WARNING: begin
                    vehicle_enable <= 1'b1;
                    motor_enable <= 1'b1;
                    throttle_limit <= 1'b1;
                    alert_level <= ALERT_WARNING;
                    ctrl_tmp[CTRL_VEHICLE_EN] = 1'b1;
                    ctrl_tmp[CTRL_MOTOR_EN] = 1'b1;
                    ctrl_tmp[CTRL_THROTTLE_LIMIT] = 1'b1;
                    if ((current_mode == `MODE_CAR) && !perception_ok_sync) ctrl_tmp[CTRL_SPEED_LIMIT] = 1'b1;
                end
                STATE_CRITICAL: begin
                    vehicle_enable <= 1'b1;
                    throttle_limit <= 1'b1;
                    hazard_lights <= 1'b1;
                    alert_level <= ALERT_CRITICAL;
                    ctrl_tmp[CTRL_VEHICLE_EN] = 1'b1;
                    ctrl_tmp[CTRL_THROTTLE_LIMIT] = 1'b1;
                    ctrl_tmp[CTRL_HAZARD_LIGHTS] = 1'b1;
                    ctrl_tmp[CTRL_POWER_REDUCE] = 1'b1;
                    if (!thermal_ok_sync) ctrl_tmp[CTRL_COOLING_MAX] = 1'b1;
                end
                STATE_EMERGENCY: begin
                    brake_control <= 1'b1;
                    hazard_lights <= 1'b1;
                    door_unlock <= 1'b1;
                    emergency_ack <= 1'b1;
                    alert_level <= ALERT_EMERGENCY;
                    ctrl_tmp[CTRL_BRAKE_CTRL] = 1'b1;
                    ctrl_tmp[CTRL_HAZARD_LIGHTS] = 1'b1;
                    ctrl_tmp[CTRL_DOOR_UNLOCK] = 1'b1;
                    if (crash_sync && (priority_score[PRIO_CRASH] > 8'd50)) begin
                        airbag_control <= 1'b1;
                        ctrl_tmp[CTRL_AIRBAG_CTRL] = 1'b1;
                    end
                end
                STATE_FAULT: begin
                    hazard_lights <= 1'b1;
                    alert_level <= ALERT_FAULT;
                    ctrl_tmp[CTRL_HAZARD_LIGHTS] = 1'b1;
                end
                STATE_SAFE_MODE: begin
                    vehicle_enable <= 1'b1;
                    motor_enable <= 1'b1;
                    throttle_limit <= 1'b1;
                    alert_level <= ALERT_WARNING;
                    ctrl_tmp[CTRL_VEHICLE_EN] = 1'b1;
                    ctrl_tmp[CTRL_MOTOR_EN] = 1'b1;
                    ctrl_tmp[CTRL_THROTTLE_LIMIT] = 1'b1;
                    ctrl_tmp[CTRL_SPEED_LIMIT] = 1'b1;
                end
                STATE_SHUTDOWN: begin
                    door_unlock <= 1'b1;
                    alert_level <= ALERT_EMERGENCY;
                    ctrl_tmp[CTRL_DOOR_UNLOCK] = 1'b1;
                end
                STATE_RECOVERY: begin
                    vehicle_enable <= 1'b1;
                    alert_level <= ALERT_WARNING;
                    ctrl_tmp[CTRL_VEHICLE_EN] = 1'b1;
                end
                default: ;
            endcase
            // Apply priority-based overrides
            case (highest_priority)
                PRIO_CRASH: begin
                    brake_control <= 1'b1;
                    hazard_lights <= 1'b1;
                    door_unlock <= 1'b1;
                    if (priority_score[PRIO_CRASH] > 8'd70) airbag_control <= 1'b1;
                end
                PRIO_EMERGENCY: begin
                    brake_control <= 1'b1;
                    motor_enable <= 1'b0;
                end
                PRIO_THERMAL: begin
                    if (priority_score[PRIO_THERMAL] > 8'd80) begin
                        motor_enable <= 1'b0;
                        ctrl_tmp[CTRL_COOLING_MAX] = 1'b1;
                    end
                end
                PRIO_BATTERY: begin
                    if (priority_score[PRIO_BATTERY] > 8'd80) begin
                        throttle_limit <= 1'b1;
                        ctrl_tmp[CTRL_POWER_REDUCE] = 1'b1;
                    end
                end
                default: ;
            endcase
            // Mode-specific overrides
            if (current_mode == `MODE_BIKE) begin
                ctrl_tmp[CTRL_AIRBAG_CTRL] = 1'b0;
                ctrl_tmp[CTRL_SPEED_LIMIT] = 1'b1;
            end
            control_signals <= ctrl_tmp;
        end
    end

    // ==================== FAULT LOGGING ================
    always @(posedge clk_aon or negedge rst_aon_n) begin : fault_logging
        integer i;
        if (!rst_aon_n) begin
            for (i = 0; i < 8; i = i + 1) active_faults[i] <= 8'd0;
            fault_ptr <= 3'd0;
            fault_code <= `FAULT_NONE;
            fault_timestamp <= 32'd0;
        end else begin
            if (current_state != next_state) begin
                if ((next_state == STATE_FAULT) || (next_state == STATE_EMERGENCY) ||
                    (next_state == STATE_CRITICAL)) begin
                    case (highest_priority)
                        PRIO_CRASH:     fault_code <= `FAULT_CRASH;
                        PRIO_THERMAL:   fault_code <= `FAULT_THERMAL;
                        PRIO_BATTERY:   fault_code <= `FAULT_BATTERY_TEMP;
                        PRIO_MOTOR:     fault_code <= `FAULT_MOTOR_TEMP;
                        default:        fault_code <= `FAULT_SENSOR_FAIL;
                    endcase
                    active_faults[fault_ptr] <= fault_code;
                    fault_ptr <= fault_ptr + 3'd1;
                    fault_timestamp <= system_uptime;
                end
            end
            if ((next_state == STATE_NORMAL) && (current_state != STATE_NORMAL)) fault_code <= `FAULT_NONE;
        end
    end

    // ==================== MODE-SPECIFIC RULES (now empty, overrides moved to control_outputs) ==========
    always @(posedge clk_aon or negedge rst_aon_n) begin : mode_specific
        if (!rst_aon_n) begin
            car_mode_rules <= 16'hFFFF;
            bike_mode_rules <= 16'h0FFF;
        end else begin
            car_mode_rules <= 16'hFFFF;
            bike_mode_rules <= 16'h0FFF;
        end
    end

endmodule
