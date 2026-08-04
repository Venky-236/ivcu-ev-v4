// mode_config_enhanced_v3.sv
`timescale 1ns/1ps
`include "defines_ivcu_ev_v3.sv"
`default_nettype none

module mode_config_enhanced_v3 (
    input  wire        clk_aon,
    input  wire        rst_aon_n,
    input  wire        clk_ai,
    input  wire        rst_ai_n,
    input  wire        mode_switch_car,
    input  wire        mode_switch_bike,
    input  wire        mode_auto_detect,
    input  wire [1:0]  user_mode_override,
    output reg  [1:0]  active_mode,
    output wire        mode_valid,
    output wire        mode_stable,
    output reg         mode_change_ack,
    output reg [41:0]  sensor_map_car,
    output reg [41:0]  sensor_map_bike,
    input  wire [2:0]  debug_mode,
    output wire [3:0]  mode_state,
    output wire        auto_detect_active
);

    localparam [7:0] MODE_STABLE_COUNT = 8'd100;
    localparam [15:0] AUTO_DETECT_TIMEOUT = 16'd5000;

    localparam [2:0] IDLE       = 3'b000;
    localparam [2:0] DETECTING  = 3'b001;
    localparam [2:0] CAR_MODE   = 3'b010;
    localparam [2:0] BIKE_MODE  = 3'b011;
    localparam [2:0] SAFE_MODE  = 3'b100;
    localparam [2:0] TRANSITION = 3'b101;

    reg [2:0] current_state;
    reg [2:0] next_state;
    reg [7:0] stable_counter;
    reg [15:0] auto_detect_counter;
    reg       mode_change_request;
    reg [1:0] requested_mode;

    // Signals that cross from AI domain to AON
    wire detecting_ai;         // from AON to AI? Actually we need a sync from AON to AI
    // We'll use a synchronizer to bring the detection state to AI domain.
    reg detecting_aon;
    wire detecting_sync_ai;
    sync_cell #(.WIDTH(1)) u_detect_sync (.clk_dst(clk_ai), .rst_dst_n(rst_ai_n),
                                          .signal_src(detecting_aon), .signal_dst(detecting_sync_ai));

    // AI domain signature counter
    reg [15:0] sensor_signature;
    reg        signature_valid;
    reg [7:0]  car_count, bike_count;
    reg        detect_done;

    // Signals from AI to AON (after synchronisation)
    wire [15:0] sensor_signature_sync;
    wire        signature_valid_sync;
    wire [7:0]  car_count_sync, bike_count_sync;
    wire        detect_done_sync;

    sync_cell #(.WIDTH(16)) u_sig_sync (.clk_dst(clk_aon), .rst_dst_n(rst_aon_n),
                                        .signal_src(sensor_signature), .signal_dst(sensor_signature_sync));
    sync_cell #(.WIDTH(1))  u_val_sync (.clk_dst(clk_aon), .rst_dst_n(rst_aon_n),
                                        .signal_src(signature_valid), .signal_dst(signature_valid_sync));
    sync_cell #(.WIDTH(8))  u_car_sync (.clk_dst(clk_aon), .rst_dst_n(rst_aon_n),
                                        .signal_src(car_count), .signal_dst(car_count_sync));
    sync_cell #(.WIDTH(8))  u_bike_sync(.clk_dst(clk_aon), .rst_dst_n(rst_aon_n),
                                        .signal_src(bike_count), .signal_dst(bike_count_sync));
    sync_cell #(.WIDTH(1))  u_done_sync(.clk_dst(clk_aon), .rst_dst_n(rst_aon_n),
                                        .signal_src(detect_done), .signal_dst(detect_done_sync));

    // AON domain FSM
    always_ff @(posedge clk_aon or negedge rst_aon_n) begin
        if (!rst_aon_n) begin
            sensor_map_car <= 42'h3FFFFFFFFFF;
            sensor_map_bike <= 42'h0000FFF0FF;
        end else begin
            sensor_map_car <= 42'h3FFFFFFFFFF;
            sensor_map_bike <= 42'h0000FFF0FF;
        end
    end

    always_ff @(posedge clk_aon or negedge rst_aon_n) begin
        if (!rst_aon_n) begin
            current_state <= IDLE;
            active_mode <= `MODE_SAFE;
            stable_counter <= 8'd0;
            detecting_aon <= 1'b0;
        end else begin
            current_state <= next_state;
            detecting_aon <= (current_state == DETECTING);
            if (mode_change_request) stable_counter <= 8'd0;
            else if (stable_counter < MODE_STABLE_COUNT) stable_counter <= stable_counter + 8'd1;
            if (next_state == TRANSITION && current_state != TRANSITION) begin
                // wait
            end else if (current_state == TRANSITION && stable_counter >= MODE_STABLE_COUNT) begin
                case (requested_mode)
                    `MODE_CAR:   active_mode <= `MODE_CAR;
                    `MODE_BIKE:  active_mode <= `MODE_BIKE;
                    `MODE_AUTO:  active_mode <= `MODE_AUTO;
                    default:     active_mode <= `MODE_SAFE;
                endcase
                mode_change_ack <= 1'b1;
            end else begin
                mode_change_ack <= 1'b0;
            end
        end
    end

    always_ff @(posedge clk_aon or negedge rst_aon_n) begin
        if (!rst_aon_n) begin
            mode_change_request <= 1'b0;
            requested_mode <= `MODE_SAFE;
            auto_detect_counter <= 16'd0;
        end else begin
            if (mode_switch_car) begin
                mode_change_request <= 1'b1;
                requested_mode <= `MODE_CAR;
            end else if (mode_switch_bike) begin
                mode_change_request <= 1'b1;
                requested_mode <= `MODE_BIKE;
            end else if (user_mode_override != 2'b11) begin
                mode_change_request <= 1'b1;
                requested_mode <= user_mode_override;
            end
            if (mode_change_ack) mode_change_request <= 1'b0;
            if (current_state == DETECTING) begin
                if (auto_detect_counter < AUTO_DETECT_TIMEOUT) auto_detect_counter <= auto_detect_counter + 16'd1;
            end else auto_detect_counter <= 16'd0;
        end
    end

    always_comb begin
        next_state = current_state;
        case (current_state)
            IDLE: begin
                if (mode_change_request) next_state = TRANSITION;
                else if (mode_auto_detect) next_state = DETECTING;
            end
            DETECTING: begin
                if (detect_done_sync) begin
                    if (car_count_sync > bike_count_sync) next_state = CAR_MODE;
                    else next_state = BIKE_MODE;
                end else if (auto_detect_counter >= AUTO_DETECT_TIMEOUT) begin
                    next_state = SAFE_MODE;
                end
            end
            CAR_MODE: begin
                if (mode_change_request && (requested_mode != `MODE_CAR)) next_state = TRANSITION;
            end
            BIKE_MODE: begin
                if (mode_change_request && (requested_mode != `MODE_BIKE)) next_state = TRANSITION;
            end
            SAFE_MODE: begin
                if (mode_change_request && (requested_mode != `MODE_SAFE)) next_state = TRANSITION;
            end
            TRANSITION: begin
                if (stable_counter >= MODE_STABLE_COUNT) begin
                    case (requested_mode)
                        `MODE_CAR:   next_state = CAR_MODE;
                        `MODE_BIKE:  next_state = BIKE_MODE;
                        `MODE_AUTO:  next_state = DETECTING;
                        default:     next_state = SAFE_MODE;
                    endcase
                end
            end
            default: next_state = IDLE;
        endcase
    end

    // AI domain signature generator
    always_ff @(posedge clk_ai or negedge rst_ai_n) begin
        if (!rst_ai_n) begin
            sensor_signature <= 16'd0;
            signature_valid <= 1'b0;
            car_count <= 8'd0;
            bike_count <= 8'd0;
            detect_done <= 1'b0;
        end else if (detecting_sync_ai) begin
            // Simple counter – in real design, this would read sensors
            sensor_signature <= sensor_signature + 1;
            signature_valid <= 1'b1;
            // Example: detect based on some sensor value – here just count up
            if (sensor_signature[0]) car_count <= car_count + 1;
            else bike_count <= bike_count + 1;
            if (car_count > 8'd50 || bike_count > 8'd50) detect_done <= 1'b1;
        end else begin
            signature_valid <= 1'b0;
            if (!detecting_sync_ai) begin
                car_count <= 8'd0;
                bike_count <= 8'd0;
                detect_done <= 1'b0;
                sensor_signature <= 16'd0;
            end
        end
    end

    assign mode_valid = (current_state != IDLE) && (current_state != TRANSITION);
    assign mode_stable = (stable_counter >= MODE_STABLE_COUNT);
    assign mode_state = {1'b0, current_state};
    assign auto_detect_active = (current_state == DETECTING);

endmodule
`default_nettype wire