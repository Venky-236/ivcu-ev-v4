//=============================================================================
// ivcu_ev_v4_top.v  -  IVCU-EV V4 top level
//
// Intelligent Vehicle Control Unit, dual mode car / motorcycle.
// Sky130 HD, Yosys + OpenROAD.
//
//-----------------------------------------------------------------------------
// THE BOUNDARY, AND WHY IT IS THE HEADLINE NUMBER
//
//     V3   1,344 sensor pins alone, 864 of them unconnected
//          on a 5,840 um perimeter that is 4.3 um per pin
//          which is why pins and wires would not align in the floorplan
//
//     V4   237 pins total, every one named, commented and connected
//          24.6 um per pin
//
// Sky130 met3/met4 pitch is under a micron.  Twenty-four microns per pin is
// not tight; it is luxurious.  That single change - the multiplexed ADC bus in
// ivcu_adc_scan_sequencer - is what turns the physical design from a fight
// into an ordinary job.
//
//-----------------------------------------------------------------------------
// WHAT IS NOT ON THIS BOUNDARY, DELIBERATELY
//
//   power domain enables   power gating is expressed in power intent, not in
//                          RTL ports, and the open-source flow does not
//                          consume UPF.  Adding pd_* pins that nothing drives
//                          would be four more V3-style dead ports.
//   scan chain pins        scan is inserted post-synthesis, and the ports come
//                          with it.  V3 had four DFT pins reaching a module
//                          with zero gates.  Pins without chains are worse
//                          than no pins, because they look like coverage.
//   steering assist        electric power steering is its own ECU.  This chip
//                          reports steering ANGLE health; it does not drive
//                          the motor.
//
//-----------------------------------------------------------------------------
// CLOCK DOMAIN CROSSINGS - THREE PATTERNS, EACH USED ON PURPOSE
//
//  1  PULSE      ivcu_cdc_pulse_sync.  For sweep_done, which is 40 ns wide
//                crossing into a 100 ns domain.  A level synchroniser would
//                silently drop it most of the time.
//
//  2  BIT        ivcu_cdc_bit_sync.  Every single-bit control or status signal
//                that gates a decision in another domain.
//
//  3  QUASI-STATIC  wide buses - the 64-bit masks, the 1024-bit value array,
//                the score and limit buses - cross unregistered, and nothing
//                acts on them except on a synchronised pulse.  Registering
//                them would cost thousands of flops to solve a problem these
//                paths do not have.
//
//                THIS PATTERN DEPENDS ON THE SDC.  ivcu_ev_v4.sdc must keep
//                set_clock_groups -asynchronous.  If that is ever removed,
//                every quasi-static path below becomes a real timing path
//                between unrelated clocks and the design becomes silently
//                wrong.  It is written here and in the SDC, in both places.
//
//                The one place a coherent snapshot IS paid for is
//                ivcu_hv_sense_sync, because the precharge comparison spans
//                two channels and a mixed snapshot welds a contactor.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_ev_v4_top (
    //=========================================================================
    // CLOCKS AND RESET  (6)
    //=========================================================================
    input  wire        clk_aon,        // 10 MHz  always-on, safety island
    input  wire        clk_sensor,     // 25 MHz  acquisition
    input  wire        clk_ai,         // 50 MHz  domain decision blocks
    input  wire        clk_mcu,        // 50 MHz  APB, log, guidance
    input  wire        por_n,          // power-on reset, async, active low
    input  wire        ext_rst_n,      // external / watchdog reset

    //=========================================================================
    // SENSOR ACQUISITION  (21)  - all 64 channels through here
    //=========================================================================
    output wire [5:0]  adc_chan,       // channel being requested
    output wire        adc_req,        // conversion request strobe
    input  wire [11:0] adc_data,       // conversion result
    input  wire        adc_valid,      // result is good
    input  wire        adc_busy,       // ADC is converting

    //=========================================================================
    // ANALOG FRONT-END POWER CHAIN  (3)  - 64 rails, 3 pins
    //=========================================================================
    output wire        afe_sclk,
    output wire        afe_sdata,
    output wire        afe_latch,

    //=========================================================================
    // DIRECT DISCRETE INPUTS  (8)  - these never wait for a scan slot
    //=========================================================================
    input  wire        crash_trig_front,  // crash discrete, front
    input  wire        crash_trig_side,   // crash discrete, side (car)
    input  wire        hvil_raw,          // interlock loop, direct
    input  wire        ignition_on,       // key / start
    input  wire        permit_ack,        // rider button, hold 3 s
    input  wire        hazard_button,     // driver hazard switch
    input  wire [1:0]  mode_strap,        // 00 car, 01 bike, 1x auto-detect

    //=========================================================================
    // HIGH VOLTAGE SAFETY  (14)
    //=========================================================================
    output wire        hv_contactor_pos_en,
    output wire        hv_contactor_neg_en,
    output wire        hv_precharge_en,
    output wire        hv_discharge_en,
    output wire        pyro_fuse_arm,
    output wire        pyro_fuse_fire,
    output wire        hv_isolated,       // safe to approach
    output wire [2:0]  hv_state,
    output wire [3:0]  hv_fault_code,

    //=========================================================================
    // POWERTRAIN  (50)
    //=========================================================================
    output wire [11:0] torque_cmd,
    output wire [11:0] regen_cmd,
    output wire        motor_enable,
    output wire [7:0]  power_derate_pct,
    output wire [7:0]  cooling_pump_pwm,
    output wire [7:0]  cooling_fan_pwm,
    output wire        charge_enable,

    //=========================================================================
    // SAFETY ACTUATORS  (10)  - airbags and doors are forced 0 in bike mode
    //=========================================================================
    output wire [3:0]  airbag_trigger,
    output wire [1:0]  belt_pretension,
    output wire        door_unlock,
    output wire        horn_en,
    output wire        headlight_en,
    output wire        hazard_lights_en,

    //=========================================================================
    // TELEMATICS AND DISPLAY  (22)
    //=========================================================================
    input  wire [7:0]  gps_rx_data,
    input  wire        gps_rx_valid,
    output wire [7:0]  sos_tx_data,
    output wire        sos_tx_valid,
    input  wire        sos_tx_ready,
    output wire        disp_sclk,
    output wire        disp_sdata,
    output wire        disp_cs,

    //=========================================================================
    // MCU INTERFACE - APB3, 32-bit  (81)
    //=========================================================================
    input  wire [11:0] paddr,
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [31:0] pwdata,
    output wire [31:0] prdata,
    output wire        pready,
    output wire        pslverr,

    //=========================================================================
    // DIRECT STATUS  (22)  - for the cluster, without an APB read
    //=========================================================================
    output wire [7:0]  system_health_score,
    output wire        vehicle_enable,
    output wire        limp_home_active,
    output wire [7:0]  speed_limit_kph,
    output wire [2:0]  safety_state,
    output wire        warn_latched
);

    //=========================================================================
    // RESETS
    //=========================================================================
    wire rst_aon_n, rst_sensor_n, rst_ai_n, rst_mcu_n, rst_hvsafe_n;

    ivcu_reset_manager u_rst (
        .clk_aon     (clk_aon),
        .clk_sensor  (clk_sensor),
        .clk_ai      (clk_ai),
        .clk_mcu     (clk_mcu),
        .por_n       (por_n),
        .ext_rst_n   (ext_rst_n),
        .rst_aon_n   (rst_aon_n),
        .rst_sensor_n(rst_sensor_n),
        .rst_ai_n    (rst_ai_n),
        .rst_mcu_n   (rst_mcu_n),
        .rst_hvsafe_n(rst_hvsafe_n)
    );

    //=========================================================================
    // INTERNAL NETS
    //=========================================================================
    wire [1023:0] sensor_value_flat;
    wire [191:0]  sensor_status_flat;
    wire [255:0]  sensor_conf_flat;
    wire [63:0]   sensor_fault, sensor_dead, sensor_fresh, implausible_flags;
    wire          throttle_disagree, sweep_done, afe_busy;

    wire [63:0]   sensor_enable, afe_power, bypass_active;
    wire [1:0]    active_mode;
    wire          mode_resolved, mode_is_car, mode_is_bike, mode_is_safe;
    wire [7:0]    detect_sweeps;

    wire [63:0]   mask_critical, mask_degrade, mask_conditional, mask_comfort;
    wire          permit_inhibit, permit_offer;
    wire [23:0]   permit_starts_flat;
    wire [15:0]   permit_state_flat;

    wire          ignition_stable, ign_cycle;
    wire [31:0]   ign_lifetime;

    wire          hv_on, torque_inhibit, crash_latched, crash_pyro, pyro_fired;
    wire [15:0]   crash_peak_long, crash_peak_lat;
    wire [2:0]    crash_direction, pyro_state;
    wire          hvil_ok, hvil_degraded, iso_warn, iso_fault;
    wire [15:0]   hv_cell_tmax, hv_encl_press;
    wire [14:0]   hv_sense_dead;

    wire [7:0]    score_battery, score_motor, score_thermal, score_dynamics;
    wire [7:0]    score_driver, score_safety, score_perception;
    wire [3:0]    status_battery, status_motor, status_thermal, status_dynamics;
    wire [3:0]    status_driver, status_safety, status_perception;
    wire          score_valid;

    wire          charge_inhibit, discharge_inhibit, thermal_runaway_alarm;
    wire          batt_cold, batt_hot, batt_low_soc, batt_worn;
    wire [7:0]    batt_power_limit_pct;
    wire          motor_inhibit, regen_inhibit, motor_hot, inverter_hot;
    wire          overspeed, overcurrent;
    wire [7:0]    torque_limit_pct;
    wire [7:0]    pump_pwm_req, fan_pwm_req;
    wire          pump_running, cooling_fault, coolant_overtemp;
    wire [7:0]    vehicle_speed_kph, lean_estimate;
    wire          vehicle_moving, vehicle_stationary, braking;
    wire          yaw_excessive, wheel_locked;
    wire [11:0]   torque_request, regen_request;
    wire          throttle_fault, brake_override, rider_present;
    wire          stand_down, gear_engaged;
    wire          feat_lane_keep, feat_adaptive_cruise, feat_blind_spot;
    wire          feat_park_assist, feat_auto_headlight, feat_auto_wiper;
    wire          feat_collision_warn, gps_usable, perception_degraded;
    wire [15:0]   gps_stale_sweeps;

    wire          hv_request, airbag_deployed;
    wire [3:0]    inhibit_reason;

    wire [2:0]    crash_severity, sos_route, occupant_count;
    wire          battery_incident, sos_sending;
    wire [1:0]    sos_tx_count;
    wire          gps_fix_valid, gps_fix_current;
    wire [15:0]   fix_age_ms;
    wire [3:0]    gps_sats;
    wire [7:0]    gps_frame_err;

    wire          service_clear;
    wire [1:0]    mode_req;
    wire          mode_req_valid;
    wire          log_rd_en;
    wire [8:0]    log_rd_addr, log_wr_ptr;
    wire [31:0]   log_rd_data;
    wire [7:0]    log_wrap_count;
    wire [5:0]    guidance_sensor_id;
    wire [2:0]    guidance_status, guidance_starts_left;
    wire [3:0]    guidance_action;
    wire          guidance_valid;

    //=========================================================================
    // CDC - PATTERN 1: PULSES
    //=========================================================================
    wire sweep_done_ai, sweep_done_aon, sweep_done_mcu;

    ivcu_cdc_pulse_sync u_sw_ai (
        .clk_src (clk_sensor), .rst_src_n(rst_sensor_n), .pulse_src(sweep_done),
        .clk_dst (clk_ai),     .rst_dst_n(rst_ai_n),     .pulse_dst(sweep_done_ai));

    ivcu_cdc_pulse_sync u_sw_aon (
        .clk_src (clk_sensor), .rst_src_n(rst_sensor_n), .pulse_src(sweep_done),
        .clk_dst (clk_aon),    .rst_dst_n(rst_aon_n),    .pulse_dst(sweep_done_aon));

    ivcu_cdc_pulse_sync u_sw_mcu (
        .clk_src (clk_sensor), .rst_src_n(rst_sensor_n), .pulse_src(sweep_done),
        .clk_dst (clk_mcu),    .rst_dst_n(rst_mcu_n),    .pulse_dst(sweep_done_mcu));

    //=========================================================================
    // CDC - PATTERN 2: SINGLE BITS
    //
    // Asynchronous pins into clk_aon.  mode_strap is a bonded strap and cannot
    // glitch, but it costs two flops to be certain and this is the signal that
    // decides whether the vehicle has airbags.
    //=========================================================================
    wire ign_s, ack_s, hazard_s;
    wire [1:0] strap_s;

    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b0)) u_s_ign (
        .clk_dst(clk_aon), .rst_dst_n(rst_aon_n),
        .d_src(ignition_on), .q_dst(ign_s));

    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b0)) u_s_ack (
        .clk_dst(clk_aon), .rst_dst_n(rst_aon_n),
        .d_src(permit_ack), .q_dst(ack_s));

    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b0)) u_s_haz (
        .clk_dst(clk_aon), .rst_dst_n(rst_aon_n),
        .d_src(hazard_button), .q_dst(hazard_s));

    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b1)) u_s_strap0 (
        .clk_dst(clk_aon), .rst_dst_n(rst_aon_n),
        .d_src(mode_strap[0]), .q_dst(strap_s[0]));

    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b1)) u_s_strap1 (
        .clk_dst(clk_aon), .rst_dst_n(rst_aon_n),
        .d_src(mode_strap[1]), .q_dst(strap_s[1]));

    // clk_ai -> clk_aon.  Each of these gates a decision in the always-on
    // domain, so each gets real flops.
    wire moving_aon, stationary_aon, braking_aon, pumprun_aon;
    wire disch_inh_aon, motor_inh_aon, runaway_aon, thrfault_aon, stand_aon;
    wire rider_aon, charge_inh_aon, regen_inh_aon;

    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b0)) u_a_mov (
        .clk_dst(clk_aon), .rst_dst_n(rst_aon_n),
        .d_src(vehicle_moving), .q_dst(moving_aon));
    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b0)) u_a_sta (
        .clk_dst(clk_aon), .rst_dst_n(rst_aon_n),
        .d_src(vehicle_stationary), .q_dst(stationary_aon));
    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b0)) u_a_brk (
        .clk_dst(clk_aon), .rst_dst_n(rst_aon_n),
        .d_src(braking), .q_dst(braking_aon));
    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b0)) u_a_pmp (
        .clk_dst(clk_aon), .rst_dst_n(rst_aon_n),
        .d_src(pump_running), .q_dst(pumprun_aon));
    // safe defaults of 1 on the inhibits: until the AI cluster has spoken,
    // the always-on domain assumes the powertrain is not permitted
    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b1)) u_a_dis (
        .clk_dst(clk_aon), .rst_dst_n(rst_aon_n),
        .d_src(discharge_inhibit), .q_dst(disch_inh_aon));
    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b1)) u_a_mot (
        .clk_dst(clk_aon), .rst_dst_n(rst_aon_n),
        .d_src(motor_inhibit), .q_dst(motor_inh_aon));
    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b0)) u_a_run (
        .clk_dst(clk_aon), .rst_dst_n(rst_aon_n),
        .d_src(thermal_runaway_alarm), .q_dst(runaway_aon));
    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b1)) u_a_thr (
        .clk_dst(clk_aon), .rst_dst_n(rst_aon_n),
        .d_src(throttle_fault), .q_dst(thrfault_aon));
    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b0)) u_a_std (
        .clk_dst(clk_aon), .rst_dst_n(rst_aon_n),
        .d_src(stand_down), .q_dst(stand_aon));
    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b0)) u_a_rid (
        .clk_dst(clk_aon), .rst_dst_n(rst_aon_n),
        .d_src(rider_present), .q_dst(rider_aon));
    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b1)) u_a_chg (
        .clk_dst(clk_aon), .rst_dst_n(rst_aon_n),
        .d_src(charge_inhibit), .q_dst(charge_inh_aon));
    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b1)) u_a_rgn (
        .clk_dst(clk_aon), .rst_dst_n(rst_aon_n),
        .d_src(regen_inhibit), .q_dst(regen_inh_aon));

    // clk_sensor -> clk_aon
    wire thrdis_aon;
    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b0)) u_s_tdis (
        .clk_dst(clk_aon), .rst_dst_n(rst_aon_n),
        .d_src(throttle_disagree), .q_dst(thrdis_aon));

    // clk_aon -> clk_ai
    wire car_ai, bike_ai;
    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b0)) u_m_car (
        .clk_dst(clk_ai), .rst_dst_n(rst_ai_n),
        .d_src(mode_is_car), .q_dst(car_ai));
    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b0)) u_m_bike (
        .clk_dst(clk_ai), .rst_dst_n(rst_ai_n),
        .d_src(mode_is_bike), .q_dst(bike_ai));

    // clk_aon -> clk_sensor
    wire hv_on_sens, moving_sens, pumprun_sens, braking_sens;
    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b0)) u_x_hv (
        .clk_dst(clk_sensor), .rst_dst_n(rst_sensor_n),
        .d_src(hv_on), .q_dst(hv_on_sens));
    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b0)) u_x_mov (
        .clk_dst(clk_sensor), .rst_dst_n(rst_sensor_n),
        .d_src(moving_aon), .q_dst(moving_sens));
    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b0)) u_x_pmp (
        .clk_dst(clk_sensor), .rst_dst_n(rst_sensor_n),
        .d_src(pumprun_aon), .q_dst(pumprun_sens));
    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b0)) u_x_brk (
        .clk_dst(clk_sensor), .rst_dst_n(rst_sensor_n),
        .d_src(braking_aon), .q_dst(braking_sens));

    //=========================================================================
    // CDC - PATTERN 3: QUASI-STATIC
    //
    // The wide buses below cross unregistered.  Each is consumed only on a
    // synchronised pulse and each changes far more slowly than the sampling
    // domain.  set_clock_groups -asynchronous in the SDC is what makes this
    // legal; see the header.
    //
    //   sensor_value_flat  1024b  clk_sensor -> clk_ai, clk_mcu
    //   sensor_status_flat  192b  clk_sensor -> clk_ai, clk_mcu, clk_aon
    //   sensor_fault         64b  clk_sensor -> clk_aon, clk_mcu
    //   sensor_enable        64b  clk_aon    -> clk_sensor, clk_ai, clk_mcu
    //   bypass_active        64b  clk_aon    -> clk_sensor, clk_ai
    //   torque/regen request 12b  clk_ai     -> clk_aon
    //   limits and scores     8b  clk_ai     -> clk_aon, clk_mcu
    //=========================================================================

    //=========================================================================
    // ACQUISITION  [clk_sensor]
    //=========================================================================
    ivcu_sensor_acquisition u_acq (
        .clk_sensor       (clk_sensor),
        .rst_sensor_n     (rst_sensor_n),
        .adc_chan         (adc_chan),
        .adc_req          (adc_req),
        .adc_data         (adc_data),
        .adc_valid        (adc_valid),
        .adc_busy         (adc_busy),
        .afe_sclk         (afe_sclk),
        .afe_sdata        (afe_sdata),
        .afe_latch        (afe_latch),
        .afe_busy         (afe_busy),
        .sensor_enable    (sensor_enable),
        .afe_power        (afe_power),
        .bypass_active    (bypass_active),
        .vehicle_moving   (moving_sens),
        .hv_on            (hv_on_sens),
        .pump_running     (pumprun_sens),
        .braking          (braking_sens),
        .sensor_value_flat(sensor_value_flat),
        .sensor_status_flat(sensor_status_flat),
        .sensor_conf_flat (sensor_conf_flat),
        .sensor_fault     (sensor_fault),
        .sensor_dead      (sensor_dead),
        .sensor_fresh     (sensor_fresh),
        .implausible_flags(implausible_flags),
        .throttle_disagree(throttle_disagree),
        .sweep_done       (sweep_done)
    );

    //=========================================================================
    // MODE  [clk_aon]
    //=========================================================================
    ivcu_mode_manager u_mode (
        .clk_aon           (clk_aon),
        .rst_aon_n         (rst_aon_n),
        .mode_strap_s      (strap_s),
        .sensor_fresh_s    (sensor_fresh),
        .sensor_dead_s     (sensor_dead),
        .sweep_done_s      (sweep_done_aon),
        .mode_req          (mode_req),
        .mode_req_valid    (mode_req_valid),
        .vehicle_stationary(stationary_aon),
        .hv_on             (hv_on),
        .active_mode       (active_mode),
        .mode_resolved     (mode_resolved),
        .detect_sweeps     (detect_sweeps),
        .sensor_enable     (sensor_enable),
        .afe_power         (afe_power),
        .mode_is_car       (mode_is_car),
        .mode_is_bike      (mode_is_bike),
        .mode_is_safe      (mode_is_safe)
    );

    //=========================================================================
    // SERVICEABILITY  [clk_aon]
    //=========================================================================
    ivcu_ignition_counter u_ign (
        .clk_aon           (clk_aon),
        .rst_aon_n         (rst_aon_n),
        .ignition_on_s     (ign_s),
        .vehicle_stationary(stationary_aon),
        .ignition_stable   (ignition_stable),
        .ign_cycle         (ign_cycle),
        .ign_lifetime      (ign_lifetime)
    );

    ivcu_serviceability_mgr u_svc (
        .clk_aon           (clk_aon),
        .rst_aon_n         (rst_aon_n),
        .sensor_fault_s    (sensor_fault),
        .permit_ack_s      (ack_s),
        .vehicle_stationary(stationary_aon),
        .ign_cycle         (ign_cycle),
        .service_clear     (service_clear),
        .mode_is_bike      (mode_is_bike),
        .bypass_active     (bypass_active),
        .limp_home_active  (limp_home_active),
        .permit_inhibit    (permit_inhibit),
        .permit_offer      (permit_offer),
        .speed_limit_kph   (speed_limit_kph),
        .permit_starts_flat(permit_starts_flat),
        .permit_state_flat (permit_state_flat),
        .mask_critical     (mask_critical),
        .mask_degrade      (mask_degrade),
        .mask_conditional  (mask_conditional),
        .mask_comfort      (mask_comfort)
    );

    //=========================================================================
    // HV SAFETY ISLAND  [clk_aon, PD_HVSAFE]
    //=========================================================================
    ivcu_hv_safety_island u_hv (
        .clk_aon            (clk_aon),
        .rst_hvsafe_n       (rst_hvsafe_n),
        .clk_sensor         (clk_sensor),
        .rst_sensor_n       (rst_sensor_n),
        .sensor_value_flat  (sensor_value_flat),
        .sensor_dead        (sensor_dead),
        .sweep_done         (sweep_done),
        .crash_trig_front   (crash_trig_front),
        .crash_trig_side    (crash_trig_side),
        .hvil_raw           (hvil_raw),
        .hv_request         (hv_request),
        .active_mode        (active_mode),
        .vehicle_speed_kph  (vehicle_speed_kph),
        .service_clear      (service_clear),
        .hv_contactor_pos_en(hv_contactor_pos_en),
        .hv_contactor_neg_en(hv_contactor_neg_en),
        .hv_precharge_en    (hv_precharge_en),
        .hv_discharge_en    (hv_discharge_en),
        .pyro_fuse_arm      (pyro_fuse_arm),
        .pyro_fuse_fire     (pyro_fuse_fire),
        .hv_on              (hv_on),
        .hv_isolated        (hv_isolated),
        .torque_inhibit     (torque_inhibit),
        .hv_state           (hv_state),
        .hv_fault_code      (hv_fault_code),
        .pyro_fired         (pyro_fired),
        .pyro_state         (pyro_state),
        .hvil_ok            (hvil_ok),
        .hvil_degraded      (hvil_degraded),
        .iso_warn           (iso_warn),
        .iso_fault          (iso_fault),
        .crash_latched      (crash_latched),
        .crash_pyro         (crash_pyro),
        .crash_peak_long    (crash_peak_long),
        .crash_peak_lat     (crash_peak_lat),
        .crash_direction    (crash_direction),
        .hv_cell_tmax       (hv_cell_tmax),
        .hv_encl_press      (hv_encl_press),
        .hv_sense_dead      (hv_sense_dead)
    );

    //=========================================================================
    // AI CLUSTER  [clk_ai]
    //=========================================================================
    ivcu_ai_cluster u_ai (
        .clk_ai              (clk_ai),
        .rst_ai_n            (rst_ai_n),
        .sensor_value_flat   (sensor_value_flat),
        .sensor_status_flat  (sensor_status_flat),
        .sensor_dead         (sensor_dead),
        .sensor_enable       (sensor_enable),
        .bypass_active       (bypass_active),
        .throttle_disagree   (throttle_disagree),
        .implausible_flags   (implausible_flags),
        .update_req          (sweep_done_ai),
        .mode_is_car         (car_ai),
        .mode_is_bike        (bike_ai),
        .score_battery       (score_battery),
        .score_motor         (score_motor),
        .score_thermal       (score_thermal),
        .score_dynamics      (score_dynamics),
        .score_driver        (score_driver),
        .score_safety        (score_safety),
        .score_perception    (score_perception),
        .system_health_score (system_health_score),
        .status_battery      (status_battery),
        .status_motor        (status_motor),
        .status_thermal      (status_thermal),
        .status_dynamics     (status_dynamics),
        .status_driver       (status_driver),
        .status_safety       (status_safety),
        .status_perception   (status_perception),
        .score_valid         (score_valid),
        .charge_inhibit      (charge_inhibit),
        .discharge_inhibit   (discharge_inhibit),
        .batt_power_limit_pct(batt_power_limit_pct),
        .thermal_runaway_alarm(thermal_runaway_alarm),
        .batt_cold           (batt_cold),
        .batt_hot            (batt_hot),
        .batt_low_soc        (batt_low_soc),
        .batt_worn           (batt_worn),
        .motor_inhibit       (motor_inhibit),
        .torque_limit_pct    (torque_limit_pct),
        .regen_inhibit       (regen_inhibit),
        .motor_hot           (motor_hot),
        .inverter_hot        (inverter_hot),
        .overspeed           (overspeed),
        .overcurrent         (overcurrent),
        .pump_pwm            (pump_pwm_req),
        .fan_pwm             (fan_pwm_req),
        .pump_running        (pump_running),
        .cooling_fault       (cooling_fault),
        .coolant_overtemp    (coolant_overtemp),
        .vehicle_speed_kph   (vehicle_speed_kph),
        .vehicle_moving      (vehicle_moving),
        .vehicle_stationary  (vehicle_stationary),
        .braking             (braking),
        .lean_estimate       (lean_estimate),
        .yaw_excessive       (yaw_excessive),
        .wheel_locked        (wheel_locked),
        .torque_request      (torque_request),
        .regen_request       (regen_request),
        .throttle_fault      (throttle_fault),
        .brake_override      (brake_override),
        .rider_present       (rider_present),
        .stand_down          (stand_down),
        .gear_engaged        (gear_engaged),
        .feat_lane_keep      (feat_lane_keep),
        .feat_adaptive_cruise(feat_adaptive_cruise),
        .feat_blind_spot     (feat_blind_spot),
        .feat_park_assist    (feat_park_assist),
        .feat_auto_headlight (feat_auto_headlight),
        .feat_auto_wiper     (feat_auto_wiper),
        .feat_collision_warn (feat_collision_warn),
        .gps_usable          (gps_usable),
        .gps_stale_sweeps    (gps_stale_sweeps),
        .perception_degraded (perception_degraded)
    );

    //=========================================================================
    // CENTRAL SAFETY FSM  [clk_aon]
    //=========================================================================
    ivcu_central_safety_fsm u_fsm (
        .clk_aon              (clk_aon),
        .rst_aon_n            (rst_aon_n),
        .active_mode          (active_mode),
        .mode_resolved        (mode_resolved),
        .sensor_fault_s       (sensor_fault),
        .mask_critical        (mask_critical),
        .permit_inhibit       (permit_inhibit),
        .limp_home_active     (limp_home_active),
        .hv_on                (hv_on),
        .hv_fault_code        (hv_fault_code),
        .crash_latched        (crash_latched),
        .torque_inhibit       (torque_inhibit),
        .discharge_inhibit    (disch_inh_aon),
        .motor_inhibit        (motor_inh_aon),
        .thermal_runaway_alarm(runaway_aon),
        .throttle_fault       (thrfault_aon | thrdis_aon),
        .stand_down           (stand_aon),
        .ignition_stable      (ignition_stable),
        .service_clear        (service_clear),
        .hv_request           (hv_request),
        .vehicle_enable       (vehicle_enable),
        .safety_state         (safety_state),
        .inhibit_reason       (inhibit_reason),
        .warn_latched         (warn_latched)
    );

    //=========================================================================
    // EMERGENCY RESPONSE  [clk_aon]
    //=========================================================================
    ivcu_emergency_response u_emg (
        .clk_aon              (clk_aon),
        .rst_aon_n            (rst_aon_n),
        .crash_latched        (crash_latched),
        .crash_pyro           (crash_pyro),
        .crash_peak_long      (crash_peak_long),
        .crash_peak_lat       (crash_peak_lat),
        .crash_direction      (crash_direction),
        .pyro_fired           (pyro_fired),
        .hv_isolated          (hv_isolated),
        .hv_state             (hv_state),
        .hv_fault_code        (hv_fault_code),
        .hv_cell_tmax         (hv_cell_tmax),
        .hv_encl_press        (hv_encl_press),
        .active_mode          (active_mode),
        .vehicle_speed_kph    (vehicle_speed_kph),
        .airbag_deployed      (airbag_deployed),
        .rider_present        (rider_aon),
        .thermal_runaway_alarm(runaway_aon),
        .hazard_button        (hazard_s),
        .service_clear        (service_clear),
        .gps_rx_data          (gps_rx_data),
        .gps_rx_valid         (gps_rx_valid),
        .sos_tx_data          (sos_tx_data),
        .sos_tx_valid         (sos_tx_valid),
        .sos_tx_ready         (sos_tx_ready),
        .hazard_lights_en     (hazard_lights_en),
        .crash_severity       (crash_severity),
        .sos_route            (sos_route),
        .occupant_count       (occupant_count),
        .battery_incident     (battery_incident),
        .sos_sending          (sos_sending),
        .sos_tx_count         (sos_tx_count),
        .gps_fix_valid        (gps_fix_valid),
        .gps_fix_current      (gps_fix_current),
        .fix_age_ms           (fix_age_ms),
        .gps_sats             (gps_sats),
        .gps_frame_err        (gps_frame_err)
    );

    //=========================================================================
    // ACTUATORS  [clk_aon]  - the one place every limit is applied
    //=========================================================================
    ivcu_actuator_output_mgr u_act (
        .clk_aon             (clk_aon),
        .rst_aon_n           (rst_aon_n),
        .torque_request      (torque_request),
        .regen_request       (regen_request),
        .batt_power_limit_pct(batt_power_limit_pct),
        .torque_limit_pct    (torque_limit_pct),
        .vehicle_enable      (vehicle_enable),
        .torque_inhibit      (torque_inhibit),
        .discharge_inhibit   (disch_inh_aon),
        .motor_inhibit       (motor_inh_aon),
        .charge_inhibit      (charge_inh_aon),
        .regen_inhibit       (regen_inh_aon),
        .crash_latched       (crash_latched),
        .speed_limit_kph     (speed_limit_kph),
        .vehicle_speed_kph   (vehicle_speed_kph),
        .pump_pwm_req        (pump_pwm_req),
        .fan_pwm_req         (fan_pwm_req),
        .crash_severity      (crash_severity),
        .rider_present       (rider_aon),
        .mode_is_car         (mode_is_car),
        .torque_cmd          (torque_cmd),
        .regen_cmd           (regen_cmd),
        .motor_enable        (motor_enable),
        .power_derate_pct    (power_derate_pct),
        .cooling_pump_pwm    (cooling_pump_pwm),
        .cooling_fan_pwm     (cooling_fan_pwm),
        .charge_enable       (charge_enable),
        .airbag_trigger      (airbag_trigger),
        .belt_pretension     (belt_pretension),
        .door_unlock         (door_unlock),
        .horn_en             (horn_en),
        .headlight_en        (headlight_en),
        .airbag_deployed     (airbag_deployed)
    );

    //=========================================================================
    // GUIDANCE  [clk_mcu]
    //=========================================================================
    ivcu_service_guidance u_guide (
        .clk_mcu             (clk_mcu),
        .rst_mcu_n           (rst_mcu_n),
        .sensor_status_flat  (sensor_status_flat),
        .sensor_enable_s     (sensor_enable),
        .permit_state_flat   (permit_state_flat),
        .permit_starts_flat  (permit_starts_flat),
        .hv_fault_code       (hv_fault_code),
        .update_req          (sweep_done_mcu),
        .disp_sclk           (disp_sclk),
        .disp_sdata          (disp_sdata),
        .disp_cs             (disp_cs),
        .guidance_sensor_id  (guidance_sensor_id),
        .guidance_status     (guidance_status),
        .guidance_action     (guidance_action),
        .guidance_starts_left(guidance_starts_left),
        .guidance_valid      (guidance_valid)
    );

    //=========================================================================
    // FAULT LOG  [clk_mcu]  + the OpenRAM macro
    //=========================================================================
    wire        sram_clk0, sram_csb0, sram_web0;
    wire [8:0]  sram_addr0;
    wire [31:0] sram_din0, sram_dout0;

    // any permit machine entering ACTIVE or EXPIRED is a loggable event
    wire permit_granted = (permit_state_flat[1:0]   == `PS_ACTIVE) |
                          (permit_state_flat[3:2]   == `PS_ACTIVE) |
                          (permit_state_flat[5:4]   == `PS_ACTIVE) |
                          (permit_state_flat[7:6]   == `PS_ACTIVE) |
                          (permit_state_flat[9:8]   == `PS_ACTIVE) |
                          (permit_state_flat[11:10] == `PS_ACTIVE) |
                          (permit_state_flat[13:12] == `PS_ACTIVE) |
                          (permit_state_flat[15:14] == `PS_ACTIVE);

    wire permit_expired = (permit_state_flat[1:0]   == `PS_EXPIRED) |
                          (permit_state_flat[3:2]   == `PS_EXPIRED) |
                          (permit_state_flat[5:4]   == `PS_EXPIRED) |
                          (permit_state_flat[7:6]   == `PS_EXPIRED) |
                          (permit_state_flat[9:8]   == `PS_EXPIRED) |
                          (permit_state_flat[11:10] == `PS_EXPIRED) |
                          (permit_state_flat[13:12] == `PS_EXPIRED) |
                          (permit_state_flat[15:14] == `PS_EXPIRED);

    ivcu_fault_logger u_log (
        .clk_mcu              (clk_mcu),
        .rst_mcu_n            (rst_mcu_n),
        .sensor_fault_s       (sensor_fault),
        .sensor_status_flat   (sensor_status_flat),
        .hv_fault_code        (hv_fault_code),
        .crash_latched        (crash_latched),
        .pyro_fired           (pyro_fired),
        .permit_granted       (permit_granted),
        .permit_expired       (permit_expired),
        .active_mode          (active_mode),
        .thermal_runaway_alarm(thermal_runaway_alarm),
        .iso_warn             (iso_warn),
        .log_rd_en            (log_rd_en),
        .log_rd_addr          (log_rd_addr),
        .log_rd_data          (log_rd_data),
        .log_wr_ptr           (log_wr_ptr),
        .log_wrap_count       (log_wrap_count),
        .sram_clk0            (sram_clk0),
        .sram_csb0            (sram_csb0),
        .sram_web0            (sram_web0),
        .sram_addr0           (sram_addr0),
        .sram_din0            (sram_din0),
        .sram_dout0           (sram_dout0)
    );

    // Port 1 is unused: one read/write port serves both the logger and the
    // MCU, which makes a simultaneous same-address access structurally
    // impossible rather than merely unlikely.
    sram_512x32_2port u_sram (
        .clk0 (sram_clk0),
        .csb0 (sram_csb0),
        .web0 (sram_web0),
        .addr0(sram_addr0),
        .din0 (sram_din0),
        .dout0(sram_dout0),
        .clk1 (clk_mcu),
        .csb1 (1'b1),        // deselected
        .addr1(9'd0),
        .dout1()             // intentionally open
    );

    //=========================================================================
    // APB REGISTERS  [clk_mcu]
    //=========================================================================
    ivcu_apb_regs u_apb (
        .clk_mcu             (clk_mcu),
        .rst_mcu_n           (rst_mcu_n),
        .paddr               (paddr),
        .psel                (psel),
        .penable             (penable),
        .pwrite              (pwrite),
        .pwdata              (pwdata),
        .prdata              (prdata),
        .pready              (pready),
        .pslverr             (pslverr),
        .sensor_value_flat   (sensor_value_flat),
        .sensor_status_flat  (sensor_status_flat),
        .sensor_conf_flat    (sensor_conf_flat),
        .sensor_enable_s     (sensor_enable),
        .sensor_fault_s      (sensor_fault),
        .bypass_active_s     (bypass_active),
        .system_health_score (system_health_score),
        .score_battery       (score_battery),
        .score_motor         (score_motor),
        .score_thermal       (score_thermal),
        .score_dynamics      (score_dynamics),
        .score_driver        (score_driver),
        .score_safety        (score_safety),
        .score_perception    (score_perception),
        .active_mode         (active_mode),
        .mode_resolved       (mode_resolved),
        .detect_sweeps       (detect_sweeps),
        .safety_state        (safety_state),
        .inhibit_reason      (inhibit_reason),
        .limp_home_active    (limp_home_active),
        .hv_state            (hv_state),
        .hv_fault_code       (hv_fault_code),
        .hv_isolated         (hv_isolated),
        .pyro_fired          (pyro_fired),
        .hvil_ok             (hvil_ok),
        .iso_warn            (iso_warn),
        .iso_fault           (iso_fault),
        .permit_starts_flat  (permit_starts_flat),
        .permit_state_flat   (permit_state_flat),
        .ign_lifetime        (ign_lifetime),
        .crash_severity      (crash_severity),
        .sos_route           (sos_route),
        .battery_incident    (battery_incident),
        .gps_fix_valid       (gps_fix_valid),
        .fix_age_ms          (fix_age_ms),
        .guidance_sensor_id  (guidance_sensor_id),
        .guidance_status     (guidance_status),
        .guidance_action     (guidance_action),
        .guidance_starts_left(guidance_starts_left),
        .log_rd_en           (log_rd_en),
        .log_rd_addr         (log_rd_addr),
        .log_rd_data         (log_rd_data),
        .log_wr_ptr          (log_wr_ptr),
        .log_wrap_count      (log_wrap_count),
        .service_clear       (service_clear),
        .mode_req            (mode_req),
        .mode_req_valid      (mode_req_valid)
    );

endmodule

`default_nettype wire
