//=============================================================================
// ivcu_ai_cluster.v  -  the seven domain blocks and the scoring engine
//
// Instantiates, in clk_ai:
//     ivcu_health_scorer      one engine, seven domain scores, no divider
//     ivcu_battery_ai         channels  6-17
//     ivcu_motor_ai           channels 18-25
//     ivcu_thermal_ai         channels 26-31
//     ivcu_dynamics_ai        channels 32-43
//     ivcu_driver_input_ai    channels 44-49
//     ivcu_perception_ai      channels 53-57
//
// The HV and crash channels (0-5, 50-52) are NOT processed here.  They are
// handled in ivcu_hv_safety_island, in the always-on domain, deliberately out
// of reach of anything in this file.  Their health is still scored - domain
// D_SAFETY - because the rider should see it, but no decision about them is
// made in clk_ai.
//
//-----------------------------------------------------------------------------
// THE CDC DECISION IN THIS FILE, AND WHY IT IS NOT THE SAME AS THE HV ISLAND
//
// sensor_value_flat is 1024 bits and sensor_status_flat is 192.  Running all
// of that through holding registers, as ivcu_hv_sense_sync does for its
// fifteen values, would cost about 1,200 extra flip-flops - a real fraction of
// the die - to solve a problem this domain does not have.
//
// So this cluster uses the quasi-static pattern instead:
//
//     the DATA crosses unregistered
//     the CONTROL - update_req - crosses through a synchroniser
//     nothing acts on the data except on an update_req pulse
//
// That is safe here and it is worth being precise about why.  The values only
// change while a sweep is in progress; update_req fires at the end of one.  A
// block sampling then may catch channel 0 from the sweep that just ended and
// channel 63 written microseconds earlier, but every value is from inside the
// same 10 us window, and no decision in this file depends on two channels
// being from the same instant.
//
// The HV island DOES have that dependency - "is the DC link within 95 % of
// pack voltage" compares two channels and a mixed snapshot welds a contactor -
// which is exactly why it pays for a coherent one and this does not.
//
// SDC REQUIREMENT.  This pattern is only correct if static timing does not try
// to time the data path.  ivcu_ev_v4.sdc must contain:
//
//     set_clock_groups -asynchronous  (covers it, and is already there)
//     set_false_path -from [get_clocks clk_sensor] -to [get_clocks clk_ai]
//
// If someone later removes the asynchronous grouping, this becomes silently
// wrong.  That is the risk of the pattern and it is the reason it is written
// down here rather than left to be inferred.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_ai_cluster (
    input  wire          clk_ai,
    input  wire          rst_ai_n,

    //--- quasi-static data from clk_sensor - see the header ------------------
    input  wire [1023:0] sensor_value_flat,
    input  wire [191:0]  sensor_status_flat,
    input  wire [63:0]   sensor_dead,
    input  wire [63:0]   sensor_enable,
    input  wire [63:0]   bypass_active,
    input  wire          throttle_disagree,
    input  wire [63:0]   implausible_flags,

    //--- the one signal that IS synchronised ---------------------------------
    input  wire          update_req,       // sweep_done, synced into clk_ai

    //--- mode ------------------------------------------------------------------
    input  wire          mode_is_car,
    input  wire          mode_is_bike,

    //--- health scores ---------------------------------------------------------
    output wire [7:0]    score_battery,
    output wire [7:0]    score_motor,
    output wire [7:0]    score_thermal,
    output wire [7:0]    score_dynamics,
    output wire [7:0]    score_driver,
    output wire [7:0]    score_safety,
    output wire [7:0]    score_perception,
    output wire [7:0]    system_health_score,
    output wire [3:0]    status_battery,
    output wire [3:0]    status_motor,
    output wire [3:0]    status_thermal,
    output wire [3:0]    status_dynamics,
    output wire [3:0]    status_driver,
    output wire [3:0]    status_safety,
    output wire [3:0]    status_perception,
    output wire          score_valid,

    //--- battery decisions ------------------------------------------------------
    output wire          charge_inhibit,
    output wire          discharge_inhibit,
    output wire [7:0]    batt_power_limit_pct,
    output wire          thermal_runaway_alarm,
    output wire          batt_cold,
    output wire          batt_hot,
    output wire          batt_low_soc,
    output wire          batt_worn,

    //--- motor decisions --------------------------------------------------------
    output wire          motor_inhibit,
    output wire [7:0]    torque_limit_pct,
    output wire          regen_inhibit,
    output wire          motor_hot,
    output wire          inverter_hot,
    output wire          overspeed,
    output wire          overcurrent,

    //--- thermal decisions ------------------------------------------------------
    output wire [7:0]    pump_pwm,
    output wire [7:0]    fan_pwm,
    output wire          pump_running,
    output wire          cooling_fault,
    output wire          coolant_overtemp,

    //--- dynamics ----------------------------------------------------------------
    output wire [7:0]    vehicle_speed_kph,
    output wire          vehicle_moving,
    output wire          vehicle_stationary,
    output wire          braking,
    output wire [7:0]    lean_estimate,
    output wire          yaw_excessive,
    output wire          wheel_locked,

    //--- driver input -------------------------------------------------------------
    output wire [11:0]   torque_request,
    output wire [11:0]   regen_request,
    output wire          throttle_fault,
    output wire          brake_override,
    output wire          rider_present,
    output wire          stand_down,
    output wire          gear_engaged,

    //--- perception ----------------------------------------------------------------
    output wire          feat_lane_keep,
    output wire          feat_adaptive_cruise,
    output wire          feat_blind_spot,
    output wire          feat_park_assist,
    output wire          feat_auto_headlight,
    output wire          feat_auto_wiper,
    output wire          feat_collision_warn,
    output wire          gps_usable,
    output wire [15:0]   gps_stale_sweeps,
    output wire          perception_degraded
);

    //=========================================================================
    // Channel indices as integers.  Sized macros truncate in part-select
    // arithmetic - the same trap documented in ivcu_sensor_plausibility.
    //=========================================================================
    localparam integer C_CVMIN = `S_CELL_VOLT_MIN;
    localparam integer C_CVMAX = `S_CELL_VOLT_MAX;
    localparam integer C_PACKV = `S_PACK_VOLTAGE;
    localparam integer C_PACKI = `S_PACK_CURRENT;
    localparam integer C_CTMIN = `S_CELL_TEMP_MIN;
    localparam integer C_CTMAX = `S_CELL_TEMP_MAX;
    localparam integer C_PACKT = `S_PACK_TEMP;
    localparam integer C_SOC   = `S_SOC;
    localparam integer C_SOH   = `S_SOH;
    localparam integer C_ENCP  = `S_PACK_ENCL_PRESS;

    localparam integer C_RPM   = `S_MOTOR_RPM;
    localparam integer C_ROTOR = `S_ROTOR_POSITION;
    localparam integer C_PHA   = `S_PHASE_CURRENT_A;
    localparam integer C_PHB   = `S_PHASE_CURRENT_B;
    localparam integer C_MTEMP = `S_MOTOR_TEMP;
    localparam integer C_ITEMP = `S_INVERTER_TEMP;
    localparam integer C_DCI   = `S_DC_LINK_CURRENT;

    localparam integer C_CLIN  = `S_COOLANT_TEMP_IN;
    localparam integer C_CLOUT = `S_COOLANT_TEMP_OUT;
    localparam integer C_CLFLW = `S_COOLANT_FLOW;
    localparam integer C_CLPRS = `S_COOLANT_PRESSURE;
    localparam integer C_AMBT  = `S_AMBIENT_TEMP;

    localparam integer C_WFA   = `S_WSPD_FRONT_A;
    localparam integer C_WFB   = `S_WSPD_FRONT_B;
    localparam integer C_WRA   = `S_WSPD_REAR_A;
    localparam integer C_WRB   = `S_WSPD_REAR_B;
    localparam integer C_YAW   = `S_YAW_RATE;
    localparam integer C_ROLL  = `S_ROLL_RATE;

    localparam integer C_THR1  = `S_THROTTLE_POS_1;
    localparam integer C_THR2  = `S_THROTTLE_POS_2;
    localparam integer C_BPRS  = `S_BRAKE_PRESSURE;
    localparam integer C_BSW   = `S_BRAKE_SWITCH;
    localparam integer C_STAND = `S_SIDE_STAND;
    localparam integer C_SEAT  = `S_SEAT_OCCUPANCY;
    localparam integer C_GEAR  = `S_GEAR_POSITION;

    localparam integer C_CAM   = `S_CAMERA_STATUS;
    localparam integer C_RAD   = `S_RADAR_STATUS;
    localparam integer C_LID   = `S_LIDAR_STATUS;
    localparam integer C_ULT   = `S_ULTRASONIC_STATUS;
    localparam integer C_GPS   = `S_GPS_STATUS;
    localparam integer C_ALGT  = `S_AMBIENT_LIGHT;
    localparam integer C_RAIN  = `S_RAIN_SENSOR;

    //=========================================================================
    // Value taps
    //=========================================================================
    wire [15:0] v_cvmin = sensor_value_flat[C_CVMIN*16 +: 16];
    wire [15:0] v_cvmax = sensor_value_flat[C_CVMAX*16 +: 16];
    wire [15:0] v_packv = sensor_value_flat[C_PACKV*16 +: 16];
    wire [15:0] v_packi = sensor_value_flat[C_PACKI*16 +: 16];
    wire [15:0] v_ctmin = sensor_value_flat[C_CTMIN*16 +: 16];
    wire [15:0] v_ctmax = sensor_value_flat[C_CTMAX*16 +: 16];
    wire [15:0] v_packt = sensor_value_flat[C_PACKT*16 +: 16];
    wire [15:0] v_soc   = sensor_value_flat[C_SOC  *16 +: 16];
    wire [15:0] v_soh   = sensor_value_flat[C_SOH  *16 +: 16];
    wire [15:0] v_encp  = sensor_value_flat[C_ENCP *16 +: 16];

    wire [15:0] v_rpm   = sensor_value_flat[C_RPM  *16 +: 16];
    wire [15:0] v_rotor = sensor_value_flat[C_ROTOR*16 +: 16];
    wire [15:0] v_pha   = sensor_value_flat[C_PHA  *16 +: 16];
    wire [15:0] v_phb   = sensor_value_flat[C_PHB  *16 +: 16];
    wire [15:0] v_mtemp = sensor_value_flat[C_MTEMP*16 +: 16];
    wire [15:0] v_itemp = sensor_value_flat[C_ITEMP*16 +: 16];
    wire [15:0] v_dci   = sensor_value_flat[C_DCI  *16 +: 16];

    wire [15:0] v_clin  = sensor_value_flat[C_CLIN *16 +: 16];
    wire [15:0] v_clout = sensor_value_flat[C_CLOUT*16 +: 16];
    wire [15:0] v_clflw = sensor_value_flat[C_CLFLW*16 +: 16];
    wire [15:0] v_clprs = sensor_value_flat[C_CLPRS*16 +: 16];
    wire [15:0] v_ambt  = sensor_value_flat[C_AMBT *16 +: 16];

    wire [15:0] v_wfa   = sensor_value_flat[C_WFA  *16 +: 16];
    wire [15:0] v_wfb   = sensor_value_flat[C_WFB  *16 +: 16];
    wire [15:0] v_wra   = sensor_value_flat[C_WRA  *16 +: 16];
    wire [15:0] v_wrb   = sensor_value_flat[C_WRB  *16 +: 16];
    wire [15:0] v_yaw   = sensor_value_flat[C_YAW  *16 +: 16];
    wire [15:0] v_roll  = sensor_value_flat[C_ROLL *16 +: 16];

    wire [15:0] v_thr1  = sensor_value_flat[C_THR1 *16 +: 16];
    wire [15:0] v_bprs  = sensor_value_flat[C_BPRS *16 +: 16];
    wire [15:0] v_bsw   = sensor_value_flat[C_BSW  *16 +: 16];
    wire [15:0] v_stand = sensor_value_flat[C_STAND*16 +: 16];
    wire [15:0] v_seat  = sensor_value_flat[C_SEAT *16 +: 16];
    wire [15:0] v_gear  = sensor_value_flat[C_GEAR *16 +: 16];

    //=========================================================================
    // Status taps
    //=========================================================================
    wire [2:0] st_cam  = sensor_status_flat[C_CAM *3 +: 3];
    wire [2:0] st_rad  = sensor_status_flat[C_RAD *3 +: 3];
    wire [2:0] st_lid  = sensor_status_flat[C_LID *3 +: 3];
    wire [2:0] st_ult  = sensor_status_flat[C_ULT *3 +: 3];
    wire [2:0] st_gps  = sensor_status_flat[C_GPS *3 +: 3];
    wire [2:0] st_algt = sensor_status_flat[C_ALGT*3 +: 3];
    wire [2:0] st_rain = sensor_status_flat[C_RAIN*3 +: 3];

    //=========================================================================
    // Trust flags.  "Dead" here means either channel of a redundant pair is
    // not answering - a battery whose minimum cell temperature is unknown is
    // not half-known, it is unknown.
    //=========================================================================
    wire dead_cell_t   = sensor_dead[C_CTMIN] | sensor_dead[C_CTMAX];
    wire dead_cell_v   = sensor_dead[C_CVMIN] | sensor_dead[C_CVMAX];
    wire dead_soc      = sensor_dead[C_SOC];

    wire dead_rotor    = sensor_dead[C_ROTOR];
    wire dead_rpm      = sensor_dead[C_RPM];
    wire dead_phase_i  = sensor_dead[C_PHA] | sensor_dead[C_PHB];
    wire dead_mtemp    = sensor_dead[C_MTEMP];
    wire dead_itemp    = sensor_dead[C_ITEMP];

    wire dead_coolant  = sensor_dead[C_CLIN]  | sensor_dead[C_CLOUT] |
                         sensor_dead[C_CLFLW] | sensor_dead[C_CLPRS];

    wire dead_throttle = sensor_dead[C_THR1] | sensor_dead[C_THR2];
    wire dead_brake    = sensor_dead[C_BPRS] | sensor_dead[C_BSW];

    wire implaus_flow  = implausible_flags[C_CLFLW];

    //=========================================================================
    // 1  scoring engine
    //=========================================================================
    ivcu_health_scorer u_score (
        .clk_ai            (clk_ai),
        .rst_ai_n          (rst_ai_n),
        .sensor_status_flat(sensor_status_flat),
        .update_req        (update_req),
        .score_battery     (score_battery),
        .score_motor       (score_motor),
        .score_thermal     (score_thermal),
        .score_dynamics    (score_dynamics),
        .score_driver      (score_driver),
        .score_safety      (score_safety),
        .score_perception  (score_perception),
        .system_health_score(system_health_score),
        .status_battery    (status_battery),
        .status_motor      (status_motor),
        .status_thermal    (status_thermal),
        .status_dynamics   (status_dynamics),
        .status_driver     (status_driver),
        .status_safety     (status_safety),
        .status_perception (status_perception),
        .score_valid       (score_valid)
    );

    //=========================================================================
    // 2  battery
    //=========================================================================
    ivcu_battery_ai u_batt (
        .clk_ai               (clk_ai),
        .rst_ai_n             (rst_ai_n),
        .v_cell_v_min         (v_cvmin),
        .v_cell_v_max         (v_cvmax),
        .v_pack_v             (v_packv),
        .v_pack_i             (v_packi),
        .v_cell_t_min         (v_ctmin),
        .v_cell_t_max         (v_ctmax),
        .v_pack_t             (v_packt),
        .v_soc                (v_soc),
        .v_soh                (v_soh),
        .v_encl_p             (v_encp),
        .dead_cell_t          (dead_cell_t),
        .dead_cell_v          (dead_cell_v),
        .dead_soc             (dead_soc),
        .update_req           (update_req),
        .charge_inhibit       (charge_inhibit),
        .discharge_inhibit    (discharge_inhibit),
        .power_limit_pct      (batt_power_limit_pct),
        .thermal_runaway_alarm(thermal_runaway_alarm),
        .batt_cold            (batt_cold),
        .batt_hot             (batt_hot),
        .batt_low_soc         (batt_low_soc),
        .batt_worn            (batt_worn)
    );

    //=========================================================================
    // 3  motor
    //=========================================================================
    ivcu_motor_ai u_motor (
        .clk_ai          (clk_ai),
        .rst_ai_n        (rst_ai_n),
        .v_motor_rpm     (v_rpm),
        .v_rotor_pos     (v_rotor),
        .v_phase_i_a     (v_pha),
        .v_phase_i_b     (v_phb),
        .v_motor_temp    (v_mtemp),
        .v_inverter_temp (v_itemp),
        .v_dc_link_i     (v_dci),
        .dead_rotor      (dead_rotor),
        .dead_rpm        (dead_rpm),
        .dead_phase_i    (dead_phase_i),
        .dead_motor_temp (dead_mtemp),
        .dead_inv_temp   (dead_itemp),
        .update_req      (update_req),
        .motor_inhibit   (motor_inhibit),
        .torque_limit_pct(torque_limit_pct),
        .regen_inhibit   (regen_inhibit),
        .motor_hot       (motor_hot),
        .inverter_hot    (inverter_hot),
        .overspeed       (overspeed),
        .overcurrent     (overcurrent)
    );

    //=========================================================================
    // 4  thermal
    //=========================================================================
    ivcu_thermal_ai u_therm (
        .clk_ai          (clk_ai),
        .rst_ai_n        (rst_ai_n),
        .v_coolant_in    (v_clin),
        .v_coolant_out   (v_clout),
        .v_coolant_flow  (v_clflw),
        .v_coolant_press (v_clprs),
        .v_ambient_t     (v_ambt),
        .v_motor_temp    (v_mtemp),
        .v_inverter_temp (v_itemp),
        .v_cell_temp_max (v_ctmax),
        .dead_coolant    (dead_coolant),
        .implausible_flow(implaus_flow),
        .mode_is_car     (mode_is_car),
        .update_req      (update_req),
        .pump_pwm        (pump_pwm),
        .fan_pwm         (fan_pwm),
        .pump_running    (pump_running),
        .cooling_fault   (cooling_fault),
        .coolant_overtemp(coolant_overtemp)
    );

    //=========================================================================
    // 5  dynamics
    //=========================================================================
    ivcu_dynamics_ai u_dyn (
        .clk_ai            (clk_ai),
        .rst_ai_n          (rst_ai_n),
        .v_wspd_fa         (v_wfa),
        .v_wspd_fb         (v_wfb),
        .v_wspd_ra         (v_wra),
        .v_wspd_rb         (v_wrb),
        .v_yaw_rate        (v_yaw),
        .v_roll_rate       (v_roll),
        .v_brake_press     (v_bprs),
        .v_brake_switch    (v_bsw),
        .mode_is_car       (mode_is_car),
        .sensor_enable_s   (sensor_enable),
        .update_req        (update_req),
        .vehicle_speed_kph (vehicle_speed_kph),
        .vehicle_moving    (vehicle_moving),
        .vehicle_stationary(vehicle_stationary),
        .braking           (braking),
        .lean_estimate     (lean_estimate),
        .yaw_excessive     (yaw_excessive),
        .wheel_locked      (wheel_locked)
    );

    //=========================================================================
    // 6  driver input
    //=========================================================================
    ivcu_driver_input_ai u_driver (
        .clk_ai           (clk_ai),
        .rst_ai_n         (rst_ai_n),
        .v_throttle_1     (v_thr1),
        .v_brake_press    (v_bprs),
        .v_brake_switch   (v_bsw),
        .v_side_stand     (v_stand),
        .v_seat_occ       (v_seat),
        .v_gear_pos       (v_gear),
        .throttle_disagree(throttle_disagree),
        .dead_throttle    (dead_throttle),
        .dead_brake       (dead_brake),
        .mode_is_bike     (mode_is_bike),
        .sensor_enable_s  (sensor_enable),
        .bypass_active_s  (bypass_active),
        .update_req       (update_req),
        .torque_request   (torque_request),
        .regen_request    (regen_request),
        .throttle_fault   (throttle_fault),
        .brake_override   (brake_override),
        .rider_present    (rider_present),
        .stand_down       (stand_down),
        .gear_engaged     (gear_engaged)
    );

    //=========================================================================
    // 7  perception
    //=========================================================================
    ivcu_perception_ai u_percep (
        .clk_ai              (clk_ai),
        .rst_ai_n            (rst_ai_n),
        .st_camera           (st_cam),
        .st_radar            (st_rad),
        .st_lidar            (st_lid),
        .st_ultrasonic       (st_ult),
        .st_gps              (st_gps),
        .st_ambient_light    (st_algt),
        .st_rain             (st_rain),
        .mode_is_car         (mode_is_car),
        .update_req          (update_req),
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

endmodule

`default_nettype wire
