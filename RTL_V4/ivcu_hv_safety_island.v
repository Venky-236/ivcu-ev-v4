//=============================================================================
// ivcu_hv_safety_island.v  -  the high voltage safety island
//
// THE HEADLINE FEATURE OF THE WHOLE PROJECT, AND THE ONE V3 DID NOT HAVE.
//
// I searched the V3 source for contactor, pyro, HV, isolation and precharge.
// Zero hits.  The V3 answer to "cut the high voltage before the fire" was
// motor_enable going low, which does not open a contactor, does not bleed the
// DC link, and leaves the pack live.
//
//-----------------------------------------------------------------------------
// WHY IT IS AN ISLAND
//
// Everything else in this chip can be wrong without killing anyone.  This
// block cannot.  So it is architecturally separated:
//
//   OWN POWER DOMAIN   PD_HVSAFE has no enable pin.  It cannot be gated, in
//                      any sleep state, ever.
//   OWN RESET          rst_hvsafe_n comes from por_n alone, not ext_rst_n.  A
//                      watchdog bite must not clear a welded-contactor latch
//                      or a fired-pyro record.  Those are facts about the
//                      physical vehicle.
//   SLOWEST CLOCK      clk_aon at 10 MHz.  Nothing here needs speed, and the
//                      lowest-frequency domain is the one with the most timing
//                      margin.
//   NO INBOUND AUTHORITY  no AI block and no MCU write can command HV on.  The
//                      MCU may request, and may command off.  Only this
//                      island's own FSM closes a contactor.
//   NO FSM IN THE CRASH PATH  crash_force_open is a combinational override on
//                      the drive outputs.  See ivcu_hv_contactor_ctrl.
//
//-----------------------------------------------------------------------------
// WHAT IS DELIBERATELY NOT HERE
//
// Dual-channel lockstep.  You chose the full island rather than the lockstep
// variant, so the disconnect decision is computed once.  A production ASIL-D
// implementation computes it in two independent channels and forces the safe
// state on any mismatch.  The upgrade is clean - instantiate
// ivcu_hv_contactor_ctrl twice and compare - and this file is structured so
// that can be done later without restructuring anything around it.
// Recorded as a known, deliberate gap, not an oversight.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_hv_safety_island (
    //--- always-on domain ---------------------------------------------------
    input  wire        clk_aon,
    input  wire        rst_hvsafe_n,      // por_n only - never ext_rst_n

    //--- sensor domain, for the coherent snapshot ---------------------------
    input  wire        clk_sensor,
    input  wire        rst_sensor_n,
    input  wire [1023:0] sensor_value_flat,
    input  wire [63:0] sensor_dead,
    input  wire        sweep_done,

    //--- direct discrete pins, raw (synchronised inside this module) --------
    input  wire        crash_trig_front,
    input  wire        crash_trig_side,
    input  wire        hvil_raw,

    //--- requests and context -----------------------------------------------
    input  wire        hv_request,        // central safety FSM
    input  wire [1:0]  active_mode,
    input  wire [7:0]  vehicle_speed_kph,
    input  wire        service_clear,     // authenticated APB write

    //--- HV drive outputs, top-level pins -----------------------------------
    output wire        hv_contactor_pos_en,
    output wire        hv_contactor_neg_en,
    output wire        hv_precharge_en,
    output wire        hv_discharge_en,
    output wire        pyro_fuse_arm,
    output wire        pyro_fuse_fire,

    //--- status to the rest of the chip and to the display ------------------
    output wire        hv_on,
    output wire        hv_isolated,
    output wire        torque_inhibit,
    output wire [2:0]  hv_state,
    output wire [3:0]  hv_fault_code,
    output wire        pyro_fired,
    output wire [2:0]  pyro_state,
    output wire        hvil_ok,
    output wire        hvil_degraded,
    output wire        iso_warn,
    output wire        iso_fault,

    //--- crash information for the emergency response block ------------------
    output wire        crash_latched,
    output wire        crash_pyro,
    output wire [15:0] crash_peak_long,
    output wire [15:0] crash_peak_lat,
    output wire [2:0]  crash_direction,

    //--- thermal state at the moment of impact, for the SOS message ---------
    output wire [15:0] hv_cell_tmax,
    output wire [15:0] hv_encl_press,
    output wire [14:0] hv_sense_dead
);

    //=========================================================================
    // 1  synchronise the three direct discrete pins
    //
    // RULE R7.  These are asynchronous inputs from the vehicle harness; they
    // must not reach a state machine raw.
    //=========================================================================
    wire crash_front_s, crash_side_s, hvil_s;

    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b0)) u_sync_crash_f (
        .clk_dst   (clk_aon),
        .rst_dst_n (rst_hvsafe_n),
        .d_src     (crash_trig_front),
        .q_dst     (crash_front_s)
    );

    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b0)) u_sync_crash_s (
        .clk_dst   (clk_aon),
        .rst_dst_n (rst_hvsafe_n),
        .d_src     (crash_trig_side),
        .q_dst     (crash_side_s)
    );

    // RESET_VAL 0 = "loop open" is the safe assumption before it is proven
    // closed.  A chip that has just come out of reset has not verified that
    // the high voltage system is intact.
    ivcu_cdc_bit_sync #(.STAGES(2), .RESET_VAL(1'b0)) u_sync_hvil (
        .clk_dst   (clk_aon),
        .rst_dst_n (rst_hvsafe_n),
        .d_src     (hvil_raw),
        .q_dst     (hvil_s)
    );

    //=========================================================================
    // 2  coherent snapshot of every HV-relevant channel
    //=========================================================================
    wire [15:0] s_hvil, s_iso, s_bus_v, s_pre_v, s_fb_pos, s_fb_neg;
    wire [15:0] s_pack_v, s_dc_i, s_cell_tmax, s_encl_p;
    wire [15:0] s_crash_f, s_crash_s, s_tip, s_accel_long, s_accel_lat;
    wire [14:0] s_dead;
    wire        s_valid;

    ivcu_hv_sense_sync u_sense (
        .clk_sensor       (clk_sensor),
        .rst_sensor_n     (rst_sensor_n),
        .sensor_value_flat(sensor_value_flat),
        .sensor_dead      (sensor_dead),
        .sweep_done       (sweep_done),
        .clk_aon          (clk_aon),
        .rst_hvsafe_n     (rst_hvsafe_n),
        .q_hvil           (s_hvil),
        .q_iso            (s_iso),
        .q_bus_v          (s_bus_v),
        .q_pre_v          (s_pre_v),
        .q_fb_pos         (s_fb_pos),
        .q_fb_neg         (s_fb_neg),
        .q_pack_v         (s_pack_v),
        .q_dc_i           (s_dc_i),
        .q_cell_tmax      (s_cell_tmax),
        .q_encl_p         (s_encl_p),
        .q_crash_f        (s_crash_f),
        .q_crash_s        (s_crash_s),
        .q_tip            (s_tip),
        .q_accel_long     (s_accel_long),
        .q_accel_lat      (s_accel_lat),
        .q_dead           (s_dead),
        .q_valid          (s_valid)
    );

    assign hv_cell_tmax  = s_cell_tmax;
    assign hv_encl_press = s_encl_p;
    assign hv_sense_dead = s_dead;

    //=========================================================================
    // 3  the crash path
    //=========================================================================
    wire crash_force_open;

    ivcu_hv_crash_detect u_crash (
        .clk_aon           (clk_aon),
        .rst_hvsafe_n      (rst_hvsafe_n),
        .crash_trig_front_s(crash_front_s),
        .crash_trig_side_s (crash_side_s),
        .sense_crash_f     (s_crash_f),
        .sense_crash_s     (s_crash_s),
        .sense_tip         (s_tip),
        .sense_accel_long  (s_accel_long),
        .sense_accel_lat   (s_accel_lat),
        .sense_valid       (s_valid),
        .active_mode       (active_mode),
        .vehicle_speed_kph (vehicle_speed_kph),
        .service_clear     (service_clear),
        .crash_latched     (crash_latched),
        .crash_force_open  (crash_force_open),
        .crash_pyro        (crash_pyro),
        .crash_peak_long   (crash_peak_long),
        .crash_peak_lat    (crash_peak_lat),
        .crash_direction   (crash_direction)
    );

    //=========================================================================
    // 4  interlock and insulation monitors
    //
    // s_dead bit order is LSB-first in the declaration order of
    // ivcu_hv_sense_sync:  0 HVIL, 1 ISO, 2 BUS_V, 3 PRE_V, 4 FB_POS,
    // 5 FB_NEG, 6 PACK_V, 7 DC_I, 8 CELL_TMAX, 9 ENCL_P, 10 CRASH_F,
    // 11 CRASH_S, 12 TIP, 13 ACCEL_LONG, 14 ACCEL_LAT.
    //=========================================================================
    wire iso_ok;

    ivcu_hv_monitors u_mon (
        .clk_aon      (clk_aon),
        .rst_hvsafe_n (rst_hvsafe_n),
        .hvil_raw_s   (hvil_s),
        .sense_hvil   (s_hvil),
        .hvil_dead    (s_dead[0]),
        .sense_iso    (s_iso),
        .iso_dead     (s_dead[1]),
        .sense_valid  (s_valid),
        .hvil_ok      (hvil_ok),
        .hvil_degraded(hvil_degraded),
        .iso_ok       (iso_ok),
        .iso_warn     (iso_warn),
        .iso_fault    (iso_fault)
    );

    //=========================================================================
    // 5  pyrotechnic fuse
    //=========================================================================
    ivcu_hv_pyro_trigger u_pyro (
        .clk_aon       (clk_aon),
        .rst_hvsafe_n  (rst_hvsafe_n),
        .crash_pyro    (crash_pyro),
        .crash_latched (crash_latched),
        .service_clear (service_clear),
        .pyro_fuse_arm (pyro_fuse_arm),
        .pyro_fuse_fire(pyro_fuse_fire),
        .pyro_fired    (pyro_fired),
        .pyro_state    (pyro_state)
    );

    //=========================================================================
    // 6  precharge, contactors, discharge
    //=========================================================================
    ivcu_hv_contactor_ctrl u_ctrl (
        .clk_aon            (clk_aon),
        .rst_hvsafe_n       (rst_hvsafe_n),
        .hv_request         (hv_request),
        .crash_force_open   (crash_force_open),
        .hvil_ok            (hvil_ok),
        .iso_ok             (iso_ok),
        .service_clear      (service_clear),
        .sense_pack_v       (s_pack_v),
        .sense_pre_v        (s_pre_v),
        .sense_bus_v        (s_bus_v),
        .sense_dc_i         (s_dc_i),
        .sense_fb_pos       (s_fb_pos),
        .sense_fb_neg       (s_fb_neg),
        .sense_valid        (s_valid),
        .hv_contactor_pos_en(hv_contactor_pos_en),
        .hv_contactor_neg_en(hv_contactor_neg_en),
        .hv_precharge_en    (hv_precharge_en),
        .hv_discharge_en    (hv_discharge_en),
        .hv_on              (hv_on),
        .hv_isolated        (hv_isolated),
        .torque_inhibit     (torque_inhibit),
        .hv_state           (hv_state),
        .hv_fault_code      (hv_fault_code)
    );

endmodule

`default_nettype wire
