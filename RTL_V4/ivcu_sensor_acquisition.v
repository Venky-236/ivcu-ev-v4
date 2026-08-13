//=============================================================================
// ivcu_sensor_acquisition.v  -  the whole sensor input path, one wrapper
//
// Instantiates, in the clk_sensor domain:
//     ivcu_adc_scan_sequencer     21 pins carry all 64 channels
//     ivcu_sensor_conditioning    per-channel IIR filter and value store
//     ivcu_sensor_fault_detect    range, slew, stuck, no-response, debounce
//     ivcu_sensor_plausibility    sensors checked against sensors
//     ivcu_afe_serializer         3 pins power 64 analog front ends
//
// RULE R2 is enforced by hand here as well as by the linter: every port of
// every instance below is connected.  V3's u_diagnostic was instantiated with
// twelve inputs and not one of its seven outputs - 386 bits - because Verilog
// lets you omit a port silently and nothing complained.  It cost 118,908 um2,
// 18.8 % of the standard cell area, for logic whose results could not leave
// the module.
//
// THE ONE PIECE OF REAL LOGIC IN THIS FILE
// The status priority merge.  ivcu_sensor_fault_detect does not know about
// plausibility, and ivcu_sensor_plausibility does not know about ranges, so
// the two verdicts are combined here.  A disagreement between sensors is
// weaker evidence than a sensor that has stopped answering, so it only
// overrides SS_OK and SS_DEGRADED - it never masks NO_RESPONSE, STUCK,
// OUT_OF_RANGE, BYPASSED or DISABLED_BY_MODE.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_sensor_acquisition (
    input  wire          clk_sensor,
    input  wire          rst_sensor_n,

    //--- external ADC, 21 top-level pins ------------------------------------
    output wire [5:0]    adc_chan,
    output wire          adc_req,
    input  wire [11:0]   adc_data,
    input  wire          adc_valid,
    input  wire          adc_busy,

    //--- external analog front-end power chain, 3 top-level pins ------------
    output wire          afe_sclk,
    output wire          afe_sdata,
    output wire          afe_latch,
    output wire          afe_busy,

    //--- configuration from the mode and serviceability managers ------------
    input  wire [63:0]   sensor_enable,   // channels fitted in the active mode
    input  wire [63:0]   afe_power,       // front ends to keep powered
    input  wire [63:0]   bypass_active,   // running under a service permit

    //--- vehicle context ----------------------------------------------------
    input  wire          vehicle_moving,
    input  wire          hv_on,
    input  wire          pump_running,
    input  wire          braking,

    //--- results ------------------------------------------------------------
    output wire [1023:0] sensor_value_flat,  // 64 x 16, conditioned
    output wire [191:0]  sensor_status_flat, // 64 x 3,  merged verdict
    output wire [255:0]  sensor_conf_flat,   // 64 x 4
    output wire [63:0]   sensor_fault,       // any confirmed fault
    output wire [63:0]   sensor_dead,        // not answering, specifically
    output wire [63:0]   sensor_fresh,       // has answered at least once
    output wire [63:0]   implausible_flags,  // per-channel disagreement
    output wire          throttle_disagree,  // fast path: cut torque now
    output wire          sweep_done          // one sweep of all live channels
);

    //-------------------------------------------------------------------------
    // internal nets
    //-------------------------------------------------------------------------
    wire [5:0]    samp_chan;
    wire [15:0]   samp_data;
    wire          samp_valid;
    wire          samp_timeout;

    wire [191:0]  det_status_flat;
    wire [63:0]   det_fault;
    wire [63:0]   implausible;

    // ivcu_thermal_ai needs to know that the coolant flow channel was caught
    // disagreeing with the delta-T across the loop, so the verdict leaves this
    // wrapper as well as feeding the status merge below.
    assign implausible_flags = implausible;

    //=========================================================================
    // 1  scan sequencer - the 21-pin sensor interface
    //=========================================================================
    ivcu_adc_scan_sequencer u_seq (
        .clk_sensor   (clk_sensor),
        .rst_sensor_n (rst_sensor_n),
        .sensor_enable(sensor_enable),
        .adc_chan     (adc_chan),
        .adc_req      (adc_req),
        .adc_data     (adc_data),
        .adc_valid    (adc_valid),
        .adc_busy     (adc_busy),
        .samp_chan    (samp_chan),
        .samp_data    (samp_data),
        .samp_valid   (samp_valid),
        .samp_timeout (samp_timeout),
        .sweep_done   (sweep_done)
    );

    //=========================================================================
    // 2  conditioning - filter and store
    //=========================================================================
    ivcu_sensor_conditioning u_cond (
        .clk_sensor       (clk_sensor),
        .rst_sensor_n     (rst_sensor_n),
        .samp_chan        (samp_chan),
        .samp_data        (samp_data),
        .samp_valid       (samp_valid),
        .sensor_enable    (sensor_enable),
        .sensor_value_flat(sensor_value_flat),
        .sensor_fresh     (sensor_fresh)
    );

    //=========================================================================
    // 3  fault detection - five mechanisms, on the RAW sample
    //=========================================================================
    ivcu_sensor_fault_detect u_det (
        .clk_sensor        (clk_sensor),
        .rst_sensor_n      (rst_sensor_n),
        .samp_chan         (samp_chan),
        .samp_data         (samp_data),
        .samp_valid        (samp_valid),
        .samp_timeout      (samp_timeout),
        .sensor_enable     (sensor_enable),
        .sensor_fresh      (sensor_fresh),
        .bypass_active     (bypass_active),
        .vehicle_moving    (vehicle_moving),
        .hv_on             (hv_on),
        .sensor_status_flat(det_status_flat),
        .sensor_conf_flat  (sensor_conf_flat),
        .sensor_fault      (det_fault),
        .sensor_dead       (sensor_dead)
    );

    //=========================================================================
    // 4  plausibility - mechanism 6, on the CONDITIONED values
    //=========================================================================
    ivcu_sensor_plausibility u_plaus (
        .clk_sensor       (clk_sensor),
        .rst_sensor_n     (rst_sensor_n),
        .sensor_value_flat(sensor_value_flat),
        .sensor_enable    (sensor_enable),
        .sensor_fresh     (sensor_fresh),
        .pump_running     (pump_running),
        .braking          (braking),
        .implausible      (implausible),
        .throttle_disagree(throttle_disagree)
    );

    //=========================================================================
    // 5  analog front-end power chain
    //=========================================================================
    ivcu_afe_serializer u_afe (
        .clk_sensor   (clk_sensor),
        .rst_sensor_n (rst_sensor_n),
        .afe_enable   (afe_power),
        .afe_sclk     (afe_sclk),
        .afe_sdata    (afe_sdata),
        .afe_latch    (afe_latch),
        .afe_busy     (afe_busy)
    );

    //=========================================================================
    // STATUS MERGE
    //
    // Plausibility is weaker evidence than silence.  A channel that has stopped
    // answering is definitely broken; a channel that merely disagrees with its
    // partner might be the honest one of the pair.  So SS_IMPLAUSIBLE is
    // allowed to override only SS_OK and SS_DEGRADED.
    //=========================================================================
    genvar i;
    generate
        for (i = 0; i < `NUM_SENSORS; i = i + 1) begin : merge

            wire [2:0] s_det = det_status_flat[i*3 +: 3];

            wire overridable = (s_det == `SS_OK) || (s_det == `SS_DEGRADED);

            assign sensor_status_flat[i*3 +: 3] =
                (implausible[i] && overridable) ? `SS_IMPLAUSIBLE : s_det;

            assign sensor_fault[i] =
                det_fault[i] | (implausible[i] & sensor_enable[i]);

        end
    endgenerate

endmodule

`default_nettype wire
