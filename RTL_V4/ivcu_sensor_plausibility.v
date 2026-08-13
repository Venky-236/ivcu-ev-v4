//=============================================================================
// ivcu_sensor_plausibility.v  -  mechanism 6: sensors checked against sensors
//
// The first five mechanisms in ivcu_sensor_fault_detect ask "is this reading
// possible?"  This block asks a harder and more useful question: "do these
// readings agree with each other?"
//
// It matters most for the channels where a time-based stuck check cannot work.
// A battery cell temperature in a parked garage is legitimately constant for
// hours, so watching it for movement tells you nothing.  But a cell
// temperature that disagrees with the pack temperature and with the other cell
// channels is a dead sensor, and that is detectable at any moment, parked or
// moving.  That is why the stuck check is switched off for temperatures and
// this block covers them instead.
//
// WHEN TWO SENSORS DISAGREE, ONE OF THEM IS LYING.  Sometimes you know which:
// if PACK_TEMP sits outside the min/max bracket reported by the cell channels,
// arithmetic says PACK_TEMP is the wrong one.  Where the design cannot tell,
// both channels are flagged and the vehicle takes the conservative action.
//
// THE THROTTLE CHECK IS THE ONE THAT CANNOT WAIT
// Every real drive-by-wire system runs two throttle tracks with opposite
// slopes so their sum is constant.  If the sum drifts, one track has failed
// and the pedal position is unknown.  There is exactly one safe response and
// it is to command zero torque immediately.  throttle_disagree is brought out
// separately from the status mask so it can reach the torque path without
// waiting for anything else.
//
// CONTACTOR FEEDBACK is deliberately NOT checked here.  Comparing the aux
// contacts against the commanded state needs the command, which lives in the
// HV safety island.  Putting it here would mean routing a safety-critical
// command out of the island and back in.  It stays inside, in
// ivcu_hv_contactor_ctrl.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_sensor_plausibility (
    input  wire          clk_sensor,
    input  wire          rst_sensor_n,

    // --- conditioned values, channel i at [i*16 +: 16] ---------------------
    input  wire [1023:0] sensor_value_flat,
    input  wire [63:0]   sensor_enable,
    input  wire [63:0]   sensor_fresh,

    // --- context ------------------------------------------------------------
    input  wire          pump_running,   // coolant pump commanded on
    input  wire          braking,        // brake applied - wheels may differ

    // --- results -------------------------------------------------------------
    output reg  [63:0]   implausible,      // per-channel disagreement flag
    output reg           throttle_disagree // fast path: cut torque NOW
);

    //-------------------------------------------------------------------------
    // Channel indices as plain integers.
    //
    // The `S_* macros are sized literals (6'dNN).  Using one directly in a
    // part-select index expression - `S_THROTTLE_POS_1*16 - would evaluate the
    // multiply in 6-bit width and silently truncate 704 to 0b000000.  Widening
    // them to integer localparams first makes the arithmetic honest.  This is
    // a small thing that would have been a very confusing bug.
    //-------------------------------------------------------------------------
    localparam integer C_THR1     = `S_THROTTLE_POS_1;
    localparam integer C_THR2     = `S_THROTTLE_POS_2;
    localparam integer C_RPM      = `S_MOTOR_RPM;
    localparam integer C_WFA      = `S_WSPD_FRONT_A;
    localparam integer C_WRA      = `S_WSPD_REAR_A;
    localparam integer C_IPACK    = `S_PACK_CURRENT;
    localparam integer C_IDCL     = `S_DC_LINK_CURRENT;
    localparam integer C_TCMIN    = `S_CELL_TEMP_MIN;
    localparam integer C_TCMAX    = `S_CELL_TEMP_MAX;
    localparam integer C_TPACK    = `S_PACK_TEMP;
    localparam integer C_VCMIN    = `S_CELL_VOLT_MIN;
    localparam integer C_VCMAX    = `S_CELL_VOLT_MAX;
    localparam integer C_CLIN     = `S_COOLANT_TEMP_IN;
    localparam integer C_CLOUT    = `S_COOLANT_TEMP_OUT;
    localparam integer C_CLFLOW   = `S_COOLANT_FLOW;

    // convenience taps
    wire [15:0] v_thr1  = sensor_value_flat[C_THR1  *16 +: 16];
    wire [15:0] v_thr2  = sensor_value_flat[C_THR2  *16 +: 16];
    wire [15:0] v_rpm   = sensor_value_flat[C_RPM   *16 +: 16];
    wire [15:0] v_wfa   = sensor_value_flat[C_WFA   *16 +: 16];
    wire [15:0] v_wra   = sensor_value_flat[C_WRA   *16 +: 16];
    wire [15:0] v_ipack = sensor_value_flat[C_IPACK *16 +: 16];
    wire [15:0] v_idcl  = sensor_value_flat[C_IDCL  *16 +: 16];
    wire [15:0] v_tcmin = sensor_value_flat[C_TCMIN *16 +: 16];
    wire [15:0] v_tcmax = sensor_value_flat[C_TCMAX *16 +: 16];
    wire [15:0] v_tpack = sensor_value_flat[C_TPACK *16 +: 16];
    wire [15:0] v_vcmin = sensor_value_flat[C_VCMIN *16 +: 16];
    wire [15:0] v_vcmax = sensor_value_flat[C_VCMAX *16 +: 16];
    wire [15:0] v_clin  = sensor_value_flat[C_CLIN  *16 +: 16];
    wire [15:0] v_clout = sensor_value_flat[C_CLOUT *16 +: 16];

    //-------------------------------------------------------------------------
    // Tolerances.  RULE R1 - localparams, never ports.
    //-------------------------------------------------------------------------
    localparam [15:0] THR_SUM_NOM  = 16'd4095;  // two inverse-slope tracks
    localparam [15:0] THR_SUM_TOL  = 16'd200;   // about 5 % of full scale
    localparam [15:0] MOTION_HI    = 16'd300;   // clearly moving
    localparam [15:0] MOTION_LO    = 16'd100;   // clearly stopped
    localparam [15:0] CUR_TOL      = 16'd400;   // about 100 A
    localparam [15:0] TEMP_TOL     = 16'd205;   // about 10 C
    localparam [15:0] WHEEL_TOL    = 16'd800;
    localparam [15:0] DELTA_T_MIN  = 16'd20;    // about 1 C across the radiator

    // every check must persist this many sweeps before it is believed
    localparam [7:0]  PERSIST      = 8'd64;     // about 0.65 ms

    //-------------------------------------------------------------------------
    // Absolute-difference helper, written as a comparison rather than a
    // subtract-and-negate so it stays unsigned throughout.
    //-------------------------------------------------------------------------
    function [15:0] absdiff;
        input [15:0] a;
        input [15:0] b;
        begin
            absdiff = (a > b) ? (a - b) : (b - a);
        end
    endfunction

    // a pair is only comparable if both channels are fitted and have answered
    function pair_live;
        input ea; input fa;
        input eb; input fb;
        begin
            pair_live = ea & fa & eb & fb;
        end
    endfunction

    //=========================================================================
    // THE SEVEN CHECKS
    //=========================================================================

    // 1  throttle tracks: sum must stay near full scale
    wire thr_ok_live = pair_live(sensor_enable[C_THR1], sensor_fresh[C_THR1],
                                 sensor_enable[C_THR2], sensor_fresh[C_THR2]);
    wire [16:0] thr_sum = {1'b0, v_thr1} + {1'b0, v_thr2};
    wire        thr_bad = thr_ok_live &&
                          (absdiff(thr_sum[15:0], THR_SUM_NOM) > THR_SUM_TOL);

    // 2  motor speed against wheel speed: one says moving, the other stopped
    wire rpm_live = pair_live(sensor_enable[C_RPM], sensor_fresh[C_RPM],
                              sensor_enable[C_WFA], sensor_fresh[C_WFA]);
    wire rpm_bad  = rpm_live &&
                    (((v_rpm > MOTION_HI) && (v_wfa < MOTION_LO)) ||
                     ((v_wfa > MOTION_HI) && (v_rpm < MOTION_LO)));

    // 3  pack current against DC link current
    wire cur_live = pair_live(sensor_enable[C_IPACK], sensor_fresh[C_IPACK],
                              sensor_enable[C_IDCL],  sensor_fresh[C_IDCL]);
    wire cur_bad  = cur_live && (absdiff(v_ipack, v_idcl) > CUR_TOL);

    // 4  cell temperature ordering: min must not exceed max
    wire tord_live = pair_live(sensor_enable[C_TCMIN], sensor_fresh[C_TCMIN],
                               sensor_enable[C_TCMAX], sensor_fresh[C_TCMAX]);
    wire tord_bad  = tord_live && (v_tcmin > (v_tcmax + TEMP_TOL));

    // 5  pack temperature must sit inside the cell bracket.  This is the check
    //    that replaces stuck detection on the thermal channels.
    wire tpack_live = sensor_enable[C_TPACK] & sensor_fresh[C_TPACK] & tord_live;
    wire tpack_bad  = tpack_live &&
                      ((v_tpack + TEMP_TOL < v_tcmin) ||
                       (v_tpack > v_tcmax + TEMP_TOL));

    // 6  cell voltage ordering: min must not exceed max
    wire vord_live = pair_live(sensor_enable[C_VCMIN], sensor_fresh[C_VCMIN],
                               sensor_enable[C_VCMAX], sensor_fresh[C_VCMAX]);
    wire vord_bad  = vord_live && (v_vcmin > v_vcmax);

    // 7  coolant: with the pump running there must be a temperature rise
    //    across the loop.  No rise means no flow.
    wire cool_live = pair_live(sensor_enable[C_CLIN],  sensor_fresh[C_CLIN],
                               sensor_enable[C_CLOUT], sensor_fresh[C_CLOUT]);
    wire cool_bad  = cool_live && pump_running &&
                     (absdiff(v_clout, v_clin) < DELTA_T_MIN);

    // 8  front against rear wheel speed, only meaningful when not braking
    wire whl_live = pair_live(sensor_enable[C_WFA], sensor_fresh[C_WFA],
                              sensor_enable[C_WRA], sensor_fresh[C_WRA]);
    wire whl_bad  = whl_live && !braking &&
                    (absdiff(v_wfa, v_wra) > WHEEL_TOL);

    //=========================================================================
    // PERSISTENCE.  One counter per check.  Nothing is believed on a single
    // sweep - a transient disagreement during a gear change or a hard launch
    // must not latch a fault and start burning the rider's permits.
    //=========================================================================
    localparam integer N_CHECK = 8;

    wire [N_CHECK-1:0] chk_raw = { whl_bad,   cool_bad,  vord_bad, tpack_bad,
                                   tord_bad,  cur_bad,   rpm_bad,  thr_bad };

    reg  [7:0] cnt [0:N_CHECK-1];
    reg  [N_CHECK-1:0] chk_ok;      // confirmed disagreement

    integer k;
    always @(posedge clk_sensor or negedge rst_sensor_n) begin
        if (!rst_sensor_n) begin
            chk_ok <= {N_CHECK{1'b0}};
            for (k = 0; k < N_CHECK; k = k + 1) begin
                cnt[k] <= 8'd0;
            end
        end else begin
            for (k = 0; k < N_CHECK; k = k + 1) begin
                if (chk_raw[k]) begin
                    if (cnt[k] != 8'hFF) cnt[k] <= cnt[k] + 8'd1;
                end else begin
                    if (cnt[k] != 8'd0)  cnt[k] <= cnt[k] - 8'd1;
                end
                chk_ok[k] <= (cnt[k] >= PERSIST);
            end
        end
    end

    wire cf_thr   = chk_ok[0];
    wire cf_rpm   = chk_ok[1];
    wire cf_cur   = chk_ok[2];
    wire cf_tord  = chk_ok[3];
    wire cf_tpack = chk_ok[4];
    wire cf_vord  = chk_ok[5];
    wire cf_cool  = chk_ok[6];
    wire cf_whl   = chk_ok[7];

    //=========================================================================
    // MAP CONFIRMED DISAGREEMENTS ONTO CHANNELS
    //
    // Where the design cannot tell which of a pair is lying, both are flagged
    // and the vehicle takes the conservative action.  Where arithmetic does
    // identify the culprit - check 5 - only that channel is flagged.
    //=========================================================================
    always @(posedge clk_sensor or negedge rst_sensor_n) begin
        if (!rst_sensor_n) begin
            implausible       <= 64'd0;
            throttle_disagree <= 1'b0;
        end else begin
            implausible <= 64'd0;

            if (cf_thr) begin
                implausible[C_THR1]  <= 1'b1;
                implausible[C_THR2]  <= 1'b1;
            end
            if (cf_rpm) begin
                implausible[C_RPM]   <= 1'b1;
                implausible[C_WFA]   <= 1'b1;
            end
            if (cf_cur) begin
                implausible[C_IPACK] <= 1'b1;
                implausible[C_IDCL]  <= 1'b1;
            end
            if (cf_tord) begin
                implausible[C_TCMIN] <= 1'b1;
                implausible[C_TCMAX] <= 1'b1;
            end
            // arithmetic identifies the culprit here: the bracket comes from
            // two channels that agree with each other, so the outlier is the
            // pack sensor
            if (cf_tpack) begin
                implausible[C_TPACK] <= 1'b1;
            end
            if (cf_vord) begin
                implausible[C_VCMIN] <= 1'b1;
                implausible[C_VCMAX] <= 1'b1;
            end
            // no temperature rise with the pump running: the flow sensor is
            // the channel that claims flow exists
            if (cf_cool) begin
                implausible[C_CLFLOW] <= 1'b1;
            end
            if (cf_whl) begin
                implausible[C_WFA]   <= 1'b1;
                implausible[C_WRA]   <= 1'b1;
            end

            // the fast path - not gated by anything else
            throttle_disagree <= cf_thr;
        end
    end

endmodule

`default_nettype wire
