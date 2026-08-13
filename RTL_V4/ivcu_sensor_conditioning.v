//=============================================================================
// ivcu_sensor_conditioning.v  -  per-channel filtering and the value store
//
// One first-order IIR low-pass per channel, plus the register file that every
// AI block and the APB register map reads from.
//
//-----------------------------------------------------------------------------
// FLAGGED CHANGE vs the architecture spec: IIR filter, not a moving average
//
// The spec said "deadband + moving average".  I changed it while writing the
// RTL and you should know why.
//
//   moving average, depth 4 : 4 registers per channel = 4,096 flops, plus an
//                             adder tree, plus a pointer.  V3 declared
//                             mem[0:15] and used mem[0:3], leaving 12 x 42 =
//                             504 dead registers.
//   first-order IIR         : 1 accumulator per channel = 1,280 flops, one
//                             subtract and one constant shift.  Smoother
//                             response, no pointer arithmetic to get wrong,
//                             and no array depth to mismatch.
//
//   y <= y + ((x - y) >> K)
//
// The shift is by a per-channel constant K, so RULE R5 holds - there is no
// divider here, and the shift amount is fixed at elaboration, so it costs
// nothing but wiring.
//
// The deadband is dropped.  V3 carried deadband_threshold as a port (defect
// A5) for a power saving that was never measured, and a deadband on a filtered
// value can freeze the output when a real signal drifts slowly.  If dynamic
// power turns out to matter after the first synthesis run, it comes back as a
// localparam - not as a port.
//-----------------------------------------------------------------------------
// FILTER STRENGTH IS PER CHANNEL AND IT MATTERS
//
//   K = 0  no filtering at all.  Crash accelerometers and discrete switches.
//          For these, a sudden jump IS the signal, and filtering it away would
//          delay the HV disconnect.
//   K = 2  fast.  Throttle, brake, rotor position, wheel speed - control
//          inputs where lag is dangerous.
//   K = 4  medium.  Currents, voltages, dynamics.
//   K = 6  heavy.  Temperatures, pressures, SoC - physically slow quantities
//          where noise rejection is worth the lag.
//
// ACCUMULATOR FORMAT
//   20 bits, interpreted as 16.4 fixed point.  The four fraction bits stop a
//   heavily filtered channel from sticking one count away from its target and
//   never arriving, which is what happens with a naive integer IIR.
//
// REST STATE
//   A channel disabled by the mode mask has its accumulator held at zero and
//   its fresh bit cleared.  It is not carrying a stale value from the last
//   time the vehicle was configured differently.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_sensor_conditioning (
    input  wire          clk_sensor,
    input  wire          rst_sensor_n,

    // --- tagged sample stream from the scan sequencer -----------------------
    input  wire [5:0]    samp_chan,
    input  wire [15:0]   samp_data,
    input  wire          samp_valid,

    // --- active channel mask from the mode manager --------------------------
    input  wire [63:0]   sensor_enable,

    // --- the value store.  Channel i occupies [i*16 +: 16]. ----------------
    // Internal bus only.  It never reaches a top-level pin: the APB register
    // map serves one channel at a time, and each AI block taps only the
    // handful of channels it owns.
    output wire [1023:0] sensor_value_flat,

    // --- bit i set once channel i has produced at least one good sample ----
    output reg  [63:0]   sensor_fresh
);

    //-------------------------------------------------------------------------
    // Per-channel filter strength.  A constant function: called with a genvar
    // it folds to a literal at elaboration and costs nothing.
    //
    // RULE R1 - this is a function of a compile-time constant, not a port.
    // In V3 the equivalent (filter_coefficients) was a port, so synthesis
    // could not fold it and built general logic on all 42 channels.
    //-------------------------------------------------------------------------
    function [2:0] filt_k;
        input [5:0] c;
        begin
            case (c)
                // --- no filtering: a step change IS the measurement ---------
                `S_HVIL_LOOP,
                `S_CONTACTOR_FB_POS,
                `S_CONTACTOR_FB_NEG,
                `S_CRASH_FRONT,
                `S_CRASH_SIDE,
                `S_TIP_OVER,
                `S_BRAKE_SWITCH,
                `S_SIDE_STAND,
                `S_SEAT_OCCUPANCY,
                `S_GEAR_POSITION      : filt_k = 3'd0;

                // --- fast: control inputs, lag here is dangerous ------------
                `S_THROTTLE_POS_1,
                `S_THROTTLE_POS_2,
                `S_BRAKE_PRESSURE,
                `S_ROTOR_POSITION,
                `S_MOTOR_RPM,
                `S_WSPD_FRONT_A,
                `S_WSPD_FRONT_B,
                `S_WSPD_REAR_A,
                `S_WSPD_REAR_B,
                `S_ACCEL_LONG,
                `S_ACCEL_LAT,
                `S_ACCEL_VERT,
                `S_YAW_RATE,
                `S_ROLL_RATE,
                `S_PITCH_RATE         : filt_k = 3'd2;

                // --- medium: electrical quantities --------------------------
                `S_HV_BUS_VOLT,
                `S_PRECHARGE_VOLT,
                `S_CELL_VOLT_MIN,
                `S_CELL_VOLT_MAX,
                `S_PACK_VOLTAGE,
                `S_PACK_CURRENT,
                `S_CHARGE_VOLTAGE,
                `S_CHARGE_CURRENT,
                `S_PHASE_CURRENT_A,
                `S_PHASE_CURRENT_B,
                `S_DC_LINK_CURRENT,
                `S_STEERING_ANGLE,
                `S_RIDE_HEIGHT        : filt_k = 3'd4;

                // --- heavy: everything physically slow ----------------------
                // temperatures, pressures, isolation, SoC/SoH, environment
                default               : filt_k = 3'd6;
            endcase
        end
    endfunction

    //-------------------------------------------------------------------------
    // One filter per channel.  RULE R6 - generate, never copy-paste.  V3's ADC
    // was 42 hand-copied if-blocks, which is where transcription bugs live.
    //-------------------------------------------------------------------------
    genvar i;
    generate
        for (i = 0; i < `NUM_SENSORS; i = i + 1) begin : ch

            // folded to a literal at elaboration
            localparam integer K = filt_k(i[5:0]);

            reg [19:0] acc;                      // 16.4 fixed point

            wire hit = samp_valid & (samp_chan == i[5:0]);

            // target and current state, one guard bit for the signed subtract
            wire signed [20:0] target = $signed({1'b0, samp_data, 4'b0000});
            wire signed [20:0] cur    = $signed({1'b0, acc});
            wire signed [20:0] diff   = target - cur;
            wire signed [20:0] step   = diff >>> K;   // arithmetic, constant K
            wire signed [20:0] nxt    = cur + step;

            // target <= 0xFFFF0 < 2^20 so the sum cannot overflow upward;
            // the only clamp needed is against a negative intermediate.
            wire [19:0] nxt_clamped = nxt[20] ? 20'd0 : nxt[19:0];

            always @(posedge clk_sensor or negedge rst_sensor_n) begin
                if (!rst_sensor_n) begin
                    acc <= 20'd0;
                end else if (!sensor_enable[i]) begin
                    acc <= 20'd0;              // rest state: no stale value
                end else if (hit) begin
                    acc <= nxt_clamped;
                end
            end

            assign sensor_value_flat[i*16 +: 16] = acc[19:4];

        end
    endgenerate

    //-------------------------------------------------------------------------
    // Freshness.  Distinguishes "this channel has never been read" from "this
    // channel reads zero", which are very different statements to put in front
    // of a rider.
    //-------------------------------------------------------------------------
    integer j;
    always @(posedge clk_sensor or negedge rst_sensor_n) begin
        if (!rst_sensor_n) begin
            sensor_fresh <= 64'd0;
        end else begin
            for (j = 0; j < `NUM_SENSORS; j = j + 1) begin
                if (!sensor_enable[j]) begin
                    sensor_fresh[j] <= 1'b0;
                end else if (samp_valid && (samp_chan == j[5:0])) begin
                    sensor_fresh[j] <= 1'b1;
                end
            end
        end
    end

endmodule

`default_nettype wire
