//=============================================================================
// ivcu_hv_crash_detect.v  -  the hardwired crash path
//
// THIS IS THE BLOCK THE WHOLE PROJECT EXISTS FOR.
//
// When an EV is hit hard enough, the high-voltage pack becomes the danger.
// It can arc, vent, ignite, and reignite hours later.  The single most
// valuable thing this chip does in the first fifty milliseconds is take the
// high voltage out of the vehicle before anyone touches it.
//
// So this path is deliberately stupid.  There is no AI block in it, no MCU
// write, no bus transaction, no arbitration, and no state machine round trip
// between the impact and the contactor drives going low.  Every one of those
// would be a place for the disconnect to not happen.
//
//   crash_trig_front (pin) -> 2-flop sync -> latch -> contactor_force_open
//
// Two clk_aon edges, about 200 ns, plus the contactor's own mechanical time.
// ivcu_hv_contactor_ctrl takes crash_force_open as a combinational override on
// its output drives, so even if its FSM is wedged in any state, the contactors
// open.
//
// TWO INDEPENDENT SOURCES, DELIBERATELY
//   1  dedicated crash discretes on their own pins - fast, and they work even
//      if the ADC has stopped answering entirely
//   2  the analog accelerometer channels through the normal scan - slower
//      (about 10 us) but they carry magnitude, which the discretes do not
//
// Either one alone opens the contactors.  Requiring both would mean a single
// failed sensor could prevent the disconnect, which is the wrong direction to
// fail in.  A spurious disconnect strands a vehicle; a missed one starts a
// fire.
//
// LATCHING
//   crash_latched is sticky.  You do not re-close the contactors after an
//   impact because the accelerometer stopped ringing.  It clears only on a
//   power cycle plus an authenticated service write.
//
// TIP-OVER IS NOT A CRASH BELOW WALKING PACE
//   A motorcycle falling over in a car park at 2 km/h should not fire the
//   pyrotechnic fuse and write off the harness.  Above TIPOVER_SPEED_MIN it is
//   a fall at speed and it is treated as severe.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_hv_crash_detect (
    input  wire        clk_aon,
    input  wire        rst_hvsafe_n,

    //--- direct discrete pins, already through ivcu_cdc_bit_sync -----------
    input  wire        crash_trig_front_s,
    input  wire        crash_trig_side_s,

    //--- analog snapshot from ivcu_hv_sense_sync (clk_aon) -----------------
    input  wire [15:0] sense_crash_f,
    input  wire [15:0] sense_crash_s,
    input  wire [15:0] sense_tip,
    input  wire [15:0] sense_accel_long,
    input  wire [15:0] sense_accel_lat,
    input  wire        sense_valid,

    //--- context ------------------------------------------------------------
    input  wire [1:0]  active_mode,
    input  wire [7:0]  vehicle_speed_kph,
    input  wire        service_clear,     // authenticated, from the APB block

    //--- outputs ------------------------------------------------------------
    output reg         crash_latched,     // sticky: an impact has happened
    output wire        crash_force_open,  // combinational override, no FSM
    output reg         crash_pyro,        // severe enough to sever the cable
    output reg  [15:0] crash_peak_long,   // held for the SOS message
    output reg  [15:0] crash_peak_lat,
    output reg  [2:0]  crash_direction    // which axis exceeded first
);

    //-------------------------------------------------------------------------
    // Offset-binary magnitudes.  The accelerometers read 2048 at rest and
    // swing either way, so the interesting quantity is distance from centre.
    //-------------------------------------------------------------------------
    localparam [15:0] ZERO_G     = 16'd2048;
    localparam [15:0] MAG_MODER  = `CRASH_THRESH_MODERATE - ZERO_G;  //  952
    localparam [15:0] MAG_HARD   = `CRASH_THRESH_HARD     - ZERO_G;  // 1352
    // CRASH_THRESH_MINOR is used by the severity classifier in the emergency
    // block, not here.  This module only answers "must the HV open" and "is it
    // severe enough to sever the cable".

    localparam [1:0]  M_CAR  = `MODE_CAR;
    localparam [1:0]  M_BIKE = `MODE_BIKE;

    localparam [2:0] DIR_NONE  = 3'd0,
                     DIR_FRONT = 3'd1,
                     DIR_SIDE  = 3'd2,
                     DIR_FALL  = 3'd3;

    function [15:0] mag;
        input [15:0] v;
        begin
            mag = (v > ZERO_G) ? (v - ZERO_G) : (ZERO_G - v);
        end
    endfunction

    wire [15:0] m_long = mag(sense_accel_long);
    wire [15:0] m_lat  = mag(sense_accel_lat);

    //-------------------------------------------------------------------------
    // Mode gating.  A motorcycle has no side-impact satellite and a car does
    // not fall over.  Firing the wrong one would be a false disconnect.
    //-------------------------------------------------------------------------
    wire is_car  = (active_mode == M_CAR);
    wire is_bike = (active_mode == M_BIKE);

    wire fall_at_speed = is_bike &&
                         (sense_tip > 16'd2048) &&
                         (vehicle_speed_kph > `TIPOVER_SPEED_MIN);

    //-------------------------------------------------------------------------
    // Source 1: the discrete pins.  These do not wait for a scan and they work
    // with a completely dead ADC.
    //-------------------------------------------------------------------------
    wire trig_discrete = crash_trig_front_s |
                         (crash_trig_side_s & is_car);

    //-------------------------------------------------------------------------
    // Source 2: the analog channels.  Slower, but they carry magnitude.
    //-------------------------------------------------------------------------
    wire trig_analog = (sense_crash_f > `CRASH_THRESH_MODERATE) |
                       ((sense_crash_s > `CRASH_THRESH_MODERATE) & is_car) |
                       (m_long > MAG_MODER) |
                       (m_lat  > MAG_MODER) |
                       fall_at_speed;

    wire crash_now = trig_discrete | trig_analog;

    //-------------------------------------------------------------------------
    // Pyro level.  Severing the HV cable destroys the harness, so it takes a
    // genuinely severe impact, not merely a disconnect-worthy one.
    //-------------------------------------------------------------------------
    wire pyro_now = (sense_crash_f > `CRASH_THRESH_HARD) |
                    ((sense_crash_s > `CRASH_THRESH_HARD) & is_car) |
                    (m_long > MAG_HARD) |
                    (m_lat  > MAG_HARD);

    //-------------------------------------------------------------------------
    // THE OVERRIDE.  Combinational, so it does not depend on this module's own
    // registers having updated, let alone anyone else's.
    //-------------------------------------------------------------------------
    assign crash_force_open = crash_now | crash_latched;

    always @(posedge clk_aon or negedge rst_hvsafe_n) begin
        if (!rst_hvsafe_n) begin
            crash_latched   <= 1'b0;
            crash_pyro      <= 1'b0;
            crash_peak_long <= ZERO_G;
            crash_peak_lat  <= ZERO_G;
            crash_direction <= DIR_NONE;
        end else if (service_clear && !crash_now) begin
            // an authorised technician, with the vehicle no longer in impact
            crash_latched   <= 1'b0;
            crash_pyro      <= 1'b0;
            crash_peak_long <= ZERO_G;
            crash_peak_lat  <= ZERO_G;
            crash_direction <= DIR_NONE;
        end else begin

            if (crash_now) begin
                crash_latched <= 1'b1;
            end

            if (pyro_now) begin
                crash_pyro <= 1'b1;
            end

            // record the direction of the FIRST axis to exceed, not the last -
            // once crash_direction is set it stays set, so the SOS message
            // reports where the vehicle was struck rather than where it ended
            // up after spinning
            if (crash_now && (crash_direction == DIR_NONE)) begin
                if (crash_trig_front_s || (sense_crash_f > `CRASH_THRESH_MODERATE)
                    || (m_long > MAG_MODER))
                    crash_direction <= DIR_FRONT;
                else if (fall_at_speed)
                    crash_direction <= DIR_FALL;
                else
                    crash_direction <= DIR_SIDE;
            end

            // peak-hold the accelerations for the emergency message
            if (sense_valid) begin
                if (m_long > mag(crash_peak_long)) crash_peak_long <= sense_accel_long;
                if (m_lat  > mag(crash_peak_lat )) crash_peak_lat  <= sense_accel_lat;
            end
        end
    end

endmodule

`default_nettype wire
