//=============================================================================
// ivcu_central_safety_fsm.v  -  may this vehicle move?
//
// One block answers that question.  Everything else in the chip produces
// evidence; this is where the evidence becomes a decision, and it lives in the
// always-on domain at 10 MHz because it must keep working when every other
// domain is gated off.
//
//-----------------------------------------------------------------------------
// THE INHIBIT SOURCES, AND WHY EACH ONE IS ABSOLUTE
//
// There is no scoring here, no threshold on system_health_score, no "mostly
// fine".  Each source below independently stops the vehicle, and the FSM
// records which one did it so the display can say something useful instead of
// "fault".
//
//   MODE UNRESOLVED     the chip does not know what vehicle it is in, so it
//                       does not know which sensors should exist or whether
//                       there are airbags.  Nothing may happen.
//   CRITICAL SENSOR     any SAFETY_CRITICAL channel faulted.  Thirty-five of
//                       the sixty-four.  No bypass exists for these in
//                       hardware - see ivcu_serviceability_mgr.
//   PERMIT              a CONDITIONAL channel faulted and the rider has not
//                       taken a permit, or has used all five starts.
//   HV                  the safety island is not in HV_ON, or has a latched
//                       fault.  No traction without high voltage, obviously,
//                       but also: no pretending.
//   POWERTRAIN          the battery or motor domain says no - commutation
//                       lost, pack venting, cells below their floor.
//   THROTTLE            the two throttle tracks disagree.  Pedal position
//                       unknown.
//   STAND               side stand deployed on a motorcycle.
//   CRASH               an impact has been latched.  This one never clears
//                       without a technician.
//
//-----------------------------------------------------------------------------
// LIMP HOME IS A STATE, NOT A FLAG
//
// SAFE_LIMP is a distinct state from SAFE_ACTIVE because the vehicle behaves
// differently in it: speed capped, warning latched on the display, and the
// event already written to the fault log.  Making it a state rather than a
// modifier means the display and the actuator manager cannot accidentally
// treat a permitted vehicle as a normal one.
//
//-----------------------------------------------------------------------------
// WHAT THIS BLOCK DOES NOT DO
//
// It does not open contactors in a crash.  That path is combinational inside
// ivcu_hv_safety_island and does not pass through here or through any state
// machine.  If this FSM were in that path, a wedged state would be a vehicle
// that stays live after an impact.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_central_safety_fsm (
    input  wire        clk_aon,
    input  wire        rst_aon_n,

    //--- mode -----------------------------------------------------------------
    input  wire [1:0]  active_mode,
    input  wire        mode_resolved,

    //--- sensor verdicts, synchronised into clk_aon -------------------------
    input  wire [63:0] sensor_fault_s,
    input  wire [63:0] mask_critical,
    // bypass_active is deliberately NOT an input to this block.  See the note
    // on critical_fault below: this is the one place where relying on another
    // module having masked correctly would mean a vehicle moving with failed
    // brakes.  Not taking the signal at all is stronger than taking it and
    // choosing not to use it.

    //--- serviceability --------------------------------------------------------
    input  wire        permit_inhibit,
    input  wire        limp_home_active,

    //--- HV island --------------------------------------------------------------
    input  wire        hv_on,
    input  wire [3:0]  hv_fault_code,
    input  wire        crash_latched,
    input  wire        torque_inhibit,

    //--- powertrain verdicts, synchronised into clk_aon ---------------------
    input  wire        discharge_inhibit,
    input  wire        motor_inhibit,
    input  wire        thermal_runaway_alarm,
    input  wire        throttle_fault,
    input  wire        stand_down,

    //--- rider -----------------------------------------------------------------
    input  wire        ignition_stable,
    input  wire        service_clear,

    //--- decisions --------------------------------------------------------------
    output reg         hv_request,        // ask the island to close HV
    output reg         vehicle_enable,    // torque is permitted downstream
    output reg  [2:0]  safety_state,
    output reg  [3:0]  inhibit_reason,
    output reg         warn_latched       // display keeps a warning up
);

    localparam [2:0] SAFE_INIT      = 3'd0,
                     SAFE_INHIBIT   = 3'd1,
                     SAFE_READY     = 3'd2,   // permitted, HV closing
                     SAFE_ACTIVE    = 3'd3,   // normal operation
                     SAFE_LIMP      = 3'd4,   // permit active, speed capped
                     SAFE_EMERGENCY = 3'd5,   // impact
                     SAFE_FAULT     = 3'd6;   // latched, needs service

    localparam [3:0] IR_NONE        = 4'd0,
                     IR_MODE        = 4'd1,
                     IR_CRITICAL    = 4'd2,
                     IR_PERMIT      = 4'd3,
                     IR_HV          = 4'd4,
                     IR_POWERTRAIN  = 4'd5,
                     IR_THROTTLE    = 4'd6,
                     IR_STAND       = 4'd7,
                     IR_CRASH       = 4'd8,
                     IR_RUNAWAY     = 4'd9;

    //=========================================================================
    // EVIDENCE
    //
    // A critical fault counts, full stop.  There is no term here that could
    // mask one.
    //
    // ivcu_serviceability_mgr already ANDs every bypass request with
    // BYPASS_ELIGIBLE, and no SAFETY_CRITICAL channel appears in that
    // constant, so a bypassed critical channel should be impossible.  This
    // block does not depend on that being true.  It is the one place where
    // being wrong means a vehicle moves with failed brakes, and the cheapest
    // way to be certain is to have no signal here that could ever clear a
    // critical fault.
    //=========================================================================
    wire critical_fault = |(sensor_fault_s & mask_critical);

    wire hv_bad         = (hv_fault_code != `HVF_NONE);
    wire powertrain_bad = discharge_inhibit | motor_inhibit;

    //-------------------------------------------------------------------------
    // Priority order.  Most serious first, so inhibit_reason names the worst
    // thing wrong rather than whichever check happened to be written first.
    //-------------------------------------------------------------------------
    wire [3:0] reason_now =
          crash_latched         ? IR_CRASH      :
          thermal_runaway_alarm ? IR_RUNAWAY    :
          hv_bad                ? IR_HV         :
          (!mode_resolved ||
           (active_mode == `MODE_SAFE) ||
           (active_mode == `MODE_DETECT)) ? IR_MODE :
          critical_fault        ? IR_CRITICAL   :
          powertrain_bad        ? IR_POWERTRAIN :
          throttle_fault        ? IR_THROTTLE   :
          stand_down            ? IR_STAND      :
          permit_inhibit        ? IR_PERMIT     :
                                  IR_NONE;

    wire blocked = (reason_now != IR_NONE);

    // these two never clear on their own
    wire terminal = crash_latched | thermal_runaway_alarm |
                    (hv_fault_code == `HVF_CONTACTOR_WELD) |
                    (hv_fault_code == `HVF_DISCHARGE_TO);

    always @(posedge clk_aon or negedge rst_aon_n) begin
        if (!rst_aon_n) begin
            safety_state   <= SAFE_INIT;
            hv_request     <= 1'b0;
            vehicle_enable <= 1'b0;
            inhibit_reason <= IR_MODE;
            warn_latched   <= 1'b0;
        end else begin

            inhibit_reason <= reason_now;

            // any inhibit that has ever fired leaves a warning up until the
            // vehicle is next started cleanly
            if (blocked) begin
                warn_latched <= 1'b1;
            end else if (!ignition_stable) begin
                warn_latched <= 1'b0;
            end

            case (safety_state)

                //-------------------------------------------------------------
                // INIT - waiting for the mode arbiter.  Nothing is powered,
                // nothing is permitted.
                //-------------------------------------------------------------
                SAFE_INIT: begin
                    hv_request     <= 1'b0;
                    vehicle_enable <= 1'b0;
                    if (mode_resolved && (active_mode != `MODE_SAFE)) begin
                        safety_state <= SAFE_INHIBIT;
                    end
                end

                //-------------------------------------------------------------
                // INHIBIT - the resting state.  A vehicle sits here until
                // every check passes and the key is on.
                //-------------------------------------------------------------
                SAFE_INHIBIT: begin
                    hv_request     <= 1'b0;
                    vehicle_enable <= 1'b0;

                    if (terminal) begin
                        safety_state <= crash_latched ? SAFE_EMERGENCY
                                                      : SAFE_FAULT;
                    end else if (!blocked && ignition_stable) begin
                        safety_state <= SAFE_READY;
                    end
                end

                //-------------------------------------------------------------
                // READY - ask the HV island to close.  It will refuse if HVIL
                // or isolation say no; this block does not get to overrule it.
                //-------------------------------------------------------------
                SAFE_READY: begin
                    hv_request     <= 1'b1;
                    vehicle_enable <= 1'b0;

                    if (terminal) begin
                        safety_state <= crash_latched ? SAFE_EMERGENCY
                                                      : SAFE_FAULT;
                    end else if (blocked || !ignition_stable) begin
                        safety_state <= SAFE_INHIBIT;
                    end else if (hv_on && !torque_inhibit) begin
                        safety_state <= limp_home_active ? SAFE_LIMP
                                                         : SAFE_ACTIVE;
                    end
                end

                //-------------------------------------------------------------
                // ACTIVE - normal operation
                //-------------------------------------------------------------
                SAFE_ACTIVE: begin
                    hv_request     <= 1'b1;
                    vehicle_enable <= 1'b1;

                    if (terminal) begin
                        vehicle_enable <= 1'b0;
                        hv_request     <= 1'b0;
                        safety_state   <= crash_latched ? SAFE_EMERGENCY
                                                        : SAFE_FAULT;
                    end else if (blocked || !ignition_stable || !hv_on ||
                                 torque_inhibit) begin
                        vehicle_enable <= 1'b0;
                        safety_state   <= SAFE_INHIBIT;
                    end else if (limp_home_active) begin
                        safety_state <= SAFE_LIMP;
                    end
                end

                //-------------------------------------------------------------
                // LIMP - a permit is active.  A distinct state, not a flag on
                // ACTIVE, so nothing downstream can mistake this for normal.
                //-------------------------------------------------------------
                SAFE_LIMP: begin
                    hv_request     <= 1'b1;
                    vehicle_enable <= 1'b1;
                    warn_latched   <= 1'b1;   // stays up for the whole permit

                    if (terminal) begin
                        vehicle_enable <= 1'b0;
                        hv_request     <= 1'b0;
                        safety_state   <= crash_latched ? SAFE_EMERGENCY
                                                        : SAFE_FAULT;
                    end else if (blocked || !ignition_stable || !hv_on ||
                                 torque_inhibit) begin
                        vehicle_enable <= 1'b0;
                        safety_state   <= SAFE_INHIBIT;
                    end else if (!limp_home_active) begin
                        // the sensor was replaced and the permit ended
                        safety_state <= SAFE_ACTIVE;
                    end
                end

                //-------------------------------------------------------------
                // EMERGENCY - an impact.  Terminal until a technician clears
                // it.  The contactors are already open; that did not happen
                // here and did not wait for this state.
                //-------------------------------------------------------------
                SAFE_EMERGENCY: begin
                    hv_request     <= 1'b0;
                    vehicle_enable <= 1'b0;
                    warn_latched   <= 1'b1;
                    if (service_clear && !crash_latched) begin
                        safety_state <= SAFE_INHIBIT;
                    end
                end

                //-------------------------------------------------------------
                // FAULT - welded contactor, failed discharge, thermal runaway.
                // The vehicle looks fine and is not.  Only service releases it.
                //-------------------------------------------------------------
                SAFE_FAULT: begin
                    hv_request     <= 1'b0;
                    vehicle_enable <= 1'b0;
                    warn_latched   <= 1'b1;
                    if (service_clear && !terminal) begin
                        safety_state <= SAFE_INHIBIT;
                    end
                end

                default: safety_state <= SAFE_INIT;

            endcase
        end
    end

endmodule

`default_nettype wire
