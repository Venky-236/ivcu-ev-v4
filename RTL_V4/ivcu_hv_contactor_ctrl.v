//=============================================================================
// ivcu_hv_contactor_ctrl.v  -  precharge, contactor control, discharge
//
// V3 had none of this.  Its idea of a high voltage disconnect was driving
// motor_enable low, which stops the inverter switching and leaves the
// contactors closed, the DC link at pack voltage, and the whole vehicle live.
// A fire crew cutting that vehicle open finds 400 volts.
//
//-----------------------------------------------------------------------------
// WHY YOU CANNOT JUST CLOSE A CONTACTOR
//
// The inverter's DC link is tens of millifarads sitting at zero volts.  Close
// 400 V onto it directly and the inrush is a fault-level current that arcs
// across the contacts as they approach and welds them shut.  A welded
// contactor is a vehicle that can never isolate its battery again - the exact
// failure this whole chip exists to prevent, caused by the chip itself.
//
// So the sequence is: close the negative contactor and a precharge relay in
// series with a resistor, let the DC link charge through it, confirm the link
// has reached 95 % of pack voltage, and only then close the positive
// contactor and drop the precharge relay out.
//
// If the link does not reach 95 % within 500 ms, something downstream is
// shorted.  That is a latched fault with no retry.  Retrying into a short is
// how you turn a diagnosable fault into a fire.
//
//-----------------------------------------------------------------------------
// WHY OPENING IS ALSO NOT SIMPLE - AND THE ONE PLACE IT IS
//
// A DC contactor interrupting 300 A draws an arc that fuses the contacts.  So
// a normal shutdown ramps torque to zero first, waits for the DC link current
// to fall near zero, and only then drops the coils.  That is HV_OPENING.
//
// In a crash there is no time to ramp, and a welded contactor is a far smaller
// problem than a live pack in a burning vehicle.  So the crash path skips the
// ramp entirely: crash_force_open is a combinational override on the drive
// outputs, below.  This is the single place where the two behaviours diverge
// and it is deliberate.
//
//-----------------------------------------------------------------------------
// THE COMBINATIONAL OVERRIDE
//
//     assign hv_contactor_pos_en = pos_cmd & ~crash_force_open;
//
// The FSM can be in any state, wedged, or mid-transition.  It does not matter.
// If crash_force_open is high the drives are low.  There is no state machine
// in the crash disconnect path - that is the entire point.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_hv_contactor_ctrl (
    input  wire        clk_aon,
    input  wire        rst_hvsafe_n,

    //--- request and overrides ----------------------------------------------
    input  wire        hv_request,        // central FSM asks for HV on
    input  wire        crash_force_open,  // combinational, from crash detect
    input  wire        hvil_ok,
    input  wire        iso_ok,
    input  wire        service_clear,     // authenticated fault clear

    //--- coherent sensor snapshot (clk_aon) ---------------------------------
    input  wire [15:0] sense_pack_v,
    input  wire [15:0] sense_pre_v,
    input  wire [15:0] sense_bus_v,
    input  wire [15:0] sense_dc_i,
    input  wire [15:0] sense_fb_pos,
    input  wire [15:0] sense_fb_neg,
    input  wire        sense_valid,

    //--- contactor and relay drives, top-level pins -------------------------
    output wire        hv_contactor_pos_en,
    output wire        hv_contactor_neg_en,
    output wire        hv_precharge_en,
    output wire        hv_discharge_en,

    //--- status --------------------------------------------------------------
    output wire        hv_on,             // HV is live and usable
    output reg         hv_isolated,       // confirmed open AND link bled down
    output reg         torque_inhibit,    // powertrain must command zero torque
    output wire [2:0]  hv_state,
    output reg  [3:0]  hv_fault_code
);

    //=========================================================================
    // CONSTANTS.  RULE R1 - not one of these is a port.
    //=========================================================================
    localparam [24:0] T_PRECHARGE = `PRECHARGE_TIMEOUT_CYC;  // 500 ms
    localparam [24:0] T_CONTACTOR = `CONTACTOR_TIMEOUT_CYC;  // 100 ms
    localparam [24:0] T_DISCHARGE = `DISCHARGE_TIMEOUT_CYC;  //   2 s
    localparam [24:0] T_RAMP      = `TORQUE_RAMP_TIMEOUT;    // 200 ms

    localparam [15:0] V_TOUCH_SAFE = `HV_TOUCH_SAFE_CNT;     // about 60 V
    localparam [15:0] FB_CLOSED_TH = 16'd2048;               // discrete midpoint
    localparam [15:0] ZERO_CURRENT = 16'd2048;               // offset binary
    localparam [15:0] I_SAFE_MAG   = `I_SAFE_OPEN_CNT - ZERO_CURRENT; // 100

    //-------------------------------------------------------------------------
    // Precharge target: 95 % of pack voltage, without a multiplier and without
    // a divider (RULE R5).
    //
    //   1 - 1/32 - 1/64 = 0.953125
    //
    // Two shifts and two subtracts.  The error against a true 95 % is 0.3 %,
    // which is far inside the tolerance of the measurement itself.
    //-------------------------------------------------------------------------
    wire [15:0] precharge_target =
        sense_pack_v - (sense_pack_v >> 5) - (sense_pack_v >> 6);

    //-------------------------------------------------------------------------
    // Discrete decode and magnitudes
    //-------------------------------------------------------------------------
    wire fb_pos_closed = (sense_fb_pos > FB_CLOSED_TH);
    wire fb_neg_closed = (sense_fb_neg > FB_CLOSED_TH);

    wire [15:0] dc_i_mag = (sense_dc_i > ZERO_CURRENT)
                         ? (sense_dc_i - ZERO_CURRENT)
                         : (ZERO_CURRENT - sense_dc_i);

    wire current_near_zero = (dc_i_mag < I_SAFE_MAG);
    wire link_bled_down    = (sense_bus_v < V_TOUCH_SAFE);
    wire precharge_reached = (sense_pre_v >= precharge_target);

    //=========================================================================
    // FSM
    //=========================================================================
    reg [2:0]  state;
    reg [24:0] tmr;
    reg        pos_cmd, neg_cmd, pre_cmd, dis_cmd;

    assign hv_state = state;
    assign hv_on    = (state == `HV_ON);

    //-------------------------------------------------------------------------
    // THE OVERRIDE.  Combinational.  No FSM in the crash disconnect path.
    //-------------------------------------------------------------------------
    assign hv_contactor_pos_en = pos_cmd & ~crash_force_open;
    assign hv_contactor_neg_en = neg_cmd & ~crash_force_open;
    assign hv_precharge_en     = pre_cmd & ~crash_force_open;
    assign hv_discharge_en     = dis_cmd | crash_force_open;

    //-------------------------------------------------------------------------
    // WELD DETECTION
    //
    // The aux contacts must agree with what the coils are being told.  A
    // mismatch that survives 100 ms is a contactor that is not doing what it
    // is told - almost always welded shut.  Real contactors move in 20 to
    // 50 ms, so 100 ms clears normal transitions without a state qualifier.
    //
    // This is the fault that must never be cleared by anything but a
    // technician, because the vehicle looks safe and is not.
    //-------------------------------------------------------------------------
    wire fb_mismatch = (pos_cmd != fb_pos_closed) | (neg_cmd != fb_neg_closed);

    reg [24:0] weld_cnt;
    reg        weld_fault;

    always @(posedge clk_aon or negedge rst_hvsafe_n) begin
        if (!rst_hvsafe_n) begin
            weld_cnt   <= 25'd0;
            weld_fault <= 1'b0;
        end else if (service_clear && !fb_mismatch) begin
            weld_cnt   <= 25'd0;
            weld_fault <= 1'b0;
        end else if (fb_mismatch) begin
            if (weld_cnt >= T_CONTACTOR) weld_fault <= 1'b1;
            else                         weld_cnt   <= weld_cnt + 25'd1;
        end else begin
            weld_cnt <= 25'd0;
        end
    end

    //-------------------------------------------------------------------------
    // May HV close at all?
    //-------------------------------------------------------------------------
    wire permit_close = hv_request & hvil_ok & iso_ok &
                        ~crash_force_open & ~weld_fault;

    always @(posedge clk_aon or negedge rst_hvsafe_n) begin
        if (!rst_hvsafe_n) begin
            state          <= `HV_OFF;
            tmr            <= 25'd0;
            pos_cmd        <= 1'b0;
            neg_cmd        <= 1'b0;
            pre_cmd        <= 1'b0;
            dis_cmd        <= 1'b0;
            torque_inhibit <= 1'b1;
            hv_isolated    <= 1'b0;
            hv_fault_code  <= `HVF_NONE;
        end else begin

            // isolation is a statement about the physical vehicle, evaluated
            // every cycle regardless of state
            hv_isolated <= ~pos_cmd & ~neg_cmd &
                           ~fb_pos_closed & ~fb_neg_closed &
                           link_bled_down;

            // any of these forces the safe path from any state
            if (crash_force_open) begin
                pos_cmd        <= 1'b0;
                neg_cmd        <= 1'b0;
                pre_cmd        <= 1'b0;
                dis_cmd        <= 1'b1;
                torque_inhibit <= 1'b1;
                if (state != `HV_FAULT) begin
                    hv_fault_code <= `HVF_CRASH;
                    state         <= `HV_DISCHARGE;
                    tmr           <= 25'd0;
                end
            end else begin

                case (state)

                //-------------------------------------------------------------
                // OFF - everything open, link bled, waiting for a request
                //-------------------------------------------------------------
                `HV_OFF: begin
                    pos_cmd        <= 1'b0;
                    neg_cmd        <= 1'b0;
                    pre_cmd        <= 1'b0;
                    dis_cmd        <= 1'b0;
                    torque_inhibit <= 1'b1;
                    tmr            <= 25'd0;

                    if (weld_fault) begin
                        hv_fault_code <= `HVF_CONTACTOR_WELD;
                        state         <= `HV_FAULT;
                    end else if (permit_close) begin
                        state <= `HV_PRECHARGE;
                    end
                end

                //-------------------------------------------------------------
                // PRECHARGE - negative contactor plus the precharge relay.
                // Charge the DC link through the resistor and watch it rise.
                //-------------------------------------------------------------
                `HV_PRECHARGE: begin
                    neg_cmd        <= 1'b1;
                    pre_cmd        <= 1'b1;
                    pos_cmd        <= 1'b0;
                    dis_cmd        <= 1'b0;
                    torque_inhibit <= 1'b1;

                    if (!permit_close) begin
                        state <= `HV_DISCHARGE;
                        tmr   <= 25'd0;
                    end else if (sense_valid && precharge_reached) begin
                        state <= `HV_CLOSING;
                        tmr   <= 25'd0;
                    end else if (tmr >= T_PRECHARGE) begin
                        // the link is not rising - something downstream is
                        // shorted.  Latched, and no retry.
                        hv_fault_code <= `HVF_PRECHARGE_TO;
                        state         <= `HV_DISCHARGE;
                        tmr           <= 25'd0;
                    end else begin
                        tmr <= tmr + 25'd1;
                    end
                end

                //-------------------------------------------------------------
                // CLOSING - main positive contactor in, precharge relay out
                //-------------------------------------------------------------
                `HV_CLOSING: begin
                    pos_cmd        <= 1'b1;
                    neg_cmd        <= 1'b1;
                    pre_cmd        <= 1'b0;
                    torque_inhibit <= 1'b1;

                    if (!permit_close) begin
                        state <= `HV_OPENING;
                        tmr   <= 25'd0;
                    end else if (sense_valid && fb_pos_closed && fb_neg_closed) begin
                        state <= `HV_ON;
                        tmr   <= 25'd0;
                    end else if (tmr >= T_CONTACTOR) begin
                        hv_fault_code <= `HVF_CONTACTOR_WELD;
                        state         <= `HV_OPENING;
                        tmr           <= 25'd0;
                    end else begin
                        tmr <= tmr + 25'd1;
                    end
                end

                //-------------------------------------------------------------
                // ON - the only state where torque is permitted
                //-------------------------------------------------------------
                `HV_ON: begin
                    pos_cmd        <= 1'b1;
                    neg_cmd        <= 1'b1;
                    pre_cmd        <= 1'b0;
                    dis_cmd        <= 1'b0;
                    torque_inhibit <= 1'b0;
                    tmr            <= 25'd0;

                    if (weld_fault) begin
                        hv_fault_code <= `HVF_CONTACTOR_WELD;
                        state         <= `HV_OPENING;
                    end else if (!hvil_ok) begin
                        // a connector came apart with the pack live
                        hv_fault_code <= `HVF_HVIL_OPEN;
                        state         <= `HV_OPENING;
                    end else if (!iso_ok) begin
                        hv_fault_code <= `HVF_ISOLATION;
                        state         <= `HV_OPENING;
                    end else if (!hv_request) begin
                        state <= `HV_OPENING;      // ordinary key-off
                    end
                end

                //-------------------------------------------------------------
                // OPENING - ramp torque to zero BEFORE dropping the coils.
                // Opening under load welds contacts.
                //-------------------------------------------------------------
                `HV_OPENING: begin
                    torque_inhibit <= 1'b1;
                    pre_cmd        <= 1'b0;

                    if (sense_valid && current_near_zero) begin
                        pos_cmd <= 1'b0;
                        neg_cmd <= 1'b0;
                        state   <= `HV_DISCHARGE;
                        tmr     <= 25'd0;
                    end else if (tmr >= T_RAMP) begin
                        // the current is not falling.  Opening under load risks
                        // a weld, but staying closed with a fault present is
                        // worse.  Open, and record why.
                        pos_cmd       <= 1'b0;
                        neg_cmd       <= 1'b0;
                        if (hv_fault_code == `HVF_NONE)
                            hv_fault_code <= `HVF_OVERCURRENT;
                        state         <= `HV_DISCHARGE;
                        tmr           <= 25'd0;
                    end else begin
                        tmr <= tmr + 25'd1;
                    end
                end

                //-------------------------------------------------------------
                // DISCHARGE - bleed the DC link to a touch-safe voltage and
                // prove it got there
                //-------------------------------------------------------------
                `HV_DISCHARGE: begin
                    pos_cmd        <= 1'b0;
                    neg_cmd        <= 1'b0;
                    pre_cmd        <= 1'b0;
                    dis_cmd        <= 1'b1;
                    torque_inhibit <= 1'b1;

                    if (sense_valid && link_bled_down) begin
                        dis_cmd <= 1'b0;
                        tmr     <= 25'd0;
                        state   <= (hv_fault_code == `HVF_NONE) ? `HV_OFF
                                                                : `HV_FAULT;
                    end else if (tmr >= T_DISCHARGE) begin
                        // the bleed resistor is open circuit.  THE DC LINK MAY
                        // STILL BE LIVE.  This gets its own fault code because
                        // it is the one condition where the vehicle looks
                        // isolated and is not.
                        hv_fault_code <= `HVF_DISCHARGE_TO;
                        state         <= `HV_FAULT;
                        tmr           <= 25'd0;
                    end else begin
                        tmr <= tmr + 25'd1;
                    end
                end

                //-------------------------------------------------------------
                // FAULT - terminal until an authenticated service clear.
                // There is no automatic recovery from any of these.
                //-------------------------------------------------------------
                `HV_FAULT: begin
                    pos_cmd        <= 1'b0;
                    neg_cmd        <= 1'b0;
                    pre_cmd        <= 1'b0;
                    dis_cmd        <= 1'b0;
                    torque_inhibit <= 1'b1;

                    if (service_clear && !weld_fault && hvil_ok && iso_ok) begin
                        hv_fault_code <= `HVF_NONE;
                        state         <= `HV_OFF;
                    end
                end

                default: begin
                    state <= `HV_OFF;
                end

                endcase
            end
        end
    end

endmodule

`default_nettype wire
