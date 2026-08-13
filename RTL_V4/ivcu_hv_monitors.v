//=============================================================================
// ivcu_hv_monitors.v  -  interlock loop and insulation monitoring
//
// TWO MEASUREMENTS THAT DECIDE WHETHER 400 VOLTS MAY EXIST
//
// HVIL - the high voltage interlock loop.  A thin wire that runs through every
// HV connector in the vehicle and returns to this chip.  Unplug any connector,
// or cut any HV cable in a crash, and the loop opens.  It is the cheapest and
// most reliable way to know that someone or something has opened the high
// voltage system.
//
// ISOLATION RESISTANCE - the insulation between the pack and the chassis.
// This is the measurement that actually detects a compromised pack after an
// impact, when the HVIL loop may still be intact but the insulation is not.
// V3 had neither of these.
//
//-----------------------------------------------------------------------------
// FAIL DIRECTIONS, CHOSEN DELIBERATELY
//
// HVIL is read twice: as a direct pin and as an ADC channel.  They are ORed in
// the unsafe direction - if EITHER says the loop is open, it is treated as
// open.  Requiring both to agree would mean one failed reading could hide a
// disconnected HV connector.
//
// If the HVIL ADC channel stops answering, the direct pin carries the
// decision alone.  That is a real degradation and it is reported, but it is
// better than declaring the loop open and stranding a vehicle on a dead ADC
// channel when the physical interlock is demonstrably fine.
//
// Isolation fails the other way.  If the isolation channel stops answering,
// iso_ok goes low and HV is not permitted to close.  There is no second
// source for insulation resistance, and closing 400 V onto a pack whose
// insulation state is unknown is not a risk worth taking to save a tow.  This
// is consistent with the channel's SAFETY_CRITICAL classification.
//
//-----------------------------------------------------------------------------
// DEBOUNCE
// Ten milliseconds.  An HV connector on a vehicle crossing a pothole chatters;
// without debounce the vehicle would shut down its own drivetrain on a rough
// road.  Ten milliseconds is long enough to reject that and far shorter than
// any plausible human interaction with a connector.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_hv_monitors (
    input  wire        clk_aon,
    input  wire        rst_hvsafe_n,

    //--- HVIL, two independent sources --------------------------------------
    input  wire        hvil_raw_s,     // direct pin, through ivcu_cdc_bit_sync
    input  wire [15:0] sense_hvil,     // ADC channel 0
    input  wire        hvil_dead,      // channel 0 is not answering

    //--- insulation ----------------------------------------------------------
    input  wire [15:0] sense_iso,      // ADC channel 1
    input  wire        iso_dead,       // channel 1 is not answering

    input  wire        sense_valid,    // a new snapshot arrived

    //--- verdicts ------------------------------------------------------------
    output reg         hvil_ok,        // loop confirmed closed
    output reg         hvil_degraded,  // running on the pin alone
    output reg         iso_ok,         // insulation adequate to close HV
    output reg         iso_warn,       // degraded, keep running, tell the driver
    output reg         iso_fault       // insulation breakdown
);

    //-------------------------------------------------------------------------
    // RULE R1 - every threshold and timeout is a constant.
    //-------------------------------------------------------------------------
    localparam [16:0] DEBOUNCE_CYC = `HVIL_DEBOUNCE_CYC;   // 10 ms at 10 MHz
    localparam [15:0] DISC_HI      = 16'd3595;             // switch closed

    //=========================================================================
    // HVIL
    //
    // The ADC reading is a discrete: near the top rail means the loop conducts.
    //=========================================================================
    wire hvil_from_adc = (sense_hvil > DISC_HI);

    // closed only if the pin says closed AND (the ADC agrees OR the ADC is
    // dead and therefore has no opinion)
    wire hvil_closed_now = hvil_raw_s & (hvil_from_adc | hvil_dead);

    reg [16:0] hvil_cnt;

    always @(posedge clk_aon or negedge rst_hvsafe_n) begin
        if (!rst_hvsafe_n) begin
            hvil_cnt      <= 17'd0;
            hvil_ok       <= 1'b0;      // safe default: assume open
            hvil_degraded <= 1'b0;
        end else begin
            hvil_degraded <= hvil_dead;

            if (hvil_closed_now != hvil_ok) begin
                // the input disagrees with the current verdict - start counting
                if (hvil_cnt >= DEBOUNCE_CYC) begin
                    hvil_ok  <= hvil_closed_now;
                    hvil_cnt <= 17'd0;
                end else begin
                    hvil_cnt <= hvil_cnt + 17'd1;
                end
            end else begin
                hvil_cnt <= 17'd0;
            end
        end
    end

    //=========================================================================
    // ISOLATION RESISTANCE
    //
    //   >= 500 ohm/V   normal
    //   100..500       warn, latch a message, keep driving
    //   <  100 ohm/V   fault, HV may not close; if already on, shut down under
    //                  control (ivcu_hv_contactor_ctrl ramps torque first)
    //=========================================================================
    wire iso_below_warn  = (sense_iso < `ISO_WARN_CNT);
    wire iso_below_fault = (sense_iso < `ISO_FAULT_CNT);

    reg [16:0] iso_warn_cnt;
    reg [16:0] iso_flt_cnt;

    always @(posedge clk_aon or negedge rst_hvsafe_n) begin
        if (!rst_hvsafe_n) begin
            iso_warn_cnt <= 17'd0;
            iso_flt_cnt  <= 17'd0;
            iso_warn     <= 1'b0;
            iso_fault    <= 1'b0;
            iso_ok       <= 1'b0;       // safe default: not proven good yet
        end else begin

            // no measurement means no permission.  There is no second source
            // for insulation resistance.
            if (iso_dead) begin
                iso_ok       <= 1'b0;
                iso_fault    <= 1'b1;
                iso_warn     <= 1'b1;
                iso_warn_cnt <= 17'd0;
                iso_flt_cnt  <= 17'd0;
            end else begin

                if (sense_valid) begin
                    // --- warning band ---------------------------------------
                    if (iso_below_warn) begin
                        if (iso_warn_cnt >= DEBOUNCE_CYC) iso_warn <= 1'b1;
                        else iso_warn_cnt <= iso_warn_cnt + 17'd1;
                    end else begin
                        iso_warn_cnt <= 17'd0;
                        iso_warn     <= 1'b0;
                    end

                    // --- fault band -----------------------------------------
                    if (iso_below_fault) begin
                        if (iso_flt_cnt >= DEBOUNCE_CYC) iso_fault <= 1'b1;
                        else iso_flt_cnt <= iso_flt_cnt + 17'd1;
                    end else begin
                        iso_flt_cnt <= 17'd0;
                        iso_fault   <= 1'b0;
                    end
                end

                iso_ok <= ~iso_fault & ~iso_below_fault;
            end
        end
    end

endmodule

`default_nettype wire
