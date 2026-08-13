//=============================================================================
// ivcu_service_guidance.v  -  telling the human what to do
//
// This block replaces V3's diagnostic_report_generator, and the reason it
// looks nothing like it is the point.
//
//-----------------------------------------------------------------------------
// WHAT V3 DID, AND WHY IT WAS AN ARCHITECTURAL MISTAKE RATHER THAN A BUG
//
//     output reg [127:0] maintenance_report;
//     output reg  [63:0] battery_life_prediction;
//     output reg  [63:0] motor_life_prediction;
//     output reg  [95:0] component_wear_level;
//
// 386 output bits, instantiated with none of them connected.  118,908 um2,
// 18.8 % of the standard cell area, for text and predictions nothing could
// read.
//
// Connecting them would not have fixed it.  Human-readable text does not
// belong on a die.  It cannot be translated, corrected, or improved after
// tapeout; it costs area proportional to how helpful it is; and it duplicates
// something the MCU already does well.
//
// So this block emits a 16-bit CODE.  Firmware renders the words, the
// pictures, the translations and the step-by-step video.  The chip supplies
// the one thing only the chip knows: which sensor, how it failed, and who is
// allowed to touch it.
//
//-----------------------------------------------------------------------------
// THE HIGH VOLTAGE RULE IS ENFORCED IN HARDWARE
//
// GA_HV_DANGER comes from the HV_HAZARD bit in SENSOR_ATTR_TABLE, not from a
// firmware lookup table.  There is no firmware defect that can display
// "unscrew this panel" for a sensor sitting on a 400 volt bus, because the
// action code for those channels is decided by a synthesis-time constant.
//
//-----------------------------------------------------------------------------
// ONE ISSUE AT A TIME, WORST FIRST
//
// A rider standing at the side of a road does not need a list.  They need the
// single most important thing wrong and what to do about it.  The scan walks
// all 64 channels and reports the highest-priority unresolved issue; when that
// one is fixed, the next appears.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_service_guidance (
    input  wire         clk_mcu,
    input  wire         rst_mcu_n,

    //--- verdicts, synchronised into clk_mcu --------------------------------
    input  wire [191:0] sensor_status_flat,
    input  wire [63:0]  sensor_enable_s,

    //--- serviceability --------------------------------------------------------
    input  wire [15:0]  permit_state_flat,   // 8 x 2 bits
    input  wire [23:0]  permit_starts_flat,  // 8 x 3 bits

    //--- HV ---------------------------------------------------------------------
    input  wire [3:0]   hv_fault_code,
    input  wire         update_req,

    //--- display serial link, top-level pins -----------------------------------
    output reg          disp_sclk,
    output reg          disp_sdata,
    output reg          disp_cs,

    //--- the same information for the register map -----------------------------
    output reg  [5:0]   guidance_sensor_id,
    output reg  [2:0]   guidance_status,
    output reg  [3:0]   guidance_action,
    output reg  [2:0]   guidance_starts_left,
    output reg          guidance_valid
);

    localparam [255:0] ATTR_ROM = `SENSOR_ATTR_TABLE;

    //=========================================================================
    // Which of the eight permit machines owns a channel, if any
    //=========================================================================
    function [3:0] permit_slot;      // 4'd8 means "not a permit channel"
        input [5:0] c;
        begin
            case (c)
                `S_GEAR_POSITION : permit_slot = 4'd0;
                `S_AMBIENT_TEMP  : permit_slot = 4'd1;
                `S_HUMIDITY      : permit_slot = 4'd2;
                `S_RIDE_HEIGHT   : permit_slot = 4'd3;
                `S_SIDE_STAND    : permit_slot = 4'd4;
                `S_SEAT_OCCUPANCY: permit_slot = 4'd5;
                `S_TPMS_FRONT    : permit_slot = 4'd6;
                `S_TPMS_REAR     : permit_slot = 4'd7;
                default          : permit_slot = 4'd8;
            endcase
        end
    endfunction

    //=========================================================================
    // How urgent is this channel's problem.  0 means nothing to report.
    //=========================================================================
    function [2:0] priority_of;
        input [2:0] status;
        input [3:0] attr;         // {hv_hazard, servicer, class[1:0]}
        input       live;
        begin
            if (!live || (status == `SS_OK) || (status == `SS_DISABLED_MODE))
                priority_of = 3'd0;
            else if (attr[3])                              // HV hazard
                priority_of = 3'd7;
            else if (attr[1:0] == `CLASS_CRITICAL)
                priority_of = 3'd6;
            else if (attr[1:0] == `CLASS_CONDITIONAL)
                priority_of = 3'd5;
            else if (attr[1:0] == `CLASS_DEGRADE)
                priority_of = 3'd4;
            else if (status == `SS_DEGRADED)
                priority_of = 3'd1;                        // barely worth saying
            else
                priority_of = 3'd2;                        // comfort, failed
        end
    endfunction

    //=========================================================================
    // What should the human do about it
    //
    // The user / mechanic split: a CONDITIONAL channel that a field technician
    // may replace is an owner job - these are the side stand, TPMS, seat and
    // similar, all accessible without opening anything dangerous.  A COMFORT
    // channel that a field technician may replace is a mechanic job - cameras
    // and parking sensors need calibration after replacement.
    //=========================================================================
    function [3:0] action_of;
        input [2:0] status;
        input [3:0] attr;
        input [1:0] pstate;
        begin
            if (attr[3])                          action_of = `GA_HV_DANGER;
            else if (status == `SS_BYPASSED)      action_of = `GA_PERMIT_OFFER;
            else if ((attr[1:0] == `CLASS_CONDITIONAL) &&
                     (pstate == `PS_INHIBITED))   action_of = `GA_PERMIT_OFFER;
            else if ((attr[1:0] == `CLASS_CONDITIONAL) &&
                     (pstate == `PS_EXPIRED))     action_of = `GA_PERMIT_EXPIRED;
            else if (attr[2])                     action_of = `GA_SERVICE_CENTRE;
            else if (attr[1:0] == `CLASS_CONDITIONAL) action_of = `GA_USER_REPLACE;
            else if (attr[1:0] == `CLASS_COMFORT) action_of = `GA_MECHANIC;
            else                                  action_of = `GA_SERVICE_CENTRE;
        end
    endfunction

    //=========================================================================
    // SCAN - one channel per clock, keeping the worst
    //=========================================================================
    localparam [1:0] ST_IDLE = 2'd0,
                     ST_SCAN = 2'd1,
                     ST_SHIFT= 2'd2;

    reg [1:0] state;
    reg [6:0] idx;
    reg [2:0] best_pri;
    reg [5:0] best_id;
    reg [2:0] best_status;
    reg [3:0] best_action;
    reg [2:0] best_starts;

    // shift-adds instead of x3 - see the note in ivcu_health_scorer.
    // x4 and x2 are shifts and fold away on their own.
    wire [7:0] stat_off  = {1'b0, idx[5:0], 1'b0} + {2'b00, idx[5:0]};
    wire [4:0] slot_off3 = {1'b0, cur_slot[2:0], 1'b0} + {2'b00, cur_slot[2:0]};

    wire [2:0] cur_status = sensor_status_flat[stat_off +: 3];
    wire [3:0] cur_attr   = ATTR_ROM[idx[5:0]*4 +: 4];
    wire       cur_live   = sensor_enable_s[idx[5:0]];
    wire [3:0] cur_slot   = permit_slot(idx[5:0]);

    wire [1:0] cur_pstate = (cur_slot < 4'd8)
                          ? permit_state_flat[cur_slot[2:0]*2 +: 2]
                          : `PS_NORMAL;

    wire [2:0] cur_starts = (cur_slot < 4'd8)
                          ? permit_starts_flat[slot_off3 +: 3]
                          : 3'd0;

    wire [2:0] cur_pri = priority_of(cur_status, cur_attr, cur_live);

    //=========================================================================
    // Serial shift out.  16 bits, MSB first, clk_mcu / 4.
    //=========================================================================
    reg [15:0] shreg;
    reg [4:0]  bitcnt;
    reg [1:0]  phase;

    always @(posedge clk_mcu or negedge rst_mcu_n) begin
        if (!rst_mcu_n) begin
            state                <= ST_IDLE;
            idx                  <= 7'd0;
            best_pri             <= 3'd0;
            best_id              <= 6'd0;
            best_status          <= `SS_OK;
            best_action          <= `GA_NONE;
            best_starts          <= 3'd0;
            guidance_sensor_id   <= 6'd0;
            guidance_status      <= `SS_OK;
            guidance_action      <= `GA_NONE;
            guidance_starts_left <= 3'd0;
            guidance_valid       <= 1'b0;
            shreg                <= 16'd0;
            bitcnt               <= 5'd0;
            phase                <= 2'd0;
            disp_sclk            <= 1'b0;
            disp_sdata           <= 1'b0;
            disp_cs              <= 1'b1;      // idle high
        end else begin
            case (state)

                ST_IDLE: begin
                    disp_cs   <= 1'b1;
                    disp_sclk <= 1'b0;
                    if (update_req) begin
                        idx      <= 7'd0;
                        best_pri <= 3'd0;
                        state    <= ST_SCAN;
                    end
                end

                //-------------------------------------------------------------
                // SCAN - 64 clocks.  Strictly greater, so the lowest channel
                // index wins a tie: HV channels are 0 to 5 and therefore
                // always reported before anything else of equal priority.
                //-------------------------------------------------------------
                ST_SCAN: begin
                    if (cur_pri > best_pri) begin
                        best_pri    <= cur_pri;
                        best_id     <= idx[5:0];
                        best_status <= cur_status;
                        best_action <= action_of(cur_status, cur_attr, cur_pstate);
                        best_starts <= cur_starts;
                    end

                    if (idx == 7'd63) begin
                        //---------------------------------------------------
                        // An HV fault outranks any sensor issue: the vehicle
                        // may look fine and be lethal to open.
                        //---------------------------------------------------
                        if (hv_fault_code != `HVF_NONE) begin
                            guidance_sensor_id   <= `S_HVIL_LOOP;
                            guidance_status      <= `SS_OUT_OF_RANGE;
                            guidance_action      <= `GA_HV_DANGER;
                            guidance_starts_left <= 3'd0;
                            guidance_valid       <= 1'b1;
                            shreg <= {`S_HVIL_LOOP, `SS_OUT_OF_RANGE,
                                      `GA_HV_DANGER, 3'd0};
                        end else if (best_pri != 3'd0) begin
                            guidance_sensor_id   <= best_id;
                            guidance_status      <= best_status;
                            guidance_action      <= best_action;
                            guidance_starts_left <= best_starts;
                            guidance_valid       <= 1'b1;
                            shreg <= {best_id, best_status, best_action,
                                      best_starts};
                        end else begin
                            guidance_sensor_id   <= 6'd0;
                            guidance_status      <= `SS_OK;
                            guidance_action      <= `GA_NONE;
                            guidance_starts_left <= 3'd0;
                            guidance_valid       <= 1'b0;
                            shreg <= {6'd0, `SS_OK, `GA_NONE, 3'd0};
                        end

                        bitcnt  <= 5'd0;
                        phase   <= 2'd0;
                        disp_cs <= 1'b0;
                        state   <= ST_SHIFT;
                    end else begin
                        idx <= idx + 7'd1;
                    end
                end

                //-------------------------------------------------------------
                // SHIFT - 16 bits at clk_mcu / 4
                //-------------------------------------------------------------
                ST_SHIFT: begin
                    phase <= phase + 2'd1;
                    case (phase)
                        2'd0: begin
                            disp_sdata <= shreg[15];
                            disp_sclk  <= 1'b0;
                        end
                        2'd1: disp_sclk <= 1'b0;
                        2'd2: disp_sclk <= 1'b1;      // sampled on the rise
                        default: begin
                            disp_sclk <= 1'b0;
                            shreg     <= {shreg[14:0], 1'b0};
                            if (bitcnt == 5'd15) begin
                                disp_cs <= 1'b1;
                                state   <= ST_IDLE;
                            end else begin
                                bitcnt <= bitcnt + 5'd1;
                            end
                        end
                    endcase
                end

                default: state <= ST_IDLE;

            endcase
        end
    end

endmodule

`default_nettype wire
