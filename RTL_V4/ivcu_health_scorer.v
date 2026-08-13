//=============================================================================
// ivcu_health_scorer.v  -  one scoring engine, seven domains, no divider
//
//-----------------------------------------------------------------------------
// WHAT THE NUMBER MEANS, STATED HONESTLY
//
// 255 means nothing is wrong.  Every fault subtracts an amount proportional to
// how bad the fault is and how much the sensor matters.  Zero means the domain
// is in no state to be trusted.
//
// It is NOT a percentage of working sensors.  Calling it a percentage would be
// a lie, because a single dead brake-pressure sensor should move the number
// further than four dead cabin-temperature sensors, and a percentage cannot do
// that.  Whatever the display calls this, it must not call it "% healthy".
//
//     penalty(channel) = severity(status) << weight_shift(class)
//     score(domain)    = 255 - sum(penalty), saturated at 0
//
//-----------------------------------------------------------------------------
// WHY THERE IS NO DIVIDER
//
// The obvious formula is score = 255 * good_weight / total_weight.  That needs
// a divide, and a divide is what put seq_divider.v on V3's critical path and
// produced the STA divider problem.  RULE R5 forbids '/' anywhere in this
// design, so the arithmetic was chosen to not need one.
//
// The weights are 8, 4, 2, 1 - powers of two - so multiplying a severity by a
// weight is a shift by 3, 2, 1 or 0.  No multiplier either.
//
//-----------------------------------------------------------------------------
// WHY IT WALKS THE CHANNELS INSTEAD OF ADDING THEM ALL AT ONCE
//
// Summing 64 penalty terms combinationally is a wide adder tree, and it would
// sit on the clk_ai critical path for a result that only changes once per
// 10 us sensor sweep.  Instead one channel is accumulated per clock: 64 clocks
// at 50 MHz is 1.28 us, comfortably inside the sweep that triggered it.
//
// One adder, one counter, eight accumulators.  A fraction of the area of the
// tree, and it cannot fail timing.
//
//-----------------------------------------------------------------------------
// CHANNELS DISABLED BY MODE SCORE NOTHING AT ALL
//
// SS_DISABLED_MODE contributes zero penalty.  A motorcycle is not less healthy
// than a car for lacking a radiator.  In V3 there was no distinction between
// "not fitted" and "not working", so a bike would have scored itself down for
// every car sensor it did not have.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_health_scorer (
    input  wire         clk_ai,
    input  wire         rst_ai_n,

    //--- synchronised into clk_ai by the top-level CDC bank -----------------
    input  wire [191:0] sensor_status_flat,
    input  wire         update_req,        // one pulse per completed sweep

    //--- per-domain results ---------------------------------------------------
    output reg  [7:0]   score_battery,
    output reg  [7:0]   score_motor,
    output reg  [7:0]   score_thermal,
    output reg  [7:0]   score_dynamics,
    output reg  [7:0]   score_driver,
    output reg  [7:0]   score_safety,
    output reg  [7:0]   score_perception,
    output reg  [7:0]   system_health_score,

    output reg  [3:0]   status_battery,
    output reg  [3:0]   status_motor,
    output reg  [3:0]   status_thermal,
    output reg  [3:0]   status_dynamics,
    output reg  [3:0]   status_driver,
    output reg  [3:0]   status_safety,
    output reg  [3:0]   status_perception,

    output reg          score_valid        // one pulse when a pass completes
);

    localparam [255:0] ATTR_ROM = `SENSOR_ATTR_TABLE;

    localparam [2:0] D_BATTERY    = 3'd0,
                     D_MOTOR      = 3'd1,
                     D_THERMAL    = 3'd2,
                     D_DYNAMICS   = 3'd3,
                     D_DRIVER     = 3'd4,
                     D_SAFETY     = 3'd5,   // HV island + crash channels
                     D_PERCEPTION = 3'd6;   // ADAS + cabin + tyres

    //-------------------------------------------------------------------------
    // Which domain owns each channel.  Every one of the 64 belongs to exactly
    // one domain - in V3, channels 35 to 41 fed no decision logic at all.
    //-------------------------------------------------------------------------
    function [2:0] domain_of;
        input [5:0] c;
        begin
            if      (c <= 6'd5 )              domain_of = D_SAFETY;      //  0-5  HV
            else if (c <= 6'd17)              domain_of = D_BATTERY;     //  6-17
            else if (c <= 6'd25)              domain_of = D_MOTOR;       // 18-25
            else if (c <= 6'd31)              domain_of = D_THERMAL;     // 26-31
            else if (c <= 6'd43)              domain_of = D_DYNAMICS;    // 32-43
            else if (c <= 6'd49)              domain_of = D_DRIVER;      // 44-49
            else if (c <= 6'd52)              domain_of = D_SAFETY;      // 50-52
            else                              domain_of = D_PERCEPTION;  // 53-63
        end
    endfunction

    //-------------------------------------------------------------------------
    // How bad is each status.  DISABLED_BY_MODE is zero - see the header.
    //-------------------------------------------------------------------------
    function [3:0] severity_of;
        input [2:0] s;
        begin
            case (s)
                `SS_OK            : severity_of = 4'd0;
                `SS_DEGRADED      : severity_of = 4'd1;
                `SS_BYPASSED      : severity_of = 4'd2;
                `SS_OUT_OF_RANGE  : severity_of = 4'd4;
                `SS_IMPLAUSIBLE   : severity_of = 4'd4;
                `SS_STUCK         : severity_of = 4'd6;
                `SS_NO_RESPONSE   : severity_of = 4'd8;
                default           : severity_of = 4'd0;   // SS_DISABLED_MODE
            endcase
        end
    endfunction

    //-------------------------------------------------------------------------
    // Weight as a shift amount.  8,4,2,1 -> 3,2,1,0.  No multiplier.
    //-------------------------------------------------------------------------
    function [1:0] weight_shift;
        input [1:0] cls;
        begin
            case (cls)
                `CLASS_CRITICAL   : weight_shift = 2'd3;   // x8
                `CLASS_DEGRADE    : weight_shift = 2'd2;   // x4
                `CLASS_CONDITIONAL: weight_shift = 2'd1;   // x2
                default           : weight_shift = 2'd0;   // x1  comfort
            endcase
        end
    endfunction

    //=========================================================================
    // The walk
    //=========================================================================
    localparam [1:0] ST_IDLE  = 2'd0,
                     ST_WALK  = 2'd1,
                     ST_LATCH = 2'd2;

    reg [1:0]  state;
    reg [6:0]  idx;              // 0..64, 7 bits so 64 is representable
    reg [8:0]  acc [0:6];        // per-domain penalty, 9 bits to see overflow
    reg [8:0]  acc_sys;          // whole-vehicle penalty

    // Bit offset of channel idx in the 3-bit-per-channel status bus.
    //
    // Written as a shift-add rather than idx*3.  A variable times a
    // non-power-of-two infers a $mul; x*3 is x*2 + x, which is one adder.
    // ABC would probably reach the same result, but "probably" is how V3
    // ended up with 42 general multipliers it did not want, and an explicit
    // adder costs nothing to write.
    //
    // idx is 6 bits, so the offset reaches 63*3 = 189 and needs 8.
    wire [7:0] stat_off = {1'b0, idx[5:0], 1'b0} + {2'b00, idx[5:0]};

    wire [2:0] cur_status = sensor_status_flat[stat_off +: 3];
    wire [1:0] cur_class  = ATTR_ROM[idx[5:0]*4 +: 2];   // x4 folds to a shift
    wire [2:0] cur_domain = domain_of(idx[5:0]);
    wire [3:0] cur_sever  = severity_of(cur_status);
    wire [1:0] cur_shift  = weight_shift(cur_class);

    // max value is 8 << 3 = 64, so 8 bits is ample
    wire [7:0] contrib = {4'd0, cur_sever} << cur_shift;

    // saturating adds
    wire [8:0] acc_next = (acc[cur_domain] + {1'b0, contrib} > 9'd255)
                        ? 9'd255
                        : acc[cur_domain] + {1'b0, contrib};

    wire [8:0] sys_next = (acc_sys + {1'b0, contrib} > 9'd255)
                        ? 9'd255
                        : acc_sys + {1'b0, contrib};

    //-------------------------------------------------------------------------
    // Score to status band
    //-------------------------------------------------------------------------
    function [3:0] band_of;
        input [7:0] sc;
        begin
            if      (sc >= 8'd240) band_of = `DS_OK;
            else if (sc >= 8'd180) band_of = `DS_WARNING;
            else if (sc >= 8'd100) band_of = `DS_CRITICAL;
            else if (sc >  8'd0  ) band_of = `DS_FAULT;
            else                   band_of = `DS_EMERGENCY;
        end
    endfunction

    integer d;

    always @(posedge clk_ai or negedge rst_ai_n) begin
        if (!rst_ai_n) begin
            state               <= ST_IDLE;
            idx                 <= 7'd0;
            acc_sys             <= 9'd0;
            score_valid         <= 1'b0;
            score_battery       <= 8'd255;
            score_motor         <= 8'd255;
            score_thermal       <= 8'd255;
            score_dynamics      <= 8'd255;
            score_driver        <= 8'd255;
            score_safety        <= 8'd255;
            score_perception    <= 8'd255;
            system_health_score <= 8'd255;
            status_battery      <= `DS_OK;
            status_motor        <= `DS_OK;
            status_thermal      <= `DS_OK;
            status_dynamics     <= `DS_OK;
            status_driver       <= `DS_OK;
            status_safety       <= `DS_OK;
            status_perception   <= `DS_OK;
            for (d = 0; d < 7; d = d + 1) acc[d] <= 9'd0;
        end else begin
            score_valid <= 1'b0;

            case (state)

                ST_IDLE: begin
                    if (update_req) begin
                        idx     <= 7'd0;
                        acc_sys <= 9'd0;
                        for (d = 0; d < 7; d = d + 1) acc[d] <= 9'd0;
                        state   <= ST_WALK;
                    end
                end

                //-------------------------------------------------------------
                // one channel per clock, 64 clocks, 1.28 us at 50 MHz
                //-------------------------------------------------------------
                ST_WALK: begin
                    acc[cur_domain] <= acc_next;
                    acc_sys         <= sys_next;

                    if (idx == 7'd63) begin
                        state <= ST_LATCH;
                    end else begin
                        idx <= idx + 7'd1;
                    end
                end

                //-------------------------------------------------------------
                // 255 minus the penalty, floored at zero
                //-------------------------------------------------------------
                ST_LATCH: begin
                    score_battery    <= (acc[D_BATTERY]    >= 9'd255) ? 8'd0
                                      : (8'd255 - acc[D_BATTERY][7:0]);
                    score_motor      <= (acc[D_MOTOR]      >= 9'd255) ? 8'd0
                                      : (8'd255 - acc[D_MOTOR][7:0]);
                    score_thermal    <= (acc[D_THERMAL]    >= 9'd255) ? 8'd0
                                      : (8'd255 - acc[D_THERMAL][7:0]);
                    score_dynamics   <= (acc[D_DYNAMICS]   >= 9'd255) ? 8'd0
                                      : (8'd255 - acc[D_DYNAMICS][7:0]);
                    score_driver     <= (acc[D_DRIVER]     >= 9'd255) ? 8'd0
                                      : (8'd255 - acc[D_DRIVER][7:0]);
                    score_safety     <= (acc[D_SAFETY]     >= 9'd255) ? 8'd0
                                      : (8'd255 - acc[D_SAFETY][7:0]);
                    score_perception <= (acc[D_PERCEPTION] >= 9'd255) ? 8'd0
                                      : (8'd255 - acc[D_PERCEPTION][7:0]);

                    system_health_score <= (acc_sys >= 9'd255) ? 8'd0
                                         : (8'd255 - acc_sys[7:0]);

                    status_battery    <= band_of((acc[D_BATTERY]    >= 9'd255)
                                        ? 8'd0 : (8'd255 - acc[D_BATTERY][7:0]));
                    status_motor      <= band_of((acc[D_MOTOR]      >= 9'd255)
                                        ? 8'd0 : (8'd255 - acc[D_MOTOR][7:0]));
                    status_thermal    <= band_of((acc[D_THERMAL]    >= 9'd255)
                                        ? 8'd0 : (8'd255 - acc[D_THERMAL][7:0]));
                    status_dynamics   <= band_of((acc[D_DYNAMICS]   >= 9'd255)
                                        ? 8'd0 : (8'd255 - acc[D_DYNAMICS][7:0]));
                    status_driver     <= band_of((acc[D_DRIVER]     >= 9'd255)
                                        ? 8'd0 : (8'd255 - acc[D_DRIVER][7:0]));
                    status_safety     <= band_of((acc[D_SAFETY]     >= 9'd255)
                                        ? 8'd0 : (8'd255 - acc[D_SAFETY][7:0]));
                    status_perception <= band_of((acc[D_PERCEPTION] >= 9'd255)
                                        ? 8'd0 : (8'd255 - acc[D_PERCEPTION][7:0]));

                    score_valid <= 1'b1;
                    state       <= ST_IDLE;
                end

                default: state <= ST_IDLE;

            endcase
        end
    end

endmodule

`default_nettype wire
