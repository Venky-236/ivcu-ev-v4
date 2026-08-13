//=============================================================================
// ivcu_hv_pyro_trigger.v  -  pyrotechnic fuse arm and fire
//
// A pyro fuse is a small explosive charge that physically severs the main high
// voltage cable.  It is the last resort when the contactors cannot be trusted
// to have opened - a welded contactor after a severe impact leaves the pack
// live no matter what this chip commands.
//
// IT IS SINGLE USE.  Firing it destroys the vehicle's HV harness and the
// vehicle is not driving anywhere afterwards.  It cannot be undone, tested, or
// taken back.
//
//-----------------------------------------------------------------------------
// WHY TWO SIGNALS AND TWO DELAYS
//
// A single output bit with a glitch on it would be unacceptable.  A coupled
// transient on one wire must not be able to write off a vehicle.  So firing
// requires a sequence that a glitch cannot produce:
//
//   IDLE  --crash_pyro--> ARMING --5 ms--> ARMED --10 ms--> FIRED
//              ^                    |                |
//              +--- crash_pyro drops at any point ---+
//
// The condition must hold continuously for 15 ms.  Any interruption returns to
// IDLE and disarms.  A glitch cannot hold for 15 ms; a real impact can.
//
// Both pyro_fuse_arm and pyro_fuse_fire go to the squib driver, which is
// designed to require both.  Neither alone does anything.
//
//-----------------------------------------------------------------------------
// TIMING IN CONTEXT
//   t = 0      contactors already commanded open by ivcu_hv_crash_detect
//   t = 5 ms   arm
//   t = 15 ms  fire
//   t = 30 ms  airbags would be fully deployed in a typical car
// The fuse fires before the occupant compartment has finished reacting to the
// impact, and long after the contactors have had their chance to open cleanly.
//
//-----------------------------------------------------------------------------
// OPEN SYSTEM-LEVEL ITEM  (spec section 13, O7)
//
// pyro_fired is a sticky flip-flop in the always-on domain, reset only by
// por_n.  For the fact "this vehicle's fuse has been fired" to survive a 12 V
// battery disconnect it needs backing store - an external non-volatile bit or
// a maintained supply.  That is a board-level decision, not an RTL one, and it
// is flagged here so it is not discovered late.  Until it is solved, a
// technician who disconnects the 12 V battery loses the record.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_hv_pyro_trigger (
    input  wire        clk_aon,
    input  wire        rst_hvsafe_n,

    //--- from ivcu_hv_crash_detect ------------------------------------------
    input  wire        crash_pyro,      // impact severe enough to sever
    input  wire        crash_latched,

    //--- authenticated service write ----------------------------------------
    input  wire        service_clear,

    //--- squib driver, both required ----------------------------------------
    output reg         pyro_fuse_arm,
    output reg         pyro_fuse_fire,

    //--- status --------------------------------------------------------------
    output reg         pyro_fired,      // sticky - see the note above
    output wire [2:0]  pyro_state
);

    localparam [2:0] P_IDLE   = 3'd0,
                     P_ARMING = 3'd1,
                     P_ARMED  = 3'd2,
                     P_FIRED  = 3'd3;

    // RULE R1 - constants, never ports
    localparam [16:0] ARM_DELAY  = `PYRO_ARM_DELAY_CYC;   //  5 ms at 10 MHz
    localparam [16:0] FIRE_DELAY = `PYRO_FIRE_DELAY_CYC;  // 10 ms at 10 MHz

    reg [2:0]  state;
    reg [16:0] cnt;

    assign pyro_state = state;

    // the condition must hold the whole way through
    wire hold = crash_pyro & crash_latched;

    always @(posedge clk_aon or negedge rst_hvsafe_n) begin
        if (!rst_hvsafe_n) begin
            state          <= P_IDLE;
            cnt            <= 17'd0;
            pyro_fuse_arm  <= 1'b0;
            pyro_fuse_fire <= 1'b0;
            pyro_fired     <= 1'b0;
        end else if (service_clear && (state == P_FIRED) && !hold) begin
            // the harness has been replaced and a technician has cleared it
            state          <= P_IDLE;
            cnt            <= 17'd0;
            pyro_fuse_arm  <= 1'b0;
            pyro_fuse_fire <= 1'b0;
            pyro_fired     <= 1'b0;
        end else begin
            case (state)

                //-------------------------------------------------------------
                P_IDLE: begin
                    pyro_fuse_arm  <= 1'b0;
                    pyro_fuse_fire <= 1'b0;
                    cnt            <= 17'd0;
                    if (hold) begin
                        state <= P_ARMING;
                    end
                end

                //-------------------------------------------------------------
                // ARMING - 5 ms of continuous evidence before arming
                //-------------------------------------------------------------
                P_ARMING: begin
                    if (!hold) begin
                        state <= P_IDLE;          // glitch - disarm
                        cnt   <= 17'd0;
                    end else if (cnt >= ARM_DELAY) begin
                        pyro_fuse_arm <= 1'b1;
                        cnt           <= 17'd0;
                        state         <= P_ARMED;
                    end else begin
                        cnt <= cnt + 17'd1;
                    end
                end

                //-------------------------------------------------------------
                // ARMED - a further 10 ms.  Dropping hold here still disarms.
                //-------------------------------------------------------------
                P_ARMED: begin
                    if (!hold) begin
                        pyro_fuse_arm <= 1'b0;
                        state         <= P_IDLE;
                        cnt           <= 17'd0;
                    end else if (cnt >= FIRE_DELAY) begin
                        pyro_fuse_fire <= 1'b1;
                        pyro_fired     <= 1'b1;
                        state          <= P_FIRED;
                    end else begin
                        cnt <= cnt + 17'd1;
                    end
                end

                //-------------------------------------------------------------
                // FIRED - terminal.  The cable is severed.  Both drives stay
                // asserted so the squib circuit stays in a defined state.
                //-------------------------------------------------------------
                P_FIRED: begin
                    pyro_fuse_arm  <= 1'b1;
                    pyro_fuse_fire <= 1'b1;
                    pyro_fired     <= 1'b1;
                end

                default: state <= P_IDLE;

            endcase
        end
    end

endmodule

`default_nettype wire
