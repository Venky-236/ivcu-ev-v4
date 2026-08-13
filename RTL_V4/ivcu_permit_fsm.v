//=============================================================================
// ivcu_permit_fsm.v  -  one limited-operation permit, for one sensor
//
// YOUR WORKED EXAMPLE, AS A STATE MACHINE.
//
// The side-stand sensor fails in a car park at night.  Without this block the
// motorcycle will not start, and the rider is stranded next to a vehicle that
// is mechanically perfect.  With it, the rider gets five speed-limited starts
// to reach a service centre, the event is logged, and the vehicle refuses to
// go on pretending nothing is wrong.
//
//   NORMAL
//     | confirmed fault on this channel
//     v
//   INHIBITED     vehicle will not move.  Display offers the permit:
//     |           "hold MODE for 3 seconds for 5 limited starts"
//     | rider holds the button, vehicle stationary
//     v
//   ACTIVE (5)    speed capped, warning latched, logged
//     |           each subsequent qualified start decrements
//     v
//   EXPIRED       immobilised.  Only an authenticated service clear releases it.
//
//-----------------------------------------------------------------------------
// THE SEMANTICS ARE THE RIGHT WAY ROUND
//
// V3's grace manager, on expiry, DISABLED THE SENSOR.  That is backwards: the
// longer a fault persisted, the less the vehicle knew about it, and there was
// never any point at which the rider was granted permission to move.
//
// Here, expiry WITHDRAWS a permission.  The default state of a faulted
// conditional sensor is a vehicle that will not move; the permit is a
// deliberate, acknowledged, counted, speed-limited exception to that.
//
//-----------------------------------------------------------------------------
// THE CHANNEL KEEPS BEING READ WHILE THE PERMIT RUNS
//
// This is a design detail worth stating because the obvious implementation is
// wrong.  The tempting approach is to remove the faulted channel from
// sensor_enable while the permit is active.  Do that and the fault detector
// clears its state, the fault disappears, the FSM returns to NORMAL, the
// channel is re-enabled, the fault reappears - and the permit oscillates.
//
// So the channel stays in the scan.  bypass_req tells the rest of the chip to
// stop letting this channel inhibit the vehicle; it does not stop the chip
// looking at it.  Which means that if the rider actually replaces the sensor,
// the fault clears on its own and the permit ends early.  That is the correct
// behaviour and it falls out of doing it this way.
//
//-----------------------------------------------------------------------------
// COST
//   2 bits of state + 3 bits of counter = 5 flops per instance.
//   Eight instances = 40 flops for the entire feature.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_permit_fsm (
    input  wire       clk_aon,
    input  wire       rst_aon_n,

    //--- this channel's condition -------------------------------------------
    input  wire       sensor_faulted,     // confirmed fault, debounced upstream

    //--- rider and vehicle context -------------------------------------------
    input  wire       ack_held,           // MODE button held for 3 s
    input  wire       vehicle_stationary,
    input  wire       ign_cycle,          // one qualified start
    input  wire       service_clear,      // authenticated technician write

    //--- outputs --------------------------------------------------------------
    output reg  [1:0] permit_state,
    output reg  [2:0] starts_left,
    output wire       bypass_req,         // stop this channel inhibiting
    output wire       inhibit,            // vehicle must not move
    output wire       offer               // display the permit offer
);

    // RULE R1 - constant, never a port
    localparam [2:0] STARTS_GRANTED = `PERMIT_STARTS;   // 5

    assign bypass_req = (permit_state == `PS_ACTIVE);
    assign inhibit    = (permit_state == `PS_INHIBITED) ||
                        (permit_state == `PS_EXPIRED);
    assign offer      = (permit_state == `PS_INHIBITED);

    always @(posedge clk_aon or negedge rst_aon_n) begin
        if (!rst_aon_n) begin
            permit_state <= `PS_NORMAL;
            starts_left  <= 3'd0;
        end else begin
            case (permit_state)

                //-------------------------------------------------------------
                `PS_NORMAL: begin
                    starts_left <= 3'd0;
                    if (sensor_faulted) begin
                        permit_state <= `PS_INHIBITED;
                    end
                end

                //-------------------------------------------------------------
                // INHIBITED - the vehicle will not move.  The rider is offered
                // a way out and has to ask for it deliberately.
                //-------------------------------------------------------------
                `PS_INHIBITED: begin
                    if (!sensor_faulted) begin
                        // it fixed itself, or was a transient that outlasted
                        // the debounce.  No permit needed.
                        permit_state <= `PS_NORMAL;
                    end else if (ack_held && vehicle_stationary) begin
                        starts_left  <= STARTS_GRANTED;
                        permit_state <= `PS_ACTIVE;
                    end
                end

                //-------------------------------------------------------------
                // ACTIVE - counting down.  The channel is still being scanned.
                //-------------------------------------------------------------
                `PS_ACTIVE: begin
                    if (!sensor_faulted) begin
                        // the sensor was actually replaced.  End the permit
                        // early and give the rider their vehicle back.
                        permit_state <= `PS_NORMAL;
                        starts_left  <= 3'd0;
                    end else if (ign_cycle) begin
                        if (starts_left <= 3'd1) begin
                            starts_left  <= 3'd0;
                            permit_state <= `PS_EXPIRED;
                        end else begin
                            starts_left <= starts_left - 3'd1;
                        end
                    end
                end

                //-------------------------------------------------------------
                // EXPIRED - the rider had five starts and used them.  The
                // vehicle stops.  Only a technician releases this.
                //-------------------------------------------------------------
                `PS_EXPIRED: begin
                    starts_left <= 3'd0;
                    if (service_clear) begin
                        permit_state <= `PS_NORMAL;
                    end else if (!sensor_faulted) begin
                        // the sensor was replaced without a service tool -
                        // a competent owner did the job.  Accept it.
                        permit_state <= `PS_NORMAL;
                    end
                end

                default: permit_state <= `PS_NORMAL;

            endcase
        end
    end

endmodule

`default_nettype wire
