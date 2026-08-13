//=============================================================================
// ivcu_afe_serializer.v  -  analog front-end power control, 3 pins for 64 rails
//
// WHY THIS BLOCK EXISTS
//
// You asked that sensors which do not belong to the active vehicle mode sit at
// rest and genuinely not work.  In V3 that was a mask on a data bus: the
// sensor was still powered, still converted, still burned scan bandwidth, and
// the "disabled" state existed only in the chip's opinion.
//
// Real rest means the sensor itself is unpowered.  That needs one enable per
// channel - 64 signals.  Sixty-four dedicated pins would undo most of the pin
// saving that ivcu_adc_scan_sequencer just bought, so they go out serially to
// an external 64-bit shift register:
//
//     afe_sclk    shift clock, 6.25 MHz  (clk_sensor / 4)
//     afe_sdata   serial data, MSB first (channel 63 shifts out first)
//     afe_latch   one pulse after all 64 bits, transfers to the output latches
//
// Three pins instead of sixty-four.  Same reasoning as the ADC bus.
//
// UPDATE RATE
//   64 bits at 6.25 MHz = 10.24 us per update, and an update only happens when
//   the mask actually changes.  The mask is latched at power-on and cannot
//   change while the vehicle is moving, so in practice this runs once per key
//   cycle.  There is no need for it to be fast and no benefit in making it so.
//
// SAFE DIRECTION OF FAILURE
//   afe_enable is held at zero through reset, so every front end is OFF until
//   the mode manager has resolved the vehicle type and driven a real mask.  A
//   chip that has not yet decided whether it is in a car or on a motorcycle
//   powers nothing.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_afe_serializer (
    input  wire        clk_sensor,
    input  wire        rst_sensor_n,

    // --- which analog front ends should be powered -------------------------
    input  wire [63:0] afe_enable,

    // --- 3-pin serial chain to the external shift register -----------------
    output reg         afe_sclk,
    output reg         afe_sdata,
    output reg         afe_latch,

    // --- status ------------------------------------------------------------
    output reg         afe_busy      // a shift-out is in progress
);

    localparam [1:0] ST_IDLE  = 2'd0,
                     ST_SHIFT = 2'd1,
                     ST_LATCH = 2'd2;

    reg [1:0]  state;
    reg [63:0] shadow;      // last value successfully shifted out
    reg [63:0] shreg;       // shift register, MSB first
    reg [6:0]  bitcnt;      // 0..63, 7 bits so 64 is representable
    reg [1:0]  phase;       // clk_sensor / 4 divider for afe_sclk

    wire changed = (afe_enable != shadow);

    always @(posedge clk_sensor or negedge rst_sensor_n) begin
        if (!rst_sensor_n) begin
            state     <= ST_IDLE;
            shadow    <= 64'd0;
            shreg     <= 64'd0;
            bitcnt    <= 7'd0;
            phase     <= 2'd0;
            afe_sclk  <= 1'b0;
            afe_sdata <= 1'b0;
            afe_latch <= 1'b0;
            afe_busy  <= 1'b0;
        end else begin
            afe_latch <= 1'b0;              // one-cycle strobe

            case (state)

                //-------------------------------------------------------------
                // IDLE - wait for the mask to change
                //-------------------------------------------------------------
                ST_IDLE: begin
                    afe_sclk <= 1'b0;
                    afe_busy <= 1'b0;
                    phase    <= 2'd0;
                    bitcnt   <= 7'd0;
                    if (changed) begin
                        shreg    <= afe_enable;
                        shadow   <= afe_enable;
                        afe_busy <= 1'b1;
                        state    <= ST_SHIFT;
                    end
                end

                //-------------------------------------------------------------
                // SHIFT - four clk_sensor phases per output bit
                //
                //   phase 0 : present the bit, sclk low
                //   phase 1 : setup time
                //   phase 2 : sclk rises - the external register samples here
                //   phase 3 : sclk falls, advance the shift register
                //-------------------------------------------------------------
                ST_SHIFT: begin
                    phase <= phase + 2'd1;
                    case (phase)
                        2'd0: begin
                            afe_sdata <= shreg[63];
                            afe_sclk  <= 1'b0;
                        end
                        2'd1: begin
                            afe_sclk  <= 1'b0;
                        end
                        2'd2: begin
                            afe_sclk  <= 1'b1;      // rising edge, data sampled
                        end
                        default: begin              // phase 3
                            afe_sclk <= 1'b0;
                            shreg    <= {shreg[62:0], 1'b0};
                            if (bitcnt == 7'd63) begin
                                state <= ST_LATCH;
                            end else begin
                                bitcnt <= bitcnt + 7'd1;
                            end
                        end
                    endcase
                end

                //-------------------------------------------------------------
                // LATCH - transfer the shifted word to the output pins of the
                //         external register
                //-------------------------------------------------------------
                ST_LATCH: begin
                    afe_sclk  <= 1'b0;
                    afe_latch <= 1'b1;
                    state     <= ST_IDLE;
                end

                default: state <= ST_IDLE;

            endcase
        end
    end

endmodule

`default_nettype wire
