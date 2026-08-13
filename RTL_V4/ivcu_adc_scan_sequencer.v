//=============================================================================
// ivcu_adc_scan_sequencer.v  -  the multiplexed sensor input path
//
// THIS BLOCK IS THE REASON THE FLOORPLAN WILL WORK THIS TIME.
//
// V3 gave every sensor its own dedicated port group: 42 channels x 32 bits =
// 1,344 pins, of which 864 were measured unconnected.  On a 1520 x 1420 um die
// the perimeter is about 5,840 um, so that is 4.3 um per pin before a single
// other port exists.  The tool was not misbehaving when pins and wires would
// not align - it was telling the truth about an impossible boundary.
//
// Here the whole sensor interface is 21 pins:
//     adc_data[11:0]  adc_chan[5:0]  adc_req  adc_valid  adc_busy
//
// This sequencer walks the enabled channels in order, requests one conversion
// at a time, and hands the result downstream tagged with its channel number.
//
// SPEED
//   Four clocks per channel (SELECT, REQUEST, WAIT, CAPTURE) with a one-cycle
//   ADC.  At 25 MHz:
//       car  mode  64 channels x 4 / 25 MHz = 10.24 us per sweep
//       bike mode  48 channels x 4 / 25 MHz =  7.68 us per sweep
//   The fastest thing on the vehicle is the crash accelerometer, which needs
//   sub-millisecond response.  This is a hundred times faster than that.  The
//   genuinely time-critical discretes - crash trigger, HVIL, brake switch -
//   do not use this path at all; they have their own direct pins and never
//   wait for a scan slot.
//
// THE FREE BENEFIT: NO-RESPONSE DETECTION
//   The sequencer knows it asked channel i for a conversion.  If adc_valid
//   never comes back, channel i is dead - not "reading a strange number",
//   dead.  That is samp_timeout, and it is what eventually puts the words
//   "sensor not working" on the dashboard instead of a plausible-looking
//   value.  V3 declared a validation_timeout signal and never used it, so a
//   sensor whose wire had fallen off read as perfectly healthy.
//
// REST STATE
//   A channel masked out by sensor_enable is skipped entirely.  It costs one
//   clock to step over, consumes no conversion, and produces no sample.  Its
//   analog front end is separately powered down by ivcu_afe_serializer.  That
//   is what "the sensor sits at rest" means here - not a mask on a data bus.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_adc_scan_sequencer (
    input  wire        clk_sensor,
    input  wire        rst_sensor_n,

    // --- which channels are live in the active vehicle mode ----------------
    input  wire [63:0] sensor_enable,

    // --- external ADC interface (21 top-level pins) ------------------------
    output reg  [5:0]  adc_chan,     // channel being requested
    output reg         adc_req,      // one-cycle conversion request
    input  wire [11:0] adc_data,     // conversion result
    input  wire        adc_valid,    // result is good
    input  wire        adc_busy,     // ADC is converting, do not request

    // --- tagged sample stream to the conditioning and detection path -------
    output reg  [5:0]  samp_chan,    // which channel this sample belongs to
    output reg  [15:0] samp_data,    // zero-extended 12-bit reading
    output reg         samp_valid,   // one-cycle strobe: good sample
    output reg         samp_timeout, // one-cycle strobe: channel did not answer
    output reg         sweep_done    // one-cycle strobe: wrapped past channel 63
);

    //-------------------------------------------------------------------------
    // State encoding.  Four states, one channel per pass.
    //-------------------------------------------------------------------------
    localparam [1:0] ST_SELECT  = 2'd0,   // is this channel enabled?
                     ST_REQUEST = 2'd1,   // drive adc_req for one cycle
                     ST_WAIT    = 2'd2,   // wait for adc_valid, or time out
                     ST_CAPTURE = 2'd3;   // emit the strobe, advance

    // R1: the timeout is a constant from ivcu_defs.vh, never a port.
    localparam [7:0] TIMEOUT_CYC = `ADC_TIMEOUT_CYC;

    reg [1:0] state;
    reg [5:0] ptr;        // channel pointer, wraps naturally at 6 bits
    reg [7:0] tmo;        // no-response counter

    wire      chan_live = sensor_enable[ptr];
    wire      ptr_last  = (ptr == 6'd63);

    always @(posedge clk_sensor or negedge rst_sensor_n) begin
        if (!rst_sensor_n) begin
            state        <= ST_SELECT;
            ptr          <= 6'd0;
            tmo          <= 8'd0;
            adc_chan     <= 6'd0;
            adc_req      <= 1'b0;
            samp_chan    <= 6'd0;
            samp_data    <= 16'd0;
            samp_valid   <= 1'b0;
            samp_timeout <= 1'b0;
            sweep_done   <= 1'b0;
        end else begin
            // default: all strobes are one cycle wide
            adc_req      <= 1'b0;
            samp_valid   <= 1'b0;
            samp_timeout <= 1'b0;
            sweep_done   <= 1'b0;

            case (state)

                //-------------------------------------------------------------
                // SELECT - skip channels that are not fitted on this vehicle
                //-------------------------------------------------------------
                ST_SELECT: begin
                    if (!chan_live) begin
                        // one clock to step over a disabled channel
                        ptr        <= ptr + 6'd1;
                        sweep_done <= ptr_last;
                    end else if (!adc_busy) begin
                        adc_chan <= ptr;
                        adc_req  <= 1'b1;
                        state    <= ST_REQUEST;
                    end
                    // if adc_busy, hold here until the ADC is free
                end

                //-------------------------------------------------------------
                // REQUEST - adc_req was asserted last cycle, arm the timeout
                //-------------------------------------------------------------
                ST_REQUEST: begin
                    tmo   <= 8'd0;
                    state <= ST_WAIT;
                end

                //-------------------------------------------------------------
                // WAIT - the answer, or the silence that means the channel is
                //        dead.  Both outcomes are information.
                //-------------------------------------------------------------
                ST_WAIT: begin
                    if (adc_valid) begin
                        samp_chan  <= adc_chan;
                        samp_data  <= {4'b0000, adc_data};
                        samp_valid <= 1'b1;
                        state      <= ST_CAPTURE;
                    end else if (tmo >= TIMEOUT_CYC) begin
                        samp_chan    <= adc_chan;
                        samp_timeout <= 1'b1;
                        state        <= ST_CAPTURE;
                    end else begin
                        tmo <= tmo + 8'd1;
                    end
                end

                //-------------------------------------------------------------
                // CAPTURE - advance to the next channel
                //-------------------------------------------------------------
                ST_CAPTURE: begin
                    ptr        <= ptr + 6'd1;
                    sweep_done <= ptr_last;
                    state      <= ST_SELECT;
                end

                default: state <= ST_SELECT;

            endcase
        end
    end

endmodule

`default_nettype wire
