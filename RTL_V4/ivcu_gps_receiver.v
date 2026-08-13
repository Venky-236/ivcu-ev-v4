//=============================================================================
// ivcu_gps_receiver.v  -  real position, from a real pin
//
// V3's emergency location was this:
//
//     location_data <= {gps_accuracy[3:0], 12'hABC};
//     latitude      <= 16'h1234;
//     longitude     <= 16'h5678;
//
// Hardcoded.  No GPS input reached the module at all.  Every vehicle would
// have reported the same position, somewhere off the coast of Africa, to
// every emergency service, forever.
//
//-----------------------------------------------------------------------------
// THE FRAME FORMAT
//
// A companion GNSS module sends an 11-byte binary frame.  This is deliberately
// not NMEA: parsing ASCII sentences with variable-length fields and comma
// counting is a surprising amount of state machine for a block that must work
// after an impact, and the module that produces NMEA can produce this instead.
//
//     byte  0     0xA5    sync
//     bytes 1-4   latitude,  signed, 1e-7 degrees, MSB first
//     bytes 5-8   longitude, signed, 1e-7 degrees, MSB first
//     byte  9     status: bit 0 fix valid, bits 7:4 satellite count
//     byte  10    checksum: XOR of bytes 1 to 9
//
// 1e-7 degrees is about 11 mm at the equator, which is far finer than any
// GNSS fix, so the format never becomes the limiting factor.
//
//-----------------------------------------------------------------------------
// THE LAST GOOD FIX IS HELD, AND ITS AGE IS REPORTED
//
// A vehicle in a tunnel, or one whose antenna was destroyed in the impact, has
// no current fix.  Reporting nothing would be useless; reporting the last
// known position as if it were current would be worse than useless, because a
// dispatcher would send a crew to where the vehicle was, with no reason to
// doubt it.
//
// So the last valid fix is held, and fix_age_ms counts how old it is.  The
// message carries both.  "Two kilometres back along this road, ninety seconds
// ago" is actionable.  "Here" when it is not here is not.
//
// The age counter saturates rather than wrapping - a wrapped counter would
// eventually announce that a two-hour-old position was current.
//
//-----------------------------------------------------------------------------
// A BAD CHECKSUM DISCARDS THE FRAME AND DOES NOT DISTURB THE HELD FIX
//
// A corrupted frame is not evidence of anything.  It does not update the
// position, does not clear the fix-valid flag, and does not reset the age
// counter.  It is counted, so a persistently noisy link becomes visible.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_gps_receiver (
    input  wire        clk_aon,
    input  wire        rst_aon_n,

    //--- byte stream from the GNSS module (top-level pins) ------------------
    input  wire [7:0]  gps_rx_data,
    input  wire        gps_rx_valid,

    //--- position -------------------------------------------------------------
    output reg  signed [31:0] gps_lat,        // last VALID fix, held
    output reg  signed [31:0] gps_lon,
    output reg  [3:0]  gps_sats,
    output reg         gps_fix_valid,         // a fix has been received, ever
    output reg         gps_fix_current,       // the last frame had a live fix
    output reg  [15:0] fix_age_ms,            // saturating
    output reg  [7:0]  frame_err_count        // bad checksums seen
);

    localparam [7:0]  SYNC_BYTE   = 8'hA5;

    // The frame is 11 bytes.  The state machine indexes them explicitly rather
    // than counting against a length constant, because a case per byte can be
    // checked line by line against the format in the header - and a length
    // constant that nothing reads is dead weight.

    // clk_aon is 10 MHz, so one millisecond is 10,000 cycles.
    localparam [13:0] MS_CYCLES   = 14'd10_000;

    reg [3:0]  bidx;           // byte index within the frame
    reg [31:0] lat_sh;
    reg [31:0] lon_sh;
    reg [7:0]  status_b;
    reg [7:0]  csum;
    reg [13:0] ms_div;

    always @(posedge clk_aon or negedge rst_aon_n) begin
        if (!rst_aon_n) begin
            bidx            <= 4'd0;
            lat_sh          <= 32'd0;
            lon_sh          <= 32'd0;
            status_b        <= 8'd0;
            csum            <= 8'd0;
            gps_lat         <= 32'sd0;
            gps_lon         <= 32'sd0;
            gps_sats        <= 4'd0;
            gps_fix_valid   <= 1'b0;
            gps_fix_current <= 1'b0;
            fix_age_ms      <= 16'd0;
            frame_err_count <= 8'd0;
            ms_div          <= 14'd0;
        end else begin

            //-----------------------------------------------------------------
            // Age of the held fix, in milliseconds, saturating.
            //-----------------------------------------------------------------
            if (ms_div >= (MS_CYCLES - 14'd1)) begin
                ms_div <= 14'd0;
                if (gps_fix_valid && !gps_fix_current &&
                    (fix_age_ms != 16'hFFFF)) begin
                    fix_age_ms <= fix_age_ms + 16'd1;
                end
            end else begin
                ms_div <= ms_div + 14'd1;
            end

            //-----------------------------------------------------------------
            // Frame assembly
            //-----------------------------------------------------------------
            if (gps_rx_valid) begin
                case (bidx)

                    4'd0: begin
                        // resynchronise on every sync byte.  If the link
                        // glitches mid-frame, the next 0xA5 recovers it.
                        if (gps_rx_data == SYNC_BYTE) begin
                            bidx <= 4'd1;
                            csum <= 8'd0;
                        end
                    end

                    4'd1, 4'd2, 4'd3, 4'd4: begin
                        lat_sh <= {lat_sh[23:0], gps_rx_data};
                        csum   <= csum ^ gps_rx_data;
                        bidx   <= bidx + 4'd1;
                    end

                    4'd5, 4'd6, 4'd7, 4'd8: begin
                        lon_sh <= {lon_sh[23:0], gps_rx_data};
                        csum   <= csum ^ gps_rx_data;
                        bidx   <= bidx + 4'd1;
                    end

                    4'd9: begin
                        status_b <= gps_rx_data;
                        csum     <= csum ^ gps_rx_data;
                        bidx     <= 4'd10;
                    end

                    4'd10: begin
                        bidx <= 4'd0;

                        if (csum == gps_rx_data) begin
                            //-----------------------------------------------
                            // Good frame.  Only a frame that claims a live
                            // fix updates the held position.
                            //-----------------------------------------------
                            gps_sats        <= status_b[7:4];
                            gps_fix_current <= status_b[0];

                            if (status_b[0]) begin
                                gps_lat       <= $signed(lat_sh);
                                gps_lon       <= $signed(lon_sh);
                                gps_fix_valid <= 1'b1;
                                fix_age_ms    <= 16'd0;
                            end
                        end else begin
                            //-----------------------------------------------
                            // Bad checksum.  Discard, count, disturb nothing.
                            //-----------------------------------------------
                            if (frame_err_count != 8'hFF) begin
                                frame_err_count <= frame_err_count + 8'd1;
                            end
                        end
                    end

                    default: bidx <= 4'd0;

                endcase
            end
        end
    end

endmodule

`default_nettype wire
