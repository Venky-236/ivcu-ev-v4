//=============================================================================
// ivcu_sos_builder.v  -  the message that gets someone sent
//
// V3 had a single sos_signal bit.  One wire, one meaning: "something".  No
// position that was real, no severity, no way to distinguish a car park bump
// from a motorcycle down at 80 km/h with a venting battery pack.
//
// This block builds a 25-byte frame and streams it to the telematics modem.
//
//-----------------------------------------------------------------------------
// THE FRAME
//
//   byte  0      0x5A                     sync
//   byte  1      0x04                     format version
//   byte  2      severity[2:0] route[2:0] mode[1:0]
//   bytes 3-6    latitude    signed 1e-7 deg, MSB first
//   bytes 7-10   longitude   signed 1e-7 deg
//   bytes 11-12  fix age in milliseconds  (0 = the fix is live)
//   byte  13     direction[2:0] occupants[2:0] hv_isolated pyro_fired
//   bytes 14-15  peak longitudinal acceleration
//   bytes 16-17  peak lateral acceleration
//   bytes 18-19  hottest cell temperature at impact
//   bytes 20-21  pack enclosure pressure at impact
//   byte  22     speed at impact, km/h
//   byte  23     hv_state[2:0] hv_fault[3:0] battery_incident
//   byte  24     checksum, XOR of bytes 1 to 23
//
//-----------------------------------------------------------------------------
// THE PAYLOAD IS LATCHED AT TRIGGER, NOT READ LIVE
//
// A crash is a moment.  The peak acceleration, the cell temperature, the speed
// - those are facts about that moment, and by the time the modem has finished
// sending, the vehicle may have rolled, cooled, or lost its GPS antenna.
//
// So every field is captured into registers when the message is triggered, and
// the stream reads only from those registers.  Retransmissions send the same
// frame, not a frame describing a slowly changing wreck.
//
//-----------------------------------------------------------------------------
// IT IS SENT MORE THAN ONCE, AND AGAIN IF THINGS GET WORSE
//
// A crash message that is lost is a crash message.  The frame is sent three
// times with a five second gap - cheap insurance on a link whose antenna may
// have just been damaged.
//
// And if severity escalates - a pack that starts venting a minute after the
// impact - the payload is re-latched and the sequence restarts.  Responders
// who were told "moderate, send police" get told "critical, lithium fire".
// That is the one case where sending again with different content is right.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_sos_builder (
    input  wire        clk_aon,
    input  wire        rst_aon_n,

    //--- classification -------------------------------------------------------
    input  wire [2:0]  crash_severity,
    input  wire [2:0]  sos_route,
    input  wire [2:0]  occupant_count,
    input  wire        severity_valid,
    input  wire        battery_incident,

    //--- position -------------------------------------------------------------
    input  wire signed [31:0] gps_lat,
    input  wire signed [31:0] gps_lon,
    input  wire [15:0] fix_age_ms,

    //--- impact facts ---------------------------------------------------------
    input  wire [2:0]  crash_direction,
    input  wire [15:0] crash_peak_long,
    input  wire [15:0] crash_peak_lat,
    input  wire [15:0] hv_cell_tmax,
    input  wire [15:0] hv_encl_press,
    input  wire [7:0]  speed_at_impact_kph,
    input  wire        hv_isolated,
    input  wire        pyro_fired,
    input  wire [2:0]  hv_state,
    input  wire [3:0]  hv_fault_code,
    input  wire [1:0]  active_mode,

    //--- telematics modem, top-level pins ------------------------------------
    output reg  [7:0]  sos_tx_data,
    output reg         sos_tx_valid,
    input  wire        sos_tx_ready,

    //--- status ----------------------------------------------------------------
    output reg         sos_sending,
    output reg  [1:0]  sos_tx_count       // frames sent so far
);

    localparam [7:0] SYNC_BYTE  = 8'h5A;
    localparam [7:0] FMT_VER    = 8'h04;
    localparam [4:0] LAST_BYTE  = 5'd24;

    localparam [1:0]  RETRY_MAX = 2'd3;
    // 5 seconds at 10 MHz
    localparam [25:0] RETRY_GAP = 26'd50_000_000;

    localparam [1:0] ST_IDLE = 2'd0,
                     ST_SEND = 2'd1,
                     ST_GAP  = 2'd2;

    //=========================================================================
    // LATCHED PAYLOAD - captured at trigger, never re-read from the inputs
    //=========================================================================
    reg [2:0]  p_sev;
    reg [2:0]  p_route;
    reg [2:0]  p_occ;
    reg [1:0]  p_mode;
    reg [31:0] p_lat;
    reg [31:0] p_lon;
    reg [15:0] p_age;
    reg [2:0]  p_dir;
    reg [15:0] p_peak_long;
    reg [15:0] p_peak_lat;
    reg [15:0] p_tmax;
    reg [15:0] p_press;
    reg [7:0]  p_speed;
    reg        p_isolated;
    reg        p_pyro;
    reg [2:0]  p_hvstate;
    reg [3:0]  p_hvfault;
    reg        p_battinc;

    reg [1:0]  state;
    reg [4:0]  bidx;
    reg [25:0] gap_cnt;
    reg [7:0]  csum;
    reg [2:0]  last_sev;      // to detect escalation

    //=========================================================================
    // BYTE MUX - reads only the latched payload
    //=========================================================================
    function [7:0] frame_byte;
        input [4:0] i;
        begin
            case (i)
                5'd0 : frame_byte = SYNC_BYTE;
                5'd1 : frame_byte = FMT_VER;
                5'd2 : frame_byte = {p_sev, p_route, p_mode};
                5'd3 : frame_byte = p_lat[31:24];
                5'd4 : frame_byte = p_lat[23:16];
                5'd5 : frame_byte = p_lat[15:8];
                5'd6 : frame_byte = p_lat[7:0];
                5'd7 : frame_byte = p_lon[31:24];
                5'd8 : frame_byte = p_lon[23:16];
                5'd9 : frame_byte = p_lon[15:8];
                5'd10: frame_byte = p_lon[7:0];
                5'd11: frame_byte = p_age[15:8];
                5'd12: frame_byte = p_age[7:0];
                5'd13: frame_byte = {p_dir, p_occ, p_isolated, p_pyro};
                5'd14: frame_byte = p_peak_long[15:8];
                5'd15: frame_byte = p_peak_long[7:0];
                5'd16: frame_byte = p_peak_lat[15:8];
                5'd17: frame_byte = p_peak_lat[7:0];
                5'd18: frame_byte = p_tmax[15:8];
                5'd19: frame_byte = p_tmax[7:0];
                5'd20: frame_byte = p_press[15:8];
                5'd21: frame_byte = p_press[7:0];
                5'd22: frame_byte = p_speed;
                5'd23: frame_byte = {p_hvstate, p_hvfault, p_battinc};
                default: frame_byte = csum;         // byte 24
            endcase
        end
    endfunction

    //=========================================================================
    // Trigger: a new classification, or an escalation of an existing one
    //=========================================================================
    wire escalated = severity_valid && (crash_severity > last_sev);

    always @(posedge clk_aon or negedge rst_aon_n) begin
        if (!rst_aon_n) begin
            state        <= ST_IDLE;
            bidx         <= 5'd0;
            gap_cnt      <= 26'd0;
            csum         <= 8'd0;
            last_sev     <= `CRASH_NONE;
            sos_tx_data  <= 8'd0;
            sos_tx_valid <= 1'b0;
            sos_sending  <= 1'b0;
            sos_tx_count <= 2'd0;
            p_sev        <= 3'd0;  p_route     <= 3'd0;  p_occ    <= 3'd0;
            p_mode       <= 2'd0;  p_lat       <= 32'd0; p_lon    <= 32'd0;
            p_age        <= 16'd0; p_dir       <= 3'd0;
            p_peak_long  <= 16'd0; p_peak_lat  <= 16'd0;
            p_tmax       <= 16'd0; p_press     <= 16'd0;
            p_speed      <= 8'd0;  p_isolated  <= 1'b0;  p_pyro   <= 1'b0;
            p_hvstate    <= 3'd0;  p_hvfault   <= 4'd0;  p_battinc<= 1'b0;
        end else begin

            //-----------------------------------------------------------------
            // Latch the payload on a new event or an escalation.  This is the
            // only place these registers are written.
            //-----------------------------------------------------------------
            if (escalated) begin
                p_sev       <= crash_severity;
                p_route     <= sos_route;
                p_occ       <= occupant_count;
                p_mode      <= active_mode;
                p_lat       <= gps_lat;
                p_lon       <= gps_lon;
                p_age       <= fix_age_ms;
                p_dir       <= crash_direction;
                p_peak_long <= crash_peak_long;
                p_peak_lat  <= crash_peak_lat;
                p_tmax      <= hv_cell_tmax;
                p_press     <= hv_encl_press;
                p_speed     <= speed_at_impact_kph;
                p_isolated  <= hv_isolated;
                p_pyro      <= pyro_fired;
                p_hvstate   <= hv_state;
                p_hvfault   <= hv_fault_code;
                p_battinc   <= battery_incident;

                last_sev     <= crash_severity;
                state        <= ST_SEND;
                bidx         <= 5'd0;
                csum         <= 8'd0;
                sos_tx_count <= 2'd0;
                sos_sending  <= 1'b1;
            end else begin

                case (state)

                    ST_IDLE: begin
                        sos_tx_valid <= 1'b0;
                        sos_sending  <= 1'b0;
                    end

                    //---------------------------------------------------------
                    // SEND - one byte per accepted handshake
                    //---------------------------------------------------------
                    ST_SEND: begin
                        sos_sending <= 1'b1;

                        if (!sos_tx_valid) begin
                            sos_tx_data  <= frame_byte(bidx);
                            sos_tx_valid <= 1'b1;
                        end else if (sos_tx_ready) begin
                            sos_tx_valid <= 1'b0;

                            // the checksum covers bytes 1 to 23
                            if ((bidx >= 5'd1) && (bidx <= 5'd23)) begin
                                csum <= csum ^ frame_byte(bidx);
                            end

                            if (bidx == LAST_BYTE) begin
                                bidx <= 5'd0;
                                if (sos_tx_count >= (RETRY_MAX - 2'd1)) begin
                                    state       <= ST_IDLE;
                                    sos_sending <= 1'b0;
                                end else begin
                                    sos_tx_count <= sos_tx_count + 2'd1;
                                    gap_cnt      <= 26'd0;
                                    csum         <= 8'd0;
                                    state        <= ST_GAP;
                                end
                            end else begin
                                bidx <= bidx + 5'd1;
                            end
                        end
                    end

                    //---------------------------------------------------------
                    // GAP - five seconds between attempts
                    //---------------------------------------------------------
                    ST_GAP: begin
                        sos_tx_valid <= 1'b0;
                        if (gap_cnt >= RETRY_GAP) begin
                            state <= ST_SEND;
                        end else begin
                            gap_cnt <= gap_cnt + 26'd1;
                        end
                    end

                    default: state <= ST_IDLE;

                endcase
            end
        end
    end

endmodule

`default_nettype wire
