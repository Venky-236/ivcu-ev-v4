//=============================================================================
// ivcu_hv_sense_sync.v  -  coherent snapshot of the HV channels, clk_sensor
//                          into clk_aon
//
// The HV safety island lives in clk_aon.  The sensor values it reasons about
// are produced in clk_sensor.  RULE R7 says every crossing goes through a
// synchroniser, and this is that synchroniser for the whole HV group.
//
// WHY ONE HANDSHAKE FOR ALL THIRTEEN VALUES, NOT THIRTEEN HANDSHAKES
//
// The island makes decisions that depend on several channels at once:
//   "is the DC link within 95 % of pack voltage"        needs 2 and 8 together
//   "is the pack venting while a cell is over 65 C"     needs 11 and 17 together
//
// If each channel crossed independently, the island could see a DC-link
// voltage from one sweep against a pack voltage from the next.  Those are
// values that never coexisted, and reasoning about them produces a conclusion
// about a vehicle that does not exist.  For a health score that is a wrong
// number; for a precharge decision it welds a contactor.
//
// So all thirteen values are captured into holding registers on the same
// sweep_done pulse, and one toggle crosses the boundary.  The destination
// either has the whole snapshot or the previous whole snapshot.  Never a mix.
//
// Cost: one toggle synchroniser plus 13 x 16 holding bits, against thirteen
// separate synchronisers.  Cheaper AND correct, which is unusual and worth
// taking when it happens.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_hv_sense_sync (
    //--- source domain ------------------------------------------------------
    input  wire          clk_sensor,
    input  wire          rst_sensor_n,
    input  wire [1023:0] sensor_value_flat,
    input  wire [63:0]   sensor_dead,
    input  wire          sweep_done,      // one pulse per complete sweep

    //--- destination domain -------------------------------------------------
    input  wire          clk_aon,
    input  wire          rst_hvsafe_n,

    //--- the snapshot, all from the same sweep ------------------------------
    output reg  [15:0]   q_hvil,
    output reg  [15:0]   q_iso,
    output reg  [15:0]   q_bus_v,
    output reg  [15:0]   q_pre_v,
    output reg  [15:0]   q_fb_pos,
    output reg  [15:0]   q_fb_neg,
    output reg  [15:0]   q_pack_v,
    output reg  [15:0]   q_dc_i,
    output reg  [15:0]   q_cell_tmax,
    output reg  [15:0]   q_encl_p,
    output reg  [15:0]   q_crash_f,
    output reg  [15:0]   q_crash_s,
    output reg  [15:0]   q_tip,
    output reg  [15:0]   q_accel_long,
    output reg  [15:0]   q_accel_lat,
    output reg  [14:0]   q_dead,          // one bit per value above, same order
    output reg           q_valid          // one pulse per new snapshot
);

    //-------------------------------------------------------------------------
    // Channel indices widened to integers before any index arithmetic.
    // `S_* are sized 6-bit literals; multiplying one by 16 inside a
    // part-select evaluates in 6-bit width and truncates.  See the same note
    // in ivcu_sensor_plausibility.
    //-------------------------------------------------------------------------
    localparam integer C_HVIL  = `S_HVIL_LOOP;
    localparam integer C_ISO   = `S_ISOLATION_RES;
    localparam integer C_BUSV  = `S_HV_BUS_VOLT;
    localparam integer C_PREV  = `S_PRECHARGE_VOLT;
    localparam integer C_FBP   = `S_CONTACTOR_FB_POS;
    localparam integer C_FBN   = `S_CONTACTOR_FB_NEG;
    localparam integer C_PACKV = `S_PACK_VOLTAGE;
    localparam integer C_DCI   = `S_DC_LINK_CURRENT;
    localparam integer C_TMAX  = `S_CELL_TEMP_MAX;
    localparam integer C_ENCP  = `S_PACK_ENCL_PRESS;
    localparam integer C_CRF   = `S_CRASH_FRONT;
    localparam integer C_CRS   = `S_CRASH_SIDE;
    localparam integer C_TIP   = `S_TIP_OVER;
    localparam integer C_ACLG  = `S_ACCEL_LONG;
    localparam integer C_ACLT  = `S_ACCEL_LAT;

    //=========================================================================
    // SOURCE SIDE - capture the whole group, then flip one bit
    //=========================================================================
    reg [15:0] h_hvil, h_iso, h_bus_v, h_pre_v, h_fb_pos, h_fb_neg;
    reg [15:0] h_pack_v, h_dc_i, h_cell_tmax, h_encl_p;
    reg [15:0] h_crash_f, h_crash_s, h_tip;
    reg [15:0] h_accel_long, h_accel_lat;
    reg [14:0] h_dead;
    reg        toggle_q;

    always @(posedge clk_sensor or negedge rst_sensor_n) begin
        if (!rst_sensor_n) begin
            h_hvil       <= 16'd0;  h_iso       <= 16'd0;
            h_bus_v      <= 16'd0;  h_pre_v     <= 16'd0;
            h_fb_pos     <= 16'd0;  h_fb_neg    <= 16'd0;
            h_pack_v     <= 16'd0;  h_dc_i      <= 16'd0;
            h_cell_tmax  <= 16'd0;  h_encl_p    <= 16'd0;
            h_crash_f    <= 16'd0;  h_crash_s   <= 16'd0;
            h_tip        <= 16'd0;
            h_accel_long <= 16'd0;  h_accel_lat <= 16'd0;
            h_dead       <= 15'd0;
            toggle_q     <= 1'b0;
        end else if (sweep_done) begin
            h_hvil      <= sensor_value_flat[C_HVIL  *16 +: 16];
            h_iso       <= sensor_value_flat[C_ISO   *16 +: 16];
            h_bus_v     <= sensor_value_flat[C_BUSV  *16 +: 16];
            h_pre_v     <= sensor_value_flat[C_PREV  *16 +: 16];
            h_fb_pos    <= sensor_value_flat[C_FBP   *16 +: 16];
            h_fb_neg    <= sensor_value_flat[C_FBN   *16 +: 16];
            h_pack_v    <= sensor_value_flat[C_PACKV *16 +: 16];
            h_dc_i      <= sensor_value_flat[C_DCI   *16 +: 16];
            h_cell_tmax <= sensor_value_flat[C_TMAX  *16 +: 16];
            h_encl_p    <= sensor_value_flat[C_ENCP  *16 +: 16];
            h_crash_f   <= sensor_value_flat[C_CRF   *16 +: 16];
            h_crash_s   <= sensor_value_flat[C_CRS   *16 +: 16];
            h_tip       <= sensor_value_flat[C_TIP   *16 +: 16];
            h_accel_long<= sensor_value_flat[C_ACLG  *16 +: 16];
            h_accel_lat <= sensor_value_flat[C_ACLT  *16 +: 16];

            // bit order matches the output declaration order above, LSB first
            h_dead      <= { sensor_dead[C_ACLT], sensor_dead[C_ACLG],
                             sensor_dead[C_TIP],  sensor_dead[C_CRS],
                             sensor_dead[C_CRF],  sensor_dead[C_ENCP],
                             sensor_dead[C_TMAX], sensor_dead[C_DCI],
                             sensor_dead[C_PACKV],sensor_dead[C_FBN],
                             sensor_dead[C_FBP],  sensor_dead[C_PREV],
                             sensor_dead[C_BUSV], sensor_dead[C_ISO],
                             sensor_dead[C_HVIL] };

            toggle_q    <= ~toggle_q;
        end
    end

    //=========================================================================
    // DESTINATION SIDE - synchronise the toggle, then take the whole snapshot
    //=========================================================================
    reg [2:0] tsync_q;
    wire      take = tsync_q[2] ^ tsync_q[1];

    always @(posedge clk_aon or negedge rst_hvsafe_n) begin
        if (!rst_hvsafe_n) begin
            tsync_q <= 3'b000;
        end else begin
            tsync_q <= {tsync_q[1:0], toggle_q};
        end
    end

    always @(posedge clk_aon or negedge rst_hvsafe_n) begin
        if (!rst_hvsafe_n) begin
            q_hvil       <= 16'd0;  q_iso       <= 16'd0;
            q_bus_v      <= 16'd0;  q_pre_v     <= 16'd0;
            q_fb_pos     <= 16'd0;  q_fb_neg    <= 16'd0;
            q_pack_v     <= 16'd0;  q_dc_i      <= 16'd0;
            q_cell_tmax  <= 16'd0;  q_encl_p    <= 16'd0;
            q_crash_f    <= 16'd0;  q_crash_s   <= 16'd0;
            q_tip        <= 16'd0;
            q_accel_long <= 16'd0;  q_accel_lat <= 16'd0;
            q_dead       <= 15'd0;
            q_valid      <= 1'b0;
        end else begin
            q_valid <= take;
            if (take) begin
                q_hvil       <= h_hvil;       q_iso       <= h_iso;
                q_bus_v      <= h_bus_v;      q_pre_v     <= h_pre_v;
                q_fb_pos     <= h_fb_pos;     q_fb_neg    <= h_fb_neg;
                q_pack_v     <= h_pack_v;     q_dc_i      <= h_dc_i;
                q_cell_tmax  <= h_cell_tmax;  q_encl_p    <= h_encl_p;
                q_crash_f    <= h_crash_f;    q_crash_s   <= h_crash_s;
                q_tip        <= h_tip;
                q_accel_long <= h_accel_long; q_accel_lat <= h_accel_lat;
                q_dead       <= h_dead;
            end
        end
    end

endmodule

`default_nettype wire
