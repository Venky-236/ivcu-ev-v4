//=============================================================================
// ivcu_apb_regs.v  -  the register map, APB3, 32-bit
//
// FLAGGED CHANGE vs V3: APB3 instead of a 64-bit AXI.
//
// V3 carried a 64-bit AXI slave whose upper 32 bits of wdata were never read
// and whose rdata[63:32] was constant zero - 72 dead pins on a boundary that
// was already impossible to place.  A register block has no bursts and no
// outstanding transactions, so AXI buys nothing here.  APB3 does the same job
// in 83 pins instead of about 128.
//
//-----------------------------------------------------------------------------
// THE TWO REGISTERS THAT MATTER MOST
//
//   0x500 / 0x504   the fault log read path.  In V3 rd_en was tied to zero and
//                   half the die was a memory nobody could read.  These two
//                   registers are that defect, fixed.
//
//   0x100-0x1FF     one word per sensor: value, status, class, servicer, HV
//                   hazard, confidence.  The entire user-facing feature set -
//                   what it reads, how healthy it is, who may replace it, and
//                   whether it is safe to touch - in 256 bytes.
//                   V3 spent 118,908 um2 on 386 bits of report text that never
//                   reached a pin.
//
//-----------------------------------------------------------------------------
// AUTHENTICATED WRITES
//
// service_clear releases latched HV faults, a fired-pyro record, a crash latch
// and expired permits.  Those latches exist because the vehicle is not safe,
// so clearing them must not be something firmware does by accident.
//
// A clear requires the magic value below, written to the clear register.  It
// is a guard against a wild pointer or a stuck bus, not against a determined
// attacker - anyone who can write arbitrary values to arbitrary APB addresses
// already owns the MCU.  Real authentication belongs in the service tool and
// the MCU's secure boot, not in a 32-bit comparison on this die.  Saying so is
// better than implying this is security.
//
//-----------------------------------------------------------------------------
// bypass_request IS NOT HERE
//
// The architecture spec had a writable bypass request at 0x020.  It is gone.
// A permit is a rider consciously accepting a degraded vehicle; if firmware
// can grant one, a firmware bug can defeat a safety inhibit with nobody
// holding a button.  The only path is the three-second physical hold in
// ivcu_serviceability_mgr.  0x020 reads back which channels are bypassed.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_apb_regs (
    input  wire         clk_mcu,
    input  wire         rst_mcu_n,

    //--- APB3 slave, top-level pins ------------------------------------------
    input  wire [11:0]  paddr,
    input  wire         psel,
    input  wire         penable,
    input  wire         pwrite,
    input  wire [31:0]  pwdata,
    output reg  [31:0]  prdata,
    output wire         pready,
    output reg          pslverr,

    //--- sensor state, synchronised into clk_mcu ----------------------------
    input  wire [1023:0] sensor_value_flat,
    input  wire [191:0]  sensor_status_flat,
    input  wire [255:0]  sensor_conf_flat,
    input  wire [63:0]   sensor_enable_s,
    input  wire [63:0]   sensor_fault_s,
    input  wire [63:0]   bypass_active_s,

    //--- health ----------------------------------------------------------------
    input  wire [7:0]   system_health_score,
    input  wire [7:0]   score_battery,
    input  wire [7:0]   score_motor,
    input  wire [7:0]   score_thermal,
    input  wire [7:0]   score_dynamics,
    input  wire [7:0]   score_driver,
    input  wire [7:0]   score_safety,
    input  wire [7:0]   score_perception,

    //--- mode and safety --------------------------------------------------------
    input  wire [1:0]   active_mode,
    input  wire         mode_resolved,
    input  wire [7:0]   detect_sweeps,
    input  wire [2:0]   safety_state,
    input  wire [3:0]   inhibit_reason,
    input  wire         limp_home_active,

    //--- HV -----------------------------------------------------------------------
    input  wire [2:0]   hv_state,
    input  wire [3:0]   hv_fault_code,
    input  wire         hv_isolated,
    input  wire         pyro_fired,
    input  wire         hvil_ok,
    input  wire         iso_warn,
    input  wire         iso_fault,

    //--- permits --------------------------------------------------------------------
    input  wire [23:0]  permit_starts_flat,
    input  wire [15:0]  permit_state_flat,
    input  wire [31:0]  ign_lifetime,

    //--- emergency --------------------------------------------------------------------
    input  wire [2:0]   crash_severity,
    input  wire [2:0]   sos_route,
    input  wire         battery_incident,
    input  wire         gps_fix_valid,
    input  wire [15:0]  fix_age_ms,

    //--- guidance ----------------------------------------------------------------------
    input  wire [5:0]   guidance_sensor_id,
    input  wire [2:0]   guidance_status,
    input  wire [3:0]   guidance_action,
    input  wire [2:0]   guidance_starts_left,

    //--- fault log -----------------------------------------------------------------------
    output reg          log_rd_en,
    output reg  [8:0]   log_rd_addr,
    input  wire [31:0]  log_rd_data,
    input  wire [8:0]   log_wr_ptr,
    input  wire [7:0]   log_wrap_count,

    //--- control out -----------------------------------------------------------------------
    output reg          service_clear,
    output reg  [1:0]   mode_req,
    output reg          mode_req_valid
);

    // APB3 with no wait states.  Every register in this block responds in the
    // access phase, so there is nothing for pready to wait for.
    assign pready = 1'b1;

    localparam [31:0] SERVICE_KEY = 32'h5AFE_C0DE;

    //-------------------------------------------------------------------------
    // Address decode.  paddr[11:0] with 4-byte registers, so paddr[1:0] is
    // always zero for a legal access.
    //-------------------------------------------------------------------------
    wire        wr_en   = psel & penable & pwrite;
    wire        rd_en   = psel & ~pwrite;
    wire [11:0] a       = paddr;
    wire        in_sens = (a >= 12'h100) && (a <= 12'h1FF);
    wire [5:0]  sens_id = a[7:2];

    //=========================================================================
    // WRITES
    //=========================================================================
    always @(posedge clk_mcu or negedge rst_mcu_n) begin
        if (!rst_mcu_n) begin
            service_clear  <= 1'b0;
            mode_req       <= `MODE_CAR;
            mode_req_valid <= 1'b0;
            log_rd_en      <= 1'b0;
            log_rd_addr    <= 9'd0;
            pslverr        <= 1'b0;
        end else begin
            // service_clear is a one-cycle pulse: a level would leave the
            // latches permanently clearable
            service_clear  <= 1'b0;
            mode_req_valid <= 1'b0;
            pslverr        <= 1'b0;

            if (wr_en) begin
                case (a)

                    12'h008: begin      // control
                        mode_req       <= pwdata[1:0];
                        mode_req_valid <= pwdata[8];
                    end

                    12'h500: begin      // fault log read control
                        log_rd_en   <= pwdata[31];
                        log_rd_addr <= pwdata[8:0];
                    end

                    //---------------------------------------------------------
                    // authenticated clear - see the header on what this is and
                    // is not
                    //---------------------------------------------------------
                    12'h30C: begin
                        if (pwdata == SERVICE_KEY) begin
                            service_clear <= 1'b1;
                        end else begin
                            pslverr <= 1'b1;   // wrong key, refused, reported
                        end
                    end

                    default: begin
                        // writing to a read-only or unmapped address is a
                        // firmware bug and is reported rather than ignored
                        pslverr <= 1'b1;
                    end

                endcase
            end
        end
    end

    //=========================================================================
    // READS
    //=========================================================================
    localparam [255:0] ATTR_ROM = `SENSOR_ATTR_TABLE;

    // x16 and x4 are shifts and fold away.  x3 does not - written as a
    // shift-add so no $mul is inferred.  See ivcu_health_scorer for the note.
    wire [7:0]  sens_off3 = {1'b0, sens_id, 1'b0} + {2'b00, sens_id};

    wire [15:0] sens_val  = sensor_value_flat [sens_id*16 +: 16];
    wire [2:0]  sens_stat = sensor_status_flat[sens_off3  +: 3];
    wire [3:0]  sens_conf = sensor_conf_flat  [sens_id*4  +: 4];

    // Indexed part-select, NOT a variable shift.
    //   ATTR_ROM >> (sens_id*4)   builds a 256-bit barrel shifter
    //   ATTR_ROM[sens_id*4 +: 4]  builds four 64-to-1 muxes
    // Same result, and the second is roughly two orders of magnitude smaller.
    // This is the kind of one-character difference that quietly costs area,
    // which is how V3 ended up where it did.
    wire [3:0]  sens_attr = ATTR_ROM[sens_id*4 +: 4];

    always @(posedge clk_mcu or negedge rst_mcu_n) begin
        if (!rst_mcu_n) begin
            prdata <= 32'd0;
        end else if (rd_en) begin
            if (in_sens) begin
                //-------------------------------------------------------------
                // one word per sensor:
                //   [15:0]  value        [18:16] status
                //   [20:19] class        [21]    servicer
                //   [22]    hv hazard    [26:23] confidence
                //   [27]    enabled      [28]    faulted     [29] bypassed
                //-------------------------------------------------------------
                prdata <= { 2'd0,
                            bypass_active_s[sens_id],
                            sensor_fault_s [sens_id],
                            sensor_enable_s[sens_id],
                            sens_conf,
                            sens_attr[3],        // hv hazard
                            sens_attr[2],        // servicer
                            sens_attr[1:0],      // class
                            sens_stat,
                            sens_val };
            end else begin
                case (a)
                    12'h000: prdata <= `IVCU_ID_CODE;
                    12'h004: prdata <= {8'd0, `IVCU_VER_MAJOR,
                                              `IVCU_VER_MINOR,
                                              `IVCU_VER_PATCH};
                    12'h008: prdata <= {23'd0, mode_req_valid, 6'd0, mode_req};
                    //  [1:0] mode   [2] resolved     [5:3] safety state
                    //  [9:6] inhibit reason         [10]  limp home
                    // [18:11] detection sweeps
                    12'h00C: prdata <= {13'd0, detect_sweeps,
                                        limp_home_active, inhibit_reason,
                                        safety_state, mode_resolved,
                                        active_mode};
                    12'h010: prdata <= sensor_enable_s[31:0];
                    12'h014: prdata <= sensor_enable_s[63:32];
                    12'h018: prdata <= sensor_fault_s[31:0];
                    12'h01C: prdata <= sensor_fault_s[63:32];
                    12'h020: prdata <= bypass_active_s[31:0];   // read-only
                    12'h024: prdata <= bypass_active_s[63:32];  // read-only

                    12'h200: prdata <= {24'd0, system_health_score};
                    12'h204: prdata <= {score_dynamics, score_thermal,
                                        score_motor,    score_battery};
                    12'h208: prdata <= {8'd0, score_safety,
                                        score_perception, score_driver};

                    //  [2:0] hv state  [3] any fault   [4] isolated
                    //  [5] pyro fired  [6] hvil ok     [7] iso warn
                    //  [8] iso fault
                    12'h300: prdata <= {23'd0, iso_fault, iso_warn, hvil_ok,
                                        pyro_fired, hv_isolated,
                                        (hv_fault_code != `HVF_NONE),
                                        hv_state};
                    12'h304: prdata <= {28'd0, hv_fault_code};

                    12'h400: prdata <= {16'd0, permit_state_flat};
                    12'h404: prdata <= {8'd0, permit_starts_flat};
                    12'h40C: prdata <= ign_lifetime;

                    12'h500: prdata <= {log_rd_en, 22'd0, log_rd_addr};
                    12'h504: prdata <= log_rd_data;          // THE FIX
                    12'h508: prdata <= {15'd0, log_wrap_count, log_wr_ptr};

                    12'h600: prdata <= {16'd0, guidance_starts_left,
                                        guidance_action, guidance_status,
                                        guidance_sensor_id};

                    12'h700: prdata <= {8'd0, fix_age_ms, gps_fix_valid,
                                        battery_incident, sos_route,
                                        crash_severity};

                    default: prdata <= 32'hDEAD_0000 | {20'd0, a};
                endcase
            end
        end
    end

endmodule

`default_nettype wire
