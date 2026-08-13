//=============================================================================
// ivcu_fault_logger.v  -  the SRAM you can actually read back
//
//-----------------------------------------------------------------------------
// THE V3 DEFECT, WHICH WAS 47 % OF THE DIE
//
//     .rd_en   (1'b0),
//     .rd_addr (10'd0),
//
// Two OpenRAM 512x32 macros - 572,456 um2, forty-seven percent of the total
// area - wired up so that nothing could ever read them back.  Events were
// written faithfully into a memory that was, functionally, a hole.
//
// Half the silicon was a write-only memory and synthesis, STA and floorplan
// all completed without comment, because none of them can tell the difference
// between a memory and a memory nobody reads.
//
//-----------------------------------------------------------------------------
// ONE MACRO, NOT TWO, AND ONE PORT, NOT TWO
//
// 512 events is far more than any service interval needs, so the second macro
// is gone - that is 286,228 um2 recovered on its own.
//
// The macro has two ports and this design uses one.  Port 0 is read/write, so
// it can serve both the logger's writes and the MCU's reads, and using a
// single port makes it structurally impossible to read and write the same
// address in the same cycle.  The OpenRAM behavioural model prints a warning
// for exactly that case; designing it out is better than designing around it.
//
//-----------------------------------------------------------------------------
// CIRCULAR, NOT STOP-ON-FULL
//
// When the log fills it wraps and overwrites the oldest entry.  A technician
// investigating a vehicle wants the events leading up to the problem in front
// of them, not the first 512 things that happened when the car was new.
// wrap_count records how many times it has been round, so nobody mistakes a
// wrapped log for a short one.
//
//-----------------------------------------------------------------------------
// EVENT WORD, 32 BITS
//
//     [31:28]  event code       what kind of thing happened
//     [27:22]  sensor id        which channel, where that makes sense
//     [21:19]  status           the SS_* code at the time
//     [18:0]   timestamp        units of 100 ms since power-on
//
// 19 bits of 100 ms is 14.5 hours of continuous operation, which is longer
// than any single drive.  Absolute date and time live in the MCU; pairing this
// timestamp with the ignition lifetime counter in the register map places any
// event precisely enough for service.
//=============================================================================

`timescale 1ns/1ps
`default_nettype none

`include "ivcu_defs.vh"

module ivcu_fault_logger (
    input  wire        clk_mcu,
    input  wire        rst_mcu_n,

    //--- things worth recording, synchronised into clk_mcu ------------------
    input  wire [63:0] sensor_fault_s,
    input  wire [191:0] sensor_status_flat,
    input  wire [3:0]  hv_fault_code,
    input  wire        crash_latched,
    input  wire        pyro_fired,
    input  wire        permit_granted,     // any permit entered PS_ACTIVE
    input  wire        permit_expired,     // any permit entered PS_EXPIRED
    input  wire [1:0]  active_mode,
    input  wire        thermal_runaway_alarm,
    input  wire        iso_warn,

    //--- MCU read-back path - THE THING V3 DID NOT HAVE ---------------------
    input  wire        log_rd_en,
    input  wire [8:0]  log_rd_addr,
    output wire [31:0] log_rd_data,
    output reg  [8:0]  log_wr_ptr,
    output reg  [7:0]  log_wrap_count,

    //--- SRAM macro pins ------------------------------------------------------
    output wire        sram_clk0,
    output wire        sram_csb0,
    output wire        sram_web0,
    output wire [8:0]  sram_addr0,
    output wire [31:0] sram_din0,
    input  wire [31:0] sram_dout0
);

    //=========================================================================
    // TIMESTAMP - 100 ms ticks.  clk_mcu is 50 MHz, so 5,000,000 cycles.
    //=========================================================================
    localparam [22:0] TICK_CYCLES = 23'd5_000_000;

    reg [22:0] tick_div;
    reg [18:0] timestamp;

    always @(posedge clk_mcu or negedge rst_mcu_n) begin
        if (!rst_mcu_n) begin
            tick_div  <= 23'd0;
            timestamp <= 19'd0;
        end else if (tick_div >= (TICK_CYCLES - 23'd1)) begin
            tick_div <= 23'd0;
            if (timestamp != 19'h7FFFF) timestamp <= timestamp + 19'd1;
        end else begin
            tick_div <= tick_div + 23'd1;
        end
    end

    //=========================================================================
    // SCALAR EVENT EDGE DETECTION
    //=========================================================================
    reg [3:0] hv_prev;
    reg       crash_prev, pyro_prev, grant_prev, expire_prev, runaway_prev;
    reg       iso_prev;
    reg [1:0] mode_prev;

    wire ev_hv      = (hv_fault_code != hv_prev) && (hv_fault_code != `HVF_NONE);
    wire ev_crash   = crash_latched          & ~crash_prev;
    wire ev_pyro    = pyro_fired             & ~pyro_prev;
    wire ev_grant   = permit_granted         & ~grant_prev;
    wire ev_expire  = permit_expired         & ~expire_prev;
    wire ev_runaway = thermal_runaway_alarm  & ~runaway_prev;
    wire ev_iso     = iso_warn               & ~iso_prev;
    wire ev_mode    = (active_mode != mode_prev);

    wire scalar_pending = ev_hv | ev_crash | ev_pyro | ev_grant |
                          ev_expire | ev_runaway | ev_iso | ev_mode;

    //-------------------------------------------------------------------------
    // Scalar priority.  Worst first, because if two happen in the same cycle
    // the log should carry the one a technician most needs to see.
    //-------------------------------------------------------------------------
    wire [3:0] scalar_code =
          ev_crash   ? `EV_CRASH          :
          ev_pyro    ? `EV_PYRO_FIRED     :
          ev_runaway ? `EV_THERMAL_DERATE :
          ev_hv      ? `EV_HV_FAULT       :
          ev_expire  ? `EV_PERMIT_EXPIRED :
          ev_grant   ? `EV_PERMIT_GRANTED :
          ev_iso     ? `EV_ISOLATION_WARN :
                       `EV_MODE_CHANGE;

    //=========================================================================
    // SENSOR FAULT SCAN
    //
    // One channel per clock against a shadow copy.  A rising edge on any
    // channel is an event.  Scanning rather than encoding 64 edges in parallel
    // keeps this to one comparator, and a fault that waits 64 clocks - 1.3 us -
    // to be logged has lost nothing.
    //=========================================================================
    reg [63:0] fault_shadow;
    reg [6:0]  scan_idx;

    wire       scan_edge = sensor_fault_s[scan_idx[5:0]] &
                          ~fault_shadow[scan_idx[5:0]];

    // shift-add rather than scan_idx*3 - see the note in ivcu_health_scorer
    wire [7:0] scan_off  = {1'b0, scan_idx[5:0], 1'b0} + {2'b00, scan_idx[5:0]};
    wire [2:0] scan_stat = sensor_status_flat[scan_off +: 3];

    //=========================================================================
    // WRITE ARBITRATION
    //
    // Scalar events outrank the sensor scan: a crash must not queue behind
    // sixty-three channel comparisons.  The MCU read gets the port only when
    // there is nothing to write, which costs the MCU an occasional cycle and
    // costs the log nothing.
    //=========================================================================
    wire        do_write = scalar_pending | scan_edge;

    wire [3:0]  wr_code  = scalar_pending ? scalar_code : `EV_SENSOR_FAULT;
    wire [5:0]  wr_id    = scalar_pending ? 6'd0        : scan_idx[5:0];
    wire [2:0]  wr_stat  = scalar_pending ? {1'b0, active_mode} : scan_stat;

    wire [31:0] wr_word  = {wr_code, wr_id, wr_stat, timestamp};

    //=========================================================================
    // SRAM PORT 0 - write when there is an event, read otherwise
    //=========================================================================
    assign sram_clk0  = clk_mcu;
    assign sram_csb0  = ~(do_write | log_rd_en);      // active low
    assign sram_web0  = ~do_write;                    // low = write
    assign sram_addr0 = do_write ? log_wr_ptr : log_rd_addr;
    assign sram_din0  = wr_word;
    assign log_rd_data = sram_dout0;

    always @(posedge clk_mcu or negedge rst_mcu_n) begin
        if (!rst_mcu_n) begin
            log_wr_ptr     <= 9'd0;
            log_wrap_count <= 8'd0;
            fault_shadow   <= 64'd0;
            scan_idx       <= 7'd0;
            hv_prev        <= `HVF_NONE;
            crash_prev     <= 1'b0;
            pyro_prev      <= 1'b0;
            grant_prev     <= 1'b0;
            expire_prev    <= 1'b0;
            runaway_prev   <= 1'b0;
            iso_prev       <= 1'b0;
            mode_prev      <= `MODE_DETECT;
        end else begin

            //-----------------------------------------------------------------
            // advance the scan every cycle; it is independent of the write
            //-----------------------------------------------------------------
            scan_idx <= (scan_idx == 7'd63) ? 7'd0 : (scan_idx + 7'd1);

            // the shadow follows the live faults so a cleared fault can be
            // logged again if it returns
            fault_shadow[scan_idx[5:0]] <= sensor_fault_s[scan_idx[5:0]];

            //-----------------------------------------------------------------
            // scalar edge registers - only updated once the event has been
            // taken, so a scalar event cannot be lost to the scan
            //-----------------------------------------------------------------
            if (scalar_pending) begin
                hv_prev      <= hv_fault_code;
                crash_prev   <= crash_latched;
                pyro_prev    <= pyro_fired;
                grant_prev   <= permit_granted;
                expire_prev  <= permit_expired;
                runaway_prev <= thermal_runaway_alarm;
                iso_prev     <= iso_warn;
                mode_prev    <= active_mode;
            end

            //-----------------------------------------------------------------
            // advance the write pointer, wrapping
            //-----------------------------------------------------------------
            if (do_write) begin
                if (log_wr_ptr == 9'd511) begin
                    log_wr_ptr <= 9'd0;
                    if (log_wrap_count != 8'hFF) begin
                        log_wrap_count <= log_wrap_count + 8'd1;
                    end
                end else begin
                    log_wr_ptr <= log_wr_ptr + 9'd1;
                end
            end
        end
    end

endmodule

`default_nettype wire
