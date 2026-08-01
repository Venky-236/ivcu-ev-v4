// fault_logger_sram_32kb.sv
//
// ============================================================================
// SRAM MACRO VERSION
// ----------------------------------------------------------------------------
// Behaviour is unchanged from the flop-array version. What changed is WHERE the
// 1024x32 storage lives:
//
//   `ifdef SYNTHESIS   -> storage is an instance of fault_log_sram_1024x32,
//                         declared as a Yosys blackbox at the bottom of this
//                         file. Yosys emits an instance and synthesises nothing
//                         inside it, so the ~32,857 inferred flip-flops that
//                         dominated the previous run disappear entirely.
//
//   else               -> storage is the original behavioural array, so
//                         simulation and cosimulation/equivalence checking work
//                         exactly as before with no testbench changes.
//
// The synthesis script already passes -DSYNTHESIS when reading this file, so no
// script change is needed for the switch to take effect.
//
// READ LATENCY IS PRESERVED. This is worth understanding rather than trusting:
//
//   Original:  at edge N, fault_memory[rd_addr] is read combinationally using
//              the pre-edge-N address, and the result is loaded into rd_data.
//              rd_data is therefore valid immediately after edge N.
//
//   Macro:     addr1 is sampled at edge N, and dout1 presents mem[addr1] after
//              edge N. Identical timing.
//
// The only pieces that needed re-aligning are the two qualifiers -- the
// (rd_en && init_done) gate and the (entry_count > 0) empty check. Both were
// evaluated pre-edge-N in the original, so they are registered by one cycle
// here (rd_active_q / rd_empty_q) to line up with dout1. rd_data is then a mux
// rather than a flop, which keeps the cycle-by-cycle behaviour bit-identical to
// the flop-array version -- important, because it means equivalence checking
// against the original should pass with zero mismatches.
//
// TRADE-OFF: rd_data is now a combinational output driven from the macro's
// dout1 rather than a registered output. That places a comb path from SRAM
// output to module port. On this diagnostic read path there is a full cycle to
// settle, so it should be fine, but flag it if STA later shows it as critical.
// If you would rather have a clean registered boundary, see the alternative
// marked ALT-REGISTERED below -- it costs one extra cycle of read latency and
// would require re-checking whoever consumes rd_data in the top module.
//
// STILL TO DO BEFORE P&R:
//   fault_log_sram_1024x32 is currently an abstract 1024x32 macro. Sky130
//   OpenRAM does not offer that geometry directly -- the nearest standard parts
//   are 32x256 and 32x512. Implementing this as 2 x sky130_sram_2kbyte_1rw1r_
//   32x512_8 with address-bit-9 selecting between them is the expected route.
//   That wrapper goes INSIDE fault_log_sram_1024x32, so this file does not
//   change again. You will also need the macro .lib and .lef for OpenROAD.
// ============================================================================

`timescale 1ns/1ps
`include "defines_ivcu_ev_v3.sv"
`default_nettype none

module fault_logger_sram_32kb (
    input  wire        clk_aon,
    input  wire        rst_aon_n,
    input  wire        wr_en,
    input  wire [9:0]  wr_addr,
    input  wire [31:0] wr_data,
    input  wire        rd_en,
    input  wire [9:0]  rd_addr,
    output wire [31:0] rd_data,
    input  wire [7:0]  fault_code,
    input  wire [31:0] timestamp,
    input  wire [15:0] sensor_data_0,
    input  wire [15:0] sensor_data_1,
    input  wire [15:0] sensor_data_2,
    input  wire [15:0] sensor_data_3,
    output reg         log_full,
    output reg         log_overflow,
    output reg  [9:0]  log_count,
    input  wire        log_clear,
    input  wire        log_read_only,
    input  wire [2:0]  debug_mode
);

    localparam MEM_DEPTH = 1024;

    reg [9:0]  write_pointer;
    reg [9:0]  read_pointer;
    reg [9:0]  oldest_entry;
    reg [10:0] entry_count;
    reg        init_done;
    reg [9:0]  init_addr;
    reg [2:0]  init_state;

`ifndef SYNTHESIS
    // Simulation-only storage. Absent under SYNTHESIS so Yosys never sees a
    // memory to infer flops from.
    reg [31:0] fault_memory [0:MEM_DEPTH-1];
`endif

    // ------------------------------------------------------------------------
    // Control FSM -- identical to the original in both modes. The only
    // difference is that the array writes are compiled out under SYNTHESIS,
    // where the macro handles storage instead.
    // ------------------------------------------------------------------------
    always_ff @(posedge clk_aon or negedge rst_aon_n) begin
        if (!rst_aon_n) begin
            write_pointer <= 10'd0;
            read_pointer <= 10'd0;
            oldest_entry <= 10'd0;
            entry_count <= 11'd0;
            log_full <= 1'b0;
            log_overflow <= 1'b0;
            log_count <= 10'd0;
            init_done <= 1'b0;
            init_addr <= 10'd0;
            init_state <= 3'd0;
        end else begin
            if (!init_done) begin
                case (init_state)
                    0: begin
`ifndef SYNTHESIS
                        fault_memory[init_addr] <= 32'd0;
`endif
                        init_addr <= init_addr + 1;
                        if (init_addr == MEM_DEPTH-1) init_state <= 1;
                    end
                    1: begin
                        init_done <= 1'b1;
                        init_state <= 2;
                    end
                    default: ;
                endcase
            end else begin
                // Normal operation
                if (log_clear) begin
                    write_pointer <= 10'd0;
                    read_pointer <= 10'd0;
                    oldest_entry <= 10'd0;
                    entry_count <= 11'd0;
                    log_full <= 1'b0;
                    log_overflow <= 1'b0;
                    log_count <= 10'd0;
                end else begin
                    if (wr_en && !log_read_only) begin
                        if (entry_count < MEM_DEPTH) begin
`ifndef SYNTHESIS
                            fault_memory[write_pointer] <= wr_data;
`endif
                            write_pointer <= write_pointer + 1;
                            entry_count <= entry_count + 1;
                            log_count <= entry_count[9:0] + 1;
                            if (entry_count == MEM_DEPTH-1) log_full <= 1'b1;
                        end else begin
`ifndef SYNTHESIS
                            fault_memory[oldest_entry] <= wr_data;
`endif
                            oldest_entry <= oldest_entry + 1;
                            read_pointer <= read_pointer + 1;
                            log_overflow <= 1'b1;
                        end
                    end
                end
            end
        end
    end

`ifdef SYNTHESIS
    // ========================================================================
    // SYNTHESIS PATH -- storage is the SRAM macro
    // ========================================================================

    // ---- write port control (port 0) ---------------------------------------
    // Mirrors exactly the three array-write sites compiled out above.
    reg        mem_we;
    reg [9:0]  mem_waddr;
    reg [31:0] mem_wdata;

    always @(*) begin
        mem_we    = 1'b0;
        mem_waddr = 10'd0;
        mem_wdata = 32'd0;
        if (!init_done) begin
            if (init_state == 3'd0) begin
                mem_we    = 1'b1;              // init clear
                mem_waddr = init_addr;
                mem_wdata = 32'd0;
            end
        end else if (!log_clear && wr_en && !log_read_only) begin
            mem_we    = 1'b1;
            mem_wdata = wr_data;
            mem_waddr = (entry_count < MEM_DEPTH) ? write_pointer : oldest_entry;
        end
    end

    // ---- read qualifier alignment (see header note) ------------------------
    // These two carry the pre-edge values forward one cycle so they line up
    // with dout1. No reset, matching the original read block which had none.
    reg rd_active_q;
    reg rd_empty_q;

    always_ff @(posedge clk_aon) begin
        rd_active_q <= rd_en && init_done;
        rd_empty_q  <= (entry_count == 11'd0);
    end

    // ---- macro instance ----------------------------------------------------
    wire [31:0] sram_dout0_unused;
    wire [31:0] sram_dout1;

    fault_log_sram_1024x32 u_fault_log_sram (
        // port 0: read/write -- used for writes only
        .clk0   (clk_aon),
        .csb0   (~mem_we),          // active low chip select
        .web0   (~mem_we),          // active low write enable
        .wmask0 (4'b1111),          // full 32-bit word writes
        .addr0  (mem_waddr),
        .din0   (mem_wdata),
        .dout0  (sram_dout0_unused),
        // port 1: read only
        .clk1   (clk_aon),
        .csb1   (~(rd_en && init_done)),
        .addr1  (rd_addr),
        .dout1  (sram_dout1)
    );

    // ---- rd_data reconstruction -------------------------------------------
    // Hold register captures the value whenever a read completes; the output
    // mux presents dout1 in the completing cycle. Net effect is bit-identical
    // to the original registered rd_data.
    reg [31:0] rd_data_hold;

    always_ff @(posedge clk_aon) begin
        if (rd_active_q) rd_data_hold <= rd_empty_q ? 32'hFFFFFFFF : sram_dout1;
    end

    assign rd_data = rd_active_q ? (rd_empty_q ? 32'hFFFFFFFF : sram_dout1)
                                 : rd_data_hold;

    // ---- ALT-REGISTERED ----------------------------------------------------
    // If you prefer a registered output boundary over preserved latency,
    // delete the two blocks above and use this instead. Costs +1 cycle on
    // rd_data; consumers in the top module must tolerate it.
    //
    //   reg [31:0] rd_data_r;
    //   always_ff @(posedge clk_aon) begin
    //       if (rd_active_q) rd_data_r <= rd_empty_q ? 32'hFFFFFFFF : sram_dout1;
    //   end
    //   assign rd_data = rd_data_r;

`else
    // ========================================================================
    // SIMULATION PATH -- original behavioural read, unchanged
    // ========================================================================
    reg [31:0] rd_data_r;

    always_ff @(posedge clk_aon) begin
        if (rd_en && init_done) begin
            if (entry_count > 0) rd_data_r <= fault_memory[rd_addr];
            else rd_data_r <= 32'hFFFFFFFF;
        end
    end

    assign rd_data = rd_data_r;
`endif

endmodule

`ifdef SYNTHESIS
// ============================================================================
// Blackbox declaration for the storage macro.
//
// Port naming follows the Sky130 OpenRAM 1rw1r convention (clk0/csb0/web0/
// wmask0/addr0/din0/dout0 + clk1/csb1/addr1/dout1) so that the eventual
// OpenRAM-generated wrapper drops in without touching the instantiation above.
//
// The (* blackbox *) attribute tells Yosys: this module has no contents, do not
// try to synthesise it, just emit the instance. That is what removes ~32,857
// flip-flops from the netlist.
//
// Before P&R this must be replaced by a real implementation -- most likely
// 2 x sky130_sram_2kbyte_1rw1r_32x512_8 with addr[9] selecting the half --
// together with its .lib and .lef for OpenROAD.
// ============================================================================
(* blackbox *)
module fault_log_sram_1024x32 (
    input  wire        clk0,
    input  wire        csb0,
    input  wire        web0,
    input  wire [3:0]  wmask0,
    input  wire [9:0]  addr0,
    input  wire [31:0] din0,
    output wire [31:0] dout0,
    input  wire        clk1,
    input  wire        csb1,
    input  wire [9:0]  addr1,
    output wire [31:0] dout1
);
endmodule
`endif

`default_nettype wire
