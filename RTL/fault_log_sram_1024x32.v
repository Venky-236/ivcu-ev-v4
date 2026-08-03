// fault_log_sram_1024x32.v
//
// ============================================================================
// 1024 x 32 dual-port storage, built from TWO OpenRAM 512 x 32 macros.
//
// WHY TWO MACROS
//   Sky130 OpenRAM cannot build a 1024-deep 1RW+1R array -- it fails the
//   bitcell row-mirroring rule required for LVS:
//       "sky130 currently requires rows to be even and to start with X
//        mirroring (left_rbl must be odd) for LVS."
//   512 x 32 1RW+1R does build. That is also the geometry of the standard
//   sky130_sram_2kbyte_1rw1r_32x512_8 macro, so it is a known-good shape.
//
//   Address bit [9] selects which half. Bits [8:0] address within a half.
//
// PORT LIST
//   Matches exactly the blackbox already instantiated in the synthesised
//   netlist (ivcu_ev_v3_hybrid_top_gate_full.v), so the main netlist does NOT
//   need re-synthesising. Only this wrapper does, and it is tiny.
//
// wmask0 IS DEPRECATED HERE
//   The blackbox declared wmask0[3:0] for byte-enables. The generated OpenRAM
//   macro has no byte mask -- writes are always full 32-bit words. The port is
//   kept so the netlist links, but it is intentionally unused. This is safe:
//   fault_logger_sram_32kb drives it to 4'b1111 (all bytes) on every write.
//   If byte-enables are ever genuinely needed, regenerate the macro with
//   write_size set in the OpenRAM config.
//
// READ LATENCY
//   The macro registers its address and presents data on dout one clock later.
//   The bank-select bit must therefore be delayed by one clock too, otherwise
//   the output mux would select using the NEW address while the data arriving
//   belongs to the OLD one. sel0_q / sel1_q below do exactly that.
//   Net latency through this wrapper is one cycle -- identical to what the
//   flop-array version gave, so fault_logger_sram_32kb needs no change.
//
// POWER PINS
//   The macro has vccd1/vssd1 behind `ifdef USE_POWER_PINS. They are not
//   connected here. OpenROAD connects macro power during PDN generation.
//   If you later run a flow that requires explicit power connections in the
//   netlist, add them here and define USE_POWER_PINS.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module fault_log_sram_1024x32 (
    // ---- port 0 : read / write ----
    input  wire        clk0,
    input  wire        csb0,      // active low chip select
    input  wire        web0,      // active low write enable
    input  wire [3:0]  wmask0,    // UNUSED -- see note above
    input  wire [9:0]  addr0,
    input  wire [31:0] din0,
    output wire [31:0] dout0,
    // ---- port 1 : read only ----
    input  wire        clk1,
    input  wire        csb1,      // active low chip select
    input  wire [9:0]  addr1,
    output wire [31:0] dout1
);

    // Silence "unused" lint on the deprecated byte mask without letting
    // synthesis build anything for it.
    wire _unused_wmask = &{1'b0, wmask0};

    wire [31:0] dout0_lo, dout0_hi;
    wire [31:0] dout1_lo, dout1_hi;

    // ------------------------------------------------------------------
    // Bank select, delayed one clock to line up with the macro read latency.
    // ------------------------------------------------------------------
    reg sel0_q;
    reg sel1_q;

    always @(posedge clk0) sel0_q <= addr0[9];
    always @(posedge clk1) sel1_q <= addr1[9];

    // ------------------------------------------------------------------
    // Lower half : addresses 0 .. 511   (addr[9] == 0)
    // csb is active low, so OR with addr[9] keeps this bank deselected
    // whenever the upper half is being addressed.
    // ------------------------------------------------------------------
    sram_512x32_2port u_bank_lo (
        .clk0  (clk0),
        .csb0  (csb0 |  addr0[9]),
        .web0  (web0),
        .addr0 (addr0[8:0]),
        .din0  (din0),
        .dout0 (dout0_lo),
        .clk1  (clk1),
        .csb1  (csb1 |  addr1[9]),
        .addr1 (addr1[8:0]),
        .dout1 (dout1_lo)
    );

    // ------------------------------------------------------------------
    // Upper half : addresses 512 .. 1023   (addr[9] == 1)
    // ------------------------------------------------------------------
    sram_512x32_2port u_bank_hi (
        .clk0  (clk0),
        .csb0  (csb0 | ~addr0[9]),
        .web0  (web0),
        .addr0 (addr0[8:0]),
        .din0  (din0),
        .dout0 (dout0_hi),
        .clk1  (clk1),
        .csb1  (csb1 | ~addr1[9]),
        .addr1 (addr1[8:0]),
        .dout1 (dout1_hi)
    );

    // ------------------------------------------------------------------
    // Output muxes, selected by the DELAYED bank bit.
    // ------------------------------------------------------------------
    assign dout0 = sel0_q ? dout0_hi : dout0_lo;
    assign dout1 = sel1_q ? dout1_hi : dout1_lo;

endmodule

`default_nettype wire
