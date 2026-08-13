//=============================================================================
// ivcu_reset_manager.v  -  reset distribution for all five domains
//
// THIS BLOCK DOES NOT GENERATE CLOCKS.
//
// All four clocks arrive as primary input pins.  There is no PLL, no divider
// and no clock gate anywhere in the IVCU-EV V4 RTL.  That is a deliberate
// choice, not an omission:
//
//   - V3 reached working synthesis and STA with four primary clock inputs.
//     Introducing on-chip division would hand OpenROAD's CTS a new class of
//     problem to solve in a project that has already spent effort fighting the
//     physical flow.  The cost of the decision is three extra pins out of 231.
//   - A divided clock built from a flop output is a generated clock that CTS
//     must balance against its parent.  Not worth it for a design whose fastest
//     requirement is a 10 us sensor sweep.
//
// FIVE RESET DOMAINS, and one of them is deliberately different:
//
//   rst_aon_n / rst_sensor_n / rst_ai_n / rst_mcu_n
//       assert on por_n OR ext_rst_n.  Normal system reset behaviour.
//
//   rst_hvsafe_n
//       asserts on por_n ONLY.  An external or soft reset must not clear the
//       HV island's latched state.  A welded contactor, a fired pyro fuse and
//       an isolation fault are facts about the physical vehicle: they survive
//       a reset, and they clear only on a real power cycle plus an
//       authenticated service-tool write.  If ext_rst_n could clear them, a
//       watchdog bite would silently re-arm a vehicle whose HV pack is
//       compromised.
//=============================================================================

`timescale 1ns / 1ps
`default_nettype none

module ivcu_reset_manager (
    // --- clocks (inputs only, passed through to the synchronisers) ----------
    input  wire clk_aon,        // 10 MHz  always-on, safety
    input  wire clk_sensor,     // 25 MHz  acquisition
    input  wire clk_ai,         // 50 MHz  AI cluster
    input  wire clk_mcu,        // 50 MHz  APB, logger, guidance

    // --- raw reset sources --------------------------------------------------
    input  wire por_n,          // power-on reset, active low
    input  wire ext_rst_n,      // external / watchdog reset, active low

    // --- synchronised, per-domain resets ------------------------------------
    output wire rst_aon_n,
    output wire rst_sensor_n,
    output wire rst_ai_n,
    output wire rst_mcu_n,
    output wire rst_hvsafe_n    // por_n only - see header
);

    //-------------------------------------------------------------------------
    // Combine the two sources for the ordinary domains.
    // Purely combinational AND: either source asserts reset immediately, with
    // no clock required.
    //-------------------------------------------------------------------------
    wire sys_rst_n;
    assign sys_rst_n = por_n & ext_rst_n;

    //-------------------------------------------------------------------------
    // One synchroniser per domain.  Async assert, synchronous de-assert.
    //-------------------------------------------------------------------------
    ivcu_reset_sync #(.STAGES(3)) u_rst_aon (
        .clk         (clk_aon),
        .async_rst_n (sys_rst_n),
        .sync_rst_n  (rst_aon_n)
    );

    ivcu_reset_sync #(.STAGES(3)) u_rst_sensor (
        .clk         (clk_sensor),
        .async_rst_n (sys_rst_n),
        .sync_rst_n  (rst_sensor_n)
    );

    ivcu_reset_sync #(.STAGES(3)) u_rst_ai (
        .clk         (clk_ai),
        .async_rst_n (sys_rst_n),
        .sync_rst_n  (rst_ai_n)
    );

    ivcu_reset_sync #(.STAGES(3)) u_rst_mcu (
        .clk         (clk_mcu),
        .async_rst_n (sys_rst_n),
        .sync_rst_n  (rst_mcu_n)
    );

    // HV safety island: power-on reset only.  ext_rst_n is deliberately not in
    // this path.
    ivcu_reset_sync #(.STAGES(3)) u_rst_hvsafe (
        .clk         (clk_aon),
        .async_rst_n (por_n),
        .sync_rst_n  (rst_hvsafe_n)
    );

endmodule

`default_nettype wire
