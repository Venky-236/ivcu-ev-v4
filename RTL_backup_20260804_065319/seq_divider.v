// seq_divider.v
//
// ============================================================================
// Sequential restoring divider -- WIDTH bits, one quotient bit per clock.
//
// WHY THIS EXISTS
//   battery_predictive_ai_complete.v:297 and perception_health_ai_complete.v:402
//   each write a division with a genuinely variable divisor:
//
//       internal_resistance <= ((volt_history[0]-batt_pack_volt)*10) / current_avg;
//       time_to_collision   <= (object_distance * 16'd10) / object_relative_speed;
//
//   Yosys unrolls those into a fully combinational shift-subtract chain about
//   128 gates deep. Masked STA measured 35.170 ns and 30.542 ns of arrival time
//   against a 10 ns clock -- worst slack -25.548 and -20.936. Nothing downstream
//   of synthesis can fix that: cell resizing cannot recover 25 ns, and there is
//   no wire delay yet to remove.
//
//   Unlike the sensor fabric, these divisors are NOT secretly constants. They
//   are real runtime values -- R = V/I and distance/speed. No shift trick applies.
//   The chain has to become sequential.
//
// COST AND BENEFIT
//   Area:    ~200 gates, ONE instance per user, replacing thousands.
//   Timing:  worst combinational depth per cycle is one WIDTH+1 compare-subtract,
//            roughly 3-4 ns at WIDTH=32 on Sky130 HD. Fits 10 ns with margin.
//   Latency: WIDTH cycles. At 100 MHz and WIDTH=32 that is 320 ns.
//
//   320 ns is irrelevant for both users. Battery internal resistance changes
//   over minutes. For time-to-collision, a vehicle closing at 30 m/s travels
//   0.01 mm in 320 ns.
//
// OPERAND STABILITY
//   dividend and divisor are LATCHED on the start pulse. The caller does NOT
//   need to hold them stable for the 32 cycles. This is deliberate -- it is the
//   single most common integration bug with sequential dividers.
//
// DIVIDE BY ZERO
//   Returns all-ones (saturated), remainder zero. Both call sites already guard
//   against a zero divisor, so this is defence in depth rather than a behaviour
//   anyone should rely on.
//
// HANDSHAKE
//   start : assert for one cycle while busy is low. Ignored while busy.
//   busy  : high from acceptance until the cycle before done.
//   done  : one-cycle pulse. quotient/remainder are valid from that edge and
//           hold until the next division completes.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module seq_divider #(
    parameter integer WIDTH = 32
) (
    input  wire              clk,
    input  wire              rst_n,

    input  wire              start,
    input  wire [WIDTH-1:0]  dividend,
    input  wire [WIDTH-1:0]  divisor,

    output reg  [WIDTH-1:0]  quotient,
    output reg  [WIDTH-1:0]  remainder,
    output reg               busy,
    output reg               done
);

    localparam integer CNTW = $clog2(WIDTH + 1);

    // dvd_q shifts left each cycle; the quotient bits shift IN at the bottom,
    // so when the count expires dvd_q IS the quotient. This is the standard
    // in-place restoring formulation -- it avoids a separate quotient register.
    reg [WIDTH-1:0] dvd_q;
    reg [WIDTH:0]   rem_q;
    reg [WIDTH-1:0] dsr_q;
    reg [CNTW-1:0]  cnt_q;
    reg             div0_q;

    // One compare-subtract. This is the entire combinational path per cycle.
    wire [WIDTH:0] rem_shifted = {rem_q[WIDTH-1:0], dvd_q[WIDTH-1]};
    wire [WIDTH:0] rem_sub     = rem_shifted - {1'b0, dsr_q};
    wire           fits        = (rem_shifted >= {1'b0, dsr_q});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dvd_q     <= {WIDTH{1'b0}};
            rem_q     <= {(WIDTH+1){1'b0}};
            dsr_q     <= {WIDTH{1'b0}};
            cnt_q     <= {CNTW{1'b0}};
            div0_q    <= 1'b0;
            busy      <= 1'b0;
            done      <= 1'b0;
            quotient  <= {WIDTH{1'b0}};
            remainder <= {WIDTH{1'b0}};
        end else begin
            done <= 1'b0;                      // done is a one-cycle pulse

            if (start && !busy) begin
                dvd_q  <= dividend;            // operands latched here
                dsr_q  <= divisor;
                rem_q  <= {(WIDTH+1){1'b0}};
                cnt_q  <= WIDTH[CNTW-1:0];
                div0_q <= (divisor == {WIDTH{1'b0}});
                busy   <= 1'b1;

            end else if (busy) begin
                if (cnt_q == {CNTW{1'b0}}) begin
                    busy      <= 1'b0;
                    done      <= 1'b1;
                    quotient  <= div0_q ? {WIDTH{1'b1}} : dvd_q;
                    remainder <= div0_q ? {WIDTH{1'b0}} : rem_q[WIDTH-1:0];
                end else begin
                    dvd_q <= {dvd_q[WIDTH-2:0], fits};
                    rem_q <= fits ? rem_sub : rem_shifted;
                    cnt_q <= cnt_q - {{(CNTW-1){1'b0}}, 1'b1};
                end
            end
        end
    end

endmodule

`default_nettype wire