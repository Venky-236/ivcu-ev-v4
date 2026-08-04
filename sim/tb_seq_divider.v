// tb_seq_divider.v
//
// Self-checking testbench for seq_divider. Compares every result against
// Verilog's built-in / and % operators, which the simulator computes exactly.
//
//   cd ~/final_ivcu_project
//   iverilog -g2012 -o /tmp/tb_div RTL/seq_divider.v sim/tb_seq_divider.v
//   vvp /tmp/tb_div
//
// Want: "PASS -- 0 mismatches in N vectors"
// Run this BEFORE integrating into battery / perception. If the divider itself
// is wrong, debugging it inside a 125k-cell design is not a good time.

`timescale 1ns/1ps

module tb_seq_divider;

    localparam integer W = 32;

    reg              clk = 1'b0;
    reg              rst_n = 1'b0;
    reg              start = 1'b0;
    reg  [W-1:0]     a = {W{1'b0}};
    reg  [W-1:0]     b = {W{1'b0}};
    wire [W-1:0]     q, r;
    wire             busy, done;

    integer          i;
    integer          errors = 0;
    integer          vectors = 0;
    reg  [W-1:0]     exp_q, exp_r;

    always #5 clk = ~clk;          // 100 MHz

    seq_divider #(.WIDTH(W)) dut (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (start),
        .dividend  (a),
        .divisor   (b),
        .quotient  (q),
        .remainder (r),
        .busy      (busy),
        .done      (done)
    );

    task run_one (input [W-1:0] aa, input [W-1:0] bb);
    begin
        @(negedge clk);
        a = aa; b = bb; start = 1'b1;
        @(negedge clk);
        start = 1'b0;

        // Deliberately scribble on the inputs while the divider runs. The
        // module latches its operands on start, so this MUST NOT matter.
        // If it does, the latching is broken and this test will catch it.
        a = ~aa; b = ~bb;

        while (done !== 1'b1) @(posedge clk);

        vectors = vectors + 1;
        if (bb == {W{1'b0}}) begin
            exp_q = {W{1'b1}};
            exp_r = {W{1'b0}};
        end else begin
            exp_q = aa / bb;
            exp_r = aa % bb;
        end

        if (q !== exp_q || r !== exp_r) begin
            errors = errors + 1;
            $display("FAIL  %0d / %0d  ->  q=%0d (exp %0d)  r=%0d (exp %0d)",
                     aa, bb, q, exp_q, r, exp_r);
        end
    end
    endtask

    initial begin
        rst_n = 1'b0;
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        // --- directed cases -------------------------------------------------
        run_one(32'd0,          32'd1);          // 0 / x
        run_one(32'd1,          32'd1);          // identity
        run_one(32'd100,        32'd10);         // exact
        run_one(32'd101,        32'd10);         // with remainder
        run_one(32'd7,          32'd9);          // divisor > dividend
        run_one(32'hFFFF_FFFF,  32'd1);          // max / 1
        run_one(32'hFFFF_FFFF,  32'hFFFF_FFFF);  // max / max
        run_one(32'd12345,      32'd0);          // divide by zero -> all ones

        // --- the two real call sites, realistic operand ranges ---------------
        // battery:297   (dV * 10) / current_avg
        run_one(32'd3700 * 32'd10,  32'd150);
        run_one(32'd12   * 32'd10,  32'd101);
        // perception:402  (object_distance * 10) / object_relative_speed
        run_one(32'd8000 * 32'd10,  32'd250);
        run_one(32'd1    * 32'd10,  32'd65535);

        // --- random sweep ---------------------------------------------------
        for (i = 0; i < 300; i = i + 1) begin
            a     = {$random};
            b     = {$random};
            // half the vectors use a 16-bit divisor, which is what both call
            // sites actually produce
            if (i[0]) b = b & 32'h0000_FFFF;
            if (b == {W{1'b0}}) b = 32'd1;
            run_one(a, b);
        end

        $display("");
        if (errors == 0)
            $display("PASS -- 0 mismatches in %0d vectors", vectors);
        else
            $display("FAILED -- %0d mismatches in %0d vectors", errors, vectors);
        $display("");
        $finish;
    end

    // safety net so a broken handshake cannot hang the run forever
    initial begin
        #2_000_000;
        $display("TIMEOUT -- divider never asserted done. Check the handshake.");
        $finish;
    end

endmodule