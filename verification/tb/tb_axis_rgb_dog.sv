`timescale 1ns / 1ps
`default_nettype none

// DSim testbench for the DoG dual-kernel chain (Phase A, 2026-06-24):
//   in -> axis_rgb_conv3x3 (A, small Gaussian) ─┐
//      -> axis_rgb_conv5x5 (B, large Gaussian) ─┴-> axis_rgb_dog_combine -> out
// Verifies the PARALLEL branches stay spatially aligned through the ordinal FIFO and the
// combiner maths. 12x8 frame, horizontal bands 40/200.
//   - mode 1 (B passthrough) uniform -> 100  (B path through combiner)
//   - mode 0 (A passthrough) uniform -> 100  (FIFO-delayed A path delivers the right pixel)
//   - mode 2 (DoG, a=b=1, shift0, offset128) uniform -> 128 (flat DoG = 0 + offset)
//   - mode 2 on bands: flat regions stay ~128 (== alignment OK), the transition deviates.
// If A/B were misaligned, the flat regions would show a spurious DoG response.
module tb_axis_rgb_dog;
    localparam int W = 8;
    localparam int H = 12;

    logic clk = 0, rst_n = 0;
    logic [23:0] in_pixel = 0;
    logic in_valid = 0, in_sof = 0, in_eol = 0, in_eof = 0, in_err = 0;

    // branch A (3x3)
    logic [71:0]  a_coeffs = 0;  logic [3:0] a_shift = 0; logic a_en = 1;
    logic [23:0]  a_pixel;  logic a_valid, a_sof, a_eol, a_eof, a_err;
    // branch B (5x5)
    logic [199:0] b_coeffs = 0;  logic [3:0] b_shift = 0; logic b_en = 1;
    logic [23:0]  b_pixel;  logic b_valid, b_sof, b_eol, b_eof, b_err;
    // combiner
    logic [1:0]  cfg_mode = 2;
    logic [7:0]  cfg_alpha = 1, cfg_beta = 1;
    logic [3:0]  cfg_shift = 0;
    logic signed [8:0] cfg_offset = 128;
    logic [23:0] out_pixel;  logic out_valid, out_sof, out_eol, out_eof, out_err;

    always #5 clk = ~clk;

    axis_rgb_conv3x3 #(.LINE_PIXELS(W), .ENABLE(1'b1)) uA (
        .clk(clk), .rst_n(rst_n), .cfg_en(a_en), .cfg_coeffs(a_coeffs), .cfg_shift(a_shift), .cfg_abs(1'b0),
        .in_pixel(in_pixel), .in_valid(in_valid), .in_sof(in_sof),
        .in_eol(in_eol), .in_eof(in_eof), .in_err(in_err),
        .out_pixel(a_pixel), .out_valid(a_valid), .out_sof(a_sof),
        .out_eol(a_eol), .out_eof(a_eof), .out_err(a_err)
    );
    axis_rgb_conv5x5 #(.LINE_PIXELS(W), .ENABLE(1'b1)) uB (
        .clk(clk), .rst_n(rst_n), .cfg_en(b_en), .cfg_coeffs(b_coeffs), .cfg_shift(b_shift), .cfg_abs(1'b0),
        .in_pixel(in_pixel), .in_valid(in_valid), .in_sof(in_sof),
        .in_eol(in_eol), .in_eof(in_eof), .in_err(in_err),
        .out_pixel(b_pixel), .out_valid(b_valid), .out_sof(b_sof),
        .out_eol(b_eol), .out_eof(b_eof), .out_err(b_err)
    );
    axis_rgb_dog_combine #(.ENABLE(1'b1), .DEPTH(64)) uC (
        .clk(clk), .rst_n(rst_n),
        .cfg_mode(cfg_mode), .cfg_alpha(cfg_alpha), .cfg_beta(cfg_beta),
        .cfg_shift(cfg_shift), .cfg_offset(cfg_offset),
        .a_pixel(a_pixel), .a_valid(a_valid),
        .b_pixel(b_pixel), .b_valid(b_valid),
        .b_sof(b_sof), .b_eol(b_eol), .b_eof(b_eof), .b_err(b_err),
        .out_pixel(out_pixel), .out_valid(out_valid), .out_sof(out_sof),
        .out_eol(out_eol), .out_eof(out_eof), .out_err(out_err)
    );

    logic [7:0] out_r [0:H*W-1];
    integer ocnt = 0;
    always_ff @(posedge clk) begin
        if (out_valid) begin
            if (ocnt < H*W) out_r[ocnt] <= out_pixel[23:16];
            ocnt <= ocnt + 1;
        end
    end

    task automatic drive_frame(input byte unsigned line_val [0:H-1]);
        integer r, c;
        @(negedge clk);
        for (r = 0; r < H; r++) begin
            for (c = 0; c < W; c++) begin
                in_valid <= 1'b1;
                in_pixel <= {line_val[r], line_val[r], line_val[r]};
                in_sof   <= (r == 0 && c == 0);
                in_eol   <= (c == W-1);
                in_eof   <= (r == H-1 && c == W-1);
                @(negedge clk);
            end
        end
        in_valid <= 1'b0; in_sof <= 0; in_eol <= 0; in_eof <= 0;
        repeat (48) @(negedge clk);   // flush both convs + FIFO + combiner
    endtask

    function automatic int row_mean(input int rr);
        automatic int s = 0; automatic int n = 0; integer i;
        for (i = rr*W+2; i < rr*W+W-2 && i < ocnt; i++) begin s += out_r[i]; n++; end
        row_mean = (n > 0) ? s/n : -1;
    endfunction

    integer errors = 0;
    task automatic expect_near(input string nm, input int got, input int exp, input int tol);
        if (got < exp-tol || got > exp+tol) begin
            $display("  FAIL %s: got %0d exp %0d +/-%0d", nm, got, exp, tol); errors++;
        end
    endtask

    byte unsigned uni [0:H-1];
    byte unsigned bands [0:H-1];
    integer i, rr;

    initial begin
        for (i = 0; i < H; i++) uni[i]   = 8'd100;
        for (i = 0; i < H; i++) bands[i] = (i < 6) ? 8'd40 : 8'd200;

        // A = 3x3 Gaussian /16 ; B = 5x5 Gaussian /256
        a_coeffs = {8'd1,8'd2,8'd1, 8'd2,8'd4,8'd2, 8'd1,8'd2,8'd1}; a_shift = 4'd4;
        b_coeffs = {8'd1,8'd4,8'd6,8'd4,8'd1, 8'd4,8'd16,8'd24,8'd16,8'd4,
                    8'd6,8'd24,8'd36,8'd24,8'd6, 8'd4,8'd16,8'd24,8'd16,8'd4,
                    8'd1,8'd4,8'd6,8'd4,8'd1};                        b_shift = 4'd8;

        rst_n = 0; repeat (4) @(negedge clk); rst_n = 1; repeat (2) @(negedge clk);

        // warm up the (cold) line buffers so the first asserted frame is not contaminated
        cfg_mode = 2'd1; ocnt = 0; drive_frame(uni);

        // --- 1) mode 1 (B passthrough) uniform -> 100 ---
        cfg_mode = 2'd1; ocnt = 0; drive_frame(uni);
        $display("[mode1 B-pass uniform] got %0d outputs", ocnt);
        for (i = 5*W; i < (H-3)*W && i < ocnt; i++) expect_near("Bpass-uni", out_r[i], 100, 1);

        // --- 2) mode 0 (A passthrough, FIFO-aligned) uniform -> 100 ---
        cfg_mode = 2'd0; ocnt = 0; drive_frame(uni);
        $display("[mode0 A-pass uniform] got %0d outputs", ocnt);
        for (i = 5*W; i < (H-3)*W && i < ocnt; i++) expect_near("Apass-uni", out_r[i], 100, 1);

        // --- 3) mode 2 (DoG) uniform -> 128 (flat = 0 + offset) ---
        cfg_mode = 2'd2; ocnt = 0; drive_frame(uni);
        $display("[mode2 DoG uniform] got %0d outputs", ocnt);
        for (i = 5*W; i < (H-3)*W && i < ocnt; i++) expect_near("DoG-uni", out_r[i], 128, 1);

        // --- 4) mode 2 (DoG) on bands: a truly-flat row stays ~128 (alignment/quiet),
        //        the transition produces a strong edge response (offset-robust check) ---
        cfg_mode = 2'd2; ocnt = 0; drive_frame(bands);
        $display("[mode2 DoG bands 40/200] output row means (R):");
        begin
            automatic int mind = 999, maxd = 0;
            for (rr = 1; rr < H; rr++) begin
                automatic int d = row_mean(rr) - 128; if (d < 0) d = -d;
                $display("   out row %0d ~= %0d  (|DoG|=%0d)", rr, row_mean(rr), d);
                if (d < mind) mind = d;
                if (d > maxd) maxd = d;
            end
            if (mind > 3)  begin $display("  FAIL DoG-flat: flattest row |DoG|=%0d (>3)", mind); errors++; end
            if (maxd < 30) begin $display("  FAIL DoG-edge: strongest row |DoG|=%0d (<30)", maxd); errors++; end
        end

        // --- 5) compare A-pass vs B-pass on bands (different blur widths) ---
        cfg_mode = 2'd0; ocnt = 0; drive_frame(bands);
        $display("[mode0 A-pass bands] row means (3x3 blur):");
        for (rr = 2; rr < H-2; rr++) $display("   out row %0d ~= %0d", rr, row_mean(rr));
        cfg_mode = 2'd1; ocnt = 0; drive_frame(bands);
        $display("[mode1 B-pass bands] row means (5x5 blur, wider):");
        for (rr = 2; rr < H-2; rr++) $display("   out row %0d ~= %0d", rr, row_mean(rr));

        if (errors == 0) $display("TB_PASS: axis_rgb_dog (alignment + DoG/passthrough OK)");
        else             $display("TB_FAIL: %0d error(s)", errors);
        $finish;
    end

endmodule

`default_nettype wire
