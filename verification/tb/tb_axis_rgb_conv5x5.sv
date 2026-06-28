`timescale 1ns / 1ps
`default_nettype none

// DSim testbench for axis_rgb_conv5x5 (DoG dual-kernel, Phase A, 2026-06-24).
// Small 8x8 frame. Checks: (1) passthrough (cfg_en=0) = centre pixel for uniform; (2)
// identity kernel (centre tap=1) on uniform = input; (3) 5x5 Gaussian (separable
// [1,4,6,4,1] outer product, sum 256, shift 8) on uniform = input unchanged; (4) Gaussian
// on horizontal bands 40/200 -> vertical blur over ~5 rows (printed). Stream protocol:
// valid + sof/eol/eof markers. Mirrors tb_axis_rgb_conv3x3 with the deeper 6-stage pipe.
module tb_axis_rgb_conv5x5;
    localparam int W = 8;
    localparam int H = 8;

    logic clk = 0, rst_n = 0;
    logic         cfg_en = 0;
    logic [199:0] cfg_coeffs = 0;
    logic [3:0]   cfg_shift = 0;
    logic [23:0]  in_pixel = 0;
    logic in_valid = 0, in_sof = 0, in_eol = 0, in_eof = 0, in_err = 0;
    logic [23:0]  out_pixel;
    logic out_valid, out_sof, out_eol, out_eof, out_err;

    always #5 clk = ~clk;

    axis_rgb_conv5x5 #(.LINE_PIXELS(W), .ENABLE(1'b1)) dut (
        .clk(clk), .rst_n(rst_n),
        .cfg_en(cfg_en), .cfg_coeffs(cfg_coeffs), .cfg_shift(cfg_shift), .cfg_abs(1'b0),
        .in_pixel(in_pixel), .in_valid(in_valid), .in_sof(in_sof),
        .in_eol(in_eol), .in_eof(in_eof), .in_err(in_err),
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
        repeat (32) @(negedge clk);   // flush 6-stage pipe + 2-line fill
    endtask

    integer errors = 0;
    task automatic expect_eq(input string nm, input int got, input int exp);
        if (got !== exp) begin
            $display("  FAIL %s: got %0d exp %0d", nm, got, exp); errors++;
        end
    endtask

    byte unsigned uni [0:H-1];
    byte unsigned bands [0:H-1];
    integer i, rr;

    initial begin
        for (i = 0; i < H; i++) uni[i]   = 8'd100;
        for (i = 0; i < H; i++) bands[i] = (i < 4) ? 8'd40 : 8'd200;

        rst_n = 0; repeat (4) @(negedge clk); rst_n = 1; repeat (2) @(negedge clk);

        // --- 1) passthrough (cfg_en=0) on uniform -> 100 ---
        cfg_en = 1'b0; ocnt = 0; drive_frame(uni);
        $display("[passthrough uniform] got %0d outputs", ocnt);
        for (i = 3*W; i < (H-2)*W && i < ocnt; i++) expect_eq("pass-uni", out_r[i], 100);

        // --- 2) identity kernel (centre tap idx12 = 1, shift 0) on uniform -> 100 ---
        cfg_coeffs = '0; cfg_coeffs[12*8 +: 8] = 8'd1; cfg_shift = 4'd0;
        cfg_en = 1'b1; ocnt = 0; drive_frame(uni);
        $display("[identity uniform] got %0d outputs", ocnt);
        for (i = 3*W; i < (H-2)*W && i < ocnt; i++) expect_eq("ident-uni", out_r[i], 100);

        // --- 3) 5x5 Gaussian (sum 256, shift 8) on uniform -> 100 (unchanged) ---
        // outer([1,4,6,4,1]) row-major idx0..24, concat is {idx24..idx0}
        cfg_coeffs = {8'd1,8'd4,8'd6,8'd4,8'd1,
                      8'd4,8'd16,8'd24,8'd16,8'd4,
                      8'd6,8'd24,8'd36,8'd24,8'd6,
                      8'd4,8'd16,8'd24,8'd16,8'd4,
                      8'd1,8'd4,8'd6,8'd4,8'd1};
        cfg_shift = 4'd8;
        cfg_en = 1'b1; ocnt = 0; drive_frame(uni);
        $display("[gaussian uniform] got %0d outputs", ocnt);
        for (i = 3*W; i < (H-3)*W && i < ocnt; i++) expect_eq("gauss-uni", out_r[i], 100);

        // --- 4) Gaussian on horizontal bands 40/200 -> vertical blur, print ---
        cfg_en = 1'b1; ocnt = 0; drive_frame(bands);
        $display("[gaussian bands 40/200] output row means (R channel):");
        for (rr = 0; rr < H; rr++) begin
            automatic int s = 0; automatic int n = 0;
            for (i = rr*W+2; i < rr*W+W-2 && i < ocnt; i++) begin s += out_r[i]; n++; end
            if (n > 0) $display("   out row %0d ~= %0d", rr, s/n);
        end

        if (errors == 0) $display("TB_PASS: axis_rgb_conv5x5 (passthrough + identity + gaussian uniform OK)");
        else             $display("TB_FAIL: %0d error(s)", errors);
        $finish;
    end

endmodule

`default_nettype wire
