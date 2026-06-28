`timescale 1ns / 1ps
`default_nettype none

// DSim testbench for axis_rgb_conv3x3 (Phase 2b, 2026-06-23).
// Small 8x8 frame. Checks: (1) passthrough kernel = input centre pixel for a uniform
// frame; (2) Gaussian kernel = input for a uniform frame (uniform region unchanged);
// (3) Gaussian on a per-line ramp blurs vertically (printed for inspection + a spot
// assert at a band centre). Stream protocol: valid + sof/eol/eof markers.
module tb_axis_rgb_conv3x3;
    localparam int W = 8;          // pixels per line (LINE_PIXELS)
    localparam int H = 8;          // lines per frame

    logic clk = 0, rst_n = 0;
    logic        cfg_en = 0;
    logic [71:0] cfg_coeffs = 0;
    logic [3:0]  cfg_shift = 0;
    logic        cfg_abs = 0;
    logic [23:0] in_pixel = 0;
    logic in_valid = 0, in_sof = 0, in_eol = 0, in_eof = 0, in_err = 0;
    logic [23:0] out_pixel;
    logic out_valid, out_sof, out_eol, out_eof, out_err;

    always #5 clk = ~clk;          // 100 MHz

    axis_rgb_conv3x3 #(.LINE_PIXELS(W), .ENABLE(1'b1)) dut (
        .clk(clk), .rst_n(rst_n),
        .cfg_en(cfg_en), .cfg_coeffs(cfg_coeffs), .cfg_shift(cfg_shift), .cfg_abs(cfg_abs),
        .in_pixel(in_pixel), .in_valid(in_valid), .in_sof(in_sof),
        .in_eol(in_eol), .in_eof(in_eof), .in_err(in_err),
        .out_pixel(out_pixel), .out_valid(out_valid), .out_sof(out_sof),
        .out_eol(out_eol), .out_eof(out_eof), .out_err(out_err)
    );

    // collect outputs into a frame buffer (row-major), tracking output position
    logic [7:0] out_r [0:H*W-1];
    logic [7:0] out_g [0:H*W-1];
    integer ocnt = 0;
    always_ff @(posedge clk) begin
        if (out_valid) begin
            if (ocnt < H*W) begin
                out_r[ocnt] <= out_pixel[23:16];
                out_g[ocnt] <= out_pixel[15:8];
            end
            ocnt <= ocnt + 1;
        end
    end

    // drive one frame of gray pixels: val[row] applied to all cols (R=G=B=val)
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
        repeat (16) @(negedge clk);   // flush pipeline (programmable MAC has ~5 stages)
    endtask

    integer errors = 0;
    task automatic expect_eq(input string nm, input int got, input int exp);
        if (got !== exp) begin
            $display("  FAIL %s: got %0d exp %0d", nm, got, exp); errors++;
        end
    endtask

    byte unsigned uni [0:H-1];
    byte unsigned ramp [0:H-1];
    integer i, rr;

    initial begin
        for (i = 0; i < H; i++) uni[i]  = 8'd100;
        // horizontal bands: lines 0-3 = 40, lines 4-7 = 200
        for (i = 0; i < H; i++) ramp[i] = (i < 4) ? 8'd40 : 8'd200;

        rst_n = 0; repeat (4) @(negedge clk); rst_n = 1; repeat (2) @(negedge clk);

        // Gaussian kernel {1,2,1,2,4,2,1,2,1}/16: coeff[idx] at cfg_coeffs[idx*8 +: 8]
        cfg_coeffs = {8'd1,8'd2,8'd1, 8'd2,8'd4,8'd2, 8'd1,8'd2,8'd1};  // idx 8..0
        cfg_shift  = 4'd4;

        // --- 1) passthrough (cfg_en=0) on uniform frame -> all 100 ---
        cfg_en = 1'b0; ocnt = 0; drive_frame(uni);
        $display("[passthrough uniform] got %0d outputs", ocnt);
        for (i = 3*W; i < (H-1)*W && i < ocnt; i++) expect_eq("pass-uni", out_r[i], 100);

        // --- 2) Gaussian (cfg_en=1) on uniform frame -> all 100 (uniform unchanged) ---
        cfg_en = 1'b1; ocnt = 0; drive_frame(uni);
        $display("[gaussian uniform] got %0d outputs", ocnt);
        for (i = 3*W; i < (H-2)*W && i < ocnt; i++) expect_eq("gauss-uni", out_r[i], 100);

        // --- 3) Gaussian on horizontal bands -> vertical blur, print ---
        cfg_en = 1'b1; ocnt = 0; drive_frame(ramp);
        $display("[gaussian bands 40/200] output row means (R channel):");
        for (rr = 0; rr < H; rr++) begin
            automatic int s = 0; automatic int n = 0;
            for (i = rr*W+1; i < rr*W+W-1 && i < ocnt; i++) begin s += out_r[i]; n++; end
            if (n > 0) $display("   out row %0d ~= %0d", rr, s/n);
        end

        // --- 4) cfg_abs (gradient magnitude): Sobel-Y on a low-high-low band has a +grad
        // (rising) AND a -grad (falling) edge. Without abs the falling edge clips to 0; with
        // abs BOTH edges show -> more "bright edge" rows. Proves |grad| recovers both polarities.
        begin
            byte unsigned peak [0:H-1];
            automatic int bright0 = 0, bright1 = 0;
            for (i = 0; i < H; i++) peak[i] = (i >= 3 && i <= 4) ? 8'd200 : 8'd40;  // low-high-low
            // Sobel-Y = {top -1,-2,-1; mid 0; bot 1,2,1}; concat idx8..0
            cfg_coeffs = {8'd1,8'd2,8'd1, 8'd0,8'd0,8'd0, -8'sd1,-8'sd2,-8'sd1};
            cfg_shift = 4'd2; cfg_en = 1'b1;
            cfg_abs = 1'b0; ocnt = 0; drive_frame(peak);
            for (rr = 1; rr < H-1; rr++) begin automatic int s=0,n=0;
                for (i=rr*W+2;i<rr*W+W-2&&i<ocnt;i++) begin s+=out_r[i];n++; end
                if (n>0 && s/n > 80) bright0++; end
            cfg_abs = 1'b1; ocnt = 0; drive_frame(peak);
            for (rr = 1; rr < H-1; rr++) begin automatic int s=0,n=0;
                for (i=rr*W+2;i<rr*W+W-2&&i<ocnt;i++) begin s+=out_r[i];n++; end
                if (n>0 && s/n > 80) bright1++; end
            $display("[sobel-Y magnitude] bright edge rows: abs=0 -> %0d, abs=1 -> %0d", bright0, bright1);
            if (!(bright1 > bright0)) begin $display("  FAIL cfg_abs (expect more edges with abs)"); errors++; end
        end

        if (errors == 0) $display("TB_PASS: axis_rgb_conv3x3 (passthrough + gaussian + cfg_abs magnitude OK)");
        else             $display("TB_FAIL: %0d error(s)", errors);
        $finish;
    end

endmodule

`default_nettype wire
