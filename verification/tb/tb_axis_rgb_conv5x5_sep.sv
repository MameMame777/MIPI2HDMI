`timescale 1ns / 1ps
`default_nettype none

// DSim testbench for axis_rgb_conv5x5_sep (cascade multi-scale, Phase A, 2026-06-24).
// 8x8 frame. Checks: (1) identity kernel (h=v={0,0,1,0,0}) on uniform = input (bypass);
// (2) separable Gaussian h=v=[1,4,6,4,1] (hshift4/vshift4) on uniform = input unchanged;
// (3) the separable Gaussian MATCHES the general 5x5 Gaussian (outer product, sum 256,
// shift 8) within +/-2 LSB (split-shift requantise) on horizontal bands -- proving h(x)v
// == the full kernel; (4) prints band row means (vertical blur). Both DUTs fed the same
// stream; their k-th outputs are the same spatial pixel (1:1 with input).
module tb_axis_rgb_conv5x5_sep;
    localparam int W = 8;
    localparam int H = 8;

    logic clk = 0, rst_n = 0;
    logic [39:0] cfg_h = 0, cfg_v = 0;
    logic [3:0]  cfg_hshift = 0, cfg_vshift = 0;
    logic [23:0] in_pixel = 0;
    logic in_valid = 0, in_sof = 0, in_eol = 0, in_eof = 0, in_err = 0;

    logic [23:0] sep_pixel;  logic sep_valid, sep_sof, sep_eol, sep_eof, sep_err;
    logic [199:0] gcoef = 0;
    logic [23:0] gen_pixel;  logic gen_valid, gen_sof, gen_eol, gen_eof, gen_err;

    always #5 clk = ~clk;

    axis_rgb_conv5x5_sep #(.LINE_PIXELS(W), .ENABLE(1'b1)) dut (
        .clk(clk), .rst_n(rst_n), .cfg_h(cfg_h), .cfg_v(cfg_v),
        .cfg_hshift(cfg_hshift), .cfg_vshift(cfg_vshift),
        .in_pixel(in_pixel), .in_valid(in_valid), .in_sof(in_sof),
        .in_eol(in_eol), .in_eof(in_eof), .in_err(in_err),
        .out_pixel(sep_pixel), .out_valid(sep_valid), .out_sof(sep_sof),
        .out_eol(sep_eol), .out_eof(sep_eof), .out_err(sep_err)
    );
    axis_rgb_conv5x5 #(.LINE_PIXELS(W), .ENABLE(1'b1)) gen (
        .clk(clk), .rst_n(rst_n), .cfg_en(1'b1), .cfg_coeffs(gcoef), .cfg_shift(4'd8), .cfg_abs(1'b0),
        .in_pixel(in_pixel), .in_valid(in_valid), .in_sof(in_sof),
        .in_eol(in_eol), .in_eof(in_eof), .in_err(in_err),
        .out_pixel(gen_pixel), .out_valid(gen_valid), .out_sof(gen_sof),
        .out_eol(gen_eol), .out_eof(gen_eof), .out_err(gen_err)
    );

    logic [7:0] sep_r [0:H*W-1];  integer sc_ = 0;
    logic [7:0] gen_r [0:H*W-1];  integer gc_ = 0;
    logic [23:0] sep_px [0:H*W-1];
    logic [23:0] gen_px [0:H*W-1];
    always_ff @(posedge clk) begin
        if (sep_valid) begin if (sc_<H*W) begin sep_r[sc_]<=sep_pixel[23:16]; sep_px[sc_]<=sep_pixel; end sc_<=sc_+1; end
        if (gen_valid) begin if (gc_<H*W) begin gen_r[gc_]<=gen_pixel[23:16]; gen_px[gc_]<=gen_pixel; end gc_<=gc_+1; end
    end

    // colour drive: per-channel line values (R,G,B differ -> tests channel independence)
    task automatic drive_color(input byte unsigned rv[0:H-1], input byte unsigned gv[0:H-1],
                               input byte unsigned bv[0:H-1]);
        integer r, c;
        @(negedge clk);
        for (r=0;r<H;r++) for (c=0;c<W;c++) begin
            in_valid<=1; in_pixel<={rv[r],gv[r],bv[r]};
            in_sof<=(r==0&&c==0); in_eol<=(c==W-1); in_eof<=(r==H-1&&c==W-1);
            @(negedge clk);
        end
        in_valid<=0; in_sof<=0; in_eol<=0; in_eof<=0;
        repeat (32) @(negedge clk);
    endtask

    task automatic drive_frame(input byte unsigned line_val [0:H-1]);
        integer r, c;
        @(negedge clk);
        for (r=0;r<H;r++) for (c=0;c<W;c++) begin
            in_valid<=1; in_pixel<={line_val[r],line_val[r],line_val[r]};
            in_sof<=(r==0&&c==0); in_eol<=(c==W-1); in_eof<=(r==H-1&&c==W-1);
            @(negedge clk);
        end
        in_valid<=0; in_sof<=0; in_eol<=0; in_eof<=0;
        repeat (32) @(negedge clk);
    endtask

    integer errors = 0;
    task automatic expect_near(input string nm, input int got, input int exp, input int tol);
        if (got < exp-tol || got > exp+tol) begin
            $display("  FAIL %s: got %0d exp %0d +/-%0d", nm, got, exp, tol); errors++; end
    endtask

    byte unsigned uni [0:H-1];
    byte unsigned bands [0:H-1];
    integer i, rr;

    initial begin
        for (i=0;i<H;i++) uni[i]=8'd100;
        for (i=0;i<H;i++) bands[i]=(i<4)?8'd40:8'd200;
        // general 5x5 Gaussian (outer of [1,4,6,4,1], sum 256), shift 8
        gcoef = {8'd1,8'd4,8'd6,8'd4,8'd1, 8'd4,8'd16,8'd24,8'd16,8'd4,
                 8'd6,8'd24,8'd36,8'd24,8'd6, 8'd4,8'd16,8'd24,8'd16,8'd4,
                 8'd1,8'd4,8'd6,8'd4,8'd1};

        rst_n=0; repeat(4) @(negedge clk); rst_n=1; repeat(2) @(negedge clk);

        // warm up the cold line buffers (1 throwaway frame) so frame 1 is not contaminated
        cfg_h={8'd0,8'd0,8'd1,8'd0,8'd0}; cfg_v={8'd0,8'd0,8'd1,8'd0,8'd0};
        cfg_hshift=0; cfg_vshift=0;
        sc_=0; gc_=0; drive_frame(uni);

        // 1) identity (bypass) on uniform -> 100
        cfg_h={8'd0,8'd0,8'd1,8'd0,8'd0}; cfg_v={8'd0,8'd0,8'd1,8'd0,8'd0};
        cfg_hshift=0; cfg_vshift=0;
        sc_=0; gc_=0; drive_frame(uni);
        $display("[identity uniform] sep got %0d", sc_);
        for (i=3*W;i<(H-2)*W&&i<sc_;i++) expect_near("ident-uni", sep_r[i], 100, 0);

        // 2) separable Gaussian on uniform -> 100
        cfg_h={8'd1,8'd4,8'd6,8'd4,8'd1}; cfg_v={8'd1,8'd4,8'd6,8'd4,8'd1};
        cfg_hshift=4; cfg_vshift=4;
        sc_=0; gc_=0; drive_frame(uni);
        $display("[gaussian uniform] sep got %0d", sc_);
        for (i=3*W;i<(H-3)*W&&i<sc_;i++) expect_near("gauss-uni", sep_r[i], 100, 1);

        // 3) separable == general 5x5 Gaussian on bands (within +/-2)
        sc_=0; gc_=0; drive_frame(bands);
        $display("[sep vs general gaussian bands] row means: sep | gen");
        for (rr=0; rr<H; rr++) begin
            automatic int ss=0, gs=0, n=0;
            for (i=rr*W+2; i<rr*W+W-2 && i<sc_ && i<gc_; i++) begin
                ss+=sep_r[i]; gs+=gen_r[i]; n++;
            end
            if (n>0) $display("   row %0d: sep=%0d gen=%0d", rr, ss/n, gs/n);
        end
        for (i=2*W; i<(H-2)*W && i<sc_ && i<gc_; i++)
            expect_near("sep==gen", sep_r[i], gen_r[i], 2);

        // 4) COLOUR channel independence: R/G/B differ -> sep must match general on ALL
        //    channels (the gray tests above only exercised R). Catches a per-channel bug.
        begin
            byte unsigned rv[0:H-1], gv[0:H-1], bv[0:H-1];
            for (i=0;i<H;i++) begin rv[i]=(i<4)?8'd40:8'd200; gv[i]=8'd128; bv[i]=8'd30+8'(i*10); end
            sc_=0; gc_=0; drive_color(rv, gv, bv);
            $display("[colour sep vs general] sample mid-rows R/G/B sep|gen:");
            for (i=4*W+2; i<(H-3)*W && i<sc_ && i<gc_; i++) begin
                expect_near("colR", sep_px[i][23:16], gen_px[i][23:16], 2);
                expect_near("colG", sep_px[i][15:8],  gen_px[i][15:8],  2);
                expect_near("colB", sep_px[i][7:0],   gen_px[i][7:0],   2);
            end
            $display("   row5: sep=%0d/%0d/%0d gen=%0d/%0d/%0d",
                     sep_px[5*W+3][23:16],sep_px[5*W+3][15:8],sep_px[5*W+3][7:0],
                     gen_px[5*W+3][23:16],gen_px[5*W+3][15:8],gen_px[5*W+3][7:0]);
        end

        if (errors==0) $display("TB_PASS: axis_rgb_conv5x5_sep (identity + gaussian + sep==general + colour OK)");
        else           $display("TB_FAIL: %0d error(s)", errors);
        $finish;
    end

endmodule

`default_nettype wire
