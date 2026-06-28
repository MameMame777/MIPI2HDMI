`timescale 1ns / 1ps
`default_nettype none

// DSim testbench for axis_rgb_prefilter (PRE spatial-denoise stage, 2026-06-25).
// Window-centre mapping (same front end as axis_rgb_conv3x3): the output beat carrying
// input-pixel (r,c)'s markers holds the 3x3 window CENTRED at (r-1,c-1), i.e. input rows
// r-2..r, cols c-2..c. So out[(r,c)] = filter(input[r-2..r][c-2..c]) for r>=2,c>=2.
// Checks: passthrough==centre, point invert/threshold(runtime level), gaussian golden,
// median golden (per-pixel over a varied frame = exercises median9), salt-and-pepper
// removal (median yes / gaussian no), and the fixed-latency invariant across modes.
module tb_axis_rgb_prefilter;
    localparam int W = 8, H = 8;

    logic clk = 0, rst_n = 0;
    logic [3:0]  cfg_op = 0;
    logic [7:0]  cfg_thresh = 8'd128;
    logic [23:0] in_pixel = 0;
    logic in_valid=0, in_sof=0, in_eol=0, in_eof=0, in_err=0;
    logic [23:0] out_pixel;
    logic out_valid, out_sof, out_eol, out_eof, out_err;

    always #5 clk = ~clk;

    axis_rgb_prefilter #(.LINE_PIXELS(W), .ENABLE(1'b1)) dut (
        .clk(clk), .rst_n(rst_n), .cfg_op(cfg_op), .cfg_thresh_level(cfg_thresh),
        .in_pixel(in_pixel), .in_valid(in_valid), .in_sof(in_sof),
        .in_eol(in_eol), .in_eof(in_eof), .in_err(in_err),
        .out_pixel(out_pixel), .out_valid(out_valid), .out_sof(out_sof),
        .out_eol(out_eol), .out_eof(out_eof), .out_err(out_err));

    logic [7:0] out_r [0:H*W-1];
    logic [7:0] out_g [0:H*W-1];
    integer ocnt = 0, sof_idx = -1, eof_idx = -1;
    always_ff @(posedge clk) begin
        if (out_valid) begin
            if (ocnt < H*W) begin out_r[ocnt] <= out_pixel[23:16]; out_g[ocnt] <= out_pixel[15:8]; end
            if (out_sof) sof_idx <= ocnt;
            if (out_eof) eof_idx <= ocnt;
            ocnt <= ocnt + 1;
        end
    end

    task automatic drive_frame(input byte unsigned px [0:H-1][0:W-1]);
        integer r, c;
        @(negedge clk);
        for (r=0;r<H;r++) for (c=0;c<W;c++) begin
            in_valid <= 1'b1;
            in_pixel <= {px[r][c], px[r][c], px[r][c]};
            in_sof   <= (r==0 && c==0);
            in_eol   <= (c==W-1);
            in_eof   <= (r==H-1 && c==W-1);
            @(negedge clk);
        end
        in_valid<=0; in_sof<=0; in_eol<=0; in_eof<=0;
        repeat (24) @(negedge clk);     // flush 7-cycle pipeline + margin
    endtask

    function automatic logic [7:0] med9sw(input byte unsigned x [0:8]);
        byte unsigned a [0:8]; integer i, j; byte unsigned t;
        for (i=0;i<9;i++) a[i]=x[i];
        for (i=0;i<9;i++) for (j=i+1;j<9;j++) if (a[j]<a[i]) begin t=a[i]; a[i]=a[j]; a[j]=t; end
        med9sw = a[4];
    endfunction

    // golden median for out[(r,c)] = window input rows r-2..r, cols c-2..c
    function automatic logic [7:0] gold_med(input byte unsigned px [0:H-1][0:W-1],
                                            input int r, input int c);
        byte unsigned win [0:8]; integer i, rr, cc;
        i = 0;
        for (rr=r-2; rr<=r; rr++) for (cc=c-2; cc<=c; cc++) begin win[i]=px[rr][cc]; i++; end
        gold_med = med9sw(win);
    endfunction

    function automatic logic [7:0] gold_gauss(input byte unsigned px [0:H-1][0:W-1],
                                              input int r, input int c);
        integer corner, edgesum, cen, tot;
        corner  = px[r-2][c-2] + px[r-2][c] + px[r][c-2] + px[r][c];
        edgesum = px[r-2][c-1] + px[r-1][c-2] + px[r-1][c] + px[r][c-1];
        cen     = px[r-1][c-1];
        tot     = corner + 2*edgesum + 4*cen;
        gold_gauss = tot >> 4;
    endfunction

    integer errors = 0;
    task automatic expect_eq(input string nm, input int got, input int exp);
        if (got !== exp) begin $display("  FAIL %s: got %0d exp %0d", nm, got, exp); errors++; end
    endtask

    byte unsigned uni [0:H-1][0:W-1];
    byte unsigned det [0:H-1][0:W-1];
    byte unsigned sp  [0:H-1][0:W-1];
    integer r, c, k;
    integer sof0, eof0, sof8, eof8, sof9, eof9;

    initial begin
        for (r=0;r<H;r++) for (c=0;c<W;c++) begin
            uni[r][c] = 8'd100;
            det[r][c] = (r*37 + c*101 + 7) & 8'hFF;       // deterministic varied frame
            sp[r][c]  = 8'd120;                            // salt-and-pepper base
        end
        sp[3][3]=8'd0; sp[4][5]=8'd255; sp[5][2]=8'd0;     // isolated impulses (interior)

        rst_n=0; repeat(4) @(negedge clk); rst_n=1; repeat(2) @(negedge clk);

        // --- 1) passthrough (op0): out[(r,c)] == centre px[r-1][c-1] (interior) ---
        cfg_op=4'd0; ocnt=0; drive_frame(det);
        for (r=2;r<H;r++) for (c=2;c<W;c++) begin k=r*W+c;
            if (k<ocnt) expect_eq("pass-centre", out_r[k], det[r-1][c-1]); end

        // --- 2) median (op9): per-pixel golden over the varied frame (exercises median9) ---
        cfg_op=4'd9; ocnt=0; drive_frame(det);
        for (r=2;r<H;r++) for (c=2;c<W;c++) begin k=r*W+c;
            if (k<ocnt) expect_eq("median", out_r[k], gold_med(det,r,c)); end

        // --- 3) gaussian (op8): per-pixel golden ---
        cfg_op=4'd8; ocnt=0; drive_frame(det);
        for (r=2;r<H;r++) for (c=2;c<W;c++) begin k=r*W+c;
            if (k<ocnt) expect_eq("gauss", out_r[k], gold_gauss(det,r,c)); end

        // --- 4) salt-and-pepper: median removes impulses (all interior == 120) ---
        cfg_op=4'd9; ocnt=0; drive_frame(sp);
        for (r=2;r<H;r++) for (c=2;c<W;c++) begin k=r*W+c;
            if (k<ocnt) expect_eq("median-sp", out_r[k], 120); end
        // gaussian does NOT fully remove an impulse: the output centred on px[3][3]=0 is
        // out[(4,4)] and must be < 120 (pulled down), i.e. gaussian keeps a trace.
        cfg_op=4'd8; ocnt=0; drive_frame(sp);
        k=4*W+4;
        if (k<ocnt && !(out_r[k] < 120)) begin
            $display("  FAIL gauss-sp: expected <120 at impulse, got %0d", out_r[k]); errors++; end
        else $display("  [gauss-sp] impulse centre out=%0d (<120 = impulse NOT removed, as expected)", out_r[k]);

        // --- 5) threshold (op4) runtime level: centre px[r-1][c-1] > thr ? white : black ---
        cfg_op=4'd4; cfg_thresh=8'd128; ocnt=0; drive_frame(det);
        for (r=2;r<H;r++) for (c=2;c<W;c++) begin k=r*W+c;
            if (k<ocnt) expect_eq("thr128", out_r[k], (det[r-1][c-1] > 8'd128) ? 8'd255 : 8'd0); end
        cfg_op=4'd4; cfg_thresh=8'd40; ocnt=0; drive_frame(det);
        for (r=2;r<H;r++) for (c=2;c<W;c++) begin k=r*W+c;
            if (k<ocnt) expect_eq("thr40", out_r[k], (det[r-1][c-1] > 8'd40) ? 8'd255 : 8'd0); end
        cfg_thresh=8'd128;

        // --- 6) invert (op1): 255 - centre ---
        cfg_op=4'd1; ocnt=0; drive_frame(det);
        for (r=2;r<H;r++) for (c=2;c<W;c++) begin k=r*W+c;
            if (k<ocnt) expect_eq("invert", out_r[k], 8'd255 - det[r-1][c-1]); end

        // --- 7) fixed-latency invariant: same uniform frame under op0/op8/op9 -> identical
        //         output count + sof/eof positions (no marker skew on mode switch) ---
        cfg_op=4'd0; ocnt=0; sof_idx=-1; eof_idx=-1; drive_frame(uni); sof0=sof_idx; eof0=eof_idx;
        expect_eq("cnt-op0", ocnt, H*W);
        cfg_op=4'd8; ocnt=0; sof_idx=-1; eof_idx=-1; drive_frame(uni); sof8=sof_idx; eof8=eof_idx;
        expect_eq("cnt-op8", ocnt, H*W);
        cfg_op=4'd9; ocnt=0; sof_idx=-1; eof_idx=-1; drive_frame(uni); sof9=sof_idx; eof9=eof_idx;
        expect_eq("cnt-op9", ocnt, H*W);
        expect_eq("sof-align-08", sof0, sof8); expect_eq("sof-align-09", sof0, sof9);
        expect_eq("eof-align-08", eof0, eof8); expect_eq("eof-align-09", eof0, eof9);
        $display("  [latency] sof_idx=%0d eof_idx=%0d (identical across op0/op8/op9)", sof0, eof0);

        if (errors==0) $display("TB_PASS: axis_rgb_prefilter (passthrough/point/gauss/median/salt-pepper/latency OK)");
        else           $display("TB_FAIL: %0d error(s)", errors);
        $finish;
    end

endmodule

`default_nettype wire
