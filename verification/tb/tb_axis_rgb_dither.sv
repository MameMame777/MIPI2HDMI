`timescale 1ns / 1ps
`default_nettype none

// DSim testbench for axis_rgb_dither (final dither stage, 2026-06-26).
// Point-wise op (no neighbourhood) -> every pixel is independent; out[k] for input pixel
// k=(r,c) uses bayer4(r%4,c%4). Checks: (1) ctrl=0 -> passthrough; (2) ordered N=2 vs an exact
// golden over a varied frame; (3) ordered N=1 on flat gray -> only {0,255} and BOTH appear
// (dither present); (4) random N=2 on flat gray -> outputs in the 4-level set; (5) fixed-latency
// invariant (sof/eof output index identical across off/ordered/random).
module tb_axis_rgb_dither;
    localparam int W = 8, H = 8;

    logic clk = 0, rst_n = 0;
    logic [7:0]  cfg_ctrl = 0;
    logic [23:0] in_pixel = 0;
    logic in_valid=0, in_sof=0, in_eol=0, in_eof=0, in_err=0;
    logic [23:0] out_pixel;
    logic out_valid, out_sof, out_eol, out_eof, out_err;

    always #5 clk = ~clk;

    axis_rgb_dither #(.LINE_PIXELS(W), .ENABLE(1'b1)) dut (
        .clk(clk), .rst_n(rst_n), .cfg_ctrl(cfg_ctrl),
        .in_pixel(in_pixel), .in_valid(in_valid), .in_sof(in_sof),
        .in_eol(in_eol), .in_eof(in_eof), .in_err(in_err),
        .out_pixel(out_pixel), .out_valid(out_valid), .out_sof(out_sof),
        .out_eol(out_eol), .out_eof(out_eof), .out_err(out_err));

    logic [7:0] out_r [0:H*W-1];
    integer ocnt=0, sof_idx=-1, eof_idx=-1;
    always_ff @(posedge clk) begin
        if (out_valid) begin
            if (ocnt < H*W) out_r[ocnt] <= out_pixel[23:16];
            if (out_sof) sof_idx <= ocnt;
            if (out_eof) eof_idx <= ocnt;
            ocnt <= ocnt + 1;
        end
    end

    task automatic drive_frame(input byte unsigned px [0:H-1][0:W-1]);
        integer r,c;
        @(negedge clk);
        for (r=0;r<H;r++) for (c=0;c<W;c++) begin
            in_valid<=1'b1; in_pixel<={px[r][c],px[r][c],px[r][c]};
            in_sof<=(r==0&&c==0); in_eol<=(c==W-1); in_eof<=(r==H-1&&c==W-1);
            @(negedge clk);
        end
        in_valid<=0; in_sof<=0; in_eol<=0; in_eof<=0;
        repeat (8) @(negedge clk);
    endtask

    function automatic logic [3:0] bayer4(input int yy, input int xx);
        int y=yy&3, x=xx&3; int idx=y*4+x;
        case (idx)
            0:bayer4=0;  1:bayer4=8;  2:bayer4=2;  3:bayer4=10;
            4:bayer4=12; 5:bayer4=4;  6:bayer4=14; 7:bayer4=6;
            8:bayer4=3;  9:bayer4=11; 10:bayer4=1; 11:bayer4=9;
            12:bayer4=15;13:bayer4=7; 14:bayer4=13;15:bayer4=5;
        endcase
    endfunction

    // exact replica of the RTL ordered dither_ch
    function automatic logic [7:0] gold_ord(input int v, input int by, input int n);
        int drop, bias, sum, q, o;
        if (n==0 || n>=7) return v[7:0];
        drop = 8-n;
        if (drop>=4) bias = by << (drop-4); else bias = by >> (4-drop);
        sum = v + bias; if (sum>255) sum=255;
        q = sum & ~((1<<drop)-1);
        o = q; o = o | (o>>n); o = o | (o>>(2*n)); o = o | (o>>(4*n));
        return o[7:0];
    endfunction

    // the 8-bit replicated level for top-N code t (0..2^N-1)
    function automatic logic [7:0] level(input int t, input int n);
        int drop=8-n; int q=(t<<drop)&255; int o=q;
        o=o|(o>>n); o=o|(o>>(2*n)); o=o|(o>>(4*n)); return o[7:0];
    endfunction

    integer errors=0;
    task automatic eq(input string nm, input int got, input int exp);
        if (got!==exp) begin $display("  FAIL %s: got %0d exp %0d", nm, got, exp); errors++; end
    endtask

    byte unsigned det [0:H-1][0:W-1];
    byte unsigned flat [0:H-1][0:W-1];
    integer r,c,k,i; integer n0,n255; logic ok; logic [7:0] L[0:3];
    integer so0,eo0,so1,eo1,so2,eo2;

    initial begin
        for (r=0;r<H;r++) for (c=0;c<W;c++) begin
            det[r][c]=(r*37+c*101+11)&8'hFF; flat[r][c]=8'd100; end

        rst_n=0; repeat(4) @(negedge clk); rst_n=1; repeat(2) @(negedge clk);

        // 1) ctrl=0 -> passthrough
        cfg_ctrl=8'h00; ocnt=0; drive_frame(det);
        for (r=0;r<H;r++) for (c=0;c<W;c++) begin k=r*W+c;
            if (k<ocnt) eq("pass", out_r[k], det[r][c]); end

        // 2) ordered N=2 (en=1,mode=0,bits=2) vs exact golden
        cfg_ctrl = 8'h01 | (2<<2); ocnt=0; drive_frame(det);   // 0x09
        for (r=0;r<H;r++) for (c=0;c<W;c++) begin k=r*W+c;
            if (k<ocnt) eq("ord2", out_r[k], gold_ord(det[r][c], bayer4(r,c), 2)); end

        // 3) ordered N=1 on flat gray -> only {0,255}, both present (dither)
        cfg_ctrl = 8'h01 | (1<<2); ocnt=0; n0=0; n255=0; drive_frame(flat);   // 0x05
        for (k=0;k<H*W && k<ocnt;k++) begin
            if (out_r[k]==8'd0) n0++; else if (out_r[k]==8'd255) n255++;
            else begin $display("  FAIL ord1 level: got %0d (expect 0/255)", out_r[k]); errors++; end
        end
        if (!(n0>0 && n255>0)) begin $display("  FAIL ord1 dither: n0=%0d n255=%0d (both must appear)", n0, n255); errors++; end
        else $display("  [ord1] flat100 -> %0d black + %0d white (halftone dither OK)", n0, n255);

        // 4) random N=2 on flat gray -> outputs in the 4-level set
        for (i=0;i<4;i++) L[i]=level(i,2);
        cfg_ctrl = 8'h01 | (1<<1) | (2<<2); ocnt=0; drive_frame(flat);   // 0x0B
        for (k=0;k<H*W && k<ocnt;k++) begin
            ok = (out_r[k]==L[0])||(out_r[k]==L[1])||(out_r[k]==L[2])||(out_r[k]==L[3]);
            if (!ok) begin $display("  FAIL rand2 level: got %0d not in {%0d,%0d,%0d,%0d}", out_r[k],L[0],L[1],L[2],L[3]); errors++; end
        end
        $display("  [rand2] levels = {%0d,%0d,%0d,%0d}", L[0],L[1],L[2],L[3]);

        // 5) fixed-latency invariant: off / ordered / random -> same count + sof/eof index
        cfg_ctrl=8'h00;       ocnt=0; sof_idx=-1; eof_idx=-1; drive_frame(flat); so0=sof_idx; eo0=eof_idx; eq("cnt-off",ocnt,H*W);
        cfg_ctrl=8'h01|(2<<2);ocnt=0; sof_idx=-1; eof_idx=-1; drive_frame(flat); so1=sof_idx; eo1=eof_idx; eq("cnt-ord",ocnt,H*W);
        cfg_ctrl=8'h0B;       ocnt=0; sof_idx=-1; eof_idx=-1; drive_frame(flat); so2=sof_idx; eo2=eof_idx; eq("cnt-rnd",ocnt,H*W);
        eq("sof-01",so0,so1); eq("sof-02",so0,so2); eq("eof-01",eo0,eo1); eq("eof-02",eo0,eo2);
        $display("  [latency] sof_idx=%0d eof_idx=%0d (identical across off/ordered/random)", so0, eo0);

        if (errors==0) $display("TB_PASS: axis_rgb_dither (passthrough/ordered-golden/halftone/random-levels/latency OK)");
        else           $display("TB_FAIL: %0d error(s)", errors);
        $finish;
    end
endmodule

`default_nettype wire
