`timescale 1ns / 1ps
`default_nettype none

// axis_rgb_conv3x3 (2026-06-23, image-processing research base, Phase 2b).
// 3x3 spatial convolution on the 24-bit RGB888 pixel stream with RUNTIME-PROGRAMMABLE
// coefficients -- an arbitrary 3x3 kernel can be loaded live (no rebuild). Two BRAM
// line buffers hold lines N-1/N-2; with the streaming line N they form a 3x3 window.
// Per RGB channel: out = sat( (sum_i coeff[i] * w[i]) >>> shift ), with 9 signed 8-bit
// coefficients + a 4-bit right shift (normalisation). The 9*3 = 27 multiplies map to
// DSP48 slices (use_dsp) -- keeping them OUT of the LUT fabric, which is at the Z-7020
// congestion edge. Pipelined: products (DSP) -> sum -> shift/clamp; markers are delayed
// to match. cfg_en=0 -> passthrough (centre pixel) for point-op modes. Identity reset
// coeffs ({0,0,0,0,1,0,0,0,0}, shift 0) = passthrough until a kernel is loaded.
// Example kernels (sccb_write 0xFE00+i): Gaussian {1,2,1,2,4,2,1,2,1} shift 4; sharpen
// {0,-1,0,-1,5,-1,0,-1,0} shift 0; Sobel-X {-1,0,1,-2,0,2,-1,0,1} shift 0; emboss
// {-2,-1,0,-1,1,1,0,1,2} shift 0. DSim: verification/tb/tb_axis_rgb_conv3x3.
module axis_rgb_conv3x3 #(
    parameter int LINE_PIXELS = 640,
    parameter bit ENABLE      = 1'b1
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        cfg_en,                 // 1 = apply kernel, 0 = passthrough (centre)
    input  wire [71:0] cfg_coeffs,             // 9 x signed[7:0], idx 0=top-left .. 8=bot-right
    input  wire [3:0]  cfg_shift,              // right shift (normalisation)
    input  wire        cfg_abs,                // 1 = output |result| (gradient magnitude, e.g. Sobel)
    input  wire [23:0] in_pixel,
    input  wire        in_valid,
    input  wire        in_sof,
    input  wire        in_eol,
    input  wire        in_eof,
    input  wire        in_err,
    output logic [23:0] out_pixel,
    output logic        out_valid,
    output logic        out_sof,
    output logic        out_eol,
    output logic        out_eof,
    output logic        out_err
);

    generate if (!ENABLE) begin : g_bypass
        assign out_pixel = in_pixel;
        assign out_valid = in_valid;
        assign out_sof   = in_sof;
        assign out_eol   = in_eol;
        assign out_eof   = in_eof;
        assign out_err   = in_err;
    end else begin : g_conv
        localparam int AW = $clog2(LINE_PIXELS);

        logic [AW-1:0] col;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n)        col <= '0;
            else if (in_valid) col <= in_eol ? '0 : col + 1'b1;
        end

        // two line buffers (lbA=N-1, lbB=N-2), read-before-write
        (* ram_style = "block" *) logic [23:0] lbA [0:LINE_PIXELS-1];
        (* ram_style = "block" *) logic [23:0] lbB [0:LINE_PIXELS-1];
        logic [23:0] prev1, prev2;
        always_ff @(posedge clk) begin
            if (in_valid) begin
                prev1    <= lbA[col];
                prev2    <= lbB[col];
                lbB[col] <= lbA[col];
                lbA[col] <= in_pixel;
            end
        end

        // stage 1: align pixel/markers to the registered read
        logic [23:0] in_d;
        logic v1, s1, e1, f1, r1;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin {v1,s1,e1,f1,r1} <= '0; in_d <= '0; end
            else begin in_d <= in_pixel;
                v1 <= in_valid; s1 <= in_sof; e1 <= in_eol; f1 <= in_eof; r1 <= in_err; end
        end
        wire [23:0] vtop = prev2, vmid = prev1, vbot = in_d;   // rows N-2 / N-1 / N

        // stage 2: 3x3 window (c=2 newest). centre = w[1][1].
        logic [23:0] w [0:2][0:2];
        logic v2, s2, e2, f2, r2;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                for (int r=0;r<3;r++) for (int c=0;c<3;c++) w[r][c] <= '0;
                {v2,s2,e2,f2,r2} <= '0;
            end else begin
                if (v1) begin
                    w[0][0]<=w[0][1]; w[0][1]<=w[0][2]; w[0][2]<=vtop;
                    w[1][0]<=w[1][1]; w[1][1]<=w[1][2]; w[1][2]<=vmid;
                    w[2][0]<=w[2][1]; w[2][1]<=w[2][2]; w[2][2]<=vbot;
                end
                v2 <= v1; s2 <= s1; e2 <= e1; f2 <= f1; r2 <= r1;
            end
        end

        // flatten window per channel: tap order 0=w00..8=w22
        function automatic logic [7:0] tap(input int idx, input int sel);
            logic [23:0] p;
            p = w[idx/3][idx%3];
            tap = (sel==2) ? p[23:16] : (sel==1) ? p[15:8] : p[7:0];
        endfunction
        function automatic logic signed [7:0] coef(input int idx);
            coef = $signed(cfg_coeffs[idx*8 +: 8]);
        endfunction

        // stage 3: 27 products (signed coeff x unsigned pixel) -> DSP48
        (* use_dsp = "yes" *) logic signed [16:0] prod [0:2][0:8];
        logic [23:0] center3;
        logic v3, s3, e3, f3, r3, en3;
        logic [3:0] sh3;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                for (int sc=0;sc<3;sc++) for (int t=0;t<9;t++) prod[sc][t] <= '0;
                center3 <= '0; {v3,s3,e3,f3,r3,en3} <= '0; sh3 <= '0;
            end else begin
                for (int sc=0; sc<3; sc++)
                    for (int t=0; t<9; t++)
                        prod[sc][t] <= coef(t) * $signed({1'b0, tap(t, sc)});
                center3 <= w[1][1];
                en3 <= cfg_en; sh3 <= cfg_shift;
                v3 <= v2; s3 <= s2; e3 <= e2; f3 <= f2; r3 <= r2;
            end
        end

        // stage 4: sum the 9 products per channel -> acc
        logic signed [21:0] acc [0:2];
        logic [23:0] center4;
        logic v4, s4, e4, f4, r4, en4;
        logic [3:0] sh4;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                acc[0]<='0; acc[1]<='0; acc[2]<='0; center4<='0;
                {v4,s4,e4,f4,r4,en4} <= '0; sh4 <= '0;
            end else begin
                for (int sc=0; sc<3; sc++)
                    acc[sc] <= prod[sc][0]+prod[sc][1]+prod[sc][2]+prod[sc][3]+prod[sc][4]
                             + prod[sc][5]+prod[sc][6]+prod[sc][7]+prod[sc][8];
                center4 <= center3; en4 <= en3; sh4 <= sh3;
                v4 <= v3; s4 <= s3; e4 <= e3; f4 <= f3; r4 <= r3;
            end
        end

        // stage 5: shift + saturate per channel, or passthrough
        function automatic logic [7:0] sat(input logic signed [21:0] a, input logic [3:0] sh,
                                           input logic ab);
            logic signed [21:0] v;
            v = a >>> sh;
            if (ab && v < 0) v = -v;                 // |gradient| (recovers both edge polarities)
            sat = (v < 0) ? 8'd0 : (v > 22'sd255) ? 8'd255 : v[7:0];
        endfunction
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                out_pixel <= '0; {out_valid,out_sof,out_eol,out_eof,out_err} <= '0;
            end else begin
                out_pixel <= en4 ? {sat(acc[2],sh4,cfg_abs), sat(acc[1],sh4,cfg_abs), sat(acc[0],sh4,cfg_abs)}
                                 : center4;
                out_valid <= v4; out_sof <= s4; out_eol <= e4; out_eof <= f4; out_err <= r4;
            end
        end
    end endgenerate

endmodule

`default_nettype wire
