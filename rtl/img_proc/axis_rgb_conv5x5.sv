`timescale 1ns / 1ps
`default_nettype none

// axis_rgb_conv5x5 (2026-06-24, DoG dual-kernel, Phase A).
// 5x5 spatial convolution on the 24-bit RGB888 pixel stream with RUNTIME-PROGRAMMABLE
// coefficients -- an arbitrary (non-separable) 5x5 kernel can be loaded live. Four BRAM
// line buffers hold rows N-1..N-4; with the streaming row N they form a 5x5 window.
// Per RGB channel: out = sat( (sum_i coeff[i]*w[i]) >>> shift ), 25 signed 8-bit coeffs
// + a 4-bit right shift. The 25*3 = 75 multiplies map to DSP48 (use_dsp) to keep them
// OUT of the LUT fabric (Z-7020 congestion edge). The 25-tap sum is pipelined as a 2-level
// adder tree (5 group-sums of 5, then 5 partials -> acc) so no single stage adds 25 inputs.
// Pipeline = 6 stages: align -> window -> products -> group-sum -> acc -> shift/clamp;
// markers + centre are delayed to match. cfg_en=0 -> passthrough (centre w[2][2]).
// Same slot contract as axis_rgb_conv3x3 but ~1 line + 1 cycle more latency (used by
// axis_rgb_dog_combine to align the parallel 3x3 branch). DSim: tb_axis_rgb_conv5x5.
module axis_rgb_conv5x5 #(
    parameter int LINE_PIXELS = 640,
    parameter bit ENABLE      = 1'b1
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         cfg_en,                 // 1 = apply kernel, 0 = passthrough (centre)
    input  wire [199:0] cfg_coeffs,             // 25 x signed[7:0], idx 0=top-left .. 24=bot-right
    input  wire [3:0]   cfg_shift,              // right shift (normalisation)
    input  wire         cfg_abs,                // 1 = output |result| (gradient magnitude, e.g. Sobel)
    input  wire [23:0]  in_pixel,
    input  wire         in_valid,
    input  wire         in_sof,
    input  wire         in_eol,
    input  wire         in_eof,
    input  wire         in_err,
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

        // four line buffers (lbA=N-1 .. lbD=N-4), read-before-write cascade
        (* ram_style = "block" *) logic [23:0] lbA [0:LINE_PIXELS-1];
        (* ram_style = "block" *) logic [23:0] lbB [0:LINE_PIXELS-1];
        (* ram_style = "block" *) logic [23:0] lbC [0:LINE_PIXELS-1];
        (* ram_style = "block" *) logic [23:0] lbD [0:LINE_PIXELS-1];
        logic [23:0] prev1, prev2, prev3, prev4;
        always_ff @(posedge clk) begin
            if (in_valid) begin
                prev1    <= lbA[col];
                prev2    <= lbB[col];
                prev3    <= lbC[col];
                prev4    <= lbD[col];
                lbD[col] <= lbC[col];
                lbC[col] <= lbB[col];
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
        // window rows top->bottom: N-4 / N-3 / N-2 / N-1 / N
        wire [23:0] vrow0 = prev4, vrow1 = prev3, vrow2 = prev2, vrow3 = prev1, vrow4 = in_d;

        // stage 2: 5x5 window (col 4 = newest). centre = w[2][2].
        logic [23:0] w [0:4][0:4];
        logic v2, s2, e2, f2, r2;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                for (int r=0;r<5;r++) for (int c=0;c<5;c++) w[r][c] <= '0;
                {v2,s2,e2,f2,r2} <= '0;
            end else begin
                if (v1) begin
                    for (int r=0;r<5;r++) begin
                        w[r][0]<=w[r][1]; w[r][1]<=w[r][2];
                        w[r][2]<=w[r][3]; w[r][3]<=w[r][4];
                    end
                    w[0][4]<=vrow0; w[1][4]<=vrow1; w[2][4]<=vrow2; w[3][4]<=vrow3; w[4][4]<=vrow4;
                end
                v2 <= v1; s2 <= s1; e2 <= e1; f2 <= f1; r2 <= r1;
            end
        end

        // tap order 0=w00 .. 24=w44 (row-major); sel 2/1/0 = R/G/B
        function automatic logic [7:0] tap(input int idx, input int sel);
            logic [23:0] p;
            p = w[idx/5][idx%5];
            tap = (sel==2) ? p[23:16] : (sel==1) ? p[15:8] : p[7:0];
        endfunction
        function automatic logic signed [7:0] coef(input int idx);
            coef = $signed(cfg_coeffs[idx*8 +: 8]);
        endfunction

        // stage 3: 75 products (signed coeff x unsigned pixel) -> DSP48
        (* use_dsp = "yes" *) logic signed [16:0] prod [0:2][0:24];
        logic [23:0] center3;
        logic v3, s3, e3, f3, r3, en3;
        logic [3:0] sh3;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                for (int sc=0;sc<3;sc++) for (int t=0;t<25;t++) prod[sc][t] <= '0;
                center3 <= '0; {v3,s3,e3,f3,r3,en3} <= '0; sh3 <= '0;
            end else begin
                for (int sc=0; sc<3; sc++)
                    for (int t=0; t<25; t++)
                        prod[sc][t] <= coef(t) * $signed({1'b0, tap(t, sc)});
                center3 <= w[2][2];
                en3 <= cfg_en; sh3 <= cfg_shift;
                v3 <= v2; s3 <= s2; e3 <= e2; f3 <= f2; r3 <= r2;
            end
        end

        // stage 4: group-sum (5 groups of 5 products per channel) -- 1st adder-tree level
        logic signed [20:0] psum [0:2][0:4];
        logic [23:0] center4;
        logic v4, s4, e4, f4, r4, en4;
        logic [3:0] sh4;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                for (int sc=0;sc<3;sc++) for (int g=0;g<5;g++) psum[sc][g] <= '0;
                center4 <= '0; {v4,s4,e4,f4,r4,en4} <= '0; sh4 <= '0;
            end else begin
                for (int sc=0; sc<3; sc++)
                    for (int g=0; g<5; g++)
                        psum[sc][g] <= prod[sc][g*5+0] + prod[sc][g*5+1] + prod[sc][g*5+2]
                                     + prod[sc][g*5+3] + prod[sc][g*5+4];
                center4 <= center3; en4 <= en3; sh4 <= sh3;
                v4 <= v3; s4 <= s3; e4 <= e3; f4 <= f3; r4 <= r3;
            end
        end

        // stage 5: accumulate the 5 partials per channel -- 2nd adder-tree level
        logic signed [23:0] acc [0:2];
        logic [23:0] center5;
        logic v5, s5, e5, f5, r5, en5;
        logic [3:0] sh5;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                acc[0]<='0; acc[1]<='0; acc[2]<='0; center5<='0;
                {v5,s5,e5,f5,r5,en5} <= '0; sh5 <= '0;
            end else begin
                for (int sc=0; sc<3; sc++)
                    acc[sc] <= psum[sc][0]+psum[sc][1]+psum[sc][2]+psum[sc][3]+psum[sc][4];
                center5 <= center4; en5 <= en4; sh5 <= sh4;
                v5 <= v4; s5 <= s4; e5 <= e4; f5 <= f4; r5 <= r4;
            end
        end

        // stage 6: shift + saturate per channel, or passthrough centre
        function automatic logic [7:0] sat(input logic signed [23:0] a, input logic [3:0] sh,
                                           input logic ab);
            logic signed [23:0] v;
            v = a >>> sh;
            if (ab && v < 0) v = -v;                 // |gradient| (recovers both edge polarities)
            sat = (v < 0) ? 8'd0 : (v > 24'sd255) ? 8'd255 : v[7:0];
        endfunction
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                out_pixel <= '0; {out_valid,out_sof,out_eol,out_eof,out_err} <= '0;
            end else begin
                out_pixel <= en5 ? {sat(acc[2],sh5,cfg_abs), sat(acc[1],sh5,cfg_abs), sat(acc[0],sh5,cfg_abs)}
                                 : center5;
                out_valid <= v5; out_sof <= s5; out_eol <= e5; out_eof <= f5; out_err <= r5;
            end
        end
    end endgenerate

endmodule

`default_nettype wire
