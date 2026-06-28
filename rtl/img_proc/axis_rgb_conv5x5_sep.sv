`timescale 1ns / 1ps
`default_nettype none

// axis_rgb_conv5x5_sep (2026-06-24, cascade multi-scale, Phase A).
// SEPARABLE 5x5 convolution on the 24-bit RGB888 stream: out = (h (x) v) * image, run as
// two 1-D passes -- horizontal 1x5 (h[0..4]) then vertical 5x1 (v[0..4]). Costs 5+5 = 10
// multiplies/channel = 30 DSP (vs 75 for a general 5x5), so several can be cascaded within
// the Z-7020 DSP budget. Only SEPARABLE kernels are reachable (Gaussian / box / separable
// derivative) -- the cascade puts arbitrary kernels in a general-5x5 stage and uses these
// for the cheap blur stages. Bypass = load the identity kernel (h=v={0,0,1,0,0}, shifts 0),
// which is the reset default -> a stage with no kernel loaded is passthrough (used to vary
// the cascade's effective kernel size). The horizontal result is requantised to signed 12b
// (cfg_hshift) before the line buffers; the vertical pass applies cfg_vshift + saturate.
// 8-stage pipeline -- the horizontal/vertical 5-add and the shift+clamp/saturate are in
// SEPARATE stages (the combined sum->shift->clamp was a WNS critical net on sysclk, like
// the DoG combiner). Same slot contract. DSim: tb_axis_rgb_conv5x5_sep.
module axis_rgb_conv5x5_sep #(
    parameter int LINE_PIXELS = 640,
    parameter bit ENABLE      = 1'b1
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [39:0] cfg_h,                  // 5 x signed[7:0] horizontal taps
    input  wire [39:0] cfg_v,                  // 5 x signed[7:0] vertical taps
    input  wire [3:0]  cfg_hshift,             // horizontal requantise shift
    input  wire [3:0]  cfg_vshift,             // vertical normalise shift
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
    end else begin : g_sep
        localparam int AW = $clog2(LINE_PIXELS);
        function automatic logic signed [7:0] hc(input int i); hc = $signed(cfg_h[i*8 +: 8]); endfunction
        function automatic logic signed [7:0] vc(input int j); vc = $signed(cfg_v[j*8 +: 8]); endfunction
        function automatic logic [7:0] pch(input logic [23:0] p, input int sc);
            pch = (sc==2) ? p[23:16] : (sc==1) ? p[15:8] : p[7:0];
        endfunction

        // ---------------- HORIZONTAL PASS (1x5) ----------------
        // stage 1: 5-wide column window (hwin[4] = newest col)
        logic [23:0] hwin [0:4];
        logic v1, s1, e1, f1, r1;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                for (int i=0;i<5;i++) hwin[i] <= '0; {v1,s1,e1,f1,r1} <= '0;
            end else begin
                if (in_valid) begin
                    hwin[0]<=hwin[1]; hwin[1]<=hwin[2]; hwin[2]<=hwin[3]; hwin[3]<=hwin[4];
                    hwin[4]<=in_pixel;
                end
                v1<=in_valid; s1<=in_sof; e1<=in_eol; f1<=in_eof; r1<=in_err;
            end
        end

        // stage 2: 15 horizontal products (h x pixel) -> DSP
        (* use_dsp = "yes" *) logic signed [16:0] hprod [0:2][0:4];
        logic v2, s2, e2, f2, r2;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                for (int sc=0;sc<3;sc++) for (int i=0;i<5;i++) hprod[sc][i] <= '0;
                {v2,s2,e2,f2,r2} <= '0;
            end else begin
                for (int sc=0;sc<3;sc++) for (int i=0;i<5;i++)
                    hprod[sc][i] <= hc(i) * $signed({1'b0, pch(hwin[i], sc)});
                v2<=v1; s2<=s1; e2<=e1; f2<=f1; r2<=r1;
            end
        end

        // stage 3: horizontal sum ONLY (registered) -- split the long sum->shift->clamp path
        // (this was a WNS critical net on sysclk: 5-add + barrel-shift + clamp in one stage).
        logic signed [21:0] hsum [0:2];
        logic v3, s3, e3, f3, r3;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin hsum[0]<='0;hsum[1]<='0;hsum[2]<='0; {v3,s3,e3,f3,r3}<='0; end
            else begin
                for (int sc=0;sc<3;sc++)
                    hsum[sc] <= hprod[sc][0]+hprod[sc][1]+hprod[sc][2]+hprod[sc][3]+hprod[sc][4];
                v3<=v2; s3<=s2; e3<=e2; f3<=f2; r3<=r2;
            end
        end

        // stage 4: requantise (>> hshift, clamp signed 12b) -> hout
        function automatic logic signed [11:0] clamp12(input logic signed [21:0] x);
            clamp12 = (x > 22'sd2047) ? 12'sd2047 : (x < -22'sd2048) ? -12'sd2048 : x[11:0];
        endfunction
        logic signed [11:0] hout [0:2];
        logic v4, s4, e4, f4, r4;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin hout[0]<='0;hout[1]<='0;hout[2]<='0; {v4,s4,e4,f4,r4}<='0; end
            else begin
                for (int sc=0;sc<3;sc++) hout[sc] <= clamp12(hsum[sc] >>> cfg_hshift);
                v4<=v3; s4<=s3; e4<=e3; f4<=f3; r4<=r3;
            end
        end
        wire [35:0] hout_pk = {hout[2], hout[1], hout[0]};
        function automatic logic signed [11:0] hch(input logic [35:0] p, input int sc);
            hch = $signed(p[sc*12 +: 12]);
        endfunction

        // ---------------- VERTICAL PASS (5x1) ----------------
        // line-buffer column index for the hout stream (resets on hout EOL)
        logic [AW-1:0] vcol;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n)    vcol <= '0;
            else if (v4)   vcol <= e4 ? '0 : vcol + 1'b1;
        end
        // four line buffers of hout (read-before-write cascade)
        (* ram_style = "block" *) logic [35:0] lbA [0:LINE_PIXELS-1];
        (* ram_style = "block" *) logic [35:0] lbB [0:LINE_PIXELS-1];
        (* ram_style = "block" *) logic [35:0] lbC [0:LINE_PIXELS-1];
        (* ram_style = "block" *) logic [35:0] lbD [0:LINE_PIXELS-1];
        logic [35:0] hp1, hp2, hp3, hp4;
        always_ff @(posedge clk) begin
            if (v4) begin
                hp1<=lbA[vcol]; hp2<=lbB[vcol]; hp3<=lbC[vcol]; hp4<=lbD[vcol];
                lbD[vcol]<=lbC[vcol]; lbC[vcol]<=lbB[vcol]; lbB[vcol]<=lbA[vcol]; lbA[vcol]<=hout_pk;
            end
        end
        // stage 5: align current hout to the registered reads -> vertical window rows N-4..N
        logic [35:0] hout_d;
        logic v5, s5, e5, f5, r5;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin hout_d<='0; {v5,s5,e5,f5,r5}<='0; end
            else begin hout_d<=hout_pk; v5<=v4; s5<=s4; e5<=e4; f5<=f4; r5<=r4; end
        end
        wire [35:0] vr0=hp4, vr1=hp3, vr2=hp2, vr3=hp1, vr4=hout_d;  // rows N-4..N

        // stage 6: 15 vertical products (v x hout) -> DSP
        (* use_dsp = "yes" *) logic signed [20:0] vprod [0:2][0:4];
        logic v6, s6, e6, f6, r6;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                for (int sc=0;sc<3;sc++) for (int j=0;j<5;j++) vprod[sc][j] <= '0;
                {v6,s6,e6,f6,r6} <= '0;
            end else begin
                for (int sc=0;sc<3;sc++) begin
                    vprod[sc][0] <= vc(0) * hch(vr0, sc);
                    vprod[sc][1] <= vc(1) * hch(vr1, sc);
                    vprod[sc][2] <= vc(2) * hch(vr2, sc);
                    vprod[sc][3] <= vc(3) * hch(vr3, sc);
                    vprod[sc][4] <= vc(4) * hch(vr4, sc);
                end
                v6<=v5; s6<=s5; e6<=e5; f6<=f5; r6<=r5;
            end
        end

        // stage 7: vertical sum ONLY (registered) -- split from the shift/saturate path
        logic signed [23:0] vsum [0:2];
        logic v7, s7, e7, f7, r7;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin vsum[0]<='0;vsum[1]<='0;vsum[2]<='0; {v7,s7,e7,f7,r7}<='0; end
            else begin
                for (int sc=0;sc<3;sc++)
                    vsum[sc] <= vprod[sc][0]+vprod[sc][1]+vprod[sc][2]+vprod[sc][3]+vprod[sc][4];
                v7<=v6; s7<=s6; e7<=e6; f7<=f6; r7<=r6;
            end
        end

        // stage 8: normalise (>> vshift) + saturate -> 8b per channel
        function automatic logic [7:0] sat8(input logic signed [23:0] x);
            sat8 = (x < 0) ? 8'd0 : (x > 24'sd255) ? 8'd255 : x[7:0];
        endfunction
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                out_pixel <= '0; {out_valid,out_sof,out_eol,out_eof,out_err} <= '0;
            end else begin
                for (int sc=0;sc<3;sc++) out_pixel[sc*8 +: 8] <= sat8(vsum[sc] >>> cfg_vshift);
                out_valid<=v7; out_sof<=s7; out_eol<=e7; out_eof<=f7; out_err<=r7;
            end
        end
    end endgenerate

endmodule

`default_nettype wire
