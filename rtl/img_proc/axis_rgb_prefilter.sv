`timescale 1ns / 1ps
`default_nettype none

// axis_rgb_prefilter (2026-06-25): the PRE stage of the image-processing chain, upgraded
// from a point-op-only slot to a 3x3 SPATIAL filter with line buffers. Modes (cfg_op):
//   0      passthrough (centre)            8  gaussian 3x3  {1,2,1;2,4,2;1,2,1}>>4 (window)
//   1      invert                          9  median 3x3    (per-channel 9-median, window)
//   2      grayscale (green approx)        10-15 reserved -> passthrough
//   3      BGR swap
//   4      threshold (green > cfg_thresh_level)
//   5/6/7  R/G/B only
// Point ops (0-7) act on the centre tap; gaussian/median act on the full 3x3 window.
// It ALWAYS forms the window (2 line buffers, reused verbatim from axis_rgb_conv3x3 so the
// border behaviour is identical) and selects the output -- so EVERY mode has the SAME fixed
// latency (window 2 + compute 5 + output 1 = 8 cycles) with markers aligned, and switching
// cfg_op live never skews pixels vs {valid,sof,eol,eof,err}. ENABLE=0 -> wire-through.
// Median network = median9 (verified). DSim: verification/tb/tb_axis_rgb_prefilter.
module axis_rgb_prefilter #(
    parameter int LINE_PIXELS = 640,
    parameter bit ENABLE      = 1'b1
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [3:0]  cfg_op,                 // see header (0=pass,1-7 point,8 gauss,9 median)
    input  wire [7:0]  cfg_thresh_level,       // op-4 threshold (green)
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
    end else begin : g_pre
        localparam int AW = $clog2(LINE_PIXELS);

        // ---- front end: col counter + 2 line buffers + window (copied from conv3x3) ----
        logic [AW-1:0] col;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n)        col <= '0;
            else if (in_valid) col <= in_eol ? '0 : col + 1'b1;
        end

        (* ram_style = "block" *) logic [23:0] lbA [0:LINE_PIXELS-1];
        (* ram_style = "block" *) logic [23:0] lbB [0:LINE_PIXELS-1];
        logic [23:0] prev1, prev2;
        always_ff @(posedge clk) begin           // reset-FREE so Vivado infers RAM
            if (in_valid) begin
                prev1    <= lbA[col];
                prev2    <= lbB[col];
                lbB[col] <= lbA[col];
                lbA[col] <= in_pixel;
            end
        end

        // stage 1: align pixel + markers to the registered read
        logic [23:0] in_d;
        logic v1, s1, e1, f1, r1;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin {v1,s1,e1,f1,r1} <= '0; in_d <= '0; end
            else begin in_d <= in_pixel;
                v1 <= in_valid; s1 <= in_sof; e1 <= in_eol; f1 <= in_eof; r1 <= in_err; end
        end
        wire [23:0] vtop = prev2, vmid = prev1, vbot = in_d;

        // stage 2: 3x3 window (c=2 newest), centre = w[1][1]
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

        function automatic logic [7:0] ch(input logic [23:0] p, input int sel);
            ch = (sel==2) ? p[23:16] : (sel==1) ? p[15:8] : p[7:0];
        endfunction

        // ===================== BRANCH A: median (median9 x3, 4-cycle latency) =====================
        wire [7:0] med_r, med_g, med_b;
        median9 u_med_r (.clk(clk), .rst_n(rst_n), .in_en(1'b1),
            .s0(w[0][0][23:16]), .s1(w[0][1][23:16]), .s2(w[0][2][23:16]),
            .s3(w[1][0][23:16]), .s4(w[1][1][23:16]), .s5(w[1][2][23:16]),
            .s6(w[2][0][23:16]), .s7(w[2][1][23:16]), .s8(w[2][2][23:16]), .med(med_r));
        median9 u_med_g (.clk(clk), .rst_n(rst_n), .in_en(1'b1),
            .s0(w[0][0][15:8]), .s1(w[0][1][15:8]), .s2(w[0][2][15:8]),
            .s3(w[1][0][15:8]), .s4(w[1][1][15:8]), .s5(w[1][2][15:8]),
            .s6(w[2][0][15:8]), .s7(w[2][1][15:8]), .s8(w[2][2][15:8]), .med(med_g));
        median9 u_med_b (.clk(clk), .rst_n(rst_n), .in_en(1'b1),
            .s0(w[0][0][7:0]), .s1(w[0][1][7:0]), .s2(w[0][2][7:0]),
            .s3(w[1][0][7:0]), .s4(w[1][1][7:0]), .s5(w[1][2][7:0]),
            .s6(w[2][0][7:0]), .s7(w[2][1][7:0]), .s8(w[2][2][7:0]), .med(med_b));
        wire [23:0] med_pix = {med_r, med_g, med_b};   // valid 4 cycles after the window

        // ===================== BRANCH B: gaussian 3x3 (4 stages, shift-add, 0 DSP) =====================
        // gA corners/edges/centre sums; gB total; gC >>4; gD hold -> 4 cycles, matches median.
        logic [9:0]  g_corner [0:2], g_edge [0:2];
        logic [7:0]  g_cen [0:2];
        logic [11:0] g_tot [0:2];
        logic [7:0]  g_q [0:2];
        logic [23:0] gauss_pix;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                for (int sc=0;sc<3;sc++) begin g_corner[sc]<='0; g_edge[sc]<='0; g_cen[sc]<='0; end
            end else for (int sc=0;sc<3;sc++) begin
                g_corner[sc] <= ch(w[0][0],sc)+ch(w[0][2],sc)+ch(w[2][0],sc)+ch(w[2][2],sc);
                g_edge[sc]   <= ch(w[0][1],sc)+ch(w[1][0],sc)+ch(w[1][2],sc)+ch(w[2][1],sc);
                g_cen[sc]    <= ch(w[1][1],sc);
            end
        end
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) for (int sc=0;sc<3;sc++) g_tot[sc]<='0;
            else for (int sc=0;sc<3;sc++)
                g_tot[sc] <= g_corner[sc] + (g_edge[sc]<<1) + (g_cen[sc]<<2);  // /16 numerator
        end
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) for (int sc=0;sc<3;sc++) g_q[sc]<='0;
            else for (int sc=0;sc<3;sc++) g_q[sc] <= g_tot[sc][11:4];          // >> 4
        end
        logic [23:0] gauss_pix2;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin gauss_pix <= '0; gauss_pix2 <= '0; end
            else begin gauss_pix <= {g_q[2], g_q[1], g_q[0]}; gauss_pix2 <= gauss_pix; end  // {R,G,B}, 5 cyc
        end

        // ===================== BRANCH C: point op (centre tap) + 5-stage delay-balance =====================
        logic [23:0] pt1, pt2, pt3, pt4, pt5;
        always_ff @(posedge clk or negedge rst_n) begin
            logic [7:0] r, g, b;
            if (!rst_n) pt1 <= '0;
            else begin
                r = w[1][1][23:16]; g = w[1][1][15:8]; b = w[1][1][7:0];   // centre; gray/thresh on green (g)
                unique case (cfg_op)
                    4'd1:    pt1 <= {~r, ~g, ~b};
                    4'd2:    pt1 <= {g, g, g};
                    4'd3:    pt1 <= {b, g, r};
                    4'd4:    pt1 <= (g > cfg_thresh_level) ? 24'hFFFFFF : 24'h000000;
                    4'd5:    pt1 <= {r, 8'd0, 8'd0};
                    4'd6:    pt1 <= {8'd0, g, 8'd0};
                    4'd7:    pt1 <= {8'd0, 8'd0, b};
                    default: pt1 <= {r, g, b};       // 0 and 10-15 = passthrough
                endcase
            end
        end
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin pt2<='0; pt3<='0; pt4<='0; pt5<='0; end
            else begin pt2<=pt1; pt3<=pt2; pt4<=pt3; pt5<=pt4; end
        end

        // ===================== markers: delay v2.. by 5 to align with the branches =====================
        logic [4:0] mk1, mk2, mk3, mk4, mk5;     // {v,s,e,f,r}
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin mk1<='0; mk2<='0; mk3<='0; mk4<='0; mk5<='0; end
            else begin
                mk1 <= {v2,s2,e2,f2,r2};
                mk2 <= mk1; mk3 <= mk2; mk4 <= mk3; mk5 <= mk4;
            end
        end

        // ===================== final registered output mux (branches all aligned at this point) =========
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                out_pixel <= '0; {out_valid,out_sof,out_eol,out_eof,out_err} <= '0;
            end else begin
                unique case (cfg_op)
                    4'd8:    out_pixel <= gauss_pix2;
                    4'd9:    out_pixel <= med_pix;
                    default: out_pixel <= pt5;        // 0-7 point/passthrough + reserved
                endcase
                {out_valid,out_sof,out_eol,out_eof,out_err} <= mk5;
            end
        end
    end endgenerate

endmodule

`default_nettype wire
