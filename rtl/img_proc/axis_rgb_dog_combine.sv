`timescale 1ns / 1ps
`default_nettype none

// axis_rgb_dog_combine (2026-06-24, DoG dual-kernel, Phase A).
// Combines two PARALLEL convolution branches of the SAME pixel stream into one output:
//   out = sat( (alpha*A - beta*B) >>> shift + offset )      (per RGB channel)
// A = small kernel (axis_rgb_conv3x3, LEADS), B = large kernel (axis_rgb_conv5x5, LAGS).
// Both branches emit one output per input pixel in identical raster order, so the k-th A
// output and k-th B output are the SAME spatial pixel; B simply trails A by a fixed
// latency (~1 line + 1 cycle). An ORDINAL alignment FIFO (push A on a_valid, pop on
// b_valid) pairs them with no fragile latency arithmetic -- depth just covers the lead.
// Output timing/markers follow B. Modes: 0=A passthrough, 1=B passthrough, 2=DoG(aA-bB),
// 3=sum(aA+bB). The 6 alpha/beta multiplies map to DSP48. Gives DoG / band-pass /
// multi-scale edge / unsharp from runtime params. DSim: tb_axis_rgb_dog.
module axis_rgb_dog_combine #(
    parameter bit ENABLE = 1'b1,
    parameter int DEPTH  = 1024            // > branch lead (LINE_PIXELS + a few); power of 2
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire [1:0]   cfg_mode,          // 0=A 1=B 2=DoG(aA-bB) 3=sum(aA+bB)
    input  wire [7:0]   cfg_alpha,
    input  wire [7:0]   cfg_beta,
    input  wire [3:0]   cfg_shift,
    input  wire signed [8:0] cfg_offset,   // -256..255
    // branch A (small kernel, leads) -- only pixel + valid are used (push into FIFO)
    input  wire [23:0]  a_pixel,
    input  wire         a_valid,
    // branch B (large kernel, lags) -- output follows B timing + markers
    input  wire [23:0]  b_pixel,
    input  wire         b_valid,
    input  wire         b_sof,
    input  wire         b_eol,
    input  wire         b_eof,
    input  wire         b_err,
    output logic [23:0] out_pixel,
    output logic        out_valid,
    output logic        out_sof,
    output logic        out_eol,
    output logic        out_eof,
    output logic        out_err
);

    generate if (!ENABLE) begin : g_bypass
        assign out_pixel = b_pixel;
        assign out_valid = b_valid;
        assign out_sof   = b_sof;
        assign out_eol   = b_eol;
        assign out_eof   = b_eof;
        assign out_err   = b_err;
    end else begin : g_comb
        localparam int DW = $clog2(DEPTH);

        // ---- ordinal alignment FIFO: store A pixels, pop one per B pixel ----
        // The RAM array access is kept in a RESET-FREE clocked block so it infers as a
        // simple-dual-port BRAM (an async reset on the array forces a huge FF bank).
        (* ram_style = "block" *) logic [23:0] afifo [0:DEPTH-1];
        logic [DW-1:0] wr, rd;
        logic [23:0] a_q1;
        always_ff @(posedge clk) begin
            if (a_valid) afifo[wr] <= a_pixel;   // write port  (no reset -> BRAM)
            if (b_valid) a_q1     <= afifo[rd];  // read  port  -> aligned A pixel
        end
        // FIFO pointers (reset-controlled); wr/rd use the same (pre-increment) value above
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin wr <= '0; rd <= '0; end
            else begin
                if (a_valid) wr <= wr + 1'b1;
                if (b_valid) rd <= rd + 1'b1;
            end
        end

        // stage 0 -> 1: register the B bundle aligned with a_q1
        logic [23:0] b_p1;
        logic b1_v, b1_sof, b1_eol, b1_eof, b1_err;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                b_p1 <= '0; {b1_v,b1_sof,b1_eol,b1_eof,b1_err} <= '0;
            end else begin
                b1_v   <= b_valid;
                b_p1   <= b_pixel;
                b1_sof <= b_sof; b1_eol <= b_eol; b1_eof <= b_eof; b1_err <= b_err;
            end
        end

        // stage 1 -> 2: alpha*A and beta*B products (6 multiplies -> DSP48)
        function automatic logic [7:0] ch(input logic [23:0] p, input int sel);
            ch = (sel==2) ? p[23:16] : (sel==1) ? p[15:8] : p[7:0];
        endfunction
        (* use_dsp = "yes" *) logic [15:0] pa [0:2];
        (* use_dsp = "yes" *) logic [15:0] pb [0:2];
        logic [23:0] a_q2, b_p2;
        logic p_v, p_sof, p_eol, p_eof, p_err;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                pa[0]<='0;pa[1]<='0;pa[2]<='0; pb[0]<='0;pb[1]<='0;pb[2]<='0;
                a_q2<='0; b_p2<='0; {p_v,p_sof,p_eol,p_eof,p_err} <= '0;
            end else begin
                for (int sc=0; sc<3; sc++) begin
                    pa[sc] <= cfg_alpha * ch(a_q1, sc);
                    pb[sc] <= cfg_beta  * ch(b_p1, sc);
                end
                a_q2 <= a_q1; b_p2 <= b_p1;
                p_v <= b1_v; p_sof <= b1_sof; p_eol <= b1_eol; p_eof <= b1_eof; p_err <= b1_err;
            end
        end

        // stage 2 -> 3: pre-combine sum/diff (alpha*A -/+ beta*B). Split off the long
        // output path -- this was the WNS critical net on sysclk (DSP -> sub -> shift ->
        // offset -> sat in one stage = -1.628 ns); registering 'sel' here halves it.
        logic signed [17:0] sel [0:2];
        logic [23:0] a_q3, b_p3;
        logic q_v, q_sof, q_eol, q_eof, q_err;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                sel[0]<='0; sel[1]<='0; sel[2]<='0; a_q3<='0; b_p3<='0;
                {q_v,q_sof,q_eol,q_eof,q_err} <= '0;
            end else begin
                for (int sc=0; sc<3; sc++)
                    sel[sc] <= (cfg_mode == 2'd3)
                        ? ($signed({2'b00, pa[sc]}) + $signed({2'b00, pb[sc]}))
                        : ($signed({2'b00, pa[sc]}) - $signed({2'b00, pb[sc]}));
                a_q3 <= a_q2; b_p3 <= b_p2;
                q_v <= p_v; q_sof <= p_sof; q_eol <= p_eol; q_eof <= p_eof; q_err <= p_err;
            end
        end

        // stage 3 -> out: shift + offset + saturate (or passthrough)
        function automatic logic [7:0] sat9(input logic signed [18:0] v);
            sat9 = (v < 0) ? 8'd0 : (v > 19'sd255) ? 8'd255 : v[7:0];
        endfunction
        function automatic logic [7:0] combine(input int sc);
            logic signed [18:0] val;
            unique case (cfg_mode)
                2'd0:    combine = ch(a_q3, sc);                       // A passthrough
                2'd1:    combine = ch(b_p3, sc);                       // B passthrough
                default: begin                                        // 2=DoG, 3=sum
                    val = (sel[sc] >>> cfg_shift) + cfg_offset;
                    combine = sat9(val);
                end
            endcase
        endfunction
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                out_pixel <= '0; {out_valid,out_sof,out_eol,out_eof,out_err} <= '0;
            end else begin
                out_pixel <= {combine(2), combine(1), combine(0)};
                out_valid <= q_v; out_sof <= q_sof; out_eol <= q_eol;
                out_eof <= q_eof; out_err <= q_err;
            end
        end
    end endgenerate

endmodule

`default_nettype wire
