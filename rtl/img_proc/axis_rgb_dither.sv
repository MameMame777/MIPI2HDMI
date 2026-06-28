`timescale 1ns / 1ps
`default_nettype none

// axis_rgb_dither (2026-06-26): final-stage dither + bit-depth quantizer, placed AFTER the POST
// point-op slot, before the capture/HDMI bridge. Per channel:
//   out = replicate_to_8b( quantize_to_N( clamp(in + bias) ) )
// where `bias` is a position-dependent ORDERED (Bayer 4x4) value or a per-pixel RANDOM (LFSR)
// value, scaled to the dropped-LSB range. This reduces a smooth gradient to N bits/channel while
// dithering away the banding: N=1 = halftone (0/255), N=2..4 = posterize/retro, N=6 = anti-banding
// for low-bit panels. Stateless (no line buffers) -> tiny, fixed 1-cycle latency, markers delayed
// to match. cfg_ctrl[0]=0 -> registered passthrough (bit-identical). ENABLE=0 -> wire-through.
// DSim: verification/tb/tb_axis_rgb_dither.
module axis_rgb_dither #(
    parameter int LINE_PIXELS = 640,
    parameter bit ENABLE      = 1'b1
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  cfg_ctrl,   // [0]=enable [1]=mode(0=ordered/1=random) [4:2]=bits/ch N(1..6)
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
    end else begin : g_dith
        localparam int AW = $clog2(LINE_PIXELS);

        wire        en   = cfg_ctrl[0];
        wire        mode = cfg_ctrl[1];      // 0=ordered(Bayer) 1=random(LFSR)
        wire [2:0]  nb   = cfg_ctrl[4:2];    // bits per channel to keep (1..6; 0 or >=7 = passthrough)

        // x/y position for the ordered Bayer matrix (col like conv3x3; add a row counter)
        logic [AW-1:0] col, row;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin col <= '0; row <= '0; end
            else if (in_valid) begin
                col <= in_eol ? '0 : col + 1'b1;
                row <= in_eof ? '0 : (in_eol ? row + 1'b1 : row);
            end
        end

        // per-pixel 8-bit Galois LFSR (maximal-length, taps 0x1D), advanced on valid
        logic [7:0] lfsr;
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n)        lfsr <= 8'hA5;                 // nonzero seed
            else if (in_valid) lfsr <= {lfsr[6:0], 1'b0} ^ (lfsr[7] ? 8'h1D : 8'h00);
        end

        // Bayer 4x4 threshold (value 0..15), indexed by {row[1:0], col[1:0]}
        function automatic logic [3:0] bayer4(input logic [1:0] yy, input logic [1:0] xx);
            unique case ({yy, xx})
                4'h0: bayer4 = 4'd0;  4'h1: bayer4 = 4'd8;  4'h2: bayer4 = 4'd2;  4'h3: bayer4 = 4'd10;
                4'h4: bayer4 = 4'd12; 4'h5: bayer4 = 4'd4;  4'h6: bayer4 = 4'd14; 4'h7: bayer4 = 4'd6;
                4'h8: bayer4 = 4'd3;  4'h9: bayer4 = 4'd11; 4'hA: bayer4 = 4'd1;  4'hB: bayer4 = 4'd9;
                4'hC: bayer4 = 4'd15; 4'hD: bayer4 = 4'd7;  4'hE: bayer4 = 4'd13; 4'hF: bayer4 = 4'd5;
            endcase
        endfunction

        // dither + quantize one 8-bit channel -> N bits, replicated back to full 8-bit range
        function automatic logic [7:0] dith_ch(input logic [7:0] v, input logic [3:0] by,
                                               input logic [7:0] rnd, input logic md,
                                               input logic [2:0] n);
            logic [3:0] drop;
            logic [8:0] bias9, sum;
            logic [7:0] mask, q, o;
            if (n == 3'd0 || n >= 3'd7) begin
                dith_ch = v;                                  // passthrough (no quantization)
            end else begin
                drop = 4'd8 - {1'b0, n};
                if (md)            bias9 = {1'b0, rnd} & ((9'd1 << drop) - 9'd1);   // random
                else if (drop >= 4) bias9 = {5'd0, by} << (drop - 3'd4);             // ordered hi-drop
                else                bias9 = {5'd0, by} >> (3'd4 - drop);             // ordered lo-drop
                sum = {1'b0, v} + bias9;
                if (sum > 9'd255) sum = 9'd255;               // clamp
                mask = (8'd1 << drop) - 8'd1;
                q = sum[7:0] & ~mask;                          // keep top N bits
                // smear the N MSBs down to fill 8 bits (full-range: N=1 -> 0/255 etc.)
                o = q;
                o = o | (o >> n);
                o = o | (o >> (n << 1));
                o = o | (o >> (n << 2));
                dith_ch = o;
            end
        endfunction

        logic [23:0] op_pixel;
        always_comb begin
            logic [3:0] by;
            by = bayer4(row[1:0], col[1:0]);
            if (en)
                op_pixel = { dith_ch(in_pixel[23:16], by, lfsr, mode, nb),
                             dith_ch(in_pixel[15:8],  by, lfsr, mode, nb),
                             dith_ch(in_pixel[7:0],   by, lfsr, mode, nb) };
            else
                op_pixel = in_pixel;
        end

        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                out_pixel <= '0; {out_valid, out_sof, out_eol, out_eof, out_err} <= '0;
            end else begin
                out_pixel <= op_pixel;
                out_valid <= in_valid; out_sof <= in_sof; out_eol <= in_eol;
                out_eof   <= in_eof;   out_err <= in_err;
            end
        end
    end endgenerate

endmodule

`default_nettype wire
