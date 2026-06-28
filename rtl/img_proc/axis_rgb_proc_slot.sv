`timescale 1ns / 1ps
`default_nettype none

// axis_rgb_proc_slot (2026-06-23, image-processing research base, Phase 2a).
// Standardised processing slot on the 24-bit RGB888 pixel stream (core_clk domain),
// inserted between the format-mux video_pixel and the capture/HDMI bridges. This is
// the slot CONTRACT: a pixel-stream in {pixel,valid,sof,eol,eof,err} -> the same out,
// 1-cycle registered, markers delayed to match. Phase 2a fills it with runtime-
// selectable POINT operations (no line buffers) to prove the slot + runtime control +
// verify flow end to end; Phase 2b adds the 3x3 line-buffer convolution to the same
// contract. cfg_op is driven from idelay GPIO bits[23:21] (a direct-read spare field,
// like cfg_settle_blank). ENABLE=0 = pure wire-through (zero logic, build-time bypass).
module axis_rgb_proc_slot #(
    parameter bit ENABLE = 1'b1
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [2:0]  cfg_op,        // 0=passthrough 1=invert 2=grayscale 3=BGR-swap
                                      // 4=threshold 5=R-only 6=G-only 7=B-only
    input  wire [7:0]  cfg_thresh_level, // op-4 threshold level (default driver = 8'd128)
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
    end else begin : g_proc
        wire [7:0] r = in_pixel[23:16];
        wire [7:0] g = in_pixel[15:8];
        wire [7:0] b = in_pixel[7:0];
        // gray = green channel (luma ~= 59% green; a multiplier-free approximation so
        // the slot stays trivial wire logic -- the full luma MAC was dropped because
        // it added sysclk-domain congestion that tipped the timing-edge design).
        wire [7:0] y = g;

        logic [23:0] op_pixel;
        always_comb begin
            unique case (cfg_op)
                3'd1: op_pixel = {~r, ~g, ~b};                 // invert
                3'd2: op_pixel = {y, y, y};                    // grayscale (green approx)
                3'd3: op_pixel = {b, g, r};                    // BGR swap (R<->B)
                3'd4: op_pixel = (y > cfg_thresh_level) ? 24'hFFFFFF
                                                        : 24'h000000;  // threshold (on green)
                3'd5: op_pixel = {r, 8'd0, 8'd0};              // R only
                3'd6: op_pixel = {8'd0, g, 8'd0};              // G only
                3'd7: op_pixel = {8'd0, 8'd0, b};              // B only
                default: op_pixel = {r, g, b};                 // 0 = passthrough
            endcase
        end

        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                out_pixel <= 24'h000000;
                out_valid <= 1'b0;
                out_sof   <= 1'b0;
                out_eol   <= 1'b0;
                out_eof   <= 1'b0;
                out_err   <= 1'b0;
            end else begin
                out_pixel <= op_pixel;     // op is combinational on the current beat
                out_valid <= in_valid;     // markers delayed 1 cycle to match the reg
                out_sof   <= in_sof;
                out_eol   <= in_eol;
                out_eof   <= in_eof;
                out_err   <= in_err;
            end
        end
    end endgenerate

endmodule

`default_nettype wire
