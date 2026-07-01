
`timescale 1ns / 1ps
`default_nettype none
// Auto-generated cascade wrapper for the cocotb port of tb_axis_rgb_cascade.sv.
// Contains ONLY the four DUT instances (no initial / no clock) so cocotb owns clk/rst and
// stimulus. Wiring is 1:1 with the DSim TB.
module cascade_harness #(
    parameter int W = 8
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire [23:0]  in_pixel,
    input  wire         in_valid,
    input  wire         in_sof,
    input  wire         in_eol,
    input  wire         in_eof,
    input  wire         in_err,
    // S1 general 5x5 config
    input  wire [199:0] c1,
    input  wire [3:0]   sh1,
    // S2 / S3 separable config
    input  wire [39:0]  h2, v2, h3, v3,
    input  wire [3:0]   hs2, vs2, hs3, vs3,
    // taps
    output wire [23:0]  t1, t2, t3, dg,
    output wire         t1v, t1s, t1e, t1f, t1r,
    output wire         t2v, t2s, t2e, t2f, t2r,
    output wire         t3v, t3s, t3e, t3f, t3r,
    output wire         dgv, dgs, dge, dgf, dgr
);
    axis_rgb_conv5x5 #(.LINE_PIXELS(W), .ENABLE(1'b1)) S1 (
        .clk(clk), .rst_n(rst_n), .cfg_en(1'b1), .cfg_coeffs(c1), .cfg_shift(sh1), .cfg_abs(1'b0),
        .in_pixel(in_pixel), .in_valid(in_valid), .in_sof(in_sof),
        .in_eol(in_eol), .in_eof(in_eof), .in_err(in_err),
        .out_pixel(t1), .out_valid(t1v), .out_sof(t1s), .out_eol(t1e), .out_eof(t1f), .out_err(t1r));
    axis_rgb_conv5x5_sep #(.LINE_PIXELS(W), .ENABLE(1'b1)) S2 (
        .clk(clk), .rst_n(rst_n), .cfg_h(h2), .cfg_v(v2), .cfg_hshift(hs2), .cfg_vshift(vs2),
        .in_pixel(t1), .in_valid(t1v), .in_sof(t1s), .in_eol(t1e), .in_eof(t1f), .in_err(t1r),
        .out_pixel(t2), .out_valid(t2v), .out_sof(t2s), .out_eol(t2e), .out_eof(t2f), .out_err(t2r));
    axis_rgb_conv5x5_sep #(.LINE_PIXELS(W), .ENABLE(1'b1)) S3 (
        .clk(clk), .rst_n(rst_n), .cfg_h(h3), .cfg_v(v3), .cfg_hshift(hs3), .cfg_vshift(vs3),
        .in_pixel(t2), .in_valid(t2v), .in_sof(t2s), .in_eol(t2e), .in_eof(t2f), .in_err(t2r),
        .out_pixel(t3), .out_valid(t3v), .out_sof(t3s), .out_eol(t3e), .out_eof(t3f), .out_err(t3r));
    axis_rgb_dog_combine #(.ENABLE(1'b1), .DEPTH(64)) DG (
        .clk(clk), .rst_n(rst_n), .cfg_mode(2'd2), .cfg_alpha(8'd1), .cfg_beta(8'd1),
        .cfg_shift(4'd0), .cfg_offset(9'sd128),
        .a_pixel(t1), .a_valid(t1v), .b_pixel(t3), .b_valid(t3v),
        .b_sof(t3s), .b_eol(t3e), .b_eof(t3f), .b_err(t3r),
        .out_pixel(dg), .out_valid(dgv), .out_sof(dgs), .out_eol(dge), .out_eof(dgf), .out_err(dgr));
endmodule
`default_nettype wire
