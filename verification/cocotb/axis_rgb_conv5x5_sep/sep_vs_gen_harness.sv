
`timescale 1ns / 1ps
`default_nettype none
// Auto-generated wrapper for the cocotb port of tb_axis_rgb_conv5x5_sep.sv.
// Contains ONLY the two DUT instances (no initial / no clock) so cocotb owns clk/rst and
// stimulus. Wiring is 1:1 with the DSim TB: both DUTs see the same input stream; the general
// 5x5 is the reference model the separable DUT is checked against.
module sep_vs_gen_harness #(
    parameter int LINE_PIXELS = 8
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire [39:0]  cfg_h,
    input  wire [39:0]  cfg_v,
    input  wire [3:0]   cfg_hshift,
    input  wire [3:0]   cfg_vshift,
    input  wire [199:0] gcoef,
    input  wire [23:0]  in_pixel,
    input  wire         in_valid,
    input  wire         in_sof,
    input  wire         in_eol,
    input  wire         in_eof,
    input  wire         in_err,
    output wire [23:0]  sep_pixel,
    output wire         sep_valid, sep_sof, sep_eol, sep_eof, sep_err,
    output wire [23:0]  gen_pixel,
    output wire         gen_valid, gen_sof, gen_eol, gen_eof, gen_err
);
    axis_rgb_conv5x5_sep #(.LINE_PIXELS(LINE_PIXELS), .ENABLE(1'b1)) dut (
        .clk(clk), .rst_n(rst_n), .cfg_h(cfg_h), .cfg_v(cfg_v),
        .cfg_hshift(cfg_hshift), .cfg_vshift(cfg_vshift),
        .in_pixel(in_pixel), .in_valid(in_valid), .in_sof(in_sof),
        .in_eol(in_eol), .in_eof(in_eof), .in_err(in_err),
        .out_pixel(sep_pixel), .out_valid(sep_valid), .out_sof(sep_sof),
        .out_eol(sep_eol), .out_eof(sep_eof), .out_err(sep_err));
    axis_rgb_conv5x5 #(.LINE_PIXELS(LINE_PIXELS), .ENABLE(1'b1)) gen (
        .clk(clk), .rst_n(rst_n), .cfg_en(1'b1), .cfg_coeffs(gcoef), .cfg_shift(4'd8), .cfg_abs(1'b0),
        .in_pixel(in_pixel), .in_valid(in_valid), .in_sof(in_sof),
        .in_eol(in_eol), .in_eof(in_eof), .in_err(in_err),
        .out_pixel(gen_pixel), .out_valid(gen_valid), .out_sof(gen_sof),
        .out_eol(gen_eol), .out_eof(gen_eof), .out_err(gen_err));
endmodule
`default_nettype wire
