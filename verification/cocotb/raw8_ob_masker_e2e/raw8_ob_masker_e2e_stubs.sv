`timescale 1ns / 1ps

// Synthesizable E2E wrapper for the raw8_ob_masker_e2e cocotb port.
//
// The DSim tb_raw8_ob_masker_e2e.sv wires seven RTL instances together inside the
// testbench module itself (there is no RTL top). cocotb/Verilator needs a single
// synthesizable toplevel whose ports it can drive/observe, so this module reproduces
// the TB's exact internal instantiation (identical parameters and port bindings) and
// exposes only:
//   - the s_byte_* byte-beat inputs (driven by the cocotb ByteBeat driver)
//   - the ob_* pixel-stream outputs (captured pixel-by-pixel, verified vs expected_y)
//   - the status counters the TB asserts on
//       (parser short/long counts, crc err count, last_frame_lines, frame_count)
//
// No stimulus and no checks live here; those are in test_raw8_ob_masker_e2e.py.
// The full pixel-path logic is entirely the real rtl/ modules (unchanged).

module raw8_ob_masker_e2e_top #(
    parameter int PARSER_IN_WIDTH = 16,
    parameter int LINE_PIXELS     = 8,
    parameter int FRAME_LINES     = 4,
    parameter int LINE_BYTES      = LINE_PIXELS
) (
    input  logic                          core_clk,
    input  logic                          core_aresetn,

    input  logic [PARSER_IN_WIDTH-1:0]    s_byte_data,
    input  logic [PARSER_IN_WIDTH/8-1:0]  s_byte_keep,
    input  logic                          s_byte_valid,
    input  logic                          s_byte_sop,
    input  logic                          s_byte_eop,

    output logic [7:0]                    ob_pixel,
    output logic                          ob_valid,
    output logic                          ob_sof,
    output logic                          ob_eol,
    output logic                          ob_eof,
    output logic                          ob_err,

    output logic [15:0]                   parser_short_count,
    output logic [15:0]                   parser_long_count,
    output logic [15:0]                   crc_err_count,
    output logic [15:0]                   last_frame_lines,
    output logic [31:0]                   frame_count
);
    localparam logic [5:0] DT_RAW8 = 6'h2A;

    // ---- internal nets (mirror the TB) ---------------------------------------------
    logic        parser_ecc_hdr_valid;
    logic [31:0] parser_ecc_hdr_raw;
    logic        ecc_hdr_corr_valid;
    logic [23:0] ecc_hdr_corr;
    logic [7:0]  ecc_hdr_di;
    logic [15:0] ecc_hdr_wc;
    logic        ecc_hdr_corrected, ecc_hdr_uncorrectable, ecc_hdr_no_error;
    logic [15:0] ecc_corr_count, ecc_uncorr_count;
    logic        pkt_hdr_valid;
    logic [31:0] pkt_hdr_raw;
    logic [7:0]  pkt_di;
    logic [15:0] pkt_wc;
    logic        pkt_is_long, pkt_is_short, pkt_ecc_uncorrectable;
    logic [7:0]  payload_data;
    logic        payload_valid, payload_first, payload_last;
    logic [15:0] footer_data;
    logic        footer_valid, pkt_done;
    logic [15:0] parser_trunc_count;
    logic        crc_check_valid, crc_match;
    logic [15:0] crc_calc, crc_received, crc_ok_count;
    logic [7:0]  filter_pkt_di;
    logic [15:0] filter_pkt_wc;
    logic        filter_pkt_is_short, filter_pkt_is_long, filter_pkt_start, filter_pkt_end, filter_pkt_err;
    logic [7:0]  filter_payload_data;
    logic        filter_payload_valid, filter_payload_first, filter_payload_last;
    logic [15:0] filter_drop_vc_count, filter_drop_dt_count;
    logic        frame_sof, frame_eof, frame_sol, frame_eol;
    logic [15:0] frame_line_idx;
    logic [7:0]  frame_payload_data;
    logic        frame_payload_valid, frame_payload_first, frame_payload_last, frame_err;
    logic [31:0] line_count;
    logic [15:0] frame_sync_err_count;

    logic [7:0]  raw8_pixel;
    logic        raw8_valid, raw8_sof, raw8_eol, raw8_eof, raw8_err;
    logic [15:0] raw8_pix_per_line;

    // ---- instances (verbatim from tb_raw8_ob_masker_e2e.sv) -------------------------
    csi2_packet_parser #(
        .IN_WIDTH(PARSER_IN_WIDTH), .WC_MAX(256), .FIFO_DEPTH(256)
    ) u_parser (
        .core_clk(core_clk), .core_aresetn(core_aresetn),
        .s_byte_data(s_byte_data), .s_byte_keep(s_byte_keep),
        .s_byte_valid(s_byte_valid), .s_byte_sop(s_byte_sop), .s_byte_eop(s_byte_eop),
        .ecc_hdr_valid(parser_ecc_hdr_valid), .ecc_hdr_raw(parser_ecc_hdr_raw),
        .ecc_hdr_corr_valid(ecc_hdr_corr_valid), .ecc_hdr_di(ecc_hdr_di), .ecc_hdr_wc(ecc_hdr_wc),
        .ecc_hdr_uncorrectable(ecc_hdr_uncorrectable),
        .m_pkt_hdr_valid(pkt_hdr_valid), .m_pkt_hdr_raw(pkt_hdr_raw),
        .m_pkt_di(pkt_di), .m_pkt_wc(pkt_wc),
        .m_pkt_is_long(pkt_is_long), .m_pkt_is_short(pkt_is_short),
        .m_pkt_ecc_uncorrectable(pkt_ecc_uncorrectable),
        .m_payload_data(payload_data), .m_payload_valid(payload_valid),
        .m_payload_first(payload_first), .m_payload_last(payload_last),
        .m_footer_data(footer_data), .m_footer_valid(footer_valid), .m_pkt_done(pkt_done),
        .sts_short_pkt_cnt(parser_short_count), .sts_long_pkt_cnt(parser_long_count),
        .sts_pkt_trunc_cnt(parser_trunc_count)
    );

    csi2_header_ecc u_header_ecc (
        .core_clk(core_clk), .core_aresetn(core_aresetn),
        .hdr_valid(parser_ecc_hdr_valid), .hdr_raw(parser_ecc_hdr_raw),
        .hdr_corr_valid(ecc_hdr_corr_valid), .hdr_corr(ecc_hdr_corr),
        .hdr_di(ecc_hdr_di), .hdr_wc(ecc_hdr_wc),
        .hdr_ecc_corrected(ecc_hdr_corrected), .hdr_ecc_uncorrectable(ecc_hdr_uncorrectable),
        .hdr_ecc_no_error(ecc_hdr_no_error),
        .sts_ecc_corr_cnt(ecc_corr_count), .sts_ecc_uncorr_cnt(ecc_uncorr_count)
    );

    csi2_payload_crc u_payload_crc (
        .core_clk(core_clk), .core_aresetn(core_aresetn),
        .payload_data(payload_data), .payload_valid(payload_valid),
        .payload_first(payload_first), .payload_last(payload_last),
        .footer_data(footer_data), .footer_valid(footer_valid),
        .crc_check_valid(crc_check_valid), .crc_match(crc_match),
        .crc_calc(crc_calc), .crc_received(crc_received),
        .sts_crc_err_cnt(crc_err_count), .sts_crc_ok_cnt(crc_ok_count)
    );

    csi2_vcdt_filter u_filter (
        .core_clk(core_clk), .core_aresetn(core_aresetn),
        .cfg_expected_vc(2'b00), .cfg_expected_dt(DT_RAW8),
        .cfg_pass_short(1'b1), .cfg_pass_emb_data(1'b0),
        .pkt_hdr_valid(pkt_hdr_valid), .pkt_di(pkt_di), .pkt_wc(pkt_wc),
        .pkt_is_long(pkt_is_long), .pkt_is_short(pkt_is_short), .pkt_done(pkt_done),
        .ecc_corrected(ecc_hdr_corrected), .ecc_uncorrectable(pkt_ecc_uncorrectable),
        .crc_check_valid(crc_check_valid), .crc_match(crc_match),
        .payload_data(payload_data), .payload_valid(payload_valid),
        .payload_first(payload_first), .payload_last(payload_last),
        .out_pkt_di(filter_pkt_di), .out_pkt_wc(filter_pkt_wc),
        .out_pkt_is_short(filter_pkt_is_short), .out_pkt_is_long(filter_pkt_is_long),
        .out_pkt_start(filter_pkt_start), .out_pkt_end(filter_pkt_end), .out_pkt_err(filter_pkt_err),
        .out_payload_data(filter_payload_data), .out_payload_valid(filter_payload_valid),
        .out_payload_first(filter_payload_first), .out_payload_last(filter_payload_last),
        .sts_drop_vc_cnt(filter_drop_vc_count), .sts_drop_dt_cnt(filter_drop_dt_count)
    );

    csi2_frame_state #(
        .MAX_LINES(16), .GUARD_FRAME_LINES(1'b1),
        .EXPECTED_FRAME_LINES(FRAME_LINES), .EXPECTED_LINE_WC(16'(LINE_BYTES))
    ) u_frame_state (
        .core_clk(core_clk), .core_aresetn(core_aresetn),
        .cfg_use_lsle(1'b0), .cfg_expected_frame_lines(16'd0),
        .in_pkt_di(filter_pkt_di), .in_pkt_wc(filter_pkt_wc),
        .in_pkt_is_short(filter_pkt_is_short), .in_pkt_is_long(filter_pkt_is_long),
        .in_pkt_start(filter_pkt_start), .in_pkt_end(filter_pkt_end), .in_pkt_err(filter_pkt_err),
        .in_payload_data(filter_payload_data), .in_payload_valid(filter_payload_valid),
        .in_payload_first(filter_payload_first), .in_payload_last(filter_payload_last),
        .out_sof(frame_sof), .out_eof(frame_eof), .out_sol(frame_sol), .out_eol(frame_eol),
        .out_line_idx(frame_line_idx),
        .out_payload_data(frame_payload_data), .out_payload_valid(frame_payload_valid),
        .out_payload_first(frame_payload_first), .out_payload_last(frame_payload_last),
        .out_frame_err(frame_err),
        .sts_frame_count(frame_count), .sts_line_count(line_count),
        .sts_last_frame_lines(last_frame_lines), .sts_frame_sync_err_cnt(frame_sync_err_count)
    );

    raw8_passthrough #(.LINE_PIXELS(LINE_PIXELS)) u_raw8 (
        .core_clk(core_clk), .core_aresetn(core_aresetn),
        .in_sof(frame_sof), .in_eof(frame_eof), .in_eol(frame_eol),
        .in_payload_data(frame_payload_data), .in_payload_valid(frame_payload_valid),
        .in_payload_first(frame_payload_first), .in_payload_last(frame_payload_last),
        .in_frame_err(frame_err),
        .out_pixel(raw8_pixel), .out_pixel_valid(raw8_valid),
        .out_pixel_sof(raw8_sof), .out_pixel_eol(raw8_eol),
        .out_pixel_eof(raw8_eof), .out_pixel_err(raw8_err),
        .sts_pixel_per_line(raw8_pix_per_line)
    );

    ob_row_masker #(.WIDTH(8), .LINE_PIXELS_MAX(64)) u_ob (
        .clk(core_clk), .aresetn(core_aresetn), .enable(1'b1),
        .in_data(raw8_pixel), .in_valid(raw8_valid),
        .in_sof(raw8_sof), .in_eol(raw8_eol),
        .in_eof(raw8_eof), .in_err(raw8_err),
        .out_data(ob_pixel), .out_valid(ob_valid),
        .out_sof(ob_sof), .out_eol(ob_eol),
        .out_eof(ob_eof), .out_err(ob_err)
    );

endmodule
