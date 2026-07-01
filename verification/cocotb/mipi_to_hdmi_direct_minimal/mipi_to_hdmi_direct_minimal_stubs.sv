`timescale 1ns / 1ps
`default_nettype none
// -----------------------------------------------------------------------------
// Auto-generated E2E harness for the cocotb port of
// tb_mipi_to_hdmi_direct_minimal.sv.
//
// Contains ONLY the DUT instances (no `initial`, no clock generators, no checks)
// so cocotb owns the clocks, resets, byte-beat stimulus and every check. The
// wiring + parameters are 1:1 with the DSim TB instantiations (lines 134-360 of
// tb_mipi_to_hdmi_direct_minimal.sv): csi2_packet_parser -> csi2_header_ecc ->
// csi2_payload_crc -> csi2_vcdt_filter -> csi2_frame_state -> yuv422_gray_unpack
// -> axis_video_bridge -> hdmi_output.
//
// The header-ECC module is REAL RTL in this TB (not a peer model), so the full
// combinational ECC loop is preserved here exactly as the DSim TB wired it.
// -----------------------------------------------------------------------------
module mipi_direct_minimal_harness #(
    parameter int PARSER_IN_WIDTH = 16,
    parameter logic [5:0] DT_YUV422 = 6'h1e
)(
    input  wire                        core_clk,
    input  wire                        core_aresetn,
    input  wire                        pix_clk,
    input  wire                        pix_aresetn,

    // parser byte-beat inputs (driven by cocotb)
    input  wire [PARSER_IN_WIDTH-1:0]  s_byte_data,
    input  wire [PARSER_IN_WIDTH/8-1:0] s_byte_keep,
    input  wire                        s_byte_valid,
    input  wire                        s_byte_sop,
    input  wire                        s_byte_eop,

    // hdmi enable (driven by cocotb)
    input  wire                        hdmi_enable,

    // ---- observed: parser status ----
    output wire [15:0]                 parser_short_count,
    output wire [15:0]                 parser_long_count,
    output wire [15:0]                 parser_trunc_count,

    // ---- observed: header ECC status ----
    output wire [15:0]                 ecc_uncorr_count,

    // ---- observed: payload CRC status ----
    output wire [15:0]                 crc_err_count,
    output wire [15:0]                 crc_ok_count,

    // ---- observed: vc/dt filter status ----
    output wire [15:0]                 filter_drop_vc_count,
    output wire [15:0]                 filter_drop_dt_count,

    // ---- observed: frame state ----
    output wire [31:0]                 frame_count,
    output wire [31:0]                 line_count,
    output wire [15:0]                 last_frame_lines,
    output wire [15:0]                 frame_sync_err_count,

    // ---- observed: yuv unpack (pixel stream) ----
    output wire [23:0]                 yuv_pixel,
    output wire                        yuv_pixel_valid,
    output wire                        yuv_pixel_sof,
    output wire                        yuv_pixel_eol,
    output wire                        yuv_pixel_eof,
    output wire                        yuv_pixel_err,
    output wire [15:0]                 yuv_pixel_per_line,

    // ---- observed: axis bridge (aclk = pix_clk domain) ----
    output wire [23:0]                 axis_tdata,
    output wire                        axis_tvalid,
    output wire                        axis_tready,
    output wire                        axis_tlast,
    output wire [1:0]                  axis_tuser,
    output wire [15:0]                 bridge_overflow_count,
    output wire [15:0]                 bridge_back_pressure_count,

    // ---- observed: hdmi output ----
    output wire [7:0]                  video_r,
    output wire [7:0]                  video_g,
    output wire [7:0]                  video_b,
    output wire                        video_de,
    output wire                        video_hsync,
    output wire                        video_vsync,
    output wire                        hdmi_running,
    output wire [31:0]                 hdmi_frame_count,
    output wire [15:0]                 hdmi_underflow_count,
    output wire [15:0]                 hdmi_axis_error_count
);

    // ---- parser <-> header ECC combinational loop nets ----
    wire        parser_ecc_hdr_valid;
    wire [31:0] parser_ecc_hdr_raw;
    wire        ecc_hdr_corr_valid;
    wire [23:0] ecc_hdr_corr;
    wire [7:0]  ecc_hdr_di;
    wire [15:0] ecc_hdr_wc;
    wire        ecc_hdr_corrected;
    wire        ecc_hdr_uncorrectable;
    wire        ecc_hdr_no_error;
    wire [15:0] ecc_corr_count;

    // ---- parser packet outputs ----
    wire        pkt_hdr_valid;
    wire [31:0] pkt_hdr_raw;
    wire [7:0]  pkt_di;
    wire [15:0] pkt_wc;
    wire        pkt_is_long;
    wire        pkt_is_short;
    wire        pkt_ecc_uncorrectable;
    wire [7:0]  payload_data;
    wire        payload_valid;
    wire        payload_first;
    wire        payload_last;
    wire [15:0] footer_data;
    wire        footer_valid;
    wire        pkt_done;

    // ---- crc outputs ----
    wire        crc_check_valid;
    wire        crc_match;
    wire [15:0] crc_calc;
    wire [15:0] crc_received;

    // ---- filter outputs ----
    wire [7:0]  filter_pkt_di;
    wire [15:0] filter_pkt_wc;
    wire        filter_pkt_is_short;
    wire        filter_pkt_is_long;
    wire        filter_pkt_start;
    wire        filter_pkt_end;
    wire        filter_pkt_err;
    wire [7:0]  filter_payload_data;
    wire        filter_payload_valid;
    wire        filter_payload_first;
    wire        filter_payload_last;

    // ---- frame state outputs ----
    wire        frame_sof;
    wire        frame_eof;
    wire        frame_sol;
    wire        frame_eol;
    wire [15:0] frame_line_idx;
    wire [7:0]  frame_payload_data;
    wire        frame_payload_valid;
    wire        frame_payload_first;
    wire        frame_payload_last;
    wire        frame_err;

    // ---- hdmi tmds words (unused by checks) ----
    wire [9:0]  tmds_data_0;
    wire [9:0]  tmds_data_1;
    wire [9:0]  tmds_data_2;
    wire [9:0]  tmds_clk_word;
    wire        hdmi_hpd_seen;

    csi2_packet_parser #(
        .IN_WIDTH(PARSER_IN_WIDTH),
        .WC_MAX(16),
        .FIFO_DEPTH(32)
    ) u_parser (
        .core_clk(core_clk),
        .core_aresetn(core_aresetn),
        .s_byte_data(s_byte_data),
        .s_byte_keep(s_byte_keep),
        .s_byte_valid(s_byte_valid),
        .s_byte_sop(s_byte_sop),
        .s_byte_eop(s_byte_eop),
        .ecc_hdr_valid(parser_ecc_hdr_valid),
        .ecc_hdr_raw(parser_ecc_hdr_raw),
        .ecc_hdr_corr_valid(ecc_hdr_corr_valid),
        .ecc_hdr_di(ecc_hdr_di),
        .ecc_hdr_wc(ecc_hdr_wc),
        .ecc_hdr_uncorrectable(ecc_hdr_uncorrectable),
        .m_pkt_hdr_valid(pkt_hdr_valid),
        .m_pkt_hdr_raw(pkt_hdr_raw),
        .m_pkt_di(pkt_di),
        .m_pkt_wc(pkt_wc),
        .m_pkt_is_long(pkt_is_long),
        .m_pkt_is_short(pkt_is_short),
        .m_pkt_ecc_uncorrectable(pkt_ecc_uncorrectable),
        .m_payload_data(payload_data),
        .m_payload_valid(payload_valid),
        .m_payload_first(payload_first),
        .m_payload_last(payload_last),
        .m_footer_data(footer_data),
        .m_footer_valid(footer_valid),
        .m_pkt_done(pkt_done),
        .sts_short_pkt_cnt(parser_short_count),
        .sts_long_pkt_cnt(parser_long_count),
        .sts_pkt_trunc_cnt(parser_trunc_count)
    );

    csi2_header_ecc u_header_ecc (
        .core_clk(core_clk),
        .core_aresetn(core_aresetn),
        .hdr_valid(parser_ecc_hdr_valid),
        .hdr_raw(parser_ecc_hdr_raw),
        .hdr_corr_valid(ecc_hdr_corr_valid),
        .hdr_corr(ecc_hdr_corr),
        .hdr_di(ecc_hdr_di),
        .hdr_wc(ecc_hdr_wc),
        .hdr_ecc_corrected(ecc_hdr_corrected),
        .hdr_ecc_uncorrectable(ecc_hdr_uncorrectable),
        .hdr_ecc_no_error(ecc_hdr_no_error),
        .sts_ecc_corr_cnt(ecc_corr_count),
        .sts_ecc_uncorr_cnt(ecc_uncorr_count)
    );

    csi2_payload_crc u_payload_crc (
        .core_clk(core_clk),
        .core_aresetn(core_aresetn),
        .payload_data(payload_data),
        .payload_valid(payload_valid),
        .payload_first(payload_first),
        .payload_last(payload_last),
        .footer_data(footer_data),
        .footer_valid(footer_valid),
        .crc_check_valid(crc_check_valid),
        .crc_match(crc_match),
        .crc_calc(crc_calc),
        .crc_received(crc_received),
        .sts_crc_err_cnt(crc_err_count),
        .sts_crc_ok_cnt(crc_ok_count)
    );

    csi2_vcdt_filter u_filter (
        .core_clk(core_clk),
        .core_aresetn(core_aresetn),
        .cfg_expected_vc(2'b00),
        .cfg_expected_dt(DT_YUV422),
        .cfg_pass_short(1'b1),
        .cfg_pass_emb_data(1'b0),
        .pkt_hdr_valid(pkt_hdr_valid),
        .pkt_di(pkt_di),
        .pkt_wc(pkt_wc),
        .pkt_is_long(pkt_is_long),
        .pkt_is_short(pkt_is_short),
        .pkt_done(pkt_done),
        .ecc_corrected(ecc_hdr_corrected),
        .ecc_uncorrectable(pkt_ecc_uncorrectable),
        .crc_check_valid(crc_check_valid),
        .crc_match(crc_match),
        .payload_data(payload_data),
        .payload_valid(payload_valid),
        .payload_first(payload_first),
        .payload_last(payload_last),
        .out_pkt_di(filter_pkt_di),
        .out_pkt_wc(filter_pkt_wc),
        .out_pkt_is_short(filter_pkt_is_short),
        .out_pkt_is_long(filter_pkt_is_long),
        .out_pkt_start(filter_pkt_start),
        .out_pkt_end(filter_pkt_end),
        .out_pkt_err(filter_pkt_err),
        .out_payload_data(filter_payload_data),
        .out_payload_valid(filter_payload_valid),
        .out_payload_first(filter_payload_first),
        .out_payload_last(filter_payload_last),
        .sts_drop_vc_cnt(filter_drop_vc_count),
        .sts_drop_dt_cnt(filter_drop_dt_count)
    );

    csi2_frame_state #(
        .MAX_LINES(8)
    ) u_frame_state (
        .core_clk(core_clk),
        .core_aresetn(core_aresetn),
        .cfg_use_lsle(1'b0),
        .cfg_expected_frame_lines(16'd0),
        .in_pkt_di(filter_pkt_di),
        .in_pkt_wc(filter_pkt_wc),
        .in_pkt_is_short(filter_pkt_is_short),
        .in_pkt_is_long(filter_pkt_is_long),
        .in_pkt_start(filter_pkt_start),
        .in_pkt_end(filter_pkt_end),
        .in_pkt_err(filter_pkt_err),
        .in_payload_data(filter_payload_data),
        .in_payload_valid(filter_payload_valid),
        .in_payload_first(filter_payload_first),
        .in_payload_last(filter_payload_last),
        .out_sof(frame_sof),
        .out_eof(frame_eof),
        .out_sol(frame_sol),
        .out_eol(frame_eol),
        .out_line_idx(frame_line_idx),
        .out_payload_data(frame_payload_data),
        .out_payload_valid(frame_payload_valid),
        .out_payload_first(frame_payload_first),
        .out_payload_last(frame_payload_last),
        .out_frame_err(frame_err),
        .sts_frame_count(frame_count),
        .sts_line_count(line_count),
        .sts_last_frame_lines(last_frame_lines),
        .sts_frame_sync_err_cnt(frame_sync_err_count)
    );

    yuv422_gray_unpack #(
        .LINE_PIXELS(2)
    ) u_yuv_unpack (
        .core_clk(core_clk),
        .core_aresetn(core_aresetn),
        .in_sof(frame_sof),
        .in_eof(frame_eof),
        .in_eol(frame_eol),
        .in_payload_data(frame_payload_data),
        .in_payload_valid(frame_payload_valid),
        .in_payload_first(frame_payload_first),
        .in_payload_last(frame_payload_last),
        .in_frame_err(frame_err),
        .out_pixel(yuv_pixel),
        .out_pixel_valid(yuv_pixel_valid),
        .out_pixel_sof(yuv_pixel_sof),
        .out_pixel_eol(yuv_pixel_eol),
        .out_pixel_eof(yuv_pixel_eof),
        .out_pixel_err(yuv_pixel_err),
        .sts_pixel_per_line(yuv_pixel_per_line)
    );

    axis_video_bridge #(
        .TDATA_WIDTH(24),
        .TUSER_WIDTH(2),
        .FIFO_DEPTH(16),
        .AXIS_TUSER_ERR_DEBUG(1'b1)
    ) u_axis_bridge (
        .core_clk(core_clk),
        .core_aresetn(core_aresetn),
        .aclk(pix_clk),
        .aresetn(pix_aresetn),
        .in_pixel(yuv_pixel),
        .in_pixel_valid(yuv_pixel_valid),
        .in_pixel_sof(yuv_pixel_sof),
        .in_pixel_eol(yuv_pixel_eol),
        .in_pixel_eof(yuv_pixel_eof),
        .in_pixel_err(yuv_pixel_err),
        .m_axis_tdata(axis_tdata),
        .m_axis_tvalid(axis_tvalid),
        .m_axis_tready(axis_tready),
        .m_axis_tlast(axis_tlast),
        .m_axis_tuser(axis_tuser),
        .sts_fifo_overflow_cnt(bridge_overflow_count),
        .sts_back_pressure_cnt(bridge_back_pressure_count)
    );

    hdmi_output #(
        .H_ACTIVE(2),
        .H_FRONT_PORCH(1),
        .H_SYNC(1),
        .H_BACK_PORCH(1),
        .V_ACTIVE(1),
        .V_FRONT_PORCH(1),
        .V_SYNC(1),
        .V_BACK_PORCH(1),
        .HSYNC_POLARITY(1'b0),
        .VSYNC_POLARITY(1'b0)
    ) u_hdmi_output (
        .pix_clk(pix_clk),
        .pix_aresetn(pix_aresetn),
        .enable(hdmi_enable),
        .soft_reset(1'b0),
        .test_pattern_en(1'b0),
        .hpd(1'b1),
        .hpd_override(1'b1),
        .s_axis_tdata(axis_tdata),
        .s_axis_tvalid(axis_tvalid),
        .s_axis_tready(axis_tready),
        .s_axis_tlast(axis_tlast),
        .s_axis_tuser(axis_tuser[0]),
        .video_r(video_r),
        .video_g(video_g),
        .video_b(video_b),
        .video_de(video_de),
        .video_hsync(video_hsync),
        .video_vsync(video_vsync),
        .tmds_data_0(tmds_data_0),
        .tmds_data_1(tmds_data_1),
        .tmds_data_2(tmds_data_2),
        .tmds_clk_word(tmds_clk_word),
        .sts_running(hdmi_running),
        .sts_hpd(hdmi_hpd_seen),
        .sts_frame_count(hdmi_frame_count),
        .sts_underflow_count(hdmi_underflow_count),
        .sts_axis_error_count(hdmi_axis_error_count)
    );

endmodule
`default_nettype wire
