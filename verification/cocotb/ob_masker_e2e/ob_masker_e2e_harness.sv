`timescale 1ns / 1ps
`default_nettype none
// Auto-generated E2E wrapper for the cocotb port of tb_ob_masker_e2e.sv.
//
// Contains ONLY the real RTL DUT instances (no initial / no clock / no queue /
// no framebuffer model) so cocotb owns clk/rst, MIPI stimulus, the behavioural
// VDMA framebuffer (capture bridge AXIS -> replay into hdmi_output AXIS), and
// the HDMI pixel check. Wiring + parameters are 1:1 with the DSim TB.
//
// Chain:
//   s_byte_* -> csi2_packet_parser -> csi2_header_ecc / csi2_payload_crc
//            -> csi2_vcdt_filter -> csi2_frame_state -> yuv422_gray_unpack
//            -> ob_row_masker (DUT) -> {Y,Y,Y} -> axis_video_bridge
//            == (bridge AXIS out captured by cocotb) ==
//   (cocotb replays framebuffer) -> hdmi_output -> video_r/g/b
module ob_masker_e2e_harness #(
    parameter int PARSER_IN_WIDTH = 16,
    parameter int LINE_PIXELS = 16,
    parameter int FRAME_LINES = 4,
    parameter int LINE_BYTES  = 32
)(
    input  wire        core_clk,
    input  wire        core_aresetn,
    input  wire        aclk,
    input  wire        aresetn,

    // ---- MIPI byte-beat input (parser) ----
    input  wire [PARSER_IN_WIDTH-1:0]   s_byte_data,
    input  wire [PARSER_IN_WIDTH/8-1:0] s_byte_keep,
    input  wire                         s_byte_valid,
    input  wire                         s_byte_sop,
    input  wire                         s_byte_eop,

    // ---- debug taps (core_clk) for yuv_capture / ob_capture ----
    output wire [7:0]  yuv_pixel_lo,
    output wire        yuv_pixel_valid,
    output wire [7:0]  ob_data,
    output wire        ob_valid,

    // ---- bridge AXIS output (aclk) -> cocotb framebuffer ----
    output wire [23:0] bridge_axis_tdata,
    output wire        bridge_axis_tvalid,
    input  wire        bridge_axis_tready,
    output wire        bridge_axis_tlast,
    output wire [1:0]  bridge_axis_tuser,

    // ---- HDMI AXIS input (aclk), driven by cocotb framebuffer playback ----
    input  wire [23:0] hdmi_in_tdata,
    input  wire        hdmi_in_tvalid,
    output wire        hdmi_in_tready,
    input  wire        hdmi_in_tlast,
    input  wire        hdmi_in_tuser,
    input  wire        hdmi_enable,

    // ---- HDMI video outputs (aclk) ----
    output wire [7:0]  video_r,
    output wire [7:0]  video_g,
    output wire [7:0]  video_b,
    output wire        video_de,
    output wire        video_hsync,
    output wire        video_vsync,

    // ---- status counters used by the TB checks ----
    output wire [15:0] parser_short_count,
    output wire [15:0] parser_long_count,
    output wire [15:0] parser_trunc_count,
    output wire [15:0] crc_err_count,
    output wire [15:0] last_frame_lines,
    output wire [31:0] frame_count,
    output wire [15:0] hdmi_underflow_count,
    output wire [15:0] hdmi_axis_error_count
);
    localparam logic [5:0] DT_YUV422 = 6'h1e;

    // ---- Parser wiring ----
    wire        parser_ecc_hdr_valid;
    wire [31:0] parser_ecc_hdr_raw;
    wire        ecc_hdr_corr_valid;
    wire [23:0] ecc_hdr_corr;
    wire [7:0]  ecc_hdr_di;
    wire [15:0] ecc_hdr_wc;
    wire        ecc_hdr_corrected, ecc_hdr_uncorrectable, ecc_hdr_no_error;
    wire [15:0] ecc_corr_count, ecc_uncorr_count;
    wire        pkt_hdr_valid;
    wire [31:0] pkt_hdr_raw;
    wire [7:0]  pkt_di;
    wire [15:0] pkt_wc;
    wire        pkt_is_long, pkt_is_short, pkt_ecc_uncorrectable;
    wire [7:0]  payload_data;
    wire        payload_valid, payload_first, payload_last;
    wire [15:0] footer_data;
    wire        footer_valid, pkt_done;

    csi2_packet_parser #(
        .IN_WIDTH(PARSER_IN_WIDTH), .WC_MAX(256), .FIFO_DEPTH(256)
    ) u_parser (
        .core_clk(core_clk), .core_aresetn(core_aresetn),
        .s_byte_data(s_byte_data), .s_byte_keep(s_byte_keep), .s_byte_valid(s_byte_valid),
        .s_byte_sop(s_byte_sop), .s_byte_eop(s_byte_eop),
        .ecc_hdr_valid(parser_ecc_hdr_valid), .ecc_hdr_raw(parser_ecc_hdr_raw),
        .ecc_hdr_corr_valid(ecc_hdr_corr_valid), .ecc_hdr_di(ecc_hdr_di), .ecc_hdr_wc(ecc_hdr_wc),
        .ecc_hdr_uncorrectable(ecc_hdr_uncorrectable),
        .m_pkt_hdr_valid(pkt_hdr_valid), .m_pkt_hdr_raw(pkt_hdr_raw), .m_pkt_di(pkt_di), .m_pkt_wc(pkt_wc),
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

    wire        crc_check_valid, crc_match;
    wire [15:0] crc_calc, crc_received, crc_ok_count;

    csi2_payload_crc u_payload_crc (
        .core_clk(core_clk), .core_aresetn(core_aresetn),
        .payload_data(payload_data), .payload_valid(payload_valid),
        .payload_first(payload_first), .payload_last(payload_last),
        .footer_data(footer_data), .footer_valid(footer_valid),
        .crc_check_valid(crc_check_valid), .crc_match(crc_match),
        .crc_calc(crc_calc), .crc_received(crc_received),
        .sts_crc_err_cnt(crc_err_count), .sts_crc_ok_cnt(crc_ok_count)
    );

    // ---- Filter wiring ----
    wire [7:0]  filter_pkt_di;
    wire [15:0] filter_pkt_wc;
    wire        filter_pkt_is_short, filter_pkt_is_long, filter_pkt_start, filter_pkt_end, filter_pkt_err;
    wire [7:0]  filter_payload_data;
    wire        filter_payload_valid, filter_payload_first, filter_payload_last;
    wire [15:0] filter_drop_vc_count, filter_drop_dt_count;

    csi2_vcdt_filter u_filter (
        .core_clk(core_clk), .core_aresetn(core_aresetn),
        .cfg_expected_vc(2'b00), .cfg_expected_dt(DT_YUV422),
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

    // ---- Frame state wiring ----
    wire        frame_sof, frame_eof, frame_sol, frame_eol, frame_in_frame;
    wire [15:0] frame_line_idx;
    wire [7:0]  frame_payload_data;
    wire        frame_payload_valid, frame_payload_first, frame_payload_last, frame_err;
    wire [31:0] frame_line_count;
    wire [15:0] frame_sync_err_count;
    wire [15:0] fs_dbg_la, fs_dbg_nols, fs_dbg_idle;
    wire [127:0] fs_dbg_hist;

    csi2_frame_state #(
        .MAX_LINES(16), .GUARD_FRAME_LINES(1'b1),
        .EXPECTED_FRAME_LINES(FRAME_LINES), .EXPECTED_LINE_WC(16'(LINE_BYTES))
    ) u_frame_state (
        .core_clk(core_clk), .core_aresetn(core_aresetn),
        .cfg_use_lsle(1'b0), .cfg_expected_frame_lines(16'd0),
        .cfg_sof_synth(1'b0), .cfg_force_expected(1'b0), .cfg_long_as_line(1'b0),
        .in_pkt_di(filter_pkt_di), .in_pkt_wc(filter_pkt_wc),
        .in_pkt_is_short(filter_pkt_is_short), .in_pkt_is_long(filter_pkt_is_long),
        .in_pkt_start(filter_pkt_start), .in_pkt_end(filter_pkt_end), .in_pkt_err(filter_pkt_err),
        .in_payload_data(filter_payload_data), .in_payload_valid(filter_payload_valid),
        .in_payload_first(filter_payload_first), .in_payload_last(filter_payload_last),
        .out_sof(frame_sof), .out_eof(frame_eof), .out_sol(frame_sol), .out_eol(frame_eol),
        .out_in_frame(frame_in_frame),
        .out_line_idx(frame_line_idx),
        .out_payload_data(frame_payload_data), .out_payload_valid(frame_payload_valid),
        .out_payload_first(frame_payload_first), .out_payload_last(frame_payload_last),
        .out_frame_err(frame_err),
        .sts_frame_count(frame_count), .sts_line_count(frame_line_count),
        .sts_last_frame_lines(last_frame_lines), .sts_frame_sync_err_cnt(frame_sync_err_count),
        .sts_dbg_long_accept(fs_dbg_la), .sts_dbg_long_nols(fs_dbg_nols),
        .sts_dbg_long_idle(fs_dbg_idle), .sts_dbg_nols_hist(fs_dbg_hist)
    );

    // ---- YUV unpack ----
    wire [23:0] yuv_pixel;
    wire        yuv_pixel_sof, yuv_pixel_eol, yuv_pixel_eof, yuv_pixel_err;
    wire [15:0] yuv_pixel_per_line;

    yuv422_gray_unpack #(.LINE_PIXELS(LINE_PIXELS)) u_yuv_unpack (
        .core_clk(core_clk), .core_aresetn(core_aresetn),
        .in_sof(frame_sof), .in_eof(frame_eof), .in_eol(frame_eol),
        .in_payload_data(frame_payload_data), .in_payload_valid(frame_payload_valid),
        .in_payload_first(frame_payload_first), .in_payload_last(frame_payload_last),
        .in_frame_err(frame_err),
        .out_pixel(yuv_pixel), .out_pixel_valid(yuv_pixel_valid),
        .out_pixel_sof(yuv_pixel_sof), .out_pixel_eol(yuv_pixel_eol),
        .out_pixel_eof(yuv_pixel_eof), .out_pixel_err(yuv_pixel_err),
        .sts_pixel_per_line(yuv_pixel_per_line)
    );
    assign yuv_pixel_lo = yuv_pixel[7:0];

    // ---- OB row masker (DUT) ----
    wire ob_sof, ob_eol, ob_eof, ob_err;
    ob_row_masker #(
        .LINE_PIXELS_MAX(64),
        .OB_THRESHOLD(8'd50), .OB_FILL_Y(8'd128), .OB_UNIFORMITY(8'd3)
    ) u_ob_masker (
        .clk(core_clk), .aresetn(core_aresetn), .enable(1'b1),
        .in_data(yuv_pixel[7:0]), .in_valid(yuv_pixel_valid),
        .in_sof(yuv_pixel_sof), .in_eol(yuv_pixel_eol),
        .in_eof(yuv_pixel_eof), .in_err(yuv_pixel_err),
        .out_data(ob_data), .out_valid(ob_valid),
        .out_sof(ob_sof), .out_eol(ob_eol),
        .out_eof(ob_eof), .out_err(ob_err)
    );

    // Bridge expects 24-bit; replicate Y -> R=G=B=Y
    wire [23:0] ob_pixel24 = {ob_data, ob_data, ob_data};
    wire [15:0] bridge_overflow_count, bridge_back_pressure_count;

    axis_video_bridge #(
        .TDATA_WIDTH(24), .TUSER_WIDTH(2), .FIFO_DEPTH(128), .AXIS_TUSER_ERR_DEBUG(1'b1)
    ) u_axis_bridge (
        .core_clk(core_clk), .core_aresetn(core_aresetn),
        .aclk(aclk), .aresetn(aresetn),
        .in_pixel(ob_pixel24), .in_pixel_valid(ob_valid),
        .in_pixel_sof(ob_sof), .in_pixel_eol(ob_eol),
        .in_pixel_eof(ob_eof), .in_pixel_err(ob_err),
        .m_axis_tdata(bridge_axis_tdata), .m_axis_tvalid(bridge_axis_tvalid),
        .m_axis_tready(bridge_axis_tready), .m_axis_tlast(bridge_axis_tlast),
        .m_axis_tuser(bridge_axis_tuser),
        .sts_fifo_overflow_cnt(bridge_overflow_count),
        .sts_back_pressure_cnt(bridge_back_pressure_count)
    );

    // ---- HDMI output (fed by cocotb framebuffer playback) ----
    wire [9:0] tmds_data_0, tmds_data_1, tmds_data_2, tmds_clk_word;
    wire       hdmi_running, hdmi_hpd_seen;
    wire [31:0] hdmi_frame_count;

    hdmi_output #(
        .H_ACTIVE(LINE_PIXELS), .H_FRONT_PORCH(8), .H_SYNC(4), .H_BACK_PORCH(8),
        .V_ACTIVE(FRAME_LINES), .V_FRONT_PORCH(2), .V_SYNC(2), .V_BACK_PORCH(2),
        .HSYNC_POLARITY(1'b0), .VSYNC_POLARITY(1'b0)
    ) u_hdmi_output (
        .pix_clk(aclk), .pix_aresetn(aresetn),
        .enable(hdmi_enable), .soft_reset(1'b0), .test_pattern_en(1'b0),
        .hpd(1'b1), .hpd_override(1'b1),
        .s_axis_tdata(hdmi_in_tdata), .s_axis_tvalid(hdmi_in_tvalid),
        .s_axis_tready(hdmi_in_tready), .s_axis_tlast(hdmi_in_tlast),
        .s_axis_tuser(hdmi_in_tuser),
        .video_r(video_r), .video_g(video_g), .video_b(video_b),
        .video_de(video_de), .video_hsync(video_hsync), .video_vsync(video_vsync),
        .tmds_data_0(tmds_data_0), .tmds_data_1(tmds_data_1),
        .tmds_data_2(tmds_data_2), .tmds_clk_word(tmds_clk_word),
        .sts_running(hdmi_running), .sts_hpd(hdmi_hpd_seen),
        .sts_frame_count(hdmi_frame_count),
        .sts_underflow_count(hdmi_underflow_count),
        .sts_axis_error_count(hdmi_axis_error_count)
    );
endmodule
`default_nettype wire
