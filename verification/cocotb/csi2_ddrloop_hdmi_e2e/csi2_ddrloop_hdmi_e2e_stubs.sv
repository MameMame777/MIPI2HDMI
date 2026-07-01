`timescale 1ns / 1ps
`default_nettype none
// -----------------------------------------------------------------------------
// Auto-generated E2E wrapper for the cocotb port of tb_csi2_ddrloop_hdmi_e2e.sv.
//
// Contains ONLY the DUT instances + the DDR-loopback model, expressed as
// SYNTHESIZABLE RTL (a small AXIS FIFO, NOT the DSim SV dynamic queue). No
// `initial`, no clocks, no `$fatal` -- cocotb owns clk/rst + stimulus + checks.
//
// Wiring + parameters are 1:1 with tb_csi2_ddrloop_hdmi_e2e.sv:
//   csi2_packet_parser -> csi2_header_ecc -> csi2_payload_crc -> csi2_vcdt_filter
//   -> csi2_frame_state -> yuv422_gray_unpack -> axis_video_bridge
//   -> axis_y8_to_vdma32 -> [DDR loop FIFO] -> axis_vdma32_to_y8 -> hdmi_output
//
// The DDR loop in the DSim TB is a `ddr_beat_t ddr_queue [$]` (depth 64) that
// buffers packer 32-bit beats and replays them to the unpacker, and $fatal's if
// any packed beat has TKEEP != 0xF. Here it is a synthesizable circular FIFO
// with the SAME depth-64 backpressure and a sticky `ddr_tkeep_err` flag that
// cocotb checks (the $fatal equivalent). `ddr_beats_seen` counts accepted beats.
// -----------------------------------------------------------------------------
module csi2_ddrloop_hdmi_e2e_harness #(
    parameter int PARSER_IN_WIDTH = 16,
    parameter int LINE_PIXELS     = 4,
    parameter int FRAME_LINES     = 2,
    parameter int LINE_BYTES      = 8,
    parameter logic [5:0] DT_YUV422 = 6'h1e
)(
    input  wire core_clk,
    input  wire core_aresetn,
    input  wire aclk,
    input  wire aresetn,

    // byte-beat stimulus into the packet parser
    input  wire [PARSER_IN_WIDTH-1:0]     s_byte_data,
    input  wire [PARSER_IN_WIDTH/8-1:0]   s_byte_keep,
    input  wire                           s_byte_valid,
    input  wire                           s_byte_sop,
    input  wire                           s_byte_eop,

    // hdmi enable
    input  wire hdmi_enable,

    // --- status / observation outputs ---
    output wire [15:0] parser_short_count,
    output wire [15:0] parser_long_count,
    output wire [15:0] parser_trunc_count,
    output wire [15:0] ecc_uncorr_count,
    output wire [15:0] crc_ok_count,
    output wire [15:0] crc_err_count,
    output wire [15:0] filter_drop_vc_count,
    output wire [15:0] filter_drop_dt_count,
    output wire [15:0] frame_sync_err_count,
    output wire [31:0] frame_count,
    output wire [31:0] line_count,
    output wire [15:0] last_frame_lines,
    output wire [15:0] yuv_pixel_per_line,
    output wire [15:0] bridge_overflow_count,

    // DDR loop model observation
    output wire [31:0] ddr_beats_seen,
    output wire        ddr_tkeep_err,

    // HDMI video outputs
    output wire [7:0]  video_r,
    output wire [7:0]  video_g,
    output wire [7:0]  video_b,
    output wire        video_de,
    output wire        video_hsync,
    output wire        video_vsync,
    output wire [15:0] hdmi_underflow_count,
    output wire [15:0] hdmi_axis_error_count
);
    // -------------------------------------------------------------------------
    // csi2_packet_parser
    // -------------------------------------------------------------------------
    wire        parser_ecc_hdr_valid;
    wire [31:0] parser_ecc_hdr_raw;
    wire        ecc_hdr_corr_valid;
    wire [7:0]  ecc_hdr_di;
    wire [15:0] ecc_hdr_wc;
    wire        ecc_hdr_uncorrectable;

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

    csi2_packet_parser #(
        .IN_WIDTH(PARSER_IN_WIDTH),
        .WC_MAX(64),
        .FIFO_DEPTH(64)
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

    // -------------------------------------------------------------------------
    // csi2_header_ecc
    // -------------------------------------------------------------------------
    wire [23:0] ecc_hdr_corr;
    wire        ecc_hdr_corrected;
    wire        ecc_hdr_no_error;
    wire [15:0] ecc_corr_count;

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

    // -------------------------------------------------------------------------
    // csi2_payload_crc
    // -------------------------------------------------------------------------
    wire        crc_check_valid;
    wire        crc_match;
    wire [15:0] crc_calc;
    wire [15:0] crc_received;

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

    // -------------------------------------------------------------------------
    // csi2_vcdt_filter
    // -------------------------------------------------------------------------
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

    // -------------------------------------------------------------------------
    // csi2_frame_state
    // -------------------------------------------------------------------------
    wire        frame_sof;
    wire        frame_eof;
    wire        frame_sol;
    wire        frame_eol;
    wire        frame_in_frame;
    wire [15:0] frame_line_idx;
    wire [7:0]  frame_payload_data;
    wire        frame_payload_valid;
    wire        frame_payload_first;
    wire        frame_payload_last;
    wire        frame_err;
    wire [15:0] fs_dbg_la, fs_dbg_nols, fs_dbg_idle;
    wire [127:0] fs_dbg_hist;

    csi2_frame_state #(
        .MAX_LINES(8),
        .GUARD_FRAME_LINES(1'b1),
        .EXPECTED_FRAME_LINES(FRAME_LINES),
        .EXPECTED_LINE_WC(16'(LINE_BYTES))
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
        .out_in_frame(frame_in_frame),
        .out_line_idx(frame_line_idx),
        .out_payload_data(frame_payload_data),
        .out_payload_valid(frame_payload_valid),
        .out_payload_first(frame_payload_first),
        .out_payload_last(frame_payload_last),
        .out_frame_err(frame_err),
        .sts_frame_count(frame_count),
        .sts_line_count(line_count),
        .sts_last_frame_lines(last_frame_lines),
        .sts_frame_sync_err_cnt(frame_sync_err_count),
        .sts_dbg_long_accept(fs_dbg_la),
        .sts_dbg_long_nols(fs_dbg_nols),
        .sts_dbg_long_idle(fs_dbg_idle),
        .sts_dbg_nols_hist(fs_dbg_hist)
    );

    // -------------------------------------------------------------------------
    // yuv422_gray_unpack
    // -------------------------------------------------------------------------
    wire [23:0] yuv_pixel;
    wire        yuv_pixel_valid;
    wire        yuv_pixel_sof;
    wire        yuv_pixel_eol;
    wire        yuv_pixel_eof;
    wire        yuv_pixel_err;

    yuv422_gray_unpack #(
        .LINE_PIXELS(LINE_PIXELS)
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

    // -------------------------------------------------------------------------
    // axis_video_bridge (core_clk -> aclk CDC, 24-bit)
    // -------------------------------------------------------------------------
    wire [23:0] bridge_axis_tdata;
    wire        bridge_axis_tvalid;
    wire        bridge_axis_tready;
    wire        bridge_axis_tlast;
    wire [1:0]  bridge_axis_tuser;
    wire [15:0] bridge_back_pressure_count;

    axis_video_bridge #(
        .TDATA_WIDTH(24),
        .TUSER_WIDTH(2),
        .FIFO_DEPTH(32),
        .AXIS_TUSER_ERR_DEBUG(1'b1)
    ) u_axis_bridge (
        .core_clk(core_clk),
        .core_aresetn(core_aresetn),
        .aclk(aclk),
        .aresetn(aresetn),
        .in_pixel(yuv_pixel),
        .in_pixel_valid(yuv_pixel_valid),
        .in_pixel_sof(yuv_pixel_sof),
        .in_pixel_eol(yuv_pixel_eol),
        .in_pixel_eof(yuv_pixel_eof),
        .in_pixel_err(yuv_pixel_err),
        .m_axis_tdata(bridge_axis_tdata),
        .m_axis_tvalid(bridge_axis_tvalid),
        .m_axis_tready(bridge_axis_tready),
        .m_axis_tlast(bridge_axis_tlast),
        .m_axis_tuser(bridge_axis_tuser),
        .sts_fifo_overflow_cnt(bridge_overflow_count),
        .sts_back_pressure_cnt(bridge_back_pressure_count)
    );

    // -------------------------------------------------------------------------
    // packer: axis_y8_to_vdma32 (Y8 -> 32-bit VDMA beats)
    // -------------------------------------------------------------------------
    wire [7:0] pack_in_tdata  = bridge_axis_tdata[7:0];
    wire       pack_in_tvalid = bridge_axis_tvalid;
    wire       pack_in_tready;
    wire       pack_in_tlast  = bridge_axis_tlast;
    wire [0:0] pack_in_tuser  = bridge_axis_tuser[0];

    assign bridge_axis_tready = pack_in_tready;

    wire [31:0] pack_out_tdata;
    wire [3:0]  pack_out_tkeep;
    wire        pack_out_tvalid;
    wire        pack_out_tready;
    wire        pack_out_tlast;
    wire [0:0]  pack_out_tuser;

    axis_y8_to_vdma32 u_packer (
        .aclk(aclk),
        .aresetn(aresetn),
        .s_axis_tdata(pack_in_tdata),
        .s_axis_tvalid(pack_in_tvalid),
        .s_axis_tready(pack_in_tready),
        .s_axis_tlast(pack_in_tlast),
        .s_axis_tuser(pack_in_tuser),
        .m_axis_tdata(pack_out_tdata),
        .m_axis_tkeep(pack_out_tkeep),
        .m_axis_tvalid(pack_out_tvalid),
        .m_axis_tready(pack_out_tready),
        .m_axis_tlast(pack_out_tlast),
        .m_axis_tuser(pack_out_tuser)
    );

    // -------------------------------------------------------------------------
    // DDR loop: synthesizable AXIS FIFO (depth 64), replaces the SV ddr_queue.
    //   - accepts packer 32-bit beats when not full (ddr_can_accept)
    //   - replays them in order to the unpacker
    //   - counts accepted beats (ddr_beats_seen)
    //   - raises a sticky ddr_tkeep_err if any accepted beat has TKEEP != 0xF
    //     (the DSim $fatal(1, "DDR loop expects full 32-bit beats ...") equivalent)
    // -------------------------------------------------------------------------
    localparam int DDR_DEPTH = 64;
    localparam int DDR_AW    = 6;  // $clog2(64)

    // Each entry carries {tuser[0], tlast, tdata[31:0]} = 34 bits.
    logic [33:0] ddr_mem [0:DDR_DEPTH-1];
    logic [DDR_AW:0] ddr_wr_ptr;
    logic [DDR_AW:0] ddr_rd_ptr;

    wire [DDR_AW-1:0] ddr_wr_addr = ddr_wr_ptr[DDR_AW-1:0];
    wire [DDR_AW-1:0] ddr_rd_addr = ddr_rd_ptr[DDR_AW-1:0];
    wire ddr_empty = (ddr_wr_ptr == ddr_rd_ptr);
    wire [DDR_AW:0] ddr_count = ddr_wr_ptr - ddr_rd_ptr;
    wire ddr_can_accept = (ddr_count < DDR_DEPTH[DDR_AW:0]);

    wire [31:0] ddr_out_tdata;
    wire        ddr_out_tvalid;
    wire        ddr_out_tready;
    wire        ddr_out_tlast;
    wire [0:0]  ddr_out_tuser;

    assign pack_out_tready = ddr_can_accept;

    assign ddr_out_tvalid   = !ddr_empty;
    assign ddr_out_tdata    = ddr_mem[ddr_rd_addr][31:0];
    assign ddr_out_tlast    = ddr_mem[ddr_rd_addr][32];
    assign ddr_out_tuser[0] = ddr_mem[ddr_rd_addr][33];

    logic [31:0] ddr_beats_seen_r;
    logic        ddr_tkeep_err_r;

    wire ddr_wr_fire = pack_out_tvalid && pack_out_tready;
    wire ddr_rd_fire = ddr_out_tvalid && ddr_out_tready;

    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            ddr_wr_ptr       <= '0;
            ddr_rd_ptr       <= '0;
            ddr_beats_seen_r <= 32'd0;
            ddr_tkeep_err_r  <= 1'b0;
        end else begin
            if (ddr_wr_fire) begin
                ddr_mem[ddr_wr_addr] <= {pack_out_tuser[0], pack_out_tlast, pack_out_tdata};
                ddr_wr_ptr           <= ddr_wr_ptr + 1'b1;
                ddr_beats_seen_r     <= ddr_beats_seen_r + 32'd1;
                if (pack_out_tkeep !== 4'hf) begin
                    ddr_tkeep_err_r <= 1'b1;
                end
            end
            if (ddr_rd_fire) begin
                ddr_rd_ptr <= ddr_rd_ptr + 1'b1;
            end
        end
    end

    assign ddr_beats_seen = ddr_beats_seen_r;
    assign ddr_tkeep_err  = ddr_tkeep_err_r;

    // -------------------------------------------------------------------------
    // unpacker: axis_vdma32_to_y8 (32-bit VDMA beats -> Y8)
    // -------------------------------------------------------------------------
    wire [7:0] unpack_out_tdata;
    wire       unpack_out_tvalid;
    wire       unpack_out_tready;
    wire       unpack_out_tlast;
    wire [0:0] unpack_out_tuser;

    axis_vdma32_to_y8 u_unpacker (
        .aclk(aclk),
        .aresetn(aresetn),
        .s_axis_tdata(ddr_out_tdata),
        .s_axis_tvalid(ddr_out_tvalid),
        .s_axis_tready(ddr_out_tready),
        .s_axis_tlast(ddr_out_tlast),
        .s_axis_tuser(ddr_out_tuser),
        .m_axis_tdata(unpack_out_tdata),
        .m_axis_tvalid(unpack_out_tvalid),
        .m_axis_tready(unpack_out_tready),
        .m_axis_tlast(unpack_out_tlast),
        .m_axis_tuser(unpack_out_tuser)
    );

    // -------------------------------------------------------------------------
    // hdmi_output (gray Y replicated onto R=G=B)
    // -------------------------------------------------------------------------
    wire [23:0] hdmi_in_tdata  = {3{unpack_out_tdata}};
    wire        hdmi_in_tvalid = unpack_out_tvalid;
    wire        hdmi_in_tready;
    wire        hdmi_in_tlast  = unpack_out_tlast;
    wire        hdmi_in_tuser  = unpack_out_tuser[0];

    assign unpack_out_tready = hdmi_in_tready;

    wire [9:0] tmds_data_0, tmds_data_1, tmds_data_2, tmds_clk_word;
    wire       hdmi_running, hdmi_hpd_seen;
    wire [31:0] hdmi_frame_count;

    hdmi_output #(
        .H_ACTIVE(LINE_PIXELS),
        .H_FRONT_PORCH(1),
        .H_SYNC(1),
        .H_BACK_PORCH(1),
        .V_ACTIVE(FRAME_LINES),
        .V_FRONT_PORCH(1),
        .V_SYNC(1),
        .V_BACK_PORCH(1),
        .HSYNC_POLARITY(1'b0),
        .VSYNC_POLARITY(1'b0)
    ) u_hdmi_output (
        .pix_clk(aclk),
        .pix_aresetn(aresetn),
        .enable(hdmi_enable),
        .soft_reset(1'b0),
        .test_pattern_en(1'b0),
        .hpd(1'b1),
        .hpd_override(1'b1),
        .s_axis_tdata(hdmi_in_tdata),
        .s_axis_tvalid(hdmi_in_tvalid),
        .s_axis_tready(hdmi_in_tready),
        .s_axis_tlast(hdmi_in_tlast),
        .s_axis_tuser(hdmi_in_tuser),
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
