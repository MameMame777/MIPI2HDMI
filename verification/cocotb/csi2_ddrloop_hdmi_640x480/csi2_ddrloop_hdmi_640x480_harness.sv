`timescale 1ns / 1ps
`default_nettype none
// -----------------------------------------------------------------------------
// Auto-generated E2E wrapper for the cocotb port of
// verification/tb/tb_csi2_ddrloop_hdmi_640x480.sv.
//
// Contains ONLY the RTL DUT chain plus the two *behavioural* queue models the
// DSim TB interposed between clock domains (the DDR-loop queue on aclk and the
// TB-only CDC FIFO between aclk and pix_clk). cocotb owns the clocks, reset,
// CSI-2 byte-beat stimulus, and all final checks. Wiring + parameters are 1:1
// with the DSim TB.
//
// The DSim TB's `$fatal` on a non-full DDR TKEEP is turned into the sticky
// output `ddr_tkeep_err` that the cocotb side asserts is 0.
// -----------------------------------------------------------------------------
module csi2_ddrloop_hdmi_640x480_harness #(
    parameter int PARSER_IN_WIDTH = 16,
    parameter int LINE_PIXELS = 640,
    parameter int FRAME_LINES = 480,
    parameter int LINE_BYTES  = LINE_PIXELS * 2,
    parameter int FRAME_BEATS = (LINE_PIXELS / 4) * FRAME_LINES
) (
    input  wire        core_clk,
    input  wire        core_aresetn,
    input  wire        aclk,
    input  wire        aresetn,
    input  wire        pix_clk,
    input  wire        pix_aresetn,

    // CSI-2 byte-beat stimulus (driven by cocotb on core_clk)
    input  wire [PARSER_IN_WIDTH-1:0]   s_byte_data,
    input  wire [PARSER_IN_WIDTH/8-1:0] s_byte_keep,
    input  wire        s_byte_valid,
    input  wire        s_byte_sop,
    input  wire        s_byte_eop,

    input  wire        hdmi_enable,

    // ---- status taps consumed by cocotb checks ----
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
    output wire [15:0] bridge_back_pressure_count,

    // DDR-loop observation
    output logic [31:0] ddr_beats_seen,
    output logic        ddr_tkeep_err,

    // HDMI video outputs (checked by cocotb)
    output wire [7:0]  video_r,
    output wire [7:0]  video_g,
    output wire [7:0]  video_b,
    output wire        video_de,
    output wire        video_hsync,
    output wire        video_vsync,
    output wire        hdmi_running,
    output wire [31:0] hdmi_frame_count,
    output wire [15:0] hdmi_underflow_count,
    output wire [15:0] hdmi_axis_error_count
);
    localparam logic [5:0] DT_YUV422 = 6'h1e;

    // ---- parser ----
    logic parser_ecc_hdr_valid;
    logic [31:0] parser_ecc_hdr_raw;
    logic ecc_hdr_corr_valid;
    logic [23:0] ecc_hdr_corr;
    logic [7:0] ecc_hdr_di;
    logic [15:0] ecc_hdr_wc;
    logic ecc_hdr_corrected;
    logic ecc_hdr_uncorrectable;
    logic ecc_hdr_no_error;

    logic pkt_hdr_valid;
    logic [31:0] pkt_hdr_raw;
    logic [7:0] pkt_di;
    logic [15:0] pkt_wc;
    logic pkt_is_long;
    logic pkt_is_short;
    logic pkt_ecc_uncorrectable;
    logic [7:0] payload_data;
    logic payload_valid;
    logic payload_first;
    logic payload_last;
    logic [15:0] footer_data;
    logic footer_valid;
    logic pkt_done;

    csi2_packet_parser #(
        .IN_WIDTH(PARSER_IN_WIDTH),
        .WC_MAX(16383),
        .FIFO_DEPTH(2048)
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
        .sts_ecc_corr_cnt(),
        .sts_ecc_uncorr_cnt(ecc_uncorr_count)
    );

    logic crc_check_valid;
    logic crc_match;
    logic [15:0] crc_calc;
    logic [15:0] crc_received;

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

    logic [7:0] filter_pkt_di;
    logic [15:0] filter_pkt_wc;
    logic filter_pkt_is_short;
    logic filter_pkt_is_long;
    logic filter_pkt_start;
    logic filter_pkt_end;
    logic filter_pkt_err;
    logic [7:0] filter_payload_data;
    logic filter_payload_valid;
    logic filter_payload_first;
    logic filter_payload_last;

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

    logic frame_sof;
    logic frame_eof;
    logic frame_sol;
    logic frame_eol;
    logic [15:0] frame_line_idx;
    logic [7:0] frame_payload_data;
    logic frame_payload_valid;
    logic frame_payload_first;
    logic frame_payload_last;
    logic frame_err;

    csi2_frame_state #(
        .MAX_LINES(512),
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

    logic [23:0] yuv_pixel;
    logic yuv_pixel_valid;
    logic yuv_pixel_sof;
    logic yuv_pixel_eol;
    logic yuv_pixel_eof;
    logic yuv_pixel_err;

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

    logic [23:0] bridge_axis_tdata;
    logic        bridge_axis_tvalid;
    logic        bridge_axis_tready;
    logic        bridge_axis_tlast;
    logic [1:0]  bridge_axis_tuser;

    axis_video_bridge #(
        .TDATA_WIDTH(24),
        .TUSER_WIDTH(2),
        .FIFO_DEPTH(2048),
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

    // ---- packer input (byte from bridge) ----
    logic [7:0] pack_in_tdata;
    logic       pack_in_tvalid;
    logic       pack_in_tready;
    logic       pack_in_tlast;
    logic [0:0] pack_in_tuser;

    assign pack_in_tdata    = bridge_axis_tdata[7:0];
    assign pack_in_tvalid   = bridge_axis_tvalid;
    assign bridge_axis_tready = pack_in_tready;
    assign pack_in_tlast    = bridge_axis_tlast;
    assign pack_in_tuser[0] = bridge_axis_tuser[0];

    logic [31:0] pack_out_tdata;
    logic [3:0]  pack_out_tkeep;
    logic        pack_out_tvalid;
    logic        pack_out_tready;
    logic        pack_out_tlast;
    logic [0:0]  pack_out_tuser;

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

    // ------------------------------------------------------------------
    // DDR-loop behavioural queue (aclk) -- 1:1 with the DSim TB.
    // ------------------------------------------------------------------
    typedef struct packed {
        logic [31:0] data;
        logic        last;
        logic        user;
    } ddr_beat_t;

    ddr_beat_t ddr_queue [$];
    localparam int DDR_DEPTH = FRAME_BEATS + 256;

    logic [31:0] ddr_out_tdata;
    logic        ddr_out_tvalid;
    logic        ddr_out_tready;
    logic        ddr_out_tlast;
    logic [0:0]  ddr_out_tuser;

    logic [31:0] ddr_head_data;
    logic        ddr_head_last;
    logic        ddr_head_user;
    logic        ddr_head_valid;
    logic        ddr_can_accept;

    assign pack_out_tready  = ddr_can_accept;
    assign ddr_out_tvalid   = ddr_head_valid;
    assign ddr_out_tdata    = ddr_head_data;
    assign ddr_out_tlast    = ddr_head_last;
    assign ddr_out_tuser[0] = ddr_head_user;

    always_comb begin
        ddr_can_accept = (ddr_queue.size() < DDR_DEPTH);
        if (ddr_queue.size() > 0) begin
            ddr_head_valid = 1'b1;
            ddr_head_data  = ddr_queue[0].data;
            ddr_head_last  = ddr_queue[0].last;
            ddr_head_user  = ddr_queue[0].user;
        end else begin
            ddr_head_valid = 1'b0;
            ddr_head_data  = 32'h0;
            ddr_head_last  = 1'b0;
            ddr_head_user  = 1'b0;
        end
    end

    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            ddr_queue.delete();
            ddr_beats_seen <= 0;
            ddr_tkeep_err  <= 1'b0;
        end else begin
            if (pack_out_tvalid && pack_out_tready) begin
                ddr_queue.push_back('{pack_out_tdata, pack_out_tlast, pack_out_tuser[0]});
                if (pack_out_tkeep !== 4'hf) begin
                    ddr_tkeep_err <= 1'b1;   // DSim TB: $fatal on non-full beat
                end
                ddr_beats_seen <= ddr_beats_seen + 1;
            end
            if (ddr_out_tvalid && ddr_out_tready) begin
                ddr_queue.delete(0);
            end
        end
    end

    logic [7:0] unpack_out_tdata;
    logic       unpack_out_tvalid;
    logic       unpack_out_tready;
    logic       unpack_out_tlast;
    logic [0:0] unpack_out_tuser;

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

    // ------------------------------------------------------------------
    // TB-only CDC FIFO between unpacker (aclk) and HDMI (pix_clk) -- 1:1.
    // ------------------------------------------------------------------
    typedef struct packed {
        logic [7:0] data;
        logic       last;
        logic       user;
    } pix_byte_t;

    pix_byte_t cdc_fifo [$];
    localparam int CDC_FIFO_DEPTH = 32;

    logic       cdc_can_accept;
    logic [7:0] cdc_head_data;
    logic       cdc_head_last;
    logic       cdc_head_user;
    logic       cdc_head_valid;

    logic [23:0] hdmi_in_tdata;
    logic        hdmi_in_tvalid;
    logic        hdmi_in_tready;
    logic        hdmi_in_tlast;
    logic        hdmi_in_tuser;

    assign cdc_can_accept    = (cdc_fifo.size() < CDC_FIFO_DEPTH);
    assign unpack_out_tready = cdc_can_accept;

    always_comb begin
        if (cdc_fifo.size() > 0) begin
            cdc_head_valid = 1'b1;
            cdc_head_data  = cdc_fifo[0].data;
            cdc_head_last  = cdc_fifo[0].last;
            cdc_head_user  = cdc_fifo[0].user;
        end else begin
            cdc_head_valid = 1'b0;
            cdc_head_data  = 8'h0;
            cdc_head_last  = 1'b0;
            cdc_head_user  = 1'b0;
        end
    end

    always_ff @(posedge aclk) begin
        if (aresetn && unpack_out_tvalid && unpack_out_tready) begin
            cdc_fifo.push_back('{unpack_out_tdata, unpack_out_tlast, unpack_out_tuser[0]});
        end
    end

    always_ff @(posedge pix_clk) begin
        if (!pix_aresetn) begin
            cdc_fifo.delete();
        end else if (cdc_head_valid && hdmi_in_tready) begin
            cdc_fifo.delete(0);
        end
    end

    assign hdmi_in_tdata  = {3{cdc_head_data}};
    assign hdmi_in_tvalid = cdc_head_valid;
    assign hdmi_in_tlast  = cdc_head_last;
    assign hdmi_in_tuser  = cdc_head_user;

    hdmi_output #(
        .H_ACTIVE(LINE_PIXELS),
        .H_FRONT_PORCH(2),
        .H_SYNC(2),
        .H_BACK_PORCH(2),
        .V_ACTIVE(FRAME_LINES),
        .V_FRONT_PORCH(2),
        .V_SYNC(2),
        .V_BACK_PORCH(2),
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
        .tmds_data_0(),
        .tmds_data_1(),
        .tmds_data_2(),
        .tmds_clk_word(),
        .sts_running(hdmi_running),
        .sts_hpd(),
        .sts_frame_count(hdmi_frame_count),
        .sts_underflow_count(hdmi_underflow_count),
        .sts_axis_error_count(hdmi_axis_error_count)
    );

endmodule
`default_nettype wire
