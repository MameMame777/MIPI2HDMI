`timescale 1ns / 1ps

// Full E2E: MIPI bytes → parser → filter → frame_state → yuv_unpack →
//          ob_row_masker → axis_video_bridge → y8→32 → DDR queue → 32→y8 →
//          hdmi_output → check {video_r,video_g,video_b}
//
// Sends a frame whose lines mix:
//   - OB rows (uniform low Y) — masker MUST replace with Y=128
//   - Checkerboard rows (8 dark + 8 bright) — masker MUST pass through
//   - Gradient row — masker MUST pass through
//
// Verifies HDMI output pixel-by-pixel against expected pattern.

module tb_ob_masker_e2e;
    localparam int PARSER_IN_WIDTH = 16;
    localparam logic [5:0] DT_FS = 6'h00;
    localparam logic [5:0] DT_FE = 6'h01;
    localparam logic [5:0] DT_YUV422 = 6'h1e;

    localparam int LINE_PIXELS = 16;
    localparam int FRAME_LINES = 4;
    localparam int LINE_BYTES  = LINE_PIXELS * 2;
    localparam int FRAME_PIXELS = LINE_PIXELS * FRAME_LINES;

    // What MIPI sends (Y component of YUYV)
    logic [7:0] input_y [0:FRAME_PIXELS-1] = '{
        // Line 0: OB row (uniform Y=36)
        8'd36, 8'd36, 8'd36, 8'd36, 8'd36, 8'd36, 8'd36, 8'd36,
        8'd36, 8'd36, 8'd36, 8'd36, 8'd36, 8'd36, 8'd36, 8'd36,
        // Line 1: Checkerboard (8 dark Y=10 + 8 bright Y=240)
        8'd10, 8'd10, 8'd10, 8'd10, 8'd10, 8'd10, 8'd10, 8'd10,
        8'd240, 8'd240, 8'd240, 8'd240, 8'd240, 8'd240, 8'd240, 8'd240,
        // Line 2: Gradient 0→240
        8'd0, 8'd16, 8'd32, 8'd48, 8'd64, 8'd80, 8'd96, 8'd112,
        8'd128, 8'd144, 8'd160, 8'd176, 8'd192, 8'd208, 8'd224, 8'd240,
        // Line 3: OB row with slight variation (uniform Y=38-39, range ≤ 3)
        8'd38, 8'd39, 8'd38, 8'd39, 8'd38, 8'd39, 8'd38, 8'd39,
        8'd38, 8'd39, 8'd38, 8'd39, 8'd38, 8'd39, 8'd38, 8'd39
    };

    // What HDMI should output
    logic [7:0] expected_y [0:FRAME_PIXELS-1] = '{
        // Line 0: OB → masked to Y=128
        8'd128, 8'd128, 8'd128, 8'd128, 8'd128, 8'd128, 8'd128, 8'd128,
        8'd128, 8'd128, 8'd128, 8'd128, 8'd128, 8'd128, 8'd128, 8'd128,
        // Line 1: Checkerboard → passes through unchanged
        8'd10, 8'd10, 8'd10, 8'd10, 8'd10, 8'd10, 8'd10, 8'd10,
        8'd240, 8'd240, 8'd240, 8'd240, 8'd240, 8'd240, 8'd240, 8'd240,
        // Line 2: Gradient → passes through
        8'd0, 8'd16, 8'd32, 8'd48, 8'd64, 8'd80, 8'd96, 8'd112,
        8'd128, 8'd144, 8'd160, 8'd176, 8'd192, 8'd208, 8'd224, 8'd240,
        // Line 3: OB with variation (range=1) → masked to Y=128
        8'd128, 8'd128, 8'd128, 8'd128, 8'd128, 8'd128, 8'd128, 8'd128,
        8'd128, 8'd128, 8'd128, 8'd128, 8'd128, 8'd128, 8'd128, 8'd128
    };

    logic core_clk;
    logic core_aresetn;
    logic aclk;
    logic aresetn;

    logic [PARSER_IN_WIDTH-1:0]   s_byte_data;
    logic [PARSER_IN_WIDTH/8-1:0] s_byte_keep;
    logic                         s_byte_valid;
    logic                         s_byte_sop;
    logic                         s_byte_eop;

    // Parser
    logic parser_ecc_hdr_valid;
    logic [31:0] parser_ecc_hdr_raw;
    logic ecc_hdr_corr_valid;
    logic [23:0] ecc_hdr_corr;
    logic [7:0] ecc_hdr_di;
    logic [15:0] ecc_hdr_wc;
    logic ecc_hdr_corrected, ecc_hdr_uncorrectable, ecc_hdr_no_error;
    logic [15:0] ecc_corr_count, ecc_uncorr_count;
    logic pkt_hdr_valid;
    logic [31:0] pkt_hdr_raw;
    logic [7:0] pkt_di;
    logic [15:0] pkt_wc;
    logic pkt_is_long, pkt_is_short, pkt_ecc_uncorrectable;
    logic [7:0] payload_data;
    logic payload_valid, payload_first, payload_last;
    logic [15:0] footer_data;
    logic footer_valid, pkt_done;
    logic [15:0] parser_short_count, parser_long_count, parser_trunc_count;

    // CRC
    logic crc_check_valid, crc_match;
    logic [15:0] crc_calc, crc_received, crc_err_count, crc_ok_count;

    // Filter
    logic [7:0] filter_pkt_di;
    logic [15:0] filter_pkt_wc;
    logic filter_pkt_is_short, filter_pkt_is_long, filter_pkt_start, filter_pkt_end, filter_pkt_err;
    logic [7:0] filter_payload_data;
    logic filter_payload_valid, filter_payload_first, filter_payload_last;
    logic [15:0] filter_drop_vc_count, filter_drop_dt_count;

    // Frame state
    logic frame_sof, frame_eof, frame_sol, frame_eol;
    logic [15:0] frame_line_idx;
    logic [7:0] frame_payload_data;
    logic frame_payload_valid, frame_payload_first, frame_payload_last, frame_err;
    logic [31:0] frame_count, line_count;
    logic [15:0] last_frame_lines, frame_sync_err_count;

    // YUV unpack
    logic [23:0] yuv_pixel;
    logic yuv_pixel_valid, yuv_pixel_sof, yuv_pixel_eol, yuv_pixel_eof, yuv_pixel_err;
    logic [15:0] yuv_pixel_per_line;

    // OB masker output (8-bit Y)
    logic [7:0] ob_data;
    logic       ob_valid, ob_sof, ob_eol, ob_eof, ob_err;

    // Bridge (24-bit; replicate Y to RGB)
    logic [23:0] bridge_axis_tdata;
    logic        bridge_axis_tvalid, bridge_axis_tready, bridge_axis_tlast;
    logic [1:0]  bridge_axis_tuser;
    logic [15:0] bridge_overflow_count, bridge_back_pressure_count;

    // HDMI
    logic [23:0] hdmi_in_tdata;
    logic        hdmi_in_tvalid, hdmi_in_tready, hdmi_in_tlast;
    logic        hdmi_in_tuser;
    logic hdmi_enable;
    logic [7:0] video_r, video_g, video_b;
    logic video_de, video_hsync, video_vsync;
    logic [9:0] tmds_data_0, tmds_data_1, tmds_data_2, tmds_clk_word;
    logic hdmi_running, hdmi_hpd_seen;
    logic [31:0] hdmi_frame_count;
    logic [15:0] hdmi_underflow_count, hdmi_axis_error_count;

    int unsigned hdmi_seen_count;

    initial begin core_clk = 1'b0; forever #5 core_clk = ~core_clk; end
    initial begin aclk     = 1'b0; forever #7 aclk     = ~aclk; end

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

    // === OB row masker (DUT under E2E test) ===
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

    // Bridge expects 24-bit; replicate Y → R=G=B=Y
    wire [23:0] ob_pixel24 = {ob_data, ob_data, ob_data};

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

    // Model VDMA as a true frame-buffer: accumulate the full bridge output
    // (one frame), then play it back at HDMI rate. This matches real VDMA
    // semantics (S2MM writes a whole frame to DDR, MM2S reads it back at
    // pix_clk rate). The y8↔y32 packer/unpacker round-trip would otherwise
    // inject periodic 1-cycle gaps that cause HDMI underflow at small frame
    // sizes — not representative of real hardware.
    typedef struct packed {
        logic [7:0] data;
        logic       sof;
        logic       last;
    } fb_entry_t;

    fb_entry_t fb_storage [$];
    logic      fb_playback_en;
    int        fb_play_idx;

    // Debug taps: capture yuv_unpack and ob_masker outputs for self-check
    logic [7:0] yuv_capture [$];
    logic [7:0] ob_capture [$];
    always_ff @(posedge core_clk) begin
        if (yuv_pixel_valid) yuv_capture.push_back(yuv_pixel[7:0]);
        if (ob_valid)        ob_capture.push_back(ob_data);
    end

    // Capture bridge output continuously (camera side)
    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            fb_storage.delete();
        end else if (bridge_axis_tvalid && bridge_axis_tready) begin
            fb_storage.push_back('{bridge_axis_tdata[7:0],
                                   bridge_axis_tuser[0],
                                   bridge_axis_tlast});
        end
    end
    assign bridge_axis_tready = 1'b1;  // never backpressure capture side

    // Playback (HDMI side): emit Y8 from buffer at HDMI rate, with SOF/EOL/EOF
    always_comb begin
        if (fb_playback_en && fb_play_idx < fb_storage.size()) begin
            hdmi_in_tdata  = {3{fb_storage[fb_play_idx].data}};
            hdmi_in_tvalid = 1'b1;
            hdmi_in_tlast  = fb_storage[fb_play_idx].last;
            hdmi_in_tuser  = fb_storage[fb_play_idx].sof;
        end else begin
            hdmi_in_tdata  = 24'h0;
            hdmi_in_tvalid = 1'b0;
            hdmi_in_tlast  = 1'b0;
            hdmi_in_tuser  = 1'b0;
        end
    end

    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            fb_play_idx    <= 0;
        end else if (hdmi_in_tvalid && hdmi_in_tready) begin
            fb_play_idx <= fb_play_idx + 1;
        end
    end

    // Large porches give the y8→y32→y8 unpacker time to refill 32-bit beats
    // between active windows so HDMI doesn't underflow at this small frame size.
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

    // -------------------- helpers --------------------

    function automatic [5:0] calc_ecc6(input logic [23:0] data);
        calc_ecc6[0] = data[0]^data[1]^data[2]^data[4]^data[5]^data[7]^data[10]^data[11]^data[13]^data[16]^data[20]^data[21]^data[22]^data[23];
        calc_ecc6[1] = data[0]^data[1]^data[3]^data[4]^data[6]^data[8]^data[10]^data[12]^data[14]^data[17]^data[20]^data[21]^data[22]^data[23];
        calc_ecc6[2] = data[0]^data[2]^data[3]^data[5]^data[6]^data[9]^data[11]^data[12]^data[15]^data[18]^data[20]^data[21]^data[22];
        calc_ecc6[3] = data[1]^data[2]^data[3]^data[7]^data[8]^data[9]^data[13]^data[14]^data[15]^data[19]^data[20]^data[21]^data[23];
        calc_ecc6[4] = data[4]^data[5]^data[6]^data[7]^data[8]^data[9]^data[16]^data[17]^data[18]^data[19]^data[20]^data[22]^data[23];
        calc_ecc6[5] = data[10]^data[11]^data[12]^data[13]^data[14]^data[15]^data[16]^data[17]^data[18]^data[19]^data[21]^data[22]^data[23];
    endfunction

    function automatic [7:0] make_ecc(input logic [7:0] di, input logic [15:0] wc);
        make_ecc = {2'b00, calc_ecc6({wc, di})};
    endfunction

    function automatic [15:0] crc_update_byte(input logic [15:0] crc_in, input logic [7:0] data);
        automatic logic [15:0] crc_next;
        automatic logic feedback;
        crc_next = crc_in;
        for (int bit_idx = 0; bit_idx < 8; bit_idx++) begin
            feedback = crc_next[0] ^ data[bit_idx];
            crc_next = crc_next >> 1;
            if (feedback) crc_next = crc_next ^ 16'h8408;
        end
        crc_update_byte = crc_next;
    endfunction

    task automatic check_condition(input bit condition, input string message);
        if (!condition) $fatal(1, "CHECK FAILED: %s", message);
    endtask

    task automatic reset_dut();
        core_aresetn = 1'b0;
        aresetn = 1'b0;
        s_byte_data = '0;
        s_byte_keep = '0;
        s_byte_valid = 1'b0;
        s_byte_sop = 1'b0;
        s_byte_eop = 1'b0;
        hdmi_enable = 1'b0;
        fb_playback_en = 1'b0;
        repeat (8) @(posedge core_clk);
        core_aresetn = 1'b1;
        repeat (8) @(posedge aclk);
        aresetn = 1'b1;
        repeat (4) @(posedge core_clk);
    endtask

    task automatic drive_idle(input int cycles);
        @(negedge core_clk);
        s_byte_valid <= 1'b0;
        s_byte_keep <= '0;
        s_byte_sop <= 1'b0;
        s_byte_eop <= 1'b0;
        s_byte_data <= '0;
        repeat (cycles) @(negedge core_clk);
    endtask

    task automatic drive_beat(
        input logic [7:0] byte0, input logic [7:0] byte1,
        input logic sop, input logic eop
    );
        @(negedge core_clk);
        s_byte_data <= {byte1, byte0};
        s_byte_keep <= 2'b11;
        s_byte_valid <= 1'b1;
        s_byte_sop <= sop;
        s_byte_eop <= eop;
    endtask

    task automatic send_short_packet(input logic [5:0] dt, input logic [15:0] data_field);
        automatic logic [7:0] di, ecc;
        di = {2'b00, dt};
        ecc = make_ecc(di, data_field);
        drive_beat(di, data_field[7:0], 1'b1, 1'b0);
        drive_beat(data_field[15:8], ecc, 1'b0, 1'b1);
        drive_idle(2);
    endtask

    // Send a YUV422 line of LINE_PIXELS pixels (Y from input_y[]).
    // chroma fixed at 0x80, bytes: 0x80, Y0, 0x80, Y1, ..., 0x80, Y(LINE_PIXELS-1)
    task automatic send_yuv422_line(input int line_idx);
        automatic logic [7:0] di, ecc;
        automatic logic [15:0] wc, crc;
        automatic logic [7:0] payload [0:LINE_BYTES-1];
        automatic int base = line_idx * LINE_PIXELS;

        di = {2'b00, DT_YUV422};
        wc = 16'(LINE_BYTES);
        ecc = make_ecc(di, wc);

        for (int p = 0; p < LINE_PIXELS; p++) begin
            payload[2*p]     = 8'h80;
            payload[2*p + 1] = input_y[base + p];
        end

        crc = 16'hffff;
        for (int i = 0; i < LINE_BYTES; i++) crc = crc_update_byte(crc, payload[i]);

        // header beat (DI, WC[7:0])
        drive_beat(di, wc[7:0], 1'b1, 1'b0);
        // header beat 2 (WC[15:8], ECC)
        drive_beat(wc[15:8], ecc, 1'b0, 1'b0);
        // payload beats: 2 bytes per beat
        for (int i = 0; i < LINE_BYTES; i += 2) begin
            drive_beat(payload[i], payload[i+1], 1'b0, 1'b0);
        end
        // CRC footer beat
        drive_beat(crc[7:0], crc[15:8], 1'b0, 1'b1);
        drive_idle(8);
    endtask

    task automatic wait_frame_done();
        for (int cycle = 0; cycle < 8000; cycle++) begin
            @(posedge core_clk);
            if (frame_count == 32'd1) return;
        end
        $fatal(1, "Timed out waiting for CSI-2 frame completion");
    endtask

    task automatic wait_parser_short_count(input logic [15:0] expected_count);
        for (int cycle = 0; cycle < 4000; cycle++) begin
            @(posedge core_clk);
            if (parser_short_count >= expected_count) return;
        end
        $fatal(1, "Timed out waiting for parser short count %0d, saw %0d",
                  expected_count, parser_short_count);
    endtask

    task automatic check_hdmi_pixels();
        automatic logic [15:0] underflow_before, axis_error_before;
        automatic int errors = 0;

        underflow_before = hdmi_underflow_count;
        axis_error_before = hdmi_axis_error_count;
        hdmi_seen_count = 0;

        $display("[INFO] yuv_capture=%0d  ob_capture=%0d  fb_storage=%0d  (expect %0d)",
                 yuv_capture.size(), ob_capture.size(), fb_storage.size(), FRAME_PIXELS);
        check_condition(yuv_capture.size() == FRAME_PIXELS, "yuv_unpack emitted all pixels");
        check_condition(ob_capture.size()  == FRAME_PIXELS, "ob_masker emitted all pixels");
        check_condition(fb_storage.size()  == FRAME_PIXELS, "framebuffer captured all pixels");

        @(negedge aclk);
        fb_playback_en = 1'b1;
        hdmi_enable = 1'b1;

        for (int cycle = 0; cycle < 20000; cycle++) begin
            @(posedge aclk);
            #1;
            if (video_de) begin
                check_condition(hdmi_seen_count < FRAME_PIXELS,
                    $sformatf("HDMI emitted more active pixels than expected (saw %0d)", hdmi_seen_count));
                if (video_r !== expected_y[hdmi_seen_count]) begin
                    $display("[FAIL] HDMI pixel %0d: got R=0x%02h G=0x%02h B=0x%02h, expected 0x%02h",
                        hdmi_seen_count, video_r, video_g, video_b, expected_y[hdmi_seen_count]);
                    errors++;
                end
                hdmi_seen_count = hdmi_seen_count + 1;
                if (hdmi_seen_count == FRAME_PIXELS) break;
            end
        end

        check_condition(hdmi_seen_count == FRAME_PIXELS,
            $sformatf("HDMI delivered all %0d expected pixels (saw %0d)",
                      FRAME_PIXELS, hdmi_seen_count));

        if (errors == 0) begin
            $display("[PASS] HDMI E2E: all %0d pixels match expected pattern", FRAME_PIXELS);
            $display("       (lines 0 & 3 = OB masked to 0x80; lines 1 & 2 = pass-through)");
        end else begin
            $fatal(1, "HDMI E2E FAILED: %0d pixel mismatches", errors);
        end

        repeat (8) @(posedge aclk);
        check_condition(hdmi_underflow_count == underflow_before,
            "HDMI active window had no underflow");
        check_condition(hdmi_axis_error_count == axis_error_before,
            "HDMI active window had no AXIS sideband error");
    endtask

    initial begin
        reset_dut();

        send_short_packet(DT_FS, 16'h0000);
        for (int line_idx = 0; line_idx < FRAME_LINES; line_idx++)
            send_yuv422_line(line_idx);
        send_short_packet(DT_FE, 16'h0000);

        wait_frame_done();
        wait_parser_short_count(16'd2);

        check_condition(parser_short_count >= 16'd2, "parser saw FS and FE");
        check_condition(parser_long_count == 16'(FRAME_LINES), "one long packet per line");
        check_condition(crc_err_count == 16'd0, "no CRC errors");
        check_condition(parser_trunc_count == 16'd0, "no parser truncation");
        check_condition(last_frame_lines == 16'(FRAME_LINES), "frame state saw all lines");

        // Let pipeline fully drain into HDMI-side buffers before consumer starts.
        // OB masker adds 1-line latency, and aclk (71MHz) > MIPI feed rate (~50MHz Y),
        // so without pre-buffer HDMI underflows.
        repeat (2000) @(posedge aclk);

        check_hdmi_pixels();

        $display("\n==== tb_ob_masker_e2e PASSED ====");
        $finish;
    end

    initial begin
        #2_000_000;
        $fatal(1, "global timeout");
    end

endmodule
