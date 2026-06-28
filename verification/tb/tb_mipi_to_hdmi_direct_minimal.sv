`timescale 1ns / 1ps

module tb_mipi_to_hdmi_direct_minimal;
    localparam int PARSER_IN_WIDTH = 16;
    localparam logic [5:0] DT_FS = 6'h00;
    localparam logic [5:0] DT_FE = 6'h01;
    localparam logic [5:0] DT_YUV422 = 6'h1e;
    localparam logic [7:0] EXPECTED_Y0 = 8'h24;
    localparam logic [7:0] EXPECTED_Y1 = 8'ha8;

    logic core_clk;
    logic core_aresetn;
    logic pix_clk;
    logic pix_aresetn;

    logic [PARSER_IN_WIDTH-1:0] s_byte_data;
    logic [PARSER_IN_WIDTH/8-1:0] s_byte_keep;
    logic s_byte_valid;
    logic s_byte_sop;
    logic s_byte_eop;

    logic parser_ecc_hdr_valid;
    logic [31:0] parser_ecc_hdr_raw;
    logic ecc_hdr_corr_valid;
    logic [23:0] ecc_hdr_corr;
    logic [7:0] ecc_hdr_di;
    logic [15:0] ecc_hdr_wc;
    logic ecc_hdr_corrected;
    logic ecc_hdr_uncorrectable;
    logic ecc_hdr_no_error;
    logic [15:0] ecc_corr_count;
    logic [15:0] ecc_uncorr_count;

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
    logic [15:0] parser_short_count;
    logic [15:0] parser_long_count;
    logic [15:0] parser_trunc_count;

    logic crc_check_valid;
    logic crc_match;
    logic [15:0] crc_calc;
    logic [15:0] crc_received;
    logic [15:0] crc_err_count;
    logic [15:0] crc_ok_count;

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
    logic [15:0] filter_drop_vc_count;
    logic [15:0] filter_drop_dt_count;

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
    logic [31:0] frame_count;
    logic [31:0] line_count;
    logic [15:0] last_frame_lines;
    logic [15:0] frame_sync_err_count;

    logic [23:0] yuv_pixel;
    logic yuv_pixel_valid;
    logic yuv_pixel_sof;
    logic yuv_pixel_eol;
    logic yuv_pixel_eof;
    logic yuv_pixel_err;
    logic [15:0] yuv_pixel_per_line;

    logic [23:0] axis_tdata;
    logic axis_tvalid;
    logic axis_tready;
    logic axis_tlast;
    logic [1:0] axis_tuser;
    logic [15:0] bridge_overflow_count;
    logic [15:0] bridge_back_pressure_count;

    logic hdmi_enable;
    logic [7:0] video_r;
    logic [7:0] video_g;
    logic [7:0] video_b;
    logic video_de;
    logic video_hsync;
    logic video_vsync;
    logic [9:0] tmds_data_0;
    logic [9:0] tmds_data_1;
    logic [9:0] tmds_data_2;
    logic [9:0] tmds_clk_word;
    logic hdmi_running;
    logic hdmi_hpd_seen;
    logic [31:0] hdmi_frame_count;
    logic [15:0] hdmi_underflow_count;
    logic [15:0] hdmi_axis_error_count;

    int yuv_seen_count;
    int axis_seen_count;

    initial begin
        core_clk = 1'b0;
        forever #5 core_clk = ~core_clk;
    end

    initial begin
        pix_clk = 1'b0;
        forever #7 pix_clk = ~pix_clk;
    end

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
            if (feedback) begin
                crc_next = crc_next ^ 16'h8408;
            end
        end
        crc_update_byte = crc_next;
    endfunction

    function automatic [15:0] crc4(
        input logic [7:0] payload0,
        input logic [7:0] payload1,
        input logic [7:0] payload2,
        input logic [7:0] payload3
    );
        automatic logic [15:0] crc_value;
        crc_value = 16'hffff;
        crc_value = crc_update_byte(crc_value, payload0);
        crc_value = crc_update_byte(crc_value, payload1);
        crc_value = crc_update_byte(crc_value, payload2);
        crc_value = crc_update_byte(crc_value, payload3);
        crc4 = crc_value;
    endfunction

    task automatic check_condition(input bit condition, input string message);
        if (!condition) begin
            $fatal(1, "CHECK FAILED: %s", message);
        end
    endtask

    task automatic reset_dut();
        core_aresetn = 1'b0;
        pix_aresetn = 1'b0;
        s_byte_data = '0;
        s_byte_keep = '0;
        s_byte_valid = 1'b0;
        s_byte_sop = 1'b0;
        s_byte_eop = 1'b0;
        hdmi_enable = 1'b0;
        repeat (8) @(posedge core_clk);
        core_aresetn = 1'b1;
        repeat (8) @(posedge pix_clk);
        pix_aresetn = 1'b1;
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
        input logic [7:0] byte0,
        input logic [7:0] byte1,
        input logic sop,
        input logic eop
    );
        @(negedge core_clk);
        s_byte_data <= {byte1, byte0};
        s_byte_keep <= 2'b11;
        s_byte_valid <= 1'b1;
        s_byte_sop <= sop;
        s_byte_eop <= eop;
    endtask

    task automatic send_short_packet(input logic [5:0] dt, input logic [15:0] data_field);
        automatic logic [7:0] di;
        automatic logic [7:0] ecc;
        di = {2'b00, dt};
        ecc = make_ecc(di, data_field);
        drive_beat(di, data_field[7:0], 1'b1, 1'b0);
        drive_beat(data_field[15:8], ecc, 1'b0, 1'b1);
        drive_idle(2);
    endtask

    task automatic send_yuv422_long_packet(
        input logic [7:0] u0,
        input logic [7:0] y0,
        input logic [7:0] v0,
        input logic [7:0] y1
    );
        automatic logic [7:0] di;
        automatic logic [15:0] wc;
        automatic logic [7:0] ecc;
        automatic logic [15:0] crc;
        di = {2'b00, DT_YUV422};
        wc = 16'd4;
        ecc = make_ecc(di, wc);
        crc = crc4(u0, y0, v0, y1);
        drive_beat(di, wc[7:0], 1'b1, 1'b0);
        drive_beat(wc[15:8], ecc, 1'b0, 1'b0);
        drive_beat(u0, y0, 1'b0, 1'b0);
        drive_beat(v0, y1, 1'b0, 1'b0);
        drive_beat(crc[7:0], crc[15:8], 1'b0, 1'b1);
        drive_idle(2);
    endtask

    task automatic wait_core_count(input int expected_yuv_count);
        for (int cycle = 0; cycle < 300; cycle++) begin
            @(posedge core_clk);
            if (yuv_seen_count == expected_yuv_count) begin
                return;
            end
        end
        $fatal(1, "Timed out waiting for %0d YUV pixels, saw %0d", expected_yuv_count, yuv_seen_count);
    endtask

    task automatic wait_frame_done();
        for (int cycle = 0; cycle < 300; cycle++) begin
            @(posedge core_clk);
            if (frame_count == 32'd1) begin
                return;
            end
        end
        $fatal(1, "Timed out waiting for CSI-2 frame completion");
    endtask

    task automatic wait_axis_ready();
        for (int cycle = 0; cycle < 300; cycle++) begin
            @(posedge pix_clk);
            #1;
            if (axis_tvalid && axis_tuser[0]) begin
                return;
            end
        end
        $fatal(1, "Timed out waiting for direct AXIS SOF pixel");
    endtask

    task automatic check_hdmi_pixels();
        automatic logic [7:0] expected_y [0:1];
        automatic logic [15:0] underflow_before;
        automatic logic [15:0] axis_error_before;
        automatic int seen;

        expected_y[0] = EXPECTED_Y0;
        expected_y[1] = EXPECTED_Y1;
        underflow_before = hdmi_underflow_count;
        axis_error_before = hdmi_axis_error_count;
        seen = 0;

        @(negedge pix_clk);
        hdmi_enable = 1'b1;
        for (int cycle = 0; cycle < 80; cycle++) begin
            @(posedge pix_clk);
            #1;
            if (video_de) begin
                check_condition(seen < 2, "HDMI emitted more active pixels than expected");
                if ({video_r, video_g, video_b} !== {expected_y[seen], expected_y[seen], expected_y[seen]}) begin
                    $fatal(1, "CHECK FAILED: HDMI pixel %0d got=%06h expected=%06h", seen, {video_r, video_g, video_b}, {expected_y[seen], expected_y[seen], expected_y[seen]});
                end
                seen++;
                if (seen == 2) begin
                    hdmi_enable <= 1'b0;
                    break;
                end
            end
        end

        check_condition(seen == 2, "HDMI consumed both minimal-line pixels");
        repeat (4) @(posedge pix_clk);
        check_condition(hdmi_underflow_count == underflow_before, "HDMI checked active window had no underflow");
        check_condition(hdmi_axis_error_count == axis_error_before, "HDMI checked active window had no AXIS sideband error");
    endtask

    always_ff @(posedge core_clk) begin
        if (!core_aresetn) begin
            yuv_seen_count <= 0;
        end else if (yuv_pixel_valid) begin
            if (yuv_seen_count == 0) begin
                check_condition(yuv_pixel === {EXPECTED_Y0, EXPECTED_Y0, EXPECTED_Y0}, "YUV first grayscale pixel value");
                check_condition(yuv_pixel_sof, "YUV first pixel carries SOF");
                check_condition(!yuv_pixel_eol, "YUV first pixel does not carry EOL");
            end else if (yuv_seen_count == 1) begin
                check_condition(yuv_pixel === {EXPECTED_Y1, EXPECTED_Y1, EXPECTED_Y1}, "YUV second grayscale pixel value");
                check_condition(!yuv_pixel_sof, "YUV second pixel does not carry SOF");
                check_condition(yuv_pixel_eol, "YUV second pixel carries EOL");
            end else begin
                $fatal(1, "CHECK FAILED: unexpected extra YUV pixel %0d", yuv_seen_count);
            end
            check_condition(!yuv_pixel_err, "YUV pixel has no frame error");
            yuv_seen_count <= yuv_seen_count + 1;
        end
    end

    always_ff @(posedge pix_clk) begin
        if (!pix_aresetn) begin
            axis_seen_count <= 0;
        end else if (axis_tvalid && axis_tready) begin
            if (axis_seen_count == 0) begin
                check_condition(axis_tdata === {EXPECTED_Y0, EXPECTED_Y0, EXPECTED_Y0}, "AXIS first pixel value");
                check_condition(axis_tuser[0], "AXIS first pixel carries SOF");
                check_condition(!axis_tlast, "AXIS first pixel does not carry TLAST");
            end else if (axis_seen_count == 1) begin
                check_condition(axis_tdata === {EXPECTED_Y1, EXPECTED_Y1, EXPECTED_Y1}, "AXIS second pixel value");
                check_condition(!axis_tuser[0], "AXIS second pixel does not carry SOF");
                check_condition(axis_tlast, "AXIS second pixel carries TLAST");
            end
            axis_seen_count <= axis_seen_count + 1;
        end
    end

    initial begin
        reset_dut();

        send_short_packet(DT_FS, 16'h0000);
        send_yuv422_long_packet(8'h80, EXPECTED_Y0, 8'h10, EXPECTED_Y1);
        send_short_packet(DT_FE, 16'h0000);

        wait_core_count(2);
        wait_frame_done();

        check_condition(parser_short_count == 16'd2, "parser saw FS and FE short packets");
        check_condition(parser_long_count == 16'd1, "parser saw one YUV422 long packet");
        check_condition(parser_trunc_count == 16'd0, "parser saw no truncation");
        check_condition(ecc_uncorr_count == 16'd0, "header ECC has no uncorrectable errors");
        check_condition(crc_ok_count == 16'd1, "payload CRC matched once");
        check_condition(crc_err_count == 16'd0, "payload CRC has no errors");
        check_condition(filter_drop_vc_count == 16'd0, "filter dropped no VC packets");
        check_condition(filter_drop_dt_count == 16'd0, "filter dropped no DT packets");
        check_condition(frame_sync_err_count == 16'd0, "frame state has no sync errors");
        check_condition(line_count == 32'd1, "frame state counted one line");
        check_condition(last_frame_lines == 16'd1, "frame state ended one-line frame");
        check_condition(yuv_pixel_per_line == 16'd2, "YUV unpacker counted two pixels per line");
        check_condition(bridge_overflow_count == 16'd0, "direct bridge did not overflow");

        wait_axis_ready();
        check_hdmi_pixels();
        check_condition(axis_seen_count == 2, "AXIS bridge delivered two pixels to HDMI");

        $display("TEST PASSED: tb_mipi_to_hdmi_direct_minimal");
        $finish;
    end

    initial begin
        #2ms;
        $fatal(1, "Simulation timeout");
    end
endmodule