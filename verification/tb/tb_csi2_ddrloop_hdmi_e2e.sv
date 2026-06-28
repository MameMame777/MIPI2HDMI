`timescale 1ns / 1ps

module tb_csi2_ddrloop_hdmi_e2e;
    localparam int PARSER_IN_WIDTH = 16;
    localparam logic [5:0] DT_FS = 6'h00;
    localparam logic [5:0] DT_FE = 6'h01;
    localparam logic [5:0] DT_YUV422 = 6'h1e;

    localparam int LINE_PIXELS = 4;
    localparam int FRAME_LINES = 2;
    localparam int LINE_BYTES = LINE_PIXELS * 2;
    localparam int FRAME_PIXELS = LINE_PIXELS * FRAME_LINES;

    logic [7:0] expected_y [0:FRAME_PIXELS-1] = '{
        8'h10, 8'h20, 8'h30, 8'h40,
        8'h50, 8'h60, 8'h70, 8'h80
    };

    logic core_clk;
    logic core_aresetn;
    logic aclk;
    logic aresetn;

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

    logic [23:0] bridge_axis_tdata;
    logic        bridge_axis_tvalid;
    logic        bridge_axis_tready;
    logic        bridge_axis_tlast;
    logic [1:0]  bridge_axis_tuser;
    logic [15:0] bridge_overflow_count;
    logic [15:0] bridge_back_pressure_count;

    logic [7:0] pack_in_tdata;
    logic       pack_in_tvalid;
    logic       pack_in_tready;
    logic       pack_in_tlast;
    logic [0:0] pack_in_tuser;

    logic [31:0] pack_out_tdata;
    logic [3:0]  pack_out_tkeep;
    logic        pack_out_tvalid;
    logic        pack_out_tready;
    logic        pack_out_tlast;
    logic [0:0]  pack_out_tuser;

    logic [31:0] ddr_out_tdata;
    logic        ddr_out_tvalid;
    logic        ddr_out_tready;
    logic        ddr_out_tlast;
    logic [0:0]  ddr_out_tuser;

    logic [7:0] unpack_out_tdata;
    logic       unpack_out_tvalid;
    logic       unpack_out_tready;
    logic       unpack_out_tlast;
    logic [0:0] unpack_out_tuser;

    logic [23:0] hdmi_in_tdata;
    logic        hdmi_in_tvalid;
    logic        hdmi_in_tready;
    logic        hdmi_in_tlast;
    logic        hdmi_in_tuser;

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

    int unsigned hdmi_seen_count;
    int unsigned ddr_beats_seen;

    initial begin
        core_clk = 1'b0;
        forever #5 core_clk = ~core_clk;
    end

    initial begin
        aclk = 1'b0;
        forever #7 aclk = ~aclk;
    end

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

    assign pack_in_tdata    = bridge_axis_tdata[7:0];
    assign pack_in_tvalid   = bridge_axis_tvalid;
    assign bridge_axis_tready = pack_in_tready;
    assign pack_in_tlast    = bridge_axis_tlast;
    assign pack_in_tuser[0] = bridge_axis_tuser[0];

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

    typedef struct packed {
        logic [31:0] data;
        logic        last;
        logic        user;
    } ddr_beat_t;

    ddr_beat_t ddr_queue [$];
    localparam int DDR_DEPTH = 64;

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
        end else begin
            if (pack_out_tvalid && pack_out_tready) begin
                ddr_queue.push_back('{pack_out_tdata, pack_out_tlast, pack_out_tuser[0]});
                if (pack_out_tkeep !== 4'hf) begin
                    $fatal(1, "DDR loop expects full 32-bit beats (TKEEP=0xF), got 0x%h", pack_out_tkeep);
                end
                ddr_beats_seen <= ddr_beats_seen + 1;
            end
            if (ddr_out_tvalid && ddr_out_tready) begin
                ddr_queue.delete(0);
            end
        end
    end

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

    assign hdmi_in_tdata  = {3{unpack_out_tdata}};
    assign hdmi_in_tvalid = unpack_out_tvalid;
    assign unpack_out_tready = hdmi_in_tready;
    assign hdmi_in_tlast  = unpack_out_tlast;
    assign hdmi_in_tuser  = unpack_out_tuser[0];

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

    task automatic check_condition(input bit condition, input string message);
        if (!condition) begin
            $fatal(1, "CHECK FAILED: %s", message);
        end
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

    // 1 line of 4 YUV422 pixels: U Y0 V Y1 U Y2 V Y3 (WC=8). chroma=0x80 (neutral).
    task automatic send_yuv422_line_4px(input int line_idx);
        automatic logic [7:0] di;
        automatic logic [15:0] wc;
        automatic logic [7:0] ecc;
        automatic logic [15:0] crc;
        automatic logic [7:0] payload [0:7];
        automatic int base = line_idx * LINE_PIXELS;

        di = {2'b00, DT_YUV422};
        wc = 16'd8;
        ecc = make_ecc(di, wc);

        payload[0] = 8'h80; payload[1] = expected_y[base + 0];
        payload[2] = 8'h80; payload[3] = expected_y[base + 1];
        payload[4] = 8'h80; payload[5] = expected_y[base + 2];
        payload[6] = 8'h80; payload[7] = expected_y[base + 3];

        crc = 16'hffff;
        for (int i = 0; i < 8; i++) crc = crc_update_byte(crc, payload[i]);

        drive_beat(di, wc[7:0], 1'b1, 1'b0);
        drive_beat(wc[15:8], ecc, 1'b0, 1'b0);
        drive_beat(payload[0], payload[1], 1'b0, 1'b0);
        drive_beat(payload[2], payload[3], 1'b0, 1'b0);
        drive_beat(payload[4], payload[5], 1'b0, 1'b0);
        drive_beat(payload[6], payload[7], 1'b0, 1'b0);
        drive_beat(crc[7:0], crc[15:8], 1'b0, 1'b1);
        drive_idle(2);
    endtask

    task automatic wait_frame_done();
        for (int cycle = 0; cycle < 4000; cycle++) begin
            @(posedge core_clk);
            if (frame_count == 32'd1) begin
                return;
            end
        end
        $fatal(1, "Timed out waiting for CSI-2 frame completion");
    endtask

    task automatic wait_parser_short_count(input logic [15:0] expected_count);
        for (int cycle = 0; cycle < 4000; cycle++) begin
            @(posedge core_clk);
            if (parser_short_count >= expected_count) begin
                return;
            end
        end
        $fatal(1, "Timed out waiting for parser short count %0d, saw %0d", expected_count, parser_short_count);
    endtask

    task automatic wait_ddr_full_frame();
        // Expect FRAME_LINES beats (1 full beat per line of 4 pixels).
        for (int cycle = 0; cycle < 4000; cycle++) begin
            @(posedge aclk);
            if (ddr_beats_seen == FRAME_LINES) return;
        end
        $fatal(1, "Timed out waiting for %0d DDR beats, saw %0d", FRAME_LINES, ddr_beats_seen);
    endtask

    task automatic check_hdmi_pixels();
        automatic logic [15:0] underflow_before;
        automatic logic [15:0] axis_error_before;

        underflow_before = hdmi_underflow_count;
        axis_error_before = hdmi_axis_error_count;
        hdmi_seen_count = 0;

        @(negedge aclk);
        hdmi_enable = 1'b1;

        for (int cycle = 0; cycle < 4000; cycle++) begin
            @(posedge aclk);
            #1;
            if (video_de) begin
                check_condition(hdmi_seen_count < FRAME_PIXELS,
                    $sformatf("HDMI emitted more active pixels than expected (saw %0d)", hdmi_seen_count));
                if ({video_r, video_g, video_b} !== {expected_y[hdmi_seen_count], expected_y[hdmi_seen_count], expected_y[hdmi_seen_count]}) begin
                    $fatal(1, "HDMI pixel %0d got=%06h expected=%02h%02h%02h",
                        hdmi_seen_count, {video_r, video_g, video_b},
                        expected_y[hdmi_seen_count], expected_y[hdmi_seen_count], expected_y[hdmi_seen_count]);
                end
                hdmi_seen_count = hdmi_seen_count + 1;
                if (hdmi_seen_count == FRAME_PIXELS) begin
                    break;
                end
            end
        end

        check_condition(hdmi_seen_count == FRAME_PIXELS,
            $sformatf("HDMI delivered all %0d pixels of the reconstructed frame", FRAME_PIXELS));
        repeat (8) @(posedge aclk);
        check_condition(hdmi_underflow_count == underflow_before,
            "HDMI active window had no underflow");
        check_condition(hdmi_axis_error_count == axis_error_before,
            "HDMI active window had no AXIS sideband error");
    endtask

    initial begin
        reset_dut();

        send_short_packet(DT_FS, 16'h0000);
        for (int line_idx = 0; line_idx < FRAME_LINES; line_idx++) begin
            send_yuv422_line_4px(line_idx);
        end
        send_short_packet(DT_FE, 16'h0000);

        wait_frame_done();
        wait_parser_short_count(16'd2);

        check_condition(parser_short_count == 16'd2, "parser saw FS and FE short packets");
        check_condition(parser_long_count == 16'(FRAME_LINES), "parser saw one long packet per line");
        check_condition(parser_trunc_count == 16'd0, "parser saw no truncation");
        check_condition(ecc_uncorr_count == 16'd0, "header ECC has no uncorrectable errors");
        check_condition(crc_ok_count == 16'(FRAME_LINES), "payload CRC matched once per line");
        check_condition(crc_err_count == 16'd0, "payload CRC has no errors");
        check_condition(filter_drop_vc_count == 16'd0, "filter dropped no VC packets");
        check_condition(filter_drop_dt_count == 16'd0, "filter dropped no DT packets");
        check_condition(frame_sync_err_count == 16'd0, "frame state has no sync errors");
        check_condition(line_count == 32'(FRAME_LINES), "frame state counted FRAME_LINES lines");
        check_condition(last_frame_lines == 16'(FRAME_LINES), "frame state ended FRAME_LINES-line frame");
        check_condition(yuv_pixel_per_line == 16'(LINE_PIXELS), "YUV unpacker counted LINE_PIXELS per line");
        check_condition(bridge_overflow_count == 16'd0, "video bridge did not overflow");

        wait_ddr_full_frame();
        check_condition(ddr_beats_seen == FRAME_LINES,
            $sformatf("DDR model received %0d packed beats (one per line)", FRAME_LINES));

        check_hdmi_pixels();

        $display("TEST PASSED: tb_csi2_ddrloop_hdmi_e2e");
        $finish;
    end

    initial begin
        #5ms;
        $fatal(1, "Simulation timeout");
    end
endmodule
