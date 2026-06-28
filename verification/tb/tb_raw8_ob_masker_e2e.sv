`timescale 1ns / 1ps

// E2E: RAW8 MIPI → parser → filter → frame_state → raw8_passthrough →
//      ob_row_masker (WIDTH=8) → captured pixel stream, verified pixel-by-pixel.

module tb_raw8_ob_masker_e2e;
    localparam int PARSER_IN_WIDTH = 16;
    localparam logic [5:0] DT_FS   = 6'h00;
    localparam logic [5:0] DT_FE   = 6'h01;
    localparam logic [5:0] DT_RAW8 = 6'h2A;

    localparam int LINE_PIXELS  = 8;
    localparam int FRAME_LINES  = 4;
    localparam int LINE_BYTES   = LINE_PIXELS;  // RAW8: 1 byte per pixel
    localparam int FRAME_PIXELS = LINE_PIXELS * FRAME_LINES;

    logic [7:0] input_y [0:FRAME_PIXELS-1] = '{
        // L0: OB uniform Y=36
        8'd36, 8'd36, 8'd36, 8'd36, 8'd36, 8'd36, 8'd36, 8'd36,
        // L1: checkerboard 10/240
        8'd10, 8'd10, 8'd10, 8'd10, 8'd240, 8'd240, 8'd240, 8'd240,
        // L2: gradient
        8'd0, 8'd32, 8'd64, 8'd96, 8'd128, 8'd160, 8'd192, 8'd224,
        // L3: Bayer-like
        8'd80, 8'd180, 8'd180, 8'd80, 8'd80, 8'd180, 8'd180, 8'd80
    };

    logic [7:0] expected_y [0:FRAME_PIXELS-1] = '{
        // L0: masked → 128
        8'd128, 8'd128, 8'd128, 8'd128, 8'd128, 8'd128, 8'd128, 8'd128,
        // L1: pass-through
        8'd10, 8'd10, 8'd10, 8'd10, 8'd240, 8'd240, 8'd240, 8'd240,
        // L2: pass-through
        8'd0, 8'd32, 8'd64, 8'd96, 8'd128, 8'd160, 8'd192, 8'd224,
        // L3: Bayer-like → pass-through (range=100 > 3)
        8'd80, 8'd180, 8'd180, 8'd80, 8'd80, 8'd180, 8'd180, 8'd80
    };

    logic core_clk;
    logic core_aresetn;

    logic [PARSER_IN_WIDTH-1:0]   s_byte_data;
    logic [PARSER_IN_WIDTH/8-1:0] s_byte_keep;
    logic s_byte_valid, s_byte_sop, s_byte_eop;

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
    logic [15:0] parser_short_count, parser_long_count, parser_trunc_count;
    logic        crc_check_valid, crc_match;
    logic [15:0] crc_calc, crc_received, crc_err_count, crc_ok_count;
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
    logic [31:0] frame_count, line_count;
    logic [15:0] last_frame_lines, frame_sync_err_count;

    logic [7:0]  raw8_pixel;
    logic        raw8_valid, raw8_sof, raw8_eol, raw8_eof, raw8_err;
    logic [15:0] raw8_pix_per_line;

    logic [7:0]  ob_pixel;
    logic        ob_valid, ob_sof, ob_eol, ob_eof, ob_err;

    int unsigned errors_cnt = 0;
    logic [7:0]  ob_capture [$];

    initial begin core_clk = 0; forever #5 core_clk = ~core_clk; end

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

    always_ff @(posedge core_clk) begin
        if (ob_valid) ob_capture.push_back(ob_pixel);
    end

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
        automatic logic fb;
        crc_next = crc_in;
        for (int b = 0; b < 8; b++) begin
            fb = crc_next[0] ^ data[b];
            crc_next = crc_next >> 1;
            if (fb) crc_next = crc_next ^ 16'h8408;
        end
        crc_update_byte = crc_next;
    endfunction

    task automatic reset_dut();
        core_aresetn = 0;
        s_byte_data = 0; s_byte_keep = 0; s_byte_valid = 0; s_byte_sop = 0; s_byte_eop = 0;
        repeat (8) @(posedge core_clk);
        core_aresetn = 1;
        repeat (4) @(posedge core_clk);
    endtask

    task automatic drive_idle(input int n);
        @(negedge core_clk);
        s_byte_valid <= 0; s_byte_keep <= 0; s_byte_sop <= 0; s_byte_eop <= 0; s_byte_data <= 0;
        repeat (n) @(negedge core_clk);
    endtask

    task automatic drive_beat(input logic [7:0] b0, input logic [7:0] b1,
                               input logic sop, input logic eop);
        @(negedge core_clk);
        s_byte_data <= {b1, b0};
        s_byte_keep <= 2'b11;
        s_byte_valid <= 1; s_byte_sop <= sop; s_byte_eop <= eop;
    endtask

    task automatic send_short_packet(input logic [5:0] dt, input logic [15:0] data_field);
        automatic logic [7:0] di, ecc;
        di = {2'b00, dt};
        ecc = make_ecc(di, data_field);
        drive_beat(di, data_field[7:0], 1'b1, 1'b0);
        drive_beat(data_field[15:8], ecc, 1'b0, 1'b1);
        drive_idle(2);
    endtask

    task automatic send_raw8_line(input int line_idx);
        automatic logic [7:0] di, ecc;
        automatic logic [15:0] wc, crc;
        automatic logic [7:0] payload [0:LINE_BYTES-1];
        automatic int base = line_idx * LINE_PIXELS;

        di = {2'b00, DT_RAW8};
        wc = 16'(LINE_BYTES);
        ecc = make_ecc(di, wc);
        for (int p = 0; p < LINE_PIXELS; p++) payload[p] = input_y[base + p];

        crc = 16'hffff;
        for (int i = 0; i < LINE_BYTES; i++) crc = crc_update_byte(crc, payload[i]);

        drive_beat(di, wc[7:0], 1'b1, 1'b0);
        drive_beat(wc[15:8], ecc, 1'b0, 1'b0);
        for (int i = 0; i < LINE_BYTES; i += 2)
            drive_beat(payload[i], (i+1 < LINE_BYTES) ? payload[i+1] : 8'h00, 1'b0, 1'b0);
        drive_beat(crc[7:0], crc[15:8], 1'b0, 1'b1);
        drive_idle(8);
    endtask

    task automatic wait_frame_done();
        for (int c = 0; c < 8000; c++) begin
            @(posedge core_clk);
            if (frame_count == 32'd1) return;
        end
        $fatal(1, "frame timeout");
    endtask

    task automatic wait_parser_short(input logic [15:0] n);
        for (int c = 0; c < 8000; c++) begin
            @(posedge core_clk);
            if (parser_short_count >= n) return;
        end
        $fatal(1, "short pkt timeout");
    endtask

    initial begin
        reset_dut();

        send_short_packet(DT_FS, 16'h0000);
        for (int i = 0; i < FRAME_LINES; i++) send_raw8_line(i);
        send_short_packet(DT_FE, 16'h0000);

        wait_frame_done();
        wait_parser_short(16'd2);

        if (parser_short_count < 2) $fatal(1, "no FE");
        if (parser_long_count != FRAME_LINES) $fatal(1, "long count");
        if (crc_err_count != 0) $fatal(1, "crc err");
        if (last_frame_lines != FRAME_LINES) $fatal(1, "last_frame_lines");

        repeat (200) @(posedge core_clk);

        $display("[INFO] ob_capture.size() = %0d (expect %0d)", ob_capture.size(), FRAME_PIXELS);

        if (ob_capture.size() !== FRAME_PIXELS) begin
            $display("[FAIL] captured %0d, expected %0d", ob_capture.size(), FRAME_PIXELS);
            errors_cnt++;
        end else begin
            for (int i = 0; i < FRAME_PIXELS; i++) begin
                if (ob_capture[i] !== expected_y[i]) begin
                    $display("[FAIL] pix[%0d] got=0x%02h expected=0x%02h",
                             i, ob_capture[i], expected_y[i]);
                    errors_cnt++;
                end
            end
        end

        if (errors_cnt == 0)
            $display("\n==== tb_raw8_ob_masker_e2e PASSED (%0d pixels) ====", FRAME_PIXELS);
        else
            $display("\n==== FAILED: %0d errors ====", errors_cnt);
        $finish;
    end

    initial begin
        #5_000_000;
        $fatal(1, "global timeout");
    end
endmodule
