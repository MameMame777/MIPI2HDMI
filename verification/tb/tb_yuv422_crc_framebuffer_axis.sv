`timescale 1ns / 1ps

module tb_yuv422_crc_framebuffer_axis;
    localparam int WIDTH = 4;
    localparam int HEIGHT = 3;
    localparam int LINE_BYTES = WIDTH * 2;

    logic core_clk;
    logic core_aresetn;
    logic pix_clk;
    logic pix_aresetn;
    logic [7:0] pkt_di;
    logic [15:0] pkt_wc;
    logic pkt_is_short;
    logic pkt_is_long;
    logic pkt_start;
    logic pkt_end;
    logic pkt_err;
    logic [7:0] payload_data;
    logic payload_valid;
    logic payload_first;
    logic payload_last;
    logic crc_check_valid;
    logic crc_match;
    logic [23:0] m_axis_tdata;
    logic m_axis_tvalid;
    logic m_axis_tready;
    logic m_axis_tlast;
    logic [0:0] m_axis_tuser;
    logic [15:0] sts_good_line_count;
    logic [15:0] sts_bad_line_count;
    logic [31:0] sts_frame_count;
    logic [15:0] sts_write_line;
    logic sts_frame_ready;

    yuv422_crc_framebuffer_axis #(
        .WIDTH(WIDTH),
        .HEIGHT(HEIGHT),
        .LINE_BYTES(LINE_BYTES),
        .TDATA_WIDTH(24)
    ) dut (
        .core_clk(core_clk),
        .core_aresetn(core_aresetn),
        .pix_clk(pix_clk),
        .pix_aresetn(pix_aresetn),
        .pkt_di(pkt_di),
        .pkt_wc(pkt_wc),
        .pkt_is_short(pkt_is_short),
        .pkt_is_long(pkt_is_long),
        .pkt_start(pkt_start),
        .pkt_end(pkt_end),
        .pkt_err(pkt_err),
        .payload_data(payload_data),
        .payload_valid(payload_valid),
        .payload_first(payload_first),
        .payload_last(payload_last),
        .crc_check_valid(crc_check_valid),
        .crc_match(crc_match),
        .m_axis_tdata(m_axis_tdata),
        .m_axis_tvalid(m_axis_tvalid),
        .m_axis_tready(m_axis_tready),
        .m_axis_tlast(m_axis_tlast),
        .m_axis_tuser(m_axis_tuser),
        .sts_good_line_count(sts_good_line_count),
        .sts_bad_line_count(sts_bad_line_count),
        .sts_frame_count(sts_frame_count),
        .sts_write_line(sts_write_line),
        .sts_frame_ready(sts_frame_ready)
    );

    initial begin
        core_clk = 1'b0;
        forever #5 core_clk = ~core_clk;
    end

    initial begin
        pix_clk = 1'b0;
        forever #7 pix_clk = ~pix_clk;
    end

    task automatic check_condition(input bit condition, input string message);
        if (!condition) begin
            $fatal(1, "CHECK FAILED: %s", message);
        end
    endtask

    task automatic reset_dut();
        core_aresetn = 1'b0;
        pix_aresetn = 1'b0;
        pkt_di = 8'h00;
        pkt_wc = 16'd0;
        pkt_is_short = 1'b0;
        pkt_is_long = 1'b0;
        pkt_start = 1'b0;
        pkt_end = 1'b0;
        pkt_err = 1'b0;
        payload_data = 8'h00;
        payload_valid = 1'b0;
        payload_first = 1'b0;
        payload_last = 1'b0;
        crc_check_valid = 1'b0;
        crc_match = 1'b0;
        m_axis_tready = 1'b1;
        repeat (6) @(posedge core_clk);
        core_aresetn = 1'b1;
        repeat (6) @(posedge pix_clk);
        pix_aresetn = 1'b1;
        repeat (4) @(posedge core_clk);
    endtask

    task automatic start_yuv_packet(input logic packet_err);
        @(posedge core_clk);
        pkt_di <= 8'h1e;
        pkt_wc <= LINE_BYTES[15:0];
        pkt_is_short <= 1'b0;
        pkt_is_long <= 1'b1;
        pkt_start <= 1'b1;
        pkt_end <= 1'b0;
        pkt_err <= packet_err;
        @(posedge core_clk);
        pkt_start <= 1'b0;
        pkt_err <= 1'b0;
    endtask

    task automatic drive_payload_line(input logic [7:0] y0, input logic [7:0] y1, input logic [7:0] y2, input logic [7:0] y3);
        logic [7:0] line_bytes [0:LINE_BYTES-1];
        line_bytes[0] = 8'h80;
        line_bytes[1] = y0;
        line_bytes[2] = 8'h81;
        line_bytes[3] = y1;
        line_bytes[4] = 8'h82;
        line_bytes[5] = y2;
        line_bytes[6] = 8'h83;
        line_bytes[7] = y3;

        for (int idx = 0; idx < LINE_BYTES; idx++) begin
            @(posedge core_clk);
            payload_data <= line_bytes[idx];
            payload_valid <= 1'b1;
            payload_first <= (idx == 0);
            payload_last <= (idx == LINE_BYTES - 1);
        end
        @(posedge core_clk);
        payload_valid <= 1'b0;
        payload_first <= 1'b0;
        payload_last <= 1'b0;
        pkt_end <= 1'b1;
        @(posedge core_clk);
        pkt_end <= 1'b0;
    endtask

    task automatic finish_crc(input logic good_crc);
        @(posedge core_clk);
        crc_match <= good_crc;
        crc_check_valid <= 1'b1;
        @(posedge core_clk);
        crc_check_valid <= 1'b0;
        crc_match <= 1'b0;
        pkt_is_long <= 1'b0;
    endtask

    task automatic send_line(input logic good_crc, input logic [7:0] y0, input logic [7:0] y1, input logic [7:0] y2, input logic [7:0] y3);
        start_yuv_packet(1'b0);
        drive_payload_line(y0, y1, y2, y3);
        finish_crc(good_crc);
    endtask

    task automatic wait_core_replay_done(input int expected_good_lines);
        for (int cycle = 0; cycle < 100; cycle++) begin
            @(posedge core_clk);
            if (sts_good_line_count == expected_good_lines[15:0]) begin
                return;
            end
        end
        $fatal(1, "Timed out waiting for good line count %0d", expected_good_lines);
    endtask

    task automatic wait_bad_count(input int expected_bad_lines);
        for (int cycle = 0; cycle < 20; cycle++) begin
            @(posedge core_clk);
            if (sts_bad_line_count == expected_bad_lines[15:0]) begin
                return;
            end
        end
        $fatal(1, "Timed out waiting for bad line count %0d", expected_bad_lines);
    endtask

    task automatic wait_axis_sof();
        for (int cycle = 0; cycle < 200; cycle++) begin
            @(posedge pix_clk);
            #1;
            if (m_axis_tvalid && m_axis_tready && m_axis_tuser[0]) begin
                return;
            end
        end
        $fatal(1, "Timed out waiting for AXIS SOF");
    endtask

    task automatic check_next_line(input logic [7:0] y0, input logic [7:0] y1, input logic [7:0] y2, input logic [7:0] y3);
        logic [7:0] expected [0:WIDTH-1];
        expected[0] = y0;
        expected[1] = y1;
        expected[2] = y2;
        expected[3] = y3;

        wait_axis_sof();
        for (int idx = 0; idx < WIDTH; idx++) begin
            if (idx != 0) begin
                @(posedge pix_clk);
                #1;
            end
            check_condition(m_axis_tvalid, "AXIS valid during displayed line");
            if (m_axis_tdata !== {expected[idx], expected[idx], expected[idx]}) begin
                $fatal(1, "CHECK FAILED: displayed pixel %0d got=%06h expected=%06h", idx, m_axis_tdata, {expected[idx], expected[idx], expected[idx]});
            end
            check_condition(m_axis_tlast == (idx == WIDTH - 1), "AXIS tlast position");
        end
    endtask

    initial begin
        reset_dut();

        send_line(1'b0, 8'h10, 8'h20, 8'h30, 8'h40);
        wait_bad_count(1);
        check_condition(!sts_frame_ready, "bad CRC does not ready frame");
        check_condition(sts_good_line_count == 16'd0, "bad CRC does not increment good lines");

        send_line(1'b1, 8'h11, 8'h22, 8'h33, 8'h44);
        wait_core_replay_done(1);
        check_condition(sts_frame_ready, "good CRC readies frame");
        check_condition(sts_frame_count == 32'd1, "good CRC increments frame count once");
        check_condition(sts_write_line == 16'd1, "write line advances after good replay");
        check_next_line(8'h11, 8'h22, 8'h33, 8'h44);

        send_line(1'b1, 8'h55, 8'h66, 8'h77, 8'h88);
        wait_core_replay_done(2);
        check_condition(sts_frame_count == 32'd2, "second good CRC increments frame count");
        check_condition(sts_write_line == 16'd2, "write line advances again");
        check_next_line(8'h55, 8'h66, 8'h77, 8'h88);

        $display("TEST PASSED: tb_yuv422_crc_framebuffer_axis");
        $finish;
    end

    initial begin
        #2ms;
        $fatal(1, "Simulation timeout");
    end
endmodule