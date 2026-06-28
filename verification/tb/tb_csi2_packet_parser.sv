`timescale 1ns / 1ps

module tb_csi2_packet_parser;
    localparam int IN_WIDTH = 16;
    localparam int BYTE_LANES = IN_WIDTH / 8;

    logic core_clk;
    logic core_aresetn;

    logic [IN_WIDTH-1:0] s_byte_data;
    logic [BYTE_LANES-1:0] s_byte_keep;
    logic s_byte_valid;
    logic s_byte_sop;
    logic s_byte_eop;

    logic ecc_hdr_valid;
    logic [31:0] ecc_hdr_raw;
    logic ecc_hdr_corr_valid;
    logic [7:0] ecc_hdr_di;
    logic [15:0] ecc_hdr_wc;
    logic ecc_hdr_uncorrectable;

    logic m_pkt_hdr_valid;
    logic [31:0] m_pkt_hdr_raw;
    logic [7:0] m_pkt_di;
    logic [15:0] m_pkt_wc;
    logic m_pkt_is_long;
    logic m_pkt_is_short;
    logic m_pkt_ecc_uncorrectable;
    logic [7:0] m_payload_data;
    logic m_payload_valid;
    logic m_payload_first;
    logic m_payload_last;
    logic [15:0] m_footer_data;
    logic m_footer_valid;
    logic m_pkt_done;
    logic [15:0] sts_short_pkt_cnt;
    logic [15:0] sts_long_pkt_cnt;
    logic [15:0] sts_pkt_trunc_cnt;

    int payload_seen;
    int done_seen;
    int hdr_seen;
    logic clear_logs_pulse;
    logic [7:0] payload_log [16];
    logic first_log [16];
    logic last_log [16];
    logic [15:0] footer_log;
    logic footer_seen;

    csi2_packet_parser #(
        .IN_WIDTH(IN_WIDTH),
        .WC_MAX(16),
        .FIFO_DEPTH(32)
    ) dut (
        .core_clk(core_clk),
        .core_aresetn(core_aresetn),
        .s_byte_data(s_byte_data),
        .s_byte_keep(s_byte_keep),
        .s_byte_valid(s_byte_valid),
        .s_byte_sop(s_byte_sop),
        .s_byte_eop(s_byte_eop),
        .ecc_hdr_valid(ecc_hdr_valid),
        .ecc_hdr_raw(ecc_hdr_raw),
        .ecc_hdr_corr_valid(ecc_hdr_corr_valid),
        .ecc_hdr_di(ecc_hdr_di),
        .ecc_hdr_wc(ecc_hdr_wc),
        .ecc_hdr_uncorrectable(ecc_hdr_uncorrectable),
        .m_pkt_hdr_valid(m_pkt_hdr_valid),
        .m_pkt_hdr_raw(m_pkt_hdr_raw),
        .m_pkt_di(m_pkt_di),
        .m_pkt_wc(m_pkt_wc),
        .m_pkt_is_long(m_pkt_is_long),
        .m_pkt_is_short(m_pkt_is_short),
        .m_pkt_ecc_uncorrectable(m_pkt_ecc_uncorrectable),
        .m_payload_data(m_payload_data),
        .m_payload_valid(m_payload_valid),
        .m_payload_first(m_payload_first),
        .m_payload_last(m_payload_last),
        .m_footer_data(m_footer_data),
        .m_footer_valid(m_footer_valid),
        .m_pkt_done(m_pkt_done),
        .sts_short_pkt_cnt(sts_short_pkt_cnt),
        .sts_long_pkt_cnt(sts_long_pkt_cnt),
        .sts_pkt_trunc_cnt(sts_pkt_trunc_cnt)
    );

    initial begin
        core_clk = 1'b0;
        forever #5 core_clk = ~core_clk;
    end

    initial begin
        ecc_hdr_corr_valid = 1'b0;
        ecc_hdr_di = 8'h00;
        ecc_hdr_wc = 16'h0000;
        ecc_hdr_uncorrectable = 1'b0;

        forever begin
            @(posedge core_clk);
            ecc_hdr_corr_valid <= 1'b0;
            if (ecc_hdr_valid) begin
                automatic logic [31:0] raw;
                raw = ecc_hdr_raw;
                repeat (2) @(posedge core_clk);
                ecc_hdr_di <= raw[7:0];
                ecc_hdr_wc <= raw[23:8];
                ecc_hdr_uncorrectable <= 1'b0;
                ecc_hdr_corr_valid <= 1'b1;
            end
        end
    end

    always_ff @(posedge core_clk) begin
        if (!core_aresetn || clear_logs_pulse) begin
            payload_seen <= 0;
            done_seen <= 0;
            hdr_seen <= 0;
            footer_seen <= 1'b0;
            footer_log <= 16'h0000;
            for (int idx = 0; idx < 16; idx++) begin
                payload_log[idx] <= 8'h00;
                first_log[idx] <= 1'b0;
                last_log[idx] <= 1'b0;
            end
        end else begin
            if (m_pkt_hdr_valid) begin
                hdr_seen <= hdr_seen + 1;
            end
            if (m_payload_valid) begin
                payload_log[payload_seen] <= m_payload_data;
                first_log[payload_seen] <= m_payload_first;
                last_log[payload_seen] <= m_payload_last;
                payload_seen <= payload_seen + 1;
            end
            if (m_footer_valid) begin
                footer_seen <= 1'b1;
                footer_log <= m_footer_data;
            end
            if (m_pkt_done) begin
                done_seen <= done_seen + 1;
            end
        end
    end

    task automatic reset_dut();
        core_aresetn = 1'b0;
        clear_logs_pulse = 1'b0;
        s_byte_data = '0;
        s_byte_keep = '0;
        s_byte_valid = 1'b0;
        s_byte_sop = 1'b0;
        s_byte_eop = 1'b0;
        repeat (8) @(posedge core_clk);
        core_aresetn = 1'b1;
        repeat (2) @(posedge core_clk);
    endtask

    task automatic clear_logs();
        @(posedge core_clk);
        clear_logs_pulse <= 1'b1;
        @(posedge core_clk);
        clear_logs_pulse <= 1'b0;
        @(posedge core_clk);
    endtask

    task automatic drive_beat(
        input logic [15:0] data,
        input logic [1:0] keep,
        input logic sop,
        input logic eop
    );
        @(posedge core_clk);
        s_byte_data <= data;
        s_byte_keep <= keep;
        s_byte_valid <= 1'b1;
        s_byte_sop <= sop;
        s_byte_eop <= eop;
        @(posedge core_clk);
        s_byte_valid <= 1'b0;
        s_byte_sop <= 1'b0;
        s_byte_eop <= 1'b0;
        s_byte_keep <= 2'b00;
        s_byte_data <= 16'h0000;
    endtask

    task automatic wait_done(input int previous_done);
        for (int cycle = 0; cycle < 200; cycle++) begin
            @(posedge core_clk);
            if (done_seen > previous_done) begin
                return;
            end
        end
        $fatal(1, "Timed out waiting for packet done");
    endtask

    task automatic check_condition(input bit condition, input string message);
        if (!condition) begin
            $fatal(1, "CHECK FAILED: %s", message);
        end
    endtask

    initial begin
        reset_dut();

        clear_logs();
        drive_beat(16'h3400, 2'b11, 1'b1, 1'b0);
        drive_beat(16'h0012, 2'b11, 1'b0, 1'b1);
        wait_done(0);
        check_condition(hdr_seen == 1, "short header event count");
        check_condition(done_seen == 1, "short done count");
        check_condition(sts_short_pkt_cnt == 16'd1, "short status count");
        check_condition(sts_long_pkt_cnt == 16'd0, "long status count after short");
        check_condition(payload_seen == 0, "short packet has no payload");
        check_condition(m_pkt_di == 8'h00, "short DI");
        check_condition(m_pkt_wc == 16'h1234, "short WC");
        check_condition(m_pkt_is_short == 1'b1, "short classification");

        clear_logs();
        drive_beat(16'h032a, 2'b11, 1'b1, 1'b0);
        drive_beat(16'h0000, 2'b11, 1'b0, 1'b0);
        drive_beat(16'hbbaa, 2'b11, 1'b0, 1'b0);
        drive_beat(16'h34cc, 2'b11, 1'b0, 1'b0);
        drive_beat(16'h0012, 2'b01, 1'b0, 1'b1);
        wait_done(0);
        check_condition(hdr_seen == 1, "long header event count");
        check_condition(done_seen == 1, "long done count");
        check_condition(sts_long_pkt_cnt == 16'd1, "long status count");
        check_condition(payload_seen == 3, "long payload count");
        check_condition(payload_log[0] == 8'haa, "payload byte 0");
        check_condition(payload_log[1] == 8'hbb, "payload byte 1");
        check_condition(payload_log[2] == 8'hcc, "payload byte 2");
        check_condition(first_log[0] == 1'b1, "payload first");
        check_condition(last_log[2] == 1'b1, "payload last");
        check_condition(footer_seen == 1'b1, "footer valid");
        check_condition(footer_log == 16'h1234, "footer byte order");

        clear_logs();
        drive_beat(16'h042a, 2'b11, 1'b1, 1'b0);
        drive_beat(16'h0000, 2'b11, 1'b0, 1'b0);
        drive_beat(16'h2211, 2'b11, 1'b0, 1'b1);
        wait_done(0);
        check_condition(done_seen == 1, "truncated done count");
        check_condition(sts_pkt_trunc_cnt == 16'd1, "truncate status count");
        check_condition(payload_seen == 2, "truncated payload count before recovery");

        repeat (10) @(posedge core_clk);
        $display("TEST PASSED: tb_csi2_packet_parser");
        $finish;
    end

    initial begin
        #1ms;
        $fatal(1, "Simulation timeout");
    end
endmodule
