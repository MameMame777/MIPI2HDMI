`timescale 1ns / 1ps

module tb_csi2_vcdt_filter;
    logic core_clk;
    logic core_aresetn;

    logic [1:0] cfg_expected_vc;
    logic [5:0] cfg_expected_dt;
    logic cfg_pass_short;
    logic cfg_pass_emb_data;

    logic pkt_hdr_valid;
    logic [7:0] pkt_di;
    logic [15:0] pkt_wc;
    logic pkt_is_long;
    logic pkt_is_short;
    logic pkt_done;
    logic ecc_corrected;
    logic ecc_uncorrectable;
    logic crc_check_valid;
    logic crc_match;
    logic [7:0] payload_data;
    logic payload_valid;
    logic payload_first;
    logic payload_last;

    logic [7:0] out_pkt_di;
    logic [15:0] out_pkt_wc;
    logic out_pkt_is_short;
    logic out_pkt_is_long;
    logic out_pkt_start;
    logic out_pkt_end;
    logic out_pkt_err;
    logic [7:0] out_payload_data;
    logic out_payload_valid;
    logic out_payload_first;
    logic out_payload_last;
    logic [15:0] sts_drop_vc_cnt;
    logic [15:0] sts_drop_dt_cnt;

    int start_count;
    int end_count;
    int payload_count;
    int err_count;
    logic [7:0] payload_log [16];
    logic first_log [16];
    logic last_log [16];
    logic [7:0] last_start_di;
    logic [15:0] last_start_wc;
    logic last_start_short;
    logic last_start_long;
    logic clear_logs_pulse;

    csi2_vcdt_filter dut (
        .core_clk(core_clk),
        .core_aresetn(core_aresetn),
        .cfg_expected_vc(cfg_expected_vc),
        .cfg_expected_dt(cfg_expected_dt),
        .cfg_pass_short(cfg_pass_short),
        .cfg_pass_emb_data(cfg_pass_emb_data),
        .pkt_hdr_valid(pkt_hdr_valid),
        .pkt_di(pkt_di),
        .pkt_wc(pkt_wc),
        .pkt_is_long(pkt_is_long),
        .pkt_is_short(pkt_is_short),
        .pkt_done(pkt_done),
        .ecc_corrected(ecc_corrected),
        .ecc_uncorrectable(ecc_uncorrectable),
        .crc_check_valid(crc_check_valid),
        .crc_match(crc_match),
        .payload_data(payload_data),
        .payload_valid(payload_valid),
        .payload_first(payload_first),
        .payload_last(payload_last),
        .out_pkt_di(out_pkt_di),
        .out_pkt_wc(out_pkt_wc),
        .out_pkt_is_short(out_pkt_is_short),
        .out_pkt_is_long(out_pkt_is_long),
        .out_pkt_start(out_pkt_start),
        .out_pkt_end(out_pkt_end),
        .out_pkt_err(out_pkt_err),
        .out_payload_data(out_payload_data),
        .out_payload_valid(out_payload_valid),
        .out_payload_first(out_payload_first),
        .out_payload_last(out_payload_last),
        .sts_drop_vc_cnt(sts_drop_vc_cnt),
        .sts_drop_dt_cnt(sts_drop_dt_cnt)
    );

    initial begin
        core_clk = 1'b0;
        forever #5 core_clk = ~core_clk;
    end

    always_ff @(posedge core_clk) begin
        if (!core_aresetn || clear_logs_pulse) begin
            start_count <= 0;
            end_count <= 0;
            payload_count <= 0;
            err_count <= 0;
            last_start_di <= 8'h00;
            last_start_wc <= 16'h0000;
            last_start_short <= 1'b0;
            last_start_long <= 1'b0;
            for (int idx = 0; idx < 16; idx++) begin
                payload_log[idx] <= 8'h00;
                first_log[idx] <= 1'b0;
                last_log[idx] <= 1'b0;
            end
        end else begin
            if (out_pkt_start) begin
                start_count <= start_count + 1;
                last_start_di <= out_pkt_di;
                last_start_wc <= out_pkt_wc;
                last_start_short <= out_pkt_is_short;
                last_start_long <= out_pkt_is_long;
            end
            if (out_payload_valid) begin
                payload_log[payload_count] <= out_payload_data;
                first_log[payload_count] <= out_payload_first;
                last_log[payload_count] <= out_payload_last;
                payload_count <= payload_count + 1;
            end
            if (out_pkt_end) begin
                end_count <= end_count + 1;
                if (out_pkt_err) begin
                    err_count <= err_count + 1;
                end
            end
        end
    end

    task automatic reset_dut();
        core_aresetn = 1'b0;
        clear_logs_pulse = 1'b0;
        cfg_expected_vc = 2'd0;
        cfg_expected_dt = 6'h2a;
        cfg_pass_short = 1'b1;
        cfg_pass_emb_data = 1'b0;
        pkt_hdr_valid = 1'b0;
        pkt_di = 8'h00;
        pkt_wc = 16'h0000;
        pkt_is_long = 1'b0;
        pkt_is_short = 1'b0;
        pkt_done = 1'b0;
        ecc_corrected = 1'b0;
        ecc_uncorrectable = 1'b0;
        crc_check_valid = 1'b0;
        crc_match = 1'b1;
        payload_data = 8'h00;
        payload_valid = 1'b0;
        payload_first = 1'b0;
        payload_last = 1'b0;
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

    task automatic drive_header(
        input logic [7:0] di,
        input logic [15:0] wc,
        input logic is_long,
        input logic is_short,
        input logic ecc_uncorr
    );
        @(posedge core_clk);
        pkt_di <= di;
        pkt_wc <= wc;
        pkt_is_long <= is_long;
        pkt_is_short <= is_short;
        ecc_uncorrectable <= ecc_uncorr;
        pkt_hdr_valid <= 1'b1;
        @(posedge core_clk);
        pkt_hdr_valid <= 1'b0;
        ecc_uncorrectable <= 1'b0;
    endtask

    task automatic drive_payload(
        input logic [7:0] data,
        input logic first,
        input logic last
    );
        @(posedge core_clk);
        payload_data <= data;
        payload_first <= first;
        payload_last <= last;
        payload_valid <= 1'b1;
        @(posedge core_clk);
        payload_valid <= 1'b0;
        payload_first <= 1'b0;
        payload_last <= 1'b0;
    endtask

    task automatic drive_done(input logic crc_valid, input logic crc_ok);
        @(posedge core_clk);
        crc_check_valid <= crc_valid;
        crc_match <= crc_ok;
        pkt_done <= 1'b1;
        @(posedge core_clk);
        crc_check_valid <= 1'b0;
        crc_match <= 1'b1;
        pkt_done <= 1'b0;
    endtask

    task automatic drive_short_packet_same_cycle(
        input logic [7:0] di,
        input logic [15:0] wc,
        input logic ecc_uncorr
    );
        @(posedge core_clk);
        pkt_di <= di;
        pkt_wc <= wc;
        pkt_is_long <= 1'b0;
        pkt_is_short <= 1'b1;
        ecc_uncorrectable <= ecc_uncorr;
        pkt_hdr_valid <= 1'b1;
        pkt_done <= 1'b1;
        @(posedge core_clk);
        pkt_hdr_valid <= 1'b0;
        pkt_done <= 1'b0;
        ecc_uncorrectable <= 1'b0;
    endtask

    task automatic check_condition(input bit condition, input string message);
        if (!condition) begin
            $fatal(1, "CHECK FAILED: %s", message);
        end
    endtask

    initial begin
        reset_dut();

        clear_logs();
        drive_header(8'h2a, 16'd3, 1'b1, 1'b0, 1'b0);
        drive_payload(8'haa, 1'b1, 1'b0);
        drive_payload(8'hbb, 1'b0, 1'b0);
        drive_payload(8'hcc, 1'b0, 1'b1);
        drive_done(1'b1, 1'b1);
        repeat (3) @(posedge core_clk);
        check_condition(start_count == 1, "RAW8 packet start passes");
        check_condition(end_count == 1, "RAW8 packet end passes");
        check_condition(payload_count == 3, "RAW8 payload count");
        check_condition(payload_log[0] == 8'haa, "payload byte 0");
        check_condition(payload_log[1] == 8'hbb, "payload byte 1");
        check_condition(payload_log[2] == 8'hcc, "payload byte 2");
        check_condition(first_log[0] == 1'b1, "payload first passes");
        check_condition(last_log[2] == 1'b1, "payload last passes");
        check_condition(err_count == 0, "no error for clean packet");
        check_condition(last_start_di == 8'h2a, "start DI pass");
        check_condition(last_start_wc == 16'd3, "start WC pass");
        check_condition(last_start_long == 1'b1, "long flag pass");

        clear_logs();
        drive_short_packet_same_cycle(8'h00, 16'h1234, 1'b0);
        repeat (3) @(posedge core_clk);
        check_condition(start_count == 1, "short packet start passes");
        check_condition(end_count == 1, "short packet end passes");
        check_condition(last_start_short == 1'b1, "short flag pass");
        check_condition(payload_count == 0, "short packet has no payload");

        clear_logs();
        drive_header(8'haa, 16'd2, 1'b1, 1'b0, 1'b0);
        drive_payload(8'h11, 1'b1, 1'b0);
        drive_payload(8'h22, 1'b0, 1'b1);
        drive_done(1'b1, 1'b1);
        repeat (3) @(posedge core_clk);
        check_condition(start_count == 0, "VC mismatch suppresses start");
        check_condition(end_count == 0, "VC mismatch suppresses end");
        check_condition(payload_count == 0, "VC mismatch suppresses payload");
        check_condition(sts_drop_vc_cnt == 16'd1, "VC drop count");

        clear_logs();
        drive_header(8'h2b, 16'd2, 1'b1, 1'b0, 1'b0);
        drive_payload(8'h33, 1'b1, 1'b0);
        drive_payload(8'h44, 1'b0, 1'b1);
        drive_done(1'b1, 1'b1);
        repeat (3) @(posedge core_clk);
        check_condition(start_count == 0, "DT mismatch suppresses start");
        check_condition(payload_count == 0, "DT mismatch suppresses payload");
        check_condition(sts_drop_dt_cnt == 16'd1, "DT drop count");

        cfg_pass_emb_data = 1'b1;
        clear_logs();
        drive_header(8'h12, 16'd1, 1'b1, 1'b0, 1'b0);
        drive_payload(8'h55, 1'b1, 1'b1);
        drive_done(1'b1, 1'b1);
        repeat (3) @(posedge core_clk);
        check_condition(start_count == 1, "embedded data pass when enabled");
        check_condition(payload_count == 1, "embedded payload pass");

        clear_logs();
        drive_header(8'h2a, 16'd1, 1'b1, 1'b0, 1'b1);
        drive_payload(8'h66, 1'b1, 1'b1);
        drive_done(1'b1, 1'b1);
        repeat (3) @(posedge core_clk);
        check_condition(end_count == 1, "ECC error packet ends");
        check_condition(err_count == 1, "ECC uncorrectable becomes packet error");
        check_condition(payload_count == 1, "ECC error frame is not dropped");

        clear_logs();
        drive_header(8'h2a, 16'd1, 1'b1, 1'b0, 1'b0);
        drive_payload(8'h77, 1'b1, 1'b1);
        drive_done(1'b1, 1'b0);
        repeat (3) @(posedge core_clk);
        check_condition(end_count == 1, "CRC error packet ends");
        check_condition(err_count == 1, "CRC mismatch becomes packet error");
        check_condition(payload_count == 1, "CRC error frame is not dropped");

        cfg_pass_short = 1'b0;
        clear_logs();
        drive_header(8'h01, 16'h0000, 1'b0, 1'b1, 1'b0);
        drive_done(1'b0, 1'b1);
        repeat (3) @(posedge core_clk);
        check_condition(start_count == 0, "short packet can be blocked");
        check_condition(sts_drop_dt_cnt == 16'd2, "short block increments DT drop count");

        repeat (10) @(posedge core_clk);
        $display("TEST PASSED: tb_csi2_vcdt_filter");
        $finish;
    end

    initial begin
        #1ms;
        $fatal(1, "Simulation timeout");
    end
endmodule
