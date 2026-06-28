`timescale 1ns / 1ps

module tb_csi2_frame_state;
    logic core_clk;
    logic core_aresetn;
    logic cfg_use_lsle;

    logic [7:0] in_pkt_di;
    logic [15:0] in_pkt_wc;
    logic in_pkt_is_short;
    logic in_pkt_is_long;
    logic in_pkt_start;
    logic in_pkt_end;
    logic in_pkt_err;
    logic [7:0] in_payload_data;
    logic in_payload_valid;
    logic in_payload_first;
    logic in_payload_last;

    logic out_sof;
    logic out_eof;
    logic out_sol;
    logic out_eol;
    logic [15:0] out_line_idx;
    logic [7:0] out_payload_data;
    logic out_payload_valid;
    logic out_payload_first;
    logic out_payload_last;
    logic out_frame_err;
    logic [31:0] sts_frame_count;
    logic [31:0] sts_line_count;
    logic [15:0] sts_last_frame_lines;
    logic [15:0] sts_frame_sync_err_cnt;

    int sof_count;
    int eof_count;
    int sol_count;
    int eol_count;
    int payload_count;
    int frame_err_count;
    logic [7:0] payload_log [16];
    logic [15:0] line_idx_log [16];
    logic clear_logs_pulse;

    csi2_frame_state #(
        .MAX_LINES(512)
    ) dut (
        .core_clk(core_clk),
        .core_aresetn(core_aresetn),
        .cfg_use_lsle(cfg_use_lsle),
        .cfg_expected_frame_lines(16'd0),
        .in_pkt_di(in_pkt_di),
        .in_pkt_wc(in_pkt_wc),
        .in_pkt_is_short(in_pkt_is_short),
        .in_pkt_is_long(in_pkt_is_long),
        .in_pkt_start(in_pkt_start),
        .in_pkt_end(in_pkt_end),
        .in_pkt_err(in_pkt_err),
        .in_payload_data(in_payload_data),
        .in_payload_valid(in_payload_valid),
        .in_payload_first(in_payload_first),
        .in_payload_last(in_payload_last),
        .out_sof(out_sof),
        .out_eof(out_eof),
        .out_sol(out_sol),
        .out_eol(out_eol),
        .out_line_idx(out_line_idx),
        .out_payload_data(out_payload_data),
        .out_payload_valid(out_payload_valid),
        .out_payload_first(out_payload_first),
        .out_payload_last(out_payload_last),
        .out_frame_err(out_frame_err),
        .sts_frame_count(sts_frame_count),
        .sts_line_count(sts_line_count),
        .sts_last_frame_lines(sts_last_frame_lines),
        .sts_frame_sync_err_cnt(sts_frame_sync_err_cnt)
    );

    initial begin
        core_clk = 1'b0;
        forever #5 core_clk = ~core_clk;
    end

    always_ff @(posedge core_clk) begin
        if (!core_aresetn || clear_logs_pulse) begin
            sof_count <= 0;
            eof_count <= 0;
            sol_count <= 0;
            eol_count <= 0;
            payload_count <= 0;
            frame_err_count <= 0;
            for (int idx = 0; idx < 16; idx++) begin
                payload_log[idx] <= 8'h00;
                line_idx_log[idx] <= 16'h0000;
            end
        end else begin
            if (out_sof) begin
                sof_count <= sof_count + 1;
            end
            if (out_eof) begin
                eof_count <= eof_count + 1;
            end
            if (out_sol) begin
                sol_count <= sol_count + 1;
            end
            if (out_eol) begin
                eol_count <= eol_count + 1;
            end
            if (out_frame_err) begin
                frame_err_count <= frame_err_count + 1;
            end
            if (out_payload_valid) begin
                if (payload_count < 16) begin
                    payload_log[payload_count] <= out_payload_data;
                    line_idx_log[payload_count] <= out_line_idx;
                end
                payload_count <= payload_count + 1;
            end
        end
    end

    task automatic reset_dut();
        core_aresetn = 1'b0;
        clear_logs_pulse = 1'b0;
        cfg_use_lsle = 1'b0;
        in_pkt_di = 8'h00;
        in_pkt_wc = 16'h0000;
        in_pkt_is_short = 1'b0;
        in_pkt_is_long = 1'b0;
        in_pkt_start = 1'b0;
        in_pkt_end = 1'b0;
        in_pkt_err = 1'b0;
        in_payload_data = 8'h00;
        in_payload_valid = 1'b0;
        in_payload_first = 1'b0;
        in_payload_last = 1'b0;
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

    task automatic drive_short(input logic [5:0] dt, input logic err);
        @(posedge core_clk);
        in_pkt_di <= {2'b00, dt};
        in_pkt_wc <= 16'h0000;
        in_pkt_is_short <= 1'b1;
        in_pkt_is_long <= 1'b0;
        in_pkt_err <= err;
        in_pkt_start <= 1'b1;
        in_pkt_end <= 1'b1;
        @(posedge core_clk);
        in_pkt_start <= 1'b0;
        in_pkt_end <= 1'b0;
        in_pkt_err <= 1'b0;
        in_pkt_is_short <= 1'b0;
    endtask

    task automatic start_long(input logic [5:0] dt, input logic [15:0] wc);
        @(posedge core_clk);
        in_pkt_di <= {2'b00, dt};
        in_pkt_wc <= wc;
        in_pkt_is_short <= 1'b0;
        in_pkt_is_long <= 1'b1;
        in_pkt_start <= 1'b1;
        @(posedge core_clk);
        in_pkt_start <= 1'b0;
    endtask

    task automatic drive_payload(input logic [7:0] data, input logic first, input logic last);
        @(posedge core_clk);
        in_payload_data <= data;
        in_payload_first <= first;
        in_payload_last <= last;
        in_payload_valid <= 1'b1;
        @(posedge core_clk);
        in_payload_valid <= 1'b0;
        in_payload_first <= 1'b0;
        in_payload_last <= 1'b0;
    endtask

    task automatic end_long(input logic err);
        @(posedge core_clk);
        in_pkt_err <= err;
        in_pkt_end <= 1'b1;
        @(posedge core_clk);
        in_pkt_end <= 1'b0;
        in_pkt_err <= 1'b0;
        in_pkt_is_long <= 1'b0;
    endtask

    task automatic drive_one_byte_line(input logic [7:0] data);
        start_long(6'h2a, 16'd1);
        drive_payload(data, 1'b1, 1'b1);
        end_long(1'b0);
    endtask

    task automatic check_condition(input bit condition, input string message);
        if (!condition) begin
            $fatal(1, "CHECK FAILED: %s", message);
        end
    endtask

    initial begin
        reset_dut();

        clear_logs();
        drive_short(6'h00, 1'b0);
        start_long(6'h2a, 16'd3);
        drive_payload(8'h10, 1'b1, 1'b0);
        drive_payload(8'h11, 1'b0, 1'b0);
        drive_payload(8'h12, 1'b0, 1'b1);
        end_long(1'b0);
        drive_short(6'h01, 1'b0);
        repeat (4) @(posedge core_clk);
        check_condition(sof_count >= 1, "FS produces SOF");
        check_condition(eof_count == 1, "FE produces EOF");
        check_condition(sol_count == 1, "long packet produces SOL");
        check_condition(eol_count == 1, "payload_last produces EOL");
        check_condition(payload_count == 3, "payload pass count");
        check_condition(payload_log[0] == 8'h10, "payload byte 0");
        check_condition(payload_log[2] == 8'h12, "payload byte 2");
        check_condition(line_idx_log[0] == 16'd0, "first line index");
        check_condition(sts_frame_count == 32'd1, "frame count");
        check_condition(sts_line_count == 32'd1, "line count");
        check_condition(sts_last_frame_lines == 16'd1, "last frame lines");
        check_condition(frame_err_count == 0, "clean frame has no error");

        clear_logs();
        start_long(6'h2a, 16'd1);
        drive_payload(8'h22, 1'b1, 1'b1);
        end_long(1'b0);
        repeat (3) @(posedge core_clk);
        check_condition(payload_count == 0, "long packet without FS is dropped");
        check_condition(sts_frame_sync_err_cnt == 16'd1, "FS missing sync error");

        clear_logs();
        drive_short(6'h00, 1'b0);
        start_long(6'h2a, 16'd1);
        drive_payload(8'h33, 1'b1, 1'b1);
        end_long(1'b1);
        drive_short(6'h01, 1'b0);
        repeat (4) @(posedge core_clk);
        check_condition(payload_count == 1, "error frame payload still passes");
        check_condition(frame_err_count == 1, "packet error becomes frame error at FE");

        cfg_use_lsle = 1'b1;
        clear_logs();
        drive_short(6'h00, 1'b0);
        drive_short(6'h02, 1'b0);
        start_long(6'h2a, 16'd1);
        drive_payload(8'h44, 1'b1, 1'b1);
        end_long(1'b0);
        drive_short(6'h03, 1'b0);
        drive_short(6'h01, 1'b0);
        repeat (4) @(posedge core_clk);
        check_condition(sol_count == 1, "LS produces SOL when enabled");
        check_condition(eol_count == 1, "LE produces EOL when enabled");
        check_condition(payload_count == 1, "LSLE payload passes");

        reset_dut();
        clear_logs();
        drive_short(6'h00, 1'b0);
        for (int line_idx = 0; line_idx < 480; line_idx++) begin
            drive_one_byte_line(8'(line_idx));
        end
        drive_short(6'h01, 1'b0);
        repeat (4) @(posedge core_clk);
        check_condition(sts_frame_count == 32'd1, "480-line frame count");
        check_condition(sts_line_count == 32'd480, "480-line line count");
        check_condition(sts_last_frame_lines == 16'd480, "clean frame ended after 480 lines");
        check_condition(payload_count == 480, "480-line payload count");
        check_condition(eol_count == 480, "480-line EOL count");

        reset_dut();
        clear_logs();
        drive_short(6'h00, 1'b0);
        for (int line_idx = 0; line_idx < 392; line_idx++) begin
            drive_one_byte_line(8'(line_idx));
        end
        drive_short(6'h01, 1'b0);
        repeat (4) @(posedge core_clk);
        check_condition(sts_frame_count == 32'd1, "early-FE frame count");
        check_condition(sts_line_count == 32'd392, "early-FE line count");
        check_condition(sts_last_frame_lines == 16'd392, "early FE latches observed line count");

        repeat (10) @(posedge core_clk);
        $display("TEST PASSED: tb_csi2_frame_state");
        $finish;
    end

    initial begin
        #1ms;
        $fatal(1, "Simulation timeout");
    end
endmodule
