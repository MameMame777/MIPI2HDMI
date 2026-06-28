`timescale 1ns / 1ps

module tb_csi2_frame_state_guarded;
    localparam int EXPECTED_LINES = 480;
    localparam logic [15:0] GOOD_WC = 16'd1;
    localparam logic [15:0] BAD_WC = 16'd2;

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
    logic clear_logs_pulse;

    csi2_frame_state #(
        .MAX_LINES(512),
        .GUARD_FRAME_LINES(1'b1),
        .EXPECTED_FRAME_LINES(EXPECTED_LINES),
        .EXPECTED_LINE_WC(GOOD_WC)
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

    task automatic start_long(input logic [15:0] wc);
        @(posedge core_clk);
        in_pkt_di <= 8'h1e;
        in_pkt_wc <= wc;
        in_pkt_is_short <= 1'b0;
        in_pkt_is_long <= 1'b1;
        in_pkt_start <= 1'b1;
        @(posedge core_clk);
        in_pkt_start <= 1'b0;
    endtask

    task automatic drive_payload(input logic [7:0] data);
        @(posedge core_clk);
        in_payload_data <= data;
        in_payload_first <= 1'b1;
        in_payload_last <= 1'b1;
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

    task automatic drive_one_byte_line(input logic [15:0] wc, input logic [7:0] data);
        start_long(wc);
        drive_payload(data);
        end_long(1'b0);
    endtask

    task automatic drive_good_lines(input int line_count, input int start_line);
        for (int line_index = 0; line_index < line_count; line_index++) begin
            drive_one_byte_line(GOOD_WC, 8'(start_line + line_index));
        end
    endtask

    task automatic check_condition(input bit condition, input string message);
        if (!condition) begin
            $fatal(1, "CHECK FAILED: %s", message);
        end
    endtask

    task automatic check_completed_frame(input int expected_sync_errors, input string case_name);
        repeat (6) @(posedge core_clk);
        check_condition(sts_frame_count == 32'd1, $sformatf("%s frame count", case_name));
        check_condition(sts_line_count == 32'(EXPECTED_LINES), $sformatf("%s line count", case_name));
        check_condition(sts_last_frame_lines == 16'(EXPECTED_LINES), $sformatf("%s last frame lines", case_name));
        check_condition(payload_count == EXPECTED_LINES, $sformatf("%s payload count", case_name));
        check_condition(eol_count == EXPECTED_LINES, $sformatf("%s EOL count", case_name));
        check_condition(eof_count == 1, $sformatf("%s EOF count", case_name));
        check_condition(frame_err_count == 0, $sformatf("%s frame error count", case_name));
        check_condition(sts_frame_sync_err_cnt == 16'(expected_sync_errors), $sformatf("%s sync error count", case_name));
    endtask

    initial begin
        reset_dut();
        clear_logs();
        drive_short(6'h00, 1'b0);
        drive_good_lines(EXPECTED_LINES, 0);
        drive_short(6'h01, 1'b0);
        check_completed_frame(0, "clean 480-line frame");

        reset_dut();
        clear_logs();
        drive_short(6'h00, 1'b0);
        drive_good_lines(392, 0);
        drive_short(6'h01, 1'b0);
        repeat (4) @(posedge core_clk);
        check_condition(sts_frame_count == 32'd0, "early FE does not end frame");
        drive_good_lines(88, 392);
        drive_short(6'h01, 1'b0);
        check_completed_frame(1, "early FE ignored frame");

        reset_dut();
        clear_logs();
        drive_short(6'h00, 1'b0);
        drive_good_lines(100, 0);
        drive_short(6'h00, 1'b0);
        drive_good_lines(380, 100);
        drive_short(6'h01, 1'b0);
        check_completed_frame(1, "overlap FS ignored frame");

        reset_dut();
        clear_logs();
        drive_short(6'h01, 1'b0);
        drive_short(6'h00, 1'b0);
        drive_good_lines(EXPECTED_LINES, 0);
        drive_short(6'h01, 1'b0);
        check_completed_frame(1, "FE without FS ignored frame");

        reset_dut();
        clear_logs();
        drive_short(6'h00, 1'b0);
        drive_one_byte_line(BAD_WC, 8'hAA);
        drive_good_lines(EXPECTED_LINES, 0);
        drive_short(6'h01, 1'b0);
        check_completed_frame(1, "bad-WC line dropped frame");

        repeat (10) @(posedge core_clk);
        $display("TEST PASSED: tb_csi2_frame_state_guarded");
        $finish;
    end

    initial begin
        #2ms;
        $fatal(1, "Simulation timeout");
    end
endmodule