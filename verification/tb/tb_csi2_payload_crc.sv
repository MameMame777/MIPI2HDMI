`timescale 1ns / 1ps

module tb_csi2_payload_crc;
    logic core_clk;
    logic core_aresetn;
    logic [7:0] payload_data;
    logic payload_valid;
    logic payload_first;
    logic payload_last;
    logic [15:0] footer_data;
    logic footer_valid;
    logic crc_check_valid;
    logic crc_match;
    logic [15:0] crc_calc;
    logic [15:0] crc_received;
    logic [15:0] sts_crc_err_cnt;
    logic [15:0] sts_crc_ok_cnt;

    csi2_payload_crc dut (
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
        .sts_crc_err_cnt(sts_crc_err_cnt),
        .sts_crc_ok_cnt(sts_crc_ok_cnt)
    );

    initial begin
        core_clk = 1'b0;
        forever #5 core_clk = ~core_clk;
    end

    function automatic [15:0] ref_crc_update(input logic [15:0] crc_in, input logic [7:0] data);
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
        ref_crc_update = crc_next;
    endfunction

    function automatic [15:0] ref_crc3(input logic [7:0] b0, input logic [7:0] b1, input logic [7:0] b2);
        automatic logic [15:0] crc;
        crc = 16'hffff;
        crc = ref_crc_update(crc, b0);
        crc = ref_crc_update(crc, b1);
        crc = ref_crc_update(crc, b2);
        ref_crc3 = crc;
    endfunction

    task automatic reset_dut();
        core_aresetn = 1'b0;
        payload_data = 8'h00;
        payload_valid = 1'b0;
        payload_first = 1'b0;
        payload_last = 1'b0;
        footer_data = 16'h0000;
        footer_valid = 1'b0;
        repeat (8) @(posedge core_clk);
        core_aresetn = 1'b1;
        repeat (2) @(posedge core_clk);
    endtask

    task automatic drive_payload(input logic [7:0] data, input logic first, input logic last);
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

    task automatic drive_footer(input logic [15:0] crc_value);
        @(posedge core_clk);
        footer_data <= crc_value;
        footer_valid <= 1'b1;
        @(posedge core_clk);
        footer_valid <= 1'b0;
    endtask

    task automatic wait_check();
        for (int cycle = 0; cycle < 50; cycle++) begin
            @(posedge core_clk);
            if (crc_check_valid) begin
                return;
            end
        end
        $fatal(1, "Timed out waiting for CRC check");
    endtask

    task automatic check_condition(input bit condition, input string message);
        if (!condition) begin
            $fatal(1, "CHECK FAILED: %s", message);
        end
    endtask

    initial begin
        automatic logic [15:0] expected_crc;
        reset_dut();

        expected_crc = ref_crc3(8'haa, 8'hbb, 8'hcc);
        drive_payload(8'haa, 1'b1, 1'b0);
        drive_payload(8'hbb, 1'b0, 1'b0);
        drive_payload(8'hcc, 1'b0, 1'b1);
        drive_footer(expected_crc);
        wait_check();
        check_condition(crc_match == 1'b1, "matching CRC accepted");
        check_condition(crc_calc == expected_crc, "calculated CRC matches reference");
        check_condition(crc_received == expected_crc, "received CRC latched");
        check_condition(sts_crc_ok_cnt == 16'd1, "CRC OK count");
        check_condition(sts_crc_err_cnt == 16'd0, "CRC error count remains zero");

        expected_crc = ref_crc3(8'h01, 8'h02, 8'h03);
        drive_payload(8'h01, 1'b1, 1'b0);
        drive_payload(8'h02, 1'b0, 1'b0);
        drive_payload(8'h03, 1'b0, 1'b1);
        drive_footer(expected_crc ^ 16'h0001);
        wait_check();
        check_condition(crc_match == 1'b0, "mismatching CRC rejected");
        check_condition(sts_crc_ok_cnt == 16'd1, "CRC OK count holds");
        check_condition(sts_crc_err_cnt == 16'd1, "CRC error count increments");

        expected_crc = ref_crc_update(16'hffff, 8'h5a);
        drive_payload(8'h5a, 1'b1, 1'b1);
        drive_footer(expected_crc);
        wait_check();
        check_condition(crc_match == 1'b1, "single byte packet accepted");
        check_condition(sts_crc_ok_cnt == 16'd2, "second CRC OK count");

        repeat (10) @(posedge core_clk);
        $display("TEST PASSED: tb_csi2_payload_crc");
        $finish;
    end

    initial begin
        #1ms;
        $fatal(1, "Simulation timeout");
    end
endmodule
