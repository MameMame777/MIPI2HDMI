`timescale 1ns / 1ps

module tb_byte_to_core_cdc;
    logic byte_clk;
    logic byte_aresetn;
    logic core_clk;
    logic core_aresetn;
    logic [15:0] s_byte_data;
    logic [1:0] s_byte_keep;
    logic s_byte_valid;
    logic s_byte_sop;
    logic s_byte_eop;
    logic [15:0] m_byte_data;
    logic [1:0] m_byte_keep;
    logic m_byte_valid;
    logic m_byte_sop;
    logic m_byte_eop;
    logic [15:0] sts_lane_fifo_ovf_cnt;

    int out_count;
    int core_cycle;
    logic [15:0] data_log [16];
    logic [1:0] keep_log [16];
    logic sop_log [16];
    logic eop_log [16];
    int cycle_log [16];

    byte_to_core_cdc #(
        .IN_WIDTH(16),
        .KEEP_WIDTH(2),
        .FIFO_DEPTH(8),
        .CORE_OUTPUT_INTERVAL(2)
    ) dut (
        .byte_clk(byte_clk),
        .byte_aresetn(byte_aresetn),
        .core_clk(core_clk),
        .core_aresetn(core_aresetn),
        .s_byte_data(s_byte_data),
        .s_byte_keep(s_byte_keep),
        .s_byte_valid(s_byte_valid),
        .s_byte_sop(s_byte_sop),
        .s_byte_eop(s_byte_eop),
        .m_byte_data(m_byte_data),
        .m_byte_keep(m_byte_keep),
        .m_byte_valid(m_byte_valid),
        .m_byte_sop(m_byte_sop),
        .m_byte_eop(m_byte_eop),
        .sts_lane_fifo_ovf_cnt(sts_lane_fifo_ovf_cnt)
    );

    initial begin
        byte_clk = 1'b0;
        forever #4 byte_clk = ~byte_clk;
    end

    initial begin
        core_clk = 1'b0;
        forever #7 core_clk = ~core_clk;
    end

    always_ff @(posedge core_clk) begin
        if (!core_aresetn) begin
            out_count <= 0;
            core_cycle <= 0;
            for (int idx = 0; idx < 16; idx++) begin
                data_log[idx] <= 16'h0000;
                keep_log[idx] <= 2'b00;
                sop_log[idx] <= 1'b0;
                eop_log[idx] <= 1'b0;
                cycle_log[idx] <= 0;
            end
        end else begin
            core_cycle <= core_cycle + 1;
            if (m_byte_valid) begin
                data_log[out_count] <= m_byte_data;
                keep_log[out_count] <= m_byte_keep;
                sop_log[out_count] <= m_byte_sop;
                eop_log[out_count] <= m_byte_eop;
                cycle_log[out_count] <= core_cycle;
                out_count <= out_count + 1;
            end
        end
    end

    task automatic push(input logic [15:0] data, input logic [1:0] keep, input logic sop, input logic eop);
        @(posedge byte_clk);
        s_byte_data <= data;
        s_byte_keep <= keep;
        s_byte_sop <= sop;
        s_byte_eop <= eop;
        s_byte_valid <= 1'b1;
        @(posedge byte_clk);
        s_byte_valid <= 1'b0;
        s_byte_sop <= 1'b0;
        s_byte_eop <= 1'b0;
        s_byte_keep <= 2'b00;
    endtask

    task automatic wait_outputs(input int count);
        for (int idx = 0; idx < 200; idx++) begin
            @(posedge core_clk);
            if (out_count >= count) begin
                return;
            end
        end
        $fatal(1, "Timed out waiting for CDC output");
    endtask

    task automatic reset_logs();
        @(posedge core_clk);
        out_count <= 0;
        for (int idx = 0; idx < 16; idx++) begin
            data_log[idx] <= 16'h0000;
            keep_log[idx] <= 2'b00;
            sop_log[idx] <= 1'b0;
            eop_log[idx] <= 1'b0;
            cycle_log[idx] <= 0;
        end
    endtask

    task automatic check_condition(input bit condition, input string message);
        #1;
        if (!condition) begin
            $fatal(1, "CHECK FAILED: %s", message);
        end
    endtask

    initial begin
        byte_aresetn = 1'b0;
        core_aresetn = 1'b0;
        s_byte_data = 16'h0000;
        s_byte_keep = 2'b00;
        s_byte_valid = 1'b0;
        s_byte_sop = 1'b0;
        s_byte_eop = 1'b0;
        repeat (6) @(posedge core_clk);
        byte_aresetn = 1'b1;
        core_aresetn = 1'b1;
        repeat (3) @(posedge byte_clk);

        push(16'h1110, 2'b11, 1'b1, 1'b0);
        push(16'h2120, 2'b11, 1'b0, 1'b0);
        push(16'h0030, 2'b01, 1'b0, 1'b1);
        wait_outputs(3);

        check_condition(data_log[0] == 16'h1110, "beat0 data");
        check_condition(data_log[1] == 16'h2120, "beat1 data");
        check_condition(data_log[2] == 16'h0030, "beat2 data");
        check_condition(keep_log[2] == 2'b01, "tail keep");
        check_condition(sop_log[0] == 1'b1, "sop forwarded");
        check_condition(eop_log[2] == 1'b1, "eop forwarded");
        check_condition(cycle_log[1] >= cycle_log[0] + 2, "rate limit gap after beat0");
        check_condition(cycle_log[2] >= cycle_log[1] + 2, "rate limit gap after beat1");
        check_condition(sts_lane_fifo_ovf_cnt == 16'h0000, "no overflow");

        reset_logs();
        push(16'h021e, 2'b11, 1'b1, 1'b0);
        push(16'h1f05, 2'b11, 1'b0, 1'b0);
        push(16'h1000, 2'b11, 1'b0, 1'b0);
        push(16'h2001, 2'b11, 1'b1, 1'b0);
        push(16'h3002, 2'b11, 1'b0, 1'b0);
        push(16'h4003, 2'b11, 1'b0, 1'b1);
        wait_outputs(6);

        check_condition(data_log[0] == 16'h021e, "stress beat0 data preserved");
        check_condition(data_log[1] == 16'h1f05, "stress beat1 data preserved");
        check_condition(data_log[2] == 16'h1000, "stress beat2 data preserved");
        check_condition(data_log[3] == 16'h2001, "stress beat3 data preserved");
        check_condition(data_log[4] == 16'h3002, "stress beat4 data preserved");
        check_condition(data_log[5] == 16'h4003, "stress beat5 data preserved");
        check_condition(sop_log[0] == 1'b1, "stress first SOP preserved");
        check_condition(sop_log[3] == 1'b1, "stress second SOP preserved");
        check_condition(eop_log[5] == 1'b1, "stress final EOP preserved");
        check_condition(sts_lane_fifo_ovf_cnt == 16'h0000, "stress no overflow");

        $display("TEST PASSED: tb_byte_to_core_cdc");
        $finish;
    end

    initial begin
        #1ms;
        $fatal(1, "Simulation timeout");
    end
endmodule
