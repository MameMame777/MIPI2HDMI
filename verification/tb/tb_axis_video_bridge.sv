`timescale 1ns / 1ps

module tb_axis_video_bridge;
    logic core_clk;
    logic core_aresetn;
    logic aclk;
    logic aresetn;

    logic [15:0] in_pixel;
    logic in_pixel_valid;
    logic in_pixel_sof;
    logic in_pixel_eol;
    logic in_pixel_eof;
    logic in_pixel_err;

    logic [15:0] m_axis_tdata;
    logic m_axis_tvalid;
    logic m_axis_tready;
    logic m_axis_tlast;
    logic [1:0] m_axis_tuser;
    logic [15:0] sts_fifo_overflow_cnt;
    logic [15:0] sts_back_pressure_cnt;

    int accepted_count;
    logic [15:0] data_log [16];
    logic last_log [16];
    logic [1:0] user_log [16];
    logic clear_logs_pulse;

    axis_video_bridge #(
        .TDATA_WIDTH(16),
        .TUSER_WIDTH(2),
        .FIFO_DEPTH(16),
        .AXIS_TUSER_ERR_DEBUG(1'b1)
    ) dut (
        .core_clk(core_clk),
        .core_aresetn(core_aresetn),
        .aclk(aclk),
        .aresetn(aresetn),
        .in_pixel(in_pixel),
        .in_pixel_valid(in_pixel_valid),
        .in_pixel_sof(in_pixel_sof),
        .in_pixel_eol(in_pixel_eol),
        .in_pixel_eof(in_pixel_eof),
        .in_pixel_err(in_pixel_err),
        .m_axis_tdata(m_axis_tdata),
        .m_axis_tvalid(m_axis_tvalid),
        .m_axis_tready(m_axis_tready),
        .m_axis_tlast(m_axis_tlast),
        .m_axis_tuser(m_axis_tuser),
        .sts_fifo_overflow_cnt(sts_fifo_overflow_cnt),
        .sts_back_pressure_cnt(sts_back_pressure_cnt)
    );

    initial begin
        core_clk = 1'b0;
        forever #5 core_clk = ~core_clk;
    end

    initial begin
        aclk = 1'b0;
        forever #7 aclk = ~aclk;
    end

    always_ff @(posedge aclk) begin
        if (!aresetn || clear_logs_pulse) begin
            accepted_count <= 0;
            for (int idx = 0; idx < 16; idx++) begin
                data_log[idx] <= 16'h0000;
                last_log[idx] <= 1'b0;
                user_log[idx] <= 2'b00;
            end
        end else if (m_axis_tvalid && m_axis_tready) begin
            data_log[accepted_count] <= m_axis_tdata;
            last_log[accepted_count] <= m_axis_tlast;
            user_log[accepted_count] <= m_axis_tuser;
            accepted_count <= accepted_count + 1;
        end
    end

    task automatic reset_dut();
        core_aresetn = 1'b0;
        aresetn = 1'b0;
        clear_logs_pulse = 1'b0;
        in_pixel = 16'h0000;
        in_pixel_valid = 1'b0;
        in_pixel_sof = 1'b0;
        in_pixel_eol = 1'b0;
        in_pixel_eof = 1'b0;
        in_pixel_err = 1'b0;
        m_axis_tready = 1'b1;
        repeat (8) @(posedge core_clk);
        core_aresetn = 1'b1;
        repeat (8) @(posedge aclk);
        aresetn = 1'b1;
        repeat (4) @(posedge aclk);
    endtask

    task automatic clear_logs();
        @(posedge aclk);
        clear_logs_pulse <= 1'b1;
        @(posedge aclk);
        clear_logs_pulse <= 1'b0;
        @(posedge aclk);
    endtask

    task automatic drive_pixel(
        input logic [15:0] pixel,
        input logic sof,
        input logic eol,
        input logic eof,
        input logic err
    );
        @(posedge core_clk);
        in_pixel <= pixel;
        in_pixel_sof <= sof;
        in_pixel_eol <= eol;
        in_pixel_eof <= eof;
        in_pixel_err <= err;
        in_pixel_valid <= 1'b1;
        @(posedge core_clk);
        in_pixel_valid <= 1'b0;
        in_pixel_sof <= 1'b0;
        in_pixel_eol <= 1'b0;
        in_pixel_eof <= 1'b0;
        in_pixel_err <= 1'b0;
    endtask

    task automatic wait_accepted(input int count);
        for (int cycle = 0; cycle < 300; cycle++) begin
            @(posedge aclk);
            if (accepted_count >= count) begin
                return;
            end
        end
        $fatal(1, "Timed out waiting for accepted AXIS data");
    endtask

    task automatic check_condition(input bit condition, input string message);
        if (!condition) begin
            $fatal(1, "CHECK FAILED: %s", message);
        end
    endtask

    initial begin
        reset_dut();

        clear_logs();
        drive_pixel(16'h0010, 1'b1, 1'b0, 1'b0, 1'b0);
        drive_pixel(16'h0011, 1'b0, 1'b0, 1'b0, 1'b0);
        drive_pixel(16'h0012, 1'b0, 1'b1, 1'b1, 1'b1);
        wait_accepted(3);
        check_condition(data_log[0] == 16'h0010, "pixel 0 order");
        check_condition(data_log[1] == 16'h0011, "pixel 1 order");
        check_condition(data_log[2] == 16'h0012, "pixel 2 order");
        check_condition(user_log[0][0] == 1'b1, "SOF maps to tuser[0]");
        check_condition(last_log[2] == 1'b1, "EOL maps to tlast");
        check_condition(user_log[2][1] == 1'b1, "err maps to tuser[1]");

        clear_logs();
        m_axis_tready = 1'b0;
        drive_pixel(16'h0020, 1'b1, 1'b0, 1'b0, 1'b0);
        drive_pixel(16'h0021, 1'b0, 1'b1, 1'b0, 1'b0);
        repeat (8) @(posedge aclk);
        check_condition(accepted_count == 0, "back-pressure holds data");
        check_condition(sts_back_pressure_cnt != 16'h0000, "back-pressure counter increments");
        m_axis_tready = 1'b1;
        wait_accepted(2);
        check_condition(data_log[0] == 16'h0020, "held pixel 0 order");
        check_condition(data_log[1] == 16'h0021, "held pixel 1 order");
        check_condition(last_log[1] == 1'b1, "held tlast order");

        repeat (10) @(posedge aclk);
        $display("TEST PASSED: tb_axis_video_bridge");
        $finish;
    end

    initial begin
        #2ms;
        $fatal(1, "Simulation timeout");
    end
endmodule
