`timescale 1ns / 1ps
`default_nettype none

module tb_ov5640_sccb_init_probe_runtime;

    localparam int CLK_HZ = 1_000_000;
    localparam int I2C_HZ = 100_000;

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic rt_test_pattern_valid = 1'b0;
    logic rt_test_pattern_enable = 1'b0;
    logic rt_test_pattern_ready;
    logic rt_test_pattern_done;
    logic rt_test_pattern_error;
    logic [7:0] rt_test_pattern_value;
    logic [7:0] rt_ack_error_count;
    logic rt_reg_write_valid = 1'b0;
    logic [15:0] rt_reg_write_addr = 16'h0000;
    logic [7:0] rt_reg_write_value = 8'h00;
    logic rt_reg_write_ready;
    logic rt_reg_write_done;
    logic rt_reg_write_error;
    logic rt_reg_write_busy;
    logic [7:0] rt_reg_write_ack_err_count;
    logic [15:0] rt_reg_write_last_addr;
    logic sccb_busy;
    logic sccb_done;
    logic sccb_error;
    logic [7:0] sccb_step_index;

    tri1 cam_scl;
    tri1 cam_sda;
    logic slave_sda_drive_low = 1'b0;

    logic slave_in_transfer = 1'b0;
    logic slave_pending_ack = 1'b0;
    logic slave_ack_phase = 1'b0;
    int unsigned slave_bit_count = 0;

    logic monitor_in_transfer = 1'b0;
    logic monitor_skip_ack = 1'b0;
    int unsigned monitor_bit_count = 0;
    int unsigned monitor_byte_count = 0;
    logic [7:0] monitor_byte = 8'h00;
    logic [7:0] monitor_bytes [0:7];
    int unsigned test_pattern_write_count = 0;
    logic [6:0] aec_reference_write_seen = 7'h00;
    logic [7:0] last_test_pattern_value = 8'h00;
    int unsigned arbitrary_write_count = 0;
    logic [15:0] last_arbitrary_addr = 16'h0000;
    logic [7:0] last_arbitrary_value = 8'h00;
    logic monitor_runtime_active_latched = 1'b0;
    logic monitor_runtime_kind_latched = 1'b0;

    assign cam_sda = slave_sda_drive_low ? 1'b0 : 1'bz;

    always #500 clk = ~clk;

    always @(posedge clk) begin
        if (rst_n) begin
            case ({dut.reg_addr, dut.reg_value})
                24'h3503_00: aec_reference_write_seen[0] <= 1'b1;
                24'h3a00_78: aec_reference_write_seen[1] <= 1'b1;
                24'h3a01_01: aec_reference_write_seen[2] <= 1'b1;
                24'h3a13_43: aec_reference_write_seen[3] <= 1'b1;
                24'h3a18_00: aec_reference_write_seen[4] <= 1'b1;
                24'h3a19_f8: aec_reference_write_seen[5] <= 1'b1;
                24'h3a1a_04: aec_reference_write_seen[6] <= 1'b1;
                default: begin
                end
            endcase
        end
    end

    ov5640_sccb_init_probe #(
        .CLK_HZ(CLK_HZ),
        .I2C_HZ(I2C_HZ),
        .POWERUP_DELAY_MS(1),
        .TEST_PATTERN_ENABLE(1'b0)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .rt_test_pattern_valid(rt_test_pattern_valid),
        .rt_test_pattern_enable(rt_test_pattern_enable),
        .rt_test_pattern_ready(rt_test_pattern_ready),
        .rt_test_pattern_done(rt_test_pattern_done),
        .rt_test_pattern_error(rt_test_pattern_error),
        .rt_test_pattern_value(rt_test_pattern_value),
        .rt_ack_error_count(rt_ack_error_count),
        .rt_reg_write_valid(rt_reg_write_valid),
        .rt_reg_write_addr(rt_reg_write_addr),
        .rt_reg_write_value(rt_reg_write_value),
        .rt_reg_write_ready(rt_reg_write_ready),
        .rt_reg_write_done(rt_reg_write_done),
        .rt_reg_write_error(rt_reg_write_error),
        .rt_reg_write_busy(rt_reg_write_busy),
        .rt_reg_write_ack_err_count(rt_reg_write_ack_err_count),
        .rt_reg_write_last_addr(rt_reg_write_last_addr),
        .cam_scl(cam_scl),
        .cam_sda(cam_sda),
        .scl_drive_low_o(),
        .sda_drive_low_o(),
        .busy(sccb_busy),
        .done(sccb_done),
        .error(sccb_error),
        .chip_id_high(),
        .chip_id_low(),
        .ack_error_count(),
        .step_index(sccb_step_index),
        .rd_mipi_ctrl_300e(),
        .rd_mipi_ctrl_4800(),
        .rd_mipi_ctrl_4805(),
        .rd_mipi_ctrl_4837(),
        .rd_format_ctrl_4300(),
        .rd_isp_format_501f(),
        .rd_isp_ctrl_5000(),
        .rd_isp_ctrl_5001(),
        .rd_timing_ctrl_3824(),
        .rd_jpeg_ctrl_4407(),
        .rd_mipi_ctrl_440e(),
        .rd_vfifo_ctrl_460b(),
        .rd_vfifo_ctrl_460c(),
        .rd_awb_5189(),
        .rd_output_width_high_3808(),
        .rd_output_width_low_3809(),
        .rd_output_height_high_380a(),
        .rd_output_height_low_380b(),
        .rd_aec_manual_3503(),
        .rd_aec_ctrl_3a13(),
        .rd_aec_gain_ceiling_high_3a18(),
        .rd_aec_gain_ceiling_low_3a19(),
        .dbg_scl_in(),
        .dbg_sda_in(),
        .dbg_ack_low_seen(),
        .dbg_scl_low_seen(),
        .dbg_scl_high_seen(),
        .dbg_sda_low_seen(),
        .dbg_sda_high_seen()
    );

    initial begin
        #3_000_000_000;
        $display(
            "Timeout status: busy=%0b done=%0b error=%0b step=%0d rt_ready=%0b writes=%0d last=0x%02x",
            sccb_busy,
            sccb_done,
            sccb_error,
            sccb_step_index,
            rt_test_pattern_ready,
            test_pattern_write_count,
            last_test_pattern_value
        );
        $fatal(1, "TEST FAILED: timeout");
    end

    initial begin
        repeat (10) @(posedge clk);
        rst_n = 1'b1;

        wait (rt_test_pattern_ready);

        if (sccb_step_index !== 8'd255) begin
            $fatal(1, "TEST FAILED: init ended at step %0d, expected 255", sccb_step_index);
        end
        if (aec_reference_write_seen !== 7'h7f) begin
            $fatal(1, "TEST FAILED: missing AEC reference writes mask=0x%02x", aec_reference_write_seen);
        end

        issue_test_pattern_request(1'b1, 8'h80);
        issue_test_pattern_request(1'b0, 8'h00);

        issue_arbitrary_reg_write(16'h380c, 8'h07);
        issue_arbitrary_reg_write(16'h380d, 8'h68);
        issue_arbitrary_reg_write(16'h380e, 8'h03);

        if (arbitrary_write_count !== 3) begin
            $fatal(
                1,
                "TEST FAILED: arbitrary write count %0d, expected 3",
                arbitrary_write_count
            );
        end
        if (last_arbitrary_addr !== 16'h380e || last_arbitrary_value !== 8'h03) begin
            $fatal(
                1,
                "TEST FAILED: last arbitrary write 0x%04x=0x%02x, expected 0x380e=0x03",
                last_arbitrary_addr,
                last_arbitrary_value
            );
        end
        if (rt_reg_write_ack_err_count !== 8'h00) begin
            $fatal(
                1,
                "TEST FAILED: arbitrary write ACK error count %0d",
                rt_reg_write_ack_err_count
            );
        end
        if (rt_ack_error_count !== 8'h00) begin
            $fatal(
                1,
                "TEST FAILED: test-pattern ACK error count %0d after mixed sequence",
                rt_ack_error_count
            );
        end

        issue_test_pattern_request(1'b1, 8'h80);
        if (test_pattern_write_count !== 3) begin
            $fatal(
                1,
                "TEST FAILED: test-pattern write count %0d after arbitrary writes, expected 3",
                test_pattern_write_count
            );
        end

        repeat (20) @(posedge clk);
        $display("TEST PASSED: tb_ov5640_sccb_init_probe_runtime");
        $finish;
    end

    task automatic issue_test_pattern_request(
        input logic enable,
        input logic [7:0] expected_value
    );
        wait (rt_test_pattern_ready);
        rt_test_pattern_enable <= enable;
        rt_test_pattern_valid <= 1'b1;
        @(posedge clk);
        rt_test_pattern_valid <= 1'b0;
        @(posedge clk);
        wait (!rt_test_pattern_done);
        wait (rt_test_pattern_done);
        if (rt_test_pattern_error) begin
            $fatal(1, "TEST FAILED: runtime SCCB error for 0x503d=0x%02x", expected_value);
        end
        if (rt_ack_error_count !== 8'h00) begin
            $fatal(1, "TEST FAILED: runtime ACK error count is %0d", rt_ack_error_count);
        end
        if (rt_test_pattern_value !== expected_value) begin
            $fatal(1, "TEST FAILED: status value 0x%02x, expected 0x%02x", rt_test_pattern_value, expected_value);
        end
        if ((dut.reg_addr !== 16'h503d) || (dut.reg_value !== expected_value)) begin
            $fatal(
                1,
                "TEST FAILED: final SCCB command addr=0x%04x value=0x%02x, expected 0x503d=0x%02x",
                dut.reg_addr,
                dut.reg_value,
                expected_value
            );
        end
    endtask

    task automatic issue_arbitrary_reg_write(
        input logic [15:0] addr,
        input logic [7:0] value
    );
        int unsigned before_count;
        before_count = arbitrary_write_count;
        wait (rt_reg_write_ready);
        rt_reg_write_addr <= addr;
        rt_reg_write_value <= value;
        rt_reg_write_valid <= 1'b1;
        @(posedge clk);
        rt_reg_write_valid <= 1'b0;
        @(posedge clk);
        wait (!rt_reg_write_done);
        wait (rt_reg_write_done);
        repeat (4) @(posedge clk);
        if (rt_reg_write_error) begin
            $fatal(1, "TEST FAILED: arbitrary write error for 0x%04x=0x%02x", addr, value);
        end
        if (rt_reg_write_last_addr !== addr) begin
            $fatal(
                1,
                "TEST FAILED: rt_reg_write_last_addr 0x%04x, expected 0x%04x",
                rt_reg_write_last_addr,
                addr
            );
        end
        if (arbitrary_write_count !== before_count + 1) begin
            $fatal(
                1,
                "TEST FAILED: monitor saw %0d arbitrary writes, expected %0d",
                arbitrary_write_count - before_count,
                1
            );
        end
        if (last_arbitrary_addr !== addr || last_arbitrary_value !== value) begin
            $fatal(
                1,
                "TEST FAILED: monitor last 0x%04x=0x%02x, expected 0x%04x=0x%02x",
                last_arbitrary_addr,
                last_arbitrary_value,
                addr,
                value
            );
        end
    endtask

    always @(negedge cam_sda) begin
        if (cam_scl) begin
            slave_in_transfer = 1'b1;
            slave_pending_ack = 1'b0;
            slave_ack_phase = 1'b0;
            slave_bit_count = 0;
            slave_sda_drive_low = 1'b0;

            monitor_in_transfer = 1'b1;
            monitor_skip_ack = 1'b0;
            monitor_bit_count = 0;
            monitor_byte_count = 0;
            monitor_byte = 8'h00;
            monitor_runtime_active_latched = dut.runtime_active;
            monitor_runtime_kind_latched = dut.runtime_kind;
        end
    end

    always @(posedge cam_sda) begin
        #1;
        if (cam_scl) begin
            if (monitor_in_transfer) begin
                process_monitor_transaction();
            end
            slave_in_transfer = 1'b0;
            slave_pending_ack = 1'b0;
            slave_ack_phase = 1'b0;
            slave_sda_drive_low = 1'b0;
            monitor_in_transfer = 1'b0;
        end
    end

    always @(posedge cam_scl) begin
        if (slave_in_transfer && !slave_ack_phase) begin
            if (slave_bit_count == 7) begin
                slave_pending_ack = 1'b1;
                slave_bit_count = 0;
            end else begin
                slave_bit_count = slave_bit_count + 1;
            end
        end

        if (monitor_in_transfer) begin
            if (monitor_skip_ack) begin
                monitor_skip_ack = 1'b0;
            end else begin
                monitor_byte = {monitor_byte[6:0], cam_sda};
                if (monitor_bit_count == 7) begin
                    if (monitor_byte_count < 8) begin
                        monitor_bytes[monitor_byte_count] = monitor_byte;
                    end
                    monitor_byte_count = monitor_byte_count + 1;
                    monitor_bit_count = 0;
                    monitor_skip_ack = 1'b1;
                end else begin
                    monitor_bit_count = monitor_bit_count + 1;
                end
            end
        end
    end

    always @(negedge cam_scl) begin
        if (slave_in_transfer) begin
            if (slave_ack_phase) begin
                slave_sda_drive_low = 1'b0;
                slave_ack_phase = 1'b0;
            end else if (slave_pending_ack) begin
                slave_sda_drive_low = 1'b1;
                slave_pending_ack = 1'b0;
                slave_ack_phase = 1'b1;
            end
        end
    end

    task automatic process_monitor_transaction();
        logic [15:0] monitor_addr;
        logic [7:0] monitor_value;

        monitor_addr = {monitor_bytes[1], monitor_bytes[2]};
        monitor_value = monitor_bytes[3];

        if ((monitor_byte_count >= 4) &&
            (monitor_bytes[0] == 8'h78) &&
            (monitor_addr == 16'h503d)) begin
            test_pattern_write_count = test_pattern_write_count + 1;
            last_test_pattern_value = monitor_value;
            $display(
                "Observed OV5640 test-pattern write %0d: 0x503d <= 0x%02x",
                test_pattern_write_count,
                last_test_pattern_value
            );
        end

        if ((monitor_byte_count >= 4) &&
            (monitor_bytes[0] == 8'h78) &&
            monitor_runtime_active_latched &&
            (monitor_runtime_kind_latched == 1'b1)) begin
            arbitrary_write_count = arbitrary_write_count + 1;
            last_arbitrary_addr = monitor_addr;
            last_arbitrary_value = monitor_value;
            $display(
                "Observed arbitrary SCCB write %0d: 0x%04x <= 0x%02x",
                arbitrary_write_count,
                last_arbitrary_addr,
                last_arbitrary_value
            );
        end
    endtask

endmodule

`default_nettype wire