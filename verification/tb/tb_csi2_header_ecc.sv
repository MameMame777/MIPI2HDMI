`timescale 1ns / 1ps

module tb_csi2_header_ecc;
    logic core_clk;
    logic core_aresetn;
    logic hdr_valid;
    logic [31:0] hdr_raw;
    logic hdr_corr_valid;
    logic [23:0] hdr_corr;
    logic [7:0] hdr_di;
    logic [15:0] hdr_wc;
    logic hdr_ecc_corrected;
    logic hdr_ecc_uncorrectable;
    logic hdr_ecc_no_error;
    logic [15:0] sts_ecc_corr_cnt;
    logic [15:0] sts_ecc_uncorr_cnt;

    csi2_header_ecc dut (
        .core_clk(core_clk),
        .core_aresetn(core_aresetn),
        .hdr_valid(hdr_valid),
        .hdr_raw(hdr_raw),
        .hdr_corr_valid(hdr_corr_valid),
        .hdr_corr(hdr_corr),
        .hdr_di(hdr_di),
        .hdr_wc(hdr_wc),
        .hdr_ecc_corrected(hdr_ecc_corrected),
        .hdr_ecc_uncorrectable(hdr_ecc_uncorrectable),
        .hdr_ecc_no_error(hdr_ecc_no_error),
        .sts_ecc_corr_cnt(sts_ecc_corr_cnt),
        .sts_ecc_uncorr_cnt(sts_ecc_uncorr_cnt)
    );

    initial begin
        core_clk = 1'b0;
        forever #5 core_clk = ~core_clk;
    end

    function automatic [5:0] ref_ecc6(input logic [23:0] data);
        ref_ecc6[0] = data[0]^data[1]^data[2]^data[4]^data[5]^data[7]^data[10]^data[11]^data[13]^data[16]^data[20]^data[21]^data[22]^data[23];
        ref_ecc6[1] = data[0]^data[1]^data[3]^data[4]^data[6]^data[8]^data[10]^data[12]^data[14]^data[17]^data[20]^data[21]^data[22]^data[23];
        ref_ecc6[2] = data[0]^data[2]^data[3]^data[5]^data[6]^data[9]^data[11]^data[12]^data[15]^data[18]^data[20]^data[21]^data[22];
        ref_ecc6[3] = data[1]^data[2]^data[3]^data[7]^data[8]^data[9]^data[13]^data[14]^data[15]^data[19]^data[20]^data[21]^data[23];
        ref_ecc6[4] = data[4]^data[5]^data[6]^data[7]^data[8]^data[9]^data[16]^data[17]^data[18]^data[19]^data[20]^data[22]^data[23];
        ref_ecc6[5] = data[10]^data[11]^data[12]^data[13]^data[14]^data[15]^data[16]^data[17]^data[18]^data[19]^data[21]^data[22]^data[23];
    endfunction

    function automatic [31:0] make_header(input logic [23:0] data);
        make_header = {2'b00, ref_ecc6(data), data};
    endfunction

    task automatic reset_dut();
        core_aresetn = 1'b0;
        hdr_valid = 1'b0;
        hdr_raw = 32'h0000_0000;
        repeat (8) @(posedge core_clk);
        core_aresetn = 1'b1;
        repeat (2) @(posedge core_clk);
    endtask

    task automatic drive_header(input logic [31:0] raw);
        @(posedge core_clk);
        hdr_raw <= raw;
        hdr_valid <= 1'b1;
        @(posedge core_clk);
        hdr_valid <= 1'b0;
    endtask

    task automatic wait_corr();
        for (int cycle = 0; cycle < 20; cycle++) begin
            @(posedge core_clk);
            if (hdr_corr_valid) begin
                return;
            end
        end
        $fatal(1, "Timed out waiting for ECC correction");
    endtask

    task automatic check_condition(input bit condition, input string message);
        if (!condition) begin
            $fatal(1, "CHECK FAILED: %s", message);
        end
    endtask

    initial begin
        automatic logic [23:0] data;
        automatic logic [31:0] header;
        automatic logic [31:0] corrupt;
        reset_dut();

        data = 24'h12342a;
        header = make_header(data);
        drive_header(header);
        wait_corr();
        check_condition(hdr_corr == data, "no-error data passes");
        check_condition(hdr_di == 8'h2a, "DI decode");
        check_condition(hdr_wc == 16'h1234, "WC decode");
        check_condition(hdr_ecc_no_error == 1'b1, "no-error pulse");
        check_condition(hdr_ecc_uncorrectable == 1'b0, "no uncorrectable on clean header");

        for (int bit_idx = 0; bit_idx < 24; bit_idx++) begin
            data = 24'h03aa2b;
            header = make_header(data);
            corrupt = header ^ (32'h1 << bit_idx);
            drive_header(corrupt);
            wait_corr();
            check_condition(hdr_corr == data, "single data bit correction");
            check_condition(hdr_ecc_corrected == 1'b1, "single data bit corrected pulse");
            check_condition(hdr_ecc_uncorrectable == 1'b0, "single data bit not uncorrectable");
        end

        data = 24'h000100;
        header = make_header(data);
        corrupt = header ^ (32'h1 << 24);
        drive_header(corrupt);
        wait_corr();
        check_condition(hdr_corr == data, "ECC bit flip leaves data unchanged");
        check_condition(hdr_ecc_corrected == 1'b1, "ECC bit flip counted corrected");

        data = 24'h00022a;
        header = make_header(data);
        corrupt = header ^ (32'h3 << 24);
        drive_header(corrupt);
        wait_corr();
        check_condition(hdr_corr == data, "multi ECC bit flip leaves data unchanged");
        check_condition(hdr_ecc_uncorrectable == 1'b1, "multi ECC bit flip uncorrectable");

        repeat (10) @(posedge core_clk);
        $display("TEST PASSED: tb_csi2_header_ecc");
        $finish;
    end

    initial begin
        #1ms;
        $fatal(1, "Simulation timeout");
    end
endmodule
