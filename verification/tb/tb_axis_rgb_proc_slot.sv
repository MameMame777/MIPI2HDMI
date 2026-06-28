`timescale 1ns / 1ps
`default_nettype none

// DSim testbench for axis_rgb_proc_slot (point-op slot, 2026-06-25).
// Covers the new runtime threshold port cfg_thresh_level (op 4 = binarize on green):
//  (1) default level 128 stays bit-identical to the old hard-coded `y > 8'd128`;
//  (2) the level is runtime-effective (a low/high level flips the same pixel);
//  (3) the new port is inert on all other ops (passthrough/invert/grayscale unchanged).
// The slot is a 1-cycle registered point op (no line buffers); op_pixel is combinational
// on the current beat, latched on the posedge -> sample one posedge after driving.
module tb_axis_rgb_proc_slot;
    logic clk = 0, rst_n = 0;
    logic [2:0]  cfg_op = 0;
    logic [7:0]  cfg_thresh_level = 8'd128;
    logic [23:0] in_pixel = 0;
    logic in_valid = 0, in_sof = 0, in_eol = 0, in_eof = 0, in_err = 0;
    logic [23:0] out_pixel;
    logic out_valid, out_sof, out_eol, out_eof, out_err;

    always #5 clk = ~clk;          // 100 MHz

    axis_rgb_proc_slot #(.ENABLE(1'b1)) dut (
        .clk(clk), .rst_n(rst_n),
        .cfg_op(cfg_op), .cfg_thresh_level(cfg_thresh_level),
        .in_pixel(in_pixel), .in_valid(in_valid), .in_sof(in_sof),
        .in_eol(in_eol), .in_eof(in_eof), .in_err(in_err),
        .out_pixel(out_pixel), .out_valid(out_valid), .out_sof(out_sof),
        .out_eol(out_eol), .out_eof(out_eof), .out_err(out_err)
    );

    integer errors = 0;

    // Drive cfg + pixel on the negedge so they are stable across the posedge that latches
    // op_pixel into out_pixel, then sample after the NBA update.
    task automatic check_op(input string nm, input [2:0] op, input [7:0] thr,
                            input [23:0] pix, input [23:0] exp);
        @(negedge clk);
        cfg_op = op; cfg_thresh_level = thr; in_pixel = pix; in_valid = 1'b1;
        @(posedge clk); #1;
        if (out_pixel !== exp) begin
            $display("  FAIL %s: op=%0d thr=%0d pix=%06h got %06h exp %06h",
                     nm, op, thr, pix, out_pixel, exp); errors++;
        end else begin
            $display("  ok   %s: op=%0d thr=%0d pix=%06h -> %06h", nm, op, thr, pix, out_pixel);
        end
    endtask

    initial begin
        rst_n = 0; repeat (4) @(negedge clk); rst_n = 1; repeat (2) @(negedge clk);

        // --- 1) threshold op 4 at default level 128 = old hard-coded `y(green) > 128` ---
        // green is in_pixel[15:8]; out = (g > thr) ? white : black.
        check_op("thr128-g127-black", 3'd4, 8'd128, 24'h00_7F_00, 24'h000000); // 127 > 128 = 0
        check_op("thr128-g128-black", 3'd4, 8'd128, 24'h00_80_00, 24'h000000); // 128 > 128 = 0 (boundary)
        check_op("thr128-g129-white", 3'd4, 8'd128, 24'h00_81_00, 24'hFFFFFF); // 129 > 128 = 1
        check_op("thr128-g200-white", 3'd4, 8'd128, 24'h00_C8_00, 24'hFFFFFF);

        // --- 2) threshold level is runtime-effective: same pixel g=100 flips with the level ---
        check_op("thr50-g100-white",  3'd4, 8'd50,  24'h00_64_00, 24'hFFFFFF); // 100 > 50  = 1
        check_op("thr200-g100-black", 3'd4, 8'd200, 24'h00_64_00, 24'h000000); // 100 > 200 = 0
        // threshold keys on GREEN only (R/B do not matter)
        check_op("thr100-greenkey",   3'd4, 8'd100, 24'hFF_C8_FF, 24'hFFFFFF); // g=200 > 100 = 1

        // --- 3) the new port is inert on other ops ---
        check_op("pass-thresh-inert", 3'd0, 8'd50,  24'h12_34_56, 24'h123456); // passthrough
        check_op("invert-thresh-inert",3'd1, 8'd50, 24'h00_00_00, 24'hFFFFFF); // ~0 = FFFFFF
        check_op("gray-thresh-inert", 3'd2, 8'd200, 24'hAA_55_CC, 24'h555555); // {g,g,g}, g=0x55

        in_valid = 1'b0;
        repeat (4) @(negedge clk);

        if (errors == 0) $display("TB_PASS: axis_rgb_proc_slot (runtime threshold + default-128 + port inert OK)");
        else             $display("TB_FAIL: %0d error(s)", errors);
        $finish;
    end

endmodule

`default_nettype wire
