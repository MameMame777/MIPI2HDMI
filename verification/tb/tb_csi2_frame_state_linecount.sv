`timescale 1ns / 1ps

// Regression for the LSLE FS-anchor hybrid frame delimiter (diary 20260601).
//
// The hardware instance runs csi2_frame_state with GUARD_FRAME_LINES=1,
// EXPECTED_FRAME_LINES=480 AND cfg_use_lsle=1. The chip's FS/FE arrive with
// unstable timing; relying purely on a free-running 480-line count phase-locked
// nothing, so a frozen test pattern ROLLED frame-to-frame (vgrad test:
// corr=1.00 at a *varying* vertical shift => lines intact, frame phase drifting).
//
// The hybrid fix delimits frames by FS (the chip's true frame top) to PHASE-LOCK
// them, and keeps the per-line LE count only as a RUNAWAY SAFETY CAP at
// MAX_LINES for frames whose FS the chip dropped. FE is swallowed. This TB uses
// MAX_LINES=8 (so the cap fires quickly) and asserts:
//   A  FS delimits frames (phase anchor)
//   B  FE is swallowed (never closes a frame)
//   C  a missing FS is bounded by the MAX_LINES cap (no unbounded merge)
//   D  an early FS is honoured (frame follows the chip, short or not)

module tb_csi2_frame_state_linecount;
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

    csi2_frame_state #(
        .MAX_LINES(8),                  // small cap so the runaway test fires fast
        .GUARD_FRAME_LINES(1'b1),
        .EXPECTED_FRAME_LINES(4),       // != 0 just to enable lsle_line_guard
        .EXPECTED_LINE_WC(16'd0)
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
        if (!core_aresetn) begin
            sof_count <= 0;
            eof_count <= 0;
        end else begin
            if (out_sof) sof_count <= sof_count + 1;
            if (out_eof) eof_count <= eof_count + 1;
        end
    end

    task automatic reset_dut();
        core_aresetn = 1'b0;
        cfg_use_lsle = 1'b1;            // hardware mode
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

    task automatic drive_short(input logic [5:0] dt);
        @(posedge core_clk);
        in_pkt_di <= {2'b00, dt};
        in_pkt_wc <= 16'h0000;
        in_pkt_is_short <= 1'b1;
        in_pkt_is_long <= 1'b0;
        in_pkt_start <= 1'b1;
        in_pkt_end <= 1'b1;
        @(posedge core_clk);
        in_pkt_start <= 1'b0;
        in_pkt_end <= 1'b0;
        in_pkt_is_short <= 1'b0;
    endtask

    task automatic drive_lsle_line(input logic [7:0] data);
        // LS, long(1 byte), LE  -- the per-line short-packet bracket in LSLE mode
        drive_short(6'h02);                 // LS
        @(posedge core_clk);
        in_pkt_di <= 8'h2a;
        in_pkt_wc <= 16'd1;
        in_pkt_is_short <= 1'b0;
        in_pkt_is_long <= 1'b1;
        in_pkt_start <= 1'b1;
        @(posedge core_clk);
        in_pkt_start <= 1'b0;
        in_payload_data <= data;
        in_payload_first <= 1'b1;
        in_payload_last <= 1'b1;
        in_payload_valid <= 1'b1;
        @(posedge core_clk);
        in_payload_valid <= 1'b0;
        in_payload_first <= 1'b0;
        in_payload_last <= 1'b0;
        in_pkt_end <= 1'b1;
        @(posedge core_clk);
        in_pkt_end <= 1'b0;
        in_pkt_is_long <= 1'b0;
        drive_short(6'h03);                 // LE
    endtask

    task automatic settle();
        repeat (4) @(posedge core_clk);
    endtask

    task automatic check_condition(input bit condition, input string message);
        if (!condition) begin
            $fatal(1, "CHECK FAILED: %s", message);
        end
    endtask

    int prev_frames;

    initial begin
        // ---------- A: FS delimits frames (phase anchor) ----------------------
        // FS, 3 lines, FS, 3 lines, FS. Each in-frame FS closes the prior frame
        // and re-opens => two complete 3-line frames, phase-locked to FS.
        reset_dut();
        drive_short(6'h00);                 // FS (open, from idle)
        for (int i = 0; i < 3; i++) drive_lsle_line(8'(i));
        drive_short(6'h00);                 // FS -> close frame A (3 lines), reopen
        for (int i = 0; i < 3; i++) drive_lsle_line(8'(10 + i));
        drive_short(6'h00);                 // FS -> close frame B (3 lines), reopen
        settle();
        $display("[A fs-anchor] frames=%0d last=%0d sync_err=%0d eof=%0d",
                 sts_frame_count, sts_last_frame_lines, sts_frame_sync_err_cnt, eof_count);
        check_condition(sts_frame_count == 32'd2, "A: FS delimits -> two frames");
        check_condition(sts_last_frame_lines == 16'd3, "A: each frame is the 3 lines between FS");
        check_condition(eof_count == 2, "A: two EOFs (one per FS close)");

        // ---------- B: FE is swallowed (never closes a frame) -----------------
        reset_dut();
        drive_short(6'h00);                 // FS (open)
        for (int i = 0; i < 3; i++) drive_lsle_line(8'(i));
        drive_short(6'h01);                 // FE -> must be swallowed (no close)
        prev_frames = sts_frame_count;
        check_condition(prev_frames == 32'd0, "B: FE did NOT close a frame");
        drive_short(6'h00);                 // FS -> close frame A (3 lines)
        for (int i = 0; i < 3; i++) drive_lsle_line(8'(10 + i));
        drive_short(6'h00);                 // FS -> close frame B
        settle();
        $display("[B fe-swallow] frames=%0d last=%0d", sts_frame_count, sts_last_frame_lines);
        check_condition(sts_frame_count == 32'd2, "B: two frames (FE never added one)");
        check_condition(sts_last_frame_lines == 16'd3, "B: frames are 3 lines");

        // ---------- C: a missing FS is bounded by the MAX_LINES cap -----------
        // FS, then 10 lines with NO further FS. The cap (MAX_LINES=8) closes the
        // runaway frame at 8 lines instead of merging unbounded.
        reset_dut();
        drive_short(6'h00);                 // FS (open)
        for (int i = 0; i < 10; i++) drive_lsle_line(8'(i));   // no FS for 10 lines
        settle();
        $display("[C cap]       frames=%0d last=%0d", sts_frame_count, sts_last_frame_lines);
        check_condition(sts_frame_count == 32'd1, "C: runaway capped -> one frame closed");
        check_condition(sts_last_frame_lines == 16'd8, "C: capped at MAX_LINES (8), not unbounded");

        // ---------- D: an early FS is honoured (frame follows the chip) -------
        reset_dut();
        drive_short(6'h00);                 // FS (open)
        drive_lsle_line(8'd0);
        drive_lsle_line(8'd1);              // only 2 lines
        drive_short(6'h00);                 // FS -> close a SHORT 2-line frame
        settle();
        $display("[D early-fs]  frames=%0d last=%0d", sts_frame_count, sts_last_frame_lines);
        check_condition(sts_frame_count == 32'd1, "D: early FS closes the frame");
        check_condition(sts_last_frame_lines == 16'd2, "D: frame honours FS (2 lines), phase follows chip");

        repeat (10) @(posedge core_clk);
        $display("TEST PASSED: tb_csi2_frame_state_linecount");
        $display("CONCLUSION: in cfg_use_lsle + GUARD mode the frame is PHASE-ANCHORED");
        $display("to the chip FS (so a frozen pattern stops rolling), FE is ignored,");
        $display("and a dropped FS is bounded by the MAX_LINES runaway cap.");
        $finish;
    end

    initial begin
        #1ms;
        $fatal(1, "Simulation timeout");
    end
endmodule
