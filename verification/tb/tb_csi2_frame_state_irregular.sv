`timescale 1ns / 1ps

// Regression for the FS-resync fix (2026-05-31, diary 20260530 Phase 9-11).
//
// Hardware runs csi2_frame_state in cfg_use_lsle=1 mode. The chip's FS/FE
// frame delimiters arrive irregularly (occasional missed FE / early FE). This
// TB feeds those irregular-but-valid short-packet sequences and asserts that
// the FSM RE-SYNCHRONISES on the next FS instead of MERGING frames.
//
// Before the fix, csi2_frame_state re-opened on FS-in-frame ONLY in
// (!guard_line_mode && !cfg_use_lsle); in cfg_use_lsle mode a 2nd FS only
// bumped sts_frame_sync_err_cnt and did NOT reset line_idx, so a single dropped
// FE merged two frames (8+8 -> one 16-line frame), amplifying chip FS/FE
// irregularity into the wild last_frame_lines variance (182..1831) on hardware.
// The fix adds an `else if (cfg_use_lsle)` branch that force-closes the stale
// frame (EOF) and re-syncs on the FS. This TB asserts the FIXED behaviour.

module tb_csi2_frame_state_irregular;
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
        .MAX_LINES(4096)
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
    int prev_sync;

    initial begin
        reset_dut();

        // ---------- Scenario 1: CLEAN baseline (LSLE, 8 lines) ----------
        drive_short(6'h00);                 // FS
        for (int i = 0; i < 8; i++) drive_lsle_line(8'(i));
        drive_short(6'h01);                 // FE
        settle();
        $display("[S1 clean]   frames=%0d last_lines=%0d sync_err=%0d",
                 sts_frame_count, sts_last_frame_lines, sts_frame_sync_err_cnt);
        check_condition(sts_frame_count == 32'd1, "S1 one frame");
        check_condition(sts_last_frame_lines == 16'd8, "S1 8 lines");
        check_condition(sts_frame_sync_err_cnt == 16'd0, "S1 no sync err");

        // ---------- Scenario 2: MISSED FE, then next FS (the fix target) ----
        // FS, 8 lines, <FE DROPPED>, FS, 8 lines, FE.
        // FIXED behaviour: FS#2 force-closes frame A (8 lines, EOF) and re-syncs;
        // FE closes frame B (8 lines). => TWO 8-line frames, NO 16-line merge.
        reset_dut();
        drive_short(6'h00);                 // FS (frame A)
        for (int i = 0; i < 8; i++) drive_lsle_line(8'(i));
        // (FE for frame A is intentionally DROPPED)
        drive_short(6'h00);                 // FS (frame B) -- arrives in-frame
        for (int i = 0; i < 8; i++) drive_lsle_line(8'(8 + i));
        drive_short(6'h01);                 // FE
        settle();
        $display("[S2 missedFE] frames=%0d last_lines=%0d sync_err=%0d sof=%0d eof=%0d",
                 sts_frame_count, sts_last_frame_lines, sts_frame_sync_err_cnt,
                 sof_count, eof_count);
        check_condition(sts_frame_count == 32'd2,
                        "S2 FIXED: dropped FE no longer merges -> TWO frames");
        check_condition(sts_last_frame_lines == 16'd8,
                        "S2 FIXED: each frame is 8 lines (NOT a 16-line merge)");
        check_condition(eof_count == 2,
                        "S2 FIXED: both frames emit EOF (stale frame force-closed)");
        check_condition(sts_frame_sync_err_cnt >= 16'd1,
                        "S2 FIXED: the missed FE is still flagged as a sync error");

        // ---------- Scenario 3: EARLY FE + orphan lines ----------
        // FS, 4 lines, FE (early), 4 orphan lines (no frame open), FS, 4, FE.
        reset_dut();
        drive_short(6'h00);                 // FS (frame A)
        for (int i = 0; i < 4; i++) drive_lsle_line(8'(i));
        drive_short(6'h01);                 // FE (early -> 4-line frame)
        settle();
        $display("[S3a earlyFE] frames=%0d last_lines=%0d sync_err=%0d",
                 sts_frame_count, sts_last_frame_lines, sts_frame_sync_err_cnt);
        check_condition(sts_last_frame_lines == 16'd4, "S3a early FE -> 4-line frame");
        prev_frames = sts_frame_count;
        prev_sync   = sts_frame_sync_err_cnt;
        // 4 orphan lines while ST_IDLE (no FS) -> long packets rejected
        for (int i = 0; i < 4; i++) drive_lsle_line(8'(100 + i));
        settle();
        $display("[S3b orphan]  frames=%0d sync_err=%0d (delta_sync=%0d)",
                 sts_frame_count, sts_frame_sync_err_cnt,
                 sts_frame_sync_err_cnt - prev_sync);
        check_condition(sts_frame_count == prev_frames,
                        "S3b orphan lines did NOT create a frame");
        check_condition(sts_frame_sync_err_cnt > prev_sync,
                        "S3b orphan long packets raised sync_err");
        drive_short(6'h00);                 // FS (frame B, clean recovery)
        for (int i = 0; i < 4; i++) drive_lsle_line(8'(i));
        drive_short(6'h01);                 // FE
        settle();
        $display("[S3c recover] frames=%0d last_lines=%0d",
                 sts_frame_count, sts_last_frame_lines);
        check_condition(sts_last_frame_lines == 16'd4, "S3c clean 4-line frame after orphans");

        repeat (10) @(posedge core_clk);
        $display("TEST PASSED: tb_csi2_frame_state_irregular");
        $display("CONCLUSION: with the FS-resync fix, a dropped FE yields TWO clean");
        $display("8-line frames (S2) instead of one merged 16-line frame. The FSM");
        $display("re-synchronises on FS in cfg_use_lsle mode, removing the FPGA-side");
        $display("amplification of chip FS/FE irregularity.");
        $finish;
    end

    initial begin
        #1ms;
        $fatal(1, "Simulation timeout");
    end
endmodule
