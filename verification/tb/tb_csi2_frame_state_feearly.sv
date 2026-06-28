`timescale 1ns / 1ps

// Spurious-early-FE windowing fix (2026-06-15).
//
// Hardware root cause (diary 20260615): on a locked continuous link ~480 long
// packets reach the parser per frame, but the only FE per frame fires ~30-40
// lines EARLY (~line 441, fe_after_480 ~= 0) -- a SPURIOUS FE on the open-loop
// byte aligner. The current FE-delimiter closes the frame on any FE past the
// FS_MIN_LINES floor (=300 on hw), so it closes short and the tail ~28 lines
// fall outside the FS->FE window (long_before_fs) -> the bottom band.
//
// Fix: FE_MIN_LINES (a floor near EXPECTED_FRAME_LINES). An FE below it is
// rejected as spurious; the frame then closes on the real next-frame FS
// (line_idx >= FE_MIN_LINES, lost-FE recovery) or the MAX_LINES cap, capturing
// the full frame. FE_MIN_LINES=0 = legacy (regression-safe).
//
// Two DUTs are driven identically: dut_old (FE_MIN_LINES=0, the buggy legacy)
// vs dut_new (FE_MIN_LINES=7). Scaled model: EXPECTED=8, FS_MIN=2, FE_MIN=7,
// MAX_LINES=16.
module tb_csi2_frame_state_feearly;
    logic core_clk, core_aresetn, cfg_use_lsle;
    logic cfg_force_expected_tb, cfg_sof_synth_tb, cfg_long_as_line_tb;
    logic [7:0] in_pkt_di; logic [15:0] in_pkt_wc;
    logic in_pkt_is_short, in_pkt_is_long, in_pkt_start, in_pkt_end, in_pkt_err;
    logic [7:0] in_payload_data; logic in_payload_valid, in_payload_first, in_payload_last;

    // dut_old (legacy FE_MIN_LINES=0) outputs
    logic o_sof, o_eof, o_sol, o_eol; logic [15:0] o_line_idx;
    logic [7:0] o_pd; logic o_pv, o_pf, o_pl, o_ferr;
    logic [31:0] o_fcnt, o_lcnt; logic [15:0] o_last, o_serr;
    // dut_new (fixed FE_MIN_LINES=7) outputs
    logic n_sof, n_eof, n_sol, n_eol; logic [15:0] n_line_idx;
    logic [7:0] n_pd; logic n_pv, n_pf, n_pl, n_ferr;
    logic [31:0] n_fcnt, n_lcnt; logic [15:0] n_last, n_serr;

    localparam int MAXL = 16, EXP = 8, FSMIN = 2, FEMIN = 7;

    csi2_frame_state #(
        .MAX_LINES(MAXL), .GUARD_FRAME_LINES(1'b1), .EXPECTED_FRAME_LINES(EXP),
        .EXPECTED_LINE_WC(16'd0), .FS_MIN_LINES(FSMIN), .FE_DELIMITS(1'b1),
        .FE_MIN_LINES(0)
    ) dut_old (
        .core_clk(core_clk), .core_aresetn(core_aresetn), .cfg_use_lsle(cfg_use_lsle),
        .cfg_expected_frame_lines(16'd0),
        .in_pkt_di(in_pkt_di), .in_pkt_wc(in_pkt_wc), .in_pkt_is_short(in_pkt_is_short),
        .in_pkt_is_long(in_pkt_is_long), .in_pkt_start(in_pkt_start), .in_pkt_end(in_pkt_end),
        .in_pkt_err(in_pkt_err), .in_payload_data(in_payload_data), .in_payload_valid(in_payload_valid),
        .in_payload_first(in_payload_first), .in_payload_last(in_payload_last),
        .out_sof(o_sof), .out_eof(o_eof), .out_sol(o_sol), .out_eol(o_eol),
        .out_line_idx(o_line_idx), .out_payload_data(o_pd), .out_payload_valid(o_pv),
        .out_payload_first(o_pf), .out_payload_last(o_pl), .out_frame_err(o_ferr),
        .sts_frame_count(o_fcnt), .sts_line_count(o_lcnt),
        .sts_last_frame_lines(o_last), .sts_frame_sync_err_cnt(o_serr)
    );

    csi2_frame_state #(
        .MAX_LINES(MAXL), .GUARD_FRAME_LINES(1'b1), .EXPECTED_FRAME_LINES(EXP),
        .EXPECTED_LINE_WC(16'd0), .FS_MIN_LINES(FSMIN), .FE_DELIMITS(1'b1),
        .FE_MIN_LINES(FEMIN)
    ) dut_new (
        .core_clk(core_clk), .core_aresetn(core_aresetn), .cfg_use_lsle(cfg_use_lsle),
        .cfg_expected_frame_lines(16'd0),
        .cfg_sof_synth(cfg_sof_synth_tb),
        .cfg_force_expected(cfg_force_expected_tb),
        .cfg_long_as_line(cfg_long_as_line_tb),
        .in_pkt_di(in_pkt_di), .in_pkt_wc(in_pkt_wc), .in_pkt_is_short(in_pkt_is_short),
        .in_pkt_is_long(in_pkt_is_long), .in_pkt_start(in_pkt_start), .in_pkt_end(in_pkt_end),
        .in_pkt_err(in_pkt_err), .in_payload_data(in_payload_data), .in_payload_valid(in_payload_valid),
        .in_payload_first(in_payload_first), .in_payload_last(in_payload_last),
        .out_sof(n_sof), .out_eof(n_eof), .out_sol(n_sol), .out_eol(n_eol),
        .out_line_idx(n_line_idx), .out_payload_data(n_pd), .out_payload_valid(n_pv),
        .out_payload_first(n_pf), .out_payload_last(n_pl), .out_frame_err(n_ferr),
        .sts_frame_count(n_fcnt), .sts_line_count(n_lcnt),
        .sts_last_frame_lines(n_last), .sts_frame_sync_err_cnt(n_serr)
    );

    initial begin core_clk=0; forever #5 core_clk=~core_clk; end

    task automatic reset_dut();
        core_aresetn=0; cfg_use_lsle=1; cfg_force_expected_tb=0; cfg_sof_synth_tb=0;
        cfg_long_as_line_tb=0;
        in_pkt_di=0; in_pkt_wc=0; in_pkt_is_short=0; in_pkt_is_long=0;
        in_pkt_start=0; in_pkt_end=0; in_pkt_err=0;
        in_payload_data=0; in_payload_valid=0; in_payload_first=0; in_payload_last=0;
        repeat(8) @(posedge core_clk); core_aresetn=1; repeat(2) @(posedge core_clk);
    endtask
    task automatic drive_short(input logic [5:0] dt);
        @(posedge core_clk); in_pkt_di<={2'b00,dt}; in_pkt_wc<=0;
        in_pkt_is_short<=1; in_pkt_is_long<=0; in_pkt_start<=1; in_pkt_end<=1;
        @(posedge core_clk); in_pkt_start<=0; in_pkt_end<=0; in_pkt_is_short<=0;
    endtask
    task automatic drive_lsle_line(input logic [7:0] d);
        drive_short(6'h02);                              // LS
        @(posedge core_clk); in_pkt_di<=8'h2a; in_pkt_wc<=16'd1;
        in_pkt_is_short<=0; in_pkt_is_long<=1; in_pkt_start<=1;
        @(posedge core_clk); in_pkt_start<=0; in_payload_data<=d;
        in_payload_first<=1; in_payload_last<=1; in_payload_valid<=1;
        @(posedge core_clk); in_payload_valid<=0; in_payload_first<=0; in_payload_last<=0; in_pkt_end<=1;
        @(posedge core_clk); in_pkt_end<=0; in_pkt_is_long<=0;
        drive_short(6'h03);                              // LE  (line_idx++)
    endtask
    // a long packet + LE, with NO preceding LS (the LS was dropped upstream)
    task automatic drive_long_le_no_ls(input logic [7:0] d);
        @(posedge core_clk); in_pkt_di<=8'h2a; in_pkt_wc<=16'd1;
        in_pkt_is_short<=0; in_pkt_is_long<=1; in_pkt_start<=1;
        @(posedge core_clk); in_pkt_start<=0; in_payload_data<=d;
        in_payload_first<=1; in_payload_last<=1; in_payload_valid<=1;
        @(posedge core_clk); in_payload_valid<=0; in_payload_first<=0; in_payload_last<=0; in_pkt_end<=1;
        @(posedge core_clk); in_pkt_end<=0; in_pkt_is_long<=0;
        drive_short(6'h03);                              // LE (line_idx++)
    endtask
    task automatic chk(input bit c, input string m); if(!c) $fatal(1,"FAIL: %s",m); endtask

    initial begin
        // ============================================================
        // T1 — hardware scenario: spurious early FE@5 + LOST real FE,
        //      frame is 8 lines, closed by the real next-frame FS.
        // ============================================================
        reset_dut();
        drive_short(6'h00);                              // FS open frame#1
        for (int i=0;i<5;i++) drive_lsle_line(8'(i));    // lines 0..4 (line_idx=5)
        drive_short(6'h01);                              // SPURIOUS FE @5 (real FE lost)
        for (int i=5;i<8;i++) drive_lsle_line(8'(i));    // lines 5..7 (line_idx=8)
        drive_short(6'h00);                              // real next-frame FS @line_idx=8
        repeat(4) @(posedge core_clk);
        $display("[T1] OLD frames=%0d last=%0d | NEW frames=%0d last=%0d",
                 o_fcnt, o_last, n_fcnt, n_last);
        // OLD: spurious FE@5 (>=FS_MIN=2) closes the frame SHORT at 5 (the bug).
        chk(o_last==5, "T1 OLD: legacy closes short on spurious FE@5 (bug repro)");
        // NEW: FE@5 (<FE_MIN=7) rejected; frame closes on real FS at full 8.
        chk(n_last==8, "T1 NEW: spurious FE rejected, frame closes at full 8 on FS");
        chk(n_fcnt==1, "T1 NEW: exactly one frame closed");

        // ============================================================
        // T2 — regression: clean real FE@8 closes the frame at 8.
        // ============================================================
        reset_dut();
        drive_short(6'h00);
        for (int i=0;i<8;i++) drive_lsle_line(8'(i));    // 8 lines
        drive_short(6'h01);                              // real FE @8 (>=FE_MIN=7)
        repeat(4) @(posedge core_clk);
        $display("[T2] NEW frames=%0d last=%0d", n_fcnt, n_last);
        chk(n_fcnt==1, "T2 NEW: clean FE closed one frame");
        chk(n_last==8, "T2 NEW: clean FE closes at 8");

        // ============================================================
        // T3 — both a spurious early FS@3 and a spurious early FE@5 are
        //      rejected; the frame still closes at the full 8 on the real FS.
        // ============================================================
        reset_dut();
        drive_short(6'h00);
        for (int i=0;i<3;i++) drive_lsle_line(8'(i));    // 3 lines
        drive_short(6'h00);                              // spurious FS @3 (<FE_MIN=7) -> ignored
        drive_short(6'h01);                              // spurious FE @3 (<FE_MIN=7) -> ignored
        for (int i=3;i<8;i++) drive_lsle_line(8'(i));    // up to 8 lines
        drive_short(6'h00);                              // real FS @8 -> close
        repeat(4) @(posedge core_clk);
        $display("[T3] NEW frames=%0d last=%0d", n_fcnt, n_last);
        chk(n_fcnt==1, "T3 NEW: one frame despite early spurious FS+FE");
        chk(n_last==8, "T3 NEW: frame closes at full 8 (early spurious FS/FE ignored)");

        // ============================================================
        // T4 — missing FE AND missing closing FS -> MAX_LINES cap (=16).
        // ============================================================
        reset_dut();
        drive_short(6'h00);
        for (int i=0;i<20;i++) drive_lsle_line(8'(i));   // no FE, no closing FS
        repeat(4) @(posedge core_clk);
        $display("[T4] NEW frames=%0d last=%0d", n_fcnt, n_last);
        chk(n_fcnt==1, "T4 NEW: runaway bounded by MAX_LINES cap");
        chk(n_last==MAXL, "T4 NEW: capped at MAX_LINES");

        // ============================================================
        // T5 — FORCE-EXPECTED (2026-06-16, live-HDMI roll fix): the chip
        //      OVERSHOOTS (12 LE/frame: EXP image + embedded), but with
        //      cfg_force_expected the frame force-closes at EXACTLY EXP=8 for a
        //      constant-height VTC stream. The overshoot lines drain in IDLE;
        //      the next FS reopens. Two overshoot frames => two clamped closes.
        // ============================================================
        reset_dut();
        cfg_force_expected_tb = 1;
        drive_short(6'h00);                              // FS open frame#1
        for (int i=0;i<12;i++) drive_lsle_line(8'(i));   // 12 LE: force-close at 8
        drive_short(6'h00);                              // FS reopens frame#2
        for (int i=0;i<12;i++) drive_lsle_line(8'(i));   // 12 LE: force-close at 8
        drive_short(6'h00);                              // FS (opens frame#3)
        repeat(4) @(posedge core_clk);
        $display("[T5] FORCE frames=%0d last=%0d serr=%0d", n_fcnt, n_last, n_serr);
        chk(n_last==EXP, "T5 FORCE: frame clamped to EXPECTED=8 (constant height)");
        chk(n_fcnt==2, "T5 FORCE: two overshoot frames each force-closed + reopened");
        // OLD (no force, FE_MIN=0): the 12-line frames are NOT clamped to 8.
        chk(o_last!=EXP || o_fcnt!=2, "T5 OLD: legacy does NOT clamp to 8 (force off)");

        // ============================================================
        // T6 — SYNTH + FORCE (2026-06-16, bottom-band fix): force_expected
        //      IGNORES every FE (the early FE can't close short), and the
        //      force-close re-syncs via synth (synth_wait_fe=1) so the next
        //      frame re-opens at the chip's TRUE top on the first LS AFTER the
        //      FE. The overshoot LE drain in IDLE while waiting. Result: a
        //      constant EXP-height frame opened at the true top (no bottom band).
        // ============================================================
        reset_dut();
        cfg_sof_synth_tb = 1; cfg_force_expected_tb = 1;
        for (int i=0;i<8;i++) drive_lsle_line(8'(i));    // frame#1: 8 lines -> force-close @8
        for (int i=0;i<4;i++) drive_lsle_line(8'(i));    // overshoot -> drained (synth_wait_fe)
        drive_short(6'h01);                              // chip FE -> clears synth_wait_fe
        for (int i=0;i<8;i++) drive_lsle_line(8'(i));    // frame#2: opens at true top, force-close @8
        for (int i=0;i<4;i++) drive_lsle_line(8'(i));    // overshoot -> drained
        drive_short(6'h01);                              // FE
        repeat(4) @(posedge core_clk);
        $display("[T6] SYNTH+FORCE frames=%0d last=%0d", n_fcnt, n_last);
        chk(n_last==EXP, "T6 SYNTH+FORCE: clamped to EXPECTED=8 (FE ignored, force closes)");
        chk(n_fcnt==2, "T6 SYNTH+FORCE: 2 frames, overshoot drained, FE-resync re-open");

        // ============================================================
        // T7 — LONG-AS-LINE (2026-06-17, bottom-band fix): longs whose LS was
        //      dropped upstream arrive with line_open=0. With cfg_long_as_line=1
        //      (dut_new) they are DELIVERED as rows (open the line on the long);
        //      with it off (dut_old, default) they are REJECTED (sync err). Feed 5
        //      such no-LS longs and confirm OLD rejects them while NEW does not.
        // ============================================================
        reset_dut();
        cfg_long_as_line_tb = 1;                         // dut_new ON, dut_old OFF
        drive_short(6'h00);                              // FS open
        for (int i=0;i<5;i++) drive_long_le_no_ls(8'(i));// 5 longs, NO LS each
        drive_short(6'h00);                              // next FS
        repeat(4) @(posedge core_clk);
        $display("[T7] LONG-AS-LINE: NEW serr=%0d lines=%0d | OLD serr=%0d lines=%0d",
                 n_serr, n_lcnt, o_serr, o_lcnt);
        chk(o_serr - n_serr >= 5,
            "T7: long-as-line ON delivers the 5 no-LS longs that OLD (off) rejects");

        $display("TEST PASSED: tb_csi2_frame_state_feearly");
        $display("CONCLUSION: FE_MIN_LINES rejects the spurious early FE; the real");
        $display("next-frame FS (lost-FE recovery) closes the frame at the full");
        $display("height, recovering the ~28 long_before_fs tail lines (the band).");
        $finish;
    end
    initial begin #1ms; $fatal(1,"timeout"); end
endmodule
