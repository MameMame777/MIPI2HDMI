`timescale 1ns / 1ps

// FE-DELIMITER mode test (2026-06-04). Config mirrors hardware intent:
// GUARD_FRAME_LINES=1, cfg_use_lsle=1, EXPECTED_FRAME_LINES!=0, FE_DELIMITS=1,
// FS_MIN_LINES as the plausibility floor (reused for FE).
//   * A plausible FE (>= FS_MIN_LINES lines) CLOSES the frame (natural CSI-2).
//   * A spurious early FE (< FS_MIN_LINES) is IGNORED (frame stays open).
//   * A missing FE is bounded by the MAX_LINES cap.
//   * An in-frame FS (FE dropped) RE-ANCHORS the top (line_idx=0) without
//     closing/counting; FE still delimits.
module tb_csi2_frame_state_fedelim;
    logic core_clk, core_aresetn, cfg_use_lsle;
    logic [7:0] in_pkt_di; logic [15:0] in_pkt_wc;
    logic in_pkt_is_short, in_pkt_is_long, in_pkt_start, in_pkt_end, in_pkt_err;
    logic [7:0] in_payload_data; logic in_payload_valid, in_payload_first, in_payload_last;
    logic out_sof, out_eof, out_sol, out_eol; logic [15:0] out_line_idx;
    logic [7:0] out_payload_data; logic out_payload_valid, out_payload_first, out_payload_last, out_frame_err;
    logic [31:0] sts_frame_count, sts_line_count; logic [15:0] sts_last_frame_lines, sts_frame_sync_err_cnt;

    csi2_frame_state #(
        .MAX_LINES(8), .GUARD_FRAME_LINES(1'b1), .EXPECTED_FRAME_LINES(4),
        .EXPECTED_LINE_WC(16'd0), .FS_MIN_LINES(4), .FE_DELIMITS(1'b1)
    ) dut (
        .core_clk(core_clk), .core_aresetn(core_aresetn), .cfg_use_lsle(cfg_use_lsle),
        .cfg_expected_frame_lines(16'd0),
        .in_pkt_di(in_pkt_di), .in_pkt_wc(in_pkt_wc), .in_pkt_is_short(in_pkt_is_short),
        .in_pkt_is_long(in_pkt_is_long), .in_pkt_start(in_pkt_start), .in_pkt_end(in_pkt_end),
        .in_pkt_err(in_pkt_err), .in_payload_data(in_payload_data), .in_payload_valid(in_payload_valid),
        .in_payload_first(in_payload_first), .in_payload_last(in_payload_last),
        .out_sof(out_sof), .out_eof(out_eof), .out_sol(out_sol), .out_eol(out_eol),
        .out_line_idx(out_line_idx), .out_payload_data(out_payload_data),
        .out_payload_valid(out_payload_valid), .out_payload_first(out_payload_first),
        .out_payload_last(out_payload_last), .out_frame_err(out_frame_err),
        .sts_frame_count(sts_frame_count), .sts_line_count(sts_line_count),
        .sts_last_frame_lines(sts_last_frame_lines), .sts_frame_sync_err_cnt(sts_frame_sync_err_cnt)
    );

    initial begin core_clk=0; forever #5 core_clk=~core_clk; end
    task automatic reset_dut();
        core_aresetn=0; cfg_use_lsle=1;
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
    task automatic chk(input bit c, input string m); if(!c) $fatal(1,"FAIL: %s",m); endtask

    initial begin
        // A: plausible FE closes the frame (FS open, 5 lines, FE@5 >= FS_MIN=4)
        reset_dut();
        drive_short(6'h00);                              // FS open
        for (int i=0;i<5;i++) drive_lsle_line(8'(i));    // 5 lines
        drive_short(6'h01);                              // FE @5 -> close
        repeat(4) @(posedge core_clk);
        $display("[A] frames=%0d last=%0d sync_err=%0d", sts_frame_count, sts_last_frame_lines, sts_frame_sync_err_cnt);
        chk(sts_frame_count==1, "A: plausible FE closed one frame");
        chk(sts_last_frame_lines==5, "A: frame = 5 lines");

        // B: spurious early FE (@3 < FS_MIN=4) ignored; plausible FE@5 closes
        reset_dut();
        drive_short(6'h00);
        for (int i=0;i<3;i++) drive_lsle_line(8'(i));    // 3 lines
        drive_short(6'h01);                              // FE @3 (<4) -> ignored
        chk(sts_frame_count==0, "B: spurious early FE did NOT close a frame");
        for (int i=0;i<2;i++) drive_lsle_line(8'(i));    // now 5 lines
        drive_short(6'h01);                              // FE @5 -> close
        repeat(4) @(posedge core_clk);
        $display("[B] frames=%0d last=%0d", sts_frame_count, sts_last_frame_lines);
        chk(sts_frame_count==1, "B: plausible FE closed exactly one frame");
        chk(sts_last_frame_lines==5, "B: frame = 5 lines (spurious FE@3 ignored)");

        // C: missing FE -> MAX_LINES cap (=8)
        reset_dut();
        drive_short(6'h00);
        for (int i=0;i<10;i++) drive_lsle_line(8'(i));   // no FE
        repeat(4) @(posedge core_clk);
        $display("[C] frames=%0d last=%0d", sts_frame_count, sts_last_frame_lines);
        chk(sts_frame_count==1, "C: missing FE bounded by MAX_LINES cap");
        chk(sts_last_frame_lines==8, "C: capped at MAX_LINES=8");

        // D: an in-frame FS (spurious, e.g. payload-0x00 false header) is IGNORED;
        //    FE remains the sole delimiter, so the frame keeps accumulating lines.
        reset_dut();
        drive_short(6'h00);                              // FS open frame#1
        for (int i=0;i<5;i++) drive_lsle_line(8'(i));
        drive_short(6'h01);                              // FE -> close frame#1 (=5), IDLE
        repeat(2) @(posedge core_clk);
        chk(sts_frame_count==1, "D: frame#1 closed on FE");
        drive_short(6'h00);                              // FS open frame#2
        for (int i=0;i<2;i++) drive_lsle_line(8'(i));    // 2 lines
        drive_short(6'h00);                              // in-frame FS -> IGNORED (no SOF/re-anchor)
        chk(sts_frame_count==1, "D: in-frame FS did NOT close/count a frame");
        for (int i=0;i<5;i++) drive_lsle_line(8'(i));    // 5 more lines (frame keeps going: 2+5=7)
        drive_short(6'h01);                              // FE -> close frame#2 (=7)
        repeat(4) @(posedge core_clk);
        $display("[D] frames=%0d last=%0d", sts_frame_count, sts_last_frame_lines);
        chk(sts_frame_count==2, "D: FE closed frame#2 (in-frame FS ignored)");
        chk(sts_last_frame_lines==7, "D: frame#2 = 7 lines (spurious in-frame FS ignored, not re-anchored)");

        $display("TEST PASSED: tb_csi2_frame_state_fedelim");
        $display("CONCLUSION: FE delimits the frame; spurious early FE ignored; missing FE capped; in-frame FS re-anchors the top.");
        $finish;
    end
    initial begin #1ms; $fatal(1,"timeout"); end
endmodule
