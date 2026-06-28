`timescale 1ns / 1ps

// FS plausibility-floor test (2026-06-03). Config under test mirrors hardware:
// GUARD_FRAME_LINES=1, cfg_use_lsle=1, EXPECTED_FRAME_LINES!=0, plus FS_MIN_LINES.
// A spurious FS arriving < FS_MIN_LINES into the frame must be IGNORED (kept
// open); a plausible FS (>= FS_MIN_LINES) delimits the frame; a missing FS is
// bounded by the MAX_LINES cap.

module tb_csi2_frame_state_fsmin;
    logic core_clk, core_aresetn, cfg_use_lsle;
    logic [7:0] in_pkt_di; logic [15:0] in_pkt_wc;
    logic in_pkt_is_short, in_pkt_is_long, in_pkt_start, in_pkt_end, in_pkt_err;
    logic [7:0] in_payload_data; logic in_payload_valid, in_payload_first, in_payload_last;
    logic out_sof, out_eof, out_sol, out_eol; logic [15:0] out_line_idx;
    logic [7:0] out_payload_data; logic out_payload_valid, out_payload_first, out_payload_last, out_frame_err;
    logic [31:0] sts_frame_count, sts_line_count; logic [15:0] sts_last_frame_lines, sts_frame_sync_err_cnt;

    csi2_frame_state #(
        .MAX_LINES(8), .GUARD_FRAME_LINES(1'b1), .EXPECTED_FRAME_LINES(4),
        .EXPECTED_LINE_WC(16'd0), .FS_MIN_LINES(4)
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
        drive_short(6'h02);
        @(posedge core_clk); in_pkt_di<=8'h2a; in_pkt_wc<=16'd1;
        in_pkt_is_short<=0; in_pkt_is_long<=1; in_pkt_start<=1;
        @(posedge core_clk); in_pkt_start<=0; in_payload_data<=d;
        in_payload_first<=1; in_payload_last<=1; in_payload_valid<=1;
        @(posedge core_clk); in_payload_valid<=0; in_payload_first<=0; in_payload_last<=0; in_pkt_end<=1;
        @(posedge core_clk); in_pkt_end<=0; in_pkt_is_long<=0;
        drive_short(6'h03);
    endtask
    task automatic chk(input bit c, input string m); if(!c) $fatal(1,"FAIL: %s",m); endtask

    initial begin
        // A: spurious early FS (@2 lines < FS_MIN=4) must be IGNORED; frame continues
        //    to a plausible FS (@5 >=4) which closes it as a 5-line frame.
        reset_dut();
        drive_short(6'h00);                       // FS (open)
        drive_lsle_line(8'd0); drive_lsle_line(8'd1);   // 2 lines
        drive_short(6'h00);                       // FS @2 (<4) -> SPURIOUS, ignored
        chk(sts_frame_count==0, "A: spurious early FS did NOT close a frame");
        drive_lsle_line(8'd2); drive_lsle_line(8'd3); drive_lsle_line(8'd4); // now 5 lines
        drive_short(6'h00);                       // FS @5 (>=4) -> plausible, close
        repeat(4) @(posedge core_clk);
        $display("[A] frames=%0d last=%0d sync_err=%0d", sts_frame_count, sts_last_frame_lines, sts_frame_sync_err_cnt);
        chk(sts_frame_count==1, "A: plausible FS closed exactly one frame");
        chk(sts_last_frame_lines==5, "A: frame = 5 lines (spurious @2 ignored, real @5 honoured)");

        // B: missing FS -> MAX_LINES cap (=8)
        reset_dut();
        drive_short(6'h00);
        for (int i=0;i<10;i++) drive_lsle_line(8'(i));
        repeat(4) @(posedge core_clk);
        $display("[B] frames=%0d last=%0d", sts_frame_count, sts_last_frame_lines);
        chk(sts_frame_count==1, "B: missing FS bounded by MAX_LINES cap");
        chk(sts_last_frame_lines==8, "B: capped at MAX_LINES=8");

        // C: plausible FS at exactly FS_MIN (4) is accepted
        reset_dut();
        drive_short(6'h00);
        for (int i=0;i<4;i++) drive_lsle_line(8'(i));
        drive_short(6'h00);                       // FS @4 (==FS_MIN) -> accepted
        repeat(4) @(posedge core_clk);
        $display("[C] frames=%0d last=%0d", sts_frame_count, sts_last_frame_lines);
        chk(sts_frame_count==1, "C: FS at exactly FS_MIN accepted");
        chk(sts_last_frame_lines==4, "C: 4-line frame");

        $display("TEST PASSED: tb_csi2_frame_state_fsmin");
        $display("CONCLUSION: spurious FS (< FS_MIN) ignored; plausible FS delimits; cap bounds missing FS.");
        $finish;
    end
    initial begin #1ms; $fatal(1,"timeout"); end
endmodule
