`timescale 1ns / 1ps

// Pure data-driven (source-following) frame assembly check (2026-06-02).
//
// Deployed config under test: GUARD_FRAME_LINES=0, cfg_use_lsle=1.
// With GUARD=0 the receiver imposes NOTHING: no 480-line forced EOF, no
// WC!=1280 reject, no >=480 long reject, no MAX_LINES cap, no lsle_line_guard.
// The FSM must purely transcribe the chip markers: FS->SOF, (LS,long,LE)->line,
// FE->EOF. Frame height = whatever the source sent BETWEEN FS and FE.
//
// Asserts:
//   A  a 5-line then 3-line frame come out as 5 and 3 (NOT forced to 480)
//   B  a missing FE is resynced by the next FS (no unbounded merge)
//   C  a tall 600-line frame is passed as 600 (NOT capped at 480/512)

module tb_csi2_frame_state_pure_follow;
    logic core_clk, core_aresetn, cfg_use_lsle;
    logic [7:0] in_pkt_di; logic [15:0] in_pkt_wc;
    logic in_pkt_is_short, in_pkt_is_long, in_pkt_start, in_pkt_end, in_pkt_err;
    logic [7:0] in_payload_data; logic in_payload_valid, in_payload_first, in_payload_last;
    logic out_sof, out_eof, out_sol, out_eol; logic [15:0] out_line_idx;
    logic [7:0] out_payload_data; logic out_payload_valid, out_payload_first, out_payload_last, out_frame_err;
    logic [31:0] sts_frame_count, sts_line_count; logic [15:0] sts_last_frame_lines, sts_frame_sync_err_cnt;

    csi2_frame_state #(
        .MAX_LINES(2048), .GUARD_FRAME_LINES(1'b0),
        .EXPECTED_FRAME_LINES(480), .EXPECTED_LINE_WC(16'd1280)
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
        @(posedge core_clk); in_pkt_di<=8'h1e; in_pkt_wc<=16'd1280;
        in_pkt_is_short<=0; in_pkt_is_long<=1; in_pkt_start<=1;
        @(posedge core_clk); in_pkt_start<=0; in_payload_data<=d;
        in_payload_first<=1; in_payload_last<=1; in_payload_valid<=1;
        @(posedge core_clk); in_payload_valid<=0; in_payload_first<=0; in_payload_last<=0; in_pkt_end<=1;
        @(posedge core_clk); in_pkt_end<=0; in_pkt_is_long<=0;
        drive_short(6'h03);
    endtask
    task automatic chk(input bit c, input string m); if(!c) $fatal(1,"FAIL: %s",m); endtask

    initial begin
        // A: 5-line then 3-line frames, delimited by FS/FE -> come out as 5 and 3
        reset_dut();
        drive_short(6'h00);                              // FS
        for (int i=0;i<5;i++) drive_lsle_line(8'(i));
        drive_short(6'h01);                              // FE -> close 5-line frame
        repeat(4) @(posedge core_clk);
        $display("[A] frames=%0d last=%0d", sts_frame_count, sts_last_frame_lines);
        chk(sts_frame_count==1, "A: one frame closed by FE");
        chk(sts_last_frame_lines==5, "A: frame height follows source (5, NOT 480)");
        drive_short(6'h00);                              // FS
        for (int i=0;i<3;i++) drive_lsle_line(8'(i));
        drive_short(6'h01);                              // FE -> close 3-line frame
        repeat(4) @(posedge core_clk);
        $display("[A] frames=%0d last=%0d", sts_frame_count, sts_last_frame_lines);
        chk(sts_frame_count==2, "A: two frames");
        chk(sts_last_frame_lines==3, "A: second frame is 3 lines (source-driven)");

        // B: missing FE -> next FS resyncs (no unbounded merge)
        reset_dut();
        drive_short(6'h00);                              // FS
        for (int i=0;i<4;i++) drive_lsle_line(8'(i));
        drive_short(6'h00);                              // FS again (FE missing) -> force close 4-line
        for (int i=0;i<6;i++) drive_lsle_line(8'(i));
        drive_short(6'h01);                              // FE -> close 6-line
        repeat(4) @(posedge core_clk);
        $display("[B] frames=%0d last=%0d sync_err=%0d", sts_frame_count, sts_last_frame_lines, sts_frame_sync_err_cnt);
        chk(sts_frame_count==2, "B: missing-FE handled by FS resync (2 frames)");

        // C: a tall 600-line frame passes uncapped (no 480/512 limit)
        reset_dut();
        drive_short(6'h00);
        for (int i=0;i<600;i++) drive_lsle_line(8'(i & 8'hff));
        drive_short(6'h01);                              // FE -> close 600-line frame
        repeat(4) @(posedge core_clk);
        $display("[C] frames=%0d last=%0d", sts_frame_count, sts_last_frame_lines);
        chk(sts_frame_count==1, "C: tall frame closed");
        chk(sts_last_frame_lines==600, "C: 600-line frame passed UNCAPPED (no 480/512 impose)");

        $display("TEST PASSED: tb_csi2_frame_state_pure_follow");
        $display("CONCLUSION: with GUARD=0 the FSM purely follows FS/FE; frame height = source.");
        $finish;
    end
    initial begin #2ms; $fatal(1,"timeout"); end
endmodule
