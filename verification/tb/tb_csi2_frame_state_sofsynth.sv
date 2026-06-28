`timescale 1ns / 1ps

// SOF-SYNTHESIS mode test (2026-06-13). With the D-PHY lane supervisor enabled
// the chip's FS (frame-start) short packet is lost (fs=0) but LS/long/FE still
// arrive. cfg_sof_synth=1 opens a frame from the first in-IDLE LS so the VDMA
// gets an SOF; FE_DELIMITS still closes at the chip's true bottom; a dropped FE
// is bounded by MAX_LINES. Config mirrors hardware intent (GUARD=1, lsle=1,
// FE_DELIMITS=1, FS_MIN=4 plausibility floor, MAX_LINES=8 scaled).
//   A: FS-LESS stream, sof_synth=1 -> SOF synthesized, FE closes each frame.
//   B: FS-LESS stream, sof_synth=0 -> frame never opens (legacy, no SOF).
//   C: intermittent FS, sof_synth=1 -> normal FS open coexists with synthetic.
//   D: FS-less + dropped FE, sof_synth=1 -> MAX_LINES cap bounds the frame.
module tb_csi2_frame_state_sofsynth;
    logic core_clk, core_aresetn, cfg_use_lsle, cfg_sof_synth;
    logic [7:0] in_pkt_di; logic [15:0] in_pkt_wc;
    logic in_pkt_is_short, in_pkt_is_long, in_pkt_start, in_pkt_end, in_pkt_err;
    logic [7:0] in_payload_data; logic in_payload_valid, in_payload_first, in_payload_last;
    logic out_sof, out_eof, out_sol, out_eol; logic [15:0] out_line_idx;
    logic [7:0] out_payload_data; logic out_payload_valid, out_payload_first, out_payload_last, out_frame_err;
    logic [31:0] sts_frame_count, sts_line_count; logic [15:0] sts_last_frame_lines, sts_frame_sync_err_cnt;

    int sof_cnt;
    always @(posedge core_clk) if (core_aresetn && out_sof) sof_cnt <= sof_cnt + 1;

    csi2_frame_state #(
        .MAX_LINES(8), .GUARD_FRAME_LINES(1'b1), .EXPECTED_FRAME_LINES(4),
        .EXPECTED_LINE_WC(16'd0), .FS_MIN_LINES(4), .FE_DELIMITS(1'b1)
    ) dut (
        .core_clk(core_clk), .core_aresetn(core_aresetn), .cfg_use_lsle(cfg_use_lsle),
        .cfg_expected_frame_lines(16'd0), .cfg_sof_synth(cfg_sof_synth),
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
    task automatic reset_dut(input bit synth);
        core_aresetn=0; cfg_use_lsle=1; cfg_sof_synth=synth; sof_cnt=0;
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
        // A: FS-LESS stream, sof_synth=1 -> synthesize SOF per frame, FE closes
        reset_dut(1'b1);
        for (int i=0;i<5;i++) drive_lsle_line(8'(i));    // 5 lines, NO opening FS
        drive_short(6'h01);                              // FE @5 (>=FS_MIN=4) -> close
        repeat(4) @(posedge core_clk);
        $display("[A] frames=%0d last=%0d sof=%0d sync_err=%0d", sts_frame_count, sts_last_frame_lines, sof_cnt, sts_frame_sync_err_cnt);
        chk(sts_frame_count==1, "A: synthetic-open frame closed by FE");
        chk(sts_last_frame_lines==5, "A: frame = 5 lines");
        chk(sof_cnt==1, "A: exactly one synthesized SOF");
        // second frame, still no FS
        for (int i=0;i<5;i++) drive_lsle_line(8'(i));
        drive_short(6'h01);                              // FE -> close frame#2
        repeat(4) @(posedge core_clk);
        $display("[A2] frames=%0d sof=%0d", sts_frame_count, sof_cnt);
        chk(sts_frame_count==2, "A2: second FS-less frame opened+closed");
        chk(sof_cnt==2, "A2: two synthesized SOFs total");

        // B: FS-LESS stream, sof_synth=0 -> frame never opens, no SOF
        reset_dut(1'b0);
        for (int i=0;i<5;i++) drive_lsle_line(8'(i));
        drive_short(6'h01);
        repeat(4) @(posedge core_clk);
        $display("[B] frames=%0d sof=%0d", sts_frame_count, sof_cnt);
        chk(sts_frame_count==0, "B: legacy (sof_synth=0) does NOT open without FS");
        chk(sof_cnt==0, "B: no SOF without FS in legacy mode");

        // C: intermittent FS, sof_synth=1 -> normal FS open coexists with synthetic
        reset_dut(1'b1);
        // frame#1 via SYNTHETIC open (no FS)
        for (int i=0;i<5;i++) drive_lsle_line(8'(i));
        drive_short(6'h01);                              // FE -> close#1
        repeat(2) @(posedge core_clk);
        chk(sts_frame_count==1, "C: synthetic frame#1 closed");
        // frame#2 via REAL FS
        drive_short(6'h00);                              // FS open frame#2 (normal path)
        for (int i=0;i<5;i++) drive_lsle_line(8'(i));
        drive_short(6'h01);                              // FE -> close#2
        repeat(4) @(posedge core_clk);
        $display("[C] frames=%0d last=%0d sof=%0d", sts_frame_count, sts_last_frame_lines, sof_cnt);
        chk(sts_frame_count==2, "C: real-FS frame#2 also closed");
        // synthetic open = 1 SOF (payload-aligned); real FS = 2 SOF (existing:
        // immediate on FS cycle + sof_pending on first payload) => total 3.
        chk(sof_cnt==3, "C: synthetic(1) + FS-driven(2, existing double-SOF) SOFs");

        // D: FS-less + dropped FE, sof_synth=1 -> MAX_LINES cap bounds the frame
        reset_dut(1'b1);
        for (int i=0;i<10;i++) drive_lsle_line(8'(i));   // no FS, no FE
        repeat(4) @(posedge core_clk);
        $display("[D] frames=%0d last=%0d sof=%0d", sts_frame_count, sts_last_frame_lines, sof_cnt);
        chk(sts_frame_count==1, "D: synthetic open + missing FE bounded by MAX cap");
        chk(sts_last_frame_lines==8, "D: capped at MAX_LINES=8");

        // E: FE-RESYNC (2026-06-13) — after a cap-close (FE was dropped, so the
        //    vblank phase is unknown) the synthetic open is SUPPRESSED until the
        //    next FE re-establishes the chip frame top. Removes per-frame vertical
        //    phase jitter (the live-HDMI "defocused" look).
        reset_dut(1'b1);
        for (int i=0;i<8;i++) drive_lsle_line(8'(i));    // 8 lines, no FS/FE -> cap@8
        repeat(2) @(posedge core_clk);
        chk(sts_frame_count==1, "E: cap-close counted one frame");
        chk(sts_last_frame_lines==8, "E: capped at 8");
        // while waiting for FE, further LS must NOT open a frame (phase unknown)
        for (int i=0;i<2;i++) drive_lsle_line(8'(i));
        chk(sts_frame_count==1, "E: LS after cap-close does NOT open (waiting for FE)");
        // the chip's FE arrives in IDLE and re-establishes phase (no open/close itself)
        drive_short(6'h01);                              // FE -> resync
        repeat(2) @(posedge core_clk);
        chk(sts_frame_count==1, "E: resync FE does not itself open/close a frame");
        // now the next frame opens at the true top and FE closes it
        for (int i=0;i<5;i++) drive_lsle_line(8'(i));    // 5 lines
        drive_short(6'h01);                              // FE -> close frame#2
        repeat(4) @(posedge core_clk);
        $display("[E] frames=%0d last=%0d", sts_frame_count, sts_last_frame_lines);
        chk(sts_frame_count==2, "E: after FE-resync the next frame opens+closes (phase-locked)");
        chk(sts_last_frame_lines==5, "E: phase-locked frame = 5 lines");

        $display("TEST PASSED: tb_csi2_frame_state_sofsynth");
        $display("CONCLUSION: cfg_sof_synth opens a frame from the first LS when FS is absent; FE delimits; legacy unaffected; MAX cap bounds dropped FE; FE-resync phase-locks after a cap-close.");
        $finish;
    end
    initial begin #1ms; $fatal(1,"timeout"); end
endmodule
