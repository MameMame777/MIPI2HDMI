`timescale 1ns / 1ps

// Rolling root-cause reproduction (task 2, 2026-06-02).
//
// Goal: with the SAME FS-anchor FSM that already passes the ideal-input
// regression (tb_csi2_frame_state_linecount), determine what INPUT pattern is
// necessary and sufficient to make the captured frame phase ROLL.
//
// The FS-anchor FSM (cfg_use_lsle + GUARD) closes a frame on the next FS and
// re-syncs line_idx=0, so each frame = the line span BETWEEN consecutive FS.
// Therefore:
//   * if FS arrives at a CONSTANT content position (constant FS-to-FS line
//     span) the per-frame line count is constant => frame phase is LOCKED
//     (writing a constant-height frame into the fixed VDMA buffer = no roll).
//   * if the FS-to-FS line span VARIES, last_frame_lines varies => when written
//     into the fixed 480-line buffer the content origin walks = ROLL.
//
// This TB drives three streams and records last_frame_lines per closed frame:
//   LOCK     : FS every 5 lines, x6           -> all spans == 5 (no roll)
//   ROLL     : FS at 5,4,6,3,7 lines          -> spans vary (roll mechanism)
//   SPURIOUS : one extra mid-frame FS          -> injects a short frame
//
// CONCLUSION the sim establishes: the FSM faithfully follows FS; rolling is
// caused ONLY by FS-to-FS span variation, NOT by the FSM. Whether that span
// variation originates at the chip MIPI-TX or in the FPGA front-end (SoT/ECC/
// CDC/vcdt) cannot be told from sim -> that is what the glitch-free hardware
// FS->LE latch counter (task 1) must measure.

module tb_csi2_frame_state_roll_repro;
    logic core_clk;
    logic core_aresetn;
    logic cfg_use_lsle;

    logic [7:0]  in_pkt_di;
    logic [15:0] in_pkt_wc;
    logic        in_pkt_is_short;
    logic        in_pkt_is_long;
    logic        in_pkt_start;
    logic        in_pkt_end;
    logic        in_pkt_err;
    logic [7:0]  in_payload_data;
    logic        in_payload_valid;
    logic        in_payload_first;
    logic        in_payload_last;

    logic        out_sof, out_eof, out_sol, out_eol;
    logic [15:0] out_line_idx;
    logic [7:0]  out_payload_data;
    logic        out_payload_valid, out_payload_first, out_payload_last, out_frame_err;
    logic [31:0] sts_frame_count;
    logic [31:0] sts_line_count;
    logic [15:0] sts_last_frame_lines;
    logic [15:0] sts_frame_sync_err_cnt;

    // record last_frame_lines on each EOF
    int  span_log[$];
    always_ff @(posedge core_clk) begin
        if (core_aresetn && out_eof) span_log.push_back(sts_last_frame_lines);
    end

    csi2_frame_state #(
        .MAX_LINES(32),
        .GUARD_FRAME_LINES(1'b1),
        .EXPECTED_FRAME_LINES(8),   // != 0 enables lsle_line_guard (FS-anchor)
        .EXPECTED_LINE_WC(16'd0)
    ) dut (
        .core_clk(core_clk), .core_aresetn(core_aresetn),
        .cfg_use_lsle(cfg_use_lsle), .cfg_expected_frame_lines(16'd0),
        .in_pkt_di(in_pkt_di), .in_pkt_wc(in_pkt_wc),
        .in_pkt_is_short(in_pkt_is_short), .in_pkt_is_long(in_pkt_is_long),
        .in_pkt_start(in_pkt_start), .in_pkt_end(in_pkt_end), .in_pkt_err(in_pkt_err),
        .in_payload_data(in_payload_data), .in_payload_valid(in_payload_valid),
        .in_payload_first(in_payload_first), .in_payload_last(in_payload_last),
        .out_sof(out_sof), .out_eof(out_eof), .out_sol(out_sol), .out_eol(out_eol),
        .out_line_idx(out_line_idx), .out_payload_data(out_payload_data),
        .out_payload_valid(out_payload_valid), .out_payload_first(out_payload_first),
        .out_payload_last(out_payload_last), .out_frame_err(out_frame_err),
        .sts_frame_count(sts_frame_count), .sts_line_count(sts_line_count),
        .sts_last_frame_lines(sts_last_frame_lines),
        .sts_frame_sync_err_cnt(sts_frame_sync_err_cnt)
    );

    initial begin core_clk = 1'b0; forever #5 core_clk = ~core_clk; end

    task automatic reset_dut();
        core_aresetn = 1'b0; cfg_use_lsle = 1'b1;
        in_pkt_di = 8'h00; in_pkt_wc = 16'h0000;
        in_pkt_is_short = 1'b0; in_pkt_is_long = 1'b0;
        in_pkt_start = 1'b0; in_pkt_end = 1'b0; in_pkt_err = 1'b0;
        in_payload_data = 8'h00; in_payload_valid = 1'b0;
        in_payload_first = 1'b0; in_payload_last = 1'b0;
        span_log.delete();
        repeat (8) @(posedge core_clk); core_aresetn = 1'b1;
        repeat (2) @(posedge core_clk);
    endtask

    task automatic drive_short(input logic [5:0] dt);
        @(posedge core_clk);
        in_pkt_di <= {2'b00, dt}; in_pkt_wc <= 16'h0000;
        in_pkt_is_short <= 1'b1; in_pkt_is_long <= 1'b0;
        in_pkt_start <= 1'b1; in_pkt_end <= 1'b1;
        @(posedge core_clk);
        in_pkt_start <= 1'b0; in_pkt_end <= 1'b0; in_pkt_is_short <= 1'b0;
    endtask

    task automatic drive_lsle_line(input logic [7:0] data);
        drive_short(6'h02);                       // LS
        @(posedge core_clk);
        in_pkt_di <= 8'h2a; in_pkt_wc <= 16'd1;
        in_pkt_is_short <= 1'b0; in_pkt_is_long <= 1'b1; in_pkt_start <= 1'b1;
        @(posedge core_clk);
        in_pkt_start <= 1'b0; in_payload_data <= data;
        in_payload_first <= 1'b1; in_payload_last <= 1'b1; in_payload_valid <= 1'b1;
        @(posedge core_clk);
        in_payload_valid <= 1'b0; in_payload_first <= 1'b0; in_payload_last <= 1'b0;
        in_pkt_end <= 1'b1;
        @(posedge core_clk);
        in_pkt_end <= 1'b0; in_pkt_is_long <= 1'b0;
        drive_short(6'h03);                       // LE
    endtask

    task automatic drive_frame(input int nlines);
        drive_short(6'h00);                       // FS (closes prior, opens new)
        for (int i = 0; i < nlines; i++) drive_lsle_line(8'(i));
    endtask

    function automatic bit all_equal(input int q[$]);
        all_equal = 1'b1;
        for (int i = 1; i < q.size(); i++) if (q[i] != q[0]) all_equal = 1'b0;
    endfunction

    task automatic check(input bit cond, input string msg);
        if (!cond) $fatal(1, "CHECK FAILED: %s", msg);
    endtask

    int spans[$];
    initial begin
        // ---------- LOCK: constant FS-to-FS span -> phase locked --------------
        reset_dut();
        for (int f = 0; f < 6; f++) drive_frame(5);
        drive_short(6'h00);                       // final FS closes last frame
        repeat (4) @(posedge core_clk);
        spans = span_log;
        $display("[LOCK]     closed-frame spans = %p", spans);
        check(spans.size() >= 5, "LOCK: at least 5 frames closed");
        check(all_equal(spans), "LOCK: constant FS interval => constant frame height (PHASE LOCKED)");

        // ---------- ROLL: varying FS-to-FS span -> rolling --------------------
        reset_dut();
        drive_frame(5); drive_frame(4); drive_frame(6); drive_frame(3); drive_frame(7);
        drive_short(6'h00);
        repeat (4) @(posedge core_clk);
        spans = span_log;
        $display("[ROLL]     closed-frame spans = %p", spans);
        check(spans.size() >= 5, "ROLL: frames closed");
        check(!all_equal(spans), "ROLL: varying FS interval => varying frame height (ROLL MECHANISM)");

        // ---------- SPURIOUS: one extra mid-frame FS injects a short frame ----
        reset_dut();
        drive_short(6'h00);                       // FS open
        for (int i = 0; i < 2; i++) drive_lsle_line(8'(i));
        drive_short(6'h00);                       // SPURIOUS mid-frame FS -> closes 2-line frame
        for (int i = 0; i < 5; i++) drive_lsle_line(8'(i));
        drive_short(6'h00);                       // closes 5-line frame
        repeat (4) @(posedge core_clk);
        spans = span_log;
        $display("[SPURIOUS] closed-frame spans = %p (a stray FS injected a short %0d-line frame)",
                 spans, spans[0]);
        check(spans.size() == 2, "SPURIOUS: two frames closed");
        check(spans[0] == 2 && spans[1] == 5, "SPURIOUS: stray FS makes a 2-line then 5-line frame");

        $display("TEST PASSED: tb_csi2_frame_state_roll_repro");
        $display("CONCLUSION: the FS-anchor FSM faithfully reproduces the FS-to-FS span as");
        $display("the frame height. CONSTANT span => locked; VARYING/STRAY FS => roll. The");
        $display("FSM is exonerated; rolling requires FS-to-FS span variation in its INPUT.");
        $display("Whether that variation is chip MIPI-TX or FPGA front-end is NOT decidable");
        $display("in sim -> needs the glitch-free hardware FS->LE latch counter (task 1).");
        $finish;
    end

    initial begin #1ms; $fatal(1, "Simulation timeout"); end
endmodule
