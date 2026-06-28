`timescale 1ns / 1ps
`default_nettype none

// TB: dphy_hwlock_fsm unit test (E2 HW deterministic-lock FSM).
// Models the lock-quality input hdr_active as a function of the FSM's swept
// bitslip target (bitslip_p0/p1 -> combo) AND a "/4 phase" that advances on each
// bufr_clr re-roll, then checks:
//   T1 lock-on-current-phase : a good combo (any phase) -> FSM sweeps to it and HOLDs (locked).
//   T4 hold + collapse re-lock: drop hdr_active > LOST_CYC in HOLD -> locked drops & re-sweeps,
//                               restore -> re-locks.
//   T3 lock-after-one-reroll : good combo only on phase 1 -> one bufr_clr re-roll then locks.
//   T2 never-lockable        : no good combo on any phase -> re-rolls MAX_REROLL times -> failed.
// Small *_CYC params keep the sim short. See dphy_hwlock_fsm.sv / plan_hwlock_fsm_20260619.

module tb_dphy_hwlock_fsm;

    localparam int unsigned SETTLE_MIN_CYC = 15;
    localparam int unsigned SETTLE_CYC     = 40;
    localparam int unsigned REROLL_CYC     = 10;
    localparam int unsigned LOST_CYC       = 120;
    localparam int unsigned RETRY_CYC      = 300;
    localparam int unsigned MAX_REROLL     = 8;

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    logic enable = 1'b0;

    logic [2:0] bitslip_p0, bitslip_p1;
    logic       bufr_clr, locked, failed;
    logic [2:0] dbg_state;
    logic [3:0] dbg_reroll;
    logic [5:0] dbg_combo;

    // lock-quality model -------------------------------------------------------
    logic [5:0] good_combo = 6'd6;     // {p0=0,p1=6}
    int         good_phase = -1;       // /4 phase that makes good_combo lock; -1 = any, 99 = never
    int         phase = 0;             // advances on each bufr_clr re-roll
    logic       force_lo = 1'b0;       // T4: force hdr_active low even on a good combo
    wire        cur_is_good = ({bitslip_p0, bitslip_p1} == good_combo)
                              && ((good_phase == -1) || (phase == good_phase));
    wire        hdr_active   = cur_is_good && !force_lo;

    // advance the modelled /4 phase on each bufr_clr rising edge
    logic bufr_clr_d;
    always @(posedge clk) begin
        bufr_clr_d <= bufr_clr;
        if (bufr_clr && !bufr_clr_d) phase <= phase + 1;
    end

    dphy_hwlock_fsm #(
        .SETTLE_MIN_CYC(SETTLE_MIN_CYC),
        .SETTLE_CYC    (SETTLE_CYC),
        .REROLL_CYC    (REROLL_CYC),
        .LOST_CYC      (LOST_CYC),
        .RETRY_CYC     (RETRY_CYC),
        .MAX_REROLL    (MAX_REROLL)
    ) dut (
        .clk(clk), .rst_n(rst_n), .enable(enable), .hdr_active(hdr_active),
        .bitslip_p0(bitslip_p0), .bitslip_p1(bitslip_p1),
        .bufr_clr(bufr_clr), .locked(locked), .failed(failed),
        .dbg_state(dbg_state), .dbg_reroll(dbg_reroll), .dbg_combo(dbg_combo)
    );

    always #2.5 clk = ~clk;   // 200 MHz

    int errors = 0;
    int checks = 0;
    task automatic chk(input logic cond, input string msg);
        checks++;
        if (cond) $display("  PASS: %s", msg);
        else begin
            errors++;
            $display("  FAIL: %s (state=%0d combo=%0d reroll=%0d locked=%0b failed=%0b phase=%0d)",
                     msg, dbg_state, dbg_combo, dbg_reroll, locked, failed, phase);
        end
    endtask

    // poll `locked` until high or `cycles` elapse
    task automatic wait_locked(input int cycles, output logic ok);
        int n; ok = 1'b0;
        for (n = 0; n < cycles; n++) begin @(posedge clk); if (locked) begin ok = 1'b1; return; end end
    endtask
    task automatic wait_failed(input int cycles, output logic ok);
        int n; ok = 1'b0;
        for (n = 0; n < cycles; n++) begin @(posedge clk); if (failed) begin ok = 1'b1; return; end end
    endtask
    task automatic wait_unlocked(input int cycles, output logic ok);
        int n; ok = 1'b0;
        for (n = 0; n < cycles; n++) begin @(posedge clk); if (!locked) begin ok = 1'b1; return; end end
    endtask

    logic ok;

    initial begin
        rst_n = 1'b0; enable = 1'b0; force_lo = 1'b0;
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (3) @(posedge clk);
        chk(dbg_state == 0 && !locked && !failed, "after reset: IDLE, !locked, !failed");

        // ================= T1: lock on current phase =================
        $display("T1: good combo (0,6) any phase -> sweep & lock");
        good_combo = 6'd6; good_phase = -1; phase = 0; force_lo = 1'b0;
        enable = 1'b1;
        wait_locked(8*SETTLE_CYC + 4*SETTLE_MIN_CYC + 200, ok);
        chk(ok, "T1 locked asserted");
        chk(dbg_combo == 6'd6, "T1 locked at combo (0,6)");
        chk(bitslip_p0 == 3'd0 && bitslip_p1 == 3'd6, "T1 bitslip target = (0,6)");
        chk(!failed, "T1 not failed");
        repeat (20) @(posedge clk);
        chk(locked && dbg_state == 3, "T1 stays in HOLD/locked");

        // ================= T4: collapse in HOLD -> re-lock =================
        $display("T4: drop hdr_active > LOST_CYC while held -> re-lock");
        force_lo = 1'b1;                          // link collapses
        wait_unlocked(LOST_CYC + 2*SETTLE_CYC + 100, ok);
        chk(ok && !locked, "T4 locked dropped after collapse");
        force_lo = 1'b0;                          // restore -> should re-lock (good_phase=-1, re-rolls OK)
        wait_locked(70*SETTLE_CYC + REROLL_CYC + 300, ok);
        chk(ok, "T4 re-locked after restore");
        enable = 1'b0;
        repeat (4) @(posedge clk);
        chk(dbg_state == 0 && !locked, "T4 disable -> IDLE, !locked");

        // ================= T3: lock only after one re-roll =================
        $display("T3: good combo (2,3) only on phase 1 -> one re-roll then lock");
        good_combo = {3'd2, 3'd3}; good_phase = 1; phase = 0; force_lo = 1'b0;
        rst_n = 1'b0; repeat (3) @(posedge clk); rst_n = 1'b1; repeat (2) @(posedge clk);
        enable = 1'b1;
        wait_locked(64*SETTLE_CYC + REROLL_CYC + 16*SETTLE_CYC + 400, ok);
        chk(ok, "T3 locked after re-roll");
        chk(dbg_reroll == 4'd1, "T3 exactly one re-roll");
        chk(dbg_combo == {3'd2, 3'd3}, "T3 locked at combo (2,3)");
        enable = 1'b0; repeat (4) @(posedge clk);

        // ================= T2: never lockable -> failed =================
        $display("T2: no good combo on any phase -> re-roll MAX then fail");
        good_combo = 6'd6; good_phase = 99; phase = 0; force_lo = 1'b0;  // 99 = unreachable
        rst_n = 1'b0; repeat (3) @(posedge clk); rst_n = 1'b1; repeat (2) @(posedge clk);
        enable = 1'b1;
        wait_failed((MAX_REROLL+1)*(64*SETTLE_CYC + REROLL_CYC) + 800, ok);
        chk(ok, "T2 failed asserted");
        chk(!locked, "T2 not locked");
        chk(dbg_reroll == MAX_REROLL[3:0], "T2 reroll count = MAX_REROLL");

        // ============ T5: FAILED auto-retries -> locks when a stream appears ============
        // (continues from T2's FAILED state) model a stream coming up: make a combo
        // good (any phase) -> the FSM must retry the sweep (RETRY_CYC) and lock,
        // clearing `failed`. This is the boot-enable-before-chip-streams path.
        $display("T5: FAILED retries -> locks once a good combo appears (boot-before-stream)");
        good_combo = {3'd0, 3'd0}; good_phase = -1;    // now lockable on any phase
        wait_locked(RETRY_CYC + 70*SETTLE_CYC + 400, ok);
        chk(ok, "T5 re-locked out of FAILED after retry");
        chk(!failed, "T5 failed cleared on lock");
        chk(dbg_state == 3, "T5 in HOLD");

        $display("");
        if (errors == 0) $display("tb_dphy_hwlock_fsm: %0d/%0d PASS", checks, checks);
        else             $display("tb_dphy_hwlock_fsm: %0d/%0d FAIL (%0d errors)", checks-errors, checks, errors);
        $finish;
    end

    initial begin
        #5_000_000;   // 5 ms watchdog
        $display("tb_dphy_hwlock_fsm: TIMEOUT watchdog -- FAIL");
        $finish;
    end

endmodule

`default_nettype wire
