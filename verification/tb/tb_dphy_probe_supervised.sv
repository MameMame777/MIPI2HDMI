`timescale 1ns / 1ps
`default_nettype none

// TB-2: dphy_hs_byte_probe + dphy_lane_supervisor integration.
//
// Validates the opt-in supervisor wiring added for the 3% capture fix
// (diary 2026-06-12, memory project_frontend_3pct_capture_root_cause):
//   - supervisor bufr_clr gates the (behavioural) BUFR -> byte_clk
//   - clock-lane HS entry releases bufr_clr so byte_clk runs
//   - the SoT-accept gate (!sup_enable || sup_hs_settled) blocks a HS-prepare
//     0xB8 before HS-SETTLE elapses, then a post-settle 0xB8+header locks
//   - sup_lock_cnt / sup_settle_cnt advance
//   - clock-lane gating (LP-11) re-asserts bufr_clr / clears rx_clk_active
//   - sup_enable=0 keeps legacy behaviour (clean burst locks with no gate)
//
// The byte stream is forced onto dut.serdes_byte_sample exactly like the other
// dphy_hs_byte_probe TBs (the ISERDES sim stub does not deserialise).
// Plan: docs/plan (luminous-twirling-hare) S5b.

module tb_dphy_probe_supervised;

    // ---- clocks ----
    logic ctl_clk = 1'b0;      // 200 MHz supervisor ctl_clk
    logic hs_clk_p = 1'b0;     // 100 MHz HS clock (byte_clk passthrough via BUFR stub)
    always #2.5 ctl_clk = ~ctl_clk;
    always #5.0 hs_clk_p = ~hs_clk_p;
    wire hs_clk_n = ~hs_clk_p;

    // ---- DUT I/O ----
    logic rst_n;
    logic [1:0] data_hs_p = 2'b00;
    logic [1:0] data_hs_n = 2'b11;
    logic [1:0] data_lp_p, data_lp_n;
    logic [1:0] clk_lp;                 // {LP_p, LP_n} clock lane

    logic byte_clk;
    logic [1:0] lane_sot_seen;
    logic sync_header_valid;
    logic [7:0] sync_header_di;
    logic [15:0] sync_header_wc;
    logic [3:0] sync_header_score;

    // ---- supervisor <-> probe nets ----
    logic sup_enable;
    logic cfg_hs_settle_gate = 1'b0;    // decoupled legacy HS-SETTLE gate (T7)
    logic [3:0] cfg_settle_blank_k = 4'd0;   // byte-domain settle blank (T8)
    logic [15:0] dbg_burst_count;
    logic [15:0] dbg_sot_burst_count;
    logic [31:0] dbg_missed_burst;
    logic sup_bufr_clr;
    logic sup_serdes_rst;
    logic sup_rx_clk_active;
    logic sup_hs_settled;
    logic [2:0] sup_clk_state;
    logic [2:0] sup_data_state;
    logic [7:0] sup_lock_cnt;
    logic [7:0] sup_settle_cnt;

    int errors = 0;
    task automatic check(input bit cond, input string msg);
        if (!cond) begin
            errors++;
            $display("CHECK FAILED: %s (t=%0t)", msg, $time);
        end else begin
            $display("PASS: %s", msg);
        end
    endtask

    // ------------------------------------------------------------------
    // DUT: probe (sup_* driven by supervisor) -- only the ports we use are
    // connected; the many unused trace/header outputs are left open.
    // ------------------------------------------------------------------
    dphy_hs_byte_probe #(
        .LANES(2),
        .SOT_WINDOW_BYTES(32),
        .SWEEP_HOLD_BYTES(8),
        .SWEEP_ENABLE(1'b0),
        .FIXED_BITSLIP_PHASE(0),
        .FIXED_BITSLIP_PHASE_LANE1(0),
        .LANE1_BITSLIP_SWEEP_ENABLE(1'b0),
        .FIXED_TRANSFORM(0),
        .TRACE_TRIGGER_MODE(3),
        .EXPECTED_LONG_DT(8'h1e),
        .EXPECTED_LONG_WC(16'd1280),
        .MIN_SYNC_HEADER_SCORE(13),
        .SYNC_HEADER_SWEEP_BIT_OFFSETS(1'b0),
        .SYNC_HEADER_USE_ALIGNED_STREAM(1'b0),
        .STREAM_PAIRING(0)
    ) dut (
        .rst_n(rst_n),
        .idelay_ref_clk(ctl_clk),
        .idelay_ref_reset(!rst_n),
        .runtime_idelay_tap(5'd0),
        .runtime_idelay_tap_lane1(5'd0),
        .runtime_bitslip_phase(3'd0),
        .runtime_bitslip_phase_lane1(3'd0),
        .runtime_lane1_sweep_enable(1'b0),
        .runtime_expected_long_dt(8'h00),
        .sup_enable(sup_enable),
        .sup_bufr_clr(sup_bufr_clr),
        .sup_serdes_rst(sup_serdes_rst),
        .sup_hs_settled(sup_hs_settled),
        .cfg_hs_settle_gate(cfg_hs_settle_gate),
        .dphy_hs_clock_clk_p(hs_clk_p),
        .dphy_hs_clock_clk_n(hs_clk_n),
        .dphy_data_hs_p(data_hs_p),
        .dphy_data_hs_n(data_hs_n),
        .dphy_data_lp_p(data_lp_p),
        .dphy_data_lp_n(data_lp_n),
        .byte_clk(byte_clk),
        .lane_sot_seen(lane_sot_seen),
        .sync_header_valid(sync_header_valid),
        .sync_header_di(sync_header_di),
        .sync_header_wc(sync_header_wc),
        .sync_header_score(sync_header_score),
        .cfg_settle_blank_k(cfg_settle_blank_k),
        .dbg_burst_count(dbg_burst_count),
        .dbg_sot_burst_count(dbg_sot_burst_count),
        .dbg_missed_burst(dbg_missed_burst)
    );

    // ------------------------------------------------------------------
    // Supervisor: small T_INIT for sim speed (full timing covered by TB-1).
    // ------------------------------------------------------------------
    dphy_lane_supervisor #(
        .CTL_CLK_HZ(200_000_000),
        .T_INIT_US(1),
        .T_INIT_FORCE_US(3)
    ) u_sup (
        .ctl_clk           (ctl_clk),
        .ctl_aresetn       (rst_n),
        .clk_lp            (clk_lp),
        .data_lp           ({data_lp_p[0], data_lp_n[0]}),
        .byte_clk          (byte_clk),
        .bufr_clr          (sup_bufr_clr),
        .rx_clk_active_byte(sup_rx_clk_active),
        .serdes_rst_byte   (sup_serdes_rst),
        .hs_settled_byte   (sup_hs_settled),
        .sts_clk_state     (sup_clk_state),
        .sts_data_state    (sup_data_state),
        .sts_lock_cnt      (sup_lock_cnt),
        .sts_settle_cnt    (sup_settle_cnt)
    );

    // ------------------------------------------------------------------
    // Stimulus helpers
    // ------------------------------------------------------------------
    // Force a post-ISERDES byte sample, advancing one byte_clk cycle.
    task automatic drive_serdes(input logic [7:0] b0, input logic [7:0] b1);
        @(negedge byte_clk);
        dut.serdes_byte_sample[0] = b0;
        dut.serdes_byte_sample[1] = b1;
        @(posedge byte_clk);
        #1;
    endtask

    // Clock-lane HS entry: 11 -> 01 -> 00. Releases bufr_clr after T_CLK_SETTLE.
    task automatic clock_lane_hs_entry;
        clk_lp = 2'b11; #60ns;
        clk_lp = 2'b01; #60ns;
        clk_lp = 2'b00;                 // HS clock active
        // wait for the supervisor to release the BUFR
        fork : wait_release
            begin wait (sup_bufr_clr == 1'b0); end
            begin #2000ns; $fatal(1, "bufr_clr never released"); end
        join_any
        disable wait_release;
    endtask

    // Wait helper on ctl_clk for a level with timeout.
    task automatic wait_settled;
        fork : wait_s
            begin wait (sup_hs_settled == 1'b1); end
            begin #3000ns; $fatal(1, "hs_settled never rose"); end
        join_any
        disable wait_s;
    endtask

    // ------------------------------------------------------------------
    initial begin
        $display("tb_dphy_probe_supervised start");
        sup_enable = 1'b1;              // supervisor mode under test
        clk_lp     = 2'b11;
        data_lp_p  = 2'b11;
        data_lp_n  = 2'b11;
        rst_n      = 1'b0;
        repeat (10) @(posedge ctl_clk);
        rst_n = 1'b1;

        // --- T1: clock-lane HS entry releases bufr_clr, byte_clk runs --------
        check(sup_bufr_clr == 1'b1, "bufr_clr held before clock-lane entry");
        clock_lane_hs_entry();
        repeat (4) @(posedge byte_clk);     // proves byte_clk is now toggling
        check(sup_bufr_clr == 1'b0, "bufr_clr released after clock-lane HS entry");
        check(sup_rx_clk_active == 1'b1, "rx_clk_active set after byte_clk restart");

        // Data lane: wait for the supervisor data FSM to leave DT_INIT (T_INIT)
        // and observe stop (LP-11). data_lp is held 11 since reset.
        fork : wait_dtstop
            begin wait (sup_data_state == 3'd2); end   // DT_STOP
            begin #4000ns; $fatal(1, "data FSM never reached DT_STOP"); end
        join_any
        disable wait_dtstop;

        // --- T2: data-lane HS entry; pre-settle SoT must be gated ------------
        // LP-11 -> LP-01 (HS request) -> LP-00 (HS). Settle starts at LP-00.
        @(negedge byte_clk); data_lp_p = 2'b00; data_lp_n = 2'b11;  // LP-01 HS request
        #40ns;
        @(negedge byte_clk); data_lp_p = 2'b00; data_lp_n = 2'b00;  // LP-00 HS
        check(sup_hs_settled == 1'b0, "hs_settled still low right after data LP-00");
        // Push a SoT 0xB8 BEFORE settle completes; the gate must block the lock.
        drive_serdes(8'hb8, 8'hb8);
        drive_serdes(8'hb8, 8'hb8);
        check(lane_sot_seen == 2'b00, "pre-settle SoT is gated (no lock)");
        check(sync_header_valid == 1'b0, "no sync header from pre-settle SoT");
        // Flush serdes to a non-SoT byte so the held 0xB8 cannot lock at settle.
        drive_serdes(8'h00, 8'h00);

        // --- T3: after settle, a fresh SoT+header locks ----------------------
        wait_settled();
        check(sup_hs_settled == 1'b1, "hs_settled rose after T_HS_SETTLE");
        // header: lane0 = b8,1e,05,11,33 ... lane1 = b8,00,<ecc>,22,44
        // ECC for DI=0x1e WC=0x0500 (pairing0) is 0x1e per the probe's table.
        drive_serdes(8'hb8, 8'hb8);
        drive_serdes(8'h1e, 8'h00);
        drive_serdes(8'h05, 8'h1e);
        drive_serdes(8'h11, 8'h22);
        drive_serdes(8'h33, 8'h44);
        // let the header pipeline settle
        for (int i = 0; i < 40 && !sync_header_valid; i++) drive_serdes(8'h00, 8'h00);
        check(lane_sot_seen[0] == 1'b1, "post-settle SoT locked lane0");
        check(sync_header_valid == 1'b1, "sync header captured after settle");
        check(sync_header_di == 8'h1e, "sync header DI = 0x1e");

        // --- T4: supervisor counters advanced --------------------------------
        check(sup_lock_cnt >= 8'd1, "lock_cnt counted clock-lane lock");
        check(sup_settle_cnt >= 8'd1, "settle_cnt counted data-lane settle");

        // --- T5: clock-lane gating re-asserts bufr_clr -----------------------
        clk_lp = 2'b11;                 // vblank: clock lane back to LP-11
        #200ns;
        check(sup_bufr_clr == 1'b1, "bufr_clr re-asserts on clock LP-11");
        check(sup_rx_clk_active == 1'b0, "rx_clk_active async-clears on gate");
        check(sup_hs_settled == 1'b0, "hs_settled clears on clock outage");

        // --- T6: legacy (sup_enable=0) clean burst still locks ---------------
        sup_enable = 1'b0;
        data_lp_p = 2'b11; data_lp_n = 2'b11;
        clock_lane_hs_entry();          // byte_clk runs again (bufr_clr ignored anyway)
        repeat (4) @(posedge byte_clk);
        // data LP edge opens the window the legacy way
        @(negedge byte_clk); data_lp_p = 2'b00; data_lp_n = 2'b00;
        drive_serdes(8'hb8, 8'hb8);
        drive_serdes(8'h1e, 8'h00);
        drive_serdes(8'h05, 8'h1e);
        drive_serdes(8'h11, 8'h22);
        drive_serdes(8'h33, 8'h44);
        for (int i = 0; i < 40 && !sync_header_valid; i++) drive_serdes(8'h00, 8'h00);
        check(sync_header_valid == 1'b1, "legacy mode (sup_enable=0) still captures header");

        // --- T7: legacy + cfg_hs_settle_gate=1 gates pre-settle SoT ----------
        // The decoupled gate (2026-06-17): sup_enable=0 (so the BUFR.CLR gating
        // that breaks continuous lock stays off) but cfg_hs_settle_gate=1 still
        // gates the SoT search on sup_hs_settled -- the same gate as T2, reached
        // via the new control bit. Pulse rst_n to clear sticky lock flags, then
        // replay a fresh gate cycle.
        sup_enable         = 1'b0;
        cfg_hs_settle_gate = 1'b1;
        clk_lp = 2'b11; data_lp_p = 2'b11; data_lp_n = 2'b11;
        rst_n = 1'b0;
        repeat (10) @(posedge ctl_clk);
        rst_n = 1'b1;
        clock_lane_hs_entry();
        repeat (4) @(posedge byte_clk);
        fork : wait_dtstop2
            begin wait (sup_data_state == 3'd2); end       // DT_STOP
            begin #4000ns; $fatal(1, "T7 data FSM never reached DT_STOP"); end
        join_any
        disable wait_dtstop2;
        @(negedge byte_clk); data_lp_p = 2'b00; data_lp_n = 2'b11;  // LP-01 HS request
        #40ns;
        @(negedge byte_clk); data_lp_p = 2'b00; data_lp_n = 2'b00;  // LP-00 HS
        check(sup_hs_settled == 1'b0, "T7 hs_settled low right after data LP-00");
        drive_serdes(8'hb8, 8'hb8);
        drive_serdes(8'hb8, 8'hb8);
        check(lane_sot_seen == 2'b00,
              "T7 cfg_hs_settle_gate blocks pre-settle SoT (sup_enable=0)");
        check(sync_header_valid == 1'b0, "T7 no sync header from pre-settle SoT");
        drive_serdes(8'h00, 8'h00);
        wait_settled();
        check(sup_hs_settled == 1'b1, "T7 hs_settled rose after T_HS_SETTLE");
        drive_serdes(8'hb8, 8'hb8);
        drive_serdes(8'h1e, 8'h00);
        drive_serdes(8'h05, 8'h1e);
        drive_serdes(8'h11, 8'h22);
        drive_serdes(8'h33, 8'h44);
        for (int i = 0; i < 40 && !sync_header_valid; i++) drive_serdes(8'h00, 8'h00);
        check(lane_sot_seen[0] == 1'b1, "T7 post-settle SoT locked lane0");
        check(sync_header_valid == 1'b1, "T7 sync header captured after settle (gated path)");

        // --- T8: cfg_settle_blank_k delays the SoT window K byte_clk after LP-exit;
        // a normal burst (SoT after the blank) still locks, and the SoT-miss
        // counters (burst / sot_burst) advance. (Blank-skip efficacy is measured on
        // HW by sweeping K vs last_fe.) ---
        sup_enable         = 1'b0;
        cfg_hs_settle_gate = 1'b0;
        cfg_settle_blank_k = 4'd4;
        clk_lp = 2'b11; data_lp_p = 2'b11; data_lp_n = 2'b11;
        rst_n = 1'b0;
        repeat (10) @(posedge ctl_clk);
        rst_n = 1'b1;
        clock_lane_hs_entry();
        repeat (4) @(posedge byte_clk);
        @(negedge byte_clk); data_lp_p = 2'b00; data_lp_n = 2'b00;   // LP-exit -> blank starts
        repeat (10) drive_serdes(8'h00, 8'h00);   // clear LP-sync latency + the 4-cycle blank with non-SoT
        drive_serdes(8'hb8, 8'hb8);               // real SoT after the window opens
        drive_serdes(8'h1e, 8'h00);
        drive_serdes(8'h05, 8'h1e);
        drive_serdes(8'h11, 8'h22);
        drive_serdes(8'h33, 8'h44);
        for (int i = 0; i < 40 && !sync_header_valid; i++) drive_serdes(8'h00, 8'h00);
        check(sync_header_valid == 1'b1, "T8 blank=4: a normal burst still locks");
        check(dbg_burst_count != 16'd0, "T8 burst_count advanced (LP-exit edges)");
        check(dbg_sot_burst_count != 16'd0, "T8 sot_burst_count advanced (SoT in window)");

        // --- T9: sup_enable=1 + cfg_settle_blank_k>0 -> the sup HS-SETTLE SoT gate is
        // DECOUPLED (settle_gate_en=0), so the byte-domain settle-blank handles the
        // burst head instead of the sup gate (the two no longer stack/over-blank in
        // gated, 2026-06-18). The combined mode (sup BUFR/ISERDES mgmt + settle-blank)
        // still locks a normal burst. ---
        sup_enable         = 1'b1;
        cfg_hs_settle_gate = 1'b0;
        cfg_settle_blank_k = 4'd4;
        clk_lp = 2'b11; data_lp_p = 2'b11; data_lp_n = 2'b11;
        rst_n = 1'b0;
        repeat (10) @(posedge ctl_clk);
        rst_n = 1'b1;
        clock_lane_hs_entry();
        repeat (4) @(posedge byte_clk);
        fork : wait_dtstop3
            begin wait (sup_data_state == 3'd2); end       // DT_STOP
            begin #4000ns; $fatal(1, "T9 data FSM never reached DT_STOP"); end
        join_any
        disable wait_dtstop3;
        @(negedge byte_clk); data_lp_p = 2'b00; data_lp_n = 2'b11;  // LP-01 HS request
        #40ns;
        @(negedge byte_clk); data_lp_p = 2'b00; data_lp_n = 2'b00;  // LP-00 HS
        wait_settled();                            // sup hs_settled rises (mgmt still active)
        repeat (10) drive_serdes(8'h00, 8'h00);    // clear the blank with non-SoT
        drive_serdes(8'hb8, 8'hb8);
        drive_serdes(8'h1e, 8'h00);
        drive_serdes(8'h05, 8'h1e);
        drive_serdes(8'h11, 8'h22);
        drive_serdes(8'h33, 8'h44);
        for (int i = 0; i < 40 && !sync_header_valid; i++) drive_serdes(8'h00, 8'h00);
        check(sync_header_valid == 1'b1,
              "T9 sup+settle-blank (decoupled sup gate) still locks");

        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED: %0d errors", errors);
        $finish;
    end

    initial begin
        #2ms;
        $display("TEST FAILED: TB watchdog timeout");
        $finish;
    end

endmodule

`default_nettype wire
