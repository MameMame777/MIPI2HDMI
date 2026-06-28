`timescale 1ns / 1ps
`default_nettype none

// TB-1: dphy_lane_supervisor unit test.
// Drives clock/data lane LP waveforms with D-PHY timing and checks:
//   - bufr_clr releases no earlier than T_CLK_SETTLE (95 ns) after clock
//     lane LP-00, and only via the 11 -> 01 -> 00 entry sequence
//   - rx_clk_active_byte / serdes_rst_byte sequence after byte_clk restart
//   - hs_settled_byte rises no earlier than T_HS_SETTLE (85 ns) after data
//     lane LP-00, and never on a settle aborted by LP-11
//   - clock gating (clock lane LP-11) re-asserts bufr_clr and async-clears
//     the byte-domain flags
//   - cold-attach escape: clock lane stuck at LP-00 from reset locks after
//     T_INIT_FORCE_US
// Plan: docs/plan (rtl-sparkling-emerson) S1.

module tb_dphy_lane_supervisor;

    localparam real CTL_PERIOD_NS  = 5.0;     // 200 MHz
    localparam real BYTE_PERIOD_NS = 18.5;    // ~54 MHz
    localparam int  T_INIT_US_TB       = 1;
    localparam int  T_INIT_FORCE_US_TB = 3;

    logic ctl_clk = 1'b0;
    logic ctl_aresetn = 1'b0;
    logic [1:0] clk_lp  = 2'b11;
    logic [1:0] data_lp = 2'b11;

    logic       bufr_clr;
    logic       rx_clk_active_byte;
    logic       serdes_rst_byte;
    logic       hs_settled_byte;
    logic [2:0] sts_clk_state;
    logic [2:0] sts_data_state;
    logic [7:0] sts_lock_cnt;
    logic [7:0] sts_settle_cnt;

    int errors = 0;
    logic [7:0] lock_snapshot;

    always #(CTL_PERIOD_NS / 2.0) ctl_clk = ~ctl_clk;

    // Behavioural BUFR: divided clock toggles only while the (virtual) HS
    // clock runs and CLR is low.
    logic hs_clk_on = 1'b0;
    logic byte_clk_int = 1'b0;
    always begin
        if (hs_clk_on && !bufr_clr) begin
            #(BYTE_PERIOD_NS / 2.0) byte_clk_int = ~byte_clk_int;
        end else begin
            byte_clk_int = 1'b0;
            #(BYTE_PERIOD_NS / 4.0);
        end
    end
    wire byte_clk = byte_clk_int;

    dphy_lane_supervisor #(
        .CTL_CLK_HZ     (200_000_000),
        .T_INIT_US      (T_INIT_US_TB),
        .T_INIT_FORCE_US(T_INIT_FORCE_US_TB)
    ) dut (
        .ctl_clk           (ctl_clk),
        .ctl_aresetn       (ctl_aresetn),
        .clk_lp            (clk_lp),
        .data_lp           (data_lp),
        .byte_clk          (byte_clk),
        .bufr_clr          (bufr_clr),
        .rx_clk_active_byte(rx_clk_active_byte),
        .serdes_rst_byte   (serdes_rst_byte),
        .hs_settled_byte   (hs_settled_byte),
        .sts_clk_state     (sts_clk_state),
        .sts_data_state    (sts_data_state),
        .sts_lock_cnt      (sts_lock_cnt),
        .sts_settle_cnt    (sts_settle_cnt)
    );

    task automatic check(input bit cond, input string msg);
        if (!cond) begin
            errors++;
            $display("CHECK FAILED: %s (t=%0t)", msg, $time);
        end
    endtask

    // Clock lane HS entry: 11 -> 01 (HS-Rqst) -> 00, HS clock starts at 00
    task automatic clock_lane_hs_entry;
        realtime t_lp00;
        clk_lp = 2'b11; #100ns;
        clk_lp = 2'b01; #60ns;
        clk_lp = 2'b00;
        t_lp00 = $realtime;
        hs_clk_on = 1'b1;
        wait (bufr_clr == 1'b0);
        check(($realtime - t_lp00) >= 95.0,
              $sformatf("bufr_clr released %.1f ns after clk LP-00 (< 95 ns settle)",
                        $realtime - t_lp00));
        check(($realtime - t_lp00) <= 250.0,
              "bufr_clr release took unexpectedly long");
    endtask

    task automatic data_burst_entry;
        realtime t_lp00;
        data_lp = 2'b11;
        // The data-lane FSM sits in DT_INIT for T_INIT after reset and must
        // observe stop (LP-11) before it accepts an HS request.
        wait (sts_data_state == 3'd2);   // DT_STOP
        #20ns;
        data_lp = 2'b01; #60ns;
        data_lp = 2'b00;
        t_lp00 = $realtime;
        wait (hs_settled_byte == 1'b1);
        check(($realtime - t_lp00) >= 85.0,
              $sformatf("hs_settled %.1f ns after data LP-00 (< 85 ns settle)",
                        $realtime - t_lp00));
        check(($realtime - t_lp00) <= 250.0,
              "hs_settled took unexpectedly long");
    endtask

    initial begin
        $display("tb_dphy_lane_supervisor start");
        ctl_aresetn = 1'b0;
        repeat (5) @(posedge ctl_clk);
        ctl_aresetn = 1'b1;

        // --- T1: idle stop state -------------------------------------------------
        #200ns;
        check(bufr_clr == 1'b1, "bufr_clr must be held while clock lane idle");
        check(rx_clk_active_byte == 1'b0, "rx_clk_active must be 0 while idle");
        check(hs_settled_byte == 1'b0, "hs_settled must be 0 while idle");

        // --- T2: clock lane HS entry with settle ---------------------------------
        clock_lane_hs_entry();
        // byte domain comes alive within a few divided-clock cycles
        #(BYTE_PERIOD_NS * 4);
        check(rx_clk_active_byte == 1'b1, "rx_clk_active must rise after restart");
        check(serdes_rst_byte == 1'b0, "serdes_rst must release after restart");
        check(sts_lock_cnt == 8'd1, "lock_cnt must count first lock");

        // --- T3: data burst settle gating ----------------------------------------
        data_burst_entry();
        #20ns;  // ctl-domain counter lags the byte-domain settled flag
        check(sts_settle_cnt == 8'd1, "settle_cnt must count first settle");
        // burst end clears settled
        data_lp = 2'b11;
        #100ns;
        check(hs_settled_byte == 1'b0, "hs_settled must clear at data LP-11");

        // --- T4: aborted settle never sets settled --------------------------------
        data_lp = 2'b01; #60ns;
        data_lp = 2'b00; #40ns;          // abort 40 ns into the 85 ns settle
        data_lp = 2'b11; #150ns;
        check(hs_settled_byte == 1'b0, "aborted settle must not set hs_settled");
        check(sts_settle_cnt == 8'd1, "settle_cnt must not count aborted settle");
        // and a clean burst afterwards still works
        data_burst_entry();
        #20ns;
        check(sts_settle_cnt == 8'd2, "settle_cnt must count clean re-settle");
        data_lp = 2'b11; #60ns;

        // --- T5: clock gating (vblank) -------------------------------------------
        clk_lp = 2'b11;
        hs_clk_on = 1'b0;
        #100ns;
        check(bufr_clr == 1'b1, "bufr_clr must re-assert on clock LP-11");
        check(rx_clk_active_byte == 1'b0, "rx_clk_active must async-clear on gate");
        check(serdes_rst_byte == 1'b1, "serdes_rst must async-assert on gate");
        check(hs_settled_byte == 1'b0, "hs_settled must clear on clock outage");

        // --- T6: restart lottery is deterministic now ----------------------------
        clock_lane_hs_entry();
        #(BYTE_PERIOD_NS * 4);
        check(rx_clk_active_byte == 1'b1, "rx_clk_active must rise after re-lock");
        check(sts_lock_cnt == 8'd2, "lock_cnt must count re-lock");
        data_burst_entry();
        data_lp = 2'b11;

        // --- T7: cold-attach escape (continuous clock, no LP-11 ever) ------------
        clk_lp = 2'b00;                  // sensor free-runs HS clock
        hs_clk_on = 1'b1;
        ctl_aresetn = 1'b0;              // FPGA "reconfigured"
        repeat (5) @(posedge ctl_clk);
        ctl_aresetn = 1'b1;
        // must lock via T_INIT_FORCE escape: force timeout + clk settle
        wait (bufr_clr == 1'b0);
        check($realtime > 0, "escape reached");
        #(BYTE_PERIOD_NS * 4);
        check(rx_clk_active_byte == 1'b1, "rx_clk_active after cold-attach escape");
        // data lane still cycles per burst
        data_lp = 2'b11; #100ns;
        data_burst_entry();

        // --- T8: continuous lock must HOLD; escape must not re-fire while locked -
        // After T7 the supervisor is locked on a continuous clock (clk_lp=00). The
        // generalised escape counts only in waiting states, so a healthy lock must
        // survive a long continuous hold without a spurious re-lock (lock_cnt stays
        // at the T7 value of 3).
        data_lp = 2'b11; #60ns;
        clk_lp = 2'b00;                       // continuous, no clock-lane gating
        hs_clk_on = 1'b1;
        #200ns;
        lock_snapshot = sts_lock_cnt;         // count once settled into the lock
        #(T_INIT_FORCE_US_TB * 1000 * 2);     // hold > 2x T_INIT_FORCE
        check(sts_clk_state == 3'd4, "must stay CK_HS_CLK during continuous hold");
        check(rx_clk_active_byte == 1'b1, "rx_clk_active stays high in continuous lock");
        check(sts_lock_cnt == lock_snapshot, "escape must not re-fire while already locked");

        // --- T9: data-lane-driven lock (continuous-clock fix) --------------------
        // clk_lp is held at 10 (never 11 for a normal entry, never a stable 00 for
        // the clk_lp escape), so ONLY the data-lane HS path can lock the clock.
        ctl_aresetn = 1'b0;
        clk_lp = 2'b10;
        data_lp = 2'b11;
        hs_clk_on = 1'b1;
        repeat (5) @(posedge ctl_clk);
        ctl_aresetn = 1'b1;
        wait (sts_data_state == 3'd2);        // DT_STOP (data lane saw stop)
        #20ns;
        data_lp = 2'b01; #60ns;               // HS-request -> DT_HS_RQST
        data_lp = 2'b00;                       // HS
        wait (bufr_clr == 1'b0);
        check(sts_clk_state == 3'd4, "data-lane HS locked clock (CK_HS_CLK) with clk_lp!=00");
        #(BYTE_PERIOD_NS * 4);
        check(rx_clk_active_byte == 1'b1, "rx_clk_active after data-lane-driven lock");

        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED: %0d errors", errors);
        $finish;
    end

    initial begin
        #500us;
        $display("TEST FAILED: TB watchdog timeout");
        $finish;
    end

`ifdef TB_DEBUG
    initial begin
        forever begin
            #100ns;
            $display("DBG t=%0t clk_lp=%b lp_f=%b ck=%0d dt=%0d bufr_clr=%b act=%b settled=%b",
                     $time, clk_lp, dut.clk_lp_f, sts_clk_state, sts_data_state,
                     bufr_clr, rx_clk_active_byte, hs_settled_byte);
        end
    end
`endif

endmodule

`default_nettype wire
