// SPDX-License-Identifier: MIT
// Portions derived from the Digilent MIPI D-PHY Receiver IP (DPHY_LaneSCNN /
// HS_Clocking / DPHY_LaneSFEN), Copyright (c) 2016 Digilent, MIT License
// (Author: Elod Gyorgy). Full notice: THIRD_PARTY_NOTICES.md.
`timescale 1ns / 1ps
`default_nettype none

// D-PHY clock/data lane supervisor — port of the Digilent MIPI_DPHY_Receiver
// mechanisms (DPHY_LaneSCNN + HS_Clocking no-MMCM path + DPHY_LaneSFEN) that
// the custom frontend lacked. Root cause it fixes (diary 2026-06-12): without
// BUFR phase realignment and HS-SETTLE gating, every clock-lane restart is a
// ~3% lock lottery, so only ~1 frame/s survives ("3% capture").
//
// All timing is measured on the free-running ctl_clk (200 MHz IDELAY refclk)
// so it is independent of the (gated) HS clock. Spec constants, no AXI knobs.
//
// Outputs and their consumers:
//   bufr_clr           -> BUFR.CLR (OR with !rst_n at the consumer). Held
//                         whenever the clock-lane FSM is not in HS_CLK, so the
//                         /4 divider phase realigns deterministically at every
//                         clock-lane restart (HS_Clocking.vhd cBUFR_Rst).
//   serdes_rst_byte    -> ISERDES RST (byte domain, async assert / sync
//                         release while CLKDIV is alive again).
//   rx_clk_active_byte -> byte-domain "divided clock alive" flag
//                         (SyncAsyncLocked pattern: D=1, async clear by
//                         bufr_clr, clocked by byte_clk).
//   hs_settled_byte    -> data-lane HS-SETTLE (85 ns) elapsed; gate SoT
//                         hunting with this so HS-prepare garbage is never
//                         examined (DPHY_LaneSFEN cHSSettled / dSettled).
//                         1FF sync per Digilent: T_HS_SETTLE_max budget is
//                         tight, use only in simple combinational logic.
module dphy_lane_supervisor #(
    parameter int CTL_CLK_HZ      = 200_000_000,
    parameter int T_CLK_SETTLE_NS = 95,        // D-PHY T_CLK_SETTLE min
    parameter int T_HS_SETTLE_NS  = 85,        // D-PHY T_HS_SETTLE min
    parameter int T_LP_GLITCH_NS  = 20,        // D-PHY T_MIN_RX
    parameter int T_INIT_US       = 100,       // D-PHY T_INIT master
    parameter int T_INIT_FORCE_US = 1000       // continuous-clock cold-attach escape
) (
    input  wire        ctl_clk,
    input  wire        ctl_aresetn,
    input  wire [1:0]  clk_lp,                 // raw clock-lane {LP_p, LP_n}
    input  wire [1:0]  data_lp,                // raw lane-0 {LP_p, LP_n} (kLPFromLane0)
    input  wire        byte_clk,               // BUFR /4 output
    // Runtime clock-lane settle count (2026-06-18, ctl_clk domain). Governs WHEN
    // byte_clk starts after a clock-lane restart (CK_HS_TERM->CK_HS_CLK gate =
    // bufr_clr release). Sweeping this is the gated FS-recovery test: too long ->
    // byte_clk starts after the vblank-exit FS (FS lost); too short -> byte_clk on
    // an unstable clock. 0 = use the build-time T_CLK_SETTLE_CYC default.
    input  wire [7:0]  cfg_clk_settle_cyc = 8'd0,
    output logic       bufr_clr,
    output logic       rx_clk_active_byte,
    output logic       serdes_rst_byte,
    output logic       hs_settled_byte,
    output logic [2:0] sts_clk_state,
    output logic [2:0] sts_data_state,
    output logic [7:0] sts_lock_cnt,           // bufr_clr release events
    output logic [7:0] sts_settle_cnt,         // hs_settled rise events
    output logic [7:0] sts_lost_cnt            // HS-clock-lost (cHSClkLost) events
);

    localparam int T_CLK_SETTLE_CYC = (T_CLK_SETTLE_NS * (CTL_CLK_HZ / 1_000_000) + 999) / 1000;
    localparam int T_HS_SETTLE_CYC  = (T_HS_SETTLE_NS  * (CTL_CLK_HZ / 1_000_000) + 999) / 1000;
    localparam int T_LP_GLITCH_CYC  = (T_LP_GLITCH_NS  * (CTL_CLK_HZ / 1_000_000) + 999) / 1000;
    localparam int T_INIT_CYC       = T_INIT_US * (CTL_CLK_HZ / 1_000_000);
    localparam int T_INIT_FORCE_CYC = T_INIT_FORCE_US * (CTL_CLK_HZ / 1_000_000);
    localparam int INIT_CW          = $clog2(T_INIT_FORCE_CYC + 1);

    // Reset synchronizer: ctl_aresetn comes from another clock domain
    // (rst_n && ref_pll_locked, in the sysclk tree). Assert asynchronously but
    // release synchronously to ctl_clk, so the FSMs never start from a
    // metastable reset-release. Without this the supervisor was stuck in
    // CK_INIT on hardware (reset-CDC violation, diary 2026-06-12).
    logic ctl_rst;
    dphy_reset_bridge u_ctl_rst_sync (
        .clk     (ctl_clk),
        .arst    (!ctl_aresetn),
        .rst_out (ctl_rst)
    );

    // ------------------------------------------------------------------
    // LP pin synchronisation + glitch filtering (per bit; HS-entry changes
    // one bit per LPX period, HS-exit only uses the "11" condition)
    // ------------------------------------------------------------------
    logic [1:0] clk_lp_raw_sync;
    logic [1:0] data_lp_raw_sync;
    logic [1:0] clk_lp_f;
    logic [1:0] data_lp_f;

    for (genvar i = 0; i < 2; i++) begin : gen_lp_sync
        dphy_sync_2ff sync_clk_lp (
            .clk (ctl_clk), .arst(1'b0), .d(clk_lp[i]),  .q(clk_lp_raw_sync[i]));
        dphy_glitch_filter #(.STABLE_CYCLES(T_LP_GLITCH_CYC)) u_clk_lp_filt (
            .clk (ctl_clk), .rst(ctl_rst), .d(clk_lp_raw_sync[i]), .q(clk_lp_f[i]));
        dphy_sync_2ff sync_data_lp (
            .clk (ctl_clk), .arst(1'b0), .d(data_lp[i]), .q(data_lp_raw_sync[i]));
        dphy_glitch_filter #(.STABLE_CYCLES(T_LP_GLITCH_CYC)) u_data_lp_filt (
            .clk (ctl_clk), .rst(ctl_rst), .d(data_lp_raw_sync[i]), .q(data_lp_f[i]));
    end

    // ------------------------------------------------------------------
    // Clock-lane FSM (DPHY_LaneSCNN minus ULPS)
    // ------------------------------------------------------------------
    typedef enum logic [2:0] {
        CK_INIT, CK_STOP, CK_HS_PRPR, CK_HS_TERM, CK_HS_CLK, CK_HS_END
    } clk_state_t;
    clk_state_t ck_state, ck_nstate;

    logic [INIT_CW-1:0] esc_cnt;               // global cold-attach escape timer
    logic [7:0]         ck_settle_cnt;
    logic               ck_settle_tout;
    logic               esc_tout;
    logic [7:0]         lost_cnt;
    logic               data_hs_active;        // data lane in HS (clock present)

    // rx_clk_active (byte domain) re-synced into ctl_clk so the clock-lane FSM
    // can see a *clock-loss* event (Digilent DPHY_LaneSCNN cHSClkLost). Without
    // this the FSM only leaves CK_HS_CLK on LP-11, so a continuous clock that
    // glitches/drops mid-stream (e.g. chip_init resets) leaves the FSM falsely
    // locked with a stale BUFR phase -> silence (diary 2026-06-14 Phase 1).
    (* ASYNC_REG = "TRUE" *) logic rx_active_meta, rx_active_ctl;
    logic rx_active_ctl_q;
    logic clk_lost_ctl;
    always_ff @(posedge ctl_clk) begin
        if (ctl_rst) begin
            rx_active_meta <= 1'b0;
            rx_active_ctl  <= 1'b0;
            rx_active_ctl_q<= 1'b0;
        end else begin
            rx_active_meta <= rx_clk_active_byte;
            rx_active_ctl  <= rx_active_meta;
            rx_active_ctl_q<= rx_active_ctl;
        end
    end
    assign clk_lost_ctl = rx_active_ctl_q && !rx_active_ctl;   // divided-clock fall

    // The active lock sequence (CK_HS_PRPR/TERM/CLK). The escape timer counts
    // only OUTSIDE this set so it never fights the CK_HS_TERM->CK_HS_CLK step.
    logic ck_lock_progress;
    assign ck_lock_progress = (ck_state == CK_HS_PRPR)
                           || (ck_state == CK_HS_TERM)
                           || (ck_state == CK_HS_CLK);

    always_comb begin
        ck_nstate = ck_state;
        unique case (ck_state)
            CK_INIT:    if (clk_lp_f == 2'b11)                    ck_nstate = CK_STOP;
            CK_STOP:    if (clk_lp_f == 2'b01)                    ck_nstate = CK_HS_PRPR;
            CK_HS_PRPR: begin
                if (clk_lp_f == 2'b11)                            ck_nstate = CK_STOP;
                else if (clk_lp_f == 2'b00)                       ck_nstate = CK_HS_TERM;
            end
            CK_HS_TERM: begin
                if (clk_lp_f == 2'b11)                            ck_nstate = CK_STOP;
                else if (clk_lp_f == 2'b00 && ck_settle_tout)     ck_nstate = CK_HS_CLK;
            end
            CK_HS_CLK:  if (clk_lp_f == 2'b11)                    ck_nstate = CK_STOP;
            CK_HS_END:  if (clk_lp_f == 2'b11)                    ck_nstate = CK_STOP;
            default:                                              ck_nstate = CK_INIT;
        endcase
        // NOTE: clk_lost_ctl is observed via lost_cnt only. Acting on it
        // (CK_HS_CLK->CK_INIT) caused churn that stranded the FSM in CK_INIT in
        // continuous mode (diary 2026-06-14 Phase 5); the data-lane path below
        // re-locks robustly instead, so the explicit clock-loss exit is dropped.

        // (b) Legacy clk_lp==00 cold-attach timeout (fallback when no data yet).
        if (!ck_lock_progress && esc_tout)                       ck_nstate = CK_HS_TERM;

        // (a) Data-lane-driven lock -- the continuous-clock fix. If the data lane
        // is in HS (DT_HS_*), the HS clock is DEFINITELY running, so drive the
        // lock to completion ignoring the clock-lane 00<->01 chatter (which in
        // continuous mode never gives a stable 00, so the clk_lp path stalls in
        // CK_INIT -- the Phase-5 stall). It both starts the lock from a waiting
        // state AND holds it across that chatter while data keeps arriving.
        // BUT a clock-lane LP-11 is an unambiguous STOP (clock genuinely gated),
        // so yield to it -- the normal case above already took CK_HS_CLK->CK_STOP.
        // That also keeps the per-line gate working in gated mode.
        if (data_hs_active && clk_lp_f != 2'b11) begin
            case (ck_state)
                CK_HS_TERM: ck_nstate = ck_settle_tout ? CK_HS_CLK : CK_HS_TERM;
                CK_HS_CLK:  ck_nstate = CK_HS_CLK;
                CK_HS_PRPR: ck_nstate = CK_HS_TERM;
                default:    ck_nstate = CK_HS_TERM;   // CK_INIT / CK_STOP / CK_HS_END
            endcase
        end
    end

    always_ff @(posedge ctl_clk) begin
        if (ctl_rst) begin
            ck_state      <= CK_INIT;
            esc_cnt       <= '0;
            ck_settle_cnt <= '0;
            lost_cnt      <= '0;
            bufr_clr      <= 1'b1;
        end else begin
            ck_state <= ck_nstate;
            // Escape timer: count while the HS clock is present (clk_lp==00) but
            // the FSM is stuck in a waiting state (not progressing toward lock).
            // Resets the moment we enter the lock sequence so it cannot stall the
            // CK_HS_TERM->CK_HS_CLK step.
            if (!ck_lock_progress && (clk_lp_f == 2'b00)) begin
                if (!esc_tout) esc_cnt <= esc_cnt + 1'b1;
            end else begin
                esc_cnt <= '0;
            end
            if (ck_state == CK_HS_TERM) begin
                if (!ck_settle_tout) ck_settle_cnt <= ck_settle_cnt + 1'b1;
            end else begin
                ck_settle_cnt <= '0;
            end
            if (ck_state == CK_HS_CLK && clk_lost_ctl && lost_cnt != 8'hff) begin
                lost_cnt <= lost_cnt + 8'd1;
            end
            // Registered on ctl_clk for a glitch-free BUFR CLR
            // (HS_Clocking.vhd: cBUFR_Rst <= cExtRst when Rising_Edge(CtlClk))
            bufr_clr <= (ck_nstate != CK_HS_CLK);
        end
    end

    // Runtime-overridable clock-lane settle: cfg_clk_settle_cyc!=0 uses the runtime
    // value (gated FS-recovery sweep), else the build-time spec-min default.
    wire [7:0] clk_settle_eff = (cfg_clk_settle_cyc != 8'd0)
                              ? cfg_clk_settle_cyc : 8'(T_CLK_SETTLE_CYC);
    assign ck_settle_tout = (ck_settle_cnt >= (clk_settle_eff - 8'd1));
    assign esc_tout       = (esc_cnt >= INIT_CW'(T_INIT_FORCE_CYC - 1));

    // Byte-domain "divided clock alive" flag (HS_Clocking SyncAsyncLocked)
    dphy_sync_2ff sync_rx_clk_active (
        .clk (byte_clk),
        .arst(bufr_clr),
        .d   (1'b1),
        .q   (rx_clk_active_byte)
    );

    // ISERDES reset: held while BUFR is cleared, released 2 byte_clk edges
    // after the divided clock is back (Xilinx: release RST synchronous to a
    // live CLKDIV).
    dphy_reset_bridge bridge_serdes_rst (
        .clk    (byte_clk),
        .arst   (bufr_clr),
        .rst_out(serdes_rst_byte)
    );

    // ------------------------------------------------------------------
    // Data-lane FSM (DPHY_LaneSFEN)
    // ------------------------------------------------------------------
    typedef enum logic [2:0] {
        DT_INIT, DT_WAIT_STOP, DT_STOP, DT_HS_RQST, DT_HS_SETTLE, DT_HS_RCV
    } data_state_t;
    data_state_t dt_state, dt_nstate;

    localparam int DT_CW = $clog2(T_INIT_CYC + 1);
    logic [DT_CW-1:0] dt_delay_cnt;
    logic             dt_init_tout;
    logic             dt_settle_tout;
    logic             dt_cnt_en;
    logic             hs_settled;

    always_comb begin
        dt_nstate = dt_state;
        unique case (dt_state)
            DT_INIT:      if (dt_init_tout)            dt_nstate = DT_WAIT_STOP;
            DT_WAIT_STOP: if (data_lp_f == 2'b11)      dt_nstate = DT_STOP;
            DT_STOP:      if (data_lp_f == 2'b01)      dt_nstate = DT_HS_RQST;
            DT_HS_RQST: begin
                if (data_lp_f == 2'b11)                dt_nstate = DT_STOP;
                else if (data_lp_f == 2'b00)           dt_nstate = DT_HS_SETTLE;
            end
            DT_HS_SETTLE: begin
                if (data_lp_f == 2'b11)                dt_nstate = DT_STOP;
                else if (dt_settle_tout)               dt_nstate = DT_HS_RCV;
            end
            DT_HS_RCV:    if (data_lp_f == 2'b11)      dt_nstate = DT_STOP;
            default:                                   dt_nstate = DT_INIT;
        endcase
    end

    // Counter enabled by NEXT state for settle so it starts at settle entry
    // (DPHY_LaneSFEN: cDelayCntEn = stInitCountDown or nstate = stHS_Settle)
    assign dt_cnt_en      = (dt_state == DT_INIT) || (dt_nstate == DT_HS_SETTLE);
    assign dt_init_tout   = (dt_delay_cnt >= DT_CW'(T_INIT_CYC - 1));
    assign dt_settle_tout = (dt_delay_cnt >= DT_CW'(T_HS_SETTLE_CYC - 1));

    // Data lane in HS (request/settle/receive) => the HS clock is present.
    // The clock-lane FSM uses this to lock even when the clock-lane LP never
    // gives a stable 00 (continuous-clock fix, diary 2026-06-14 Phase 5). No
    // combinational loop: dt_state depends on data_lp_f, not on ck_state.
    assign data_hs_active = (dt_state == DT_HS_RQST)
                         || (dt_state == DT_HS_SETTLE)
                         || (dt_state == DT_HS_RCV);

    always_ff @(posedge ctl_clk) begin
        if (ctl_rst) begin
            dt_state     <= DT_INIT;
            dt_delay_cnt <= '0;
        end else begin
            dt_state <= dt_nstate;
            if (!dt_cnt_en) dt_delay_cnt <= '0;
            else            dt_delay_cnt <= dt_delay_cnt + 1'b1;
        end
    end

    // cHSSettled: set at settle timeout, cleared on clock outage or stop
    logic clk_outage_ctl;
    dphy_reset_bridge bridge_clk_outage (
        .clk    (ctl_clk),
        .arst   (!rx_clk_active_byte),
        .rst_out(clk_outage_ctl)
    );

    always_ff @(posedge ctl_clk) begin
        if (ctl_rst || clk_outage_ctl) begin
            hs_settled <= 1'b0;
        end else if (dt_state == DT_HS_SETTLE && dt_settle_tout) begin
            hs_settled <= 1'b1;
        end else if (dt_state == DT_STOP || dt_state == DT_WAIT_STOP) begin
            hs_settled <= 1'b0;
        end
    end

    // 1-stage sync into byte domain (Digilent dSettled: budget is tight)
    (* ASYNC_REG = "TRUE" *) logic sync_settled_byte;
    always_ff @(posedge byte_clk or posedge bufr_clr) begin
        if (bufr_clr) sync_settled_byte <= 1'b0;
        else          sync_settled_byte <= hs_settled;
    end
    assign hs_settled_byte = sync_settled_byte;

    // ------------------------------------------------------------------
    // Status
    // ------------------------------------------------------------------
    logic bufr_clr_q;
    logic settled_q;
    always_ff @(posedge ctl_clk) begin
        if (ctl_rst) begin
            sts_lock_cnt   <= '0;
            sts_settle_cnt <= '0;
            bufr_clr_q     <= 1'b1;
            settled_q      <= 1'b0;
        end else begin
            bufr_clr_q <= bufr_clr;
            settled_q  <= hs_settled;
            if (bufr_clr_q && !bufr_clr)  sts_lock_cnt   <= sts_lock_cnt + 8'd1;
            if (!settled_q && hs_settled) sts_settle_cnt <= sts_settle_cnt + 8'd1;
        end
    end

    assign sts_clk_state  = ck_state;
    assign sts_data_state = dt_state;
    assign sts_lost_cnt   = lost_cnt;

endmodule

`default_nettype wire
