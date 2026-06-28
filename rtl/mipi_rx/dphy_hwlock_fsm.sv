// dphy_hwlock_fsm.sv (2026-06-19) -- HW deterministic-lock FSM
// ---------------------------------------------------------------------------
// Ports the software `lock_mode` (scripts/bitslip_lock.py) -- the 8x8 bitslip
// sweep + /4-phase BUFR.CLR re-roll-on-fail + hold -- into RTL, so a bare
// bitstream auto-locks on power-up with NO PYNQ script (Xilinx-IP-equivalent
// power-on behaviour). Continuous (0x14) only; gated's per-vblank re-lock is a
// structural dead-end (diary_20260618).
//
// CLOCK DOMAIN: runs on the always-on refclk_200 (the supervisor's ctl_clk),
// because the /4 re-roll = BUFR.CLR resets the BUFR that generates byte_clk; an
// FSM on byte_clk would reset its own clock mid-re-roll. The top muxes this
// FSM's bitslip target over the GPIO path when cfg_hw_lock=1 and ORs `bufr_clr`
// into the probe BUFR.CLR; lock quality arrives as the 1-bit `hdr_active`
// (a byte_clk windowed sync-header detector synced up to refclk_200).
//
// Opt-in via `enable` (cfg_hw_lock). enable=0 -> FSM idle, GPIO/lock_mode path
// drives unchanged (fallback). Defaults make a sweep ~0.5 s; override the *_CYC
// params small in simulation.
module dphy_hwlock_fsm #(
    parameter int unsigned SETTLE_MIN_CYC = 32'd80_000,   // refclk_200 cyc to wait after a combo change (retrain-walk + skip the stale window) before trusting hdr_active
    parameter int unsigned SETTLE_CYC     = 32'd200_000,  // max refclk_200 cyc per combo (~1 ms; a few byte_clk header windows)
    parameter int unsigned REROLL_CYC     = 32'd4_000,    // BUFR.CLR pulse width + byte_clk restart settle (~20 us)
    parameter int unsigned LOST_CYC       = 32'd1_000_000,// hdr_active low this long in HOLD -> re-lock (~5 ms)
    parameter int unsigned RETRY_CYC      = 32'd2_000_000,// in FAILED, re-attempt the whole sweep after this (~10 ms) -- so a boot-enable before the chip streams does NOT stick; it keeps trying until the stream is up
    parameter int unsigned MAX_REROLL     = 4'd8
)(
    input  wire        clk,         // refclk_200
    input  wire        rst_n,
    input  wire        enable,      // cfg_hw_lock (synced to refclk_200)
    input  wire        hdr_active,  // 1-bit lock quality (synced from byte_clk)

    output wire  [2:0] bitslip_p0,  // swept lane-0 bitslip target
    output wire  [2:0] bitslip_p1,  // swept lane-1 bitslip target
    output logic       bufr_clr,    // /4 re-roll pulse (level, held REROLL_CYC)
    output logic       locked,      // a clean lock is held
    output logic       failed,      // gave up after MAX_REROLL (software fallback)

    output wire  [2:0] dbg_state,   // FSM state (HW verify, page 0x2e)
    output wire  [3:0] dbg_reroll,  // re-roll count
    output wire  [5:0] dbg_combo    // current bitslip combo index
);
    typedef enum logic [2:0] {
        S_IDLE   = 3'd0,
        S_SWEEP  = 3'd1,
        S_REROLL = 3'd2,
        S_HOLD   = 3'd3,
        S_FAILED = 3'd4
    } state_t;

    state_t       state;
    logic [5:0]   combo;   // combo[5:3] = lane-0 bitslip, combo[2:0] = lane-1 bitslip (sweeps p1 fastest)
    logic [31:0]  timer;
    logic [3:0]   reroll;

    assign bitslip_p0 = combo[5:3];
    assign bitslip_p1 = combo[2:0];
    assign dbg_state  = state;
    assign dbg_reroll = reroll;
    assign dbg_combo  = combo;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            combo    <= 6'd0;
            timer    <= 32'd0;
            reroll   <= 4'd0;
            bufr_clr <= 1'b0;
            locked   <= 1'b0;
            failed   <= 1'b0;
        end else begin
            bufr_clr <= 1'b0;                    // default low; pulsed only in S_REROLL
            case (state)
                S_IDLE: begin
                    locked <= 1'b0;
                    failed <= 1'b0;
                    if (enable) begin
                        combo  <= 6'd0;
                        reroll <= 4'd0;
                        timer  <= 32'd0;
                        state  <= S_SWEEP;
                    end
                end

                S_SWEEP: begin
                    if (!enable) begin
                        state <= S_IDLE;
                    end else if (timer < SETTLE_MIN_CYC) begin
                        timer <= timer + 32'd1;          // let the byte_clk retrain walk to the new combo + skip the stale window
                    end else if (hdr_active) begin
                        locked <= 1'b1;                  // clean header stream at this combo -> lock
                        failed <= 1'b0;                  // clear any prior failed-retry flag
                        timer  <= 32'd0;
                        state  <= S_HOLD;
                    end else if (timer >= SETTLE_CYC) begin
                        timer <= 32'd0;
                        if (combo == 6'd63) state <= S_REROLL;   // exhausted all 64 bitslip combos -> re-roll /4 phase
                        else                combo <= combo + 6'd1;
                    end else begin
                        timer <= timer + 32'd1;
                    end
                end

                S_REROLL: begin
                    bufr_clr <= 1'b1;                    // hold BUFR.CLR -> re-roll the /4 byte phase + restart byte_clk
                    if (timer >= REROLL_CYC) begin
                        bufr_clr <= 1'b0;
                        timer    <= 32'd0;
                        combo    <= 6'd0;
                        if (reroll >= MAX_REROLL) begin
                            state <= S_FAILED;
                        end else begin
                            reroll <= reroll + 4'd1;
                            state  <= S_SWEEP;
                        end
                    end else begin
                        timer <= timer + 32'd1;
                    end
                end

                S_HOLD: begin
                    if (!enable) begin
                        state <= S_IDLE;
                    end else if (hdr_active) begin
                        timer <= 32'd0;                  // healthy -> reset the lost-link timer
                    end else if (timer >= LOST_CYC) begin
                        locked <= 1'b0;                  // link collapsed -> re-lock from scratch
                        timer  <= 32'd0;
                        combo  <= 6'd0;
                        reroll <= 4'd0;
                        state  <= S_SWEEP;
                    end else begin
                        timer <= timer + 32'd1;
                    end
                end

                S_FAILED: begin
                    failed <= 1'b1;                      // no clean lock found this sweep cycle
                    if (!enable) begin
                        state <= S_IDLE;
                    end else if (timer >= RETRY_CYC) begin
                        // keep trying: a boot-enable before the chip streams (no
                        // headers) lands here -> re-attempt the whole sweep so it
                        // locks as soon as the stream comes up. `failed` stays set
                        // until a lock clears it (S_SWEEP), so 0x2e shows "struggling".
                        timer  <= 32'd0;
                        combo  <= 6'd0;
                        reroll <= 4'd0;
                        state  <= S_SWEEP;
                    end else begin
                        timer <= timer + 32'd1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
