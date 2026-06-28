`timescale 1ns / 1ps

module csi2_frame_state #(
    parameter int MAX_LINES = 4096,
    parameter bit GUARD_FRAME_LINES = 1'b0,
    parameter int EXPECTED_FRAME_LINES = 0,
    parameter logic [15:0] EXPECTED_LINE_WC = 16'd0,
    // FS plausibility floor (lsle mode): an in-frame FS is accepted as a real
    // frame boundary only after at least FS_MIN_LINES lines have been seen in
    // the current frame. An FS arriving earlier is a SPURIOUS FS (e.g. payload
    // 0x00 runs / black pixels decoding as a DI=0x00,WC=0,ECC=0 short packet on
    // the open-loop byte aligner) and is ignored, so it cannot chop the frame
    // into short pieces that then stack in the fixed-height VDMA buffer. Real
    // frames are delimited by a plausible FS in [FS_MIN_LINES, MAX_LINES].
    // 0 = disabled (accept any FS, original behaviour).
    parameter int FS_MIN_LINES = 0,
    // FE-DELIMITER mode (lsle, 2026-06-04): when set, the frame is delimited by
    // the chip's FE short packet (natural CSI-2 FS-opens / FE-closes), and FS is
    // used only to (re-)anchor the frame top. Enabled because on a stable AEC the
    // chip emits balanced FS==FE (observed 39==39) so FE is a reliable bottom
    // marker; closing on FE at the true ~480-line boundary phase-locks the VDMA
    // buffer (vs. the FS-anchor path where most frames hit the MAX_LINES cap and
    // stack). FE earlier than FS_MIN_LINES is treated as spurious and ignored;
    // a dropped FE is still bounded by the MAX_LINES runaway cap.
    // 0 = FS-anchor mode (force-close on plausible FS, swallow FE).
    parameter bit FE_DELIMITS = 1'b0,
    // FE plausibility floor (FE_DELIMITS mode, 2026-06-15). An FE arriving
    // before FE_MIN_LINES is a SPURIOUS early FE (false DT=0x01 short packet on
    // the open-loop byte aligner) and is rejected. Hardware evidence: the only
    // FE per frame fires at ~line 441 (fe_after_480 ~= 0), closing the frame
    // ~30-40 lines short while ~480 long packets actually reach the parser; the
    // lost tail shows up as long_before_fs (~28/frame). With FE_MIN_LINES set
    // near EXPECTED_FRAME_LINES the spurious FE is ignored and the frame instead
    // closes on the real next-frame FS (>= FE_MIN_LINES) or the MAX_LINES cap,
    // capturing the full ~480 lines. 0 = legacy (FE closes at the FS_MIN_LINES
    // floor; an in-frame FS is swallowed).
    parameter int FE_MIN_LINES = 0
) (
    input  logic        core_clk,
    input  logic        core_aresetn,

    input  logic        cfg_use_lsle,
    input  logic [15:0] cfg_expected_frame_lines,
    // SOF-synthesis (opt-in): open a frame from the first in-IDLE LS when the
    // chip's FS short packet never arrives (D-PHY lane supervisor enabled:
    // fs=0 but ls/fe present, diary 2026-06-13). Default 0 = legacy (FS opens).
    input  logic        cfg_sof_synth = 1'b0,
    // FORCE-EXPECTED height (opt-in, 2026-06-16): in lsle mode, force-close the
    // frame at EXACTLY cfg_expected_frame_lines (e.g. 480) so the VDMA/VTC sees a
    // CONSTANT-height frame and genlock can phase-lock (the live-HDMI roll fix).
    // The chip's FS still re-anchors the top each frame (anti-drift); overshoot
    // lines (chip emits ~495 LE: 480 image + embedded) drain in IDLE until the
    // next FS reopens. Diagnostic (diary 2026-06-16): ~480 lines reach the parser
    // and frame_state already closes >=480 (modal 495), but the VARIABLE height +
    // merges break genlock; clamping to a fixed height regularises the SOF cadence.
    // Default 0 = legacy (close on FE/next-FS, variable height).
    input  logic        cfg_force_expected = 1'b0,
    // LONG-AS-LINE (opt-in, 2026-06-17): in lsle mode, deliver a long packet that
    // arrives WITHOUT a preceding LS as a row anyway (open the line on the long,
    // synthesise the SOL) instead of rejecting it. Recovers the ~14 no-LS-reject
    // rows (the residual bottom band) caused by scattered LS short-packet loss
    // upstream. SAFE: the VC/DT filter already gates embedded data, so every long
    // reaching frame_state is a DT=0x22 pixel row. Default 0 = legacy (reject).
    input  logic        cfg_long_as_line = 1'b0,

    input  logic [7:0]  in_pkt_di,
    input  logic [15:0] in_pkt_wc,
    input  logic        in_pkt_is_short,
    input  logic        in_pkt_is_long,
    input  logic        in_pkt_start,
    input  logic        in_pkt_end,
    input  logic        in_pkt_err,
    input  logic [7:0]  in_payload_data,
    input  logic        in_payload_valid,
    input  logic        in_payload_first,
    input  logic        in_payload_last,

    output logic        out_sof,
    output logic        out_eof,
    output logic        out_sol,
    output logic        out_eol,
    // diagnostic (2026-06-16): high while a frame window is open (ST_IN_FRAME).
    // Used by the top-level boundary trace to tag each packet accept/reject:
    // a long arriving with out_in_frame=0 is dropped (long_before_fs).
    output logic        out_in_frame,
    output logic [15:0] out_line_idx,
    output logic [7:0]  out_payload_data,
    output logic        out_payload_valid,
    output logic        out_payload_first,
    output logic        out_payload_last,
    output logic        out_frame_err,

    output logic [31:0] sts_frame_count,
    output logic [31:0] sts_line_count,
    output logic [15:0] sts_last_frame_lines,
    output logic [15:0] sts_frame_sync_err_cnt,
    // Long-packet disposition counters (2026-06-17, diagnostic): cumulative count
    // of long packets ACCEPTED as rows vs REJECTED for no preceding LS (line 475)
    // vs REJECTED while IDLE (state != IN_FRAME). Software takes deltas / frame to
    // localise the residual bottom band: if reject_nols ~= the row shortfall, the
    // band is the no-LS rejection (scattered LS drop); if accept ~= 480 the band
    // is downstream of frame_state.
    output logic [15:0] sts_dbg_long_accept,
    output logic [15:0] sts_dbg_long_nols,
    output logic [15:0] sts_dbg_long_idle,
    // no-LS reject position histogram (2026-06-17): 8 buckets of 64 lines each,
    // indexed by line_idx[8:6], incremented at the no-LS reject. Reveals WHERE in
    // the frame the LS drops cluster: bucket 0 high = frame-top / vblank-exit
    // (vsync-adjacent); flat = random/scattered. Packed {b7,...,b0}.
    output logic [127:0] sts_dbg_nols_hist
);

    localparam logic [5:0] DT_FS = 6'h00;
    localparam logic [5:0] DT_FE = 6'h01;
    localparam logic [5:0] DT_LS = 6'h02;
    localparam logic [5:0] DT_LE = 6'h03;
    localparam logic [15:0] EXPECTED_FRAME_LINES_PARAM_U16 = EXPECTED_FRAME_LINES;
    localparam logic [15:0] MAX_LINES_U16 = MAX_LINES;
    // Use runtime override when cfg_expected_frame_lines != 0; else fall back to parameter.
    wire [15:0] EXPECTED_FRAME_LINES_U16 =
        (cfg_expected_frame_lines != 16'd0) ? cfg_expected_frame_lines : EXPECTED_FRAME_LINES_PARAM_U16;

    typedef enum logic [0:0] {
        ST_IDLE,
        ST_IN_FRAME
    } frame_state_t;

    frame_state_t state;

    logic [15:0] line_idx;
    logic [15:0] dbg_nols_bucket [0:7];   // no-LS reject position histogram (line_idx[8:6])
    logic        frame_err_sticky;
    logic        current_long_active;
    logic        line_open;
    logic        sof_pending;
    logic        eof_pending;
    logic        frame_err_pending;
    logic        guard_wait_fe_after_complete;
    // SOF-synthesis phase resync (2026-06-13): after a MAX_LINES cap-close (FE
    // was dropped, so the phase is unknown) suppress the synthetic LS-open until
    // the next FE is seen. This guarantees every synthetic frame opens at the
    // chip's true frame top (FE -> first LS), removing the per-frame vertical
    // phase jitter that made the live HDMI image look defocused.
    logic        synth_wait_fe;

    function automatic [15:0] sat_inc16(input [15:0] value);
        if (value == 16'hffff) begin
            sat_inc16 = value;
        end else begin
            sat_inc16 = value + 16'd1;
        end
    endfunction

    function automatic [31:0] sat_inc32(input [31:0] value);
        if (value == 32'hffff_ffff) begin
            sat_inc32 = value;
        end else begin
            sat_inc32 = value + 32'd1;
        end
    endfunction

    wire [5:0] pkt_dt = in_pkt_di[5:0];
    wire       is_fs  = in_pkt_is_short && (pkt_dt == DT_FS);
    wire       is_fe  = in_pkt_is_short && (pkt_dt == DT_FE);
    wire       is_ls  = in_pkt_is_short && (pkt_dt == DT_LS);
    wire       is_le  = in_pkt_is_short && (pkt_dt == DT_LE);
    wire       guard_line_mode = GUARD_FRAME_LINES && (EXPECTED_FRAME_LINES_U16 != 16'd0) && !cfg_use_lsle;
    wire       guard_line_wc_ok = (EXPECTED_LINE_WC == 16'd0) || (in_pkt_wc == EXPECTED_LINE_WC);
    wire       guard_expected_last_line = guard_line_mode && (line_idx == (EXPECTED_FRAME_LINES_U16 - 16'd1));
    wire       guard_line_count_reached = guard_line_mode && (line_idx >= EXPECTED_FRAME_LINES_U16);
    // LSLE line-count delimiter: in cfg_use_lsle mode the chip's FS/FE arrive with
    // unstable TIMING (FS late, FE early/missing — diary 20260530 Phase 6-10), which
    // the FSM otherwise amplifies into wild frame heights (182..1831) that slip the
    // image into the fixed-height VDMA buffer => the "stacked" artifact, or (when
    // delimited purely by line count) a free-running vertical "roll".
    // When this guard is active the FSM uses a HYBRID delimiter (diary 20260601):
    //   * FS (frame-start short packet) PHASE-ANCHORS the frame -- each FS force-
    //     closes the current frame and re-syncs line_idx=0, locking the frame top to
    //     the chip's true content boundary (stops the roll; lines are intact -- vgrad
    //     test corr=1.00). A normal mid-frame FS is the delimiter, not an error.
    //   * the per-line LE count is only a RUNAWAY SAFETY CAP at MAX_LINES, firing for
    //     frames whose FS the chip dropped (bounds the old 182..1831 merge).
    //   * FE is swallowed (the chip's FE timing is unreliable; FS defines frames).
    wire       lsle_line_guard = GUARD_FRAME_LINES && cfg_use_lsle && (EXPECTED_FRAME_LINES_U16 != 16'd0);
    localparam logic [15:0] FS_MIN_LINES_U16 = FS_MIN_LINES;
    // An in-frame FS only delimits a real frame once enough lines have elapsed;
    // earlier FS = spurious (ignored). FS_MIN_LINES=0 disables the floor.
    wire       fs_plausible = (FS_MIN_LINES_U16 == 16'd0) || (line_idx >= FS_MIN_LINES_U16);
    localparam logic [15:0] FE_MIN_LINES_U16 = FE_MIN_LINES;
    // FE close floor: when FE_MIN_LINES is set, an FE only closes the frame once
    // line_idx has reached it (rejecting the spurious early FE). When the floor
    // is also reached, a real next-frame FS is allowed to close+re-anchor in
    // FE_DELIMITS mode (recovering a lost real FE). FE_MIN_LINES=0 = legacy.
    // Guard the FE_MIN behaviour to LEGACY mode (cfg_sof_synth=0), where the
    // chip's FS short packet is reliably received so the lost-FE recovery
    // (fs_close_ok, close on the real next-frame FS) actually has an FS to fire
    // on. In SOF-synth mode the FS is lost (the D-PHY supervisor's per-gate
    // re-lock drops it), so rejecting the spurious early FE would leave NO close
    // and the frame would run to the MAX_LINES cap. So in synth mode fall back to
    // the legacy FE close (fs_plausible). 2026-06-15.
    wire       fe_min_active = (FE_MIN_LINES_U16 != 16'd0) && !cfg_sof_synth;
    wire       fe_plausible  = fe_min_active ? (line_idx >= FE_MIN_LINES_U16)
                                             : fs_plausible;
    wire       fs_close_ok   = fe_min_active && (line_idx >= FE_MIN_LINES_U16);

    // Registered force-close threshold/guard (2026-06-16, timing): keep the
    // EXPECTED-1 subtractor and the slow-changing config AND out of the LE-handler
    // critical path. cfg_force_expected / lsle_line_guard / EXPECTED are runtime
    // config (stable across frames), so a 1-cycle latency is irrelevant; the
    // force-close then reduces to reg-vs-reg compare + a registered enable.
    logic        force_guard_r;
    logic [15:0] expected_m1_r;
    always_ff @(posedge core_clk) begin
        if (!core_aresetn) begin
            force_guard_r <= 1'b0;
            expected_m1_r <= 16'hffff;
        end else begin
            force_guard_r <= cfg_force_expected && lsle_line_guard
                             && (EXPECTED_FRAME_LINES_U16 != 16'd0);
            expected_m1_r <= EXPECTED_FRAME_LINES_U16 - 16'd1;
        end
    end

    always_ff @(posedge core_clk) begin
        if (!core_aresetn) begin
            state                  <= ST_IDLE;
            line_idx               <= 16'h0000;
            frame_err_sticky       <= 1'b0;
            current_long_active    <= 1'b0;
            line_open              <= 1'b0;
            sof_pending            <= 1'b0;
            eof_pending            <= 1'b0;
            frame_err_pending      <= 1'b0;
            guard_wait_fe_after_complete <= 1'b0;
            synth_wait_fe          <= 1'b0;
            out_sof                <= 1'b0;
            out_eof                <= 1'b0;
            out_sol                <= 1'b0;
            out_eol                <= 1'b0;
            out_line_idx           <= 16'h0000;
            out_payload_data       <= 8'h00;
            out_payload_valid      <= 1'b0;
            out_payload_first      <= 1'b0;
            out_payload_last       <= 1'b0;
            out_frame_err          <= 1'b0;
            sts_frame_count        <= 32'h0000_0000;
            sts_line_count         <= 32'h0000_0000;
            sts_last_frame_lines   <= 16'h0000;
            sts_frame_sync_err_cnt <= 16'h0000;
            sts_dbg_long_accept    <= 16'h0000;
            sts_dbg_long_nols      <= 16'h0000;
            sts_dbg_long_idle      <= 16'h0000;
            for (int b = 0; b < 8; b++) dbg_nols_bucket[b] <= 16'h0000;
        end else begin
            automatic logic next_frame_err;
            automatic logic frame_closed;

            frame_closed = 1'b0;

            out_sof           <= 1'b0;
            out_eof           <= 1'b0;
            out_sol           <= 1'b0;
            out_eol           <= 1'b0;
            out_payload_valid <= 1'b0;
            out_payload_first <= 1'b0;
            out_payload_last  <= 1'b0;
            out_frame_err     <= 1'b0;

            next_frame_err = frame_err_sticky | (in_pkt_err && in_pkt_end);

            if (in_pkt_start) begin
                if (is_fs) begin
                    if (state == ST_IN_FRAME) begin
                        if (!guard_line_mode && !cfg_use_lsle) begin
                            sts_frame_sync_err_cnt <= sat_inc16(sts_frame_sync_err_cnt);
                            state               <= ST_IN_FRAME;
                            line_idx            <= 16'h0000;
                            frame_err_sticky    <= in_pkt_err;
                            current_long_active <= 1'b0;
                            line_open           <= 1'b0;
                            sof_pending         <= 1'b1;
                            eof_pending         <= 1'b0;
                            frame_err_pending   <= 1'b0;
                            guard_wait_fe_after_complete <= 1'b0;
                            out_sof             <= 1'b1;
                            out_line_idx        <= 16'h0000;
                        end else if (cfg_use_lsle && lsle_line_guard) begin
                            // FS-ANCHOR (hybrid, diary 20260601): the chip's FS marks
                            // the true frame top, so use it to PHASE-LOCK the frame
                            // (without this the free-running line count drifts and the
                            // image rolls — vgrad test corr=1.00 at varying shift).
                            // Force-close the current frame and re-sync line_idx=0 here;
                            // the new frame's SOF is raised via sof_pending on its first
                            // payload (kept out of this EOF cycle). The LE line-count
                            // below is now only a runaway safety cap (MAX_LINES) for the
                            // frames whose FS the chip dropped. A normal mid-frame FS is
                            // the expected delimiter, NOT a sync error.
                            // PLAUSIBILITY FLOOR: only accept the FS once enough lines
                            // have elapsed (fs_plausible). An FS arriving too early is a
                            // SPURIOUS FS (payload-zero false header on the open-loop
                            // aligner) and is ignored so it cannot chop the frame short.
                            if (FE_DELIMITS && fs_close_ok) begin
                                // LOST-FE RECOVERY (2026-06-15): the real next-frame FS
                                // after a FULL frame (line_idx >= FE_MIN_LINES) whose only
                                // FE was the spurious early one (already rejected by
                                // fe_plausible). This FS is the true frame boundary, so
                                // close+re-anchor here -- the frame captures its full
                                // ~480 lines instead of the spurious-FE short close. The
                                // FE_MIN_LINES floor keeps the early black-pixel false FS
                                // (handled below) from triggering this.
                                frame_closed         = 1'b1;
                                out_eof              <= 1'b1;
                                out_frame_err        <= next_frame_err;
                                sts_frame_count      <= sat_inc32(sts_frame_count);
                                sts_last_frame_lines <= line_idx;
                                line_idx             <= 16'h0000;
                                frame_err_sticky     <= 1'b0;
                                current_long_active  <= 1'b0;
                                line_open            <= 1'b0;
                                sof_pending          <= 1'b1;
                                eof_pending          <= 1'b0;
                                frame_err_pending    <= 1'b0;
                                guard_wait_fe_after_complete <= 1'b0;
                                out_line_idx         <= 16'h0000;
                            end else if (FE_DELIMITS) begin
                                // FE-DELIMITER mode: an EARLY in-frame FS (line_idx <
                                // FE_MIN_LINES) is IGNORED (no re-anchor, no SOF). A CSI-2
                                // FS short packet is 00 00 00 00 (DI=0,WC=0,ECC=0), which is
                                // indistinguishable from a run of payload-0x00 bytes (black
                                // pixels) on the open-loop byte aligner -> dark regions emit
                                // SPURIOUS in-frame FS (observed FS=7 vs FE=4). Re-anchoring
                                // on each of these raised an extra SOF and made the VDMA
                                // reset to the buffer top mid-frame, tiling the image
                                // (strong on dark subjects, weak on a flat bright field).
                                // Ignoring them keeps exactly one SOF/frame (opened by the
                                // FS-from-IDLE after the previous FE). A genuinely dropped
                                // FE is bounded by the MAX_LINES runaway cap below.
                                sts_frame_sync_err_cnt <= sat_inc16(sts_frame_sync_err_cnt);
                            end else if (fs_plausible) begin
                                frame_closed         = 1'b1;
                                out_eof              <= 1'b1;
                                out_frame_err        <= next_frame_err;
                                sts_frame_count      <= sat_inc32(sts_frame_count);
                                sts_last_frame_lines <= line_idx;
                                line_idx             <= 16'h0000;
                                frame_err_sticky     <= 1'b0;
                                current_long_active  <= 1'b0;
                                line_open            <= 1'b0;
                                sof_pending          <= 1'b1;
                                eof_pending          <= 1'b0;
                                frame_err_pending    <= 1'b0;
                                guard_wait_fe_after_complete <= 1'b0;
                                out_line_idx         <= 16'h0000;
                            end else begin
                                // spurious early FS: ignore, keep current frame open
                                sts_frame_sync_err_cnt <= sat_inc16(sts_frame_sync_err_cnt);
                            end
                        end else if (cfg_use_lsle) begin
                            sts_frame_sync_err_cnt <= sat_inc16(sts_frame_sync_err_cnt);
                            // FS while in-frame in LSLE mode (no line-count guard) =>
                            // the previous frame lost its FE. Force-close the stale
                            // frame (EOF + frame_err + accounting) and re-synchronise
                            // on this FS so a single dropped FE cannot merge two
                            // frames. SOF for the new frame is raised via sof_pending
                            // on its first payload (kept out of this EOF cycle).
                            // diary 20260530 Phase 11, sim-validated.
                            out_eof              <= 1'b1;
                            out_frame_err        <= 1'b1;
                            sts_frame_count      <= sat_inc32(sts_frame_count);
                            sts_last_frame_lines <= line_idx;
                            line_idx             <= 16'h0000;
                            frame_err_sticky     <= in_pkt_err;
                            current_long_active  <= 1'b0;
                            line_open            <= 1'b0;
                            sof_pending          <= 1'b1;
                            eof_pending          <= 1'b0;
                            frame_err_pending    <= 1'b0;
                            guard_wait_fe_after_complete <= 1'b0;
                            out_line_idx         <= 16'h0000;
                        end else begin
                            // guard_line_mode (non-lsle GUARD_FRAME_LINES) path: an
                            // in-frame FS is an unexpected overlap. Count it as a sync
                            // error and otherwise ignore it (line-count delimits here).
                            sts_frame_sync_err_cnt <= sat_inc16(sts_frame_sync_err_cnt);
                        end
                    end else begin
                        state               <= ST_IN_FRAME;
                        line_idx            <= 16'h0000;
                        frame_err_sticky    <= in_pkt_err;
                        current_long_active <= 1'b0;
                        line_open           <= 1'b0;
                        sof_pending         <= 1'b1;
                        eof_pending         <= 1'b0;
                        frame_err_pending   <= 1'b0;
                        guard_wait_fe_after_complete <= 1'b0;
                        out_sof             <= 1'b1;
                        out_line_idx        <= 16'h0000;
                    end
                end else if (is_fe) begin
                    if (state == ST_IN_FRAME) begin
                        if (lsle_line_guard && FE_DELIMITS) begin
                            // FE-DELIMITER mode (2026-06-04): the chip's FE is the
                            // natural frame bottom. Close the frame on a PLAUSIBLE FE
                            // (>= FE_MIN_LINES, or >= FS_MIN_LINES when FE_MIN_LINES=0)
                            // so the VDMA frame ends at the chip's true ~480-line
                            // boundary and phase-locks. An FE arriving too early is a
                            // spurious early FE (2026-06-15: observed ~line 441, closing
                            // the frame ~30-40 lines short) and is ignored; a dropped FE
                            // is recovered by the next FS (fs_close_ok) or the MAX_LINES
                            // cap.
                            // FORCE-EXPECTED (2026-06-16): when forcing a constant
                            // height, the line-count force-close (in the LE handler) is
                            // the SOLE closer, so IGNORE every FE here -- otherwise the
                            // spurious early FE (~446, plausible in synth mode where
                            // FE_MIN is off) closes the frame short before line 480, and
                            // the frame opens ~30 lines late on the next FS (the bottom
                            // band). Ignoring the FE lets synth re-open at the true top
                            // (FE-resync) and force-480 set the height.
                            if (fe_plausible && !cfg_force_expected) begin
                                frame_closed         = 1'b1;
                                out_eof              <= 1'b1;
                                out_frame_err        <= next_frame_err;
                                sts_frame_count      <= sat_inc32(sts_frame_count);
                                sts_last_frame_lines <= line_idx;
                                line_idx             <= 16'h0000;
                                state                <= ST_IDLE;
                                frame_err_sticky     <= 1'b0;
                                current_long_active  <= 1'b0;
                                line_open            <= 1'b0;
                                sof_pending          <= 1'b0;
                                eof_pending          <= 1'b0;
                                frame_err_pending    <= 1'b0;
                                guard_wait_fe_after_complete <= 1'b0;
                                // A normal FE close marks the true chip bottom, so the
                                // next LS is the correct phase top: clear any wait.
                                synth_wait_fe        <= 1'b0;
                            end else begin
                                // spurious early FE: ignore, keep frame open
                                sts_frame_sync_err_cnt <= sat_inc16(sts_frame_sync_err_cnt);
                            end
                        end else if (lsle_line_guard) begin
                            // FS-anchor mode: the chip's FE (early or trailing the
                            // previous LE-count close) is untrusted noise. Swallow it —
                            // never close the frame on FE — so a mis-timed FE cannot
                            // inject a short/empty frame. Counted as sync error only.
                            sts_frame_sync_err_cnt <= sat_inc16(sts_frame_sync_err_cnt);
                        end else if (guard_line_mode && (line_idx < EXPECTED_FRAME_LINES_U16)) begin
                            sts_frame_sync_err_cnt <= sat_inc16(sts_frame_sync_err_cnt);
                        end else begin
                            out_eof              <= 1'b1;
                            out_frame_err        <= next_frame_err;
                            eof_pending          <= 1'b1;
                            frame_err_pending    <= next_frame_err;
                            sts_frame_count      <= sat_inc32(sts_frame_count);
                            sts_last_frame_lines <= line_idx;
                            state                <= ST_IDLE;
                            frame_err_sticky     <= 1'b0;
                            current_long_active  <= 1'b0;
                            line_open            <= 1'b0;
                        end
                    end else begin
                        if (cfg_sof_synth && synth_wait_fe) begin
                            // FE-RESYNC point: this IDLE FE is the chip frame bottom
                            // after a cap-closed frame. Clear the wait so the NEXT LS
                            // opens the synthetic frame at the true top (not a sync err).
                            synth_wait_fe <= 1'b0;
                        end else if (guard_line_mode && guard_wait_fe_after_complete) begin
                            guard_wait_fe_after_complete <= 1'b0;
                        end else begin
                            sts_frame_sync_err_cnt <= sat_inc16(sts_frame_sync_err_cnt);
                        end
                    end
                end else if (is_ls && state == ST_IDLE && cfg_use_lsle && cfg_sof_synth && !synth_wait_fe) begin
                    // SOF-SYNTHESIS (2026-06-13): with the D-PHY lane supervisor
                    // enabled the chip's FS short packet is lost (it is the first
                    // burst after the long vblank clock-gate, ending before the
                    // supervisor re-locks: CLK_SETTLE + ISERDES reset + HS-SETTLE).
                    // LS/long/FE (in-frame, clock warm) still arrive — measured
                    // fs=0 but ls=7637/s, fe=16.4/s — so the frame never opens and
                    // the VDMA stays empty. Open the frame from the first LS after
                    // IDLE; the deployed FE_DELIMITS closes it at the chip's true
                    // bottom, and a dropped FE is bounded by the MAX_LINES cap.
                    state               <= ST_IN_FRAME;
                    line_idx            <= 16'h0000;
                    frame_err_sticky    <= 1'b0;
                    current_long_active <= 1'b0;
                    line_open           <= 1'b1;
                    sof_pending         <= 1'b1;
                    eof_pending         <= 1'b0;
                    frame_err_pending   <= 1'b0;
                    guard_wait_fe_after_complete <= 1'b0;
                    out_sol             <= 1'b1;
                    out_line_idx        <= 16'h0000;
                end else if (is_ls && state == ST_IN_FRAME && cfg_use_lsle) begin
                    out_sol       <= 1'b1;
                    out_line_idx  <= line_idx;
                    line_open     <= 1'b1;
                end else if (is_le && state == ST_IN_FRAME && cfg_use_lsle) begin
                    out_eol        <= 1'b1;
                    out_line_idx   <= line_idx;
                    line_open      <= 1'b0;
                    sts_line_count <= sat_inc32(sts_line_count);
                    if (force_guard_r && (line_idx >= expected_m1_r)) begin
                        // FORCE-EXPECTED close (2026-06-16): this LE is the
                        // EXPECTED_FRAME_LINES-th line. Close NOW at the constant
                        // height (out_eof) and drop to IDLE so the VDMA/VTC sees a
                        // fixed-height frame (genlock; live-HDMI roll fix). The
                        // overshoot lines (chip emits ~495 LE) drain in IDLE; the
                        // next FS reopens + re-anchors the top (anti-drift). A
                        // dropped FS just repeats the previous VDMA frame (no roll).
                        frame_closed         = 1'b1;
                        out_eof              <= 1'b1;
                        out_frame_err        <= frame_err_sticky;
                        sts_frame_count      <= sat_inc32(sts_frame_count);
                        sts_last_frame_lines <= line_idx + 16'd1;
                        line_idx             <= 16'h0000;
                        state                <= ST_IDLE;
                        frame_err_sticky     <= 1'b0;
                        current_long_active  <= 1'b0;
                        line_open            <= 1'b0;
                        sof_pending          <= 1'b0;
                        eof_pending          <= 1'b0;
                        frame_err_pending    <= 1'b0;
                        // Re-open at the chip's TRUE top: in synth mode WAIT for the
                        // next FE (the chip frame bottom) before the synthetic LS-open
                        // (synth_wait_fe=1), so the ~30 lines that arrive before the
                        // (late) FS are captured at the frame top instead of orphaned
                        // -> removes the bottom band. Overshoot LE (chip emits ~495)
                        // drain in IDLE while waiting. In legacy mode synth_wait_fe is
                        // unused (the next FS reopens).
                        synth_wait_fe        <= 1'b1;
                    end else if (lsle_line_guard && (line_idx >= (MAX_LINES_U16 - 16'd1))) begin
                        // RUNAWAY SAFETY CAP only: frames are normally delimited by FS
                        // (the phase anchor above). This fires solely when the chip
                        // dropped the FS so the frame would otherwise grow unbounded
                        // (the old 182..1831 merge). Capping at MAX_LINES keeps a
                        // missing-FS frame bounded just above the expected height
                        // instead of merging many frames. The cap is set ABOVE the
                        // expected frame so it never races the FS close in the normal
                        // case (no spurious empty/short frame). diary 20260601.
                        frame_closed         = 1'b1;
                        out_eof              <= 1'b1;
                        out_frame_err        <= frame_err_sticky;
                        sts_frame_count      <= sat_inc32(sts_frame_count);
                        sts_last_frame_lines <= line_idx + 16'd1;
                        line_idx             <= 16'h0000;
                        frame_err_sticky     <= 1'b0;
                        current_long_active  <= 1'b0;
                        if (cfg_sof_synth) begin
                            // FE-RESYNC: a cap-close means the FE was dropped, so the
                            // current vblank phase is unknown. Drop to IDLE and wait
                            // for the next FE before synthesizing a new SOF, so the
                            // next frame opens at the chip's true top (no phase jitter).
                            state          <= ST_IDLE;
                            synth_wait_fe  <= 1'b1;
                            line_open      <= 1'b0;
                            sof_pending    <= 1'b0;
                        end else begin
                            sof_pending    <= 1'b1;
                        end
                    end else begin
                        line_idx       <= line_idx + 16'd1;
                    end
                end else if (in_pkt_is_long) begin
                    if (state == ST_IN_FRAME) begin
                        if ((guard_line_mode && !guard_line_wc_ok) || guard_line_count_reached) begin
                            sts_frame_sync_err_cnt <= sat_inc16(sts_frame_sync_err_cnt);
                            current_long_active    <= 1'b0;
                            line_open              <= 1'b0;
                        end else if (cfg_use_lsle && !line_open && cfg_long_as_line) begin
                            // LONG-AS-LINE (2026-06-17): the LS for this line was
                            // dropped upstream (scattered loss), but the long IS a
                            // pixel row -> open the line on the long itself and
                            // deliver it, instead of rejecting. Recovers the bottom
                            // band. (DT filter already gated embedded data.)
                            current_long_active <= 1'b1;
                            line_open           <= 1'b1;
                            out_sol             <= 1'b1;
                            out_line_idx        <= line_idx;
                            sts_dbg_long_accept <= sat_inc16(sts_dbg_long_accept);
                        end else if (cfg_use_lsle && !line_open) begin
                            // Long packet without preceding LS — reject (embedded stat data)
                            sts_frame_sync_err_cnt <= sat_inc16(sts_frame_sync_err_cnt);
                            sts_dbg_long_nols      <= sat_inc16(sts_dbg_long_nols);
                            dbg_nols_bucket[line_idx[8:6]] <= sat_inc16(dbg_nols_bucket[line_idx[8:6]]);
                            current_long_active    <= 1'b0;
                        end else begin
                            current_long_active <= 1'b1;
                            sts_dbg_long_accept <= sat_inc16(sts_dbg_long_accept);
                            if (!cfg_use_lsle) begin
                                out_sol       <= 1'b1;
                                out_line_idx  <= line_idx;
                                line_open     <= 1'b1;
                            end
                        end
                    end else begin
                        sts_frame_sync_err_cnt <= sat_inc16(sts_frame_sync_err_cnt);
                        sts_dbg_long_idle      <= sat_inc16(sts_dbg_long_idle);
                        current_long_active    <= 1'b0;
                        guard_wait_fe_after_complete <= 1'b0;
                    end
                end
            end

            if (state == ST_IN_FRAME && !frame_closed) begin
                frame_err_sticky <= next_frame_err;
            end

            if (in_payload_valid && state == ST_IN_FRAME && current_long_active) begin
                out_payload_data  <= in_payload_data;
                out_payload_valid <= 1'b1;
                out_payload_first <= in_payload_first;
                out_payload_last  <= in_payload_last;
                out_line_idx      <= line_idx;

                if (sof_pending) begin
                    out_sof     <= 1'b1;
                    sof_pending <= 1'b0;
                end

                if (!cfg_use_lsle && in_payload_first && !line_open) begin
                    out_sol   <= 1'b1;
                    line_open <= 1'b1;
                end

                if (!cfg_use_lsle && in_payload_last) begin
                    out_eol        <= 1'b1;
                    line_open      <= 1'b0;
                    if (guard_expected_last_line) begin
                        out_eof           <= 1'b1;
                        out_frame_err     <= frame_err_sticky;
                        eof_pending       <= 1'b0;
                        frame_err_pending <= 1'b0;
                    end
                end

                if (eof_pending && in_payload_last) begin
                    out_eof           <= 1'b1;
                    out_frame_err     <= frame_err_pending;
                    eof_pending       <= 1'b0;
                    frame_err_pending <= 1'b0;
                end
            end

            if (in_pkt_end && in_pkt_is_long) begin
                current_long_active <= 1'b0;
                if (state == ST_IN_FRAME && !cfg_use_lsle && current_long_active) begin
                    line_open      <= 1'b0;
                    sts_line_count <= sat_inc32(sts_line_count);
                    if (line_idx == MAX_LINES - 1) begin
                        sts_frame_sync_err_cnt <= sat_inc16(sts_frame_sync_err_cnt);
                    end else if (guard_expected_last_line) begin
                        sts_frame_count      <= sat_inc32(sts_frame_count);
                        sts_last_frame_lines <= line_idx + 16'd1;
                        line_idx             <= line_idx + 16'd1;
                        state                <= ST_IDLE;
                        frame_err_sticky     <= 1'b0;
                        current_long_active  <= 1'b0;
                        line_open            <= 1'b0;
                        sof_pending          <= 1'b0;
                        eof_pending          <= 1'b0;
                        frame_err_pending    <= 1'b0;
                        guard_wait_fe_after_complete <= 1'b1;
                    end else begin
                        line_idx <= line_idx + 16'd1;
                    end
                end
            end
        end
    end

    assign out_in_frame = (state == ST_IN_FRAME);
    assign sts_dbg_nols_hist = {dbg_nols_bucket[7], dbg_nols_bucket[6],
                                dbg_nols_bucket[5], dbg_nols_bucket[4],
                                dbg_nols_bucket[3], dbg_nols_bucket[2],
                                dbg_nols_bucket[1], dbg_nols_bucket[0]};

endmodule
