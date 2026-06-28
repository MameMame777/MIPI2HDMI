`timescale 1ns / 1ps
`default_nettype none

// video_frame_normalizer (2026-06-04)
// -----------------------------------------------------------------------------
// Force a variable-geometry video pixel stream to EXACTLY OUT_LINES lines of
// EXACTLY OUT_PIXELS pixels per frame, so the downstream AXI-VDMA (fixed
// VSIZE x HSIZE) never tiles/rolls. The e2e sim (tb_e2e_vdma_stacking) proved
// the assembly RTL framing is clean (1 SOF/frame) but a free-running VDMA with
// frame_lines != VSIZE tiles the buffer (~VSIZE/frame copies = the observed
// ~4-tile hardware stacking). Pinning frame == VSIZE removes the mismatch.
//
//   * short input line  -> padded with FILL to OUT_PIXELS
//   * long  input line  -> truncated at OUT_PIXELS (extra src pixels dropped)
//   * short input frame -> padded with FILL lines to OUT_LINES
//   * long  input frame -> truncated at OUT_LINES (extra src lines dropped)
//
// Frame top = in_sof; frame end = in_eof; per-line end = in_eol (presented on
// the LAST source pixel of a line). Output is a clean constant-geometry stream.
// The chip stream has gaps between lines (LS/LE) and frames (blanking) so the
// FILL padding fits in those gaps -- output never runs faster than input over a
// frame. NORMALIZE=0 makes the block a transparent pass-through.

module video_frame_normalizer #(
    parameter int OUT_LINES  = 480,
    parameter int OUT_PIXELS = 640,
    parameter logic [7:0] FILL = 8'h00,
    parameter bit NORMALIZE  = 1'b1
) (
    input  wire        clk,
    input  wire        aresetn,

    input  wire [7:0]  in_data,
    input  wire        in_valid,
    input  wire        in_sof,
    input  wire        in_eol,
    input  wire        in_eof,
    input  wire        in_err,

    output logic [7:0] out_data,
    output logic       out_valid,
    output logic       out_sof,
    output logic       out_eol,
    output logic       out_eof,
    output logic       out_err
);
    localparam int LW = (OUT_LINES  <= 1) ? 1 : $clog2(OUT_LINES  + 1);
    localparam int PW = (OUT_PIXELS <= 1) ? 1 : $clog2(OUT_PIXELS + 1);

    if (!NORMALIZE) begin : g_bypass
        always_comb begin
            out_data  = in_data;  out_valid = in_valid; out_sof = in_sof;
            out_eol   = in_eol;   out_eof   = in_eof;   out_err = in_err;
        end
    end else begin : g_norm
        localparam logic [1:0] ST_IDLE = 2'd0, ST_ACTIVE = 2'd1,
                               ST_DROP = 2'd2, ST_TAIL  = 2'd3;
        logic [1:0]    state;
        logic [LW-1:0] row;            // output lines completed this frame
        logic [PW-1:0] col;            // output pixels emitted in current line
        logic          line_src_done;  // current src line exhausted -> pad
        logic          frame_src_done; // src frame ended (in_eof seen) -> tail-pad
        logic          frame_err;

        always_ff @(posedge clk) begin
            if (!aresetn) begin
                state <= ST_IDLE; row <= '0; col <= '0;
                line_src_done <= 1'b0; frame_src_done <= 1'b0; frame_err <= 1'b0;
                out_data <= 8'h00; out_valid <= 1'b0; out_sof <= 1'b0;
                out_eol <= 1'b0; out_eof <= 1'b0; out_err <= 1'b0;
            end else begin
                automatic logic        do_emit;
                automatic logic [7:0]  emit_d;
                automatic logic        emit_sof;
                automatic logic        emit_was_src;  // emitted a forwarded source pixel (not pad/FILL)
                automatic logic        src_eol_now;   // this emitted pixel is the src line's last
                automatic logic        close_line;
                automatic logic        close_frame;
                automatic logic        frame_end_now; // src frame ended (registered or this cycle)

                out_valid <= 1'b0; out_sof <= 1'b0; out_eol <= 1'b0;
                out_eof   <= 1'b0; out_err <= 1'b0;

                do_emit = 1'b0; emit_d = FILL; emit_sof = 1'b0; src_eol_now = 1'b0;
                emit_was_src = 1'b0;

                unique case (state)
                    ST_IDLE: begin
                        // wait for a frame top
                        if (in_valid && in_sof) begin
                            row <= '0; col <= '0;
                            line_src_done  <= 1'b0;
                            frame_src_done <= in_eof;     // 1-line frame edge case
                            frame_err      <= in_err;
                            do_emit  = 1'b1; emit_d = in_data; emit_sof = 1'b1;
                            emit_was_src = 1'b1; src_eol_now = in_eol | in_eof;
                            state <= ST_ACTIVE;
                        end
                    end
                    ST_ACTIVE: begin
                        frame_err <= frame_err | (in_valid & in_err);
                        if (in_eof) frame_src_done <= 1'b1;   // eof may arrive with valid=0
                        if (!line_src_done && in_valid) begin
                            // forward a source pixel (eof on a valid pixel still carries it)
                            do_emit = 1'b1; emit_d = in_data; emit_was_src = 1'b1;
                            src_eol_now = in_eol | in_eof;
                        end else if (line_src_done || frame_src_done) begin
                            // pad rest of short line / pad once the src frame has ended
                            do_emit = 1'b1; emit_d = FILL;
                        end
                        // else: src line active but no valid this cycle -> stall
                    end
                    ST_DROP: begin
                        // src line ran longer than OUT_PIXELS: swallow until its end
                        if (in_eof) frame_src_done <= 1'b1;
                        if (in_valid && (in_eol || in_eof)) begin
                            line_src_done <= 1'b0;
                            state <= (frame_src_done || in_eof) ? ST_TAIL : ST_ACTIVE;
                        end
                    end
                    ST_TAIL: begin
                        do_emit = 1'b1; emit_d = FILL;        // pad remaining FILL lines
                    end
                    default: state <= ST_IDLE;
                endcase

                // ---- common emit + col/row advance ----
                close_line    = do_emit && (col == PW'(OUT_PIXELS - 1));
                close_frame   = close_line && (row == LW'(OUT_LINES - 1));
                frame_end_now = frame_src_done | in_eof;

                if (do_emit) begin
                    out_data  <= emit_d;
                    out_valid <= 1'b1;
                    out_sof   <= emit_sof;
                    if (close_line) begin
                        out_eol <= 1'b1;
                        col <= '0;
                        if (close_frame) begin
                            out_eof <= 1'b1;
                            out_err <= frame_err | (in_valid & in_err);
                            row <= '0;
                            line_src_done  <= 1'b0;
                            frame_src_done <= 1'b0;
                            state <= ST_IDLE;          // long frame: extra src lines dropped in IDLE
                        end else begin
                            row <= row + LW'(1);
                            line_src_done <= 1'b0;
                            // pick next-line behaviour:
                            if (frame_end_now || (state == ST_TAIL)) begin
                                state <= ST_TAIL;                 // src frame ended -> keep padding
                            end else if (emit_was_src && !src_eol_now) begin
                                state <= ST_DROP;                 // src line longer than OUT_PIXELS
                            end else begin
                                state <= ST_ACTIVE;               // line done (exact/padded) -> next line
                            end
                        end
                    end else begin
                        col <= col + PW'(1);
                        if (src_eol_now) line_src_done <= 1'b1;  // short src line: pad remainder
                    end
                end
            end
        end
    end
endmodule
`default_nettype wire
