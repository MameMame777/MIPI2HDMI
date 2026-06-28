`timescale 1ns / 1ps

// Optical-Black row masker (full-line stats + ping-pong line buffer).
//
// Tracks min/max across the ENTIRE input line, finalizes a decision at EOL,
// and applies it to the line's output via a ping-pong buffer. A line is
// masked (all pixels replaced with OB_FILL_Y) iff
//     max < OB_THRESHOLD && (max - min) <= OB_UNIFORMITY.
//
// Local sample windows (first N pixels only) cannot distinguish OB rows from
// image rows whose left edge happens to be a dark block; full-line statistics
// can: a real OB row has std≈0.5 across all 640 pixels, an image row does not.
//
// WIDTH parameter selects pixel data width:
//   WIDTH = 8  → YUV422 grayscale / RAW8 / RGB565-gray (default)
//   WIDTH = 10 → RAW10 (10-bit Bayer)
//
// For RAW10, instantiate with scaled thresholds, e.g.
//   #(.WIDTH(10), .OB_THRESHOLD(10'd200), .OB_FILL_Y(10'd512),
//     .OB_UNIFORMITY(10'd12))
//
// Latency: ~1 line. Assumes max line length LINE_PIXELS_MAX.

module ob_row_masker #(
    parameter int               WIDTH           = 8,
    parameter int               LINE_PIXELS_MAX = 1024,
    parameter logic [WIDTH-1:0] OB_THRESHOLD    = WIDTH'(50),
    parameter logic [WIDTH-1:0] OB_FILL_Y       = WIDTH'(128),
    parameter logic [WIDTH-1:0] OB_UNIFORMITY   = WIDTH'(3)
) (
    input  logic              clk,
    input  logic              aresetn,
    input  logic              enable,

    input  logic [WIDTH-1:0]  in_data,
    input  logic              in_valid,
    input  logic              in_sof,
    input  logic              in_eol,
    input  logic              in_eof,
    input  logic              in_err,

    output logic [WIDTH-1:0]  out_data,
    output logic              out_valid,
    output logic              out_sof,
    output logic              out_eol,
    output logic              out_eof,
    output logic              out_err
);

    localparam int ADDR_W   = $clog2(LINE_PIXELS_MAX);
    localparam int LEN_W    = $clog2(LINE_PIXELS_MAX + 1);
    localparam int FLAG_W   = 4;                       // sof, eol, eof, err
    localparam int WORD_W   = WIDTH + FLAG_W;

    logic [WORD_W-1:0] buf_a [0:LINE_PIXELS_MAX-1];
    logic [WORD_W-1:0] buf_b [0:LINE_PIXELS_MAX-1];

    logic              buf_a_full, buf_b_full;
    logic              buf_a_dark, buf_b_dark;
    logic [LEN_W-1:0]  buf_a_len,  buf_b_len;

    logic              wr_to_a;
    logic [ADDR_W-1:0] wr_idx;
    logic [WIDTH-1:0]  wr_min, wr_max;
    logic              wr_first;

    logic              rd_from_a;
    logic [ADDR_W-1:0] rd_idx;
    logic              rd_active;

    wire [WORD_W-1:0] wr_word = {in_data, in_sof, in_eol, in_eof, in_err};
    wire [WORD_W-1:0] rd_word = rd_from_a ? buf_a[rd_idx] : buf_b[rd_idx];

    always_ff @(posedge clk) begin
        if (!aresetn) begin
            buf_a_full <= 1'b0;
            buf_b_full <= 1'b0;
            buf_a_dark <= 1'b0;
            buf_b_dark <= 1'b0;
            buf_a_len  <= '0;
            buf_b_len  <= '0;
            wr_to_a    <= 1'b1;
            wr_idx     <= '0;
            wr_min     <= '1;
            wr_max     <= '0;
            wr_first   <= 1'b1;
            rd_from_a  <= 1'b1;
            rd_idx     <= '0;
            rd_active  <= 1'b0;
            out_data   <= '0;
            out_valid  <= 1'b0;
            out_sof    <= 1'b0;
            out_eol    <= 1'b0;
            out_eof    <= 1'b0;
            out_err    <= 1'b0;
        end else begin
            // -------------------- WRITE SIDE --------------------
            if (in_valid) begin
                if (wr_to_a) buf_a[wr_idx] <= wr_word;
                else         buf_b[wr_idx] <= wr_word;

                wr_idx <= wr_idx + 1;

                if (wr_first) begin
                    wr_min   <= in_data;
                    wr_max   <= in_data;
                    wr_first <= 1'b0;
                end else begin
                    if (in_data < wr_min) wr_min <= in_data;
                    if (in_data > wr_max) wr_max <= in_data;
                end

                if (in_eol) begin
                    automatic logic [WIDTH-1:0] fmin;
                    automatic logic [WIDTH-1:0] fmax;
                    fmin = wr_first ? in_data
                                    : ((in_data < wr_min) ? in_data : wr_min);
                    fmax = wr_first ? in_data
                                    : ((in_data > wr_max) ? in_data : wr_max);

                    if (wr_to_a) begin
                        buf_a_full <= 1'b1;
                        buf_a_len  <= wr_idx + 1;
                        buf_a_dark <= (fmax < OB_THRESHOLD) &&
                                      ((fmax - fmin) <= OB_UNIFORMITY);
                    end else begin
                        buf_b_full <= 1'b1;
                        buf_b_len  <= wr_idx + 1;
                        buf_b_dark <= (fmax < OB_THRESHOLD) &&
                                      ((fmax - fmin) <= OB_UNIFORMITY);
                    end

                    wr_to_a  <= !wr_to_a;
                    wr_idx   <= '0;
                    wr_first <= 1'b1;
                end
            end

            // -------------------- READ SIDE --------------------
            out_valid <= 1'b0;
            out_sof   <= 1'b0;
            out_eol   <= 1'b0;
            out_eof   <= 1'b0;
            out_err   <= 1'b0;

            if (!rd_active) begin
                if (rd_from_a && buf_a_full) begin
                    rd_active <= 1'b1;
                    rd_idx    <= '0;
                end else if (!rd_from_a && buf_b_full) begin
                    rd_active <= 1'b1;
                    rd_idx    <= '0;
                end
            end else begin
                automatic logic [WIDTH-1:0] data_out;
                automatic logic              sof_out, eol_out, eof_out, err_out;
                automatic logic              dark_apply;
                automatic logic [LEN_W-1:0]  cur_len;
                data_out   = rd_word[WORD_W-1 -: WIDTH];
                sof_out    = rd_word[3];
                eol_out    = rd_word[2];
                eof_out    = rd_word[1];
                err_out    = rd_word[0];
                dark_apply = enable && (rd_from_a ? buf_a_dark : buf_b_dark);
                cur_len    = rd_from_a ? buf_a_len : buf_b_len;

                out_data  <= dark_apply ? OB_FILL_Y : data_out;
                out_valid <= 1'b1;
                out_sof   <= sof_out;
                out_eol   <= eol_out;
                out_eof   <= eof_out;
                out_err   <= err_out;

                rd_idx <= rd_idx + 1;

                if (eol_out || ((rd_idx + 1) >= cur_len)) begin
                    rd_active <= 1'b0;
                    if (rd_from_a) buf_a_full <= 1'b0;
                    else           buf_b_full <= 1'b0;
                    rd_from_a <= !rd_from_a;
                end
            end
        end
    end

endmodule
