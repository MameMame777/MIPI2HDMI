`timescale 1ns / 1ps
`default_nettype none

// RAW8 passthrough: MIPI byte stream → 8-bit pixel stream (1:1).
// CSI-2 DT = 0x2A. Each byte is one pixel; no demosaic, no unpacking.
// Output interface mirrors yuv422_gray_unpack for drop-in replacement.

module raw8_passthrough #(
    parameter int LINE_PIXELS = 0  // 0 = no length check; otherwise validate
) (
    input  wire         core_clk,
    input  wire         core_aresetn,

    input  wire         in_sof,
    input  wire         in_eof,
    input  wire         in_eol,
    input  wire  [7:0]  in_payload_data,
    input  wire         in_payload_valid,
    input  wire         in_payload_first,
    input  wire         in_payload_last,
    input  wire         in_frame_err,

    output logic [7:0]  out_pixel,
    output logic        out_pixel_valid,
    output logic        out_pixel_sof,
    output logic        out_pixel_eol,
    output logic        out_pixel_eof,
    output logic        out_pixel_err,

    output logic [15:0] sts_pixel_per_line
);

    logic        sof_pending;
    logic        err_pending;
    logic [15:0] line_count;

    always_ff @(posedge core_clk) begin
        if (!core_aresetn) begin
            sof_pending        <= 1'b0;
            err_pending        <= 1'b0;
            line_count         <= 16'd0;
            sts_pixel_per_line <= 16'd0;
            out_pixel          <= 8'h00;
            out_pixel_valid    <= 1'b0;
            out_pixel_sof      <= 1'b0;
            out_pixel_eol      <= 1'b0;
            out_pixel_eof      <= 1'b0;
            out_pixel_err      <= 1'b0;
        end else begin
            automatic logic is_line_end;

            out_pixel_valid <= 1'b0;
            out_pixel_sof   <= 1'b0;
            out_pixel_eol   <= 1'b0;
            out_pixel_eof   <= 1'b0;
            out_pixel_err   <= 1'b0;

            if (in_sof) begin
                sof_pending <= 1'b1;
                err_pending <= in_frame_err;
                line_count  <= 16'd0;
            end else if (in_frame_err) begin
                err_pending <= 1'b1;
            end

            if (in_payload_valid) begin
                is_line_end = (LINE_PIXELS > 0)
                    ? (line_count == LINE_PIXELS[15:0] - 16'd1)
                    : (in_eol || in_payload_last);

                out_pixel       <= in_payload_data;
                out_pixel_valid <= 1'b1;
                out_pixel_sof   <= sof_pending || in_sof;
                out_pixel_eol   <= is_line_end;
                out_pixel_eof   <= in_eof && is_line_end;
                out_pixel_err   <= in_eof && (err_pending || in_frame_err);
                sof_pending     <= 1'b0;

                if (is_line_end) begin
                    sts_pixel_per_line <= line_count + 16'd1;
                    line_count <= 16'd0;
                    if (in_eof) err_pending <= 1'b0;
                end else begin
                    line_count <= line_count + 16'd1;
                end
            end
        end
    end

endmodule

`default_nettype wire
