`timescale 1ns / 1ps
`default_nettype none

module yuv422_gray_unpack #(
    parameter logic [3:0] YUV422_SEQUENCE = 4'hf,
    parameter bit Y_AT_ODD_PHASE = (YUV422_SEQUENCE == 4'h2) || (YUV422_SEQUENCE == 4'h3) || (YUV422_SEQUENCE == 4'hf),
    parameter int LINE_PIXELS = 0,
    parameter int LEFT_REPAIR_PIXELS = 0
) (
    input  wire        core_clk,
    input  wire        core_aresetn,

    input  wire        in_sof,
    input  wire        in_eof,
    input  wire        in_eol,
    input  wire [7:0]  in_payload_data,
    input  wire        in_payload_valid,
    input  wire        in_payload_first,
    input  wire        in_payload_last,
    input  wire        in_frame_err,

    output logic [23:0] out_pixel,
    output logic        out_pixel_valid,
    output logic        out_pixel_sof,
    output logic        out_pixel_eol,
    output logic        out_pixel_eof,
    output logic        out_pixel_err,

    output logic [15:0] sts_pixel_per_line
);

    logic [1:0] yuv_phase;
    logic       sof_pending;
    logic       frame_err_pending;
    logic [15:0] line_pixel_count;
    logic [7:0]  left_repair_value;

    always_ff @(posedge core_clk) begin
        if (!core_aresetn) begin
            yuv_phase <= 2'd0;
            sof_pending <= 1'b0;
            frame_err_pending <= 1'b0;
            line_pixel_count <= 16'd0;
            left_repair_value <= 8'h00;
            sts_pixel_per_line <= 16'd0;
            out_pixel <= 24'h000000;
            out_pixel_valid <= 1'b0;
            out_pixel_sof <= 1'b0;
            out_pixel_eol <= 1'b0;
            out_pixel_eof <= 1'b0;
            out_pixel_err <= 1'b0;
        end else begin
            automatic logic [1:0] active_phase;
            automatic logic is_y_sample;
            automatic logic is_line_end;
            automatic logic [7:0] pixel_y;

            out_pixel_valid <= 1'b0;
            out_pixel_sof <= 1'b0;
            out_pixel_eol <= 1'b0;
            out_pixel_eof <= 1'b0;
            out_pixel_err <= 1'b0;

            active_phase = in_payload_first ? 2'd0 : yuv_phase;
            is_y_sample = in_payload_valid && (active_phase[0] == Y_AT_ODD_PHASE);

            if (in_sof) begin
                sof_pending <= 1'b1;
                frame_err_pending <= in_frame_err;
                line_pixel_count <= 16'd0;
            end else if (in_frame_err) begin
                frame_err_pending <= 1'b1;
            end

            if (in_payload_valid) begin
                if (is_y_sample) begin
                    is_line_end = (LINE_PIXELS > 0) ?
                        (line_pixel_count == LINE_PIXELS[15:0] - 16'd1) :
                        (in_eol || in_payload_last);
                    pixel_y = ((LEFT_REPAIR_PIXELS > 0) && (line_pixel_count < LEFT_REPAIR_PIXELS[15:0])) ?
                        left_repair_value : in_payload_data;
                    out_pixel <= {pixel_y, pixel_y, pixel_y};
                    out_pixel_valid <= 1'b1;
                    out_pixel_sof <= sof_pending || in_sof;
                    out_pixel_eol <= is_line_end;
                    out_pixel_eof <= in_eof;
                    out_pixel_err <= in_eof && (frame_err_pending || in_frame_err);
                    sof_pending <= 1'b0;

                    if ((LEFT_REPAIR_PIXELS > 0) && (line_pixel_count == LEFT_REPAIR_PIXELS[15:0])) begin
                        left_repair_value <= in_payload_data;
                    end

                    if (is_line_end) begin
                        sts_pixel_per_line <= line_pixel_count + 16'd1;
                        line_pixel_count <= 16'd0;
                    end else begin
                        line_pixel_count <= line_pixel_count + 16'd1;
                    end
                end

                if (active_phase == 2'd3) begin
                    yuv_phase <= 2'd0;
                end else begin
                    yuv_phase <= active_phase + 2'd1;
                end
            end

            // Standalone end-of-frame: frame_state asserts in_eof on a separate
            // cycle (the FE short packet has no payload), so the in_eof above --
            // gated by is_y_sample -- would be dropped. Forward it as a marker
            // pulse (out_pixel_valid stays 0; downstream frame delimiters key off
            // out_pixel_eof) so the frame boundary is not lost.
            if (in_eof && !(in_payload_valid && is_y_sample)) begin
                out_pixel_eof <= 1'b1;
                out_pixel_err <= frame_err_pending || in_frame_err;
            end
        end
    end

endmodule

`default_nettype wire
