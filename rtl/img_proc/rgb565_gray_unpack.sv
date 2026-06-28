`timescale 1ns / 1ps
`default_nettype none

module rgb565_gray_unpack #(
    parameter bit RGB565_BIG_ENDIAN = 1'b0,
    // RGB_OUT (2026-06-23, color path): 0 = legacy luma/gray out (24-bit {Y,Y,Y});
    // 1 = true RGB888 out (24-bit {R,G,B}). R/G/B are already reconstructed from the
    // RGB565 word below; this just selects RGB vs the luma-replicate. Set 1 with the
    // probe COLOR_CAPTURE for a real colour capture/HDMI.
    parameter bit RGB_OUT = 1'b0,
    parameter int LINE_PIXELS = 0
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

    logic       byte_phase;
    logic       sof_pending;
    logic       frame_err_pending;
    logic [7:0] first_payload_byte;
    logic [15:0] line_pixel_count;

    always_ff @(posedge core_clk) begin
        if (!core_aresetn) begin
            byte_phase <= 1'b0;
            sof_pending <= 1'b0;
            frame_err_pending <= 1'b0;
            first_payload_byte <= 8'h00;
            line_pixel_count <= 16'd0;
            sts_pixel_per_line <= 16'd0;
            out_pixel <= 24'h000000;
            out_pixel_valid <= 1'b0;
            out_pixel_sof <= 1'b0;
            out_pixel_eol <= 1'b0;
            out_pixel_eof <= 1'b0;
            out_pixel_err <= 1'b0;
        end else begin
            automatic logic active_phase;
            automatic logic is_line_end;
            automatic logic [15:0] rgb_word;
            automatic logic [7:0] red8;
            automatic logic [7:0] green8;
            automatic logic [7:0] blue8;
            automatic logic [17:0] luma_sum;
            automatic logic [7:0] gray8;

            out_pixel_valid <= 1'b0;
            out_pixel_sof <= 1'b0;
            out_pixel_eol <= 1'b0;
            out_pixel_eof <= 1'b0;
            out_pixel_err <= 1'b0;

            active_phase = in_payload_first ? 1'b0 : byte_phase;

            if (in_sof) begin
                sof_pending <= 1'b1;
                frame_err_pending <= in_frame_err;
                line_pixel_count <= 16'd0;
            end else if (in_frame_err) begin
                frame_err_pending <= 1'b1;
            end

            if (in_payload_valid) begin
                if (!active_phase) begin
                    first_payload_byte <= in_payload_data;
                    byte_phase <= 1'b1;
                end else begin
                    rgb_word = RGB565_BIG_ENDIAN ? {first_payload_byte, in_payload_data} : {in_payload_data, first_payload_byte};
                    red8 = {rgb_word[15:11], rgb_word[15:13]};
                    green8 = {rgb_word[10:5], rgb_word[10:9]};
                    blue8 = {rgb_word[4:0], rgb_word[4:2]};
                    luma_sum = (red8 * 18'd77) + (green8 * 18'd150) + (blue8 * 18'd29);
                    gray8 = luma_sum[15:8];

                    is_line_end = (LINE_PIXELS > 0) ?
                        (line_pixel_count == LINE_PIXELS[15:0] - 16'd1) :
                        (in_eol || in_payload_last);
                    out_pixel <= RGB_OUT ? {red8, green8, blue8} : {gray8, gray8, gray8};
                    out_pixel_valid <= 1'b1;
                    out_pixel_sof <= sof_pending || in_sof;
                    out_pixel_eol <= is_line_end;
                    out_pixel_eof <= in_eof;
                    out_pixel_err <= in_eof && (frame_err_pending || in_frame_err);
                    sof_pending <= 1'b0;

                    if (is_line_end) begin
                        sts_pixel_per_line <= line_pixel_count + 16'd1;
                        line_pixel_count <= 16'd0;
                    end else begin
                        line_pixel_count <= line_pixel_count + 16'd1;
                    end

                    byte_phase <= 1'b0;
                end
            end
        end
    end

endmodule

`default_nettype wire
