`timescale 1ns / 1ps
`default_nettype none

// RAW10 unpacker: MIPI CSI-2 packed RAW10 (DT=0x2B) → 10-bit pixel stream.
//
// Packed format (5 bytes = 4 pixels):
//   byte[0] = pixel[0][9:2]   (MSBs)
//   byte[1] = pixel[1][9:2]
//   byte[2] = pixel[2][9:2]
//   byte[3] = pixel[3][9:2]
//   byte[4] = {pixel[3][1:0], pixel[2][1:0], pixel[1][1:0], pixel[0][1:0]}
//
// Output rate is 4 pixels per 5 input bytes, so the stream has periodic
// 1-cycle gaps. Downstream (axis_video_bridge) handles gaps.

module raw10_unpack #(
    parameter int LINE_PIXELS = 0
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

    output logic [9:0]  out_pixel,
    output logic        out_pixel_valid,
    output logic        out_pixel_sof,
    output logic        out_pixel_eol,
    output logic        out_pixel_eof,
    output logic        out_pixel_err,

    output logic [15:0] sts_pixel_per_line
);

    // Each fifo entry: {data[9:0], sof, eol, eof, err} = 14 bits
    localparam int FIFO_W = 14;

    logic [7:0]        msb [0:3];
    logic [2:0]        byte_idx;
    logic              sof_pending;
    logic              err_pending;
    logic [15:0]       line_count;

    logic [FIFO_W-1:0] fifo [0:3];
    logic [2:0]        fifo_count;
    logic [1:0]        fifo_rd_idx;
    logic [1:0]        fifo_wr_idx;

    always_ff @(posedge core_clk) begin
        if (!core_aresetn) begin
            byte_idx           <= 3'd0;
            sof_pending        <= 1'b0;
            err_pending        <= 1'b0;
            line_count         <= 16'd0;
            sts_pixel_per_line <= 16'd0;
            fifo_count         <= 3'd0;
            fifo_rd_idx        <= 2'd0;
            fifo_wr_idx        <= 2'd0;
            out_pixel          <= 10'd0;
            out_pixel_valid    <= 1'b0;
            out_pixel_sof      <= 1'b0;
            out_pixel_eol      <= 1'b0;
            out_pixel_eof      <= 1'b0;
            out_pixel_err      <= 1'b0;
            for (int i = 0; i < 4; i++) begin
                msb[i]  <= 8'h00;
                fifo[i] <= '0;
            end
        end else begin
            automatic logic [2:0] next_count;
            automatic logic [1:0] next_rd_idx;
            automatic logic [1:0] next_wr_idx;
            automatic logic [2:0] eff_byte_idx;

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

            next_count   = fifo_count;
            next_rd_idx  = fifo_rd_idx;
            next_wr_idx  = fifo_wr_idx;

            // -------- OUTPUT SIDE: emit one pixel per cycle when available --------
            if (fifo_count > 0) begin
                automatic logic [FIFO_W-1:0] word;
                automatic logic              is_eol;
                word   = fifo[fifo_rd_idx];
                is_eol = word[2];
                out_pixel       <= word[FIFO_W-1 -: 10];
                out_pixel_valid <= 1'b1;
                out_pixel_sof   <= word[3];
                out_pixel_eol   <= is_eol;
                out_pixel_eof   <= word[1];
                out_pixel_err   <= word[0];
                next_count   = next_count - 3'd1;
                next_rd_idx  = next_rd_idx + 2'd1;

                if (is_eol) begin
                    sts_pixel_per_line <= line_count + 16'd1;
                    line_count <= 16'd0;
                end else begin
                    line_count <= line_count + 16'd1;
                end
            end

            // -------- INPUT SIDE: byte accumulation + group push --------
            if (in_payload_valid) begin
                eff_byte_idx = in_payload_first ? 3'd0 : byte_idx;

                unique case (eff_byte_idx)
                    3'd0: msb[0] <= in_payload_data;
                    3'd1: msb[1] <= in_payload_data;
                    3'd2: msb[2] <= in_payload_data;
                    3'd3: msb[3] <= in_payload_data;
                    3'd4: begin
                        // Push 4 pixels using msb[] + in_payload_data (LSBs)
                        automatic logic line_end;
                        automatic logic frame_end;
                        automatic logic emit_err;
                        line_end  = in_payload_last || in_eol;
                        frame_end = in_eof && line_end;
                        emit_err  = err_pending || in_frame_err;
                        fifo[0] <= {msb[0], in_payload_data[1:0],
                                    sof_pending || in_sof, 1'b0, 1'b0, 1'b0};
                        fifo[1] <= {msb[1], in_payload_data[3:2],
                                    1'b0, 1'b0, 1'b0, 1'b0};
                        fifo[2] <= {msb[2], in_payload_data[5:4],
                                    1'b0, 1'b0, 1'b0, 1'b0};
                        fifo[3] <= {msb[3], in_payload_data[7:6],
                                    1'b0, line_end, frame_end,
                                    line_end && emit_err};
                        next_count  = next_count + 3'd4;
                        next_wr_idx = 2'd0;
                        next_rd_idx = 2'd0;
                        sof_pending <= 1'b0;
                        if (line_end) err_pending <= 1'b0;
                    end
                    default: ;
                endcase
                byte_idx <= (eff_byte_idx == 3'd4) ? 3'd0 : eff_byte_idx + 3'd1;
            end

            fifo_count  <= next_count;
            fifo_rd_idx <= next_rd_idx;
            fifo_wr_idx <= next_wr_idx;
        end
    end

endmodule

`default_nettype wire
