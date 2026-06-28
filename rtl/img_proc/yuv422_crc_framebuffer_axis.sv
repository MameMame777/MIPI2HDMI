`timescale 1ns / 1ps
`default_nettype none

module yuv422_crc_framebuffer_axis #(
    parameter int WIDTH = 640,
    parameter int HEIGHT = 480,
    parameter int LINE_BYTES = WIDTH * 2,
    parameter int TDATA_WIDTH = 24
) (
    input  wire                    core_clk,
    input  wire                    core_aresetn,
    input  wire                    pix_clk,
    input  wire                    pix_aresetn,

    input  wire [7:0]              pkt_di,
    input  wire [15:0]             pkt_wc,
    input  wire                    pkt_is_short,
    input  wire                    pkt_is_long,
    input  wire                    pkt_start,
    input  wire                    pkt_end,
    input  wire                    pkt_err,
    input  wire [7:0]              payload_data,
    input  wire                    payload_valid,
    input  wire                    payload_first,
    input  wire                    payload_last,
    input  wire                    crc_check_valid,
    input  wire                    crc_match,

    output logic [TDATA_WIDTH-1:0] m_axis_tdata,
    output logic                   m_axis_tvalid,
    input  wire                    m_axis_tready,
    output logic                   m_axis_tlast,
    output logic [0:0]             m_axis_tuser,

    output logic [15:0]            sts_good_line_count,
    output logic [15:0]            sts_bad_line_count,
    output logic [31:0]            sts_frame_count,
    output logic [15:0]            sts_write_line,
    output logic                   sts_frame_ready
);

    localparam logic [5:0] DT_FS = 6'h00;
    localparam logic [5:0] DT_FE = 6'h01;
    localparam logic [5:0] DT_YUV422_8 = 6'h1e;
    localparam int LINE_ADDR_WIDTH = (LINE_BYTES <= 2) ? 1 : $clog2(LINE_BYTES);
    localparam int X_WIDTH = (WIDTH <= 2) ? 1 : $clog2(WIDTH);
    localparam int Y_WIDTH = (HEIGHT <= 2) ? 1 : $clog2(HEIGHT);

    logic [7:0] line_buf [0:LINE_BYTES-1];
    logic [7:0] display_line [0:WIDTH-1];

    logic in_frame;
    logic capture_active;
    logic capture_err;
    logic [LINE_ADDR_WIDTH:0] capture_count;
    logic replay_active;
    logic [X_WIDTH:0] replay_x;
    logic [Y_WIDTH-1:0] replay_line;
    logic [15:0] good_lines_in_frame;

    logic frame_ready_core;
    logic frame_ready_pix_meta;
    logic frame_ready_pix;
    logic [X_WIDTH-1:0] rd_x;
    logic [Y_WIDTH-1:0] rd_y;

    function automatic [15:0] sat_inc16(input [15:0] value);
        sat_inc16 = (value == 16'hffff) ? value : (value + 16'd1);
    endfunction

    function automatic [31:0] sat_inc32(input [31:0] value);
        sat_inc32 = (value == 32'hffff_ffff) ? value : (value + 32'd1);
    endfunction

    function automatic [7:0] diagnostic_gray(input logic [X_WIDTH-1:0] x_pos, input logic [Y_WIDTH-1:0] y_pos);
        if (x_pos[X_WIDTH-1 -: 1] ^ y_pos[Y_WIDTH-1 -: 1]) begin
            diagnostic_gray = 8'h30;
        end else begin
            diagnostic_gray = 8'h08;
        end
    endfunction

    wire [5:0] pkt_dt = pkt_di[5:0];
    wire pkt_is_fs = pkt_is_short && (pkt_dt == DT_FS);
    wire pkt_is_fe = pkt_is_short && (pkt_dt == DT_FE);
    wire pkt_is_yuv_line = pkt_is_long && (pkt_dt == DT_YUV422_8) && (pkt_wc == LINE_BYTES[15:0]);

    always_ff @(posedge core_clk) begin
        if (!core_aresetn) begin
            in_frame <= 1'b0;
            capture_active <= 1'b0;
            capture_err <= 1'b0;
            capture_count <= '0;
            replay_active <= 1'b0;
            replay_x <= '0;
            replay_line <= '0;
            good_lines_in_frame <= 16'h0000;
            frame_ready_core <= 1'b0;
            sts_good_line_count <= 16'h0000;
            sts_bad_line_count <= 16'h0000;
            sts_frame_count <= 32'h0000_0000;
            sts_write_line <= 16'h0000;
            sts_frame_ready <= 1'b0;
        end else begin
            if (pkt_start) begin
                if (pkt_is_fs) begin
                    in_frame <= 1'b1;
                    capture_active <= 1'b0;
                    capture_count <= '0;
                    replay_active <= 1'b0;
                    replay_x <= '0;
                    replay_line <= '0;
                    good_lines_in_frame <= 16'h0000;
                    sts_write_line <= 16'h0000;
                end else if (pkt_is_fe) begin
                    capture_active <= 1'b0;
                    in_frame <= 1'b0;
                    if (good_lines_in_frame != 16'h0000) begin
                        frame_ready_core <= 1'b1;
                        sts_frame_ready <= 1'b1;
                        sts_frame_count <= sat_inc32(sts_frame_count);
                    end
                end else if (pkt_is_yuv_line && !replay_active) begin
                    capture_active <= 1'b1;
                    capture_err <= pkt_err;
                    capture_count <= '0;
                end else if (pkt_is_long) begin
                    capture_active <= 1'b0;
                end
            end

            if (capture_active && payload_valid) begin
                if (payload_first) begin
                    capture_count <= '0;
                end
                if (capture_count < LINE_BYTES[LINE_ADDR_WIDTH:0]) begin
                    line_buf[capture_count[LINE_ADDR_WIDTH-1:0]] <= payload_data;
                    capture_count <= capture_count + 1'b1;
                end else begin
                    capture_err <= 1'b1;
                end
                if (payload_last && ((capture_count + 1'b1) != LINE_BYTES[LINE_ADDR_WIDTH:0])) begin
                    capture_err <= 1'b1;
                end
            end

            if (crc_check_valid && capture_active) begin
                if (crc_match && !capture_err && (capture_count == LINE_BYTES[LINE_ADDR_WIDTH:0])) begin
                    replay_active <= 1'b1;
                    replay_x <= '0;
                    replay_line <= sts_write_line[Y_WIDTH-1:0];
                end else begin
                    sts_bad_line_count <= sat_inc16(sts_bad_line_count);
                end
                capture_active <= 1'b0;
            end

            if (pkt_end && capture_active && !crc_check_valid) begin
                capture_err <= capture_err | pkt_err;
            end

            if (replay_active) begin
                automatic logic [LINE_ADDR_WIDTH-1:0] y_index;

                y_index = ({replay_x[X_WIDTH-1:0], 1'b0} + {{(LINE_ADDR_WIDTH-1){1'b0}}, 1'b1});
                display_line[replay_x[X_WIDTH-1:0]] <= line_buf[y_index];

                if (replay_x == WIDTH[X_WIDTH:0] - 1'b1) begin
                    replay_active <= 1'b0;
                    frame_ready_core <= 1'b1;
                    sts_frame_ready <= 1'b1;
                    sts_frame_count <= sat_inc32(sts_frame_count);
                    sts_good_line_count <= sat_inc16(sts_good_line_count);
                    good_lines_in_frame <= sat_inc16(good_lines_in_frame);
                    if (sts_write_line == HEIGHT[15:0] - 16'd1) begin
                        sts_write_line <= 16'd0;
                    end else begin
                        sts_write_line <= sts_write_line + 16'd1;
                    end
                end else begin
                    replay_x <= replay_x + 1'b1;
                end
            end
        end
    end

    always_ff @(posedge pix_clk) begin
        if (!pix_aresetn) begin
            frame_ready_pix_meta <= 1'b0;
            frame_ready_pix <= 1'b0;
            rd_x <= '0;
            rd_y <= '0;
            m_axis_tdata <= '0;
            m_axis_tvalid <= 1'b1;
            m_axis_tlast <= 1'b0;
            m_axis_tuser <= 1'b1;
        end else begin
            frame_ready_pix_meta <= frame_ready_core;
            frame_ready_pix <= frame_ready_pix_meta;
            m_axis_tvalid <= 1'b1;

            if (!m_axis_tvalid || m_axis_tready) begin
                automatic logic [7:0] gray;
                automatic logic [X_WIDTH-1:0] next_x;
                automatic logic [Y_WIDTH-1:0] next_y;

                gray = frame_ready_pix ? display_line[rd_x] : diagnostic_gray(rd_x, rd_y);
                m_axis_tdata <= {gray, gray, gray};
                m_axis_tlast <= (rd_x == WIDTH[X_WIDTH-1:0] - 1'b1);
                m_axis_tuser[0] <= (rd_x == '0) && (rd_y == '0);

                next_x = rd_x;
                next_y = rd_y;

                if (rd_x == WIDTH[X_WIDTH-1:0] - 1'b1) begin
                    next_x = '0;
                    if (rd_y == HEIGHT[Y_WIDTH-1:0] - 1'b1) begin
                        next_y = '0;
                    end else begin
                        next_y = rd_y + 1'b1;
                    end
                end else begin
                    next_x = rd_x + 1'b1;
                end

                rd_x <= next_x;
                rd_y <= next_y;
            end
        end
    end

endmodule

`default_nettype wire
