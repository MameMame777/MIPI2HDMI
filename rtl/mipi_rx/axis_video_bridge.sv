`timescale 1ns / 1ps

module axis_video_bridge #(
    parameter int TDATA_WIDTH = 16,
    parameter int TUSER_WIDTH = 2,
    parameter int FIFO_DEPTH = 4096,
    parameter bit AXIS_TUSER_ERR_DEBUG = 1'b1
) (
    input  logic                       core_clk,
    input  logic                       core_aresetn,
    input  logic                       aclk,
    input  logic                       aresetn,

    input  logic [TDATA_WIDTH-1:0]     in_pixel,
    input  logic                       in_pixel_valid,
    input  logic                       in_pixel_sof,
    input  logic                       in_pixel_eol,
    input  logic                       in_pixel_eof,
    input  logic                       in_pixel_err,

    output logic [TDATA_WIDTH-1:0]     m_axis_tdata,
    output logic                       m_axis_tvalid,
    input  logic                       m_axis_tready,
    output logic                       m_axis_tlast,
    output logic [TUSER_WIDTH-1:0]     m_axis_tuser,

    output logic [15:0]                sts_fifo_overflow_cnt,
    output logic [15:0]                sts_back_pressure_cnt
);

    localparam int SIDE_WIDTH = 4;
    localparam int WORD_WIDTH = TDATA_WIDTH + SIDE_WIDTH;
    localparam int ADDR_WIDTH = (FIFO_DEPTH <= 2) ? 1 : $clog2(FIFO_DEPTH);
    localparam int PTR_WIDTH = ADDR_WIDTH + 1;

    (* ram_style = "block" *) logic [WORD_WIDTH-1:0] fifo_mem [0:FIFO_DEPTH-1];

    logic [PTR_WIDTH-1:0] wr_bin;
    logic [PTR_WIDTH-1:0] wr_gray;
    logic [PTR_WIDTH-1:0] rd_bin_core_sync1;
    logic [PTR_WIDTH-1:0] rd_bin_core_sync2;
    logic [PTR_WIDTH-1:0] rd_gray_core_sync1;
    logic [PTR_WIDTH-1:0] rd_gray_core_sync2;

    logic [PTR_WIDTH-1:0] rd_bin;
    logic [PTR_WIDTH-1:0] rd_gray;
    logic [PTR_WIDTH-1:0] wr_gray_aclk_sync1;
    logic [PTR_WIDTH-1:0] wr_gray_aclk_sync2;

    logic overflow_toggle_core;
    logic overflow_toggle_aclk_sync1;
    logic overflow_toggle_aclk_sync2;
    logic overflow_toggle_aclk_prev;

    logic fifo_full;
    logic fifo_empty;
    logic [WORD_WIDTH-1:0] rd_word;
    logic rd_word_valid;
    logic rd_prefetch_valid;
    logic [WORD_WIDTH-1:0] rd_prefetch_word;

    function automatic [PTR_WIDTH-1:0] bin_to_gray(input [PTR_WIDTH-1:0] value);
        bin_to_gray = (value >> 1) ^ value;
    endfunction

    function automatic [15:0] sat_inc16(input [15:0] value);
        if (value == 16'hffff) begin
            sat_inc16 = value;
        end else begin
            sat_inc16 = value + 16'd1;
        end
    endfunction

    assign fifo_full = (bin_to_gray(wr_bin + {{(PTR_WIDTH-1){1'b0}}, 1'b1}) ==
                        {~rd_gray_core_sync2[PTR_WIDTH-1:PTR_WIDTH-2], rd_gray_core_sync2[PTR_WIDTH-3:0]});
    assign fifo_empty = (rd_gray == wr_gray_aclk_sync2);

    always_ff @(posedge core_clk) begin
        if (!core_aresetn) begin
            wr_bin                 <= '0;
            wr_gray                <= '0;
            rd_bin_core_sync1      <= '0;
            rd_bin_core_sync2      <= '0;
            rd_gray_core_sync1     <= '0;
            rd_gray_core_sync2     <= '0;
            overflow_toggle_core   <= 1'b0;
        end else begin
            automatic logic [PTR_WIDTH-1:0] wr_bin_next;

            rd_bin_core_sync1  <= rd_bin;
            rd_bin_core_sync2  <= rd_bin_core_sync1;
            rd_gray_core_sync1 <= rd_gray;
            rd_gray_core_sync2 <= rd_gray_core_sync1;

            if (in_pixel_valid) begin
                if (!fifo_full) begin
                    fifo_mem[wr_bin[ADDR_WIDTH-1:0]] <= {
                        in_pixel_err,
                        in_pixel_eof,
                        in_pixel_eol,
                        in_pixel_sof,
                        in_pixel
                    };
                    wr_bin_next = wr_bin + {{(PTR_WIDTH-1){1'b0}}, 1'b1};
                    wr_bin      <= wr_bin_next;
                    wr_gray     <= bin_to_gray(wr_bin_next);
                end else begin
                    overflow_toggle_core <= ~overflow_toggle_core;
                end
            end
        end
    end

    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            rd_bin                    <= '0;
            rd_gray                   <= '0;
            wr_gray_aclk_sync1        <= '0;
            wr_gray_aclk_sync2        <= '0;
            overflow_toggle_aclk_sync1 <= 1'b0;
            overflow_toggle_aclk_sync2 <= 1'b0;
            overflow_toggle_aclk_prev <= 1'b0;
            rd_word                   <= '0;
            rd_word_valid             <= 1'b0;
            rd_prefetch_valid         <= 1'b0;
            rd_prefetch_word          <= '0;
            m_axis_tdata              <= '0;
            m_axis_tvalid             <= 1'b0;
            m_axis_tlast              <= 1'b0;
            m_axis_tuser              <= '0;
            sts_fifo_overflow_cnt     <= 16'h0000;
            sts_back_pressure_cnt     <= 16'h0000;
        end else begin
            automatic logic [PTR_WIDTH-1:0] rd_bin_next;
            automatic logic output_can_load;
            automatic logic consume_prefetch;
            automatic logic consume_rd_word;
            automatic logic issue_read;

            wr_gray_aclk_sync1         <= wr_gray;
            wr_gray_aclk_sync2         <= wr_gray_aclk_sync1;
            overflow_toggle_aclk_sync1 <= overflow_toggle_core;
            overflow_toggle_aclk_sync2 <= overflow_toggle_aclk_sync1;

            if (overflow_toggle_aclk_sync2 != overflow_toggle_aclk_prev) begin
                sts_fifo_overflow_cnt <= sat_inc16(sts_fifo_overflow_cnt);
                overflow_toggle_aclk_prev <= overflow_toggle_aclk_sync2;
            end

            if (m_axis_tvalid && !m_axis_tready) begin
                sts_back_pressure_cnt <= sat_inc16(sts_back_pressure_cnt);
            end

            output_can_load = !m_axis_tvalid || m_axis_tready;
            consume_prefetch = output_can_load && rd_prefetch_valid;
            consume_rd_word = output_can_load && !rd_prefetch_valid && rd_word_valid;

            if (output_can_load) begin
                if (rd_prefetch_valid) begin
                    m_axis_tdata  <= rd_prefetch_word[TDATA_WIDTH-1:0];
                    m_axis_tvalid <= 1'b1;
                    m_axis_tlast  <= rd_prefetch_word[TDATA_WIDTH+1];
                    m_axis_tuser  <= '0;
                    m_axis_tuser[0] <= rd_prefetch_word[TDATA_WIDTH];
                    if (TUSER_WIDTH > 1) begin
                        m_axis_tuser[1] <= AXIS_TUSER_ERR_DEBUG ? rd_prefetch_word[TDATA_WIDTH+3] : 1'b0;
                    end
                end else if (rd_word_valid) begin
                    m_axis_tdata  <= rd_word[TDATA_WIDTH-1:0];
                    m_axis_tvalid <= 1'b1;
                    m_axis_tlast  <= rd_word[TDATA_WIDTH+1];
                    m_axis_tuser  <= '0;
                    m_axis_tuser[0] <= rd_word[TDATA_WIDTH];
                    if (TUSER_WIDTH > 1) begin
                        m_axis_tuser[1] <= AXIS_TUSER_ERR_DEBUG ? rd_word[TDATA_WIDTH+3] : 1'b0;
                    end
                end else begin
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast  <= 1'b0;
                    m_axis_tuser  <= '0;
                end
            end

            if (consume_prefetch) begin
                rd_prefetch_valid <= 1'b0;
            end

            if (consume_rd_word) begin
                rd_word_valid <= 1'b0;
            end else if (rd_word_valid && !rd_prefetch_valid) begin
                rd_prefetch_word <= rd_word;
                rd_prefetch_valid <= 1'b1;
                rd_word_valid <= 1'b0;
            end

            issue_read = !fifo_empty && (!rd_word_valid || consume_rd_word) && (!rd_prefetch_valid || consume_prefetch);
            if (issue_read) begin
                rd_word <= fifo_mem[rd_bin[ADDR_WIDTH-1:0]];
                rd_word_valid <= 1'b1;
                rd_bin_next = rd_bin + {{(PTR_WIDTH-1){1'b0}}, 1'b1};
                rd_bin      <= rd_bin_next;
                rd_gray     <= bin_to_gray(rd_bin_next);
            end
        end
    end

endmodule
