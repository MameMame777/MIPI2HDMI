`timescale 1ns / 1ps

module byte_to_core_cdc #(
    parameter int IN_WIDTH = 16,
    parameter int KEEP_WIDTH = IN_WIDTH / 8,
    parameter int FIFO_DEPTH = 256,
    parameter int CORE_OUTPUT_INTERVAL = 1
) (
    input  logic                  byte_clk,
    input  logic                  byte_aresetn,
    input  logic                  core_clk,
    input  logic                  core_aresetn,

    input  logic [IN_WIDTH-1:0]   s_byte_data,
    input  logic [KEEP_WIDTH-1:0] s_byte_keep,
    input  logic                  s_byte_valid,
    input  logic                  s_byte_sop,
    input  logic                  s_byte_eop,

    output logic [IN_WIDTH-1:0]   m_byte_data,
    output logic [KEEP_WIDTH-1:0] m_byte_keep,
    output logic                  m_byte_valid,
    output logic                  m_byte_sop,
    output logic                  m_byte_eop,

    output logic [15:0]           sts_lane_fifo_ovf_cnt
);

    localparam int SIDE_WIDTH = KEEP_WIDTH + 2;
    localparam int WORD_WIDTH = IN_WIDTH + SIDE_WIDTH;
    localparam int ADDR_WIDTH = (FIFO_DEPTH <= 2) ? 1 : $clog2(FIFO_DEPTH);
    localparam int PTR_WIDTH = ADDR_WIDTH + 1;
    localparam int OUTPUT_INTERVAL = (CORE_OUTPUT_INTERVAL < 1) ? 1 : CORE_OUTPUT_INTERVAL;
    localparam int OUTPUT_GAP_WIDTH = (OUTPUT_INTERVAL <= 1) ? 1 : $clog2(OUTPUT_INTERVAL);

    logic [WORD_WIDTH-1:0] fifo_mem [FIFO_DEPTH];

    logic [PTR_WIDTH-1:0] wr_bin;
    logic [PTR_WIDTH-1:0] wr_gray;
    logic [PTR_WIDTH-1:0] rd_gray_byte_sync1;
    logic [PTR_WIDTH-1:0] rd_gray_byte_sync2;

    logic [PTR_WIDTH-1:0] rd_bin;
    logic [PTR_WIDTH-1:0] rd_gray;
    logic [PTR_WIDTH-1:0] wr_gray_core_sync1;
    logic [PTR_WIDTH-1:0] wr_gray_core_sync2;

    logic overflow_toggle_byte;
    logic overflow_toggle_core_sync1;
    logic overflow_toggle_core_sync2;
    logic overflow_toggle_core_prev;

    logic fifo_full;
    logic fifo_empty;
    logic [WORD_WIDTH-1:0] rd_word;
    logic [OUTPUT_GAP_WIDTH-1:0] output_gap_count;
    logic core_read_enable;

    function automatic [PTR_WIDTH-1:0] bin_to_gray(input logic [PTR_WIDTH-1:0] value);
        bin_to_gray = (value >> 1) ^ value;
    endfunction

    function automatic [15:0] sat_inc16(input logic [15:0] value);
        if (value == 16'hffff) begin
            sat_inc16 = value;
        end else begin
            sat_inc16 = value + 16'd1;
        end
    endfunction

    assign fifo_full = (bin_to_gray(wr_bin + {{(PTR_WIDTH-1){1'b0}}, 1'b1}) ==
                        {~rd_gray_byte_sync2[PTR_WIDTH-1:PTR_WIDTH-2], rd_gray_byte_sync2[PTR_WIDTH-3:0]});
    assign fifo_empty = (rd_gray == wr_gray_core_sync2);
    assign rd_word = fifo_mem[rd_bin[ADDR_WIDTH-1:0]];
    assign core_read_enable = !fifo_empty && (output_gap_count == '0);

    always_ff @(posedge byte_clk) begin
        if (!byte_aresetn) begin
            wr_bin <= '0;
            wr_gray <= '0;
            rd_gray_byte_sync1 <= '0;
            rd_gray_byte_sync2 <= '0;
            overflow_toggle_byte <= 1'b0;
        end else begin
            automatic logic [PTR_WIDTH-1:0] wr_bin_next;

            rd_gray_byte_sync1 <= rd_gray;
            rd_gray_byte_sync2 <= rd_gray_byte_sync1;

            if (s_byte_valid) begin
                if (!fifo_full) begin
                    fifo_mem[wr_bin[ADDR_WIDTH-1:0]] <= {s_byte_eop, s_byte_sop, s_byte_keep, s_byte_data};
                    wr_bin_next = wr_bin + {{(PTR_WIDTH-1){1'b0}}, 1'b1};
                    wr_bin <= wr_bin_next;
                    wr_gray <= bin_to_gray(wr_bin_next);
                end else begin
                    overflow_toggle_byte <= ~overflow_toggle_byte;
                end
            end
        end
    end

    always_ff @(posedge core_clk) begin
        if (!core_aresetn) begin
            rd_bin <= '0;
            rd_gray <= '0;
            wr_gray_core_sync1 <= '0;
            wr_gray_core_sync2 <= '0;
            overflow_toggle_core_sync1 <= 1'b0;
            overflow_toggle_core_sync2 <= 1'b0;
            overflow_toggle_core_prev <= 1'b0;
            output_gap_count <= '0;
            m_byte_data <= '0;
            m_byte_keep <= '0;
            m_byte_valid <= 1'b0;
            m_byte_sop <= 1'b0;
            m_byte_eop <= 1'b0;
            sts_lane_fifo_ovf_cnt <= 16'h0000;
        end else begin
            automatic logic [PTR_WIDTH-1:0] rd_bin_next;

            wr_gray_core_sync1 <= wr_gray;
            wr_gray_core_sync2 <= wr_gray_core_sync1;
            overflow_toggle_core_sync1 <= overflow_toggle_byte;
            overflow_toggle_core_sync2 <= overflow_toggle_core_sync1;

            if (overflow_toggle_core_sync2 != overflow_toggle_core_prev) begin
                sts_lane_fifo_ovf_cnt <= sat_inc16(sts_lane_fifo_ovf_cnt);
                overflow_toggle_core_prev <= overflow_toggle_core_sync2;
            end

            if (core_read_enable) begin
                m_byte_data <= rd_word[IN_WIDTH-1:0];
                m_byte_keep <= rd_word[IN_WIDTH +: KEEP_WIDTH];
                m_byte_sop <= rd_word[IN_WIDTH + KEEP_WIDTH];
                m_byte_eop <= rd_word[IN_WIDTH + KEEP_WIDTH + 1];
                m_byte_valid <= 1'b1;
                rd_bin_next = rd_bin + {{(PTR_WIDTH-1){1'b0}}, 1'b1};
                rd_bin <= rd_bin_next;
                rd_gray <= bin_to_gray(rd_bin_next);
                if (OUTPUT_INTERVAL > 1) begin
                    output_gap_count <= OUTPUT_INTERVAL - 1;
                end
            end else begin
                m_byte_valid <= 1'b0;
                m_byte_keep <= '0;
                m_byte_sop <= 1'b0;
                m_byte_eop <= 1'b0;
                if (output_gap_count != '0) begin
                    output_gap_count <= output_gap_count - 1'b1;
                end
            end
        end
    end

endmodule
