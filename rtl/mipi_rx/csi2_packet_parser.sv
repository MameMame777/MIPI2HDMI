`timescale 1ns / 1ps

module csi2_packet_parser #(
    parameter int IN_WIDTH = 16,
    parameter int WC_MAX = 16383,
    parameter int FIFO_DEPTH = 32
) (
    input  logic                    core_clk,
    input  logic                    core_aresetn,

    input  logic [IN_WIDTH-1:0]     s_byte_data,
    input  logic [IN_WIDTH/8-1:0]   s_byte_keep,
    input  logic                    s_byte_valid,
    input  logic                    s_byte_sop,
    input  logic                    s_byte_eop,

    output logic                    ecc_hdr_valid,
    output logic [31:0]             ecc_hdr_raw,
    input  logic                    ecc_hdr_corr_valid,
    input  logic [7:0]              ecc_hdr_di,
    input  logic [15:0]             ecc_hdr_wc,
    input  logic                    ecc_hdr_uncorrectable,

    output logic                    m_pkt_hdr_valid,
    output logic [31:0]             m_pkt_hdr_raw,
    output logic [7:0]              m_pkt_di,
    output logic [15:0]             m_pkt_wc,
    output logic                    m_pkt_is_long,
    output logic                    m_pkt_is_short,
    output logic                    m_pkt_ecc_uncorrectable,

    output logic [7:0]              m_payload_data,
    output logic                    m_payload_valid,
    output logic                    m_payload_first,
    output logic                    m_payload_last,

    output logic [15:0]             m_footer_data,
    output logic                    m_footer_valid,

    output logic                    m_pkt_done,

    output logic [15:0]             sts_short_pkt_cnt,
    output logic [15:0]             sts_long_pkt_cnt,
    output logic [15:0]             sts_pkt_trunc_cnt
);

    localparam int BYTE_LANES = IN_WIDTH / 8;
    localparam int FIFO_PTR_WIDTH = (FIFO_DEPTH <= 2) ? 1 : $clog2(FIFO_DEPTH);
    localparam int FIFO_CNT_WIDTH = $clog2(FIFO_DEPTH + 1);

    typedef enum logic [2:0] {
        ST_IDLE,
        ST_HDR1,
        ST_HDR2,
        ST_HDR3,
        ST_WAIT_ECC,
        ST_PAYLOAD,
        ST_FOOTER0,
        ST_FOOTER1
    } parser_state_t;

    parser_state_t state;

    logic [7:0] byte_fifo [FIFO_DEPTH];
    logic       sop_fifo  [FIFO_DEPTH];
    logic       eop_fifo  [FIFO_DEPTH];

    logic [FIFO_PTR_WIDTH-1:0] wr_ptr;
    logic [FIFO_PTR_WIDTH-1:0] rd_ptr;
    logic [FIFO_CNT_WIDTH-1:0] fifo_count;

    logic [31:0] hdr_raw_reg;
    logic [7:0]  pkt_di_reg;
    logic [15:0] pkt_wc_reg;
    logic        pkt_is_long_reg;
    logic        pkt_is_short_reg;
    logic        pkt_ecc_uncorr_reg;
    logic [15:0] payload_count;
    logic [7:0]  footer_lsb;

    logic [7:0]  pop_byte;
    logic        pop_sop;
    logic        pop_eop;
    logic        pop_valid;

    function automatic [FIFO_PTR_WIDTH-1:0] ptr_inc(input [FIFO_PTR_WIDTH-1:0] ptr);
        if (ptr == FIFO_DEPTH - 1) begin
            ptr_inc = '0;
        end else begin
            ptr_inc = ptr + {{(FIFO_PTR_WIDTH-1){1'b0}}, 1'b1};
        end
    endfunction

    function automatic [FIFO_PTR_WIDTH-1:0] ptr_add(
        input [FIFO_PTR_WIDTH-1:0] ptr,
        input int unsigned increment
    );
        automatic int unsigned sum;
        sum = ptr + increment;
        while (sum >= FIFO_DEPTH) begin
            sum = sum - FIFO_DEPTH;
        end
        ptr_add = sum[FIFO_PTR_WIDTH-1:0];
    endfunction

    function automatic [15:0] sat_inc16(input [15:0] value);
        if (value == 16'hffff) begin
            sat_inc16 = value;
        end else begin
            sat_inc16 = value + 16'd1;
        end
    endfunction

    function automatic int unsigned keep_count(input logic [BYTE_LANES-1:0] keep);
        keep_count = 0;
        for (int idx = 0; idx < BYTE_LANES; idx++) begin
            if (keep[idx]) begin
                keep_count++;
            end
        end
    endfunction

    function automatic int unsigned last_keep_index(input logic [BYTE_LANES-1:0] keep);
        last_keep_index = 0;
        for (int idx = 0; idx < BYTE_LANES; idx++) begin
            if (keep[idx]) begin
                last_keep_index = idx;
            end
        end
    endfunction

    assign pop_valid = (fifo_count != 0);
    assign pop_byte  = byte_fifo[rd_ptr];
    assign pop_sop   = sop_fifo[rd_ptr];
    assign pop_eop   = eop_fifo[rd_ptr];

    always_ff @(posedge core_clk) begin
        if (!core_aresetn) begin
            state                  <= ST_IDLE;
            wr_ptr                 <= '0;
            rd_ptr                 <= '0;
            fifo_count             <= '0;
            hdr_raw_reg            <= 32'h0;
            pkt_di_reg             <= 8'h00;
            pkt_wc_reg             <= 16'h0000;
            pkt_is_long_reg        <= 1'b0;
            pkt_is_short_reg       <= 1'b0;
            pkt_ecc_uncorr_reg     <= 1'b0;
            payload_count          <= 16'h0000;
            footer_lsb             <= 8'h00;
            ecc_hdr_valid          <= 1'b0;
            ecc_hdr_raw            <= 32'h0;
            m_pkt_hdr_valid        <= 1'b0;
            m_pkt_hdr_raw          <= 32'h0;
            m_pkt_di               <= 8'h00;
            m_pkt_wc               <= 16'h0000;
            m_pkt_is_long          <= 1'b0;
            m_pkt_is_short         <= 1'b0;
            m_pkt_ecc_uncorrectable <= 1'b0;
            m_payload_data         <= 8'h00;
            m_payload_valid        <= 1'b0;
            m_payload_first        <= 1'b0;
            m_payload_last         <= 1'b0;
            m_footer_data          <= 16'h0000;
            m_footer_valid         <= 1'b0;
            m_pkt_done             <= 1'b0;
            sts_short_pkt_cnt      <= 16'h0000;
            sts_long_pkt_cnt       <= 16'h0000;
            sts_pkt_trunc_cnt      <= 16'h0000;
        end else begin
            automatic int unsigned valid_bytes;
            automatic int unsigned last_idx;
            automatic int unsigned free_slots;
            automatic int unsigned push_idx;
            automatic logic [FIFO_CNT_WIDTH:0] push_count_vec;
            automatic logic [FIFO_PTR_WIDTH-1:0] wr_ptr_next;
            automatic logic [FIFO_CNT_WIDTH:0] fifo_count_next;
            automatic logic can_push_word;
            automatic logic consume_byte;
            automatic logic flush_fifo;
            automatic logic [7:0] resolved_di;
            automatic logic [15:0] resolved_wc;
            automatic logic resolved_is_long;

            ecc_hdr_valid           <= 1'b0;
            m_pkt_hdr_valid         <= 1'b0;
            m_payload_valid         <= 1'b0;
            m_payload_first         <= 1'b0;
            m_payload_last          <= 1'b0;
            m_footer_valid          <= 1'b0;
            m_pkt_done              <= 1'b0;
            m_pkt_ecc_uncorrectable <= 1'b0;

            consume_byte = 1'b0;
            flush_fifo   = 1'b0;

            if (pop_valid) begin
                unique case (state)
                    ST_IDLE: begin
                        if (pop_sop) begin
                            hdr_raw_reg[7:0] <= pop_byte;
                            state            <= ST_HDR1;
                        end
                        consume_byte = 1'b1;
                    end

                    ST_HDR1: begin
                        if (pop_sop) begin
                            sts_pkt_trunc_cnt <= sat_inc16(sts_pkt_trunc_cnt);
                            hdr_raw_reg[7:0]  <= pop_byte;
                            state             <= ST_HDR1;
                        end else begin
                            hdr_raw_reg[15:8] <= pop_byte;
                            state             <= ST_HDR2;
                        end
                        if (pop_eop) begin
                            sts_pkt_trunc_cnt <= sat_inc16(sts_pkt_trunc_cnt);
                            state             <= ST_IDLE;
                        end
                        consume_byte = 1'b1;
                    end

                    ST_HDR2: begin
                        if (pop_sop) begin
                            sts_pkt_trunc_cnt <= sat_inc16(sts_pkt_trunc_cnt);
                            hdr_raw_reg[7:0]  <= pop_byte;
                            state             <= ST_HDR1;
                        end else begin
                            hdr_raw_reg[23:16] <= pop_byte;
                            state              <= ST_HDR3;
                        end
                        if (pop_eop) begin
                            sts_pkt_trunc_cnt <= sat_inc16(sts_pkt_trunc_cnt);
                            state             <= ST_IDLE;
                        end
                        consume_byte = 1'b1;
                    end

                    ST_HDR3: begin
                        if (pop_sop) begin
                            sts_pkt_trunc_cnt <= sat_inc16(sts_pkt_trunc_cnt);
                            hdr_raw_reg[7:0]  <= pop_byte;
                            state             <= ST_HDR1;
                        end else begin
                            hdr_raw_reg[31:24] <= pop_byte;
                            ecc_hdr_raw        <= {pop_byte, hdr_raw_reg[23:0]};
                            ecc_hdr_valid      <= 1'b1;
                            state              <= ST_WAIT_ECC;
                        end
                        consume_byte = 1'b1;
                    end

                    default: begin
                    end
                endcase
            end

            if (state == ST_WAIT_ECC && ecc_hdr_corr_valid) begin
                resolved_di      = ecc_hdr_uncorrectable ? hdr_raw_reg[7:0] : ecc_hdr_di;
                resolved_wc      = ecc_hdr_uncorrectable ? hdr_raw_reg[23:8] : ecc_hdr_wc;
                resolved_is_long = (resolved_di[5:0] >= 6'h10);

                pkt_di_reg         <= resolved_di;
                pkt_wc_reg         <= resolved_wc;
                pkt_is_long_reg    <= resolved_is_long;
                pkt_is_short_reg   <= !resolved_is_long;
                pkt_ecc_uncorr_reg <= ecc_hdr_uncorrectable;

                m_pkt_hdr_valid         <= 1'b1;
                m_pkt_hdr_raw           <= hdr_raw_reg;
                m_pkt_di                <= resolved_di;
                m_pkt_wc                <= resolved_wc;
                m_pkt_is_long           <= resolved_is_long;
                m_pkt_is_short          <= !resolved_is_long;
                m_pkt_ecc_uncorrectable <= ecc_hdr_uncorrectable;

                if (resolved_is_long && (resolved_wc <= WC_MAX)) begin
                    payload_count     <= 16'h0000;
                    sts_long_pkt_cnt  <= sat_inc16(sts_long_pkt_cnt);
                    if (resolved_wc == 16'h0000) begin
                        state <= ST_FOOTER0;
                    end else begin
                        state <= ST_PAYLOAD;
                    end
                end else if (resolved_is_long) begin
                    sts_pkt_trunc_cnt <= sat_inc16(sts_pkt_trunc_cnt);
                    m_pkt_done        <= 1'b1;
                    state             <= ST_IDLE;
                    flush_fifo        = 1'b1;
                end else begin
                    sts_short_pkt_cnt <= sat_inc16(sts_short_pkt_cnt);
                    m_pkt_done        <= 1'b1;
                    state             <= ST_IDLE;
                end
            end

            if (pop_valid && state == ST_PAYLOAD) begin
                if (pop_sop) begin
                    sts_pkt_trunc_cnt <= sat_inc16(sts_pkt_trunc_cnt);
                    hdr_raw_reg[7:0]  <= pop_byte;
                    m_pkt_done        <= 1'b1;
                    state             <= ST_HDR1;
                end else begin
                    m_payload_data  <= pop_byte;
                    m_payload_valid <= 1'b1;
                    m_payload_first <= (payload_count == 16'h0000);
                    m_payload_last  <= (payload_count == (pkt_wc_reg - 16'd1));
                    payload_count   <= payload_count + 16'd1;

                    if (payload_count == (pkt_wc_reg - 16'd1)) begin
                        state <= ST_FOOTER0;
                    end else if (pop_eop) begin
                        sts_pkt_trunc_cnt <= sat_inc16(sts_pkt_trunc_cnt);
                        m_pkt_done        <= 1'b1;
                        state             <= ST_IDLE;
                    end
                end
                consume_byte = 1'b1;
            end

            if (pop_valid && state == ST_FOOTER0) begin
                if (pop_sop) begin
                    sts_pkt_trunc_cnt <= sat_inc16(sts_pkt_trunc_cnt);
                    hdr_raw_reg[7:0]  <= pop_byte;
                    m_pkt_done        <= 1'b1;
                    state             <= ST_HDR1;
                end else begin
                    footer_lsb <= pop_byte;
                    state      <= ST_FOOTER1;
                    if (pop_eop) begin
                        sts_pkt_trunc_cnt <= sat_inc16(sts_pkt_trunc_cnt);
                        m_pkt_done        <= 1'b1;
                        state             <= ST_IDLE;
                    end
                end
                consume_byte = 1'b1;
            end

            if (pop_valid && state == ST_FOOTER1) begin
                if (pop_sop) begin
                    sts_pkt_trunc_cnt <= sat_inc16(sts_pkt_trunc_cnt);
                    hdr_raw_reg[7:0]  <= pop_byte;
                    m_pkt_done        <= 1'b1;
                    state             <= ST_HDR1;
                end else begin
                    m_footer_data  <= {pop_byte, footer_lsb};
                    m_footer_valid <= 1'b1;
                    m_pkt_done     <= 1'b1;
                    state          <= ST_IDLE;
                end
                consume_byte = 1'b1;
            end

            if (flush_fifo) begin
                rd_ptr     <= wr_ptr;
                fifo_count <= '0;
            end else begin
                wr_ptr_next     = wr_ptr;
                fifo_count_next = fifo_count;

                if (consume_byte && fifo_count_next != 0) begin
                    rd_ptr          <= ptr_inc(rd_ptr);
                    fifo_count_next = fifo_count_next - 1'b1;
                end

                valid_bytes = (s_byte_valid) ? keep_count(s_byte_keep) : 0;
                last_idx    = last_keep_index(s_byte_keep);
                free_slots  = FIFO_DEPTH - fifo_count_next;
                push_idx    = 0;
                can_push_word = (valid_bytes <= free_slots);

                if (!can_push_word) begin
                    sts_pkt_trunc_cnt <= sat_inc16(sts_pkt_trunc_cnt);
                end else begin
                    for (int lane = 0; lane < BYTE_LANES; lane++) begin
                        if (s_byte_valid && s_byte_keep[lane]) begin
                            byte_fifo[wr_ptr_next] <= s_byte_data[(lane * 8) +: 8];
                            sop_fifo[wr_ptr_next]  <= s_byte_sop && (push_idx == 0);
                            eop_fifo[wr_ptr_next]  <= s_byte_eop && (lane == last_idx);
                            wr_ptr_next            = ptr_inc(wr_ptr_next);
                            push_idx++;
                        end
                    end
                end

                push_count_vec = push_idx[FIFO_CNT_WIDTH:0];
                wr_ptr     <= wr_ptr_next;
                fifo_count <= fifo_count_next + push_count_vec;
            end
        end
    end

endmodule
