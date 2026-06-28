`timescale 1ns / 1ps

module csi2_payload_crc #(
    parameter logic [15:0] INIT = 16'hffff,
    parameter logic [15:0] REFLECTED_POLY = 16'h8408
) (
    input  logic        core_clk,
    input  logic        core_aresetn,

    input  logic [7:0]  payload_data,
    input  logic        payload_valid,
    input  logic        payload_first,
    input  logic        payload_last,
    input  logic [15:0] footer_data,
    input  logic        footer_valid,

    output logic        crc_check_valid,
    output logic        crc_match,
    output logic [15:0] crc_calc,
    output logic [15:0] crc_received,

    output logic [15:0] sts_crc_err_cnt,
    output logic [15:0] sts_crc_ok_cnt
);

    logic [15:0] crc_reg;

    function automatic [15:0] sat_inc16(input [15:0] value);
        if (value == 16'hffff) begin
            sat_inc16 = value;
        end else begin
            sat_inc16 = value + 16'd1;
        end
    endfunction

    function automatic [15:0] crc_update_byte(
        input logic [15:0] crc_in,
        input logic [7:0] data
    );
        automatic logic [15:0] crc_next;
        automatic logic feedback;
        crc_next = crc_in;
        for (int bit_idx = 0; bit_idx < 8; bit_idx++) begin
            feedback = crc_next[0] ^ data[bit_idx];
            crc_next = crc_next >> 1;
            if (feedback) begin
                crc_next = crc_next ^ REFLECTED_POLY;
            end
        end
        crc_update_byte = crc_next;
    endfunction

    always_ff @(posedge core_clk) begin
        if (!core_aresetn) begin
            crc_reg          <= INIT;
            crc_check_valid  <= 1'b0;
            crc_match        <= 1'b0;
            crc_calc         <= INIT;
            crc_received     <= 16'h0000;
            sts_crc_err_cnt  <= 16'h0000;
            sts_crc_ok_cnt   <= 16'h0000;
        end else begin
            automatic logic [15:0] next_crc;
            automatic logic match_now;

            crc_check_valid <= 1'b0;

            if (payload_valid) begin
                next_crc = crc_update_byte(payload_first ? INIT : crc_reg, payload_data);
                crc_reg  <= next_crc;
                crc_calc <= next_crc;
            end

            if (footer_valid) begin
                match_now = (crc_reg == footer_data);
                crc_received    <= footer_data;
                crc_calc        <= crc_reg;
                crc_match       <= match_now;
                crc_check_valid <= 1'b1;
                if (match_now) begin
                    sts_crc_ok_cnt <= sat_inc16(sts_crc_ok_cnt);
                end else begin
                    sts_crc_err_cnt <= sat_inc16(sts_crc_err_cnt);
                end
            end
        end
    end

endmodule
