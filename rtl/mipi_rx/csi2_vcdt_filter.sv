`timescale 1ns / 1ps

module csi2_vcdt_filter #(
    parameter int NUM_VC = 4,
    parameter int NUM_DT_RAW = 2
) (
    input  logic        core_clk,
    input  logic        core_aresetn,

    input  logic [1:0]  cfg_expected_vc,
    input  logic [5:0]  cfg_expected_dt,
    input  logic        cfg_pass_short,
    input  logic        cfg_pass_emb_data,

    input  logic        pkt_hdr_valid,
    input  logic [7:0]  pkt_di,
    input  logic [15:0] pkt_wc,
    input  logic        pkt_is_long,
    input  logic        pkt_is_short,
    input  logic        pkt_done,
    input  logic        ecc_corrected,
    input  logic        ecc_uncorrectable,
    input  logic        crc_check_valid,
    input  logic        crc_match,
    input  logic [7:0]  payload_data,
    input  logic        payload_valid,
    input  logic        payload_first,
    input  logic        payload_last,

    output logic [7:0]  out_pkt_di,
    output logic [15:0] out_pkt_wc,
    output logic        out_pkt_is_short,
    output logic        out_pkt_is_long,
    output logic        out_pkt_start,
    output logic        out_pkt_end,
    output logic        out_pkt_err,
    output logic [7:0]  out_payload_data,
    output logic        out_payload_valid,
    output logic        out_payload_first,
    output logic        out_payload_last,

    output logic [15:0] sts_drop_vc_cnt,
    output logic [15:0] sts_drop_dt_cnt
);

    localparam logic [5:0] DT_EMBEDDED_DATA = 6'h12;

    logic        packet_admit;
    logic        packet_err;
    logic [7:0]  active_di;
    logic [15:0] active_wc;
    logic        active_is_short;
    logic        active_is_long;

    function automatic [15:0] sat_inc16(input [15:0] value);
        if (value == 16'hffff) begin
            sat_inc16 = value;
        end else begin
            sat_inc16 = value + 16'd1;
        end
    endfunction

    function automatic logic accept_dt(
        input logic [5:0] dt,
        input logic is_short,
        input logic is_long,
        input logic [5:0] expected_dt,
        input logic pass_short,
        input logic pass_emb_data
    );
        if (is_short) begin
            accept_dt = pass_short;
        end else if (is_long && dt == DT_EMBEDDED_DATA) begin
            accept_dt = pass_emb_data;
        end else begin
            accept_dt = is_long && (dt == expected_dt);
        end
    endfunction

    always_ff @(posedge core_clk) begin
        if (!core_aresetn) begin
            packet_admit      <= 1'b0;
            packet_err        <= 1'b0;
            active_di         <= 8'h00;
            active_wc         <= 16'h0000;
            active_is_short   <= 1'b0;
            active_is_long    <= 1'b0;
            out_pkt_di        <= 8'h00;
            out_pkt_wc        <= 16'h0000;
            out_pkt_is_short  <= 1'b0;
            out_pkt_is_long   <= 1'b0;
            out_pkt_start     <= 1'b0;
            out_pkt_end       <= 1'b0;
            out_pkt_err       <= 1'b0;
            out_payload_data  <= 8'h00;
            out_payload_valid <= 1'b0;
            out_payload_first <= 1'b0;
            out_payload_last  <= 1'b0;
            sts_drop_vc_cnt   <= 16'h0000;
            sts_drop_dt_cnt   <= 16'h0000;
        end else begin
            automatic logic next_admit;
            automatic logic next_accept_vc;
            automatic logic next_accept_dt;
            automatic logic next_pkt_err;
            automatic logic done_admit;
            automatic logic [7:0] done_di;
            automatic logic [15:0] done_wc;
            automatic logic done_is_short;
            automatic logic done_is_long;

            out_pkt_start     <= 1'b0;
            out_pkt_end       <= 1'b0;
            out_pkt_err       <= 1'b0;
            out_payload_valid <= 1'b0;
            out_payload_first <= 1'b0;
            out_payload_last  <= 1'b0;

            next_pkt_err = packet_err | ecc_uncorrectable |
                           (crc_check_valid && !crc_match);

            if (pkt_hdr_valid) begin
                next_accept_vc = (pkt_di[7:6] == cfg_expected_vc);
                next_accept_dt = accept_dt(
                    pkt_di[5:0],
                    pkt_is_short,
                    pkt_is_long,
                    cfg_expected_dt,
                    cfg_pass_short,
                    cfg_pass_emb_data
                );
                next_admit = next_accept_vc && next_accept_dt;
                next_pkt_err = ecc_uncorrectable;

                packet_admit    <= next_admit;
                packet_err      <= next_pkt_err;
                active_di       <= pkt_di;
                active_wc       <= pkt_wc;
                active_is_short <= pkt_is_short;
                active_is_long  <= pkt_is_long;

                if (!next_accept_vc) begin
                    sts_drop_vc_cnt <= sat_inc16(sts_drop_vc_cnt);
                end else if (!next_accept_dt) begin
                    sts_drop_dt_cnt <= sat_inc16(sts_drop_dt_cnt);
                end

                if (next_admit) begin
                    out_pkt_di       <= pkt_di;
                    out_pkt_wc       <= pkt_wc;
                    out_pkt_is_short <= pkt_is_short;
                    out_pkt_is_long  <= pkt_is_long;
                    out_pkt_start    <= 1'b1;
                end
            end else begin
                packet_err <= next_pkt_err;
            end

            if (payload_valid && packet_admit) begin
                out_payload_data  <= payload_data;
                out_payload_valid <= 1'b1;
                out_payload_first <= payload_first;
                out_payload_last  <= payload_last;
            end

            if (pkt_done) begin
                done_admit    = pkt_hdr_valid ? next_admit : packet_admit;
                done_di       = pkt_hdr_valid ? pkt_di : active_di;
                done_wc       = pkt_hdr_valid ? pkt_wc : active_wc;
                done_is_short = pkt_hdr_valid ? pkt_is_short : active_is_short;
                done_is_long  = pkt_hdr_valid ? pkt_is_long : active_is_long;

                if (done_admit) begin
                    out_pkt_di       <= done_di;
                    out_pkt_wc       <= done_wc;
                    out_pkt_is_short <= done_is_short;
                    out_pkt_is_long  <= done_is_long;
                    out_pkt_end      <= 1'b1;
                    out_pkt_err      <= next_pkt_err;
                end

                packet_admit    <= 1'b0;
                packet_err      <= 1'b0;
                active_is_short <= 1'b0;
                active_is_long  <= 1'b0;
            end
        end
    end

endmodule
