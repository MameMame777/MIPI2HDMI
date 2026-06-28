`timescale 1ns / 1ps

module csi2_header_ecc (
    input  logic        core_clk,
    input  logic        core_aresetn,

    input  logic        hdr_valid,
    input  logic [31:0] hdr_raw,

    output logic        hdr_corr_valid,
    output logic [23:0] hdr_corr,
    output logic [7:0]  hdr_di,
    output logic [15:0] hdr_wc,
    output logic        hdr_ecc_corrected,
    output logic        hdr_ecc_uncorrectable,
    output logic        hdr_ecc_no_error,

    output logic [15:0] sts_ecc_corr_cnt,
    output logic [15:0] sts_ecc_uncorr_cnt
);

    logic        stage_valid;
    logic [23:0] stage_data;
    logic [7:0]  stage_ecc;
    logic [5:0]  stage_syndrome;

    function automatic [15:0] sat_inc16(input [15:0] value);
        if (value == 16'hffff) begin
            sat_inc16 = value;
        end else begin
            sat_inc16 = value + 16'd1;
        end
    endfunction

    function automatic [5:0] calc_ecc6(input logic [23:0] data);
        calc_ecc6[0] = data[0]^data[1]^data[2]^data[4]^data[5]^data[7]^data[10]^data[11]^data[13]^data[16]^data[20]^data[21]^data[22]^data[23];
        calc_ecc6[1] = data[0]^data[1]^data[3]^data[4]^data[6]^data[8]^data[10]^data[12]^data[14]^data[17]^data[20]^data[21]^data[22]^data[23];
        calc_ecc6[2] = data[0]^data[2]^data[3]^data[5]^data[6]^data[9]^data[11]^data[12]^data[15]^data[18]^data[20]^data[21]^data[22];
        calc_ecc6[3] = data[1]^data[2]^data[3]^data[7]^data[8]^data[9]^data[13]^data[14]^data[15]^data[19]^data[20]^data[21]^data[23];
        calc_ecc6[4] = data[4]^data[5]^data[6]^data[7]^data[8]^data[9]^data[16]^data[17]^data[18]^data[19]^data[20]^data[22]^data[23];
        calc_ecc6[5] = data[10]^data[11]^data[12]^data[13]^data[14]^data[15]^data[16]^data[17]^data[18]^data[19]^data[21]^data[22]^data[23];
    endfunction

    function automatic [5:0] bit_syndrome(input int bit_idx);
        automatic logic [23:0] onehot;
        onehot = 24'h000000;
        onehot[bit_idx] = 1'b1;
        bit_syndrome = calc_ecc6(onehot);
    endfunction

    function automatic int decode_data_bit(input logic [5:0] syndrome);
        decode_data_bit = -1;
        for (int idx = 0; idx < 24; idx++) begin
            if (syndrome == bit_syndrome(idx)) begin
                decode_data_bit = idx;
            end
        end
    endfunction

    function automatic logic is_onehot6(input logic [5:0] value);
        is_onehot6 = (value != 6'b000000) && ((value & (value - 6'd1)) == 6'b000000);
    endfunction

    always_ff @(posedge core_clk) begin
        if (!core_aresetn) begin
            stage_valid          <= 1'b0;
            stage_data           <= 24'h000000;
            stage_ecc            <= 8'h00;
            stage_syndrome       <= 6'h00;
            hdr_corr_valid       <= 1'b0;
            hdr_corr             <= 24'h000000;
            hdr_di               <= 8'h00;
            hdr_wc               <= 16'h0000;
            hdr_ecc_corrected    <= 1'b0;
            hdr_ecc_uncorrectable <= 1'b0;
            hdr_ecc_no_error     <= 1'b0;
            sts_ecc_corr_cnt     <= 16'h0000;
            sts_ecc_uncorr_cnt   <= 16'h0000;
        end else begin
            automatic logic [23:0] corrected_data;
            automatic int corrected_bit;
            automatic logic data_bit_error;
            automatic logic ecc_bit_error;
            automatic logic uncorrectable;

            hdr_corr_valid        <= 1'b0;
            hdr_ecc_corrected     <= 1'b0;
            hdr_ecc_uncorrectable <= 1'b0;
            hdr_ecc_no_error      <= 1'b0;

            stage_valid    <= hdr_valid;
            stage_data     <= hdr_raw[23:0];
            stage_ecc      <= hdr_raw[31:24];
            stage_syndrome <= hdr_raw[29:24] ^ calc_ecc6(hdr_raw[23:0]);

            if (stage_valid) begin
                corrected_data = stage_data;
                corrected_bit  = decode_data_bit(stage_syndrome);
                data_bit_error = (stage_syndrome != 6'h00) && (corrected_bit >= 0);
                ecc_bit_error  = (stage_syndrome != 6'h00) && is_onehot6(stage_syndrome) && !data_bit_error;
                uncorrectable  = (stage_syndrome != 6'h00) && !data_bit_error && !ecc_bit_error;

                if (data_bit_error) begin
                    corrected_data[corrected_bit] = ~corrected_data[corrected_bit];
                end

                hdr_corr              <= corrected_data;
                hdr_di                <= corrected_data[7:0];
                hdr_wc                <= corrected_data[23:8];
                hdr_corr_valid        <= 1'b1;
                hdr_ecc_no_error      <= (stage_syndrome == 6'h00);
                hdr_ecc_corrected     <= data_bit_error || ecc_bit_error;
                hdr_ecc_uncorrectable <= uncorrectable;

                if (data_bit_error || ecc_bit_error) begin
                    sts_ecc_corr_cnt <= sat_inc16(sts_ecc_corr_cnt);
                end else if (uncorrectable) begin
                    sts_ecc_uncorr_cnt <= sat_inc16(sts_ecc_uncorr_cnt);
                end
            end
        end
    end

endmodule
