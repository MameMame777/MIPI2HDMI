`timescale 1ns / 1ps
`default_nettype none

module tb_dphy_hs_stream_boundary;
    logic rst_n;
    logic parser_aresetn;
    logic idelay_ref_clk;
    logic hs_clk_p;
    logic hs_clk_n;
    logic [1:0] data_hs_p;
    logic [1:0] data_hs_n;
    logic [1:0] data_lp_p;
    logic [1:0] data_lp_n;
    logic byte_clk;

    logic [15:0] stream_byte_data;
    logic [1:0] stream_byte_keep;
    logic stream_byte_valid;
    logic stream_byte_sop;
    logic stream_byte_eop;

    logic sync_header_valid;
    logic [7:0] sync_header_di;
    logic [15:0] sync_header_wc;
    logic [7:0] sync_header_ecc;
    logic [2:0] sync_header_bit_offset_lane0;
    logic [2:0] sync_header_bit_offset_lane1;
    logic [2:0] sync_header_pairing;
    logic [3:0] sync_header_score;
    logic sync_header_ecc_no_error;
    logic [7:0] trace_slot_valid;
    logic [7:0][7:0] trace_slot_lane0_candidate;
    logic [7:0][7:0] trace_slot_lane1_candidate;
    logic [7:0] live_trace_seq;
    logic [7:0] live_trace_slot_valid;
    logic [7:0][7:0] live_trace_slot_lane0_candidate;
    logic [7:0][7:0] live_trace_slot_lane1_candidate;
    logic [7:0][7:0] live_trace_slot_lane0_aligned;
    logic [7:0][7:0] live_trace_slot_lane1_aligned;
    logic [7:0] live_trace_slot_sot_hit_lane0;
    logic [7:0] live_trace_slot_sot_hit_lane1;

    logic ecc_hdr_valid;
    logic [31:0] ecc_hdr_raw;
    logic ecc_hdr_corr_valid;
    logic [23:0] ecc_hdr_corr;
    logic [7:0] ecc_hdr_di;
    logic [15:0] ecc_hdr_wc;
    logic ecc_hdr_corrected;
    logic ecc_hdr_uncorrectable;
    logic ecc_hdr_no_error;
    logic [15:0] ecc_corr_count;
    logic [15:0] ecc_uncorr_count;

    logic parser_pkt_hdr_valid;
    logic [31:0] parser_pkt_hdr_raw;
    logic [7:0] parser_pkt_di;
    logic [15:0] parser_pkt_wc;
    logic parser_pkt_is_long;
    logic parser_pkt_is_short;
    logic parser_pkt_ecc_uncorrectable;
    logic [7:0] parser_payload_data;
    logic parser_payload_valid;
    logic parser_payload_first;
    logic parser_payload_last;
    logic [15:0] parser_footer_data;
    logic parser_footer_valid;
    logic parser_pkt_done;
    logic [15:0] parser_short_count;
    logic [15:0] parser_long_count;
    logic [15:0] parser_trunc_count;

    logic parser_last_hdr_seen;
    logic [7:0] parser_last_pkt_di;
    logic [15:0] parser_last_pkt_wc;
    logic parser_last_pkt_ecc_uncorrectable;
    logic parser_last_ecc_seen;
    logic parser_last_ecc_no_error;
    logic parser_last_ecc_corrected;
    int unsigned parser_header_count;
    int unsigned parser_payload_byte_count;

    logic [15:0] captured_stream_data [0:7];
    logic captured_stream_sop [0:7];
    int unsigned captured_stream_count;
    logic [15:0] stream_sop_word0_log [0:15];
    logic [15:0] stream_sop_word1_log [0:15];
    logic stream_sop_seen_log [0:15];
    int unsigned stream_packet_count;
    int unsigned stream_pending_index;
    logic stream_second_pending;
    logic [7:0] parser_di_log [0:15];
    logic [15:0] parser_wc_log [0:15];
    logic parser_ecc_uncorrectable_log [0:15];
    logic ecc_no_error_log [0:15];
    logic ecc_corrected_log [0:15];
    logic ecc_uncorrectable_log [0:15];
    int unsigned ecc_header_count;

    dphy_hs_byte_probe #(
        .LANES(2),
        .SOT_WINDOW_BYTES(8),
        .SWEEP_HOLD_BYTES(4),
        .SWEEP_ENABLE(1'b0),
        .FIXED_BITSLIP_PHASE(0),
        .FIXED_BITSLIP_PHASE_LANE1(0),
        .LANE1_BITSLIP_SWEEP_ENABLE(1'b0),
        .FIXED_TRANSFORM(0),
        .TRACE_TRIGGER_MODE(3),
        .EXPECTED_LONG_DT(8'h1e),
        .EXPECTED_LONG_WC(16'd1280),
        .MIN_SYNC_HEADER_SCORE(13),
        .SYNC_HEADER_SWEEP_BIT_OFFSETS(1'b0),
        .SYNC_HEADER_USE_ALIGNED_STREAM(1'b0),
        .STREAM_PAIRING(0)
    ) dut (
        .rst_n(rst_n),
        .idelay_ref_clk(idelay_ref_clk),
        .idelay_ref_reset(!rst_n),
        .runtime_idelay_tap(5'd0),
        .runtime_bitslip_phase(3'd0),
        .runtime_bitslip_phase_lane1(3'd0),
        .runtime_expected_long_dt(8'h00),
        .sup_enable(1'b0),
        .sup_bufr_clr(1'b0),
        .sup_serdes_rst(1'b0),
        .sup_hs_settled(1'b0),
        .serdes_byte_sample_out(),
        .dphy_hs_clock_clk_p(hs_clk_p),
        .dphy_hs_clock_clk_n(hs_clk_n),
        .dphy_data_hs_p(data_hs_p),
        .dphy_data_hs_n(data_hs_n),
        .dphy_data_lp_p(data_lp_p),
        .dphy_data_lp_n(data_lp_n),
        .byte_clk(byte_clk),
        .idelayctrl_rdy(),
        .hs_clk_seen(),
        .lane_sot_seen(),
        .lane_last_byte(),
        .lane_raw_changed_seen(),
        .lane_raw_non_ff_seen(),
        .lane_raw_non_00_seen(),
        .lane_raw_change_count(),
        .stream_byte_data(stream_byte_data),
        .stream_byte_keep(stream_byte_keep),
        .stream_byte_valid(stream_byte_valid),
        .stream_byte_sop(stream_byte_sop),
        .stream_byte_eop(stream_byte_eop),
        .header_valid(),
        .header_di(),
        .header_wc(),
        .header_ecc(),
        .sync_header_valid(sync_header_valid),
        .sync_header_di(sync_header_di),
        .sync_header_wc(sync_header_wc),
        .sync_header_ecc(sync_header_ecc),
        .sync_header_rotation_lane0(),
        .sync_header_rotation_lane1(),
        .sync_header_bit_offset_lane0(sync_header_bit_offset_lane0),
        .sync_header_bit_offset_lane1(sync_header_bit_offset_lane1),
        .sync_header_score(sync_header_score),
        .sync_header_start_slot(),
        .sync_header_pairing(sync_header_pairing),
        .sync_header_syndrome(),
        .sync_header_ecc_no_error(sync_header_ecc_no_error),
        .sync_header_ecc_corrected(),
        .sync_header_ecc_uncorrectable(),
        .header_slot_valid(),
        .header_slot_di(),
        .header_slot_wc(),
        .header_slot_ecc(),
        .header_slot_bitslip_phase(),
        .header_slot_bitslip_phase_lane1(),
        .header_slot_transform(),
        .header_slot_rotation(),
        .header_slot_corr_di(),
        .header_slot_corr_wc(),
        .header_slot_syndrome(),
        .header_slot_ecc_no_error(),
        .header_slot_ecc_corrected(),
        .header_slot_ecc_uncorrectable(),
        .trace_slot_valid(trace_slot_valid),
        .trace_slot_lane0_raw(),
        .trace_slot_lane1_raw(),
        .trace_slot_lane0_candidate(trace_slot_lane0_candidate),
        .trace_slot_lane1_candidate(trace_slot_lane1_candidate),
        .trace_slot_lane0_aligned(),
        .trace_slot_lane1_aligned(),
        .trace_slot_lane0_rotation(),
        .trace_slot_lane1_rotation(),
        .trace_slot_bitslip_phase_lane0(),
        .trace_slot_bitslip_phase_lane1(),
        .trace_slot_sot_hit_lane0(),
        .trace_slot_sot_hit_lane1(),
        .live_trace_seq(live_trace_seq),
        .live_trace_slot_valid(live_trace_slot_valid),
        .live_trace_slot_lane0_raw(),
        .live_trace_slot_lane1_raw(),
        .live_trace_slot_lane0_candidate(live_trace_slot_lane0_candidate),
        .live_trace_slot_lane1_candidate(live_trace_slot_lane1_candidate),
        .live_trace_slot_lane0_aligned(live_trace_slot_lane0_aligned),
        .live_trace_slot_lane1_aligned(live_trace_slot_lane1_aligned),
        .live_trace_slot_sot_hit_lane0(live_trace_slot_sot_hit_lane0),
        .live_trace_slot_sot_hit_lane1(live_trace_slot_sot_hit_lane1)
    );

    csi2_packet_parser #(
        .IN_WIDTH(16),
        .WC_MAX(4096),
        .FIFO_DEPTH(32)
    ) u_parser (
        .core_clk(byte_clk),
        .core_aresetn(parser_aresetn),
        .s_byte_data(stream_byte_data),
        .s_byte_keep(stream_byte_keep),
        .s_byte_valid(stream_byte_valid),
        .s_byte_sop(stream_byte_sop),
        .s_byte_eop(stream_byte_eop),
        .ecc_hdr_valid(ecc_hdr_valid),
        .ecc_hdr_raw(ecc_hdr_raw),
        .ecc_hdr_corr_valid(ecc_hdr_corr_valid),
        .ecc_hdr_di(ecc_hdr_di),
        .ecc_hdr_wc(ecc_hdr_wc),
        .ecc_hdr_uncorrectable(ecc_hdr_uncorrectable),
        .m_pkt_hdr_valid(parser_pkt_hdr_valid),
        .m_pkt_hdr_raw(parser_pkt_hdr_raw),
        .m_pkt_di(parser_pkt_di),
        .m_pkt_wc(parser_pkt_wc),
        .m_pkt_is_long(parser_pkt_is_long),
        .m_pkt_is_short(parser_pkt_is_short),
        .m_pkt_ecc_uncorrectable(parser_pkt_ecc_uncorrectable),
        .m_payload_data(parser_payload_data),
        .m_payload_valid(parser_payload_valid),
        .m_payload_first(parser_payload_first),
        .m_payload_last(parser_payload_last),
        .m_footer_data(parser_footer_data),
        .m_footer_valid(parser_footer_valid),
        .m_pkt_done(parser_pkt_done),
        .sts_short_pkt_cnt(parser_short_count),
        .sts_long_pkt_cnt(parser_long_count),
        .sts_pkt_trunc_cnt(parser_trunc_count)
    );

    csi2_header_ecc u_header_ecc (
        .core_clk(byte_clk),
        .core_aresetn(parser_aresetn),
        .hdr_valid(ecc_hdr_valid),
        .hdr_raw(ecc_hdr_raw),
        .hdr_corr_valid(ecc_hdr_corr_valid),
        .hdr_corr(ecc_hdr_corr),
        .hdr_di(ecc_hdr_di),
        .hdr_wc(ecc_hdr_wc),
        .hdr_ecc_corrected(ecc_hdr_corrected),
        .hdr_ecc_uncorrectable(ecc_hdr_uncorrectable),
        .hdr_ecc_no_error(ecc_hdr_no_error),
        .sts_ecc_corr_cnt(ecc_corr_count),
        .sts_ecc_uncorr_cnt(ecc_uncorr_count)
    );

    initial begin
        idelay_ref_clk = 1'b0;
        forever #2.5 idelay_ref_clk = ~idelay_ref_clk;
    end

    initial begin
        hs_clk_p = 1'b0;
        forever #5 hs_clk_p = ~hs_clk_p;
    end

    assign hs_clk_n = ~hs_clk_p;
    assign data_hs_n = ~data_hs_p;

    always_ff @(posedge byte_clk) begin
        if (!parser_aresetn) begin
            captured_stream_count <= 0;
            parser_last_hdr_seen <= 1'b0;
            parser_last_pkt_di <= 8'h00;
            parser_last_pkt_wc <= 16'h0000;
            parser_last_pkt_ecc_uncorrectable <= 1'b0;
            parser_last_ecc_seen <= 1'b0;
            parser_last_ecc_no_error <= 1'b0;
            parser_last_ecc_corrected <= 1'b0;
            parser_header_count <= 0;
            parser_payload_byte_count <= 0;
            stream_packet_count <= 0;
            stream_pending_index <= 0;
            stream_second_pending <= 1'b0;
            ecc_header_count <= 0;
            for (int idx = 0; idx < 8; idx++) begin
                captured_stream_data[idx] <= 16'h0000;
                captured_stream_sop[idx] <= 1'b0;
            end
            for (int idx = 0; idx < 16; idx++) begin
                stream_sop_word0_log[idx] <= 16'h0000;
                stream_sop_word1_log[idx] <= 16'h0000;
                stream_sop_seen_log[idx] <= 1'b0;
                parser_di_log[idx] <= 8'h00;
                parser_wc_log[idx] <= 16'h0000;
                parser_ecc_uncorrectable_log[idx] <= 1'b0;
                ecc_no_error_log[idx] <= 1'b0;
                ecc_corrected_log[idx] <= 1'b0;
                ecc_uncorrectable_log[idx] <= 1'b0;
            end
        end else begin
            if (stream_byte_valid && (captured_stream_count < 8)) begin
                captured_stream_data[captured_stream_count] <= stream_byte_data;
                captured_stream_sop[captured_stream_count] <= stream_byte_sop;
                captured_stream_count <= captured_stream_count + 1;
            end
            if (stream_byte_valid && stream_byte_sop && (stream_packet_count < 16)) begin
                stream_sop_word0_log[stream_packet_count] <= stream_byte_data;
                stream_sop_seen_log[stream_packet_count] <= 1'b1;
                stream_pending_index <= stream_packet_count;
                stream_second_pending <= 1'b1;
            end else if (stream_byte_valid && stream_second_pending) begin
                stream_sop_word1_log[stream_pending_index] <= stream_byte_data;
                stream_packet_count <= stream_pending_index + 1;
                stream_second_pending <= 1'b0;
            end
            if (parser_pkt_hdr_valid) begin
                parser_last_hdr_seen <= 1'b1;
                parser_last_pkt_di <= parser_pkt_di;
                parser_last_pkt_wc <= parser_pkt_wc;
                parser_last_pkt_ecc_uncorrectable <= parser_pkt_ecc_uncorrectable;
                if (parser_header_count < 16) begin
                    parser_di_log[parser_header_count] <= parser_pkt_di;
                    parser_wc_log[parser_header_count] <= parser_pkt_wc;
                    parser_ecc_uncorrectable_log[parser_header_count] <= parser_pkt_ecc_uncorrectable;
                end
                parser_header_count <= parser_header_count + 1;
            end
            if (ecc_hdr_corr_valid) begin
                parser_last_ecc_seen <= 1'b1;
                parser_last_ecc_no_error <= ecc_hdr_no_error;
                parser_last_ecc_corrected <= ecc_hdr_corrected;
                if (ecc_header_count < 16) begin
                    ecc_no_error_log[ecc_header_count] <= ecc_hdr_no_error;
                    ecc_corrected_log[ecc_header_count] <= ecc_hdr_corrected;
                    ecc_uncorrectable_log[ecc_header_count] <= ecc_hdr_uncorrectable;
                end
                ecc_header_count <= ecc_header_count + 1;
            end
            if (parser_payload_valid) begin
                parser_payload_byte_count <= parser_payload_byte_count + 1;
            end
        end
    end

    function automatic logic [5:0] ref_ecc6(input logic [23:0] data);
        ref_ecc6[0] = data[0]^data[1]^data[2]^data[4]^data[5]^data[7]^data[10]^data[11]^data[13]^data[16]^data[20]^data[21]^data[22]^data[23];
        ref_ecc6[1] = data[0]^data[1]^data[3]^data[4]^data[6]^data[8]^data[10]^data[12]^data[14]^data[17]^data[20]^data[21]^data[22]^data[23];
        ref_ecc6[2] = data[0]^data[2]^data[3]^data[5]^data[6]^data[9]^data[11]^data[12]^data[15]^data[18]^data[20]^data[21]^data[22];
        ref_ecc6[3] = data[1]^data[2]^data[3]^data[7]^data[8]^data[9]^data[13]^data[14]^data[15]^data[19]^data[20]^data[21]^data[23];
        ref_ecc6[4] = data[4]^data[5]^data[6]^data[7]^data[8]^data[9]^data[16]^data[17]^data[18]^data[19]^data[20]^data[22]^data[23];
        ref_ecc6[5] = data[10]^data[11]^data[12]^data[13]^data[14]^data[15]^data[16]^data[17]^data[18]^data[19]^data[21]^data[22]^data[23];
    endfunction

    function automatic logic [7:0] make_ecc(input logic [7:0] di, input logic [15:0] wc);
        make_ecc = {2'b00, ref_ecc6({wc, di})};
    endfunction

    function automatic logic [15:0] expected_stream_sop_word0(input logic [2:0] pairing);
        unique case (pairing)
            3'd0: expected_stream_sop_word0 = {trace_slot_lane1_candidate[1], trace_slot_lane0_candidate[1]};
            3'd1: expected_stream_sop_word0 = {trace_slot_lane0_candidate[1], trace_slot_lane1_candidate[1]};
            3'd2: expected_stream_sop_word0 = {trace_slot_lane1_candidate[2], trace_slot_lane0_candidate[1]};
            3'd3: expected_stream_sop_word0 = {trace_slot_lane1_candidate[1], trace_slot_lane0_candidate[2]};
            3'd4: expected_stream_sop_word0 = {trace_slot_lane0_candidate[2], trace_slot_lane1_candidate[1]};
            default: expected_stream_sop_word0 = {trace_slot_lane0_candidate[1], trace_slot_lane1_candidate[2]};
        endcase
    endfunction

    function automatic logic [15:0] expected_stream_sop_word1(input logic [2:0] pairing);
        unique case (pairing)
            3'd0: expected_stream_sop_word1 = {trace_slot_lane1_candidate[2], trace_slot_lane0_candidate[2]};
            3'd1: expected_stream_sop_word1 = {trace_slot_lane0_candidate[2], trace_slot_lane1_candidate[2]};
            3'd2: expected_stream_sop_word1 = {trace_slot_lane1_candidate[3], trace_slot_lane0_candidate[2]};
            3'd3: expected_stream_sop_word1 = {trace_slot_lane1_candidate[2], trace_slot_lane0_candidate[3]};
            3'd4: expected_stream_sop_word1 = {trace_slot_lane0_candidate[3], trace_slot_lane1_candidate[2]};
            default: expected_stream_sop_word1 = {trace_slot_lane0_candidate[2], trace_slot_lane1_candidate[3]};
        endcase
    endfunction

    function automatic logic [15:0] live_trace_pair0_word0();
        live_trace_pair0_word0 = {live_trace_slot_lane1_aligned[1], live_trace_slot_lane0_aligned[1]};
    endfunction

    function automatic logic [15:0] live_trace_pair0_word1();
        live_trace_pair0_word1 = {live_trace_slot_lane1_aligned[2], live_trace_slot_lane0_aligned[2]};
    endfunction

    task automatic check_condition(input bit condition, input string message);
        if (!condition) begin
            $fatal(1, "CHECK FAILED: %s", message);
        end
    endtask

    task automatic check_live_trace_pair0(
        input logic [15:0] expected_word0,
        input logic [15:0] expected_word1,
        input string name
    );
        check_condition(&live_trace_slot_valid[3:0], $sformatf("%s: live trace slots 0..3 captured", name));
        check_condition(live_trace_slot_sot_hit_lane0[0], $sformatf("%s: live trace lane0 SoT at slot0", name));
        check_condition(live_trace_slot_sot_hit_lane1[0], $sformatf("%s: live trace lane1 SoT at slot0", name));
        check_condition(live_trace_pair0_word0() == expected_word0, $sformatf("%s: live trace pair0 word0", name));
        check_condition(live_trace_pair0_word1() == expected_word1, $sformatf("%s: live trace pair0 word1", name));
    endtask

    task automatic check_scanner_stream_contract(
        input int unsigned pkt_idx,
        input logic [2:0] expected_pairing,
        input string name
    );
        automatic logic [15:0] expected_word0;
        automatic logic [15:0] expected_word1;
        expected_word0 = expected_stream_sop_word0(expected_pairing);
        expected_word1 = expected_stream_sop_word1(expected_pairing);

        check_condition(sync_header_valid, $sformatf("%s: scanner valid", name));
        check_condition(&trace_slot_valid[3:0], $sformatf("%s: trace slots 0..3 captured", name));
        check_condition(sync_header_pairing == expected_pairing, $sformatf("%s: scanner pairing", name));
        check_condition(sync_header_bit_offset_lane0 == 3'd0, $sformatf("%s: scanner lane0 bit offset zero", name));
        check_condition(sync_header_bit_offset_lane1 == 3'd0, $sformatf("%s: scanner lane1 bit offset zero", name));
        check_condition(sync_header_di == expected_word0[7:0], $sformatf("%s: scanner DI matches selected trace bytes", name));
        check_condition(sync_header_wc == {expected_word1[7:0], expected_word0[15:8]}, $sformatf("%s: scanner WC matches selected trace bytes", name));
        check_condition(sync_header_ecc == expected_word1[15:8], $sformatf("%s: scanner ECC matches selected trace bytes", name));
        check_condition(sync_header_ecc_no_error, $sformatf("%s: scanner selected header ECC clean", name));
        check_condition(stream_sop_seen_log[pkt_idx], $sformatf("%s: stream SOP captured", name));
        check_condition(stream_sop_word0_log[pkt_idx] == expected_word0, $sformatf("%s: stream SOP word0 matches scanner-selected trace bytes", name));
        check_condition(stream_sop_word1_log[pkt_idx] == expected_word1, $sformatf("%s: stream SOP word1 matches scanner-selected trace bytes", name));
    endtask

    task automatic reset_dut();
        rst_n = 1'b0;
        parser_aresetn = 1'b0;
        data_hs_p = 2'b00;
        data_lp_p = 2'b11;
        data_lp_n = 2'b11;
        repeat (8) @(posedge hs_clk_p);
        rst_n = 1'b1;
        repeat (4) @(posedge byte_clk);
        parser_aresetn = 1'b1;
        repeat (8) @(posedge byte_clk);
    endtask

    task automatic drive_lp_state(input logic [1:0] lane_lp_p, input logic [1:0] lane_lp_n, input int unsigned cycles);
        @(negedge byte_clk);
        data_lp_p = lane_lp_p;
        data_lp_n = lane_lp_n;
        repeat (cycles) @(posedge byte_clk);
        #1;
    endtask

    task automatic drive_serdes_sample(input logic [7:0] lane0_byte, input logic [7:0] lane1_byte);
        @(negedge byte_clk);
        dut.serdes_byte_sample[0] = lane0_byte;
        dut.serdes_byte_sample[1] = lane1_byte;
        @(posedge byte_clk);
        #1;
    endtask

    task automatic wait_for_parser_header_count(input string name, input int unsigned target_count);
        for (int cycle = 0; cycle < 80; cycle++) begin
            @(posedge byte_clk);
            #1;
            if (parser_header_count >= target_count) begin
                return;
            end
        end
        $display("%s: captured_stream_count=%0d data0=%04h sop0=%0b data1=%04h sop1=%0b ecc_hdr_valid=%0b ecc_hdr_raw=%08h state=%0d fifo_count=%0d",
            name,
            captured_stream_count,
            captured_stream_data[0],
            captured_stream_sop[0],
            captured_stream_data[1],
            captured_stream_sop[1],
            ecc_hdr_valid,
            ecc_hdr_raw,
            u_parser.state,
            u_parser.fifo_count);
        $fatal(1, "%s: timed out waiting for parser header count %0d", name, target_count);
    endtask

    task automatic wait_for_parser_header(input string name);
        wait_for_parser_header_count(name, 1);
    endtask

    task automatic wait_for_stream_packet_count(input string name, input int unsigned target_count);
        for (int cycle = 0; cycle < 80; cycle++) begin
            @(posedge byte_clk);
            #1;
            if (stream_packet_count >= target_count) begin
                return;
            end
        end
        $fatal(1, "%s: timed out waiting for stream packet count %0d", name, target_count);
    endtask

    task automatic wait_for_parser_payload_count(input string name, input int unsigned target_count);
        for (int cycle = 0; cycle < 120; cycle++) begin
            @(posedge byte_clk);
            #1;
            if (parser_payload_byte_count >= target_count) begin
                return;
            end
        end
        $fatal(1, "%s: timed out waiting for parser payload byte count %0d", name, target_count);
    endtask

    task automatic wait_for_sync_header(input string name);
        for (int cycle = 0; cycle < 120; cycle++) begin
            @(posedge byte_clk);
            #1;
            if (sync_header_valid) begin
                return;
            end
        end
        $fatal(1, "%s: timed out waiting for sync header", name);
    endtask

    task automatic drive_aligned_pair0_header(input logic [7:0] ecc1280);
        drive_serdes_sample(8'hb8, 8'hb8);
        drive_serdes_sample(8'h1e, 8'h00);
        drive_serdes_sample(8'h05, ecc1280);
        drive_serdes_sample(8'h11, 8'h22);
        drive_serdes_sample(8'h33, 8'h44);
        drive_serdes_sample(8'h55, 8'h66);
        drive_serdes_sample(8'h77, 8'h88);
        drive_serdes_sample(8'h99, 8'haa);
    endtask

    task automatic drive_aligned_pair0_short_packet(input logic [7:0] di, input logic [15:0] short_data, input logic [7:0] ecc);
        drive_serdes_sample(8'hb8, 8'hb8);
        drive_serdes_sample(di, short_data[7:0]);
        drive_serdes_sample(short_data[15:8], ecc);
        drive_serdes_sample(8'h11, 8'h22);
        drive_serdes_sample(8'h33, 8'h44);
        drive_serdes_sample(8'h55, 8'h66);
        drive_serdes_sample(8'h77, 8'h88);
        drive_serdes_sample(8'h99, 8'haa);
    endtask

    task automatic drive_lane1_delayed_header(input logic [7:0] ecc1280);
        drive_serdes_sample(8'hb8, 8'hb8);
        drive_serdes_sample(8'h1e, 8'h02);
        drive_serdes_sample(8'h05, 8'h00);
        drive_serdes_sample(8'h11, ecc1280);
        drive_serdes_sample(8'h22, 8'h33);
        drive_serdes_sample(8'h44, 8'h55);
        drive_serdes_sample(8'h66, 8'h77);
        drive_serdes_sample(8'h88, 8'h99);
    endtask

    task automatic drive_pair0_corrupt_header(input logic [7:0] wc_low_bad, input logic [7:0] ecc_bad);
        drive_serdes_sample(8'hb8, 8'hb8);
        drive_serdes_sample(8'h1e, wc_low_bad);
        drive_serdes_sample(8'h05, ecc_bad);
        drive_serdes_sample(8'h11, 8'h22);
        drive_serdes_sample(8'h33, 8'h44);
        drive_serdes_sample(8'h55, 8'h66);
        drive_serdes_sample(8'h77, 8'h88);
        drive_serdes_sample(8'h99, 8'haa);
    endtask

    task automatic drive_next_packet_gap();
        drive_lp_state(2'b11, 2'b11, 4);
        drive_lp_state(2'b00, 2'b00, 4);
    endtask

    task automatic run_aligned_pair0_case(input logic [7:0] ecc1280);
        reset_dut();
        drive_lp_state(2'b00, 2'b00, 4);
        drive_aligned_pair0_header(ecc1280);

        wait_for_parser_header("aligned_pair0");
        wait_for_stream_packet_count("aligned_pair0", 1);
        wait_for_sync_header("aligned_pair0");
        check_live_trace_pair0(16'h001e, {ecc1280, 8'h05}, "aligned_pair0");
        check_scanner_stream_contract(0, 3'd0, "aligned_pair0");

        check_condition(captured_stream_count >= 2, "aligned_pair0: captured at least two parser stream beats");
        check_condition(captured_stream_sop[0], "aligned_pair0: SOP is on first post-SoT stream beat");
        check_condition(captured_stream_data[0] == 16'h001e, "aligned_pair0: first stream beat is DI/WC-low");
        check_condition(captured_stream_data[1] == {ecc1280, 8'h05}, "aligned_pair0: second stream beat is WC-high/ECC");

        check_condition(sync_header_valid, "aligned_pair0: scanner valid");
        check_condition(sync_header_score == 4'd15, "aligned_pair0: scanner score 15");
        check_condition(sync_header_pairing == 3'd0, "aligned_pair0: scanner pairing 0");
        check_condition(sync_header_di == 8'h1e, "aligned_pair0: scanner DI");
        check_condition(sync_header_wc == 16'd1280, "aligned_pair0: scanner WC");
        check_condition(sync_header_ecc_no_error, "aligned_pair0: scanner ECC clean");

        check_condition(parser_last_hdr_seen, "aligned_pair0: parser header valid");
        check_condition(parser_last_pkt_di == 8'h1e, "aligned_pair0: parser DI");
        check_condition(parser_last_pkt_wc == 16'd1280, "aligned_pair0: parser WC");
        check_condition(parser_last_ecc_seen, "aligned_pair0: parser ECC seen");
        check_condition(parser_last_ecc_no_error, "aligned_pair0: parser ECC no-error");
        check_condition(!parser_last_pkt_ecc_uncorrectable, "aligned_pair0: parser ECC clean");
        wait_for_parser_payload_count("aligned_pair0", 2);
        check_condition(parser_payload_byte_count >= 2, "aligned_pair0: parser receives payload after scanner-qualified release");
    endtask

    task automatic run_frame_short_release_case(input logic [7:0] ecc_fs, input logic [7:0] ecc1280);
        reset_dut();
        drive_lp_state(2'b00, 2'b00, 4);
        drive_aligned_pair0_short_packet(8'h00, 16'h0001, ecc_fs);

        wait_for_parser_header_count("frame_short_fs", 1);
        wait_for_stream_packet_count("frame_short_fs", 1);
        wait_for_sync_header("frame_short_fs");
        check_scanner_stream_contract(0, 3'd0, "frame_short_fs");

        check_condition(sync_header_score == 4'd13, "frame_short_fs: clean short packet reaches release threshold");
        check_condition(sync_header_di == 8'h00, "frame_short_fs: scanner DI is FS");
        check_condition(sync_header_wc == 16'h0001, "frame_short_fs: scanner short data");
        check_condition(parser_di_log[0] == 8'h00, "frame_short_fs: parser DI is FS");
        check_condition(parser_wc_log[0] == 16'h0001, "frame_short_fs: parser short data");
        check_condition(parser_short_count == 16'd1, "frame_short_fs: parser short count increments");
        check_condition(ecc_no_error_log[0], "frame_short_fs: parser ECC clean");

        drive_next_packet_gap();
        drive_aligned_pair0_header(ecc1280);

        wait_for_parser_header_count("frame_short_then_long", 2);
        wait_for_stream_packet_count("frame_short_then_long", 2);
        check_clean_pair0_packet_contract(1, ecc1280, "frame_short_then_long");
        wait_for_parser_payload_count("frame_short_then_long", 2);
        check_condition(parser_payload_byte_count >= 2, "frame_short_then_long: long payload follows accepted FS short packet");
    endtask

    task automatic run_lane1_delayed_auto_pair_case(input logic [7:0] ecc1280);
        reset_dut();
        drive_lp_state(2'b00, 2'b00, 4);
        drive_lane1_delayed_header(ecc1280);

        wait_for_parser_header_count("lane1_delayed_first", 1);
        wait_for_stream_packet_count("lane1_delayed_first", 1);
        wait_for_sync_header("lane1_delayed_first");
        check_live_trace_pair0(16'h021e, 16'h0005, "lane1_delayed_first");
        check_scanner_stream_contract(0, 3'd2, "lane1_delayed_first");

        check_condition(captured_stream_count >= 2, "lane1_delayed_first: captured at least two parser stream beats");
        check_condition(captured_stream_sop[0], "lane1_delayed_first: SOP is on the scanner-qualified stream beat");
        check_condition(captured_stream_data[0] == 16'h001e, "lane1_delayed_first: stream emits repaired DI/WC-low");
        check_condition(captured_stream_data[1] == {ecc1280, 8'h05}, "lane1_delayed_first: stream emits repaired WC-high/ECC");

        check_condition(sync_header_valid, "lane1_delayed_first: scanner valid");
        check_condition(sync_header_score == 4'd15, "lane1_delayed_first: scanner score 15");
        check_condition(sync_header_pairing == 3'd2, "lane1_delayed_first: scanner chooses pairing 2");
        check_condition(sync_header_di == 8'h1e, "lane1_delayed_first: scanner DI");
        check_condition(sync_header_wc == 16'd1280, "lane1_delayed_first: scanner WC");
        check_condition(sync_header_ecc_no_error, "lane1_delayed_first: scanner ECC clean");
        check_condition(dut.stream_pairing_next == 3'd2, "lane1_delayed_first: scanner pairing is learned for next packet");
        check_condition(dut.stream_pairing_active == 3'd2, "lane1_delayed_first: scanner pairing is applied to current packet");

        check_condition(parser_last_hdr_seen, "lane1_delayed_first: parser header valid");
        check_condition(parser_last_pkt_di == 8'h1e, "lane1_delayed_first: parser DI");
        check_condition(parser_last_pkt_wc == 16'd1280, "lane1_delayed_first: parser WC repaired to 1280");
        check_condition(parser_last_ecc_seen, "lane1_delayed_first: parser ECC seen");
        check_condition(parser_last_ecc_no_error, "lane1_delayed_first: parser header ECC is clean");
        check_condition(!parser_last_pkt_ecc_uncorrectable, "lane1_delayed_first: parser ECC not uncorrectable");

        drive_lp_state(2'b11, 2'b11, 4);
        drive_lp_state(2'b00, 2'b00, 4);
        drive_lane1_delayed_header(ecc1280);

        wait_for_parser_header_count("lane1_delayed_second", 2);
        wait_for_stream_packet_count("lane1_delayed_second", 2);
        check_scanner_stream_contract(1, 3'd2, "lane1_delayed_second");

        check_condition(dut.stream_pairing_active == 3'd2, "lane1_delayed_second: scanner pairing applied at current SoT");
        check_condition(parser_last_pkt_di == 8'h1e, "lane1_delayed_second: parser DI");
        check_condition(parser_last_pkt_wc == 16'd1280, "lane1_delayed_second: parser WC remains 1280");
        check_condition(parser_last_ecc_seen, "lane1_delayed_second: parser ECC seen");
        check_condition(parser_last_ecc_no_error, "lane1_delayed_second: parser ECC clean");
        check_condition(!parser_last_pkt_ecc_uncorrectable, "lane1_delayed_second: parser ECC not uncorrectable");
    endtask

    task automatic check_clean_pair0_packet_contract(
        input int unsigned pkt_idx,
        input logic [7:0] ecc1280,
        input string name
    );
        check_condition(stream_sop_seen_log[pkt_idx], $sformatf("%s: stream SOP captured", name));
        check_condition(stream_sop_word0_log[pkt_idx] == 16'h001e, $sformatf("%s: stream first SOP word is DI/WC-low", name));
        check_condition(stream_sop_word1_log[pkt_idx] == {ecc1280, 8'h05}, $sformatf("%s: stream second SOP word is WC-high/ECC", name));
        check_condition(parser_di_log[pkt_idx] == 8'h1e, $sformatf("%s: parser DI", name));
        check_condition(parser_wc_log[pkt_idx] == 16'd1280, $sformatf("%s: parser WC", name));
        check_condition(ecc_no_error_log[pkt_idx], $sformatf("%s: parser ECC clean", name));
        check_condition(!parser_ecc_uncorrectable_log[pkt_idx], $sformatf("%s: parser ECC not uncorrectable", name));
    endtask

    task automatic run_sustained_valid_pair0_contract_case(input logic [7:0] ecc1280);
        reset_dut();
        drive_lp_state(2'b00, 2'b00, 4);
        for (int pkt_idx = 0; pkt_idx < 10; pkt_idx++) begin
            if (pkt_idx != 0) begin
                drive_next_packet_gap();
            end
            drive_aligned_pair0_header(ecc1280);
            wait_for_parser_header_count($sformatf("sustained_valid_pair0_%0d", pkt_idx), pkt_idx + 1);
            wait_for_stream_packet_count($sformatf("sustained_valid_pair0_%0d", pkt_idx), pkt_idx + 1);
                check_live_trace_pair0(16'h001e, {ecc1280, 8'h05}, $sformatf("sustained_valid_pair0_%0d", pkt_idx));
            check_clean_pair0_packet_contract(pkt_idx, ecc1280, $sformatf("sustained_valid_pair0_%0d", pkt_idx));
        end

        check_condition(sync_header_valid, "sustained_valid_pair0: scanner valid");
        check_condition(sync_header_pairing == 3'd0, "sustained_valid_pair0: scanner pairing 0");
        check_condition(sync_header_score == 4'd15, "sustained_valid_pair0: scanner score 15");
        check_condition(sync_header_wc == 16'd1280, "sustained_valid_pair0: scanner WC 1280");
        check_condition(dut.stream_pairing_active == 3'd0, "sustained_valid_pair0: active stream pairing stays pair0");
        check_condition(dut.stream_pairing_next == 3'd0, "sustained_valid_pair0: learned stream pairing stays pair0");
    endtask

    task automatic run_diagnostic_bad_live_sop_signature_case(input logic [7:0] ecc1280);
        reset_dut();
        drive_lp_state(2'b00, 2'b00, 4);
        drive_aligned_pair0_header(ecc1280);

        wait_for_parser_header_count("latched_clean_first", 1);
        wait_for_stream_packet_count("latched_clean_first", 1);
        wait_for_sync_header("latched_clean_first");

        check_condition(sync_header_valid, "latched_clean_first: scanner valid");
        check_condition(sync_header_pairing == 3'd0, "latched_clean_first: scanner pairing 0");
        check_condition(sync_header_score == 4'd15, "latched_clean_first: scanner score 15");
        check_condition(sync_header_di == 8'h1e, "latched_clean_first: scanner DI");
        check_condition(sync_header_wc == 16'd1280, "latched_clean_first: scanner WC 1280");
        check_condition(parser_wc_log[0] == 16'd1280, "latched_clean_first: parser WC 1280");
        check_condition(ecc_no_error_log[0], "latched_clean_first: parser ECC clean");

        drive_next_packet_gap();
        drive_pair0_corrupt_header(8'h02, 8'h1f);

        for (int cycle = 0; cycle < 100; cycle++) begin
            @(posedge byte_clk);
            #1;
        end
        check_live_trace_pair0(16'h021e, 16'h1f05, "diagnostic_bad_live_sop");

        check_condition(!sync_header_valid, "diagnostic_bad_live_sop: scanner rejects corrupt header");
        check_condition(sync_header_score < 4'd13, "diagnostic_bad_live_sop: scanner score stays below release threshold");
        check_condition(stream_packet_count == 1, "diagnostic_bad_live_sop: corrupt live pair0 is not released as a stream SOP");
        check_condition(parser_header_count == 1, "diagnostic_bad_live_sop: parser does not see a second corrupt header");
        check_condition(ecc_header_count == 1, "diagnostic_bad_live_sop: parser ECC does not process the rejected header");
    endtask

    initial begin
        automatic logic [7:0] ecc1280;
        ecc1280 = make_ecc(8'h1e, 16'd1280);
        rst_n = 1'b0;
        parser_aresetn = 1'b0;
        data_hs_p = 2'b00;
        data_lp_p = 2'b11;
        data_lp_n = 2'b11;

        run_aligned_pair0_case(ecc1280);
        run_frame_short_release_case(make_ecc(8'h00, 16'h0001), ecc1280);
        run_lane1_delayed_auto_pair_case(ecc1280);
        run_sustained_valid_pair0_contract_case(ecc1280);
        run_diagnostic_bad_live_sop_signature_case(ecc1280);

        $display("TEST PASSED: tb_dphy_hs_stream_boundary");
        $finish;
    end

    initial begin
        #1ms;
        $fatal(1, "Simulation timeout");
    end
endmodule

`default_nettype wire
