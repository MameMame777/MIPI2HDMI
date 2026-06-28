`timescale 1ns/1ps
// End-to-end TPG → pipelined-mux → parser → ECC → CRC integration test.
//
// Verifies:
//   1. csi2_tpg generates valid CSI-2 byte streams (correct ECC + CRC).
//   2. csi2_packet_parser correctly extracts headers and payload.
//   3. csi2_header_ecc reports zero uncorrectable errors for TPG output.
//   4. csi2_payload_crc reports crc_ok for every long packet.
//   5. The pipelined AND-OR mux (probe_top.sv copy) passes all five signals
//      correctly when use_tpg_rt=1.
//
// Simulation size: H=4 pixels, V=3 lines, GAP=10 clocks → fast.
// Expected: 3 frames × 3 lines/frame = 9 crc_ok events in < 10000 cycles.

module tb_tpg_parser_full;

    // -----------------------------------------------------------------------
    // Clock and reset
    // -----------------------------------------------------------------------
    logic clk, resetn;
    initial clk = 0;
    always #4 clk = ~clk;   // 125 MHz

    task automatic clk_step(input int n = 1);
        repeat (n) @(posedge clk); #1;
    endtask

    // -----------------------------------------------------------------------
    // Parameters
    // -----------------------------------------------------------------------
    localparam int H_PIXELS = 4;
    localparam int V_LINES  = 3;
    localparam int GAP      = 10;
    localparam int FRAMES   = 3;

    // -----------------------------------------------------------------------
    // TPG outputs
    // -----------------------------------------------------------------------
    logic [15:0] tpg_byte_data;
    logic [1:0]  tpg_byte_keep;
    logic        tpg_byte_valid;
    logic        tpg_byte_sop;
    logic        tpg_byte_eop;

    csi2_tpg #(
        .H_PIXELS        (H_PIXELS),
        .V_LINES         (V_LINES),
        .DT              (6'h22),
        .VC              (2'h0),
        .LSLE_EN         (1'b0),
        .FRAME_GAP_CLOCKS(GAP),
        .OUTPUT_INTERVAL (2)     // match CDC CORE_OUTPUT_INTERVAL=2; prevents parser FIFO overflow
    ) u_tpg (
        .clk         (clk),
        .rst_n       (resetn),
        .m_byte_data (tpg_byte_data),
        .m_byte_keep (tpg_byte_keep),
        .m_byte_valid(tpg_byte_valid),
        .m_byte_sop  (tpg_byte_sop),
        .m_byte_eop  (tpg_byte_eop)
    );

    // -----------------------------------------------------------------------
    // Dummy camera (always idle)
    // -----------------------------------------------------------------------
    logic [15:0] cdc_byte_data  = 16'hCAFE;  // detectable if it leaks
    logic [1:0]  cdc_byte_keep  = 2'b11;
    logic        cdc_byte_valid = 1'b0;
    logic        cdc_byte_sop   = 1'b0;
    logic        cdc_byte_eop   = 1'b0;

    // -----------------------------------------------------------------------
    // Pipelined AND-OR mux (exact copy from probe_top.sv v17)
    // -----------------------------------------------------------------------
    logic use_tpg_rt;

    logic        tpg_gate_r,  cam_gate_r;
    logic        tpg_valid_r, cdc_valid_r;
    logic        tpg_sop_r,   cdc_sop_r;
    logic        tpg_eop_r,   cdc_eop_r;
    logic [15:0] tpg_data_r,  cdc_data_r;
    logic [1:0]  tpg_keep_r,  cdc_keep_r;

    always_ff @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            tpg_gate_r  <= 1'b0; cam_gate_r  <= 1'b1;
            tpg_valid_r <= 1'b0; cdc_valid_r <= 1'b0;
            tpg_sop_r   <= 1'b0; cdc_sop_r   <= 1'b0;
            tpg_eop_r   <= 1'b0; cdc_eop_r   <= 1'b0;
            tpg_data_r  <= 16'h0; cdc_data_r  <= 16'h0;
            tpg_keep_r  <= 2'h0;  cdc_keep_r  <= 2'h0;
        end else begin
            tpg_gate_r  <= use_tpg_rt;
            cam_gate_r  <= ~use_tpg_rt;
            tpg_valid_r <= tpg_byte_valid; cdc_valid_r <= cdc_byte_valid;
            tpg_sop_r   <= tpg_byte_sop;   cdc_sop_r   <= cdc_byte_sop;
            tpg_eop_r   <= tpg_byte_eop;   cdc_eop_r   <= cdc_byte_eop;
            tpg_data_r  <= tpg_byte_data;  cdc_data_r  <= cdc_byte_data;
            tpg_keep_r  <= tpg_byte_keep;  cdc_keep_r  <= cdc_byte_keep;
        end
    end

    wire        pkt_byte_valid = (tpg_valid_r & tpg_gate_r) | (cdc_valid_r & cam_gate_r);
    wire        pkt_byte_sop   = (tpg_sop_r   & tpg_gate_r) | (cdc_sop_r   & cam_gate_r);
    wire        pkt_byte_eop   = (tpg_eop_r   & tpg_gate_r) | (cdc_eop_r   & cam_gate_r);
    wire [15:0] pkt_byte_data  = ({16{tpg_gate_r}} & tpg_data_r) | ({16{cam_gate_r}} & cdc_data_r);
    wire [1:0]  pkt_byte_keep  = ({2{tpg_gate_r}}  & tpg_keep_r) | ({2{cam_gate_r}}  & cdc_keep_r);

    // -----------------------------------------------------------------------
    // Parser
    // -----------------------------------------------------------------------
    logic        ecc_hdr_valid;
    logic [31:0] ecc_hdr_raw;
    logic        ecc_hdr_corr_valid;
    logic [7:0]  ecc_hdr_di;
    logic [15:0] ecc_hdr_wc;
    logic        ecc_hdr_uncorrectable;

    logic        m_pkt_hdr_valid;
    logic [7:0]  m_pkt_di;
    logic [15:0] m_pkt_wc;
    logic        m_pkt_is_long;
    logic        m_pkt_is_short;
    logic        m_pkt_ecc_uncorrectable;
    logic        m_pkt_done;
    logic [7:0]  m_payload_data;
    logic        m_payload_valid;
    logic        m_payload_first;
    logic        m_payload_last;
    logic [15:0] m_footer_data;
    logic        m_footer_valid;
    logic [15:0] sts_pkt_trunc_cnt;
    logic [15:0] sts_long_pkt_cnt;
    logic [15:0] sts_short_pkt_cnt;

    csi2_packet_parser #(.IN_WIDTH(16), .FIFO_DEPTH(16)) u_parser (
        .core_clk           (clk),
        .core_aresetn       (resetn),
        .s_byte_data        (pkt_byte_data),
        .s_byte_keep        (pkt_byte_keep),
        .s_byte_valid       (pkt_byte_valid),
        .s_byte_sop         (pkt_byte_sop),
        .s_byte_eop         (pkt_byte_eop),
        .ecc_hdr_valid      (ecc_hdr_valid),
        .ecc_hdr_raw        (ecc_hdr_raw),
        .ecc_hdr_corr_valid (ecc_hdr_corr_valid),
        .ecc_hdr_di         (ecc_hdr_di),
        .ecc_hdr_wc         (ecc_hdr_wc),
        .ecc_hdr_uncorrectable(ecc_hdr_uncorrectable),
        .m_pkt_hdr_valid    (m_pkt_hdr_valid),
        .m_pkt_hdr_raw      (),
        .m_pkt_di           (m_pkt_di),
        .m_pkt_wc           (m_pkt_wc),
        .m_pkt_is_long      (m_pkt_is_long),
        .m_pkt_is_short     (m_pkt_is_short),
        .m_pkt_ecc_uncorrectable(m_pkt_ecc_uncorrectable),
        .m_payload_data     (m_payload_data),
        .m_payload_valid    (m_payload_valid),
        .m_payload_first    (m_payload_first),
        .m_payload_last     (m_payload_last),
        .m_footer_data      (m_footer_data),
        .m_footer_valid     (m_footer_valid),
        .m_pkt_done         (m_pkt_done),
        .sts_short_pkt_cnt  (sts_short_pkt_cnt),
        .sts_long_pkt_cnt   (sts_long_pkt_cnt),
        .sts_pkt_trunc_cnt  (sts_pkt_trunc_cnt)
    );

    // -----------------------------------------------------------------------
    // ECC checker
    // -----------------------------------------------------------------------
    logic        hdr_ecc_corrected;
    logic        hdr_ecc_uncorrectable;
    logic        hdr_ecc_no_error;
    logic [15:0] sts_ecc_corr_cnt;
    logic [15:0] sts_ecc_uncorr_cnt;

    csi2_header_ecc u_ecc (
        .core_clk             (clk),
        .core_aresetn         (resetn),
        .hdr_valid            (ecc_hdr_valid),
        .hdr_raw              (ecc_hdr_raw),
        .hdr_corr_valid       (ecc_hdr_corr_valid),
        .hdr_corr             (),
        .hdr_di               (ecc_hdr_di),
        .hdr_wc               (ecc_hdr_wc),
        .hdr_ecc_corrected    (hdr_ecc_corrected),
        .hdr_ecc_uncorrectable(ecc_hdr_uncorrectable),
        .hdr_ecc_no_error     (hdr_ecc_no_error),
        .sts_ecc_corr_cnt     (sts_ecc_corr_cnt),
        .sts_ecc_uncorr_cnt   (sts_ecc_uncorr_cnt)
    );

    // -----------------------------------------------------------------------
    // CRC checker
    // -----------------------------------------------------------------------
    logic        crc_check_valid;
    logic        crc_match;
    logic [15:0] sts_crc_err_cnt;
    logic [15:0] sts_crc_ok_cnt;

    csi2_payload_crc u_crc (
        .core_clk      (clk),
        .core_aresetn  (resetn),
        .payload_data  (m_payload_data),
        .payload_valid (m_payload_valid),
        .payload_first (m_payload_first),
        .payload_last  (m_payload_last),
        .footer_data   (m_footer_data),
        .footer_valid  (m_footer_valid),
        .crc_check_valid(crc_check_valid),
        .crc_match     (crc_match),
        .crc_calc      (),
        .crc_received  (),
        .sts_crc_err_cnt(sts_crc_err_cnt),
        .sts_crc_ok_cnt (sts_crc_ok_cnt)
    );

    // -----------------------------------------------------------------------
    // Stimulus and checker
    // -----------------------------------------------------------------------
    int pass_cnt = 0;
    int fail_cnt = 0;

    task automatic check_val(
        input string label,
        input int    actual,
        input int    expected_min,
        input int    expected_max
    );
        if (actual >= expected_min && actual <= expected_max) begin
            $display("PASS [%s] = %0d (in [%0d,%0d])", label, actual, expected_min, expected_max);
            pass_cnt++;
        end else begin
            $display("FAIL [%s] = %0d  expected [%0d,%0d]", label, actual, expected_min, expected_max);
            fail_cnt++;
        end
    endtask

    // Track every header seen for spot-checking
    always_ff @(posedge clk) begin
        if (m_pkt_hdr_valid) begin
            $display("  t=%0t  HDR di=0x%02h wc=0x%04h is_long=%b ecc_uncorr=%b",
                     $time, m_pkt_di, m_pkt_wc, m_pkt_is_long, m_pkt_ecc_uncorrectable);
        end
        if (crc_check_valid) begin
            $display("  t=%0t  CRC_CHECK match=%b  ok_cnt=%0d  err_cnt=%0d",
                     $time, crc_match, sts_crc_ok_cnt, sts_crc_err_cnt);
        end
    end

    initial begin
        resetn     = 0;
        use_tpg_rt = 0;
        clk_step(4);

        // Release reset with use_tpg_rt=1 (same as hardware: written before sccb_done)
        use_tpg_rt = 1;
        clk_step(1);
        resetn = 1;

        // With OUTPUT_INTERVAL=2 each beat takes 2 clocks.
        // Per frame: idle(2)+FS(4)+3×[lhdr0+lhdr1+4pay+crc+next](16)+FE(4)+gap(20) = 82 clocks.
        // From sim trace: Frame 2 FE short_pkt increment at posedge t=1844000ps.
        // Frame 3 FS short_pkt increment at posedge t=2004000ps (= the 246th posedge in clk_step).
        // Stop at clk_step(245) = posedges 6..250 (last at t=1996000ps): frame 3 FS not yet counted.
        clk_step(245);

        // ----------------------------------------------------------------
        // Checks
        // ----------------------------------------------------------------
        $display("");
        $display("=== RESULTS after %0d cycles ===", 245);

        // ECC: all TPG headers must have zero uncorrectable errors
        check_val("ecc_uncorr_cnt",  int'(sts_ecc_uncorr_cnt), 0, 0);
        check_val("ecc_corr_cnt",    int'(sts_ecc_corr_cnt),   0, 0);

        // CRC: expect FRAMES × V_LINES = 9 ok events, 0 errors
        check_val("crc_ok_cnt",      int'(sts_crc_ok_cnt),  FRAMES*V_LINES, FRAMES*V_LINES);
        check_val("crc_err_cnt",     int'(sts_crc_err_cnt), 0, 0);

        // Parser: expect FRAMES short pkts × 2 (FS+FE) = 6 short, FRAMES×V_LINES=9 long, 0 trunc
        check_val("short_pkt_cnt",   int'(sts_short_pkt_cnt), FRAMES*2, FRAMES*2);
        check_val("long_pkt_cnt",    int'(sts_long_pkt_cnt),  FRAMES*V_LINES, FRAMES*V_LINES);
        check_val("pkt_trunc_cnt",   int'(sts_pkt_trunc_cnt), 0, 0);

        $display("");
        $display("=== RESULT: %0d passed, %0d failed ===", pass_cnt, fail_cnt);
        if (fail_cnt == 0)
            $display("ALL PASS — TPG end-to-end ECC+CRC correct through pipelined mux");
        else
            $display("FAILURES — TPG/mux/parser pipeline has bugs");
        $finish;
    end

endmodule
