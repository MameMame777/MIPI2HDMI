`timescale 1ns / 1ps
//
// tb_dphy_lane1_trace
//
// Reproduce the deployed bitstream's parameters (FIXED_BITSLIP_PHASE=6,
// FIXED_TRANSFORM=1) and drive multiple back-to-back long packet headers
// with LP-HS transitions to simulate real OV5640 streaming. Trace lane 1's
// byte capture pipeline at every stage to find where 0x1E (expected ECC)
// becomes 0x2B (observed on hardware).
//
// If lane 1 slot[2] reads 0x1E in DSim, the RTL is correct and the hardware
// regression must be hardware-level. If it reads 0x2B, we have a reproducible
// logic bug to chase.

module IBUFDS #(
    parameter string DIFF_TERM = "FALSE",
    parameter string IBUF_LOW_PWR = "TRUE",
    parameter string IOSTANDARD = "DEFAULT"
) (
    input  wire I,
    input  wire IB,
    output wire O
);
    assign O = I;
endmodule

module BUFIO (
    input  wire I,
    output wire O
);
    assign O = I;
endmodule

module BUFR #(
    parameter string BUFR_DIVIDE = "BYPASS",
    parameter string SIM_DEVICE = "7SERIES"
) (
    input  wire I,
    input  wire CE,
    input  wire CLR,
    output wire O
);
    assign O = CLR ? 1'b0 : (CE ? I : 1'b0);
endmodule

module IDELAYCTRL (
    input  wire REFCLK,
    input  wire RST,
    output wire RDY
);
    assign RDY = !RST;
endmodule

module IDELAYE2 #(
    parameter string CINVCTRL_SEL = "FALSE",
    parameter string DELAY_SRC = "IDATAIN",
    parameter string HIGH_PERFORMANCE_MODE = "TRUE",
    parameter string IDELAY_TYPE = "FIXED",
    parameter int IDELAY_VALUE = 0,
    parameter string PIPE_SEL = "FALSE",
    parameter real REFCLK_FREQUENCY = 200.0,
    parameter string SIGNAL_PATTERN = "DATA"
) (
    input  wire C,
    input  wire REGRST,
    input  wire LD,
    input  wire CE,
    input  wire INC,
    input  wire LDPIPEEN,
    input  wire CINVCTRL,
    input  wire [4:0] CNTVALUEIN,
    input  wire IDATAIN,
    input  wire DATAIN,
    output wire DATAOUT,
    output wire [4:0] CNTVALUEOUT
);
    assign DATAOUT = IDATAIN;
    assign CNTVALUEOUT = CNTVALUEIN;
endmodule

module ISERDESE2 #(
    parameter string DATA_RATE = "DDR",
    parameter int DATA_WIDTH = 8,
    parameter string DYN_CLKDIV_INV_EN = "FALSE",
    parameter string DYN_CLK_INV_EN = "FALSE",
    parameter string INTERFACE_TYPE = "NETWORKING",
    parameter string IOBDELAY = "IFD",
    parameter int NUM_CE = 1,
    parameter string OFB_USED = "FALSE",
    parameter string SERDES_MODE = "MASTER"
) (
    output wire Q1, output wire Q2, output wire Q3, output wire Q4,
    output wire Q5, output wire Q6, output wire Q7, output wire Q8,
    output wire SHIFTOUT1, output wire SHIFTOUT2,
    input  wire BITSLIP, input  wire CE1, input  wire CE2,
    input  wire CLK, input  wire CLKB, input  wire CLKDIV, input  wire CLKDIVP,
    input  wire D, input  wire DDLY, input  wire DYNCLKDIVSEL, input  wire DYNCLKSEL,
    input  wire OCLK, input  wire OCLKB, input  wire OFB, input  wire RST,
    input  wire SHIFTIN1, input  wire SHIFTIN2,
    output wire O
);
    assign {Q8, Q7, Q6, Q5, Q4, Q3, Q2, Q1} = 8'h00;
    assign SHIFTOUT1 = 1'b0;
    assign SHIFTOUT2 = 1'b0;
    assign O = DDLY;
endmodule

module tb_dphy_lane1_trace;

    // Bit-reverse helper (TB-side ref)
    function automatic logic [7:0] reverse8(input logic [7:0] v);
        for (int i = 0; i < 8; i++) reverse8[i] = v[7-i];
    endfunction

    // Independent ECC reference (Python-equivalent)
    function automatic logic [5:0] ref_ecc6(input logic [23:0] data);
        ref_ecc6[0] = data[0]^data[1]^data[2]^data[4]^data[5]^data[7]^data[10]^data[11]^data[13]^data[16]^data[20]^data[21]^data[22]^data[23];
        ref_ecc6[1] = data[0]^data[1]^data[3]^data[4]^data[6]^data[8]^data[10]^data[12]^data[14]^data[17]^data[20]^data[21]^data[22]^data[23];
        ref_ecc6[2] = data[0]^data[2]^data[3]^data[5]^data[6]^data[9]^data[11]^data[12]^data[15]^data[18]^data[20]^data[21]^data[22];
        ref_ecc6[3] = data[1]^data[2]^data[3]^data[7]^data[8]^data[9]^data[13]^data[14]^data[15]^data[19]^data[20]^data[21]^data[23];
        ref_ecc6[4] = data[4]^data[5]^data[6]^data[7]^data[8]^data[9]^data[16]^data[17]^data[18]^data[19]^data[20]^data[22]^data[23];
        ref_ecc6[5] = data[10]^data[11]^data[12]^data[13]^data[14]^data[15]^data[16]^data[17]^data[18]^data[19]^data[21]^data[22]^data[23];
    endfunction

    // === Signals ===
    logic rst_n;
    logic idelay_ref_clk;
    logic hs_clk_p;
    logic [1:0] data_hs_p;
    logic [1:0] data_lp_p;
    logic [1:0] data_lp_n;
    logic byte_clk;

    // catch-all outputs (not all asserted in this trace TB)
    logic idelayctrl_rdy, hs_clk_seen;
    logic [1:0] lane_sot_seen;
    logic [1:0][7:0] lane_last_byte;
    logic [1:0] lane_raw_changed_seen, lane_raw_non_ff_seen, lane_raw_non_00_seen;
    logic [1:0][7:0] lane_raw_change_count;
    logic [15:0] stream_byte_data;
    logic [1:0] stream_byte_keep;
    logic stream_byte_valid, stream_byte_sop, stream_byte_eop;
    logic [2:0] stream_pairing_active_dbg, stream_pairing_next_dbg;
    logic header_valid;
    logic [7:0] header_di;
    logic [15:0] header_wc;
    logic [7:0] header_ecc;
    logic sync_header_valid;
    logic [7:0] sync_header_di;
    logic [15:0] sync_header_wc;
    logic [7:0] sync_header_ecc;
    logic [2:0] sync_header_rotation_lane0, sync_header_rotation_lane1;
    logic [2:0] sync_header_bit_offset_lane0, sync_header_bit_offset_lane1;
    logic [3:0] sync_header_score;
    logic [2:0] sync_header_start_slot, sync_header_pairing;
    logic [5:0] sync_header_syndrome;
    logic sync_header_ecc_no_error, sync_header_ecc_corrected, sync_header_ecc_uncorrectable;
    logic [7:0] header_slot_valid;
    logic [7:0][7:0] header_slot_di;
    logic [7:0][15:0] header_slot_wc;
    logic [7:0][7:0] header_slot_ecc;
    logic [7:0][2:0] header_slot_bitslip_phase, header_slot_bitslip_phase_lane1;
    logic [7:0][2:0] header_slot_transform, header_slot_rotation;
    logic [7:0][7:0] header_slot_corr_di;
    logic [7:0][15:0] header_slot_corr_wc;
    logic [7:0][5:0] header_slot_syndrome;
    logic [7:0] header_slot_ecc_no_error, header_slot_ecc_corrected, header_slot_ecc_uncorrectable;
    logic [7:0] trace_slot_valid;
    logic [7:0][7:0] trace_slot_lane0_raw, trace_slot_lane1_raw;
    logic [7:0][7:0] trace_slot_lane0_candidate, trace_slot_lane1_candidate;
    logic [7:0][7:0] trace_slot_lane0_aligned, trace_slot_lane1_aligned;
    logic [7:0][2:0] trace_slot_lane0_rotation, trace_slot_lane1_rotation;
    logic [7:0][2:0] trace_slot_bitslip_phase_lane0, trace_slot_bitslip_phase_lane1;
    logic [7:0] trace_slot_sot_hit_lane0, trace_slot_sot_hit_lane1;
    logic [7:0] live_trace_seq, live_trace_slot_valid;
    logic [7:0][7:0] live_trace_slot_lane0_raw, live_trace_slot_lane1_raw;
    logic [7:0][7:0] live_trace_slot_lane0_candidate, live_trace_slot_lane1_candidate;
    logic [7:0][7:0] live_trace_slot_lane0_aligned, live_trace_slot_lane1_aligned;
    logic [7:0] live_trace_slot_sot_hit_lane0, live_trace_slot_sot_hit_lane1;
    logic [7:0][2:0] live_trace_slot_lane0_rotation, live_trace_slot_lane1_rotation;

    // === DUT — DEPLOYED BITSTREAM PARAMETERS ===
    dphy_hs_byte_probe #(
        .LANES(2),
        .SOT_WINDOW_BYTES(64),         // matches deployed
        .SWEEP_HOLD_BYTES(8),          // shortened for TB (deployed=16384)
        .SWEEP_ENABLE(1'b0),
        .FIXED_BITSLIP_PHASE(6),       // matches deployed
        .FIXED_BITSLIP_PHASE_LANE1(6), // matches deployed
        .LANE1_BITSLIP_SWEEP_ENABLE(1'b0),
        .FIXED_TRANSFORM(1),           // matches deployed = bit-reverse
        .TRACE_TRIGGER_MODE(3),        // matches deployed = both lanes
        .IDELAY_TAP(8),
        .IDELAY_REFCLK_MHZ(200.0),
        .EXPECTED_LONG_DT(8'h1e),
        .EXPECTED_LONG_WC(16'd1280),
        .MIN_SYNC_HEADER_SCORE(13),
        .SYNC_HEADER_SWEEP_BIT_OFFSETS(1'b0),
        .SYNC_HEADER_USE_ALIGNED_STREAM(1'b1),
        .STREAM_PAIRING(0)
    ) dut (
        .rst_n(rst_n),
        .idelay_ref_clk(idelay_ref_clk),
        .idelay_ref_reset(!rst_n),
        .runtime_idelay_tap(5'd8),
        .runtime_idelay_tap_lane1(5'd8),
        .runtime_bitslip_phase(3'd6),
        .runtime_bitslip_phase_lane1(3'd6),
        .runtime_expected_long_dt(8'h00),
        .sup_enable(1'b0),
        .sup_bufr_clr(1'b0),
        .sup_serdes_rst(1'b0),
        .sup_hs_settled(1'b0),
        .serdes_byte_sample_out(),
        .dphy_hs_clock_clk_p(hs_clk_p),
        .dphy_hs_clock_clk_n(~hs_clk_p),
        .dphy_data_hs_p(data_hs_p),
        .dphy_data_hs_n(~data_hs_p),
        .dphy_data_lp_p(data_lp_p),
        .dphy_data_lp_n(data_lp_n),
        .byte_clk(byte_clk),
        .idelayctrl_rdy(idelayctrl_rdy),
        .hs_clk_seen(hs_clk_seen),
        .lane_sot_seen(lane_sot_seen),
        .lane_last_byte(lane_last_byte),
        .lane_raw_changed_seen(lane_raw_changed_seen),
        .lane_raw_non_ff_seen(lane_raw_non_ff_seen),
        .lane_raw_non_00_seen(lane_raw_non_00_seen),
        .lane_raw_change_count(lane_raw_change_count),
        .stream_byte_data(stream_byte_data),
        .stream_byte_keep(stream_byte_keep),
        .stream_byte_valid(stream_byte_valid),
        .stream_byte_sop(stream_byte_sop),
        .stream_byte_eop(stream_byte_eop),
        .stream_pairing_active_dbg(stream_pairing_active_dbg),
        .stream_pairing_next_dbg(stream_pairing_next_dbg),
        .header_valid(header_valid),
        .header_di(header_di),
        .header_wc(header_wc),
        .header_ecc(header_ecc),
        .sync_header_valid(sync_header_valid),
        .sync_header_di(sync_header_di),
        .sync_header_wc(sync_header_wc),
        .sync_header_ecc(sync_header_ecc),
        .sync_header_rotation_lane0(sync_header_rotation_lane0),
        .sync_header_rotation_lane1(sync_header_rotation_lane1),
        .sync_header_bit_offset_lane0(sync_header_bit_offset_lane0),
        .sync_header_bit_offset_lane1(sync_header_bit_offset_lane1),
        .sync_header_score(sync_header_score),
        .sync_header_start_slot(sync_header_start_slot),
        .sync_header_pairing(sync_header_pairing),
        .sync_header_syndrome(sync_header_syndrome),
        .sync_header_ecc_no_error(sync_header_ecc_no_error),
        .sync_header_ecc_corrected(sync_header_ecc_corrected),
        .sync_header_ecc_uncorrectable(sync_header_ecc_uncorrectable),
        .header_slot_valid(header_slot_valid),
        .header_slot_di(header_slot_di),
        .header_slot_wc(header_slot_wc),
        .header_slot_ecc(header_slot_ecc),
        .header_slot_bitslip_phase(header_slot_bitslip_phase),
        .header_slot_bitslip_phase_lane1(header_slot_bitslip_phase_lane1),
        .header_slot_transform(header_slot_transform),
        .header_slot_rotation(header_slot_rotation),
        .header_slot_corr_di(header_slot_corr_di),
        .header_slot_corr_wc(header_slot_corr_wc),
        .header_slot_syndrome(header_slot_syndrome),
        .header_slot_ecc_no_error(header_slot_ecc_no_error),
        .header_slot_ecc_corrected(header_slot_ecc_corrected),
        .header_slot_ecc_uncorrectable(header_slot_ecc_uncorrectable),
        .trace_slot_valid(trace_slot_valid),
        .trace_slot_lane0_raw(trace_slot_lane0_raw),
        .trace_slot_lane1_raw(trace_slot_lane1_raw),
        .trace_slot_lane0_candidate(trace_slot_lane0_candidate),
        .trace_slot_lane1_candidate(trace_slot_lane1_candidate),
        .trace_slot_lane0_aligned(trace_slot_lane0_aligned),
        .trace_slot_lane1_aligned(trace_slot_lane1_aligned),
        .trace_slot_lane0_rotation(trace_slot_lane0_rotation),
        .trace_slot_lane1_rotation(trace_slot_lane1_rotation),
        .trace_slot_bitslip_phase_lane0(trace_slot_bitslip_phase_lane0),
        .trace_slot_bitslip_phase_lane1(trace_slot_bitslip_phase_lane1),
        .trace_slot_sot_hit_lane0(trace_slot_sot_hit_lane0),
        .trace_slot_sot_hit_lane1(trace_slot_sot_hit_lane1),
        .live_trace_seq(live_trace_seq),
        .live_trace_slot_valid(live_trace_slot_valid),
        .live_trace_slot_lane0_raw(live_trace_slot_lane0_raw),
        .live_trace_slot_lane1_raw(live_trace_slot_lane1_raw),
        .live_trace_slot_lane0_candidate(live_trace_slot_lane0_candidate),
        .live_trace_slot_lane1_candidate(live_trace_slot_lane1_candidate),
        .live_trace_slot_lane0_aligned(live_trace_slot_lane0_aligned),
        .live_trace_slot_lane1_aligned(live_trace_slot_lane1_aligned),
        .live_trace_slot_sot_hit_lane0(live_trace_slot_sot_hit_lane0),
        .live_trace_slot_sot_hit_lane1(live_trace_slot_sot_hit_lane1),
        .live_trace_slot_lane0_rotation(live_trace_slot_lane0_rotation),
        .live_trace_slot_lane1_rotation(live_trace_slot_lane1_rotation)
    );

    initial begin
        idelay_ref_clk = 1'b0;
        forever #2.5 idelay_ref_clk = ~idelay_ref_clk;
    end
    initial begin
        hs_clk_p = 1'b0;
        forever #5 hs_clk_p = ~hs_clk_p;
    end

    int fail_count = 0;
    int packet_idx = 0;

    // === Stimulus helpers ===
    task automatic reset_dut();
        rst_n = 1'b0;
        data_hs_p = 2'b00;
        data_lp_p = 2'b11;
        data_lp_n = 2'b11;
        repeat (8) @(posedge hs_clk_p);
        rst_n = 1'b1;
        repeat (12) @(posedge byte_clk);
    endtask

    task automatic drive_lp_state(input logic [1:0] lp_p, input logic [1:0] lp_n, input int unsigned cycles);
        @(negedge byte_clk);
        data_lp_p = lp_p;
        data_lp_n = lp_n;
        repeat (cycles) @(posedge byte_clk);
        #1;
    endtask

    // Drive serdes_byte_sample with PRE-TRANSFORM bytes (= what would be at ISERDES output).
    // Receiver applies transform (= reverse8 with FIXED_TRANSFORM=1) to get current_candidate_byte.
    // For receiver to see desired byte X, drive serdes = reverse8(X).
    task automatic drive_byte(input logic [7:0] lane0_logical, input logic [7:0] lane1_logical);
        @(negedge byte_clk);
        dut.serdes_byte_sample[0] = reverse8(lane0_logical);
        dut.serdes_byte_sample[1] = reverse8(lane1_logical);
        @(posedge byte_clk);
        #1;
    endtask

    // Wait for bitslip retrain to complete (lane_bitslip_phase[0]==6 AND [1]==6 AND sweep_hold settled)
    task automatic wait_for_retrain_done();
        int timeout = 1000;
        // Wait until lane_bitslip_phase reaches target on both lanes
        while (timeout > 0) begin
            @(posedge byte_clk);
            if (dut.lane_bitslip_phase[0] == 3'd6 &&
                dut.lane_bitslip_phase[1] == 3'd6) break;
            timeout--;
        end
        if (timeout == 0) $fatal(1, "retrain timeout");
        // Wait for sweep_hold_count to reach SWEEP_HOLD_BYTES-1 (= reset_alignment goes 0)
        timeout = 100;
        while (timeout > 0) begin
            @(posedge byte_clk);
            if (dut.sweep_hold_count >= 16'd6) break;  // SWEEP_HOLD_BYTES-2 ish
            timeout--;
        end
        repeat (4) @(posedge byte_clk);
        #1;
        $display("[t=%0t] retrain done: lane0_bitslip=%0d lane1_bitslip=%0d sweep_hold=%0d sot_win[0]=%0b sot_win[1]=%0b",
                 $time, dut.lane_bitslip_phase[0], dut.lane_bitslip_phase[1], dut.sweep_hold_count,
                 dut.sot_window_active[0], dut.sot_window_active[1]);
    endtask

    task automatic dump_lane1_path(input string label);
        $display("  [%s] t=%0t serdes_sample[1]=0x%02h candidate[1]=0x%02h has_sot[1]=%0b sot_window[1]=%0b lane_rot[1]=%0d",
                 label, $time,
                 dut.serdes_byte_sample[1],
                 // Note: current_candidate_byte and current_aligned_byte are automatic,
                 // not directly observable. Use the registered lane_last_byte instead.
                 dut.lane_last_byte[1],
                 1'b0, // placeholder - has_sot not directly exposed
                 dut.sot_window_active[1],
                 dut.lane_rotation[1]);
    endtask

    task automatic dump_trace_state();
        $display("  trace_slot_valid=%08b trace_capture_active=%0b trace_capture_done=%0b",
                 dut.trace_slot_valid, dut.trace_capture_active, dut.trace_capture_done);
        $display("  lane0 trace raw      : %02h %02h %02h %02h %02h %02h %02h %02h",
                 dut.trace_slot_lane0_raw[0], dut.trace_slot_lane0_raw[1], dut.trace_slot_lane0_raw[2], dut.trace_slot_lane0_raw[3],
                 dut.trace_slot_lane0_raw[4], dut.trace_slot_lane0_raw[5], dut.trace_slot_lane0_raw[6], dut.trace_slot_lane0_raw[7]);
        $display("  lane0 trace candidate: %02h %02h %02h %02h %02h %02h %02h %02h",
                 dut.trace_slot_lane0_candidate[0], dut.trace_slot_lane0_candidate[1], dut.trace_slot_lane0_candidate[2], dut.trace_slot_lane0_candidate[3],
                 dut.trace_slot_lane0_candidate[4], dut.trace_slot_lane0_candidate[5], dut.trace_slot_lane0_candidate[6], dut.trace_slot_lane0_candidate[7]);
        $display("  lane0 trace aligned  : %02h %02h %02h %02h %02h %02h %02h %02h",
                 dut.trace_slot_lane0_aligned[0], dut.trace_slot_lane0_aligned[1], dut.trace_slot_lane0_aligned[2], dut.trace_slot_lane0_aligned[3],
                 dut.trace_slot_lane0_aligned[4], dut.trace_slot_lane0_aligned[5], dut.trace_slot_lane0_aligned[6], dut.trace_slot_lane0_aligned[7]);
        $display("  lane1 trace raw      : %02h %02h %02h %02h %02h %02h %02h %02h",
                 dut.trace_slot_lane1_raw[0], dut.trace_slot_lane1_raw[1], dut.trace_slot_lane1_raw[2], dut.trace_slot_lane1_raw[3],
                 dut.trace_slot_lane1_raw[4], dut.trace_slot_lane1_raw[5], dut.trace_slot_lane1_raw[6], dut.trace_slot_lane1_raw[7]);
        $display("  lane1 trace candidate: %02h %02h %02h %02h %02h %02h %02h %02h",
                 dut.trace_slot_lane1_candidate[0], dut.trace_slot_lane1_candidate[1], dut.trace_slot_lane1_candidate[2], dut.trace_slot_lane1_candidate[3],
                 dut.trace_slot_lane1_candidate[4], dut.trace_slot_lane1_candidate[5], dut.trace_slot_lane1_candidate[6], dut.trace_slot_lane1_candidate[7]);
        $display("  lane1 trace aligned  : %02h %02h %02h %02h %02h %02h %02h %02h",
                 dut.trace_slot_lane1_aligned[0], dut.trace_slot_lane1_aligned[1], dut.trace_slot_lane1_aligned[2], dut.trace_slot_lane1_aligned[3],
                 dut.trace_slot_lane1_aligned[4], dut.trace_slot_lane1_aligned[5], dut.trace_slot_lane1_aligned[6], dut.trace_slot_lane1_aligned[7]);
    endtask

    task automatic check_byte(input string name, input logic [7:0] actual, input logic [7:0] expected);
        if (actual !== expected) begin
            $display("  ## FAIL: %s actual=0x%02h expected=0x%02h", name, actual, expected);
            fail_count++;
        end else begin
            $display("  PASS: %s = 0x%02h", name, actual);
        end
    endtask

    // Emit one CSI-2 long packet header (DI=0x1E, WC=1280, ECC=0x1E) with LP→HS transition
    // and a few payload bytes. Verify lane 1 trace slot[2] = 0x1E.
    task automatic emit_one_packet(input logic [7:0] payload_seed);
        automatic logic [7:0] ecc_byte;
        automatic logic [5:0] computed_ecc;

        packet_idx++;
        $display("\n=== PACKET %0d (LP→HS, long packet header + 4 payload bytes) ===", packet_idx);

        // Compute ECC for (DI=0x1E, WC=1280=0x0500) using independent reference
        computed_ecc = ref_ecc6({16'd1280, 8'h1e});
        ecc_byte = {2'b00, computed_ecc};
        $display("  expected ECC for (DI=0x1E, WC=1280) = 0x%02h", ecc_byte);

        // LP-11 → LP-00 (HS entry)
        drive_lp_state(2'b11, 2'b11, 2);
        drive_lp_state(2'b00, 2'b00, 4);

        // SoT byte on both lanes simultaneously, then long packet header bytes.
        // Receiver expects current_candidate=0xB8 for SoT detection.
        // With FIXED_TRANSFORM=1, drive serdes_byte_sample = reverse8(0xB8) = 0x1D.
        // For data, drive serdes = reverse8(logical_byte) handled by drive_byte().

        drive_byte(8'hB8, 8'hB8);  // slot[0] SoT
        drive_byte(8'h1E, 8'h00);  // slot[1] DI / WC[7:0]
        drive_byte(8'h05, ecc_byte); // slot[2] WC[15:8] / ECC
        drive_byte(payload_seed + 8'h00, payload_seed + 8'h01); // slot[3]
        drive_byte(payload_seed + 8'h02, payload_seed + 8'h03); // slot[4]
        drive_byte(payload_seed + 8'h04, payload_seed + 8'h05); // slot[5]
        drive_byte(payload_seed + 8'h06, payload_seed + 8'h07); // slot[6]
        drive_byte(payload_seed + 8'h08, payload_seed + 8'h09); // slot[7]

        // Allow trace state to settle
        repeat (4) @(posedge byte_clk);
        #1;

        $display("\n  After packet %0d, trace state:", packet_idx);
        dump_trace_state();

        // === CRITICAL CHECK ===
        $display("\n  Lane 1 slot[2] check (= ECC byte position):");
        check_byte("lane1 slot[2] aligned", trace_slot_lane1_aligned[2], 8'h1e);
        check_byte("lane1 slot[1] aligned", trace_slot_lane1_aligned[1], 8'h00);
        check_byte("lane0 slot[1] aligned", trace_slot_lane0_aligned[1], 8'h1e);
        check_byte("lane0 slot[2] aligned", trace_slot_lane0_aligned[2], 8'h05);

        // EoT
        drive_lp_state(2'b11, 2'b11, 4);
    endtask

    // === Main test ===
    initial begin
        $display("=== tb_dphy_lane1_trace ===");
        $display("DUT params: FIXED_BITSLIP_PHASE=6/6 FIXED_TRANSFORM=1 (matches deployed)");

        reset_dut();
        wait_for_retrain_done();

        // Drive 5 long packets back-to-back with LP transitions
        for (int p = 0; p < 5; p++) begin
            emit_one_packet(8'h10 + p[7:0] * 8'h10);
            // Force trace_capture state to ready for next event by waiting more cycles
            repeat (40) @(posedge byte_clk);
            #1;
        end

        if (fail_count == 0) begin
            $display("\n=========================================");
            $display("TEST PASSED: lane 1 trace correct across all packets");
            $display("=========================================");
        end else begin
            $display("\n=========================================");
            $display("TEST FAILED: %0d failures", fail_count);
            $display("=========================================");
        end
        $finish;
    end

    initial begin
        #50_000_000;
        $display("FAIL: watchdog timeout");
        $fatal(1, "watchdog");
    end

endmodule
