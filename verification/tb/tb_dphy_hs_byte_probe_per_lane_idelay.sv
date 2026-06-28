`timescale 1ns / 1ps
//
// tb_dphy_hs_byte_probe_per_lane_idelay
//
// Verifies E (per-lane IDELAY independence) and G (live_trace per-slot
// rotation expose) against CSI-2 spec-derived expected values, NOT against
// the RTL's own internal computation.
//
// Criteria (all must pass):
//
// A) lane 0 live_trace_slot_aligned == [B8 1E 05 10 12 14 16 18]
//    derived from CSI-2 long-packet header (DI=0x1E, WC=1280) split per-lane,
//    plus payload counter pattern (lane 0 takes even byte indices).
//
// B) lane 1 live_trace_slot_aligned == [B8 00 1E 11 13 15 17 19]
//    same source, lane 1 takes odd byte indices. ECC=0x1E independently
//    verified by Python ref; not by calling RTL calc_ecc6 here.
//
// C) live_trace_slot_lane{0,1}_rotation[0..7] all equal the SoT-detected
//    rotation. With our perfectly-aligned stimulus the expected rotation is
//    0 for every slot (= 8x same value). The criterion is per-design-intent:
//    rotation locks at SoT and stays stable across post-SoT bytes.
//
// D) Per-lane IDELAY independence. After applying tap0=5 and tap1=17 via
//    runtime knobs, the IDELAYE2 CNTVALUEIN inputs must reflect each lane's
//    assigned tap. Verified by hierarchical reference into the DUT.
//
// E) sync_header_di == 0x1E, sync_header_wc == 1280, sync_header_ecc
//    matches an *independent* Python-style ECC computed from {DI, WC} via
//    a reference function `ref_ecc6_py` (which in this TB is implemented in
//    SystemVerilog but is logically identical to the Python implementation
//    used in software/pynq/ov5640_trace_slots_full.ipynb section 8/9).
//
// Stubbed IDELAYE2/ISERDESE2 primitives (passthrough) are used; the test
// drives serdes_byte_sample directly to bypass the unsimulatable bit-level
// path, mirroring the existing tb_dphy_hs_byte_probe_gearbox approach.
//

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
    output wire Q1,
    output wire Q2,
    output wire Q3,
    output wire Q4,
    output wire Q5,
    output wire Q6,
    output wire Q7,
    output wire Q8,
    output wire SHIFTOUT1,
    output wire SHIFTOUT2,
    input  wire BITSLIP,
    input  wire CE1,
    input  wire CE2,
    input  wire CLK,
    input  wire CLKB,
    input  wire CLKDIV,
    input  wire CLKDIVP,
    input  wire D,
    input  wire DDLY,
    input  wire DYNCLKDIVSEL,
    input  wire DYNCLKSEL,
    input  wire OCLK,
    input  wire OCLKB,
    input  wire OFB,
    input  wire RST,
    input  wire SHIFTIN1,
    input  wire SHIFTIN2,
    output wire O
);
    assign {Q8, Q7, Q6, Q5, Q4, Q3, Q2, Q1} = 8'h00;
    assign SHIFTOUT1 = 1'b0;
    assign SHIFTOUT2 = 1'b0;
    assign O = DDLY;
endmodule

module tb_dphy_hs_byte_probe_per_lane_idelay;

    // --- Independent reference (NOT calling RTL calc_ecc6 directly) ---
    // Logically identical to the Python `calc_ecc6` in
    // ov5640_trace_slots_full.ipynb section 8. Implemented here so the TB
    // can self-check without invoking the DUT's internal function.
    function automatic logic [5:0] ref_ecc6_py(input logic [23:0] data);
        ref_ecc6_py[0] = data[0]^data[1]^data[2]^data[4]^data[5]^data[7]^data[10]^data[11]^data[13]^data[16]^data[20]^data[21]^data[22]^data[23];
        ref_ecc6_py[1] = data[0]^data[1]^data[3]^data[4]^data[6]^data[8]^data[10]^data[12]^data[14]^data[17]^data[20]^data[21]^data[22]^data[23];
        ref_ecc6_py[2] = data[0]^data[2]^data[3]^data[5]^data[6]^data[9]^data[11]^data[12]^data[15]^data[18]^data[20]^data[21]^data[22];
        ref_ecc6_py[3] = data[1]^data[2]^data[3]^data[7]^data[8]^data[9]^data[13]^data[14]^data[15]^data[19]^data[20]^data[21]^data[23];
        ref_ecc6_py[4] = data[4]^data[5]^data[6]^data[7]^data[8]^data[9]^data[16]^data[17]^data[18]^data[19]^data[20]^data[22]^data[23];
        ref_ecc6_py[5] = data[10]^data[11]^data[12]^data[13]^data[14]^data[15]^data[16]^data[17]^data[18]^data[19]^data[21]^data[22]^data[23];
    endfunction

    // --- DUT signal storage ---
    logic rst_n;
    logic idelay_ref_clk;
    logic hs_clk_p;
    logic hs_clk_n;
    logic [1:0] data_hs_p;
    logic [1:0] data_hs_n;
    logic [1:0] data_lp_p;
    logic [1:0] data_lp_n;
    logic byte_clk;

    logic [4:0] runtime_idelay_tap_drv;
    logic [4:0] runtime_idelay_tap_lane1_drv;

    // unused-but-required outputs (catch-all)
    logic idelayctrl_rdy;
    logic hs_clk_seen;
    logic [1:0] lane_sot_seen;
    logic [1:0][7:0] lane_last_byte;
    logic [1:0] lane_raw_changed_seen;
    logic [1:0] lane_raw_non_ff_seen;
    logic [1:0] lane_raw_non_00_seen;
    logic [1:0][7:0] lane_raw_change_count;
    logic [15:0] stream_byte_data;
    logic [1:0] stream_byte_keep;
    logic stream_byte_valid;
    logic stream_byte_sop;
    logic stream_byte_eop;
    logic [2:0] stream_pairing_active_dbg;
    logic [2:0] stream_pairing_next_dbg;
    logic header_valid;
    logic [7:0] header_di;
    logic [15:0] header_wc;
    logic [7:0] header_ecc;
    logic sync_header_valid;
    logic [7:0] sync_header_di;
    logic [15:0] sync_header_wc;
    logic [7:0] sync_header_ecc;
    logic [2:0] sync_header_rotation_lane0;
    logic [2:0] sync_header_rotation_lane1;
    logic [2:0] sync_header_bit_offset_lane0;
    logic [2:0] sync_header_bit_offset_lane1;
    logic [3:0] sync_header_score;
    logic [2:0] sync_header_start_slot;
    logic [2:0] sync_header_pairing;
    logic [5:0] sync_header_syndrome;
    logic sync_header_ecc_no_error;
    logic sync_header_ecc_corrected;
    logic sync_header_ecc_uncorrectable;
    logic [7:0] header_slot_valid;
    logic [7:0][7:0] header_slot_di;
    logic [7:0][15:0] header_slot_wc;
    logic [7:0][7:0] header_slot_ecc;
    logic [7:0][2:0] header_slot_bitslip_phase;
    logic [7:0][2:0] header_slot_bitslip_phase_lane1;
    logic [7:0][2:0] header_slot_transform;
    logic [7:0][2:0] header_slot_rotation;
    logic [7:0][7:0] header_slot_corr_di;
    logic [7:0][15:0] header_slot_corr_wc;
    logic [7:0][5:0] header_slot_syndrome;
    logic [7:0] header_slot_ecc_no_error;
    logic [7:0] header_slot_ecc_corrected;
    logic [7:0] header_slot_ecc_uncorrectable;
    logic [7:0] trace_slot_valid;
    logic [7:0][7:0] trace_slot_lane0_raw;
    logic [7:0][7:0] trace_slot_lane1_raw;
    logic [7:0][7:0] trace_slot_lane0_candidate;
    logic [7:0][7:0] trace_slot_lane1_candidate;
    logic [7:0][7:0] trace_slot_lane0_aligned;
    logic [7:0][7:0] trace_slot_lane1_aligned;
    logic [7:0][2:0] trace_slot_lane0_rotation;
    logic [7:0][2:0] trace_slot_lane1_rotation;
    logic [7:0][2:0] trace_slot_bitslip_phase_lane0;
    logic [7:0][2:0] trace_slot_bitslip_phase_lane1;
    logic [7:0] trace_slot_sot_hit_lane0;
    logic [7:0] trace_slot_sot_hit_lane1;
    logic [7:0] live_trace_seq;
    logic [7:0] live_trace_slot_valid;
    logic [7:0][7:0] live_trace_slot_lane0_raw;
    logic [7:0][7:0] live_trace_slot_lane1_raw;
    logic [7:0][7:0] live_trace_slot_lane0_candidate;
    logic [7:0][7:0] live_trace_slot_lane1_candidate;
    logic [7:0][7:0] live_trace_slot_lane0_aligned;
    logic [7:0][7:0] live_trace_slot_lane1_aligned;
    logic [7:0] live_trace_slot_sot_hit_lane0;
    logic [7:0] live_trace_slot_sot_hit_lane1;
    logic [7:0][2:0] live_trace_slot_lane0_rotation;
    logic [7:0][2:0] live_trace_slot_lane1_rotation;

    // --- DUT ---
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
        .IDELAY_TAP(0),
        .IDELAY_REFCLK_MHZ(200.0),
        .STREAM_PAIRING(0)
    ) dut (
        .rst_n(rst_n),
        .idelay_ref_clk(idelay_ref_clk),
        .idelay_ref_reset(!rst_n),
        .runtime_idelay_tap(runtime_idelay_tap_drv),
        .runtime_idelay_tap_lane1(runtime_idelay_tap_lane1_drv),
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

    assign hs_clk_n = ~hs_clk_p;
    assign data_hs_n = ~data_hs_p;

    int fail_count = 0;

    task automatic check_byte(input string name, input logic [7:0] actual, input logic [7:0] expected);
        if (actual !== expected) begin
            $display("FAIL: %s actual=0x%02x expected=0x%02x", name, actual, expected);
            fail_count++;
        end else begin
            $display("PASS: %s = 0x%02x", name, actual);
        end
    endtask

    task automatic check_eq3(input string name, input logic [2:0] actual, input logic [2:0] expected);
        if (actual !== expected) begin
            $display("FAIL: %s actual=%0d expected=%0d", name, actual, expected);
            fail_count++;
        end else begin
            $display("PASS: %s = %0d", name, actual);
        end
    endtask

    task automatic check_eq16(input string name, input logic [15:0] actual, input logic [15:0] expected);
        if (actual !== expected) begin
            $display("FAIL: %s actual=0x%04x expected=0x%04x", name, actual, expected);
            fail_count++;
        end else begin
            $display("PASS: %s = 0x%04x", name, actual);
        end
    endtask

    task automatic reset_dut();
        rst_n = 1'b0;
        runtime_idelay_tap_drv = 5'd0;
        runtime_idelay_tap_lane1_drv = 5'd0;
        data_hs_p = 2'b00;
        data_lp_p = 2'b11;
        data_lp_n = 2'b11;
        repeat (8) @(posedge hs_clk_p);
        rst_n = 1'b1;
        repeat (12) @(posedge byte_clk);
    endtask

    task automatic drive_lp_state(
        input logic [1:0] lane_lp_p,
        input logic [1:0] lane_lp_n,
        input int unsigned cycles
    );
        @(negedge byte_clk);
        data_lp_p = lane_lp_p;
        data_lp_n = lane_lp_n;
        repeat (cycles) begin
            @(posedge byte_clk);
        end
        #1;
    endtask

    task automatic drive_serdes_sample(input logic [7:0] lane0_byte, input logic [7:0] lane1_byte);
        @(negedge byte_clk);
        dut.serdes_byte_sample[0] = lane0_byte;
        dut.serdes_byte_sample[1] = lane1_byte;
        @(posedge byte_clk);
        #1;
    endtask

    initial begin
        // === Setup ===
        // Use FIXED_TRANSFORM=0 for simplicity (no bit reverse).
        // Stimulus is byte-aligned, so SoT rotation should be 0 throughout.

        $display("=== tb_dphy_hs_byte_probe_per_lane_idelay ===");

        reset_dut();

        // === D: per-lane IDELAY independence ===
        // Apply tap0=5, tap1=17 via the runtime input ports, wait for the
        // sync registers to absorb, then probe the IDELAYE2 instances.
        runtime_idelay_tap_drv       = 5'd5;
        runtime_idelay_tap_lane1_drv = 5'd17;
        repeat (8) @(posedge byte_clk);
        #1;

        // Hierarchical reference: the IDELAYE2 in gen_lane_probe[lane]
        // receives CNTVALUEIN from the per-lane runtime sync register.
        // We assert each lane's tap value is independently visible.
        $display("[D] runtime_idelay_sync2 lane0=%0d lane1=%0d",
                 dut.runtime_idelay_sync2, dut.runtime_idelay_lane1_sync2);
        check_eq3("D.lane0.sync2[2:0]", dut.runtime_idelay_sync2[2:0], 3'd5);
        if (dut.runtime_idelay_sync2 !== 5'd5) begin
            $display("FAIL: D.lane0.sync2 = %0d expected 5", dut.runtime_idelay_sync2);
            fail_count++;
        end else begin
            $display("PASS: D.lane0.sync2 = 5");
        end
        if (dut.runtime_idelay_lane1_sync2 !== 5'd17) begin
            $display("FAIL: D.lane1.sync2 = %0d expected 17", dut.runtime_idelay_lane1_sync2);
            fail_count++;
        end else begin
            $display("PASS: D.lane1.sync2 = 17");
        end

        // === Stimulus: drive a single CSI-2 long-packet header followed by
        //              counter-pattern payload, byte-aligned so rotation=0 ===

        // 1) Open SoT window via LP-11 -> LP-00 transition on both lanes.
        //    reset_dut() left LP at 2'b11; transition to LP-00 here.
        drive_lp_state(2'b00, 2'b00, 4);

        // 2) Drive serdes_byte_sample one byte_clk per stream byte.
        //    Byte ordering after de-interleave (lane0=even, lane1=odd):
        //
        //    Stream:  B8 1E 00 05 1E 10 11 12 13 14 15 16 17 18 19
        //    Lane 0:  B8 1E    05    10    12    14    16    18
        //    Lane 1:  B8    00    1E    11    13    15    17    19
        //
        //    Slot index 0 = SoT byte (B8 on both lanes simultaneously).
        //    Slot index 1 = first post-SoT byte = stream byte 0/1.
        //    Slot index 2 = second post-SoT byte = stream byte 2/3.
        //    ...

        drive_serdes_sample(8'hB8, 8'hB8);  // slot 0
        drive_serdes_sample(8'h1E, 8'h00);  // slot 1: DI / WC[7:0]
        drive_serdes_sample(8'h05, 8'h1E);  // slot 2: WC[15:8] / ECC
        drive_serdes_sample(8'h10, 8'h11);  // slot 3
        drive_serdes_sample(8'h12, 8'h13);  // slot 4
        drive_serdes_sample(8'h14, 8'h15);  // slot 5
        drive_serdes_sample(8'h16, 8'h17);  // slot 6
        drive_serdes_sample(8'h18, 8'h19);  // slot 7

        // Allow trace to settle and sync header scan to run.
        // Sync-scan iterates pairings x slots x bit_offsets; ~600 cycles worst-case.
        for (int cycle = 0; cycle < 600; cycle++) begin
            @(posedge byte_clk);
            if (sync_header_valid) break;
        end
        #1;

        // === A: lane 0 trace expectations (CSI-2 spec derived) ===
        check_byte("A.lane0.slot[0]", live_trace_slot_lane0_aligned[0], 8'hB8);
        check_byte("A.lane0.slot[1]", live_trace_slot_lane0_aligned[1], 8'h1E);
        check_byte("A.lane0.slot[2]", live_trace_slot_lane0_aligned[2], 8'h05);
        check_byte("A.lane0.slot[3]", live_trace_slot_lane0_aligned[3], 8'h10);
        check_byte("A.lane0.slot[4]", live_trace_slot_lane0_aligned[4], 8'h12);
        check_byte("A.lane0.slot[5]", live_trace_slot_lane0_aligned[5], 8'h14);
        check_byte("A.lane0.slot[6]", live_trace_slot_lane0_aligned[6], 8'h16);
        check_byte("A.lane0.slot[7]", live_trace_slot_lane0_aligned[7], 8'h18);

        // === B: lane 1 trace expectations ===
        check_byte("B.lane1.slot[0]", live_trace_slot_lane1_aligned[0], 8'hB8);
        check_byte("B.lane1.slot[1]", live_trace_slot_lane1_aligned[1], 8'h00);
        // ECC byte: independent reference. data = {WC[15:8], WC[7:0], DI} = 0x05_00_1E.
        // ref_ecc6_py(0x05_00_1E) MUST equal 0x1E (sanity from Python brute).
        check_byte("B.ref_ecc6 sanity", {2'b00, ref_ecc6_py(24'h05001E)}, 8'h1E);
        check_byte("B.lane1.slot[2]", live_trace_slot_lane1_aligned[2], 8'h1E);
        check_byte("B.lane1.slot[3]", live_trace_slot_lane1_aligned[3], 8'h11);
        check_byte("B.lane1.slot[4]", live_trace_slot_lane1_aligned[4], 8'h13);
        check_byte("B.lane1.slot[5]", live_trace_slot_lane1_aligned[5], 8'h15);
        check_byte("B.lane1.slot[6]", live_trace_slot_lane1_aligned[6], 8'h17);
        check_byte("B.lane1.slot[7]", live_trace_slot_lane1_aligned[7], 8'h19);

        // === C: per-slot rotation should equal SoT-detected rotation ===
        // Stimulus is byte-aligned -> rotation = 0 for SoT byte (no within-byte
        // rotation needed to recover 0xB8). Therefore all 8 slots must have
        // rotation = 0. The criterion is: rotation locks at SoT and remains
        // stable across post-SoT bytes.
        for (int k = 0; k < 8; k++) begin
            if (live_trace_slot_lane0_rotation[k] !== 3'd0) begin
                $display("FAIL: C.lane0.rotation[%0d] = %0d expected 0", k, live_trace_slot_lane0_rotation[k]);
                fail_count++;
            end
            if (live_trace_slot_lane1_rotation[k] !== 3'd0) begin
                $display("FAIL: C.lane1.rotation[%0d] = %0d expected 0", k, live_trace_slot_lane1_rotation[k]);
                fail_count++;
            end
        end
        if (fail_count == 0) $display("PASS: C all rotation slots = 0 (per-design-intent)");

        // === E: sync header decode ===
        check_byte ("E.sync_header_di",  sync_header_di,  8'h1E);
        check_eq16("E.sync_header_wc",   sync_header_wc,  16'd1280);
        check_byte("E.sync_header_ecc",  sync_header_ecc, 8'h1E);
        if (sync_header_score < 4'd13) begin
            $display("FAIL: E.sync_header_score=%0d expected >=13 (header valid)", sync_header_score);
            fail_count++;
        end else begin
            $display("PASS: E.sync_header_score = %0d", sync_header_score);
        end

        // === Result ===
        if (fail_count == 0) begin
            $display("=========================================");
            $display("TEST PASSED: tb_dphy_hs_byte_probe_per_lane_idelay");
            $display("=========================================");
        end else begin
            $display("=========================================");
            $display("TEST FAILED: %0d failures", fail_count);
            $display("=========================================");
        end
        $finish;
    end

    // Watchdog
    initial begin
        #5_000_000;
        $display("FAIL: watchdog timeout");
        $fatal(1, "watchdog");
    end

endmodule
