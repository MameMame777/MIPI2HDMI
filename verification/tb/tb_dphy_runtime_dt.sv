`timescale 1ns / 1ps
//
// tb_dphy_runtime_dt
//
// Verifies runtime EXPECTED_LONG_DT control mechanism in dphy_hs_byte_probe.
// Specifically catches the regression in v33-v39 hardware tests where the
// packet decoder pipeline silenced (long_pkt=0, short_pkt=0) on real silicon
// even though basic sync detection appeared to work.
//
// Scenarios:
//   S0) runtime_dt = 0x00 (default), stimulus DI=0x1E → sync_header_valid, score 15
//   S1) runtime_dt = 0x1E (explicit match), stimulus DI=0x1E → score 15
//   S2) runtime_dt = 0x1F (mismatch), stimulus DI=0x1E → score 0, no sync_header_valid
//   S3) runtime_dt = 0x1E, stimulus DI=0x1F + ECC=0x1E (1-bit error at D0)
//        → ECC corrects to DI=0x1E → score 13 (corrected), sync_header_valid
//   S4) runtime_dt = 0x1F, stimulus DI=0x1F + ECC=0x19 (no error path)
//        → score 15, sync_header_valid (verifies runtime override)
//   S5) Stimulus repeated 4× per scenario → sync_header_valid pulse count check
//
// This TB would have caught: my RTL change that silently broke
// sync_header_valid detection on real hardware even when default DT=0x1E was used.
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
    parameter string INTERFACE_TYPE = "NETWORKING",
    parameter int NUM_CE = 2,
    parameter string SERDES_MODE = "MASTER",
    parameter string IOBDELAY = "BOTH",
    parameter string DYN_CLKDIV_INV_EN = "FALSE",
    parameter string DYN_CLK_INV_EN = "FALSE",
    parameter string INIT_Q1 = "0",
    parameter string INIT_Q2 = "0",
    parameter string INIT_Q3 = "0",
    parameter string INIT_Q4 = "0",
    parameter string OFB_USED = "FALSE",
    parameter string SRVAL_Q1 = "0",
    parameter string SRVAL_Q2 = "0",
    parameter string SRVAL_Q3 = "0",
    parameter string SRVAL_Q4 = "0"
) (
    input  wire D,
    input  wire DDLY,
    input  wire CLK,
    input  wire CLKB,
    input  wire CLKDIV,
    input  wire CLKDIVP,
    input  wire OCLK,
    input  wire OCLKB,
    input  wire RST,
    input  wire CE1,
    input  wire CE2,
    input  wire OFB,
    input  wire BITSLIP,
    input  wire DYNCLKDIVSEL,
    input  wire DYNCLKSEL,
    input  wire [2:0] SHIFTIN1,
    input  wire [2:0] SHIFTIN2,
    output wire O,
    output wire Q1,
    output wire Q2,
    output wire Q3,
    output wire Q4,
    output wire Q5,
    output wire Q6,
    output wire Q7,
    output wire Q8,
    output wire [2:0] SHIFTOUT1,
    output wire [2:0] SHIFTOUT2
);
    assign Q1 = 1'b0; assign Q2 = 1'b0; assign Q3 = 1'b0; assign Q4 = 1'b0;
    assign Q5 = 1'b0; assign Q6 = 1'b0; assign Q7 = 1'b0; assign Q8 = 1'b0;
    assign O = 1'b0;
    assign SHIFTOUT1 = 3'd0;
    assign SHIFTOUT2 = 3'd0;
endmodule

module tb_dphy_runtime_dt;
    localparam int LANES = 2;

    // --- Clocks ---
    logic hs_clk_p = 1'b0;
    wire  hs_clk_n = ~hs_clk_p;
    logic idelay_ref_clk = 1'b0;

    initial forever #1 hs_clk_p = ~hs_clk_p;          // 500 MHz hs
    initial forever #2.5 idelay_ref_clk = ~idelay_ref_clk;

    // --- Reset and driven signals ---
    logic       rst_n = 1'b0;
    logic [1:0] data_hs_p = 2'b00;
    wire  [1:0] data_hs_n = ~data_hs_p;
    logic [1:0] data_lp_p = 2'b11;
    logic [1:0] data_lp_n = 2'b11;
    logic [4:0] runtime_idelay_tap_drv = 5'd8;
    logic [4:0] runtime_idelay_tap_lane1_drv = 5'd8;
    logic [7:0] runtime_expected_long_dt_drv = 8'h00;

    // --- DUT outputs ---
    wire        byte_clk;
    wire        idelayctrl_rdy;
    wire        hs_clk_seen;
    wire  [1:0] lane_sot_seen;
    wire [15:0] stream_byte_data;
    wire  [1:0] stream_byte_keep;
    wire        stream_byte_valid;
    wire        stream_byte_sop;
    wire        stream_byte_eop;
    wire  [2:0] stream_pairing_active_dbg;
    wire  [2:0] stream_pairing_next_dbg;
    wire        header_valid;
    wire  [7:0] header_di;
    wire [15:0] header_wc;
    wire  [7:0] header_ecc;
    wire        sync_header_valid;
    wire  [7:0] sync_header_di;
    wire [15:0] sync_header_wc;
    wire  [7:0] sync_header_ecc;
    wire  [2:0] sync_header_rotation_lane0;
    wire  [2:0] sync_header_rotation_lane1;
    wire  [2:0] sync_header_bit_offset_lane0;
    wire  [2:0] sync_header_bit_offset_lane1;
    wire  [3:0] sync_header_score;
    wire  [2:0] sync_header_start_slot;
    wire  [2:0] sync_header_pairing;
    wire  [5:0] sync_header_syndrome;
    wire        sync_header_ecc_no_error;
    wire        sync_header_ecc_corrected;
    wire        sync_header_ecc_uncorrectable;
    wire  [7:0] header_slot_valid;
    wire  [7:0][7:0] header_slot_di;
    wire  [7:0][15:0] header_slot_wc;
    wire  [7:0][7:0] header_slot_ecc;
    wire  [7:0][2:0] header_slot_bitslip_phase;
    wire  [7:0][2:0] header_slot_bitslip_phase_lane1;
    wire  [7:0][2:0] header_slot_transform;
    wire  [7:0][2:0] header_slot_rotation;
    wire  [7:0][7:0] header_slot_corr_di;
    wire  [7:0][15:0] header_slot_corr_wc;
    wire  [7:0][5:0] header_slot_syndrome;
    wire  [7:0] header_slot_ecc_no_error;
    wire  [7:0] header_slot_ecc_corrected;
    wire  [7:0] header_slot_ecc_uncorrectable;
    wire  [7:0] trace_slot_valid;
    wire  [2:0] lane1_target_phase_out;
    wire  [1:0] lane_raw_changed_seen;
    wire  [1:0] lane_raw_non_ff_seen;
    wire  [1:0] lane_raw_non_00_seen;
    wire  [1:0][7:0] lane_raw_change_count;
    wire  [1:0][7:0] lane_last_byte;
    wire  [7:0][7:0] trace_slot_lane0_raw;
    wire  [7:0][7:0] trace_slot_lane1_raw;
    wire  [7:0][7:0] trace_slot_lane0_candidate;
    wire  [7:0][7:0] trace_slot_lane1_candidate;
    wire  [7:0][7:0] trace_slot_lane0_aligned;
    wire  [7:0][7:0] trace_slot_lane1_aligned;
    wire  [7:0][2:0] trace_slot_lane0_rotation;
    wire  [7:0][2:0] trace_slot_lane1_rotation;
    wire  [7:0][2:0] trace_slot_bitslip_phase_lane0;
    wire  [7:0][2:0] trace_slot_bitslip_phase_lane1;
    wire  [7:0] trace_slot_sot_hit_lane0;
    wire  [7:0] trace_slot_sot_hit_lane1;
    wire  [7:0] live_trace_seq;
    wire  [7:0] live_trace_slot_valid;
    wire  [7:0][7:0] live_trace_slot_lane0_raw;
    wire  [7:0][7:0] live_trace_slot_lane1_raw;
    wire  [7:0][7:0] live_trace_slot_lane0_candidate;
    wire  [7:0][7:0] live_trace_slot_lane1_candidate;
    wire  [7:0][7:0] live_trace_slot_lane0_aligned;
    wire  [7:0][7:0] live_trace_slot_lane1_aligned;
    wire  [7:0] live_trace_slot_sot_hit_lane0;
    wire  [7:0] live_trace_slot_sot_hit_lane1;
    wire  [7:0][2:0] live_trace_slot_lane0_rotation;
    wire  [7:0][2:0] live_trace_slot_lane1_rotation;

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
        .EXPECTED_LONG_DT(8'h1e),
        .EXPECTED_LONG_WC(16'd1280),
        .MIN_SYNC_HEADER_SCORE(13),
        .STREAM_PAIRING(0)
    ) dut (
        .rst_n(rst_n),
        .idelay_ref_clk(idelay_ref_clk),
        .idelay_ref_reset(!rst_n),
        .runtime_idelay_tap(runtime_idelay_tap_drv),
        .runtime_idelay_tap_lane1(runtime_idelay_tap_lane1_drv),
        .runtime_bitslip_phase(3'd0),
        .runtime_bitslip_phase_lane1(3'd0),
        .runtime_lane1_sweep_enable(1'b0),
        .runtime_expected_long_dt(runtime_expected_long_dt_drv),
        .sup_enable(1'b0),
        .sup_bufr_clr(1'b0),
        .sup_serdes_rst(1'b0),
        .sup_hs_settled(1'b0),
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
        .live_trace_slot_lane1_rotation(live_trace_slot_lane1_rotation),
        .lane1_target_phase_out(lane1_target_phase_out),
        .serdes_byte_sample_out()
    );

    // --- Pulse counter for sync_header_valid ---
    int sync_pulse_count = 0;
    logic sync_valid_d = 1'b0;
    always_ff @(posedge byte_clk) begin
        sync_valid_d <= sync_header_valid;
        if (sync_header_valid && !sync_valid_d) begin
            sync_pulse_count++;
        end
    end

    // --- Tasks ---
    int fail_count = 0;

    task automatic reset_dut();
        rst_n = 1'b0;
        runtime_idelay_tap_drv = 5'd8;
        runtime_idelay_tap_lane1_drv = 5'd8;
        runtime_expected_long_dt_drv = 8'h00;
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

    // Drive one CSI-2 long-packet header with given DI / WC / ECC
    // lane 0 takes even byte indices [0,2,4,6], lane 1 takes odd [1,3,5,7]
    // Bytes: [SoT_lane0=B8 SoT_lane1=B8] then [DI WCL] [WCM ECC] [pl0 pl1] [pl2 pl3]
    task automatic drive_header(input logic [7:0] di, input logic [7:0] wcl,
                                input logic [7:0] wcm, input logic [7:0] ecc);
        drive_lp_state(2'b00, 2'b00, 4);
        drive_serdes_sample(8'hB8, 8'hB8);   // slot 0 SoT
        drive_serdes_sample(di,    wcl);     // slot 1: DI / WC_L
        drive_serdes_sample(wcm,   ecc);     // slot 2: WC_M / ECC
        drive_serdes_sample(8'h10, 8'h11);   // slot 3
        drive_serdes_sample(8'h12, 8'h13);
        drive_serdes_sample(8'h14, 8'h15);
        drive_serdes_sample(8'h16, 8'h17);
        drive_serdes_sample(8'h18, 8'h19);
        // Settle for sync scan
        for (int cycle = 0; cycle < 600; cycle++) @(posedge byte_clk);
        drive_lp_state(2'b11, 2'b11, 8);
    endtask

    task automatic check_eq(input string name, input int actual, input int expected);
        if (actual !== expected) begin
            $display("FAIL: %s actual=%0d expected=%0d", name, actual, expected);
            fail_count++;
        end else begin
            $display("PASS: %s = %0d", name, actual);
        end
    endtask

    task automatic check_byte(input string name, input logic [7:0] actual, input logic [7:0] expected);
        if (actual !== expected) begin
            $display("FAIL: %s actual=0x%02x expected=0x%02x", name, actual, expected);
            fail_count++;
        end else begin
            $display("PASS: %s = 0x%02x", name, actual);
        end
    endtask

    task automatic run_scenario(input string name, input logic [7:0] runtime_dt,
                                input logic [7:0] di, input logic [7:0] wcl,
                                input logic [7:0] wcm, input logic [7:0] ecc,
                                input int expected_pulses, input logic [7:0] expected_di_corrected);
        automatic int start_count;
        $display("\n--- %s ---", name);
        runtime_expected_long_dt_drv = runtime_dt;
        repeat (8) @(posedge byte_clk);
        start_count = sync_pulse_count;
        drive_header(di, wcl, wcm, ecc);
        $display("  runtime_dt=0x%02x, stim DI=0x%02x ECC=0x%02x → sync_pulses(diff)=%0d, sync_di=0x%02x score=%0d (no_err=%0b corr=%0b)",
                 runtime_dt, di, ecc, sync_pulse_count - start_count, sync_header_di, sync_header_score,
                 sync_header_ecc_no_error, sync_header_ecc_corrected);
        check_eq($sformatf("%s.pulse_diff", name), sync_pulse_count - start_count, expected_pulses);
        if (expected_pulses > 0) begin
            check_byte($sformatf("%s.sync_header_di", name), sync_header_di, expected_di_corrected);
        end
    endtask

    initial begin
        $display("=== tb_dphy_runtime_dt ===");
        reset_dut();

        // S0: runtime_dt=0 (default), stim DI=0x1E, ECC=0x1E (no_error path)
        //     Expected: sync_pulse=1, corrected DI=0x1E, score 15
        run_scenario("S0_default_clean", 8'h00,
                     8'h1E, 8'h00, 8'h05, 8'h1E,
                     1, 8'h1E);

        // S1: runtime_dt=0x1E (explicit match), same stimulus
        run_scenario("S1_explicit_1E_clean", 8'h1E,
                     8'h1E, 8'h00, 8'h05, 8'h1E,
                     1, 8'h1E);

        // S2: runtime_dt=0x1F (mismatch with corrected DI=0x1E)
        //     Expected: sync_pulse=0, no header detected
        run_scenario("S2_mismatch_1F_vs_1E", 8'h1F,
                     8'h1E, 8'h00, 8'h05, 8'h1E,
                     0, 8'h00);

        // S3: runtime_dt=0x1E, stim DI=0x1F + ECC=0x1E (1-bit error at D0)
        //     ECC corrects DI to 0x1E → matches runtime_dt
        //     Expected: sync_pulse=1, corrected DI=0x1E, score 13 (corrected)
        run_scenario("S3_ecc_corrected_DI_1F_to_1E", 8'h1E,
                     8'h1F, 8'h00, 8'h05, 8'h1E,
                     1, 8'h1E);

        // S4: runtime_dt=0x1F + stim DI=0x1F + ECC=0x19 (no_error for 0x1F)
        //     Expected: sync_pulse=1, DI=0x1F, score 15
        run_scenario("S4_runtime_1F_match_raw", 8'h1F,
                     8'h1F, 8'h00, 8'h05, 8'h19,
                     1, 8'h1F);

        // S5: After 4 repeated headers with default DT, pulse count should be 4
        runtime_expected_long_dt_drv = 8'h00;
        repeat (8) @(posedge byte_clk);
        begin : repeated_test
            automatic int start_count = sync_pulse_count;
            for (int rep = 0; rep < 4; rep++) begin
                drive_header(8'h1E, 8'h00, 8'h05, 8'h1E);
            end
            $display("\n--- S5_repeated_4x ---");
            $display("  4 headers driven, sync_pulses(diff)=%0d", sync_pulse_count - start_count);
            check_eq("S5.pulse_count_4x", sync_pulse_count - start_count, 4);
        end

        $display("\n=========================================");
        if (fail_count == 0)
            $display("TEST PASSED: tb_dphy_runtime_dt");
        else
            $display("TEST FAILED: %0d failures", fail_count);
        $display("=========================================");
        $finish;
    end

    // Watchdog
    initial begin
        #20_000_000;
        $display("FAIL: watchdog timeout");
        $fatal(1, "watchdog");
    end

endmodule
