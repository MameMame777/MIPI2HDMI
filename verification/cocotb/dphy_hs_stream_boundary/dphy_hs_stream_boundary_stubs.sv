`timescale 1ns / 1ps
// Behavioral Xilinx 7-series primitive stubs for the D-PHY HS byte probe, copied
// verbatim from verification/tb/dphy_hs_byte_probe_sim_prims.sv (the exact model the
// DSim tb_dphy_hs_stream_boundary relied on). Kept local because the shared
// lib/verilator_unisim_stubs.sv trips a Verilator BADVLTPRAGMA on its banner comment
// (its "Verilator-compatible ..." line is parsed as a verilator pragma). These stubs only
// reproduce connectivity, NOT serialization/bitslip; the boundary test injects bytes at
// the post-ISERDES serdes_byte_sample register directly, so the ISERDESE2 data path is
// intentionally a pass-through that drives Q1..Q8 = 0.

module IBUFDS #(
    parameter DIFF_TERM = "FALSE",
    parameter IBUF_LOW_PWR = "TRUE",
    parameter IOSTANDARD = "DEFAULT"
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
    parameter BUFR_DIVIDE = "BYPASS",
    parameter SIM_DEVICE = "7SERIES"
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
    parameter CINVCTRL_SEL = "FALSE",
    parameter DELAY_SRC = "IDATAIN",
    parameter HIGH_PERFORMANCE_MODE = "TRUE",
    parameter IDELAY_TYPE = "FIXED",
    parameter IDELAY_VALUE = 0,
    parameter PIPE_SEL = "FALSE",
    parameter REFCLK_FREQUENCY = 200.0,
    parameter SIGNAL_PATTERN = "DATA"
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
    parameter DATA_RATE = "DDR",
    parameter DATA_WIDTH = 8,
    parameter DYN_CLKDIV_INV_EN = "FALSE",
    parameter DYN_CLK_INV_EN = "FALSE",
    parameter INTERFACE_TYPE = "NETWORKING",
    parameter IOBDELAY = "IFD",
    parameter NUM_CE = 1,
    parameter OFB_USED = "FALSE",
    parameter SERDES_MODE = "MASTER"
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

// -----------------------------------------------------------------------------
// Harness top for the cocotb port of tb_dphy_hs_stream_boundary.
//
// Replicates the three DUT instances of tb_dphy_hs_stream_boundary.sv (the
// dphy_hs_byte_probe, csi2_packet_parser, csi2_header_ecc) and their exact
// inter-wiring, exposing every top-level tb signal (clocks, resets, D-PHY
// stimulus inputs) plus every output the DSim checks read. The probe instance is
// named u_dut so the cocotb test can reach the internal serdes_byte_sample
// injection register and the stream_pairing_active/next FSM state via Verilator
// --public-flat-rw hierarchy access, matching the DSim tb's dut.serdes_byte_sample
// deposits and dut.stream_pairing_* reads 1:1.
// -----------------------------------------------------------------------------
module dphy_hs_stream_boundary_harness (
    input  wire        rst_n,
    input  wire        parser_aresetn,
    input  wire        idelay_ref_clk,
    input  wire        hs_clk_p,
    input  wire        hs_clk_n,
    input  wire [1:0]  data_hs_p,
    input  wire [1:0]  data_hs_n,
    input  wire [1:0]  data_lp_p,
    input  wire [1:0]  data_lp_n,
    output wire        byte_clk,

    // probe stream + scanner observables
    output wire [15:0] stream_byte_data,
    output wire [1:0]  stream_byte_keep,
    output wire        stream_byte_valid,
    output wire        stream_byte_sop,
    output wire        stream_byte_eop,
    output wire        sync_header_valid,
    output wire [7:0]  sync_header_di,
    output wire [15:0] sync_header_wc,
    output wire [7:0]  sync_header_ecc,
    output wire [2:0]  sync_header_bit_offset_lane0,
    output wire [2:0]  sync_header_bit_offset_lane1,
    output wire [2:0]  sync_header_pairing,
    output wire [3:0]  sync_header_score,
    output wire        sync_header_ecc_no_error,
    output wire [7:0]  trace_slot_valid,
    output wire [7:0][7:0] trace_slot_lane0_candidate,
    output wire [7:0][7:0] trace_slot_lane1_candidate,
    output wire [7:0]  live_trace_slot_valid,
    output wire [7:0][7:0] live_trace_slot_lane0_aligned,
    output wire [7:0][7:0] live_trace_slot_lane1_aligned,
    output wire [7:0]  live_trace_slot_sot_hit_lane0,
    output wire [7:0]  live_trace_slot_sot_hit_lane1,
    output wire [2:0]  stream_pairing_active_dbg,
    output wire [2:0]  stream_pairing_next_dbg,

    // ecc header decoder observables
    output wire        ecc_hdr_valid,
    output wire [31:0] ecc_hdr_raw,
    output wire        ecc_hdr_corr_valid,
    output wire [23:0] ecc_hdr_corr,
    output wire [7:0]  ecc_hdr_di,
    output wire [15:0] ecc_hdr_wc,
    output wire        ecc_hdr_corrected,
    output wire        ecc_hdr_uncorrectable,
    output wire        ecc_hdr_no_error,
    output wire [15:0] ecc_corr_count,
    output wire [15:0] ecc_uncorr_count,

    // parser observables
    output wire        parser_pkt_hdr_valid,
    output wire [31:0] parser_pkt_hdr_raw,
    output wire [7:0]  parser_pkt_di,
    output wire [15:0] parser_pkt_wc,
    output wire        parser_pkt_is_long,
    output wire        parser_pkt_is_short,
    output wire        parser_pkt_ecc_uncorrectable,
    output wire [7:0]  parser_payload_data,
    output wire        parser_payload_valid,
    output wire        parser_payload_first,
    output wire        parser_payload_last,
    output wire [15:0] parser_footer_data,
    output wire        parser_footer_valid,
    output wire        parser_pkt_done,
    output wire [15:0] parser_short_count,
    output wire [15:0] parser_long_count,
    output wire [15:0] parser_trunc_count
);

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
    ) u_dut (
        .rst_n(rst_n),
        .idelay_ref_clk(idelay_ref_clk),
        .idelay_ref_reset(!rst_n),
        .runtime_idelay_tap(5'd0),
        .runtime_idelay_tap_lane1(5'd0),
        .runtime_bitslip_phase(3'd0),
        .runtime_bitslip_phase_lane1(3'd0),
        .runtime_lane1_sweep_enable(1'b0),
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
        .stream_pairing_active_dbg(stream_pairing_active_dbg),
        .stream_pairing_next_dbg(stream_pairing_next_dbg),
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
        .live_trace_seq(),
        .live_trace_slot_valid(live_trace_slot_valid),
        .live_trace_slot_lane0_raw(),
        .live_trace_slot_lane1_raw(),
        .live_trace_slot_lane0_candidate(),
        .live_trace_slot_lane1_candidate(),
        .live_trace_slot_lane0_aligned(live_trace_slot_lane0_aligned),
        .live_trace_slot_lane1_aligned(live_trace_slot_lane1_aligned),
        .live_trace_slot_sot_hit_lane0(live_trace_slot_sot_hit_lane0),
        .live_trace_slot_sot_hit_lane1(live_trace_slot_sot_hit_lane1),
        .live_trace_slot_lane0_rotation(),
        .live_trace_slot_lane1_rotation(),
        .lane1_target_phase_out(),
        .dbg_burst_count(),
        .dbg_sot_burst_count(),
        .dbg_missed_burst(),
        .dbg_relock_latency(),
        .dbg_relock_max()
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
endmodule
