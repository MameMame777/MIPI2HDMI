`timescale 1ns / 1ps
`default_nettype none
// cocotb harness for the port of verification/tb/tb_dphy_probe_supervised.sv.
//
// Instantiates ONLY the two DUTs (dphy_hs_byte_probe + dphy_lane_supervisor) with
// EXACTLY the same parameters and wiring as the DSim TB -- no `initial`, no clocks,
// no stimulus. cocotb owns the ctl_clk / hs_clk_p clocks, rst_n, the LP-line
// stimulus, and the post-ISERDES byte injection.
//
// Byte injection: the DSim TB forced dut.serdes_byte_sample[lane] directly. Here the
// probe is instanced as u_probe, so cocotb injects the post-ISERDES sample by writing
// the internal register u_probe.serdes_byte_sample[lane] over the VPI (the same
// per-cycle override the DSim force performed: written on negedge byte_clk, consumed
// combinationally on the following posedge, then reverted by the RTL's
// `serdes_byte_sample <= serdes_byte` NBA -- serdes_byte is 0 from the ISERDES stub).
//
// The sim primitives (IBUFDS/BUFIO/BUFR/IDELAYCTRL/IDELAYE2/ISERDESE2) come from
// verification/tb/dphy_hs_byte_probe_sim_prims.sv (listed in the .f). The BUFR stub
// there is a BYPASS passthrough, so byte_clk == hs_clk (100 MHz) whenever CLR is low.
module dphy_probe_supervised_harness (
    // clocks / reset (cocotb-driven)
    input  wire        ctl_clk,       // 200 MHz supervisor ctl_clk / IDELAY refclk
    input  wire        hs_clk_p,      // 100 MHz HS clock (byte_clk passthrough via BUFR stub)
    input  wire        hs_clk_n,
    input  wire        rst_n,

    // LP-line stimulus
    input  wire [1:0]  data_lp_p,
    input  wire [1:0]  data_lp_n,
    input  wire [1:0]  clk_lp,        // {LP_p, LP_n} clock lane (into supervisor)

    // probe control
    input  wire        sup_enable,
    input  wire        cfg_hs_settle_gate,
    input  wire [3:0]  cfg_settle_blank_k,

    // observable probe outputs
    output wire        byte_clk,
    output wire [1:0]  lane_sot_seen,
    output wire        sync_header_valid,
    output wire [7:0]  sync_header_di,
    output wire [15:0] sync_header_wc,
    output wire [3:0]  sync_header_score,
    output wire [15:0] dbg_burst_count,
    output wire [15:0] dbg_sot_burst_count,
    output wire [31:0] dbg_missed_burst,

    // supervisor <-> probe nets (observable)
    output wire        sup_bufr_clr,
    output wire        sup_serdes_rst,
    output wire        sup_rx_clk_active,
    output wire        sup_hs_settled,
    output wire [2:0]  sup_clk_state,
    output wire [2:0]  sup_data_state,
    output wire [7:0]  sup_lock_cnt,
    output wire [7:0]  sup_settle_cnt
);
    // data HS pins are unused by the byte-injection flow (the ISERDES stub deserialises
    // nothing); tie them off exactly as the TB did (p=00, n=11).
    wire [1:0] data_hs_p = 2'b00;
    wire [1:0] data_hs_n = 2'b11;

    // ------------------------------------------------------------------
    // DUT: probe (sup_* driven by supervisor). Ports/params 1:1 with the TB.
    // ------------------------------------------------------------------
    dphy_hs_byte_probe #(
        .LANES(2),
        .SOT_WINDOW_BYTES(32),
        .SWEEP_HOLD_BYTES(8),
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
    ) u_probe (
        .rst_n(rst_n),
        .idelay_ref_clk(ctl_clk),
        .idelay_ref_reset(!rst_n),
        .runtime_idelay_tap(5'd0),
        .runtime_idelay_tap_lane1(5'd0),
        .runtime_bitslip_phase(3'd0),
        .runtime_bitslip_phase_lane1(3'd0),
        .runtime_lane1_sweep_enable(1'b0),
        .runtime_expected_long_dt(8'h00),
        .sup_enable(sup_enable),
        .sup_bufr_clr(sup_bufr_clr),
        .sup_serdes_rst(sup_serdes_rst),
        .sup_hs_settled(sup_hs_settled),
        .cfg_hs_settle_gate(cfg_hs_settle_gate),
        .dphy_hs_clock_clk_p(hs_clk_p),
        .dphy_hs_clock_clk_n(hs_clk_n),
        .dphy_data_hs_p(data_hs_p),
        .dphy_data_hs_n(data_hs_n),
        .dphy_data_lp_p(data_lp_p),
        .dphy_data_lp_n(data_lp_n),
        .byte_clk(byte_clk),
        .lane_sot_seen(lane_sot_seen),
        .sync_header_valid(sync_header_valid),
        .sync_header_di(sync_header_di),
        .sync_header_wc(sync_header_wc),
        .sync_header_score(sync_header_score),
        .cfg_settle_blank_k(cfg_settle_blank_k),
        .dbg_burst_count(dbg_burst_count),
        .dbg_sot_burst_count(dbg_sot_burst_count),
        .dbg_missed_burst(dbg_missed_burst)
    );

    // ------------------------------------------------------------------
    // Supervisor: small T_INIT for sim speed (matches the TB).
    // ------------------------------------------------------------------
    dphy_lane_supervisor #(
        .CTL_CLK_HZ(200_000_000),
        .T_INIT_US(1),
        .T_INIT_FORCE_US(3)
    ) u_sup (
        .ctl_clk           (ctl_clk),
        .ctl_aresetn       (rst_n),
        .clk_lp            (clk_lp),
        .data_lp           ({data_lp_p[0], data_lp_n[0]}),
        .byte_clk          (byte_clk),
        .bufr_clr          (sup_bufr_clr),
        .rx_clk_active_byte(sup_rx_clk_active),
        .serdes_rst_byte   (sup_serdes_rst),
        .hs_settled_byte   (sup_hs_settled),
        .sts_clk_state     (sup_clk_state),
        .sts_data_state    (sup_data_state),
        .sts_lock_cnt      (sup_lock_cnt),
        .sts_settle_cnt    (sup_settle_cnt)
    );

endmodule
`default_nettype wire
