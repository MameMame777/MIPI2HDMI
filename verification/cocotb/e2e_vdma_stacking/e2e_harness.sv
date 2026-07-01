
`timescale 1ns / 1ps
`default_nettype none
// Auto-generated E2E wrapper for the cocotb port of tb_e2e_vdma_stacking.sv.
// Contains ONLY the four DUT instances (no initial / no clock / no VDMA model) so cocotb
// owns clk/rst, stimulus, and the behavioural VDMA write models. Wiring + parameters are
// 1:1 with the DSim TB. NORMALIZE_EN=1, br_tready tied high (VDMA always ready).
module e2e_harness #(
    parameter int LINE_PIXELS = 8,
    parameter int LINE_BYTES  = 16,
    parameter int FRAME_LINES = 11,
    parameter int VSIZE       = 44
)(
    input  wire        core_clk,
    input  wire        core_aresetn,
    input  wire        aclk,
    input  wire        aresetn,
    input  wire        cfg_use_lsle,

    // frame_state packet inputs
    input  wire [7:0]  in_pkt_di,
    input  wire [15:0] in_pkt_wc,
    input  wire        in_pkt_is_short,
    input  wire        in_pkt_is_long,
    input  wire        in_pkt_start,
    input  wire        in_pkt_end,
    input  wire        in_pkt_err,
    input  wire [7:0]  in_payload_data,
    input  wire        in_payload_valid,
    input  wire        in_payload_first,
    input  wire        in_payload_last,

    // unpack-out markers (for the SOF/EOF counters)
    output wire        up_v,
    output wire        up_sof,
    output wire        up_eof,
    // normalizer-out markers
    output wire        nm_v,
    output wire        nm_sof,
    output wire        nm_eof,

    // bridge AXIS output (aclk side) -> VDMA models
    output wire [7:0]  br_tdata,
    output wire        br_tvalid,
    output wire        br_tlast,
    output wire        br_tuser,
    input  wire        br_tready,

    // frame_state status
    output wire [31:0] fs_fcnt,
    output wire [15:0] fs_lastlines,
    output wire [15:0] fs_syncerr
);
    // frame_state I/O
    wire        fs_sof, fs_eof, fs_sol, fs_eol, fs_in_frame;
    wire [15:0] fs_line_idx;
    wire [7:0]  fs_pd; wire fs_pv, fs_pf, fs_pl, fs_ferr;
    wire [31:0] fs_lcnt;
    wire [15:0] fs_dbg_la, fs_dbg_nols, fs_dbg_idle;
    wire [127:0] fs_dbg_hist;

    csi2_frame_state #(
        .MAX_LINES(64), .GUARD_FRAME_LINES(1'b1), .EXPECTED_FRAME_LINES(FRAME_LINES),
        .EXPECTED_LINE_WC(16'(LINE_BYTES)), .FS_MIN_LINES(4), .FE_DELIMITS(1'b1)
    ) u_fs (
        .core_clk(core_clk), .core_aresetn(core_aresetn), .cfg_use_lsle(cfg_use_lsle),
        .cfg_expected_frame_lines(16'd0),
        .cfg_sof_synth(1'b0), .cfg_force_expected(1'b0), .cfg_long_as_line(1'b0),
        .in_pkt_di(in_pkt_di), .in_pkt_wc(in_pkt_wc), .in_pkt_is_short(in_pkt_is_short),
        .in_pkt_is_long(in_pkt_is_long), .in_pkt_start(in_pkt_start), .in_pkt_end(in_pkt_end),
        .in_pkt_err(in_pkt_err), .in_payload_data(in_payload_data),
        .in_payload_valid(in_payload_valid),
        .in_payload_first(in_payload_first), .in_payload_last(in_payload_last),
        .out_sof(fs_sof), .out_eof(fs_eof), .out_sol(fs_sol), .out_eol(fs_eol),
        .out_in_frame(fs_in_frame),
        .out_line_idx(fs_line_idx), .out_payload_data(fs_pd), .out_payload_valid(fs_pv),
        .out_payload_first(fs_pf), .out_payload_last(fs_pl), .out_frame_err(fs_ferr),
        .sts_frame_count(fs_fcnt), .sts_line_count(fs_lcnt),
        .sts_last_frame_lines(fs_lastlines), .sts_frame_sync_err_cnt(fs_syncerr),
        .sts_dbg_long_accept(fs_dbg_la), .sts_dbg_long_nols(fs_dbg_nols),
        .sts_dbg_long_idle(fs_dbg_idle), .sts_dbg_nols_hist(fs_dbg_hist)
    );

    // unpack I/O
    wire [23:0] up_pixel; wire up_eol, up_err; wire [15:0] up_ppl;
    yuv422_gray_unpack #(.LINE_PIXELS(0)) u_up (
        .core_clk(core_clk), .core_aresetn(core_aresetn),
        .in_sof(fs_sof), .in_eof(fs_eof), .in_eol(fs_eol),
        .in_payload_data(fs_pd), .in_payload_valid(fs_pv),
        .in_payload_first(fs_pf), .in_payload_last(fs_pl), .in_frame_err(fs_ferr),
        .out_pixel(up_pixel), .out_pixel_valid(up_v), .out_pixel_sof(up_sof),
        .out_pixel_eol(up_eol), .out_pixel_eof(up_eof), .out_pixel_err(up_err),
        .sts_pixel_per_line(up_ppl)
    );

    // frame normalizer: pin every frame to exactly VSIZE x LINE_PIXELS
    wire [7:0] nm_data; wire nm_err;
    video_frame_normalizer #(.OUT_LINES(VSIZE), .OUT_PIXELS(LINE_PIXELS),
                             .FILL(8'h00), .NORMALIZE(1'b1)) u_norm (
        .clk(core_clk), .aresetn(core_aresetn),
        .in_data(up_pixel[7:0]), .in_valid(up_v), .in_sof(up_sof),
        .in_eol(up_eol), .in_eof(up_eof), .in_err(up_err),
        .out_data(nm_data), .out_valid(nm_v), .out_sof(nm_sof),
        .out_eol(nm_eol), .out_eof(nm_eof), .out_err(nm_err)
    );
    wire nm_eol;

    // bridge (8-bit data, 1-bit tuser=SOF)
    wire [15:0] br_ovf, br_bp;
    wire [0:0]  br_tuser_w;
    axis_video_bridge #(.TDATA_WIDTH(8), .TUSER_WIDTH(1), .FIFO_DEPTH(4096),
                        .AXIS_TUSER_ERR_DEBUG(1'b0)) u_br (
        .core_clk(core_clk), .core_aresetn(core_aresetn), .aclk(aclk), .aresetn(aresetn),
        .in_pixel(nm_data), .in_pixel_valid(nm_v), .in_pixel_sof(nm_sof),
        .in_pixel_eol(nm_eol), .in_pixel_eof(nm_eof), .in_pixel_err(nm_err),
        .m_axis_tdata(br_tdata), .m_axis_tvalid(br_tvalid), .m_axis_tready(br_tready),
        .m_axis_tlast(br_tlast), .m_axis_tuser(br_tuser_w),
        .sts_fifo_overflow_cnt(br_ovf), .sts_back_pressure_cnt(br_bp)
    );
    assign br_tuser = br_tuser_w[0];
endmodule
`default_nettype wire
