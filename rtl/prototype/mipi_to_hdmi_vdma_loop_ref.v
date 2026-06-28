`timescale 1ns / 1ps
`default_nettype none

module mipi_to_hdmi_vdma_loop_ref #(
    parameter integer PROBE_IDELAY_TAP = 8,
    parameter integer STREAM_PAIRING = 0,
    parameter [7:0] OV5640_MIPI_CTRL_4800 = 8'h24,
    parameter [7:0] OV5640_FORMAT_CTRL_4300 = 8'h30,
    parameter [7:0] OV5640_ISP_FORMAT_501F = 8'h01,  // RGB565 ISP mux (2026-06-19 zero-PYNQ); NOT in the core0 BD CONFIG -> this wrapper default controls it. mainline ref: RGB=1 (was 0x00 YUV)
    parameter [7:0] OV5640_ISP_CTRL_5000 = 8'ha7,
    parameter [7:0] OV5640_ISP_CTRL_5001 = 8'h83,
    parameter OV5640_TEST_PATTERN_ENABLE = 1'b0,
    parameter CAPTURE_RAW_PAYLOAD = 1'b0,
    parameter USE_RGB565_GRAY = 1'b0,
    parameter PROBE_LANE1_BITSLIP_SWEEP = 1'b0,
    parameter integer IMAGE_FORMAT = 1
) (
    (* X_INTERFACE_PARAMETER = "FREQ_HZ 125000000" *)
    input  wire        sysclk,
    output wire [3:0]  led,

    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF m_axis_capture, ASSOCIATED_RESET capture_aresetn, FREQ_HZ 100000000" *)
    input  wire        capture_aclk,
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        capture_aresetn,

    // m_axis_capture is 24-bit RGB888 (color path, 2026-06-23). Hardcoded (not a
    // param-ternary) because the BD module-reference interface inference does NOT
    // evaluate ternary param expressions in a port range -- it fell back to 8-bit
    // and only the lower 8 bits were wired (confirmed via bd_color_recreate.tcl).
    // The probe is instantiated with COLOR_CAPTURE=1 below. For the legacy Y8 build,
    // revert this wrapper (git) -- it is the BD's core0 reference, color-only now.
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis_capture TDATA" *)
    output wire [23:0]  m_axis_capture_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis_capture TVALID" *)
    output wire        m_axis_capture_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis_capture TREADY" *)
    input  wire        m_axis_capture_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis_capture TLAST" *)
    output wire        m_axis_capture_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis_capture TUSER" *)
    output wire [0:0]  m_axis_capture_tuser,
    output wire [31:0] capture_debug,
    input  wire [7:0]  debug_page_sel,
    input  wire [31:0] sccb_rt_write_word_in,
    output wire [31:0] sccb_rt_write_status_out,
    input  wire [31:0] idelay_runtime_word_in,
    output wire [31:0] idelay_runtime_status_out,
    input  wire [31:0] bitslip_runtime_word_in,
    output wire [31:0] bitslip_runtime_status_out,
    input  wire [31:0] frame_lines_runtime_word_in,
    output wire [31:0] frame_lines_runtime_status_out,
    input  wire [31:0] rawcap_word_in,
    output wire [31:0] rawcap_status_out,

    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 pix_clk_out CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axis_hdmi, ASSOCIATED_RESET pix_aresetn_out, FREQ_HZ 25000000" *)
    output wire        pix_clk_out,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 pix_aresetn_out RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    output wire        pix_aresetn_out,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis_hdmi TDATA" *)
    input  wire [23:0] s_axis_hdmi_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis_hdmi TVALID" *)
    input  wire        s_axis_hdmi_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis_hdmi TREADY" *)
    output wire        s_axis_hdmi_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis_hdmi TLAST" *)
    input  wire        s_axis_hdmi_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis_hdmi TUSER" *)
    input  wire [0:0]  s_axis_hdmi_tuser,

    input  wire        dphy_hs_clock_clk_p,
    input  wire        dphy_hs_clock_clk_n,
    input  wire [1:0]  dphy_data_hs_p,
    input  wire [1:0]  dphy_data_hs_n,
    input  wire        dphy_clk_lp_p,
    input  wire        dphy_clk_lp_n,
    input  wire [1:0]  dphy_data_lp_p,
    input  wire [1:0]  dphy_data_lp_n,

    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 cam_clk CLK" *)
    (* X_INTERFACE_PARAMETER = "FREQ_HZ 12500000" *)
    output wire        cam_clk,
    output wire        cam_gpio,
    inout  wire        cam_scl,
    inout  wire        cam_sda,

    input  wire        hdmi_tx_hpd,
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 hdmi_tx_clk_p CLK" *)
    (* X_INTERFACE_PARAMETER = "FREQ_HZ 125000000" *)
    output wire        hdmi_tx_clk_p,
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 hdmi_tx_clk_n CLK" *)
    (* X_INTERFACE_PARAMETER = "FREQ_HZ 125000000" *)
    output wire        hdmi_tx_clk_n,
    output wire [2:0]  hdmi_tx_p,
    output wire [2:0]  hdmi_tx_n,
    output wire        hdmi_tx_scl,
    inout  wire        hdmi_tx_sda,
    output wire        hdmi_tx_cec
);

    mipi_to_hdmi_probe_top #(
        .PROBE_IDELAY_TAP(PROBE_IDELAY_TAP),
        .PROBE_LANE1_BITSLIP_SWEEP(PROBE_LANE1_BITSLIP_SWEEP),
        .STREAM_PAIRING(STREAM_PAIRING),
        .OV5640_MIPI_CTRL_4800(OV5640_MIPI_CTRL_4800),
        .OV5640_FORMAT_CTRL_4300(OV5640_FORMAT_CTRL_4300),
        .OV5640_ISP_FORMAT_501F(OV5640_ISP_FORMAT_501F),
        .OV5640_ISP_CTRL_5000(OV5640_ISP_CTRL_5000),
        .OV5640_ISP_CTRL_5001(OV5640_ISP_CTRL_5001),
        .OV5640_TEST_PATTERN_ENABLE(OV5640_TEST_PATTERN_ENABLE),
        .CAPTURE_RAW_PAYLOAD(CAPTURE_RAW_PAYLOAD),
        .COLOR_CAPTURE(1'b1),
        .USE_RGB565_GRAY(USE_RGB565_GRAY),
        .IMAGE_FORMAT(IMAGE_FORMAT)
    ) u_mipi_to_hdmi_probe_top (
        .sysclk(sysclk),
        .led(led),
        .capture_aclk(capture_aclk),
        .capture_aresetn(capture_aresetn),
        .m_axis_capture_tdata(m_axis_capture_tdata),
        .m_axis_capture_tvalid(m_axis_capture_tvalid),
        .m_axis_capture_tready(m_axis_capture_tready),
        .m_axis_capture_tlast(m_axis_capture_tlast),
        .m_axis_capture_tuser(m_axis_capture_tuser),
        .capture_debug(capture_debug),
        .debug_page_sel(debug_page_sel),
        .sccb_rt_write_word_in(sccb_rt_write_word_in),
        .sccb_rt_write_status_out(sccb_rt_write_status_out),
        .idelay_runtime_word_in(idelay_runtime_word_in),
        .idelay_runtime_status_out(idelay_runtime_status_out),
        .bitslip_runtime_word_in(bitslip_runtime_word_in),
        .bitslip_runtime_status_out(bitslip_runtime_status_out),
        .frame_lines_runtime_word_in(frame_lines_runtime_word_in),
        .frame_lines_runtime_status_out(frame_lines_runtime_status_out),
        .rawcap_word_in(rawcap_word_in),
        .rawcap_status_out(rawcap_status_out),
        .pix_clk_out(pix_clk_out),
        .pix_aresetn_out(pix_aresetn_out),
        .s_axis_hdmi_tdata(s_axis_hdmi_tdata),
        .s_axis_hdmi_tvalid(s_axis_hdmi_tvalid),
        .s_axis_hdmi_tready(s_axis_hdmi_tready),
        .s_axis_hdmi_tlast(s_axis_hdmi_tlast),
        .s_axis_hdmi_tuser(s_axis_hdmi_tuser),
        .dphy_hs_clock_clk_p(dphy_hs_clock_clk_p),
        .dphy_hs_clock_clk_n(dphy_hs_clock_clk_n),
        .dphy_data_hs_p(dphy_data_hs_p),
        .dphy_data_hs_n(dphy_data_hs_n),
        .dphy_clk_lp_p(dphy_clk_lp_p),
        .dphy_clk_lp_n(dphy_clk_lp_n),
        .dphy_data_lp_p(dphy_data_lp_p),
        .dphy_data_lp_n(dphy_data_lp_n),
        .cam_clk(cam_clk),
        .cam_gpio(cam_gpio),
        .cam_scl(cam_scl),
        .cam_sda(cam_sda),
        .hdmi_tx_hpd(hdmi_tx_hpd),
        .hdmi_tx_clk_p(hdmi_tx_clk_p),
        .hdmi_tx_clk_n(hdmi_tx_clk_n),
        .hdmi_tx_p(hdmi_tx_p),
        .hdmi_tx_n(hdmi_tx_n),
        .hdmi_tx_scl(hdmi_tx_scl),
        .hdmi_tx_sda(hdmi_tx_sda),
        .hdmi_tx_cec(hdmi_tx_cec)
    );

endmodule

`default_nettype wire
