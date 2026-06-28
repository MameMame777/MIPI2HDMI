`timescale 1ns / 1ps
`default_nettype none

// axis_vdma32_to_rgb24 (2026-06-23, color path / image-processing research base)
// Unpack the VDMA MM2S 32-bit RGBA32 stream (1 pixel per word, {A, R, G, B}) into a
// 24-bit RGB888 AXI4-Stream for the HDMI side (s_axis_hdmi is 24-bit). This is the
// colour counterpart of axis_vdma32_to_y8 + the old gray->RGB replicate (sub_y_to_rgb):
// it simply drops the alpha/top byte, 1 word -> 1 RGB pixel, so the existing 24-bit
// HDMI bridge consumes real colour. TUSER[0] = SOF is carried through 1:1.
module axis_vdma32_to_rgb24 (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF S_AXIS:M_AXIS, ASSOCIATED_RESET aresetn" *)
    input  wire        aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        aresetn,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TDATA" *)
    input  wire [31:0] s_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TVALID" *)
    input  wire        s_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TREADY" *)
    output wire        s_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TLAST" *)
    input  wire        s_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TUSER" *)
    input  wire [0:0]  s_axis_tuser,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TDATA" *)
    output logic [23:0] m_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TVALID" *)
    output logic        m_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TREADY" *)
    input  wire         m_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TLAST" *)
    output logic        m_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TUSER" *)
    output logic [0:0]  m_axis_tuser
);

    wire s_axis_fire = s_axis_tvalid && s_axis_tready;

    assign s_axis_tready = !m_axis_tvalid || m_axis_tready;

    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            m_axis_tdata  <= 24'h000000;
            m_axis_tvalid <= 1'b0;
            m_axis_tlast  <= 1'b0;
            m_axis_tuser  <= 1'b0;
        end else begin
            if (m_axis_tvalid && m_axis_tready) begin
                m_axis_tvalid <= 1'b0;
                m_axis_tlast  <= 1'b0;
                m_axis_tuser  <= 1'b0;
            end
            if (s_axis_fire) begin
                m_axis_tdata  <= s_axis_tdata[23:0];   // drop alpha/top byte -> {R, G, B}
                m_axis_tvalid <= 1'b1;
                m_axis_tlast  <= s_axis_tlast;
                m_axis_tuser  <= s_axis_tuser[0];
            end
        end
    end

endmodule

`default_nettype wire
