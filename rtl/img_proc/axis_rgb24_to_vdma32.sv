`timescale 1ns / 1ps
`default_nettype none

// axis_rgb24_to_vdma32 (2026-06-23, color path / image-processing research base)
// Pack a 24-bit RGB888 AXI4-Stream (1 pixel per beat) into the VDMA's 32-bit data
// path as RGBA32: one pixel per 32-bit word, {8'h00, R[23:16], G[15:8], B[7:0]}.
// This is the colour counterpart of axis_y8_to_vdma32 (which packs 4x Y8/word); the
// 1px=1word mapping keeps the VDMA stride/HSIZE trivial (HSIZE = WIDTH*4) and lets
// the MM2S->HDMI side drop the top byte (axis_vdma32_to_rgb24) with no re-pack.
// TUSER[0] = SOF (start of frame) is carried straight through (1:1 beats).
module axis_rgb24_to_vdma32 (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF S_AXIS:M_AXIS, ASSOCIATED_RESET aresetn" *)
    input  wire        aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        aresetn,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TDATA" *)
    input  wire [23:0] s_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TVALID" *)
    input  wire        s_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TREADY" *)
    output wire        s_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TLAST" *)
    input  wire        s_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TUSER" *)
    input  wire [0:0]  s_axis_tuser,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TDATA" *)
    output logic [31:0] m_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TKEEP" *)
    output logic [3:0]  m_axis_tkeep,
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

    // 1:1 skid-free passthrough: accept whenever the held beat is free or draining.
    assign s_axis_tready = !m_axis_tvalid || m_axis_tready;

    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            m_axis_tdata  <= 32'h0000_0000;
            m_axis_tkeep  <= 4'h0;
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
                m_axis_tdata  <= {8'h00, s_axis_tdata};   // {A=0, R, G, B}
                m_axis_tkeep  <= 4'b1111;
                m_axis_tvalid <= 1'b1;
                m_axis_tlast  <= s_axis_tlast;
                m_axis_tuser  <= s_axis_tuser[0];
            end
        end
    end

endmodule

`default_nettype wire
