`timescale 1ns / 1ps
`default_nettype none

module axis_vdma32_to_y8 (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF S_AXIS:M_AXIS, ASSOCIATED_RESET aresetn" *)
    input  wire         aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire         aresetn,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TDATA" *)
    input  wire [31:0]  s_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TVALID" *)
    input  wire         s_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TREADY" *)
    output wire         s_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TLAST" *)
    input  wire         s_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TUSER" *)
    input  wire [0:0]   s_axis_tuser,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TDATA" *)
    output logic [7:0]  m_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TVALID" *)
    output wire         m_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TREADY" *)
    input  wire         m_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TLAST" *)
    output wire         m_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TUSER" *)
    output wire [0:0]   m_axis_tuser
);

    logic [31:0] beat_data;
    logic [1:0]  byte_index;
    logic        beat_valid;
    logic        beat_last;
    logic        beat_user;

    assign s_axis_tready = !beat_valid;
    assign m_axis_tvalid = beat_valid;
    assign m_axis_tlast = beat_last && (byte_index == 2'd3);
    assign m_axis_tuser[0] = beat_user && (byte_index == 2'd0);

    always_comb begin
        unique case (byte_index)
            2'd0: m_axis_tdata = beat_data[7:0];
            2'd1: m_axis_tdata = beat_data[15:8];
            2'd2: m_axis_tdata = beat_data[23:16];
            default: m_axis_tdata = beat_data[31:24];
        endcase
    end

    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            beat_data <= 32'h0000_0000;
            byte_index <= 2'd0;
            beat_valid <= 1'b0;
            beat_last <= 1'b0;
            beat_user <= 1'b0;
        end else begin
            if (beat_valid && m_axis_tready) begin
                if (byte_index == 2'd3) begin
                    beat_valid <= 1'b0;
                    byte_index <= 2'd0;
                    beat_last <= 1'b0;
                    beat_user <= 1'b0;
                end else begin
                    byte_index <= byte_index + 2'd1;
                end
            end

            if (!beat_valid && s_axis_tvalid) begin
                beat_data <= s_axis_tdata;
                byte_index <= 2'd0;
                beat_valid <= 1'b1;
                beat_last <= s_axis_tlast;
                beat_user <= s_axis_tuser[0];
            end
        end
    end

endmodule

`default_nettype wire