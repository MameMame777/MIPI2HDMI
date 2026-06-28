`timescale 1ns / 1ps
`default_nettype none

module axis_y8_to_vdma32 (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF S_AXIS:M_AXIS, ASSOCIATED_RESET aresetn" *)
    input  wire        aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        aresetn,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TDATA" *)
    input  wire [7:0]  s_axis_tdata,
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

    logic [31:0] pack_data;
    logic [3:0]  pack_keep;
    logic [1:0]  pack_count;
    logic        pack_tuser;

    wire s_axis_fire = s_axis_tvalid && s_axis_tready;

    assign s_axis_tready = !m_axis_tvalid || m_axis_tready;

    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            pack_data     <= 32'h0000_0000;
            pack_keep     <= 4'h0;
            pack_count    <= 2'd0;
            pack_tuser    <= 1'b0;
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
                automatic logic [31:0] next_data;
                automatic logic [3:0] next_keep;
                automatic logic next_tuser;

                next_data = pack_data;
                next_keep = pack_keep;
                unique case (pack_count)
                    2'd0: begin
                        next_data = {24'h000000, s_axis_tdata};
                        next_keep = 4'b0001;
                    end
                    2'd1: begin
                        next_data = {16'h0000, s_axis_tdata, pack_data[7:0]};
                        next_keep = 4'b0011;
                    end
                    2'd2: begin
                        next_data = {8'h00, s_axis_tdata, pack_data[15:0]};
                        next_keep = 4'b0111;
                    end
                    default: begin
                        next_data = {s_axis_tdata, pack_data[23:0]};
                        next_keep = 4'b1111;
                    end
                endcase

                next_tuser = pack_tuser | s_axis_tuser[0];

                if ((pack_count == 2'd3) || s_axis_tlast) begin
                    m_axis_tdata  <= next_data;
                    m_axis_tkeep  <= next_keep;
                    m_axis_tvalid <= 1'b1;
                    m_axis_tlast  <= s_axis_tlast;
                    m_axis_tuser  <= next_tuser;
                    pack_data     <= 32'h0000_0000;
                    pack_keep     <= 4'h0;
                    pack_count    <= 2'd0;
                    pack_tuser    <= 1'b0;
                end else begin
                    pack_data  <= next_data;
                    pack_keep  <= next_keep;
                    pack_count <= pack_count + 2'd1;
                    pack_tuser <= next_tuser;
                end
            end
        end
    end

endmodule

`default_nettype wire