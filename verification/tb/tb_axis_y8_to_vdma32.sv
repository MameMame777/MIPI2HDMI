`timescale 1ns / 1ps
`default_nettype none

module tb_axis_y8_to_vdma32;

    logic clk = 1'b0;
    logic rstn = 1'b0;
    logic [7:0] s_tdata;
    logic s_tvalid;
    logic s_tready;
    logic s_tlast;
    logic [0:0] s_tuser;
    logic [31:0] m_tdata;
    logic [3:0] m_tkeep;
    logic m_tvalid;
    logic m_tready;
    logic m_tlast;
    logic [0:0] m_tuser;

    always #5 clk = ~clk;

    axis_y8_to_vdma32 dut (
        .aclk(clk),
        .aresetn(rstn),
        .s_axis_tdata(s_tdata),
        .s_axis_tvalid(s_tvalid),
        .s_axis_tready(s_tready),
        .s_axis_tlast(s_tlast),
        .s_axis_tuser(s_tuser),
        .m_axis_tdata(m_tdata),
        .m_axis_tkeep(m_tkeep),
        .m_axis_tvalid(m_tvalid),
        .m_axis_tready(m_tready),
        .m_axis_tlast(m_tlast),
        .m_axis_tuser(m_tuser)
    );

    task automatic send_byte(input logic [7:0] data, input logic last, input logic user);
        begin
            @(posedge clk);
            while (!s_tready) begin
                @(posedge clk);
            end
            s_tdata <= data;
            s_tlast <= last;
            s_tuser <= user;
            s_tvalid <= 1'b1;
            @(posedge clk);
            while (!s_tready) begin
                @(posedge clk);
            end
            s_tvalid <= 1'b0;
            s_tlast <= 1'b0;
            s_tuser <= 1'b0;
        end
    endtask

    task automatic expect_beat(
        input logic [31:0] data,
        input logic [3:0] keep,
        input logic last,
        input logic user
    );
        begin
            @(posedge clk);
            while (!m_tvalid) begin
                @(posedge clk);
            end
            if (m_tdata !== data) begin
                $fatal(1, "TDATA expected %08x got %08x", data, m_tdata);
            end
            if (m_tkeep !== keep) begin
                $fatal(1, "TKEEP expected %x got %x", keep, m_tkeep);
            end
            if (m_tlast !== last) begin
                $fatal(1, "TLAST expected %0b got %0b", last, m_tlast);
            end
            if (m_tuser[0] !== user) begin
                $fatal(1, "TUSER expected %0b got %0b", user, m_tuser[0]);
            end
            m_tready <= 1'b1;
            @(posedge clk);
            m_tready <= 1'b0;
        end
    endtask

    initial begin
        s_tdata = 8'h00;
        s_tvalid = 1'b0;
        s_tlast = 1'b0;
        s_tuser = 1'b0;
        m_tready = 1'b0;

        repeat (4) @(posedge clk);
        rstn <= 1'b1;
        repeat (2) @(posedge clk);

        send_byte(8'h11, 1'b0, 1'b1);
        send_byte(8'h22, 1'b0, 1'b0);
        send_byte(8'h33, 1'b0, 1'b0);
        send_byte(8'h44, 1'b1, 1'b0);
        expect_beat(32'h4433_2211, 4'hf, 1'b1, 1'b1);

        send_byte(8'haa, 1'b0, 1'b0);
        send_byte(8'hbb, 1'b1, 1'b0);
        expect_beat(32'h0000_bbaa, 4'h3, 1'b1, 1'b0);

        send_byte(8'h01, 1'b0, 1'b0);
        send_byte(8'h02, 1'b0, 1'b0);
        send_byte(8'h03, 1'b0, 1'b0);
        send_byte(8'h04, 1'b0, 1'b0);
        @(posedge clk);
        if (!m_tvalid || s_tready) begin
            $fatal(1, "Backpressure did not hold pending output");
        end
        expect_beat(32'h0403_0201, 4'hf, 1'b0, 1'b0);

        $display("TEST PASSED: tb_axis_y8_to_vdma32");
        $finish;
    end

endmodule

`default_nettype wire