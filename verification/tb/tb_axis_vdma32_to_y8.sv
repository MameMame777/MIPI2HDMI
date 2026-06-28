`timescale 1ns / 1ps
`default_nettype none

module tb_axis_vdma32_to_y8;

    logic clk = 1'b0;
    logic rstn = 1'b0;
    logic [31:0] s_tdata;
    logic s_tvalid;
    logic s_tready;
    logic s_tlast;
    logic [0:0] s_tuser;
    logic [7:0] m_tdata;
    logic m_tvalid;
    logic m_tready;
    logic m_tlast;
    logic [0:0] m_tuser;

    always #5 clk = ~clk;

    axis_vdma32_to_y8 dut (
        .aclk(clk),
        .aresetn(rstn),
        .s_axis_tdata(s_tdata),
        .s_axis_tvalid(s_tvalid),
        .s_axis_tready(s_tready),
        .s_axis_tlast(s_tlast),
        .s_axis_tuser(s_tuser),
        .m_axis_tdata(m_tdata),
        .m_axis_tvalid(m_tvalid),
        .m_axis_tready(m_tready),
        .m_axis_tlast(m_tlast),
        .m_axis_tuser(m_tuser)
    );

    task automatic send_beat(input logic [31:0] data, input logic last, input logic user);
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
            s_tvalid <= 1'b0;
            s_tlast <= 1'b0;
            s_tuser <= 1'b0;
        end
    endtask

    task automatic expect_byte(input logic [7:0] data, input logic last, input logic user);
        begin
            @(posedge clk);
            while (!m_tvalid) begin
                @(posedge clk);
            end
            if (m_tdata !== data) begin
                $fatal(1, "TDATA expected %02x got %02x", data, m_tdata);
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
        s_tdata = 32'h0000_0000;
        s_tvalid = 1'b0;
        s_tlast = 1'b0;
        s_tuser = 1'b0;
        m_tready = 1'b0;

        repeat (4) @(posedge clk);
        rstn <= 1'b1;
        repeat (2) @(posedge clk);

        send_beat(32'h4433_2211, 1'b0, 1'b1);
        @(posedge clk);
        if (s_tready) begin
            $fatal(1, "Input was ready while the first beat was pending");
        end
        expect_byte(8'h11, 1'b0, 1'b1);
        expect_byte(8'h22, 1'b0, 1'b0);
        expect_byte(8'h33, 1'b0, 1'b0);
        expect_byte(8'h44, 1'b0, 1'b0);

        send_beat(32'h8877_6655, 1'b1, 1'b0);
        expect_byte(8'h55, 1'b0, 1'b0);
        expect_byte(8'h66, 1'b0, 1'b0);
        expect_byte(8'h77, 1'b0, 1'b0);
        expect_byte(8'h88, 1'b1, 1'b0);

        $display("TEST PASSED: tb_axis_vdma32_to_y8");
        $finish;
    end

endmodule

`default_nettype wire