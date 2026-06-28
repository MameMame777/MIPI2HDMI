`timescale 1ns / 1ps
`default_nettype none

module tmds_serializer_10b (
    input  wire       tmds_clk,
    input  wire       pix_clk,
    input  wire       reset,
    input  wire [9:0] tmds_word,
    output wire       tmds_serial
);

    wire shift_1;
    wire shift_2;

    OSERDESE2 #(
        .DATA_RATE_OQ("DDR"),
        .DATA_RATE_TQ("SDR"),
        .DATA_WIDTH(10),
        .INIT_OQ(1'b0),
        .INIT_TQ(1'b0),
        .SERDES_MODE("MASTER"),
        .SRVAL_OQ(1'b0),
        .SRVAL_TQ(1'b0),
        .TBYTE_CTL("FALSE"),
        .TBYTE_SRC("FALSE"),
        .TRISTATE_WIDTH(1)
    ) u_oserdes_master (
        .OQ(tmds_serial),
        .OFB(),
        .TQ(),
        .TFB(),
        .SHIFTOUT1(),
        .SHIFTOUT2(),
        .CLK(tmds_clk),
        .CLKDIV(pix_clk),
        .D1(tmds_word[0]),
        .D2(tmds_word[1]),
        .D3(tmds_word[2]),
        .D4(tmds_word[3]),
        .D5(tmds_word[4]),
        .D6(tmds_word[5]),
        .D7(tmds_word[6]),
        .D8(tmds_word[7]),
        .OCE(1'b1),
        .RST(reset),
        .SHIFTIN1(shift_1),
        .SHIFTIN2(shift_2),
        .T1(1'b0),
        .T2(1'b0),
        .T3(1'b0),
        .T4(1'b0),
        .TBYTEIN(1'b0),
        .TCE(1'b0)
    );

    OSERDESE2 #(
        .DATA_RATE_OQ("DDR"),
        .DATA_RATE_TQ("SDR"),
        .DATA_WIDTH(10),
        .INIT_OQ(1'b0),
        .INIT_TQ(1'b0),
        .SERDES_MODE("SLAVE"),
        .SRVAL_OQ(1'b0),
        .SRVAL_TQ(1'b0),
        .TBYTE_CTL("FALSE"),
        .TBYTE_SRC("FALSE"),
        .TRISTATE_WIDTH(1)
    ) u_oserdes_slave (
        .OQ(),
        .OFB(),
        .TQ(),
        .TFB(),
        .SHIFTOUT1(shift_1),
        .SHIFTOUT2(shift_2),
        .CLK(tmds_clk),
        .CLKDIV(pix_clk),
        .D1(1'b0),
        .D2(1'b0),
        .D3(tmds_word[8]),
        .D4(tmds_word[9]),
        .D5(1'b0),
        .D6(1'b0),
        .D7(1'b0),
        .D8(1'b0),
        .OCE(1'b1),
        .RST(reset),
        .SHIFTIN1(1'b0),
        .SHIFTIN2(1'b0),
        .T1(1'b0),
        .T2(1'b0),
        .T3(1'b0),
        .T4(1'b0),
        .TBYTEIN(1'b0),
        .TCE(1'b0)
    );

endmodule

module hdmi_tpg_top (
    input  wire       sysclk,
    input  wire       hdmi_tx_hpd,
    output wire       hdmi_tx_clk_p,
    output wire       hdmi_tx_clk_n,
    output wire [2:0] hdmi_tx_p,
    output wire [2:0] hdmi_tx_n,
    output wire       hdmi_tx_scl,
    inout  wire       hdmi_tx_sda,
    output wire       hdmi_tx_cec,
    output logic [3:0] led
);

    wire clk_feedback;
    wire clk_feedback_buf;
    wire tmds_clk_unbuf;
    wire pix_clk_unbuf;
    wire tmds_clk;
    wire pix_clk;
    wire mmcm_locked;

    logic [7:0] reset_shift = 8'h00;
    logic pix_aresetn;
    logic [23:0] axis_tdata;
    logic axis_tvalid;
    logic axis_tready;
    logic axis_tlast;
    logic [0:0] axis_tuser;
    logic [7:0] video_r;
    logic [7:0] video_g;
    logic [7:0] video_b;
    logic video_de;
    logic video_hsync;
    logic video_vsync;
    logic [9:0] tmds_data_0;
    logic [9:0] tmds_data_1;
    logic [9:0] tmds_data_2;
    logic [9:0] tmds_clk_word;
    logic sts_running;
    logic sts_hpd;
    logic [31:0] sts_frame_count;
    logic [15:0] sts_underflow_count;
    logic [15:0] sts_axis_error_count;
    wire [3:0] tmds_serial;

    MMCME2_BASE #(
        .BANDWIDTH("OPTIMIZED"),
        .CLKFBOUT_MULT_F(8.0),
        .CLKFBOUT_PHASE(0.0),
        .CLKIN1_PERIOD(8.0),
        .CLKOUT0_DIVIDE_F(8.0),
        .CLKOUT0_DUTY_CYCLE(0.5),
        .CLKOUT0_PHASE(0.0),
        .CLKOUT1_DIVIDE(40),
        .CLKOUT1_DUTY_CYCLE(0.5),
        .CLKOUT1_PHASE(0.0),
        .DIVCLK_DIVIDE(1),
        .STARTUP_WAIT("FALSE")
    ) u_mmcm (
        .CLKIN1(sysclk),
        .CLKFBIN(clk_feedback_buf),
        .CLKFBOUT(clk_feedback),
        .CLKFBOUTB(),
        .CLKOUT0(tmds_clk_unbuf),
        .CLKOUT0B(),
        .CLKOUT1(pix_clk_unbuf),
        .CLKOUT1B(),
        .CLKOUT2(),
        .CLKOUT2B(),
        .CLKOUT3(),
        .CLKOUT3B(),
        .CLKOUT4(),
        .CLKOUT5(),
        .CLKOUT6(),
        .LOCKED(mmcm_locked),
        .PWRDWN(1'b0),
        .RST(1'b0)
    );

    BUFG u_clkfb_bufg (
        .I(clk_feedback),
        .O(clk_feedback_buf)
    );

    BUFG u_tmds_clk_bufg (
        .I(tmds_clk_unbuf),
        .O(tmds_clk)
    );

    BUFG u_pix_clk_bufg (
        .I(pix_clk_unbuf),
        .O(pix_clk)
    );

    always_ff @(posedge pix_clk) begin
        reset_shift <= {reset_shift[6:0], mmcm_locked};
        pix_aresetn <= &reset_shift;
    end

    hdmi_output #(
        .H_ACTIVE(640),
        .H_FRONT_PORCH(16),
        .H_SYNC(96),
        .H_BACK_PORCH(48),
        .V_ACTIVE(480),
        .V_FRONT_PORCH(10),
        .V_SYNC(2),
        .V_BACK_PORCH(33),
        .HSYNC_POLARITY(1'b0),
        .VSYNC_POLARITY(1'b0)
    ) u_hdmi_output (
        .pix_clk(pix_clk),
        .pix_aresetn(pix_aresetn),
        .enable(1'b1),
        .soft_reset(1'b0),
        .test_pattern_en(1'b1),
        .hpd(hdmi_tx_hpd),
        .hpd_override(1'b1),
        .s_axis_tdata(axis_tdata),
        .s_axis_tvalid(axis_tvalid),
        .s_axis_tready(axis_tready),
        .s_axis_tlast(axis_tlast),
        .s_axis_tuser(axis_tuser),
        .video_r(video_r),
        .video_g(video_g),
        .video_b(video_b),
        .video_de(video_de),
        .video_hsync(video_hsync),
        .video_vsync(video_vsync),
        .tmds_data_0(tmds_data_0),
        .tmds_data_1(tmds_data_1),
        .tmds_data_2(tmds_data_2),
        .tmds_clk_word(tmds_clk_word),
        .sts_running(sts_running),
        .sts_hpd(sts_hpd),
        .sts_frame_count(sts_frame_count),
        .sts_underflow_count(sts_underflow_count),
        .sts_axis_error_count(sts_axis_error_count)
    );

    assign axis_tdata = 24'h000000;
    assign axis_tvalid = 1'b0;
    assign axis_tlast = 1'b0;
    assign axis_tuser = 1'b0;

    tmds_serializer_10b u_serialize_blue (
        .tmds_clk(tmds_clk),
        .pix_clk(pix_clk),
        .reset(!pix_aresetn),
        .tmds_word(tmds_data_0),
        .tmds_serial(tmds_serial[0])
    );

    tmds_serializer_10b u_serialize_green (
        .tmds_clk(tmds_clk),
        .pix_clk(pix_clk),
        .reset(!pix_aresetn),
        .tmds_word(tmds_data_1),
        .tmds_serial(tmds_serial[1])
    );

    tmds_serializer_10b u_serialize_red (
        .tmds_clk(tmds_clk),
        .pix_clk(pix_clk),
        .reset(!pix_aresetn),
        .tmds_word(tmds_data_2),
        .tmds_serial(tmds_serial[2])
    );

    tmds_serializer_10b u_serialize_clock (
        .tmds_clk(tmds_clk),
        .pix_clk(pix_clk),
        .reset(!pix_aresetn),
        .tmds_word(tmds_clk_word),
        .tmds_serial(tmds_serial[3])
    );

    OBUFDS u_tmds_clk_obufds (
        .I(tmds_serial[3]),
        .O(hdmi_tx_clk_p),
        .OB(hdmi_tx_clk_n)
    );

    for (genvar lane = 0; lane < 3; lane++) begin : gen_tmds_data_obufds
        OBUFDS u_tmds_data_obufds (
            .I(tmds_serial[lane]),
            .O(hdmi_tx_p[lane]),
            .OB(hdmi_tx_n[lane])
        );
    end

    assign hdmi_tx_scl = 1'b1;
    assign hdmi_tx_sda = 1'bz;
    assign hdmi_tx_cec = 1'b0;

    always_ff @(posedge pix_clk) begin
        led[0] <= mmcm_locked;
        led[1] <= sts_hpd;
        led[2] <= sts_running;
        led[3] <= sts_frame_count[4];
    end

endmodule

`default_nettype wire