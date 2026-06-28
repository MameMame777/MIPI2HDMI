`timescale 1ns / 1ps

module tb_hdmi_output;
    localparam int H_ACTIVE = 8;
    localparam int H_FRONT_PORCH = 2;
    localparam int H_SYNC = 2;
    localparam int H_BACK_PORCH = 2;
    localparam int V_ACTIVE = 4;
    localparam int V_FRONT_PORCH = 1;
    localparam int V_SYNC = 1;
    localparam int V_BACK_PORCH = 1;
    localparam int H_TOTAL = H_ACTIVE + H_FRONT_PORCH + H_SYNC + H_BACK_PORCH;
    localparam int V_TOTAL = V_ACTIVE + V_FRONT_PORCH + V_SYNC + V_BACK_PORCH;

    logic pix_clk;
    logic pix_aresetn;
    logic enable;
    logic soft_reset;
    logic test_pattern_en;
    logic hpd;
    logic hpd_override;
    logic [23:0] s_axis_tdata;
    logic s_axis_tvalid;
    logic s_axis_tready;
    logic s_axis_tlast;
    logic [0:0] s_axis_tuser;
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

    int active_count;
    int axis_pixel_index;
    logic axis_drive_done;
    logic prev_video_de;
    logic prev_video_hsync;
    logic prev_video_vsync;

    hdmi_output #(
        .H_ACTIVE(H_ACTIVE),
        .H_FRONT_PORCH(H_FRONT_PORCH),
        .H_SYNC(H_SYNC),
        .H_BACK_PORCH(H_BACK_PORCH),
        .V_ACTIVE(V_ACTIVE),
        .V_FRONT_PORCH(V_FRONT_PORCH),
        .V_SYNC(V_SYNC),
        .V_BACK_PORCH(V_BACK_PORCH),
        .HSYNC_POLARITY(1'b1),
        .VSYNC_POLARITY(1'b1)
    ) dut (
        .pix_clk(pix_clk),
        .pix_aresetn(pix_aresetn),
        .enable(enable),
        .soft_reset(soft_reset),
        .test_pattern_en(test_pattern_en),
        .hpd(hpd),
        .hpd_override(hpd_override),
        .s_axis_tdata(s_axis_tdata),
        .s_axis_tvalid(s_axis_tvalid),
        .s_axis_tready(s_axis_tready),
        .s_axis_tlast(s_axis_tlast),
        .s_axis_tuser(s_axis_tuser),
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

    initial begin
        pix_clk = 1'b0;
        forever #5 pix_clk = ~pix_clk;
    end

    function automatic logic [9:0] control_code(input logic c0, input logic c1);
        unique case ({c1, c0})
            2'b00: control_code = 10'b1101010100;
            2'b01: control_code = 10'b0010101011;
            2'b10: control_code = 10'b0101010100;
            default: control_code = 10'b1010101011;
        endcase
    endfunction

    function automatic logic [23:0] color_bar(input int x_pos, input int y_pos);
        unique case (x_pos[2:0])
            3'd0: color_bar = 24'hffffff;
            3'd1: color_bar = 24'hffff00;
            3'd2: color_bar = 24'h00ffff;
            3'd3: color_bar = 24'h00ff00;
            3'd4: color_bar = 24'hff00ff;
            3'd5: color_bar = 24'hff0000;
            3'd6: color_bar = 24'h0000ff;
            default: color_bar = {y_pos[7:0], x_pos[7:0], 8'h40};
        endcase
    endfunction

    function automatic logic [23:0] axis_pixel(input int index);
        axis_pixel = {8'(index + 1), 8'(8'h80 + index), 8'(8'h40 + index)};
    endfunction

    task automatic check_condition(input bit condition, input string message);
        if (!condition) begin
            $fatal(1, "CHECK FAILED: %s", message);
        end
    endtask

    task automatic reset_dut();
        pix_aresetn = 1'b0;
        enable = 1'b0;
        soft_reset = 1'b0;
        test_pattern_en = 1'b0;
        hpd = 1'b1;
        hpd_override = 1'b0;
        s_axis_tdata = 24'h000000;
        s_axis_tvalid = 1'b0;
        s_axis_tlast = 1'b0;
        s_axis_tuser = 1'b0;
        active_count = 0;
        axis_pixel_index = 0;
        axis_drive_done = 1'b0;
        prev_video_de = 1'b0;
        prev_video_hsync = 1'b0;
        prev_video_vsync = 1'b0;
        repeat (8) @(posedge pix_clk);
        pix_aresetn = 1'b1;
        repeat (2) @(posedge pix_clk);
    endtask

    task automatic wait_frame_count(input int frame_count);
        for (int cycle = 0; cycle < 2000; cycle++) begin
            @(posedge pix_clk);
            #1;
            if (sts_frame_count >= frame_count) begin
                return;
            end
        end
        $fatal(1, "Timed out waiting for HDMI frame count %0d", frame_count);
    endtask

    task automatic run_tpg_check();
        automatic int expected_x;
        reset_dut();
        test_pattern_en = 1'b1;
        enable = 1'b1;
        expected_x = 0;
        for (int cycle = 0; cycle < H_TOTAL * V_TOTAL * 2; cycle++) begin
            @(posedge pix_clk);
            #1;
            if (!prev_video_de) begin
                check_condition(tmds_data_0 == control_code(prev_video_hsync, prev_video_vsync), "blue channel control code during blanking");
                check_condition(tmds_data_1 == control_code(1'b0, 1'b0), "green channel control code during blanking");
                check_condition(tmds_data_2 == control_code(1'b0, 1'b0), "red channel control code during blanking");
            end
            if (video_de) begin
                automatic logic [23:0] expected_rgb;
                expected_rgb = color_bar(expected_x % H_ACTIVE, (expected_x / H_ACTIVE) % V_ACTIVE);
                if ({video_r, video_g, video_b} != expected_rgb) begin
                    $fatal(1, "CHECK FAILED: TPG RGB pixel idx=%0d got=%06h expected=%06h", expected_x, {video_r, video_g, video_b}, expected_rgb);
                end
                expected_x++;
            end
            prev_video_de = video_de;
            prev_video_hsync = video_hsync;
            prev_video_vsync = video_vsync;
        end
        check_condition(expected_x >= H_ACTIVE * V_ACTIVE, "TPG active pixel count");
        check_condition(tmds_clk_word == 10'b1111100000, "TMDS clock pattern");
    endtask

    task automatic drive_axis_frame();
        axis_pixel_index = 0;
        while (axis_pixel_index < H_ACTIVE * V_ACTIVE) begin
            @(negedge pix_clk);
            s_axis_tvalid = 1'b1;
            s_axis_tdata = axis_pixel(axis_pixel_index);
            s_axis_tuser[0] = (axis_pixel_index == 0);
            s_axis_tlast = ((axis_pixel_index % H_ACTIVE) == (H_ACTIVE - 1));

            @(posedge pix_clk);
            if (s_axis_tvalid && s_axis_tready) begin
                axis_pixel_index++;
            end
        end
        @(negedge pix_clk);
        s_axis_tvalid = 1'b0;
        s_axis_tuser[0] = 1'b0;
        s_axis_tlast = 1'b0;
        axis_drive_done = 1'b1;
    endtask

    task automatic run_axis_check();
        reset_dut();
        test_pattern_en = 1'b0;
        enable = 1'b1;
        fork
            drive_axis_frame();
            begin
                automatic int checked_pixels;
                checked_pixels = 0;
                while (checked_pixels < H_ACTIVE * V_ACTIVE) begin
                    @(posedge pix_clk);
                    #1;
                    if (video_de) begin
                        if ({video_r, video_g, video_b} != axis_pixel(checked_pixels)) begin
                            $fatal(1, "CHECK FAILED: AXIS RGB pixel idx=%0d got=%06h expected=%06h", checked_pixels, {video_r, video_g, video_b}, axis_pixel(checked_pixels));
                        end
                        checked_pixels++;
                    end
                end
            end
        join
        wait_frame_count(1);
        check_condition(sts_axis_error_count == 16'h0000, "AXIS sideband check clean");
        check_condition(sts_underflow_count == 16'h0000, "AXIS underflow clean");
    endtask

    task automatic run_underflow_check();
        reset_dut();
        test_pattern_en = 1'b0;
        enable = 1'b1;
        s_axis_tvalid = 1'b0;
        for (int cycle = 0; cycle < H_TOTAL * 2; cycle++) begin
            @(posedge pix_clk);
            #1;
        end
        check_condition(sts_underflow_count != 16'h0000, "underflow counter increments");
    endtask

    initial begin
        run_tpg_check();
        run_axis_check();
        run_underflow_check();
        $display("TEST PASSED: tb_hdmi_output");
        $finish;
    end

    initial begin
        #5ms;
        $fatal(1, "Simulation timeout");
    end
endmodule