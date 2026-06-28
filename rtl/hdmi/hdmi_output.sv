`timescale 1ns / 1ps
`default_nettype none

module tmds_encoder (
    input  wire        pix_clk,
    input  wire        pix_aresetn,
    input  wire  [7:0] data_in,
    input  wire        data_enable,
    input  wire        control_0,
    input  wire        control_1,
    output logic [9:0] tmds_out
);

    logic signed [5:0] running_disparity;

    function automatic int count_ones8(input logic [7:0] value);
        count_ones8 = 0;
        for (int idx = 0; idx < 8; idx++) begin
            count_ones8 += value[idx];
        end
    endfunction

    function automatic logic [9:0] control_code(input logic c0, input logic c1);
        unique case ({c1, c0})
            2'b00: control_code = 10'b1101010100;
            2'b01: control_code = 10'b0010101011;
            2'b10: control_code = 10'b0101010100;
            default: control_code = 10'b1010101011;
        endcase
    endfunction

    always_ff @(posedge pix_clk) begin
        if (!pix_aresetn) begin
            running_disparity <= '0;
            tmds_out <= 10'b1101010100;
        end else if (!data_enable) begin
            running_disparity <= '0;
            tmds_out <= control_code(control_0, control_1);
        end else begin
            automatic logic [8:0] q_m;
            automatic logic use_xnor;
            automatic int data_ones;
            automatic int q_ones;
            automatic int q_zeros;
            automatic int disparity_next;

            data_ones = count_ones8(data_in);
            use_xnor = (data_ones > 4) || ((data_ones == 4) && !data_in[0]);
            q_m[0] = data_in[0];
            for (int idx = 1; idx < 8; idx++) begin
                q_m[idx] = use_xnor ? ~(q_m[idx - 1] ^ data_in[idx]) : (q_m[idx - 1] ^ data_in[idx]);
            end
            q_m[8] = !use_xnor;

            q_ones = count_ones8(q_m[7:0]);
            q_zeros = 8 - q_ones;
            disparity_next = running_disparity;

            if ((running_disparity == 0) || (q_ones == q_zeros)) begin
                tmds_out[9] <= ~q_m[8];
                tmds_out[8] <= q_m[8];
                tmds_out[7:0] <= q_m[8] ? q_m[7:0] : ~q_m[7:0];
                disparity_next += q_m[8] ? (q_ones - q_zeros) : (q_zeros - q_ones);
            end else if (((running_disparity > 0) && (q_ones > q_zeros)) ||
                         ((running_disparity < 0) && (q_zeros > q_ones))) begin
                tmds_out[9] <= 1'b1;
                tmds_out[8] <= q_m[8];
                tmds_out[7:0] <= ~q_m[7:0];
                disparity_next += (q_m[8] ? 2 : 0) + (q_zeros - q_ones);
            end else begin
                tmds_out[9] <= 1'b0;
                tmds_out[8] <= q_m[8];
                tmds_out[7:0] <= q_m[7:0];
                disparity_next += (q_ones - q_zeros) - (q_m[8] ? 0 : 2);
            end

            running_disparity <= disparity_next[5:0];
        end
    end

endmodule

module hdmi_output #(
    parameter int H_ACTIVE = 1280,
    parameter int H_FRONT_PORCH = 110,
    parameter int H_SYNC = 40,
    parameter int H_BACK_PORCH = 220,
    parameter int V_ACTIVE = 720,
    parameter int V_FRONT_PORCH = 5,
    parameter int V_SYNC = 5,
    parameter int V_BACK_PORCH = 20,
    parameter bit HSYNC_POLARITY = 1'b1,
    parameter bit VSYNC_POLARITY = 1'b1
) (
    input  wire         pix_clk,
    input  wire         pix_aresetn,
    input  wire         enable,
    input  wire         soft_reset,
    input  wire         test_pattern_en,
    input  wire         hpd,
    input  wire         hpd_override,

    input  wire  [23:0] s_axis_tdata,
    input  wire         s_axis_tvalid,
    output logic        s_axis_tready,
    input  wire         s_axis_tlast,
    input  wire  [0:0]  s_axis_tuser,

    output logic [7:0]  video_r,
    output logic [7:0]  video_g,
    output logic [7:0]  video_b,
    output logic        video_de,
    output logic        video_hsync,
    output logic        video_vsync,
    output logic [9:0]  tmds_data_0,
    output logic [9:0]  tmds_data_1,
    output logic [9:0]  tmds_data_2,
    output logic [9:0]  tmds_clk_word,

    output logic        sts_running,
    output logic        sts_hpd,
    output logic [31:0] sts_frame_count,
    output logic [15:0] sts_underflow_count,
    output logic [15:0] sts_axis_error_count
);

    localparam int H_SYNC_START = H_ACTIVE + H_FRONT_PORCH;
    localparam int H_SYNC_END = H_SYNC_START + H_SYNC;
    localparam int H_TOTAL = H_ACTIVE + H_FRONT_PORCH + H_SYNC + H_BACK_PORCH;
    localparam int V_SYNC_START = V_ACTIVE + V_FRONT_PORCH;
    localparam int V_SYNC_END = V_SYNC_START + V_SYNC;
    localparam int V_TOTAL = V_ACTIVE + V_FRONT_PORCH + V_SYNC + V_BACK_PORCH;
    localparam int H_COUNT_W = (H_TOTAL <= 2) ? 1 : $clog2(H_TOTAL);
    localparam int V_COUNT_W = (V_TOTAL <= 2) ? 1 : $clog2(V_TOTAL);

    logic [H_COUNT_W-1:0] h_count;
    logic [V_COUNT_W-1:0] v_count;
    logic active_region;
    logic hsync_region;
    logic vsync_region;
    logic axis_take;
    logic stream_aligned;
    logic frame_origin;
    logic last_active_pixel;

    function automatic [15:0] sat_inc16(input [15:0] value);
        sat_inc16 = (value == 16'hffff) ? value : (value + 16'd1);
    endfunction

    function automatic [23:0] test_pattern_pixel(input int x_pos, input int y_pos);
        automatic int bar;
        bar = (x_pos * 8) / H_ACTIVE;
        unique case (bar[2:0])
            3'd0: test_pattern_pixel = 24'hffffff;
            3'd1: test_pattern_pixel = 24'hffff00;
            3'd2: test_pattern_pixel = 24'h00ffff;
            3'd3: test_pattern_pixel = 24'h00ff00;
            3'd4: test_pattern_pixel = 24'hff00ff;
            3'd5: test_pattern_pixel = 24'hff0000;
            3'd6: test_pattern_pixel = 24'h0000ff;
            default: test_pattern_pixel = {y_pos[7:0], x_pos[7:0], 8'h40};
        endcase
    endfunction

    assign sts_hpd = hpd || hpd_override;
    assign active_region = (h_count < H_ACTIVE[H_COUNT_W-1:0]) && (v_count < V_ACTIVE[V_COUNT_W-1:0]);
    assign hsync_region = (h_count >= H_SYNC_START[H_COUNT_W-1:0]) && (h_count < H_SYNC_END[H_COUNT_W-1:0]);
    assign vsync_region = (v_count >= V_SYNC_START[V_COUNT_W-1:0]) && (v_count < V_SYNC_END[V_COUNT_W-1:0]);
    assign frame_origin = (h_count == '0) && (v_count == '0);
    assign last_active_pixel = (h_count == H_ACTIVE[H_COUNT_W-1:0] - 1'b1) && (v_count == V_ACTIVE[V_COUNT_W-1:0] - 1'b1);
    assign s_axis_tready = sts_running && active_region && !test_pattern_en &&
        (stream_aligned || (frame_origin && s_axis_tvalid && s_axis_tuser[0]));
    assign axis_take = s_axis_tready && s_axis_tvalid;

    always_ff @(posedge pix_clk) begin
        if (!pix_aresetn || soft_reset) begin
            h_count <= '0;
            v_count <= '0;
            video_r <= 8'h00;
            video_g <= 8'h00;
            video_b <= 8'h00;
            video_de <= 1'b0;
            video_hsync <= !HSYNC_POLARITY;
            video_vsync <= !VSYNC_POLARITY;
            sts_running <= 1'b0;
            stream_aligned <= 1'b0;
            sts_frame_count <= 32'h0000_0000;
            sts_underflow_count <= 16'h0000;
            sts_axis_error_count <= 16'h0000;
        end else begin
            sts_running <= enable && sts_hpd;
            if (!enable || !sts_hpd) begin
                stream_aligned <= 1'b0;
            end
            video_de <= sts_running && active_region;
            video_hsync <= hsync_region ? HSYNC_POLARITY : !HSYNC_POLARITY;
            video_vsync <= vsync_region ? VSYNC_POLARITY : !VSYNC_POLARITY;

            {video_r, video_g, video_b} <= 24'h000000;
            if (sts_running && active_region) begin
                if (test_pattern_en) begin
                    {video_r, video_g, video_b} <= test_pattern_pixel(h_count, v_count);
                end else if (axis_take) begin
                    {video_r, video_g, video_b} <= s_axis_tdata;
                    if ((h_count == '0) && (v_count == '0)) begin
                        if (!s_axis_tuser[0]) begin
                            sts_axis_error_count <= sat_inc16(sts_axis_error_count);
                        end
                    end else if (s_axis_tuser[0]) begin
                        sts_axis_error_count <= sat_inc16(sts_axis_error_count);
                    end

                    if ((h_count == H_ACTIVE[H_COUNT_W-1:0] - 1'b1) != s_axis_tlast) begin
                        sts_axis_error_count <= sat_inc16(sts_axis_error_count);
                    end
                    if (frame_origin && s_axis_tuser[0]) begin
                        stream_aligned <= 1'b1;
                    end
                end else begin
                    sts_underflow_count <= sat_inc16(sts_underflow_count);
                end
                // Clear unconditionally so CDC drain before last_active_pixel
                // (tvalid=0 → axis_take=0) cannot leave stream_aligned=1 into next run.
                if (last_active_pixel) begin
                    stream_aligned <= 1'b0;
                end
            end

            if (sts_running) begin
                if (h_count == H_TOTAL[H_COUNT_W-1:0] - 1'b1) begin
                    h_count <= '0;
                    if (v_count == V_TOTAL[V_COUNT_W-1:0] - 1'b1) begin
                        v_count <= '0;
                        sts_frame_count <= sts_frame_count + 32'd1;
                    end else begin
                        v_count <= v_count + 1'b1;
                    end
                end else begin
                    h_count <= h_count + 1'b1;
                end
            end else begin
                h_count <= '0;
                v_count <= '0;
            end
        end
    end

    tmds_encoder u_tmds_blue (
        .pix_clk(pix_clk),
        .pix_aresetn(pix_aresetn),
        .data_in(video_b),
        .data_enable(video_de),
        .control_0(video_hsync),
        .control_1(video_vsync),
        .tmds_out(tmds_data_0)
    );

    tmds_encoder u_tmds_green (
        .pix_clk(pix_clk),
        .pix_aresetn(pix_aresetn),
        .data_in(video_g),
        .data_enable(video_de),
        .control_0(1'b0),
        .control_1(1'b0),
        .tmds_out(tmds_data_1)
    );

    tmds_encoder u_tmds_red (
        .pix_clk(pix_clk),
        .pix_aresetn(pix_aresetn),
        .data_in(video_r),
        .data_enable(video_de),
        .control_0(1'b0),
        .control_1(1'b0),
        .tmds_out(tmds_data_2)
    );

    assign tmds_clk_word = 10'b1111100000;

endmodule

`default_nettype wire