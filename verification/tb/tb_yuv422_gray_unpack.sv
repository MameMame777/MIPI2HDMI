`timescale 1ns / 1ps
`default_nettype none

module tb_yuv422_gray_unpack;

    logic yuyv_done;
    logic yvyu_done;
    logic uyvy_done;
    logic vyuy_done;
    logic legacy_uyvy_done;
    logic repair_done;

    yuv422_gray_unpack_case #(
        .CASE_ID(1),
        .YUV422_SEQUENCE(4'h0),
        .LINE_PIXELS(2),
        .LEFT_REPAIR_PIXELS(0)
    ) u_yuyv_case (
        .done(yuyv_done)
    );

    yuv422_gray_unpack_case #(
        .CASE_ID(2),
        .YUV422_SEQUENCE(4'h1),
        .LINE_PIXELS(2),
        .LEFT_REPAIR_PIXELS(0)
    ) u_yvyu_case (
        .done(yvyu_done)
    );

    yuv422_gray_unpack_case #(
        .CASE_ID(3),
        .YUV422_SEQUENCE(4'h2),
        .LINE_PIXELS(2),
        .LEFT_REPAIR_PIXELS(0)
    ) u_uyvy_case (
        .done(uyvy_done)
    );

    yuv422_gray_unpack_case #(
        .CASE_ID(4),
        .YUV422_SEQUENCE(4'h3),
        .LINE_PIXELS(2),
        .LEFT_REPAIR_PIXELS(0)
    ) u_vyuy_case (
        .done(vyuy_done)
    );

    yuv422_gray_unpack_case #(
        .CASE_ID(5),
        .YUV422_SEQUENCE(4'hf),
        .LINE_PIXELS(2),
        .LEFT_REPAIR_PIXELS(0)
    ) u_legacy_uyvy_case (
        .done(legacy_uyvy_done)
    );

    yuv422_gray_unpack_case #(
        .CASE_ID(6),
        .YUV422_SEQUENCE(4'h2),
        .Y_AT_ODD_PHASE(1'b1),
        .LINE_PIXELS(4),
        .LEFT_REPAIR_PIXELS(2)
    ) u_repair_case (
        .done(repair_done)
    );

    initial begin
        wait (yuyv_done && yvyu_done && uyvy_done && vyuy_done && legacy_uyvy_done && repair_done);
        repeat (2) #10;
        $display("TEST PASSED: tb_yuv422_gray_unpack");
        $finish;
    end

endmodule

module yuv422_gray_unpack_case #(
    parameter int CASE_ID = 1,
    parameter logic [3:0] YUV422_SEQUENCE = 4'hf,
    parameter bit Y_AT_ODD_PHASE = (YUV422_SEQUENCE == 4'h2) || (YUV422_SEQUENCE == 4'h3) || (YUV422_SEQUENCE == 4'hf),
    parameter int LINE_PIXELS = 2,
    parameter int LEFT_REPAIR_PIXELS = 0
) (
    output logic done
);

    logic clk = 1'b0;
    logic rstn = 1'b0;
    logic in_sof;
    logic in_eof;
    logic in_eol;
    logic [7:0] in_payload_data;
    logic in_payload_valid;
    logic in_payload_first;
    logic in_payload_last;
    logic in_frame_err;
    logic [23:0] out_pixel;
    logic out_pixel_valid;
    logic out_pixel_sof;
    logic out_pixel_eol;
    logic out_pixel_eof;
    logic out_pixel_err;
    logic [15:0] sts_pixel_per_line;
    int pixel_count;
    localparam int PAYLOAD_BYTES = LINE_PIXELS * 2;

    always #5 clk = ~clk;

    yuv422_gray_unpack #(
        .YUV422_SEQUENCE(YUV422_SEQUENCE),
        .Y_AT_ODD_PHASE(Y_AT_ODD_PHASE),
        .LINE_PIXELS(LINE_PIXELS),
        .LEFT_REPAIR_PIXELS(LEFT_REPAIR_PIXELS)
    ) dut (
        .core_clk(clk),
        .core_aresetn(rstn),
        .in_sof(in_sof),
        .in_eof(in_eof),
        .in_eol(in_eol),
        .in_payload_data(in_payload_data),
        .in_payload_valid(in_payload_valid),
        .in_payload_first(in_payload_first),
        .in_payload_last(in_payload_last),
        .in_frame_err(in_frame_err),
        .out_pixel(out_pixel),
        .out_pixel_valid(out_pixel_valid),
        .out_pixel_sof(out_pixel_sof),
        .out_pixel_eol(out_pixel_eol),
        .out_pixel_eof(out_pixel_eof),
        .out_pixel_err(out_pixel_err),
        .sts_pixel_per_line(sts_pixel_per_line)
    );

    function automatic logic [7:0] payload_byte(input int idx);
        begin
            payload_byte = 8'h00;
            case (CASE_ID)
                1: begin
                    case (idx)
                        0: payload_byte = 8'h11;
                        1: payload_byte = 8'h80;
                        2: payload_byte = 8'h22;
                        3: payload_byte = 8'h10;
                        default: payload_byte = 8'h00;
                    endcase
                end
                2: begin
                    case (idx)
                        0: payload_byte = 8'h11;
                        1: payload_byte = 8'h10;
                        2: payload_byte = 8'h22;
                        3: payload_byte = 8'h80;
                        default: payload_byte = 8'h00;
                    endcase
                end
                3: begin
                    case (idx)
                        0: payload_byte = 8'h80;
                        1: payload_byte = 8'h11;
                        2: payload_byte = 8'h10;
                        3: payload_byte = 8'h22;
                        4: payload_byte = 8'h80;
                        5: payload_byte = 8'h33;
                        6: payload_byte = 8'h10;
                        7: payload_byte = 8'h44;
                        default: payload_byte = 8'h00;
                    endcase
                end
                4: begin
                    case (idx)
                        0: payload_byte = 8'h10;
                        1: payload_byte = 8'h11;
                        2: payload_byte = 8'h80;
                        3: payload_byte = 8'h22;
                        default: payload_byte = 8'h00;
                    endcase
                end
                5: begin
                    case (idx)
                        0: payload_byte = 8'h80;
                        1: payload_byte = 8'h11;
                        2: payload_byte = 8'h10;
                        3: payload_byte = 8'h22;
                        default: payload_byte = 8'h00;
                    endcase
                end
                6: begin
                    case (idx)
                        0: payload_byte = 8'h80;
                        1: payload_byte = 8'h11;
                        2: payload_byte = 8'h10;
                        3: payload_byte = 8'h22;
                        4: payload_byte = 8'h80;
                        5: payload_byte = 8'h33;
                        6: payload_byte = 8'h10;
                        7: payload_byte = 8'h44;
                        default: payload_byte = 8'h00;
                    endcase
                end
                default: payload_byte = 8'h00;
            endcase
        end
    endfunction

    function automatic logic [7:0] expected_y(input int idx);
        begin
            expected_y = 8'h00;
            case (CASE_ID)
                1, 2, 3, 4, 5: begin
                    case (idx)
                        0: expected_y = 8'h11;
                        1: expected_y = 8'h22;
                        default: expected_y = 8'h00;
                    endcase
                end
                6: begin
                    case (idx)
                        0: expected_y = 8'h00;
                        1: expected_y = 8'h00;
                        2: expected_y = 8'h33;
                        3: expected_y = 8'h44;
                        default: expected_y = 8'h00;
                    endcase
                end
                default: expected_y = 8'h00;
            endcase
        end
    endfunction

    task automatic send_payload(
        input logic [7:0] data,
        input logic first,
        input logic last,
        input logic sof
    );
        begin
            @(posedge clk);
            in_payload_data <= data;
            in_payload_first <= first;
            in_payload_last <= last;
            in_sof <= sof;
            in_payload_valid <= 1'b1;
            @(posedge clk);
            in_payload_valid <= 1'b0;
            in_payload_first <= 1'b0;
            in_payload_last <= 1'b0;
            in_sof <= 1'b0;
        end
    endtask

    always_ff @(posedge clk) begin
        if (!rstn) begin
            pixel_count <= 0;
        end else if (out_pixel_valid) begin
            automatic logic [7:0] expected;
            expected = expected_y(pixel_count);
            if (pixel_count >= LINE_PIXELS) begin
                $fatal(1, "case %0d unexpected extra pixel %0d", CASE_ID, pixel_count);
            end
            if (out_pixel !== {expected, expected, expected}) begin
                $fatal(1, "case %0d pixel %0d mismatch pixel=%06h expected=%02h", CASE_ID, pixel_count, out_pixel, expected);
            end
            if ((pixel_count == 0) && !out_pixel_sof) begin
                $fatal(1, "case %0d first pixel missing sof", CASE_ID);
            end
            if ((pixel_count != 0) && out_pixel_sof) begin
                $fatal(1, "case %0d unexpected sof on pixel %0d", CASE_ID, pixel_count);
            end
            if ((pixel_count == LINE_PIXELS - 1) != out_pixel_eol) begin
                $fatal(1, "case %0d eol mismatch pixel=%0d eol=%0b", CASE_ID, pixel_count, out_pixel_eol);
            end
            pixel_count <= pixel_count + 1;
        end
    end

    initial begin
        in_sof = 1'b0;
        in_eof = 1'b0;
        in_eol = 1'b0;
        in_payload_data = 8'h00;
        in_payload_valid = 1'b0;
        in_payload_first = 1'b0;
        in_payload_last = 1'b0;
        in_frame_err = 1'b0;
        done = 1'b0;

        repeat (4) @(posedge clk);
        rstn <= 1'b1;
        repeat (2) @(posedge clk);

        for (int idx = 0; idx < PAYLOAD_BYTES; idx++) begin
            send_payload(payload_byte(idx), idx == 0, idx == PAYLOAD_BYTES - 1, idx == 0);
        end

        repeat (4) @(posedge clk);
        if (pixel_count != LINE_PIXELS) begin
            $fatal(1, "case %0d expected %0d pixels, got %0d", CASE_ID, LINE_PIXELS, pixel_count);
        end
        if (sts_pixel_per_line != LINE_PIXELS[15:0]) begin
            $fatal(1, "case %0d expected sts_pixel_per_line=%0d got %0d", CASE_ID, LINE_PIXELS, sts_pixel_per_line);
        end
        done <= 1'b1;
    end

endmodule

`default_nettype wire