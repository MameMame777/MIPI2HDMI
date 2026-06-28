`timescale 1ns / 1ps
`default_nettype none

module tb_rgb565_gray_unpack;

    logic little_done;
    logic big_done;

    rgb565_gray_unpack_case #(
        .CASE_ID(1),
        .RGB565_BIG_ENDIAN(1'b0),
        .LINE_PIXELS(4)
    ) u_little_case (
        .done(little_done)
    );

    rgb565_gray_unpack_case #(
        .CASE_ID(2),
        .RGB565_BIG_ENDIAN(1'b1),
        .LINE_PIXELS(4)
    ) u_big_case (
        .done(big_done)
    );

    initial begin
        wait (little_done && big_done);
        repeat (2) #10;
        $display("TEST PASSED: tb_rgb565_gray_unpack");
        $finish;
    end

endmodule

module rgb565_gray_unpack_case #(
    parameter int CASE_ID = 1,
    parameter bit RGB565_BIG_ENDIAN = 1'b0,
    parameter int LINE_PIXELS = 4
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

    rgb565_gray_unpack #(
        .RGB565_BIG_ENDIAN(RGB565_BIG_ENDIAN),
        .LINE_PIXELS(LINE_PIXELS)
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
            if (!RGB565_BIG_ENDIAN) begin
                case (idx)
                    0: payload_byte = 8'h00;
                    1: payload_byte = 8'hf8;
                    2: payload_byte = 8'he0;
                    3: payload_byte = 8'h07;
                    4: payload_byte = 8'h1f;
                    5: payload_byte = 8'h00;
                    6: payload_byte = 8'hff;
                    7: payload_byte = 8'hff;
                    default: payload_byte = 8'h00;
                endcase
            end else begin
                case (idx)
                    0: payload_byte = 8'hf8;
                    1: payload_byte = 8'h00;
                    2: payload_byte = 8'h07;
                    3: payload_byte = 8'he0;
                    4: payload_byte = 8'h00;
                    5: payload_byte = 8'h1f;
                    6: payload_byte = 8'hff;
                    7: payload_byte = 8'hff;
                    default: payload_byte = 8'h00;
                endcase
            end
        end
    endfunction

    function automatic logic [7:0] expected_gray(input int idx);
        begin
            expected_gray = 8'h00;
            case (idx)
                0: expected_gray = 8'h4c;
                1: expected_gray = 8'h95;
                2: expected_gray = 8'h1c;
                3: expected_gray = 8'hff;
                default: expected_gray = 8'h00;
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
            expected = expected_gray(pixel_count);
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
