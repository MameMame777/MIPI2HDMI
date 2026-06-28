`timescale 1ns / 1ps

// 640x480 burst-mode verification of ob_row_masker.
//
// Drives realistic patterns at full hardware frame size with bursty input
// (solid 640-cycle bursts + inter-line LP-style gaps) and verifies that:
//   1. All H × W pixels arrive at the output bit-exact (no over-firing).
//   2. eol/sof/eof markers fire exactly H/1/1 times per frame.
//   3. Output line length = W consistently.
//   4. No pixels are lost or duplicated across the ping-pong boundary.
//
// All test patterns are above the OB threshold (or fail uniformity) so the
// masker MUST pass them through unchanged.

module tb_ob_row_masker_640x480;
    localparam int W = 640;
    localparam int H = 480;

    logic clk = 1'b0;
    logic aresetn = 1'b0;
    logic enable = 1'b1;

    logic [7:0] in_data;
    logic       in_valid, in_sof, in_eol, in_eof, in_err;
    logic [7:0] out_data;
    logic       out_valid, out_sof, out_eol, out_eof, out_err;

    ob_row_masker #(
        .WIDTH(8),
        .LINE_PIXELS_MAX(1024),
        .OB_THRESHOLD(8'd50),
        .OB_FILL_Y(8'd128),
        .OB_UNIFORMITY(8'd12)
    ) u_dut (
        .clk(clk), .aresetn(aresetn), .enable(enable),
        .in_data(in_data), .in_valid(in_valid),
        .in_sof(in_sof), .in_eol(in_eol),
        .in_eof(in_eof), .in_err(in_err),
        .out_data(out_data), .out_valid(out_valid),
        .out_sof(out_sof), .out_eol(out_eol),
        .out_eof(out_eof), .out_err(out_err)
    );

    always #5 clk = ~clk;

    // Full-frame storage
    bit [7:0] input_frame  [H][W];
    bit [7:0] output_frame [H][W];

    int sof_count, eol_count, eof_count;
    int out_row, out_col;
    int pixel_count;
    int per_line_count [H];   // pixels captured per output row

    always_ff @(posedge clk) begin
        if (aresetn && out_valid) begin
            if (out_row < H && out_col < W) begin
                output_frame[out_row][out_col] <= out_data;
            end
            if (out_row < H) begin
                per_line_count[out_row] <= per_line_count[out_row] + 1;
            end
            pixel_count <= pixel_count + 1;
            if (out_sof) sof_count <= sof_count + 1;
            if (out_eof) eof_count <= eof_count + 1;
            if (out_eol) begin
                eol_count <= eol_count + 1;
                out_row   <= out_row + 1;
                out_col   <= 0;
            end else begin
                out_col <= out_col + 1;
            end
        end
    end

    task automatic reset_capture();
        eol_count = 0; sof_count = 0; eof_count = 0;
        out_row = 0; out_col = 0; pixel_count = 0;
        for (int r = 0; r < H; r++) begin
            per_line_count[r] = 0;
            for (int c = 0; c < W; c++) output_frame[r][c] = 8'h00;
        end
    endtask

    task automatic drive_frame(input int interline_gap_cycles);
        for (int row = 0; row < H; row++) begin
            for (int col = 0; col < W; col++) begin
                @(posedge clk);
                in_data  <= input_frame[row][col];
                in_valid <= 1'b1;
                in_sof   <= (row == 0) && (col == 0);
                in_eol   <= (col == W - 1);
                in_eof   <= (row == H - 1) && (col == W - 1);
                in_err   <= 1'b0;
            end
            // LP-style gap between lines (matches MIPI inter-packet idle)
            for (int g = 0; g < interline_gap_cycles; g++) begin
                @(posedge clk);
                in_data  <= 8'h00;
                in_valid <= 1'b0;
                in_sof   <= 1'b0;
                in_eol   <= 1'b0;
                in_eof   <= 1'b0;
            end
        end
        @(posedge clk);
        in_valid <= 1'b0;
        in_sof   <= 1'b0;
        in_eol   <= 1'b0;
        in_eof   <= 1'b0;
    endtask

    task automatic wait_drain();
        automatic int idle = 0;
        automatic int budget = 4_000_000;
        while (idle < 4000 && budget > 0) begin
            @(posedge clk);
            if (out_valid) idle = 0;
            else            idle = idle + 1;
            budget = budget - 1;
        end
    endtask

    task automatic check_pass(input string name, output int errors);
        errors = 0;
        if (pixel_count != W * H) begin
            $display("[FAIL] %s: captured %0d pixels, expected %0d",
                     name, pixel_count, W * H);
            errors = -1;
            return;
        end
        if (sof_count != 1) begin
            $display("[FAIL] %s: sof_count=%0d expected 1", name, sof_count);
            errors = errors + 1;
        end
        if (eol_count != H) begin
            $display("[FAIL] %s: eol_count=%0d expected %0d", name, eol_count, H);
            errors = errors + 1;
        end
        if (eof_count != 1) begin
            $display("[FAIL] %s: eof_count=%0d expected 1", name, eof_count);
            errors = errors + 1;
        end
        for (int r = 0; r < H; r++) begin
            if (per_line_count[r] != W) begin
                if (errors < 5) begin
                    $display("[FAIL] %s: row %0d had %0d pixels, expected %0d",
                             name, r, per_line_count[r], W);
                end
                errors = errors + 1;
            end
        end
        for (int r = 0; r < H && errors < 10; r++) begin
            for (int c = 0; c < W && errors < 10; c++) begin
                if (output_frame[r][c] !== input_frame[r][c]) begin
                    $display("[FAIL] %s: out[%0d][%0d]=0x%02h expected 0x%02h",
                             name, r, c, output_frame[r][c], input_frame[r][c]);
                    errors = errors + 1;
                end
            end
        end
        if (errors == 0)
            $display("[PASS] %s: %0d pixels, sof=1 eol=%0d eof=1, all bit-match",
                     name, pixel_count, eol_count);
    endtask

    int total_errors = 0;
    int e;

    initial begin
        in_data = 0; in_valid = 0; in_sof = 0; in_eol = 0; in_eof = 0; in_err = 0;
        repeat (5) @(posedge clk);
        aresetn = 1;
        repeat (3) @(posedge clk);

        // ============ Test 1: Checkerboard 32×32 ============
        for (int r = 0; r < H; r++) begin
            for (int c = 0; c < W; c++) begin
                input_frame[r][c] = (((r >> 5) + (c >> 5)) & 1) ? 8'd240 : 8'd10;
            end
        end
        reset_capture();
        drive_frame(50);
        wait_drain();
        check_pass("Checkerboard 32x32 burst+gap=50", e);
        total_errors += (e > 0) ? e : (e == -1 ? 1 : 0);

        // ============ Test 2: Horizontal gradient ============
        for (int r = 0; r < H; r++) begin
            for (int c = 0; c < W; c++) begin
                input_frame[r][c] = 8'((c * 256) / W);
            end
        end
        reset_capture();
        drive_frame(0);  // solid burst, no inter-line gap
        wait_drain();
        check_pass("Horizontal gradient burst+gap=0", e);
        total_errors += (e > 0) ? e : (e == -1 ? 1 : 0);

        // ============ Test 3: Vertical stripe (column-based) ============
        for (int r = 0; r < H; r++) begin
            for (int c = 0; c < W; c++) begin
                input_frame[r][c] = ((c >> 5) & 1) ? 8'd200 : 8'd60;
            end
        end
        reset_capture();
        drive_frame(100);
        wait_drain();
        check_pass("Vertical stripe burst+gap=100", e);
        total_errors += (e > 0) ? e : (e == -1 ? 1 : 0);

        // ============ Test 4: Horizontal stripe (row-based) ============
        for (int r = 0; r < H; r++) begin
            for (int c = 0; c < W; c++) begin
                input_frame[r][c] = ((r >> 4) & 1) ? 8'd220 : 8'd70;
            end
        end
        reset_capture();
        drive_frame(20);
        wait_drain();
        check_pass("Horizontal stripe burst+gap=20", e);
        total_errors += (e > 0) ? e : (e == -1 ? 1 : 0);

        // ============ Test 5: Random ============
        for (int r = 0; r < H; r++) begin
            for (int c = 0; c < W; c++) begin
                // Above OB threshold so masker MUST pass (range > UNIFORMITY)
                input_frame[r][c] = 8'($urandom_range(60, 255));
            end
        end
        reset_capture();
        drive_frame(30);
        wait_drain();
        check_pass("Random burst+gap=30", e);
        total_errors += (e > 0) ? e : (e == -1 ? 1 : 0);

        // ============ Test 6: Two frames back-to-back ============
        // Use checkerboard again, drive twice, verify second frame too
        for (int r = 0; r < H; r++) begin
            for (int c = 0; c < W; c++) begin
                input_frame[r][c] = (((r >> 4) + (c >> 4)) & 1) ? 8'd230 : 8'd80;
            end
        end
        reset_capture();
        drive_frame(40);  // frame 1
        drive_frame(40);  // frame 2 -- but capture only sees first based on out_row<H
        // Note: capture would saturate at out_row=H for second frame.
        // Skip the bit-check for this test; instead verify counts.
        wait_drain();
        $display("[INFO] Back-to-back: pixel_count=%0d (expect 2*%0d=%0d) eol_count=%0d sof_count=%0d eof_count=%0d",
                 pixel_count, W*H, 2*W*H, eol_count, sof_count, eof_count);
        if (pixel_count != 2 * W * H) begin
            $display("[FAIL] Back-to-back: missing pixels");
            total_errors += 1;
        end else if (eol_count != 2 * H) begin
            $display("[FAIL] Back-to-back: eol_count=%0d expected %0d", eol_count, 2*H);
            total_errors += 1;
        end else if (sof_count != 2 || eof_count != 2) begin
            $display("[FAIL] Back-to-back: sof_count=%0d eof_count=%0d both should be 2",
                     sof_count, eof_count);
            total_errors += 1;
        end else begin
            $display("[PASS] Back-to-back: counts match 2-frame expectation");
        end

        // ============ Summary ============
        repeat (10) @(posedge clk);
        if (total_errors == 0)
            $display("\n==== ALL 640x480 TESTS PASSED ====");
        else
            $display("\n==== 640x480 TESTS FAILED: %0d total errors ====", total_errors);
        $finish;
    end

    initial begin
        #500_000_000;  // 500 ms hardware time
        $fatal(1, "global timeout");
    end
endmodule
