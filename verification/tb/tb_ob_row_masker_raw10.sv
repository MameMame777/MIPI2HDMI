`timescale 1ns / 1ps

// OB row masker — RAW10 (WIDTH=10) tests.
// OB-row Y values are roughly 4x higher than 8-bit (~36 → ~144 in 10-bit).
// Thresholds scale accordingly: 50→200, 128→512, 3→12.

module tb_ob_row_masker_raw10;

    localparam int           WIDTH      = 10;
    localparam logic [9:0]   TH         = 10'd200;
    localparam logic [9:0]   FILL       = 10'd512;
    localparam logic [9:0]   UNIF       = 10'd12;

    logic clk = 1'b0;
    logic aresetn = 1'b0;
    logic enable = 1'b1;

    logic [WIDTH-1:0] in_data;
    logic             in_valid, in_sof, in_eol, in_eof, in_err;
    logic [WIDTH-1:0] out_data;
    logic             out_valid, out_sof, out_eol, out_eof, out_err;

    int errors = 0;
    int test_num = 0;

    ob_row_masker #(
        .WIDTH(WIDTH),
        .OB_THRESHOLD(TH),
        .OB_FILL_Y(FILL),
        .OB_UNIFORMITY(UNIF)
    ) u_dut (
        .clk(clk), .aresetn(aresetn), .enable(enable),
        .in_data(in_data), .in_valid(in_valid), .in_sof(in_sof),
        .in_eol(in_eol), .in_eof(in_eof), .in_err(in_err),
        .out_data(out_data), .out_valid(out_valid), .out_sof(out_sof),
        .out_eol(out_eol), .out_eof(out_eof), .out_err(out_err)
    );

    always #5 clk = ~clk;

    logic [WIDTH-1:0] out_row [$];
    always_ff @(posedge clk) begin
        if (out_valid) out_row.push_back(out_data);
    end

    task automatic drive_row(input logic [WIDTH-1:0] data []);
        for (int i = 0; i < data.size(); i++) begin
            @(posedge clk);
            in_data  <= data[i];
            in_valid <= 1'b1;
            in_sof   <= 1'b0;
            in_eol   <= (i == data.size() - 1);
            in_eof   <= 1'b0;
            in_err   <= 1'b0;
        end
        @(posedge clk);
        in_valid <= 1'b0;
        in_eol   <= 1'b0;
    endtask

    task automatic wait_pipeline_flush();
        automatic int idle = 0;
        automatic int max_wait = 4096;
        while (idle < 8 && max_wait > 0) begin
            @(posedge clk);
            if (out_valid) idle = 0;
            else            idle++;
            max_wait--;
        end
    endtask

    task automatic check_all_equal(input logic [WIDTH-1:0] expected, input string name);
        test_num++;
        if (out_row.size() == 0) begin
            $display("[FAIL] test %0d (%s): no output", test_num, name);
            errors++;
            return;
        end
        for (int i = 0; i < out_row.size(); i++) begin
            if (out_row[i] !== expected) begin
                $display("[FAIL] test %0d (%s): pix %0d = 0x%03h, expected 0x%03h",
                         test_num, name, i, out_row[i], expected);
                errors++;
                return;
            end
        end
        $display("[PASS] test %0d (%s): all %0d pixels = 0x%03h",
                 test_num, name, out_row.size(), expected);
    endtask

    task automatic check_equal_array(input logic [WIDTH-1:0] expected [], input string name);
        test_num++;
        if (out_row.size() != expected.size()) begin
            $display("[FAIL] test %0d (%s): size %0d != %0d",
                     test_num, name, out_row.size(), expected.size());
            errors++;
            return;
        end
        for (int i = 0; i < out_row.size(); i++) begin
            if (out_row[i] !== expected[i]) begin
                $display("[FAIL] test %0d (%s): pix %0d = 0x%03h, expected 0x%03h",
                         test_num, name, i, out_row[i], expected[i]);
                errors++;
                return;
            end
        end
        $display("[PASS] test %0d (%s): %0d pixels match", test_num, name, out_row.size());
    endtask

    initial begin
        in_data  = '0;
        in_valid = 1'b0;
        in_sof   = 1'b0;
        in_eol   = 1'b0;
        in_eof   = 1'b0;
        in_err   = 1'b0;
        repeat (5) @(posedge clk);
        aresetn = 1'b1;
        repeat (3) @(posedge clk);

        // 1: True OB uniform Y=144 (≈ 36*4)  → MASK
        begin
            automatic logic [9:0] row [] = '{16{10'd144}};
            out_row.delete(); drive_row(row); wait_pipeline_flush();
            check_all_equal(FILL, "RAW10 OB Y=144 uniform -> mask");
        end

        // 2: OB with small variation (Y=140-148, range=8 ≤ 12) → MASK
        begin
            automatic logic [9:0] row [] = '{10'd140, 10'd148, 10'd142, 10'd146,
                                             10'd141, 10'd147, 10'd143, 10'd145,
                                             10'd144, 10'd142, 10'd148, 10'd140,
                                             10'd146, 10'd144, 10'd141, 10'd147};
            out_row.delete(); drive_row(row); wait_pipeline_flush();
            check_all_equal(FILL, "RAW10 OB range=8 -> mask");
        end

        // 3: Image row, dark first pixel only — full-line range huge → PASS
        begin
            automatic logic [9:0] row [] = '{10'd180, 10'd800, 10'd840, 10'd720,
                                             10'd600, 10'd480, 10'd360, 10'd240,
                                             10'd120, 10'd200, 10'd340, 10'd540,
                                             10'd680, 10'd820, 10'd900, 10'd1020};
            out_row.delete(); drive_row(row); wait_pipeline_flush();
            check_equal_array(row, "RAW10 image dark first pixel -> pass");
        end

        // 4: Bright uniform Y=800 → PASS (above threshold)
        begin
            automatic logic [9:0] row [] = '{16{10'd800}};
            out_row.delete(); drive_row(row); wait_pipeline_flush();
            check_equal_array(row, "RAW10 bright uniform Y=800 -> pass");
        end

        // 5: Dark non-uniform (all < 200 but range > 12) → PASS
        begin
            automatic logic [9:0] row [] = '{10'd120, 10'd20, 10'd180, 10'd8,
                                             10'd100, 10'd180, 10'd40, 10'd160,
                                             10'd140, 10'd60, 10'd190, 10'd32,
                                             10'd80, 10'd170, 10'd48, 10'd150};
            out_row.delete(); drive_row(row); wait_pipeline_flush();
            check_equal_array(row, "RAW10 dark non-uniform -> pass");
        end

        // 6: Uniform but ≥ threshold (Y=320 ± 4) → PASS
        begin
            automatic logic [9:0] row [] = '{10'd320, 10'd322, 10'd318, 10'd321,
                                             10'd319, 10'd322, 10'd320, 10'd318,
                                             10'd321, 10'd320, 10'd322, 10'd318,
                                             10'd319, 10'd321, 10'd320, 10'd322};
            out_row.delete(); drive_row(row); wait_pipeline_flush();
            check_equal_array(row, "RAW10 uniform Y=320 above thresh -> pass");
        end

        // 7: enable=0 bypass on OB → pass
        begin
            automatic logic [9:0] row [] = '{16{10'd144}};
            out_row.delete(); enable = 1'b0;
            drive_row(row); wait_pipeline_flush();
            check_equal_array(row, "RAW10 bypass enable=0 -> pass");
            enable = 1'b1;
        end

        // 8: Checkerboard 8 dark + 8 bright (Y=40 + Y=960) → PASS
        begin
            automatic logic [9:0] row [] = '{10'd40, 10'd40, 10'd40, 10'd40,
                                             10'd40, 10'd40, 10'd40, 10'd40,
                                             10'd960, 10'd960, 10'd960, 10'd960,
                                             10'd960, 10'd960, 10'd960, 10'd960};
            out_row.delete(); drive_row(row); wait_pipeline_flush();
            check_equal_array(row, "RAW10 checkerboard 8+8 -> pass");
        end

        // 9: Grayscale uniform Y=256 → PASS (above threshold)
        begin
            automatic logic [9:0] row [] = '{16{10'd256}};
            out_row.delete(); drive_row(row); wait_pipeline_flush();
            check_equal_array(row, "RAW10 grayscale Y=256 -> pass");
        end

        // 10: Gradient 0→960 (steps of 64) → PASS
        begin
            automatic logic [9:0] row [] = '{10'd0,   10'd64,  10'd128, 10'd192,
                                             10'd256, 10'd320, 10'd384, 10'd448,
                                             10'd512, 10'd576, 10'd640, 10'd704,
                                             10'd768, 10'd832, 10'd896, 10'd960};
            out_row.delete(); drive_row(row); wait_pipeline_flush();
            check_equal_array(row, "RAW10 gradient 0->960 -> pass");
        end

        // 11: Gradient with tight dark start (40,44,48,52,... then bright) → PASS
        begin
            automatic logic [9:0] row [] = '{10'd40,  10'd44,  10'd48,  10'd52,
                                             10'd200, 10'd320, 10'd440, 10'd560,
                                             10'd680, 10'd800, 10'd920, 10'd960,
                                             10'd920, 10'd800, 10'd680, 10'd560};
            out_row.delete(); drive_row(row); wait_pipeline_flush();
            check_equal_array(row, "RAW10 gradient tight dark start -> pass");
        end

        // 12: Bayer-like pattern (R/G/G/B pattern simulated as 4-pixel repeating) → PASS
        begin
            // OV5640 Bayer: typically RGRG or BGBG pattern
            // Use repeating (R=300, G=600, G=600, B=300)
            automatic logic [9:0] row [] = '{10'd300, 10'd600, 10'd600, 10'd300,
                                             10'd300, 10'd600, 10'd600, 10'd300,
                                             10'd300, 10'd600, 10'd600, 10'd300,
                                             10'd300, 10'd600, 10'd600, 10'd300};
            out_row.delete(); drive_row(row); wait_pipeline_flush();
            check_equal_array(row, "RAW10 Bayer RGGB -> pass");
        end

        // 13-17: Random patterns (5 trials, full 10-bit range)
        for (int trial = 0; trial < 5; trial++) begin
            automatic logic [9:0] row [16];
            for (int i = 0; i < 16; i++) row[i] = $urandom_range(0, 1023);
            out_row.delete(); drive_row(row); wait_pipeline_flush();
            check_equal_array(row, $sformatf("RAW10 random trial %0d -> pass", trial));
        end

        repeat (10) @(posedge clk);
        if (errors == 0)
            $display("\n==== ALL RAW10 TESTS PASSED (%0d tests) ====", test_num);
        else
            $display("\n==== RAW10 TEST FAILED: %0d errors out of %0d ====", errors, test_num);
        $finish;
    end

    initial begin
        #100000;
        $fatal(1, "RAW10 tb timeout");
    end

endmodule
