`timescale 1ns / 1ps

module tb_ob_row_masker;

    logic clk = 1'b0;
    logic aresetn = 1'b0;
    logic enable = 1'b1;

    logic [7:0] in_data;
    logic       in_valid, in_sof, in_eol, in_eof, in_err;
    logic [7:0] out_data;
    logic       out_valid, out_sof, out_eol, out_eof, out_err;

    int         errors = 0;
    int         test_num = 0;

    ob_row_masker u_dut (
        .clk(clk), .aresetn(aresetn), .enable(enable),
        .in_data(in_data), .in_valid(in_valid), .in_sof(in_sof),
        .in_eol(in_eol), .in_eof(in_eof), .in_err(in_err),
        .out_data(out_data), .out_valid(out_valid), .out_sof(out_sof),
        .out_eol(out_eol), .out_eof(out_eof), .out_err(out_err)
    );

    always #5 clk = ~clk;

    // Capture output bytes per line (drained on out_eol)
    logic [7:0] out_row [$];
    always_ff @(posedge clk) begin
        if (out_valid) out_row.push_back(out_data);
    end

    task automatic drive_row(input logic [7:0] data [], input bit is_sof, input bit is_eof);
        for (int i = 0; i < data.size(); i++) begin
            @(posedge clk);
            in_data  <= data[i];
            in_valid <= 1'b1;
            in_sof   <= (i == 0) && is_sof;
            in_eol   <= (i == data.size() - 1);
            in_eof   <= (i == data.size() - 1) && is_eof;
            in_err   <= 1'b0;
        end
        @(posedge clk);
        in_data  <= 8'h00;
        in_valid <= 1'b0;
        in_sof   <= 1'b0;
        in_eol   <= 1'b0;
        in_eof   <= 1'b0;
    endtask

    task automatic wait_pipeline_flush();
        // New masker has up to 1-line latency; poll until output idle for several cycles
        automatic int idle = 0;
        automatic int max_wait = 4096;
        while (idle < 8 && max_wait > 0) begin
            @(posedge clk);
            if (out_valid) idle = 0;
            else            idle++;
            max_wait--;
        end
    endtask

    task automatic check_all_equal(input logic [7:0] expected, input string name);
        test_num++;
        if (out_row.size() == 0) begin
            $display("[FAIL] test %0d (%s): no output captured", test_num, name);
            errors++;
            return;
        end
        for (int i = 0; i < out_row.size(); i++) begin
            if (out_row[i] !== expected) begin
                $display("[FAIL] test %0d (%s): pixel %0d = 0x%02x, expected 0x%02x",
                         test_num, name, i, out_row[i], expected);
                errors++;
                return;
            end
        end
        $display("[PASS] test %0d (%s): all %0d pixels = 0x%02x",
                 test_num, name, out_row.size(), expected);
    endtask

    task automatic check_equal_array(input logic [7:0] expected [], input string name);
        test_num++;
        if (out_row.size() != expected.size()) begin
            $display("[FAIL] test %0d (%s): output size %0d != expected size %0d",
                     test_num, name, out_row.size(), expected.size());
            errors++;
            return;
        end
        for (int i = 0; i < out_row.size(); i++) begin
            if (out_row[i] !== expected[i]) begin
                $display("[FAIL] test %0d (%s): pixel %0d = 0x%02x, expected 0x%02x",
                         test_num, name, i, out_row[i], expected[i]);
                errors++;
                return;
            end
        end
        $display("[PASS] test %0d (%s): %0d pixels match input",
                 test_num, name, out_row.size());
    endtask

    initial begin
        in_data  = 8'h00;
        in_valid = 1'b0;
        in_sof   = 1'b0;
        in_eol   = 1'b0;
        in_eof   = 1'b0;
        in_err   = 1'b0;

        repeat (5) @(posedge clk);
        aresetn = 1'b1;
        repeat (3) @(posedge clk);

        // ----------------------------------------------------------
        // Test 1: True OB row — uniform Y=36 across 16 pixels
        // Expectation: all output pixels = OB_FILL_Y (128)
        // ----------------------------------------------------------
        begin
            automatic logic [7:0] row [] ='{16{8'd36}};
            out_row.delete();
            drive_row(row, 1, 0);
            wait_pipeline_flush();
            check_all_equal(8'd128, "OB row Y=36 uniform -> mask");
        end

        // ----------------------------------------------------------
        // Test 2: OB row with slight variation (Y=36,37,35,36,...)
        // range=2 <= OB_UNIFORMITY=3, all < 50 -> MASK
        // ----------------------------------------------------------
        begin
            automatic logic [7:0] row [] ='{8'd36, 8'd37, 8'd35, 8'd36,
                                   8'd35, 8'd37, 8'd36, 8'd35,
                                   8'd36, 8'd36, 8'd37, 8'd35,
                                   8'd36, 8'd35, 8'd37, 8'd36};
            out_row.delete();
            drive_row(row, 0, 0);
            wait_pipeline_flush();
            check_all_equal(8'd128, "OB row Y=35-37 (range 2) -> mask");
        end

        // ----------------------------------------------------------
        // Test 3: Image row with dark first pixel only (Y=45,200,210,180,...)
        // First pixel below threshold but range across first 4 huge -> PASS
        // ----------------------------------------------------------
        begin
            automatic logic [7:0] row [] ='{8'd45, 8'd200, 8'd210, 8'd180,
                                   8'd150, 8'd120, 8'd100, 8'd80,
                                   8'd60, 8'd40, 8'd55, 8'd75,
                                   8'd95, 8'd115, 8'd135, 8'd155};
            out_row.delete();
            drive_row(row, 0, 0);
            wait_pipeline_flush();
            check_equal_array(row, "image row, dark first pixel only -> pass");
        end

        // ----------------------------------------------------------
        // Test 4: Bright uniform row (Y=128 throughout)
        // Above threshold -> PASS
        // ----------------------------------------------------------
        begin
            automatic logic [7:0] row [] ='{16{8'd128}};
            out_row.delete();
            drive_row(row, 0, 0);
            wait_pipeline_flush();
            check_equal_array(row, "bright uniform Y=128 -> pass");
        end

        // ----------------------------------------------------------
        // Test 5: Dark but non-uniform row (Y=30,5,49,2,...)
        // All < 50 but range>3 -> PASS (not OB)
        // ----------------------------------------------------------
        begin
            automatic logic [7:0] row [] ='{8'd30, 8'd5, 8'd49, 8'd2,
                                   8'd25, 8'd45, 8'd10, 8'd40,
                                   8'd35, 8'd15, 8'd48, 8'd8,
                                   8'd20, 8'd42, 8'd12, 8'd38};
            out_row.delete();
            drive_row(row, 0, 0);
            wait_pipeline_flush();
            check_equal_array(row, "dark non-uniform -> pass");
        end

        // ----------------------------------------------------------
        // Test 6: Uniform but above threshold (Y=80,81,80,79,...)
        // Uniform but >= 50 -> PASS
        // ----------------------------------------------------------
        begin
            automatic logic [7:0] row [] ='{8'd80, 8'd81, 8'd80, 8'd79,
                                   8'd80, 8'd81, 8'd79, 8'd80,
                                   8'd80, 8'd81, 8'd80, 8'd79,
                                   8'd80, 8'd81, 8'd80, 8'd79};
            out_row.delete();
            drive_row(row, 0, 0);
            wait_pipeline_flush();
            check_equal_array(row, "uniform Y=80 above threshold -> pass");
        end

        // ----------------------------------------------------------
        // Test 7: enable=0 (bypass) on a true OB row
        // Should pass through unchanged
        // ----------------------------------------------------------
        begin
            automatic logic [7:0] row [] ='{16{8'd36}};
            out_row.delete();
            enable = 1'b0;
            drive_row(row, 0, 0);
            wait_pipeline_flush();
            check_equal_array(row, "enable=0 bypass on OB row -> pass");
            enable = 1'b1;
        end

        // ----------------------------------------------------------
        // Test 8: Two consecutive lines — OB row followed by image row
        // First should be masked, second pass through
        // ----------------------------------------------------------
        begin
            automatic logic [7:0] ob_row [] = '{16{8'd36}};
            automatic logic [7:0] img_row [] = '{8'd100, 8'd110, 8'd120, 8'd130,
                                       8'd140, 8'd150, 8'd160, 8'd170,
                                       8'd180, 8'd170, 8'd160, 8'd150,
                                       8'd140, 8'd130, 8'd120, 8'd110};
            out_row.delete();
            drive_row(ob_row, 0, 0);
            // No pipeline flush — chain directly into next line
            drive_row(img_row, 0, 1);
            wait_pipeline_flush();

            // Check first 16 outputs = 128, next 16 = img_row
            test_num++;
            if (out_row.size() != 32) begin
                $display("[FAIL] test %0d (chained OB+img): size %0d != 32",
                         test_num, out_row.size());
                errors++;
            end else begin
                automatic bit ok = 1;
                for (int i = 0; i < 16; i++)
                    if (out_row[i] !== 8'd128) ok = 0;
                for (int i = 0; i < 16; i++)
                    if (out_row[16+i] !== img_row[i]) ok = 0;
                if (ok)
                    $display("[PASS] test %0d (chained OB+img): OB masked, img passed", test_num);
                else begin
                    $display("[FAIL] test %0d (chained OB+img): mismatch", test_num);
                    for (int i = 0; i < 32; i++)
                        $display("  out[%0d]=0x%02x", i, out_row[i]);
                    errors++;
                end
            end
        end

        // ----------------------------------------------------------
        // Test 9: Checkerboard row — 8-pixel dark block then 8 bright
        // (Y_DARK=10 for 8 pixels, then Y_BRIGHT=240 for 8 pixels)
        // First 4 samples are uniform Y=10 → masker WILL FALSELY MASK
        // This test documents that pathological case.
        // ----------------------------------------------------------
        begin
            automatic logic [7:0] row [] = '{8'd10, 8'd10, 8'd10, 8'd10,
                                             8'd10, 8'd10, 8'd10, 8'd10,
                                             8'd240, 8'd240, 8'd240, 8'd240,
                                             8'd240, 8'd240, 8'd240, 8'd240};
            out_row.delete();
            drive_row(row, 0, 0);
            wait_pipeline_flush();
            check_equal_array(row, "checkerboard row (8 dark + 8 bright) -> pass");
        end

        // ----------------------------------------------------------
        // Test 10: Checkerboard with short dark prefix
        // (Y=10 for 2 pixels, then Y=240) - range across first 4 huge → PASS
        // ----------------------------------------------------------
        begin
            automatic logic [7:0] row [] = '{8'd10, 8'd10, 8'd240, 8'd240,
                                             8'd10, 8'd10, 8'd240, 8'd240,
                                             8'd10, 8'd10, 8'd240, 8'd240,
                                             8'd10, 8'd10, 8'd240, 8'd240};
            out_row.delete();
            drive_row(row, 0, 0);
            wait_pipeline_flush();
            check_equal_array(row, "checkerboard 2px blocks -> pass");
        end

        // ----------------------------------------------------------
        // Test 11: Grayscale row Y=64 — uniform, above OB_THRESHOLD=50
        // → PASS (not OB; safely above threshold)
        // ----------------------------------------------------------
        begin
            automatic logic [7:0] row [] = '{16{8'd64}};
            out_row.delete();
            drive_row(row, 0, 0);
            wait_pipeline_flush();
            check_equal_array(row, "grayscale uniform Y=64 -> pass");
        end

        // ----------------------------------------------------------
        // Test 12: Grayscale row Y=200 — uniform bright
        // → PASS
        // ----------------------------------------------------------
        begin
            automatic logic [7:0] row [] = '{16{8'd200}};
            out_row.delete();
            drive_row(row, 0, 0);
            wait_pipeline_flush();
            check_equal_array(row, "grayscale uniform Y=200 -> pass");
        end

        // ----------------------------------------------------------
        // Test 13: Gradient 0→240 (steps of 16)
        // First 4 pixels (0,16,32,48): max=48<50 BUT range=48 > 3 → PASS
        // ----------------------------------------------------------
        begin
            automatic logic [7:0] row [] = '{8'd0, 8'd16, 8'd32, 8'd48,
                                             8'd64, 8'd80, 8'd96, 8'd112,
                                             8'd128, 8'd144, 8'd160, 8'd176,
                                             8'd192, 8'd208, 8'd224, 8'd240};
            out_row.delete();
            drive_row(row, 0, 0);
            wait_pipeline_flush();
            check_equal_array(row, "gradient 0->240 -> pass");
        end

        // ----------------------------------------------------------
        // Test 14: Gradient with tight dark start (10,11,12,13,...,240)
        // First 4 (10..13): max<50 AND range=3 → current masker MASKS
        // Full-line range = 230 → after fix → PASS
        // ----------------------------------------------------------
        begin
            automatic logic [7:0] row [] = '{8'd10, 8'd11, 8'd12, 8'd13,
                                             8'd50, 8'd80, 8'd110, 8'd140,
                                             8'd170, 8'd200, 8'd230, 8'd240,
                                             8'd230, 8'd200, 8'd170, 8'd140};
            out_row.delete();
            drive_row(row, 0, 0);
            wait_pipeline_flush();
            check_equal_array(row, "gradient tight dark start -> pass");
        end

        // ----------------------------------------------------------
        // Test 15: Reverse gradient 240→0 (steps of -16)
        // First 4 (240,224,208,192): all bright → PASS
        // ----------------------------------------------------------
        begin
            automatic logic [7:0] row [] = '{8'd240, 8'd224, 8'd208, 8'd192,
                                             8'd176, 8'd160, 8'd144, 8'd128,
                                             8'd112, 8'd96, 8'd80, 8'd64,
                                             8'd48, 8'd32, 8'd16, 8'd0};
            out_row.delete();
            drive_row(row, 0, 0);
            wait_pipeline_flush();
            check_equal_array(row, "reverse gradient 240->0 -> pass");
        end

        // ----------------------------------------------------------
        // Test 16: Random pixel pattern (16 pixels, full 0-255 range)
        // Random pixels have wide range → PASS
        // ----------------------------------------------------------
        begin
            automatic logic [7:0] row [16];
            for (int i = 0; i < 16; i++) row[i] = $urandom_range(0, 255);
            out_row.delete();
            drive_row(row, 0, 0);
            wait_pipeline_flush();
            check_equal_array(row, "random pixel pattern -> pass");
        end

        // ----------------------------------------------------------
        // Test 17: Multiple random rows (stress test)
        // ----------------------------------------------------------
        for (int trial = 0; trial < 5; trial++) begin
            automatic logic [7:0] row [16];
            for (int i = 0; i < 16; i++) row[i] = $urandom_range(0, 255);
            out_row.delete();
            drive_row(row, 0, 0);
            wait_pipeline_flush();
            check_equal_array(row, $sformatf("random trial %0d -> pass", trial));
        end

        repeat (10) @(posedge clk);
        if (errors == 0)
            $display("\n==== ALL TESTS PASSED (%0d tests) ====", test_num);
        else
            $display("\n==== TEST FAILED: %0d errors out of %0d tests ====", errors, test_num);
        $finish;
    end

    initial begin
        #100000;
        $display("[FAIL] timeout");
        $fatal;
    end

endmodule
