`timescale 1ns / 1ps
//
// tb_dphy_raw_byte_ringbuf
//
// Unit test for the 32-bit-extended dphy_raw_byte_ringbuf.
// All driver signals settle on the negedge of byte_clk to avoid race with
// posedge-clocked DUT sampling. Verifies:
//   S0) Free-run mode (trigger_mode=0): arm → captures DEPTH entries
//   S1) Markers: sot_l0/sot_l1/sync_header_valid bits propagate per captured cycle
//   S2) Trigger mode (trigger_mode=1): armed → waits → captures on sync_trigger edge
//   S3) hi/lo split read returns correct halves
//

module tb_dphy_raw_byte_ringbuf;
    localparam int DEPTH = 16;

    logic        byte_clk = 0;
    logic        rd_clk   = 0;
    logic        rst_n    = 0;

    logic [7:0]  lane0_byte;
    logic [7:0]  lane1_byte;
    logic        sync_header_valid_byte;
    logic        sync_trigger_byte;
    logic        arm_trigger_byte;
    logic        trigger_mode_byte;

    logic [9:0]  rd_addr;
    logic [15:0] rd_data;
    logic [9:0]  last_write_addr_sync;
    logic        full_sync;
    logic        armed_sync;
    logic        waiting_sync;

    always #5 byte_clk = ~byte_clk;
    always #4 rd_clk   = ~rd_clk;

    dphy_raw_byte_ringbuf #(.DEPTH(DEPTH)) dut (
        .byte_clk(byte_clk),
        .rst_n_byte(rst_n),
        .lane0_byte_in(lane0_byte),
        .lane1_byte_in(lane1_byte),
        .sync_header_valid_byte(sync_header_valid_byte),
        .sync_trigger_byte(sync_trigger_byte),
        .arm_trigger_byte(arm_trigger_byte),
        .trigger_mode_byte(trigger_mode_byte),
        .rd_clk(rd_clk),
        .rd_addr(rd_addr),
        .rd_data(rd_data),
        .last_write_addr_sync(last_write_addr_sync),
        .full_sync(full_sync),
        .armed_sync(armed_sync),
        .waiting_sync(waiting_sync)
    );

    // ------------- Helpers -------------
    int errors = 0;
    task expect_eq(input string label, input int got, input int want);
        if (got !== want) begin
            $display("[FAIL] %s: got=0x%0x want=0x%0x", label, got, want);
            errors++;
        end else begin
            $display("[ OK ] %s: 0x%0x", label, got);
        end
    endtask

    task automatic read_word(input int idx, output logic [31:0] word);
        logic [15:0] lo, hi;
        // settle reads on rd_clk; allow 4 cycles for 2-stage latency
        rd_addr = {1'b0, idx[8:0]};
        repeat (4) @(posedge rd_clk);
        lo = rd_data;
        rd_addr = {1'b1, idx[8:0]};
        repeat (4) @(posedge rd_clk);
        hi = rd_data;
        word = {hi, lo};
    endtask

    task wait_full(input int max_cycles);
        int i;
        for (i = 0; i < max_cycles; i++) begin
            @(posedge byte_clk);
            if (full_sync) return;
        end
        $display("[FAIL] wait_full: timeout after %0d cycles", max_cycles);
        errors++;
    endtask

    task reset_dut;
        rst_n = 0;
        repeat (4) @(posedge byte_clk);
        rst_n = 1;
        repeat (4) @(posedge byte_clk);
    endtask

    initial begin
        // Init
        lane0_byte = 8'h00;
        lane1_byte = 8'h00;
        sync_header_valid_byte = 0;
        sync_trigger_byte      = 0;
        arm_trigger_byte       = 0;
        trigger_mode_byte      = 0;
        rd_addr                = 0;

        repeat (4) @(posedge byte_clk);
        rst_n = 1;
        repeat (4) @(posedge byte_clk);

        // === S0: free-run, drive lane bytes = cycle counter ===
        $display("\n--- S0: free-run capture ---");
        trigger_mode_byte = 0;

        // Drive sequential data on every byte_clk via fork process
        fork : driver_s0
            begin
                int c;
                for (c = 0; c < DEPTH + 8; c++) begin
                    @(negedge byte_clk);
                    lane0_byte = 8'h00 | c[7:0];
                    lane1_byte = 8'h80 | c[7:0];
                    sync_header_valid_byte = c[0];
                end
            end
            begin
                @(negedge byte_clk);    // wait one cycle so driver is at c=0
                arm_trigger_byte = 1;
                @(negedge byte_clk);
                arm_trigger_byte = 0;
            end
        join_any
        // Wait until fully captured
        wait_full(DEPTH + 12);
        disable driver_s0;
        expect_eq("S0: full=1",  full_sync,  1);
        expect_eq("S0: armed=0", armed_sync, 0);

        // Read entry 0 — first_entry marker must be set
        begin
            logic [31:0] w0;
            read_word(0, w0);
            expect_eq("S0: entry0 first_entry marker", w0[20], 1'b1);
            $display("       entry0: lane0=0x%02x lane1=0x%02x sync_hv=%0b",
                     w0[7:0], w0[15:8], w0[23]);
        end

        // === S1: SoT marker propagation ===
        $display("\n--- S1: SoT marker propagation ---");
        reset_dut();
        trigger_mode_byte = 0;
        // Drive a known sequence; positions of 0xB8 chosen at specific cycles
        fork : driver_s1
            begin
                int c;
                for (c = 0; c < DEPTH + 12; c++) begin
                    @(negedge byte_clk);
                    case (c)
                        2:  begin lane0_byte=8'hB8; lane1_byte=8'hB8; sync_header_valid_byte=1; end
                        5:  begin lane0_byte=8'hB8; lane1_byte=8'h11; sync_header_valid_byte=0; end
                        7:  begin lane0_byte=8'h22; lane1_byte=8'hB8; sync_header_valid_byte=0; end
                        default: begin lane0_byte=8'h00; lane1_byte=8'h00; sync_header_valid_byte=0; end
                    endcase
                end
            end
            begin
                @(negedge byte_clk);
                arm_trigger_byte = 1;
                @(negedge byte_clk);
                arm_trigger_byte = 0;
            end
        join_any
        wait_full(DEPTH + 16);
        disable driver_s1;

        // Check every captured entry: where lane0==0xB8, sot_l0 must be 1.
        // Where lane1==0xB8, sot_l1 must be 1. Vice versa for non-B8 → 0.
        begin
            int idx;
            logic [31:0] w;
            static int b8_l0_words = 0;
            static int b8_l1_words = 0;
            static int marker_mismatch = 0;
            for (idx = 0; idx < DEPTH; idx++) begin
                read_word(idx, w);
                if (w[7:0] == 8'hB8) begin
                    b8_l0_words++;
                    if (w[21] !== 1'b1) begin
                        $display("[FAIL] S1: idx%0d lane0=B8 but sot_l0=0", idx);
                        marker_mismatch++; errors++;
                    end
                end else begin
                    if (w[21] !== 1'b0) begin
                        $display("[FAIL] S1: idx%0d lane0=0x%02x but sot_l0=1", idx, w[7:0]);
                        marker_mismatch++; errors++;
                    end
                end
                if (w[15:8] == 8'hB8) begin
                    b8_l1_words++;
                    if (w[22] !== 1'b1) begin
                        $display("[FAIL] S1: idx%0d lane1=B8 but sot_l1=0", idx);
                        marker_mismatch++; errors++;
                    end
                end else begin
                    if (w[22] !== 1'b0) begin
                        $display("[FAIL] S1: idx%0d lane1=0x%02x but sot_l1=1", idx, w[15:8]);
                        marker_mismatch++; errors++;
                    end
                end
            end
            $display("[ S1 ] captured 0xB8: lane0=%0d times, lane1=%0d times, marker_mismatches=%0d",
                     b8_l0_words, b8_l1_words, marker_mismatch);
            if (b8_l0_words == 0) begin
                $display("[FAIL] S1: no 0xB8 captured on lane0 — driver timing off");
                errors++;
            end
        end

        // === S2: trigger_mode=1, must wait then capture on sync_trigger ===
        $display("\n--- S2: trigger_mode capture on sync_trigger ---");
        reset_dut();
        trigger_mode_byte = 1;
        sync_trigger_byte = 0;
        lane0_byte = 8'h00; lane1_byte = 8'h00;

        @(negedge byte_clk);
        arm_trigger_byte = 1;
        @(negedge byte_clk);
        arm_trigger_byte = 0;

        // 12 cycles of no trigger — verify waiting state, no capture
        repeat (12) @(posedge byte_clk);
        repeat (6)  @(posedge rd_clk);
        expect_eq("S2: armed=1 (waiting)",   armed_sync,   1);
        expect_eq("S2: waiting=1",           waiting_sync, 1);
        expect_eq("S2: full=0 before trig",  full_sync,    0);

        // Drive sync_trigger pulse
        fork : driver_s2
            begin
                int c;
                for (c = 0; c < DEPTH + 12; c++) begin
                    @(negedge byte_clk);
                    lane0_byte = 8'h40 | c[7:0];
                    lane1_byte = 8'hC0 | c[7:0];
                    if (c == 3) sync_trigger_byte = 1;
                    if (c == 4) sync_trigger_byte = 0;
                end
            end
        join_none
        wait_full(DEPTH + 20);
        disable driver_s2;

        expect_eq("S2: full=1 after trigger", full_sync, 1);
        expect_eq("S2: waiting=0",            waiting_sync, 0);

        // Entry 0 must have first_entry marker set
        begin
            logic [31:0] w0;
            read_word(0, w0);
            expect_eq("S2: entry0 first_entry marker", w0[20], 1'b1);
            $display("       entry0: lane0=0x%02x lane1=0x%02x", w0[7:0], w0[15:8]);
        end

        // === S3: idx3 should NOT have first_entry marker ===
        $display("\n--- S3: first_entry marker only at idx 0 ---");
        begin
            logic [31:0] w3;
            read_word(3, w3);
            expect_eq("S3: idx3 first_entry=0", w3[20], 1'b0);
        end

        // === Summary ===
        $display("\n=========================================");
        if (errors == 0) $display("[PASS] All tests passed");
        else             $display("[FAIL] %0d errors", errors);
        $display("=========================================");
        $finish;
    end

    initial begin
        #500000;
        $display("[FAIL] Timeout");
        $finish;
    end
endmodule
