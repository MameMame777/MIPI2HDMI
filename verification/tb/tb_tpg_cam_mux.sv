`timescale 1ns/1ps
// Targeted test for the AND-OR camera/TPG runtime mux.
// Verifies that use_tpg_rt=1 passes only TPG signals and blocks camera,
// and use_tpg_rt=0 passes only camera and blocks TPG.
// Pure combinational — no clocks needed.
module tb_tpg_cam_mux;

    logic        use_tpg_rt;

    // TPG inputs
    logic        tpg_byte_valid;
    logic        tpg_byte_sop;
    logic        tpg_byte_eop;
    logic [15:0] tpg_byte_data;
    logic [1:0]  tpg_byte_keep;

    // Camera CDC inputs
    logic        cdc_byte_valid;
    logic        cdc_byte_sop;
    logic        cdc_byte_eop;
    logic [15:0] cdc_byte_data;
    logic [1:0]  cdc_byte_keep;

    // Mux outputs (DUT — inline AND-OR)
    // DUT: pipelined AND-OR (mirrors RTL v17)
    logic clk, resetn;
    initial clk = 0;
    always #5 clk = ~clk;  // 100 MHz

    logic        tpg_gate_r,  cam_gate_r;
    logic        tpg_valid_r, cdc_valid_r;
    logic        tpg_sop_r,   cdc_sop_r;
    logic        tpg_eop_r,   cdc_eop_r;
    logic [15:0] tpg_data_r,  cdc_data_r;
    logic [1:0]  tpg_keep_r,  cdc_keep_r;

    always_ff @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            tpg_gate_r  <= 0; cam_gate_r  <= 1;
            tpg_valid_r <= 0; cdc_valid_r <= 0;
            tpg_sop_r   <= 0; cdc_sop_r   <= 0;
            tpg_eop_r   <= 0; cdc_eop_r   <= 0;
            tpg_data_r  <= 0; cdc_data_r  <= 0;
            tpg_keep_r  <= 0; cdc_keep_r  <= 0;
        end else begin
            tpg_gate_r  <= use_tpg_rt;    cam_gate_r  <= ~use_tpg_rt;
            tpg_valid_r <= tpg_byte_valid; cdc_valid_r <= cdc_byte_valid;
            tpg_sop_r   <= tpg_byte_sop;   cdc_sop_r   <= cdc_byte_sop;
            tpg_eop_r   <= tpg_byte_eop;   cdc_eop_r   <= cdc_byte_eop;
            tpg_data_r  <= tpg_byte_data;  cdc_data_r  <= cdc_byte_data;
            tpg_keep_r  <= tpg_byte_keep;  cdc_keep_r  <= cdc_byte_keep;
        end
    end

    wire        pkt_byte_valid = (tpg_valid_r & tpg_gate_r) | (cdc_valid_r & cam_gate_r);
    wire        pkt_byte_sop   = (tpg_sop_r   & tpg_gate_r) | (cdc_sop_r   & cam_gate_r);
    wire        pkt_byte_eop   = (tpg_eop_r   & tpg_gate_r) | (cdc_eop_r   & cam_gate_r);
    wire [15:0] pkt_byte_data  = ({16{tpg_gate_r}} & tpg_data_r) | ({16{cam_gate_r}} & cdc_data_r);
    wire [1:0]  pkt_byte_keep  = ({2{tpg_gate_r}}  & tpg_keep_r) | ({2{cam_gate_r}}  & cdc_keep_r);

    int pass_cnt = 0;
    int fail_cnt = 0;

    task automatic check(
        input string label,
        input logic exp_valid, exp_sop, exp_eop,
        input logic [15:0] exp_data,
        input logic [1:0]  exp_keep
    );
        if (pkt_byte_valid !== exp_valid ||
            pkt_byte_sop   !== exp_sop   ||
            pkt_byte_eop   !== exp_eop   ||
            pkt_byte_data  !== exp_data  ||
            pkt_byte_keep  !== exp_keep) begin
            $display("FAIL [%s] valid=%b(exp%b) sop=%b(exp%b) eop=%b(exp%b) data=%04x(exp%04x) keep=%b(exp%b)",
                label,
                pkt_byte_valid, exp_valid,
                pkt_byte_sop,   exp_sop,
                pkt_byte_eop,   exp_eop,
                pkt_byte_data,  exp_data,
                pkt_byte_keep,  exp_keep);
            fail_cnt++;
        end else begin
            $display("PASS [%s]", label);
            pass_cnt++;
        end
    endtask

    task automatic clk_step(input int n = 1);
        repeat (n) @(posedge clk); #1;
    endtask

    initial begin
        resetn = 0;
        use_tpg_rt    = 0;
        tpg_byte_valid = 0; tpg_byte_sop = 0; tpg_byte_eop = 0;
        tpg_byte_data  = 0; tpg_byte_keep = 0;
        cdc_byte_valid = 0; cdc_byte_sop = 0; cdc_byte_eop = 0;
        cdc_byte_data  = 0; cdc_byte_keep = 0;
        clk_step(2);
        resetn = 1;
        clk_step(1);

        // Drive both paths with distinct values
        tpg_byte_valid = 1'b1; tpg_byte_sop = 1'b1; tpg_byte_eop = 1'b0;
        tpg_byte_data  = 16'hA5A5; tpg_byte_keep = 2'b11;
        cdc_byte_valid = 1'b1; cdc_byte_sop = 1'b0; cdc_byte_eop = 1'b1;
        cdc_byte_data  = 16'h5A5A; cdc_byte_keep = 2'b10;

        // --- use_tpg_rt = 1: TPG through after 1 pipeline cycle ---
        use_tpg_rt = 1'b1;
        clk_step(1);  // pipeline register captures inputs
        check("tpg_sel_valid", 1'b1, 1'b1, 1'b0, 16'hA5A5, 2'b11);

        // Camera valid blocked: tpg_valid=0, cdc_valid=1 → pkt_valid must be 0
        tpg_byte_valid = 1'b0; cdc_byte_valid = 1'b1;
        clk_step(1);
        check("tpg_sel_cam_blocked", 1'b0, tpg_byte_sop, tpg_byte_eop, tpg_byte_data, tpg_byte_keep);

        // Restore
        tpg_byte_valid = 1'b1; cdc_byte_valid = 1'b1;

        // --- use_tpg_rt = 0: camera through ---
        use_tpg_rt = 1'b0;
        clk_step(1);
        check("cam_sel_valid", 1'b1, 1'b0, 1'b1, 16'h5A5A, 2'b10);

        // TPG valid blocked: tpg_valid=1, cdc_valid=0 → pkt_valid must be 0
        tpg_byte_valid = 1'b1; cdc_byte_valid = 1'b0;
        clk_step(1);
        check("cam_sel_tpg_blocked", 1'b0, cdc_byte_sop, cdc_byte_eop, cdc_byte_data, cdc_byte_keep);

        // --- Switching mid-stream ---
        tpg_byte_valid = 1'b1; tpg_byte_sop = 1'b1; tpg_byte_data = 16'hDEAD; tpg_byte_keep = 2'b11;
        cdc_byte_valid = 1'b1; cdc_byte_sop = 1'b0; cdc_byte_data = 16'hBEEF; cdc_byte_keep = 2'b10;
        use_tpg_rt = 1'b1; clk_step(1);
        check("switch_to_tpg", 1'b1, 1'b1, 1'b0, 16'hDEAD, 2'b11);
        use_tpg_rt = 1'b0; clk_step(1);
        check("switch_to_cam", 1'b1, 1'b0, 1'b1, 16'hBEEF, 2'b10);
        use_tpg_rt = 1'b1; clk_step(1);
        check("switch_back_tpg", 1'b1, 1'b1, 1'b0, 16'hDEAD, 2'b11);

        // Summary
        $display("=== RESULT: %0d passed, %0d failed ===", pass_cnt, fail_cnt);
        if (fail_cnt == 0)
            $display("ALL PASS — AND-OR mux logic correct");
        else
            $display("FAILURES — check mux implementation");
        $finish;
    end

endmodule
