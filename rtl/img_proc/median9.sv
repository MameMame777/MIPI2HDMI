`timescale 1ns / 1ps
`default_nettype none

// median9 (2026-06-25): pipelined median-of-9 for one 8-bit channel.
// Computes the MEDIAN (5th order statistic) of nine 8-bit unsigned samples with the
// classic 19 compare-exchange (CAS) Smith/Paeth median-of-9 network. It does NOT fully
// sort -- only the median is produced, landing in working element index 4. cas(a,b) =
// {min(a,b), max(a,b)}.
//
// Sample order MUST match axis_rgb_conv3x3's flattened window tap order
// (idx = r*3 + c, 0=top-left .. 8=bot-right; centre = s4):
//   s0 s1 s2   (row N-2)
//   s3 s4 s5   (row N-1, s4 = centre)
//   s6 s7 s8   (row N)
//
// PIPELINE: 5 register stages (LATENCY = 5 cycles when in_en is held high every cycle).
// The 19 CAS form 9 dependency layers; partitioned <=2 serial CAS per stage so each
// stage is ~2 (compare + 8b-mux) deep -> closes 100 MHz on the congested xc7z020 sysclk.
// (v2: stage 3's 3-serial-CAS path D5/D6/D7 was the WNS critical net at -0.175 -> split into
//  3a=D5 and 3b=D6,D7, giving the 5-stage pipeline.)
//
// CORRECTNESS: this exact staged network was verified (workflow design pass) by Knuth's
// 0-1 principle (all 512 binary vectors), all 362,880 permutations of distinct values,
// and 3e5 random 0..255 vectors. tb_median9 re-checks it in-tree.
module median9 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_en,                 // pipeline advance (tie high to advance every cycle)
    input  wire [7:0]  s0, s1, s2,
    input  wire [7:0]  s3, s4, s5,
    input  wire [7:0]  s6, s7, s8,
    output logic [7:0] med                    // median, 4 cycles after the inputs
);
    // compare-exchange: returns {lo,hi} = {min,max}
    function automatic logic [15:0] cas(input logic [7:0] a, input logic [7:0] b);
        cas = (a > b) ? {b, a} : {a, b};
    endfunction

    // ===================== STAGE 1 : layers D1, D2 =====================
    // D1: cas(1,2) cas(4,5) cas(7,8)   D2: cas(0,1) cas(3,4) cas(6,7)
    logic [7:0] p1 [0:8];
    always_ff @(posedge clk or negedge rst_n) begin
        logic [15:0] d1_12, d1_45, d1_78;       // D1 results
        logic [7:0]  t0, t1, t2, t3, t4, t5, t6, t7, t8;
        logic [15:0] d2_01, d2_34, d2_67;       // D2 results
        if (!rst_n) begin
            for (int k = 0; k < 9; k++) p1[k] <= '0;
        end else if (in_en) begin
            // D1 (parallel)
            d1_12 = cas(s1, s2); d1_45 = cas(s4, s5); d1_78 = cas(s7, s8);
            t0 = s0;            t3 = s3;            t6 = s6;
            t1 = d1_12[15:8]; t2 = d1_12[7:0];      // lo=[15:8], hi=[7:0]
            t4 = d1_45[15:8]; t5 = d1_45[7:0];
            t7 = d1_78[15:8]; t8 = d1_78[7:0];
            // D2 (parallel)
            d2_01 = cas(t0, t1); d2_34 = cas(t3, t4); d2_67 = cas(t6, t7);
            p1[0] <= d2_01[15:8]; p1[1] <= d2_01[7:0];
            p1[3] <= d2_34[15:8]; p1[4] <= d2_34[7:0];
            p1[6] <= d2_67[15:8]; p1[7] <= d2_67[7:0];
            p1[2] <= t2; p1[5] <= t5; p1[8] <= t8;
        end
    end

    // ===================== STAGE 2 : layers D3, D4 =====================
    // D3: cas(1,2) cas(4,5) cas(7,8)   D4: cas(0,3) cas(5,8) cas(4,7)
    logic [7:0] p2 [0:8];
    always_ff @(posedge clk or negedge rst_n) begin
        logic [15:0] d3_12, d3_45, d3_78;
        logic [7:0]  t0, t1, t2, t3, t4, t5, t6, t7, t8;
        logic [15:0] d4_03, d4_58, d4_47;
        if (!rst_n) begin
            for (int k = 0; k < 9; k++) p2[k] <= '0;
        end else if (in_en) begin
            d3_12 = cas(p1[1], p1[2]); d3_45 = cas(p1[4], p1[5]); d3_78 = cas(p1[7], p1[8]);
            t0 = p1[0];           t3 = p1[3];           t6 = p1[6];
            t1 = d3_12[15:8]; t2 = d3_12[7:0];
            t4 = d3_45[15:8]; t5 = d3_45[7:0];
            t7 = d3_78[15:8]; t8 = d3_78[7:0];
            d4_03 = cas(t0, t3); d4_58 = cas(t5, t8); d4_47 = cas(t4, t7);
            p2[0] <= d4_03[15:8]; p2[3] <= d4_03[7:0];
            p2[5] <= d4_58[15:8]; p2[8] <= d4_58[7:0];
            p2[4] <= d4_47[15:8]; p2[7] <= d4_47[7:0];
            p2[1] <= t1; p2[2] <= t2; p2[6] <= t6;
        end
    end

    // ===================== STAGE 3a : layer D5 =====================
    // D5: cas(3,6) cas(1,4) cas(2,5)  (3 parallel CAS) -> register p3a
    logic [7:0] p3a [0:8];
    always_ff @(posedge clk or negedge rst_n) begin
        logic [15:0] d5_36, d5_14, d5_25;
        if (!rst_n) begin
            for (int k = 0; k < 9; k++) p3a[k] <= '0;
        end else if (in_en) begin
            d5_36 = cas(p2[3], p2[6]); d5_14 = cas(p2[1], p2[4]); d5_25 = cas(p2[2], p2[5]);
            p3a[0] <= p2[0]; p3a[7] <= p2[7]; p3a[8] <= p2[8];
            p3a[3] <= d5_36[15:8]; p3a[6] <= d5_36[7:0];
            p3a[1] <= d5_14[15:8]; p3a[4] <= d5_14[7:0];
            p3a[2] <= d5_25[15:8]; p3a[5] <= d5_25[7:0];
        end
    end

    // ===================== STAGE 3b : layers D6, D7 =====================
    // D6: cas(4,7)  then  D7: cas(2,4) (reads D6's lo, u4) -> register p3
    logic [7:0] p3 [0:8];
    always_ff @(posedge clk or negedge rst_n) begin
        logic [15:0] d6_47, d7_24;
        logic [7:0]  u4, u7, x2, x4;
        if (!rst_n) begin
            for (int k = 0; k < 9; k++) p3[k] <= '0;
        end else if (in_en) begin
            d6_47 = cas(p3a[4], p3a[7]); u4 = d6_47[15:8]; u7 = d6_47[7:0];
            d7_24 = cas(p3a[2], u4);     x2 = d7_24[15:8]; x4 = d7_24[7:0];
            p3[0] <= p3a[0]; p3[1] <= p3a[1]; p3[2] <= x2; p3[3] <= p3a[3];
            p3[4] <= x4; p3[5] <= p3a[5]; p3[6] <= p3a[6]; p3[7] <= u7; p3[8] <= p3a[8];
        end
    end

    // ===================== STAGE 4 : layers D8, D9 -> median in idx 4 =====
    // D8: cas(4,6)   D9: cas(2,4)  -> median = max of (p3[2], min(p3[4],p3[6]))
    always_ff @(posedge clk or negedge rst_n) begin
        logic [15:0] d8_46, d9_24;
        logic [7:0]  x4;
        if (!rst_n) begin
            med <= '0;
        end else if (in_en) begin
            d8_46 = cas(p3[4], p3[6]); x4 = d8_46[15:8];   // lo -> idx4
            d9_24 = cas(p3[2], x4);    med <= d9_24[7:0];  // hi(max) -> idx4 = MEDIAN
        end
    end
endmodule

`default_nettype wire
