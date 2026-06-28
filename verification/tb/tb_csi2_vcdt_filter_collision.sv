`timescale 1ns / 1ps

// csi2_vcdt_filter pkt_done + pkt_hdr_valid COLLISION repro (2026-06-17).
//
// HW chase (diary 2026-06-17): the residual bottom band = scattered LS/LE short-
// packet drops. Localized past the frontend and parser (parser LS == parser long)
// to the VC/DT filter. Suspected bug: a long packet's pkt_done (= crc_check_valid)
// can land on the SAME cycle as the next short packet's pkt_hdr_valid (the LE/LS
// right after the long). In that cycle the filter:
//   done_di = pkt_hdr_valid ? pkt_di : active_di;  // mis-attributes the OLD end
//   packet_admit <= 1'b0;                           // overrides the NEW admit
// so the colliding short is dropped/corrupted (~3% of lines = the band).
//
// This TB drives realistic per-line [LS, long, LE] packets and, for half the
// lines, forces the long's crc_check_valid to COINCIDE with the next LE header.
// It counts complete short packets out of the filter; a deficit reproduces the
// bug. DSim-capable (byte-level, no D-PHY).
module tb_csi2_vcdt_filter_collision;
    logic core_clk = 0, core_aresetn;
    always #5 core_clk = ~core_clk;

    logic        pkt_hdr_valid, pkt_is_long, pkt_is_short, pkt_done;
    logic [7:0]  pkt_di; logic [15:0] pkt_wc;
    logic        ecc_corrected, ecc_uncorrectable, crc_check_valid, crc_match;
    logic [7:0]  payload_data; logic payload_valid, payload_first, payload_last;

    logic [7:0]  o_di; logic [15:0] o_wc;
    logic        o_is_short, o_is_long, o_start, o_end, o_err;
    logic [7:0]  o_pd; logic o_pv, o_pf, o_pl;
    logic [15:0] drop_vc, drop_dt;

    csi2_vcdt_filter dut (
        .core_clk(core_clk), .core_aresetn(core_aresetn),
        .cfg_expected_vc(2'd0), .cfg_expected_dt(6'h22), .cfg_pass_short(1'b1), .cfg_pass_emb_data(1'b0),
        .pkt_hdr_valid(pkt_hdr_valid), .pkt_di(pkt_di), .pkt_wc(pkt_wc),
        .pkt_is_long(pkt_is_long), .pkt_is_short(pkt_is_short), .pkt_done(pkt_done),
        .ecc_corrected(ecc_corrected), .ecc_uncorrectable(ecc_uncorrectable),
        .crc_check_valid(crc_check_valid), .crc_match(crc_match),
        .payload_data(payload_data), .payload_valid(payload_valid),
        .payload_first(payload_first), .payload_last(payload_last),
        .out_pkt_di(o_di), .out_pkt_wc(o_wc), .out_pkt_is_short(o_is_short),
        .out_pkt_is_long(o_is_long), .out_pkt_start(o_start), .out_pkt_end(o_end),
        .out_pkt_err(o_err), .out_payload_data(o_pd), .out_payload_valid(o_pv),
        .out_payload_first(o_pf), .out_payload_last(o_pl),
        .sts_drop_vc_cnt(drop_vc), .sts_drop_dt_cnt(drop_dt)
    );

    // count complete short packets out (start with a matching end)
    int short_starts, short_ends, long_ends;
    logic short_open;
    always_ff @(posedge core_clk) begin
        if (!core_aresetn) begin
            short_starts <= 0; short_ends <= 0; long_ends <= 0; short_open <= 0;
        end else begin
            if (o_start && o_is_short) short_starts <= short_starts + 1;
            if (o_end && o_is_short)   short_ends   <= short_ends + 1;
            if (o_end && o_is_long)    long_ends    <= long_ends + 1;
        end
    end

    task automatic idle(int n); repeat(n) begin
        @(posedge core_clk);
        pkt_hdr_valid<=0; pkt_done<=0; pkt_is_long<=0; pkt_is_short<=0;
        crc_check_valid<=0; payload_valid<=0; ecc_uncorrectable<=0;
    end endtask

    // short packet: header + done in the same cycle (parser short behaviour)
    task automatic short_pkt(input [5:0] dt);
        @(posedge core_clk);
        pkt_hdr_valid<=1; pkt_di<={2'b00,dt}; pkt_wc<=0; pkt_is_short<=1; pkt_is_long<=0;
        pkt_done<=1; crc_check_valid<=0; payload_valid<=0;
        @(posedge core_clk);
        pkt_hdr_valid<=0; pkt_done<=0; pkt_is_short<=0;
    endtask

    // long packet: header, 2 payload beats, then crc_check_valid (= pkt_done).
    // collide=1 -> assert the NEXT LE header on the SAME cycle as crc_check_valid.
    task automatic long_then_le(input bit collide);
        @(posedge core_clk);                          // long header
        pkt_hdr_valid<=1; pkt_di<=8'h22; pkt_wc<=16'd2; pkt_is_long<=1; pkt_is_short<=0;
        pkt_done<=0; crc_check_valid<=0;
        @(posedge core_clk);
        pkt_hdr_valid<=0; pkt_is_long<=0;
        payload_valid<=1; payload_first<=1; payload_last<=0; payload_data<=8'hA0;
        @(posedge core_clk);
        payload_first<=0; payload_last<=1; payload_data<=8'hA1;
        @(posedge core_clk);
        payload_valid<=0; payload_last<=0;
        // crc_check_valid = the long's pkt_done
        @(posedge core_clk);
        crc_check_valid<=1; crc_match<=1; pkt_done<=1;
        if (collide) begin                            // LE header on the SAME cycle
            pkt_hdr_valid<=1; pkt_di<=8'h03; pkt_wc<=0; pkt_is_short<=1; pkt_is_long<=0;
        end
        @(posedge core_clk);
        crc_check_valid<=0; pkt_done<=0; pkt_hdr_valid<=0; pkt_is_short<=0;
        if (!collide) short_pkt(6'h03);               // clean: LE separately
    endtask

    int N = 8;
    initial begin
        core_aresetn=0; pkt_hdr_valid=0; pkt_done=0; pkt_is_long=0; pkt_is_short=0;
        pkt_di=0; pkt_wc=0; ecc_corrected=0; ecc_uncorrectable=0;
        crc_check_valid=0; crc_match=1; payload_data=0; payload_valid=0;
        payload_first=0; payload_last=0;
        repeat(6) @(posedge core_clk); core_aresetn=1; repeat(2) @(posedge core_clk);

        // N lines, COLLIDING (long crc coincides with the LE header)
        for (int i=0;i<N;i++) begin
            short_pkt(6'h02);          // LS
            long_then_le(1'b1);        // long + colliding LE
            idle(1);
        end
        idle(4);
        $display("[COLLIDE] short_starts=%0d short_ends=%0d long_ends=%0d (fed LS=%0d LE=%0d long=%0d)",
                 short_starts, short_ends, long_ends, N, N, N);
        // Expect 2N complete shorts (N LS + N LE) and N long ends if no drop.
        if (short_ends < 2*N)
            $display("BUG REPRODUCED: %0d/%0d short packets completed (%0d dropped at the long->LE collision)",
                     short_ends, 2*N, 2*N - short_ends);
        else
            $display("NO DROP: all %0d shorts completed -> collision handled correctly", 2*N);
        $finish;
    end
    initial begin #1ms; $fatal(1,"timeout"); end
endmodule
