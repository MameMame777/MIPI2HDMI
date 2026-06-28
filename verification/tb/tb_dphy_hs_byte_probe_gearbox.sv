`timescale 1ns / 1ps

module IBUFDS #(
    parameter string DIFF_TERM = "FALSE",
    parameter string IBUF_LOW_PWR = "TRUE",
    parameter string IOSTANDARD = "DEFAULT"
) (
    input  wire I,
    input  wire IB,
    output wire O
);
    assign O = I;
endmodule

module BUFIO (
    input  wire I,
    output wire O
);
    assign O = I;
endmodule

module BUFR #(
    parameter string BUFR_DIVIDE = "BYPASS",
    parameter string SIM_DEVICE = "7SERIES"
) (
    input  wire I,
    input  wire CE,
    input  wire CLR,
    output wire O
);
    assign O = CLR ? 1'b0 : (CE ? I : 1'b0);
endmodule

module IDELAYCTRL (
    input  wire REFCLK,
    input  wire RST,
    output wire RDY
);
    assign RDY = !RST;
endmodule

module IDELAYE2 #(
    parameter string CINVCTRL_SEL = "FALSE",
    parameter string DELAY_SRC = "IDATAIN",
    parameter string HIGH_PERFORMANCE_MODE = "TRUE",
    parameter string IDELAY_TYPE = "FIXED",
    parameter int IDELAY_VALUE = 0,
    parameter string PIPE_SEL = "FALSE",
    parameter real REFCLK_FREQUENCY = 200.0,
    parameter string SIGNAL_PATTERN = "DATA"
) (
    input  wire C,
    input  wire REGRST,
    input  wire LD,
    input  wire CE,
    input  wire INC,
    input  wire LDPIPEEN,
    input  wire CINVCTRL,
    input  wire [4:0] CNTVALUEIN,
    input  wire IDATAIN,
    input  wire DATAIN,
    output wire DATAOUT,
    output wire [4:0] CNTVALUEOUT
);
    assign DATAOUT = IDATAIN;
    assign CNTVALUEOUT = CNTVALUEIN;
endmodule

module ISERDESE2 #(
    parameter string DATA_RATE = "DDR",
    parameter int DATA_WIDTH = 8,
    parameter string DYN_CLKDIV_INV_EN = "FALSE",
    parameter string DYN_CLK_INV_EN = "FALSE",
    parameter string INTERFACE_TYPE = "NETWORKING",
    parameter string IOBDELAY = "IFD",
    parameter int NUM_CE = 1,
    parameter string OFB_USED = "FALSE",
    parameter string SERDES_MODE = "MASTER"
) (
    output wire Q1,
    output wire Q2,
    output wire Q3,
    output wire Q4,
    output wire Q5,
    output wire Q6,
    output wire Q7,
    output wire Q8,
    output wire SHIFTOUT1,
    output wire SHIFTOUT2,
    input  wire BITSLIP,
    input  wire CE1,
    input  wire CE2,
    input  wire CLK,
    input  wire CLKB,
    input  wire CLKDIV,
    input  wire CLKDIVP,
    input  wire D,
    input  wire DDLY,
    input  wire DYNCLKDIVSEL,
    input  wire DYNCLKSEL,
    input  wire OCLK,
    input  wire OCLKB,
    input  wire OFB,
    input  wire RST,
    input  wire SHIFTIN1,
    input  wire SHIFTIN2,
    output wire O
);
    assign {Q8, Q7, Q6, Q5, Q4, Q3, Q2, Q1} = 8'h00;
    assign SHIFTOUT1 = 1'b0;
    assign SHIFTOUT2 = 1'b0;
    assign O = DDLY;
endmodule

module tb_dphy_hs_byte_probe_gearbox;
    logic rst_n;
    logic idelay_ref_clk;
    logic hs_clk_p;
    logic hs_clk_n;
    logic [1:0] data_hs_p;
    logic [1:0] data_hs_n;
    logic [1:0] data_lp_p;
    logic [1:0] data_lp_n;
    logic byte_clk;
    logic idelayctrl_rdy;
    logic hs_clk_seen;
    logic [1:0] lane_sot_seen;
    logic [1:0][7:0] lane_last_byte;
    logic [1:0] lane_raw_changed_seen;
    logic [1:0] lane_raw_non_ff_seen;
    logic [1:0] lane_raw_non_00_seen;
    logic [1:0][7:0] lane_raw_change_count;
    logic [15:0] stream_byte_data;
    logic [1:0] stream_byte_keep;
    logic stream_byte_valid;
    logic stream_byte_sop;
    logic stream_byte_eop;
    logic header_valid;
    logic [7:0] header_di;
    logic [15:0] header_wc;
    logic [7:0] header_ecc;
    logic sync_header_valid;
    logic [7:0] sync_header_di;
    logic [15:0] sync_header_wc;
    logic [7:0] sync_header_ecc;
    logic [2:0] sync_header_rotation_lane0;
    logic [2:0] sync_header_rotation_lane1;
    logic [2:0] sync_header_bit_offset_lane0;
    logic [2:0] sync_header_bit_offset_lane1;
    logic [3:0] sync_header_score;
    logic [2:0] sync_header_start_slot;
    logic [2:0] sync_header_pairing;
    logic [5:0] sync_header_syndrome;
    logic sync_header_ecc_no_error;
    logic sync_header_ecc_corrected;
    logic sync_header_ecc_uncorrectable;
    logic [7:0] header_slot_valid;
    logic [7:0][7:0] header_slot_di;
    logic [7:0][15:0] header_slot_wc;
    logic [7:0][7:0] header_slot_ecc;
    logic [7:0][2:0] header_slot_bitslip_phase;
    logic [7:0][2:0] header_slot_bitslip_phase_lane1;
    logic [7:0][2:0] header_slot_transform;
    logic [7:0][2:0] header_slot_rotation;
    logic [7:0][7:0] header_slot_corr_di;
    logic [7:0][15:0] header_slot_corr_wc;
    logic [7:0][5:0] header_slot_syndrome;
    logic [7:0] header_slot_ecc_no_error;
    logic [7:0] header_slot_ecc_corrected;
    logic [7:0] header_slot_ecc_uncorrectable;
    logic [7:0] trace_slot_valid;
    logic [7:0][7:0] trace_slot_lane0_raw;
    logic [7:0][7:0] trace_slot_lane1_raw;
    logic [7:0][7:0] trace_slot_lane0_candidate;
    logic [7:0][7:0] trace_slot_lane1_candidate;
    logic [7:0][7:0] trace_slot_lane0_aligned;
    logic [7:0][7:0] trace_slot_lane1_aligned;
    logic [7:0][2:0] trace_slot_lane0_rotation;
    logic [7:0][2:0] trace_slot_lane1_rotation;
    logic [7:0][2:0] trace_slot_bitslip_phase_lane0;
    logic [7:0][2:0] trace_slot_bitslip_phase_lane1;
    logic [7:0] trace_slot_sot_hit_lane0;
    logic [7:0] trace_slot_sot_hit_lane1;

    dphy_hs_byte_probe #(
        .LANES(2),
        .SOT_WINDOW_BYTES(8),
        .SWEEP_HOLD_BYTES(4),
        .SWEEP_ENABLE(1'b0),
        .FIXED_BITSLIP_PHASE(0),
        .FIXED_BITSLIP_PHASE_LANE1(0),
        .LANE1_BITSLIP_SWEEP_ENABLE(1'b0),
        .FIXED_TRANSFORM(0),
        .TRACE_TRIGGER_MODE(3),
        .IDELAY_TAP(0),
        .IDELAY_REFCLK_MHZ(200.0),
        .STREAM_PAIRING(0)
    ) dut (
        .rst_n(rst_n),
        .idelay_ref_clk(idelay_ref_clk),
        .idelay_ref_reset(!rst_n),
        .runtime_idelay_tap(5'd0),
        .runtime_bitslip_phase(3'd0),
        .runtime_bitslip_phase_lane1(3'd0),
        .runtime_expected_long_dt(8'h00),
        .sup_enable(1'b0),
        .sup_bufr_clr(1'b0),
        .sup_serdes_rst(1'b0),
        .sup_hs_settled(1'b0),
        .serdes_byte_sample_out(),
        .dphy_hs_clock_clk_p(hs_clk_p),
        .dphy_hs_clock_clk_n(hs_clk_n),
        .dphy_data_hs_p(data_hs_p),
        .dphy_data_hs_n(data_hs_n),
        .dphy_data_lp_p(data_lp_p),
        .dphy_data_lp_n(data_lp_n),
        .byte_clk(byte_clk),
        .idelayctrl_rdy(idelayctrl_rdy),
        .hs_clk_seen(hs_clk_seen),
        .lane_sot_seen(lane_sot_seen),
        .lane_last_byte(lane_last_byte),
        .lane_raw_changed_seen(lane_raw_changed_seen),
        .lane_raw_non_ff_seen(lane_raw_non_ff_seen),
        .lane_raw_non_00_seen(lane_raw_non_00_seen),
        .lane_raw_change_count(lane_raw_change_count),
        .stream_byte_data(stream_byte_data),
        .stream_byte_keep(stream_byte_keep),
        .stream_byte_valid(stream_byte_valid),
        .stream_byte_sop(stream_byte_sop),
        .stream_byte_eop(stream_byte_eop),
        .header_valid(header_valid),
        .header_di(header_di),
        .header_wc(header_wc),
        .header_ecc(header_ecc),
        .sync_header_valid(sync_header_valid),
        .sync_header_di(sync_header_di),
        .sync_header_wc(sync_header_wc),
        .sync_header_ecc(sync_header_ecc),
        .sync_header_rotation_lane0(sync_header_rotation_lane0),
        .sync_header_rotation_lane1(sync_header_rotation_lane1),
        .sync_header_bit_offset_lane0(sync_header_bit_offset_lane0),
        .sync_header_bit_offset_lane1(sync_header_bit_offset_lane1),
        .sync_header_score(sync_header_score),
        .sync_header_start_slot(sync_header_start_slot),
        .sync_header_pairing(sync_header_pairing),
        .sync_header_syndrome(sync_header_syndrome),
        .sync_header_ecc_no_error(sync_header_ecc_no_error),
        .sync_header_ecc_corrected(sync_header_ecc_corrected),
        .sync_header_ecc_uncorrectable(sync_header_ecc_uncorrectable),
        .header_slot_valid(header_slot_valid),
        .header_slot_di(header_slot_di),
        .header_slot_wc(header_slot_wc),
        .header_slot_ecc(header_slot_ecc),
        .header_slot_bitslip_phase(header_slot_bitslip_phase),
        .header_slot_bitslip_phase_lane1(header_slot_bitslip_phase_lane1),
        .header_slot_transform(header_slot_transform),
        .header_slot_rotation(header_slot_rotation),
        .header_slot_corr_di(header_slot_corr_di),
        .header_slot_corr_wc(header_slot_corr_wc),
        .header_slot_syndrome(header_slot_syndrome),
        .header_slot_ecc_no_error(header_slot_ecc_no_error),
        .header_slot_ecc_corrected(header_slot_ecc_corrected),
        .header_slot_ecc_uncorrectable(header_slot_ecc_uncorrectable),
        .trace_slot_valid(trace_slot_valid),
        .trace_slot_lane0_raw(trace_slot_lane0_raw),
        .trace_slot_lane1_raw(trace_slot_lane1_raw),
        .trace_slot_lane0_candidate(trace_slot_lane0_candidate),
        .trace_slot_lane1_candidate(trace_slot_lane1_candidate),
        .trace_slot_lane0_aligned(trace_slot_lane0_aligned),
        .trace_slot_lane1_aligned(trace_slot_lane1_aligned),
        .trace_slot_lane0_rotation(trace_slot_lane0_rotation),
        .trace_slot_lane1_rotation(trace_slot_lane1_rotation),
        .trace_slot_bitslip_phase_lane0(trace_slot_bitslip_phase_lane0),
        .trace_slot_bitslip_phase_lane1(trace_slot_bitslip_phase_lane1),
        .trace_slot_sot_hit_lane0(trace_slot_sot_hit_lane0),
        .trace_slot_sot_hit_lane1(trace_slot_sot_hit_lane1)
    );

    initial begin
        idelay_ref_clk = 1'b0;
        forever #2.5 idelay_ref_clk = ~idelay_ref_clk;
    end

    initial begin
        hs_clk_p = 1'b0;
        forever #5 hs_clk_p = ~hs_clk_p;
    end

    assign hs_clk_n = ~hs_clk_p;
    assign data_hs_n = ~data_hs_p;

    function automatic logic [5:0] ref_ecc6(input logic [23:0] data);
        ref_ecc6[0] = data[0]^data[1]^data[2]^data[4]^data[5]^data[7]^data[10]^data[11]^data[13]^data[16]^data[20]^data[21]^data[22]^data[23];
        ref_ecc6[1] = data[0]^data[1]^data[3]^data[4]^data[6]^data[8]^data[10]^data[12]^data[14]^data[17]^data[20]^data[21]^data[22]^data[23];
        ref_ecc6[2] = data[0]^data[2]^data[3]^data[5]^data[6]^data[9]^data[11]^data[12]^data[15]^data[18]^data[20]^data[21]^data[22];
        ref_ecc6[3] = data[1]^data[2]^data[3]^data[7]^data[8]^data[9]^data[13]^data[14]^data[15]^data[19]^data[20]^data[21]^data[23];
        ref_ecc6[4] = data[4]^data[5]^data[6]^data[7]^data[8]^data[9]^data[16]^data[17]^data[18]^data[19]^data[20]^data[22]^data[23];
        ref_ecc6[5] = data[10]^data[11]^data[12]^data[13]^data[14]^data[15]^data[16]^data[17]^data[18]^data[19]^data[21]^data[22]^data[23];
    endfunction

    function automatic logic [63:0] make_stream(
        input logic [2:0] bit_offset,
        input logic [7:0] byte0,
        input logic [7:0] byte1,
        input logic [7:0] byte2,
        input logic [7:0] byte3
    );
        make_stream = 64'h0000_0000_0000_0000;
        make_stream |= 64'(8'hb8) << bit_offset;
        make_stream |= 64'(byte0) << (bit_offset + 6'd8);
        make_stream |= 64'(byte1) << (bit_offset + 6'd16);
        make_stream |= 64'(byte2) << (bit_offset + 6'd24);
        make_stream |= 64'(byte3) << (bit_offset + 6'd32);
    endfunction

    task automatic check_condition(input bit condition, input string message);
        if (!condition) begin
            $fatal(1, "CHECK FAILED: %s", message);
        end
    endtask

    task automatic release_trace_forces();
        dut.trace_capture_done = 1'b0;
        dut.trace_capture_active = 1'b0;
        dut.trace_slot_valid = 8'h00;
        dut.trace_slot_lane0_candidate = '0;
        dut.trace_slot_lane1_candidate = '0;
    endtask

    task automatic write_trace_slot(input int slot, input logic [7:0] lane0_byte, input logic [7:0] lane1_byte);
        unique case (slot)
            0: begin
                dut.trace_slot_lane0_candidate[0] = lane0_byte;
                dut.trace_slot_lane1_candidate[0] = lane1_byte;
            end
            1: begin
                dut.trace_slot_lane0_candidate[1] = lane0_byte;
                dut.trace_slot_lane1_candidate[1] = lane1_byte;
            end
            2: begin
                dut.trace_slot_lane0_candidate[2] = lane0_byte;
                dut.trace_slot_lane1_candidate[2] = lane1_byte;
            end
            3: begin
                dut.trace_slot_lane0_candidate[3] = lane0_byte;
                dut.trace_slot_lane1_candidate[3] = lane1_byte;
            end
            4: begin
                dut.trace_slot_lane0_candidate[4] = lane0_byte;
                dut.trace_slot_lane1_candidate[4] = lane1_byte;
            end
            5: begin
                dut.trace_slot_lane0_candidate[5] = lane0_byte;
                dut.trace_slot_lane1_candidate[5] = lane1_byte;
            end
            6: begin
                dut.trace_slot_lane0_candidate[6] = lane0_byte;
                dut.trace_slot_lane1_candidate[6] = lane1_byte;
            end
            default: begin
                dut.trace_slot_lane0_candidate[7] = lane0_byte;
                dut.trace_slot_lane1_candidate[7] = lane1_byte;
            end
        endcase
    endtask

    task automatic reset_dut();
        release_trace_forces();
        rst_n = 1'b0;
        data_hs_p = 2'b00;
        data_lp_p = 2'b11;
        data_lp_n = 2'b11;
        repeat (8) @(posedge hs_clk_p);
        rst_n = 1'b1;
        repeat (12) @(posedge byte_clk);
    endtask

    task automatic drive_serdes_sample(input logic [7:0] lane0_byte, input logic [7:0] lane1_byte);
        @(negedge byte_clk);
        dut.serdes_byte_sample[0] = lane0_byte;
        dut.serdes_byte_sample[1] = lane1_byte;
        @(posedge byte_clk);
        #1;
    endtask

    task automatic drive_lp_state(
        input logic [1:0] lane_lp_p,
        input logic [1:0] lane_lp_n,
        input int unsigned cycles
    );
        @(negedge byte_clk);
        data_lp_p = lane_lp_p;
        data_lp_n = lane_lp_n;
        repeat (cycles) begin
            @(posedge byte_clk);
        end
        #1;
    endtask

    task automatic wait_for_stream_word(
        input string name,
        input logic [15:0] expected_data,
        input logic expected_sop,
        input logic [7:0] filler0,
        input logic [7:0] filler1
    );
        for (int cycle = 0; cycle < 800; cycle++) begin
            drive_serdes_sample(filler0, filler1);
            if (stream_byte_valid) begin
                check_condition(stream_byte_sop == expected_sop, {name, ": SOP"});
                check_condition(stream_byte_keep == 2'b11, {name, ": keep"});
                check_condition(stream_byte_data == expected_data, {name, ": data"});
                return;
            end
        end
        $display("%s: stream timeout valid=%0b sop=%0b data=%04h sync_valid=%0b score=%0d trace_done=%0b scan_active=%0b buf_active=%0b buf_release=%0b buf_count=%0d pairing=%0d",
            name,
            stream_byte_valid,
            stream_byte_sop,
            stream_byte_data,
            sync_header_valid,
            sync_header_score,
            dut.trace_capture_done,
            dut.sync_scan_active,
            dut.stream_buffer_active,
            dut.stream_buffer_releasing,
            dut.stream_buffer_count,
            dut.stream_buffer_pairing);
        $fatal(1, "%s: timed out waiting for stream word %04h", name, expected_data);
    endtask

    task automatic run_stream_pair0_case(input logic [7:0] ecc1280);
        reset_dut();
        drive_lp_state(2'b00, 2'b00, 4);
        drive_serdes_sample(8'hb8, 8'hb8);
        check_condition(!stream_byte_valid, "pair0 stream does not emit SoT");
        drive_serdes_sample(8'h1e, 8'h00);
        check_condition(!stream_byte_valid, "pair0 first post-SoT beat is buffered");
        drive_serdes_sample(8'h05, ecc1280);
        check_condition(!stream_byte_valid, "pair0 second post-SoT beat is buffered");
        wait_for_stream_word("pair0 first beat", 16'h001e, 1'b1, 8'h11, 8'h22);
        wait_for_stream_word("pair0 second beat", {ecc1280, 8'h05}, 1'b0, 8'h33, 8'h44);
    endtask

    task automatic run_payload_sot_like_bytes_case(input logic [7:0] ecc1280);
        reset_dut();
        drive_lp_state(2'b00, 2'b00, 4);
        drive_serdes_sample(8'hb8, 8'hb8);
        drive_serdes_sample(8'h1e, 8'h00);
        drive_serdes_sample(8'h05, ecc1280);
        drive_serdes_sample(8'hb8, 8'hb8);
        wait_for_stream_word("payload sot-like setup first beat", 16'h001e, 1'b1, 8'h12, 8'h34);
        wait_for_stream_word("payload sot-like setup second beat", {ecc1280, 8'h05}, 1'b0, 8'h56, 8'h78);
        wait_for_stream_word("payload sot-like byte", 16'hb8b8, 1'b0, 8'h9a, 8'hbc);

        drive_serdes_sample(8'h12, 8'h34);
        drive_lp_state(2'b11, 2'b11, 4);
        drive_lp_state(2'b00, 2'b00, 4);
        drive_serdes_sample(8'hb8, 8'hb8);
        check_condition(!stream_byte_valid, "windowed real SoT suppresses sync byte");
        drive_serdes_sample(8'h1e, 8'h00);
        drive_serdes_sample(8'h05, ecc1280);
        wait_for_stream_word("windowed real SoT retriggers", 16'h001e, 1'b1, 8'hde, 8'hf0);
    endtask

    task automatic run_payload_rotated_sot_pattern_case(input logic [7:0] ecc1280);
        reset_dut();
        drive_lp_state(2'b00, 2'b00, 4);
        drive_serdes_sample(8'hb8, 8'hb8);
        drive_serdes_sample(8'h1e, 8'h00);
        drive_serdes_sample(8'h05, ecc1280);
        drive_serdes_sample(8'h71, 8'hc5);
        wait_for_stream_word("rotated sot pattern setup first beat", 16'h001e, 1'b1, 8'h2e, 8'h17);
        wait_for_stream_word("rotated sot pattern setup second beat", {ecc1280, 8'h05}, 1'b0, 8'h8b, 8'he2);
        wait_for_stream_word("rotated sot pattern byte", 16'hc571, 1'b0, 8'h45, 8'h67);
        drive_serdes_sample(8'h2e, 8'h17);
        wait_for_stream_word("second rotated sot pattern beat", 16'h172e, 1'b0, 8'h89, 8'hab);
        drive_serdes_sample(8'h8b, 8'he2);
        wait_for_stream_word("third rotated sot pattern beat", 16'h172e, 1'b0, 8'hcd, 8'hef);
    endtask

    task automatic drive_streams(input logic [63:0] lane0_stream, input logic [63:0] lane1_stream);
        dut.trace_capture_active = 1'b0;
        dut.trace_capture_done = 1'b1;
        dut.trace_slot_valid = 8'hff;
        for (int slot = 0; slot < 8; slot++) begin
            write_trace_slot(
                slot,
                (lane0_stream >> (8 * slot)) & 8'hff,
                (lane1_stream >> (8 * slot)) & 8'hff
            );
        end
        for (int cycle = 0; cycle < 600; cycle++) begin
            @(posedge byte_clk);
            if (sync_header_valid || (!dut.trace_capture_done && !dut.sync_scan_active)) begin
                return;
            end
        end
        $fatal(1, "Timed out waiting for sync-header scan");
    endtask

    task automatic run_valid_case(
        input string name,
        input logic [63:0] lane0_stream,
        input logic [63:0] lane1_stream,
        input logic [7:0] expected_di,
        input logic [15:0] expected_wc,
        input logic [2:0] expected_pairing,
        input logic [2:0] expected_bit_offset_lane0,
        input logic [2:0] expected_bit_offset_lane1
    );
        reset_dut();
        drive_streams(lane0_stream, lane1_stream);
        check_condition(sync_header_valid, {name, ": valid"});
        check_condition(sync_header_di == expected_di, {name, ": DI"});
        check_condition(sync_header_wc == expected_wc, {name, ": WC"});
        check_condition(sync_header_pairing == expected_pairing, {name, ": pairing"});
        check_condition(sync_header_bit_offset_lane0 == expected_bit_offset_lane0, {name, ": lane0 bit offset"});
        check_condition(sync_header_bit_offset_lane1 == expected_bit_offset_lane1, {name, ": lane1 bit offset"});
        check_condition(sync_header_ecc_no_error, {name, ": ECC no-error"});
    endtask

    task automatic run_invalid_case(input string name, input logic [63:0] lane0_stream, input logic [63:0] lane1_stream);
        reset_dut();
        drive_streams(lane0_stream, lane1_stream);
        check_condition(!sync_header_valid, {name, ": invalid"});
        check_condition(sync_header_score == 4'd0, {name, ": zero score"});
    endtask

    task automatic run_nonqualifying_case(input string name, input logic [63:0] lane0_stream, input logic [63:0] lane1_stream);
        reset_dut();
        drive_streams(lane0_stream, lane1_stream);
        check_condition(!sync_header_valid, {name, ": not valid"});
        check_condition(sync_header_score != 4'd0, {name, ": diagnostic score retained"});
        check_condition(sync_header_score < 4'd13, {name, ": below valid threshold"});
    endtask

    initial begin
        automatic logic [7:0] ecc_yuv422_1280;
        automatic logic [7:0] ecc_yuv422_1567;
        ecc_yuv422_1280 = {2'b00, ref_ecc6({16'd1280, 8'h1e})};
        ecc_yuv422_1567 = {2'b00, ref_ecc6({16'd1567, 8'h1e})};

        rst_n = 1'b0;
        run_valid_case(
            "pair0_no_offset",
            make_stream(3'd0, 8'h1e, 8'h05, 8'h00, 8'h00),
            make_stream(3'd0, 8'h00, ecc_yuv422_1280, 8'h00, 8'h00),
            8'h1e,
            16'd1280,
            3'd0,
            3'd0,
            3'd0
        );

        run_valid_case(
            "pair2_lane1_delayed",
            make_stream(3'd0, 8'h1e, 8'h05, 8'h00, 8'h00),
            make_stream(3'd0, 8'ha5, 8'h00, ecc_yuv422_1280, 8'h00),
            8'h1e,
            16'd1280,
            3'd2,
            3'd0,
            3'd0
        );

        run_valid_case(
            "pair0_with_bit_offsets",
            make_stream(3'd3, 8'h1e, 8'h05, 8'h00, 8'h00),
            make_stream(3'd5, 8'h00, ecc_yuv422_1280, 8'h00, 8'h00),
            8'h1e,
            16'd1280,
            3'd0,
            3'd3,
            3'd5
        );

        run_nonqualifying_case(
            "non_exact_wc_rejected",
            make_stream(3'd0, 8'h1e, 8'h06, 8'h00, 8'h00),
            make_stream(3'd0, 8'h1f, ecc_yuv422_1567, 8'h00, 8'h00)
        );

        run_invalid_case(
            "lane0_only_sot_rejected",
            make_stream(3'd0, 8'h1e, 8'h05, 8'h00, 8'h00),
            64'hfedc_ba98_7654_3210
        );

        run_invalid_case(
            "lane1_only_sot_rejected",
            64'h0123_4567_89ab_cdef,
            make_stream(3'd0, 8'h00, ecc_yuv422_1280, 8'h00, 8'h00)
        );

        run_invalid_case("no_sot", 64'h0123_4567_89ab_cdef, 64'hfedc_ba98_7654_3210);

        run_stream_pair0_case(ecc_yuv422_1280);
        run_payload_sot_like_bytes_case(ecc_yuv422_1280);
        run_payload_rotated_sot_pattern_case(ecc_yuv422_1280);

        $display("TEST PASSED: tb_dphy_hs_byte_probe_gearbox");
        $finish;
    end

    initial begin
        #1ms;
        $fatal(1, "Simulation timeout");
    end
endmodule