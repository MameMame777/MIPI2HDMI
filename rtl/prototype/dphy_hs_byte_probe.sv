`timescale 1ns / 1ps
`default_nettype none

module dphy_hs_byte_probe #(
    parameter int LANES = 2,
    parameter int SOT_WINDOW_BYTES = 64,
    parameter int SWEEP_HOLD_BYTES = 16384,
    parameter bit SWEEP_ENABLE = 1'b1,
    parameter int FIXED_BITSLIP_PHASE = 0,
    parameter int FIXED_BITSLIP_PHASE_LANE1 = 0,
    parameter bit LANE1_BITSLIP_SWEEP_ENABLE = 1'b0,
    parameter int FIXED_TRANSFORM = 0,
    parameter int TRACE_TRIGGER_MODE = 0,
    parameter int IDELAY_TAP = 0,
    parameter real IDELAY_REFCLK_MHZ = 200.0,
    parameter logic [7:0] EXPECTED_LONG_DT = 8'h1e,
    parameter logic [15:0] EXPECTED_LONG_WC = 16'd1280,
    parameter int MIN_SYNC_HEADER_SCORE = 13,
    parameter bit SYNC_HEADER_SWEEP_BIT_OFFSETS = 1'b1,
    parameter bit SYNC_HEADER_USE_ALIGNED_STREAM = 1'b0,
    parameter int STREAM_DESKEW_DEPTH = (SYNC_HEADER_SWEEP_BIT_OFFSETS ? 512 : 32),
    parameter int STREAM_PAIRING = 0
) (
    input  wire                  rst_n,
    input  wire                  idelay_ref_clk,
    input  wire                  idelay_ref_reset,
    input  wire [4:0]            runtime_idelay_tap,
    input  wire [4:0]            runtime_idelay_tap_lane1,
    input  wire [4:0]            runtime_idelay_tap_clk = 5'd0,  // clock-lane IDELAY tap (cal)
    input  wire                  rt_bufr_clr = 1'b0,             // runtime BUFR.CLR re-roll (sysclk level)
    input  wire [2:0]            runtime_bitslip_phase,
    input  wire [2:0]            runtime_bitslip_phase_lane1,
    input  wire                  runtime_lane1_sweep_enable,
    input  wire [7:0]            runtime_expected_long_dt,
    // D-PHY lane supervisor control (opt-in; see dphy_lane_supervisor.sv).
    // sup_enable is the byte_clk-synced enable; when low every supervisor
    // input is ignored and the probe behaves exactly as the legacy bitstream.
    input  wire                  sup_enable,        // byte_clk domain (2FF synced upstream)
    input  wire                  sup_bufr_clr,      // ctl_clk domain  (BUFR async CLR)
    input  wire                  sup_serdes_rst,    // byte_clk domain (ISERDES RST)
    input  wire                  sup_hs_settled,    // byte_clk domain (HS-SETTLE gate)
    // hwlock_bufr_clr (2026-06-19): /4 re-roll from the HW deterministic-lock FSM
    // (dphy_hwlock_fsm, refclk_200/ctl_clk domain). Same domain + purpose as
    // rt_bufr_clr_ctl (idelay_ref_clk = refclk_200), so it is ORed into the same
    // bufr_reroll net (BUFR.CLR + ISERDES re-roll). Default 0 = unused.
    input  wire                  hwlock_bufr_clr = 1'b0,  // refclk_200 (ctl_clk) domain
    // cfg_hs_settle_gate (2026-06-17): apply the per-line HS-SETTLE SoT gate in
    // the LEGACY continuous path too, DECOUPLED from sup_enable (so the BUFR.CLR
    // gating that breaks continuous lock stays off). sup_hs_settled is valid even
    // when sup_enable=0 (the supervisor FSM runs on ctl_clk regardless; verified
    // 2026-06-17: per-line settle ~6426/s). Gates the SoT search past the
    // HS-prepare/settle garbage at each burst head -> recovers the >=16 line/frame
    // drop. Default 0 = exact legacy behaviour.
    input  wire                  cfg_hs_settle_gate = 1'b0,  // byte_clk domain
    // cfg_settle_blank_k (2026-06-17): byte_clk-domain per-line settle blank. After
    // a data-lane LP-exit, hold the SoT window CLOSED for K byte_clk cycles so the
    // SoT search skips the HS-prepare/settle garbage at the burst head. This is the
    // correctly-timed (byte-domain) version of cfg_hs_settle_gate (sup_hs_settled was
    // ctl_clk-domain -> too late, blocked real SoTs). 0 = off (legacy: open at LP-exit).
    input  wire [3:0]            cfg_settle_blank_k = 4'd0,  // byte_clk domain
    input  wire                  dphy_hs_clock_clk_p,
    input  wire                  dphy_hs_clock_clk_n,
    input  wire [LANES-1:0]      dphy_data_hs_p,
    input  wire [LANES-1:0]      dphy_data_hs_n,
    input  wire [LANES-1:0]      dphy_data_lp_p,
    input  wire [LANES-1:0]      dphy_data_lp_n,
    output logic                 byte_clk,
    output logic                 idelayctrl_rdy,
    output logic                 hs_clk_seen,
    output logic [LANES-1:0]     lane_sot_seen,
    output logic [LANES-1:0][7:0] lane_last_byte,
    output logic [LANES-1:0]     lane_raw_changed_seen,
    output logic [LANES-1:0]     lane_raw_non_ff_seen,
    output logic [LANES-1:0]     lane_raw_non_00_seen,
    output logic [LANES-1:0][7:0] lane_raw_change_count,
    output logic [LANES*8-1:0]   stream_byte_data,
    output logic [LANES-1:0]     stream_byte_keep,
    output logic                 stream_byte_valid,
    output logic                 stream_byte_sop,
    output logic                 stream_byte_eop,
    output logic [2:0]           stream_pairing_active_dbg,
    output logic [2:0]           stream_pairing_next_dbg,
    output logic                 header_valid,
    output logic [7:0]           header_di,
    output logic [15:0]          header_wc,
    output logic [7:0]           header_ecc,
    output logic                 sync_header_valid,
    output logic [7:0]           sync_header_di,
    output logic [15:0]          sync_header_wc,
    output logic [7:0]           sync_header_ecc,
    output logic [2:0]           sync_header_rotation_lane0,
    output logic [2:0]           sync_header_rotation_lane1,
    output logic [2:0]           sync_header_bit_offset_lane0,
    output logic [2:0]           sync_header_bit_offset_lane1,
    output logic [3:0]           sync_header_score,
    output logic [2:0]           sync_header_start_slot,
    output logic [2:0]           sync_header_pairing,
    output logic [5:0]           sync_header_syndrome,
    output logic                 sync_header_ecc_no_error,
    output logic                 sync_header_ecc_corrected,
    output logic                 sync_header_ecc_uncorrectable,
    output logic [7:0]           header_slot_valid,
    output logic [7:0][7:0]      header_slot_di,
    output logic [7:0][15:0]     header_slot_wc,
    output logic [7:0][7:0]      header_slot_ecc,
    output logic [7:0][2:0]      header_slot_bitslip_phase,
    output logic [7:0][2:0]      header_slot_bitslip_phase_lane1,
    output logic [7:0][2:0]      header_slot_transform,
    output logic [7:0][2:0]      header_slot_rotation,
    output logic [7:0][7:0]      header_slot_corr_di,
    output logic [7:0][15:0]     header_slot_corr_wc,
    output logic [7:0][5:0]      header_slot_syndrome,
    output logic [7:0]           header_slot_ecc_no_error,
    output logic [7:0]           header_slot_ecc_corrected,
    output logic [7:0]           header_slot_ecc_uncorrectable,
    output logic [7:0]           trace_slot_valid,
    output logic [7:0][7:0]      trace_slot_lane0_raw,
    output logic [7:0][7:0]      trace_slot_lane1_raw,
    output logic [7:0][7:0]      trace_slot_lane0_candidate,
    output logic [7:0][7:0]      trace_slot_lane1_candidate,
    output logic [7:0][7:0]      trace_slot_lane0_aligned,
    output logic [7:0][7:0]      trace_slot_lane1_aligned,
    output logic [7:0][2:0]      trace_slot_lane0_rotation,
    output logic [7:0][2:0]      trace_slot_lane1_rotation,
    output logic [7:0][2:0]      trace_slot_bitslip_phase_lane0,
    output logic [7:0][2:0]      trace_slot_bitslip_phase_lane1,
    output logic [7:0]           trace_slot_sot_hit_lane0,
    output logic [7:0]           trace_slot_sot_hit_lane1,
    output logic [7:0]           live_trace_seq,
    output logic [7:0]           live_trace_slot_valid,
    output logic [7:0][7:0]      live_trace_slot_lane0_raw,
    output logic [7:0][7:0]      live_trace_slot_lane1_raw,
    output logic [7:0][7:0]      live_trace_slot_lane0_candidate,
    output logic [7:0][7:0]      live_trace_slot_lane1_candidate,
    output logic [7:0][7:0]      live_trace_slot_lane0_aligned,
    output logic [7:0][7:0]      live_trace_slot_lane1_aligned,
    output logic [7:0]           live_trace_slot_sot_hit_lane0,
    output logic [7:0]           live_trace_slot_sot_hit_lane1,
    output logic [7:0][2:0]      live_trace_slot_lane0_rotation,
    output logic [7:0][2:0]      live_trace_slot_lane1_rotation,
    output logic [2:0]           lane1_target_phase_out,
    // Raw serdes byte sample exposure (post-ISERDES, pre-decode) for ring buffer
    output logic [LANES-1:0][7:0] serdes_byte_sample_out,
    // SoT-miss diagnostics (2026-06-17, lane 0): bursts = data-lane LP-exit edges
    // (~one per chip line); sot_bursts = bursts in which the stream-path SoT
    // (current_sot_in_window) fired. burst-sot_burst ~= the per-frame line shortfall
    // means SoT-DETECTION miss; if equal but last_fe still short -> header/sync miss.
    // missed_burst = the first 4 lane-0 candidate bytes of the last no-SoT burst
    // (SW checks for a 0xB8 rotation to see if the SoT was present but undetected).
    output logic [15:0]          dbg_burst_count,
    output logic [15:0]          dbg_sot_burst_count,
    output logic [31:0]          dbg_missed_burst,
    // Vblank-exit re-lock latency (2026-06-18, gated branch): byte_clk cycles from
    // the ISERDES-reset RELEASE (clock/byte_clk just restarted after a clock-lane
    // gate) to the FIRST accepted SoT. In gated (0x34) the clock gates per-frame at
    // vblank, so this is the per-frame vblank-exit re-acquire time = the suspected
    // root of the FS-loss + top-line loss. {max[31:16], last[15:0]} byte_clk cycles
    // (~640/line). 0 in continuous (no per-gate ISERDES reset).
    output logic [15:0]          dbg_relock_latency,
    output logic [15:0]          dbg_relock_max
);

    localparam logic [7:0] SOT_SYNC = 8'hb8;
    localparam int FOCUS_HEADER_START_SLOT = 1;
    localparam int FOCUS_HEADER_PAIRING = 2;
    localparam int SYNC_HEADER_BIT_OFFSET_COUNT = SYNC_HEADER_SWEEP_BIT_OFFSETS ? 8 : 1;
    localparam logic [2:0] SYNC_HEADER_LAST_BIT_OFFSET = SYNC_HEADER_BIT_OFFSET_COUNT - 1;
    localparam int STREAM_DESKEW_ADDR_W = $clog2(STREAM_DESKEW_DEPTH);
    localparam logic [STREAM_DESKEW_ADDR_W:0] STREAM_DESKEW_COUNT_MAX = STREAM_DESKEW_DEPTH;
    localparam logic [STREAM_DESKEW_ADDR_W:0] STREAM_DESKEW_COUNT_ONE = {{STREAM_DESKEW_ADDR_W{1'b0}}, 1'b1};
    localparam logic [STREAM_DESKEW_ADDR_W:0] STREAM_DESKEW_COUNT_TWO = {{(STREAM_DESKEW_ADDR_W-1){1'b0}}, 2'd2};
    localparam int WINDOW_COUNT_W = $clog2(SOT_WINDOW_BYTES + 1);
    localparam int SWEEP_COUNT_W = $clog2(SWEEP_HOLD_BYTES + 1);

    logic hs_clk_ibuf;
    logic hs_clk_io;
    logic [LANES-1:0] data_ibuf;
    logic [LANES-1:0] data_delay;
    logic [LANES-1:0][7:0] serdes_byte;
    logic [LANES-1:0][7:0] serdes_byte_sample;
    logic [LANES-1:0][1:0] lp_sync1;
    logic [LANES-1:0][1:0] lp_sync2;
    logic [LANES-1:0] lp11_prev;
    logic sup_hs_settled_prev;          // supervisor HS-SETTLE rising-edge detect
    logic [LANES-1:0] sot_window_active;
    logic [LANES-1:0][WINDOW_COUNT_W-1:0] sot_window_count;
    logic [LANES-1:0][3:0] sot_blank_count;     // settle-blank countdown (byte_clk) per lane
    logic        sot_found_burst;               // lane0: a stream SoT fired this burst
    logic [2:0]  burst_cap_idx;                 // lane0: burst-head byte capture index
    logic [7:0]  burst_cap [0:3];               // lane0: first 4 candidate bytes of this burst
    logic        relock_pending;                // vblank-exit re-lock latency: counting
    logic [15:0] relock_cnt;                    // byte_clk since ISERDES-reset release
    logic [LANES-1:0] lane_locked;
    logic [LANES-1:0][2:0] lane_rotation;
    logic [LANES-1:0][2:0] lane_header_count;
    logic [7:0] hs_clk_seen_count;
    logic [2:0] header_write_index;
    logic header_commit_pending;
    logic [2:0] header_pending_bitslip_phase;
    logic [2:0] header_pending_bitslip_phase_lane1;
    logic [2:0] header_pending_transform;
    logic [2:0] header_pending_rotation;
    logic [LANES-1:0][2:0] lane_bitslip_phase;
    logic [2:0] lane1_target_bitslip_phase;
    assign lane1_target_phase_out = lane1_target_bitslip_phase;
    logic [2:0] sweep_transform;
    logic [SWEEP_COUNT_W-1:0] sweep_hold_count;
    logic [LANES-1:0] sweep_bitslip_pulse;
    logic [LANES-1:0][3:0] fixed_bitslip_wait;
    logic [2:0] active_transform;
    logic trace_capture_active;
    logic trace_capture_done;
    logic [2:0] trace_write_index;
    logic live_trace_capture_active;
    logic [2:0] live_trace_write_index;
    logic [LANES-1:0][7:0] lane_raw_prev;
    logic sync_header_active;
    logic [1:0] sync_header_count;
    logic [LANES-1:0][7:0] sync_header_candidate_1;
    logic [LANES-1:0][7:0] sync_header_candidate_2;
    logic stream_started;
    logic stream_sop_pending;
    logic stream_prev_valid;
    logic [2:0] stream_pairing_active;
    logic [2:0] stream_pairing_next;
    logic [LANES-1:0][7:0] stream_prev_aligned_byte;
    logic stream_buffer_active;
    logic stream_buffer_releasing;
    logic [2:0] stream_buffer_pairing;
    logic [STREAM_DESKEW_ADDR_W-1:0] stream_buffer_wr_ptr;
    logic [STREAM_DESKEW_ADDR_W-1:0] stream_buffer_rd_ptr;
    logic [STREAM_DESKEW_ADDR_W:0] stream_buffer_count;
    logic [STREAM_DESKEW_DEPTH-1:0][LANES-1:0][7:0] stream_buffer_data;
    logic stream_buffer_overflow_seen;
    logic sync_scan_active;
    logic [2:0] sync_scan_bit_offset_lane0;
    logic [2:0] sync_scan_bit_offset_lane1;
    logic [2:0] sync_scan_pairing;
    logic [63:0] sync_scan_lane0_stream;
    logic [63:0] sync_scan_lane1_stream;
    logic [3:0] sync_scan_best_score;
    logic [7:0] sync_scan_best_di;
    logic [15:0] sync_scan_best_wc;
    logic [7:0] sync_scan_best_ecc;
    logic [5:0] sync_scan_best_syndrome;
    logic [2:0] sync_scan_best_pairing;
    logic [2:0] sync_scan_best_bit_offset_lane0;
    logic [2:0] sync_scan_best_bit_offset_lane1;
    logic sync_scan_best_no_error;
    logic sync_scan_best_corrected;
    logic sync_scan_best_uncorrectable;

    always_comb begin
        active_transform = SWEEP_ENABLE ? sweep_transform : FIXED_TRANSFORM[2:0];
        stream_pairing_active_dbg = stream_pairing_active;
        stream_pairing_next_dbg = stream_pairing_next;
    end

    function automatic logic [7:0] rotate_right8(input logic [7:0] value, input logic [2:0] amount);
        automatic logic [7:0] result;
        result = value;
        for (int idx = 0; idx < amount; idx++) begin
            result = {result[0], result[7:1]};
        end
        rotate_right8 = result;
    endfunction

    function automatic logic [7:0] window_forward8(
        input logic [7:0] this_byte,
        input logic [7:0] next_byte,
        input logic [2:0] bit_offset
    );
        automatic logic [15:0] window_bits;
        window_bits = {next_byte, this_byte};
        window_forward8 = (window_bits >> bit_offset) & 8'hff;
    endfunction

    function automatic logic [7:0] trace_candidate_window(
        input logic lane,
        input logic [2:0] slot,
        input logic [2:0] bit_offset
    );
        if (lane) begin
            trace_candidate_window = window_forward8(trace_slot_lane1_candidate[slot], trace_slot_lane1_candidate[slot + 3'd1], bit_offset);
        end else begin
            trace_candidate_window = window_forward8(trace_slot_lane0_candidate[slot], trace_slot_lane0_candidate[slot + 3'd1], bit_offset);
        end
    endfunction

    function automatic logic [7:0] stream_byte_at(
        input logic [63:0] stream_bits,
        input int bit_index
    );
        automatic logic [63:0] shifted_stream;
        shifted_stream = stream_bits >> bit_index;
        stream_byte_at = shifted_stream[7:0];
    endfunction

    function automatic logic [STREAM_DESKEW_ADDR_W-1:0] stream_ptr_inc(
        input logic [STREAM_DESKEW_ADDR_W-1:0] ptr
    );
        if (ptr == STREAM_DESKEW_DEPTH - 1) begin
            stream_ptr_inc = '0;
        end else begin
            stream_ptr_inc = ptr + {{(STREAM_DESKEW_ADDR_W-1){1'b0}}, 1'b1};
        end
    endfunction

    function automatic logic [STREAM_DESKEW_ADDR_W-1:0] stream_ptr_add(
        input logic [STREAM_DESKEW_ADDR_W-1:0] ptr,
        input int unsigned increment
    );
        automatic int unsigned sum;
        sum = ptr + increment;
        while (sum >= STREAM_DESKEW_DEPTH) begin
            sum = sum - STREAM_DESKEW_DEPTH;
        end
        stream_ptr_add = sum[STREAM_DESKEW_ADDR_W-1:0];
    endfunction

    function automatic logic stream_pairing_uses_next_slot(input logic [2:0] pairing);
        stream_pairing_uses_next_slot = (pairing >= 3'd2);
    endfunction

    function automatic logic [LANES*8-1:0] select_stream_word(
        input logic [2:0] pairing,
        input logic [LANES-1:0][7:0] base_bytes,
        input logic [LANES-1:0][7:0] next_bytes
    );
        select_stream_word = '0;
        if (LANES == 2) begin
            unique case (pairing)
                3'd0: begin
                    select_stream_word[7:0] = base_bytes[0];
                    select_stream_word[15:8] = base_bytes[1];
                end
                3'd1: begin
                    select_stream_word[7:0] = base_bytes[1];
                    select_stream_word[15:8] = base_bytes[0];
                end
                3'd2: begin
                    select_stream_word[7:0] = base_bytes[0];
                    select_stream_word[15:8] = next_bytes[1];
                end
                3'd3: begin
                    select_stream_word[7:0] = next_bytes[0];
                    select_stream_word[15:8] = base_bytes[1];
                end
                3'd4: begin
                    select_stream_word[7:0] = base_bytes[1];
                    select_stream_word[15:8] = next_bytes[0];
                end
                default: begin
                    select_stream_word[7:0] = next_bytes[1];
                    select_stream_word[15:8] = base_bytes[0];
                end
            endcase
        end else begin
            for (int lane = 0; lane < LANES; lane++) begin
                select_stream_word[(lane * 8) +: 8] = base_bytes[lane];
            end
        end
    endfunction

    function automatic logic [7:0] reverse8(input logic [7:0] value);
        for (int idx = 0; idx < 8; idx++) begin
            reverse8[idx] = value[7 - idx];
        end
    endfunction

    function automatic logic [7:0] transform_byte(input logic [7:0] value, input logic [2:0] transform);
        automatic logic [7:0] result;
        result = transform[0] ? reverse8(value) : value;
        if (transform[1]) begin
            result = ~result;
        end
        transform_byte = result;
    endfunction

    function automatic logic [5:0] calc_ecc6(input logic [23:0] data);
        calc_ecc6[0] = data[0]^data[1]^data[2]^data[4]^data[5]^data[7]^data[10]^data[11]^data[13]^data[16]^data[20]^data[21]^data[22]^data[23];
        calc_ecc6[1] = data[0]^data[1]^data[3]^data[4]^data[6]^data[8]^data[10]^data[12]^data[14]^data[17]^data[20]^data[21]^data[22]^data[23];
        calc_ecc6[2] = data[0]^data[2]^data[3]^data[5]^data[6]^data[9]^data[11]^data[12]^data[15]^data[18]^data[20]^data[21]^data[22];
        calc_ecc6[3] = data[1]^data[2]^data[3]^data[7]^data[8]^data[9]^data[13]^data[14]^data[15]^data[19]^data[20]^data[21]^data[23];
        calc_ecc6[4] = data[4]^data[5]^data[6]^data[7]^data[8]^data[9]^data[16]^data[17]^data[18]^data[19]^data[20]^data[22]^data[23];
        calc_ecc6[5] = data[10]^data[11]^data[12]^data[13]^data[14]^data[15]^data[16]^data[17]^data[18]^data[19]^data[21]^data[22]^data[23];
    endfunction

    function automatic logic [5:0] bit_syndrome(input int bit_idx);
        automatic logic [23:0] onehot;
        onehot = 24'h000000;
        onehot[bit_idx] = 1'b1;
        bit_syndrome = calc_ecc6(onehot);
    endfunction

    function automatic int decode_data_bit(input logic [5:0] syndrome);
        decode_data_bit = -1;
        for (int idx = 0; idx < 24; idx++) begin
            if (syndrome == bit_syndrome(idx)) begin
                decode_data_bit = idx;
            end
        end
    endfunction

    function automatic logic is_onehot6(input logic [5:0] value);
        is_onehot6 = (value != 6'b000000) && ((value & (value - 6'd1)) == 6'b000000);
    endfunction

    function automatic logic is_expected_dt(input logic [7:0] di);
        unique case (di[5:0])
            6'h00, 6'h01, 6'h02, 6'h03, 6'h1e, 6'h1f, 6'h2a, 6'h2b: is_expected_dt = 1'b1;
            default: is_expected_dt = 1'b0;
        endcase
    endfunction
    function automatic logic is_frame_short_packet(input logic [7:0] di);
        unique case (di[5:0])
            6'h00, 6'h01, 6'h02, 6'h03: is_frame_short_packet = 1'b1;
            default: is_frame_short_packet = 1'b0;
        endcase
    endfunction

    function automatic logic is_plausible_wc(input logic [7:0] di, input logic [15:0] wc);
        unique case (di[5:0])
            6'h00, 6'h01, 6'h02, 6'h03: is_plausible_wc = 1'b1;
            6'h1e, 6'h1f, 6'h2a, 6'h2b: is_plausible_wc = (wc == 16'd1280) || ((wc >= 16'd640) && (wc <= 16'd4096));
            default: is_plausible_wc = 1'b0;
        endcase
    endfunction

    // Runtime-configurable expected DT. Synchronize from sysclk domain to byte_clk.
    // If non-zero, use the synchronized value; else fall back to build-time parameter.
    (* ASYNC_REG = "TRUE" *) logic [7:0] runtime_expected_long_dt_meta;
    (* ASYNC_REG = "TRUE" *) logic [7:0] runtime_expected_long_dt_sync;
    always_ff @(posedge byte_clk) begin
        runtime_expected_long_dt_meta <= runtime_expected_long_dt;
        runtime_expected_long_dt_sync <= runtime_expected_long_dt_meta;
    end
    wire [7:0] active_expected_long_dt = (runtime_expected_long_dt_sync != 8'h00) ? runtime_expected_long_dt_sync : EXPECTED_LONG_DT;

    function automatic logic is_expected_long_packet(input logic [7:0] di, input logic [15:0] wc);
        is_expected_long_packet = (di[5:0] == active_expected_long_dt[5:0]) && (wc == EXPECTED_LONG_WC);
    endfunction

    function automatic logic [3:0] header_candidate_score(
        input logic no_error,
        input logic corrected,
        input logic [7:0] di,
        input logic [15:0] wc
    );
        header_candidate_score = 4'd0;
        if (is_expected_long_packet(di, wc)) begin
            if (no_error) begin
                header_candidate_score = 4'd15;
            end else if (corrected) begin
                header_candidate_score = 4'd13;
            end else begin
                header_candidate_score = 4'd4;
            end
            return header_candidate_score;
        end
        if (is_frame_short_packet(di)) begin
            if (no_error) begin
                header_candidate_score = 4'd13;
            end else if (corrected) begin
                header_candidate_score = 4'd10;
            end
            return header_candidate_score;
        end
        unique case (di[5:0])
            6'h1e, 6'h1f, 6'h2a, 6'h2b: begin
                if (is_plausible_wc(di, wc)) begin
                    if (no_error) begin
                        header_candidate_score = 4'd6;
                    end else if (corrected) begin
                        header_candidate_score = 4'd3;
                    end else begin
                        header_candidate_score = 4'd4;
                    end
                end else begin
                    if (no_error) begin
                        header_candidate_score = 4'd6;
                    end else if (corrected) begin
                        header_candidate_score = 4'd3;
                    end else begin
                        header_candidate_score = 4'd1;
                    end
                end
            end
            default: begin
                header_candidate_score = 4'd0;
            end
        endcase
        if ((header_candidate_score == 4'd0) && is_expected_dt(di) && no_error) begin
            if (no_error) begin
                header_candidate_score = 4'd1;
            end
        end
    endfunction

    function automatic logic has_sot_rotation(input logic [7:0] value);
        has_sot_rotation = 1'b0;
        for (int idx = 0; idx < 8; idx++) begin
            if (rotate_right8(value, idx[2:0]) == SOT_SYNC) begin
                has_sot_rotation = 1'b1;
            end
        end
    endfunction

    function automatic logic [2:0] find_sot_rotation(input logic [7:0] value);
        find_sot_rotation = 3'd0;
        for (int idx = 0; idx < 8; idx++) begin
            if (rotate_right8(value, idx[2:0]) == SOT_SYNC) begin
                find_sot_rotation = idx[2:0];
            end
        end
    endfunction

    IBUFDS #(
        .DIFF_TERM("FALSE"),
        .IBUF_LOW_PWR("TRUE"),
        .IOSTANDARD("LVDS_25")
    ) u_hs_clk_ibufds (
        .I (dphy_hs_clock_clk_p),
        .IB(dphy_hs_clock_clk_n),
        .O (hs_clk_ibuf)
    );

    // Clock-lane IDELAYE2 REMOVED (2026-06-15): an IDELAYE2 on the HS clock ->
    // BUFIO/BUFR degraded byte_clk quality so the link no longer SUSTAINED a
    // lock (0/6 continuous-legacy downloads, full bitslip x clk scan sync=0),
    // vs ~2/8 sustained without it. The clock goes straight to BUFIO/BUFR again.
    // The byte-phase determinism instead comes from the BUFR.CLR re-roll below
    // (the baseline sustains ~1/4 of /4 phases; software re-rolls in us until a
    // sustaining phase, then holds). runtime_idelay_tap_clk is now unused.

    // Runtime BUFR.CLR re-roll (2026-06-15): software toggles rt_bufr_clr to
    // re-roll the BUFR /4 byte phase in us (vs a 12 s re-download), so the
    // calibration can hunt for a SUSTAINING /4 phase and then hold it. 2FF synced
    // to idelay_ref_clk (always running) so the release edge is glitch-free; the
    // release point is intentionally async to the HS clock so it lands on a
    // fresh /4 phase.
    (* ASYNC_REG = "TRUE" *) logic rt_bufr_clr_meta, rt_bufr_clr_ctl;
    always_ff @(posedge idelay_ref_clk or negedge rst_n) begin
        if (!rst_n) begin
            rt_bufr_clr_meta <= 1'b0;
            rt_bufr_clr_ctl  <= 1'b0;
        end else begin
            rt_bufr_clr_meta <= rt_bufr_clr;
            rt_bufr_clr_ctl  <= rt_bufr_clr_meta;
        end
    end

    BUFIO u_hs_clk_bufio (
        .I(hs_clk_ibuf),
        .O(hs_clk_io)
    );

    // BUFR /4 divider. When the supervisor is enabled its bufr_clr realigns the
    // divider phase deterministically at every clock-lane restart (the missing
    // mechanism behind the 3% capture rate; diary 2026-06-12). rt_bufr_clr_ctl
    // adds a software-driven re-roll for the calibration phase search.
    // rt_bufr_clr_ctl (software re-roll, idelay_ref_clk-synced) and hwlock_bufr_clr
    // (HW lock FSM re-roll, refclk_200 = idelay_ref_clk) are the same domain and the
    // same purpose: combine into one re-roll net for the BUFR.CLR + ISERDES reset.
    wire bufr_reroll = rt_bufr_clr_ctl || hwlock_bufr_clr;

    BUFR #(
        .BUFR_DIVIDE("4"),
        .SIM_DEVICE("7SERIES")
    ) u_hs_clk_bufr (
        .I(hs_clk_ibuf),
        .CE(1'b1),
        .CLR(!rst_n || (sup_enable && sup_bufr_clr) || bufr_reroll),
        .O(byte_clk)
    );

    // Re-roll ISERDES reset (2026-06-15): when rt_bufr_clr re-rolls the BUFR /4
    // phase, byte_clk (the ISERDES CLKDIV) restarts on a new phase, so the
    // ISERDES must be reset and released SYNCHRONOUS to the restored CLKDIV
    // (Xilinx requirement) -- exactly what the supervisor does on its per-gate
    // re-lock. Without this the deserializer stays wedged after a re-roll
    // (observed: sync_header_valid -> 0 after the first re-roll). Inlined
    // async-assert / sync-release reset bridge (kept self-contained so the
    // standalone probe testbench needs no extra sources).
    (* ASYNC_REG = "TRUE" *) logic rt_serdes_rst_meta, rt_serdes_rst;
    always_ff @(posedge byte_clk or posedge bufr_reroll) begin
        if (bufr_reroll) begin
            rt_serdes_rst_meta <= 1'b1;
            rt_serdes_rst      <= 1'b1;
        end else begin
            rt_serdes_rst_meta <= 1'b0;
            rt_serdes_rst      <= rt_serdes_rst_meta;
        end
    end

    (* IODELAY_GROUP = "mipi_dphy_idelay" *)
    IDELAYCTRL u_idelayctrl (
        .REFCLK(idelay_ref_clk),
        .RST(idelay_ref_reset || !rst_n),
        .RDY(idelayctrl_rdy)
    );

    logic [4:0] runtime_idelay_sync1;
    logic [4:0] runtime_idelay_sync2;
    logic [4:0] runtime_idelay_prev;
    logic       runtime_idelay_load;
    logic       runtime_idelay_first_load;
    logic [4:0] runtime_idelay_lane1_sync1;
    logic [4:0] runtime_idelay_lane1_sync2;
    logic [4:0] runtime_idelay_lane1_prev;
    logic       runtime_idelay_lane1_load;
    logic       runtime_idelay_lane1_first_load;

    always_ff @(posedge byte_clk or negedge rst_n) begin
        if (!rst_n) begin
            runtime_idelay_sync1      <= IDELAY_TAP[4:0];
            runtime_idelay_sync2      <= IDELAY_TAP[4:0];
            runtime_idelay_prev       <= 5'h1f;
            runtime_idelay_first_load <= 1'b1;
            runtime_idelay_lane1_sync1      <= IDELAY_TAP[4:0];
            runtime_idelay_lane1_sync2      <= IDELAY_TAP[4:0];
            runtime_idelay_lane1_prev       <= 5'h1f;
            runtime_idelay_lane1_first_load <= 1'b1;
        end else begin
            runtime_idelay_sync1      <= runtime_idelay_tap;
            runtime_idelay_sync2      <= runtime_idelay_sync1;
            runtime_idelay_prev       <= runtime_idelay_sync2;
            runtime_idelay_first_load <= 1'b0;
            runtime_idelay_lane1_sync1      <= runtime_idelay_tap_lane1;
            runtime_idelay_lane1_sync2      <= runtime_idelay_lane1_sync1;
            runtime_idelay_lane1_prev       <= runtime_idelay_lane1_sync2;
            runtime_idelay_lane1_first_load <= 1'b0;
        end
    end

    assign runtime_idelay_load =
        (runtime_idelay_sync2 != runtime_idelay_prev) || runtime_idelay_first_load;
    assign runtime_idelay_lane1_load =
        (runtime_idelay_lane1_sync2 != runtime_idelay_lane1_prev) || runtime_idelay_lane1_first_load;

    // Bitslip phase CDC. The bitslip retrain loop below is comparison-based, so we just
    // need a stable byte_clk-domain copy of the runtime target phases. The reset value
    // matches the FIXED_BITSLIP_PHASE* parameter so behaviour at power-up matches the
    // pre-runtime-knob bitstream until the PS overrides via apply strobe.
    logic [2:0] runtime_bitslip_phase_sync1;
    logic [2:0] runtime_bitslip_phase_sync2;
    logic [2:0] runtime_bitslip_phase_lane1_sync1;
    logic [2:0] runtime_bitslip_phase_lane1_sync2;

    always_ff @(posedge byte_clk or negedge rst_n) begin
        if (!rst_n) begin
            runtime_bitslip_phase_sync1       <= FIXED_BITSLIP_PHASE[2:0];
            runtime_bitslip_phase_sync2       <= FIXED_BITSLIP_PHASE[2:0];
            runtime_bitslip_phase_lane1_sync1 <= FIXED_BITSLIP_PHASE_LANE1[2:0];
            runtime_bitslip_phase_lane1_sync2 <= FIXED_BITSLIP_PHASE_LANE1[2:0];
        end else begin
            runtime_bitslip_phase_sync1       <= runtime_bitslip_phase;
            runtime_bitslip_phase_sync2       <= runtime_bitslip_phase_sync1;
            runtime_bitslip_phase_lane1_sync1 <= runtime_bitslip_phase_lane1;
            runtime_bitslip_phase_lane1_sync2 <= runtime_bitslip_phase_lane1_sync1;
        end
    end

    for (genvar lane = 0; lane < LANES; lane++) begin : gen_lane_probe
        logic q1;
        logic q2;
        logic q3;
        logic q4;
        logic q5;
        logic q6;
        logic q7;
        logic q8;

        IBUFDS #(
            .DIFF_TERM("FALSE"),
            .IBUF_LOW_PWR("TRUE"),
            .IOSTANDARD("LVDS_25")
        ) u_data_ibufds (
            .I (dphy_data_hs_p[lane]),
            .IB(dphy_data_hs_n[lane]),
            .O (data_ibuf[lane])
        );

        (* IODELAY_GROUP = "mipi_dphy_idelay" *)
        IDELAYE2 #(
            .CINVCTRL_SEL("FALSE"),
            .DELAY_SRC("IDATAIN"),
            .HIGH_PERFORMANCE_MODE("TRUE"),
            .IDELAY_TYPE("VAR_LOAD"),
            .IDELAY_VALUE(IDELAY_TAP),
            .PIPE_SEL("FALSE"),
            .REFCLK_FREQUENCY(IDELAY_REFCLK_MHZ),
            .SIGNAL_PATTERN("DATA")
        ) u_data_idelay (
            .C(byte_clk),
            .REGRST(!rst_n),
            .LD(lane == 0 ? runtime_idelay_load : runtime_idelay_lane1_load),
            .CE(1'b0),
            .INC(1'b0),
            .LDPIPEEN(1'b0),
            .CINVCTRL(1'b0),
            .CNTVALUEIN(lane == 0 ? runtime_idelay_sync2 : runtime_idelay_lane1_sync2),
            .IDATAIN(data_ibuf[lane]),
            .DATAIN(1'b0),
            .DATAOUT(data_delay[lane]),
            .CNTVALUEOUT()
        );

        ISERDESE2 #(
            .DATA_RATE("DDR"),
            .DATA_WIDTH(8),
            .DYN_CLKDIV_INV_EN("FALSE"),
            .DYN_CLK_INV_EN("FALSE"),
            .INTERFACE_TYPE("NETWORKING"),
            .IOBDELAY("IFD"),
            .NUM_CE(1),
            .OFB_USED("FALSE"),
            .SERDES_MODE("MASTER")
        ) u_data_iserdes (
            .Q1(q1),
            .Q2(q2),
            .Q3(q3),
            .Q4(q4),
            .Q5(q5),
            .Q6(q6),
            .Q7(q7),
            .Q8(q8),
            .SHIFTOUT1(),
            .SHIFTOUT2(),
            .BITSLIP(sweep_bitslip_pulse[lane]),
            .CE1(1'b1),
            .CE2(1'b1),
            .CLK(hs_clk_io),
            .CLKB(!hs_clk_io),
            .CLKDIV(byte_clk),
            .CLKDIVP(1'b0),
            .D(1'b0),
            .DDLY(data_delay[lane]),
            .DYNCLKDIVSEL(1'b0),
            .DYNCLKSEL(1'b0),
            .OCLK(1'b0),
            .OCLKB(1'b0),
            .OFB(1'b0),
            .RST(!rst_n || (sup_enable && sup_serdes_rst) || rt_serdes_rst),
            .SHIFTIN1(1'b0),
            .SHIFTIN2(1'b0),
            .O()
        );

        assign serdes_byte[lane] = {q8, q7, q6, q5, q4, q3, q2, q1};
    end

    always_ff @(posedge byte_clk or negedge rst_n) begin
        if (!rst_n) begin
            hs_clk_seen <= 1'b0;
            hs_clk_seen_count <= 8'h00;
            lane_sot_seen <= '0;
            lane_last_byte <= '0;
            lane_raw_changed_seen <= '0;
            lane_raw_non_ff_seen <= '0;
            lane_raw_non_00_seen <= '0;
            lane_raw_change_count <= '0;
            stream_byte_data <= '0;
            stream_byte_keep <= '0;
            stream_byte_valid <= 1'b0;
            stream_byte_sop <= 1'b0;
            stream_byte_eop <= 1'b0;
            stream_started <= 1'b0;
            stream_sop_pending <= 1'b0;
            stream_prev_valid <= 1'b0;
            stream_pairing_active <= STREAM_PAIRING[2:0];
            stream_pairing_next <= STREAM_PAIRING[2:0];
            stream_prev_aligned_byte <= '0;
            stream_buffer_active <= 1'b0;
            stream_buffer_releasing <= 1'b0;
            stream_buffer_pairing <= STREAM_PAIRING[2:0];
            stream_buffer_wr_ptr <= '0;
            stream_buffer_rd_ptr <= '0;
            stream_buffer_count <= '0;
            stream_buffer_data <= '0;
            stream_buffer_overflow_seen <= 1'b0;
            lane_raw_prev <= '0;
            serdes_byte_sample <= '0;
            lp_sync1 <= '0;
            lp_sync2 <= '0;
            lp11_prev <= '1;
            sup_hs_settled_prev <= 1'b0;
            sot_window_active <= '0;
            sot_window_count <= '0;
            sot_blank_count <= '0;
            sot_found_burst <= 1'b0;
            burst_cap_idx <= 3'd0;
            for (int c = 0; c < 4; c++) burst_cap[c] <= 8'h00;
            dbg_burst_count <= 16'h0000;
            dbg_sot_burst_count <= 16'h0000;
            dbg_missed_burst <= 32'h0000_0000;
            relock_pending <= 1'b1;
            relock_cnt <= 16'h0000;
            dbg_relock_latency <= 16'h0000;
            dbg_relock_max <= 16'h0000;
            lane_locked <= '0;
            lane_rotation <= '0;
            lane_header_count <= '0;
            header_valid <= 1'b0;
            header_di <= 8'h00;
            header_wc <= 16'h0000;
            header_ecc <= 8'h00;
            sync_header_valid <= 1'b0;
            sync_header_di <= 8'h00;
            sync_header_wc <= 16'h0000;
            sync_header_ecc <= 8'h00;
            sync_header_rotation_lane0 <= 3'd0;
            sync_header_rotation_lane1 <= 3'd0;
            sync_header_bit_offset_lane0 <= 3'd0;
            sync_header_bit_offset_lane1 <= 3'd0;
            sync_header_score <= 4'd0;
            sync_header_start_slot <= 3'd0;
            sync_header_pairing <= 3'd0;
            sync_header_syndrome <= 6'h00;
            sync_header_ecc_no_error <= 1'b0;
            sync_header_ecc_corrected <= 1'b0;
            sync_header_ecc_uncorrectable <= 1'b0;
            sync_header_candidate_1 <= '0;
            sync_header_candidate_2 <= '0;
            sync_scan_active <= 1'b0;
            sync_scan_bit_offset_lane0 <= 3'd0;
            sync_scan_bit_offset_lane1 <= 3'd0;
            sync_scan_pairing <= 3'd0;
            sync_scan_lane0_stream <= 64'h0000_0000_0000_0000;
            sync_scan_lane1_stream <= 64'h0000_0000_0000_0000;
            sync_scan_best_score <= 4'd0;
            sync_scan_best_di <= 8'h00;
            sync_scan_best_wc <= 16'h0000;
            sync_scan_best_ecc <= 8'h00;
            sync_scan_best_syndrome <= 6'h00;
            sync_scan_best_pairing <= 3'd0;
            sync_scan_best_bit_offset_lane0 <= 3'd0;
            sync_scan_best_bit_offset_lane1 <= 3'd0;
            sync_scan_best_no_error <= 1'b0;
            sync_scan_best_corrected <= 1'b0;
            sync_scan_best_uncorrectable <= 1'b0;
            header_slot_valid <= 8'h00;
            header_slot_di <= '0;
            header_slot_wc <= '0;
            header_slot_ecc <= '0;
            header_slot_bitslip_phase <= '0;
            header_slot_bitslip_phase_lane1 <= '0;
            header_slot_transform <= '0;
            header_slot_rotation <= '0;
            header_slot_corr_di <= '0;
            header_slot_corr_wc <= '0;
            header_slot_syndrome <= '0;
            header_slot_ecc_no_error <= 8'h00;
            header_slot_ecc_corrected <= 8'h00;
            header_slot_ecc_uncorrectable <= 8'h00;
            trace_slot_valid <= 8'h00;
            trace_slot_lane0_raw <= '0;
            trace_slot_lane1_raw <= '0;
            trace_slot_lane0_candidate <= '0;
            trace_slot_lane1_candidate <= '0;
            trace_slot_lane0_aligned <= '0;
            trace_slot_lane1_aligned <= '0;
            trace_slot_lane0_rotation <= '0;
            trace_slot_lane1_rotation <= '0;
            trace_slot_bitslip_phase_lane0 <= '0;
            trace_slot_bitslip_phase_lane1 <= '0;
            trace_slot_sot_hit_lane0 <= 8'h00;
            trace_slot_sot_hit_lane1 <= 8'h00;
            trace_capture_active <= 1'b0;
            trace_capture_done <= 1'b0;
            trace_write_index <= 3'd0;
            live_trace_seq <= 8'h00;
            live_trace_slot_valid <= 8'h00;
            live_trace_slot_lane0_raw <= '0;
            live_trace_slot_lane1_raw <= '0;
            live_trace_slot_lane0_candidate <= '0;
            live_trace_slot_lane1_candidate <= '0;
            live_trace_slot_lane0_aligned <= '0;
            live_trace_slot_lane1_aligned <= '0;
            live_trace_slot_sot_hit_lane0 <= 8'h00;
            live_trace_slot_sot_hit_lane1 <= 8'h00;
            live_trace_slot_lane0_rotation <= '0;
            live_trace_slot_lane1_rotation <= '0;
            live_trace_capture_active <= 1'b0;
            live_trace_write_index <= 3'd0;
            sync_header_active <= 1'b0;
            sync_header_count <= 2'd0;
            header_write_index <= 3'd0;
            header_commit_pending <= 1'b0;
            header_pending_bitslip_phase <= 3'd0;
            header_pending_bitslip_phase_lane1 <= 3'd0;
            header_pending_transform <= 3'd0;
            header_pending_rotation <= 3'd0;
            lane_bitslip_phase <= '0;
            lane1_target_bitslip_phase <= runtime_bitslip_phase_lane1_sync2;
            sweep_transform <= 3'd0;
            sweep_hold_count <= '0;
            sweep_bitslip_pulse <= '0;
            fixed_bitslip_wait <= '0;
        end else begin
            automatic logic advance_config;
            automatic logic reset_alignment;
            automatic logic [LANES-1:0][7:0] current_candidate_byte;
            automatic logic [LANES-1:0][7:0] current_aligned_byte;
            automatic logic [LANES-1:0][2:0] current_rotation;
            automatic logic [LANES-1:0] current_has_sot;
            automatic logic [LANES-1:0] current_sot_in_window;
            automatic logic settle_ok;
            automatic logic settle_gate_en;
            automatic logic trace_trigger;
            automatic logic stream_emit_valid;
            automatic logic [LANES*8-1:0] stream_emit_data;
            automatic logic trace_capture_now;
            automatic logic [2:0] trace_capture_index;
            automatic logic live_trace_capture_now;
            automatic logic [2:0] live_trace_capture_index;
            automatic logic stream_buffer_write_now;
            automatic logic stream_buffer_read_now;
            automatic logic stream_buffer_needs_next;
            automatic logic [STREAM_DESKEW_ADDR_W-1:0] stream_buffer_wr_ptr_next;
            automatic logic [STREAM_DESKEW_ADDR_W-1:0] stream_buffer_rd_ptr_next;
            automatic logic [STREAM_DESKEW_ADDR_W-1:0] stream_buffer_rd_ptr_plus1;
            automatic logic [STREAM_DESKEW_ADDR_W:0] stream_buffer_count_next;

            advance_config = 1'b0;
            reset_alignment = 1'b0;
            trace_capture_now = 1'b0;
            trace_capture_index = trace_write_index;
            live_trace_capture_now = 1'b0;
            live_trace_capture_index = live_trace_write_index;
            stream_buffer_write_now = 1'b0;
            stream_buffer_read_now = 1'b0;
            stream_buffer_needs_next = 1'b0;
            stream_buffer_wr_ptr_next = stream_buffer_wr_ptr;
            stream_buffer_rd_ptr_next = stream_buffer_rd_ptr;
            stream_buffer_rd_ptr_plus1 = stream_ptr_inc(stream_buffer_rd_ptr);
            stream_buffer_count_next = stream_buffer_count;
            sweep_bitslip_pulse <= '0;
            serdes_byte_sample <= serdes_byte;
            trace_trigger = 1'b0;
            stream_emit_valid = 1'b0;
            stream_emit_data = '0;
            stream_byte_data <= '0;
            stream_byte_keep <= '0;
            stream_byte_valid <= 1'b0;
            stream_byte_sop <= 1'b0;
            stream_byte_eop <= 1'b0;

            // HS-SETTLE SoT gate (2026-06-17). When the gate is enabled, no SoT may
            // be ACCEPTED until sup_hs_settled is high -> the SoT search skips the
            // HS-prepare/settle garbage at each per-line burst head.
            // settle_gate_en (2026-06-18): the sup HS-SETTLE SoT gate is DECOUPLED
            // from the sup BUFR/ISERDES per-gate management. In sup mode, if
            // cfg_settle_blank_k>0 the sup SoT gate is turned OFF and the byte-domain
            // settle-blank (the proven continuous burst-head fix) handles the burst
            // head instead -- the two no longer stack/over-blank in gated. blank=0
            // keeps the legacy sup gate (settle_gate_en reduces to the old
            // sup_enable||cfg_hs_settle_gate, so all prior behaviour is unchanged).
            settle_gate_en = (sup_enable && (cfg_settle_blank_k == 4'd0)) || cfg_hs_settle_gate;
            settle_ok = !settle_gate_en || sup_hs_settled;
            for (int lane = 0; lane < LANES; lane++) begin
                current_candidate_byte[lane] = transform_byte(serdes_byte_sample[lane], active_transform);
                current_has_sot[lane] = has_sot_rotation(current_candidate_byte[lane]);
                current_sot_in_window[lane] = current_has_sot[lane] && sot_window_active[lane] && settle_ok;
                current_rotation[lane] = (current_has_sot[lane] && sot_window_active[lane] && settle_ok) ? find_sot_rotation(current_candidate_byte[lane]) : lane_rotation[lane];
                current_aligned_byte[lane] = rotate_right8(current_candidate_byte[lane], current_rotation[lane]);
                if (serdes_byte_sample[lane] != lane_raw_prev[lane]) begin
                    lane_raw_changed_seen[lane] <= 1'b1;
                    if (lane_raw_change_count[lane] != 8'hff) begin
                        lane_raw_change_count[lane] <= lane_raw_change_count[lane] + 8'd1;
                    end
                end
                if (serdes_byte_sample[lane] != 8'hff) begin
                    lane_raw_non_ff_seen[lane] <= 1'b1;
                end
                if (serdes_byte_sample[lane] != 8'h00) begin
                    lane_raw_non_00_seen[lane] <= 1'b1;
                end
                lane_raw_prev[lane] <= serdes_byte_sample[lane];
            end

            unique case (TRACE_TRIGGER_MODE[1:0])
                2'd1: trace_trigger = current_sot_in_window[0];
                2'd2: trace_trigger = current_sot_in_window[1];
                2'd3: trace_trigger = &current_sot_in_window;
                default: trace_trigger = |current_sot_in_window;
            endcase

            if (hs_clk_seen_count != 8'hff) begin
                hs_clk_seen_count <= hs_clk_seen_count + 8'h01;
            end
            if (hs_clk_seen_count == 8'h10) begin
                hs_clk_seen <= 1'b1;
            end

            if (SWEEP_ENABLE) begin
                advance_config = (sweep_hold_count == SWEEP_HOLD_BYTES[SWEEP_COUNT_W-1:0] - 1'b1);
                if (advance_config) begin
                    sweep_hold_count <= '0;
                    header_commit_pending <= 1'b0;
                    trace_capture_active <= 1'b0;
                    trace_capture_done <= 1'b0;
                    trace_slot_valid <= 8'h00;
                    reset_alignment = 1'b1;
                    if (sweep_transform == 3'd7) begin
                        sweep_transform <= 3'd0;
                        lane_bitslip_phase[0] <= lane_bitslip_phase[0] + 3'd1;
                        lane_bitslip_phase[1] <= lane_bitslip_phase[1] + 3'd1;
                        sweep_bitslip_pulse <= '1;
                    end else begin
                        sweep_transform <= sweep_transform + 3'd1;
                    end
                end else begin
                    sweep_hold_count <= sweep_hold_count + 1'b1;
                end
            end else if (sup_enable) begin
                // Supervisor mode: the BUFR /4 phase is realigned deterministically
                // at every clock-lane restart, so the HW bitslip retrain loop (which
                // assumes a stable byte_clk and would fight the per-gate ISERDES reset)
                // is bypassed entirely. The residual byte rotation is absorbed each
                // burst by the SW rotation hunt (find_sot_rotation, all 8 phases),
                // exactly as the Digilent reference does without HW bitslip.
                sweep_bitslip_pulse <= '0;
            end else begin
                // lane1_target_bitslip_phase keeps the LANE1_BITSLIP_SWEEP_ENABLE incremental
                // sweep semantics; in the normal (sweep-disabled) case we follow the runtime
                // knob directly. Use `effective_lane1_target` for the actual comparison so
                // the runtime knob can drive lane 1 even when the sweep branch isn't reached.
                automatic logic [2:0] effective_lane1_target;
                effective_lane1_target = (LANE1_BITSLIP_SWEEP_ENABLE | runtime_lane1_sweep_enable)
                                       ? lane1_target_bitslip_phase
                                       : runtime_bitslip_phase_lane1_sync2;
                if ((lane_bitslip_phase[0] != runtime_bitslip_phase_sync2) ||
                    (lane_bitslip_phase[1] != effective_lane1_target)) begin
                    reset_alignment = 1'b1;
                    header_commit_pending <= 1'b0;
                    trace_capture_active <= 1'b0;
                    trace_capture_done <= 1'b0;
                    trace_slot_valid <= 8'h00;
                    if (lane_bitslip_phase[0] != runtime_bitslip_phase_sync2) begin
                        if (fixed_bitslip_wait[0] == 4'd0) begin
                            sweep_bitslip_pulse[0] <= 1'b1;
                            lane_bitslip_phase[0] <= lane_bitslip_phase[0] + 3'd1;
                            fixed_bitslip_wait[0] <= 4'd8;
                        end else begin
                            fixed_bitslip_wait[0] <= fixed_bitslip_wait[0] - 4'd1;
                        end
                    end
                    if (lane_bitslip_phase[1] != effective_lane1_target) begin
                        if (fixed_bitslip_wait[1] == 4'd0) begin
                            sweep_bitslip_pulse[1] <= 1'b1;
                            lane_bitslip_phase[1] <= lane_bitslip_phase[1] + 3'd1;
                            fixed_bitslip_wait[1] <= 4'd8;
                        end else begin
                            fixed_bitslip_wait[1] <= fixed_bitslip_wait[1] - 4'd1;
                        end
                    end
                end else if (LANE1_BITSLIP_SWEEP_ENABLE | runtime_lane1_sweep_enable) begin
                    if (sweep_hold_count == SWEEP_HOLD_BYTES[SWEEP_COUNT_W-1:0] - 1'b1) begin
                        reset_alignment = 1'b1;
                        header_commit_pending <= 1'b0;
                        trace_capture_active <= 1'b0;
                        trace_capture_done <= 1'b0;
                        trace_slot_valid <= 8'h00;
                        sweep_hold_count <= '0;
                        lane1_target_bitslip_phase <= lane1_target_bitslip_phase + 3'd1;
                    end else begin
                        sweep_hold_count <= sweep_hold_count + 1'b1;
                    end
                end else if (sweep_hold_count != SWEEP_HOLD_BYTES[SWEEP_COUNT_W-1:0] - 1'b1) begin
                    reset_alignment = 1'b1;
                    sweep_hold_count <= sweep_hold_count + 1'b1;
                end
            end

            if (reset_alignment) begin
                trace_capture_active <= 1'b0;
                trace_capture_done <= 1'b0;
                trace_write_index <= 3'd0;
                sync_header_active <= 1'b0;
                sync_scan_active <= 1'b0;
                sync_header_valid <= 1'b0;
                stream_started <= 1'b0;
                stream_sop_pending <= 1'b0;
                stream_prev_valid <= 1'b0;
                stream_pairing_active <= STREAM_PAIRING[2:0];
                stream_pairing_next <= STREAM_PAIRING[2:0];
                stream_prev_aligned_byte <= '0;
                stream_buffer_active <= 1'b0;
                stream_buffer_releasing <= 1'b0;
                stream_buffer_pairing <= STREAM_PAIRING[2:0];
                stream_buffer_wr_ptr <= '0;
                stream_buffer_rd_ptr <= '0;
                stream_buffer_count <= '0;
                live_trace_capture_active <= 1'b0;
                live_trace_slot_valid <= 8'h00;
                live_trace_write_index <= 3'd0;
            end else if (trace_trigger) begin
                trace_capture_active <= 1'b1;
                trace_capture_done <= 1'b0;
                trace_slot_valid <= 8'h00;
                trace_write_index <= 3'd0;
                sync_scan_active <= 1'b0;
                sync_header_valid <= 1'b0;
            end

            if (!reset_alignment) begin
                if (trace_trigger) begin
                    stream_started <= 1'b1;
                    stream_sop_pending <= 1'b1;
                    stream_prev_valid <= 1'b0;
                    stream_pairing_active <= stream_pairing_next;
                    stream_buffer_active <= 1'b1;
                    stream_buffer_releasing <= 1'b0;
                    stream_buffer_pairing <= stream_pairing_next;
                    stream_buffer_wr_ptr <= '0;
                    stream_buffer_rd_ptr <= '0;
                    stream_buffer_count <= '0;
                    stream_prev_aligned_byte <= '0;
                end else if (stream_buffer_active) begin
                    stream_buffer_needs_next = stream_pairing_uses_next_slot(stream_buffer_pairing);
                    stream_buffer_write_now = (stream_buffer_count != STREAM_DESKEW_COUNT_MAX);
                    stream_buffer_read_now = stream_buffer_releasing &&
                                             ((stream_buffer_needs_next && (stream_buffer_count >= STREAM_DESKEW_COUNT_TWO)) ||
                                              (!stream_buffer_needs_next && (stream_buffer_count >= STREAM_DESKEW_COUNT_ONE)));
                    stream_buffer_wr_ptr_next = stream_buffer_wr_ptr;
                    stream_buffer_rd_ptr_next = stream_buffer_rd_ptr;
                    stream_buffer_count_next = stream_buffer_count;

                    if (stream_buffer_write_now) begin
                        stream_buffer_data[stream_buffer_wr_ptr] <= current_aligned_byte;
                        stream_buffer_wr_ptr_next = stream_ptr_inc(stream_buffer_wr_ptr);
                        stream_buffer_count_next = stream_buffer_count_next + STREAM_DESKEW_COUNT_ONE;
                    end else begin
                        stream_buffer_overflow_seen <= 1'b1;
                    end

                    if (stream_buffer_read_now) begin
                        stream_emit_valid = 1'b1;
                        stream_emit_data = select_stream_word(
                            stream_buffer_pairing,
                            stream_buffer_data[stream_buffer_rd_ptr],
                            stream_buffer_data[stream_buffer_rd_ptr_plus1]
                        );
                        stream_byte_data <= stream_emit_data;
                        stream_byte_keep <= '1;
                        stream_byte_valid <= 1'b1;
                        stream_byte_sop <= stream_sop_pending;
                        stream_sop_pending <= 1'b0;
                        stream_buffer_rd_ptr_next = stream_ptr_inc(stream_buffer_rd_ptr);
                        stream_buffer_count_next = stream_buffer_count_next - STREAM_DESKEW_COUNT_ONE;
                    end
                    stream_buffer_wr_ptr <= stream_buffer_wr_ptr_next;
                    stream_buffer_rd_ptr <= stream_buffer_rd_ptr_next;
                    stream_buffer_count <= stream_buffer_count_next;
                    stream_prev_aligned_byte <= current_aligned_byte;
                    stream_prev_valid <= 1'b1;
                end
            end

            if (!reset_alignment) begin
                trace_capture_now = trace_trigger || trace_capture_active;
                trace_capture_index = trace_trigger ? 3'd0 : trace_write_index;
                if (trace_capture_now) begin
                    if (trace_trigger) begin
                        trace_slot_valid <= 8'h01;
                    end else begin
                        trace_slot_valid[trace_capture_index] <= 1'b1;
                    end
                    trace_slot_lane0_raw[trace_capture_index] <= serdes_byte_sample[0];
                    trace_slot_lane1_raw[trace_capture_index] <= serdes_byte_sample[1];
                    trace_slot_lane0_candidate[trace_capture_index] <= current_candidate_byte[0];
                    trace_slot_lane1_candidate[trace_capture_index] <= current_candidate_byte[1];
                    trace_slot_lane0_aligned[trace_capture_index] <= current_aligned_byte[0];
                    trace_slot_lane1_aligned[trace_capture_index] <= current_aligned_byte[1];
                    trace_slot_lane0_rotation[trace_capture_index] <= current_rotation[0];
                    trace_slot_lane1_rotation[trace_capture_index] <= current_rotation[1];
                    trace_slot_bitslip_phase_lane0[trace_capture_index] <= lane_bitslip_phase[0];
                    trace_slot_bitslip_phase_lane1[trace_capture_index] <= lane_bitslip_phase[1];
                    trace_slot_sot_hit_lane0[trace_capture_index] <= current_has_sot[0];
                    trace_slot_sot_hit_lane1[trace_capture_index] <= current_has_sot[1];

                    if (trace_capture_index == 3'd7) begin
                        trace_capture_active <= 1'b0;
                        trace_capture_done <= 1'b1;
                    end else begin
                        trace_capture_active <= 1'b1;
                        trace_write_index <= trace_capture_index + 3'd1;
                    end
                end
            end

            if (!reset_alignment) begin
                live_trace_capture_now = trace_trigger || live_trace_capture_active;
                live_trace_capture_index = trace_trigger ? 3'd0 : live_trace_write_index;
                if (live_trace_capture_now) begin
                    if (trace_trigger) begin
                        live_trace_seq <= live_trace_seq + 8'd1;
                        live_trace_slot_valid <= 8'h01;
                    end else begin
                        live_trace_slot_valid[live_trace_capture_index] <= 1'b1;
                    end
                    live_trace_slot_lane0_raw[live_trace_capture_index] <= serdes_byte_sample[0];
                    live_trace_slot_lane1_raw[live_trace_capture_index] <= serdes_byte_sample[1];
                    live_trace_slot_lane0_candidate[live_trace_capture_index] <= current_candidate_byte[0];
                    live_trace_slot_lane1_candidate[live_trace_capture_index] <= current_candidate_byte[1];
                    live_trace_slot_lane0_aligned[live_trace_capture_index] <= current_aligned_byte[0];
                    live_trace_slot_lane1_aligned[live_trace_capture_index] <= current_aligned_byte[1];
                    live_trace_slot_sot_hit_lane0[live_trace_capture_index] <= current_sot_in_window[0];
                    live_trace_slot_sot_hit_lane1[live_trace_capture_index] <= current_sot_in_window[1];
                    live_trace_slot_lane0_rotation[live_trace_capture_index] <= current_rotation[0];
                    live_trace_slot_lane1_rotation[live_trace_capture_index] <= current_rotation[1];
                    if (live_trace_capture_index == 3'd7) begin
                        live_trace_capture_active <= 1'b0;
                    end else begin
                        live_trace_capture_active <= 1'b1;
                        live_trace_write_index <= live_trace_capture_index + 3'd1;
                    end
                end
            end

            if (!reset_alignment && trace_capture_done && !sync_scan_active) begin
                sync_scan_active <= 1'b1;
                sync_scan_bit_offset_lane0 <= 3'd0;
                sync_scan_bit_offset_lane1 <= 3'd0;
                sync_scan_pairing <= 3'd0;
                sync_scan_best_score <= 4'd0;
                sync_scan_best_di <= 8'h00;
                sync_scan_best_wc <= 16'h0000;
                sync_scan_best_ecc <= 8'h00;
                sync_scan_best_syndrome <= 6'h00;
                sync_scan_best_pairing <= 3'd0;
                sync_scan_best_bit_offset_lane0 <= 3'd0;
                sync_scan_best_bit_offset_lane1 <= 3'd0;
                sync_scan_best_no_error <= 1'b0;
                sync_scan_best_corrected <= 1'b0;
                sync_scan_best_uncorrectable <= 1'b0;
                if (SYNC_HEADER_USE_ALIGNED_STREAM) begin
                    sync_scan_lane0_stream <= {
                        trace_slot_lane0_aligned[7], trace_slot_lane0_aligned[6], trace_slot_lane0_aligned[5], trace_slot_lane0_aligned[4],
                        trace_slot_lane0_aligned[3], trace_slot_lane0_aligned[2], trace_slot_lane0_aligned[1], trace_slot_lane0_aligned[0]
                    };
                    sync_scan_lane1_stream <= {
                        trace_slot_lane1_aligned[7], trace_slot_lane1_aligned[6], trace_slot_lane1_aligned[5], trace_slot_lane1_aligned[4],
                        trace_slot_lane1_aligned[3], trace_slot_lane1_aligned[2], trace_slot_lane1_aligned[1], trace_slot_lane1_aligned[0]
                    };
                end else begin
                    sync_scan_lane0_stream <= {
                        trace_slot_lane0_candidate[7], trace_slot_lane0_candidate[6], trace_slot_lane0_candidate[5], trace_slot_lane0_candidate[4],
                        trace_slot_lane0_candidate[3], trace_slot_lane0_candidate[2], trace_slot_lane0_candidate[1], trace_slot_lane0_candidate[0]
                    };
                    sync_scan_lane1_stream <= {
                        trace_slot_lane1_candidate[7], trace_slot_lane1_candidate[6], trace_slot_lane1_candidate[5], trace_slot_lane1_candidate[4],
                        trace_slot_lane1_candidate[3], trace_slot_lane1_candidate[2], trace_slot_lane1_candidate[1], trace_slot_lane1_candidate[0]
                    };
                end
            end else if (!reset_alignment && sync_scan_active) begin
                automatic logic [3:0] best_score_next;
                automatic logic [7:0] best_di_next;
                automatic logic [15:0] best_wc_next;
                automatic logic [7:0] best_ecc_next;
                automatic logic [5:0] best_syndrome_next;
                automatic logic [2:0] best_pairing_next;
                automatic logic [2:0] best_bit_offset_lane0_next;
                automatic logic [2:0] best_bit_offset_lane1_next;
                automatic logic best_no_error_next;
                automatic logic best_corrected_next;
                automatic logic best_uncorrectable_next;
                automatic logic [3:0][7:0] lane0_post_sot;
                automatic logic [3:0][7:0] lane1_post_sot;
                automatic logic [7:0] candidate_di;
                automatic logic [7:0] candidate_wc_low;
                automatic logic [7:0] candidate_wc_high;
                automatic logic [7:0] candidate_ecc;
                automatic logic [23:0] candidate_data;
                automatic logic [23:0] corrected_data;
                automatic logic [5:0] candidate_syndrome;
                automatic int candidate_bit;
                automatic logic candidate_data_error;
                automatic logic candidate_ecc_error;
                automatic logic candidate_no_error;
                automatic logic candidate_corrected;
                automatic logic candidate_uncorrectable;
                automatic logic [3:0] candidate_score;
                automatic logic last_candidate;
                automatic logic [2:0] scan_bit_offset_lane0;
                automatic logic [2:0] scan_bit_offset_lane1;

                best_score_next = sync_scan_best_score;
                best_di_next = sync_scan_best_di;
                best_wc_next = sync_scan_best_wc;
                best_ecc_next = sync_scan_best_ecc;
                best_syndrome_next = sync_scan_best_syndrome;
                best_pairing_next = sync_scan_best_pairing;
                best_bit_offset_lane0_next = sync_scan_best_bit_offset_lane0;
                best_bit_offset_lane1_next = sync_scan_best_bit_offset_lane1;
                best_no_error_next = sync_scan_best_no_error;
                best_corrected_next = sync_scan_best_corrected;
                best_uncorrectable_next = sync_scan_best_uncorrectable;
                scan_bit_offset_lane0 = SYNC_HEADER_SWEEP_BIT_OFFSETS ? sync_scan_bit_offset_lane0 : 3'd0;
                scan_bit_offset_lane1 = SYNC_HEADER_SWEEP_BIT_OFFSETS ? sync_scan_bit_offset_lane1 : 3'd0;

                if ((stream_byte_at(sync_scan_lane0_stream, scan_bit_offset_lane0) == SOT_SYNC) &&
                    (stream_byte_at(sync_scan_lane1_stream, scan_bit_offset_lane1) == SOT_SYNC)) begin
                    for (int byte_idx = 0; byte_idx < 4; byte_idx++) begin
                        lane0_post_sot[byte_idx] = stream_byte_at(sync_scan_lane0_stream, scan_bit_offset_lane0 + (8 * (byte_idx + 1)));
                        lane1_post_sot[byte_idx] = stream_byte_at(sync_scan_lane1_stream, scan_bit_offset_lane1 + (8 * (byte_idx + 1)));
                    end

                    candidate_di = 8'h00;
                    candidate_wc_low = 8'h00;
                    candidate_wc_high = 8'h00;
                    candidate_ecc = 8'h00;

                    unique case (sync_scan_pairing)
                        3'd0: begin
                            candidate_di = lane0_post_sot[0];
                            candidate_wc_low = lane1_post_sot[0];
                            candidate_wc_high = lane0_post_sot[1];
                            candidate_ecc = lane1_post_sot[1];
                        end
                        3'd1: begin
                            candidate_di = lane1_post_sot[0];
                            candidate_wc_low = lane0_post_sot[0];
                            candidate_wc_high = lane1_post_sot[1];
                            candidate_ecc = lane0_post_sot[1];
                        end
                        3'd2: begin
                            candidate_di = lane0_post_sot[0];
                            candidate_wc_low = lane1_post_sot[1];
                            candidate_wc_high = lane0_post_sot[1];
                            candidate_ecc = lane1_post_sot[2];
                        end
                        3'd3: begin
                            candidate_di = lane0_post_sot[1];
                            candidate_wc_low = lane1_post_sot[0];
                            candidate_wc_high = lane0_post_sot[2];
                            candidate_ecc = lane1_post_sot[1];
                        end
                        3'd4: begin
                            candidate_di = lane1_post_sot[0];
                            candidate_wc_low = lane0_post_sot[1];
                            candidate_wc_high = lane1_post_sot[1];
                            candidate_ecc = lane0_post_sot[2];
                        end
                        default: begin
                            candidate_di = lane1_post_sot[1];
                            candidate_wc_low = lane0_post_sot[0];
                            candidate_wc_high = lane1_post_sot[2];
                            candidate_ecc = lane0_post_sot[1];
                        end
                    endcase

                    candidate_data = {candidate_wc_high, candidate_wc_low, candidate_di};
                    candidate_syndrome = candidate_ecc[5:0] ^ calc_ecc6(candidate_data);
                    corrected_data = candidate_data;
                    if (SYNC_HEADER_SWEEP_BIT_OFFSETS) begin
                        candidate_bit = decode_data_bit(candidate_syndrome);
                        candidate_data_error = (candidate_syndrome != 6'h00) && (candidate_bit >= 0);
                        candidate_ecc_error = (candidate_syndrome != 6'h00) && is_onehot6(candidate_syndrome) && !candidate_data_error;
                        if (candidate_data_error) begin
                            corrected_data[candidate_bit] = ~corrected_data[candidate_bit];
                        end

                        candidate_no_error = (candidate_syndrome == 6'h00);
                        candidate_corrected = candidate_data_error || candidate_ecc_error;
                        candidate_uncorrectable = (candidate_syndrome != 6'h00) && !candidate_data_error && !candidate_ecc_error;
                        candidate_score = header_candidate_score(candidate_no_error, candidate_corrected, corrected_data[7:0], corrected_data[23:8]);
                    end else begin
                        candidate_bit = -1;
                        candidate_data_error = 1'b0;
                        candidate_ecc_error = 1'b0;
                        candidate_no_error = (candidate_syndrome == 6'h00);
                        candidate_corrected = 1'b0;
                        candidate_uncorrectable = (candidate_syndrome != 6'h00);
                        candidate_score = header_candidate_score(candidate_no_error, candidate_corrected, candidate_di, {candidate_wc_high, candidate_wc_low});
                    end

                    if ((candidate_score > best_score_next) || ((candidate_score == best_score_next) && (sync_scan_pairing == 3'd0))) begin
                        best_score_next = candidate_score;
                        best_di_next = corrected_data[7:0];
                        best_wc_next = corrected_data[23:8];
                        best_ecc_next = candidate_ecc;
                        best_syndrome_next = candidate_syndrome;
                        best_pairing_next = sync_scan_pairing;
                        best_bit_offset_lane0_next = scan_bit_offset_lane0;
                        best_bit_offset_lane1_next = scan_bit_offset_lane1;
                        best_no_error_next = candidate_no_error;
                        best_corrected_next = candidate_corrected;
                        best_uncorrectable_next = candidate_uncorrectable;
                    end
                end

                last_candidate = (sync_scan_pairing == 3'd5) &&
                                 (!SYNC_HEADER_SWEEP_BIT_OFFSETS ||
                                  ((sync_scan_bit_offset_lane0 == SYNC_HEADER_LAST_BIT_OFFSET) &&
                                   (sync_scan_bit_offset_lane1 == SYNC_HEADER_LAST_BIT_OFFSET)));

                if (last_candidate) begin
                    sync_scan_active <= 1'b0;
                    trace_capture_done <= 1'b0;
                    sync_header_valid <= (best_score_next >= MIN_SYNC_HEADER_SCORE);
                    sync_header_di <= best_di_next;
                    sync_header_wc <= best_wc_next;
                    sync_header_ecc <= best_ecc_next;
                    sync_header_start_slot <= 3'd0;
                    sync_header_pairing <= best_pairing_next;
                    sync_header_bit_offset_lane0 <= best_bit_offset_lane0_next;
                    sync_header_bit_offset_lane1 <= best_bit_offset_lane1_next;
                    sync_header_score <= best_score_next;
                    sync_header_syndrome <= best_syndrome_next;
                    sync_header_ecc_no_error <= best_no_error_next;
                    sync_header_ecc_corrected <= best_corrected_next;
                    sync_header_ecc_uncorrectable <= best_uncorrectable_next;
                    if ((best_score_next >= MIN_SYNC_HEADER_SCORE) &&
                        (best_bit_offset_lane0_next == 3'd0) &&
                        (best_bit_offset_lane1_next == 3'd0)) begin
                        stream_pairing_next <= best_pairing_next;
                        stream_pairing_active <= best_pairing_next;
                        stream_buffer_pairing <= best_pairing_next;
                        stream_buffer_releasing <= 1'b1;
                    end else begin
                        stream_started <= 1'b0;
                        stream_sop_pending <= 1'b0;
                        stream_buffer_active <= 1'b0;
                        stream_buffer_releasing <= 1'b0;
                        stream_buffer_count <= '0;
                    end
                    if (best_score_next < MIN_SYNC_HEADER_SCORE) begin
                        trace_capture_active <= 1'b0;
                        trace_capture_done <= 1'b0;
                    end
                end else begin
                    sync_scan_best_score <= best_score_next;
                    sync_scan_best_di <= best_di_next;
                    sync_scan_best_wc <= best_wc_next;
                    sync_scan_best_ecc <= best_ecc_next;
                    sync_scan_best_syndrome <= best_syndrome_next;
                    sync_scan_best_pairing <= best_pairing_next;
                    sync_scan_best_bit_offset_lane0 <= best_bit_offset_lane0_next;
                    sync_scan_best_bit_offset_lane1 <= best_bit_offset_lane1_next;
                    sync_scan_best_no_error <= best_no_error_next;
                    sync_scan_best_corrected <= best_corrected_next;
                    sync_scan_best_uncorrectable <= best_uncorrectable_next;
                    if (sync_scan_pairing != 3'd5) begin
                        sync_scan_pairing <= sync_scan_pairing + 3'd1;
                    end else begin
                        sync_scan_pairing <= 3'd0;
                        if (SYNC_HEADER_SWEEP_BIT_OFFSETS) begin
                            if (sync_scan_bit_offset_lane1 != SYNC_HEADER_LAST_BIT_OFFSET) begin
                                sync_scan_bit_offset_lane1 <= sync_scan_bit_offset_lane1 + 3'd1;
                            end else begin
                                sync_scan_bit_offset_lane1 <= 3'd0;
                                sync_scan_bit_offset_lane0 <= sync_scan_bit_offset_lane0 + 3'd1;
                            end
                        end
                    end
                end
            end

            if (header_commit_pending) begin
                automatic logic [23:0] raw_header_data;
                automatic logic [23:0] corrected_header_data;
                automatic logic [5:0] syndrome;
                automatic int corrected_bit;
                automatic logic data_bit_error;
                automatic logic ecc_bit_error;
                automatic logic uncorrectable;

                raw_header_data = {header_wc, header_di};
                syndrome = header_ecc[5:0] ^ calc_ecc6(raw_header_data);
                corrected_header_data = raw_header_data;
                corrected_bit = decode_data_bit(syndrome);
                data_bit_error = (syndrome != 6'h00) && (corrected_bit >= 0);
                ecc_bit_error = (syndrome != 6'h00) && is_onehot6(syndrome) && !data_bit_error;
                uncorrectable = (syndrome != 6'h00) && !data_bit_error && !ecc_bit_error;

                if (data_bit_error) begin
                    corrected_header_data[corrected_bit] = ~corrected_header_data[corrected_bit];
                end

                header_valid <= 1'b1;
                header_slot_valid[header_write_index] <= 1'b1;
                header_slot_di[header_write_index] <= header_di;
                header_slot_wc[header_write_index] <= header_wc;
                header_slot_ecc[header_write_index] <= header_ecc;
                header_slot_bitslip_phase[header_write_index] <= header_pending_bitslip_phase;
                header_slot_bitslip_phase_lane1[header_write_index] <= header_pending_bitslip_phase_lane1;
                header_slot_transform[header_write_index] <= header_pending_transform;
                header_slot_rotation[header_write_index] <= header_pending_rotation;
                header_slot_corr_di[header_write_index] <= corrected_header_data[7:0];
                header_slot_corr_wc[header_write_index] <= corrected_header_data[23:8];
                header_slot_syndrome[header_write_index] <= syndrome;
                header_slot_ecc_no_error[header_write_index] <= (syndrome == 6'h00);
                header_slot_ecc_corrected[header_write_index] <= data_bit_error || ecc_bit_error;
                header_slot_ecc_uncorrectable[header_write_index] <= uncorrectable;
                header_write_index <= header_write_index + 3'd1;
                header_commit_pending <= 1'b0;
            end

            for (int lane = 0; lane < LANES; lane++) begin
                automatic logic lp11_now;
                automatic logic [7:0] aligned_byte;
                automatic logic [7:0] candidate_byte;
                automatic logic logical_lane;
                automatic logic [2:0] logical_lane0_physical;

                lp_sync1[lane] <= {dphy_data_lp_p[lane], dphy_data_lp_n[lane]};
                lp_sync2[lane] <= lp_sync1[lane];
                lp11_now = (lp_sync2[lane] == 2'b11);
                candidate_byte = current_candidate_byte[lane];
                aligned_byte = rotate_right8(candidate_byte, lane_rotation[lane]);
                logical_lane = lane[0] ^ active_transform[2];
                logical_lane0_physical = {2'b00, active_transform[2]};
                lane_last_byte[lane] <= aligned_byte;

                if (reset_alignment) begin
                    sot_window_active[lane] <= 1'b1;
                    sot_window_count[lane] <= '0;
                    lane_locked[lane] <= 1'b0;
                    lane_header_count[lane] <= '0;
                // Legacy: LP-11 -> exit on the data lane opens the SoT window.
                // Supervisor: the byte_clk is gated, so the data-lane LP edge may
                // be unobservable; instead open the window on the supervisor's
                // HS-SETTLE rising edge (a fresh burst's payload is about to start).
                end else if ((lp11_prev[lane] && !lp11_now) ||
                             (settle_gate_en && sup_hs_settled && !sup_hs_settled_prev)) begin
                    // LP-exit (burst head). With cfg_settle_blank_k>0, hold the
                    // window CLOSED for K byte_clk so the SoT search skips the
                    // HS-prepare/settle garbage; else open immediately (legacy).
                    sot_window_count[lane] <= '0;
                    lane_locked[lane] <= 1'b0;
                    lane_header_count[lane] <= '0;
                    if (cfg_settle_blank_k != 4'd0) begin
                        sot_blank_count[lane]  <= cfg_settle_blank_k;
                        sot_window_active[lane] <= 1'b0;
                    end else begin
                        sot_window_active[lane] <= 1'b1;
                    end
                end else if (sot_blank_count[lane] != 4'd0) begin
                    sot_blank_count[lane] <= sot_blank_count[lane] - 4'd1;
                    if (sot_blank_count[lane] == 4'd1) begin
                        sot_window_active[lane] <= 1'b1;   // blank elapsed -> open
                        sot_window_count[lane] <= '0;
                    end
                end else if (sot_window_active[lane]) begin
                    if (sot_window_count[lane] == SOT_WINDOW_BYTES[WINDOW_COUNT_W-1:0] - 1'b1) begin
                        sot_window_active[lane] <= 1'b0;
                    end else begin
                        sot_window_count[lane] <= sot_window_count[lane] + 1'b1;
                    end
                end

                // In supervisor mode require HS-SETTLE to have elapsed before
                // accepting a SoT, so HS-prepare garbage can never false-lock.
                if (!reset_alignment && sot_window_active[lane] && has_sot_rotation(candidate_byte)
                    && (!settle_gate_en || sup_hs_settled)) begin
                    lane_sot_seen[lane] <= 1'b1;
                    lane_locked[lane] <= 1'b1;
                    lane_rotation[lane] <= find_sot_rotation(candidate_byte);
                    lane_header_count[lane] <= '0;
                    sot_window_active[lane] <= 1'b0;
                end else if (!reset_alignment && lane_locked[lane]) begin
                    if (!logical_lane) begin
                        if (lane_header_count[lane] == 3'd0) begin
                            header_di <= aligned_byte;
                        end else if (lane_header_count[lane] == 3'd1) begin
                            header_wc[15:8] <= aligned_byte;
                        end
                    end else begin
                        if (lane_header_count[lane] == 3'd0) begin
                            header_wc[7:0] <= aligned_byte;
                        end else if (lane_header_count[lane] == 3'd1) begin
                            header_ecc <= aligned_byte;
                            header_pending_bitslip_phase <= lane_bitslip_phase[0];
                            header_pending_bitslip_phase_lane1 <= lane_bitslip_phase[1];
                            header_pending_transform <= active_transform;
                            header_pending_rotation <= lane_rotation[logical_lane0_physical];
                            header_commit_pending <= 1'b1;
                        end
                    end

                    if (lane_header_count[lane] != 3'd7) begin
                        lane_header_count[lane] <= lane_header_count[lane] + 3'd1;
                    end
                end

                // SoT-miss diagnostics (lane 0): count bursts (LP-exit edges) vs
                // bursts with a stream SoT, and snapshot the head bytes of a no-SoT
                // burst. Independent of the blank, so sweeping cfg_settle_blank_k and
                // watching sot_burst_count shows whether the blank recovers SoTs.
                if (lane == 0) begin
                    if (lp11_prev[0] && !lp11_now) begin
                        if (!sot_found_burst) begin
                            dbg_missed_burst <= {burst_cap[3], burst_cap[2],
                                                 burst_cap[1], burst_cap[0]};
                        end
                        dbg_burst_count <= dbg_burst_count + 16'd1;
                        sot_found_burst <= 1'b0;
                        burst_cap_idx   <= 3'd0;
                    end else begin
                        if (burst_cap_idx < 3'd4) begin
                            burst_cap[burst_cap_idx] <= current_candidate_byte[0];
                            burst_cap_idx <= burst_cap_idx + 3'd1;
                        end
                        if (current_sot_in_window[0] && !sot_found_burst) begin
                            dbg_sot_burst_count <= dbg_sot_burst_count + 16'd1;
                            sot_found_burst     <= 1'b1;
                        end
                    end
                end

                lp11_prev[lane] <= lp11_now;
            end

            // Vblank-exit re-lock latency: from the ISERDES-reset RELEASE (byte_clk
            // resumed after a clock-lane gate) count byte_clk until the first SoT.
            begin
                automatic logic srst_eff;
                srst_eff = (sup_enable && sup_serdes_rst) || rt_serdes_rst;
                if (srst_eff) begin
                    relock_pending <= 1'b1;
                    relock_cnt     <= 16'h0000;
                end else if (relock_pending) begin
                    if (|current_sot_in_window) begin
                        dbg_relock_latency <= relock_cnt;
                        if (relock_cnt > dbg_relock_max) dbg_relock_max <= relock_cnt;
                        relock_pending <= 1'b0;
                    end else if (relock_cnt != 16'hFFFF) begin
                        relock_cnt <= relock_cnt + 16'h0001;
                    end
                end
            end

            sup_hs_settled_prev <= sup_hs_settled;
        end
    end

    // Expose serdes_byte_sample as output (passthrough for ring buffer capture)
    assign serdes_byte_sample_out = serdes_byte_sample;

endmodule

`default_nettype wire