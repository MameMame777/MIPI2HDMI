`timescale 1ns / 1ps
`default_nettype none

module mipi_to_hdmi_probe_top #(
    // zero-PYNQ RX bake (2026-06-19): defaults set to the verified continuous +
    // RGB565 runtime config so the bitstream-init configures a *usable* stream at
    // boot (no PYNQ). OV5640 values cross-checked vs docs/doc/ov5640_linux_mainline_
    // reference.md: 0x4800=0x14 continuous (mainline/Digilent 0x24 non-continuous --
    // deliberate divergence; the continuous-only HW-lock FSM + band fix require it,
    // diary 2026-06-14/17); 0x4300=0x6F RGB565 (mainline 0x3F UYVY -- project target,
    // ref line 300); 0x501F=0x01 RGB565 ISP mux (mainline 0x00 YUV, ref line 300/307).
    parameter int  PROBE_IDELAY_TAP            = 16,   // eye-centre (was 8; runtime used 16)
    parameter bit  PROBE_LANE1_BITSLIP_SWEEP   = 1'b0,
    parameter int  STREAM_PAIRING              = 0,
    parameter logic [7:0] OV5640_MIPI_CTRL_4800 = 8'h14,   // continuous (was 0x24 gated)
    parameter logic [7:0] OV5640_FORMAT_CTRL_4300 = 8'h6F, // RGB565 (was 0x30 YUV422)
    parameter logic [7:0] OV5640_ISP_FORMAT_501F = 8'h01,  // RGB565 ISP mux (was 0x00 YUV)
    parameter logic [7:0] OV5640_ISP_CTRL_5000 = 8'ha7,
    parameter logic [7:0] OV5640_ISP_CTRL_5001 = 8'h83,
    parameter bit OV5640_TEST_PATTERN_ENABLE = 1'b0,
    parameter bit USE_CRC_LINE_REPLAY = 1'b0,
    parameter bit CAPTURE_RAW_PAYLOAD = 1'b0,
    // COLOR_CAPTURE (2026-06-23, image-processing research base): 0 = legacy Y8
    // grayscale capture (the 30fps/lock/K=14 verified path -> m_axis_capture 8-bit,
    // packed 4px/word by the BD axis_y8_to_vdma32). 1 = 24-bit RGB888 capture: tap
    // the muxed video_pixel[23:0] straight into a 24-bit bridge (bypassing the
    // Y8-only frame_normalizer/ob_row_masker) -> m_axis_capture 24-bit, packed
    // 1px/32-bit-word (RGBA32) by the BD axis_rgb24_to_vdma32. Set via core0 BD
    // CONFIG; the BD must match the port width + use the RGB pack/unpack cells.
    parameter bit COLOR_CAPTURE = 1'b0,
    parameter bit USE_RGB565_GRAY = 1'b0,
    // IMAGE_FORMAT: 0 = YUV422 (default), 1 = RGB565 (alias USE_RGB565_GRAY),
    //               2 = RAW8 (DT=0x2A), 3 = RAW10 packed (DT=0x2B)
    // RAW10 output is truncated to upper 8 bits for downstream Y8 path.
    // OV5640 SCCB init still configures YUV422 — RAW init must be added before
    // RAW formats can be exercised on real hardware.
    parameter int IMAGE_FORMAT = 1,
    // HWLOCK_DEFAULT_ON: bake the HW deterministic-lock FSM ON at power-up. The FSM
    // auto-locks as soon as the chip streams, with no PYNQ lock step; bit[26] of the
    // bitslip word INHIBITS it at runtime (software lock_mode fallback). Controlled
    // by this RTL DEFAULT (= the binding path in this BD flow -- the pre_synth
    // fileset generics are all reported "Unused", so values come from the module
    // default, exactly like IMAGE_FORMAT=1). Set to 1'b0 here to build opt-in
    // (bitslip_word[25]-only). (2026-06-19)
    parameter bit HWLOCK_DEFAULT_ON = 1'b1
) (
    input  wire        sysclk,
    output logic [3:0] led,

`ifdef MIPI_CAPTURE_PORTS
    input  wire        capture_aclk,
    input  wire        capture_aresetn,
    output logic [(COLOR_CAPTURE ? 24 : 8)-1:0] m_axis_capture_tdata,
    output logic       m_axis_capture_tvalid,
    input  wire        m_axis_capture_tready,
    output logic       m_axis_capture_tlast,
    output logic [0:0] m_axis_capture_tuser,
    output logic [31:0] capture_debug,
`endif

`ifdef MIPI_VDMA_LOOP_PORTS
    output wire        pix_clk_out,
    output wire        pix_aresetn_out,
    input  wire [23:0] s_axis_hdmi_tdata,
    input  wire        s_axis_hdmi_tvalid,
    output wire        s_axis_hdmi_tready,
    input  wire        s_axis_hdmi_tlast,
    input  wire [0:0]  s_axis_hdmi_tuser,
    input  wire [7:0]  debug_page_sel,
    input  wire [31:0] sccb_rt_write_word_in,
    output logic [31:0] sccb_rt_write_status_out,
    input  wire [31:0] idelay_runtime_word_in,
    output logic [31:0] idelay_runtime_status_out,
    input  wire [31:0] bitslip_runtime_word_in,
    output logic [31:0] bitslip_runtime_status_out,
    input  wire [31:0] frame_lines_runtime_word_in,
    output logic [31:0] frame_lines_runtime_status_out,
    input  wire [31:0] rawcap_word_in,
    output logic [31:0] rawcap_status_out,
`endif

    input  wire        dphy_hs_clock_clk_p,
    input  wire        dphy_hs_clock_clk_n,
    input  wire [1:0]  dphy_data_hs_p,
    input  wire [1:0]  dphy_data_hs_n,
    input  wire        dphy_clk_lp_p,
    input  wire        dphy_clk_lp_n,
    input  wire [1:0]  dphy_data_lp_p,
    input  wire [1:0]  dphy_data_lp_n,

    output logic       cam_clk,
    output logic       cam_gpio,
    inout  wire        cam_scl,
    inout  wire        cam_sda,

    input  wire        hdmi_tx_hpd,
    output wire        hdmi_tx_clk_p,
    output wire        hdmi_tx_clk_n,
    output wire [2:0]  hdmi_tx_p,
    output wire [2:0]  hdmi_tx_n,
    output wire        hdmi_tx_scl,
    inout  wire        hdmi_tx_sda,
    output wire        hdmi_tx_cec
);

    logic [7:0] rst_cnt = 8'h00;
    logic rst_n;

    always_ff @(posedge sysclk) begin
        if (!(&rst_cnt)) begin
            rst_cnt <= rst_cnt + 8'h01;
        end
    end

    assign rst_n = &rst_cnt;

    logic        sccb_rt_test_pattern_valid;
    logic        sccb_rt_test_pattern_enable;
    logic        sccb_rt_test_pattern_ready;
    logic        sccb_rt_test_pattern_done;
    logic        sccb_rt_test_pattern_error;
    logic [7:0]  sccb_rt_test_pattern_value;
    logic [7:0]  sccb_rt_ack_error_count;

    logic        sccb_rt_reg_write_valid;
    logic [15:0] sccb_rt_reg_write_addr;
    logic [7:0]  sccb_rt_reg_write_value;
    logic        sccb_rt_reg_write_ready;
    logic        sccb_rt_reg_write_done;
    logic        sccb_rt_reg_write_error;
    logic        sccb_rt_reg_write_busy;
    logic [7:0]  sccb_rt_reg_write_ack_err_count;
    logic [15:0] sccb_rt_reg_write_last_addr;

    logic        sccb_rt_reg_read_valid;
    logic [15:0] sccb_rt_reg_read_addr;
    logic        sccb_rt_reg_read_ready;
    logic        sccb_rt_reg_read_done;
    logic        sccb_rt_reg_read_error;
    logic [7:0]  sccb_rt_reg_read_data;
    logic [15:0] sccb_rt_reg_read_last_addr;

`ifdef MIPI_VDMA_LOOP_PORTS
    logic [7:0] debug_control_sys_meta;
    logic [7:0] debug_control_sys;
    logic [5:0] debug_page_sel_sys;
    logic       sccb_rt_test_pattern_apply_sys_d;
    logic       sccb_rt_test_pattern_pending;
    logic       sccb_rt_test_pattern_enable_latched;

    logic [31:0] sccb_rt_write_word_meta;
    logic [31:0] sccb_rt_write_word_sys;
    logic        sccb_rt_reg_write_apply_sys_d;
    logic        sccb_rt_reg_write_pending;
    logic        sccb_rt_reg_read_apply_sys_d;
    logic        sccb_rt_reg_read_pending;

    logic [31:0] idelay_runtime_word_meta;
    logic [31:0] idelay_runtime_word_sys;
    logic        idelay_runtime_apply_sys_d;
    logic [4:0]  idelay_runtime_tap_sys;
    logic [4:0]  idelay_runtime_tap_lane1_sys;
    logic [4:0]  idelay_runtime_tap_clk_sys;   // clock-lane IDELAY tap (cal), GPIO word [20:16]
    logic [15:0] idelay_runtime_load_count;

    logic [31:0] bitslip_runtime_word_meta;
    logic [31:0] bitslip_runtime_word_sys;
    logic        bitslip_runtime_apply_sys_d;
    logic [2:0]  bitslip_runtime_phase_sys;
    logic [2:0]  bitslip_runtime_phase_lane1_sys;
    logic        bitslip_runtime_lane1_sweep_en_sys;
    logic [15:0] bitslip_runtime_load_count;

    logic [31:0] frame_lines_runtime_word_meta;
    logic [31:0] frame_lines_runtime_word_sys;
    logic        frame_lines_runtime_apply_sys_d;
    logic [15:0] frame_lines_runtime_value_sys;
    logic [15:0] frame_lines_runtime_load_count;
    wire         cfg_use_lsle_sys = frame_lines_runtime_word_sys[16];
    // Runtime-configurable EXPECTED_LONG_DT, bits [23:17] of frame_lines_runtime word
    wire [7:0]   expected_long_dt_sys = {1'b0, frame_lines_runtime_word_sys[23:17]};
    // bit[30] = cfg_sof_synth: open a frame from the first LS when the chip's FS
    // never arrives (D-PHY lane supervisor enabled, fs=0; diary 2026-06-13).
    wire         cfg_sof_synth_sys = frame_lines_runtime_word_sys[30];
    // bit[27] = rt_bufr_clr: software-driven BUFR.CLR re-roll for the byte-phase
    // calibration (toggle 1->0 to re-roll the /4 phase in us). Direct level (no
    // apply strobe) so the toggle takes effect immediately. 2026-06-15.
    wire         rt_bufr_clr_sys = frame_lines_runtime_word_sys[27];
    // bit[31] = cfg_force_expected: in lsle mode, force-close the frame at exactly
    // cfg_expected_frame_lines (480) so the VDMA/VTC sees a constant-height frame
    // and genlock locks (live-HDMI roll fix; diary 2026-06-16). FS still re-anchors
    // the top each frame. Runtime so on/off + target height tune without a rebuild.
    wire         cfg_force_expected_sys = frame_lines_runtime_word_sys[31];
    // cfg_long_as_line: idelay-word spare bit[26] (frame_lines word is full). When
    // set, frame_state delivers a long without a preceding LS as a row (recovers
    // the scattered no-LS-reject band). Read direct (no apply) so it A/Bs live.
    wire         cfg_long_as_line_sys = idelay_runtime_word_sys[26];
    // Phase 2 processing-slot select: idelay GPIO spare bits[24:21] (direct read, like
    // cfg_settle_blank). 0-7 = point ops (axis_rgb_proc_slot): 0=pass 1=invert 2=gray
    // 3=BGR 4=thresh 5/6/7=R/G/B. 8-11 = 3x3 conv (axis_rgb_conv3x3): 8=pass 9=Gaussian
    // 10=Sobel 11=sharpen. The point slot and the conv are chained; each bypasses when
    // the other's range is selected (cfg[3] picks point vs conv).
    wire [3:0]   cfg_proc_op_sys = idelay_runtime_word_sys[24:21];
    // Phase 2b: runtime-programmable 3x3 kernel. 9 signed coeffs + 4-bit shift, loaded
    // over the SCCB runtime-write path with a reserved address 0xFE0i (i=0..8 = coeff,
    // i=9 = shift) intercepted in the sccb decode below (NOT issued to the chip). Reset
    // = identity ({0,0,0,0,1,0,0,0,0}, shift 0) = passthrough until a kernel is loaded.
    logic [7:0] conv_coeff_reg [0:8];
    logic [3:0] conv_shift_reg;
    wire [71:0] conv_coeffs_packed =
        {conv_coeff_reg[8], conv_coeff_reg[7], conv_coeff_reg[6],
         conv_coeff_reg[5], conv_coeff_reg[4], conv_coeff_reg[3],
         conv_coeff_reg[2], conv_coeff_reg[1], conv_coeff_reg[0]};

    // DoG dual-kernel (op 12, plan_dog_dual_kernel_20260624): a general 5x5 (B branch)
    // runs in PARALLEL with the 3x3 (A branch) and is combined as clamp(a*A - b*B + off).
    // Reserved-address loads on the 0xFE page (NOT chip SCCB): 0x20-0x38 = 5x5 coeff,
    // 0x39 = 5x5 shift, 0x40 mode, 0x41 alpha, 0x42 beta, 0x43 shift, 0x44 offset.
    logic [7:0] conv5_coeff_reg [0:24];
    logic [3:0] conv5_shift_reg;
    logic [1:0] dog_mode_reg;
    logic [7:0] dog_alpha_reg, dog_beta_reg;
    logic [3:0] dog_shift_reg;
    logic [7:0] dog_offset_reg;
    logic [1:0] dog_abs_reg;                 // [0]=conv3x3(A) |grad|, [1]=conv5x5(B) |grad| (0xFE45)
    // Point-op chaining around the conv stage (plan 2026-06-25): a point op runs in the
    // slot BEFORE conv (pre_op, applied even in conv mode) and a second slot AFTER the
    // conv/mux (post_op) -> e.g. binarize->Sobel (pre=4) or Sobel->binarize (post=4) in
    // one pass, any order, generic for all point ops 0-7. Loaded on the 0xFE page:
    //   0x46 pre_op (4-bit: 0 pass,1-7 point,8 gaussian,9 median 3x3), 0x47 pre_thresh (8-bit),
    //   0x48 post_op (3-bit), 0x49 post_thresh. The PRE stage is axis_rgb_prefilter (3x3 spatial
    //   denoise + point ops); POST stays axis_rgb_proc_slot (point ops).
    // Defaults (pre_op/post_op=0 passthrough, thresh=128) keep the build bit-identical.
    logic [3:0] pre_op_reg;
    logic [2:0] post_op_reg;
    logic [7:0] pre_thresh_reg, post_thresh_reg;
    // Dither stage AFTER post (0xFE4A): [0]=en [1]=mode(0 ordered/1 random) [4:2]=bits/ch. 0=off.
    logic [7:0] dither_ctrl_reg;
    wire [199:0] conv5_coeffs_packed;
    for (genvar gk = 0; gk < 25; gk++)
        assign conv5_coeffs_packed[gk*8 +: 8] = conv5_coeff_reg[gk];
    wire dog_en_sys = (cfg_proc_op_sys == 4'd12);   // op 12 = DoG parallel dual-kernel

    // Multi-scale cascade (plan_cascade_multiscale_20260624): two separable 5x5 blur stages
    // S2,S3 after conv5x5(=S1). op 13/14/15 output tap t1(5x5)/t2(9x9)/t3(13x13) = runtime-
    // variable effective kernel size. Reserved 0xFE: S2 h=0x50-54/hsh=0x55/v=0x56-5A/vsh=0x5B,
    // S3 h=0x60-64/hsh=0x65/v=0x66-6A/vsh=0x6B. Reset = identity (passthrough).
    logic [7:0] s2_h_reg [0:4], s2_v_reg [0:4], s3_h_reg [0:4], s3_v_reg [0:4];
    logic [3:0] s2_hsh_reg, s2_vsh_reg, s3_hsh_reg, s3_vsh_reg;
    wire [39:0] s2_h_pk, s2_v_pk, s3_h_pk, s3_v_pk;
    for (genvar gs = 0; gs < 5; gs++) begin : g_seppack
        assign s2_h_pk[gs*8 +: 8] = s2_h_reg[gs];
        assign s2_v_pk[gs*8 +: 8] = s2_v_reg[gs];
        assign s3_h_pk[gs*8 +: 8] = s3_h_reg[gs];
        assign s3_v_pk[gs*8 +: 8] = s3_v_reg[gs];
    end
    wire conv5_en_sys = (cfg_proc_op_sys >= 4'd12);  // S1 active for DoG(12) + cascade(13-15)

    assign sccb_rt_test_pattern_valid = sccb_rt_test_pattern_pending && sccb_rt_test_pattern_ready;
    assign sccb_rt_test_pattern_enable = sccb_rt_test_pattern_enable_latched;

    assign sccb_rt_reg_write_valid = sccb_rt_reg_write_pending && sccb_rt_reg_write_ready
                                       && !sccb_rt_test_pattern_pending;
    assign sccb_rt_reg_read_valid  = sccb_rt_reg_read_pending && sccb_rt_reg_read_ready
                                       && !sccb_rt_test_pattern_pending
                                       && !sccb_rt_reg_write_pending;

    always_ff @(posedge sysclk) begin
        if (!rst_n) begin
            debug_control_sys_meta <= 8'h00;
            debug_control_sys <= 8'h00;
            debug_page_sel_sys <= 6'h00;
            sccb_rt_test_pattern_apply_sys_d <= 1'b0;
            sccb_rt_test_pattern_pending <= 1'b0;
            sccb_rt_test_pattern_enable_latched <= OV5640_TEST_PATTERN_ENABLE;
            sccb_rt_write_word_meta <= 32'h00000000;
            sccb_rt_write_word_sys <= 32'h00000000;
            sccb_rt_reg_write_apply_sys_d <= 1'b0;
            sccb_rt_reg_write_pending <= 1'b0;
            sccb_rt_reg_write_addr <= 16'h0000;
            sccb_rt_reg_write_value <= 8'h00;
            for (int k = 0; k < 9; k++) conv_coeff_reg[k] <= (k == 4) ? 8'h01 : 8'h00;  // identity
            conv_shift_reg <= 4'h0;
            for (int k = 0; k < 25; k++) conv5_coeff_reg[k] <= (k == 12) ? 8'h01 : 8'h00;  // identity
            conv5_shift_reg <= 4'h0;
            dog_mode_reg <= 2'd0; dog_alpha_reg <= 8'h01; dog_beta_reg <= 8'h01;
            dog_shift_reg <= 4'h0; dog_offset_reg <= 8'd128; dog_abs_reg <= 2'd0;
            pre_op_reg <= 4'd0; post_op_reg <= 3'd0;                    // passthrough (old behaviour)
            pre_thresh_reg <= 8'd128; post_thresh_reg <= 8'd128;        // old hard-coded threshold level
            dither_ctrl_reg <= 8'h00;                                   // dither off (bit-identical)
            for (int k = 0; k < 5; k++) begin                          // S2/S3 identity (bypass)
                s2_h_reg[k] <= (k==2)?8'h01:8'h00; s2_v_reg[k] <= (k==2)?8'h01:8'h00;
                s3_h_reg[k] <= (k==2)?8'h01:8'h00; s3_v_reg[k] <= (k==2)?8'h01:8'h00;
            end
            s2_hsh_reg <= 4'h0; s2_vsh_reg <= 4'h0; s3_hsh_reg <= 4'h0; s3_vsh_reg <= 4'h0;
            sccb_rt_reg_read_apply_sys_d <= 1'b0;
            sccb_rt_reg_read_pending <= 1'b0;
            sccb_rt_reg_read_addr <= 16'h0000;
            idelay_runtime_word_meta <= 32'h00000000;
            idelay_runtime_word_sys <= 32'h00000000;
            idelay_runtime_apply_sys_d <= 1'b0;
            idelay_runtime_tap_sys <= PROBE_IDELAY_TAP[4:0];
            idelay_runtime_tap_lane1_sys <= PROBE_IDELAY_TAP[4:0];
            idelay_runtime_tap_clk_sys <= PROBE_IDELAY_TAP[4:0];
            idelay_runtime_load_count <= 16'h0000;
            bitslip_runtime_word_meta <= 32'h00000000;
            bitslip_runtime_word_sys <= 32'h00000000;
            bitslip_runtime_apply_sys_d <= 1'b0;
            bitslip_runtime_phase_sys <= 3'd6;
            bitslip_runtime_phase_lane1_sys <= 3'd6;
            bitslip_runtime_lane1_sweep_en_sys <= PROBE_LANE1_BITSLIP_SWEEP;
            bitslip_runtime_load_count <= 16'h0000;
            // bit 25 drives cam_gpio (RESETB). Default = 1 so chip is OUT of reset
            // at bitstream load (matches legacy hardcoded `cam_gpio = 1'b1` behavior).
            // PYNQ can later pulse bit 25 to 0 for HW reset.
            frame_lines_runtime_word_meta <= 32'h02000000;
            frame_lines_runtime_word_sys <= 32'h02000000;
            frame_lines_runtime_apply_sys_d <= 1'b0;
            frame_lines_runtime_value_sys <= 16'd480;
            frame_lines_runtime_load_count <= 16'h0000;
        end else begin
            debug_control_sys_meta <= debug_page_sel;
            debug_control_sys <= debug_control_sys_meta;
            debug_page_sel_sys <= {debug_control_sys[7], debug_control_sys[4:0]};
            sccb_rt_test_pattern_apply_sys_d <= debug_control_sys[6];
            if (debug_control_sys[6] && !sccb_rt_test_pattern_apply_sys_d) begin
                sccb_rt_test_pattern_enable_latched <= debug_control_sys[5];
                sccb_rt_test_pattern_pending <= 1'b1;
            end else if (sccb_rt_test_pattern_valid) begin
                sccb_rt_test_pattern_pending <= 1'b0;
            end

            sccb_rt_write_word_meta <= sccb_rt_write_word_in;
            sccb_rt_write_word_sys <= sccb_rt_write_word_meta;
            sccb_rt_reg_write_apply_sys_d <= sccb_rt_write_word_sys[24];
            if (sccb_rt_write_word_sys[24] && !sccb_rt_reg_write_apply_sys_d) begin
                if (sccb_rt_write_word_sys[15:8] == 8'hFE) begin
                    // reserved 0xFE page: load kernels/combiner locally (NOT a chip SCCB write)
                    //   0x00-0x08 3x3 coeff, 0x09 3x3 shift
                    //   0x20-0x38 5x5 coeff, 0x39 5x5 shift
                    //   0x40 mode, 0x41 alpha, 0x42 beta, 0x43 shift, 0x44 offset
                    if (sccb_rt_write_word_sys[7:0] == 8'h09)
                        conv_shift_reg <= sccb_rt_write_word_sys[19:16];
                    else if (sccb_rt_write_word_sys[7:0] <= 8'h08)
                        conv_coeff_reg[sccb_rt_write_word_sys[3:0]] <= sccb_rt_write_word_sys[23:16];
                    else if (sccb_rt_write_word_sys[7:0] == 8'h39)
                        conv5_shift_reg <= sccb_rt_write_word_sys[19:16];
                    else if (sccb_rt_write_word_sys[7:0] >= 8'h20 && sccb_rt_write_word_sys[7:0] <= 8'h38)
                        conv5_coeff_reg[sccb_rt_write_word_sys[7:0] - 8'h20] <= sccb_rt_write_word_sys[23:16];
                    else if (sccb_rt_write_word_sys[7:0] == 8'h40) dog_mode_reg   <= sccb_rt_write_word_sys[17:16];
                    else if (sccb_rt_write_word_sys[7:0] == 8'h41) dog_alpha_reg  <= sccb_rt_write_word_sys[23:16];
                    else if (sccb_rt_write_word_sys[7:0] == 8'h42) dog_beta_reg   <= sccb_rt_write_word_sys[23:16];
                    else if (sccb_rt_write_word_sys[7:0] == 8'h43) dog_shift_reg  <= sccb_rt_write_word_sys[19:16];
                    else if (sccb_rt_write_word_sys[7:0] == 8'h44) dog_offset_reg <= sccb_rt_write_word_sys[23:16];
                    else if (sccb_rt_write_word_sys[7:0] == 8'h45) dog_abs_reg    <= sccb_rt_write_word_sys[17:16];
                    // point-op chaining: pre_op/pre_thresh (before conv), post_op/post_thresh (after conv)
                    else if (sccb_rt_write_word_sys[7:0] == 8'h46) pre_op_reg      <= sccb_rt_write_word_sys[19:16];
                    else if (sccb_rt_write_word_sys[7:0] == 8'h47) pre_thresh_reg  <= sccb_rt_write_word_sys[23:16];
                    else if (sccb_rt_write_word_sys[7:0] == 8'h48) post_op_reg     <= sccb_rt_write_word_sys[18:16];
                    else if (sccb_rt_write_word_sys[7:0] == 8'h49) post_thresh_reg <= sccb_rt_write_word_sys[23:16];
                    else if (sccb_rt_write_word_sys[7:0] == 8'h4A) dither_ctrl_reg <= sccb_rt_write_word_sys[23:16];
                    // cascade S2 (separable): h 0x50-54, hsh 0x55, v 0x56-5A, vsh 0x5B
                    else if (sccb_rt_write_word_sys[7:0] >= 8'h50 && sccb_rt_write_word_sys[7:0] <= 8'h54)
                        s2_h_reg[sccb_rt_write_word_sys[7:0] - 8'h50] <= sccb_rt_write_word_sys[23:16];
                    else if (sccb_rt_write_word_sys[7:0] == 8'h55) s2_hsh_reg <= sccb_rt_write_word_sys[19:16];
                    else if (sccb_rt_write_word_sys[7:0] >= 8'h56 && sccb_rt_write_word_sys[7:0] <= 8'h5A)
                        s2_v_reg[sccb_rt_write_word_sys[7:0] - 8'h56] <= sccb_rt_write_word_sys[23:16];
                    else if (sccb_rt_write_word_sys[7:0] == 8'h5B) s2_vsh_reg <= sccb_rt_write_word_sys[19:16];
                    // cascade S3 (separable): h 0x60-64, hsh 0x65, v 0x66-6A, vsh 0x6B
                    else if (sccb_rt_write_word_sys[7:0] >= 8'h60 && sccb_rt_write_word_sys[7:0] <= 8'h64)
                        s3_h_reg[sccb_rt_write_word_sys[7:0] - 8'h60] <= sccb_rt_write_word_sys[23:16];
                    else if (sccb_rt_write_word_sys[7:0] == 8'h65) s3_hsh_reg <= sccb_rt_write_word_sys[19:16];
                    else if (sccb_rt_write_word_sys[7:0] >= 8'h66 && sccb_rt_write_word_sys[7:0] <= 8'h6A)
                        s3_v_reg[sccb_rt_write_word_sys[7:0] - 8'h66] <= sccb_rt_write_word_sys[23:16];
                    else if (sccb_rt_write_word_sys[7:0] == 8'h6B) s3_vsh_reg <= sccb_rt_write_word_sys[19:16];
                end else begin
                    sccb_rt_reg_write_addr <= sccb_rt_write_word_sys[15:0];
                    sccb_rt_reg_write_value <= sccb_rt_write_word_sys[23:16];
                    sccb_rt_reg_write_pending <= 1'b1;
                end
            end else if (sccb_rt_reg_write_valid) begin
                sccb_rt_reg_write_pending <= 1'b0;
            end

            sccb_rt_reg_read_apply_sys_d <= sccb_rt_write_word_sys[26];
            if (sccb_rt_write_word_sys[26] && !sccb_rt_reg_read_apply_sys_d) begin
                sccb_rt_reg_read_addr <= sccb_rt_write_word_sys[15:0];
                sccb_rt_reg_read_pending <= 1'b1;
            end else if (sccb_rt_reg_read_valid) begin
                sccb_rt_reg_read_pending <= 1'b0;
            end

            idelay_runtime_word_meta <= idelay_runtime_word_in;
            idelay_runtime_word_sys <= idelay_runtime_word_meta;
            idelay_runtime_apply_sys_d <= idelay_runtime_word_sys[24];
            if (idelay_runtime_word_sys[24] && !idelay_runtime_apply_sys_d) begin
                idelay_runtime_tap_sys       <= idelay_runtime_word_sys[4:0];
                idelay_runtime_tap_lane1_sys <= idelay_runtime_word_sys[12:8];
                idelay_runtime_tap_clk_sys   <= idelay_runtime_word_sys[20:16];
                if (idelay_runtime_load_count != 16'hffff) begin
                    idelay_runtime_load_count <= idelay_runtime_load_count + 16'd1;
                end
            end

            bitslip_runtime_word_meta <= bitslip_runtime_word_in;
            bitslip_runtime_word_sys  <= bitslip_runtime_word_meta;
            bitslip_runtime_apply_sys_d <= bitslip_runtime_word_sys[24];
            if (bitslip_runtime_word_sys[24] && !bitslip_runtime_apply_sys_d) begin
                bitslip_runtime_phase_sys          <= bitslip_runtime_word_sys[2:0];
                bitslip_runtime_phase_lane1_sys    <= bitslip_runtime_word_sys[10:8];
                bitslip_runtime_lane1_sweep_en_sys <= bitslip_runtime_word_sys[16];
                if (bitslip_runtime_load_count != 16'hffff) begin
                    bitslip_runtime_load_count <= bitslip_runtime_load_count + 16'd1;
                end
            end

            frame_lines_runtime_word_meta <= frame_lines_runtime_word_in;
            frame_lines_runtime_word_sys  <= frame_lines_runtime_word_meta;
            frame_lines_runtime_apply_sys_d <= frame_lines_runtime_word_sys[24];
            if (frame_lines_runtime_word_sys[24] && !frame_lines_runtime_apply_sys_d) begin
                frame_lines_runtime_value_sys <= frame_lines_runtime_word_sys[15:0];
                if (frame_lines_runtime_load_count != 16'hffff) begin
                    frame_lines_runtime_load_count <= frame_lines_runtime_load_count + 16'd1;
                end
            end
        end
    end

    always_comb begin
        sccb_rt_write_status_out = 32'h00000000;
        // Write status (bits[4:0] preserved for backward compat)
        sccb_rt_write_status_out[0] = sccb_rt_reg_write_pending;
        sccb_rt_write_status_out[1] = sccb_rt_reg_write_ready;
        sccb_rt_write_status_out[2] = sccb_rt_reg_write_done;
        sccb_rt_write_status_out[3] = sccb_rt_reg_write_error;
        sccb_rt_write_status_out[4] = sccb_rt_reg_write_busy;
        // Read status (new, bits[7:5])
        sccb_rt_write_status_out[5] = sccb_rt_reg_read_done;
        sccb_rt_write_status_out[6] = sccb_rt_reg_read_error;
        sccb_rt_write_status_out[7] = sccb_rt_reg_read_ready;
        // Read data (new, bits[15:8] — matches legacy Python expectation (st >> 8) & 0xFF)
        sccb_rt_write_status_out[15:8] = sccb_rt_reg_read_data;
        // Write debug (relocated):
        sccb_rt_write_status_out[23:16] = sccb_rt_reg_write_ack_err_count;
        // Last addr — keep lower byte of write_last_addr (most-used bits in 0x3000-0x5xxx)
        sccb_rt_write_status_out[31:24] = sccb_rt_reg_write_last_addr[7:0];
    end

    always_comb begin
        idelay_runtime_status_out = 32'h00000000;
        idelay_runtime_status_out[4:0]   = idelay_runtime_tap_sys;
        idelay_runtime_status_out[12:8]  = idelay_runtime_tap_lane1_sys;
        idelay_runtime_status_out[20:16] = idelay_runtime_tap_clk_sys;
        idelay_runtime_status_out[31:21] = idelay_runtime_load_count[10:0];
    end

    always_comb begin
        bitslip_runtime_status_out = 32'h00000000;
        bitslip_runtime_status_out[2:0]   = bitslip_runtime_phase_sys;
        bitslip_runtime_status_out[10:8]  = bitslip_runtime_phase_lane1_sys;
        bitslip_runtime_status_out[14:12] = phy_lane1_target_bitslip_phase;
        bitslip_runtime_status_out[16]    = bitslip_runtime_lane1_sweep_en_sys;
        bitslip_runtime_status_out[31:17] = bitslip_runtime_load_count[14:0];
    end

    always_comb begin
        frame_lines_runtime_status_out = 32'h00000000;
        frame_lines_runtime_status_out[15:0]  = frame_lines_runtime_value_sys;
        frame_lines_runtime_status_out[16]     = cfg_use_lsle_sys;
        // [23:17] = active DT readback so PYNQ can verify the runtime register reached RTL
        frame_lines_runtime_status_out[23:17] = expected_long_dt_sys[6:0];
        frame_lines_runtime_status_out[31:24] = frame_lines_runtime_load_count[7:0];
    end
`else
    wire [5:0] debug_page_sel_sys = 6'h00;
    wire sccb_rt_test_pattern_pending = 1'b0;
    assign sccb_rt_test_pattern_valid = 1'b0;
    assign sccb_rt_test_pattern_enable = 1'b0;
    assign sccb_rt_reg_write_valid = 1'b0;
    assign sccb_rt_reg_write_addr = 16'h0000;
    assign sccb_rt_reg_write_value = 8'h00;
    assign sccb_rt_reg_read_valid = 1'b0;
    assign sccb_rt_reg_read_addr = 16'h0000;
    wire [4:0] idelay_runtime_tap_sys = PROBE_IDELAY_TAP[4:0];
    wire [4:0] idelay_runtime_tap_lane1_sys = PROBE_IDELAY_TAP[4:0];
    wire [2:0] bitslip_runtime_phase_sys = 3'd6;
    wire [2:0] bitslip_runtime_phase_lane1_sys = 3'd6;
    wire [15:0] frame_lines_runtime_value_sys = 16'd480;
    wire        cfg_use_lsle_sys = 1'b0;
    wire [7:0]  expected_long_dt_sys = 8'h00;
`endif

    logic        cam_reset_released;
    logic        sccb_resetn;

    assign cam_reset_released = rst_n;
    // cam_gpio = OV5640 RESETB pin (active LOW). frame_lines_runtime_word[25]
    // controls it directly:
    //   0 = RESETB asserted (chip in reset) → LED0 OFF, no MIPI output
    //   1 = RESETB released (chip operating) → LED0 ON, chip streams
    // Default bit 25 = 0 means chip in reset! So PYNQ must set bit 25 = 1 first.
    // HW reset pulse: bit25 1 → 0 (≥1ms) → 1 (wait ≥1ms before SCCB).
    assign cam_gpio = frame_lines_runtime_word_sys[25];
    assign sccb_resetn = rst_n;

    logic refclk_200_unbuf;
    logic refclk_200;
    logic cam_clk_25m_unbuf;
    logic cam_clk_25m;
    logic ref_pll_feedback;
    logic ref_pll_locked;

    // VCO = sysclk(125 MHz) * 8 = 1000 MHz
    // CLKOUT0: 1000/5 = 200 MHz (IDELAY refclk)
    // CLKOUT1: 1000/40 = 25 MHz (cam_clk to OV5640 — close to Linux's 24 MHz reference)
    PLLE2_BASE #(
        .BANDWIDTH("OPTIMIZED"),
        .CLKFBOUT_MULT(8),
        .CLKFBOUT_PHASE(0.0),
        .CLKIN1_PERIOD(8.000),
        .CLKOUT0_DIVIDE(5),
        .CLKOUT0_DUTY_CYCLE(0.5),
        .CLKOUT0_PHASE(0.0),
        .CLKOUT1_DIVIDE(40),
        .CLKOUT1_DUTY_CYCLE(0.5),
        .CLKOUT1_PHASE(0.0),
        .DIVCLK_DIVIDE(1),
        .REF_JITTER1(0.010),
        .STARTUP_WAIT("FALSE")
    ) u_refclk_pll (
        .CLKIN1(sysclk),
        .CLKFBIN(ref_pll_feedback),
        .RST(!rst_n),
        .PWRDWN(1'b0),
        .CLKFBOUT(ref_pll_feedback),
        .CLKOUT0(refclk_200_unbuf),
        .CLKOUT1(cam_clk_25m_unbuf),
        .CLKOUT2(),
        .CLKOUT3(),
        .CLKOUT4(),
        .CLKOUT5(),
        .LOCKED(ref_pll_locked)
    );

    BUFG u_refclk_200_bufg (.I(refclk_200_unbuf), .O(refclk_200));
    BUFG u_cam_clk_25m_bufg (.I(cam_clk_25m_unbuf), .O(cam_clk_25m));

    // Forward 25 MHz to the OV5640 sensor as XCLK reference
    assign cam_clk = cam_clk_25m;

    logic sccb_busy;
    logic sccb_done;
    logic sccb_error;
    logic [7:0] sccb_chip_id_high;
    logic [7:0] sccb_chip_id_low;
    logic [7:0] sccb_ack_error_count;
    logic [8:0] sccb_step_index;  // 2026-05-30: widened 8b -> 9b for LINUX_ANALOG insertion (LAST_STEP 255 -> 282)
    logic [7:0] sccb_rd_mipi_ctrl_300e;
    logic [7:0] sccb_rd_mipi_ctrl_4800;
    logic [7:0] sccb_rd_mipi_ctrl_4805;
    logic [7:0] sccb_rd_mipi_ctrl_4837;
    logic [7:0] sccb_rd_format_ctrl_4300;
    logic [7:0] sccb_rd_isp_format_501f;
    logic [7:0] sccb_rd_isp_ctrl_5000;
    logic [7:0] sccb_rd_isp_ctrl_5001;
    logic [7:0] sccb_rd_timing_ctrl_3824;
    logic [7:0] sccb_rd_jpeg_ctrl_4407;
    logic [7:0] sccb_rd_mipi_ctrl_440e;
    logic [7:0] sccb_rd_vfifo_ctrl_460b;
    logic [7:0] sccb_rd_vfifo_ctrl_460c;
    logic [7:0] sccb_rd_awb_5189;
    logic [7:0] sccb_rd_output_width_high_3808;
    logic [7:0] sccb_rd_output_width_low_3809;
    logic [7:0] sccb_rd_output_height_high_380a;
    logic [7:0] sccb_rd_output_height_low_380b;
    logic [7:0] sccb_rd_aec_manual_3503;
    logic [7:0] sccb_rd_aec_ctrl_3a13;
    logic [7:0] sccb_rd_aec_gain_ceiling_high_3a18;
    logic [7:0] sccb_rd_aec_gain_ceiling_low_3a19;
    localparam logic [8:0] OV5640_SCCB_LAST_STEP = 9'd260;  // 2026-05-30: ROM-based init (261 entries 0..260)

`ifdef MIPI_VDMA_LOOP_PORTS
    wire sccb_sda_drive_low_w;
    wire sccb_scl_drive_low_w;
    wire sda_pad_o;
    wire scl_pad_o;
    wire sda_to_sccb;
    wire scl_to_sccb;

    IOBUF u_iobuf_cam_sda (
        .IO(cam_sda),
        .I(1'b0),
        .T(!sccb_sda_drive_low_w),
        .O(sda_pad_o)
    );
    IOBUF u_iobuf_cam_scl (
        .IO(cam_scl),
        .I(1'b0),
        .T(!sccb_scl_drive_low_w),
        .O(scl_pad_o)
    );

    assign sda_to_sccb = sccb_sda_drive_low_w ? 1'b0 : sda_pad_o;
    assign scl_to_sccb = sccb_scl_drive_low_w ? 1'b0 : scl_pad_o;
`endif

    ov5640_sccb_init_probe #(
        .CLK_HZ(125_000_000),
        .I2C_HZ(100_000),
        .POWERUP_DELAY_MS(50),
        // 0x300E idle = 0x40 (was 0x44): match the verified runtime stream cycle
        // (flicker_exposure_sweep.stream_cycle_write: 0x40 -> format -> 0x45) and the
        // mainline reference (0x300E = on?0x45:0x40, ref line 272). The init ROM
        // writes 0x4300/0x501F between this idle (step44) and stream (step259), so a
        // 0x40->0x45 cycle latches the RGB565 format change at boot (avoids the
        // "monolithic init can't do RGB565" trap, memory ov5640_rgb565_requires_stream_cycle).
        .MIPI_CTRL_300E_IDLE_2LANE(8'h40),
        .MIPI_CTRL_300E_STREAM_2LANE(8'h45),
        .MIPI_CTRL_4800(OV5640_MIPI_CTRL_4800),
        .FORMAT_CTRL_4300(OV5640_FORMAT_CTRL_4300),
        .ISP_FORMAT_501F(OV5640_ISP_FORMAT_501F),
        .ISP_CTRL_5000(OV5640_ISP_CTRL_5000),
        .ISP_CTRL_5001(OV5640_ISP_CTRL_5001),
        .TEST_PATTERN_ENABLE(OV5640_TEST_PATTERN_ENABLE),
`ifdef MIPI_VDMA_LOOP_PORTS
        .USE_EXTERNAL_IOBUF(1'b1)
`else
        .USE_EXTERNAL_IOBUF(1'b0)
`endif
    ) u_ov5640_sccb_init_probe (
        .clk(sysclk),
        .rst_n(sccb_resetn),
        .rt_test_pattern_valid(sccb_rt_test_pattern_valid),
        .rt_test_pattern_enable(sccb_rt_test_pattern_enable),
        .rt_test_pattern_ready(sccb_rt_test_pattern_ready),
        .rt_test_pattern_done(sccb_rt_test_pattern_done),
        .rt_test_pattern_error(sccb_rt_test_pattern_error),
        .rt_test_pattern_value(sccb_rt_test_pattern_value),
        .rt_ack_error_count(sccb_rt_ack_error_count),
        .rt_reg_write_valid(sccb_rt_reg_write_valid),
        .rt_reg_write_addr(sccb_rt_reg_write_addr),
        .rt_reg_write_value(sccb_rt_reg_write_value),
        .rt_reg_write_ready(sccb_rt_reg_write_ready),
        .rt_reg_write_done(sccb_rt_reg_write_done),
        .rt_reg_write_error(sccb_rt_reg_write_error),
        .rt_reg_write_busy(sccb_rt_reg_write_busy),
        .rt_reg_write_ack_err_count(sccb_rt_reg_write_ack_err_count),
        .rt_reg_write_last_addr(sccb_rt_reg_write_last_addr),
        .rt_reg_read_valid(sccb_rt_reg_read_valid),
        .rt_reg_read_addr(sccb_rt_reg_read_addr),
        .rt_reg_read_ready(sccb_rt_reg_read_ready),
        .rt_reg_read_done(sccb_rt_reg_read_done),
        .rt_reg_read_error(sccb_rt_reg_read_error),
        .rt_reg_read_data(sccb_rt_reg_read_data),
        .rt_reg_read_last_addr(sccb_rt_reg_read_last_addr),
`ifdef MIPI_VDMA_LOOP_PORTS
        .cam_scl(scl_to_sccb),
        .cam_sda(sda_to_sccb),
        .scl_drive_low_o(sccb_scl_drive_low_w),
        .sda_drive_low_o(sccb_sda_drive_low_w),
`else
        .cam_scl(cam_scl),
        .cam_sda(cam_sda),
        .scl_drive_low_o(),
        .sda_drive_low_o(),
`endif
        .busy(sccb_busy),
        .done(sccb_done),
        .error(sccb_error),
        .chip_id_high(sccb_chip_id_high),
        .chip_id_low(sccb_chip_id_low),
        .ack_error_count(sccb_ack_error_count),
        .step_index(sccb_step_index),
        .rd_mipi_ctrl_300e(sccb_rd_mipi_ctrl_300e),
        .rd_mipi_ctrl_4800(sccb_rd_mipi_ctrl_4800),
        .rd_mipi_ctrl_4805(sccb_rd_mipi_ctrl_4805),
        .rd_mipi_ctrl_4837(sccb_rd_mipi_ctrl_4837),
        .rd_format_ctrl_4300(sccb_rd_format_ctrl_4300),
        .rd_isp_format_501f(sccb_rd_isp_format_501f),
        .rd_isp_ctrl_5000(sccb_rd_isp_ctrl_5000),
        .rd_isp_ctrl_5001(sccb_rd_isp_ctrl_5001),
        .rd_timing_ctrl_3824(sccb_rd_timing_ctrl_3824),
        .rd_jpeg_ctrl_4407(sccb_rd_jpeg_ctrl_4407),
        .rd_mipi_ctrl_440e(sccb_rd_mipi_ctrl_440e),
        .rd_vfifo_ctrl_460b(sccb_rd_vfifo_ctrl_460b),
        .rd_vfifo_ctrl_460c(sccb_rd_vfifo_ctrl_460c),
        .rd_awb_5189(sccb_rd_awb_5189),
        .rd_output_width_high_3808(sccb_rd_output_width_high_3808),
        .rd_output_width_low_3809(sccb_rd_output_width_low_3809),
        .rd_output_height_high_380a(sccb_rd_output_height_high_380a),
        .rd_output_height_low_380b(sccb_rd_output_height_low_380b),
        .rd_aec_manual_3503(sccb_rd_aec_manual_3503),
        .rd_aec_ctrl_3a13(sccb_rd_aec_ctrl_3a13),
        .rd_aec_gain_ceiling_high_3a18(sccb_rd_aec_gain_ceiling_high_3a18),
        .rd_aec_gain_ceiling_low_3a19(sccb_rd_aec_gain_ceiling_low_3a19)
    );

    logic phy_byte_clk;
    logic phy_idelayctrl_rdy;
    logic phy_hs_clk_seen;
    // D-PHY lane supervisor (opt-in via frame_lines_runtime_word[29]).
    logic       sup_bufr_clr;        // refclk_200 (ctl_clk) domain -> probe BUFR CLR
    logic       sup_serdes_rst;      // phy_byte_clk domain -> probe ISERDES RST
    logic       sup_rx_clk_active;   // phy_byte_clk domain (diagnostic)
    logic       sup_hs_settled;      // phy_byte_clk domain -> probe SoT gate
    logic [2:0] sup_clk_state;
    logic [2:0] sup_data_state;
    logic [7:0] sup_lock_cnt;
    logic [7:0] sup_settle_cnt;
    logic [7:0] sup_lost_cnt;        // HS-clock-lost (cHSClkLost) events
    (* ASYNC_REG = "TRUE" *) logic sup_enable_meta, sup_enable_byte;
    // cfg_hs_settle_gate (frame_lines_runtime_word[28], 2026-06-17): per-line
    // HS-SETTLE SoT gate in the legacy continuous path, decoupled from sup_enable.
    (* ASYNC_REG = "TRUE" *) logic sup_settle_gate_meta, sup_settle_gate_byte;
    // cfg_settle_blank_k (idelay_runtime_word[30:27], 2026-06-17): byte-domain
    // per-line settle blank (K byte_clk) + SoT-miss diagnostics.
    (* ASYNC_REG = "TRUE" *) logic [3:0] cfg_settle_blank_k_meta, cfg_settle_blank_k_byte;
    logic [15:0] phy_burst_count, phy_sot_burst_count;
    logic [31:0] phy_missed_burst;
    logic [15:0] phy_relock_latency, phy_relock_max;   // vblank-exit re-lock latency (byte_clk)
    // === HW deterministic-lock FSM (E2, 2026-06-19): RTL bitslip-sweep + /4
    //     re-roll + hold so a bare bitstream auto-locks on power-up (continuous
    //     only). Opt-in via bitslip_runtime_word[25]. refclk_200 domain (survives
    //     the BUFR.CLR re-roll). See dphy_hwlock_fsm.sv / plan_hwlock_fsm_20260619.
    // bit[25] enables; in a HWLOCK_DEFAULT_ON build the FSM is enabled by default
    // (power-on auto-lock) and bit[26] inhibits it at runtime (lock_mode fallback).
    // HWLOCK_DEFAULT_ON=0 build with bit26=0 -> exactly bit[25] (opt-in, as the
    // first E2 build).
    wire        cfg_hw_lock_sys = (bitslip_runtime_word_sys[25] | (HWLOCK_DEFAULT_ON != 1'b0))
                                  & ~bitslip_runtime_word_sys[26];
    (* ASYNC_REG = "TRUE" *) logic cfg_hw_lock_meta, cfg_hw_lock_ctl;
    logic [2:0] hwlock_p0, hwlock_p1;        // FSM swept bitslip target (refclk_200)
    logic       hwlock_bufr_clr_w;           // FSM /4 re-roll -> probe BUFR.CLR
    logic       hwlock_locked, hwlock_failed;
    logic [2:0] hwlock_dbg_state;
    logic [3:0] hwlock_dbg_reroll;
    logic [5:0] hwlock_dbg_combo;
    logic       hdr_ok_byte;                 // byte_clk windowed sync-header detector (1-bit lock quality)
    logic [13:0] hwlock_win_cnt;
    logic [7:0]  hwlock_hdr_cnt;
    (* ASYNC_REG = "TRUE" *) logic hdr_active_meta, hdr_active_ctl;  // hdr_ok_byte -> refclk_200
    // muxed bitslip target: FSM when cfg_hw_lock, else the GPIO/lock_mode path.
    wire [2:0]  bitslip_phase_eff       = cfg_hw_lock_ctl ? hwlock_p0 : bitslip_runtime_phase_sys;
    wire [2:0]  bitslip_phase_lane1_eff = cfg_hw_lock_ctl ? hwlock_p1 : bitslip_runtime_phase_lane1_sys;
    logic [1:0] phy_lane_sot_seen;
    logic [15:0] phy_stream_byte_data;
    logic [1:0] phy_stream_byte_keep;
    logic phy_stream_byte_valid;
    logic phy_stream_byte_sop;
    logic phy_stream_byte_eop;
    logic [2:0] phy_stream_pairing_active;
    logic [2:0] phy_stream_pairing_next;
    logic phy_sync_header_valid;
    logic phy_header_valid;
    logic [7:0] phy_header_di;
    logic [15:0] phy_header_wc;
    logic [7:0] phy_header_ecc;
    logic [7:0] phy_sync_header_di;
    logic [15:0] phy_sync_header_wc;
    logic [7:0] phy_sync_header_ecc;
    logic [2:0] phy_sync_header_bit_offset_lane0;
    logic [2:0] phy_sync_header_bit_offset_lane1;
    logic [2:0] phy_sync_header_pairing;
    logic [3:0] phy_sync_header_score;
    logic [5:0] phy_sync_header_syndrome;
    logic phy_sync_header_ecc_no_error;
    logic phy_sync_header_ecc_corrected;
    logic phy_sync_header_ecc_uncorrectable;
    logic [31:0] phy_sync_header_debug_word;
    logic [31:0] phy_stream_sop0_debug_word;
    logic [31:0] phy_stream_sop1_debug_word;
    logic phy_stream_sop_second_pending;
    logic [7:0] phy_trace_slot_sot_hit_lane0;
    logic [7:0] phy_trace_slot_sot_hit_lane1;
    logic [7:0][7:0] phy_trace_slot_lane0_raw;
    logic [7:0][7:0] phy_trace_slot_lane1_raw;
    logic [7:0][7:0] phy_trace_slot_lane0_candidate;
    logic [7:0][7:0] phy_trace_slot_lane1_candidate;
    logic [7:0][7:0] phy_trace_slot_lane0_aligned;
    logic [7:0][7:0] phy_trace_slot_lane1_aligned;
    logic [7:0] phy_live_trace_seq;
    logic [7:0] phy_live_trace_slot_valid;
    logic [7:0][7:0] phy_live_trace_slot_lane0_raw;
    logic [7:0][7:0] phy_live_trace_slot_lane1_raw;
    logic [7:0][7:0] phy_live_trace_slot_lane0_candidate;
    logic [7:0][7:0] phy_live_trace_slot_lane1_candidate;
    logic [7:0][7:0] phy_live_trace_slot_lane0_aligned;
    logic [7:0][7:0] phy_live_trace_slot_lane1_aligned;
    logic [7:0] phy_live_trace_slot_sot_hit_lane0;
    logic [7:0] phy_live_trace_slot_sot_hit_lane1;
    logic [7:0][2:0] phy_live_trace_slot_lane0_rotation;
    logic [7:0][2:0] phy_live_trace_slot_lane1_rotation;
    logic [15:0] phy_live_pair0_word0;
    logic [15:0] phy_live_pair0_word1;
    logic [7:0] phy_live_pair0_di;
    logic [2:0]  phy_lane1_target_bitslip_phase;
    logic [15:0] phy_live_pair0_wc;
    logic [7:0]  phy_live_pair0_ecc;
    logic [15:0] phy_stream_sop_word0;
    logic [15:0] phy_stream_sop_word1;
    logic [15:0] phy_stream_sop_wc;
    logic [7:0] phy_obs_compare_flags;

    dphy_hs_byte_probe #(
        .LANES(2),
        .SOT_WINDOW_BYTES(64),
        .SWEEP_HOLD_BYTES(16384),
        .SWEEP_ENABLE(1'b0),
        .FIXED_BITSLIP_PHASE(6),
        .FIXED_BITSLIP_PHASE_LANE1(6),
        .LANE1_BITSLIP_SWEEP_ENABLE(PROBE_LANE1_BITSLIP_SWEEP),
        .FIXED_TRANSFORM(1),
        .TRACE_TRIGGER_MODE(3),
        .IDELAY_TAP(PROBE_IDELAY_TAP),
        .IDELAY_REFCLK_MHZ(200.0),
        .EXPECTED_LONG_DT(8'h1e),
        .EXPECTED_LONG_WC(16'd1280),
        .MIN_SYNC_HEADER_SCORE(13),
        .SYNC_HEADER_SWEEP_BIT_OFFSETS(1'b0),
        .SYNC_HEADER_USE_ALIGNED_STREAM(1'b1),
        .STREAM_PAIRING(STREAM_PAIRING)
    ) u_dphy_hs_byte_probe (
        .rst_n(rst_n),
        .idelay_ref_clk(refclk_200),
        .idelay_ref_reset(!rst_n || !ref_pll_locked),
        .runtime_idelay_tap(idelay_runtime_tap_sys),
        .runtime_idelay_tap_lane1(idelay_runtime_tap_lane1_sys),
        .runtime_idelay_tap_clk(idelay_runtime_tap_clk_sys),
        .rt_bufr_clr(rt_bufr_clr_sys),
        .hwlock_bufr_clr(hwlock_bufr_clr_w),
        .runtime_bitslip_phase(bitslip_phase_eff),
        .runtime_bitslip_phase_lane1(bitslip_phase_lane1_eff),
        .runtime_lane1_sweep_enable(bitslip_runtime_lane1_sweep_en_sys),
        .runtime_expected_long_dt(expected_long_dt_sys),
        .sup_enable(sup_enable_byte),
        .sup_bufr_clr(sup_bufr_clr),
        .sup_serdes_rst(sup_serdes_rst),
        .sup_hs_settled(sup_hs_settled),
        .cfg_hs_settle_gate(sup_settle_gate_byte),
        .dphy_hs_clock_clk_p(dphy_hs_clock_clk_p),
        .dphy_hs_clock_clk_n(dphy_hs_clock_clk_n),
        .dphy_data_hs_p(dphy_data_hs_p),
        .dphy_data_hs_n(dphy_data_hs_n),
        .dphy_data_lp_p(dphy_data_lp_p),
        .dphy_data_lp_n(dphy_data_lp_n),
        .byte_clk(phy_byte_clk),
        .idelayctrl_rdy(phy_idelayctrl_rdy),
        .hs_clk_seen(phy_hs_clk_seen),
        .lane_sot_seen(phy_lane_sot_seen),
        .stream_byte_data(phy_stream_byte_data),
        .stream_byte_keep(phy_stream_byte_keep),
        .stream_byte_valid(phy_stream_byte_valid),
        .stream_byte_sop(phy_stream_byte_sop),
        .stream_byte_eop(phy_stream_byte_eop),
        .stream_pairing_active_dbg(phy_stream_pairing_active),
        .stream_pairing_next_dbg(phy_stream_pairing_next),
        .header_valid(phy_header_valid),
        .header_di(phy_header_di),
        .header_wc(phy_header_wc),
        .header_ecc(phy_header_ecc),
        .sync_header_valid(phy_sync_header_valid),
        .sync_header_di(phy_sync_header_di),
        .sync_header_wc(phy_sync_header_wc),
        .sync_header_ecc(phy_sync_header_ecc),
        .sync_header_pairing(phy_sync_header_pairing),
        .sync_header_bit_offset_lane0(phy_sync_header_bit_offset_lane0),
        .sync_header_bit_offset_lane1(phy_sync_header_bit_offset_lane1),
        .sync_header_score(phy_sync_header_score),
        .sync_header_syndrome(phy_sync_header_syndrome),
        .sync_header_ecc_no_error(phy_sync_header_ecc_no_error),
        .sync_header_ecc_corrected(phy_sync_header_ecc_corrected),
        .sync_header_ecc_uncorrectable(phy_sync_header_ecc_uncorrectable),
        .trace_slot_lane0_raw(phy_trace_slot_lane0_raw),
        .trace_slot_lane1_raw(phy_trace_slot_lane1_raw),
        .trace_slot_lane0_candidate(phy_trace_slot_lane0_candidate),
        .trace_slot_lane1_candidate(phy_trace_slot_lane1_candidate),
        .trace_slot_lane0_aligned(phy_trace_slot_lane0_aligned),
        .trace_slot_lane1_aligned(phy_trace_slot_lane1_aligned),
        .trace_slot_sot_hit_lane0(phy_trace_slot_sot_hit_lane0),
        .trace_slot_sot_hit_lane1(phy_trace_slot_sot_hit_lane1),
        .live_trace_seq(phy_live_trace_seq),
        .live_trace_slot_valid(phy_live_trace_slot_valid),
        .live_trace_slot_lane0_raw(phy_live_trace_slot_lane0_raw),
        .live_trace_slot_lane1_raw(phy_live_trace_slot_lane1_raw),
        .live_trace_slot_lane0_candidate(phy_live_trace_slot_lane0_candidate),
        .live_trace_slot_lane1_candidate(phy_live_trace_slot_lane1_candidate),
        .live_trace_slot_lane0_aligned(phy_live_trace_slot_lane0_aligned),
        .live_trace_slot_lane1_aligned(phy_live_trace_slot_lane1_aligned),
        .live_trace_slot_sot_hit_lane0(phy_live_trace_slot_sot_hit_lane0),
        .live_trace_slot_sot_hit_lane1(phy_live_trace_slot_sot_hit_lane1),
        .live_trace_slot_lane0_rotation(phy_live_trace_slot_lane0_rotation),
        .live_trace_slot_lane1_rotation(phy_live_trace_slot_lane1_rotation),
        .lane1_target_phase_out(phy_lane1_target_bitslip_phase),
        .serdes_byte_sample_out(phy_serdes_byte_sample),
        .cfg_settle_blank_k(cfg_settle_blank_k_byte),
        .dbg_burst_count(phy_burst_count),
        .dbg_sot_burst_count(phy_sot_burst_count),
        .dbg_missed_burst(phy_missed_burst),
        .dbg_relock_latency(phy_relock_latency),
        .dbg_relock_max(phy_relock_max)
    );

    // === HW deterministic-lock FSM wiring (E2, 2026-06-19) ===================
    // byte_clk windowed sync-header detector: hdr_ok_byte = (#sync_header_valid
    // in the last HWLOCK_WINDOW byte_clk >= HWLOCK_HDR_MIN). A clean 1-bit
    // lock-quality updated once per window (a right bitslip -> headers stream at
    // line rate -> hdr_ok; a wrong one -> none); 2FF-synced up to refclk_200 for
    // the FSM. WINDOW ~16k byte_clk ~= a few lines; the FSM waits a few windows/combo.
    localparam int HWLOCK_WINDOW  = 16384;
    localparam int HWLOCK_HDR_MIN = 4;
    always_ff @(posedge phy_byte_clk or negedge rst_n) begin
        if (!rst_n) begin
            hwlock_win_cnt <= 14'd0;
            hwlock_hdr_cnt <= 8'd0;
            hdr_ok_byte    <= 1'b0;
        end else if (hwlock_win_cnt == HWLOCK_WINDOW-1) begin
            hdr_ok_byte    <= (hwlock_hdr_cnt >= HWLOCK_HDR_MIN[7:0]);
            hwlock_hdr_cnt <= 8'd0;
            hwlock_win_cnt <= 14'd0;
        end else begin
            hwlock_win_cnt <= hwlock_win_cnt + 14'd1;
            if (phy_sync_header_valid && (hwlock_hdr_cnt != 8'hFF))
                hwlock_hdr_cnt <= hwlock_hdr_cnt + 8'd1;
        end
    end

    // cfg_hw_lock + hdr_ok_byte -> refclk_200 (the FSM domain). Matches the
    // clk_settle CDC style (static config + 1-bit, no reset needed; FFs init 0).
    always_ff @(posedge refclk_200) begin
        cfg_hw_lock_meta <= cfg_hw_lock_sys;
        cfg_hw_lock_ctl  <= cfg_hw_lock_meta;
        hdr_active_meta  <= hdr_ok_byte;
        hdr_active_ctl   <= hdr_active_meta;
    end

    dphy_hwlock_fsm u_dphy_hwlock_fsm (
        .clk        (refclk_200),
        .rst_n      (rst_n),
        .enable     (cfg_hw_lock_ctl),
        .hdr_active (hdr_active_ctl),
        .bitslip_p0 (hwlock_p0),
        .bitslip_p1 (hwlock_p1),
        .bufr_clr   (hwlock_bufr_clr_w),
        .locked     (hwlock_locked),
        .failed     (hwlock_failed),
        .dbg_state  (hwlock_dbg_state),
        .dbg_reroll (hwlock_dbg_reroll),
        .dbg_combo  (hwlock_dbg_combo)
    );

    // === D-PHY lane supervisor (Digilent MIPI_DPHY_Receiver mechanisms) =====
    // Runs unconditionally on the free-running refclk_200; the probe only
    // *uses* its bufr_clr/serdes_rst/hs_settled outputs when sup_enable_byte=1
    // (frame_lines_runtime_word[29]). This is the missing clock-lane management
    // that caused the ~3% capture rate (diary 2026-06-12, memory
    // project_frontend_3pct_capture_root_cause). bufr_clr is registered on
    // refclk_200 inside the supervisor, so feeding it back to the BUFR CLR is
    // not a combinational loop.
    always_ff @(posedge phy_byte_clk or negedge rst_n) begin
        if (!rst_n) begin
            sup_enable_meta <= 1'b0;
            sup_enable_byte <= 1'b0;
            sup_settle_gate_meta <= 1'b0;
            sup_settle_gate_byte <= 1'b0;
            cfg_settle_blank_k_meta <= 4'd0;
            cfg_settle_blank_k_byte <= 4'd0;
        end else begin
            sup_enable_meta <= frame_lines_runtime_word_sys[29];
            sup_enable_byte <= sup_enable_meta;
            sup_settle_gate_meta <= frame_lines_runtime_word_sys[28];
            sup_settle_gate_byte <= sup_settle_gate_meta;
            cfg_settle_blank_k_meta <= idelay_runtime_word_sys[30:27];
            cfg_settle_blank_k_byte <= cfg_settle_blank_k_meta;
        end
    end

    // Runtime clock-lane settle count (bitslip_word[23:17], level) synced into the
    // supervisor's refclk_200 domain. Multi-bit CDC of a static config value (set
    // once + held during a measurement); 2FF is fine (not bit-coherent transient).
    (* ASYNC_REG = "TRUE" *) logic [6:0] clk_settle_meta, clk_settle_ctl;
    always_ff @(posedge refclk_200) begin
        clk_settle_meta <= bitslip_runtime_word_sys[23:17];
        clk_settle_ctl  <= clk_settle_meta;
    end

    dphy_lane_supervisor #(
        .CTL_CLK_HZ(200_000_000)
    ) u_dphy_lane_supervisor (
        .ctl_clk           (refclk_200),
        .ctl_aresetn       (rst_n && ref_pll_locked),
        .clk_lp            ({dphy_clk_lp_p, dphy_clk_lp_n}),
        .data_lp           ({dphy_data_lp_p[0], dphy_data_lp_n[0]}),
        .byte_clk          (phy_byte_clk),
        .cfg_clk_settle_cyc({1'b0, clk_settle_ctl}),
        .bufr_clr          (sup_bufr_clr),
        .rx_clk_active_byte(sup_rx_clk_active),
        .serdes_rst_byte   (sup_serdes_rst),
        .hs_settled_byte   (sup_hs_settled),
        .sts_clk_state     (sup_clk_state),
        .sts_data_state    (sup_data_state),
        .sts_lock_cnt      (sup_lock_cnt),
        .sts_settle_cnt    (sup_settle_cnt),
        .sts_lost_cnt      (sup_lost_cnt)
    );

    // Supervisor status bundle: 2FF sync from refclk_200 into sysclk for the
    // debug-page readback. Metastability-safe; not bit-coherent (diagnostic).
    // Bundle = {settle_cnt[8], lock_cnt[8], data_state[3], clk_state[3],
    //           sup_enable[1], bufr_clr[1], rx_clk_active[1], hs_settled[1]} = 26b
    (* ASYNC_REG = "TRUE" *) logic [25:0] sup_status_sys_meta, sup_status_sys;
    always_ff @(posedge sysclk) begin
        sup_status_sys_meta <= {sup_settle_cnt, sup_lock_cnt, sup_data_state,
                                sup_clk_state, sup_enable_byte, sup_bufr_clr,
                                sup_rx_clk_active, sup_hs_settled};
        sup_status_sys      <= sup_status_sys_meta;
    end

    // Supervisor diagnostic bundle on a page the VDMA-loop readout mux does NOT
    // shadow (it overrides only 0x00/0x06). All fields are refclk_200 (ctl_clk)
    // sourced -> readable even when phy_byte_clk is gated/dead (the continuous-
    // clock case where 0x06's byte-domain status is meaningless, diary 06-14).
    // Layout: [31:24]=lost_cnt [23:16]=settle_cnt [15:8]=lock_cnt
    //         [7:5]=data_state [4:2]=clk_state [1]=bufr_clr [0]=rx_clk_active
    (* ASYNC_REG = "TRUE" *) logic [31:0] sup_dbg_sys_meta, sup_dbg_sys;
    always_ff @(posedge sysclk) begin
        sup_dbg_sys_meta <= {sup_lost_cnt, sup_settle_cnt, sup_lock_cnt,
                             sup_data_state, sup_clk_state, sup_bufr_clr,
                             sup_rx_clk_active};
        sup_dbg_sys      <= sup_dbg_sys_meta;
    end

    // SoT-miss diagnostics CDC (phy_byte_clk -> sysclk, 2FF; diagnostic, not
    // bit-coherent). burst/sot-burst counts (page 0x2b) + last no-SoT burst-head
    // bytes (page 0x2c). 2026-06-17.
    (* ASYNC_REG = "TRUE" *) logic [31:0] sot_miss_sys_meta, sot_miss_sys;
    (* ASYNC_REG = "TRUE" *) logic [31:0] missed_burst_sys_meta, missed_burst_sys;
    (* ASYNC_REG = "TRUE" *) logic [31:0] relock_sys_meta, relock_sys;
    // HW lock FSM status -> sysclk (page 0x2e). Packing:
    //   [31]=failed [30]=locked [29:27]=state [26:23]=reroll [22:17]=combo
    //   [16:14]=bitslip_p0 [13:11]=bitslip_p1 [10]=hdr_active [9:0]=0
    (* ASYNC_REG = "TRUE" *) logic [31:0] hwlock_sys_meta, hwlock_sys;
    always_ff @(posedge sysclk) begin
        sot_miss_sys_meta    <= {phy_sot_burst_count, phy_burst_count};
        sot_miss_sys         <= sot_miss_sys_meta;
        missed_burst_sys_meta<= phy_missed_burst;
        missed_burst_sys     <= missed_burst_sys_meta;
        relock_sys_meta      <= {phy_relock_max, phy_relock_latency};
        relock_sys           <= relock_sys_meta;
        hwlock_sys_meta      <= {hwlock_failed, hwlock_locked, hwlock_dbg_state,
                                 hwlock_dbg_reroll, hwlock_dbg_combo,
                                 hwlock_p0, hwlock_p1, hdr_active_ctl, 10'd0};
        hwlock_sys           <= hwlock_sys_meta;
    end

    // === Raw byte ring buffer (32-bit BRAM, byte_clk capture, sysclk read) ===
    // GPIO word layout:
    //   [8:0]  rd_addr index (0..511)
    //   [9]    rd hi/lo: 0=low 16 bits, 1=high 16 bits
    //   [24]   arm_trigger pulse
    //   [25]   trigger_mode (0=free-run on arm, 1=wait for sync after arm)
`ifdef MIPI_VDMA_LOOP_PORTS
    logic [1:0][7:0] phy_serdes_byte_sample;
    logic [15:0]     rawcap_rd_data;
    logic [9:0]      rawcap_wp_sync;
    logic            rawcap_full_sync;
    logic            rawcap_armed_sync;
    logic            rawcap_waiting_sync;
    // Sysclk → byte_clk CDC for arm_trigger and trigger_mode
    (* ASYNC_REG = "TRUE" *) logic rawcap_arm_meta, rawcap_arm_byte;
    (* ASYNC_REG = "TRUE" *) logic rawcap_tmode_meta, rawcap_tmode_byte;
    always_ff @(posedge phy_byte_clk or negedge rst_n) begin
        if (!rst_n) begin
            rawcap_arm_meta   <= 1'b0;
            rawcap_arm_byte   <= 1'b0;
            rawcap_tmode_meta <= 1'b0;
            rawcap_tmode_byte <= 1'b0;
        end else begin
            rawcap_arm_meta   <= rawcap_word_in[24];
            rawcap_arm_byte   <= rawcap_arm_meta;
            rawcap_tmode_meta <= rawcap_word_in[25];
            rawcap_tmode_byte <= rawcap_tmode_meta;
        end
    end
    // SoT detection for trigger (byte_clk, combinational over latched sample)
    logic rawcap_sot_trigger;
    assign rawcap_sot_trigger = phy_sync_header_valid |
                                (phy_serdes_byte_sample[0] == 8'hB8) |
                                (phy_serdes_byte_sample[1] == 8'hB8);
    // DEPTH 512->64 (2026-06-15): the 512x32 ring buffer fell back to distributed
    // RAM (~928 RAMD64E despite ram_style="block") and was the South Level-5 133%
    // routing-congestion hotspot (78% of the window, report_design_analysis
    // -congestion) that deterministically stalled the router once the re-roll
    // calibration nets shifted placement. It is debug-only (SoT-triggered raw-byte
    // capture, not in the live datapath or the byte-phase cal), so a shorter
    // window relieves the congestion while keeping the rawcap GPIO/BD intact.
    dphy_raw_byte_ringbuf #(.DEPTH(64)) u_rawcap (
        .byte_clk(phy_byte_clk),
        .rst_n_byte(rst_n),
        .lane0_byte_in(phy_serdes_byte_sample[0]),
        .lane1_byte_in(phy_serdes_byte_sample[1]),
        .sync_header_valid_byte(phy_sync_header_valid),
        .sync_trigger_byte(rawcap_sot_trigger),
        .arm_trigger_byte(rawcap_arm_byte),
        .trigger_mode_byte(rawcap_tmode_byte),
        .rd_clk(sysclk),
        .rd_addr(rawcap_word_in[9:0]),
        .rd_data(rawcap_rd_data),
        .last_write_addr_sync(rawcap_wp_sync),
        .full_sync(rawcap_full_sync),
        .armed_sync(rawcap_armed_sync),
        .waiting_sync(rawcap_waiting_sync)
    );
    always_comb begin
        rawcap_status_out = 32'h00000000;
        rawcap_status_out[15:0]  = rawcap_rd_data;
        rawcap_status_out[25:16] = rawcap_wp_sync;
        rawcap_status_out[26]    = rawcap_full_sync;
        rawcap_status_out[27]    = rawcap_armed_sync;
        rawcap_status_out[28]    = rawcap_waiting_sync;
    end
`else
    logic [1:0][7:0] phy_serdes_byte_sample;
`endif

    assign phy_live_pair0_word0 = {phy_live_trace_slot_lane1_aligned[1], phy_live_trace_slot_lane0_aligned[1]};
    assign phy_live_pair0_word1 = {phy_live_trace_slot_lane1_aligned[2], phy_live_trace_slot_lane0_aligned[2]};
    assign phy_live_pair0_di = phy_live_pair0_word0[7:0];
    assign phy_live_pair0_wc = {phy_live_pair0_word1[7:0], phy_live_pair0_word0[15:8]};
    assign phy_live_pair0_ecc = phy_live_pair0_word1[15:8];
    assign phy_stream_sop_word0 = phy_stream_sop0_debug_word[15:0];
    assign phy_stream_sop_word1 = phy_stream_sop1_debug_word[15:0];
    assign phy_stream_sop_wc = {phy_stream_sop_word1[7:0], phy_stream_sop_word0[15:8]};
    assign phy_obs_compare_flags = {
        phy_stream_sop0_debug_word[31],
        phy_stream_sop1_debug_word[31],
        phy_stream_sop_word0 == phy_live_pair0_word0,
        phy_stream_sop_word1 == phy_live_pair0_word1,
        (phy_live_pair0_di[5:0] == 6'h1e) && (phy_live_pair0_wc == 16'd1280),
        phy_stream_pairing_active == 3'd0,
        phy_sync_header_pairing == 3'd0,
        phy_sync_header_valid
    };

    logic protocol_resetn;
    logic [15:0] cdc_byte_data;
    logic [1:0] cdc_byte_keep;
    logic cdc_byte_valid;
    logic cdc_byte_sop;
    logic cdc_byte_eop;

    // use_tpg_rt: runtime switch via frame_lines_gpio bit[26].
    // 0 = camera path (default), 1 = internal TPG.
    // Both paths always exist in the netlist — Vivado cannot sweep either.
    wire use_tpg_rt = frame_lines_runtime_word_sys[26];

    logic [15:0] tpg_byte_data;
    logic [1:0]  tpg_byte_keep;
    logic        tpg_byte_valid;
    logic        tpg_byte_sop;
    logic        tpg_byte_eop;

    // tpg_sop_cnt_core: counts raw TPG output SOPs (before mux). Page P=0x9b → case 6'h3b.
    logic [15:0] tpg_sop_cnt_core;
    always_ff @(posedge sysclk or negedge protocol_resetn) begin
        if (!protocol_resetn) tpg_sop_cnt_core <= 16'h0;
        else if (tpg_byte_valid && tpg_byte_sop) tpg_sop_cnt_core <= tpg_sop_cnt_core + 16'd1;
    end

    // Pipelined AND-OR mux (1-cycle register stage).
    //
    // Combinational AND-OR on 16-bit data: use_tpg_rt (AXI GPIO FF, fixed placement)
    // needs fanout ~38 → routing congestion on Z7-020.  Fix: register gate + all data
    // inputs together so Vivado can place tpg_gate_r/cam_gate_r FFs right next to their
    // consumers; use_tpg_rt fanout drops to 2.  All signals delayed the same 1 cycle →
    // byte-stream alignment preserved.  Unconditional _r assignments also prevent Vivado
    // CE inference on tpg_byte_data FFs inside csi2_tpg.
    logic        tpg_gate_r,  cam_gate_r;
    logic        tpg_valid_r, cdc_valid_r;
    logic        tpg_sop_r,   cdc_sop_r;
    logic        tpg_eop_r,   cdc_eop_r;
    logic [15:0] tpg_data_r,  cdc_data_r;
    logic [1:0]  tpg_keep_r,  cdc_keep_r;

    always_ff @(posedge sysclk or negedge protocol_resetn) begin
        if (!protocol_resetn) begin
            tpg_gate_r  <= 1'b0; cam_gate_r  <= 1'b1;
            tpg_valid_r <= 1'b0; cdc_valid_r <= 1'b0;
            tpg_sop_r   <= 1'b0; cdc_sop_r   <= 1'b0;
            tpg_eop_r   <= 1'b0; cdc_eop_r   <= 1'b0;
            tpg_data_r  <= 16'h0; cdc_data_r  <= 16'h0;
            tpg_keep_r  <= 2'h0;  cdc_keep_r  <= 2'h0;
        end else begin
            tpg_gate_r  <= use_tpg_rt;
            cam_gate_r  <= ~use_tpg_rt;
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

    // pkt_sop_cnt_core: counts SOPs AFTER the mux (what the parser actually sees).
    // If mux works: matches tpg_sop_cnt_core when use_tpg_rt=1, cdc_byte_sop_cnt when =0.
    logic [15:0] pkt_sop_cnt_core;
    always_ff @(posedge sysclk or negedge protocol_resetn) begin
        if (!protocol_resetn) pkt_sop_cnt_core <= 16'h0;
        else if (pkt_byte_valid && pkt_byte_sop) pkt_sop_cnt_core <= pkt_sop_cnt_core + 16'd1;
    end

    // SOP data latches: capture the first beat at each SOP event.
    // Comparing these three tells us definitively where wrong bytes are introduced:
    //   tpg_sop_data_latch: what tpg_data_r contains at TPG SOP  → should be 0x0022 for long pkt (DI=0x22, WC[7:0]=0x00)
    //   pkt_sop_data_latch: what pkt_byte_data contains at mux SOP → must equal tpg value when use_tpg_rt=1
    //   cdc_sop_data_latch: what cdc_data_r contains at camera SOP → reference camera DI (expect 0x1Exx)
    logic [15:0] pkt_sop_data_latch;
    logic [15:0] tpg_sop_data_latch;
    logic [15:0] cdc_sop_data_latch;
    always_ff @(posedge sysclk or negedge protocol_resetn) begin
        if (!protocol_resetn) begin
            pkt_sop_data_latch <= 16'hDEAD;
            tpg_sop_data_latch <= 16'hDEAD;
            cdc_sop_data_latch <= 16'hDEAD;
        end else begin
            if (pkt_byte_valid && pkt_byte_sop) pkt_sop_data_latch <= pkt_byte_data;
            if (tpg_valid_r    && tpg_sop_r)    tpg_sop_data_latch <= tpg_data_r;
            if (cdc_valid_r    && cdc_sop_r)    cdc_sop_data_latch <= cdc_data_r;
        end
    end
    logic protocol_resetn_phy_meta;
    logic protocol_resetn_phy;
    logic [15:0] sts_byte_cdc_ovf_cnt;
    logic [31:0] cdc_stream_sop0_debug_word;
    logic [31:0] cdc_stream_sop1_debug_word;
    logic cdc_stream_sop_second_pending;

    assign protocol_resetn = rst_n && sccb_done;

    always_ff @(posedge phy_byte_clk or negedge rst_n) begin
        if (!rst_n) begin
            protocol_resetn_phy_meta <= 1'b0;
            protocol_resetn_phy <= 1'b0;
        end else begin
            protocol_resetn_phy_meta <= protocol_resetn;
            protocol_resetn_phy <= protocol_resetn_phy_meta;
        end
    end

    byte_to_core_cdc #(
        .IN_WIDTH(16),
        .KEEP_WIDTH(2),
        .FIFO_DEPTH(1024),
        .CORE_OUTPUT_INTERVAL(2)
    ) u_probe_stream_cdc (
        .byte_clk(phy_byte_clk),
        .byte_aresetn(protocol_resetn_phy),
        .core_clk(sysclk),
        .core_aresetn(protocol_resetn),
        .s_byte_data(phy_stream_byte_data),
        .s_byte_keep(phy_stream_byte_keep),
        .s_byte_valid(phy_stream_byte_valid),
        .s_byte_sop(phy_stream_byte_sop),
        .s_byte_eop(phy_stream_byte_eop),
        .m_byte_data(cdc_byte_data),
        .m_byte_keep(cdc_byte_keep),
        .m_byte_valid(cdc_byte_valid),
        .m_byte_sop(cdc_byte_sop),
        .m_byte_eop(cdc_byte_eop),
        .sts_lane_fifo_ovf_cnt(sts_byte_cdc_ovf_cnt)
    );

    // -------------------------------------------------------------------------
    // CSI-2 TPG — always instantiated so both paths exist in the netlist.
    // Runtime switch: use_tpg_rt (frame_lines_gpio bit[26]) selects TPG vs camera.
    // Pattern select: frame_lines_gpio bits[28:27] → pattern_sel[1:0]
    //   00 = vertical ramp  01 = horizontal ramp
    //   10 = checkerboard   11 = diagonal ramp
    wire [1:0] tpg_pattern_sel = frame_lines_runtime_word_sys[28:27];
    csi2_tpg #(
        .H_PIXELS        (640),
        .V_LINES         (480),
        .DT              (FORMAT_EXPECTED_DT),
        .VC              (2'h0),
        .LSLE_EN         (1'b0),
        .FRAME_GAP_CLOCKS(1_000_000),
        .OUTPUT_INTERVAL (2)    // match CDC CORE_OUTPUT_INTERVAL=2; prevents parser FIFO overflow
    ) u_csi2_tpg (
        .clk         (sysclk),
        .rst_n       (protocol_resetn),
        .pattern_sel (tpg_pattern_sel),
        .m_byte_data (tpg_byte_data),
        .m_byte_keep (tpg_byte_keep),
        .m_byte_valid(tpg_byte_valid),
        .m_byte_sop  (tpg_byte_sop),
        .m_byte_eop  (tpg_byte_eop)
    );

    always_ff @(posedge sysclk) begin
        if (!protocol_resetn) begin
            cdc_stream_sop0_debug_word <= 32'h00000000;
            cdc_stream_sop1_debug_word <= 32'h00000000;
            cdc_stream_sop_second_pending <= 1'b0;
        end else if (cdc_byte_valid && cdc_byte_sop) begin
            cdc_stream_sop0_debug_word <= {1'b1, 15'h0000, cdc_byte_data};
            cdc_stream_sop1_debug_word <= 32'h00000000;
            cdc_stream_sop_second_pending <= 1'b1;
        end else if (cdc_byte_valid && cdc_stream_sop_second_pending) begin
            cdc_stream_sop1_debug_word <= {1'b1, 15'h0000, cdc_byte_data};
            cdc_stream_sop_second_pending <= 1'b0;
        end
    end

    logic parser_ecc_hdr_valid;
    logic [31:0] parser_ecc_hdr_raw;
    logic ecc_hdr_corr_valid;
    logic [23:0] ecc_hdr_corr;
    logic [7:0] ecc_hdr_di;
    logic [15:0] ecc_hdr_wc;
    logic ecc_hdr_corrected;
    logic ecc_hdr_uncorrectable;
    logic ecc_hdr_no_error;
    logic parser_pkt_hdr_valid;
    logic [31:0] parser_pkt_hdr_raw;
    logic [7:0] parser_pkt_di;
    logic [15:0] parser_pkt_wc;
    logic parser_pkt_is_long;
    logic parser_pkt_is_short;
    logic parser_pkt_ecc_uncorrectable;
    logic [7:0] parser_payload_data;
    logic parser_payload_valid;
    logic parser_payload_first;
    logic parser_payload_last;
    logic [15:0] parser_footer_data;
    logic parser_footer_valid;
    logic parser_pkt_done;
    logic crc_check_valid;
    logic crc_match;
    logic [15:0] crc_calc;
    logic [15:0] crc_received;
    logic filter_pkt_done;
    logic [7:0] filter_pkt_di;
    logic [15:0] filter_pkt_wc;
    logic filter_pkt_is_short;
    logic filter_pkt_is_long;
    logic filter_pkt_start;
    logic filter_pkt_end;
    logic filter_pkt_err;
    logic [7:0] filter_payload_data;
    logic filter_payload_valid;
    logic filter_payload_first;
    logic filter_payload_last;
    logic frame_sof;
    logic frame_eof;
    logic frame_sol;
    logic frame_eol;
    logic frame_in_frame;          // frame_state window open (boundary trace tag)
    logic [15:0] frame_line_idx;
    logic [7:0] frame_payload_data;
    logic frame_payload_valid;
    logic frame_payload_first;
    logic frame_payload_last;
    logic frame_err;
    logic [15:0] sts_short_pkt_cnt;
    logic [15:0] sts_long_pkt_cnt;
    logic [15:0] sts_pkt_trunc_cnt;
    logic [15:0] sts_ecc_corr_cnt;
    logic [15:0] sts_ecc_uncorr_cnt;
    logic [15:0] sts_crc_err_cnt;
    logic [15:0] sts_crc_ok_cnt;
    logic [15:0] sts_drop_vc_cnt;
    logic [15:0] sts_drop_dt_cnt;
    logic [31:0] sts_frame_count_core;
    logic [31:0] sts_line_count_core;
    logic [15:0] sts_last_frame_lines;
    logic [15:0] sts_frame_sync_err_cnt;
    logic [15:0] sts_dbg_long_accept, sts_dbg_long_nols, sts_dbg_long_idle;
    logic [127:0] sts_dbg_nols_hist;   // no-LS reject position histogram (8x16)

    assign filter_pkt_done = (parser_pkt_is_short && parser_pkt_done) || crc_check_valid;

    csi2_packet_parser #(.IN_WIDTH(16), .FIFO_DEPTH(16)) u_packet_parser (
        .core_clk(sysclk), .core_aresetn(protocol_resetn),
        .s_byte_data(pkt_byte_data), .s_byte_keep(pkt_byte_keep), .s_byte_valid(pkt_byte_valid), .s_byte_sop(pkt_byte_sop), .s_byte_eop(pkt_byte_eop),
        .ecc_hdr_valid(parser_ecc_hdr_valid), .ecc_hdr_raw(parser_ecc_hdr_raw), .ecc_hdr_corr_valid(ecc_hdr_corr_valid), .ecc_hdr_di(ecc_hdr_di), .ecc_hdr_wc(ecc_hdr_wc), .ecc_hdr_uncorrectable(ecc_hdr_uncorrectable),
        .m_pkt_hdr_valid(parser_pkt_hdr_valid), .m_pkt_hdr_raw(parser_pkt_hdr_raw), .m_pkt_di(parser_pkt_di), .m_pkt_wc(parser_pkt_wc), .m_pkt_is_long(parser_pkt_is_long), .m_pkt_is_short(parser_pkt_is_short), .m_pkt_ecc_uncorrectable(parser_pkt_ecc_uncorrectable),
        .m_payload_data(parser_payload_data), .m_payload_valid(parser_payload_valid), .m_payload_first(parser_payload_first), .m_payload_last(parser_payload_last), .m_footer_data(parser_footer_data), .m_footer_valid(parser_footer_valid), .m_pkt_done(parser_pkt_done),
        .sts_short_pkt_cnt(sts_short_pkt_cnt), .sts_long_pkt_cnt(sts_long_pkt_cnt), .sts_pkt_trunc_cnt(sts_pkt_trunc_cnt)
    );

    csi2_header_ecc u_header_ecc (
        .core_clk(sysclk), .core_aresetn(protocol_resetn), .hdr_valid(parser_ecc_hdr_valid), .hdr_raw(parser_ecc_hdr_raw), .hdr_corr_valid(ecc_hdr_corr_valid), .hdr_corr(ecc_hdr_corr), .hdr_di(ecc_hdr_di), .hdr_wc(ecc_hdr_wc),
        .hdr_ecc_corrected(ecc_hdr_corrected), .hdr_ecc_uncorrectable(ecc_hdr_uncorrectable), .hdr_ecc_no_error(ecc_hdr_no_error), .sts_ecc_corr_cnt(sts_ecc_corr_cnt), .sts_ecc_uncorr_cnt(sts_ecc_uncorr_cnt)
    );

    csi2_payload_crc u_payload_crc (
        .core_clk(sysclk), .core_aresetn(protocol_resetn), .payload_data(parser_payload_data), .payload_valid(parser_payload_valid), .payload_first(parser_payload_first), .payload_last(parser_payload_last),
        .footer_data(parser_footer_data), .footer_valid(parser_footer_valid), .crc_check_valid(crc_check_valid), .crc_match(crc_match), .crc_calc(crc_calc), .crc_received(crc_received), .sts_crc_err_cnt(sts_crc_err_cnt), .sts_crc_ok_cnt(sts_crc_ok_cnt)
    );

    localparam logic [5:0] FORMAT_EXPECTED_DT =
        (IMAGE_FORMAT == 2) ? 6'h2a :  // RAW8
        (IMAGE_FORMAT == 3) ? 6'h2b :  // RAW10
        (IMAGE_FORMAT == 1 || USE_RGB565_GRAY) ? 6'h22 :  // RGB565
        6'h1e;                         // YUV422 (default)

    csi2_vcdt_filter u_vcdt_filter (
        .core_clk(sysclk), .core_aresetn(protocol_resetn), .cfg_expected_vc(2'd0), .cfg_expected_dt(FORMAT_EXPECTED_DT), .cfg_pass_short(1'b1), .cfg_pass_emb_data(1'b0),
        .pkt_hdr_valid(parser_pkt_hdr_valid), .pkt_di(parser_pkt_di), .pkt_wc(parser_pkt_wc), .pkt_is_long(parser_pkt_is_long), .pkt_is_short(parser_pkt_is_short), .pkt_done(filter_pkt_done),
        .ecc_corrected(ecc_hdr_corrected), .ecc_uncorrectable(parser_pkt_ecc_uncorrectable), .crc_check_valid(crc_check_valid), .crc_match(crc_match),
        .payload_data(parser_payload_data), .payload_valid(parser_payload_valid), .payload_first(parser_payload_first), .payload_last(parser_payload_last),
        .out_pkt_di(filter_pkt_di), .out_pkt_wc(filter_pkt_wc), .out_pkt_is_short(filter_pkt_is_short), .out_pkt_is_long(filter_pkt_is_long), .out_pkt_start(filter_pkt_start), .out_pkt_end(filter_pkt_end), .out_pkt_err(filter_pkt_err),
        .out_payload_data(filter_payload_data), .out_payload_valid(filter_payload_valid), .out_payload_first(filter_payload_first), .out_payload_last(filter_payload_last), .sts_drop_vc_cnt(sts_drop_vc_cnt), .sts_drop_dt_cnt(sts_drop_dt_cnt)
    );

    // 2026-06-02: DATA-DRIVEN frame assembly (follow the transmitter, do not
    // impose a fixed format). GUARD_FRAME_LINES=0 disables the receiver-side
    // 480-line forced-EOF (guard_expected_last_line), the WC!=1280 reject
    // (guard_line_wc_ok), the >=480 long-packet reject, the MAX_LINES cap, and
    // the lsle_line_guard FS-anchor. The FSM now purely transcribes the chip's
    // FS->SOF / FE->EOF / (LS,long,LE)->line markers. EXPECTED_* kept non-zero
    // for status only; with GUARD=0 they gate nothing.
    // 2026-06-03: FS plausibility-window frame delimiter. The open-loop byte
    // aligner emits SPURIOUS FS (DI=0x00) on payload-zero runs, chopping frames
    // into short pieces that stack in the 480-line VDMA buffer (the "banding"
    // the cover-test exposed). FS_MIN_LINES=300 ignores any FS arriving <300
    // lines into the frame (spurious); a plausible FS in [300, MAX_LINES] is the
    // real frame boundary; a missing FS is bounded by the MAX_LINES=640 cap.
    csi2_frame_state #(
        .MAX_LINES(640),
        .GUARD_FRAME_LINES(1'b1),
        .EXPECTED_FRAME_LINES(480),
        .EXPECTED_LINE_WC(16'd1280),
        .FS_MIN_LINES(300),
        // FE-DELIMITER mode (2026-06-04): close the frame on the chip's FE (FS only
        // re-anchors the top). On a stable AEC the chip emits balanced FS==FE
        // (39==39 measured), so FE marks the true ~480-line bottom and phase-locks
        // the VDMA buffer, vs. the FS-anchor path where most frames hit the 640 cap
        // and stack into ~6 rolling bands. Spurious early FE (<300) ignored; dropped
        // FE bounded by the 640 cap.
        .FE_DELIMITS(1'b1),
        // FE_MIN_LINES (2026-06-15): the bottom band's root cause. On a locked link
        // ~480 long packets reach the parser, but the ONLY FE per frame is a
        // SPURIOUS early FE that closes the frame at line_idx ~441-450
        // (fe_after_480 ~= 0); the lost tail ~28 lines (long_before_fs) become the
        // band. With the FE close floor at 460 (between the spurious FE <=450 and the
        // real next-frame FS at line_idx ~469-478) the spurious FE is rejected and the
        // frame closes on the real FS (lost-FE recovery), capturing the full ~480.
        // Validated in tb_csi2_frame_state_feearly; tune on hardware if frames merge
        // (lower) or the band persists (raise). diary 20260615.
        .FE_MIN_LINES(460)
    ) u_frame_state (
        .core_clk(sysclk), .core_aresetn(protocol_resetn), .cfg_use_lsle(cfg_use_lsle_sys),
        .cfg_expected_frame_lines(frame_lines_runtime_value_sys),
        .cfg_sof_synth(cfg_sof_synth_sys),
        .cfg_force_expected(cfg_force_expected_sys),
        .cfg_long_as_line(cfg_long_as_line_sys),
        .in_pkt_di(filter_pkt_di), .in_pkt_wc(filter_pkt_wc), .in_pkt_is_short(filter_pkt_is_short), .in_pkt_is_long(filter_pkt_is_long), .in_pkt_start(filter_pkt_start), .in_pkt_end(filter_pkt_end), .in_pkt_err(filter_pkt_err),
        .in_payload_data(filter_payload_data), .in_payload_valid(filter_payload_valid), .in_payload_first(filter_payload_first), .in_payload_last(filter_payload_last),
        .out_sof(frame_sof), .out_eof(frame_eof), .out_sol(frame_sol), .out_eol(frame_eol), .out_in_frame(frame_in_frame), .out_line_idx(frame_line_idx), .out_payload_data(frame_payload_data), .out_payload_valid(frame_payload_valid), .out_payload_first(frame_payload_first), .out_payload_last(frame_payload_last), .out_frame_err(frame_err),
        .sts_frame_count(sts_frame_count_core), .sts_line_count(sts_line_count_core), .sts_last_frame_lines(sts_last_frame_lines), .sts_frame_sync_err_cnt(sts_frame_sync_err_cnt),
        .sts_dbg_long_accept(sts_dbg_long_accept), .sts_dbg_long_nols(sts_dbg_long_nols), .sts_dbg_long_idle(sts_dbg_long_idle),
        .sts_dbg_nols_hist(sts_dbg_nols_hist)
    );

    // ==================================================================
    // Boundary packet trace (2026-06-16, diagnostic only). On each packet header
    // into frame_state, record {in_frame, is_long, DT[5:0]} into a 32-deep ring;
    // ~24 packets after each FE, snapshot the ring so software reads a STABLE view
    // of the FE -> next-FS boundary. This shows the packet ORDER (which aggregate
    // counters cannot), distinguishing the ~28 dropped longs: this-frame TAIL
    // (spurious/early FE, chip keeps sending) vs next-frame LEAD (real FS late) vs
    // in-frame reject. Read via page 0x3F; index = idelay GPIO [20:16], freeze =
    // idelay GPIO [25] -- both read DIRECT (no apply) so they reuse the now-dead
    // clk-IDELAY field without a new GPIO or any functional change.
    logic [7:0] btrace_buf  [0:31];
    logic [7:0] btrace_snap [0:31];
    logic [4:0] btrace_wptr;
    logic [4:0] btrace_snap_wptr;
    logic [4:0] btrace_post;
    logic       btrace_arm;
    logic       btrace_have;
    wire        btrace_freeze = idelay_runtime_word_sys[25];
    wire [4:0]  btrace_idx    = idelay_runtime_word_sys[20:16];
    wire        btrace_is_fe  = filter_pkt_is_short && (filter_pkt_di[5:0] == 6'h01);
    wire [7:0]  btrace_entry  = {frame_in_frame, filter_pkt_is_long, filter_pkt_di[5:0]};
    always_ff @(posedge sysclk) begin
        if (!protocol_resetn) begin
            btrace_wptr      <= 5'd0;
            btrace_snap_wptr <= 5'd0;
            btrace_post      <= 5'd0;
            btrace_arm       <= 1'b0;
            btrace_have      <= 1'b0;
        end else if (filter_pkt_start) begin
            btrace_buf[btrace_wptr] <= btrace_entry;
            btrace_wptr <= btrace_wptr + 5'd1;
            if (btrace_is_fe) begin
                btrace_arm  <= 1'b1;
                btrace_post <= 5'd0;
            end else if (btrace_arm) begin
                if (btrace_post == 5'd23) begin
                    btrace_arm <= 1'b0;
                    if (!btrace_freeze) begin
                        for (int i = 0; i < 32; i++) btrace_snap[i] <= btrace_buf[i];
                        btrace_snap_wptr <= btrace_wptr + 5'd1;  // oldest entry slot
                        btrace_have      <= 1'b1;
                    end
                end else begin
                    btrace_post <= btrace_post + 5'd1;
                end
            end
        end
    end
    wire [7:0] btrace_rd = btrace_snap[btrace_idx];

    logic [23:0] yuv_pixel;
    logic yuv_pixel_valid;
    logic yuv_pixel_sof;
    logic yuv_pixel_eol;
    logic yuv_pixel_eof;
    logic yuv_pixel_err;
    logic [15:0] yuv_pixel_per_line;

    logic [23:0] rgb565_pixel;
    logic rgb565_pixel_valid;
    logic rgb565_pixel_sof;
    logic rgb565_pixel_eol;
    logic rgb565_pixel_eof;
    logic rgb565_pixel_err;
    logic [15:0] rgb565_pixel_per_line;

    logic [7:0]  raw8_pix_data;
    logic        raw8_pix_valid, raw8_pix_sof, raw8_pix_eol, raw8_pix_eof, raw8_pix_err;
    logic [15:0] raw8_pix_per_line;

    logic [9:0]  raw10_pix_data;
    logic        raw10_pix_valid, raw10_pix_sof, raw10_pix_eol, raw10_pix_eof, raw10_pix_err;
    logic [15:0] raw10_pix_per_line;

    // Promote RAW outputs to 24-bit replicated grayscale (Y8 → {Y,Y,Y});
    // RAW10 truncated to upper 8 bits for downstream Y8 path.
    wire [23:0] raw8_pixel  = {raw8_pix_data,           raw8_pix_data,           raw8_pix_data};
    wire [23:0] raw10_pixel = {raw10_pix_data[9:2],     raw10_pix_data[9:2],     raw10_pix_data[9:2]};

    wire [23:0] video_pixel =
        (IMAGE_FORMAT == 3) ? raw10_pixel :
        (IMAGE_FORMAT == 2) ? raw8_pixel  :
        (IMAGE_FORMAT == 1 || USE_RGB565_GRAY) ? rgb565_pixel :
        yuv_pixel;
    wire video_pixel_valid =
        (IMAGE_FORMAT == 3) ? raw10_pix_valid :
        (IMAGE_FORMAT == 2) ? raw8_pix_valid  :
        (IMAGE_FORMAT == 1 || USE_RGB565_GRAY) ? rgb565_pixel_valid :
        yuv_pixel_valid;
    wire video_pixel_sof =
        (IMAGE_FORMAT == 3) ? raw10_pix_sof :
        (IMAGE_FORMAT == 2) ? raw8_pix_sof  :
        (IMAGE_FORMAT == 1 || USE_RGB565_GRAY) ? rgb565_pixel_sof :
        yuv_pixel_sof;
    wire video_pixel_eol =
        (IMAGE_FORMAT == 3) ? raw10_pix_eol :
        (IMAGE_FORMAT == 2) ? raw8_pix_eol  :
        (IMAGE_FORMAT == 1 || USE_RGB565_GRAY) ? rgb565_pixel_eol :
        yuv_pixel_eol;
    wire video_pixel_eof =
        (IMAGE_FORMAT == 3) ? raw10_pix_eof :
        (IMAGE_FORMAT == 2) ? raw8_pix_eof  :
        (IMAGE_FORMAT == 1 || USE_RGB565_GRAY) ? rgb565_pixel_eof :
        yuv_pixel_eof;
    wire video_pixel_err =
        (IMAGE_FORMAT == 3) ? raw10_pix_err :
        (IMAGE_FORMAT == 2) ? raw8_pix_err  :
        (IMAGE_FORMAT == 1 || USE_RGB565_GRAY) ? rgb565_pixel_err :
        yuv_pixel_err;
    wire [15:0] video_pixel_per_line =
        (IMAGE_FORMAT == 3) ? raw10_pix_per_line :
        (IMAGE_FORMAT == 2) ? raw8_pix_per_line  :
        (IMAGE_FORMAT == 1 || USE_RGB565_GRAY) ? rgb565_pixel_per_line :
        yuv_pixel_per_line;

    yuv422_gray_unpack #(
        .YUV422_SEQUENCE(OV5640_FORMAT_CTRL_4300[3:0]),
        .LINE_PIXELS(640),
        .LEFT_REPAIR_PIXELS(0)
    ) u_yuv422_gray_unpack (
        .core_clk(sysclk), .core_aresetn(protocol_resetn),
        .in_sof(frame_sof), .in_eof(frame_eof), .in_eol(frame_eol), .in_payload_data(frame_payload_data), .in_payload_valid(frame_payload_valid), .in_payload_first(frame_payload_first), .in_payload_last(frame_payload_last), .in_frame_err(frame_err),
        .out_pixel(yuv_pixel), .out_pixel_valid(yuv_pixel_valid), .out_pixel_sof(yuv_pixel_sof), .out_pixel_eol(yuv_pixel_eol), .out_pixel_eof(yuv_pixel_eof), .out_pixel_err(yuv_pixel_err), .sts_pixel_per_line(yuv_pixel_per_line)
    );

    rgb565_gray_unpack #(
        .RGB565_BIG_ENDIAN(1'b0),
        .RGB_OUT(COLOR_CAPTURE),   // true RGB888 out when capturing color (else luma/gray)
        .LINE_PIXELS(640)
    ) u_rgb565_gray_unpack (
        .core_clk(sysclk), .core_aresetn(protocol_resetn),
        .in_sof(frame_sof), .in_eof(frame_eof), .in_eol(frame_eol), .in_payload_data(frame_payload_data), .in_payload_valid(frame_payload_valid), .in_payload_first(frame_payload_first), .in_payload_last(frame_payload_last), .in_frame_err(frame_err),
        .out_pixel(rgb565_pixel), .out_pixel_valid(rgb565_pixel_valid), .out_pixel_sof(rgb565_pixel_sof), .out_pixel_eol(rgb565_pixel_eol), .out_pixel_eof(rgb565_pixel_eof), .out_pixel_err(rgb565_pixel_err), .sts_pixel_per_line(rgb565_pixel_per_line)
    );

    raw8_passthrough #(.LINE_PIXELS(640)) u_raw8_passthrough (
        .core_clk(sysclk), .core_aresetn(protocol_resetn),
        .in_sof(frame_sof), .in_eof(frame_eof), .in_eol(frame_eol),
        .in_payload_data(frame_payload_data), .in_payload_valid(frame_payload_valid),
        .in_payload_first(frame_payload_first), .in_payload_last(frame_payload_last),
        .in_frame_err(frame_err),
        .out_pixel(raw8_pix_data), .out_pixel_valid(raw8_pix_valid),
        .out_pixel_sof(raw8_pix_sof), .out_pixel_eol(raw8_pix_eol),
        .out_pixel_eof(raw8_pix_eof), .out_pixel_err(raw8_pix_err),
        .sts_pixel_per_line(raw8_pix_per_line)
    );

    raw10_unpack #(.LINE_PIXELS(640)) u_raw10_unpack (
        .core_clk(sysclk), .core_aresetn(protocol_resetn),
        .in_sof(frame_sof), .in_eof(frame_eof), .in_eol(frame_eol),
        .in_payload_data(frame_payload_data), .in_payload_valid(frame_payload_valid),
        .in_payload_first(frame_payload_first), .in_payload_last(frame_payload_last),
        .in_frame_err(frame_err),
        .out_pixel(raw10_pix_data), .out_pixel_valid(raw10_pix_valid),
        .out_pixel_sof(raw10_pix_sof), .out_pixel_eol(raw10_pix_eol),
        .out_pixel_eof(raw10_pix_eof), .out_pixel_err(raw10_pix_err),
        .sts_pixel_per_line(raw10_pix_per_line)
    );

    wire hdmi_clk_feedback;
    wire hdmi_clk_feedback_buf;
    wire tmds_clk_unbuf;
    wire pix_clk_unbuf;
    wire tmds_clk;
    wire pix_clk;
    wire hdmi_mmcm_locked;

    MMCME2_BASE #(
        .BANDWIDTH("OPTIMIZED"), .CLKFBOUT_MULT_F(8.0), .CLKFBOUT_PHASE(0.0), .CLKIN1_PERIOD(8.0),
        .CLKOUT0_DIVIDE_F(8.0), .CLKOUT0_DUTY_CYCLE(0.5), .CLKOUT0_PHASE(0.0),
        .CLKOUT1_DIVIDE(40), .CLKOUT1_DUTY_CYCLE(0.5), .CLKOUT1_PHASE(0.0),
        .DIVCLK_DIVIDE(1), .STARTUP_WAIT("FALSE")
    ) u_hdmi_mmcm (
        .CLKIN1(sysclk), .CLKFBIN(hdmi_clk_feedback_buf), .CLKFBOUT(hdmi_clk_feedback), .CLKFBOUTB(),
        .CLKOUT0(tmds_clk_unbuf), .CLKOUT0B(), .CLKOUT1(pix_clk_unbuf), .CLKOUT1B(), .CLKOUT2(), .CLKOUT2B(), .CLKOUT3(), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6(),
        .LOCKED(hdmi_mmcm_locked), .PWRDWN(1'b0), .RST(1'b0)
    );

    BUFG u_hdmi_clkfb_bufg (.I(hdmi_clk_feedback), .O(hdmi_clk_feedback_buf));
    BUFG u_tmds_clk_bufg (.I(tmds_clk_unbuf), .O(tmds_clk));
    BUFG u_pix_clk_bufg (.I(pix_clk_unbuf), .O(pix_clk));

    logic [7:0] pix_reset_shift = 8'h00;
    logic pix_aresetn;

    always_ff @(posedge pix_clk) begin
        pix_reset_shift <= {pix_reset_shift[6:0], hdmi_mmcm_locked};
        pix_aresetn <= &pix_reset_shift;
    end

    logic [23:0] axis_tdata;
    logic axis_tvalid;
    logic axis_tready;
    logic axis_tlast;
    logic [1:0] axis_tuser2;
    logic [0:0] axis_tuser;
    logic [23:0] direct_axis_tdata;
    logic direct_axis_tvalid;
    logic direct_axis_tready;
    logic direct_axis_tlast;
    logic [1:0] direct_axis_tuser;
    logic [15:0] direct_fifo_overflow_count;
    logic [15:0] direct_back_pressure_count;
    logic [23:0] fb_axis_tdata;
    logic fb_axis_tvalid;
    logic fb_axis_tready;
    logic fb_axis_tlast;
    logic [0:0] fb_axis_tuser;
    logic [15:0] fb_good_line_count;
    logic [15:0] fb_bad_line_count;
    logic [31:0] fb_frame_count;
    logic [15:0] fb_write_line;
    logic fb_frame_ready;

`ifdef MIPI_VDMA_LOOP_PORTS
    assign direct_axis_tdata = 24'h000000;
    assign direct_axis_tvalid = 1'b0;
    assign direct_axis_tlast = 1'b0;
    assign direct_axis_tuser = 2'b00;
    assign direct_fifo_overflow_count = 16'h0000;
    assign direct_back_pressure_count = 16'h0000;
`else
    axis_video_bridge #(
        .TDATA_WIDTH(24),
        .TUSER_WIDTH(2),
        .FIFO_DEPTH(4096),
        .AXIS_TUSER_ERR_DEBUG(1'b1)
    ) u_direct_axis_video_bridge (
        .core_clk(sysclk),
        .core_aresetn(protocol_resetn),
        .aclk(pix_clk),
        .aresetn(pix_aresetn),
        .in_pixel(video_pixel),
        .in_pixel_valid(video_pixel_valid),
        .in_pixel_sof(video_pixel_sof),
        .in_pixel_eol(video_pixel_eol),
        .in_pixel_eof(video_pixel_eof),
        .in_pixel_err(video_pixel_err),
        .m_axis_tdata(direct_axis_tdata),
        .m_axis_tvalid(direct_axis_tvalid),
        .m_axis_tready(direct_axis_tready),
        .m_axis_tlast(direct_axis_tlast),
        .m_axis_tuser(direct_axis_tuser),
        .sts_fifo_overflow_cnt(direct_fifo_overflow_count),
        .sts_back_pressure_cnt(direct_back_pressure_count)
    );
`endif

`ifdef MIPI_CAPTURE_PORTS
    logic [15:0] capture_fifo_overflow_count;
    logic [15:0] capture_back_pressure_count;
    logic [7:0] capture_axis_tlast_count;
    logic [7:0] capture_axis_sof_count;
    logic [15:0] capture_axis_line_pixels;
    logic [15:0] capture_axis_last_line_pixels;
    logic [4:0] phy_status_byte;
    wire [7:0] capture_input_data = CAPTURE_RAW_PAYLOAD ? frame_payload_data : video_pixel[7:0];
    wire capture_input_valid = CAPTURE_RAW_PAYLOAD ? frame_payload_valid : video_pixel_valid;
    wire capture_input_sof = CAPTURE_RAW_PAYLOAD ? frame_sof : video_pixel_sof;
    wire capture_input_eol = CAPTURE_RAW_PAYLOAD ? frame_payload_last : video_pixel_eol;
    wire capture_input_eof = CAPTURE_RAW_PAYLOAD ? frame_eof : video_pixel_eof;
    wire capture_input_err = CAPTURE_RAW_PAYLOAD ? frame_err : video_pixel_err;

    // Frame normalizer (2026-06-04): pin every capture frame to EXACTLY
    // 480x640 before the bridge/VDMA. The e2e sim (tb_e2e_vdma_stacking) proved
    // the assembly RTL framing is clean (1 SOF/frame) but a free-running
    // AXI-VDMA with chip-frame-lines != VSIZE tiles the buffer (~4 copies = the
    // observed horizontal-band stacking). Pinning frame == VSIZE removes the
    // mismatch (sim: free-run tiles 4 -> 1). Placed right after the unpack so it
    // keys off the (now standalone-eof-fixed) yuv422_gray_unpack frame markers.
    // FILL=0x80 keeps pad lines mid-gray (above the OB masker threshold).
    // Disabled 2026-06-06: the real root cause was the AXI-VDMA running free
    // (C_USE_S2MM_FSYNC=0) and ignoring SOF; with the VDMA now frame-synced to
    // TUSER (C_USE_S2MM_FSYNC=2, matching the Digilent reference) the VDMA itself
    // flushes/aligns each frame, so the normalizer is redundant. Bypassing it
    // also relieves Z7-020 routing congestion. Re-enable only if needed.
    localparam bit CAPTURE_NORMALIZE = 1'b0;
    logic [7:0] cn_data;
    logic       cn_valid, cn_sof, cn_eol, cn_eof, cn_err;

    video_frame_normalizer #(
        .OUT_LINES(480), .OUT_PIXELS(640), .FILL(8'h80), .NORMALIZE(CAPTURE_NORMALIZE)
    ) u_capture_frame_norm (
        .clk      (sysclk),
        .aresetn  (protocol_resetn),
        .in_data  (capture_input_data),
        .in_valid (capture_input_valid),
        .in_sof   (capture_input_sof),
        .in_eol   (capture_input_eol),
        .in_eof   (capture_input_eof),
        .in_err   (capture_input_err),
        .out_data (cn_data),
        .out_valid(cn_valid),
        .out_sof  (cn_sof),
        .out_eol  (cn_eol),
        .out_eof  (cn_eof),
        .out_err  (cn_err)
    );

    // OB-row masker (4-pixel uniformity check, 4-cycle pipeline).
    // Verified standalone via verification/tb/tb_ob_row_masker.sv.
    logic [7:0] ob_masked_data;
    logic       ob_masked_valid, ob_masked_sof, ob_masked_eol, ob_masked_eof, ob_masked_err;

    ob_row_masker #(
        .OB_THRESHOLD (8'd50),
        .OB_FILL_Y    (8'd128),
        .OB_UNIFORMITY(8'd3)
    ) u_ob_row_masker (
        .clk      (sysclk),
        .aresetn  (protocol_resetn),
        .enable   (!CAPTURE_RAW_PAYLOAD && !use_tpg_rt),
        .in_data  (cn_data),
        .in_valid (cn_valid),
        .in_sof   (cn_sof),
        .in_eol   (cn_eol),
        .in_eof   (cn_eof),
        .in_err   (cn_err),
        .out_data (ob_masked_data),
        .out_valid(ob_masked_valid),
        .out_sof  (ob_masked_sof),
        .out_eol  (ob_masked_eol),
        .out_eof  (ob_masked_eof),
        .out_err  (ob_masked_err)
    );

    // Phase 2 processing slot (image-processing research base): runtime-selectable
    // ops on the 24-bit RGB stream, between the format-mux video_pixel and the capture
    // bridge -> the processed pixels go to VDMA (PYNQ) AND the HDMI readout. ENABLE in
    // colour mode only; cfg_proc_op from idelay[23:21]. (Phase 2b swaps in a 3x3 conv.)
    logic [23:0] proc_pixel;
    logic        proc_valid, proc_sof, proc_eol, proc_eof, proc_err;
    axis_rgb_prefilter #(.LINE_PIXELS(640), .ENABLE(COLOR_CAPTURE)) u_rgb_prefilter (
        .clk      (sysclk),
        .rst_n    (protocol_resetn),
        // conv mode (op>=8): apply pre_op (0-9, incl 8 gaussian / 9 median); point mode
        // (op 0-7): legacy cam.proc(1..7) point ops via proc_op (zero SW change).
        .cfg_op   (cfg_proc_op_sys[3] ? pre_op_reg : {1'b0, cfg_proc_op_sys[2:0]}),
        .cfg_thresh_level (pre_thresh_reg),
        .in_pixel (video_pixel),
        .in_valid (video_pixel_valid),
        .in_sof   (video_pixel_sof),
        .in_eol   (video_pixel_eol),
        .in_eof   (video_pixel_eof),
        .in_err   (video_pixel_err),
        .out_pixel(proc_pixel),
        .out_valid(proc_valid),
        .out_sof  (proc_sof),
        .out_eol  (proc_eol),
        .out_eof  (proc_eof),
        .out_err  (proc_err)
    );

    // Phase 2b: 3x3 convolution after the point slot (same slot contract). cfg 8-11
    // select it (Gaussian/Sobel/sharpen); for point-op cfg (0-7) it passes through
    // (kernel 0). DSim-verified (verification/tb/tb_axis_rgb_conv3x3).
    logic [23:0] conv_pixel;
    logic        conv_valid, conv_sof, conv_eol, conv_eof, conv_err;
    axis_rgb_conv3x3 #(.LINE_PIXELS(640), .ENABLE(COLOR_CAPTURE)) u_rgb_conv3x3 (
        .clk       (sysclk),
        .rst_n     (protocol_resetn),
        .cfg_en    (cfg_proc_op_sys[3]),       // conv mode (point ops 0-7 -> passthrough)
        .cfg_coeffs(conv_coeffs_packed),       // runtime-programmable 3x3 kernel (0xFE0i)
        .cfg_shift (conv_shift_reg),
        .cfg_abs   (dog_abs_reg[0]),           // |grad| for A (Sobel magnitude path)
        .in_pixel  (proc_pixel),
        .in_valid  (proc_valid),
        .in_sof    (proc_sof),
        .in_eol    (proc_eol),
        .in_eof    (proc_eof),
        .in_err    (proc_err),
        .out_pixel (conv_pixel),
        .out_valid (conv_valid),
        .out_sof   (conv_sof),
        .out_eol   (conv_eol),
        .out_eof   (conv_eof),
        .out_err   (conv_err)
    );

    // DoG dual-kernel (op 12): general 5x5 (B branch) in PARALLEL with the 3x3 (A branch,
    // = conv_*), combined as clamp(alpha*A - beta*B + offset). conv5x5 is fed the SAME
    // proc_pixel as the 3x3; the combiner ordinal-FIFO aligns the two branches. DSim-
    // verified (tb_axis_rgb_dog). cfg_en gated by op==12 so it idles in single-conv modes.
    logic [23:0] conv5_pixel;
    logic        conv5_valid, conv5_sof, conv5_eol, conv5_eof, conv5_err;
    axis_rgb_conv5x5 #(.LINE_PIXELS(640), .ENABLE(COLOR_CAPTURE)) u_rgb_conv5x5 (
        .clk       (sysclk),
        .rst_n     (protocol_resetn),
        .cfg_en    (conv5_en_sys),               // S1: active for DoG (op12) + cascade (op13-15)
        .cfg_abs   (dog_abs_reg[1]),             // |grad| for B (Sobel magnitude path)
        .cfg_coeffs(conv5_coeffs_packed),
        .cfg_shift (conv5_shift_reg),
        .in_pixel  (proc_pixel),
        .in_valid  (proc_valid),
        .in_sof    (proc_sof),
        .in_eol    (proc_eol),
        .in_eof    (proc_eof),
        .in_err    (proc_err),
        .out_pixel (conv5_pixel),
        .out_valid (conv5_valid),
        .out_sof   (conv5_sof),
        .out_eol   (conv5_eol),
        .out_eof   (conv5_eof),
        .out_err   (conv5_err)
    );

    logic [23:0] dog_pixel;
    logic        dog_valid, dog_sof, dog_eol, dog_eof, dog_err;
    axis_rgb_dog_combine #(.ENABLE(COLOR_CAPTURE), .DEPTH(1024)) u_rgb_dog (
        .clk       (sysclk),
        .rst_n     (protocol_resetn),
        .cfg_mode  (dog_mode_reg),
        .cfg_alpha (dog_alpha_reg),
        .cfg_beta  (dog_beta_reg),
        .cfg_shift (dog_shift_reg),
        .cfg_offset({1'b0, dog_offset_reg}),     // 0..255 (e.g. 128 = DoG zero level)
        .a_pixel   (conv_pixel),                 // A = 3x3 branch (leads)
        .a_valid   (conv_valid),
        .b_pixel   (conv5_pixel),                // B = 5x5 branch (lags)
        .b_valid   (conv5_valid),
        .b_sof     (conv5_sof),
        .b_eol     (conv5_eol),
        .b_eof     (conv5_eof),
        .b_err     (conv5_err),
        .out_pixel (dog_pixel),
        .out_valid (dog_valid),
        .out_sof   (dog_sof),
        .out_eol   (dog_eol),
        .out_eof   (dog_eof),
        .out_err   (dog_err)
    );

    // Cascade blur stages S2 (fed by t1=conv5x5) -> S3, separable 5x5. op 14 = t2 (eff 9x9),
    // op 15 = t3 (eff 13x13). Identity reset = passthrough until a blur kernel is loaded.
    logic [23:0] s2_pixel, s3_pixel;
    logic        s2_valid, s2_sof, s2_eol, s2_eof, s2_err;
    logic        s3_valid, s3_sof, s3_eol, s3_eof, s3_err;
    axis_rgb_conv5x5_sep #(.LINE_PIXELS(640), .ENABLE(COLOR_CAPTURE)) u_rgb_s2 (
        .clk(sysclk), .rst_n(protocol_resetn),
        .cfg_h(s2_h_pk), .cfg_v(s2_v_pk), .cfg_hshift(s2_hsh_reg), .cfg_vshift(s2_vsh_reg),
        .in_pixel(conv5_pixel), .in_valid(conv5_valid), .in_sof(conv5_sof),
        .in_eol(conv5_eol), .in_eof(conv5_eof), .in_err(conv5_err),
        .out_pixel(s2_pixel), .out_valid(s2_valid), .out_sof(s2_sof),
        .out_eol(s2_eol), .out_eof(s2_eof), .out_err(s2_err));
    axis_rgb_conv5x5_sep #(.LINE_PIXELS(640), .ENABLE(COLOR_CAPTURE)) u_rgb_s3 (
        .clk(sysclk), .rst_n(protocol_resetn),
        .cfg_h(s3_h_pk), .cfg_v(s3_v_pk), .cfg_hshift(s3_hsh_reg), .cfg_vshift(s3_vsh_reg),
        .in_pixel(s2_pixel), .in_valid(s2_valid), .in_sof(s2_sof),
        .in_eol(s2_eol), .in_eof(s2_eof), .in_err(s2_err),
        .out_pixel(s3_pixel), .out_valid(s3_valid), .out_sof(s3_sof),
        .out_eol(s3_eol), .out_eof(s3_eof), .out_err(s3_err));

    // final processed stream mux by op: 12=DoG / 13=t1(5x5) / 14=t2(9x9) / 15=t3(13x13)
    // / else = single conv (op 8-11) or point path (op 0-7) = conv_pixel.
    logic [23:0] final_pixel; logic final_valid, final_sof, final_eol, final_eof, final_err;
    always_comb begin
        unique case (cfg_proc_op_sys)
            4'd12:   begin final_pixel=dog_pixel;   final_valid=dog_valid;   final_sof=dog_sof;
                           final_eol=dog_eol;       final_eof=dog_eof;       final_err=dog_err;   end
            4'd13:   begin final_pixel=conv5_pixel; final_valid=conv5_valid; final_sof=conv5_sof;
                           final_eol=conv5_eol;     final_eof=conv5_eof;     final_err=conv5_err; end
            4'd14:   begin final_pixel=s2_pixel;    final_valid=s2_valid;    final_sof=s2_sof;
                           final_eol=s2_eol;        final_eof=s2_eof;        final_err=s2_err;    end
            4'd15:   begin final_pixel=s3_pixel;    final_valid=s3_valid;    final_sof=s3_sof;
                           final_eol=s3_eol;        final_eof=s3_eof;        final_err=s3_err;    end
            default: begin final_pixel=conv_pixel;  final_valid=conv_valid;  final_sof=conv_sof;
                           final_eol=conv_eol;      final_eof=conv_eof;      final_err=conv_err;  end
        endcase
    end

    // Post-conv point op (plan 2026-06-25): a second proc_slot AFTER the final mux so a
    // point op (e.g. threshold) can run on the conv/edge result -> Sobel->binarize = a
    // binary edge map. post_op=0 (default) = passthrough = old behaviour (bit-identical).
    logic [23:0] post_pixel;
    logic        post_valid, post_sof, post_eol, post_eof, post_err;
    axis_rgb_proc_slot #(.ENABLE(COLOR_CAPTURE)) u_rgb_post_slot (
        .clk      (sysclk),
        .rst_n    (protocol_resetn),
        .cfg_op   (post_op_reg),
        .cfg_thresh_level (post_thresh_reg),
        .in_pixel (final_pixel),
        .in_valid (final_valid),
        .in_sof   (final_sof),
        .in_eol   (final_eol),
        .in_eof   (final_eof),
        .in_err   (final_err),
        .out_pixel(post_pixel),
        .out_valid(post_valid),
        .out_sof  (post_sof),
        .out_eol  (post_eol),
        .out_eof  (post_eof),
        .out_err  (post_err)
    );

    // Dither stage (plan 2026-06-26): final ordered(Bayer)/random(LFSR) dither + bit-depth
    // quantize AFTER post, before the capture bridge. cfg_ctrl=0 (0xFE4A) = passthrough.
    logic [23:0] dith_pixel;
    logic        dith_valid, dith_sof, dith_eol, dith_eof, dith_err;
    axis_rgb_dither #(.LINE_PIXELS(640), .ENABLE(COLOR_CAPTURE)) u_rgb_dither (
        .clk      (sysclk),
        .rst_n    (protocol_resetn),
        .cfg_ctrl (dither_ctrl_reg),
        .in_pixel (post_pixel), .in_valid(post_valid), .in_sof(post_sof),
        .in_eol   (post_eol),   .in_eof  (post_eof),   .in_err(post_err),
        .out_pixel(dith_pixel), .out_valid(dith_valid), .out_sof(dith_sof),
        .out_eol  (dith_eol),   .out_eof (dith_eof),   .out_err(dith_err)
    );

    // Capture bridge. COLOR_CAPTURE=1: the processed 24-bit RGB888 stream (point slot
    // + 3x3 conv / DoG dual-kernel + post-op on the muxed video_pixel) -> capture -> VDMA -> PYNQ + HDMI.
    // COLOR_CAPTURE=0: legacy Y8 path (ob_masked_data -> 8-bit capture).
    generate if (COLOR_CAPTURE) begin : g_capture_rgb24
        axis_video_bridge #(
            .TDATA_WIDTH(24),
            .TUSER_WIDTH(1),
            .FIFO_DEPTH(4096),
            .AXIS_TUSER_ERR_DEBUG(1'b0)
        ) u_capture_axis_video_bridge (
            .core_clk(sysclk),
            .core_aresetn(protocol_resetn),
            .aclk(capture_aclk),
            .aresetn(capture_aresetn),
            .in_pixel(dith_pixel),
            .in_pixel_valid(dith_valid),
            .in_pixel_sof(dith_sof),
            .in_pixel_eol(dith_eol),
            .in_pixel_eof(dith_eof),
            .in_pixel_err(dith_err),
            .m_axis_tdata(m_axis_capture_tdata),
            .m_axis_tvalid(m_axis_capture_tvalid),
            .m_axis_tready(m_axis_capture_tready),
            .m_axis_tlast(m_axis_capture_tlast),
            .m_axis_tuser(m_axis_capture_tuser),
            .sts_fifo_overflow_cnt(capture_fifo_overflow_count),
            .sts_back_pressure_cnt(capture_back_pressure_count)
        );
    end else begin : g_capture_y8
        axis_video_bridge #(
            .TDATA_WIDTH(8),
            .TUSER_WIDTH(1),
            .FIFO_DEPTH(4096),
            .AXIS_TUSER_ERR_DEBUG(1'b0)
        ) u_capture_axis_video_bridge (
            .core_clk(sysclk),
            .core_aresetn(protocol_resetn),
            .aclk(capture_aclk),
            .aresetn(capture_aresetn),
            .in_pixel(ob_masked_data),
            .in_pixel_valid(ob_masked_valid),
            .in_pixel_sof(ob_masked_sof),
            .in_pixel_eol(ob_masked_eol),
            .in_pixel_eof(ob_masked_eof),
            .in_pixel_err(ob_masked_err),
            .m_axis_tdata(m_axis_capture_tdata),
            .m_axis_tvalid(m_axis_capture_tvalid),
            .m_axis_tready(m_axis_capture_tready),
            .m_axis_tlast(m_axis_capture_tlast),
            .m_axis_tuser(m_axis_capture_tuser),
            .sts_fifo_overflow_cnt(capture_fifo_overflow_count),
            .sts_back_pressure_cnt(capture_back_pressure_count)
        );
    end endgenerate

    always_ff @(posedge phy_byte_clk or negedge rst_n) begin
        if (!rst_n) begin
            phy_status_byte <= 5'b00000;
            phy_sync_header_debug_word <= 32'h00000000;
            phy_stream_sop0_debug_word <= 32'h00000000;
            phy_stream_sop1_debug_word <= 32'h00000000;
            phy_stream_sop_second_pending <= 1'b0;
        end else begin
            phy_status_byte[4] <= 1'b1;
            if (phy_hs_clk_seen) begin
                phy_status_byte[3] <= 1'b1;
            end
            if (|phy_lane_sot_seen) begin
                phy_status_byte[2] <= 1'b1;
            end
            if (phy_stream_byte_valid) begin
                phy_status_byte[1] <= 1'b1;
            end
            if (phy_sync_header_valid) begin
                phy_status_byte[0] <= 1'b1;
                phy_sync_header_debug_word <= {
                    1'b1,
                    phy_sync_header_pairing,
                    phy_sync_header_score,
                    phy_sync_header_di,
                    phy_sync_header_wc
                };
            end
            if (phy_stream_byte_valid && phy_stream_byte_sop) begin
                phy_stream_sop0_debug_word <= {
                    1'b1,
                    phy_stream_pairing_active,
                    phy_stream_pairing_next,
                    9'h000,
                    phy_stream_byte_data
                };
                phy_stream_sop1_debug_word <= 32'h00000000;
                phy_stream_sop_second_pending <= 1'b1;
            end else if (phy_stream_byte_valid && phy_stream_sop_second_pending) begin
                phy_stream_sop1_debug_word <= {
                    1'b1,
                    15'h0000,
                    phy_stream_byte_data
                };
                phy_stream_sop_second_pending <= 1'b0;
            end
        end
    end

    always_ff @(posedge capture_aclk) begin
        if (!capture_aresetn) begin
            capture_axis_tlast_count <= 8'h00;
            capture_axis_sof_count <= 8'h00;
            capture_axis_line_pixels <= 16'h0000;
            capture_axis_last_line_pixels <= 16'h0000;
        end else if (m_axis_capture_tvalid && m_axis_capture_tready && m_axis_capture_tlast) begin
            capture_axis_tlast_count <= capture_axis_tlast_count + 8'd1;
            if (m_axis_capture_tuser[0]) begin
                capture_axis_sof_count <= capture_axis_sof_count + 8'd1;
                capture_axis_last_line_pixels <= 16'd1;
            end else begin
                capture_axis_last_line_pixels <= capture_axis_line_pixels + 16'd1;
            end
            capture_axis_line_pixels <= 16'h0000;
        end else if (m_axis_capture_tvalid && m_axis_capture_tready) begin
            if (m_axis_capture_tuser[0]) begin
                capture_axis_sof_count <= capture_axis_sof_count + 8'd1;
                capture_axis_line_pixels <= 16'd1;
            end else if (capture_axis_line_pixels != 16'hffff) begin
                capture_axis_line_pixels <= capture_axis_line_pixels + 16'd1;
            end
        end
    end
`endif

    yuv422_crc_framebuffer_axis #(
        .WIDTH(640),
        .HEIGHT(480),
        .LINE_BYTES(1280),
        .TDATA_WIDTH(24)
    ) u_yuv422_crc_framebuffer_axis (
        .core_clk(sysclk),
        .core_aresetn(protocol_resetn),
        .pix_clk(pix_clk),
        .pix_aresetn(pix_aresetn),
        .pkt_di(filter_pkt_di),
        .pkt_wc(filter_pkt_wc),
        .pkt_is_short(filter_pkt_is_short),
        .pkt_is_long(filter_pkt_is_long),
        .pkt_start(filter_pkt_start),
        .pkt_end(filter_pkt_end),
        .pkt_err(filter_pkt_err),
        .payload_data(filter_payload_data),
        .payload_valid(filter_payload_valid),
        .payload_first(filter_payload_first),
        .payload_last(filter_payload_last),
        .crc_check_valid(crc_check_valid),
        .crc_match(crc_match),
        .m_axis_tdata(fb_axis_tdata),
        .m_axis_tvalid(fb_axis_tvalid),
        .m_axis_tready(fb_axis_tready),
        .m_axis_tlast(fb_axis_tlast),
        .m_axis_tuser(fb_axis_tuser),
        .sts_good_line_count(fb_good_line_count),
        .sts_bad_line_count(fb_bad_line_count),
        .sts_frame_count(fb_frame_count),
        .sts_write_line(fb_write_line),
        .sts_frame_ready(fb_frame_ready)
    );

`ifdef MIPI_VDMA_LOOP_PORTS
    assign axis_tdata = s_axis_hdmi_tdata;
    assign axis_tvalid = s_axis_hdmi_tvalid;
    assign axis_tlast = s_axis_hdmi_tlast;
    assign axis_tuser[0] = s_axis_hdmi_tuser[0];
    assign axis_tuser2 = {1'b0, axis_tuser[0]};
    assign s_axis_hdmi_tready = axis_tready;
    assign fb_axis_tready = 1'b0;
    assign direct_axis_tready = 1'b0;
    assign pix_clk_out = pix_clk;
    assign pix_aresetn_out = pix_aresetn;
`else
    assign axis_tdata = USE_CRC_LINE_REPLAY ? fb_axis_tdata : direct_axis_tdata;
    assign axis_tvalid = USE_CRC_LINE_REPLAY ? fb_axis_tvalid : direct_axis_tvalid;
    assign axis_tlast = USE_CRC_LINE_REPLAY ? fb_axis_tlast : direct_axis_tlast;
    assign axis_tuser[0] = USE_CRC_LINE_REPLAY ? fb_axis_tuser[0] : direct_axis_tuser[0];
    assign axis_tuser2 = {1'b0, axis_tuser[0]};
    assign fb_axis_tready = USE_CRC_LINE_REPLAY ? axis_tready : 1'b0;
    assign direct_axis_tready = USE_CRC_LINE_REPLAY ? 1'b0 : axis_tready;
`endif

    logic [7:0] video_r;
    logic [7:0] video_g;
    logic [7:0] video_b;
    logic video_de;
    logic video_hsync;
    logic video_vsync;
    logic [9:0] tmds_data_0;
    logic [9:0] tmds_data_1;
    logic [9:0] tmds_data_2;
    logic [9:0] tmds_clk_word;
    logic hdmi_running;
    logic hdmi_hpd_seen;
    logic [31:0] hdmi_frame_count;
    logic [15:0] hdmi_underflow_count;
    logic [15:0] hdmi_axis_error_count;
    wire [3:0] tmds_serial;

    logic setup_ready_core;
    logic fs_seen_core;
    logic fe_seen_core;
    logic expected_long_seen_core;
    logic crc_ok_seen_core;
    logic crc_err_seen_core;
    logic frame_sof_seen_core;
    logic frame_eof_seen_core;
    logic frame_sync_err_seen_core;
    logic yuv_pixel_seen_core;
    logic direct_axis_take_seen_pix;
    logic direct_axis_sof_toggle_pix;
    logic hdmi_underflow_seen_pix;
    logic hdmi_axis_error_seen_pix;
    logic [3:0] pix_debug_sys_meta;
    logic [3:0] pix_debug_sys;
    logic [23:0] expected_long_event_count_core;
    logic [23:0] crc_ok_event_count_core;
    logic [23:0] crc_err_event_count_core;
    logic        direct_fifo_overflow_seen_core;
    logic        direct_back_pressure_seen_core;
    logic [26:0] direct_debug_word_core;
    logic        debug_last_hdr_seen_core;
    logic        debug_last_pkt_is_long_core;
    logic        debug_last_pkt_is_short_core;
    logic        debug_last_pkt_ecc_uncorr_core;
    logic        debug_last_crc_valid_core;
    logic        debug_last_crc_match_core;
    logic        debug_last_filter_err_core;
    logic [7:0]  debug_last_pkt_di_core;
    logic [15:0] debug_last_pkt_wc_core;
    logic [31:0] debug_page_word_core;
    // === Pipeline probe counters (added to diagnose where sync→counter chain breaks) ===
    // byte_clk domain pulse counters
    logic [15:0] phy_sync_header_valid_cnt_byte;
    logic [15:0] phy_stream_byte_sop_cnt_byte;
    logic [15:0] phy_header_valid_cnt_byte;
    // Raw long/short pkt counters (DI-based classification at sync_header_valid pulse,
    // independent of ECC validity or downstream parser state).
    logic [15:0] phy_raw_long_pkt_cnt_byte;
    logic [15:0] phy_raw_short_pkt_cnt_byte;
    logic        phy_sync_header_valid_d_byte;
    logic        phy_stream_byte_sop_d_byte;
    logic        phy_header_valid_d_byte;
    always_ff @(posedge phy_byte_clk or negedge rst_n) begin
        if (!rst_n) begin
            phy_sync_header_valid_cnt_byte <= 16'h0;
            phy_stream_byte_sop_cnt_byte   <= 16'h0;
            phy_header_valid_cnt_byte      <= 16'h0;
            phy_raw_long_pkt_cnt_byte      <= 16'h0;
            phy_raw_short_pkt_cnt_byte     <= 16'h0;
            phy_sync_header_valid_d_byte   <= 1'b0;
            phy_stream_byte_sop_d_byte     <= 1'b0;
            phy_header_valid_d_byte        <= 1'b0;
        end else begin
            phy_sync_header_valid_d_byte <= phy_sync_header_valid;
            phy_stream_byte_sop_d_byte   <= phy_stream_byte_sop;
            phy_header_valid_d_byte      <= phy_header_valid;
            if (phy_sync_header_valid && !phy_sync_header_valid_d_byte && phy_sync_header_valid_cnt_byte != 16'hFFFF)
                phy_sync_header_valid_cnt_byte <= phy_sync_header_valid_cnt_byte + 16'd1;
            if (phy_stream_byte_sop && phy_stream_byte_sop_cnt_byte != 16'hFFFF)
                phy_stream_byte_sop_cnt_byte <= phy_stream_byte_sop_cnt_byte + 16'd1;
            if (phy_header_valid && !phy_header_valid_d_byte && phy_header_valid_cnt_byte != 16'hFFFF)
                phy_header_valid_cnt_byte <= phy_header_valid_cnt_byte + 16'd1;
            // Raw classification on sync_header_valid rising edge: DI[5:0] >= 0x10 = long
            if (phy_sync_header_valid && !phy_sync_header_valid_d_byte) begin
                if (phy_sync_header_di[5:0] >= 6'h10) begin
                    if (phy_raw_long_pkt_cnt_byte != 16'hFFFF)
                        phy_raw_long_pkt_cnt_byte <= phy_raw_long_pkt_cnt_byte + 16'd1;
                end else begin
                    if (phy_raw_short_pkt_cnt_byte != 16'hFFFF)
                        phy_raw_short_pkt_cnt_byte <= phy_raw_short_pkt_cnt_byte + 16'd1;
                end
            end
        end
    end
    // 2FF CDC to sysclk
    (* ASYNC_REG = "TRUE" *) logic [15:0] phy_sync_header_valid_cnt_meta, phy_sync_header_valid_cnt_sys;
    (* ASYNC_REG = "TRUE" *) logic [15:0] phy_stream_byte_sop_cnt_meta, phy_stream_byte_sop_cnt_sys;
    (* ASYNC_REG = "TRUE" *) logic [15:0] phy_header_valid_cnt_meta, phy_header_valid_cnt_sys;
    (* ASYNC_REG = "TRUE" *) logic [15:0] phy_raw_long_pkt_cnt_meta, phy_raw_long_pkt_cnt_sys;
    (* ASYNC_REG = "TRUE" *) logic [15:0] phy_raw_short_pkt_cnt_meta, phy_raw_short_pkt_cnt_sys;
    always_ff @(posedge sysclk) begin
        phy_sync_header_valid_cnt_meta <= phy_sync_header_valid_cnt_byte;
        phy_sync_header_valid_cnt_sys  <= phy_sync_header_valid_cnt_meta;
        phy_stream_byte_sop_cnt_meta   <= phy_stream_byte_sop_cnt_byte;
        phy_stream_byte_sop_cnt_sys    <= phy_stream_byte_sop_cnt_meta;
        phy_header_valid_cnt_meta      <= phy_header_valid_cnt_byte;
        phy_header_valid_cnt_sys       <= phy_header_valid_cnt_meta;
        phy_raw_long_pkt_cnt_meta      <= phy_raw_long_pkt_cnt_byte;
        phy_raw_long_pkt_cnt_sys       <= phy_raw_long_pkt_cnt_meta;
        phy_raw_short_pkt_cnt_meta     <= phy_raw_short_pkt_cnt_byte;
        phy_raw_short_pkt_cnt_sys      <= phy_raw_short_pkt_cnt_meta;
    end
    // Pipeline FF for page 0x03 view_mode MUX — breaks the wide-mux + bit26-select
    // critical path (was WNS=-0.573ns post-route). Adds 1 sysclk latency to page 0x03
    // reads only; other pages remain combinational.
    logic [31:0] page03_view_mux_sys;
    always_ff @(posedge sysclk) begin
        page03_view_mux_sys <= frame_lines_runtime_word_sys[26]
            ? {phy_raw_short_pkt_cnt_sys, phy_raw_long_pkt_cnt_sys}
            : {sts_short_pkt_cnt, sts_long_pkt_cnt};
    end
    // sysclk domain counters
    logic [15:0] cdc_byte_sop_cnt_sys;
    logic [15:0] parser_ecc_hdr_valid_cnt_sys;
    logic [15:0] parser_pkt_hdr_valid_cnt_sys;
    logic        parser_ecc_hdr_valid_d_sys;
    logic        parser_pkt_hdr_valid_d_sys;
    // Per-type parser-OUTPUT packet counters (2026-06-17, diagnostic): count each
    // recognised packet header by type at the parser output, to localise the
    // scattered short-packet drop. If parser LS << parser long, the LS drop is at
    // or before the parser (frontend/parser); if parser LS ~= long, the drop is
    // downstream (parser->frame_state). Read on page 0x3F idx 16/17/18.
    logic [15:0] dbg_par_long, dbg_par_ls, dbg_par_le;
    always_ff @(posedge sysclk or negedge protocol_resetn) begin
        if (!protocol_resetn) begin
            cdc_byte_sop_cnt_sys         <= 16'h0;
            parser_ecc_hdr_valid_cnt_sys <= 16'h0;
            parser_pkt_hdr_valid_cnt_sys <= 16'h0;
            parser_ecc_hdr_valid_d_sys   <= 1'b0;
            parser_pkt_hdr_valid_d_sys   <= 1'b0;
            dbg_par_long <= 16'h0; dbg_par_ls <= 16'h0; dbg_par_le <= 16'h0;
        end else begin
            parser_ecc_hdr_valid_d_sys <= parser_ecc_hdr_valid;
            parser_pkt_hdr_valid_d_sys <= parser_pkt_hdr_valid;
            if (cdc_byte_valid && cdc_byte_sop && cdc_byte_sop_cnt_sys != 16'hFFFF)
                cdc_byte_sop_cnt_sys <= cdc_byte_sop_cnt_sys + 16'd1;
            if (parser_ecc_hdr_valid && !parser_ecc_hdr_valid_d_sys && parser_ecc_hdr_valid_cnt_sys != 16'hFFFF)
                parser_ecc_hdr_valid_cnt_sys <= parser_ecc_hdr_valid_cnt_sys + 16'd1;
            if (parser_pkt_hdr_valid && !parser_pkt_hdr_valid_d_sys) begin
                if (parser_pkt_hdr_valid_cnt_sys != 16'hFFFF)
                    parser_pkt_hdr_valid_cnt_sys <= parser_pkt_hdr_valid_cnt_sys + 16'd1;
                if (parser_pkt_is_long) begin
                    if (dbg_par_long != 16'hFFFF) dbg_par_long <= dbg_par_long + 16'd1;
                end else if (parser_pkt_di[5:0] == 6'h02) begin
                    if (dbg_par_ls != 16'hFFFF) dbg_par_ls <= dbg_par_ls + 16'd1;
                end else if (parser_pkt_di[5:0] == 6'h03) begin
                    if (dbg_par_le != 16'hFFFF) dbg_par_le <= dbg_par_le + 16'd1;
                end
            end
        end
    end
    logic [15:0] frame_asm_fs_cnt_core;
    logic [15:0] frame_asm_fe_cnt_core;
    logic [15:0] frame_asm_ls_cnt_core;
    logic [15:0] frame_asm_le_cnt_core;
    logic [15:0] frame_asm_other_short_cnt_core;
    logic [15:0] frame_asm_live_lines_core;
    logic [15:0] frame_asm_last_fe_lines_core;
    logic [15:0] frame_asm_fe_before_480_cnt_core;
    logic [15:0] frame_asm_fe_after_480_cnt_core;
    logic [15:0] frame_asm_fs_overlap_cnt_core;
    logic [15:0] frame_asm_fe_without_fs_cnt_core;
    logic [15:0] frame_asm_long_before_fs_cnt_core;
    logic [7:0]  frame_asm_last_short_di_core;
    logic [15:0] frame_asm_last_short_wc_core;
    logic        frame_asm_in_frame_core;

    // FS-to-FS span statistics (task 1, 2026-06-02): glitch-free hardware latch
    // of the per-frame line span at each frame close (out_eof). sts_last_frame_lines
    // is updated by csi2_frame_state on the same cycle out_eof pulses, so sampling
    // it on frame_eof gives the true FS-to-FS span. Accumulating min/max/sum/count
    // here (one read of pages 0x34-0x36) yields the span distribution without the
    // software-polling glitches that corrupted the 2026-06-01 tight-loop attempt.
    // span_min==span_max => constant chip frame period (rolling is downstream/FPGA);
    // wide min..max => chip MIPI-TX frame-period jitter.
    logic [15:0] span_min_core;
    logic [15:0] span_max_core;
    logic [31:0] span_sum_core;
    logic [15:0] span_cnt_core;
    logic [15:0] span_last_core;

    // LP-11 idle-duration probe (2026-06-03, user insight): MIPI frame boundary =
    // long LP-11 idle (vertical blanking); inter-line gaps are short. Measuring the
    // LP-11 run length on data lane0 (sampled in sysclk) reveals whether a robust
    // long-idle = frame boundary exists, so frames can be delimited by physical LP
    // state instead of the unreliable DT-0x00 (FS) short-packet byte decode.
    // 3 lanes watched: 0=clock lane, 1=data lane0, 2=data lane1. Measures LP-11
    // run_max per lane + raw current state, to find which LP signal toggles and
    // whether a long-idle (vertical blanking = frame boundary) exists.
    // dbg 0x37=clk run_max, 0x38=data0 run_max, 0x39=data1 run_max,
    //     0x3A={raw clk/d0/d1 lp[1:0] (6b), 10'h0, clk runs_total[15:0]}.
    (* ASYNC_REG = "TRUE" *) logic [1:0] lpc_sync1, lpc_sync2;  // clock lane
    (* ASYNC_REG = "TRUE" *) logic [1:0] lp0_sync1, lp0_sync2;  // data lane0
    (* ASYNC_REG = "TRUE" *) logic [1:0] lp1_sync1, lp1_sync2;  // data lane1
    logic [2:0]  lp11_in, lp11_in_d;            // per-watched-lane LP-11 now/prev
    logic [31:0] lp_run_core   [3];
    logic [31:0] lp_run_max_core [3];
    logic [15:0] lp_runs_total_core [3];

    function automatic [15:0] sat_inc16_probe(input [15:0] value);
        if (value == 16'hffff) begin
            sat_inc16_probe = value;
        end else begin
            sat_inc16_probe = value + 16'd1;
        end
    endfunction

    wire expected_long_event_core = parser_pkt_hdr_valid && parser_pkt_is_long && (parser_pkt_di[5:0] == 6'h1e) && (parser_pkt_wc == 16'd1280);
    wire crc_ok_event_core = crc_check_valid && crc_match;
    wire crc_err_event_core = crc_check_valid && !crc_match;
    wire frame_asm_short_start_core = filter_pkt_start && filter_pkt_is_short;
    wire frame_asm_long_end_core = filter_pkt_end && filter_pkt_is_long;
    wire [5:0] frame_asm_short_dt_core = filter_pkt_di[5:0];
    wire ov5640_chip_id_ok_core = (sccb_chip_id_high == 8'h56) && (sccb_chip_id_low == 8'h40);
    wire ov5640_readback_ok_core = sccb_done && !sccb_error && (sccb_ack_error_count == 8'h00) &&
        (sccb_step_index == OV5640_SCCB_LAST_STEP) && ov5640_chip_id_ok_core &&
        (sccb_rd_mipi_ctrl_300e == 8'h45) && (sccb_rd_mipi_ctrl_4800 == OV5640_MIPI_CTRL_4800) &&
        (sccb_rd_mipi_ctrl_4805 == 8'h10) && (sccb_rd_mipi_ctrl_4837 == 8'h18) &&
        (sccb_rd_format_ctrl_4300 == OV5640_FORMAT_CTRL_4300) && (sccb_rd_isp_format_501f == OV5640_ISP_FORMAT_501F) &&
        (sccb_rd_isp_ctrl_5000 == OV5640_ISP_CTRL_5000) && (sccb_rd_isp_ctrl_5001 == OV5640_ISP_CTRL_5001) &&
        (sccb_rd_timing_ctrl_3824 == 8'h02) && (sccb_rd_jpeg_ctrl_4407 == 8'h04) &&
        (sccb_rd_mipi_ctrl_440e == 8'h00) && (sccb_rd_vfifo_ctrl_460b == 8'h35) &&
        (sccb_rd_vfifo_ctrl_460c == 8'h22) && (sccb_rd_awb_5189 == 8'h88) &&
        (sccb_rd_output_width_high_3808 == 8'h02) && (sccb_rd_output_width_low_3809 == 8'h80) &&
        (sccb_rd_output_height_high_380a == 8'h01) && (sccb_rd_output_height_low_380b == 8'he0) &&
        (sccb_rd_aec_manual_3503 == 8'h00) && (sccb_rd_aec_ctrl_3a13 == 8'h43) &&
        (sccb_rd_aec_gain_ceiling_high_3a18 == 8'h00) && (sccb_rd_aec_gain_ceiling_low_3a19 == 8'hf8);

    assign setup_ready_core = hdmi_mmcm_locked && ref_pll_locked && ov5640_readback_ok_core;

    always_ff @(posedge sysclk) begin
        if (!protocol_resetn) begin
            fs_seen_core <= 1'b0;
            fe_seen_core <= 1'b0;
            expected_long_seen_core <= 1'b0;
            crc_ok_seen_core <= 1'b0;
            crc_err_seen_core <= 1'b0;
            frame_sof_seen_core <= 1'b0;
            frame_eof_seen_core <= 1'b0;
            frame_sync_err_seen_core <= 1'b0;
            yuv_pixel_seen_core <= 1'b0;
            expected_long_event_count_core <= 24'h000000;
            crc_ok_event_count_core <= 24'h000000;
            crc_err_event_count_core <= 24'h000000;
            direct_fifo_overflow_seen_core <= 1'b0;
            direct_back_pressure_seen_core <= 1'b0;
            debug_last_hdr_seen_core <= 1'b0;
            debug_last_pkt_is_long_core <= 1'b0;
            debug_last_pkt_is_short_core <= 1'b0;
            debug_last_pkt_ecc_uncorr_core <= 1'b0;
            debug_last_crc_valid_core <= 1'b0;
            debug_last_crc_match_core <= 1'b0;
            debug_last_filter_err_core <= 1'b0;
            debug_last_pkt_di_core <= 8'h00;
            debug_last_pkt_wc_core <= 16'h0000;
        end else begin
            if (direct_fifo_overflow_count != 16'h0000) begin
                direct_fifo_overflow_seen_core <= 1'b1;
            end
            if (direct_back_pressure_count != 16'h0000) begin
                direct_back_pressure_seen_core <= 1'b1;
            end
            if (parser_pkt_hdr_valid) begin
                debug_last_hdr_seen_core <= 1'b1;
                debug_last_pkt_is_long_core <= parser_pkt_is_long;
                debug_last_pkt_is_short_core <= parser_pkt_is_short;
                debug_last_pkt_ecc_uncorr_core <= parser_pkt_ecc_uncorrectable;
                debug_last_pkt_di_core <= parser_pkt_di;
                debug_last_pkt_wc_core <= parser_pkt_wc;
            end
            if (crc_check_valid) begin
                debug_last_crc_valid_core <= 1'b1;
                debug_last_crc_match_core <= crc_match;
            end
            if (filter_pkt_end) begin
                debug_last_filter_err_core <= filter_pkt_err;
            end
            if (filter_pkt_start && filter_pkt_is_short && (filter_pkt_di[5:0] == 6'h00)) begin
                fs_seen_core <= 1'b1;
            end
            if (filter_pkt_start && filter_pkt_is_short && (filter_pkt_di[5:0] == 6'h01)) begin
                fe_seen_core <= 1'b1;
            end
            if (expected_long_event_core) begin
                expected_long_seen_core <= 1'b1;
                expected_long_event_count_core <= expected_long_event_count_core + 24'd1;
            end
            if (crc_ok_event_core) begin
                crc_ok_seen_core <= 1'b1;
                crc_ok_event_count_core <= crc_ok_event_count_core + 24'd1;
            end
            if (crc_err_event_core) begin
                crc_err_seen_core <= 1'b1;
                crc_err_event_count_core <= crc_err_event_count_core + 24'd1;
            end
            if (frame_sof) begin
                frame_sof_seen_core <= 1'b1;
            end
            if (frame_eof) begin
                frame_eof_seen_core <= 1'b1;
            end
            if (sts_frame_sync_err_cnt != 16'h0000) begin
                frame_sync_err_seen_core <= 1'b1;
            end
            if (video_pixel_valid) begin
                yuv_pixel_seen_core <= 1'b1;
            end
        end
    end

    // FS-to-FS span accumulator: latch sts_last_frame_lines on every frame close.
    always_ff @(posedge sysclk) begin
        if (!protocol_resetn) begin
            span_min_core  <= 16'hffff;
            span_max_core  <= 16'h0000;
            span_sum_core  <= 32'h0000_0000;
            span_cnt_core  <= 16'h0000;
            span_last_core <= 16'h0000;
        end else if (frame_eof) begin
            span_last_core <= sts_last_frame_lines;
            span_cnt_core  <= sat_inc16_probe(span_cnt_core);
            span_sum_core  <= span_sum_core + {16'h0000, sts_last_frame_lines};
            if (sts_last_frame_lines < span_min_core) span_min_core <= sts_last_frame_lines;
            if (sts_last_frame_lines > span_max_core) span_max_core <= sts_last_frame_lines;
        end
    end

    // LP-11 idle-duration accumulator: clock lane + both data lanes (sysclk).
    always_ff @(posedge sysclk) begin
        if (!protocol_resetn) begin
            lpc_sync1 <= 2'b11; lpc_sync2 <= 2'b11;
            lp0_sync1 <= 2'b11; lp0_sync2 <= 2'b11;
            lp1_sync1 <= 2'b11; lp1_sync2 <= 2'b11;
            lp11_in <= 3'b111; lp11_in_d <= 3'b111;
            for (int k=0;k<3;k++) begin
                lp_run_core[k] <= 32'd0; lp_run_max_core[k] <= 32'd0; lp_runs_total_core[k] <= 16'd0;
            end
        end else begin
            lpc_sync1 <= {dphy_clk_lp_p,    dphy_clk_lp_n};    lpc_sync2 <= lpc_sync1;
            lp0_sync1 <= {dphy_data_lp_p[0], dphy_data_lp_n[0]}; lp0_sync2 <= lp0_sync1;
            lp1_sync1 <= {dphy_data_lp_p[1], dphy_data_lp_n[1]}; lp1_sync2 <= lp1_sync1;
            lp11_in[0] <= (lpc_sync2 == 2'b11);
            lp11_in[1] <= (lp0_sync2 == 2'b11);
            lp11_in[2] <= (lp1_sync2 == 2'b11);
            lp11_in_d <= lp11_in;
            for (int k=0;k<3;k++) begin
                if (lp11_in[k]) begin
                    lp_run_core[k] <= lp_run_core[k] + 32'd1;
                end else begin
                    if (lp11_in_d[k]) begin
                        lp_runs_total_core[k] <= sat_inc16_probe(lp_runs_total_core[k]);
                        if (lp_run_core[k] > lp_run_max_core[k]) lp_run_max_core[k] <= lp_run_core[k];
                    end
                    lp_run_core[k] <= 32'd0;
                end
            end
        end
    end

    always_ff @(posedge sysclk) begin
        if (!protocol_resetn) begin
            frame_asm_fs_cnt_core <= 16'h0000;
            frame_asm_fe_cnt_core <= 16'h0000;
            frame_asm_ls_cnt_core <= 16'h0000;
            frame_asm_le_cnt_core <= 16'h0000;
            frame_asm_other_short_cnt_core <= 16'h0000;
            frame_asm_live_lines_core <= 16'h0000;
            frame_asm_last_fe_lines_core <= 16'h0000;
            frame_asm_fe_before_480_cnt_core <= 16'h0000;
            frame_asm_fe_after_480_cnt_core <= 16'h0000;
            frame_asm_fs_overlap_cnt_core <= 16'h0000;
            frame_asm_fe_without_fs_cnt_core <= 16'h0000;
            frame_asm_long_before_fs_cnt_core <= 16'h0000;
            frame_asm_last_short_di_core <= 8'h00;
            frame_asm_last_short_wc_core <= 16'h0000;
            frame_asm_in_frame_core <= 1'b0;
        end else begin
            if (frame_asm_short_start_core) begin
                frame_asm_last_short_di_core <= filter_pkt_di;
                frame_asm_last_short_wc_core <= filter_pkt_wc;

                unique case (frame_asm_short_dt_core)
                    6'h00: begin
                        frame_asm_fs_cnt_core <= sat_inc16_probe(frame_asm_fs_cnt_core);
                        if (frame_asm_in_frame_core) begin
                            frame_asm_fs_overlap_cnt_core <= sat_inc16_probe(frame_asm_fs_overlap_cnt_core);
                        end
                        frame_asm_in_frame_core <= 1'b1;
                        frame_asm_live_lines_core <= 16'h0000;
                    end
                    6'h01: begin
                        frame_asm_fe_cnt_core <= sat_inc16_probe(frame_asm_fe_cnt_core);
                        if (frame_asm_in_frame_core) begin
                            frame_asm_last_fe_lines_core <= frame_asm_live_lines_core;
                            if (frame_asm_live_lines_core < 16'd480) begin
                                frame_asm_fe_before_480_cnt_core <= sat_inc16_probe(frame_asm_fe_before_480_cnt_core);
                            end else if (frame_asm_live_lines_core > 16'd480) begin
                                frame_asm_fe_after_480_cnt_core <= sat_inc16_probe(frame_asm_fe_after_480_cnt_core);
                            end
                            frame_asm_in_frame_core <= 1'b0;
                        end else begin
                            frame_asm_fe_without_fs_cnt_core <= sat_inc16_probe(frame_asm_fe_without_fs_cnt_core);
                        end
                    end
                    6'h02: frame_asm_ls_cnt_core <= sat_inc16_probe(frame_asm_ls_cnt_core);
                    6'h03: frame_asm_le_cnt_core <= sat_inc16_probe(frame_asm_le_cnt_core);
                    default: frame_asm_other_short_cnt_core <= sat_inc16_probe(frame_asm_other_short_cnt_core);
                endcase
            end

            if (frame_asm_long_end_core) begin
                if (frame_asm_in_frame_core) begin
                    frame_asm_live_lines_core <= sat_inc16_probe(frame_asm_live_lines_core);
                end else begin
                    frame_asm_long_before_fs_cnt_core <= sat_inc16_probe(frame_asm_long_before_fs_cnt_core);
                end
            end
        end
    end

    always_ff @(posedge pix_clk) begin
        if (!pix_aresetn) begin
            direct_axis_take_seen_pix <= 1'b0;
            direct_axis_sof_toggle_pix <= 1'b0;
            hdmi_underflow_seen_pix <= 1'b0;
            hdmi_axis_error_seen_pix <= 1'b0;
        end else begin
            if (axis_tvalid && axis_tready) begin
                direct_axis_take_seen_pix <= 1'b1;
            end
            if (axis_tvalid && axis_tready && axis_tuser[0]) begin
                direct_axis_sof_toggle_pix <= ~direct_axis_sof_toggle_pix;
            end
            if (hdmi_underflow_count != 16'h0000) begin
                hdmi_underflow_seen_pix <= 1'b1;
            end
            if (hdmi_axis_error_count != 16'h0000) begin
                hdmi_axis_error_seen_pix <= 1'b1;
            end
        end
    end

    always_ff @(posedge sysclk) begin
        if (!rst_n) begin
            pix_debug_sys_meta <= 4'h0;
            pix_debug_sys <= 4'h0;
        end else begin
            pix_debug_sys_meta <= {direct_axis_sof_toggle_pix, direct_axis_take_seen_pix, hdmi_underflow_seen_pix, hdmi_axis_error_seen_pix};
            pix_debug_sys <= pix_debug_sys_meta;
        end
    end

`ifdef MIPI_VDMA_LOOP_PORTS
    assign direct_debug_word_core = {
        setup_ready_core,
        ov5640_chip_id_ok_core,
        sccb_done,
        expected_long_seen_core,
        crc_ok_seen_core,
        crc_err_seen_core,
        yuv_pixel_seen_core,
        pix_debug_sys,
        expected_long_event_count_core[7:0],
        8'h00
    };
`else
    assign direct_debug_word_core = {
        setup_ready_core,
        ov5640_chip_id_ok_core,
        sccb_error,
        expected_long_event_count_core[7:0],
        crc_ok_event_count_core[7:0],
        crc_err_event_count_core[7:0]
    };
`endif

    // Source-side pipeline register for direct_debug_word_core. Lets the placer
    // keep the CDC source close to direct_debug_sync1_reg in mipi_to_hdmi_probe_top,
    // shortening the long route exposed by 0x3035=0x21 (D-PHY rebuild) congestion.
    logic [26:0] direct_debug_word_core_q;
    always_ff @(posedge sysclk) begin
        if (!rst_n) begin
            direct_debug_word_core_q <= 27'h0000000;
        end else begin
            direct_debug_word_core_q <= direct_debug_word_core;
        end
    end

    always_comb begin
        unique case (debug_page_sel_sys)
            5'h01: debug_page_word_core = {
                debug_last_hdr_seen_core,
                debug_last_pkt_is_long_core,
                debug_last_pkt_is_short_core,
                debug_last_pkt_ecc_uncorr_core,
                debug_last_crc_valid_core,
                debug_last_crc_match_core,
                debug_last_filter_err_core,
                expected_long_seen_core,
                debug_last_pkt_di_core,
                debug_last_pkt_wc_core
            };
            5'h02: debug_page_word_core = {sts_crc_ok_cnt, sts_crc_err_cnt};
            // view_mode_raw = frame_lines_runtime_word_sys[26]:
            //   0 = parser counter (default, backward compat)
            //   1 = raw D-PHY counter (DI-based, independent of parser/ECC)
            // Note: bit 24 = APPLY strobe, bit 25 = cam_gpio. Bit 26 is free.
            // MUX output is pipelined via page03_view_mux_sys to break critical path.
            5'h03: debug_page_word_core = page03_view_mux_sys;
            5'h04: debug_page_word_core = {sts_pkt_trunc_cnt, sts_ecc_uncorr_cnt};
            5'h05: debug_page_word_core = {sts_last_frame_lines, video_pixel_per_line};
            // 0x06: D-PHY lane supervisor status. [31:24]=settle_cnt [23:16]=lock_cnt
            //   [15:10]=0 [9:7]=data_state [6:4]=clk_state
            //   [3]=sup_enable [2]=bufr_clr [1]=rx_clk_active [0]=hs_settled
            5'h06: debug_page_word_core = {sup_status_sys[25:10], 6'h00, sup_status_sys[9:0]};
            5'h07: debug_page_word_core = {sts_drop_dt_cnt, sts_drop_vc_cnt};
            5'h08: debug_page_word_core = {fb_good_line_count, fb_bad_line_count};
            5'h09: debug_page_word_core = phy_sync_header_debug_word;
            5'h0a: debug_page_word_core = phy_stream_sop0_debug_word;
            5'h0b: debug_page_word_core = phy_stream_sop1_debug_word;
            5'h0c: debug_page_word_core = cdc_stream_sop0_debug_word;
            5'h0d: debug_page_word_core = cdc_stream_sop1_debug_word;
            5'h0e: debug_page_word_core = {16'h0000, sts_byte_cdc_ovf_cnt};
            5'h0f: debug_page_word_core = {phy_live_trace_slot_lane0_aligned[3], phy_live_trace_slot_lane0_aligned[2], phy_live_trace_slot_lane0_aligned[1], phy_live_trace_slot_lane0_aligned[0]};
            5'h10: debug_page_word_core = {phy_live_trace_slot_lane1_aligned[3], phy_live_trace_slot_lane1_aligned[2], phy_live_trace_slot_lane1_aligned[1], phy_live_trace_slot_lane1_aligned[0]};
            5'h11: debug_page_word_core = {phy_live_trace_seq, phy_live_trace_slot_valid, phy_live_trace_slot_sot_hit_lane1, phy_live_trace_slot_sot_hit_lane0};
            5'h12: debug_page_word_core = {
                phy_sync_header_valid,
                phy_sync_header_pairing,
                phy_sync_header_score,
                phy_sync_header_bit_offset_lane0,
                phy_sync_header_bit_offset_lane1,
                phy_sync_header_ecc_no_error,
                phy_sync_header_ecc_corrected,
                phy_sync_header_ecc_uncorrectable,
                phy_sync_header_syndrome,
                phy_sync_header_ecc,
                1'b0
            };
            5'h13: debug_page_word_core = {phy_live_trace_seq, phy_obs_compare_flags, phy_stream_sop_wc};
            5'h14: debug_page_word_core = {phy_live_trace_slot_lane0_candidate[3], phy_live_trace_slot_lane0_candidate[2], phy_live_trace_slot_lane0_candidate[1], phy_live_trace_slot_lane0_candidate[0]};
            5'h15: debug_page_word_core = {phy_live_trace_slot_lane1_candidate[3], phy_live_trace_slot_lane1_candidate[2], phy_live_trace_slot_lane1_candidate[1], phy_live_trace_slot_lane1_candidate[0]};
            5'h16: debug_page_word_core = {phy_live_trace_slot_lane0_raw[3], phy_live_trace_slot_lane0_raw[2], phy_live_trace_slot_lane0_raw[1], phy_live_trace_slot_lane0_raw[0]};
            5'h17: debug_page_word_core = {phy_live_trace_slot_lane1_raw[3], phy_live_trace_slot_lane1_raw[2], phy_live_trace_slot_lane1_raw[1], phy_live_trace_slot_lane1_raw[0]};
            5'h18: debug_page_word_core = {frame_asm_fs_cnt_core, frame_asm_fe_cnt_core};
            5'h19: debug_page_word_core = {frame_asm_ls_cnt_core, frame_asm_le_cnt_core};
            5'h1a: debug_page_word_core = {8'h00, frame_asm_last_short_di_core, frame_asm_last_short_wc_core};
            5'h1b: debug_page_word_core = {frame_asm_live_lines_core, frame_asm_last_fe_lines_core};
            5'h1c: debug_page_word_core = {frame_asm_fe_before_480_cnt_core, frame_asm_fe_after_480_cnt_core};
            5'h1d: debug_page_word_core = {frame_asm_fs_overlap_cnt_core, frame_asm_fe_without_fs_cnt_core};
            5'h1e: debug_page_word_core = {frame_asm_other_short_cnt_core, frame_asm_long_before_fs_cnt_core};
            5'h1f: debug_page_word_core = {sts_frame_count_core[15:0], sts_line_count_core[15:0]};
            // === Pipeline probe pages ===
            // 0x20: sync_header_valid pulses (byte_clk) | stream_byte_sop pulses (byte_clk)
            6'h20: debug_page_word_core = {phy_sync_header_valid_cnt_sys, phy_stream_byte_sop_cnt_sys};
            // 0x21: header_valid pulses (byte_clk) | cdc_byte_sop pulses (sysclk, post-CDC)
            6'h21: debug_page_word_core = {phy_header_valid_cnt_sys, cdc_byte_sop_cnt_sys};
            6'h22: debug_page_word_core = {phy_live_trace_slot_lane0_aligned[7], phy_live_trace_slot_lane0_aligned[6], phy_live_trace_slot_lane0_aligned[5], phy_live_trace_slot_lane0_aligned[4]};
            6'h23: debug_page_word_core = {phy_live_trace_slot_lane1_aligned[7], phy_live_trace_slot_lane1_aligned[6], phy_live_trace_slot_lane1_aligned[5], phy_live_trace_slot_lane1_aligned[4]};
            6'h24: debug_page_word_core = {phy_live_trace_slot_lane0_candidate[7], phy_live_trace_slot_lane0_candidate[6], phy_live_trace_slot_lane0_candidate[5], phy_live_trace_slot_lane0_candidate[4]};
            6'h25: debug_page_word_core = {phy_live_trace_slot_lane1_candidate[7], phy_live_trace_slot_lane1_candidate[6], phy_live_trace_slot_lane1_candidate[5], phy_live_trace_slot_lane1_candidate[4]};
            6'h26: debug_page_word_core = {phy_live_trace_slot_lane0_raw[7], phy_live_trace_slot_lane0_raw[6], phy_live_trace_slot_lane0_raw[5], phy_live_trace_slot_lane0_raw[4]};
            6'h27: debug_page_word_core = {phy_live_trace_slot_lane1_raw[7], phy_live_trace_slot_lane1_raw[6], phy_live_trace_slot_lane1_raw[5], phy_live_trace_slot_lane1_raw[4]};
            // 0x28: parser_ecc_hdr_valid (sysclk) | parser_pkt_hdr_valid (sysclk)
            6'h28: debug_page_word_core = {parser_ecc_hdr_valid_cnt_sys, parser_pkt_hdr_valid_cnt_sys};
            // 0x2a: supervisor diagnostic (ctl_clk-sourced; not shadowed by the
            // VDMA-loop 0x00/0x06 override). Reliable ck_state for continuous-clock
            // bring-up debug (diary 2026-06-14 Phase 2b).
            6'h2a: debug_page_word_core = sup_dbg_sys;
            // 0x2b: SoT-miss counts {sot_burst_count[31:16], burst_count[15:0]}.
            // 0x2c: last no-SoT burst-head bytes {b3,b2,b1,b0} (lane0 candidate).
            6'h2b: debug_page_word_core = sot_miss_sys;
            6'h2c: debug_page_word_core = missed_burst_sys;
            // 0x2d: vblank-exit re-lock latency {relock_max[31:16], last[15:0]} byte_clk.
            6'h2d: debug_page_word_core = relock_sys;
            // 0x2e: HW lock FSM {failed,locked,state[3],reroll[4],combo[6],p0[3],p1[3],hdr_active}.
            6'h2e: debug_page_word_core = hwlock_sys;
            // 0x30 (moved from duplicate 0x20): SCCB test pattern + engine state
            6'h30: debug_page_word_core = {
                sccb_rt_test_pattern_pending,
                sccb_rt_test_pattern_ready,
                sccb_rt_test_pattern_done,
                sccb_rt_test_pattern_error,
                sccb_busy,
                sccb_done,
                sccb_error,
                sccb_rt_test_pattern_enable,
                sccb_ack_error_count,
                sccb_rt_ack_error_count,
                sccb_rt_test_pattern_value
            };
            // 0x31 (moved from duplicate 0x21): AEC readback bundle
            6'h31: debug_page_word_core = {
                sccb_rd_aec_manual_3503,
                sccb_rd_aec_ctrl_3a13,
                sccb_rd_aec_gain_ceiling_high_3a18,
                sccb_rd_aec_gain_ceiling_low_3a19
            };
            // 0x32 (moved from duplicate 0x28): lane0 rotation slot snapshot
            6'h32: debug_page_word_core = {
                8'h00,
                phy_live_trace_slot_lane0_rotation[7], phy_live_trace_slot_lane0_rotation[6], phy_live_trace_slot_lane0_rotation[5], phy_live_trace_slot_lane0_rotation[4],
                phy_live_trace_slot_lane0_rotation[3], phy_live_trace_slot_lane0_rotation[2], phy_live_trace_slot_lane0_rotation[1], phy_live_trace_slot_lane0_rotation[0]
            };
            // 0x33 (NEW): raw long/short pkt counter (D-PHY layer, DI-based, independent of parser/ECC)
            6'h33: debug_page_word_core = {phy_raw_short_pkt_cnt_sys, phy_raw_long_pkt_cnt_sys};
            // 0x34-0x36 (task 1, 2026-06-02): FS-to-FS span stats (glitch-free chip frame-period jitter probe)
            6'h34: debug_page_word_core = {span_min_core, span_max_core};
            6'h35: debug_page_word_core = {span_cnt_core, span_last_core};
            6'h36: debug_page_word_core = span_sum_core;
            // 0x37-0x3A (2026-06-03): LP-11 idle-duration probe (frame-boundary via physical LP state)
            6'h37: debug_page_word_core = lp_run_max_core[0];   // clock-lane LP-11 longest run
            6'h38: debug_page_word_core = lp_run_max_core[1];   // data-lane0 LP-11 longest run
            6'h39: debug_page_word_core = lp_run_max_core[2];   // data-lane1 LP-11 longest run
            6'h3a: debug_page_word_core = {lpc_sync2, lp0_sync2, lp1_sync2, 10'h0, lp_runs_total_core[0]};
            // 0x3B (P=0x9b): raw TPG output SOP count (before mux)
            6'h3b: debug_page_word_core = {16'h0000, tpg_sop_cnt_core};
            // 0x3C (P=0x9c): mux output SOP count (what parser sees); should match tpg when use_tpg_rt=1
            6'h3c: debug_page_word_core = {16'h0000, pkt_sop_cnt_core};
            // 0x3D (P=0x9d): SOP data comparison — [31:16]=pkt (parser input) [15:0]=tpg (pipeline reg)
            //   mux working:  both = 0x0022 (TPG long DI) when use_tpg_rt=1
            //   mux broken:   pkt = camera DI (0x1Exx), tpg = 0x0022
            //   tpg wrong:    tpg = 0x0000 or 0xDEAD → TPG module not driving output
            6'h3d: debug_page_word_core = {pkt_sop_data_latch, tpg_sop_data_latch};
            // 0x3E (P=0x9e): camera SOP reference — [31:16]=cdc_data_r at camera SOP, [15:0]=unused
            6'h3e: debug_page_word_core = {cdc_sop_data_latch, 16'h0000};
            6'h29: debug_page_word_core = {
                8'h00,
                phy_live_trace_slot_lane1_rotation[7], phy_live_trace_slot_lane1_rotation[6], phy_live_trace_slot_lane1_rotation[5], phy_live_trace_slot_lane1_rotation[4],
                phy_live_trace_slot_lane1_rotation[3], phy_live_trace_slot_lane1_rotation[2], phy_live_trace_slot_lane1_rotation[1], phy_live_trace_slot_lane1_rotation[0]
            };
            // 0x3F (NEW 2026-06-16): boundary packet trace readout.
            //   [7:0]=snap entry {in_frame, is_long, DT[5:0]}, [12:8]=snap_wptr
            //   (oldest-entry slot for linearisation), [13]=have (a snapshot taken).
            //   index = idelay GPIO [20:16], freeze = idelay GPIO [25].
            // idx 0/1/2 return the frame_state long-disposition counters (accept /
            // no-LS reject / IDLE reject); idx >= 3 return the boundary trace entry.
            // idx 0/1/2 = long disposition (accept / no-LS / IDLE); idx 8..15 =
            // no-LS reject position histogram bucket (idx-8); else boundary trace.
            6'h3f: debug_page_word_core =
                (btrace_idx == 5'd0) ? {16'h0, sts_dbg_long_accept} :
                (btrace_idx == 5'd1) ? {16'h0, sts_dbg_long_nols}   :
                (btrace_idx == 5'd2) ? {16'h0, sts_dbg_long_idle}   :
                (btrace_idx[4:3] == 2'b01) ? {16'h0, sts_dbg_nols_hist[(btrace_idx[2:0])*16 +: 16]} :
                (btrace_idx == 5'd16) ? {16'h0, dbg_par_long} :
                (btrace_idx == 5'd17) ? {16'h0, dbg_par_ls}   :
                (btrace_idx == 5'd18) ? {16'h0, dbg_par_le}   :
                                       {18'h0, btrace_have, btrace_snap_wptr, btrace_rd};
            default: debug_page_word_core = 32'h00000000;
        endcase
    end

`ifdef MIPI_CAPTURE_PORTS
    logic [4:0] phy_status_sync1;
    logic [4:0] phy_status_sync2;
    logic [26:0] direct_debug_sync1;
    logic [26:0] direct_debug_sync2;
    logic [31:0] core_debug_page_sync1;
    logic [31:0] core_debug_page_sync2;
`ifdef MIPI_VDMA_LOOP_PORTS
    logic [7:0] debug_control_capture_meta;
    logic [7:0] debug_control_capture;
`endif

    always_ff @(posedge capture_aclk) begin
        if (!capture_aresetn) begin
            phy_status_sync1 <= 5'b00000;
            phy_status_sync2 <= 5'b00000;
            direct_debug_sync1 <= 27'h0000000;
            direct_debug_sync2 <= 27'h0000000;
            core_debug_page_sync1 <= 32'h00000000;
            core_debug_page_sync2 <= 32'h00000000;
`ifdef MIPI_VDMA_LOOP_PORTS
            debug_control_capture_meta <= 8'h00;
            debug_control_capture <= 8'h00;
`endif
            capture_debug <= 32'h00000000;
        end else begin
            phy_status_sync1 <= phy_status_byte;
            phy_status_sync2 <= phy_status_sync1;
            direct_debug_sync1 <= direct_debug_word_core_q;
            direct_debug_sync2 <= direct_debug_sync1;
            core_debug_page_sync1 <= debug_page_word_core;
            core_debug_page_sync2 <= core_debug_page_sync1;
`ifdef MIPI_VDMA_LOOP_PORTS
            debug_control_capture_meta <= debug_page_sel;
            debug_control_capture <= debug_control_capture_meta;
            unique case ({debug_control_capture[7], debug_control_capture[4:0]})
                6'h00: capture_debug <= {phy_status_sync2, direct_debug_sync2[26:8], capture_axis_tlast_count};
                6'h06: capture_debug <= {capture_axis_tlast_count, capture_axis_sof_count, capture_axis_last_line_pixels};
                default: capture_debug <= core_debug_page_sync2;
            endcase
`else
            capture_debug <= {phy_status_sync2, direct_debug_sync2};
`endif
        end
    end
`endif

    hdmi_output #(
        .H_ACTIVE(640), .H_FRONT_PORCH(16), .H_SYNC(96), .H_BACK_PORCH(48),
        .V_ACTIVE(480), .V_FRONT_PORCH(10), .V_SYNC(2), .V_BACK_PORCH(33),
        .HSYNC_POLARITY(1'b0), .VSYNC_POLARITY(1'b0)
    ) u_hdmi_output (
        .pix_clk(pix_clk), .pix_aresetn(pix_aresetn), .enable(1'b1), .soft_reset(1'b0), .test_pattern_en(1'b0), .hpd(hdmi_tx_hpd), .hpd_override(1'b1),
        .s_axis_tdata(axis_tdata), .s_axis_tvalid(axis_tvalid), .s_axis_tready(axis_tready), .s_axis_tlast(axis_tlast), .s_axis_tuser(axis_tuser),
        .video_r(video_r), .video_g(video_g), .video_b(video_b), .video_de(video_de), .video_hsync(video_hsync), .video_vsync(video_vsync),
        .tmds_data_0(tmds_data_0), .tmds_data_1(tmds_data_1), .tmds_data_2(tmds_data_2), .tmds_clk_word(tmds_clk_word),
        .sts_running(hdmi_running), .sts_hpd(hdmi_hpd_seen), .sts_frame_count(hdmi_frame_count), .sts_underflow_count(hdmi_underflow_count), .sts_axis_error_count(hdmi_axis_error_count)
    );

    tmds_serializer_10b u_serialize_blue (.tmds_clk(tmds_clk), .pix_clk(pix_clk), .reset(!pix_aresetn), .tmds_word(tmds_data_0), .tmds_serial(tmds_serial[0]));
    tmds_serializer_10b u_serialize_green (.tmds_clk(tmds_clk), .pix_clk(pix_clk), .reset(!pix_aresetn), .tmds_word(tmds_data_1), .tmds_serial(tmds_serial[1]));
    tmds_serializer_10b u_serialize_red (.tmds_clk(tmds_clk), .pix_clk(pix_clk), .reset(!pix_aresetn), .tmds_word(tmds_data_2), .tmds_serial(tmds_serial[2]));
    tmds_serializer_10b u_serialize_clock (.tmds_clk(tmds_clk), .pix_clk(pix_clk), .reset(!pix_aresetn), .tmds_word(tmds_clk_word), .tmds_serial(tmds_serial[3]));

    OBUFDS u_tmds_clk_obufds (.I(tmds_serial[3]), .O(hdmi_tx_clk_p), .OB(hdmi_tx_clk_n));

    for (genvar lane = 0; lane < 3; lane++) begin : gen_tmds_data_obufds
        OBUFDS u_tmds_data_obufds (.I(tmds_serial[lane]), .O(hdmi_tx_p[lane]), .OB(hdmi_tx_n[lane]));
    end

    assign hdmi_tx_scl = 1'b1;
    assign hdmi_tx_sda = 1'bz;
    assign hdmi_tx_cec = 1'b0;

`ifdef MIPI_VDMA_LOOP_PORTS
    always_ff @(posedge sysclk) begin
        led[0] <= ov5640_chip_id_ok_core;
        led[1] <= sccb_done;
        led[2] <= sccb_step_index[5];
        led[3] <= use_tpg_rt ? tpg_sop_cnt_core[15] : setup_ready_core;
    end
`else
    always_ff @(posedge sysclk) begin
        led[0] <= setup_ready_core;
        led[1] <= crc_ok_event_count_core[14];
        led[2] <= crc_err_event_count_core[14];
        led[3] <= use_tpg_rt ? tpg_sop_cnt_core[15] : direct_fifo_overflow_seen_core;
    end
`endif

endmodule

`default_nettype wire
