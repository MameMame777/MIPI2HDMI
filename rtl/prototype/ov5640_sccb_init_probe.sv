`timescale 1ns / 1ps
`default_nettype none

module ov5640_sccb_init_probe #(
    parameter int CLK_HZ = 125_000_000,
    parameter int I2C_HZ = 100_000,
    parameter int POWERUP_DELAY_MS = 50,
    parameter logic [7:0] MIPI_CTRL_300E_IDLE_2LANE = 8'h44,
    parameter logic [7:0] MIPI_CTRL_300E_STREAM_2LANE = 8'h45,
    parameter logic [7:0] MIPI_CTRL_4800 = 8'h24,
    parameter logic [7:0] FORMAT_CTRL_4300 = 8'h30,
    parameter logic [7:0] ISP_FORMAT_501F = 8'h00,
    parameter logic [7:0] ISP_CTRL_5000 = 8'ha7,
    parameter logic [7:0] ISP_CTRL_5001 = 8'h83,
    parameter bit TEST_PATTERN_ENABLE = 1'b0,
    parameter bit USE_EXTERNAL_IOBUF = 1'b0
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        rt_test_pattern_valid,
    input  wire        rt_test_pattern_enable,
    output logic       rt_test_pattern_ready,
    output logic       rt_test_pattern_done,
    output logic       rt_test_pattern_error,
    output logic [7:0] rt_test_pattern_value,
    output logic [7:0] rt_ack_error_count,
    input  wire        rt_reg_write_valid,
    input  wire [15:0] rt_reg_write_addr,
    input  wire [7:0]  rt_reg_write_value,
    output logic       rt_reg_write_ready,
    output logic       rt_reg_write_done,
    output logic       rt_reg_write_error,
    output logic       rt_reg_write_busy,
    output logic [7:0] rt_reg_write_ack_err_count,
    output logic [15:0] rt_reg_write_last_addr,
    input  wire        rt_reg_read_valid,
    input  wire [15:0] rt_reg_read_addr,
    output logic       rt_reg_read_ready,
    output logic       rt_reg_read_done,
    output logic       rt_reg_read_error,
    output logic [7:0] rt_reg_read_data,
    output logic [15:0] rt_reg_read_last_addr,
    inout  wire        cam_scl,
    inout  wire        cam_sda,
    output wire        scl_drive_low_o,
    output wire        sda_drive_low_o,
    output logic       busy,
    output logic       done,
    output logic       error,
    output logic [7:0] chip_id_high,
    output logic [7:0] chip_id_low,
    output logic [7:0] ack_error_count,
    output logic [8:0] step_index,
    output logic [7:0] rd_mipi_ctrl_300e,
    output logic [7:0] rd_mipi_ctrl_4800,
    output logic [7:0] rd_mipi_ctrl_4805,
    output logic [7:0] rd_mipi_ctrl_4837,
    output logic [7:0] rd_format_ctrl_4300,
    output logic [7:0] rd_isp_format_501f,
    output logic [7:0] rd_isp_ctrl_5000,
    output logic [7:0] rd_isp_ctrl_5001,
    output logic [7:0] rd_timing_ctrl_3824,
    output logic [7:0] rd_jpeg_ctrl_4407,
    output logic [7:0] rd_mipi_ctrl_440e,
    output logic [7:0] rd_vfifo_ctrl_460b,
    output logic [7:0] rd_vfifo_ctrl_460c,
    output logic [7:0] rd_awb_5189,
    output logic [7:0] rd_output_width_high_3808,
    output logic [7:0] rd_output_width_low_3809,
    output logic [7:0] rd_output_height_high_380a,
    output logic [7:0] rd_output_height_low_380b,
    output logic [7:0] rd_aec_manual_3503,
    output logic [7:0] rd_aec_ctrl_3a13,
    output logic [7:0] rd_aec_gain_ceiling_high_3a18,
    output logic [7:0] rd_aec_gain_ceiling_low_3a19,
    output wire        dbg_scl_in,
    output wire        dbg_sda_in,
    output logic       dbg_ack_low_seen,
    output logic       dbg_scl_low_seen,
    output logic       dbg_scl_high_seen,
    output logic       dbg_sda_low_seen,
    output logic       dbg_sda_high_seen
);

    localparam int HALF_PERIOD_CYCLES = CLK_HZ / (I2C_HZ * 2);
    localparam int POWERUP_DELAY_CYCLES = (CLK_HZ / 1000) * POWERUP_DELAY_MS;
    localparam logic [7:0] DEV_ADDR_W = 8'h78;
    localparam logic [7:0] DEV_ADDR_R = 8'h79;
    localparam int LAST_STEP = 260;  // 2026-05-30: flat ROM (261 entries 0..260) replaces 4-task dispatch + read-back drop

    typedef enum logic [1:0] {
        OP_READ,
        OP_WRITE
    } op_t;

    typedef enum logic [5:0] {
        ST_POWERUP_WAIT,
        ST_LOAD_STEP,
        ST_START_A,
        ST_START_B,
        ST_START_C,
        ST_BYTE_SETUP,
        ST_BYTE_HIGH,
        ST_BYTE_LOW,
        ST_ACK_SETUP,
        ST_ACK_HIGH,
        ST_ACK_LOW,
        ST_NEXT,
        ST_READ_SETUP,
        ST_READ_HIGH,
        ST_READ_LOW,
        ST_MASTER_NACK_SETUP,
        ST_MASTER_NACK_HIGH,
        ST_MASTER_NACK_LOW,
        ST_STOP_A,
        ST_STOP_B,
        ST_STEP_DELAY,
        ST_DONE
    } state_t;

    state_t state;
    op_t op;
    logic [$clog2(HALF_PERIOD_CYCLES+1)-1:0] tick_count;
    logic [31:0] delay_count;
    logic [31:0] delay_target;
    logic scl_drive_low;
    logic sda_drive_low;
    logic [7:0] tx_byte;
    logic [7:0] rx_byte;
    logic [2:0] bit_index;
    logic [2:0] byte_index;
    logic [15:0] reg_addr;
    logic [7:0] reg_value;
    logic [15:0] step_delay_ms;
    logic runtime_active;
    logic [1:0] runtime_kind; // 0 = test pattern toggle, 1 = arbitrary write, 2 = arbitrary read

    generate
        if (!USE_EXTERNAL_IOBUF) begin : gen_internal_iobuf
            assign cam_scl = scl_drive_low ? 1'b0 : 1'bz;
            assign cam_sda = sda_drive_low ? 1'b0 : 1'bz;
        end
    endgenerate

    assign scl_drive_low_o = scl_drive_low;
    assign sda_drive_low_o = sda_drive_low;
    assign rt_test_pattern_ready = (state == ST_DONE) && done && !busy;
    assign rt_reg_write_ready = (state == ST_DONE) && done && !busy;
    assign rt_reg_write_busy = busy && runtime_active && (runtime_kind == 2'b01);
    assign rt_reg_read_ready = (state == ST_DONE) && done && !busy;

    (* keep = "true" *) wire scl_in = cam_scl;
    (* keep = "true" *) wire sda_in = cam_sda;

    assign dbg_scl_in = scl_in;
    assign dbg_sda_in = sda_in;

    function automatic logic half_tick_done(input logic [$clog2(HALF_PERIOD_CYCLES+1)-1:0] value);
        half_tick_done = (value == HALF_PERIOD_CYCLES[$clog2(HALF_PERIOD_CYCLES+1)-1:0] - 1'b1);
    endfunction

    // 2026-05-30 RTL refactor: replaced the 4-task dispatch chain
    // (get_inline_step / get_linux_analog_step / get_low_res_helper_step /
    //  get_isp_table_step + get_step dispatch) with a flat distributed-ROM
    // lookup. The original nested dispatch synthesised to a deep mux that
    // hit Z7-020 routing congestion (557 node overlaps at 49% slice
    // utilisation, route_design failed after 1:54 h). The flat ROM packs
    // all 261 entries as a {op, addr, value, delay_ms} word indexed
    // directly by step_index, eliminating the dispatch overhead.
    //
    // Layout (all writes unless noted):
    //   0..11    chip ID read + SW reset + PLL setup (orig inline 0..11)
    //   12..38   Linux mainline ANALOG batch 27 reg (NEW for stripe fix)
    //   39..74   system/MIPI/timing/format (orig inline 12..47)
    //   75..108  low_res helper banding/AEC (orig helper 0..33)
    //   109..254 ISP table AWB/CMTX/gamma/LENC (orig isp 0..145)
    //   255..259 post-init writes 0x503D/4837/4814/4202/300E (orig inline 228..232)
    //   260      stream-on 0x3008=0x02 (orig inline 233)
    //
    // Diagnostic read-backs (orig inline 234..255, 22 entries) DROPPED -
    // runtime sccb_read() (since commit 82ab667) supersedes them.
    //
    // Entry packing (41 bits):
    //   [40]    op (0=WRITE, 1=READ)
    //   [39:24] addr[15:0]
    //   [23:16] value[7:0]
    //   [15:0]  delay_ms[15:0]

    localparam int INIT_ROM_DEPTH = 261;
    (* rom_style = "distributed" *)
    logic [40:0] init_rom [0:INIT_ROM_DEPTH-1];

    initial begin
        init_rom[  0] = { 1'b1, 16'h300a, 8'h00, 16'd0 };
        init_rom[  1] = { 1'b1, 16'h300b, 8'h00, 16'd0 };
        init_rom[  2] = { 1'b0, 16'h3008, 8'h82, 16'd1000 };
        init_rom[  3] = { 1'b0, 16'h3008, 8'h42, 16'd20 };
        init_rom[  4] = { 1'b1, 16'h300a, 8'h00, 16'd0 };
        init_rom[  5] = { 1'b1, 16'h300b, 8'h00, 16'd0 };
        init_rom[  6] = { 1'b0, 16'h3103, 8'h03, 16'd0 };
        init_rom[  7] = { 1'b0, 16'h3034, 8'h18, 16'd0 };
        init_rom[  8] = { 1'b0, 16'h3035, 8'h12, 16'd0 };
        init_rom[  9] = { 1'b0, 16'h3036, 8'h60, 16'd0 };  // mult=96 (30fps): VCO=24*96/3=768, PCLK=VCO/16=48MHz->30fps@1600x1000, link_freq=VCO/mipi_div(2)=384MHz=768Mbps (mainline 30fps low-res). XDC dphy_hs re-constrained to 384MHz. (was 0x30 mult48=15fps)
        init_rom[ 10] = { 1'b0, 16'h3037, 8'h13, 16'd0 };
        init_rom[ 11] = { 1'b0, 16'h3108, 8'h01, 16'd0 };
        init_rom[ 12] = { 1'b0, 16'h3601, 8'h33, 16'd0 };
        init_rom[ 13] = { 1'b0, 16'h3620, 8'h52, 16'd0 };
        init_rom[ 14] = { 1'b0, 16'h3621, 8'he0, 16'd0 };
        init_rom[ 15] = { 1'b0, 16'h3622, 8'h01, 16'd0 };
        init_rom[ 16] = { 1'b0, 16'h3630, 8'h36, 16'd0 };
        init_rom[ 17] = { 1'b0, 16'h3631, 8'h0e, 16'd0 };
        init_rom[ 18] = { 1'b0, 16'h3632, 8'he2, 16'd0 };
        init_rom[ 19] = { 1'b0, 16'h3633, 8'h12, 16'd0 };
        init_rom[ 20] = { 1'b0, 16'h3634, 8'h40, 16'd0 };
        init_rom[ 21] = { 1'b0, 16'h3635, 8'h13, 16'd0 };
        init_rom[ 22] = { 1'b0, 16'h3636, 8'h03, 16'd0 };
        init_rom[ 23] = { 1'b0, 16'h3703, 8'h5a, 16'd0 };
        init_rom[ 24] = { 1'b0, 16'h3704, 8'ha0, 16'd0 };
        init_rom[ 25] = { 1'b0, 16'h3705, 8'h1a, 16'd0 };
        init_rom[ 26] = { 1'b0, 16'h370b, 8'h60, 16'd0 };
        init_rom[ 27] = { 1'b0, 16'h3715, 8'h78, 16'd0 };
        init_rom[ 28] = { 1'b0, 16'h3717, 8'h01, 16'd0 };
        init_rom[ 29] = { 1'b0, 16'h371b, 8'h20, 16'd0 };
        init_rom[ 30] = { 1'b0, 16'h3731, 8'h12, 16'd0 };
        init_rom[ 31] = { 1'b0, 16'h302d, 8'h60, 16'd0 };
        init_rom[ 32] = { 1'b0, 16'h3c01, 8'ha4, 16'd0 };
        init_rom[ 33] = { 1'b0, 16'h3c04, 8'h28, 16'd0 };
        init_rom[ 34] = { 1'b0, 16'h3c05, 8'h98, 16'd0 };
        init_rom[ 35] = { 1'b0, 16'h3901, 8'h0a, 16'd0 };
        init_rom[ 36] = { 1'b0, 16'h3905, 8'h02, 16'd0 };
        init_rom[ 37] = { 1'b0, 16'h3906, 8'h10, 16'd0 };
        init_rom[ 38] = { 1'b0, 16'h5001, 8'ha3, 16'd0 };
        init_rom[ 39] = { 1'b0, 16'h3000, 8'h00, 16'd0 };
        init_rom[ 40] = { 1'b0, 16'h3002, 8'h1c, 16'd0 };
        init_rom[ 41] = { 1'b0, 16'h3004, 8'hff, 16'd0 };
        init_rom[ 42] = { 1'b0, 16'h3006, 8'hc3, 16'd0 };
        init_rom[ 43] = { 1'b0, 16'h302e, 8'h08, 16'd0 };
        init_rom[ 44] = { 1'b0, 16'h300e, MIPI_CTRL_300E_IDLE_2LANE, 16'd1 };
        init_rom[ 45] = { 1'b0, 16'h4800, MIPI_CTRL_4800, 16'd0 };
        init_rom[ 46] = { 1'b0, 16'h3019, 8'h70, 16'd1 };
        init_rom[ 47] = { 1'b0, 16'h3820, 8'h41, 16'd0 };
        init_rom[ 48] = { 1'b0, 16'h3821, 8'h07, 16'd0 };
        init_rom[ 49] = { 1'b0, 16'h3814, 8'h31, 16'd0 };
        init_rom[ 50] = { 1'b0, 16'h3815, 8'h31, 16'd0 };
        init_rom[ 51] = { 1'b0, 16'h3800, 8'h00, 16'd0 };
        init_rom[ 52] = { 1'b0, 16'h3801, 8'h10, 16'd0 };
        init_rom[ 53] = { 1'b0, 16'h3802, 8'h00, 16'd0 };
        init_rom[ 54] = { 1'b0, 16'h3803, 8'h0e, 16'd0 };
        init_rom[ 55] = { 1'b0, 16'h3804, 8'h0a, 16'd0 };
        init_rom[ 56] = { 1'b0, 16'h3805, 8'h2f, 16'd0 };
        init_rom[ 57] = { 1'b0, 16'h3806, 8'h07, 16'd0 };
        init_rom[ 58] = { 1'b0, 16'h3807, 8'ha5, 16'd0 };
        init_rom[ 59] = { 1'b0, 16'h3808, 8'h02, 16'd0 };
        init_rom[ 60] = { 1'b0, 16'h3809, 8'h80, 16'd0 };
        init_rom[ 61] = { 1'b0, 16'h380a, 8'h01, 16'd0 };
        init_rom[ 62] = { 1'b0, 16'h380b, 8'he0, 16'd0 };
        init_rom[ 63] = { 1'b0, 16'h380c, 8'h06, 16'd0 };
        init_rom[ 64] = { 1'b0, 16'h380d, 8'h40, 16'd0 };
        init_rom[ 65] = { 1'b0, 16'h380e, 8'h03, 16'd0 };
        init_rom[ 66] = { 1'b0, 16'h380f, 8'he8, 16'd0 };
        init_rom[ 67] = { 1'b0, 16'h3810, 8'h00, 16'd0 };
        init_rom[ 68] = { 1'b0, 16'h3811, 8'h02, 16'd0 };
        init_rom[ 69] = { 1'b0, 16'h3812, 8'h00, 16'd0 };
        init_rom[ 70] = { 1'b0, 16'h3813, 8'h04, 16'd0 };
        init_rom[ 71] = { 1'b0, 16'h4300, FORMAT_CTRL_4300, 16'd0 };
        init_rom[ 72] = { 1'b0, 16'h501f, ISP_FORMAT_501F, 16'd0 };
        init_rom[ 73] = { 1'b0, 16'h5000, ISP_CTRL_5000, 16'd0 };
        init_rom[ 74] = { 1'b0, 16'h5001, ISP_CTRL_5001, 16'd0 };
        init_rom[ 75] = { 1'b0, 16'h3c07, 8'h08, 16'd0 };
        init_rom[ 76] = { 1'b0, 16'h3c09, 8'h1c, 16'd0 };
        init_rom[ 77] = { 1'b0, 16'h3c0a, 8'h9c, 16'd0 };
        init_rom[ 78] = { 1'b0, 16'h3c0b, 8'h40, 16'd0 };
        init_rom[ 79] = { 1'b0, 16'h3618, 8'h00, 16'd0 };
        init_rom[ 80] = { 1'b0, 16'h3612, 8'h29, 16'd0 };
        init_rom[ 81] = { 1'b0, 16'h3708, 8'h64, 16'd0 };
        init_rom[ 82] = { 1'b0, 16'h3709, 8'h52, 16'd0 };
        init_rom[ 83] = { 1'b0, 16'h370c, 8'h03, 16'd0 };
        init_rom[ 84] = { 1'b0, 16'h3a02, 8'h03, 16'd0 };
        init_rom[ 85] = { 1'b0, 16'h3a03, 8'hd8, 16'd0 };
        init_rom[ 86] = { 1'b0, 16'h3a08, 8'h01, 16'd0 };  // B50 hi: 300=0x012C (mainline @mult48)
        init_rom[ 87] = { 1'b0, 16'h3a09, 8'h2c, 16'd0 };  // B50 lo (was 0x27=295 @mult54)
        init_rom[ 88] = { 1'b0, 16'h3a0a, 8'h00, 16'd0 };  // B60 hi: 250=0x00FA (mainline @mult48)
        init_rom[ 89] = { 1'b0, 16'h3a0b, 8'hfa, 16'd0 };  // B60 lo (was 0xF6=246 @mult54)
        init_rom[ 90] = { 1'b0, 16'h3a0e, 8'h03, 16'd0 };  // max_band50 = (VTS-4)/300 = 3
        init_rom[ 91] = { 1'b0, 16'h3a0d, 8'h03, 16'd0 };  // max_band60 = (VTS-4)/250 = 3 (was 0x04)
        init_rom[ 92] = { 1'b0, 16'h3a14, 8'h03, 16'd0 };
        init_rom[ 93] = { 1'b0, 16'h3a15, 8'hd8, 16'd0 };
        init_rom[ 94] = { 1'b0, 16'h3503, 8'h00, 16'd0 };
        init_rom[ 95] = { 1'b0, 16'h3a00, 8'h78, 16'd0 };
        init_rom[ 96] = { 1'b0, 16'h3a01, 8'h01, 16'd0 };
        init_rom[ 97] = { 1'b0, 16'h3a13, 8'h43, 16'd0 };
        init_rom[ 98] = { 1'b0, 16'h3a18, 8'h00, 16'd0 };
        init_rom[ 99] = { 1'b0, 16'h3a19, 8'hf8, 16'd0 };
        init_rom[100] = { 1'b0, 16'h3a1a, 8'h04, 16'd0 };
        init_rom[101] = { 1'b0, 16'h4001, 8'h02, 16'd0 };
        init_rom[102] = { 1'b0, 16'h4004, 8'h02, 16'd0 };
        init_rom[103] = { 1'b0, 16'h4407, 8'h04, 16'd0 };
        init_rom[104] = { 1'b0, 16'h440e, 8'h00, 16'd0 };
        init_rom[105] = { 1'b0, 16'h460b, 8'h35, 16'd0 };
        init_rom[106] = { 1'b0, 16'h460c, 8'h22, 16'd0 };
        init_rom[107] = { 1'b0, 16'h3824, 8'h02, 16'd0 };
        init_rom[108] = { 1'b0, 16'h5001, ISP_CTRL_5001, 16'd0 };
        init_rom[109] = { 1'b0, 16'h5180, 8'hff, 16'd0 };
        init_rom[110] = { 1'b0, 16'h5181, 8'hf2, 16'd0 };
        init_rom[111] = { 1'b0, 16'h5182, 8'h00, 16'd0 };
        init_rom[112] = { 1'b0, 16'h5183, 8'h14, 16'd0 };
        init_rom[113] = { 1'b0, 16'h5184, 8'h25, 16'd0 };
        init_rom[114] = { 1'b0, 16'h5185, 8'h24, 16'd0 };
        init_rom[115] = { 1'b0, 16'h5186, 8'h09, 16'd0 };
        init_rom[116] = { 1'b0, 16'h5187, 8'h09, 16'd0 };
        init_rom[117] = { 1'b0, 16'h5188, 8'h09, 16'd0 };
        init_rom[118] = { 1'b0, 16'h5189, 8'h88, 16'd0 };
        init_rom[119] = { 1'b0, 16'h518a, 8'h54, 16'd0 };
        init_rom[120] = { 1'b0, 16'h518b, 8'hee, 16'd0 };
        init_rom[121] = { 1'b0, 16'h518c, 8'hb2, 16'd0 };
        init_rom[122] = { 1'b0, 16'h518d, 8'h50, 16'd0 };
        init_rom[123] = { 1'b0, 16'h518e, 8'h34, 16'd0 };
        init_rom[124] = { 1'b0, 16'h518f, 8'h6b, 16'd0 };
        init_rom[125] = { 1'b0, 16'h5190, 8'h46, 16'd0 };
        init_rom[126] = { 1'b0, 16'h5191, 8'hf8, 16'd0 };
        init_rom[127] = { 1'b0, 16'h5192, 8'h04, 16'd0 };
        init_rom[128] = { 1'b0, 16'h5193, 8'h70, 16'd0 };
        init_rom[129] = { 1'b0, 16'h5194, 8'hf0, 16'd0 };
        init_rom[130] = { 1'b0, 16'h5195, 8'hf0, 16'd0 };
        init_rom[131] = { 1'b0, 16'h5196, 8'h03, 16'd0 };
        init_rom[132] = { 1'b0, 16'h5197, 8'h01, 16'd0 };
        init_rom[133] = { 1'b0, 16'h5198, 8'h04, 16'd0 };
        init_rom[134] = { 1'b0, 16'h5199, 8'h6c, 16'd0 };
        init_rom[135] = { 1'b0, 16'h519a, 8'h04, 16'd0 };
        init_rom[136] = { 1'b0, 16'h519b, 8'h00, 16'd0 };
        init_rom[137] = { 1'b0, 16'h519c, 8'h09, 16'd0 };
        init_rom[138] = { 1'b0, 16'h519d, 8'h2b, 16'd0 };
        init_rom[139] = { 1'b0, 16'h519e, 8'h38, 16'd0 };
        init_rom[140] = { 1'b0, 16'h5381, 8'h1e, 16'd0 };
        init_rom[141] = { 1'b0, 16'h5382, 8'h5b, 16'd0 };
        init_rom[142] = { 1'b0, 16'h5383, 8'h08, 16'd0 };
        init_rom[143] = { 1'b0, 16'h5384, 8'h0a, 16'd0 };
        init_rom[144] = { 1'b0, 16'h5385, 8'h7e, 16'd0 };
        init_rom[145] = { 1'b0, 16'h5386, 8'h88, 16'd0 };
        init_rom[146] = { 1'b0, 16'h5387, 8'h7c, 16'd0 };
        init_rom[147] = { 1'b0, 16'h5388, 8'h6c, 16'd0 };
        init_rom[148] = { 1'b0, 16'h5389, 8'h10, 16'd0 };
        init_rom[149] = { 1'b0, 16'h538a, 8'h01, 16'd0 };
        init_rom[150] = { 1'b0, 16'h538b, 8'h98, 16'd0 };
        init_rom[151] = { 1'b0, 16'h5300, 8'h08, 16'd0 };
        init_rom[152] = { 1'b0, 16'h5301, 8'h30, 16'd0 };
        init_rom[153] = { 1'b0, 16'h5302, 8'h10, 16'd0 };
        init_rom[154] = { 1'b0, 16'h5303, 8'h00, 16'd0 };
        init_rom[155] = { 1'b0, 16'h5304, 8'h08, 16'd0 };
        init_rom[156] = { 1'b0, 16'h5305, 8'h30, 16'd0 };
        init_rom[157] = { 1'b0, 16'h5306, 8'h08, 16'd0 };
        init_rom[158] = { 1'b0, 16'h5307, 8'h16, 16'd0 };
        init_rom[159] = { 1'b0, 16'h5309, 8'h08, 16'd0 };
        init_rom[160] = { 1'b0, 16'h530a, 8'h30, 16'd0 };
        init_rom[161] = { 1'b0, 16'h530b, 8'h04, 16'd0 };
        init_rom[162] = { 1'b0, 16'h530c, 8'h06, 16'd0 };
        init_rom[163] = { 1'b0, 16'h5480, 8'h01, 16'd0 };
        init_rom[164] = { 1'b0, 16'h5481, 8'h08, 16'd0 };
        init_rom[165] = { 1'b0, 16'h5482, 8'h14, 16'd0 };
        init_rom[166] = { 1'b0, 16'h5483, 8'h28, 16'd0 };
        init_rom[167] = { 1'b0, 16'h5484, 8'h51, 16'd0 };
        init_rom[168] = { 1'b0, 16'h5485, 8'h65, 16'd0 };
        init_rom[169] = { 1'b0, 16'h5486, 8'h71, 16'd0 };
        init_rom[170] = { 1'b0, 16'h5487, 8'h7d, 16'd0 };
        init_rom[171] = { 1'b0, 16'h5488, 8'h87, 16'd0 };
        init_rom[172] = { 1'b0, 16'h5489, 8'h91, 16'd0 };
        init_rom[173] = { 1'b0, 16'h548a, 8'h9a, 16'd0 };
        init_rom[174] = { 1'b0, 16'h548b, 8'haa, 16'd0 };
        init_rom[175] = { 1'b0, 16'h548c, 8'hb8, 16'd0 };
        init_rom[176] = { 1'b0, 16'h548d, 8'hcd, 16'd0 };
        init_rom[177] = { 1'b0, 16'h548e, 8'hdd, 16'd0 };
        init_rom[178] = { 1'b0, 16'h548f, 8'hea, 16'd0 };
        init_rom[179] = { 1'b0, 16'h5490, 8'h1d, 16'd0 };
        init_rom[180] = { 1'b0, 16'h5580, 8'h02, 16'd0 };
        init_rom[181] = { 1'b0, 16'h5583, 8'h40, 16'd0 };
        init_rom[182] = { 1'b0, 16'h5584, 8'h10, 16'd0 };
        init_rom[183] = { 1'b0, 16'h5589, 8'h10, 16'd0 };
        init_rom[184] = { 1'b0, 16'h558a, 8'h00, 16'd0 };
        init_rom[185] = { 1'b0, 16'h558b, 8'hf8, 16'd0 };
        init_rom[186] = { 1'b0, 16'h5800, 8'h23, 16'd0 };
        init_rom[187] = { 1'b0, 16'h5801, 8'h14, 16'd0 };
        init_rom[188] = { 1'b0, 16'h5802, 8'h0f, 16'd0 };
        init_rom[189] = { 1'b0, 16'h5803, 8'h0f, 16'd0 };
        init_rom[190] = { 1'b0, 16'h5804, 8'h12, 16'd0 };
        init_rom[191] = { 1'b0, 16'h5805, 8'h26, 16'd0 };
        init_rom[192] = { 1'b0, 16'h5806, 8'h0c, 16'd0 };
        init_rom[193] = { 1'b0, 16'h5807, 8'h08, 16'd0 };
        init_rom[194] = { 1'b0, 16'h5808, 8'h05, 16'd0 };
        init_rom[195] = { 1'b0, 16'h5809, 8'h05, 16'd0 };
        init_rom[196] = { 1'b0, 16'h580a, 8'h08, 16'd0 };
        init_rom[197] = { 1'b0, 16'h580b, 8'h0d, 16'd0 };
        init_rom[198] = { 1'b0, 16'h580c, 8'h08, 16'd0 };
        init_rom[199] = { 1'b0, 16'h580d, 8'h03, 16'd0 };
        init_rom[200] = { 1'b0, 16'h580e, 8'h00, 16'd0 };
        init_rom[201] = { 1'b0, 16'h580f, 8'h00, 16'd0 };
        init_rom[202] = { 1'b0, 16'h5810, 8'h03, 16'd0 };
        init_rom[203] = { 1'b0, 16'h5811, 8'h09, 16'd0 };
        init_rom[204] = { 1'b0, 16'h5812, 8'h07, 16'd0 };
        init_rom[205] = { 1'b0, 16'h5813, 8'h03, 16'd0 };
        init_rom[206] = { 1'b0, 16'h5814, 8'h00, 16'd0 };
        init_rom[207] = { 1'b0, 16'h5815, 8'h01, 16'd0 };
        init_rom[208] = { 1'b0, 16'h5816, 8'h03, 16'd0 };
        init_rom[209] = { 1'b0, 16'h5817, 8'h08, 16'd0 };
        init_rom[210] = { 1'b0, 16'h5818, 8'h0d, 16'd0 };
        init_rom[211] = { 1'b0, 16'h5819, 8'h08, 16'd0 };
        init_rom[212] = { 1'b0, 16'h581a, 8'h05, 16'd0 };
        init_rom[213] = { 1'b0, 16'h581b, 8'h06, 16'd0 };
        init_rom[214] = { 1'b0, 16'h581c, 8'h08, 16'd0 };
        init_rom[215] = { 1'b0, 16'h581d, 8'h0e, 16'd0 };
        init_rom[216] = { 1'b0, 16'h581e, 8'h29, 16'd0 };
        init_rom[217] = { 1'b0, 16'h581f, 8'h17, 16'd0 };
        init_rom[218] = { 1'b0, 16'h5820, 8'h11, 16'd0 };
        init_rom[219] = { 1'b0, 16'h5821, 8'h11, 16'd0 };
        init_rom[220] = { 1'b0, 16'h5822, 8'h15, 16'd0 };
        init_rom[221] = { 1'b0, 16'h5823, 8'h28, 16'd0 };
        init_rom[222] = { 1'b0, 16'h5824, 8'h46, 16'd0 };
        init_rom[223] = { 1'b0, 16'h5825, 8'h26, 16'd0 };
        init_rom[224] = { 1'b0, 16'h5826, 8'h08, 16'd0 };
        init_rom[225] = { 1'b0, 16'h5827, 8'h26, 16'd0 };
        init_rom[226] = { 1'b0, 16'h5828, 8'h64, 16'd0 };
        init_rom[227] = { 1'b0, 16'h5829, 8'h26, 16'd0 };
        init_rom[228] = { 1'b0, 16'h582a, 8'h24, 16'd0 };
        init_rom[229] = { 1'b0, 16'h582b, 8'h22, 16'd0 };
        init_rom[230] = { 1'b0, 16'h582c, 8'h24, 16'd0 };
        init_rom[231] = { 1'b0, 16'h582d, 8'h24, 16'd0 };
        init_rom[232] = { 1'b0, 16'h582e, 8'h06, 16'd0 };
        init_rom[233] = { 1'b0, 16'h582f, 8'h22, 16'd0 };
        init_rom[234] = { 1'b0, 16'h5830, 8'h40, 16'd0 };
        init_rom[235] = { 1'b0, 16'h5831, 8'h42, 16'd0 };
        init_rom[236] = { 1'b0, 16'h5832, 8'h24, 16'd0 };
        init_rom[237] = { 1'b0, 16'h5833, 8'h26, 16'd0 };
        init_rom[238] = { 1'b0, 16'h5834, 8'h24, 16'd0 };
        init_rom[239] = { 1'b0, 16'h5835, 8'h22, 16'd0 };
        init_rom[240] = { 1'b0, 16'h5836, 8'h22, 16'd0 };
        init_rom[241] = { 1'b0, 16'h5837, 8'h26, 16'd0 };
        init_rom[242] = { 1'b0, 16'h5838, 8'h44, 16'd0 };
        init_rom[243] = { 1'b0, 16'h5839, 8'h24, 16'd0 };
        init_rom[244] = { 1'b0, 16'h583a, 8'h26, 16'd0 };
        init_rom[245] = { 1'b0, 16'h583b, 8'h28, 16'd0 };
        init_rom[246] = { 1'b0, 16'h583c, 8'h42, 16'd0 };
        init_rom[247] = { 1'b0, 16'h583d, 8'hce, 16'd0 };
        init_rom[248] = { 1'b0, 16'h5025, 8'h00, 16'd0 };
        init_rom[249] = { 1'b0, 16'h3a0f, 8'h30, 16'd0 };
        init_rom[250] = { 1'b0, 16'h3a10, 8'h28, 16'd0 };
        init_rom[251] = { 1'b0, 16'h3a1b, 8'h30, 16'd0 };
        init_rom[252] = { 1'b0, 16'h3a1e, 8'h26, 16'd0 };
        init_rom[253] = { 1'b0, 16'h3a11, 8'h60, 16'd0 };
        init_rom[254] = { 1'b0, 16'h3a1f, 8'h14, 16'd0 };
        init_rom[255] = { 1'b0, 16'h503d, (TEST_PATTERN_ENABLE ? 8'h80 : 8'h00), 16'd0 };
        init_rom[256] = { 1'b0, 16'h4837, 8'h0a, 16'd0 };  // PCLK period @link384M: sample_rate=(384*mipi2*lanes2*2)/16=192M, 2e9/192M=10.4ns->0x0A (was 0x14 @link192M/mult48)
        init_rom[257] = { 1'b0, 16'h4814, 8'h2a, 16'd0 };
        init_rom[258] = { 1'b0, 16'h4202, 8'h00, 16'd0 };
        init_rom[259] = { 1'b0, 16'h300e, MIPI_CTRL_300E_STREAM_2LANE, 16'd0 };
        init_rom[260] = { 1'b0, 16'h3008, 8'h02, 16'd300 };
    end

    // Combinational lookup replaces the get_step task dispatch chain.
    wire [40:0] rom_entry    = init_rom[step_index];
    wire        rom_is_read  = rom_entry[40];
    wire [15:0] rom_addr     = rom_entry[39:24];
    wire [7:0]  rom_value    = rom_entry[23:16];
    wire [15:0] rom_delay_ms = rom_entry[15:0];

    task automatic load_tx_byte(
        input  op_t current_op,
        input  logic [2:0] index,
        input  logic [15:0] current_addr,
        input  logic [7:0] current_value,
        output logic [7:0] value
    );
        unique case (index)
            3'd0: value = DEV_ADDR_W;
            3'd1: value = current_addr[15:8];
            3'd2: value = current_addr[7:0];
            3'd3: value = (current_op == OP_READ) ? DEV_ADDR_R : current_value;
            default: value = 8'h00;
        endcase
    endtask

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state           <= ST_POWERUP_WAIT;
            tick_count      <= '0;
            delay_count     <= '0;
            delay_target    <= 32'(POWERUP_DELAY_CYCLES);
            scl_drive_low   <= 1'b0;
            sda_drive_low   <= 1'b0;
            busy            <= 1'b1;
            done            <= 1'b0;
            error           <= 1'b0;
            rt_test_pattern_done <= 1'b0;
            rt_test_pattern_error <= 1'b0;
            rt_test_pattern_value <= TEST_PATTERN_ENABLE ? 8'h80 : 8'h00;
            rt_ack_error_count <= 8'h00;
            rt_reg_write_done <= 1'b0;
            rt_reg_write_error <= 1'b0;
            rt_reg_write_ack_err_count <= 8'h00;
            rt_reg_write_last_addr <= 16'h0000;
            rt_reg_read_done <= 1'b0;
            rt_reg_read_error <= 1'b0;
            rt_reg_read_data <= 8'h00;
            rt_reg_read_last_addr <= 16'h0000;
            chip_id_high    <= 8'h00;
            chip_id_low     <= 8'h00;
            ack_error_count <= 8'h00;
            step_index      <= 9'h000;
            rd_mipi_ctrl_300e <= 8'h00;
            rd_mipi_ctrl_4800 <= 8'h00;
            rd_mipi_ctrl_4805 <= 8'h00;
            rd_mipi_ctrl_4837 <= 8'h00;
            rd_format_ctrl_4300 <= 8'h00;
            rd_isp_format_501f <= 8'h00;
            rd_isp_ctrl_5000 <= 8'h00;
            rd_isp_ctrl_5001 <= 8'h00;
            rd_timing_ctrl_3824 <= 8'h00;
            rd_jpeg_ctrl_4407 <= 8'h00;
            rd_mipi_ctrl_440e <= 8'h00;
            rd_vfifo_ctrl_460b <= 8'h00;
            rd_vfifo_ctrl_460c <= 8'h00;
            rd_awb_5189 <= 8'h00;
            rd_output_width_high_3808 <= 8'h00;
            rd_output_width_low_3809 <= 8'h00;
            rd_output_height_high_380a <= 8'h00;
            rd_output_height_low_380b <= 8'h00;
            rd_aec_manual_3503 <= 8'h00;
            rd_aec_ctrl_3a13 <= 8'h00;
            rd_aec_gain_ceiling_high_3a18 <= 8'h00;
            rd_aec_gain_ceiling_low_3a19 <= 8'h00;
            dbg_ack_low_seen <= 1'b0;
            dbg_scl_low_seen <= 1'b0;
            dbg_scl_high_seen <= 1'b0;
            dbg_sda_low_seen <= 1'b0;
            dbg_sda_high_seen <= 1'b0;
            op              <= OP_READ;
            tx_byte         <= 8'h00;
            rx_byte         <= 8'h00;
            bit_index       <= 3'd7;
            byte_index      <= 3'd0;
            reg_addr        <= 16'h300a;
            reg_value       <= 8'h00;
            step_delay_ms   <= 16'd0;
            runtime_active  <= 1'b0;
            runtime_kind    <= 2'b00;
        end else begin
            automatic logic tick_done;
            automatic logic [7:0] next_tx_byte;
            automatic op_t next_op;
            automatic logic [15:0] next_addr;
            automatic logic [7:0] next_value;
            automatic logic [15:0] next_delay_ms;
            automatic logic [7:0] requested_test_pattern_value;

            requested_test_pattern_value = rt_test_pattern_enable ? 8'h80 : 8'h00;

            if (scl_in) begin
                dbg_scl_high_seen <= 1'b1;
            end else begin
                dbg_scl_low_seen <= 1'b1;
            end

            if (sda_in) begin
                dbg_sda_high_seen <= 1'b1;
            end else begin
                dbg_sda_low_seen <= 1'b1;
            end

            tick_done = half_tick_done(tick_count);
            if (tick_done) begin
                tick_count <= '0;
            end else begin
                tick_count <= tick_count + 1'b1;
            end

            if (state == ST_POWERUP_WAIT) begin
                scl_drive_low <= 1'b0;
                sda_drive_low <= 1'b0;
                busy <= 1'b1;
                done <= 1'b0;
                if (delay_count == delay_target - 32'd1) begin
                    delay_count <= 32'd0;
                    tick_count <= '0;
                    state <= ST_LOAD_STEP;
                end else begin
                    delay_count <= delay_count + 32'd1;
                end
            end else if (state == ST_STEP_DELAY) begin
                if (delay_count == delay_target - 32'd1) begin
                    delay_count <= 32'd0;
                    if (step_index == LAST_STEP) begin
                        state <= ST_DONE;
                    end else begin
                        step_index <= step_index + 9'd1;
                        state <= ST_LOAD_STEP;
                    end
                end else begin
                    delay_count <= delay_count + 32'd1;
                end
            end else if (state == ST_LOAD_STEP) begin
                next_op       = rom_is_read ? OP_READ : OP_WRITE;
                next_addr     = rom_addr;
                next_value    = rom_value;
                next_delay_ms = rom_delay_ms;
                op <= next_op;
                reg_addr <= next_addr;
                reg_value <= next_value;
                step_delay_ms <= next_delay_ms;
                byte_index <= 3'd0;
                bit_index <= 3'd7;
                rx_byte <= 8'h00;
                load_tx_byte(next_op, 3'd0, next_addr, next_value, next_tx_byte);
                tx_byte <= next_tx_byte;
                state <= ST_START_A;
            end else if (state == ST_DONE) begin
                scl_drive_low <= 1'b0;
                sda_drive_low <= 1'b0;
                done <= 1'b1;
                if (rt_test_pattern_valid) begin
                    busy <= 1'b1;
                    runtime_active <= 1'b1;
                    runtime_kind <= 2'b00;
                    rt_test_pattern_done <= 1'b0;
                    rt_test_pattern_error <= 1'b0;
                    rt_test_pattern_value <= requested_test_pattern_value;
                    op <= OP_WRITE;
                    reg_addr <= 16'h503d;
                    reg_value <= requested_test_pattern_value;
                    step_delay_ms <= 16'd0;
                    byte_index <= 3'd0;
                    bit_index <= 3'd7;
                    rx_byte <= 8'h00;
                    load_tx_byte(OP_WRITE, 3'd0, 16'h503d, requested_test_pattern_value, next_tx_byte);
                    tx_byte <= next_tx_byte;
                    state <= ST_START_A;
                end else if (rt_reg_write_valid) begin
                    busy <= 1'b1;
                    runtime_active <= 1'b1;
                    runtime_kind <= 2'b01;
                    rt_reg_write_done <= 1'b0;
                    rt_reg_write_error <= 1'b0;
                    rt_reg_write_last_addr <= rt_reg_write_addr;
                    op <= OP_WRITE;
                    reg_addr <= rt_reg_write_addr;
                    reg_value <= rt_reg_write_value;
                    step_delay_ms <= 16'd0;
                    byte_index <= 3'd0;
                    bit_index <= 3'd7;
                    rx_byte <= 8'h00;
                    load_tx_byte(OP_WRITE, 3'd0, rt_reg_write_addr, rt_reg_write_value, next_tx_byte);
                    tx_byte <= next_tx_byte;
                    state <= ST_START_A;
                end else if (rt_reg_read_valid) begin
                    busy <= 1'b1;
                    runtime_active <= 1'b1;
                    runtime_kind <= 2'b10;
                    rt_reg_read_done <= 1'b0;
                    rt_reg_read_error <= 1'b0;
                    rt_reg_read_last_addr <= rt_reg_read_addr;
                    op <= OP_READ;
                    reg_addr <= rt_reg_read_addr;
                    reg_value <= 8'h00;
                    step_delay_ms <= 16'd0;
                    byte_index <= 3'd0;
                    bit_index <= 3'd7;
                    rx_byte <= 8'h00;
                    load_tx_byte(OP_READ, 3'd0, rt_reg_read_addr, 8'h00, next_tx_byte);
                    tx_byte <= next_tx_byte;
                    state <= ST_START_A;
                end else begin
                    busy <= 1'b0;
                    state <= ST_DONE;
                end
            end else if (!tick_done) begin
                state <= state;
            end else begin
                unique case (state)
                    ST_START_A: begin
                        scl_drive_low <= 1'b0;
                        sda_drive_low <= 1'b0;
                        state <= ST_START_B;
                    end
                    ST_START_B: begin
                        scl_drive_low <= 1'b0;
                        sda_drive_low <= 1'b1;
                        state <= ST_START_C;
                    end
                    ST_START_C: begin
                        scl_drive_low <= 1'b1;
                        state <= ST_BYTE_SETUP;
                    end

                    ST_BYTE_SETUP: begin
                        scl_drive_low <= 1'b1;
                        sda_drive_low <= !tx_byte[bit_index];
                        state <= ST_BYTE_HIGH;
                    end
                    ST_BYTE_HIGH: begin
                        scl_drive_low <= 1'b0;
                        state <= ST_BYTE_LOW;
                    end
                    ST_BYTE_LOW: begin
                        scl_drive_low <= 1'b1;
                        if (bit_index == 3'd0) begin
                            state <= ST_ACK_SETUP;
                        end else begin
                            bit_index <= bit_index - 3'd1;
                            state <= ST_BYTE_SETUP;
                        end
                    end

                    ST_ACK_SETUP: begin
                        scl_drive_low <= 1'b1;
                        sda_drive_low <= 1'b0;
                        state <= ST_ACK_HIGH;
                    end
                    ST_ACK_HIGH: begin
                        scl_drive_low <= 1'b0;
                        if (sda_in) begin
                            if (runtime_active) begin
                                if (runtime_kind == 2'b00) begin
                                    rt_test_pattern_error <= 1'b1;
                                    if (rt_ack_error_count != 8'hff) begin
                                        rt_ack_error_count <= rt_ack_error_count + 8'd1;
                                    end
                                end else if (runtime_kind == 2'b01) begin
                                    rt_reg_write_error <= 1'b1;
                                    if (rt_reg_write_ack_err_count != 8'hff) begin
                                        rt_reg_write_ack_err_count <= rt_reg_write_ack_err_count + 8'd1;
                                    end
                                end else begin
                                    rt_reg_read_error <= 1'b1;
                                end
                            end else begin
                                error <= 1'b1;
                                if (ack_error_count != 8'hff) begin
                                    ack_error_count <= ack_error_count + 8'd1;
                                end
                            end
                        end else begin
                            dbg_ack_low_seen <= 1'b1;
                        end
                        state <= ST_ACK_LOW;
                    end
                    ST_ACK_LOW: begin
                        scl_drive_low <= 1'b1;
                        state <= ST_NEXT;
                    end

                    ST_NEXT: begin
                        if ((op == OP_WRITE && byte_index == 3'd3) ||
                            (op == OP_READ && byte_index == 3'd3)) begin
                            if (op == OP_READ) begin
                                bit_index <= 3'd7;
                                rx_byte <= 8'h00;
                                state <= ST_READ_SETUP;
                            end else begin
                                state <= ST_STOP_A;
                            end
                        end else if (op == OP_READ && byte_index == 3'd2) begin
                            byte_index <= 3'd3;
                            load_tx_byte(op, 3'd3, reg_addr, reg_value, next_tx_byte);
                            tx_byte <= next_tx_byte;
                            bit_index <= 3'd7;
                            state <= ST_START_A;
                        end else begin
                            byte_index <= byte_index + 3'd1;
                            load_tx_byte(op, byte_index + 3'd1, reg_addr, reg_value, next_tx_byte);
                            tx_byte <= next_tx_byte;
                            bit_index <= 3'd7;
                            state <= ST_BYTE_SETUP;
                        end
                    end

                    ST_READ_SETUP: begin
                        scl_drive_low <= 1'b1;
                        sda_drive_low <= 1'b0;
                        state <= ST_READ_HIGH;
                    end
                    ST_READ_HIGH: begin
                        scl_drive_low <= 1'b0;
                        rx_byte[bit_index] <= sda_in;
                        state <= ST_READ_LOW;
                    end
                    ST_READ_LOW: begin
                        scl_drive_low <= 1'b1;
                        if (bit_index == 3'd0) begin
                            state <= ST_MASTER_NACK_SETUP;
                        end else begin
                            bit_index <= bit_index - 3'd1;
                            state <= ST_READ_SETUP;
                        end
                    end

                    ST_MASTER_NACK_SETUP: begin
                        scl_drive_low <= 1'b1;
                        sda_drive_low <= 1'b0;
                        state <= ST_MASTER_NACK_HIGH;
                    end
                    ST_MASTER_NACK_HIGH: begin
                        scl_drive_low <= 1'b0;
                        state <= ST_MASTER_NACK_LOW;
                    end
                    ST_MASTER_NACK_LOW: begin
                        scl_drive_low <= 1'b1;
                        if (runtime_active && runtime_kind == 2'b10) begin
                            rt_reg_read_data <= rx_byte;
                        end else if (reg_addr == 16'h300a) begin
                            chip_id_high <= rx_byte;
                        end else if (reg_addr == 16'h300b) begin
                            chip_id_low <= rx_byte;
                        end else if (reg_addr == 16'h300e) begin
                            rd_mipi_ctrl_300e <= rx_byte;
                        end else if (reg_addr == 16'h4800) begin
                            rd_mipi_ctrl_4800 <= rx_byte;
                        end else if (reg_addr == 16'h4805) begin
                            rd_mipi_ctrl_4805 <= rx_byte;
                        end else if (reg_addr == 16'h4837) begin
                            rd_mipi_ctrl_4837 <= rx_byte;
                        end else if (reg_addr == 16'h4300) begin
                            rd_format_ctrl_4300 <= rx_byte;
                        end else if (reg_addr == 16'h501f) begin
                            rd_isp_format_501f <= rx_byte;
                        end else if (reg_addr == 16'h5000) begin
                            rd_isp_ctrl_5000 <= rx_byte;
                        end else if (reg_addr == 16'h5001) begin
                            rd_isp_ctrl_5001 <= rx_byte;
                        end else if (reg_addr == 16'h3824) begin
                            rd_timing_ctrl_3824 <= rx_byte;
                        end else if (reg_addr == 16'h4407) begin
                            rd_jpeg_ctrl_4407 <= rx_byte;
                        end else if (reg_addr == 16'h440e) begin
                            rd_mipi_ctrl_440e <= rx_byte;
                        end else if (reg_addr == 16'h460b) begin
                            rd_vfifo_ctrl_460b <= rx_byte;
                        end else if (reg_addr == 16'h460c) begin
                            rd_vfifo_ctrl_460c <= rx_byte;
                        end else if (reg_addr == 16'h5189) begin
                            rd_awb_5189 <= rx_byte;
                        end else if (reg_addr == 16'h3808) begin
                            rd_output_width_high_3808 <= rx_byte;
                        end else if (reg_addr == 16'h3809) begin
                            rd_output_width_low_3809 <= rx_byte;
                        end else if (reg_addr == 16'h380a) begin
                            rd_output_height_high_380a <= rx_byte;
                        end else if (reg_addr == 16'h380b) begin
                            rd_output_height_low_380b <= rx_byte;
                        end else if (reg_addr == 16'h3503) begin
                            rd_aec_manual_3503 <= rx_byte;
                        end else if (reg_addr == 16'h3a13) begin
                            rd_aec_ctrl_3a13 <= rx_byte;
                        end else if (reg_addr == 16'h3a18) begin
                            rd_aec_gain_ceiling_high_3a18 <= rx_byte;
                        end else if (reg_addr == 16'h3a19) begin
                            rd_aec_gain_ceiling_low_3a19 <= rx_byte;
                        end
                        state <= ST_STOP_A;
                    end

                    ST_STOP_A: begin
                        scl_drive_low <= 1'b1;
                        sda_drive_low <= 1'b1;
                        state <= ST_STOP_B;
                    end
                    ST_STOP_B: begin
                        scl_drive_low <= 1'b0;
                        sda_drive_low <= 1'b0;
                        if (runtime_active) begin
                            runtime_active <= 1'b0;
                            busy <= 1'b0;
                            if (runtime_kind == 2'b00) begin
                                rt_test_pattern_done <= 1'b1;
                            end else if (runtime_kind == 2'b01) begin
                                rt_reg_write_done <= 1'b1;
                            end else begin
                                rt_reg_read_done <= 1'b1;
                            end
                            state <= ST_DONE;
                        end else if (step_delay_ms != 16'd0) begin
                            delay_target <= 32'(step_delay_ms) * (CLK_HZ / 1000);
                            delay_count <= 32'd0;
                            state <= ST_STEP_DELAY;
                        end else if (step_index == LAST_STEP) begin
                            state <= ST_DONE;
                        end else begin
                            step_index <= step_index + 9'd1;
                            state <= ST_LOAD_STEP;
                        end
                    end

                    ST_DONE: begin
                        scl_drive_low <= 1'b0;
                        sda_drive_low <= 1'b0;
                        done <= 1'b1;
                        busy <= 1'b0;
                        state <= ST_DONE;
                    end

                    default: begin
                        error <= 1'b1;
                        state <= ST_DONE;
                    end
                endcase
            end
        end
    end

endmodule

`default_nettype wire