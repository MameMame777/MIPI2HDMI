`timescale 1ns / 1ps
`default_nettype none

// CSI-2 test pattern generator.
//
// Produces syntactically valid MIPI CSI-2 frames with correct per-packet ECC
// headers and per-line CRC-16 footers.  Feeds directly into the s_byte_*
// interface of csi2_packet_parser (IN_WIDTH=16, 2 bytes per beat).
//
// Frame layout (LSLE_EN=0, default):
//   FS  →  (long-pkt × V_LINES)  →  FE  →  gap × FRAME_GAP_CLOCKS
//
// Payload patterns selected by pattern_sel[1:0] (AXI-GPIO runtime, no rebuild):
//   2'b00  Vertical ramp    — gray8 = row × 255/479, same across all columns
//   2'b01  Horizontal ramp  — gray8 = col × 255/511, saturates at 255 for col 512+
//   2'b10  Checkerboard     — 32×32-pixel white/black cells (row[5] XOR col[5])
//   2'b11  Diagonal ramp    — gray8 = (row[7:0] + col[8:1]) mod 256
//
// RGB565 little-endian encoding (same gray8 in R=G=B → luma ≈ gray8 after pipeline):
//   byte0 = {G[2:0], B[4:0]} = {gray8[4:2], gray8[7:3]}
//   byte1 = {R[4:0], G[5:3]} = {gray8[7:3], gray8[7:5]}
//
// Timing: pb0_r/pb1_r are registered one state before ST_PAY (or per-pixel inside
// ST_PAY) to break any DSP48→CRC critical path.
//
// Diagnostic knob: FRAME_GAP_CLOCKS=0 → frames back-to-back.
//                  FRAME_GAP_CLOCKS=1_000_000 → ~16 ms gap @ 125 MHz ≈ 47 fps.
module csi2_tpg #(
    parameter int         H_PIXELS         = 640,
    parameter int         V_LINES          = 480,
    parameter logic [5:0] DT               = 6'h22,      // 0x22 = RGB565
    parameter logic [1:0] VC               = 2'h0,
    parameter bit         LSLE_EN          = 1'b0,
    parameter int         FRAME_GAP_CLOCKS = 1_000_000,
    // OUTPUT_INTERVAL: insert (OUTPUT_INTERVAL-1) idle cycles between every valid beat.
    // Set to 2 to match CDC CORE_OUTPUT_INTERVAL=2 so the downstream parser FIFO
    // (depth=16, designed for camera 50% duty cycle) never overflows.
    parameter int         OUTPUT_INTERVAL  = 2
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire [1:0]   pattern_sel,   // runtime pattern select (AXI GPIO bits[28:27])

    output logic [15:0] m_byte_data,
    output logic [1:0]  m_byte_keep,
    output logic        m_byte_valid,
    output logic        m_byte_sop,
    output logic        m_byte_eop
);

    // DI bytes for each packet type
    localparam logic [7:0] DI_LONG = {VC, DT};
    localparam logic [7:0] DI_FS   = {VC, 6'h00};
    localparam logic [7:0] DI_FE   = {VC, 6'h01};
    localparam logic [7:0] DI_LS   = {VC, 6'h02};
    localparam logic [7:0] DI_LE   = {VC, 6'h03};

    // Long packet word count: H_PIXELS × 2 bytes (RGB565)
    localparam logic [15:0] LONG_WC = 16'(H_PIXELS * 2);

    typedef enum logic [3:0] {
        ST_IDLE,
        ST_FS_0, ST_FS_1,
        ST_LS_0, ST_LS_1,
        ST_LHDR_0, ST_LHDR_1, ST_PAY, ST_CRC,
        ST_LE_0, ST_LE_1,
        ST_NEXT,
        ST_FE_0, ST_FE_1,
        ST_GAP
    } state_t;

    state_t      state;
    logic [15:0] line_idx;      // current line (0..V_LINES-1)
    logic [15:0] frame_ctr;     // frame counter — used as FS/FE WC field
    logic [15:0] pay_ctr;       // payload beat counter (0..H_PIXELS-1)
    logic [31:0] gap_ctr;       // inter-frame gap counter
    logic [15:0] crc_reg;       // running CRC for current long packet

    // Throttle: free-running counter, out_en pulses once every OUTPUT_INTERVAL clocks.
    logic [7:0] out_cnt;
    wire        out_en = (OUTPUT_INTERVAL <= 1) ? 1'b1 : (out_cnt == 8'b0);

    // ---------------------------------------------------------------------------
    // CSI-2 header ECC (Hamming, 6 bits).
    // ---------------------------------------------------------------------------
    function automatic [5:0] calc_ecc6(input logic [23:0] d);
        calc_ecc6[0] = d[0]^d[1]^d[2]^d[4]^d[5]^d[7]^d[10]^d[11]^d[13]^d[16]^d[20]^d[21]^d[22]^d[23];
        calc_ecc6[1] = d[0]^d[1]^d[3]^d[4]^d[6]^d[8]^d[10]^d[12]^d[14]^d[17]^d[20]^d[21]^d[22]^d[23];
        calc_ecc6[2] = d[0]^d[2]^d[3]^d[5]^d[6]^d[9]^d[11]^d[12]^d[15]^d[18]^d[20]^d[21]^d[22];
        calc_ecc6[3] = d[1]^d[2]^d[3]^d[7]^d[8]^d[9]^d[13]^d[14]^d[15]^d[19]^d[20]^d[21]^d[23];
        calc_ecc6[4] = d[4]^d[5]^d[6]^d[7]^d[8]^d[9]^d[16]^d[17]^d[18]^d[19]^d[20]^d[22]^d[23];
        calc_ecc6[5] = d[10]^d[11]^d[12]^d[13]^d[14]^d[15]^d[16]^d[17]^d[18]^d[19]^d[21]^d[22]^d[23];
    endfunction

    function automatic [7:0] hdr_ecc(input logic [7:0] di, input logic [15:0] wc);
        return {2'b00, calc_ecc6({wc[15:8], wc[7:0], di})};
    endfunction

    // ---------------------------------------------------------------------------
    // CRC-16 (reflected poly 0x8408, init 0xFFFF).
    // ---------------------------------------------------------------------------
    function automatic [15:0] crc_byte(input logic [15:0] c, input logic [7:0] d);
        for (int i = 0; i < 8; i++) begin
            automatic logic fb;
            fb = c[0] ^ d[i];
            c  = c >> 1;
            if (fb) c = c ^ 16'h8408;
        end
        return c;
    endfunction

    function automatic [15:0] crc_beat(
        input logic [15:0] c,
        input logic [7:0]  b0, b1
    );
        return crc_byte(crc_byte(c, b0), b1);
    endfunction

    // ---------------------------------------------------------------------------
    // Pixel gray8 computation
    //
    // Pattern 0 (VERT): DSP multiply, computed once per line.
    //   gray8_vert_w — combinatorial DSP output (line_idx → DSP)
    //   gray8_vert_r — registered in ST_LHDR_0 to break DSP→CRC timing path
    //
    // Patterns 1–3: combinatorial fast paths (shifts / XOR), no DSP.
    //   Computed per-pixel via gray8_fn; registered into pb0_r/pb1_r in ST_PAY.
    // ---------------------------------------------------------------------------
    localparam int GRAY_SHIFT = 9;
    localparam int GRAY_SCALE = (255 * (1 << GRAY_SHIFT) + (V_LINES - 1) / 2) / (V_LINES - 1);

    wire [7:0] gray8_vert_w = 8'((int'(line_idx) * GRAY_SCALE) >> GRAY_SHIFT);
    logic [7:0] gray8_vert_r;   // registered each LHDR_0

    // Per-pixel gray8 mux: all cases except pattern 0 use fast logic (no DSP).
    // vert  : pre-registered vertical ramp value (stable across the whole line)
    // lidx  : line_idx register (stable during payload)
    // sel   : pattern_sel AXI input
    // pc    : pay_ctr value for the TARGET pixel (current or next)
    function automatic [7:0] gray8_fn(
        input logic [7:0]  vert,
        input logic [15:0] lidx,
        input logic [1:0]  sel,
        input logic [15:0] pc
    );
        case (sel)
            2'b00: gray8_fn = vert;                                 // vertical ramp
            2'b01: gray8_fn = pc[9] ? 8'hFF : 8'(pc[8:1]);        // horiz: 0→255 over 512 cols
            2'b10: gray8_fn = (lidx[5] ^ pc[5]) ? 8'hFF : 8'h00;  // checkerboard 32×32 px
            default: gray8_fn = 8'(lidx[7:0] + pc[8:1]);           // diagonal (row+col/2) mod256
        endcase
    endfunction

    // Pixel-0 gray8: computed in LHDR_0 to initialise pb0_r/pb1_r for the first beat.
    // Uses gray8_vert_w (combinatorial DSP) so that pattern 0 timing path is
    // DSP→pb0_r FF rather than DSP→CARRY4→CRC→crc_reg.
    wire [7:0] gray8_p0   = gray8_fn(gray8_vert_w, line_idx, pattern_sel, 16'd0);

    // Next-pixel gray8: used inside ST_PAY to pre-register pb0_r/pb1_r for the next beat.
    // Uses gray8_vert_r (registered stable value) → mux → FF; no DSP in this path.
    wire [7:0] gray8_next = gray8_fn(gray8_vert_r, line_idx, pattern_sel, pay_ctr + 16'd1);

    // pb0/pb1 registered: always consumed one cycle after being written.
    logic [7:0] pb0_r, pb1_r;

    // Helper: pack gray8 into RGB565 bytes
    function automatic [7:0] pack_pb0(input logic [7:0] g); return {g[4:2], g[7:3]}; endfunction
    function automatic [7:0] pack_pb1(input logic [7:0] g); return {g[7:3], g[7:5]}; endfunction

    // ---------------------------------------------------------------------------
    // Main state machine
    // ---------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            line_idx     <= '0;
            frame_ctr    <= '0;
            pay_ctr      <= '0;
            gap_ctr      <= '0;
            crc_reg      <= 16'hFFFF;
            gray8_vert_r <= '0;
            pb0_r        <= '0;
            pb1_r        <= '0;
            out_cnt      <= '0;
            m_byte_data  <= '0;
            m_byte_keep  <= '0;
            m_byte_valid <= 1'b0;
            m_byte_sop   <= 1'b0;
            m_byte_eop   <= 1'b0;
        end else begin
            // Throttle counter: always advances every clock
            if (OUTPUT_INTERVAL > 1) begin
                if (out_cnt == 8'(OUTPUT_INTERVAL - 1)) out_cnt <= '0;
                else                                     out_cnt <= out_cnt + 8'd1;
            end

            // Default: no output this cycle.
            m_byte_valid <= 1'b0;
            m_byte_sop   <= 1'b0;
            m_byte_eop   <= 1'b0;
            m_byte_keep  <= 2'b11;

            if (out_en) unique case (state)

                // -----------------------------------------------------------------
                ST_IDLE: begin
                    line_idx <= '0;
                    state    <= ST_FS_0;
                end

                // -----------------------------------------------------------------
                // Frame Start short packet
                ST_FS_0: begin
                    m_byte_data  <= {frame_ctr[7:0], DI_FS};
                    m_byte_valid <= 1'b1;
                    m_byte_sop   <= 1'b1;
                    state        <= ST_FS_1;
                end
                ST_FS_1: begin
                    m_byte_data  <= {hdr_ecc(DI_FS, frame_ctr), frame_ctr[15:8]};
                    m_byte_valid <= 1'b1;
                    m_byte_eop   <= 1'b1;
                    state        <= LSLE_EN ? ST_LS_0 : ST_LHDR_0;
                end

                // -----------------------------------------------------------------
                // Line Start short packet (LSLE_EN only)
                ST_LS_0: begin
                    m_byte_data  <= {line_idx[7:0], DI_LS};
                    m_byte_valid <= 1'b1;
                    m_byte_sop   <= 1'b1;
                    state        <= ST_LS_1;
                end
                ST_LS_1: begin
                    m_byte_data  <= {hdr_ecc(DI_LS, line_idx), line_idx[15:8]};
                    m_byte_valid <= 1'b1;
                    m_byte_eop   <= 1'b1;
                    state        <= ST_LHDR_0;
                end

                // -----------------------------------------------------------------
                // Long packet header
                ST_LHDR_0: begin
                    m_byte_data  <= {LONG_WC[7:0], DI_LONG};
                    m_byte_valid <= 1'b1;
                    m_byte_sop   <= 1'b1;
                    crc_reg      <= 16'hFFFF;
                    pay_ctr      <= '0;
                    // Register per-line vertical ramp (DSP output) — breaks DSP→CRC path.
                    gray8_vert_r <= gray8_vert_w;
                    // Initialise pb0/pb1 for pixel 0 of this line.
                    // Pattern 0: uses gray8_vert_w (DSP→FF, timing-clean).
                    // Patterns 1-3: use simple combinatorial logic (fast).
                    pb0_r        <= pack_pb0(gray8_p0);
                    pb1_r        <= pack_pb1(gray8_p0);
                    state        <= ST_LHDR_1;
                end
                ST_LHDR_1: begin
                    m_byte_data  <= {hdr_ecc(DI_LONG, LONG_WC), LONG_WC[15:8]};
                    m_byte_valid <= 1'b1;
                    state        <= ST_PAY;
                end

                // -----------------------------------------------------------------
                // Long packet payload (H_PIXELS beats)
                // pb0_r/pb1_r hold the value for the CURRENT pixel (pre-registered).
                // CRC path: pb0_r(FF) → CRC LUTs → crc_reg(FF)  [no DSP].
                // pb0_r update: gray8_vert_r(FF) or fast logic → mux → pb0_r(FF).
                ST_PAY: begin
                    m_byte_data  <= {pb1_r, pb0_r};
                    m_byte_valid <= 1'b1;
                    crc_reg      <= crc_beat(crc_reg, pb0_r, pb1_r);
                    // Pre-register gray8 for the NEXT pixel.
                    // pay_ctr+1 is evaluated before pay_ctr increments (nonblocking).
                    pb0_r        <= pack_pb0(gray8_next);
                    pb1_r        <= pack_pb1(gray8_next);
                    if (pay_ctr == 16'(H_PIXELS - 1)) begin
                        pay_ctr <= '0;
                        state   <= ST_CRC;
                    end else begin
                        pay_ctr <= pay_ctr + 16'd1;
                    end
                end

                // -----------------------------------------------------------------
                // CRC footer
                ST_CRC: begin
                    m_byte_data  <= {crc_reg[15:8], crc_reg[7:0]};
                    m_byte_valid <= 1'b1;
                    m_byte_eop   <= 1'b1;
                    state        <= LSLE_EN ? ST_LE_0 : ST_NEXT;
                end

                // -----------------------------------------------------------------
                // Line End short packet (LSLE_EN only)
                ST_LE_0: begin
                    m_byte_data  <= {line_idx[7:0], DI_LE};
                    m_byte_valid <= 1'b1;
                    m_byte_sop   <= 1'b1;
                    state        <= ST_LE_1;
                end
                ST_LE_1: begin
                    m_byte_data  <= {hdr_ecc(DI_LE, line_idx), line_idx[15:8]};
                    m_byte_valid <= 1'b1;
                    m_byte_eop   <= 1'b1;
                    state        <= ST_NEXT;
                end

                // -----------------------------------------------------------------
                ST_NEXT: begin
                    if (line_idx == 16'(V_LINES - 1)) begin
                        state <= ST_FE_0;
                    end else begin
                        line_idx <= line_idx + 16'd1;
                        state    <= LSLE_EN ? ST_LS_0 : ST_LHDR_0;
                    end
                end

                // -----------------------------------------------------------------
                // Frame End short packet
                ST_FE_0: begin
                    m_byte_data  <= {frame_ctr[7:0], DI_FE};
                    m_byte_valid <= 1'b1;
                    m_byte_sop   <= 1'b1;
                    state        <= ST_FE_1;
                end
                ST_FE_1: begin
                    m_byte_data  <= {hdr_ecc(DI_FE, frame_ctr), frame_ctr[15:8]};
                    m_byte_valid <= 1'b1;
                    m_byte_eop   <= 1'b1;
                    frame_ctr    <= frame_ctr + 16'd1;
                    gap_ctr      <= '0;
                    state        <= (FRAME_GAP_CLOCKS > 0) ? ST_GAP : ST_IDLE;
                end

                // -----------------------------------------------------------------
                ST_GAP: begin
                    if (gap_ctr == 32'(FRAME_GAP_CLOCKS - 1)) begin
                        state <= ST_IDLE;
                    end else begin
                        gap_ctr <= gap_ctr + 32'd1;
                    end
                end

                default: state <= ST_IDLE;

            endcase
        end
    end

endmodule

`default_nettype wire
