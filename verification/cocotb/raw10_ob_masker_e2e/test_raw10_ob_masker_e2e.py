"""cocotb port of verification/tb/tb_raw10_ob_masker_e2e.sv (RAW10 E2E pipeline).

E2E: RAW10 MIPI byte-beats -> csi2_packet_parser -> csi2_header_ecc -> csi2_payload_crc ->
csi2_vcdt_filter -> csi2_frame_state -> raw10_unpack -> ob_row_masker (WIDTH=10) ->
captured 10-bit pixel stream, verified pixel-by-pixel.

The DSim TB is itself the toplevel: it wires 7 DUT instances together and drives the
parser's byte-beat input (``s_byte_*``) with hand-packed CSI-2 packets (FS short, 4 RAW10
long packets, FE short). cocotb needs a single HDL toplevel, so the 7-DUT wiring is emitted
as a tiny wrapper (``raw10_ob_e2e_harness``, written at build time) containing ONLY the DUT
instances (no ``initial``, no clock) with parameters/port maps 1:1 with the DSim TB. cocotb
owns the clock/reset and stimulus; the SV ``always_ff`` capture of ``ob_pixel`` and the ECC
helper functions (``calc_ecc6``/``make_ecc``/``crc_update_byte``) become Python.

Stimulus + checks are replicated 1:1:
  * L0 uniform OB Y=144      -> MASK to 512
  * L1 checkerboard 40/960   -> pass-through
  * L2 gradient (10-bit)     -> pass-through
  * L3 OB variation 140/148  -> MASK (range=8 <= 12)
Plus the parser/crc status-counter checks the TB asserts before the pixel compare.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import start_clock  # noqa: E402
from lib.scoreboard import check  # noqa: E402

# ---- TB localparams (1:1) ------------------------------------------------------------
PARSER_IN_WIDTH = 16
DT_FS = 0x00
DT_FE = 0x01
DT_RAW10 = 0x2B

LINE_PIXELS = 8
FRAME_LINES = 4
LINE_BYTES = (LINE_PIXELS * 10) // 8   # packed: 5 bytes per 4 pix = 10
FRAME_PIXELS = LINE_PIXELS * FRAME_LINES  # 32

INPUT_Y = [
    # L0: OB uniform Y=144
    144, 144, 144, 144, 144, 144, 144, 144,
    # L1: checkerboard 40/960
    40, 40, 40, 40, 960, 960, 960, 960,
    # L2: gradient
    0, 128, 256, 384, 512, 640, 768, 1020,
    # L3: OB with variation
    140, 148, 140, 148, 140, 148, 140, 148,
]

EXPECTED_Y = [
    # L0: masked -> 512
    512, 512, 512, 512, 512, 512, 512, 512,
    # L1: pass-through
    40, 40, 40, 40, 960, 960, 960, 960,
    # L2: pass-through
    0, 128, 256, 384, 512, 640, 768, 1020,
    # L3: masked -> 512
    512, 512, 512, 512, 512, 512, 512, 512,
]


# ---- ECC / CRC helpers (ports of the SV automatic functions) -------------------------

def _calc_ecc6(data: int) -> int:
    """Port of tb calc_ecc6(24-bit data) -> 6-bit ECC."""
    def bit(i: int) -> int:
        return (data >> i) & 1

    e = [0] * 6
    e[0] = bit(0) ^ bit(1) ^ bit(2) ^ bit(4) ^ bit(5) ^ bit(7) ^ bit(10) ^ bit(11) ^ bit(13) ^ bit(16) ^ bit(20) ^ bit(21) ^ bit(22) ^ bit(23)
    e[1] = bit(0) ^ bit(1) ^ bit(3) ^ bit(4) ^ bit(6) ^ bit(8) ^ bit(10) ^ bit(12) ^ bit(14) ^ bit(17) ^ bit(20) ^ bit(21) ^ bit(22) ^ bit(23)
    e[2] = bit(0) ^ bit(2) ^ bit(3) ^ bit(5) ^ bit(6) ^ bit(9) ^ bit(11) ^ bit(12) ^ bit(15) ^ bit(18) ^ bit(20) ^ bit(21) ^ bit(22)
    e[3] = bit(1) ^ bit(2) ^ bit(3) ^ bit(7) ^ bit(8) ^ bit(9) ^ bit(13) ^ bit(14) ^ bit(15) ^ bit(19) ^ bit(20) ^ bit(21) ^ bit(23)
    e[4] = bit(4) ^ bit(5) ^ bit(6) ^ bit(7) ^ bit(8) ^ bit(9) ^ bit(16) ^ bit(17) ^ bit(18) ^ bit(19) ^ bit(20) ^ bit(22) ^ bit(23)
    e[5] = bit(10) ^ bit(11) ^ bit(12) ^ bit(13) ^ bit(14) ^ bit(15) ^ bit(16) ^ bit(17) ^ bit(18) ^ bit(19) ^ bit(21) ^ bit(22) ^ bit(23)
    return e[0] | (e[1] << 1) | (e[2] << 2) | (e[3] << 3) | (e[4] << 4) | (e[5] << 5)


def _make_ecc(di: int, wc: int) -> int:
    """Port of tb make_ecc(di, wc) = {2'b00, calc_ecc6({wc, di})}."""
    data = ((wc & 0xFFFF) << 8) | (di & 0xFF)   # {wc[15:0], di[7:0]} = 24 bits
    return _calc_ecc6(data) & 0x3F


def _crc_update_byte(crc_in: int, data: int) -> int:
    """Port of tb crc_update_byte (CRC-16, poly 0x8408, reflected)."""
    crc = crc_in & 0xFFFF
    for b in range(8):
        fb = (crc & 1) ^ ((data >> b) & 1)
        crc >>= 1
        if fb:
            crc ^= 0x8408
    return crc & 0xFFFF


# ---- byte-beat driver (ports of the SV drive_* tasks) --------------------------------
#
# The SV tasks assert on @(negedge core_clk) and hold the value until the next drive; the
# beat therefore spans one full core clock. Here we drive on RisingEdge and hold for one
# cycle, then the *next* beat overwrites it -- functionally the parser (which samples on
# posedge) sees the identical one-cycle-per-beat cadence the DSim TB presented (drive_beat
# does not deassert valid between consecutive beats, so back-to-back beats are contiguous,
# and drive_idle explicitly deasserts).

class ByteDrv:
    """Faithful port of the SV drive_* tasks.

    The SV tasks change signals on ``@(negedge core_clk)`` with NBA, so each ``drive_beat``
    holds ``valid`` high for exactly one full clock (sampled once at the following posedge)
    and back-to-back beats form a contiguous burst; ``drive_idle`` then deasserts on the
    next negedge. Here every driver method first ``await RisingEdge`` (advancing one full
    clock so the previously-driven beat is sampled) and *then* changes the signals -- this
    reproduces the one-beat-per-clock cadence exactly. A beat driven by ``drive_beat`` is
    held until the next method's ``await RisingEdge``, so its valid pulse is a full cycle
    wide (the earlier bug was deasserting in the same delta, giving a zero-width pulse).
    """

    def __init__(self, dut, clk):
        self.dut = dut
        self.clk = clk

    def _idle_assign(self):
        self.dut.s_byte_valid.value = 0
        self.dut.s_byte_keep.value = 0
        self.dut.s_byte_sop.value = 0
        self.dut.s_byte_eop.value = 0
        self.dut.s_byte_data.value = 0

    async def idle_now(self):
        """Initial idle at reset (no preceding beat to preserve)."""
        self._idle_assign()

    async def drive_idle(self, n: int):
        # Advance one clock so the preceding beat is sampled, THEN deassert (mirrors the
        # SV drive_idle @(negedge) deassert), then hold idle for n clocks.
        await RisingEdge(self.clk)
        self._idle_assign()
        for _ in range(n):
            await RisingEdge(self.clk)

    async def drive_beat(self, b0: int, b1: int, sop: int, eop: int):
        await RisingEdge(self.clk)
        self.dut.s_byte_data.value = ((b1 & 0xFF) << 8) | (b0 & 0xFF)
        self.dut.s_byte_keep.value = 0b11
        self.dut.s_byte_valid.value = 1
        self.dut.s_byte_sop.value = sop
        self.dut.s_byte_eop.value = eop

    async def send_short_packet(self, dt: int, data_field: int):
        di = dt & 0x3F              # {2'b00, dt}
        ecc = _make_ecc(di, data_field)
        await self.drive_beat(di, data_field & 0xFF, 1, 0)
        await self.drive_beat((data_field >> 8) & 0xFF, ecc, 0, 1)
        await self.drive_idle(2)

    async def send_raw10_line(self, line_idx: int):
        di = DT_RAW10 & 0x3F
        wc = LINE_BYTES & 0xFFFF
        ecc = _make_ecc(di, wc)
        base = line_idx * LINE_PIXELS

        # Pack 4 pixels per 5 bytes
        payload = [0] * LINE_BYTES
        for g in range(LINE_PIXELS // 4):
            p0 = INPUT_Y[base + g * 4 + 0] & 0x3FF
            p1 = INPUT_Y[base + g * 4 + 1] & 0x3FF
            p2 = INPUT_Y[base + g * 4 + 2] & 0x3FF
            p3 = INPUT_Y[base + g * 4 + 3] & 0x3FF
            payload[g * 5 + 0] = (p0 >> 2) & 0xFF
            payload[g * 5 + 1] = (p1 >> 2) & 0xFF
            payload[g * 5 + 2] = (p2 >> 2) & 0xFF
            payload[g * 5 + 3] = (p3 >> 2) & 0xFF
            payload[g * 5 + 4] = ((p3 & 0x3) << 6) | ((p2 & 0x3) << 4) | ((p1 & 0x3) << 2) | (p0 & 0x3)

        crc = 0xFFFF
        for byte in payload:
            crc = _crc_update_byte(crc, byte)

        await self.drive_beat(di, wc & 0xFF, 1, 0)
        await self.drive_beat((wc >> 8) & 0xFF, ecc, 0, 0)
        i = 0
        while i < LINE_BYTES:
            b1 = payload[i + 1] if (i + 1 < LINE_BYTES) else 0x00
            await self.drive_beat(payload[i], b1, 0, 0)
            i += 2
        await self.drive_beat(crc & 0xFF, (crc >> 8) & 0xFF, 0, 1)
        await self.drive_idle(8)


# ---- OB output capture (port of the SV always_ff push_back) ---------------------------

class ObCapture:
    def __init__(self, dut, clk):
        self.dut = dut
        self.clk = clk
        self.pixels = []

    def start(self):
        return cocotb.start_soon(self._run())

    async def _run(self):
        while True:
            await RisingEdge(self.clk)
            if int(self.dut.ob_valid.value) == 1:
                self.pixels.append(int(self.dut.ob_pixel.value))


async def _wait_frame_done(dut, clk):
    """Port of wait_frame_done: fatal if frame_count never reaches 1 in 8000 cycles."""
    for _ in range(8000):
        await RisingEdge(clk)
        if int(dut.frame_count.value) == 1:
            return
    check(False, "frame timeout (frame_count never reached 1)")


async def _wait_parser_short(dut, clk, n: int):
    for _ in range(8000):
        await RisingEdge(clk)
        if int(dut.parser_short_count.value) >= n:
            return
    check(False, "short pkt timeout (parser_short_count never reached n)")


@cocotb.test(timeout_time=20, timeout_unit="ms")
async def raw10_ob_masker_e2e(dut):
    clk = dut.core_clk
    start_clock(clk, 10.0)   # #5 half-period -> 10 ns period, 100 MHz

    drv = ByteDrv(dut, clk)

    # reset_dut()
    dut.core_aresetn.value = 0
    await drv.idle_now()
    for _ in range(8):
        await RisingEdge(clk)
    dut.core_aresetn.value = 1
    for _ in range(4):
        await RisingEdge(clk)

    cap = ObCapture(dut, clk)
    cap.start()

    # --- stimulus (the SV initial block) ---
    await drv.send_short_packet(DT_FS, 0x0000)
    for i in range(FRAME_LINES):
        await drv.send_raw10_line(i)
    await drv.send_short_packet(DT_FE, 0x0000)

    await _wait_frame_done(dut, clk)
    await _wait_parser_short(dut, clk, 2)

    # --- status-counter checks (1:1 with the SV $fatal guards) ---
    check(int(dut.parser_short_count.value) >= 2, "no FE seen (parser_short_count < 2)")
    check(int(dut.parser_long_count.value) == FRAME_LINES,
          f"long count wrong: got {int(dut.parser_long_count.value)}, exp {FRAME_LINES}")
    check(int(dut.crc_err_count.value) == 0,
          f"crc err (crc_err_count={int(dut.crc_err_count.value)})")
    check(int(dut.parser_trunc_count.value) == 0,
          f"trunc (parser_trunc_count={int(dut.parser_trunc_count.value)})")
    check(int(dut.last_frame_lines.value) == FRAME_LINES,
          f"last_frame_lines wrong: got {int(dut.last_frame_lines.value)}, exp {FRAME_LINES}")

    # Drain OB masker (1-line latency + slack) -- SV: repeat(200)
    for _ in range(200):
        await RisingEdge(clk)

    dut._log.info("ob_capture size = %d (expect %d)", len(cap.pixels), FRAME_PIXELS)

    # --- pixel-by-pixel compare (the SV FAIL loop) ---
    check(len(cap.pixels) == FRAME_PIXELS,
          f"captured {len(cap.pixels)} pixels, expected {FRAME_PIXELS}")
    for i in range(FRAME_PIXELS):
        got = cap.pixels[i]
        exp = EXPECTED_Y[i]
        check(got == exp,
              f"pix[{i}] got=0x{got:03x} expected=0x{exp:03x}")


# ---- build harness: emit the 7-DUT wiring wrapper, then build+run under Verilator ----
#
# The port maps + parameters below are copied 1:1 from tb_raw10_ob_masker_e2e.sv. The
# wrapper exposes only the ports cocotb drives (core_clk/core_aresetn/s_byte_*) and the
# ports/statuses cocotb reads (ob_pixel/ob_valid + the counters the TB checks).

_HARNESS = r"""
`timescale 1ns / 1ps
// Auto-generated E2E wrapper for the cocotb port of tb_raw10_ob_masker_e2e.sv.
// Contains ONLY the 7 DUT instances (no initial / no clock) so cocotb owns clk/rst +
// stimulus + capture. Wiring + parameters are 1:1 with the DSim TB.
module raw10_ob_e2e_harness #(
    parameter int PARSER_IN_WIDTH = 16,
    parameter int LINE_PIXELS     = 8,
    parameter int FRAME_LINES     = 4,
    parameter int LINE_BYTES      = 10
)(
    input  wire                        core_clk,
    input  wire                        core_aresetn,

    input  wire [PARSER_IN_WIDTH-1:0]  s_byte_data,
    input  wire [PARSER_IN_WIDTH/8-1:0] s_byte_keep,
    input  wire                        s_byte_valid,
    input  wire                        s_byte_sop,
    input  wire                        s_byte_eop,

    output wire [9:0]                  ob_pixel,
    output wire                        ob_valid,

    output wire [15:0]                 parser_short_count,
    output wire [15:0]                 parser_long_count,
    output wire [15:0]                 parser_trunc_count,
    output wire [15:0]                 crc_err_count,
    output wire [31:0]                 frame_count,
    output wire [15:0]                 last_frame_lines
);
    localparam logic [5:0] DT_RAW10 = 6'h2B;

    // ---- parser <-> ecc/crc/filter/frame nets ----
    logic        parser_ecc_hdr_valid;
    logic [31:0] parser_ecc_hdr_raw;
    logic        ecc_hdr_corr_valid;
    logic [23:0] ecc_hdr_corr;
    logic [7:0]  ecc_hdr_di;
    logic [15:0] ecc_hdr_wc;
    logic        ecc_hdr_corrected, ecc_hdr_uncorrectable, ecc_hdr_no_error;
    logic [15:0] ecc_corr_count, ecc_uncorr_count;
    logic        pkt_hdr_valid;
    logic [31:0] pkt_hdr_raw;
    logic [7:0]  pkt_di;
    logic [15:0] pkt_wc;
    logic        pkt_is_long, pkt_is_short, pkt_ecc_uncorrectable;
    logic [7:0]  payload_data;
    logic        payload_valid, payload_first, payload_last;
    logic [15:0] footer_data;
    logic        footer_valid, pkt_done;

    logic        crc_check_valid, crc_match;
    logic [15:0] crc_calc, crc_received, crc_ok_count;

    logic [7:0]  filter_pkt_di;
    logic [15:0] filter_pkt_wc;
    logic        filter_pkt_is_short, filter_pkt_is_long, filter_pkt_start, filter_pkt_end, filter_pkt_err;
    logic [7:0]  filter_payload_data;
    logic        filter_payload_valid, filter_payload_first, filter_payload_last;
    logic [15:0] filter_drop_vc_count, filter_drop_dt_count;

    logic        frame_sof, frame_eof, frame_sol, frame_eol;
    logic [15:0] frame_line_idx;
    logic [7:0]  frame_payload_data;
    logic        frame_payload_valid, frame_payload_first, frame_payload_last, frame_err;
    logic [31:0] line_count;
    logic [15:0] frame_sync_err_count;

    logic [9:0]  raw10_pixel;
    logic        raw10_valid, raw10_sof, raw10_eol, raw10_eof, raw10_err;
    logic [15:0] raw10_pix_per_line;

    logic [9:0]  ob_pixel_i;
    logic        ob_valid_i, ob_sof, ob_eol, ob_eof, ob_err;

    csi2_packet_parser #(
        .IN_WIDTH(PARSER_IN_WIDTH), .WC_MAX(256), .FIFO_DEPTH(256)
    ) u_parser (
        .core_clk(core_clk), .core_aresetn(core_aresetn),
        .s_byte_data(s_byte_data), .s_byte_keep(s_byte_keep),
        .s_byte_valid(s_byte_valid), .s_byte_sop(s_byte_sop), .s_byte_eop(s_byte_eop),
        .ecc_hdr_valid(parser_ecc_hdr_valid), .ecc_hdr_raw(parser_ecc_hdr_raw),
        .ecc_hdr_corr_valid(ecc_hdr_corr_valid), .ecc_hdr_di(ecc_hdr_di), .ecc_hdr_wc(ecc_hdr_wc),
        .ecc_hdr_uncorrectable(ecc_hdr_uncorrectable),
        .m_pkt_hdr_valid(pkt_hdr_valid), .m_pkt_hdr_raw(pkt_hdr_raw),
        .m_pkt_di(pkt_di), .m_pkt_wc(pkt_wc),
        .m_pkt_is_long(pkt_is_long), .m_pkt_is_short(pkt_is_short),
        .m_pkt_ecc_uncorrectable(pkt_ecc_uncorrectable),
        .m_payload_data(payload_data), .m_payload_valid(payload_valid),
        .m_payload_first(payload_first), .m_payload_last(payload_last),
        .m_footer_data(footer_data), .m_footer_valid(footer_valid), .m_pkt_done(pkt_done),
        .sts_short_pkt_cnt(parser_short_count), .sts_long_pkt_cnt(parser_long_count),
        .sts_pkt_trunc_cnt(parser_trunc_count)
    );

    csi2_header_ecc u_header_ecc (
        .core_clk(core_clk), .core_aresetn(core_aresetn),
        .hdr_valid(parser_ecc_hdr_valid), .hdr_raw(parser_ecc_hdr_raw),
        .hdr_corr_valid(ecc_hdr_corr_valid), .hdr_corr(ecc_hdr_corr),
        .hdr_di(ecc_hdr_di), .hdr_wc(ecc_hdr_wc),
        .hdr_ecc_corrected(ecc_hdr_corrected), .hdr_ecc_uncorrectable(ecc_hdr_uncorrectable),
        .hdr_ecc_no_error(ecc_hdr_no_error),
        .sts_ecc_corr_cnt(ecc_corr_count), .sts_ecc_uncorr_cnt(ecc_uncorr_count)
    );

    csi2_payload_crc u_payload_crc (
        .core_clk(core_clk), .core_aresetn(core_aresetn),
        .payload_data(payload_data), .payload_valid(payload_valid),
        .payload_first(payload_first), .payload_last(payload_last),
        .footer_data(footer_data), .footer_valid(footer_valid),
        .crc_check_valid(crc_check_valid), .crc_match(crc_match),
        .crc_calc(crc_calc), .crc_received(crc_received),
        .sts_crc_err_cnt(crc_err_count), .sts_crc_ok_cnt(crc_ok_count)
    );

    csi2_vcdt_filter u_filter (
        .core_clk(core_clk), .core_aresetn(core_aresetn),
        .cfg_expected_vc(2'b00), .cfg_expected_dt(DT_RAW10),
        .cfg_pass_short(1'b1), .cfg_pass_emb_data(1'b0),
        .pkt_hdr_valid(pkt_hdr_valid), .pkt_di(pkt_di), .pkt_wc(pkt_wc),
        .pkt_is_long(pkt_is_long), .pkt_is_short(pkt_is_short), .pkt_done(pkt_done),
        .ecc_corrected(ecc_hdr_corrected), .ecc_uncorrectable(pkt_ecc_uncorrectable),
        .crc_check_valid(crc_check_valid), .crc_match(crc_match),
        .payload_data(payload_data), .payload_valid(payload_valid),
        .payload_first(payload_first), .payload_last(payload_last),
        .out_pkt_di(filter_pkt_di), .out_pkt_wc(filter_pkt_wc),
        .out_pkt_is_short(filter_pkt_is_short), .out_pkt_is_long(filter_pkt_is_long),
        .out_pkt_start(filter_pkt_start), .out_pkt_end(filter_pkt_end), .out_pkt_err(filter_pkt_err),
        .out_payload_data(filter_payload_data), .out_payload_valid(filter_payload_valid),
        .out_payload_first(filter_payload_first), .out_payload_last(filter_payload_last),
        .sts_drop_vc_cnt(filter_drop_vc_count), .sts_drop_dt_cnt(filter_drop_dt_count)
    );

    csi2_frame_state #(
        .MAX_LINES(16), .GUARD_FRAME_LINES(1'b1),
        .EXPECTED_FRAME_LINES(FRAME_LINES), .EXPECTED_LINE_WC(16'(LINE_BYTES))
    ) u_frame_state (
        .core_clk(core_clk), .core_aresetn(core_aresetn),
        .cfg_use_lsle(1'b0), .cfg_expected_frame_lines(16'd0),
        .in_pkt_di(filter_pkt_di), .in_pkt_wc(filter_pkt_wc),
        .in_pkt_is_short(filter_pkt_is_short), .in_pkt_is_long(filter_pkt_is_long),
        .in_pkt_start(filter_pkt_start), .in_pkt_end(filter_pkt_end), .in_pkt_err(filter_pkt_err),
        .in_payload_data(filter_payload_data), .in_payload_valid(filter_payload_valid),
        .in_payload_first(filter_payload_first), .in_payload_last(filter_payload_last),
        .out_sof(frame_sof), .out_eof(frame_eof), .out_sol(frame_sol), .out_eol(frame_eol),
        .out_line_idx(frame_line_idx),
        .out_payload_data(frame_payload_data), .out_payload_valid(frame_payload_valid),
        .out_payload_first(frame_payload_first), .out_payload_last(frame_payload_last),
        .out_frame_err(frame_err),
        .sts_frame_count(frame_count), .sts_line_count(line_count),
        .sts_last_frame_lines(last_frame_lines), .sts_frame_sync_err_cnt(frame_sync_err_count)
    );

    raw10_unpack #(.LINE_PIXELS(LINE_PIXELS)) u_raw10 (
        .core_clk(core_clk), .core_aresetn(core_aresetn),
        .in_sof(frame_sof), .in_eof(frame_eof), .in_eol(frame_eol),
        .in_payload_data(frame_payload_data), .in_payload_valid(frame_payload_valid),
        .in_payload_first(frame_payload_first), .in_payload_last(frame_payload_last),
        .in_frame_err(frame_err),
        .out_pixel(raw10_pixel), .out_pixel_valid(raw10_valid),
        .out_pixel_sof(raw10_sof), .out_pixel_eol(raw10_eol),
        .out_pixel_eof(raw10_eof), .out_pixel_err(raw10_err),
        .sts_pixel_per_line(raw10_pix_per_line)
    );

    ob_row_masker #(
        .WIDTH(10), .LINE_PIXELS_MAX(64),
        .OB_THRESHOLD(10'd200), .OB_FILL_Y(10'd512), .OB_UNIFORMITY(10'd12)
    ) u_ob (
        .clk(core_clk), .aresetn(core_aresetn), .enable(1'b1),
        .in_data(raw10_pixel), .in_valid(raw10_valid),
        .in_sof(raw10_sof), .in_eol(raw10_eol),
        .in_eof(raw10_eof), .in_err(raw10_err),
        .out_data(ob_pixel_i), .out_valid(ob_valid_i),
        .out_sof(ob_sof), .out_eol(ob_eol),
        .out_eof(ob_eof), .out_err(ob_err)
    );

    assign ob_pixel = ob_pixel_i;
    assign ob_valid = ob_valid_i;
endmodule
"""


def test_raw10_ob_masker_e2e():
    from runner_support import build_and_test

    here = Path(__file__).resolve().parent
    harness = here / "raw10_ob_masker_e2e_stubs.sv"
    harness.write_text(_HARNESS, encoding="ascii")

    build_and_test(
        block="raw10_ob_masker_e2e",
        sources=[
            "rtl/mipi_rx/csi2_packet_parser.sv",
            "rtl/mipi_rx/csi2_header_ecc.sv",
            "rtl/mipi_rx/csi2_payload_crc.sv",
            "rtl/mipi_rx/csi2_vcdt_filter.sv",
            "rtl/mipi_rx/csi2_frame_state.sv",
            "rtl/img_proc/raw10_unpack.sv",
            "rtl/img_proc/ob_row_masker.sv",
            str(harness),
        ],
        toplevel="raw10_ob_e2e_harness",
        test_module="test_raw10_ob_masker_e2e",
        test_dir=here,
        parameters={
            "PARSER_IN_WIDTH": PARSER_IN_WIDTH,
            "LINE_PIXELS": LINE_PIXELS,
            "FRAME_LINES": FRAME_LINES,
            "LINE_BYTES": LINE_BYTES,
        },
        engine="verilator",
    )
