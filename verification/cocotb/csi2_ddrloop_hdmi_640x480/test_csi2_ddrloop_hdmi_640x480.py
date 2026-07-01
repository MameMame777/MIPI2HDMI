"""cocotb port of verification/tb/tb_csi2_ddrloop_hdmi_640x480.sv
(large multi-source E2E: CSI-2 receive -> DDR round-trip -> HDMI out, 640x480 YUV422).

The DSim TB chains the real RTL end to end:

    s_byte (16-bit byte-beat, core_clk)
      -> csi2_packet_parser      (header/payload/CRC split)
      -> csi2_header_ecc         (Hamming ECC over the packet header)
      -> csi2_payload_crc        (CRC-16 over the payload)
      -> csi2_vcdt_filter        (VC/DT gate, DT=YUV422)
      -> csi2_frame_state        (SOF/EOF/SOL/EOL framing, FS/FE short-packet delimited)
      -> yuv422_gray_unpack      (YUV422 byte stream -> Y8 = 24-bit grey pixel)
      -> axis_video_bridge       (dual-clock CDC core_clk -> aclk, AXI4-Stream)
      -> axis_y8_to_vdma32       (pack 4 x Y8 -> 32-bit DDR beat)  ]  DDR round-trip
      -> [behavioural DDR queue on aclk]                          ]  (loopback via a
      -> axis_vdma32_to_y8       (unpack 32-bit beat -> Y8)       ]   SW model, no real DDR)
      -> [behavioural CDC FIFO aclk -> pix_clk]                       models bd_cc_mm2s
      -> hdmi_output             (video timing + RGB + TMDS, pix_clk)

Because cocotb needs a single HDL toplevel and the two cross-domain queue models are
delicate handshake models the DSim test relied on, the whole chain (DUTs + the two SV
queue models, 1:1 with the TB) is emitted as ``csi2_ddrloop_hdmi_640x480_harness``.
cocotb owns the three clocks, reset, the CSI-2 byte-beat stimulus, and every check.
The TB's ``$fatal`` on a non-full DDR TKEEP becomes the sticky ``ddr_tkeep_err`` flag,
which the test asserts is 0.

The single SV ``initial`` run is replicated 1:1: FS short packet, FRAME_LINES YUV422
lines (Y = ((line*LINE_PIXELS)+col)&0xff, chroma=0x80, WC=LINE_BYTES, correct ECC+CRC),
FE short packet; then the status-counter checks, the DDR full-frame beat count, and the
HDMI active-window pixel check (every active pixel == expected grey).
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.scoreboard import check  # noqa: E402

# --- TB localparams (1:1) --------------------------------------------------------------
PARSER_IN_WIDTH = 16
DT_FS = 0x00
DT_FE = 0x01
DT_YUV422 = 0x1E

LINE_PIXELS = 640
FRAME_LINES = 480
LINE_BYTES = LINE_PIXELS * 2                 # 1280 bytes / YUV422 line
BEATS_PER_LINE = LINE_PIXELS // 4            # 160 packed 32-bit beats / line
FRAME_BEATS = BEATS_PER_LINE * FRAME_LINES   # 76800


def expected_y(line_idx: int, col_idx: int) -> int:
    """Test image: gradient. Mirror of the SV expected_y function."""
    return ((line_idx * LINE_PIXELS) + col_idx) & 0xFF


# --- ECC / CRC reference (ports of the SV functions) -----------------------------------
def _calc_ecc6(data: int) -> int:
    def b(i):
        return (data >> i) & 1
    e = [0] * 6
    e[0] = b(0) ^ b(1) ^ b(2) ^ b(4) ^ b(5) ^ b(7) ^ b(10) ^ b(11) ^ b(13) ^ b(16) ^ b(20) ^ b(21) ^ b(22) ^ b(23)
    e[1] = b(0) ^ b(1) ^ b(3) ^ b(4) ^ b(6) ^ b(8) ^ b(10) ^ b(12) ^ b(14) ^ b(17) ^ b(20) ^ b(21) ^ b(22) ^ b(23)
    e[2] = b(0) ^ b(2) ^ b(3) ^ b(5) ^ b(6) ^ b(9) ^ b(11) ^ b(12) ^ b(15) ^ b(18) ^ b(20) ^ b(21) ^ b(22)
    e[3] = b(1) ^ b(2) ^ b(3) ^ b(7) ^ b(8) ^ b(9) ^ b(13) ^ b(14) ^ b(15) ^ b(19) ^ b(20) ^ b(21) ^ b(23)
    e[4] = b(4) ^ b(5) ^ b(6) ^ b(7) ^ b(8) ^ b(9) ^ b(16) ^ b(17) ^ b(18) ^ b(19) ^ b(20) ^ b(22) ^ b(23)
    e[5] = b(10) ^ b(11) ^ b(12) ^ b(13) ^ b(14) ^ b(15) ^ b(16) ^ b(17) ^ b(18) ^ b(19) ^ b(21) ^ b(22) ^ b(23)
    return sum(e[i] << i for i in range(6))


def make_ecc(di: int, wc: int) -> int:
    """{2'b00, calc_ecc6({wc, di})} -- data field is {wc[15:0], di[7:0]} (24 bits)."""
    return _calc_ecc6(((wc & 0xFFFF) << 8) | (di & 0xFF)) & 0x3F


def crc_update_byte(crc_in: int, data: int) -> int:
    """CSI-2 CRC-16 (poly 0x8408, LSB-first) -- mirror of the SV function."""
    crc = crc_in & 0xFFFF
    for bit_idx in range(8):
        feedback = (crc & 1) ^ ((data >> bit_idx) & 1)
        crc >>= 1
        if feedback:
            crc ^= 0x8408
    return crc & 0xFFFF


# --- CSI-2 byte-beat stimulus (negedge cadence, 1:1 with the SV drive tasks) ------------
class ByteBeatDriver:
    """Mirror of the SV drive_beat/drive_idle tasks: all changes on negedge core_clk with
    NBA-style hold (each drive holds the bus until the next drive/idle changes it)."""

    def __init__(self, dut):
        self.dut = dut

    async def drive_beat(self, byte0: int, byte1: int, sop: int, eop: int) -> None:
        # SV drive_beat: @(negedge); s_byte_data<={byte1,byte0}; keep=11; valid=1; sop; eop
        await FallingEdge(self.dut.core_clk)
        self.dut.s_byte_data.value = ((byte1 & 0xFF) << 8) | (byte0 & 0xFF)
        self.dut.s_byte_keep.value = 0b11
        self.dut.s_byte_valid.value = 1
        self.dut.s_byte_sop.value = sop
        self.dut.s_byte_eop.value = eop

    async def drive_idle(self, cycles: int) -> None:
        # SV drive_idle: @(negedge); valid=0; keep=0; sop=0; eop=0; data=0; repeat(cycles)@(negedge)
        await FallingEdge(self.dut.core_clk)
        self.dut.s_byte_valid.value = 0
        self.dut.s_byte_keep.value = 0
        self.dut.s_byte_sop.value = 0
        self.dut.s_byte_eop.value = 0
        self.dut.s_byte_data.value = 0
        for _ in range(cycles):
            await FallingEdge(self.dut.core_clk)

    async def send_short_packet(self, dt: int, data_field: int) -> None:
        di = dt & 0x3F                      # {2'b00, dt}
        ecc = make_ecc(di, data_field)
        await self.drive_beat(di, data_field & 0xFF, 1, 0)
        await self.drive_beat((data_field >> 8) & 0xFF, ecc, 0, 1)
        await self.drive_idle(2)

    async def send_yuv422_line(self, line_idx: int) -> None:
        di = DT_YUV422 & 0x3F
        wc = LINE_BYTES & 0xFFFF
        ecc = make_ecc(di, wc)

        # CRC over payload only (chroma=0x80 then Y per pixel).
        crc = 0xFFFF
        for col in range(LINE_PIXELS):
            crc = crc_update_byte(crc, 0x80)
            crc = crc_update_byte(crc, expected_y(line_idx, col))

        # Header (4 bytes = 2 beats).
        await self.drive_beat(di, wc & 0xFF, 1, 0)
        await self.drive_beat((wc >> 8) & 0xFF, ecc, 0, 0)

        # Payload pairs (chroma, Y), 1 beat then 1 idle cycle (throttle to 1 byte/cycle).
        for col in range(LINE_PIXELS):
            await self.drive_beat(0x80, expected_y(line_idx, col), 0, 0)
            await self.drive_idle(1)

        # CRC footer (2 bytes = 1 beat).
        await self.drive_beat(crc & 0xFF, (crc >> 8) & 0xFF, 0, 1)
        await self.drive_idle(2)


# --- reset (three domains), 1:1 with the SV reset_dut task ------------------------------
async def reset_dut(dut):
    dut.core_aresetn.value = 0
    dut.aresetn.value = 0
    dut.pix_aresetn.value = 0
    dut.s_byte_data.value = 0
    dut.s_byte_keep.value = 0
    dut.s_byte_valid.value = 0
    dut.s_byte_sop.value = 0
    dut.s_byte_eop.value = 0
    dut.hdmi_enable.value = 0
    for _ in range(8):
        await RisingEdge(dut.core_clk)
    dut.core_aresetn.value = 1
    for _ in range(8):
        await RisingEdge(dut.aclk)
    dut.aresetn.value = 1
    for _ in range(4):
        await RisingEdge(dut.pix_clk)
    dut.pix_aresetn.value = 1
    for _ in range(4):
        await RisingEdge(dut.core_clk)


# --- waiters (ports of the SV timeout tasks) -------------------------------------------
async def wait_frame_done(dut, timeout_cycles: int):
    for _ in range(timeout_cycles):
        await RisingEdge(dut.core_clk)
        if int(dut.frame_count.value) == 1:
            return
    raise AssertionError(
        f"CHECK FAILED: Timed out waiting for CSI-2 frame completion "
        f"(frame_count={int(dut.frame_count.value)})")


async def wait_parser_short_count(dut, expected_count: int, timeout_cycles: int):
    for _ in range(timeout_cycles):
        await RisingEdge(dut.core_clk)
        if int(dut.parser_short_count.value) >= expected_count:
            return
    raise AssertionError(
        f"CHECK FAILED: Timed out waiting for parser short count {expected_count}, "
        f"saw {int(dut.parser_short_count.value)}")


async def wait_ddr_full_frame(dut, timeout_cycles: int):
    for _ in range(timeout_cycles):
        await RisingEdge(dut.aclk)
        if int(dut.ddr_beats_seen.value) == FRAME_BEATS:
            return
    raise AssertionError(
        f"CHECK FAILED: Timed out waiting for {FRAME_BEATS} DDR beats, "
        f"saw {int(dut.ddr_beats_seen.value)}")


async def check_hdmi_full_frame(dut, timeout_cycles: int):
    """Port of the SV check_hdmi_full_frame task: enable HDMI, then on each active
    (video_de) pixel verify {r,g,b} == {expected, expected, expected} in raster order and
    that neither the underflow nor the AXIS-error counter moved during the window."""
    underflow_before = int(dut.hdmi_underflow_count.value)
    axis_error_before = int(dut.hdmi_axis_error_count.value)
    hdmi_pixel_count = 0
    hdmi_line_count = 0
    hdmi_col_count = 0

    await FallingEdge(dut.pix_clk)
    dut.hdmi_enable.value = 1

    total = LINE_PIXELS * FRAME_LINES
    for _ in range(timeout_cycles):
        await RisingEdge(dut.pix_clk)   # sample one delta after the edge (== SV #1)
        if int(dut.video_de.value):
            if hdmi_pixel_count >= total:
                raise AssertionError("CHECK FAILED: HDMI emitted more active pixels than frame size")
            expected_pixel = expected_y(hdmi_line_count, hdmi_col_count)
            got_r = int(dut.video_r.value)
            got_g = int(dut.video_g.value)
            got_b = int(dut.video_b.value)
            if (got_r, got_g, got_b) != (expected_pixel, expected_pixel, expected_pixel):
                raise AssertionError(
                    f"CHECK FAILED: HDMI pixel mismatch line={hdmi_line_count} "
                    f"col={hdmi_col_count} got={got_r:02x}{got_g:02x}{got_b:02x} "
                    f"expected={expected_pixel:02x}{expected_pixel:02x}{expected_pixel:02x}")
            hdmi_pixel_count += 1
            hdmi_col_count += 1
            if hdmi_col_count == LINE_PIXELS:
                hdmi_col_count = 0
                hdmi_line_count += 1
            if hdmi_pixel_count == total:
                break

    check(hdmi_pixel_count == total,
          f"HDMI delivered all {total} pixels (saw {hdmi_pixel_count})")
    for _ in range(16):
        await RisingEdge(dut.pix_clk)
    check(int(dut.hdmi_underflow_count.value) == underflow_before,
          f"HDMI active window had no underflow "
          f"(before={underflow_before} after={int(dut.hdmi_underflow_count.value)})")
    check(int(dut.hdmi_axis_error_count.value) == axis_error_before,
          f"HDMI active window had no AXIS sideband error "
          f"(before={axis_error_before} after={int(dut.hdmi_axis_error_count.value)})")


# --- the test: replicate the single SV initial run 1:1 ---------------------------------
@cocotb.test(timeout_time=50, timeout_unit="ms")
async def ddrloop_hdmi_640x480(dut):
    # Three async clocks, exactly the SV periods: core_clk #5 (10 ns), aclk #4 (8 ns),
    # pix_clk #20 (40 ns).
    cocotb.start_soon(Clock(dut.core_clk, 10.0, unit="ns").start())
    cocotb.start_soon(Clock(dut.aclk, 8.0, unit="ns").start())
    cocotb.start_soon(Clock(dut.pix_clk, 40.0, unit="ns").start())

    dut._log.info(
        "INFO: starting csi2_ddrloop_hdmi_640x480, frame=%dx%d, beats/frame=%d",
        LINE_PIXELS, FRAME_LINES, FRAME_BEATS)

    await reset_dut(dut)

    drv = ByteBeatDriver(dut)

    # --- drive the CSI-2 frame: FS, FRAME_LINES YUV422 lines, FE ---
    await drv.send_short_packet(DT_FS, 0x0000)
    for line_idx in range(FRAME_LINES):
        await drv.send_yuv422_line(line_idx)
    await drv.send_short_packet(DT_FE, 0x0000)
    dut._log.info("INFO: CSI-2 input drive complete")

    await wait_frame_done(dut, 1_000_000)
    await wait_parser_short_count(dut, 2, 1_000)
    dut._log.info("INFO: CSI-2 frame complete, line_count=%d", int(dut.line_count.value))
    dut._log.info(
        "INFO: counters short=%d long=%d trunc=%d ecc_uncorr=%d crc_ok=%d crc_err=%d "
        "drop_vc=%d drop_dt=%d frame_sync_err=%d bridge_ovf=%d bridge_bp=%d",
        int(dut.parser_short_count.value), int(dut.parser_long_count.value),
        int(dut.parser_trunc_count.value), int(dut.ecc_uncorr_count.value),
        int(dut.crc_ok_count.value), int(dut.crc_err_count.value),
        int(dut.filter_drop_vc_count.value), int(dut.filter_drop_dt_count.value),
        int(dut.frame_sync_err_count.value), int(dut.bridge_overflow_count.value),
        int(dut.bridge_back_pressure_count.value))

    # --- status-counter checks (1:1 with the SV check_condition calls) ---
    check(int(dut.parser_short_count.value) == 2, "parser saw FS and FE short packets")
    check(int(dut.parser_long_count.value) == FRAME_LINES, "parser saw one long packet per line")
    check(int(dut.parser_trunc_count.value) == 0, "parser saw no truncation")
    check(int(dut.ecc_uncorr_count.value) == 0, "header ECC has no uncorrectable errors")
    check(int(dut.crc_ok_count.value) == FRAME_LINES, "payload CRC matched once per line")
    check(int(dut.crc_err_count.value) == 0, "payload CRC has no errors")
    check(int(dut.filter_drop_vc_count.value) == 0, "filter dropped no VC packets")
    check(int(dut.filter_drop_dt_count.value) == 0, "filter dropped no DT packets")
    check(int(dut.frame_sync_err_count.value) == 0, "frame state has no sync errors")
    check(int(dut.line_count.value) == FRAME_LINES, "frame state counted FRAME_LINES lines")
    check(int(dut.last_frame_lines.value) == FRAME_LINES, "frame state ended FRAME_LINES-line frame")
    check(int(dut.yuv_pixel_per_line.value) == LINE_PIXELS, "YUV unpacker counted LINE_PIXELS per line")
    check(int(dut.bridge_overflow_count.value) == 0, "video bridge did not overflow")

    # --- DDR round-trip full frame ---
    await wait_ddr_full_frame(dut, 1_000_000)
    dut._log.info("INFO: DDR full frame received, beats=%d", int(dut.ddr_beats_seen.value))
    check(int(dut.ddr_beats_seen.value) == FRAME_BEATS,
          f"DDR model received {FRAME_BEATS} packed beats")
    check(int(dut.ddr_tkeep_err.value) == 0,
          "DDR loop saw only full 32-bit beats (TKEEP=0xF)")

    # --- HDMI active-window full-frame check ---
    await check_hdmi_full_frame(dut, 2_000_000)
    dut._log.info("INFO: HDMI full-frame check complete")


def test_csi2_ddrloop_hdmi_640x480():
    from runner_support import build_and_test

    here = Path(__file__).resolve().parent
    build_and_test(
        block="csi2_ddrloop_hdmi_640x480",
        sources=[
            "rtl/mipi_rx/csi2_packet_parser.sv",
            "rtl/mipi_rx/csi2_header_ecc.sv",
            "rtl/mipi_rx/csi2_payload_crc.sv",
            "rtl/mipi_rx/csi2_vcdt_filter.sv",
            "rtl/mipi_rx/csi2_frame_state.sv",
            "rtl/img_proc/yuv422_gray_unpack.sv",
            "rtl/img_proc/rgb565_gray_unpack.sv",
            "rtl/mipi_rx/axis_video_bridge.sv",
            "rtl/img_proc/axis_y8_to_vdma32.sv",
            "rtl/img_proc/axis_vdma32_to_y8.sv",
            "rtl/hdmi/hdmi_output.sv",
            str(here / "csi2_ddrloop_hdmi_640x480_harness.sv"),
        ],
        toplevel="csi2_ddrloop_hdmi_640x480_harness",
        test_module="test_csi2_ddrloop_hdmi_640x480",
        test_dir=here,
        parameters={
            "PARSER_IN_WIDTH": PARSER_IN_WIDTH,
            "LINE_PIXELS": LINE_PIXELS,
            "FRAME_LINES": FRAME_LINES,
        },
        engine="verilator",
    )
