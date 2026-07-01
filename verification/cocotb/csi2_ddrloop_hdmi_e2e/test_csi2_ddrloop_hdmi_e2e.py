"""cocotb port of verification/tb/tb_csi2_ddrloop_hdmi_e2e.sv (E2E CSI-2 -> DDR-loop -> HDMI).

The DSim TB chains the full receive-to-display pipeline and closes a DDR round-trip in
between:

    s_byte_* (byte-beat stimulus)
      -> csi2_packet_parser -> csi2_header_ecc -> csi2_payload_crc -> csi2_vcdt_filter
      -> csi2_frame_state -> yuv422_gray_unpack -> axis_video_bridge (core_clk->aclk CDC)
      -> axis_y8_to_vdma32 (Y8 -> 32b VDMA beats) -> [DDR loop] -> axis_vdma32_to_y8
      -> hdmi_output (Y replicated on R=G=B)

The TB feeds one FS short packet, FRAME_LINES YUV422 long-packet lines (4 pixels each,
chroma neutral 0x80, luma = expected_y[]), one FE short packet, then verifies the whole
chain's status counters, the DDR beat count (one full 32-bit beat per 4-pixel line), and
finally that hdmi_output emits every reconstructed pixel with R=G=B=Y in order.

cocotb needs one synthesizable HDL toplevel, so the DUT wiring + the DDR-loop model (a SV
`ddr_queue [$]` in the TB) are emitted as a synthesizable wrapper
(``csi2_ddrloop_hdmi_e2e_harness``, in csi2_ddrloop_hdmi_e2e_stubs.sv): the DDR loop is a
depth-64 AXIS FIFO, and the TB's ``$fatal`` on TKEEP != 0xF becomes a sticky
``ddr_tkeep_err`` flag this test checks. All ``initial`` stimulus tasks + the
``check_condition``/``$fatal`` checks are ported 1:1 here.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import FallingEdge, RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import start_clock  # noqa: E402
from lib.scoreboard import check  # noqa: E402

# --- TB localparams (1:1) -------------------------------------------------------------
PARSER_IN_WIDTH = 16
DT_FS = 0x00
DT_FE = 0x01
DT_YUV422 = 0x1E

LINE_PIXELS = 4
FRAME_LINES = 2
LINE_BYTES = LINE_PIXELS * 2
FRAME_PIXELS = LINE_PIXELS * FRAME_LINES

EXPECTED_Y = [0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80]


# --- ECC / CRC helpers, ported 1:1 from the SV functions ------------------------------

def _calc_ecc6(data: int) -> int:
    """Port of calc_ecc6(input logic [23:0] data)."""
    def b(i: int) -> int:
        return (data >> i) & 1

    e0 = b(0) ^ b(1) ^ b(2) ^ b(4) ^ b(5) ^ b(7) ^ b(10) ^ b(11) ^ b(13) ^ b(16) ^ b(20) ^ b(21) ^ b(22) ^ b(23)
    e1 = b(0) ^ b(1) ^ b(3) ^ b(4) ^ b(6) ^ b(8) ^ b(10) ^ b(12) ^ b(14) ^ b(17) ^ b(20) ^ b(21) ^ b(22) ^ b(23)
    e2 = b(0) ^ b(2) ^ b(3) ^ b(5) ^ b(6) ^ b(9) ^ b(11) ^ b(12) ^ b(15) ^ b(18) ^ b(20) ^ b(21) ^ b(22)
    e3 = b(1) ^ b(2) ^ b(3) ^ b(7) ^ b(8) ^ b(9) ^ b(13) ^ b(14) ^ b(15) ^ b(19) ^ b(20) ^ b(21) ^ b(23)
    e4 = b(4) ^ b(5) ^ b(6) ^ b(7) ^ b(8) ^ b(9) ^ b(16) ^ b(17) ^ b(18) ^ b(19) ^ b(20) ^ b(22) ^ b(23)
    e5 = b(10) ^ b(11) ^ b(12) ^ b(13) ^ b(14) ^ b(15) ^ b(16) ^ b(17) ^ b(18) ^ b(19) ^ b(21) ^ b(22) ^ b(23)
    return e0 | (e1 << 1) | (e2 << 2) | (e3 << 3) | (e4 << 4) | (e5 << 5)


def _make_ecc(di: int, wc: int) -> int:
    """Port of make_ecc: {2'b00, calc_ecc6({wc, di})}."""
    return _calc_ecc6(((wc & 0xFFFF) << 8) | (di & 0xFF)) & 0x3F


def _crc_update_byte(crc_in: int, data: int) -> int:
    """Port of crc_update_byte (LSB-first, poly 0x8408)."""
    crc = crc_in & 0xFFFF
    for bit_idx in range(8):
        feedback = (crc & 1) ^ ((data >> bit_idx) & 1)
        crc >>= 1
        if feedback:
            crc ^= 0x8408
    return crc & 0xFFFF


# --- byte-beat driver, matching the TB's negedge cadence exactly ----------------------
#
# The TB drives s_byte_* on `@(negedge core_clk)`: drive_beat asserts valid for a cycle,
# and back-to-back drive_beats keep valid high across the whole packet (no idle gap inside
# a line), while drive_idle deasserts for N cycles. We replicate that by aligning writes to
# the falling edge (setup before the rising sample edge).

class ByteDriver:
    def __init__(self, dut):
        self.dut = dut

    async def drive_beat(self, byte0: int, byte1: int, sop: int, eop: int) -> None:
        await FallingEdge(self.dut.core_clk)
        self.dut.s_byte_data.value = ((byte1 & 0xFF) << 8) | (byte0 & 0xFF)
        self.dut.s_byte_keep.value = 0b11
        self.dut.s_byte_valid.value = 1
        self.dut.s_byte_sop.value = sop
        self.dut.s_byte_eop.value = eop

    async def drive_idle(self, cycles: int) -> None:
        await FallingEdge(self.dut.core_clk)
        self.dut.s_byte_valid.value = 0
        self.dut.s_byte_keep.value = 0
        self.dut.s_byte_sop.value = 0
        self.dut.s_byte_eop.value = 0
        self.dut.s_byte_data.value = 0
        for _ in range(cycles):
            await FallingEdge(self.dut.core_clk)

    async def send_short_packet(self, dt: int, data_field: int) -> None:
        di = dt & 0x3F  # {2'b00, dt}
        ecc = _make_ecc(di, data_field)
        await self.drive_beat(di, data_field & 0xFF, 1, 0)
        await self.drive_beat((data_field >> 8) & 0xFF, ecc, 0, 1)
        await self.drive_idle(2)

    async def send_yuv422_line_4px(self, line_idx: int) -> None:
        di = DT_YUV422 & 0x3F
        wc = 8
        ecc = _make_ecc(di, wc)
        base = line_idx * LINE_PIXELS
        payload = [
            0x80, EXPECTED_Y[base + 0],
            0x80, EXPECTED_Y[base + 1],
            0x80, EXPECTED_Y[base + 2],
            0x80, EXPECTED_Y[base + 3],
        ]
        crc = 0xFFFF
        for p in payload:
            crc = _crc_update_byte(crc, p)

        await self.drive_beat(di, wc & 0xFF, 1, 0)
        await self.drive_beat((wc >> 8) & 0xFF, ecc, 0, 0)
        await self.drive_beat(payload[0], payload[1], 0, 0)
        await self.drive_beat(payload[2], payload[3], 0, 0)
        await self.drive_beat(payload[4], payload[5], 0, 0)
        await self.drive_beat(payload[6], payload[7], 0, 0)
        await self.drive_beat(crc & 0xFF, (crc >> 8) & 0xFF, 0, 1)
        await self.drive_idle(2)


# --- reset (port of reset_dut) --------------------------------------------------------

async def reset_dut(dut):
    dut.core_aresetn.value = 0
    dut.aresetn.value = 0
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
        await RisingEdge(dut.core_clk)


# --- wait tasks (ports of the SV wait_* / check tasks) --------------------------------

async def wait_frame_done(dut):
    for _ in range(4000):
        await RisingEdge(dut.core_clk)
        if int(dut.frame_count.value) == 1:
            return
    raise AssertionError("CHECK FAILED: timed out waiting for CSI-2 frame completion")


async def wait_parser_short_count(dut, expected):
    for _ in range(4000):
        await RisingEdge(dut.core_clk)
        if int(dut.parser_short_count.value) >= expected:
            return
    raise AssertionError(
        f"CHECK FAILED: timed out waiting for parser short count {expected}, "
        f"saw {int(dut.parser_short_count.value)}")


async def wait_ddr_full_frame(dut):
    for _ in range(4000):
        await RisingEdge(dut.aclk)
        if int(dut.ddr_beats_seen.value) == FRAME_LINES:
            return
    raise AssertionError(
        f"CHECK FAILED: timed out waiting for {FRAME_LINES} DDR beats, "
        f"saw {int(dut.ddr_beats_seen.value)}")


async def check_hdmi_pixels(dut):
    """Port of check_hdmi_pixels: enable HDMI, then verify each active pixel R=G=B=Y in
    order, no more than FRAME_PIXELS active pixels, and no underflow/axis error added."""
    underflow_before = int(dut.hdmi_underflow_count.value)
    axis_error_before = int(dut.hdmi_axis_error_count.value)
    hdmi_seen_count = 0

    await FallingEdge(dut.aclk)
    dut.hdmi_enable.value = 1

    for _ in range(4000):
        await RisingEdge(dut.aclk)
        # SV: `#1;` after posedge, then sample -- read the settled combinational outputs.
        await cocotb.triggers.Timer(1, unit="ns")
        if int(dut.video_de.value) == 1:
            check(hdmi_seen_count < FRAME_PIXELS,
                  f"HDMI emitted more active pixels than expected (saw {hdmi_seen_count})")
            exp = EXPECTED_Y[hdmi_seen_count]
            got_r = int(dut.video_r.value)
            got_g = int(dut.video_g.value)
            got_b = int(dut.video_b.value)
            if (got_r, got_g, got_b) != (exp, exp, exp):
                raise AssertionError(
                    f"CHECK FAILED: HDMI pixel {hdmi_seen_count} "
                    f"got={got_r:02x}{got_g:02x}{got_b:02x} "
                    f"expected={exp:02x}{exp:02x}{exp:02x}")
            hdmi_seen_count += 1
            if hdmi_seen_count == FRAME_PIXELS:
                break

    check(hdmi_seen_count == FRAME_PIXELS,
          f"HDMI delivered all {FRAME_PIXELS} pixels of the reconstructed frame "
          f"(saw {hdmi_seen_count})")
    for _ in range(8):
        await RisingEdge(dut.aclk)
    check(int(dut.hdmi_underflow_count.value) == underflow_before,
          "HDMI active window had no underflow")
    check(int(dut.hdmi_axis_error_count.value) == axis_error_before,
          "HDMI active window had no AXIS sideband error")


# --- the test: replicate the single SV initial run 1:1 --------------------------------

@cocotb.test(timeout_time=5, timeout_unit="ms")
async def e2e_ddrloop_hdmi(dut):
    # SV: core_clk toggles #5 (10 ns), aclk #7 (14 ns).
    start_clock(dut.core_clk, 10.0)
    start_clock(dut.aclk, 14.0)

    await reset_dut(dut)

    drv = ByteDriver(dut)
    await drv.send_short_packet(DT_FS, 0x0000)
    for line_idx in range(FRAME_LINES):
        await drv.send_yuv422_line_4px(line_idx)
    await drv.send_short_packet(DT_FE, 0x0000)

    await wait_frame_done(dut)
    await wait_parser_short_count(dut, 2)

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

    await wait_ddr_full_frame(dut)
    check(int(dut.ddr_beats_seen.value) == FRAME_LINES,
          f"DDR model received {FRAME_LINES} packed beats (one per line)")
    check(int(dut.ddr_tkeep_err.value) == 0,
          "DDR loop saw only full 32-bit beats (TKEEP=0xF)")

    await check_hdmi_pixels(dut)

    dut._log.info("TEST PASSED: tb_csi2_ddrloop_hdmi_e2e")


def test_csi2_ddrloop_hdmi_e2e():
    from runner_support import build_and_test

    here = Path(__file__).resolve().parent

    build_and_test(
        block="csi2_ddrloop_hdmi_e2e",
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
            str(here / "csi2_ddrloop_hdmi_e2e_stubs.sv"),
        ],
        toplevel="csi2_ddrloop_hdmi_e2e_harness",
        test_module="test_csi2_ddrloop_hdmi_e2e",
        test_dir=here,
        parameters={
            "PARSER_IN_WIDTH": PARSER_IN_WIDTH,
            "LINE_PIXELS": LINE_PIXELS,
            "FRAME_LINES": FRAME_LINES,
            "LINE_BYTES": LINE_BYTES,
        },
        engine="verilator",
    )
