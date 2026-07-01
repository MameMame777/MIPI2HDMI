"""cocotb port of verification/tb/tb_mipi_to_hdmi_direct_minimal.sv
(E2E MIPI CSI-2 -> HDMI "direct minimal" pipeline).

The DSim TB wires the full receive->display chain and drives a single 1-line YUV422
frame (FS short / one 4-byte YUV422 long / FE short) through it, then verifies every
stage's status counters, the two grayscale pixels at the YUV unpacker, the two AXIS
beats at the bridge, and the two HDMI active-window pixels.

No wrapper module exists in the RTL (the TB itself is the top with 8 DUT instances), so
-- exactly like the e2e_vdma_stacking port -- the 8-DUT wiring is emitted as
``mipi_direct_minimal_harness`` (in mipi_to_hdmi_direct_minimal_stubs.sv) that contains
ONLY the DUT instances (no ``initial``, no clock). cocotb owns the two clocks
(core_clk #5 = 10 ns, pix_clk #7 = 14 ns), the reset sequence, the byte-beat stimulus and
every check. There are NO Xilinx primitives in this pipeline, so it runs on Verilator.

The two SV ``always_ff`` checker blocks (yuv_seen_count / axis_seen_count) become the
``YuvChecker`` / ``AxisChecker`` monitor coroutines; ``check_condition`` -> ``check``;
``$fatal`` -> ``AssertionError``; the ``#2ms`` watchdog -> the @cocotb.test timeout.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import FallingEdge, RisingEdge, Timer

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import start_clock  # noqa: E402
from lib.scoreboard import check  # noqa: E402

# --- TB localparams (1:1) -------------------------------------------------------------
DT_FS = 0x00
DT_FE = 0x01
DT_YUV422 = 0x1E
EXPECTED_Y0 = 0x24
EXPECTED_Y1 = 0xA8


# --- SV pure functions ported 1:1 -----------------------------------------------------

def _calc_ecc6(data: int) -> int:
    """Port of calc_ecc6(24-bit) -> 6-bit."""
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


def _make_ecc(di: int, wc: int) -> int:
    """Port of make_ecc: {2'b00, calc_ecc6({wc, di})}."""
    return _calc_ecc6(((wc & 0xFFFF) << 8) | (di & 0xFF)) & 0x3F


def _crc_update_byte(crc_in: int, data: int) -> int:
    """Port of crc_update_byte (CRC-16 poly 0x8408, reflected)."""
    crc = crc_in & 0xFFFF
    for bit_idx in range(8):
        feedback = (crc & 1) ^ ((data >> bit_idx) & 1)
        crc >>= 1
        if feedback:
            crc ^= 0x8408
    return crc & 0xFFFF


def _crc4(p0: int, p1: int, p2: int, p3: int) -> int:
    """Port of crc4: seed 0xFFFF then update four payload bytes."""
    crc = 0xFFFF
    for p in (p0, p1, p2, p3):
        crc = _crc_update_byte(crc, p)
    return crc & 0xFFFF


# --- byte-beat driver: port of the TB drive_beat / drive_idle tasks -------------------
# The TB drives on negedge core_clk and holds s_byte_valid HIGH across consecutive beats
# within a packet (valid only drops in drive_idle). This differs from lib.byte_beat's
# 2-cycle cadence, so the driver is reproduced here 1:1.

class ByteDriver:
    def __init__(self, dut):
        self.dut = dut
        self.clk = dut.core_clk

    async def beat(self, byte0: int, byte1: int, sop: bool, eop: bool) -> None:
        await FallingEdge(self.clk)
        self.dut.s_byte_data.value = ((byte1 & 0xFF) << 8) | (byte0 & 0xFF)
        self.dut.s_byte_keep.value = 0b11
        self.dut.s_byte_valid.value = 1
        self.dut.s_byte_sop.value = 1 if sop else 0
        self.dut.s_byte_eop.value = 1 if eop else 0

    async def idle(self, cycles: int) -> None:
        await FallingEdge(self.clk)
        self.dut.s_byte_valid.value = 0
        self.dut.s_byte_keep.value = 0
        self.dut.s_byte_sop.value = 0
        self.dut.s_byte_eop.value = 0
        self.dut.s_byte_data.value = 0
        for _ in range(cycles):
            await FallingEdge(self.clk)

    async def send_short_packet(self, dt: int, data_field: int) -> None:
        di = dt & 0x3F  # {2'b00, dt}
        ecc = _make_ecc(di, data_field)
        await self.beat(di, data_field & 0xFF, True, False)
        await self.beat((data_field >> 8) & 0xFF, ecc, False, True)
        await self.idle(2)

    async def send_yuv422_long_packet(self, u0: int, y0: int, v0: int, y1: int) -> None:
        di = DT_YUV422 & 0x3F
        wc = 4
        ecc = _make_ecc(di, wc)
        crc = _crc4(u0, y0, v0, y1)
        await self.beat(di, wc & 0xFF, True, False)
        await self.beat((wc >> 8) & 0xFF, ecc, False, False)
        await self.beat(u0, y0, False, False)
        await self.beat(v0, y1, False, False)
        await self.beat(crc & 0xFF, (crc >> 8) & 0xFF, False, True)
        await self.idle(2)


# --- SV always_ff checkers -> monitor coroutines --------------------------------------

class YuvChecker:
    """Port of the core_clk always_ff yuv_seen_count block: checks the two grayscale
    pixels' values and SOF/EOL markers as they appear at the unpacker output."""

    def __init__(self, dut):
        self.dut = dut
        self.count = 0

    def start(self):
        return cocotb.start_soon(self._run())

    async def _run(self):
        d = self.dut
        while True:
            await RisingEdge(d.core_clk)
            if int(d.core_aresetn.value) == 0:
                self.count = 0
                continue
            if int(d.yuv_pixel_valid.value) != 1:
                continue
            pix = int(d.yuv_pixel.value)
            if self.count == 0:
                exp = (EXPECTED_Y0 << 16) | (EXPECTED_Y0 << 8) | EXPECTED_Y0
                check(pix == exp, "YUV first grayscale pixel value")
                check(int(d.yuv_pixel_sof.value) == 1, "YUV first pixel carries SOF")
                check(int(d.yuv_pixel_eol.value) == 0, "YUV first pixel does not carry EOL")
            elif self.count == 1:
                exp = (EXPECTED_Y1 << 16) | (EXPECTED_Y1 << 8) | EXPECTED_Y1
                check(pix == exp, "YUV second grayscale pixel value")
                check(int(d.yuv_pixel_sof.value) == 0, "YUV second pixel does not carry SOF")
                check(int(d.yuv_pixel_eol.value) == 1, "YUV second pixel carries EOL")
            else:
                raise AssertionError(
                    f"CHECK FAILED: unexpected extra YUV pixel {self.count}")
            check(int(d.yuv_pixel_err.value) == 0, "YUV pixel has no frame error")
            self.count += 1


class AxisChecker:
    """Port of the pix_clk always_ff axis_seen_count block: checks the two AXIS beats'
    data and SOF(tuser[0])/TLAST markers on each accepted transfer."""

    def __init__(self, dut):
        self.dut = dut
        self.count = 0

    def start(self):
        return cocotb.start_soon(self._run())

    async def _run(self):
        d = self.dut
        while True:
            await RisingEdge(d.pix_clk)
            if int(d.pix_aresetn.value) == 0:
                self.count = 0
                continue
            if not (int(d.axis_tvalid.value) == 1 and int(d.axis_tready.value) == 1):
                continue
            tdata = int(d.axis_tdata.value)
            tuser0 = int(d.axis_tuser.value) & 0x1
            tlast = int(d.axis_tlast.value)
            if self.count == 0:
                exp = (EXPECTED_Y0 << 16) | (EXPECTED_Y0 << 8) | EXPECTED_Y0
                check(tdata == exp, "AXIS first pixel value")
                check(tuser0 == 1, "AXIS first pixel carries SOF")
                check(tlast == 0, "AXIS first pixel does not carry TLAST")
            elif self.count == 1:
                exp = (EXPECTED_Y1 << 16) | (EXPECTED_Y1 << 8) | EXPECTED_Y1
                check(tdata == exp, "AXIS second pixel value")
                check(tuser0 == 0, "AXIS second pixel does not carry SOF")
                check(tlast == 1, "AXIS second pixel carries TLAST")
            self.count += 1


# --- reset + wait helpers (ports of the SV tasks) -------------------------------------

async def reset_dut(dut):
    """Port of reset_dut()."""
    dut.core_aresetn.value = 0
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
        await RisingEdge(dut.pix_clk)
    dut.pix_aresetn.value = 1
    for _ in range(4):
        await RisingEdge(dut.core_clk)


async def wait_core_count(dut, checker, expected):
    """Port of wait_core_count: up to 300 core_clk cycles."""
    for _ in range(300):
        await RisingEdge(dut.core_clk)
        if checker.count == expected:
            return
    raise AssertionError(
        f"Timed out waiting for {expected} YUV pixels, saw {checker.count}")


async def wait_frame_done(dut):
    """Port of wait_frame_done: frame_count == 1 within 300 core_clk cycles."""
    for _ in range(300):
        await RisingEdge(dut.core_clk)
        if int(dut.frame_count.value) == 1:
            return
    raise AssertionError("Timed out waiting for CSI-2 frame completion")


async def wait_axis_ready(dut):
    """Port of wait_axis_ready: axis_tvalid && axis_tuser[0] sampled #1 after pix_clk."""
    for _ in range(300):
        await RisingEdge(dut.pix_clk)
        await Timer(1, unit="ns")
        if int(dut.axis_tvalid.value) == 1 and (int(dut.axis_tuser.value) & 0x1):
            return
    raise AssertionError("Timed out waiting for direct AXIS SOF pixel")


async def check_hdmi_pixels(dut):
    """Port of check_hdmi_pixels: enable HDMI, watch the active window for exactly the
    two expected grayscale pixels, confirm no underflow / AXIS sideband error."""
    expected_y = [EXPECTED_Y0, EXPECTED_Y1]
    underflow_before = int(dut.hdmi_underflow_count.value)
    axis_error_before = int(dut.hdmi_axis_error_count.value)
    seen = 0

    await FallingEdge(dut.pix_clk)
    dut.hdmi_enable.value = 1
    for _ in range(80):
        await RisingEdge(dut.pix_clk)
        await Timer(1, unit="ns")
        if int(dut.video_de.value) == 1:
            check(seen < 2, "HDMI emitted more active pixels than expected")
            got = (int(dut.video_r.value) << 16) | (int(dut.video_g.value) << 8) | int(dut.video_b.value)
            exp = (expected_y[seen] << 16) | (expected_y[seen] << 8) | expected_y[seen]
            if got != exp:
                raise AssertionError(
                    f"CHECK FAILED: HDMI pixel {seen} got={got:06x} expected={exp:06x}")
            seen += 1
            if seen == 2:
                dut.hdmi_enable.value = 0
                break

    check(seen == 2, "HDMI consumed both minimal-line pixels")
    for _ in range(4):
        await RisingEdge(dut.pix_clk)
    check(int(dut.hdmi_underflow_count.value) == underflow_before,
          "HDMI checked active window had no underflow")
    check(int(dut.hdmi_axis_error_count.value) == axis_error_before,
          "HDMI checked active window had no AXIS sideband error")


# --- the test: replicate the single SV initial run 1:1 --------------------------------

@cocotb.test(timeout_time=5, timeout_unit="ms")
async def mipi_to_hdmi_direct_minimal(dut):
    # Two async clocks, exactly the SV periods: core_clk #5 = 10 ns, pix_clk #7 = 14 ns.
    start_clock(dut.core_clk, 10.0)
    start_clock(dut.pix_clk, 14.0)

    await reset_dut(dut)

    yuv_chk = YuvChecker(dut)
    yuv_chk.start()
    axis_chk = AxisChecker(dut)
    axis_chk.start()

    drv = ByteDriver(dut)

    await drv.send_short_packet(DT_FS, 0x0000)
    await drv.send_yuv422_long_packet(0x80, EXPECTED_Y0, 0x10, EXPECTED_Y1)
    await drv.send_short_packet(DT_FE, 0x0000)

    await wait_core_count(dut, yuv_chk, 2)
    await wait_frame_done(dut)

    check(int(dut.parser_short_count.value) == 2, "parser saw FS and FE short packets")
    check(int(dut.parser_long_count.value) == 1, "parser saw one YUV422 long packet")
    check(int(dut.parser_trunc_count.value) == 0, "parser saw no truncation")
    check(int(dut.ecc_uncorr_count.value) == 0, "header ECC has no uncorrectable errors")
    check(int(dut.crc_ok_count.value) == 1, "payload CRC matched once")
    check(int(dut.crc_err_count.value) == 0, "payload CRC has no errors")
    check(int(dut.filter_drop_vc_count.value) == 0, "filter dropped no VC packets")
    check(int(dut.filter_drop_dt_count.value) == 0, "filter dropped no DT packets")
    check(int(dut.frame_sync_err_count.value) == 0, "frame state has no sync errors")
    check(int(dut.line_count.value) == 1, "frame state counted one line")
    check(int(dut.last_frame_lines.value) == 1, "frame state ended one-line frame")
    check(int(dut.yuv_pixel_per_line.value) == 2, "YUV unpacker counted two pixels per line")
    check(int(dut.bridge_overflow_count.value) == 0, "direct bridge did not overflow")

    await wait_axis_ready(dut)
    await check_hdmi_pixels(dut)
    check(axis_chk.count == 2, "AXIS bridge delivered two pixels to HDMI")

    dut._log.info("TEST PASSED: tb_mipi_to_hdmi_direct_minimal")


def test_mipi_to_hdmi_direct_minimal():
    from runner_support import build_and_test

    here = Path(__file__).resolve().parent
    build_and_test(
        block="mipi_to_hdmi_direct_minimal",
        sources=[
            "rtl/mipi_rx/csi2_packet_parser.sv",
            "rtl/mipi_rx/csi2_header_ecc.sv",
            "rtl/mipi_rx/csi2_payload_crc.sv",
            "rtl/mipi_rx/csi2_vcdt_filter.sv",
            "rtl/mipi_rx/csi2_frame_state.sv",
            "rtl/img_proc/yuv422_gray_unpack.sv",
            "rtl/img_proc/rgb565_gray_unpack.sv",
            "rtl/mipi_rx/axis_video_bridge.sv",
            "rtl/hdmi/hdmi_output.sv",
            str(here / "mipi_to_hdmi_direct_minimal_stubs.sv"),
        ],
        toplevel="mipi_direct_minimal_harness",
        test_module="test_mipi_to_hdmi_direct_minimal",
        test_dir=here,
        parameters={},
        engine="verilator",
    )
