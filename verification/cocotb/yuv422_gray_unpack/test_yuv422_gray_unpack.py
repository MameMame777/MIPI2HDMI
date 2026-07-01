"""cocotb port of verification/tb/tb_yuv422_gray_unpack.sv (custom byte-payload -> pixel).

The DSim TB wraps the DUT in a parameterised ``yuv422_gray_unpack_case`` instantiated six
times, one per YUV422 byte ordering / repair scenario:

  CASE_ID=1  YUYV   seq=0x0  Y_AT_ODD_PHASE=0  LINE_PIXELS=2  LEFT_REPAIR=0
  CASE_ID=2  YVYU   seq=0x1  Y_AT_ODD_PHASE=0  LINE_PIXELS=2  LEFT_REPAIR=0
  CASE_ID=3  UYVY   seq=0x2  Y_AT_ODD_PHASE=1  LINE_PIXELS=2  LEFT_REPAIR=0
  CASE_ID=4  VYUY   seq=0x3  Y_AT_ODD_PHASE=1  LINE_PIXELS=2  LEFT_REPAIR=0
  CASE_ID=5  legacy seq=0xf  Y_AT_ODD_PHASE=1  LINE_PIXELS=2  LEFT_REPAIR=0
  CASE_ID=6  repair seq=0x2  Y_AT_ODD_PHASE=1  LINE_PIXELS=4  LEFT_REPAIR=2

``YUV422_SEQUENCE``, ``Y_AT_ODD_PHASE``, ``LINE_PIXELS`` and ``LEFT_REPAIR_PIXELS`` are
compile-time parameters that change the DUT elaboration, so each case needs its own Verilator
build. Each becomes one ``@cocotb.test()`` (whose body carries that case's payload table,
expected-Y table and geometry) plus one ``def test_case_*()`` pytest entry that builds the DUT
with the matching ``parameters`` and runs only that ``testcase`` (distinct ``build_dir`` per
case).

The ``send_payload`` task, the ``always_ff`` per-pixel checker, and the two final status
checks (``pixel_count == LINE_PIXELS`` and ``sts_pixel_per_line == LINE_PIXELS``) are
translated 1:1 from the TB. Every ``$fatal`` becomes a ``check()``; no check is weakened.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import bringup  # noqa: E402
from lib.scoreboard import check  # noqa: E402

# ---------------------------------------------------------------------------
# Per-case tables, copied verbatim from the TB payload_byte()/expected_y() and the
# instance parameters in tb_yuv422_gray_unpack.
# ---------------------------------------------------------------------------
CASES = {
    1: {  # YUYV
        "seq": 0x0, "y_at_odd": 0, "line_pixels": 2, "left_repair": 0,
        "payload": [0x11, 0x80, 0x22, 0x10],
        "expected_y": [0x11, 0x22],
    },
    2: {  # YVYU
        "seq": 0x1, "y_at_odd": 0, "line_pixels": 2, "left_repair": 0,
        "payload": [0x11, 0x10, 0x22, 0x80],
        "expected_y": [0x11, 0x22],
    },
    3: {  # UYVY
        "seq": 0x2, "y_at_odd": 1, "line_pixels": 2, "left_repair": 0,
        "payload": [0x80, 0x11, 0x10, 0x22, 0x80, 0x33, 0x10, 0x44],
        "expected_y": [0x11, 0x22],
    },
    4: {  # VYUY
        "seq": 0x3, "y_at_odd": 1, "line_pixels": 2, "left_repair": 0,
        "payload": [0x10, 0x11, 0x80, 0x22],
        "expected_y": [0x11, 0x22],
    },
    5: {  # legacy UYVY (seq=0xf)
        "seq": 0xF, "y_at_odd": 1, "line_pixels": 2, "left_repair": 0,
        "payload": [0x80, 0x11, 0x10, 0x22],
        "expected_y": [0x11, 0x22],
    },
    6: {  # UYVY with left-repair
        "seq": 0x2, "y_at_odd": 1, "line_pixels": 4, "left_repair": 2,
        "payload": [0x80, 0x11, 0x10, 0x22, 0x80, 0x33, 0x10, 0x44],
        "expected_y": [0x00, 0x00, 0x33, 0x44],
    },
}


async def _send_payload(dut, clk, data, first, last, sof):
    """Mirror send_payload: assert data+markers+valid for one clock, deassert for one."""
    await RisingEdge(clk)
    dut.in_payload_data.value = data
    dut.in_payload_first.value = 1 if first else 0
    dut.in_payload_last.value = 1 if last else 0
    dut.in_sof.value = 1 if sof else 0
    dut.in_payload_valid.value = 1
    await RisingEdge(clk)
    dut.in_payload_valid.value = 0
    dut.in_payload_first.value = 0
    dut.in_payload_last.value = 0
    dut.in_sof.value = 0


class PixelMonitor:
    """The TB always_ff pixel checker: replicates the per-pixel $fatal assertions."""

    def __init__(self, dut, clk, case_id, expected_y, line_pixels):
        self.dut = dut
        self.clk = clk
        self.case_id = case_id
        self.expected_y = expected_y
        self.line_pixels = line_pixels
        self.pixel_count = 0

    def start(self):
        return cocotb.start_soon(self._run())

    async def _run(self):
        d = self.dut
        while True:
            await RisingEdge(self.clk)
            if int(d.out_pixel_valid.value) != 1:
                continue
            pc = self.pixel_count
            # $fatal: unexpected extra pixel
            check(pc < self.line_pixels,
                  f"case {self.case_id} unexpected extra pixel {pc}")
            expected = self.expected_y[pc]
            got = int(d.out_pixel.value)
            exp_word = (expected << 16) | (expected << 8) | expected
            # $fatal: pixel value mismatch (gray {Y,Y,Y})
            check(got == exp_word,
                  f"case {self.case_id} pixel {pc} mismatch "
                  f"pixel={got:06x} expected={expected:02x}")
            sof = int(d.out_pixel_sof.value)
            eol = int(d.out_pixel_eol.value)
            # $fatal: sof only on the first pixel
            if pc == 0:
                check(sof == 1, f"case {self.case_id} first pixel missing sof")
            else:
                check(sof == 0, f"case {self.case_id} unexpected sof on pixel {pc}")
            # $fatal: eol only on the last pixel of the line
            check((pc == self.line_pixels - 1) == (eol == 1),
                  f"case {self.case_id} eol mismatch pixel={pc} eol={eol}")
            self.pixel_count = pc + 1


async def _run_case(dut, case_id):
    cfg = CASES[case_id]
    line_pixels = cfg["line_pixels"]
    payload = cfg["payload"]
    payload_bytes = line_pixels * 2

    # Initialise every input before releasing reset (mirrors the TB initial block).
    dut.in_sof.value = 0
    dut.in_eof.value = 0
    dut.in_eol.value = 0
    dut.in_payload_data.value = 0
    dut.in_payload_valid.value = 0
    dut.in_payload_first.value = 0
    dut.in_payload_last.value = 0
    dut.in_frame_err.value = 0

    # TB: #5 half-period clock, active-low sync reset held 4 cyc then +2 settle.
    clk, _ = await bringup(dut, clk="core_clk", rst="core_aresetn",
                           period_ns=10.0, cycles=4, post=2)

    mon = PixelMonitor(dut, clk, case_id, cfg["expected_y"], line_pixels)
    mon.start()

    # TB drives exactly PAYLOAD_BYTES = LINE_PIXELS*2 bytes; first/last/sof mirror the TB.
    for idx in range(payload_bytes):
        await _send_payload(dut, clk, payload[idx],
                            first=(idx == 0),
                            last=(idx == payload_bytes - 1),
                            sof=(idx == 0))

    # TB: repeat(4) @(posedge clk) drain, then final checks.
    for _ in range(4):
        await RisingEdge(clk)

    check(mon.pixel_count == line_pixels,
          f"case {case_id} expected {line_pixels} pixels, got {mon.pixel_count}")
    check(int(dut.sts_pixel_per_line.value) == line_pixels,
          f"case {case_id} expected sts_pixel_per_line={line_pixels} "
          f"got {int(dut.sts_pixel_per_line.value)}")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def case_yuyv(dut):
    """CASE_ID=1: YUYV (seq=0x0, Y at even phase)."""
    await _run_case(dut, 1)


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def case_yvyu(dut):
    """CASE_ID=2: YVYU (seq=0x1, Y at even phase)."""
    await _run_case(dut, 2)


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def case_uyvy(dut):
    """CASE_ID=3: UYVY (seq=0x2, Y at odd phase)."""
    await _run_case(dut, 3)


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def case_vyuy(dut):
    """CASE_ID=4: VYUY (seq=0x3, Y at odd phase)."""
    await _run_case(dut, 4)


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def case_legacy_uyvy(dut):
    """CASE_ID=5: legacy UYVY (seq=0xf, Y at odd phase)."""
    await _run_case(dut, 5)


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def case_repair(dut):
    """CASE_ID=6: UYVY with LEFT_REPAIR_PIXELS=2, LINE_PIXELS=4."""
    await _run_case(dut, 6)


def _build_case(case_id, testcase):
    from runner_support import build_and_test
    from cocotb_site import BUILD_DIR

    cfg = CASES[case_id]
    build_and_test(
        block="yuv422_gray_unpack",
        sources=["rtl/img_proc/yuv422_gray_unpack.sv"],
        toplevel="yuv422_gray_unpack",
        test_module="test_yuv422_gray_unpack",
        test_dir=Path(__file__).resolve().parent,
        parameters={
            "YUV422_SEQUENCE": cfg["seq"],
            "Y_AT_ODD_PHASE": cfg["y_at_odd"],
            "LINE_PIXELS": cfg["line_pixels"],
            "LEFT_REPAIR_PIXELS": cfg["left_repair"],
        },
        testcase=testcase,
        build_dir=BUILD_DIR / "sim" / f"yuv422_gray_unpack_case{case_id}",
    )


def test_case_yuyv():
    _build_case(1, "case_yuyv")


def test_case_yvyu():
    _build_case(2, "case_yvyu")


def test_case_uyvy():
    _build_case(3, "case_uyvy")


def test_case_vyuy():
    _build_case(4, "case_vyuy")


def test_case_legacy_uyvy():
    _build_case(5, "case_legacy_uyvy")


def test_case_repair():
    _build_case(6, "case_repair")
