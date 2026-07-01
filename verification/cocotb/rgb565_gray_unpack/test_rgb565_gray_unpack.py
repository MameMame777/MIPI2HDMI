"""cocotb port of verification/tb/tb_rgb565_gray_unpack.sv (custom byte-payload -> pixel).

The DSim TB wraps the DUT in a parameterised ``rgb565_gray_unpack_case`` instantiated twice:
a little-endian case (``RGB565_BIG_ENDIAN=0``) and a big-endian case
(``RGB565_BIG_ENDIAN=1``), both with ``LINE_PIXELS=4``. Each case sends 8 payload bytes
(one RGB565 word every two bytes) and an ``always_ff`` monitor checks every emitted pixel:
value (gray {Y,Y,Y}), SOF only on pixel 0, EOL only on the last pixel, no extra pixels; then
``pixel_count == LINE_PIXELS`` and ``sts_pixel_per_line == LINE_PIXELS``.

``RGB565_BIG_ENDIAN`` is a compile-time parameter, so each endianness needs its own Verilator
build. This is expressed as two pytest entry points (``test_rgb565_gray_unpack_little`` /
``_big``), each calling ``build_and_test`` with the matching parameter, its own ``build_dir``,
and a ``testcase=`` filter selecting the one cocotb coroutine for that endianness. The
``send_payload`` task, the ``always_ff`` pixel monitor, and the final status checks are
translated 1:1. ``RGB_OUT`` is left at its default 0 (gray-replicate output).

The little/big-endian payload byte tables differ but, once byte-paired by the DUT for the
matching endianness, produce the SAME four RGB565 words (0xf800 red, 0x07e0 green, 0x001f
blue, 0xffff white) and therefore the SAME expected gray sequence [0x4c, 0x95, 0x1c, 0xff] --
exactly as in the TB.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import bringup  # noqa: E402
from lib.scoreboard import check  # noqa: E402

LINE_PIXELS = 4
PAYLOAD_BYTES = LINE_PIXELS * 2

# payload_byte() from the TB, keyed by endianness (both tables encode the same 4 words).
PAYLOAD_LITTLE = [0x00, 0xf8, 0xe0, 0x07, 0x1f, 0x00, 0xff, 0xff]
PAYLOAD_BIG = [0xf8, 0x00, 0x07, 0xe0, 0x00, 0x1f, 0xff, 0xff]

# expected_gray() from the TB (pixels 0..3).
EXPECTED_GRAY = [0x4c, 0x95, 0x1c, 0xff]


async def _send_payload(dut, clk, data, first, last, sof):
    """Mirror send_payload: assert valid+markers for one clock, then deassert for one."""
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
    """The always_ff pixel checker: replicates the per-pixel $fatal assertions."""

    def __init__(self, dut, clk):
        self.dut = dut
        self.clk = clk
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
            check(pc < LINE_PIXELS, f"unexpected extra pixel {pc}")
            expected = EXPECTED_GRAY[pc]
            got = int(d.out_pixel.value)
            exp_word = (expected << 16) | (expected << 8) | expected
            check(got == exp_word,
                  f"pixel {pc} mismatch pixel={got:06x} expected={expected:02x}")
            sof = int(d.out_pixel_sof.value)
            eol = int(d.out_pixel_eol.value)
            if pc == 0:
                check(sof == 1, "first pixel missing sof")
            else:
                check(sof == 0, f"unexpected sof on pixel {pc}")
            check((pc == LINE_PIXELS - 1) == (eol == 1),
                  f"eol mismatch pixel={pc} eol={eol}")
            self.pixel_count = pc + 1


async def _run_case(dut, payload):
    # Initialise all inputs (mirrors the TB initial block) before releasing reset.
    dut.in_sof.value = 0
    dut.in_eof.value = 0
    dut.in_eol.value = 0
    dut.in_payload_data.value = 0
    dut.in_payload_valid.value = 0
    dut.in_payload_first.value = 0
    dut.in_payload_last.value = 0
    dut.in_frame_err.value = 0

    # #5 half-period clock + active-low sync reset; TB holds reset 4 cyc then +2 settle.
    clk, _ = await bringup(dut, clk="core_clk", rst="core_aresetn",
                           period_ns=10.0, cycles=4, post=2)

    mon = PixelMonitor(dut, clk)
    mon.start()

    for idx in range(PAYLOAD_BYTES):
        await _send_payload(dut, clk, payload[idx],
                            first=(idx == 0),
                            last=(idx == PAYLOAD_BYTES - 1),
                            sof=(idx == 0))

    # TB: repeat(4) @(posedge clk) drain, then final checks.
    for _ in range(4):
        await RisingEdge(clk)

    check(mon.pixel_count == LINE_PIXELS,
          f"expected {LINE_PIXELS} pixels, got {mon.pixel_count}")
    check(int(dut.sts_pixel_per_line.value) == LINE_PIXELS,
          f"expected sts_pixel_per_line={LINE_PIXELS} got {int(dut.sts_pixel_per_line.value)}")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def little_endian_case(dut):
    """CASE_ID=1: RGB565_BIG_ENDIAN=0. Run against the little-endian build."""
    await _run_case(dut, PAYLOAD_LITTLE)


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def big_endian_case(dut):
    """CASE_ID=2: RGB565_BIG_ENDIAN=1. Run against the big-endian build."""
    await _run_case(dut, PAYLOAD_BIG)


_COMMON = dict(
    block="rgb565_gray_unpack",
    sources=["rtl/img_proc/rgb565_gray_unpack.sv"],
    toplevel="rgb565_gray_unpack",
    test_module="test_rgb565_gray_unpack",
    test_dir=Path(__file__).resolve().parent,
)


def test_rgb565_gray_unpack_little():
    from runner_support import build_and_test
    import cocotb_site as cs

    build_and_test(
        parameters={"RGB565_BIG_ENDIAN": 0, "RGB_OUT": 0, "LINE_PIXELS": LINE_PIXELS},
        testcase="little_endian_case",
        build_dir=cs.BUILD_DIR / "sim" / "rgb565_gray_unpack_little",
        **_COMMON,
    )


def test_rgb565_gray_unpack_big():
    from runner_support import build_and_test
    import cocotb_site as cs

    build_and_test(
        parameters={"RGB565_BIG_ENDIAN": 1, "RGB_OUT": 0, "LINE_PIXELS": LINE_PIXELS},
        testcase="big_endian_case",
        build_dir=cs.BUILD_DIR / "sim" / "rgb565_gray_unpack_big",
        **_COMMON,
    )
