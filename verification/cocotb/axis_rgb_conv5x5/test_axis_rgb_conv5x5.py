"""cocotb port of verification/tb/tb_axis_rgb_conv5x5.sv (valid-only pixel family).

8x8 gray frames through the runtime-programmable 5x5 convolution (DoG dual-kernel, Phase A).
Mirrors the DSim TB 1:1:
  (1) passthrough (cfg_en=0) on a uniform frame -> centre pixel (== input);
  (2) identity kernel (centre tap idx12 = 1, shift 0) on uniform -> input;
  (3) 5x5 Gaussian (separable [1,4,6,4,1] outer product, sum 256, shift 8) on uniform ->
      input unchanged;
  (4) Gaussian on horizontal bands 40/200 -> vertical blur (printed row means, no assertion
      in the TB; replicated here to exercise the path).

Same DUT + same stimulus as the DSim TB, so the output sequence -- and therefore the
index-based checks -- match exactly. The DUT uses async active-low reset and 24-bit RGB
pixels driven R=G=B=line_val; out_r is byte lane 2 (out_pixel[23:16]).
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import ClockCycles

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import bringup  # noqa: E402
from lib.pixel_stream import PixelMonitor, PixelStreamDriver  # noqa: E402
from lib.scoreboard import check  # noqa: E402

W = 8  # LINE_PIXELS
H = 8

# 5x5 Gaussian = outer([1,4,6,4,1]), row-major idx 0..24, sum 256, shift 8.
GAUSS5 = [
    1, 4, 6, 4, 1,
    4, 16, 24, 16, 4,
    6, 24, 36, 24, 6,
    4, 16, 24, 16, 4,
    1, 4, 6, 4, 1,
]


def pack_coeffs(coeffs):
    """25 signed 8-bit coeffs packed at cfg_coeffs[idx*8 +: 8] (idx 0 = top-left)."""
    val = 0
    for idx, c in enumerate(coeffs):
        val |= (c & 0xFF) << (idx * 8)
    return val


def gray_frame(line_vals):
    """line_vals: per-row 8-bit value -> flat row-major RGB (R=G=B=val) frame."""
    px = []
    for v in line_vals:
        rgb = (v << 16) | (v << 8) | v
        px.extend([rgb] * W)
    return px


async def _run_frame(dut, clk, drv, mon, pixels, flush=32):
    """Drive one frame, flush the 6-stage pipe + 2-line fill (TB uses repeat(32))."""
    base = len(mon.beats)
    await drv.send_frame(pixels, W)
    await ClockCycles(clk, flush)
    return [(b["pixel"] >> 16) & 0xFF for b in mon.beats[base:]]  # out_r channel


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def passthrough_uniform(dut):
    """(1) cfg_en=0 on uniform 100 -> centre pixel = 100 for interior rows."""
    clk, _ = await bringup(dut, clk="clk", rst="rst_n")
    dut.cfg_coeffs.value = 0
    dut.cfg_shift.value = 0
    dut.cfg_abs.value = 0
    dut.cfg_en.value = 0
    drv = PixelStreamDriver(dut, clk)
    await drv.idle()
    mon = PixelMonitor(dut, clk)
    mon.start()
    r = await _run_frame(dut, clk, drv, mon, gray_frame([100] * H))
    check(len(r) >= (H - 2) * W, f"enough output beats ({len(r)})")
    for i in range(3 * W, min((H - 2) * W, len(r))):
        check(r[i] == 100, f"pass-uni[{i}] (got {r[i]})")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def identity_uniform(dut):
    """(2) identity kernel (centre tap idx12 = 1, shift 0) on uniform 100 -> 100."""
    clk, _ = await bringup(dut, clk="clk", rst="rst_n")
    coeffs = [0] * 25
    coeffs[12] = 1
    dut.cfg_coeffs.value = pack_coeffs(coeffs)
    dut.cfg_shift.value = 0
    dut.cfg_abs.value = 0
    dut.cfg_en.value = 1
    drv = PixelStreamDriver(dut, clk)
    await drv.idle()
    mon = PixelMonitor(dut, clk)
    mon.start()
    r = await _run_frame(dut, clk, drv, mon, gray_frame([100] * H))
    check(len(r) >= (H - 2) * W, f"enough output beats ({len(r)})")
    for i in range(3 * W, min((H - 2) * W, len(r))):
        check(r[i] == 100, f"ident-uni[{i}] (got {r[i]})")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def gaussian_uniform(dut):
    """(3) 5x5 Gaussian (sum 256, shift 8) on uniform 100 -> 100 (unchanged)."""
    clk, _ = await bringup(dut, clk="clk", rst="rst_n")
    dut.cfg_coeffs.value = pack_coeffs(GAUSS5)
    dut.cfg_shift.value = 8
    dut.cfg_abs.value = 0
    dut.cfg_en.value = 1
    drv = PixelStreamDriver(dut, clk)
    await drv.idle()
    mon = PixelMonitor(dut, clk)
    mon.start()
    r = await _run_frame(dut, clk, drv, mon, gray_frame([100] * H))
    check(len(r) >= (H - 3) * W, f"enough output beats ({len(r)})")
    for i in range(3 * W, min((H - 3) * W, len(r))):
        check(r[i] == 100, f"gauss-uni[{i}] (got {r[i]})")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def gaussian_bands(dut):
    """(4) Gaussian on horizontal bands 40/200 -> vertical blur.

    The DSim TB has NO expect_eq here -- it only prints the per-row R-channel means (the
    exact per-row values depend on the line-buffer fill / pipeline boundary, so the TB
    deliberately does not assert them). We replicate the path 1:1 and log the same row
    means, then assert only what the Gaussian (unity-DC, sum 256 >> 8) physically
    guarantees: output stays inside the [40,200] input span, and the interior of the
    high band (rows fully inside 200, window completely filled) blurs back to ~200 while
    the low band produces a clearly darker region -- i.e. the band structure survives.
    """
    clk, _ = await bringup(dut, clk="clk", rst="rst_n")
    dut.cfg_coeffs.value = pack_coeffs(GAUSS5)
    dut.cfg_shift.value = 8
    dut.cfg_abs.value = 0
    dut.cfg_en.value = 1
    drv = PixelStreamDriver(dut, clk)
    await drv.idle()
    mon = PixelMonitor(dut, clk)
    mon.start()

    bands = [40 if rr < 4 else 200 for rr in range(H)]
    r = await _run_frame(dut, clk, drv, mon, gray_frame(bands))

    # Per output row mean over interior columns [2 .. W-3], exactly as the TB prints.
    row_means = []
    for rr in range(H):
        s = n = 0
        for i in range(rr * W + 2, min(rr * W + W - 2, len(r))):
            s += r[i]
            n += 1
        row_means.append(s / n if n else None)
        cocotb.log.info("   out row %d ~= %s", rr, "-" if n == 0 else int(s / n))

    means = [m for m in row_means if m is not None]
    check(len(means) >= 5, f"enough interior rows ({len(means)})")
    # A Gaussian with unity DC gain cannot push any pixel outside the input span.
    for rr, m in enumerate(means):
        check(40 - 1 <= m <= 200 + 1, f"row {rr} within [40,200] span (got {m:.1f})")
    # The band structure must survive the blur: a clearly dark region and a clearly bright
    # region are both present (min well below, max well above the 120 midpoint).
    check(min(means) < 80, f"a dark (~40 band) region survives (min={min(means):.1f})")
    check(max(means) > 160, f"a bright (~200 band) region survives (max={max(means):.1f})")


def test_axis_rgb_conv5x5():
    from runner_support import build_and_test

    build_and_test(
        block="axis_rgb_conv5x5",
        sources=["rtl/img_proc/axis_rgb_conv5x5.sv"],
        toplevel="axis_rgb_conv5x5",
        test_module="test_axis_rgb_conv5x5",
        test_dir=Path(__file__).resolve().parent,
        parameters={"LINE_PIXELS": W, "ENABLE": 1},
    )
