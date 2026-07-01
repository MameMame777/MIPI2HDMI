"""cocotb port of verification/tb/tb_axis_rgb_conv3x3.sv (valid-only pixel family).

8x8 gray frames through the runtime-programmable 3x3 convolution. Checks: (1) passthrough
(cfg_en=0) leaves a uniform frame unchanged; (2) a Gaussian kernel leaves a uniform region
unchanged; (3) |gradient| (cfg_abs) recovers both edge polarities of a Sobel-Y response, so
more "bright edge" rows appear with abs than without. Same DUT + same stimulus as the DSim
TB, so the output sequence -- and therefore these index-based checks -- match exactly.

The DUT uses async active-low reset and 24-bit RGB pixels; out_r/out_g are byte lanes 2/1.
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

W = 8   # LINE_PIXELS
H = 8

# idx0..8 packed at cfg_coeffs[idx*8 +: 8] (signed 8-bit)
GAUSS = [1, 2, 1, 2, 4, 2, 1, 2, 1]
SOBEL_Y = [-1, -2, -1, 0, 0, 0, 1, 2, 1]


def pack_coeffs(coeffs):
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


async def _run_frame(dut, clk, drv, mon, pixels, flush=24):
    base = len(mon.beats)
    await drv.send_frame(pixels, W)
    await ClockCycles(clk, flush)
    return [(b["pixel"] >> 16) & 0xFF for b in mon.beats[base:]]  # out_r channel


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def passthrough_uniform(dut):
    clk, _ = await bringup(dut, clk="clk", rst="rst_n")
    dut.cfg_coeffs.value = pack_coeffs(GAUSS)
    dut.cfg_shift.value = 4
    dut.cfg_abs.value = 0
    dut.cfg_en.value = 0
    drv = PixelStreamDriver(dut, clk)
    await drv.idle()
    mon = PixelMonitor(dut, clk)
    mon.start()
    r = await _run_frame(dut, clk, drv, mon, gray_frame([100] * H))
    check(len(r) >= (H - 1) * W, f"enough output beats ({len(r)})")
    for i in range(3 * W, min((H - 1) * W, len(r))):
        check(r[i] == 100, f"pass-uni[{i}] (got {r[i]})")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def gaussian_uniform(dut):
    clk, _ = await bringup(dut, clk="clk", rst="rst_n")
    dut.cfg_coeffs.value = pack_coeffs(GAUSS)
    dut.cfg_shift.value = 4
    dut.cfg_abs.value = 0
    dut.cfg_en.value = 1
    drv = PixelStreamDriver(dut, clk)
    await drv.idle()
    mon = PixelMonitor(dut, clk)
    mon.start()
    r = await _run_frame(dut, clk, drv, mon, gray_frame([100] * H))
    check(len(r) >= (H - 2) * W, f"enough output beats ({len(r)})")
    for i in range(3 * W, min((H - 2) * W, len(r))):
        check(r[i] == 100, f"gauss-uni[{i}] (got {r[i]})")


def _bright_rows(r):
    cnt = 0
    for rr in range(1, H - 1):
        s = n = 0
        for i in range(rr * W + 2, min(rr * W + W - 2, len(r))):
            s += r[i]
            n += 1
        if n > 0 and s / n > 80:
            cnt += 1
    return cnt


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def sobel_y_abs_magnitude(dut):
    clk, _ = await bringup(dut, clk="clk", rst="rst_n")
    dut.cfg_coeffs.value = pack_coeffs(SOBEL_Y)
    dut.cfg_shift.value = 2
    dut.cfg_en.value = 1
    drv = PixelStreamDriver(dut, clk)
    await drv.idle()
    mon = PixelMonitor(dut, clk)
    mon.start()

    # low-high-low band: rows 3..4 = 200 else 40 -> a rising AND a falling edge
    peak = gray_frame([200 if 3 <= rr <= 4 else 40 for rr in range(H)])

    dut.cfg_abs.value = 0
    r0 = await _run_frame(dut, clk, drv, mon, peak)
    dut.cfg_abs.value = 1
    r1 = await _run_frame(dut, clk, drv, mon, peak)

    bright0 = _bright_rows(r0)
    bright1 = _bright_rows(r1)
    check(bright1 > bright0,
          f"cfg_abs should reveal more edge rows (abs0={bright0}, abs1={bright1})")


def test_axis_rgb_conv3x3():
    from runner_support import build_and_test

    build_and_test(
        block="axis_rgb_conv3x3",
        sources=["rtl/img_proc/axis_rgb_conv3x3.sv"],
        toplevel="axis_rgb_conv3x3",
        test_module="test_axis_rgb_conv3x3",
        test_dir=Path(__file__).resolve().parent,
        parameters={"LINE_PIXELS": W, "ENABLE": 1},
    )
