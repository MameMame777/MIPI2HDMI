"""cocotb port of verification/tb/tb_axis_rgb_prefilter.sv (valid-only pixel family).

axis_rgb_prefilter = the PRE spatial-denoise stage. Same window front end as
axis_rgb_conv3x3: the output beat carrying input-pixel (r,c)'s markers holds the 3x3 window
CENTRED at (r-1,c-1), i.e. input rows r-2..r, cols c-2..c. So out[(r,c)] =
filter(input[r-2..r][c-2..c]) for r>=2,c>=2.

Replicated 1:1 from the DSim TB:
  1) passthrough (op0): out[(r,c)] == centre px[r-1][c-1]
  2) median  (op9): per-pixel golden over a varied frame (exercises median9)
  3) gaussian (op8): per-pixel golden
  4) salt-and-pepper: median removes impulses (interior == 120); gaussian keeps a trace (<120)
  5) threshold (op4) runtime level: centre > thr ? white : black  (two levels)
  6) invert (op1): 255 - centre
  7) fixed-latency invariant: same uniform frame under op0/op8/op9 -> identical output
     count + sof/eof positions (no marker skew on mode switch)

Pixels are {v,v,v} so all channels equal; the TB checks out_r = pixel[23:16] (byte lane 2).
The TB indexes output beat k = r*W+c; since the pipeline preserves beat count/order, output
beat k carries input-pixel k's markers -- identical to the DSim monitor's ocnt indexing.
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


# ---- stimulus frames (mirror the TB's initial block) ----
def make_frames():
    uni = [[100 for _ in range(W)] for _ in range(H)]
    det = [[(r * 37 + c * 101 + 7) & 0xFF for c in range(W)] for r in range(H)]
    sp = [[120 for _ in range(W)] for _ in range(H)]
    sp[3][3] = 0
    sp[4][5] = 255
    sp[5][2] = 0
    return uni, det, sp


def flatten(px):
    """H x W byte matrix -> flat row-major list of {v,v,v} 24-bit RGB pixels."""
    out = []
    for r in range(H):
        for c in range(W):
            v = px[r][c] & 0xFF
            out.append((v << 16) | (v << 8) | v)
    return out


# ---- software goldens (transliterated from the TB) ----
def med9sw(x):
    a = sorted(x)
    return a[4]


def gold_med(px, r, c):
    win = []
    for rr in range(r - 2, r + 1):
        for cc in range(c - 2, c + 1):
            win.append(px[rr][cc])
    return med9sw(win)


def gold_gauss(px, r, c):
    corner = px[r - 2][c - 2] + px[r - 2][c] + px[r][c - 2] + px[r][c]
    edgesum = px[r - 2][c - 1] + px[r - 1][c - 2] + px[r - 1][c] + px[r][c - 1]
    cen = px[r - 1][c - 1]
    tot = corner + 2 * edgesum + 4 * cen
    return tot >> 4


async def _drive_frame(dut, clk, drv, mon, px):
    """Mirror the TB drive_frame + ocnt reset: send H*W pixels, flush, return this frame's
    out_r channel plus (sof_idx, eof_idx) as ocnt-relative indices."""
    base = len(mon.beats)
    await drv.send_frame(flatten(px), W)
    await ClockCycles(clk, 24)   # flush pipeline + margin (TB: repeat(24))
    frame = mon.beats[base:]
    out_r = [(b["pixel"] >> 16) & 0xFF for b in frame]
    sof_idx = -1
    eof_idx = -1
    for k, b in enumerate(frame):
        if b["sof"]:
            sof_idx = k
        if b["eof"]:
            eof_idx = k
    return out_r, sof_idx, eof_idx, len(frame)


async def _setup(dut):
    clk, _ = await bringup(dut, clk="clk", rst="rst_n")
    dut.cfg_op.value = 0
    dut.cfg_thresh_level.value = 128
    drv = PixelStreamDriver(dut, clk)
    await drv.idle()
    mon = PixelMonitor(dut, clk)
    mon.start()
    return clk, drv, mon


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def passthrough_centre(dut):
    """op0: out[(r,c)] == centre px[r-1][c-1] (interior)."""
    clk, drv, mon = await _setup(dut)
    _, det, _ = make_frames()
    dut.cfg_op.value = 0
    out_r, _, _, ocnt = await _drive_frame(dut, clk, drv, mon, det)
    for r in range(2, H):
        for c in range(2, W):
            k = r * W + c
            if k < ocnt:
                check(out_r[k] == det[r - 1][c - 1],
                      f"pass-centre[{k}] got {out_r[k]} exp {det[r-1][c-1]}")


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def median_golden(dut):
    """op9: per-pixel median golden over the varied frame (exercises median9)."""
    clk, drv, mon = await _setup(dut)
    _, det, _ = make_frames()
    dut.cfg_op.value = 9
    out_r, _, _, ocnt = await _drive_frame(dut, clk, drv, mon, det)
    for r in range(2, H):
        for c in range(2, W):
            k = r * W + c
            if k < ocnt:
                exp = gold_med(det, r, c)
                check(out_r[k] == exp, f"median[{k}] got {out_r[k]} exp {exp}")


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def gaussian_golden(dut):
    """op8: per-pixel gaussian golden."""
    clk, drv, mon = await _setup(dut)
    _, det, _ = make_frames()
    dut.cfg_op.value = 8
    out_r, _, _, ocnt = await _drive_frame(dut, clk, drv, mon, det)
    for r in range(2, H):
        for c in range(2, W):
            k = r * W + c
            if k < ocnt:
                exp = gold_gauss(det, r, c)
                check(out_r[k] == exp, f"gauss[{k}] got {out_r[k]} exp {exp}")


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def salt_and_pepper(dut):
    """median removes impulses (all interior == 120); gaussian keeps a trace (<120)."""
    clk, drv, mon = await _setup(dut)
    _, _, sp = make_frames()

    dut.cfg_op.value = 9
    out_r, _, _, ocnt = await _drive_frame(dut, clk, drv, mon, sp)
    for r in range(2, H):
        for c in range(2, W):
            k = r * W + c
            if k < ocnt:
                check(out_r[k] == 120, f"median-sp[{k}] got {out_r[k]} exp 120")

    # gaussian does NOT fully remove an impulse: output centred on px[3][3]=0 is out[(4,4)]
    # and must be < 120 (pulled down) -> gaussian keeps a trace.
    dut.cfg_op.value = 8
    out_r, _, _, ocnt = await _drive_frame(dut, clk, drv, mon, sp)
    k = 4 * W + 4
    if k < ocnt:
        check(out_r[k] < 120,
              f"gauss-sp: expected <120 at impulse, got {out_r[k]}")


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def threshold_runtime(dut):
    """op4 runtime level: centre px[r-1][c-1] > thr ? white(255) : black(0), at thr 128 & 40."""
    clk, drv, mon = await _setup(dut)
    _, det, _ = make_frames()

    dut.cfg_op.value = 4
    dut.cfg_thresh_level.value = 128
    out_r, _, _, ocnt = await _drive_frame(dut, clk, drv, mon, det)
    for r in range(2, H):
        for c in range(2, W):
            k = r * W + c
            if k < ocnt:
                exp = 255 if det[r - 1][c - 1] > 128 else 0
                check(out_r[k] == exp, f"thr128[{k}] got {out_r[k]} exp {exp}")

    dut.cfg_op.value = 4
    dut.cfg_thresh_level.value = 40
    out_r, _, _, ocnt = await _drive_frame(dut, clk, drv, mon, det)
    for r in range(2, H):
        for c in range(2, W):
            k = r * W + c
            if k < ocnt:
                exp = 255 if det[r - 1][c - 1] > 40 else 0
                check(out_r[k] == exp, f"thr40[{k}] got {out_r[k]} exp {exp}")


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def invert(dut):
    """op1: 255 - centre."""
    clk, drv, mon = await _setup(dut)
    _, det, _ = make_frames()
    dut.cfg_op.value = 1
    out_r, _, _, ocnt = await _drive_frame(dut, clk, drv, mon, det)
    for r in range(2, H):
        for c in range(2, W):
            k = r * W + c
            if k < ocnt:
                exp = 255 - det[r - 1][c - 1]
                check(out_r[k] == exp, f"invert[{k}] got {out_r[k]} exp {exp}")


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def fixed_latency_invariant(dut):
    """Same uniform frame under op0/op8/op9 -> identical output count + sof/eof positions
    (no marker skew on mode switch)."""
    clk, drv, mon = await _setup(dut)
    uni, _, _ = make_frames()

    dut.cfg_op.value = 0
    _, sof0, eof0, cnt0 = await _drive_frame(dut, clk, drv, mon, uni)
    check(cnt0 == H * W, f"cnt-op0 got {cnt0} exp {H*W}")

    dut.cfg_op.value = 8
    _, sof8, eof8, cnt8 = await _drive_frame(dut, clk, drv, mon, uni)
    check(cnt8 == H * W, f"cnt-op8 got {cnt8} exp {H*W}")

    dut.cfg_op.value = 9
    _, sof9, eof9, cnt9 = await _drive_frame(dut, clk, drv, mon, uni)
    check(cnt9 == H * W, f"cnt-op9 got {cnt9} exp {H*W}")

    check(sof0 == sof8, f"sof-align-08 got {sof8} exp {sof0}")
    check(sof0 == sof9, f"sof-align-09 got {sof9} exp {sof0}")
    check(eof0 == eof8, f"eof-align-08 got {eof8} exp {eof0}")
    check(eof0 == eof9, f"eof-align-09 got {eof9} exp {eof0}")


def test_axis_rgb_prefilter():
    from runner_support import build_and_test

    build_and_test(
        block="axis_rgb_prefilter",
        sources=[
            "rtl/img_proc/median9.sv",
            "rtl/img_proc/axis_rgb_prefilter.sv",
        ],
        toplevel="axis_rgb_prefilter",
        test_module="test_axis_rgb_prefilter",
        test_dir=Path(__file__).resolve().parent,
        parameters={"LINE_PIXELS": W, "ENABLE": 1},
        engine="verilator",
    )
