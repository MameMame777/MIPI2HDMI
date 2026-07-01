"""cocotb port of verification/tb/tb_axis_rgb_dither.sv (valid-only pixel family).

axis_rgb_dither is a point-wise final dither/quantize stage (no neighbourhood): each output
pixel k=(r,c) uses bayer4(r%4, c%4). This mirrors the DSim TB 1:1:

  1) cfg_ctrl=0 -> registered passthrough (out == in).
  2) ordered N=2 (0x09) vs an exact golden over a varied 8x8 frame.
  3) ordered N=1 (0x05) on flat gray -> only {0,255} and BOTH appear (halftone dither).
  4) random N=2 (0x0B) on flat gray -> every output is in the 4-level set.
  5) fixed-latency invariant: off / ordered / random all yield the same beat count and the
     same sof/eof output index.

Same DUT + same stimulus (continuous-valid 8x8 gray frame) as the DSim TB, so the output
sequence -- and therefore these index-based checks -- match exactly. Frames are gray
(R=G=B=v), so out_r = byte lane 2 (bits 23:16).
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

# ---------------------------------------------------------------------------
# Golden models (exact replicas of the TB functions).
# ---------------------------------------------------------------------------
_BAYER4 = [
    0,  8,  2,  10,
    12, 4,  14, 6,
    3,  11, 1,  9,
    15, 7,  13, 5,
]


def bayer4(yy: int, xx: int) -> int:
    return _BAYER4[(yy & 3) * 4 + (xx & 3)]


def gold_ord(v: int, by: int, n: int) -> int:
    """Exact replica of the TB gold_ord (ordered dither_ch, 8-bit)."""
    if n == 0 or n >= 7:
        return v & 0xFF
    drop = 8 - n
    if drop >= 4:
        bias = by << (drop - 4)
    else:
        bias = by >> (4 - drop)
    s = v + bias
    if s > 255:
        s = 255
    q = s & ~((1 << drop) - 1)
    o = q
    o = o | (o >> n)
    o = o | (o >> (2 * n))
    o = o | (o >> (4 * n))
    return o & 0xFF


def level(t: int, n: int) -> int:
    """8-bit replicated level for top-N code t (0..2^N-1). Replica of TB level()."""
    drop = 8 - n
    q = (t << drop) & 255
    o = q
    o = o | (o >> n)
    o = o | (o >> (2 * n))
    o = o | (o >> (4 * n))
    return o & 0xFF


def gray_frame(vals):
    """vals: row-major list of H*W 8-bit values -> RGB (R=G=B) frame."""
    return [(v << 16) | (v << 8) | v for v in vals]


# det[r][c] = (r*37 + c*101 + 11) & 0xFF ; flat[r][c] = 100
DET = [((r * 37 + c * 101 + 11) & 0xFF) for r in range(H) for c in range(W)]
FLAT = [100 for _ in range(H * W)]


async def _run_frame(dut, clk, drv, mon, vals, flush=16):
    """Drive one 8x8 frame (continuous valid, matching the TB drive_frame), return the
    list of captured out beats for this frame (dicts)."""
    base = len(mon.beats)
    await drv.send_frame(gray_frame(vals), W)
    await ClockCycles(clk, flush)
    return mon.beats[base:]


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def passthrough(dut):
    """1) cfg_ctrl=0 -> out == in (registered passthrough, bit-identical)."""
    clk, _ = await bringup(dut, clk="clk", rst="rst_n")
    dut.cfg_ctrl.value = 0x00
    drv = PixelStreamDriver(dut, clk)
    await drv.idle()
    mon = PixelMonitor(dut, clk)
    mon.start()

    beats = await _run_frame(dut, clk, drv, mon, DET)
    r = [(b["pixel"] >> 16) & 0xFF for b in beats]
    check(len(r) >= H * W, f"enough passthrough beats ({len(r)})")
    for k in range(H * W):
        if k < len(r):
            check(r[k] == DET[k], f"pass[{k}]: got {r[k]} exp {DET[k]}")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def ordered_n2_golden(dut):
    """2) ordered N=2 (en=1,mode=0,bits=2 -> 0x09) vs exact golden over a varied frame."""
    clk, _ = await bringup(dut, clk="clk", rst="rst_n")
    dut.cfg_ctrl.value = 0x01 | (2 << 2)  # 0x09
    drv = PixelStreamDriver(dut, clk)
    await drv.idle()
    mon = PixelMonitor(dut, clk)
    mon.start()

    beats = await _run_frame(dut, clk, drv, mon, DET)
    r = [(b["pixel"] >> 16) & 0xFF for b in beats]
    check(len(r) >= H * W, f"enough ord2 beats ({len(r)})")
    for rr in range(H):
        for cc in range(W):
            k = rr * W + cc
            if k < len(r):
                exp = gold_ord(DET[k], bayer4(rr, cc), 2)
                check(r[k] == exp, f"ord2[{k}] (r={rr},c={cc}): got {r[k]} exp {exp}")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def ordered_n1_halftone(dut):
    """3) ordered N=1 (0x05) on flat gray -> only {0,255}, and both appear (dither)."""
    clk, _ = await bringup(dut, clk="clk", rst="rst_n")
    dut.cfg_ctrl.value = 0x01 | (1 << 2)  # 0x05
    drv = PixelStreamDriver(dut, clk)
    await drv.idle()
    mon = PixelMonitor(dut, clk)
    mon.start()

    beats = await _run_frame(dut, clk, drv, mon, FLAT)
    r = [(b["pixel"] >> 16) & 0xFF for b in beats]
    check(len(r) >= H * W, f"enough ord1 beats ({len(r)})")
    n0 = n255 = 0
    for k in range(min(H * W, len(r))):
        if r[k] == 0:
            n0 += 1
        elif r[k] == 255:
            n255 += 1
        else:
            check(False, f"ord1 level[{k}]: got {r[k]} (expect 0/255)")
    check(n0 > 0 and n255 > 0,
          f"ord1 dither: n0={n0} n255={n255} (both must appear)")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def random_n2_levels(dut):
    """4) random N=2 (0x0B) on flat gray -> every output is in the 4-level set."""
    clk, _ = await bringup(dut, clk="clk", rst="rst_n")
    dut.cfg_ctrl.value = 0x01 | (1 << 1) | (2 << 2)  # 0x0B
    drv = PixelStreamDriver(dut, clk)
    await drv.idle()
    mon = PixelMonitor(dut, clk)
    mon.start()

    L = [level(i, 2) for i in range(4)]
    beats = await _run_frame(dut, clk, drv, mon, FLAT)
    r = [(b["pixel"] >> 16) & 0xFF for b in beats]
    check(len(r) >= H * W, f"enough rand2 beats ({len(r)})")
    for k in range(min(H * W, len(r))):
        check(r[k] in L, f"rand2 level[{k}]: got {r[k]} not in {L}")


@cocotb.test(timeout_time=4, timeout_unit="ms")
async def fixed_latency_invariant(dut):
    """5) off / ordered / random -> same beat count and same sof/eof output index."""
    clk, _ = await bringup(dut, clk="clk", rst="rst_n")
    drv = PixelStreamDriver(dut, clk)
    await drv.idle()
    mon = PixelMonitor(dut, clk)
    mon.start()

    async def measure(ctrl):
        dut.cfg_ctrl.value = ctrl
        beats = await _run_frame(dut, clk, drv, mon, FLAT)
        sof_idx = eof_idx = -1
        for i, b in enumerate(beats):
            if b["sof"]:
                sof_idx = i
            if b["eof"]:
                eof_idx = i
        return len(beats), sof_idx, eof_idx

    cnt0, so0, eo0 = await measure(0x00)
    cnt1, so1, eo1 = await measure(0x01 | (2 << 2))  # 0x09
    cnt2, so2, eo2 = await measure(0x0B)

    check(cnt0 == H * W, f"cnt-off: got {cnt0} exp {H * W}")
    check(cnt1 == H * W, f"cnt-ord: got {cnt1} exp {H * W}")
    check(cnt2 == H * W, f"cnt-rnd: got {cnt2} exp {H * W}")
    check(so0 == so1, f"sof-01: {so0} vs {so1}")
    check(so0 == so2, f"sof-02: {so0} vs {so2}")
    check(eo0 == eo1, f"eof-01: {eo0} vs {eo1}")
    check(eo0 == eo2, f"eof-02: {eo0} vs {eo2}")


def test_axis_rgb_dither():
    from runner_support import build_and_test

    build_and_test(
        block="axis_rgb_dither",
        sources=["rtl/img_proc/axis_rgb_dither.sv"],
        toplevel="axis_rgb_dither",
        test_module="test_axis_rgb_dither",
        test_dir=Path(__file__).resolve().parent,
        parameters={"LINE_PIXELS": W, "ENABLE": 1},
    )
