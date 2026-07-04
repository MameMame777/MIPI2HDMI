"""cocotb port of verification/tb/tb_axis_rgb_cascade.sv (mixed 3-stage cascade harness).

The DSim TB wires four DUTs into a cascade:

    in -> S1 (axis_rgb_conv5x5, general 5x5 Gaussian)
       -> S2 (axis_rgb_conv5x5_sep, separable 5x5 Gaussian)
       -> S3 (axis_rgb_conv5x5_sep, separable 5x5 Gaussian)
    taps: t1 = after S1 (eff 5x5), t2 = after S2 (eff 9x9), t3 = after S3 (eff 13x13)
    t1 (A, leads) + t3 (B, lags) -> axis_rgb_dog_combine (mode 2 = DoG)  => multi-scale DoG

cocotb needs a single HDL toplevel, so the four-DUT wiring is emitted as a tiny wrapper
module (``cascade_harness``) at build time -- it contains ONLY the four instances (no
``initial``, no clock), exposing every config knob + stimulus + the four tap bundles. This
is a build tool-input file, not a hand-maintained RTL source; the three real RTL sources
from axis_rgb_cascade.f are the DUTs under test.

Scenarios replicate the DSim ``initial`` checks 1:1:
  (1) uniform 100 -> every tap stays 100;
  (2) horizontal bands 40/200 -> cascading WIDENS the blur (transition width t1 < t3,
      t1 <= t2), width = count of rows whose center mean is in 70..170;
  (3) runtime BYPASS: reload S3 = identity kernel -> S3 stops blurring -> t3's effective
      size shrinks 13x13 -> 9x9, so t3's transition width drops (< 8) toward t2's (|d|<=3).

16x8 frame. W = LINE_PIXELS = 8, H = 16. Same DUTs + same stimulus + same index-based
checks as the DSim TB, so the row-mean sequence matches exactly.
"""
from __future__ import annotations

import os
import random
import sys
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "img_file_uvm"))
import golden as G  # noqa: E402
from lib.clkreset import bringup  # noqa: E402
from lib.scoreboard import check  # noqa: E402

W = 8    # LINE_PIXELS
H = 16

# S1 general 5x5 Gaussian (25 taps, row-major). Symmetric -> packing direction irrelevant.
C1 = [1, 4, 6, 4, 1,
      4, 16, 24, 16, 4,
      6, 24, 36, 24, 6,
      4, 16, 24, 16, 4,
      1, 4, 6, 4, 1]
SH1 = 8
# S2/S3 separable 5x5 Gaussian taps (h == v). Symmetric.
G5 = [1, 4, 6, 4, 1]
IDENT5 = [0, 0, 1, 0, 0]   # identity kernel used for the runtime-bypass test


def pack(coeffs):
    """Pack signed-8b taps LSB-first (matches RTL cfg[idx*8 +: 8])."""
    val = 0
    for idx, c in enumerate(coeffs):
        val |= (c & 0xFF) << (idx * 8)
    return val


# ---------------------------------------------------------------------------
# Tap capture: one monitor coroutine per tap valid, storing the R (bits[23:16]) channel,
# mirroring the DSim always_ff logger (b1/b2/b3/bd arrays, capped at H*W).
# ---------------------------------------------------------------------------
class TapCapture:
    def __init__(self, dut, clk):
        self.dut = dut
        self.clk = clk
        self.reset()

    def reset(self):
        self.b1, self.b2, self.b3, self.bd = [], [], [], []

    def start(self):
        cocotb.start_soon(self._run())

    async def _run(self):
        d = self.dut
        while True:
            await RisingEdge(self.clk)
            if int(d.t1v.value):
                if len(self.b1) < H * W:
                    self.b1.append((int(d.t1.value) >> 16) & 0xFF)
            if int(d.t2v.value):
                if len(self.b2) < H * W:
                    self.b2.append((int(d.t2.value) >> 16) & 0xFF)
            if int(d.t3v.value):
                if len(self.b3) < H * W:
                    self.b3.append((int(d.t3.value) >> 16) & 0xFF)
            if int(d.dgv.value):
                if len(self.bd) < H * W:
                    self.bd.append((int(d.dg.value) >> 16) & 0xFF)


# ---------------------------------------------------------------------------
# rmean / twidth: identical to the DSim functions. rmean averages interior cols [2 .. W-3]
# of a row; twidth counts rows whose mean is "in transition" (70..170).
# ---------------------------------------------------------------------------
def rmean(buf, rr):
    lo = rr * W + 2
    hi = rr * W + W - 2
    if hi > len(buf):
        return -1
    s = sum(buf[lo:hi])
    n = hi - lo
    return (s // n) if n > 0 else -1


def twidth(buf):
    n = 0
    for rr in range(H):
        m = rmean(buf, rr)
        if 70 <= m <= 170:
            n += 1
    return n


async def _cfg_common(dut):
    """S1 general Gaussian; S2 separable Gaussian. (S3 set per-test.)"""
    dut.c1.value = pack(C1)
    dut.sh1.value = SH1
    dut.h2.value = pack(G5)
    dut.v2.value = pack(G5)
    dut.hs2.value = 4
    dut.vs2.value = 4


async def _idle_in(dut):
    dut.in_valid.value = 0
    dut.in_sof.value = 0
    dut.in_eol.value = 0
    dut.in_eof.value = 0
    dut.in_err.value = 0
    dut.in_pixel.value = 0


async def _drive_frame(dut, clk, line_vals, flush=64):
    """Mirror the DSim drive_frame task: row-major raster of gray(lv[r]) pixels with
    sof/eol/eof framing, then 64 idle cycles to flush the pipelines."""
    for r in range(H):
        for c in range(W):
            await RisingEdge(clk)
            v = line_vals[r]
            dut.in_valid.value = 1
            dut.in_pixel.value = (v << 16) | (v << 8) | v
            dut.in_sof.value = 1 if (r == 0 and c == 0) else 0
            dut.in_eol.value = 1 if (c == W - 1) else 0
            dut.in_eof.value = 1 if (r == H - 1 and c == W - 1) else 0
    await RisingEdge(clk)
    await _idle_in(dut)
    for _ in range(flush):
        await RisingEdge(clk)


UNI = [100] * H
BANDS = [40 if i < 8 else 200 for i in range(H)]


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def uniform_taps_100(dut):
    """DSim check 1: uniform 100 input -> every tap 100 (t1 exact over the settled interior,
    t3 within 98..102 at the middle row)."""
    clk, _ = await bringup(dut, clk="clk", rst="rst_n")
    await _cfg_common(dut)
    dut.h3.value = pack(G5)
    dut.v3.value = pack(G5)
    dut.hs3.value = 4
    dut.vs3.value = 4
    await _idle_in(dut)
    cap = TapCapture(dut, clk)
    cap.start()

    await _drive_frame(dut, clk, UNI)   # warm up (fills line buffers)
    cap.reset()
    await _drive_frame(dut, clk, UNI)   # measured frame

    check(len(cap.b1) >= (H - 4) * W, f"enough t1 beats ({len(cap.b1)})")
    ok1 = all(cap.b1[i] == 100 for i in range(5 * W, min((H - 4) * W, len(cap.b1))))
    check(ok1, "uni-t1 (settled interior of t1 must be exactly 100)")
    check(len(cap.b3) > 8 * W, f"enough t3 beats ({len(cap.b3)})")
    check(98 <= cap.b3[8 * W] <= 102, f"uni-t3 (b3[8W]={cap.b3[8 * W]})")


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def cascade_widens_blur(dut):
    """DSim check 2: bands 40/200 -> cascading widens the blur: twidth(t3) > twidth(t1)
    and twidth(t2) >= twidth(t1)."""
    clk, _ = await bringup(dut, clk="clk", rst="rst_n")
    await _cfg_common(dut)
    dut.h3.value = pack(G5)
    dut.v3.value = pack(G5)
    dut.hs3.value = 4
    dut.vs3.value = 4
    await _idle_in(dut)
    cap = TapCapture(dut, clk)
    cap.start()

    await _drive_frame(dut, clk, UNI)     # warm up
    cap.reset()
    await _drive_frame(dut, clk, BANDS)   # measured frame

    w1, w2, w3 = twidth(cap.b1), twidth(cap.b2), twidth(cap.b3)
    dut._log.info(f"[cascade bands] transition widths: t1={w1} t2={w2} t3={w3}")
    check(w3 > w1, f"widen t1<t3 (t1={w1}, t3={w3})")
    check(w2 >= w1, f"widen t1<=t2 (t1={w1}, t2={w2})")


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def runtime_bypass_s3(dut):
    """DSim check 3: reload S3 = identity kernel at runtime -> S3 stops blurring, so t3's
    effective size shrinks 13x13 -> 9x9 and its transition width drops (< 8) toward t2's
    (|twidth(t3)-twidth(t2)| <= 3)."""
    clk, _ = await bringup(dut, clk="clk", rst="rst_n")
    await _cfg_common(dut)
    # S3 = identity (bypass): h3=v3={0,0,1,0,0}, shifts 0.
    dut.h3.value = pack(IDENT5)
    dut.v3.value = pack(IDENT5)
    dut.hs3.value = 0
    dut.vs3.value = 0
    await _idle_in(dut)
    cap = TapCapture(dut, clk)
    cap.start()

    await _drive_frame(dut, clk, BANDS)   # warm up S3 line buffers with the new kernel
    cap.reset()
    await _drive_frame(dut, clk, BANDS)   # measure

    w2, w3 = twidth(cap.b2), twidth(cap.b3)
    dut._log.info(f"[bypass S3=identity] transition widths: t2={w2} t3={w3}")
    d = abs(w3 - w2)
    check(w3 < 8 and d <= 3, f"bypass shrinks t3 (t3={w3}, t2={w2}, |d|={d})")


# ---------------------------------------------------------------------------
# additive: bit-exact check of ALL FOUR taps against the composed goldens. The property tests
# above prove blur ordering; this proves exact values. Each tap composes the verified building-
# block goldens: t1 = conv5x5(C1), t2 = conv5x5_sep(t1), t3 = conv5x5_sep(t2), dg = DoG(t1,t3).
# t3 lags t1 by two sep pipelines (~16 cyc), so the DoG FIFO pairs t1[k] with t3[k] cleanly.
# Stream two random frames and check the SECOND (steady state; the cold-start transient and the
# deep line-buffer fill are confined to frame 1), exactly like the DoG chain test.
# ---------------------------------------------------------------------------
class _FullCap:
    """Capture full 24-bit pixels for all four taps on each tap's valid."""

    def __init__(self, dut, clk):
        self.dut, self.clk = dut, clk
        self.t1, self.t2, self.t3, self.dg = [], [], [], []

    def start(self):
        cocotb.start_soon(self._run())

    async def _run(self):
        d = self.dut
        while True:
            await RisingEdge(self.clk)
            if int(d.t1v.value):
                self.t1.append(int(d.t1.value))
            if int(d.t2v.value):
                self.t2.append(int(d.t2.value))
            if int(d.t3v.value):
                self.t3.append(int(d.t3.value))
            if int(d.dgv.value):
                self.dg.append(int(d.dg.value))


async def _drive_pixels(dut, clk, pixels, flush=96):
    n = len(pixels)
    for i, px in enumerate(pixels):
        await RisingEdge(clk)
        dut.in_valid.value = 1
        dut.in_pixel.value = px
        dut.in_sof.value = 1 if i == 0 else 0
        dut.in_eol.value = 1 if (i % W) == W - 1 else 0
        dut.in_eof.value = 1 if i == n - 1 else 0
    await RisingEdge(clk)
    await _idle_in(dut)
    for _ in range(flush):
        await RisingEdge(clk)


@cocotb.test(timeout_time=6, timeout_unit="ms")
async def cascade_bitexact(dut):
    clk, _ = await bringup(dut, clk="clk", rst="rst_n")
    await _cfg_common(dut)
    dut.h3.value = pack(G5)
    dut.v3.value = pack(G5)
    dut.hs3.value = 4
    dut.vs3.value = 4
    await _idle_in(dut)
    cap = _FullCap(dut, clk)
    cap.start()

    rng = random.Random((int(os.environ.get("COCOTB_SEED", "1"), 0) << 5) ^ 0xCA5CADE)
    fsz = W * H
    stream = [rng.randrange(0x1000000) for _ in range(2 * fsz)]
    await _drive_pixels(dut, clk, stream)

    t1 = G.conv_golden(stream, W, C1, SH1, 0, 1, 5)
    t2 = G.conv5x5_sep_golden(t1, W, G5, G5, 4, 4)
    t3 = G.conv5x5_sep_golden(t2, W, G5, G5, 4, 4)
    dg = G.dog_combine_golden(t1, t3, 2, 1, 1, 0, 128)

    for nm, got, exp in (("t1", cap.t1, t1), ("t2", cap.t2, t2),
                         ("t3", cap.t3, t3), ("dg", cap.dg, dg)):
        check(len(got) >= 2 * fsz, f"{nm}: captured {len(got)} < {2 * fsz} beats")
        mism = [(fsz + i, f"{got[fsz + i]:06x}", f"{exp[fsz + i]:06x}")
                for i in range(fsz) if got[fsz + i] != exp[fsz + i]]
        check(not mism, f"cascade {nm} bit-exact (frame2): "
                        f"{len(mism)} mismatch(es), first {mism[:3]}")


# ---------------------------------------------------------------------------
# Build harness: emit the 4-DUT wiring wrapper, then build+run under Verilator.
# ---------------------------------------------------------------------------
_HARNESS = r"""
`timescale 1ns / 1ps
`default_nettype none
// Auto-generated cascade wrapper for the cocotb port of tb_axis_rgb_cascade.sv.
// Contains ONLY the four DUT instances (no initial / no clock) so cocotb owns clk/rst and
// stimulus. Wiring is 1:1 with the DSim TB.
module cascade_harness #(
    parameter int W = 8
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire [23:0]  in_pixel,
    input  wire         in_valid,
    input  wire         in_sof,
    input  wire         in_eol,
    input  wire         in_eof,
    input  wire         in_err,
    // S1 general 5x5 config
    input  wire [199:0] c1,
    input  wire [3:0]   sh1,
    // S2 / S3 separable config
    input  wire [39:0]  h2, v2, h3, v3,
    input  wire [3:0]   hs2, vs2, hs3, vs3,
    // taps
    output wire [23:0]  t1, t2, t3, dg,
    output wire         t1v, t1s, t1e, t1f, t1r,
    output wire         t2v, t2s, t2e, t2f, t2r,
    output wire         t3v, t3s, t3e, t3f, t3r,
    output wire         dgv, dgs, dge, dgf, dgr
);
    axis_rgb_conv5x5 #(.LINE_PIXELS(W), .ENABLE(1'b1)) S1 (
        .clk(clk), .rst_n(rst_n), .cfg_en(1'b1), .cfg_coeffs(c1), .cfg_shift(sh1), .cfg_abs(1'b0),
        .in_pixel(in_pixel), .in_valid(in_valid), .in_sof(in_sof),
        .in_eol(in_eol), .in_eof(in_eof), .in_err(in_err),
        .out_pixel(t1), .out_valid(t1v), .out_sof(t1s), .out_eol(t1e), .out_eof(t1f), .out_err(t1r));
    axis_rgb_conv5x5_sep #(.LINE_PIXELS(W), .ENABLE(1'b1)) S2 (
        .clk(clk), .rst_n(rst_n), .cfg_h(h2), .cfg_v(v2), .cfg_hshift(hs2), .cfg_vshift(vs2),
        .in_pixel(t1), .in_valid(t1v), .in_sof(t1s), .in_eol(t1e), .in_eof(t1f), .in_err(t1r),
        .out_pixel(t2), .out_valid(t2v), .out_sof(t2s), .out_eol(t2e), .out_eof(t2f), .out_err(t2r));
    axis_rgb_conv5x5_sep #(.LINE_PIXELS(W), .ENABLE(1'b1)) S3 (
        .clk(clk), .rst_n(rst_n), .cfg_h(h3), .cfg_v(v3), .cfg_hshift(hs3), .cfg_vshift(vs3),
        .in_pixel(t2), .in_valid(t2v), .in_sof(t2s), .in_eol(t2e), .in_eof(t2f), .in_err(t2r),
        .out_pixel(t3), .out_valid(t3v), .out_sof(t3s), .out_eol(t3e), .out_eof(t3f), .out_err(t3r));
    axis_rgb_dog_combine #(.ENABLE(1'b1), .DEPTH(64)) DG (
        .clk(clk), .rst_n(rst_n), .cfg_mode(2'd2), .cfg_alpha(8'd1), .cfg_beta(8'd1),
        .cfg_shift(4'd0), .cfg_offset(9'sd128),
        .a_pixel(t1), .a_valid(t1v), .b_pixel(t3), .b_valid(t3v),
        .b_sof(t3s), .b_eol(t3e), .b_eof(t3f), .b_err(t3r),
        .out_pixel(dg), .out_valid(dgv), .out_sof(dgs), .out_eol(dge), .out_eof(dgf), .out_err(dgr));
endmodule
`default_nettype wire
"""


def test_axis_rgb_cascade():
    from runner_support import build_and_test

    here = Path(__file__).resolve().parent
    harness = here / "cascade_harness.sv"
    harness.write_text(_HARNESS, encoding="ascii")

    build_and_test(
        block="axis_rgb_cascade",
        sources=[
            "rtl/img_proc/axis_rgb_conv5x5.sv",
            "rtl/img_proc/axis_rgb_conv5x5_sep.sv",
            "rtl/img_proc/axis_rgb_dog_combine.sv",
            str(harness),
        ],
        toplevel="cascade_harness",
        test_module="test_axis_rgb_cascade",
        test_dir=here,
        parameters={"W": W},
    )
