"""cocotb port of verification/tb/tb_axis_rgb_conv5x5_sep.sv (valid-only pixel family).

The DSim TB proves the SEPARABLE 5x5 (``axis_rgb_conv5x5_sep``, the DUT) against the general
non-separable 5x5 (``axis_rgb_conv5x5``, the reference model): both are fed the SAME pixel
stream and their k-th outputs are the same spatial pixel (1:1 with the input). cocotb needs a
single HDL toplevel, so the two-DUT wiring is emitted as a tiny wrapper module
(``sep_vs_gen_harness``) at build time -- it contains ONLY the two instances (no ``initial``,
no clock), exposing both tap bundles. This is a build tool-input file, not a hand-maintained
RTL source; the two real RTL sources from axis_rgb_conv5x5_sep.f are the DUTs under test.

8x8 frame (W=H=8). The four DSim scenarios are replicated 1:1, each preceded by the exact
warm-up + config sequence the TB runs so the DUT line-buffer state matches:
  (1) identity kernel (h=v={0,0,1,0,0}, shifts 0) on uniform 100 -> sep bypass == 100 exactly;
  (2) separable Gaussian h=v=[1,4,6,4,1] (hshift/vshift 4) on uniform 100 -> unchanged (+/-1);
  (3) separable == general 5x5 Gaussian on horizontal bands 40/200 (sep_r[i] vs gen_r[i], +/-2);
  (4) COLOUR channel independence: R/G/B differ -> sep must match general on all 3 channels
      (+/-2), catching a per-channel bug the gray tests (R only) would miss.

The general 5x5 reference uses the outer product of [1,4,6,4,1] (sum 256, cfg_shift 8); the
separable path uses the split shifts (hshift4/vshift4) -- they agree within the +/-2 LSB
split-shift requantise budget. The DUTs use async active-low reset and 24-bit RGB pixels.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import bringup  # noqa: E402
from lib.scoreboard import check  # noqa: E402

W = 8   # LINE_PIXELS
H = 8

# separable identity / Gaussian taps (5 x signed-8b, LSB-first)
IDENT5 = [0, 0, 1, 0, 0]
G5 = [1, 4, 6, 4, 1]
# general 5x5 Gaussian: outer product of [1,4,6,4,1] (sum 256), cfg_shift 8, row-major idx0..24
GEN25 = [1, 4, 6, 4, 1,
         4, 16, 24, 16, 4,
         6, 24, 36, 24, 6,
         4, 16, 24, 16, 4,
         1, 4, 6, 4, 1]

UNI = [100] * H
BANDS = [40 if i < 4 else 200 for i in range(H)]


def pack(coeffs):
    """Pack signed-8b taps LSB-first (matches RTL cfg[idx*8 +: 8])."""
    val = 0
    for idx, c in enumerate(coeffs):
        val |= (c & 0xFF) << (idx * 8)
    return val


# ---------------------------------------------------------------------------
# Tap capture: mirrors the DSim always_ff logger (sep_r/gen_r + sep_px/gen_px, capped H*W).
# ---------------------------------------------------------------------------
class TapCapture:
    def __init__(self, dut, clk):
        self.dut = dut
        self.clk = clk
        self.reset()

    def reset(self):
        # full 24-bit pixels; the R channel (bits[23:16]) is sep_r/gen_r in the TB.
        self.sep_px, self.gen_px = [], []

    def start(self):
        cocotb.start_soon(self._run())

    async def _run(self):
        d = self.dut
        while True:
            await RisingEdge(self.clk)
            if int(d.sep_valid.value):
                if len(self.sep_px) < H * W:
                    self.sep_px.append(int(d.sep_pixel.value))
            if int(d.gen_valid.value):
                if len(self.gen_px) < H * W:
                    self.gen_px.append(int(d.gen_pixel.value))

    def sep_r(self, i):
        return (self.sep_px[i] >> 16) & 0xFF

    def gen_r(self, i):
        return (self.gen_px[i] >> 16) & 0xFF


def chan(px, sc):
    """Byte lane sc (2=R/23:16, 1=G/15:8, 0=B/7:0)."""
    return (px >> (sc * 8)) & 0xFF


async def _idle_in(dut):
    dut.in_valid.value = 0
    dut.in_sof.value = 0
    dut.in_eol.value = 0
    dut.in_eof.value = 0
    dut.in_err.value = 0
    dut.in_pixel.value = 0


async def _drive_pixels(dut, clk, rows_rgb, flush=32):
    """Drive a row-major H*W raster; rows_rgb[r] is the 24-bit pixel for every col of row r.
    Mirrors the DSim drive_frame / drive_color tasks (sof/eol/eof framing + 32-cycle flush)."""
    for r in range(H):
        for c in range(W):
            await RisingEdge(clk)
            dut.in_valid.value = 1
            dut.in_pixel.value = rows_rgb[r]
            dut.in_sof.value = 1 if (r == 0 and c == 0) else 0
            dut.in_eol.value = 1 if (c == W - 1) else 0
            dut.in_eof.value = 1 if (r == H - 1 and c == W - 1) else 0
    await RisingEdge(clk)
    await _idle_in(dut)
    for _ in range(flush):
        await RisingEdge(clk)


def _gray(line_vals):
    return [(v << 16) | (v << 8) | v for v in line_vals]


async def _cfg(dut, h, v, hshift, vshift):
    dut.cfg_h.value = pack(h)
    dut.cfg_v.value = pack(v)
    dut.cfg_hshift.value = hshift
    dut.cfg_vshift.value = vshift
    # general reference kernel is fixed (loaded once); harness ties cfg_en/shift/abs.
    dut.gcoef.value = pack(GEN25)


async def _bringup_and_warm(dut):
    """Reset, load the general reference kernel, then run the TB warm-up (identity uniform)
    frame that fills the cold line buffers -- identical to the DSim initial-block preamble."""
    clk, _ = await bringup(dut, clk="clk", rst="rst_n")
    dut.gcoef.value = pack(GEN25)
    await _idle_in(dut)
    cap = TapCapture(dut, clk)
    cap.start()
    # warm up cold line buffers (identity kernel, uniform) -- 1 throwaway frame
    await _cfg(dut, IDENT5, IDENT5, 0, 0)
    await _drive_pixels(dut, clk, _gray(UNI))
    return clk, cap


# ---------------------------------------------------------------------------
# Scenario 1: identity (bypass) on uniform -> sep == 100 exactly on interior rows.
# ---------------------------------------------------------------------------
@cocotb.test(timeout_time=5, timeout_unit="ms")
async def identity_uniform(dut):
    clk, cap = await _bringup_and_warm(dut)
    await _cfg(dut, IDENT5, IDENT5, 0, 0)
    cap.reset()
    await _drive_pixels(dut, clk, _gray(UNI))
    n = len(cap.sep_px)
    dut._log.info(f"[identity uniform] sep got {n}")
    check(n >= (H - 2) * W, f"enough sep beats ({n})")
    for i in range(3 * W, min((H - 2) * W, n)):
        check(cap.sep_r(i) == 100, f"ident-uni[{i}] (got {cap.sep_r(i)} exp 100)")


# ---------------------------------------------------------------------------
# Scenario 2: separable Gaussian on uniform -> unchanged 100 (+/-1).
# ---------------------------------------------------------------------------
@cocotb.test(timeout_time=5, timeout_unit="ms")
async def gaussian_uniform(dut):
    clk, cap = await _bringup_and_warm(dut)
    # DSim runs the identity-uniform frame (scenario 1) before scenario 2; replicate so the
    # DUT line-buffer state matches exactly.
    await _cfg(dut, IDENT5, IDENT5, 0, 0)
    await _drive_pixels(dut, clk, _gray(UNI))
    # scenario 2: separable Gaussian on uniform
    await _cfg(dut, G5, G5, 4, 4)
    cap.reset()
    await _drive_pixels(dut, clk, _gray(UNI))
    n = len(cap.sep_px)
    dut._log.info(f"[gaussian uniform] sep got {n}")
    check(n >= (H - 3) * W, f"enough sep beats ({n})")
    for i in range(3 * W, min((H - 3) * W, n)):
        got = cap.sep_r(i)
        check(abs(got - 100) <= 1, f"gauss-uni[{i}] (got {got} exp 100 +/-1)")


# ---------------------------------------------------------------------------
# Scenario 3: separable == general 5x5 Gaussian on horizontal bands (+/-2).
# ---------------------------------------------------------------------------
@cocotb.test(timeout_time=5, timeout_unit="ms")
async def sep_matches_general_bands(dut):
    clk, cap = await _bringup_and_warm(dut)
    # replicate the DSim preamble: identity-uniform (sc1) then gaussian-uniform (sc2).
    await _cfg(dut, IDENT5, IDENT5, 0, 0)
    await _drive_pixels(dut, clk, _gray(UNI))
    await _cfg(dut, G5, G5, 4, 4)
    await _drive_pixels(dut, clk, _gray(UNI))
    # scenario 3: same separable Gaussian, horizontal bands 40/200
    cap.reset()
    await _drive_pixels(dut, clk, _gray(BANDS))
    ns, ng = len(cap.sep_px), len(cap.gen_px)
    dut._log.info(f"[sep vs general bands] sep got {ns} gen got {ng}")
    for rr in range(H):
        ss = gs = cnt = 0
        for i in range(rr * W + 2, min(rr * W + W - 2, ns, ng)):
            ss += cap.sep_r(i)
            gs += cap.gen_r(i)
            cnt += 1
        if cnt > 0:
            dut._log.info(f"   row {rr}: sep={ss // cnt} gen={gs // cnt}")
    for i in range(2 * W, min((H - 2) * W, ns, ng)):
        s, g = cap.sep_r(i), cap.gen_r(i)
        check(abs(s - g) <= 2, f"sep==gen[{i}] (sep={s} gen={g} +/-2)")


# ---------------------------------------------------------------------------
# Scenario 4: colour channel independence -- R/G/B differ, sep must match general on ALL
# channels (+/-2). Catches a per-channel bug the gray tests (R only) miss.
# ---------------------------------------------------------------------------
@cocotb.test(timeout_time=5, timeout_unit="ms")
async def colour_channel_independence(dut):
    clk, cap = await _bringup_and_warm(dut)
    # replicate the DSim preamble: sc1 (identity-uni), sc2 (gauss-uni), sc3 (gauss-bands).
    await _cfg(dut, IDENT5, IDENT5, 0, 0)
    await _drive_pixels(dut, clk, _gray(UNI))
    await _cfg(dut, G5, G5, 4, 4)
    await _drive_pixels(dut, clk, _gray(UNI))
    await _drive_pixels(dut, clk, _gray(BANDS))
    # scenario 4: per-channel colour drive (still separable Gaussian h=v config)
    rv = [40 if i < 4 else 200 for i in range(H)]
    gv = [128 for _ in range(H)]
    bv = [(30 + i * 10) & 0xFF for i in range(H)]
    rows = [(rv[i] << 16) | (gv[i] << 8) | bv[i] for i in range(H)]
    cap.reset()
    await _drive_pixels(dut, clk, rows)
    ns, ng = len(cap.sep_px), len(cap.gen_px)
    dut._log.info(f"[colour sep vs general] sep got {ns} gen got {ng}")
    for i in range(4 * W + 2, min((H - 3) * W, ns, ng)):
        sp, gp = cap.sep_px[i], cap.gen_px[i]
        check(abs(chan(sp, 2) - chan(gp, 2)) <= 2, f"colR[{i}] (sep={chan(sp,2)} gen={chan(gp,2)})")
        check(abs(chan(sp, 1) - chan(gp, 1)) <= 2, f"colG[{i}] (sep={chan(sp,1)} gen={chan(gp,1)})")
        check(abs(chan(sp, 0) - chan(gp, 0)) <= 2, f"colB[{i}] (sep={chan(sp,0)} gen={chan(gp,0)})")
    idx = 5 * W + 3
    if idx < min(ns, ng):
        sp, gp = cap.sep_px[idx], cap.gen_px[idx]
        dut._log.info(
            f"   row5: sep={chan(sp,2)}/{chan(sp,1)}/{chan(sp,0)} "
            f"gen={chan(gp,2)}/{chan(gp,1)}/{chan(gp,0)}")


# ---------------------------------------------------------------------------
# Build harness: emit the 2-DUT (sep DUT + general reference) wiring wrapper, then build+run.
# ---------------------------------------------------------------------------
_HARNESS = r"""
`timescale 1ns / 1ps
`default_nettype none
// Auto-generated wrapper for the cocotb port of tb_axis_rgb_conv5x5_sep.sv.
// Contains ONLY the two DUT instances (no initial / no clock) so cocotb owns clk/rst and
// stimulus. Wiring is 1:1 with the DSim TB: both DUTs see the same input stream; the general
// 5x5 is the reference model the separable DUT is checked against.
module sep_vs_gen_harness #(
    parameter int LINE_PIXELS = 8
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire [39:0]  cfg_h,
    input  wire [39:0]  cfg_v,
    input  wire [3:0]   cfg_hshift,
    input  wire [3:0]   cfg_vshift,
    input  wire [199:0] gcoef,
    input  wire [23:0]  in_pixel,
    input  wire         in_valid,
    input  wire         in_sof,
    input  wire         in_eol,
    input  wire         in_eof,
    input  wire         in_err,
    output wire [23:0]  sep_pixel,
    output wire         sep_valid, sep_sof, sep_eol, sep_eof, sep_err,
    output wire [23:0]  gen_pixel,
    output wire         gen_valid, gen_sof, gen_eol, gen_eof, gen_err
);
    axis_rgb_conv5x5_sep #(.LINE_PIXELS(LINE_PIXELS), .ENABLE(1'b1)) dut (
        .clk(clk), .rst_n(rst_n), .cfg_h(cfg_h), .cfg_v(cfg_v),
        .cfg_hshift(cfg_hshift), .cfg_vshift(cfg_vshift),
        .in_pixel(in_pixel), .in_valid(in_valid), .in_sof(in_sof),
        .in_eol(in_eol), .in_eof(in_eof), .in_err(in_err),
        .out_pixel(sep_pixel), .out_valid(sep_valid), .out_sof(sep_sof),
        .out_eol(sep_eol), .out_eof(sep_eof), .out_err(sep_err));
    axis_rgb_conv5x5 #(.LINE_PIXELS(LINE_PIXELS), .ENABLE(1'b1)) gen (
        .clk(clk), .rst_n(rst_n), .cfg_en(1'b1), .cfg_coeffs(gcoef), .cfg_shift(4'd8), .cfg_abs(1'b0),
        .in_pixel(in_pixel), .in_valid(in_valid), .in_sof(in_sof),
        .in_eol(in_eol), .in_eof(in_eof), .in_err(in_err),
        .out_pixel(gen_pixel), .out_valid(gen_valid), .out_sof(gen_sof),
        .out_eol(gen_eol), .out_eof(gen_eof), .out_err(gen_err));
endmodule
`default_nettype wire
"""


def test_axis_rgb_conv5x5_sep():
    from runner_support import build_and_test

    here = Path(__file__).resolve().parent
    harness = here / "sep_vs_gen_harness.sv"
    harness.write_text(_HARNESS, encoding="ascii")

    build_and_test(
        block="axis_rgb_conv5x5_sep",
        sources=[
            "rtl/img_proc/axis_rgb_conv5x5_sep.sv",
            "rtl/img_proc/axis_rgb_conv5x5.sv",
            str(harness),
        ],
        toplevel="sep_vs_gen_harness",
        test_module="test_axis_rgb_conv5x5_sep",
        test_dir=here,
        parameters={"LINE_PIXELS": W},
    )
