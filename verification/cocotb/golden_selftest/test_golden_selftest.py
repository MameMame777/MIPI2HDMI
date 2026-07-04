r"""Sim-free self-tests for the img_proc golden models (img_file_uvm/golden.py).

``golden.py`` is the single oracle for the whole ``image`` suite: ``FrameScoreboard``
compares RTL output *bit-exact* against it, so one silent edit to ``golden.py`` corrupts
every img test at once with nothing to catch it. These tests need NO simulator -- they run
the golden models directly and:

  (1) cross-check each model against an INDEPENDENT re-derivation written a different way
      (direct 2D indexing vs the streaming window generator; explicit weights vs the packed
      accumulator), so a windowing / coefficient bug is caught; and
  (2) pin the deliberately-quirky bits -- the dither 3-bit self-determined smear-shift wrap
      ((2*n)&7 / (4*n)&7) that ``golden.py`` says "do NOT fix" -- so a well-meaning refactor
      that "fixes" it fails loudly.

Stdlib-only, so it runs under the same MSYS2 ucrt64 python as the gate (no numpy/Pillow on
the sim side); an optional numpy cross-check is import-guarded. Registered in manifest.toml
as block ``golden_selftest`` (engine ``none``) in the ``smoke`` suite, so it gates every run.

Run:  .\scripts\run_cocotb.ps1 golden_selftest      (or .\scripts\pytest_cocotb.ps1 <dir>)
"""
from __future__ import annotations

import random
import sys
from pathlib import Path

# golden.py / img_config.py live in the sibling img_file_uvm block dir.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "img_file_uvm"))

import golden as G       # noqa: E402
import img_config as C   # noqa: E402


# --------------------------------------------------------------------------- helpers

def _rand_image(width: int, height: int, seed: int = 0xC0FFEE) -> list[int]:
    """Deterministic RGB image as a flat row-major list of 24-bit {R,G,B} ints."""
    rng = random.Random(seed)
    return [rng.randrange(0x1000000) for _ in range(width * height)]


def _rows(pixels: list[int], width: int) -> list[list[int]]:
    return [pixels[r * width:(r + 1) * width] for r in range(len(pixels) // width)]


def _ch(p: int, sel: int) -> int:
    return (p >> (sel * 8)) & 0xFF


def _clamp8(v: int) -> int:
    return 0 if v < 0 else 255 if v > 255 else v


def _ref_conv_interior(pixels, width, coeffs, shift, absf, taps):
    """Independent taps x taps convolution by DIRECT 2D indexing (not the streaming window
    generator ``_windows``), for the border-free interior only (rows/cols >= taps-1, where
    the window holds only real image pixels). Returns {beat_index: expected_pixel}. Beat k
    at (R,C) sees rows [R-(taps-1)..R] x cols [C-(taps-1)..C]."""
    rows = _rows(pixels, width)
    height = len(rows)
    out = {}
    for R in range(taps - 1, height):
        for Ci in range(taps - 1, width):
            px = 0
            for sel in (2, 1, 0):
                acc = 0
                for r in range(taps):
                    for c in range(taps):
                        acc += coeffs[r * taps + c] * _ch(
                            rows[R - (taps - 1) + r][Ci - (taps - 1) + c], sel)
                v = acc >> shift
                if absf and v < 0:
                    v = -v
                px = (px << 8) | _clamp8(v)
            out[R * width + Ci] = px
    return out


# --------------------------------------------------------------------------- conv

def test_conv3x3_interior_matches_independent_reference():
    width, height, taps = 9, 7, 3
    pixels = _rand_image(width, height)
    for name, (coeffs, shift, absf) in C.CONV3_KERNELS.items():
        got = G.conv_golden(pixels, width, coeffs, shift, absf, 1, taps)
        ref = _ref_conv_interior(pixels, width, coeffs, shift, absf, taps)
        assert ref, "no interior beats -- test image too small"
        for k, exp in ref.items():
            assert got[k] == exp, f"conv3x3 {name} beat {k}: {got[k]:#08x} != {exp:#08x}"


def test_conv5x5_interior_matches_independent_reference():
    width, height, taps = 11, 9, 5
    pixels = _rand_image(width, height, seed=0x5A5A)
    for name, (coeffs, shift, absf) in C.CONV5_KERNELS.items():
        got = G.conv_golden(pixels, width, coeffs, shift, absf, 1, taps)
        ref = _ref_conv_interior(pixels, width, coeffs, shift, absf, taps)
        assert ref, "no interior beats -- test image too small"
        for k, exp in ref.items():
            assert got[k] == exp, f"conv5x5 {name} beat {k}: {got[k]:#08x} != {exp:#08x}"


def test_conv_disabled_is_window_centre_passthrough():
    for taps, width, height in ((3, 8, 6), (5, 10, 8)):
        pixels = _rand_image(width, height, seed=taps * 7)
        out = G.conv_golden(pixels, width, [0] * (taps * taps), 3, 0, 0, taps)
        rows = _rows(pixels, width)
        d = taps // 2
        for R in range(taps - 1, height):
            for Ci in range(taps - 1, width):
                k = R * width + Ci
                assert out[k] == rows[R - (taps - 1) + d][Ci - (taps - 1) + d]


def test_conv_identity_kernel_equals_disabled():
    width, height, taps = 8, 6, 3
    pixels = _rand_image(width, height, seed=11)
    ident = C.CONV3_KERNELS["identity"][0]
    en1 = G.conv_golden(pixels, width, ident, 0, 0, 1, taps)
    en0 = G.conv_golden(pixels, width, [0] * 9, 0, 0, 0, taps)
    assert en1 == en0


def test_conv_saturates_both_rails():
    width, height, taps = 6, 6, 3
    white = [0xFFFFFF] * (width * height)
    hi = G.conv_golden(white, width, [10] * 9, 0, 0, 1, taps)   # +22950 >> 0 -> clamp 255
    lo = G.conv_golden(white, width, [-10] * 9, 0, 0, 1, taps)  # -22950 >> 0 -> clamp 0
    for R in range(taps - 1, height):
        for Ci in range(taps - 1, width):
            k = R * width + Ci
            for sel in (2, 1, 0):
                assert _ch(hi[k], sel) == 255
                assert _ch(lo[k], sel) == 0


def test_conv_output_channels_in_range():
    width, height = 9, 7
    pixels = _rand_image(width, height, seed=0xBEEF)
    for name, (coeffs, shift, absf) in C.CONV3_KERNELS.items():
        for p in G.conv_golden(pixels, width, coeffs, shift, absf, 1, 3):
            for sel in (2, 1, 0):
                assert 0 <= _ch(p, sel) <= 255, f"conv3x3 {name}: channel out of range"


def test_conv_numpy_cross_check():
    """Optional third path: numpy 'valid' correlation. Skipped where numpy is absent
    (the ucrt64 sim-side python has none; the repo .venv may)."""
    import pytest
    np = pytest.importorskip("numpy")
    width, height, taps = 9, 7, 3
    pixels = _rand_image(width, height, seed=0x101)
    rows = _rows(pixels, width)
    for name, (coeffs, shift, absf) in C.CONV3_KERNELS.items():
        got = G.conv_golden(pixels, width, coeffs, shift, absf, 1, taps)
        ker = np.array(coeffs, dtype=np.int64).reshape(taps, taps)
        for sel in (2, 1, 0):
            plane = np.array([[_ch(px, sel) for px in row] for row in rows], dtype=np.int64)
            for R in range(taps - 1, height):
                for Ci in range(taps - 1, width):
                    patch = plane[R - taps + 1:R + 1, Ci - taps + 1:Ci + 1]
                    v = int((patch * ker).sum()) >> shift
                    if absf and v < 0:
                        v = -v
                    assert _ch(got[R * width + Ci], sel) == _clamp8(v), \
                        f"conv3x3 {name} numpy mismatch"


# --------------------------------------------------------------------------- point ops

def test_proc_slot_point_ops():
    pixels = _rand_image(5, 4, seed=3)
    thresh = 100
    for op in range(8):
        out = G.proc_slot_golden(pixels, op, thresh)
        assert len(out) == len(pixels)
        for p, o in zip(pixels, out):
            r, g, b = _ch(p, 2), _ch(p, 1), _ch(p, 0)
            if op == 0:
                assert o == p
            elif op == 1:
                assert o == (((~r & 0xFF) << 16) | ((~g & 0xFF) << 8) | (~b & 0xFF))
            elif op == 2:
                assert _ch(o, 2) == _ch(o, 1) == _ch(o, 0) == g
            elif op == 3:
                assert (_ch(o, 2), _ch(o, 1), _ch(o, 0)) == (b, g, r)
            elif op == 4:
                assert o in (0x000000, 0xFFFFFF)
                assert (o == 0xFFFFFF) == (g > thresh)
            elif op == 5:
                assert o == (r << 16)
            elif op == 6:
                assert o == (g << 8)
            elif op == 7:
                assert o == b
            for sel in (2, 1, 0):
                assert 0 <= _ch(o, sel) <= 255


def test_invert_is_involution():
    pixels = _rand_image(7, 5, seed=99)
    once = G.proc_slot_golden(pixels, 1, 0)
    twice = G.proc_slot_golden(once, 1, 0)
    assert twice == pixels


# --------------------------------------------------------------------------- prefilter

def test_prefilter_median_is_order_statistic():
    width, height = 9, 7
    pixels = _rand_image(width, height, seed=0x1234)
    out = G.prefilter_golden(pixels, width, 9, 0)   # op 9 = per-channel 9-median
    rows = _rows(pixels, width)
    for R in range(2, height):
        for Ci in range(2, width):
            k = R * width + Ci
            for sel in (2, 1, 0):
                vals = sorted(_ch(rows[R - 2 + r][Ci - 2 + c], sel)
                              for r in range(3) for c in range(3))
                assert _ch(out[k], sel) == vals[4]
                assert _ch(out[k], sel) in vals        # median is a member of the window


def test_prefilter_gaussian_matches_weights():
    width, height = 9, 7
    pixels = _rand_image(width, height, seed=0x77)
    out = G.prefilter_golden(pixels, width, 8, 0)   # op 8 = (corners + 2*edges + 4*centre)>>4
    rows = _rows(pixels, width)
    for R in range(2, height):
        for Ci in range(2, width):
            k = R * width + Ci
            for sel in (2, 1, 0):
                w = [[_ch(rows[R - 2 + r][Ci - 2 + c], sel) for c in range(3)]
                     for r in range(3)]
                corner = w[0][0] + w[0][2] + w[2][0] + w[2][2]
                edge = w[0][1] + w[1][0] + w[1][2] + w[2][1]
                assert _ch(out[k], sel) == ((corner + 2 * edge + 4 * w[1][1]) >> 4)


def test_prefilter_passthrough_ops_hit_window_centre():
    width, height = 8, 6
    pixels = _rand_image(width, height, seed=0x55)
    rows = _rows(pixels, width)
    for op in (0, 10, 15):          # point-op passthrough -> centre pixel unchanged
        out = G.prefilter_golden(pixels, width, op, 0)
        for R in range(2, height):
            for Ci in range(2, width):
                assert out[R * width + Ci] == rows[R - 1][Ci - 1]


# --------------------------------------------------------------------------- dither

def test_dither_disabled_is_identity():
    pixels = _rand_image(8, 4, seed=1)
    assert G.dither_golden(pixels, 8, 4, ctrl=0x00) == pixels        # ctrl bit0 (en) = 0


def _ref_dith_ch(v, by, rnd, mode, n):
    """Independent re-derivation of golden._dith_ch that KEEPS the documented 3-bit
    self-determined shift wrap ((2*n)&7, (4*n)&7). If golden.py is ever 'fixed' to the
    naive shifts this diverges, so the wrap-quirk regression is caught."""
    if n == 0 or n >= 7:
        return v
    drop = 8 - n
    if mode:
        bias = rnd & ((1 << drop) - 1)
    elif drop >= 4:
        bias = by << (drop - 4)
    else:
        bias = by >> (4 - drop)
    s = min(255, v + bias)
    o = s & ~((1 << drop) - 1) & 0xFF
    o |= o >> n
    o |= o >> ((2 * n) & 7)
    o |= o >> ((4 * n) & 7)
    return o & 0xFF


def test_dither_channel_matches_reference_including_wrap():
    for n in range(0, 8):
        for mode in (0, 1):
            for v in range(0, 256, 7):
                for by in G._BAYER4:
                    for rnd in (0x00, 0xA5, 0xFF):
                        assert G._dith_ch(v, by, rnd, mode, n) == _ref_dith_ch(
                            v, by, rnd, mode, n), f"dith n={n} mode={mode} v={v}"


def test_dither_smear_shift_wrap_is_load_bearing():
    """Direct proof the third smear uses (4*n)&7 == 4 (not a naive 12, which is a no-op on
    an 8-bit value). Pick n=3 with an input whose quantised value gains low bits only via
    the >>4 term."""
    n = 3
    diff = [v for v in range(256)
            if G._dith_ch(v, 0, 0, 0, n) != _ref_dith_ch_naive(v, 0, 0, 0, n)]
    assert diff, "n=3 third smear shift is a no-op -- the (4*n)&7 wrap was lost"


def _ref_dith_ch_naive(v, by, rnd, mode, n):
    """The 'fixed' (WRONG) variant: naive shifts 2*n / 4*n with no 3-bit wrap. Used only to
    demonstrate the wrap is observable."""
    if n == 0 or n >= 7:
        return v
    drop = 8 - n
    bias = (rnd & ((1 << drop) - 1)) if mode else (
        by << (drop - 4) if drop >= 4 else by >> (4 - drop))
    o = min(255, v + bias) & ~((1 << drop) - 1) & 0xFF
    o |= o >> n
    o |= o >> (2 * n)
    o |= o >> (4 * n)
    return o & 0xFF


def test_dither_lfsr_is_maximal_length():
    lfsr = 0xA5
    seen = []
    for _ in range(255):
        seen.append(lfsr)
        lfsr = ((lfsr << 1) & 0xFF) ^ (0x1D if lfsr & 0x80 else 0)
    assert 0 not in seen
    assert len(set(seen)) == 255        # all 255 nonzero states, each exactly once
    assert lfsr == 0xA5                 # returns to seed after the full period


def test_dither_lfsr_carries_across_frames():
    width, height = 4, 2
    frame = _rand_image(width, height, seed=7)
    ctrl = 0x01 | (1 << 1) | (2 << 2)   # en=1, random mode, 2 bits/channel
    two = G.dither_golden(frame + frame, width, height, ctrl)
    one = G.dither_golden(frame, width, height, ctrl)
    assert two[:len(frame)] == one              # first frame reproducible
    assert two != one + one                     # LFSR NOT reset between frames


# --------------------------------------------------------------------------- conv5x5 separable

def _ref_sep_interior(pixels, width, hc, vc, hshift, vshift):
    """Independent two-pass separable reference for the border-free interior (r,c >= 4), where
    both passes see only real image pixels -- a different code path from the streaming golden."""
    rows = _rows(pixels, width)
    height = len(rows)
    hout = {}                                       # (r,c>=4) -> [B12,G12,R12]
    for r in range(height):
        for c in range(4, width):
            chans = [0, 0, 0]
            for sel in (0, 1, 2):
                acc = sum(hc[i] * _ch(rows[r][c - 4 + i], sel) for i in range(5))
                v = acc >> hshift
                chans[sel] = -2048 if v < -2048 else 2047 if v > 2047 else v
            hout[(r, c)] = chans
    out = {}
    for r in range(4, height):
        for c in range(4, width):
            px = 0
            for sel in (2, 1, 0):
                vsum = sum(vc[j] * hout[(r - 4 + j, c)][sel] for j in range(5))
                px = (px << 8) | _clamp8(vsum >> vshift)
            out[r * width + c] = px
    return out


def test_conv5x5_sep_interior_matches_independent_reference():
    width, height = 13, 11
    px = _rand_image(width, height, seed=0x5E9)
    for name, (hc, vc, hs, vs) in C.SEP_KERNELS.items():
        got = G.conv5x5_sep_golden(px, width, hc, vc, hs, vs)
        ref = _ref_sep_interior(px, width, hc, vc, hs, vs)
        assert ref, "no interior beats -- test image too small"
        for k, exp in ref.items():
            assert got[k] == exp, f"sep {name} beat {k}: {got[k]:#08x} != {exp:#08x}"


# --------------------------------------------------------------------------- DoG combine

def _ref_combine_ch(av, bv, mode, alpha, beta, shift, offset):
    if mode == 0:
        return av
    if mode == 1:
        return bv
    s = (alpha * av + beta * bv) if mode == 3 else (alpha * av - beta * bv)
    v = (s >> shift) + offset
    return 0 if v < 0 else 255 if v > 255 else v


def test_dog_combine_matches_reference():
    rng = random.Random(0xD06)
    a = [rng.randrange(0x1000000) for _ in range(200)]
    b = [rng.randrange(0x1000000) for _ in range(200)]
    params = [(1, 1, 0, 0), (1, 1, 0, 128), (2, 1, 1, -64), (1, 2, 2, 255), (3, 5, 3, -256)]
    for mode in (0, 1, 2, 3):
        for alpha, beta, shift, offset in params:
            got = G.dog_combine_golden(a, b, mode, alpha, beta, shift, offset)
            for k, (ap, bp) in enumerate(zip(a, b)):
                exp = 0
                for sel in (2, 1, 0):
                    exp = (exp << 8) | _ref_combine_ch(
                        _ch(ap, sel), _ch(bp, sel), mode, alpha, beta, shift, offset)
                assert got[k] == exp, f"combine mode={mode} k={k}: {got[k]:#08x} != {exp:#08x}"
