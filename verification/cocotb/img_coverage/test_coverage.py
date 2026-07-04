r"""Sim-free functional-coverage closure for the img_proc golden input space.

"All green" says nothing about *what* the stimulus reached. These checks sample the golden
models over the built-in 64x48 test pattern and assert the behavioral corners are actually
exercised -- saturation clamped to BOTH rails, border AND interior pixels, a threshold
boundary crossed on both sides, both dither modes -- and that the item-2 GapPolicy emits the
gap sizes it claims. Pure Python (lib/coverage.CoverageTally, no cocotb, no numpy), so it
runs under any interpreter and gates in the ``smoke`` suite as block ``img_coverage``
(engine ``none``).

Run:  .\scripts\run_cocotb.ps1 img_coverage      (or .\scripts\pytest_cocotb.ps1 <dir>)
"""
from __future__ import annotations

import random
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parents[0]))               # verification/cocotb (lib.*)
sys.path.insert(0, str(_HERE.parents[0] / "img_file_uvm"))  # golden, img_config, image_io

import golden as G       # noqa: E402
import image_io          # noqa: E402
from lib.coverage import CoverageTally  # noqa: E402
from lib.gap import GapPolicy           # noqa: E402


def _ch(p, sel):
    return (p >> (sel * 8)) & 0xFF


def _pattern():
    return image_io.make_test_pattern()     # (pixels, w, h) -- deterministic 64x48


def _rand_image(w, h, seed=0xC0FFEE):
    rng = random.Random(seed)
    return [rng.randrange(0x1000000) for _ in range(w * h)]


def test_conv_saturation_and_border_coverage():
    px, w, h = _pattern()
    cov = CoverageTally("conv")
    # sobel_x, shift 0, abs 0 -> signed gradients on the colour-bar edges: negatives clamp to
    # 0, large positives clamp to 255, so both rails must appear.
    coeffs = [-1, 0, 1, -2, 0, 2, -1, 0, 1]
    out = G.conv_golden(px, w, coeffs, 0, 0, 1, 3)
    for k, p in enumerate(out):
        r, c = divmod(k, w)
        cov.sample("region", "border" if (r < 2 or c < 2) else "interior")
        for sel in (2, 1, 0):
            v = _ch(p, sel)
            cov.sample("clamp", "lo" if v == 0 else "hi" if v == 255 else "mid")
    print(cov.summary())
    cov.assert_covered("clamp", ["lo", "hi", "mid"])       # both saturation rails reached
    cov.assert_covered("region", ["border", "interior"])   # borders are verified, not masked


def test_threshold_boundary_coverage():
    px, _, _ = _pattern()
    cov = CoverageTally("threshold")
    thresh = 128
    out = G.proc_slot_golden(px, 4, thresh)                # op 4 = threshold on green
    for p, o in zip(px, out):
        g = _ch(p, 1)
        cov.sample("side", "above" if g > thresh else "at_or_below")
        cov.sample("result", "white" if o == 0xFFFFFF else "black")
    print(cov.summary())
    cov.assert_covered("side", ["above", "at_or_below"])
    cov.assert_covered("result", ["white", "black"])


def test_median_tie_coverage():
    cov = CoverageTally("median")
    # the flat colour-bar pattern gives only ties; a random image gives distinct windows --
    # both stimuli together close the coverage on the median's ordering path.
    pat = _pattern()
    for px, w, h in (pat, (_rand_image(24, 18), 24, 18)):
        rows = [px[r * w:(r + 1) * w] for r in range(h)]
        for R in range(2, h):
            for C in range(2, w):
                for sel in (2, 1, 0):
                    vals = [_ch(rows[R - 2 + r][C - 2 + c], sel)
                            for r in range(3) for c in range(3)]
                    cov.sample("tie", "tie" if len(set(vals)) < 9 else "distinct")
    print(cov.summary())
    cov.assert_covered("tie", ["tie", "distinct"])         # median must handle both


def test_dither_mode_and_depth_coverage():
    px, w, h = _pattern()
    cov = CoverageTally("dither")
    for bits in range(1, 7):
        for mode_name, mode_bit in (("ordered", 0), ("random", 1)):
            ctrl = 0x01 | (mode_bit << 1) | (bits << 2)
            out = G.dither_golden(px, w, h, ctrl)
            cov.sample("mode", mode_name)
            cov.sample("bits", bits)
            cov.sample("changed", "yes" if out != list(px) else "no")
    print(cov.summary())
    cov.assert_covered("mode", ["ordered", "random"])
    cov.assert_covered("bits", list(range(1, 7)))
    cov.assert_covered("changed", ["yes"])                 # dither actually altered pixels


def test_gap_policy_bin_coverage():
    """Item-2 coverage: each GapPolicy kind emits the gap sizes it promises."""
    cov = CoverageTally("gap")
    for kind in ("none", "sparse", "burst", "adversarial"):
        pol = GapPolicy(kind=kind, seed=1, max_gap=3)
        for _ in range(500):
            pol.next_gap()
        cov.merge_counter(kind, pol.produced)
    print(cov.summary())
    assert cov.covered("none") == {0}                      # off -> only zero-gaps
    cov.assert_covered("sparse", [0, 1, 2, 3])
    cov.assert_covered("burst", [0, 1, 2, 3])
    cov.assert_covered("adversarial", [1, 2, 3])
    assert not cov.hit("adversarial", 0)                   # adversarial always stalls
