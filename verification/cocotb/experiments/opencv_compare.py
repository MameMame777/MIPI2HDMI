"""Experiment: can an OpenCV filter serve as the verification oracle instead of the
RTL-exact golden?

We compare, on identical input, three "expected" images per filter:
  G   = golden.py (RTL-exact; proven bit-identical to the Verilator RTL output)
  N   = OpenCV the *naive* way a person writes it (float kernel, default border, uint8 round)
  I   = OpenCV with the RTL's integer arithmetic bolted on (int kernel, zero border, floor >>)

and decompose where G and the OpenCV variants diverge into three causes:
  (1) spatial shift  -- the RTL emits each output offset by the window radius d=(taps-1)/2
  (2) border         -- RTL zero-inits line buffers + carries the prev row's tail on the left;
                        no cv2 borderType reproduces that
  (3) rounding       -- RTL floors (arithmetic >>shift); cv2 rounds-to-nearest

Dev-only (NOT part of the sim toolchain): run under the repo-root CPython venv with
``numpy`` + ``opencv-python-headless`` (and the stdlib ``golden.py``). See README.md.
Backs the "OpenCV as the oracle" numbers in docs/doc/image_file_verification.md and the
diary. Output images go to the gitignored verification/cocotb/_exec/opencv_exp/.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "verification/cocotb/img_file_uvm"))
import golden as G          # noqa: E402
import image_io            # noqa: E402
import img_config as C     # noqa: E402

OUT = REPO / "verification/cocotb/_exec/opencv_exp"
OUT.mkdir(parents=True, exist_ok=True)


# ---- flat 24-bit {R,G,B} list  <->  HxWx3 uint8 (RGB) ----
def to_np(pixels, w, h):
    a = np.asarray(pixels, dtype=np.uint32).reshape(h, w)
    return np.stack([(a >> 16) & 0xFF, (a >> 8) & 0xFF, a & 0xFF], axis=-1).astype(np.uint8)


def to_flat(img):
    p = ((img[..., 0].astype(np.uint32) << 16)
         | (img[..., 1].astype(np.uint32) << 8) | img[..., 2].astype(np.uint32))
    return p.reshape(-1).tolist()


def rand_image(w, h, seed=1234):
    rng = np.random.default_rng(seed)
    return rng.integers(0, 256, size=(h, w, 3), dtype=np.uint8)


# ---- metric: decompose divergence between the RTL-exact golden and an OpenCV output ----
def report(name, gold, cv_naive, cv_int, taps):
    h, w, _ = gold.shape
    d = taps // 2
    n = h * w

    def diffcount(A, B):
        return int(np.any(A != B, axis=-1).sum()), int(np.abs(A.astype(int) - B.astype(int)).max())

    # (0) what you get if you just diff golden-output vs cv2-output, no alignment at all
    raw_mm, raw_max = diffcount(gold, cv_naive)

    # (1) align the d-pixel spatial shift, restrict to the clean interior:
    #     golden[R][C] (R,C >= 2d) vs cv2[R-d][C-d]  ->  gold[2d:,2d:] vs cv2[d:h-d, d:w-d]
    gi = gold[2 * d:, 2 * d:].astype(int)
    ni = cv_naive[d:h - d, d:w - d].astype(int)
    ii = cv_int[d:h - d, d:w - d].astype(int)
    interior_n = int(np.prod(gi.shape[:2]))
    naive_mm = int(np.any(gi != ni, axis=-1).sum())
    naive_max = int(np.abs(gi - ni).max())
    naive_mean = float(np.abs(gi - ni).mean())
    int_mm = int(np.any(gi != ii, axis=-1).sum())
    int_max = int(np.abs(gi - ii).max())

    # (2) BORDER only: shift-aligned FULL overlap vs int-matched cv2. Interior is 0, so every
    #     mismatch here is the upper/left fringe -- pure padding-convention divergence.
    gb = gold[d:, d:].astype(int)
    ib = cv_int[0:h - d, 0:w - d].astype(int)
    border_mm = int(np.any(gb != ib, axis=-1).sum())
    border_max = int(np.abs(gb - ib).max())

    print(f"\n=== {name}  ({w}x{h}, taps={taps}, d={d}) ===")
    print(f"  [0] raw golden-out vs naive-cv2 (no shift align): "
          f"{raw_mm:>6}/{n} px differ ({100*raw_mm/n:5.1f}%), max |diff|={raw_max}")
    print(f"  [1] shift-aligned INTERIOR ({interior_n} px, borders excluded):")
    print(f"        vs naive cv2 (float,reflect,round): {naive_mm:>6} px differ "
          f"({100*naive_mm/interior_n:5.1f}%), max={naive_max}, mean={naive_mean:.3f}   <- rounding+any residual")
    print(f"        vs int-matched cv2 (int,zero,floor): {int_mm:>6} px differ "
          f"({100*int_mm/interior_n:5.1f}%), max={int_max}                    <- should be ~0 (arith matched)")
    print(f"  [2] BORDER fringe (arith matched, so this is pure padding-convention): "
          f"{border_mm} px differ, max |diff|={border_max}")
    return dict(name=name, raw_mm=raw_mm, raw_pct=100 * raw_mm / n, raw_max=raw_max,
                interior_n=interior_n, naive_mm=naive_mm, naive_pct=100 * naive_mm / interior_n,
                naive_max=naive_max, naive_mean=naive_mean,
                int_mm=int_mm, int_pct=100 * int_mm / interior_n, int_max=int_max)


def save_gallery(name, inp, gold, cv_naive, taps):
    h, w, _ = gold.shape
    image_io.write_png(OUT / f"{name}_input.png", to_flat(inp), w, h)
    image_io.write_png(OUT / f"{name}_golden.png", to_flat(gold), w, h)
    image_io.write_png(OUT / f"{name}_opencv.png", to_flat(cv_naive), w, h)
    # heatmap of |golden - naive_cv2| (max over channels), shift-aligned, amplified
    d = taps // 2
    g = gold[2 * d:, 2 * d:].astype(int)
    c = cv_naive[d:h - d, d:w - d].astype(int)
    dif = np.abs(g - c).max(axis=-1).astype(np.uint8)
    dif = np.clip(dif.astype(int) * 12, 0, 255).astype(np.uint8)   # amplify for visibility
    heat = cv2.applyColorMap(dif, cv2.COLORMAP_INFERNO)
    cv2.imwrite(str(OUT / f"{name}_diff_heat.png"), heat)


def run_conv(label, img, coeffs, shift, taps):
    w, h = img.shape[1], img.shape[0]
    pixels = to_flat(img)
    gold = to_np(G.conv_golden(pixels, w, coeffs, shift, 0, 1, taps), w, h)

    k = np.asarray(coeffs, dtype=np.float32).reshape(taps, taps)
    # naive: how a person writes it -- normalized float kernel, default border, uint8 rounding
    cv_naive = cv2.filter2D(img, ddepth=-1, kernel=k / float(1 << shift))
    # int-matched: same integer sum the RTL does, zero border, arithmetic floor shift, clamp.
    # (filter2D can't take int32 src; the sums are small ints so float64 holds them exactly.)
    accf = cv2.filter2D(img.astype(np.float64), ddepth=cv2.CV_64F,
                        kernel=k.astype(np.float64), borderType=cv2.BORDER_CONSTANT)
    acc = np.rint(accf).astype(np.int64)          # exact integer sum
    cv_int = np.clip(acc >> shift, 0, 255).astype(np.uint8)   # arithmetic floor >> like the RTL

    r = report(label, gold, cv_naive, cv_int, taps)
    save_gallery(label, img, gold, cv_naive, taps)
    return r


def run_median(label, img):
    w, h = img.shape[1], img.shape[0]
    pixels = to_flat(img)
    gold = to_np(G.prefilter_golden(pixels, w, 9, 0), w, h)   # op 9 = per-channel 3x3 median
    cv_med = cv2.medianBlur(img, 3)                            # exact median, reflect border
    # median has no arithmetic rounding -> "naive" and "int" are the same image here
    r = report(label, gold, cv_med, cv_med, 3)
    save_gallery(label, img, gold, cv_med, 3)
    return r


if __name__ == "__main__":
    pat_px, pw, ph = image_io.make_test_pattern()
    pattern = to_np(pat_px, pw, ph)
    natural = rand_image(96, 72, seed=0xC0FFEE)   # varied content -> exercises rounding

    g3 = C.CONV3_KERNELS["gaussian"][0]           # [1,2,1,2,4,2,1,2,1]
    g5 = C.CONV5_KERNELS["gaussian5"][0]          # 25-tap binomial

    print("################ OpenCV-as-oracle experiment ################")
    print("golden.py == RTL (bit-exact, proven). Numbers below are OpenCV-vs-RTL.")

    for tag, im in (("pattern", pattern), ("random", natural)):
        run_conv(f"{tag}_conv3x3_gauss", im, g3, 4, 3)
        run_conv(f"{tag}_conv5x5_gauss", im, g5, 8, 5)
        run_median(f"{tag}_median3x3", im)

    print(f"\nImages written to: {OUT}")
