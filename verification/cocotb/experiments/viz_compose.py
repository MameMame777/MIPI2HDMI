"""Build labeled composite figures: input / RTL golden / OpenCV / |diff| heatmap, for
conv3x3 gaussian and 3x3 median on the built-in 64x48 pattern.

Dev-only (needs numpy + opencv-python-headless + Pillow in the repo-root CPython venv).
Regenerates docs/doc/samples/img_file_uvm/opencv_vs_rtl_{conv3x3,median}.png, referenced by
docs/doc/image_file_verification.md. See README.md.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "verification/cocotb/img_file_uvm"))
import golden as G          # noqa: E402
import image_io            # noqa: E402
import img_config as C     # noqa: E402

DOCS = REPO / "docs/doc/samples/img_file_uvm"
DOCS.mkdir(parents=True, exist_ok=True)

SCALE = 5          # nearest-neighbour upscale for on-screen visibility
GAP = 12
LABEL_H = 22


def to_np(pixels, w, h):
    a = np.asarray(pixels, dtype=np.uint32).reshape(h, w)
    return np.stack([(a >> 16) & 0xFF, (a >> 8) & 0xFF, a & 0xFF], axis=-1).astype(np.uint8)


def to_flat(img):
    p = ((img[..., 0].astype(np.uint32) << 16)
         | (img[..., 1].astype(np.uint32) << 8) | img[..., 2].astype(np.uint32))
    return p.reshape(-1).tolist()


def _font():
    try:
        return ImageFont.truetype("arial.ttf", 15)
    except Exception:
        return ImageFont.load_default()


def _panel(arr_rgb, label, w, h):
    """Upscale an HxWx3 RGB array and add a label strip on top -> PIL.Image."""
    im = Image.fromarray(arr_rgb, "RGB").resize((w * SCALE, h * SCALE), Image.NEAREST)
    canvas = Image.new("RGB", (w * SCALE, h * SCALE + LABEL_H), (255, 255, 255))
    canvas.paste(im, (0, LABEL_H))
    d = ImageDraw.Draw(canvas)
    d.text((3, 3), label, fill=(0, 0, 0), font=_font())
    return canvas


def compose(name, panels, w, h, caption):
    pw, ph = w * SCALE, h * SCALE + LABEL_H
    total_w = pw * len(panels) + GAP * (len(panels) - 1)
    cap_h = 26
    fig = Image.new("RGB", (total_w, ph + cap_h), (255, 255, 255))
    x = 0
    for arr, lab in panels:
        fig.paste(_panel(arr, lab, w, h), (x, 0))
        x += pw + GAP
    ImageDraw.Draw(fig).text((3, ph + 5), caption, fill=(40, 40, 40), font=_font())
    out = DOCS / name
    fig.save(out)
    print(f"wrote {out}  ({fig.width}x{fig.height})")


def diff_heat(gold, cvimg, taps, amp):
    """Shift-aligned |gold - cv| (max over channels) over the FULL overlap, amplified, INFERNO,
    placed at the golden output coordinates. Shows both the interior residual AND the top/left
    border fringe (top/left d rows/cols have no shift-partner -> left black)."""
    h, w, _ = gold.shape
    d = taps // 2
    g = gold[d:, d:].astype(int)
    c = cvimg[0:h - d, 0:w - d].astype(int)
    dif = np.abs(g - c).max(axis=-1)
    amp8 = np.clip(dif * amp, 0, 255).astype(np.uint8)
    heat = cv2.applyColorMap(amp8, cv2.COLORMAP_INFERNO)          # BGR
    heat = cv2.cvtColor(heat, cv2.COLOR_BGR2RGB)
    full = np.zeros((h, w, 3), np.uint8)
    full[d:, d:] = heat
    return full


pat_px, w, h = image_io.make_test_pattern()
pat = to_np(pat_px, w, h)

# --- conv3x3 gaussian ---
g3 = C.CONV3_KERNELS["gaussian"][0]
gold3 = to_np(G.conv_golden(pat_px, w, g3, 4, 0, 1, 3), w, h)
k3 = np.asarray(g3, np.float32).reshape(3, 3)
cv3 = cv2.filter2D(pat, -1, k3 / 16.0)      # naive: float kernel, reflect border, round
compose("opencv_vs_rtl_conv3x3.png",
        [(pat, "input"), (gold3, "RTL golden"), (cv3, "OpenCV (naive)"),
         (diff_heat(gold3, cv3, 3, 48), "|RTL - OpenCV| x48")],
        w, h,
        "conv3x3 gaussian: interior 48% of px differ (all by exactly +/-1, cv2 rounds vs RTL floors); "
        "top/left fringe = border convention (RTL zero-init line buffers). Visually identical, bit-different.")

# --- median 3x3 ---
goldm = to_np(G.prefilter_golden(pat_px, w, 9, 0), w, h)
cvm = cv2.medianBlur(pat, 3)
compose("opencv_vs_rtl_median.png",
        [(pat, "input"), (goldm, "RTL golden"), (cvm, "OpenCV medianBlur"),
         (diff_heat(goldm, cvm, 3, 1), "|RTL - OpenCV| x1")],
        w, h,
        "median 3x3: interior is EXACT (no rounding) -> the diff panel is black almost everywhere. "
        "Only the top/left border differs (padding convention); on a natural image that flip reaches 255 LSB.")
print("done")
