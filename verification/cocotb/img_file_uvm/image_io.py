"""Stdlib-only image I/O for the img_file_uvm block.

The cocotb venv (MinGW ucrt64 python) has no Pillow/numpy, so everything the sim-side
test touches is stdlib: binary PPM (P6) / PGM (P5) read+write, and a minimal PNG writer
(zlib + struct, 8-bit RGB truecolor, filter 0). Arbitrary input formats are converted to
PPM beforehand by ``scripts/img_to_ppm.py`` running under the repo-root CPython venv
(which has Pillow) -- see test_img_file_uvm.prepare_input().

Pixels are flat row-major lists of 24-bit ints ``{R[23:16], G[15:8], B[7:0]}`` -- the
img_proc slot-contract lane order.
"""
from __future__ import annotations

import struct
import zlib
from pathlib import Path
from typing import List, Sequence, Tuple


def _read_ppm_tokens(data: bytes, count: int) -> Tuple[List[bytes], int]:
    """Read `count` whitespace-separated header tokens, skipping ``#`` comments.
    Returns (tokens, offset_of_first_raster_byte)."""
    tokens: List[bytes] = []
    i = 0
    while len(tokens) < count:
        if i >= len(data):
            raise ValueError("PPM/PGM header truncated")
        c = data[i:i + 1]
        if c in b" \t\r\n":
            i += 1
        elif c == b"#":
            while i < len(data) and data[i:i + 1] not in b"\r\n":
                i += 1
        else:
            j = i
            while j < len(data) and data[j:j + 1] not in b" \t\r\n":
                j += 1
            tokens.append(data[i:j])
            i = j
    # exactly one whitespace byte separates the header from the raster
    return tokens, i + 1


def read_ppm(path) -> Tuple[List[int], int, int]:
    """Read a binary PPM (P6) or PGM (P5, gray replicated to {Y,Y,Y}).
    Returns (pixels, width, height) with pixels as flat row-major 24-bit RGB ints."""
    data = Path(path).read_bytes()
    tokens, off = _read_ppm_tokens(data, 4)
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic not in (b"P6", b"P5"):
        raise ValueError(f"{path}: unsupported magic {magic!r} (need binary P6/P5)")
    if maxval != 255:
        raise ValueError(f"{path}: maxval {maxval} unsupported (need 255)")
    n = w * h
    if magic == b"P6":
        raster = data[off:off + 3 * n]
        if len(raster) < 3 * n:
            raise ValueError(f"{path}: raster truncated ({len(raster)} < {3 * n})")
        px = [(raster[3 * k] << 16) | (raster[3 * k + 1] << 8) | raster[3 * k + 2]
              for k in range(n)]
    else:
        raster = data[off:off + n]
        if len(raster) < n:
            raise ValueError(f"{path}: raster truncated ({len(raster)} < {n})")
        px = [(v << 16) | (v << 8) | v for v in raster]
    return px, w, h


def write_ppm(path, pixels: Sequence[int], width: int, height: int) -> None:
    raw = bytearray()
    for p in pixels[:width * height]:
        raw += bytes(((p >> 16) & 0xFF, (p >> 8) & 0xFF, p & 0xFF))
    Path(path).write_bytes(b"P6\n%d %d\n255\n" % (width, height) + bytes(raw))


def write_png(path, pixels: Sequence[int], width: int, height: int) -> None:
    """Minimal PNG writer: 8-bit RGB truecolor, filter 0 per scanline, one IDAT."""
    def chunk(tag: bytes, payload: bytes) -> bytes:
        return (struct.pack(">I", len(payload)) + tag + payload
                + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF))

    raw = bytearray()
    for r in range(height):
        raw.append(0)  # filter type 0 (None)
        for c in range(width):
            p = pixels[r * width + c]
            raw += bytes(((p >> 16) & 0xFF, (p >> 8) & 0xFF, p & 0xFF))
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    Path(path).write_bytes(b"\x89PNG\r\n\x1a\n"
                           + chunk(b"IHDR", ihdr)
                           + chunk(b"IDAT", zlib.compress(bytes(raw), 6))
                           + chunk(b"IEND", b""))


def decimate(pixels: Sequence[int], width: int, height: int,
             max_w: int, max_h: int) -> Tuple[List[int], int, int]:
    """Integer-stride subsample so the result fits within max_w x max_h (stdlib fallback
    for oversized direct-PPM input; the Pillow converter path resizes properly)."""
    stride = 1
    while width // stride > max_w or height // stride > max_h:
        stride += 1
    if stride == 1:
        return list(pixels), width, height
    nw, nh = width // stride, height // stride
    out = [pixels[(r * stride) * width + (c * stride)]
           for r in range(nh) for c in range(nw)]
    return out, nw, nh


def make_test_pattern(width: int = 64, height: int = 48) -> Tuple[List[int], int, int]:
    """Deterministic built-in pattern (no external file needed): vertical color bars
    overlaid with a horizontal luminance ramp plus a bright diagonal -- enough structure
    to exercise convolution kernels, thresholds and dither."""
    bars = [(255, 255, 255), (255, 255, 0), (0, 255, 255), (0, 255, 0),
            (255, 0, 255), (255, 0, 0), (0, 0, 255), (0, 0, 0)]
    px: List[int] = []
    for r in range(height):
        ramp = (r * 255) // max(1, height - 1)
        for c in range(width):
            br, bg, bb = bars[(c * len(bars)) // width]
            v_r = (br * ramp) // 255
            v_g = (bg * ramp) // 255
            v_b = (bb * ramp) // 255
            if abs((c * height) // width - r) <= 1:  # bright diagonal
                v_r, v_g, v_b = 255, 255, 255
            px.append((v_r << 16) | (v_g << 8) | v_b)
    return px, width, height
