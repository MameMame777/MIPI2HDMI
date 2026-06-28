#!/usr/bin/env python3
"""Convert a raw VDMA capture (HEIGHT x STRIDE_BYTES, uint8) into 3 viewable PNGs:

  *_y.png   : extract Y plane assuming UYVY YUV422 layout
  *_bytes.png : reinterpret as grayscale, 1 byte/pixel (stride 1280 -> 1280px wide)
  *_raw.png : raw stride untouched (1280 wide, useful to spot byte-vs-pixel issues)

Usage:
    python3 raw_to_png.py captures/bpp2_test/bpp2_test_buf0.raw [-W 640 -H 480 --bpp 2]
"""
from __future__ import annotations
import argparse
import os
import numpy as np
from PIL import Image


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('raw_path')
    ap.add_argument('-W', '--width', type=int, default=640)
    ap.add_argument('-H', '--height', type=int, default=480)
    ap.add_argument('--bpp', type=int, default=1, choices=[1, 2],
                    help='bytes per buffer column. Current FPGA pipeline outputs '
                         'Y8 to VDMA (axis_video_bridge TDATA_WIDTH=8), so default 1. '
                         'Use 2 only if you have a UYVY/RGB565 dump captured with HSIZE=1280.')
    args = ap.parse_args()

    stride_bytes = args.width * args.bpp
    expected = args.height * stride_bytes
    data = np.fromfile(args.raw_path, dtype=np.uint8)
    if data.size != expected:
        print(f'WARNING: file size {data.size} != expected {expected} '
              f'({args.height}x{stride_bytes})')
        data = data[:expected] if data.size > expected else np.pad(data, (0, expected - data.size))

    arr2d = data.reshape(args.height, stride_bytes)
    base = os.path.splitext(args.raw_path)[0]

    Image.fromarray(arr2d).save(f'{base}_bytes.png')
    print(f'  -> {base}_bytes.png   shape={arr2d.shape}')

    if args.bpp == 2:
        # UYVY: byte order U Y V Y -> Y at odd byte positions
        y_uyvy = arr2d[:, 1::2]
        Image.fromarray(y_uyvy).save(f'{base}_y_uyvy.png')
        print(f'  -> {base}_y_uyvy.png  shape={y_uyvy.shape}  (Y from UYVY)')

        # YUYV: byte order Y U Y V -> Y at even byte positions
        y_yuyv = arr2d[:, 0::2]
        Image.fromarray(y_yuyv).save(f'{base}_y_yuyv.png')
        print(f'  -> {base}_y_yuyv.png  shape={y_yuyv.shape}  (Y from YUYV)')

    # Histogram-ish stats per row to spot stride mismatch
    row_means = arr2d.mean(axis=1)
    row_var = arr2d.var(axis=1)
    print(f'  row mean range = [{row_means.min():.1f}, {row_means.max():.1f}]')
    print(f'  row var  range = [{row_var.min():.1f}, {row_var.max():.1f}]')
    print(f'  rows with var<1 (flat) = {int((row_var < 1).sum())}/{args.height}')


if __name__ == '__main__':
    main()
