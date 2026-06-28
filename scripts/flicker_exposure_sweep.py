#!/usr/bin/env python3
"""Flicker root-cause / anti-banding calibration: manual-exposure sweep.

Confirmed (2026-06-04) that the live-scene horizontal banding is NOT an FPGA
frame-assembly artifact (the chip 0x503D test pattern comes through with
row_mean_stdev=0.00). The bands are mains-flicker (Tokyo 50 Hz -> 100 Hz)
rolling-shutter banding. The OV5640 banding filter IS enabled (0x3A00 bit5)
but the chip's auto-AEC locked exposure to a multiple of B60_step (60 Hz),
not B50_step (50 Hz), and it is unclear whether B50_step (295) matches the
true flicker period (observed image band period ~110 rows).

This script settles that empirically: it brings the pipeline up ONCE, then
sweeps the MANUAL exposure (0x3503=0x07 + 0x3500-02) across a range and, for
each value, captures a VDMA buffer and measures the banding strength
(max row-mean autocorrelation over candidate lags). Banding minimises when
exposure == k * (flicker period in exposure-line units); the spacing between
minima IS that period -> set B50_step to it for a permanent fix.

Point the camera at a BRIGHT, fairly uniform scene first (banding needs light;
a dark frame shows no signal).

Compliant with CLAUDE.md: uses pynq_bringup.setup_session() and
v65_capture.install_vdma_cleanup_signals().
"""
from __future__ import annotations
import sys
import time
import argparse
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from pynq import allocate, MMIO
from pynq_bringup import setup_session
import v65_capture as v65
from v65_capture import (
    install_vdma_cleanup_signals, configure_vdma_s2mm, stop_vdma,
    HEIGHT, STRIDE, CAM_GPIO_BIT,
)


def stream_cycle_write(h, regs):
    """Apply chip register writes inside a 0x300E stream off/on cycle so the
    sensor latches them (matches stage_15fps_drive's mechanism)."""
    h['sccb_write'](0x300E, 0x40); time.sleep(0.03)
    h['sccb_write'](0x4202, 0x0F); time.sleep(0.03)
    for a, v in regs:
        h['sccb_write'](a, v)
    h['sccb_write'](0x300E, 0x45); time.sleep(0.03)
    h['sccb_write'](0x4202, 0x00); time.sleep(0.3)


def set_exposure(h, exp_lines, gain=0xF8):
    e = int(exp_lines) & 0xFFFFF
    regs = [
        (0x3503, 0x07),                 # AEC + AGC manual
        (0x3500, (e >> 12) & 0x0F),
        (0x3501, (e >> 4) & 0xFF),
        (0x3502, (e << 4) & 0xF0),
        (0x350A, (gain >> 8) & 0x03),
        (0x350B, gain & 0xFF),
    ]
    stream_cycle_write(h, regs)


def grab(bufs):
    """Copy the 3 cycled buffers and return the one with the most signal."""
    arrs = [np.asarray(b).reshape(HEIGHT, STRIDE).copy() for b in bufs]
    # pick the buffer with the largest dynamic range (freshest live content)
    return max(arrs, key=lambda a: int(a.max()) - int(a.min()))


def banding_strength(img, lags):
    rm = img.mean(axis=1).astype(float)
    m = rm.mean()
    rstd = float(rm.std())                 # absolute row-mean stdev = flicker amplitude
    rmc = rm - m
    denom = float((rmc * rmc).sum()) or 1.0
    best_c, best_l = 0.0, 0
    for lag in lags:
        num = float((rmc[:-lag] * rmc[lag:]).sum())
        c = num / denom
        if c > best_c:
            best_c, best_l = c, lag
    return best_c, best_l, float(m), rstd


def main():
    install_vdma_cleanup_signals()
    ap = argparse.ArgumentParser()
    ap.add_argument('--exposures', type=str,
                    default='440,495,550,605,660,715,770,825,880,935,990')
    ap.add_argument('--settle', type=float, default=1.2,
                    help='seconds to wait after each exposure write before grab')
    ap.add_argument('--gain', type=lambda x: int(x, 0), default=0xF8,
                    help='manual gain (0x350A/0x350B), lower it to avoid '
                         'saturation on a bright uniform surface (e.g. 0x10=1x)')
    ap.add_argument('--download', type=int, default=0)
    args = ap.parse_args()

    ol, h = setup_session(download=bool(args.download), settle_s=(10.0 if args.download else 0.0),
                          raise_resetb=True)
    h['bitslip_set'](0, 6)
    h['idelay_set'](8, 8)
    h['frame_lines_set_keep_cam'](value=480, use_lsle=True, expected_dt=0x00)
    time.sleep(0.3)
    # 0x4800=0x34 (continuous clk + LS/LE + LP11 idle) via stream cycle
    stream_cycle_write(h, [(0x4800, 0x34)])
    time.sleep(2.0)

    d = ol.ip_dict['axi_vdma_0']
    vdma = MMIO(int(d['phys_addr']), int(d['addr_range']))
    bufs = [allocate(shape=(HEIGHT, STRIDE), dtype=np.uint8) for _ in range(3)]
    configure_vdma_s2mm(vdma, bufs, start_mm2s=False, start_s2mm=True)
    time.sleep(0.5)

    lags = list(range(60, 200))
    exposures = [int(x) for x in args.exposures.split(',') if x.strip()]
    print('\n exp_lines |  mean | row_stdev | band_ac | period(img rows)')
    print(' ----------+-------+-----------+---------+-----------------')
    results = []
    try:
        for e in exposures:
            set_exposure(h, e, gain=args.gain)
            time.sleep(args.settle)
            img = grab(bufs)
            c, l, m, rstd = banding_strength(img, lags)
            results.append((e, m, c, l, rstd))
            print('   %5d   | %5.1f |   %6.2f  |  %.3f  |   %3d' % (e, m, rstd, c, l))
    finally:
        stop_vdma(vdma)

    print('\nInterpretation (use row_stdev on a UNIFORM scene = pure flicker amplitude):')
    print(' - row_stdev MINIMA mark exposures that are integer multiples of the flicker')
    print('   period; the spacing between consecutive minima = period in EXPOSURE lines.')
    if results:
        bestmin = min(results, key=lambda r: r[4])
        print(' - lowest flicker (row_stdev) at exposure=%d lines (stdev %.2f, mean %.1f)' %
              (bestmin[0], bestmin[4], bestmin[1]))


if __name__ == '__main__':
    sys.exit(main())
