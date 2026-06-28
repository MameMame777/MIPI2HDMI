#!/usr/bin/env python3
"""Objective focus/contrast probe (2026-06-21).

The real-camera capture is flat (std~7, sharpness low) while the chip 0x503D test
pattern is sharp (std~57) -> the sensor pixel-array/lens path produces a blurry
low-contrast image even with a real subject. This captures a frame at a sweep of
VCM focus DAC codes and reports per-code std (contrast) + sharpness (sum of
gradient variance) + mean, so we can OBJECTIVELY tell whether ANY focus position
yields a sharp/contrasty image (=focus issue, find the code) or all codes stay
flat (=subject too close / sensor / exposure issue, not focus). Saves the best
(highest-sharpness) frame's npy/png (pulled to _capture/ by the deploy wrapper).

Run: deploy_banding_test.py --host 192.168.2.99 --script focus_probe.py \
        --download 1 --full-init 1 --extra-args "--codes 0,128,256,384,512,640,768,896,1023"
Point the camera at a real, lit, textured subject >=30cm away during the run.
"""
from __future__ import annotations
import argparse
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from pynq import MMIO, allocate
from pynq_bringup import setup_session
import v65_capture as v65
from v65_capture import (install_vdma_cleanup_signals, configure_vdma_s2mm,
                         stop_vdma, HEIGHT, STRIDE, WIDTH)
import frame_height_stability as fhs
from bitslip_lock import lock_mode

S2MM_VDMASR = 0x34


def vcm_set(h, code: int) -> None:
    code &= 0x3FF
    h['sccb_write'](0x3603, (code >> 4) & 0x3F)
    h['sccb_write'](0x3602, (code & 0xF) << 4)


def sharpness(im: np.ndarray) -> float:
    a = im.astype(np.int32)
    gx = np.diff(a, axis=1)
    gy = np.diff(a, axis=0)
    return float(gx.var() + gy.var())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--download', type=int, default=1)
    ap.add_argument('--codes', default='0,128,256,384,512,640,768,896,1023')
    args, _ = ap.parse_known_args()

    install_vdma_cleanup_signals()
    ol, h = setup_session(download=bool(args.download))
    print(f'chip ID = {(h["sccb_read"](0x300A) << 8) | h["sccb_read"](0x300B):04X}')

    steps = fhs.patch_init_steps(list(fhs.FULL_INIT_STEPS), [(0x4800, 0x14)])
    v65.chip_init(h, steps, 'focus-init', settle_s=10.0)
    h['idelay_set'](16, 16)
    h['frame_lines_set_keep_cam'](value=480, use_lsle=True, expected_dt=0x22,
                                  sup_enable=False, sof_synth=True, force_expected=True)
    time.sleep(0.3)
    fhs.stream_cycle_write(h, list(fhs.ARM_REGS))
    h['set_settle_blank'](8)
    time.sleep(1.5)
    h['set_hw_lock'](False)
    time.sleep(0.3)
    if lock_mode(h, 8) != 0:
        print('*** no clean lock; results may be invalid ***')

    d = ol.ip_dict['axi_vdma_0']
    vdma = MMIO(int(d['phys_addr']), int(d['addr_range']))
    bufs = [allocate(shape=(HEIGHT, STRIDE), dtype=np.uint8) for _ in range(3)]

    codes = [int(x) for x in args.codes.split(',') if x.strip()]
    print(f'\n=== focus/contrast probe: codes {codes} ===')
    print(f'{"vcm":>5} {"mean":>6} {"std":>6} {"min":>4} {"max":>4} {"sharpness":>10}')
    best = None
    best_frame = None
    for c in codes:
        vcm_set(h, c)
        time.sleep(1.2)                       # VCM settle + a few frames
        for b in bufs:
            np.asarray(b).fill(0xAA)
        configure_vdma_s2mm(vdma, bufs, start_mm2s=False, start_s2mm=True)
        time.sleep(1.2)
        fr = np.array(bufs[1])[:, :WIDTH].copy()
        stop_vdma(vdma)
        sp = sharpness(fr)
        print(f'{c:>5} {fr.mean():6.1f} {fr.std():6.1f} {int(fr.min()):>4} '
              f'{int(fr.max()):>4} {sp:10.1f}')
        if best is None or sp > best[1]:
            best = (c, sp)
            best_frame = fr
        time.sleep(0.3)

    for b in bufs:
        if hasattr(b, 'freebuffer'):
            b.freebuffer()

    print(f'\nBEST sharpness at vcm={best[0]} (sharpness={best[1]:.1f}).')
    print('Interpretation: if BEST std/sharpness is much higher than the others, '
          'focus fixes it (bake that vcm). If ALL codes stay flat (std~7), it is '
          'NOT focus -> subject too close / lighting / sensor analog.')
    # save the best frame
    try:
        from PIL import Image
        jup = Path('/home/xilinx/jupyter_notebooks')
        outdir = (jup if jup.is_dir() else HERE) / '_capture'
        outdir.mkdir(parents=True, exist_ok=True)
        ts = time.strftime('pic_focusbest_%Y%m%d_%H%M%S')
        np.save((outdir / f'{ts}.npy').resolve(), best_frame)
        Image.fromarray(best_frame, 'L').save((outdir / f'{ts}.png').resolve())
        print(f'SAVED best frame -> {ts}.png (vcm={best[0]})')
    except Exception as e:
        print(f'(save skipped: {e})')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
