#!/usr/bin/env python3
"""settle_blank K sweep for the 30fps (96MHz byte_clk) bitstream (2026-06-22).

The 30fps rebuild (chip PLL mult=96 -> link 384MHz -> byte_clk 96MHz) locks clean
(bitslip via software lock_mode, CRC=0, fs=fe=30) BUT the burst-head settle-blank
K=8 was tuned at the 17fps byte_clk (84MHz). settle_blank closes the SoT window
for K *byte_clk* cycles after an LP-exit; at 96MHz each cycle is shorter, so K=8
spans less real time and the burst-head settle garbage leaks back in -> short
frames (fe_b480) / bottom band. The fix is purely runtime (set_settle_blank).

This locks once then sweeps K, reporting the drop-insensitive ground truth
(max_last_fe_lines, fe_before_480) per K. The winning K is the smallest K that
gives max_last_fe=480 with fe_b480~0 (the 84MHz sweep was monotonic
K0=452..K8=480; expect the 96MHz optimum a few steps higher, ~K10-12).

Run: deploy_banding_test.py --host 192.168.2.99 --script settle_blank_sweep.py \
        --download 1 --full-init 1 --extra-args "--ks 8,10,11,12,13,14,15"
Point the camera at a lit, textured subject.
"""
from __future__ import annotations
import argparse, sys, time
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


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--download', type=int, default=1)
    ap.add_argument('--ks', default='8,10,11,12,13,14,15')
    args, _ = ap.parse_known_args()

    install_vdma_cleanup_signals()
    ol, h = setup_session(download=bool(args.download))
    steps = fhs.patch_init_steps(list(fhs.FULL_INIT_STEPS), [(0x4800, 0x14)])
    v65.chip_init(h, steps, 'sb-init', settle_s=10.0)   # full_init now bakes mult=96 (30fps)
    h['idelay_set'](16, 16)
    h['frame_lines_set_keep_cam'](value=480, use_lsle=True, expected_dt=0x22,
                                  sup_enable=False, sof_synth=True, force_expected=True)
    time.sleep(0.3)
    fhs.stream_cycle_write(h, list(fhs.ARM_REGS))
    h['set_settle_blank'](8)
    time.sleep(0.5)
    h['set_hw_lock'](False); time.sleep(0.2)
    locked = (lock_mode(h, 12) == 0)
    print(f'\n=== settle_blank K sweep @30fps (lock={locked}) ===')
    m0 = fhs.measure_link(h, dur=2.0, label='base')
    print(f'  base fps fe={m0["fe"]:.0f} crc={m0["crc_err_pct"]:.1f}%')

    d = ol.ip_dict['axi_vdma_0']
    vdma = MMIO(int(d['phys_addr']), int(d['addr_range']))
    bufs = [allocate(shape=(HEIGHT, STRIDE), dtype=np.uint8) for _ in range(3)]

    ks = [int(x, 0) for x in args.ks.split(',') if x.strip()]
    print(f'{"K":>3} {"max_last_fe":>11} {"fe_b480":>8} {"fe_a480":>8} {"fps":>5} {"crc%":>6} {"std":>6} {"wrap":>6}')
    rows = []
    for k in ks:
        h['set_settle_blank'](k); time.sleep(0.5)
        gt = fhs.groundtruth_lines(h, dur=3.0, label=f'K{k}')
        m = fhs.measure_link(h, dur=2.0, label=f'K{k}')
        for b in bufs: np.asarray(b).fill(0xAA)
        configure_vdma_s2mm(vdma, bufs, start_mm2s=False, start_s2mm=True)
        time.sleep(0.8)
        fr = np.array(bufs[1])[:, :WIDTH]
        stop_vdma(vdma)
        # bottom wrap detection: first row (from bottom) that is still ~0xAA prefill
        rowmean = fr.mean(axis=1)
        wrap = HEIGHT
        for r in range(HEIGHT - 1, -1, -1):
            if abs(rowmean[r] - 0xAA) > 8:
                wrap = r + 1; break
        rows.append(dict(k=k, mfe=gt['max_last_fe_lines'], b480=gt['fe_before_480'],
                         a480=gt['fe_after_480'], fps=m['fe'], crc=m['crc_err_pct'],
                         std=fr.std(), wrap=wrap))
        print(f'{k:>3} {gt["max_last_fe_lines"]:>11} {gt["fe_before_480"]:>8} '
              f'{gt["fe_after_480"]:>8} {m["fe"]:>5.0f} {m["crc_err_pct"]:>6.1f} '
              f'{fr.std():>6.1f} {wrap:>6}')

    for b in bufs:
        if hasattr(b, 'freebuffer'): b.freebuffer()

    print('\n=== SUMMARY (want: max_last_fe=480, fe_b480~0, wrap=480, fps~30) ===')
    good = [r for r in rows if r['mfe'] >= 480 and r['fps'] >= 28 and r['crc'] < 1.0]
    if good:
        best = min(good, key=lambda r: (r['b480'], r['k']))   # smallest clean K
        print(f'>>> WINNER K={best["k"]}: max_last_fe={best["mfe"]} fe_b480={best["b480"]} '
              f'fps={best["fps"]:.0f} crc={best["crc"]:.1f}% std={best["std"]:.0f} wrap={best["wrap"]}')
        print(f'    -> set camera_hdmi_demo/oneshot default --settle-blank {best["k"]} for 30fps; '
              f'then bake set_settle_blank default in RTL.')
    else:
        bp = max(rows, key=lambda r: r['mfe'])
        print(f'no K reaches 480. best max_last_fe={bp["mfe"]} at K={bp["k"]} '
              f'(fe_b480={bp["b480"]}) -- raise K range or investigate.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
