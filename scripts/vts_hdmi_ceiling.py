#!/usr/bin/env python3
"""Find the live-HDMI VTS ceiling (fps plan, 2026-06-21).

VTS=520 (32.5fps) froze live HDMI with S2MM DMAIntErr; VTS=1000 (17fps) is clean.
The VDMA is already fsync=2/genlock=2, so this is RATE-dependent, not a free-run
config issue. This sweeps VTS with MM2S(HDMI)+S2MM both running and reads the
S2MM VDMASR DMAIntErr bit (bit4) + checks whether S2MM actually overwrites the
0xAA prefill -- so the live-HDMI ceiling is found programmatically (no monitor
watching). The smallest VTS (highest fps) that stays DMAIntErr-free with the
buffer overwritten = the live-HDMI-safe fps to bake.

Runtime only on the current fsync=2 bitstream; no rebuild.

Run: deploy_banding_test.py --host 192.168.2.99 --script vts_hdmi_ceiling.py \
        --download 1 --full-init 0 --extra-args "--vts-list 1000,900,800,720,660,600,560,520"
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
                         stop_vdma, HEIGHT, STRIDE)
import frame_height_stability as fhs

S2MM_VDMASR = 0x34


def write_vts(h, vts: int, margin: int = 8) -> int:
    expo = max(16, vts - margin)
    h['sccb_write'](0x380E, (vts >> 8) & 0xFF)
    h['sccb_write'](0x380F, vts & 0xFF)
    for hi, lo in ((0x3A02, 0x3A03), (0x3A14, 0x3A15)):
        h['sccb_write'](hi, (expo >> 8) & 0xFF)
        h['sccb_write'](lo, expo & 0xFF)
    return expo


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--download', type=int, default=1)
    ap.add_argument('--vts-list', default='1000,900,800,720,660,600,560,520')
    ap.add_argument('--dwell', type=float, default=5.0,
                    help='HDMI-active seconds per VTS before reading VDMASR')
    args, _ = ap.parse_known_args()

    install_vdma_cleanup_signals()
    ol, h = setup_session(download=bool(args.download))
    print(f'chip ID = {(h["sccb_read"](0x300A) << 8) | h["sccb_read"](0x300B):04X}')

    # baseline: continuous init + RGB565 arm + settle-blank + HW lock (once)
    steps = fhs.patch_init_steps(list(fhs.FULL_INIT_STEPS), [(0x4800, 0x14)])
    v65.chip_init(h, steps, 'vtsceil-init', settle_s=10.0)
    h['idelay_set'](16, 16)
    h['frame_lines_set_keep_cam'](value=480, use_lsle=True, expected_dt=0x22,
                                  sup_enable=False, sof_synth=True, force_expected=True)
    time.sleep(0.3)
    fhs.stream_cycle_write(h, list(fhs.ARM_REGS))
    h['set_settle_blank'](8)
    time.sleep(1.5)
    h['bitslip_set'](0, 0)
    h['set_hw_lock'](True)
    t0 = time.time()
    while time.time() - t0 < 15:
        s = h['read_hwlock']()
        if s['locked']:
            print(f'  HW-locked ({s["p0"]},{s["p1"]}) hdr_active={s["hdr_active"]}')
            break
        if s['failed']:
            print('  *** lock FAILED ***')
            break
        time.sleep(0.4)

    d = ol.ip_dict['axi_vdma_0']
    vdma = MMIO(int(d['phys_addr']), int(d['addr_range']))
    bufs = [allocate(shape=(HEIGHT, STRIDE), dtype=np.uint8) for _ in range(3)]

    vts_list = [int(x) for x in args.vts_list.split(',') if x.strip()]
    print(f'\n=== live-HDMI VTS ceiling sweep {vts_list} (dwell {args.dwell}s, HDMI active) ===')
    rows = []
    for vts in vts_list:
        expo = write_vts(h, vts)
        time.sleep(1.0)
        for b in bufs:
            np.asarray(b).fill(0xAA)                    # prefill: frozen => stays 0xAA
        configure_vdma_s2mm(vdma, bufs, start_mm2s=True, start_s2mm=True)   # live HDMI
        time.sleep(args.dwell)                          # let DMAIntErr manifest
        sr = int(vdma.read(S2MM_VDMASR))
        dmaint = (sr >> 4) & 1
        m = fhs.measure_link(h, dur=2.0, label=f'vts{vts}')
        nonpre = int((np.array(bufs[1])[:, :STRIDE] != 0xAA).sum())  # S2MM overwrote prefill?
        stop_vdma(vdma)
        live_ok = (dmaint == 0) and (nonpre > 1000)
        rows.append(dict(vts=vts, expo=expo, fps=m['fe'], dmaint=dmaint,
                         nonpre=nonpre, ok=live_ok))
        print(f'  VTS={vts:4} fps={m["fe"]:5.1f} S2MM_VDMASR=0x{sr:08X} '
              f'DMAIntErr={dmaint} buf_written={nonpre>1000} '
              f'-> {"LIVE-OK" if live_ok else "FROZEN"}')
        time.sleep(0.5)

    write_vts(h, 1000)
    for b in bufs:
        if hasattr(b, 'freebuffer'):
            b.freebuffer()

    print('\n================= LIVE-HDMI CEILING =================')
    for r in rows:
        print(f'  VTS={r["vts"]:4} fps={r["fps"]:5.1f} DMAIntErr={r["dmaint"]} '
              f'{"LIVE-OK" if r["ok"] else "FROZEN"}')
    live = [r for r in rows if r['ok']]
    if live:
        best = min(live, key=lambda r: r['vts'])        # smallest VTS = max fps
        print(f'\nCEILING: VTS={best["vts"]} -> fps={best["fps"]:.1f} (DMAIntErr-free, '
              f'buffer written). AEC max-expo={best["expo"]}. '
              f'Bake this VTS for live-HDMI-safe fps.')
    else:
        print('\nNo VTS below 1000 stayed DMAIntErr-free -> live HDMI ceiling = 17fps (VTS=1000).')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
