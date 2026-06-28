#!/usr/bin/env python3
"""AEC state probe (2026-06-21): is the sensor actually using long exposure +
high gain? "Needs way too much light (only a direct bulb shows)" suggests the
effective sensitivity is far too low -- maybe the AEC is NOT ramping exposure/
gain. This inits + locks, lets the AEC settle, then reads back the ACTUAL
exposure (0x3500-2, in lines), AGC gain (0x350A/B), AEC mode (0x3503), banding/
night (0x3A00), VTS (0x380E/F), and the AEC max-exposure/max-band caps, and
prints them in human units alongside a captured-frame mean/std. If exposure ~=
the max (~600 lines) AND gain ~= 15.5x, the room is genuinely too dark. If
exposure is short / gain low, the AEC isn't using the available range (a bug).
"""
from __future__ import annotations
import sys, time
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


def rd(h, a):
    return h['sccb_read'](a) or 0


def main() -> int:
    install_vdma_cleanup_signals()
    ol, h = setup_session(download=True)
    steps = fhs.patch_init_steps(list(fhs.FULL_INIT_STEPS), [(0x4800, 0x14)])
    v65.chip_init(h, steps, 'aec-init', settle_s=10.0)
    h['idelay_set'](16, 16)
    h['frame_lines_set_keep_cam'](value=480, use_lsle=True, expected_dt=0x22,
                                  sup_enable=False, sof_synth=True, force_expected=True)
    time.sleep(0.3)
    fhs.stream_cycle_write(h, list(fhs.ARM_REGS))
    h['set_settle_blank'](8)
    time.sleep(1.5)
    h['set_hw_lock'](False); time.sleep(0.3)
    lock_mode(h, 8)
    print('letting AEC settle 4s...')
    time.sleep(4.0)

    expo_raw = (rd(h, 0x3500) << 16) | (rd(h, 0x3501) << 8) | rd(h, 0x3502)
    expo_lines = (expo_raw >> 4) / 1.0           # 0x3500-2 is exposure in 1/16 line
    gain_raw = ((rd(h, 0x350A) & 0x03) << 8) | rd(h, 0x350B)
    gain_x = gain_raw / 16.0
    vts = (rd(h, 0x380E) << 8) | rd(h, 0x380F)
    maxexpo = (rd(h, 0x3A02) << 8) | rd(h, 0x3A03)
    print('\n===== AEC STATE =====')
    print(f'  AEC mode 0x3503   = 0x{rd(h,0x3503):02X}  (0x00=auto exp+gain)')
    print(f'  exposure 0x3500-2 = {expo_lines:.0f} lines  (max-expo cap 0x3A02/3 = {maxexpo})')
    print(f'  AGC gain 0x350A/B = {gain_x:.2f}x  (ceiling 0x3A18/9 = {((rd(h,0x3A18)&3)<<8|rd(h,0x3A19))/16.0:.1f}x)')
    print(f'  max_band50 0x3A0E = {rd(h,0x3A0E)}   max_band60 0x3A0D = {rd(h,0x3A0D)}')
    print(f'  AEC ctrl 0x3A00   = 0x{rd(h,0x3A00):02X}  VTS 0x380E/F = {vts}')
    print(f'  exposure {expo_lines:.0f}/{vts} lines = {100*expo_lines/vts:.0f}% of frame')

    # capture mean/std
    d = ol.ip_dict['axi_vdma_0']
    vdma = MMIO(int(d['phys_addr']), int(d['addr_range']))
    bufs = [allocate(shape=(HEIGHT, STRIDE), dtype=np.uint8) for _ in range(3)]
    for b in bufs:
        np.asarray(b).fill(0xAA)
    configure_vdma_s2mm(vdma, bufs, start_mm2s=False, start_s2mm=True)
    time.sleep(1.2)
    fr = np.array(bufs[1])[:, :WIDTH]
    stop_vdma(vdma)
    print(f'  captured frame: mean={fr.mean():.1f} std={fr.std():.1f} min={fr.min()} max={fr.max()}')
    for b in bufs:
        if hasattr(b, 'freebuffer'):
            b.freebuffer()

    print('\n===== VERDICT =====')
    if expo_lines < 0.5 * vts and gain_x < 8:
        print('  exposure AND gain are LOW -> AEC is NOT ramping -> config bug (fixable).')
    elif expo_lines >= 0.7 * (maxexpo if maxexpo else vts) and gain_x >= 12:
        print('  exposure AND gain are near MAX -> sensor at full sensitivity -> room is genuinely too dark (physics).')
    else:
        print('  intermediate -> AEC is regulating; mean is the AEC target. Check if the')
        print('  scene contrast (std) is just low (flat/dim) or detail is present.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
