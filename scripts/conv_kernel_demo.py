#!/usr/bin/env python3
"""Phase 2b programmable-kernel demo (2026-06-23). Drives live colour HDMI and loads a
sequence of RUNTIME-PROGRAMMABLE 3x3 kernels (no rebuild) -- passthrough, Gaussian blur,
Sobel-X edge, sharpen, emboss -- via set_conv_kernel (SCCB reserved-addr 0xFE0i) +
set_proc_op(8) (conv mode). Proves arbitrary 3x3 coefficients are loadable live."""
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

# (name, coeffs or None=point passthrough, shift)
KERNELS = [
    ('passthrough',   None,                          0),
    ('Gaussian blur', [1, 2, 1, 2, 4, 2, 1, 2, 1],   4),
    ('Sobel-X edge',  [-1, 0, 1, -2, 0, 2, -1, 0, 1], 0),
    ('sharpen',       [0, -1, 0, -1, 5, -1, 0, -1, 0], 0),
    ('emboss',        [-2, -1, 0, -1, 1, 1, 0, 1, 2],  0),
    ('passthrough',   None,                          0),
]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--download', type=int, default=1)
    ap.add_argument('--full-init', type=int, default=1)
    ap.add_argument('--dwell', type=float, default=10.0)
    args, _ = ap.parse_known_args()

    install_vdma_cleanup_signals()
    ol, h = setup_session(download=bool(args.download))
    steps = fhs.patch_init_steps(list(fhs.FULL_INIT_STEPS), [(0x4800, 0x14)])
    v65.chip_init(h, steps, 'conv-init', settle_s=10.0)
    h['idelay_set'](16, 16)
    h['frame_lines_set_keep_cam'](value=480, use_lsle=True, expected_dt=0x22,
                                  sup_enable=False, sof_synth=True, force_expected=True)
    time.sleep(0.3)
    fhs.stream_cycle_write(h, list(fhs.ARM_REGS))
    h['set_hw_lock'](False); time.sleep(0.2)
    locked = (lock_mode(h, 12, settle_blank=14) == 0)
    h['set_settle_blank'](14)
    time.sleep(0.5)
    m = fhs.measure_link(h, dur=2.0, label='conv')
    print(f'lock={locked} fs={m["fs"]:.0f} fe={m["fe"]:.0f} crc={m["crc_err_pct"]:.1f}%')

    d = ol.ip_dict['axi_vdma_0']
    vdma = MMIO(int(d['phys_addr']), int(d['addr_range']))
    bufs = [allocate(shape=(HEIGHT, STRIDE), dtype=np.uint8) for _ in range(3)]
    configure_vdma_s2mm(vdma, bufs, start_mm2s=True, start_s2mm=True)
    print('=== HDMI LIVE + cycling programmable 3x3 kernels (watch the monitor) ===')
    try:
        for name, coeffs, shift in KERNELS:
            if coeffs is None:
                h['set_proc_op'](0)               # point path passthrough
            else:
                h['set_conv_kernel'](coeffs, shift)
                h['set_proc_op'](8)               # conv mode
            print(f'  {name:14s} coeffs={coeffs} >>{shift}  for {args.dwell:.0f}s')
            time.sleep(args.dwell)
    finally:
        stop_vdma(vdma)
        for b in bufs:
            if hasattr(b, 'freebuffer'): b.freebuffer()
    print('done.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
