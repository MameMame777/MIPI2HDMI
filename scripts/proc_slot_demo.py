#!/usr/bin/env python3
"""Phase 2a processing-slot demo (2026-06-23). Drives live colour HDMI and cycles the
runtime cfg_proc_op (idelay[23:21]) so the slot's point ops are visible on the monitor:
passthrough -> invert -> grayscale -> BGR-swap -> threshold -> R/G/B only. Proves the
standardised AXI4-Stream processing-slot insertion + runtime control + verify flow
(Phase 2b swaps a 3x3 convolution into the same slot)."""
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

OPS = {0: 'passthrough', 1: 'invert', 2: 'grayscale', 3: 'BGR-swap',
       4: 'threshold', 5: 'R-only', 6: 'G-only', 7: 'B-only',
       8: 'conv-passthrough', 9: 'conv-Gaussian-blur', 10: 'conv-Sobel-edge',
       11: 'conv-sharpen'}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--download', type=int, default=1)
    ap.add_argument('--full-init', type=int, default=1)
    ap.add_argument('--ops', default='0,1,2,3,4,5,6,7')
    ap.add_argument('--dwell', type=float, default=8.0, help='seconds per op')
    args, _ = ap.parse_known_args()

    install_vdma_cleanup_signals()
    ol, h = setup_session(download=bool(args.download))
    steps = fhs.patch_init_steps(list(fhs.FULL_INIT_STEPS), [(0x4800, 0x14)])
    v65.chip_init(h, steps, 'proc-init', settle_s=10.0)
    h['idelay_set'](16, 16)
    h['frame_lines_set_keep_cam'](value=480, use_lsle=True, expected_dt=0x22,
                                  sup_enable=False, sof_synth=True, force_expected=True)
    time.sleep(0.3)
    fhs.stream_cycle_write(h, list(fhs.ARM_REGS))
    h['set_hw_lock'](False); time.sleep(0.2)
    locked = (lock_mode(h, 12, settle_blank=14) == 0)
    h['set_settle_blank'](14)
    time.sleep(0.5)
    m = fhs.measure_link(h, dur=2.0, label='proc')
    print(f'lock={locked} fs={m["fs"]:.0f} fe={m["fe"]:.0f} crc={m["crc_err_pct"]:.1f}%')

    d = ol.ip_dict['axi_vdma_0']
    vdma = MMIO(int(d['phys_addr']), int(d['addr_range']))
    bufs = [allocate(shape=(HEIGHT, STRIDE), dtype=np.uint8) for _ in range(3)]
    configure_vdma_s2mm(vdma, bufs, start_mm2s=True, start_s2mm=True)
    print('=== HDMI LIVE + cycling proc_op (watch the monitor) ===')
    ops = [int(x) for x in args.ops.split(',') if x.strip() != '']
    try:
        for op in ops:
            h['set_proc_op'](op)
            print(f'  proc_op={op} ({OPS.get(op, "?")})  for {args.dwell:.0f}s')
            time.sleep(args.dwell)
    finally:
        stop_vdma(vdma)
        for b in bufs:
            if hasattr(b, 'freebuffer'): b.freebuffer()
    print('done.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
