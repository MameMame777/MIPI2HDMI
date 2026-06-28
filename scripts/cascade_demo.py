#!/usr/bin/env python3
"""Cascade variable-blur live demo (2026-06-24, plan_cascade_multiscale_20260624 Phase B/C).
Brings up the 30fps colour stream, starts live HDMI, and cycles the runtime-variable
Gaussian blur via the 3-stage cascade -- colour / blur 5x5 (op13) / blur 9x9 (op14) /
blur 13x13 (op15) / colour -- capturing one still per stage to /home/xilinx/casc_<stage>.png.
The std of the captured frame should DROP as the effective kernel grows (more smoothing).
VDMA stopped on return/exit/signal; the script ends naturally (do NOT kill it mid-run)."""
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
from v65_capture import (install_vdma_cleanup_signals, configure_vdma_s2mm, stop_vdma,
                         HEIGHT, STRIDE, WIDTH)
import frame_height_stability as fhs
from bitslip_lock import lock_mode

OUTDIR = Path('/home/xilinx')


def save_still(bufs, tag: str) -> float:
    buf = np.array(bufs[1]).copy()
    bpp = STRIDE // WIDTH
    if bpp >= 3:
        px = buf.reshape(HEIGHT, WIDTH, bpp)
        frame = np.stack([px[:, :, 2], px[:, :, 1], px[:, :, 0]], axis=-1).astype(np.uint8)
        mode = 'RGB'
    else:
        frame = buf[:, :WIDTH]; mode = 'L'
    base = OUTDIR / f'casc_{tag}_{time.strftime("%H%M%S")}'
    np.save(base.with_suffix('.npy'), frame)
    try:
        from PIL import Image
        Image.fromarray(frame, mode).save(base.with_suffix('.png'))
    except Exception as e:
        print(f'   (png skipped: {e})')
    print(f'   captured {tag}: mean={frame.mean():.1f} std={frame.std():.1f} -> {base}.png')
    return float(frame.std())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--download', type=int, default=1)
    ap.add_argument('--full-init', type=int, default=1)
    ap.add_argument('--dwell', type=float, default=8.0)
    args, _ = ap.parse_known_args()

    install_vdma_cleanup_signals()
    ol, h = setup_session(download=bool(args.download))
    steps = fhs.patch_init_steps(list(fhs.FULL_INIT_STEPS), [(0x4800, 0x14)])
    v65.chip_init(h, steps, 'casc-init', settle_s=10.0)
    h['idelay_set'](16, 16)
    h['frame_lines_set_keep_cam'](value=480, use_lsle=True, expected_dt=0x22,
                                  sup_enable=False, sof_synth=True, force_expected=True)
    time.sleep(0.3)
    fhs.stream_cycle_write(h, list(fhs.ARM_REGS))
    h['set_hw_lock'](False); time.sleep(0.2)
    locked = (lock_mode(h, 12, settle_blank=14) == 0)
    h['set_settle_blank'](14); time.sleep(0.5)
    m = fhs.measure_link(h, dur=2.0, label='casc')
    print(f'lock={locked} fs={m["fs"]:.0f} fe={m["fe"]:.0f} crc={m["crc_err_pct"]:.1f}% '
          f'last_fe={m["last_frame_lines"]}')

    stages = [
        ('colour', lambda: h['set_proc_op'](0)),
        ('blur5',  lambda: h['set_blur'](5)),    # op13 eff 5x5
        ('blur9',  lambda: h['set_blur'](9)),    # op14 eff 9x9
        ('blur13', lambda: h['set_blur'](13)),   # op15 eff 13x13
        ('colour', lambda: h['set_proc_op'](0)),
    ]

    d = ol.ip_dict['axi_vdma_0']
    vdma = MMIO(int(d['phys_addr']), int(d['addr_range']))
    bufs = [allocate(shape=(HEIGHT, STRIDE), dtype=np.uint8) for _ in range(3)]
    configure_vdma_s2mm(vdma, bufs, start_mm2s=True, start_s2mm=True)
    print('=== HDMI LIVE + cycling cascade variable blur (watch the monitor) ===')
    stds = {}
    try:
        for tag, apply in stages:
            apply()
            print(f'  stage {tag} for {args.dwell:.0f}s')
            time.sleep(args.dwell * 0.6)
            stds[tag] = save_still(bufs, tag)
            time.sleep(args.dwell * 0.4)
        h['set_proc_op'](0)
    finally:
        stop_vdma(vdma)
        for b in bufs:
            if hasattr(b, 'freebuffer'):
                b.freebuffer()
    # blur should monotonically reduce std (more smoothing with larger kernel)
    print(f'  std: colour~{stds.get("colour",0):.1f} blur5={stds.get("blur5",0):.1f} '
          f'blur9={stds.get("blur9",0):.1f} blur13={stds.get("blur13",0):.1f} '
          f'(expect decreasing)')
    print('done.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
