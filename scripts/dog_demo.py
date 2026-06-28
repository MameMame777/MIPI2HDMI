#!/usr/bin/env python3
"""DoG dual-kernel live demo (2026-06-24, plan_dog_dual_kernel_20260624 Phase B verify).
Brings up the 30fps colour stream, starts live HDMI, and cycles the op-12 Difference-of-
Gaussians dual-kernel (parallel 3x3 A + general 5x5 B, out = clamp(alpha*A - beta*B +
offset)) through a few presets -- colour passthrough / DoG 'blob' / DoG 'unsharp' / 5x5
blur (combiner mode B) -- capturing one still per stage to /home/xilinx/dog_<stage>.png
for pull-back. VDMA is stopped on return/exit/signal (sshd-hang guard); the script ends
naturally after the cycle (do NOT kill it mid-run)."""
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
                         HEIGHT, STRIDE, WIDTH, CONV_KERNELS, GAUSS5)
import frame_height_stability as fhs
from bitslip_lock import lock_mode

OUTDIR = Path('/home/xilinx')


def save_still(bufs, tag: str) -> None:
    """Grab the settled S2MM buffer -> dog_<tag>.{npy,png} (colour-aware RGBA32)."""
    buf = np.array(bufs[1]).copy()
    bpp = STRIDE // WIDTH
    if bpp >= 3:                                   # 32b {0,R,G,B} LE -> [B,G,R,0]
        px = buf.reshape(HEIGHT, WIDTH, bpp)
        frame = np.stack([px[:, :, 2], px[:, :, 1], px[:, :, 0]], axis=-1).astype(np.uint8)
        mode = 'RGB'
    else:
        frame = buf[:, :WIDTH]; mode = 'L'
    base = OUTDIR / f'dog_{tag}_{time.strftime("%H%M%S")}'
    np.save(base.with_suffix('.npy'), frame)
    try:
        from PIL import Image
        Image.fromarray(frame, mode).save(base.with_suffix('.png'))
    except Exception as e:
        print(f'   (png skipped: {e})')
    print(f'   captured {tag}: mean={frame.mean():.1f} std={frame.std():.1f} -> {base}.png')


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--download', type=int, default=1)
    ap.add_argument('--full-init', type=int, default=1)
    ap.add_argument('--dwell', type=float, default=8.0)
    args, _ = ap.parse_known_args()

    install_vdma_cleanup_signals()
    ol, h = setup_session(download=bool(args.download))

    # verified 30fps bring-up: full init + RGB565 arm + software lock + settle K=14
    steps = fhs.patch_init_steps(list(fhs.FULL_INIT_STEPS), [(0x4800, 0x14)])
    v65.chip_init(h, steps, 'dog-init', settle_s=10.0)
    h['idelay_set'](16, 16)
    h['frame_lines_set_keep_cam'](value=480, use_lsle=True, expected_dt=0x22,
                                  sup_enable=False, sof_synth=True, force_expected=True)
    time.sleep(0.3)
    fhs.stream_cycle_write(h, list(fhs.ARM_REGS))
    h['set_hw_lock'](False); time.sleep(0.2)
    locked = (lock_mode(h, 12, settle_blank=14) == 0)
    h['set_settle_blank'](14); time.sleep(0.5)
    m = fhs.measure_link(h, dur=2.0, label='dog')
    print(f'lock={locked} fs={m["fs"]:.0f} fe={m["fe"]:.0f} crc={m["crc_err_pct"]:.1f}% '
          f'last_fe={m["last_frame_lines"]}')

    # stage = (label, apply-fn). DoG presets via set_dog_named; 5x5 blur via combiner mode B.
    G3 = CONV_KERNELS['gaussian'][0]
    ID3 = CONV_KERNELS['identity'][0]
    stages = [
        ('colour',     lambda: h['set_proc_op'](0)),
        ('dog_blob',   lambda: h['set_dog_named']('blob')),       # G3 - G5 band-pass
        ('dog_unsharp', lambda: h['set_dog_named']('unsharp')),   # 2*id - G5 edge boost
        ('blur5x5',    lambda: h['set_dog'](ID3, 0, GAUSS5[0], 8, 1, 1, 0, 0, 1)),  # mode B
        ('colour',     lambda: h['set_proc_op'](0)),
    ]

    d = ol.ip_dict['axi_vdma_0']
    vdma = MMIO(int(d['phys_addr']), int(d['addr_range']))
    bufs = [allocate(shape=(HEIGHT, STRIDE), dtype=np.uint8) for _ in range(3)]
    configure_vdma_s2mm(vdma, bufs, start_mm2s=True, start_s2mm=True)
    print('=== HDMI LIVE + cycling op-12 DoG dual-kernel (watch the monitor) ===')
    try:
        for tag, apply in stages:
            apply()
            print(f'  stage {tag} for {args.dwell:.0f}s')
            time.sleep(args.dwell * 0.6)
            save_still(bufs, tag)
            time.sleep(args.dwell * 0.4)
        h['set_proc_op'](0)
    finally:
        stop_vdma(vdma)
        for b in bufs:
            if hasattr(b, 'freebuffer'):
                b.freebuffer()
    print('done.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
