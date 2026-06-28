#!/usr/bin/env python3
"""Color RGBA32 capture diagnostic (2026-06-23). Grabs the VDMA S2MM buffer as
RGBA32 (1 px = 4 bytes, STRIDE=2560), reshapes (480,640,4), and reports per-byte
channel stats + cross-channel differences. If the channels differ (|bi-bj| large)
COLOR data reached DDR (any remaining gray is a display-path/byte-order issue); if
all channels are ~equal the capture path/chip/bitstream is still producing gray.
Also saves an RGB PNG (best-guess byte order [B,G,R,0])."""
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


def main() -> int:
    install_vdma_cleanup_signals()
    ol, h = setup_session(download=1)
    steps = fhs.patch_init_steps(list(fhs.FULL_INIT_STEPS), [(0x4800, 0x14)])
    v65.chip_init(h, steps, 'color-init', settle_s=10.0)
    h['idelay_set'](16, 16)
    h['frame_lines_set_keep_cam'](value=480, use_lsle=True, expected_dt=0x22,
                                  sup_enable=False, sof_synth=True, force_expected=True)
    time.sleep(0.3)
    fhs.stream_cycle_write(h, list(fhs.ARM_REGS))
    h['set_hw_lock'](False); time.sleep(0.2)
    locked = (lock_mode(h, 12, settle_blank=14) == 0)
    m = fhs.measure_link(h, dur=2.0, label='color')
    print(f'lock={locked} fs={m["fs"]:.0f} fe={m["fe"]:.0f} crc={m["crc_err_pct"]:.1f}% '
          f'pix/line={m.get("pix_per_line","?")}')
    print(f'STRIDE={STRIDE} (expect 2560 for RGBA32)')

    d = ol.ip_dict['axi_vdma_0']
    vdma = MMIO(int(d['phys_addr']), int(d['addr_range']))
    bufs = [allocate(shape=(HEIGHT, STRIDE), dtype=np.uint8) for _ in range(3)]
    for b in bufs: np.asarray(b).fill(0)
    configure_vdma_s2mm(vdma, bufs, start_mm2s=False, start_s2mm=True)
    time.sleep(1.0)
    fr = np.array(bufs[1])
    stop_vdma(vdma)

    px = fr.reshape(HEIGHT, WIDTH, 4).astype(int)
    print('\n=== per-byte-position channel stats (1px = 4 bytes) ===')
    for i in range(4):
        ch = px[:, :, i]
        print(f'  byte[{i}]: mean={ch.mean():6.1f} std={ch.std():6.1f} '
              f'min={ch.min():3d} max={ch.max():3d}')
    b0, b1, b2, b3 = px[:, :, 0], px[:, :, 1], px[:, :, 2], px[:, :, 3]
    d01, d12, d02 = np.abs(b0 - b1).mean(), np.abs(b1 - b2).mean(), np.abs(b0 - b2).mean()
    print(f'\n  cross-channel |b0-b1|={d01:.1f}  |b1-b2|={d12:.1f}  |b0-b2|={d02:.1f}')
    colorful = max(d01, d12, d02) > 4.0
    print(f'  >>> {"COLOR data present in DDR (channels differ) -> display/byte-order issue" if colorful else "GRAY in DDR (channels ~equal) -> capture path / chip / bitstream cache"}')

    try:
        from PIL import Image
        # best-guess: 32-bit word {0,R,G,B}=0x00RRGGBB -> LE bytes [B,G,R,0]
        rgb = np.stack([px[:, :, 2], px[:, :, 1], px[:, :, 0]], axis=-1).astype(np.uint8)
        name = f'/home/xilinx/picr_{time.strftime("%H%M%S")}.png'
        Image.fromarray(rgb).save(name)
        print(f'SAVED {name}  (RGB from bytes [2,1,0])')
    except Exception as e:
        print('png save failed:', e)

    for b in bufs:
        if hasattr(b, 'freebuffer'): b.freebuffer()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
