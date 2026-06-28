"""TPG HDMI live demo — cycles 3 patterns on HDMI output.

S2MM captures TPG frames to DDR; MM2S reads them out to the HDMI subsystem.
Patterns switch every HOLD_S seconds, cycling 0→1→2→0→... for TOTAL_S seconds.

Usage on PYNQ:
    python3 tpg_hdmi_demo.py [--hold 5] [--total 60] [--bit /path/to/bd_wrapper.bit]
"""
from __future__ import annotations
import argparse
import sys
import time
import numpy as np
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from pynq import MMIO, allocate
from pynq_bringup import setup_session
import v65_capture as v65
from v65_capture import install_vdma_cleanup_signals

WIDTH  = 640
HEIGHT = 480
N_BUFS = 4

APPLY_BIT         = 1 << 24
CAM_GPIO_BIT      = 1 << 25
TPG_RT_BIT        = 1 << 26
PATTERN_SEL_SHIFT = 27

PATTERN_NAMES = {0: 'VERT_RAMP', 1: 'HORIZ_RAMP', 2: 'CHECKER'}


def set_pattern(ol, pattern_sel: int) -> None:
    word = (CAM_GPIO_BIT
            | TPG_RT_BIT
            | ((pattern_sel & 0x3) << PATTERN_SEL_SHIFT)
            | 480)
    ol.frame_lines_gpio.channel1.write(word, 0xFFFFFFFF)
    time.sleep(0.005)
    ol.frame_lines_gpio.channel1.write(word | APPLY_BIT, 0xFFFFFFFF)
    time.sleep(0.01)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--bit',   default=None)
    ap.add_argument('--hold',  type=float, default=5.0, help='seconds per pattern')
    ap.add_argument('--total', type=float, default=60.0, help='total run time (s)')
    args = ap.parse_args()

    install_vdma_cleanup_signals()

    print('=== TPG HDMI demo: loading bitstream ===')
    ol, h = setup_session(bit_path=args.bit, settle_s=0.0, raise_resetb=False)

    h['frame_lines_set_keep_cam'](480, use_lsle=False, use_tpg=True)
    set_pattern(ol, 0)

    print('Waiting 12 s for SCCB init FSM ...')
    time.sleep(12.0)

    print(f'Allocating {N_BUFS} VDMA buffers {HEIGHT}x{WIDTH} ...')
    bufs = [allocate(shape=(HEIGHT, WIDTH), dtype=np.uint8) for _ in range(N_BUFS)]
    for b in bufs:
        b[:] = 0x55
        b.flush()

    vdma_desc = ol.ip_dict['axi_vdma_0']
    vdma = MMIO(int(vdma_desc['phys_addr']), int(vdma_desc['addr_range']))

    # Start S2MM + MM2S together — S2MM writes, MM2S reads 1 frame behind (FRMDLY_SHIFT)
    v65.configure_vdma_s2mm(vdma, bufs, start_mm2s=True, start_s2mm=True)
    time.sleep(0.5)   # prime: S2MM writes at least 1 frame before MM2S reads

    print(f'\nHDMI output running. Patterns will switch every {args.hold:.1f} s.')
    print(f'Total duration: {args.total:.0f} s  (Ctrl+C to stop)\n')

    t_start = time.time()
    pat     = 0
    t_pat   = t_start

    try:
        while True:
            now = time.time()
            if now - t_start >= args.total:
                break

            elapsed_pat = now - t_pat
            if elapsed_pat >= args.hold:
                pat     = (pat + 1) % 3
                t_pat   = now
                set_pattern(ol, pat)
                pname = PATTERN_NAMES[pat]
                print(f'  [{now - t_start:5.1f}s] Pattern {pat}: {pname}')

            time.sleep(0.2)

    except KeyboardInterrupt:
        print('\nInterrupted by user.')
    finally:
        v65.stop_vdma(vdma)
        for b in bufs:
            del b
        print('VDMA stopped. Done.')


if __name__ == '__main__':
    main()
