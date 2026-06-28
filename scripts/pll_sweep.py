#!/usr/bin/env python3
"""PLL mult sweep for PROPER 30fps (2026-06-22, fps plan).

The right way to 30fps is to raise the pixel clock (PLL), NOT cut VTS -- so each
frame keeps full vblank (no rate-dependent x=135 column line) and full exposure
(no darkening). This holds VTS=984 (standard full vblank) and sweeps the PLL
multiplier (0x3036), which raises both PCLK (fps) and the MIPI lane rate. Per
memory the D-PHY survives runtime mult up to ~105; this measures, per mult, the
actual fps (fe), D-PHY re-lock, CRC%, and a capture's std + the x=135 column,
to find the highest CLEAN fps. If the FPGA byte-clk can't keep up at some mult,
CRC rises -> that's the ceiling (then a rebuild with the XDC re-constrained to
the higher HS clock would be needed).

Run: deploy_banding_test.py --host 192.168.2.99 --script pll_sweep.py \
        --download 1 --full-init 1 --extra-args "--mults 0x36,0x42,0x4e,0x58,0x5e,0x64 --vts 984"
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


def set_pll(h, mult, vts, m35=None):
    # stream-cycle is required for PLL changes (chip re-evaluates on 0x300E edge)
    h['sccb_write'](0x300E, 0x40); h['sccb_write'](0x4202, 0x0F); time.sleep(0.1)
    if m35 is not None:
        h['sccb_write'](0x3035, m35)   # 0x3035 = sysdiv[7:4] | mipi_div[3:0]; raise mipi_div to lower the lane rate while PCLK (fps) is unchanged
    h['sccb_write'](0x3036, mult)
    h['sccb_write'](0x380E, (vts >> 8) & 0xFF)
    h['sccb_write'](0x380F, vts & 0xFF)
    h['sccb_write'](0x300E, 0x45); h['sccb_write'](0x4202, 0x00); time.sleep(2.0)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--download', type=int, default=1)
    ap.add_argument('--mults', default='0x36,0x42,0x4e,0x58,0x5e,0x64')
    ap.add_argument('--m35', type=lambda x: int(x, 0), default=None,
                    help='0x3035 value (sysdiv|mipi_div). 0x14 = sysdiv1/mipi_div4 '
                         '(lane rate halved vs default 0x12 -> keeps lane within ISERDES '
                         'while PCLK/fps tracks mult).')
    ap.add_argument('--vts', type=int, default=984)
    args, _ = ap.parse_known_args()

    install_vdma_cleanup_signals()
    ol, h = setup_session(download=bool(args.download))
    steps = fhs.patch_init_steps(list(fhs.FULL_INIT_STEPS), [(0x4800, 0x14)])
    v65.chip_init(h, steps, 'pll-init', settle_s=10.0)
    h['idelay_set'](16, 16)
    h['frame_lines_set_keep_cam'](value=480, use_lsle=True, expected_dt=0x22,
                                  sup_enable=False, sof_synth=True, force_expected=True)
    time.sleep(0.3)
    fhs.stream_cycle_write(h, list(fhs.ARM_REGS))
    h['set_settle_blank'](8)
    time.sleep(1.0)

    d = ol.ip_dict['axi_vdma_0']
    vdma = MMIO(int(d['phys_addr']), int(d['addr_range']))
    bufs = [allocate(shape=(HEIGHT, STRIDE), dtype=np.uint8) for _ in range(3)]

    mults = [int(x, 0) for x in args.mults.split(',') if x.strip()]
    print(f'\n=== PLL mult sweep (VTS={args.vts} full-vblank) ===')
    print(f'{"mult":>5} {"fps":>6} {"crc%":>6} {"long/s":>8} {"lock":>10} {"std":>6} {"x135dip":>8}')
    rows = []
    for m in mults:
        set_pll(h, m, args.vts, m35=args.m35)
        h['set_hw_lock'](False); time.sleep(0.3)
        locked = (lock_mode(h, 8) == 0)
        m_link = fhs.measure_link(h, dur=4.0, label=f'mult{m:#x}')
        # capture for std + x=135 dip
        for b in bufs: np.asarray(b).fill(0xAA)
        configure_vdma_s2mm(vdma, bufs, start_mm2s=False, start_s2mm=True)
        time.sleep(1.0)
        fr = np.array(bufs[1])[:, :WIDTH]
        stop_vdma(vdma)
        col = fr.mean(axis=0); med = np.median(col)
        x135dip = med - col[120:160].min()
        rows.append(dict(mult=m, fps=m_link['fe'], crc=m_link['crc_err_pct'],
                         longs=m_link['long_pkt'], lock=locked, std=fr.std(), dip=x135dip))
        print(f'{m:>#5x} {m_link["fe"]:>6.1f} {m_link["crc_err_pct"]:>6.1f} '
              f'{m_link["long_pkt"]:>8.0f} {str(locked):>10} {fr.std():>6.1f} {x135dip:>8.0f}')
        time.sleep(0.3)

    for b in bufs:
        if hasattr(b, 'freebuffer'): b.freebuffer()

    print('\n=== SUMMARY (target: fps>=30, crc~0, lock, x135dip small) ===')
    clean = [r for r in rows if r['crc'] < 1.0 and r['lock'] and r['fps'] >= 5]
    if clean:
        best = max(clean, key=lambda r: r['fps'])
        print(f'highest CLEAN fps: mult={best["mult"]:#x} -> {best["fps"]:.1f} fps, '
              f'crc={best["crc"]:.1f}%, std={best["std"]:.0f}, x135dip={best["dip"]:.0f}')
        if best['fps'] >= 28:
            print(f'  >>> ~30fps CLEAN reachable via PLL mult={best["mult"]:#x} (runtime). '
                  f'Bake it + recheck timing/XDC for the higher HS clock.')
        else:
            print(f'  highest clean fps is {best["fps"]:.1f} -- above this CRC rises '
                  f'(FPGA byte-clk ceiling) -> rebuild with XDC re-constrained for 30fps.')
    else:
        print('no clean config -- inspect CRC/lock per mult above.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
