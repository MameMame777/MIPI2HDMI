#!/usr/bin/env python3
"""30fps IDELAY-tuning lock test (2026-06-22, fps plan).

pll_sweep proved that at mult=94 + mipi_div=4 (0x3035=0x14) the chip streams a
real image at fs=30/s with std~70 and NO x=135 column (full vblank -> proper
30fps quality), but the link did not hold a clean lock (long=0, fe=0). byte_clk
at this config is ~73MHz (LOWER than the 17fps 84MHz, because mipi_div=4 halves
the lane rate to ~585Mbps) so it is NOT a timing/ISERDES ceiling -- the most
likely cause is that the fixed IDELAY=16 (tuned at the 17fps lane rate) is
off-centre in the bit eye at the new lane rate, so every bitslip sees bit
errors and long packets are rejected.

This holds the 30fps PLL (mult=94, mipi_div=4, VTS=984) and sweeps the IDELAY
tap; for each tap it runs the SOFTWARE lock_mode (which scores bitslips by long
count, so it only reports success when real long packets flow) and measures
long/s, CRC%, fs/fe, plus a capture's std + the x=135 column. A tap with
long>0 / crc~0 / fe~30 is a clean RUNTIME 30fps lock (no rebuild). If no tap is
clean, the lane eye is genuinely marginal on this bitstream -> rebuild.

Run: deploy_banding_test.py --host 192.168.2.99 --script pll30_idelay_lock.py \
        --download 1 --full-init 1 \
        --extra-args "--mult 0x5e --m35 0x14 --vts 984 --idelays 2,6,10,14,18,22,26,30"
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


def set_pll(h, mult, vts, m35):
    # stream-cycle is required for PLL changes (chip re-evaluates on 0x300E edge)
    h['sccb_write'](0x300E, 0x40); h['sccb_write'](0x4202, 0x0F); time.sleep(0.1)
    h['sccb_write'](0x3035, m35)   # sysdiv[7:4]|mipi_div[3:0]; 0x14 = sysdiv1/mipi_div4 -> lane halved, PCLK unchanged
    h['sccb_write'](0x3036, mult)
    h['sccb_write'](0x380E, (vts >> 8) & 0xFF)
    h['sccb_write'](0x380F, vts & 0xFF)
    h['sccb_write'](0x300E, 0x45); h['sccb_write'](0x4202, 0x00); time.sleep(2.0)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--download', type=int, default=1)
    ap.add_argument('--mult', type=lambda x: int(x, 0), default=0x5e)   # 94 -> ~30fps
    ap.add_argument('--m35', type=lambda x: int(x, 0), default=0x14)    # mipi_div=4
    ap.add_argument('--vts', type=int, default=984)
    ap.add_argument('--idelays', default='2,6,10,14,18,22,26,30')
    args, _ = ap.parse_known_args()

    install_vdma_cleanup_signals()
    ol, h = setup_session(download=bool(args.download))
    steps = fhs.patch_init_steps(list(fhs.FULL_INIT_STEPS), [(0x4800, 0x14)])
    v65.chip_init(h, steps, 'pll30-init', settle_s=10.0)
    h['idelay_set'](16, 16)
    h['frame_lines_set_keep_cam'](value=480, use_lsle=True, expected_dt=0x22,
                                  sup_enable=False, sof_synth=True, force_expected=True)
    time.sleep(0.3)
    fhs.stream_cycle_write(h, list(fhs.ARM_REGS))
    h['set_settle_blank'](8)
    time.sleep(1.0)

    # set the 30fps PLL ONCE (single chip write -> minimal degradation risk)
    set_pll(h, args.mult, args.vts, args.m35)
    print(f'\n=== 30fps IDELAY sweep: mult={args.mult:#x} m35={args.m35:#x} '
          f'(mipi_div={args.m35 & 0xF}) VTS={args.vts} ===')

    d = ol.ip_dict['axi_vdma_0']
    vdma = MMIO(int(d['phys_addr']), int(d['addr_range']))
    bufs = [allocate(shape=(HEIGHT, STRIDE), dtype=np.uint8) for _ in range(3)]

    taps = [int(x, 0) for x in args.idelays.split(',') if x.strip()]
    print(f'{"idelay":>6} {"fps":>6} {"crc%":>6} {"long/s":>9} {"lock":>6} {"std":>6} {"x135dip":>8}')
    rows = []
    for t in taps:
        h['set_hw_lock'](False); time.sleep(0.1)
        h['idelay_set'](t, t); time.sleep(0.2)
        locked = (lock_mode(h, 8) == 0)
        m = fhs.measure_link(h, dur=3.0, label=f'idelay{t}')
        for b in bufs: np.asarray(b).fill(0xAA)
        configure_vdma_s2mm(vdma, bufs, start_mm2s=False, start_s2mm=True)
        time.sleep(0.8)
        fr = np.array(bufs[1])[:, :WIDTH]
        stop_vdma(vdma)
        col = fr.mean(axis=0); med = np.median(col)
        dip = med - col[120:160].min()
        rows.append(dict(t=t, fps=m['fe'], crc=m['crc_err_pct'], longs=m['long_pkt'],
                         lock=locked, std=fr.std(), dip=dip))
        print(f'{t:>6} {m["fe"]:>6.1f} {m["crc_err_pct"]:>6.1f} {m["long_pkt"]:>9.0f} '
              f'{str(locked):>6} {fr.std():>6.1f} {dip:>8.0f}')
        time.sleep(0.2)

    for b in bufs:
        if hasattr(b, 'freebuffer'): b.freebuffer()

    print('\n=== SUMMARY (target: fps~30, crc~0, long>0, std high, x135dip small) ===')
    clean = [r for r in rows if r['crc'] < 1.0 and r['longs'] > 1000 and r['fps'] >= 25]
    if clean:
        best = max(clean, key=lambda r: (r['longs'], r['fps']))
        print(f'>>> CLEAN 30fps at IDELAY={best["t"]}: fps={best["fps"]:.1f} '
              f'crc={best["crc"]:.1f}% long={best["longs"]:.0f}/s std={best["std"]:.0f} '
              f'x135dip={best["dip"]:.0f}')
        print(f'    -> runtime 30fps WORKS. Drive HDMI with this idelay, then bake '
              f'(PLL + idelay default) for zero-PYNQ 30fps.')
    else:
        # report the best partial (highest long) to see how close
        bp = max(rows, key=lambda r: r['longs'])
        print(f'no fully-clean tap. best long: IDELAY={bp["t"]} long={bp["longs"]:.0f}/s '
              f'crc={bp["crc"]:.1f}% fps={bp["fps"]:.1f} std={bp["std"]:.0f}')
        print('    -> if best long is still ~0, the lane eye is marginal on this '
              'bitstream at the 30fps rate -> needs a rebuild (XDC re-constrained).')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
