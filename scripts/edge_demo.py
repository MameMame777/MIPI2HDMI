#!/usr/bin/env python3
"""Sobel edge-magnitude live demo (2026-06-25). Brings up the 30fps colour stream, starts
live HDMI, and cycles colour / single Sobel-X (op8, one direction+polarity) / omnidirectional
edge magnitude |Gx|+|Gy| (cam.edges, op12) / binarize->Sobel (bin_edges, pre_op=4) /
Sobel->binarize (edge_binary, post_op=4) / colour -- capturing a still per stage to
/home/xilinx/edge_<stage>.png. The omnidirectional 'edges' image should detect MORE edge
pixels than the single Sobel-X (both gradient polarities + both directions); 'edgebinary' is a
black/white edge map (~Canny stage 1). VDMA stopped on return/exit/signal; the script ends
naturally (do NOT kill it mid-run)."""
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


def save_still(bufs, tag: str):
    buf = np.array(bufs[1]).copy()
    bpp = STRIDE // WIDTH
    if bpp >= 3:
        px = buf.reshape(HEIGHT, WIDTH, bpp)
        frame = np.stack([px[:, :, 2], px[:, :, 1], px[:, :, 0]], axis=-1).astype(np.uint8)
        mode = 'RGB'
    else:
        frame = buf[:, :WIDTH]; mode = 'L'
    base = OUTDIR / f'edge_{tag}_{time.strftime("%H%M%S")}'
    np.save(base.with_suffix('.npy'), frame)
    try:
        from PIL import Image
        Image.fromarray(frame, mode).save(base.with_suffix('.png'))
    except Exception as e:
        print(f'   (png skipped: {e})')
    y = frame.mean(axis=2) if frame.ndim == 3 else frame
    edge_frac = float((y > 60).mean() * 100)        # "edge pixel" fraction (bright = edge)
    print(f'   captured {tag}: mean={y.mean():.1f} std={y.std():.1f} '
          f'edge%(>60)={edge_frac:.1f} -> {base}.png')
    return edge_frac


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--download', type=int, default=1)
    ap.add_argument('--full-init', type=int, default=1)
    ap.add_argument('--dwell', type=float, default=8.0)
    ap.add_argument('--testpattern', type=int, default=1,
                    help='1 = inject OV5640 test pattern (0x503D) so the input is known regardless '
                         'of the lens/scene. 0x80=colour bar (edges), 0x84=vertical gradient (dither).')
    ap.add_argument('--tp-val', type=lambda x: int(x, 0), default=0x80,
                    help='OV5640 0x503D test-pattern value (0x80 colour bar / 0x84 vgrad)')
    args, _ = ap.parse_known_args()

    install_vdma_cleanup_signals()
    ol, h = setup_session(download=bool(args.download))
    steps = fhs.patch_init_steps(list(fhs.FULL_INIT_STEPS), [(0x4800, 0x14)])
    v65.chip_init(h, steps, 'edge-init', settle_s=10.0)
    h['idelay_set'](16, 16)
    h['frame_lines_set_keep_cam'](value=480, use_lsle=True, expected_dt=0x22,
                                  sup_enable=False, sof_synth=True, force_expected=True)
    time.sleep(0.3)
    fhs.stream_cycle_write(h, list(fhs.ARM_REGS))
    h['set_hw_lock'](False); time.sleep(0.2)
    locked = (lock_mode(h, 12, settle_blank=14) == 0)
    h['set_settle_blank'](14); time.sleep(0.5)
    if args.testpattern:
        # OV5640 test pattern (lens-independent). 0x80=colour bar (edges), 0x84=vgrad (dither).
        fhs.stream_cycle_write(h, [(0x503D, args.tp_val)] + list(fhs.ARM_REGS))
        time.sleep(0.5)
        print(f'test pattern injected (0x503D=0x{args.tp_val:02X})')
    m = fhs.measure_link(h, dur=2.0, label='edge')
    print(f'lock={locked} fs={m["fs"]:.0f} fe={m["fe"]:.0f} crc={m["crc_err_pct"]:.1f}% '
          f'last_fe={m["last_frame_lines"]}')

    def reset_chain():
        # pre/post point ops + dither live on the 0xFE page, independent of set_proc_op -> clear
        # all at each stage so a previous bin_edges/edge_binary/dither does not leak into the next.
        h['set_pre_op'](0); h['set_post_op'](0); h['set_dither'](enable=False)
    def sobel_x():
        reset_chain(); h['set_conv_named']('sobel_x'); h['set_proc_op'](8)
    def edges():
        reset_chain(); h['set_edges']()
    def bin_edges():                          # binarize(green>128) THEN omnidirectional Sobel
        h['set_post_op'](0); h['set_pre_thresh'](128); h['set_pre_op'](4); h['set_edges'](2)
    def edge_binary():                        # omnidirectional Sobel THEN binarize (edge map)
        h['set_pre_op'](0); h['set_edges'](2); h['set_post_thresh'](64); h['set_post_op'](4)
    def median():                             # PRE 3x3 median denoise, conv passthrough (identity)
        h['set_post_op'](0); h['set_pre_op'](9); h['set_conv_named']('identity'); h['set_proc_op'](8)
    def denoise_edges():                      # PRE median denoise THEN omnidirectional Sobel
        h['set_post_op'](0); h['set_pre_op'](9); h['set_edges'](2)
    def halftone():                           # grayscale (POST) -> 1-bit ordered dither
        reset_chain(); h['set_proc_op'](0); h['set_post_op'](2); h['set_dither'](enable=True, mode='ordered', bits=1)
    def dither4():                            # colour -> 4-bit ordered dither (posterize / anti-band)
        reset_chain(); h['set_proc_op'](0); h['set_dither'](enable=True, mode='ordered', bits=4)
    def dither_rand():                        # colour -> 2-bit random (LFSR) dither
        reset_chain(); h['set_proc_op'](0); h['set_dither'](enable=True, mode='random', bits=2)
    stages = [
        ('colour',     lambda: (reset_chain(), h['set_proc_op'](0))),
        ('halftone',   halftone),           # gray -> 1-bit ordered dither
        ('dither4',    dither4),            # 4-bit ordered dither
        ('dither_rand',dither_rand),        # 2-bit random dither
        ('sobelx',     sobel_x),            # single 3x3, one direction + one polarity
        ('edges',      edges),              # omnidirectional |Gx|+|Gy| (default gain)
        ('median',     median),             # PRE 3x3 median denoise (spatial)
        ('denoise_edges', denoise_edges),   # median -> Sobel (cleaner edges)
        ('binedges',   bin_edges),          # 2値化 -> Sobel (contours of the binary regions)
        ('edgebinary', edge_binary),        # Sobel -> 2値化 (binary edge map, ~Canny stage 1)
        ('colour',     lambda: (reset_chain(), h['set_proc_op'](0))),
    ]

    d = ol.ip_dict['axi_vdma_0']
    vdma = MMIO(int(d['phys_addr']), int(d['addr_range']))
    bufs = [allocate(shape=(HEIGHT, STRIDE), dtype=np.uint8) for _ in range(3)]
    configure_vdma_s2mm(vdma, bufs, start_mm2s=True, start_s2mm=True)
    print('=== HDMI LIVE + cycling colour / Sobel-X / omnidirectional edges ===')
    ef = {}
    try:
        for tag, apply in stages:
            apply()
            print(f'  stage {tag} for {args.dwell:.0f}s')
            time.sleep(args.dwell * 0.6)
            ef[tag] = save_still(bufs, tag)
            time.sleep(args.dwell * 0.4)
        h['set_proc_op'](0)
    finally:
        stop_vdma(vdma)
        for b in bufs:
            if hasattr(b, 'freebuffer'):
                b.freebuffer()
    print(f'  edge%: sobelx={ef.get("sobelx",0):.1f}  omnidirectional edges={ef.get("edges",0):.1f} '
          f'(edges should be >= sobelx)')
    print(f'  chain: binedges(2値化->Sobel)={ef.get("binedges",0):.1f}  '
          f'edgebinary(Sobel->2値化)={ef.get("edgebinary",0):.1f}')
    print('done.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
