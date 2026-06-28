#!/usr/bin/env python3
"""One-shot camera capture (supervisor + SOF synth) -> single PNG/NPY.

Brings the OV5640 up, grabs ONE frame through the working path (sup_enable bit29
+ sof_synth bit30, RGB565->Y8, full 480 rows via FE-resync) and saves it to
/home/xilinx/pic_<timestamp>.{png,npy}. S2MM only (no MM2S => no VDMA halt).
deploy_banding_test.py pulls pic_* back to the local --pull-dir.

Run: deploy_banding_test.py --script oneshot_capture.py --download 1 \
        --full-init 1 --pull-dir picture
"""
from __future__ import annotations
import argparse
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from pynq import MMIO, allocate
from pynq_bringup import setup_session
import v65_capture as v65
from v65_capture import (install_vdma_cleanup_signals, configure_vdma_s2mm,
                         stop_vdma, HEIGHT, STRIDE)
import frame_height_stability as fhs
from bitslip_lock import lock_mode   # convergent bitslip-sweep + re-roll lock


def repair_edge_cols(img: np.ndarray, n: int) -> np.ndarray:
    """Edge-replicate the last `n` columns from the first good column before
    them. The OV5640 array's right edge columns are a sensor-side artifact
    (cols 637-639 spike/dip/cliff, ~30-60 LSB on the last column) that the
    FPGA pixel path faithfully captures (proven clean by the 0x503D test
    pattern, diary 2026-06-13). Keeps 640x480 geometry; only the dead edge
    columns are overwritten with their nearest valid neighbour."""
    if n > 0 and img.shape[1] > n:
        img[:, -n:] = img[:, -n - 1][:, None]
    return img


def detect_wrap_boundary(img: np.ndarray, prefill: int = 0xAA,
                         search_frac: float = 0.25,
                         step_thresh: float = 12.0, blk: int = 4) -> int:
    """Row index where the current frame ends and the bottom wrap begins
    (or HEIGHT if the whole buffer is one clean frame).

    The capture window (480) exceeds the delivered frame height (~432-448,
    jittering -- diary 2026-06-14 Phase 8), so the bottom rows hold either
    (B) 0xAA prefill or (A) next-frame content separated by a sharp, sustained
    ROW-MEAN STEP (the diary measured 89->55 / +13 jumps across the wrap).
    Only the bottom `search_frac` is examined and the step is measured between
    `blk`-row blocks (not single rows) so smooth gradients and 1-row spikes do
    not trigger. The wrap position jitters per grab, so this is detected
    dynamically -- the vertical analogue of repair_edge_cols.

    NOTE: a genuine horizontal scene edge in the bottom band also makes a
    row-mean step, so the caller guards this with the hardware-reported
    delivered height (only crop when a real frame-height shortfall exists)."""
    a = img.astype(np.float64)
    H = a.shape[0]
    lo = max(blk, int(H * (1.0 - search_frac)))

    # (B) 0xAA prefill onset: top of the contiguous all-prefill bottom block.
    pre = (np.abs(a - prefill) < 4).mean(axis=1) > 0.9
    boundary = H
    if pre[-1]:
        r = H
        while r > lo and pre[r - 1]:
            r -= 1
        boundary = r

    # (A) strongest sustained row-mean step within the search band, block-paired
    # so flat/gradient content stays below step_thresh.
    rm = a.mean(axis=1)
    best_i, best_d = H, 0.0
    for i in range(lo, max(lo, boundary - blk) + 1):
        d = abs(rm[i:i + blk].mean() - rm[i - blk:i].mean())
        if d > best_d:
            best_d, best_i = d, i
    if best_d >= step_thresh:
        boundary = min(boundary, best_i)
    return boundary


def repair_edge_rows(img: np.ndarray, boundary: int) -> np.ndarray:
    """Edge-replicate the last clean row down to the bottom, hiding the wrap/
    prefill rows below `boundary` while keeping 640x480 geometry (the vertical
    analogue of repair_edge_cols; geometry must stay 640x480 for HDMI/VTC
    parity). `boundary>=HEIGHT` is a no-op (clean frame)."""
    if 0 < boundary < img.shape[0]:
        img[boundary:] = img[boundary - 1][None, :]
    return img


def sharpness(img: np.ndarray) -> float:
    mask = (img != 0xAA).any(axis=1)
    f = img[mask].astype(np.float64)
    if f.shape[0] < 16:
        return 0.0
    lap = (-4.0 * f + np.roll(f, 1, 0) + np.roll(f, -1, 0)
           + np.roll(f, 1, 1) + np.roll(f, -1, 1))
    return float(lap[2:-2, 2:-2].var())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--download', type=int, default=1)
    ap.add_argument('--full-init', type=int, default=1)
    ap.add_argument('--val4800', type=lambda x: int(x, 0), default=0x14,
                    help='chip 0x4800 MIPI_CTRL00 patched into init. Default 0x14 = '
                         'continuous + line-sync: healthy fs=fe=30 constant-height stream '
                         '(SOF-synth + force-480) -> clean, untiled grabs. 0x24 = no-LS '
                         '(fs~2, unstable height); 0x34 = LS/LE short packets.')
    ap.add_argument('--vcm', type=int, default=-1,
                    help='optional VCM focus DAC code 0..1023 (-1 = leave default)')
    ap.add_argument('--crop-edge-cols', type=int, default=3,
                    help='edge-replicate the last N columns to hide the OV5640 '
                         'right-edge sensor artifact (cols 637-639). 0 = off.')
    ap.add_argument('--crop-edge-rows', type=int, default=-1,
                    help='hide the bottom frame-boundary WRAP (capture 480 > '
                         'delivered ~432-448, diary 2026-06-14 Phase 8) by '
                         'edge-replicating the last clean row down. -1 = auto-'
                         'detect the wrap boundary per grab (it jitters); '
                         'N>0 = fixed last-N rows; 0 = off. Interim band-aid '
                         'for STILL capture only -- live HDMI needs the RTL '
                         'deterministic-lock fix.')
    ap.add_argument('--gain-ceiling', type=lambda x: int(x, 0), default=-1,
                    help='cap AGC max gain (0x3A18/0x3A19, /16 format: 0x80=8x, '
                         '0x40=4x). Reduces low-light column-FPN amplification. '
                         '-1 = leave init/mainline 0x00F8 (15.5x). NOTE: divergence '
                         'from Linux mainline 0xF8; tradeoff = darker low light.')
    ap.add_argument('--val503d', type=lambda x: int(x, 0), default=-1,
                    help='optional OV5640 0x503D test pattern (0x84 vgrad, '
                         '0x80 color bar; -1 = sensor mode). For vertical-line '
                         'FPGA-vs-sensor diagnosis: a chip-generated pattern '
                         'bypasses the sensor array, so any fixed vertical line '
                         'that survives is in the FPGA pixel path, not the sensor.')
    ap.add_argument('--sup', type=int, default=1,
                    help='supervisor enable (bit29). 0 = legacy (BUFR always '
                         'released) -- the working path for continuous clock '
                         '(--val4800 0x14), diary 2026-06-14.')
    ap.add_argument('--synth', type=int, default=1,
                    help='SOF synth (bit30). With --sup 0 + 0x14 the real FS is '
                         'received so synth is usually not needed (--synth 0).')
    ap.add_argument('--lock-rerolls', type=int, default=8,
                    help='deterministic lock: after streaming, run the 8x8 '
                         'bitslip sweep + /4-phase re-roll-on-fail (the real lock '
                         'fix, 2026-06-15) instead of trusting a fixed bitslip. '
                         '0 = keep the legacy fixed bitslip(0,6).')
    ap.add_argument('--hw-lock', type=int, default=0,
                    help='use the HW deterministic-lock FSM (bitslip_word[25], E2 '
                         '2026-06-19) instead of the software lock_mode: the RTL '
                         'sweeps the 8x8 bitslip + /4 re-roll and HOLDs the lock on '
                         'its own. Continuous only (use --val4800 0x14 --sup 0). '
                         'Overrides --lock-rerolls. Status on debug page 0x2e.')
    ap.add_argument('--force-expected', type=int, default=0,
                    help='force-close each frame at exactly --value(480) lines '
                         '(bit31, 2026-06-16) for a constant height; with --synth 1 '
                         'also opens at the chip true top (FE-resync). 0 = off.')
    ap.add_argument('--long-as-line', type=int, default=0,
                    help='deliver a long whose LS was dropped as a row anyway '
                         '(idelay bit26, 2026-06-17) -> recovers the no-LS-reject '
                         'bottom band. Set AFTER lock. 0 = off.')
    ap.add_argument('--hs-settle-gate', type=int, default=0,
                    help='per-line HS-SETTLE SoT gate in the legacy continuous '
                         'path (frame_lines bit28, 2026-06-17): skip the HS-prepare '
                         'garbage at each burst head so the SoT search no longer '
                         'mis-locks -> recovers the >=16 line/frame frontend drop. '
                         '0 = off.')
    ap.add_argument('--settle-blank', type=int, default=14,
                    help='byte-domain per-line settle blank K (idelay[30:27], '
                         '2026-06-17): hold the SoT window closed K byte_clk after '
                         'each LP-exit to skip the burst-head settle garbage. K is '
                         'in byte_clk cycles -> K=8 was the band fix at the 17fps '
                         '84MHz byte_clk; the 30fps build (byte_clk 96MHz) needs '
                         'K=14 (sweep 2026-06-22). Set after lock. 0 = off (legacy).')
    args, _ = ap.parse_known_args()

    install_vdma_cleanup_signals()
    ol, h = setup_session(download=bool(args.download))
    cid = (h['sccb_read'](0x300A) << 8) | h['sccb_read'](0x300B)
    print(f'chip ID = {cid:04X} (expect 5640)')

    steps = fhs.patch_init_steps(list(fhs.FULL_INIT_STEPS), [(0x4800, args.val4800)])
    v65.chip_init(h, steps, 'oneshot-init', settle_s=10.0)
    h['bitslip_set'](0, 6)
    h['idelay_set'](8, 8)
    h['frame_lines_set_keep_cam'](value=480, use_lsle=(args.val4800 & 0x10 != 0),
                                  expected_dt=0x22, sup_enable=bool(args.sup),
                                  sof_synth=bool(args.synth),
                                  force_expected=bool(args.force_expected),
                                  hs_settle_gate=bool(args.hs_settle_gate))
    if args.vcm >= 0:
        c = args.vcm & 0x3FF
        h['sccb_write'](0x3603, (c >> 4) & 0x3F)
        h['sccb_write'](0x3602, ((c & 0xF) << 4) | 0x00)
        print(f'VCM focus code set to {c}')
    time.sleep(0.3)
    gc = ([(0x3A18, (args.gain_ceiling >> 8) & 0x03), (0x3A19, args.gain_ceiling & 0xFF)]
          if args.gain_ceiling >= 0 else [])
    arm = (([(0x503D, args.val503d)] if args.val503d >= 0 else [])
           + gc + list(fhs.ARM_REGS))
    fhs.stream_cycle_write(h, arm)
    if args.val503d >= 0:
        print(f'test pattern 0x503D=0x{args.val503d:02X} enabled '
              f'(vertical-line FPGA-vs-sensor diagnosis)')
    if args.gain_ceiling >= 0:
        print(f'AGC gain ceiling capped to 0x{args.gain_ceiling:04X} '
              f'(~{args.gain_ceiling / 16:.1f}x; mainline divergence to cut '
              f'low-light column FPN)')
    time.sleep(2.0)

    # Deterministic lock (2026-06-15): the per-/4-phase correct bitslip varies, so
    # a fixed bitslip is a lottery. Sweep the 8x8 bitslip keyed on long packets and
    # re-roll the /4 phase on failure; on success the eye idelay is centred + held.
    if args.hw_lock:
        print('--- HW deterministic-lock FSM (no software lock_mode) ---')
        h['idelay_set'](16, 16)              # eye-centre (the FSM holds bitslip, not idelay)
        h['bitslip_set'](0, 0)               # neutral GPIO (ignored once the FSM drives)
        h['set_hw_lock'](True)
        t0 = time.time()
        while time.time() - t0 < 15.0:
            s = h['read_hwlock']()
            if s['locked']:
                print(f'  HW-locked bitslip=({s["p0"]},{s["p1"]}) after {s["reroll"]} '
                      f're-roll(s) at t={time.time()-t0:.1f}s')
                break
            if s['failed']:
                print('  *** HW lock FSM FAILED; capture may be empty. Power-cycle '
                      'if long stays 0. ***')
                break
            time.sleep(0.4)
    elif args.lock_rerolls > 0:
        h['set_hw_lock'](False)              # inhibit the FSM (HWLOCK_DEFAULT_ON builds) so lock_mode's bitslip applies
        print('--- deterministic lock: bitslip sweep + re-roll-on-fail ---')
        if lock_mode(h, args.lock_rerolls) != 0:
            print('*** WARNING no clean lock found via bitslip sweep; '
                  'capture may be tiled/short. Power-cycle if long stays 0. ***')
    # long-as-line AFTER lock (idelay writes during lock clear bit26)
    h['set_long_as_line'](bool(args.long_as_line))
    if args.long_as_line:
        print('long-as-line ENABLED: no-LS longs delivered as rows (band fix)')
    # settle-blank AFTER lock (set_settle_blank is a no-apply level write)
    h['set_settle_blank'](int(args.settle_blank))
    if args.settle_blank:
        print(f'settle-blank K={args.settle_blank}: SoT search skips the burst-head '
              f'settle garbage (last_fe->480 band fix)')

    m = fhs.measure_link(h, dur=3.0, label='oneshot')
    if m['long_pkt'] < 100:
        print('*** WARNING long_pkt low — chip may be degraded; '
              'power-cycle the board and retry. ***')

    # Hardware-reported delivered frame height (drop-insensitive, page 0x1B
    # last_fe_lines). Used to GUARD the auto row-crop: only hide a bottom wrap
    # when the bridge confirms a real frame-height shortfall (<480), so a
    # genuine 480-row frame with a horizontal scene edge is never cut.
    # NOTE: page 0x1B is fed by the probe frame-assembler, which only counts
    # while in_frame, and in_frame opens on a REAL FS short packet. In sup mode
    # the per-gate re-lock drops the FS (fs=0) so last_fe_lines reads 0 -> the
    # hw metric is unavailable; fall back to the image wrap detector (the sup
    # working path always delivers <480, so a detected wrap is always real).
    gt = fhs.groundtruth_lines(h, dur=4.0, label='oneshot')
    fe_vals = {k: v for k, v in gt['fe_values'].items() if 16 < k <= HEIGHT}
    hw_h = max(fe_vals, key=fe_vals.get) if fe_vals else None     # modal height
    if hw_h is not None:
        print(f'delivered frame height (hw modal last_fe_lines) = {hw_h}/{HEIGHT}'
              f'  shortfall={hw_h < HEIGHT - 5}')
    else:
        print('hw last_fe_lines unavailable (sup mode FS loss) -> auto row-crop '
              'defers to the image wrap detector')

    dvdma = ol.ip_dict['axi_vdma_0']
    vdma = MMIO(int(dvdma['phys_addr']), int(dvdma['addr_range']))
    bufs = [allocate(shape=(HEIGHT, STRIDE), dtype=np.uint8) for _ in range(3)]
    for b in bufs:
        np.asarray(b).fill(0xAA)
    configure_vdma_s2mm(vdma, bufs, start_mm2s=False, start_s2mm=True)
    time.sleep(0.5)

    # A grab can land tiled: the VDMA S2MM with genlock_mode=2 expects the MM2S
    # read channel as its sync master, but a still capture runs S2MM only, so the
    # frame boundary lands at arbitrary offsets and the buffer is a mosaic of
    # fragments. A clean single frame has ~1 row-to-row discontinuity (the chip
    # frame boundary); a tiled grab has several. So over many grabs we pick the
    # one with the FEWEST tile seams (then the most written rows).
    def tile_seams(a):
        g = a.astype(np.float64)
        w = (a != 0xAA).any(axis=1)
        prev = prevd = None
        seams = 0
        for i in range(a.shape[0]):
            if not w[i]:
                continue
            r = g[i] - g[i].mean()
            d = float(np.sqrt((r * r).sum()))
            if prev is not None and d > 0 and prevd > 0:
                if float((prev * r).sum()) / (prevd * d) < 0.5:
                    seams += 1
            prev, prevd = r, d
        return seams

    best, best_key, best_rows, best_bnd = None, None, 0, HEIGHT
    for _ in range(6):
        fhs.vdma_restart(vdma, bufs)
        for a in fhs.refill_and_grab(bufs, 1.2):
            rows = int((a != 0xAA).any(axis=1).sum())
            bnd = detect_wrap_boundary(a)       # clean current-frame height
            # fewest seams, then the most CLEAN rows (highest wrap boundary)
            key = (-tile_seams(a), bnd, rows)
            if best_key is None or key > best_key:
                best, best_key, best_rows, best_bnd = a.copy(), key, rows, bnd
    print(f'selected grab: {best_rows} written rows, wrap boundary at '
          f'row {best_bnd}, {-best_key[0]} tile seams (min over grabs)')

    if args.crop_edge_cols > 0:
        best = repair_edge_cols(best, args.crop_edge_cols)
        print(f'edge-replicated last {args.crop_edge_cols} cols '
              f'(OV5640 right-edge sensor artifact)')

    if args.crop_edge_rows != 0:
        if args.crop_edge_rows > 0:                      # fixed last-N
            bnd = HEIGHT - args.crop_edge_rows
        elif hw_h is not None:                           # auto, hw-guarded
            bnd = best_bnd if hw_h < HEIGHT - 5 else HEIGHT
        else:                                            # auto, sup mode: image
            bnd = best_bnd
        if bnd < HEIGHT:
            best = repair_edge_rows(best, bnd)
            print(f'edge-replicated rows {bnd}..{HEIGHT - 1} '
                  f'(bottom frame-boundary wrap; '
                  f'{"auto" if args.crop_edge_rows < 0 else "fixed"})')
        else:
            print('no bottom wrap cropped (no visible wrap: smooth bottom, '
                  'no prefill — a static-scene wrap is invisible)')

    ts = time.strftime('%Y%m%d_%H%M%S')
    name = f'/home/xilinx/pic_{ts}'
    fhs.save_frame(best, name)
    print(f'SAVED {name}.png  written_rows={best_rows}/{HEIGHT} '
          f'long={m["long_pkt"]:.0f}/s sharpness={sharpness(best):.0f}')

    stop_vdma(vdma)
    for b in bufs:
        if hasattr(b, 'freebuffer'):
            b.freebuffer()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
