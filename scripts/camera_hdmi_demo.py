#!/usr/bin/env python3
"""Live camera -> HDMI output (supervisor + SOF synthesis).

S2MM captures camera frames (sup_enable bit29 + sof_synth bit30, fs=0 path) into
DDR; MM2S reads them back to the HDMI TX one frame behind (FRMDLY_SHIFT). The
camera image appears on the monitor connected to the Zybo for --total seconds.

Run via: deploy_banding_test.py --script camera_hdmi_demo.py --reboot
         --download 1 --full-init 1 --extra-args "--total 180"
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


def vcm_set(h, code: int) -> None:
    """Set the OV5640 VCM focus DAC (10-bit). 0x3603[5:0]=D[9:4] (bit7=PD),
    0x3602[7:4]=D[3:0], slew=direct."""
    code &= 0x3FF
    h['sccb_write'](0x3603, (code >> 4) & 0x3F)
    h['sccb_write'](0x3602, ((code & 0xF) << 4) | 0x00)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--download', type=int, default=1)
    ap.add_argument('--full-init', type=int, default=1)
    ap.add_argument('--total', type=float, default=180.0, help='HDMI run seconds')
    # Default 0x4800=0x14 (continuous + line-sync): the healthy fs=fe=30, constant-
    # height non-rolling stream (SOF-synth + force-480) verified band-free 2026-06-17,
    # with the settle-blank band fix (set after lock). This is the working live path.
    # (0x24 = no-LS makes the frame height unstable, fs~2. 0x34 = LS/LE short packets.)
    ap.add_argument('--val4800', type=lambda x: int(x, 0), default=0x14)
    ap.add_argument('--sup', type=int, default=0, help='supervisor enable (bit29); '
                    '0 = continuous-legacy (the verified band-fix path)')
    ap.add_argument('--synth', type=int, default=1, help='SOF synth (bit30)')
    ap.add_argument('--vcm-sweep', type=int, default=1,
                    help='1 = step the VCM focus DAC through codes during the '
                         'live output so you can watch the monitor for the sharp '
                         'position; 0 = leave focus untouched')
    ap.add_argument('--vcm-step-s', type=float, default=8.0,
                    help='seconds to hold each VCM focus code')
    ap.add_argument('--vcm', type=int, default=-1,
                    help='fixed VCM focus DAC code 0..1023 (-1 = leave default)')
    ap.add_argument('--val503d', type=lambda x: int(x, 0), default=-1,
                    help='optional OV5640 0x503D test pattern (0x84 vgrad; '
                         '-1 = sensor). Drives a chip-generated pattern through '
                         'the full MM2S->HDMI display path to tell whether fixed '
                         'vertical noise is sensor FPN (vanishes) or display-path '
                         '(persists).')
    ap.add_argument('--gain-ceiling', type=lambda x: int(x, 0), default=-1,
                    help='cap AGC max gain (0x3A18/0x3A19, /16: 0x80=8x, 0x40=4x) '
                         'to reduce low-light column-FPN amplification. -1 = leave '
                         'mainline 0x00F8 (15.5x). Tradeoff: darker low light.')
    ap.add_argument('--lock-rerolls', type=int, default=8,
                    help='deterministic lock: after streaming, run the 8x8 bitslip '
                         'sweep + /4-phase re-roll-on-fail (the real lock fix, '
                         '2026-06-15) before starting HDMI. 0 = fixed bitslip(0,6).')
    ap.add_argument('--hw-lock', type=int, default=0,
                    help='use the HW deterministic-lock FSM (bitslip_word[25], E2 '
                         '2026-06-19) instead of the software lock_mode: the RTL '
                         'sweeps the 8x8 bitslip + /4 re-roll and HOLDs the lock on '
                         'its own (power-on auto-lock). Continuous only. Overrides '
                         '--lock-rerolls. Status on debug page 0x2e. Default 0 for '
                         'the 30fps (link 384MHz/byte_clk 96MHz) build: the FSM '
                         'hdr_ok window is byte_clk-based and tuned at 84MHz, so it '
                         'BOGUS-locks (fs=0, white screen) at 96MHz -- software '
                         'lock_mode scores by long packets and locks clean. (For the '
                         '17fps build use --hw-lock 1; FSM re-tune for 96MHz is TODO.)')
    ap.add_argument('--force-expected', type=int, default=1,
                    help='force-close each frame at exactly --value(480) lines '
                         '(bit31) for a constant-height VTC stream -> genlock '
                         'locks, stopping the live-HDMI roll (2026-06-16). '
                         'Default 1 (verified non-rolling); 0 = variable (rolls).')
    ap.add_argument('--long-as-line', type=int, default=0,
                    help='deliver a long whose LS was dropped as a row anyway '
                         '(idelay bit26, 2026-06-17) -> recovers the no-LS-reject '
                         'bottom band. 0 = off.')
    ap.add_argument('--hs-settle-gate', type=int, default=0,
                    help='per-line HS-SETTLE SoT gate in the legacy continuous '
                         'path (frame_lines bit28, 2026-06-17) -> recovers the '
                         '>=16 line/frame frontend drop (the bottom band). 0 = off.')
    ap.add_argument('--settle-blank', type=int, default=14,
                    help='byte-domain per-line settle blank K (idelay[30:27], '
                         '2026-06-17): hold the SoT window closed K byte_clk after '
                         'each LP-exit to skip the burst-head settle garbage. K is '
                         'in byte_clk cycles so the optimum scales with byte_clk: '
                         'K=8 -> last_fe=480/480 at the 17fps 84MHz byte_clk; the '
                         '30fps build runs byte_clk=96MHz so K=14 is the new band '
                         'fix (sweep 2026-06-22: K>=13 -> max_last_fe=480, err none). '
                         'Set after lock.')
    ap.add_argument('--clk-settle', type=int, default=0,
                    help='gated only: supervisor clock-lane settle count set AT INIT '
                         '(bitslip_word[23:17]). Shorter (e.g. 8) starts byte_clk '
                         'earlier after the vblank clock-restart -> recovers the '
                         'leading lines -> stable sof_synth open (480/480), removing '
                         'the FS-loss framing jitter. 0 = build-time default (~20).')
    # processing chain (PRE denoise/point -> MID conv -> POST point), applied live before
    # the HDMI loop so the MONITOR shows the processed stream. e.g. median->Sobel->threshold:
    #   --pre median --mid edges --post thresh --post-thresh 64
    ap.add_argument('--pre', default='off',
                    help='PRE stage: off/invert/gray/bgr/thresh/r/g/b/gaussian/median (3x3 denoise 8/9)')
    ap.add_argument('--mid', default='none',
                    help="MID conv: none/edges/<CONV_KERNELS name e.g. sobel_x/gaussian/sharpen/laplacian>")
    ap.add_argument('--post', default='off',
                    help='POST point op: off/invert/gray/bgr/thresh/r/g/b')
    ap.add_argument('--pre-thresh', type=int, default=128)
    ap.add_argument('--post-thresh', type=int, default=64)
    # live dither demo: cycle off -> halftone -> poster2 -> poster4 -> random through the run so
    # the effect is visible ON THE MONITOR. (Dither shines on smooth tones/gradients -- point the
    # camera at a real scene, or it cycles on whatever is live.)
    ap.add_argument('--dither-cycle', type=int, default=0,
                    help='1 = cycle dither modes live (colour/halftone/poster2/poster4/random)')
    ap.add_argument('--dither-step-s', type=float, default=6.0, help='seconds per dither mode')
    args, _ = ap.parse_known_args()

    install_vdma_cleanup_signals()
    ol, h = setup_session(download=bool(args.download))
    print(f'chip ID = {h["sccb_read"](0x300A):02X}{h["sccb_read"](0x300B):02X} (expect 5640)')

    # fresh chip_init, supervisor + SOF synthesis, RGB565 arm
    steps = fhs.patch_init_steps(list(fhs.FULL_INIT_STEPS), [(0x4800, args.val4800)])
    v65.chip_init(h, steps, 'hdmi-demo-init', settle_s=10.0)
    h['bitslip_set'](0, 6)
    h['idelay_set'](16, 16)   # eye-centre (crc=0 all taps); 8 was marginal at short clk-settle
    h['frame_lines_set_keep_cam'](value=480, use_lsle=(args.val4800 & 0x10 != 0),
                                  expected_dt=0x22, sup_enable=bool(args.sup),
                                  sof_synth=bool(args.synth),
                                  force_expected=bool(args.force_expected),
                                  hs_settle_gate=bool(args.hs_settle_gate))
    if args.clk_settle > 0:                 # gated: set BEFORE the lock (live change half-hangs)
        h['set_clk_settle'](args.clk_settle)
        print(f'clk-settle={args.clk_settle} (init): byte_clk starts earlier post-vblank '
              f'-> recover leading lines -> stable sof_synth open')
    if args.vcm >= 0:
        vcm_set(h, args.vcm)
        print(f'VCM focus code set to {args.vcm & 0x3FF}')
    time.sleep(0.3)
    gc = ([(0x3A18, (args.gain_ceiling >> 8) & 0x03), (0x3A19, args.gain_ceiling & 0xFF)]
          if args.gain_ceiling >= 0 else [])
    arm = (([(0x503D, args.val503d)] if args.val503d >= 0 else [])
           + gc + list(fhs.ARM_REGS))
    fhs.stream_cycle_write(h, arm)
    if args.val503d >= 0:
        print(f'test pattern 0x503D=0x{args.val503d:02X} enabled '
              f'(MM2S/HDMI display-path vertical-noise diagnosis)')
    if args.gain_ceiling >= 0:
        print(f'AGC gain ceiling capped to 0x{args.gain_ceiling:04X} '
              f'(~{args.gain_ceiling / 16:.1f}x; cuts low-light column FPN)')
    time.sleep(2.0)

    # Deterministic lock before HDMI: the per-/4-phase correct bitslip varies, so
    # a fixed bitslip is a lottery (HDMI was historically black with no lock).
    if args.hw_lock:
        print('--- HW deterministic-lock FSM (no software lock_mode) ---')
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
                print('  *** HW lock FSM FAILED; HDMI may be empty. Power-cycle if '
                      'long stays 0. ***')
                break
            time.sleep(0.4)
    elif args.lock_rerolls > 0:
        h['set_hw_lock'](False)              # inhibit the FSM (HWLOCK_DEFAULT_ON builds) so lock_mode's bitslip applies
        print('--- deterministic lock: bitslip sweep + re-roll-on-fail ---')
        if lock_mode(h, args.lock_rerolls) != 0:
            print('*** WARNING no clean lock via bitslip sweep; HDMI may be empty/'
                  'unstable. Power-cycle the board if long stays 0. ***')
    h['set_long_as_line'](bool(args.long_as_line))   # after lock (idelay writes clear bit26)
    if args.long_as_line:
        print('long-as-line ENABLED: no-LS longs delivered as rows (band fix)')
    h['set_settle_blank'](int(args.settle_blank))    # after lock (no-apply level write)
    if args.settle_blank:
        print(f'settle-blank K={args.settle_blank}: burst-head settle garbage skipped '
              f'(last_fe->480 band fix)')

    # apply the processing chain (PRE -> MID -> POST) so the HDMI shows the processed stream
    _PRE  = {'off':0,'pass':0,'invert':1,'gray':2,'bgr':3,'thresh':4,'r':5,'g':6,'b':7,
             'gaussian':8,'gauss':8,'median':9,'med':9}
    _POST = {'off':0,'pass':0,'invert':1,'gray':2,'bgr':3,'thresh':4,'r':5,'g':6,'b':7}
    pre  = args.pre  if isinstance(args.pre, int)  else _PRE[str(args.pre).lower()]
    post = args.post if isinstance(args.post, int) else _POST[str(args.post).lower()]
    mid  = str(args.mid).lower()
    if pre or post or mid not in ('none', 'off', ''):
        h['set_pre_thresh'](int(args.pre_thresh)); h['set_post_thresh'](int(args.post_thresh))
        h['set_post_op'](post)
        if mid in ('none', 'off', ''):
            if pre >= 8:                               # denoise pre needs conv mode (identity)
                h['set_pre_op'](pre); h['set_conv_named']('identity'); h['set_proc_op'](8)
            else:
                h['set_pre_op'](0); h['set_proc_op'](pre)
        elif mid == 'edges':                           # omnidirectional Sobel |Gx|+|Gy|
            h['set_pre_op'](pre); h['set_edges'](2)
        else:                                          # named 3x3 kernel
            h['set_pre_op'](pre); h['set_conv_named'](mid); h['set_proc_op'](8)
        print(f'processing chain: pre={pre} -> mid={mid} -> post={post} '
              f'(pre_thr={args.pre_thresh} post_thr={args.post_thresh})')

    m = fhs.measure_link(h, dur=5.0, label='hdmi-demo')
    if m['long_pkt'] < 100:
        print('*** WARNING long_pkt low — chip may be degraded; HDMI may be empty. ***')

    # VDMA: S2MM (camera write) + MM2S (HDMI read, 1 frame behind via FRMDLY_SHIFT)
    dvdma = ol.ip_dict['axi_vdma_0']
    vdma = MMIO(int(dvdma['phys_addr']), int(dvdma['addr_range']))
    bufs = [allocate(shape=(HEIGHT, STRIDE), dtype=np.uint8) for _ in range(3)]
    for b in bufs:
        np.asarray(b).fill(0xAA)
    configure_vdma_s2mm(vdma, bufs, start_mm2s=True, start_s2mm=True)

    print(f'\n=== HDMI LIVE OUTPUT running {args.total:.0f}s ===')
    print('The camera image should now be visible on the HDMI monitor.')
    try:
        if args.vcm_sweep:
            codes = list(range(0, 1024, 64)) + [1023]
            print(f'VCM focus sweep: stepping {len(codes)} codes, '
                  f'{args.vcm_step_s:.0f}s each. WATCH THE MONITOR and note the '
                  f'code printed when the image is SHARPEST.')
            for c in codes:
                vcm_set(h, c)
                print(f'  >>> VCM focus code = {c:4d}  (watch monitor) <<<',
                      flush=True)
                time.sleep(args.vcm_step_s)
            print('VCM sweep done; holding last code for the remainder.')
        if args.dither_cycle:
            def _d(label, fn):
                h['set_pre_op'](0)           # plain colour path; dither/post-op is the effect
                fn(); print(f'  >>> dither: {label}  (watch monitor) <<<', flush=True)
            stages = [
                ('OFF (colour)',     lambda: (h['set_post_op'](0), h['set_dither'](enable=False), h['set_proc_op'](0))),
                ('halftone (gray 1-bit)', lambda: (h['set_proc_op'](0), h['set_post_op'](2), h['set_dither'](enable=True, mode='ordered', bits=1))),
                ('poster 2-bit',     lambda: (h['set_proc_op'](0), h['set_post_op'](0), h['set_dither'](enable=True, mode='ordered', bits=2))),
                ('poster 4-bit',     lambda: (h['set_proc_op'](0), h['set_post_op'](0), h['set_dither'](enable=True, mode='ordered', bits=4))),
                ('random 2-bit',     lambda: (h['set_proc_op'](0), h['set_post_op'](0), h['set_dither'](enable=True, mode='random', bits=2))),
            ]
            print(f'DITHER CYCLE: {len(stages)} modes, {args.dither_step_s:.0f}s each. WATCH THE MONITOR.')
            t0 = time.time()
            while time.time() - t0 < args.total:
                for label, fn in stages:
                    if time.time() - t0 >= args.total: break
                    _d(label, fn); time.sleep(args.dither_step_s)
            h['set_dither'](enable=False); h['set_post_op'](0)
        else:
            t0 = time.time()
            while time.time() - t0 < args.total:
                time.sleep(15.0)
                fhs.measure_link(h, dur=2.0, label=f'live+{time.time()-t0:.0f}s')
    except KeyboardInterrupt:
        print('interrupted')
    finally:
        stop_vdma(vdma)
        for b in bufs:
            if hasattr(b, 'freebuffer'):
                b.freebuffer()
        print('VDMA stopped (S2MM+MM2S). Done.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
