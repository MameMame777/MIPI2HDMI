#!/usr/bin/env python3
"""Runtime VTS sweep — Phase 1 of the FPS plan (plan: fps-starry-frog).

VTS (vertical total, 0x380E/0x380F) is currently 1000 with only 480 active
lines, so >half the frame is idle vblank. fps = PCLK / (VTS x HTS), so cutting
VTS raises fps inversely WITHOUT touching the PLL / lane rate (no D-PHY retrain,
no rebuild). This script establishes the verified band-free auto-lock baseline
(identical to hwlock_verify.py: continuous 0x14 + RGB565 arm + HW-lock FSM +
settle-blank K=8), confirms it is live, then steps VTS down, capping the AEC
max-exposure pairs (0x3A02/3 @60Hz, 0x3A14/5 @50Hz) to VTS-8 each step so
auto-AEC cannot target a longer integration than the frame. Auto-AEC is kept
(night mode 0x3A00 bit2 is already off, so the chip will NOT auto-extend).

LIVENESS / fps are read from `fs` (frame-start/s), `fe` (frame-end/s ~ fps),
`pix_per_line` (640 = real data, 0 = dead) and `crc_err_pct` — NOT `long_pkt`,
whose hardware counter freezes after ~6 s (hwlock_verify.py rule). A "winner"
VTS = the smallest value still holding fs/fe high, pix/line ~640, crc ~0.

Run (Windows; NO --reboot: a power-cycle re-rolls the /4 lock phase, the live
board already holds a good HW lock):
  python scripts/deploy_banding_test.py --host 192.168.2.99 \
      --script vts_sweep.py --download 1 --full-init 0 \
      --extra-args "--vts-list 1000,800,660,580,520,500,480"
"""
from __future__ import annotations
import argparse
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from pynq_bringup import setup_session
import v65_capture as v65
import frame_height_stability as fhs


def write_vts(h, vts: int, expo_margin: int = 8) -> int:
    """Set VTS (0x380E/F) and cap both AEC max-exposure pairs to vts-margin.
    Timing registers latch on the next frame boundary, so a direct write while
    streaming applies cleanly without a stream cycle (which would drop the HW
    lock)."""
    expo = max(16, vts - expo_margin)
    h['sccb_write'](0x380E, (vts >> 8) & 0xFF)
    h['sccb_write'](0x380F, vts & 0xFF)
    for hi, lo in ((0x3A02, 0x3A03), (0x3A14, 0x3A15)):   # 60Hz + 50Hz max-expo
        h['sccb_write'](hi, (expo >> 8) & 0xFF)
        h['sccb_write'](lo, expo & 0xFF)
    return expo


def is_live(m: dict) -> bool:
    """Liveness by fs/fe/pix_per_line/crc — NOT long_pkt (frozen counter)."""
    return (m['fs'] > 5.0 and m['fe'] > 5.0
            and m['pix_per_line'] >= 600 and m['crc_err_pct'] < 1.0)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--download', type=int, default=1)
    ap.add_argument('--val4800', type=lambda x: int(x, 0), default=0x14,
                    help='MIPI ctrl 0x4800 (0x14 = continuous, HW-lock target)')
    ap.add_argument('--settle-blank', type=int, default=8)
    ap.add_argument('--vts-list', type=str,
                    default='1000,800,660,580,520,500,480',
                    help='comma-separated VTS values to sweep (high->low)')
    ap.add_argument('--dur', type=float, default=6.0, help='measure seconds/step')
    args, _ = ap.parse_known_args()

    # --- verified auto-lock baseline (mirror of hwlock_verify.main) ---
    ol, h = setup_session(download=bool(args.download),
                          settle_s=(10.0 if args.download else 0.0),
                          raise_resetb=True)
    cid = (h['sccb_read'](0x300A) << 8) | h['sccb_read'](0x300B)
    print(f'chip ID = {cid:04X} (expect 5640)')

    steps = fhs.patch_init_steps(list(fhs.FULL_INIT_STEPS), [(0x4800, args.val4800)])
    v65.chip_init(h, steps, 'vts-sweep-init', settle_s=10.0)
    h['idelay_set'](16, 16)                  # eye-centre
    h['frame_lines_set_keep_cam'](value=480, use_lsle=(args.val4800 & 0x10 != 0),
                                  expected_dt=0x22, sup_enable=False,
                                  sof_synth=True, force_expected=True)
    time.sleep(0.3)
    fhs.stream_cycle_write(h, list(fhs.ARM_REGS))
    h['set_settle_blank'](int(args.settle_blank))
    time.sleep(1.5)

    # --- HW deterministic-lock FSM (no software lock_mode) ---
    print('--- HW deterministic-lock FSM ---')
    h['bitslip_set'](0, 0)                   # neutral target (ignored once FSM drives)
    h['set_hw_lock'](True)
    t0 = time.time()
    while time.time() - t0 < 15.0:
        s = h['read_hwlock']()
        if s['locked']:
            print(f'  HW-locked bitslip=({s["p0"]},{s["p1"]}) state={s["state_name"]} '
                  f'hdr_active={s["hdr_active"]} reroll={s["reroll"]} t={time.time()-t0:.1f}s')
            break
        if s['failed']:
            print('  *** HW lock FSM FAILED ***')
            break
        time.sleep(0.4)

    # --- baseline health gate (VTS=1000) BEFORE sweeping ---
    write_vts(h, 1000)
    time.sleep(1.0)
    base = fhs.measure_link(h, dur=args.dur, label='baseline-vts1000')
    if not is_live(base):
        print(f'*** baseline NOT live (fs={base["fs"]:.1f} fe={base["fe"]:.1f} '
              f'pix/line={base["pix_per_line"]} crc={base["crc_err_pct"]:.1f}%). '
              f'Bad HW lock — re-run (no reboot) or power-cycle. Aborting sweep. ***')
        return 1
    print(f'baseline live: fs={base["fs"]:.1f}/s fe={base["fe"]:.1f}/s '
          f'pix/line={base["pix_per_line"]} crc={base["crc_err_pct"]:.1f}%')

    # --- VTS sweep ---
    vts_list = [int(x) for x in args.vts_list.split(',') if x.strip()]
    print(f'\n=== VTS sweep {vts_list}  (HTS=1600, active=480, auto-AEC) ===')
    rows = []
    for vts in vts_list:
        expo = write_vts(h, vts)
        time.sleep(1.0)                      # let a few frames apply
        m = fhs.measure_link(h, dur=args.dur, label=f'vts{vts}')
        s = h['read_hwlock']()
        live = is_live(m)
        rows.append(dict(vts=vts, expo=expo, fps=m['fe'], fs=m['fs'],
                         crc=m['crc_err_pct'], pix=m['pix_per_line'],
                         lfl=m['last_frame_lines'], locked=s['locked'], live=live))

    write_vts(h, 1000)                        # restore safe baseline before exit

    print('\n================= VTS SWEEP SUMMARY =================')
    print(f'{"VTS":>5} {"expo":>5} {"fps(fe)":>8} {"fs/s":>6} {"crc%":>6} '
          f'{"pix/ln":>6} {"lfl":>5} {"lock":>5} {"verdict":>8}')
    base_fps = base['fe']
    winner = None
    for r in rows:
        verdict = 'OK' if r['live'] else 'BAD'
        if r['live']:
            winner = r            # smallest live (list is high->low)
        print(f'{r["vts"]:>5} {r["expo"]:>5} {r["fps"]:>8.2f} {r["fs"]:>6.1f} '
              f'{r["crc"]:>6.1f} {r["pix"]:>6} {r["lfl"]:>5} '
              f'{str(r["locked"]):>5} {verdict:>8}')
    if winner:
        gain = (winner['fps'] / base_fps) if base_fps > 0 else 0.0
        print(f'\nWINNER: VTS={winner["vts"]} -> fps={winner["fps"]:.2f} '
              f'(x{gain:.2f} vs VTS=1000 @ {base_fps:.2f} fps), '
              f'AEC max-expo={winner["expo"]}. '
              f'Bake init_rom[65/66]=VTS, [84/85]+0x3A14/5=expo.')
    else:
        print('\nNo VTS below 1000 held a clean link — keep VTS=1000 / re-examine.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
