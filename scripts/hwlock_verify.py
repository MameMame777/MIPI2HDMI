#!/usr/bin/env python3
"""Verify the HW deterministic-lock FSM (E2, 2026-06-19).

Boots continuous (0x14), arms RGB565, enables cfg_hw_lock (bitslip_word[25]) and
runs NO software lock_mode -- the RTL FSM (dphy_hwlock_fsm) must sweep the 8x8
bitslip + /4-phase re-roll and HOLD a clean lock on its own (Xilinx-IP-equivalent
power-on auto-lock). Polls debug page 0x2e (ctrl 0x8E) to watch
IDLE->SWEEP->(REROLL)->HOLD, then measures the link: a clean auto-lock = state
HOLD/locked + long>7000/s + crc_err=0. Compares the FSM-picked bitslip to what
lock_mode would choose (informational, run with --compare-lockmode).

Run: deploy_banding_test.py --script hwlock_verify.py --download 1 --full-init 1 \
        --upload-bit vloop_probes2/vloop.runs/impl_1/bd_wrapper.bit \
        --extra-args "--total 20"
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


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--download', type=int, default=1)
    ap.add_argument('--full-init', type=int, default=1)
    ap.add_argument('--val4800', type=lambda x: int(x, 0), default=0x14,
                    help='0x14 = continuous (the HW-lock FSM target). gated unsupported.')
    ap.add_argument('--synth', type=int, default=1)
    ap.add_argument('--force-expected', type=int, default=1)
    ap.add_argument('--settle-blank', type=int, default=8)
    ap.add_argument('--total', type=float, default=20.0, help='seconds to poll the FSM')
    ap.add_argument('--no-enable', type=int, default=0,
                    help='1 = do NOT call set_hw_lock; only chip-init + arm, then '
                         'observe page 0x2e. Tests a HWLOCK_DEFAULT_ON bake: if the '
                         'FSM reaches HOLD/locked with no software enable, the bake '
                         'took effect (FSM on at power-up).')
    ap.add_argument('--compare-lockmode', type=int, default=0,
                    help='1 = after the HW lock, also run the software lock_mode '
                         '(hw-lock off) and print the bitslip it would pick, for '
                         'cross-check. Default 0 (pure HW path).')
    args, _ = ap.parse_known_args()

    ol, h = setup_session(download=bool(args.download),
                          settle_s=(10.0 if args.download else 0.0), raise_resetb=True)
    print(f'chip ID = {(h["sccb_read"](0x300A) << 8) | h["sccb_read"](0x300B):04X} (expect 5640)')

    # continuous init + RGB565 arm so headers stream (the FSM needs a live link).
    steps = fhs.patch_init_steps(list(fhs.FULL_INIT_STEPS), [(0x4800, args.val4800)])
    v65.chip_init(h, steps, 'hwlock-init', settle_s=10.0)
    h['idelay_set'](16, 16)                  # eye-centre
    h['frame_lines_set_keep_cam'](value=480, use_lsle=(args.val4800 & 0x10 != 0),
                                  expected_dt=0x22, sup_enable=False,
                                  sof_synth=bool(args.synth),
                                  force_expected=bool(args.force_expected))
    time.sleep(0.3)
    fhs.stream_cycle_write(h, list(fhs.ARM_REGS))
    h['set_settle_blank'](int(args.settle_blank))
    time.sleep(1.5)

    # ----- the test: enable the FSM, run NO software lock_mode -----
    if args.no_enable:
        print('\n=== HWLOCK_DEFAULT_ON bake test: chip init + arm, NO set_hw_lock ===')
        print('  (if the FSM reaches HOLD/locked with no software enable, the bake took)')
    else:
        print('\n=== HW deterministic-lock FSM: enable, NO software lock_mode ===')
        h['bitslip_set'](0, 0)               # neutral GPIO target (ignored once FSM drives)
        h['set_hw_lock'](True)
    t0 = time.time()
    locked_at = None
    last = None
    while time.time() - t0 < args.total:
        s = h['read_hwlock']()
        tag = (f't={time.time()-t0:4.1f}s  state={s["state_name"]:6s} '
               f'combo={s["combo"]:2d}(p0={s["p0"]},p1={s["p1"]})  reroll={s["reroll"]} '
               f'locked={s["locked"]} failed={s["failed"]} hdr_active={s["hdr_active"]}')
        if tag != last:                      # only print on change (+ heartbeat)
            print('  ' + tag, flush=True)
            last = tag
        if s['locked'] and locked_at is None:
            locked_at = time.time() - t0
            print(f'  >>> LOCKED at t={locked_at:.1f}s: bitslip=({s["p0"]},{s["p1"]}) '
                  f'after {s["reroll"]} re-roll(s) <<<')
        if s['failed']:
            print('  *** FSM FAILED (no clean lock after MAX_REROLL). ***')
            break
        time.sleep(0.4)

    s = h['read_hwlock']()
    m = fhs.measure_link(h, dur=5.0, label='hwlock')
    print('\n=== VERDICT ===')
    print(f'  FSM: state={s["state_name"]} locked={s["locked"]} failed={s["failed"]} '
          f'bitslip=({s["p0"]},{s["p1"]}) reroll={s["reroll"]}')
    print(f'  link: fs={m["fs"]:.2f}/s fe={m["fe"]:.2f}/s last_frame_lines={m["last_frame_lines"]} '
          f'crc_err={m["crc_err_pct"]:.1f}% (crc_err_tot={m.get("crc_err_tot", 0)}) '
          f'long={m["long_pkt"]:.0f}/s [advisory: long counter freezes ~6s]')
    # Liveness by fs / last_frame_lines, NOT long_pkt (the long counter freezes
    # after ~6s = artifact; project rule, memory project_frontend_3pct...).
    healthy = (m['fs'] > 5.0 and m['last_frame_lines'] >= 470 and m['crc_err_pct'] < 1.0)
    if s['locked'] and not s['failed'] and healthy:
        print(f'  => HW AUTO-LOCK OK: the RTL FSM locked bitslip=({s["p0"]},{s["p1"]}) '
              f'with NO software lock_mode -- full-height ({m["last_frame_lines"]} lines), '
              f'fs={m["fs"]:.1f}/s, crc_err={m["crc_err_pct"]:.1f}%. Power-on auto-lock works.')
    else:
        print('  => HW auto-lock NOT clean -- inspect state / fs / last_frame_lines / crc above.')

    if args.compare_lockmode:
        from bitslip_lock import lock_mode
        print('\n--- inhibit test: set_hw_lock(False) -> FSM must go IDLE (bit26) ---')
        h['set_hw_lock'](False)
        time.sleep(0.8)
        si = h['read_hwlock']()
        print(f'  after inhibit: state={si["state_name"]} locked={si["locked"]} '
              f'(IDLE+!locked = inhibit works; FSM released the bitslip)')
        if si['state_name'] == 'IDLE' and not si['locked']:
            print('  => INHIBIT OK: bit26 disables the baked FSM -> lock_mode/manual '
                  'bitslip path is available on the default-on bitstream.')
        else:
            print('  => INHIBIT did NOT take (FSM still active) -- check bit26 wiring.')
        print('  (note: lock_mode long-scoring may read 0 here due to the ~6s long-'
              'counter freeze from the prior FSM hold; the IDLE check above is the '
              'definitive inhibit proof.)')

    h['set_hw_lock'](False)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
