#!/usr/bin/env python3
"""Byte-alignment lock on the (recovered) live chip (2026-06-15).

After the power cycle the OV5640 streams real pixel data again, but long_pkt=0
because the byte alignment is wrong: SoT (0xB8) is found by rotation, yet the
packet header after it is misaligned (last_pkt_di reads garbage / VC1), so no
long packet frames. The fix is the classic bring-up step the cal had been
skipping: sweep BITSLIP (the 8-bit barrel shift) -- and if needed re-roll the
BUFR /4 byte phase -- keyed on the LONG-PACKET count.

All of bitslip / idelay / bufr_clr are FPGA-side AXI GPIO (chip-safe); only the
single init + RGB565 arm touch the chip (no register storm).

  per /4 phase (boot, then BUFR.CLR re-rolls):
    1. v65.find_best_bitslip 8x8 sweep -> best (p0,p1) by long then short.
    2. measure long at that bitslip; if long>0 -> idelay_sweep to centre, DONE.
    3. else re-roll the /4 phase and retry.

Run: deploy_banding_test.py --script bitslip_lock.py --download 1 --full-init 1 \
        --extra-args "--val4800 0x14 --rerolls 4"
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

IDELAY_CENTRE_TAPS = [0, 4, 8, 12, 16, 20, 24, 28]


LONG_LOCK = 3000.0     # long/window above which a phase is a solid lock
IDELAY_LOCK = 16       # eye-centre tap once locked (sweep showed 0..20 clean)


def measure(h, dur: float = 0.6) -> dict:
    b = h['snap']()
    time.sleep(dur)
    a = h['snap']()
    return {k: (a[k] - b[k]) % 65536 for k in b}


def stable_lock(h, dur: float = 0.8) -> dict:
    """Confirm a lock is sustained (two windows both clean long), not a transient
    sweep hit (re-roll5 locked mid-sweep but lost it on confirm)."""
    d1 = measure(h, dur)
    d2 = measure(h, dur)
    ok = (d1['long_pkt'] > LONG_LOCK and d2['long_pkt'] > LONG_LOCK
          and d1['crc_err'] == 0 and d2['crc_err'] == 0)
    return dict(ok=ok, long=min(d1['long_pkt'], d2['long_pkt']),
                crc_err=d1['crc_err'] + d2['crc_err'])


def lock_mode(h, rerolls: int, settle_blank: int = 8) -> int:
    """Convergent deterministic lock: sweep the 8x8 bitslip; if a clean sustained
    long-lock appears, centre the idelay eye and HOLD; else re-roll the /4 phase
    and retry. ~6/8 phases lock immediately so this converges in 1-2 re-rolls.

    settle_blank (default 8): AFTER the lock, apply the byte-domain per-line
    settle blank (cfg_settle_blank_k) so the SoT search skips the burst-head
    HS-settle garbage -> last_fe=480/480, no bottom band (2026-06-17). This is the
    universal default so every caller gets the band fix without a per-script flag;
    the lock itself still runs with blank=0 (the validated flow). Callers may
    override afterward (e.g. the K-sweep diagnostic). Harmless on a bitstream
    without the knob (writes an unused GPIO field)."""
    for ph in range(rerolls + 1):
        tag = 'boot' if ph == 0 else f're-roll{ph}'
        if ph > 0:
            h['bufr_clr_pulse']()
            time.sleep(0.3)
        p0, p1 = v65.find_best_bitslip(h, None, None)
        s = stable_lock(h)
        print(f'  {tag}: bitslip=({p0},{p1}) long~{s["long"]:.0f} '
              f'crc_err={s["crc_err"]} stable={s["ok"]}')
        if s['ok']:
            h['idelay_set'](IDELAY_LOCK, IDELAY_LOCK)
            time.sleep(0.3)
            if settle_blank and 'set_settle_blank' in h:
                h['set_settle_blank'](settle_blank)   # band fix, default ON post-lock
            d = measure(h, 1.0)
            print('\n' + '=' * 64)
            print(f'LOCKED after {ph} re-roll(s): bitslip=({p0},{p1}), '
                  f'idelay={IDELAY_LOCK}, settle_blank={settle_blank}, '
                  f'long={d["long_pkt"]} crc_err={d["crc_err"]} '
                  f'fs={d["fs"]} fe={d["fe"]} ls={d["ls"]} le={d["le"]}. Link HELD.')
            print('=' * 64)
            return 0
    print('\n' + '=' * 64)
    print(f'NO clean lock in boot + {rerolls} re-rolls. Increase --rerolls.')
    print('=' * 64)
    return 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--download', type=int, default=1)
    ap.add_argument('--full-init', type=int, default=1)
    ap.add_argument('--val4800', type=lambda x: int(x, 0), default=0x14)
    ap.add_argument('--rerolls', type=int, default=8)
    ap.add_argument('--mode', default='lock', choices=('lock', 'determinism'),
                    help='lock = converge (bitslip sweep + re-roll-on-fail) then '
                         'hold; determinism = sweep every /4 phase and report the '
                         'lock rate (evidence).')
    args, _ = ap.parse_known_args()

    ol, h = setup_session(download=bool(args.download),
                          settle_s=(10.0 if args.download else 0.0),
                          raise_resetb=True)
    cid = (h['sccb_read'](0x300A) << 8) | h['sccb_read'](0x300B)
    print(f'chip ID = {cid:04X} (expect 5640)')

    steps = fhs.patch_init_steps(list(fhs.FULL_INIT_STEPS), [(0x4800, args.val4800)])
    v65.chip_init(h, steps, f'bitslip-lock-4800={args.val4800:02X}', settle_s=10.0)
    h['idelay_set'](8, 8)
    h['frame_lines_set_keep_cam'](value=480, use_lsle=(args.val4800 & 0x10 != 0),
                                  expected_dt=0x22, sup_enable=False)
    time.sleep(0.3)
    fhs.stream_cycle_write(h, fhs.ARM_REGS)   # LIVE RGB565 (no test pattern)
    time.sleep(3.0)

    if args.mode == 'lock':
        print(f'\n=== CONVERGENT LOCK (bitslip sweep + re-roll, budget '
              f'{args.rerolls}) ===')
        return lock_mode(h, args.rerolls)

    print(f'\n=== BITSLIP-SWEEP DETERMINISM TEST: {args.rerolls + 1} /4 phases ===')
    print('Claim: an 8x8 bitslip sweep locks EVERY /4 phase (bitslip 0-7 covers '
          'all intra-byte rotations), so the lock is deterministic without a '
          'clock IDELAY or a re-roll lottery. Re-rolling the /4 phase each round '
          'and confirming the sweep re-locks.')
    results = []   # (phase_tag, p0, p1, long, crc_err)
    for ph in range(args.rerolls + 1):
        tag = 'boot' if ph == 0 else f're-roll{ph}'
        if ph > 0:
            h['bufr_clr_pulse']()
            time.sleep(0.3)
        p0, p1 = v65.find_best_bitslip(h, None, None)
        d = measure(h, 0.8)
        results.append((tag, p0, p1, d['long_pkt'], d['crc_err']))
        print(f'  {tag}: best bitslip=({p0},{p1}) long={d["long_pkt"]} '
              f'short={d["short_pkt"]} fs={d["fs"]} fe={d["fe"]} '
              f'crc_err={d["crc_err"]} drop_dt={d["drop_dt"]}')

    locked = [r for r in results if r[3] > 0 and r[4] == 0]
    print('\n' + '=' * 64)
    print(f'DETERMINISM: {len(locked)}/{len(results)} /4 phases LOCKED clean '
          '(long>0, crc_err=0) via the 8x8 bitslip sweep.')
    print('  phase    bitslip   long  crc_err')
    for tag, p0, p1, lp, ce in results:
        mark = '  <== LOCK' if (lp > 0 and ce == 0) else ''
        print(f'  {tag:8s} ({p0},{p1})   {lp:5d}  {ce:6d}{mark}')
    print('-' * 64)
    if len(locked) == len(results):
        print('  RESULT = bitslip sweep locks EVERY /4 phase -> the lock IS '
              'deterministic build-free. The "lottery" was a fixed wrong bitslip, '
              'not an unreachable phase. Permanent fix = boot-time 8x8 bitslip '
              'sweep (software now, RTL FSM later); no clock IDELAY needed.')
    else:
        print(f'  RESULT = {len(results) - len(locked)} phase(s) did NOT lock even '
              'with a full bitslip sweep -> those /4 phases are genuinely '
              'unalignable (need the re-roll to skip them, or idelay help).')
    print('=' * 64)
    return 0 if len(locked) == len(results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
