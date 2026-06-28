#!/usr/bin/env python3
"""Zero-PYNQ boot-init test (2026-06-19).

Tests the C_DOUT_DEFAULT=0x02000000 fix: load the bitstream and do NOTHING else
-- NO chip SCCB init, NO set_hw_lock, NO frame_lines/arm config. If the
bitstream-init FSM configured the chip at boot (RESETB now high from PL config),
long packets flow on their own = the boot-init NACK is resolved (chip self-inits).

What this build CAN show: chip self-configured + streaming at boot (long_pkt>0).
What it likely CANNOT yet show: a clean IMAGE, because the bitstream bakes
0x4800=0x24 (gated; the continuous HW-lock FSM wants 0x14) and the frame_lines GPIO
default expected_dt=0/no settle-blank/etc. A clean-image zero-PYNQ needs a follow-up
that bakes the full verified config (0x4800=0x14 + expected_dt=0x22 + settle-blank=8
+ frame_lines flags) into the GPIO C_DOUT_DEFAULTs / RTL params.

Run: deploy_banding_test.py --script zero_pynq_test.py --download 1 --full-init 0 \
        --upload-bit vloop_probes2/vloop.runs/impl_1/bd_wrapper.bit --extra-args ""
(NOTE --full-init 0: the whole point is to NOT run the runtime SCCB init.)
"""
from __future__ import annotations
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from pynq_bringup import setup_session
import frame_height_stability as fhs


def main() -> int:
    # Load the bitstream (triggers the bitstream-init FSM with RESETB high from
    # config) + 12 s settle. NO chip_init, NO set_hw_lock, NO arm.
    ol, h = setup_session(download=True, settle_s=12.0, raise_resetb=True)

    # IMPORTANT: measure the link FIRST, with NO chip-register SCCB reads, because
    # SCCB reads during an active lock glitch the link (project rule) -> they tank
    # long/fs and corrupt the register readback. Read the FSM state (passive read_dbg)
    # + measure, THEN do the diagnostic chip reads afterwards.
    time.sleep(2.0)
    s = h['read_hwlock']()
    m = fhs.measure_link(h, dur=6.0, label='zero-pynq')

    # Diagnostic chip reads AFTER the measurement (glitch is then harmless).
    cid = (h['sccb_read'](0x300A) << 8) | h['sccb_read'](0x300B)
    try:
        r4800 = h['sccb_read'](0x4800)
        r4300 = h['sccb_read'](0x4300)
        r501f = h['sccb_read'](0x501F)
        r300e = h['sccb_read'](0x300E)
        print(f'chip ID={cid:04X}  0x4800={r4800:02X} 0x4300={r4300:02X} 0x501F={r501f:02X} '
              f'0x300E={r300e:02X} (post-measure reads may glitch under active lock; the '
              f'baked core0 CONFIG is 0x14/0x6F/0x01)')
    except Exception as e:
        print(f'(SCCB read failed: {e})')

    print('\n=== VERDICT (zero-PYNQ RX) ===')
    print(f'  chip ID readable: {"YES" if cid == 0x5640 else "NO"} ({cid:04X})')
    print(f'  link@boot (no PYNQ init): long={m["long_pkt"]:.0f}/s fs={m["fs"]:.2f}/s '
          f'crc_err={m["crc_err_pct"]:.1f}% last_frame_lines={m["last_frame_lines"]}')
    print(f'  HW-lock FSM (baked on): state={s["state_name"]} locked={s["locked"]} '
          f'bitslip=({s["p0"]},{s["p1"]}) reroll={s["reroll"]}')
    rx_ok = (cid == 0x5640 and s['locked'] and not s['failed']
             and m['crc_err_pct'] < 1.0 and m['last_frame_lines'] >= 470)
    if rx_ok:
        print('  => ZERO-PYNQ RX OK: the chip self-configured (continuous + RGB565) AND '
              'the HW-lock FSM auto-locked at boot with NO PYNQ -- clean (crc=0%) '
              f'{m["last_frame_lines"]}-line frames assembling. (HDMI display still needs '
              'PYNQ to start the VDMA -- separate task.)')
    elif cid == 0x5640 and (m['long_pkt'] > 100 or m['fs'] > 1.0):
        print('  => chip self-configured + streaming, but not a clean locked RX -- '
              'inspect crc / last_frame_lines / FSM state above.')
    elif cid == 0x5640:
        print('  => chip alive but NOT streaming (long~0): bitstream-init may not be '
              'completing -- inspect 0x300E + the init FSM.')
    else:
        print('  => chip ID not 0x5640: chip not responding (RESETB/power).')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
