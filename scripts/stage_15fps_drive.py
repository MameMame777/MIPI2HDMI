#!/usr/bin/env python3
"""Stage-by-stage drive of OV5640 to >=15 fps on the new mipi_div=2 bitstream.

Plan reference: (local agent note)

Walks: clean baseline -> IDELAY sweep -> baseline frame_fps -> AEC short ->
PLL mult step-up (cap 0x50) -> VTS reduction -> optional 0x3108/0x4837.
Measures 5 s sustained after each delta, auto-reverts on degrade, stops at
the first stage hitting frame_fps >= 15.0.

Acceptable per user: 10-20 fps with stability priority (CRC <= 10 %).
"""
from __future__ import annotations
import argparse
import dataclasses
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image
from pynq import Overlay, MMIO, allocate

sys.path.insert(0, str(Path(__file__).resolve().parent))
from full_init_steps import FULL_INIT_STEPS
from v65_capture import (
    make_helpers,
    chip_init,
    idelay_sweep,
    find_best_bitslip,
    configure_vdma_s2mm,
    stop_vdma as v65_stop_vdma,
    install_vdma_cleanup_signals,
    decode_vdmasr,
    CAM_GPIO_BIT,
    R as VDMAR,
    RS,
    CIRCULAR_PARK,
    RESET,
    WIDTH,
    HEIGHT,
    STRIDE,
    BUF_FILL_PATTERN,
)

BIT_DEFAULT = '/home/xilinx/mipi2hdml/bd_wrapper.bit'

# AEC dicts: verified 2026-05-24 続編 5 to give 2.75x frame_fps boost.
LINUX_AEC = {
    0x3a00: 0x70,  # night mode OFF (banding filter / long exposure auto OFF)
    0x3503: 0x07,  # AEC + AGC manual, delay OFF
    0x3500: 0x00,  # exposure[19:16]
    0x3501: 0x00,  # exposure[15:8]
    0x3502: 0x10,  # exposure[7:0]  -> 16 lines total
    0x350A: 0x00,  # gain[9:8]
    0x350B: 0x40,  # gain[7:0]      -> ~4x to compensate the short exposure
}
BASELINE_AEC = {
    0x3a00: 0x78,  # night mode default
    0x3503: 0x00,  # auto AEC + AGC
    0x3500: 0x00,
    0x3501: 0x7B,  # exposure ~123 lines (Linux baseline)
    0x3502: 0x00,
    0x350A: 0x00,
    0x350B: 0x00,
}

# Linux mainline 27-reg analog batch — verbatim from
# scripts/linux_analog_capture.py (the 2026-05-24 demo) and
# scripts/pll_safe_combo_aec_capture.py. This is what enabled the chip
# to emit clean MIPI TX with CRC=0 in the demo. Without it the chip
# defaults to Digilent analog values which produce sporadic long packet
# loss especially after IDELAY/BITSLIP sweep activity.
LINUX_ANALOG = {
    0x3601: 0x33, 0x3620: 0x52, 0x3621: 0xE0, 0x3622: 0x01,
    0x3630: 0x36, 0x3631: 0x0E, 0x3632: 0xE2, 0x3633: 0x12,
    0x3634: 0x40, 0x3635: 0x13, 0x3636: 0x03,
    0x3703: 0x5A, 0x3704: 0xA0, 0x3705: 0x1A,
    0x370B: 0x60, 0x3715: 0x78, 0x3717: 0x01, 0x371B: 0x20, 0x3731: 0x12,
    0x302D: 0x60, 0x3C01: 0xA4, 0x3C04: 0x28, 0x3C05: 0x98,
    0x3901: 0x0A, 0x3905: 0x02, 0x3906: 0x10,
    0x5001: 0xA3,
}

# 0x4837 peak found by 2026-05-25 sweep_4837_diag.py on v3 bitstream
# (long_pkt 844 vs 697 at default 0x18). Applied as a pre-Stage 3 tweak.
PCLK_PERIOD_PEAK = 0x24

# Optional Stage 6 chip-side fine trim (verified safe from pll_individual_narrow.py).
# 0x3108=0x05 is the Linux safe value; 0x4837 stays at PCLK_PERIOD_PEAK.
SAFE_PLL_FINE = {0x3108: 0x05, 0x4837: PCLK_PERIOD_PEAK}
BASELINE_PLL_FINE = {0x3108: 0x01, 0x4837: 0x18}

# Baseline chip state right after bitstream-init FSM completes on v3 bit
# (`1e18d5a9`). Used as the starting point for per-stage snapshots and as
# the floor for absolute-write rewind in apply_state().
BASELINE_STATE = {
    'pll_mult'     : 0x36,
    'pclk_4837'    : 0x18,
    'aec_dict'     : BASELINE_AEC,
    'linux_analog' : False,
    'vts'          : None,            # None -> chip 640x480 default ~1000
    'pll_fine'     : BASELINE_PLL_FINE,
}

# Registers gating FE generation per Linux mainline + diary 20260524 続編 3.
# Sampled around Stage 2 / 2a to root-cause "long_pkt up, fe down".
FE_GATING_REGS = [
    0x4814,                          # MIPI DEBUG MODE - bit[3] gates FE
    0x4202,                          # FRAME CTRL 02 - non-zero halts stream
    0x300E,                          # MIPI SC CTRL - 0x45=stream, 0x40=halt
    0x380E, 0x380F,                  # VTS
    0x380C, 0x380D,                  # HTS
    0x3500, 0x3501, 0x3502,          # AEC exposure (lines)
    0x3503,                          # AEC manual bit
    0x350A, 0x350B,                  # gain
    0x3A00,                          # AEC ctrl 00 (night mode -> long auto)
    0x4300,                          # format ctrl
    0x501F,                          # ISP mux
]

DEMO_LINUX_ANALOG_RAW = '/home/xilinx/mipi2hdml/captures/demo_20260524/linux_analog_buf0.raw'


@dataclasses.dataclass
class StageResult:
    label: str
    frame_fps: float
    long_rate: float
    crc_err_pct: float
    fs: int
    fe: int
    drops: int
    last_di: int
    healthy: bool
    raw: dict


def measure(h: dict, label: str, dur: float = 5.0) -> StageResult:
    """5 s diff snap. Returns frame_fps (from FE count) and other rates."""
    a = h['snap']()
    time.sleep(dur)
    b = h['snap']()
    d = {k: (b[k] - a[k]) % 65536 for k in a}
    long_rate = d['long_pkt'] / dur
    fps = d['fe'] / dur if d['fe'] > 0 else 0.0
    crc_tot = d['crc_ok'] + d['crc_err']
    crc_pct = 100.0 * d['crc_err'] / crc_tot if crc_tot > 0 else 0.0
    drops = d['drop_dt'] + d['drop_vc']
    p01 = h['read_dbg'](0x01)
    last_di = (p01 >> 16) & 0xFF
    healthy = (d['long_pkt'] > 100 and crc_pct < 50.0
               and d['fs'] > 0 and d['fe'] > 0)
    print(f'  {label:30s} long={d["long_pkt"]:5} ({long_rate:6.0f}/s) '
          f'frame_fps={fps:6.2f} crc%={crc_pct:5.1f}  '
          f'fs={d["fs"]:3} fe={d["fe"]:3} drops={drops:3} di=0x{last_di:02X}'
          f'  {"OK" if healthy else "BAD"}')
    return StageResult(label, fps, long_rate, crc_pct,
                       d['fs'], d['fe'], drops, last_di, healthy, d)


def stream_cycle_writes(h: dict, writes) -> None:
    """Chip stream-off, sequence of SCCB writes, then stream-on. Required for
    PLL / format changes (0x300E cycle is what tells the chip to re-evaluate)."""
    h['sccb_write'](0x300E, 0x40)
    h['sccb_write'](0x4202, 0x0F)
    time.sleep(0.1)
    for a, v in writes:
        h['sccb_write'](a, v)
    h['sccb_write'](0x300E, 0x45)
    h['sccb_write'](0x4202, 0x00)
    time.sleep(2.0)


def apply_dict(h: dict, name: str, batch: dict) -> None:
    print(f'  apply {name} ({len(batch)} reg) via stream cycle')
    stream_cycle_writes(h, batch.items())


def apply_state(h: dict, state: dict, linux_analog_in_bitstream: bool = False) -> None:
    """Absolute-write the chip to a recorded snapshot. Idempotent.
    Order: VTS first (so a Stage 5 victim can't keep a small VTS), then
    PLL-affecting registers via stream cycle, then AEC overwrite, then
    LINUX_ANALOG if it had been applied.

    If linux_analog_in_bitstream is True (paired with --linux-analog-in-bitstream),
    the LINUX_ANALOG rewind re-apply is suppressed - bitstream-init FSM already
    holds the values and runtime stream-cycle re-apply is verified destructive
    on the 2026-05-30 ROM-based bitstream."""
    vts = state['vts'] if state['vts'] is not None else 1000
    h['sccb_write'](0x380E, (vts >> 8) & 0xFF)
    h['sccb_write'](0x380F, vts & 0xFF)
    time.sleep(0.2)
    pll_writes = [
        (0x3036, state['pll_mult']),
        (0x4837, state['pclk_4837']),
        (0x3108, state['pll_fine'][0x3108]),
    ]
    stream_cycle_writes(h, pll_writes)
    for a, v in state['aec_dict'].items():
        h['sccb_write'](a, v)
    time.sleep(0.2)
    if state['linux_analog'] and not linux_analog_in_bitstream:
        apply_dict(h, 'LINUX_ANALOG (rewind)', LINUX_ANALOG)


def chip_snapshot(h: dict, label: str) -> dict:
    snap = {a: h['sccb_read'](a) for a in FE_GATING_REGS}
    print(f'  chip snapshot @ {label}:')
    for a in FE_GATING_REGS:
        v = snap[a]
        v_s = f'0x{v:02X}' if v is not None else 'ERR'
        print(f'    0x{a:04X} = {v_s}')
    return snap


def chip_diff(label: str, before: dict, after: dict) -> None:
    diffs = [(a, before[a], after[a])
             for a in FE_GATING_REGS
             if before[a] != after[a]]
    if not diffs:
        print(f'\n  chip diff @ {label}: (no FE_GATING_REGS changed)')
        return
    print(f'\n  chip diff @ {label}: {len(diffs)} reg changed')
    for a, b, c in diffs:
        b_s = f'0x{b:02X}' if b is not None else 'ERR'
        c_s = f'0x{c:02X}' if c is not None else 'ERR'
        print(f'    0x{a:04X}  {b_s} -> {c_s}')


def sweep_idelay_and_bitslip(h: dict, taps, window_s: float = 2.0):
    """Walk a SINGLE BITSLIP pair (0,6) over the supplied tap list. Each
    `bitslip_set` issues a pulse that the ISERDES counts; doing 64 of them
    (4 BITSLIPs × 16 taps) was empirically shown on 2026-05-25 to leave
    the chip MIPI TX unable to emit long packets afterwards. Keeping the
    walk to one BITSLIP pair limits the perturbation."""
    rows = idelay_sweep(h, 0, 6, taps, window_s=window_s)
    flat = [(((0, 6), tap, d)) for tap, d in rows]
    if not flat:
        return None
    def score(item):
        (_p, _t, d) = item
        crc_tot = d['crc_ok'] + d['crc_err']
        crc_pct = (d['crc_err'] / crc_tot) if crc_tot > 0 else 1.0
        return (d['long_pkt'], -crc_pct)
    flat.sort(key=score, reverse=True)
    best_p, best_t, best_d = flat[0]
    print(f'\nBest training: BITSLIP=({best_p[0]},{best_p[1]}) IDELAY={best_t} '
          f'long={best_d["long_pkt"]} crc_ok={best_d["crc_ok"]} '
          f'crc_err={best_d["crc_err"]}')
    h['bitslip_set'](*best_p)
    h['idelay_set'](best_t, best_t)
    print('  stabilise pause 1.0 s before Stage 2 measure')
    time.sleep(1.0)
    return best_p, best_t


def setup_vdma_capture(ol: Any, enable_hdmi: bool = False):
    """Allocate 3 frame buffers and start VDMA S2MM (and optionally MM2S
    for HDMI live readout). Reuses v65_capture.configure_vdma_s2mm so all
    the FRMDLY_SHIFT / VSIZE / STRIDE invariants stay in one place."""
    vdma_desc = ol.ip_dict['axi_vdma_0']
    vdma = MMIO(int(vdma_desc['phys_addr']), int(vdma_desc['addr_range']))
    bufs = [allocate(shape=(HEIGHT, STRIDE), dtype=np.uint8) for _ in range(3)]
    for b in bufs:
        np.asarray(b).fill(BUF_FILL_PATTERN)
    configure_vdma_s2mm(vdma, bufs, start_mm2s=enable_hdmi, start_s2mm=True)
    return vdma, bufs


def stop_vdma(vdma):
    v65_stop_vdma(vdma)


def snapshot_buf_png(buf, path: str) -> dict:
    """Snapshot one VDMA-cycled buffer (np.copy to avoid tearing during read),
    save as PNG, return summary metrics."""
    arr = np.asarray(buf).reshape(HEIGHT, STRIDE).copy()
    Image.fromarray(arr).save(path)
    m = dict(
        mean=float(arr.mean()),
        std=float(arr.std()),
        flat_rows=int((arr.var(axis=1) < 1).sum()),
        row_mean_std=float(arr.mean(axis=1).std()),
        col_mean_std=float(arr.mean(axis=0).std()),
        nonzero_vs_fill=int((arr != BUF_FILL_PATTERN).sum()),
    )
    print(f'    -> {path}')
    print(f'       mean={m["mean"]:.1f}  std={m["std"]:.1f}  '
          f'flat_rows={m["flat_rows"]}/{HEIGHT}  '
          f'row_mean_std={m["row_mean_std"]:.1f}  '
          f'col_mean_std={m["col_mean_std"]:.1f}  '
          f'nonzero_vs_fill={m["nonzero_vs_fill"]}')
    return m


def parse_tuning_sccb(s: str) -> list:
    """Parse '0x3C01=0x00,0x5001=0xA3' -> [(0x3C01,0x00),(0x5001,0xA3)].
    Empty string or None -> []."""
    if not s:
        return []
    out = []
    for kv in s.split(','):
        kv = kv.strip()
        if not kv:
            continue
        a, _, v = kv.partition('=')
        out.append((int(a.strip(), 0), int(v.strip(), 0)))
    return out


def apply_tuning_sccb(h, regs: list, label: str) -> None:
    """Apply SCCB writes via stream cycle (one cycle for the whole batch)."""
    if not regs:
        return
    print(f'\n========== tuning SCCB apply: {label} ==========')
    print('  stream-off (0x300E=0x40, 0x4202=0x0F)')
    h['sccb_write'](0x300E, 0x40); time.sleep(0.05)
    h['sccb_write'](0x4202, 0x0F); time.sleep(0.05)
    for addr, val in regs:
        ok = h['sccb_write'](addr, val)
        print(f'  write 0x{addr:04X}=0x{val:02X}  ok={ok}')
        time.sleep(0.01)
    print('  stream-on (0x300E=0x45, 0x4202=0x00)')
    h['sccb_write'](0x300E, 0x45); time.sleep(0.05)
    h['sccb_write'](0x4202, 0x00); time.sleep(0.5)
    print('  readback:')
    for addr, val in regs:
        rb = h['sccb_read'](addr)
        rb_str = f'0x{rb:02X}' if rb is not None else 'READ_FAIL'
        match = '' if rb == val else '  *** MISMATCH ***'
        print(f'    0x{addr:04X} = {rb_str} (expected 0x{val:02X}){match}')
    print('  wait 5s for chip to stabilize streaming with new register(s)')
    time.sleep(5.0)


def capture_503d_toggle(h, bufs, dump_prefix: str, pattern_val: int = 0x80) -> None:
    """Snapshot sensor mode -> stream-cycle 0x503D=pattern_val -> snapshot test
    pattern mode -> stream-cycle 0x503D=0x00 restore. VDMA must be running
    and chip must already be streaming (called after 0x4800 apply + settle).

    pattern_val (OV5640 0x503D PRE_ISP_TEST_SET1, bit7=enable):
      0x80 = color bar (8 vertical bars; row-CONSTANT -> blind to row drop/order)
      0x81 = bit[1:0]=01 random data (frozen per-pixel; row-VARYING)
      0x82 = bit[1:0]=10 color square (vertical block structure; row-VARYING)
    Row-varying patterns (0x81/0x82) close the color-bar blind spot: within one
    frame they reveal line drop/reorder (torn block edges), and being frozen they
    reveal frame-phase rolling across consecutive buffers (Plan B, diary 20260531)."""
    print('\n========== 0x503D toggle capture ==========')
    ts = time.strftime('%Y%m%d_%H%M%S')

    print('  [phase 1/3] Snapshot sensor mode (0x503D=0x00)')
    time.sleep(2.0)
    m_sensor = snapshot_buf_png(bufs[0],
                                f'{dump_prefix}_503d_off_{ts}.png')

    print(f'  [phase 2/3] Stream cycle: 0x503D=0x{pattern_val:02X} (test pattern)')
    h['sccb_write'](0x300E, 0x40); time.sleep(0.05)
    h['sccb_write'](0x4202, 0x0F); time.sleep(0.05)
    ok = h['sccb_write'](0x503D, pattern_val)
    print(f'      0x503D=0x{pattern_val:02X} write_ok={ok}')
    time.sleep(0.05)
    h['sccb_write'](0x300E, 0x45); time.sleep(0.05)
    h['sccb_write'](0x4202, 0x00); time.sleep(0.5)
    rb = h['sccb_read'](0x503D)
    rb_str = f'0x{rb:02X}' if rb is not None else 'READ_FAIL'
    print(f'      0x503D readback={rb_str}')
    print('      wait 5s for chip ISP re-arm + a few full frames into bufs')
    time.sleep(5.0)
    m_test = snapshot_buf_png(bufs[0],
                              f'{dump_prefix}_503d_on_{ts}.png')
    # Plan B: dump all 3 consecutive frame stores in test-pattern mode so the
    # within-frame line integrity and inter-frame phase (rolling) can be analysed.
    print('      [Plan B] dumping 3 consecutive buffers in test-pattern mode')
    dump_buffers(bufs, f'{dump_prefix}_503d_pat_{ts}')
    # Frozen-pattern vertical-integrity verdict (only valid when rolling bit off).
    if not (pattern_val & 0x40):
        try:
            import frozen_pattern_test as fpt
            frames = [np.asarray(b).reshape(HEIGHT, STRIDE) for b in bufs]
            fpt.analyze_frozen_frames(frames, label=f'0x{pattern_val:02X}')
        except Exception as e:
            print(f'      [frozen analysis skipped: {e}]')

    print('  [phase 3/3] Stream cycle: 0x503D=0x00 (restore)')
    h['sccb_write'](0x300E, 0x40); time.sleep(0.05)
    h['sccb_write'](0x4202, 0x0F); time.sleep(0.05)
    ok = h['sccb_write'](0x503D, 0x00)
    print(f'      0x503D=0x00 write_ok={ok}')
    time.sleep(0.05)
    h['sccb_write'](0x300E, 0x45); time.sleep(0.05)
    h['sccb_write'](0x4202, 0x00); time.sleep(0.5)
    rb = h['sccb_read'](0x503D)
    rb_str = f'0x{rb:02X}' if rb is not None else 'READ_FAIL'
    print(f'      0x503D readback={rb_str}')

    # delta summary
    print('\n  toggle delta summary:')
    print(f'  {"metric":15s} {"sensor":>10s} {"test_pat":>10s} {"delta":>10s}')
    for key in ('mean', 'std', 'row_mean_std', 'col_mean_std'):
        s = m_sensor[key]; t = m_test[key]
        print(f'  {key:15s} {s:10.2f} {t:10.2f} {t - s:+10.2f}')
    print(f'  {"flat_rows":15s} {m_sensor["flat_rows"]:10d} '
          f'{m_test["flat_rows"]:10d} '
          f'{m_test["flat_rows"] - m_sensor["flat_rows"]:+10d}')


def dump_buffers(bufs, prefix: str):
    print('\n=== VDMA buffer summary ===')
    for i, b in enumerate(bufs):
        arr = np.asarray(b).reshape(HEIGHT, STRIDE)
        flat = int((arr.var(axis=1) < 1).sum())
        rm = arr.mean(axis=1)
        diffs = np.abs(np.diff(rm))
        big_jumps = int((diffs > 30).sum())
        nonzero = int((arr != BUF_FILL_PATTERN).sum())
        print(f'buf{i}: min={int(arr.min())} max={int(arr.max())} '
              f'mean={float(arr.mean()):.1f} flat_rows={flat}/{HEIGHT} '
              f'row_jumps>30={big_jumps} nonzero_vs_fill={nonzero}')
        path = f'{prefix}_buf{i}.raw'
        Path(path).write_bytes(arr.tobytes())
        png_path = f'{prefix}_buf{i}.png'
        Image.fromarray(arr).save(png_path)
        print(f'  wrote {path}')
        print(f'  wrote {png_path}')


def compare_with_demo(buf0_arr: np.ndarray) -> None:
    demo_path = Path(DEMO_LINUX_ANALOG_RAW)
    if not demo_path.exists():
        print(f'\n  [skip A/B] demo raw not at {demo_path}')
        return
    demo = np.fromfile(demo_path, dtype=np.uint8)
    if demo.size != HEIGHT * STRIDE:
        print(f'\n  [skip A/B] demo size {demo.size} != expected {HEIGHT * STRIDE}')
        return
    demo = demo.reshape(HEIGHT, STRIDE)
    cur = buf0_arr.reshape(HEIGHT, STRIDE)
    print(f'\n  === A/B vs 2026-05-24 LINUX_ANALOG demo ===')
    print(f'  {"metric":15s} {"current":>10s}   {"demo":>10s}   {"delta":>10s}')
    metrics = [
        ('mean',          lambda x: float(x.mean())),
        ('std',           lambda x: float(x.std())),
        ('flat_rows',     lambda x: float((x.var(axis=1) < 1).sum())),
        ('row_mean_std',  lambda x: float(x.mean(axis=1).std())),
        ('row_var_mean',  lambda x: float(x.var(axis=1).mean())),
    ]
    for name, fn in metrics:
        c = fn(cur)
        d = fn(demo)
        print(f'  {name:15s} {c:10.2f}   {d:10.2f}   {c - d:+10.2f}')


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bit', default=BIT_DEFAULT)
    ap.add_argument('--target-fps', type=float, default=15.0,
                    help='stop at first stage reaching this frame_fps')
    ap.add_argument('--p0', type=int, default=None)
    ap.add_argument('--p1', type=int, default=None)
    ap.add_argument('--idelay', type=int, default=None,
                    help='skip Stage 1 IDELAY sweep, use this tap directly')
    ap.add_argument('--sweep', action='store_true',
                    help='enable IDELAY sweep (4 taps × 1 BITSLIP). Default '
                         'off: 2026-05-25 verified the 64-iteration sweep '
                         'destabilises the chip. Bitstream-init defaults '
                         '(p0=0,p1=6,idelay=8) work without sweep.')
    ap.add_argument('--sweep-taps', type=str, default='0,8,16,24',
                    help='taps to walk if --sweep. Kept short to bound '
                         'bitslip-pulse count.')
    ap.add_argument('--measure-s', type=float, default=5.0)
    ap.add_argument('--sweep-window-s', type=float, default=2.0)
    ap.add_argument('--init-settle-s', type=float, default=10.0,
                    help='post-overlay-load wait for bitstream-init FSM to '
                         'finish initialising the chip')
    ap.add_argument('--run-python-init', action='store_true',
                    help='[diagnostic] also run Python chip_init (HW RESETB + '
                         '227 SCCB writes) in addition to the bitstream-init '
                         'FSM. NOT RECOMMENDED -- 2026-05-25 verified that '
                         'doing this leaves the chip emitting only short pkts '
                         '(long_pkt=0) on the mipi_div=2 bitstream. Default off.')
    ap.add_argument('--skip-stage6', action='store_true',
                    help='do not try 0x3108/0x4837 trim')
    ap.add_argument('--enable-hdmi', action='store_true',
                    help='also start MM2S engine so DDR is read out to HDMI')
    ap.add_argument('--hold-seconds', type=float, default=20.0,
                    help='after stage progression, hold VDMA running this '
                         'many seconds (HDMI live preview / observation)')
    ap.add_argument('--no-rewind', action='store_true',
                    help='skip absolute-write rewind to best healthy stage '
                         'before the final VDMA capture. Default: rewind.')
    ap.add_argument('--dump-prefix', type=str, default='/tmp/stage15fps')
    ap.add_argument('--stop-after-stage2', action='store_true',
                    help='Skip Stage 2a/2b/3/4/5/6/rewind. Useful for the '
                         '2026-05-30 ROM-based bitstream where the post-'
                         'Stage-2 progression (PLL/AEC/VTS sweeps via stream '
                         'cycle) destabilises the chip and overwrites the '
                         'boot-time LINUX_ANALOG values. Final VDMA capture '
                         'runs directly after Stage 2 baseline.')
    ap.add_argument('--linux-analog-in-bitstream', action='store_true',
                    help='Skip the Stage 2a / rewind runtime stream-cycle '
                         're-application of LINUX_ANALOG. Use with the '
                         '2026-05-30 ROM-based bitstream that bakes the '
                         '27-reg ANALOG batch into bitstream-init FSM '
                         '(steps 12..38). The runtime re-apply via stream '
                         'cycle was verified destructive on that bitstream '
                         '(chip degrades to flat 0x80 fill). When set, '
                         "cur_state['linux_analog'] is initialised True so "
                         'apply_state() also skips the analog batch.')
    ap.add_argument('--tuning-sccb', type=str, default='',
                    help='Comma-separated chip register tuning to apply via '
                         'stream cycle AFTER 0x4800=0x34 + 5s settle, BEFORE '
                         '--capture-503d-toggle. Format: "addr=val,addr=val" '
                         '(int literals, hex with 0x). Example for stripe '
                         'root-cause hypothesis testing: '
                         '"0x3C01=0x00" (H2 banding filter disable), '
                         '"0x5001=0x00" (H3 ISP gate sweep off), '
                         '"0x3503=0x07,0x3502=0x10,0x350B=0x40" (H4 AEC '
                         'manual + short exposure). Applied as a single stream '
                         'cycle batch with readback verification. Sensor mode '
                         'snapshot in capture_503d_toggle then reflects the '
                         'tuning effect on stripe (row_mean_std vs baseline '
                         '40.33).')
    ap.add_argument('--tuning-label', type=str, default='tuning',
                    help='Label string for --tuning-sccb run (used in dump '
                         'prefix and log header).')
    ap.add_argument('--capture-503d-toggle', action='store_true',
                    help='After 0x4800 apply + 5s settle, before the periodic '
                         'hold loop: snapshot bufs[0] (sensor mode), stream '
                         'cycle 0x503D=0x80 (color bar), wait 5s, snapshot '
                         'bufs[0] (test pattern), stream cycle 0x503D=0x00 '
                         '(restore). Writes two PNGs under --dump-prefix with '
                         '_503d_off_/_503d_on_ suffix. Used for stripe-vs-'
                         'sensor-pipeline root-cause split.')
    ap.add_argument('--frame-lines', type=lambda x: int(x, 0), default=480,
                    help='frame_lines_gpio value = the line-count frame-close '
                         'period (cfg_expected_frame_lines) on the line-count '
                         'bitstream. Set to the chip true LE/frame to phase-lock '
                         '(stop the vgrad roll). Default 480.')
    ap.add_argument('--test-pattern-val', type=lambda x: int(x, 0), default=0x80,
                    help='0x503D value for --capture-503d-toggle test-pattern '
                         'phase. 0x80=color bar (row-constant), 0x81=random, '
                         '0x82=color square (row-varying, Plan B). Default 0x80.')
    ap.add_argument('--chip-4800', type=lambda x: int(x, 0), default=0x34,
                    help='runtime override chip 0x4800 (MIPI_CTRL_0) after init. '
                         'Default = 0x34 (bit5=continuous_clk + bit4=line_sync_enable + bit2=LP11_idle). '
                         '2026-05-28 verified: this is required for chip to emit LS/LE short packets '
                         '(~500/s); RTL bitstream-init default 0x24 (bit5 only) gives LE=0/s and the '
                         'cfg_use_lsle=True frame_state rejects long packets, leaving VDMA at 0xAA prefill. '
                         'Pass 0x24 to skip the override (reproduces pre-2026-05-28 behavior). '
                         'See diary 20260528 and memory project_ov5640_4800_34_enables_lsle.')
    return ap.parse_args()


def main():
    install_vdma_cleanup_signals()
    args = parse_args()
    target = args.target_fps

    print(f'Loading bitstream {args.bit}')
    ol = Overlay(args.bit)
    print(f'Overlay loaded, settle {args.init_settle_s:.1f}s for bitstream-init FSM')
    time.sleep(args.init_settle_s)
    h = make_helpers(ol)

    if args.run_python_init:
        print('\n========== chip_init (Python) [diagnostic mode] ==========')
        nacks = chip_init(h, FULL_INIT_STEPS, label='FULL_INIT', settle_s=5.0)
        print(f'init complete, {nacks} NACKs')
    else:
        print('\n========== chip_init: SKIPPED ==========')
        print('  Relying on bitstream-init FSM only (2026-05-25 verified that')
        print('  re-running Python chip_init on v3 bitstream leaves long_pkt=0).')

    h['frame_lines_set_keep_cam'](value=args.frame_lines, use_lsle=True, expected_dt=0x00)
    print(f'  frame_lines (line-count close period) = {args.frame_lines}')
    time.sleep(0.3)

    stages: list[StageResult] = []
    cur_state = dict(BASELINE_STATE)
    snapshots: list[tuple[StageResult, dict]] = []
    vdma = None
    bufs = None

    try:
        # ---------- Stage 1: IDELAY/BITSLIP training ----------
        if args.idelay is not None and args.p0 is not None and args.p1 is not None:
            print(f'\n========== Stage 1: skipped (forced '
                  f'p0={args.p0} p1={args.p1} idelay={args.idelay}) ==========')
            h['bitslip_set'](args.p0, args.p1)
            h['idelay_set'](args.idelay, args.idelay)
            time.sleep(1.0)  # stabilise pause
            best_p = (args.p0, args.p1)
            best_t = args.idelay
        elif args.sweep:
            print('\n========== Stage 1: IDELAY sweep (1 BITSLIP × short tap list) ==========')
            taps = [int(s) for s in args.sweep_taps.split(',') if s.strip()]
            picked = sweep_idelay_and_bitslip(h, taps, window_s=args.sweep_window_s)
            if picked is None:
                print('ABORT: no activity in sweep, chip may be silent')
                return 2
            best_p, best_t = picked
        else:
            best_p, best_t = (0, 6), 8
            print(f'\n========== Stage 1: SKIPPED (fixed defaults '
                  f'p0={best_p[0]}, p1={best_p[1]}, idelay={best_t}) ==========')
            print('  2026-05-25 verified: sweeping perturbs the chip and leaves '
                  'long_pkt=0 at Stage 2. Use --sweep to opt in to a short walk.')
            h['bitslip_set'](*best_p)
            h['idelay_set'](best_t, best_t)
            time.sleep(1.0)  # stabilise pause

        # ---------- Stage 2: baseline frame_fps (post-init, pre-LINUX_ANALOG) ----------
        print('\n========== Stage 2: baseline frame_fps (no chip overrides) ==========')
        snap_s2 = chip_snapshot(h, 'pre-Stage-2 (after Stage 1 BITSLIP/IDELAY)')
        s2 = measure(h, 'stage2-baseline', dur=args.measure_s)
        stages.append(s2)
        snapshots.append((s2, dict(cur_state)))
        if not s2.healthy:
            print('ABORT: Stage 2 baseline unhealthy. Cannot proceed.')
            return 3
        if s2.frame_fps >= target:
            print(f'TARGET MET at Stage 2 ({s2.frame_fps:.2f} fps >= {target}).')

        last_fps = s2.frame_fps  # default; overwritten by later stages if they run
        if args.stop_after_stage2:
            print('\n========== --stop-after-stage2: skipping Stages 2a/2b/3/4/5/6/rewind ==========')
            print('  (2026-05-30 ROM-based bitstream: post-Stage-2 sweeps destabilise chip.)')
        # ---------- Stage 2a: apply Linux 27-reg analog batch ----------
        # This is the documented enabler of clean MIPI TX behaviour (see
        # diary 20260524 続編 3 + linux_analog_capture.py). Skipping it was
        # the second smoking gun behind the 2026-05-25 driver failure.
        last_fps = s2.frame_fps
        if args.linux_analog_in_bitstream:
            print('\n========== Stage 2a: SKIPPED (--linux-analog-in-bitstream) ==========')
            print('  Bitstream-init FSM already applied LINUX_ANALOG at boot;')
            print('  runtime stream-cycle re-apply is destructive on the 2026-05-30')
            print('  ROM-based bitstream. Trusting boot-time apply.')
            cur_state['linux_analog'] = True
            stages.append(s2)  # treat Stage 2 baseline as the "post-LINUX_ANALOG" state
            snapshots.append((s2, dict(cur_state)))
        elif last_fps < target and not args.stop_after_stage2:
            print('\n========== Stage 2a: Linux 27-reg analog batch ==========')
            apply_dict(h, 'LINUX_ANALOG', LINUX_ANALOG)
            cur_state['linux_analog'] = True
            s2a = measure(h, 'stage2a-linux-analog', dur=args.measure_s)
            stages.append(s2a)
            snapshots.append((s2a, dict(cur_state)))
            snap_s2a = chip_snapshot(h, 'post-Stage-2a (after LINUX_ANALOG stream cycle)')
            chip_diff('Stage 2 -> 2a', snap_s2, snap_s2a)
            if not s2a.healthy:
                print('WARN: Stage 2a unhealthy. Continuing anyway -- the '
                      '27-reg batch may need a longer settle, or chip is in '
                      'an unexpected state. The reverse (skip 27-reg) was '
                      'already verified worse on 2026-05-25.')
            last_fps = s2a.frame_fps
            if last_fps >= target:
                print(f'TARGET MET at Stage 2a ({last_fps:.2f} fps).')

        # ---------- Stage 2b: 0x4837 = 0x24 (verified peak on v3 bitstream) ----------
        if last_fps < target and not args.stop_after_stage2:
            print('\n========== Stage 2b: 0x4837 = 0x24 (pclk_period peak) ==========')
            prev_4837 = cur_state['pclk_4837']
            cur_state['pclk_4837'] = PCLK_PERIOD_PEAK
            stream_cycle_writes(h, [(0x4837, PCLK_PERIOD_PEAK)])
            s2b = measure(h, 'stage2b-pclk-peak', dur=args.measure_s)
            stages.append(s2b)
            if (not s2b.healthy) or s2b.crc_err_pct > 20.0:
                print('  Stage 2b degraded, reverting 0x4837=0x18')
                stream_cycle_writes(h, [(0x4837, 0x18)])
                cur_state['pclk_4837'] = prev_4837
                _ = measure(h, 'after-revert-4837', dur=2.0)
            else:
                last_fps = s2b.frame_fps
            snapshots.append((s2b, dict(cur_state)))

        # ---------- Stage 3: AEC short exposure ----------
        if last_fps < target and not args.stop_after_stage2:
            print('\n========== Stage 3: AEC manual short exposure ==========')
            prev_aec = cur_state['aec_dict']
            cur_state['aec_dict'] = LINUX_AEC
            apply_dict(h, 'LINUX_AEC', LINUX_AEC)
            s3 = measure(h, 'stage3-AEC', dur=args.measure_s)
            stages.append(s3)
            if not s3.healthy or s3.crc_err_pct > 10.0:
                print(f'  Stage 3 degraded (crc={s3.crc_err_pct:.1f}%), reverting AEC')
                apply_dict(h, 'revert AEC', BASELINE_AEC)
                cur_state['aec_dict'] = prev_aec
                _ = measure(h, 'after-revert-AEC', dur=2.0)
            elif s3.frame_fps >= target:
                print(f'TARGET MET at Stage 3 ({s3.frame_fps:.2f} fps).')
            snapshots.append((s3, dict(cur_state)))

        # ---------- Stage 4: PLL mult step-up (cap 0x50) ----------
        last_mult = 0x36  # baseline
        last_fps = stages[-1].frame_fps
        if last_fps < target and not args.stop_after_stage2:
            print('\n========== Stage 4a: PLL mult = 0x40 (=64) ==========')
            prev_mult = cur_state['pll_mult']
            cur_state['pll_mult'] = 0x40
            stream_cycle_writes(h, [(0x3036, 0x40)])
            s4a = measure(h, 'stage4a-mult64', dur=args.measure_s)
            stages.append(s4a)
            if (not s4a.healthy) or s4a.crc_err_pct > 20.0:
                print('  Stage 4a degraded, reverting mult to 0x36')
                stream_cycle_writes(h, [(0x3036, 0x36)])
                cur_state['pll_mult'] = prev_mult
                _ = measure(h, 'after-revert-mult', dur=2.0)
            else:
                last_mult = 0x40
                last_fps = s4a.frame_fps
            snapshots.append((s4a, dict(cur_state)))

        if last_fps < target and last_mult == 0x40 and not args.stop_after_stage2:
            print('\n========== Stage 4b: PLL mult = 0x50 (=80) ==========')
            prev_mult = cur_state['pll_mult']
            cur_state['pll_mult'] = 0x50
            stream_cycle_writes(h, [(0x3036, 0x50)])
            s4b = measure(h, 'stage4b-mult80', dur=args.measure_s)
            stages.append(s4b)
            if (not s4b.healthy) or s4b.crc_err_pct > 20.0:
                print('  Stage 4b degraded, reverting mult to 0x40')
                stream_cycle_writes(h, [(0x3036, 0x40)])
                cur_state['pll_mult'] = prev_mult
                _ = measure(h, 'after-revert-mult', dur=2.0)
            else:
                last_mult = 0x50
                last_fps = s4b.frame_fps
            snapshots.append((s4b, dict(cur_state)))

        # ---------- Stage 5: VTS reduction ----------
        if last_fps < target and not args.stop_after_stage2:
            for vts in (500, 600, 700):
                print(f'\n========== Stage 5: VTS = {vts} ==========')
                # VTS is just two SCCB writes; chip evaluates at next vsync.
                h['sccb_write'](0x380E, (vts >> 8) & 0xFF)
                h['sccb_write'](0x380F, vts & 0xFF)
                cur_state['vts'] = vts
                time.sleep(0.5)
                s5 = measure(h, f'stage5-vts{vts}', dur=args.measure_s)
                stages.append(s5)
                snapshots.append((s5, dict(cur_state)))
                if (not s5.healthy) or s5.crc_err_pct > 20.0:
                    print(f'  Stage 5 (VTS={vts}) degraded; backing off')
                    continue
                last_fps = s5.frame_fps
                if s5.frame_fps >= target:
                    break

        # ---------- Stage 6 (optional): 0x3108 / 0x4837 fine trim ----------
        if last_fps < target and not args.skip_stage6 and not args.stop_after_stage2:
            print('\n========== Stage 6: 0x3108=0x05 + 0x4837=0x16 (safe combo) ==========')
            prev_fine = cur_state['pll_fine']
            cur_state['pll_fine'] = SAFE_PLL_FINE
            stream_cycle_writes(h, list(SAFE_PLL_FINE.items()))
            s6 = measure(h, 'stage6-3108-4837', dur=args.measure_s)
            stages.append(s6)
            if (not s6.healthy) or s6.crc_err_pct > 20.0:
                print('  Stage 6 degraded, reverting')
                stream_cycle_writes(h, list(BASELINE_PLL_FINE.items()))
                cur_state['pll_fine'] = prev_fine
                _ = measure(h, 'after-revert-3108', dur=2.0)
            else:
                last_fps = s6.frame_fps
            snapshots.append((s6, dict(cur_state)))

        # ---------- Rewind to best healthy stage before hold ----------
        if not args.no_rewind and not args.stop_after_stage2:
            healthy_snaps = [(s, st) for (s, st) in snapshots if s.healthy]
            if healthy_snaps:
                best_s, best_state = max(healthy_snaps,
                                         key=lambda x: x[0].frame_fps)
                print(f'\n========== Rewind: applying state of best healthy '
                      f'stage "{best_s.label}" (frame_fps={best_s.frame_fps:.2f}) '
                      f'==========')
                apply_state(h, best_state, args.linux_analog_in_bitstream)
                s_rw = measure(h, 'rewind-verify', dur=args.measure_s)
                stages.append(s_rw)
                if not s_rw.healthy:
                    print('WARN: rewind did not re-establish healthy; continuing')
            else:
                print('\nWARN: no healthy stage to rewind to; using current chip state')

        # ---------- Final VDMA capture (+ optional HDMI live preview) ----------
        hold_s = args.hold_seconds
        hdmi = args.enable_hdmi
        tag = 'S2MM + MM2S (HDMI live)' if hdmi else 'S2MM only'
        print(f'\n========== Final VDMA capture, {tag}, hold {hold_s:.1f}s ==========')
        vdma, bufs = setup_vdma_capture(ol, enable_hdmi=hdmi)

        # 2026-05-28: apply chip 0x4800=0x34 (bit5+bit4) so chip emits LS/LE short
        # packets, allowing cfg_use_lsle=True frame_state to pass long packets to
        # VDMA (otherwise buffer stays at 0xAA prefill).
        #
        # Order matters (verified 2026-05-28 diary 20260528 evening): we need
        # 1) VDMA already running with prefill, 2) brief settle, 3) chip stream
        # cycle + 0x34 write, 4) wait 5s+ for chip to stabilize streaming with
        # new register. Applying BEFORE VDMA setup, or applying before stages
        # ran, results in chip stuck in saturated state (mean=128, flat rows).
        if args.chip_4800 != 0x24:
            print('-- Let VDMA prefill stabilise for 3s before chip register change')
            time.sleep(3.0)
            b54 = (args.chip_4800 >> 4) & 0x3
            emits = 'YES (LS/LE)' if b54 == 0x3 else 'NO'
            print(f'-- Apply 0x4800=0x{args.chip_4800:02X} '
                  f'(bit5+bit4=0b{b54:02b}, LS/LE emit: {emits})')
            h['sccb_write'](0x300E, 0x40); time.sleep(0.05)
            ok = h['sccb_write'](0x4800, args.chip_4800)
            h['sccb_write'](0x300E, 0x45); time.sleep(0.5)
            rb = h['sccb_read'](0x4800)
            rb_str = f'0x{rb:02X}' if rb is not None else 'READ_FAIL'
            print(f'   write_ok={ok}  readback={rb_str} (expected 0x{args.chip_4800:02X})')
            print('-- Wait 5s for chip to stabilise streaming with new 0x4800')
            time.sleep(5.0)
        else:
            print('-- 0x4800 override SKIPPED (--chip-4800 0x24 = no-op)')

        # Optional tuning-SCCB apply BEFORE capture (for hypothesis testing)
        tuning_regs = parse_tuning_sccb(args.tuning_sccb)
        if tuning_regs:
            apply_tuning_sccb(h, tuning_regs, args.tuning_label)

        # Optional 0x503D toggle capture (for stripe vs sensor pipeline split)
        if args.capture_503d_toggle:
            capture_503d_toggle(h, bufs, f'{args.dump_prefix}_{args.tuning_label}',
                                 pattern_val=args.test_pattern_val)

        # Periodic status during hold so the operator can see HDMI is alive.
        tick = 5.0
        t_end = time.monotonic() + hold_s
        i = 0
        while time.monotonic() < t_end:
            time.sleep(min(tick, max(0.0, t_end - time.monotonic())))
            i += 1
            sr = int(vdma.read(VDMAR.S2MM_VDMASR))
            decode_vdmasr(sr, f'  [{i*tick:>4.0f}s] S2MM_VDMASR')
            if hdmi:
                mm2s_sr = int(vdma.read(VDMAR.MM2S_VDMASR))
                decode_vdmasr(mm2s_sr, f'  [{i*tick:>4.0f}s] MM2S_VDMASR')
        stop_vdma(vdma)
        ts = time.strftime('%Y%m%d_%H%M%S')
        dump_buffers(bufs, f'{args.dump_prefix}_{ts}')
        compare_with_demo(np.asarray(bufs[0]))

        # ---------- Summary ----------
        print('\n\n========== STAGE SUMMARY ==========')
        print(f'{"stage":28s} {"fps":>6} {"long/s":>7} {"crc%":>6} {"fs":>3} {"fe":>3}')
        for s in stages:
            print(f'{s.label:28s} {s.frame_fps:6.2f} {s.long_rate:7.0f} '
                  f'{s.crc_err_pct:5.1f}% {s.fs:3} {s.fe:3}')

        print(f'\nBest BITSLIP=({best_p[0]},{best_p[1]}) IDELAY={best_t}')
        if last_fps >= target:
            print(f'PASS: final frame_fps {last_fps:.2f} >= target {target}')
            return 0
        else:
            print(f'PARTIAL: final frame_fps {last_fps:.2f} < target {target}; '
                  f'stability priority allows 10-20 fps with crc<=10%')
            return 1

    finally:
        if vdma is not None:
            stop_vdma(vdma)
        if bufs is not None:
            del bufs
        print('-- VDMA stopped, sshd safe')


if __name__ == '__main__':
    sys.exit(main())
