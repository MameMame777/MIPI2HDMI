#!/usr/bin/env python3
"""Frame-height stabilisation + Linux-mainline PLL standardisation (v2).

Per docs/plan/plan_frame_height_480_pll_mainline_20260612.md.

v2 findings driving this revision (first hardware run, 2026-06-12):
- Parser receives only ~330-530 long pkts/s and fs ~0.5-2/s, i.e. ~2-3% of
  the chip's nominal output (PCLK 54M / HTS 1600 -> 33 fps x 480 lines =
  ~16k lines/s). The prefill band is the direct consequence: between two
  received FS packets only ~300-1000 lines arrive. The extended counters
  (pkt_trunc, ecc_uncorr, drop_dt/vc, fs_overlap, fe_no_fs, fe_b/a480)
  are sampled here to localise where the other 97% die.
- The first s1 attempt left long_pkt=0 with fs/fe alive: classic
  "RGB565 disarmed" signature. The PLL batch now includes 0x4300/0x501F
  in the SAME stream cycle (mainline does PLL + format while stream off).
- VDMA accumulated DMAIntErr|SOFEarlyErr|EOLEarlyErr; once DMAIntErr is
  set the channel halts and every later height sample reads 0. The VDMA
  is now reset+reprogrammed at each scenario start and VDMASR is logged.

Exposure unit note: set_exposure(h, e) programs e LINES; the 0x3500-02
triplet holds lines*16 (1/16-line units). banding_isolation.read_exp_gain
under-reports by 16x; read_exp_lines() here is the corrected version.

Compliant with CLAUDE.md: pynq_bringup.setup_session() +
v65_capture.install_vdma_cleanup_signals().
"""
from __future__ import annotations
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from pynq import allocate, MMIO
from pynq_bringup import setup_session
import v65_capture as v65
from v65_capture import (
    install_vdma_cleanup_signals, configure_vdma_s2mm, stop_vdma,
    read_diagnostic_pages, decode_vdmasr, R,
    HEIGHT, STRIDE,
)
from flicker_exposure_sweep import stream_cycle_write, set_exposure
from banding_isolation import band_clean, save_frame
from full_init_steps import FULL_INIT_STEPS

OUT_PREFIX = '/home/xilinx/fhs'

ARM_REGS = [(0x4300, 0x6F), (0x501F, 0x01)]   # RGB565 arm (DT=0x22)

# Linux mainline ov5640_set_mipi_pclk() solution for VGA CSI-2 RGB565 2-lane
# (see plan doc). Applied together with ARM_REGS in one stream cycle, the
# same ordering mainline uses (PLL + format programmed while stream is off).
PLL_MAINLINE = [
    (0x3036, 0x30),   # PLL mult 54 -> 48
    (0x4837, 0x14),   # pclk_period 24 -> 20  (= 2e9 / 96 MHz sample rate)
    (0x3A08, 0x01), (0x3A09, 0x2C),   # B50 step 295 -> 300
    (0x3A0A, 0x00), (0x3A0B, 0xFA),   # B60 step 246 -> 250
    (0x3A0D, 0x03),                   # max band60 (996/250 = 3)
]
PLL_STOCK = [
    (0x3036, 0x36),
    (0x4837, 0x18),
    (0x3A08, 0x01), (0x3A09, 0x27),
    (0x3A0A, 0x00), (0x3A0B, 0xF6),
    (0x3A0D, 0x04),
]
VTS_15FPS = [
    (0x380E, 0x07), (0x380F, 0xD0),   # VTS 1000 -> 2000 (increase, not the
    (0x3A0D, 0x07), (0x3A0E, 0x06),   # FE-killing reduction of diary 20260525)
]
VTS_STOCK = [
    (0x380E, 0x03), (0x380F, 0xE8),
    (0x3A0D, 0x03), (0x3A0E, 0x03),   # post-s1 values (s3 only runs after s1)
]

VERIFY_REGS = [0x3034, 0x3035, 0x3036, 0x3037, 0x3108, 0x4837,
               0x3A08, 0x3A09, 0x3A0A, 0x3A0B, 0x3A0D, 0x3A0E,
               0x380C, 0x380D, 0x380E, 0x380F,
               0x4300, 0x501F, 0x4800, 0x300E]

RATE_KEYS = ('fs', 'fe', 'ls', 'le', 'long_pkt', 'crc_ok')
ERR_KEYS = ('crc_err', 'pkt_trunc', 'ecc_uncorr', 'drop_dt', 'drop_vc',
            'fs_ovl', 'fe_nofs', 'fe_b480', 'fe_a480', 'oth_short',
            'long_prefs', 'short_pkt')


def patch_init_steps(steps, overrides):
    """Return FULL_INIT_STEPS with addr->val overrides applied in place."""
    omap = dict(overrides)
    out = []
    for entry in steps:
        if entry != 'STREAM_ON' and entry[0] in omap:
            out.append((entry[0], omap.pop(entry[0])))
        else:
            out.append(entry)
    for addr, val in omap.items():
        out.insert(-1 if 'STREAM_ON' in steps else len(out), (addr, val))
    return out


def read_exp_lines(h):
    """AEC exposure in LINES + AGC gain in x (fixes the 16x under-report in
    banding_isolation.read_exp_gain)."""
    e0 = h['sccb_read'](0x3500) or 0
    e1 = h['sccb_read'](0x3501) or 0
    e2 = h['sccb_read'](0x3502) or 0
    g0 = h['sccb_read'](0x350A) or 0
    g1 = h['sccb_read'](0x350B) or 0
    raw = (e0 << 16) | (e1 << 8) | e2          # 1/16-line units
    gain = ((g0 & 0x03) << 8) | g1             # 1/16x units
    return raw / 16.0, gain / 16.0


def snap2(h) -> dict:
    """Extended counter snapshot (all relevant debug pages)."""
    rd = h['read_dbg']
    p02, p03, p04 = rd(0x02), rd(0x03), rd(0x04)
    p07, p18, p19 = rd(0x07), rd(0x18), rd(0x19)
    p1c, p1d, p1e = rd(0x1C), rd(0x1D), rd(0x1E)
    return dict(
        crc_ok=(p02 >> 16) & 0xFFFF, crc_err=p02 & 0xFFFF,
        short_pkt=(p03 >> 16) & 0xFFFF, long_pkt=p03 & 0xFFFF,
        pkt_trunc=(p04 >> 16) & 0xFFFF, ecc_uncorr=p04 & 0xFFFF,
        drop_dt=(p07 >> 16) & 0xFFFF, drop_vc=p07 & 0xFFFF,
        fs=(p18 >> 16) & 0xFFFF, fe=p18 & 0xFFFF,
        ls=(p19 >> 16) & 0xFFFF, le=p19 & 0xFFFF,
        fe_b480=(p1c >> 16) & 0xFFFF, fe_a480=p1c & 0xFFFF,
        fs_ovl=(p1d >> 16) & 0xFFFF, fe_nofs=p1d & 0xFFFF,
        oth_short=(p1e >> 16) & 0xFFFF, long_prefs=p1e & 0xFFFF,
    )


def measure_link(h, dur: float = 12.0, label: str = '', chunk: float = 3.0
                 ) -> dict:
    """Wrap-safe accumulated counter deltas over `dur` (chunked so 16-bit
    counters cannot wrap within one chunk at <20k events/s)."""
    tot = {k: 0 for k in snap2(h)}
    elapsed = 0.0
    prev = snap2(h)
    while elapsed < dur:
        step = min(chunk, dur - elapsed)
        time.sleep(step)
        cur = snap2(h)
        for k in tot:
            tot[k] += (cur[k] - prev[k]) % 65536
        prev = cur
        elapsed += step
    r = {k: tot[k] / dur for k in tot}
    lpf = (tot['long_pkt'] / tot['fs']) if tot['fs'] else 0.0
    crc_tot = tot['crc_ok'] + tot['crc_err']
    crc_err_pct = 100.0 * tot['crc_err'] / crc_tot if crc_tot else 0.0
    p05 = h['read_dbg'](0x05)
    res = dict(dur=dur, lines_per_frame=lpf, crc_err_pct=crc_err_pct,
               last_frame_lines=(p05 >> 16) & 0xFFFF,
               pix_per_line=p05 & 0xFFFF)
    res.update({k: r[k] for k in RATE_KEYS})
    res.update({f'{k}_tot': tot[k] for k in ERR_KEYS})
    print(f'  link[{label}] {dur:.0f}s: fs={r["fs"]:5.2f}/s fe={r["fe"]:5.2f}/s '
          f'ls={r["ls"]:7.1f}/s long={r["long_pkt"]:7.1f}/s '
          f'lines/frame={lpf:6.1f} crc_err={crc_err_pct:4.1f}% '
          f'last_frame_lines={res["last_frame_lines"]} '
          f'pix/line={res["pix_per_line"]}')
    errs = {k: tot[k] for k in ERR_KEYS if tot[k]}
    print(f'  link[{label}] err totals: {errs if errs else "none"}')
    return res


def link_gate(m: dict) -> bool:
    return (m['long_pkt'] > 100 and m['fs'] > 0.2 and m['fe'] > 0.2
            and m['crc_err_pct'] < 20.0)


def groundtruth_lines(h, dur: float = 8.0, label: str = '',
                      poll_dt: float = 0.015) -> dict:
    """Drop-insensitive chip active-line ground truth (band root-cause split).

    frame_asm_live_lines_core counts the LONG packets (= pixel-data lines)
    actually received between FS and FE, and is latched into last_fe_lines at
    the chip's FE short packet (mipi_to_hdmi_probe_top.sv:1966-1990). Per-line
    DROPS can only REDUCE these counts, never inflate them, so
    `max(last_fe_lines)` over many frames is a LOWER BOUND on the chip's active
    line count: if it reaches 480 the chip emits >=480 and the bottom band is a
    frontend loss on imperfect locks, NOT a short chip frame. fe_after_480
    (page 0x1C low, strictly >480) being nonzero is independent proof of >=480.
    NOTE: a clean exactly-480 frame increments NEITHER fe_before_480 nor
    fe_after_480 (the RTL gap at live==480), so the primary signal is max_fe.

    Polls page 0x1B {live_lines[31:16], last_fe_lines[15:0]} and page 0x05
    {last_frame_lines[31:16], pix/line}; last_fe_lines holds for a full frame
    (~60 ms at 16.75 fps) so a 15 ms poll samples every frame several times."""
    rd = h['read_dbg']
    max_fe = max_live = max_lfl = 0
    seen: dict[int, int] = {}              # last_fe_lines -> frames latched
    prev_fe = None
    t0 = time.time()
    while time.time() - t0 < dur:
        p1b = rd(0x1B)
        live = (p1b >> 16) & 0xFFFF
        lfl = p1b & 0xFFFF
        p05h = (rd(0x05) >> 16) & 0xFFFF
        if live > max_live:
            max_live = live
        if lfl > max_fe:
            max_fe = lfl
        if p05h > max_lfl:
            max_lfl = p05h
        if lfl != prev_fe:
            seen[lfl] = seen.get(lfl, 0) + 1
            prev_fe = lfl
        time.sleep(poll_dt)
    p1c = rd(0x1C)
    fe_b480 = (p1c >> 16) & 0xFFFF
    fe_a480 = p1c & 0xFFFF
    chip480 = (max_fe >= 480) or (max_live >= 480) or (max_lfl >= 480) or (fe_a480 > 0)
    out = dict(max_last_fe_lines=max_fe, max_live_lines=max_live,
               max_last_frame_lines=max_lfl, fe_before_480=fe_b480,
               fe_after_480=fe_a480, chip_emits_480=chip480,
               fe_values=dict(sorted(seen.items())))
    print(f'  groundtruth[{label}]: max(last_fe_lines)={max_fe} '
          f'max(live)={max_live} max(last_frame_lines)={max_lfl}  '
          f'fe_b480={fe_b480} fe_a480={fe_a480}  chip>=480={chip480}')
    print(f'  groundtruth[{label}]: last_fe_lines seen = '
          f'{ {k: v for k, v in sorted(seen.items())} }')
    return out


def refill_and_grab(bufs, sleep_s: float):
    for b in bufs:
        np.asarray(b).fill(0xAA)
        if hasattr(b, 'flush'):
            b.flush()
    time.sleep(sleep_s)
    arrs = []
    for b in bufs:
        if hasattr(b, 'invalidate'):
            b.invalidate()
        arrs.append(np.asarray(b).reshape(HEIGHT, STRIDE).copy())
    return arrs


def height_series(h, bufs, label: str, n: int, fs_rate: float) -> dict:
    """Per-frame written-rows distribution. Keep top-2 of 3 buffers per
    refill cycle (one may be in-flight); h==0 entries are buffers the VDMA
    never touched (too few frames in the window), reported separately."""
    period = 1.0 / fs_rate if fs_rate > 0.05 else 10.0
    sleep_s = min(max(4.5 * period, 1.0), 12.0)
    heights, rstds, means = [], [], []
    print(f'  height[{label}]: {n} cycles, refill sleep {sleep_s:.2f}s '
          f'(fs={fs_rate:.2f}/s)')
    for i in range(n):
        arrs = refill_and_grab(bufs, sleep_s)
        hs = []
        for a in arrs:
            rstd, mean, npre = band_clean(a)
            hs.append((HEIGHT - npre, rstd, mean))
        hs.sort(key=lambda t: t[0], reverse=True)
        for ht, rstd, mean in hs[:2]:          # drop possible in-flight buffer
            heights.append(ht)
            if ht > 0:
                rstds.append(rstd)
                means.append(mean)
        if i == 0:
            save_frame(arrs[0], f'{OUT_PREFIX}_{label}_first')
    exp, gain = read_exp_lines(h)
    hgt = np.array(heights)
    written = hgt[hgt > 0]
    n_zero = int((hgt == 0).sum())
    full_pct = 100.0 * float((written >= HEIGHT).mean()) if written.size else 0.0
    out = dict(full_pct=full_pct, n_zero=n_zero, n_samples=len(heights),
               h_min=int(written.min()) if written.size else 0,
               h_med=float(np.median(written)) if written.size else 0.0,
               h_max=int(written.max()) if written.size else 0,
               rstd=float(np.mean(rstds)) if rstds else 0.0,
               mean=float(np.mean(means)) if means else 0.0,
               exp_lines=exp, gain=gain,
               heights=[int(x) for x in heights])
    print(f'  height[{label}]: full480={full_pct:5.1f}% of written '
          f'(zeros={n_zero}/{len(heights)})  '
          f'min/med/max={out["h_min"]}/{out["h_med"]:.0f}/{out["h_max"]}  '
          f'rstd={out["rstd"]:5.1f} mean={out["mean"]:5.1f}  '
          f'AEC exp={exp:6.1f} lines gain={gain:.1f}x')
    return out


def dump_regs(h, label: str) -> dict:
    vals = {}
    for a in VERIFY_REGS:
        vals[f'0x{a:04X}'] = h['sccb_read'](a)
    print(f'  regs[{label}]: ' + ' '.join(
        f'{k}={("%02X" % v) if v is not None else "??"}'
        for k, v in vals.items()))
    return vals


def aec_auto(h, gain_ceiling: int | None = None) -> None:
    regs = [(0x3A00, 0x78), (0x3503, 0x00)]
    if gain_ceiling is not None:
        regs += [(0x3A18, (gain_ceiling >> 8) & 0x03),
                 (0x3A19, gain_ceiling & 0xFF)]
    stream_cycle_write(h, regs)


def vdma_restart(vdma, bufs) -> None:
    """Full reset + reprogram: clears latched DMAIntErr/SOFEarlyErr which
    otherwise halt S2MM and make every later height sample read as 0."""
    sr = vdma.read(R.S2MM_VDMASR)
    if sr & 0x0007F0:                          # any error bit latched
        decode_vdmasr(sr, 'S2MM pre-restart')
    stop_vdma(vdma)
    for b in bufs:
        np.asarray(b).fill(0xAA)
        if hasattr(b, 'flush'):
            b.flush()
    configure_vdma_s2mm(vdma, bufs, start_mm2s=False, start_s2mm=True)
    time.sleep(0.3)


def run_scenarios(h, vdma, bufs, tag: str, n_series: int) -> dict:
    """S0/S2 A/B scenario set. man300 = 1 x B50_mainline (10.0 ms at
    PCLK=48M; only 8.9 ms at stock mult=54 -> flicker null only after s1)."""
    res = {}
    scenarios = [
        ('aec',    lambda: aec_auto(h)),
        ('aecgc',  lambda: aec_auto(h, gain_ceiling=0x40)),
        ('man300', lambda: set_exposure(h, 300, gain=0x20)),
        ('man150', lambda: set_exposure(h, 150, gain=0x40)),
    ]
    for name, setup in scenarios:
        setup(); time.sleep(2.5)
        vdma_restart(vdma, bufs)
        m = measure_link(h, label=f'{tag}-{name}')
        res[f'{tag}_{name}_link'] = m
        res[f'{tag}_{name}'] = height_series(h, bufs, f'{tag}_{name}',
                                             n_series, m['fs'])
        sr = vdma.read(R.S2MM_VDMASR)
        res[f'{tag}_{name}']['vdmasr'] = sr
        if sr & 0x0007F0:
            decode_vdmasr(sr, f'S2MM post-{tag}-{name}')
    aec_auto(h)   # leave AEC auto for the next stage
    return res


def apply_batch(h, batch, label: str) -> dict:
    """PLL/timing batch + RGB565 re-arm in ONE stream cycle (mainline
    programs PLL and format while the stream is off, then streams on)."""
    full = list(batch) + ARM_REGS
    print(f'\n--- applying {label} (stream cycle, {len(full)} regs incl. arm) ---')
    stream_cycle_write(h, full)
    time.sleep(2.0)
    h['bitslip_set'](0, 6)
    h['idelay_set'](8, 8)
    time.sleep(0.5)
    dump_regs(h, label)
    m = measure_link(h, dur=8.0, label=label)
    if m['long_pkt'] == 0:
        print(f'  [{label}] long=0: extra RGB565 re-arm cycle ...')
        stream_cycle_write(h, ARM_REGS)
        time.sleep(2.0)
        m = measure_link(h, dur=8.0, label=f'{label}-rearm')
    return m


def clock_mode_sweep_init(h, pll_at_init: bool) -> dict:
    """0x4800 sweep with FULL ISOLATION: every point gets a fresh chip_init
    (RESETB + SW reset + 227-step replay with 0x4800 patched) because ANY
    stream cycle can collapse the link until the next full init (s1/man300
    lesson, 2026-06-12). The arm cycle here must NOT rewrite 0x4800.
    Final 0x34 point repeats the first to verify reproducibility."""
    res = {}
    for i, val in enumerate((0x34, 0x14, 0x24, 0x04, 0x34)):
        steps = patch_init_steps(list(FULL_INIT_STEPS), [(0x4800, val)])
        if pll_at_init:
            steps = patch_init_steps(steps, PLL_MAINLINE)
        v65.chip_init(h, steps, f'c-sweep 0x4800={val:02X}', settle_s=10.0)
        h['bitslip_set'](0, 6)
        h['idelay_set'](8, 8)
        h['frame_lines_set_keep_cam'](value=480, use_lsle=(val & 0x10 != 0),
                                      expected_dt=0x22)
        time.sleep(0.3)
        stream_cycle_write(h, ARM_REGS)        # arm WITHOUT touching 0x4800
        time.sleep(2.0)
        m = measure_link(h, dur=10.0, label=f'cinit-4800={val:02X}#{i}')
        res[f'cinit_4800_{val:02X}_{i}'] = m
    return res


def clock_mode_sweep(h) -> dict:
    """0x4800 MIPI ctrl sweep: does clock-lane gating (bit5) starve the
    D-PHY frontend? Probe finding 2026-06-12: only ~3% of packets reach the
    parser, in contiguous ~1-frame runs at ~1 Hz, with ALL error counters
    clean -> packets are invisible at SoT level. If continuous clock
    (bit5=0: 0x04/0x14, the Digilent reference value is 0x14) restores
    ~16k long/s, gating is the killer. Restores 0x34 at the end."""
    res = {}
    for val in (0x34, 0x14, 0x04, 0x24, 0x34):
        stream_cycle_write(h, [(0x4800, val)] + ARM_REGS)
        time.sleep(1.5)
        m = measure_link(h, dur=8.0, label=f'c-4800={val:02X}')
        res[f'c_4800_{val:02X}'] = m
    return res


def arm_rgb565(h) -> dict:
    h['frame_lines_set_keep_cam'](value=480, use_lsle=True, expected_dt=0x22)
    time.sleep(0.3)
    stream_cycle_write(h, ARM_REGS + [(0x4800, 0x34)])
    time.sleep(2.0)
    m = measure_link(h, dur=6.0, label='arm')
    if m['long_pkt'] == 0:
        print('  long_pkt=0: retrying RGB565 arm cycle once ...')
        stream_cycle_write(h, ARM_REGS)
        time.sleep(2.0)
        m = measure_link(h, dur=6.0, label='arm-retry')
    return m


def main() -> int:
    install_vdma_cleanup_signals()
    ap = argparse.ArgumentParser()
    ap.add_argument('--stages', default='p,s0,s1,s2',
                    help='comma list of p,c,s0,s1,s2,s3 (p=probe dump, '
                         'c=0x4800 clock-mode sweep, s3=VTS=2000 15fps)')
    ap.add_argument('--download', type=int, default=1)
    ap.add_argument('--full-init', type=int, default=1)
    ap.add_argument('--pll-at-init', type=int, default=0,
                    help='1 = bake PLL_MAINLINE into the init replay instead '
                         'of a runtime stream-cycle batch (skips s0/s1 A/B)')
    ap.add_argument('--series', type=int, default=6,
                    help='refill cycles per height series (x2 samples each)')
    args = ap.parse_args()
    stages = [s.strip() for s in args.stages.split(',') if s.strip()]

    summary = {'args': vars(args)}

    ol, h = setup_session(download=bool(args.download),
                          settle_s=(10.0 if args.download else 0.0),
                          raise_resetb=True)
    if args.full_init:
        steps = list(FULL_INIT_STEPS)
        if args.pll_at_init:
            steps = patch_init_steps(steps, PLL_MAINLINE)
            print('init replay: PLL_MAINLINE patched into FULL_INIT_STEPS')
        v65.chip_init(h, steps, 'frame-height init', settle_s=15.0)

    h['bitslip_set'](0, 6)
    h['idelay_set'](8, 8)
    m = arm_rgb565(h)
    summary['arm_link'] = m
    if m['long_pkt'] == 0:
        print('*** still long_pkt=0. Power-cycle the board and retry. ***')
        return 1
    dump_regs(h, 'post-arm')

    dvdma = ol.ip_dict['axi_vdma_0']
    vdma = MMIO(int(dvdma['phys_addr']), int(dvdma['addr_range']))
    bufs = [allocate(shape=(HEIGHT, STRIDE), dtype=np.uint8) for _ in range(3)]
    for buf in bufs:
        np.asarray(buf).fill(0xAA)
    configure_vdma_s2mm(vdma, bufs, start_mm2s=False, start_s2mm=True)
    time.sleep(0.5)

    rc = 0
    try:
        if 'p' in stages:
            print('\n========== P: probe (counter localisation) ==========')
            read_diagnostic_pages(h)
            summary['probe_link'] = measure_link(h, dur=15.0, label='probe')

        if 's0' in stages and not args.pll_at_init:
            print('\n========== S0: baseline (stock PLL mult=54, B50=295) ==========')
            summary.update(run_scenarios(h, vdma, bufs, 's0', args.series))

        if 's1' in stages and not args.pll_at_init:
            print('\n========== S1: mainline PLL batch (mult=48, B50=300) ==========')
            m = apply_batch(h, PLL_MAINLINE, 's1-mainline')
            summary['s1_link'] = m
            if not link_gate(m):
                print('*** S1 GATE FAILED -> reverting to stock PLL ***')
                mrev = apply_batch(h, PLL_STOCK, 's1-revert')
                summary['s1_revert_link'] = mrev
                summary['s1_verdict'] = 'REVERTED'
                rc = 2
            else:
                summary['s1_verdict'] = 'PASS'

        if 's2' in stages and summary.get('s1_verdict') == 'PASS':
            print('\n========== S2: scenarios on mainline PLL ==========')
            summary.update(run_scenarios(h, vdma, bufs, 's2', args.series))

        if 's3' in stages and summary.get('s1_verdict') == 'PASS':
            print('\n========== S3: VTS=2000 (15 fps variant) ==========')
            m = apply_batch(h, VTS_15FPS, 's3-vts2000')
            summary['s3_link'] = m
            if not link_gate(m):
                print('*** S3 GATE FAILED -> reverting VTS=1000 ***')
                summary['s3_revert_link'] = apply_batch(h, VTS_STOCK, 's3-revert')
                summary['s3_verdict'] = 'REVERTED'
            else:
                summary['s3_verdict'] = 'PASS'
                aec_auto(h); time.sleep(2.5)
                vdma_restart(vdma, bufs)
                mm = measure_link(h, label='s3-aec')
                summary['s3_aec_link'] = mm
                summary['s3_aec'] = height_series(h, bufs, 's3_aec',
                                                  args.series, mm['fs'])
                print('  (leaving VTS=2000; rerun with s1 only to go back)')

        if args.pll_at_init:
            print('\n========== scenarios on PLL-at-init (mainline from boot) ==========')
            dump_regs(h, 'pll-at-init')
            summary.update(run_scenarios(h, vdma, bufs, 's2', args.series))

        # LAST: each sweep point is a stream cycle that may collapse the
        # link until the next full init (s1 lesson) — never run before
        # the height scenarios.
        if 'c' in stages:
            print('\n========== C: 0x4800 clock-mode sweep (stream-cycle) ==========')
            summary.update(clock_mode_sweep(h))

        if 'cinit' in stages:
            print('\n========== CINIT: 0x4800 sweep, fresh init per point ==========')
            summary.update(clock_mode_sweep_init(h, bool(args.pll_at_init)))
    finally:
        stop_vdma(vdma)
        del bufs
        with open(f'{OUT_PREFIX}_summary.json', 'w') as f:
            json.dump(summary, f, indent=1, default=str)
        print(f'\nsummary -> {OUT_PREFIX}_summary.json')
        print('-- cleanup complete; sshd safe to remain up')

    print('\n=== Verdict guide ===')
    print('  H-CHIP-SHORT  : lines/frame < 480 at the parser')
    print('  H-DOWNSTREAM  : lines/frame ~ 480 but VDMA full480% low')
    print('  H-AEC-HUNT    : man300/man150 full480%=100, aec unstable')
    print('  PLL fix proof : s2_man300 rstd << s0_man300 rstd (10ms null)')
    print('  packet-loss   : err totals show where the ~97% of lines die')
    return rc


if __name__ == '__main__':
    sys.exit(main())
