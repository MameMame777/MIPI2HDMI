#!/usr/bin/env python3
"""Horizontal-banding origin isolation: T1-T5 hypothesis tests.

FPGA pipeline is proven clean (csi2_tpg 12/12 PASS + HDMI demo band-free,
2026-06-12). The remaining live-camera banding hypotheses:

  H1  100 Hz mains flicker (optical, Tokyo 50 Hz)  - vanishes in darkness,
      vanishes when exposure = k x 10 ms
  H2  sensor analog / BLC / row noise              - persists in darkness,
      persists at any exposure
  H3  AEC/AGC erratic (night mode bit, gain hunt)  - persists in darkness with
      AEC auto, vanishes with fully manual exposure+gain
  H4  D-PHY / FPGA                                 - already innocent (chip
      0x503D test pattern row-clean); T1 re-confirms on current bitstream

Tests (select with --tests):
  t1  chip test pattern 0x503D=0x84 (vgrad) -> frozen_pattern_test verdict
  t2  normal light + AEC auto: band period + frame-to-frame phase motion
  t3  DARK (lens covered) + AEC auto             <- user must cover the lens
  t4  DARK + manual exposure/gain                <- user must cover the lens
  t5  normal light + manual exposure sweep (flicker_exposure_sweep core)

Artifacts: /home/xilinx/banding_<test>_<i>.npy / .png  (downloaded to _capture/
by deploy_banding_test.py).

Compliant with CLAUDE.md: uses pynq_bringup.setup_session() and
v65_capture.install_vdma_cleanup_signals().  Bring-up mirrors
flicker_exposure_sweep.py (the script that produced the 2026-06-04 banding
data): bitslip(0,6), idelay(8,8), use_lsle=1, 0x4800=0x34.
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

from pynq import allocate, MMIO
from pynq_bringup import setup_session
import v65_capture as v65
from v65_capture import (
    install_vdma_cleanup_signals, configure_vdma_s2mm, stop_vdma,
    HEIGHT, STRIDE,
)
import frozen_pattern_test as fpt
from flicker_exposure_sweep import (
    set_exposure, banding_strength, grab, stream_cycle_write,
)
from full_init_steps import FULL_INIT_STEPS

OUT_PREFIX = '/home/xilinx/banding'
LAGS = list(range(40, 240))   # candidate band periods [image rows]


# ---------------------------------------------------------------------------
# Capture / metric helpers
# ---------------------------------------------------------------------------

def _invalidate(bufs):
    for b in bufs:
        if hasattr(b, 'invalidate'):
            b.invalidate()


def save_frame(img: np.ndarray, name: str) -> None:
    np.save(f'{name}.npy', img.astype(np.uint8))
    try:
        from PIL import Image
        Image.fromarray(img.astype(np.uint8)).save(f'{name}.png')
    except Exception as e:
        print(f'  (png save failed: {e})')


def band_metrics(img: np.ndarray) -> dict:
    """Banding strength of one frame: row-mean stdev + autocorr period."""
    ac, lag, mean, rstd = banding_strength(img, LAGS)
    return dict(mean=mean, row_stdev=rstd, band_ac=ac, band_period=lag)


def profile_shift(a: np.ndarray, b: np.ndarray, max_shift: int = 240):
    """Best circular vertical shift of row-mean profile b onto a.
    Moving bands (flicker beat) show a consistent nonzero drift; fixed bands
    (analog row noise) show shift ~ 0."""
    pa = a.mean(axis=1); pa = pa - pa.mean()
    pb = b.mean(axis=1); pb = pb - pb.mean()
    na = np.sqrt((pa * pa).sum()); nb = np.sqrt((pb * pb).sum())
    denom = na * nb + 1e-9
    best = (-2.0, 0)
    for s in range(-max_shift, max_shift + 1):
        c = float((pa * np.roll(pb, s)).sum() / denom)
        if c > best[0]:
            best = (c, s)
    return best[1], best[0]


def grab_series(bufs, n: int = 6, gap_s: float = 0.7):
    """Capture n temporally-spaced frames from the cycling VDMA buffers."""
    frames = []
    for _ in range(n):
        time.sleep(gap_s)
        _invalidate(bufs)
        frames.append(grab(bufs).copy())
    return frames


def report_series(frames, label: str, save_stem: str) -> dict:
    """Per-frame band metrics + inter-frame band phase motion. Saves frames."""
    print(f'\n  --- {label}: per-frame metrics ---')
    print('   frame |  mean | row_stdev | band_ac | period')
    mets = []
    for i, f in enumerate(frames):
        m = band_metrics(f)
        mets.append(m)
        print('    %3d  | %5.1f |  %7.2f  |  %.3f  |  %3d' %
              (i, m['mean'], m['row_stdev'], m['band_ac'], m['band_period']))
        save_frame(f, f'{save_stem}_{i}')

    print('  --- inter-frame band phase (row-profile circular shift) ---')
    shifts = []
    for i in range(len(frames) - 1):
        s, c = profile_shift(frames[i], frames[i + 1])
        shifts.append(s)
        print(f'    f{i}->f{i+1}: shift={s:+4d} rows  corr={c:5.2f}')

    rstd_mean = float(np.mean([m['row_stdev'] for m in mets]))
    ac_mean   = float(np.mean([m['band_ac'] for m in mets]))
    periods   = [m['band_period'] for m in mets]
    moving    = bool(np.mean([abs(s) for s in shifts]) > 5) if shifts else None
    print(f'  summary[{label}]: row_stdev_mean={rstd_mean:.2f} '
          f'band_ac_mean={ac_mean:.3f} periods={periods} '
          f'bands_moving={moving}')
    return dict(row_stdev_mean=rstd_mean, band_ac_mean=ac_mean,
                periods=periods, shifts=shifts, bands_moving=moving)


# ---------------------------------------------------------------------------
# Chip state helpers
# ---------------------------------------------------------------------------

def aec_auto(h) -> None:
    """Return to auto AEC/AGC, night mode OFF (0x3A00=0x70 is the verified
    runtime-safe value; bit5 banding filter stays ON)."""
    stream_cycle_write(h, [(0x3A00, 0x70), (0x3503, 0x00)])


def dump_banding_regs(h) -> None:
    """Read back every register relevant to the banding-filter computation."""
    regs = [
        ('3A00 AEC ctrl (bit5=band en, bit2=night)', 0x3A00),
        ('3A08 B50 step hi', 0x3A08), ('3A09 B50 step lo', 0x3A09),
        ('3A0A B60 step hi', 0x3A0A), ('3A0B B60 step lo', 0x3A0B),
        ('3A0D B60 max', 0x3A0D), ('3A0E B50 max', 0x3A0E),
        ('3503 AEC manual', 0x3503),
        ('3C01 5060 ctrl (bit7=manual)', 0x3C01),
        ('3C00 5060 ctrl2 (bit2=50Hz sel)', 0x3C00),
        ('3C0C 5060 detect (bit0: 1=50Hz)', 0x3C0C),
        ('380C HTS hi', 0x380C), ('380D HTS lo', 0x380D),
        ('380E VTS hi', 0x380E), ('380F VTS lo', 0x380F),
        ('3034 PLL mode', 0x3034), ('3035 PLL div', 0x3035),
        ('3036 PLL mult', 0x3036), ('3037 PLL root', 0x3037),
        ('3108 SCLK div', 0x3108),
        ('3500 exp[19:16]', 0x3500), ('3501 exp[15:8]', 0x3501),
        ('3502 exp[7:0]', 0x3502),
        ('350A gain hi', 0x350A), ('350B gain lo', 0x350B),
    ]
    print('\n=== Banding-related chip registers ===')
    vals = {}
    for label, a in regs:
        v = h['sccb_read'](a)
        vals[a] = v
        print(f'  0x{a:04X} {label:42s} = '
              f'{("0x%02X" % v) if v is not None else "READ_FAIL"}')
    if vals.get(0x380C) is not None and vals.get(0x380D) is not None:
        hts = (vals[0x380C] << 8) | vals[0x380D]
        print(f'  -> HTS = {hts}')
    if vals.get(0x380E) is not None and vals.get(0x380F) is not None:
        vts = (vals[0x380E] << 8) | vals[0x380F]
        print(f'  -> VTS = {vts}')
    if vals.get(0x3A08) is not None and vals.get(0x3A09) is not None:
        b50 = ((vals[0x3A08] & 0x03) << 8) | vals[0x3A09]
        print(f'  -> B50 step = {b50} lines')
    if vals.get(0x3A0A) is not None and vals.get(0x3A0B) is not None:
        b60 = ((vals[0x3A0A] & 0x3F) << 8) | vals[0x3A0B]
        print(f'  -> B60 step = {b60} lines')


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_t1(h, bufs) -> None:
    print('\n========== T1: chip test pattern 0x503D=0x84 (vgrad) ==========')
    print('Re-confirms D-PHY + FPGA innocence on the current (v26) bitstream.')
    res = fpt.capture_and_analyze(h, bufs, pattern='vgrad',
                                  dump_prefix=f'{OUT_PREFIX}_t1',
                                  settle_s=5.0, restore=True)
    print(f'  T1 verdict: {res["verdict"]}  '
          f'(expect "stable": D-PHY/FPGA clean -> H4 rejected)')


def run_t2(h, bufs, label: str = 't2',
           banner: str = 'normal light + AEC auto (baseline)') -> dict:
    print(f'\n========== {label.upper()}: {banner} ==========')
    frames = grab_series(bufs, n=6, gap_s=0.7)
    return report_series(frames, label, f'{OUT_PREFIX}_{label}')


def run_t4(h, bufs, exp_lines: int, gain: int) -> dict:
    print(f'\n========== T4: DARK + manual exposure={exp_lines} '
          f'gain=0x{gain:02X} ==========')
    set_exposure(h, exp_lines, gain=gain)
    time.sleep(1.5)
    frames = grab_series(bufs, n=6, gap_s=0.7)
    res = report_series(frames, 't4', f'{OUT_PREFIX}_t4')
    return res


def read_exp_gain(h):
    """Read back AEC exposure (lines) and AGC gain (1/16 units)."""
    e0 = h['sccb_read'](0x3500) or 0
    e1 = h['sccb_read'](0x3501) or 0
    e2 = h['sccb_read'](0x3502) or 0
    g0 = h['sccb_read'](0x350A) or 0
    g1 = h['sccb_read'](0x350B) or 0
    exp = ((e0 & 0x0F) << 12) | (e1 << 4) | ((e2 >> 4) & 0x0F)  # 1/16-line units
    gain = ((g0 & 0x03) << 8) | g1                              # 1/16x units
    return exp / 16.0, gain / 16.0   # -> (lines, x-gain)


def band_clean(img: np.ndarray, prefill: int = 0xAA):
    """Written-region horizontal-band stdev (excludes 0xAA prefill rows)."""
    a = img.astype(float)
    mask = (np.abs(a - prefill) < 4).mean(axis=1) > 0.9
    w = a[~mask]
    n_pre = int(mask.sum())
    if w.shape[0] < 10:
        return 0.0, 0.0, n_pre
    rp = w.mean(axis=1)
    return float(rp.std()), float(rp.mean()), n_pre


def measure_clean(h, bufs, label: str, save_stem: str, n: int = 4):
    frames = grab_series(bufs, n=n, gap_s=0.6)
    rstds, means, npres = [], [], []
    for i, f in enumerate(frames):
        rstd, mean, npre = band_clean(f)
        rstds.append(rstd); means.append(mean); npres.append(npre)
        save_frame(f, f'{save_stem}_{i}')
    exp, gain = read_exp_gain(h)
    print(f'  [{label}] band_rstd={np.mean(rstds):5.1f} mean={np.mean(means):5.1f} '
          f'prefill_rows={int(np.mean(npres)):3d}  AEC exp={exp:6.1f}lines gain={gain:.1f}x')
    return float(np.mean(rstds))


def run_t6(h, bufs) -> None:
    """Anti-banding config comparison (LIT scene, AEC auto). Datasheet 4.6.1.1:
    with band filter auto (0x3A00[5]=1) the chip drops below 1 band step in
    bright light -> flicker. Capping AGC gain ceiling (0x3A19) forces AEC to use
    a longer exposure (>= 1 band step = 10 ms) so flicker averages out. Manual
    exposure quantised to the band step is the runtime-safe fallback."""
    print('\n========== T6: anti-banding config comparison (LIT, AEC auto) ==========')
    print('  Point the camera at a normally-lit scene (NOT covered).')
    B50 = 295  # band step in lines (chip-computed, verified in dump)

    # C0: baseline as left by init (0x3A00=0x78, AEC auto)
    stream_cycle_write(h, [(0x3A00, 0x78), (0x3503, 0x00)])
    time.sleep(2.0)
    measure_clean(h, bufs, 'C0 baseline    ', f'{OUT_PREFIX}_t6_c0')

    # C1: AEC auto + low gain ceiling -> AEC prefers long exposure
    stream_cycle_write(h, [(0x3A00, 0x78), (0x3503, 0x00),
                           (0x3A18, 0x00), (0x3A19, 0x40)])  # max gain 4x
    time.sleep(2.5)
    measure_clean(h, bufs, 'C1 gainceil 4x ', f'{OUT_PREFIX}_t6_c1')

    # C2: AEC auto + even lower gain ceiling 2x
    stream_cycle_write(h, [(0x3A19, 0x20)])  # max gain 2x
    time.sleep(2.5)
    measure_clean(h, bufs, 'C2 gainceil 2x ', f'{OUT_PREFIX}_t6_c2')

    # C3: manual exposure quantised to 1 band step (10 ms), moderate gain
    er = int(round(B50 * 16))  # band step in 1/16-line register units
    set_exposure(h, er, gain=0x20)
    time.sleep(1.5)
    measure_clean(h, bufs, 'C3 man 1xBand  ', f'{OUT_PREFIX}_t6_c3')

    # C4: manual exposure 2 band steps (20 ms)
    set_exposure(h, er * 2, gain=0x10)
    time.sleep(1.5)
    measure_clean(h, bufs, 'C4 man 2xBand  ', f'{OUT_PREFIX}_t6_c4')

    print('  Lower band_rstd = less flicker. Compare C0 (baseline) vs C1-C4.')


def run_t5(h, bufs, exposures, gain: int) -> None:
    print(f'\n========== T5: manual exposure sweep (gain=0x{gain:02X}) ==========')
    print(' exp_lines |  mean | row_stdev | band_ac | period(img rows)')
    print(' ----------+-------+-----------+---------+-----------------')
    results = []
    for e in exposures:
        set_exposure(h, e, gain=gain)
        time.sleep(1.2)
        _invalidate(bufs)
        img = grab(bufs)
        m = band_metrics(img)
        results.append((e, m))
        save_frame(img, f'{OUT_PREFIX}_t5_exp{e}')
        print('   %5d   | %5.1f |  %7.2f  |  %.3f  |  %3d' %
              (e, m['mean'], m['row_stdev'], m['band_ac'], m['band_period']))
    if results:
        best = min(results, key=lambda r: r[1]['row_stdev'])
        worst = max(results, key=lambda r: r[1]['row_stdev'])
        print(f'\n  T5 summary: min row_stdev={best[1]["row_stdev"]:.2f} '
              f'@ exp={best[0]}; max={worst[1]["row_stdev"]:.2f} @ exp={worst[0]}')
        print('  H1 (flicker) signature: clear minima at exposures = k x period,')
        print('  spacing between minima = flicker period in exposure-line units.')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    install_vdma_cleanup_signals()
    ap = argparse.ArgumentParser()
    ap.add_argument('--tests', default='t1,t2,t5',
                    help='comma list of t1,t2,t3,t4,t5 (t3/t4 need lens covered)')
    ap.add_argument('--download', type=int, default=1,
                    help='1 = reprogram bitstream + settle; 0 = attach to running')
    ap.add_argument('--full-init', type=int, default=0,
                    help='1 = run FULL_INIT_STEPS runtime replay before tests '
                         '(use if bitstream-init FSM did not bring the chip up)')
    ap.add_argument('--exposures', default='440,495,550,605,660,715,770,825,880,935,990')
    ap.add_argument('--t5-gain', type=lambda x: int(x, 0), default=0x40)
    ap.add_argument('--t4-exp', type=int, default=440)
    ap.add_argument('--t4-gain', type=lambda x: int(x, 0), default=0xF8,
                    help='high gain amplifies dark-floor row noise (H2 probe)')
    args = ap.parse_args()
    tests = [t.strip() for t in args.tests.split(',') if t.strip()]

    # ---- bring-up (mirrors flicker_exposure_sweep.py, 2026-06-04 proven) ----
    ol, h = setup_session(download=bool(args.download),
                          settle_s=(10.0 if args.download else 0.0),
                          raise_resetb=True)
    if args.full_init:
        v65.chip_init(h, list(FULL_INIT_STEPS), 'banding full init', settle_s=15.0)

    h['bitslip_set'](0, 6)
    h['idelay_set'](8, 8)
    # RGB565 (DT=0x22) must be ARMED with a 0x300E stream cycle that also writes
    # 0x501F=0x01 + 0x4300=0x6F — the monolithic full-init write alone leaves the
    # chip emitting only LS/LE short packets (long_pkt=0). See memory
    # project_ov5640_rgb565_requires_stream_cycle. 0x4800=0x34 enables LS/LE.
    h['frame_lines_set_keep_cam'](value=480, use_lsle=True, expected_dt=0x22)
    time.sleep(0.3)
    stream_cycle_write(h, [(0x4300, 0x6F), (0x501F, 0x01), (0x4800, 0x34)])
    time.sleep(2.0)

    # link sanity
    b = h['snap'](); time.sleep(2.0); a = h['snap']()
    d = {k: (a[k] - b[k]) % 65536 for k in b}
    print(f'\nLink check (2s): long={d["long_pkt"]} crc_ok={d["crc_ok"]} '
          f'crc_err={d["crc_err"]} fs={d["fs"]} fe={d["fe"]} ls={d["ls"]}')
    if d['long_pkt'] == 0:
        print('*** long_pkt=0: chip not streaming pixel data. Retrying RGB565 '
              'stream cycle once more ...')
        stream_cycle_write(h, [(0x4300, 0x6F), (0x501F, 0x01)])
        time.sleep(2.0)
        b = h['snap'](); time.sleep(2.0); a = h['snap']()
        d = {k: (a[k] - b[k]) % 65536 for k in b}
        print(f'Link check retry (2s): long={d["long_pkt"]} crc_ok={d["crc_ok"]} '
              f'crc_err={d["crc_err"]} fs={d["fs"]} fe={d["fe"]} ls={d["ls"]}')
        if d['long_pkt'] == 0:
            print('*** still long_pkt=0. Power-cycle the board and retry. ***')
            return 1

    dump_banding_regs(h)

    dvdma = ol.ip_dict['axi_vdma_0']
    vdma = MMIO(int(dvdma['phys_addr']), int(dvdma['addr_range']))
    bufs = [allocate(shape=(HEIGHT, STRIDE), dtype=np.uint8) for _ in range(3)]
    for buf in bufs:
        np.asarray(buf).fill(0xAA)
    configure_vdma_s2mm(vdma, bufs, start_mm2s=False, start_s2mm=True)
    time.sleep(0.5)

    try:
        if 't1' in tests:
            run_t1(h, bufs)
        if 't2' in tests:
            aec_auto(h); time.sleep(2.0)
            run_t2(h, bufs, 't2', 'normal light + AEC auto (baseline)')
        if 't3' in tests:
            aec_auto(h); time.sleep(2.0)
            run_t2(h, bufs, 't3', 'DARK (lens covered) + AEC auto')
        if 't4' in tests:
            run_t4(h, bufs, args.t4_exp, args.t4_gain)
        if 't5' in tests:
            exposures = [int(x) for x in args.exposures.split(',') if x.strip()]
            run_t5(h, bufs, exposures, args.t5_gain)
        if 't6' in tests:
            run_t6(h, bufs)
    finally:
        stop_vdma(vdma)
        del bufs
        print('\n-- cleanup complete; sshd safe to remain up')

    print('\n=== Hypothesis decision matrix ===')
    print('  T3 bands gone                -> H1 (flicker) supported')
    print('  T3 bands persist, T4 gone    -> H3 (AEC/AGC erratic)')
    print('  T3 + T4 bands persist        -> H2 (sensor analog / BLC row noise)')
    print('  T5 minima at k x period      -> H1 confirmed (period = B50 step)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
