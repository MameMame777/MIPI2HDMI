#!/usr/bin/env python3
"""Frozen test-pattern vertical-integrity test for the MIPI -> VDMA path.

The OV5640 register 0x503D (PRE_ISP_TEST_SET1) makes the chip generate a test
pattern at the *pre-ISP* stage, bypassing the sensor pixel array / analog
frontend.  With the rolling bit (bit6) OFF the pattern is FROZEN: every chip
frame is byte-identical.  So if we capture N consecutive VDMA frame stores and
they are NOT identical, the difference can only come from the MIPI framing
(FS/FE/LS/LE timing) plus the FPGA frame/line assembly -- not from image
content.  This isolates *vertical-direction* defects (frame-phase rolling, line
drop / reorder) that a row-CONSTANT color bar physically cannot reveal.

0x503D bit map (datasheet 7.21, docs/ov5640_registers_extracted.md:567):
  bit7    : test pattern enable
  bit6    : rolling bar     (1 => NOT frozen; MUST be 0 for this test)
  bit[3:2]: style   00=standard 01=vert-grad-1 10=horiz-grad 11=vert-grad-2
  bit[1:0]: base    00=color bar 01=random 10=square 11=black

Why a *vertical gradient* (0x84/0x8C) is the best probe for vertical defects:
its per-row mean is a monotonic ramp, so a dropped / reordered / mis-phased line
shows up as a step or kink in an otherwise smooth ramp -- directly measurable.
A color bar / square / random pattern has a (near-)flat row-mean, so only the
raw pixel-equality fraction is usable there.

Usage
-----
As a module (on PYNQ, inside a driver that owns VDMA + chip helpers):
    import frozen_pattern_test as fpt
    fpt.capture_and_analyze(h, bufs, pattern='vgrad', dump_prefix='/tmp/frz')

Standalone, to re-analyse already-captured raw dumps (480x640 uint8):
    python3 frozen_pattern_test.py buf0.raw buf1.raw buf2.raw
"""
from __future__ import annotations

import sys
import time

import numpy as np

HEIGHT = 480
WIDTH = 640

# 0x503D presets.  bit6 (rolling) is kept 0 so every entry below is FROZEN.
TEST_PATTERNS = {
    'off':         0x00,                       # no test pattern (live sensor)
    'color_bar':   0x80,                       # 8 vertical bars; row-CONSTANT (blind to vertical defects)
    'random':      0x81,                       # per-pixel frozen random; row-mean flat
    'square':      0x82,                       # checkerboard; row-mean flat
    'black':       0x83,                       # solid black
    'vgrad':       0x84,                       # color bar + vertical gradient (mode1): BEST for vertical defects
    'hgrad':       0x88,                       # color bar + horizontal gradient
    'vgrad2':      0x8C,                       # color bar + vertical gradient (mode2)
}

# Patterns whose per-row mean is a usable (non-flat) vertical signal.
_ROW_RAMP_PATTERNS = {0x84, 0x8C}


def _as_frame(x) -> np.ndarray:
    """Coerce a buffer / path / array to an (HEIGHT, WIDTH) float64 frame."""
    if isinstance(x, (str, bytes)) and not isinstance(x, np.ndarray):
        data = np.fromfile(x, dtype=np.uint8)
    else:
        data = np.asarray(x).reshape(-1)
    data = data[:HEIGHT * WIDTH].astype(np.float64)
    return data.reshape(HEIGHT, WIDTH)


def best_vshift(a: np.ndarray, b: np.ndarray, max_shift: int = HEIGHT // 2):
    """Return (shift, corr) of the vertical roll of `b` that best matches `a`.

    Uses the full 2-D content (normalised cross-correlation over vertical
    shifts), so it works even when the per-row mean is flat (square/random)."""
    a = a - a.mean()
    b = b - b.mean()
    na = np.sqrt((a * a).sum())
    nb = np.sqrt((b * b).sum())
    denom = na * nb + 1e-9
    best = (-2.0, 0)
    for s in range(-max_shift, max_shift + 1):
        c = float((a * np.roll(b, s, axis=0)).sum() / denom)
        if c > best[0]:
            best = (c, s)
    return best[1], best[0]


def analyze_frozen_frames(frames, label: str = '', max_shift: int = HEIGHT // 2) -> dict:
    """Analyse a list of >=2 FROZEN-pattern frames and print a verdict.

    Returns a dict with the raw metrics. The core invariant: a frozen pattern
    must reproduce identically every frame, so any deviation is a
    framing/assembly defect localised to the vertical (line/frame) dimension."""
    fr = [_as_frame(f) for f in frames]
    n = len(fr)
    if n < 2:
        raise ValueError('need at least 2 frames to compare')

    print(f'\n=== frozen-pattern analysis {label} ({n} frames) ===')
    pix_eq, shifts, corrs, col_corrs = [], [], [], []
    for i in range(n - 1):
        a, b = fr[i], fr[i + 1]
        eq = float((a == b).mean())
        s, c = best_vshift(a, b, max_shift)
        # column profile (mean over rows): tests the HORIZONTAL / intra-line axis
        ca = a.mean(axis=0) - a.mean()
        cb = b.mean(axis=0) - b.mean()
        cc = float((ca * cb).sum() / (np.sqrt((ca * ca).sum()) * np.sqrt((cb * cb).sum()) + 1e-9))
        pix_eq.append(eq); shifts.append(s); corrs.append(c); col_corrs.append(cc)
        print(f'  buf{i}->buf{i+1}: pixel_equal={eq:6.1%}  best_vshift={s:+4d} lines '
              f'corr={c:5.2f}  column_corr={cc:5.2f}')

    eq_mean = float(np.mean(pix_eq))
    corr_mean = float(np.mean(corrs))
    # Use |corr| for the column (horizontal) axis: a periodic pattern (square /
    # color bar) can flip the column-profile sign on a sub-period horizontal
    # shift, and a signed mean would spuriously cancel. Magnitude = structure
    # preserved.
    col_mean = float(np.mean(np.abs(col_corrs)))
    nonzero_shift = any(abs(s) > 2 for s in shifts)

    # Caveat: vertical periodicity (color_bar/square/random repeat structure)
    # makes the vertical-shift correlation ambiguous (many partial matches), so a
    # low corr there does NOT reliably prove line-scramble. Only a vertically
    # MONOTONIC pattern (vgrad / vgrad2, 0x84 / 0x8C) gives an unambiguous
    # vertical-shift verdict.
    code = TEST_PATTERNS.get(label.replace('0x', '').lower(), None)
    try:
        code = code if code is not None else int(label, 0)
    except (ValueError, TypeError):
        code = None
    vmonotonic = code in _ROW_RAMP_PATTERNS

    print('\n  verdict:')
    if eq_mean > 0.95:
        print('  STABLE: frozen pattern reproduces identically => frame is '
              'phase-locked, no vertical defect.')
        verdict = 'stable'
    elif corr_mean > 0.85 and nonzero_shift:
        print(f'  ROLLING (rigid): high correlation at a NON-zero vertical shift '
              f'(~{shifts}) => lines are intact but the frame is NOT phase-locked '
              f'(content slides vertically each frame).')
        verdict = 'rolling'
    elif corr_mean > 0.85:
        print('  NEAR-STABLE: aligns at shift~0 but pixels differ (noise / minor '
              'jitter).')
        verdict = 'near-stable'
    elif vmonotonic:
        print(f'  LINE-SCRAMBLE: even at the best vertical shift the correlation '
              f'is low (corr~{corr_mean:.2f}) on a vertically-MONOTONIC pattern '
              f'=> lines are dropped / reordered, not merely shifted (vertical '
              f'assembly broken).')
        verdict = 'line-scramble'
    else:
        print(f'  UNSTABLE (ambiguous): low correlation (corr~{corr_mean:.2f}) but '
              f'this pattern is vertically PERIODIC, so the vertical-shift metric '
              f'cannot separate rolling from scramble. Re-run with pattern=vgrad '
              f'(0x84) for an unambiguous verdict.')
        verdict = 'unstable-ambiguous'

    if col_mean > 0.9:
        print(f'  HORIZONTAL OK: column profile stable across frames '
              f'(column_corr~{col_mean:.2f}) => intra-line pixel order is intact; '
              f'the defect is purely vertical.')
    else:
        print(f'  HORIZONTAL also affected: column_corr~{col_mean:.2f}.')

    return {
        'pixel_equal': pix_eq, 'vshift': shifts, 'corr': corrs,
        'column_corr': col_corrs, 'eq_mean': eq_mean, 'corr_mean': corr_mean,
        'column_corr_mean': col_mean, 'verdict': verdict,
    }


def resolve_pattern(pattern) -> int:
    """Accept a preset name ('vgrad'), an int, or a hex string ('0x84')."""
    if isinstance(pattern, str):
        if pattern.lower() in TEST_PATTERNS:
            return TEST_PATTERNS[pattern.lower()]
        return int(pattern, 0)
    return int(pattern)


def stream_cycle_write(h, reg: int, val: int) -> None:
    """Apply an OV5640 format/ISP change inside a 0x300E stream off->on cycle
    (idle writes are ignored by the chip; the 0x40->0x45 transition is what
    arms the change). See memory feedback_chip_format_change_requires_stream_cycle."""
    h['sccb_write'](0x300E, 0x40); time.sleep(0.05)
    h['sccb_write'](0x4202, 0x0F); time.sleep(0.05)
    h['sccb_write'](reg, val);     time.sleep(0.05)
    h['sccb_write'](0x300E, 0x45); time.sleep(0.05)
    h['sccb_write'](0x4202, 0x00); time.sleep(0.5)


def capture_and_analyze(h, bufs, pattern='vgrad', dump_prefix: str = '/tmp/frozen',
                        settle_s: float = 5.0, save_png: bool = True,
                        restore: bool = True) -> dict:
    """Full routine: stream-cycle the chosen frozen 0x503D pattern, settle,
    capture the (up to 3) consecutive VDMA frame stores, analyse them, and
    optionally restore live sensor mode.  `h` is the v65 helper dict (needs
    'sccb_write'/'sccb_read'); `bufs` are the running VDMA frame buffers."""
    val = resolve_pattern(pattern)
    if val & 0x40:
        print(f'  WARNING: 0x{val:02X} has the rolling bit (bit6) set -> NOT '
              f'frozen; the identical-frame test is invalid for this value.')
    ts = time.strftime('%Y%m%d_%H%M%S')
    print(f'\n========== frozen-pattern capture (0x503D=0x{val:02X}) ==========')
    stream_cycle_write(h, 0x503D, val)
    rb = h['sccb_read'](0x503D)
    print(f'  0x503D readback={("0x%02X" % rb) if rb is not None else "READ_FAIL"}')
    print(f'  settle {settle_s}s for ISP re-arm + full frames into all buffers')
    time.sleep(settle_s)

    frames = []
    for i, b in enumerate(bufs):
        arr = np.asarray(b).reshape(HEIGHT, WIDTH)
        frames.append(arr.astype(np.float64))
        path = f'{dump_prefix}_0x{val:02X}_{ts}_buf{i}.raw'
        np.asarray(b).reshape(HEIGHT, WIDTH).astype(np.uint8).tofile(path)
        if save_png:
            try:
                from PIL import Image
                Image.fromarray(arr.astype(np.uint8)).save(path[:-4] + '.png')
            except Exception:
                pass
        print(f'  wrote {path}')

    res = analyze_frozen_frames(frames, label=f'0x{val:02X}')

    if restore:
        print('  restoring live sensor mode (0x503D=0x00)')
        stream_cycle_write(h, 0x503D, 0x00)
    return res


def _main(argv) -> int:
    if len(argv) < 3:
        print('usage: python3 frozen_pattern_test.py <frame0.raw> <frame1.raw> '
              '[frame2.raw ...]')
        print('  (re-analyse already-captured 480x640 uint8 frozen-pattern dumps)')
        print('\navailable patterns:', ', '.join(f'{k}=0x{v:02X}'
                                                  for k, v in TEST_PATTERNS.items()))
        return 2
    analyze_frozen_frames(argv[1:], label='(files)')
    return 0


if __name__ == '__main__':
    raise SystemExit(_main(sys.argv))
