"""TPG hardware verification — multi-pattern edition.

Tests 3 visually distinct test patterns by switching pattern_sel at runtime:
  0: Vertical ramp    (same gray per row, dark→bright top-to-bottom)
  1: Horizontal ramp  (same gray per column, dark→bright left-to-right)
  2: Checkerboard     (32×32 pixel black/white cells)

For each pattern: captures one VDMA frame, compares to expected, saves PNG.

Structural pipeline checks (ECC, CRC, frame_lines, overflow) run once.
Pattern checks run per-pattern; all must pass for overall PASS.

Usage on PYNQ:
    python3 tpg_verify.py [--bit /path/to/bd_wrapper.bit] [--frames N]
"""
from __future__ import annotations
import argparse
import sys
import time
import numpy as np
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from pynq import MMIO
from pynq_bringup import setup_session
import v65_capture as v65
from v65_capture import install_vdma_cleanup_signals

WIDTH   = 640
HEIGHT  = 480
N_BUFS  = 3      # 3 buffers (VDMA cycles 0→1→2→0...)
AA_FILL = 0xAA   # Pre-fill sentinel

# frame_lines_gpio bit constants (match v65_capture.py)
APPLY_BIT        = 1 << 24
CAM_GPIO_BIT     = 1 << 25
TPG_RT_BIT       = 1 << 26
PATTERN_SEL_SHIFT = 27   # bits[28:27] = tpg_pattern_sel[1:0]

PATTERN_NAMES = {0: 'VERT_RAMP', 1: 'HORIZ_RAMP', 2: 'CHECKER'}

# Gray scale constants matching RTL (V_LINES=480, GRAY_SHIFT=9)
_GRAY_SHIFT = 9
_GRAY_SCALE = (255 * (1 << _GRAY_SHIFT) + 479 // 2) // 479  # = 273


def expected_gray8(pattern_sel: int) -> np.ndarray:
    """Return ideal gray8 array (H×W uint8) for the given pattern.

    Note: the pipeline extracts G-channel from RGB565 and outputs
    Y8 = (gray8 >> 2) << 2  (lower 2 bits zeroed).
    expected_y8() applies this quantization for pixel-exact comparison.
    """
    rows = np.arange(HEIGHT, dtype=np.int32)
    cols = np.arange(WIDTH,  dtype=np.int32)
    rr, cc = np.meshgrid(rows, cols, indexing='ij')

    if pattern_sel == 0:   # vertical ramp
        vert = np.clip((rows * _GRAY_SCALE) >> _GRAY_SHIFT, 0, 255).astype(np.uint8)
        return np.broadcast_to(vert[:, None], (HEIGHT, WIDTH)).copy()
    elif pattern_sel == 1: # horizontal ramp: cols 0..511 → 0..255, 512..639 → 255
        horiz = np.where(cols >= 512, 255, cols >> 1).astype(np.uint8)
        return np.broadcast_to(horiz[None, :], (HEIGHT, WIDTH)).copy()
    elif pattern_sel == 2: # checkerboard 32×32
        alt = ((rr >> 5) & 1) ^ ((cc >> 5) & 1)
        return (alt * 255).astype(np.uint8)
    else:                  # diagonal (row + col/2) mod 256
        return ((rows[:, None].astype(np.int32) +
                 (cols[None, :].astype(np.int32) >> 1)) & 0xFF).astype(np.uint8)


def expected_y8(pattern_sel: int) -> np.ndarray:
    """Expected Y8 after RGB565 G-channel extraction: Y8 = (gray8 >> 2) << 2."""
    g = expected_gray8(pattern_sel).astype(np.uint16)
    return ((g >> 2) << 2).astype(np.uint8)


def set_pattern(ol, pattern_sel: int) -> None:
    """Write pattern_sel[1:0] to frame_lines_gpio bits[28:27].

    Keeps cam_gpio=1 (RESETB high) and use_tpg=1.
    """
    word = (CAM_GPIO_BIT
            | TPG_RT_BIT
            | ((pattern_sel & 0x3) << PATTERN_SEL_SHIFT)
            | 480)           # frame_lines = 480
    ol.frame_lines_gpio.channel1.write(word, 0xFFFFFFFF)
    time.sleep(0.005)
    ol.frame_lines_gpio.channel1.write(word | APPLY_BIT, 0xFFFFFFFF)
    time.sleep(0.01)


def capture_frame(vdma, bufs) -> np.ndarray:
    """Start S2MM, wait for 3 frames, stop, return the last complete buffer."""
    for b in bufs:
        b[:] = AA_FILL
        b.flush()
    v65.configure_vdma_s2mm(vdma, bufs, start_mm2s=False, start_s2mm=True)
    time.sleep(0.20)   # ~10 TPG frames at 47 fps — enough to fill all buffers
    v65.stop_vdma(vdma)
    # Find the most recently completed (non-prefill) buffer
    frames = []
    for b in bufs:
        b.invalidate()
        arr = np.array(b)
        if arr.mean() != AA_FILL:
            frames.append(arr)
    if not frames:
        return np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    # Return the frame whose row-0 pixel values differ most from AA_FILL
    return frames[-1]


def check_pattern(frame: np.ndarray, pattern_sel: int) -> tuple[bool, str]:
    """Structural pass/fail for a captured frame against expected pattern.

    Returns (passed, detail_string).
    """
    exp_y8 = expected_y8(pattern_sel)
    mae = float(np.abs(frame.astype(np.int16) - exp_y8.astype(np.int16)).mean())

    if pattern_sel == 0:   # vertical ramp — row means must increase top→bottom
        row_means = frame.mean(axis=1)
        diff = np.diff(row_means)
        bwd  = int(np.sum(diff < -10))
        mono = bool(row_means[0] < row_means[HEIGHT // 2] < row_means[HEIGHT - 1])
        ok   = (bwd == 0) and mono and (mae < 8)
        return ok, f'bwd_jumps={bwd} mono={mono} MAE={mae:.1f}'

    elif pattern_sel == 1: # horizontal ramp — col means must increase left→right
        col_means = frame.mean(axis=0)
        diff  = np.diff(col_means[:512])    # only the ramp portion
        bwd   = int(np.sum(diff < -2))
        mono  = bool(col_means[0] < col_means[256] < col_means[511])
        sat   = bool(col_means[512:].mean() > 200)  # saturated right section
        ok    = (bwd == 0) and mono and sat and (mae < 8)
        return ok, f'bwd_jumps={bwd} mono={mono} sat={sat} MAE={mae:.1f}'

    elif pattern_sel == 2: # checkerboard — alternating 32×32 blocks
        rr, cc  = np.mgrid[0:HEIGHT, 0:WIDTH]
        is_white = (((rr >> 5) & 1) ^ ((cc >> 5) & 1)).astype(bool)
        white_mean = float(frame[is_white].mean())
        black_mean = float(frame[~is_white].mean())
        contrast   = white_mean - black_mean
        ok = (white_mean > 180) and (black_mean < 60) and (contrast > 150)
        return ok, f'white={white_mean:.0f} black={black_mean:.0f} contrast={contrast:.0f}'

    return False, 'unknown pattern'


def save_png(frame: np.ndarray, path: str, label: str) -> None:
    """Save frame as PNG with a 1-pixel red border for easy visual inspection."""
    from PIL import Image, ImageDraw
    img = Image.fromarray(frame, 'L').convert('RGB')
    d   = ImageDraw.Draw(img)
    for x in range(WIDTH):
        img.putpixel((x, 0), (255, 0, 0))
        img.putpixel((x, HEIGHT - 1), (255, 0, 0))
    for y in range(HEIGHT):
        img.putpixel((0, y), (255, 0, 0))
        img.putpixel((WIDTH - 1, y), (255, 0, 0))
    d.text((4, 4), label, fill=(255, 255, 0))
    img.save(path)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--bit',    default=None)
    ap.add_argument('--frames', type=int, default=5)
    ap.add_argument('--save',   default='/home/xilinx/tpg_capture.npy')
    args = ap.parse_args()

    install_vdma_cleanup_signals()

    # ------------------------------------------------------------------
    # 1. Load bitstream, switch to TPG mode before camera traffic arrives
    # ------------------------------------------------------------------
    print('=== TPG verify (multi-pattern): loading bitstream ===')
    ol, h = setup_session(bit_path=args.bit, settle_s=0.0, raise_resetb=False)
    read_dbg = h['read_dbg']

    print('Setting use_tpg_rt=1 and pattern=0 (VERT_RAMP) ...')
    h['frame_lines_set_keep_cam'](480, use_lsle=False, use_tpg=True)
    set_pattern(ol, 0)

    print('Waiting 12 s for SCCB init FSM + pipeline settle ...')
    time.sleep(12.0)

    # ------------------------------------------------------------------
    # 2. Pipeline health checks (run once, pattern-independent)
    # ------------------------------------------------------------------
    p00 = read_dbg(0x00)
    print(f'page0=0x{p00:08x}  setup_ready={(p00>>26)&1}  sccb_done={(p00>>24)&1}')

    h['frame_lines_set_keep_cam'](480, use_lsle=False, use_tpg=True)
    set_pattern(ol, 0)
    time.sleep(0.2)

    from pynq import allocate
    print(f'Allocating {N_BUFS} VDMA buffers {HEIGHT}×{WIDTH} ...')
    bufs = [allocate(shape=(HEIGHT, WIDTH), dtype=np.uint8) for _ in range(N_BUFS)]

    vdma_desc = ol.ip_dict['axi_vdma_0']
    vdma = MMIO(int(vdma_desc['phys_addr']), int(vdma_desc['addr_range']))

    # Pre-capture snapshot
    p04_pre = read_dbg(0x04)
    p03_pre = read_dbg(0x03)
    pkt_trunc_pre = (p04_pre >> 16) & 0xFFFF
    long_pkt_pre  = p03_pre & 0xFFFF

    # Warm-up capture (pattern 0) to flush pipeline
    _ = capture_frame(vdma, bufs)

    p04_post = read_dbg(0x04)
    p03_post = read_dbg(0x03)
    p02      = read_dbg(0x02)
    p05      = read_dbg(0x05)
    p07      = read_dbg(0x07)
    p9b      = read_dbg(0x9b)
    p9c      = read_dbg(0x9c)

    crc_ok          = (p02 >> 16) & 0xFFFF
    crc_err         = p02 & 0xFFFF
    long_pkt        = p03_post & 0xFFFF
    pkt_trunc_post  = (p04_post >> 16) & 0xFFFF
    last_frame_lines= (p05 >> 16) & 0xFFFF
    pix_per_line    = p05 & 0xFFFF
    drop_dt         = (p07 >> 16) & 0xFFFF
    tpg_sop_cnt     = p9b & 0xFFFF
    pkt_sop_cnt     = p9c & 0xFFFF
    pkt_trunc_delta = pkt_trunc_post - pkt_trunc_pre

    print('\n=== Pipeline health checks ===')
    results = []

    def check(label, cond, actual=''):
        ok = 'PASS' if cond else 'FAIL'
        results.append(cond)
        print(f'  [{ok}] {label}  {actual}')

    check('tpg_sop_cnt > 0',             tpg_sop_cnt > 0,          f'(={tpg_sop_cnt})')
    check('pkt_sop_cnt > 0',             pkt_sop_cnt > 0,          f'(={pkt_sop_cnt})')
    check('mux transparent',             tpg_sop_cnt == pkt_sop_cnt,
          f'(tpg={tpg_sop_cnt} pkt={pkt_sop_cnt})')
    check('last_frame_lines == 480',     last_frame_lines == 480,   f'(={last_frame_lines})')
    check('pix_per_line == 640',         pix_per_line == 640,       f'(={pix_per_line})')
    check('crc_err == 0',                crc_err == 0,              f'(={crc_err} ok={crc_ok})')
    check('drop_dt == 0',                drop_dt == 0,              f'(={drop_dt})')
    check('long_pkt > 0',                long_pkt > 0,              f'(={long_pkt})')
    check('pkt_trunc_delta == 0',        pkt_trunc_delta == 0,
          f'(delta={pkt_trunc_delta} pre={pkt_trunc_pre} post={pkt_trunc_post})')

    pipeline_ok = all(results)

    # ------------------------------------------------------------------
    # 3. Per-pattern capture and verification
    # ------------------------------------------------------------------
    print('\n=== Pattern captures ===')
    save_stem = args.save.replace('.npy', '')
    pat_results = []
    frames_by_pat = {}

    for pat in [0, 1, 2]:
        pname = PATTERN_NAMES[pat]
        print(f'\n--- Pattern {pat}: {pname} ---')
        set_pattern(ol, pat)
        time.sleep(0.10)   # settle: let TPG produce 4-5 frames of new pattern
        frame = capture_frame(vdma, bufs)
        frames_by_pat[pat] = frame

        # Pixel stats
        row_means = frame.mean(axis=1)
        col_means = frame.mean(axis=0)
        unwritten = int(np.all(frame == AA_FILL, axis=1).sum())
        print(f'  unwritten_rows={unwritten}  min={frame.min()}  max={frame.max()}')
        print(f'  row_means[0,120,240,360,479]: '
              f'{[int(row_means[i]) for i in [0, 120, 240, 360, 479]]}')
        print(f'  col_means[0,160,320,480,639]: '
              f'{[int(col_means[i]) for i in [0, 160, 320, 480, 639]]}')

        # Save npy
        np.save(f'{save_stem}_pat{pat}.npy', frame)

        # Save PNG
        png_path = f'{save_stem}_pat{pat}.png'
        try:
            save_png(frame, png_path, f'pat{pat}:{pname}')
            print(f'  Saved {png_path}')
        except Exception as e:
            print(f'  PNG save error: {e}')

        # Structural check
        ok, detail = check_pattern(frame, pat)
        status = 'PASS' if ok else 'FAIL'
        pat_results.append(ok)
        print(f'  [{status}] {pname}: {detail}')

    # Save primary capture (pattern 0) to canonical path
    np.save(args.save, frames_by_pat.get(0, np.zeros((HEIGHT, WIDTH), np.uint8)))
    print(f'\nPrimary frame saved to {args.save}')

    # ------------------------------------------------------------------
    # 4. Summary
    # ------------------------------------------------------------------
    print('\n=== RESULT ===')
    pipeline_pass = sum(1 for r in results if r)
    pipeline_fail = sum(1 for r in results if not r)
    pat_pass = sum(1 for r in pat_results if r)
    pat_fail = sum(1 for r in pat_results if not r)
    total_pass = pipeline_pass + pat_pass
    total_fail = pipeline_fail + pat_fail

    for i, (ok, pname) in enumerate(zip(pat_results, PATTERN_NAMES.values())):
        print(f'  [{"PASS" if ok else "FAIL"}] Pattern {i}: {pname}')

    print(f'\nPipeline: {pipeline_pass}/{pipeline_pass+pipeline_fail} pass')
    print(f'Patterns: {pat_pass}/{pat_pass+pat_fail} pass')
    print(f'Total   : {total_pass} passed, {total_fail} failed')

    if total_fail == 0:
        print('\n✓ TPG TEST PASSED')
        print('  RTL pipeline is clean. Image artifacts are camera/D-PHY origin.')
    else:
        print('\n✗ TPG TEST FAILED')
        print('  RTL or VDMA issue confirmed. Investigate pipeline independently of camera.')

    for b in bufs:
        del b


if __name__ == '__main__':
    main()
