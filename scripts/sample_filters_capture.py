"""Acquire a gallery of image-processing sample stills on the PYNQ board.

For each curated filter (singles + combinations) this applies the live runtime
config and grabs ONE still. Each image is saved ON THE BOARD as
``<filter-name>.png`` (+ ``.npy``) under ``/home/xilinx/jupyter_notebooks/_capture/``.

Capture is a VDMA-safe single shot: S2MM started, settled, ALL frame buffers
copied, then STOPPED (the sshd-hang-safe path). Because the genlock'd S2MM
round-robins the buffers and an occasional frame-sync hiccup (EOL/SOF-early)
can leave one buffer at its 0xAA prefill, we read all three and pick the
fullest (max-variance) frame, and RETRY with a longer settle if every buffer
is still blank -- so no sample comes back as the 0xAA grey prefill.

Driven by ``scripts/deploy_sample_filters.py`` (uploads deps + this, runs it
over SSH with no manual steps, then pulls the stills into
``<repo>/_capture/samples/`` with clean ``<filter-name>.png`` names).

Per CLAUDE.md this uses ``pynq_bringup.setup_session()`` for bring-up, and the
``Cam`` facade (which installs the VDMA-cleanup signal handlers + atexit). All
processing slots are runtime/no-rebuild (point ops, 3x3 conv, DoG dual kernel,
cascade blur, dither) so the whole gallery comes from one bitstream.
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from pynq_bringup import setup_session
from camera_repl import Cam
from v65_capture import WIDTH, HEIGHT, STRIDE, configure_vdma_s2mm
import frame_height_stability as fhs
from oneshot_capture import detect_wrap_boundary, repair_edge_rows

OUTDIR = Path('/home/xilinx/jupyter_notebooks/_capture')


def _to_rgb(buf):
    """(HEIGHT, STRIDE) raw S2MM buffer -> (H, W, 3) RGB uint8 (or (H,W) Y8)."""
    bpp = STRIDE // WIDTH
    if bpp >= 3:                                  # 32b {0,R,G,B} little-endian -> [B,G,R,0]
        px = buf.reshape(HEIGHT, WIDTH, bpp)
        return np.stack([px[:, :, 2], px[:, :, 1], px[:, :, 0]], axis=-1).astype(np.uint8)
    return buf[:, :WIDTH]


def _tile_seams(a):
    """Count row-to-row content discontinuities in a raw (H, STRIDE) buffer
    (copied from oneshot_capture). A clean single frame has ~1 seam (the chip
    frame boundary); a free-running S2MM tiled grab has several. Used to pick the
    least-tiled grab over several tries."""
    g = a.astype(np.float64)
    w = (a != 0xAA).any(axis=1)
    prev = prevd = None
    seams = 0
    for i in range(a.shape[0]):
        if not w[i]:
            continue
        r = g[i] - g[i].mean()
        d = float(np.sqrt((r * r).sum()))
        if prev is not None and d > 0 and prevd > 0:
            if float((prev * r).sum()) / (prevd * d) < 0.5:
                seams += 1
        prev, prevd = r, d
    return seams


def _full_top(a):
    """Rows of contiguous real (non-prefill) content from row 0 down -- a clean
    genlock'd frame fills from row 0, a mid-frame S2MM start leaves a prefill top."""
    w = (a != 0xAA).any(axis=1)
    i = 0
    while i < a.shape[0] and w[i]:
        i += 1
    return i


def grab(cam, settle, grabs=6):
    """CONTINUOUS-genlock still capture. The VDMA runs both MM2S+S2MM started ONCE
    (in main) and is NOT restarted here, so the S2MM genlock stays locked to the
    frame TUSER and each buffer fills cleanly from row 0. We read the three
    rotating buffers several times and keep the best: fewest tile seams, then the
    most contiguous rows from the top (no prefill gap), then the tallest clean
    frame. The bottom frame-boundary wrap is edge-replicated. Returns
    (rgb_uint8, (seams, boundary)) or (None, None)."""
    best = best_key = None
    best_bnd = HEIGHT
    for _ in range(grabs):
        time.sleep(settle)
        for b in cam._bufs:
            if hasattr(b, 'invalidate'):
                b.invalidate()
            a = np.asarray(b).reshape(HEIGHT, STRIDE).copy()
            rows = int((a != 0xAA).any(axis=1).sum())
            if rows < 64:                                 # essentially blank
                continue
            bnd = detect_wrap_boundary(a)
            # fewest seams, then fewest prefill rows at the top, then tallest, then fullest
            key = (-_tile_seams(a), _full_top(a), bnd, rows)
            if best_key is None or key > best_key:
                best, best_key, best_bnd = a.copy(), key, bnd
    if best is None:
        return None, None
    if best_bnd < HEIGHT - 5:
        best = repair_edge_rows(best, best_bnd)           # hide bottom wrap (raw row replicate)
    rgb = _to_rgb(best)
    rgb[:, -3:] = rgb[:, -4:-3]                           # hide OV5640 right-edge artifact (3 px)
    return rgb, (-best_key[0], best_bnd)


def save(frame, name):
    OUTDIR.mkdir(parents=True, exist_ok=True)
    npy = (OUTDIR / f'{name}.npy').resolve()
    png = (OUTDIR / f'{name}.png').resolve()
    np.save(npy, frame)
    try:
        from PIL import Image
        Image.fromarray(frame, 'RGB' if frame.ndim == 3 else 'L').save(png)
    except Exception as e:
        print(f'   (png skipped: {e})')
    y = frame.mean(axis=2) if frame.ndim == 3 else frame
    print(f'   saved {name}.png  mean={y.mean():.1f} std={y.std():.1f} -> {png}')


def build_items(cam):
    """Curated [(name, apply_fn)] gallery. ``name`` becomes the file stem.
    Pipeline presets (``cam.pipeline``) clear all stages themselves; direct
    single filters are prefixed with ``cam.passthrough()`` to drop stale state."""
    def P(n):                       # named filter combination / preset
        return lambda: cam.pipeline(n)

    def POINT(op):                  # single point op (0-7)
        return lambda: (cam.passthrough(), cam.proc(op))

    def K(n):                       # single named 3x3 conv kernel
        return lambda: (cam.passthrough(), cam.k(n))

    def BLUR(sz):                   # cascade variable blur (effective 5/9/13)
        return lambda: (cam.passthrough(), cam.blur(sz))

    def DOG(n):                     # DoG dual-kernel preset
        return lambda: (cam.passthrough(), cam.dog(n))

    return [
        # ---- baseline / point ops (single) ----
        ('colour',        P('colour')),        # passthrough reference (health check)
        ('invert',        P('invert')),
        ('grayscale',     POINT(2)),
        ('binarize',      P('binarize')),
        ('r_only',        POINT(5)),
        ('g_only',        POINT(6)),
        ('b_only',        POINT(7)),
        # ---- spatial single filters ----
        ('gaussian',      P('gaussian')),      # 3x3 blur
        ('median',        P('median')),        # 3x3 impulse denoise
        ('sharpen',       P('sharpen')),
        ('emboss',        P('emboss')),
        ('sobel_x',       K('sobel_x')),       # single-direction gradient
        ('sobel_y',       K('sobel_y')),
        ('laplacian',     K('laplacian')),
        ('outline',       K('outline')),
        ('edges',         P('edges')),         # omnidirectional |Gx|+|Gy|
        # ---- cascade blur + DoG dual kernel ----
        ('blur_5',        BLUR(5)),
        ('blur_9',        BLUR(9)),
        ('blur_13',       BLUR(13)),
        ('dog_blob',      P('dog_blob')),
        ('dog_unsharp',   DOG('unsharp')),
        # ---- combinations (curated optimal chains) ----
        ('bin_edges',     P('bin_edges')),     # binarize -> Sobel
        ('edge_binary',   P('edge_binary')),   # Sobel -> binarize
        ('sketch',        P('sketch')),        # gray -> edges -> binarize
        ('gray_edges',    P('gray_edges')),
        ('denoise_edges', P('denoise_edges')), # median -> Sobel
        ('median_sketch', P('median_sketch')),
        ('smooth_sketch', P('smooth_sketch')),
        # ---- dither / halftone (final stage) ----
        ('halftone',      P('halftone')),      # gray -> 1-bit ordered
        ('poster',        P('poster')),        # 2-bit posterize
        ('edge_halftone', P('edge_halftone')),
        ('dither_random', P('dither_random')),
    ]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--download', type=int, default=1, help='1 = reprogram overlay')
    ap.add_argument('--hw', type=int, default=1, help='cam.go HW-lock FSM (hw=)')
    ap.add_argument('--val4800', type=lambda x: int(x, 0), default=0x14,
                    help='chip 0x4800 init value. 0x14 = continuous + line-sync: gives the '
                         'healthy fs=fe=30 constant-height non-rolling stream (SOF-synth + '
                         'force-480) that the VDMA genlock needs for CLEAN, untiled stills. '
                         '0x24 (no-LS) makes the frame height unstable (fs~2) -> tiled grabs.')
    ap.add_argument('--settle', type=float, default=1.0,
                    help='per-grab settle seconds (6 grabs/filter pick the least-tiled)')
    ap.add_argument('--grabs', type=int, default=6,
                    help='grabs per filter; the least-tiled is kept')
    ap.add_argument('--long-as-line', type=int, default=0,
                    help='set_long_as_line after lock: deliver no-LS long packets as '
                         'rows so the frame_state emits regular row/frame TUSER (no-LS path)')
    ap.add_argument('--force-480', type=int, default=0,
                    help='re-assert force_expected (close every frame at exactly 480 lines)')
    ap.add_argument('--only', default='',
                    help='comma list of filter names = capture ONLY these')
    args = ap.parse_args()

    ol, h = setup_session(download=bool(args.download))
    cam = Cam(ol, h)                       # installs VDMA-cleanup signal handlers + atexit
    cam.go(val4800=args.val4800, hw=bool(args.hw))
    if args.long_as_line and 'set_long_as_line' in h:
        h['set_long_as_line'](True)
        print('  long-as-line ENABLED (no-LS longs delivered as rows for regular frame TUSER)')

    # Health gate. A degraded OV5640 (repeated full-init register storms) still ACKs SCCB
    # (chip ID reads) but stops streaming pixels -> a black gallery. Gate on the RELIABLE
    # liveness signals (CLAUDE.md known-trap): fs/fe (~30) and pix_per_line (~640) -- NOT
    # long_pkt/ls/crc, which can read 0 on a healthy 30fps stream. Bail loudly if dead.
    m = fhs.measure_link(h, dur=3.0, label='health-gate')
    fs, fe, pix = m.get('fs', 0), m.get('fe', 0), m.get('pix_per_line', 0)
    if fs < 5 or fe < 5 or pix < 100:
        print('*** LINK DEAD (fs=%.1f/s fe=%.1f/s pix/line=%d): OV5640 not streaming '
              'pixels (degraded from repeated bring-ups). PHYSICALLY power-cycle the board '
              '(unplug power ~10s), then retry. Aborting -- no capture. ***' % (fs, fe, pix))
        cam.stop()
        print('SAMPLE_FILES=')
        return 2
    print(f'  health OK: fs={fs:.0f}/s fe={fe:.0f}/s pix/line={pix} -> capturing')
    # one continuous VDMA session (BOTH channels) so the S2MM genlock stays locked
    # to the frame TUSER across every filter -- restarting S2MM per grab desyncs it
    # and tiles. Started once here; grab() only re-reads the rotating buffers.
    cam._alloc()
    configure_vdma_s2mm(cam._vdma, cam._bufs, start_mm2s=True, start_s2mm=True)
    print('  VDMA continuous (MM2S+S2MM); settling genlock...')
    time.sleep(4.0)

    items = build_items(cam)
    if args.only:
        want = {s.strip() for s in args.only.split(',') if s.strip()}
        items = [it for it in items if it[0] in want]

    done, failed = [], []
    print(f'=== capturing {len(items)} image-processing samples ===')
    for i, (name, apply) in enumerate(items, 1):
        print(f'--- [{i}/{len(items)}] {name} ---')
        try:
            apply()
            time.sleep(0.4)                # let the config propagate before the grab
            frame, info = grab(cam, args.settle, grabs=args.grabs)
            if frame is None:
                print(f'  !! {name} blank (no frame landed)')
                failed.append(name)
                continue
            save(frame, name)
            print(f'   ({info[0]} tile seams, wrap boundary row {info[1]})')
            done.append(name)
        except Exception as e:             # one bad filter must not abort the gallery
            print(f'  !! {name} FAILED: {e!r}')
            failed.append(name)
            try:
                cam.stop()
            except Exception:
                pass

    try:
        cam.passthrough()                  # leave the pipeline in plain colour
    except Exception:
        pass
    cam.stop()

    print(f'=== done: {len(done)} ok, {len(failed)} failed ===')
    if failed:
        print('  failed: ' + ','.join(failed))
    # machine-readable manifest parsed by deploy_sample_filters.py for the pull
    print('SAMPLE_FILES=' + ','.join(done))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
