"""Integer-exact software golden models for the img_proc slot family.

Each model is a STREAMING transliteration of the RTL: output beat k is predicted from
input beats 0..k with the line buffers / window shift registers / counters / LFSR
modelled exactly as in the SystemVerilog. That reproduces the true frame-edge behaviour
(rows < taps-1 see zero-initialised BRAM contents; cols < taps-1 see the previous row's
last columns carried over in the window shift register) without any don't-care masking:
EVERY output pixel is predicted, including borders. Multi-frame streams are exact too --
line buffers, window and LFSR are NOT reset between frames, matching the RTL.

Pixels are flat row-major 24-bit ints {R[23:16], G[15:8], B[7:0]}.

RTL sources of truth (read before changing anything here):
  rtl/img_proc/axis_rgb_conv3x3.sv   rtl/img_proc/axis_rgb_conv5x5.sv
  rtl/img_proc/axis_rgb_prefilter.sv rtl/img_proc/axis_rgb_proc_slot.sv
  rtl/img_proc/axis_rgb_dither.sv    rtl/img_proc/median9.sv
"""
from __future__ import annotations

from typing import List, Sequence


def _clamp8(v: int) -> int:
    return 0 if v < 0 else 255 if v > 255 else v


def _clamp12(v: int) -> int:
    """Signed 12-bit clamp, matching axis_rgb_conv5x5_sep clamp12() [-2048, 2047]."""
    return -2048 if v < -2048 else 2047 if v > 2047 else v


def _ch(p: int, sel: int) -> int:
    """Byte lane sel: 2=R, 1=G, 0=B (matches the RTL tap()/ch() helpers)."""
    return (p >> (sel * 8)) & 0xFF


def pack_coeffs(coeffs: Sequence[int]) -> int:
    """Pack signed ints into cfg_coeffs (idx*8 +: 8, two's complement)."""
    val = 0
    for idx, c in enumerate(coeffs):
        val |= (c & 0xFF) << (idx * 8)
    return val


# --------------------------------------------------------------------------- windows

def _windows(pixels: Sequence[int], width: int, taps: int):
    """Yield the RTL's taps x taps window (rows top->bottom, cols old->new) after each
    input beat. Models the read-before-write BRAM line buffers (zero initial state) and
    the window shift register exactly -- including the cross-row carry-over at c < taps-1
    and the zero rows at r < taps-1."""
    nbuf = taps - 1
    lb = [[0] * width for _ in range(nbuf)]     # lb[0]=lbA (row N-1) .. lb[nbuf-1] (oldest)
    win = [[0] * taps for _ in range(taps)]
    col = 0
    for px in pixels:
        prev = [lb[i][col] for i in range(nbuf)]    # read-before-write
        for i in range(nbuf - 1, 0, -1):            # lbD<=lbC<=lbB<=lbA cascade
            lb[i][col] = lb[i - 1][col]
        lb[0][col] = px
        # shift window left, insert newest column (top row = oldest line)
        newest = prev[::-1] + [px]                  # rows N-(taps-1) .. N
        for r in range(taps):
            for c in range(taps - 1):
                win[r][c] = win[r][c + 1]
            win[r][taps - 1] = newest[r]
        col = 0 if col == width - 1 else col + 1
        yield win


# --------------------------------------------------------------------------- conv

def conv_golden(pixels: Sequence[int], width: int, coeffs: Sequence[int],
                shift: int, absf: int, en: int, taps: int) -> List[int]:
    """axis_rgb_conv3x3 / axis_rgb_conv5x5: per channel
    sat((sum coeff[i]*tap[i]) >>> shift), cfg_abs -> |v|; cfg_en=0 -> window centre."""
    out: List[int] = []
    centre = taps // 2
    for win in _windows(pixels, width, taps):
        if not en:
            out.append(win[centre][centre])
            continue
        px = 0
        for sel in (2, 1, 0):
            acc = 0
            t = 0
            for r in range(taps):
                for c in range(taps):
                    acc += coeffs[t] * _ch(win[r][c], sel)
                    t += 1
            v = acc >> shift            # Python >> on negative int = arithmetic (>>>)
            if absf and v < 0:
                v = -v
            px = (px << 8) | _clamp8(v)
        out.append(px)
    return out


# --------------------------------------------------------------------------- conv5x5 separable

def conv5x5_sep_golden(pixels: Sequence[int], width: int, hcoeffs: Sequence[int],
                       vcoeffs: Sequence[int], hshift: int, vshift: int) -> List[int]:
    """axis_rgb_conv5x5_sep: separable 5x5 as a horizontal 1x5 pass (requantise to signed 12b
    via >>hshift + clamp12) followed by a vertical 5x1 pass (>>vshift + saturate to 8b).

    Streaming transliteration:
      * horizontal window ``hwin`` is a continuous 5-column shift register that is NOT reset on
        EOL, so columns < 4 of each row carry the previous row's tail exactly like the RTL
        (and it is not reset between frames either);
      * ``hcoeffs[i]`` weights ``hwin[i]`` where hwin[4] is the newest column (hc(0)=oldest);
      * the vertical pass keeps 4 zero-initialised line buffers of the signed-12 hout stream
        indexed by column (col = k % width, i.e. the RTL vcol that resets on hout EOL);
      * ``vcoeffs[0]`` weights the oldest row (N-4), vcoeffs[4] the current row (N).
    conv5x5_sep is only +/-2 LSB equivalent to a general 5x5 by design, so this models the
    ACTUAL two-pass separable requantisation, never a general-5x5 reference."""
    n = len(pixels)
    # --- horizontal pass -> hout[k][sel] signed-12, sel: 0=B 1=G 2=R ---
    hwin = [0] * 5                                  # hwin[4] = newest column
    hout: List[List[int]] = []
    for px in pixels:
        hwin = hwin[1:] + [px]
        chans = [0, 0, 0]
        for sel in (0, 1, 2):
            acc = sum(hcoeffs[i] * _ch(hwin[i], sel) for i in range(5))
            chans[sel] = _clamp12(acc >> hshift)    # arithmetic >> then signed-12 clamp
        hout.append(chans)
    # --- vertical pass over the hout stream, 4 line buffers (rows N-1..N-4) ---
    lb = [[[0, 0, 0] for _ in range(width)] for _ in range(4)]   # lb[0]=N-1 .. lb[3]=N-4
    out: List[int] = []
    for k in range(n):
        c = k % width
        cur = hout[k]                                           # current row (vr4)
        r1, r2, r3, r4 = lb[0][c], lb[1][c], lb[2][c], lb[3][c]  # read-before-write
        lb[3][c] = lb[2][c]
        lb[2][c] = lb[1][c]
        lb[1][c] = lb[0][c]
        lb[0][c] = cur
        px = 0
        for sel in (2, 1, 0):
            vsum = (vcoeffs[0] * r4[sel] + vcoeffs[1] * r3[sel] + vcoeffs[2] * r2[sel]
                    + vcoeffs[3] * r1[sel] + vcoeffs[4] * cur[sel])
            px = (px << 8) | _clamp8(vsum >> vshift)
        out.append(px)
    return out


# --------------------------------------------------------------------------- DoG combine

def dog_combine_golden(a_pixels: Sequence[int], b_pixels: Sequence[int], mode: int,
                       alpha: int, beta: int, shift: int, offset: int) -> List[int]:
    """axis_rgb_dog_combine: ordinally-aligned combine of two conv-branch output streams. The
    ordinal FIFO pairs the k-th A output with the k-th B output (A leads B by conv5x5's ~6-cycle
    valid latency, so pairing is clean from k=0), and both branches emit one output per input
    pixel in raster order -- so the k-th pair is the SAME spatial pixel. Modes: 0=A passthrough,
    1=B passthrough, 2=DoG(alpha*A - beta*B), 3=sum(alpha*A + beta*B); modes 2/3 do
    sat9((sel >>> shift) + offset) per channel (alpha/beta unsigned 8b, offset signed -256..255)."""
    out: List[int] = []
    for a, b in zip(a_pixels, b_pixels):
        px = 0
        for sel in (2, 1, 0):
            av, bv = _ch(a, sel), _ch(b, sel)
            if mode == 0:
                o = av
            elif mode == 1:
                o = bv
            else:
                pa, pb = alpha * av, beta * bv           # 16-bit unsigned products
                s = (pa + pb) if mode == 3 else (pa - pb)
                v = (s >> shift) + offset                # arithmetic >> on the signed sum
                o = 0 if v < 0 else 255 if v > 255 else v
            px = (px << 8) | o
        out.append(px)
    return out


def dog_chain_golden(pixels: Sequence[int], width: int, a_coeffs: Sequence[int], a_shift: int,
                     b_coeffs: Sequence[int], b_shift: int, mode: int, alpha: int, beta: int,
                     shift: int, offset: int) -> List[int]:
    """Full dog_chain_top = axis_rgb_conv3x3 (A) || axis_rgb_conv5x5 (B) -> dog_combine. Composes
    the two conv goldens (both cfg_abs=0, cfg_en=1) and the combiner -- bit-exact end to end."""
    a = conv_golden(pixels, width, a_coeffs, a_shift, 0, 1, 3)
    b = conv_golden(pixels, width, b_coeffs, b_shift, 0, 1, 5)
    return dog_combine_golden(a, b, mode, alpha, beta, shift, offset)


# --------------------------------------------------------------------------- point ops

def _point_op(p: int, op: int, thresh: int) -> int:
    """Shared point-op branch of axis_rgb_proc_slot (ops 0-7) and axis_rgb_prefilter
    (ops 0-7, 10-15 -> passthrough). Gray/threshold key on the GREEN channel (RTL y=g)."""
    r, g, b = _ch(p, 2), _ch(p, 1), _ch(p, 0)
    if op == 1:
        return ((~r & 0xFF) << 16) | ((~g & 0xFF) << 8) | (~b & 0xFF)
    if op == 2:
        return (g << 16) | (g << 8) | g
    if op == 3:
        return (b << 16) | (g << 8) | r
    if op == 4:
        return 0xFFFFFF if g > thresh else 0x000000
    if op == 5:
        return r << 16
    if op == 6:
        return g << 8
    if op == 7:
        return b
    return p                            # 0 and 10-15 = passthrough


def proc_slot_golden(pixels: Sequence[int], op: int, thresh: int) -> List[int]:
    """axis_rgb_proc_slot: pure point op on every beat."""
    return [_point_op(p, op, thresh) for p in pixels]


def prefilter_golden(pixels: Sequence[int], width: int, op: int,
                     thresh: int) -> List[int]:
    """axis_rgb_prefilter: 3x3 window front end (same as conv3x3); op 8 = gaussian
    (corners + 2*edges + 4*centre) >> 4 per channel, op 9 = per-channel 9-median,
    ops 0-7/10-15 = point op on the window CENTRE w[1][1]."""
    out: List[int] = []
    for win in _windows(pixels, width, 3):
        if op == 8:
            px = 0
            for sel in (2, 1, 0):
                corner = (_ch(win[0][0], sel) + _ch(win[0][2], sel)
                          + _ch(win[2][0], sel) + _ch(win[2][2], sel))
                edge = (_ch(win[0][1], sel) + _ch(win[1][0], sel)
                        + _ch(win[1][2], sel) + _ch(win[2][1], sel))
                cen = _ch(win[1][1], sel)
                px = (px << 8) | ((corner + 2 * edge + 4 * cen) >> 4)
            out.append(px)
        elif op == 9:
            px = 0
            for sel in (2, 1, 0):
                vals = sorted(_ch(win[r][c], sel) for r in range(3) for c in range(3))
                px = (px << 8) | vals[4]
            out.append(px)
        else:
            out.append(_point_op(win[1][1], op, thresh))
    return out


# --------------------------------------------------------------------------- dither

_BAYER4 = [0, 8, 2, 10,
           12, 4, 14, 6,
           3, 11, 1, 9,
           15, 7, 13, 5]


def _dith_ch(v: int, by: int, rnd: int, mode: int, n: int) -> int:
    """axis_rgb_dither dith_ch(). NB the RTL smear shifts (n<<1)/(n<<2) are 3-bit
    self-determined Verilog expressions, so the shift amounts wrap modulo 8 -- e.g.
    n=3 smears by 3,6,4 (not 3,6,12). Replicated bit-exactly here; do NOT "fix" it."""
    if n == 0 or n >= 7:
        return v
    drop = 8 - n
    if mode:
        bias = rnd & ((1 << drop) - 1)
    elif drop >= 4:
        bias = by << (drop - 4)
    else:
        bias = by >> (4 - drop)
    s = v + bias
    if s > 255:
        s = 255
    q = s & ~((1 << drop) - 1) & 0xFF
    o = q
    o |= o >> n
    o |= o >> ((2 * n) & 7)
    o |= o >> ((4 * n) & 7)
    return o & 0xFF


def dither_golden(pixels: Sequence[int], width: int, height: int,
                  ctrl: int) -> List[int]:
    """axis_rgb_dither: ordered (Bayer 4x4 on row%4/col%4) or random (8-bit Galois LFSR,
    seed 0xA5, taps 0x1D, advanced per valid beat, NOT reset between frames; each beat
    uses the pre-update state, same value for all 3 channels)."""
    en = ctrl & 1
    mode = (ctrl >> 1) & 1
    n = (ctrl >> 2) & 7
    out: List[int] = []
    lfsr = 0xA5
    fsz = width * height
    for k, p in enumerate(pixels):
        if not en:
            out.append(p)
        else:
            j = k % fsz                             # row resets on eof, col on eol
            r, c = divmod(j, width)
            by = _BAYER4[(r & 3) * 4 + (c & 3)]
            out.append((_dith_ch(_ch(p, 2), by, lfsr, mode, n) << 16)
                       | (_dith_ch(_ch(p, 1), by, lfsr, mode, n) << 8)
                       | _dith_ch(_ch(p, 0), by, lfsr, mode, n))
        lfsr = ((lfsr << 1) & 0xFF) ^ (0x1D if lfsr & 0x80 else 0)
    return out
