#!/usr/bin/env python3
"""v65 chip-init + VDMA S2MM capture in a single run.

Runs the OV5640.h cfg_init_ sequence the same way ov5640_v65_proper_sequence.py
does (verified to wake the chip and produce FS/FE), then sets up VDMA S2MM
to grab frames into DDR, samples key debug pages, and dumps the buffers.

Usage:
    python3 v65_capture.py [--bit /home/xilinx/mipi2hdml/bd_wrapper.bit]
                           [--p0 7 --p1 1]      # BITSLIP override
                           [--idelay 8]         # IDELAY tap
                           [--dump-prefix /tmp/v65cap]
"""
from __future__ import annotations
import argparse
import binascii
import signal
import time
from typing import Any, List

import numpy as np
from pynq import Overlay, MMIO, allocate

import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from full_init_steps import FULL_INIT_STEPS  # 227 writes + 'STREAM_ON' sentinel

BIT_DEFAULT = '/home/xilinx/mipi2hdml/bd_wrapper.bit'

# ----- AXI GPIO bit positions -----
SCCB_STATUS_OFFSET = 0x08
WRITE_APPLY_BIT    = 1 << 24
SCCB_READ_APPLY    = 1 << 26
GPIO_REQ           = 0x00
GPIO_REQ_TRI       = 0x04
APPLY_BIT          = 1 << 24
CAM_GPIO_BIT       = 1 << 25
TPG_RT_BIT         = 1 << 26   # frame_lines_gpio bit[26] → use_tpg_rt mux select
HS_SETTLE_GATE_BIT = 1 << 28   # frame_lines_gpio bit[28] → cfg_hs_settle_gate (per-line HS-SETTLE SoT gate in legacy continuous; band fix 2026-06-17)
SUP_ENABLE_BIT     = 1 << 29   # frame_lines_gpio bit[29] → D-PHY lane supervisor opt-in
SOF_SYNTH_BIT      = 1 << 30   # frame_lines_gpio bit[30] → cfg_sof_synth (open frame on LS when FS absent)
FORCE_EXPECTED_BIT = 1 << 31   # frame_lines_gpio bit[31] → cfg_force_expected (close at exactly expected lines; HDMI roll fix)
BUFR_CLR_BIT       = 1 << 27   # frame_lines_gpio bit[27] → runtime BUFR.CLR re-roll (byte-phase cal)
CLK_IDELAY_SHIFT   = 16        # idelay_gpio bits[20:16] → clock-lane IDELAY tap (byte-phase cal)

# ----- VDMA registers -----
class R:
    MM2S_VDMACR = 0x00
    MM2S_VDMASR = 0x04
    PARK_PTR    = 0x28
    MM2S_VSIZE  = 0x50
    MM2S_HSIZE  = 0x54
    MM2S_STRIDE = 0x58
    MM2S_ADDR0  = 0x5C
    S2MM_VDMACR = 0x30
    S2MM_VDMASR = 0x34
    S2MM_VSIZE  = 0xA0
    S2MM_HSIZE  = 0xA4
    S2MM_STRIDE = 0xA8
    S2MM_ADDR0  = 0xAC

RS            = 1 << 0
CIRCULAR_PARK = 1 << 1
RESET         = 1 << 2

WIDTH  = 640
HEIGHT = 480
# 2026-06-23: COLOR RGBA32 path (image-processing research base). The probe now
# captures 24-bit RGB888 and the BD axis_rgb24_to_vdma32 packs 1 pixel per 32-bit
# VDMA beat as {8'h00, R, G, B}, so one 640-px line = 640*4 = 2560 bytes on the
# VDMA port. STRIDE/HSIZE = 2560. For the legacy Y8 bitstream set BYTES_PER_PX=1
# (STRIDE=640: Y8 packs 4 px/beat -> still 640 bytes/line).
#   Y8 note (legacy): yuv422/rgb565 gray_unpack drops to 8-bit, axis_y8_to_vdma32
#   packs 4 Y/beat -> 640 bytes/line (do NOT scale by bpp in Y8 mode).
BYTES_PER_PX = 4          # 4 = RGBA32 color (current build); 1 = Y8 (legacy)
STRIDE = WIDTH * BYTES_PER_PX

# Named 3x3 conv kernels (row-major 9 signed coeffs + right-shift normalisation) for
# the programmable conv (set_conv_named / cam.k). Power-of-two / sum-{0,1} so they map
# to shift-add and need no non-power-of-two divide. See axis_rgb_conv3x3.sv.
CONV_KERNELS = {
    'identity':  ([0, 0, 0, 0, 1, 0, 0, 0, 0], 0),
    'gaussian':  ([1, 2, 1, 2, 4, 2, 1, 2, 1], 4),   # blur
    'sharpen':   ([0, -1, 0, -1, 5, -1, 0, -1, 0], 0),      # mild unsharp (4-neigh)
    'sharpen_hi':([-1, -1, -1, -1, 9, -1, -1, -1, -1], 0),  # strong unsharp (8-neigh)
    'sobel_x':   ([-1, 0, 1, -2, 0, 2, -1, 0, 1], 0),  # vertical edges
    'sobel_y':   ([-1, -2, -1, 0, 0, 0, 1, 2, 1], 0),  # horizontal edges
    'laplacian': ([0, -1, 0, -1, 4, -1, 0, -1, 0], 0), # edges
    'outline':   ([-1, -1, -1, -1, 8, -1, -1, -1, -1], 0),
    'emboss':    ([-2, -1, 0, -1, 1, 1, 0, 1, 2], 0),  # 3D relief
}

# General 5x5 Gaussian (outer product of [1,4,6,4,1], sum 256) for the DoG large branch.
GAUSS5 = ([1, 4, 6, 4, 1,  4, 16, 24, 16, 4,  6, 24, 36, 24, 6,
          4, 16, 24, 16, 4,  1, 4, 6, 4, 1], 8)
# Separable 1-D Gaussian (sum 16, shift 4 per pass) for the cascade blur stages S2/S3.
GAUSS1D = [1, 4, 6, 4, 1]
SEP_IDENTITY = [0, 0, 1, 0, 0]
# Named DoG presets for set_dog_named / cam.dog. Tuple = (small3x3, small_shift, large5x5,
# large_shift, alpha, beta, comb_shift, offset, mode). mode 2 = aA-bB (DoG), 3 = aA+bB.
DOG_PRESETS = {
    # Difference of Gaussians (G3/16 - G5/256) about 128 -> blob / band-pass / edge halo
    'blob':    (CONV_KERNELS['gaussian'][0], 4, GAUSS5[0], 8, 1, 1, 0, 128, 2),
    # unsharp: 2*identity - G5 -> edge boost (sharpen with a wide radius)
    'unsharp': (CONV_KERNELS['identity'][0], 0, GAUSS5[0], 8, 2, 1, 0, 0,   2),
}
N_FRAMES = 3
FRMDLY_SHIFT = 24

# Buffer pre-fill pattern: 0xAA so a written region stands out against
# the unwritten background, making partial-frame VDMA halts visible.
BUF_FILL_PATTERN = 0xAA

# ----- OV5640.h cfg_init_ (verbatim from v65) -----
CFG_INIT = [
    (0x3008, 0x42), (0x3103, 0x03),
    (0x3017, 0x00), (0x3018, 0x00), (0x3034, 0x18),
    (0x3035, 0x11), (0x3036, 0x38), (0x3037, 0x11), (0x3108, 0x01),
    (0x303D, 0x10), (0x303B, 0x19),
    (0x3630, 0x2e), (0x3631, 0x0e), (0x3632, 0xe2), (0x3633, 0x23),
    (0x3621, 0xe0), (0x3704, 0xa0), (0x3703, 0x5a),
    (0x3715, 0x78), (0x3717, 0x01), (0x370b, 0x60), (0x3705, 0x1a),
    (0x3905, 0x02), (0x3906, 0x10), (0x3901, 0x0a), (0x3731, 0x02),
    (0x3600, 0x37), (0x3601, 0x33),
    (0x302d, 0x60), (0x3620, 0x52), (0x371b, 0x20), (0x471c, 0x50),
    (0x3a13, 0x43), (0x3a18, 0x00), (0x3a19, 0xf8),
    (0x3635, 0x13), (0x3636, 0x06), (0x3634, 0x44), (0x3622, 0x01),
    (0x3c01, 0x34), (0x3c04, 0x28), (0x3c05, 0x98),
    (0x3c06, 0x00), (0x3c07, 0x08), (0x3c08, 0x00), (0x3c09, 0x1c),
    (0x3c0a, 0x9c), (0x3c0b, 0x40),
    (0x503d, 0x80), (0x3820, 0x46),
    (0x300e, 0x45), (0x4800, 0x14), (0x302e, 0x08),
    (0x4300, 0x6f), (0x501f, 0x01),
    (0x4713, 0x03), (0x4407, 0x04),
    (0x440e, 0x00), (0x460b, 0x35), (0x460c, 0x20), (0x3824, 0x01),
    (0x5000, 0x07), (0x5001, 0x03),
]


def make_helpers(ol: Any):
    def wait_sccb_idle(t: float = 10) -> int:
        # bit[1]=ready set かつ bit[4]=busy, bit[0]=pending clear。
        # bit[3]=error は latch 型なので idle 条件に含めない (NACK で hang を防ぐ)。
        deadline = time.monotonic() + t
        while time.monotonic() < deadline:
            st = ol.sccb_gpio.read(SCCB_STATUS_OFFSET)
            if (st & 0x02) and (st & 0x11) == 0:
                return st
            time.sleep(0.02)
        return st

    def sccb_write(addr: int, value: int, t: float = 3.0) -> bool:
        sccb = ol.sccb_gpio
        a = int(addr) & 0xFFFF
        v = int(value) & 0xFF
        wait_sccb_idle()
        sccb.write(0x04, 0)
        base = (v << 16) | a
        sccb.write(0x00, base)
        sccb.write(0x00, base | WRITE_APPLY_BIT)
        sccb.write(0x00, base)
        deadline = time.monotonic() + t
        while time.monotonic() < deadline:
            time.sleep(0.003)
            st = sccb.read(SCCB_STATUS_OFFSET)
            # status_out layout after 2026-05-24 続編 3 RTL change:
            #   bits[31:24] = write_last_addr[7:0]  (low byte only)
            #   bits[23:16] = write_ack_err_count[7:0]
            #   bit[2] = write_done, bit[3] = write_error (preserved)
            # Match the low byte of last_addr; that pins us to the current
            # transaction without requiring the full 16-bit addr (which the
            # RTL no longer surfaces).
            if ((st >> 24) & 0xFF) == (a & 0xFF) and (st & ((1 << 2) | (1 << 3))):
                return bool(st & (1 << 2)) and not bool(st & (1 << 3))
        return False

    def set_conv_coeff(idx: int, value: int) -> None:
        """Load one programmable 3x3-conv kernel element via the SCCB GPIO at the
        reserved address 0xFE0i (the probe intercepts it -> conv_coeff_reg, NOT the
        chip; no SCCB done-wait). idx 0..8 = signed coeff (two's-complement byte);
        idx 9 = right shift (0..15). See rtl/img_proc/axis_rgb_conv3x3.sv."""
        sccb = ol.sccb_gpio
        a = 0xFE00 | (int(idx) & 0x0F)
        v = int(value) & 0xFF
        wait_sccb_idle()
        sccb.write(0x04, 0)
        base = (v << 16) | a
        sccb.write(0x00, base)
        sccb.write(0x00, base | WRITE_APPLY_BIT)   # apply edge; probe intercepts 0xFE
        sccb.write(0x00, base)
        time.sleep(0.002)

    def set_conv_kernel(coeffs, shift: int = 0) -> None:
        """Load a full programmable 3x3 kernel: coeffs = 9 signed ints (row-major,
        idx 0=top-left .. 8=bottom-right), shift = right-shift normalisation. Examples:
        Gaussian [1,2,1,2,4,2,1,2,1] shift 4; Sobel-X [-1,0,1,-2,0,2,-1,0,1] 0; sharpen
        [0,-1,0,-1,5,-1,0,-1,0] 0; emboss [-2,-1,0,-1,1,1,0,1,2] 0. Then set_proc_op(8)
        selects the conv (conv mode); set_proc_op(0) returns to the point path."""
        cs = list(coeffs)[:9] + [0] * max(0, 9 - len(coeffs))
        for i, c in enumerate(cs):
            set_conv_coeff(i, int(c) & 0xFF)
        set_conv_coeff(9, int(shift) & 0xF)

    def set_conv_named(name: str) -> None:
        """Load a named 3x3 kernel from CONV_KERNELS (identity/gaussian/sharpen/
        sharpen_hi/sobel_x/sobel_y/laplacian/outline/emboss), then the caller selects
        conv mode with set_proc_op(8)."""
        if name not in CONV_KERNELS:
            raise ValueError(f'unknown kernel {name!r}; choices: {sorted(CONV_KERNELS)}')
        coeffs, shift = CONV_KERNELS[name]
        set_conv_kernel(coeffs, shift)

    def fe_write(lo: int, value: int) -> None:
        """Write one byte to a reserved 0xFE-page processing register (probe-intercepted,
        NOT sent to the chip). lo = address low byte (see the probe 0xFE map), value = byte."""
        sccb = ol.sccb_gpio
        a = 0xFE00 | (int(lo) & 0xFF)
        v = int(value) & 0xFF
        wait_sccb_idle()
        sccb.write(0x04, 0)
        base = (v << 16) | a
        sccb.write(0x00, base)
        sccb.write(0x00, base | WRITE_APPLY_BIT)
        sccb.write(0x00, base)
        time.sleep(0.002)

    def set_conv5_kernel(coeffs, shift: int = 0) -> None:
        """Load the general 25-coeff 5x5 kernel (DoG B branch): coeffs row-major
        idx0=top-left..24=bot-right -> 0xFE20+i, shift -> 0xFE39. See axis_rgb_conv5x5.sv."""
        cs = list(coeffs)[:25] + [0] * max(0, 25 - len(coeffs))
        for i, c in enumerate(cs):
            fe_write(0x20 + i, int(c) & 0xFF)
        fe_write(0x39, int(shift) & 0xF)

    def set_dog_params(mode: int = 2, alpha: int = 1, beta: int = 1,
                       shift: int = 0, offset: int = 128) -> None:
        """Set the DoG combiner (0xFE40-44): out = clamp((alpha*A - beta*B) >> shift + offset).
        mode 0=A 1=B 2=DoG(aA-bB) 3=sum(aA+bB)."""
        fe_write(0x40, int(mode) & 0x3)
        fe_write(0x41, int(alpha) & 0xFF)
        fe_write(0x42, int(beta) & 0xFF)
        fe_write(0x43, int(shift) & 0xF)
        fe_write(0x44, int(offset) & 0xFF)

    def set_dog(small, small_shift, large, large_shift, alpha=1, beta=1,
                shift=0, offset=128, mode=2) -> None:
        """Load the full DoG dual-kernel and select it (op 12): A = 9-coeff 3x3 (small),
        B = 25-coeff 5x5 (large); out = clamp(alpha*A - beta*B + offset) per channel.
        set_proc_op(0) returns to the colour path."""
        set_conv_kernel(small, small_shift)
        set_conv5_kernel(large, large_shift)
        set_dog_params(mode, alpha, beta, shift, offset)
        set_proc_op(12)

    def set_dog_named(name: str) -> None:
        """Load a named DoG preset from DOG_PRESETS (blob/unsharp) and select op 12."""
        if name not in DOG_PRESETS:
            raise ValueError(f'unknown DoG preset {name!r}; choices: {sorted(DOG_PRESETS)}')
        set_dog(*DOG_PRESETS[name])

    def set_sep_kernel(stage: int, h, v, hshift: int = 0, vshift: int = 0) -> None:
        """Load a cascade separable-5x5 stage kernel: stage 2 -> 0xFE50.., stage 3 -> 0xFE60..
        (h[5] + hshift, v[5] + vshift). See axis_rgb_conv5x5_sep.sv."""
        base = 0x50 if int(stage) == 2 else 0x60
        hh = list(h)[:5] + [0]*max(0, 5-len(h))
        vv = list(v)[:5] + [0]*max(0, 5-len(v))
        for i in range(5): fe_write(base + i, int(hh[i]) & 0xFF)
        fe_write(base + 5, int(hshift) & 0xF)
        for i in range(5): fe_write(base + 6 + i, int(vv[i]) & 0xFF)
        fe_write(base + 0xB, int(vshift) & 0xF)

    def set_blur(size: int = 13) -> None:
        """Runtime-variable Gaussian blur via the cascade. size in {5,9,13} selects op
        13/14/15 = effective kernel 5x5 / 9x9 / 13x13. Loads Gaussians into the active
        stages (S1 general 5x5, S2/S3 separable) and bypasses the rest (identity)."""
        set_conv5_kernel(GAUSS5[0], GAUSS5[1])                       # S1 = 5x5 Gaussian
        if size >= 9:  set_sep_kernel(2, GAUSS1D, GAUSS1D, 4, 4)
        else:          set_sep_kernel(2, SEP_IDENTITY, SEP_IDENTITY, 0, 0)
        if size >= 13: set_sep_kernel(3, GAUSS1D, GAUSS1D, 4, 4)
        else:          set_sep_kernel(3, SEP_IDENTITY, SEP_IDENTITY, 0, 0)
        set_proc_op(13 if size < 9 else (14 if size < 13 else 15))

    def set_edges(shift: int = 1, csh: int = 0) -> None:
        """Omnidirectional Sobel edge MAGNITUDE = |Gx| + |Gy| (op 12): A = conv3x3 Sobel-X
        with |.| (cfg_abs), B = conv5x5 Sobel-Y with |.|, combiner sum mode -> edges in ALL
        directions (bright on black). The abs (0xFE45) recovers BOTH gradient polarities --
        a single 3x3 Sobel only shows one edge side. `shift` scales each gradient (per-conv,
        1 = /2), `csh` scales the |A|+|B| sum (combiner). Higher = dimmer/less sensitive.
        Needs the edge-magnitude bitstream (conv cfg_abs). set_proc_op(0) exits."""
        sx = [-1, 0, 1, -2, 0, 2, -1, 0, 1]            # Sobel-X (3x3, branch A)
        sy = [-1, -2, -1, 0, 0, 0, 1, 2, 1]            # Sobel-Y (3x3 -> 5x5 centre, branch B)
        s5 = [0] * 25
        for r in range(3):
            for c in range(3):
                s5[(r + 1) * 5 + (c + 1)] = sy[r * 3 + c]
        set_conv_kernel(sx, shift)                     # A = 3x3 Sobel-X
        set_conv5_kernel(s5, shift)                    # B = 5x5 Sobel-Y
        set_dog_params(mode=3, alpha=1, beta=1, shift=csh, offset=0)  # sum = (|A|+|B|) >> csh
        fe_write(0x45, 0x3)                            # |grad| on A and B (0xFE45 bit0/bit1)
        set_proc_op(12)

    def set_pre_op(op: int) -> None:
        """PRE stage (axis_rgb_prefilter) select, applied BEFORE the conv stage while in conv
        mode (proc_op >= 8). 0=passthrough 1=invert 2=gray 3=BGR 4=threshold 5/6/7=R/G/B
        (point ops on the centre), 8=GAUSSIAN 3x3 blur, 9=MEDIAN 3x3 (spatial denoise on the
        window). Loaded on 0xFE46 (4-bit). e.g. set_pre_op(9)+set_edges() = median denoise ->
        Sobel; set_pre_op(4)+set_edges() = binarize -> Sobel."""
        fe_write(0x46, int(op) & 0xF)

    def set_pre_thresh(level: int) -> None:
        """Threshold level (on green) for the PRE-conv slot op 4, 0..255. 0xFE47 (default 128)."""
        fe_write(0x47, int(level) & 0xFF)

    def set_post_op(op: int) -> None:
        """Point op applied AFTER the conv/mux stage (second slot). Same op codes as
        set_pre_op. Loaded on 0xFE48. e.g. set_edges()+set_post_op(4) = Sobel -> binarize
        (a binary edge map)."""
        fe_write(0x48, int(op) & 0x7)

    def set_post_thresh(level: int) -> None:
        """Threshold level for the POST-conv slot op 4, 0..255. 0xFE49 (default 128). Edge-
        magnitude dynamic range differs from raw luma, so a lower level (~64) is typical."""
        fe_write(0x49, int(level) & 0xFF)

    def set_dither(enable: bool = True, mode: str = 'ordered', bits: int = 1) -> None:
        """Final DITHER stage (AFTER post, 0xFE4A): quantize each channel to `bits` with a
        position-dependent bias so gradients dither instead of band. mode 'ordered' (Bayer 4x4)
        or 'random' (LFSR). bits 1=halftone(0/255) .. 6=anti-banding. enable=False -> off
        (bit-identical). e.g. set_proc_op(2)+set_dither(bits=1) = gray -> halftone."""
        m = 1 if str(mode).lower().startswith('rand') else 0
        ctrl = ((1 if enable else 0) | (m << 1) | ((int(bits) & 0x7) << 2)) & 0xFF
        fe_write(0x4A, ctrl)

    def sccb_read(addr: int, t: float = 1.0):
        # Runtime SCCB read protocol (RTL since commit 82ab667):
        #   ch1 data: bits[15:0]=addr, bit[26]=READ_APPLY (rising edge triggers)
        #   ch2 status: bit[5]=read_done, bit[6]=read_error, bits[15:8]=read_data
        sccb = ol.sccb_gpio
        a = int(addr) & 0xFFFF
        wait_sccb_idle()
        sccb.write(0x00, a)
        sccb.write(0x00, a | SCCB_READ_APPLY)
        sccb.write(0x00, a)
        deadline = time.monotonic() + t
        while time.monotonic() < deadline:
            st = sccb.read(SCCB_STATUS_OFFSET)
            if st & (1 << 5):
                err = bool(st & (1 << 6))
                return None if err else ((st >> 8) & 0xFF)
            time.sleep(0.002)
        return None

    def read_dbg(p: int) -> int:
        ol.dbg_gpio.channel2.write(int(p), 0xFFFFFFFF)
        time.sleep(0.0001)
        return ol.dbg_gpio.channel1.read()

    # Current IDELAY taps: data lane0/lane1 [4:0]/[12:8] + clock lane [20:16].
    # Tracked so clk_idelay_set / idelay_set can update one field without
    # clobbering the others (the GPIO write carries all three in one word).
    idelay_cur = {'t0': 8, 't1': 8, 'tclk': 0, 'blank': 0, 'proc_op': 0}

    def _idelay_write() -> None:
        g = ol.idelay_gpio
        word = ((idelay_cur['blank'] & 0xF) << 27) \
            | ((idelay_cur['proc_op'] & 0xF) << 21) \
            | ((idelay_cur['tclk'] & 0x1F) << CLK_IDELAY_SHIFT) \
            | ((idelay_cur['t1'] & 0x1F) << 8) | (idelay_cur['t0'] & 0x1F)
        g.write(GPIO_REQ_TRI, 0)
        g.write(GPIO_REQ, word)
        g.write(GPIO_REQ, word | APPLY_BIT)
        g.write(GPIO_REQ, word)
        time.sleep(0.03)

    def idelay_set(t0: int, t1: int | None = None, tclk: int | None = None) -> None:
        idelay_cur['t0'] = int(t0) & 0x1F
        idelay_cur['t1'] = idelay_cur['t0'] if t1 is None else (int(t1) & 0x1F)
        if tclk is not None:
            idelay_cur['tclk'] = int(tclk) & 0x1F
        _idelay_write()

    def clk_idelay_set(tclk: int) -> None:
        """Set the clock-lane IDELAY tap (byte-phase cal), keeping data taps."""
        idelay_cur['tclk'] = int(tclk) & 0x1F
        _idelay_write()

    def set_long_as_line(on: bool) -> None:
        """Drive cfg_long_as_line on idelay GPIO bit[26] WITHOUT pulsing apply (the
        RTL reads it direct; the locked data taps, loaded only on apply, are kept).
        Call AFTER lock (idelay_set/lock_mode write the idelay word with apply and
        would clear bit[26])."""
        g = ol.idelay_gpio
        word = ((1 << 26) if on else 0) \
            | ((idelay_cur['t1'] & 0x1F) << 8) | (idelay_cur['t0'] & 0x1F)
        g.write(GPIO_REQ_TRI, 0)
        g.write(GPIO_REQ, word)   # no APPLY edge -> data taps preserved
        time.sleep(0.001)

    def set_settle_blank(k: int) -> None:
        """Drive cfg_settle_blank_k on idelay GPIO [30:27] (byte-domain per-line
        settle blank: hold the SoT window closed K byte_clk after LP-exit). Read
        direct by the RTL (no apply), and persisted in idelay_cur so later
        idelay_set calls keep it. k=0 = off."""
        idelay_cur['blank'] = int(k) & 0xF
        g = ol.idelay_gpio
        word = ((idelay_cur['blank'] & 0xF) << 27) \
            | ((idelay_cur['proc_op'] & 0xF) << 21) \
            | ((idelay_cur['tclk'] & 0x1F) << CLK_IDELAY_SHIFT) \
            | ((idelay_cur['t1'] & 0x1F) << 8) | (idelay_cur['t0'] & 0x1F)
        g.write(GPIO_REQ_TRI, 0)
        g.write(GPIO_REQ, word)   # no APPLY edge -> data taps preserved
        time.sleep(0.001)

    def set_proc_op(op: int) -> None:
        """Processing-slot select on idelay GPIO [24:21] (direct combinational read, no
        apply; persisted in idelay_cur like settle_blank). 0-7 = point ops (0=passthrough
        1=invert 2=grayscale 3=BGR-swap 4=threshold 5=R-only 6=G-only 7=B-only); 8 = single
        3x3 conv with the loaded kernel; 12 = DoG dual-kernel; 13/14/15 = cascade blur tap
        t1/t2/t3 (eff 5x5/9x9/13x13). See docs/doc/image_processing_slot_spec_20260625.md."""
        idelay_cur['proc_op'] = int(op) & 0xF
        g = ol.idelay_gpio
        word = ((idelay_cur['blank'] & 0xF) << 27) \
            | ((idelay_cur['proc_op'] & 0xF) << 21) \
            | ((idelay_cur['tclk'] & 0x1F) << CLK_IDELAY_SHIFT) \
            | ((idelay_cur['t1'] & 0x1F) << 8) | (idelay_cur['t0'] & 0x1F)
        g.write(GPIO_REQ_TRI, 0)
        g.write(GPIO_REQ, word)   # no APPLY edge -> data taps preserved
        time.sleep(0.001)

    def btrace_sel(idx: int, freeze: bool = True, long_as_line: bool = False) -> None:
        """Boundary packet trace (page 0x3F) read control: set the 5-bit read
        index on idelay GPIO [20:16] and the freeze bit [25], WITHOUT pulsing
        apply (bit24) -- the RTL reads these direct from the synced GPIO word, so
        the locked data taps (loaded only on apply) are untouched. The dead
        clk-IDELAY field is reused, so no new GPIO. NOTE: page 0x3F is read via
        read_dbg(0x9F) (control form 0x80|(0x3F&0x1F) for pages >= 0x20).
        long_as_line keeps bit[26] set so the disposition counters can be read
        WHILE cfg_long_as_line is active (else this write would clear it)."""
        g = ol.idelay_gpio
        word = ((1 << 26) if long_as_line else 0) \
            | ((1 << 25) if freeze else 0) \
            | ((int(idx) & 0x1F) << CLK_IDELAY_SHIFT) \
            | ((idelay_cur['t1'] & 0x1F) << 8) | (idelay_cur['t0'] & 0x1F)
        g.write(GPIO_REQ_TRI, 0)
        g.write(GPIO_REQ, word)   # no APPLY edge -> data taps preserved
        time.sleep(0.0005)

    # bitslip word: [2:0]=p0 [10:8]=p1 [16]=lane1_sweep [23:17]=cfg_clk_settle_cyc
    # (supervisor clock-lane settle, level-read) [24]=apply [25]=cfg_hw_lock (HW
    # deterministic-lock FSM enable, level-read) [26]=hwlock_inhibit (force the FSM
    # OFF in a HWLOCK_DEFAULT_ON bitstream -> software lock_mode fallback). Track so
    # set_clk_settle / set_hw_lock / bitslip_set don't clobber each other.
    bitslip_cur = {'p0': 0, 'p1': 6, 'settle': 0, 'hwlock': 0, 'inhibit': 0}

    def _bitslip_word(apply: bool) -> int:
        return ((1 << 24) if apply else 0) \
            | ((1 << 25) if bitslip_cur['hwlock'] else 0) \
            | ((1 << 26) if bitslip_cur['inhibit'] else 0) \
            | ((bitslip_cur['settle'] & 0x7F) << 17) \
            | ((bitslip_cur['p1'] & 0x7) << 8) | (bitslip_cur['p0'] & 0x7)

    def bitslip_set(p0: int, p1: int) -> None:
        bitslip_cur['p0'] = p0 & 0x7
        bitslip_cur['p1'] = p1 & 0x7
        g = ol.bitslip_gpio
        g.channel1.write(_bitslip_word(True), 0xFFFFFFFF)
        time.sleep(0.02)
        g.channel1.write(_bitslip_word(False), 0xFFFFFFFF)
        time.sleep(0.03)

    def set_clk_settle(cyc: int) -> None:
        """Set the supervisor clock-lane settle count (bitslip word [23:17], level-
        read by the RTL; governs when byte_clk starts after a clock-lane restart =
        the gated FS-recovery knob). 0 = build-time default. No apply edge."""
        bitslip_cur['settle'] = int(cyc) & 0x7F
        ol.bitslip_gpio.channel1.write(_bitslip_word(False), 0xFFFFFFFF)
        time.sleep(0.01)

    def set_hw_lock(enable: bool) -> None:
        """Enable the HW deterministic-lock FSM (bitslip word [25], level-read by
        the RTL). When set, the RTL sweeps the 8x8 bitslip + /4-phase re-roll and
        holds a clean lock with NO software lock_mode -- the bitslip target is then
        driven by the FSM (the GPIO bitslip_set value is ignored). =0 -> GPIO/
        lock_mode path (fallback). Read FSM status on debug page 0x2e (ctrl 0x8E).
        Continuous (0x14) only. Set at/after init; the FSM auto-locks on its own. In
        a HWLOCK_DEFAULT_ON bitstream the FSM is already on at boot; enable=False then
        drives bit[26] to INHIBIT it for the software lock_mode fallback."""
        bitslip_cur['hwlock'] = 1 if enable else 0
        bitslip_cur['inhibit'] = 0 if enable else 1
        ol.bitslip_gpio.channel1.write(_bitslip_word(False), 0xFFFFFFFF)
        time.sleep(0.01)

    def read_hwlock() -> dict:
        """Decode debug page 0x2e (ctrl 0x8E): the HW lock FSM status word.
        {failed[31], locked[30], state[29:27], reroll[26:23], combo[22:17],
         p0[16:14], p1[13:11], hdr_active[10]}."""
        w = read_dbg(0x8E)
        st = (w >> 27) & 0x7
        names = {0: 'IDLE', 1: 'SWEEP', 2: 'REROLL', 3: 'HOLD', 4: 'FAILED'}
        return dict(raw=w, failed=(w >> 31) & 1, locked=(w >> 30) & 1,
                    state=st, state_name=names.get(st, f'?{st}'),
                    reroll=(w >> 23) & 0xF, combo=(w >> 17) & 0x3F,
                    p0=(w >> 14) & 0x7, p1=(w >> 11) & 0x7, hdr_active=(w >> 10) & 1)

    def frame_lines_write_raw(word: int) -> None:
        g = ol.frame_lines_gpio
        g.channel1.write(word, 0xFFFFFFFF)
        time.sleep(0.005)
        g.channel1.write(word | APPLY_BIT, 0xFFFFFFFF)
        time.sleep(0.005)
        g.channel1.write(word, 0xFFFFFFFF)
        time.sleep(0.005)

    def cam_resetb_pulse(low_ms: int = 5, post_release_wait_ms: int = 30) -> None:
        print(f'  RESETB pulse: high → low ({low_ms}ms) → high (wait {post_release_wait_ms}ms)')
        frame_lines_write_raw(CAM_GPIO_BIT)
        time.sleep(0.005)
        frame_lines_write_raw(0)
        time.sleep(low_ms / 1000.0)
        frame_lines_write_raw(CAM_GPIO_BIT)
        time.sleep(post_release_wait_ms / 1000.0)

    # Last frame_lines base word (without the BUFR.CLR re-roll bit), so
    # bufr_clr_pulse can toggle bit[27] without disturbing the other config.
    fl_base = [CAM_GPIO_BIT]

    def frame_lines_set_keep_cam(value: int = 480, use_lsle: bool = False,
                                 expected_dt: int = 0x00, use_tpg: bool = False,
                                 sup_enable: bool = False,
                                 sof_synth: bool = False,
                                 force_expected: bool = False,
                                 hs_settle_gate: bool = False) -> None:
        # sup_enable (bit29) opts the D-PHY lane supervisor in. sof_synth (bit30)
        # opens a frame from the first LS when the chip's FS never arrives
        # (supervisor enabled, fs=0). force_expected (bit31) force-closes the frame
        # at exactly `value` lines for a constant-height VTC/genlock stream (live-
        # HDMI roll fix). hs_settle_gate (bit28) applies the per-line HS-SETTLE SoT
        # gate in the legacy continuous path (decoupled from sup_enable; recovers
        # the >=16 line/frame frontend drop, 2026-06-17). All default off = legacy
        # frontend behaviour (A/B on the same bitstream).
        base = (CAM_GPIO_BIT
                | (int(value) & 0xFFFF)
                | ((1 << 16) if use_lsle else 0)
                | ((int(expected_dt) & 0x7F) << 17)
                | (TPG_RT_BIT if use_tpg else 0)
                | (HS_SETTLE_GATE_BIT if hs_settle_gate else 0)
                | (SUP_ENABLE_BIT if sup_enable else 0)
                | (SOF_SYNTH_BIT if sof_synth else 0)
                | (FORCE_EXPECTED_BIT if force_expected else 0))
        fl_base[0] = base
        frame_lines_write_raw(base)

    def bufr_clr_pulse(hold_ms: float = 2.0) -> None:
        """Re-roll the BUFR /4 byte phase (byte-phase cal): assert bit[27] to
        hold the divider in reset, then release so it restarts on a fresh phase.
        Direct level write (bit[27] is read directly in RTL, not apply-gated).
        Preserves the current frame_lines config (cam_gpio/sup/synth/value)."""
        g = ol.frame_lines_gpio
        g.channel1.write(fl_base[0] | BUFR_CLR_BIT, 0xFFFFFFFF)
        time.sleep(hold_ms / 1000.0)
        g.channel1.write(fl_base[0], 0xFFFFFFFF)
        time.sleep(0.005)

    def snap() -> dict:
        p02 = read_dbg(0x02)
        p03 = read_dbg(0x03)
        p07 = read_dbg(0x07)
        p18 = read_dbg(0x18)
        p19 = read_dbg(0x19)
        return dict(
            crc_ok=(p02 >> 16) & 0xFFFF,
            crc_err=p02 & 0xFFFF,
            short_pkt=(p03 >> 16) & 0xFFFF,
            long_pkt=p03 & 0xFFFF,
            drop_dt=(p07 >> 16) & 0xFFFF,
            drop_vc=p07 & 0xFFFF,
            fs=(p18 >> 16) & 0xFFFF,
            fe=p18 & 0xFFFF,
            ls=(p19 >> 16) & 0xFFFF,
            le=p19 & 0xFFFF,
        )

    return dict(
        wait_sccb_idle=wait_sccb_idle,
        sccb_write=sccb_write,
        sccb_read=sccb_read,
        read_dbg=read_dbg,
        idelay_set=idelay_set,
        clk_idelay_set=clk_idelay_set,
        set_long_as_line=set_long_as_line,
        set_settle_blank=set_settle_blank,
        set_proc_op=set_proc_op,
        set_conv_coeff=set_conv_coeff,
        set_conv_kernel=set_conv_kernel,
        set_conv_named=set_conv_named,
        fe_write=fe_write,
        set_conv5_kernel=set_conv5_kernel,
        set_dog_params=set_dog_params,
        set_dog=set_dog,
        set_dog_named=set_dog_named,
        set_sep_kernel=set_sep_kernel,
        set_blur=set_blur,
        set_edges=set_edges,
        set_pre_op=set_pre_op,
        set_pre_thresh=set_pre_thresh,
        set_post_op=set_post_op,
        set_post_thresh=set_post_thresh,
        set_dither=set_dither,
        set_clk_settle=set_clk_settle,
        set_hw_lock=set_hw_lock,
        read_hwlock=read_hwlock,
        btrace_sel=btrace_sel,
        bitslip_set=bitslip_set,
        bufr_clr_pulse=bufr_clr_pulse,
        frame_lines_write_raw=frame_lines_write_raw,
        cam_resetb_pulse=cam_resetb_pulse,
        frame_lines_set_keep_cam=frame_lines_set_keep_cam,
        snap=snap,
    )


def chip_init(h: dict, init_steps: list, label: str, settle_s: float = 5.0) -> int:
    """HW reset + 0x3008 reset/power-down + iterate init_steps (with optional
    'STREAM_ON' sentinel) + settle. Returns NACK count."""
    print('Step 0: cam_gpio=1 (chip running)')
    h['frame_lines_write_raw'](CAM_GPIO_BIT)
    time.sleep(0.5)

    print('Step 1: HW RESETB pulse')
    h['cam_resetb_pulse'](low_ms=5, post_release_wait_ms=30)

    print('Step 2: 0x3008=0x82 (SW reset)')
    h['sccb_write'](0x3008, 0x82)
    time.sleep(0.020)
    print('Step 3: 0x3008=0x42 (power down)')
    h['sccb_write'](0x3008, 0x42)
    time.sleep(0.020)

    has_stream_on = 'STREAM_ON' in init_steps
    total = sum(1 for e in init_steps if e != 'STREAM_ON')
    suffix = ' + stream_on' if has_stream_on else ' (then explicit stream_on)'
    print(f'Step 4: {label} init ({total} writes{suffix})')
    nacks = 0
    for entry in init_steps:
        if entry == 'STREAM_ON':
            print('  >>> 0x3008=0x02 stream on')
            ok = h['sccb_write'](0x3008, 0x02)
            if not ok: nacks += 1
            time.sleep(0.300)
        else:
            addr, val = entry
            ok = h['sccb_write'](addr, val)
            if not ok:
                nacks += 1
    if not has_stream_on:
        print('  >>> 0x3008=0x02 stream on (appended)')
        ok = h['sccb_write'](0x3008, 0x02)
        if not ok: nacks += 1
        time.sleep(0.300)
    n_trans = total + 1
    print(f'Step 4: completed, {nacks} NACKs out of {n_trans} transactions')

    print(f'Step 5: wait {settle_s:.1f}s for pipeline settling')
    time.sleep(settle_s)
    return nacks


def find_best_bitslip(h: dict, p0_hint: int | None, p1_hint: int | None) -> tuple[int, int]:
    """If hint provided, use that. Else 8x8 sweep, return best by long_pkt then short_pkt."""
    if p0_hint is not None and p1_hint is not None:
        print(f'Using BITSLIP hint: ({p0_hint},{p1_hint})')
        h['bitslip_set'](p0_hint, p1_hint)
        time.sleep(0.3)
        return p0_hint, p1_hint

    print('BITSLIP 8x8 sweep:')
    print(f'{"p0":>2} {"p1":>2}  {"long":>6} {"short":>6} {"fs":>3} {"fe":>3}')
    best = (-1, -1, 0, 0)  # (long, short, p0, p1)
    for p0 in range(8):
        for p1 in range(8):
            h['bitslip_set'](p0, p1)
            time.sleep(0.05)
            b = h['snap']()
            time.sleep(0.3)
            a = h['snap']()
            d = {k: (a[k] - b[k]) % 65536 for k in b}
            if d['long_pkt'] > 0 or d['short_pkt'] > 0 or d['fs'] > 0 or d['fe'] > 0:
                print(f'{p0:>2} {p1:>2}  {d["long_pkt"]:>6} {d["short_pkt"]:>6} {d["fs"]:>3} {d["fe"]:>3}')
                score = (d['long_pkt'], d['short_pkt'])
                if score > (best[0], best[1]):
                    best = (d['long_pkt'], d['short_pkt'], p0, p1)
    if best[0] < 0:
        print('No activity in sweep.')
        return 0, 0
    print(f'Best: BITSLIP=({best[2]},{best[3]}) long={best[0]} short={best[1]}')
    h['bitslip_set'](best[2], best[3])
    time.sleep(0.3)
    return best[2], best[3]


def idelay_sweep(h: dict, p0: int, p1: int,
                 taps: List[int], window_s: float = 1.0) -> List[tuple]:
    """For a fixed BITSLIP pair, walk IDELAY taps and snap diff each.

    Returns list of (tap, metric_diff_dict)."""
    h['bitslip_set'](p0, p1)
    time.sleep(0.3)
    print(f'\nIDELAY sweep @ BITSLIP=({p0},{p1}), taps={taps}, window={window_s:.2f}s')
    print(f'{"tap":>3} {"long":>6} {"short":>6} {"fs":>3} {"fe":>3} '
          f'{"ls":>4} {"le":>4} {"drop_dt":>7} {"drop_vc":>7} '
          f'{"crc_ok":>6} {"crc_err":>7}')
    results: List[tuple] = []
    for tap in taps:
        h['idelay_set'](tap, tap)
        time.sleep(0.3)
        b = h['snap']()
        time.sleep(window_s)
        a = h['snap']()
        d = {k: (a[k] - b[k]) % 65536 for k in b}
        print(f'{tap:>3} {d["long_pkt"]:>6} {d["short_pkt"]:>6} '
              f'{d["fs"]:>3} {d["fe"]:>3} {d["ls"]:>4} {d["le"]:>4} '
              f'{d["drop_dt"]:>7} {d["drop_vc"]:>7} '
              f'{d["crc_ok"]:>6} {d["crc_err"]:>7}')
        results.append((tap, d))
    return results


def report_branch(sweep_results: List[tuple], nacks: int) -> None:
    """Print a single-line diagnosis based on the sweep matrix and NACK count.

    sweep_results is a flat list of (label, tap, diff_dict) across all BITSLIP runs.
    判定順は **chip 出力 → FPGA 受信** で固定:
      1. NACK            (chip SCCB / RESETB / 接触: chip まで届かない)
      2. short_pkt も低い (chip PLL / stream-on / 物理層: chip が出していない)
      3. short_pkt のみ  (DT/WC/format 不整合: chip 出してるが契約ずれ)
      4. long_pkt > 0    (FPGA 受信 sampling OK: ここで初めて FPGA 側成功宣言)
    """
    max_long = max((d['long_pkt'] for _, _, d in sweep_results), default=0)
    max_short = max((d['short_pkt'] for _, _, d in sweep_results), default=0)
    if nacks > 0:
        print(f'\n[BRANCH:chip-SCCB] nacks>0 ({nacks}) — '
              'SCCB/RESETB/contact 経路を疑う。 D-PHY 調整に進まず、 '
              'cam_resetb_pulse / pin 接触に戻る')
        return
    if max_short <= 4:
        print(f'\n[BRANCH:chip-output] short_pkt も低い (max={max_short}) — '
              'chip が出していない疑い。 OV5640 PLL / stream-on / 物理層: '
              'chip MIPI control (0x4800), RESETB pulse 再試行, clock 経路 / 接触 へ')
        return
    if max_long == 0:
        print(f'\n[BRANCH:chip-format] short_pkt は出る (max={max_short}) が long_pkt=0 — '
              'chip 出力はあるが DT/WC/format 契約不整合。 0x4300, 0x501f, '
              'expected_dt=0x22, RTL FORMAT_EXPECTED_DT の整合確認へ')
        return
    best = max(sweep_results, key=lambda x: x[2]['long_pkt'])
    label, tap, d = best
    print(f'\n[BRANCH:FPGA-rx-ok] long_pkt>0 観測 (best: {label} tap={tap} '
          f'long={d["long_pkt"]} short={d["short_pkt"]}) — '
          'chip 出力健全 + FPGA 受信 sampling 決まり。 '
          'best tap/BITSLIP 固定で VDMA dump を画像化する')


PAGE0_FLAGS = [
    (31, 'phy_probe_alive'),
    (30, 'phy_hs_clk_seen'),
    (29, 'phy_lane_sot_seen'),
    (28, 'phy_stream_byte_seen'),
    (27, 'phy_sync_header_seen'),
    (26, 'setup_ready'),
    (25, 'ov5640_chip_id_ok'),
    (24, 'sccb_done'),
    (23, 'expected_long_seen'),
    (22, 'crc_ok_seen'),
    (21, 'crc_err_seen'),
    (20, 'yuv_pixel_seen'),
    (19, 'hdmi_axis_sof_toggle'),
    (18, 'hdmi_axis_take_seen'),
    (17, 'hdmi_underflow_seen'),
    (16, 'hdmi_axis_error_seen'),
]


def read_diagnostic_pages(h: dict) -> None:
    pages = {
        0x00: 'status_flags',
        0x01: 'last_pkt_di_wc',
        0x02: 'crc_ok_err',
        0x03: 'short_long_cnt',
        0x04: 'pkt_trunc_ecc_uncorr',
        0x05: 'last_frame_lines_pix_per_line',
        0x07: 'drop_dt_vc',
        0x0F: 'lane0_trace_bytes',
        0x10: 'lane1_trace_bytes',
        0x18: 'fs_fe_cnt',
        0x19: 'ls_le_cnt',
        0x1a: 'last_short_di_wc',
        0x1b: 'live_lines_last_fe',
        0x1c: 'fe_before480_after480',
        0x1d: 'fs_overlap_fe_no_fs',
        0x1e: 'other_short_long_pre_fs',
        0x1f: 'frame_line_cnt_lo',
    }
    print('-- diagnostic pages')
    for page, name in pages.items():
        w = h['read_dbg'](page)
        print(f'  page 0x{page:02x} {name:20s} = 0x{w:08x}')

    p00 = h['read_dbg'](0x00)
    p01 = h['read_dbg'](0x01)
    p03 = h['read_dbg'](0x03)
    p05 = h['read_dbg'](0x05)
    p07 = h['read_dbg'](0x07)
    p0f = h['read_dbg'](0x0F)
    p10 = h['read_dbg'](0x10)
    p18 = h['read_dbg'](0x18)
    p1a = h['read_dbg'](0x1a)
    p1c = h['read_dbg'](0x1c)
    p1d = h['read_dbg'](0x1d)
    p1e = h['read_dbg'](0x1e)
    print('-- decoded')
    flag_state = ', '.join(f'{name}={(p00 >> bit) & 1}' for bit, name in PAGE0_FLAGS)
    print(f'  page0 flags: {flag_state}')
    print(f'  last_pkt_di (chip-side DT) = 0x{(p01 >> 16) & 0xFF:02X}   (expect 0x22 for RGB565)')
    print(f'  last_pkt_wc                = {p01 & 0xFFFF}')
    print(f'  short_pkt_cnt              = {(p03 >> 16) & 0xFFFF}')
    print(f'  long_pkt_cnt               = {p03 & 0xFFFF}   ← key indicator')
    print(f'  last_frame_lines           = {(p05 >> 16) & 0xFFFF}   (chip-driven LE count, expect 480)')
    print(f'  video_pixel_per_line       = {p05 & 0xFFFF}   (FPGA unpacker count, expect 640)')
    print(f'  drop_dt_cnt                = {(p07 >> 16) & 0xFFFF}   ← packets dropped by DT filter')
    print(f'  drop_vc_cnt                = {p07 & 0xFFFF}')
    print(f'  FS cnt                     = {(p18 >> 16) & 0xFFFF}')
    print(f'  FE cnt                     = {p18 & 0xFFFF}')
    print(f'  last_short_di              = 0x{(p1a >> 16) & 0xFF:02X}   (FS/FE DT, expect 0x00 or 0x01)')
    print(f'  fe_before_480_cnt          = {(p1c >> 16) & 0xFFFF}   ← chip FE arrived with <480 lines')
    print(f'  fe_after_480_cnt           = {p1c & 0xFFFF}            ← chip FE arrived with >480 lines')
    print(f'  fs_overlap_cnt             = {(p1d >> 16) & 0xFFFF}   ← FS during active frame (protocol err)')
    print(f'  fe_without_fs_cnt          = {p1d & 0xFFFF}            ← FE without preceding FS')
    print(f'  other_short_cnt            = {(p1e >> 16) & 0xFFFF}   ← unrecognized short pkts')
    print(f'  long_before_fs_cnt         = {p1e & 0xFFFF}            ← long pkt before any FS')
    lane0 = [(p0f >> (8 * i)) & 0xFF for i in range(4)]
    lane1 = [(p10 >> (8 * i)) & 0xFF for i in range(4)]
    lane0_hits_b8 = any(b == 0xB8 for b in lane0)
    lane1_hits_b8 = any(b == 0xB8 for b in lane1)
    print(f'  lane0 trace [0..3]         = {[f"0x{b:02X}" for b in lane0]}'
          f'   (0xB8 sync: {lane0_hits_b8})')
    print(f'  lane1 trace [0..3]         = {[f"0x{b:02X}" for b in lane1]}'
          f'   (0xB8 sync: {lane1_hits_b8})')


def sample_wc_histogram(h: dict, duration_s: float) -> None:
    """Tight-loop poll page 0x01 (last_pkt_di_wc) for `duration_s` seconds and
    report the (DT, WC) frequency table.

    Page 0x01 latches the most recent long packet's header. The chip sends
    long packets at ~14k/s (480 lines x 30 fps); MMIO reads run at ~10-30us
    each, so each poll captures a different packet most of the time, modulo
    the page latch holding a value briefly between packets. The resulting
    histogram is a *biased* sample — frequent WC values dominate, rare ones
    may be missed — but it is sufficient to answer the question "does chip
    output have line-to-line WC variance?" since the alternative (constant
    1280) would yield a single histogram bin.

    Decision rule:
      - histogram has 1 (DT, WC) entry -> chip output is constant -> hypothesis E
        rejected; drift must come from elsewhere (CDC drop, downstream packer)
      - histogram has >1 distinct WC entries -> chip output varies -> hypothesis
        E confirmed; the yuv422_gray_unpack counter (LINE_PIXELS=640) drifts
        relative to actual chip line boundaries
    """
    print(f'\n=== WC histogram poll ({duration_s:.1f}s) ===')
    hist: dict[tuple[int, int], int] = {}
    deadline = time.monotonic() + duration_s
    polls = 0
    while time.monotonic() < deadline:
        p01 = h['read_dbg'](0x01)
        di = (p01 >> 16) & 0xFF
        wc = p01 & 0xFFFF
        key = (di, wc)
        hist[key] = hist.get(key, 0) + 1
        polls += 1
    if not hist:
        print('  (no samples — read_dbg returned nothing in the window)')
        return
    print(f'  polls={polls}  distinct (DT,WC)={len(hist)}')
    print(f'  {"DT":>4} {"WC":>6} {"count":>8} {"pct":>6}')
    ordered = sorted(hist.items(), key=lambda kv: -kv[1])
    for (di, wc), cnt in ordered:
        pct = 100.0 * cnt / polls
        print(f'  0x{di:02X} {wc:>6} {cnt:>8} {pct:>5.1f}%')
    if len(hist) == 1:
        print('  → single (DT,WC) -> chip output appears constant '
              '(hypothesis E rejected, look downstream)')
    else:
        wcs = sorted({wc for (_, wc), _ in hist.items()})
        spread = wcs[-1] - wcs[0]
        print(f'  → {len(hist)} distinct entries, WC spread = {spread} bytes '
              '(hypothesis E candidate — chip WC variance confirmed)')


def configure_vdma_s2mm(vdma: Any, bufs: List[Any],
                        start_mm2s: bool = False, start_s2mm: bool = True) -> None:
    """Configure VDMA S2MM and/or MM2S.

    Normal capture: start_s2mm=True writes chip data to DDR.
    HDMI live (with S2MM): start_mm2s=True, MM2S reads buf N-1 with FRMDLY_SHIFT
    while S2MM writes buf N. Output goes via axis_vdma32_to_y8 -> HDMI.
    MM2S-only sanity: start_s2mm=False, start_mm2s=True — DDR pre-fill content
    flows out to HDMI, validates MM2S/VTC/HDMI without chip.
    """
    print(f'-- VDMA: reset + program addrs (HSIZE={STRIDE}B, VSIZE={HEIGHT}, '
          f's2mm={start_s2mm}, mm2s={start_mm2s})')
    vdma.write(R.S2MM_VDMACR, RESET)
    while int(vdma.read(R.S2MM_VDMACR)) & RESET:
        time.sleep(0.001)
    vdma.write(R.MM2S_VDMACR, RESET)
    while int(vdma.read(R.MM2S_VDMACR)) & RESET:
        time.sleep(0.001)
    for idx, buf in enumerate(bufs):
        addr = int(buf.physical_address)
        if start_s2mm:
            vdma.write(R.S2MM_ADDR0 + idx * 4, addr)
        if start_mm2s:
            vdma.write(R.MM2S_ADDR0 + idx * 4, addr)
    if start_s2mm:
        vdma.write(R.S2MM_HSIZE, STRIDE)
        vdma.write(R.S2MM_STRIDE, STRIDE & 0xFFFF)
    if start_mm2s:
        vdma.write(R.MM2S_HSIZE, STRIDE)
        vdma.write(R.MM2S_STRIDE, (1 << FRMDLY_SHIFT) | (STRIDE & 0xFFFF))
    if start_s2mm:
        vdma.write(R.S2MM_VDMACR, RS | CIRCULAR_PARK)
        vdma.write(R.S2MM_VSIZE, HEIGHT)
    if start_mm2s:
        if start_s2mm:
            time.sleep(0.1)  # prime S2MM with at least one frame before MM2S reads
        vdma.write(R.MM2S_VDMACR, RS | CIRCULAR_PARK)
        vdma.write(R.MM2S_VSIZE, HEIGHT)
        print(f'  MM2S started (HDMI live readout enabled)')
    time.sleep(0.1)


def decode_vdmasr(value: int, label: str) -> None:
    flags = {
        'Halted':         (value >> 0) & 1,
        'Idle':           (value >> 1) & 1,
        'SGIncld':        (value >> 3) & 1,
        'DMAIntErr':      (value >> 4) & 1,
        'DMASlvErr':      (value >> 5) & 1,
        'DMADecErr':      (value >> 6) & 1,
        'SGIntErr':       (value >> 8) & 1,
        'SGSlvErr':       (value >> 9) & 1,
        'SGDecErr':       (value >> 10) & 1,
        'EOLEarlyErr':    (value >> 12) & 1,
        'EOLLateErr':     (value >> 13) & 1,
        'SOFEarlyErr':    (value >> 14) & 1,
        'SOFLateErr':     (value >> 15) & 1,
    }
    set_flags = [k for k, v in flags.items() if v]
    print(f'  {label} = 0x{value:08X}  set: {set_flags if set_flags else "(none)"}')


def stop_vdma(vdma: Any) -> None:
    """Halt both S2MM and MM2S engines so the PL stops accessing DDR. Without
    this, the AXI master would keep writing to the (now-freed) CMA buffer
    addresses after Python exits, eventually clobbering kernel memory and
    hanging sshd. See plan: VDMA cleanup root cause."""
    print('-- VDMA: stopping S2MM + MM2S (final cleanup)')
    decode_vdmasr(int(vdma.read(R.S2MM_VDMASR)), 'S2MM_VDMASR pre-stop ')
    vdma.write(R.S2MM_VDMACR, 0)
    vdma.write(R.MM2S_VDMACR, 0)
    time.sleep(0.05)
    vdma.write(R.S2MM_VDMACR, RESET)
    vdma.write(R.MM2S_VDMACR, RESET)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0:
        if not (int(vdma.read(R.S2MM_VDMACR)) & RESET):
            break
        time.sleep(0.005)
    decode_vdmasr(int(vdma.read(R.S2MM_VDMASR)), 'S2MM_VDMASR post-stop')


def install_vdma_cleanup_signals() -> None:
    """Install SIGTERM/SIGHUP/SIGINT handlers that raise SystemExit so the
    try/finally around setup_vdma_capture() runs stop_vdma() before exit.

    Why: Python's default SIGTERM/SIGHUP handlers terminate the interpreter
    without executing finally blocks. ssh disconnect (SIGHUP) and external
    kill (SIGTERM) therefore bypass cleanup, leaving VDMA writing to freed
    CMA -> kernel + sshd hang requiring a power cycle. Verified 2026-05-27:
    orchestration kill mid-hold required board reboot.
    """
    def _handler(signum, _frame):
        print(f'!!! caught signal {signum}; cleaning up VDMA before exit',
              file=sys.stderr)
        sys.exit(128 + signum)
    for sig_name in ('SIGTERM', 'SIGHUP', 'SIGINT'):
        sig = getattr(signal, sig_name, None)
        if sig is None:
            continue
        try:
            signal.signal(sig, _handler)
        except (OSError, ValueError):
            pass


def dump_buffers(bufs: List[Any], prefix: str) -> None:
    for idx, buf in enumerate(bufs):
        arr = np.asarray(buf).reshape(HEIGHT, STRIDE)
        crc = binascii.crc32(arr.tobytes()) & 0xFFFFFFFF
        nonzero = int(np.count_nonzero(arr))
        vmin, vmax = int(arr.min()), int(arr.max())
        mean = float(arr.mean())
        print(f'  buf{idx} addr=0x{buf.physical_address:08x} '
              f'crc32=0x{crc:08x} min={vmin} max={vmax} mean={mean:.2f} nonzero={nonzero}/{arr.size}')
        if prefix:
            raw_path = f'{prefix}_buf{idx}.raw'
            with open(raw_path, 'wb') as f:
                f.write(arr.tobytes())
            print(f'    -> wrote {raw_path}')


def main():
    install_vdma_cleanup_signals()
    ap = argparse.ArgumentParser()
    ap.add_argument('--bit', default=BIT_DEFAULT)
    ap.add_argument('--p0', type=int, default=None)
    ap.add_argument('--p1', type=int, default=None)
    ap.add_argument('--idelay', type=int, default=8)
    ap.add_argument('--expected-dt', type=lambda x: int(x, 0), default=0x1E,
                    help='FPGA expected long DT (frame_lines_gpio override). '
                         'Bitstream was built with EXPECTED_LONG_DT=0x1E (YUV422). '
                         'Use 0x22 for RGB565 only if also forcing chip 0x4300=0x6F.')
    ap.add_argument('--dump-prefix', default='')
    ap.add_argument('--capture-hold-s', type=float, default=3.0)
    ap.add_argument('--init-settle-s', type=float, default=20.0,
                    help='post-init time.sleep to let chip PLL / pixel pipeline ramp')
    ap.add_argument('--sweep', action='store_true',
                    help='run IDELAY sweep over BITSLIP=(0,6) and (6,4) instead of 8x8 BITSLIP sweep')
    ap.add_argument('--sweep-taps', type=str, default='4,6,8,10,12,14,16',
                    help='comma-separated IDELAY tap list for --sweep')
    ap.add_argument('--sweep-window-s', type=float, default=1.0,
                    help='per-tap snap diff window for --sweep')
    ap.add_argument('--enable-test-pattern', action='store_true',
                    help='after STREAM_ON, runtime write 0x503D=0x80 to force chip internal '
                         'color-bar pattern (bypasses sensor/exposure)')
    ap.add_argument('--chip-4300', type=lambda x: int(x, 0), default=0x30,
                    help='runtime override chip 0x4300 (FORMAT_CTRL) after init. '
                         '0x30=YUV422 (bitstream-trained default), 0x6F=RGB565. '
                         'full_init_steps.py contains 0x6F unconditionally — this overrides it.')
    ap.add_argument('--init-mode', choices=['full', 'minimal'], default='full',
                    help='full = 227-write FULL_INIT_STEPS extracted from RTL (default); '
                         'minimal = 63-write CFG_INIT (verbatim OV5640.h cfg_init_) like v57')
    ap.add_argument('--chip-4800', type=lambda x: int(x, 0), default=0x34,
                    help='runtime override chip 0x4800 (MIPI_CTRL_0) after init. '
                         'Default = 0x34 (bit5=continuous_clk + bit4=line_sync_enable + bit2=LP11_idle); '
                         'this is the value that 2026-05-28 实测 verified makes the chip emit LS/LE '
                         'short packets at ~500/s (without it, LE counter stays at 0 and '
                         'cfg_use_lsle=True rejects long packets, leaving VDMA buffer at 0xAA prefill). '
                         'Other values: 0x24=continuous_clk only (RTL bitstream-init default, no LS/LE), '
                         '0x14=line_sync only (no continuous_clk; verified 0/s LS/LE in 2026-05-28 sweep). '
                         'See diary 20260528 and memory project_ov5640_4800_34_enables_lsle. '
                         'Pass 0x24 to reproduce the bitstream-init-only state (i.e. no override).')
    ap.add_argument('--chip-4814', type=lambda x: int(x, 0), default=None,
                    help='runtime override chip 0x4814 (MIPI_CTRL_14) after init. '
                         'RTL init already writes 0x2A (LS_LE_EN). Use 0x2F to also enable bit0/1.')
    ap.add_argument('--wc-histogram-s', type=float, default=0.0,
                    help='poll page 0x01 in a tight loop for N seconds to '
                         'histogram chip-side (DT, WC) values. Use to test '
                         'hypothesis E (chip per-line WC variance) before VDMA capture. '
                         '0 = disabled (default); 5.0 = recommended.')
    ap.add_argument('--use-lsle', choices=['0', '1'], default='1',
                    help='cfg_use_lsle (frame_lines_gpio bit[16]). '
                         '1 (default, prior behavior): require LS short pkt before each '
                         'long pkt — strict but drops long if LS missing '
                         '(csi2_frame_state.sv:203-206). '
                         '0: long pkt itself opens line — lenient, recommended when '
                         'LS<<LE observed.')
    ap.add_argument('--mm2s-sanity', action='store_true',
                    help='Step 5 (plan): MM2S-only HDMI sanity. Skips chip init, '
                         'prefills DDR buffers with 8-bar vertical gradient (8 bars '
                         'x 80px wide, luminance 0..255 step), starts only MM2S engine '
                         '(S2MM halted). Verify HDMI monitor shows the 8-bar gradient '
                         'to confirm MM2S -> VTC -> HDMI path independently of chip. '
                         'Combine with --hold-seconds 30 for visual inspection.')
    ap.add_argument('--enable-hdmi', action='store_true',
                    help='also start MM2S engine so DDR buffers are read back '
                         'into the HDMI subsystem. Connect a monitor to the HDMI '
                         'TX port to view live output. Combine with --hold-seconds '
                         'to keep VDMA running for visual inspection.')
    ap.add_argument('--hold-seconds', type=float, default=0.0,
                    help='extra seconds to keep VDMA running after capture, '
                         'so live HDMI output can be observed before cleanup. '
                         'Use with --enable-hdmi.')
    ap.add_argument('--rgb565', action='store_true',
                    help='Post-init stream cycle to switch chip MIPI to RGB565 (DT=0x22). '
                         'Sequence: stream-off (0x300E=0x40, 0x4202=0x0F) -> '
                         '0x4300=0x6F + 0x501F=0x01 -> stream-on (0x300E=0x45, 0x4202=0x00). '
                         'Auto-sets --chip-4300=0x6F and --expected-dt=0x22 unless overridden.')
    ap.add_argument('--stream-cycle', action='store_true',
                    help='Post-init stream-off -> stream-on cycle WITHOUT format change. '
                         'Forces chip to re-evaluate frame timing. Auto-enabled by --rgb565.')
    ap.add_argument('--post-cycle-4814', type=lambda x: int(x, 0), default=None,
                    help='With --stream-cycle, write this value to 0x4814 between stream-off and stream-on. '
                         'Common values: 0x00 (chip default-like), 0x02 (LS/LE infra only), 0x22 (no FE suppress), '
                         '0x2A (Digilent, FE suppressed).')
    ap.add_argument('--vts', type=int, default=None,
                    help='Override chip VTS (0x380E/F) post-init to change frame rate. '
                         'Default init writes 1000. Try 500 to ~double fps.')
    ap.add_argument('--hts', type=int, default=None,
                    help='Override chip HTS (0x380C/D) post-init. Default 1600.')
    ap.add_argument('--short-exposure', action='store_true',
                    help='Post-init: switch to manual AEC, short exposure + boosted gain '
                         '(0x3503=0x07, 0x3500-2=16, 0x350B=0x40). Disables auto-exposure.')
    ap.add_argument('--pll-mult', type=lambda x: int(x, 0), default=None,
                    help='Post-init runtime PLL multiplier override (0x3036). '
                         'Default bitstream-trained = 0x36 (54). '
                         'Try 0x40 (64) for +37% chip rate / -41% CRC.')
    args = ap.parse_args()

    if args.rgb565:
        args.chip_4300 = 0x6F
        if args.expected_dt == 0x1E:
            args.expected_dt = 0x22
            print('--rgb565: auto-set chip_4300=0x6F and expected_dt=0x22')

    print(f'Loading overlay: {args.bit}')
    ol = Overlay(args.bit)
    # RACE WINDOW MINIMIZATION: AXI GPIO `frame_lines_gpio` has C_DOUT_DEFAULT=0,
    # so cam_gpio (RTL bit25 default=1) gets overwritten to 0 one clock after PL
    # config. Without intervention the chip stays in RESETB while the bitstream-init
    # SCCB FSM emits 232 NACKing writes, corrupting chip analog state. We re-assert
    # cam_gpio=1 immediately on top of the AXI GPIO data register before the FSM
    # has time to do meaningful damage.
    ol.frame_lines_gpio.channel1.write(CAM_GPIO_BIT, 0xFFFFFFFF)
    time.sleep(0.001)
    ol.frame_lines_gpio.channel1.write(CAM_GPIO_BIT | APPLY_BIT, 0xFFFFFFFF)
    time.sleep(0.001)
    ol.frame_lines_gpio.channel1.write(CAM_GPIO_BIT, 0xFFFFFFFF)
    print('Overlay loaded, cam_gpio=1 asserted immediately after Overlay()')
    print('Waiting 5s for bitstream-init SCCB FSM to drain (was 0.5s)')
    time.sleep(5.0)

    if args.mm2s_sanity:
        print('\n=== MM2S-only HDMI sanity (chip init skipped) ===')
        vdma_desc = ol.ip_dict['axi_vdma_0']
        vdma = MMIO(int(vdma_desc['phys_addr']), int(vdma_desc['addr_range']))
        bufs = [allocate(shape=(HEIGHT, STRIDE), dtype=np.uint8) for _ in range(N_FRAMES)]
        bar_width = WIDTH // 8
        for buf in bufs:
            arr = np.asarray(buf)
            for bar in range(8):
                x0 = bar * bar_width
                x1 = x0 + bar_width if bar < 7 else WIDTH
                arr[:, x0:x1] = bar * 255 // 7
        print(f'  prefilled {N_FRAMES} buffers with 8-bar vertical gradient '
              f'(bar width = {bar_width}px, luminance 0..255 in 8 steps)')
        try:
            configure_vdma_s2mm(vdma, bufs, start_mm2s=True, start_s2mm=False)
            hold = max(args.hold_seconds, 30.0)
            print(f'-- holding {hold:.1f}s for HDMI visual inspection')
            print('   Expect: 8 vertical bars (dark to light) on HDMI monitor.')
            print('   If bars are visible -> MM2S/VTC/HDMI path independently OK.')
            print('   If black/garbled  -> MM2S setup issue (ADDR/STRIDE/FRMDLY/VSIZE/PARK).')
            holds = max(1, int(hold / 5))
            for i in range(holds):
                time.sleep(min(5.0, hold - i * 5))
                mm2s_sr = int(vdma.read(R.MM2S_VDMASR))
                decode_vdmasr(mm2s_sr, f'  [{(i+1)*5:>3}s] MM2S_VDMASR')
            print('-- dumping prefill buffers (PNG for cross-check)')
            dump_buffers(bufs, args.dump_prefix)
        finally:
            stop_vdma(vdma)
            del bufs
            print('-- cleanup complete; sshd safe to remain up')
        return

    h = make_helpers(ol)
    if args.init_mode == 'minimal':
        # CFG_INIT verbatim の PLL は古い bitstream 用 (mult=56)。
        # 現 bitstream は mult=54 で D-PHY 訓練済なので post-CFG_INIT で override。
        pll_override = [
            (0x3034, 0x18),  # 8-bit MIPI mode
            (0x3035, 0x14),  # sys_div=1, mipi_div=4
            (0x3036, 0x36),  # mult=54 (bitstream-trained)
            (0x3037, 0x13),  # root_div=2, pre_div=2
            (0x3108, 0x01),  # PCLK/SCLK divider
            (0x4837, 0x18),  # PCLK period for mult=54
        ]
        init_steps = list(CFG_INIT) + pll_override
        init_label = f'minimal (CFG_INIT {len(CFG_INIT)}w + {len(pll_override)} PLL override)'
    else:
        init_steps = list(FULL_INIT_STEPS)
        init_label = 'full SCCB (FULL_INIT_STEPS, 227 writes)'
    nacks = chip_init(h, init_steps=init_steps, label=init_label,
                      settle_s=args.init_settle_s)

    print(f'Format match: 0x4300=0x{args.chip_4300:02X} '
          f'({"YUV422" if args.chip_4300 == 0x30 else "RGB565" if args.chip_4300 == 0x6F else "custom"})')
    ok = h['sccb_write'](0x4300, args.chip_4300)
    print(f'  SCCB result: {"OK" if ok else "NACK"}')
    time.sleep(0.5)

    if args.chip_4800 is not None:
        b54 = (args.chip_4800 >> 4) & 0x3
        emits = 'YES (LS/LE)' if b54 == 0x3 else 'NO (LE=0/s)'
        print(f'Runtime override: 0x4800=0x{args.chip_4800:02X} '
              f'(bit5+bit4=0b{b54:02b}, expected LS/LE emit: {emits})')
        # 2026-05-28 verified: bit5+bit4 (=0x34) both required for chip to emit
        # LS/LE per-line short packets. Stream cycle for clean apply.
        h['sccb_write'](0x300E, 0x40); time.sleep(0.05)
        ok = h['sccb_write'](0x4800, args.chip_4800)
        h['sccb_write'](0x300E, 0x45); time.sleep(0.5)
        print(f'  SCCB result: {"OK" if ok else "NACK"}')
        time.sleep(0.5)

    if args.chip_4814 is not None:
        print(f'Runtime override: 0x4814=0x{args.chip_4814:02X} (LS_LE_EN)')
        ok = h['sccb_write'](0x4814, args.chip_4814)
        print(f'  SCCB result: {"OK" if ok else "NACK"}')
        time.sleep(0.5)

    if args.enable_test_pattern:
        print('Runtime override: 0x503D=0x80 (test pattern, color bar)')
        ok = h['sccb_write'](0x503D, 0x80)
        print(f'  SCCB result: {"OK" if ok else "NACK"}')
        time.sleep(0.5)

    if args.short_exposure:
        print('Runtime override: short exposure + manual AEC (0x3503=0x07, 0x3500-2=16, 0x350B=0x40)')
        h['sccb_write'](0x3a00, 0x70)  # night mode OFF
        h['sccb_write'](0x3503, 0x07)
        h['sccb_write'](0x3500, 0x00)
        h['sccb_write'](0x3501, 0x00)
        h['sccb_write'](0x3502, 0x10)
        h['sccb_write'](0x350A, 0x00)
        h['sccb_write'](0x350B, 0x40)
        time.sleep(0.3)

    if args.vts is not None:
        print(f'Runtime override: VTS={args.vts} (0x380E/F)')
        h['sccb_write'](0x380E, (args.vts >> 8) & 0xFF)
        h['sccb_write'](0x380F, args.vts & 0xFF)
        time.sleep(0.3)

    if args.hts is not None:
        print(f'Runtime override: HTS={args.hts} (0x380C/D)')
        h['sccb_write'](0x380C, (args.hts >> 8) & 0xFF)
        h['sccb_write'](0x380D, args.hts & 0xFF)
        time.sleep(0.3)

    needs_cycle = args.rgb565 or args.stream_cycle or args.pll_mult is not None
    if needs_cycle:
        tags = []
        if args.rgb565: tags.append('RGB565')
        if args.pll_mult is not None: tags.append(f'PLL mult=0x{args.pll_mult:02X}')
        if args.stream_cycle and not args.rgb565 and args.pll_mult is None: tags.append('timing reset')
        mode_tag = ' + '.join(tags) if tags else 'cycle'
        print(f'{mode_tag}: stream-off -> [writes] -> stream-on')
        print('  stream off: 0x300E=0x40, 0x4202=0x0F')
        h['sccb_write'](0x300E, 0x40)
        h['sccb_write'](0x4202, 0x0F)
        time.sleep(0.1)
        if args.rgb565:
            print('  format: 0x4300=0x6F (RGB565), 0x501F=0x01 (ISP RGB)')
            h['sccb_write'](0x4300, 0x6F)
            h['sccb_write'](0x501F, 0x01)
            time.sleep(0.05)
        if args.post_cycle_4814 is not None:
            print(f'  4814: 0x{args.post_cycle_4814:02X}')
            h['sccb_write'](0x4814, args.post_cycle_4814)
            time.sleep(0.05)
        if args.pll_mult is not None:
            print(f'  PLL mult: 0x3036=0x{args.pll_mult:02X}')
            h['sccb_write'](0x3036, args.pll_mult)
            time.sleep(0.1)
        print('  stream on: 0x300E=0x45, 0x4202=0x00')
        h['sccb_write'](0x300E, 0x45)
        h['sccb_write'](0x4202, 0x00)
        time.sleep(2.0)
        print('  Stream should now have re-evaluated format + frame timing')

    use_lsle_bool = args.use_lsle == '1'
    print(f'Configure frame_lines: 480 lines, use_lsle={int(use_lsle_bool)}, '
          f'expected_dt=0x{args.expected_dt:02X}')
    h['frame_lines_set_keep_cam'](value=480, use_lsle=use_lsle_bool, expected_dt=args.expected_dt)
    h['idelay_set'](args.idelay, args.idelay)
    time.sleep(0.3)

    if args.sweep:
        taps = [int(s) for s in args.sweep_taps.split(',') if s.strip()]
        flat_results: List[tuple] = []
        for p0, p1 in [(0, 6), (6, 4)]:
            rows = idelay_sweep(h, p0, p1, taps, window_s=args.sweep_window_s)
            for tap, d in rows:
                flat_results.append((f'({p0},{p1})', tap, d))
        report_branch(flat_results, nacks)

        if flat_results:
            best_label, best_tap, best_d = max(
                flat_results,
                key=lambda x: (x[2]['long_pkt'], x[2]['short_pkt'], x[2]['crc_ok']),
            )
            print(f'\nBest from sweep: {best_label} tap={best_tap} '
                  f'long={best_d["long_pkt"]} short={best_d["short_pkt"]}')
            p0 = int(best_label[1])
            p1 = int(best_label[3])
            h['bitslip_set'](p0, p1)
            h['idelay_set'](best_tap, best_tap)
            time.sleep(0.3)
        else:
            p0, p1 = 0, 6
    else:
        p0, p1 = find_best_bitslip(h, args.p0, args.p1)

    print('\n=== 5s sustained sample @ best BITSLIP ===')
    b = h['snap']()
    time.sleep(5.0)
    a = h['snap']()
    d = {k: (a[k] - b[k]) % 65536 for k in b}
    print(f'  long={d["long_pkt"]} short={d["short_pkt"]} crc_ok={d["crc_ok"]} crc_err={d["crc_err"]} fs={d["fs"]} fe={d["fe"]}')

    read_diagnostic_pages(h)

    if args.wc_histogram_s > 0:
        sample_wc_histogram(h, args.wc_histogram_s)

    if d['long_pkt'] == 0:
        print('\n*** long_pkt=0 — chip not sending matching pixel data, but capturing anyway ***')

    print('\n=== VDMA S2MM capture ===')
    vdma_desc = ol.ip_dict['axi_vdma_0']
    vdma = MMIO(int(vdma_desc['phys_addr']), int(vdma_desc['addr_range']))
    bufs = [allocate(shape=(HEIGHT, STRIDE), dtype=np.uint8) for _ in range(N_FRAMES)]
    for buf in bufs:
        np.asarray(buf).fill(BUF_FILL_PATTERN)

    try:
        configure_vdma_s2mm(vdma, bufs, start_mm2s=args.enable_hdmi)

        print(f'-- holding {args.capture_hold_s:.1f}s for capture')
        time.sleep(args.capture_hold_s)

        print('-- VDMA status mid-capture')
        decode_vdmasr(int(vdma.read(R.S2MM_VDMASR)), 'S2MM_VDMASR        ')
        if args.enable_hdmi:
            decode_vdmasr(int(vdma.read(R.MM2S_VDMASR)), 'MM2S_VDMASR        ')
            p00 = h['read_dbg'](0x00)
            print(f'  HDMI flags: sof_toggle={(p00 >> 19) & 1} '
                  f'take_seen={(p00 >> 18) & 1} '
                  f'underflow_seen={(p00 >> 17) & 1} '
                  f'axis_error_seen={(p00 >> 16) & 1}')

        print('-- buffer state after capture')
        dump_buffers(bufs, args.dump_prefix)

        print('-- counters after capture')
        final_snap = h['snap']()
        print(f'  long={final_snap["long_pkt"]} short={final_snap["short_pkt"]} '
              f'fs={final_snap["fs"]} fe={final_snap["fe"]} '
              f'ls={final_snap["ls"]} le={final_snap["le"]}')

        if args.hold_seconds > 0:
            print(f'-- holding {args.hold_seconds:.1f}s for live observation '
                  f'(VDMA still running, HDMI={"on" if args.enable_hdmi else "off"})')
            holds = max(1, int(args.hold_seconds / 5))
            for i in range(holds):
                time.sleep(min(5.0, args.hold_seconds - i * 5))
                p00 = h['read_dbg'](0x00)
                snap_i = h['snap']()
                tag = f'[{(i+1)*5:>3}s]' if (i+1)*5 < args.hold_seconds else f'[end]'
                print(f'  {tag} long={snap_i["long_pkt"]} '
                      f'fs={snap_i["fs"]} fe={snap_i["fe"]} '
                      f'HDMI sof_toggle={(p00 >> 19) & 1} '
                      f'take={(p00 >> 18) & 1} '
                      f'uflow={(p00 >> 17) & 1}')
    finally:
        stop_vdma(vdma)
        del bufs
        print('-- cleanup complete; sshd safe to remain up')


if __name__ == '__main__':
    main()
