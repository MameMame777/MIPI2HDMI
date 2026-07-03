"""IMG_* environment parsing for the img_file_uvm block.

One dataclass, parsed identically by the pytest (host) process and the Verilator sim
process (env vars are inherited), so the two sides can never disagree about kernels,
ops or thresholds. Host-only fields (file/max size/out dir/python) are simply unused
on the sim side.

Env surface (all optional):
  IMG_FILE          input image path (any Pillow format, or .ppm/.pgm stdlib-direct);
                    unset -> built-in 64x48 test pattern
  IMG_DUT           conv3x3|conv5x5|prefilter|proc_slot|dither ; unset -> all
  IMG_MAX_W/H       downscale bound for the converter (default 640x480)
  IMG_OUT_DIR       output base dir (default verification/cocotb/_exec/img_file_uvm)
  IMG_PYTHON        interpreter for the Pillow converter (default repo .venv CPython)
  IMG_KERNEL        conv3x3: identity|gaussian|sharpen|sobel_x|sobel_y|emboss|laplacian
                    conv5x5: identity|gaussian5|log5
  IMG_COEFFS        comma list of 9/25 signed ints (-128..127), overrides IMG_KERNEL
  IMG_SHIFT         conv right-shift override (0..15)
  IMG_ABS           conv |result| override (0/1)
  IMG_EN            conv cfg_en (default 1; 0 = passthrough)
  IMG_OP            prefilter/proc_slot op (name or number)
  IMG_THRESH        threshold level for op "threshold" (default 128)
  IMG_DITHER_MODE   ordered|random (default ordered)
  IMG_DITHER_BITS   bits/channel 1..6 (default 2)
  IMG_DITHER_CTRL   raw cfg_ctrl byte override (e.g. 0x0B)
  IMG_FRAMES        frames to stream back-to-back (default 1)
  IMG_SELFTEST_CORRUPT  1 -> corrupt one expected pixel (scoreboard must go red)
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import List, Optional

# ---- conv kernel presets: name -> (coeffs row-major, default shift, default abs) ----
CONV3_KERNELS = {
    "identity":  ([0, 0, 0, 0, 1, 0, 0, 0, 0], 0, 0),
    "gaussian":  ([1, 2, 1, 2, 4, 2, 1, 2, 1], 4, 0),
    "sharpen":   ([0, -1, 0, -1, 5, -1, 0, -1, 0], 0, 0),
    "sobel_x":   ([-1, 0, 1, -2, 0, 2, -1, 0, 1], 0, 1),
    "sobel_y":   ([-1, -2, -1, 0, 0, 0, 1, 2, 1], 0, 1),
    "emboss":    ([-2, -1, 0, -1, 1, 1, 0, 1, 2], 0, 0),
    "laplacian": ([0, -1, 0, -1, 4, -1, 0, -1, 0], 0, 1),
}
CONV5_KERNELS = {
    "identity": ([0] * 12 + [1] + [0] * 12, 0, 0),
    "gaussian5": ([1, 4, 6, 4, 1,
                   4, 16, 24, 16, 4,
                   6, 24, 36, 24, 6,
                   4, 16, 24, 16, 4,
                   1, 4, 6, 4, 1], 8, 0),
    "log5": ([0, 0, -1, 0, 0,
              0, -1, -2, -1, 0,
              -1, -2, 16, -2, -1,
              0, -1, -2, -1, 0,
              0, 0, -1, 0, 0], 0, 1),
}

# ---- point/window op name tables (numeric strings also accepted) ----
PREFILTER_OPS = {"pass": 0, "invert": 1, "gray": 2, "swap": 3, "threshold": 4,
                 "r_only": 5, "g_only": 6, "b_only": 7, "gaussian": 8, "median": 9}
PROC_SLOT_OPS = {"pass": 0, "invert": 1, "gray": 2, "swap": 3, "threshold": 4,
                 "r_only": 5, "g_only": 6, "b_only": 7}

# per-DUT defaults when the relevant env var is unset
DEFAULT_KERNEL_3 = "gaussian"
DEFAULT_KERNEL_5 = "gaussian5"
DEFAULT_OP_PREFILTER = "median"
DEFAULT_OP_PROC_SLOT = "invert"
DEFAULT_DITHER_MODE = "ordered"
DEFAULT_DITHER_BITS = 2


def _env_int(name: str, default: Optional[int]) -> Optional[int]:
    v = os.environ.get(name)
    if v is None or v == "":
        return default
    return int(v, 0)


@dataclass
class ImgConfig:
    # host side
    file: Optional[str]
    max_w: int
    max_h: int
    out_dir: Optional[str]
    python: Optional[str]
    # both sides
    kernel: Optional[str]
    coeffs: Optional[List[int]]
    shift: Optional[int]
    abs_: Optional[int]
    en: int
    op: Optional[str]
    thresh: int
    dither_mode: str
    dither_bits: int
    dither_ctrl: Optional[int]
    frames: int
    selftest_corrupt: bool

    @classmethod
    def from_env(cls) -> "ImgConfig":
        coeffs = None
        raw = os.environ.get("IMG_COEFFS")
        if raw:
            coeffs = [int(t, 0) for t in raw.replace(";", ",").split(",") if t.strip()]
            for c in coeffs:
                if not -128 <= c <= 127:
                    raise ValueError(f"IMG_COEFFS entry {c} outside signed-8 range")
        mode = os.environ.get("IMG_DITHER_MODE", DEFAULT_DITHER_MODE).lower()
        if mode not in ("ordered", "random"):
            raise ValueError(f"IMG_DITHER_MODE {mode!r} (need ordered|random)")
        return cls(
            file=os.environ.get("IMG_FILE") or None,
            max_w=_env_int("IMG_MAX_W", 640),
            max_h=_env_int("IMG_MAX_H", 480),
            out_dir=os.environ.get("IMG_OUT_DIR") or None,
            python=os.environ.get("IMG_PYTHON") or None,
            kernel=os.environ.get("IMG_KERNEL") or None,
            coeffs=coeffs,
            shift=_env_int("IMG_SHIFT", None),
            abs_=_env_int("IMG_ABS", None),
            en=_env_int("IMG_EN", 1),
            op=os.environ.get("IMG_OP") or None,
            thresh=_env_int("IMG_THRESH", 128),
            dither_mode=mode,
            dither_bits=_env_int("IMG_DITHER_BITS", DEFAULT_DITHER_BITS),
            dither_ctrl=_env_int("IMG_DITHER_CTRL", None),
            frames=max(1, _env_int("IMG_FRAMES", 1)),
            selftest_corrupt=os.environ.get("IMG_SELFTEST_CORRUPT", "") not in ("", "0"),
        )

    # ---- per-DUT resolution (single source of truth for pins AND golden) ----

    def resolve_conv(self, taps: int) -> dict:
        """-> {en, coeffs(list, taps*taps), shift, abs} for conv3x3/conv5x5."""
        table = CONV3_KERNELS if taps == 3 else CONV5_KERNELS
        default = DEFAULT_KERNEL_3 if taps == 3 else DEFAULT_KERNEL_5
        n = taps * taps
        if self.coeffs is not None:
            if len(self.coeffs) != n:
                raise ValueError(f"IMG_COEFFS needs {n} entries, got {len(self.coeffs)}")
            coeffs, shift, absf = self.coeffs, 0, 0
            name = "custom"
        else:
            name = (self.kernel or default).lower()
            if name not in table:
                raise ValueError(f"IMG_KERNEL {name!r}: choose from {sorted(table)}")
            coeffs, shift, absf = table[name]
        if self.shift is not None:
            shift = self.shift
        if self.abs_ is not None:
            absf = self.abs_
        if not 0 <= shift <= 15:
            raise ValueError(f"IMG_SHIFT {shift} outside 0..15")
        return {"name": name, "en": int(self.en), "coeffs": list(coeffs),
                "shift": int(shift), "abs": int(absf)}

    def _resolve_op(self, table: dict, default: str, what: str) -> int:
        raw = (self.op or default).lower()
        if raw in table:
            return table[raw]
        try:
            num = int(raw, 0)
        except ValueError:
            raise ValueError(f"IMG_OP {raw!r}: choose from {sorted(table)} or a number")
        if num not in table.values():
            raise ValueError(f"IMG_OP {num} out of range for {what}")
        return num

    def resolve_prefilter(self) -> dict:
        op = self._resolve_op(PREFILTER_OPS, DEFAULT_OP_PREFILTER, "prefilter")
        return {"op": op, "thresh": int(self.thresh) & 0xFF}

    def resolve_proc_slot(self) -> dict:
        op = self._resolve_op(PROC_SLOT_OPS, DEFAULT_OP_PROC_SLOT, "proc_slot")
        return {"op": op, "thresh": int(self.thresh) & 0xFF}

    def resolve_dither(self) -> dict:
        if self.dither_ctrl is not None:
            ctrl = self.dither_ctrl & 0xFF
        else:
            if not 1 <= self.dither_bits <= 6:
                raise ValueError(f"IMG_DITHER_BITS {self.dither_bits} outside 1..6")
            ctrl = 0x01 | ((1 if self.dither_mode == "random" else 0) << 1) \
                | (self.dither_bits << 2)
        return {"ctrl": ctrl}
