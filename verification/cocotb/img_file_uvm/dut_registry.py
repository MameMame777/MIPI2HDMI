"""DUT registry for the img_file_uvm block: one table drives everything.

Each entry maps a short name (IMG_DUT) to its RTL sources, Verilator toplevel, build
parameters, cfg-pin drive procedure and software golden model. The pytest wrapper uses
sources/toplevel/has_line_pixels for the per-DUT build; the sim-side pyuvm test uses
resolve/drive_cfg/golden/describe. Adding a DUT = adding one DutSpec (the conv5x5_sep /
dog_combine chain would slot in the same way).

Golden signature: golden(stream, width, height, resolved) -> expected stream, where
``stream`` is the full multi-frame pixel sequence (frames concatenated -- the streaming
models carry line-buffer/LFSR state across frame boundaries exactly like the RTL).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Tuple

import golden as G
from img_config import ImgConfig


@dataclass(frozen=True)
class DutSpec:
    name: str
    sources: Tuple[str, ...]            # repo-relative
    toplevel: str
    has_line_pixels: bool
    resolve: Callable                   # (ImgConfig) -> dict (pins + golden params)
    drive_cfg: Callable                 # async (dut, resolved) -> None
    golden: Callable                    # (stream, width, height, resolved) -> list
    describe: Callable                  # (resolved) -> str


# --------------------------------------------------------------------------- conv3x3/5x5

async def _drive_conv(dut, rc):
    dut.cfg_en.value = rc["en"]
    dut.cfg_coeffs.value = G.pack_coeffs(rc["coeffs"])
    dut.cfg_shift.value = rc["shift"]
    dut.cfg_abs.value = rc["abs"]


def _mk_conv(taps: int):
    def resolve(cfg: ImgConfig) -> dict:
        return cfg.resolve_conv(taps)

    def gold(stream, width, height, rc):
        return G.conv_golden(stream, width, rc["coeffs"], rc["shift"],
                             rc["abs"], rc["en"], taps)

    def describe(rc) -> str:
        return (f"conv{taps}x{taps} kernel={rc['name']} coeffs={rc['coeffs']} "
                f"shift={rc['shift']} abs={rc['abs']} en={rc['en']}")

    return resolve, gold, describe


_R3, _G3, _D3 = _mk_conv(3)
_R5, _G5, _D5 = _mk_conv(5)


# --------------------------------------------------------------------------- prefilter

async def _drive_prefilter(dut, rc):
    dut.cfg_op.value = rc["op"]
    dut.cfg_thresh_level.value = rc["thresh"]


def _gold_prefilter(stream, width, height, rc):
    return G.prefilter_golden(stream, width, rc["op"], rc["thresh"])


# --------------------------------------------------------------------------- proc_slot

def _gold_proc_slot(stream, width, height, rc):
    return G.proc_slot_golden(stream, rc["op"], rc["thresh"])


# --------------------------------------------------------------------------- dither

async def _drive_dither(dut, rc):
    dut.cfg_ctrl.value = rc["ctrl"]


def _gold_dither(stream, width, height, rc):
    return G.dither_golden(stream, width, height, rc["ctrl"])


DUTS = {
    "conv3x3": DutSpec(
        name="conv3x3",
        sources=("rtl/img_proc/axis_rgb_conv3x3.sv",),
        toplevel="axis_rgb_conv3x3",
        has_line_pixels=True,
        resolve=_R3, drive_cfg=_drive_conv, golden=_G3, describe=_D3),
    "conv5x5": DutSpec(
        name="conv5x5",
        sources=("rtl/img_proc/axis_rgb_conv5x5.sv",),
        toplevel="axis_rgb_conv5x5",
        has_line_pixels=True,
        resolve=_R5, drive_cfg=_drive_conv, golden=_G5, describe=_D5),
    "prefilter": DutSpec(
        name="prefilter",
        sources=("rtl/img_proc/median9.sv", "rtl/img_proc/axis_rgb_prefilter.sv"),
        toplevel="axis_rgb_prefilter",
        has_line_pixels=True,
        resolve=ImgConfig.resolve_prefilter, drive_cfg=_drive_prefilter,
        golden=_gold_prefilter,
        describe=lambda rc: f"prefilter op={rc['op']} thresh={rc['thresh']}"),
    "proc_slot": DutSpec(
        name="proc_slot",
        sources=("rtl/img_proc/axis_rgb_proc_slot.sv",),
        toplevel="axis_rgb_proc_slot",
        has_line_pixels=False,
        resolve=ImgConfig.resolve_proc_slot, drive_cfg=_drive_prefilter,
        golden=_gold_proc_slot,
        describe=lambda rc: f"proc_slot op={rc['op']} thresh={rc['thresh']}"),
    "dither": DutSpec(
        name="dither",
        sources=("rtl/img_proc/axis_rgb_dither.sv",),
        toplevel="axis_rgb_dither",
        has_line_pixels=True,
        resolve=ImgConfig.resolve_dither, drive_cfg=_drive_dither,
        golden=_gold_dither,
        describe=lambda rc: f"dither ctrl=0x{rc['ctrl']:02x}"),
}
