"""Driver + monitor for the img_proc "valid-only pixel" family (no tready).

Signal names differ across blocks (``in_valid`` vs ``in_pixel_valid``), so every role is
configurable. ``send_frame`` asserts ``sof`` on the first pixel, ``eol`` at each row end,
and ``eof`` on the last -- the framing these DUTs expect.
"""
from __future__ import annotations

from typing import List, Optional, Sequence

import cocotb
from cocotb.triggers import FallingEdge, RisingEdge


def _maybe(dut, name: Optional[str]):
    if name and hasattr(dut, name):
        return getattr(dut, name)
    return None


class PixelStreamDriver:
    def __init__(self, dut, clk, pixel="in_pixel", valid="in_valid", sof="in_sof",
                 eol="in_eol", eof="in_eof", err="in_err", drive_edge: str = "rising") -> None:
        self.clk = clk
        self.pixel = getattr(dut, pixel)
        self.valid = getattr(dut, valid)
        self.sof = getattr(dut, sof)
        self.eol = getattr(dut, eol)
        self.eof = getattr(dut, eof)
        self.err = _maybe(dut, err)
        self._edge = FallingEdge if drive_edge == "falling" else RisingEdge

    async def idle(self) -> None:
        self.valid.value = 0
        self.sof.value = 0
        self.eol.value = 0
        self.eof.value = 0
        if self.err is not None:
            self.err.value = 0
        self.pixel.value = 0

    async def send(self, pixel: int, sof=0, eol=0, eof=0, err=0) -> None:
        """Drive one pixel for a single cycle, then deassert valid/markers (mirrors the
        DSim ``drive_pixel`` task -- a 2-cycle cadence)."""
        await self._edge(self.clk)
        self.pixel.value = pixel
        self.valid.value = 1
        self.sof.value = int(sof)
        self.eol.value = int(eol)
        self.eof.value = int(eof)
        if self.err is not None:
            self.err.value = int(err)
        await self._edge(self.clk)
        self.valid.value = 0
        self.sof.value = 0
        self.eol.value = 0
        self.eof.value = 0
        if self.err is not None:
            self.err.value = 0

    async def send_frame(self, pixels: Sequence[int], width: int, err: int = 0) -> None:
        n = len(pixels)
        for i, px in enumerate(pixels):
            await self._edge(self.clk)
            self.pixel.value = px
            self.valid.value = 1
            self.sof.value = 1 if i == 0 else 0
            self.eol.value = 1 if (i % width) == width - 1 else 0
            self.eof.value = 1 if i == n - 1 else 0
            if self.err is not None:
                self.err.value = err
        await self._edge(self.clk)
        await self.idle()


class PixelMonitor:
    """Captures ``out_*`` beats on each rising edge where ``out_valid`` is high."""

    def __init__(self, dut, clk, pixel="out_pixel", valid="out_valid", sof="out_sof",
                 eol="out_eol", eof="out_eof", err="out_err") -> None:
        self.clk = clk
        self.pixel = getattr(dut, pixel)
        self.valid = getattr(dut, valid)
        self.sof = _maybe(dut, sof)
        self.eol = _maybe(dut, eol)
        self.eof = _maybe(dut, eof)
        self.err = _maybe(dut, err)
        self.beats: List[dict] = []
        self._task = None

    def start(self):
        self._task = cocotb.start_soon(self._run())
        return self._task

    async def _run(self) -> None:
        while True:
            await RisingEdge(self.clk)
            if int(self.valid.value) == 1:
                self.beats.append({
                    "pixel": int(self.pixel.value),
                    "sof": int(self.sof.value) if self.sof is not None else 0,
                    "eol": int(self.eol.value) if self.eol is not None else 0,
                    "eof": int(self.eof.value) if self.eof is not None else 0,
                    "err": int(self.err.value) if self.err is not None else 0,
                })

    def channel(self, idx: int) -> List[int]:
        """Extract byte lane ``idx`` (0=LSB) from each captured pixel."""
        return [(b["pixel"] >> (idx * 8)) & 0xFF for b in self.beats]
