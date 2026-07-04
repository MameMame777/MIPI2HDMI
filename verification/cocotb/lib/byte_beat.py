"""Driver for the csi2 byte-beat interface: ``s_byte_{data,keep,valid,sop,eop}`` (no
tready). Mirrors the DSim ``drive_beat`` task -- one beat asserts ``valid`` for a single
cycle then idles one cycle (the 2-cycle cadence the parser was validated against).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from cocotb.triggers import RisingEdge

from lib.gap import GapPolicy, default_gap_policy


@dataclass
class Beat:
    data: int
    keep: int = 0b11
    sop: bool = False
    eop: bool = False


class ByteBeatDriver:
    def __init__(self, dut, clk, prefix: str = "s_byte") -> None:
        self.clk = clk
        self.data = getattr(dut, f"{prefix}_data")
        self.keep = getattr(dut, f"{prefix}_keep")
        self.valid = getattr(dut, f"{prefix}_valid")
        self.sop = getattr(dut, f"{prefix}_sop")
        self.eop = getattr(dut, f"{prefix}_eop")

    async def idle(self) -> None:
        self.data.value = 0
        self.keep.value = 0
        self.valid.value = 0
        self.sop.value = 0
        self.eop.value = 0

    async def send(self, beat: Beat, gap_policy: Optional[GapPolicy] = None) -> None:
        pol = gap_policy if gap_policy is not None else default_gap_policy()
        if pol.active:
            for _ in range(pol.next_gap()):     # extra idle (valid=0) cycles before the beat
                await RisingEdge(self.clk)
                self.valid.value = 0
        await RisingEdge(self.clk)
        self.data.value = beat.data
        self.keep.value = beat.keep
        self.valid.value = 1
        self.sop.value = int(beat.sop)
        self.eop.value = int(beat.eop)
        await RisingEdge(self.clk)
        self.valid.value = 0
        self.sop.value = 0
        self.eop.value = 0
        self.keep.value = 0
        self.data.value = 0
