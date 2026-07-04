"""True AXI4-Stream helpers (``*_t{valid,ready,data,last,user}``): a sink that drives
tready (with optional back-pressure gaps), a monitor that captures accepted beats, and a
minimal source for driving AXIS *inputs* (VDMA bridges, Phase 2+). Only the subset this
project uses is modelled -- no tstrb/tid/tdest.
"""
from __future__ import annotations

from typing import List, Optional

import cocotb
from cocotb.triggers import RisingEdge

from lib.gap import GapPolicy, default_gap_policy


def _maybe(dut, name: Optional[str]):
    return getattr(dut, name) if name and hasattr(dut, name) else None


class AxisMonitor:
    """Records ``{data,last,user}`` on each rising edge where tvalid & tready are high."""

    def __init__(self, dut, clk, prefix: str = "m_axis") -> None:
        self.clk = clk
        self.tvalid = getattr(dut, f"{prefix}_tvalid")
        self.tready = getattr(dut, f"{prefix}_tready")
        self.tdata = getattr(dut, f"{prefix}_tdata")
        self.tlast = _maybe(dut, f"{prefix}_tlast")
        self.tuser = _maybe(dut, f"{prefix}_tuser")
        self.beats: List[dict] = []

    def start(self):
        return cocotb.start_soon(self._run())

    async def _run(self) -> None:
        while True:
            await RisingEdge(self.clk)
            if int(self.tvalid.value) == 1 and int(self.tready.value) == 1:
                self.beats.append({
                    "data": int(self.tdata.value),
                    "last": int(self.tlast.value) if self.tlast is not None else 0,
                    "user": int(self.tuser.value) if self.tuser is not None else 0,
                })


class AxisSink:
    """Drives ``tready``; optional randomized backpressure via ``backpressure()``."""

    def __init__(self, dut, clk, prefix: str = "m_axis", ready: bool = True) -> None:
        self.clk = clk
        self.tready = getattr(dut, f"{prefix}_tready")
        self.tready.value = 1 if ready else 0
        self._bp = None

    def set_ready(self, value: bool) -> None:
        self.tready.value = 1 if value else 0

    def start_backpressure(self, gap_policy: Optional[GapPolicy] = None):
        """Spawn a background task that toggles ``tready`` per the gap policy: ready for one
        cycle, then de-asserted for ``next_gap()`` cycles, repeating. Off (held ready) when the
        policy is inactive, so the default gate is unchanged. Returns the task."""
        pol = gap_policy if gap_policy is not None else default_gap_policy()
        self._bp = cocotb.start_soon(self._backpressure(pol))
        return self._bp

    async def _backpressure(self, pol: GapPolicy) -> None:
        while True:
            self.tready.value = 1
            await RisingEdge(self.clk)
            if not pol.active:
                continue
            self.tready.value = 0
            for _ in range(pol.next_gap()):
                await RisingEdge(self.clk)


class AxisSource:
    """Drives an AXIS slave input, honouring tready. For Phase 2+ VDMA bridges."""

    def __init__(self, dut, clk, prefix: str = "s_axis") -> None:
        self.clk = clk
        self.tvalid = getattr(dut, f"{prefix}_tvalid")
        self.tready = _maybe(dut, f"{prefix}_tready")
        self.tdata = getattr(dut, f"{prefix}_tdata")
        self.tlast = _maybe(dut, f"{prefix}_tlast")
        self.tuser = _maybe(dut, f"{prefix}_tuser")

    async def idle(self) -> None:
        self.tvalid.value = 0
        if self.tlast is not None:
            self.tlast.value = 0

    async def send(self, data: int, last: int = 0, user: int = 0,
                   gap_policy: Optional[GapPolicy] = None) -> None:
        pol = gap_policy if gap_policy is not None else default_gap_policy()
        if pol.active:
            for _ in range(pol.next_gap()):     # idle (tvalid=0) cycles before offering data
                await RisingEdge(self.clk)
                self.tvalid.value = 0
        await RisingEdge(self.clk)
        self.tdata.value = data
        self.tvalid.value = 1
        if self.tlast is not None:
            self.tlast.value = int(last)
        if self.tuser is not None:
            self.tuser.value = int(user)
        # wait until accepted (tready high) if backpressure is modelled
        while True:
            await RisingEdge(self.clk)
            if self.tready is None or int(self.tready.value) == 1:
                break
        self.tvalid.value = 0
        if self.tlast is not None:
            self.tlast.value = 0
