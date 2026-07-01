"""cocotb port of verification/tb/tb_csi2_frame_state_roll_repro.sv.

Rolling root-cause reproduction for the FS-anchor frame-state FSM
(``csi2_frame_state`` in cfg_use_lsle + GUARD mode). The DUT closes a frame on
the next FS and re-syncs line_idx=0, so each frame == the line span between
consecutive FS. This TB drives three streams and records ``sts_last_frame_lines``
on every EOF:

  * LOCK     : FS every 5 lines, x6            -> all spans == 5  (no roll)
  * ROLL     : FS at 5,4,6,3,7 lines           -> spans vary       (roll mechanism)
  * SPURIOUS : one extra mid-frame FS           -> 2-line then 5-line frame

Interface family: this block has a bespoke packet-marker port
(``in_pkt_*`` / ``in_payload_*``) with no *tready* backpressure, so the raw
signals are driven directly (RisingEdge-based, equivalent to the TB's negedge/NBA
driving for a posedge-sampled DUT). The SV ``always_ff`` span logger becomes the
``EofSpanLogger`` monitor coroutine; the three ``initial``-block scenarios become
three ``@cocotb.test()`` coroutines (each fresh-reset), which reproduces the same
cumulative span_log expectations as the original single run.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge, ClockCycles

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import start_clock, reset_active_low  # noqa: E402
from lib.scoreboard import check  # noqa: E402


# ---------------------------------------------------------------------------
# always_ff span logger:  record sts_last_frame_lines on each EOF.
# ---------------------------------------------------------------------------
class EofSpanLogger:
    def __init__(self, dut):
        self.dut = dut
        self.spans: list[int] = []

    def start(self, clk):
        return cocotb.start_soon(self._run(clk))

    async def _run(self, clk):
        d = self.dut
        while True:
            await RisingEdge(clk)
            if int(d.core_aresetn.value) == 1 and int(d.out_eof.value) == 1:
                self.spans.append(int(d.sts_last_frame_lines.value))


# ---------------------------------------------------------------------------
# Stimulus driver: mirrors the SV tasks 1:1.  All input signals are assigned
# after a RisingEdge (the cocotb analogue of the TB's @(posedge) NBA writes).
# ---------------------------------------------------------------------------
class RollDriver:
    def __init__(self, dut, clk):
        self.dut = dut
        self.clk = clk

    def _idle_inputs(self):
        d = self.dut
        d.cfg_use_lsle.value = 1
        d.cfg_expected_frame_lines.value = 0
        d.in_pkt_di.value = 0x00
        d.in_pkt_wc.value = 0x0000
        d.in_pkt_is_short.value = 0
        d.in_pkt_is_long.value = 0
        d.in_pkt_start.value = 0
        d.in_pkt_end.value = 0
        d.in_pkt_err.value = 0
        d.in_payload_data.value = 0x00
        d.in_payload_valid.value = 0
        d.in_payload_first.value = 0
        d.in_payload_last.value = 0

    async def reset_dut(self):
        """SV reset_dut(): drive inputs idle, hold aresetn low 8 cyc, release, +2 cyc."""
        d = self.dut
        d.core_aresetn.value = 0
        self._idle_inputs()
        # extra opt-in config knobs default to 0 (match TB / DUT defaults)
        for name in ("cfg_sof_synth", "cfg_force_expected", "cfg_long_as_line"):
            if hasattr(d, name):
                getattr(d, name).value = 0
        await ClockCycles(self.clk, 8)
        d.core_aresetn.value = 1
        await ClockCycles(self.clk, 2)

    async def drive_short(self, dt):
        d = self.dut
        await RisingEdge(self.clk)
        d.in_pkt_di.value = dt & 0x3F        # {2'b00, dt}
        d.in_pkt_wc.value = 0x0000
        d.in_pkt_is_short.value = 1
        d.in_pkt_is_long.value = 0
        d.in_pkt_start.value = 1
        d.in_pkt_end.value = 1
        await RisingEdge(self.clk)
        d.in_pkt_start.value = 0
        d.in_pkt_end.value = 0
        d.in_pkt_is_short.value = 0

    async def drive_lsle_line(self, data):
        d = self.dut
        await self.drive_short(0x02)                       # LS
        await RisingEdge(self.clk)
        d.in_pkt_di.value = 0x2A
        d.in_pkt_wc.value = 1
        d.in_pkt_is_short.value = 0
        d.in_pkt_is_long.value = 1
        d.in_pkt_start.value = 1
        await RisingEdge(self.clk)
        d.in_pkt_start.value = 0
        d.in_payload_data.value = data & 0xFF
        d.in_payload_first.value = 1
        d.in_payload_last.value = 1
        d.in_payload_valid.value = 1
        await RisingEdge(self.clk)
        d.in_payload_valid.value = 0
        d.in_payload_first.value = 0
        d.in_payload_last.value = 0
        d.in_pkt_end.value = 1
        await RisingEdge(self.clk)
        d.in_pkt_end.value = 0
        d.in_pkt_is_long.value = 0
        await self.drive_short(0x03)                       # LE

    async def drive_frame(self, nlines):
        await self.drive_short(0x00)                       # FS
        for i in range(nlines):
            await self.drive_lsle_line(i & 0xFF)


def _all_equal(q):
    return all(x == q[0] for x in q) if q else True


async def _setup(dut):
    clk = dut.core_clk
    start_clock(clk, period_ns=10.0)     # forever #5 core_clk
    drv = RollDriver(dut, clk)
    logger = EofSpanLogger(dut)
    logger.start(clk)
    return clk, drv, logger


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def lock_constant_span(dut):
    """LOCK: constant FS-to-FS span -> phase locked (all spans equal)."""
    clk, drv, logger = await _setup(dut)
    await drv.reset_dut()
    logger.spans.clear()                 # span_log.delete() (reset_dut clears it in SV)
    for _ in range(6):
        await drv.drive_frame(5)
    await drv.drive_short(0x00)          # final FS closes last frame
    await ClockCycles(clk, 4)
    spans = list(logger.spans)
    dut._log.info(f"[LOCK]     closed-frame spans = {spans}")
    check(len(spans) >= 5, "LOCK: at least 5 frames closed")
    check(_all_equal(spans),
          "LOCK: constant FS interval => constant frame height (PHASE LOCKED)")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def roll_varying_span(dut):
    """ROLL: varying FS-to-FS span -> rolling (spans differ)."""
    clk, drv, logger = await _setup(dut)
    await drv.reset_dut()
    logger.spans.clear()
    await drv.drive_frame(5)
    await drv.drive_frame(4)
    await drv.drive_frame(6)
    await drv.drive_frame(3)
    await drv.drive_frame(7)
    await drv.drive_short(0x00)
    await ClockCycles(clk, 4)
    spans = list(logger.spans)
    dut._log.info(f"[ROLL]     closed-frame spans = {spans}")
    check(len(spans) >= 5, "ROLL: frames closed")
    check(not _all_equal(spans),
          "ROLL: varying FS interval => varying frame height (ROLL MECHANISM)")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def spurious_midframe_fs(dut):
    """SPURIOUS: one extra mid-frame FS injects a short frame (spans == [2, 5])."""
    clk, drv, logger = await _setup(dut)
    await drv.reset_dut()
    logger.spans.clear()
    await drv.drive_short(0x00)          # FS open
    for i in range(2):
        await drv.drive_lsle_line(i & 0xFF)
    await drv.drive_short(0x00)          # SPURIOUS mid-frame FS -> closes 2-line frame
    for i in range(5):
        await drv.drive_lsle_line(i & 0xFF)
    await drv.drive_short(0x00)          # closes 5-line frame
    await ClockCycles(clk, 4)
    spans = list(logger.spans)
    dut._log.info(
        f"[SPURIOUS] closed-frame spans = {spans} "
        f"(a stray FS injected a short {spans[0] if spans else '?'}-line frame)")
    check(len(spans) == 2, "SPURIOUS: two frames closed")
    check(spans[0] == 2 and spans[1] == 5,
          "SPURIOUS: stray FS makes a 2-line then 5-line frame")


def test_csi2_frame_state_roll_repro():
    from runner_support import build_and_test

    build_and_test(
        block="csi2_frame_state_roll_repro",
        sources=["rtl/mipi_rx/csi2_frame_state.sv"],
        toplevel="csi2_frame_state",
        test_module="test_csi2_frame_state_roll_repro",
        test_dir=Path(__file__).resolve().parent,
        parameters={
            "MAX_LINES": 32,
            "GUARD_FRAME_LINES": 1,
            "EXPECTED_FRAME_LINES": 8,
            "EXPECTED_LINE_WC": 0,
        },
        engine="verilator",
    )
