"""cocotb port of verification/tb/tb_csi2_frame_state_fsmin.sv.

FS plausibility-floor test. The DUT (csi2_frame_state) is configured to mirror the
hardware LSLE line-guard path: GUARD_FRAME_LINES=1, cfg_use_lsle=1, EXPECTED_FRAME_LINES=4,
FS_MIN_LINES=4, MAX_LINES=8, FE_DELIMITS=0. The interface is the module's custom
in_pkt_* / in_payload_* packet interface (not one of the shared-lib families), so a small
local driver replicates the three SV tasks (drive_short / drive_lsle_line / reset_dut)
cycle-for-cycle. Each frame line = LS(0x02) short + a 1-byte long + LE(0x03) short; the LE
increments the DUT's line counter.

Three scenarios, each fresh-reset (mirroring the SV `reset_dut()` before A/B/C):
  A: a spurious early FS (< FS_MIN_LINES=4) is IGNORED; a plausible FS (>= 4) closes a
     5-line frame.
  B: a missing FS is bounded by the MAX_LINES=8 cap (8-line frame).
  C: a plausible FS at exactly FS_MIN_LINES=4 is accepted (4-line frame).

Every SV chk(...) is replicated 1:1 with check(...). RisingEdge-based driving here is the
posedge-sampled equivalent of the TB's @(posedge) + non-blocking assignments.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import ClockCycles, RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import bringup  # noqa: E402
from lib.scoreboard import check  # noqa: E402


class FrameStateDriver:
    """Replicates the SV drive_short / drive_lsle_line tasks on the in_pkt_* interface."""

    def __init__(self, dut, clk):
        self.dut = dut
        self.clk = clk

    async def idle(self):
        d = self.dut
        d.in_pkt_di.value = 0
        d.in_pkt_wc.value = 0
        d.in_pkt_is_short.value = 0
        d.in_pkt_is_long.value = 0
        d.in_pkt_start.value = 0
        d.in_pkt_end.value = 0
        d.in_pkt_err.value = 0
        d.in_payload_data.value = 0
        d.in_payload_valid.value = 0
        d.in_payload_first.value = 0
        d.in_payload_last.value = 0

    async def drive_short(self, dt: int):
        """SV task drive_short: one-cycle short packet with the given 6-bit DT."""
        d = self.dut
        await RisingEdge(self.clk)
        d.in_pkt_di.value = dt & 0x3F          # {2'b00, dt}
        d.in_pkt_wc.value = 0
        d.in_pkt_is_short.value = 1
        d.in_pkt_is_long.value = 0
        d.in_pkt_start.value = 1
        d.in_pkt_end.value = 1
        await RisingEdge(self.clk)
        d.in_pkt_start.value = 0
        d.in_pkt_end.value = 0
        d.in_pkt_is_short.value = 0

    async def drive_lsle_line(self, data: int):
        """SV task drive_lsle_line: LS(0x02) short, 1-byte long, LE(0x03) short."""
        d = self.dut
        await self.drive_short(0x02)
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
        await self.drive_short(0x03)


async def _reset(dut):
    """Mirror the SV reset_dut(): cfg_use_lsle=1, idle inputs, 8-low + 2-settle reset."""
    dut.cfg_use_lsle.value = 1
    dut.cfg_expected_frame_lines.value = 0
    # optional cfg knobs default 0 (legacy path), matching the DSim instantiation defaults
    for name in ("cfg_sof_synth", "cfg_force_expected", "cfg_long_as_line"):
        if hasattr(dut, name):
            getattr(dut, name).value = 0
    drv = FrameStateDriver(dut, dut.core_clk)
    await drv.idle()
    clk, _ = await bringup(dut, clk="core_clk", rst="core_aresetn")
    return clk, drv


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def spurious_early_fs_ignored(dut):
    """A: spurious early FS (@2 < FS_MIN=4) ignored; plausible FS (@5) closes a 5-line frame."""
    clk, drv = await _reset(dut)

    await drv.drive_short(0x00)                       # FS (open)
    await drv.drive_lsle_line(0)
    await drv.drive_lsle_line(1)                      # 2 lines
    await drv.drive_short(0x00)                       # FS @2 (<4) -> SPURIOUS, ignored
    check(int(dut.sts_frame_count.value) == 0,
          "A: spurious early FS did NOT close a frame")
    await drv.drive_lsle_line(2)
    await drv.drive_lsle_line(3)
    await drv.drive_lsle_line(4)                      # now 5 lines
    await drv.drive_short(0x00)                       # FS @5 (>=4) -> plausible, close
    await ClockCycles(clk, 4)
    dut._log.info("[A] frames=%d last=%d sync_err=%d",
                  int(dut.sts_frame_count.value),
                  int(dut.sts_last_frame_lines.value),
                  int(dut.sts_frame_sync_err_cnt.value))
    check(int(dut.sts_frame_count.value) == 1,
          "A: plausible FS closed exactly one frame")
    check(int(dut.sts_last_frame_lines.value) == 5,
          "A: frame = 5 lines (spurious @2 ignored, real @5 honoured)")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def missing_fs_capped_at_max(dut):
    """B: missing FS -> MAX_LINES cap (=8)."""
    clk, drv = await _reset(dut)

    await drv.drive_short(0x00)
    for i in range(10):
        await drv.drive_lsle_line(i & 0xFF)
    await ClockCycles(clk, 4)
    dut._log.info("[B] frames=%d last=%d",
                  int(dut.sts_frame_count.value),
                  int(dut.sts_last_frame_lines.value))
    check(int(dut.sts_frame_count.value) == 1,
          "B: missing FS bounded by MAX_LINES cap")
    check(int(dut.sts_last_frame_lines.value) == 8,
          "B: capped at MAX_LINES=8")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def plausible_fs_at_exactly_fs_min(dut):
    """C: plausible FS at exactly FS_MIN (4) is accepted."""
    clk, drv = await _reset(dut)

    await drv.drive_short(0x00)
    for i in range(4):
        await drv.drive_lsle_line(i & 0xFF)
    await drv.drive_short(0x00)                       # FS @4 (==FS_MIN) -> accepted
    await ClockCycles(clk, 4)
    dut._log.info("[C] frames=%d last=%d",
                  int(dut.sts_frame_count.value),
                  int(dut.sts_last_frame_lines.value))
    check(int(dut.sts_frame_count.value) == 1,
          "C: FS at exactly FS_MIN accepted")
    check(int(dut.sts_last_frame_lines.value) == 4,
          "C: 4-line frame")


def test_csi2_frame_state_fsmin():
    from runner_support import build_and_test

    build_and_test(
        block="csi2_frame_state_fsmin",
        sources=["rtl/mipi_rx/csi2_frame_state.sv"],
        toplevel="csi2_frame_state",
        test_module="test_csi2_frame_state_fsmin",
        test_dir=Path(__file__).resolve().parent,
        parameters={
            "MAX_LINES": 8,
            "GUARD_FRAME_LINES": 1,
            "EXPECTED_FRAME_LINES": 4,
            "EXPECTED_LINE_WC": 0,
            "FS_MIN_LINES": 4,
        },
        engine="verilator",
    )
