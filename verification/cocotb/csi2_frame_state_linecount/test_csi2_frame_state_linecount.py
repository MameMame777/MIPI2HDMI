"""cocotb port of verification/tb/tb_csi2_frame_state_linecount.sv.

Regression for the LSLE FS-anchor hybrid frame delimiter (diary 20260601).

The DUT (``csi2_frame_state``) is instantiated with MAX_LINES=8, GUARD_FRAME_LINES=1,
EXPECTED_FRAME_LINES=4, EXPECTED_LINE_WC=0 and driven in ``cfg_use_lsle=1`` (hardware)
mode. In this mode the ``lsle_line_guard`` HYBRID delimiter is active:

  A  FS delimits frames (phase anchor)   -> two 3-line frames, 2 EOFs
  B  FE is swallowed (never closes)       -> FE adds no frame; still two 3-line frames
  C  a missing FS is bounded by MAX_LINES -> runaway capped at 8 lines, one frame
  D  an early FS is honoured              -> a short 2-line frame closes on FS

This block has a bespoke packet interface (``in_pkt_*`` / ``in_payload_*``) with no
matching lib driver, so the SV ``drive_short`` / ``drive_lsle_line`` tasks are replicated
here as coroutines that poke the signals directly with the same per-cycle cadence. The SV
tasks drive with non-blocking assignment after ``@(posedge core_clk)``; the cocotb
equivalent is ``await RisingEdge(clk)`` then set ``.value`` (applied after that edge,
sampled on the next), which is timing-equivalent for this posedge-sampled DUT.

The ``sof_count`` / ``eof_count`` always_ff logger in the TB becomes an ``EofCounter``
monitor coroutine (only ``eof_count`` is asserted on; ``sof_count`` is unused by the TB
checks). Each scenario is a fresh-reset ``@cocotb.test()`` -- the SV ``reset_dut()`` at the
head of every scenario resets the counters, so splitting the single cumulative ``initial``
into four independent tests reproduces the same expectations.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import start_clock  # noqa: E402
from lib.scoreboard import check  # noqa: E402

DT_FS = 0x00
DT_FE = 0x01
DT_LS = 0x02
DT_LE = 0x03


class EofCounter:
    """Mirror the TB always_ff logger: count out_eof (and out_sof) pulses, reset-aware."""

    def __init__(self, dut):
        self.dut = dut
        self.eof_count = 0
        self.sof_count = 0

    def start(self, clk):
        return cocotb.start_soon(self._run(clk))

    async def _run(self, clk):
        d = self.dut
        while True:
            await RisingEdge(clk)
            if int(d.core_aresetn.value) == 0:
                self.eof_count = 0
                self.sof_count = 0
            else:
                if int(d.out_sof.value) == 1:
                    self.sof_count += 1
                if int(d.out_eof.value) == 1:
                    self.eof_count += 1


class FrameStateDriver:
    """Replicates the SV drive_* tasks 1:1 (bespoke in_pkt_* / in_payload_* interface)."""

    def __init__(self, dut, clk):
        self.dut = dut
        self.clk = clk

    def _idle_inputs(self):
        d = self.dut
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
        d = self.dut
        d.core_aresetn.value = 0
        d.cfg_use_lsle.value = 1            # hardware mode
        self._idle_inputs()
        for _ in range(8):
            await RisingEdge(self.clk)
        d.core_aresetn.value = 1
        for _ in range(2):
            await RisingEdge(self.clk)

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
        """LS, long(1 byte), LE -- the per-line short-packet bracket in LSLE mode."""
        d = self.dut
        await self.drive_short(DT_LS)                 # LS
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
        await self.drive_short(DT_LE)                 # LE

    async def settle(self):
        for _ in range(4):
            await RisingEdge(self.clk)


async def _bringup(dut):
    start_clock(dut.core_clk, 10.0)
    drv = FrameStateDriver(dut, dut.core_clk)
    ctr = EofCounter(dut)
    ctr.start(dut.core_clk)
    await drv.reset_dut()
    return drv, ctr


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def a_fs_anchor(dut):
    """A: FS delimits frames (phase anchor) -> two 3-line frames, 2 EOFs."""
    drv, ctr = await _bringup(dut)
    await drv.drive_short(DT_FS)                 # FS (open, from idle)
    for i in range(3):
        await drv.drive_lsle_line(i)
    await drv.drive_short(DT_FS)                 # FS -> close frame A (3 lines), reopen
    for i in range(3):
        await drv.drive_lsle_line(10 + i)
    await drv.drive_short(DT_FS)                 # FS -> close frame B (3 lines), reopen
    await drv.settle()
    dut._log.info(
        "[A fs-anchor] frames=%d last=%d sync_err=%d eof=%d",
        int(dut.sts_frame_count.value), int(dut.sts_last_frame_lines.value),
        int(dut.sts_frame_sync_err_cnt.value), ctr.eof_count)
    check(int(dut.sts_frame_count.value) == 2, "A: FS delimits -> two frames")
    check(int(dut.sts_last_frame_lines.value) == 3,
          "A: each frame is the 3 lines between FS")
    check(ctr.eof_count == 2, "A: two EOFs (one per FS close)")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def b_fe_swallow(dut):
    """B: FE is swallowed (never closes a frame)."""
    drv, ctr = await _bringup(dut)
    await drv.drive_short(DT_FS)                 # FS (open)
    for i in range(3):
        await drv.drive_lsle_line(i)
    await drv.drive_short(DT_FE)                 # FE -> must be swallowed (no close)
    prev_frames = int(dut.sts_frame_count.value)
    check(prev_frames == 0, "B: FE did NOT close a frame")
    await drv.drive_short(DT_FS)                 # FS -> close frame A (3 lines)
    for i in range(3):
        await drv.drive_lsle_line(10 + i)
    await drv.drive_short(DT_FS)                 # FS -> close frame B
    await drv.settle()
    dut._log.info("[B fe-swallow] frames=%d last=%d",
                  int(dut.sts_frame_count.value), int(dut.sts_last_frame_lines.value))
    check(int(dut.sts_frame_count.value) == 2, "B: two frames (FE never added one)")
    check(int(dut.sts_last_frame_lines.value) == 3, "B: frames are 3 lines")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def c_runaway_cap(dut):
    """C: a missing FS is bounded by the MAX_LINES cap (8), not unbounded."""
    drv, ctr = await _bringup(dut)
    await drv.drive_short(DT_FS)                 # FS (open)
    for i in range(10):                         # no FS for 10 lines
        await drv.drive_lsle_line(i)
    await drv.settle()
    dut._log.info("[C cap]       frames=%d last=%d",
                  int(dut.sts_frame_count.value), int(dut.sts_last_frame_lines.value))
    check(int(dut.sts_frame_count.value) == 1, "C: runaway capped -> one frame closed")
    check(int(dut.sts_last_frame_lines.value) == 8,
          "C: capped at MAX_LINES (8), not unbounded")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def d_early_fs(dut):
    """D: an early FS is honoured (frame follows the chip, short 2-line frame)."""
    drv, ctr = await _bringup(dut)
    await drv.drive_short(DT_FS)                 # FS (open)
    await drv.drive_lsle_line(0)
    await drv.drive_lsle_line(1)                 # only 2 lines
    await drv.drive_short(DT_FS)                 # FS -> close a SHORT 2-line frame
    await drv.settle()
    dut._log.info("[D early-fs]  frames=%d last=%d",
                  int(dut.sts_frame_count.value), int(dut.sts_last_frame_lines.value))
    check(int(dut.sts_frame_count.value) == 1, "D: early FS closes the frame")
    check(int(dut.sts_last_frame_lines.value) == 2,
          "D: frame honours FS (2 lines), phase follows chip")


def test_csi2_frame_state_linecount():
    from runner_support import build_and_test

    build_and_test(
        block="csi2_frame_state_linecount",
        sources=["rtl/mipi_rx/csi2_frame_state.sv"],
        toplevel="csi2_frame_state",
        test_module="test_csi2_frame_state_linecount",
        test_dir=Path(__file__).resolve().parent,
        parameters={
            "MAX_LINES": 8,
            "GUARD_FRAME_LINES": 1,
            "EXPECTED_FRAME_LINES": 4,
            "EXPECTED_LINE_WC": 0,
        },
    )
