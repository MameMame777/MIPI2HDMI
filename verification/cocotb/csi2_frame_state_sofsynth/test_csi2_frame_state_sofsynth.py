"""cocotb port of verification/tb/tb_csi2_frame_state_sofsynth.sv.

SOF-SYNTHESIS mode test for csi2_frame_state (DUT parameters mirror hardware
intent: MAX_LINES=8, GUARD_FRAME_LINES=1, EXPECTED_FRAME_LINES=4,
EXPECTED_LINE_WC=0, FS_MIN_LINES=4, FE_DELIMITS=1).

The DUT sits downstream of the packet parser, so its inputs are the parser
outputs (in_pkt_* / in_payload_*), NOT a standard AXI-Stream. The DSim TB drives
them with hand-rolled tasks (drive_short / drive_lsle_line) using non-blocking
assignments on posedge core_clk. Those tasks are replicated 1:1 here as a
FrameStateDriver: each ``await RisingEdge(clk); sig.value = X`` schedules X to be
sampled at the NEXT posedge, exactly matching the SV ``@(posedge core_clk); sig<=X``
cadence.

The ``sof_cnt`` counter (SV always_ff on out_sof while aresetn) becomes the
SofCounter monitor coroutine. The five scenarios A/B/C/D/E become five
@cocotb.test coroutines, each fresh-reset (the SV TB called reset_dut between
scenarios). Every SV chk(...) is preserved as a check(...).
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import ClockCycles, RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import start_clock  # noqa: E402
from lib.scoreboard import check  # noqa: E402


class SofCounter:
    """Mirror the SV always_ff: increment on out_sof while core_aresetn=1."""

    def __init__(self, dut):
        self.dut = dut
        self.count = 0

    def start(self, clk):
        return cocotb.start_soon(self._run(clk))

    async def _run(self, clk):
        d = self.dut
        while True:
            await RisingEdge(clk)
            if int(d.core_aresetn.value) == 1 and int(d.out_sof.value) == 1:
                self.count += 1


class FrameStateDriver:
    """Replicates the SV drive_short / drive_lsle_line tasks on posedge core_clk.

    All writes are scheduled after a RisingEdge so they take effect at the next
    posedge -- the cocotb equivalent of the SV non-blocking (``<=``) task writes.
    """

    def __init__(self, dut, clk):
        self.dut = dut
        self.clk = clk

    async def reset(self, synth: bool):
        """SV reset_dut(synth): drive-quiet inputs, hold aresetn low 8 cyc, release, +2."""
        d = self.dut
        d.core_aresetn.value = 0
        d.cfg_use_lsle.value = 1
        d.cfg_sof_synth.value = 1 if synth else 0
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
        # tie off other cfg inputs the TB left at their defaults (0)
        d.cfg_expected_frame_lines.value = 0
        d.cfg_force_expected.value = 0
        d.cfg_long_as_line.value = 0
        await ClockCycles(self.clk, 8)
        d.core_aresetn.value = 1
        await ClockCycles(self.clk, 2)

    async def drive_short(self, dt: int):
        """SV drive_short(dt): a 1-cycle short packet (start&end both high)."""
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
        """SV drive_lsle_line(d): LS short, then a 1-byte long packet, then LE short."""
        d = self.dut
        await self.drive_short(0x02)           # LS
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
        await self.drive_short(0x03)           # LE (line_idx++)


# DUT parameters -- exactly the values the TB instantiates the DUT with.
PARAMS = {
    "MAX_LINES": 8,
    "GUARD_FRAME_LINES": 1,
    "EXPECTED_FRAME_LINES": 4,
    "EXPECTED_LINE_WC": 0,
    "FS_MIN_LINES": 4,
    "FE_DELIMITS": 1,
}


async def _setup(dut, synth: bool):
    clk = dut.core_clk
    start_clock(clk, period_ns=10.0)           # SV: forever #5 core_clk=~core_clk
    sof = SofCounter(dut)
    sof.start(clk)
    drv = FrameStateDriver(dut, clk)
    await drv.reset(synth)
    return clk, sof, drv


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def scenario_a(dut):
    """A: FS-LESS stream, sof_synth=1 -> synthesize SOF per frame, FE closes."""
    clk, sof, drv = await _setup(dut, synth=True)
    for i in range(5):
        await drv.drive_lsle_line(i)           # 5 lines, NO opening FS
    await drv.drive_short(0x01)                # FE @5 (>=FS_MIN=4) -> close
    await ClockCycles(clk, 4)
    dut._log.info("[A] frames=%d last=%d sof=%d sync_err=%d",
                  int(dut.sts_frame_count.value), int(dut.sts_last_frame_lines.value),
                  sof.count, int(dut.sts_frame_sync_err_cnt.value))
    check(int(dut.sts_frame_count.value) == 1, "A: synthetic-open frame closed by FE")
    check(int(dut.sts_last_frame_lines.value) == 5, "A: frame = 5 lines")
    check(sof.count == 1, "A: exactly one synthesized SOF")

    # second frame, still no FS
    for i in range(5):
        await drv.drive_lsle_line(i)
    await drv.drive_short(0x01)                # FE -> close frame#2
    await ClockCycles(clk, 4)
    dut._log.info("[A2] frames=%d sof=%d", int(dut.sts_frame_count.value), sof.count)
    check(int(dut.sts_frame_count.value) == 2, "A2: second FS-less frame opened+closed")
    check(sof.count == 2, "A2: two synthesized SOFs total")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def scenario_b(dut):
    """B: FS-LESS stream, sof_synth=0 -> frame never opens, no SOF."""
    clk, sof, drv = await _setup(dut, synth=False)
    for i in range(5):
        await drv.drive_lsle_line(i)
    await drv.drive_short(0x01)
    await ClockCycles(clk, 4)
    dut._log.info("[B] frames=%d sof=%d", int(dut.sts_frame_count.value), sof.count)
    check(int(dut.sts_frame_count.value) == 0, "B: legacy (sof_synth=0) does NOT open without FS")
    check(sof.count == 0, "B: no SOF without FS in legacy mode")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def scenario_c(dut):
    """C: intermittent FS, sof_synth=1 -> normal FS open coexists with synthetic."""
    clk, sof, drv = await _setup(dut, synth=True)
    # frame#1 via SYNTHETIC open (no FS)
    for i in range(5):
        await drv.drive_lsle_line(i)
    await drv.drive_short(0x01)                # FE -> close#1
    await ClockCycles(clk, 2)
    check(int(dut.sts_frame_count.value) == 1, "C: synthetic frame#1 closed")
    # frame#2 via REAL FS
    await drv.drive_short(0x00)                # FS open frame#2 (normal path)
    for i in range(5):
        await drv.drive_lsle_line(i)
    await drv.drive_short(0x01)                # FE -> close#2
    await ClockCycles(clk, 4)
    dut._log.info("[C] frames=%d last=%d sof=%d",
                  int(dut.sts_frame_count.value), int(dut.sts_last_frame_lines.value), sof.count)
    check(int(dut.sts_frame_count.value) == 2, "C: real-FS frame#2 also closed")
    # synthetic open = 1 SOF (payload-aligned); real FS = 2 SOF (immediate on FS
    # cycle + sof_pending on first payload) => total 3.
    check(sof.count == 3, "C: synthetic(1) + FS-driven(2, existing double-SOF) SOFs")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def scenario_d(dut):
    """D: FS-less + dropped FE, sof_synth=1 -> MAX_LINES cap bounds the frame."""
    clk, sof, drv = await _setup(dut, synth=True)
    for i in range(10):
        await drv.drive_lsle_line(i)           # no FS, no FE
    await ClockCycles(clk, 4)
    dut._log.info("[D] frames=%d last=%d sof=%d",
                  int(dut.sts_frame_count.value), int(dut.sts_last_frame_lines.value), sof.count)
    check(int(dut.sts_frame_count.value) == 1, "D: synthetic open + missing FE bounded by MAX cap")
    check(int(dut.sts_last_frame_lines.value) == 8, "D: capped at MAX_LINES=8")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def scenario_e(dut):
    """E: FE-RESYNC -- after a cap-close the synthetic open is suppressed until the
    next FE re-establishes the chip frame top."""
    clk, sof, drv = await _setup(dut, synth=True)
    for i in range(8):
        await drv.drive_lsle_line(i)           # 8 lines, no FS/FE -> cap@8
    await ClockCycles(clk, 2)
    check(int(dut.sts_frame_count.value) == 1, "E: cap-close counted one frame")
    check(int(dut.sts_last_frame_lines.value) == 8, "E: capped at 8")
    # while waiting for FE, further LS must NOT open a frame (phase unknown)
    for i in range(2):
        await drv.drive_lsle_line(i)
    check(int(dut.sts_frame_count.value) == 1, "E: LS after cap-close does NOT open (waiting for FE)")
    # the chip's FE arrives in IDLE and re-establishes phase (no open/close itself)
    await drv.drive_short(0x01)                # FE -> resync
    await ClockCycles(clk, 2)
    check(int(dut.sts_frame_count.value) == 1, "E: resync FE does not itself open/close a frame")
    # now the next frame opens at the true top and FE closes it
    for i in range(5):
        await drv.drive_lsle_line(i)           # 5 lines
    await drv.drive_short(0x01)                # FE -> close frame#2
    await ClockCycles(clk, 4)
    dut._log.info("[E] frames=%d last=%d",
                  int(dut.sts_frame_count.value), int(dut.sts_last_frame_lines.value))
    check(int(dut.sts_frame_count.value) == 2, "E: after FE-resync the next frame opens+closes (phase-locked)")
    check(int(dut.sts_last_frame_lines.value) == 5, "E: phase-locked frame = 5 lines")


def test_csi2_frame_state_sofsynth():
    from runner_support import build_and_test

    build_and_test(
        block="csi2_frame_state_sofsynth",
        sources=["rtl/mipi_rx/csi2_frame_state.sv"],
        toplevel="csi2_frame_state",
        test_module="test_csi2_frame_state_sofsynth",
        test_dir=Path(__file__).resolve().parent,
        parameters=PARAMS,
        engine="verilator",
    )
