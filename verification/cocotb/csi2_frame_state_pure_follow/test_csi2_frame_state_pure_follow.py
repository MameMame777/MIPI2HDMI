"""cocotb port of verification/tb/tb_csi2_frame_state_pure_follow.sv.

Pure data-driven (source-following) frame-assembly check. The DUT (csi2_frame_state) is
instantiated with GUARD_FRAME_LINES=0 and driven with cfg_use_lsle=1, so the receiver
imposes NOTHING: no forced 480-line EOF, no WC!=1280 reject, no >=480 long reject, no
MAX_LINES cap, no lsle_line_guard. The FSM must purely transcribe the chip markers:
FS->SOF, (LS,long,LE)->line, FE->EOF. Frame height = whatever was sent BETWEEN FS and FE.

The DSim TB has one cumulative ``initial`` block with three scenarios (A/B/C), each
preceded by ``reset_dut()``. Because every scenario resets the DUT first, they are
independent and are split here into three ``@cocotb.test()``s (fresh reset each). The
marker/payload interface has no standard family driver, so the TB's ``drive_short`` /
``drive_lsle_line`` tasks are ported 1:1 as coroutines, driving on RisingEdge (equivalent
to the TB's ``@(posedge core_clk)`` non-blocking cadence for a posedge-sampled DUT).

Checks (replicated exactly):
  A  a 5-line then 3-line frame come out as 5 and 3 (NOT forced to 480)
  B  a missing FE is resynced by the next FS (2 frames, no unbounded merge)
  C  a tall 600-line frame is passed as 600 (NOT capped at 480/512)
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import ClockCycles, RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import start_clock  # noqa: E402
from lib.scoreboard import check  # noqa: E402


async def reset_dut(dut):
    """Port of the SV reset_dut() task: hold aresetn low 8 cycles, release, settle 2.

    Also drives every input to its idle value (matching the TB) including the config
    inputs that the TB leaves at their RTL default of 0 (cfg_sof_synth /
    cfg_force_expected / cfg_long_as_line) -- cocotb must drive them explicitly.
    """
    dut.core_aresetn.value = 0
    dut.cfg_use_lsle.value = 1
    dut.cfg_expected_frame_lines.value = 0
    dut.cfg_sof_synth.value = 0
    dut.cfg_force_expected.value = 0
    dut.cfg_long_as_line.value = 0
    dut.in_pkt_di.value = 0
    dut.in_pkt_wc.value = 0
    dut.in_pkt_is_short.value = 0
    dut.in_pkt_is_long.value = 0
    dut.in_pkt_start.value = 0
    dut.in_pkt_end.value = 0
    dut.in_pkt_err.value = 0
    dut.in_payload_data.value = 0
    dut.in_payload_valid.value = 0
    dut.in_payload_first.value = 0
    dut.in_payload_last.value = 0
    await ClockCycles(dut.core_clk, 8)
    dut.core_aresetn.value = 1
    await ClockCycles(dut.core_clk, 2)


async def drive_short(dut, dt):
    """Port of drive_short(): a one-beat short packet with DI={2'b00,dt}, WC=0."""
    await RisingEdge(dut.core_clk)
    dut.in_pkt_di.value = dt & 0x3F
    dut.in_pkt_wc.value = 0
    dut.in_pkt_is_short.value = 1
    dut.in_pkt_is_long.value = 0
    dut.in_pkt_start.value = 1
    dut.in_pkt_end.value = 1
    await RisingEdge(dut.core_clk)
    dut.in_pkt_start.value = 0
    dut.in_pkt_end.value = 0
    dut.in_pkt_is_short.value = 0


async def drive_lsle_line(dut, d):
    """Port of drive_lsle_line(): LS short, then a long (WC=1280, DT=0x1e) carrying one
    payload byte (first==last==valid), then LE short. Mirrors the TB cadence exactly."""
    await drive_short(dut, 0x02)                       # LS
    await RisingEdge(dut.core_clk)
    dut.in_pkt_di.value = 0x1E
    dut.in_pkt_wc.value = 1280
    dut.in_pkt_is_short.value = 0
    dut.in_pkt_is_long.value = 1
    dut.in_pkt_start.value = 1
    await RisingEdge(dut.core_clk)
    dut.in_pkt_start.value = 0
    dut.in_payload_data.value = d & 0xFF
    dut.in_payload_first.value = 1
    dut.in_payload_last.value = 1
    dut.in_payload_valid.value = 1
    await RisingEdge(dut.core_clk)
    dut.in_payload_valid.value = 0
    dut.in_payload_first.value = 0
    dut.in_payload_last.value = 0
    dut.in_pkt_end.value = 1
    await RisingEdge(dut.core_clk)
    dut.in_pkt_end.value = 0
    dut.in_pkt_is_long.value = 0
    await drive_short(dut, 0x03)                       # LE


async def _bringup(dut):
    """Start the 100 MHz core clock (TB: #5 half-period) and apply reset."""
    start_clock(dut.core_clk, 10.0)
    await reset_dut(dut)


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def a_source_follows_5_then_3(dut):
    """A: FS/FE-delimited 5-line then 3-line frames come out as 5 and 3 (NOT 480)."""
    await _bringup(dut)

    await drive_short(dut, 0x00)                       # FS
    for i in range(5):
        await drive_lsle_line(dut, i)
    await drive_short(dut, 0x01)                       # FE -> close 5-line frame
    await ClockCycles(dut.core_clk, 4)
    dut._log.info("[A] frames=%d last=%d",
                  int(dut.sts_frame_count.value), int(dut.sts_last_frame_lines.value))
    check(int(dut.sts_frame_count.value) == 1, "A: one frame closed by FE")
    check(int(dut.sts_last_frame_lines.value) == 5,
          "A: frame height follows source (5, NOT 480)")

    await drive_short(dut, 0x00)                       # FS
    for i in range(3):
        await drive_lsle_line(dut, i)
    await drive_short(dut, 0x01)                       # FE -> close 3-line frame
    await ClockCycles(dut.core_clk, 4)
    dut._log.info("[A] frames=%d last=%d",
                  int(dut.sts_frame_count.value), int(dut.sts_last_frame_lines.value))
    check(int(dut.sts_frame_count.value) == 2, "A: two frames")
    check(int(dut.sts_last_frame_lines.value) == 3,
          "A: second frame is 3 lines (source-driven)")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def b_missing_fe_resynced_by_fs(dut):
    """B: a missing FE is force-closed by the next FS -> 2 frames (no unbounded merge)."""
    await _bringup(dut)

    await drive_short(dut, 0x00)                       # FS
    for i in range(4):
        await drive_lsle_line(dut, i)
    await drive_short(dut, 0x00)                       # FS again (FE missing) -> close 4-line
    for i in range(6):
        await drive_lsle_line(dut, i)
    await drive_short(dut, 0x01)                       # FE -> close 6-line
    await ClockCycles(dut.core_clk, 4)
    dut._log.info("[B] frames=%d last=%d sync_err=%d",
                  int(dut.sts_frame_count.value), int(dut.sts_last_frame_lines.value),
                  int(dut.sts_frame_sync_err_cnt.value))
    check(int(dut.sts_frame_count.value) == 2,
          "B: missing-FE handled by FS resync (2 frames)")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def c_tall_600_line_uncapped(dut):
    """C: a tall 600-line frame passes uncapped (no 480/512 limit)."""
    await _bringup(dut)

    await drive_short(dut, 0x00)                       # FS
    for i in range(600):
        await drive_lsle_line(dut, i & 0xFF)
    await drive_short(dut, 0x01)                       # FE -> close 600-line frame
    await ClockCycles(dut.core_clk, 4)
    dut._log.info("[C] frames=%d last=%d",
                  int(dut.sts_frame_count.value), int(dut.sts_last_frame_lines.value))
    check(int(dut.sts_frame_count.value) == 1, "C: tall frame closed")
    check(int(dut.sts_last_frame_lines.value) == 600,
          "C: 600-line frame passed UNCAPPED (no 480/512 impose)")


def test_csi2_frame_state_pure_follow():
    from runner_support import build_and_test

    build_and_test(
        block="csi2_frame_state_pure_follow",
        sources=["rtl/mipi_rx/csi2_frame_state.sv"],
        toplevel="csi2_frame_state",
        test_module="test_csi2_frame_state_pure_follow",
        test_dir=Path(__file__).resolve().parent,
        parameters={
            "MAX_LINES": 2048,
            "GUARD_FRAME_LINES": 0,
            "EXPECTED_FRAME_LINES": 480,
            "EXPECTED_LINE_WC": 1280,
        },
        engine="verilator",
    )
