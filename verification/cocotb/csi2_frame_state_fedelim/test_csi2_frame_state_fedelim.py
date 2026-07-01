"""cocotb port of verification/tb/tb_csi2_frame_state_fedelim.sv (custom packet interface).

FE-DELIMITER mode for ``csi2_frame_state`` (2026-06-04). DUT is instantiated with
``MAX_LINES=8, GUARD_FRAME_LINES=1, EXPECTED_FRAME_LINES=4, EXPECTED_LINE_WC=0,
FS_MIN_LINES=4, FE_DELIMITS=1`` and driven with ``cfg_use_lsle=1`` /
``cfg_expected_frame_lines=0`` so ``lsle_line_guard`` and the FE-delimiter path are active.

The block has a bespoke packet-header + payload interface (``in_pkt_*`` / ``in_payload_*``)
that no standard lib driver covers, so the SV ``reset_dut`` / ``drive_short`` /
``drive_lsle_line`` tasks are ported 1:1 as coroutines that drive the DUT signals directly
on ``RisingEdge`` (equivalent to the TB's NBA-on-posedge driving for this posedge-sampled
DUT). The four scenarios A..D each start from a fresh reset, exactly as the SV ``initial``
block does, and every ``chk(...)`` becomes a ``check(...)``.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import ClockCycles, RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import start_clock  # noqa: E402
from lib.scoreboard import check  # noqa: E402


# --- SV task ports --------------------------------------------------------------------

async def reset_dut(dut, clk):
    """Port of the SV reset_dut task: hold reset low 8 cycles with all inputs cleared,
    cfg_use_lsle=1, then release and settle 2 cycles."""
    dut.core_aresetn.value = 0
    dut.cfg_use_lsle.value = 1
    dut.cfg_expected_frame_lines.value = 0
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
    # optional opt-in cfg knobs (default 0 in the SV DUT port defaults)
    if hasattr(dut, "cfg_sof_synth"):
        dut.cfg_sof_synth.value = 0
    if hasattr(dut, "cfg_force_expected"):
        dut.cfg_force_expected.value = 0
    if hasattr(dut, "cfg_long_as_line"):
        dut.cfg_long_as_line.value = 0
    for _ in range(8):
        await RisingEdge(clk)
    dut.core_aresetn.value = 1
    for _ in range(2):
        await RisingEdge(clk)


async def drive_short(dut, clk, dt):
    """Port of drive_short: a 1-cycle short packet with DI[5:0]=dt, is_short/start/end=1."""
    await RisingEdge(clk)
    dut.in_pkt_di.value = dt & 0x3F      # {2'b00, dt}
    dut.in_pkt_wc.value = 0
    dut.in_pkt_is_short.value = 1
    dut.in_pkt_is_long.value = 0
    dut.in_pkt_start.value = 1
    dut.in_pkt_end.value = 1
    await RisingEdge(clk)
    dut.in_pkt_start.value = 0
    dut.in_pkt_end.value = 0
    dut.in_pkt_is_short.value = 0


async def drive_lsle_line(dut, clk, d):
    """Port of drive_lsle_line: LS short, a DT=0x2a long packet carrying one payload byte,
    then LE short (which increments line_idx)."""
    await drive_short(dut, clk, 0x02)                       # LS
    await RisingEdge(clk)
    dut.in_pkt_di.value = 0x2A
    dut.in_pkt_wc.value = 1
    dut.in_pkt_is_short.value = 0
    dut.in_pkt_is_long.value = 1
    dut.in_pkt_start.value = 1
    await RisingEdge(clk)
    dut.in_pkt_start.value = 0
    dut.in_payload_data.value = d & 0xFF
    dut.in_payload_first.value = 1
    dut.in_payload_last.value = 1
    dut.in_payload_valid.value = 1
    await RisingEdge(clk)
    dut.in_payload_valid.value = 0
    dut.in_payload_first.value = 0
    dut.in_payload_last.value = 0
    dut.in_pkt_end.value = 1
    await RisingEdge(clk)
    dut.in_pkt_end.value = 0
    dut.in_pkt_is_long.value = 0
    await drive_short(dut, clk, 0x03)                       # LE (line_idx++)


def _fc(dut):
    return int(dut.sts_frame_count.value)


def _last(dut):
    return int(dut.sts_last_frame_lines.value)


def _synerr(dut):
    return int(dut.sts_frame_sync_err_cnt.value)


# --- scenarios ------------------------------------------------------------------------

@cocotb.test(timeout_time=1, timeout_unit="ms")
async def scenario_a_plausible_fe_closes(dut):
    """A: a plausible FE (>= FS_MIN=4 lines) closes the frame (FS open, 5 lines, FE@5)."""
    start_clock(dut.core_clk, 10.0)
    await reset_dut(dut, dut.core_clk)

    await drive_short(dut, dut.core_clk, 0x00)              # FS open
    for i in range(5):
        await drive_lsle_line(dut, dut.core_clk, i)        # 5 lines
    await drive_short(dut, dut.core_clk, 0x01)             # FE @5 -> close
    await ClockCycles(dut.core_clk, 4)

    dut._log.info("[A] frames=%d last=%d sync_err=%d",
                  _fc(dut), _last(dut), _synerr(dut))
    check(_fc(dut) == 1, "A: plausible FE closed one frame")
    check(_last(dut) == 5, "A: frame = 5 lines")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def scenario_b_spurious_early_fe_ignored(dut):
    """B: a spurious early FE (@3 < FS_MIN=4) is ignored; plausible FE@5 closes."""
    start_clock(dut.core_clk, 10.0)
    await reset_dut(dut, dut.core_clk)

    await drive_short(dut, dut.core_clk, 0x00)
    for i in range(3):
        await drive_lsle_line(dut, dut.core_clk, i)        # 3 lines
    await drive_short(dut, dut.core_clk, 0x01)             # FE @3 (<4) -> ignored
    check(_fc(dut) == 0, "B: spurious early FE did NOT close a frame")

    for i in range(2):
        await drive_lsle_line(dut, dut.core_clk, i)        # now 5 lines
    await drive_short(dut, dut.core_clk, 0x01)             # FE @5 -> close
    await ClockCycles(dut.core_clk, 4)

    dut._log.info("[B] frames=%d last=%d", _fc(dut), _last(dut))
    check(_fc(dut) == 1, "B: plausible FE closed exactly one frame")
    check(_last(dut) == 5, "B: frame = 5 lines (spurious FE@3 ignored)")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def scenario_c_missing_fe_capped(dut):
    """C: a missing FE is bounded by the MAX_LINES cap (=8)."""
    start_clock(dut.core_clk, 10.0)
    await reset_dut(dut, dut.core_clk)

    await drive_short(dut, dut.core_clk, 0x00)
    for i in range(10):
        await drive_lsle_line(dut, dut.core_clk, i)        # no FE
    await ClockCycles(dut.core_clk, 4)

    dut._log.info("[C] frames=%d last=%d", _fc(dut), _last(dut))
    check(_fc(dut) == 1, "C: missing FE bounded by MAX_LINES cap")
    check(_last(dut) == 8, "C: capped at MAX_LINES=8")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def scenario_d_in_frame_fs_ignored(dut):
    """D: an in-frame FS is IGNORED (no re-anchor/close); FE stays the sole delimiter."""
    start_clock(dut.core_clk, 10.0)
    await reset_dut(dut, dut.core_clk)

    await drive_short(dut, dut.core_clk, 0x00)             # FS open frame#1
    for i in range(5):
        await drive_lsle_line(dut, dut.core_clk, i)
    await drive_short(dut, dut.core_clk, 0x01)             # FE -> close frame#1 (=5), IDLE
    await ClockCycles(dut.core_clk, 2)
    check(_fc(dut) == 1, "D: frame#1 closed on FE")

    await drive_short(dut, dut.core_clk, 0x00)             # FS open frame#2
    for i in range(2):
        await drive_lsle_line(dut, dut.core_clk, i)        # 2 lines
    await drive_short(dut, dut.core_clk, 0x00)             # in-frame FS -> IGNORED
    check(_fc(dut) == 1, "D: in-frame FS did NOT close/count a frame")

    for i in range(5):
        await drive_lsle_line(dut, dut.core_clk, i)        # 5 more lines (2+5=7)
    await drive_short(dut, dut.core_clk, 0x01)             # FE -> close frame#2 (=7)
    await ClockCycles(dut.core_clk, 4)

    dut._log.info("[D] frames=%d last=%d", _fc(dut), _last(dut))
    check(_fc(dut) == 2, "D: FE closed frame#2 (in-frame FS ignored)")
    check(_last(dut) == 7,
          "D: frame#2 = 7 lines (spurious in-frame FS ignored, not re-anchored)")


def test_csi2_frame_state_fedelim():
    from runner_support import build_and_test

    build_and_test(
        block="csi2_frame_state_fedelim",
        sources=["rtl/mipi_rx/csi2_frame_state.sv"],
        toplevel="csi2_frame_state",
        test_module="test_csi2_frame_state_fedelim",
        test_dir=Path(__file__).resolve().parent,
        parameters={
            "MAX_LINES": 8,
            "GUARD_FRAME_LINES": 1,
            "EXPECTED_FRAME_LINES": 4,
            "EXPECTED_LINE_WC": 0,
            "FS_MIN_LINES": 4,
            "FE_DELIMITS": 1,
        },
        engine="verilator",
    )
