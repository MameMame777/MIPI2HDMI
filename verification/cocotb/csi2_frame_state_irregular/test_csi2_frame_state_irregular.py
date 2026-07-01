"""cocotb port of verification/tb/tb_csi2_frame_state_irregular.sv.

Regression for the FS-resync fix (diary 20260530 Phase 9-11). The DUT
(``csi2_frame_state``) is instantiated in ``cfg_use_lsle=1`` mode with
``GUARD_FRAME_LINES`` defaulted to 0 (so ``lsle_line_guard`` is inactive and an
in-frame FS takes the "force-close stale frame + re-sync" branch that is the
subject of this test).

The DSim TB drives the raw ``in_pkt_*`` / ``in_payload_*`` packet interface via
three hand-written tasks (``drive_short``, ``drive_lsle_line``, ``drive_short``
FS/FE) and counts ``out_sof`` / ``out_eof`` in an ``always_ff`` block. Those are
ported 1:1: the tasks become coroutines that drive the DUT signals on rising
edges (equivalent to the TB's non-blocking posedge driving for a posedge-sampled
DUT), and the ``always_ff`` counter becomes the ``SofEofCounter`` monitor.

Each DSim ``reset_dut()`` boundary starts a fresh ``@cocotb.test()``; the three
TB scenarios (clean / missed-FE / early-FE + orphans + recovery) map onto three
tests, replicating every ``check_condition``.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import ClockCycles, RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import start_clock  # noqa: E402
from lib.scoreboard import check  # noqa: E402


# ---------------------------------------------------------------------------
# always_ff @(posedge core_clk) sof_count/eof_count logger
# ---------------------------------------------------------------------------
class SofEofCounter:
    """Mirror the TB always_ff: count out_sof / out_eof pulses while out of reset."""

    def __init__(self, dut):
        self.dut = dut
        self.sof_count = 0
        self.eof_count = 0
        self._task = None

    def start(self, clk):
        self._task = cocotb.start_soon(self._run(clk))
        return self._task

    async def _run(self, clk):
        d = self.dut
        while True:
            await RisingEdge(clk)
            if int(d.core_aresetn.value) == 0:
                self.sof_count = 0
                self.eof_count = 0
            else:
                if int(d.out_sof.value) == 1:
                    self.sof_count += 1
                if int(d.out_eof.value) == 1:
                    self.eof_count += 1


# ---------------------------------------------------------------------------
# TB tasks -> coroutines. All driving is on RisingEdge(core_clk); values set
# right after the edge are sampled at the next posedge -- the same one-cycle
# pulse the TB produces with non-blocking assignments.
# ---------------------------------------------------------------------------
async def reset_dut(dut, clk):
    """Port of task reset_dut(): active-low reset, cfg_use_lsle=1, inputs cleared."""
    dut.core_aresetn.value = 0
    dut.cfg_use_lsle.value = 1            # hardware mode
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
    await ClockCycles(clk, 8)
    dut.core_aresetn.value = 1
    await ClockCycles(clk, 2)


async def drive_short(dut, clk, dt):
    """Port of task drive_short(dt): one-cycle short packet (start+end)."""
    await RisingEdge(clk)
    dut.in_pkt_di.value = dt & 0x3F           # {2'b00, dt}
    dut.in_pkt_wc.value = 0
    dut.in_pkt_is_short.value = 1
    dut.in_pkt_is_long.value = 0
    dut.in_pkt_start.value = 1
    dut.in_pkt_end.value = 1
    await RisingEdge(clk)
    dut.in_pkt_start.value = 0
    dut.in_pkt_end.value = 0
    dut.in_pkt_is_short.value = 0


async def drive_lsle_line(dut, clk, data):
    """Port of task drive_lsle_line(data): LS short, long(1 byte), LE short."""
    await drive_short(dut, clk, 0x02)             # LS
    await RisingEdge(clk)
    dut.in_pkt_di.value = 0x2A
    dut.in_pkt_wc.value = 1
    dut.in_pkt_is_short.value = 0
    dut.in_pkt_is_long.value = 1
    dut.in_pkt_start.value = 1
    await RisingEdge(clk)
    dut.in_pkt_start.value = 0
    dut.in_payload_data.value = data & 0xFF
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
    await drive_short(dut, clk, 0x03)             # LE


async def settle(clk):
    """Port of task settle(): 4 idle clocks."""
    await ClockCycles(clk, 4)


async def _bringup(dut):
    start_clock(dut.core_clk, period_ns=10.0)
    counter = SofEofCounter(dut)
    counter.start(dut.core_clk)
    await reset_dut(dut, dut.core_clk)
    return dut.core_clk, counter


# ---------------------------------------------------------------------------
# Scenario 1: CLEAN baseline (LSLE, 8 lines)
# ---------------------------------------------------------------------------
@cocotb.test(timeout_time=1, timeout_unit="ms")
async def s1_clean_baseline(dut):
    clk, _ = await _bringup(dut)

    await drive_short(dut, clk, 0x00)             # FS
    for i in range(8):
        await drive_lsle_line(dut, clk, i)
    await drive_short(dut, clk, 0x01)             # FE
    await settle(clk)

    dut._log.info(
        "[S1 clean]   frames=%d last_lines=%d sync_err=%d"
        % (int(dut.sts_frame_count.value),
           int(dut.sts_last_frame_lines.value),
           int(dut.sts_frame_sync_err_cnt.value))
    )
    check(int(dut.sts_frame_count.value) == 1, "S1 one frame")
    check(int(dut.sts_last_frame_lines.value) == 8, "S1 8 lines")
    check(int(dut.sts_frame_sync_err_cnt.value) == 0, "S1 no sync err")


# ---------------------------------------------------------------------------
# Scenario 2: MISSED FE, then next FS (the fix target)
# FS, 8 lines, <FE DROPPED>, FS, 8 lines, FE.
# FIXED: FS#2 force-closes frame A (8 lines, EOF) and re-syncs; FE closes frame
# B (8 lines) => TWO 8-line frames, NO 16-line merge.
# ---------------------------------------------------------------------------
@cocotb.test(timeout_time=1, timeout_unit="ms")
async def s2_missed_fe(dut):
    clk, counter = await _bringup(dut)

    await drive_short(dut, clk, 0x00)             # FS (frame A)
    for i in range(8):
        await drive_lsle_line(dut, clk, i)
    # (FE for frame A is intentionally DROPPED)
    await drive_short(dut, clk, 0x00)             # FS (frame B) -- arrives in-frame
    for i in range(8):
        await drive_lsle_line(dut, clk, 8 + i)
    await drive_short(dut, clk, 0x01)             # FE
    await settle(clk)

    dut._log.info(
        "[S2 missedFE] frames=%d last_lines=%d sync_err=%d sof=%d eof=%d"
        % (int(dut.sts_frame_count.value),
           int(dut.sts_last_frame_lines.value),
           int(dut.sts_frame_sync_err_cnt.value),
           counter.sof_count, counter.eof_count)
    )
    check(int(dut.sts_frame_count.value) == 2,
          "S2 FIXED: dropped FE no longer merges -> TWO frames")
    check(int(dut.sts_last_frame_lines.value) == 8,
          "S2 FIXED: each frame is 8 lines (NOT a 16-line merge)")
    check(counter.eof_count == 2,
          "S2 FIXED: both frames emit EOF (stale frame force-closed)")
    check(int(dut.sts_frame_sync_err_cnt.value) >= 1,
          "S2 FIXED: the missed FE is still flagged as a sync error")


# ---------------------------------------------------------------------------
# Scenario 3: EARLY FE + orphan lines + clean recovery
# FS, 4 lines, FE (early), 4 orphan lines (no frame open), FS, 4, FE.
# ---------------------------------------------------------------------------
@cocotb.test(timeout_time=1, timeout_unit="ms")
async def s3_early_fe_orphans_recover(dut):
    clk, _ = await _bringup(dut)

    # ---- S3a: early FE -> 4-line frame ----
    await drive_short(dut, clk, 0x00)             # FS (frame A)
    for i in range(4):
        await drive_lsle_line(dut, clk, i)
    await drive_short(dut, clk, 0x01)             # FE (early -> 4-line frame)
    await settle(clk)

    dut._log.info(
        "[S3a earlyFE] frames=%d last_lines=%d sync_err=%d"
        % (int(dut.sts_frame_count.value),
           int(dut.sts_last_frame_lines.value),
           int(dut.sts_frame_sync_err_cnt.value))
    )
    check(int(dut.sts_last_frame_lines.value) == 4, "S3a early FE -> 4-line frame")

    prev_frames = int(dut.sts_frame_count.value)
    prev_sync = int(dut.sts_frame_sync_err_cnt.value)

    # ---- S3b: 4 orphan lines while ST_IDLE (no FS) -> long packets rejected ----
    for i in range(4):
        await drive_lsle_line(dut, clk, 100 + i)
    await settle(clk)

    dut._log.info(
        "[S3b orphan]  frames=%d sync_err=%d (delta_sync=%d)"
        % (int(dut.sts_frame_count.value),
           int(dut.sts_frame_sync_err_cnt.value),
           int(dut.sts_frame_sync_err_cnt.value) - prev_sync)
    )
    check(int(dut.sts_frame_count.value) == prev_frames,
          "S3b orphan lines did NOT create a frame")
    check(int(dut.sts_frame_sync_err_cnt.value) > prev_sync,
          "S3b orphan long packets raised sync_err")

    # ---- S3c: clean recovery ----
    await drive_short(dut, clk, 0x00)             # FS (frame B, clean recovery)
    for i in range(4):
        await drive_lsle_line(dut, clk, i)
    await drive_short(dut, clk, 0x01)             # FE
    await settle(clk)

    dut._log.info(
        "[S3c recover] frames=%d last_lines=%d"
        % (int(dut.sts_frame_count.value),
           int(dut.sts_last_frame_lines.value))
    )
    check(int(dut.sts_last_frame_lines.value) == 4,
          "S3c clean 4-line frame after orphans")


def test_csi2_frame_state_irregular():
    from runner_support import build_and_test

    build_and_test(
        block="csi2_frame_state_irregular",
        sources=["rtl/mipi_rx/csi2_frame_state.sv"],
        toplevel="csi2_frame_state",
        test_module="test_csi2_frame_state_irregular",
        test_dir=Path(__file__).resolve().parent,
        parameters={"MAX_LINES": 4096},
    )
