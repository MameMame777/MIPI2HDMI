"""cocotb port of verification/tb/tb_csi2_frame_state.sv.

The DUT (``csi2_frame_state``) has a bespoke CSI-2 packet/payload interface
(``in_pkt_*`` header pulses + ``in_payload_*`` beats), so no ``lib/`` driver
family fits it directly. The SV tasks (``drive_short``, ``start_long``,
``drive_payload``, ``end_long``, ``drive_one_byte_line``) are ported 1:1 as
coroutines that assign on RisingEdge (the posedge-driving equivalent of the
TB's ``@(posedge core_clk)`` non-blocking task bodies), and the SV
``always_ff`` logger becomes the ``Logger`` monitor coroutine. Each of the six
sequential TB scenarios (which each begin with a ``clear_logs``/``reset_dut``)
becomes its own ``@cocotb.test()`` with a fresh reset, replicating every
``check_condition`` from the original run. All scenarios use the DUT with
``MAX_LINES=512`` and the remaining parameters at their defaults, exactly as the
TB instantiates it.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import ClockCycles, RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import bringup, reset_active_low  # noqa: E402
from lib.scoreboard import check  # noqa: E402


# ---------------------------------------------------------------------------
# Logger: mirrors the SV always_ff logger (counts markers / payload, records the
# first 16 payload bytes and their line indices).
# ---------------------------------------------------------------------------
class Logger:
    def __init__(self, dut):
        self.dut = dut
        self._clear()

    def _clear(self):
        self.sof_count = 0
        self.eof_count = 0
        self.sol_count = 0
        self.eol_count = 0
        self.payload_count = 0
        self.frame_err_count = 0
        self.payload_log = [0] * 16
        self.line_idx_log = [0] * 16

    def clear(self):
        """Zero the counters/logs (the clear_logs_pulse effect)."""
        self._clear()

    def start(self, clk):
        return cocotb.start_soon(self._run(clk))

    async def _run(self, clk):
        d = self.dut
        while True:
            await RisingEdge(clk)
            if int(d.out_sof.value):
                self.sof_count += 1
            if int(d.out_eof.value):
                self.eof_count += 1
            if int(d.out_sol.value):
                self.sol_count += 1
            if int(d.out_eol.value):
                self.eol_count += 1
            if int(d.out_frame_err.value):
                self.frame_err_count += 1
            if int(d.out_payload_valid.value):
                if self.payload_count < 16:
                    self.payload_log[self.payload_count] = int(d.out_payload_data.value)
                    self.line_idx_log[self.payload_count] = int(d.out_line_idx.value)
                self.payload_count += 1


# ---------------------------------------------------------------------------
# Stimulus tasks: 1:1 ports of the SV tasks. Each waits a RisingEdge, drives the
# inputs (so they are sampled on the *next* posedge, matching the TB's
# non-blocking task bodies), then deasserts on the following edge.
# ---------------------------------------------------------------------------
def _reset_inputs(dut):
    dut.cfg_use_lsle.value = 0
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


async def drive_short(dut, clk, dt, err=0):
    await RisingEdge(clk)
    dut.in_pkt_di.value = dt & 0x3F
    dut.in_pkt_wc.value = 0
    dut.in_pkt_is_short.value = 1
    dut.in_pkt_is_long.value = 0
    dut.in_pkt_err.value = err
    dut.in_pkt_start.value = 1
    dut.in_pkt_end.value = 1
    await RisingEdge(clk)
    dut.in_pkt_start.value = 0
    dut.in_pkt_end.value = 0
    dut.in_pkt_err.value = 0
    dut.in_pkt_is_short.value = 0


async def start_long(dut, clk, dt, wc):
    await RisingEdge(clk)
    dut.in_pkt_di.value = dt & 0x3F
    dut.in_pkt_wc.value = wc
    dut.in_pkt_is_short.value = 0
    dut.in_pkt_is_long.value = 1
    dut.in_pkt_start.value = 1
    await RisingEdge(clk)
    dut.in_pkt_start.value = 0


async def drive_payload(dut, clk, data, first, last):
    await RisingEdge(clk)
    dut.in_payload_data.value = data & 0xFF
    dut.in_payload_first.value = 1 if first else 0
    dut.in_payload_last.value = 1 if last else 0
    dut.in_payload_valid.value = 1
    await RisingEdge(clk)
    dut.in_payload_valid.value = 0
    dut.in_payload_first.value = 0
    dut.in_payload_last.value = 0


async def end_long(dut, clk, err):
    await RisingEdge(clk)
    dut.in_pkt_err.value = err
    dut.in_pkt_end.value = 1
    await RisingEdge(clk)
    dut.in_pkt_end.value = 0
    dut.in_pkt_err.value = 0
    dut.in_pkt_is_long.value = 0


async def drive_one_byte_line(dut, clk, data):
    await start_long(dut, clk, 0x2A, 1)
    await drive_payload(dut, clk, data, 1, 1)
    await end_long(dut, clk, 0)


async def _setup(dut):
    _reset_inputs(dut)
    clk, _ = await bringup(dut, clk="core_clk", rst="core_aresetn")
    log = Logger(dut)
    log.start(clk)
    # Give the logger a settled post-reset baseline (matches clear_logs()).
    log.clear()
    return clk, log


# ---------------------------------------------------------------------------
# Scenario 1: clean single-line-group frame (non-lsle).
# ---------------------------------------------------------------------------
@cocotb.test(timeout_time=1, timeout_unit="ms")
async def clean_frame(dut):
    clk, log = await _setup(dut)

    log.clear()
    await drive_short(dut, clk, 0x00, 0)           # FS
    await start_long(dut, clk, 0x2A, 3)
    await drive_payload(dut, clk, 0x10, 1, 0)
    await drive_payload(dut, clk, 0x11, 0, 0)
    await drive_payload(dut, clk, 0x12, 0, 1)
    await end_long(dut, clk, 0)
    await drive_short(dut, clk, 0x01, 0)           # FE
    await ClockCycles(clk, 4)

    check(log.sof_count >= 1, "FS produces SOF")
    check(log.eof_count == 1, "FE produces EOF")
    check(log.sol_count == 1, "long packet produces SOL")
    check(log.eol_count == 1, "payload_last produces EOL")
    check(log.payload_count == 3, "payload pass count")
    check(log.payload_log[0] == 0x10, "payload byte 0")
    check(log.payload_log[2] == 0x12, "payload byte 2")
    check(log.line_idx_log[0] == 0, "first line index")
    check(int(dut.sts_frame_count.value) == 1, "frame count")
    check(int(dut.sts_line_count.value) == 1, "line count")
    check(int(dut.sts_last_frame_lines.value) == 1, "last frame lines")
    check(log.frame_err_count == 0, "clean frame has no error")


# ---------------------------------------------------------------------------
# Scenario 2: long packet without a preceding FS is dropped (sync error).
# ---------------------------------------------------------------------------
@cocotb.test(timeout_time=1, timeout_unit="ms")
async def long_without_fs_dropped(dut):
    clk, log = await _setup(dut)

    log.clear()
    await start_long(dut, clk, 0x2A, 1)
    await drive_payload(dut, clk, 0x22, 1, 1)
    await end_long(dut, clk, 0)
    await ClockCycles(clk, 3)

    check(log.payload_count == 0, "long packet without FS is dropped")
    check(int(dut.sts_frame_sync_err_cnt.value) == 1, "FS missing sync error")


# ---------------------------------------------------------------------------
# Scenario 3: a packet error at FE becomes a frame error, payload still passes.
# ---------------------------------------------------------------------------
@cocotb.test(timeout_time=1, timeout_unit="ms")
async def error_frame(dut):
    clk, log = await _setup(dut)

    log.clear()
    await drive_short(dut, clk, 0x00, 0)           # FS
    await start_long(dut, clk, 0x2A, 1)
    await drive_payload(dut, clk, 0x33, 1, 1)
    await end_long(dut, clk, 1)                    # error at long end
    await drive_short(dut, clk, 0x01, 0)           # FE
    await ClockCycles(clk, 4)

    check(log.payload_count == 1, "error frame payload still passes")
    check(log.frame_err_count == 1, "packet error becomes frame error at FE")


# ---------------------------------------------------------------------------
# Scenario 4: LS/LE markers drive SOL/EOL when cfg_use_lsle is enabled.
# ---------------------------------------------------------------------------
@cocotb.test(timeout_time=1, timeout_unit="ms")
async def lsle_markers(dut):
    clk, log = await _setup(dut)

    dut.cfg_use_lsle.value = 1
    log.clear()
    await drive_short(dut, clk, 0x00, 0)           # FS
    await drive_short(dut, clk, 0x02, 0)           # LS
    await start_long(dut, clk, 0x2A, 1)
    await drive_payload(dut, clk, 0x44, 1, 1)
    await end_long(dut, clk, 0)
    await drive_short(dut, clk, 0x03, 0)           # LE
    await drive_short(dut, clk, 0x01, 0)           # FE
    await ClockCycles(clk, 4)

    check(log.sol_count == 1, "LS produces SOL when enabled")
    check(log.eol_count == 1, "LE produces EOL when enabled")
    check(log.payload_count == 1, "LSLE payload passes")


# ---------------------------------------------------------------------------
# Scenario 5: full 480-line frame (non-lsle), one payload byte per line.
# ---------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def frame_480_lines(dut):
    clk, log = await _setup(dut)

    # reset_dut() then clear_logs() in the TB (re-uses the running clock).
    _reset_inputs(dut)
    await reset_active_low(clk, dut.core_aresetn)
    log.clear()

    await drive_short(dut, clk, 0x00, 0)           # FS
    for line_idx in range(480):
        await drive_one_byte_line(dut, clk, line_idx & 0xFF)
    await drive_short(dut, clk, 0x01, 0)           # FE
    await ClockCycles(clk, 4)

    check(int(dut.sts_frame_count.value) == 1, "480-line frame count")
    check(int(dut.sts_line_count.value) == 480, "480-line line count")
    check(int(dut.sts_last_frame_lines.value) == 480,
          "clean frame ended after 480 lines")
    check(log.payload_count == 480, "480-line payload count")
    check(log.eol_count == 480, "480-line EOL count")


# ---------------------------------------------------------------------------
# Scenario 6: early FE at 392 lines latches the observed line count.
# ---------------------------------------------------------------------------
@cocotb.test(timeout_time=20, timeout_unit="ms")
async def early_fe(dut):
    clk, log = await _setup(dut)

    _reset_inputs(dut)
    await reset_active_low(clk, dut.core_aresetn)
    log.clear()

    await drive_short(dut, clk, 0x00, 0)           # FS
    for line_idx in range(392):
        await drive_one_byte_line(dut, clk, line_idx & 0xFF)
    await drive_short(dut, clk, 0x01, 0)           # FE
    await ClockCycles(clk, 4)

    check(int(dut.sts_frame_count.value) == 1, "early-FE frame count")
    check(int(dut.sts_line_count.value) == 392, "early-FE line count")
    check(int(dut.sts_last_frame_lines.value) == 392,
          "early FE latches observed line count")


def test_csi2_frame_state():
    from runner_support import build_and_test

    build_and_test(
        block="csi2_frame_state",
        sources=["rtl/mipi_rx/csi2_frame_state.sv"],
        toplevel="csi2_frame_state",
        test_module="test_csi2_frame_state",
        test_dir=Path(__file__).resolve().parent,
        parameters={"MAX_LINES": 512},
        engine="verilator",
    )
