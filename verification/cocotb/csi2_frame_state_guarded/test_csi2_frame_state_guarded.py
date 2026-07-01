"""cocotb port of verification/tb/tb_csi2_frame_state_guarded.sv.

DUT: rtl/mipi_rx/csi2_frame_state.sv instantiated in GUARD_FRAME_LINES / non-lsle mode
(MAX_LINES=512, GUARD_FRAME_LINES=1, EXPECTED_FRAME_LINES=480, EXPECTED_LINE_WC=1,
cfg_use_lsle=0). This is the "guarded" line-count delimiter: a frame is opened by an FS
short packet, each long packet is a line, and the frame closes when the
EXPECTED_FRAME_LINES-th line's long-end arrives. Early FE, overlapping in-frame FS, an FE
without a preceding FS, and a bad-word-count line are all rejected (counted as sync errors)
without corrupting the 480-line frame.

The block's interface (in_pkt_* / in_payload_*) matches none of the three lib driver
families, so this port drives the raw signals with a small driver that reproduces the SV
tasks (drive_short / start_long / drive_payload / end_long) edge-for-edge. The TB's
always_ff counter logger becomes the CounterLog monitor coroutine; check_condition ->
check(); the 5 initial-block cases become 5 @cocotb.test()s (each fresh-reset), which
matches the TB's reset_dut()+clear_logs() at the head of every case.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import start_clock  # noqa: E402
from lib.scoreboard import check  # noqa: E402

EXPECTED_LINES = 480
GOOD_WC = 1
BAD_WC = 2


class CounterLog:
    """Mirror of the TB always_ff @(posedge core_clk) logger.

    Samples the DUT outputs each rising edge and accumulates the same counts the SV TB
    tracks, honouring the reset / clear_logs_pulse zeroing semantics.
    """

    def __init__(self, dut):
        self.dut = dut
        self.clear = False
        self.reset()

    def reset(self):
        self.sof_count = 0
        self.eof_count = 0
        self.sol_count = 0
        self.eol_count = 0
        self.payload_count = 0
        self.frame_err_count = 0

    def start(self, clk):
        return cocotb.start_soon(self._run(clk))

    async def _run(self, clk):
        d = self.dut
        while True:
            await RisingEdge(clk)
            # The SV always_ff samples the pre-edge values; RisingEdge fires just after the
            # edge, so the settled post-edge registered outputs are what the TB counts.
            if int(d.core_aresetn.value) == 0 or self.clear:
                self.reset()
                continue
            if int(d.out_sof.value) == 1:
                self.sof_count += 1
            if int(d.out_eof.value) == 1:
                self.eof_count += 1
            if int(d.out_sol.value) == 1:
                self.sol_count += 1
            if int(d.out_eol.value) == 1:
                self.eol_count += 1
            if int(d.out_frame_err.value) == 1:
                self.frame_err_count += 1
            if int(d.out_payload_valid.value) == 1:
                self.payload_count += 1


class Driver:
    """Reproduces the SV stimulus tasks edge-for-edge on the raw pkt/payload inputs."""

    def __init__(self, dut, clk):
        self.dut = dut
        self.clk = clk

    def _idle_inputs(self):
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

    async def reset_dut(self):
        d = self.dut
        d.core_aresetn.value = 0
        d.cfg_use_lsle.value = 0
        self._idle_inputs()
        for _ in range(8):
            await RisingEdge(self.clk)
        d.core_aresetn.value = 1
        for _ in range(2):
            await RisingEdge(self.clk)

    async def drive_short(self, dt, err):
        d = self.dut
        await RisingEdge(self.clk)
        d.in_pkt_di.value = dt & 0x3F  # {2'b00, dt}
        d.in_pkt_wc.value = 0
        d.in_pkt_is_short.value = 1
        d.in_pkt_is_long.value = 0
        d.in_pkt_err.value = 1 if err else 0
        d.in_pkt_start.value = 1
        d.in_pkt_end.value = 1
        await RisingEdge(self.clk)
        d.in_pkt_start.value = 0
        d.in_pkt_end.value = 0
        d.in_pkt_err.value = 0
        d.in_pkt_is_short.value = 0

    async def start_long(self, wc):
        d = self.dut
        await RisingEdge(self.clk)
        d.in_pkt_di.value = 0x1E
        d.in_pkt_wc.value = wc & 0xFFFF
        d.in_pkt_is_short.value = 0
        d.in_pkt_is_long.value = 1
        d.in_pkt_start.value = 1
        await RisingEdge(self.clk)
        d.in_pkt_start.value = 0

    async def drive_payload(self, data):
        d = self.dut
        await RisingEdge(self.clk)
        d.in_payload_data.value = data & 0xFF
        d.in_payload_first.value = 1
        d.in_payload_last.value = 1
        d.in_payload_valid.value = 1
        await RisingEdge(self.clk)
        d.in_payload_valid.value = 0
        d.in_payload_first.value = 0
        d.in_payload_last.value = 0

    async def end_long(self, err):
        d = self.dut
        await RisingEdge(self.clk)
        d.in_pkt_err.value = 1 if err else 0
        d.in_pkt_end.value = 1
        await RisingEdge(self.clk)
        d.in_pkt_end.value = 0
        d.in_pkt_err.value = 0
        d.in_pkt_is_long.value = 0

    async def drive_one_byte_line(self, wc, data):
        await self.start_long(wc)
        await self.drive_payload(data)
        await self.end_long(False)

    async def drive_good_lines(self, line_count, start_line):
        for i in range(line_count):
            await self.drive_one_byte_line(GOOD_WC, (start_line + i) & 0xFF)


async def clear_logs(clk, log: CounterLog):
    # Mirror the SV clear_logs task: pulse clear_logs_pulse for one cycle.
    await RisingEdge(clk)
    log.clear = True
    await RisingEdge(clk)   # this edge sees clear=1 -> reset()
    log.clear = False
    await RisingEdge(clk)


async def setup(dut):
    clk = dut.core_clk
    start_clock(clk, 10.0)
    # Non-parameterised DUT config held at the TB's constant values.
    dut.cfg_expected_frame_lines.value = 0
    dut.cfg_sof_synth.value = 0
    dut.cfg_force_expected.value = 0
    dut.cfg_long_as_line.value = 0
    log = CounterLog(dut)
    log.start(clk)
    drv = Driver(dut, clk)
    await drv.reset_dut()
    await clear_logs(clk, log)
    return clk, drv, log


async def check_completed_frame(dut, clk, log, expected_sync_errors, case_name):
    for _ in range(6):
        await RisingEdge(clk)
    check(int(dut.sts_frame_count.value) == 1, f"{case_name} frame count")
    check(int(dut.sts_line_count.value) == EXPECTED_LINES, f"{case_name} line count")
    check(int(dut.sts_last_frame_lines.value) == EXPECTED_LINES,
          f"{case_name} last frame lines")
    check(log.payload_count == EXPECTED_LINES, f"{case_name} payload count")
    check(log.eol_count == EXPECTED_LINES, f"{case_name} EOL count")
    check(log.eof_count == 1, f"{case_name} EOF count")
    check(log.frame_err_count == 0, f"{case_name} frame error count")
    check(int(dut.sts_frame_sync_err_cnt.value) == expected_sync_errors,
          f"{case_name} sync error count")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def clean_480_line_frame(dut):
    clk, drv, log = await setup(dut)
    await drv.drive_short(0x00, False)          # FS
    await drv.drive_good_lines(EXPECTED_LINES, 0)
    await drv.drive_short(0x01, False)          # FE
    await check_completed_frame(dut, clk, log, 0, "clean 480-line frame")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def early_fe_ignored_frame(dut):
    clk, drv, log = await setup(dut)
    await drv.drive_short(0x00, False)          # FS
    await drv.drive_good_lines(392, 0)
    await drv.drive_short(0x01, False)          # early FE (line_idx < 480) -> ignored
    for _ in range(4):
        await RisingEdge(clk)
    check(int(dut.sts_frame_count.value) == 0, "early FE does not end frame")
    await drv.drive_good_lines(88, 392)
    await drv.drive_short(0x01, False)          # FE at line 480 -> closes
    await check_completed_frame(dut, clk, log, 1, "early FE ignored frame")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def overlap_fs_ignored_frame(dut):
    clk, drv, log = await setup(dut)
    await drv.drive_short(0x00, False)          # FS
    await drv.drive_good_lines(100, 0)
    await drv.drive_short(0x00, False)          # in-frame FS overlap -> sync err, ignored
    await drv.drive_good_lines(380, 100)
    await drv.drive_short(0x01, False)          # FE at line 480
    await check_completed_frame(dut, clk, log, 1, "overlap FS ignored frame")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def fe_without_fs_ignored_frame(dut):
    clk, drv, log = await setup(dut)
    await drv.drive_short(0x01, False)          # FE while IDLE -> sync err
    await drv.drive_short(0x00, False)          # FS -> opens frame
    await drv.drive_good_lines(EXPECTED_LINES, 0)
    await drv.drive_short(0x01, False)          # FE at line 480
    await check_completed_frame(dut, clk, log, 1, "FE without FS ignored frame")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def bad_wc_line_dropped_frame(dut):
    clk, drv, log = await setup(dut)
    await drv.drive_short(0x00, False)          # FS
    await drv.drive_one_byte_line(BAD_WC, 0xAA)  # bad WC -> line rejected, sync err
    await drv.drive_good_lines(EXPECTED_LINES, 0)
    await drv.drive_short(0x01, False)          # FE at line 480
    await check_completed_frame(dut, clk, log, 1, "bad-WC line dropped frame")


def test_csi2_frame_state_guarded():
    from runner_support import build_and_test

    build_and_test(
        block="csi2_frame_state_guarded",
        sources=["rtl/mipi_rx/csi2_frame_state.sv"],
        toplevel="csi2_frame_state",
        test_module="test_csi2_frame_state_guarded",
        test_dir=Path(__file__).resolve().parent,
        parameters={
            "MAX_LINES": 512,
            "GUARD_FRAME_LINES": 1,
            "EXPECTED_FRAME_LINES": EXPECTED_LINES,
            "EXPECTED_LINE_WC": GOOD_WC,
        },
        engine="verilator",
    )
