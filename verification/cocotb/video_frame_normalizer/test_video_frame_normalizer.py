"""cocotb port of verification/tb/tb_video_frame_normalizer.sv (valid-only pixel family).

The DUT forces a variable-geometry pixel stream (short/long lines, short/long frames)
to EXACTLY OUT_LINES x OUT_PIXELS per frame: short lines are padded with FILL, long lines
truncated, short frames padded with FILL lines, long frames truncated. The TB drives 6
frame geometries (A..F) and after each checks that the output frame is always 4x4 with
one SOF and one EOF, and that every output line carries exactly OUT_PIXELS pixels.

This port replicates the TB 1:1:
  * DUT params OUT_LINES=4, OUT_PIXELS=4, FILL=0xEE, NORMALIZE=1
  * the drive cadence mirrors drv_line/drv_frame: pixels are driven back-to-back within a
    line (valid stays high), then valid drops for a 3-cycle inter-line gap. sof on the
    first pixel of a frame, eol on the last pixel of each line, eof on the last pixel of
    the frame.
  * a scoreboard coroutine mirrors the TB's always_ff, counting output px/lines/sof/eof
    and per-line pixel counts.
  * settle() = 60 idle clocks after each frame; the #2ms watchdog -> @cocotb.test timeout.

Reset is active-low synchronous (aresetn); the DUT samples on posedge clk.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import ClockCycles, RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import bringup  # noqa: E402
from lib.scoreboard import check  # noqa: E402

OL = 4  # OUT_LINES
OP = 4  # OUT_PIXELS


class Scoreboard:
    """Mirrors the TB always_ff scoreboard for one output frame."""

    def __init__(self, dut, clk):
        self.dut = dut
        self.clk = clk
        self.reset_sb()
        self.line_px = [0] * 64
        self._task = None

    def reset_sb(self):
        self.o_px = 0
        self.o_lines = 0
        self.o_px_this_line = 0
        self.sof_seen = 0
        self.eof_seen = 0

    def start(self):
        self._task = cocotb.start_soon(self._run())
        return self._task

    async def _run(self):
        dut = self.dut
        while True:
            await RisingEdge(self.clk)
            if int(dut.aresetn.value) == 0:
                # reset clears the running counters (matches TB reset branch)
                self.o_px = 0
                self.o_lines = 0
                self.o_px_this_line = 0
                self.sof_seen = 0
                self.eof_seen = 0
                continue
            if int(dut.out_valid.value) != 1:
                continue
            if int(dut.out_sof.value) == 1:
                self.sof_seen += 1
            self.o_px += 1
            self.o_px_this_line += 1
            if int(dut.out_eol.value) == 1:
                if self.o_lines < 64:
                    self.line_px[self.o_lines] = self.o_px_this_line
                self.o_lines += 1
                self.o_px_this_line = 0
            if int(dut.out_eof.value) == 1:
                self.eof_seen += 1


class Driver:
    """Mirrors the TB drv_line / drv_frame tasks (back-to-back pixels within a line)."""

    def __init__(self, dut, clk):
        self.dut = dut
        self.clk = clk

    async def idle(self):
        d = self.dut
        d.in_valid.value = 0
        d.in_data.value = 0
        d.in_sof.value = 0
        d.in_eol.value = 0
        d.in_eof.value = 0
        d.in_err.value = 0

    async def drv_line(self, npx, val, eof):
        d = self.dut
        for i in range(npx):
            await RisingEdge(self.clk)
            d.in_valid.value = 1
            d.in_data.value = val & 0xFF
            d.in_sof.value = 0
            d.in_eol.value = 1 if (i == npx - 1) else 0
            d.in_eof.value = 1 if (eof and i == npx - 1) else 0
        await RisingEdge(self.clk)
        d.in_valid.value = 0
        d.in_eol.value = 0
        d.in_eof.value = 0
        # inter-line gap
        await ClockCycles(self.clk, 3)

    async def drv_frame(self, nlines, npx):
        d = self.dut
        # first pixel of frame carries sof
        await RisingEdge(self.clk)
        d.in_valid.value = 1
        d.in_data.value = 0x10
        d.in_sof.value = 1
        d.in_eol.value = 1 if (npx == 1) else 0
        d.in_eof.value = 0
        for i in range(1, npx):
            await RisingEdge(self.clk)
            d.in_valid.value = 1
            d.in_sof.value = 0
            d.in_data.value = 0x10
            d.in_eol.value = 1 if (i == npx - 1) else 0
        await RisingEdge(self.clk)
        d.in_valid.value = 0
        d.in_sof.value = 0
        d.in_eol.value = 0
        await ClockCycles(self.clk, 3)
        for k in range(1, nlines):
            await self.drv_line(npx, (0x10 + k) & 0xFF, (k == nlines - 1))


async def settle(clk):
    await ClockCycles(clk, 60)


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def normalize_all_geometries(dut):
    clk, rst = await bringup(dut, clk="clk", rst="aresetn", cycles=8, post=4)
    drv = Driver(dut, clk)
    await drv.idle()
    sb = Scoreboard(dut, clk)
    sb.start()

    # A: exact frame 4 lines x 4 px
    sb.reset_sb()
    await drv.drv_frame(4, 4)
    await settle(clk)
    dut._log.info(
        "[A exact 4x4] sof=%d lines=%d px=%d eof=%d",
        sb.sof_seen, sb.o_lines, sb.o_px, sb.eof_seen)
    check(sb.sof_seen == 1, "A sof==1")
    check(sb.o_lines == OL, "A lines==4")
    check(sb.o_px == OL * OP, "A px==16")
    check(sb.eof_seen == 1, "A eof==1")
    for k in range(OL):
        check(sb.line_px[k] == OP, "A each line 4px")

    # B: short lines (2 px) -> pad to 4 ; 4 lines
    sb.reset_sb()
    await drv.drv_frame(4, 2)
    await settle(clk)
    dut._log.info(
        "[B short-line 4x2] sof=%d lines=%d px=%d",
        sb.sof_seen, sb.o_lines, sb.o_px)
    check(sb.o_lines == OL, "B lines==4")
    check(sb.o_px == OL * OP, "B px==16")
    check(sb.sof_seen == 1, "B sof==1")
    for k in range(OL):
        check(sb.line_px[k] == OP, "B each padded to 4px")

    # C: long lines (6 px) -> truncate to 4 ; 4 lines
    sb.reset_sb()
    await drv.drv_frame(4, 6)
    await settle(clk)
    dut._log.info("[C long-line 4x6] lines=%d px=%d", sb.o_lines, sb.o_px)
    check(sb.o_lines == OL, "C lines==4")
    check(sb.o_px == OL * OP, "C px==16")
    for k in range(OL):
        check(sb.line_px[k] == OP, "C each truncated to 4px")

    # D: short frame (2 lines) -> pad to 4 lines
    sb.reset_sb()
    await drv.drv_frame(2, 4)
    await settle(clk)
    dut._log.info(
        "[D short-frame 2x4] lines=%d px=%d eof=%d",
        sb.o_lines, sb.o_px, sb.eof_seen)
    check(sb.o_lines == OL, "D lines padded to 4")
    check(sb.o_px == OL * OP, "D px==16")
    check(sb.eof_seen == 1, "D eof==1")

    # E: long frame (7 lines) -> truncate to 4 lines
    sb.reset_sb()
    await drv.drv_frame(7, 4)
    await settle(clk)
    dut._log.info(
        "[E long-frame 7x4] lines=%d px=%d eof=%d",
        sb.o_lines, sb.o_px, sb.eof_seen)
    check(sb.o_lines == OL, "E lines truncated to 4")
    check(sb.o_px == OL * OP, "E px==16")
    check(sb.eof_seen == 1, "E eof==1")

    # F: mismatched both (3 lines x 5 px) -> 4x4
    sb.reset_sb()
    await drv.drv_frame(3, 5)
    await settle(clk)
    dut._log.info("[F 3x5] lines=%d px=%d", sb.o_lines, sb.o_px)
    check(sb.o_lines == OL, "F lines==4")
    check(sb.o_px == OL * OP, "F px==16")


def test_video_frame_normalizer():
    from runner_support import build_and_test

    build_and_test(
        block="video_frame_normalizer",
        sources=["rtl/img_proc/video_frame_normalizer.sv"],
        toplevel="video_frame_normalizer",
        test_module="test_video_frame_normalizer",
        test_dir=Path(__file__).resolve().parent,
        parameters={"OUT_LINES": OL, "OUT_PIXELS": OP, "FILL": 0xEE, "NORMALIZE": 1},
        engine="verilator",
    )
