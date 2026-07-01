"""cocotb port of verification/tb/tb_axis_video_bridge.sv (true AXI4-Stream + dual-clock CDC).

The bridge takes a valid-only pixel stream on ``core_clk`` and emits AXI4-Stream on ``aclk``
through a Gray-code FIFO. Two scenarios: (1) pixel-order + marker mapping (sof->tuser[0],
eol->tlast, err->tuser[1]); (2) back-pressure (tready held low holds data and bumps the
back-pressure counter, then drains in order). Input driven on core_clk (#5); output captured
on aclk (#7) -- the two-clock template for the CDC family.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import ClockCycles, RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.axis import AxisMonitor  # noqa: E402
from lib.clkreset import bringup_dual  # noqa: E402
from lib.pixel_stream import PixelStreamDriver  # noqa: E402
from lib.scoreboard import check  # noqa: E402


def _pixel_driver(dut, core_clk):
    return PixelStreamDriver(
        dut, core_clk, pixel="in_pixel", valid="in_pixel_valid",
        sof="in_pixel_sof", eol="in_pixel_eol", eof="in_pixel_eof", err="in_pixel_err")


async def _bringup(dut):
    dut.in_pixel.value = 0
    dut.in_pixel_valid.value = 0
    dut.in_pixel_sof.value = 0
    dut.in_pixel_eol.value = 0
    dut.in_pixel_eof.value = 0
    dut.in_pixel_err.value = 0
    dut.m_axis_tready.value = 1
    (core_clk, _), (aclk, _) = await bringup_dual(
        dut, "core_clk", "core_aresetn", "aclk", "aresetn",
        period_a_ns=10.0, period_b_ns=14.0)
    return core_clk, aclk


async def wait_accepted(clk, mon, count, cycles=300):
    for _ in range(cycles):
        await RisingEdge(clk)
        if len(mon.beats) >= count:
            return
    raise AssertionError("CHECK FAILED: timed out waiting for accepted AXIS data")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def pixel_order_and_markers(dut):
    core_clk, aclk = await _bringup(dut)
    mon = AxisMonitor(dut, aclk, prefix="m_axis")
    mon.start()
    drv = _pixel_driver(dut, core_clk)

    await drv.send(0x0010, sof=1)
    await drv.send(0x0011)
    await drv.send(0x0012, eol=1, eof=1, err=1)
    await wait_accepted(aclk, mon, 3)

    check(mon.beats[0]["data"] == 0x0010, "pixel 0 order")
    check(mon.beats[1]["data"] == 0x0011, "pixel 1 order")
    check(mon.beats[2]["data"] == 0x0012, "pixel 2 order")
    check(mon.beats[0]["user"] & 0x1 == 0x1, "SOF maps to tuser[0]")
    check(mon.beats[2]["last"] == 1, "EOL maps to tlast")
    check((mon.beats[2]["user"] >> 1) & 0x1 == 0x1, "err maps to tuser[1]")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def back_pressure(dut):
    core_clk, aclk = await _bringup(dut)
    mon = AxisMonitor(dut, aclk, prefix="m_axis")
    mon.start()
    drv = _pixel_driver(dut, core_clk)

    dut.m_axis_tready.value = 0
    await drv.send(0x0020, sof=1)
    await drv.send(0x0021, eol=1)
    await ClockCycles(aclk, 8)
    check(len(mon.beats) == 0, "back-pressure holds data")
    check(int(dut.sts_back_pressure_cnt.value) != 0, "back-pressure counter increments")

    dut.m_axis_tready.value = 1
    await wait_accepted(aclk, mon, 2)
    check(mon.beats[0]["data"] == 0x0020, "held pixel 0 order")
    check(mon.beats[1]["data"] == 0x0021, "held pixel 1 order")
    check(mon.beats[1]["last"] == 1, "held tlast order")


def test_axis_video_bridge():
    from runner_support import build_and_test

    build_and_test(
        block="axis_video_bridge",
        sources=["rtl/mipi_rx/axis_video_bridge.sv"],
        toplevel="axis_video_bridge",
        test_module="test_axis_video_bridge",
        test_dir=Path(__file__).resolve().parent,
        parameters={"TDATA_WIDTH": 16, "TUSER_WIDTH": 2, "FIFO_DEPTH": 16,
                    "AXIS_TUSER_ERR_DEBUG": 1},
    )
