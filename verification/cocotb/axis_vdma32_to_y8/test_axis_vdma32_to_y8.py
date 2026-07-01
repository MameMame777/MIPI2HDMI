"""cocotb port of verification/tb/tb_axis_vdma32_to_y8.sv (32-bit AXIS -> 8-bit AXIS width converter).

The DUT buffers one 32-bit slave beat (``s_axis_tready = !beat_valid``) and emits its four
bytes LSB-first on the 8-bit master stream. ``m_axis_tlast`` asserts only on byte 3 when the
buffered beat carried ``tlast``; ``m_axis_tuser[0]`` asserts only on byte 0 when the buffered
beat carried ``tuser[0]``.

This is a true AXI4-Stream block (``*_t{valid,ready,data,last,user}``) driven single-clock.
The TB uses precise cycle-based ``send_beat``/``expect_byte`` tasks (including a check that
``s_tready`` deasserts while a beat is pending), so the port drives the signals directly to
replicate that cadence 1:1 rather than routing through the generic lib source/monitor.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import start_clock  # noqa: E402
from lib.scoreboard import check  # noqa: E402


async def send_beat(dut, clk, data: int, last: int, user: int) -> None:
    """Mirror tb send_beat: wait for s_tready, drive one beat for one cycle, then idle."""
    await RisingEdge(clk)
    while int(dut.s_axis_tready.value) != 1:
        await RisingEdge(clk)
    dut.s_axis_tdata.value = data
    dut.s_axis_tlast.value = last
    dut.s_axis_tuser.value = user
    dut.s_axis_tvalid.value = 1
    await RisingEdge(clk)
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tlast.value = 0
    dut.s_axis_tuser.value = 0


async def expect_byte(dut, clk, data: int, last: int, user: int) -> None:
    """Mirror tb expect_byte: wait for m_tvalid, check data/last/user, pulse m_tready once."""
    await RisingEdge(clk)
    while int(dut.m_axis_tvalid.value) != 1:
        await RisingEdge(clk)
    check(int(dut.m_axis_tdata.value) == data,
          f"TDATA expected {data:02x} got {int(dut.m_axis_tdata.value):02x}")
    check(int(dut.m_axis_tlast.value) == last,
          f"TLAST expected {last} got {int(dut.m_axis_tlast.value)}")
    check(int(dut.m_axis_tuser.value) & 0x1 == user,
          f"TUSER expected {user} got {int(dut.m_axis_tuser.value) & 0x1}")
    dut.m_axis_tready.value = 1
    await RisingEdge(clk)
    dut.m_axis_tready.value = 0


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def width_convert(dut):
    clk = dut.aclk
    start_clock(clk, period_ns=10.0)

    # Initial values (mirror the TB initial block).
    dut.s_axis_tdata.value = 0x00000000
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tlast.value = 0
    dut.s_axis_tuser.value = 0
    dut.m_axis_tready.value = 0
    dut.aresetn.value = 0

    # repeat(4) @(posedge clk); rstn<=1; repeat(2) @(posedge clk);
    for _ in range(4):
        await RisingEdge(clk)
    dut.aresetn.value = 1
    for _ in range(2):
        await RisingEdge(clk)

    # First 32-bit beat: 0x44332211, tlast=0, tuser=1.
    await send_beat(dut, clk, 0x44332211, 0, 1)
    # @(posedge clk); assert !s_tready (a beat is pending -> input must be back-pressured).
    await RisingEdge(clk)
    check(int(dut.s_axis_tready.value) == 0,
          "Input was ready while the first beat was pending")
    # Bytes emitted LSB-first; tuser[0] only on byte 0.
    await expect_byte(dut, clk, 0x11, 0, 1)
    await expect_byte(dut, clk, 0x22, 0, 0)
    await expect_byte(dut, clk, 0x33, 0, 0)
    await expect_byte(dut, clk, 0x44, 0, 0)

    # Second 32-bit beat: 0x88776655, tlast=1, tuser=0. tlast only on byte 3.
    await send_beat(dut, clk, 0x88776655, 1, 0)
    await expect_byte(dut, clk, 0x55, 0, 0)
    await expect_byte(dut, clk, 0x66, 0, 0)
    await expect_byte(dut, clk, 0x77, 0, 0)
    await expect_byte(dut, clk, 0x88, 1, 0)


def test_axis_vdma32_to_y8():
    from runner_support import build_and_test

    build_and_test(
        block="axis_vdma32_to_y8",
        sources=["rtl/img_proc/axis_vdma32_to_y8.sv"],
        toplevel="axis_vdma32_to_y8",
        test_module="test_axis_vdma32_to_y8",
        test_dir=Path(__file__).resolve().parent,
        parameters={},
        engine="verilator",
    )
