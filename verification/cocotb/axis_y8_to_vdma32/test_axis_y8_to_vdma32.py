"""cocotb port of verification/tb/tb_axis_y8_to_vdma32.sv (8-bit AXIS -> 32-bit VDMA packer).

The DUT accumulates up to four 8-bit slave beats LSB-first into one 32-bit master beat and
flushes when either four bytes have been packed (``pack_count==3``) or the current slave beat
carries ``s_axis_tlast``. ``m_axis_tkeep`` reflects how many bytes are valid (0001/0011/0111/
1111), ``m_axis_tlast`` mirrors the flushing beat's ``s_axis_tlast`` and ``m_axis_tuser[0]``
is the OR of every ``s_axis_tuser[0]`` folded into the beat. Back-pressure: while an output
beat is pending (``m_axis_tvalid`` and ``!m_axis_tready``) the slave is stalled
(``s_axis_tready = !m_axis_tvalid || m_axis_tready``).

This is a true AXI4-Stream block (``*_t{valid,ready,data,last,user}``) driven single-clock.
The TB uses precise cycle-based ``send_byte``/``expect_beat`` tasks (including a check that
``s_tready`` deasserts while a beat is pending) so the port drives the signals directly to
replicate that cadence 1:1 rather than routing through the generic lib source/monitor -- this
mirrors the sibling ``axis_vdma32_to_y8`` port.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import start_clock  # noqa: E402
from lib.scoreboard import check  # noqa: E402


async def send_byte(dut, clk, data: int, last: int, user: int) -> None:
    """Mirror tb send_byte: wait for s_tready, drive one accepted byte, then idle.

    The TB pre-waits for ``s_tready`` before driving, so every byte is presented only in a
    cycle where the slave is ready; the accepting edge is then a single ``@(posedge clk)``.
    """
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


async def expect_beat(dut, clk, data: int, keep: int, last: int, user: int) -> None:
    """Mirror tb expect_beat: wait for m_tvalid, check data/keep/last/user, pulse m_tready."""
    await RisingEdge(clk)
    while int(dut.m_axis_tvalid.value) != 1:
        await RisingEdge(clk)
    got_data = int(dut.m_axis_tdata.value)
    got_keep = int(dut.m_axis_tkeep.value)
    got_last = int(dut.m_axis_tlast.value)
    got_user = int(dut.m_axis_tuser.value) & 0x1
    check(got_data == data, f"TDATA expected {data:08x} got {got_data:08x}")
    check(got_keep == keep, f"TKEEP expected {keep:x} got {got_keep:x}")
    check(got_last == last, f"TLAST expected {last} got {got_last}")
    check(got_user == user, f"TUSER expected {user} got {got_user}")
    dut.m_axis_tready.value = 1
    await RisingEdge(clk)
    dut.m_axis_tready.value = 0


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def pack_y8_to_vdma32(dut):
    clk = dut.aclk
    start_clock(clk, period_ns=10.0)

    # Initial values (mirror the TB initial block).
    dut.s_axis_tdata.value = 0x00
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

    # Scenario 1: four bytes -> one full 32-bit beat, tlast on the 4th byte, tuser on the 1st.
    await send_byte(dut, clk, 0x11, 0, 1)
    await send_byte(dut, clk, 0x22, 0, 0)
    await send_byte(dut, clk, 0x33, 0, 0)
    await send_byte(dut, clk, 0x44, 1, 0)
    await expect_beat(dut, clk, 0x4433_2211, 0xF, 1, 1)

    # Scenario 2: two bytes then tlast -> partial beat flushed, tkeep=0x3, tuser=0.
    await send_byte(dut, clk, 0xAA, 0, 0)
    await send_byte(dut, clk, 0xBB, 1, 0)
    await expect_beat(dut, clk, 0x0000_BBAA, 0x3, 1, 0)

    # Scenario 3: four bytes with m_tready held low -> the full beat is pending and must
    # back-pressure the slave (m_tvalid=1 && s_tready=0), then drain to 0x04030201.
    await send_byte(dut, clk, 0x01, 0, 0)
    await send_byte(dut, clk, 0x02, 0, 0)
    await send_byte(dut, clk, 0x03, 0, 0)
    await send_byte(dut, clk, 0x04, 0, 0)
    await RisingEdge(clk)
    check(int(dut.m_axis_tvalid.value) == 1 and int(dut.s_axis_tready.value) == 0,
          "Backpressure did not hold pending output")
    await expect_beat(dut, clk, 0x0403_0201, 0xF, 0, 0)


def test_axis_y8_to_vdma32():
    from runner_support import build_and_test

    build_and_test(
        block="axis_y8_to_vdma32",
        sources=["rtl/img_proc/axis_y8_to_vdma32.sv"],
        toplevel="axis_y8_to_vdma32",
        test_module="test_axis_y8_to_vdma32",
        test_dir=Path(__file__).resolve().parent,
        parameters={},
        engine="verilator",
    )
