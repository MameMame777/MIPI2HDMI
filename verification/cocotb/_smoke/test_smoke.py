"""Phase-0 proof-of-life: clock + active-low sync reset + one flop under Verilator.

Green here means the whole native-Windows cocotb+Verilator chain works: the perl
``verilator`` wrapper (WA#2), the ``make`` shim (WA#3), the static VPI link (WA#5/#6),
and value get/set across the VPI boundary. Run via::

    python verification/cocotb/runner.py smoke
    pytest verification/cocotb/_smoke
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge, ClockCycles

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import bringup  # noqa: E402


@cocotb.test()
async def flop_follows_d(dut):
    await bringup(dut, clk="clk", rst="rst_n")
    dut.d.value = 1
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    assert int(dut.q.value) == 1, "q should follow d=1"
    dut.d.value = 0
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    assert int(dut.q.value) == 0, "q should follow d=0"


@cocotb.test()
async def reset_clears_q(dut):
    await bringup(dut, clk="clk", rst="rst_n")
    dut.d.value = 1
    await ClockCycles(dut.clk, 2)
    dut.rst_n.value = 0
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    assert int(dut.q.value) == 0, "sync reset should clear q"


def test_smoke():
    """pytest / runner entry point: build the DUT under Verilator and run the tests."""
    from runner_support import build_and_test

    build_and_test(
        block="smoke",
        sources=["verification/cocotb/_smoke/smoke.sv"],
        toplevel="smoke",
        test_module="test_smoke",
        test_dir=Path(__file__).resolve().parent,
        parameters={},
    )
