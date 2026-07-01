"""Phase-0 pyuvm compatibility spike: a minimal uvm_test on the 1-flop smoke DUT.

Proves pyuvm 4.0.1 works through THIS project's native-Windows Verilator runner:
``@pyuvm.test()`` registers as a cocotb test that the cocotb-2.0 runner discovers via
``COCOTB_TEST_MODULES``; ``cocotb.top`` reaches the DUT (built ``--public-flat-rw``);
``run_phase`` + objections drive the sim; the sitecustomize DLL fix + ``-Wno-attributes``
all cooperate. If this is green, the full pyuvm base (lib/uvm/) is viable.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
import pyuvm
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge
from pyuvm import uvm_test

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


@pyuvm.test()
class SmokeUvmTest(uvm_test):
    """No env/agents yet -- just prove the pyuvm test lifecycle runs on the DUT."""

    async def run_phase(self):
        self.raise_objection()
        dut = cocotb.top
        cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
        dut.d.value = 0
        dut.rst_n.value = 0
        await ClockCycles(dut.clk, 4)
        dut.rst_n.value = 1
        await ClockCycles(dut.clk, 2)

        dut.d.value = 1
        await RisingEdge(dut.clk)
        await RisingEdge(dut.clk)
        assert int(dut.q.value) == 1, "CHECK FAILED: q should follow d=1"

        dut.d.value = 0
        await RisingEdge(dut.clk)
        await RisingEdge(dut.clk)
        assert int(dut.q.value) == 0, "CHECK FAILED: q should follow d=0"

        self.drop_objection()


def test_smoke_uvm():
    from runner_support import build_and_test

    build_and_test(
        block="smoke_uvm",
        sources=["verification/cocotb/_smoke/smoke.sv"],
        toplevel="smoke",
        test_module="test_smoke_uvm",
        test_dir=Path(__file__).resolve().parent,
        parameters={},
    )
