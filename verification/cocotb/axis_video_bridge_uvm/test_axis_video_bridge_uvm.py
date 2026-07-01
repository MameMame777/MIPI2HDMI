"""pyuvm demonstrator for axis_video_bridge (the plain test_axis_video_bridge.py stays).

Shows the full real-UVM stack on the project's pyuvm base (lib/uvm): a uvm_test builds a
uvm_env holding a pixel-input agent (sequencer+driver) and a passive AXIS-output agent
(monitor), plus a Scoreboard; a uvm_sequence drives 3 pixels; the scoreboard compares the
observed AXIS beats (data + tlast + tuser) against the expected stream live and asserts in
check_phase. The bring-up/monitor/wait/compare boilerplate collapses into the base.

DUT marker mapping (axis_video_bridge): tuser[0]=sof, tuser[1]=err (AXIS_TUSER_ERR_DEBUG),
tlast=eol -- so a full AxisItem(data,last,user) compare covers data + all markers.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
import pyuvm
from cocotb.triggers import RisingEdge
from pyuvm import ConfigDB

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.uvm import (  # noqa: E402
    AxisItem, AxisOutputAgent, ItemsSequence, PixelInputAgent, PixelItem, Scoreboard, UvmEnv,
    UvmTest,
)


class BridgeEnv(UvmEnv):
    def build_phase(self):
        self.pixel_agent = PixelInputAgent("pixel_agent", self)
        self.axis_agent = AxisOutputAgent("axis_agent", self)
        # expected AXIS output for the 3 driven pixels (data, tlast=eol, tuser={err,sof}):
        expected = [
            AxisItem(data=0x0010, last=0, user=0b01),   # sof -> tuser[0]
            AxisItem(data=0x0011, last=0, user=0b00),
            AxisItem(data=0x0012, last=1, user=0b10),   # eol -> tlast, err -> tuser[1]
        ]
        self.sb = Scoreboard("sb", self, expected=expected)

    def connect_phase(self):
        self.axis_agent.monitor.ap.connect(self.sb.analysis_export)


@pyuvm.test()
class AxisVideoBridgeUvmTest(UvmTest):
    clock_specs = [("core_clk", "core_aresetn", 10.0), ("aclk", "aresetn", 14.0)]
    stagger = True

    def build_phase(self):
        ConfigDB().set(None, "*", "pixel_in_cfg", {
            "clk": "core_clk", "pixel": "in_pixel", "valid": "in_pixel_valid",
            "sof": "in_pixel_sof", "eol": "in_pixel_eol", "eof": "in_pixel_eof",
            "err": "in_pixel_err"})
        ConfigDB().set(None, "*", "axis_out_cfg", {"clk": "aclk", "prefix": "m_axis"})
        self.env = BridgeEnv("env", self)

    async def stimulus(self):
        dut = cocotb.top
        aclk = self.clock_pairs[1][0]
        dut.m_axis_tready.value = 1
        seq = ItemsSequence("pix_seq", items=[
            PixelItem(pixel=0x0010, sof=1),
            PixelItem(pixel=0x0011),
            PixelItem(pixel=0x0012, eol=1, eof=1, err=1),
        ])
        await seq.start(self.env.pixel_agent.seqr)
        for _ in range(400):
            await RisingEdge(aclk)
            if self.env.sb.matched >= 3:
                break


def test_axis_video_bridge_uvm():
    from runner_support import build_and_test

    build_and_test(
        block="axis_video_bridge_uvm",
        sources=["rtl/mipi_rx/axis_video_bridge.sv"],
        toplevel="axis_video_bridge",
        test_module="test_axis_video_bridge_uvm",
        test_dir=Path(__file__).resolve().parent,
        parameters={"TDATA_WIDTH": 16, "TUSER_WIDTH": 2, "FIFO_DEPTH": 16,
                    "AXIS_TUSER_ERR_DEBUG": 1},
    )
