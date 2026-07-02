"""Project pyuvm base -- a real-UVM (pyuvm 4.0.1) verification layer for the cocotb tests.

Additive to the plain ``lib`` drivers/monitors (which the 53 tests use directly): the pyuvm
drivers here REUSE that plain driving logic by composition. New tests opt into UVM via::

    from lib.uvm import UvmTest, UvmEnv, PixelInputAgent, AxisOutputAgent, Scoreboard, \
        AxisItem, PixelItem, ItemsSequence

See verification/cocotb/axis_video_bridge/test_axis_video_bridge_uvm.py for a worked example,
and skills/cocotb-verilator/references/porting-patterns.md for the recipe.
"""
from lib.uvm.agents import (  # noqa: F401
    AxisInputAgent, AxisOutputAgent, ByteBeatInputAgent, FrameInputAgent, PixelInputAgent,
    PixelOutputAgent,
)
from lib.uvm.env import UvmEnv, UvmTest  # noqa: F401
from lib.uvm.interfaces import (  # noqa: F401
    AxisMonitor, AxisSourceDriver, ByteBeatDriver, FramePixelDriver, PixelDriver,
    PixelMonitor,
)
from lib.uvm.items import AxisItem, ByteBeatItem, ImageFrameItem, PixelItem  # noqa: F401
from lib.uvm.scoreboard import FrameScoreboard, Scoreboard  # noqa: F401
from lib.uvm.sequences import ItemsSequence  # noqa: F401

__all__ = [
    "UvmEnv", "UvmTest",
    "PixelInputAgent", "ByteBeatInputAgent", "AxisInputAgent", "FrameInputAgent",
    "AxisOutputAgent", "PixelOutputAgent",
    "PixelDriver", "ByteBeatDriver", "AxisSourceDriver", "FramePixelDriver",
    "AxisMonitor", "PixelMonitor",
    "ByteBeatItem", "PixelItem", "AxisItem", "ImageFrameItem",
    "Scoreboard", "FrameScoreboard", "ItemsSequence",
]
