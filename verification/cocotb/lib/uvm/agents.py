"""pyuvm agents bundling a sequencer + driver (active) or a monitor (passive) per interface.

Active agents wire ``driver.seq_item_port -> sequencer.seq_item_export`` in connect_phase.
Config for the contained driver/monitor comes from ConfigDB (see interfaces.py cfg keys).
"""
from __future__ import annotations

from pyuvm import uvm_agent, uvm_sequencer

from lib.uvm.interfaces import (
    AxisMonitor, AxisSourceDriver, ByteBeatDriver, PixelDriver, PixelMonitor,
)


class _ActiveAgent(uvm_agent):
    driver_cls = None  # set by subclass

    def build_phase(self):
        self.seqr = uvm_sequencer("seqr", self)
        self.driver = self.driver_cls("driver", self)

    def connect_phase(self):
        self.driver.seq_item_port.connect(self.seqr.seq_item_export)


class PixelInputAgent(_ActiveAgent):
    """Valid-only pixel input (config key ``pixel_in_cfg``)."""
    driver_cls = PixelDriver


class ByteBeatInputAgent(_ActiveAgent):
    """csi2 byte-beat input (config key ``byte_in_cfg``)."""
    driver_cls = ByteBeatDriver


class AxisInputAgent(_ActiveAgent):
    """true AXIS input (config key ``axis_in_cfg``)."""
    driver_cls = AxisSourceDriver


class AxisOutputAgent(uvm_agent):
    """Passive AXIS output monitor (config key ``axis_out_cfg``). Its ``.monitor.ap`` is the
    analysis source to connect to a scoreboard."""

    def build_phase(self):
        self.monitor = AxisMonitor("monitor", self)


class PixelOutputAgent(uvm_agent):
    """Passive valid-only pixel output monitor (config key ``pixel_out_cfg``)."""

    def build_phase(self):
        self.monitor = PixelMonitor("monitor", self)
