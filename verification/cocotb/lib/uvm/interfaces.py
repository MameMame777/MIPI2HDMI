"""pyuvm drivers + monitors for the 3 DUT interface families.

Drivers REUSE the plain `lib/` driving logic by composition (a pyuvm ``uvm_driver`` holds a
``lib.pixel_stream.PixelStreamDriver`` / ``lib.byte_beat.ByteBeatDriver`` / ``lib.axis.AxisSource``
on ``cocotb.top`` and delegates per item) -- so the cadence lives in exactly one place and the
53 plain-lib tests are untouched. Monitors run a minimal sample loop and ``write()`` each
observed item to a ``uvm_analysis_port`` live (the plain monitor loop is trivial).

Each component reads its signal/clock config from ConfigDB under a per-role key set by the
test's ``build_phase`` (e.g. ``ConfigDB().set(None, "*", "pixel_in_cfg", {...})``).
"""
from __future__ import annotations

import cocotb
from cocotb.triggers import RisingEdge
from pyuvm import ConfigDB, uvm_analysis_port, uvm_driver, uvm_monitor

from lib.axis import AxisSource as _AxisSrc
from lib.byte_beat import Beat as _Beat
from lib.byte_beat import ByteBeatDriver as _ByteDrv
from lib.pixel_stream import PixelStreamDriver as _PixelDrv
from lib.uvm.items import AxisItem, PixelItem


def _cfg(comp, key):
    return ConfigDB().get(comp, "", key)


# --------------------------------------------------------------------------- drivers

class PixelDriver(uvm_driver):
    """Valid-only pixel input. Config key ``pixel_in_cfg`` = {clk, pixel, valid, sof, eol,
    eof, err}. Reuses lib.pixel_stream.PixelStreamDriver."""

    cfg_key = "pixel_in_cfg"

    def build_phase(self):
        self.cfg = _cfg(self, self.cfg_key)

    async def run_phase(self):
        dut = cocotb.top
        c = self.cfg
        drv = _PixelDrv(
            dut, getattr(dut, c["clk"]),
            pixel=c.get("pixel", "in_pixel"), valid=c.get("valid", "in_valid"),
            sof=c.get("sof", "in_sof"), eol=c.get("eol", "in_eol"),
            eof=c.get("eof", "in_eof"), err=c.get("err", "in_err"))
        await drv.idle()
        while True:
            it = await self.seq_item_port.get_next_item()
            await drv.send(it.pixel, sof=it.sof, eol=it.eol, eof=it.eof, err=it.err)
            self.seq_item_port.item_done()


class ByteBeatDriver(uvm_driver):
    """csi2 byte-beat input. Config ``byte_in_cfg`` = {clk, prefix}. Reuses ByteBeatDriver."""

    cfg_key = "byte_in_cfg"

    def build_phase(self):
        self.cfg = _cfg(self, self.cfg_key)

    async def run_phase(self):
        dut = cocotb.top
        c = self.cfg
        drv = _ByteDrv(dut, getattr(dut, c["clk"]), prefix=c.get("prefix", "s_byte"))
        await drv.idle()
        while True:
            it = await self.seq_item_port.get_next_item()
            await drv.send(_Beat(it.data, it.keep, bool(it.sop), bool(it.eop)))
            self.seq_item_port.item_done()


class AxisSourceDriver(uvm_driver):
    """true AXIS input. Config ``axis_in_cfg`` = {clk, prefix}. Reuses AxisSource."""

    cfg_key = "axis_in_cfg"

    def build_phase(self):
        self.cfg = _cfg(self, self.cfg_key)

    async def run_phase(self):
        dut = cocotb.top
        c = self.cfg
        src = _AxisSrc(dut, getattr(dut, c["clk"]), prefix=c.get("prefix", "s_axis"))
        await src.idle()
        while True:
            it = await self.seq_item_port.get_next_item()
            await src.send(it.data, last=it.last, user=it.user)
            self.seq_item_port.item_done()


# --------------------------------------------------------------------------- monitors

class AxisMonitor(uvm_monitor):
    """Samples the AXIS output; writes an AxisItem to ``.ap`` on each accepted beat.
    Config ``axis_out_cfg`` = {clk, prefix}."""

    cfg_key = "axis_out_cfg"

    def build_phase(self):
        self.ap = uvm_analysis_port("ap", self)
        self.cfg = _cfg(self, self.cfg_key)

    async def run_phase(self):
        dut = cocotb.top
        c = self.cfg
        clk = getattr(dut, c["clk"])
        pfx = c.get("prefix", "m_axis")
        tvalid = getattr(dut, f"{pfx}_tvalid")
        tready = getattr(dut, f"{pfx}_tready")
        tdata = getattr(dut, f"{pfx}_tdata")
        tlast = getattr(dut, f"{pfx}_tlast", None)
        tuser = getattr(dut, f"{pfx}_tuser", None)
        while True:
            await RisingEdge(clk)
            if int(tvalid.value) == 1 and int(tready.value) == 1:
                self.ap.write(AxisItem(
                    data=int(tdata.value),
                    last=int(tlast.value) if tlast is not None else 0,
                    user=int(tuser.value) if tuser is not None else 0))


class PixelMonitor(uvm_monitor):
    """Samples a valid-only pixel stream; writes PixelItem to ``.ap``.
    Config ``pixel_out_cfg`` = {clk, pixel, valid, sof, eol, eof, err}."""

    cfg_key = "pixel_out_cfg"

    def build_phase(self):
        self.ap = uvm_analysis_port("ap", self)
        self.cfg = _cfg(self, self.cfg_key)

    async def run_phase(self):
        dut = cocotb.top
        c = self.cfg
        clk = getattr(dut, c["clk"])
        pixel = getattr(dut, c.get("pixel", "out_pixel"))
        valid = getattr(dut, c.get("valid", "out_valid"))
        sof = getattr(dut, c.get("sof", "out_sof"), None)
        eol = getattr(dut, c.get("eol", "out_eol"), None)
        eof = getattr(dut, c.get("eof", "out_eof"), None)
        err = getattr(dut, c.get("err", "out_err"), None)

        def rd(h):
            return int(h.value) if h is not None else 0

        while True:
            await RisingEdge(clk)
            if int(valid.value) == 1:
                self.ap.write(PixelItem(
                    pixel=int(pixel.value), sof=rd(sof), eol=rd(eol),
                    eof=rd(eof), err=rd(err)))
