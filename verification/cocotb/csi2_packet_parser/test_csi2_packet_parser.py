"""cocotb port of verification/tb/tb_csi2_packet_parser.sv (byte-beat interface family).

Faithful 1:1 port: the DSim TB models a neighbouring ECC responder (a second ``initial``
block that drives ``ecc_hdr_*`` two cycles after ``ecc_hdr_valid``) and an ``always_ff``
logger; here those become the ``ecc_responder`` coroutine and the ``Capture`` monitor. The
three scenarios are split into three ``@cocotb.test()``s (each fresh-reset), which yields
the same status-counter expectations as the original single cumulative run.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import ClockCycles, RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.byte_beat import Beat, ByteBeatDriver  # noqa: E402
from lib.clkreset import bringup  # noqa: E402
from lib.scoreboard import check  # noqa: E402


async def ecc_responder(dut):
    """Mirror the SV initial block: 2 cycles after ecc_hdr_valid, pulse ecc_hdr_corr_valid
    with di=raw[7:0], wc=raw[23:8] (uncorrectable=0)."""
    dut.ecc_hdr_corr_valid.value = 0
    dut.ecc_hdr_di.value = 0
    dut.ecc_hdr_wc.value = 0
    dut.ecc_hdr_uncorrectable.value = 0
    while True:
        await RisingEdge(dut.core_clk)
        dut.ecc_hdr_corr_valid.value = 0
        if int(dut.ecc_hdr_valid.value) == 1:
            raw = int(dut.ecc_hdr_raw.value)
            await ClockCycles(dut.core_clk, 2)
            dut.ecc_hdr_di.value = raw & 0xFF
            dut.ecc_hdr_wc.value = (raw >> 8) & 0xFFFF
            dut.ecc_hdr_uncorrectable.value = 0
            dut.ecc_hdr_corr_valid.value = 1


class Capture:
    """The always_ff logger: header/payload/footer/done counters."""

    def __init__(self, dut):
        self.dut = dut
        self.hdr_seen = 0
        self.payload = []        # list of (data, first, last)
        self.footer_seen = False
        self.footer = 0
        self.done_seen = 0

    def start(self, clk):
        return cocotb.start_soon(self._run(clk))

    async def _run(self, clk):
        d = self.dut
        while True:
            await RisingEdge(clk)
            if int(d.m_pkt_hdr_valid.value) == 1:
                self.hdr_seen += 1
            if int(d.m_payload_valid.value) == 1:
                self.payload.append((int(d.m_payload_data.value),
                                     int(d.m_payload_first.value),
                                     int(d.m_payload_last.value)))
            if int(d.m_footer_valid.value) == 1:
                self.footer_seen = True
                self.footer = int(d.m_footer_data.value)
            if int(d.m_pkt_done.value) == 1:
                self.done_seen += 1


async def wait_done(clk, cap, prev=0, cycles=200):
    for _ in range(cycles):
        await RisingEdge(clk)
        if cap.done_seen > prev:
            return
    raise AssertionError("CHECK FAILED: timed out waiting for packet done")


async def _setup(dut):
    clk, _ = await bringup(dut, clk="core_clk", rst="core_aresetn")
    cocotb.start_soon(ecc_responder(dut))
    cap = Capture(dut)
    cap.start(clk)
    drv = ByteBeatDriver(dut, clk, prefix="s_byte")
    await drv.idle()
    return clk, cap, drv


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def short_packet(dut):
    clk, cap, drv = await _setup(dut)
    await drv.send(Beat(0x3400, 0b11, sop=True, eop=False))
    await drv.send(Beat(0x0012, 0b11, sop=False, eop=True))
    await wait_done(clk, cap, 0)
    check(cap.hdr_seen == 1, "short header event count")
    check(cap.done_seen == 1, "short done count")
    check(int(dut.sts_short_pkt_cnt.value) == 1, "short status count")
    check(int(dut.sts_long_pkt_cnt.value) == 0, "long status count after short")
    check(len(cap.payload) == 0, "short packet has no payload")
    check(int(dut.m_pkt_di.value) == 0x00, "short DI")
    check(int(dut.m_pkt_wc.value) == 0x1234, "short WC")
    check(int(dut.m_pkt_is_short.value) == 1, "short classification")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def long_packet(dut):
    clk, cap, drv = await _setup(dut)
    await drv.send(Beat(0x032a, 0b11, sop=True, eop=False))
    await drv.send(Beat(0x0000, 0b11, sop=False, eop=False))
    await drv.send(Beat(0xbbaa, 0b11, sop=False, eop=False))
    await drv.send(Beat(0x34cc, 0b11, sop=False, eop=False))
    await drv.send(Beat(0x0012, 0b01, sop=False, eop=True))
    await wait_done(clk, cap, 0)
    check(cap.hdr_seen == 1, "long header event count")
    check(cap.done_seen == 1, "long done count")
    check(int(dut.sts_long_pkt_cnt.value) == 1, "long status count")
    check(len(cap.payload) == 3, "long payload count")
    check(cap.payload[0][0] == 0xaa, "payload byte 0")
    check(cap.payload[1][0] == 0xbb, "payload byte 1")
    check(cap.payload[2][0] == 0xcc, "payload byte 2")
    check(cap.payload[0][1] == 1, "payload first")
    check(cap.payload[2][2] == 1, "payload last")
    check(cap.footer_seen, "footer valid")
    check(cap.footer == 0x1234, "footer byte order")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def truncated_packet(dut):
    clk, cap, drv = await _setup(dut)
    await drv.send(Beat(0x042a, 0b11, sop=True, eop=False))
    await drv.send(Beat(0x0000, 0b11, sop=False, eop=False))
    await drv.send(Beat(0x2211, 0b11, sop=False, eop=True))
    await wait_done(clk, cap, 0)
    check(cap.done_seen == 1, "truncated done count")
    check(int(dut.sts_pkt_trunc_cnt.value) == 1, "truncate status count")
    check(len(cap.payload) == 2, "truncated payload count before recovery")


def test_csi2_packet_parser():
    from runner_support import build_and_test

    build_and_test(
        block="csi2_packet_parser",
        sources=["rtl/mipi_rx/csi2_packet_parser.sv"],
        toplevel="csi2_packet_parser",
        test_module="test_csi2_packet_parser",
        test_dir=Path(__file__).resolve().parent,
        parameters={"IN_WIDTH": 16, "WC_MAX": 16, "FIFO_DEPTH": 32},
    )
