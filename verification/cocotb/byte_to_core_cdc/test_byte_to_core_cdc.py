"""cocotb port of verification/tb/tb_byte_to_core_cdc.sv (byte-beat interface family, dual-clock CDC).

The DUT is a Gray-code FIFO that crosses a byte-beat stream
(``s_byte_{data,keep,valid,sop,eop}`` on ``byte_clk``) into an identical byte-beat stream
(``m_byte_*`` on ``core_clk``), with a per-output rate limiter (``CORE_OUTPUT_INTERVAL``)
and a saturating overflow counter.

Faithful 1:1 port of the single DSim ``initial`` stimulus block:

* The SV ``always_ff @(posedge core_clk)`` logger -> the ``Capture`` monitor coroutine.
  It records, for every cycle ``m_byte_valid`` is high, the beat (data/keep/sop/eop) plus
  ``core_cycle`` -- a counter that increments every core clock after reset -- into ordered
  logs. This reproduces the ``cycle_log`` used by the rate-limit-gap checks.
* The SV ``push`` task -> ``ByteBeatDriver.send`` on ``byte_clk`` (valid one cycle, idle one
  cycle -- the same 2-cycle cadence).
* ``wait_outputs`` -> :func:`wait_outputs`.
* ``reset_logs`` (which zeroes ``out_count`` mid-run) -> a ``Capture.reset_logs`` snapshot so
  the second (stress) scenario indexes from 0 like the TB.
* The two scenarios stay in ONE ``@cocotb.test()`` because the second reuses the first's
  live FIFO/counters (``sts_lane_fifo_ovf_cnt`` is cumulative), exactly like the TB.
* ``check_condition`` -> ``check``; the ``#1ms`` watchdog -> the ``@cocotb.test`` timeout.

Reset cadence mirrors the TB precisely: both ``*_aresetn`` are held low, released on the
SAME edge (after 6 core clocks), then a 3-byte-clock settle -- so ``bringup_dual`` (which
staggers the two releases) is deliberately not used here.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import ClockCycles, RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.byte_beat import Beat, ByteBeatDriver  # noqa: E402
from lib.clkreset import start_clock  # noqa: E402
from lib.scoreboard import check  # noqa: E402


class Capture:
    """The SV always_ff @(posedge core_clk) logger.

    Increments ``core_cycle`` every core clock after reset release and, whenever
    ``m_byte_valid`` is high, appends the beat plus the current ``core_cycle`` into ordered
    logs (mirroring ``data_log``/``keep_log``/``sop_log``/``eop_log``/``cycle_log``).
    """

    def __init__(self, dut):
        self.dut = dut
        self.core_cycle = 0
        self.data = []
        self.keep = []
        self.sop = []
        self.eop = []
        self.cycle = []

    @property
    def out_count(self):
        return len(self.data)

    def reset_logs(self):
        """Mirror the SV reset_logs task: clears the ordered logs so the next scenario
        indexes from 0. ``core_cycle`` keeps counting (the SV counter is not reset here)."""
        self.data.clear()
        self.keep.clear()
        self.sop.clear()
        self.eop.clear()
        self.cycle.clear()

    def start(self, clk):
        return cocotb.start_soon(self._run(clk))

    async def _run(self, clk):
        d = self.dut
        while True:
            await RisingEdge(clk)
            self.core_cycle += 1
            if int(d.m_byte_valid.value) == 1:
                self.data.append(int(d.m_byte_data.value))
                self.keep.append(int(d.m_byte_keep.value))
                self.sop.append(int(d.m_byte_sop.value))
                self.eop.append(int(d.m_byte_eop.value))
                self.cycle.append(self.core_cycle)


async def wait_outputs(core_clk, cap, count, cycles=200):
    """Mirror the SV wait_outputs task: poll for out_count >= count on core_clk."""
    for _ in range(cycles):
        await RisingEdge(core_clk)
        if cap.out_count >= count:
            return
    raise AssertionError("CHECK FAILED: Timed out waiting for CDC output")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def byte_to_core_cdc(dut):
    # --- clocks: byte_clk #4 (8 ns), core_clk #7 (14 ns) ---
    start_clock(dut.byte_clk, period_ns=8.0)
    start_clock(dut.core_clk, period_ns=14.0)

    # --- reset + initial input state (mirror the SV initial block) ---
    dut.byte_aresetn.value = 0
    dut.core_aresetn.value = 0
    dut.s_byte_data.value = 0
    dut.s_byte_keep.value = 0
    dut.s_byte_valid.value = 0
    dut.s_byte_sop.value = 0
    dut.s_byte_eop.value = 0

    # The logger must be live before reset release so core_cycle counts from the same
    # point as the SV always_ff (which starts counting the cycle after reset deasserts).
    cap = Capture(dut)
    cap.start(dut.core_clk)

    await ClockCycles(dut.core_clk, 6)
    dut.byte_aresetn.value = 1
    dut.core_aresetn.value = 1
    await ClockCycles(dut.byte_clk, 3)

    drv = ByteBeatDriver(dut, dut.byte_clk, prefix="s_byte")

    # --- scenario 1: three-beat packet, rate-limited output ---
    await drv.send(Beat(0x1110, 0b11, sop=True, eop=False))
    await drv.send(Beat(0x2120, 0b11, sop=False, eop=False))
    await drv.send(Beat(0x0030, 0b01, sop=False, eop=True))
    await wait_outputs(dut.core_clk, cap, 3)

    check(cap.data[0] == 0x1110, "beat0 data")
    check(cap.data[1] == 0x2120, "beat1 data")
    check(cap.data[2] == 0x0030, "beat2 data")
    check(cap.keep[2] == 0b01, "tail keep")
    check(cap.sop[0] == 1, "sop forwarded")
    check(cap.eop[2] == 1, "eop forwarded")
    check(cap.cycle[1] >= cap.cycle[0] + 2, "rate limit gap after beat0")
    check(cap.cycle[2] >= cap.cycle[1] + 2, "rate limit gap after beat1")
    check(int(dut.sts_lane_fifo_ovf_cnt.value) == 0x0000, "no overflow")

    # --- scenario 2 (stress): two packets, six beats, order + markers preserved ---
    # reset_logs re-syncs on a core edge and clears the ordered logs (like the SV task).
    await RisingEdge(dut.core_clk)
    cap.reset_logs()

    await drv.send(Beat(0x021E, 0b11, sop=True, eop=False))
    await drv.send(Beat(0x1F05, 0b11, sop=False, eop=False))
    await drv.send(Beat(0x1000, 0b11, sop=False, eop=False))
    await drv.send(Beat(0x2001, 0b11, sop=True, eop=False))
    await drv.send(Beat(0x3002, 0b11, sop=False, eop=False))
    await drv.send(Beat(0x4003, 0b11, sop=False, eop=True))
    await wait_outputs(dut.core_clk, cap, 6)

    check(cap.data[0] == 0x021E, "stress beat0 data preserved")
    check(cap.data[1] == 0x1F05, "stress beat1 data preserved")
    check(cap.data[2] == 0x1000, "stress beat2 data preserved")
    check(cap.data[3] == 0x2001, "stress beat3 data preserved")
    check(cap.data[4] == 0x3002, "stress beat4 data preserved")
    check(cap.data[5] == 0x4003, "stress beat5 data preserved")
    check(cap.sop[0] == 1, "stress first SOP preserved")
    check(cap.sop[3] == 1, "stress second SOP preserved")
    check(cap.eop[5] == 1, "stress final EOP preserved")
    check(int(dut.sts_lane_fifo_ovf_cnt.value) == 0x0000, "stress no overflow")


def test_byte_to_core_cdc():
    from runner_support import build_and_test

    build_and_test(
        block="byte_to_core_cdc",
        sources=["rtl/mipi_rx/byte_to_core_cdc.sv"],
        toplevel="byte_to_core_cdc",
        test_module="test_byte_to_core_cdc",
        test_dir=Path(__file__).resolve().parent,
        parameters={"IN_WIDTH": 16, "KEEP_WIDTH": 2, "FIFO_DEPTH": 8,
                    "CORE_OUTPUT_INTERVAL": 2},
        engine="verilator",
    )
