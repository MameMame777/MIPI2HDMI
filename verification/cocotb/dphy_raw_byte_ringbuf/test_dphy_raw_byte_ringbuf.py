"""cocotb port of verification/tb/tb_dphy_raw_byte_ringbuf.sv (dual-clock raw byte ring buffer).

The DUT captures the post-ISERDES byte stream (lane0/lane1 + SoT/sync markers) into a BRAM
on ``byte_clk`` and exposes an address-mapped 16-bit read port on ``rd_clk`` (bit[9] of
``rd_addr`` selects the hi/lo half of each 32-bit word). Status flags are 2FF-synced from
the write domain to the read domain.

Faithful 1:1 port of the four DSim scenarios, each with a fresh async reset of the
``byte_clk`` domain (mirrors the TB's ``reset_dut`` task):
  S0  free-run capture: arm -> captures DEPTH entries; entry0 has first_entry marker.
  S1  SoT marker propagation: wherever lane0/lane1 == 0xB8, sot_l0/sot_l1 must be set.
  S2  trigger mode: armed -> waiting (no capture) until sync_trigger rising edge -> capture.
  S3  first_entry marker only at index 0 (idx3 must be 0).

The TB's two ``initial`` blocks (driver fork branches) become ``cocotb.start_soon`` coros;
the SV ``expect_eq`` maps to ``check``; the ``#500us`` watchdog maps to the test timeout.
The TB settles driver signals on ``negedge byte_clk``; here we set them right after
``RisingEdge(byte_clk)``, which is phase-equivalent for the posedge-sampled DUT.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.scoreboard import check  # noqa: E402

DEPTH = 16  # tb localparam

# Word bit layout (see rtl/prototype/dphy_raw_byte_ringbuf.sv):
#   [7:0]  lane0   [15:8] lane1   [20] first_entry   [21] sot_l0   [22] sot_l1   [23] sync_hv


async def _start_clocks(dut):
    """byte_clk = #5 (10 ns), rd_clk = #4 (8 ns) as in the TB."""
    cocotb.start_soon(Clock(dut.byte_clk, 10.0, unit="ns").start())
    cocotb.start_soon(Clock(dut.rd_clk, 8.0, unit="ns").start())


async def _reset_dut(dut):
    """Mirror the TB reset_dut task: hold rst_n low 4 byte_clks, release, settle 4."""
    dut.rst_n_byte.value = 0
    await ClockCycles(dut.byte_clk, 4)
    dut.rst_n_byte.value = 1
    await ClockCycles(dut.byte_clk, 4)


def _init_inputs(dut):
    dut.lane0_byte_in.value = 0x00
    dut.lane1_byte_in.value = 0x00
    dut.sync_header_valid_byte.value = 0
    dut.sync_trigger_byte.value = 0
    dut.arm_trigger_byte.value = 0
    dut.trigger_mode_byte.value = 0
    dut.rd_addr.value = 0


async def read_word(dut, idx: int) -> int:
    """Address-mapped 32-bit read: lo half (rd_addr[9]=0) then hi half (rd_addr[9]=1),
    each after 4 rd_clk edges to cover the 2-stage read latency (mirrors read_word task)."""
    dut.rd_addr.value = (idx & 0x1FF)              # bit9 = 0 -> low 16
    for _ in range(4):
        await RisingEdge(dut.rd_clk)
    lo = int(dut.rd_data.value)
    dut.rd_addr.value = (1 << 9) | (idx & 0x1FF)   # bit9 = 1 -> high 16
    for _ in range(4):
        await RisingEdge(dut.rd_clk)
    hi = int(dut.rd_data.value)
    return (hi << 16) | lo


async def wait_full(dut, max_cycles: int):
    """Poll full_sync on byte_clk edges; fail on timeout (mirrors wait_full task)."""
    for _ in range(max_cycles):
        await RisingEdge(dut.byte_clk)
        if int(dut.full_sync.value) == 1:
            return
    raise AssertionError("CHECK FAILED: wait_full timeout")


async def _arm_pulse(dut):
    """One-cycle arm_trigger pulse, aligned like the TB's fork arm branch."""
    await FallingEdge(dut.byte_clk)
    dut.arm_trigger_byte.value = 1
    await FallingEdge(dut.byte_clk)
    dut.arm_trigger_byte.value = 0


@cocotb.test(timeout_time=500, timeout_unit="us")
async def s0_free_run_capture(dut):
    await _start_clocks(dut)
    _init_inputs(dut)

    dut.rst_n_byte.value = 0
    await ClockCycles(dut.byte_clk, 4)
    dut.rst_n_byte.value = 1
    await ClockCycles(dut.byte_clk, 4)

    dut.trigger_mode_byte.value = 0

    async def driver():
        for c in range(DEPTH + 8):
            await FallingEdge(dut.byte_clk)
            dut.lane0_byte_in.value = 0x00 | (c & 0xFF)
            dut.lane1_byte_in.value = 0x80 | (c & 0xFF)
            dut.sync_header_valid_byte.value = c & 1

    drv = cocotb.start_soon(driver())
    cocotb.start_soon(_arm_pulse(dut))

    await wait_full(dut, DEPTH + 12)
    drv.kill()

    check(int(dut.full_sync.value) == 1, "S0: full=1")
    check(int(dut.armed_sync.value) == 0, "S0: armed=0")

    w0 = await read_word(dut, 0)
    check((w0 >> 20) & 1 == 1, "S0: entry0 first_entry marker")


@cocotb.test(timeout_time=500, timeout_unit="us")
async def s1_sot_marker_propagation(dut):
    await _start_clocks(dut)
    _init_inputs(dut)
    await _reset_dut(dut)

    dut.trigger_mode_byte.value = 0

    async def driver():
        for c in range(DEPTH + 12):
            await FallingEdge(dut.byte_clk)
            if c == 2:
                dut.lane0_byte_in.value = 0xB8
                dut.lane1_byte_in.value = 0xB8
                dut.sync_header_valid_byte.value = 1
            elif c == 5:
                dut.lane0_byte_in.value = 0xB8
                dut.lane1_byte_in.value = 0x11
                dut.sync_header_valid_byte.value = 0
            elif c == 7:
                dut.lane0_byte_in.value = 0x22
                dut.lane1_byte_in.value = 0xB8
                dut.sync_header_valid_byte.value = 0
            else:
                dut.lane0_byte_in.value = 0x00
                dut.lane1_byte_in.value = 0x00
                dut.sync_header_valid_byte.value = 0

    drv = cocotb.start_soon(driver())
    cocotb.start_soon(_arm_pulse(dut))

    await wait_full(dut, DEPTH + 16)
    drv.kill()

    b8_l0_words = 0
    b8_l1_words = 0
    marker_mismatch = 0
    for idx in range(DEPTH):
        w = await read_word(dut, idx)
        lane0 = w & 0xFF
        lane1 = (w >> 8) & 0xFF
        sot_l0 = (w >> 21) & 1
        sot_l1 = (w >> 22) & 1
        if lane0 == 0xB8:
            b8_l0_words += 1
            if sot_l0 != 1:
                marker_mismatch += 1
                check(False, f"S1: idx{idx} lane0=B8 but sot_l0=0")
        else:
            if sot_l0 != 0:
                marker_mismatch += 1
                check(False, f"S1: idx{idx} lane0=0x{lane0:02x} but sot_l0=1")
        if lane1 == 0xB8:
            b8_l1_words += 1
            if sot_l1 != 1:
                marker_mismatch += 1
                check(False, f"S1: idx{idx} lane1=B8 but sot_l1=0")
        else:
            if sot_l1 != 0:
                marker_mismatch += 1
                check(False, f"S1: idx{idx} lane1=0x{lane1:02x} but sot_l1=1")

    check(b8_l0_words != 0, "S1: no 0xB8 captured on lane0 - driver timing off")
    check(marker_mismatch == 0, "S1: SoT marker mismatches")


@cocotb.test(timeout_time=500, timeout_unit="us")
async def s2_trigger_mode_and_s3_first_entry(dut):
    await _start_clocks(dut)
    _init_inputs(dut)
    await _reset_dut(dut)

    dut.trigger_mode_byte.value = 1
    dut.sync_trigger_byte.value = 0
    dut.lane0_byte_in.value = 0x00
    dut.lane1_byte_in.value = 0x00

    # Arm (inline, not forked): the TB arms then waits before launching the data driver.
    await FallingEdge(dut.byte_clk)
    dut.arm_trigger_byte.value = 1
    await FallingEdge(dut.byte_clk)
    dut.arm_trigger_byte.value = 0

    # 12 byte_clk cycles of no trigger, then 6 rd_clk for the status 2FF to settle.
    await ClockCycles(dut.byte_clk, 12)
    await ClockCycles(dut.rd_clk, 6)
    check(int(dut.armed_sync.value) == 1, "S2: armed=1 (waiting)")
    check(int(dut.waiting_sync.value) == 1, "S2: waiting=1")
    check(int(dut.full_sync.value) == 0, "S2: full=0 before trig")

    async def driver():
        for c in range(DEPTH + 12):
            await FallingEdge(dut.byte_clk)
            dut.lane0_byte_in.value = 0x40 | (c & 0xFF)
            dut.lane1_byte_in.value = 0xC0 | (c & 0xFF)
            if c == 3:
                dut.sync_trigger_byte.value = 1
            if c == 4:
                dut.sync_trigger_byte.value = 0

    drv = cocotb.start_soon(driver())
    await wait_full(dut, DEPTH + 20)
    drv.kill()

    check(int(dut.full_sync.value) == 1, "S2: full=1 after trigger")
    check(int(dut.waiting_sync.value) == 0, "S2: waiting=0")

    w0 = await read_word(dut, 0)
    check((w0 >> 20) & 1 == 1, "S2: entry0 first_entry marker")

    # S3: idx3 must NOT have the first_entry marker.
    w3 = await read_word(dut, 3)
    check((w3 >> 20) & 1 == 0, "S3: idx3 first_entry=0")


def test_dphy_raw_byte_ringbuf():
    from runner_support import build_and_test

    build_and_test(
        block="dphy_raw_byte_ringbuf",
        sources=["rtl/prototype/dphy_raw_byte_ringbuf.sv"],
        toplevel="dphy_raw_byte_ringbuf",
        test_module="test_dphy_raw_byte_ringbuf",
        test_dir=Path(__file__).resolve().parent,
        parameters={"DEPTH": 16},
        engine="verilator",
    )
