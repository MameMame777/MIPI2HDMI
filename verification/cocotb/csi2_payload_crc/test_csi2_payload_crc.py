"""cocotb port of verification/tb/tb_csi2_payload_crc.sv (custom payload/footer interface).

Faithful 1:1 port of the DSim TB. The DUT (``rtl/mipi_rx/csi2_payload_crc.sv``) accumulates
a CSI-2 reflected CRC-16 (poly 0x8408, init 0xffff) over payload bytes, then compares the
running ``crc_reg`` against a footer word and pulses ``crc_check_valid`` with match/count
outputs.

The TB's three scenarios (matching CRC, mismatching CRC, single-byte packet) are cumulative
against the status counters (``sts_crc_ok_cnt`` / ``sts_crc_err_cnt``), so they are replicated
as one coroutine over a single reset -- exactly like the original single ``initial`` block --
rather than split into fresh-reset tests. The ``ref_crc_update`` / ``ref_crc3`` SV functions
become the pure-Python helpers below; every ``check_condition`` becomes a ``check()``.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import bringup  # noqa: E402
from lib.scoreboard import check  # noqa: E402

REFLECTED_POLY = 0x8408
INIT = 0xFFFF


def ref_crc_update(crc_in: int, data: int) -> int:
    """Mirror the SV ref_crc_update: 8 bit-serial reflected-CRC steps."""
    crc_next = crc_in & 0xFFFF
    for bit_idx in range(8):
        feedback = (crc_next & 0x1) ^ ((data >> bit_idx) & 0x1)
        crc_next >>= 1
        if feedback:
            crc_next ^= REFLECTED_POLY
    return crc_next & 0xFFFF


def ref_crc3(b0: int, b1: int, b2: int) -> int:
    crc = INIT
    crc = ref_crc_update(crc, b0)
    crc = ref_crc_update(crc, b1)
    crc = ref_crc_update(crc, b2)
    return crc


async def drive_payload(dut, clk, data: int, first: bool, last: bool) -> None:
    """Mirror the SV drive_payload task: one active-valid cycle, then deassert."""
    await RisingEdge(clk)
    dut.payload_data.value = data
    dut.payload_first.value = 1 if first else 0
    dut.payload_last.value = 1 if last else 0
    dut.payload_valid.value = 1
    await RisingEdge(clk)
    dut.payload_valid.value = 0
    dut.payload_first.value = 0
    dut.payload_last.value = 0


async def drive_footer(dut, clk, crc_value: int) -> None:
    """Mirror the SV drive_footer task: one active footer_valid cycle, then deassert."""
    await RisingEdge(clk)
    dut.footer_data.value = crc_value & 0xFFFF
    dut.footer_valid.value = 1
    await RisingEdge(clk)
    dut.footer_valid.value = 0


async def wait_check(dut, clk) -> None:
    """Mirror the SV wait_check task: up to 50 cycles for crc_check_valid, else fatal."""
    for _ in range(50):
        await RisingEdge(clk)
        if int(dut.crc_check_valid.value) == 1:
            return
    raise AssertionError("CHECK FAILED: Timed out waiting for CRC check")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def payload_crc(dut):
    # reset_dut(): drive inputs to idle, hold reset, release.
    dut.payload_data.value = 0
    dut.payload_valid.value = 0
    dut.payload_first.value = 0
    dut.payload_last.value = 0
    dut.footer_data.value = 0
    dut.footer_valid.value = 0
    clk, _ = await bringup(dut, clk="core_clk", rst="core_aresetn")

    # --- scenario 1: matching CRC over three bytes ---
    expected_crc = ref_crc3(0xAA, 0xBB, 0xCC)
    await drive_payload(dut, clk, 0xAA, True, False)
    await drive_payload(dut, clk, 0xBB, False, False)
    await drive_payload(dut, clk, 0xCC, False, True)
    await drive_footer(dut, clk, expected_crc)
    await wait_check(dut, clk)
    check(int(dut.crc_match.value) == 1, "matching CRC accepted")
    check(int(dut.crc_calc.value) == expected_crc, "calculated CRC matches reference")
    check(int(dut.crc_received.value) == expected_crc, "received CRC latched")
    check(int(dut.sts_crc_ok_cnt.value) == 1, "CRC OK count")
    check(int(dut.sts_crc_err_cnt.value) == 0, "CRC error count remains zero")

    # --- scenario 2: mismatching CRC (footer flipped by 1 bit) ---
    expected_crc = ref_crc3(0x01, 0x02, 0x03)
    await drive_payload(dut, clk, 0x01, True, False)
    await drive_payload(dut, clk, 0x02, False, False)
    await drive_payload(dut, clk, 0x03, False, True)
    await drive_footer(dut, clk, expected_crc ^ 0x0001)
    await wait_check(dut, clk)
    check(int(dut.crc_match.value) == 0, "mismatching CRC rejected")
    check(int(dut.sts_crc_ok_cnt.value) == 1, "CRC OK count holds")
    check(int(dut.sts_crc_err_cnt.value) == 1, "CRC error count increments")

    # --- scenario 3: single-byte packet (first and last together) ---
    expected_crc = ref_crc_update(INIT, 0x5A)
    await drive_payload(dut, clk, 0x5A, True, True)
    await drive_footer(dut, clk, expected_crc)
    await wait_check(dut, clk)
    check(int(dut.crc_match.value) == 1, "single byte packet accepted")
    check(int(dut.sts_crc_ok_cnt.value) == 2, "second CRC OK count")

    # tail settle (mirrors the SV repeat(10) before $finish)
    for _ in range(10):
        await RisingEdge(clk)


def test_csi2_payload_crc():
    from runner_support import build_and_test

    build_and_test(
        block="csi2_payload_crc",
        sources=["rtl/mipi_rx/csi2_payload_crc.sv"],
        toplevel="csi2_payload_crc",
        test_module="test_csi2_payload_crc",
        test_dir=Path(__file__).resolve().parent,
        parameters={"INIT": 0xFFFF, "REFLECTED_POLY": 0x8408},
        engine="verilator",
    )
