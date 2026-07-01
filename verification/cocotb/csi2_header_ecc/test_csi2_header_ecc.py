"""cocotb port of verification/tb/tb_csi2_header_ecc.sv (valid-only custom header interface).

Faithful 1:1 port. The DSim TB is a single ``initial`` block that:
  * drives a 32-bit CSI-2 short-packet header (``hdr_raw``) with a one-cycle ``hdr_valid``
    pulse (``drive_header``),
  * waits up to 20 cycles for ``hdr_corr_valid`` (``wait_corr``),
  * checks the corrected data / DI / WC decode and the ECC status pulses.

Here ``drive_header`` and ``wait_corr`` become async helpers and every ``check_condition``
becomes ``check``. The DUT has no tready and no parameters (fixed-width ports), so it needs
no lib driver -- ``hdr_raw`` / ``hdr_valid`` are poked directly, mirroring the TB task.

The single TB ``initial`` scenario walks: (1) clean header, (2) all 24 single data-bit
flips, (3) an ECC-parity-bit flip, (4) a double ECC-parity-bit flip. It is kept as one
``@cocotb.test()`` so the cumulative status counters match the original run.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import bringup  # noqa: E402
from lib.scoreboard import check  # noqa: E402


def ref_ecc6(data: int) -> int:
    """Mirror of the TB ref_ecc6 / DUT calc_ecc6 parity tables."""
    def b(i: int) -> int:
        return (data >> i) & 1

    e = [0] * 6
    e[0] = b(0) ^ b(1) ^ b(2) ^ b(4) ^ b(5) ^ b(7) ^ b(10) ^ b(11) ^ b(13) ^ b(16) ^ b(20) ^ b(21) ^ b(22) ^ b(23)
    e[1] = b(0) ^ b(1) ^ b(3) ^ b(4) ^ b(6) ^ b(8) ^ b(10) ^ b(12) ^ b(14) ^ b(17) ^ b(20) ^ b(21) ^ b(22) ^ b(23)
    e[2] = b(0) ^ b(2) ^ b(3) ^ b(5) ^ b(6) ^ b(9) ^ b(11) ^ b(12) ^ b(15) ^ b(18) ^ b(20) ^ b(21) ^ b(22)
    e[3] = b(1) ^ b(2) ^ b(3) ^ b(7) ^ b(8) ^ b(9) ^ b(13) ^ b(14) ^ b(15) ^ b(19) ^ b(20) ^ b(21) ^ b(23)
    e[4] = b(4) ^ b(5) ^ b(6) ^ b(7) ^ b(8) ^ b(9) ^ b(16) ^ b(17) ^ b(18) ^ b(19) ^ b(20) ^ b(22) ^ b(23)
    e[5] = b(10) ^ b(11) ^ b(12) ^ b(13) ^ b(14) ^ b(15) ^ b(16) ^ b(17) ^ b(18) ^ b(19) ^ b(21) ^ b(22) ^ b(23)
    return sum(bit << i for i, bit in enumerate(e))


def make_header(data: int) -> int:
    """{2'b00, ecc6, data[23:0]} -> 32-bit raw header."""
    return ((ref_ecc6(data) & 0x3F) << 24) | (data & 0xFFFFFF)


async def drive_header(dut, clk, raw: int) -> None:
    """Mirror the TB drive_header task: one-cycle hdr_valid pulse carrying raw."""
    await RisingEdge(clk)
    dut.hdr_raw.value = raw
    dut.hdr_valid.value = 1
    await RisingEdge(clk)
    dut.hdr_valid.value = 0


async def wait_corr(dut, clk) -> None:
    """Mirror the TB wait_corr task: up to 20 cycles for hdr_corr_valid, else fatal."""
    for _ in range(20):
        await RisingEdge(clk)
        if int(dut.hdr_corr_valid.value) == 1:
            return
    raise AssertionError("CHECK FAILED: Timed out waiting for ECC correction")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def header_ecc(dut):
    clk, _ = await bringup(dut, clk="core_clk", rst="core_aresetn")
    dut.hdr_valid.value = 0
    dut.hdr_raw.value = 0

    # (1) clean header: no error, data/DI/WC decode.
    data = 0x12342A
    header = make_header(data)
    await drive_header(dut, clk, header)
    await wait_corr(dut, clk)
    check(int(dut.hdr_corr.value) == data, "no-error data passes")
    check(int(dut.hdr_di.value) == 0x2A, "DI decode")
    check(int(dut.hdr_wc.value) == 0x1234, "WC decode")
    check(int(dut.hdr_ecc_no_error.value) == 1, "no-error pulse")
    check(int(dut.hdr_ecc_uncorrectable.value) == 0, "no uncorrectable on clean header")

    # (2) every single data-bit flip is corrected.
    for bit_idx in range(24):
        data = 0x03AA2B
        header = make_header(data)
        corrupt = header ^ (1 << bit_idx)
        await drive_header(dut, clk, corrupt)
        await wait_corr(dut, clk)
        check(int(dut.hdr_corr.value) == data, "single data bit correction")
        check(int(dut.hdr_ecc_corrected.value) == 1, "single data bit corrected pulse")
        check(int(dut.hdr_ecc_uncorrectable.value) == 0, "single data bit not uncorrectable")

    # (3) single ECC-parity-bit flip: data unchanged, counted as corrected.
    data = 0x000100
    header = make_header(data)
    corrupt = header ^ (1 << 24)
    await drive_header(dut, clk, corrupt)
    await wait_corr(dut, clk)
    check(int(dut.hdr_corr.value) == data, "ECC bit flip leaves data unchanged")
    check(int(dut.hdr_ecc_corrected.value) == 1, "ECC bit flip counted corrected")

    # (4) double ECC-parity-bit flip: data unchanged, uncorrectable.
    data = 0x00022A
    header = make_header(data)
    corrupt = header ^ (0x3 << 24)
    await drive_header(dut, clk, corrupt)
    await wait_corr(dut, clk)
    check(int(dut.hdr_corr.value) == data, "multi ECC bit flip leaves data unchanged")
    check(int(dut.hdr_ecc_uncorrectable.value) == 1, "multi ECC bit flip uncorrectable")

    for _ in range(10):
        await RisingEdge(clk)


def test_csi2_header_ecc():
    from runner_support import build_and_test

    build_and_test(
        block="csi2_header_ecc",
        sources=["rtl/mipi_rx/csi2_header_ecc.sv"],
        toplevel="csi2_header_ecc",
        test_module="test_csi2_header_ecc",
        test_dir=Path(__file__).resolve().parent,
        parameters={},
        engine="verilator",
    )
