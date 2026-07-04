"""cocotb port of verification/tb/tb_axis_rgb_proc_slot.sv (valid-only pixel family).

axis_rgb_proc_slot is a 1-cycle registered point-op slot on the 24-bit RGB888 stream.
op_pixel is combinational on the current beat and latched on the posedge, so the result of
driving {cfg_op, cfg_thresh_level, in_pixel, in_valid} at one edge is visible on out_pixel
one posedge later. This mirrors the DSim ``check_op`` task (drive on negedge, sample after
the posedge NBA update).

Covered, 1:1 with the DSim TB:
  1) op 4 (threshold on green) at default level 128 = the old hard-coded ``g > 128``;
  2) the level is runtime-effective (same pixel flips with a low vs high level);
  3) the new cfg_thresh_level port is inert on all other ops (pass / invert / grayscale).
"""
from __future__ import annotations

import os
import random
import sys
from pathlib import Path

import cocotb
from cocotb.triggers import FallingEdge, RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "img_file_uvm"))
import golden as G  # noqa: E402
from lib.clkreset import bringup  # noqa: E402
from lib.coverage import CoverageTally  # noqa: E402
from lib.scoreboard import check  # noqa: E402


async def check_op(dut, clk, nm, op, thr, pix, exp):
    """Replicate the DSim check_op task 1:1: drive cfg + pixel + valid on the negedge so they
    are stable across the posedge that latches op_pixel into out_pixel, then sample after that
    posedge (the TB's ``@(posedge clk); #1``)."""
    await FallingEdge(clk)
    dut.cfg_op.value = op
    dut.cfg_thresh_level.value = thr
    dut.in_pixel.value = pix
    dut.in_valid.value = 1
    await RisingEdge(clk)
    # settle into the ReadOnly phase so the NBA update on out_pixel is visible (TB's #1)
    await FallingEdge(clk)
    got = int(dut.out_pixel.value)
    check(got == exp,
          f"{nm}: op={op} thr={thr} pix={pix:06x} got {got:06x} exp {exp:06x}")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def rgb_proc_slot_threshold_runtime(dut):
    clk, _ = await bringup(dut, clk="clk", rst="rst_n", cycles=4, post=2)

    # idle inputs
    dut.cfg_op.value = 0
    dut.cfg_thresh_level.value = 0x80
    dut.in_pixel.value = 0
    dut.in_valid.value = 0
    dut.in_sof.value = 0
    dut.in_eol.value = 0
    dut.in_eof.value = 0
    dut.in_err.value = 0
    await RisingEdge(clk)

    # --- 1) threshold op 4 at default level 128 = old hard-coded `y(green) > 128` ---
    # green is in_pixel[15:8]; out = (g > thr) ? white : black.
    await check_op(dut, clk, "thr128-g127-black", 4, 0x80, 0x007F00, 0x000000)  # 127 > 128 = 0
    await check_op(dut, clk, "thr128-g128-black", 4, 0x80, 0x008000, 0x000000)  # 128 > 128 = 0 (boundary)
    await check_op(dut, clk, "thr128-g129-white", 4, 0x80, 0x008100, 0xFFFFFF)  # 129 > 128 = 1
    await check_op(dut, clk, "thr128-g200-white", 4, 0x80, 0x00C800, 0xFFFFFF)

    # --- 2) threshold level is runtime-effective: same pixel g=100 flips with the level ---
    await check_op(dut, clk, "thr50-g100-white", 4, 50, 0x006400, 0xFFFFFF)   # 100 > 50  = 1
    await check_op(dut, clk, "thr200-g100-black", 4, 200, 0x006400, 0x000000)  # 100 > 200 = 0
    # threshold keys on GREEN only (R/B do not matter)
    await check_op(dut, clk, "thr100-greenkey", 4, 100, 0xFFC8FF, 0xFFFFFF)   # g=200 > 100 = 1

    # --- 3) the new port is inert on other ops ---
    await check_op(dut, clk, "pass-thresh-inert", 0, 50, 0x123456, 0x123456)    # passthrough
    await check_op(dut, clk, "invert-thresh-inert", 1, 50, 0x000000, 0xFFFFFF)  # ~0 = FFFFFF
    await check_op(dut, clk, "gray-thresh-inert", 2, 200, 0xAA55CC, 0x555555)   # {g,g,g}, g=0x55

    dut.in_valid.value = 0
    await RisingEdge(clk)


# --- additive: cocotb-native parametrized random sweep (one Verilator elaboration) ---------
# @cocotb.parametrize generates one test per op WITHOUT re-elaborating the DUT (contrast
# img_file_uvm, which rebuilds per config at the pytest level). Each op drives seeded-random
# pixels + boundary-biased thresholds and checks every beat against the golden _point_op
# oracle, sampling functional coverage. Complements -- does not replace -- the DSim-parity
# directed test above.

@cocotb.test(timeout_time=4, timeout_unit="ms")
@cocotb.parametrize(op=list(range(8)))
async def rgb_proc_slot_random_sweep(dut, op):
    clk, _ = await bringup(dut, clk="clk", rst="rst_n", cycles=4, post=2)
    for nm in ("in_sof", "in_eol", "in_eof", "in_err"):
        getattr(dut, nm).value = 0
    seed = int(os.environ.get("COCOTB_SEED", "1"), 0)
    rng = random.Random((seed << 8) ^ (op + 1))         # deterministic, distinct per op
    cov = CoverageTally(f"proc_slot_op{op}")
    for i in range(64):
        pix = rng.randrange(0x1000000)
        g = (pix >> 8) & 0xFF
        # bias thresholds toward the green value so op-4 crosses g<thr / g==thr / g>thr
        thr = rng.choice([g, (g + 1) & 0xFF, (g - 1) & 0xFF, rng.randrange(256)])
        exp = G.proc_slot_golden([pix], op, thr)[0]
        await check_op(dut, clk, f"sweep-op{op}-{i}", op, thr, pix, exp)
        cov.sample("op", op)
        if op == 4:
            cov.sample("thr_side", "gt" if g > thr else "eq" if g == thr else "lt")
    dut.in_valid.value = 0
    await RisingEdge(clk)
    if op == 4:
        cov.assert_covered("thr_side", ["gt", "eq", "lt"])   # boundary fully exercised
    dut._log.info(cov.summary())


def test_axis_rgb_proc_slot():
    from runner_support import build_and_test

    build_and_test(
        block="axis_rgb_proc_slot",
        sources=["rtl/img_proc/axis_rgb_proc_slot.sv"],
        toplevel="axis_rgb_proc_slot",
        test_module="test_axis_rgb_proc_slot",
        test_dir=Path(__file__).resolve().parent,
        parameters={"ENABLE": 1},
        engine="verilator",
    )
