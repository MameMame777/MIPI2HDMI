"""cocotb port of verification/tb/tb_ob_row_masker_raw10.sv (valid-only pixel family).

RAW10 (WIDTH=10) tests of the optical-black row masker. The DUT tracks full-line
min/max, finalizes a "dark" decision at EOL, and (when enable=1) replaces the entire
line with OB_FILL_Y iff  max < OB_THRESHOLD && (max - min) <= OB_UNIFORMITY.

Scaled RAW10 thresholds: OB_THRESHOLD=200, OB_FILL_Y=512, OB_UNIFORMITY=12.

Each scenario drives one contiguous row (in_valid high across all pixels, in_eol on the
last pixel -- matching the DSim ``drive_row`` task), waits for the line-buffer read-out to
flush, and checks the collected output row against the expected values (FILL for masked
rows, the identity row for passed rows). Same DUT + same stimulus as the DSim TB, so these
checks match 1:1.
"""
from __future__ import annotations

import random
import sys
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import bringup  # noqa: E402
from lib.scoreboard import check  # noqa: E402

WIDTH = 10
TH = 200
FILL = 512
UNIF = 12
MASK = 0x3FF  # WIDTH-bit mask


async def drive_row(dut, clk, data):
    """Mirror the DSim ``drive_row`` task: drive each pixel on a rising edge with
    in_valid=1, in_eol=1 only on the last pixel, then deassert valid/eol."""
    n = len(data)
    for i, px in enumerate(data):
        await RisingEdge(clk)
        dut.in_data.value = px & MASK
        dut.in_valid.value = 1
        dut.in_sof.value = 0
        dut.in_eol.value = 1 if i == n - 1 else 0
        dut.in_eof.value = 0
        dut.in_err.value = 0
    await RisingEdge(clk)
    dut.in_valid.value = 0
    dut.in_eol.value = 0


async def wait_pipeline_flush(dut, clk):
    """Mirror the DSim ``wait_pipeline_flush``: run until out_valid has been idle for 8
    consecutive cycles (or 4096-cycle safety cap)."""
    idle = 0
    max_wait = 4096
    while idle < 8 and max_wait > 0:
        await RisingEdge(clk)
        if int(dut.out_valid.value) == 1:
            idle = 0
        else:
            idle += 1
        max_wait -= 1


class OutRow:
    """Collect out_data on every rising edge where out_valid=1 (the DSim always_ff push)."""

    def __init__(self, dut, clk):
        self.dut = dut
        self.clk = clk
        self.row = []

    def start(self):
        return cocotb.start_soon(self._run())

    async def _run(self):
        while True:
            await RisingEdge(self.clk)
            if int(self.dut.out_valid.value) == 1:
                self.row.append(int(self.dut.out_data.value))

    def delete(self):
        self.row.clear()


def check_all_equal(row, expected, name):
    check(len(row) > 0, f"{name}: no output")
    for i, v in enumerate(row):
        check(v == (expected & MASK),
              f"{name}: pix {i} = 0x{v:03x}, expected 0x{expected & MASK:03x}")


def check_equal_array(row, expected, name):
    check(len(row) == len(expected),
          f"{name}: size {len(row)} != {len(expected)}")
    for i, v in enumerate(row):
        check(v == (expected[i] & MASK),
              f"{name}: pix {i} = 0x{v:03x}, expected 0x{expected[i] & MASK:03x}")


async def _init(dut):
    dut.in_data.value = 0
    dut.in_valid.value = 0
    dut.in_sof.value = 0
    dut.in_eol.value = 0
    dut.in_eof.value = 0
    dut.in_err.value = 0
    dut.enable.value = 1
    # bringup: start clk, async active-low reset via aresetn (hold low, then release).
    clk, _ = await bringup(dut, clk="clk", rst="aresetn", cycles=5, post=3)
    return clk


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def raw10_masking(dut):
    """Tests 1-2: rows that satisfy max<200 && range<=12 are masked to FILL."""
    clk = await _init(dut)
    out = OutRow(dut, clk)
    out.start()

    # 1: True OB uniform Y=144 -> MASK
    row = [144] * 16
    out.delete()
    await drive_row(dut, clk, row)
    await wait_pipeline_flush(dut, clk)
    check_all_equal(out.row, FILL, "RAW10 OB Y=144 uniform -> mask")

    # 2: OB with small variation (range=8 <= 12) -> MASK
    row = [140, 148, 142, 146, 141, 147, 143, 145,
           144, 142, 148, 140, 146, 144, 141, 147]
    out.delete()
    await drive_row(dut, clk, row)
    await wait_pipeline_flush(dut, clk)
    check_all_equal(out.row, FILL, "RAW10 OB range=8 -> mask")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def raw10_pass(dut):
    """Tests 3-6, 8-12: rows that fail the dark test pass through unchanged; plus the
    enable=0 bypass (test 7)."""
    clk = await _init(dut)
    out = OutRow(dut, clk)
    out.start()

    # 3: Image row, dark first pixel only -> PASS
    row = [180, 800, 840, 720, 600, 480, 360, 240,
           120, 200, 340, 540, 680, 820, 900, 1020]
    out.delete()
    await drive_row(dut, clk, row)
    await wait_pipeline_flush(dut, clk)
    check_equal_array(out.row, row, "RAW10 image dark first pixel -> pass")

    # 4: Bright uniform Y=800 -> PASS (above threshold)
    row = [800] * 16
    out.delete()
    await drive_row(dut, clk, row)
    await wait_pipeline_flush(dut, clk)
    check_equal_array(out.row, row, "RAW10 bright uniform Y=800 -> pass")

    # 5: Dark non-uniform (all < 200 but range > 12) -> PASS
    row = [120, 20, 180, 8, 100, 180, 40, 160,
           140, 60, 190, 32, 80, 170, 48, 150]
    out.delete()
    await drive_row(dut, clk, row)
    await wait_pipeline_flush(dut, clk)
    check_equal_array(out.row, row, "RAW10 dark non-uniform -> pass")

    # 6: Uniform but >= threshold (Y=320 +/- 4) -> PASS
    row = [320, 322, 318, 321, 319, 322, 320, 318,
           321, 320, 322, 318, 319, 321, 320, 322]
    out.delete()
    await drive_row(dut, clk, row)
    await wait_pipeline_flush(dut, clk)
    check_equal_array(out.row, row, "RAW10 uniform Y=320 above thresh -> pass")

    # 7: enable=0 bypass on OB -> pass
    row = [144] * 16
    out.delete()
    dut.enable.value = 0
    await drive_row(dut, clk, row)
    await wait_pipeline_flush(dut, clk)
    check_equal_array(out.row, row, "RAW10 bypass enable=0 -> pass")
    dut.enable.value = 1

    # 8: Checkerboard 8 dark + 8 bright (Y=40 + Y=960) -> PASS
    row = [40] * 8 + [960] * 8
    out.delete()
    await drive_row(dut, clk, row)
    await wait_pipeline_flush(dut, clk)
    check_equal_array(out.row, row, "RAW10 checkerboard 8+8 -> pass")

    # 9: Grayscale uniform Y=256 -> PASS (above threshold)
    row = [256] * 16
    out.delete()
    await drive_row(dut, clk, row)
    await wait_pipeline_flush(dut, clk)
    check_equal_array(out.row, row, "RAW10 grayscale Y=256 -> pass")

    # 10: Gradient 0->960 (steps of 64) -> PASS
    row = [0, 64, 128, 192, 256, 320, 384, 448,
           512, 576, 640, 704, 768, 832, 896, 960]
    out.delete()
    await drive_row(dut, clk, row)
    await wait_pipeline_flush(dut, clk)
    check_equal_array(out.row, row, "RAW10 gradient 0->960 -> pass")

    # 11: Gradient with tight dark start -> PASS
    row = [40, 44, 48, 52, 200, 320, 440, 560,
           680, 800, 920, 960, 920, 800, 680, 560]
    out.delete()
    await drive_row(dut, clk, row)
    await wait_pipeline_flush(dut, clk)
    check_equal_array(out.row, row, "RAW10 gradient tight dark start -> pass")

    # 12: Bayer-like pattern -> PASS
    row = [300, 600, 600, 300] * 4
    out.delete()
    await drive_row(dut, clk, row)
    await wait_pipeline_flush(dut, clk)
    check_equal_array(out.row, row, "RAW10 Bayer RGGB -> pass")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def raw10_random(dut):
    """Tests 13-17: random full 10-bit rows pass through unchanged.

    A random 16-pixel row over [0,1023] can, with vanishingly small probability, satisfy
    max<200 && range<=12; the DSim TB assumes this never happens for its $urandom seed. A
    fixed seed here keeps that assumption deterministic while covering the pass path."""
    clk = await _init(dut)
    out = OutRow(dut, clk)
    out.start()

    rng = random.Random(0xB16B00B5)
    for trial in range(5):
        row = [rng.randint(0, 1023) for _ in range(16)]
        out.delete()
        await drive_row(dut, clk, row)
        await wait_pipeline_flush(dut, clk)
        check_equal_array(out.row, row, f"RAW10 random trial {trial} -> pass")


def test_ob_row_masker_raw10():
    from runner_support import build_and_test

    build_and_test(
        block="ob_row_masker_raw10",
        sources=["rtl/img_proc/ob_row_masker.sv"],
        toplevel="ob_row_masker",
        test_module="test_ob_row_masker_raw10",
        test_dir=Path(__file__).resolve().parent,
        parameters={
            "WIDTH": WIDTH,
            "OB_THRESHOLD": TH,
            "OB_FILL_Y": FILL,
            "OB_UNIFORMITY": UNIF,
        },
        engine="verilator",
    )
