"""cocotb port of verification/tb/tb_ob_row_masker.sv (valid-only pixel family).

Optical-Black row masker: full-line min/max statistics + ping-pong line buffer.
A line is masked (every output pixel replaced with OB_FILL_Y=128) iff
    max < OB_THRESHOLD(50) && (max - min) <= OB_UNIFORMITY(3),
otherwise it passes through unchanged. ``enable=0`` bypasses masking.

Latency is ~1 line: a line's output is emitted only after its ``in_eol`` has been
seen and the buffer marked full, then drained one pixel/cycle from the ping-pong
buffer. Each test drives its row(s), flushes until the output is idle, and checks
the captured output bytes -- mirroring the DSim TB's ``out_row`` queue 1:1.

Signals differ from the img_proc default (in_pixel/out_pixel): this DUT uses
``in_data``/``out_data`` (8-bit) with async active-low reset ``aresetn`` and an
``enable`` bypass, so the PixelStreamDriver/PixelMonitor are configured explicitly.

The random tests (16, 17) reproduce the SV ``$urandom_range(0,255)`` stimulus: a
random row's full-line range is essentially always > OB_UNIFORMITY, so it passes
through unchanged -- the same expectation the TB checks.
"""
from __future__ import annotations

import random
import sys
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import start_clock  # noqa: E402
from lib.pixel_stream import PixelMonitor  # noqa: E402
from lib.scoreboard import check  # noqa: E402

OB_FILL_Y = 128


async def _reset(dut, clk):
    """Async active-low reset held low 5 cycles, released, settle 3 (mirrors the TB)."""
    dut.in_data.value = 0
    dut.in_valid.value = 0
    dut.in_sof.value = 0
    dut.in_eol.value = 0
    dut.in_eof.value = 0
    dut.in_err.value = 0
    dut.aresetn.value = 0
    dut.enable.value = 1
    for _ in range(5):
        await RisingEdge(clk)
    dut.aresetn.value = 1
    for _ in range(3):
        await RisingEdge(clk)


async def _drive_row(dut, clk, data, is_sof=False, is_eof=False):
    """Contiguous-valid row drive, 1:1 with the TB ``drive_row`` task."""
    n = len(data)
    for i, d in enumerate(data):
        await RisingEdge(clk)
        dut.in_data.value = d
        dut.in_valid.value = 1
        dut.in_sof.value = 1 if (i == 0 and is_sof) else 0
        dut.in_eol.value = 1 if (i == n - 1) else 0
        dut.in_eof.value = 1 if (i == n - 1 and is_eof) else 0
        dut.in_err.value = 0
    await RisingEdge(clk)
    dut.in_data.value = 0
    dut.in_valid.value = 0
    dut.in_sof.value = 0
    dut.in_eol.value = 0
    dut.in_eof.value = 0


async def _wait_flush(dut, clk):
    """Poll until output has been idle for 8 cycles (mirrors ``wait_pipeline_flush``)."""
    idle = 0
    max_wait = 4096
    while idle < 8 and max_wait > 0:
        await RisingEdge(clk)
        if int(dut.out_valid.value) == 1:
            idle = 0
        else:
            idle += 1
        max_wait -= 1


def _out_data(mon, base):
    return [b["pixel"] & 0xFF for b in mon.beats[base:]]


def _check_all_equal(out_row, expected, name):
    check(len(out_row) > 0, f"{name}: no output captured")
    for i, v in enumerate(out_row):
        check(v == expected,
              f"{name}: pixel {i} = 0x{v:02x}, expected 0x{expected:02x}")


def _check_equal_array(out_row, expected, name):
    check(len(out_row) == len(expected),
          f"{name}: output size {len(out_row)} != expected size {len(expected)}")
    for i in range(min(len(out_row), len(expected))):
        check(out_row[i] == expected[i],
              f"{name}: pixel {i} = 0x{out_row[i]:02x}, expected 0x{expected[i]:02x}")


@cocotb.test(timeout_time=10, timeout_unit="ms")
async def ob_row_masker_scenarios(dut):
    clk = dut.clk
    start_clock(clk, 10.0)  # #5 half period -> 10 ns
    await _reset(dut, clk)

    mon = PixelMonitor(dut, clk, pixel="out_data", valid="out_valid",
                       sof="out_sof", eol="out_eol", eof="out_eof", err="out_err")
    mon.start()

    async def run(data, name, is_sof=False, is_eof=False):
        base = len(mon.beats)
        await _drive_row(dut, clk, data, is_sof, is_eof)
        await _wait_flush(dut, clk)
        return _out_data(mon, base)

    # Test 1: True OB row -- uniform Y=36 across 16 pixels -> mask to 128
    out = await run([36] * 16, "OB row Y=36 uniform -> mask", is_sof=True)
    _check_all_equal(out, OB_FILL_Y, "OB row Y=36 uniform -> mask")

    # Test 2: OB row with slight variation, range=2 <= 3, all < 50 -> mask
    out = await run([36, 37, 35, 36, 35, 37, 36, 35,
                     36, 36, 37, 35, 36, 35, 37, 36],
                    "OB row Y=35-37 (range 2) -> mask")
    _check_all_equal(out, OB_FILL_Y, "OB row Y=35-37 (range 2) -> mask")

    # Test 3: image row, dark first pixel only -> pass
    row = [45, 200, 210, 180, 150, 120, 100, 80,
           60, 40, 55, 75, 95, 115, 135, 155]
    out = await run(row, "image row, dark first pixel only -> pass")
    _check_equal_array(out, row, "image row, dark first pixel only -> pass")

    # Test 4: bright uniform Y=128 -> pass (above threshold)
    row = [128] * 16
    out = await run(row, "bright uniform Y=128 -> pass")
    _check_equal_array(out, row, "bright uniform Y=128 -> pass")

    # Test 5: dark but non-uniform (all < 50 but range > 3) -> pass
    row = [30, 5, 49, 2, 25, 45, 10, 40,
           35, 15, 48, 8, 20, 42, 12, 38]
    out = await run(row, "dark non-uniform -> pass")
    _check_equal_array(out, row, "dark non-uniform -> pass")

    # Test 6: uniform but >= 50 -> pass
    row = [80, 81, 80, 79, 80, 81, 79, 80,
           80, 81, 80, 79, 80, 81, 80, 79]
    out = await run(row, "uniform Y=80 above threshold -> pass")
    _check_equal_array(out, row, "uniform Y=80 above threshold -> pass")

    # Test 7: enable=0 bypass on a true OB row -> pass through unchanged
    row = [36] * 16
    dut.enable.value = 0
    out = await run(row, "enable=0 bypass on OB row -> pass")
    _check_equal_array(out, row, "enable=0 bypass on OB row -> pass")
    dut.enable.value = 1

    # Test 8: chained OB row then image row -> first masked, second passes
    ob_row = [36] * 16
    img_row = [100, 110, 120, 130, 140, 150, 160, 170,
               180, 170, 160, 150, 140, 130, 120, 110]
    base = len(mon.beats)
    await _drive_row(dut, clk, ob_row, is_sof=False, is_eof=False)
    await _drive_row(dut, clk, img_row, is_sof=False, is_eof=True)
    await _wait_flush(dut, clk)
    out = _out_data(mon, base)
    check(len(out) == 32, f"chained OB+img: size {len(out)} != 32")
    if len(out) == 32:
        for i in range(16):
            check(out[i] == OB_FILL_Y,
                  f"chained OB+img: OB pixel {i} = 0x{out[i]:02x}, expected 0x80")
        for i in range(16):
            check(out[16 + i] == img_row[i],
                  f"chained OB+img: img pixel {i} = 0x{out[16 + i]:02x}, "
                  f"expected 0x{img_row[i]:02x}")

    # Test 9: checkerboard 8 dark + 8 bright -> pass (full-line range 230)
    row = [10] * 8 + [240] * 8
    out = await run(row, "checkerboard row (8 dark + 8 bright) -> pass")
    _check_equal_array(out, row, "checkerboard row (8 dark + 8 bright) -> pass")

    # Test 10: checkerboard 2px blocks -> pass
    row = [10, 10, 240, 240, 10, 10, 240, 240,
           10, 10, 240, 240, 10, 10, 240, 240]
    out = await run(row, "checkerboard 2px blocks -> pass")
    _check_equal_array(out, row, "checkerboard 2px blocks -> pass")

    # Test 11: grayscale uniform Y=64 (above threshold) -> pass
    row = [64] * 16
    out = await run(row, "grayscale uniform Y=64 -> pass")
    _check_equal_array(out, row, "grayscale uniform Y=64 -> pass")

    # Test 12: grayscale uniform Y=200 -> pass
    row = [200] * 16
    out = await run(row, "grayscale uniform Y=200 -> pass")
    _check_equal_array(out, row, "grayscale uniform Y=200 -> pass")

    # Test 13: gradient 0->240 -> pass (full-line range 240)
    row = [0, 16, 32, 48, 64, 80, 96, 112,
           128, 144, 160, 176, 192, 208, 224, 240]
    out = await run(row, "gradient 0->240 -> pass")
    _check_equal_array(out, row, "gradient 0->240 -> pass")

    # Test 14: gradient tight dark start -> pass (full-line range = 230)
    row = [10, 11, 12, 13, 50, 80, 110, 140,
           170, 200, 230, 240, 230, 200, 170, 140]
    out = await run(row, "gradient tight dark start -> pass")
    _check_equal_array(out, row, "gradient tight dark start -> pass")

    # Test 15: reverse gradient 240->0 -> pass
    row = [240, 224, 208, 192, 176, 160, 144, 128,
           112, 96, 80, 64, 48, 32, 16, 0]
    out = await run(row, "reverse gradient 240->0 -> pass")
    _check_equal_array(out, row, "reverse gradient 240->0 -> pass")

    # Test 16: random pixel pattern -> pass (wide range)
    rng = random.Random(0xC0C07B)
    row = [rng.randint(0, 255) for _ in range(16)]
    out = await run(row, "random pixel pattern -> pass")
    _check_equal_array(out, row, "random pixel pattern -> pass")

    # Test 17: multiple random rows (stress) -> pass
    for trial in range(5):
        row = [rng.randint(0, 255) for _ in range(16)]
        out = await run(row, f"random trial {trial} -> pass")
        _check_equal_array(out, row, f"random trial {trial} -> pass")


def test_ob_row_masker():
    from runner_support import build_and_test

    build_and_test(
        block="ob_row_masker",
        sources=["rtl/img_proc/ob_row_masker.sv"],
        toplevel="ob_row_masker",
        test_module="test_ob_row_masker",
        test_dir=Path(__file__).resolve().parent,
        parameters={
            "WIDTH": 8,
            "LINE_PIXELS_MAX": 1024,
            "OB_THRESHOLD": 50,
            "OB_FILL_Y": 128,
            "OB_UNIFORMITY": 3,
        },
        engine="verilator",
    )
