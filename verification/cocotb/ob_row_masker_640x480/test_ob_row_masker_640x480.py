"""cocotb port of verification/tb/tb_ob_row_masker_640x480.sv (valid-only pixel family).

640x480 burst-mode verification of ob_row_masker. Drives realistic patterns at full
hardware frame size with bursty input (solid 640-cycle line bursts + inter-line LP-style
idle gaps) and verifies, per frame, that:
  1. All H x W pixels arrive at the output bit-exact (no over-/under-firing).
  2. eol/sof/eof markers fire exactly H/1/1 times.
  3. Every output line length == W.
  4. No pixels are lost or duplicated across the ping-pong boundary.

All test patterns are above the OB threshold (or fail uniformity), so the masker MUST pass
them through unchanged. Same DUT + same stimulus as the DSim TB; the checks replicate
``check_pass`` 1:1. Test 6 (two frames back-to-back) mirrors the TB's count-only check.

The DUT uses async active-low reset (``aresetn``), an ``enable`` input, and 8-bit
``in_data``/``out_data`` (no tready).
"""
from __future__ import annotations

import random
import sys
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import bringup  # noqa: E402
from lib.pixel_stream import PixelMonitor  # noqa: E402
from lib.scoreboard import check  # noqa: E402

W = 640
H = 480


async def drive_frame(dut, clk, frame, interline_gap_cycles):
    """Replicate the DSim ``drive_frame`` task: solid W-cycle line bursts separated by
    ``interline_gap_cycles`` idle cycles. ``frame`` is a list of H rows, each a list of W
    8-bit values."""
    for row in range(H):
        line = frame[row]
        for col in range(W):
            await RisingEdge(clk)
            dut.in_data.value = line[col]
            dut.in_valid.value = 1
            dut.in_sof.value = 1 if (row == 0 and col == 0) else 0
            dut.in_eol.value = 1 if col == W - 1 else 0
            dut.in_eof.value = 1 if (row == H - 1 and col == W - 1) else 0
            dut.in_err.value = 0
        for _ in range(interline_gap_cycles):
            await RisingEdge(clk)
            dut.in_data.value = 0
            dut.in_valid.value = 0
            dut.in_sof.value = 0
            dut.in_eol.value = 0
            dut.in_eof.value = 0
    await RisingEdge(clk)
    dut.in_valid.value = 0
    dut.in_sof.value = 0
    dut.in_eol.value = 0
    dut.in_eof.value = 0


async def wait_drain(dut, clk, idle_target=4000, budget=4_000_000):
    """Wait until ``out_valid`` has been idle for ``idle_target`` consecutive cycles (mirror
    of the DSim ``wait_drain`` task)."""
    idle = 0
    while idle < idle_target and budget > 0:
        await RisingEdge(clk)
        if int(dut.out_valid.value) == 1:
            idle = 0
        else:
            idle += 1
        budget -= 1


def check_pass(name, beats, frame):
    """Replicate the DSim ``check_pass`` task: reconstruct the output frame from the captured
    beats and verify pixel count, marker counts, per-line length, and bit-exact data.
    ``beats`` are the PixelMonitor dicts captured for exactly this frame."""
    pixel_count = len(beats)
    check(pixel_count == W * H,
          f"{name}: captured {pixel_count} pixels, expected {W * H}")

    sof_count = sum(b["sof"] for b in beats)
    eol_count = sum(b["eol"] for b in beats)
    eof_count = sum(b["eof"] for b in beats)
    check(sof_count == 1, f"{name}: sof_count={sof_count} expected 1")
    check(eol_count == H, f"{name}: eol_count={eol_count} expected {H}")
    check(eof_count == 1, f"{name}: eof_count={eof_count} expected 1")

    # Reconstruct per-line data and lengths exactly like the TB's out_row/out_col walk:
    # advance the column each beat, advance the row (and reset col) on each eol beat.
    per_line_count = [0] * H
    out_frame = [[0] * W for _ in range(H)]
    out_row = 0
    out_col = 0
    for b in beats:
        if out_row < H and out_col < W:
            out_frame[out_row][out_col] = b["pixel"]
        if out_row < H:
            per_line_count[out_row] += 1
        if b["eol"]:
            out_row += 1
            out_col = 0
        else:
            out_col += 1

    for r in range(H):
        check(per_line_count[r] == W,
              f"{name}: row {r} had {per_line_count[r]} pixels, expected {W}")

    for r in range(H):
        for c in range(W):
            check(out_frame[r][c] == frame[r][c],
                  f"{name}: out[{r}][{c}]=0x{out_frame[r][c]:02x} "
                  f"expected 0x{frame[r][c]:02x}")


async def _setup(dut):
    clk, _ = await bringup(dut, clk="clk", rst="aresetn", cycles=5, post=3)
    dut.enable.value = 1
    dut.in_data.value = 0
    dut.in_valid.value = 0
    dut.in_sof.value = 0
    dut.in_eol.value = 0
    dut.in_eof.value = 0
    dut.in_err.value = 0
    mon = PixelMonitor(dut, clk, pixel="out_data")
    mon.start()
    return clk, mon


async def _run_one(dut, name, frame, gap):
    clk, mon = await _setup(dut)
    base = len(mon.beats)
    await drive_frame(dut, clk, frame, gap)
    await wait_drain(dut, clk)
    check_pass(name, mon.beats[base:], frame)


@cocotb.test(timeout_time=200, timeout_unit="ms")
async def checkerboard_32x32(dut):
    frame = [[240 if (((r >> 5) + (c >> 5)) & 1) else 10 for c in range(W)]
             for r in range(H)]
    await _run_one(dut, "Checkerboard 32x32 burst+gap=50", frame, 50)


@cocotb.test(timeout_time=200, timeout_unit="ms")
async def horizontal_gradient(dut):
    frame = [[(c * 256) // W for c in range(W)] for _ in range(H)]
    await _run_one(dut, "Horizontal gradient burst+gap=0", frame, 0)


@cocotb.test(timeout_time=200, timeout_unit="ms")
async def vertical_stripe(dut):
    frame = [[200 if ((c >> 5) & 1) else 60 for c in range(W)] for _ in range(H)]
    await _run_one(dut, "Vertical stripe burst+gap=100", frame, 100)


@cocotb.test(timeout_time=200, timeout_unit="ms")
async def horizontal_stripe(dut):
    frame = [[220 if ((r >> 4) & 1) else 70 for c in range(W)] for r in range(H)]
    await _run_one(dut, "Horizontal stripe burst+gap=20", frame, 20)


@cocotb.test(timeout_time=200, timeout_unit="ms")
async def random_pattern(dut):
    rng = random.Random(0xB055)
    # Above OB threshold so masker MUST pass (range > UNIFORMITY across a full line).
    frame = [[rng.randint(60, 255) for _ in range(W)] for _ in range(H)]
    await _run_one(dut, "Random burst+gap=30", frame, 30)


@cocotb.test(timeout_time=300, timeout_unit="ms")
async def two_frames_back_to_back(dut):
    """Drive two frames back-to-back and verify aggregate counts (mirror of the TB, which
    skips the bit-check for this scenario and only checks 2x counts)."""
    clk, mon = await _setup(dut)
    frame = [[230 if (((r >> 4) + (c >> 4)) & 1) else 80 for c in range(W)]
             for r in range(H)]
    base = len(mon.beats)
    await drive_frame(dut, clk, frame, 40)  # frame 1
    await drive_frame(dut, clk, frame, 40)  # frame 2
    await wait_drain(dut, clk)
    beats = mon.beats[base:]

    pixel_count = len(beats)
    eol_count = sum(b["eol"] for b in beats)
    sof_count = sum(b["sof"] for b in beats)
    eof_count = sum(b["eof"] for b in beats)
    check(pixel_count == 2 * W * H,
          f"Back-to-back: pixel_count={pixel_count} expected {2 * W * H}")
    check(eol_count == 2 * H,
          f"Back-to-back: eol_count={eol_count} expected {2 * H}")
    check(sof_count == 2 and eof_count == 2,
          f"Back-to-back: sof_count={sof_count} eof_count={eof_count} both should be 2")


def test_ob_row_masker_640x480():
    from runner_support import build_and_test

    build_and_test(
        block="ob_row_masker_640x480",
        sources=["rtl/img_proc/ob_row_masker.sv"],
        toplevel="ob_row_masker",
        test_module="test_ob_row_masker_640x480",
        test_dir=Path(__file__).resolve().parent,
        parameters={
            "WIDTH": 8,
            "LINE_PIXELS_MAX": 1024,
            "OB_THRESHOLD": 50,
            "OB_FILL_Y": 128,
            "OB_UNIFORMITY": 12,
        },
        engine="verilator",
    )
