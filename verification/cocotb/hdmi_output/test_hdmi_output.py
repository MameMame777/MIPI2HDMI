"""cocotb port of verification/tb/tb_hdmi_output.sv (true AXI4-Stream slave + video timing).

The ``hdmi_output`` block is a single-clock (``pix_clk``) video timing generator that either
paints an internal color-bar test pattern (``test_pattern_en``) or consumes an AXI4-Stream
pixel frame (``s_axis_*``, with ``s_axis_tready`` gated by the active region), then drives
RGB + sync outputs and three TMDS-encoded channels.

Three scenarios replicated 1:1 from the DSim TB:
  * run_tpg_check    -- test pattern: verify TMDS control codes during blanking, color-bar
                        RGB during active, active-pixel count and the fixed TMDS clock word.
  * run_axis_check   -- drive one AXIS frame (sof->tuser[0], eol->tlast); verify each active
                        pixel equals the driven data and both sideband/underflow counters stay 0.
  * run_underflow_check -- enable with tvalid held low; verify the underflow counter increments.

The DUT samples inputs / drives outputs on posedge pix_clk; the TB observes outputs one delta
after posedge (``#1``). We mirror that by reading outputs *after* ``RisingEdge`` (post-NBA).
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import start_clock  # noqa: E402
from lib.scoreboard import check  # noqa: E402
from cocotb.triggers import ClockCycles  # noqa: E402

# --- DUT geometry (localparams from tb_hdmi_output.sv) ------------------------------------
H_ACTIVE = 8
H_FRONT_PORCH = 2
H_SYNC = 2
H_BACK_PORCH = 2
V_ACTIVE = 4
V_FRONT_PORCH = 1
V_SYNC = 1
V_BACK_PORCH = 1
H_TOTAL = H_ACTIVE + H_FRONT_PORCH + H_SYNC + H_BACK_PORCH
V_TOTAL = V_ACTIVE + V_FRONT_PORCH + V_SYNC + V_BACK_PORCH


def control_code(c0: int, c1: int) -> int:
    """TMDS control-period code (mirror of the SV function)."""
    key = ((c1 & 1) << 1) | (c0 & 1)
    return {
        0b00: 0b1101010100,
        0b01: 0b0010101011,
        0b10: 0b0101010100,
        0b11: 0b1010101011,
    }[key]


def color_bar(x_pos: int, y_pos: int) -> int:
    """Color-bar reference (mirror of the SV function). For H_ACTIVE=8 this equals the DUT's
    test_pattern_pixel: bar index = (x_pos*8)/H_ACTIVE = x_pos."""
    idx = x_pos & 0b111
    table = {
        0: 0xFFFFFF,
        1: 0xFFFF00,
        2: 0x00FFFF,
        3: 0x00FF00,
        4: 0xFF00FF,
        5: 0xFF0000,
        6: 0x0000FF,
    }
    if idx in table:
        return table[idx]
    return ((y_pos & 0xFF) << 16) | ((x_pos & 0xFF) << 8) | 0x40


def axis_pixel(index: int) -> int:
    """AXIS payload pixel (mirror of the SV function): {idx+1, 0x80+idx, 0x40+idx}."""
    r = (index + 1) & 0xFF
    g = (0x80 + index) & 0xFF
    b = (0x40 + index) & 0xFF
    return (r << 16) | (g << 8) | b


def rgb(dut) -> int:
    return (int(dut.video_r.value) << 16) | (int(dut.video_g.value) << 8) | int(dut.video_b.value)


async def reset_dut(dut, clk):
    """Mirror of the SV reset_dut task: assert reset + defaults, hold 8 clocks, release, +2."""
    dut.pix_aresetn.value = 0
    dut.enable.value = 0
    dut.soft_reset.value = 0
    dut.test_pattern_en.value = 0
    dut.hpd.value = 1
    dut.hpd_override.value = 0
    dut.s_axis_tdata.value = 0
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tlast.value = 0
    dut.s_axis_tuser.value = 0
    await ClockCycles(clk, 8)
    dut.pix_aresetn.value = 1
    await ClockCycles(clk, 2)


async def _start(dut):
    dut.pix_clk.value = 0
    start_clock(dut.pix_clk, 10.0)
    return dut.pix_clk


# =========================================================================================
@cocotb.test(timeout_time=5, timeout_unit="ms")
async def run_tpg_check(dut):
    """Test-pattern generator: TMDS control codes during blanking + color-bar RGB active."""
    clk = await _start(dut)
    await reset_dut(dut, clk)
    dut.test_pattern_en.value = 1
    dut.enable.value = 1

    expected_x = 0
    prev_de = 0
    prev_hsync = 0
    prev_vsync = 0

    for _ in range(H_TOTAL * V_TOTAL * 2):
        await RisingEdge(clk)
        # sampled one delta after the edge (== SV's #1): NBA updates have settled.
        de = int(dut.video_de.value)
        hsync = int(dut.video_hsync.value)
        vsync = int(dut.video_vsync.value)

        if not prev_de:
            check(int(dut.tmds_data_0.value) == control_code(prev_hsync, prev_vsync),
                  "blue channel control code during blanking")
            check(int(dut.tmds_data_1.value) == control_code(0, 0),
                  "green channel control code during blanking")
            check(int(dut.tmds_data_2.value) == control_code(0, 0),
                  "red channel control code during blanking")

        if de:
            expected_rgb = color_bar(expected_x % H_ACTIVE, (expected_x // H_ACTIVE) % V_ACTIVE)
            got = rgb(dut)
            if got != expected_rgb:
                raise AssertionError(
                    f"CHECK FAILED: TPG RGB pixel idx={expected_x} "
                    f"got={got:06x} expected={expected_rgb:06x}")
            expected_x += 1

        prev_de = de
        prev_hsync = hsync
        prev_vsync = vsync

    check(expected_x >= H_ACTIVE * V_ACTIVE, "TPG active pixel count")
    check(int(dut.tmds_clk_word.value) == 0b1111100000, "TMDS clock pattern")


# =========================================================================================
def _drive_index(dut, idx):
    dut.s_axis_tvalid.value = 1
    dut.s_axis_tdata.value = axis_pixel(idx)
    dut.s_axis_tuser.value = 1 if idx == 0 else 0
    dut.s_axis_tlast.value = 1 if (idx % H_ACTIVE) == (H_ACTIVE - 1) else 0


async def _drive_axis_frame(dut, clk, state):
    """Mirror of the SV drive_axis_frame task: keep a valid pixel presented every clock and
    advance to the next index whenever it was accepted (tvalid & tready both high at the edge).

    The DUT drains one active pixel per clock once ``stream_aligned`` is set, so the source
    must present a *new* pixel every cycle -- hence the present-then-sample cadence (drive
    signals right after each RisingEdge; on the next edge, if accepted, advance and re-drive
    in the same delta before the DUT samples again)."""
    idx = 0
    # Present pixel 0 before the first sampling edge (so tvalid & tuser are seen at origin).
    _drive_index(dut, idx)
    while idx < H_ACTIVE * V_ACTIVE:
        await RisingEdge(clk)
        if int(dut.s_axis_tvalid.value) == 1 and int(dut.s_axis_tready.value) == 1:
            idx += 1
            if idx < H_ACTIVE * V_ACTIVE:
                _drive_index(dut, idx)  # re-drive immediately (same delta) for next cycle

    dut.s_axis_tvalid.value = 0
    dut.s_axis_tuser.value = 0
    dut.s_axis_tlast.value = 0
    state["done"] = True


async def _check_axis_pixels(dut, clk):
    checked = 0
    while checked < H_ACTIVE * V_ACTIVE:
        await RisingEdge(clk)
        if int(dut.video_de.value) == 1:
            got = rgb(dut)
            exp = axis_pixel(checked)
            if got != exp:
                raise AssertionError(
                    f"CHECK FAILED: AXIS RGB pixel idx={checked} "
                    f"got={got:06x} expected={exp:06x}")
            checked += 1


async def _wait_frame_count(dut, clk, target):
    for _ in range(2000):
        await RisingEdge(clk)
        if int(dut.sts_frame_count.value) >= target:
            return
    raise AssertionError(f"CHECK FAILED: Timed out waiting for HDMI frame count {target}")


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def run_axis_check(dut):
    """Drive one AXIS frame; verify pixel data on video_de and clean sideband/underflow."""
    clk = await _start(dut)
    await reset_dut(dut, clk)
    dut.test_pattern_en.value = 0
    dut.enable.value = 1

    state = {"done": False}
    drive = cocotb.start_soon(_drive_axis_frame(dut, clk, state))
    checker = cocotb.start_soon(_check_axis_pixels(dut, clk))
    await drive
    await checker

    await _wait_frame_count(dut, clk, 1)
    check(int(dut.sts_axis_error_count.value) == 0, "AXIS sideband check clean")
    check(int(dut.sts_underflow_count.value) == 0, "AXIS underflow clean")


# =========================================================================================
@cocotb.test(timeout_time=5, timeout_unit="ms")
async def run_underflow_check(dut):
    """Enable with no AXIS data -> underflow counter must increment."""
    clk = await _start(dut)
    await reset_dut(dut, clk)
    dut.test_pattern_en.value = 0
    dut.enable.value = 1
    dut.s_axis_tvalid.value = 0

    for _ in range(H_TOTAL * 2):
        await RisingEdge(clk)

    check(int(dut.sts_underflow_count.value) != 0, "underflow counter increments")


# =========================================================================================
def test_hdmi_output():
    from runner_support import build_and_test

    build_and_test(
        block="hdmi_output",
        sources=["rtl/hdmi/hdmi_output.sv"],
        toplevel="hdmi_output",
        test_module="test_hdmi_output",
        test_dir=Path(__file__).resolve().parent,
        parameters={
            "H_ACTIVE": H_ACTIVE,
            "H_FRONT_PORCH": H_FRONT_PORCH,
            "H_SYNC": H_SYNC,
            "H_BACK_PORCH": H_BACK_PORCH,
            "V_ACTIVE": V_ACTIVE,
            "V_FRONT_PORCH": V_FRONT_PORCH,
            "V_SYNC": V_SYNC,
            "V_BACK_PORCH": V_BACK_PORCH,
            "HSYNC_POLARITY": 1,
            "VSYNC_POLARITY": 1,
        },
        engine="verilator",
    )
