"""cocotb port of verification/tb/tb_yuv422_crc_framebuffer_axis.sv.

Dual-clock CRC-gated framebuffer: packet/payload/CRC inputs are driven on ``core_clk``
(#5 -> 10 ns); the true AXI4-Stream output (m_axis_t{data,valid,ready,last,user}) is emitted
on ``pix_clk`` (#7 -> 14 ns). A YUV422-8 long packet (DT 0x1e, wc==LINE_BYTES) is captured
byte-by-byte; on ``crc_check_valid`` the line is either replayed into the framebuffer (good
CRC + no capture error + full byte count) or counted as a bad line. Replayed lines are read
out on pix_clk with luma-triplicated 24-bit data, tlast at end-of-line, and tuser[0] at
(0,0). TB params: WIDTH=4, HEIGHT=3, LINE_BYTES=8, TDATA_WIDTH=24.

Scenarios (1:1 with the SV ``initial`` block):
  1. bad-CRC line: bumps bad_line_count, no frame_ready, good_line_count stays 0.
  2. good-CRC line 1: frame_ready, frame_count==1, write_line==1, AXIS shows Y0..Y3.
  3. good-CRC line 2: frame_count==2, write_line==2, AXIS shows the new Y0..Y3.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.axis import AxisMonitor  # noqa: E402
from lib.scoreboard import check  # noqa: E402
from cocotb.clock import Clock  # noqa: E402
from cocotb.triggers import ClockCycles  # noqa: E402

WIDTH = 4
HEIGHT = 3
LINE_BYTES = WIDTH * 2


async def reset_dut(dut):
    """Mirror tb reset_dut(): both clocks running, resets low, inputs idle, then release."""
    dut.core_aresetn.value = 0
    dut.pix_aresetn.value = 0
    dut.pkt_di.value = 0
    dut.pkt_wc.value = 0
    dut.pkt_is_short.value = 0
    dut.pkt_is_long.value = 0
    dut.pkt_start.value = 0
    dut.pkt_end.value = 0
    dut.pkt_err.value = 0
    dut.payload_data.value = 0
    dut.payload_valid.value = 0
    dut.payload_first.value = 0
    dut.payload_last.value = 0
    dut.crc_check_valid.value = 0
    dut.crc_match.value = 0
    dut.m_axis_tready.value = 1

    # core_clk = #5 (10 ns), pix_clk = #7 (14 ns)
    cocotb.start_soon(Clock(dut.core_clk, 10.0, unit="ns").start())
    cocotb.start_soon(Clock(dut.pix_clk, 14.0, unit="ns").start())

    await ClockCycles(dut.core_clk, 6)
    dut.core_aresetn.value = 1
    await ClockCycles(dut.pix_clk, 6)
    dut.pix_aresetn.value = 1
    await ClockCycles(dut.core_clk, 4)


async def start_yuv_packet(dut, packet_err):
    await RisingEdge(dut.core_clk)
    dut.pkt_di.value = 0x1E
    dut.pkt_wc.value = LINE_BYTES
    dut.pkt_is_short.value = 0
    dut.pkt_is_long.value = 1
    dut.pkt_start.value = 1
    dut.pkt_end.value = 0
    dut.pkt_err.value = int(packet_err)
    await RisingEdge(dut.core_clk)
    dut.pkt_start.value = 0
    dut.pkt_err.value = 0


async def drive_payload_line(dut, y0, y1, y2, y3):
    line_bytes = [0x80, y0, 0x81, y1, 0x82, y2, 0x83, y3]
    for idx in range(LINE_BYTES):
        await RisingEdge(dut.core_clk)
        dut.payload_data.value = line_bytes[idx]
        dut.payload_valid.value = 1
        dut.payload_first.value = 1 if idx == 0 else 0
        dut.payload_last.value = 1 if idx == LINE_BYTES - 1 else 0
    await RisingEdge(dut.core_clk)
    dut.payload_valid.value = 0
    dut.payload_first.value = 0
    dut.payload_last.value = 0
    dut.pkt_end.value = 1
    await RisingEdge(dut.core_clk)
    dut.pkt_end.value = 0


async def finish_crc(dut, good_crc):
    await RisingEdge(dut.core_clk)
    dut.crc_match.value = int(good_crc)
    dut.crc_check_valid.value = 1
    await RisingEdge(dut.core_clk)
    dut.crc_check_valid.value = 0
    dut.crc_match.value = 0
    dut.pkt_is_long.value = 0


async def send_line(dut, good_crc, y0, y1, y2, y3):
    await start_yuv_packet(dut, 0)
    await drive_payload_line(dut, y0, y1, y2, y3)
    await finish_crc(dut, good_crc)


async def wait_core_replay_done(dut, expected_good_lines):
    for _ in range(100):
        await RisingEdge(dut.core_clk)
        if int(dut.sts_good_line_count.value) == (expected_good_lines & 0xFFFF):
            return
    raise AssertionError(
        f"CHECK FAILED: Timed out waiting for good line count {expected_good_lines}")


async def wait_bad_count(dut, expected_bad_lines):
    for _ in range(20):
        await RisingEdge(dut.core_clk)
        if int(dut.sts_bad_line_count.value) == (expected_bad_lines & 0xFFFF):
            return
    raise AssertionError(
        f"CHECK FAILED: Timed out waiting for bad line count {expected_bad_lines}")


async def wait_axis_sof(dut):
    """Wait for an accepted AXIS beat carrying tuser[0] (start of frame)."""
    for _ in range(200):
        await RisingEdge(dut.pix_clk)
        if (int(dut.m_axis_tvalid.value) == 1
                and int(dut.m_axis_tready.value) == 1
                and (int(dut.m_axis_tuser.value) & 0x1) == 1):
            return
    raise AssertionError("CHECK FAILED: Timed out waiting for AXIS SOF")


async def check_next_line(dut, y0, y1, y2, y3):
    """Replicate tb check_next_line(): SOF beat + WIDTH pixels, luma-triplicated, tlast at end."""
    expected = [y0, y1, y2, y3]
    await wait_axis_sof(dut)
    for idx in range(WIDTH):
        if idx != 0:
            await RisingEdge(dut.pix_clk)
        check(int(dut.m_axis_tvalid.value) == 1, "AXIS valid during displayed line")
        got = int(dut.m_axis_tdata.value)
        exp = (expected[idx] << 16) | (expected[idx] << 8) | expected[idx]
        if got != exp:
            raise AssertionError(
                f"CHECK FAILED: displayed pixel {idx} got={got:06x} expected={exp:06x}")
        check(int(dut.m_axis_tlast.value) == (1 if idx == WIDTH - 1 else 0),
              "AXIS tlast position")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def yuv422_crc_framebuffer(dut):
    await reset_dut(dut)

    # 1) bad CRC line: no frame ready, no good lines, one bad line.
    await send_line(dut, 0, 0x10, 0x20, 0x30, 0x40)
    await wait_bad_count(dut, 1)
    check(int(dut.sts_frame_ready.value) == 0, "bad CRC does not ready frame")
    check(int(dut.sts_good_line_count.value) == 0, "bad CRC does not increment good lines")

    # 2) good CRC line 1: frame ready, frame count 1, write line advances, AXIS replays line.
    await send_line(dut, 1, 0x11, 0x22, 0x33, 0x44)
    await wait_core_replay_done(dut, 1)
    check(int(dut.sts_frame_ready.value) == 1, "good CRC readies frame")
    check(int(dut.sts_frame_count.value) == 1, "good CRC increments frame count once")
    check(int(dut.sts_write_line.value) == 1, "write line advances after good replay")
    await check_next_line(dut, 0x11, 0x22, 0x33, 0x44)

    # 3) second good CRC line: frame count 2, write line advances, AXIS replays new line.
    await send_line(dut, 1, 0x55, 0x66, 0x77, 0x88)
    await wait_core_replay_done(dut, 2)
    check(int(dut.sts_frame_count.value) == 2, "second good CRC increments frame count")
    check(int(dut.sts_write_line.value) == 2, "write line advances again")
    await check_next_line(dut, 0x55, 0x66, 0x77, 0x88)


def test_yuv422_crc_framebuffer_axis():
    from runner_support import build_and_test

    build_and_test(
        block="yuv422_crc_framebuffer_axis",
        sources=["rtl/img_proc/yuv422_crc_framebuffer_axis.sv"],
        toplevel="yuv422_crc_framebuffer_axis",
        test_module="test_yuv422_crc_framebuffer_axis",
        test_dir=Path(__file__).resolve().parent,
        parameters={"WIDTH": WIDTH, "HEIGHT": HEIGHT,
                    "LINE_BYTES": LINE_BYTES, "TDATA_WIDTH": 24},
        engine="verilator",
    )
