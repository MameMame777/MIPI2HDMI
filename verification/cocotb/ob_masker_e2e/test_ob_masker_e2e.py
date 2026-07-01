"""cocotb port of verification/tb/tb_ob_masker_e2e.sv (full MIPI->HDMI E2E, OB masker DUT).

The DSim TB chains the real RTL:

    s_byte_* -> csi2_packet_parser -> csi2_header_ecc / csi2_payload_crc
             -> csi2_vcdt_filter -> csi2_frame_state -> yuv422_gray_unpack
             -> ob_row_masker (DUT) -> {Y,Y,Y} -> axis_video_bridge
             == bridge AXIS out ==>  (behavioural VDMA framebuffer)  -> hdmi_output
             -> {video_r,video_g,video_b}

and feeds a *behavioural* VDMA frame-buffer coded in the TB (SV queue ``fb_storage``):
it captures the whole bridge AXIS frame on aclk, then plays it back into hdmi_output's
AXIS input at HDMI rate. This models real VDMA (S2MM writes a full frame to DDR, MM2S
reads it back at pix_clk) and avoids the y8<->y32 round-trip gaps that would underflow
HDMI at this tiny frame size.

cocotb needs a single HDL toplevel, so the 8-DUT wiring is emitted as ``ob_masker_e2e_harness``
(the real RTL only, no clock / no initial / no framebuffer). The SV framebuffer capture,
playback FSM, the yuv/ob debug taps (always_ff), and the final pixel check all become cocotb
coroutines/logic, replicating the register-transfer semantics 1:1.

Frame (1:1 with the TB): LINE_PIXELS=16, FRAME_LINES=4.
  Line 0: OB row (uniform Y=36)                -> masker replaces with Y=128
  Line 1: checkerboard (8x Y=10 + 8x Y=240)    -> pass-through
  Line 2: gradient 0..240                       -> pass-through
  Line 3: OB row w/ variation (Y=38/39, range=1)-> masker replaces with Y=128
HDMI output is checked pixel-by-pixel against ``EXPECTED_Y``.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge, ReadOnly

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import start_clock  # noqa: E402
from lib.scoreboard import check  # noqa: E402

# --- TB localparams (1:1) -------------------------------------------------------------
PARSER_IN_WIDTH = 16
DT_FS = 0x00
DT_FE = 0x01
DT_YUV422 = 0x1E

LINE_PIXELS = 16
FRAME_LINES = 4
LINE_BYTES = LINE_PIXELS * 2
FRAME_PIXELS = LINE_PIXELS * FRAME_LINES

# What MIPI sends (Y component of YUYV), 1:1 with the TB input_y[].
INPUT_Y = [
    # Line 0: OB row (uniform Y=36)
    36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36,
    # Line 1: checkerboard (8 dark Y=10 + 8 bright Y=240)
    10, 10, 10, 10, 10, 10, 10, 10, 240, 240, 240, 240, 240, 240, 240, 240,
    # Line 2: gradient 0->240
    0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240,
    # Line 3: OB row with slight variation (uniform Y=38-39, range=1)
    38, 39, 38, 39, 38, 39, 38, 39, 38, 39, 38, 39, 38, 39, 38, 39,
]

# What HDMI should output, 1:1 with the TB expected_y[].
EXPECTED_Y = [
    # Line 0: OB -> masked to Y=128
    128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128,
    # Line 1: checkerboard -> pass-through
    10, 10, 10, 10, 10, 10, 10, 10, 240, 240, 240, 240, 240, 240, 240, 240,
    # Line 2: gradient -> pass-through
    0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240,
    # Line 3: OB with variation -> masked to Y=128
    128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128,
]


# --- SV helper functions, ported 1:1 --------------------------------------------------

def calc_ecc6(data: int) -> int:
    """Port of calc_ecc6(data[23:0])."""
    def b(i):
        return (data >> i) & 1
    e = [0] * 6
    e[0] = b(0) ^ b(1) ^ b(2) ^ b(4) ^ b(5) ^ b(7) ^ b(10) ^ b(11) ^ b(13) ^ b(16) ^ b(20) ^ b(21) ^ b(22) ^ b(23)
    e[1] = b(0) ^ b(1) ^ b(3) ^ b(4) ^ b(6) ^ b(8) ^ b(10) ^ b(12) ^ b(14) ^ b(17) ^ b(20) ^ b(21) ^ b(22) ^ b(23)
    e[2] = b(0) ^ b(2) ^ b(3) ^ b(5) ^ b(6) ^ b(9) ^ b(11) ^ b(12) ^ b(15) ^ b(18) ^ b(20) ^ b(21) ^ b(22)
    e[3] = b(1) ^ b(2) ^ b(3) ^ b(7) ^ b(8) ^ b(9) ^ b(13) ^ b(14) ^ b(15) ^ b(19) ^ b(20) ^ b(21) ^ b(23)
    e[4] = b(4) ^ b(5) ^ b(6) ^ b(7) ^ b(8) ^ b(9) ^ b(16) ^ b(17) ^ b(18) ^ b(19) ^ b(20) ^ b(22) ^ b(23)
    e[5] = b(10) ^ b(11) ^ b(12) ^ b(13) ^ b(14) ^ b(15) ^ b(16) ^ b(17) ^ b(18) ^ b(19) ^ b(21) ^ b(22) ^ b(23)
    return sum(e[i] << i for i in range(6))


def make_ecc(di: int, wc: int) -> int:
    """Port of make_ecc(di, wc) = {2'b00, calc_ecc6({wc, di})}."""
    return calc_ecc6(((wc & 0xFFFF) << 8) | (di & 0xFF)) & 0x3F


def crc_update_byte(crc_in: int, data: int) -> int:
    """Port of crc_update_byte: reflected CRC-16, poly 0x8408."""
    crc = crc_in & 0xFFFF
    for bit_idx in range(8):
        feedback = (crc & 1) ^ ((data >> bit_idx) & 1)
        crc >>= 1
        if feedback:
            crc ^= 0x8408
    return crc & 0xFFFF


# --- MIPI byte-beat stimulus, ported 1:1 from the SV tasks ----------------------------
# The SV tasks drive at @(negedge core_clk); with cocotb we drive right after
# RisingEdge(core_clk) which places stable values well before the next sampling edge --
# equivalent single-beat cadence (each drive_beat holds valid for exactly one cycle).

async def drive_idle(dut, clk, cycles: int):
    dut.s_byte_valid.value = 0
    dut.s_byte_keep.value = 0
    dut.s_byte_sop.value = 0
    dut.s_byte_eop.value = 0
    dut.s_byte_data.value = 0
    for _ in range(cycles):
        await RisingEdge(clk)


async def drive_beat(dut, clk, byte0: int, byte1: int, sop: int, eop: int):
    """Port of drive_beat: one beat, valid asserted for a single core_clk cycle."""
    await RisingEdge(clk)
    dut.s_byte_data.value = ((byte1 & 0xFF) << 8) | (byte0 & 0xFF)
    dut.s_byte_keep.value = 0b11
    dut.s_byte_valid.value = 1
    dut.s_byte_sop.value = sop
    dut.s_byte_eop.value = eop
    # hold one cycle then the caller issues the next drive_beat (RisingEdge) which
    # overwrites; deassert here so a trailing beat idles cleanly.
    await RisingEdge(clk)
    dut.s_byte_valid.value = 0
    dut.s_byte_sop.value = 0
    dut.s_byte_eop.value = 0
    dut.s_byte_keep.value = 0
    dut.s_byte_data.value = 0


async def send_short_packet(dut, clk, dt: int, data_field: int):
    """Port of send_short_packet: 2 header beats (DI, DF[7:0]) / (DF[15:8], ECC), idle 2."""
    di = dt & 0x3F
    ecc = make_ecc(di, data_field)
    await drive_beat(dut, clk, di, data_field & 0xFF, 1, 0)
    await drive_beat(dut, clk, (data_field >> 8) & 0xFF, ecc, 0, 1)
    await drive_idle(dut, clk, 2)


async def send_yuv422_line(dut, clk, line_idx: int):
    """Port of send_yuv422_line: header (DI,WC[7:0]) / (WC[15:8],ECC), LINE_BYTES payload
    bytes (chroma 0x80, Y from INPUT_Y) two per beat, then CRC footer beat, idle 8."""
    di = DT_YUV422 & 0x3F
    wc = LINE_BYTES
    ecc = make_ecc(di, wc)
    base = line_idx * LINE_PIXELS

    payload = [0] * LINE_BYTES
    for p in range(LINE_PIXELS):
        payload[2 * p] = 0x80
        payload[2 * p + 1] = INPUT_Y[base + p]

    crc = 0xFFFF
    for i in range(LINE_BYTES):
        crc = crc_update_byte(crc, payload[i])

    # header beat 1 (DI, WC[7:0])
    await drive_beat(dut, clk, di, wc & 0xFF, 1, 0)
    # header beat 2 (WC[15:8], ECC)
    await drive_beat(dut, clk, (wc >> 8) & 0xFF, ecc, 0, 0)
    # payload beats: 2 bytes per beat
    for i in range(0, LINE_BYTES, 2):
        await drive_beat(dut, clk, payload[i], payload[i + 1], 0, 0)
    # CRC footer beat
    await drive_beat(dut, clk, crc & 0xFF, (crc >> 8) & 0xFF, 0, 1)
    await drive_idle(dut, clk, 8)


# --- behavioural debug taps + framebuffer (the SV always_ff / always_comb blocks) ------

class DebugTaps:
    """Port of the core_clk always_ff: push yuv_pixel[7:0] on yuv_pixel_valid, ob_data on
    ob_valid."""

    def __init__(self, dut, clk):
        self.dut = dut
        self.clk = clk
        self.yuv_capture = []
        self.ob_capture = []

    def start(self):
        return cocotb.start_soon(self._run())

    async def _run(self):
        d = self.dut
        while True:
            await RisingEdge(self.clk)
            if int(d.yuv_pixel_valid.value):
                self.yuv_capture.append(int(d.yuv_pixel_lo.value))
            if int(d.ob_valid.value):
                self.ob_capture.append(int(d.ob_data.value))


class FrameBufferCapture:
    """Port of the aclk always_ff capturing the bridge AXIS output into fb_storage.

    On (bridge_axis_tvalid && bridge_axis_tready) push {data[7:0], sof=tuser[0], last=tlast}.
    Reset clears the queue. bridge_axis_tready is tied high (assign br_tready=1) in cocotb.
    """

    def __init__(self, dut, clk):
        self.dut = dut
        self.clk = clk
        self.fb_storage = []   # list of dict(data, sof, last)

    def start(self):
        return cocotb.start_soon(self._run())

    async def _run(self):
        d = self.dut
        while True:
            await RisingEdge(self.clk)
            if not int(d.aresetn.value):
                self.fb_storage.clear()
            elif int(d.bridge_axis_tvalid.value) and int(d.bridge_axis_tready.value):
                self.fb_storage.append({
                    "data": int(d.bridge_axis_tdata.value) & 0xFF,
                    "sof": int(d.bridge_axis_tuser.value) & 0x1,
                    "last": int(d.bridge_axis_tlast.value) & 0x1,
                })


class FrameBufferPlayback:
    """Port of the SV playback (always_comb driving hdmi_in_* from fb_storage[fb_play_idx],
    always_ff advancing fb_play_idx on hdmi_in_tvalid && hdmi_in_tready).

    Enabled only after fb_playback_en is set. Presents the current entry combinationally;
    advances the index on an accepted beat. Mirrors the SV exactly: tdata={3{data}},
    tvalid when in-range, tlast=entry.last, tuser=entry.sof.
    """

    def __init__(self, dut, clk, fb):
        self.dut = dut
        self.clk = clk
        self.fb = fb
        self.enable = False
        self.play_idx = 0

    def start(self):
        return cocotb.start_soon(self._run())

    def _present(self):
        d = self.dut
        store = self.fb.fb_storage
        if self.enable and self.play_idx < len(store):
            e = store[self.play_idx]
            data = e["data"] & 0xFF
            d.hdmi_in_tdata.value = (data << 16) | (data << 8) | data
            d.hdmi_in_tvalid.value = 1
            d.hdmi_in_tlast.value = e["last"]
            d.hdmi_in_tuser.value = e["sof"]
        else:
            d.hdmi_in_tdata.value = 0
            d.hdmi_in_tvalid.value = 0
            d.hdmi_in_tlast.value = 0
            d.hdmi_in_tuser.value = 0

    async def _run(self):
        d = self.dut
        # SV: always_comb drives hdmi_in_* from fb_storage[fb_play_idx] (combinational,
        # stable through the whole cycle incl. the posedge that samples it); always_ff
        # advances fb_play_idx on (hdmi_in_tvalid && hdmi_in_tready). Model: at each
        # cycle boundary present the CURRENT index (holds through the next posedge),
        # sample accept in ReadOnly, and advance the index for the following cycle.
        self._present()
        while True:
            await RisingEdge(self.clk)
            self._present()
            await ReadOnly()
            accept = int(d.hdmi_in_tvalid.value) and int(d.hdmi_in_tready.value)
            if accept:
                self.play_idx += 1


# --- the test: replicate the single SV initial run 1:1 --------------------------------

async def wait_frame_done(dut, clk):
    """Port of wait_frame_done: wait until frame_count == 1 (<= 8000 cycles)."""
    for _ in range(8000):
        await RisingEdge(clk)
        if int(dut.frame_count.value) == 1:
            return
    raise AssertionError("CHECK FAILED: Timed out waiting for CSI-2 frame completion")


async def wait_parser_short_count(dut, clk, expected):
    """Port of wait_parser_short_count."""
    for _ in range(4000):
        await RisingEdge(clk)
        if int(dut.parser_short_count.value) >= expected:
            return
    raise AssertionError(
        f"CHECK FAILED: Timed out waiting for parser short count {expected}, "
        f"saw {int(dut.parser_short_count.value)}")


@cocotb.test(timeout_time=40, timeout_unit="ms")
async def ob_masker_e2e(dut):
    core_clk = dut.core_clk
    aclk = dut.aclk

    # Two async clocks: core_clk #5 (10 ns), aclk #7 (14 ns) -- exactly the SV periods.
    start_clock(core_clk, 10.0)
    start_clock(aclk, 14.0)

    # --- reset_dut() ---
    dut.core_aresetn.value = 0
    dut.aresetn.value = 0
    dut.s_byte_data.value = 0
    dut.s_byte_keep.value = 0
    dut.s_byte_valid.value = 0
    dut.s_byte_sop.value = 0
    dut.s_byte_eop.value = 0
    dut.hdmi_enable.value = 0
    dut.bridge_axis_tready.value = 1     # SV: assign bridge_axis_tready = 1'b1
    dut.hdmi_in_tdata.value = 0
    dut.hdmi_in_tvalid.value = 0
    dut.hdmi_in_tlast.value = 0
    dut.hdmi_in_tuser.value = 0
    for _ in range(8):
        await RisingEdge(core_clk)
    dut.core_aresetn.value = 1
    for _ in range(8):
        await RisingEdge(aclk)
    dut.aresetn.value = 1
    for _ in range(4):
        await RisingEdge(core_clk)

    taps = DebugTaps(dut, core_clk)
    taps.start()
    fb = FrameBufferCapture(dut, aclk)
    fb.start()
    playback = FrameBufferPlayback(dut, aclk, fb)
    playback.start()

    # --- stimulus: FS, 4 YUV422 lines, FE ---
    await send_short_packet(dut, core_clk, DT_FS, 0x0000)
    for line_idx in range(FRAME_LINES):
        await send_yuv422_line(dut, core_clk, line_idx)
    await send_short_packet(dut, core_clk, DT_FE, 0x0000)

    await wait_frame_done(dut, core_clk)
    await wait_parser_short_count(dut, core_clk, 2)

    # --- CSI-2-level checks (1:1 with the TB) ---
    check(int(dut.parser_short_count.value) >= 2, "parser saw FS and FE")
    check(int(dut.parser_long_count.value) == FRAME_LINES, "one long packet per line")
    check(int(dut.crc_err_count.value) == 0, "no CRC errors")
    check(int(dut.parser_trunc_count.value) == 0, "no parser truncation")
    check(int(dut.last_frame_lines.value) == FRAME_LINES, "frame state saw all lines")

    # Let the pipeline fully drain into the HDMI-side buffers before the consumer starts.
    for _ in range(2000):
        await RisingEdge(aclk)

    # --- check_hdmi_pixels() ---
    dut._log.info(
        "yuv_capture=%d ob_capture=%d fb_storage=%d (expect %d)",
        len(taps.yuv_capture), len(taps.ob_capture), len(fb.fb_storage), FRAME_PIXELS)
    check(len(taps.yuv_capture) == FRAME_PIXELS, "yuv_unpack emitted all pixels")
    check(len(taps.ob_capture) == FRAME_PIXELS, "ob_masker emitted all pixels")
    check(len(fb.fb_storage) == FRAME_PIXELS, "framebuffer captured all pixels")

    underflow_before = int(dut.hdmi_underflow_count.value)
    axis_error_before = int(dut.hdmi_axis_error_count.value)

    # enable playback + HDMI (SV: @(negedge aclk); fb_playback_en=1; hdmi_enable=1)
    await RisingEdge(aclk)
    playback.enable = True
    dut.hdmi_enable.value = 1

    hdmi_seen = 0
    errors = 0
    for _ in range(20000):
        await RisingEdge(aclk)
        await ReadOnly()
        if int(dut.video_de.value):
            check(hdmi_seen < FRAME_PIXELS,
                  f"HDMI emitted more active pixels than expected (saw {hdmi_seen})")
            got_r = int(dut.video_r.value)
            if got_r != EXPECTED_Y[hdmi_seen]:
                dut._log.error(
                    "[FAIL] HDMI pixel %d: got R=0x%02x G=0x%02x B=0x%02x, expected 0x%02x",
                    hdmi_seen, got_r, int(dut.video_g.value), int(dut.video_b.value),
                    EXPECTED_Y[hdmi_seen])
                errors += 1
            hdmi_seen += 1
            if hdmi_seen == FRAME_PIXELS:
                break

    check(hdmi_seen == FRAME_PIXELS,
          f"HDMI delivered all {FRAME_PIXELS} expected pixels (saw {hdmi_seen})")
    check(errors == 0, f"HDMI E2E: {errors} pixel mismatches")
    dut._log.info(
        "[PASS] HDMI E2E: all %d pixels match (lines 0&3 masked to 0x80; 1&2 pass-through)",
        FRAME_PIXELS)

    for _ in range(8):
        await RisingEdge(aclk)
    check(int(dut.hdmi_underflow_count.value) == underflow_before,
          "HDMI active window had no underflow")
    check(int(dut.hdmi_axis_error_count.value) == axis_error_before,
          "HDMI active window had no AXIS sideband error")


def test_ob_masker_e2e():
    from runner_support import build_and_test

    here = Path(__file__).resolve().parent
    harness = here / "ob_masker_e2e_harness.sv"

    build_and_test(
        block="ob_masker_e2e",
        sources=[
            "rtl/mipi_rx/csi2_packet_parser.sv",
            "rtl/mipi_rx/csi2_header_ecc.sv",
            "rtl/mipi_rx/csi2_payload_crc.sv",
            "rtl/mipi_rx/csi2_vcdt_filter.sv",
            "rtl/mipi_rx/csi2_frame_state.sv",
            "rtl/img_proc/yuv422_gray_unpack.sv",
            "rtl/img_proc/ob_row_masker.sv",
            "rtl/mipi_rx/axis_video_bridge.sv",
            "rtl/hdmi/hdmi_output.sv",
            str(harness),
        ],
        toplevel="ob_masker_e2e_harness",
        test_module="test_ob_masker_e2e",
        test_dir=here,
        parameters={
            "PARSER_IN_WIDTH": PARSER_IN_WIDTH,
            "LINE_PIXELS": LINE_PIXELS,
            "FRAME_LINES": FRAME_LINES,
            "LINE_BYTES": LINE_BYTES,
        },
        engine="verilator",
    )
