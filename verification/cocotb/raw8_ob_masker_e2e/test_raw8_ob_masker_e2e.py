"""cocotb port of verification/tb/tb_raw8_ob_masker_e2e.sv.

E2E: a synthetic RAW8 CSI-2 byte stream (FS short packet, 4 RAW8 long-packet lines, FE
short packet) is driven into the real pixel path
    csi2_packet_parser -> csi2_header_ecc -> csi2_payload_crc -> csi2_vcdt_filter ->
    csi2_frame_state -> raw8_passthrough -> ob_row_masker (WIDTH=8)
and the ob_row_masker output is captured pixel-by-pixel and verified against the DSim
TB's golden ``expected_y`` array.

The DSim TB wires seven RTL instances together inside the testbench module; there is no
RTL top. Verilator needs a single synthesizable toplevel, so ``raw8_ob_masker_e2e_top``
(in the local ``raw8_ob_masker_e2e_stubs.sv``) reproduces the TB's exact instantiation
(identical params/port bindings) and exposes the byte-beat inputs, the ob_* outputs, and
the status counters the TB asserts on. All pixel-path logic is the real rtl/ modules.

The DSim ``initial`` stimulus (reset -> FS -> 4 lines -> FE -> waits -> checks) becomes the
single ``raw8_ob_masker_e2e`` coroutine; the SV ``always_ff`` that pushes ``ob_pixel`` into
``ob_capture`` on ``ob_valid`` becomes the ``ob_monitor`` coroutine; every ``$fatal`` /
``[FAIL]`` becomes a ``check()``. ECC (Hamming) and CRC-16 are computed with the same
algorithms as the TB's ``make_ecc`` / ``crc_update_byte`` helpers.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import FallingEdge, RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import start_clock  # noqa: E402
from lib.scoreboard import check  # noqa: E402

# --- TB localparams -----------------------------------------------------------------
PARSER_IN_WIDTH = 16
DT_FS = 0x00
DT_FE = 0x01
DT_RAW8 = 0x2A

LINE_PIXELS = 8
FRAME_LINES = 4
LINE_BYTES = LINE_PIXELS  # RAW8: 1 byte per pixel
FRAME_PIXELS = LINE_PIXELS * FRAME_LINES

# TB input_y[FRAME_PIXELS]
INPUT_Y = [
    # L0: OB uniform Y=36
    36, 36, 36, 36, 36, 36, 36, 36,
    # L1: checkerboard 10/240
    10, 10, 10, 10, 240, 240, 240, 240,
    # L2: gradient
    0, 32, 64, 96, 128, 160, 192, 224,
    # L3: Bayer-like
    80, 180, 180, 80, 80, 180, 180, 80,
]

# TB expected_y[FRAME_PIXELS]
EXPECTED_Y = [
    # L0: masked -> 128
    128, 128, 128, 128, 128, 128, 128, 128,
    # L1: pass-through
    10, 10, 10, 10, 240, 240, 240, 240,
    # L2: pass-through
    0, 32, 64, 96, 128, 160, 192, 224,
    # L3: Bayer-like -> pass-through (range=100 > 3)
    80, 180, 180, 80, 80, 180, 180, 80,
]


# --- Golden reference algorithms (mirror the TB's SV helpers exactly) ----------------
def calc_ecc6(data: int) -> int:
    """Mirror tb calc_ecc6 (24-bit Hamming parity over {wc, di})."""
    def b(i: int) -> int:
        return (data >> i) & 1

    e = [0] * 6
    e[0] = b(0)^b(1)^b(2)^b(4)^b(5)^b(7)^b(10)^b(11)^b(13)^b(16)^b(20)^b(21)^b(22)^b(23)
    e[1] = b(0)^b(1)^b(3)^b(4)^b(6)^b(8)^b(10)^b(12)^b(14)^b(17)^b(20)^b(21)^b(22)^b(23)
    e[2] = b(0)^b(2)^b(3)^b(5)^b(6)^b(9)^b(11)^b(12)^b(15)^b(18)^b(20)^b(21)^b(22)
    e[3] = b(1)^b(2)^b(3)^b(7)^b(8)^b(9)^b(13)^b(14)^b(15)^b(19)^b(20)^b(21)^b(23)
    e[4] = b(4)^b(5)^b(6)^b(7)^b(8)^b(9)^b(16)^b(17)^b(18)^b(19)^b(20)^b(22)^b(23)
    e[5] = b(10)^b(11)^b(12)^b(13)^b(14)^b(15)^b(16)^b(17)^b(18)^b(19)^b(21)^b(22)^b(23)
    return sum(bit << i for i, bit in enumerate(e))


def make_ecc(di: int, wc: int) -> int:
    """Mirror tb make_ecc: {2'b00, calc_ecc6({wc, di})}."""
    return calc_ecc6((di & 0xFF) | ((wc & 0xFFFF) << 8)) & 0x3F


def crc_update_byte(crc_in: int, data: int) -> int:
    """Mirror tb crc_update_byte (reflected CRC-16, poly 0x8408)."""
    c = crc_in & 0xFFFF
    for bit in range(8):
        fb = (c & 1) ^ ((data >> bit) & 1)
        c >>= 1
        if fb:
            c ^= 0x8408
    return c & 0xFFFF


# --- ob output monitor (mirrors the SV always_ff ob_capture logger) ------------------
class ObCapture:
    def __init__(self, dut):
        self.dut = dut
        self.pixels: list[int] = []

    def start(self, clk):
        return cocotb.start_soon(self._run(clk))

    async def _run(self, clk):
        d = self.dut
        while True:
            await RisingEdge(clk)
            if int(d.ob_valid.value) == 1:
                self.pixels.append(int(d.ob_pixel.value))


# --- byte-beat driver (mirrors the SV drive_beat/drive_idle, driven on negedge) ------
class BeatDriver:
    """Faithful port of the TB's drive_beat/drive_idle: valid is asserted on negedge and
    held continuously across back-to-back drive_beat() calls; drive_idle() deasserts."""

    def __init__(self, dut):
        self.dut = dut
        self.clk = dut.core_clk

    async def idle(self, n: int):
        await FallingEdge(self.clk)
        self.dut.s_byte_valid.value = 0
        self.dut.s_byte_keep.value = 0
        self.dut.s_byte_sop.value = 0
        self.dut.s_byte_eop.value = 0
        self.dut.s_byte_data.value = 0
        for _ in range(n):
            await FallingEdge(self.clk)

    async def beat(self, b0: int, b1: int, sop: int, eop: int):
        await FallingEdge(self.clk)
        self.dut.s_byte_data.value = ((b1 & 0xFF) << 8) | (b0 & 0xFF)
        self.dut.s_byte_keep.value = 0b11
        self.dut.s_byte_valid.value = 1
        self.dut.s_byte_sop.value = int(sop)
        self.dut.s_byte_eop.value = int(eop)


async def send_short_packet(drv: BeatDriver, dt: int, data_field: int):
    di = dt & 0x3F  # {2'b00, dt}
    ecc = make_ecc(di, data_field)
    await drv.beat(di, data_field & 0xFF, 1, 0)
    await drv.beat((data_field >> 8) & 0xFF, ecc, 0, 1)
    await drv.idle(2)


async def send_raw8_line(drv: BeatDriver, line_idx: int):
    di = DT_RAW8 & 0x3F
    wc = LINE_BYTES & 0xFFFF
    ecc = make_ecc(di, wc)
    base = line_idx * LINE_PIXELS
    payload = [INPUT_Y[base + p] for p in range(LINE_PIXELS)]

    crc = 0xFFFF
    for byte in payload:
        crc = crc_update_byte(crc, byte)

    await drv.beat(di, wc & 0xFF, 1, 0)
    await drv.beat((wc >> 8) & 0xFF, ecc, 0, 0)
    for i in range(0, LINE_BYTES, 2):
        b1 = payload[i + 1] if (i + 1) < LINE_BYTES else 0x00
        await drv.beat(payload[i], b1, 0, 0)
    await drv.beat(crc & 0xFF, (crc >> 8) & 0xFF, 0, 1)
    await drv.idle(8)


async def wait_frame_done(clk, dut):
    for _ in range(8000):
        await RisingEdge(clk)
        if int(dut.frame_count.value) == 1:
            return
    raise AssertionError("CHECK FAILED: frame timeout")


async def wait_parser_short(clk, dut, n: int):
    for _ in range(8000):
        await RisingEdge(clk)
        if int(dut.parser_short_count.value) >= n:
            return
    raise AssertionError("CHECK FAILED: short pkt timeout")


@cocotb.test(timeout_time=20, timeout_unit="ms")
async def raw8_ob_masker_e2e(dut):
    clk = dut.core_clk
    start_clock(clk, period_ns=10.0)  # TB: #5 -> 100 MHz

    cap = ObCapture(dut)
    cap.start(clk)
    drv = BeatDriver(dut)

    # --- reset_dut() ---
    dut.core_aresetn.value = 0
    dut.s_byte_data.value = 0
    dut.s_byte_keep.value = 0
    dut.s_byte_valid.value = 0
    dut.s_byte_sop.value = 0
    dut.s_byte_eop.value = 0
    for _ in range(8):
        await RisingEdge(clk)
    dut.core_aresetn.value = 1
    for _ in range(4):
        await RisingEdge(clk)

    # --- stimulus: FS, 4 RAW8 lines, FE ---
    await send_short_packet(drv, DT_FS, 0x0000)
    for i in range(FRAME_LINES):
        await send_raw8_line(drv, i)
    await send_short_packet(drv, DT_FE, 0x0000)

    await wait_frame_done(clk, dut)
    await wait_parser_short(clk, dut, 2)

    # --- TB status-counter checks ---
    check(int(dut.parser_short_count.value) >= 2, "no FE (short pkt count < 2)")
    check(int(dut.parser_long_count.value) == FRAME_LINES,
          f"long count (got {int(dut.parser_long_count.value)}, expected {FRAME_LINES})")
    check(int(dut.crc_err_count.value) == 0,
          f"crc err (got {int(dut.crc_err_count.value)}, expected 0)")
    check(int(dut.last_frame_lines.value) == FRAME_LINES,
          f"last_frame_lines (got {int(dut.last_frame_lines.value)}, expected {FRAME_LINES})")

    # drain the ob pipeline (TB: repeat(200))
    for _ in range(200):
        await RisingEdge(clk)

    # --- pixel-by-pixel verification vs expected_y ---
    check(len(cap.pixels) == FRAME_PIXELS,
          f"captured {len(cap.pixels)}, expected {FRAME_PIXELS}")
    for i in range(FRAME_PIXELS):
        check(cap.pixels[i] == EXPECTED_Y[i],
              f"pix[{i}] got=0x{cap.pixels[i]:02x} expected=0x{EXPECTED_Y[i]:02x}")


def test_raw8_ob_masker_e2e():
    from runner_support import build_and_test

    build_and_test(
        block="raw8_ob_masker_e2e",
        sources=[
            "verification/cocotb/raw8_ob_masker_e2e/raw8_ob_masker_e2e_stubs.sv",
            "rtl/mipi_rx/csi2_packet_parser.sv",
            "rtl/mipi_rx/csi2_header_ecc.sv",
            "rtl/mipi_rx/csi2_payload_crc.sv",
            "rtl/mipi_rx/csi2_vcdt_filter.sv",
            "rtl/mipi_rx/csi2_frame_state.sv",
            "rtl/img_proc/raw8_passthrough.sv",
            "rtl/img_proc/ob_row_masker.sv",
        ],
        toplevel="raw8_ob_masker_e2e_top",
        test_module="test_raw8_ob_masker_e2e",
        test_dir=Path(__file__).resolve().parent,
        parameters={
            "PARSER_IN_WIDTH": PARSER_IN_WIDTH,
            "LINE_PIXELS": LINE_PIXELS,
            "FRAME_LINES": FRAME_LINES,
            "LINE_BYTES": LINE_BYTES,
        },
        engine="verilator",
    )
