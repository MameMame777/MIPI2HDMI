"""cocotb port of verification/tb/tb_dphy_hs_byte_probe_gearbox.sv.

This DSim TB is unusual: it does NOT deserialize through the Xilinx ISERDESE2 cells.
Instead it drives DUT-INTERNAL registers hierarchically:

  * ``dut.serdes_byte_sample[lane]`` -- the post-ISERDES byte sample the stream path
    consumes each byte_clk (the ISERDES stubs output 0; the TB overrides this reg).
  * ``dut.trace_slot_lane{0,1}_candidate[slot]`` + ``dut.trace_capture_active/done`` +
    ``dut.trace_slot_valid`` -- pre-loaded so the sync-header scan FSM runs on a fixed
    8-slot candidate window (the "gearbox" sync-scan path).

To reproduce this faithfully the DUT (``dphy_hs_byte_probe``) is the cocotb toplevel and
is built with Verilator ``--public-flat-rw`` so cocotb can write those internal signals.
The Xilinx primitives are replaced by the connectivity-only stubs in
``dphy_hs_byte_probe_gearbox_stubs.sv`` (the TB never relies on the ISERDES gearbox output).

BUFR stub is a pass-through (no /4 divide), exactly as the DSim inline stub, so
``byte_clk`` follows ``dphy_hs_clock_clk_p`` (10 ns). During reset BUFR.CLR forces
byte_clk low, so we clock the ``hs_clk`` and derive byte_clk edges from ``dut.byte_clk``.

Packed-array bit layout (Verilator flattens ``[MSB:0]`` packed dims MSB-first):
  serdes_byte_sample [1:0][7:0]  -> {lane1[7:0], lane0[7:0]}  (16 bits)
  trace_slot_lane*_candidate [7:0][7:0] -> {slot7..slot0}     (64 bits, slot k @ 8k)
  trace_slot_valid [7:0]

Each DSim task becomes one @cocotb.test (fresh reset), mirroring the single cumulative
``initial`` run's per-task reset_dut(). Every check_condition -> check().
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, Timer

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.scoreboard import check  # noqa: E402

HS_PERIOD_NS = 10.0  # tb: #5 half period -> 10 ns

# ---------------------------------------------------------------------------
# reference helpers mirrored from the TB
# ---------------------------------------------------------------------------


def ref_ecc6(data: int) -> int:
    """SV ref_ecc6: 6-bit ECC over a 24-bit word (identical bit taps to the TB)."""
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


def make_stream(bit_offset: int, byte0: int, byte1: int, byte2: int, byte3: int) -> int:
    """SV make_stream: 64-bit lane stream = 0xB8 SoT then 4 bytes, shifted by bit_offset."""
    s = 0
    s |= (0xB8) << bit_offset
    s |= (byte0 & 0xFF) << (bit_offset + 8)
    s |= (byte1 & 0xFF) << (bit_offset + 16)
    s |= (byte2 & 0xFF) << (bit_offset + 24)
    s |= (byte3 & 0xFF) << (bit_offset + 32)
    return s & 0xFFFF_FFFF_FFFF_FFFF


def ecc_yuv422(wc: int) -> int:
    """ecc for DT=0x1e, given WC (the TB uses {2'b00, ref_ecc6({wc, 8'h1e})})."""
    return ref_ecc6(((wc & 0xFFFF) << 8) | 0x1E) & 0x3F


# ---------------------------------------------------------------------------
# low-level DUT internal-signal access (packed array pack/unpack)
# ---------------------------------------------------------------------------


def set_serdes_sample(dut, lane0: int, lane1: int) -> None:
    dut.serdes_byte_sample.value = ((lane1 & 0xFF) << 8) | (lane0 & 0xFF)


def arm_sync_scan(dut, lane0_stream: int, lane1_stream: int) -> None:
    """Drive the sync-header scan from two 64-bit lane candidate streams.

    ENGINE NOTE: the DSim TB force-writes the DUT *output ports*
    trace_slot_lane{0,1}_candidate[slot] and trace_slot_valid, then lets the arm branch
    (trace_capture_done && !sync_scan_active) latch them into sync_scan_lane*_stream.
    Verilator's --public-flat-rw exposes INTERNAL regs writable but NOT output ports
    (verified: a write to trace_slot_lane0_candidate reads back 0), and the installed
    Icarus cannot elaborate this DUT ("overriding the default variable lifetime is not
    yet supported" on the block-local `automatic`s), so the candidate ports cannot be
    forced on either engine.

    Instead we inject the SCAN INPUT directly: sync_scan_lane{0,1}_stream and the whole
    sync_scan_* state are internal regs (verified writable), and the arm branch's ONLY
    effect is to (a) reset the scan-best/index regs and (b) latch
    sync_scan_lane*_stream = {candidate[7]..candidate[0]} == lane*_stream. We reproduce
    that state exactly and set sync_scan_active=1, so the DUT's scan-iteration branch runs
    bit-for-bit on the same 64-bit streams. Same FSM, same computation, same outputs; no
    check is weakened. (SYNC_HEADER_USE_ALIGNED_STREAM=0 here, so candidate == the stream.)
    """
    dut.trace_capture_active.value = 0
    dut.trace_capture_done.value = 1
    dut.sync_scan_lane0_stream.value = lane0_stream & 0xFFFF_FFFF_FFFF_FFFF
    dut.sync_scan_lane1_stream.value = lane1_stream & 0xFFFF_FFFF_FFFF_FFFF
    dut.sync_scan_bit_offset_lane0.value = 0
    dut.sync_scan_bit_offset_lane1.value = 0
    dut.sync_scan_pairing.value = 0
    dut.sync_scan_best_score.value = 0
    dut.sync_scan_best_di.value = 0
    dut.sync_scan_best_wc.value = 0
    dut.sync_scan_best_ecc.value = 0
    dut.sync_scan_best_syndrome.value = 0
    dut.sync_scan_best_pairing.value = 0
    dut.sync_scan_best_bit_offset_lane0.value = 0
    dut.sync_scan_best_bit_offset_lane1.value = 0
    dut.sync_scan_best_no_error.value = 0
    dut.sync_scan_best_corrected.value = 0
    dut.sync_scan_best_uncorrectable.value = 0
    dut.sync_scan_active.value = 1


# ---------------------------------------------------------------------------
# clocking + reset
# ---------------------------------------------------------------------------


async def start_hs_clock(dut):
    # BUFR stub passes hs_clk through as byte_clk (no /4), matching the DSim inline stub.
    cocotb.start_soon(Clock(dut.dphy_hs_clock_clk_p, HS_PERIOD_NS, unit="ns").start())
    # idelay_ref_clk: #2.5 half -> 5 ns period (only affects idelayctrl_rdy; not load-bearing)
    cocotb.start_soon(Clock(dut.idelay_ref_clk, HS_PERIOD_NS / 2.0, unit="ns").start())


def _init_ports(dut):
    dut.rst_n.value = 0
    dut.idelay_ref_reset.value = 1
    dut.runtime_idelay_tap.value = 0
    dut.runtime_idelay_tap_lane1.value = 0
    dut.runtime_bitslip_phase.value = 0
    dut.runtime_bitslip_phase_lane1.value = 0
    dut.runtime_lane1_sweep_enable.value = 0
    dut.runtime_expected_long_dt.value = 0
    dut.sup_enable.value = 0
    dut.sup_bufr_clr.value = 0
    dut.sup_serdes_rst.value = 0
    dut.sup_hs_settled.value = 0
    dut.dphy_data_hs_p.value = 0
    dut.dphy_data_lp_p.value = 0b11
    dut.dphy_data_lp_n.value = 0b11


def _release_trace_forces(dut):
    # SV release_trace_forces()
    dut.trace_capture_done.value = 0
    dut.trace_capture_active.value = 0
    dut.trace_slot_valid.value = 0
    dut.trace_slot_lane0_candidate.value = 0
    dut.trace_slot_lane1_candidate.value = 0


async def reset_dut(dut):
    """Mirror the SV reset_dut task."""
    _release_trace_forces(dut)
    dut.rst_n.value = 0
    dut.dphy_data_hs_p.value = 0
    dut.dphy_data_lp_p.value = 0b11
    dut.dphy_data_lp_n.value = 0b11
    # 8 hs_clk posedges while in reset (byte_clk is held low by BUFR.CLR)
    for _ in range(8):
        await RisingEdge(dut.dphy_hs_clock_clk_p)
    dut.rst_n.value = 1
    dut.idelay_ref_reset.value = 0
    # 12 byte_clk posedges after release
    for _ in range(12):
        await RisingEdge(dut.byte_clk)


async def drive_serdes_sample(dut, lane0: int, lane1: int):
    """SV drive_serdes_sample: set at negedge, consume at the following posedge."""
    await FallingEdge(dut.byte_clk)
    set_serdes_sample(dut, lane0, lane1)
    await RisingEdge(dut.byte_clk)
    await Timer(1, unit="ns")


async def drive_lp_state(dut, lane_lp_p: int, lane_lp_n: int, cycles: int):
    """SV drive_lp_state."""
    await FallingEdge(dut.byte_clk)
    dut.dphy_data_lp_p.value = lane_lp_p
    dut.dphy_data_lp_n.value = lane_lp_n
    for _ in range(cycles):
        await RisingEdge(dut.byte_clk)
    await Timer(1, unit="ns")


async def wait_for_stream_word(dut, name, expected_data, expected_sop, filler0, filler1):
    """SV wait_for_stream_word: drive filler samples until a stream word appears; check it."""
    for _ in range(800):
        await drive_serdes_sample(dut, filler0, filler1)
        if int(dut.stream_byte_valid.value) == 1:
            check(int(dut.stream_byte_sop.value) == expected_sop, f"{name}: SOP")
            check(int(dut.stream_byte_keep.value) == 0b11, f"{name}: keep")
            check(int(dut.stream_byte_data.value) == (expected_data & 0xFFFF), f"{name}: data")
            return
    raise AssertionError(
        f"CHECK FAILED: {name}: timed out waiting for stream word {expected_data:04x} "
        f"(valid={int(dut.stream_byte_valid.value)} sop={int(dut.stream_byte_sop.value)} "
        f"data={int(dut.stream_byte_data.value):04x} sync_valid={int(dut.sync_header_valid.value)} "
        f"score={int(dut.sync_header_score.value)} scan_active={int(dut.sync_scan_active.value)} "
        f"buf_active={int(dut.stream_buffer_active.value)} buf_count={int(dut.stream_buffer_count.value)})"
    )


async def drive_streams(dut, lane0_stream: int, lane1_stream: int):
    """SV drive_streams: arm the sync scan on the candidate window, spin until it resolves.

    The DSim task pre-loads the 8-slot candidate output ports then lets the arm branch
    latch them; we inject the equivalent scan state directly (see arm_sync_scan). The
    exit condition mirrors the SV loop: done when the scan clears trace_capture_done and
    sync_scan_active (last_candidate) -- i.e. sync_header_valid has been resolved.
    """
    arm_sync_scan(dut, lane0_stream, lane1_stream)
    for _ in range(600):
        await RisingEdge(dut.byte_clk)
        if int(dut.sync_header_valid.value) == 1 or (
            int(dut.trace_capture_done.value) == 0 and int(dut.sync_scan_active.value) == 0
        ):
            return
    raise AssertionError("CHECK FAILED: Timed out waiting for sync-header scan")


# ---------------------------------------------------------------------------
# scenario tasks (1:1 with the SV tasks)
# ---------------------------------------------------------------------------


async def run_valid_case(dut, name, lane0_stream, lane1_stream, expected_di, expected_wc,
                         expected_pairing, expected_bit_offset_lane0, expected_bit_offset_lane1):
    await reset_dut(dut)
    await drive_streams(dut, lane0_stream, lane1_stream)
    check(int(dut.sync_header_valid.value) == 1, f"{name}: valid")
    check(int(dut.sync_header_di.value) == expected_di, f"{name}: DI")
    check(int(dut.sync_header_wc.value) == expected_wc, f"{name}: WC")
    check(int(dut.sync_header_pairing.value) == expected_pairing, f"{name}: pairing")
    check(int(dut.sync_header_bit_offset_lane0.value) == expected_bit_offset_lane0,
          f"{name}: lane0 bit offset")
    check(int(dut.sync_header_bit_offset_lane1.value) == expected_bit_offset_lane1,
          f"{name}: lane1 bit offset")
    check(int(dut.sync_header_ecc_no_error.value) == 1, f"{name}: ECC no-error")


async def run_invalid_case(dut, name, lane0_stream, lane1_stream):
    await reset_dut(dut)
    await drive_streams(dut, lane0_stream, lane1_stream)
    check(int(dut.sync_header_valid.value) == 0, f"{name}: invalid")
    check(int(dut.sync_header_score.value) == 0, f"{name}: zero score")


async def run_nonqualifying_case(dut, name, lane0_stream, lane1_stream):
    await reset_dut(dut)
    await drive_streams(dut, lane0_stream, lane1_stream)
    check(int(dut.sync_header_valid.value) == 0, f"{name}: not valid")
    check(int(dut.sync_header_score.value) != 0, f"{name}: diagnostic score retained")
    check(int(dut.sync_header_score.value) < 13, f"{name}: below valid threshold")


async def run_stream_pair0_case(dut, ecc1280):
    await reset_dut(dut)
    await drive_lp_state(dut, 0b00, 0b00, 4)
    await drive_serdes_sample(dut, 0xB8, 0xB8)
    check(int(dut.stream_byte_valid.value) == 0, "pair0 stream does not emit SoT")
    await drive_serdes_sample(dut, 0x1E, 0x00)
    check(int(dut.stream_byte_valid.value) == 0, "pair0 first post-SoT beat is buffered")
    await drive_serdes_sample(dut, 0x05, ecc1280)
    check(int(dut.stream_byte_valid.value) == 0, "pair0 second post-SoT beat is buffered")
    await wait_for_stream_word(dut, "pair0 first beat", 0x001E, 1, 0x11, 0x22)
    await wait_for_stream_word(dut, "pair0 second beat", (ecc1280 << 8) | 0x05, 0, 0x33, 0x44)


async def run_payload_sot_like_bytes_case(dut, ecc1280):
    await reset_dut(dut)
    await drive_lp_state(dut, 0b00, 0b00, 4)
    await drive_serdes_sample(dut, 0xB8, 0xB8)
    await drive_serdes_sample(dut, 0x1E, 0x00)
    await drive_serdes_sample(dut, 0x05, ecc1280)
    await drive_serdes_sample(dut, 0xB8, 0xB8)
    await wait_for_stream_word(dut, "payload sot-like setup first beat", 0x001E, 1, 0x12, 0x34)
    await wait_for_stream_word(dut, "payload sot-like setup second beat", (ecc1280 << 8) | 0x05, 0, 0x56, 0x78)
    await wait_for_stream_word(dut, "payload sot-like byte", 0xB8B8, 0, 0x9A, 0xBC)

    await drive_serdes_sample(dut, 0x12, 0x34)
    await drive_lp_state(dut, 0b11, 0b11, 4)
    await drive_lp_state(dut, 0b00, 0b00, 4)
    await drive_serdes_sample(dut, 0xB8, 0xB8)
    check(int(dut.stream_byte_valid.value) == 0, "windowed real SoT suppresses sync byte")
    await drive_serdes_sample(dut, 0x1E, 0x00)
    await drive_serdes_sample(dut, 0x05, ecc1280)
    await wait_for_stream_word(dut, "windowed real SoT retriggers", 0x001E, 1, 0xDE, 0xF0)


async def run_payload_rotated_sot_pattern_case(dut, ecc1280):
    await reset_dut(dut)
    await drive_lp_state(dut, 0b00, 0b00, 4)
    await drive_serdes_sample(dut, 0xB8, 0xB8)
    await drive_serdes_sample(dut, 0x1E, 0x00)
    await drive_serdes_sample(dut, 0x05, ecc1280)
    await drive_serdes_sample(dut, 0x71, 0xC5)
    await wait_for_stream_word(dut, "rotated sot pattern setup first beat", 0x001E, 1, 0x2E, 0x17)
    await wait_for_stream_word(dut, "rotated sot pattern setup second beat", (ecc1280 << 8) | 0x05, 0, 0x8B, 0xE2)
    await wait_for_stream_word(dut, "rotated sot pattern byte", 0xC571, 0, 0x45, 0x67)
    await drive_serdes_sample(dut, 0x2E, 0x17)
    await wait_for_stream_word(dut, "second rotated sot pattern beat", 0x172E, 0, 0x89, 0xAB)
    await drive_serdes_sample(dut, 0x8B, 0xE2)
    await wait_for_stream_word(dut, "third rotated sot pattern beat", 0x172E, 0, 0xCD, 0xEF)


# ---------------------------------------------------------------------------
# @cocotb.test entry points (one per DSim scenario; matches the cumulative initial run)
# ---------------------------------------------------------------------------


async def _prologue(dut):
    _init_ports(dut)
    await start_hs_clock(dut)


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def valid_pair0_no_offset(dut):
    await _prologue(dut)
    ecc = ecc_yuv422(1280)
    await run_valid_case(
        dut, "pair0_no_offset",
        make_stream(0, 0x1E, 0x05, 0x00, 0x00),
        make_stream(0, 0x00, ecc, 0x00, 0x00),
        0x1E, 1280, 0, 0, 0,
    )


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def valid_pair2_lane1_delayed(dut):
    await _prologue(dut)
    ecc = ecc_yuv422(1280)
    await run_valid_case(
        dut, "pair2_lane1_delayed",
        make_stream(0, 0x1E, 0x05, 0x00, 0x00),
        make_stream(0, 0xA5, 0x00, ecc, 0x00),
        0x1E, 1280, 2, 0, 0,
    )


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def valid_pair0_with_bit_offsets(dut):
    await _prologue(dut)
    ecc = ecc_yuv422(1280)
    await run_valid_case(
        dut, "pair0_with_bit_offsets",
        make_stream(3, 0x1E, 0x05, 0x00, 0x00),
        make_stream(5, 0x00, ecc, 0x00, 0x00),
        0x1E, 1280, 0, 3, 5,
    )


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def nonqualifying_non_exact_wc_rejected(dut):
    await _prologue(dut)
    ecc1567 = ecc_yuv422(1567)
    await run_nonqualifying_case(
        dut, "non_exact_wc_rejected",
        make_stream(0, 0x1E, 0x06, 0x00, 0x00),
        make_stream(0, 0x1F, ecc1567, 0x00, 0x00),
    )


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def invalid_lane0_only_sot_rejected(dut):
    await _prologue(dut)
    await run_invalid_case(
        dut, "lane0_only_sot_rejected",
        make_stream(0, 0x1E, 0x05, 0x00, 0x00),
        0xFEDC_BA98_7654_3210,
    )


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def invalid_lane1_only_sot_rejected(dut):
    await _prologue(dut)
    ecc = ecc_yuv422(1280)
    await run_invalid_case(
        dut, "lane1_only_sot_rejected",
        0x0123_4567_89AB_CDEF,
        make_stream(0, 0x00, ecc, 0x00, 0x00),
    )


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def invalid_no_sot(dut):
    await _prologue(dut)
    await run_invalid_case(dut, "no_sot", 0x0123_4567_89AB_CDEF, 0xFEDC_BA98_7654_3210)


@cocotb.test(timeout_time=10, timeout_unit="ms")
async def stream_pair0(dut):
    await _prologue(dut)
    await run_stream_pair0_case(dut, ecc_yuv422(1280))


@cocotb.test(timeout_time=10, timeout_unit="ms")
async def payload_sot_like_bytes(dut):
    await _prologue(dut)
    await run_payload_sot_like_bytes_case(dut, ecc_yuv422(1280))


@cocotb.test(timeout_time=10, timeout_unit="ms")
async def payload_rotated_sot_pattern(dut):
    await _prologue(dut)
    await run_payload_rotated_sot_pattern_case(dut, ecc_yuv422(1280))


# ---------------------------------------------------------------------------
# pytest entry point
# ---------------------------------------------------------------------------


def test_dphy_hs_byte_probe_gearbox():
    import cocotb_site as cs
    from runner_support import prepare_verilator_toolchain
    from cocotb_tools.runner import get_runner

    prepare_verilator_toolchain()
    # --public-flat-rw: the DSim TB drives DUT-internal regs (serdes_byte_sample, trace_*)
    # hierarchically; this exposes them writable to cocotb.
    build_args = cs.common_build_args() + ["--public-flat-rw"]

    here = Path(__file__).resolve().parent
    sources = [
        here / "dphy_hs_byte_probe_gearbox_stubs.sv",   # unisim stubs FIRST
        cs.REPO_ROOT / "rtl/prototype/dphy_hs_byte_probe.sv",
    ]
    parameters = {
        "LANES": 2,
        "SOT_WINDOW_BYTES": 8,
        "SWEEP_HOLD_BYTES": 4,
        "SWEEP_ENABLE": 0,
        "FIXED_BITSLIP_PHASE": 0,
        "FIXED_BITSLIP_PHASE_LANE1": 0,
        "LANE1_BITSLIP_SWEEP_ENABLE": 0,
        "FIXED_TRANSFORM": 0,
        "TRACE_TRIGGER_MODE": 3,
        "IDELAY_TAP": 0,
        "STREAM_PAIRING": 0,
    }
    build_dir = cs.BUILD_DIR / "sim" / "dphy_hs_byte_probe_gearbox"
    runner = get_runner("verilator")
    runner.build(
        sources=sources,
        hdl_toplevel="dphy_hs_byte_probe",
        parameters=parameters,
        build_args=build_args,
        build_dir=build_dir,
        always=True,
        timescale=("1ns", "1ps"),
    )
    runner.test(
        hdl_toplevel="dphy_hs_byte_probe",
        test_module="test_dphy_hs_byte_probe_gearbox",
        test_dir=here,
        build_dir=build_dir,
        timescale=("1ns", "1ps"),
    )
