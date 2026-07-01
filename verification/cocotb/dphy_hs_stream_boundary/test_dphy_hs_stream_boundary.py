"""cocotb port of verification/tb/tb_dphy_hs_stream_boundary.sv.

This is the D-PHY HS byte-probe stream/scanner boundary test: the sync-header scanner
qualifies a header candidate (pairing sweep + ECC), the stream path deskews and pairs
lane0/lane1 bytes into the 16-bit ``stream_byte_*`` bus, and a downstream
``csi2_packet_parser`` + ``csi2_header_ecc`` re-decode it. The DSim TB drives the probe by
force-depositing the post-ISERDES capture register ``serdes_byte_sample`` one byte-pair per
byte_clk (bypassing the ISERDESE2 gearbox, which the Verilator stub does not model), then
checks the scanner outputs, the paired stream SOP words, and the parser/ECC results.

Faithful 1:1 port:
  * The three DUT instances (probe/parser/ecc) and their wiring live in
    ``dphy_hs_stream_boundary_stubs.sv`` (harness ``dphy_hs_stream_boundary_harness``),
    which also carries the behavioral Xilinx primitive stubs. byte_clk is the BUFR-stub
    pass-through of hs_clk (10 ns), exactly as in DSim.
  * The TB ``always_ff @(posedge byte_clk)`` logger becomes the ``Capture`` monitor
    coroutine (same state machine: captured_stream_*, stream_sop_word0/1 pending FSM,
    parser/ecc header logs, payload byte count).
  * ``drive_serdes_sample`` deposits ``serdes_byte_sample`` (packed [1:0][7:0]) via
    ``dut.u_dut.serdes_byte_sample`` at ``FallingEdge(byte_clk)`` -- the DSim negedge
    deposit that the probe's posedge combinational path reads before its own
    ``serdes_byte_sample <= serdes_byte(=0)`` NBA overwrites it.
  * ``dut.stream_pairing_active/next`` reads use the harness ``*_dbg`` mirror ports.
  * Each ``run_*`` scenario is one ``@cocotb.test()`` (fresh reset), replicating every
    ``check_condition``; the TB ``#1ms`` watchdog maps to the per-test timeout.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge, Timer

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.scoreboard import check  # noqa: E402


# --------------------------------------------------------------------------------------
# CSI-2 header ECC reference (mirrors tb ref_ecc6 / make_ecc).
# --------------------------------------------------------------------------------------
def _ref_ecc6(data: int) -> int:
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
    # {2'b00, ref_ecc6({wc, di})}
    return _ref_ecc6(((wc & 0xFFFF) << 8) | (di & 0xFF)) & 0x3F


# --------------------------------------------------------------------------------------
# Small helpers.
# --------------------------------------------------------------------------------------
def _slot(packed_val, idx: int) -> int:
    """Extract byte ``idx`` from a packed [7:0][7:0] array value (slot i = bits i*8 +: 8)."""
    return (int(packed_val) >> (idx * 8)) & 0xFF


async def _start_clocks(dut):
    # idelay_ref_clk = #2.5 -> 5 ns period; hs_clk_p = #5 -> 10 ns period (byte_clk follows).
    cocotb.start_soon(Clock(dut.idelay_ref_clk, 5.0, unit="ns").start())
    cocotb.start_soon(Clock(dut.hs_clk_p, 10.0, unit="ns").start())


class Capture:
    """The TB always_ff @(posedge byte_clk) logger, held-in-reset by parser_aresetn.

    Reproduces the exact ordering and the stream_sop word0/word1 pending state machine.
    """

    def __init__(self, dut):
        self.dut = dut
        self._reset_state()

    def _reset_state(self):
        self.captured_stream_data = [0] * 8
        self.captured_stream_sop = [0] * 8
        self.captured_stream_count = 0
        self.stream_sop_word0_log = [0] * 16
        self.stream_sop_word1_log = [0] * 16
        self.stream_sop_seen_log = [0] * 16
        self.stream_packet_count = 0
        self.stream_pending_index = 0
        self.stream_second_pending = 0
        self.parser_last_hdr_seen = 0
        self.parser_last_pkt_di = 0
        self.parser_last_pkt_wc = 0
        self.parser_last_pkt_ecc_uncorrectable = 0
        self.parser_last_ecc_seen = 0
        self.parser_last_ecc_no_error = 0
        self.parser_last_ecc_corrected = 0
        self.parser_header_count = 0
        self.parser_payload_byte_count = 0
        self.parser_di_log = [0] * 16
        self.parser_wc_log = [0] * 16
        self.parser_ecc_uncorrectable_log = [0] * 16
        self.ecc_no_error_log = [0] * 16
        self.ecc_corrected_log = [0] * 16
        self.ecc_uncorrectable_log = [0] * 16
        self.ecc_header_count = 0

    def start(self):
        return cocotb.start_soon(self._run())

    async def _run(self):
        d = self.dut
        while True:
            await RisingEdge(d.byte_clk)
            # #1-equivalent: sample post-NBA settled values (matches the tb wait #1 reads).
            await Timer(1, unit="step")
            if int(d.parser_aresetn.value) == 0:
                self._reset_state()
                continue

            sbv = int(d.stream_byte_valid.value)
            sbs = int(d.stream_byte_sop.value)
            sbd = int(d.stream_byte_data.value)

            if sbv and self.captured_stream_count < 8:
                self.captured_stream_data[self.captured_stream_count] = sbd
                self.captured_stream_sop[self.captured_stream_count] = sbs
                self.captured_stream_count += 1

            if sbv and sbs and self.stream_packet_count < 16:
                self.stream_sop_word0_log[self.stream_packet_count] = sbd
                self.stream_sop_seen_log[self.stream_packet_count] = 1
                self.stream_pending_index = self.stream_packet_count
                self.stream_second_pending = 1
            elif sbv and self.stream_second_pending:
                self.stream_sop_word1_log[self.stream_pending_index] = sbd
                self.stream_packet_count = self.stream_pending_index + 1
                self.stream_second_pending = 0

            if int(d.parser_pkt_hdr_valid.value) == 1:
                self.parser_last_hdr_seen = 1
                self.parser_last_pkt_di = int(d.parser_pkt_di.value)
                self.parser_last_pkt_wc = int(d.parser_pkt_wc.value)
                self.parser_last_pkt_ecc_uncorrectable = int(d.parser_pkt_ecc_uncorrectable.value)
                if self.parser_header_count < 16:
                    self.parser_di_log[self.parser_header_count] = int(d.parser_pkt_di.value)
                    self.parser_wc_log[self.parser_header_count] = int(d.parser_pkt_wc.value)
                    self.parser_ecc_uncorrectable_log[self.parser_header_count] = int(d.parser_pkt_ecc_uncorrectable.value)
                self.parser_header_count += 1

            if int(d.ecc_hdr_corr_valid.value) == 1:
                self.parser_last_ecc_seen = 1
                self.parser_last_ecc_no_error = int(d.ecc_hdr_no_error.value)
                self.parser_last_ecc_corrected = int(d.ecc_hdr_corrected.value)
                if self.ecc_header_count < 16:
                    self.ecc_no_error_log[self.ecc_header_count] = int(d.ecc_hdr_no_error.value)
                    self.ecc_corrected_log[self.ecc_header_count] = int(d.ecc_hdr_corrected.value)
                    self.ecc_uncorrectable_log[self.ecc_header_count] = int(d.ecc_hdr_uncorrectable.value)
                self.ecc_header_count += 1

            if int(d.parser_payload_valid.value) == 1:
                self.parser_payload_byte_count += 1


# --------------------------------------------------------------------------------------
# Expected-stream helpers (tb expected_stream_sop_word0/1, live_trace_pair0_word0/1).
# --------------------------------------------------------------------------------------
def expected_stream_sop_word0(dut, pairing: int) -> int:
    l0 = int(dut.trace_slot_lane0_candidate.value)
    l1 = int(dut.trace_slot_lane1_candidate.value)

    def c0(i):
        return (l0 >> (i * 8)) & 0xFF

    def c1(i):
        return (l1 >> (i * 8)) & 0xFF

    if pairing == 0:
        return (c1(1) << 8) | c0(1)
    if pairing == 1:
        return (c0(1) << 8) | c1(1)
    if pairing == 2:
        return (c1(2) << 8) | c0(1)
    if pairing == 3:
        return (c1(1) << 8) | c0(2)
    if pairing == 4:
        return (c0(2) << 8) | c1(1)
    return (c0(1) << 8) | c1(2)


def expected_stream_sop_word1(dut, pairing: int) -> int:
    l0 = int(dut.trace_slot_lane0_candidate.value)
    l1 = int(dut.trace_slot_lane1_candidate.value)

    def c0(i):
        return (l0 >> (i * 8)) & 0xFF

    def c1(i):
        return (l1 >> (i * 8)) & 0xFF

    if pairing == 0:
        return (c1(2) << 8) | c0(2)
    if pairing == 1:
        return (c0(2) << 8) | c1(2)
    if pairing == 2:
        return (c1(3) << 8) | c0(2)
    if pairing == 3:
        return (c1(2) << 8) | c0(3)
    if pairing == 4:
        return (c0(3) << 8) | c1(2)
    return (c0(2) << 8) | c1(3)


def live_trace_pair0_word0(dut) -> int:
    a0 = int(dut.live_trace_slot_lane0_aligned.value)
    a1 = int(dut.live_trace_slot_lane1_aligned.value)
    return (((a1 >> 8) & 0xFF) << 8) | ((a0 >> 8) & 0xFF)


def live_trace_pair0_word1(dut) -> int:
    a0 = int(dut.live_trace_slot_lane0_aligned.value)
    a1 = int(dut.live_trace_slot_lane1_aligned.value)
    return (((a1 >> 16) & 0xFF) << 8) | ((a0 >> 16) & 0xFF)


# --------------------------------------------------------------------------------------
# Check tasks (tb check_live_trace_pair0 / check_scanner_stream_contract /
# check_clean_pair0_packet_contract).
# --------------------------------------------------------------------------------------
def check_live_trace_pair0(dut, cap, expected_word0, expected_word1, name):
    ltv = int(dut.live_trace_slot_valid.value)
    check((ltv & 0xF) == 0xF, f"{name}: live trace slots 0..3 captured")
    check((int(dut.live_trace_slot_sot_hit_lane0.value) >> 0) & 1, f"{name}: live trace lane0 SoT at slot0")
    check((int(dut.live_trace_slot_sot_hit_lane1.value) >> 0) & 1, f"{name}: live trace lane1 SoT at slot0")
    check(live_trace_pair0_word0(dut) == expected_word0, f"{name}: live trace pair0 word0")
    check(live_trace_pair0_word1(dut) == expected_word1, f"{name}: live trace pair0 word1")


def check_scanner_stream_contract(dut, cap, pkt_idx, expected_pairing, name):
    expected_word0 = expected_stream_sop_word0(dut, expected_pairing)
    expected_word1 = expected_stream_sop_word1(dut, expected_pairing)

    check(int(dut.sync_header_valid.value) == 1, f"{name}: scanner valid")
    check((int(dut.trace_slot_valid.value) & 0xF) == 0xF, f"{name}: trace slots 0..3 captured")
    check(int(dut.sync_header_pairing.value) == expected_pairing, f"{name}: scanner pairing")
    check(int(dut.sync_header_bit_offset_lane0.value) == 0, f"{name}: scanner lane0 bit offset zero")
    check(int(dut.sync_header_bit_offset_lane1.value) == 0, f"{name}: scanner lane1 bit offset zero")
    check(int(dut.sync_header_di.value) == (expected_word0 & 0xFF), f"{name}: scanner DI matches selected trace bytes")
    exp_wc = ((expected_word1 & 0xFF) << 8) | ((expected_word0 >> 8) & 0xFF)
    check(int(dut.sync_header_wc.value) == exp_wc, f"{name}: scanner WC matches selected trace bytes")
    check(int(dut.sync_header_ecc.value) == ((expected_word1 >> 8) & 0xFF), f"{name}: scanner ECC matches selected trace bytes")
    check(int(dut.sync_header_ecc_no_error.value) == 1, f"{name}: scanner selected header ECC clean")
    check(cap.stream_sop_seen_log[pkt_idx], f"{name}: stream SOP captured")
    check(cap.stream_sop_word0_log[pkt_idx] == expected_word0, f"{name}: stream SOP word0 matches scanner-selected trace bytes")
    check(cap.stream_sop_word1_log[pkt_idx] == expected_word1, f"{name}: stream SOP word1 matches scanner-selected trace bytes")


def check_clean_pair0_packet_contract(dut, cap, pkt_idx, ecc1280, name):
    check(cap.stream_sop_seen_log[pkt_idx], f"{name}: stream SOP captured")
    check(cap.stream_sop_word0_log[pkt_idx] == 0x001E, f"{name}: stream first SOP word is DI/WC-low")
    check(cap.stream_sop_word1_log[pkt_idx] == ((ecc1280 << 8) | 0x05), f"{name}: stream second SOP word is WC-high/ECC")
    check(cap.parser_di_log[pkt_idx] == 0x1E, f"{name}: parser DI")
    check(cap.parser_wc_log[pkt_idx] == 1280, f"{name}: parser WC")
    check(cap.ecc_no_error_log[pkt_idx], f"{name}: parser ECC clean")
    check(not cap.parser_ecc_uncorrectable_log[pkt_idx], f"{name}: parser ECC not uncorrectable")


# --------------------------------------------------------------------------------------
# Stimulus tasks.
# --------------------------------------------------------------------------------------
async def reset_dut(dut, cap):
    dut.rst_n.value = 0
    dut.parser_aresetn.value = 0
    dut.data_hs_p.value = 0b00
    dut.data_lp_p.value = 0b11
    dut.data_lp_n.value = 0b11
    await ClockCycles(dut.hs_clk_p, 8)
    dut.rst_n.value = 1
    await ClockCycles(dut.byte_clk, 4)
    dut.parser_aresetn.value = 1
    await ClockCycles(dut.byte_clk, 8)


async def drive_lp_state(dut, lane_lp_p, lane_lp_n, cycles):
    await FallingEdge(dut.byte_clk)
    dut.data_lp_p.value = lane_lp_p
    dut.data_lp_n.value = lane_lp_n
    await ClockCycles(dut.byte_clk, cycles)
    await Timer(1, unit="step")


async def drive_serdes_sample(dut, lane0_byte, lane1_byte):
    await FallingEdge(dut.byte_clk)
    dut.u_dut.serdes_byte_sample.value = ((lane1_byte & 0xFF) << 8) | (lane0_byte & 0xFF)
    await RisingEdge(dut.byte_clk)
    await Timer(1, unit="step")


async def drive_aligned_pair0_header(dut, ecc1280):
    await drive_serdes_sample(dut, 0xB8, 0xB8)
    await drive_serdes_sample(dut, 0x1E, 0x00)
    await drive_serdes_sample(dut, 0x05, ecc1280)
    await drive_serdes_sample(dut, 0x11, 0x22)
    await drive_serdes_sample(dut, 0x33, 0x44)
    await drive_serdes_sample(dut, 0x55, 0x66)
    await drive_serdes_sample(dut, 0x77, 0x88)
    await drive_serdes_sample(dut, 0x99, 0xAA)


async def drive_aligned_pair0_short_packet(dut, di, short_data, ecc):
    await drive_serdes_sample(dut, 0xB8, 0xB8)
    await drive_serdes_sample(dut, di, short_data & 0xFF)
    await drive_serdes_sample(dut, (short_data >> 8) & 0xFF, ecc)
    await drive_serdes_sample(dut, 0x11, 0x22)
    await drive_serdes_sample(dut, 0x33, 0x44)
    await drive_serdes_sample(dut, 0x55, 0x66)
    await drive_serdes_sample(dut, 0x77, 0x88)
    await drive_serdes_sample(dut, 0x99, 0xAA)


async def drive_lane1_delayed_header(dut, ecc1280):
    await drive_serdes_sample(dut, 0xB8, 0xB8)
    await drive_serdes_sample(dut, 0x1E, 0x02)
    await drive_serdes_sample(dut, 0x05, 0x00)
    await drive_serdes_sample(dut, 0x11, ecc1280)
    await drive_serdes_sample(dut, 0x22, 0x33)
    await drive_serdes_sample(dut, 0x44, 0x55)
    await drive_serdes_sample(dut, 0x66, 0x77)
    await drive_serdes_sample(dut, 0x88, 0x99)


async def drive_pair0_corrupt_header(dut, wc_low_bad, ecc_bad):
    await drive_serdes_sample(dut, 0xB8, 0xB8)
    await drive_serdes_sample(dut, 0x1E, wc_low_bad)
    await drive_serdes_sample(dut, 0x05, ecc_bad)
    await drive_serdes_sample(dut, 0x11, 0x22)
    await drive_serdes_sample(dut, 0x33, 0x44)
    await drive_serdes_sample(dut, 0x55, 0x66)
    await drive_serdes_sample(dut, 0x77, 0x88)
    await drive_serdes_sample(dut, 0x99, 0xAA)


async def drive_next_packet_gap(dut):
    await drive_lp_state(dut, 0b11, 0b11, 4)
    await drive_lp_state(dut, 0b00, 0b00, 4)


async def wait_for_parser_header_count(dut, cap, name, target_count):
    for _ in range(80):
        await RisingEdge(dut.byte_clk)
        await Timer(1, unit="step")
        if cap.parser_header_count >= target_count:
            return
    raise AssertionError(f"CHECK FAILED: {name}: timed out waiting for parser header count {target_count}")


async def wait_for_parser_header(dut, cap, name):
    await wait_for_parser_header_count(dut, cap, name, 1)


async def wait_for_stream_packet_count(dut, cap, name, target_count):
    for _ in range(80):
        await RisingEdge(dut.byte_clk)
        await Timer(1, unit="step")
        if cap.stream_packet_count >= target_count:
            return
    raise AssertionError(f"CHECK FAILED: {name}: timed out waiting for stream packet count {target_count}")


async def wait_for_parser_payload_count(dut, cap, name, target_count):
    for _ in range(120):
        await RisingEdge(dut.byte_clk)
        await Timer(1, unit="step")
        if cap.parser_payload_byte_count >= target_count:
            return
    raise AssertionError(f"CHECK FAILED: {name}: timed out waiting for parser payload byte count {target_count}")


async def wait_for_sync_header(dut, name):
    for _ in range(120):
        await RisingEdge(dut.byte_clk)
        await Timer(1, unit="step")
        if int(dut.sync_header_valid.value) == 1:
            return
    raise AssertionError(f"CHECK FAILED: {name}: timed out waiting for sync header")


async def _bringup(dut):
    await _start_clocks(dut)
    dut.rst_n.value = 0
    dut.parser_aresetn.value = 0
    dut.data_hs_p.value = 0b00
    dut.data_lp_p.value = 0b11
    dut.data_lp_n.value = 0b11
    cap = Capture(dut)
    cap.start()
    return cap


# --------------------------------------------------------------------------------------
# Scenarios (one @cocotb.test each; TB #1ms watchdog -> timeout).
# --------------------------------------------------------------------------------------
@cocotb.test(timeout_time=2, timeout_unit="ms")
async def aligned_pair0(dut):
    cap = await _bringup(dut)
    ecc1280 = make_ecc(0x1E, 1280)

    await reset_dut(dut, cap)
    await drive_lp_state(dut, 0b00, 0b00, 4)
    await drive_aligned_pair0_header(dut, ecc1280)

    await wait_for_parser_header(dut, cap, "aligned_pair0")
    await wait_for_stream_packet_count(dut, cap, "aligned_pair0", 1)
    await wait_for_sync_header(dut, "aligned_pair0")
    check_live_trace_pair0(dut, cap, 0x001E, (ecc1280 << 8) | 0x05, "aligned_pair0")
    check_scanner_stream_contract(dut, cap, 0, 0, "aligned_pair0")

    check(cap.captured_stream_count >= 2, "aligned_pair0: captured at least two parser stream beats")
    check(cap.captured_stream_sop[0], "aligned_pair0: SOP is on first post-SoT stream beat")
    check(cap.captured_stream_data[0] == 0x001E, "aligned_pair0: first stream beat is DI/WC-low")
    check(cap.captured_stream_data[1] == ((ecc1280 << 8) | 0x05), "aligned_pair0: second stream beat is WC-high/ECC")

    check(int(dut.sync_header_valid.value) == 1, "aligned_pair0: scanner valid")
    check(int(dut.sync_header_score.value) == 15, "aligned_pair0: scanner score 15")
    check(int(dut.sync_header_pairing.value) == 0, "aligned_pair0: scanner pairing 0")
    check(int(dut.sync_header_di.value) == 0x1E, "aligned_pair0: scanner DI")
    check(int(dut.sync_header_wc.value) == 1280, "aligned_pair0: scanner WC")
    check(int(dut.sync_header_ecc_no_error.value) == 1, "aligned_pair0: scanner ECC clean")

    check(cap.parser_last_hdr_seen, "aligned_pair0: parser header valid")
    check(cap.parser_last_pkt_di == 0x1E, "aligned_pair0: parser DI")
    check(cap.parser_last_pkt_wc == 1280, "aligned_pair0: parser WC")
    check(cap.parser_last_ecc_seen, "aligned_pair0: parser ECC seen")
    check(cap.parser_last_ecc_no_error, "aligned_pair0: parser ECC no-error")
    check(not cap.parser_last_pkt_ecc_uncorrectable, "aligned_pair0: parser ECC clean")
    await wait_for_parser_payload_count(dut, cap, "aligned_pair0", 2)
    check(cap.parser_payload_byte_count >= 2, "aligned_pair0: parser receives payload after scanner-qualified release")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def frame_short_release(dut):
    cap = await _bringup(dut)
    ecc1280 = make_ecc(0x1E, 1280)
    ecc_fs = make_ecc(0x00, 0x0001)

    await reset_dut(dut, cap)
    await drive_lp_state(dut, 0b00, 0b00, 4)
    await drive_aligned_pair0_short_packet(dut, 0x00, 0x0001, ecc_fs)

    await wait_for_parser_header_count(dut, cap, "frame_short_fs", 1)
    await wait_for_stream_packet_count(dut, cap, "frame_short_fs", 1)
    await wait_for_sync_header(dut, "frame_short_fs")
    check_scanner_stream_contract(dut, cap, 0, 0, "frame_short_fs")

    check(int(dut.sync_header_score.value) == 13, "frame_short_fs: clean short packet reaches release threshold")
    check(int(dut.sync_header_di.value) == 0x00, "frame_short_fs: scanner DI is FS")
    check(int(dut.sync_header_wc.value) == 0x0001, "frame_short_fs: scanner short data")
    check(cap.parser_di_log[0] == 0x00, "frame_short_fs: parser DI is FS")
    check(cap.parser_wc_log[0] == 0x0001, "frame_short_fs: parser short data")
    check(int(dut.parser_short_count.value) == 1, "frame_short_fs: parser short count increments")
    check(cap.ecc_no_error_log[0], "frame_short_fs: parser ECC clean")

    await drive_next_packet_gap(dut)
    await drive_aligned_pair0_header(dut, ecc1280)

    await wait_for_parser_header_count(dut, cap, "frame_short_then_long", 2)
    await wait_for_stream_packet_count(dut, cap, "frame_short_then_long", 2)
    check_clean_pair0_packet_contract(dut, cap, 1, ecc1280, "frame_short_then_long")
    await wait_for_parser_payload_count(dut, cap, "frame_short_then_long", 2)
    check(cap.parser_payload_byte_count >= 2, "frame_short_then_long: long payload follows accepted FS short packet")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def lane1_delayed_auto_pair(dut):
    cap = await _bringup(dut)
    ecc1280 = make_ecc(0x1E, 1280)

    await reset_dut(dut, cap)
    await drive_lp_state(dut, 0b00, 0b00, 4)
    await drive_lane1_delayed_header(dut, ecc1280)

    await wait_for_parser_header_count(dut, cap, "lane1_delayed_first", 1)
    await wait_for_stream_packet_count(dut, cap, "lane1_delayed_first", 1)
    await wait_for_sync_header(dut, "lane1_delayed_first")
    check_live_trace_pair0(dut, cap, 0x021E, 0x0005, "lane1_delayed_first")
    check_scanner_stream_contract(dut, cap, 0, 2, "lane1_delayed_first")

    check(cap.captured_stream_count >= 2, "lane1_delayed_first: captured at least two parser stream beats")
    check(cap.captured_stream_sop[0], "lane1_delayed_first: SOP is on the scanner-qualified stream beat")
    check(cap.captured_stream_data[0] == 0x001E, "lane1_delayed_first: stream emits repaired DI/WC-low")
    check(cap.captured_stream_data[1] == ((ecc1280 << 8) | 0x05), "lane1_delayed_first: stream emits repaired WC-high/ECC")

    check(int(dut.sync_header_valid.value) == 1, "lane1_delayed_first: scanner valid")
    check(int(dut.sync_header_score.value) == 15, "lane1_delayed_first: scanner score 15")
    check(int(dut.sync_header_pairing.value) == 2, "lane1_delayed_first: scanner chooses pairing 2")
    check(int(dut.sync_header_di.value) == 0x1E, "lane1_delayed_first: scanner DI")
    check(int(dut.sync_header_wc.value) == 1280, "lane1_delayed_first: scanner WC")
    check(int(dut.sync_header_ecc_no_error.value) == 1, "lane1_delayed_first: scanner ECC clean")
    check(int(dut.stream_pairing_next_dbg.value) == 2, "lane1_delayed_first: scanner pairing is learned for next packet")
    check(int(dut.stream_pairing_active_dbg.value) == 2, "lane1_delayed_first: scanner pairing is applied to current packet")

    check(cap.parser_last_hdr_seen, "lane1_delayed_first: parser header valid")
    check(cap.parser_last_pkt_di == 0x1E, "lane1_delayed_first: parser DI")
    check(cap.parser_last_pkt_wc == 1280, "lane1_delayed_first: parser WC repaired to 1280")
    check(cap.parser_last_ecc_seen, "lane1_delayed_first: parser ECC seen")
    check(cap.parser_last_ecc_no_error, "lane1_delayed_first: parser header ECC is clean")
    check(not cap.parser_last_pkt_ecc_uncorrectable, "lane1_delayed_first: parser ECC not uncorrectable")

    await drive_lp_state(dut, 0b11, 0b11, 4)
    await drive_lp_state(dut, 0b00, 0b00, 4)
    await drive_lane1_delayed_header(dut, ecc1280)

    await wait_for_parser_header_count(dut, cap, "lane1_delayed_second", 2)
    await wait_for_stream_packet_count(dut, cap, "lane1_delayed_second", 2)
    check_scanner_stream_contract(dut, cap, 1, 2, "lane1_delayed_second")

    check(int(dut.stream_pairing_active_dbg.value) == 2, "lane1_delayed_second: scanner pairing applied at current SoT")
    check(cap.parser_last_pkt_di == 0x1E, "lane1_delayed_second: parser DI")
    check(cap.parser_last_pkt_wc == 1280, "lane1_delayed_second: parser WC remains 1280")
    check(cap.parser_last_ecc_seen, "lane1_delayed_second: parser ECC seen")
    check(cap.parser_last_ecc_no_error, "lane1_delayed_second: parser ECC clean")
    check(not cap.parser_last_pkt_ecc_uncorrectable, "lane1_delayed_second: parser ECC not uncorrectable")


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def sustained_valid_pair0_contract(dut):
    cap = await _bringup(dut)
    ecc1280 = make_ecc(0x1E, 1280)

    await reset_dut(dut, cap)
    await drive_lp_state(dut, 0b00, 0b00, 4)
    for pkt_idx in range(10):
        if pkt_idx != 0:
            await drive_next_packet_gap(dut)
        await drive_aligned_pair0_header(dut, ecc1280)
        await wait_for_parser_header_count(dut, cap, f"sustained_valid_pair0_{pkt_idx}", pkt_idx + 1)
        await wait_for_stream_packet_count(dut, cap, f"sustained_valid_pair0_{pkt_idx}", pkt_idx + 1)
        check_live_trace_pair0(dut, cap, 0x001E, (ecc1280 << 8) | 0x05, f"sustained_valid_pair0_{pkt_idx}")
        check_clean_pair0_packet_contract(dut, cap, pkt_idx, ecc1280, f"sustained_valid_pair0_{pkt_idx}")

    check(int(dut.sync_header_valid.value) == 1, "sustained_valid_pair0: scanner valid")
    check(int(dut.sync_header_pairing.value) == 0, "sustained_valid_pair0: scanner pairing 0")
    check(int(dut.sync_header_score.value) == 15, "sustained_valid_pair0: scanner score 15")
    check(int(dut.sync_header_wc.value) == 1280, "sustained_valid_pair0: scanner WC 1280")
    check(int(dut.stream_pairing_active_dbg.value) == 0, "sustained_valid_pair0: active stream pairing stays pair0")
    check(int(dut.stream_pairing_next_dbg.value) == 0, "sustained_valid_pair0: learned stream pairing stays pair0")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def diagnostic_bad_live_sop_signature(dut):
    cap = await _bringup(dut)
    ecc1280 = make_ecc(0x1E, 1280)

    await reset_dut(dut, cap)
    await drive_lp_state(dut, 0b00, 0b00, 4)
    await drive_aligned_pair0_header(dut, ecc1280)

    await wait_for_parser_header_count(dut, cap, "latched_clean_first", 1)
    await wait_for_stream_packet_count(dut, cap, "latched_clean_first", 1)
    await wait_for_sync_header(dut, "latched_clean_first")

    check(int(dut.sync_header_valid.value) == 1, "latched_clean_first: scanner valid")
    check(int(dut.sync_header_pairing.value) == 0, "latched_clean_first: scanner pairing 0")
    check(int(dut.sync_header_score.value) == 15, "latched_clean_first: scanner score 15")
    check(int(dut.sync_header_di.value) == 0x1E, "latched_clean_first: scanner DI")
    check(int(dut.sync_header_wc.value) == 1280, "latched_clean_first: scanner WC 1280")
    check(cap.parser_wc_log[0] == 1280, "latched_clean_first: parser WC 1280")
    check(cap.ecc_no_error_log[0], "latched_clean_first: parser ECC clean")

    await drive_next_packet_gap(dut)
    await drive_pair0_corrupt_header(dut, 0x02, 0x1F)

    for _ in range(100):
        await RisingEdge(dut.byte_clk)
        await Timer(1, unit="step")
    check_live_trace_pair0(dut, cap, 0x021E, 0x1F05, "diagnostic_bad_live_sop")

    check(int(dut.sync_header_valid.value) == 0, "diagnostic_bad_live_sop: scanner rejects corrupt header")
    check(int(dut.sync_header_score.value) < 13, "diagnostic_bad_live_sop: scanner score stays below release threshold")
    check(cap.stream_packet_count == 1, "diagnostic_bad_live_sop: corrupt live pair0 is not released as a stream SOP")
    check(cap.parser_header_count == 1, "diagnostic_bad_live_sop: parser does not see a second corrupt header")
    check(cap.ecc_header_count == 1, "diagnostic_bad_live_sop: parser ECC does not process the rejected header")


def test_dphy_hs_stream_boundary():
    from runner_support import build_and_test

    here = Path(__file__).resolve().parent
    build_and_test(
        block="dphy_hs_stream_boundary",
        sources=[
            str(here / "dphy_hs_stream_boundary_stubs.sv"),
            "rtl/prototype/dphy_hs_byte_probe.sv",
            "rtl/mipi_rx/csi2_packet_parser.sv",
            "rtl/mipi_rx/csi2_header_ecc.sv",
        ],
        toplevel="dphy_hs_stream_boundary_harness",
        test_module="test_dphy_hs_stream_boundary",
        test_dir=here,
        engine="verilator",
    )
