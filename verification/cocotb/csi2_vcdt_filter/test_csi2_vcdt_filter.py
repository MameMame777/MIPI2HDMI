"""cocotb port of verification/tb/tb_csi2_vcdt_filter.sv.

The DUT (``csi2_vcdt_filter``) is a valid-only sink of the CSI-2 packet-parser output:
header pulse (``pkt_hdr_valid``), payload beats (``payload_valid``), and a ``pkt_done``
pulse (with CRC status). There is no ``tready`` handshake, so the inputs are driven
directly on the negedge-equivalent (RisingEdge-based) cadence the TB uses.

Faithful 1:1 port. The DSim TB runs a single cumulative ``initial`` block of nine
scenarios separated by ``clear_logs()`` pulses; those pulses reset a local ``always_ff``
logger (start/end/payload/err counters + payload log) but NOT the DUT, so the DUT's
``sts_drop_vc_cnt`` / ``sts_drop_dt_cnt`` accumulate across scenarios. To preserve that
cumulative status-counter behaviour this port keeps everything inside ONE
``@cocotb.test()`` with a single reset, exactly mirroring the TB.

Translation map:
  - SV ``always_ff`` logger              -> the ``Logger`` monitor coroutine.
  - SV ``clear_logs()`` (pulse)          -> ``Logger.clear()`` (drives clear_logs_pulse).
  - SV tasks ``drive_header`` / ``drive_payload`` / ``drive_done`` /
    ``drive_short_packet_same_cycle`` -> the async helpers of the same name.
  - SV ``check_condition``               -> ``lib.scoreboard.check``.
  - SV ``#1ms`` watchdog                 -> ``@cocotb.test(timeout_time=1, unit="ms")``.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import ClockCycles, RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import start_clock  # noqa: E402
from lib.scoreboard import check  # noqa: E402


class Logger:
    """Mirror of the TB ``always_ff`` block: counts out_pkt_start / out_payload_valid /
    out_pkt_end (+ out_pkt_err) and logs the payload stream. ``clear_logs_pulse`` (like the
    TB) zeroes only these local counters, not the DUT."""

    def __init__(self, dut):
        self.dut = dut
        self.clear_pulse = False
        self._reset_state()

    def _reset_state(self):
        self.start_count = 0
        self.end_count = 0
        self.payload_count = 0
        self.err_count = 0
        self.payload_log = [0] * 16
        self.first_log = [0] * 16
        self.last_log = [0] * 16
        self.last_start_di = 0
        self.last_start_wc = 0
        self.last_start_short = 0
        self.last_start_long = 0

    def start(self, clk):
        return cocotb.start_soon(self._run(clk))

    async def _run(self, clk):
        d = self.dut
        while True:
            await RisingEdge(clk)
            # Same-cycle semantics as the SV always_ff: sample reset/clear and the
            # outputs registered on this edge.
            if int(d.core_aresetn.value) == 0 or self.clear_pulse:
                self._reset_state()
                continue
            if int(d.out_pkt_start.value) == 1:
                self.start_count += 1
                self.last_start_di = int(d.out_pkt_di.value)
                self.last_start_wc = int(d.out_pkt_wc.value)
                self.last_start_short = int(d.out_pkt_is_short.value)
                self.last_start_long = int(d.out_pkt_is_long.value)
            if int(d.out_payload_valid.value) == 1:
                idx = self.payload_count
                if idx < 16:
                    self.payload_log[idx] = int(d.out_payload_data.value)
                    self.first_log[idx] = int(d.out_payload_first.value)
                    self.last_log[idx] = int(d.out_payload_last.value)
                self.payload_count += 1
            if int(d.out_pkt_end.value) == 1:
                self.end_count += 1
                if int(d.out_pkt_err.value) == 1:
                    self.err_count += 1


async def reset_dut(dut, clk):
    """Port of the SV ``reset_dut`` task: default config + all-idle inputs, hold reset
    low 8 cycles, release, settle 2 cycles."""
    dut.core_aresetn.value = 0
    dut.cfg_expected_vc.value = 0
    dut.cfg_expected_dt.value = 0x2A
    dut.cfg_pass_short.value = 1
    dut.cfg_pass_emb_data.value = 0
    dut.pkt_hdr_valid.value = 0
    dut.pkt_di.value = 0
    dut.pkt_wc.value = 0
    dut.pkt_is_long.value = 0
    dut.pkt_is_short.value = 0
    dut.pkt_done.value = 0
    dut.ecc_corrected.value = 0
    dut.ecc_uncorrectable.value = 0
    dut.crc_check_valid.value = 0
    dut.crc_match.value = 1
    dut.payload_data.value = 0
    dut.payload_valid.value = 0
    dut.payload_first.value = 0
    dut.payload_last.value = 0
    await ClockCycles(clk, 8)
    dut.core_aresetn.value = 1
    await ClockCycles(clk, 2)


async def clear_logs(dut, clk, logger):
    """Port of the SV ``clear_logs`` task: pulse clear_logs_pulse for one cycle."""
    await RisingEdge(clk)
    logger.clear_pulse = True
    await RisingEdge(clk)
    logger.clear_pulse = False
    await RisingEdge(clk)


async def drive_header(dut, clk, di, wc, is_long, is_short, ecc_uncorr):
    await RisingEdge(clk)
    dut.pkt_di.value = di
    dut.pkt_wc.value = wc
    dut.pkt_is_long.value = is_long
    dut.pkt_is_short.value = is_short
    dut.ecc_uncorrectable.value = ecc_uncorr
    dut.pkt_hdr_valid.value = 1
    await RisingEdge(clk)
    dut.pkt_hdr_valid.value = 0
    dut.ecc_uncorrectable.value = 0


async def drive_payload(dut, clk, data, first, last):
    await RisingEdge(clk)
    dut.payload_data.value = data
    dut.payload_first.value = first
    dut.payload_last.value = last
    dut.payload_valid.value = 1
    await RisingEdge(clk)
    dut.payload_valid.value = 0
    dut.payload_first.value = 0
    dut.payload_last.value = 0


async def drive_done(dut, clk, crc_valid, crc_ok):
    await RisingEdge(clk)
    dut.crc_check_valid.value = crc_valid
    dut.crc_match.value = crc_ok
    dut.pkt_done.value = 1
    await RisingEdge(clk)
    dut.crc_check_valid.value = 0
    dut.crc_match.value = 1
    dut.pkt_done.value = 0


async def drive_short_packet_same_cycle(dut, clk, di, wc, ecc_uncorr):
    await RisingEdge(clk)
    dut.pkt_di.value = di
    dut.pkt_wc.value = wc
    dut.pkt_is_long.value = 0
    dut.pkt_is_short.value = 1
    dut.ecc_uncorrectable.value = ecc_uncorr
    dut.pkt_hdr_valid.value = 1
    dut.pkt_done.value = 1
    await RisingEdge(clk)
    dut.pkt_hdr_valid.value = 0
    dut.pkt_done.value = 0
    dut.ecc_uncorrectable.value = 0


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def vcdt_filter_full(dut):
    """Single cumulative run: all nine TB scenarios in order (drop counters accumulate)."""
    clk = dut.core_clk
    start_clock(clk, period_ns=10.0)

    logger = Logger(dut)
    logger.start(clk)

    await reset_dut(dut, clk)

    # --- Scenario 1: clean RAW8 long packet passes -----------------------------------
    await clear_logs(dut, clk, logger)
    await drive_header(dut, clk, 0x2A, 3, 1, 0, 0)
    await drive_payload(dut, clk, 0xAA, 1, 0)
    await drive_payload(dut, clk, 0xBB, 0, 0)
    await drive_payload(dut, clk, 0xCC, 0, 1)
    await drive_done(dut, clk, 1, 1)
    await ClockCycles(clk, 3)
    check(logger.start_count == 1, "RAW8 packet start passes")
    check(logger.end_count == 1, "RAW8 packet end passes")
    check(logger.payload_count == 3, "RAW8 payload count")
    check(logger.payload_log[0] == 0xAA, "payload byte 0")
    check(logger.payload_log[1] == 0xBB, "payload byte 1")
    check(logger.payload_log[2] == 0xCC, "payload byte 2")
    check(logger.first_log[0] == 1, "payload first passes")
    check(logger.last_log[2] == 1, "payload last passes")
    check(logger.err_count == 0, "no error for clean packet")
    check(logger.last_start_di == 0x2A, "start DI pass")
    check(logger.last_start_wc == 3, "start WC pass")
    check(logger.last_start_long == 1, "long flag pass")

    # --- Scenario 2: short packet passes (same-cycle hdr+done) ------------------------
    await clear_logs(dut, clk, logger)
    await drive_short_packet_same_cycle(dut, clk, 0x00, 0x1234, 0)
    await ClockCycles(clk, 3)
    check(logger.start_count == 1, "short packet start passes")
    check(logger.end_count == 1, "short packet end passes")
    check(logger.last_start_short == 1, "short flag pass")
    check(logger.payload_count == 0, "short packet has no payload")

    # --- Scenario 3: VC mismatch is dropped ------------------------------------------
    await clear_logs(dut, clk, logger)
    await drive_header(dut, clk, 0xAA, 2, 1, 0, 0)
    await drive_payload(dut, clk, 0x11, 1, 0)
    await drive_payload(dut, clk, 0x22, 0, 1)
    await drive_done(dut, clk, 1, 1)
    await ClockCycles(clk, 3)
    check(logger.start_count == 0, "VC mismatch suppresses start")
    check(logger.end_count == 0, "VC mismatch suppresses end")
    check(logger.payload_count == 0, "VC mismatch suppresses payload")
    check(int(dut.sts_drop_vc_cnt.value) == 1, "VC drop count")

    # --- Scenario 4: DT mismatch is dropped ------------------------------------------
    await clear_logs(dut, clk, logger)
    await drive_header(dut, clk, 0x2B, 2, 1, 0, 0)
    await drive_payload(dut, clk, 0x33, 1, 0)
    await drive_payload(dut, clk, 0x44, 0, 1)
    await drive_done(dut, clk, 1, 1)
    await ClockCycles(clk, 3)
    check(logger.start_count == 0, "DT mismatch suppresses start")
    check(logger.payload_count == 0, "DT mismatch suppresses payload")
    check(int(dut.sts_drop_dt_cnt.value) == 1, "DT drop count")

    # --- Scenario 5: embedded data passes when enabled -------------------------------
    dut.cfg_pass_emb_data.value = 1
    await clear_logs(dut, clk, logger)
    await drive_header(dut, clk, 0x12, 1, 1, 0, 0)
    await drive_payload(dut, clk, 0x55, 1, 1)
    await drive_done(dut, clk, 1, 1)
    await ClockCycles(clk, 3)
    check(logger.start_count == 1, "embedded data pass when enabled")
    check(logger.payload_count == 1, "embedded payload pass")

    # --- Scenario 6: ECC-uncorrectable becomes a packet error (not dropped) ----------
    await clear_logs(dut, clk, logger)
    await drive_header(dut, clk, 0x2A, 1, 1, 0, 1)
    await drive_payload(dut, clk, 0x66, 1, 1)
    await drive_done(dut, clk, 1, 1)
    await ClockCycles(clk, 3)
    check(logger.end_count == 1, "ECC error packet ends")
    check(logger.err_count == 1, "ECC uncorrectable becomes packet error")
    check(logger.payload_count == 1, "ECC error frame is not dropped")

    # --- Scenario 7: CRC mismatch becomes a packet error (not dropped) ---------------
    await clear_logs(dut, clk, logger)
    await drive_header(dut, clk, 0x2A, 1, 1, 0, 0)
    await drive_payload(dut, clk, 0x77, 1, 1)
    await drive_done(dut, clk, 1, 0)
    await ClockCycles(clk, 3)
    check(logger.end_count == 1, "CRC error packet ends")
    check(logger.err_count == 1, "CRC mismatch becomes packet error")
    check(logger.payload_count == 1, "CRC error frame is not dropped")

    # --- Scenario 8: short packet can be blocked (cfg_pass_short=0) -------------------
    dut.cfg_pass_short.value = 0
    await clear_logs(dut, clk, logger)
    await drive_header(dut, clk, 0x01, 0x0000, 0, 1, 0)
    await drive_done(dut, clk, 0, 1)
    await ClockCycles(clk, 3)
    check(logger.start_count == 0, "short packet can be blocked")
    check(int(dut.sts_drop_dt_cnt.value) == 2, "short block increments DT drop count")

    await ClockCycles(clk, 10)


def test_csi2_vcdt_filter():
    from runner_support import build_and_test

    build_and_test(
        block="csi2_vcdt_filter",
        sources=["rtl/mipi_rx/csi2_vcdt_filter.sv"],
        toplevel="csi2_vcdt_filter",
        test_module="test_csi2_vcdt_filter",
        test_dir=Path(__file__).resolve().parent,
        parameters={"NUM_VC": 4, "NUM_DT_RAW": 2},
        engine="verilator",
    )
