"""cocotb port of verification/tb/tb_dphy_runtime_dt.sv.

Verifies the runtime EXPECTED_LONG_DT control mechanism in dphy_hs_byte_probe: the
``runtime_expected_long_dt`` knob (synchronized to byte_clk) overrides the build-time
``EXPECTED_LONG_DT`` parameter when non-zero, gating whether a CSI-2 long-packet header
scores high enough to raise ``sync_header_valid``.

The DSim TB does NOT exercise the ISERDES deserialization path. Its IBUFDS/BUFIO/BUFR/
IDELAY/ISERDESE2 stubs are pure pass-throughs (identical to lib/verilator_unisim_stubs.sv);
the ISERDES emits 0. Instead the TB injects header bytes by *forcing the DUT-internal
register* ``dut.serdes_byte_sample[lane]`` at negedge byte_clk (the RTL reloads it from
``serdes_byte`` -- i.e. 0 -- every posedge, but the always_ff reads the forced value in its
combinational computations before the NBA overwrite, exactly as the SV force did). This
cocotb port reproduces that byte-injection by depositing into the same hierarchical signal.

Scenarios (1:1 with the TB ``run_scenario`` calls):
  S0 runtime_dt=0x00 (default->param 0x1e), DI=0x1E ECC=0x1E  -> 1 pulse, sync_di=0x1E
  S1 runtime_dt=0x1E (explicit match),      DI=0x1E ECC=0x1E  -> 1 pulse, sync_di=0x1E
  S2 runtime_dt=0x1F (mismatch),            DI=0x1E ECC=0x1E  -> 0 pulses (no header)
  S3 runtime_dt=0x1E, DI=0x1F ECC=0x1E (1-bit err) -> ECC corrects to 0x1E -> 1 pulse
  S4 runtime_dt=0x1F, DI=0x1F ECC=0x19 (no-error)  -> runtime override match -> 1 pulse
  S5 4 repeated default-DT headers -> sync_header_valid pulse count == 4

A pulse-counter coroutine reproduces the TB's ``always_ff`` rising-edge counter on
``sync_header_valid``. The ``#20ms`` watchdog maps to the test timeout. All scenarios share
one cumulative run (the TB is one ``initial`` block after a single reset), so they live in
one @cocotb.test(); num_tests = 1.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge, Timer

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.scoreboard import check  # noqa: E402


class PulseCounter:
    """Mirror the TB always_ff rising-edge counter on sync_header_valid."""

    def __init__(self, dut):
        self.dut = dut
        self.count = 0
        self._prev = 0

    def start(self, clk):
        return cocotb.start_soon(self._run(clk))

    async def _run(self, clk):
        while True:
            await RisingEdge(clk)
            cur = int(self.dut.sync_header_valid.value)
            if cur and not self._prev:
                self.count += 1
            self._prev = cur


async def _reset_dut(dut):
    """Mirror the TB reset_dut task."""
    dut.rst_n.value = 0
    dut.runtime_idelay_tap.value = 8
    dut.runtime_idelay_tap_lane1.value = 8
    dut.runtime_expected_long_dt.value = 0x00
    dut.dphy_data_hs_p.value = 0b00
    dut.dphy_data_lp_p.value = 0b11
    dut.dphy_data_lp_n.value = 0b11
    # The BUFR stub forces byte_clk=0 while rst_n=0 (CLR=!rst_n), so byte_clk does NOT
    # toggle during reset. Match the TB: wait on the always-running HS clock for the
    # 8-cycle reset hold, then on byte_clk for the 12-cycle post-release settle.
    await ClockCycles(dut.dphy_hs_clock_clk_p, 8)
    dut.rst_n.value = 1
    await ClockCycles(dut.byte_clk, 12)


async def _drive_lp_state(dut, lp_p, lp_n, cycles):
    """Mirror drive_lp_state: set LP state on negedge, hold `cycles` byte_clks."""
    await FallingEdge(dut.byte_clk)
    dut.dphy_data_lp_p.value = lp_p
    dut.dphy_data_lp_n.value = lp_n
    await ClockCycles(dut.byte_clk, cycles)


async def _drive_serdes_sample(dut, lane0_byte, lane1_byte):
    """Mirror drive_serdes_sample: force the DUT-internal serdes_byte_sample at negedge,
    let the DUT capture it at the next posedge.

    ``serdes_byte_sample`` is ``logic [LANES-1:0][7:0]`` -- a fully packed 2D array that
    Verilator flattens to one 16-bit vector, so cocotb cannot index it with ``[lane]``
    (TB used a hierarchical index into the unpacked-in-SV force). Write the whole word:
    lane0 -> bits [7:0], lane1 -> bits [15:8] (outer index is the MSB half)."""
    await FallingEdge(dut.byte_clk)
    dut.serdes_byte_sample.value = ((lane1_byte & 0xFF) << 8) | (lane0_byte & 0xFF)
    await RisingEdge(dut.byte_clk)


async def _drive_header(dut, di, wcl, wcm, ecc):
    """Mirror drive_header: LP-exit -> 8 injected byte slots -> settle 600 -> LP-11."""
    await _drive_lp_state(dut, 0b00, 0b00, 4)
    await _drive_serdes_sample(dut, 0xB8, 0xB8)  # slot 0 SoT
    await _drive_serdes_sample(dut, di, wcl)     # slot 1: DI / WC_L
    await _drive_serdes_sample(dut, wcm, ecc)    # slot 2: WC_M / ECC
    await _drive_serdes_sample(dut, 0x10, 0x11)  # slot 3
    await _drive_serdes_sample(dut, 0x12, 0x13)
    await _drive_serdes_sample(dut, 0x14, 0x15)
    await _drive_serdes_sample(dut, 0x16, 0x17)
    await _drive_serdes_sample(dut, 0x18, 0x19)
    for _ in range(600):
        await RisingEdge(dut.byte_clk)
    await _drive_lp_state(dut, 0b11, 0b11, 8)


async def _run_scenario(dut, pc, name, runtime_dt, di, wcl, wcm, ecc,
                        expected_pulses, expected_di_corrected):
    dut._log.info(f"--- {name} ---")
    dut.runtime_expected_long_dt.value = runtime_dt
    await ClockCycles(dut.byte_clk, 8)
    start_count = pc.count
    await _drive_header(dut, di, wcl, wcm, ecc)
    diff = pc.count - start_count
    dut._log.info(
        f"  runtime_dt=0x{runtime_dt:02x}, stim DI=0x{di:02x} ECC=0x{ecc:02x} -> "
        f"pulses(diff)={diff}, sync_di=0x{int(dut.sync_header_di.value):02x} "
        f"score={int(dut.sync_header_score.value)}"
    )
    check(diff == expected_pulses,
          f"{name}.pulse_diff (got {diff}, expected {expected_pulses})")
    if expected_pulses > 0:
        got_di = int(dut.sync_header_di.value)
        check(got_di == expected_di_corrected,
              f"{name}.sync_header_di (got 0x{got_di:02x}, expected 0x{expected_di_corrected:02x})")


@cocotb.test(timeout_time=40, timeout_unit="ms")
async def runtime_dt(dut):
    # hs_clk_p toggles every 1ns (500 MHz) in the TB; byte_clk = pass-through BUFR of it.
    # Drive the differential HS clock; the pass-through IBUFDS/BUFIO/BUFR make byte_clk
    # track hs_clk_p. Also generate byte_clk explicitly is NOT possible (it's an output),
    # so drive the HS clock source and let the stubs propagate it.
    cocotb.start_soon(Clock(dut.dphy_hs_clock_clk_p, 2.0, unit="ns").start())

    # In the TB, data_hs_n = ~data_hs_p and both LP nets start at 2'b11. The pass-through
    # IBUFDS ignores IB, so data_hs_n is irrelevant, but drive it for completeness.
    dut.dphy_data_hs_p.value = 0b00
    dut.dphy_data_hs_n.value = 0b11
    dut.dphy_data_lp_p.value = 0b11
    dut.dphy_data_lp_n.value = 0b11
    # tie unused runtime controls
    dut.runtime_bitslip_phase.value = 0
    dut.runtime_bitslip_phase_lane1.value = 0
    dut.runtime_lane1_sweep_enable.value = 0
    dut.sup_enable.value = 0
    dut.sup_bufr_clr.value = 0
    dut.sup_serdes_rst.value = 0
    dut.sup_hs_settled.value = 0
    dut.idelay_ref_reset.value = 1

    # idelay_ref_clk toggles every 2.5ns in the TB.
    cocotb.start_soon(Clock(dut.idelay_ref_clk, 5.0, unit="ns").start())

    await _reset_dut(dut)
    dut.idelay_ref_reset.value = 0

    pc = PulseCounter(dut)
    pc.start(dut.byte_clk)

    await _run_scenario(dut, pc, "S0_default_clean", 0x00,
                        0x1E, 0x00, 0x05, 0x1E, 1, 0x1E)
    await _run_scenario(dut, pc, "S1_explicit_1E_clean", 0x1E,
                        0x1E, 0x00, 0x05, 0x1E, 1, 0x1E)
    await _run_scenario(dut, pc, "S2_mismatch_1F_vs_1E", 0x1F,
                        0x1E, 0x00, 0x05, 0x1E, 0, 0x00)
    await _run_scenario(dut, pc, "S3_ecc_corrected_DI_1F_to_1E", 0x1E,
                        0x1F, 0x00, 0x05, 0x1E, 1, 0x1E)
    await _run_scenario(dut, pc, "S4_runtime_1F_match_raw", 0x1F,
                        0x1F, 0x00, 0x05, 0x19, 1, 0x1F)

    # S5: 4 repeated headers with default DT -> pulse count 4
    dut.runtime_expected_long_dt.value = 0x00
    await ClockCycles(dut.byte_clk, 8)
    start_count = pc.count
    for _ in range(4):
        await _drive_header(dut, 0x1E, 0x00, 0x05, 0x1E)
    diff = pc.count - start_count
    dut._log.info(f"--- S5_repeated_4x --- 4 headers driven, pulses(diff)={diff}")
    check(diff == 4, f"S5.pulse_count_4x (got {diff}, expected 4)")


def test_dphy_runtime_dt():
    from runner_support import build_and_test

    build_and_test(
        block="dphy_runtime_dt",
        sources=[
            "verification/cocotb/dphy_runtime_dt/dphy_runtime_dt_stubs.sv",
            "rtl/prototype/dphy_hs_byte_probe.sv",
        ],
        toplevel="dphy_hs_byte_probe",
        test_module="test_dphy_runtime_dt",
        test_dir=Path(__file__).resolve().parent,
        parameters={
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
            "EXPECTED_LONG_DT": 0x1e,
            "EXPECTED_LONG_WC": 1280,
            "MIN_SYNC_HEADER_SCORE": 13,
            "STREAM_PAIRING": 0,
        },
        engine="verilator",
    )
