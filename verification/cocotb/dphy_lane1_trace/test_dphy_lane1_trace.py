"""cocotb port of verification/tb/tb_dphy_lane1_trace.sv (D-PHY lane-1 byte-capture trace).

This is a *trace* testbench for ``dphy_hs_byte_probe`` configured with the DEPLOYED
bitstream parameters (FIXED_BITSLIP_PHASE=6/6, FIXED_TRANSFORM=1, TRACE_TRIGGER_MODE=3).
It drives 5 back-to-back CSI-2 long-packet headers (DI=0x1E, WC=1280, ECC) with LP->HS
transitions and verifies that lane-1's trace slot[2] (the ECC-byte position) reads 0x1E --
i.e. the RTL byte-capture pipeline is correct (the check the hardware regression chases).

STIMULUS PATH: the DSim TB injects data by *force-writing the DUT's internal*
``serdes_byte_sample`` register at ``negedge byte_clk`` (there is NO input port for it; the
real data path goes through the Xilinx ISERDES, which the behavioral stub zeroes). It also
polls internal ``lane_bitslip_phase`` / ``sweep_hold_count`` to wait for retrain. cocotb's
Verilator runner builds with ``--public-flat-rw`` (verified in the build command), so these
module-internal signals are read/write-accessible. The force-then-posedge-NBA-overwrite on
``serdes_byte_sample`` (the RTL does ``serdes_byte_sample <= serdes_byte`` every byte_clk)
is safe: the write is committed on the falling edge and the always_ff samples it at the next
posedge *before* the NBA re-zeroes it -- exactly the SV negedge blocking-write semantics.

WHY VERILATOR (not Icarus): the installed Icarus rejects the RTL outright -- ``iverilog``
errors ``sorry: overriding the default variable lifetime is not yet supported`` on the
many block-local ``automatic`` declarations inside the DUT's ``always_ff``. Verilator
elaborates them fine, and the test needs no real deserialization/bitslip timing (stimulus
is the forced ``serdes_byte_sample``), so the pass-through Xilinx stubs are sufficient.

The Xilinx primitives are supplied by the local dphy_lane1_trace_stubs.sv (byte-for-byte the
same pass-through models the DSim TB defined inline and the shared
lib/verilator_unisim_stubs.sv: IBUFDS/BUFIO/BUFR/IDELAYCTRL/IDELAYE2/ISERDESE2 -- a local
copy is used only because the shared lib's header comment contains a token Verilator
misreads as a metacomment pragma). ``byte_clk`` is a DUT *output* = BUFR /4 stub passthrough
of ``hs_clk_p``; the test drives ``idelay_ref_clk`` (#2.5) and ``hs_clk_p`` (#5) and clocks
off ``byte_clk`` (frozen while in reset, so the reset phase clocks off ``hs_clk_p``).

Faithful mapping of the single ``initial`` main block:
  reset_dut -> _reset_dut ; wait_for_retrain_done -> _wait_for_retrain_done ;
  emit_one_packet (x5) -> _emit_one_packet ; check_byte -> lib.scoreboard.check ;
  #50_000_000 watchdog -> @cocotb.test(timeout_time=...). num_tests = 1.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, Timer

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.scoreboard import check  # noqa: E402


# ---------------------------------------------------------------------------
# TB-side reference helpers (1:1 with the SV functions)
# ---------------------------------------------------------------------------
def reverse8(v: int) -> int:
    """Bit-reverse an 8-bit value (SV reverse8)."""
    v &= 0xFF
    r = 0
    for i in range(8):
        r |= ((v >> (7 - i)) & 1) << i
    return r


def ref_ecc6(data24: int) -> int:
    """Independent CSI-2 ECC reference (SV ref_ecc6), data = {WC[15:0], DI[7:0]}."""
    def b(i: int) -> int:
        return (data24 >> i) & 1

    e = [0] * 6
    e[0] = b(0) ^ b(1) ^ b(2) ^ b(4) ^ b(5) ^ b(7) ^ b(10) ^ b(11) ^ b(13) ^ b(16) ^ b(20) ^ b(21) ^ b(22) ^ b(23)
    e[1] = b(0) ^ b(1) ^ b(3) ^ b(4) ^ b(6) ^ b(8) ^ b(10) ^ b(12) ^ b(14) ^ b(17) ^ b(20) ^ b(21) ^ b(22) ^ b(23)
    e[2] = b(0) ^ b(2) ^ b(3) ^ b(5) ^ b(6) ^ b(9) ^ b(11) ^ b(12) ^ b(15) ^ b(18) ^ b(20) ^ b(21) ^ b(22)
    e[3] = b(1) ^ b(2) ^ b(3) ^ b(7) ^ b(8) ^ b(9) ^ b(13) ^ b(14) ^ b(15) ^ b(19) ^ b(20) ^ b(21) ^ b(23)
    e[4] = b(4) ^ b(5) ^ b(6) ^ b(7) ^ b(8) ^ b(9) ^ b(16) ^ b(17) ^ b(18) ^ b(19) ^ b(20) ^ b(22) ^ b(23)
    e[5] = b(10) ^ b(11) ^ b(12) ^ b(13) ^ b(14) ^ b(15) ^ b(16) ^ b(17) ^ b(18) ^ b(19) ^ b(21) ^ b(22) ^ b(23)
    return sum(e[i] << i for i in range(6))


# ---------------------------------------------------------------------------
# packed-array access helpers
# ---------------------------------------------------------------------------
def _set_serdes(dut, lane0_logical: int, lane1_logical: int) -> None:
    """Force the DUT's internal serdes_byte_sample (pre-transform bytes).

    Receiver applies transform=reverse8 (FIXED_TRANSFORM=1) to obtain the candidate byte,
    so to make the receiver see logical byte X we drive serdes = reverse8(X). Layout of
    ``logic [1:0][7:0] serdes_byte_sample``: [7:0]=lane0, [15:8]=lane1.
    """
    lo = reverse8(lane0_logical)
    hi = reverse8(lane1_logical)
    dut.serdes_byte_sample.value = (hi << 8) | lo


def _trace_slot(sig, idx: int) -> int:
    """Read element ``idx`` (each 8 bits) of a packed ``logic [7:0][7:0]`` output."""
    full = int(sig.value)
    return (full >> (8 * idx)) & 0xFF


# ---------------------------------------------------------------------------
# stimulus tasks (1:1 with the SV tasks)
# ---------------------------------------------------------------------------
async def _reset_dut(dut):
    """SV reset_dut: rst_n=0, LP-11, 8 hs_clk cycles (byte_clk frozen in reset), release,
    settle 12 byte_clk cycles."""
    dut.rst_n.value = 0
    dut.dphy_data_hs_p.value = 0b00          # SV: data_hs_p = 2'b00
    dut.dphy_data_hs_n.value = 0b11          # SV: ~data_hs_p
    dut.dphy_data_lp_p.value = 0b11
    dut.dphy_data_lp_n.value = 0b11
    for _ in range(8):
        await RisingEdge(dut.dphy_hs_clock_clk_p)
    dut.rst_n.value = 1
    for _ in range(12):
        await RisingEdge(dut.byte_clk)


async def _drive_lp_state(dut, lp_p: int, lp_n: int, cycles: int):
    """SV drive_lp_state: set LP lines on negedge byte_clk, hold ``cycles`` byte_clks."""
    await FallingEdge(dut.byte_clk)
    dut.dphy_data_lp_p.value = lp_p
    dut.dphy_data_lp_n.value = lp_n
    for _ in range(cycles):
        await RisingEdge(dut.byte_clk)


async def _drive_byte(dut, lane0_logical: int, lane1_logical: int):
    """SV drive_byte: force serdes_byte_sample = reverse8(logical) at negedge, step 1 posedge.

    The RTL captures serdes_byte_sample at posedge byte_clk *before* its own
    ``serdes_byte_sample <= serdes_byte`` NBA re-zeroes it, so writing on the falling edge
    is phase-correct (identical to the SV negedge blocking write)."""
    await FallingEdge(dut.byte_clk)
    _set_serdes(dut, lane0_logical & 0xFF, lane1_logical & 0xFF)
    await RisingEdge(dut.byte_clk)


async def _wait_for_retrain_done(dut):
    """SV wait_for_retrain_done: poll internal lane_bitslip_phase until (6,6), then
    sweep_hold_count >= 6, then 4 more cycles."""
    timeout = 1000
    while timeout > 0:
        await RisingEdge(dut.byte_clk)
        p0 = int(dut.lane_bitslip_phase.value) & 0x3F
        # packed logic [1:0][2:0]: [2:0]=lane0, [5:3]=lane1
        lane0 = p0 & 0x7
        lane1 = (p0 >> 3) & 0x7
        if lane0 == 6 and lane1 == 6:
            break
        timeout -= 1
    check(timeout > 0, "retrain timeout (bitslip never reached 6/6)")

    timeout = 100
    while timeout > 0:
        await RisingEdge(dut.byte_clk)
        if int(dut.sweep_hold_count.value) >= 6:
            break
        timeout -= 1
    check(timeout > 0, "retrain timeout (sweep_hold_count never reached 6)")

    for _ in range(4):
        await RisingEdge(dut.byte_clk)


async def _emit_one_packet(dut, packet_idx: int, payload_seed: int):
    """SV emit_one_packet: LP->HS entry, SoT + long-packet header + payload bytes, then the
    critical lane-1 slot[2] check. Returns the number of failing check_byte calls (0 = OK)."""
    computed_ecc = ref_ecc6((1280 << 8) | 0x1E)
    ecc_byte = computed_ecc & 0x3F  # {2'b00, computed_ecc}

    dut._log.info(
        f"=== PACKET {packet_idx} (LP->HS, long packet header + payload) "
        f"expected ECC(DI=0x1E,WC=1280)=0x{ecc_byte:02x} ==="
    )

    # LP-11 -> LP-00 (HS entry)
    await _drive_lp_state(dut, 0b11, 0b11, 2)
    await _drive_lp_state(dut, 0b00, 0b00, 4)

    # SoT byte on both lanes, then long-packet header + payload.
    await _drive_byte(dut, 0xB8, 0xB8)                             # slot[0] SoT
    await _drive_byte(dut, 0x1E, 0x00)                             # slot[1] DI / WC[7:0]
    await _drive_byte(dut, 0x05, ecc_byte)                         # slot[2] WC[15:8] / ECC
    await _drive_byte(dut, (payload_seed + 0x00) & 0xFF, (payload_seed + 0x01) & 0xFF)  # slot[3]
    await _drive_byte(dut, (payload_seed + 0x02) & 0xFF, (payload_seed + 0x03) & 0xFF)  # slot[4]
    await _drive_byte(dut, (payload_seed + 0x04) & 0xFF, (payload_seed + 0x05) & 0xFF)  # slot[5]
    await _drive_byte(dut, (payload_seed + 0x06) & 0xFF, (payload_seed + 0x07) & 0xFF)  # slot[6]
    await _drive_byte(dut, (payload_seed + 0x08) & 0xFF, (payload_seed + 0x09) & 0xFF)  # slot[7]

    # Allow trace state to settle.
    for _ in range(4):
        await RisingEdge(dut.byte_clk)
    await Timer(1, unit="ns")

    l1_slot2 = _trace_slot(dut.trace_slot_lane1_aligned, 2)
    l1_slot1 = _trace_slot(dut.trace_slot_lane1_aligned, 1)
    l0_slot1 = _trace_slot(dut.trace_slot_lane0_aligned, 1)
    l0_slot2 = _trace_slot(dut.trace_slot_lane0_aligned, 2)
    dut._log.info(
        f"  packet {packet_idx} trace: lane1 slot[1]=0x{l1_slot1:02x} slot[2]=0x{l1_slot2:02x} "
        f"| lane0 slot[1]=0x{l0_slot1:02x} slot[2]=0x{l0_slot2:02x}"
    )

    fails = 0
    for name, actual, expected in (
        ("lane1 slot[2] aligned", l1_slot2, 0x1E),
        ("lane1 slot[1] aligned", l1_slot1, 0x00),
        ("lane0 slot[1] aligned", l0_slot1, 0x1E),
        ("lane0 slot[2] aligned", l0_slot2, 0x05),
    ):
        if actual != expected:
            dut._log.error(f"  ## FAIL pkt{packet_idx}: {name} actual=0x{actual:02x} expected=0x{expected:02x}")
            fails += 1

    # EoT
    await _drive_lp_state(dut, 0b11, 0b11, 4)
    return fails


@cocotb.test(timeout_time=50, timeout_unit="ms")
async def dphy_lane1_trace(dut):
    dut._log.info("=== tb_dphy_lane1_trace ===")
    dut._log.info("DUT params: FIXED_BITSLIP_PHASE=6/6 FIXED_TRANSFORM=1 (matches deployed)")

    # Clocks (TB: idelay_ref_clk #2.5 -> 5 ns period; hs_clk_p #5 -> 10 ns period).
    cocotb.start_soon(Clock(dut.idelay_ref_clk, 5.0, unit="ns").start())
    cocotb.start_soon(Clock(dut.dphy_hs_clock_clk_p, 10.0, unit="ns").start())

    # Static / config inputs matching the SV DUT instantiation.
    dut.idelay_ref_reset.value = 1                     # = !rst_n at t0
    dut.runtime_idelay_tap.value = 8
    dut.runtime_idelay_tap_lane1.value = 8
    dut.runtime_idelay_tap_clk.value = 0
    dut.rt_bufr_clr.value = 0
    dut.runtime_bitslip_phase.value = 6
    dut.runtime_bitslip_phase_lane1.value = 6
    dut.runtime_lane1_sweep_enable.value = 0
    dut.runtime_expected_long_dt.value = 0x00
    dut.sup_enable.value = 0
    dut.sup_bufr_clr.value = 0
    dut.sup_serdes_rst.value = 0
    dut.sup_hs_settled.value = 0
    dut.hwlock_bufr_clr.value = 0
    dut.cfg_hs_settle_gate.value = 0
    dut.cfg_settle_blank_k.value = 0
    dut.rst_n.value = 0

    # dphy_hs_clock_clk_n is the complement of clk_p. Drive it as a real anti-phase clock
    # (clk_p starts high, so clk_n starts low) -- NOT a 1ps polling loop, which would force
    # the scheduler to step every picosecond and take hours to reach the ms-scale timeout.
    cocotb.start_soon(Clock(dut.dphy_hs_clock_clk_n, 10.0, unit="ns").start(start_high=False))

    # idelay_ref_reset tracks !rst_n; keep it consistent with the reset release.
    async def _drive_idelay_ref_reset():
        while True:
            await RisingEdge(dut.idelay_ref_clk)
            dut.idelay_ref_reset.value = 0 if int(dut.rst_n.value) else 1

    cocotb.start_soon(_drive_idelay_ref_reset())

    await _reset_dut(dut)
    await _wait_for_retrain_done(dut)

    total_fails = 0
    for p in range(5):
        seed = (0x10 + (p * 0x10)) & 0xFF
        total_fails += await _emit_one_packet(dut, p + 1, seed)
        # Force trace_capture state back to ready for the next event.
        for _ in range(40):
            await RisingEdge(dut.byte_clk)

    check(total_fails == 0, f"lane trace mismatches across packets: {total_fails} failures")
    dut._log.info("TEST PASSED: lane 1 (and lane 0) trace correct across all 5 packets")


def test_dphy_lane1_trace():
    from runner_support import build_and_test

    build_and_test(
        block="dphy_lane1_trace",
        sources=[
            # Local copy of the Xilinx-primitive stubs (same models as the DSim TB inline
            # stubs / lib/verilator_unisim_stubs.sv, minus a header line Verilator misreads
            # as a metacomment pragma). Listed FIRST so the DUT binds to these cells.
            "verification/cocotb/dphy_lane1_trace/dphy_lane1_trace_stubs.sv",
            "rtl/prototype/dphy_hs_byte_probe.sv",
        ],
        toplevel="dphy_hs_byte_probe",
        test_module="test_dphy_lane1_trace",
        test_dir=Path(__file__).resolve().parent,
        parameters={
            "LANES": 2,
            "SOT_WINDOW_BYTES": 64,
            "SWEEP_HOLD_BYTES": 8,
            "SWEEP_ENABLE": 0,
            "FIXED_BITSLIP_PHASE": 6,
            "FIXED_BITSLIP_PHASE_LANE1": 6,
            "LANE1_BITSLIP_SWEEP_ENABLE": 0,
            "FIXED_TRANSFORM": 1,
            "TRACE_TRIGGER_MODE": 3,
            "IDELAY_TAP": 8,
            "EXPECTED_LONG_DT": 0x1E,
            "EXPECTED_LONG_WC": 1280,
            "MIN_SYNC_HEADER_SCORE": 13,
            "SYNC_HEADER_SWEEP_BIT_OFFSETS": 0,
            "SYNC_HEADER_USE_ALIGNED_STREAM": 1,
            "STREAM_PAIRING": 0,
        },
        engine="verilator",
    )
