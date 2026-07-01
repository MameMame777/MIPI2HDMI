"""cocotb port of verification/tb/tb_dphy_hs_byte_probe_per_lane_idelay.sv.

The DSim TB verifies the D-PHY HS byte probe's per-lane IDELAY independence (D),
the CSI-2-spec-derived live_trace slot bytes (A/B), the per-slot rotation-lock
invariant (C), and the sync-header decode (E). It does this by *bypassing* the
unsimulatable bit-level ISERDES path: it drives the DUT's internal
``serdes_byte_sample`` register directly (one byte per byte_clk), exactly as the
DSim tb_dphy_hs_byte_probe_gearbox approach does. The Xilinx primitives
(IBUFDS/BUFIO/BUFR/IDELAYCTRL/IDELAYE2/ISERDESE2) are pass-through/assign stubs in
both the DSim TB and lib/verilator_unisim_stubs.sv, so no real deserialization is
exercised -- only the byte-sample decode / trace / sync FSM.

Because the ISERDES is stubbed to Q=0, ``serdes_byte`` is 0 and the DUT would
overwrite ``serdes_byte_sample <= serdes_byte`` (=0) every byte_clk; the DSim TB
therefore *re-deposits* the intended byte at each negedge just before the posedge
FSM read. We reproduce that exact cadence: deposit on FallingEdge(byte_clk), let
the posedge sample it. cocotb's Verilator backend runs with --public-flat-rw, so
the internal register ``serdes_byte_sample`` and the sync signals (D check) are
readable and writable from Python.

Notes on clocks: byte_clk is a DUT *output* driven combinationally from
dphy_hs_clock_clk_p through the pass-through IBUFDS + BUFR stubs (BUFR is a
plain pass-through -- it does NOT /4 in the stub), gated low by BUFR.CLR while
rst_n=0. So we drive hs_clk_p as the master (10 ns, matching #5) and idelay_ref_clk
(5 ns, matching #2.5); byte_clk then tracks hs_clk_p with zero delay once out of
reset. All waits key off dut.byte_clk except during reset (where byte_clk is held
low), which keys off dut.dphy_hs_clock_clk_p -- mirroring the TB's reset_dut.

Single @cocotb.test replicates the TB's single ``initial`` scenario 1:1, with the
#5_000_000 ns (5 ms) watchdog mapped to the test timeout.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, Timer

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.scoreboard import check  # noqa: E402


SOT_SYNC = 0xB8


def _ref_ecc6_py(data: int) -> int:
    """Independent CSI-2 ECC6 reference (bit-for-bit identical to the TB's
    ref_ecc6_py, which is logically identical to the Python calc_ecc6). data is
    the 24-bit {WC[15:8], WC[7:0], DI}."""
    def b(i: int) -> int:
        return (data >> i) & 1

    e = [0] * 6
    e[0] = b(0) ^ b(1) ^ b(2) ^ b(4) ^ b(5) ^ b(7) ^ b(10) ^ b(11) ^ b(13) ^ b(16) ^ b(20) ^ b(21) ^ b(22) ^ b(23)
    e[1] = b(0) ^ b(1) ^ b(3) ^ b(4) ^ b(6) ^ b(8) ^ b(10) ^ b(12) ^ b(14) ^ b(17) ^ b(20) ^ b(21) ^ b(22) ^ b(23)
    e[2] = b(0) ^ b(2) ^ b(3) ^ b(5) ^ b(6) ^ b(9) ^ b(11) ^ b(12) ^ b(15) ^ b(18) ^ b(20) ^ b(21) ^ b(22)
    e[3] = b(1) ^ b(2) ^ b(3) ^ b(7) ^ b(8) ^ b(9) ^ b(13) ^ b(14) ^ b(15) ^ b(19) ^ b(20) ^ b(21) ^ b(23)
    e[4] = b(4) ^ b(5) ^ b(6) ^ b(7) ^ b(8) ^ b(9) ^ b(16) ^ b(17) ^ b(18) ^ b(19) ^ b(20) ^ b(22) ^ b(23)
    e[5] = b(10) ^ b(11) ^ b(12) ^ b(13) ^ b(14) ^ b(15) ^ b(16) ^ b(17) ^ b(18) ^ b(19) ^ b(21) ^ b(22) ^ b(23)
    return sum(e[i] << i for i in range(6))


def _slot_byte(dut, sig_name: str, idx: int) -> int:
    """Read element [idx] of a packed [7:0][7:0] slot output as an int.

    Verilator flattens ``logic [7:0][7:0] x`` to a 64-bit signal; cocotb may expose
    it either as an indexable array handle or as a flat value. Handle both."""
    sig = getattr(dut, sig_name)
    try:
        return int(sig[idx].value)
    except (TypeError, IndexError):
        full = int(sig.value)
        return (full >> (idx * 8)) & 0xFF


def _slot_rot(dut, sig_name: str, idx: int) -> int:
    """Read element [idx] of a packed [7:0][2:0] rotation output as an int."""
    sig = getattr(dut, sig_name)
    try:
        return int(sig[idx].value)
    except (TypeError, IndexError):
        full = int(sig.value)
        return (full >> (idx * 3)) & 0x7


def _init_inputs(dut):
    """Drive every DUT input to a defined value. As the toplevel, port default
    expressions do NOT apply, so we set them explicitly to the values the DSim TB
    used (connected ports) or to the module-header defaults (unconnected ports)."""
    dut.rst_n.value = 0
    dut.idelay_ref_reset.value = 1            # = !rst_n
    dut.runtime_idelay_tap.value = 0
    dut.runtime_idelay_tap_lane1.value = 0
    dut.runtime_idelay_tap_clk.value = 0      # header default 5'd0
    dut.rt_bufr_clr.value = 0                 # header default 1'b0
    dut.runtime_bitslip_phase.value = 0
    dut.runtime_bitslip_phase_lane1.value = 0
    dut.runtime_lane1_sweep_enable.value = 0  # no header default; TB left it disabled
    dut.runtime_expected_long_dt.value = 0
    dut.sup_enable.value = 0
    dut.sup_bufr_clr.value = 0
    dut.sup_serdes_rst.value = 0
    dut.sup_hs_settled.value = 0
    dut.hwlock_bufr_clr.value = 0             # header default 1'b0
    dut.cfg_hs_settle_gate.value = 0          # header default 1'b0
    dut.cfg_settle_blank_k.value = 0          # header default 4'd0
    dut.dphy_hs_clock_clk_n.value = 1         # ~hs_clk_p (p starts 0)
    dut.dphy_data_hs_p.value = 0
    dut.dphy_data_hs_n.value = 0b11           # ~data_hs_p
    dut.dphy_data_lp_p.value = 0b11           # LP-11 idle (reset_dut)
    dut.dphy_data_lp_n.value = 0b11


async def _reset_dut(dut):
    """Mirror the TB reset_dut task. During reset byte_clk is gated low by BUFR.CLR,
    so wait on hs_clk_p (like the TB waits @(posedge hs_clk_p)); after release wait
    12 byte_clk (like @(posedge byte_clk))."""
    dut.rst_n.value = 0
    dut.idelay_ref_reset.value = 1
    dut.runtime_idelay_tap.value = 0
    dut.runtime_idelay_tap_lane1.value = 0
    dut.dphy_data_hs_p.value = 0
    dut.dphy_data_lp_p.value = 0b11
    dut.dphy_data_lp_n.value = 0b11
    for _ in range(8):
        await RisingEdge(dut.dphy_hs_clock_clk_p)
    dut.rst_n.value = 1
    dut.idelay_ref_reset.value = 0
    for _ in range(12):
        await RisingEdge(dut.byte_clk)


async def _drive_lp_state(dut, lp_p: int, lp_n: int, cycles: int):
    """Mirror drive_lp_state: settle LP on negedge byte_clk, hold for `cycles`."""
    await FallingEdge(dut.byte_clk)
    dut.dphy_data_lp_p.value = lp_p
    dut.dphy_data_lp_n.value = lp_n
    for _ in range(cycles):
        await RisingEdge(dut.byte_clk)
    await Timer(1, unit="ns")


async def _drive_serdes_sample(dut, lane0: int, lane1: int):
    """Mirror drive_serdes_sample: deposit the internal serdes_byte_sample register
    on negedge byte_clk (so the posedge FSM samples it), one byte per byte_clk.

    Verilator flattens ``logic [1:0][7:0]`` to a 16-bit signal: lane0 = [7:0],
    lane1 = [15:8]. Depositing the whole 16-bit value reproduces the TB's two
    per-lane element writes."""
    await FallingEdge(dut.byte_clk)
    dut.serdes_byte_sample.value = ((lane1 & 0xFF) << 8) | (lane0 & 0xFF)
    await RisingEdge(dut.byte_clk)
    await Timer(1, unit="ns")


@cocotb.test(timeout_time=6, timeout_unit="ms")
async def per_lane_idelay(dut):
    # --- clocks: hs_clk_p 10 ns (#5), idelay_ref_clk 5 ns (#2.5) ---
    cocotb.start_soon(Clock(dut.dphy_hs_clock_clk_p, 10.0, unit="ns").start())
    cocotb.start_soon(Clock(dut.idelay_ref_clk, 5.0, unit="ns").start())
    # Keep the differential mates driven (they don't affect the stubbed path but
    # avoid X on the IBUFDS inputs).
    cocotb.start_soon(_drive_hs_clk_n(dut))

    _init_inputs(dut)

    # === Setup / reset ===
    await _reset_dut(dut)

    # === D: per-lane IDELAY independence ===
    dut.runtime_idelay_tap.value = 5
    dut.runtime_idelay_tap_lane1.value = 17
    for _ in range(8):
        await RisingEdge(dut.byte_clk)
    await Timer(1, unit="ns")

    sync2_l0 = int(dut.runtime_idelay_sync2.value)
    sync2_l1 = int(dut.runtime_idelay_lane1_sync2.value)
    check((sync2_l0 & 0x7) == 5, f"D.lane0.sync2[2:0]=0x{sync2_l0 & 0x7:x} expected 5")
    check(sync2_l0 == 5, f"D.lane0.sync2={sync2_l0} expected 5")
    check(sync2_l1 == 17, f"D.lane1.sync2={sync2_l1} expected 17")

    # === Stimulus: single CSI-2 long-packet header + counter payload ===
    # 1) Open the SoT window via LP-11 -> LP-00 on both lanes.
    await _drive_lp_state(dut, 0b00, 0b00, 4)

    # 2) Drive serdes_byte_sample one byte_clk per stream byte (de-interleaved:
    #    lane0=even, lane1=odd). Slot 0 = SoT (0xB8 on both lanes).
    stimulus = [
        (0xB8, 0xB8),  # slot 0
        (0x1E, 0x00),  # slot 1: DI / WC[7:0]
        (0x05, 0x1E),  # slot 2: WC[15:8] / ECC
        (0x10, 0x11),  # slot 3
        (0x12, 0x13),  # slot 4
        (0x14, 0x15),  # slot 5
        (0x16, 0x17),  # slot 6
        (0x18, 0x19),  # slot 7
    ]
    for lane0, lane1 in stimulus:
        await _drive_serdes_sample(dut, lane0, lane1)

    # Allow the trace to settle and the sync-header scan to run (~600 cycles).
    for _ in range(600):
        await RisingEdge(dut.byte_clk)
        if int(dut.sync_header_valid.value) == 1:
            break
    await Timer(1, unit="ns")

    # === A: lane 0 live_trace expectations ===
    expect_l0 = [0xB8, 0x1E, 0x05, 0x10, 0x12, 0x14, 0x16, 0x18]
    for k, exp in enumerate(expect_l0):
        got = _slot_byte(dut, "live_trace_slot_lane0_aligned", k)
        check(got == exp, f"A.lane0.slot[{k}]=0x{got:02x} expected 0x{exp:02x}")

    # === B: lane 1 live_trace expectations ===
    expect_l1 = [0xB8, 0x00, 0x1E, 0x11, 0x13, 0x15, 0x17, 0x19]
    # ECC sanity: ref_ecc6_py({WC[15:8],WC[7:0],DI}=0x05001E) must be 0x1E.
    ecc_ref = _ref_ecc6_py(0x05001E)
    check(ecc_ref == 0x1E, f"B.ref_ecc6 sanity=0x{ecc_ref:02x} expected 0x1E")
    for k, exp in enumerate(expect_l1):
        got = _slot_byte(dut, "live_trace_slot_lane1_aligned", k)
        check(got == exp, f"B.lane1.slot[{k}]=0x{got:02x} expected 0x{exp:02x}")

    # === C: per-slot rotation should equal the SoT-detected rotation (0) ===
    for k in range(8):
        r0 = _slot_rot(dut, "live_trace_slot_lane0_rotation", k)
        r1 = _slot_rot(dut, "live_trace_slot_lane1_rotation", k)
        check(r0 == 0, f"C.lane0.rotation[{k}]={r0} expected 0")
        check(r1 == 0, f"C.lane1.rotation[{k}]={r1} expected 0")

    # === E: sync header decode ===
    sh_di = int(dut.sync_header_di.value)
    sh_wc = int(dut.sync_header_wc.value)
    sh_ecc = int(dut.sync_header_ecc.value)
    sh_score = int(dut.sync_header_score.value)
    check(sh_di == 0x1E, f"E.sync_header_di=0x{sh_di:02x} expected 0x1E")
    check(sh_wc == 1280, f"E.sync_header_wc={sh_wc} expected 1280")
    check(sh_ecc == 0x1E, f"E.sync_header_ecc=0x{sh_ecc:02x} expected 0x1E")
    check(sh_score >= 13, f"E.sync_header_score={sh_score} expected >=13")


async def _drive_hs_clk_n(dut):
    """dphy_hs_clock_clk_n = ~dphy_hs_clock_clk_p (the TB's assign)."""
    while True:
        await RisingEdge(dut.dphy_hs_clock_clk_p)
        dut.dphy_hs_clock_clk_n.value = 0
        await FallingEdge(dut.dphy_hs_clock_clk_p)
        dut.dphy_hs_clock_clk_n.value = 1


def test_dphy_hs_byte_probe_per_lane_idelay():
    from runner_support import build_and_test

    build_and_test(
        block="dphy_hs_byte_probe_per_lane_idelay",
        sources=[
            # Local unisim stubs FIRST. lib/verilator_unisim_stubs.sv trips a
            # Verilator BADVLTPRAGMA on its "// Verilator-compatible" banner when the
            # DUT is built with --public-flat-rw (required here so cocotb can write the
            # DUT-internal serdes_byte_sample register). Same connectivity-only stubs.
            str(Path(__file__).resolve().parent / "dphy_hs_byte_probe_per_lane_idelay_stubs.sv"),
            "rtl/prototype/dphy_hs_byte_probe.sv",
        ],
        toplevel="dphy_hs_byte_probe",
        test_module="test_dphy_hs_byte_probe_per_lane_idelay",
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
            "IDELAY_REFCLK_MHZ": 200.0,
            "STREAM_PAIRING": 0,
        },
        engine="verilator",
    )
