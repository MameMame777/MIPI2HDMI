"""cocotb port of verification/tb/tb_dphy_probe_supervised.sv.

TB-2: dphy_hs_byte_probe + dphy_lane_supervisor integration. Validates the opt-in
supervisor wiring added for the 3% capture fix:
  - supervisor bufr_clr gates the (behavioural) BUFR -> byte_clk
  - clock-lane HS entry releases bufr_clr so byte_clk runs
  - the SoT-accept gate (!sup_enable || sup_hs_settled) blocks a HS-prepare 0xB8
    before HS-SETTLE elapses, then a post-settle 0xB8+header locks
  - sup_lock_cnt / sup_settle_cnt advance
  - clock-lane gating (LP-11) re-asserts bufr_clr / clears rx_clk_active
  - sup_enable=0 keeps legacy behaviour (clean burst locks with no gate)
  - cfg_hs_settle_gate (decoupled gate) and cfg_settle_blank_k paths (T7..T9)

The byte stream is injected onto the probe's internal post-ISERDES register
u_probe.serdes_byte_sample exactly like the DSim TB forced dut.serdes_byte_sample
(the ISERDES sim stub deserialises nothing). serdes_byte_sample is packed
logic [1:0][7:0] -> lane0 = bits[7:0], lane1 = bits[15:8]; written as one 16-bit
deposit on negedge byte_clk, consumed combinationally on the following posedge, then
reverted by the RTL's `serdes_byte_sample <= serdes_byte` NBA (serdes_byte == 0).

The whole DSim `initial` scenario (T1..T9) is one deterministic waveform, so it is
replicated 1:1 in a single @cocotb.test with EVERY check(...) preserved. The TB `#N`
timeouts inside fork/join_any -> $fatal become explicit polling loops that fail the
check; the outer `#2ms` watchdog becomes the test timeout.

FSM state encodings (from dphy_lane_supervisor enums):
  data: DT_INIT=0 DT_WAIT_STOP=1 DT_STOP=2 DT_HS_RQST=3 DT_HS_SETTLE=4 DT_HS_RCV=5
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, Timer

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.scoreboard import check  # noqa: E402

CTL_PERIOD_NS = 5.0    # 200 MHz ctl_clk (TB: always #2.5)
HS_PERIOD_NS = 10.0    # 100 MHz hs_clk_p (TB: always #5.0); byte_clk == hs_clk (BUFR bypass)

DT_STOP = 2            # sts_data_state encoding


class TB:
    """Holds the DUT + the serdes injection helper (packed-array deposit)."""

    def __init__(self, dut):
        self.dut = dut
        self.probe = dut.u_probe

    async def drive_serdes(self, b0: int, b1: int):
        """TB drive_serdes(b0,b1): force the post-ISERDES sample for ONE byte_clk cycle.

        Injected on negedge byte_clk; the probe reads it on the next posedge, then the
        RTL NBA overwrites serdes_byte_sample with serdes_byte (0) -- so the byte lives
        exactly one cycle, matching the DSim procedural-force-vs-NBA race.
        """
        await FallingEdge(self.dut.byte_clk)
        self.probe.serdes_byte_sample.value = ((b1 & 0xFF) << 8) | (b0 & 0xFF)
        await RisingEdge(self.dut.byte_clk)
        await Timer(1, unit="ns")

    async def clock_lane_hs_entry(self):
        """TB clock_lane_hs_entry: clk 11 -> 01 -> 00, wait bufr_clr release (<2000 ns)."""
        dut = self.dut
        dut.clk_lp.value = 0b11
        await Timer(60, unit="ns")
        dut.clk_lp.value = 0b01
        await Timer(60, unit="ns")
        dut.clk_lp.value = 0b00      # HS clock active
        # fork: wait(sup_bufr_clr==0) join_any #2000ns $fatal
        await self._wait_level_or_fatal("sup_bufr_clr", 0, 2000.0,
                                        "bufr_clr never released")

    async def wait_settled(self):
        """TB wait_settled: wait(sup_hs_settled==1) with a 3000 ns fatal timeout."""
        await self._wait_level_or_fatal("sup_hs_settled", 1, 3000.0,
                                        "hs_settled never rose")

    async def wait_data_stop(self, budget_ns: float, msg: str):
        """TB: wait(sup_data_state==DT_STOP) with a fatal timeout."""
        await self._wait_level_or_fatal("sup_data_state", DT_STOP, budget_ns, msg)

    async def _wait_level_or_fatal(self, sig: str, value: int, budget_ns: float, msg: str):
        handle = getattr(self.dut, sig)
        deadline = _now(self.dut) + budget_ns
        while int(handle.value) != value:
            await RisingEdge(self.dut.ctl_clk)
            if _now(self.dut) > deadline:
                check(False, msg)   # TB $fatal -> record a failure and stop waiting
                return

    async def reset_pulse(self):
        """TB: rst_n=0; repeat(10)@(posedge ctl_clk); rst_n=1."""
        self.dut.rst_n.value = 0
        for _ in range(10):
            await RisingEdge(self.dut.ctl_clk)
        self.dut.rst_n.value = 1


def _now(dut) -> float:
    from cocotb.utils import get_sim_time
    return get_sim_time("ns")


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def dphy_probe_supervised(dut):
    tb = TB(dut)

    dut._log.info("tb_dphy_probe_supervised start")
    # Initial input state (TB initial block).
    dut.sup_enable.value = 1
    dut.cfg_hs_settle_gate.value = 0
    dut.cfg_settle_blank_k.value = 0
    dut.clk_lp.value = 0b11
    dut.data_lp_p.value = 0b11
    dut.data_lp_n.value = 0b11
    dut.rst_n.value = 0

    # Free-running clocks (ctl_clk 200 MHz, hs_clk_p 100 MHz). hs_clk_n is derived in RTL.
    cocotb.start_soon(Clock(dut.ctl_clk, CTL_PERIOD_NS, unit="ns").start())
    cocotb.start_soon(Clock(dut.hs_clk_p, HS_PERIOD_NS, unit="ns").start())

    for _ in range(10):
        await RisingEdge(dut.ctl_clk)
    dut.rst_n.value = 1

    # --- T1: clock-lane HS entry releases bufr_clr, byte_clk runs --------------
    check(int(dut.sup_bufr_clr.value) == 1, "bufr_clr held before clock-lane entry")
    await tb.clock_lane_hs_entry()
    for _ in range(4):
        await RisingEdge(dut.byte_clk)   # proves byte_clk is now toggling
    check(int(dut.sup_bufr_clr.value) == 0, "bufr_clr released after clock-lane HS entry")
    check(int(dut.sup_rx_clk_active.value) == 1, "rx_clk_active set after byte_clk restart")

    # Data FSM must leave DT_INIT (T_INIT) and observe stop (LP-11).
    await tb.wait_data_stop(4000.0, "data FSM never reached DT_STOP")

    # --- T2: data-lane HS entry; pre-settle SoT must be gated ------------------
    await FallingEdge(dut.byte_clk)
    dut.data_lp_p.value = 0b00
    dut.data_lp_n.value = 0b11        # LP-01 HS request
    await Timer(40, unit="ns")
    await FallingEdge(dut.byte_clk)
    dut.data_lp_p.value = 0b00
    dut.data_lp_n.value = 0b00        # LP-00 HS
    check(int(dut.sup_hs_settled.value) == 0, "hs_settled still low right after data LP-00")
    # Push a SoT 0xB8 BEFORE settle completes; the gate must block the lock.
    await tb.drive_serdes(0xB8, 0xB8)
    await tb.drive_serdes(0xB8, 0xB8)
    check(int(dut.lane_sot_seen.value) == 0b00, "pre-settle SoT is gated (no lock)")
    check(int(dut.sync_header_valid.value) == 0, "no sync header from pre-settle SoT")
    # Flush serdes to a non-SoT byte so the held 0xB8 cannot lock at settle.
    await tb.drive_serdes(0x00, 0x00)

    # --- T3: after settle, a fresh SoT+header locks ----------------------------
    await tb.wait_settled()
    check(int(dut.sup_hs_settled.value) == 1, "hs_settled rose after T_HS_SETTLE")
    # header: lane0 = b8,1e,05,11,33 ... lane1 = b8,00,1e,22,44 (ECC for DI=0x1e WC=0x0500)
    await tb.drive_serdes(0xB8, 0xB8)
    await tb.drive_serdes(0x1E, 0x00)
    await tb.drive_serdes(0x05, 0x1E)
    await tb.drive_serdes(0x11, 0x22)
    await tb.drive_serdes(0x33, 0x44)
    for _ in range(40):
        if int(dut.sync_header_valid.value):
            break
        await tb.drive_serdes(0x00, 0x00)
    check(int(dut.lane_sot_seen.value) & 0b01 == 0b01, "post-settle SoT locked lane0")
    check(int(dut.sync_header_valid.value) == 1, "sync header captured after settle")
    check(int(dut.sync_header_di.value) == 0x1E, "sync header DI = 0x1e")

    # --- T4: supervisor counters advanced --------------------------------------
    check(int(dut.sup_lock_cnt.value) >= 1, "lock_cnt counted clock-lane lock")
    check(int(dut.sup_settle_cnt.value) >= 1, "settle_cnt counted data-lane settle")

    # --- T5: clock-lane gating re-asserts bufr_clr -----------------------------
    dut.clk_lp.value = 0b11           # vblank: clock lane back to LP-11
    await Timer(200, unit="ns")
    check(int(dut.sup_bufr_clr.value) == 1, "bufr_clr re-asserts on clock LP-11")
    check(int(dut.sup_rx_clk_active.value) == 0, "rx_clk_active async-clears on gate")
    check(int(dut.sup_hs_settled.value) == 0, "hs_settled clears on clock outage")

    # --- T6: legacy (sup_enable=0) clean burst still locks ---------------------
    dut.sup_enable.value = 0
    dut.data_lp_p.value = 0b11
    dut.data_lp_n.value = 0b11
    await tb.clock_lane_hs_entry()    # byte_clk runs again (bufr_clr ignored anyway)
    for _ in range(4):
        await RisingEdge(dut.byte_clk)
    # data LP edge opens the window the legacy way
    await FallingEdge(dut.byte_clk)
    dut.data_lp_p.value = 0b00
    dut.data_lp_n.value = 0b00
    await tb.drive_serdes(0xB8, 0xB8)
    await tb.drive_serdes(0x1E, 0x00)
    await tb.drive_serdes(0x05, 0x1E)
    await tb.drive_serdes(0x11, 0x22)
    await tb.drive_serdes(0x33, 0x44)
    for _ in range(40):
        if int(dut.sync_header_valid.value):
            break
        await tb.drive_serdes(0x00, 0x00)
    check(int(dut.sync_header_valid.value) == 1,
          "legacy mode (sup_enable=0) still captures header")

    # --- T7: legacy + cfg_hs_settle_gate=1 gates pre-settle SoT ----------------
    dut.sup_enable.value = 0
    dut.cfg_hs_settle_gate.value = 1
    dut.clk_lp.value = 0b11
    dut.data_lp_p.value = 0b11
    dut.data_lp_n.value = 0b11
    await tb.reset_pulse()
    await tb.clock_lane_hs_entry()
    for _ in range(4):
        await RisingEdge(dut.byte_clk)
    await tb.wait_data_stop(4000.0, "T7 data FSM never reached DT_STOP")
    await FallingEdge(dut.byte_clk)
    dut.data_lp_p.value = 0b00
    dut.data_lp_n.value = 0b11        # LP-01 HS request
    await Timer(40, unit="ns")
    await FallingEdge(dut.byte_clk)
    dut.data_lp_p.value = 0b00
    dut.data_lp_n.value = 0b00        # LP-00 HS
    check(int(dut.sup_hs_settled.value) == 0, "T7 hs_settled low right after data LP-00")
    await tb.drive_serdes(0xB8, 0xB8)
    await tb.drive_serdes(0xB8, 0xB8)
    check(int(dut.lane_sot_seen.value) == 0b00,
          "T7 cfg_hs_settle_gate blocks pre-settle SoT (sup_enable=0)")
    check(int(dut.sync_header_valid.value) == 0, "T7 no sync header from pre-settle SoT")
    await tb.drive_serdes(0x00, 0x00)
    await tb.wait_settled()
    check(int(dut.sup_hs_settled.value) == 1, "T7 hs_settled rose after T_HS_SETTLE")
    await tb.drive_serdes(0xB8, 0xB8)
    await tb.drive_serdes(0x1E, 0x00)
    await tb.drive_serdes(0x05, 0x1E)
    await tb.drive_serdes(0x11, 0x22)
    await tb.drive_serdes(0x33, 0x44)
    for _ in range(40):
        if int(dut.sync_header_valid.value):
            break
        await tb.drive_serdes(0x00, 0x00)
    check(int(dut.lane_sot_seen.value) & 0b01 == 0b01, "T7 post-settle SoT locked lane0")
    check(int(dut.sync_header_valid.value) == 1,
          "T7 sync header captured after settle (gated path)")

    # --- T8: cfg_settle_blank_k delays the SoT window K byte_clk after LP-exit --
    dut.sup_enable.value = 0
    dut.cfg_hs_settle_gate.value = 0
    dut.cfg_settle_blank_k.value = 4
    dut.clk_lp.value = 0b11
    dut.data_lp_p.value = 0b11
    dut.data_lp_n.value = 0b11
    await tb.reset_pulse()
    await tb.clock_lane_hs_entry()
    for _ in range(4):
        await RisingEdge(dut.byte_clk)
    await FallingEdge(dut.byte_clk)
    dut.data_lp_p.value = 0b00
    dut.data_lp_n.value = 0b00        # LP-exit -> blank starts
    for _ in range(10):
        await tb.drive_serdes(0x00, 0x00)   # clear LP-sync latency + the 4-cycle blank
    await tb.drive_serdes(0xB8, 0xB8)        # real SoT after the window opens
    await tb.drive_serdes(0x1E, 0x00)
    await tb.drive_serdes(0x05, 0x1E)
    await tb.drive_serdes(0x11, 0x22)
    await tb.drive_serdes(0x33, 0x44)
    for _ in range(40):
        if int(dut.sync_header_valid.value):
            break
        await tb.drive_serdes(0x00, 0x00)
    check(int(dut.sync_header_valid.value) == 1, "T8 blank=4: a normal burst still locks")
    check(int(dut.dbg_burst_count.value) != 0, "T8 burst_count advanced (LP-exit edges)")
    check(int(dut.dbg_sot_burst_count.value) != 0, "T8 sot_burst_count advanced (SoT in window)")

    # --- T9: sup_enable=1 + cfg_settle_blank_k>0 -> sup HS-SETTLE gate DECOUPLED -
    dut.sup_enable.value = 1
    dut.cfg_hs_settle_gate.value = 0
    dut.cfg_settle_blank_k.value = 4
    dut.clk_lp.value = 0b11
    dut.data_lp_p.value = 0b11
    dut.data_lp_n.value = 0b11
    await tb.reset_pulse()
    await tb.clock_lane_hs_entry()
    for _ in range(4):
        await RisingEdge(dut.byte_clk)
    await tb.wait_data_stop(4000.0, "T9 data FSM never reached DT_STOP")
    await FallingEdge(dut.byte_clk)
    dut.data_lp_p.value = 0b00
    dut.data_lp_n.value = 0b11        # LP-01 HS request
    await Timer(40, unit="ns")
    await FallingEdge(dut.byte_clk)
    dut.data_lp_p.value = 0b00
    dut.data_lp_n.value = 0b00        # LP-00 HS
    await tb.wait_settled()           # sup hs_settled rises (mgmt still active)
    for _ in range(10):
        await tb.drive_serdes(0x00, 0x00)   # clear the blank with non-SoT
    await tb.drive_serdes(0xB8, 0xB8)
    await tb.drive_serdes(0x1E, 0x00)
    await tb.drive_serdes(0x05, 0x1E)
    await tb.drive_serdes(0x11, 0x22)
    await tb.drive_serdes(0x33, 0x44)
    for _ in range(40):
        if int(dut.sync_header_valid.value):
            break
        await tb.drive_serdes(0x00, 0x00)
    check(int(dut.sync_header_valid.value) == 1,
          "T9 sup+settle-blank (decoupled sup gate) still locks")

    dut._log.info("TEST PASSED")


def test_dphy_probe_supervised():
    from runner_support import build_and_test

    build_and_test(
        block="dphy_probe_supervised",
        sources=[
            "verification/cocotb/lib/verilator_unisim_stubs.sv",
            "rtl/mipi_rx/dphy_cdc_prims.sv",
            "rtl/mipi_rx/dphy_lane_supervisor.sv",
            "rtl/prototype/dphy_hs_byte_probe.sv",
            "verification/cocotb/dphy_probe_supervised/dphy_probe_supervised_harness.sv",
        ],
        toplevel="dphy_probe_supervised_harness",
        test_module="test_dphy_probe_supervised",
        test_dir=Path(__file__).resolve().parent,
        engine="verilator",
    )
