"""cocotb port of verification/tb/tb_dphy_lane_supervisor.sv (dual-clock D-PHY supervisor).

The DUT is the D-PHY clock/data lane supervisor. It runs entirely on a free-running
``ctl_clk`` (200 MHz IDELAY refclk) and drives a gated ``byte_clk`` domain via ``bufr_clr``.
The DSim TB models the BUFR with a *behavioural* divided clock: ``byte_clk`` toggles only
while ``hs_clk_on && !bufr_clr`` and is held at 0 otherwise. That behavioural BUFR is
replicated here as the ``byte_clk_gen`` coroutine (state carried in a mutable ``Ctx``).

The whole DSim ``initial`` scenario (T1..T9) is a single deterministic waveform sequence,
so it is replicated 1:1 in one ``@cocotb.test()`` with EVERY ``check(...)`` preserved.
Timing checks (``>= 95 ns`` etc.) use ``get_sim_time('ns')`` for the same real-time deltas
the TB measured with ``$realtime``. ``wait(cond)`` becomes an edge-polling loop.

Signal / marker map (TB localparams -> DUT parameters):
  CTL_CLK_HZ=200_000_000, T_INIT_US=1, T_INIT_FORCE_US=3 (T_INIT_US_TB / T_INIT_FORCE_US_TB).
  CTL_PERIOD_NS=5.0 (200 MHz ctl_clk), BYTE_PERIOD_NS=18.5 (~54 MHz gated byte_clk).
FSM state encodings (from the DUT enums):
  clk:  CK_INIT=0 CK_STOP=1 CK_HS_PRPR=2 CK_HS_TERM=3 CK_HS_CLK=4 CK_HS_END=5
  data: DT_INIT=0 DT_WAIT_STOP=1 DT_STOP=2 DT_HS_RQST=3 DT_HS_SETTLE=4 DT_HS_RCV=5
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer
from cocotb.utils import get_sim_time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.scoreboard import check  # noqa: E402

CTL_PERIOD_NS = 5.0     # 200 MHz
BYTE_PERIOD_NS = 18.5   # ~54 MHz

# clk-lane FSM state encodings (dut sts_clk_state)
CK_HS_CLK = 4
# data-lane FSM state encodings (dut sts_data_state)
DT_STOP = 2


class Ctx:
    """Mutable stimulus context (mirrors the TB's shared ``hs_clk_on`` reg)."""

    def __init__(self, dut):
        self.dut = dut
        self.hs_clk_on = False


async def byte_clk_gen(ctx):
    """Behavioural BUFR: byte_clk toggles only while hs_clk_on && !bufr_clr, else held 0.

    1:1 port of the TB ``always`` block that drives ``byte_clk_int``. The half/quarter
    period Timers reproduce the ~54 MHz divided clock and the gate latency exactly.
    """
    dut = ctx.dut
    dut.byte_clk.value = 0
    val = 0
    while True:
        if ctx.hs_clk_on and int(dut.bufr_clr.value) == 0:
            await Timer(BYTE_PERIOD_NS / 2.0, unit="ns")
            val ^= 1
            dut.byte_clk.value = val
        else:
            val = 0
            dut.byte_clk.value = 0
            await Timer(BYTE_PERIOD_NS / 4.0, unit="ns")


async def wait_signal(dut, sig_name, value, clk):
    """SV ``wait(sig == value)`` -> poll on each ctl_clk rising edge until satisfied."""
    while int(getattr(dut, sig_name).value) != value:
        await RisingEdge(clk)


async def clock_lane_hs_entry(dut, ctx):
    """TB clock_lane_hs_entry(): 11 -> 01 (HS-Rqst) -> 00, HS clock starts at 00.

    Checks bufr_clr releases no earlier than 95 ns (T_CLK_SETTLE) after clk LP-00 and
    not unexpectedly late.
    """
    dut.clk_lp.value = 0b11
    await Timer(100, unit="ns")
    dut.clk_lp.value = 0b01
    await Timer(60, unit="ns")
    dut.clk_lp.value = 0b00
    t_lp00 = get_sim_time("ns")
    ctx.hs_clk_on = True
    # wait (bufr_clr == 0)
    while int(dut.bufr_clr.value) != 0:
        await RisingEdge(dut.ctl_clk)
    delta = get_sim_time("ns") - t_lp00
    check(delta >= 95.0,
          f"bufr_clr released {delta:.1f} ns after clk LP-00 (< 95 ns settle)")
    check(delta <= 250.0, "bufr_clr release took unexpectedly long")


async def data_burst_entry(dut, ctx):
    """TB data_burst_entry(): wait DT_STOP, then 11->01->00 HS entry on the data lane.

    Checks hs_settled rises no earlier than 85 ns (T_HS_SETTLE) after data LP-00.
    """
    dut.data_lp.value = 0b11
    # The data-lane FSM sits in DT_INIT for T_INIT after reset and must observe stop
    # (LP-11) before it accepts an HS request.
    await wait_signal(dut, "sts_data_state", DT_STOP, dut.ctl_clk)
    await Timer(20, unit="ns")
    dut.data_lp.value = 0b01
    await Timer(60, unit="ns")
    dut.data_lp.value = 0b00
    t_lp00 = get_sim_time("ns")
    while int(dut.hs_settled_byte.value) != 1:
        await RisingEdge(dut.ctl_clk)
    delta = get_sim_time("ns") - t_lp00
    check(delta >= 85.0,
          f"hs_settled {delta:.1f} ns after data LP-00 (< 85 ns settle)")
    check(delta <= 250.0, "hs_settled took unexpectedly long")


@cocotb.test(timeout_time=500, timeout_unit="us")
async def dphy_lane_supervisor(dut):
    ctx = Ctx(dut)

    # Initial input state (TB: clk_lp = 2'b11, data_lp = 2'b11, ctl_aresetn = 0).
    dut.clk_lp.value = 0b11
    dut.data_lp.value = 0b11
    dut.ctl_aresetn.value = 0
    if hasattr(dut, "cfg_clk_settle_cyc"):
        dut.cfg_clk_settle_cyc.value = 0

    # Free-running 200 MHz ctl_clk (CTL_PERIOD_NS = 5.0).
    cocotb.start_soon(Clock(dut.ctl_clk, CTL_PERIOD_NS, unit="ns").start())
    # Behavioural BUFR / byte_clk generator.
    cocotb.start_soon(byte_clk_gen(ctx))

    # Reset: hold low for 5 ctl_clk edges, then release.
    for _ in range(5):
        await RisingEdge(dut.ctl_clk)
    dut.ctl_aresetn.value = 1

    # --- T1: idle stop state -------------------------------------------------
    await Timer(200, unit="ns")
    check(int(dut.bufr_clr.value) == 1, "bufr_clr must be held while clock lane idle")
    check(int(dut.rx_clk_active_byte.value) == 0, "rx_clk_active must be 0 while idle")
    check(int(dut.hs_settled_byte.value) == 0, "hs_settled must be 0 while idle")

    # --- T2: clock lane HS entry with settle ---------------------------------
    await clock_lane_hs_entry(dut, ctx)
    # byte domain comes alive within a few divided-clock cycles
    await Timer(BYTE_PERIOD_NS * 4, unit="ns")
    check(int(dut.rx_clk_active_byte.value) == 1, "rx_clk_active must rise after restart")
    check(int(dut.serdes_rst_byte.value) == 0, "serdes_rst must release after restart")
    check(int(dut.sts_lock_cnt.value) == 1, "lock_cnt must count first lock")

    # --- T3: data burst settle gating ----------------------------------------
    await data_burst_entry(dut, ctx)
    await Timer(20, unit="ns")  # ctl-domain counter lags the byte-domain settled flag
    check(int(dut.sts_settle_cnt.value) == 1, "settle_cnt must count first settle")
    # burst end clears settled
    dut.data_lp.value = 0b11
    await Timer(100, unit="ns")
    check(int(dut.hs_settled_byte.value) == 0, "hs_settled must clear at data LP-11")

    # --- T4: aborted settle never sets settled --------------------------------
    dut.data_lp.value = 0b01
    await Timer(60, unit="ns")
    dut.data_lp.value = 0b00
    await Timer(40, unit="ns")   # abort 40 ns into the 85 ns settle
    dut.data_lp.value = 0b11
    await Timer(150, unit="ns")
    check(int(dut.hs_settled_byte.value) == 0, "aborted settle must not set hs_settled")
    check(int(dut.sts_settle_cnt.value) == 1, "settle_cnt must not count aborted settle")
    # and a clean burst afterwards still works
    await data_burst_entry(dut, ctx)
    await Timer(20, unit="ns")
    check(int(dut.sts_settle_cnt.value) == 2, "settle_cnt must count clean re-settle")
    dut.data_lp.value = 0b11
    await Timer(60, unit="ns")

    # --- T5: clock gating (vblank) -------------------------------------------
    dut.clk_lp.value = 0b11
    ctx.hs_clk_on = False
    await Timer(100, unit="ns")
    check(int(dut.bufr_clr.value) == 1, "bufr_clr must re-assert on clock LP-11")
    check(int(dut.rx_clk_active_byte.value) == 0, "rx_clk_active must async-clear on gate")
    check(int(dut.serdes_rst_byte.value) == 1, "serdes_rst must async-assert on gate")
    check(int(dut.hs_settled_byte.value) == 0, "hs_settled must clear on clock outage")

    # --- T6: restart lottery is deterministic now ----------------------------
    await clock_lane_hs_entry(dut, ctx)
    await Timer(BYTE_PERIOD_NS * 4, unit="ns")
    check(int(dut.rx_clk_active_byte.value) == 1, "rx_clk_active must rise after re-lock")
    check(int(dut.sts_lock_cnt.value) == 2, "lock_cnt must count re-lock")
    await data_burst_entry(dut, ctx)
    dut.data_lp.value = 0b11

    # --- T7: cold-attach escape (continuous clock, no LP-11 ever) ------------
    dut.clk_lp.value = 0b00        # sensor free-runs HS clock
    ctx.hs_clk_on = True
    dut.ctl_aresetn.value = 0      # FPGA "reconfigured"
    for _ in range(5):
        await RisingEdge(dut.ctl_clk)
    dut.ctl_aresetn.value = 1
    # must lock via T_INIT_FORCE escape: force timeout + clk settle
    while int(dut.bufr_clr.value) != 0:
        await RisingEdge(dut.ctl_clk)
    check(get_sim_time("ns") > 0, "escape reached")
    await Timer(BYTE_PERIOD_NS * 4, unit="ns")
    check(int(dut.rx_clk_active_byte.value) == 1, "rx_clk_active after cold-attach escape")
    # data lane still cycles per burst
    dut.data_lp.value = 0b11
    await Timer(100, unit="ns")
    await data_burst_entry(dut, ctx)

    # --- T8: continuous lock must HOLD; escape must not re-fire while locked -
    # After T7 the supervisor is locked on a continuous clock (clk_lp=00). The
    # generalised escape counts only in waiting states, so a healthy lock must survive a
    # long continuous hold without a spurious re-lock (lock_cnt stays at the T7 value).
    dut.data_lp.value = 0b11
    await Timer(60, unit="ns")
    dut.clk_lp.value = 0b00        # continuous, no clock-lane gating
    ctx.hs_clk_on = True
    await Timer(200, unit="ns")
    lock_snapshot = int(dut.sts_lock_cnt.value)   # count once settled into the lock
    await Timer(3 * 1000 * 2, unit="ns")          # hold > 2x T_INIT_FORCE (T_INIT_FORCE_US_TB)
    check(int(dut.sts_clk_state.value) == CK_HS_CLK,
          "must stay CK_HS_CLK during continuous hold")
    check(int(dut.rx_clk_active_byte.value) == 1,
          "rx_clk_active stays high in continuous lock")
    check(int(dut.sts_lock_cnt.value) == lock_snapshot,
          "escape must not re-fire while already locked")

    # --- T9: data-lane-driven lock (continuous-clock fix) --------------------
    # clk_lp is held at 10 (never 11 for a normal entry, never a stable 00 for the
    # clk_lp escape), so ONLY the data-lane HS path can lock the clock.
    dut.ctl_aresetn.value = 0
    dut.clk_lp.value = 0b10
    dut.data_lp.value = 0b11
    ctx.hs_clk_on = True
    for _ in range(5):
        await RisingEdge(dut.ctl_clk)
    dut.ctl_aresetn.value = 1
    await wait_signal(dut, "sts_data_state", DT_STOP, dut.ctl_clk)  # data lane saw stop
    await Timer(20, unit="ns")
    dut.data_lp.value = 0b01      # HS-request -> DT_HS_RQST
    await Timer(60, unit="ns")
    dut.data_lp.value = 0b00      # HS
    while int(dut.bufr_clr.value) != 0:
        await RisingEdge(dut.ctl_clk)
    check(int(dut.sts_clk_state.value) == CK_HS_CLK,
          "data-lane HS locked clock (CK_HS_CLK) with clk_lp!=00")
    await Timer(BYTE_PERIOD_NS * 4, unit="ns")
    check(int(dut.rx_clk_active_byte.value) == 1,
          "rx_clk_active after data-lane-driven lock")


def test_dphy_lane_supervisor():
    from runner_support import build_and_test

    build_and_test(
        block="dphy_lane_supervisor",
        sources=[
            "rtl/mipi_rx/dphy_cdc_prims.sv",
            "rtl/mipi_rx/dphy_lane_supervisor.sv",
        ],
        toplevel="dphy_lane_supervisor",
        test_module="test_dphy_lane_supervisor",
        test_dir=Path(__file__).resolve().parent,
        parameters={
            "CTL_CLK_HZ": 200_000_000,
            "T_INIT_US": 1,
            "T_INIT_FORCE_US": 3,
        },
        engine="verilator",
    )
