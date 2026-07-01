"""cocotb port of verification/tb/tb_dphy_hwlock_fsm.sv (E2 HW deterministic-lock FSM).

The DSim TB is a single cumulative ``initial`` block that walks the FSM through five
scenarios (T1 lock-on-current-phase, T4 hold+collapse+re-lock, T3 lock-after-one-reroll,
T2 never-lockable->failed, T5 FAILED auto-retry->lock), plus a combinational lock-quality
*model* (the ``hdr_active`` wire) and a small ``always @(posedge clk)`` that advances a
modelled ``/4 phase`` on every ``bufr_clr`` rising edge.

Here:
  * the lock-quality model (``phase`` register + ``hdr_active`` wire) becomes the
    ``LockModel`` coroutine, which recomputes ``hdr_active`` combinationally after every
    posedge (so the DUT samples the value derived from the current registered ``combo`` /
    ``phase`` on the next edge -- identical to the SV ``wire`` semantics).
  * ``wait_locked`` / ``wait_failed`` / ``wait_unlocked`` become the three poll helpers.
  * ``chk`` -> ``check`` from lib.scoreboard.

The five scenarios share the DUT's cumulative state exactly as the TB does, so they are
kept in ONE @cocotb.test() coroutine (the TB is one continuous run; splitting T5 out would
break the "continues from T2's FAILED state" dependency). num_tests = 1.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import start_clock  # noqa: E402
from lib.scoreboard import check  # noqa: E402

# TB localparams (small so the sim stays short) -- also passed as DUT parameters=.
SETTLE_MIN_CYC = 15
SETTLE_CYC = 40
REROLL_CYC = 10
LOST_CYC = 120
RETRY_CYC = 300
MAX_REROLL = 8

S_IDLE = 0
S_HOLD = 3


class LockModel:
    """Replicates the SV lock-quality model:

        wire cur_is_good = ({bitslip_p0,bitslip_p1} == good_combo)
                           && (good_phase==-1 || phase==good_phase);
        wire hdr_active  = cur_is_good && !force_lo;

    plus the registered /4-phase counter that advances on each bufr_clr rising edge:

        always @(posedge clk) begin
            bufr_clr_d <= bufr_clr;
            if (bufr_clr && !bufr_clr_d) phase <= phase + 1;
        end
    """

    def __init__(self, dut):
        self.dut = dut
        self.good_combo = 6  # {p0=0,p1=6}
        self.good_phase = -1  # -1 = any, 99 = never
        self.phase = 0
        self.force_lo = False
        self._bufr_clr_d = 0

    def _recompute(self):
        p0 = int(self.dut.bitslip_p0.value)
        p1 = int(self.dut.bitslip_p1.value)
        combo = (p0 << 3) | p1
        cur_is_good = (combo == self.good_combo) and (
            self.good_phase == -1 or self.phase == self.good_phase
        )
        self.dut.hdr_active.value = 1 if (cur_is_good and not self.force_lo) else 0

    def start(self, clk):
        # Drive an initial hdr_active before the DUT leaves reset.
        self._recompute()
        return cocotb.start_soon(self._run(clk))

    async def _run(self, clk):
        while True:
            await RisingEdge(clk)
            # registered /4-phase advance (edge-detect on bufr_clr), matching the SV
            # always @(posedge clk); reads bufr_clr as sampled at this edge.
            bufr_clr = int(self.dut.bufr_clr.value)
            if bufr_clr and not self._bufr_clr_d:
                self.phase += 1
            self._bufr_clr_d = bufr_clr
            # combinational hdr_active now reflects the just-registered combo/phase; it
            # will be sampled by the DUT's always_ff on the next posedge.
            self._recompute()


async def _reset_pulse(dut, clk, low_cycles, post_cycles):
    dut.rst_n.value = 0
    for _ in range(low_cycles):
        await RisingEdge(clk)
    dut.rst_n.value = 1
    for _ in range(post_cycles):
        await RisingEdge(clk)


async def wait_locked(dut, clk, cycles):
    for _ in range(cycles):
        await RisingEdge(clk)
        if int(dut.locked.value) == 1:
            return True
    return False


async def wait_failed(dut, clk, cycles):
    for _ in range(cycles):
        await RisingEdge(clk)
        if int(dut.failed.value) == 1:
            return True
    return False


async def wait_unlocked(dut, clk, cycles):
    for _ in range(cycles):
        await RisingEdge(clk)
        if int(dut.locked.value) == 0:
            return True
    return False


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def hwlock_fsm(dut):
    clk = dut.clk
    # 200 MHz in the TB (#2.5); the actual period is irrelevant to the cycle-based checks.
    start_clock(clk, period_ns=5.0)

    dut.enable.value = 0
    dut.rst_n.value = 0
    model = LockModel(dut)

    # initial: rst_n=0, enable=0, force_lo=0; 5 clks reset, release, 3 clks settle.
    for _ in range(5):
        await RisingEdge(clk)
    dut.rst_n.value = 1
    # start the lock-quality model only after clocks are running (it edge-detects bufr_clr).
    model.start(clk)
    for _ in range(3):
        await RisingEdge(clk)

    check(
        int(dut.dbg_state.value) == 0
        and int(dut.locked.value) == 0
        and int(dut.failed.value) == 0,
        "after reset: IDLE, !locked, !failed",
    )

    # ================= T1: lock on current phase =================
    dut._log.info("T1: good combo (0,6) any phase -> sweep & lock")
    model.good_combo = 6
    model.good_phase = -1
    model.phase = 0
    model.force_lo = False
    dut.enable.value = 1
    ok = await wait_locked(dut, clk, 8 * SETTLE_CYC + 4 * SETTLE_MIN_CYC + 200)
    check(ok, "T1 locked asserted")
    check(int(dut.dbg_combo.value) == 6, "T1 locked at combo (0,6)")
    check(
        int(dut.bitslip_p0.value) == 0 and int(dut.bitslip_p1.value) == 6,
        "T1 bitslip target = (0,6)",
    )
    check(int(dut.failed.value) == 0, "T1 not failed")
    for _ in range(20):
        await RisingEdge(clk)
    check(
        int(dut.locked.value) == 1 and int(dut.dbg_state.value) == S_HOLD,
        "T1 stays in HOLD/locked",
    )

    # ================= T4: collapse in HOLD -> re-lock =================
    dut._log.info("T4: drop hdr_active > LOST_CYC while held -> re-lock")
    model.force_lo = True  # link collapses
    ok = await wait_unlocked(dut, clk, LOST_CYC + 2 * SETTLE_CYC + 100)
    check(ok and int(dut.locked.value) == 0, "T4 locked dropped after collapse")
    model.force_lo = False  # restore -> should re-lock (good_phase=-1, re-rolls OK)
    ok = await wait_locked(dut, clk, 70 * SETTLE_CYC + REROLL_CYC + 300)
    check(ok, "T4 re-locked after restore")
    dut.enable.value = 0
    for _ in range(4):
        await RisingEdge(clk)
    check(
        int(dut.dbg_state.value) == S_IDLE and int(dut.locked.value) == 0,
        "T4 disable -> IDLE, !locked",
    )

    # ================= T3: lock only after one re-roll =================
    dut._log.info("T3: good combo (2,3) only on phase 1 -> one re-roll then lock")
    model.good_combo = (2 << 3) | 3
    model.good_phase = 1
    model.phase = 0
    model.force_lo = False
    await _reset_pulse(dut, clk, 3, 2)
    dut.enable.value = 1
    ok = await wait_locked(dut, clk, 64 * SETTLE_CYC + REROLL_CYC + 16 * SETTLE_CYC + 400)
    check(ok, "T3 locked after re-roll")
    check(int(dut.dbg_reroll.value) == 1, "T3 exactly one re-roll")
    check(int(dut.dbg_combo.value) == ((2 << 3) | 3), "T3 locked at combo (2,3)")
    dut.enable.value = 0
    for _ in range(4):
        await RisingEdge(clk)

    # ================= T2: never lockable -> failed =================
    dut._log.info("T2: no good combo on any phase -> re-roll MAX then fail")
    model.good_combo = 6
    model.good_phase = 99  # unreachable
    model.phase = 0
    model.force_lo = False
    await _reset_pulse(dut, clk, 3, 2)
    dut.enable.value = 1
    ok = await wait_failed(
        dut, clk, (MAX_REROLL + 1) * (64 * SETTLE_CYC + REROLL_CYC) + 800
    )
    check(ok, "T2 failed asserted")
    check(int(dut.locked.value) == 0, "T2 not locked")
    check(int(dut.dbg_reroll.value) == (MAX_REROLL & 0xF), "T2 reroll count = MAX_REROLL")

    # ============ T5: FAILED auto-retries -> locks when a stream appears ============
    # continues from T2's FAILED state: make a combo good (any phase) -> the FSM must
    # retry the sweep (RETRY_CYC) and lock, clearing `failed`.
    dut._log.info("T5: FAILED retries -> locks once a good combo appears")
    model.good_combo = 0  # (0,0)
    model.good_phase = -1  # lockable on any phase
    ok = await wait_locked(dut, clk, RETRY_CYC + 70 * SETTLE_CYC + 400)
    check(ok, "T5 re-locked out of FAILED after retry")
    check(int(dut.failed.value) == 0, "T5 failed cleared on lock")
    check(int(dut.dbg_state.value) == S_HOLD, "T5 in HOLD")


def test_dphy_hwlock_fsm():
    from runner_support import build_and_test

    build_and_test(
        block="dphy_hwlock_fsm",
        sources=["rtl/mipi_rx/dphy_hwlock_fsm.sv"],
        toplevel="dphy_hwlock_fsm",
        test_module="test_dphy_hwlock_fsm",
        test_dir=Path(__file__).resolve().parent,
        parameters={
            "SETTLE_MIN_CYC": SETTLE_MIN_CYC,
            "SETTLE_CYC": SETTLE_CYC,
            "REROLL_CYC": REROLL_CYC,
            "LOST_CYC": LOST_CYC,
            "RETRY_CYC": RETRY_CYC,
            "MAX_REROLL": MAX_REROLL,
        },
        engine="verilator",
    )
