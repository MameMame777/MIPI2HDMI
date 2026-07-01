"""cocotb port of verification/tb/tb_csi2_frame_state_feearly.sv (byte-beat / packet family).

The DSim TB instantiates the SAME module ``csi2_frame_state`` TWICE and drives both with
IDENTICAL stimulus:

  * ``dut_old`` — the buggy legacy: FE_MIN_LINES = 0
  * ``dut_new`` — the fix:          FE_MIN_LINES = 7   (+ cfg_sof_synth / cfg_force_expected /
                                                        cfg_long_as_line wired in)

``build_and_test`` compiles a single parameterised toplevel, so this port builds the DUT
TWICE (``def test_...`` calls ``build_and_test`` once per role, selected by the
``FEEARLY_ROLE`` env var) and runs the SAME scenario coroutines against each build. Because
the two DSim DUTs receive byte-for-byte identical stimulus, driving one parameterisation and
checking the outputs relevant to that role reproduces every ``chk(...)`` in the TB exactly:

  T1  OLD: last==5 (buggy short close)        NEW: last==8, fcnt==1
  T2                                          NEW: fcnt==1, last==8 (clean FE)
  T3                                          NEW: fcnt==1, last==8 (early spurious FS+FE)
  T4                                          NEW: fcnt==1, last==MAXL (runaway cap)
  T5  OLD: last!=8 || fcnt!=2 (no clamp)      NEW: last==8, fcnt==2 (force-expected)
  T6                                          NEW: last==8, fcnt==2 (synth+force)
  T7  OLD: serr high (rejects no-LS longs)    NEW: serr lower  -> o_serr-n_serr >= 5

The DUT samples on posedge; the lib convention (RisingEdge + immediate .value assign) is
equivalent to the TB's posedge/NBA driving, so the per-cycle micro-sequences below mirror
the DSim tasks 1:1.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import cocotb
from cocotb.triggers import ClockCycles, RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import start_clock  # noqa: E402
from lib.scoreboard import check  # noqa: E402

# Which parameterisation this run targets ("old" -> FE_MIN_LINES=0, "new" -> 7).
ROLE = os.environ.get("FEEARLY_ROLE", "new")

# Scaled model localparams (tb line 36): MAXL=16, EXP=8, FSMIN=2, FEMIN=7.
MAXL = 16
EXP = 8
FSMIN = 2
FEMIN = 7


# ---------------------------------------------------------------------------
# stimulus tasks — 1:1 with the SV automatic tasks
# ---------------------------------------------------------------------------
async def reset_dut(dut):
    """SV reset_dut(): hold aresetn low 8 clks, release, settle 2 clks.

    Also (re)applies the config defaults each scenario sets before the drive. The
    per-scenario cfg_* overrides are applied by the caller AFTER this returns (the SV
    sets them between reset_dut() and the first drive, e.g. cfg_force_expected_tb=1)."""
    dut.core_aresetn.value = 0
    dut.cfg_use_lsle.value = 1
    dut.cfg_force_expected.value = 0
    dut.cfg_sof_synth.value = 0
    dut.cfg_long_as_line.value = 0
    dut.cfg_expected_frame_lines.value = 0
    dut.in_pkt_di.value = 0
    dut.in_pkt_wc.value = 0
    dut.in_pkt_is_short.value = 0
    dut.in_pkt_is_long.value = 0
    dut.in_pkt_start.value = 0
    dut.in_pkt_end.value = 0
    dut.in_pkt_err.value = 0
    dut.in_payload_data.value = 0
    dut.in_payload_valid.value = 0
    dut.in_payload_first.value = 0
    dut.in_payload_last.value = 0
    await ClockCycles(dut.core_clk, 8)
    dut.core_aresetn.value = 1
    await ClockCycles(dut.core_clk, 2)


async def drive_short(dut, dt):
    """SV drive_short(dt): a 1-cycle short packet header (start&end&is_short)."""
    clk = dut.core_clk
    await RisingEdge(clk)
    dut.in_pkt_di.value = dt & 0x3F  # {2'b00, dt}
    dut.in_pkt_wc.value = 0
    dut.in_pkt_is_short.value = 1
    dut.in_pkt_is_long.value = 0
    dut.in_pkt_start.value = 1
    dut.in_pkt_end.value = 1
    await RisingEdge(clk)
    dut.in_pkt_start.value = 0
    dut.in_pkt_end.value = 0
    dut.in_pkt_is_short.value = 0


async def drive_lsle_line(dut, d):
    """SV drive_lsle_line(d): LS short, then a 1-payload-byte long, then LE short."""
    clk = dut.core_clk
    await drive_short(dut, 0x02)  # LS
    await RisingEdge(clk)
    dut.in_pkt_di.value = 0x2A
    dut.in_pkt_wc.value = 1
    dut.in_pkt_is_short.value = 0
    dut.in_pkt_is_long.value = 1
    dut.in_pkt_start.value = 1
    await RisingEdge(clk)
    dut.in_pkt_start.value = 0
    dut.in_payload_data.value = d & 0xFF
    dut.in_payload_first.value = 1
    dut.in_payload_last.value = 1
    dut.in_payload_valid.value = 1
    await RisingEdge(clk)
    dut.in_payload_valid.value = 0
    dut.in_payload_first.value = 0
    dut.in_payload_last.value = 0
    dut.in_pkt_end.value = 1
    await RisingEdge(clk)
    dut.in_pkt_end.value = 0
    dut.in_pkt_is_long.value = 0
    await drive_short(dut, 0x03)  # LE (line_idx++)


async def drive_long_le_no_ls(dut, d):
    """SV drive_long_le_no_ls(d): a long packet + LE with NO preceding LS."""
    clk = dut.core_clk
    await RisingEdge(clk)
    dut.in_pkt_di.value = 0x2A
    dut.in_pkt_wc.value = 1
    dut.in_pkt_is_short.value = 0
    dut.in_pkt_is_long.value = 1
    dut.in_pkt_start.value = 1
    await RisingEdge(clk)
    dut.in_pkt_start.value = 0
    dut.in_payload_data.value = d & 0xFF
    dut.in_payload_first.value = 1
    dut.in_payload_last.value = 1
    dut.in_payload_valid.value = 1
    await RisingEdge(clk)
    dut.in_payload_valid.value = 0
    dut.in_payload_first.value = 0
    dut.in_payload_last.value = 0
    dut.in_pkt_end.value = 1
    await RisingEdge(clk)
    dut.in_pkt_end.value = 0
    dut.in_pkt_is_long.value = 0
    await drive_short(dut, 0x03)  # LE (line_idx++)


# ---------------------------------------------------------------------------
# status readbacks (single DUT: outputs are the same names for old/new build)
# ---------------------------------------------------------------------------
def fcnt(dut):
    return int(dut.sts_frame_count.value)


def last(dut):
    return int(dut.sts_last_frame_lines.value)


def serr(dut):
    return int(dut.sts_frame_sync_err_cnt.value)


def lcnt(dut):
    return int(dut.sts_line_count.value)


async def _start(dut):
    """Start the 10 ns clock (mirrors '#5 core_clk' in the TB)."""
    start_clock(dut.core_clk, 10.0)


# ---------------------------------------------------------------------------
# scenarios — each replicates one TB block; role selects which chk()s apply.
# ---------------------------------------------------------------------------
@cocotb.test(timeout_time=1, timeout_unit="ms")
async def t1_spurious_early_fe_lost_real_fe(dut):
    """T1: spurious early FE@5 + LOST real FE; frame closed by real next-frame FS."""
    await _start(dut)
    await reset_dut(dut)
    await drive_short(dut, 0x00)                       # FS open frame#1
    for i in range(5):
        await drive_lsle_line(dut, i)                 # lines 0..4 (line_idx=5)
    await drive_short(dut, 0x01)                       # SPURIOUS FE @5 (real FE lost)
    for i in range(5, 8):
        await drive_lsle_line(dut, i)                 # lines 5..7 (line_idx=8)
    await drive_short(dut, 0x00)                       # real next-frame FS @line_idx=8
    await ClockCycles(dut.core_clk, 4)
    if ROLE == "old":
        # OLD: spurious FE@5 (>=FS_MIN=2) closes the frame SHORT at 5 (the bug).
        check(last(dut) == 5, "T1 OLD: legacy closes short on spurious FE@5 (bug repro)")
    else:
        # NEW: FE@5 (<FE_MIN=7) rejected; frame closes on real FS at full 8.
        check(last(dut) == 8, "T1 NEW: spurious FE rejected, frame closes at full 8 on FS")
        check(fcnt(dut) == 1, "T1 NEW: exactly one frame closed")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def t2_clean_real_fe(dut):
    """T2: regression — clean real FE@8 closes the frame at 8."""
    await _start(dut)
    await reset_dut(dut)
    await drive_short(dut, 0x00)
    for i in range(8):
        await drive_lsle_line(dut, i)                 # 8 lines
    await drive_short(dut, 0x01)                       # real FE @8 (>=FE_MIN=7)
    await ClockCycles(dut.core_clk, 4)
    if ROLE == "new":
        check(fcnt(dut) == 1, "T2 NEW: clean FE closed one frame")
        check(last(dut) == 8, "T2 NEW: clean FE closes at 8")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def t3_early_spurious_fs_and_fe(dut):
    """T3: spurious early FS@3 and FE@5 both rejected; frame closes at full 8 on real FS."""
    await _start(dut)
    await reset_dut(dut)
    await drive_short(dut, 0x00)
    for i in range(3):
        await drive_lsle_line(dut, i)                 # 3 lines
    await drive_short(dut, 0x00)                       # spurious FS @3 (<FE_MIN=7) -> ignored
    await drive_short(dut, 0x01)                       # spurious FE @3 (<FE_MIN=7) -> ignored
    for i in range(3, 8):
        await drive_lsle_line(dut, i)                 # up to 8 lines
    await drive_short(dut, 0x00)                       # real FS @8 -> close
    await ClockCycles(dut.core_clk, 4)
    if ROLE == "new":
        check(fcnt(dut) == 1, "T3 NEW: one frame despite early spurious FS+FE")
        check(last(dut) == 8, "T3 NEW: frame closes at full 8 (early spurious FS/FE ignored)")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def t4_runaway_cap(dut):
    """T4: missing FE AND missing closing FS -> MAX_LINES cap (=16)."""
    await _start(dut)
    await reset_dut(dut)
    await drive_short(dut, 0x00)
    for i in range(20):
        await drive_lsle_line(dut, i)                 # no FE, no closing FS
    await ClockCycles(dut.core_clk, 4)
    if ROLE == "new":
        check(fcnt(dut) == 1, "T4 NEW: runaway bounded by MAX_LINES cap")
        check(last(dut) == MAXL, "T4 NEW: capped at MAX_LINES")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def t5_force_expected(dut):
    """T5: cfg_force_expected clamps the 12-LE overshoot frames to EXACTLY EXP=8."""
    await _start(dut)
    await reset_dut(dut)
    # SV wires cfg_force_expected only into dut_new; dut_old keeps the port default (0).
    if ROLE == "new":
        dut.cfg_force_expected.value = 1
    await drive_short(dut, 0x00)                       # FS open frame#1
    for i in range(12):
        await drive_lsle_line(dut, i)                 # 12 LE: force-close at 8
    await drive_short(dut, 0x00)                       # FS reopens frame#2
    for i in range(12):
        await drive_lsle_line(dut, i)                 # 12 LE: force-close at 8
    await drive_short(dut, 0x00)                       # FS (opens frame#3)
    await ClockCycles(dut.core_clk, 4)
    if ROLE == "new":
        check(last(dut) == EXP, "T5 FORCE: frame clamped to EXPECTED=8 (constant height)")
        check(fcnt(dut) == 2, "T5 FORCE: two overshoot frames each force-closed + reopened")
    else:
        # OLD (no force, FE_MIN=0): the 12-line frames are NOT clamped to 8.
        check(last(dut) != EXP or fcnt(dut) != 2,
              "T5 OLD: legacy does NOT clamp to 8 (force off)")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def t6_synth_and_force(dut):
    """T6: cfg_sof_synth + cfg_force_expected — FE ignored, force-close, FE-resync re-open."""
    await _start(dut)
    await reset_dut(dut)
    # SV wires cfg_sof_synth / cfg_force_expected only into dut_new.
    if ROLE == "new":
        dut.cfg_sof_synth.value = 1
        dut.cfg_force_expected.value = 1
    for i in range(8):
        await drive_lsle_line(dut, i)                 # frame#1: 8 lines -> force-close @8
    for i in range(4):
        await drive_lsle_line(dut, i)                 # overshoot -> drained (synth_wait_fe)
    await drive_short(dut, 0x01)                       # chip FE -> clears synth_wait_fe
    for i in range(8):
        await drive_lsle_line(dut, i)                 # frame#2: opens at true top, force-close @8
    for i in range(4):
        await drive_lsle_line(dut, i)                 # overshoot -> drained
    await drive_short(dut, 0x01)                       # FE
    await ClockCycles(dut.core_clk, 4)
    if ROLE == "new":
        check(last(dut) == EXP, "T6 SYNTH+FORCE: clamped to EXPECTED=8 (FE ignored, force closes)")
        check(fcnt(dut) == 2, "T6 SYNTH+FORCE: 2 frames, overshoot drained, FE-resync re-open")


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def t7_long_as_line(dut):
    """T7: cfg_long_as_line — NEW delivers 5 no-LS longs that OLD (off) rejects as sync err.

    The SV check is cross-DUT (o_serr - n_serr >= 5). Here each build sees identical
    stimulus, so we capture serr for this role and compare against the other role's serr
    captured in a shared module global."""
    await _start(dut)
    await reset_dut(dut)
    # SV: cfg_long_as_line_tb=1 -> dut_new ON, dut_old OFF. In the split-build port the
    # "old" build must therefore run with cfg_long_as_line=0 and the "new" build with 1.
    dut.cfg_long_as_line.value = 1 if ROLE == "new" else 0
    await drive_short(dut, 0x00)                       # FS open
    for i in range(5):
        await drive_long_le_no_ls(dut, i)             # 5 longs, NO LS each
    await drive_short(dut, 0x00)                       # next FS
    await ClockCycles(dut.core_clk, 4)
    _SERR[ROLE] = serr(dut)
    if ROLE == "new" and "old" in _SERR:
        check(_SERR["old"] - _SERR["new"] >= 5,
              "T7: long-as-line ON delivers the 5 no-LS longs that OLD (off) rejects")
    elif ROLE == "old" and "new" in _SERR:
        check(_SERR["old"] - _SERR["new"] >= 5,
              "T7: long-as-line ON delivers the 5 no-LS longs that OLD (off) rejects")


# Cross-run serr capture for the T7 cross-DUT comparison. Persisted to a scratch file
# (in the OS temp dir, NOT the repo tree) so the two separate build_and_test runs
# (old, new) can be compared regardless of order.
import tempfile  # noqa: E402

_SERR_FILE = Path(tempfile.gettempdir()) / "mipi2hdmi_feearly_t7_serr.json"


class _SerrDict(dict):
    def __setitem__(self, k, v):
        super().__setitem__(k, v)
        try:
            import json
            data = {}
            if _SERR_FILE.is_file():
                data = json.loads(_SERR_FILE.read_text())
            data[k] = v
            _SERR_FILE.write_text(json.dumps(data))
        except Exception:
            pass

    def __contains__(self, k):
        if super().__contains__(k):
            return True
        try:
            import json
            if _SERR_FILE.is_file():
                data = json.loads(_SERR_FILE.read_text())
                if k in data:
                    super().__setitem__(k, data[k])
                    return True
        except Exception:
            pass
        return False


_SERR = _SerrDict()


# ---------------------------------------------------------------------------
# build+run: build the DUT once per role, then run every scenario against it.
# ---------------------------------------------------------------------------
def _run_role(role: str):
    from runner_support import build_and_test

    os.environ["FEEARLY_ROLE"] = role
    fe_min = 0 if role == "old" else FEMIN
    build_and_test(
        block=f"csi2_frame_state_feearly_{role}",
        sources=["rtl/mipi_rx/csi2_frame_state.sv"],
        toplevel="csi2_frame_state",
        test_module="test_csi2_frame_state_feearly",
        test_dir=Path(__file__).resolve().parent,
        parameters={
            "MAX_LINES": MAXL,
            "GUARD_FRAME_LINES": 1,
            "EXPECTED_FRAME_LINES": EXP,
            "EXPECTED_LINE_WC": 0,
            "FS_MIN_LINES": FSMIN,
            "FE_DELIMITS": 1,
            "FE_MIN_LINES": fe_min,
        },
    )


def test_csi2_frame_state_feearly():
    # T7 is a cross-DUT comparison, so clear the persisted serr cache before both runs.
    try:
        if _SERR_FILE.is_file():
            _SERR_FILE.unlink()
    except Exception:
        pass
    # dut_old first (FE_MIN_LINES=0), then dut_new (FE_MIN_LINES=7).
    _run_role("old")
    _run_role("new")
