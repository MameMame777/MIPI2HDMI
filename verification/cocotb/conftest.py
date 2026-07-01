"""pytest bootstrap for the cocotb + Verilator verification environment.

Runs once in the pytest (host) process before any test is collected: makes the cocotb
package importable, and fails loud if pytest is running under the wrong Python (the VPI
link is ABI-tied to the MinGW ucrt64 interpreter -- an MSVC/venv python silently mismatches).

The per-simulation process (the Verilated executable with embedded Python) does NOT run
this conftest; each ``test_<block>.py`` therefore adds the cocotb dir to ``sys.path`` at
import time so ``lib`` is importable inside the sim.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

_COCOTB_DIR = Path(__file__).resolve().parent
if str(_COCOTB_DIR) not in sys.path:
    sys.path.insert(0, str(_COCOTB_DIR))

import cocotb_site as cs  # noqa: E402


def _assert_mingw_python() -> None:
    try:
        root = cs.msys2_root()
    except cs.ToolchainError as exc:
        raise RuntimeError(str(exc)) from exc
    exe = Path(sys.executable).resolve()
    base = Path(sys.base_prefix).resolve()      # a venv's base install; == ucrt64 dir for us
    ucrt = (root / "ucrt64").resolve()
    # Accept the raw ucrt64 python (exe under ucrt64) OR a venv whose BASE is that ucrt64
    # python (same libpython -> same VPI ABI). Reject an MSVC/other-base python/venv.
    ok = (ucrt in exe.parents) or (base == ucrt) or (ucrt in base.parents)
    if not ok:
        raise RuntimeError(
            "cocotb tests must run under the MSYS2 ucrt64 (MinGW) Python (or a venv based on "
            "it) for VPI ABI compatibility.\n"
            f"  running:     {exe}\n  base_prefix: {base}\n  expected ucrt64: {ucrt}\n"
            "Use scripts/run_cocotb.ps1 / pytest_cocotb.ps1, which select the project venv "
            "(verification/cocotb/.venv) or the ucrt64 python automatically."
        )


_assert_mingw_python()
