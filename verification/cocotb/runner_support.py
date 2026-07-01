"""Shared build+run harness used by every ``test_<block>.py`` and by ``runner.py``.

A per-block test file exposes ``def test_<block>()`` (the pytest entry point) which calls
:func:`build_and_test`. This module owns the native-Windows toolchain setup (PATH, the
``make`` shim, the static VPI lib) and drives ``cocotb_tools.runner`` so the individual
tests stay declarative.
"""
from __future__ import annotations

import os
import shutil
import sys
from pathlib import Path
from typing import Mapping, Optional, Sequence

import cocotb_site as cs

_TOOLCHAIN_READY = False


def _export_sim_dll_dirs(root: Path) -> None:
    """Tell the sim's ``sitecustomize`` which dirs to ``os.add_dll_directory`` so the
    embedded Python can load stdlib ``.pyd`` DLLs (Windows ignores PATH for those)."""
    import cocotb_tools.config as cfg

    dirs = [str(cs.ucrt64_bin(root)), str(cfg.libs_dir)]
    os.environ["COCOTB_DLL_DIRS"] = os.pathsep.join(dirs)
    # Ensure the cocotb dir (holding sitecustomize.py) is on sys.path so the runner
    # propagates it via PYTHONPATH to the sim interpreter.
    if str(cs.COCOTB_DIR) not in sys.path:
        sys.path.insert(0, str(cs.COCOTB_DIR))


def _ensure_make_shim(root: Path) -> None:
    # WA#3: cocotb calls subprocess(["make", ...]); ucrt64 ships mingw32-make.exe, not
    # make.exe. Provide a make.exe shim on PATH (idempotent, cached).
    cs.MAKE_SHIM_DIR.mkdir(parents=True, exist_ok=True)
    make_exe = cs.MAKE_SHIM_DIR / "make.exe"
    if not make_exe.is_file():
        shutil.copy2(cs.ucrt64_bin(root) / "mingw32-make.exe", make_exe)


def prepare_verilator_toolchain() -> None:
    """PATH + make shim + static VPI lib. Idempotent; safe to call per test."""
    global _TOOLCHAIN_READY
    if _TOOLCHAIN_READY:
        return
    import bootstrap_vpi

    root = cs.prepend_path()
    _ensure_make_shim(root)
    _export_sim_dll_dirs(root)
    bootstrap_vpi.ensure()
    _TOOLCHAIN_READY = True


def _resolve(src) -> Path:
    p = Path(src)
    return p if p.is_absolute() else (cs.REPO_ROOT / p)


def build_and_test(
    block: str,
    sources: Sequence[str],
    toplevel: str,
    test_module: str,
    test_dir: os.PathLike,
    parameters: Optional[Mapping[str, object]] = None,
    engine: str = "verilator",
    waves: Optional[bool] = None,
    timescale=("1ns", "1ps"),
    testcase: Optional[str] = None,
    build_dir: Optional[os.PathLike] = None,
) -> Path:
    """Build ``toplevel`` from ``sources`` and run ``test_module`` under ``engine``.

    Raises on build failure or any failing cocotb test (so pytest reports a red).
    Returns the path to the JUnit results XML.
    """
    from cocotb_tools.runner import get_runner

    if waves is None:
        waves = os.environ.get("COCOTB_WAVES") == "1"
    parameters = dict(parameters or {})
    resolved = [_resolve(s) for s in sources]

    if engine == "verilator":
        prepare_verilator_toolchain()
        build_args = cs.common_build_args()
    elif engine == "icarus":
        # cocotb ships a Windows Icarus VPI (libcocotbvpi_icarus.vpl); only PATH is needed.
        cs.prepend_path()
        build_args = []
    else:
        raise cs.ToolchainError(f"Unsupported engine {engine!r} (verilator|icarus).")

    if build_dir is None:
        # Separate waves / no-waves build dirs. A Verilator model built with --trace leaves
        # Vtop__Trace*.cpp behind; a later no-trace rebuild in the SAME dir regenerates the
        # symbols without trace members and `make` fails compiling the stale trace files.
        build_dir = cs.BUILD_DIR / "sim" / (f"{block}_waves" if waves else block)

    # Pin the cocotb random seed for a REPRODUCIBLE regression: cocotb 2.0 randomizes the
    # resume order of coroutines woken by the same trigger, which can shift a driven input
    # by a cycle and make a phase-sensitive test flaky run-to-run. A fixed seed makes every
    # block deterministic (a pass always passes, a fail always fails). Override with
    # COCOTB_SEED for reproduction/bisection.
    seed = os.environ.get("COCOTB_SEED", "1")

    runner = get_runner(engine)
    runner.build(
        sources=resolved,
        hdl_toplevel=toplevel,
        parameters=parameters,
        build_args=build_args,
        build_dir=build_dir,
        always=True,
        timescale=timescale,
        waves=waves,
    )
    return runner.test(
        hdl_toplevel=toplevel,
        test_module=test_module,
        test_dir=test_dir,
        build_dir=build_dir,
        timescale=timescale,
        testcase=testcase,
        waves=waves,
        seed=seed,
    )
