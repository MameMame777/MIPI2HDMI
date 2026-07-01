"""Toolchain + path resolution for the cocotb + Verilator verification environment.

Native Windows / MSYS2 ucrt64 (no WSL). This module is the single source of truth for
"where are the tools" and "what flags does Verilator need on Windows". It mirrors the
DSIM_HOME resolution pattern in ``scripts/run_dsim.ps1``: resolve from env, then probe,
then fail loud with an actionable message. No absolute paths are committed -- every path
is *derived* from the resolved MSYS2 root.

MSYS2 root resolution order:
  1. ``$MSYS2_ROOT`` (the documented override knob).
  2. Derive from a ``verilator(.bat|_bin.exe)`` found on ``PATH`` (works out-of-the-box
     when MSYS2 is already on PATH, even at a non-standard install dir).
  3. Probe well-known roots (``C:\\msys64`` etc.).
  4. Raise ``ToolchainError``.

Background: the Windows-native cocotb+Verilator recipe needs seven workarounds; this
module owns the path-related ones (WA#1 PATH, WA#4 VERILATOR_ROOT forward slashes) and
exposes the common Verilator build args (WA#5 static VPI lib, WA#6 force -O2). See
``toolchain/README.md`` for the full list.
"""
from __future__ import annotations

import os
import shutil
from pathlib import Path

# verification/cocotb/cocotb_site.py -> parents[0]=cocotb, [1]=verification, [2]=repo root
COCOTB_DIR = Path(__file__).resolve().parent
REPO_ROOT = COCOTB_DIR.parents[1]
TOOLCHAIN_DIR = COCOTB_DIR / "toolchain"
MAKE_SHIM_DIR = TOOLCHAIN_DIR / "make_shim"
BUILD_DIR = COCOTB_DIR / ".build"
VPI_LIB_NAME = "libcocotbvpi_verilator.a"

_WELL_KNOWN_ROOTS = (
    r"C:\msys64",
    r"C:\msys2",
    r"C:\tools\msys64",
)


class ToolchainError(RuntimeError):
    """Raised when the MSYS2 ucrt64 toolchain cannot be located or is incomplete."""


def _is_valid_root(p: Path | None) -> bool:
    return p is not None and (p / "ucrt64" / "bin" / "verilator_bin.exe").is_file()


def _root_from_path_verilator() -> Path | None:
    # shutil.which finds verilator.bat (the .BAT perl cannot run -- WA#2); its location
    # is <root>/ucrt64/bin/, so the root is two parents up from the bin directory.
    for name in ("verilator_bin.exe", "verilator.bat", "verilator"):
        hit = shutil.which(name)
        if hit:
            cand = Path(hit).resolve().parents[2]
            if _is_valid_root(cand):
                return cand
    return None


def msys2_root() -> Path:
    """Locate the MSYS2 install root (the dir containing ``ucrt64/``)."""
    env = os.environ.get("MSYS2_ROOT")
    if env:
        p = Path(env)
        if _is_valid_root(p):
            return p
        raise ToolchainError(
            f"MSYS2_ROOT={env!r} is set but "
            f"{p / 'ucrt64' / 'bin' / 'verilator_bin.exe'} is missing."
        )

    derived = _root_from_path_verilator()
    if derived:
        return derived

    candidates = list(_WELL_KNOWN_ROOTS)
    local = os.environ.get("LOCALAPPDATA")
    if local:
        candidates.append(str(Path(local) / "msys64"))
    for c in candidates:
        if _is_valid_root(Path(c)):
            return Path(c)

    raise ToolchainError(
        "MSYS2 ucrt64 toolchain not found. Set MSYS2_ROOT to the install dir "
        "(it must contain ucrt64\\bin\\verilator_bin.exe), e.g.\n"
        "    $env:MSYS2_ROOT = 'E:\\path\\to\\msys'\n"
        "or put the ucrt64 bin dir on PATH. See verification/cocotb/toolchain/README.md."
    )


def ucrt64_bin(root: Path | None = None) -> Path:
    return (root or msys2_root()) / "ucrt64" / "bin"


def usr_bin(root: Path | None = None) -> Path:
    return (root or msys2_root()) / "usr" / "bin"


def ucrt64_python(root: Path | None = None) -> Path:
    """The MinGW python.exe -- the only ABI-compatible interpreter for the VPI link."""
    return ucrt64_bin(root) / "python.exe"


def verilator_root(root: Path | None = None) -> str:
    # WA#4: forward slashes, else the path is mangled inside the generated Makefiles.
    return str((root or msys2_root()) / "ucrt64" / "share" / "verilator").replace("\\", "/")


def prepend_path(root: Path | None = None) -> Path:
    """Make the toolchain discoverable to cocotb's runner (idempotent within a process).

    Prepends, in order: our committed shims (the perl ``verilator`` wrapper WA#2 and the
    ``make.exe`` shim WA#3), then ucrt64\\bin (gcc, verilator_bin, python) and usr\\bin
    (perl) -- WA#1. Also exports VERILATOR_ROOT (WA#4). Returns the resolved root.
    """
    root = root or msys2_root()
    parts = [
        str(TOOLCHAIN_DIR),     # WA#2: perl `verilator` wrapper, found before verilator.bat
        str(MAKE_SHIM_DIR),     # WA#3: make.exe shim
        str(ucrt64_bin(root)),  # WA#1: gcc, verilator_bin.exe, python.exe
        str(usr_bin(root)),     # WA#1: perl lives here
    ]
    existing = os.environ.get("PATH", "")
    os.environ["PATH"] = os.pathsep.join(parts) + (os.pathsep + existing if existing else "")
    os.environ["VERILATOR_ROOT"] = verilator_root(root)
    return root


def common_build_args() -> list[str]:
    """Verilator build_args shared by every block.

    WA#6: force -O2 -- ucrt64's shared libstdc++ does not export the std::string move ctor
    out-of-line that Verilator's -Os runtime references. WA#5: link -lgpi/-lgpilog so the
    statically-linked cocotb VPI lib (built by bootstrap_vpi into cocotb's own libs dir,
    which the runner already puts on -L before its -lcocotbvpi_verilator) resolves its GPI
    symbols. The DUT RTL has no ``#delay`` so Verilator's slow --timing is intentionally
    NOT added; waves/--trace is handled by the runner via ``waves=``.

    ``-Wno-fatal``: Verilator's lint is stricter than DSim and treats WIDTH/UNOPTFLAT/etc.
    as fatal. The existing RTL has benign width mismatches DSim tolerated; keep the warnings
    visible (printed) but non-fatal so migration is unblocked. Real RTL issues surfaced this
    way get fixed at the source; documented false positives get a scoped ``lint_off``.

    ``-CFLAGS -Wno-attributes``: g++ emits ~200 harmless ``dllimport`` redeclaration warnings
    per build when compiling Verilator's ``verilated_vpi.cpp`` against cocotb's VPI headers on
    Windows. They dominated the run log (signal buried); silence just that g++ warning class.
    """
    return [
        "-Wno-fatal",
        "-CFLAGS", "-O2",
        "-CFLAGS", "-Wno-attributes",
        "-LDFLAGS", "-lgpi", "-LDFLAGS", "-lgpilog",
    ]


def summary() -> str:
    root = msys2_root()
    lines = [
        f"MSYS2_ROOT     : {root}",
        f"ucrt64 bin     : {ucrt64_bin(root)}",
        f"ucrt64 python  : {ucrt64_python(root)}",
        f"VERILATOR_ROOT : {verilator_root(root)}",
        f"repo root      : {REPO_ROOT}",
    ]
    return "\n".join(lines)


if __name__ == "__main__":
    print(summary())
