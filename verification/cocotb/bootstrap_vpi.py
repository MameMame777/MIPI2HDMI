"""Build the cocotb VPI static library for Verilator on Windows (WA#5/#6).

Background: cocotb's Verilator runner links the simulation executable with
``-lcocotbvpi_verilator``, but the prebuilt Windows (mingw ucrt64) cocotb wheel does NOT
ship that library -- Verilator generates a standalone executable, and a Windows DLL cannot
export VPI symbols (``vlog_startup_routines_bootstrap`` et al.) into a host executable, so
the VPI layer must be *statically linked in*. cocotb builds this lib on POSIX but guards it
out on Windows.

This module reproduces that build: it fetches the cocotb VPI C++ sources matching the
installed cocotb version, compiles ``share/lib/vpi/*.cpp`` into ``libcocotbvpi_verilator.a``
(defines that strip the dllimport decorators, force the symbols to be *provided* here), and
drops the archive into cocotb's own ``libs`` dir -- the only ``-L`` path the runner places
*before* its ``-lcocotbvpi_verilator`` on the link line.

``ensure()`` is idempotent and self-healing: it hashes {cocotb version, gcc version,
verilator version, source names, compile flags} into a stamp; it rebuilds only when that
changes, so a cocotb/Verilator/GCC upgrade transparently triggers a fresh build.
"""
from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys
import tarfile
import urllib.request
from pathlib import Path

import cocotb_site as cs

VPI_LIB_NAME = "libcocotbvpi_verilator.a"

# WA#5: define COCOTBVPI_EXPORTS so the symbols are dllexport (provided here, not imported);
# blank PLI_DLLISPEC/ESPEC strip the __imp_ decorators in the vendored vpi_user.h.
_DEFINES = [
    "-DCOCOTBVPI_EXPORTS=1",
    "-DVERILATOR=1",
    "-D__STDC_FORMAT_MACROS=1",
    "-DWIN32=1",
    "-DPLI_DLLISPEC=",
    "-DPLI_DLLESPEC=",
]
_CFLAGS = ["-O2", "-std=c++17", "-fpermissive"]


def _cocotb_version() -> str:
    import cocotb

    return cocotb.__version__


def _libs_dir() -> Path:
    import cocotb_tools.config as cfg

    return Path(cfg.libs_dir)


def _tool_version(exe: str, *args: str) -> str:
    try:
        out = subprocess.run(
            [exe, *(args or ("--version",))],
            capture_output=True, text=True, check=False,
        )
        return (out.stdout or out.stderr).splitlines()[0].strip()
    except Exception as exc:  # noqa: BLE001
        return f"<{exe}: {exc}>"


def _vpi_source_dir(version: str) -> Path:
    """Locate (fetching+caching if needed) the cocotb VPI C++ sources for ``version``.

    The binary wheel ships no .cpp sources, so we download the matching sdist from PyPI
    once and cache it under ``.build/cocotb-<version>-src``. To run fully offline, pre-seed
    that directory with an extracted sdist (``src/cocotb/...``).
    """
    cache = cs.BUILD_DIR / f"cocotb-{version}-src"
    vpi_dir = cache / "src" / "cocotb" / "share" / "lib" / "vpi"
    if (vpi_dir / "VpiImpl.cpp").is_file():
        return cache / "src" / "cocotb"

    cs.BUILD_DIR.mkdir(parents=True, exist_ok=True)
    url = _pypi_sdist_url(version)
    tgz = cs.BUILD_DIR / f"cocotb-{version}.tar.gz"
    if not tgz.is_file():
        print(f"[bootstrap_vpi] downloading cocotb {version} sdist for VPI sources ...")
        urllib.request.urlretrieve(url, tgz)
    with tarfile.open(tgz) as tf:
        # extract into cache/, stripping the top-level cocotb-<version>/ dir
        prefix = f"cocotb-{version}/"
        for m in tf.getmembers():
            if m.name.startswith(prefix):
                m.name = m.name[len(prefix):]
                tf.extract(m, cache)
    if not (vpi_dir / "VpiImpl.cpp").is_file():
        raise cs.ToolchainError(
            f"cocotb VPI sources not found after extracting {tgz}. "
            f"Expected {vpi_dir / 'VpiImpl.cpp'}."
        )
    return cache / "src" / "cocotb"


def _pypi_sdist_url(version: str) -> str:
    api = f"https://pypi.org/pypi/cocotb/{version}/json"
    with urllib.request.urlopen(api, timeout=60) as resp:
        data = json.load(resp)
    for u in data["urls"]:
        if u["packagetype"] == "sdist":
            return u["url"]
    raise cs.ToolchainError(f"No sdist found on PyPI for cocotb {version}.")


def _stamp_inputs(version: str, sources: list[Path]) -> str:
    payload = {
        "cocotb": version,
        "gcc": _tool_version("g++"),
        "verilator": _tool_version("verilator_bin.exe"),
        "sources": sorted(p.name for p in sources),
        "defines": _DEFINES,
        "cflags": _CFLAGS,
    }
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()


def ensure(force: bool = False) -> Path:
    """Build (if needed) and return the path to ``libcocotbvpi_verilator.a``.

    Safe to call once per pytest session. Requires the toolchain to be on PATH already
    (call :func:`cocotb_site.prepend_path` first).
    """
    version = _cocotb_version()
    src_root = _vpi_source_dir(version)
    vpi_dir = src_root / "share" / "lib" / "vpi"
    share_inc = src_root / "share" / "include"
    sources = sorted(vpi_dir.glob("*.cpp"))
    if not sources:
        raise cs.ToolchainError(f"No VPI .cpp sources under {vpi_dir}.")

    lib_path = _libs_dir() / VPI_LIB_NAME
    stamp_path = cs.BUILD_DIR / ".vpi_stamp"
    want = _stamp_inputs(version, sources)

    if not force and lib_path.is_file() and stamp_path.is_file():
        if stamp_path.read_text(encoding="utf-8").strip() == want:
            return lib_path

    print(f"[bootstrap_vpi] building {VPI_LIB_NAME} (cocotb {version}) ...")
    cs.BUILD_DIR.mkdir(parents=True, exist_ok=True)
    obj_dir = cs.BUILD_DIR / "vpi_obj"
    obj_dir.mkdir(parents=True, exist_ok=True)
    includes = [f"-I{share_inc}", f"-I{src_root}", f"-I{vpi_dir}"]

    objs: list[str] = []
    for src in sources:
        obj = obj_dir / (src.stem + ".o")
        cmd = ["g++", *_CFLAGS, *_DEFINES, *includes, "-c", str(src), "-o", str(obj)]
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode != 0:
            raise cs.ToolchainError(
                f"VPI compile failed for {src.name}:\n{res.stderr[-4000:]}"
            )
        objs.append(str(obj))

    if lib_path.is_file():
        lib_path.unlink()
    res = subprocess.run(["ar", "rcs", str(lib_path), *objs], capture_output=True, text=True)
    if res.returncode != 0 or not lib_path.is_file():
        raise cs.ToolchainError(f"ar failed:\n{res.stderr}")

    stamp_path.write_text(want, encoding="utf-8")
    print(f"[bootstrap_vpi] built {lib_path} ({lib_path.stat().st_size} bytes)")
    return lib_path


if __name__ == "__main__":
    cs.prepend_path()
    ensure(force="--force" in sys.argv)
    print("OK")
