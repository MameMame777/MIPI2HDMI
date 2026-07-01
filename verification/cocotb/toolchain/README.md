# cocotb + Verilator toolchain (native Windows / MSYS2 ucrt64)

This is the runbook for the license-free RTL verification environment that replaces DSim.
It runs **cocotb 2.0.1 + Verilator 5.048 natively on Windows** via MSYS2 ucrt64 (no WSL),
driven by the Python `cocotb_tools.runner` API. Based on the write-up at
<https://qiita.com/MameMame777/items/2e954be29b9f21934567>.

## Tool versions (pinned)

| Component | Version | Package |
|-----------|---------|---------|
| Verilator | 5.048 | `mingw-w64-ucrt-x86_64-verilator` |
| GCC | 16.1.0 | `mingw-w64-ucrt-x86_64-gcc` |
| Python | 3.14 (MinGW, **not** MSVC) | `mingw-w64-ucrt-x86_64-python` |
| GNU Make | 4.4.1 | `mingw-w64-ucrt-x86_64-make` |
| Perl | 5.42 | MSYS base (`usr/bin`) |
| cocotb | 2.0.1 | pip (`requirements.lock`) |
| Icarus (fallback) | latest | `mingw-w64-ucrt-x86_64-iverilog` |

## Install (one-time)

Prerequisite: MSYS2 must be installed (<https://www.msys2.org>). Then:

```powershell
.\verification\cocotb\toolchain\install_toolchain.ps1
```

This runs `pacman` for the ucrt64 packages, `pip install -r requirements.lock` into the
**ucrt64** python, and builds the static VPI lib. It resolves the MSYS2 root from
`$MSYS2_ROOT` → a `verilator` on PATH → well-known dirs; set `$env:MSYS2_ROOT` if your
install is elsewhere (no absolute paths are committed).

## Run tests

```powershell
.\scripts\run_cocotb.ps1 -Suite smoke            # the completion gate
.\scripts\run_cocotb.ps1 csi2_packet_parser      # one block
.\scripts\run_cocotb.ps1 csi2_packet_parser -Waves   # + dump.vcd
.\scripts\run_cocotb.ps1 -List                   # list blocks
```

Or directly with the ucrt64 python: `python -m pytest verification/cocotb/<block>`.
Reports land in the gitignored `verification/cocotb/_exec/regression_cocotb_<ts>.md`.

## The 8 Windows workarounds (and where each lives)

The article documents 7; a Windows-embedded-Python DLL issue (#8) surfaced in this repo.

| # | Problem | Fix | Home |
|---|---------|-----|------|
| 1 | tools not on PATH | prepend ucrt64\bin + usr\bin (perl) | `cocotb_site.prepend_path` |
| 2 | `shutil.which("verilator")` finds `verilator.bat` (a cmd batch `perl` can't run) | a perl wrapper named `verilator.cmd`, put first on PATH so `shutil.which` returns it; run as `perl verilator.cmd` it execs `verilator_bin.exe` | `toolchain/verilator.cmd` |
| 3 | `subprocess(["make"])` finds no `make.exe` (ucrt64 has `mingw32-make.exe`) | copy it to a `make.exe` shim on PATH | `runner_support._ensure_make_shim` → `toolchain/make_shim/` |
| 4 | `VERILATOR_ROOT` back-slashes corrupt generated Makefiles | forward slashes | `cocotb_site.verilator_root` |
| 5 | cocotb ships no `libcocotbvpi_verilator` on Windows (Verilator links it into the exe; a DLL can't export VPI symbols to a host exe) | compile the cocotb VPI sources into a static `.a` in cocotb's own `libs` dir (the `-L` the runner puts before its `-lcocotbvpi_verilator`) | `bootstrap_vpi.ensure` |
| 6 | ucrt64 libstdc++ lacks the out-of-line `std::string` move ctor Verilator's `-Os` references | force `-O2` | `cocotb_site.common_build_args` |
| 7 | waveforms | `waves=True` → `dump.vcd` | runner `--waves` |
| **8** | the Verilated exe's embedded Python can't load stdlib `.pyd`s (Windows resolves a `.pyd`'s dependent DLLs from the *exe* dir + AddDllDirectory, **not** PATH; ucrt64 runtime DLLs are invisible) → `ImportError: DLL load failed while importing binascii` | `os.add_dll_directory(ucrt64\bin, cocotb\libs)` at sim-Python startup via `sitecustomize` on the sim's PYTHONPATH | `sitecustomize.py` + `runner_support._export_sim_dll_dirs` |

Lint note: Verilator lint is stricter than DSim; `-Wno-fatal` keeps WIDTH/UNOPTFLAT warnings
visible but non-fatal so existing RTL builds. Fix real issues at the source; scope a
`lint_off` only for documented false positives.

## Upgrade runbook (cocotb / Verilator / GCC bump)

1. Edit `requirements.lock` (cocotb) and/or `pacman -Syu` (Verilator/GCC).
2. `bootstrap_vpi.ensure()` self-heals: it hashes {cocotb ver, gcc ver, verilator ver,
   VPI source names, flags} and rebuilds `libcocotbvpi_verilator.a` automatically when any
   changes (it re-fetches the matching cocotb sdist for the VPI sources — needs network
   once, cached under `.build/`). To force: `python verification/cocotb/bootstrap_vpi.py --force`.
3. Run `-Suite smoke`. If the VPI link breaks on a cocotb bump, fall back to Icarus (below)
   while the VPI sources/flags are reconciled.

## Icarus fallback

cocotb ships a Windows Icarus VPI (`libcocotbvpi_icarus.vpl`) — no static-link workaround.
Set `engine = "icarus"` on a block in `manifest.toml` (used for D-PHY blocks whose real
ISERDES/bitslip timing the Verilator stub cells can't reproduce). It is also the escape
hatch if a Verilator/cocotb upgrade ever breaks the VPI build.

## Troubleshooting

- **`ImportError: DLL load failed while importing binascii`** → workaround #8; ensure the
  sim runs via `runner_support` (which sets `COCOTB_DLL_DIRS`) and `sitecustomize.py` is on
  PYTHONPATH.
- **`verilator executable not found`** → `$env:MSYS2_ROOT` unset and no verilator on PATH.
- **VPI ABI / crash on import** → you're running an MSVC/venv python; use the ucrt64 python
  (`conftest.py` asserts this).
- **`cannot find -lcocotbvpi_verilator`** → the VPI lib isn't built; run `bootstrap_vpi.py`.
