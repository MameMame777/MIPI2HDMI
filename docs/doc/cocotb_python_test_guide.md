# Python Test Guide — cocotb + Verilator Verification Environment

**Scope.** This document explains how the project's Python-based RTL tests work
(what runs, in what order, and why the code is shaped the way it is) and lays out
a concrete improvement plan. It is the companion "how it works / where to go next"
doc to the two existing runbooks:

- [`verification/cocotb/README.md`](../../verification/cocotb/README.md) — quick-start / layout
- [`verification/cocotb/toolchain/README.md`](../../verification/cocotb/toolchain/README.md) — install + the 8 Windows workarounds
- [`adr_001_cocotb_verilator_over_dsim.md`](adr_001_cocotb_verilator_over_dsim.md) — why cocotb+Verilator replaced DSim
- [`image_file_verification.md`](image_file_verification.md) ([日本語](image_file_verification_ja.md)) — the image-file-driven pyuvm test (`img_file_uvm`): stream any image through an img_proc DUT, compare input/expected/output images against RTL-exact Python golden models

Toolchain: **cocotb 2.0.1 + Verilator 5.048**, native Windows via MSYS2 ucrt64 (no WSL).

**Two verification layers.** The plain drivers/monitors in `lib/` (used by the 53 ported
tests, straight-line stimulus) and, additive on top, a **real UVM** layer built on
**pyuvm 4.0.1** (pure Python, cocotb-2.0-compatible; pinned in `requirements.lock`) under
`verification/cocotb/lib/uvm/` for structured testbenches. The pyuvm drivers reuse the plain
ones by composition, so cadence lives in one place; see the README "pyuvm layer" section.

---

## Part 1 — How the Python test works

### 1.1 The big picture

The tests are ordinary **pytest** test functions, but each one drives a hardware
simulation. The chain is:

```
pytest (ucrt64 python)                 ← host process; collects test_<block>()
  └─ runner_support.build_and_test()   ← sets up toolchain, calls cocotb runner
       └─ Verilator                    ← compiles RTL → a native .exe with embedded Python
            └─ <block>.exe             ← SIM process; re-imports test_<block>.py,
                                          runs the @cocotb.test() coroutines against the DUT
```

There are **two Python processes**, and the same `test_<block>.py` file is imported
in both — this dual role is the single most important thing to understand:

| Role | Runs in | Which function |
|------|---------|----------------|
| **pytest entry point** | the host `pytest` process | `def test_<block>()` — builds + launches the sim |
| **testbench** | the Verilated `.exe` (embedded Python) | the `@cocotb.test()` coroutines — drive/observe the DUT |

`conftest.py` (host only) asserts the interpreter is the MSYS2 ucrt64 python, because
the VPI bridge is ABI-tied to that exact interpreter — an MSVC/venv python silently
mismatches. The sim process does **not** run `conftest.py`; instead each test file
inserts the cocotb dir onto `sys.path` at import time so `lib.*` is importable inside
the sim.

### 1.2 Directory layout and file responsibilities

```
verification/cocotb/
  cocotb_site.py        Toolchain path resolution (MSYS2_ROOT → derive-from-PATH → probe → fail loud);
                        owns Verilator build flags. Single source of truth for "where are the tools".
  runner_support.py     build_and_test(): the shared build+run harness every test file calls.
  bootstrap_vpi.py      Compiles the static cocotb VPI lib for Verilator on Windows (WA#5/#6),
                        content-hash cached so cocotb/Verilator/GCC upgrades self-heal.
  conftest.py           Host-process bootstrap: makes cocotb importable, asserts ucrt64 python.
  sitecustomize.py      Sim-process startup: os.add_dll_directory() so embedded Python finds .pyd DLLs (WA#8).
  runner.py             Regression front-end: manifest → suite selection → per-block pytest → markdown report.
  manifest.toml         Block registry (group, suites, engine, path). Successor to the DSim .f filelists.
  requirements.lock     Pinned deps: cocotb==2.0.1, find_libpython, pytest.
  toolchain/            install/setup PowerShell, the perl `verilator` wrapper, the make shim, README.
  lib/                  Reusable testbench helpers (see §1.5).
  <block>/test_<block>.py   One directory per RTL block; the test + its pytest entry point.
  <block>/<block>_stubs.sv  (optional) behavioral primitive stubs / harness top for D-PHY & E2E blocks.
```

### 1.3 Anatomy of one test file

Using [`csi2_packet_parser/test_csi2_packet_parser.py`](../../verification/cocotb/csi2_packet_parser/test_csi2_packet_parser.py)
as the reference. Every ported block follows this same three-part shape:

**(a) Model coroutines / monitors** — the DSim testbench's `initial`/`always_ff` blocks
become `async def` coroutines started with `cocotb.start_soon(...)`. In the parser test,
the SV "ECC responder" `initial` block becomes the `ecc_responder(dut)` coroutine, and
the `always_ff` logger becomes the `Capture` monitor class. The translation rule is:

| DSim SystemVerilog | cocotb Python |
|--------------------|---------------|
| `initial` stimulus/model block | `async def` + `cocotb.start_soon()` |
| `always_ff` logger | monitor coroutine capturing signals on `RisingEdge` |
| `check_condition(...)` | `check(cond, msg)` from `lib/scoreboard.py` |
| `#N ms` watchdog | `@cocotb.test(timeout_time=N, timeout_unit="ms")` |
| TB `localparam` | `parameters={...}` passed to the build |

**(b) `@cocotb.test()` coroutines** — the actual scenarios. Each is a fresh-reset run;
the parser test splits its three scenarios (`short_packet`, `long_packet`,
`truncated_packet`) into three tests. A test **passes by returning normally** and
**fails by raising** (`check()` raises `AssertionError("CHECK FAILED: ...")` — the same
`CHECK FAILED` token the old DSim TBs emitted, so logs stay grep-compatible).

**(c) The pytest entry `def test_<block>()`** — calls
`runner_support.build_and_test(...)`, declaring the RTL `sources`, the `toplevel` module,
the `test_module` name, and RTL `parameters`. This is what pytest collects and what
actually compiles the RTL and launches the sim.

### 1.4 Execution flow of `build_and_test()`

[`runner_support.py`](../../verification/cocotb/runner_support.py) — the one function
every test funnels through:

1. **Toolchain prep** (`prepare_verilator_toolchain()`, idempotent, once per process):
   prepend the perl `verilator` wrapper + make shim + ucrt64/usr bin onto PATH,
   export `VERILATOR_ROOT` with forward slashes, export `COCOTB_DLL_DIRS` for the sim,
   and `bootstrap_vpi.ensure()` the static VPI `.a`.
2. **Seed pinning** — `COCOTB_SEED` defaults to `"1"`. cocotb 2.0 randomizes the resume
   order of coroutines woken by the same trigger, which can shift a driven input by a
   cycle and make phase-sensitive tests flaky; a fixed seed makes each block
   deterministic (a pass always passes). Override for bisection.
3. **Build** — `runner.build(...)` with `always=True` (always re-Verilate), the common
   build args (`-Wno-fatal -CFLAGS -O2 -LDFLAGS -lgpi -LDFLAGS -lgpilog`), and `waves=`.
4. **Run** — `runner.test(...)` launches the Verilated `.exe`, which re-imports the test
   module and runs the coroutines. Returns the path to the JUnit results XML.

Engine is `verilator` by default; a block can set `engine = "icarus"` in the manifest
(cocotb ships a Windows Icarus VPI, so no static-link workaround) — used for D-PHY blocks
whose real ISERDES/bitslip timing the Verilator stub cells can't reproduce, and as the
escape hatch if a Verilator/cocotb upgrade ever breaks the VPI build.

### 1.5 Reusable helpers (`lib/`)

The project standardized on **three DUT interface families**, one reusable driver each:

| Family | Signals | Driver in `lib/` | Example block |
|--------|---------|------------------|---------------|
| byte-beat | `s_byte_{data,keep,valid,sop,eop}` (no tready) | `byte_beat.ByteBeatDriver` | `csi2_packet_parser` |
| valid-only pixel | `in_{pixel,valid,sof,eol,eof,err}` → `out_*` | `pixel_stream` | `axis_rgb_conv3x3` |
| true AXI4-Stream | `*_t{valid,ready,data,last,user}` | `axis.{AxisMonitor,AxisSink,AxisSource}` | `axis_video_bridge` |

Plus shared infrastructure:
- `clkreset.py` — `bringup()` (single-clock active-low sync reset, 100 MHz default) and
  `bringup_dual()` (two-clock CDC bring-up, e.g. core_clk 10 ns / aclk 14 ns).
- `scoreboard.py` — `check()`, `check_eq()`, and an ordered `Scoreboard` expected-vs-actual queue.
- `verilator_unisim_stubs.sv` — behavioral Xilinx-primitive stubs for D-PHY blocks; listed
  first in those blocks' `sources`, with internal post-ISERDES registers driven via
  Verilator `--public-flat-rw`.

### 1.6 The runner, manifest, and suites

[`runner.py`](../../verification/cocotb/runner.py) is the regression front-end and the
successor to `scripts/run_dsim.ps1` / `run_regression.ps1`:

- **`manifest.toml`** registers every block with `group` (mipi_rx / img_proc / dphy / hdmi),
  `suites`, `engine`, and `path`. Suites: **`smoke`** (fast completion gate — smoke +
  one block per interface family), **`parity`** (proven to match the DSim verdict), and
  **`migrated`** (effectively "all ported blocks").
- **Selection** — a run targets explicit block names, or `--suite <name>`, or all blocks.
- **Per-block run** — each block runs in an **isolated pytest subprocess**
  (`pytest <path> -s -v -p no:cacheprovider`), so a crash in one block can't take down the run.
- **Verdicts** — `PASS` (rc 0), `NOCHK` (rc 5, pytest collected no tests), `SKIP` (engine
  binary not on PATH), `FAIL` (anything else).
- **Report** — a markdown file `verification/cocotb/_exec/regression_cocotb_<timestamp>.md`
  with a `PASS/FAIL/NOCHK/SKIP` header and a `block | group | verdict | sec | log` table.

### 1.7 How to run, and where the output goes

```powershell
.\scripts\run_cocotb.ps1 -Suite smoke            # completion gate
.\scripts\run_cocotb.ps1 csi2_packet_parser      # one block
.\scripts\run_cocotb.ps1 csi2_packet_parser -Waves   # + dump.vcd
.\scripts\run_cocotb.ps1 -List                   # list blocks
.\scripts\pytest_cocotb.ps1 verification/cocotb/<block>   # run a not-yet-registered block directly
```

`run_cocotb.ps1` resolves the ucrt64 python via `toolchain/resolve_msys2.ps1` and drives
`runner.py`. Output lands in the gitignored **`verification/cocotb/_exec/`** directory:

- **Per-block log:** `verification/cocotb/_exec/logs/<block>_<YYYYMMDD_HHMMSS>.log` (full stdout+stderr)
- **Regression report:** `verification/cocotb/_exec/regression_cocotb_<YYYYMMDD_HHMMSS>.md`

### 1.8 Why native Windows needs workarounds

Running cocotb+Verilator natively on Windows (not WSL) requires 8 workarounds, all
packaged as committed code with no absolute paths. Summary (full detail in
[`toolchain/README.md`](../../verification/cocotb/toolchain/README.md)):

| # | Problem | Fix (home) |
|---|---------|------------|
| 1 | tools not on PATH | prepend ucrt64\bin + usr\bin — `cocotb_site.prepend_path` |
| 2 | `which("verilator")` finds a `.bat` perl can't run | perl `verilator.cmd` wrapper first on PATH |
| 3 | no `make.exe` (ucrt64 has `mingw32-make.exe`) | `make.exe` shim — `runner_support._ensure_make_shim` |
| 4 | `VERILATOR_ROOT` backslashes corrupt Makefiles | forward slashes — `cocotb_site.verilator_root` |
| 5 | no Windows Verilator VPI lib shipped | compile a static `.a` — `bootstrap_vpi.ensure` |
| 6 | ucrt64 libstdc++ lacks the `-Os` `std::string` move ctor | force `-O2` — `cocotb_site.common_build_args` |
| 7 | waveforms | `waves=True` → `dump.vcd` |
| 8 | embedded Python can't load stdlib `.pyd`s | `os.add_dll_directory` — `sitecustomize.py` |

---

## Part 2 — Improvement plan

The environment is functionally complete (migration done 2026-07-01, full suite green).
The items below are quality/scale/CI hardening, ordered by value-per-effort. Each names
the concrete file to change.

### Priority table

| # | Improvement | Impact | Effort | Type |
|---|-------------|--------|--------|------|
| 1 | Fix false-green exit codes (SKIP / NOCHK) | High | Low | Correctness |
| 2 | Aggregate JUnit XML into the report | High | Low | Observability |
| 3 | Parallelize block execution (`-j`) | High | Med | Speed |
| 4 | Incremental build (skip unchanged Verilate) | High | Med | Speed |
| 5 | CI gate on `-Suite smoke` | High | Med | Process |
| 6 | Tiered suite taxonomy + wider smoke | Med | Low | Process |
| 7 | Opt-in coverage (`--coverage`) | Med | Med | Quality |
| 8 | Randomized-seed nightly | Med | Low | Quality |
| 9 | Prune / rotate `verification/cocotb/_exec/logs/` | Low | Low | Housekeeping |

### 1. Fix false-green exit codes (the highest-value fix)

`runner.py` `main()` returns non-zero **only** when a block is `FAIL`:

```python
failed = sum(1 for r in results if r["verdict"] == "FAIL")
return 1 if failed else 0
```

This means a run where every block is **SKIP** (Verilator not found — broken toolchain)
or **NOCHK** (pytest collected no tests — e.g. a test function got renamed) exits `0` and
looks green. That is a silent hole in the "green `-Suite smoke` is the completion gate"
contract. **Fix:** treat `SKIP` and `NOCHK` as non-green (or return a distinct exit code
and print a loud warning), and fail if zero blocks were selected/collected. Low effort,
directly protects the gate that everything else relies on.

### 2. Aggregate the JUnit XML into the report

`build_and_test()` already **returns the JUnit results XML path**, but `runner.py`
discards it and inspects only the subprocess return code. So the report is block-grained:
it can't say *which* `@cocotb.test()` failed without opening the log. **Fix:** capture the
returned XML path, parse per-testcase pass/fail/time, and roll it into the markdown table
(and a machine-readable `results.xml` for CI). Turns "block FAIL, go read the log" into
"testcase `long_packet` failed: CHECK FAILED payload byte 1".

### 3. Parallelize block execution

`runner.py` runs blocks strictly sequentially (`for n in names:`). With ~53 blocks each
spawning a pytest subprocess that re-Verilates from scratch, the full suite is slow and
CPU-underutilized. **Fix:** a `--jobs/-j N` flag driving a `concurrent.futures`
ProcessPool over `run_block`. Blocks are already isolated subprocesses writing to distinct
build dirs (`.build/sim/<block>`) and distinct timestamped logs, so this is low-risk;
the report aggregation stays the same.

### 4. Incremental build (skip unchanged Verilate)

`build_and_test()` builds with `always=True` — every run re-Verilates every block even
when neither the RTL nor the parameters changed. **Fix:** reuse the pattern already proven
in `bootstrap_vpi.py`: content-hash `{sources' mtimes/hashes, parameters, build_args,
verilator version}` into a per-block stamp under `.build/sim/<block>/`, and pass
`always=False` when the stamp matches. Combined with #3 this is the biggest wall-clock win
for the inner dev loop. (Keep an `--always`/`--clean` override for release runs.)

### 5. CI gate on `-Suite smoke`

The gate is currently manual. Because the toolchain is native-Windows/MSYS2, CI needs a
Windows runner with MSYS2. **Fix:** a GitHub Actions workflow on a `windows-latest` runner
that installs MSYS2, runs `verification/cocotb/toolchain/install_toolchain.ps1`, then
`run_cocotb.ps1 -Suite smoke`, and uploads `verification/cocotb/_exec/regression_cocotb_*.md` +
`cocotb_logs/` as artifacts. Depends on #1 (so the job actually fails on SKIP/NOCHK) and
benefits from #2 (machine-readable results). The `bootstrap_vpi` sdist fetch needs network
once, then caches under `.build/` — cache that dir between runs.

### 6. Tiered suite taxonomy + wider smoke

Today the suites are effectively `smoke` (4 blocks), `parity` (same 4), and `migrated`
(everything). There's no named `full`/`nightly` even though the report calls the whole run
the gate, and `smoke` has no D-PHY block (the trickiest family). **Fix:** define
`smoke ⊂ parity ⊂ full`, add at least one D-PHY block to `smoke`, and give the all-blocks
run an explicit `full` suite name so intent is legible in `manifest.toml`.

### 7. Opt-in coverage

No line/toggle coverage is collected. Verilator supports `--coverage`; cocotb can emit
Python coverage. **Fix:** a `--coverage` flag that adds Verilator coverage build args and
writes an annotated report alongside the markdown — turns "all green" into "all green and
here's what the tests never exercised", which is where the next real bugs hide.

### 8. Randomized-seed nightly

The pinned `COCOTB_SEED=1` is correct for a reproducible gate, but it also *masks*
phase-sensitivity (the exact class of flakiness the ADR calls out). **Fix:** a nightly job
that runs `full` with a rotating/random seed and reports any block that only fails off the
pinned seed — those are real RTL or testbench ordering bugs worth fixing at the source.

### 9. Prune / rotate the log directory

`verification/cocotb/_exec/logs/` accumulates timestamped logs indefinitely (already several hundred).
It's gitignored so it's harmless, but **a `--keep N` prune** (or keep only the latest run
per block) in `runner.py` keeps the directory navigable.

### Suggested sequencing

Do **#1 + #2** first (both low-effort, both protect and enrich the gate), then **#3 + #4**
together (the dev-loop speed win), then **#5** (CI, which wants #1 and #2 in place). #6–#9
are independent and can land opportunistically.

---

## Part 3 — Methodology depth (landed 2026-07-04)

Part 2 is the *infra* axis (CI / exit codes / parallelism). This part is the *methodology*
axis — depth that makes the green gate earn its green. All shipped and verified; see
[diary_20260704.md](../progress/diary_20260704.md).

| Practice | Where | Why it matters |
|----------|-------|----------------|
| **Golden self-tests** | `golden_selftest/` (engine `none`, `smoke`) | `golden.py` is the oracle for the whole `image` suite; the self-tests cross-check it against an independent re-derivation + pin its documented quirks, so a silent edit fails the gate before any sim runs. Mutation-tested. |
| **Valid-gap / backpressure** | `lib/gap.py`; `-Gap` flag; `stress` suite | The drivers fed continuous valid, so "advance on clk not in_valid" bugs were unreachable. `COCOTB_GAP` (off by default) injects random stalls; the golden is timing-invariant so the existing bit-exact scoreboards catch handshake bugs unchanged. Verified all 6 img DUTs bit-exact under gaps. |
| **`@cocotb.parametrize`** | `axis_rgb_proc_slot` | Sweeps N configs in ONE elaboration (vs img_file_uvm's per-config rebuild), each checked against the golden with coverage asserted. |
| **Functional-coverage tally** | `lib/coverage.py`; `img_coverage/` (engine `none`, `smoke`) | Stdlib dict-of-Counters (not `cocotb-coverage` — no sim-side dep). Asserts the stimulus reaches the behavioral corners (saturation rails, border/interior, threshold sides, dither modes, gap bins). |
| **Bit-exact goldens for conv5x5_sep / DoG / cascade** | `golden.py`, `dut_registry.py`, `axis_rgb_dog`, `axis_rgb_cascade` | The last img blocks with only tolerance checks now have RTL-exact goldens; conv5x5_sep is a 6th img_file_uvm DUT. DoG/cascade use a two-frame steady-state compare to skip the cold-start FIFO transient. |
| **ruff + pyproject + markers** | `verification/cocotb/pyproject.toml` | Dev-only lint (not in `requirements.lock`), mypy opt-in on `lib/`, registered suite markers. |

New knobs: `run_cocotb.ps1 -Suite stress -Gap sparse|burst|adversarial` (re-run the directed
suite under randomized handshake timing); `run_cocotb.ps1 golden_selftest` / `img_coverage`
(sim-free gate blocks). Both `golden_selftest` and `img_coverage` are `engine = "none"` blocks
so they run under any interpreter and gate in `smoke` without Verilator.
