# cocotb + Verilator verification environment

License-free, Python-driven RTL verification for MIPI2HDMI — the replacement for the DSim
environment in `verification/tb/`. Runs **cocotb 2.0.1 + Verilator 5.048 natively on
Windows** (MSYS2 ucrt64, no WSL). Setup and the Windows workarounds are documented in
[`toolchain/README.md`](toolchain/README.md).

## Status

**Migration complete (2026-07-01).** All 52 DSim blocks are ported to cocotb + Verilator (53
cocotb blocks incl. `_smoke`); the full suite is green (`PASS 53 / FAIL 0`). DSim,
`verification/tb/`, and `scripts/run_dsim.ps1` have been removed — cocotb + Verilator is the
sole verification path. `manifest.toml` is the block registry.

Three DUT interface families, one reusable driver each in `lib/`:

| Family | Signals | Example block |
|--------|---------|---------------|
| byte-beat | `s_byte_{data,keep,valid,sop,eop}` (no tready) | `csi2_packet_parser` |
| valid-only pixel | `in_{pixel,valid,sof,eol,eof,err}` → `out_*` (no tready) | `axis_rgb_conv3x3` |
| true AXI4-Stream | `*_t{valid,ready,data,last,user}` | `axis_video_bridge` |

The Xilinx-primitive D-PHY blocks (`dphy_hs_byte_probe_*`, `dphy_lane1_trace`, etc.) build the
DUT itself as toplevel with the behavioral primitive stubs (`lib/verilator_unisim_stubs.sv`
or a local `<block>_stubs.sv`) and drive internal post-ISERDES registers via Verilator
`--public-flat-rw`; the large E2E blocks build a small harness top over the full pipeline.

## Run

```powershell
.\scripts\run_cocotb.ps1 -Suite smoke        # completion gate (all Phase-1 blocks)
.\scripts\run_cocotb.ps1 csi2_packet_parser  # one block
.\scripts\run_cocotb.ps1 <block> -Waves      # + dump.vcd
.\scripts\run_cocotb.ps1 -List               # list registered blocks
.\scripts\pytest_cocotb.ps1 verification/cocotb/<block>   # run a not-yet-registered block
```

Tests must run under the MSYS2 ucrt64 python — the scripts select it automatically; a
plain `pytest` under an MSVC/venv python is rejected by `conftest.py` (VPI ABI mismatch).

## Tests in detail

### The two-process model

A test is an ordinary **pytest** test that drives a hardware simulation. The same
`test_<block>.py` file is imported in **two** processes, and this dual role is the key
thing to understand:

```
pytest (ucrt64 python)                 ← host process; collects test_<block>()
  └─ runner_support.build_and_test()   ← toolchain setup + cocotb runner
       └─ Verilator                    ← compiles RTL → native .exe with embedded Python
            └─ <block>.exe             ← SIM process; re-imports test_<block>.py and runs
                                          the @cocotb.test() coroutines against the DUT
```

| Role | Runs in | Which function |
|------|---------|----------------|
| **pytest entry point** | host `pytest` process | `def test_<block>()` — builds + launches the sim |
| **testbench** | Verilated `.exe` (embedded Python) | the `@cocotb.test()` coroutines — drive/observe the DUT |

### Anatomy of a test file

Every block port has the same three parts (reference:
[`csi2_packet_parser/test_csi2_packet_parser.py`](csi2_packet_parser/test_csi2_packet_parser.py)):

1. **Model coroutines / monitors** — the DSim TB's `initial`/`always_ff` blocks become
   `async def` coroutines started with `cocotb.start_soon(...)` (e.g. the parser's ECC
   responder `initial` → `ecc_responder(dut)`; the `always_ff` logger → the `Capture` monitor).
2. **`@cocotb.test()` coroutines** — the scenarios, each a fresh-reset run (the parser's
   `short_packet` / `long_packet` / `truncated_packet`). A test **passes by returning
   normally** and **fails by raising**.
3. **`def test_<block>()`** — the pytest entry; calls `runner_support.build_and_test(...)`
   with the RTL `sources`, `toplevel`, `test_module`, and RTL `parameters`.

DSim-testbench → cocotb translation cheatsheet:

| DSim SystemVerilog | cocotb Python |
|--------------------|---------------|
| `initial` stimulus/model block | `async def` + `cocotb.start_soon()` |
| `always_ff` logger | monitor coroutine sampling on `RisingEdge` |
| `check_condition(...)` | `check(cond, msg)` from `lib/scoreboard.py` |
| `#N ms` watchdog | `@cocotb.test(timeout_time=N, timeout_unit="ms")` |
| TB `localparam` | `parameters={...}` on `build_and_test` |
| `$display("TEST PASSED"); $finish` | coroutine returns normally |

### Interface-family drivers (`lib/`)

Pick the driver matching the DUT's handshake; do **not** hand-roll signal wiggling:

| Family | Signals | Helper | Example |
|--------|---------|--------|---------|
| byte-beat | `s_byte_{data,keep,valid,sop,eop}` (no tready) | `byte_beat.ByteBeatDriver` (send `Beat(...)`) | `csi2_packet_parser` |
| valid-only pixel | `in_{pixel,valid,sof,eol,eof,err}` → `out_*` | `pixel_stream` | `axis_rgb_conv3x3` |
| true AXI4-Stream | `*_t{valid,ready,data,last,user}` | `axis.{AxisMonitor,AxisSink,AxisSource}` | `axis_video_bridge` |

Shared infra: `clkreset.bringup()` (single-clock active-low sync reset, 100 MHz default) and
`bringup_dual()` (two-clock CDC bring-up, e.g. core_clk 10 ns / aclk 14 ns);
`scoreboard.{check,check_eq,Scoreboard}`. `check()` raises
`AssertionError("CHECK FAILED: ...")` — the same `CHECK FAILED` token the DSim TBs emitted,
so logs stay grep-compatible.

### Determinism, timeouts, waves

- **Seed** — `COCOTB_SEED` defaults to `"1"`. cocotb 2.0 randomizes the resume order of
  coroutines woken by the same trigger, which can shift a driven input by a cycle and make
  phase-sensitive tests flaky; a fixed seed makes each block reproducible. Override
  (`$env:COCOTB_SEED=...`) for bisection.
- **Timeouts** — annotate long scenarios with `@cocotb.test(timeout_time=N, timeout_unit="ms")`;
  a hung test fails instead of blocking the suite. Never drive a clock/reset with a
  sub-timestep polling loop (`while True: await Timer(1, unit="ps")`) — use a real `Clock`
  (one such loop once hung the runner for hours).
- **Waves** — `-Waves` (or `COCOTB_WAVES=1`) dumps `dump.vcd` into the **block's test dir**
  (`verification/cocotb/<block>/dump.vcd`). Gotcha: enabling waves on a block previously
  built without traces can fail the compile with `'class Vtop__Syms' has no member named
  '__Vm_baseCode'` (stale non-trace artifacts) — delete `.build/sim/<block>/` and re-run.
- **Build** — every run re-Verilates (`always=True`); D-PHY/E2E blocks build with
  behavioral primitive stubs and expose internal regs via Verilator `--public-flat-rw`.

### Suites and verdicts

`runner.py` selects blocks by name or `--suite`, runs each in an **isolated pytest
subprocess**, and writes `_exec/regression_cocotb_<ts>.md`. Suites (in `manifest.toml`):

| Suite | Meaning |
|-------|---------|
| `smoke` | fast completion gate — `_smoke` + one block per interface family |
| `parity` | proven to match the DSim verdict during migration |
| `migrated` | all ported blocks (effectively the full run) |

Verdicts: **PASS** (rc 0) · **FAIL** (test failed / build error) · **NOCHK** (rc 5, pytest
collected no tests) · **SKIP** (engine binary not on PATH). Note the current runner exits
non-zero only on FAIL — an all-SKIP or NOCHK run still exits 0 (see the improvement plan in
the guide below).

### Output

- Per-block log: `_exec/logs/<block>_<YYYYMMDD_HHMMSS>.log` (full stdout+stderr)
- Regression report: `_exec/regression_cocotb_<YYYYMMDD_HHMMSS>.md`
- Waveforms (with `-Waves`): `verification/cocotb/<block>/dump.vcd`

**Deeper dive:** the execution model, `build_and_test()` internals, the 8 Windows
workarounds, and a prioritized improvement plan are in
[`../../docs/doc/cocotb_python_test_guide.md`](../../docs/doc/cocotb_python_test_guide.md).

## Layout

```
verification/cocotb/
  cocotb_site.py        toolchain path resolution (MSYS2_ROOT -> derive -> probe)
  bootstrap_vpi.py      builds the static cocotb VPI lib for Verilator (WA#5)
  runner_support.py     build_and_test(): toolchain setup + cocotb runner
  conftest.py           asserts ucrt64 python
  sitecustomize.py      sim-process DLL-dir fix (WA#8)
  runner.py             regression front-end (manifest -> suite -> markdown report)
  manifest.toml         block registry (successor to the .f filelists)
  requirements.lock     pinned deps (cocotb==2.0.1)
  toolchain/            install/setup scripts, perl verilator wrapper, README
  lib/                  clkreset, scoreboard, byte_beat, pixel_stream, axis, unisim stubs
  <block>/test_<block>.py
```

## Writing a new block port

1. Create `verification/cocotb/<block>/test_<block>.py`.
2. Write `@cocotb.test()` coroutines using `lib/` helpers (`bringup`, the family driver,
   `check`). Translate: SV `initial` model → `cocotb.start_soon` coroutine; `always_ff`
   logger → monitor coroutine; `check_condition` → `check()`; `#Nms` watchdog →
   `@cocotb.test(timeout_time=N, timeout_unit="ms")`; TB localparams → `parameters=`.
3. Add a `def test_<block>()` that calls `runner_support.build_and_test(...)` with the RTL
   `sources`, `toplevel`, and `parameters`.
4. Register it in `manifest.toml` (group, suites, path).
5. `.\scripts\run_cocotb.ps1 <block>`.

D-PHY blocks that instantiate Xilinx primitives: list `lib/verilator_unisim_stubs.sv` first
in `sources`; if real ISERDES/bitslip timing is needed, set `engine = "icarus"` for that
block in the manifest.
