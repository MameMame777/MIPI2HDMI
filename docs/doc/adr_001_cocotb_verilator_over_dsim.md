# ADR-001: cocotb + Verilator (native Windows) replaces DSim for RTL verification

- Status: **Accepted & migration COMPLETE** (2026-07-01). All 52 DSim blocks ported to
  cocotb + Verilator (53 cocotb blocks incl. `_smoke`); full suite green (PASS 53 / FAIL 0).
  `verification/tb/` and `scripts/run_dsim.ps1` deleted (cutover done).
- Supersedes: the DSim direct-invocation flow (`verification/tb/`, `scripts/run_dsim.ps1`)

## Context

RTL verification ran on **DSim 2026** via 52 procedural SystemVerilog testbenches
(`verification/tb/tb_<block>.sv` + `<block>.f`), driven by `scripts/run_dsim.ps1`. Problems:

- **Licensing.** DSim is a licensed Altair tool; this project has repeatedly hit
  entitlement/usage-meter blockers (see memory `dsim-usagemeter-sandbox-blocker`; a bounded
  DSim run on 2026-07-01 failed immediately on a license error). Verification cannot depend
  on a tool that may not run.
- **Reproducibility.** The flow was Windows-only by accident, not by design, with no pinned
  toolchain a contributor could stand up.

The maintainer had a working recipe (Qiita) for running **cocotb + Verilator natively on
Windows** via MSYS2 ucrt64. cocotb+Verilator is open-source, license-free, and Python-driven.

## Decision

1. **Adopt cocotb 2.0.1 + Verilator 5.048** on native Windows (MSYS2 ucrt64, no WSL), driven
   by the `cocotb_tools.runner` API. Pin versions in `verification/cocotb/requirements.lock`.
2. **Single toolchain, Verilator-primary.** Verilator for all blocks; **Icarus Verilog** (which
   cocotb supports natively on Windows) as a per-block fallback, selected via `engine =
   "icarus"` in `manifest.toml`.
3. **Xilinx-primitive D-PHY strategy.** Promote the existing behavioral primitive stubs
   (`dphy_hs_byte_probe_sim_prims.sv`) to a committed
   `verification/cocotb/lib/verilator_unisim_stubs.sv` and list it first in those blocks'
   sources. Where the stubs cannot reproduce real ISERDES/bitslip serialization timing, mark
   that specific block `engine = "icarus"` — a one-line decision, not an architecture fork.
4. **Decommission DSim** at cutover, once all blocks are migrated. **Done 2026-07-01**:
   `verification/tb/` and `scripts/run_dsim.ps1` deleted (recoverable from git history).
5. **Naming exception to `base-instructions.md` §5.** §5 mandates `tb_<module>.sv` +
   `<module>.f` + `run_<module>_test.ps1`. The cocotb analogue is
   `verification/cocotb/<module>/test_<module>.py` + an entry in `manifest.toml`, run by
   `scripts/run_cocotb.ps1` / `verification/cocotb/runner.py`. A green `-Suite smoke` is the
   new completion gate (the old `run_regression.ps1 -Suite smoke` was aspirational and never
   existed). §5 is updated at Phase-5 cutover.

## Consequences

- **Windows workarounds.** Native cocotb+Verilator on Windows needs 8 workarounds (7 from the
  article + an embedded-Python `.pyd` DLL-search fix discovered here). All are packaged as
  committed code with no absolute paths; see `verification/cocotb/toolchain/README.md`.
- **Static-VPI maintenance.** cocotb ships no Windows Verilator VPI lib; `bootstrap_vpi.py`
  compiles it from the version-matched cocotb sources with content-hash caching so
  cocotb/Verilator upgrades self-heal. Icarus is the always-works escape hatch.
- **2-state vs 4-state.** Verilator is 2-state; some X-dependent DSim checks may need
  re-expression as post-reset value checks (the RTL already mandates explicit reset values).
- **Lint strictness.** Verilator lint is stricter than DSim; the build uses `-Wno-fatal`
  (warnings visible, non-fatal). Real RTL issues get fixed at the source.
- **Parity.** Ports replicate each DSim TB's stimulus + checks 1:1. Because DSim runtime is
  license-blocked on this host, parity is established by construction plus green cocotb runs.
  All 52 blocks migrated (37 easy/dual-clock + 12 Xilinx-primitive/E2E + 3 seeds). The
  Xilinx-primitive D-PHY blocks drive the DUT's *internal* post-ISERDES registers via
  Verilator `--public-flat-rw` (the DSim TBs did the same hierarchically) with the behavioral
  primitive stubs — no real deserialization model needed, no Icarus fallback used.
- **Trust-but-verify was load-bearing.** Sub-agent self-reports were re-run independently;
  one block (`dphy_lane1_trace`) self-reported PASS but *hung for hours* under the runner due
  to a 1-picosecond clock-driver busy-loop (`while True: await Timer(1, unit="ps")`). Fixed to
  a real anti-phase `Clock`. Lesson: never drive a clock/reset with a sub-timestep polling
  loop; always independently re-run agent-produced tests.
- **Verification-base follow-on (2026-07-01):** for structured testbenches, adopted **pyuvm**
  (real UVM in pure Python; 4.0.1 supports cocotb 2.0) rather than hand-rolling a UVM clone or
  adding a fragile C-extension. It is an **additive** layer (`verification/cocotb/lib/uvm/`);
  the 53 plain-lib tests are untouched and the pyuvm drivers reuse the plain ones by
  composition. `pyuvm==4.0.1` pinned in `requirements.lock`.

## Alternatives considered

- **Keep DSim (hybrid).** Rejected: leaves the licensing dependency and two toolchains.
- **Port all 52 blocks in one sweep.** Rejected: the toolchain was unproven in-repo; infra +
  a parity subset per interface family de-risks before bulk porting.
- **WSL cocotb+Verilator.** Rejected: the maintainer's environment and PYNQ/Vivado flows are
  native Windows; a WSL split would fragment the toolchain.
