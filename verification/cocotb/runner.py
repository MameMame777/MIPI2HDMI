"""Regression front-end for the cocotb + Verilator verification environment.

Reads ``manifest.toml``, selects blocks (by name or ``--suite``), runs each block's
``test_<block>.py`` in an isolated pytest subprocess, and writes a markdown report in the
format the project already uses (``PASS/FAIL/NOCHK/SKIP`` header, ``block | group | verdict
| sec | log`` rows). This is the successor to ``scripts/run_dsim.ps1`` /
``run_regression.ps1``; a green ``--suite smoke`` is the new completion gate.

Usage (via the ucrt64 python; see scripts/run_cocotb.ps1)::

    python verification/cocotb/runner.py --suite smoke
    python verification/cocotb/runner.py csi2_packet_parser --waves
    python verification/cocotb/runner.py --list
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
import tomllib
from pathlib import Path

import cocotb_site as cs

MANIFEST = cs.COCOTB_DIR / "manifest.toml"
REPORT_DIR = cs.COCOTB_DIR / "_exec"          # gitignored cocotb run outputs (logs + reports)
LOG_DIR = REPORT_DIR / "logs"


def load_manifest() -> dict:
    with open(MANIFEST, "rb") as fh:
        data = tomllib.load(fh)
    default_engine = data.get("defaults", {}).get("engine", "verilator")
    blocks = {}
    for name, meta in data.get("blocks", {}).items():
        meta = dict(meta)
        meta.setdefault("engine", default_engine)
        meta.setdefault("group", "")
        meta.setdefault("suites", [])
        blocks[name] = meta
    return blocks


def select(blocks: dict, names: list[str], suite: str | None) -> list[str]:
    if names:
        missing = [n for n in names if n not in blocks]
        if missing:
            raise SystemExit(f"Unknown block(s): {', '.join(missing)}")
        return names
    if suite:
        return [n for n, m in blocks.items() if suite in m.get("suites", [])]
    return list(blocks)


def _engine_available(engine: str) -> bool:
    if engine == "none":
        return True                    # pure-Python block (e.g. sim-free golden self-tests)
    if engine == "verilator":
        return shutil.which("verilator") is not None
    if engine == "icarus":
        return shutil.which("iverilog") is not None
    return False


def run_block(name: str, meta: dict, timestamp: str, waves: bool,
              hermetic: bool = False, gap: str | None = None) -> dict:
    path = cs.REPO_ROOT / meta["path"]
    log_path = LOG_DIR / f"{name}_{timestamp}.log"
    engine = meta.get("engine", "verilator")
    if not _engine_available(engine):
        return {"name": name, "group": meta["group"], "verdict": "SKIP",
                "sec": 0.0, "log": str(log_path), "reason": f"{engine} not found"}

    env = dict(os.environ)
    if hermetic:
        # Suite runs are the registered deterministic regression: scrub block-config env
        # vars (IMG_* -- img_file_uvm) so stale interactive-session state cannot silently
        # narrow the parametrization (e.g. leftover IMG_DUT drops 4 of 5 DUTs while the
        # suite stays green) or corrupt it (leftover IMG_SELFTEST_CORRUPT=1 -> false red).
        # Explicit block-name runs keep the pass-through (documented env-driven use).
        env = {k: v for k, v in env.items() if not k.startswith("IMG_")}
    if gap:
        # Re-run the directed stimulus under randomized valid-gap / backpressure timing.
        # Read by lib/gap.default_gap_policy(); survives the IMG_* scrub above (not IMG_-prefixed).
        env["COCOTB_GAP"] = gap
    if waves:
        env["COCOTB_WAVES"] = "1"
    # -s (no capture) lets the cocotb sim's full output -- the TESTS=/PASS=/FAIL= regression
    # table, per-@cocotb.test() SIM TIME rows, dut._log messages, seed/banner, and (on
    # failure) the assertion + traceback -- flow into the captured stdout we write to the log.
    # Without it, `-q` discards all of that on a passing block (log = just "1 passed").
    cmd = [sys.executable, "-m", "pytest", str(path), "-s", "-v", "-p", "no:cacheprovider"]
    start = time.perf_counter()
    proc = subprocess.run(cmd, cwd=cs.REPO_ROOT, env=env,
                          capture_output=True, text=True)
    secs = time.perf_counter() - start
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path.write_text(proc.stdout + "\n" + proc.stderr, encoding="utf-8")

    if proc.returncode == 0:
        verdict = "PASS"
    elif proc.returncode == 5:        # pytest: no tests collected
        verdict = "NOCHK"
    else:
        verdict = "FAIL"
    return {"name": name, "group": meta["group"], "verdict": verdict,
            "sec": secs, "log": str(log_path), "reason": ""}


def write_report(results: list[dict], timestamp: str) -> Path:
    counts = {k: 0 for k in ("PASS", "FAIL", "NOCHK", "SKIP")}
    for r in results:
        counts[r["verdict"]] = counts.get(r["verdict"], 0) + 1
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    report = REPORT_DIR / f"regression_cocotb_{timestamp}.md"
    lines = [
        f"# cocotb + Verilator regression {timestamp}",
        "",
        f"PASS {counts['PASS']} / FAIL {counts['FAIL']} / "
        f"NOCHK {counts['NOCHK']} / SKIP {counts['SKIP']}",
        "",
        "| block | group | verdict | sec | log |",
        "|-------|-------|---------|-----|-----|",
    ]
    for r in results:
        lines.append(
            f"| {r['name']} | {r['group']} | {r['verdict']} | "
            f"{r['sec']:.1f} | {Path(r['log']).name} |"
        )
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="cocotb + Verilator regression runner")
    ap.add_argument("blocks", nargs="*", help="block name(s); default: all or --suite")
    ap.add_argument("--suite", help="run all blocks in this suite (e.g. smoke, parity)")
    ap.add_argument("--waves", action="store_true", help="dump waveforms (dump.vcd)")
    ap.add_argument("--gap", choices=("none", "sparse", "burst", "adversarial"),
                    help="re-run under randomized valid-gap/backpressure timing (COCOTB_GAP)")
    ap.add_argument("--list", action="store_true", help="list blocks and exit")
    args = ap.parse_args(argv)

    blocks = load_manifest()
    if args.list:
        for n, m in blocks.items():
            print(f"{n:28s} group={m['group']:10s} engine={m['engine']:9s} "
                  f"suites={','.join(m['suites'])}")
        return 0

    names = select(blocks, args.blocks, args.suite)
    if not names:
        print("No blocks selected.", file=sys.stderr)
        return 2

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    results = []
    for n in names:
        print(f"=== {n} ===", flush=True)
        r = run_block(n, blocks[n], timestamp, args.waves,
                      hermetic=args.suite is not None, gap=args.gap)
        tag = r["verdict"] + (f" ({r['reason']})" if r["reason"] else "")
        print(f"    {tag}  {r['sec']:.1f}s  {r['log']}", flush=True)
        results.append(r)

    report = write_report(results, timestamp)
    print(f"\nReport: {report}")
    failed = sum(1 for r in results if r["verdict"] == "FAIL")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
