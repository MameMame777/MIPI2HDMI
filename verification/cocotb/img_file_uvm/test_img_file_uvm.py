"""Image-file-driven pyuvm verification of the img_proc slot family.

An arbitrary user image (PNG/JPEG/BMP/... via the repo-root Pillow venv, or .ppm/.pgm
directly) is streamed through a selectable slot DUT; the output frame is captured by a
UVM monitor, saved as PPM+PNG, and compared PIXEL-EXACTLY against a software golden
model (the same filter applied to the same image in Python, transliterated from the RTL
including frame-border behaviour). No image -> a built-in deterministic test pattern,
so the registered regression needs no external files.

Dual-role module (project convention):
  host (pytest)  -- prepare_input() + per-DUT build_and_test(); parametrized over
                    IMG_DUT (unset = all five DUTs)
  sim (Verilator)-- the pyuvm env/test below, configured via IMG_* env vars
                    (inherited by the sim process; see img_config.py for the surface)

Typical runs:
  .\\scripts\\run_image_test.ps1 -Image photo.png -Dut conv3x3 -Kernel sobel_x
  $env:IMG_FILE='photo.jpg'; $env:IMG_DUT='dither'
  .\\scripts\\pytest_cocotb.ps1 verification/cocotb/img_file_uvm

Outputs land in verification/cocotb/_exec/img_file_uvm/<dut>_<timestamp>/:
  input.png/.ppm, output.png/.ppm, expected.png, run_info.txt (+ mismatches.txt on fail).
"""
from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

import cocotb
import pytest
import pyuvm
from cocotb.triggers import ClockCycles
from pyuvm import ConfigDB

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parents[0]))   # verification/cocotb (lib, runner_support)
sys.path.insert(0, str(_HERE))              # block-local modules

import image_io  # noqa: E402
from dut_registry import DUTS  # noqa: E402
from img_config import ImgConfig  # noqa: E402
from lib.uvm import (  # noqa: E402
    FrameInputAgent, FrameScoreboard, ImageFrameItem, ItemsSequence, PixelOutputAgent,
    UvmEnv, UvmTest,
)

REPO_ROOT = _HERE.parents[2]


# =============================================================================
# sim side (runs inside the Verilator process under pyuvm)
# =============================================================================

class ImgEnv(UvmEnv):
    def build_phase(self):
        self.in_agent = FrameInputAgent("in_agent", self)
        self.out_agent = PixelOutputAgent("out_agent", self)
        self.sb = FrameScoreboard("sb", self)

    def connect_phase(self):
        self.out_agent.monitor.ap.connect(self.sb.analysis_export)


def _write_outputs(out_dir: Path, observed, golden, width, height, frames,
                   desc: str, log) -> None:
    """Save captured + expected images. Runs BEFORE check_phase, so the output image is
    written on pass AND fail. A short capture is zero-padded (count fails the check)."""
    out_dir.mkdir(parents=True, exist_ok=True)
    fsz = width * height
    total = fsz * frames
    obs = list(observed[:total]) + [0] * max(0, total - len(observed))
    last = (frames - 1) * fsz
    image_io.write_ppm(out_dir / "output.ppm", obs[last:last + fsz], width, height)
    image_io.write_png(out_dir / "output.png", obs[last:last + fsz], width, height)
    image_io.write_png(out_dir / "expected.png", golden[last:last + fsz], width, height)
    if frames > 1:
        for i in range(frames):
            image_io.write_png(out_dir / f"output_f{i}.png",
                               obs[i * fsz:(i + 1) * fsz], width, height)
    (out_dir / "run_info.txt").write_text(
        f"{desc}\nsize: {width}x{height} frames={frames}\n"
        f"observed beats: {len(observed)} / expected {total}\n", encoding="utf-8")
    log.info("IMG OUT: %s", out_dir / "output.png")
    log.info("IMG EXP: %s", out_dir / "expected.png")


@pyuvm.test()
class ImgFileUvmTest(UvmTest):
    clock_specs = [("clk", "rst_n", 10.0)]

    def build_phase(self):
        # slot-contract default signal names (in_pixel/in_valid/... , out_pixel/...)
        ConfigDB().set(None, "*", "pixel_in_cfg", {"clk": "clk"})
        ConfigDB().set(None, "*", "pixel_out_cfg", {"clk": "clk"})
        self.env = ImgEnv("env", self)

    async def stimulus(self):
        cfg = ImgConfig.from_env()
        spec = DUTS[os.environ["IMG_DUT_ACTIVE"]]
        out_dir = Path(os.environ["IMG_RUN_DIR"])
        pixels, w, h = image_io.read_ppm(os.environ["IMG_PPM"])

        rc = spec.resolve(cfg)
        await spec.drive_cfg(cocotb.top, rc)
        desc = f"dut={spec.name} {spec.describe(rc)}"
        self.logger.info("img_file_uvm: %s size=%dx%d frames=%d", desc, w, h, cfg.frames)

        stream = pixels * cfg.frames
        golden = spec.golden(stream, w, h, rc)
        expected = list(golden)
        if cfg.selftest_corrupt:
            expected[len(expected) // 2] ^= 0x01   # scoreboard MUST go red
        self.env.sb.set_expected(expected, w, h, cfg.frames)
        self.env.sb.report_dir = out_dir

        items = [ImageFrameItem(pixels=pixels, width=w) for _ in range(cfg.frames)]
        await ItemsSequence("frame_seq", items=items).start(self.env.in_agent.seqr)

        # drain the fixed pipeline latency into the scoreboard (budget-bounded so a dead
        # DUT still reaches the image dump + a clean count failure)
        clk = self.clock_pairs[0][0]
        total = w * h * cfg.frames
        waited = 0
        while len(self.env.sb.beats) < total and waited < 2 * w + 512:
            await ClockCycles(clk, 32)
            waited += 32

        _write_outputs(out_dir, self.env.sb.observed_pixels(), golden,
                       w, h, cfg.frames, desc, self.logger)


# =============================================================================
# host side (pytest entry: input prep + per-DUT build)
# =============================================================================

def _selected_duts():
    name = os.environ.get("IMG_DUT")
    if not name:
        return list(DUTS)
    if name not in DUTS:
        raise ValueError(f"IMG_DUT {name!r}: choose from {sorted(DUTS)}")
    return [name]


def _converter_python(cfg: ImgConfig) -> Path:
    py = Path(cfg.python) if cfg.python else REPO_ROOT / ".venv" / "Scripts" / "python.exe"
    if not py.is_file():
        raise RuntimeError(
            f"Image converter interpreter not found: {py}\n"
            "Arbitrary image formats need the repo-root CPython venv with Pillow "
            "(set IMG_PYTHON to override), or supply a binary .ppm/.pgm via IMG_FILE.")
    return py


def prepare_input(cfg: ImgConfig, run_dir: Path):
    """Materialise the input frame as <run_dir>/input.ppm (+ input.png preview).
    Returns (ppm_path, width, height)."""
    run_dir.mkdir(parents=True, exist_ok=True)
    ppm = run_dir / "input.ppm"
    if cfg.file is None:
        px, w, h = image_io.make_test_pattern()
        image_io.write_ppm(ppm, px, w, h)
    else:
        src = Path(cfg.file)
        if not src.is_file():
            raise FileNotFoundError(f"IMG_FILE not found: {src}")
        if src.suffix.lower() in (".ppm", ".pgm"):
            px, w, h = image_io.read_ppm(src)
            px, w, h = image_io.decimate(px, w, h, cfg.max_w, cfg.max_h)
            image_io.write_ppm(ppm, px, w, h)
        else:
            py = _converter_python(cfg)
            conv = REPO_ROOT / "scripts" / "img_to_ppm.py"
            res = subprocess.run(
                [str(py), str(conv), str(src), str(ppm),
                 "--max-width", str(cfg.max_w), "--max-height", str(cfg.max_h)],
                capture_output=True, text=True)
            if res.returncode != 0:
                raise RuntimeError(
                    f"img_to_ppm failed (rc={res.returncode}) for {src}:\n"
                    f"{res.stderr}\nFallback: supply a binary .ppm/.pgm via IMG_FILE.")
            px, w, h = image_io.read_ppm(ppm)
    if w < 8 or h < 8:
        raise ValueError(f"image too small ({w}x{h}): need at least 8x8")
    image_io.write_png(run_dir / "input.png", px, w, h)
    return ppm, w, h


@pytest.mark.parametrize("dut_name", _selected_duts())
def test_img_file_uvm(dut_name):
    from runner_support import build_and_test

    cfg = ImgConfig.from_env()
    spec = DUTS[dut_name]
    base = Path(cfg.out_dir) if cfg.out_dir else _HERE.parents[0] / "_exec" / "img_file_uvm"
    # absolutize: the sim process runs with cwd=test_dir (cocotb runner), not the host cwd,
    # so relative paths in the env handoff would break every sim-side open()
    run_dir = (base / f"{dut_name}_{time.strftime('%Y%m%d_%H%M%S')}").resolve()
    ppm, w, h = prepare_input(cfg, run_dir)

    # handoff to the sim process (env is inherited; IMG_* config vars pass through as-is)
    os.environ["IMG_PPM"] = str(ppm.resolve())
    os.environ["IMG_RUN_DIR"] = str(run_dir)
    os.environ["IMG_DUT_ACTIVE"] = dut_name

    params = {"ENABLE": 1}
    if spec.has_line_pixels:
        params["LINE_PIXELS"] = w
    print(f"[img_file_uvm] dut={dut_name} input={cfg.file or 'builtin pattern'} "
          f"{w}x{h} -> {run_dir}")
    build_and_test(
        block=f"img_file_uvm_{dut_name}",
        sources=list(spec.sources),
        toplevel=spec.toplevel,
        test_module="test_img_file_uvm",
        test_dir=_HERE,
        parameters=params,
    )
