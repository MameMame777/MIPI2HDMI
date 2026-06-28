# MIPI2HDML

🌐 **English** | **[日本語](README_ja.md)**

Full-pipeline FPGA implementation from MIPI CSI-2 reception to HDMI output (custom RTL + PYNQ Python).

- **Target**: Digilent Zybo Z7-20 (Zynq xc7z020clg400-1)
- **Camera**: Pcam 5C with OV5640 (2-lane MIPI CSI-2)
- **Image format**: RGB565 (chip `0x4300=0x6F`, FPGA `expected_dt=0x22`)
- **Resolution / frame rate**: VGA 640×480 **30fps** (PLL mult=96 → link 384MHz/768Mbps → byte_clk 96MHz)
- **Output**: HDMI (AXI4-Stream Video → AXI VDMA → HDMI TX)

> **Status**: The **full pipeline runs live** from camera to HDMI monitor.
> **Genuine 30fps** (the proper approach of raising PCLK via the PLL rather than cutting VTS, keeping full vblank/exposure)
> displays continuously at full 480 lines, no banding, CRC 0%. `fs=fe=30 / std~76`. The claim that "30fps is
> unroutable on the Z-7020" is wrong: re-constraining the XDC to 384MHz and rebuilding converges at byte_clk 96MHz
> with **WNS=+0.112** (no additional resources). Full design details in
> [RTL Design Specification](docs/doc/rtl_design_spec.md).
>
> **30fps lock note**: The HW deterministic lock FSM is tuned for byte_clk 84MHz (17fps), and at
> 96MHz it produces a bogus lock (fs=0 / white screen), so at 30fps use the **software lock_mode
> (`--hw-lock 0`)** (the default for `camera_hdmi_demo` / `camera_repl`). The banding fix scales with
> byte_clk as **K=14** (17fps is K=8). Re-tuning the FSM for 96MHz is a TODO.

> **Reproduction steps**: The complete procedure for build → DSim verification → board deployment → capture / HDMI is in
> [REPRODUCE.md](REPRODUCE.md).

---

## Pipeline Architecture

```text
Pcam 5C (OV5640)  ─ PLL mult=96 → link 384MHz (768Mbps/lane) → 30fps
  └─ MIPI 2-lane HS (continuous clock 0x4800=0x14)
       └─ dphy_hs_byte_probe         IBUFDS/BUFIO/BUFR(÷4 → byte_clk 96MHz)/ISERDES/IDELAY, SoT(0xB8)+bitslip
       │    + dphy_hwlock_fsm         HW deterministic lock (8x8 bitslip sweep + /4 re-roll, refclk_200) ※ bogus at 96MHz → use software lock
       │    + dphy_lane_supervisor    Digilent-derived clock-lane management (opt-in)
       │    + settle-blank K=14       excludes burst-head settle garbage from the SoT window (banding fix, scales with byte_clk: 30fps=14 / 17fps=8)
       └─ byte_to_core_cdc            byte_clk → core_clk (Gray FIFO)
            └─ csi2_packet_parser     header/payload/CRC separation
                 └─ csi2_header_ecc / csi2_payload_crc
                      └─ csi2_vcdt_filter        VC/DT filter (expected_dt=0x22)
                           └─ csi2_frame_state    SOF/EOF management + SOF synthesis + force-480
                                └─ rgb565_gray_unpack  byte → RGB888 pixel
                                     └─ axis_video_bridge   AXI4-Stream out
                                          └─ AXI VDMA → HDMI TX
```

An in-bitstream SCCB sequencer (`ov5640_sccb_init_probe`) initializes the OV5640.
Runtime control is performed from PYNQ Python via AXI GPIO. For the complete per-module specification, see
[RTL Design Specification](docs/doc/rtl_design_spec.md).

> **Colorization + 3-stage image-processing pipeline**: The `rgb565_gray_unpack` block above is colorized to **true RGB888**
> (`{R,G,B}` via `RGB_OUT`), and a **3-stage runtime image-processing pipeline** is inserted ahead of `axis_video_bridge`.
> **Live switching with no rebuild required** (controlled via SCCB reserved-page `0xFE` coefficients + idelay GPIO ops):
>
> ```text
> video → PRE (3×3 spatial denoise + point ops) → MID (convolution) → POST (point ops) → DITHER → capture/HDMI
> ```
>
> **PRE — `axis_rgb_prefilter` (3×3, line-buffered)**
>
> - passthrough / invert / grayscale / threshold (binarization) / R/G/B (point ops)
> - **median 3×3** (impulse/salt-and-pepper noise removal), **gaussian 3×3** (blur) = spatial denoise
>   — `cam.denoise('median'|'gaussian')` / `cam.pre_op(n)` (median is a verified 19-CAS network)
>
> **MID — convolution**
>
> - **arbitrary 3×3** edge / emboss / sharpen / arbitrary — `cam.k(name)` / `cam.kernel(c,s)`
> - **DoG dual-kernel** parallel 3×3 + general 5×5 + difference = bandpass / feature detection — `cam.dog('blob')`
> - **omnidirectional edge** `|Gx|+|Gy|` (Sobel magnitude, both polarities, all directions) — `cam.edges()`
> - **variable-size blur** 3-stage cascade (effective 5×5 / 9×9 / 13×13) — `cam.blur(5|9|13)` / `cam.cascade(...)`
>
> **POST — point ops** invert / grayscale / threshold / channel — `cam.post_op(n)`
>
> **DITHER (after POST, final stage)** `axis_rgb_dither`: bit-depth quantization via ordered (Bayer 4×4) / random (LFSR).
> 1bit = halftone (gray→`cam.halftone()`) / 2–4bit = posterize / 6bit = banding suppression — `cam.dither(bits, mode)`
>
> **Composition (free ordering, single command)**: `cam.chain(pre, mid, post, …)` sets PRE→MID→POST at once.
> Named presets `cam.pipeline(name)` (list with `cam.pipelines()`):
> `bin_edges` (binarize→Sobel) / `edge_binary` (Sobel→binarize) / `denoise_edges` (median→Sobel) /
> `median_sketch` (median→Sobel→binarize) / `smooth_sketch` (gaussian→Sobel→binarize) / `sketch` / `sharpen` / `dog_blob` …
> These can also be set from the interactive REPL `scripts/camera_repl.ps1 -Go` (menu Live HDMI → Filter combinations →
> named preset / build custom chain, or `cam.*` at the `>>>` prompt).
> Output is VGA 640×480 true RGB888 **30fps live HDMI** (CRC0).
>
> For all modules, principles, op tables, the 0xFE coefficient map, SW API, and resources, see
> **[Image Processing Pipeline — Principles & Architecture](docs/doc/image_processing_principles.md)**.

---

## Hardware Architecture (Resource Utilization / PS-PL Split)

### FPGA Resource Utilization

Actual build (maximum image-processing-slot configuration = including 3-stage cascade, `xc7z020clg400-1`,
**WNS = +0.017 ns** @ sysclk 100MHz):

| Resource | Used | Total (Z-7020) | Utilization |
| --- | ---: | ---: | ---: |
| LUT | 18,606 | 53,200 | **35.0 %** |
| FF (registers) | 17,705 | 106,400 | 16.6 % |
| BRAM (36Kb) | 9 | 140 | 6.4 % |
| DSP48E1 | 170 | 220 | **77.3 %** |

**DSP scales with the processing-slot configuration** (all modules resident, output selected by op. For per-stage detail, see
[Image Processing Pipeline](docs/doc/image_processing_principles.md)):

| Slot configuration | DSP | WNS |
| --- | ---: | ---: |
| Point ops + arbitrary 3×3 | 29 / 220 (13%) | +0.125 |
| + DoG dual-kernel (op12) | 110 / 220 (50%) | +0.156 |
| + 3-stage cascade variable blur (op13-15) (current) | **170 / 220 (77%)** | +0.017 |

- **Route all multipliers onto DSP48** (adding LUT multipliers in the congested sysclk/AXI domain drops WNS to
  −1.6 to −2.6 → offload to DSP, and split "sum → shift → saturate" across stages to converge).
  ★ Lesson: before deciding a timing failure is "congestion," suspect **insufficient pipeline stages in the new logic**.
- Even at 77% DSP, there is headroom with LUT 35% / BRAM 6%. To stack further stages, move the separable line buffers into BRAM
  (`xpm_memory_sdpram`; Vivado inference is impossible due to 8-6849) or use a larger device (Kria, etc.).

### How the Zynq PS Is Used (PS-PL Split)

The Zynq-7020 is a single-chip integration of the **PS (dual Cortex-A9 + DDR3 controller + peripherals)** and the
**PL (FPGA fabric)**. The role split in this design:

```text
            ┌─────────────── PS (ARM Cortex-A9, Linux + PYNQ) ───────────────┐
  bitstream │  Loads the .bit into the PL at boot                            │
   ─────────┤  Control plane:  M_AXI_GP0 (AXI-Lite 32bit) ─► AXI ifc         │
            │     └─► R/W the RTL knobs via 6×AXI-GPIO:                      │
            │         bitslip / IDELAY / frame_lines / SCCB engine /         │
            │         conv coeffs (0xFE0i) / proc_op / debug pages           │
            │  Data plane: DDR3 = VDMA frame buffer (PYNQ CMA alloc)         │
            └───────────────┬──────────────────────────▲───────────────────┘
       FCLK_CLK0 100MHz     │ S_AXI_HP0 (high-speed)    │ S_AXI_HP0
       (PL sysclk/AXI dom)  ▼ S2MM: PL→DDR write        │ MM2S: DDR→PL read
            ┌─────────────── PL (custom RTL = real-time pixel processing) ───┐
            │  D-PHY RX → CSI-2 decode → RGB unpack → proc slot →            │
            │  AXI4-Stream → AXI VDMA ─(S2MM)→ DDR / (MM2S)→ rgb2dvi → HDMI  │
            │  ※ byte_clk/core_clk/pixel_clk are generated in the PL from the│
            │     D-PHY clock lane (MMCM/BUFR; independent of the PS FCLK)   │
            └────────────────────────────────────────────────────────────────┘
```

- **PS = Linux + control + DRAM frame buffer**. No pixel processing runs on the PS at all
  (the PYNQ Python is a control role that "turns the knobs"; it never touches pixels).
- **Control**: `M_AXI_GP0` (AXI-Lite master) → 6×AXI-GPIO. PYNQ MMIO writes reach all RTL
  knobs (lock, IDELAY, SCCB, conv coeffs, proc op), and read the debug pages.
- **Data**: VDMA moves **PL→DDR (camera write, S2MM)** and **DDR→PL (HDMI read, MM2S)**
  via `S_AXI_HP0` (AXI high-speed slave). The frame buffer is the PS DDR3.
- **Clocks**: `FCLK_CLK0 = 100MHz` is the PL sysclk / AXI domain (GPIO, VDMA control,
  capture bridge). The byte/core/pixel clocks are generated in the PL and are PS-independent.
- This is the textbook Zynq split (PS = software control + memory, PL = deterministic real-time processing).
  The custom RTL **uses no Xilinx MIPI/CSI-2 IP** and handles everything from D-PHY lock to pixel processing within the PL.

---

## Key Features (Problems Solved in This Pipeline)

| Feature | Description | Related modules |
| ---- | ---- | ---- |
| **Genuine 30fps (PLL mult=96)** | The proper approach of raising PCLK from 27→48MHz via the PLL, not cutting VTS (borrowing time). `0x3036=0x60` gives VCO=768 → PCLK=48MHz (30fps) / link=384MHz (768Mbps) → byte_clk 96MHz. Re-constraining XDC `dphy_hs` to 384MHz and rebuilding gives WNS=+0.112 (0 additional resources). Full vblank/exposure preserved, no banding/darkening | `ov5640_sccb_init_probe.sv`, `mipi_to_hdmi_probe.xdc` |
| **Banding fix (settle-blank K)** | Solves the problem of missing per-line SoT due to HS-settle garbage at the burst head, with a K byte_clk SoT-window blank in the byte domain → full 480 lines. K scales with byte_clk (30fps=14 / 17fps=8) | `dphy_hs_byte_probe.sv` |
| **HW deterministic lock FSM (E2)** | The soft lock_mode (8x8 bitslip sweep + /4 BUFR.CLR re-roll + hold) implemented as an RTL FSM. Auto-locks at power-on. `HWLOCK_DEFAULT_ON` baked in + bit26 inhibit | `dphy_hwlock_fsm.sv` |
| **boot-init NACK fix** | `C_DOUT_DEFAULT=0x02000000` on `frame_lines_gpio` holds RESETB High from boot → the bitstream-init SCCB ACKs | BD GPIO config |
| **zero-PYNQ RX** | The above + baking in continuous/RGB565 makes the chip self-configure + the FSM auto-lock + crc0% 480 lines from power-on alone (HDMI display still requires starting VDMA separately) | `zero_pynq_test.py` |
| **VDMA genlock (TUSER/FSYNC)** | `C_USE_S2MM_FSYNC=2` + `genlock_mode=2` resolves tiling (free-run) | BD VDMA config |
| **SOF synthesis / force-480** | `csi2_frame_state` opens a frame on the first LS when FS is missing + fixes 480 lines to stabilize VTC genlock (resolving rolling) | `csi2_frame_state.sv` |

---

## Directory Structure

```text
rtl/
  mipi_rx/       CSI-2 protocol layer + D-PHY frontend + lock/supervisor FSMs
  img_proc/      unpack: RGB565=current (yuv422/raw8/raw10 are alternatives for
                 IMAGE_FORMAT switching, unused in this build → pruned in synth);
                 image-processing slot (prefilter/conv3x3/conv5x5/
                 DoG/cascade/proc_slot/dither); VDMA bridge; frame normalizer
  hdmi/          HDMI output / TPG
  prototype/     hardware top (mipi_to_hdmi_probe_top), SCCB init FSM, probe
verification/tb/  per-block testbenches (.sv) + DSim filelists (.f)
scripts/          PYNQ Python (bring-up / capture / deploy / diagnostics) + DSim runner (.ps1)
vivado/           build TCL (rebuild_*.tcl, pre_synth_tpg.tcl) + XDC
vloop_probes2/    current deploy Vivado project (BD-based, *gitignored)
docs/doc/         design specs (RTL / image processing, EN & JA)
```

> **Note**: The current deployment project `vloop_probes2/` is subject to `.gitignore` (including the BD).
> The RTL/scripts/TCL are git-managed, but the BD design (`C_DOUT_DEFAULT`, core0 CONFIG,
> VDMA fsync=2, etc.) is local only. Backup recommended.

---

## Build / Deploy

### RTL Build (Vivado 2024.2)

```powershell
# OOC re-synth + impl + bitstream of core0 (mipi_to_hdmi_probe_top)
& "$VIVADO\bin\vivado.bat" -mode batch -source vivado/rebuild_fe_min.tcl
# bitstream: vloop_probes2/vloop.runs/impl_1/bd_wrapper.bit
```

- `rebuild_fe_min.tcl`: standard core0 OOC re-synth (preserves the VDMA fsync=2 OOC).
- `rebuild_zeropynq.tcl`: zero-PYNQ RX configuration (bakes in GPIO `C_DOUT_DEFAULT` + core0 BD CONFIG).
- For the RTL parameter-binding rules, see [RTL Design Specification](docs/doc/rtl_design_spec.md) (core0 BD CONFIG > RTL default > fileset generic [cosmetic]).

### DSim Verification (DSim 2026)

```powershell
& "$DSIM_HOME\shell_activate.ps1"
& "$DSIM_HOME\bin\dsim.exe" -timescale 1ns/1ps -f verification/tb/<block>.f -top tb_<block>
```

### Board Deploy / Live HDMI (PYNQ)

```powershell
# Live HDMI (30fps: software lock + banding fix K=14, verified clean configuration)
# The default for camera_hdmi_demo / oneshot is the 30fps configuration (--hw-lock 0 / --settle-blank 14).
python scripts/deploy_banding_test.py --script camera_hdmi_demo.py `
    --download 1 --full-init 1 `
    --upload-bit vloop_probes2/vloop.runs/impl_1/bd_wrapper.bit `
    --extra-args "--vcm-sweep 0 --total 90"

# Still capture
python scripts/deploy_banding_test.py --script oneshot_capture.py ...
```

> Use `--upload-bit` only on the first run that flashes a new build, to send the .bit/.hwh to the board. It can be omitted afterward
> (`--download 1` reloads the on-board `bd_wrapper.bit`). If you revert to a 17fps build, add
> `--extra-args "--hw-lock 1 --settle-blank 8 ..."`.

### Interactive Control REPL (Unified Camera-Control Tool)

Instead of scattered one-off scripts, this REPL unifies bring-up, lock, register R/W, live HDMI,
still capture, focus, etc., into a single interactive object `cam`. **On launch a menu opens, and you
select operations by number. Arbitrary values (seconds, registers, coefficients, etc.) are entered interactively with `[default]` shown**
(Enter accepts the default). Quitting the menu with `q` drops to the `>>>` prompt, where the conventional `cam.*` API
works directly (re-enter with `Menu(cam).run()`).

```powershell
.\scripts\camera_repl.ps1            # upload + open the menu REPL (SSH password: xilinx)
.\scripts\camera_repl.ps1 -Go        # run cam.go() (full bring-up) on launch, then the menu
```

**Menu operation** (enter a number → Enter. `0`/empty/`b` to go back; top-level `q` to `>>>`):

```text
   1) Bring-up / status      (go / status / diagnostics)
   2) Live HDMI / processing (hdmi / proc / kernel / dog / blur / edges / capture)
   3) Registers / debug      (read / write / dbg / regs / accounting / eye)
   4) Knobs                  (vcm / idelay / settle / window / gain / sharpen / testpattern)
   h) Command help (raw cam.* API)
   q) Quit menu -> >>> python prompt (cam stays alive)
```

Direct API at the `>>>` prompt (after quitting with `q`, or when launched with `--menu 0`):

```python
>>> cam.go()          # init + RGB565 arm + software lock + banding fix K=14 (30fps config)
>>> cam.hdmi(60)      # live HDMI 60s (VDMA auto-stop)
>>> cam.capture()     # still image -> _capture/
>>> cam.read(0x300A)  # SCCB R/W | cam.dbg(0x18) | cam.link() | cam.status()
>>> cam.vcm(280)      # focus | cam.settle(14) | cam.idelay(16,16)
>>> cam.kernel([-1,0,1,-2,0,2,-1,0,1], 0)   # ad-hoc injection of an arbitrary 3x3 kernel
```

> At 30fps, `cam.go()` (menu "Full bring-up") defaults to software lock + K=14. For a 17fps
> build, answer y to "HW-lock FSM" at the bring-up prompt, or use `cam.go(hw=True)`. If the live image
> stutters, re-apply settle-blank K=14 (Knobs → Settle-blank, or `cam.settle(14)`).
> To drop straight to `>>>` without the menu, use `--menu 0` (for non-interactive piped execution).
>
> **Important**: Do not kill/TaskStop a job while VDMA is running (sshd hang → physical power cycle).
> The REPL's `cam.hdmi()`/`cam.capture()` auto-stop VDMA on return/exit (atexit + signal).
> Let it terminate naturally with `--total`. PYNQ-side scripts must always use `pynq_bringup.setup_session()`.

---

## Tools

| Tool | Version | Path example |
| ------ | --------- | ------ |
| Vivado | 2024.2 | `E:\...\xilinx\Vivado\2024.2\bin\vivado.bat` |
| DSim | 2026 | `C:\Program Files\Altair\DSim\2026` |
| Python (PYNQ) | 3.8+ | on board |

---

## Documentation

The authoritative design specs are the following 4 documents (EN & JA).

| Topic | English | 日本語 |
| ---- | ------- | ------ |
| **RTL Design Specification** (complete detail of all modules) | [rtl_design_spec.md](docs/doc/rtl_design_spec.md) | [rtl_design_spec_ja.md](docs/doc/rtl_design_spec_ja.md) |
| **Image Processing Pipeline** (principles and architecture) | [image_processing_principles.md](docs/doc/image_processing_principles.md) | [image_processing_principles_ja.md](docs/doc/image_processing_principles_ja.md) |
| **Image Processing Sample Gallery** (live captures of every filter, with settings) | [image_processing_samples.md](docs/doc/image_processing_samples.md) | [image_processing_samples_ja.md](docs/doc/image_processing_samples_ja.md) |

- **Reproduction steps**: [REPRODUCE.md](REPRODUCE.md)
- **License / attribution**: [LICENSE](LICENSE) / [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)

---

## License

This project is under the MIT License ([LICENSE](LICENSE)). Some D-PHY RTL
(`rtl/mipi_rx/dphy_lane_supervisor.sv`, `dphy_cdc_prims.sv`) is derived from the Digilent MIPI D-PHY
Receiver IP (MIT, Copyright (c) 2016 Digilent, Author: Elod Gyorgy); attribution is
consolidated in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
