# Image-File-Driven Verification (`img_file_uvm`)

🌐 **English** | **[日本語](image_file_verification_ja.md)**

Simulation-based verification of the img_proc slot RTL that uses **an image as the test
pattern**: an arbitrary image file (or a built-in pattern) is streamed pixel-by-pixel into
a selectable DUT under Verilator + cocotb + pyuvm (real UVM), the DUT's output frame is
captured by a UVM monitor and saved as an image, and **every output pixel is compared
against an expected image produced by applying the same filter to the same input in
Python software** (a golden model transliterated bit-exactly from the RTL).

Code: [verification/cocotb/img_file_uvm/](../../verification/cocotb/img_file_uvm/) ·
Runner wrapper: [scripts/run_image_test.ps1](../../scripts/run_image_test.ps1) ·
Related: [cocotb_python_test_guide.md](cocotb_python_test_guide.md),
[image_processing_principles.md](image_processing_principles.md)

---

## 1. The three images: input → expected vs output

Every run produces three images, and the test verdict is literally their comparison.
Worked example — run `proc_slot_20260703_052153` (POST slot, op=invert, built-in 64×48
pattern):

| | Image | Produced by | Meaning |
|---|---|---|---|
| <img src="samples/img_file_uvm/pattern_input.png" width="192"> | `input.png` | host (test-pattern generator or your `IMG_FILE` after conversion) | the frame streamed into the DUT, 1 pixel/clk with sof/eol/eof markers |
| <img src="samples/img_file_uvm/proc_slot_invert_expected.png" width="192"> | `expected.png` | **software golden model** — the same invert filter applied to `input.png` in Python ([golden.py](../../verification/cocotb/img_file_uvm/golden.py)) | what the RTL *must* output, bit-exactly |
| <img src="samples/img_file_uvm/proc_slot_invert_output.png" width="192"> | `output.png` | **RTL simulation** — the DUT's output stream captured beat-by-beat by the UVM monitor | what the RTL *did* output |

The UVM scoreboard (`FrameScoreboard`) compares `output` against `expected`
**pixel-for-pixel — all 3072 of 3072 pixels, frame borders included** — plus the
sof/eol/eof/err framing markers of every beat. `PASS` means the two images are
bit-identical (as above); any differing pixel fails the test with its (row, col) and
got/exp values, and the full diff goes to `mismatches.txt`. `run_info.txt` records the
configuration:

```text
dut=proc_slot proc_slot op=1 thresh=128
size: 64x48 frames=1
observed beats: 3072 / expected 3072
```

The output images are written **before** the check phase, so you always get the captured
image for visual inspection even when the comparison fails.

## 2. Expected/output pairs for all five DUTs

Same built-in input pattern as above; each row shows the golden expectation next to the
captured RTL output (bit-identical in these passing runs):

| DUT (filter) | expected.png (Python golden) | output.png (RTL sim) |
|---|---|---|
| `proc_slot` — invert | <img src="samples/img_file_uvm/proc_slot_invert_expected.png" width="160"> | <img src="samples/img_file_uvm/proc_slot_invert_output.png" width="160"> |
| `conv3x3` — gaussian, shift 4 | <img src="samples/img_file_uvm/conv3x3_gaussian_expected.png" width="160"> | <img src="samples/img_file_uvm/conv3x3_gaussian_output.png" width="160"> |
| `conv5x5` — gaussian5, shift 8 | <img src="samples/img_file_uvm/conv5x5_gaussian5_expected.png" width="160"> | <img src="samples/img_file_uvm/conv5x5_gaussian5_output.png" width="160"> |
| `prefilter` — median 3×3 | <img src="samples/img_file_uvm/prefilter_median_expected.png" width="160"> | <img src="samples/img_file_uvm/prefilter_median_output.png" width="160"> |
| `dither` — ordered Bayer, 2 bit/ch | <img src="samples/img_file_uvm/dither_ordered2_expected.png" width="160"> | <img src="samples/img_file_uvm/dither_ordered2_output.png" width="160"> |

The dark upper/left fringe on the convolution outputs is **real, verified RTL border
behaviour** — the first rows/columns of the window see the zero-initialised line buffers —
and the golden model reproduces it exactly, so borders are compared, not masked.

## 3. How to run

```powershell
# your own image (PNG/JPEG/BMP/... decoded via the repo-root Pillow venv; .ppm/.pgm direct)
.\scripts\run_image_test.ps1 -Image photo.png -Dut conv3x3 -Kernel sobel_x
.\scripts\run_image_test.ps1 -Image photo.jpg -Dut prefilter -Op median
.\scripts\run_image_test.ps1 -Image photo.png -Dut dither -DitherMode random -DitherBits 2

# built-in pattern, all five DUTs (what the registered regression runs)
.\scripts\run_image_test.ps1

# env-var form (full surface documented in img_file_uvm/img_config.py)
$env:IMG_FILE='photo.jpg'; $env:IMG_DUT='dither'
.\scripts\pytest_cocotb.ps1 verification/cocotb/img_file_uvm

# registered suite
.\scripts\run_cocotb.ps1 -Suite image
```

Each run writes to `verification/cocotb/_exec/img_file_uvm/<dut>_<timestamp>/`:

| File | Content |
|---|---|
| `input.png` / `input.ppm` | the frame that was streamed in (after decode/downscale) |
| `expected.png` | the golden image (same filter, Python software) |
| `output.png` / `output.ppm` | the captured DUT output frame |
| `run_info.txt` | DUT, filter config, size, observed/expected beat counts |
| `mismatches.txt` | on failure: every differing pixel as `f<frame> r<row> c<col> got/exp` |

Notes: `-MaxWidth`/`-MaxHeight` bound the downscale (default 640×480); `LINE_PIXELS` is
set per build from the image width; suite runs (`-Suite ...`) scrub stale `IMG_*` env vars
so the registered regression is always the deterministic built-in-pattern configuration.

## 4. Why the expected image is trustworthy

`expected.png` is not a generic library filter — it is a **streaming, beat-indexed
transliteration of the RTL** ([golden.py](../../verification/cocotb/img_file_uvm/golden.py)):
same read-before-write line buffers (zero initial state), same window shift registers,
same signed-coefficient × unsigned-tap arithmetic with arithmetic shift and saturation,
same Bayer table and Galois LFSR (seed `0xA5`, advanced per valid beat, carried across
frames), and even the RTL's Verilog width quirks (the dither smear shifts `n<<1`/`n<<2`
are 3-bit expressions that wrap modulo 8 — replicated, not "fixed"). A mismatch therefore
means a real RTL/model divergence, never a rounding convention difference. The models
survived an adversarial multi-agent audit against the RTL with zero confirmed defects
(see [diary_20260703.md](../progress/diary_20260703.md)).

## 5. UVM architecture

```
ImageFrameItem (1 frame = 1 transaction)
      │  ItemsSequence → sequencer
      ▼
FrameInputAgent ── FramePixelDriver ──► DUT (slot contract: pixel/valid/sof/eol/eof/err)
                                          │
PixelOutputAgent ── PixelMonitor ◄────────┘  (analysis port, one PixelItem per beat)
      │
      ▼
FrameScoreboard ── set_expected(golden) ── check_phase: count + markers + every pixel
```

All components live in [verification/cocotb/lib/uvm/](../../verification/cocotb/lib/uvm/)
and are reusable by other video testbenches.
