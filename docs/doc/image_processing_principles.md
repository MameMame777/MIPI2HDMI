# Image Processing Pipeline — Principles and Architecture

Target: Zybo Z7-20 (xc7z020clg400-1), RGB888 24-bit pixel stream, VGA 640x480 30fps.

> **Verification:** every slot described here is verified in simulation by the
> image-file-driven cocotb+pyuvm test — an image is streamed through the DUT and every
> output pixel is compared against the same filter applied in Python. See
> [image_file_verification.md](image_file_verification.md). Live on-hardware captures of
> the same filters: [image_processing_samples.md](image_processing_samples.md).

---

## 1. Pipeline Overview

The image processing pipeline is inserted between the MIPI CSI-2 receiver (which decodes OV5640 camera data into a 24-bit RGB pixel stream) and the AXI4-Stream video bridge (which feeds VDMA for DDR capture and HDMI display). All processing runs in the `sysclk` (125 MHz) domain.

```
video_pixel [23:0] (from rgb565_gray_unpack)
       |
       v
  [PRE] axis_rgb_prefilter ---- 3x3 denoising (median / gaussian) + point ops
       |
       +------ A branch --------+------ B branch (S1) ------+
       |                        |                            |
       v                        v                            |
  axis_rgb_conv3x3         axis_rgb_conv5x5                  |
  (arbitrary 3x3 kernel)  (arbitrary 5x5 kernel)             |
       |                        |                            |
       +----> dog_combine <-----+                            |
       |      (alpha*A - beta*B + offset)                    |
       |                        +---> axis_rgb_conv5x5_sep (S2) --- separable 5x5
       |                        |          |
       |                        |     axis_rgb_conv5x5_sep (S3) --- separable 5x5
       |                        |          |
       v                        v          v
  +----- final mux (cfg_proc_op) ---------+
  |  op 0-11:  conv3x3 output (single kernel or point op)   |
  |  op 12:    DoG combiner output (dual kernel)             |
  |  op 13:    conv5x5 output (cascade tier 1, eff. 5x5)    |
  |  op 14:    S2 output (cascade tier 2, eff. 9x9)         |
  |  op 15:    S3 output (cascade tier 3, eff. 13x13)       |
  +----------------------------------------------------------+
       |
       v
  [POST] axis_rgb_proc_slot ---- point ops (invert, grayscale, threshold, etc.)
       |
       v
  [DITHER] axis_rgb_dither ---- ordered (Bayer) / random (LFSR) bit-depth dither
       |
       v
  axis_video_bridge --> VDMA --> HDMI
```

### Slot Contract

Every processing module shares the same interface:

```
input:  {pixel[23:0], valid, sof, eol, eof, err}
output: {pixel[23:0], valid, sof, eol, eof, err}
```

- `pixel[23:0]` = `{R[7:0], G[7:0], B[7:0]}` (RGB888)
- `valid` = pixel is valid on this clock cycle
- `sof` / `eof` = start / end of frame
- `eol` = end of line (column 639)
- `err` = upstream error

Every module has `ENABLE` parameter: when 0, the module is a pure wire-through (zero logic cost). When enabled, all modes have **identical fixed latency** so that switching modes at runtime never causes marker misalignment.

---

## 2. Point Operations (PRE / POST)

### 2.1 Principle

A **point operation** transforms each pixel independently using only the value of that pixel — no spatial neighborhood is needed. These are the simplest and cheapest image operations (pure combinational logic, no line buffers, no DSP).

### 2.2 Operations

| op | Name | Formula | Purpose |
|----|------|---------|---------|
| 0 | Passthrough | `out = in` | No processing |
| 1 | Invert | `out = {255-R, 255-G, 255-B}` | Negative image |
| 2 | Grayscale | `Y = G; out = {Y, Y, Y}` | Luminance approximation (green channel ~59% of luma) |
| 3 | BGR swap | `out = {B, G, R}` | Red/blue channel exchange |
| 4 | Threshold | `out = (G > T) ? white : black` | Binary segmentation (threshold T configurable) |
| 5/6/7 | R/G/B only | Zero out two channels | Color channel isolation |

The grayscale approximation uses the green channel directly rather than the weighted sum `0.299R + 0.587G + 0.114B`. This avoids a multiply-accumulate unit that would add routing congestion on the Z-7020.

Point ops appear in two locations: as PRE (before convolution) and POST (after convolution), enabling chains like "edge detect then threshold".

---

## 3. Spatial Convolution — Principle

### 3.1 The Convolution Operation

2-D discrete convolution applies a small **kernel** (weight matrix) to a sliding window of the image:

```
out(x, y) = SUM over (i, j) in kernel:  kernel(i, j) * image(x+i, y+j)
```

For a 3x3 kernel this means 9 multiply-accumulate operations per pixel, per color channel. The result is then normalized (right-shifted) and clamped to [0, 255].

Different kernels produce different effects:

| Kernel | Weights | Effect |
|--------|---------|--------|
| Identity | `[0,0,0; 0,1,0; 0,0,0]` | No change |
| Gaussian blur | `[1,2,1; 2,4,2; 1,2,1] / 16` | Low-pass smoothing |
| Sharpen | `[0,-1,0; -1,5,-1; 0,-1,0]` | Edge enhancement |
| Sobel-X | `[-1,0,1; -2,0,2; -1,0,1]` | Horizontal gradient |
| Emboss | `[-2,-1,0; -1,1,1; 0,1,2]` | Relief effect |

### 3.2 Software vs Hardware — Why FPGA Convolution is Different

In software (CPU/GPU), a 3x3 convolution is simply a nested loop:

```python
for y in range(H):
    for x in range(W):
        acc = 0
        for i in range(-1, 2):
            for j in range(-1, 2):
                acc += kernel[i+1][j+1] * image[y+i][x+j]
        out[y][x] = acc >> shift
```

The CPU has random access to the entire image in memory, so it can fetch any pixel at any time. This loop processes one pixel at a time, sequentially.

In an FPGA streaming pipeline, the situation is fundamentally different:

1. **No random access**: Pixels arrive one per clock cycle in raster scan order (left to right, top to bottom). There is no "frame buffer" to index into arbitrarily.
2. **Throughput requirement**: At 640x480 @ 30fps, a new pixel arrives every ~33 ns (at ~9.2 Mpixel/s). The pipeline must produce one output pixel per input pixel, every clock cycle, with no stalls.
3. **Parallelism**: Unlike a CPU loop that does 9 multiplications sequentially, the FPGA performs all 9 multiplications **simultaneously** in dedicated hardware (DSP48 slices). This is spatial parallelism -- the "loop body" is unrolled into 9 physical multiplier circuits.

The central challenge is: **how do you access a 3x3 neighborhood of pixels when you only receive one pixel per clock?**

### 3.3 Line Buffers — Forming the Sliding Window in Hardware

#### 3.3.1 The Problem

A streaming pixel arrives one at a time in raster order (left to right, top to bottom). To compute a 3x3 convolution at pixel (x, y), the hardware needs pixels from rows y-2, y-1, and y simultaneously.

When pixel (x, y) arrives from the camera, the 3x3 window centred on it requires:

```
(x-1,y-1)  (x,y-1)  (x+1,y-1)     <-- row y-1 (arrived 640 pixels ago)
(x-1,y  )  (x,y  )  (x+1,y  )     <-- row y   (arrived 1 pixel ago .. now)
(x-1,y+1)  (x,y+1)  (x+1,y+1)     <-- row y+1 (not yet arrived!)
```

Row y+1 hasn't arrived yet, so the hardware actually computes the convolution **centred on the pixel that arrived 1 line + 1 pixel ago**, using the current pixel as the bottom-right corner of the window. The output has a fixed latency of ~1 line relative to the input.

#### 3.3.2 BRAM Line Buffer Architecture

**Solution: BRAM line buffers.** Two BRAM arrays, each 640 entries deep (= image width), form a shift-register chain of lines:

```
                     col index
                   0   1   2   ...  639
                 +---+---+---+---+---+
  lbB[640]       |   |   |   |...|   |  <-- row N-2 (oldest)
  (BRAM block)   +-+-+---+---+---+---+
                   | read prev2 = lbB[col]
                   | then write lbB[col] <- lbA[col]
                 +-v-+---+---+---+---+
  lbA[640]       |   |   |   |...|   |  <-- row N-1
  (BRAM block)   +-+-+---+---+---+---+
                   | read prev1 = lbA[col]
                   | then write lbA[col] <- in_pixel
                   |
                   v
              in_pixel (row N, arriving now)
```

Each clock cycle when `in_valid` is asserted:

```
Step 1 -- Read:    prev1 = lbA[col]     // the pixel from 1 line ago at this column
                   prev2 = lbB[col]     // the pixel from 2 lines ago at this column
Step 2 -- Write:   lbB[col] <- lbA[col] // shift the 1-line-old pixel down to 2-line-old
                   lbA[col] <- in_pixel // store the new pixel as 1-line-old
Step 3 -- Advance: col <- (eol) ? 0 : col + 1
```

This is a **read-before-write** pattern: each BRAM entry is read and then overwritten in the same clock cycle. Xilinx BRAM supports this natively.

After Step 1, three row values at the current column are available simultaneously:
- `prev2` = row N-2 at column `col`
- `prev1` = row N-1 at column `col`
- `in_pixel` = row N at column `col` (the just-arrived pixel)

**Why BRAM?** Each line buffer stores 640 x 24 bits = 15,360 bits. The Z-7020 has 36Kb BRAM tiles -- one tile per line buffer, with negligible area cost. If these were implemented as flip-flop registers instead, 640x24 = 15,360 FFs per buffer would consume ~14% of the Z-7020's FF budget for just two buffers.

The key RTL pattern that ensures BRAM inference:

```systemverilog
(* ram_style = "block" *) logic [23:0] lbA [0:LINE_PIXELS-1];  // attribute -> BRAM
always_ff @(posedge clk) begin       // NO reset on this block (critical!)
    if (in_valid) begin
        prev1    <= lbA[col];        // read
        lbA[col] <= in_pixel;        // write (same address, same cycle)
    end
end
```

**Trap: async reset on array blocks BRAM inference.** If you write `if (!rst_n) for (i) lbA[i] <= 0;`, Vivado cannot map the array to BRAM (BRAM has no async reset of contents) and instead synthesizes ~15,000 flip-flops.

#### 3.3.3 Column Shift Register -- Completing the 2-D Window

The line buffers provide three rows at the **current** column. But the 3x3 window needs three columns (current, previous, two-ago). A shift register per row delays the column values:

```
                         time ----------------->
                      col-2    col-1    col (newest)
Row N-2:  w[0][0] <-- w[0][1] <-- w[0][2] <-- prev2
Row N-1:  w[1][0] <-- w[1][1] <-- w[1][2] <-- prev1
Row N:    w[2][0] <-- w[2][1] <-- w[2][2] <-- in_pixel
```

Each clock cycle (when valid):
```systemverilog
w[0][0] <= w[0][1];  w[0][1] <= w[0][2];  w[0][2] <= prev2;   // row N-2 slides left
w[1][0] <= w[1][1];  w[1][1] <= w[1][2];  w[1][2] <= prev1;   // row N-1 slides left
w[2][0] <= w[2][1];  w[2][1] <= w[2][2];  w[2][2] <= in_pixel; // row N slides left
```

After this shift, `w[0:2][0:2]` holds the complete 3x3 window. `w[1][1]` is the centre pixel (the convolution target). The window slides one pixel to the right each clock cycle, tracking the input stream.

#### 3.3.4 Concrete Example -- Window Formation Over Time

Consider a 6-pixel-wide image (simplified):

```
Image in memory:            Pixel arrival order:
  A B C D E F               A(t=0) B(t=1) C(t=2) D(t=3) E(t=4) F(t=5)
  G H I J K L               G(t=6) H(t=7) I(t=8) J(t=9) ...
  M N O P Q R               M(t=12) N(t=13) O(t=14) P(t=15) ...
```

At t=15, pixel P (row 2, col 3) arrives. The line buffers contain:
- `lbA[3]` was written at t=9 with J (row 1, col 3)
- `lbB[3]` was written at t=9 from lbA[3]'s old value = D (row 0, col 3)

The column shift registers have shifted previous columns through, so the 3x3 window is:

```
w[0][0]=B  w[0][1]=C  w[0][2]=D     <-- row 0, cols 1-3
w[1][0]=H  w[1][1]=I  w[1][2]=J     <-- row 1, cols 1-3
w[2][0]=N  w[2][1]=O  w[2][2]=P     <-- row 2, cols 1-3
```

Centre pixel = `w[1][1]` = I. The convolution output at this clock is for pixel I -- not P. The output always lags the input by ~1 line + 1 column.

#### 3.3.5 Scaling to 5x5

For a 5x5 kernel, the same principle extends:
- **4 BRAM line buffers** (lbA..lbD) storing rows N-1 through N-4
- **5-deep column shift registers** per row, giving `w[0:4][0:4]`
- Centre pixel = `w[2][2]`

### 3.4 DSP48 Multiply-Accumulate -- The Arithmetic Core

#### 3.4.1 The Computation Per Clock

Once the window `w[0:2][0:2]` is formed, the convolution sum must be computed for each of the 3 RGB channels independently:

```
acc = coeff[0]*w[0][0] + coeff[1]*w[0][1] + coeff[2]*w[0][2]
    + coeff[3]*w[1][0] + coeff[4]*w[1][1] + coeff[5]*w[1][2]
    + coeff[6]*w[2][0] + coeff[7]*w[2][1] + coeff[8]*w[2][2]
```

That's 9 multiplications and 8 additions per channel, x3 channels = **27 multiplications per clock cycle**.

A CPU would execute these 27 multiplications sequentially (27 clock cycles minimum). The FPGA instantiates 27 physical multiplier circuits that all operate **in parallel** -- all 27 products are computed in a single clock cycle.

#### 3.4.2 DSP48E1 Slice -- Dedicated Silicon Multiplier

The Zynq-7020 contains 220 DSP48E1 slices. Each is a hardened silicon block (not configurable fabric -- fixed transistor circuits optimized for arithmetic):

```
+---------------------------------------------+
|                DSP48E1                       |
|                                              |
|  A[29:0] --+                                 |
|             +-- Pre-adder --> 25x18 Mult --> 48-bit Accumulator --> P[47:0]
|  B[17:0] --+     (A+/-D)                                            |
|  D[24:0] --+                                                        |
|  C[47:0] ----------------------------------------> (bypass adder) -+
|                                                                      |
|  Internal pipeline registers at every sub-stage                      |
+---------------------------------------------+
```

For convolution, each DSP48 computes one `coeff x pixel_channel`:
- **A input** = coefficient (signed 8-bit, sign-extended to 25-bit)
- **B input** = pixel channel value (unsigned 8-bit, zero-extended to 18-bit)
- **P output** = signed 17-bit product

The DSP48 runs at the fabric clock (125 MHz) and produces one product per cycle with zero additional LUT cost -- the multiply is performed in dedicated transistor logic, not in the configurable fabric.

#### 3.4.3 Why DSP48 and Not LUT Multipliers?

An 8x8 signed multiplier built from LUTs uses ~60 LUTs and several levels of carry-chain logic. 27 such multipliers would consume ~1,620 LUTs and create dense local routing. On the Z-7020:

| Approach | LUT cost | Routing | Timing (WNS) |
|----------|----------|---------|---------------|
| LUT multipliers (27x) | ~1,620 LUTs | Severe congestion | **-0.925 ns (FAIL)** |
| DSP48 slices (27x) | ~0 LUTs | Clean | **+0.125 ns (PASS)** |

The `(* use_dsp = "yes" *)` synthesis attribute forces Vivado to map the multiply to DSP48:

```systemverilog
(* use_dsp = "yes" *) logic signed [16:0] prod [0:2][0:8];
// prod[channel][tap] = coeff[tap] * pixel_channel_value
// 3 channels x 9 taps = 27 DSP48 slices
```

#### 3.4.4 Signed x Unsigned Multiplication

Coefficients are signed (e.g., Sobel has -1, -2), but pixel values are unsigned (0-255). The RTL handles this with explicit sign extension:

```systemverilog
prod[sc][t] <= coef(t) * $signed({1'b0, tap(t, sc)});
//              ^ signed 8-bit    ^ unsigned 8-bit, zero-extended to signed 9-bit
//              (-128..+127)       (0..255 -> signed 0..255)
```

The `{1'b0, pixel}` prepends a 0-bit, converting unsigned 8-bit [0, 255] to signed 9-bit [0, 255]. The product is then signed 17-bit (range: -128x255 = -32,640 to +127x255 = +32,385).

### 3.5 Pipeline Staging -- Meeting Timing at 125 MHz

#### 3.5.1 Why Pipeline?

At 125 MHz, each clock cycle is 8 ns. If the entire computation (window formation, 9 multiplies, 9-input sum, shift, clamp) were purely combinational, the propagation delay would far exceed 8 ns. The solution is to split the computation into **pipeline stages**, each completing within one clock period:

```
     Stage 1       Stage 2       Stage 3       Stage 4       Stage 5
     (1 cycle)     (1 cycle)     (1 cycle)     (1 cycle)     (1 cycle)
     +--------+    +--------+    +--------+    +--------+    +---------+
in > |Line    | -> |Window  | -> |27 DSP  | -> |9-way   | -> |Shift    | -> out
     |buffer  |    |shift   |    |multiply|    |add     |    |+saturate|
     |read    |    |register|    |(all //) |    |(sum)   |    |         |
     +---+----+    +---+----+    +---+----+    +---+----+    +---+-----+
         |             |             |             |             |
         v             v             v             v             v
      register      register      register      register      register
      (FF)          (FF)          (FF)          (FF)          (FF)
```

Each stage's output is captured in flip-flop registers. The combinational logic between any two registers is short enough to complete within 8 ns.

**Throughput**: Despite the 5-stage latency, a new pixel enters stage 1 every clock cycle, and a finished pixel exits stage 5 every clock cycle. The pipeline processes 125 million pixels per second -- far more than the required ~9.2M pixels/s (640x480x30fps).

**Latency**: Each pixel's result appears 5 clock cycles after it enters. This fixed delay is invisible to the viewer (40 ns) and is compensated by delaying the control markers (sof, eol, eof, err) through a matching 5-stage shift register.

#### 3.5.2 Marker Delay Chain

The frame/line markers must arrive at the output at the same time as the pixel they accompany. A parallel shift register delays them through the same number of stages:

```systemverilog
// markers travel alongside the pixel data, delayed to match
always_ff @(posedge clk) begin
    {v2, s2, e2, f2, r2} <= {v1, s1, e1, f1, r1};   // stage 2
    {v3, s3, e3, f3, r3} <= {v2, s2, e2, f2, r2};   // stage 3
    {v4, s4, e4, f4, r4} <= {v3, s3, e3, f3, r3};   // stage 4
    // ... through all stages
end
// Final output: out_valid <= v4; out_sof <= s4; ...
```

This guarantees that `out_sof` (start of frame) always appears on the exact same clock cycle as the first processed pixel of that frame.

#### 3.5.3 The 5x5 Adder Tree Problem

For the 5x5 convolution, summing 25 products in a single clock stage creates a 25-input addition -- too deep for 8 ns. The solution is a **2-level adder tree**:

```
Stage 4 (group-sum):   5 groups of 5 products each
   group[0] = prod[0] + prod[1] + prod[2] + prod[3] + prod[4]
   group[1] = prod[5] + prod[6] + prod[7] + prod[8] + prod[9]
   ... (5 partial sums, each a 5-input addition)
                          |
                       register (FF)
                          |
Stage 5 (accumulate):  acc = group[0] + group[1] + group[2] + group[3] + group[4]
                          |
                       register (FF)
```

Each stage sums only 5 values (4 additions), which fits in 8 ns. The total 25-value sum is computed over 2 clock cycles.

### 3.6 Normalization and Saturation

After summing the products:

```
v = accumulated_sum >>> cfg_shift    // arithmetic right shift (preserves sign)
if (cfg_abs && v < 0)  v = -v       // absolute value (for gradient magnitude)
out = (v < 0) ? 0 : (v > 255) ? 255 : v[7:0]   // saturate to [0, 255]
```

The `cfg_shift` parameter normalizes the kernel. For a Gaussian `{1,2,1; 2,4,2; 1,2,1}`, the coefficients sum to 16, so `shift = 4` (divide by 16) preserves brightness.

**Why arithmetic right shift (`>>>`) instead of divide?** Division is expensive in hardware (iterative or large combinational divider). Shifting right by N bits is equivalent to dividing by 2^N -- and in hardware, a right shift is **free**: just wire the upper bits to the output, discarding the lower N bits. No logic gates are consumed. This is why convolution kernels are designed with power-of-2 normalization factors (16, 256, etc.).

**Saturation** clamps the result to the valid unsigned 8-bit range [0, 255]. Without clamping, a sum like `(4 x 255) >>> 0 = 1020` would wrap around to `252` (1020 mod 256), producing incorrect bright pixels. The clamp logic is a simple comparator:

```systemverilog
sat = (v < 0) ? 8'd0 : (v > 255) ? 8'd255 : v[7:0];
```

### 3.7 Absolute Value for Edge Detection (`cfg_abs`)

Edge detection kernels like Sobel produce **signed** outputs: positive values for one gradient direction, negative for the other. Without `cfg_abs`, negative values are clamped to 0 — half of the edges are lost.

With `cfg_abs = 1`, the absolute value is taken **before** the final [0, 255] saturation:
- Sobel-X without abs: detects only light-to-dark horizontal edges
- Sobel-X with abs: detects horizontal edges in both directions

This is essential for the omnidirectional edge magnitude mode (see Section 6).

---

## 4. General 5x5 Convolution (`axis_rgb_conv5x5`)

### 4.1 Principle

A 5x5 kernel has 25 coefficients, enabling larger spatial support — useful for Gaussian blur with a wider radius, or as one half of a Difference of Gaussians pair.

### 4.2 Two-Level Adder Tree

Summing 25 products in a single clock cycle creates a very long combinational path. The 5x5 module uses a **2-level adder tree**:

```
Stage 4 (group-sum):  5 groups of 5 products -> 5 partial sums
Stage 5 (accumulate): 5 partial sums -> 1 total
```

This splits the 25-input sum into two 5-input stages, each of which meets timing at 125 MHz.

### 4.3 Resource Cost

- 75 DSP48 slices (25 taps x 3 RGB channels)
- 4 BRAM line buffers (640 x 24-bit each)
- 6 pipeline stages

---

## 5. Difference of Gaussians (DoG) — Dual Kernel Combiner

### 5.1 Principle

The **Difference of Gaussians (DoG)** is a band-pass filter formed by subtracting a wide-radius Gaussian (blurred) from a narrow-radius Gaussian (less blurred):

```
DoG(x, y) = G_small(x, y) - G_large(x, y)
```

This approximates the Laplacian of Gaussian (LoG), a standard technique in computer vision for:
- **Blob detection**: finding regions that differ from their surroundings
- **Edge detection**: zero-crossings of the DoG correspond to edges
- **Unsharp masking**: subtracting the blurred version enhances detail

### 5.2 Architecture — Parallel Branches

The 3x3 and 5x5 convolution modules process the **same input pixel stream** in parallel:

```
input ----+---- conv3x3 (small kernel, A) ---- fast path (5-stage pipeline)
          |
          +---- conv5x5 (large kernel, B) ---- slow path (6-stage pipeline)
```

Both produce one output pixel per input pixel in identical raster order. The k-th output from A and the k-th output from B correspond to the **same spatial pixel**. B simply trails A by a fixed latency (~1 line + 1 cycle, due to the extra line buffers).

### 5.3 Ordinal Alignment FIFO

The combiner does NOT attempt to compute the latency difference arithmetically (which would be fragile and error-prone). Instead, it uses an **ordinal alignment FIFO**:

- A pixels are **pushed** into a BRAM FIFO on `a_valid`
- A pixels are **popped** from the FIFO on `b_valid`

Since both branches process pixels in identical order, the k-th pop always matches the k-th push — automatic alignment with zero latency arithmetic.

The FIFO depth (1024 entries) only needs to exceed the maximum lead of A over B (~641 entries for 640 pixels/line + pipeline difference).

### 5.4 Combination Formula

```
out = sat( (alpha * A - beta * B) >>> shift + offset )
```

Four modes:
- **mode 0**: A passthrough (3x3 only)
- **mode 1**: B passthrough (5x5 only)
- **mode 2**: DoG = alpha*A - beta*B (band-pass / edge / blob)
- **mode 3**: Sum = alpha*A + beta*B (multi-scale blend)

The `alpha`, `beta` (unsigned 8-bit), `shift` (4-bit), and `offset` (signed 9-bit) are all runtime-configurable.

### 5.5 Practical Presets

| Preset | 3x3 Kernel | 5x5 Kernel | alpha | beta | shift | offset | Effect |
|--------|-----------|-----------|-------|------|-------|--------|--------|
| blob | Gaussian 3x3 | Gaussian 5x5 | 1 | 1 | 0 | 128 | Band-pass blob detector |
| unsharp | Identity | Gaussian 5x5 | 2 | 1 | 0 | 0 | Detail enhancement |

---

## 6. Omnidirectional Edge Magnitude (`|Gx| + |Gy|`)

### 6.1 The Problem with Single-Kernel Edge Detection

A single 3x3 Sobel-X kernel detects **horizontal edges** (vertical gradients) only. Similarly, Sobel-Y detects only vertical edges. A single kernel cannot capture edges in all orientations.

The true edge magnitude is:

```
M = sqrt(Gx^2 + Gy^2)    (Euclidean, expensive in hardware)
```

A common approximation that avoids the square root:

```
M ≈ |Gx| + |Gy|          (L1 norm, cheap, overestimates by at most sqrt(2))
```

### 6.2 Implementation via Parallel Convolutions + Absolute Value + Sum

The existing dual-kernel architecture is reused:

```
Branch A (conv3x3):  Sobel-X kernel, cfg_abs = 1  -->  |Gx|
Branch B (conv5x5):  Sobel-Y kernel (centred in 5x5), cfg_abs = 1  -->  |Gy|
Combiner (mode 3 = sum):  alpha=1, beta=1  -->  |Gx| + |Gy|
```

**Key insight**: The absolute value MUST be computed inside each convolution module (before the [0, 255] saturation), NOT in the combiner. If abs were applied after saturation, negative gradient values would already be clamped to 0 — half the edge information is irretrievably lost.

This is why `cfg_abs` is a per-convolution-module control:
- `cfg_abs` in conv3x3: `v = acc >>> shift; if (v < 0) v = -v; clamp [0, 255]`
- `cfg_abs` in conv5x5: same
- Combiner sees only non-negative [0, 255] values, so `sum` mode produces `|Gx| + |Gy|`

### 6.3 Verification

Colorbar test pattern: single Sobel-X (abs off) shows edge% = 1.1%. With omnidirectional (abs on both, sum mode), edge% = 2.2% — exactly 2x, confirming both gradient polarities are recovered.

---

## 7. Cascaded Variable-Size Gaussian Blur

### 7.1 Principle — Separable Convolution

A 2-D Gaussian kernel is **separable**: it can be decomposed into the outer product of two 1-D vectors:

```
G_2D = g_v * g_h^T
```

where `g_h` and `g_v` are 1-D Gaussian vectors. This means a 5x5 Gaussian can be computed as two 1-D passes:

```
Step 1 (horizontal):  row_filtered(x, y) = SUM over j: h[j] * image(x+j, y)
Step 2 (vertical):    out(x, y) = SUM over i: v[i] * row_filtered(x, y+i)
```

**Cost: 5+5 = 10 multiplies/channel instead of 25.** For 3 channels: 30 DSP vs 75 DSP. This enables cascading multiple stages within the Z-7020's 220 DSP budget.

### 7.2 Cascade Architecture

Three blur stages are chained:

```
conv5x5 (S1, general) --> conv5x5_sep (S2, separable) --> conv5x5_sep (S3, separable)
```

Each stage can independently be set to identity (passthrough) or a Gaussian kernel. By enabling different combinations:

| Active Stages | Effective Kernel Size | DSP Used |
|---------------|----------------------|----------|
| S1 only | 5x5 | 75 |
| S1 + S2 | 9x9 | 75 + 30 = 105 |
| S1 + S2 + S3 | 13x13 | 75 + 60 = 135 |

### 7.3 Why Cascade Increases Effective Size

When two Gaussian filters are applied sequentially, their variances ADD:

```
sigma_combined^2 = sigma_1^2 + sigma_2^2
```

Spatially, the effective kernel width also grows. A 5x5 Gaussian followed by another 5x5 Gaussian is equivalent to a single ~9x9 Gaussian (the convolution of two 5-wide functions has support 5+5-1 = 9). Three stages give 5+5+5-2 = 13.

### 7.4 Separable Module Internals

The `axis_rgb_conv5x5_sep` module has an 8-stage pipeline:

**Horizontal pass (stages 1-4):**
1. 5-deep column shift register (1 clock)
2. 15 DSP products: `h_coeff[i] * pixel_channel` (1 clock)
3. 5-input horizontal sum (1 clock) — separated from shift/clamp to fix timing
4. Requantize to signed 12-bit with configurable shift (1 clock)

**Vertical pass (stages 5-8):**
5. 4 BRAM line buffers of packed 36-bit horizontal results (1 clock)
6. 15 DSP products: `v_coeff[j] * h_result_row_j` (1 clock)
7. 5-input vertical sum (1 clock) — separated from shift/saturate
8. Normalize + saturate to unsigned 8-bit (1 clock)

The intermediate horizontal result is kept as signed 12-bit (not 8-bit) to preserve precision through the two-pass computation.

---

## 8. Median Filter (3x3)

### 8.1 Principle

The **median filter** replaces each pixel with the median (middle value when sorted) of its 3x3 neighborhood. Unlike linear filters (Gaussian), the median filter:

- Preserves edges (no blurring across sharp boundaries)
- Effectively removes **salt-and-pepper noise** (isolated extreme values)
- Is nonlinear (cannot be expressed as a convolution kernel)

### 8.2 Sorting Network (median9)

Finding the median of 9 values requires a **sorting network** — a fixed sequence of compare-and-swap (CAS) operations:

```
CAS(a, b) = { min(a,b), max(a,b) }
```

The implementation uses the **Smith/Paeth median-of-9 network**: 19 CAS operations arranged in 9 dependency layers. These are partitioned into 5 pipeline stages (at most 2 serial CAS per stage) to meet 125 MHz timing:

```
Stage 1: CAS(1,2) CAS(4,5) CAS(7,8) then CAS(0,1') CAS(3,4') CAS(6,7')
Stage 2: CAS(1,2) CAS(4,5) CAS(7,8) then CAS(0,3) CAS(5,8) CAS(4,7)
Stage 3a: CAS(3,6) CAS(1,4) CAS(2,5)
Stage 3b: CAS(4,7) then CAS(2,4')
Stage 4: CAS(4,6) then CAS(2,4') --> median at index 4
```

This network does NOT fully sort all 9 values — it only determines the 5th order statistic (the median), which is sufficient and cheaper.

**Correctness**: Verified by Knuth's 0-1 principle (all 512 binary input vectors), all 362,880 permutations of 9 distinct values, and 300,000 random vectors.

### 8.3 Per-Channel Application

Three independent `median9` instances run in parallel for R, G, B channels. The median is computed per-channel, not on luminance — this preserves color information while denoising.

### 8.4 Latency Matching

The prefilter module has three branches (median, Gaussian, point) that must all produce output at the same time. All branches are padded to equal latency (5 cycles after the window is formed), with markers delayed through a matching shift register chain.

---

## 9. Gaussian Blur (3x3, Zero-DSP)

### 9.1 Principle

The PRE stage includes a fixed 3x3 Gaussian that uses **no DSP slices** — only shift-and-add:

```
Kernel:  [1, 2, 1]     Weights:  corners x1, edges x2, centre x4
         [2, 4, 2]     Sum = 16, normalize by >> 4
         [1, 2, 1]
```

### 9.2 Shift-Add Implementation

The weights (1, 2, 4) are all powers of 2, so multiplication is replaced by bit shifts:

```
Per channel:
  corners = w[0][0] + w[0][2] + w[2][0] + w[2][2]         (4 additions, x1)
  edges   = w[0][1] + w[1][0] + w[1][2] + w[2][1]         (4 additions, x2)
  centre  = w[1][1]                                         (x4)
  total   = corners + (edges << 1) + (centre << 2)          (shifts + adds)
  output  = total >> 4                                      (truncation = /16)
```

This avoids all multipliers. The 4-stage pipeline matches the median branch latency.

---

## 10. Dithering — Bit-Depth Reduction (after POST)

### 10.1 Principle

The final pipeline stage (`axis_rgb_dither`, after the POST point-op slot, before the capture
bridge) reduces each channel to N bits while adding a position- or noise-dependent bias *before*
quantizing. The bias pushes pixels across quantization boundaries in a structured way, so a smooth
gradient is rendered as a dithered pattern instead of hard bands. Uses: N=1 halftone (0/255),
N=2-4 posterize/retro, N=6 anti-banding on low-bit panels. Stateless (no line buffers) — ~tens of
LUTs, fixed 1-cycle latency. `cfg_ctrl=0` (0xFE4A) = passthrough (bit-identical default).

### 10.2 Ordered Dithering (Bayer 4x4)

A 4x4 Bayer threshold matrix indexed by `(x%4, y%4)` gives a deterministic, tileable bias:

```
   0  8  2 10
  12  4 14  6     value 0..15 (/16)
   3 11  1  9
  15  7 13  5
```

The matrix value is scaled into the dropped-LSB range and added before truncation → a fixed,
regular dot pattern (the classic "ordered dither" look). No line buffers, no feedback.

### 10.3 Random Dithering (LFSR)

An 8-bit Galois LFSR (advanced once per valid pixel) supplies a pseudo-random bias masked to the
dropped-LSB range → white-noise dithering (grainy, pattern-free). One LFSR, negligible cost.

### 10.4 Quantize + Full-Range Replication

```
drop = 8 - N
bias = ordered: bayer << (drop-4)     |   random: lfsr & (2^drop - 1)
v    = clamp(in + bias, 255)
q    = v & ~(2^drop - 1)              (keep the top N bits)
out  = q | q>>N | q>>2N | q>>4N       (replicate MSBs back to full 8-bit range)
```

The replication ("bit smearing") maps the N-bit code to the full 0..255 range, so N=1 → {0,255}
(not 0/128) and N=2 → {0,85,170,255}. Position `(x,y)` comes from internal col/row counters (`col`
like conv3x3; a `row` counter added: inc on EOL, reset on EOF).

### 10.5 Why Ordered, Not Error Diffusion

Floyd–Steinberg / Atkinson error diffusion gives the best (pattern-free) quality but needs a
full-width line buffer plus a per-pixel left-error feedback loop — extra logic and a tight feedback
path on the already-congested sysclk. Ordered + random are stateless and close timing trivially;
error diffusion is deferred. HW-verified (WNS +0.112): halftone shows the Bayer dot pattern live on
HDMI; the 5 dither modes cycle (OFF / halftone / poster-2 / poster-4 / random) on the monitor.

---

## 11. Runtime Configuration

All processing parameters are configurable at runtime through the **0xFE SCCB intercept** mechanism: when the SCCB engine receives a write to address `0xFExx`, it routes the data to internal coefficient registers instead of sending it to the camera chip. This reuses the existing SCCB write path (AXI GPIO) without requiring additional block design modifications.

| Address Range | Target |
|---------------|--------|
| `0xFE00-08` | 3x3 kernel coefficients (9 x signed 8-bit) |
| `0xFE09` | 3x3 normalization shift |
| `0xFE20-38` | 5x5 kernel coefficients (25 x signed 8-bit) |
| `0xFE39` | 5x5 normalization shift |
| `0xFE40-44` | DoG combiner (mode, alpha, beta, shift, offset) |
| `0xFE45` | Absolute value enable (bit0=3x3, bit1=5x5) |
| `0xFE46-49` | PRE/POST op and threshold |
| `0xFE4A` | Dither control ([0]=enable, [1]=mode 0 ordered/1 random, [4:2]=bits/ch) |
| `0xFE50-5B` | Cascade S2 separable kernel (h[5], hshift, v[5], vshift) |
| `0xFE60-6B` | Cascade S3 separable kernel |

The `cfg_proc_op[3:0]` field (from the IDELAY GPIO word, bits [24:21]) selects which pipeline output reaches the final mux.

---

## 12. Resource Usage (xc7z020)

| Module | DSP48 | BRAM | Pipeline Stages |
|--------|-------|------|-----------------|
| PRE prefilter (Gaussian) | 0 | 2 | 8 |
| PRE prefilter (Median) | 0 | 2 | 8 |
| conv3x3 (A branch) | 27 | 2 | 5 |
| conv5x5 (B/S1 branch) | 75 | 4 | 6 |
| DoG combiner | 6 | 1 (FIFO) | 4 |
| conv5x5_sep (S2) | 30 | 4 | 8 |
| conv5x5_sep (S3) | 30 | 4 | 8 |
| POST proc_slot | 0 | 0 | 1 |
| **Total (all active)** | **168** | **17** | — |

Z-7020 budget: 220 DSP (77% utilized), 140 BRAM (12%), 53,200 LUT (35%).

---

## 13. Design Lessons

1. **DSP vs LUT for multiplies**: On the Z-7020 (tight LUT routing), convolution multiplies MUST use DSP48 slices (`(* use_dsp = "yes" *)`). LUT-based multiplies cause timing failure at 125 MHz.

2. **Split long combinational paths**: Sum + shift + clamp in one pipeline stage causes WNS violations. Splitting into separate registered stages (sum | shift+clamp) reliably closes timing.

3. **Absolute value before saturation**: For gradient-based operations, |v| must be computed before the [0, 255] clamp. After clamping, negative values are already 0 and cannot be recovered.

4. **Reset-free RAM arrays for BRAM inference**: Async reset on array declarations (`always_ff ... if (!rst_n) for(...) mem[i] <= 0`) forces Vivado to implement the array as flip-flops (~24k FFs) instead of BRAM. Solution: RAM write/read in a reset-free `always_ff` block; only pointers/control in the reset block.

5. **Ordinal FIFO for branch alignment**: Computing exact latency differences between parallel branches is fragile. An ordinal FIFO (push on A valid, pop on B valid) provides automatic alignment with zero arithmetic.

6. **Separable kernels save DSP**: A separable 5x5 costs 30 DSP vs 75 for a general 5x5 — enabling cascade architectures within a small FPGA's DSP budget.
