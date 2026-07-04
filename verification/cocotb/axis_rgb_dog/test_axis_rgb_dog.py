"""cocotb port of verification/tb/tb_axis_rgb_dog.sv (DoG dual-kernel chain).

The DSim TB wires three DUTs in parallel/series and checks that the two convolution
branches stay spatially aligned through the ordinal FIFO in the combiner:

    in -> axis_rgb_conv3x3 (A, small Gaussian) --+
       -> axis_rgb_conv5x5 (B, large Gaussian) --+-> axis_rgb_dog_combine -> out

Because Verilator elaborates a single toplevel, this port generates a tiny synthesizable
wrapper (``dog_chain_top``) that instantiates the three DUTs *exactly* as the DSim TB does
(same params: conv LINE_PIXELS=W ENABLE=1; combine ENABLE=1 DEPTH=64) and exposes the same
driving ports. The wrapper is a build artifact emitted by ``test_axis_rgb_dog()`` -- the
only hand-authored file is this one.

Stimulus + checks mirror the TB 1:1 on a 12x8 (H x W) frame:
  - warm-up frame primes the cold line buffers (result discarded)
  - mode 1 (B passthrough) uniform 100 -> ~100
  - mode 0 (A passthrough, FIFO-aligned) uniform 100 -> ~100
  - mode 2 (DoG, a=b=1 shift0 offset128) uniform 100 -> ~128 (flat DoG = 0 + offset)
  - mode 2 on 40/200 bands: the flattest row stays ~128 (alignment OK), the strongest
    transition row deviates a lot (real edge response)

out_r/out_g/out_b are byte lanes 2/1/0; the TB samples out_pixel[23:16] = R.
"""
from __future__ import annotations

import os
import random
import sys
from pathlib import Path

import cocotb
from cocotb.triggers import ClockCycles

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "img_file_uvm"))
import golden as G  # noqa: E402
from lib.clkreset import bringup  # noqa: E402
from lib.pixel_stream import PixelMonitor, PixelStreamDriver  # noqa: E402
from lib.scoreboard import check  # noqa: E402

W = 8
H = 12

# --- kernels, packed to mirror the SV concat {c_first(MSB) .. c_last(LSB)} ---
# The DUT reads coef(idx) = cfg_coeffs[idx*8 +: 8], so idx0 is the LSB byte = the LAST
# element written in the SV concatenation. Both Gaussians are symmetric so ordering is
# academic, but we pack exactly as the TB does regardless.
A_COEFFS_LIST = [1, 2, 1, 2, 4, 2, 1, 2, 1]                       # 3x3 Gaussian /16
A_SHIFT = 4
B_COEFFS_LIST = [1, 4, 6, 4, 1, 4, 16, 24, 16, 4,
                 6, 24, 36, 24, 6, 4, 16, 24, 16, 4,
                 1, 4, 6, 4, 1]                                   # 5x5 Gaussian /256
B_SHIFT = 8


def _pack_concat(elems, width=8):
    """SV {a,b,c,...}: first-listed element is the MSB."""
    val = 0
    for e in elems:
        val = (val << width) | (e & ((1 << width) - 1))
    return val


A_COEFFS = _pack_concat(A_COEFFS_LIST)
B_COEFFS = _pack_concat(B_COEFFS_LIST)


def gray_frame(line_vals):
    """Per-row value -> flat row-major RGB (R=G=B=val) frame, H rows x W cols."""
    px = []
    for v in line_vals:
        rgb = (v << 16) | (v << 8) | v
        px.extend([rgb] * W)
    return px


def _row_mean(out_r, rr):
    """Mirror the TB row_mean: mean of columns [2 .. W-3] of output row rr (R channel)."""
    s = n = 0
    for i in range(rr * W + 2, min(rr * W + W - 2, len(out_r))):
        s += out_r[i]
        n += 1
    return (s // n) if n > 0 else -1


async def _set_cfg(dut, mode):
    """Program the two kernels + combiner config exactly as the TB initial block."""
    dut.a_coeffs.value = A_COEFFS
    dut.a_shift.value = A_SHIFT
    dut.a_en.value = 1
    dut.b_coeffs.value = B_COEFFS
    dut.b_shift.value = B_SHIFT
    dut.b_en.value = 1
    dut.cfg_alpha.value = 1
    dut.cfg_beta.value = 1
    dut.cfg_shift.value = 0
    dut.cfg_offset.value = 128
    dut.cfg_mode.value = mode


async def _drive_frame(dut, clk, drv, mon, line_vals, flush=48):
    """Mirror the TB drive_frame: continuous-valid frame + 48-cycle flush.

    Returns the R-channel of the output beats captured during (and flushed after) this
    frame -- the analogue of the TB's per-frame ocnt/out_r[] (reset each drive_frame).
    """
    base = len(mon.beats)
    await drv.send_frame(gray_frame(line_vals), W)
    await ClockCycles(clk, flush)
    return [(b["pixel"] >> 16) & 0xFF for b in mon.beats[base:]]


async def _bringup_chain(dut):
    clk, _ = await bringup(dut, clk="clk", rst="rst_n")
    drv = PixelStreamDriver(dut, clk)
    await drv.idle()
    mon = PixelMonitor(dut, clk)
    mon.start()
    return clk, drv, mon


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def dog_alignment_and_modes(dut):
    """The full TB scenario 1:1: warm-up, then modes 1/0/2 uniform + mode 2 on bands."""
    clk, drv, mon = await _bringup_chain(dut)

    uni = [100] * H
    bands = [40 if i < 6 else 200 for i in range(H)]

    errors = 0

    def expect_near(nm, got, exp, tol):
        nonlocal errors
        if got < exp - tol or got > exp + tol:
            cocotb.log.error(f"  FAIL {nm}: got {got} exp {exp} +/-{tol}")
            errors += 1

    # warm up the cold line buffers so the first asserted frame is clean (mode 1)
    await _set_cfg(dut, mode=1)
    await _drive_frame(dut, clk, drv, mon, uni)

    # --- 1) mode 1 (B passthrough) uniform -> 100 ---
    await _set_cfg(dut, mode=1)
    out_r = await _drive_frame(dut, clk, drv, mon, uni)
    cocotb.log.info(f"[mode1 B-pass uniform] got {len(out_r)} outputs")
    for i in range(5 * W, min((H - 3) * W, len(out_r))):
        expect_near("Bpass-uni", out_r[i], 100, 1)

    # --- 2) mode 0 (A passthrough, FIFO-aligned) uniform -> 100 ---
    await _set_cfg(dut, mode=0)
    out_r = await _drive_frame(dut, clk, drv, mon, uni)
    cocotb.log.info(f"[mode0 A-pass uniform] got {len(out_r)} outputs")
    for i in range(5 * W, min((H - 3) * W, len(out_r))):
        expect_near("Apass-uni", out_r[i], 100, 1)

    # --- 3) mode 2 (DoG) uniform -> 128 (flat = 0 + offset) ---
    await _set_cfg(dut, mode=2)
    out_r = await _drive_frame(dut, clk, drv, mon, uni)
    cocotb.log.info(f"[mode2 DoG uniform] got {len(out_r)} outputs")
    for i in range(5 * W, min((H - 3) * W, len(out_r))):
        expect_near("DoG-uni", out_r[i], 128, 1)

    # --- 4) mode 2 (DoG) on bands: flattest row ~128, strongest transition deviates ---
    await _set_cfg(dut, mode=2)
    out_r = await _drive_frame(dut, clk, drv, mon, bands)
    cocotb.log.info("[mode2 DoG bands 40/200] output row means (R):")
    mind, maxd = 999, 0
    for rr in range(1, H):
        rm = _row_mean(out_r, rr)
        d = abs(rm - 128)
        cocotb.log.info(f"   out row {rr} ~= {rm}  (|DoG|={d})")
        if d < mind:
            mind = d
        if d > maxd:
            maxd = d
    if mind > 3:
        cocotb.log.error(f"  FAIL DoG-flat: flattest row |DoG|={mind} (>3)")
        errors += 1
    if maxd < 30:
        cocotb.log.error(f"  FAIL DoG-edge: strongest row |DoG|={maxd} (<30)")
        errors += 1

    # --- 5) compare A-pass vs B-pass on bands (log only, as in the TB) ---
    await _set_cfg(dut, mode=0)
    out_r = await _drive_frame(dut, clk, drv, mon, bands)
    cocotb.log.info("[mode0 A-pass bands] row means (3x3 blur):")
    for rr in range(2, H - 2):
        cocotb.log.info(f"   out row {rr} ~= {_row_mean(out_r, rr)}")
    await _set_cfg(dut, mode=1)
    out_r = await _drive_frame(dut, clk, drv, mon, bands)
    cocotb.log.info("[mode1 B-pass bands] row means (5x5 blur, wider):")
    for rr in range(2, H - 2):
        cocotb.log.info(f"   out row {rr} ~= {_row_mean(out_r, rr)}")

    check(errors == 0, f"axis_rgb_dog alignment + DoG/passthrough ({errors} error(s))")


# --- additive: bit-exact STEADY-STATE check of the full chain against the composed golden -----
# The tolerance test above proves alignment; this proves the exact values. A leads B by one
# cycle (conv3x3 valid latency 5, conv5x5's 6), so once the pipelines fill the ordinal FIFO
# pairs the k-th A with the k-th B exactly -- dog_chain_golden composes conv3x3||conv5x5->combine
# and matches bit-for-bit. The first ~row is a cold-start FIFO/pipeline transient (why the TB
# above warms up), so -- exactly like img_file_uvm's multi-frame handling -- we stream TWO
# frames back-to-back and check the SECOND frame, where both the conv line buffers and the FIFO
# are in steady state. The golden carries state across frames (streaming, no reset), matching
# the RTL. Both Gaussians are symmetric, so the SV concat packing and the golden idx-order list
# coincide.

@cocotb.test(timeout_time=6, timeout_unit="ms")
@cocotb.parametrize(mode=[0, 1, 2, 3])
async def dog_chain_bitexact(dut, mode):
    clk, drv, mon = await _bringup_chain(dut)
    alpha, beta, shift, offset = 1, 1, 0, 32
    dut.a_coeffs.value = A_COEFFS
    dut.a_shift.value = A_SHIFT
    dut.a_en.value = 1
    dut.b_coeffs.value = B_COEFFS
    dut.b_shift.value = B_SHIFT
    dut.b_en.value = 1
    dut.cfg_alpha.value = alpha
    dut.cfg_beta.value = beta
    dut.cfg_shift.value = shift
    dut.cfg_offset.value = offset
    dut.cfg_mode.value = mode

    rng = random.Random((int(os.environ.get("COCOTB_SEED", "1"), 0) << 3) ^ (mode + 1))
    fsz = W * H
    frame1 = [rng.randrange(0x1000000) for _ in range(fsz)]
    frame2 = [rng.randrange(0x1000000) for _ in range(fsz)]
    stream = frame1 + frame2
    await drv.send_frame(stream, W)                 # two frames back-to-back, continuous valid
    await ClockCycles(clk, 64)

    got = [b["pixel"] for b in mon.beats]
    exp = G.dog_chain_golden(stream, W, A_COEFFS_LIST, A_SHIFT, B_COEFFS_LIST, B_SHIFT,
                             mode, alpha, beta, shift, offset)
    check(len(got) >= 2 * fsz, f"captured {len(got)} < {2 * fsz} beats")
    # compare the SECOND frame (steady state: cold-start transient is confined to frame 1)
    mism = [(fsz + i, f"{got[fsz + i]:06x}", f"{exp[fsz + i]:06x}")
            for i in range(fsz) if got[fsz + i] != exp[fsz + i]]
    check(not mism, f"dog chain mode={mode} bit-exact (frame2): "
                    f"{len(mism)} mismatch(es), first {mism[:3]}")


# --- synthesizable wrapper generated as a build artifact (single toplevel for Verilator) ---
_WRAPPER = r"""
`timescale 1ns / 1ps
`default_nettype none
// Auto-generated by test_axis_rgb_dog.py. Instantiates the three DoG DUTs exactly as
// verification/tb/tb_axis_rgb_dog.sv does (conv LINE_PIXELS=8 ENABLE=1; combine ENABLE=1
// DEPTH=64) and exposes the driving ports so cocotb can drive the chain top.
module dog_chain_top #(
    parameter int W = 8
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire [23:0]  in_pixel,
    input  wire         in_valid,
    input  wire         in_sof,
    input  wire         in_eol,
    input  wire         in_eof,
    input  wire         in_err,
    input  wire [71:0]  a_coeffs,
    input  wire [3:0]   a_shift,
    input  wire         a_en,
    input  wire [199:0] b_coeffs,
    input  wire [3:0]   b_shift,
    input  wire         b_en,
    input  wire [1:0]   cfg_mode,
    input  wire [7:0]   cfg_alpha,
    input  wire [7:0]   cfg_beta,
    input  wire [3:0]   cfg_shift,
    input  wire signed [8:0] cfg_offset,
    output wire [23:0]  out_pixel,
    output wire         out_valid,
    output wire         out_sof,
    output wire         out_eol,
    output wire         out_eof,
    output wire         out_err
);
    wire [23:0] a_pixel; wire a_valid, a_sof, a_eol, a_eof, a_err;
    wire [23:0] b_pixel; wire b_valid, b_sof, b_eol, b_eof, b_err;

    axis_rgb_conv3x3 #(.LINE_PIXELS(W), .ENABLE(1'b1)) uA (
        .clk(clk), .rst_n(rst_n), .cfg_en(a_en), .cfg_coeffs(a_coeffs),
        .cfg_shift(a_shift), .cfg_abs(1'b0),
        .in_pixel(in_pixel), .in_valid(in_valid), .in_sof(in_sof),
        .in_eol(in_eol), .in_eof(in_eof), .in_err(in_err),
        .out_pixel(a_pixel), .out_valid(a_valid), .out_sof(a_sof),
        .out_eol(a_eol), .out_eof(a_eof), .out_err(a_err)
    );
    axis_rgb_conv5x5 #(.LINE_PIXELS(W), .ENABLE(1'b1)) uB (
        .clk(clk), .rst_n(rst_n), .cfg_en(b_en), .cfg_coeffs(b_coeffs),
        .cfg_shift(b_shift), .cfg_abs(1'b0),
        .in_pixel(in_pixel), .in_valid(in_valid), .in_sof(in_sof),
        .in_eol(in_eol), .in_eof(in_eof), .in_err(in_err),
        .out_pixel(b_pixel), .out_valid(b_valid), .out_sof(b_sof),
        .out_eol(b_eol), .out_eof(b_eof), .out_err(b_err)
    );
    axis_rgb_dog_combine #(.ENABLE(1'b1), .DEPTH(64)) uC (
        .clk(clk), .rst_n(rst_n),
        .cfg_mode(cfg_mode), .cfg_alpha(cfg_alpha), .cfg_beta(cfg_beta),
        .cfg_shift(cfg_shift), .cfg_offset(cfg_offset),
        .a_pixel(a_pixel), .a_valid(a_valid),
        .b_pixel(b_pixel), .b_valid(b_valid),
        .b_sof(b_sof), .b_eol(b_eol), .b_eof(b_eof), .b_err(b_err),
        .out_pixel(out_pixel), .out_valid(out_valid), .out_sof(out_sof),
        .out_eol(out_eol), .out_eof(out_eof), .out_err(out_err)
    );
endmodule
`default_nettype wire
"""


def test_axis_rgb_dog():
    from runner_support import build_and_test

    test_dir = Path(__file__).resolve().parent
    wrapper = test_dir / "dog_chain_top_generated.sv"
    wrapper.write_text(_WRAPPER, encoding="utf-8")

    build_and_test(
        block="axis_rgb_dog",
        sources=[
            "rtl/img_proc/axis_rgb_conv3x3.sv",
            "rtl/img_proc/axis_rgb_conv5x5.sv",
            "rtl/img_proc/axis_rgb_dog_combine.sv",
            str(wrapper),
        ],
        toplevel="dog_chain_top",
        test_module="test_axis_rgb_dog",
        test_dir=test_dir,
        parameters={"W": W},
        engine="verilator",
    )
