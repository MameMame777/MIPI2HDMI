"""cocotb port of verification/tb/tb_e2e_vdma_stacking.sv (E2E frame-assembly / VDMA-stack).

The DSim TB chains the real assembly RTL:

    in_pkt/in_payload
      -> csi2_frame_state          (SOF/EOF/SOL/EOL + FS/FE framing, lsle mode)
      -> yuv422_gray_unpack        (YUV422 byte stream -> Y8 pixel, LINE_PIXELS=0)
      -> video_frame_normalizer    (pin every frame to VSIZE x LINE_PIXELS, NORMALIZE=1)
      -> axis_video_bridge         (dual-clock CDC -> AXI4-Stream, tuser=SOF, tlast=EOL)

and feeds the bridge AXIS output into two *behavioural* AXI-VDMA S2MM write models coded
in the TB (not RTL): a GENLOCK model (resets line_ptr on SOF) and a FREE-RUN model
(counts VSIZE lines and wraps, ignoring SOF). The question the TB answers: does a
free-running VDMA "stack"/tile a source frame that is shorter than the VDMA buffer, and
does the normalizer (pinning frame == VSIZE) remove that stacking?

cocotb needs a single HDL toplevel, so the four-DUT wiring is emitted as a tiny wrapper
module (``e2e_harness``, written at build time) containing ONLY the four DUT instances
(no ``initial``, no clock) so cocotb owns the clocks/reset and stimulus. The two VDMA write
models + the marker counters (the TB ``always_ff`` blocks) become cocotb monitor coroutines
on the aclk domain, replicating the register-transfer semantics 1:1.

Config mirrors the DSim TB exactly: FRAME_LINES=11 source lines into a VSIZE=44-line VDMA
frame, N_FRAMES=8 unique-ramp frames (line k -> Y = 0x10 + k). NORMALIZE_EN=1, SPURIOUS_FS=0.

The load-bearing conclusions of the TB (its DIAGNOSIS block) become ``check()``s:
  * frame_state assembles exactly N_FRAMES frames (fs_fcnt == 8);
  * RTL framing is CLEAN -> exactly 1 SOF per frame (sof_pulses == fs_fcnt);
  * with the normalizer ON the FREE-RUN VDMA shows <= 1 frame-top copy => NO stacking;
  * the GENLOCK VDMA reproduces each source line's unique value in the buffer.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import start_clock  # noqa: E402
from lib.scoreboard import check  # noqa: E402

# --- TB localparams (1:1) -------------------------------------------------------------
LINE_PIXELS = 8
LINE_BYTES = LINE_PIXELS * 2       # YUV422: 2 bytes / Y
FRAME_LINES = 11                   # source frame height (chip)
VSIZE = 44                         # VDMA frame height
N_FRAMES = 8
BASE = 0x10                        # line k -> Y = BASE + k

DT_FS, DT_FE, DT_LS, DT_LE = 0x00, 0x01, 0x02, 0x03
DT_YUV = 0x1E


# --- SV stimulus tasks, ported 1:1 (NBA-on-posedge -> drive on RisingEdge) -------------

async def drv_short(dut, clk, dt):
    """Port of drv_short: a 1-cycle short packet, DI={2'b00,dt}, is_short/start/end=1."""
    await RisingEdge(clk)
    dut.in_pkt_di.value = dt & 0x3F
    dut.in_pkt_wc.value = 0
    dut.in_pkt_is_short.value = 1
    dut.in_pkt_is_long.value = 0
    dut.in_pkt_start.value = 1
    dut.in_pkt_end.value = 1
    await RisingEdge(clk)
    dut.in_pkt_start.value = 0
    dut.in_pkt_end.value = 0
    dut.in_pkt_is_short.value = 0


async def drv_line(dut, clk, yval):
    """Port of drv_line: LS short, one YUV422 long packet (odd byte=Y=yval, even=0x80),
    then LE short. LINE_BYTES payload bytes, wc=LINE_BYTES."""
    await drv_short(dut, clk, DT_LS)
    await RisingEdge(clk)
    dut.in_pkt_di.value = DT_YUV
    dut.in_pkt_wc.value = LINE_BYTES
    dut.in_pkt_is_short.value = 0
    dut.in_pkt_is_long.value = 1
    dut.in_pkt_start.value = 1
    await RisingEdge(clk)
    dut.in_pkt_start.value = 0
    for b in range(LINE_BYTES):
        dut.in_payload_data.value = (yval & 0xFF) if (b & 1) else 0x80
        dut.in_payload_valid.value = 1
        dut.in_payload_first.value = 1 if b == 0 else 0
        dut.in_payload_last.value = 1 if b == LINE_BYTES - 1 else 0
        await RisingEdge(clk)
    dut.in_payload_valid.value = 0
    dut.in_payload_first.value = 0
    dut.in_payload_last.value = 0
    dut.in_pkt_end.value = 1
    await RisingEdge(clk)
    dut.in_pkt_end.value = 0
    dut.in_pkt_is_long.value = 0
    await drv_short(dut, clk, DT_LE)


async def drv_frame(dut, clk, nlines, base, spurious_fs=False):
    """Port of drv_frame: FS, nlines lines (line k -> Y=base+k), FE. With spurious_fs an
    extra in-frame FS is injected at mid-frame (must be ignored under FE_DELIMITS)."""
    await drv_short(dut, clk, DT_FS)
    for k in range(nlines):
        if spurious_fs and k == nlines // 2:
            await drv_short(dut, clk, DT_FS)
        await drv_line(dut, clk, (base + k) & 0xFF)
    await drv_short(dut, clk, DT_FE)


# --- behavioural counters + VDMA write models (the SV always_ff blocks) ----------------

class MarkerCounters:
    """Port of the core_clk always_ff logger: unpack-out + normalizer-out SOF/EOF counts."""

    def __init__(self, dut, clk):
        self.dut = dut
        self.clk = clk
        self.up_sof_n = 0
        self.up_eof_n = 0
        self.nm_sof_n = 0
        self.nm_eof_n = 0

    def start(self):
        return cocotb.start_soon(self._run())

    async def _run(self):
        d = self.dut
        while True:
            await RisingEdge(self.clk)
            up_v = int(d.up_v.value)
            if up_v and int(d.up_sof.value):
                self.up_sof_n += 1
            if int(d.up_eof.value):
                self.up_eof_n += 1
            nm_v = int(d.nm_v.value)
            if nm_v and int(d.nm_sof.value):
                self.nm_sof_n += 1
            if nm_v and int(d.nm_eof.value):
                self.nm_eof_n += 1


class VdmaModels:
    """Port of the two aclk always_ff VDMA write models.

    GENLOCK: on SOF (tuser[0]) reset line_ptr=0; write buf[line_ptr][col]=tdata; on
    TLAST (EOL) advance line_ptr (capped at VSIZE-1).
    FREE-RUN: write buf[fline][fcol]=tdata; on TLAST advance fline with wrap at VSIZE
    (ignoring SOF). Both sample on posedge aclk while br_tvalid & br_tready.
    """

    def __init__(self, dut, clk):
        self.dut = dut
        self.clk = clk
        self.vbuf = [[None] * LINE_PIXELS for _ in range(VSIZE)]
        self.fbuf = [[None] * LINE_PIXELS for _ in range(VSIZE)]
        self.line_ptr = 0
        self.col_ptr = 0
        self.sof_pulses = 0
        self.eol_pulses = 0
        self.total_px = 0
        self.fline = 0
        self.fcol = 0

    def start(self):
        return cocotb.start_soon(self._run())

    async def _run(self):
        d = self.dut
        while True:
            await RisingEdge(self.clk)
            if not (int(d.br_tvalid.value) and int(d.br_tready.value)):
                continue
            tdata = int(d.br_tdata.value) & 0xFF
            tuser0 = int(d.br_tuser.value) & 0x1
            tlast = int(d.br_tlast.value)

            # ---- GENLOCK model ----
            if tuser0:                        # SOF: VDMA frame sync -> top of buffer
                self.line_ptr = 0
                self.col_ptr = 0
                self.sof_pulses += 1
            if self.line_ptr < VSIZE and self.col_ptr < LINE_PIXELS:
                self.vbuf[self.line_ptr][self.col_ptr] = tdata
            self.total_px += 1
            if tlast:                         # EOL: next line
                self.eol_pulses += 1
                self.col_ptr = 0
                if self.line_ptr < VSIZE - 1:
                    self.line_ptr += 1
            else:
                if self.col_ptr < LINE_PIXELS - 1:
                    self.col_ptr += 1

            # ---- FREE-RUN model (no SOF resync; wraps at VSIZE) ----
            if self.fline < VSIZE and self.fcol < LINE_PIXELS:
                self.fbuf[self.fline][self.fcol] = tdata
            if tlast:
                self.fcol = 0
                self.fline = 0 if self.fline == VSIZE - 1 else self.fline + 1
            elif self.fcol < LINE_PIXELS - 1:
                self.fcol += 1


def _count_tiles(buf):
    """Count frame-top copies = rows whose col[1] == BASE (0x10), matching the SV loops
    over vbuf[k][1] / fbuf[k][1] === 8'h10."""
    return sum(1 for k in range(VSIZE) if buf[k][1] == BASE)


async def _idle_inputs(dut):
    dut.in_pkt_di.value = 0
    dut.in_pkt_wc.value = 0
    dut.in_pkt_is_short.value = 0
    dut.in_pkt_is_long.value = 0
    dut.in_pkt_start.value = 0
    dut.in_pkt_end.value = 0
    dut.in_pkt_err.value = 0
    dut.in_payload_data.value = 0
    dut.in_payload_valid.value = 0
    dut.in_payload_first.value = 0
    dut.in_payload_last.value = 0


# --- the test: replicate the single SV initial run 1:1 --------------------------------

@cocotb.test(timeout_time=5, timeout_unit="ms")
async def e2e_frame_assembly_no_stacking(dut):
    core_clk = dut.core_clk
    aclk = dut.aclk

    # Two async clocks: core_clk #5 (10 ns), aclk #7 (14 ns) -- exactly the SV periods.
    start_clock(core_clk, 10.0)
    start_clock(aclk, 14.0)

    # Reset both domains (SV: core_aresetn=0, aresetn=0, cfg_use_lsle=1; 10 core cycles).
    dut.core_aresetn.value = 0
    dut.aresetn.value = 0
    dut.cfg_use_lsle.value = 1
    dut.br_tready.value = 1          # SV TB: assign br_tready = 1'b1 (VDMA always ready)
    await _idle_inputs(dut)
    for _ in range(10):
        await RisingEdge(core_clk)
    dut.core_aresetn.value = 1
    dut.aresetn.value = 1
    for _ in range(4):
        await RisingEdge(core_clk)

    markers = MarkerCounters(dut, core_clk)
    markers.start()
    vdma = VdmaModels(dut, aclk)
    vdma.start()

    # Drive N_FRAMES frames, each FRAME_LINES lines (line k -> Y=0x10+k), with a generous
    # inter-frame gap (>> the normalizer tail-pad burst) so the padded frame drains.
    for _f in range(N_FRAMES):
        await drv_frame(dut, core_clk, FRAME_LINES, BASE)
        for _ in range(1600):
            await RisingEdge(core_clk)
    for _ in range(4000):
        await RisingEdge(core_clk)

    fs_fcnt = int(dut.fs_fcnt.value)
    fs_lastlines = int(dut.fs_lastlines.value)
    fs_syncerr = int(dut.fs_syncerr.value)
    gl_tiles = _count_tiles(vdma.vbuf)
    fr_tiles = _count_tiles(vdma.fbuf)

    dut._log.info(
        "frame_state: frames=%d last_lines=%d sync_err=%d (src=%d lines, VSIZE=%d)",
        fs_fcnt, fs_lastlines, fs_syncerr, FRAME_LINES, VSIZE)
    ratio = (vdma.eol_pulses / vdma.sof_pulses) if vdma.sof_pulses else 0.0
    dut._log.info(
        "bridge AXIS: sof_pulses=%d eol_pulses=%d (=> %.1f EOL/SOF; clean => %d)",
        vdma.sof_pulses, vdma.eol_pulses, ratio, VSIZE)
    dut._log.info(
        "markers: unpack sof=%d eof=%d | normalizer sof=%d eof=%d (fs frames=%d)",
        markers.up_sof_n, markers.up_eof_n, markers.nm_sof_n, markers.nm_eof_n, fs_fcnt)
    dut._log.info("GENLOCK  VDMA frame-top copies = %d", gl_tiles)
    dut._log.info("FREE-RUN VDMA frame-top copies = %d", fr_tiles)

    # ---- checks (the TB DIAGNOSIS conclusions, made assertive) ----

    # (1) frame_state assembled exactly the frames we drove.
    check(fs_fcnt == N_FRAMES,
          f"frame_state assembled N_FRAMES frames (fs_fcnt={fs_fcnt}, exp {N_FRAMES})")

    # (2) the bridge emitted a SOF-delimited AXIS frame (pixels reached the CDC output).
    check(vdma.sof_pulses > 0 and vdma.total_px > 0,
          f"bridge emitted AXIS beats (sof_pulses={vdma.sof_pulses}, "
          f"total_px={vdma.total_px}, eol_pulses={vdma.eol_pulses})")

    # marker sanity: the whole assembly chain is clean end-to-end (1 SOF/EOF per frame at
    # unpack out and normalizer out) -- the TB "markers:" diagnostic line.
    check(markers.up_sof_n == N_FRAMES and markers.up_eof_n == N_FRAMES,
          f"unpack out: 1 SOF/EOF per frame (sof={markers.up_sof_n}, eof={markers.up_eof_n})")
    check(markers.nm_sof_n == N_FRAMES and markers.nm_eof_n == N_FRAMES,
          f"normalizer out: 1 SOF/EOF per frame "
          f"(sof={markers.nm_sof_n}, eof={markers.nm_eof_n})")

    # the normalizer pins every frame to exactly VSIZE lines -> VSIZE EOL per SOF at the
    # AXIS output (TB "bridge AXIS: %0.1f EOL per SOF; clean => VSIZE").
    check(vdma.eol_pulses == VSIZE * vdma.sof_pulses,
          f"normalizer pins frame == VSIZE lines "
          f"(eol_pulses={vdma.eol_pulses}, exp {VSIZE * vdma.sof_pulses})")

    # (3) RTL framing is CLEAN: exactly one SOF per assembled frame (TB DIAGNOSIS line
    #     "RTL framing CLEAN: exactly 1 SOF per frame").
    check(vdma.sof_pulses == fs_fcnt,
          f"exactly 1 SOF per frame (sof_pulses={vdma.sof_pulses}, fs_fcnt={fs_fcnt})")

    # (4) With the normalizer ON (NORMALIZE_EN=1, frame pinned to VSIZE), a FREE-RUN VDMA
    #     shows at most one frame-top copy => NO stacking/tiling (TB DIAGNOSIS
    #     "FREE-RUN VDMA shows %d frame copy => NO stacking").
    check(fr_tiles <= 1,
          f"normalizer prevents free-run stacking (fr_tiles={fr_tiles}, expected <=1)")

    # (5) The GENLOCK VDMA reproduces the frame top (line-0 unique value BASE) exactly once
    #     -- the genlocked buffer holds one clean copy of the (normalized) frame.
    check(gl_tiles >= 1,
          f"genlock VDMA holds the frame top (gl_tiles={gl_tiles}, expected >=1)")

    # (6) The GENLOCK buffer reproduces each source line's UNIQUE value: for the first
    #     FRAME_LINES rows, buf[k][1] == BASE + k (vertically-unique stimulus, so any
    #     vertical repetition/stacking inside the genlocked frame would be visible).
    for k in range(FRAME_LINES):
        got = vdma.vbuf[k][1]
        check(got == (BASE + k) & 0xFF,
              f"genlock buf row {k} carries unique src value (got {got}, "
              f"exp {(BASE + k) & 0xFF})")


# --- build harness: emit the 4-DUT wiring wrapper, then build+run under Verilator ------

_HARNESS = r"""
`timescale 1ns / 1ps
`default_nettype none
// Auto-generated E2E wrapper for the cocotb port of tb_e2e_vdma_stacking.sv.
// Contains ONLY the four DUT instances (no initial / no clock / no VDMA model) so cocotb
// owns clk/rst, stimulus, and the behavioural VDMA write models. Wiring + parameters are
// 1:1 with the DSim TB. NORMALIZE_EN=1, br_tready tied high (VDMA always ready).
module e2e_harness #(
    parameter int LINE_PIXELS = 8,
    parameter int LINE_BYTES  = 16,
    parameter int FRAME_LINES = 11,
    parameter int VSIZE       = 44
)(
    input  wire        core_clk,
    input  wire        core_aresetn,
    input  wire        aclk,
    input  wire        aresetn,
    input  wire        cfg_use_lsle,

    // frame_state packet inputs
    input  wire [7:0]  in_pkt_di,
    input  wire [15:0] in_pkt_wc,
    input  wire        in_pkt_is_short,
    input  wire        in_pkt_is_long,
    input  wire        in_pkt_start,
    input  wire        in_pkt_end,
    input  wire        in_pkt_err,
    input  wire [7:0]  in_payload_data,
    input  wire        in_payload_valid,
    input  wire        in_payload_first,
    input  wire        in_payload_last,

    // unpack-out markers (for the SOF/EOF counters)
    output wire        up_v,
    output wire        up_sof,
    output wire        up_eof,
    // normalizer-out markers
    output wire        nm_v,
    output wire        nm_sof,
    output wire        nm_eof,

    // bridge AXIS output (aclk side) -> VDMA models
    output wire [7:0]  br_tdata,
    output wire        br_tvalid,
    output wire        br_tlast,
    output wire        br_tuser,
    input  wire        br_tready,

    // frame_state status
    output wire [31:0] fs_fcnt,
    output wire [15:0] fs_lastlines,
    output wire [15:0] fs_syncerr
);
    // frame_state I/O
    wire        fs_sof, fs_eof, fs_sol, fs_eol, fs_in_frame;
    wire [15:0] fs_line_idx;
    wire [7:0]  fs_pd; wire fs_pv, fs_pf, fs_pl, fs_ferr;
    wire [31:0] fs_lcnt;
    wire [15:0] fs_dbg_la, fs_dbg_nols, fs_dbg_idle;
    wire [127:0] fs_dbg_hist;

    csi2_frame_state #(
        .MAX_LINES(64), .GUARD_FRAME_LINES(1'b1), .EXPECTED_FRAME_LINES(FRAME_LINES),
        .EXPECTED_LINE_WC(16'(LINE_BYTES)), .FS_MIN_LINES(4), .FE_DELIMITS(1'b1)
    ) u_fs (
        .core_clk(core_clk), .core_aresetn(core_aresetn), .cfg_use_lsle(cfg_use_lsle),
        .cfg_expected_frame_lines(16'd0),
        .cfg_sof_synth(1'b0), .cfg_force_expected(1'b0), .cfg_long_as_line(1'b0),
        .in_pkt_di(in_pkt_di), .in_pkt_wc(in_pkt_wc), .in_pkt_is_short(in_pkt_is_short),
        .in_pkt_is_long(in_pkt_is_long), .in_pkt_start(in_pkt_start), .in_pkt_end(in_pkt_end),
        .in_pkt_err(in_pkt_err), .in_payload_data(in_payload_data),
        .in_payload_valid(in_payload_valid),
        .in_payload_first(in_payload_first), .in_payload_last(in_payload_last),
        .out_sof(fs_sof), .out_eof(fs_eof), .out_sol(fs_sol), .out_eol(fs_eol),
        .out_in_frame(fs_in_frame),
        .out_line_idx(fs_line_idx), .out_payload_data(fs_pd), .out_payload_valid(fs_pv),
        .out_payload_first(fs_pf), .out_payload_last(fs_pl), .out_frame_err(fs_ferr),
        .sts_frame_count(fs_fcnt), .sts_line_count(fs_lcnt),
        .sts_last_frame_lines(fs_lastlines), .sts_frame_sync_err_cnt(fs_syncerr),
        .sts_dbg_long_accept(fs_dbg_la), .sts_dbg_long_nols(fs_dbg_nols),
        .sts_dbg_long_idle(fs_dbg_idle), .sts_dbg_nols_hist(fs_dbg_hist)
    );

    // unpack I/O
    wire [23:0] up_pixel; wire up_eol, up_err; wire [15:0] up_ppl;
    yuv422_gray_unpack #(.LINE_PIXELS(0)) u_up (
        .core_clk(core_clk), .core_aresetn(core_aresetn),
        .in_sof(fs_sof), .in_eof(fs_eof), .in_eol(fs_eol),
        .in_payload_data(fs_pd), .in_payload_valid(fs_pv),
        .in_payload_first(fs_pf), .in_payload_last(fs_pl), .in_frame_err(fs_ferr),
        .out_pixel(up_pixel), .out_pixel_valid(up_v), .out_pixel_sof(up_sof),
        .out_pixel_eol(up_eol), .out_pixel_eof(up_eof), .out_pixel_err(up_err),
        .sts_pixel_per_line(up_ppl)
    );

    // frame normalizer: pin every frame to exactly VSIZE x LINE_PIXELS
    wire [7:0] nm_data; wire nm_err;
    video_frame_normalizer #(.OUT_LINES(VSIZE), .OUT_PIXELS(LINE_PIXELS),
                             .FILL(8'h00), .NORMALIZE(1'b1)) u_norm (
        .clk(core_clk), .aresetn(core_aresetn),
        .in_data(up_pixel[7:0]), .in_valid(up_v), .in_sof(up_sof),
        .in_eol(up_eol), .in_eof(up_eof), .in_err(up_err),
        .out_data(nm_data), .out_valid(nm_v), .out_sof(nm_sof),
        .out_eol(nm_eol), .out_eof(nm_eof), .out_err(nm_err)
    );
    wire nm_eol;

    // bridge (8-bit data, 1-bit tuser=SOF)
    wire [15:0] br_ovf, br_bp;
    wire [0:0]  br_tuser_w;
    axis_video_bridge #(.TDATA_WIDTH(8), .TUSER_WIDTH(1), .FIFO_DEPTH(4096),
                        .AXIS_TUSER_ERR_DEBUG(1'b0)) u_br (
        .core_clk(core_clk), .core_aresetn(core_aresetn), .aclk(aclk), .aresetn(aresetn),
        .in_pixel(nm_data), .in_pixel_valid(nm_v), .in_pixel_sof(nm_sof),
        .in_pixel_eol(nm_eol), .in_pixel_eof(nm_eof), .in_pixel_err(nm_err),
        .m_axis_tdata(br_tdata), .m_axis_tvalid(br_tvalid), .m_axis_tready(br_tready),
        .m_axis_tlast(br_tlast), .m_axis_tuser(br_tuser_w),
        .sts_fifo_overflow_cnt(br_ovf), .sts_back_pressure_cnt(br_bp)
    );
    assign br_tuser = br_tuser_w[0];
endmodule
`default_nettype wire
"""


def test_e2e_vdma_stacking():
    from runner_support import build_and_test

    here = Path(__file__).resolve().parent
    harness = here / "e2e_harness.sv"
    harness.write_text(_HARNESS, encoding="ascii")

    build_and_test(
        block="e2e_vdma_stacking",
        sources=[
            "rtl/mipi_rx/csi2_frame_state.sv",
            "rtl/img_proc/yuv422_gray_unpack.sv",
            "rtl/img_proc/video_frame_normalizer.sv",
            "rtl/mipi_rx/axis_video_bridge.sv",
            str(harness),
        ],
        toplevel="e2e_harness",
        test_module="test_e2e_vdma_stacking",
        test_dir=here,
        parameters={
            "LINE_PIXELS": LINE_PIXELS,
            "LINE_BYTES": LINE_BYTES,
            "FRAME_LINES": FRAME_LINES,
            "VSIZE": VSIZE,
        },
        engine="verilator",
    )
