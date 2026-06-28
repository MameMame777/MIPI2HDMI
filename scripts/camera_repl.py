#!/usr/bin/env python3
"""Interactive camera-control REPL (OV5640 / Pcam 5C -> HDMI).

Consolidates the common camera-control operations (bring-up, lock, register R/W,
debug, live HDMI, still capture, focus/gain knobs) into one object `cam`, so you
don't need the scattered one-off scripts for interactive work. The board session
(chip lock, VDMA) persists across commands.

Run ON THE BOARD with an interactive Python (the launcher scripts/camera_repl.ps1
does the upload + ssh):

    sudo /usr/local/share/pynq-venv/bin/python3 -i camera_repl.py

then at the >>> prompt:

  bring-up / status:
    cam.go()            # full verified bring-up: chip init + RGB565 arm + HW lock
    cam.status()        # chip ID + FSM lock + link snapshot
    cam.diagnostics()   # full health report (chip + FSM + link + accounting + verdict)

  live HDMI + image processing (Phase 2 slot -- runtime, no rebuild):
    cam.hdmi(60)        # live HDMI for 60 s, blocking (auto-stops the VDMA -- safe)
    cam.hdmi_on()       # live HDMI, NON-blocking -> switch processing live, then cam.stop()
    cam.proc(n)         # slot op: 0=pass 1=invert 2=gray 3=BGR 4=thresh 5/6/7=R/G/B ; n>=8 = conv
    cam.k('sharpen')    # named 3x3 conv kernel (applies + enters conv mode)
    cam.kernels()       # list named kernels (identity/gaussian/sharpen/sharpen_hi/
                        #   sobel_x/sobel_y/laplacian/outline/emboss)
    cam.kernel(c,s)     # custom 3x3 conv: 9 signed coeffs row-major + right-shift
    cam.dog('blob')     # DoG dual-kernel op12 (3x3 vs 5x5; cam.dogs() lists) -- DoG bitstream
    cam.blur(13)        # cascade variable Gaussian blur: size 5/9/13 = eff 5x5/9x9/13x13
    cam.sharpen(0x20)   # CHIP CIP edge sharpen (source-side, runtime; 0=auto)
    cam.k_gauss() / cam.k_sobel() / cam.k_sharpen() / cam.k_emboss()   # shortcuts
    cam.capture()       # grab one still -> _capture/ (colour-aware)

  registers / debug:
    cam.read(0x300A)    # SCCB read    | cam.write(0x4800, 0x14)
    cam.dbg(0x18)       # debug page   | cam.link()  -> measure_link
    cam.vcm(280)        # focus DAC    | cam.idelay(16,16) | cam.settle(8)
    cam.window(dx,dy)   # PAN output window to re-centre on lens axis (0,0=reset)
    cam.gain_ceiling(c) # cap AGC gain (cuts low-light noise) | cam.testpattern()
    cam.regs()          # chip register dump   | cam.pages()    # debug page scan
    cam.accounting()    # frame-sync drop split | cam.eye()      # IDELAY eye scan
    cam.lockstats()     # lock stability (real death vs frozen counter)
    cam.stop()          # stop the VDMA (safety) | cam.help()  list commands

SAFETY: cam.hdmi()/cam.capture() drive the VDMA. They auto-stop on return AND on
exit/Ctrl-C/disconnect (signal handlers + atexit). Do NOT kill the python process
mid-VDMA by other means -- that hangs sshd (physical power cycle).
"""
from __future__ import annotations
import atexit
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from pynq import MMIO, allocate
from pynq_bringup import setup_session
import v65_capture as v65
from v65_capture import (install_vdma_cleanup_signals, configure_vdma_s2mm,
                         stop_vdma, HEIGHT, STRIDE, WIDTH, CONV_KERNELS, DOG_PRESETS,
                         GAUSS5, GAUSS1D)
import frame_height_stability as fhs


# Point-op name -> code (axis_rgb_proc_slot op, used for pre/post stages).
_POINT_OPS = {'pass': 0, 'none': 0, 'off': 0, 'colour': 0, 'color': 0,
              'invert': 1, 'gray': 2, 'grey': 2, 'bgr': 3,
              'thresh': 4, 'threshold': 4, 'binarize': 4, 'binary': 4,
              'r': 5, 'red': 5, 'g': 6, 'green': 6, 'b': 7, 'blue': 7}

# Named filter COMBINATIONS over the live chain: pre point-op -> mid spatial filter ->
# post point-op (all runtime, no rebuild). mid: None (point-only) | 'edges' | a CONV_KERNELS
# name as ('kernel', name) | ('dog', preset) | ('blur', size). pre/post: point-op code/name;
# *_thresh = the level used when that op is 'thresh' (4). Apply with cam.pipeline(name).
PIPELINES = {
    'colour':      dict(pre=0, mid=None, post=0),
    'invert':      dict(pre='invert', mid=None),
    'binarize':    dict(pre='thresh', mid=None, pre_thresh=128),
    'edges':       dict(mid='edges'),
    'bin_edges':   dict(pre='thresh', mid='edges', pre_thresh=128),     # 2値化 -> Sobel (contours)
    'edge_binary': dict(mid='edges', post='thresh', post_thresh=64),    # Sobel -> 2値化 (edge map)
    'sketch':      dict(pre='gray', mid='edges', post='thresh', post_thresh=64),  # gray->edges->binarize
    'gray_edges':  dict(pre='gray', mid='edges'),
    'sharpen':     dict(mid=('kernel', 'sharpen')),
    'emboss':      dict(mid=('kernel', 'emboss')),
    'blur':        dict(mid=('blur', 9)),
    'dog_blob':    dict(mid=('dog', 'blob')),
    'median':        dict(pre='median', mid=None),            # 3x3 median denoise only
    'gaussian':      dict(pre='gaussian', mid=None),          # 3x3 gaussian blur only
    'denoise_edges': dict(pre='median', mid='edges'),         # median -> Sobel (clean edges)
    'median_sketch': dict(pre='median', mid='edges', post='thresh', post_thresh=64),    # median->Sobel->binarize
    'smooth_sketch': dict(pre='gaussian', mid='edges', post='thresh', post_thresh=64),  # blur->edge->binarize
    'halftone':    dict(pre='gray', mid=None, dither=(1, 'ordered')),       # gray -> 1-bit newspaper halftone
    'poster':      dict(mid=None, dither=(2, 'ordered')),                   # colour -> 2-bit posterize
    'edge_halftone': dict(pre='gray', mid='edges', dither=(1, 'ordered')),  # gray -> Sobel -> halftone
    'dither_random': dict(mid=None, dither=(2, 'random')),                  # colour -> random 2-bit dither
}


def _point_code(v):
    """Map a point-op spec (int 0-7 or name) to its op code (POST stage)."""
    if v is None:
        return 0
    if isinstance(v, int):
        return v & 0x7
    return _POINT_OPS[str(v).lower()]

# PRE stage (axis_rgb_prefilter) superset: point ops + 3x3 spatial denoise (8/9).
_PRE_OPS = dict(_POINT_OPS, gaussian=8, gauss=8, median=9, med=9)


def _pre_code(v):
    """Map a PRE-stage spec (int 0-9 or name incl 'gaussian'/'median') to its code."""
    if v is None:
        return 0
    if isinstance(v, int):
        return v & 0xF
    return _PRE_OPS[str(v).lower()]


class Cam:
    """Camera-control facade over the shared modules. One instance = `cam`."""

    def __init__(self, ol, h):
        self.ol, self.h = ol, h
        self._vdma = None
        self._bufs = None
        self._val4800 = 0x14
        install_vdma_cleanup_signals()       # SIGTERM/SIGHUP -> stop VDMA (sshd-hang guard)
        atexit.register(self.stop)

    # ---- bring-up -----------------------------------------------------------
    def init(self, val4800=0x14, sup=False, synth=True, force=True):
        """Fresh chip init (RESETB pulse + SW reset + full SCCB replay) + frame
        config + eye-centre IDELAY. Default 0x4800=0x14 (continuous + line-sync):
        gives the healthy fs=fe=30, constant-height non-rolling stream (SOF-synth +
        force-480) -- the working path, and the one a VDMA still grab needs to stay
        untiled. (0x24 = no-LS makes the frame height unstable, fs~2 -> tiled grabs.)"""
        steps = fhs.patch_init_steps(list(fhs.FULL_INIT_STEPS), [(0x4800, val4800)])
        v65.chip_init(self.h, steps, 'repl-init', settle_s=10.0)
        self.h['idelay_set'](16, 16)
        self.h['frame_lines_set_keep_cam'](
            value=480, use_lsle=(val4800 & 0x10 != 0), expected_dt=0x22,
            sup_enable=sup, sof_synth=synth, force_expected=force)
        self._val4800 = val4800
        print(f'  chip ID = {self.chip_id():04X}  (init 0x4800={val4800:#04x})')
        return self

    def arm(self, settle_blank=14):
        """RGB565 arm (0x300E stream cycle: 0x4300=0x6F + 0x501F=0x01) + the
        byte-domain settle-blank band fix. K is in byte_clk cycles: K=14 for the
        30fps build (byte_clk 96MHz); use K=8 for the 17fps build (84MHz)."""
        fhs.stream_cycle_write(self.h, list(fhs.ARM_REGS))
        self.h['set_settle_blank'](int(settle_blank))
        time.sleep(1.0)
        return self

    def hwlock(self, on=True, wait=15.0):
        """Enable the HW deterministic-lock FSM and wait for HOLD (page 0x2e)."""
        self.h['set_hw_lock'](bool(on))
        if not on:
            print('  HW lock FSM inhibited'); return None
        t0 = time.time()
        while time.time() - t0 < wait:
            s = self.h['read_hwlock']()
            if s['locked']:
                print(f'  HW-locked bitslip=({s["p0"]},{s["p1"]}) reroll={s["reroll"]} '
                      f't={time.time()-t0:.1f}s'); return s
            if s['failed']:
                print('  *** HW lock FSM FAILED (power-cycle if long stays 0) ***'); return s
            time.sleep(0.4)
        print('  (no lock within %.0fs)' % wait); return self.h['read_hwlock']()

    def lock(self, rerolls=8, settle_blank=14):
        """Software lock_mode (8x8 bitslip sweep + /4 re-roll). Inhibits the FSM
        first so manual bitslip applies. settle_blank is applied AFTER the lock
        (lock_mode's own default is 8 = the 17fps band fix; pass 14 for the 30fps
        96MHz byte_clk build, else short frames return -> jerky live)."""
        from bitslip_lock import lock_mode
        self.h['set_hw_lock'](False); time.sleep(0.3)
        return lock_mode(self.h, rerolls, settle_blank=settle_blank)

    def go(self, val4800=0x14, hw=True, settle_blank=14):
        """Full verified bring-up: init -> arm -> lock -> status.
        Default uses the baked HW-lock FSM (hw=True): on this build it HOLD-locks
        deterministically on boot (e.g. bitslip=(3,3), state=HOLD, fs=fe=30,
        last_frame_lines=480) with no software sweep -- faster, power-on only.
        (The older "HW-lock BOGUS at 96MHz / fs=0 white screen" was an earlier build;
        the refined FSM holds here -- verified 2026-06-27.) Pass hw=False for the
        SOFTWARE lock_mode (8x8 bitslip sweep scored by long packets) as a fallback.
        settle_blank=14 is applied AFTER the lock (96MHz band fix); the software sweep
        runs with blank=0. Verify with cam.status(): trust fs/fe, last_frame_lines, the
        FSM state and 'err totals' -- long_pkt/crc_ok/ls can read 0 in some configs even
        when the link is healthy (the image is the ground truth)."""
        self.init(val4800); time.sleep(0.3)
        self.arm(settle_blank=0); time.sleep(1.0)   # lock with blank=0; band fix applied post-lock
        if hw:
            self.hwlock(True)
            self.h['set_settle_blank'](settle_blank)
        else:
            r = self.lock(settle_blank=settle_blank)
            print('  software-locked' if r == 0 else '  *** software lock FAILED ***')
        self.status()
        return self

    # ---- inspection ---------------------------------------------------------
    def chip_id(self):
        return (self.h['sccb_read'](0x300A) << 8) | self.h['sccb_read'](0x300B)

    def read(self, addr):
        v = self.h['sccb_read'](addr); print(f'  0x{addr:04X} = 0x{v:02X}'); return v

    def write(self, addr, val):
        self.h['sccb_write'](addr, val); print(f'  0x{addr:04X} <- 0x{val:02X}')

    def dbg(self, page):
        v = self.h['read_dbg'](page); print(f'  page 0x{page:02X} = 0x{v:08X}'); return v

    def link(self, dur=5.0):
        """measure_link: fs/fe/long(rate)/crc%/last_frame_lines (liveness by fs/last_fe)."""
        return fhs.measure_link(self.h, dur=dur, label='repl')

    def status(self):
        """chip ID + HW-lock FSM (page 0x2e) + link snapshot."""
        cid = self.chip_id()
        s = self.h['read_hwlock']()
        m = fhs.measure_link(self.h, dur=3.0, label='status')
        print(f'  chip={cid:04X}  FSM: state={s["state_name"]} locked={s["locked"]} '
              f'failed={s["failed"]} bitslip=({s["p0"]},{s["p1"]})  '
              f'fs={m["fs"]:.1f}/s crc={m["crc_err_pct"]:.1f}% last_fe={m["last_frame_lines"]}')
        return dict(chip_id=cid, fsm=s, link=m)

    # ---- diagnostics (consolidated from the standalone one-off scripts) ------
    @staticmethod
    def _pg(p):
        """debug page -> read_dbg control form (0x80|(p&0x1F) for p>=0x20)."""
        return p if p < 0x20 else (0x80 | (p & 0x1F))

    _KEY_REGS = [
        (0x300A, 'chip ID hi'), (0x300B, 'chip ID lo'), (0x300E, 'MIPI ctrl00 (0x45=stream)'),
        (0x4800, 'MIPI ctrl (0x14=continuous/0x24=gated)'), (0x4300, 'format (0x6F=RGB565)'),
        (0x501F, 'ISP mux (0x01=RGB)'), (0x3036, 'PLL mult (0x36=54/0x30=48)'), (0x3035, 'PLL sysdiv'),
        (0x380C, 'HTS hi'), (0x380D, 'HTS lo'), (0x380E, 'VTS hi'), (0x380F, 'VTS lo'),
        (0x3500, 'exposure[19:16]'), (0x3501, 'exposure[15:8]'), (0x3502, 'exposure[7:0]'),
        (0x350A, 'gain hi'), (0x350B, 'gain lo'), (0x3A00, 'AEC ctrl (bit5=night)'),
        (0x3A18, 'gain ceiling hi'), (0x3A19, 'gain ceiling lo'),
    ]
    # full Linux-comparison dump set (from dump_all_chip_regs.py).
    _FULL_REGS = [
        0x3008, 0x300A, 0x300B, 0x300E, 0x3017, 0x3018, 0x3019, 0x302D, 0x302E,
        0x3034, 0x3035, 0x3036, 0x3037, 0x3108,
        0x3500, 0x3501, 0x3502, 0x3503, 0x350A, 0x350B,
        0x3A00, 0x3A02, 0x3A03, 0x3A08, 0x3A09, 0x3A0A, 0x3A0B, 0x3A0D, 0x3A0E,
        0x3A0F, 0x3A10, 0x3A11, 0x3A14, 0x3A15, 0x3A18, 0x3A19, 0x3A1B, 0x3A1F,
        0x3800, 0x3801, 0x3802, 0x3803, 0x3804, 0x3805, 0x3806, 0x3807,
        0x3808, 0x3809, 0x380A, 0x380B, 0x380C, 0x380D, 0x380E, 0x380F,
        0x3810, 0x3811, 0x3814, 0x3815, 0x3820, 0x3821,
        0x4001, 0x4004, 0x4202, 0x4300, 0x4800, 0x4814, 0x4837,
        0x5000, 0x5001, 0x501F, 0x503D, 0x3C00, 0x3C01,
    ]
    _PAGE_LABELS = {
        0x02: 'crc {ok,err}', 0x03: 'parser {short,long}', 0x04: '{pkt_trunc,ecc_uncorr}',
        0x05: '{last_frame_lines,pix}', 0x07: '{drop_dt,drop_vc}',
        0x18: '{fs,fe}', 0x19: '{ls,le}', 0x1a: 'last short {di,wc}',
        0x1b: '{live_lines,last_fe_lines}', 0x1c: '{fe_before480,fe_after480}',
        0x1d: '{fs_overlap,fe_without_fs}', 0x1e: '{other_short,long_before_fs}',
        0x20: '{sync_hdr_valid,stream_sop}', 0x21: '{header_valid,cdc_sop}',
        0x28: '{ecc_hdr_valid,pkt_hdr_valid}', 0x2a: 'supervisor',
        0x2b: '{sot_burst,burst}', 0x2c: 'missed_burst', 0x2d: 'relock {max,last}',
        0x2e: 'HW lock FSM', 0x33: 'raw {short,long}',
    }

    def regs(self, full=False):
        """Read chip registers (SCCB). Read these BEFORE any measure -- SCCB reads
        during an active lock glitch the link. full=True = the ~80-reg Linux dump."""
        vals = {}
        if full:
            for a in self._FULL_REGS:
                try: v = self.h['sccb_read'](a)
                except Exception: v = None
                vals[a] = v
                print(f'  0x{a:04X} = ' + (f'0x{v:02X}' if v is not None else 'ERR'))
        else:
            for a, name in self._KEY_REGS:
                try: v = self.h['sccb_read'](a)
                except Exception: v = None
                vals[a] = v
                print(f'  0x{a:04X} = ' + (f'0x{v:02X}' if v is not None else 'ERR') + f'  {name}')
            hts = ((vals.get(0x380C, 0) or 0) << 8) | (vals.get(0x380D, 0) or 0)
            vts = ((vals.get(0x380E, 0) or 0) << 8) | (vals.get(0x380F, 0) or 0)
            print(f'  -> HTS={hts} VTS={vts}  (mainline VGA mult48: 1600/1000 = ~30fps)')
        return vals

    def pages(self, lo=0x00, hi=0x40):
        """Scan debug pages lo..hi (raw 32-bit) + decode the labelled ones. Passive
        (read_dbg only; no chip touch)."""
        for p in range(lo, hi):
            w = self.h['read_dbg'](self._pg(p))
            lab = self._PAGE_LABELS.get(p, '')
            extra = f'   {{{(w>>16)&0xFFFF},{w&0xFFFF}}} {lab}' if lab else ''
            print(f'  page 0x{p:02x} = 0x{w:08x}{extra}')

    def _snap_acct(self):
        rd = self.h['read_dbg']
        pages = [0x02, 0x03, 0x04, 0x07, 0x18, 0x19, 0x1c, 0x1d, 0x1e, 0x20, 0x28, 0x2b, 0x33]
        return {p: rd(self._pg(p)) for p in pages}

    def accounting(self, dur=3.0):
        """Frame-sync drop localisation (PHY-raw vs parser vs frame_asm) + FE timing.
        Deltas over `dur`. Localises a line/frame shortfall to chip / PHY / FPGA side."""
        hi = lambda w: (w >> 16) & 0xFFFF
        lo = lambda w: w & 0xFFFF
        a = self._snap_acct(); time.sleep(dur); b = self._snap_acct()
        d = lambda p, f: (f(b[p]) - f(a[p])) % 65536
        r = lambda x: x / dur
        phy_long = d(0x33, lo);  parser_long = d(0x03, lo);  pkt_hdr = d(0x28, lo)
        fs = d(0x18, hi); fe = d(0x18, lo); ls = d(0x19, hi); le = d(0x19, lo)
        feb = d(0x1c, hi); fea = d(0x1c, lo)
        fsov = d(0x1d, hi); fenofs = d(0x1d, lo); longbfs = d(0x1e, lo)
        sotb = d(0x2b, hi); burst = d(0x2b, lo)
        dropdt = d(0x07, hi); dropvc = d(0x07, lo); trunc = d(0x04, hi); eccu = d(0x04, lo)
        print(f'  PHY raw long={r(phy_long):.0f}/s | parser pkt_hdr={r(pkt_hdr):.0f}/s '
              f'long={r(parser_long):.0f}/s | frame_asm fs={r(fs):.1f} fe={r(fe):.1f} '
              f'ls={r(ls):.0f} le={r(le):.0f}/s')
        print(f'  burst={r(burst):.0f}/s sot_burst={r(sotb):.0f}/s (burst-sot = per-line SoT miss)')
        print(f'  FE_before480={feb} FE_after480={fea}  fs_overlap={fsov} fe_without_fs={fenofs} '
              f'long_before_fs={longbfs}')
        print(f'  reject: drop_dt={dropdt} drop_vc={dropvc} pkt_trunc={trunc} ecc_uncorr={eccu}')
        # localisation heuristic (diag_fsfe_accounting)
        v = '  => '
        if r(phy_long) < 200 and r(ls) < 200:
            v += 'PHY/chip side: few raw longs reach the PHY (chip not streaming / SoT miss).'
        elif r(parser_long) < r(phy_long) * 0.5:
            v += 'PARSER/CDC side: raw longs arrive but parser drops them.'
        elif feb > fea * 2:
            v += 'frames end EARLY (short) -- spurious early FE / FS-anchor close.'
        elif fea > feb * 2:
            v += 'frames end LATE (tall/merged) -- FE dropped.'
        elif burst and (sotb < burst * 0.9):
            v += 'per-line SoT miss (burst-head settle garbage) -- raise settle-blank.'
        else:
            v += 'PHY ~ parser ~ frame_asm balanced -- clean.'
        print(v)
        link = fhs.measure_link(self.h, dur=2.0, label='acct')
        return dict(phy_long=r(phy_long), parser_long=r(parser_long), fs=r(fs), fe=r(fe),
                    ls=r(ls), le=r(le), fe_before480=feb, fe_after480=fea, link=link)

    def eye(self, taps=None, dur=0.4):
        """Per-lane IDELAY eye scan: hold the locked bitslip, inhibit the FSM, sweep
        the IDELAY tap, score each by crc=0 + long-rate, report the widest clean
        window + centre. FPGA-side / chip-safe. Restores tap 16 + the FSM at the end."""
        if taps is None: taps = list(range(0, 32))
        s = self.h['read_hwlock']()
        self.h['set_hw_lock'](False)
        self.h['bitslip_set'](s['p0'], s['p1'])          # hold the FSM-locked byte phase
        time.sleep(0.3)
        clean = []
        for t in taps:
            self.h['idelay_set'](t, t); time.sleep(0.05)
            m = fhs.measure_link(self.h, dur=dur, label=f'eye{t}')
            ok = (m['crc_err_pct'] < 1.0 and m['long_pkt'] > 1000)
            clean.append(t if ok else None)
            print(f'  tap {t:2d}: long={m["long_pkt"]:6.0f}/s crc={m["crc_err_pct"]:4.1f}%  {"CLEAN" if ok else ""}')
        # widest run of clean taps
        best_lo = best_hi = cur_lo = None
        for i, t in enumerate(taps):
            if clean[i] is not None:
                if cur_lo is None: cur_lo = t
                if best_lo is None or (t - cur_lo) > (best_hi - best_lo):
                    best_lo, best_hi = cur_lo, t
            else:
                cur_lo = None
        if best_lo is not None:
            centre = (best_lo + best_hi) // 2
            print(f'  widest clean window: taps {best_lo}..{best_hi} -> centre {centre}')
        else:
            centre = 16
            print('  no clean window found (placement / chip issue)')
        self.h['idelay_set'](16, 16)                     # restore eye-centre
        self.h['set_hw_lock'](True)                       # re-arm the FSM
        return centre

    def lockstats(self, dur=15.0, interval=2.0):
        """Sample fs/last_fe/long over `dur` to tell a REAL link death from the ~6s
        long-counter freeze (fs/last_fe stay live even when the long counter freezes)."""
        t0 = time.time(); fs_min = 99.0; lfe_min = 9999; froze = False
        while time.time() - t0 < dur:
            m = fhs.measure_link(self.h, dur=1.5, label=f'lock+{time.time()-t0:.0f}s')
            fs_min = min(fs_min, m['fs']); lfe_min = min(lfe_min, m['last_frame_lines'])
            if m['long_pkt'] < 100: froze = True
            time.sleep(interval)
        print('  => ' + (
            'HEALTHY (long froze but fs/last_fe stayed live = counter artifact)'
            if (froze and fs_min > 5 and lfe_min >= 470) else
            'REAL DEATH (fs/last_fe dropped)' if fs_min <= 5 else 'stable'))
        return dict(fs_min=fs_min, last_fe_min=lfe_min, long_froze=froze)

    def diagnostics(self):
        """Comprehensive one-shot health report: chip regs + FSM lock + link +
        frame-sync accounting + verdict. Reads chip regs FIRST (pre-glitch)."""
        print('\n=== DIAGNOSTICS ===')
        print('--- chip registers (pre-measure) ---')
        rv = self.regs(full=False)
        print('--- HW-lock FSM (page 0x2e) ---')
        s = self.h['read_hwlock']()
        print(f'  state={s["state_name"]} locked={s["locked"]} failed={s["failed"]} '
              f'bitslip=({s["p0"]},{s["p1"]}) reroll={s["reroll"]} hdr_active={s["hdr_active"]}')
        print('--- link + frame-sync accounting ---')
        acct = self.accounting(dur=3.0)
        m = acct['link']
        clean = (s['locked'] and not s['failed'] and m['crc_err_pct'] < 1.0
                 and m['fs'] > 5 and m['last_frame_lines'] >= 470)
        print('--- VERDICT ---')
        print('  => CLEAN: FSM locked, crc=0%, full-height, frames flowing.' if clean
              else '  => ISSUES -- inspect FSM / crc / last_fe / accounting above.')
        return dict(regs=rv, fsm=s, acct=acct, clean=clean)

    # ---- knobs --------------------------------------------------------------
    def idelay(self, t0, t1=None): self.h['idelay_set'](t0, t1); print(f'  idelay={t0},{t1 if t1 is not None else t0}')
    def bitslip(self, p0, p1): self.h['set_hw_lock'](False); self.h['bitslip_set'](p0, p1); print(f'  bitslip=({p0},{p1}) (FSM inhibited)')
    def settle(self, k): self.h['set_settle_blank'](int(k)); print(f'  settle-blank K={k}')
    def reroll(self): self.h['bufr_clr_pulse'](); print('  BUFR.CLR re-rolled /4 phase')

    def vcm(self, code):
        """OV5640 VCM focus DAC (10-bit). 0x3603[5:0]=D[9:4], 0x3602[7:4]=D[3:0]."""
        c = int(code) & 0x3FF
        self.h['sccb_write'](0x3603, (c >> 4) & 0x3F)
        self.h['sccb_write'](0x3602, ((c & 0xF) << 4) | 0x00)
        print(f'  VCM focus code = {c}')

    _WIN_BASE = (257, 4)      # init ISP offsets: X={0x3810,0x3811}=257 (centred), Y=4

    def window(self, dx=0, dy=0):
        """Digitally PAN the 640x480 output window to re-centre it on the lens optical
        axis (parallel shift), runtime SCCB. The full sensor array is read out, so the
        OV5640 ISP windowing offset selects which region maps to the output -- panning
        pulls in real pixels from the margin (no black border) within the ISP range.
          X offset = base(257) + dx -> 0x3810/0x3811  (larger dx shifts content one way)
          Y offset = base(4) + dy   -> 0x3812/0x3813
        cam.window(0,0) restores the init centre. Sweep live on HDMI in small steps
        (~+/-8..64) to find the value that puts the lens optical centre at screen centre;
        a too-large value runs past the ISP margin and corrupts/clips the frame.
        Ref: docs/doc/ov5640_linux_mainline_reference.md timing/window block
        0x3800-0x3813; this is a per-module optical-centre calibration (init untouched)."""
        xb, yb = self._WIN_BASE
        xo = max(0, min(0xFFF, xb + int(dx)))
        yo = max(0, min(0xFFF, yb + int(dy)))
        self.h['sccb_write'](0x3810, (xo >> 8) & 0x0F)
        self.h['sccb_write'](0x3811, xo & 0xFF)
        self.h['sccb_write'](0x3812, (yo >> 8) & 0x0F)
        self.h['sccb_write'](0x3813, yo & 0xFF)
        print(f'  ISP window offset X={xo} Y={yo} (dx={dx},dy={dy})  '
              f'[cam.window(0,0) = re-centre to init]')

    def sharpen(self, level=0x20):
        """Chip CIP edge sharpening (OV5640 CIP block 0x5300-0x530C), SOURCE-side,
        runtime SCCB. Sharpens the RGB565 BEFORE the MIPI output -- complementary to
        cam.k('sharpen')/cam.k('sharpen_hi') which sharpen in the FPGA conv slot.
          level 0      -> AUTO (restore POR 0x5308=0x25; chip's own auto sharpen)
          level 1..0x3F -> MANUAL strength: enable manual-MT (0x5308 bit6 -> 0x65) and
                           set SHARPENMT/TH offsets 0x5302/0x530B (higher = crisper,
                           too high = ringing/halos + noise gain).
        Ref: docs/doc/ov5640_linux_mainline_reference.md CIP table (matches mainline);
        boot init leaves 0x5308 at POR auto -- this is an opt-in runtime override."""
        lv = int(level) & 0x3F
        if lv == 0:
            self.h['sccb_write'](0x5308, 0x25)             # POR auto sharpen/denoise
            print('  CIP sharpen = AUTO (0x5308=0x25)')
        else:
            self.h['sccb_write'](0x5308, 0x65)             # bit6=1 manual edge-MT enable
            self.h['sccb_write'](0x5302, lv)               # SHARPENMT offset1 (smooth area)
            self.h['sccb_write'](0x530B, lv)               # SHARPENTH offset1 (edges)
            print(f'  CIP sharpen = MANUAL 0x{lv:02X} (0x5302/0x530B; 0=auto)')

    def gain_ceiling(self, ceil):
        """Cap AGC max gain (0x3A18/0x3A19, /16: 0x80=8x). Cuts low-light column FPN."""
        self.h['sccb_write'](0x3A18, (int(ceil) >> 8) & 0x03)
        self.h['sccb_write'](0x3A19, int(ceil) & 0xFF)
        print(f'  AGC gain ceiling = 0x{int(ceil):04X} (~{int(ceil)/16:.1f}x)')

    def testpattern(self, val=0x84):
        """OV5640 0x503D test pattern (0x84 vgrad, 0x80 color bar; 0 = sensor)."""
        fhs.stream_cycle_write(self.h, [(0x503D, int(val))] + list(fhs.ARM_REGS))
        print(f'  0x503D = 0x{int(val):02X}')

    # ---- VDMA output --------------------------------------------------------
    def _alloc(self):
        self._bufs = [allocate(shape=(HEIGHT, STRIDE), dtype=np.uint8) for _ in range(3)]
        for b in self._bufs:
            np.asarray(b).fill(0xAA)
        d = self.ol.ip_dict['axi_vdma_0']
        self._vdma = MMIO(int(d['phys_addr']), int(d['addr_range']))

    def hdmi(self, secs=60.0):
        """Live HDMI for `secs` (S2MM camera->DDR + MM2S DDR->HDMI, 1 frame delay).
        Blocking; auto-stops the VDMA on return / exit. Do NOT kill mid-run."""
        self._alloc()
        configure_vdma_s2mm(self._vdma, self._bufs, start_mm2s=True, start_s2mm=True)
        print(f'  HDMI live {secs:.0f}s -- watch the monitor (auto-stops; do NOT Ctrl-C kill)')
        try:
            t0 = time.time()
            while time.time() - t0 < secs:
                time.sleep(min(15.0, secs))
                fhs.measure_link(self.h, dur=2.0, label=f'hdmi+{time.time()-t0:.0f}s')
        finally:
            self.stop()

    def hdmi_on(self):
        """Start live HDMI WITHOUT blocking (S2MM camera->DDR + MM2S DDR->HDMI), so you
        can switch cam.proc(op) interactively. Call cam.stop() when done (atexit too)."""
        self._alloc()
        configure_vdma_s2mm(self._vdma, self._bufs, start_mm2s=True, start_s2mm=True)
        print('  HDMI live (non-blocking). Try: cam.proc(1)..cam.proc(7) ; cam.stop()')

    def proc(self, op):
        """Phase 2 processing-slot select, live (no rebuild).
        Point ops 0-7: 0=passthrough 1=invert 2=grayscale 3=BGR-swap 4=threshold
        5=R-only 6=G-only 7=B-only.
        op>=8: 3x3 CONV mode -- applies the CURRENTLY-LOADED kernel (set it with
        cam.k(name) or cam.kernel(coeffs,shift); reset default = identity).
        NB only bit3 selects conv, so ops 8..15 are all 'conv with the loaded
        kernel' -- they are NOT distinct fixed kernels. cam.proc(0) exits to colour."""
        base = int(op) & 0xF
        if base == 0:
            # true colour reset: also clear the pre/post point-op slots (0xFE46/48), else a
            # leftover post-op (e.g. from edge_binary) keeps filtering after "back to colour".
            self.h['set_pre_op'](0); self.h['set_post_op'](0)
        if base < 8:
            names = {0: 'passthrough', 1: 'invert', 2: 'grayscale', 3: 'BGR-swap',
                     4: 'threshold', 5: 'R-only', 6: 'G-only', 7: 'B-only'}
            label = names[base]
        elif base == 12:
            label = 'DoG dual-kernel (3x3 vs 5x5 -- set via cam.dog)'
        elif base in (13, 14, 15):
            label = f'cascade blur {("5x5","9x9","13x13")[base-13]} (set via cam.blur)'
        else:
            label = '3x3 conv (loaded kernel -- set via cam.k/cam.kernel)'
        self.h['set_proc_op'](base)
        print(f'  proc_op={base} ({label})')

    def passthrough(self):
        """Back to plain HDMI colour passthrough: clears BOTH point-op slots (pre 0xFE46 +
        post 0xFE48), resets thresholds to 128, and selects proc_op 0. This is the reliable
        'undo any filter/combination' reset -- e.g. after edge_binary/bin_edges/pipeline.
        Leaves the VDMA/HDMI running so colour appears live."""
        self.h['set_pre_op'](0); self.h['set_post_op'](0)
        self.h['set_pre_thresh'](128); self.h['set_post_thresh'](128)
        self.h['set_dither'](enable=False)        # also clear the final dither stage
        self.h['set_proc_op'](0)
        print('  HDMI passthrough (colour) -- pre/post point-ops + dither cleared, proc_op=0')
        return self

    def kernel(self, coeffs, shift=0):
        """Load a RUNTIME-PROGRAMMABLE 3x3 conv kernel live (no rebuild) and switch to
        conv mode. coeffs = 9 signed ints row-major (idx 0=top-left..8=bottom-right),
        shift = right-shift normalisation. Presets: cam.k_gauss()/k_sobel()/k_sharpen()/
        k_emboss(). Examples:
          cam.kernel([1,2,1,2,4,2,1,2,1], 4)   # Gaussian blur
          cam.kernel([-1,0,1,-2,0,2,-1,0,1], 0) # Sobel-X edge
        cam.proc(0) returns to the point path (passthrough)."""
        self.h['set_conv_kernel'](coeffs, shift)
        self.h['set_proc_op'](8)                       # conv mode
        print(f'  kernel {list(coeffs)[:9]} >>{shift} loaded; conv mode ON '
              f'(cam.proc(0) to exit)')

    def k(self, name):
        """Load a NAMED 3x3 kernel and switch to conv mode. Names: cam.kernels().
        e.g. cam.k('sobel_x'), cam.k('emboss'), cam.k('gaussian')."""
        self.h['set_conv_named'](name)
        self.h['set_proc_op'](8)
        print(f"  kernel '{name}' = {CONV_KERNELS[name][0]} >>{CONV_KERNELS[name][1]}; "
              f"conv mode ON (cam.proc(0) to exit)")

    def kernels(self):
        """List the named 3x3 kernels available to cam.k(name)."""
        for nm, (c, s) in CONV_KERNELS.items():
            print(f"  {nm:10s} {c} >>{s}")
        return list(CONV_KERNELS)

    def k_gauss(self):   self.k('gaussian')    # blur
    def k_sobel(self):   self.k('sobel_x')     # edge (vertical)
    def k_sharpen(self): self.k('sharpen')
    def k_emboss(self):  self.k('emboss')

    def dog(self, name='blob'):
        """Difference-of-Gaussians DUAL-kernel (op 12, requires the DoG bitstream): a 3x3
        (A) and a general 5x5 (B) run in PARALLEL on the same pixels and combine as
        clamp(alpha*A - beta*B + offset). Named presets: cam.dogs(). e.g. cam.dog('blob')
        (band-pass/edge), cam.dog('unsharp'). cam.proc(0) returns to colour. For a custom
        kernel use h['set_dog'](small9, sshift, large25, lshift, alpha, beta, shift, off, mode)."""
        self.h['set_dog_named'](name)
        s = DOG_PRESETS[name]
        print(f"  DoG '{name}': A=3x3>>{s[1]}  B=5x5>>{s[3]}  a={s[4]} b={s[5]} "
              f">>{s[6]} +{s[7]} mode={s[8]}; op 12 (cam.proc(0) to exit)")

    def dogs(self):
        """List the named DoG presets available to cam.dog(name)."""
        for nm in DOG_PRESETS:
            print(f"  {nm}")
        return list(DOG_PRESETS)

    def blur(self, size=13):
        """Runtime-VARIABLE Gaussian blur via the 3-stage cascade (op 13/14/15). size in
        {5,9,13} = effective kernel 5x5 / 9x9 / 13x13 -- more cascade stages = wider blur,
        switched live (no rebuild). Requires the cascade bitstream. cam.proc(0) exits."""
        self.h['set_blur'](size)
        op = 13 if size < 9 else (14 if size < 13 else 15)
        print(f"  cascade blur {size}x{size} (op {op}); cam.proc(0) to exit")

    def edges(self, shift=2):
        """Omnidirectional Sobel edge MAGNITUDE |Gx|+|Gy| live (op 12): all-direction edges,
        bright on black. Uses the conv |.| (cfg_abs) so BOTH gradient polarities show -- a
        single cam.k('sobel_x') only shows one side. Needs the edge-magnitude bitstream.
        `shift` scales the gradient. cam.proc(0) exits."""
        self.h['set_edges'](shift)
        print(f"  Sobel edge magnitude |Gx|+|Gy| (shift {shift}); cam.proc(0) to exit")

    def pre_op(self, op=0):
        """PRE-stage select, applied BEFORE the conv stage (active in conv mode, cam.proc>=8).
        Accepts a code or name: 0=off 1=invert 2=gray 3=BGR 4=threshold 5/6/7=R/G/B,
        8/'gaussian' = 3x3 blur, 9/'median' = 3x3 median (denoise). e.g. cam.pre_op('median')
        then cam.edges() = denoise -> Sobel. NB 8/9 only show in conv mode -- use cam.denoise()
        to apply + enter conv-passthrough in one step. cam.pre_op(0) restores."""
        code = _pre_code(op)
        self.h['set_pre_op'](code); print(f"  pre_op={code} ({op})")

    def denoise(self, kind='median'):
        """Apply a 3x3 PRE-stage spatial DENOISE and make it visible now (enters conv-
        passthrough): kind='median' (impulse/salt-pepper removal) or 'gaussian' (blur).
        Compose with an edge filter next, e.g. cam.denoise('median'); cam.edges() =
        denoise -> Sobel. cam.passthrough() resets."""
        code = _pre_code(kind)
        self.h['set_pre_op'](code)
        self.h['set_post_op'](0)
        self.h['set_conv_named']('identity')   # force conv passthrough so the PRE denoise IS the output
        self.h['set_proc_op'](8)               # conv mode (identity) -> output = denoised pixels
        print(f"  PRE denoise = {kind} (code {code}); conv-passthrough. Add cam.edges()/cam.k(); cam.passthrough() resets")

    def pre_thresh(self, level=128):
        """Threshold level (on green, 0..255) for a PRE-conv binarize (op 4)."""
        self.h['set_pre_thresh'](level); print(f"  pre_thresh={level}")

    def post_op(self, op=0):
        """Point op run AFTER the conv/mux stage. Same codes as pre_op. e.g. cam.edges()
        then cam.post_op(4) = Sobel -> binarize. cam.post_op(0) restores."""
        self.h['set_post_op'](op); print(f"  post_op={op} (point op after conv)")

    def post_thresh(self, level=128):
        """Threshold level (0..255) for a POST-conv binarize (op 4). Edge maps want ~64."""
        self.h['set_post_thresh'](level); print(f"  post_thresh={level}")

    def dither(self, bits=1, mode='ordered'):
        """Final DITHER stage AFTER post: quantize each channel to `bits` with ordered (Bayer)
        or random (LFSR) dithering. bits 1=halftone(0/255) .. 6=anti-banding; cam.dither(0)=off.
        Composes with POST, e.g. cam.proc(2); cam.dither(1) = gray -> halftone."""
        if not bits:
            self.h['set_dither'](enable=False); print('  dither off'); return self
        self.h['set_dither'](enable=True, mode=mode, bits=int(bits))
        print(f"  dither: {mode} {int(bits)}bit/ch (after POST); cam.dither(0) to disable")
        return self

    def halftone(self, mode='ordered'):
        """Grayscale -> 1-bit dither = newspaper halftone. cam.passthrough() resets."""
        self.passthrough(); self.proc(2); self.dither(1, mode); return self

    def poster(self, bits=2, mode='ordered'):
        """Colour posterize: dither each channel to `bits` (retro look). cam.dither(0) off."""
        self.passthrough(); self.dither(bits, mode); return self

    def bin_edges(self, level=128, shift=2):
        """BINARIZE then SOBEL: threshold (on green > level) -> omnidirectional edge
        magnitude = clean contours of the binary regions. cam.proc(0)+cam.pre_op(0) exit."""
        self.h['set_post_op'](0)                       # clear the post chain (order-independent)
        self.h['set_pre_thresh'](level)
        self.h['set_pre_op'](4)
        self.h['set_edges'](shift)
        print(f"  binarize(g>{level}) -> Sobel |Gx|+|Gy| (shift {shift}); cam.passthrough() to reset")

    def edge_binary(self, level=64, shift=2):
        """SOBEL then BINARIZE: omnidirectional edge magnitude -> threshold = a BINARY edge
        map (black/white, ~Canny stage 1). Lower level = more edges. cam.proc(0)+cam.post_op(0) exit."""
        self.h['set_pre_op'](0)                        # clear the pre chain (order-independent)
        self.h['set_edges'](shift)
        self.h['set_post_thresh'](level)
        self.h['set_post_op'](4)
        print(f"  Sobel |Gx|+|Gy| -> binarize(>{level}) edge map (shift {shift}); cam.passthrough() to reset")

    def _snap(self, tag, outdir=None, prefix='repl_edge_'):
        """Copy the live S2MM buffer (does NOT stop the VDMA) and save a still. Used by
        edge_demo() per stage while HDMI stays live. outdir=None -> _capture/."""
        buf = np.array(self._bufs[1]).copy()
        bpp = STRIDE // WIDTH
        if bpp >= 3:
            px = buf.reshape(HEIGHT, WIDTH, bpp)
            frame = np.stack([px[:, :, 2], px[:, :, 1], px[:, :, 0]], axis=-1).astype(np.uint8)
            mode = 'RGB'
        else:
            frame = buf[:, :WIDTH]; mode = 'L'
        if outdir is None:
            jup = Path('/home/xilinx/jupyter_notebooks')
            outdir = (jup if jup.is_dir() else HERE) / '_capture'
        outdir = Path(outdir)
        outdir.mkdir(parents=True, exist_ok=True)
        base = (outdir / f'{prefix}{tag}_{time.strftime("%H%M%S")}').resolve()
        np.save(base.with_suffix('.npy'), frame)
        try:
            from PIL import Image
            Image.fromarray(frame, mode).save(base.with_suffix('.png'))
        except Exception as e:
            print(f'   (png skipped: {e})')
        y = frame.mean(axis=2) if frame.ndim == 3 else frame
        print(f'   [{tag}] mean={y.mean():.1f} std={y.std():.1f} '
              f'edge%(>60)={float((y > 60).mean() * 100):.1f} -> {base}.png')

    def edge_demo(self, dwell=6.0, testpattern=False):
        """LIVE edge-processing cycle on HDMI (= edge_demo.py, from the REPL): colour ->
        Sobel-X -> omnidirectional edges -> binarize->Sobel -> Sobel->binarize -> colour,
        capturing a still per stage to _capture/repl_edge_<tag>.png. testpattern=True injects
        the OV5640 colour bar (known sharp edges, lens-independent). Run cam.go() first.
        Auto-stops the VDMA on return/exit; do NOT kill it mid-run."""
        if testpattern:
            self.testpattern(0x80)
        def clear():
            self.h['set_pre_op'](0); self.h['set_post_op'](0)
        stages = [
            ('colour',     lambda: (clear(), self.h['set_proc_op'](0))),
            ('sobelx',     lambda: (clear(), self.h['set_conv_named']('sobel_x'), self.h['set_proc_op'](8))),
            ('edges',      lambda: (clear(), self.h['set_edges'](2))),
            ('binedges',   lambda: self.bin_edges(128, 2)),       # clears post chain itself
            ('edgebinary', lambda: self.edge_binary(64, 2)),      # clears pre chain itself
            ('colour',     lambda: (clear(), self.h['set_proc_op'](0))),
        ]
        self._alloc()
        configure_vdma_s2mm(self._vdma, self._bufs, start_mm2s=True, start_s2mm=True)
        print('  === LIVE edge demo on HDMI -- cycling stages, capturing each ===')
        try:
            for tag, apply in stages:
                apply()
                print(f'  stage {tag} for {dwell:.0f}s')
                time.sleep(dwell * 0.6)
                self._snap(tag)
                time.sleep(dwell * 0.4)
            clear(); self.h['set_proc_op'](0)
        finally:
            self.stop()
        print('  edge demo done (VDMA stopped).')

    def dither_demo(self, dwell=6.0, testpattern=False):
        """LIVE dither cycle on HDMI: colour -> halftone (gray 1-bit) -> poster 2-bit -> poster
        4-bit -> random 2-bit -> colour, capturing a still per stage to _capture/repl_dith_<tag>.png.
        testpattern=True injects the OV5640 colour bar (lens-independent). Run cam.go() first.
        Auto-stops the VDMA on return/exit; do NOT kill it mid-run."""
        if testpattern:
            self.testpattern(0x80)
        h = self.h
        def reset():
            h['set_pre_op'](0); h['set_post_op'](0); h['set_dither'](enable=False); h['set_proc_op'](0)
        stages = [
            ('colour',   lambda: reset()),
            ('halftone', lambda: (reset(), h['set_post_op'](2), h['set_dither'](enable=True, mode='ordered', bits=1))),
            ('poster2',  lambda: (reset(), h['set_dither'](enable=True, mode='ordered', bits=2))),
            ('poster4',  lambda: (reset(), h['set_dither'](enable=True, mode='ordered', bits=4))),
            ('random2',  lambda: (reset(), h['set_dither'](enable=True, mode='random', bits=2))),
            ('colour',   lambda: reset()),
        ]
        self._alloc()
        configure_vdma_s2mm(self._vdma, self._bufs, start_mm2s=True, start_s2mm=True)
        print('  === LIVE dither demo on HDMI -- cycling modes, capturing each ===')
        try:
            for tag, apply in stages:
                apply()
                print(f'  stage {tag} for {dwell:.0f}s')
                time.sleep(dwell * 0.6)
                self._snap(tag, prefix='repl_dith_')
                time.sleep(dwell * 0.4)
            reset()
        finally:
            self.stop()
        print('  dither demo done (VDMA stopped).')

    # ---- configurable filter combinations ----------------------------------
    def _apply_mid(self, mid):
        """Configure the middle (spatial) filter of a chain and select its conv mode.
        Returns a short label. mid: 'edges' | CONV_KERNELS name | ('kernel',name|coeffs[,shift])
        | ('dog',preset) | ('blur',size) | ('edges',shift)."""
        if isinstance(mid, str):
            m = mid.lower()
            if m == 'edges':
                self.h['set_edges'](2); return 'edges'
            if mid in CONV_KERNELS:
                self.h['set_conv_named'](mid); self.h['set_proc_op'](8); return f'kernel:{mid}'
            raise ValueError(f"unknown mid filter {mid!r}")
        kind = str(mid[0]).lower()
        if kind == 'edges':
            self.h['set_edges'](mid[1] if len(mid) > 1 else 2); return 'edges'
        if kind == 'kernel':
            spec = mid[1]
            if isinstance(spec, str):
                self.h['set_conv_named'](spec); label = f'kernel:{spec}'
            else:
                self.h['set_conv_kernel'](spec, mid[2] if len(mid) > 2 else 0); label = 'kernel:custom'
            self.h['set_proc_op'](8); return label
        if kind == 'dog':
            self.h['set_dog_named'](mid[1]); return f'dog:{mid[1]}'     # sets proc_op 12
        if kind == 'blur':
            self.h['set_blur'](mid[1]); return f'blur:{mid[1]}'         # sets proc_op 13/14/15
        raise ValueError(f"unknown mid filter {mid!r}")

    def chain(self, pre=0, mid=None, post=0, pre_thresh=128, post_thresh=64, dither=None):
        """Configure a full live FILTER COMBINATION in one call (no rebuild):
            pre point-op  ->  mid spatial filter  ->  post point-op  ->  dither.
        pre: 0-7 / name, OR 'gaussian'/'median' (3x3 spatial denoise). post: 0-7 point-op / name.
        mid: None (point-only) | 'edges' | a CONV_KERNELS name | ('kernel',name|coeffs[,shift])
             | ('dog',preset) | ('blur',size).
        dither: None/0 = off; int = bits/channel (1=halftone .. 6=anti-banding, ordered);
                or (bits, mode) with mode 'ordered'/'random'. Applied AFTER post (final stage).
        pre_thresh/post_thresh: threshold level when that op is 'thresh' (4).
        NB pre-op only takes effect in conv mode; a point-only chain with a denoise pre
        (gaussian/median) enters conv-passthrough automatically. Start cam.hdmi_on() (or use
        the menu) to see it live; cam.passthrough() resets."""
        pc, qc = _pre_code(pre), _point_code(post)
        self.h['set_pre_thresh'](pre_thresh); self.h['set_post_thresh'](post_thresh)
        self.h['set_post_op'](qc)
        if mid is None or (isinstance(mid, str) and mid.lower() in ('none', 'off', '')):
            if pc >= 8:                        # denoise pre needs conv mode (identity) to be visible
                self.h['set_pre_op'](pc); self.h['set_conv_named']('identity'); self.h['set_proc_op'](8)
            else:                              # point-only: pre op goes through proc_op directly
                self.h['set_pre_op'](0); self.h['set_proc_op'](pc)
            label = 'none'
        else:
            self.h['set_pre_op'](pc)           # conv mode: pre slot applies pre_op before the filter
            label = self._apply_mid(mid)
        # final DITHER stage (after POST). Always set it so a chain also clears any stale dither.
        if dither in (None, 0, False):
            self.h['set_dither'](enable=False); dlabel = 'off'
        else:
            if isinstance(dither, (tuple, list)):
                dbits = int(dither[0]); dmode = dither[1] if len(dither) > 1 else 'ordered'
            else:
                dbits, dmode = int(dither), 'ordered'
            self.h['set_dither'](enable=True, mode=dmode, bits=dbits); dlabel = f'{dmode} {dbits}b'
        print(f"  chain: pre={pc} -> {label} -> post={qc} -> dither={dlabel} "
              f"(pre_thr={pre_thresh}, post_thr={post_thresh})")
        return self

    def pipeline(self, name='edges'):
        """Apply a NAMED filter combination (cam.pipelines() lists them), live/no rebuild.
        e.g. cam.pipeline('bin_edges'|'edge_binary'|'sketch'|'gray_edges'|'sharpen'|'dog_blob')."""
        if name not in PIPELINES:
            raise ValueError(f"unknown pipeline {name!r}; choices: {sorted(PIPELINES)}")
        self.chain(**PIPELINES[name])
        print(f"  pipeline '{name}' applied (start cam.hdmi_on() / cam.capture() to view)")
        return self

    def pipelines(self):
        """List the named filter combinations available to cam.pipeline(name)."""
        for nm in PIPELINES:
            print(f"  {nm:12s} {PIPELINES[nm]}")
        return list(PIPELINES)

    @staticmethod
    def k3to5(k9):
        """Embed a 3x3 kernel (9 row-major) in the centre of a 5x5 (25 coeffs) for stage S1."""
        out = [0]*25
        for r in range(3):
            for c in range(3):
                out[(r+1)*5 + (c+1)] = k9[r*3 + c]
        return out

    def cascade(self, s1=None, s2=None, s3=None, out=15):
        """Cascade DIFFERENT kernels per stage, live (no rebuild). Each stage has its own
        coefficient registers. Stages compose (series convolution); output = tap `out`
        (13=after S1 / 14=after S1.S2 / 15=after S1.S2.S3).
          s1 = (coeffs25, shift)   GENERAL 5x5 (any kernel; use cam.k3to5(k9) to embed a 3x3,
                                   e.g. s1=(cam.k3to5(CONV_KERNELS['laplacian'][0]), 0))
          s2,s3 = (h5, v5, hshift, vshift)  SEPARABLE 5x5 (Gaussian/box/separable only)
          None = leave the stage at identity (passthrough).
        Note: non-separable ops (edge/emboss) only fit S1 (the general stage). e.g. edge then
        blur: cam.cascade(s1=(cam.k3to5([0,-1,0,-1,4,-1,0,-1,0]),0), s2=([1,4,6,4,1],[1,4,6,4,1],4,4), out=14)"""
        if s1 is not None: self.h['set_conv5_kernel'](s1[0], s1[1])
        if s2 is not None: self.h['set_sep_kernel'](2, s2[0], s2[1], s2[2], s2[3])
        if s3 is not None: self.h['set_sep_kernel'](3, s3[0], s3[1], s3[2], s3[3])
        self.h['set_proc_op'](int(out))
        print(f"  cascade S1={'set' if s1 else 'id'} S2={'set' if s2 else 'id'} "
              f"S3={'set' if s3 else 'id'} -> tap op{out}; cam.proc(0) to exit")

    def capture(self, name=None, settle_s=1.5):
        """Grab one still frame (S2MM only) -> <script-dir>/_capture/repl_<ts>.{npy,png}
        on the BOARD. The REPL is interactive (not auto-pulled): the absolute path + an
        scp command are printed so you can copy it to the PC."""
        self._alloc()
        configure_vdma_s2mm(self._vdma, self._bufs, start_mm2s=False, start_s2mm=True)
        time.sleep(settle_s)                          # let a few frames land
        buf = np.array(self._bufs[1]).copy()                # (HEIGHT, STRIDE), settled
        self.stop()
        bpp = STRIDE // WIDTH                                # 4 = RGBA32 colour, 1 = Y8
        if bpp >= 3:                                         # colour: 32b {0,R,G,B} -> LE [B,G,R,0]
            px = buf.reshape(HEIGHT, WIDTH, bpp)
            frame = np.stack([px[:, :, 2], px[:, :, 1], px[:, :, 0]], axis=-1).astype(np.uint8)
            pil_mode = 'RGB'
        else:                                               # Y8 grayscale
            frame = buf[:, :WIDTH]
            pil_mode = 'L'
        # prefer the Jupyter notebook root so captures show up in the file browser;
        # fall back to <script-dir>/_capture off the board.
        jup = Path('/home/xilinx/jupyter_notebooks')
        outdir = (jup if jup.is_dir() else HERE) / '_capture'
        outdir.mkdir(parents=True, exist_ok=True)
        ts = name or time.strftime('repl_%Y%m%d_%H%M%S')
        npy = (outdir / f'{ts}.npy').resolve()
        png = (outdir / f'{ts}.png').resolve()
        np.save(npy, frame)
        try:
            from PIL import Image
            Image.fromarray(frame, pil_mode).save(png)
        except Exception as e:
            print(f'  (png skipped: {e})')
        print(f'  captured {frame.shape} (mean={frame.mean():.1f}) -> {png}')
        print(f'  pull to PC:  scp xilinx@<board-ip>:{png} .')
        return frame

    def stop(self):
        """Stop the VDMA (S2MM+MM2S) and free buffers. Idempotent; safety net."""
        if self._vdma is not None:
            try:
                stop_vdma(self._vdma)
            except Exception:
                pass
            self._vdma = None
        if self._bufs is not None:
            for b in self._bufs:
                if hasattr(b, 'freebuffer'):
                    try: b.freebuffer()
                    except Exception: pass
            self._bufs = None

    def help(self):
        print(__doc__)


# ============================ menu driver ================================
# A menu-first front end over the `cam` facade. The same `cam.*` methods stay
# available at the >>> prompt (quit the menu to drop into them). Free-form values
# (durations, register addr/val, kernel coeffs) are prompted interactively with a
# default shown in [brackets]: press Enter to accept it. Integers accept 0x.. hex.

def _input(prompt):
    try:
        return input(prompt)
    except EOFError:                       # Ctrl-D in a prompt = treat as 'back'
        return 'q'


def _ask_int(prompt, default=None):
    """Read an int (decimal or 0x.. hex). Empty input -> `default`."""
    raw = _input(prompt).strip()
    if raw == '':
        return default
    try:
        return int(raw, 0)
    except ValueError:
        print(f'  (not a number -- keeping {default})')
        return default


def _ask_float(prompt, default=None):
    raw = _input(prompt).strip()
    if raw == '':
        return default
    try:
        return float(raw)
    except ValueError:
        print(f'  (not a number -- keeping {default})')
        return default


def _ask_yn(prompt, default=False):
    d = 'Y/n' if default else 'y/N'
    raw = _input(f'{prompt} [{d}]: ').strip().lower()
    if raw == '':
        return default
    return raw.startswith('y')


def _pick(title, options, default=None):
    """options = list of (label, value). Return the chosen value, or `default`
    on empty/back. Numbered 1..N; 0 / blank / b = back."""
    print(f'  {title}')
    for i, (label, _) in enumerate(options, 1):
        print(f'    {i:2d}) {label}')
    raw = _input('  choose [back]> ').strip().lower()
    if raw in ('', '0', 'q', 'b', 'back'):
        return default
    try:
        idx = int(raw)
        if 1 <= idx <= len(options):
            return options[idx - 1][1]
    except ValueError:
        pass
    print('  (invalid choice)')
    return default


class Menu:
    """Number-driven menu over a `Cam` instance. `Menu(cam).run()`."""

    def __init__(self, cam):
        self.cam = cam

    # ---- generic submenu loop ----------------------------------------------
    def _submenu(self, title, items):
        """items = list of (label, callable). Loop until 'back'."""
        while True:
            print(f'\n  --- {title} ---')
            for i, (label, _) in enumerate(items, 1):
                print(f'    {i:2d}) {label}')
            print('     0) back')
            sel = _input(f'  {title.lower()}> ').strip().lower()
            if sel in ('', '0', 'q', 'b', 'back'):
                return
            try:
                idx = int(sel)
            except ValueError:
                print('  (invalid)'); continue
            if not (1 <= idx <= len(items)):
                print('  (invalid)'); continue
            try:
                items[idx - 1][1]()
            except KeyboardInterrupt:
                print('\n  (cancelled -- back to menu)')
            except Exception as e:
                print(f'  !! error: {e}')

    # ---- bring-up ----------------------------------------------------------
    def _go(self):
        print('  Full bring-up: init -> arm -> lock -> status.')
        hw = _ask_yn('  use the baked HW-lock FSM? (default; auto HOLD-lock. N = software lock)', True)
        sb = _ask_int('  settle-blank K (14=30fps/96MHz, 8=17fps/84MHz) [14]: ', 14)
        v = _ask_int('  init 0x4800 (0x14=continuous fs30 [default], 0x24=no-LS) [0x14]: ', 0x14)
        self.cam.go(val4800=v, hw=hw, settle_blank=sb)

    def bringup(self):
        self._submenu('Bring-up / status', [
            ('Full bring-up  (init + arm + lock)', self._go),
            ('Status         (chip + FSM + link)', self.cam.status),
            ('Diagnostics    (full health report)', self.cam.diagnostics),
            ('Init only      (chip init + frame config)', self._init),
            ('Arm RGB565     (stream cycle + settle-blank)', self._arm),
            ('Lock (software 8x8 sweep)', self._lock),
            ('HW-lock FSM on/off', self._hwlock),
        ])

    def _init(self):
        v = _ask_int('  init 0x4800 [0x14]: ', 0x14)
        self.cam.init(val4800=v)

    def _arm(self):
        sb = _ask_int('  settle-blank K [14]: ', 14)
        self.cam.arm(settle_blank=sb)

    def _lock(self):
        n = _ask_int('  re-rolls [8]: ', 8)
        sb = _ask_int('  settle-blank K [14]: ', 14)
        self.cam.lock(rerolls=n, settle_blank=sb)

    def _hwlock(self):
        on = _ask_yn('  enable HW-lock FSM?', True)
        self.cam.hwlock(on=on)

    # ---- live HDMI / image processing --------------------------------------
    def live(self):
        self._submenu('Live HDMI / processing', [
            ('Start live HDMI (blocking, N s)', self._hdmi),
            ('Start live HDMI (non-blocking)', self.cam.hdmi_on),
            ('HDMI passthrough / reset to colour', self.cam.passthrough),
            ('Filter combinations  >', self._filters_combo),
            ('Single filters       >', self._filters_single),
            ('Capture still -> _capture/', self._capture),
            ('Stop VDMA (safety)', self.cam.stop),
        ])

    def _filters_combo(self):
        self._submenu('Filter combinations', [
            ('Named preset (bin_edges / sketch / ...)', self._pipeline),
            ('Build custom chain (pre -> filter -> post -> dither)', self._build),
            ('Edge demo: cycle all + capture (live)', self._edge_demo),
            ('Dither demo: cycle all + capture (live)', self._dither_demo),
            ('Reset to colour (clear pre/post)', self.cam.passthrough),
        ])

    def _filters_single(self):
        self._submenu('Single filters', [
            ('Processing-slot op (point / conv select)', self._proc),
            ('PRE denoise (median / gaussian 3x3)', self._denoise),
            ('Named 3x3 kernel', self._kernel_named),
            ('Custom 3x3 kernel (9 coeffs + shift)', self._kernel_custom),
            ('DoG dual-kernel preset', self._dog),
            ('Cascade Gaussian blur (5/9/13)', self._blur),
            ('Sobel edge magnitude |Gx|+|Gy|', self._edges),
            ('Binarize -> Sobel edges (contours)', self._bin_edges),
            ('Sobel edges -> binarize (edge map)', self._edge_binary),
            ('Dither (after POST): bits + ordered/random', self._dither),
            ('Reset to colour (clear pre/post)', self.cam.passthrough),
        ])

    def _hdmi(self):
        secs = _ask_float('  duration seconds [60]: ', 60.0)
        self.cam.hdmi(secs)

    def _proc(self):
        op = _pick('processing op:', [
            ('passthrough (colour)', 0), ('invert', 1), ('grayscale', 2),
            ('BGR-swap', 3), ('threshold', 4), ('R-only', 5), ('G-only', 6),
            ('B-only', 7), ('conv (currently-loaded 3x3 kernel)', 8),
        ])
        if op is not None:
            self.cam.proc(op)

    def _denoise(self):
        kind = _pick('PRE 3x3 denoise:', [('median (impulse/salt-pepper)', 'median'),
                                          ('gaussian (blur)', 'gaussian'), ('off', 'off')])
        if kind is None:
            return
        if kind == 'off':
            self.cam.passthrough()
        else:
            self.cam.denoise(kind)

    def _dither(self):
        mode = _pick('dither mode:', [('ordered (Bayer 4x4)', 'ordered'),
                                      ('random (LFSR noise)', 'random'), ('off', 'off')])
        if mode is None:
            return
        if mode == 'off':
            self.cam.dither(0); return
        bits = _ask_int('  bits per channel (1=halftone .. 6=anti-banding) [1]: ', 1)
        self.cam.dither(bits, mode)

    def _kernel_named(self):
        opts = [(f'{nm:10s} {c} >>{s}', nm) for nm, (c, s) in CONV_KERNELS.items()]
        nm = _pick('named 3x3 kernel:', opts)
        if nm is not None:
            self.cam.k(nm)

    def _kernel_custom(self):
        print('  Enter 9 signed coeffs row-major (top-left .. bottom-right).')
        raw = _input('  coeffs (space/comma separated) [identity]: ').strip()
        if raw == '':
            coeffs = [0, 0, 0, 0, 1, 0, 0, 0, 0]
        else:
            try:
                coeffs = [int(x, 0) for x in raw.replace(',', ' ').split()]
            except ValueError:
                print('  (bad coeffs -- aborting)'); return
            if len(coeffs) != 9:
                print(f'  (need exactly 9 coeffs, got {len(coeffs)} -- aborting)'); return
        shift = _ask_int('  right-shift normalisation [0]: ', 0)
        self.cam.kernel(coeffs, shift)

    def _dog(self):
        opts = [(nm, nm) for nm in DOG_PRESETS]
        nm = _pick('DoG preset:', opts)
        if nm is not None:
            self.cam.dog(nm)

    def _blur(self):
        size = _pick('cascade blur size:', [
            ('5x5  (eff 5x5)', 5), ('9x9  (eff 9x9)', 9), ('13x13 (eff 13x13)', 13)])
        if size is not None:
            self.cam.blur(size)

    def _edges(self):
        sh = _ask_int('  gradient right-shift [2]: ', 2)
        self.cam.edges(sh)

    def _bin_edges(self):
        lvl = _ask_int('  binarize threshold (green, 0..255) [128]: ', 128)
        sh = _ask_int('  gradient right-shift [2]: ', 2)
        self.cam.bin_edges(lvl, sh)

    def _edge_binary(self):
        sh = _ask_int('  gradient right-shift [2]: ', 2)
        lvl = _ask_int('  edge-map threshold (0..255) [64]: ', 64)
        self.cam.edge_binary(lvl, sh)

    def _edge_demo(self):
        tp = _ask_yn('  inject test pattern (colour bar)?', False)
        dw = _ask_float('  seconds per stage [6]: ', 6.0)
        self.cam.edge_demo(dwell=dw, testpattern=tp)

    def _dither_demo(self):
        tp = _ask_yn('  inject test pattern (colour bar)?', False)
        dw = _ask_float('  seconds per stage [6]: ', 6.0)
        self.cam.dither_demo(dwell=dw, testpattern=tp)

    def _pipeline(self):
        opts = [(f'{nm:12s} {PIPELINES[nm]}', nm) for nm in PIPELINES]
        nm = _pick('filter combination (preset):', opts)
        if nm is not None:
            self.cam.pipeline(nm)
            if _ask_yn('  start live HDMI now?', False):
                self.cam.hdmi_on()

    _POINT_PICK = [('none', 0), ('invert', 1), ('grayscale', 2), ('BGR-swap', 3),
                   ('threshold', 4), ('R-only', 5), ('G-only', 6), ('B-only', 7)]
    # PRE stage adds 3x3 spatial denoise (gaussian/median) on top of the point ops.
    _PRE_PICK = _POINT_PICK + [('gaussian (3x3 blur)', 8), ('median (3x3 denoise)', 9)]
    _MID_PICK = [('none (point only)', 'none'), ('edges (omni Sobel |Gx|+|Gy|)', 'edges'),
                 ('sobel_x', 'k:sobel_x'), ('sobel_y', 'k:sobel_y'), ('gaussian', 'k:gaussian'),
                 ('sharpen', 'k:sharpen'), ('laplacian', 'k:laplacian'), ('outline', 'k:outline'),
                 ('emboss', 'k:emboss'), ('blur 5', 'blur:5'), ('blur 9', 'blur:9'),
                 ('blur 13', 'blur:13'), ('DoG blob', 'dog:blob'), ('DoG unsharp', 'dog:unsharp')]
    # DITHER stage (final, after POST). 'off' or (bits, mode); chain() maps 'off'->None.
    _DITHER_PICK = [('off', 'off'), ('halftone (1-bit ordered)', (1, 'ordered')),
                    ('posterize 2-bit', (2, 'ordered')), ('posterize 4-bit', (4, 'ordered')),
                    ('anti-banding 6-bit', (6, 'ordered')), ('random 2-bit', (2, 'random'))]

    def _build(self):
        """Interactive builder: pick PRE (denoise/point) -> spatial filter -> POST point-op."""
        pre = _pick('PRE stage (denoise / point-op, before the filter):', self._PRE_PICK)
        if pre is None:
            return
        pre_thr = _ask_int('  pre threshold (green, 0..255) [128]: ', 128) if pre == 4 else 128
        mid = _pick('SPATIAL filter:', self._MID_PICK)
        if mid is None:
            return
        post = _pick('POST point-op (after the filter):', self._POINT_PICK)
        if post is None:
            return
        post_thr = _ask_int('  post threshold (0..255) [64]: ', 64) if post == 4 else 64
        dith = _pick('DITHER (final stage, after POST):', self._DITHER_PICK)
        if dith is None:                       # cancelled
            return
        if mid == 'none':
            mspec = None
        elif mid == 'edges':
            mspec = 'edges'
        elif mid.startswith('k:'):
            mspec = ('kernel', mid[2:])
        elif mid.startswith('blur:'):
            mspec = ('blur', int(mid[5:]))
        elif mid.startswith('dog:'):
            mspec = ('dog', mid[4:])
        else:
            mspec = None
        self.cam.chain(pre=pre, mid=mspec, post=post, pre_thresh=pre_thr, post_thresh=post_thr,
                       dither=(None if dith == 'off' else dith))
        if _ask_yn('  start live HDMI now?', False):
            self.cam.hdmi_on()

    def _capture(self):
        nm = _input('  filename stem (blank = timestamp): ').strip() or None
        self.cam.capture(name=nm)

    # ---- registers / debug -------------------------------------------------
    def registers(self):
        self._submenu('Registers / debug', [
            ('SCCB read (addr)', self._read),
            ('SCCB write (addr, val)', self._write),
            ('Debug page read', self._dbg),
            ('Key register dump', lambda: self.cam.regs(False)),
            ('Full Linux-comparison dump', lambda: self.cam.regs(True)),
            ('Debug page scan', self.cam.pages),
            ('Frame-sync accounting', self.cam.accounting),
            ('IDELAY eye scan', self.cam.eye),
            ('Lock stability stats', self.cam.lockstats),
            ('Link measure', self.cam.link),
        ])

    def _read(self):
        a = _ask_int('  SCCB addr (hex) [0x300A]: ', 0x300A)
        self.cam.read(a)

    def _write(self):
        a = _ask_int('  SCCB addr (hex): ', None)
        if a is None:
            print('  (no addr -- aborting)'); return
        v = _ask_int('  value (hex): ', None)
        if v is None:
            print('  (no value -- aborting)'); return
        self.cam.write(a, v)

    def _dbg(self):
        p = _ask_int('  debug page (hex) [0x18]: ', 0x18)
        self.cam.dbg(p)

    # ---- knobs -------------------------------------------------------------
    def knobs(self):
        self._submenu('Knobs', [
            ('VCM focus DAC', self._vcm),
            ('IDELAY tap', self._idelay),
            ('Bitslip (FSM inhibited)', self._bitslip),
            ('Settle-blank K', self._settle),
            ('Re-roll /4 phase', self.cam.reroll),
            ('Output window pan (re-centre)', self._window),
            ('AGC gain ceiling', self._gain),
            ('Chip CIP sharpen', self._sharpen),
            ('Test pattern', self._testpattern),
        ])

    def _vcm(self):
        c = _ask_int('  VCM focus code (0..1023) [280]: ', 280)
        self.cam.vcm(c)

    def _idelay(self):
        t0 = _ask_int('  IDELAY tap lane0 [16]: ', 16)
        t1 = _ask_int(f'  IDELAY tap lane1 [{t0}]: ', t0)
        self.cam.idelay(t0, t1)

    def _bitslip(self):
        p0 = _ask_int('  bitslip lane0 [6]: ', 6)
        p1 = _ask_int('  bitslip lane1 [6]: ', 6)
        self.cam.bitslip(p0, p1)

    def _settle(self):
        k = _ask_int('  settle-blank K [14]: ', 14)
        self.cam.settle(k)

    def _window(self):
        print('  Pan the output window to re-centre on the lens axis (0,0 = init centre).')
        dx = _ask_int('  dx [0]: ', 0)
        dy = _ask_int('  dy [0]: ', 0)
        self.cam.window(dx, dy)

    def _gain(self):
        c = _ask_int('  AGC gain ceiling (e.g. 0x80=8x) [0x80]: ', 0x80)
        self.cam.gain_ceiling(c)

    def _sharpen(self):
        lv = _ask_int('  CIP sharpen level (0=auto, 1..0x3F manual) [0x20]: ', 0x20)
        self.cam.sharpen(lv)

    def _testpattern(self):
        val = _pick('test pattern:', [
            ('off (live sensor)', 0x00), ('vertical gradient 0x84', 0x84),
            ('colour bar 0x80', 0x80), ('custom 0x503D value...', -1)])
        if val == -1:
            val = _ask_int('  0x503D value (hex) [0x84]: ', 0x84)
        if val is not None:
            self.cam.testpattern(val)

    # ---- main loop ---------------------------------------------------------
    def run(self):
        actions = {'1': self.bringup, '2': self.live,
                   '3': self.registers, '4': self.knobs, 'h': self.cam.help}
        while True:
            print(_MAIN_MENU)
            sel = _input('  cam> ').strip().lower()
            if sel in ('q', 'quit', 'exit', '0'):
                print('  Leaving menu. `cam`, `h`, `ol` are live at the >>> prompt.')
                print('  Re-enter the menu any time with:  Menu(cam).run()')
                return
            fn = actions.get(sel)
            if not fn:
                print('  (unknown selection -- pick 1-4, h, or q)'); continue
            try:
                fn()
            except KeyboardInterrupt:
                print('\n  (cancelled -- back to main menu)')
            except Exception as e:
                print(f'  !! error: {e}')


_MAIN_MENU = """
============================================================
 Camera-control MENU
   1) Bring-up / status      (go / status / diagnostics)
   2) Live HDMI / processing (live / passthrough / filter combos / single filters / capture)
   3) Registers / debug      (read / write / dbg / regs / accounting / eye)
   4) Knobs                  (vcm / idelay / settle / window / gain / sharpen / testpattern)
   h) Command help (raw cam.* API)
   q) Quit menu -> drop to the >>> python prompt (cam still live)
============================================================"""


_BANNER = """
============================================================
 Camera-control REPL  (OV5640 / Pcam 5C -> HDMI)
   A menu opens below -- pick by number; free-form values are
   prompted with a [default] you can accept with Enter.
   Quit the menu (q) to drop to the >>> prompt, where the full
   cam.* API is live (cam.help() lists it). Re-open the menu
   with  Menu(cam).run().
============================================================
"""

if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--download', type=int, default=1)
    ap.add_argument('--go', type=int, default=0, help='1 = run cam.go() on startup')
    ap.add_argument('--diag', type=int, default=0, help='1 = run cam.diagnostics() on startup')
    ap.add_argument('--menu', type=int, default=1,
                    help='1 = launch the menu (default); 0 = straight to the >>> prompt')
    ap.add_argument('--edge-demo', type=int, default=0,
                    help='1 = run cam.edge_demo() on startup (needs --go 1); non-interactive '
                         'with --menu 0')
    ap.add_argument('--edge-testpattern', type=int, default=0,
                    help='edge_demo: 1 = inject OV5640 colour bar (lens-independent edges)')
    ap.add_argument('--edge-dwell', type=float, default=6.0,
                    help='edge_demo: seconds per stage')
    ap.add_argument('--dither-demo', type=int, default=0,
                    help='1 = run cam.dither_demo() on startup (needs --go 1); non-interactive with --menu 0')
    ap.add_argument('--dither-testpattern', type=int, default=0,
                    help='dither_demo: 1 = inject OV5640 colour bar (lens-independent)')
    ap.add_argument('--dither-dwell', type=float, default=6.0,
                    help='dither_demo: seconds per mode')
    ap.add_argument('--pipeline', default='',
                    help='apply a named filter combination on startup + capture one still to '
                         '/home/xilinx/repl_pipe_<name>.png (needs --go 1; non-interactive with --menu 0)')
    args, _ = ap.parse_known_args()
    ol, h = setup_session(download=bool(args.download))
    cam = Cam(ol, h)
    print(_BANNER)
    if args.go:
        cam.go()
    if args.diag:
        cam.diagnostics()
    if getattr(args, 'edge_demo', 0):
        cam.edge_demo(dwell=args.edge_dwell, testpattern=bool(args.edge_testpattern))
    if getattr(args, 'dither_demo', 0):
        cam.dither_demo(dwell=args.dither_dwell, testpattern=bool(args.dither_testpattern))
    if args.pipeline:
        cam.pipeline(args.pipeline)
        cam._alloc()
        configure_vdma_s2mm(cam._vdma, cam._bufs, start_mm2s=True, start_s2mm=True)
        try:
            time.sleep(2.0)
            cam._snap(args.pipeline, outdir=Path('/home/xilinx'), prefix='repl_pipe_')
        finally:
            cam.stop()
    if args.menu:
        try:
            Menu(cam).run()                # menu-first; quitting drops to >>> below
        except KeyboardInterrupt:
            print('\n  (menu interrupted -- dropping to >>> ; cam is live)')
    # python3 -i drops to the interactive prompt here with `cam`, `h`, `ol` in scope.
