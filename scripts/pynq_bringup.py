"""Shared PYNQ session bring-up entry point.

Usage from any notebook (PYNQ-side):

    from pynq_bringup import setup_session
    ol, h = setup_session()
    # h['sccb_read'](0x300A) -> 0x56  (chip ID)

On local Windows the module imports cleanly but `setup_session()` itself
will fail at the `pynq` import — by design, since the function only
makes sense on a PYNQ board.
"""
from __future__ import annotations
import os
import sys
import time
from pathlib import Path

DEFAULT_BIT = '/home/xilinx/mipi2hdml/bd_wrapper.bit'


def setup_session(bit_path: str | None = None,
                  settle_s: float = 10.0,
                  raise_resetb: bool = True,
                  download: bool = True):
    """Load the overlay, wait for the bitstream-init FSM, return (ol, h).

    Parameters
    ----------
    bit_path
        Path to the .bit file. Defaults to /home/xilinx/mipi2hdml/bd_wrapper.bit.
    settle_s
        Seconds to wait after Overlay() for the bitstream-init FSM to finish
        its 232 SCCB writes. 10 s is the value verified in diary 20260525.
    raise_resetb
        If True, drive CAM_GPIO_BIT high via frame_lines_gpio so the chip's
        RESETB is released. Should always be True for normal capture; set
        False only for diagnostic runs that intentionally hold the chip in
        reset.
    download
        If True (default) the overlay is reprogrammed; if False the existing
        bitstream is kept and we only re-attach (the pattern from
        scripts/sccb_read_state.py).

    Returns
    -------
    (ol, h)
        ol = pynq.Overlay, h = dict of helpers from v65_capture.make_helpers
        (sccb_read, sccb_write, read_dbg, idelay_set, bitslip_set,
        frame_lines_write_raw, cam_resetb_pulse, frame_lines_set_keep_cam,
        snap, wait_sccb_idle).
    """
    here = Path(__file__).resolve().parent
    if str(here) not in sys.path:
        sys.path.insert(0, str(here))

    from pynq import Overlay
    from v65_capture import make_helpers, CAM_GPIO_BIT

    bit = bit_path or DEFAULT_BIT
    print(f'Loading bitstream {bit} (download={download})')
    ol = Overlay(bit, download=download)
    print(f'Overlay loaded, settle {settle_s:.1f}s for bitstream-init FSM')
    time.sleep(settle_s)
    h = make_helpers(ol)

    if raise_resetb:
        h['frame_lines_write_raw'](CAM_GPIO_BIT)
        time.sleep(0.1)

    return ol, h
