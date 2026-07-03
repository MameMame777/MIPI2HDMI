"""Sequence items (transactions) for the pyuvm base.

Fields mirror the plain lib (`byte_beat.Beat`, pixel, axis) so the two verification layers
stay aligned. Each item exposes ``key()`` -> a comparable tuple, which the Scoreboard uses
(avoids overriding ``__eq__``/``__hash__`` on ``uvm_object``).
"""
from __future__ import annotations

from pyuvm import uvm_sequence_item


class ByteBeatItem(uvm_sequence_item):
    def __init__(self, name="byte_beat_item", data=0, keep=0b11, sop=0, eop=0):
        super().__init__(name)
        self.data = int(data)
        self.keep = int(keep)
        self.sop = int(sop)
        self.eop = int(eop)

    def key(self):
        return (self.data, self.keep, self.sop, self.eop)

    def __repr__(self):
        return (f"ByteBeatItem(data=0x{self.data:x}, keep=0b{self.keep:b}, "
                f"sop={self.sop}, eop={self.eop})")


class PixelItem(uvm_sequence_item):
    def __init__(self, name="pixel_item", pixel=0, sof=0, eol=0, eof=0, err=0):
        super().__init__(name)
        self.pixel = int(pixel)
        self.sof = int(sof)
        self.eol = int(eol)
        self.eof = int(eof)
        self.err = int(err)

    def key(self):
        return (self.pixel, self.sof, self.eol, self.eof, self.err)

    def __repr__(self):
        return (f"PixelItem(pixel=0x{self.pixel:x}, sof={self.sof}, eol={self.eol}, "
                f"eof={self.eof}, err={self.err})")


class ImageFrameItem(uvm_sequence_item):
    """One whole video frame as a single transaction (flat row-major 24-bit RGB pixels).

    Frame-level granularity keeps the pyuvm sequencer handshake off the per-pixel path:
    a 640x480 frame is ONE get_next_item/item_done round-trip instead of 307200, and the
    driver can stream continuous-valid at 1 px/clk via PixelStreamDriver.send_frame."""

    def __init__(self, name="image_frame_item", pixels=None, width=0, err=0):
        super().__init__(name)
        self.pixels = list(pixels or [])
        self.width = int(width)
        self.err = int(err)

    def key(self):
        return (tuple(self.pixels), self.width, self.err)

    def __repr__(self):
        return (f"ImageFrameItem(pixels={len(self.pixels)}, width={self.width}, "
                f"err={self.err})")


class AxisItem(uvm_sequence_item):
    def __init__(self, name="axis_item", data=0, last=0, user=0):
        super().__init__(name)
        self.data = int(data)
        self.last = int(last)
        self.user = int(user)

    def key(self):
        return (self.data, self.last, self.user)

    def __repr__(self):
        return f"AxisItem(data=0x{self.data:x}, last={self.last}, user={self.user})"
