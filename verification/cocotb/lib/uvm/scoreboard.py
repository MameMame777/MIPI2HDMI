"""Base comparing scoreboard for the pyuvm layer.

A ``uvm_subscriber``: connect a monitor's analysis port to this component's
``analysis_export`` (``monitor.ap.connect(sb.analysis_export)``). Each observed item is
compared against an ordered ``expected`` list of items OR a ``predict(observed)->expected``
callback. Mismatch raises ``AssertionError('CHECK FAILED: ...')`` (the grep-compatible token
the DSim TBs and ``lib.scoreboard.check`` used). ``check_phase`` asserts nothing expected is
left unmatched.
"""
from __future__ import annotations

from pyuvm import uvm_subscriber

_MISSING = object()


class FrameScoreboard(uvm_subscriber):
    """Frame-assembling scoreboard for valid-only pixel streams (whole-image compare).

    ``write`` cheaply accumulates every observed beat; ``check_phase`` then verifies
    (1) the beat count, (2) the sof/eol/eof/err framing geometry of every beat, and
    (3) every pixel against the expected image set via ``set_expected``. Mismatches are
    reported as (frame, row, col) with got/exp hex; the full list can be dumped to
    ``<report_dir>/mismatches.txt``. Accumulate-then-check keeps the per-beat cost to a
    tuple append (307k beats for VGA), and lets the test save the output image BEFORE
    check_phase can raise."""

    def __init__(self, name, parent):
        super().__init__(name, parent)
        self.beats = []          # (pixel, sof, eol, eof, err) per observed beat
        self._exp = None
        self._geom = None        # (width, height, frames)
        self.report_dir = None   # optional Path: mismatch dump target

    def write(self, item):
        self.beats.append((item.pixel, item.sof, item.eol, item.eof, item.err))

    def set_expected(self, pixels, width, height, frames=1):
        self._exp = list(pixels)
        self._geom = (int(width), int(height), int(frames))
        total = self._geom[0] * self._geom[1] * self._geom[2]
        if len(self._exp) != total:
            raise ValueError(f"expected {total} pixels, got {len(self._exp)}")

    def observed_pixels(self):
        return [b[0] for b in self.beats]

    def check_phase(self):
        if self._geom is None:
            raise AssertionError(
                f"CHECK FAILED: {self.get_name()}: set_expected() never called")
        w, h, frames = self._geom
        fsz = w * h
        total = fsz * frames
        problems = []
        if len(self.beats) != total:
            problems.append(f"beat count {len(self.beats)} != expected {total} "
                            f"(w={w} h={h} frames={frames})")

        marker_bad = 0
        mismatches = []
        for k, (px, sof, eol, eof, err) in enumerate(self.beats[:total]):
            f, j = divmod(k, fsz)
            r, c = divmod(j, w)
            exp_mk = (1 if j == 0 else 0, 1 if c == w - 1 else 0,
                      1 if j == fsz - 1 else 0, 0)
            if (sof, eol, eof, err) != exp_mk:
                marker_bad += 1
                if marker_bad <= 5:
                    problems.append(
                        f"marker[f{f} r{r} c{c}] got sof/eol/eof/err="
                        f"{(sof, eol, eof, err)} exp {exp_mk}")
            if px != self._exp[k]:
                mismatches.append((f, r, c, px, self._exp[k]))
        if marker_bad > 5:
            problems.append(f"... {marker_bad} marker mismatches total")
        for f, r, c, got, exp in mismatches[:10]:
            problems.append(f"pixel[f{f} r{r} c{c}] got 0x{got:06x} exp 0x{exp:06x}")
        if len(mismatches) > 10:
            problems.append(f"... {len(mismatches)} pixel mismatches total")

        if mismatches and self.report_dir is not None:
            try:
                path = self.report_dir / "mismatches.txt"
                with open(path, "w", encoding="utf-8") as fh:
                    for f, r, c, got, exp in mismatches[:10000]:
                        fh.write(f"f{f} r{r} c{c} got 0x{got:06x} exp 0x{exp:06x}\n")
                problems.append(f"full list: {path}")
            except OSError:
                pass

        if problems:
            raise AssertionError(
                f"CHECK FAILED: {self.get_name()}: " + "; ".join(problems))


class Scoreboard(uvm_subscriber):
    def __init__(self, name, parent, expected=None, predict=None, key=None):
        super().__init__(name, parent)
        self._expected = list(expected) if expected is not None else []
        self._predict = predict
        # key extracts the compared fields from an item; default = the item's full key().
        # Pass e.g. key=lambda it: (it.data, it.last) to ignore some fields. Both observed
        # and expected must be items the key() applies to.
        self._key = key or (lambda x: x.key() if hasattr(x, "key") else x)
        self.matched = 0

    def write(self, item):
        if self._predict is not None:
            exp = self._predict(item)
        elif self._expected:
            exp = self._expected.pop(0)
        else:
            exp = _MISSING
        if exp is _MISSING:
            raise AssertionError(
                f"CHECK FAILED: {self.get_name()}: unexpected item {item!r}")
        if self._key(item) != self._key(exp):
            raise AssertionError(
                f"CHECK FAILED: {self.get_name()}[{self.matched}] "
                f"(got {item!r}, expected {exp!r})")
        self.matched += 1

    def check_phase(self):
        if self._predict is None and self._expected:
            raise AssertionError(
                f"CHECK FAILED: {self.get_name()}: {len(self._expected)} expected item(s) "
                f"never observed (matched {self.matched})")
