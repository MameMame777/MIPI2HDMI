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
