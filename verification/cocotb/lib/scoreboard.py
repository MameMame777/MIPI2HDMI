"""Pass/fail helpers.

``check(cond, msg)`` raises ``AssertionError("CHECK FAILED: ...")`` on failure -- the same
``CHECK FAILED`` token the old DSim testbenches emitted, so logs stay grep-compatible. A
test passes by returning normally (the cocotb analogue of ``$display("TEST PASSED")`` +
``$finish``).
"""
from __future__ import annotations

from typing import Any, List


def check(cond: Any, msg: str) -> None:
    if not cond:
        raise AssertionError(f"CHECK FAILED: {msg}")


def check_eq(got: Any, exp: Any, msg: str) -> None:
    if got != exp:
        raise AssertionError(f"CHECK FAILED: {msg} (got {got!r}, expected {exp!r})")


class Scoreboard:
    """Ordered expected-vs-actual queue compare."""

    def __init__(self, name: str = "scoreboard") -> None:
        self.name = name
        self.expected: List[Any] = []
        self.actual: List[Any] = []

    def expect(self, item: Any) -> None:
        self.expected.append(item)

    def observe(self, item: Any) -> None:
        self.actual.append(item)

    def check(self) -> None:
        check_eq(len(self.actual), len(self.expected), f"{self.name}: beat count")
        for i, (exp, got) in enumerate(zip(self.expected, self.actual)):
            check_eq(got, exp, f"{self.name}[{i}]")
