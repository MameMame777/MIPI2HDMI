"""Lightweight stdlib functional-coverage tally.

No third-party dependency (deliberately NOT ``cocotb-coverage``): the interesting img_proc
conditions are behavioral, not directly driven -- saturation clamped to 0 vs 255, a
border-vs-interior pixel, a threshold boundary crossed, a dither mode/bit-depth, an LFSR
wrap, an injected valid-gap size. They are few and enumerable, so a dict-of-Counters tally
sampled in a monitor/scoreboard (or host-side directly on golden outputs -- both work, since
the tally has no cocotb dependency) is enough to turn "all green" into "all green AND here is
what the stimulus actually reached".

``assert_covered`` raises the house ``CHECK FAILED`` token so a coverage hole fails a test
just like any other check.
"""
from __future__ import annotations

from collections import Counter
from typing import Dict, Iterable

from lib.scoreboard import check


class CoverageTally:
    def __init__(self, name: str = "coverage") -> None:
        self.name = name
        self.groups: Dict[str, Counter] = {}

    def sample(self, group: str, bin_) -> None:
        self.groups.setdefault(group, Counter())[bin_] += 1

    def hits(self, group: str, bin_) -> int:
        return self.groups.get(group, Counter()).get(bin_, 0)

    def hit(self, group: str, bin_) -> bool:
        return self.hits(group, bin_) > 0

    def covered(self, group: str) -> set:
        return set(self.groups.get(group, {}))

    def assert_covered(self, group: str, required: Iterable) -> None:
        missing = [b for b in required if not self.hit(group, b)]
        check(not missing, f"{self.name}: group {group!r} bins never hit: {missing}")

    def merge_counter(self, group: str, counter: Counter) -> None:
        """Fold an externally-collected histogram (e.g. GapPolicy.produced) into a group."""
        self.groups.setdefault(group, Counter()).update(counter)

    def summary(self) -> str:
        lines = [f"COVERAGE {self.name}:"]
        for g, c in sorted(self.groups.items()):
            total = sum(c.values())
            bins = ", ".join(f"{b}={n}" for b, n in
                             sorted(c.items(), key=lambda kv: str(kv[0])))
            lines.append(f"  {g} ({len(c)} bins, {total} samples): {bins}")
        return "\n".join(lines)
