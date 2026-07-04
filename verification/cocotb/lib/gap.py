"""Deterministic valid-gap / backpressure injection for handshake-robustness stress.

The img_proc datapaths gate every state update on ``in_valid`` (col counter, line-buffer
writes, window shift, dither LFSR advance), but the plain drivers feed *continuous* valid
(1 beat/clk), so a bug that advances a line buffer / window / LFSR on the raw clock instead
of on ``in_valid`` is structurally unreachable by the directed suite. Injecting random idle
(valid=0) cycles between accepted beats -- and, on true-AXIS blocks, random ``tready``
backpressure -- exercises exactly that path.

Key property that makes this cheap: the golden output is a pure function of the ACCEPTED-BEAT
sequence, independent of inter-beat idle timing, so the existing bit-exact scoreboards need
NO change. A mismatch under gaps is a real handshake bug, not a timing artefact.

Off by default: ``COCOTB_GAP`` unset -> kind ``none`` -> ``next_gap()`` returns 0 ->
byte-identical to the continuous-valid behaviour every existing test was written against.
Seeded from ``COCOTB_SEED`` (pinned to 1 by runner_support) so a stress run is reproducible.

Turn it on for a whole run with the runner:  ``run_cocotb.ps1 -Suite stress -Gap sparse``
(the runner exports ``COCOTB_GAP`` for the selected blocks; see runner.py ``--gap``).

Env:
  COCOTB_GAP       none | sparse | burst | adversarial   (default none)
  COCOTB_GAP_MAX   max idle cycles per injected gap        (default 3)
"""
from __future__ import annotations

import os
import random
from collections import Counter
from typing import Optional

KINDS = ("none", "sparse", "burst", "adversarial")


class GapPolicy:
    """Yields the number of idle cycles to insert before each accepted beat.

    * ``none``        -- always 0 (continuous valid; the default).
    * ``sparse``      -- a 1..max_gap stall before ~``prob`` of beats, else 0.
    * ``burst``       -- a uniform 0..max_gap stall before every beat.
    * ``adversarial`` -- a 1..max_gap stall before EVERY beat (worst-case single-beat bursts).

    ``produced`` is a histogram {gap_size: count} of what was actually emitted -- read it after
    a run to prove (via the coverage tally) that stalls really occurred.
    """

    def __init__(self, kind: str = "none", seed: int = 1, max_gap: int = 3,
                 prob: float = 0.5) -> None:
        if kind not in KINDS:
            raise ValueError(f"gap kind {kind!r}: choose from {KINDS}")
        self.kind = kind
        self.max_gap = max(0, int(max_gap))
        self.prob = prob
        self.rng = random.Random(seed)
        self.produced: Counter = Counter()

    @property
    def active(self) -> bool:
        return self.kind != "none" and self.max_gap > 0

    def next_gap(self) -> int:
        if not self.active:
            g = 0
        elif self.kind == "sparse":
            g = self.rng.randint(1, self.max_gap) if self.rng.random() < self.prob else 0
        elif self.kind == "burst":
            g = self.rng.randint(0, self.max_gap)
        else:  # adversarial
            g = self.rng.randint(1, self.max_gap)
        self.produced[g] += 1
        return g


def _seed() -> int:
    try:
        return int(os.environ.get("COCOTB_SEED", "1"), 0)
    except ValueError:
        return 1


def make_gap_policy() -> GapPolicy:
    """Build a fresh policy from COCOTB_GAP / COCOTB_GAP_MAX / COCOTB_SEED."""
    try:
        max_gap = int(os.environ.get("COCOTB_GAP_MAX", "3"), 0)
    except ValueError:
        max_gap = 3
    return GapPolicy(kind=os.environ.get("COCOTB_GAP", "none").lower(),
                     seed=_seed(), max_gap=max_gap)


_DEFAULT: Optional[GapPolicy] = None


def default_gap_policy() -> GapPolicy:
    """Process-wide default, built once from the environment. Shared by the plain drivers so
    a single ``COCOTB_GAP=...`` re-runs the whole directed suite under gaps with no per-test
    edits. Deterministic: one input driver per test advances it in a fixed order."""
    global _DEFAULT
    if _DEFAULT is None:
        _DEFAULT = make_gap_policy()
    return _DEFAULT


def reset_default_gap_policy() -> None:
    """Test hook: drop the cached default so the next call rereads the environment."""
    global _DEFAULT
    _DEFAULT = None
