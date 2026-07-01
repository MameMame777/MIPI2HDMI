"""Base env + base test for the pyuvm layer.

``UvmEnv`` is a thin ``uvm_env`` base -- concrete envs subclass it to create agents +
scoreboard (build_phase) and wire ``monitor.ap -> scoreboard.analysis_export`` (connect_phase).

``UvmTest`` is the base test: a subclass sets ``clock_specs``, overrides ``build_phase``
(create the env + set ConfigDB) and ``async stimulus()`` (run sequences). ``run_phase``
raises an objection, brings up the clock(s)/reset via ``lib.clkreset.bringup_n``, runs the
stimulus, drains, and drops the objection; scoreboards assert in their ``check_phase``.
"""
from __future__ import annotations

import cocotb
from cocotb.triggers import ClockCycles
from pyuvm import uvm_env, uvm_test

from lib.clkreset import bringup_n


class UvmEnv(uvm_env):
    """Base environment. Subclass and create agents/scoreboard in build_phase."""


class UvmTest(uvm_test):
    clock_specs: list = []      # [(clk_name, rstn_name, period_ns), ...]
    stagger: bool = True        # staggered (CDC) vs same-edge reset release
    drain_cycles: int = 20      # extra cycles after stimulus before ending

    async def stimulus(self):
        """Override: start sequences on the env's sequencers and await completion."""

    async def run_phase(self):
        self.raise_objection()
        self.clock_pairs = await bringup_n(
            cocotb.top, self.clock_specs, stagger=self.stagger)
        await self.stimulus()
        await ClockCycles(self.clock_pairs[0][0], self.drain_cycles)
        self.drop_objection()
