"""Clock + reset bring-up matching the project convention: active-low *synchronous* reset
(``*_aresetn`` / ``rst_n``), 100 MHz default single clock, ``#4``/``#7`` for byte/core CDC.

Note some img_proc DUTs use *asynchronous* active-low reset (``@(posedge clk or negedge
rst_n)``); the same helpers drive them correctly (a synchronous release is a superset).
"""
from __future__ import annotations

from cocotb.clock import Clock
from cocotb.triggers import ClockCycles


def start_clock(clk, period_ns: float = 10.0, start_high: bool = False):
    """Start driving ``clk`` and return the clock Task. 10 ns = 100 MHz (mirrors ``#5``)."""
    return Clock(clk, period_ns, unit="ns").start(start_high=start_high)


async def reset_active_low(clk, rst_n, cycles: int = 8, post: int = 2) -> None:
    """Hold ``rst_n`` low for ``cycles`` clocks, release, settle ``post`` clocks."""
    rst_n.value = 0
    await ClockCycles(clk, cycles)
    rst_n.value = 1
    if post:
        await ClockCycles(clk, post)


async def bringup(dut, clk: str = "clk", rst: str = "rst_n",
                  period_ns: float = 10.0, cycles: int = 8, post: int = 2):
    """Start the clock and apply an active-low synchronous reset. Returns (clk, rst)."""
    clk_sig = getattr(dut, clk)
    rst_sig = getattr(dut, rst)
    start_clock(clk_sig, period_ns)
    await reset_active_low(clk_sig, rst_sig, cycles, post)
    return clk_sig, rst_sig


async def bringup_dual(dut, clk_a: str, rst_a: str, clk_b: str, rst_b: str,
                       period_a_ns: float = 10.0, period_b_ns: float = 14.0):
    """Two-clock bring-up for CDC blocks (e.g. core_clk #5 / aclk #7 ~= 10/14 ns).

    Both clocks start first, then both resets release, so neither domain samples the other
    while still held in reset.
    """
    ca, ra = getattr(dut, clk_a), getattr(dut, rst_a)
    cb, rb = getattr(dut, clk_b), getattr(dut, rst_b)
    start_clock(ca, period_a_ns)
    start_clock(cb, period_b_ns)
    ra.value = 0
    rb.value = 0
    await ClockCycles(ca, 8)
    ra.value = 1
    await ClockCycles(cb, 8)
    rb.value = 1
    await ClockCycles(cb, 4)
    return (ca, ra), (cb, rb)
