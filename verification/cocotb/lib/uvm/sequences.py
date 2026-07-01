"""Base sequences for the pyuvm layer."""
from __future__ import annotations

from pyuvm import uvm_sequence


class ItemsSequence(uvm_sequence):
    """Drive a fixed list of pre-built sequence items, in order.

        seq = ItemsSequence("s", items=[PixelItem(pixel=0x10, sof=1), ...])
        await seq.start(agent.seqr)
    """

    def __init__(self, name="items_seq", items=None):
        super().__init__(name)
        self.items = list(items or [])

    async def body(self):
        for it in self.items:
            await self.start_item(it)
            await self.finish_item(it)
