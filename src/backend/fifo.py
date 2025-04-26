from amaranth import Elaboratable, Module, Signal
from amaranth.lib.fifo import SyncFIFO
from .interfaces import StreamInterface

__all__ = ["StreamFIFO"]

# --- FIFO Interfaces ---


class StreamFIFO(Elaboratable):
    def __init__(self, width=8, depth=64):
        self.input = StreamInterface(payload_width=width)
        self.output = StreamInterface(payload_width=width)
        self.depth = depth

    def elaborate(self, platform):
        m = Module()
        m.submodules.fifo = fifo = SyncFIFO(
            width=self.input.payload_width, depth=self.depth
        )

        m.d.comb += [
            # Input side (host -> FIFO)
            fifo.w_en.eq(self.input.valid & self.input.ready),
            fifo.w_data.eq(self.input.payload),
            self.input.ready.eq(fifo.w_rdy),
            # Output side (FIFO -> device)
            self.output.valid.eq(fifo.r_rdy),
            self.output.payload.eq(fifo.r_data),
            fifo.r_en.eq(self.output.ready),
        ]

        return m
