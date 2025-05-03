from amaranth import Signal

__all__ = [
    "StreamInterface",  # Renamed from FIFOStreamInterface
    "USBPacketID",
    # Removed USBInStreamInterface, USBOutStreamInterface if they are part of old complexity
    # Assuming they are for now, can add back if needed.
]


class USBPacketID:
    OUT = 0b0001
    IN = 0b1001
    SOF = 0b0101
    SETUP = 0b1101
    DATA0 = 0b0011
    DATA1 = 0b1011
    DATA2 = 0b0111
    MDATA = 0b1111


class StreamInterface:  # Renamed from FIFOStreamInterface
    """Standard stream interface with valid/ready/payload."""

    def __init__(self, payload_width=8):
        self.payload_width = payload_width
        self.valid = Signal()
        self.ready = Signal()
        self.payload = Signal(payload_width)

    # Removed connect_fifo method, use standard .connect()
