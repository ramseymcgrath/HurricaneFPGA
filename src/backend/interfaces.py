from amaranth import Signal

__all__ = [
    "StreamInterface",
    "USBInStreamInterface",
    "USBOutStreamInterface",
    "USBPacketID",
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


class StreamInterface:
    def __init__(self, payload_width=8):
        self.payload_width = payload_width
        self.valid = Signal()
        self.ready = Signal()
        self.payload = Signal(payload_width)
        self.first = Signal()
        self.last = Signal()

    def stream_eq(self, other, *, omit=None):
        if omit is None:
            omit = set()
        elif isinstance(omit, str):
            omit = {omit}
        connect_list = []
        if "valid" not in omit:
            connect_list.append(other.valid.eq(self.valid))
        if "payload" not in omit:
            connect_list.append(other.payload.eq(self.payload))
        if "first" not in omit:
            connect_list.append(other.first.eq(self.first))
        if "last" not in omit:
            connect_list.append(other.last.eq(self.last))
        # Add ready connection in reverse
        if "ready" not in omit:
            connect_list.append(self.ready.eq(other.ready))
        return connect_list


class USBInStreamInterface(StreamInterface):
    def __init__(self, payload_width=8):
        super().__init__(payload_width)


class USBOutStreamInterface(StreamInterface):
    def __init__(self, payload_width=8):
        super().__init__(payload_width)
