# -*- coding: utf-8 -*-
# Only import the simplified handler and UARTTXHandler from passthrough
from .passthrough import (
    USBDataPassthroughHandler,
    UARTTXHandler,
    # Removed PacketArbiter
    # Removed PHYTranslatorHandler
)
from .interfaces import (
    USBPacketID,
    StreamInterface,
    # Removed USBInStreamInterface
    # Removed USBOutStreamInterface
)

# Corrected duplicate MouseCommandParser import
from .mouse_injector import SimpleMouseInjector, MouseCommandParser
from .utils import AsyncSerialRX, AsyncSerialTX, ActivityMonitor, LEDController

# Only StreamFIFO is defined in fifo.py now
from .fifo import StreamFIFO  # Assuming StreamFIFO should be imported/exported

# Removed HyperRAMPacketFIFO, Stream16to8, SyncFIFOBuffered imports
from .usb_serial import (
    USBSerialDevice,
    USBRequestHandler,
    USBStreamInEndpoint,
    USBStreamOutEndpoint,
)
from .uart import CommandAckSystem, UARTTXHandler

__all__ = [
    "USBDataPassthroughHandler",  # Keep simplified handler
    "SimpleMouseInjector",
    "MouseCommandParser",
    # Removed HyperRAMPacketFIFO
    "StreamInterface",  # Keep standard interface
    "StreamFIFO",  # Added StreamFIFO export
    "AsyncSerialRX",
    "AsyncSerialTX",
    "ActivityMonitor",
    "LEDController",
    # Removed USBInStreamInterface
    # Removed USBOutStreamInterface
    "USBPacketID",  # Keep
    "CommandAckSystem",
    "UARTTXHandler",  # Keep (imported above)
    # Removed PacketArbiter
    # Removed PHYTranslatorHandler
    "USBSerialDevice",
    "USBRequestHandler",
    "USBStreamInEndpoint",
    "USBStreamOutEndpoint",
    # Removed Stream16to8
    # Removed SyncFIFOBuffered (usually imported directly from amaranth.lib.fifo)
    "AsyncFIFOBuffered",  # Keep? (used in simplified passthrough for CDC)
]
