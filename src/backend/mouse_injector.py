#!/usr/bin/env python3

# -*- coding: utf-8 -*-

from amaranth import Elaboratable, Module, Signal
from amaranth.lib.fifo import SyncFIFOBuffered
from .interfaces import USBInStreamInterface, USBPacketID, StreamInterface

__all__ = ["SimpleMouseInjector", "MouseCommandParser"]


# --- Constants and Helper Classes ---


class SimpleMouseInjector(Elaboratable):
    def __init__(self):
        self.source = USBInStreamInterface(payload_width=8)
        self.trigger = Signal()
        self.buttons = Signal(8)
        self.dx = Signal(8)
        self.dy = Signal(8)
        self._data_pid = Signal(4, reset=USBPacketID.DATA1)

    def elaborate(self, platform):
        m = Module()
        source = self.source
        latched_buttons = Signal.like(self.buttons)
        latched_dx = Signal.like(self.dx)
        latched_dy = Signal.like(self.dy)
        m.d.comb += [
            source.valid.eq(0),
            source.first.eq(0),
            source.last.eq(0),
            source.payload.eq(0),
        ]
        with m.FSM(domain="usb", name="injector_fsm"):
            with m.State("IDLE"):
                with m.If(self.trigger):
                    m.d.usb += [
                        latched_buttons.eq(self.buttons),
                        latched_dx.eq(self.dx),
                        latched_dy.eq(self.dy),
                    ]
                    m.next = "SEND_PID"
            with m.State("SEND_PID"):
                m.d.comb += [
                    source.valid.eq(1),
                    source.payload.eq(self._data_pid),
                    source.first.eq(1),
                    source.last.eq(0),
                ]
                with m.If(source.ready):
                    m.next = "SEND_BYTE_0"
            with m.State("SEND_BYTE_0"):  # Buttons
                m.d.comb += [
                    source.valid.eq(1),
                    source.payload.eq(latched_buttons),
                    source.first.eq(0),
                    source.last.eq(0),
                ]
                with m.If(source.ready):
                    m.next = "SEND_BYTE_1"
            with m.State("SEND_BYTE_1"):  # DX
                m.d.comb += [
                    source.valid.eq(1),
                    source.payload.eq(latched_dx),
                    source.first.eq(0),
                    source.last.eq(0),
                ]
                with m.If(source.ready):
                    m.next = "SEND_BYTE_2"
            with m.State("SEND_BYTE_2"):  # DY
                m.d.comb += [
                    source.valid.eq(1),
                    source.payload.eq(latched_dy),
                    source.first.eq(0),
                    source.last.eq(1),
                ]
                with m.If(source.ready):
                    m.next = "IDLE"
        return m


class MouseCommandParser(Elaboratable):
    """
    Parses a 3-byte mouse command sequence (buttons, dx, dy) from
    an incoming byte stream. Operates in the 'sync' clock domain.

    Input Stream format:
        - byte 0: buttons
        - byte 1: dx
        - byte 2: dy

    Special commands:
        - [0xA5, 0x5A, 0xFF]: Handshake request - will respond with ACK
                              and set handshake_complete flag

    Attributes
    ----------
    i_uart_stream : StreamInterface, input
        Stream carrying received UART bytes.
    o_buttons : Signal(8), output
        Parsed button state from the command sequence.
    o_dx : Signal(8), output
        Parsed dx value from the command sequence.
    o_dy : Signal(8), output
        Parsed dy value from the command sequence.
    o_cmd_ready : Signal, output
        Strobes high for one clock cycle when a complete 3-byte
        command sequence has been successfully parsed.
    o_handshake_complete : Signal, output
        Set high when handshake with host is completed.
    """

    # Special command constants
    HANDSHAKE_BYTE1 = 0xA5
    HANDSHAKE_BYTE2 = 0x5A
    HANDSHAKE_BYTE3 = 0xFF

    def __init__(self, input_fifo_depth=16):
        # Input Stream (sync domain)
        self.i_uart_stream = StreamInterface(payload_width=8)

        # Outputs (sync domain)
        self.o_buttons = Signal(8)
        self.o_dx = Signal(8)
        self.o_dy = Signal(8)
        self.o_cmd_ready = Signal()  # Pulse high for one cycle when command complete

        # Handshake status
        self.o_handshake_complete = (
            Signal()
        )  # Persistent signal indicating handshake completed

        self._input_fifo_depth = input_fifo_depth

    def elaborate(self, platform):
        m = Module()

        # --- Input FIFO (Buffers incoming stream bytes) ---
        m.submodules.rx_fifo = rx_fifo = SyncFIFOBuffered(
            width=8, depth=self._input_fifo_depth
        )

        # Connect input stream to the FIFO write port
        m.d.comb += [
            rx_fifo.w_en.eq(self.i_uart_stream.valid),
            rx_fifo.w_data.eq(self.i_uart_stream.payload),
            self.i_uart_stream.ready.eq(rx_fifo.w_rdy),
        ]

        # --- Command Parsing FSM ---
        temp_buttons = Signal(8)
        temp_dx = Signal(8)

        # Default outputs
        m.d.comb += self.o_cmd_ready.eq(0)  # o_cmd_ready is combinatorially pulsed
        handshake_detected = Signal()  # Internal signal to detect handshake pattern

        # Control FIFO read enable
        # Read enable is asserted combinatorially based on state and fifo ready,
        # but registered assignment happens within the FSM sync block if needed,
        # however, safer to keep it combinatorial reacting to r_rdy.
        fifo_read_enable = Signal()
        m.d.comb += rx_fifo.r_en.eq(fifo_read_enable)  # Connect combo signal to r_en

        with m.FSM(domain="sync", name="cmd_parse_fsm"):
            # IDLE: Wait for the first byte (buttons)
            with m.State("IDLE"):
                m.d.comb += fifo_read_enable.eq(0)  # Default don't read
                with m.If(rx_fifo.r_rdy):  # If a byte is available
                    m.d.comb += fifo_read_enable.eq(1)  # Enable read for this cycle
                    m.d.sync += temp_buttons.eq(
                        rx_fifo.r_data
                    )  # Latch data on next cycle
                    m.next = "WAIT_DX"

            # WAIT_DX: Wait for the second byte (dx)
            with m.State("WAIT_DX"):
                m.d.comb += fifo_read_enable.eq(0)
                with m.If(rx_fifo.r_rdy):
                    m.d.comb += fifo_read_enable.eq(1)
                    m.d.sync += temp_dx.eq(rx_fifo.r_data)
                    m.next = "WAIT_DY"

            # WAIT_DY: Wait for the third byte (dy)
            with m.State("WAIT_DY"):
                m.d.comb += fifo_read_enable.eq(0)
                with m.If(rx_fifo.r_rdy):
                    m.d.comb += fifo_read_enable.eq(1)

                    # Check for handshake pattern [0xA5, 0x5A, 0xFF]
                    m.d.sync += handshake_detected.eq(
                        (temp_buttons == self.HANDSHAKE_BYTE1)
                        & (temp_dx == self.HANDSHAKE_BYTE2)
                        & (rx_fifo.r_data == self.HANDSHAKE_BYTE3)
                    )

                    # Process as normal mouse command
                    m.d.sync += [
                        self.o_buttons.eq(temp_buttons),
                        self.o_dx.eq(temp_dx),
                        self.o_dy.eq(rx_fifo.r_data),
                    ]

                    # Pulse o_cmd_ready combinatorially for this cycle only
                    m.d.comb += self.o_cmd_ready.eq(1)
                    m.next = "PROCESS_COMMAND"

            with m.State("PROCESS_COMMAND"):
                # Check if this was a handshake command
                with m.If(handshake_detected):
                    # Set persistent handshake status
                    m.d.sync += self.o_handshake_complete.eq(1)
                with m.Else():
                    # Normal command, do nothing special
                    pass

                # Back to IDLE state for the next command
                m.next = "IDLE"

        return m
