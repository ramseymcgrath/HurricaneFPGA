# -*- coding: utf-8 -*-

from amaranth import Elaboratable, Module, Signal, Mux
from .interfaces import StreamInterface
from amaranth.lib.fifo import SyncFIFOBuffered

__all__ = ["CommandAckSystem", "UARTTXHandler"]


# --- Command Acknowledgment System ---
class CommandAckSystem(Elaboratable):
    """Provides acknowledgment and error responses for UART commands.
    Listens for commands being processed and sends appropriate responses.
    """

    # Response codes
    ACK_SUCCESS = 0x06  # ASCII ACK
    NAK_ERROR = 0x15  # ASCII NAK

    # Error codes - sent after NAK to provide more detail
    ERR_NONE = 0x00  # No error
    ERR_VALUE = 0x01  # Value out of range
    ERR_SYNTAX = 0x02  # Command syntax error
    ERR_BUSY = 0x03  # System busy
    ERR_OVERFLOW = 0x04  # Buffer overflow

    def __init__(self):
        # Input signals
        self.i_cmd_processed = Signal()  # Pulses when a command is processed
        self.i_cmd_error = Signal()  # Asserted if there was an error
        self.i_error_code = Signal(8, reset=self.ERR_NONE)  # Error code (if error)

        # UART TX stream for sending responses
        self.o_tx_stream = StreamInterface(payload_width=8)

        # Input UART stream (connected in parent module)
        self.i_uart_stream = StreamInterface(payload_width=8)

    def elaborate(self, platform):
        m = Module()

        # State machine to send acknowledgment
        sending = Signal()
        data_to_send = Signal(8)
        send_error_code = Signal()

        # Mux between our generated acks and passthrough data
        generating_ack = (
            Signal()
        )  # Indicates we're actively generating an ACK/NAK sequence

        # Default to passing through the input stream when not generating ACKs
        m.d.comb += [
            self.o_tx_stream.valid.eq(self.i_uart_stream.valid | sending),
            self.o_tx_stream.payload.eq(
                Mux(generating_ack, data_to_send, self.i_uart_stream.payload)
            ),
            self.o_tx_stream.first.eq(self.i_uart_stream.first & ~generating_ack),
            self.o_tx_stream.last.eq(self.i_uart_stream.last & ~generating_ack),
            # Only allow input stream to proceed when we're not generating ACKs and output is ready
            self.i_uart_stream.ready.eq(~generating_ack & self.o_tx_stream.ready),
        ]

        # State machine for sending responses
        with m.FSM(domain="sync", name="ack_fsm"):
            with m.State("IDLE"):
                # Not generating ACKs in IDLE state
                m.d.comb += generating_ack.eq(0)

                # Wait for command processed signal
                with m.If(self.i_cmd_processed):
                    # Start generating ACKs
                    m.d.comb += generating_ack.eq(1)

                    with m.If(self.i_cmd_error):
                        m.d.sync += [
                            data_to_send.eq(self.NAK_ERROR),
                            send_error_code.eq(1),
                        ]
                    with m.Else():
                        m.d.sync += [
                            data_to_send.eq(self.ACK_SUCCESS),
                            send_error_code.eq(0),
                        ]

                    m.d.sync += sending.eq(1)
                    m.next = "SENDING_ACK"

            with m.State("SENDING_ACK"):
                # We are generating ACKs in this state
                m.d.comb += generating_ack.eq(1)

                # Wait for ready signal from UART TX
                with m.If(self.o_tx_stream.ready):
                    m.d.sync += sending.eq(0)

                    # If this was a NAK, send the error code next
                    with m.If(send_error_code):
                        m.d.sync += [data_to_send.eq(self.i_error_code), sending.eq(1)]
                        m.next = "SENDING_ERROR_CODE"
                    with m.Else():
                        m.next = "IDLE"

            with m.State("SENDING_ERROR_CODE"):
                # Still generating ACKs in this state
                m.d.comb += generating_ack.eq(1)

                # Wait for ready signal from UART TX for error code
                with m.If(self.o_tx_stream.ready):
                    m.d.sync += sending.eq(0)
                    m.next = "IDLE"

        return m


class UARTTXHandler(Elaboratable):
    """Handles outgoing UART data transmission.
    Provides a stream interface for queuing data to be sent over UART.
    """

    def __init__(self, fifo_depth=16):
        # Input stream for queueing outgoing data
        self.i_tx_stream = StreamInterface(payload_width=8)

        # Connection to hardware UART TX
        self.o_uart_data = Signal(8)
        self.o_uart_valid = Signal()  # Set when we have valid data to send
        self.i_uart_ready = Signal()  # Set by UART when it's ready to accept data

        self._fifo_depth = fifo_depth

    def elaborate(self, platform):
        m = Module()

        # Create a FIFO to store outgoing messages
        m.submodules.tx_fifo = tx_fifo = SyncFIFOBuffered(
            width=8, depth=self._fifo_depth
        )

        # Connect input stream to the FIFO write port
        m.d.comb += [
            tx_fifo.w_en.eq(self.i_tx_stream.valid),
            tx_fifo.w_data.eq(self.i_tx_stream.payload),
            self.i_tx_stream.ready.eq(tx_fifo.w_rdy),
        ]

        # Connect FIFO read port to UART output
        m.d.comb += [
            self.o_uart_data.eq(tx_fifo.r_data),
            self.o_uart_valid.eq(tx_fifo.r_rdy),
            tx_fifo.r_en.eq(self.i_uart_ready & tx_fifo.r_rdy),
        ]

        return m
