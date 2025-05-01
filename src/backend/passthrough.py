#!/usr/bin/env python3

# -*- coding: utf-8 -*-

# Removed unused imports
from amaranth import Elaboratable, Module, Signal, DomainRenamer, Cat, C, Record, Array
from amaranth.build.res import ResourceError

# from amaranth.lib.fifo import SyncFIFOBuffered, AsyncFIFOBuffered # Removed
from luna.gateware.interface.ulpi import UTMITranslator  # Keep for Control Port
from amaranth.lib.cdc import FFSynchronizer  # Keep for simple signal CDC

# from luna.gateware.interface.psram import HyperRAMInterface, HyperRAMPHY # Removed
from .mouse_injector import SimpleMouseInjector, MouseCommandParser

# from .fifo import HyperRAMPacketFIFO # Removed
from .utils import AsyncSerialRX, AsyncSerialTX
from .uart import UARTTXHandler
from .usb_serial import USBSerialDevice

# Import the interfaces from the shared module
from .interfaces import StreamInterface

__all__ = [
    "USBDataPassthroughHandler",
    
]

# --- Removed PacketArbiter Class ---
# --- Removed PHYTranslatorHandler Class ---


class USBDataPassthroughHandler(Elaboratable):
    """
    Handles command processing (UART/USB Serial) and mouse event injection.
    The core USB data path (Host A <-> FIFO <-> Device C) is now handled in top.py.
    """

    DEFAULT_UART_BAUD = 115200

    def __init__(self, uart_baud_rate=None):
        self.uart_baud_rate = (
            uart_baud_rate if uart_baud_rate else self.DEFAULT_UART_BAUD
        )

        # Outputs for status LEDs
        # Removed o_host_packet_activity, o_dev_packet_activity as they were tied to old translators/paths
        self.o_uart_rx_activity = Signal()  # Activity on UART RX
        self.o_uart_tx_activity = Signal()  # Activity on UART TX / USB Serial TX
        self.o_command_rx_activity = (
            Signal()
        )  # Activity on command interface (UART or USB Serial RX)

        # Stream interface for sending data out via UART TX / USB Serial TX
        # This stream is driven externally (e.g., by debug modules)
        self.i_uart_tx_stream = StreamInterface(payload_width=8)

        # Signals for injector (inputs to this module)
        # These are driven by the command parser after CDC
        # self.i_inject_trigger = Signal() # No longer direct inputs
        # self.i_buttons = Signal(8)
        # self.i_dx = Signal(8)
        # self.i_dy = Signal(8)

        # Internal references to submodules (populated during elaborate)
        self._cmd_parser = None
        self.use_control_port = (
            False  # Flag to indicate if USB Serial command interface is used
        )
        # Add references for UART/Serial if needed externally
        self._uart_rx = None
        self._uart_tx = None
        self._usb_serial = None

    def get_cmd_parser(self):
        """Returns the command parser submodule instance."""
        return self._cmd_parser

    def elaborate(self, platform):
        m = Module()

        # --- Get Clock Frequency ---
        try:
            # Try to get the specific clock first
            clk_freq = platform.lookup(platform.default_clk).clock.frequency
        except (ResourceError, AttributeError, KeyError):
            # Fallback if default_clk lookup fails or doesn't have frequency
            try:
                clk_freq = platform.lookup("clk_60MHz").clock.frequency
            except (ResourceError, AttributeError, KeyError):
                # Assume 60MHz if platform doesn't define it (e.g., simulation)
                clk_freq = 60e6
                print(
                    "WARNING: Platform does not define default_clk or clk_60MHz frequency, assuming 60MHz."
                )
        print(f"USBPassthroughHandler using clk_freq: {clk_freq / 1e6} MHz")

        # --- UART Setup (Always needed for fallback or dedicated use) ---
        try:
            uart_pins = platform.request("uart", 0)
            # Use DomainRenamer if UART needs to run in 'sync' domain explicitly
            self._uart_rx = uart_rx = DomainRenamer("sync")(
                AsyncSerialRX(divisor=int(clk_freq // self.uart_baud_rate))
            )
            self._uart_tx = uart_tx = DomainRenamer("sync")(
                AsyncSerialTX(divisor=int(clk_freq // self.uart_baud_rate))
            )
            m.submodules += [uart_rx, uart_tx]
            m.d.comb += [
                uart_rx.i.eq(uart_pins.rx.i),
                uart_pins.tx.o.eq(uart_tx.o),
            ]
            print("USBPassthroughHandler: Hardware UART interface connected.")
        except ResourceError as e:
            print(
                f"WARNING: UART resource not found: {e}. UART command interface disabled."
            )

            # Create dummy UART modules if resource is missing
            class DummyUartRx:
                def __init__(self):
                    self.rdy = Signal()
                    self.data = Signal(8)
                    self.ack = Signal()
                    self.err = Record(
                        [("overrun", 1), ("frame", 1)]
                    )  # Match original fields

            class DummyUartTx:
                def __init__(self):
                    self.data = Signal(8)
                    self.ack = Signal()
                    self.rdy = Signal(reset=1)  # Assume ready if dummy
                    self.o = Signal()  # Dummy output pin

            self._uart_rx = uart_rx = DummyUartRx()
            self._uart_tx = uart_tx = DummyUartTx()
        except Exception as e:
            print(f"ERROR: Unexpected error during UART setup: {e}")
            # Handle unexpected errors gracefully, maybe create dummies
            self._uart_rx = uart_rx = DummyUartRx()  # Use dummy on error
            self._uart_tx = uart_tx = DummyUartTx()

        # --- Command Parser (Handles commands from UART or USB Serial) ---
        # Runs in sync domain
        m.submodules.cmd_parser = self._cmd_parser = cmd_parser = DomainRenamer("sync")(
            MouseCommandParser()
        )

        # --- UART TX Handler (Buffers data for UART TX) ---
        # This is needed whether using UART or USB Serial for outputting debug/responses
        # Runs in sync domain
        m.submodules.uart_tx_handler = uart_tx_handler = DomainRenamer("sync")(
            UARTTXHandler(fifo_depth=32)
        )

        # --- Attempt CONTROL Port USB CDC-ACM Serial Implementation ---
        # Try to use USB Serial on the CONTROL port if available
        control_translator = None
        try:
            # Request control PHY - assumes name 'control_phy'
            control_ulpi_res = platform.request("control_phy", 0)
            # Instantiate translator ONLY for the control port if found
            # Runs in sync domain
            m.submodules.control_translator = control_translator = DomainRenamer(
                "sync"
            )(UTMITranslator(ulpi=control_ulpi_res))
            print("USBPassthroughHandler: Found and initialized CONTROL PHY.")
            self.use_control_port = True
        except ResourceError:
            print(
                "USBPassthroughHandler: CONTROL PHY not found. Falling back to UART for commands."
            )
            self.use_control_port = False
        except Exception as e:
            print(f"ERROR: Unexpected error requesting/init CONTROL PHY: {e}")
            self.use_control_port = False

        if self.use_control_port and control_translator is not None:
            # --- USB Serial Command Interface (Control Port) ---
            try:
                # Create the USB CDC-ACM serial device using the control translator's bus
                # USBSerialDevice likely runs in 'usb' domain internally, but connects to 'sync' translator
                m.submodules.usb_serial = self._usb_serial = usb_serial = (
                    USBSerialDevice(
                        bus=control_translator.ulpi,  # Use the ULPI bus from the control translator
                        idVendor=0x1209,
                        idProduct=0x5BFA,
                        manufacturer_string="Hurricane FPGA",
                        product_string="USB Command Interface",
                        serial_number="HFPGA-001",
                        max_packet_size=64,
                    )
                )
                # Connect the device - needs CDC if connect signal is from sync domain
                # Assuming connect is driven from sync domain for now
                connect_sync = Signal(reset=1)  # Enable by default
                m.submodules.connect_cdc = FFSynchronizer(
                    i=connect_sync,
                    o=usb_serial.connect,
                    i_domain="sync",
                    o_domain="usb",
                )

                # Connect USB serial RX (usb domain) to command parser (sync domain) via CDC FIFO
                m.submodules.cmd_rx_cdc_fifo = cmd_rx_cdc_fifo = DomainRenamer("sync")(
                    AsyncFIFOBuffered(
                        width=8, depth=32, w_domain="usb", r_domain="sync"
                    )
                )
                m.d.comb += [
                    cmd_rx_cdc_fifo.w_en.eq(usb_serial.rx.valid),
                    cmd_rx_cdc_fifo.w_data.eq(usb_serial.rx.payload),
                    usb_serial.rx.ready.eq(
                        cmd_rx_cdc_fifo.w_rdy
                    ),  # Backpressure from FIFO
                ]
                m.d.comb += [
                    cmd_parser.i_uart_stream.valid.eq(cmd_rx_cdc_fifo.r_rdy),
                    cmd_parser.i_uart_stream.payload.eq(cmd_rx_cdc_fifo.r_data),
                    cmd_rx_cdc_fifo.r_en.eq(
                        cmd_parser.i_uart_stream.ready
                    ),  # Backpressure to FIFO
                    self.o_command_rx_activity.eq(
                        cmd_rx_cdc_fifo.r_rdy
                    ),  # Command LED from FIFO read side
                ]

                # Connect the shared TX stream (sync domain) to USB serial TX (usb domain) via CDC FIFO
                m.submodules.cmd_tx_cdc_fifo = cmd_tx_cdc_fifo = DomainRenamer("usb")(
                    AsyncFIFOBuffered(
                        width=8, depth=32, w_domain="sync", r_domain="usb"
                    )
                )
                # Connect input stream to FIFO write side (sync domain)
                m.d.comb += [
                    cmd_tx_cdc_fifo.w_en.eq(self.i_uart_tx_stream.valid),
                    cmd_tx_cdc_fifo.w_data.eq(self.i_uart_tx_stream.payload),
                    self.i_uart_tx_stream.ready.eq(
                        cmd_tx_cdc_fifo.w_rdy
                    ),  # Backpressure from FIFO
                ]
                # Connect FIFO read side to USB serial TX (usb domain)
                m.d.comb += [
                    usb_serial.tx.valid.eq(cmd_tx_cdc_fifo.r_rdy),
                    usb_serial.tx.payload.eq(cmd_tx_cdc_fifo.r_data),
                    # Assuming simple byte stream for USB serial TX, no first/last
                    # usb_serial.tx.first.eq(...),
                    # usb_serial.tx.last.eq(...),
                    cmd_tx_cdc_fifo.r_en.eq(
                        usb_serial.tx.ready
                    ),  # Backpressure to FIFO
                    self.o_uart_tx_activity.eq(
                        cmd_tx_cdc_fifo.r_rdy
                    ),  # TX LED from FIFO read side
                ]

                # Acknowledge hardware UART RX if USB Serial is active (prevent buffer fill)
                m.d.comb += uart_rx.ack.eq(uart_rx.rdy)
                m.d.comb += self.o_uart_rx_activity.eq(
                    uart_rx.rdy
                )  # Show UART RX activity even if ignored

                print(
                    "USBPassthroughHandler: CDC-ACM serial interface configured for CONTROL port."
                )

            except Exception as e:
                print(f"ERROR: USB Serial setup failed: {e}. Falling back to UART.")
                self.use_control_port = False
                self._usb_serial = None  # Ensure usb_serial is None on failure

        # --- Fallback to UART if CONTROL port/USB Serial failed or not available ---
        if not self.use_control_port:
            print("USBPassthroughHandler: Using Hardware UART for command interface.")
            # Connect Hardware UART RX (sync domain) to Parser input stream (sync domain)
            m.d.comb += [
                cmd_parser.i_uart_stream.valid.eq(uart_rx.rdy),
                cmd_parser.i_uart_stream.payload.eq(uart_rx.data),
                uart_rx.ack.eq(cmd_parser.i_uart_stream.ready),  # Parser controls ack
                self.o_command_rx_activity.eq(uart_rx.rdy),  # Command LED from UART
                self.o_uart_rx_activity.eq(uart_rx.rdy),  # UART RX LED
            ]
            # Connect the shared TX stream (sync domain) via the UART TX handler (sync) to hardware UART TX (sync)
            m.d.comb += [
                uart_tx_handler.i_tx_stream.stream_eq(
                    self.i_uart_tx_stream
                ),  # Connect input stream
                uart_tx.data.eq(uart_tx_handler.o_uart_data),
                uart_tx.ack.eq(uart_tx_handler.o_uart_valid),  # TX controls ack
                uart_tx_handler.i_uart_ready.eq(uart_tx.rdy),
                self.o_uart_tx_activity.eq(
                    uart_tx_handler.o_uart_valid
                ),  # TX LED from UART handler
            ]

        # --- Mouse Injector Setup ---
        # Instantiated in USB domain as mouse events are USB protocol level
        # Use DomainRenamer explicitly if needed, assuming SimpleMouseInjector runs in 'usb'
        m.submodules.injector = injector = DomainRenamer("usb")(SimpleMouseInjector())

        # Connect PARSER outputs (sync domain) to the INJECTOR inputs (usb domain) using CDC
        # Simple FFSynchronizer should be sufficient for control signals and data bytes
        m.submodules.trigger_cdc = FFSynchronizer(
            i=cmd_parser.o_cmd_ready,
            o=injector.trigger,
            i_domain="sync",
            o_domain="usb",
        )
        cmd_buttons_usb = Signal(8)
        cmd_dx_usb = Signal(8)
        cmd_dy_usb = Signal(8)
        m.submodules.buttons_cdc = FFSynchronizer(
            i=cmd_parser.o_buttons, o=cmd_buttons_usb, i_domain="sync", o_domain="usb"
        )
        m.submodules.dx_cdc = FFSynchronizer(
            i=cmd_parser.o_dx, o=cmd_dx_usb, i_domain="sync", o_domain="usb"
        )
        m.submodules.dy_cdc = FFSynchronizer(
            i=cmd_parser.o_dy, o=cmd_dy_usb, i_domain="sync", o_domain="usb"
        )

        # Connect synchronized signals to injector
        # Ensure these connections are in the 'usb' domain if injector is usb domain
        with m.Domain("usb"):
            m.d.comb += [
                # injector.trigger connected via trigger_cdc
                injector.buttons.eq(cmd_buttons_usb),
                injector.dx.eq(cmd_dx_usb),
                injector.dy.eq(cmd_dy_usb),
            ]

        # Removed connection of self.i_buttons etc. Assuming parser is the source.

        # --- Removed Data Path Logic ---
        # The complex CDC FIFOs, HyperRAM, Arbiter, and stream routing
        # for Host <-> Device passthrough have been removed.
        # The direct connection is handled in top.py

        # --- Removed Packet Activity Indicators ---
        # o_host_packet_activity and o_dev_packet_activity removed

        # --- Startup Message ---
        # Send a test message on startup via the active command interface's TX path
        startup_message = "Hurricane FPGA Command Interface Ready\r\n"
        startup_msg_rom = Array([ord(c) for c in startup_message])
        msg_counter = Signal(range(len(startup_message) + 1), reset=0)
        msg_sent = Signal(reset=0)

        # Drive the shared i_uart_tx_stream (sync domain)
        with m.Domain("sync"):
            with m.If(~msg_sent):
                # Check if the output interface (FIFO input or UART handler input) is ready
                with m.If(self.i_uart_tx_stream.ready):
                    m.d.comb += [
                        self.i_uart_tx_stream.valid.eq(1),
                        self.i_uart_tx_stream.payload.eq(startup_msg_rom[msg_counter]),
                        # Assuming simple byte stream, no first/last needed for UART/Serial TX?
                        # self.i_uart_tx_stream.first.eq(msg_counter == 0),
                        # self.i_uart_tx_stream.last.eq(msg_counter == len(startup_message) - 1),
                    ]
                    # Increment counter or mark as sent
                    with m.If(msg_counter == len(startup_message) - 1):
                        m.d.sync += msg_sent.eq(1)
                    with m.Else():
                        m.d.sync += msg_counter.eq(msg_counter + 1)
                with m.Else():
                    # Ensure valid is low if not ready
                    m.d.comb += self.i_uart_tx_stream.valid.eq(0)
            with m.Else():
                # Ensure valid is low after message sent
                m.d.comb += self.i_uart_tx_stream.valid.eq(0)

        return m
