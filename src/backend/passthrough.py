#!/usr/bin/env python3

# -*- coding: utf-8 -*-

from amaranth import Elaboratable, Module, Signal, DomainRenamer, Cat, C, Record, Array
from amaranth.build.res import ResourceError
from amaranth.lib.fifo import SyncFIFOBuffered, AsyncFIFOBuffered
from luna.gateware.interface.ulpi import UTMITranslator
from amaranth.lib.cdc import FFSynchronizer
from luna.gateware.interface.psram import HyperRAMInterface, HyperRAMPHY
from .mouse_injector import SimpleMouseInjector, MouseCommandParser
from .fifo import HyperRAMPacketFIFO
from .utils import AsyncSerialRX, AsyncSerialTX
from .uart import UARTTXHandler
from .usb_serial import USBSerialDevice

# Import the interfaces from the shared module
from .interfaces import (
    StreamInterface,
    USBInStreamInterface,
    USBOutStreamInterface,
)

__all__ = [
    "USBDataPassthroughHandler",
    "PacketArbiter",
    "PHYTranslatorHandler",
    "StreamAdapter",
]


class PacketArbiter(Elaboratable):
    def __init__(self):
        self.passthrough_in = USBInStreamInterface()
        self.inject_in = USBInStreamInterface()
        self.merged_out = USBInStreamInterface()

    def elaborate(self, platform):
        m = Module()
        passthrough, inject, merged = (
            self.passthrough_in,
            self.inject_in,
            self.merged_out,
        )
        # Ensure ready is propagated correctly in stream_eq
        m.d.comb += [
            merged.valid.eq(0),
            merged.payload.eq(0),
            merged.first.eq(0),
            merged.last.eq(0),
            passthrough.ready.eq(0),
            inject.ready.eq(0),
        ]
        with m.FSM(domain="usb", name="arbiter_fsm"):
            with m.State("IDLE"):
                m.d.comb += passthrough.ready.eq(0)  # Explicitly deassert ready
                m.d.comb += inject.ready.eq(0)
                with m.If(inject.valid):
                    m.next = "FORWARD_INJECT"
                with m.Elif(passthrough.valid):
                    m.next = "FORWARD_PASSTHROUGH"
            with m.State("FORWARD_PASSTHROUGH"):
                # Use stream_eq which now includes ready
                m.d.comb += passthrough.stream_eq(merged)
                m.d.comb += inject.ready.eq(0)  # Don't accept inject data now
                # State transition logic
                with m.If(merged.valid & merged.last & merged.ready):
                    m.next = "IDLE"
                with m.Elif(
                    ~merged.valid
                ):  # If source becomes invalid mid-packet (shouldn't happen ideally)
                    m.next = "IDLE"

            with m.State("FORWARD_INJECT"):
                # Use stream_eq which now includes ready
                m.d.comb += inject.stream_eq(merged)
                m.d.comb += passthrough.ready.eq(0)  # Don't accept passthrough data now
                # State transition logic
                with m.If(merged.valid & merged.last & merged.ready):
                    m.next = "IDLE"
                with m.Elif(~merged.valid):  # If source becomes invalid mid-packet
                    m.next = "IDLE"
        return m


class PHYTranslatorHandler(Elaboratable):
    """Handles USB PHY setup, initialization and configuration.
    This class manages the UTMI translators and VBUS control for both host and device connections.
    """

    def __init__(self, platform=None):
        self.platform = platform

        # Outputs - UTMI translators
        self.host_translator = None  # AUX port (PC connection)
        self.dev_translator = None  # TARGET port (Mouse connection)
        self.control_translator = None  # CONTROL port for commands

        # Status
        self.simulation_mode = platform is None

    def elaborate(self, platform):
        m = Module()

        if self.simulation_mode:
            # Dummy setup for simulation mode
            ulpi_bus_layout = [
                ("data", 8),
                ("clk", 1),
                ("dir", 1),
                ("nxt", 1),
                ("stp", 1),
                ("rst", 1),
            ]
            # Simulation port mappings
            target_ulpi_res = Record(
                [("i", ulpi_bus_layout), ("o", ulpi_bus_layout)]
            )  # TARGET port (Mouse)
            aux_ulpi_res = Record(
                [("i", ulpi_bus_layout), ("o", ulpi_bus_layout)]
            )  # AUX port (PC)
            # Control for command interface
            control_ulpi_res = Record([("i", ulpi_bus_layout), ("o", ulpi_bus_layout)])

            # Instantiate translators with dummy resources
            m.submodules.host_translator = self.host_translator = DomainRenamer("sync")(
                UTMITranslator(ulpi=aux_ulpi_res)  # AUX port (PC connection)
            )
            m.submodules.dev_translator = self.dev_translator = DomainRenamer("sync")(
                UTMITranslator(ulpi=target_ulpi_res)  # TARGET port (Mouse connection)
            )
            m.submodules.control_translator = self.control_translator = DomainRenamer(
                "sync"
            )(UTMITranslator(ulpi=control_ulpi_res))

            # Dummy VBUS enable signals
            control_vbus_en = Record([("o", 1)])
            aux_vbus_en = Record([("o", 1)])
        else:
            # Request actual PHY resources based on platform version
            try:
                # Determine the correct PHY names based on platform version
                print("DEBUG: Checking platform version")
                if hasattr(platform, "version"):
                    print(f"DEBUG: Platform version detected: {platform.version}")
                else:
                    print("DEBUG: Platform has no version attribute")

                aux_phy_name = (
                    "aux_phy"
                    if hasattr(platform, "version") and platform.version >= (0, 6)
                    else "host_phy"
                )
                control_phy_name = (
                    "control_phy"
                    if hasattr(platform, "version") and platform.version >= (0, 6)
                    else "sideband_phy"
                )
                target_phy_name = "target_phy"  # Always the same

                print(
                    f"DEBUG: Using PHY names: aux={aux_phy_name}, target={target_phy_name}, control={control_phy_name}"
                )

                # Get a list of available resources for debugging
                print("DEBUG: Available resources:")
                try:
                    available_resources = []
                    for resource_name in platform.resources:
                        available_resources.append(resource_name)
                    print(f"DEBUG: {available_resources}")
                except Exception as e:
                    print(f"DEBUG: Error getting resource list: {e}")

                # Try both aux/host and target PHYs, with fallbacks
                try:
                    print(f"DEBUG: Requesting {aux_phy_name}")
                    aux_ulpi_res = platform.request(aux_phy_name, 0)
                    print(f"DEBUG: Successfully got {aux_phy_name}")

                    print(f"DEBUG: Creating host_translator for {aux_phy_name}")
                    m.submodules.host_translator = self.host_translator = DomainRenamer(
                        "sync"
                    )(
                        UTMITranslator(ulpi=aux_ulpi_res)
                    )  # PC connection
                except ResourceError as e:
                    print(f"DEBUG: Error requesting {aux_phy_name}: {e}")
                    # Try alternate name if primary failed
                    alt_aux_name = (
                        "host_phy" if aux_phy_name == "aux_phy" else "aux_phy"
                    )
                    try:
                        print(f"DEBUG: Trying alternate name {alt_aux_name}")
                        aux_ulpi_res = platform.request(alt_aux_name, 0)
                        print(f"DEBUG: Successfully got {alt_aux_name}")

                        print(f"DEBUG: Creating host_translator for {alt_aux_name}")
                        m.submodules.host_translator = (
                            self.host_translator
                        ) = DomainRenamer("sync")(
                            UTMITranslator(ulpi=aux_ulpi_res)
                        )  # PC connection
                    except ResourceError as e2:
                        print(
                            f"DEBUG: Error requesting alternate name {alt_aux_name}: {e2}"
                        )
                        print(
                            "DEBUG: AUX/HOST PHY unavailable - using dummy translator"
                        )
                        self.host_translator = None

                try:
                    print(f"DEBUG: Requesting {target_phy_name}")
                    target_ulpi_res = platform.request(target_phy_name, 0)
                    print(f"DEBUG: Successfully got {target_phy_name}")

                    print(f"DEBUG: Creating dev_translator for {target_phy_name}")
                    m.submodules.dev_translator = self.dev_translator = DomainRenamer(
                        "sync"
                    )(
                        UTMITranslator(ulpi=target_ulpi_res)
                    )  # Mouse connection
                except ResourceError as e:
                    print(f"DEBUG: Error requesting {target_phy_name}: {e}")
                    print("DEBUG: TARGET PHY unavailable - using dummy translator")
                    self.dev_translator = None

                # Try to request Control PHY if available
                try:
                    control_ulpi_res = platform.request(control_phy_name, 0)
                    m.submodules.control_translator = self.control_translator = (
                        DomainRenamer("sync")(UTMITranslator(ulpi=control_ulpi_res))
                    )
                    print(
                        f"USBPassthrough: Found {control_phy_name} resource. Command interface will use this."
                    )
                except ResourceError:
                    self.control_translator = None
                    print(
                        f"USBPassthrough: {control_phy_name} resource not found. Command interface will use UART."
                    )

                # VBUS input enable signals (Active High)
                try:
                    aux_vbus_name = (
                        "aux_vbus_in_en"
                        if hasattr(platform, "version") and platform.version >= (0, 6)
                        else "host_vbus_in_en"
                    )
                    control_vbus_name = (
                        "control_vbus_in_en"
                        if hasattr(platform, "version") and platform.version >= (0, 6)
                        else "sideband_vbus_in_en"
                    )
                    target_vbus_name = (
                        "target_c_vbus_en"
                        if hasattr(platform, "version") and platform.version >= (0, 6)
                        else "target_vbus_in_en"
                    )

                    try:
                        control_vbus_en = platform.request(control_vbus_name, 0)
                        m.d.comb += control_vbus_en.o.eq(1)  # Enable Control VBUS input
                        print(f"USBPassthrough: Enabled {control_vbus_name}.")
                    except ResourceError:
                        print(f"USBPassthrough: {control_vbus_name} enable not found.")

                    try:
                        aux_vbus_en = platform.request(aux_vbus_name, 0)
                        m.d.comb += aux_vbus_en.o.eq(1)  # Enable Aux VBUS input
                        print(f"USBPassthrough: Enabled {aux_vbus_name}.")
                    except ResourceError:
                        print(f"USBPassthrough: {aux_vbus_name} enable not found.")

                    try:
                        # For Target Port A, make sure we enable VBUS passthrough from Target-C
                        target_vbus_en = platform.request(target_vbus_name, 0)
                        m.d.comb += target_vbus_en.o.eq(1)  # Enable Target VBUS input
                        print(
                            f"USBPassthrough: Enabled {target_vbus_name} for Target Port A."
                        )

                        # Enable Target A discharge control (set to 0 to disable discharge)
                        try:
                            target_discharge = platform.request("target_a_discharge", 0)
                            m.d.comb += target_discharge.o.eq(
                                0
                            )  # Disable discharge to allow power
                            print("USBPassthrough: Disabled Target A discharge.")
                        except ResourceError:
                            print(
                                "USBPassthrough: target_a_discharge control not found."
                            )
                    except ResourceError:
                        print(f"USBPassthrough: {target_vbus_name} enable not found.")
                        print("WARNING: Target Port A may not receive power correctly!")

                except Exception as e:
                    print(f"USBPassthrough: Error configuring VBUS: {e}")

                # After all resource setup, check and report translator status
                if self.host_translator is not None:
                    print("DEBUG: host_translator successfully initialized")
                else:
                    print("DEBUG: host_translator is None")

                if self.dev_translator is not None:
                    print("DEBUG: dev_translator successfully initialized")
                else:
                    print("DEBUG: dev_translator is None")

            except Exception as e:
                print(f"DEBUG: Unexpected error in PHY setup: {e}")
                # Both translators will be None, triggering dummy mode in parent module

        # Configure both PHYs with standard settings
        if self.host_translator is not None and self.dev_translator is not None:
            m.d.comb += [
                self.host_translator.op_mode.eq(0b00),
                self.host_translator.xcvr_select.eq(0b10),  # HS mode
                self.host_translator.term_select.eq(0),  # Disable termination
                self.host_translator.suspend.eq(0),
            ]
            m.d.comb += [
                self.dev_translator.op_mode.eq(0b00),
                self.dev_translator.xcvr_select.eq(0b10),  # HS mode
                self.dev_translator.term_select.eq(0),  # Disable termination
                self.dev_translator.suspend.eq(0),
            ]

        return m


class USBDataPassthroughHandler(Elaboratable):
    DEFAULT_UART_BAUD = 115200

    def __init__(self, uart_baud_rate=None):
        self.uart_baud_rate = (
            uart_baud_rate if uart_baud_rate else self.DEFAULT_UART_BAUD
        )

        # Outputs for status LEDs
        self.o_host_packet_activity = Signal()  # Activity on AUX port (to/from PC)
        self.o_dev_packet_activity = Signal()  # Activity on TARGET port (to/from mouse)
        self.o_uart_rx_activity = Signal()  # Activity on UART RX
        self.o_uart_tx_activity = Signal()  # Activity on UART TX
        self.o_command_rx_activity = (
            Signal()
        )  # Activity on command interface (UART or CONTROL)

        # Stream interface for sending data out via UART TX
        self.i_uart_tx_stream = StreamInterface(payload_width=8)

        # Signals for injector (initially defined in elaborate, but safer to define here)
        self.i_inject_trigger = Signal()
        self.i_buttons = Signal(8)
        self.i_dx = Signal(8)
        self.i_dy = Signal(8)

        # Internal references to submodules (populated during elaborate)
        self._cmd_parser = None
        self.use_control_port = (
            False  # Will be set during elaborate if CONTROL PHY is available
        )

    def get_cmd_parser(self):
        """Returns the command parser submodule instance."""
        return self._cmd_parser

    def elaborate(self, platform):
        m = Module()

        # --- Get Clock Frequency ---
        try:
            clk_freq = platform.lookup("clk_60MHz").clock.frequency
        except (ResourceError, AttributeError):
            # Assume 60MHz if platform doesn't define it (e.g., simulation)
            clk_freq = 60e6
            print(
                "WARNING: Platform does not define clk_60MHz frequency, assuming 60MHz."
            )

        # --- PHY Setup and Power Control ---
        m.submodules.phy_handler = phy_handler = PHYTranslatorHandler(platform=platform)
        host_translator = phy_handler.host_translator
        dev_translator = phy_handler.dev_translator

        # Check if translators are properly initialized
        if host_translator is None or dev_translator is None:
            print(
                "WARNING: One or more USB translators are not initialized yet. Using fallback initialization..."
            )
            try:
                # Try to extract the submodules directly from the PHYTranslatorHandler
                for name, sub in phy_handler._submodules.items():
                    if name == "host_translator" and host_translator is None:
                        host_translator = sub
                        print(
                            "Retrieved host_translator directly from phy_handler submodules"
                        )
                    if name == "dev_translator" and dev_translator is None:
                        dev_translator = sub
                        print(
                            "Retrieved dev_translator directly from phy_handler submodules"
                        )

                # Configure translators if they were successfully retrieved
                if host_translator is not None:
                    m.d.comb += [
                        host_translator.op_mode.eq(0b00),
                        host_translator.xcvr_select.eq(0b10),  # HS mode
                        host_translator.term_select.eq(0),  # Disable termination
                        host_translator.suspend.eq(0),
                    ]
                    print("Configured retrieved host_translator")

                if dev_translator is not None:
                    m.d.comb += [
                        dev_translator.op_mode.eq(0b00),
                        dev_translator.xcvr_select.eq(0b10),  # HS mode
                        dev_translator.term_select.eq(0),  # Disable termination
                        dev_translator.suspend.eq(0),
                    ]
                    print("Configured retrieved dev_translator")
            except Exception as e:
                print(
                    f"WARNING: Could not retrieve translators from PHYTranslatorHandler: {e}"
                )
            print("NOTE: Using dummy operations for any missing translators")

        # --- FIFOs ---
        cdc_fifo_width = 10  # 8 data + first + last
        cdc_fifo_depth = 32  # Depth for domain crossing FIFOs

        # RX path FIFOs (sync -> usb)
        # Host (PC) data comes from AUX PHY
        m.submodules.host_rx_fifo = host_rx_fifo = AsyncFIFOBuffered(
            width=cdc_fifo_width, depth=cdc_fifo_depth, w_domain="sync", r_domain="usb"
        )
        # Device (Mouse) data comes from TARGET PHY
        m.submodules.dev_rx_fifo = dev_rx_fifo = AsyncFIFOBuffered(
            width=cdc_fifo_width, depth=cdc_fifo_depth, w_domain="sync", r_domain="usb"
        )

        # TX path CDC FIFOs (usb -> sync)
        # Host data goes to AUX PHY (PC connection - Port C)
        m.submodules.host_tx_cdc = host_tx_cdc = AsyncFIFOBuffered(
            width=cdc_fifo_width, depth=cdc_fifo_depth, w_domain="usb", r_domain="sync"
        )
        # Device data goes to TARGET PHY (mouse connection - Port A)
        m.submodules.dev_tx_cdc = dev_tx_cdc = AsyncFIFOBuffered(
            width=cdc_fifo_width, depth=cdc_fifo_depth, w_domain="usb", r_domain="sync"
        )

        # --- TX Path CDC FIFOs -> HyperRAM/Buffer -> PHYs (All 'sync' domain from here) ---
        host_tx_cdc_out_stream = StreamInterface(
            payload_width=cdc_fifo_width
        )  # Data going TO host (AUX)
        dev_tx_cdc_out_stream = StreamInterface(
            payload_width=cdc_fifo_width
        )  # Data going TO device (TARGET)

        m.d.comb += [
            host_tx_cdc_out_stream.valid.eq(host_tx_cdc.r_rdy),
            host_tx_cdc.r_en.eq(host_tx_cdc_out_stream.ready),
            host_tx_cdc_out_stream.payload.eq(host_tx_cdc.r_data),
        ]
        m.d.comb += [
            dev_tx_cdc_out_stream.valid.eq(dev_tx_cdc.r_rdy),
            dev_tx_cdc.r_en.eq(dev_tx_cdc_out_stream.ready),
            dev_tx_cdc_out_stream.payload.eq(dev_tx_cdc.r_data),
        ]

        # HyperRAM FIFOs / Fallback Buffers (sync domain bulk storage)
        use_hyperram = False
        ram_bus = None
        psram_phy = None
        psram = None

        try:
            # Request RAM resource once to share between both FIFOs
            ram_bus = platform.request("ram")
            use_hyperram = True
            print(
                "USBPassthroughAnalyzer: Found 'ram' resource. HyperRAM FIFOs will be used."
            )

            # Create a single shared PSRAM controller for both FIFOs
            psram_phy = HyperRAMPHY(bus=ram_bus)
            psram = HyperRAMInterface(phy=psram_phy.phy)
            m.submodules += [psram_phy, psram]

        except ResourceError as e:
            print(
                f"USBPassthroughAnalyzer: ResourceError ('{e}'). HyperRAM FIFOs disabled (using fallback BRAM buffers)."
            )
            use_hyperram = False  # Disable if RAM is missing

        if use_hyperram:
            hyperram_out_buf_depth = 32  # Define buffer depth for HyperRAM FIFOs

            # --- HyperRAM Path ---
            host_hyperram_in_stream = StreamInterface(
                payload_width=16
            )  # Input to host FIFO (to AUX)
            dev_hyperram_in_stream = StreamInterface(
                payload_width=16
            )  # Input to device FIFO (to TARGET)
            m.submodules.host_buffer_fifo = host_buffer_fifo = SyncFIFOBuffered(
                width=16, depth=hyperram_out_buf_depth
            )
            m.submodules.dev_buffer_fifo = dev_buffer_fifo = SyncFIFOBuffered(
                width=16, depth=hyperram_out_buf_depth
            )

            # Connect CDC outputs to buffer FIFOs
            m.d.comb += [
                # Host buffer connections
                host_buffer_fifo.w_data.eq(
                    Cat(host_tx_cdc_out_stream.payload[0:8], C(0, 8))
                ),
                host_buffer_fifo.w_en.eq(host_tx_cdc_out_stream.valid),
                host_tx_cdc_out_stream.ready.eq(host_buffer_fifo.w_rdy),
                # Device buffer connections
                dev_buffer_fifo.w_data.eq(
                    Cat(dev_tx_cdc_out_stream.payload[0:8], C(0, 8))
                ),
                dev_buffer_fifo.w_en.eq(dev_tx_cdc_out_stream.valid),
                dev_tx_cdc_out_stream.ready.eq(dev_buffer_fifo.w_rdy),
            ]

            # Create a single HyperRAM FIFO that will be used by both paths
            m.submodules.shared_hyperram_fifo = shared_hyperram_fifo = (
                HyperRAMPacketFIFO(
                    out_fifo_depth=hyperram_out_buf_depth
                    * 2,  # Larger to accommodate both streams
                    ram_bus=ram_bus,
                    shared_psram=psram,
                )
            )

            # Arbitration between host and device paths
            # Priority to device path (TARGET) as it's more time-sensitive
            shared_input = StreamInterface(payload_width=16)

            arbiter_state = Signal()  # 0 = idle/device, 1 = host
            m.d.sync += [
                # Default: reset to idle/device priority
                arbiter_state.eq(0)
            ]

            # Arbitration logic
            with m.If(arbiter_state == 0):
                # Device has priority
                with m.If(dev_buffer_fifo.r_rdy):
                    # If device has data, use it
                    m.d.comb += [
                        shared_input.valid.eq(dev_buffer_fifo.r_rdy),
                        shared_input.payload.eq(dev_buffer_fifo.r_data),
                        # First/last flags not used in this simplified arbitration
                        shared_input.first.eq(0),
                        shared_input.last.eq(0),
                        dev_buffer_fifo.r_en.eq(shared_input.ready),
                    ]
                with m.Elif(host_buffer_fifo.r_rdy):
                    # If device has no data but host does, switch to host
                    m.d.sync += arbiter_state.eq(1)
            with m.Else():
                # Host has priority
                with m.If(host_buffer_fifo.r_rdy):
                    # If host has data, use it
                    m.d.comb += [
                        shared_input.valid.eq(host_buffer_fifo.r_rdy),
                        shared_input.payload.eq(host_buffer_fifo.r_data),
                        # First/last flags not used in this simplified arbitration
                        shared_input.first.eq(0),
                        shared_input.last.eq(0),
                        host_buffer_fifo.r_en.eq(shared_input.ready),
                    ]
                with m.Elif(dev_buffer_fifo.r_rdy):
                    # If host has no data but device does, switch back to device
                    m.d.sync += arbiter_state.eq(0)

            # Connect the arbitrated input to the shared HyperRAM FIFO
            m.d.comb += [
                shared_hyperram_fifo.input.valid.eq(shared_input.valid),
                shared_hyperram_fifo.input.payload.eq(shared_input.payload),
                shared_hyperram_fifo.input.first.eq(shared_input.first),
                shared_hyperram_fifo.input.last.eq(shared_input.last),
                shared_input.ready.eq(shared_hyperram_fifo.input.ready),
            ]

            # Output path logic - need to demux the shared output
            # This is not bandwidth-efficient but ensures no driver conflicts

            # Only proceed if both translators are initialized
            if host_translator is not None and dev_translator is not None:
                # Host Path (To AUX PHY - Port C)
                m.d.comb += [
                    host_translator.tx_data.eq(
                        shared_hyperram_fifo.output.payload[0:8]
                    ),
                    host_translator.tx_valid.eq(shared_hyperram_fifo.output.valid),
                ]

                # Device Path (To TARGET PHY - Port A)
                m.d.comb += [
                    dev_translator.tx_data.eq(shared_hyperram_fifo.output.payload[0:8]),
                    dev_translator.tx_valid.eq(shared_hyperram_fifo.output.valid),
                ]

                # Only dequeue from the HyperRAM FIFO when both PHYs are ready
                m.d.comb += [
                    shared_hyperram_fifo.output.ready.eq(
                        host_translator.tx_ready & dev_translator.tx_ready
                    ),
                ]
            else:
                # Fallback when translators aren't available
                print(
                    "WARNING: HyperRAM path cannot be connected because translators aren't initialized"
                )
                m.d.comb += [
                    shared_hyperram_fifo.output.ready.eq(
                        0
                    ),  # Never ready to receive data
                ]

        else:
            # --- Non-HyperRAM Fallback Path (CDC -> Sync Buffer -> PHY TX) ---
            sync_buffer_depth = 64  # Fallback BRAM depth
            # Host path buffer (to/from TARGET)
            m.submodules.host_sync_buffer = host_sync_buffer = SyncFIFOBuffered(
                width=8, depth=sync_buffer_depth
            )
            # Device path buffer (to/from AUX)
            m.submodules.dev_sync_buffer = dev_sync_buffer = SyncFIFOBuffered(
                width=8, depth=sync_buffer_depth
            )

            # Host Path (To AUX PHY)
            m.d.comb += [
                host_sync_buffer.w_en.eq(host_tx_cdc_out_stream.valid),
                host_sync_buffer.w_data.eq(host_tx_cdc_out_stream.payload[0:8]),
                host_tx_cdc_out_stream.ready.eq(host_sync_buffer.w_rdy),
            ]
            # Device Path (To TARGET PHY)
            m.d.comb += [
                dev_sync_buffer.w_en.eq(dev_tx_cdc_out_stream.valid),
                dev_sync_buffer.w_data.eq(dev_tx_cdc_out_stream.payload[0:8]),
                dev_tx_cdc_out_stream.ready.eq(dev_sync_buffer.w_rdy),
            ]

            # Connect Sync Buffer Output to PHY TX
            # Host Path (To AUX PHY)
            m.d.comb += [
                host_translator.tx_data.eq(host_sync_buffer.r_data),
                host_translator.tx_valid.eq(host_sync_buffer.r_rdy),
                host_sync_buffer.r_en.eq(host_translator.tx_ready),
            ]
            # Device Path (To TARGET PHY)
            m.d.comb += [
                dev_translator.tx_data.eq(dev_sync_buffer.r_data),
                dev_translator.tx_valid.eq(dev_sync_buffer.r_rdy),
                dev_sync_buffer.r_en.eq(dev_translator.tx_ready),
            ]

        # --- Host RX Path (AUX PHY -> CDC -> USB Domain Logic) ---
        if host_translator is not None:
            host_rx_first = Signal()
            host_rx_last = Signal()
            host_prev_rx_active = Signal(reset_less=True)
            m.d.sync += host_prev_rx_active.eq(host_translator.rx_active)
            m.d.comb += host_rx_first.eq(
                host_translator.rx_valid
                & (host_translator.rx_active & ~host_prev_rx_active)
            )
            host_rx_last_comb = Signal()
            m.d.comb += host_rx_last_comb.eq(
                ~host_translator.rx_active & host_prev_rx_active
            )
            m.d.sync += host_rx_last.eq(
                host_rx_last_comb & host_translator.rx_valid
            )  # Align with valid data

            m.d.comb += [
                host_rx_fifo.w_en.eq(host_translator.rx_valid),
                host_rx_fifo.w_data.eq(
                    Cat(host_translator.rx_data, host_rx_first, host_rx_last_comb)
                ),
            ]
        else:
            # If host_translator is None, make sure the FIFO isn't expecting data
            m.d.comb += [
                host_rx_fifo.w_en.eq(0),
                host_rx_fifo.w_data.eq(0),
            ]

        # --- Device RX Path (TARGET PHY -> CDC -> USB Domain Logic) ---
        if dev_translator is not None:
            dev_rx_first = Signal()
            dev_rx_last = Signal()
            dev_prev_rx_active = Signal(reset_less=True)
            m.d.sync += dev_prev_rx_active.eq(dev_translator.rx_active)
            m.d.comb += dev_rx_first.eq(
                dev_translator.rx_valid
                & (dev_translator.rx_active & ~dev_prev_rx_active)
            )
            dev_rx_last_comb = Signal()
            m.d.comb += dev_rx_last_comb.eq(
                ~dev_translator.rx_active & dev_prev_rx_active
            )
            m.d.sync += dev_rx_last.eq(
                dev_rx_last_comb & dev_translator.rx_valid
            )  # Align with valid data

            m.d.comb += [
                dev_rx_fifo.w_en.eq(dev_translator.rx_valid),
                dev_rx_fifo.w_data.eq(
                    Cat(dev_translator.rx_data, dev_rx_first, dev_rx_last_comb)
                ),
            ]
        else:
            # If dev_translator is None, make sure the FIFO isn't expecting data
            m.d.comb += [
                dev_rx_fifo.w_en.eq(0),
                dev_rx_fifo.w_data.eq(0),
            ]

        # --- Stream Interfaces from RX FIFOs (Output in 'usb' domain) ---
        host_rx_stream_raw = StreamInterface(
            payload_width=cdc_fifo_width
        )  # From AUX PHY
        dev_rx_stream_raw = StreamInterface(
            payload_width=cdc_fifo_width
        )  # From TARGET PGY

        m.d.comb += [
            host_rx_stream_raw.valid.eq(host_rx_fifo.r_rdy),
            host_rx_fifo.r_en.eq(host_rx_stream_raw.ready),
            host_rx_stream_raw.payload.eq(host_rx_fifo.r_data),
        ]
        m.d.comb += [
            dev_rx_stream_raw.valid.eq(dev_rx_fifo.r_rdy),
            dev_rx_fifo.r_en.eq(dev_rx_stream_raw.ready),
            dev_rx_stream_raw.payload.eq(dev_rx_fifo.r_data),
        ]

        # --- Check if Control PHY is available for commands ---
        control_translator = phy_handler.control_translator
        self.use_control_port = control_translator is not None

        # --- UART Setup (Hardware Interface) ---
        uart_divisor = int(clk_freq // self.uart_baud_rate)

        # Get the UART resource once and share it between RX and TX
        try:
            uart_resource = platform.request("uart", 0)

            # RX path: Hardware pins -> AsyncSerialRX
            m.submodules.uart_rx = uart_rx = AsyncSerialRX(
                divisor=uart_divisor, pins=uart_resource
            )

            # TX path: AsyncSerialTX -> Hardware pins
            m.submodules.uart_tx = uart_tx = AsyncSerialTX(
                divisor=uart_divisor, pins=uart_resource
            )
        except Exception as e:
            print(f"WARNING: UART setup error: {e}. Creating dummy UART interfaces.")
            # Create dummy UART interfaces if the resource can't be requested

            # Dummy RX module
            class DummyUartRx:
                def __init__(self):
                    self.rdy = Signal(reset=0)  # No data ready by default
                    self.data = Signal(8)
                    self.ack = Signal()

            # Dummy TX module
            class DummyUartTx:
                def __init__(self):
                    self.rdy = Signal(reset=1)  # Always ready to send
                    self.data = Signal(8)
                    self.ack = Signal()

            uart_rx = DummyUartRx()
            uart_tx = DummyUartTx()
            # Add as normal signals, not submodules
            m.d.comb += uart_rx.ack.eq(0)  # Never acknowledge (dummy mode)

        # --- Instantiate the Command Parser ---
        m.submodules.cmd_parser = cmd_parser = MouseCommandParser(
            input_fifo_depth=4
        )  # Use smaller FIFO maybe
        self._cmd_parser = cmd_parser

        # --- UART TX handler ---
        # Create a UART TX handler to manage outgoing debug messages
        m.submodules.uart_tx_handler = uart_tx_handler = UARTTXHandler(fifo_depth=32)

        # --- CONTROL Port USB CDC-ACM Serial Implementation ---
        if self.use_control_port:
            try:
                # Get raw USB bus directly from control PHY resource
                control_bus = control_translator.ulpi

                # Create the USB CDC-ACM serial device
                m.submodules.usb_serial = usb_serial = USBSerialDevice(
                    bus=control_bus,
                    idVendor=0x1209,  # pid.codes open source VID
                    idProduct=0x5BFA,  # Unique PID for Hurricane FPGA
                    manufacturer_string="Hurricane FPGA",
                    product_string="USB Command Interface",
                    serial_number="HFPGA-001",
                    max_packet_size=64,
                )

                # Enable the USB device
                m.d.comb += usb_serial.connect.eq(1)

                # Connect serial RX to command parser
                m.d.comb += [
                    # Connect USB serial RX to command parser
                    cmd_parser.i_uart_stream.valid.eq(usb_serial.rx.valid),
                    cmd_parser.i_uart_stream.payload.eq(usb_serial.rx.payload),
                    usb_serial.rx.ready.eq(cmd_parser.i_uart_stream.ready),
                    # Command activity LED
                    self.o_command_rx_activity.eq(usb_serial.rx.valid),
                ]

                # Connect debug output to USB serial TX
                m.d.comb += [
                    usb_serial.tx.valid.eq(self.i_uart_tx_stream.valid),
                    usb_serial.tx.payload.eq(self.i_uart_tx_stream.payload),
                    usb_serial.tx.first.eq(self.i_uart_tx_stream.first),
                    usb_serial.tx.last.eq(self.i_uart_tx_stream.last),
                    self.i_uart_tx_stream.ready.eq(usb_serial.tx.ready),
                ]

                # Activity indicator for debug output
                m.d.comb += self.o_uart_tx_activity.eq(usb_serial.tx.valid)

                # UART is still available for legacy/debug purposes
                m.d.comb += uart_rx.ack.eq(uart_rx.rdy)  # Just acknowledge UART data
                m.d.comb += self.o_uart_rx_activity.eq(uart_rx.rdy)

                print(
                    "USBPassthrough: CDC-ACM serial interface successfully configured for CONTROL port."
                )

            except Exception as e:
                print(f"WARNING: USB Serial setup error: {e}. Falling back to UART.")
                # Use UART as fallback
                self.use_control_port = False

        # --- Fallback to UART if CONTROL port is not available or setup failed ---
        if not self.use_control_port:
            # Connect Hardware UART for command input/output
            m.d.comb += [
                # Connect Hardware UART RX to Parser input stream
                cmd_parser.i_uart_stream.valid.eq(uart_rx.rdy),
                cmd_parser.i_uart_stream.payload.eq(uart_rx.data),
                uart_rx.ack.eq(cmd_parser.i_uart_stream.ready),
                # Connect UART TX handler to hardware UART TX
                uart_tx.data.eq(uart_tx_handler.o_uart_data),
                uart_tx.ack.eq(uart_tx_handler.o_uart_valid),
                uart_tx_handler.i_uart_ready.eq(uart_tx.rdy),
                # Connect the external UART TX stream interface to our handler
                self.i_uart_tx_stream.stream_eq(uart_tx_handler.i_tx_stream),
                # Set activity LEDs for UART mode
                self.o_command_rx_activity.eq(uart_rx.rdy),
                self.o_uart_rx_activity.eq(uart_rx.rdy),
                self.o_uart_tx_activity.eq(uart_tx_handler.o_uart_valid),
            ]

        # --- Data Path Logic (USB domain) ---
        m.submodules.injector = injector = SimpleMouseInjector()
        m.submodules.arbiter = arbiter = PacketArbiter()

        # Connect PARSER outputs to the INJECTOR inputs
        # Need CDC for cmd_parser outputs (sync) to injector inputs (usb)
        # Option 1: Simple FFSynchronizer for trigger
        m.submodules.trigger_cdc = FFSynchronizer(
            i=cmd_parser.o_cmd_ready, o=injector.trigger, o_domain="usb"
        )

        # Option 2: Buffer parsed commands via another Async FIFO if backpressure needed
        cmd_buttons_usb = Signal(8)
        cmd_dx_usb = Signal(8)
        cmd_dy_usb = Signal(8)
        m.submodules.buttons_cdc = FFSynchronizer(
            i=cmd_parser.o_buttons, o=cmd_buttons_usb, o_domain="usb"
        )
        m.submodules.dx_cdc = FFSynchronizer(
            i=cmd_parser.o_dx, o=cmd_dx_usb, o_domain="usb"
        )
        m.submodules.dy_cdc = FFSynchronizer(
            i=cmd_parser.o_dy, o=cmd_dy_usb, o_domain="usb"
        )

        m.d.comb += [
            # injector.trigger is connected via trigger_cdc above
            injector.buttons.eq(cmd_buttons_usb),
            injector.dx.eq(cmd_dx_usb),
            injector.dy.eq(cmd_dy_usb),
        ]

        # Convert 10-bit streams to standard 8-bit payload streams
        # Data stream coming FROM the host (PC via TARGET PHY)
        host_logic_in_stream = USBOutStreamInterface()  # Terminology: OUT of PC
        # Data stream coming FROM the device (Mouse via AUX PHY)
        dev_logic_in_stream = USBInStreamInterface()  # Terminology: IN to PC

        m.d.comb += [
            host_logic_in_stream.valid.eq(host_rx_stream_raw.valid),
            host_logic_in_stream.payload.eq(host_rx_stream_raw.payload[0:8]),
            host_logic_in_stream.first.eq(host_rx_stream_raw.payload[8]),
            host_logic_in_stream.last.eq(host_rx_stream_raw.payload[9]),
            host_rx_stream_raw.ready.eq(host_logic_in_stream.ready),
        ]
        m.d.comb += [
            dev_logic_in_stream.valid.eq(dev_rx_stream_raw.valid),
            dev_logic_in_stream.payload.eq(dev_rx_stream_raw.payload[0:8]),
            dev_logic_in_stream.first.eq(dev_rx_stream_raw.payload[8]),
            dev_logic_in_stream.last.eq(dev_rx_stream_raw.payload[9]),
            dev_rx_stream_raw.ready.eq(dev_logic_in_stream.ready),
        ]

        m.d.comb += [
            injector.buttons.eq(self.i_buttons),
            injector.dx.eq(self.i_dx),
            injector.dy.eq(self.i_dy),
        ]

        # --- Host -> Device Path (TARGET -> AUX) ---
        # Data from PC (host_logic_in_stream) goes directly to the device TX CDC fifo
        dev_tx_cdc_input_stream = StreamInterface(payload_width=8)
        m.d.comb += host_logic_in_stream.stream_eq(dev_tx_cdc_input_stream)

        m.d.comb += [
            dev_tx_cdc.w_en.eq(dev_tx_cdc_input_stream.valid),
            dev_tx_cdc.w_data.eq(
                Cat(
                    dev_tx_cdc_input_stream.payload,
                    dev_tx_cdc_input_stream.first,
                    dev_tx_cdc_input_stream.last,
                )
            ),
            dev_tx_cdc_input_stream.ready.eq(dev_tx_cdc.w_rdy),
        ]

        # --- Device -> Host Path (AUX -> TARGET) ---
        # Data from Mouse (dev_logic_in_stream) and Injector go into Arbiter
        m.d.comb += dev_logic_in_stream.stream_eq(arbiter.passthrough_in)
        m.d.comb += injector.source.stream_eq(arbiter.inject_in)

        # Arbiter output goes to the host TX CDC fifo
        arbiter_out_stream = arbiter.merged_out
        m.d.comb += [
            host_tx_cdc.w_en.eq(arbiter_out_stream.valid),
            host_tx_cdc.w_data.eq(
                Cat(
                    arbiter_out_stream.payload,
                    arbiter_out_stream.first,
                    arbiter_out_stream.last,
                )
            ),
            arbiter_out_stream.ready.eq(host_tx_cdc.w_rdy),
        ]

        # --- Packet Activity Indicators ---
        # Add explicit activity indicators for USB packet detection
        # These will be used to drive the activity LEDs
        usb_host_activity = Signal(reset=0)
        usb_dev_activity = Signal(reset=0)

        # Create activity indicators for host (AUX/PC) traffic
        if host_translator is not None:
            # Set activity signal when valid data is received from host
            with m.If(host_translator.rx_valid):
                m.d.sync += usb_host_activity.eq(1)
            with m.Else():
                m.d.sync += usb_host_activity.eq(0)

        # Create activity indicators for device (TARGET/Mouse) traffic
        if dev_translator is not None:
            # Set activity signal when valid data is received from device
            with m.If(dev_translator.rx_valid):
                m.d.sync += usb_dev_activity.eq(1)
            with m.Else():
                m.d.sync += usb_dev_activity.eq(0)

        # Connect activity signals to output pins for LEDs
        m.d.comb += [
            self.o_host_packet_activity.eq(usb_host_activity),
            self.o_dev_packet_activity.eq(usb_dev_activity),
        ]

        # Make sure UART TX activity is visible
        # Use safe access pattern for usb_serial which might not be defined
        if self.use_control_port:
            # Only use this part when usb_serial is guaranteed to exist
            m.d.comb += self.o_uart_tx_activity.eq(
                uart_tx_handler.o_uart_valid | usb_serial.tx.valid
            )
        else:
            # Fallback when only using UART
            m.d.comb += self.o_uart_tx_activity.eq(uart_tx_handler.o_uart_valid)

        # For debugging: send a test message on startup
        startup_message = "Hurricane FPGA USB Proxy Initialized\r\n"
        startup_msg_rom = Array([ord(c) for c in startup_message])

        msg_counter = Signal(8, reset=0)
        msg_sent = Signal(reset=0)

        with m.If(~msg_sent):
            # Basic message sender for startup
            if self.use_control_port:
                # If using CDC-ACM USB serial
                with m.If(usb_serial.tx.ready):
                    m.d.comb += [
                        usb_serial.tx.valid.eq(1),
                        usb_serial.tx.payload.eq(startup_msg_rom[msg_counter]),
                        usb_serial.tx.first.eq(msg_counter == 0),
                        usb_serial.tx.last.eq(msg_counter == len(startup_message) - 1),
                    ]

                    # Increment counter or mark as sent
                    with m.If(msg_counter == len(startup_message) - 1):
                        m.d.sync += [
                            msg_counter.eq(0),
                            msg_sent.eq(1),
                        ]
                    with m.Else():
                        m.d.sync += msg_counter.eq(msg_counter + 1)
            else:
                # If using UART
                with m.If(uart_tx_handler.i_tx_stream.ready):
                    m.d.comb += [
                        uart_tx_handler.i_tx_stream.valid.eq(1),
                        uart_tx_handler.i_tx_stream.payload.eq(
                            startup_msg_rom[msg_counter]
                        ),
                        uart_tx_handler.i_tx_stream.first.eq(msg_counter == 0),
                        uart_tx_handler.i_tx_stream.last.eq(
                            msg_counter == len(startup_message) - 1
                        ),
                    ]

                    # Increment counter or mark as sent
                    with m.If(msg_counter == len(startup_message) - 1):
                        m.d.sync += [
                            msg_counter.eq(0),
                            msg_sent.eq(1),
                        ]
                    with m.Else():
                        m.d.sync += msg_counter.eq(msg_counter + 1)

        return m


class StreamAdapter(Elaboratable):
    """Handles stream conversions between different interfaces and formats.
    Adapters for CDC FIFOs, width changes, and other stream transformations.
    """

    def __init__(self):
        # CDC FIFO to Stream interfaces - 10-bit to 8-bit
        self.rx_stream_raw = StreamInterface(payload_width=10)  # Input from CDC FIFO
        self.rx_stream = StreamInterface(payload_width=8)  # Output to logic

        # Stream to CDC FIFO interfaces - 8-bit to 10-bit
        self.tx_stream = StreamInterface(payload_width=8)  # Input from logic
        self.cdc_w_en = Signal()  # CDC FIFO write enable
        self.cdc_w_data = Signal(10)  # CDC FIFO write data
        self.cdc_w_rdy = Signal()  # CDC FIFO write ready (backpressure)

    def elaborate(self, platform):
        m = Module()

        # Convert from 10-bit CDC output to 8-bit stream
        # (rx_stream_raw -> rx_stream)
        m.d.comb += [
            self.rx_stream.valid.eq(self.rx_stream_raw.valid),
            self.rx_stream.payload.eq(self.rx_stream_raw.payload[0:8]),
            self.rx_stream.first.eq(self.rx_stream_raw.payload[8]),
            self.rx_stream.last.eq(self.rx_stream_raw.payload[9]),
            self.rx_stream_raw.ready.eq(self.rx_stream.ready),
        ]

        # Convert from 8-bit stream to 10-bit CDC input
        # (tx_stream -> CDC FIFO)
        m.d.comb += [
            self.cdc_w_en.eq(self.tx_stream.valid),
            self.cdc_w_data.eq(
                Cat(self.tx_stream.payload, self.tx_stream.first, self.tx_stream.last)
            ),
            self.tx_stream.ready.eq(self.cdc_w_rdy),
        ]

        return m
