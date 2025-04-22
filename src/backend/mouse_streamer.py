#!/usr/bin/env python3

## Welcome to some hacky USB passthrough code. YMMV on if it works

# -*- coding: utf-8 -*-

import os
from amaranth import *
from amaranth.build import Platform, Attrs, Pins, PinsN, Resource, Subsignal
from amaranth.lib.data import Struct
from amaranth.lib.io import Pin
from amaranth.lib.cdc import FFSynchronizer
from amaranth.lib.fifo import SyncFIFOBuffered, AsyncFIFOBuffered
from amaranth.lib.data import Layout
from amaranth.lib.wiring import In, Out

# Import the necessary modules from LUNA
from luna import top_level_cli
from luna.gateware.platform import get_appropriate_platform, NullPin
from luna.gateware.usb.usb2.device import USBDevice
from luna.gateware.interface.ulpi import UTMITranslator

# Import the required standard modules from Amaranth StdIO and SoC
from luna_soc.gateware.vendor.amaranth_stdio.serial import AsyncSerial
from luna_soc.gateware.vendor.lambdasoc.periph         import Peripheral
from luna_soc.gateware.vendor.amaranth_soc.memory import MemoryMap


class USBPacketID:
    OUT = 0b0001
    IN = 0b1001
    SOF = 0b0101
    SETUP = 0b1101
    DATA0 = 0b0011
    DATA1 = 0b1011
    DATA2 = 0b0111
    MDATA = 0b1111

# Custom Stream Interface
class StreamInterface:
    def __init__(self, payload_width=8):
        self.payload_width = payload_width
        self.valid = Signal()
        self.ready = Signal()
        self.payload = Signal(payload_width)
        self.first = Signal()
        self.last = Signal()

    def stream_eq(self, other, *, omit=None):
        """Connects FORWARD signals (valid, payload, first, last)"""
        if omit is None:
            omit = set()
        elif isinstance(omit, str):
            omit = {omit}
        connect_list = []
        if 'valid' not in omit:
            connect_list.append(other.valid.eq(self.valid))
        if 'payload' not in omit:
            connect_list.append(other.payload.eq(self.payload))
        if 'first' not in omit:
            connect_list.append(other.first.eq(self.first))
        if 'last' not in omit:
            connect_list.append(other.last.eq(self.last))
        return connect_list

class USBInStreamInterface(StreamInterface):
    def __init__(self, payload_width=8):
        super().__init__(payload_width)

class USBOutStreamInterface(StreamInterface):
    def __init__(self, payload_width=8):
        super().__init__(payload_width)

# LED Controller
class LEDController(Elaboratable):
    def __init__(self, platform, num_leds=4):
        self.platform = platform
        self.num_leds = num_leds
        self.led_outputs = Array(Signal(name=f"led_{i}") for i in range(num_leds))  # Supports up to 4 LEDs

    def elaborate(self, platform):
        m = Module()

        # Request and connect all available LEDs
        for i in range(self.num_leds):
            try:
                led_pin = platform.request("led", i)
                m.d.comb += led_pin.o.eq(self.led_outputs[i])
            except (ResourceError, AttributeError, IndexError):
                print(f"Note: LED {i} not available")

        return m

# Simplified Boundary Detector
class USBOutStreamBoundaryDetector(Elaboratable):
    """Simplified Boundary Detector using payload == 0 as heuristic for EOP"""

    def __init__(self, domain="usb"):
        self.domain = domain
        self.unprocessed_stream = USBOutStreamInterface()
        self.processed_stream = USBOutStreamInterface()

    def elaborate(self, platform):
        m = Module()
        unprocessed = self.unprocessed_stream
        processed = self.processed_stream

        active_packet = Signal(reset=0)
        payload = Signal.like(unprocessed.payload)
        first = Signal()
        last = Signal()
        valid = Signal()

        m.d.comb += [
            processed.payload.eq(payload),
            processed.first.eq(first),
            processed.last.eq(last),
            processed.valid.eq(valid),
            unprocessed.ready.eq(processed.ready),
        ]

        m.d.comb += [
            valid.eq(unprocessed.valid),
            payload.eq(unprocessed.payload),
            first.eq(0),
            last.eq(0),
        ]

        with m.If(unprocessed.valid & ~active_packet):
            m.d.comb += first.eq(1)
        with m.If(unprocessed.valid & active_packet & (unprocessed.payload == 0)):
            m.d.comb += last.eq(1)
        with m.If(unprocessed.valid & processed.ready):
            with m.If(~active_packet):
                m.d[self.domain] += active_packet.eq(1)
            with m.Elif(last):
                m.d[self.domain] += active_packet.eq(0)
        with m.Elif(~unprocessed.valid & active_packet):
            m.d[self.domain] += active_packet.eq(0)
        return m

# Custom CDC Synchronizer
def synchronize(signal, o_domain, stages=2, name=None):
    """Synchronizes a signal from one clock domain to another."""
    sync_stages = []
    last_stage = signal
    for i in range(stages):
        sig_name = f"{name}_stage{i}" if name else None
        stage = Signal.like(signal, name=sig_name, reset_less=True)
        sync_stages.append(stage)
        last_stage = stage
    m = Module()
    m.d[o_domain] += stage.eq(last_stage)
    return stage

# Simple Mouse Packet Injector
class SimpleMouseInjector(Elaboratable):
    """Injects mouse packets into the USB stream based on UART commands."""

    def __init__(self):
        self.source = USBInStreamInterface(payload_width=8)
        self.trigger = Signal()
        self.buttons = Signal(8)
        self.dx = Signal(8)
        self.dy = Signal(8)
        self._data_pid = Signal(4, reset=USBPacketID.DATA1)

    def elaborate(self, platform):
        m = Module()
        source, buttons, dx, dy = self.source, self.buttons, self.dx, self.dy
        latched_buttons = Signal.like(buttons)
        latched_dx = Signal.like(dx)
        latched_dy = Signal.like(dy)
        # Default assignments
        m.d.comb += [
            source.valid.eq(0),
            source.first.eq(0),
            source.last.eq(0),
            source.payload.eq(0)
        ]
        with m.FSM(domain="usb", name="injector_fsm"):
            with m.State("IDLE"):
                with m.If(self.trigger):
                    m.d.usb += [
                        latched_buttons.eq(buttons),
                        latched_dx.eq(dx),
                        latched_dy.eq(dy)
                    ]
                    m.next = "SEND_PID"

            with m.State("SEND_PID"):
                m.d.comb += [
                    source.valid.eq(1),
                    source.payload.eq(self._data_pid),
                    source.first.eq(1),
                    source.last.eq(0)
                ]
                with m.If(source.ready):
                    m.next = "SEND_BYTE_0"

            with m.State("SEND_BYTE_0"):
                m.d.comb += [
                    source.valid.eq(1),
                    source.payload.eq(latched_buttons),
                    source.first.eq(0),
                    source.last.eq(0)
                ]
                with m.If(source.ready):
                    m.next = "SEND_BYTE_1"

            with m.State("SEND_BYTE_1"):
                m.d.comb += [
                    source.valid.eq(1),
                    source.payload.eq(latched_dx),
                    source.first.eq(0),
                    source.last.eq(0)
                ]
                with m.If(source.ready):
                    m.next = "SEND_BYTE_2"

            with m.State("SEND_BYTE_2"):
                m.d.comb += [
                    source.valid.eq(1),
                    source.payload.eq(latched_dy),
                    source.first.eq(0),
                    source.last.eq(1)
                ]
                with m.If(source.ready):
                    m.next = "IDLE"

        return m

# Packet Arbiter
class PacketArbiter(Elaboratable):
    """Arbitrates between passthrough and inject streams."""
    def __init__(self):
        self.passthrough_in = USBInStreamInterface(payload_width=8)
        self.inject_in = USBInStreamInterface(payload_width=8)
        self.merged_out = USBInStreamInterface(payload_width=8)

    def elaborate(self, platform):
        m = Module()
        passthrough, inject, merged = self.passthrough_in, self.inject_in, self.merged_out
        # Default assignments for the output stream and input readys
        m.d.comb += [
            merged.valid.eq(0),
            merged.payload.eq(0),  # Default all output signals
            merged.first.eq(0),
            merged.last.eq(0),
            passthrough.ready.eq(0),
            inject.ready.eq(0)
        ]

        with m.FSM(domain="usb", name="arbiter_fsm"):
            with m.State("IDLE"):
                # Prioritize inject stream
                with m.If(inject.valid):
                    m.next = "FORWARD_INJECT"
                with m.Elif(passthrough.valid):
                    m.next = "FORWARD_PASSTHROUGH"
            with m.State("FORWARD_PASSTHROUGH"):
                m.d.comb += [
                    merged.valid.eq(passthrough.valid),
                    merged.payload.eq(passthrough.payload),
                    merged.first.eq(passthrough.first),
                    merged.last.eq(passthrough.last),
                    passthrough.ready.eq(merged.ready)  # Connect ready back
                ]

                # Transition logic
                with m.If(passthrough.valid & passthrough.last & merged.ready):
                    m.next = "IDLE"
                # If the source disappears mid-packet (optional, depends on desired behavior)
                with m.Elif(~passthrough.valid):
                    m.next = "IDLE"

            with m.State("FORWARD_INJECT"):
                m.d.comb += [
                    merged.valid.eq(inject.valid),
                    merged.payload.eq(inject.payload),
                    merged.first.eq(inject.first),
                    merged.last.eq(inject.last),
                    inject.ready.eq(merged.ready)  # Connect ready back
                ]

                # Transition logic
                with m.If(inject.valid & inject.last & merged.ready):
                    m.next = "IDLE"
                # If the source disappears mid-packet
                with m.Elif(~inject.valid):
                    m.next = "IDLE"

        return m

# USB Passthrough Analyzer
class USBPassthroughAnalyzer(Elaboratable):
    """Analyzes USB passthrough data and drives status LEDs."""

    def __init__(self):
        # Control signals now mostly inputs to UTMITranslator
        self.i_inject_trigger = Signal()
        self.i_buttons = Signal(8)
        self.i_dx = Signal(8)
        self.i_dy = Signal(8)
        # Status LEDs
        self.host_packet = Signal()
        self.dev_packet = Signal()

    def elaborate(self, platform):
        m = Module()

        if platform is None:
            # --- Offline Mode ---
            print("USBPassthroughAnalyzer: Running in offline mode.")
            # Define dummy layout for offline synthesis/simulation
            ulpi_bus_layout = [
                ("data", [
                    ("i", 8, In),
                    ("o", 8, Out),
                    ("oe", 1, Out)
                ]),
                ("clk", [("o", 1, Out)]),  # Clock out from FPGA
                ("dir", [("i", 1, In)]),   # Direction in to FPGA
                ("nxt", [("i", 1, In)]),   # Next in to FPGA
                ("stp", [("o", 1, Out)]),  # Stop out from FPGA
                ("rst", [("o", 1, Out)])   # Reset out from FPGA
            ]
            target_ulpi_res = Record(ulpi_bus_layout, name="offline_target_ulpi")
            control_ulpi_res = Record(ulpi_bus_layout, name="offline_control_ulpi")
        else:
            target_ulpi_res = platform.request('target_phy', 0)  # J2 -> Host PC side
            control_ulpi_res = platform.request('control_phy', 0)  # J3 -> Device side
        m.submodules.host_translator = host_translator = DomainRenamer("sync")(UTMITranslator(ulpi=target_ulpi_res))
        m.submodules.dev_translator = dev_translator = DomainRenamer("sync")(UTMITranslator(ulpi=control_ulpi_res))
        if platform is not None:  # Only connect in hardware mode
            m.d.comb += [
                target_ulpi_res.clk.o.eq(ClockSignal("sync")),
                control_ulpi_res.clk.o.eq(ClockSignal("sync")),
            ]
        m.d.comb += [
            host_translator.op_mode.eq(0b00),
            host_translator.xcvr_select.eq(0b01),
            host_translator.term_select.eq(1),
            host_translator.suspend.eq(0),
            dev_translator.op_mode.eq(0b00),
            dev_translator.xcvr_select.eq(0b01),
            dev_translator.term_select.eq(1),
            dev_translator.suspend.eq(0),
        ]

        # Remove deprecated pulldown controls if they are not supported
        # host_translator.dp_pulldown.eq(0),
        # host_translator.dm_pulldown.eq(0),
        # dev_translator.dp_pulldown.eq(0),
        # dev_translator.dm_pulldown.eq(0),

        # Configure FIFOs
        fifo_depth, fifo_width = 16, 10
        m.submodules.host_rx_fifo = host_rx_fifo = AsyncFIFOBuffered(width=fifo_width, depth=fifo_depth, w_domain="sync", r_domain="usb")
        m.submodules.dev_rx_fifo = dev_rx_fifo = AsyncFIFOBuffered(width=fifo_width, depth=fifo_depth, w_domain="sync", r_domain="usb")
        m.submodules.host_tx_fifo = host_tx_fifo = AsyncFIFOBuffered(width=fifo_width, depth=fifo_depth, w_domain="usb", r_domain="sync")
        m.submodules.dev_tx_fifo = dev_tx_fifo = AsyncFIFOBuffered(width=fifo_width, depth=fifo_depth, w_domain="usb", r_domain="sync")
        
        # Host RX Boundary Detection
        host_rx_first = Signal()
        host_rx_last = Signal()
        host_prev_rx_active = Signal(reset_less=True)
        m.d.sync += host_prev_rx_active.eq(host_translator.rx_active)
        m.d.comb += host_rx_first.eq(host_translator.rx_valid & (host_translator.rx_active & ~host_prev_rx_active))
        m.d.comb += host_rx_last.eq(~host_translator.rx_active & host_prev_rx_active)

        m.d.comb += [
            host_rx_fifo.w_en.eq(host_translator.rx_valid),
            host_rx_fifo.w_data.eq(Cat(host_translator.rx_data, host_rx_first, host_rx_last)),
        ]

        # Device RX Boundary Detection
        dev_rx_first = Signal()
        dev_rx_last = Signal()
        dev_prev_rx_active = Signal(reset_less=True)
        m.d.sync += dev_prev_rx_active.eq(dev_translator.rx_active)
        m.d.comb += dev_rx_first.eq(dev_translator.rx_valid & (dev_translator.rx_active & ~dev_prev_rx_active))
        # 'last' occurs on falling edge of rx_active
        m.d.comb += dev_rx_last.eq(~dev_translator.rx_active & dev_prev_rx_active)
        m.d.comb += [
            dev_rx_fifo.w_en.eq(dev_translator.rx_valid),
            dev_rx_fifo.w_data.eq(Cat(dev_translator.rx_data, dev_rx_first, dev_rx_last)),
        ]

        # Stream Interfaces
        host_rx_fifo_out = USBOutStreamInterface(payload_width=8)
        dev_rx_fifo_out = USBOutStreamInterface(payload_width=8)

        m.d.comb += [
            host_rx_fifo_out.valid.eq(host_rx_fifo.r_rdy),
            host_rx_fifo.r_en.eq(host_rx_fifo_out.ready),
            Cat(host_rx_fifo_out.payload, host_rx_fifo_out.first, host_rx_fifo_out.last).eq(host_rx_fifo.r_data),

            dev_rx_fifo_out.valid.eq(dev_rx_fifo.r_rdy),
            dev_rx_fifo.r_en.eq(dev_rx_fifo_out.ready),
            Cat(dev_rx_fifo_out.payload, dev_rx_fifo_out.first, dev_rx_fifo_out.last).eq(dev_rx_fifo.r_data),
        ]

        # Boundary Detectors
        m.submodules.host_rx_boundary = host_rx_boundary = USBOutStreamBoundaryDetector(domain="usb")
        m.submodules.dev_rx_boundary = dev_rx_boundary = USBOutStreamBoundaryDetector(domain="usb")

        m.d.comb += [
            *host_rx_fifo_out.stream_eq(host_rx_boundary.unprocessed_stream),
            *dev_rx_fifo_out.stream_eq(dev_rx_boundary.unprocessed_stream),
        ]

        host_logic_out_stream = host_rx_boundary.processed_stream
        dev_logic_out_stream = dev_rx_boundary.processed_stream

        # Forward from host to device
        m.d.comb += [
            dev_tx_fifo.w_en.eq(host_logic_out_stream.valid),
            dev_tx_fifo.w_data.eq(Cat(host_logic_out_stream.payload, host_logic_out_stream.first, host_logic_out_stream.last)),
            host_logic_out_stream.ready.eq(dev_tx_fifo.w_rdy),
        ]

        # Instantiate Injector and Arbiter
        m.submodules.injector = injector = SimpleMouseInjector()
        m.submodules.arbiter = arbiter = PacketArbiter()

        # Hook up injector control signals
        m.d.comb += [
            injector.trigger.eq(self.i_inject_trigger),
            injector.buttons.eq(self.i_buttons),
            injector.dx.eq(self.i_dx),
            injector.dy.eq(self.i_dy),
        ]

        # Connect dev_logic_out_stream to arbiter passthrough input
        m.d.comb += [
            *dev_logic_out_stream.stream_eq(arbiter.passthrough_in)
        ]

        # Connect injector source to arbiter inject input
        m.d.comb += [
            *injector.source.stream_eq(arbiter.inject_in)
        ]

        # Get arbiter output stream
        arbiter_out_stream = arbiter.merged_out

        # Hook up arbiter output to host_tx_fifo
        m.d.comb += [
            host_tx_fifo.w_en.eq(arbiter_out_stream.valid),
            host_tx_fifo.w_data.eq(Cat(arbiter_out_stream.payload, arbiter_out_stream.first, arbiter_out_stream.last)),
            arbiter_out_stream.ready.eq(host_tx_fifo.w_rdy),
        ]

        # Forward from host_tx_fifo to host transmitter
        m.d.comb += [
            host_translator.tx_data.eq(host_tx_fifo.r_data[0:8]),
            host_translator.tx_valid.eq(host_tx_fifo.r_rdy),
            host_tx_fifo.r_en.eq(host_translator.tx_ready),
        ]

        # Forward from dev_tx_fifo to device transmitter
        m.d.comb += [
            dev_translator.tx_data.eq(dev_tx_fifo.r_data[0:8]),
            dev_translator.tx_valid.eq(dev_tx_fifo.r_rdy),
            dev_tx_fifo.r_en.eq(dev_translator.tx_ready),
        ]

        # Status LEDs
        host_activity_duration = int(60e6 * 0.05)  # 50ms at 60MHz
        host_counter = Signal(range(host_activity_duration))
        dev_activity_duration = int(60e6 * 0.05)
        dev_counter = Signal(range(dev_activity_duration))

        with m.If(host_rx_fifo.w_en):
            m.d.sync += host_counter.eq(host_activity_duration - 1)
        with m.Elif(host_counter > 0):
            m.d.sync += host_counter.eq(host_counter - 1)

        with m.If(dev_rx_fifo.w_en):
            m.d.sync += dev_counter.eq(dev_activity_duration - 1)
        with m.Elif(dev_counter > 0):
            m.d.sync += dev_counter.eq(dev_counter - 1)

        m.d.comb += [
            self.host_packet.eq(host_counter > 0),
            self.dev_packet.eq(dev_counter > 0)
        ]

        return m

# Top-Level Module
class CynthionUartInjectionTop(Elaboratable):
    BAUD_RATE = 115200
    SYNC_CLK_FREQ = 60_000_000
    USB_CLK_FREQ = 60_000_000  # Adjusted to match sync_clk for simplicity

    def elaborate(self, platform: Platform):
        m = Module()

        # Clock and Reset Setup
        if platform is None:
            print("Running in offline mode...")
            m.domains.sync = ClockDomain(local=True)
            m.domains.usb = ClockDomain(local=True)
            m.submodules.analyzer = USBPassthroughAnalyzer()
        else:
            # Obtain the default platform clock
            clk = platform.request(platform.default_clk)
            m.domains.sync = ClockDomain()
            m.d.comb += ClockSignal("sync").eq(clk.i)

            # For simplicity, use the same clock for USB domain
            m.domains.usb = ClockDomain()
            m.d.comb += ClockSignal("usb").eq(clk.i)

            # Reset Logic
            m.d.comb += [
                ResetSignal("sync").eq(0),
                ResetSignal("usb").eq(0)
            ]

            # UART Handler using AsyncSerial
            uart_pins = platform.request("uart", 0)
            print(f"Using UART resource: uart_0")
            uart_divisor = int(self.SYNC_CLK_FREQ // self.BAUD_RATE)

            m.submodules.uart = uart = AsyncSerial(
                divisor = uart_divisor,
                pins    = uart_pins,
                data_bits = 8
            )

            # FIFO Buffers for UART
            rx_fifo = SyncFIFOBuffered(width=8, depth=16)
            tx_fifo = SyncFIFOBuffered(width=8, depth=16)
            m.submodules.rx_fifo = rx_fifo
            m.submodules.tx_fifo = tx_fifo

            # Connect UART RX and TX to FIFOs
            m.d.comb += [
                # UART RX to RX FIFO
                rx_fifo.w_en.eq(uart.rx.rdy),
                rx_fifo.w_data.eq(uart.rx.data),
                uart.rx.ack.eq(rx_fifo.w_rdy),

                # TX FIFO to UART TX
                uart.tx.data.eq(tx_fifo.r_data),
                uart.tx.ack.eq(tx_fifo.r_en),
                uart.tx.rdy.eq(tx_fifo.r_rdy),
            ]

            # Command Handling FSM
            temp_buttons = Signal(8)
            temp_dx = Signal(8)
            buttons = Signal(8)
            dx = Signal(8)
            dy = Signal(8)
            cmd_ready = Signal()
            m.d.comb += cmd_ready.eq(0)
            with m.FSM(name="uart_rx_fsm"):
                with m.State("IDLE"):
                    with m.If(rx_fifo.r_rdy):
                        m.d.sync += temp_buttons.eq(rx_fifo.r_data)
                        m.d.sync += rx_fifo.r_en.eq(1)
                        m.next = "WAIT_DX"
                    with m.Else():
                        m.d.sync += rx_fifo.r_en.eq(0)

                with m.State("WAIT_DX"):
                    with m.If(rx_fifo.r_rdy):
                        m.d.sync += temp_dx.eq(rx_fifo.r_data)
                        m.d.sync += rx_fifo.r_en.eq(1)
                        m.next = "WAIT_DY"
                    with m.Else():
                        m.d.sync += rx_fifo.r_en.eq(0)

                with m.State("WAIT_DY"):
                    with m.If(rx_fifo.r_rdy):
                        m.d.sync += [
                            buttons.eq(temp_buttons),
                            dx.eq(temp_dx),
                            dy.eq(rx_fifo.r_data)
                        ]
                        m.d.sync += rx_fifo.r_en.eq(1)
                        m.d.comb += cmd_ready.eq(1)
                        m.next = "IDLE"
                    with m.Else():
                        m.d.sync += rx_fifo.r_en.eq(0)

            # Synchronize UART signals to USB domain
            m.submodules += FFSynchronizer(buttons, buttons_usb := Signal(8), o_domain="usb")
            m.submodules += FFSynchronizer(dx, dx_usb := Signal(8), o_domain="usb")
            m.submodules += FFSynchronizer(dy, dy_usb := Signal(8), o_domain="usb")
            m.submodules += FFSynchronizer(cmd_ready, cmd_ready_usb := Signal(), o_domain="usb")

            prev_cmd_ready = Signal(reset_less=True, domain="usb")
            inject_trigger = Signal()

            with m.If(cmd_ready_usb & ~prev_cmd_ready):
                m.d.usb += inject_trigger.eq(1)
            with m.Else():
                m.d.usb += inject_trigger.eq(0)
            m.d.usb += prev_cmd_ready.eq(cmd_ready_usb)

            # Instantiate Analyzer Module
            m.submodules.analyzer = analyzer = USBPassthroughAnalyzer()
            m.d.comb += [
                analyzer.i_buttons.eq(buttons_usb),
                analyzer.i_dx.eq(dx_usb),
                analyzer.i_dy.eq(dy_usb),
                analyzer.i_inject_trigger.eq(inject_trigger)
            ]

            # Instantiate LED Controller
            m.submodules.leds = leds = LEDController(platform)
            led_state = Signal()
            with m.If(inject_trigger):
                m.d.usb += led_state.eq(~led_state)
            m.d.comb += [
                # LED 0: Injection trigger (toggle on each injection)
                leds.led_outputs[0].eq(led_state),
                # LED 1: Host packet activity
                leds.led_outputs[1].eq(analyzer.host_packet),
                # LED 2: Device packet activity
                leds.led_outputs[2].eq(analyzer.dev_packet),
                # LED 3: Optional usage (set to 0 for now)
                leds.led_outputs[3].eq(0)
            ]

        return m

# Entry Point
if __name__ == "__main__":
    _PlatformClass = get_appropriate_platform()
    platform_instance = _PlatformClass
    print(f"Using located platform instance: {platform_instance.name}")
    print("\nAttempting offline elaboration check...")
    if os.getenv("BUILD_LOCAL") == "1":
        try:
            top_design = CynthionUartInjectionTop()
            print("Elaborating design with platform=None...")
            fragment = Fragment.get(top_design, platform=None)
            print("Offline elaboration successful.")

            # Optionally generate Verilog output
            # print("Generating Verilog from offline fragment...")
            # from amaranth.back import verilog
            # from pathlib import Path
            # build_dir = Path("./build_offline")
            # build_dir.mkdir(parents=True, exist_ok=True)
            # verilog_file = build_dir / "top_offline.v"
            # with open(verilog_file, "w") as f:
            #     f.write(verilog.convert(fragment, name="top_offline"))
            # print(f"Offline Verilog written to: {verilog_file}")

        except Exception as e:
            print("\nERROR during offline elaboration:")
            import traceback
            traceback.print_exc()
            exit(1)
    else:
        # Hardware Build (using luna.top_level_cli)
        print("\nInitiating hardware build...")
        try:
            top_level_cli(
                CynthionUartInjectionTop(),
                platform=platform_instance,
                name="top_hw",
                build_dir="build_hw",
            )
            print("\nHardware build process completed or Cynthion programmer launched.")
            print("Check 'build_hw/' directory and programmer output.")
        except Exception as e:
            print("\nERROR during hardware build:")
            import traceback
            traceback.print_exc()
            exit(1)
