#!/usr/bin/env python3

# -*- coding: utf-8 -*-

import os
from amaranth import *
from amaranth.build import Platform, ResourceError, PinsN, Pins, Subsignal, Resource, Attrs, Connector, DiffPairs
from amaranth.lib.cdc import FFSynchronizer
from amaranth.lib.fifo import SyncFIFOBuffered, AsyncFIFOBuffered
from amaranth.hdl import Record

from luna import top_level_cli
from luna.gateware.platform import get_appropriate_platform
from luna.gateware.interface.ulpi import UTMITranslator

# --- Constants and Helper Classes ---

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
        if omit is None: omit = set()
        elif isinstance(omit, str): omit = {omit}
        connect_list = []
        if 'valid' not in omit: connect_list.append(other.valid.eq(self.valid))
        if 'payload' not in omit: connect_list.append(other.payload.eq(self.payload))
        if 'first' not in omit: connect_list.append(other.first.eq(self.first))
        if 'last' not in omit: connect_list.append(other.last.eq(self.last))
        return connect_list

class USBInStreamInterface(StreamInterface):
    def __init__(self, payload_width=8): 
        super().__init__(payload_width)
        
class USBOutStreamInterface(StreamInterface):
    def __init__(self, payload_width=8): 
        super().__init__(payload_width)

# --- Core Hardware Modules ---
class UARTReceiver(Elaboratable):
    """ Simple UART receiver, similar style to LUNA's UARTTransmitter.

    Receives data based on a specified divisor. Assumes 8N1 format.
    Outputs received bytes via a StreamInterface.

    Attributes
    ----------
    rx : Signal, input
        The UART data input line.
    stream : StreamInterface, output stream
        Stream carrying the received data bytes. `valid` is asserted for one
        cycle when a byte is ready and the stop bit was correct. `ready`
        from downstream indicates the byte was consumed.

    Parameters
    ----------
    divisor : int
        The number of `sync` clock cycles per bit period. Must be >= 4.
    """

    START_BIT = 0
    STOP_BIT  = 1
    EXPECTED_STOP_BIT = STOP_BIT # For 8N1, stop bit should be high

    def __init__(self, *, divisor):
        if divisor < 4:
             # Needs time to detect start, wait half period, potentially check again/wait full period
             raise ValueError("UART divisor must be >= 4 for stable sampling.")
        self.divisor = divisor
        self._center_sample_time = divisor // 2 # Sample near the middle of the bit time

        # I/O port
        self.rx = Signal(reset=1) # Input line, default high (idle)
        self.stream = StreamInterface() # Output stream for received bytes

    def elaborate(self, platform):
        m = Module()
        stream = self.stream

        bit_timer = Signal(range(self.divisor)) # Counts cycles within a bit period
        bit_count = Signal(range(9))            # Counts received bits (0-7 data, 8 stop)
        data_reg = Signal(8)                    # Shift register for incoming data bits
        framing_error = Signal()                # Latches if stop bit is wrong

        # Controls when to sample the input line
        sample_strobe = Signal()
        m.d.comb += sample_strobe.eq(bit_timer == self._center_sample_time)

        # --- Default outputs ---
        m.d.comb += [
            stream.valid.eq(0),
            stream.payload.eq(data_reg) # Output the latched byte when valid
            # stream.ready is an input from downstream
        ]

        with m.FSM(name="uart_rx_fsm") as fsm:

            # IDLE: Wait for the line to go low (start bit)
            with m.State("IDLE"):
                m.d.sync += bit_timer.eq(0)     # Reset timer
                m.d.sync += bit_count.eq(0)     # Reset bit counter
                m.d.sync += framing_error.eq(0) # Clear error flag

                # Detect falling edge
                with m.If(~self.rx):
                    # Start timing from the assumed beginning of the start bit
                    # Wait until the center of the start bit to confirm it's not a glitch
                    m.d.sync += bit_timer.eq(self.divisor - 1)
                    m.next = "CHECK_START"

            # CHECK_START: Verify the start bit around its center time
            with m.State("CHECK_START"):
                m.d.sync += bit_timer.eq(bit_timer - 1)
                with m.If(sample_strobe): # Reached center of start bit?
                    with m.If(~self.rx): # Still low? Valid start bit confirmed.
                        # Reset timer to time the first data bit
                        m.d.sync += bit_timer.eq(self.divisor - 1)
                        m.next = "RECEIVE_DATA"
                    with m.Else(): # Glitch? Go back to idle.
                        m.next = "IDLE"
                # Handle edge case where divisor is small and we might miss the sample strobe exactly
                with m.Elif(bit_timer == 0): # If we somehow ran out of time without sampling
                    m.next = "IDLE" # Treat as glitch/timeout

            # RECEIVE_DATA: Clock in the 8 data bits
            with m.State("RECEIVE_DATA"):
                m.d.sync += bit_timer.eq(bit_timer - 1)

                # If it's sample time, shift the received bit into the data register (LSB first)
                with m.If(sample_strobe):
                    m.d.sync += data_reg.eq(Cat(self.rx, data_reg[:-1])) # Shift right, new LSB is self.rx

                # If the bit period just ended
                with m.If(bit_timer == 0):
                    m.d.sync += bit_timer.eq(self.divisor - 1) # Reset timer for next bit
                    m.d.sync += bit_count.eq(bit_count + 1)    # Move to next bit index

                    # If we just finished clocking in the last data bit (bit 7)
                    with m.If(bit_count == 7):
                        m.next = "RECEIVE_STOP" # Move to check the stop bit

            # RECEIVE_STOP: Wait for the stop bit period and check its value at sample time
            with m.State("RECEIVE_STOP"):
                m.d.sync += bit_timer.eq(bit_timer - 1)

                # Check the stop bit value at the sampling point
                with m.If(sample_strobe):
                    with m.If(self.rx != self.EXPECTED_STOP_BIT):
                        m.d.sync += framing_error.eq(1) # Record framing error if stop bit is low

                # If the stop bit period has ended
                with m.If(bit_timer == 0):
                    # Byte reception complete. Make data available if no error.
                    with m.If(~framing_error):
                        m.d.comb += stream.valid.eq(1) # Assert valid for one cycle

                        # If downstream consumes the data this cycle, return to IDLE immediately.
                        # Otherwise, stay here until ready is asserted.
                        with m.If(stream.ready):
                            m.next = "IDLE"
                        with m.Else():
                            m.next = "WAIT_READY" # Wait for downstream consumer
                    with m.Else():
                        # Framing error detected, discard byte and return to idle
                         m.next = "IDLE"

            # WAIT_READY: Data is valid, but downstream hasn't taken it yet
            with m.State("WAIT_READY"):
                # Keep data valid
                m.d.comb += stream.valid.eq(1)
                # Once downstream is ready, move back to idle for next byte
                with m.If(stream.ready):
                    m.next = "IDLE"

        return m

class LEDController(Elaboratable):
    def __init__(self, platform, num_leds=4):
        self.platform = platform
        self.num_leds = num_leds
        self.led_outputs = Array(Signal(name=f"led_{i}") for i in range(self.num_leds))

    def elaborate(self, platform):
        m = Module()
        for i in range(self.num_leds):
            try:
                led_pin = platform.request("led", i)
                m.d.comb += led_pin.o.eq(self.led_outputs[i])
            except (ResourceError, AttributeError, IndexError): print(f"Note: LED {i} not available")
        return m

class USBOutStreamBoundaryDetector(Elaboratable):
    # NOTE: The payload == 0 heuristic might be unreliable. Consider proper EOP signaling if needed.
    def __init__(self, domain="usb"):
        self.domain = domain
        self.unprocessed_stream = USBOutStreamInterface()
        self.processed_stream = USBOutStreamInterface()
    def elaborate(self, platform):
        m = Module(); unprocessed = self.unprocessed_stream; processed = self.processed_stream
        active_packet = Signal(reset=0); payload = Signal.like(unprocessed.payload)
        first = Signal(); last = Signal(); valid = Signal()
        m.d.comb += [processed.payload.eq(payload), processed.first.eq(first), processed.last.eq(last), processed.valid.eq(valid), unprocessed.ready.eq(processed.ready)]
        m.d.comb += [valid.eq(unprocessed.valid), payload.eq(unprocessed.payload), first.eq(0), last.eq(0)]
        with m.If(unprocessed.valid & ~active_packet): m.d.comb += first.eq(1)
        with m.If(unprocessed.valid & active_packet & (unprocessed.payload == 0)): m.d.comb += last.eq(1) # Heuristic EOP
        with m.If(unprocessed.valid & processed.ready):
            with m.If(~active_packet): m.d[self.domain] += active_packet.eq(1)
            with m.Elif(last): m.d[self.domain] += active_packet.eq(0)
        with m.Elif(~unprocessed.valid & active_packet): m.d[self.domain] += active_packet.eq(0)
        return m

class SimpleMouseInjector(Elaboratable):
    def __init__(self):
        self.source = USBInStreamInterface(payload_width=8)
        self.trigger = Signal()
        self.buttons = Signal(8)
        self.dx = Signal(8)
        self.dy = Signal(8)
        self._data_pid = Signal(4, reset=USBPacketID.DATA1)

    def elaborate(self, platform):
        m = Module(); source = self.source; latched_buttons = Signal.like(self.buttons); latched_dx = Signal.like(self.dx); latched_dy = Signal.like(self.dy)
        m.d.comb += [source.valid.eq(0), source.first.eq(0), source.last.eq(0), source.payload.eq(0)]
        with m.FSM(domain="usb", name="injector_fsm"):
            with m.State("IDLE"):
                with m.If(self.trigger):
                    m.d.usb += [latched_buttons.eq(self.buttons), latched_dx.eq(self.dx), latched_dy.eq(self.dy)]
                    m.next = "SEND_PID"
            with m.State("SEND_PID"):
                m.d.comb += [source.valid.eq(1), source.payload.eq(self._data_pid), source.first.eq(1), source.last.eq(0)]
                with m.If(source.ready): m.next = "SEND_BYTE_0"
            with m.State("SEND_BYTE_0"): # Buttons
                m.d.comb += [source.valid.eq(1), source.payload.eq(latched_buttons), source.first.eq(0), source.last.eq(0)]
                with m.If(source.ready): m.next = "SEND_BYTE_1"
            with m.State("SEND_BYTE_1"): # DX
                m.d.comb += [source.valid.eq(1), source.payload.eq(latched_dx), source.first.eq(0), source.last.eq(0)]
                with m.If(source.ready): m.next = "SEND_BYTE_2"
            with m.State("SEND_BYTE_2"): # DY
                m.d.comb += [source.valid.eq(1), source.payload.eq(latched_dy), source.first.eq(0), source.last.eq(1)]
                with m.If(source.ready): m.next = "IDLE"
        return m

class PacketArbiter(Elaboratable):
    def __init__(self): 
        self.passthrough_in = USBInStreamInterface() 
        self.inject_in = USBInStreamInterface()
        self.merged_out = USBInStreamInterface()

    def elaborate(self, platform):
        m = Module(); passthrough, inject, merged = self.passthrough_in, self.inject_in, self.merged_out
        m.d.comb += [merged.valid.eq(0), merged.payload.eq(0), merged.first.eq(0), merged.last.eq(0), passthrough.ready.eq(0), inject.ready.eq(0)]
        with m.FSM(domain="usb", name="arbiter_fsm"):
            with m.State("IDLE"):
                with m.If(inject.valid): m.next = "FORWARD_INJECT"
                with m.Elif(passthrough.valid): m.next = "FORWARD_PASSTHROUGH"
            with m.State("FORWARD_PASSTHROUGH"):
                m.d.comb += passthrough.stream_eq(merged); m.d.comb += passthrough.ready.eq(merged.ready)
                with m.If(passthrough.valid & passthrough.last & merged.ready): m.next = "IDLE"
                with m.Elif(~passthrough.valid): m.next = "IDLE"
            with m.State("FORWARD_INJECT"):
                m.d.comb += inject.stream_eq(merged); m.d.comb += inject.ready.eq(merged.ready)
                with m.If(inject.valid & inject.last & merged.ready): m.next = "IDLE"
                with m.Elif(~inject.valid): m.next = "IDLE"
        return m

class USBPassthroughAnalyzer(Elaboratable):
    def __init__(self):
        self.i_inject_trigger = Signal()
        self.i_buttons = Signal(8)
        self.i_dx = Signal(8)
        self.i_dy = Signal(8)
        self.o_host_packet_activity = Signal()
        self.o_dev_packet_activity = Signal()

    def elaborate(self, platform):
        m = Module()
        if platform is None:
             print("USBPassthroughAnalyzer: Running in offline mode.")
             # Dummy setup for offline...
             ulpi_bus_layout = [ ("data", 8), ("clk", 1), ("dir", 1), ("nxt", 1), ("stp", 1), ("rst", 1)]
             target_ulpi_res = Record([('i', ulpi_bus_layout), ('o', ulpi_bus_layout)])
             control_ulpi_res = Record([('i', ulpi_bus_layout), ('o', ulpi_bus_layout)])
             m.submodules.host_translator = host_translator = DomainRenamer("sync")(UTMITranslator(ulpi=target_ulpi_res))
             m.submodules.dev_translator = dev_translator = DomainRenamer("sync")(UTMITranslator(ulpi=control_ulpi_res))
        else:
            target_ulpi_res = platform.request('target_phy', 0)
            control_ulpi_res = platform.request('control_phy', 0)
            m.submodules.host_translator = host_translator = DomainRenamer("sync")(UTMITranslator(ulpi=target_ulpi_res))
            m.submodules.dev_translator = dev_translator = DomainRenamer("sync")(UTMITranslator(ulpi=control_ulpi_res))
            m.d.comb += [ host_translator.op_mode.eq(0b00), host_translator.xcvr_select.eq(0b01), host_translator.term_select.eq(1), host_translator.suspend.eq(0) ]
            m.d.comb += [ dev_translator.op_mode.eq(0b00), dev_translator.xcvr_select.eq(0b01), dev_translator.term_select.eq(1), dev_translator.suspend.eq(0) ]
        # --- FIFOs ---
        fifo_depth, fifo_width = 16, 10
        m.submodules.host_rx_fifo = host_rx_fifo = AsyncFIFOBuffered(width=fifo_width, depth=fifo_depth, w_domain="sync", r_domain="usb")
        m.submodules.dev_rx_fifo = dev_rx_fifo = AsyncFIFOBuffered(width=fifo_width, depth=fifo_depth, w_domain="sync", r_domain="usb")
        m.submodules.host_tx_fifo = host_tx_fifo = AsyncFIFOBuffered(width=fifo_width, depth=fifo_depth, w_domain="usb", r_domain="sync")
        m.submodules.dev_tx_fifo = dev_tx_fifo = AsyncFIFOBuffered(width=fifo_width, depth=fifo_depth, w_domain="usb", r_domain="sync")
        # --- Host RX Path ---
        host_rx_first = Signal(); host_rx_last = Signal(); host_prev_rx_active = Signal(reset_less=True)
        m.d.sync += host_prev_rx_active.eq(host_translator.rx_active)
        m.d.comb += host_rx_first.eq(host_translator.rx_valid & (host_translator.rx_active & ~host_prev_rx_active))
        m.d.comb += host_rx_last.eq(~host_translator.rx_active & host_prev_rx_active); m.d.comb += [host_rx_fifo.w_en.eq(host_translator.rx_valid), host_rx_fifo.w_data.eq(Cat(host_translator.rx_data, host_rx_first, host_rx_last))]
        # --- Device RX Path ---
        dev_rx_first = Signal(); dev_rx_last = Signal(); dev_prev_rx_active = Signal(reset_less=True)
        m.d.sync += dev_prev_rx_active.eq(dev_translator.rx_active)
        m.d.comb += dev_rx_first.eq(dev_translator.rx_valid & (dev_translator.rx_active & ~dev_prev_rx_active))
        m.d.comb += dev_rx_last.eq(~dev_translator.rx_active & dev_prev_rx_active); m.d.comb += [dev_rx_fifo.w_en.eq(dev_translator.rx_valid), dev_rx_fifo.w_data.eq(Cat(dev_translator.rx_data, dev_rx_first, dev_rx_last))]
        # --- Stream Interfaces from RX FIFOs ---
        host_rx_fifo_out = USBOutStreamInterface(); dev_rx_fifo_out = USBOutStreamInterface()
        m.d.comb += [host_rx_fifo_out.valid.eq(host_rx_fifo.r_rdy), host_rx_fifo.r_en.eq(host_rx_fifo_out.ready), Cat(host_rx_fifo_out.payload, host_rx_fifo_out.first, host_rx_fifo_out.last).eq(host_rx_fifo.r_data)]
        m.d.comb += [dev_rx_fifo_out.valid.eq(dev_rx_fifo.r_rdy), dev_rx_fifo.r_en.eq(dev_rx_fifo_out.ready), Cat(dev_rx_fifo_out.payload, dev_rx_fifo_out.first, dev_rx_fifo_out.last).eq(dev_rx_fifo.r_data)]
        # --- Use direct FIFO outputs ---
        host_logic_out_stream = host_rx_fifo_out; dev_logic_out_stream = dev_rx_fifo_out
        # --- Data Path Logic ---
        m.submodules.injector = injector = SimpleMouseInjector(); m.submodules.arbiter = arbiter = PacketArbiter()
        m.d.comb += [injector.trigger.eq(self.i_inject_trigger), injector.buttons.eq(self.i_buttons), injector.dx.eq(self.i_dx), injector.dy.eq(self.i_dy)]
        # Host -> Device Path
        m.d.comb += [dev_tx_fifo.w_en.eq(host_logic_out_stream.valid), dev_tx_fifo.w_data.eq(Cat(host_logic_out_stream.payload, host_logic_out_stream.first, host_logic_out_stream.last)), host_logic_out_stream.ready.eq(dev_tx_fifo.w_rdy)]
        # Device -> Host Path (via Arbiter)
        m.d.comb += dev_logic_out_stream.stream_eq(arbiter.passthrough_in); m.d.comb += injector.source.stream_eq(arbiter.inject_in)
        arbiter_out_stream = arbiter.merged_out
        m.d.comb += [host_tx_fifo.w_en.eq(arbiter_out_stream.valid), host_tx_fifo.w_data.eq(Cat(arbiter_out_stream.payload, arbiter_out_stream.first, arbiter_out_stream.last)), arbiter_out_stream.ready.eq(host_tx_fifo.w_rdy)]
        # --- TX Path FIFOs -> PHYs ---
        m.d.comb += [host_translator.tx_data.eq(host_tx_fifo.r_data[0:8]), host_translator.tx_valid.eq(host_tx_fifo.r_rdy), host_tx_fifo.r_en.eq(host_translator.tx_ready)]
        m.d.comb += [dev_translator.tx_data.eq(dev_tx_fifo.r_data[0:8]), dev_translator.tx_valid.eq(dev_tx_fifo.r_rdy), dev_tx_fifo.r_en.eq(dev_translator.tx_ready)]
        # --- Status LED Logic ---
        host_activity_duration = int(60e6 * 0.05); host_counter = Signal(range(host_activity_duration))
        dev_activity_duration = int(60e6 * 0.05); dev_counter = Signal(range(dev_activity_duration))
        with m.If(host_rx_fifo.w_en): m.d.sync += host_counter.eq(host_activity_duration - 1)
        with m.Elif(host_counter > 0): m.d.sync += host_counter.eq(host_counter - 1)
        with m.If(dev_rx_fifo.w_en): m.d.sync += dev_counter.eq(dev_activity_duration - 1)
        with m.Elif(dev_counter > 0): m.d.sync += dev_counter.eq(dev_counter - 1)
        m.d.comb += [self.o_host_packet_activity.eq(host_counter > 0), self.o_dev_packet_activity.eq(dev_counter > 0)]
        return m


class UartCommandHandler(Elaboratable):
    """
    Handles UART communication over PMOD using custom UARTReceiver,
    parses 3-byte mouse commands, and outputs command data.
    Operates in the 'sync' clock domain.
    """
    def __init__(self, platform, baud_rate, clk_freq):
        self.platform = platform
        self.baud_rate = baud_rate
        self.clk_freq = clk_freq

        # Outputs (sync domain)
        self.o_buttons = Signal(8)
        self.o_dx = Signal(8)
        self.o_dy = Signal(8)
        self.o_cmd_ready = Signal() # Pulse high for one cycle when command complete

    def elaborate(self, platform):
        m = Module()

        # --- UART Pin Setup ---
        print(f"UartCommandHandler: Configuring UART on PMOD using custom UARTReceiver...")
        try:
            # Request the PMOD interface
            pmod_io = self.platform.request("user_pmod", 0)
            print(f"Available pins on pmod_io: {dir(pmod_io)}")
        except ResourceError:
            print("ERROR: Could not request 'user_pmod', number 0.")
            print("Available resources:", self.platform.resources)
            raise

        # Try to figure out available pins first by inspecting
        try:
            # Direct access to pins - first try numbered attributes
            available_attrs = [a for a in dir(pmod_io) if not a.startswith('_')]
            print(f"Available pmod_io attributes: {available_attrs}")
            
            # Try a few common naming patterns
            # Approach 1: Look for pin objects directly
            uart_rx_pin = None
            uart_tx_pin = None
            
            # Pins might be accessed by index, number, or name
            for attr in available_attrs:
                if attr.lower() in ['0', '1', '2', '3', '4', '5', '6', '7'] or attr in ['p0', 'p1', 'p2', 'p3', 'p4', 'p5', 'p6', 'p7']:
                    print(f"Found potential pin attribute: {attr}")
                    if uart_rx_pin is None:
                        uart_rx_pin = getattr(pmod_io, attr)
                        print(f"Using {attr} as UART RX pin")
                    elif uart_tx_pin is None:
                        uart_tx_pin = getattr(pmod_io, attr)
                        print(f"Using {attr} as UART TX pin")
                        break
            
            # If no pins found yet, they might be in a collection
            if uart_rx_pin is None:
                # They might be in a list/array
                if hasattr(pmod_io, 'pins'):
                    print(f"Found pins collection, using pins[0] as RX and pins[1] as TX")
                    uart_rx_pin = pmod_io.pins[0]
                    uart_tx_pin = pmod_io.pins[1]
                    
            # A last resort - just use the object itself if it's simple
            if uart_rx_pin is None and hasattr(pmod_io, 'i') and hasattr(pmod_io, 'o'):
                print("Using pmod_io directly as a single pin")
                uart_rx_pin = pmod_io
                uart_tx_pin = pmod_io  # same pin
                
        except AttributeError as e:
            print(f"ERROR accessing PMOD pins: {e}")
            print(f"Please update the code with the correct pin attribute names.")
            raise

        # Configure pin directions - if we found valid pins
        if uart_rx_pin is not None:
            print(f"Setting up UART RX pin as input")
            if hasattr(uart_rx_pin, 'oe'):
                m.d.comb += uart_rx_pin.oe.eq(0)  # RX is input
            
            # Connect UART Receiver to RX pin
            uart_divisor = int(self.clk_freq // self.baud_rate)
            m.submodules.uart_rx = uart_rx = UARTReceiver(divisor=uart_divisor)
            print(f"  - UART RX Divisor: {uart_divisor}")
            
            # Connect the physical pin to the UART receiver
            if hasattr(uart_rx_pin, 'i'):
                m.d.comb += uart_rx.rx.eq(uart_rx_pin.i)
                print("Connected UART RX to pin.i")
            else:
                # Maybe it's just a Signal?
                m.d.comb += uart_rx.rx.eq(uart_rx_pin)
                print("Connected UART RX directly to pin")
                
            # Configure TX pin as Hi-Z if it's separate from RX
            if uart_tx_pin is not uart_rx_pin and hasattr(uart_tx_pin, 'oe'):
                m.d.comb += uart_tx_pin.oe.eq(0)  # TX as Hi-Z since we're not using it
        else:
            print("ERROR: Could not find any usable pins on the PMOD interface!")
            raise ValueError("No usable pins found on PMOD interface")
            
        # --- UART RX FIFO (Sync Domain) ---
        rx_fifo = SyncFIFOBuffered(width=8, depth=16)
        m.submodules.rx_fifo = rx_fifo
        # --- Connect UART Receiver Stream -> RX FIFO ---
        m.d.comb += [
            rx_fifo.w_en.eq(uart_rx.stream.valid),
            rx_fifo.w_data.eq(uart_rx.stream.payload),
            uart_rx.stream.ready.eq(rx_fifo.w_rdy),
        ]
        temp_buttons = Signal(8)
        temp_dx = Signal(8)
        m.d.comb += self.o_cmd_ready.eq(0)
        m.d.sync += rx_fifo.r_en.eq(0)
        with m.FSM(domain="sync", name="uart_rx_fsm"):
            with m.State("IDLE"):
                with m.If(rx_fifo.r_rdy): 
                    m.d.sync += [
                        temp_buttons.eq(rx_fifo.r_data),
                        rx_fifo.r_en.eq(1)
                    ]
                    m.next="WAIT_DX"
            with m.State("WAIT_DX"):
                m.d.sync += rx_fifo.r_en.eq(0)
                with m.If(rx_fifo.r_rdy): 
                    m.d.sync += [
                        temp_dx.eq(rx_fifo.r_data),
                        rx_fifo.r_en.eq(1)
                    ]
                    m.next="WAIT_DY"
            with m.State("WAIT_DY"):
                m.d.sync += rx_fifo.r_en.eq(0)
                with m.If(rx_fifo.r_rdy): 
                    m.d.sync += [
                        self.o_buttons.eq(temp_buttons),
                        self.o_dx.eq(temp_dx),
                        self.o_dy.eq(rx_fifo.r_data),
                        rx_fifo.r_en.eq(1)
                    ]
                    m.d.comb += self.o_cmd_ready.eq(1)
                    m.next="IDLE"

        return m

# --- Top-Level Module ---
class CynthionUartInjectionTop(Elaboratable):
    """ Top-level design connecting UART handler, USB passthrough/analyzer, and LEDs """
    BAUD_RATE = 115200
    SYNC_CLK_FREQ = int(60e6)
    USB_CLK_FREQ = SYNC_CLK_FREQ

    def elaborate(self, platform: Platform):
        m = Module()
        # --- Clocking and Reset ---
        if platform is None:
            print("Running in offline mode (no platform)...")
            m.domains.sync = ClockDomain(local=True); m.domains.usb = ClockDomain(local=True)
        else:
            clk = platform.request(platform.default_clk)
            m.domains.sync = ClockDomain()
            m.domains.usb = ClockDomain()
            rst_signal = Const(0) # Default to no reset (tie low)
            rst_res = None        # To hold the found reset resource

            try:
                rst_res = platform.request("rst", 0)
                print(f"Using '{rst_res.name}' as reset signal.")
            except ResourceError:
                print("Resource 'rst#0' not found. Trying 'button_user#0' as reset.")
                try:
                    # The button resource is defined as PinsN("M14", dir="i")
                    rst_res = platform.request("button_user", 0)
                    print(f"Using '{rst_res.name}' as reset signal (active-low).")
                except ResourceError:
                    print("Warning: Neither 'rst#0' nor 'button_user#0' resource found for reset. Using constant 0.")
                    rst_res = None # Ensure rst_res is None if neither worked
            if rst_res is not None:
                # button_user is defined as PinsN, meaning it's active-low.
                # A standard reset signal is typically active-low.
                # So, using the raw input `.i` might be correct if the downstream
                # logic expects an active-low reset. If reset logic expects
                # active-high, you would invert it here (~rst_res.i).
                # Assuming standard active-low reset behavior:
                rst_signal = rst_res.i
                print(f"Connecting '{rst_res.name}.i' directly as reset (assuming active-low).")

            # Connect the derived reset signal to both domains
            m.d.comb += [
                ResetSignal("sync").eq(rst_signal),
                ResetSignal("usb").eq(rst_signal)
            ]


            m.d.comb += [ ClockSignal("sync").eq(clk.i), ClockSignal("usb").eq(clk.i)]
            m.d.comb += [ ResetSignal("sync").eq(rst_signal), ResetSignal("usb").eq(rst_signal) ]

            # --- Instantiate Core Modules ---
            m.submodules.uart_handler = uart_handler = UartCommandHandler(platform=platform, baud_rate=self.BAUD_RATE, clk_freq=self.SYNC_CLK_FREQ)
            m.submodules.analyzer = analyzer = USBPassthroughAnalyzer()
            m.submodules.leds = leds = LEDController(platform, num_leds=4) # Cynthion has 6 LEDs, check indices needed

            # --- Connect Modules ---
            buttons_sync=uart_handler.o_buttons; dx_sync=uart_handler.o_dx; dy_sync=uart_handler.o_dy; cmd_ready_sync=uart_handler.o_cmd_ready
            buttons_usb=Signal(8); dx_usb=Signal(8); dy_usb=Signal(8); cmd_ready_usb=Signal()
            m.submodules += FFSynchronizer(buttons_sync, buttons_usb, o_domain="usb")
            m.submodules += FFSynchronizer(dx_sync, dx_usb, o_domain="usb")
            m.submodules += FFSynchronizer(dy_sync, dy_usb, o_domain="usb")
            m.submodules += FFSynchronizer(cmd_ready_sync, cmd_ready_usb, o_domain="usb")
            # USB domain trigger generation
            prev_cmd_ready_usb = Signal(reset_less=True); m.d.usb += prev_cmd_ready_usb.eq(cmd_ready_usb)
            inject_trigger_usb = Signal(); m.d.comb += inject_trigger_usb.eq(cmd_ready_usb & ~prev_cmd_ready_usb)
            # Connect trigger/data to analyzer
            m.d.comb += [analyzer.i_buttons.eq(buttons_usb), analyzer.i_dx.eq(dx_usb), analyzer.i_dy.eq(dy_usb), analyzer.i_inject_trigger.eq(inject_trigger_usb)]
            # USB -> Sync Synchronization for LED
            led_inject_toggle_usb = Signal();
            with m.If(inject_trigger_usb): m.d.usb += led_inject_toggle_usb.eq(~led_inject_toggle_usb)
            led_inject_toggle_sync = Signal()
            m.submodules += FFSynchronizer(led_inject_toggle_usb, led_inject_toggle_sync, o_domain="sync")
            # Connect LEDs
            m.d.comb += [
                leds.led_outputs[0].eq(led_inject_toggle_sync),       # LED 0: Toggle on command
                leds.led_outputs[1].eq(analyzer.o_host_packet_activity), # LED 1: Host activity
                leds.led_outputs[2].eq(analyzer.o_dev_packet_activity),  # LED 2: Device activity
                leds.led_outputs[3].eq(0)                              # LED 3: Unused
                # Add LEDs 4, 5 if needed
            ]
        return m

# --- Main Execution ---
if __name__ == "__main__":
    platform = get_appropriate_platform()
    print(f"Using located platform instance: {platform.name}")
    top_design = CynthionUartInjectionTop()
    top_level_cli(
        top_design,
        platform=platform,
        do_program=False if os.getenv("BUILD_LOCAL") else True,
    )
