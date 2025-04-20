#!/usr/bin/env python3

## Welcome to some hacky USB passthrough code. YMMV on if it works

# -*- coding: utf-8 -*-

import os
import amaranth
from amaranth import *
from amaranth.hdl import ClockSignal, ResetSignal, Instance, DomainRenamer, Fragment
from amaranth.lib.fifo import SyncFIFOBuffered, AsyncFIFOBuffered
from amaranth.build import Platform, Pins, Resource, Connector, Subsignal, Attrs, PinsN, Clock
from amaranth.hdl.rec import DIR_FANIN, DIR_FANOUT, Record 
from luna.gateware.platform import configure_toolchain, get_appropriate_platform

# Some janky imports
try:
    import luna
    print(f"Found LUNA version.")

    import pygreat.errors
    from luna import configure_default_logging, top_level_cli
    # Import the standard platform loader
    from luna.gateware.platform import get_appropriate_platform

    try:
        from .ulpi_core import UTMITranslator
        print("Imported UTMITranslator from local ulpi_core.py")
    except ImportError:
        try:
            from luna.gateware.interface.ulpi import UTMITranslator
            print("Imported UTMITranslator from installed LUNA package.")
        except ImportError:
            print("\nERROR: Could not find UTMITranslator class.")
            exit(1)

    # This will automatically use the LUNA_PLATFORM env var set in Dockerfile if youre using the docker image
    try:
        _PlatformClass = get_appropriate_platform()
        print(f"LUNA dynamically located platform class: {_PlatformClass}")
    except KeyError:
        print("\nERROR: LUNA couldn't find platform.")
        print("       Ensure LUNA_PLATFORM environment variable is set correctly")
        print("       (e.g., 'cynthion.gateware.platform:CynthionPlatformRev0D4')")
        print("       and the corresponding package (e.g., cynthion) is installed.")
        exit(1)
    except ImportError as e:
        print(f"\nERROR: Failed to import the platform class located by LUNA.")
        print(f"       Import error was: {e}")
        print(f"       Check the path specified in LUNA_PLATFORM and package installation.")
        exit(1)

except ImportError as e: # Outer check for LUNA itself
    print(f"Import Error: {e}")
    print("Please ensure compatible LUNA, cynthion, and amaranth versions are installed.")
    exit(1)
except Exception as e: # Outer general check
    print(f"An unexpected error occurred during LUNA import: {e}")
    exit(1)

except ImportError as e:
    print(f"Import Error: {e}")
    print("Please ensure compatible LUNA, cynthion, and amaranth versions are installed.")
    print("Try the specific installation steps provided previously.")
    exit(1)
except Exception as e:
    print(f"An unexpected error occurred during LUNA import: {e}")
    exit(1)


# --- ULPI Register Definitions (for now we don't need em)
# REG_FUNC_CTRL  = 0x04
# REG_OTG_CTRL   = 0x0A
# REG_LINE_STATE = 0x07
# LINE_STATE_SE0 = 0b00
# LINE_STATE_FS_K = 0b01
# LINE_STATE_FS_J = 0b10

# USB packet types
class USBPacketID:
    OUT   = 0b0001; IN    = 0b1001; SOF   = 0b0101; SETUP = 0b1101
    DATA0 = 0b0011; DATA1 = 0b1011; DATA2 = 0b0111; MDATA = 0b1111

# Custom Stream Interface 
class StreamInterface:
    def __init__(self, payload_width=8):
        self.valid = Signal()
        self.ready = Signal()
        self.payload = Signal(payload_width)
        
    def stream_eq(self, other, *, omit=None):
        """ Connects FORWARD signals (valid, payload, first, last) """
        if omit is None: omit = set()
        elif isinstance(omit, str): omit = {omit}
        connect_list = []
        # Connect 'valid' forward
        if 'valid' not in omit and hasattr(self, 'valid') and hasattr(other, 'valid'):
            connect_list.append(other.valid.eq(self.valid))
        # Connect payload forward
        if 'payload' not in omit and hasattr(self, 'payload') and hasattr(other, 'payload'):
             connect_list.append(other.payload.eq(self.payload))
        # Connect first/last forward
        if 'first' not in omit and hasattr(self, 'first') and hasattr(other, 'first'):
             connect_list.append(other.first.eq(self.first))
        if 'last' not in omit and hasattr(self, 'last') and hasattr(other, 'last'):
             connect_list.append(other.last.eq(self.last))
        return connect_list

class USBInStreamInterface(StreamInterface):
    def __init__(self, payload_width=8): 
        super().__init__(payload_width)
        self.first = Signal()
        self.last = Signal()

class USBOutStreamInterface(StreamInterface):
    def __init__(self, payload_width=8): 
        super().__init__(payload_width)
        self.first = Signal()
        self.last = Signal()

# Simplified Boundary Detector
class USBOutStreamBoundaryDetector(Elaboratable):
    # TODO: Still uses simplified heuristic. Real boundaries depend on EOP.
    def __init__(self, domain="usb"):
        self.domain = domain; self.unprocessed_stream = USBOutStreamInterface(); self.processed_stream = USBOutStreamInterface()
    def elaborate(self, platform):
        m = Module(); unprocessed=self.unprocessed_stream; processed=self.processed_stream
        payload=Signal.like(processed.payload); first=Signal(); last=Signal(); valid=Signal()
        m.d.comb += [ processed.payload.eq(payload), processed.first.eq(first), processed.last.eq(last), processed.valid.eq(valid), unprocessed.ready.eq(processed.ready) ]
        active_packet=Signal(reset=0); m.d.comb += [ valid.eq(unprocessed.valid), payload.eq(unprocessed.payload), first.eq(0), last.eq(0) ]
        with m.If(unprocessed.valid & ~active_packet):
            m.d.comb += first.eq(1)
        # Simplified last detection - payload == 0 heuristic
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
def synchronize(m, signal, o_domain, stages=2, name=None):
    sync_stages=[]; last_stage=signal
    for i in range(stages):
        sig_name=f"{name}_stage{i}" if name else None; stage=Signal.like(signal, name=sig_name, reset_less=True)
        m.d[o_domain]+=stage.eq(last_stage); sync_stages.append(stage); last_stage=stage
    return sync_stages[-1]

# Simple Mouse Packet Injector
class SimpleMouseInjector(Elaboratable):
    def __init__(self):
        self.source=USBInStreamInterface(payload_width=8); self.trigger=Signal()
        self.buttons=Signal(8); self.dx=Signal(8); self.dy=Signal(8); self._data_pid=Signal(4, reset=USBPacketID.DATA1)
    def elaborate(self, platform):
        m = Module(); source, buttons, dx, dy = self.source, self.buttons, self.dx, self.dy
        latched_buttons=Signal.like(buttons); latched_dx=Signal.like(dx); latched_dy=Signal.like(dy)
        m.d.comb += [ source.valid.eq(0), source.first.eq(0), source.last.eq(0), source.payload.eq(0) ]
        # Whole bunch of signals to hold latched values, this could be cleaner
        with m.FSM(domain="usb", name="injector_fsm"):
            with m.State("IDLE"):
                with m.If(self.trigger):
                    m.d.usb += [ latched_buttons.eq(buttons), latched_dx.eq(dx), latched_dy.eq(dy) ]; m.next = "SEND_PID"
            with m.State("SEND_PID"):
                m.d.comb += [ source.valid.eq(1), source.payload.eq(self._data_pid), source.first.eq(1), source.last.eq(0) ]
                with m.If(source.ready):
                    m.next = "SEND_BYTE_0"
            with m.State("SEND_BYTE_0"):
                m.d.comb += [ source.valid.eq(1), source.payload.eq(latched_buttons), source.first.eq(0), source.last.eq(0) ]
                with m.If(source.ready):
                    m.next = "SEND_BYTE_1"
            with m.State("SEND_BYTE_1"):
                m.d.comb += [ source.valid.eq(1), source.payload.eq(latched_dx), source.first.eq(0), source.last.eq(0) ]
                with m.If(source.ready):
                    m.next = "SEND_BYTE_2"
            with m.State("SEND_BYTE_2"):
                m.d.comb += [ source.valid.eq(1), source.payload.eq(latched_dy), source.first.eq(0), source.last.eq(1) ]
                with m.If(source.ready):
                    m.next = "IDLE"
        return m
    
class PacketArbiter(Elaboratable):
    def __init__(self):
        self.passthrough_in=USBInStreamInterface(payload_width=8)
        self.inject_in=USBInStreamInterface(payload_width=8)
        self.merged_out=USBInStreamInterface(payload_width=8)

    def elaborate(self, platform):
        m = Module()
        passthrough, inject, merged = self.passthrough_in, self.inject_in, self.merged_out

        # Default assignments for the output stream and input readys
        m.d.comb += [
            merged.valid.eq(0),
            merged.payload.eq(0), # Default all output signals
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
                # --- MODIFIED: Explicit assignments instead of stream_eq ---
                m.d.comb += [
                    merged.valid.eq(passthrough.valid),
                    merged.payload.eq(passthrough.payload),
                    merged.first.eq(passthrough.first),
                    merged.last.eq(passthrough.last),
                    passthrough.ready.eq(merged.ready) # Connect ready back
                ]
                # --- End Modification ---

                # Transition logic
                with m.If(passthrough.valid & passthrough.last & merged.ready):
                    m.next = "IDLE"
                # If the source disappears mid-packet (optional, depends on desired behavior)
                with m.Elif(~passthrough.valid):
                     m.next = "IDLE"


            with m.State("FORWARD_INJECT"):
                # --- MODIFIED: Explicit assignments instead of stream_eq ---
                m.d.comb += [
                    merged.valid.eq(inject.valid),
                    merged.payload.eq(inject.payload),
                    merged.first.eq(inject.first),
                    merged.last.eq(inject.last),
                    inject.ready.eq(merged.ready) # Connect ready back
                ]
                # --- End Modification ---

                # Transition logic
                with m.If(inject.valid & inject.last & merged.ready):
                    m.next = "IDLE"
                # If the source disappears mid-packet
                with m.Elif(~inject.valid):
                     m.next = "IDLE"

        return m


# UART
class Parity: NONE, ODD, EVEN = range(3)
class StopBits: ONE, TWO = range(2)
class UARTRxInterface:
    def __init__(self, data_bits=8): self.data=Signal(data_bits); self.rdy=Signal(); self.ack=Signal(); self.err=Signal()
class UARTTxInterface:
    def __init__(self, data_bits=8): self.data=Signal(data_bits); self.req=Signal(); self.ack=Signal() # ack might not be used
class AsyncUART(Elaboratable):
    def __init__(self, *, divisor, pins, data_bits=8, parity=Parity.NONE, stop_bits=StopBits.ONE):
        self.divisor=divisor; self.rx_pin=pins.rx; self.tx_pin=pins.tx; self.data_bits=data_bits
        self.rx=UARTRxInterface(data_bits=data_bits); self.tx=UARTTxInterface(data_bits=data_bits)

    def elaborate(self, platform):
        m = Module()
        rx_shiftreg=Signal(self.data_bits+2,reset=-1) # Includes Start (index -1) and Stop (index 0) bits
        rx_counter=Signal(range(self.divisor))
        rx_bit_counter=Signal(range(len(rx_shiftreg))) # Counts received bits including start/stop

        rx_rdy_internal=Signal()
        rx_err_internal=Signal()
        rx_data_internal=Signal.like(self.rx.data)

        m.d.comb+=[
            self.rx.rdy.eq(rx_rdy_internal),
            self.rx.err.eq(rx_err_internal),
            self.rx.data.eq(rx_data_internal)
        ]

        with m.FSM(domain="sync", name="uart_rx_fsm_core"):

            with m.State("IDLE"):
                with m.If(~self.rx.ack):
                    m.d.sync += [
                        rx_rdy_internal.eq(0),
                        rx_err_internal.eq(0)
                    ]
                with m.If(~self.rx_pin):
                    m.d.sync += [
                        rx_counter.eq(self.divisor // 2 - 1), # Sample mid-bit
                        rx_bit_counter.eq(0),
                        rx_shiftreg.eq(-1) # Reset to all 1s
                    ]
                    m.next = "START"

            with m.State("START"):
                # Wait for middle of start bit time
                m.d.sync += rx_counter.eq(rx_counter - 1)
                with m.If(rx_counter == 0):
                    m.d.sync += rx_counter.eq(self.divisor - 1) # Reload for first data bit sampling
                    # Re-check if it's still low (valid start bit)
                    with m.If(~self.rx_pin):
                        m.d.sync += [
                             rx_bit_counter.eq(1), # Start bit is the 0th bit received
                             rx_shiftreg[-1].eq(0) # Store '0' for start bit
                        ]
                        m.next = "DATA"
                    with m.Else(): # Glitch, false start
                        m.next = "IDLE"

            with m.State("DATA"):
                # Wait for middle of data bit time
                m.d.sync += rx_counter.eq(rx_counter - 1)
                with m.If(rx_counter == 0):
                    m.d.sync += rx_counter.eq(self.divisor - 1) # Reload for next bit
                    # Shift in the received bit
                    m.d.sync += rx_shiftreg.eq(Cat(self.rx_pin, rx_shiftreg[:-1]))
                    m.d.sync += rx_bit_counter.eq(rx_bit_counter + 1)
                    # Check if all bits (Start + Data + Stop) have been shifted
                    with m.If(rx_bit_counter == (len(rx_shiftreg) - 1)):
                        m.next = "STOP"

            with m.State("STOP"):
                # Data is now in rx_shiftreg[1:-1]
                # Stop bit is at rx_shiftreg[0], Start bit at rx_shiftreg[-1]
                is_start_ok = (rx_shiftreg[-1] == 0)
                is_stop_ok = (rx_shiftreg[0] == 1)

                # Check framing and set flags/data if OK
                with m.If(is_start_ok & is_stop_ok):
                    with m.If(~self.rx.ack):
                        m.d.sync += [
                            rx_data_internal.eq(rx_shiftreg[1:-1]), # Assign data
                            rx_rdy_internal.eq(1),                  # Signal data ready
                            rx_err_internal.eq(0)                   # Clear error
                        ]
                    # If acked this cycle, ensure ready goes low
                    with m.Else():
                         m.d.sync += rx_rdy_internal.eq(0)
                with m.Else(): # Framing error
                    m.d.sync += rx_err_internal.eq(1)
                    # If acked this cycle, still clear ready
                    with m.If(self.rx.ack):
                        m.d.sync += rx_rdy_internal.eq(0)

                # Transition back to IDLE once acknowledged or if line goes idle (allowing next byte)
                # Or immediately on framing error to potentially sync on next start bit faster
                with m.If(self.rx.ack | rx_err_internal): # Go idle if acked or framing error
                    m.next = "IDLE"
                # Also go idle if still ready (not acked yet) but line goes idle?
                # This requires holding 'rdy' state.
                # FSM stays in STOP until ack is received (or error occurred).

        # TX is basic bit-banging
        m.d.comb += self.tx_pin.eq(1) # Default idle high
        ack_pulse_timer = Signal(4, reset=0)
        with m.If(self.tx.req & (ack_pulse_timer == 0)):
            m.d.sync += ack_pulse_timer.eq(15) # Load timer on request
        with m.If(ack_pulse_timer != 0):
            m.d.comb += self.tx_pin.eq(0) # Drive low during pulse
            m.d.sync += ack_pulse_timer.eq(ack_pulse_timer - 1) # Decrement timer

        return m

class UARTCommandHandler(Elaboratable):
    def __init__(self, *, uart_pins, baud_rate=115200, clk_freq=60_000_000):
        self._pins=uart_pins; self._baud=baud_rate; self._clk_freq=clk_freq
        self.o_buttons=Signal(8); self.o_dx=Signal(8); self.o_dy=Signal(8); self.o_cmd_ready=Signal()
    def elaborate(self, platform):
        m = Module(); uart_divisor=int(self._clk_freq//self._baud)
        m.submodules.uart=uart=AsyncUART(divisor=uart_divisor,pins=self._pins)
        temp_buttons=Signal(8); temp_dx=Signal(8); m.d.comb+=[self.o_cmd_ready.eq(0),uart.rx.ack.eq(0),uart.tx.req.eq(0)]
        with m.FSM(domain="sync", name="uart_rx_fsm"):
            with m.State("IDLE"):
                with m.If(uart.rx.rdy & ~uart.rx.err): m.d.sync+=temp_buttons.eq(uart.rx.data); m.d.comb+=uart.rx.ack.eq(1); m.next="WAIT_DX";
                with m.Elif(uart.rx.rdy & uart.rx.err): m.d.comb+=uart.rx.ack.eq(1)
            with m.State("WAIT_DX"):
                with m.If(uart.rx.rdy & ~uart.rx.err): m.d.sync+=temp_dx.eq(uart.rx.data); m.d.comb+=uart.rx.ack.eq(1); m.next="WAIT_DY";
                with m.Elif(uart.rx.rdy & uart.rx.err): m.d.comb+=uart.rx.ack.eq(1); m.next="IDLE"
            with m.State("WAIT_DY"):
                with m.If(uart.rx.rdy & ~uart.rx.err): m.d.sync+=[self.o_buttons.eq(temp_buttons),self.o_dx.eq(temp_dx),self.o_dy.eq(uart.rx.data)]; m.d.comb+=[self.o_cmd_ready.eq(1),uart.rx.ack.eq(1),uart.tx.req.eq(1)]; m.next="IDLE";
                with m.Elif(uart.rx.rdy & uart.rx.err): m.d.comb+=uart.rx.ack.eq(1); m.next="IDLE"
        return m
    
# Old Analyzer code
class USBPassthroughAnalyzer(Elaboratable):
    def __init__(self):
        # Control signals now mostly inputs to UTMITranslator
        self.i_inject_trigger = Signal()
        self.i_buttons = Signal(8)
        self.i_dx = Signal(8)
        self.i_dy = Signal(8)

    def elaborate(self, platform):
        m = Module()

        if platform is None:
            # --- Offline Mode ---
            print("USBPassthroughAnalyzer: Running in offline mode.")
            # Define dummy layout for offline synthesis/simulation
            ulpi_bus_layout = [
                # Bidirectional data signal
                ("data", [("i", 8, DIR_FANIN), ("o", 8, DIR_FANOUT), ("oe", 1, DIR_FANOUT)]),
                # Unidirectional signals defined with explicit direction subsignal
                ("clk",  [("o", 1, DIR_FANOUT)]),  # Clock out from FPGA
                ("dir",  [("i", 1, DIR_FANIN)]),   # Direction in to FPGA
                ("nxt",  [("i", 1, DIR_FANIN)]),   # Next in to FPGA
                ("stp",  [("o", 1, DIR_FANOUT)]),  # Stop out from FPGA
                ("rst",  [("o", 1, DIR_FANOUT)])   # Reset out from FPGA
            ]
            target_ulpi_res = Record(ulpi_bus_layout, name="offline_target_ulpi")
            control_ulpi_res = Record(ulpi_bus_layout, name="offline_control_ulpi")
        else:

            target_ulpi_res = platform.request('target_phy', 0)  # J2 -> Host PC side
            control_ulpi_res = platform.request('control_phy', 0) # J3 -> Device side

        m.submodules.host_translator = host_translator = DomainRenamer("sync")(UTMITranslator(ulpi=target_ulpi_res, handle_clocking=False))
        m.submodules.dev_translator = dev_translator = DomainRenamer("sync")(UTMITranslator(ulpi=control_ulpi_res, handle_clocking=False))
        if platform is not None: # Only connect in hardware mode
            m.d.comb += [
                target_ulpi_res.clk.o.eq(ClockSignal("sync")),
                control_ulpi_res.clk.o.eq(ClockSignal("sync")),
            ]

        m.d.comb += [
            host_translator.op_mode.eq(0b00), host_translator.xcvr_select.eq(0b01), host_translator.term_select.eq(1), host_translator.suspend.eq(0), host_translator.dp_pulldown.eq(0), host_translator.dm_pulldown.eq(0),
            dev_translator.op_mode.eq(0b00), dev_translator.xcvr_select.eq(0b01), dev_translator.term_select.eq(1), dev_translator.suspend.eq(0), dev_translator.dp_pulldown.eq(0), dev_translator.dm_pulldown.eq(0),
        ]

        fifo_depth, fifo_width = 16, 10
        m.submodules.host_rx_fifo=host_rx_fifo=AsyncFIFOBuffered(width=fifo_width,depth=fifo_depth,w_domain="sync",r_domain="usb")
        m.submodules.dev_rx_fifo=dev_rx_fifo=AsyncFIFOBuffered(width=fifo_width,depth=fifo_depth,w_domain="sync",r_domain="usb")
        m.submodules.host_tx_fifo=host_tx_fifo=AsyncFIFOBuffered(width=fifo_width,depth=fifo_depth,w_domain="usb",r_domain="sync")
        m.submodules.dev_tx_fifo=dev_tx_fifo=AsyncFIFOBuffered(width=fifo_width,depth=fifo_depth,w_domain="usb",r_domain="sync")
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

        dev_rx_first = Signal()
        dev_rx_last = Signal()
        dev_prev_rx_active = Signal(reset_less=True)
        m.d.sync += dev_prev_rx_active.eq(dev_translator.rx_active)
        m.d.comb += dev_rx_first.eq(dev_translator.rx_valid & (dev_translator.rx_active & ~dev_prev_rx_active))
        # CORRECTED: 'last' occurs on falling edge of rx_active
        m.d.comb += dev_rx_last.eq(~dev_translator.rx_active & dev_prev_rx_active)
        m.d.comb += [
            dev_rx_fifo.w_en.eq(dev_translator.rx_valid),
            dev_rx_fifo.w_data.eq(Cat(dev_translator.rx_data, dev_rx_first, dev_rx_last)),
        ]
        host_rx_fifo_out=USBOutStreamInterface(payload_width=8); dev_rx_fifo_out=USBOutStreamInterface(payload_width=8); m.d.comb+=[host_rx_fifo_out.valid.eq(host_rx_fifo.r_rdy),host_rx_fifo.r_en.eq(host_rx_fifo_out.ready),Cat(host_rx_fifo_out.payload,host_rx_fifo_out.first,host_rx_fifo_out.last).eq(host_rx_fifo.r_data), dev_rx_fifo_out.valid.eq(dev_rx_fifo.r_rdy),dev_rx_fifo.r_en.eq(dev_rx_fifo_out.ready),Cat(dev_rx_fifo_out.payload,dev_rx_fifo_out.first,dev_rx_fifo_out.last).eq(dev_rx_fifo.r_data),]
        m.submodules.host_rx_boundary=host_rx_boundary=USBOutStreamBoundaryDetector(domain="usb"); m.submodules.dev_rx_boundary=dev_rx_boundary=USBOutStreamBoundaryDetector(domain="usb"); m.d.comb += host_rx_fifo_out.stream_eq(host_rx_boundary.unprocessed_stream); m.d.comb += dev_rx_fifo_out.stream_eq(dev_rx_boundary.unprocessed_stream)
        host_logic_out_stream=host_rx_boundary.processed_stream; m.d.comb+=[dev_tx_fifo.w_en.eq(host_logic_out_stream.valid),dev_tx_fifo.w_data.eq(Cat(host_logic_out_stream.payload,host_logic_out_stream.first,host_logic_out_stream.last)),host_logic_out_stream.ready.eq(dev_tx_fifo.w_rdy),]
        m.submodules.injector=injector=SimpleMouseInjector(); m.submodules.arbiter=arbiter=PacketArbiter(); m.d.comb+=[injector.trigger.eq(self.i_inject_trigger),injector.buttons.eq(self.i_buttons),injector.dx.eq(self.i_dx),injector.dy.eq(self.i_dy)]
        dev_logic_out_stream=dev_rx_boundary.processed_stream; m.d.comb += dev_logic_out_stream.stream_eq(arbiter.passthrough_in); m.d.comb += injector.source.stream_eq(arbiter.inject_in) 
        arbiter_out_stream=arbiter.merged_out; m.d.comb+=[host_tx_fifo.w_en.eq(arbiter_out_stream.valid),host_tx_fifo.w_data.eq(Cat(arbiter_out_stream.payload,arbiter_out_stream.first,arbiter_out_stream.last)),arbiter_out_stream.ready.eq(host_tx_fifo.w_rdy),]
        host_tx_fifo_data=Signal(10); host_tx_payload=Signal(8); host_tx_first=Signal(); host_tx_last=Signal(); m.d.comb+=[host_tx_fifo_data.eq(host_tx_fifo.r_data),host_tx_payload.eq(host_tx_fifo_data[0:8]),host_tx_first.eq(host_tx_fifo_data[8]),host_tx_last.eq(host_tx_fifo_data[9])]
        m.d.comb+=[host_translator.tx_data.eq(host_tx_payload), host_translator.tx_valid.eq(host_tx_fifo.r_rdy), host_tx_fifo.r_en.eq(host_translator.tx_ready)]
        dev_tx_fifo_data=Signal(10); dev_tx_payload=Signal(8); dev_tx_first=Signal(); dev_tx_last=Signal(); m.d.comb+=[dev_tx_fifo_data.eq(dev_tx_fifo.r_data),dev_tx_payload.eq(dev_tx_fifo_data[0:8]),dev_tx_first.eq(dev_tx_fifo_data[8]),dev_tx_last.eq(dev_tx_fifo_data[9])]
        m.d.comb+=[dev_translator.tx_data.eq(dev_tx_payload), dev_translator.tx_valid.eq(dev_tx_fifo.r_rdy), dev_tx_fifo.r_en.eq(dev_translator.tx_ready)]
        return m

class CynthionUartInjectionTop(Elaboratable):
    BAUD_RATE = 115200; SYNC_CLK_FREQ = 60_000_000; USB_CLK_FREQ = 48_000_000
    def elaborate(self, platform: Platform):
        m = Module()
        if platform is None:
            print("Running in offline mode..."); m.domains.sync=ClockDomain(local=True); m.domains.usb=ClockDomain(local=True); m.submodules.analyzer = USBPassthroughAnalyzer()
        else:
            clk100_resource=platform.request(platform.default_clk); clk100_input_signal=Signal(name="clk100_input")
            try: m.d.comb+=clk100_input_signal.eq(clk100_resource)
            except TypeError:
                pin_signal=None
                if hasattr(clk100_resource,'i'): pin_signal=clk100_resource.i
                elif hasattr(clk100_resource,'p'): pin_signal=clk100_resource.p
                if pin_signal is not None: m.d.comb+=clk100_input_signal.eq(pin_signal)
                else: raise TypeError(f"Clock resource {platform.default_clk} not connectable.")
            m.domains.sync=ClockDomain(); m.domains.usb=ClockDomain()

            # PLL Instantiation
            pll_locked=Signal(); clk_sync_unbuf=Signal(name="clk_sync_unbuf"); clk_usb_unbuf=Signal(name="clk_usb_unbuf"); clk_feedback=Signal(name="pll_clk_feedback")
            m.submodules.pll = Instance("EHXPLLL",
                i_CLKI=clk100_input_signal, i_CLKFB=clk_feedback, o_CLKOP=clk_feedback,
                o_CLKOS=clk_usb_unbuf, o_CLKOS2=clk_sync_unbuf, o_LOCK=pll_locked,
                p_CLKI_DIV=5, p_CLKOP_DIV=1, p_CLKFB_DIV=24, p_CLKOS_DIV=10, p_CLKOS2_DIV=8,
                p_FEEDBK_PATH="CLKOP", p_CLKOP_ENABLE="ENABLED", p_CLKOS_ENABLE="ENABLED", p_CLKOS2_ENABLE="ENABLED",
            )
            m.d.comb += ClockSignal("sync").eq(clk_sync_unbuf); m.d.comb += ClockSignal("usb").eq(clk_usb_unbuf)

            # Reset Generation
            reset_sig=Signal(name="pll_reset"); m.d.comb+=reset_sig.eq(~pll_locked)
            m.d.comb+=[ResetSignal("sync").eq(reset_sig), ResetSignal("usb").eq(reset_sig)]

            # UART Handler
            uart_resource_name="uart"; uart_resource_index=0; uart_pins_rec=Record([('rx',1),('tx',1)])
            try:
                uart_io=platform.request(uart_resource_name,uart_resource_index); m.d.comb+=[uart_pins_rec.rx.eq(uart_io.rx.i),uart_io.tx.o.eq(uart_pins_rec.tx)]; print(f"Using UART resource: {uart_resource_name}_{uart_resource_index}")
            except amaranth.build.ResourceError:
                uart_resource_name="pmod"; uart_resource_index=0; uart_rx_pin_index=0; uart_tx_pin_index=1; print(f"WARNING: UART resource not found. Falling back to PMOD {uart_resource_index}...")
                try: pmod_io=platform.request(uart_resource_name,uart_resource_index); m.d.comb+=[uart_pins_rec.rx.eq(pmod_io.io[uart_rx_pin_index].i),pmod_io.io[uart_tx_pin_index].o.eq(uart_pins_rec.tx),]
                except amaranth.build.ResourceError as e: print(f"\n*** Resource Error: Cannot request PMOD UART pins. Check platform file. ***\nError: {e}"); raise
            m.submodules.uart_handler=uart_handler=UARTCommandHandler(uart_pins=uart_pins_rec,baud_rate=self.BAUD_RATE,clk_freq=self.SYNC_CLK_FREQ)


            sync_buttons=synchronize(m,uart_handler.o_buttons,o_domain="usb",stages=3,name="sync_buttons"); sync_dx=synchronize(m,uart_handler.o_dx,o_domain="usb",stages=3,name="sync_dx"); sync_dy=synchronize(m,uart_handler.o_dy,o_domain="usb",stages=3,name="sync_dy")
            cmd_ready_sync_cdc=synchronize(m,uart_handler.o_cmd_ready,o_domain="usb",stages=3,name="cmd_ready_sync"); prev_cmd_ready=Signal(reset_less=True); inject_trigger=Signal()
            m.d.usb+=prev_cmd_ready.eq(cmd_ready_sync_cdc); m.d.comb+=inject_trigger.eq(cmd_ready_sync_cdc & ~prev_cmd_ready)
            m.submodules.analyzer=analyzer=USBPassthroughAnalyzer(); m.d.comb+=[analyzer.i_buttons.eq(sync_buttons),analyzer.i_dx.eq(sync_dx),analyzer.i_dy.eq(sync_dy),analyzer.i_inject_trigger.eq(inject_trigger)]

        # Some LED display
            try:
               led_pin=platform.request("led",0); led_state=Signal()
               with m.If(inject_trigger):
                   m.d.usb+=led_state.eq(~led_state)
               m.d.comb+=led_pin.o.eq(led_state)
            except (amaranth.build.ResourceError,AttributeError,IndexError): print("Note: LED feedback disabled."); pass
        return m

if __name__ == "__main__":
    _PlatformClass = get_appropriate_platform()
    platform_instance = _PlatformClass
    print(f"Using located platform instance: {platform_instance.name}") # Info for user
    print("\nAttempting offline elaboration check...")
    if build_local := os.getenv("BUILD_LOCAL") == "1":
        try:
            top_design = CynthionUartInjectionTop()
            print("Elaborating design with platform=None...")
            fragment = Fragment.get(top_design, platform=None)
            print("Offline elaboration successful.")

            # You can generate the legit verilog if you really want
            # print("Generating Verilog from offline fragment...")
            # from amaranth.back import verilog
            # from pathlib import Path
            # build_dir = Path("./build_offline") # Use a specific directory
            # build_dir.mkdir(parents=True, exist_ok=True)
            # verilog_file = build_dir / "top_offline.v"
            # with open(verilog_file, "w") as f:
            #     # Note: Automatically determining ports for verilog.convert can be tricky.
            #     # You might need to manually list top-level ports if needed.
            #     f.write(verilog.convert(fragment, name="top_offline")) # Removed ports= for simplicity
            # print(f"Offline Verilog potentially written to: {verilog_file}")

        except Exception as e:
            print("\nERROR during offline elaboration:")
            import traceback
            traceback.print_exc()
            exit(1)
    else:
        # --- Hardware Build (using luna.top_level_cli) ---
        # TODO: Uncomment and adjust the following lines for hardware build. 
        # python is all screwed up on my laptop so for now its just the docker bs
        print("\nInitiating hardware build...")
        try:
            top_level_cli(
                CynthionUartInjectionTop(),
                platform=platform_instance, # Pass the ACTUAL hardware platform instance
                name="top_hw",              # Different name/build dir recommended
                # toolchain=platform_instance.toolchain, # Let LUNA handle toolchain detection
                build_dir="build_hw",
                # output="build.bit" # LUNA handles default output name
                # add --upload argument to the docker run command if needed
            )
            print("\nHardware build process completed or Cynthion programmer launched.")
            print("Check 'build_hw/' directory and programmer output.")
        except Exception as e:
            print("\nERROR during hardware build:")
            import traceback
            traceback.print_exc()
            exit(1)
