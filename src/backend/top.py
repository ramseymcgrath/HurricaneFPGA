#!/usr/bin/env python3

# -*- coding: utf-8 -*-

import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from amaranth import (
    Elaboratable,
    Module,
    ClockDomain,
    ResetSignal,
    Const,
    ClockSignal,
    Signal,
)
from amaranth.build.res import ResourceError
from luna import top_level_cli
from luna.gateware.platform import get_appropriate_platform

from src.backend.passthrough import USBDataPassthroughHandler
from src.backend.utils import LEDController
from src.backend.uart import CommandAckSystem


# --- Top-Level Design ---
class HurricaneFPGATop(Elaboratable):
    """
    Top-level design connecting the integrated USB passthrough/analyzer/UART handler
    to the platform resources and LEDs.
    """

    BAUD_RATE = 115200

    def elaborate(self, platform):  # Pass platform directly
        m = Module()

        # Check if we have a platform instance
        if platform is None:
            # This mode is primarily for basic elaboration checks without hardware.
            # top_level_cli typically requires a platform.
            print(
                "WARNING: Running in offline mode (no platform provided). Submodule behavior might differ. ",
                "LEDs and specific clock/reset resources are unavailable.",
            )
            # Define local clock domains for simulation/elaboration without platform clocks
            m.domains.sync = ClockDomain(local=True)
            m.domains.usb = ClockDomain(local=True)

            # Instantiate analyzer in offline mode (it has internal simulation logic)
            # Need to pass a baud rate even in offline mode, though clk_freq might be assumed
            m.submodules.analyzer = analyzer = USBDataPassthroughHandler(
                uart_baud_rate=self.BAUD_RATE
            )

        else:
            # --- Clocking and Reset ---
            # Get the default clock resource from the platform
            clk = platform.request(platform.default_clk)
            clk_freq = platform.lookup(
                platform.default_clk
            ).clock.frequency  # Get frequency for UART

            # Define clock domains connected to the platform clock
            m.domains.sync = ClockDomain()
            m.domains.usb = (
                ClockDomain()
            )  # Assuming USB domain runs at sync clock speed

            m.d.comb += [
                ClockSignal("sync").eq(clk.i),
                ClockSignal("usb").eq(clk.i),  # Connect both domains to the same clock
            ]

            # Handle Reset Signal
            rst_signal = Const(0)  # Default to no reset (tie low)
            try:
                # Prefer dedicated 'rst' pin
                rst_res = platform.request("rst", 0)
                print(f"Using '{rst_res.name}' as reset signal.")
                # Assuming platform 'rst' is active-low, connect directly
                rst_signal = rst_res.i
            except ResourceError:
                print("Resource 'rst#0' not found. Trying 'button_user#0' as reset.")
                try:
                    # Fallback to user button (often active-low on dev boards)
                    rst_res = platform.request("button_user", 0)
                    print(f"Using '{rst_res.name}' as reset signal.")
                    # button_user resource is often PinsN, so .i is active-low
                    rst_signal = rst_res.i
                except ResourceError:
                    print(
                        "Warning: Neither 'rst#0' nor 'button_user#0' resource found. Tying reset low."
                    )
                    # rst_signal remains Const(0)

            # Connect the derived reset signal to both domains
            # Assuming downstream logic uses synchronous, active-high reset sense
            # If ResetSignal() is used, Amaranth handles sync reset generation based on domain config
            m.d.comb += [
                ResetSignal("sync").eq(rst_signal),  # Connect same reset source
                ResetSignal("usb").eq(rst_signal),
            ]

            # --- Instantiate Core Modules ---
            m.submodules.analyzer = analyzer = USBDataPassthroughHandler(
                uart_baud_rate=self.BAUD_RATE
            )

            # LEDs - Cynthion r1.4 has 6 LEDs (0-5)
            m.submodules.leds = leds = LEDController(platform, num_leds=6)

            # Command acknowledgment system
            m.submodules.command_ack = command_ack = CommandAckSystem()

            # Handle cmd_parser (which might be None)
            cmd_parser = analyzer.get_cmd_parser()

            # If cmd_parser is available, use it; otherwise skip this functionality
            if cmd_parser is not None:
                cmd_valid = Signal()
                cmd_error = Signal()
                error_code = Signal(8)

                # Add a small FSM to detect command completion and errors
                with m.FSM(domain="sync"):
                    with m.State("IDLE"):
                        m.d.sync += [
                            cmd_valid.eq(0),
                            cmd_error.eq(0),
                            error_code.eq(command_ack.ERR_NONE),
                        ]
                        # Detect command completion from parser
                        with m.If(cmd_parser.o_cmd_ready):
                            # Check for common error conditions (could add more checks)
                            with m.If(
                                (cmd_parser.o_dx == 0)
                                & (cmd_parser.o_dy == 0)
                                & (cmd_parser.o_buttons == 0)
                            ):
                                # All zeros might be a valid "no-op" command, but we'll treat it as success
                                m.d.sync += cmd_error.eq(0)
                            with m.Elif(
                                (cmd_parser.o_dx > 127) | (cmd_parser.o_dy > 127)
                            ):
                                # Values too large - might cause unexpected behavior
                                m.d.sync += [
                                    cmd_error.eq(1),
                                    error_code.eq(command_ack.ERR_VALUE),
                                ]
                            with m.Else():
                                m.d.sync += cmd_error.eq(0)

                            m.d.sync += cmd_valid.eq(1)
                            m.next = "ACK"

                    with m.State("ACK"):
                        # One-cycle delay to ensure signals are stable
                        m.d.comb += [
                            command_ack.i_cmd_processed.eq(cmd_valid),
                            command_ack.i_cmd_error.eq(cmd_error),
                            command_ack.i_error_code.eq(error_code),
                        ]
                        m.next = "IDLE"

                # Connect acknowledgment system to UART TX but without ready signal
                m.d.comb += [
                    command_ack.i_uart_stream.valid.eq(analyzer.i_uart_tx_stream.valid),
                    command_ack.i_uart_stream.payload.eq(
                        analyzer.i_uart_tx_stream.payload
                    ),
                    command_ack.i_uart_stream.first.eq(analyzer.i_uart_tx_stream.first),
                    command_ack.i_uart_stream.last.eq(analyzer.i_uart_tx_stream.last),
                    # Removing the ready signal connection to avoid driver conflicts
                    # analyzer.i_uart_tx_stream.ready.eq(command_ack.i_uart_stream.ready)
                ]

                # Map LED 4 to handshake status if available
                m.d.comb += leds.led_outputs[4].eq(cmd_parser.o_handshake_complete)
            else:
                # Skip command parser functionality if not available
                print(
                    "Warning: Command parser not available. Command acknowledgment disabled."
                )

                # Connect acknowledgment system to UART TX but without ready signal
                m.d.comb += [
                    command_ack.i_uart_stream.valid.eq(analyzer.i_uart_tx_stream.valid),
                    command_ack.i_uart_stream.payload.eq(
                        analyzer.i_uart_tx_stream.payload
                    ),
                    command_ack.i_uart_stream.first.eq(analyzer.i_uart_tx_stream.first),
                    command_ack.i_uart_stream.last.eq(analyzer.i_uart_tx_stream.last),
                    # Removing the ready signal connection to avoid driver conflicts
                    # analyzer.i_uart_tx_stream.ready.eq(command_ack.i_uart_stream.ready)
                ]

                # Set LED 4 to always off or to another signal
                m.d.comb += leds.led_outputs[4].eq(0)  # LED 4: Off when no handshake

            # --- Connect LEDs ---
            # Map analyzer status outputs to LEDs
            m.d.comb += [
                leds.led_outputs[0].eq(
                    analyzer.o_uart_rx_activity
                ),  # LED 0: UART RX activity
                leds.led_outputs[1].eq(
                    analyzer.o_host_packet_activity
                ),  # LED 1: Host (AUX) activity
                leds.led_outputs[2].eq(
                    analyzer.o_dev_packet_activity
                ),  # LED 2: Device (TARGET) activity
                leds.led_outputs[3].eq(
                    analyzer.o_uart_tx_activity
                ),  # LED 3: UART TX activity
                # LED 4 is set above based on cmd_parser availability
                leds.led_outputs[5].eq(0),  # LED 5: Unused
            ]

            # Add connection for the final UART output stream -
            # this is now from the output of the CommandAckSystem
            # since it handles both ACK generation and normal UART output
            uart_tx_stream = command_ack.o_tx_stream

        return m


# --- Main Execution ---
if __name__ == "__main__":
    platform = get_appropriate_platform()
    print(f"Using located platform instance: {platform.name}")

    top_design = HurricaneFPGATop()
    top_level_cli(
        top_design,
        platform=platform,
        do_program=False if os.getenv("BUILD_LOCAL") else True,
    )
