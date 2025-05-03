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
    DomainRenamer,
    Signal,
)
from amaranth.build.res import ResourceError
from luna import top_level_cli
from luna.gateware.platform import get_appropriate_platform
from luna.gateware.interface.ulpi import UTMITranslator  # Keep for Control Port

from src.backend.fifo import StreamFIFO  # Corrected import
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
            # Request Target A PHY for USB HOST side (talking to real mouse)
            target_a_ulpi_res = platform.request("target_a_phy", 0)

            # Request Target C PHY for USB DEVICE side (talking to PC)
            target_c_ulpi_res = platform.request("target_c_phy", 0)

            # Set up Host Translator (for talking to real mouse)
            m.submodules.host_translator = self.host_translator = DomainRenamer("sync")(
                UTMITranslator(ulpi=target_a_ulpi_res)
            )

            # Set up Device Translator (for talking to PC)
            m.submodules.dev_translator = self.dev_translator = DomainRenamer("sync")(
                UTMITranslator(ulpi=target_c_ulpi_res)
            )

            # Simplified FIFO connection using stream interfaces
            m.submodules.fifo = fifo = StreamFIFO()  # Use StreamFIFO
            m.d.comb += [
                # Host (A) translator output connected to FIFO input
                self.host_translator.phy.connect(fifo.input),
                # FIFO output connected to Device (C) translator input
                fifo.output.connect(self.dev_translator.phy),
            ]
            # Removed outdated comment about passthrough.py handling connections

        return m


# --- Main Execution ---
if __name__ == "__main__":
    platform = get_appropriate_platform()
    print(f"Using located platform instance: {platform.name}")

    # Check if we should program the device
    should_program = not os.getenv("BUILD_LOCAL")

    if should_program:
        print("PROGRAMMING ENABLED: Device will be flashed after build")
    else:
        print("PROGRAMMING DISABLED: Only building bitstream")

    top_design = HurricaneFPGATop()

    # Use LUNA's top_level_cli which handles its own arguments
    top_level_cli(
        top_design,
        platform=platform,
        do_program=should_program,
    )
