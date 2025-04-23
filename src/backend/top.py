import os
from amaranth import Elaboratable, Module, ClockDomain, ResetSignal, Const, ClockSignal
from amaranth.build.res                  import ResourceError
from luna import top_level_cli
from luna.gateware.platform import get_appropriate_platform

from .passthrough import USBPassthroughAnalyzer
from .utils import LEDController

# --- Top-Level Module ---
# --- Top-Level Module ---
class CynthionUartInjectionTop(Elaboratable):
    """
    Top-level design connecting the integrated USB passthrough/analyzer/UART handler
    to the platform resources and LEDs.
    """
    BAUD_RATE = 115200
    # SYNC_CLK_FREQ no longer needed here, analyzer gets it from platform
    # USB_CLK_FREQ no longer needed here

    def elaborate(self, platform): # Pass platform directly
        m = Module()

        # Check if we have a platform instance
        if platform is None:
             # This mode is primarily for basic elaboration checks without hardware.
             # top_level_cli typically requires a platform.
             print("WARNING: Running in offline mode (no platform provided). Submodule behavior might differ. ", \
                   "LEDs and specific clock/reset resources are unavailable.")
             # Define local clock domains for simulation/elaboration without platform clocks
             m.domains.sync = ClockDomain(local=True)
             m.domains.usb = ClockDomain(local=True)

             # Instantiate analyzer in offline mode (it has internal simulation logic)
             # Need to pass a baud rate even in offline mode, though clk_freq might be assumed
             m.submodules.analyzer = analyzer = USBPassthroughAnalyzer(uart_baud_rate=self.BAUD_RATE)
             # Cannot instantiate LEDs without platform

        else:
             # --- Clocking and Reset ---
             # Get the default clock resource from the platform
             clk = platform.request(platform.default_clk) # Assumes default_clk exists
             clk_freq = platform.lookup(platform.default_clk).clock.frequency # Get frequency for UART

             # Define clock domains connected to the platform clock
             m.domains.sync = ClockDomain()
             m.domains.usb = ClockDomain() # Assuming USB domain runs at sync clock speed

             m.d.comb += [
                 ClockSignal("sync").eq(clk.i),
                 ClockSignal("usb").eq(clk.i) # Connect both domains to the same clock
             ]

             # Handle Reset Signal
             rst_signal = Const(0) # Default to no reset (tie low)
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
                     print("Warning: Neither 'rst#0' nor 'button_user#0' resource found. Tying reset low.")
                     # rst_signal remains Const(0)

             # Connect the derived reset signal to both domains
             # Assuming downstream logic uses synchronous, active-high reset sense
             # If ResetSignal() is used, Amaranth handles sync reset generation based on domain config
             m.d.comb += [
                 ResetSignal("sync").eq(rst_signal), # Connect same reset source
                 ResetSignal("usb").eq(rst_signal)
             ]

             # --- Instantiate Core Modules ---
             # Analyzer now handles UART internally, pass baud rate and rely on its platform access
             m.submodules.analyzer = analyzer = USBPassthroughAnalyzer(uart_baud_rate=self.BAUD_RATE)

             # LEDs - Cynthion r1.4 has 6 LEDs (0-5)
             m.submodules.leds = leds = LEDController(platform, num_leds=6)

             # --- Connect LEDs ---
             # Map analyzer status outputs to LEDs
             m.d.comb += [
                 leds.led_outputs[0].eq(analyzer.o_uart_rx_activity),    # LED 0: UART RX activity
                 leds.led_outputs[1].eq(analyzer.o_host_packet_activity),# LED 1: Host (AUX) activity
                 leds.led_outputs[2].eq(analyzer.o_dev_packet_activity), # LED 2: Device (TARGET) activity
                 leds.led_outputs[3].eq(analyzer.o_uart_tx_activity),    # LED 3: UART TX activity (if used)
                 leds.led_outputs[4].eq(0),                              # LED 4: Unused
                 leds.led_outputs[5].eq(0),                              # LED 5: Unused
             ]

        # --- Removed Obsolete Connections ---
        # The following signals and logic are no longer needed here as they are internal to analyzer:
        # - m.submodules.uart_handler = ...
        # - buttons_sync, dx_sync, dy_sync, cmd_ready_sync
        # - buttons_usb, dx_usb, dy_usb, cmd_ready_usb
        # - FFSynchronizers for the above signals
        # - inject_trigger_usb logic
        # - analyzer.i_buttons, analyzer.i_dx, analyzer.i_dy, analyzer.i_inject_trigger connections
        # - led_inject_toggle_usb/sync logic

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
