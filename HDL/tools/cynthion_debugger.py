#!/usr/bin/env python3
# =====================================================
# cynthion_debugger.py - Cynthion USB Sniffer Debugging Tool
# =====================================================
#
# This script provides a command-line interface for interacting with
# the debugging interface of the Cynthion USB sniffer.
#
# Dependencies:
#   - pyusb (pip install pyusb)
#   - libusb (or similar backend)
#

import sys
import time
import argparse
import usb.core
import usb.util
from enum import Enum
from typing import List, Tuple, Optional, Dict, Any


# Command definitions - must match debug_interface.v
class DebugCommand(Enum):
    CMD_NOP = 0x00
    CMD_GET_STATUS = 0x01
    CMD_GET_BUFFER_STATUS = 0x02
    CMD_GET_PACKET_COUNT = 0x03
    CMD_GET_ERROR_COUNT = 0x04
    CMD_GET_LINE_STATE = 0x05
    CMD_GET_TIMESTAMP = 0x06
    CMD_SET_DEBUG_LEDS = 0x10
    CMD_SET_DEBUG_PROBE = 0x11
    CMD_SET_DEBUG_MODE = 0x12
    CMD_FORCE_RESET = 0x20
    CMD_LOOPBACK_ENABLE = 0x21
    CMD_TRIGGER_CONFIG = 0x22
    CMD_VERSION = 0xF0


# USB device parameters
CYNTHION_VID = 0x1D50  # Assume OpenMoko VID used for Cynthion
CYNTHION_PID = 0x614B  # Arbitrary PID - adjust to match actual device
DEBUG_INTERFACE = 1  # Interface number for debug control
DEBUG_ENDPOINT_OUT = 1  # Endpoint for sending commands
DEBUG_ENDPOINT_IN = 0x81  # Endpoint for receiving responses


class CynthionDebugger:
    def __init__(
        self, vid: int = CYNTHION_VID, pid: int = CYNTHION_PID, verbose: bool = False
    ):
        self.verbose = verbose
        self.dev = None
        self.connected = False
        self.vid = vid
        self.pid = pid
        self.timeout = 1000  # 1 second timeout for USB operations

    def log(self, message: str) -> None:
        """Print message if verbose mode is enabled"""
        if self.verbose:
            print(f"[DEBUG] {message}")

    def info(self, message: str) -> None:
        """Print informational message"""
        print(f"[INFO] {message}")

    def error(self, message: str) -> None:
        """Print error message"""
        print(f"[ERROR] {message}", file=sys.stderr)

    def connect(self) -> bool:
        """Connect to the Cynthion device"""
        try:
            # Find the Cynthion device
            self.dev = usb.core.find(idVendor=self.vid, idProduct=self.pid)

            if self.dev is None:
                self.error(
                    f"Cynthion device (VID=0x{self.vid:04x}, PID=0x{self.pid:04x}) not found"
                )
                return False

            self.log(f"Found Cynthion device: {self.dev}")

            # Detach kernel driver if necessary (Linux)
            try:
                if self.dev.is_kernel_driver_active(DEBUG_INTERFACE):
                    self.log("Detaching kernel driver")
                    self.dev.detach_kernel_driver(DEBUG_INTERFACE)
            except (AttributeError, usb.core.USBError):
                self.log("Could not check/detach kernel driver")

            # Set configuration and claim interface
            try:
                self.dev.set_configuration()
                usb.util.claim_interface(self.dev, DEBUG_INTERFACE)
            except usb.core.USBError as e:
                self.error(f"Could not set configuration or claim interface: {e}")
                return False

            self.connected = True
            self.info("Connected to Cynthion device")
            return True

        except usb.core.USBError as e:
            self.error(f"USB Error during connection: {e}")
            return False

    def disconnect(self) -> None:
        """Disconnect from the Cynthion device"""
        if self.dev and self.connected:
            try:
                usb.util.release_interface(self.dev, DEBUG_INTERFACE)
                # Reattach kernel driver if needed (Linux)
                try:
                    self.dev.attach_kernel_driver(DEBUG_INTERFACE)
                except (AttributeError, usb.core.USBError):
                    pass
            except usb.core.USBError as e:
                self.log(f"Error during disconnect: {e}")

            self.connected = False
            self.dev = None
            self.info("Disconnected from Cynthion device")

    def send_command(self, command: DebugCommand, param: int = 0) -> bool:
        """Send a command to the debug interface"""
        if not self.connected or not self.dev:
            self.error("Not connected to device")
            return False

        try:
            cmd_byte = command.value
            cmd_data = bytes([cmd_byte, param])

            self.log(f"Sending command: 0x{cmd_byte:02x}, param: 0x{param:02x}")
            bytes_written = self.dev.write(
                DEBUG_ENDPOINT_OUT, cmd_data, timeout=self.timeout
            )

            if bytes_written != len(cmd_data):
                self.error(
                    f"Failed to send complete command: {bytes_written}/{len(cmd_data)} bytes sent"
                )
                return False

            return True

        except usb.core.USBError as e:
            self.error(f"USB Error sending command: {e}")
            return False

    def read_response(self, expected_length: int = 1) -> Optional[bytearray]:
        """Read response from the debug interface"""
        if not self.connected or not self.dev:
            self.error("Not connected to device")
            return None

        try:
            self.log(f"Reading response, expecting {expected_length} bytes")
            response = self.dev.read(
                DEBUG_ENDPOINT_IN, expected_length, timeout=self.timeout
            )

            if len(response) > 0:
                self.log(
                    f"Received {len(response)} bytes: {' '.join([f'0x{b:02x}' for b in response])}"
                )
                return response
            else:
                self.error("Got empty response")
                return None

        except usb.core.USBTimeoutError:
            self.error("Timeout reading response")
            return None
        except usb.core.USBError as e:
            self.error(f"USB Error reading response: {e}")
            return None

    def execute_command(
        self, command: DebugCommand, param: int = 0, resp_len: int = 1
    ) -> Optional[bytearray]:
        """Execute a command and return the response"""
        if not self.send_command(command, param):
            return None

        # Short sleep to give the device time to process the command
        time.sleep(0.01)

        return self.read_response(resp_len)

    def get_version(self) -> Optional[Tuple[int, int, int]]:
        """Get firmware version from device"""
        resp = self.execute_command(DebugCommand.CMD_VERSION, 0, 4)
        if resp and len(resp) >= 4:
            return (resp[1], resp[2], resp[3])  # Major, minor, patch
        return None

    def get_status(self) -> Optional[Dict[str, Any]]:
        """Get device status"""
        resp = self.execute_command(DebugCommand.CMD_GET_STATUS, 0, 4)
        if resp and len(resp) >= 4:
            status = {
                "proxy_active": bool(resp[1] & 0x08),
                "host_connected": bool(resp[1] & 0x04),
                "device_connected": bool(resp[1] & 0x02),
                "host_speed": (resp[2] >> 2) & 0x03,
                "device_speed": resp[2] & 0x03,
                "buffer_overflow": bool(resp[3] & 0x01),
            }
            return status
        return None

    def get_buffer_status(self) -> Optional[int]:
        """Get buffer usage"""
        resp = self.execute_command(DebugCommand.CMD_GET_BUFFER_STATUS, 0, 3)
        if resp and len(resp) >= 3:
            return (resp[2] << 8) | resp[1]  # 16-bit value
        return None

    def get_packet_count(self) -> Optional[int]:
        """Get packet counter value"""
        resp = self.execute_command(DebugCommand.CMD_GET_PACKET_COUNT, 0, 5)
        if resp and len(resp) >= 5:
            return (resp[4] << 24) | (resp[3] << 16) | (resp[2] << 8) | resp[1]
        return None

    def get_error_count(self) -> Optional[int]:
        """Get error counter value"""
        resp = self.execute_command(DebugCommand.CMD_GET_ERROR_COUNT, 0, 3)
        if resp and len(resp) >= 3:
            return (resp[2] << 8) | resp[1]  # 16-bit value
        return None

    def get_line_state(self) -> Optional[Dict[str, str]]:
        """Get current USB line states"""
        resp = self.execute_command(DebugCommand.CMD_GET_LINE_STATE, 0, 2)
        if resp and len(resp) >= 2:
            host_state = resp[1] & 0x03
            device_state = (resp[1] >> 2) & 0x03

            states = {0: "SE0", 1: "J", 2: "K", 3: "SE1"}

            return {
                "host": states.get(host_state, "Unknown"),
                "device": states.get(device_state, "Unknown"),
            }
        return None

    def get_timestamp(self) -> Optional[int]:
        """Get current timestamp value"""
        resp = self.execute_command(DebugCommand.CMD_GET_TIMESTAMP, 0, 5)
        if resp and len(resp) >= 5:
            return (resp[4] << 24) | (resp[3] << 16) | (resp[2] << 8) | resp[1]
        return None

    def set_debug_leds(self, pattern: int) -> bool:
        """Set debug LED pattern"""
        resp = self.execute_command(DebugCommand.CMD_SET_DEBUG_LEDS, pattern, 2)
        return resp is not None and len(resp) >= 2 and resp[1] == pattern

    def set_debug_probe(self, pattern: int) -> bool:
        """Set debug probe output pattern"""
        resp = self.execute_command(DebugCommand.CMD_SET_DEBUG_PROBE, pattern, 2)
        return resp is not None and len(resp) >= 2 and resp[1] == pattern

    def set_debug_mode(self, mode: int) -> bool:
        """Set debug mode (0-3)"""
        if mode < 0 or mode > 3:
            self.error("Debug mode must be between 0 and 3")
            return False

        resp = self.execute_command(DebugCommand.CMD_SET_DEBUG_MODE, mode, 2)
        return resp is not None and len(resp) >= 2 and (resp[1] & 0x03) == mode

    def force_reset(self) -> bool:
        """Force a system reset"""
        resp = self.execute_command(DebugCommand.CMD_FORCE_RESET, 0, 1)
        return (
            resp is not None
            and len(resp) >= 1
            and resp[0] == DebugCommand.CMD_FORCE_RESET.value
        )

    def set_loopback(self, enabled: bool) -> bool:
        """Enable or disable loopback mode"""
        resp = self.execute_command(
            DebugCommand.CMD_LOOPBACK_ENABLE, 1 if enabled else 0, 2
        )
        return (
            resp is not None
            and len(resp) >= 2
            and (resp[1] & 0x01) == (1 if enabled else 0)
        )

    def set_trigger(self, config: int) -> bool:
        """Set trigger configuration"""
        resp = self.execute_command(DebugCommand.CMD_TRIGGER_CONFIG, config, 2)
        return resp is not None and len(resp) >= 2 and resp[1] == config

    def monitor_mode(self, duration: float = 10.0, interval: float = 0.5) -> None:
        """Enter monitoring mode for a specified duration"""
        self.info(
            f"Starting monitor mode for {duration:.1f} seconds (press Ctrl+C to exit early)"
        )

        try:
            start_time = time.time()
            while time.time() - start_time < duration:
                status = self.get_status()
                if status:
                    print(
                        f"\rProxy: {'Active' if status['proxy_active'] else 'Inactive'}, "
                        f"Host: {'Connected' if status['host_connected'] else 'Disconnected'}, "
                        f"Device: {'Connected' if status['device_connected'] else 'Disconnected'}",
                        end="",
                    )

                    # Update line states
                    line_states = self.get_line_state()
                    if line_states:
                        print(
                            f", Lines: H={line_states['host']}, D={line_states['device']}",
                            end="",
                        )

                    # Update packet count
                    packet_count = self.get_packet_count()
                    if packet_count is not None:
                        print(f", Packets: {packet_count}", end="")

                    # Update buffer status
                    buffer_used = self.get_buffer_status()
                    if buffer_used is not None:
                        print(f", Buffer: {buffer_used}/32768", end="")

                    sys.stdout.flush()

                time.sleep(interval)

            print()  # Final newline

        except KeyboardInterrupt:
            print("\nMonitoring stopped by user")


def main() -> int:
    """Main function"""
    parser = argparse.ArgumentParser(description="Cynthion USB Sniffer Debug Tool")
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="Enable verbose output"
    )
    parser.add_argument(
        "--vid",
        type=lambda x: int(x, 0),
        default=CYNTHION_VID,
        help=f"USB Vendor ID (default: 0x{CYNTHION_VID:04x})",
    )
    parser.add_argument(
        "--pid",
        type=lambda x: int(x, 0),
        default=CYNTHION_PID,
        help=f"USB Product ID (default: 0x{CYNTHION_PID:04x})",
    )

    # Command subparsers
    subparsers = parser.add_subparsers(dest="command", help="Command")

    # Info command
    info_parser = subparsers.add_parser("info", help="Get device information")

    # Status command
    status_parser = subparsers.add_parser("status", help="Get device status")

    # Monitor command
    monitor_parser = subparsers.add_parser("monitor", help="Monitor device status")
    monitor_parser.add_argument(
        "-d",
        "--duration",
        type=float,
        default=60.0,
        help="Monitoring duration in seconds (default: 60)",
    )
    monitor_parser.add_argument(
        "-i",
        "--interval",
        type=float,
        default=0.5,
        help="Update interval in seconds (default: 0.5)",
    )

    # LED control
    led_parser = subparsers.add_parser("led", help="Control debug LEDs")
    led_parser.add_argument(
        "pattern",
        type=lambda x: int(x, 0),
        help="LED pattern (0-255, can use hex with 0x prefix)",
    )

    # Debug mode
    mode_parser = subparsers.add_parser("mode", help="Set debug mode")
    mode_parser.add_argument(
        "mode", type=int, choices=[0, 1, 2, 3], help="Debug mode (0-3)"
    )

    # Reset command
    reset_parser = subparsers.add_parser("reset", help="Force system reset")

    # Loopback command
    loopback_parser = subparsers.add_parser("loopback", help="Set loopback mode")
    loopback_parser.add_argument(
        "enable",
        type=lambda x: x.lower() in ["1", "true", "yes", "on"],
        help="Enable loopback mode (1/true/yes/on or 0/false/no/off)",
    )

    args = parser.parse_args()

    # If no command specified, print help
    if not args.command:
        parser.print_help()
        return 1

    # Create debugger instance
    debugger = CynthionDebugger(vid=args.vid, pid=args.pid, verbose=args.verbose)

    # Connect to device
    if not debugger.connect():
        return 1

    try:
        # Process commands
        if args.command == "info":
            version = debugger.get_version()
            if version:
                print(f"Firmware Version: {version[0]}.{version[1]}.{version[2]}")
            else:
                print("Could not retrieve firmware version")

        elif args.command == "status":
            status = debugger.get_status()
            if status:
                print("Device Status:")
                print(f"  Proxy Active: {status['proxy_active']}")
                print(f"  Host Connected: {status['host_connected']}")
                print(f"  Device Connected: {status['device_connected']}")
                print(f"  Host Speed: {status['host_speed']}")
                print(f"  Device Speed: {status['device_speed']}")
                print(f"  Buffer Overflow: {status['buffer_overflow']}")

                buffer_used = debugger.get_buffer_status()
                if buffer_used is not None:
                    print(
                        f"  Buffer Usage: {buffer_used}/32768 bytes ({buffer_used/327.68:.1f}%)"
                    )

                packet_count = debugger.get_packet_count()
                if packet_count is not None:
                    print(f"  Packet Count: {packet_count}")

                error_count = debugger.get_error_count()
                if error_count is not None:
                    print(f"  Error Count: {error_count}")

                line_states = debugger.get_line_state()
                if line_states:
                    print(
                        f"  Line States: Host={line_states['host']}, Device={line_states['device']}"
                    )
            else:
                print("Could not retrieve device status")

        elif args.command == "monitor":
            debugger.monitor_mode(args.duration, args.interval)

        elif args.command == "led":
            if debugger.set_debug_leds(args.pattern):
                print(f"LEDs set to pattern: {args.pattern:08b} (0x{args.pattern:02x})")
            else:
                print("Failed to set LED pattern")

        elif args.command == "mode":
            if debugger.set_debug_mode(args.mode):
                print(f"Debug mode set to {args.mode}")
            else:
                print("Failed to set debug mode")

        elif args.command == "reset":
            if debugger.force_reset():
                print("System reset initiated")
            else:
                print("Failed to initiate system reset")

        elif args.command == "loopback":
            if debugger.set_loopback(args.enable):
                print(f"Loopback mode {'enabled' if args.enable else 'disabled'}")
            else:
                print(
                    f"Failed to {'enable' if args.enable else 'disable'} loopback mode"
                )

    finally:
        # Always disconnect
        debugger.disconnect()

    return 0


if __name__ == "__main__":
    sys.exit(main())
