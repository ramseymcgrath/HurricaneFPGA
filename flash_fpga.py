#!/usr/bin/env python3

"""
Hurricane FPGA Flashing Tool
============================

This script provides a more reliable and verbose flashing process for the Hurricane FPGA.
It ensures proper DFU programming with timing information to verify the process completed.
"""

import os
import sys
import time
import argparse
import subprocess
from pathlib import Path
from apollo_fpga import ApolloDebugger
from luna.gateware.platform import get_appropriate_platform


def find_bitstream(build_dir=None):
    """Find the most recently modified bitstream file."""
    # First, try the standard build directory paths
    possible_paths = [
        Path.cwd() / "build" / "gateware",  # Default
        Path("/home/ramsey/code/HurricaneFPGA/build"),  # Absolute path
        Path.cwd() / "build",  # Just the build directory
        Path.cwd(),  # Current directory
    ]

    # Add custom directory if provided
    if build_dir:
        possible_paths.insert(0, Path(build_dir))

    # Check each possible path
    for path in possible_paths:
        if not path.exists():
            print(f"Directory not found: {path}")
            continue

        # Look for .bit files
        bit_files = list(path.glob("**/*.bit"))  # Recursive search

        # Also look for other ECP5 output files
        if not bit_files:
            bit_files = list(path.glob("**/*.svf"))  # SVF format
        if not bit_files:
            bit_files = list(path.glob("**/*.lpf"))  # LPF format
        if not bit_files:
            bit_files = list(path.glob("**/*.json"))  # Nextpnr JSON
        if not bit_files:
            # Common Amaranth/LUNA output formats
            bit_files = list(path.glob("**/top.*.bin"))

        if bit_files:
            # Sort by modification time (newest first)
            bit_files.sort(key=lambda x: x.stat().st_mtime, reverse=True)
            print(f"Found {len(bit_files)} bitstream files in {path}")
            return bit_files[0]

    # Also try running find command to locate recent bitstream files
    try:
        print("Searching for recently modified bitstream files...")
        find_cmd = [
            "find",
            str(Path.cwd()),
            "-name",
            "*.bit",
            "-o",
            "-name",
            "*.svf",
            "-o",
            "-name",
            "top.*.bin",
            "-type",
            "f",
            "-mtime",
            "-1",
        ]
        find_result = subprocess.run(
            find_cmd, capture_output=True, text=True, check=False
        )

        if find_result.returncode == 0 and find_result.stdout.strip():
            # Parse the results and find the most recent file
            files = [
                Path(line.strip())
                for line in find_result.stdout.splitlines()
                if line.strip()
            ]
            if files:
                files.sort(key=lambda x: x.stat().st_mtime, reverse=True)
                print(f"Found recent bitstream using find: {files[0]}")
                return files[0]
    except (subprocess.SubprocessError, FileNotFoundError) as e:
        print(f"Error running find command: {e}")

    # Also try using locate if available on the system
    try:
        import subprocess

        locate_result = subprocess.run(
            ["locate", "-l", "1", "--regex", ".*HurricaneFPGA.*\.bit$"],
            capture_output=True,
            text=True,
            check=False,
        )
        if locate_result.returncode == 0 and locate_result.stdout.strip():
            path = Path(locate_result.stdout.strip())
            if path.exists():
                print(f"Found bitstream using locate: {path}")
                return path
    except (ImportError, FileNotFoundError):
        # locate command not available or failed
        pass

    print("⚠️ No .bit files found in any of the expected directories")
    return None


def build_bitstream(verbose=False):
    """Build the bitstream without programming."""
    print("\n=== BUILDING BITSTREAM ===")

    # Set environment variable to disable programming during build
    os.environ["BUILD_LOCAL"] = "1"

    # Check if we need to use python-env instead of standard python
    python_cmd = (
        "python-env" if os.path.exists("/usr/local/bin/python-env") else sys.executable
    )

    # Build command - use python-env if it exists
    build_cmd = [python_cmd, str(Path.cwd() / "src" / "backend" / "top.py")]

    # Add any LUNA CLI arguments that control the build
    if verbose:
        build_cmd.append("--dry-run")

    # Add keep-files flag to ensure we have access to the bitstream
    build_cmd.append("--keep-files")

    print(f"Running: {' '.join(build_cmd)}")

    # Run the build process
    try:
        build_process = subprocess.run(
            build_cmd,
            stdout=None if verbose else subprocess.PIPE,
            stderr=None if verbose else subprocess.STDOUT,
            text=True,
            check=True,
        )

        if not verbose and build_process.stdout:
            print(build_process.stdout)

        print("✅ Bitstream built successfully!")

        # Find the generated bitstream file
        bitstream_file = find_bitstream()
        if bitstream_file:
            print(f"📁 Found bitstream: {bitstream_file}")
            return bitstream_file
        else:
            print("⚠️ Bitstream built but file not found in expected location.")
            return None

    except subprocess.CalledProcessError as e:
        print("❌ Bitstream build failed!")
        if not verbose and e.stdout:
            print(e.stdout)
        return None


def program_via_dfu(bitstream_file=None, verbose=False):
    """Program the FPGA using DFU (Device Firmware Upgrade)."""
    print("\n=== PROGRAMMING VIA DFU ===")

    if bitstream_file is None:
        # Try to find the most recent bitstream
        bitstream_file = find_bitstream()
        if not bitstream_file:
            print("❌ No bitstream file found! Run the build step first.")
            return False

    print(f"📁 Using bitstream: {bitstream_file}")

    # Ensure the file exists
    if not os.path.exists(bitstream_file):
        print(f"❌ Bitstream file not found: {bitstream_file}")
        return False

    # Check bitstream file size
    file_size = os.path.getsize(bitstream_file)
    print(f"💾 Bitstream size: {file_size/1024:.1f} KB")

    # Check if the file is a valid bitstream (basic check)
    if file_size < 1000:  # Bitstreams are typically at least several KB
        print(f"⚠️ Warning: Bitstream file seems unusually small ({file_size} bytes)")

    # Put device in DFU mode if needed
    try:
        # Try to find DFU devices
        device = ApolloDebugger()

        if not device:
            print("\n⚠️ No DFU device found. Please put your device in DFU mode:")
            print("   1. Connect the FPGA board via USB")
            print("   2. Press and hold the DFU button (typically RESET)")
            print("   3. Briefly press the PROGRAM button while holding DFU")
            print("   4. Release both buttons")
            input("\nPress Enter when ready to check again...")

            # Try again after the user confirms
            device = ApolloDebugger()

        if not device:
            print("❌ Still no DFU device found. Check connections and try again.")
            return False

        # Show device info
        print(f"✅ Found DFU device: {device.detect_connected_version()}")

        # Start programming with timing
        print("\n⏳ Programming device (this may take 30-60 seconds)...")
        start_time = time.time()

        # Program the bitstream
        try:
            with open(bitstream_file, "rb") as f:
                bitstream = f.read()
                # Actually flash the device
                device.flash(bitstream, offset=0)

            # Calculate elapsed time
            elapsed = time.time() - start_time

            # Verify the time is reasonable (DFU programming should take at least a few seconds)
            if elapsed < 2.0:
                print(
                    f"⚠️ Warning: Programming completed suspiciously quickly ({elapsed:.2f} seconds)"
                )
                print(
                    "   This might indicate that the actual programming didn't occur."
                )
                print("   Try running with --verbose for more details.")
            else:
                print(f"✅ Programming complete! Took {elapsed:.2f} seconds")

            # Reset the device
            print("🔄 Resetting device...")
            device.soft_reset()
            return True

        except Exception as e:
            print(f"❌ Programming failed: {e}")
            return False

    except Exception as e:
        print(f"❌ Error during DFU programming: {e}")
        return False


def main():
    """Main function for the flashing tool."""
    parser = argparse.ArgumentParser(description="Hurricane FPGA Flashing Tool")
    parser.add_argument(
        "--no-build", "-n", action="store_true", help="Skip building the bitstream"
    )
    parser.add_argument(
        "--bitstream",
        "-b",
        type=str,
        help="Path to bitstream file to use (default: auto-detect)",
    )

    args = parser.parse_args()

    print("🌀 Hurricane FPGA Flashing Tool")
    print("===============================")

    # Build the bitstream if needed
    bitstream_file = args.bitstream
    if not args.no_build:
        bitstream_file = build_bitstream(verbose=args.verbose)
        if not bitstream_file:
            print("\n❌ Failed to build bitstream.")
            return 1
    elif not bitstream_file:
        # If --no-build but no bitstream specified, try to find it
        bitstream_file = find_bitstream()
        if bitstream_file:
            print(f"🔍 Found bitstream: {bitstream_file}")
        else:
            print(
                "\n❌ No bitstream file specified and none found. Please build first or specify a file."
            )
            return 1

    # Program the device
    if program_via_dfu(bitstream_file, verbose=args.verbose):
        print("\n✅ Device successfully programmed!")
        return 0
    else:
        print("\n❌ Failed to program the device.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
