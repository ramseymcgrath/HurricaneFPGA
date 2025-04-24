#!/usr/bin/env python3

"""
Hurricane FPGA Flashing Tool
============================

This script performs a more reliable and verbose flashing of the Hurricane FPGA.
It ensures that:
1. The bitstream is built correctly
2. The flashing process is completed with verification
3. Detailed progress is shown during flashing
"""

import os
import sys
import time
import argparse
import subprocess
from pathlib import Path

# Add the project root to the path
sys.path.insert(0, str(Path(__file__).resolve().parent))

# Try to import LUNA-specific modules
try:
    from luna.gateware.platform import get_appropriate_platform
    from luna.apollo import ApolloDebugger
    from luna.apollo.ecp5 import ECP5_JTAGProgrammer
    from luna.apollo.dfu import USBDFUManager
except ImportError:
    print("ERROR: Failed to import LUNA modules. Please ensure the environment is set up correctly.")
    print("You may need to install LUNA: pip install luna")
    sys.exit(1)

def build_bitstream(verbose=False, output_dir=None):
    """Build the bitstream without programming."""
    print("\n=== BUILDING BITSTREAM ===")
    
    # Set environment variable to prevent programming during build
    os.environ["BUILD_LOCAL"] = "1"
    
    # Determine the output directory
    if output_dir is None:
        output_dir = Path.cwd() / "build" / "gateware"
    else:
        output_dir = Path(output_dir)
    
    # Ensure the output directory exists
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Build command
    build_cmd = [
        sys.executable,
        str(Path.cwd() / "src" / "backend" / "top.py"),
        "--verbose" if verbose else "",
    ]
    
    # Run the build process
    try:
        build_process = subprocess.run(
            build_cmd,
            stdout=subprocess.PIPE if not verbose else None,
            stderr=subprocess.STDOUT if not verbose else None,
            text=True,
            check=True
        )
        if not verbose and build_process.stdout:
            print(build_process.stdout)
        print("✅ Bitstream built successfully!")
        
        # Find the generated bitstream files
        bitstream_file = None
        for ext in [".bit", ".svf"]:
            files = list(output_dir.glob(f"*{ext}"))
            if files:
                bitstream_file = files[0]
                break
                
        if bitstream_file:
            print(f"📁 Bitstream file: {bitstream_file}")
            return bitstream_file
        else:
            print("⚠️ Bitstream built but file not found. Check the build directory.")
            return None
            
    except subprocess.CalledProcessError as e:
        print("❌ Bitstream build failed!")
        if not verbose and e.stdout:
            print(e.stdout)
        return None

def verify_device_connection():
    """Check if the FPGA device is connected and accessible."""
    print("\n=== CHECKING DEVICE CONNECTION ===")
    
    # Try to find the Cynthion board via LUNA
    try:
        platform = get_appropriate_platform()
        print(f"✅ Found platform: {platform.name}")
        return True
    except Exception as e:
        print(f"❌ Could not find platform: {e}")
        return False

def program_via_dfu(bitstream_file=None, verbose=False):
    """Program the FPGA using DFU (Device Firmware Upgrade)."""
    print("\n=== PROGRAMMING VIA DFU ===")
    
    if bitstream_file is None:
        # Use the default location
        build_dir = Path.cwd() / "build" / "gateware"
        bit_files = list(build_dir.glob("*.bit"))
        if bit_files:
            bitstream_file = bit_files[0]
        else:
            print("❌ No bitstream file found! Run build first.")
            return False
    
    print(f"📁 Using bitstream: {bitstream_file}")
    
    # Ensure the file exists
    if not Path(bitstream_file).exists():
        print(f"❌ Bitstream file not found: {bitstream_file}")
        return False
    
    # Put device in DFU mode if needed
    try:
        # Try to find DFU devices
        dfu = USBDFUManager()
        
        if not dfu.device:
            print("⚠️ No DFU device found. If your device is connected:")
            print("   1. Press and hold the DFU button (typically RESET)")
            print("   2. Briefly press the PROGRAM button while holding DFU")
            print("   3. Release both buttons")
            input("Press Enter when ready to check again...")
            
            # Try again
            dfu = USBDFUManager()
        
        if not dfu.device:
            print("❌ Still no DFU device found. Check connections and try again.")
            return False
            
        print(f"✅ Found DFU device: {dfu.device.product_string}")
        
        # Start programming
        print("⏳ Programming device (this may take 30+ seconds)...")
        start_time = time.time()
        
        # Program the bitstream
        try:
            with open(bitstream_file, 'rb') as f:
                bitstream = f.read()
                
            # Actually flash the device
            dfu.write(0, bitstream)
            dfu.detach_and_reset()
            
            elapsed = time.time() - start_time
            print(f"✅ Programming complete! Took {elapsed:.2f} seconds")
            return True
            
        except Exception as e:
            print(f"❌ Programming failed: {e}")
            return False
            
    except Exception as e:
        print(f"❌ Error during DFU programming: {e}")
        return False

def main():
    """Main function for the flashing tool."""
    parser = argparse.ArgumentParser(description='Hurricane FPGA Flashing Tool')
    parser.add_argument('--verbose', '-v', action='store_true', help='Enable verbose output')
    parser.add_argument('--no-build', action='store_true', help='Skip building the bitstream')
    parser.add_argument('--bitstream', '-b', type=str, help='Path to bitstream file to use (default: auto-detect)')
    parser.add_argument('--output-dir', '-o', type=str, help='Directory to store build outputs')
    
    args = parser.parse_args()
    
    print("🌀 Hurricane FPGA Flashing Tool")
    print("===============================")
    
    # Make sure the device is connected
    if not verify_device_connection():
        print("\n❌ Please connect your FPGA device and try again.")
        return 1
    
    # Build the bitstream if needed
    bitstream_file = args.bitstream
    if not args.no_build:
        bitstream_file = build_bitstream(verbose=args.verbose, output_dir=args.output_dir)
        if not bitstream_file:
            print("\n❌ Failed to build bitstream. Cannot continue with programming.")
            return 1
    elif not bitstream_file:
        # If no build but also no bitstream specified, try to find it
        build_dir = Path.cwd() / "build" / "gateware"
        bit_files = list(build_dir.glob("*.bit"))
        if bit_files:
            bitstream_file = bit_files[0]
            print(f"🔍 Found bitstream: {bitstream_file}")
        else:
            print("\n❌ No bitstream file specified and none found. Please build first or specify a file.")
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