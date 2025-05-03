#!/bin/bash
#
# compile_bitstream.sh - Bitstream Compilation Script for Cynthion FPGA
#
# This script compiles Verilog HDL files for the Cynthion FPGA (Lattice ECP5)
# using the open-source toolchain (Yosys, nextpnr-ecp5, ecppack)
#
# Usage: ./tools/compile_bitstream.sh [options]
#
# Options:
#   -c, --clean      Clean build directory before compilation
#   -v, --verbose    Enable verbose output
#   -s, --synth-only Only run synthesis (no P&R or bitstream generation)
#   -h, --help       Show this help message
#
# Author: GitHub Copilot (generated on May 3, 2025)

# Stop on first error
set -e

# Default settings
VERBOSE=0
CLEAN=0
SYNTH_ONLY=0
DEVICE="45k"  # ECP5 device variant (25k, 45k, 85k)
PACKAGE="CABGA381"  # ECP5 package type

# Parse arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    -c|--clean)
      CLEAN=1
      shift
      ;;
    -v|--verbose)
      VERBOSE=1
      shift
      ;;
    -s|--synth-only)
      SYNTH_ONLY=1
      shift
      ;;
    -h|--help)
      echo "Usage: $0 [options]"
      echo "Options:"
      echo "  -c, --clean      Clean build directory before compilation"
      echo "  -v, --verbose    Enable verbose output"
      echo "  -s, --synth-only Only run synthesis (no P&R or bitstream generation)"
      echo "  -h, --help       Show this help message"
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      echo "Use '$0 --help' for usage information"
      exit 1
      ;;
  esac
done

# Script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

# Set paths
RTL_DIR="$PROJECT_ROOT/hardware/rtl"
CONSTRAINTS_DIR="$PROJECT_ROOT/hardware/constraints"
BUILD_DIR="$PROJECT_ROOT/tools/build"
LOG_DIR="$BUILD_DIR/logs"

# Create build and log directories
mkdir -p "$BUILD_DIR"
mkdir -p "$LOG_DIR"

# Set file paths
CONSTRAINT_FILE="$CONSTRAINTS_DIR/cynthion_pins.lpf"
TOP_MODULE="top"
SYNTH_JSON="$BUILD_DIR/${TOP_MODULE}_synth.json"
ROUTED_JSON="$BUILD_DIR/${TOP_MODULE}_routed.json"
BITSTREAM_FILE="$BUILD_DIR/${TOP_MODULE}.bit"
SVFSTREAM_FILE="$BUILD_DIR/${TOP_MODULE}.svf"

# Clean option
if [ $CLEAN -eq 1 ]; then
  echo "Cleaning build directory..."
  rm -rf "$BUILD_DIR"/*
  mkdir -p "$BUILD_DIR"
  mkdir -p "$LOG_DIR"
fi

# Check if required tools are installed
check_tool() {
  if ! command -v $1 &> /dev/null; then
    echo "Error: $1 command not found."
    echo "Please install the required tools using:"
    echo "  For macOS: brew install open-fpga-toolchain"
    echo "  For Linux: apt-get install yosys nextpnr-ecp5 prjtrellis"
    return 1
  fi
  return 0
}

# Find all Verilog files
find_verilog_files() {
  find "$RTL_DIR" -type f -name "*.v" | sort
}

# Validate HDL
echo "Validating HDL files..."
"$PROJECT_ROOT/tools/validate_hdl.sh" -v || {
  echo "HDL validation failed. Please fix syntax errors before compiling."
  exit 1
}

# Get list of all Verilog files for synthesis
VERILOG_FILES=$(find_verilog_files)
NUM_FILES=$(echo "$VERILOG_FILES" | wc -l)
echo "Found $NUM_FILES Verilog files for synthesis."

# Check Yosys installation
if ! check_tool yosys; then
  echo "Missing Yosys installation. Please install before continuing."
  exit 1
fi

# Create Yosys synthesis script
YOSYS_SCRIPT="$BUILD_DIR/synth.ys"
cat > "$YOSYS_SCRIPT" << EOF
# Yosys synthesis script for Cynthion FPGA
EOF

# Add each Verilog file individually to the script
for file in $VERILOG_FILES; do
  echo "read_verilog -sv $file" >> "$YOSYS_SCRIPT"
done

# Add the rest of the synthesis commands
cat >> "$YOSYS_SCRIPT" << EOF
hierarchy -check -top $TOP_MODULE
synth_ecp5 -json $SYNTH_JSON
EOF

# Run synthesis with Yosys
echo "Synthesizing design with Yosys..."
if [ $VERBOSE -eq 1 ]; then
  yosys -l "$LOG_DIR/synthesis.log" "$YOSYS_SCRIPT"
else
  yosys -q -l "$LOG_DIR/synthesis.log" "$YOSYS_SCRIPT"
fi

echo "Synthesis completed. JSON netlist generated at $SYNTH_JSON"

# Stop here if only synthesis is requested
if [ $SYNTH_ONLY -eq 1 ]; then
  echo "Synthesis-only option selected. Stopping after synthesis."
  exit 0
fi

# Check nextpnr-ecp5 installation
if ! check_tool nextpnr-ecp5; then
  echo "Warning: nextpnr-ecp5 not found. Cannot continue with place and route."
  echo "Install with: brew install nextpnr-ecp5"
  exit 1
fi

# Run place and route with nextpnr-ecp5
echo "Running place and route with nextpnr-ecp5..."
if [ $VERBOSE -eq 1 ]; then
  nextpnr-ecp5 --${DEVICE} --package ${PACKAGE} \
    --json "$SYNTH_JSON" \
    --lpf "$CONSTRAINT_FILE" \
    --textcfg "$BUILD_DIR/${TOP_MODULE}_out.config" \
    --json "$ROUTED_JSON" \
    --log "$LOG_DIR/pnr.log"
else
  nextpnr-ecp5 --${DEVICE} --package ${PACKAGE} \
    --json "$SYNTH_JSON" \
    --lpf "$CONSTRAINT_FILE" \
    --textcfg "$BUILD_DIR/${TOP_MODULE}_out.config" \
    --json "$ROUTED_JSON" \
    --quiet \
    --log "$LOG_DIR/pnr.log"
fi

echo "Place and route completed."

# Check ecppack installation
if ! check_tool ecppack; then
  echo "Warning: ecppack not found. Cannot generate bitstream."
  echo "Install with: brew install trellis"
  exit 1
fi

# Generate bitstream with ecppack
echo "Generating bitstream with ecppack..."
ecppack --input "$BUILD_DIR/${TOP_MODULE}_out.config" --bit "$BITSTREAM_FILE" --svf "$SVFSTREAM_FILE"

echo "Bitstream generation completed successfully!"
echo "Bitstream file: $BITSTREAM_FILE"
echo "SVF file for programming: $SVFSTREAM_FILE"

# Provide instructions for programming the FPGA
echo ""
echo "To program the Cynthion FPGA, use:"
echo "  $PROJECT_ROOT/tools/flash_cynthion.sh $BITSTREAM_FILE"