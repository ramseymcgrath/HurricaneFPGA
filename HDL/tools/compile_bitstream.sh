#!/bin/bash
set -euo pipefail

VERBOSE=0
CLEAN=0
SYNTH_ONLY=0
DEVICE="45k"
PACKAGE="CABGA381"
NUM_THREADS=$(nproc || sysctl -n hw.ncpu || echo 4)

while [[ $# -gt 0 ]]; do
  case $1 in
    -c|--clean) CLEAN=1; shift ;;
    -v|--verbose) VERBOSE=1; shift ;;
    -s|--synth-only) SYNTH_ONLY=1; shift ;;
    -h|--help)
      echo "Usage: $0 [options]"
      echo "  -c, --clean      Clean build directory"
      echo "  -v, --verbose    Verbose build logs"
      echo "  -s, --synth-only Stop after synthesis"
      echo "  -h, --help       Show help"
      exit 0 ;;
    *) echo "Unknown option: $1"; exit 1 ;;
  esac
done

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

RTL_DIR="$PROJECT_ROOT/hardware/rtl"
CONSTRAINTS_DIR="$PROJECT_ROOT/hardware/constraints"
BUILD_DIR="$PROJECT_ROOT/tools/build"
LOG_DIR="$BUILD_DIR/logs"

mkdir -p "$BUILD_DIR" "$LOG_DIR"

TOP_MODULE="top"
SYNTH_JSON="$BUILD_DIR/${TOP_MODULE}_synth.json"
ROUTED_JSON="$BUILD_DIR/${TOP_MODULE}_routed.json"
BITSTREAM_FILE="$BUILD_DIR/${TOP_MODULE}.bit"
SVFSTREAM_FILE="$BUILD_DIR/${TOP_MODULE}.svf"
CONSTRAINT_FILE="$CONSTRAINTS_DIR/cynthion_pins.lpf"

if [[ $CLEAN -eq 1 ]]; then
  echo "[clean] Wiping build dir..."
  rm -rf "$BUILD_DIR"/*
  mkdir -p "$BUILD_DIR" "$LOG_DIR"
fi

check_tool() {
  if ! command -v "$1" &>/dev/null; then
    echo "Missing tool: $1"
    echo "Install via: apt/pacman/dnf/brew depending on your OS"
    return 1
  fi
}

echo "[validate] Checking HDL files..."
"$PROJECT_ROOT/tools/validate_hdl.sh" -v || {
  echo "HDL validation failed."
  exit 1
}

readarray -t VERILOG_FILES < <(find "$RTL_DIR" -type f -name "*.v")
echo "[synth] Found ${#VERILOG_FILES[@]} Verilog source files."

check_tool yosys || exit 1

YOSYS_SCRIPT="$BUILD_DIR/synth.ys"
{
  echo "# Yosys synthesis script"
  for file in "${VERILOG_FILES[@]}"; do
    echo "read_verilog -sv $file"
  done
  echo "hierarchy -check -top $TOP_MODULE"
  echo "synth_ecp5 -json $SYNTH_JSON"
  # use threaded ABC if available
  echo "set yosys_abc_exec abc -D $NUM_THREADS"
} > "$YOSYS_SCRIPT"

echo "[synth] Running Yosys synthesis..."
if [[ $VERBOSE -eq 1 ]]; then
  yosys -l "$LOG_DIR/synthesis.log" "$YOSYS_SCRIPT"
else
  yosys -q -l "$LOG_DIR/synthesis.log" "$YOSYS_SCRIPT"
fi

echo "[synth] Done -> $SYNTH_JSON"

[[ $SYNTH_ONLY -eq 1 ]] && { echo "Stopping at synthesis."; exit 0; }

check_tool nextpnr-ecp5 || exit 1

echo "[pnr] Launching nextpnr-ecp5 with $NUM_THREADS threads..."
nextpnr_args=(
  --${DEVICE}
  --package ${PACKAGE}
  --json "$SYNTH_JSON"
  --lpf "$CONSTRAINT_FILE"
  --textcfg "$BUILD_DIR/${TOP_MODULE}_out.config"
  --json "$ROUTED_JSON"
  --log "$LOG_DIR/pnr.log"
  --threads $NUM_THREADS
)
[[ $VERBOSE -eq 0 ]] && nextpnr_args+=(--quiet)

nextpnr-ecp5 "${nextpnr_args[@]}"
echo "[pnr] Completed."

check_tool ecppack || exit 1

echo "[bitgen] Creating bitstream..."
ecppack --input "$BUILD_DIR/${TOP_MODULE}_out.config" \
        --bit "$BITSTREAM_FILE" \
        --svf "$SVFSTREAM_FILE"

echo ""
echo "[✓] Bitstream ready:"
echo "    → $BITSTREAM_FILE"
echo "    → $SVFSTREAM_FILE"
echo ""
echo "To flash:"
echo "    openFPGALoader --board cynthion $BITSTREAM_FILE"
