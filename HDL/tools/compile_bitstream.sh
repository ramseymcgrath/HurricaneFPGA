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

# Use RAM disk if available
TMP_BUILD_BASE="${TMPDIR:-/dev/shm}"
BUILD_DIR="$TMP_BUILD_BASE/cynthion_build"
LOG_DIR="$BUILD_DIR/logs"

mkdir -p "$BUILD_DIR" "$LOG_DIR"

TOP_MODULE="top"
SYNTH_JSON="$BUILD_DIR/${TOP_MODULE}_synth.json"
ROUTED_JSON="$BUILD_DIR/${TOP_MODULE}_routed.json"
BITSTREAM_FILE="$BUILD_DIR/${TOP_MODULE}.bit"
SVFSTREAM_FILE="$BUILD_DIR/${TOP_MODULE}.svf"
CONSTRAINT_FILE="$CONSTRAINTS_DIR/cynthion_pins.lpf"

[[ $CLEAN -eq 1 ]] && {
  echo "[clean] Wiping build dir..."
  rm -rf "$BUILD_DIR"/*
  mkdir -p "$BUILD_DIR" "$LOG_DIR"
}

check_tool() {
  if ! command -v "$1" &>/dev/null; then
    echo "Missing tool: $1"
    exit 1
  fi
}
for tool in yosys nextpnr-ecp5 ecppack; do check_tool "$tool"; done

echo "[validate] Checking HDL files..."
"$PROJECT_ROOT/tools/validate_hdl.sh" -v || {
  echo "HDL validation failed."
  exit 1
}

readarray -t VERILOG_FILES < <(find "$RTL_DIR" -type f -name "*.v")
echo "[synth] Found ${#VERILOG_FILES[@]} Verilog files."

YOSYS_SCRIPT="$BUILD_DIR/synth.ys"
{
  echo "# Yosys synthesis script"
  for file in "${VERILOG_FILES[@]}"; do
    echo "read_verilog -sv $file"
  done
  echo "hierarchy -check -top $TOP_MODULE"
  echo "# Set ABC parameters for multi-threading"
  echo "plugin -i design"
  echo "plugin -i abc"
  echo "abc -D $NUM_THREADS"
  echo "synth_ecp5 -json $SYNTH_JSON"
} > "$YOSYS_SCRIPT"

echo "[synth] Starting Yosys..."
start_time=$(date +%s)
if [[ $VERBOSE -eq 1 ]]; then
  yosys -l "$LOG_DIR/synthesis.log" "$YOSYS_SCRIPT"
else
  yosys -q -l "$LOG_DIR/synthesis.log" "$YOSYS_SCRIPT"
fi
end_time=$(date +%s)
echo "[synth] Done → $SYNTH_JSON (in $((end_time - start_time))s)"

[[ $SYNTH_ONLY -eq 1 ]] && { echo "Stopping at synthesis."; exit 0; }

echo "[pnr] Launching nextpnr-ecp5 with $NUM_THREADS threads..."
start_time=$(date +%s)
nextpnr_args=(
  --${DEVICE}
  --package ${PACKAGE}
  --json "$SYNTH_JSON"
  --lpf "$CONSTRAINT_FILE"
  --textcfg "$BUILD_DIR/${TOP_MODULE}_out.config"
  --write "$ROUTED_JSON"
  --threads $NUM_THREADS
  --fast-expr
  --log "$LOG_DIR/pnr.log"
)
[[ $VERBOSE -eq 0 ]] && nextpnr_args+=(--quiet)
nextpnr-ecp5 "${nextpnr_args[@]}"
end_time=$(date +%s)
echo "[pnr] Done → $ROUTED_JSON (in $((end_time - start_time))s)"

echo "[bitgen] Creating bitstream..."
start_time=$(date +%s)
ecppack --input "$BUILD_DIR/${TOP_MODULE}_out.config" \
        --bit "$BITSTREAM_FILE" \
        --svf "$SVFSTREAM_FILE"
end_time=$(date +%s)
echo "[bitgen] Done → $BITSTREAM_FILE (in $((end_time - start_time))s)"

echo ""
echo "[✓] Build complete:"
echo "    Bitstream : $BITSTREAM_FILE"
echo "    SVF       : $SVFSTREAM_FILE"
echo ""
echo "To flash:"
echo "    openFPGALoader --board cynthion $BITSTREAM_FILE"
