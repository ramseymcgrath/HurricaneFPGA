#!/usr/bin/env bash
# ===============================================================
# Cynthion Build Driver (ECP5)
# ===============================================================
# A portable, deterministic, and CI‑friendly build script that
# synthesises, places & routes, and packages a bitstream for the
# Cynthion FPGA platform.  Designed for Lattice ECP5 devices.
# ---------------------------------------------------------------
# Features
#   • CLI flags for device, package, top module, threads, etc.
#   • Auto‑detects logical processors on Linux/macOS/WSL.
#   • Optional use of a RAM‑disk/TMPDIR for faster builds.
#   • Deterministic file ordering; reproducible outputs.
#   • Strict timing‑failure detection.
#   • Colourised, timestamped logging.
#   • Clean + synth‑only modes; optional HDL validation hook.
# ---------------------------------------------------------------
set -euo pipefail
IFS=$'\n\t'

# ---------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------
colour() { [[ -t 1 ]] || return; tput setaf "$1"; }
reset_colour() { [[ -t 1 ]] && tput sgr0 || true; }
log() { printf "%s[%s]%s %s\n" "$(colour 2)" "$1" "$(reset_colour)" "${2:-}"; }
err() { printf "%s[x]%s %s\n" "$(colour 1)" "$(reset_colour)" "$1" >&2; }
usage() {
  cat <<-EOF
    Usage: ${0##*/} [options]

      -d, --device <name>        ECP5 device (25k|45k|85k)       [45k]
      -p, --package <pkg>        Package code (e.g. CABGA381)     [CABGA381]
      -t, --top <module>         Top‑level RTL module             [top]
      -c, --constraint <file>    LPF/PCF constraint file          [constraints/cynthion_pins.lpf]
      -j, --jobs <N>             Parallel jobs / threads          [auto]
      -b, --build-dir <dir>      Build directory                  [tmp‑ramdisk]
      -v, --verbose              Verbose tool output
      -s, --synth-only           Stop after synthesis
      --no-validate              Skip HDL syntax validation
      --clean                    Remove previous build artifacts
      -h, --help                 Show this help and exit
  EOF
}

# ---------------------------------------------------------------
# Default configuration (overridable by env vars *or* CLI flags)
# ---------------------------------------------------------------
DEVICE="${DEVICE:-45k}"
PACKAGE="${PACKAGE:-CABGA381}"
TOP_MODULE="${TOP_MODULE:-top}"
CONSTRAINT_FILE="${CONSTRAINT_FILE:-hardware/constraints/cynthion_pins.lpf}"
NUM_THREADS="auto"
VERBOSE=0
SYNTH_ONLY=0
CLEAN=0
VALIDATE=1

# ---------------------------------------------------------------
# CLI parsing (GNU & BSD getopts compatible)
# ---------------------------------------------------------------
while [[ $# -gt 0 ]]; do
  case "$1" in
    -d|--device)       DEVICE="$2"; shift 2;;
    -p|--package)      PACKAGE="$2"; shift 2;;
    -t|--top)          TOP_MODULE="$2"; shift 2;;
    -c|--constraint)   CONSTRAINT_FILE="$2"; shift 2;;
    -j|--jobs)         NUM_THREADS="$2"; shift 2;;
    -b|--build-dir)    BUILD_DIR="$2"; shift 2;;
    -v|--verbose)      VERBOSE=1; shift;;
    -s|--synth-only)   SYNTH_ONLY=1; shift;;
    --no-validate)     VALIDATE=0; shift;;
    --clean)           CLEAN=1; shift;;
    -h|--help)         usage; exit 2;;
    *) err "Unknown option: $1"; usage; exit 1;;
  esac
done

# ---------------------------------------------------------------
# Derive paths and environment
# ---------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
RTL_DIR="$PROJECT_ROOT/hardware/rtl"
CONSTRAINT_FILE="$(realpath -m "$CONSTRAINT_FILE")"

# Thread detection
if [[ "$NUM_THREADS" == "auto" ]]; then
  if command -v nproc &>/dev/null; then
    NUM_THREADS="$(nproc)"
  else
    NUM_THREADS="$(sysctl -n hw.ncpu 2>/dev/null || echo 4)"
  fi
fi

# Build directory (RAM disk preferred)
if [[ -z "${BUILD_DIR:-}" ]]; then
  RAMDISK="${TMPDIR:-/dev/shm}"
  BUILD_DIR="$RAMDISK/cynthion_build"
fi
LOG_DIR="$BUILD_DIR/logs"

# ---------------------------------------------------------------
# House‑keeping
# ---------------------------------------------------------------
trap 'err "Aborted by user"; exit 130' INT
mkdir -p "$BUILD_DIR" "$LOG_DIR"
[[ $CLEAN -eq 1 ]] && { log clean "Removing $BUILD_DIR"; rm -rf "$BUILD_DIR"/*; }

# Verify constraint file exists
[[ -f "$CONSTRAINT_FILE" ]] || { err "Constraint file not found: $CONSTRAINT_FILE"; exit 1; }

# Toolchain sanity check
for tool in yosys nextpnr-ecp5 ecppack; do
  command -v "$tool" &>/dev/null || { err "Missing tool: $tool"; exit 1; }
  "$tool" --version &>/dev/null || true
done > "$LOG_DIR/tool_versions.txt"

# ---------------------------------------------------------------
# Optional HDL validation
# ---------------------------------------------------------------
if (( VALIDATE )); then
  if [[ -x "$PROJECT_ROOT/tools/validate_hdl.sh" ]]; then
    log validate "Running HDL syntax validation…"
    "$PROJECT_ROOT/tools/validate_hdl.sh" ${VERBOSE:+-v} || { err "HDL validation failed"; exit 1; }
  else
    log validate "No validate_hdl.sh found – skipping"
  fi
fi

# ---------------------------------------------------------------
# Gather & sort RTL files deterministically
# ---------------------------------------------------------------
mapfile -t VERILOG_FILES < <(find "$RTL_DIR" -type f -name "*.v" | sort)
[[ ${#VERILOG_FILES[@]} -eq 0 ]] && { err "No Verilog files found in $RTL_DIR"; exit 1; }
log synth "Found ${#VERILOG_FILES[@]} Verilog sources"

# ---------------------------------------------------------------
# Create Yosys synthesis script
# ---------------------------------------------------------------
SYNTH_JSON="$BUILD_DIR/${TOP_MODULE}_synth.json"
YOSYS_SCRIPT="$BUILD_DIR/synth.ys"
{
  echo "# Auto‑generated by cynthion_build.sh";
  for src in "${VERILOG_FILES[@]}"; do printf 'read_verilog -sv %q\n' "$src"; done
  printf 'hierarchy -check -top %s\n' "$TOP_MODULE"
  printf 'synth_ecp5 -abc9 -json %q\n' "$SYNTH_JSON"
} > "$YOSYS_SCRIPT"

# ---------------------------------------------------------------
# Run synthesis
# ---------------------------------------------------------------
log synth "Launching Yosys…"
start_ts=$(date +%s)
if (( VERBOSE )); then yosys -l "$LOG_DIR/synthesis.log" "$YOSYS_SCRIPT"; else yosys -q -l "$LOG_DIR/synthesis.log" "$YOSYS_SCRIPT"; fi
log synth "Done → $SYNTH_JSON (in $(( $(date +%s) - start_ts ))s)"

# Early out if synth‑only
if (( SYNTH_ONLY )); then log info "Stopping at synthesis as requested"; exit 0; fi

# ---------------------------------------------------------------
# Place & Route
# ---------------------------------------------------------------
ROUTED_JSON="$BUILD_DIR/${TOP_MODULE}_routed.json"
TEXTCFG="$BUILD_DIR/${TOP_MODULE}_out.config"
log pnr "Running nextpnr‑ecp5 ($NUM_THREADS threads)…"
start_ts=$(date +%s)
nextpnr_args=(
  --${DEVICE}
  --package "$PACKAGE"
  --json "$SYNTH_JSON"
  --lpf "$CONSTRAINT_FILE"
  --textcfg "$TEXTCFG"
  --write "$ROUTED_JSON"
  --threads "$NUM_THREADS"
  ${VERBOSE:+ }
)
(( ! VERBOSE )) && nextpnr_args+=(--quiet) || true
nextpnr-ecp5 "${nextpnr_args[@]}" &> "$LOG_DIR/pnr.log"
log pnr "Done → $ROUTED_JSON (in $(( $(date +%s) - start_ts ))s)"

# Check timing report
if grep -q "Timing failed" "$LOG_DIR/pnr.log"; then
  err "nextpnr timing closure FAILED"; exit 1;
fi

# ---------------------------------------------------------------
# Bitstream generation
# ---------------------------------------------------------------
BITSTREAM_FILE="$BUILD_DIR/${TOP_MODULE}.bit"
SVFSTREAM_FILE="$BUILD_DIR/${TOP_MODULE}.svf"
log bitgen "Packaging bitstream…"
start_ts=$(date +%s)
ecppack --input "$TEXTCFG" --bit "$BITSTREAM_FILE" --svf "$SVFSTREAM_FILE" &> "$LOG_DIR/bitgen.log"
log bitgen "Done → $BITSTREAM_FILE (in $(( $(date +%s) - start_ts ))s)"

# ---------------------------------------------------------------
# Success summary
# ---------------------------------------------------------------
printf "\n%s[✓] Build complete%s\n" "$(colour 2)" "$(reset_colour)"
cat <<-EOF
  Bitstream : $BITSTREAM_FILE
  SVF       : $SVFSTREAM_FILE
  Logs      : $LOG_DIR

To flash:
  openFPGALoader --board cynthion $BITSTREAM_FILE
EOF
