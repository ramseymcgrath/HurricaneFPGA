#!/bin/bash
# Verilog HDL Syntax Validation Script

set -euo pipefail

# Text colors
RED="\033[0;31m"
GREEN="\033[0;32m"
YELLOW="\033[1;33m"
BLUE="\033[0;34m"
NC="\033[0m"

VERBOSE=0
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RTL_DIR="${PROJECT_ROOT}/hardware/rtl"
INCLUDE_DIR="${PROJECT_ROOT}/hardware/rtl"
CUSTOM_DIR=""
FILES=()

# Display usage
usage() {
    echo -e "${BLUE}Verilog HDL Syntax Validation Script${NC}"
    echo "Usage: $0 [options] [file1.v file2.v ...]"
    echo "Options:"
    echo "  -v, --verbose             Verbose output"
    echo "  -d, --directory <path>    Custom directory (default: ${RTL_DIR})"
    echo "  -h, --help                Show this help"
    exit 1
}

# Logging helpers
status() { echo -e "${GREEN}[INFO]${NC} $1"; }
warning() { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; }

command_exists() {
    command -v "$1" >/dev/null 2>&1
}

install_tools() {
    if ! command_exists iverilog; then
        error "Icarus Verilog (iverilog) is not installed."

        case "$OSTYPE" in
            linux*)
                echo "To install on Linux:"
                echo "  Debian/Ubuntu: sudo apt install iverilog"
                echo "  Arch:          sudo pacman -S iverilog"
                echo "  Fedora:        sudo dnf install iverilog"
                ;;
            darwin*)
                echo "To install on macOS:"
                echo "  brew install icarus-verilog"
                ;;
            *)
                echo "Please install 'iverilog' manually for your OS."
                ;;
        esac

        exit 1
    fi
}

find_verilog_files() {
    find "$1" -name "*.v" -type f | sort
}

validate_file() {
    local file="$1"
    local relative_path="${file#$PROJECT_ROOT/}"

    if [[ $VERBOSE -eq 1 ]]; then
        status "Checking: $relative_path"
    fi

    iverilog -t null -I"$INCLUDE_DIR" \
        -y"$RTL_DIR" -y"$RTL_DIR/usb_interface" -y"$RTL_DIR/usb_proxy" \
        "$file" 2> /tmp/iverilog_errors.txt

    if [[ $? -eq 0 ]]; then
        [[ $VERBOSE -eq 1 ]] && echo -e "  ${GREEN}✓ Syntax OK${NC}"
    else
        echo -e "  ${RED}✗ Syntax error in: $relative_path${NC}"
        sed 's/^/    /' /tmp/iverilog_errors.txt
        return 1
    fi
}

validate_all_files() {
    local files=("$@")
    local errors=0

    for file in "${files[@]}"; do
        validate_file "$file" || ((errors++))
    done

    echo ""
    if [[ $errors -eq 0 ]]; then
        status "All ${#files[@]} Verilog files passed validation."
        return 0
    else
        error "$errors of ${#files[@]} files have syntax errors."
        return 1
    fi
}

# === Parse CLI Args ===
while [[ $# -gt 0 ]]; do
    case "$1" in
        -v|--verbose) VERBOSE=1; shift ;;
        -d|--directory) CUSTOM_DIR="$2"; shift 2 ;;
        -h|--help) usage ;;
        -*) error "Unknown option: $1"; usage ;;
        *) FILES+=("$1"); shift ;;
    esac
done

echo -e "${BLUE}=== Verilog HDL Syntax Validation Tool ===${NC}"
echo ""

install_tools

# Collect files
if [[ ${#FILES[@]} -eq 0 ]]; then
    TARGET_DIR="${CUSTOM_DIR:-$RTL_DIR}"
    status "Scanning directory: $TARGET_DIR"
    while IFS= read -r line; do FILES+=("$line"); done < <(find_verilog_files "$TARGET_DIR")
    [[ ${#FILES[@]} -eq 0 ]] && { error "No Verilog files found!"; exit 1; }
    status "Found ${#FILES[@]} Verilog files to check."
else
    status "Validating ${#FILES[@]} user-specified file(s)."
fi

# Validate
validate_all_files "${FILES[@]}"
EXIT_CODE=$?

if [[ $EXIT_CODE -eq 0 ]]; then
    echo -e "${GREEN}HDL validation complete — no syntax errors found.${NC}"
else
    echo -e "${RED}HDL validation failed. Fix errors above.${NC}"
fi

exit $EXIT_CODE
