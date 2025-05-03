#!/bin/bash
# =====================================================
# validate_hdl.sh - Verilog/HDL Syntax Validation Script
# =====================================================
#
# This script validates the syntax of all Verilog HDL files
# in the project before flashing to the FPGA.
#
# Dependencies:
#   - Icarus Verilog (iverilog) for syntax checking
#

# Text colors
RED="\033[0;31m"
GREEN="\033[0;32m"
YELLOW="\033[1;33m"
BLUE="\033[0;34m"
NC="\033[0m" # No Color

# Configuration
VERBOSE=0                   # Verbose output (0=disabled, 1=enabled)
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RTL_DIR="${PROJECT_ROOT}/hardware/rtl"
INCLUDE_DIR="${PROJECT_ROOT}/hardware/rtl"

# Function to display usage
usage() {
    echo -e "${BLUE}Verilog HDL Syntax Validation Script${NC}"
    echo "Usage: $0 [options] [file1.v file2.v ...]"
    echo ""
    echo "Options:"
    echo "  -v, --verbose             Enable verbose output"
    echo "  -d, --directory <path>    Specify a custom directory to scan (default: ${RTL_DIR})"
    echo "  -h, --help                Display this help message"
    echo ""
    echo "If specific files are provided, only those will be checked."
    echo "Otherwise, all .v files in the RTL directory will be checked."
    echo ""
    exit 1
}

# Function to display status messages
status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

# Function to display warning messages
warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# Function to display error messages
error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to install required tools
install_tools() {
    if ! command_exists iverilog; then
        status "Icarus Verilog not found. Attempting to install..."
        
        if command_exists brew; then
            status "Installing Icarus Verilog via Homebrew..."
            brew install icarus-verilog
            
            if [ $? -ne 0 ]; then
                error "Failed to install Icarus Verilog. Please install it manually:"
                echo "    brew install icarus-verilog"
                exit 1
            fi
        else
            error "Homebrew not found. Please install it first:"
            echo "    /bin/bash -c \"\$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)\""
            echo "Then install Icarus Verilog:"
            echo "    brew install icarus-verilog"
            exit 1
        fi
    fi
}

# Function to find all Verilog files in a directory
find_verilog_files() {
    find "$1" -name "*.v" -type f | sort
}

# Function to validate a single Verilog file
validate_file() {
    local file="$1"
    local relative_path="${file#$PROJECT_ROOT/}"
    
    if [ $VERBOSE -eq 1 ]; then
        status "Checking: $relative_path"
    fi
    
    # Run Icarus Verilog in syntax-check-only mode
    iverilog -t null -I"$INCLUDE_DIR" -y"$RTL_DIR" -y"$RTL_DIR/usb_interface" -y"$RTL_DIR/usb_proxy" "$file" 2> /tmp/iverilog_errors.txt
    
    if [ $? -eq 0 ]; then
        if [ $VERBOSE -eq 1 ]; then
            echo -e "  ${GREEN}✓ Syntax OK${NC}"
        fi
        return 0
    else
        echo -e "  ${RED}✗ Syntax errors in: $relative_path${NC}"
        cat /tmp/iverilog_errors.txt | sed 's/^/    /'
        return 1
    fi
}

# Function to validate all files and return overall status
validate_all_files() {
    local files=("$@")
    local error_count=0
    local file_count=0
    
    for file in "${files[@]}"; do
        validate_file "$file"
        if [ $? -ne 0 ]; then
            ((error_count++))
        fi
        ((file_count++))
    done
    
    echo ""
    if [ $error_count -eq 0 ]; then
        status "All $file_count Verilog files passed syntax validation."
        return 0
    else
        error "$error_count of $file_count Verilog files contain syntax errors."
        return 1
    fi
}

# Parse command line arguments
CUSTOM_DIR=""
FILES=()

while [[ $# -gt 0 ]]; do
    case $1 in
        -v|--verbose)
            VERBOSE=1
            shift
            ;;
        -d|--directory)
            CUSTOM_DIR="$2"
            shift 2
            ;;
        -h|--help)
            usage
            ;;
        -*)
            error "Unknown option: $1"
            usage
            ;;
        *)
            FILES+=("$1")
            shift
            ;;
    esac
done

# Main execution
echo -e "${BLUE}=== Verilog HDL Syntax Validation Tool ===${NC}"
echo ""

# Check for required dependencies
install_tools

# Determine which files to validate
if [ ${#FILES[@]} -eq 0 ]; then
    if [ -n "$CUSTOM_DIR" ]; then
        status "Scanning directory: $CUSTOM_DIR"
        # Use a more portable approach instead of mapfile
        FILES=()
        while IFS= read -r line; do
            FILES+=("$line")
        done < <(find_verilog_files "$CUSTOM_DIR")
    else
        status "Scanning RTL directory: $RTL_DIR"
        # Use a more portable approach instead of mapfile
        FILES=()
        while IFS= read -r line; do
            FILES+=("$line")
        done < <(find_verilog_files "$RTL_DIR")
    fi
    
    if [ ${#FILES[@]} -eq 0 ]; then
        error "No Verilog (.v) files found!"
        exit 1
    fi
    
    status "Found ${#FILES[@]} Verilog files to validate."
else
    status "Validating ${#FILES[@]} specified Verilog files."
fi

echo ""
# Validate all files
validate_all_files "${FILES[@]}"
EXIT_CODE=$?

# Final message
if [ $EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}HDL validation completed successfully!${NC}"
    echo "Your Verilog code is syntactically correct and ready for flashing."
else
    echo -e "${RED}HDL validation failed.${NC}"
    echo "Please fix the syntax errors before flashing to the FPGA."
fi

exit $EXIT_CODE