#!/bin/bash
# =====================================================
# flash_cynthion.sh - Cynthion FPGA Flashing Script
# =====================================================
#
# This script programs the ECP5 FPGA on the Cynthion device
# with the generated bitstream file.
#
# Dependencies:
#   - ecpprog or ecpdap (ecpdap preferred)
#   - nextpnr-ecp5 toolchain to build bitstream
#

# Configuration
BITSTREAM="./build/bitstreams/cynthion_usb_sniffer.bit"
DEFAULT_INTERFACE="ecpdap"  # Options: ecpdap, ecpprog, openocd
DEFAULT_CABLE="ftdi"        # Default FTDI cable type
VERBOSE=0                   # Verbose output (0=disabled, 1=enabled)
FLASH_MODE="sram"           # Options: sram, flash
VALIDATE_HDL=1              # Validate HDL before flashing (0=disabled, 1=enabled)
# OpenOCD configuration for macOS
OPENOCD_MAC_CONFIG_DIR="/opt/homebrew/share/openocd/scripts"  # Homebrew path for Apple Silicon
if [ ! -d "$OPENOCD_MAC_CONFIG_DIR" ]; then
    OPENOCD_MAC_CONFIG_DIR="/usr/local/share/openocd/scripts"  # Homebrew path for Intel Macs
fi

# Text colors
RED="\033[0;31m"
GREEN="\033[0;32m"
YELLOW="\033[1;33m"
BLUE="\033[0;34m"
NC="\033[0m" # No Color

# Function to display usage
usage() {
    echo -e "${BLUE}Cynthion FPGA Flashing Script${NC}"
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  -b, --bitstream <file>    Specify bitstream file (default: $BITSTREAM)"
    echo "  -i, --interface <tool>    Specify programming interface (ecpdap, ecpprog, openocd)"
    echo "  -c, --cable <type>        Specify cable type"
    echo "  -m, --mode <mode>         Specify programming mode (sram, flash)"
    echo "  -v, --verbose             Enable verbose output"
    echo "  --skip-validate           Skip HDL validation before flashing"
    echo "  -h, --help                Display this help message"
    echo ""
    exit 1
}

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
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

# Function to check for required tools
check_dependencies() {
    if [ "$INTERFACE" == "ecpdap" ]; then
        if ! command_exists ecpdap; then
            error "ecpdap not found. Please install it:"
            echo "    cargo install ecpdap"
            exit 1
        fi
    elif [ "$INTERFACE" == "ecpprog" ]; then
        if ! command_exists ecpprog; then
            error "ecpprog not found. Please install it:"
            echo "    git clone https://github.com/gregdavill/ecpprog.git"
            echo "    cd ecpprog && make"
            echo "    sudo make install"
            exit 1
        fi
    elif [ "$INTERFACE" == "openocd" ]; then
        if ! command_exists openocd; then
            error "openocd not found. Please install it:"
            echo "    sudo apt install openocd   # For Debian/Ubuntu"
            echo "    brew install open-ocd      # For macOS with Homebrew"
            exit 1
        fi
    else
        error "Unsupported interface: $INTERFACE"
        usage
    fi
}

# Function to check if bitstream file exists
check_bitstream() {
    if [ ! -f "$BITSTREAM" ]; then
        error "Bitstream file not found: $BITSTREAM"
        echo "Use the -b option to specify a valid bitstream file"
        exit 1
    fi
}

# Function to detect connected Cynthion devices
detect_devices() {
    status "Detecting connected Cynthion devices..."
    
    if [ "$INTERFACE" == "ecpdap" ]; then
        ecpdap list
    elif [ "$INTERFACE" == "ecpprog" ]; then
        ecpprog -I
    elif [ "$INTERFACE" == "openocd" ]; then
        echo "Note: OpenOCD doesn't have a simple device listing command."
        echo "Using default FTDI interface."
    fi
}

# Function to flash the FPGA
flash_fpga() {
    status "Flashing FPGA with bitstream: $BITSTREAM"
    status "Using interface: $INTERFACE, mode: $FLASH_MODE"
    
    if [ "$FLASH_MODE" == "sram" ]; then
        # SRAM mode - temporary until power cycle
        if [ "$INTERFACE" == "ecpdap" ]; then
            status "Programming FPGA SRAM using ecpdap..."
            if [ $VERBOSE -eq 1 ]; then
                ecpdap program "$BITSTREAM"
            else
                ecpdap program "$BITSTREAM" > /dev/null
            fi
        elif [ "$INTERFACE" == "ecpprog" ]; then
            status "Programming FPGA SRAM using ecpprog..."
            if [ $VERBOSE -eq 1 ]; then
                ecpprog -S "$BITSTREAM"
            else
                ecpprog -S "$BITSTREAM" > /dev/null
            fi
        elif [ "$INTERFACE" == "openocd" ]; then
            status "Programming FPGA SRAM using OpenOCD..."
            TEMP_CFG="/tmp/openocd_cynthion.cfg"
            echo "interface ftdi" > $TEMP_CFG
            echo "ftdi_device_desc \"Cynthion\"" >> $TEMP_CFG
            echo "ftdi_vid_pid 0x0403 0x6010" >> $TEMP_CFG
            echo "ftdi_channel 0" >> $TEMP_CFG
            echo "transport select jtag" >> $TEMP_CFG
            echo "adapter_khz 10000" >> $TEMP_CFG
            echo "jtag newtap ecp5 tap -irlen 8 -expected-id 0x41111043" >> $TEMP_CFG
            
            # Add macOS-specific configuration
            if [[ "$(uname)" == "Darwin" ]]; then
                status "Using macOS OpenOCD configuration from: $OPENOCD_MAC_CONFIG_DIR"
                # Use appropriate path based on macOS OpenOCD installation
                if [ $VERBOSE -eq 1 ]; then
                    openocd -s "$OPENOCD_MAC_CONFIG_DIR" -f $TEMP_CFG -c "svf $BITSTREAM" -c "exit"
                else
                    openocd -s "$OPENOCD_MAC_CONFIG_DIR" -f $TEMP_CFG -c "svf $BITSTREAM" -c "exit" > /dev/null 2>&1
                fi
            else
                # Non-macOS systems
                if [ $VERBOSE -eq 1 ]; then
                    openocd -f $TEMP_CFG -c "svf $BITSTREAM" -c "exit"
                else
                    openocd -f $TEMP_CFG -c "svf $BITSTREAM" -c "exit" > /dev/null 2>&1
                fi
            fi
            rm $TEMP_CFG
        fi
    elif [ "$FLASH_MODE" == "flash" ]; then
        # Flash mode - persistent across power cycles
        if [ "$INTERFACE" == "ecpdap" ]; then
            status "Programming FPGA Flash using ecpdap..."
            if [ $VERBOSE -eq 1 ]; then
                ecpdap flash write "$BITSTREAM"
            else
                ecpdap flash write "$BITSTREAM" > /dev/null
            fi
        elif [ "$INTERFACE" == "ecpprog" ]; then
            status "Programming FPGA Flash using ecpprog..."
            if [ $VERBOSE -eq 1 ]; then
                ecpprog "$BITSTREAM"
            else
                ecpprog "$BITSTREAM" > /dev/null
            fi
        elif [ "$INTERFACE" == "openocd" ]; then
            status "Programming FPGA Flash using OpenOCD..."
            TEMP_CFG="/tmp/openocd_cynthion.cfg"
            echo "interface ftdi" > $TEMP_CFG
            echo "ftdi_device_desc \"Cynthion\"" >> $TEMP_CFG
            echo "ftdi_vid_pid 0x0403 0x6010" >> $TEMP_CFG
            echo "ftdi_channel 0" >> $TEMP_CFG
            echo "transport select jtag" >> $TEMP_CFG
            echo "adapter_khz 5000" >> $TEMP_CFG
            echo "jtag newtap ecp5 tap -irlen 8 -expected-id 0x41111043" >> $TEMP_CFG
            
            # Add macOS-specific configuration for flash programming
            if [[ "$(uname)" == "Darwin" ]]; then
                status "Using macOS OpenOCD configuration from: $OPENOCD_MAC_CONFIG_DIR"
                
                # Create script for flash programming
                echo "source [find cpld/lattice_ecp5.cfg]" >> $TEMP_CFG
                echo "ecp5 flash_spi 0 $BITSTREAM" >> $TEMP_CFG
                
                # Use appropriate path based on macOS OpenOCD installation
                if [ $VERBOSE -eq 1 ]; then
                    openocd -s "$OPENOCD_MAC_CONFIG_DIR" -f $TEMP_CFG -c "init" -c "exit"
                else
                    openocd -s "$OPENOCD_MAC_CONFIG_DIR" -f $TEMP_CFG -c "init" -c "exit" > /dev/null 2>&1
                fi
            else
                # Non-macOS systems
                if [ $VERBOSE -eq 1 ]; then
                    # Create script for flash programming
                    echo "source [find cpld/lattice_ecp5.cfg]" >> $TEMP_CFG
                    echo "ecp5 flash_spi 0 $BITSTREAM" >> $TEMP_CFG
                    
                    openocd -f $TEMP_CFG -c "init" -c "exit"
                else
                    # Create script for flash programming
                    echo "source [find cpld/lattice_ecp5.cfg]" >> $TEMP_CFG
                    echo "ecp5 flash_spi 0 $BITSTREAM" >> $TEMP_CFG
                    
                    openocd -f $TEMP_CFG -c "init" -c "exit" > /dev/null 2>&1
                fi
            fi
            rm $TEMP_CFG
        fi
    else
        error "Unsupported flash mode: $FLASH_MODE"
        usage
    fi
    
    if [ $? -eq 0 ]; then
        status "Programming successful!"
    else
        error "Programming failed with error code $?"
        exit 1
    fi
}

# Function to run post-flashing verification
verify_programming() {
    status "Verifying programming..."
    
    if [ "$INTERFACE" == "ecpdap" ] && [ "$FLASH_MODE" == "flash" ]; then
        if [ $VERBOSE -eq 1 ]; then
            ecpdap flash verify "$BITSTREAM"
        else
            ecpdap flash verify "$BITSTREAM" > /dev/null
        fi
        
        if [ $? -eq 0 ]; then
            status "Verification successful!"
        else
            error "Verification failed with error code $?"
            exit 1
        fi
    elif [ "$INTERFACE" == "ecpprog" ] && [ "$FLASH_MODE" == "flash" ]; then
        if [ $VERBOSE -eq 1 ]; then
            ecpprog -v "$BITSTREAM"
        else
            ecpprog -v "$BITSTREAM" > /dev/null
        fi
        
        if [ $? -eq 0 ]; then
            status "Verification successful!"
        else
            error "Verification failed with error code $?"
            exit 1
        fi
    else
        warning "Verification not supported for the current interface/mode combination"
    fi
}

# Function to validate HDL before flashing
validate_hdl() {
    if [ $VALIDATE_HDL -eq 1 ]; then
        status "Validating HDL code before flashing..."
        
        # Path to the validation script
        VALIDATE_SCRIPT="$(dirname "$0")/validate_hdl.sh"
        
        if [ ! -f "$VALIDATE_SCRIPT" ]; then
            warning "HDL validation script not found: $VALIDATE_SCRIPT"
            warning "Skipping HDL validation"
            return 0
        fi
        
        # Run the validation script
        if [ $VERBOSE -eq 1 ]; then
            bash "$VALIDATE_SCRIPT" --verbose
        else
            bash "$VALIDATE_SCRIPT"
        fi
        
        if [ $? -ne 0 ]; then
            error "HDL validation failed. Fix the errors before flashing."
            echo -e "${YELLOW}To skip validation, use: $0 --skip-validate${NC}"
            exit 1
        fi
        
        status "HDL validation passed. Proceeding with flashing..."
    else
        warning "HDL validation skipped"
    fi
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -b|--bitstream)
            BITSTREAM="$2"
            shift 2
            ;;
        -i|--interface)
            INTERFACE="$2"
            shift 2
            ;;
        -c|--cable)
            CABLE="$2"
            shift 2
            ;;
        -m|--mode)
            FLASH_MODE="$2"
            shift 2
            ;;
        -v|--verbose)
            VERBOSE=1
            shift
            ;;
        --skip-validate)
            VALIDATE_HDL=0
            shift
            ;;
        -h|--help)
            usage
            ;;
        *)
            error "Unknown option: $1"
            usage
            ;;
    esac
done

# Set default interface if not specified
if [ -z "$INTERFACE" ]; then
    INTERFACE="$DEFAULT_INTERFACE"
fi

# Set default cable if not specified
if [ -z "$CABLE" ]; then
    CABLE="$DEFAULT_CABLE"
fi

# Main execution
echo -e "${BLUE}=== Cynthion FPGA Flashing Tool ===${NC}"
echo ""

# Check for required dependencies
check_dependencies

# Check if bitstream file exists
check_bitstream

# Show size of bitstream file
BITSTREAM_SIZE=$(ls -lh "$BITSTREAM" | awk '{print $5}')
status "Bitstream size: $BITSTREAM_SIZE"

# Detect connected devices
detect_devices

# Validate HDL code
validate_hdl

# Flash the FPGA
flash_fpga

# Verify programming if in flash mode
if [ "$FLASH_MODE" == "flash" ]; then
    verify_programming
fi

status "All operations completed successfully!"
echo ""
echo -e "${GREEN}Cynthion USB sniffer is now programmed and ready to use.${NC}"

exit 0