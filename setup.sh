#!/bin/bash
# setup.sh - Setup script for KMBoxetry development environment

set -e  # Exit on error

# Colors for better readability
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}Setting up KMBoxetry development environment...${NC}"

# Create and activate a virtual environment
if [ ! -d "venv" ]; then
    echo -e "${BLUE}Creating virtual environment...${NC}"
    python3 -m venv venv
    echo -e "${GREEN}Virtual environment created!${NC}"
else
    echo -e "${BLUE}Virtual environment exists, using existing one.${NC}"
fi

# Source the virtual environment - this doesn't work in the script itself but gives instructions
echo -e "${BLUE}Virtual environment created. To activate, run:${NC}"
echo -e "${GREEN}source venv/bin/activate${NC}"

# Instruct user to activate venv before continuing
echo -e "${BLUE}Please activate the virtual environment, then press enter to continue...${NC}"
read -p "Press enter after activating the virtual environment..."

# Verify we're in a virtual environment
if [[ "$VIRTUAL_ENV" == "" ]]; then
    echo -e "${RED}Error: Virtual environment not activated. Please run 'source venv/bin/activate' first.${NC}"
    exit 1
fi

echo -e "${BLUE}Installing Python dependencies...${NC}"
pip install -U pip  # Upgrade pip first
pip install setuptools wheel pyvcd pytest
pip install amaranth amaranth-boards
pip install git+https://github.com/greatscottgadgets/luna.git
pip install pyserial

echo -e "${BLUE}Building Rust components...${NC}"
cargo build

echo -e "${GREEN}Setup complete! Your environment is ready for development.${NC}"
echo -e "${BLUE}To build the FPGA bitstream, run:${NC}"
echo -e "${GREEN}python3 src/backend/mouse_streamer.py${NC}"
echo -e "${BLUE}To run the gateway application, run:${NC}"
echo -e "${GREEN}cargo run -- --udp 127.0.0.1:9001 --control-serial <YOUR_PORT>${NC}"
echo -e "${BLUE}For a list of serial ports, run:${NC}"
echo -e "${GREEN}cargo run -- --list${NC}"