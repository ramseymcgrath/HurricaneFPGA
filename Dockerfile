# Use Python 3.11 slim as the base image
FROM python:latest

# Basic build tools and fonts (for graphviz if needed)
RUN apt-get update
RUN apt-get install build-essential graphviz pkg-config libgraphviz-dev libusb-1.0-0
RUN rm -rf /var/lib/apt/lists/*

# Install yowasp-yosys FIRST (gets the right wheel for this OS/arch)
RUN pip install --upgrade pip \
    && pip install yowasp-yosys

# Install Python dependencies
RUN pip install amaranth amaranth-boards pyserial setuptools wheel pyvcd pytest pygreat cynthion yowasp-nextpnr-ecp5 yowasp-yosys 
RUN pip install git+https://github.com/greatscottgadgets/luna.git

# Copy your project files into the container (change path if needed)
COPY . /work
WORKDIR /work
ENV LUNA_PLATFORM="cynthion.gateware.platform:CynthionPlatformRev1D4"
ENV BUILD_LOCAL="1"
# Default command (change if needed)
CMD ["python", "src/backend/mouse_streamer.py"]