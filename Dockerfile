# Use Python 3.11 slim as the base image
FROM python:latest

# Basic build tools and fonts (for graphviz if needed)
RUN apt-get update && \
    apt-get install --no-install-recommends -y \
        build-essential \
        graphviz \
        pkg-config \
        libgraphviz-dev \
        && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y libusb-1.0-0

# Install yowasp-yosys FIRST (gets the right wheel for this OS/arch)
RUN pip install --upgrade pip \
    && pip install yowasp-yosys

# Install Python dependencies
RUN pip install --no-cache-dir \
    amaranth \
    amaranth-boards \
    pyserial \
    setuptools \
    wheel \
    pyvcd \
    pytest \
    pygreat \
    cynthion \
    yowasp-nextpnr-ecp5 \
    yowasp-yosys 
RUN pip install git+https://github.com/greatscottgadgets/luna.git

# Copy your project files into the container (change path if needed)
COPY . /work
WORKDIR /work
ENV LUNA_PLATFORM="cynthion.gateware.platform:CynthionPlatformRev0D4"
# Default command (change if needed)
CMD ["python", "src/backend/mouse_streamer.py"]