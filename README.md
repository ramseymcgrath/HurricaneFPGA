# HurricaneFPGA – HID Injection Tools

[![Build Status](https://github.com/ramseymcgrath/HurricaneFPGA/actions/workflows/get_bitstream.yml/badge.svg)](https://github.com/ramseymcgrath/HurricaneFPGA/actions/workflows/build_and_test.yml)
[![Code Coverage](https://codecov.io/gh/ramseymcgrath/HurricaneFPGA/branch/main/graph/badge.svg)](https://codecov.io/gh/yourusername/kmboxetry)

> ✨ **Current status – Full HDL Implementation Released!**  
> The project now provides **two implementations**:
> - **New HDL Implementation** (`HDL/`) - A complete Verilog implementation with enhanced features
> - **Legacy Python/Luna** (`legacy/`) - Original amaranth/LUNA implementation with UART-controlled HID injection
>
> Both provide USB passthrough, HID injection capabilities, and command interfaces.

HurricaneFPGA explores low‑level USB manipulation on the **[Cynthion FPGA](https://greatscottgadgets.com/cynthion/)**. It ships:

- **Hurricane HDL** – Complete Verilog implementation for advanced USB manipulation
- **Legacy Amaranth/LUNA gateware** – FS passthrough + UART HID injection
- **Rust CLI** – network‑to‑UART bridge for easy scripting

---

## Table of Contents
1. [Features](#features)  
2. [Requirements](#requirements)  
3. [Installation](#installation)  
   3.1 [Environment setup](#1-environment-setup) | 3.2 [Build gateware](#2-build-the-fpga-gateware) | 3.3 [Flash gateware](#3-flash-the-fpga-gateware) | 3.4 [Build Rust CLI](#4-build-the-rust-cli)  
4. [Hardware setup](#hardware-setup)  
5. [Usage](#usage)  
   5.1 [Run gateway](#running-the-rust-gateway) | 5.2 [Send commands](#sending-commands)  
6. [Architecture](#architecture)  
7. [Development](#development)  
8. [Troubleshooting](#troubleshooting)  
9. [License](#license)  
10. [Acknowledgements](#acknowledgements)

---

## Features

- **USB passthrough** – Full‑/Low‑Speed packets flow between **TARGET (J2)** ⇄ **CONTROL (J3)**.
- **HID injection** – FPGA can splice one‑byte‑per‑axis mouse reports (`buttons, dx, dy`).
- **UART control** – Send _exactly_ three bytes over PMOD A @ 115200 baud to inject.
- **Rust gateway** – Accepts UDP strings (`buttons,dx,dy`) → forwards raw UART bytes.
- **Command acknowledgment** – FPGA sends ACK/NAK responses with error codes.

---

## Requirements

| Category | Items |
| -------- | ----- |
| **Hardware** | Cynthion board · **UART↔USB adapter** (FT232 / CP210x) |
| **FPGA toolchain** | [OSS CAD Suite](https://github.com/YosysHQ/oss-cad-suite-build) (Yosys + nextpnr‑ecp5 + Trellis) |
| **Python** | Python 3 · `amaranth` · `luna` · `pyserial` |
| **Rust** | Stable toolchain (`rustup`, `cargo`) |

---

## Installation (Generic build)

### 1. Environment setup

You only need docker for the initial build now, use `docker build -t amaranth-cynthion .` to build it. Still DFU to flash it though

### 2. Flash the FPGA gateware

#### Legacy Python/Luna Implementation
Python should be able to run this with `python legacy/src/flash_fpga.py`

#### Hurricane HDL Implementation
The HDL implementation provides dedicated flashing tools:
```bash
cd HDL/tools
./flash_cynthion.sh  # Flash the precompiled bitstream
```

For advanced builds and debugging:
```bash
# Compile a custom bitstream
./compile_bitstream.sh

# Validate HDL before building
./validate_hdl.sh

# Debug connected Cynthion
python cynthion_debugger.py
```

See `HDL/architecture.md` and `HDL/transparent_proxy_implementation.md` for detailed documentation on the HDL implementation.

### 4. Build the Rust CLI

```bash
git clone https://github.com/ramseymcgrath/hurricanefpga.git
cd src
cargo build --release
```
Binary: `target/release/packetry_injector`.

---

## Hardware setup

| Connection | Details |
| ---------- | ------- |
| Host PC USB | → **TARGET (J2)** |
| USB device (mouse, etc.) | → **CONTROL (J3)** |
| UART adapter TX | → PMOD A Pin 1 (FPGA RX) |
| UART adapter RX | → PMOD A Pin 2 (FPGA TX) |
| Ground | → PMOD A Pin 5/6 |

> Default UART settings: **115200 8N1**.

---

## Usage

### Running the Rust gateway

```bash
# list serial ports
target/release/packetry_injector --list

# start UDP→UART bridge
target/release/packetry_injector \
    --udp 127.0.0.1:9001 \
    --control-serial /dev/ttyUSB0  # or COM3 on Windows
```

### Command Acknowledgments

The FPGA now provides acknowledgments for each command sent:
- **ACK (0x06)** - Command was successfully received and processed
- **NAK (0x15) + Error Code** - Command failed with specific error code:
  - `0x01` - Value out of range
  - `0x02` - Syntax error
  - `0x03` - System busy
  - `0x04` - Buffer overflow

The Rust CLI automatically handles these acknowledgments.

---

## Architecture

```text
                     ┌──────────────────┐  Serial  ┌────────────┐
                     │ Rust Gateway CLI │ ───────► │ UART Dongle│
                     │ packetry_injector│          │ (FT232)   │
                     └──────────────────┘          └─────┬──────┘
                                                         │
                                                         ▼
                      ┌────────────────────────────────────────┐
       ─────────────> │  Cynthion FPGA (ECP5)                  │
      |DEVICE|        │  • ULPI AUX  ↔ Host PC                 │
                      │  • ULPI HOST ↔ Target Device           │
                      │  • UART Rx/Tx ↔ PMOD A                 │
                      │  • Amaranth/LUNA passthrough/injector  │
                      └────────────────────────────────────────┘
                                                         |
                                                         |
                                                         ▼
                                                      HOST PC

```

---

## Development

```bash
# Rust
cargo build && cargo test

# Gateware (needs Python env)
python src/backend/top.py
```

### Coverage

```bash
cargo install cargo-tarpaulin
cargo tarpaulin --out Html --output-dir coverage \
                --packages packetry_injector
open coverage/tarpaulin-report.html
```

---

## Troubleshooting

<details>
<summary>USB device on J3 not detected by host</summary>

* Re‑flash correct bitstream & reset.
* Check cabling: Host ↔ J2, Device ↔ J3.
* Only FS/LS devices work.
* Ensure VBUS on J3 (jumper) or self‑powered device.
</details>


### udev rules (Linux)

```udev
# Cynthion DFU
SUBSYSTEM=="usb", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="615b", MODE="0666"
# CP210x example
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout"
# FT232 example
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666", GROUP="dialout"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## License

Distributed under the terms of the **MIT License** – see `LICENSE`.

## Acknowledgements

* **Cynthion** by *Great Scott Gadgets*.
* Built with **Amaranth HDL**, **LUNA USB framework**, and a stack of fantastic Rust crates.
