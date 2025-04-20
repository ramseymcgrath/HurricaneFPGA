# KMBoxetry – Cynthion USB Tooling

[![Build Status](https://github.com/ramseymcgrath/kmboxetry/actions/workflows/get_bitstream.yml/badge.svg)](https://github.com/yourusername/kmboxetry/actions/workflows/get_bitstream.yml)
[![Code Coverage](https://codecov.io/gh/ramseymcgrath/kmboxetry/branch/main/graph/badge.svg)](https://codecov.io/gh/yourusername/kmboxetry)

> ✨ **Current status – UART‑controlled HID injection**  
> The latest bitstream (`cynthion_passthrough.py`) still provides a **USB Full‑Speed passthrough**, **plus** a tiny UART‑driven injector.  
> The Rust CLI (`packetry_injector`) now acts as a **gateway** – it listens for network commands and forwards raw 3‑byte packets over a serial link (PMOD A) to the FPGA.

KMBoxetry explores low‑level USB manipulation on the **[Cynthion FPGA](https://greatscottgadgets.com/cynthion/)**. It ships:

- **Amaranth/LUNA gateware** – FS passthrough + UART HID injection.
- **Rust CLI** – network‑to‑UART bridge for easy scripting.

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

- **USB passthrough** – Full‑/Low‑Speed packets flow between **TARGET (J2)** ⇄ **CONTROL (J3)**.
- **HID injection** – FPGA can splice one‑byte‑per‑axis mouse reports (`buttons, dx, dy`).
- **UART control** – Send _exactly_ three bytes over PMOD A @ 115200 baud to inject.
- **Rust gateway** – Accepts UDP strings (`buttons,dx,dy`) → forwards raw UART bytes.

---

## Requirements

| Category | Items |
| -------- | ----- |
| **Hardware** | Cynthion board · **UART↔USB adapter** (FT232 / CP210x) |
| **FPGA toolchain** | [OSS CAD Suite](https://github.com/YosysHQ/oss-cad-suite-build) (Yosys + nextpnr‑ecp5 + Trellis) |
| **Python** | Python 3 · `amaranth` · `luna` · `pyserial` |
| **Flashing** | `dfu-util` |
| **Rust** | Stable toolchain (`rustup`, `cargo`) |

---

## Installation (Generic build)

### 1. Environment setup

You only need docker for the initial build now, use `docker build -t amaranth-cynthion .` to build it. Still DFU to flash it though

### 2. Flash the FPGA gateware

1. **Enter DFU** – hold **USR**, tap **RST**, release **USR** (green **STAT** LED off).  
2. **Flash**
   ```bash
   dfu-util -d 1d50:615b -a 0 -D build/gateware/top.bit
   ```
3. **Reset** – press **RST**; passthrough + injector are live.

### 4. Build the Rust CLI

```bash
git clone https://github.com/yourusername/kmboxetry.git
cd kmboxetry
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

### Running the Rust gateway

```bash
# list serial ports
target/release/packetry_injector --list

# start UDP→UART bridge
target/release/packetry_injector \
    --udp 127.0.0.1:9001 \
    --control-serial /dev/ttyUSB0  # or COM3 on Windows
```

### Sending commands

Each UDP payload is an ASCII string: `buttons,dx,dy`.

```bash
# left‑click
echo "1,0,0" | nc -u -w0 127.0.0.1 9001

# move right 20 / down 5
echo "0,20,5" | nc -u -w0 127.0.0.1 9001
```

Python snippet:

```python
import socket, time
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
addr = ("127.0.0.1", 9001)

def send(btn, dx, dy):
    msg = f"{btn},{dx},{dy}".encode()
    sock.sendto(msg, addr)

send(0, 10, 0)   # move
time.sleep(0.1)
send(1, 0, 0)    # press
time.sleep(0.1)
send(0, 0, 0)    # release
```

---

## Architecture

```text
┌────────────┐  UDP  ┌──────────────────┐  Serial  ┌────────────┐
│ Script/App │ ────► │ Rust Gateway CLI │ ───────► │ UART Dongle│
└────────────┘       │ packetry_injector│          │ (FT232)   │
                     └────────┬─────────┘          └─────┬──────┘
                              │ 115200 8N1               │
                              ▼                          ▼
                      ┌──────────────────────────────────────────┐
                      │  Cynthion FPGA (ECP5)                    │
                      │  • ULPI PHY J2 ↔ Host PC                │
                      │  • ULPI PHY J3 ↔ Target Device          │
                      │  • UART Rx/Tx ↔ PMOD A                  │
                      │  • Amaranth/LUNA passthrough + injector │
                      └──────────────────────────────────────────┘
```

---

## Development

```bash
# Rust
cargo build && cargo test

# Gateware (needs Python env)
python src/backend/cynthion_passthrough.py
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

<details>
<summary>Gateway running but nothing happens</summary>

* `--control-serial` port correct? Use `--list`.
* Baud mismatch – pass `--control-baud 115200` if changed.
* Verify TX/RX wiring on PMOD A.
* Confirm UDP target IP/port & firewall.
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

