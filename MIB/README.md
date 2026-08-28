# MIB (Motor Interface Board) Firmware

ESP-IDF firmware project for the **MIB** prototype using the ESP32-P4-ETH board and TWAI (CAN bus) to communicate with a Basicmicro MCP266 (RoboClaw family) motor controller to drive two DC motors and read quadrature encoder feedback in real time.

## Overview

- **Target MCU / Board**: Waveshare ESP32-P4-ETH (ESP32-P4)
- **Motor Controller**: Basicmicro MCP266 (RoboClaw family) dual-channel motor controller
- **Bus Interface**: TWAI (Two-Wire Automotive Interface / CAN 2.0)
- **Networking**: 10/100 Ethernet via internal EMAC and IP101GRI RMII PHY

## Features

- Transport-agnostic motor driver controller (`mib::Mcp266Controller`) supporting both **Basicmicro Packet Serial over TWAI** and **CANopen (CiA 301 / CiA 402)** modes.
- Closed-loop velocity control (`drive_speed`) and duty-cycle drive (`drive_duty`) for Motor 1 and Motor 2.
- Real-time encoder readback (`read_encoders`) for Motor 1 and Motor 2 position and velocity feedback.
- Telemetry monitoring including main battery voltage, driver board temperature, and error status bitmasks.
- Board support integration via `espp::Esp32P4Eth` with link state monitoring and IP reporting.

## Pinout & Wiring

| Interface | Signal | ESP32-P4 GPIO | Description |
|-----------|--------|---------------|-------------|
| **TWAI / CAN** | TX | GPIO 16 | CAN Transceiver TXD (1 Mbps bus speed) |
| **TWAI / CAN** | RX | GPIO 17 | CAN Transceiver RXD (1 Mbps bus speed) |
| **Ethernet** | RMII PHY | Standard RMII | Board default (50 MHz REF_CLK on GPIO 50, PHY RST on GPIO 51) |

*Note: Ensure proper 120 Ω CAN bus termination resistors are present on the TWAI network.*

## Build & Flash

Ensure your ESP-IDF environment is exported (v5.3+ required):

```bash
source $IDF_PATH/export.sh
```

To build, flash, and monitor the project:

```bash
cd prototype-fw/MIB
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```
