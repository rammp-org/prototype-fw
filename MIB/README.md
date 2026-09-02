# MIB (Motor Interface Board) Firmware

ESP-IDF firmware project for the **MIB** prototype using the ESP32-P4-ETH board and TWAI (CAN bus) to communicate with a Basicmicro MCP266 (RoboClaw family) motor controller to drive two DC motors and read quadrature encoder feedback in real time.

## Overview

- **Target MCU / Board**: Waveshare ESP32-P4-ETH (ESP32-P4)
- **Motor Controller**: Basicmicro MCP266 (RoboClaw family) dual-channel motor controller
- **Bus Interface**: TWAI (Two-Wire Automotive Interface / CAN 2.0)
- **Networking**: 10/100 Ethernet via internal EMAC and IP101GRI RMII PHY

## Features

- Dual-channel (`M1` / `M2`) motor controller (`mib::Mcp266Controller`) over **CANopen (CiA 301 / CiA 402)**.
- **Position control** via CiA 402 profile position mode (`move_to_position`, `configure_position_loop`, `set_position_limits`) for both axes — validated and reliable.
- Real-time encoder, speed, and statusword feedback (`read_encoder`, `read_speed`, `read_statusword`) for both axes.
- Telemetry: main battery voltage and board temperature, read over CAN via the manufacturer command mirror.
- Board support integration via `espp::Esp32P4Eth` with link state monitoring and IP reporting.

### Control notes / known limitations

- The MCP266's control-loop parameters are **not** standard CiA 402 objects. The MCP mirrors its packet-serial command set into the manufacturer region at object index `0x2000 + command number`; this driver uses that to configure the position PID (commands 61–64) and read telemetry (24 / 82).
- The position loop's `MinPos`/`MaxPos` clamp defaults to `[0, 0]` (every target forced to zero) and its P gain can read back as zero; `configure_position_loop()` widens the clamp and seeds a non-zero P gain on each boot (the MCP reverts to EEPROM at power-up). Note the position-PID setter (cmds 61/62) uses field order `D, P, I` while the readback (63/64) uses `P, I, D`.
- **Velocity / duty over CAN is not functional** on the tested MCP266 firmware. Both the standard target objects (`0x60FF` / `0x6071`) and the mirrored manufacturer speed/duty commands (`0x2023` / `0x2020`) are accepted but leave the velocity generator idle (`0x606B` stays 0) even with the drive in Operation Enabled. Supported-drive-modes (`0x6502 = 0x700`) advertises only the cyclic-sync modes (csp/csv/cst), so velocity likely requires csv mode with cyclic SYNC/PDO updates — undocumented for this device. `drive_speed`/`drive_duty` are implemented but currently a no-op for motion; use position mode. A Basicmicro CANopen manual / EDS would resolve this.

## Pinout & Wiring

| Interface | Signal | ESP32-P4 GPIO | Description |
|-----------|--------|---------------|-------------|
| **TWAI / CAN** | TX | GPIO 17 | CAN Transceiver TXD (1 Mbps bus speed) |
| **TWAI / CAN** | RX | GPIO 16 | CAN Transceiver RXD (1 Mbps bus speed) |
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
