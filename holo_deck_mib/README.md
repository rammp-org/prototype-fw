# Holo Deck MIB (ESP32-P4-ETH)

The holonomic-drive platform's Main Interface Board: a Waveshare **ESP32-P4-ETH**
kit that drives the four Reflex RMD-X6-S2 wheel motors over CAN and talks to the
joystick HMI ([pace-hmi-fw](https://github.com/rammp-org/pace-hmi-fw)) over
Ethernet with the shared RTPS messages ([rammp-rtps](https://github.com/rammp-org/rammp-rtps),
the `external/rammp-rtps` submodule).

It is the `holo_deck` (M5Stack Tab5) firmware with the touch GUI and the local
ADC joystick replaced by the RTPS link: same kinematics (`holo_deck_platform`),
same motor protocol (`motor_actuator`), same controller (a copy of
`holo_deck_controller` with the motor status extended for diagnostics).

## Architecture

```
 pace-hmi-fw (Tab5)                        holo_deck_mib (ESP32-P4-ETH)
 ┌────────────────┐   rammp/joystick/xy_twist  (~30 Hz)   ┌──────────────┐
 │ stick, buttons │ ────────────────────────────────────▶ │ RtpsLink     │
 │ drive screen   │   rammp/joystick/drive_command        │   ▼          │
 │                │ ────────────────────────────────────▶ │ HoloDeck-    │  CAN 1 Mbit/s
 │ status labels  │ ◀──────────────────────────────────── │ Controller ──┼──▶ 4x RMD-X6-S2
 │ diagnostics    │   rammp/mib/status       (2 Hz)       │ (50 Hz loop) │    (TwaiMotorBus
 │                │ ◀──────────────────────────────────── │              │     on espp::Twai)
 └────────────────┘   rammp/mcb/diagnostics  (2 Hz)       └──────────────┘
```

| file | what it does |
| --- | --- |
| `main/rtps_link.*` | the RTPS participant, the joystick subscriptions, the status / diagnostics publishers, the joystick watchdog |
| `main/holo_deck_controller.*` | command state + the 50 Hz control loop (copied from `holo_deck`) |
| `main/twai_motor_bus.*` | RMD motor transport on `espp::Twai` (request / reply matching) |
| `main/hw_config.hpp` | pins, limits, drive profiles, link timing |
| `main/main.cpp` | Ethernet bring-up, wiring, the UART CLI |

### How the HMI's messages are used

- **XYTwist** (x + = right, y + = forward, twist + = clockwise, deadzoned on the
  HMI) becomes the controller's joystick input (forward, left, counter-clockwise)
  scaled by the active profile's limits. It only drives while the controller is
  in DRIVE.
- **DriveCommand** ENABLE puts the controller in DRIVE (the stick must be
  centered first, so nothing lurches); DISABLE e-stops it (zero velocity, motors
  hold). The profile it carries (LOW / NORMAL / HIGH) selects the speed and
  rotation limits in `hw_config.hpp`.
- **SeatCommand** is logged once and ignored: the platform has no seat.
- **MibStatus** every 500 ms: `INITIALIZING` until the CAN bus is up, `ENABLED`
  while driving, `IDLE` otherwise, `ERROR` with a message when the bus failed;
  `activeProfile`; seat fields zero; `speed` = commanded translation speed; `seq`.
  The HMI opens its drive screen on `ENABLED` and declares the link lost after
  2 s without this message.
- **Diagnostics** every 500 ms: one item per `RAMMP_DIAG_TABLE` row from motors
  1..3's status polls: temperature (0.1 C), torque raw (in the "Current" column),
  output angle (0.1 deg).

### Safety

- Boots e-stopped; only a DriveCommand ENABLE (or the CLI) starts driving.
- **Joystick watchdog**: no XYTwist for 500 ms while driving commands zero
  velocity; none for 2 s e-stops and reports `JOYSTICK LOST` in MibStatus until
  the HMI sends ENABLE again.
- Every motor command comes from the single control loop, so an e-stop can never
  be overtaken by an in-flight drive command.
- Wheel speeds are scaled together so none exceeds `kMaxWheelRpm`.

## Wiring

| signal | GPIO |
| --- | --- |
| TWAI TX → transceiver TXD | 17 |
| TWAI RX ← transceiver RXD | 16 |

3.3 V CAN transceiver (SN65HVD230 or similar) on a 120 Ω-terminated bus at
1 Mbit/s; the transceiver's standby / enable pin must be in its active state.
Swap the two constants in `hw_config.hpp` if the board is wired the other way.

## Network

The HMI is a DHCP client. `idf.py menuconfig` → **Holo Deck MIB Configuration**:

- **DHCP server** (default): this board serves 192.168.4.1/24, for a direct cable
  to the HMI (the chair, no router).
- **DHCP client**: for a bench LAN with a router, so the PC tools from
  pace-hmi-fw (`scripts/rtps_mcb_gui.py`, `rtps_adc_plot.py`, ...) share the
  subnet with both boards.

RTPS discovery is multicast on the local segment; nothing needs to know the
other side's address.

## Build & flash

```sh
git submodule update --init            # external/rammp-rtps
cd holo_deck_mib
idf.py set-target esp32p4
idf.py -p <PORT> flash monitor
```

The console (logs + CLI) is on the kit's USB-UART. CLI: `status`, `enable`,
`estop`, `disable`, `velocity <x> <y> <w>`, `limits <mps> <rpm>`.

## Bench test without the chair

1. Flash, connect the HMI (or a PC running pace-hmi-fw's `rtps_mcb_gui.py` peer
   tools) on the same segment; `status` shows `hmi: matched` once discovery
   completes and `last XYTwist ... ms ago` while the stick streams.
2. On the HMI, push the stick up and hold: it sends DriveCommand ENABLE; the
   MIB logs it, MibStatus goes `ENABLED`, and the HMI's drive screen opens.
3. Deflect the stick: `status` shows `source: JOYSTICK` and the commanded
   velocity; the motors follow. Center it: after 500 ms control returns to the
   (zeroed) local setpoint.
4. Unplug the Ethernet cable while driving: within 500 ms the platform holds
   zero, after 2 s it e-stops and MibStatus (once the link is back) shows
   `JOYSTICK LOST`.
