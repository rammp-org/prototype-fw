# STEM_V3

ESP-IDF firmware for the STEM prototype: a planar 5-bar seat linkage (see
`Geometry.md`) driven by three reversed pairs of Eyou harmonic rotary actuators
(CANopen/DS402) over CAN, on the Waveshare ESP32-P4-ETH. Same linkage/IK/USB
protocol design as the original `STEM` project (RMD-X6-S2 motors); this variant
swaps the actuator layer for `EyouMotor`/`PairedEyouActuator`.

- **Board**: Waveshare ESP32-P4-ETH (ESP32-P4, 16 MB flash), `espp::Esp32P4Eth`
- **Motors**: Eyou harmonic rotary actuators, CANopen/DS402 profile-position
  mode, CAN on GPIO 16 (TX) / 17 (RX)
- **Pairs**: left = motors 2/1, right = 4/3, seat = 6/5 (`PairedEyouActuator`,
  the secondary motor runs mirrored)
- **Speed**: EyouMotor moves at a pre-configured profile velocity (set once at
  boot via `EyouMotor::configure_profile_position`), not a per-call speed; the
  `rpm` parameters in the CLI/USB protocol are accepted for compatibility but
  ignored.

## What it does

- **Kinematics** (`main/ik_5bar.*`): inverse kinematics from an end-effector
  target (M3) to crank angles, and forward kinematics back from crank angles
  to every joint. Both enforce the two assembly-mode invariants of the built
  machine (crossed cranks, seat bar on the CCW side of J1→J2) and pick the
  branch nearest the documented calibration pose. Verified against the joint
  tables in `Geometry.md` by `test/kinematics_host_test.cpp`.
- **Controller** (`main/stem_controller.*`): the one place that turns targets
  into pair commands (IK reference convention, pair sign convention, position
  limits, tracked target / pose). Shared by the CLI and the USB module; moves
  whose pair angles fall outside the limits are **rejected**, not clamped.
- **Console CLI** (USB-Serial-JTAG / UART0): `move <x_rel> <y_rel>`, `home`,
  `stop`, `set <pair> <deg>`, `get`, `status`, `release`, `zero <pair|all>`,
  `zero_align <pair|all>`, `ik <x> <y>`, `ik_ref <x_rel> <y_rel>`,
  `torque_limit <permille>`, `set_node_id <old> <new>`, `can`, `can_recover`.
- **Safety, from the single-actuator test stand and the PEAR base**
  (`main/stem_controller.*`, `main/Kconfig.projbuild`):
  - every motor's **torque limit** (CANopen Max torque 0x6072, per-thousand
    of the rated torque) is written before the drives are enabled
    (`STEM_BOOT_TORQUE_PERMILLE`, default 300) and may be raised from the
    console / CLI up to the ceiling the firmware was built with
    (`STEM_MAX_TORQUE_PERMILLE`, default 500) - a browser cannot ask the
    linkage to push harder than the person who flashed it allowed; a move
    cannot exceed the limit in any mode;
  - **STOP halts** the drives with the profile deceleration and holds the
    linkage there, drives enabled (`EyouMotor::halt`); a pair that does not
    confirm the halt is disabled and coasts. The old stop disabled everything,
    and the linkage sagged under its own weight. A set-point nobody
    acknowledged, stuck in the TWAI queue, is flushed first so the halt does
    not queue behind it;
  - the drives are put in profile-position mode **before** the DS402 enable
    sequence (a fresh Eyou drive refuses the enable otherwise);
  - the **CAN controller's** error state and counters (TWAI; wiring /
    bitrate diagnosis) are in the telemetry and the CLI `can`; bus-off
    recovers automatically (`can_recover` by hand). The CAN driver retries a
    frame that lost arbitration to a drive's reply instead of dropping it;
  - `set_node_id` re-addresses a replacement actuator in place (both Node-ID
    objects, saved, communication reset), then that motor object follows it.
- **Native USB** (`main/main.cpp`): an espp *dispatcher* device on the P4's
  USB-OTG port — a composite WebUSB vendor interface + CDC-ACM port, each
  carrying [`stream_frame`](https://github.com/esp-cpp/espp/tree/main/components/stream_frame)
  traffic routed by module id, with capability discovery for the espp Device Hub:

  | module | protocol | browser console |
  |---|---|---|
  | 0 | espp OTA (`ota_stream`) | [ota_console](https://esp-cpp.github.io/espp/apps/ota_console.html) |
  | 4 | espp core dump (`CoreDumpService`) | [coredump_console](https://esp-cpp.github.io/espp/apps/coredump_console.html) |
  | 7 | STEM (`main/stem_protocol.hpp`) | `web/stem_console.html` |

  USB VID:PID `1209:0d3a`, product "STEM Linkage". OTA updates alternate
  between two 4 MB app slots with rollback (`partitions.csv`); panics are
  saved to the `coredump` partition and reported on the next boot / served to
  the core-dump console.
- **Web console** (`web/stem_console.html`): draws the linkage from the
  firmware's measured pose, lets you drag the end effector to move it, sends
  direct pose / pair commands, sets the torque limit, and streams motor
  telemetry (DS402 state, position, torque, velocity) and the CAN controller's
  state. Protocol v2 (`main/stem_protocol.hpp`); a v1 device still works
  without the new fields. See `web/README.md`.

Note on "current pose": each Eyou motor reports its actual output position
over CANopen (object 0x6064, relative to the saved software zero), and the
firmware reads the pairs live; the pose it reports is the FK of those measured
positions. The DS402 statusword (target-reached bit) is the "still moving"
indicator.

## Build, flash, test

```console
idf.py build
idf.py -p PORT flash monitor
```

`sdkconfig.defaults` selects the 16 MB flash / OTA partition table, the TinyUSB
vendor + CDC classes and flash core dumps; delete a stale `sdkconfig` if it
predates those defaults. The top-level `CMakeLists.txt` relaxes one
warning-as-error for `espp/esp32-p4-eth` under ESP-IDF 6.1 (its DPI callback
struct gained an `on_vsync` field); drop that block once espp ships a fix.

Host-side tests (no hardware; kinematics vs. `Geometry.md` and the wire
protocol against the real `stream_frame` codec):

```console
cd test && ./run_host_tests.sh          # after one idf.py build (uses managed_components)
cd test && ./run_host_tests.sh ~/esp-cpp/espp
```

`visual_check_5bar.py` plots the IK for the documented poses (Python /
matplotlib).

## USB / hardware notes

- The USB frame streams use the ESP32-P4 **USB 2.0 OTG (HS)** port with its own
  PHY, so the USB-Serial-JTAG console keeps working alongside it.
- WebUSB / Web Serial need Chrome or Edge. Each browser console opens its own
  connection to the device.
- `webserial_example/` is an earlier text-protocol experiment superseded by
  the USB module + `web/stem_console.html`.
