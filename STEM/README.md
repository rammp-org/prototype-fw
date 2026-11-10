# STEM

ESP-IDF firmware for the STEM prototype: a planar 5-bar seat linkage (see
`Geometry.md`) driven by three reversed pairs of Reflex RMD-X6-S2 actuators
over CAN, on the Waveshare ESP32-P4-ETH. Based on the original five-bar
application.

- **Board**: Waveshare ESP32-P4-ETH (ESP32-P4, 16 MB flash), `espp::Esp32P4Eth`
- **Motors**: Reflex RMD-X6-S2, 1:36 gear ratio, CAN on GPIO 16 (TX) / 17 (RX)
- **Pairs**: left = motors 2/1, right = 4/3, seat = 6/5 (`PairedActuator`, the
  secondary motor runs mirrored)

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
  `zero_align <pair|all>`, `ik <x> <y>`, `ik_ref <x_rel> <y_rel>`.
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
  firmware's tracked pose, lets you drag the end effector to move it, sends
  direct pose / pair commands and streams motor telemetry. See `web/README.md`.

Note on "current pose": the RMD status frame only carries the motor-side
(pre-gearbox) encoder, so there is no measured output-shaft angle. The pose the
firmware reports is the FK of the pair positions it has commanded since the
last zero; motor velocity is the "still moving" indicator.

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
- `serial_protocol_example/` and `webserial_example/` are earlier text-protocol
  experiments superseded by the USB module + `web/stem_console.html`.
