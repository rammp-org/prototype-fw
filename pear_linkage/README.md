# PEAR linkage console

ESP-IDF firmware + browser console for **configuring and testing ONE 5-bar
linkage** driven by **two Eyou harmonic actuators** (CANopen / DS402, one per
crank) over CAN, on the Waveshare ESP32-P4-ETH: the linkages that move PEAR's
middle drive wheels. Unlike STEM's seat linkage (reversed motor pairs on each
axis) each crank here has a single motor.

The firmware runs the `EyouMotor` driver and the 5-bar kinematics on the
device and exposes a small protocol (`main/linkage_protocol.hpp`, dispatcher
**module id 0x21**, protocol id `pear.linkage`) over native USB. The console
(`web/linkage_console.html`, a single offline HTML file) connects over WebUSB
or Web Serial and can

- **configure the geometry**: M2's position, the two crank lengths, the bar
  (J2→J3, the link the end effector rides on), the coupler (J1→J3), an end
  effector extension / offset past J3 (0 = the wheel axle is J3), the crank
  assembly (any / crossed / open) and which side of J1→J2 the joint J3 folds
  to; and the home crank angles,
- **configure each motor**: node id (0 = no motor on that crank), direction
  (+1: positive motor angle = counter-clockwise crank), the motor angle
  **limits**, the crank angle when the motor reads 0, and its **torque limit**
  (per-thousand of rated, capped by a menuconfig ceiling); plus the profile
  velocity / acceleration / deceleration,
- **move the end effector**: drag it in the drawing (inverse kinematics on
  the device; the page runs the same solver to preview the pose, shade the
  reachable area within the limits and turn the ghost red when a target is
  not allowed), type a target, jog with buttons / arrow keys, Home,
- **drive the cranks directly**, enable / disable / reset fault / halt each
  motor or both,
- **set the zeros**: "zero here", or tell the device where the crank
  physically is ("crank is at N°"), then save / load them on the device,
- **watch** the live pose (forward kinematics of the measured crank angles),
  motor angles, speeds, torque vs limit, current, DS402 state and errors.

Every setting is stored on the device (NVS) and survives a reboot; menuconfig
only provides the defaults. The standard espp **system** (7), **monitor** (8),
**OTA** (0) and **core dump** (4) services run on both transports too, so the
hosted [Device Hub](https://esp-cpp.github.io/espp/apps/dispatcher_hub.html)
and its consoles work against this device.

## Hardware

| signal | default | menuconfig (`PEAR Linkage Console Configuration`) |
| --- | --- | --- |
| CAN TX (TWAI TX → transceiver TXD) | GPIO 17 | `LINKAGE_CAN_TX_GPIO` |
| CAN RX (TWAI RX ← transceiver RXD) | GPIO 16 | `LINKAGE_CAN_RX_GPIO` |
| CAN bitrate | 1 Mbit/s | `LINKAGE_CAN_BITRATE` |
| default node ids (crank 1 / crank 2) | 1 / 2 | `LINKAGE_MOTOR1_NODE_ID`, `..2..` |
| poll period | 100 ms | `LINKAGE_POLL_PERIOD_MS` |
| torque limit ceiling / default | 500 / 300 ‰ | `LINKAGE_MAX_TORQUE_PERMILLE`, `LINKAGE_DEFAULT_TORQUE_PERMILLE` |
| USB port | high-speed (the P4-ETH's 4-pin "USB" connector) | `LINKAGE_USB_PORT` |

On the ESP32-P4-ETH the **Type-C is a CH343 USB-to-UART bridge** (flashing +
this firmware's console / logs), not a USB connection to the chip. The chip's
USB 2.0 high-speed OTG signals are on the **4-pin MX1.25 "USB" connector**:
wire D+ / D- / GND to a USB cable to the PC (VBUS is not needed while the
board is powered from the Type-C). The bus needs 120 Ω termination at each
end; the drives must be set to the bitrate and node ids configured here.

**Safety.** The drives are NOT enabled at boot. Each motor's torque limit is
written before anything can move (and again whenever a drive appears on the
bus). Targets whose motor angles fall outside the configured limits are
**refused, not clamped**, on the device as in the page. HALT stops the moves
in progress with the profile deceleration and keeps the drives enabled;
Disable frees the linkage.

## Build, flash, test

```bash
idf.py set-target esp32p4
idf.py build flash monitor   # console / logs on the Type-C (UART0)
cd test && ./run_host_tests.sh   # kinematics + protocol, no hardware
```

The espp components (≥ 1.4.2) and `espressif/esp_tinyusb` come from the
component registry (`main/idf_component.yml`); `eyou_motor` and `can_bus` are
this repository's. The 16 MB flash layout (`partitions.csv`) is the dual-OTA +
core dump layout of `pear/`.

Then, with the 4-pin USB connector cabled, open `web/linkage_console.html` in
a Chromium browser (`file://` works; WebUSB needs a secure origin, which
`file://` is) and *Connect (WebUSB)* — or *Connect (Web Serial)* on the CDC
port. The page also works offline: edit the geometry and drag in the drawing
to explore the kinematics.

## Frame and kinematics

The linkage frame has **M1 at the origin and +x toward M2**; crank angles are
measured counter-clockwise from +x (so a crank hanging straight down is at
−90°). The loop is M1 → J2 → J3 → J1 → M2 with the end effector M3 on the bar
(J2→J3), `extension` past J3 along it and `offset` 90° counter-clockwise of it
(the STEM seat linkage is bar 170, extension 100; a wheel on the J3 axle is
0 / 0). `main/five_bar.{hpp,cpp}` solves the forward kinematics (crank angles
→ pose) and the inverse (M3 → crank angles); among the assemblies that reach
a point it returns the one closest to a reference — the measured pose, or the
home angles while none is measured — optionally restricted to crossed or open
cranks, and the fold of J3 (which side of J1→J2) is a fixed property set in
the settings. The host test checks it against STEM's documented edge poses and
round-trips a classic 5-bar over its workspace; the page's JavaScript is a
line-by-line port.

A motor angle `m` and its crank angle `θ` relate through the motor settings:
`θ = θ_at_zero + direction · m` (and back, wrapped to (−180°, 180°]). Each
motor's **limits** are on `m`. To calibrate: put the linkage in a known pose,
and for each motor enter the crank's angle in *crank is at* and press *Set*
(the device sets that motor's software zero so the configured `θ_at_zero` /
direction make it read correctly), then *Save zero* on both. Or press *Zero
here* and set *crank angle at zero* in the settings to the crank's angle at
that pose.

## Protocol (`pear.linkage` v1, module 0x21)

Little-endian; floats are binary32; angles in degrees, lengths in mm. Every
request is answered with OK `[request u8]` or ERROR `[request u8][code u32]
[utf8 message]`, except the ones that reply with a payload.

| type | request | payload / reply |
| --- | --- | --- |
| 0x10 | GetInfo | → INFO `[proto u8][motors u8][max_torque u16][poll_ms u16][str project][str version][str build]` v2: `[default node id M1 u8][M2 u8][node ids from menuconfig u8]` |
| 0x11 | GetStatus | → STATUS |
| 0x12 | SetStream | `[enable u8][period_ms u16]` |
| 0x13 | GetSettings | → SETTINGS |
| 0x14 | SetSettings | `[SETTINGS]` validated, applied, saved; → OK then SETTINGS |
| 0x15 | ResetSettings | firmware defaults; → OK then SETTINGS |
| 0x16 | MoveTo | `[x f32][y f32]` |
| 0x17 | MoveBy | `[dx f32][dy f32]` relative to the pending target, else the measured pose |
| 0x18 | Home | both cranks to the home angles |
| 0x19 | SetMotor | `[motor u8][degrees f32]` one crank's motor angle |
| 0x1B..0x1E | Enable / Disable / ResetFault / Halt | `[motor u8]` 0, 1 or 0xFF (both) |
| 0x1F | ZeroMotor | `[motor u8][mode u8][value f32]` mode 0: reads 0 here; 1: reads `value` here; 2: the crank is at `value`° here |
| 0x20 / 0x21 | SaveZero / LoadZero | `[motor u8]` |
| 0x22 | SolveIk | `[x f32][y f32]` → IK_RESULT (no motion) |
| 0x23 | RecoverCan | start the CAN bus-off recovery (it is automatic too) |
| 0x24 | SetDriveNodeId | `[motor u8][new_id u8]` re-address the drive on that crank (both Node-ID objects, saved, communication reset, ~5 s); on success the settings' node id follows it and is saved; "saved, needs a power cycle" and failure come back as ERROR text |

**Node ids: who wins.** The console's settings are saved in NVS and win over
the menuconfig defaults, so changing `LINKAGE_MOTOR1_NODE_ID` /
`LINKAGE_MOTOR2_NODE_ID` and reflashing does NOT change the ids a device with
saved settings uses: the boot log says so, the console shows the firmware
default beside a differing node id, and *Reset settings* adopts the defaults.
Set `LINKAGE_NODE_IDS_FROM_MENUCONFIG=y` to make the menuconfig ids win at every
boot instead. Two different things in the console: the **node id** field is the
id the drive answers on (Apply makes the device talk to it; the drive is not
touched), **re-address drive to** programs a new id INTO the drive (both Node-ID
objects, saved, communication reset) and the node id follows it on success.

Protocol v2 (2026-11): STATUS ends with the CAN controller block
`[state u8][TEC u16][REC u16][bus errors u32][bus-off count u32][stuff+form+bit u32][ack u32]`
(state 1 error active, 2 warning, 3 passive, 4 bus off) and each motor's record
carries the drive's rated torque (0.01 Nm) in its last word, so the console sets
and shows torque in Nm. The halt / disable requests flush a set-point stuck in the
CAN TX queue first; enabling a drive in DS402 Fault resets the fault on the way
(before, the enable timed out and only a power cycle recovered the drive).

SETTINGS (95 B): `[version u8 = 1]` geometry `[m2_x][m2_y][crank1][crank2]
[bar][coupler][ee_extension][ee_offset]` (f32) `[assembly u8][coupler_side u8]`
`[home_theta1 f32][home_theta2 f32]` then per motor `[node u8][direction i8]
[res u16][min f32][max f32][theta_at_zero f32][torque_limit u16][res u16]`,
then `[profile_velocity f32][profile_accel f32][profile_decel f32]`.

STATUS (87 B): `[flags u8][sequence u32][timestamp_ms u32][stream_period u16]
[x][y][theta1][theta2][bar_angle][target_x][target_y]` (f32, NaN when not
valid) then per motor `[node u8][flags u8][statusword u16][error u16][res u16]
[angle f32][velocity f32][torque i16][current i16][torque_limit u16][res u16]`.
Flags: bit 0 stream, 1 CAN ok, 2 pose valid, 3 target valid, 4 comm ok, 5
enabled, 6 faulted, 7 moving; motor flags: bit 0 comm ok, 1 enabled, 2
faulted, 3 target reached, 4 zero set, 5 present.

## Files

| file | what |
| --- | --- |
| `main/five_bar.{hpp,cpp}` | the generalized 5-bar kinematics (standard-library only) |
| `main/linkage_protocol.hpp` | the wire protocol and the Settings record |
| `main/linkage_controller.{hpp,cpp}` | one linkage: two `EyouMotor`s + settings → moves, polling, zeros |
| `main/linkage_service.hpp` | the dispatcher module: requests → controller, STATUS streaming |
| `main/main.cpp` | CAN, motors, NVS settings, USB + the standard services, the poll task |
| `web/linkage_console.html` | the browser console |
| `test/` | host tests (`run_host_tests.sh`) |
