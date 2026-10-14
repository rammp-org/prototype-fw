# PEAR prototype base MIB (ESP32-P4-ETH)

The PEAR prototype base is a holonomic platform like the holodeck (four hub-drive
omniwheels at ±30° / ±150°) whose four wheel modules each hang on an independent,
motor-driven 4-bar linkage at a corner. A Waveshare **ESP32-P4-ETH** kit stands
in for the MIB: it takes the joystick HMI's commands
([pace-hmi-fw](https://github.com/rammp-org/pace-hmi-fw)) over Ethernet with the
shared RTPS messages ([rammp-rtps](https://github.com/rammp-org/rammp-rtps), the
`external/rammp-rtps` submodule) and drives all eight motors over ONE CAN bus.

It is `holo_deck_mib` (RTPS link, 50 Hz drive loop, DHCP server/client) with the
RMD wheel motors replaced by CANopen DS402 hub drives, plus the linkages.

## Architecture

```
 pace-hmi-fw (Tab5)                               pear (ESP32-P4-ETH)
 ┌────────────────┐  rammp/joystick/xy_twist (~30 Hz)  ┌───────────────────┐
 │ stick, buttons │ ─────────────────────────────────▶ │ RtpsLink          │
 │ drive screen   │  rammp/joystick/drive_command      │   │          │    │   CAN 1 Mbit/s (CanBus)
 │ DEBUG ACTUATORS│ ─────────────────────────────────▶ │   ▼          ▼    │ ┌─▶ 4x AMPS L2DB4830 hub drive
 │ L1..L4 rows    │  rammp/joystick/seat_command       │ Drive-    Linkage-│─┤   (Ds402VelocityMotor, ids 2/1/3/4)
 │                │ ─────────────────────────────────▶ │ Controller Ctrl   │ └─▶ 4x Eyou linkage motor
 │ status labels  │ ◀───────────────────────────────── │ (50 Hz)   (250 ms)│     (EyouMotor, ids 107..110)
 │ diagnostics    │  rammp/mib/status (2 Hz)           └───────────────────┘
 │                │ ◀───────────────────────────────── 
 └────────────────┘  rammp/mcb/diagnostics (2 Hz)
 PC tools          ◀───────────────────────────────── rammp/pear_<motor>/motor_state x8 (10 Hz)
```

| file | what it does |
| --- | --- |
| `main/hw_config.hpp` | pins, node ids, base + linkage geometry, drive config, limits, profiles, link timing (every placeholder is marked `TODO(hardware)`) |
| `main/drive_controller.*` | command state + the 50 Hz wheel loop (`holo_deck_controller` on `Ds402VelocityMotor`) |
| `main/linkage_geometry.hpp` | pure pose ⇄ corner-height ⇄ crank-angle functions (host-tested) |
| `main/linkage_controller.*` | the four linkage motors: bring-up, zero, interlocked moves, status |
| `main/rtps_link.*` | the RTPS participant, the joystick subscriptions, the status / diagnostics / MotorState publishers, the joystick watchdog |
| `main/main.cpp` | NVS, Ethernet, CAN bring-up, wiring, the UART CLI |
| `test/linkage_geometry_test.cpp` | host test of the geometry |
| `spec/rammp-rtps-pear.patch` | the rammp-rtps extension this firmware is built against |
| `../components/ds402_velocity_motor` | the L2DB (simple DS402 velocity) driver |

### Motors

One CAN bus, TWAI at 1 Mbit/s, eight CANopen nodes (SDO only, no PDO mapping):

| corner | wheel drive (AMPS L2DB4830-CAFC, velocity) | linkage motor (Eyou, profile position) |
| --- | --- | --- |
| FR | 2 | 107 |
| FL | 1 | 108 |
| BR | 3 | 109 |
| BL | 4 | 110 |

Every per-corner table in the code is in `pear::Corner` order (FR, FL, BR, BL),
which is also the order of the PEAR rows in the RTPS tables (seat axes 4..7,
diagnostics 3..10, motor axes 4..11).

The wheel drives are plain velocity actuators: `enable_drive` puts them in speed
mode with the ramps from `hw_config.hpp` (the ramps live in the drive), then the
50 Hz loop streams `Target Velocity_DEC` set-points non-blocking. The L2DB's
**communication-loss protection** (object 0x4100:10, 600 ms by default) releases
the shaft when the set-point stream stops, so the controller keeps streaming
zeros every tick while e-stopped (STOPPED) exactly like `holo_deck_mib` does;
only DISABLED (CLI `disable`) powers the stage off deliberately, and the next
`enable` re-enables the drives from the control loop before the first set-point.

### How the HMI's messages are used

- **XYTwist** (x + = right, y + = forward, twist + = clockwise, deadzoned on the
  HMI) becomes the drive controller's joystick input (forward, left,
  counter-clockwise) scaled by the active profile's limits. It only drives while
  the controller is in DRIVE.
- **DriveCommand** ENABLE puts the drive controller in DRIVE (the stick must be
  centered first, so nothing lurches); DISABLE e-stops it (zero velocity
  streamed, drives hold). The profile it carries (LOW / NORMAL / HIGH) selects
  the speed and rotation limits in `hw_config.hpp`.
- **SeatCommand** `{axis, target}` (absolute, whole units) moves the linkages:

  | seat-axis row | what it does |
  | --- | --- |
  | M1 `FRONT_BACK_TILT` (deg) | the measured pose with this tilt replaced → `request_pose` (front up = positive) |
  | M2 `LATERAL_TILT` (deg) | same, left up = positive |
  | M3 `ELEVATION` (mm) | same; 0 = all linkages at the level angle (the table's min is 0, so the HMI cannot jog below level; the L rows can) |
  | M4 `TRANSLATION` | logged once and ignored (no translation axis) |
  | L1..L4 `PEAR_LINK_FR/FL/BR/BL` (deg) | one corner's crank angle (DEBUG ACTUATORS page, PIN 1234) |

  Targets are clamped to the row's range AND the geometry's angle range; a pose
  that one corner cannot reach is approximated (that corner sits at its limit).
  There is no per-request ack: a refusal shows as "the value did not move".
- **MibStatus** every 500 ms: `INITIALIZING` until the CAN bus is up, `ENABLED`
  while driving, `IDLE` otherwise, `ERROR` with the motor named when a wheel
  command fails or a wheel / linkage reports a DS402 fault; `activeProfile`;
  `currentSeatState` = the measured pose in table units (tilts deg, elevation
  mm, translation 0) + the four corner angles (deg); `speed` = commanded
  translation speed in m/s; `seq`. The HMI opens its drive screen on `ENABLED`,
  shows the seat rows once they have a value, and declares the link lost after
  2 s without this message.
- **Diagnostics** every 500 ms, one item per `RAMMP_DIAG_TABLE` row:

  | row | readings |
  | --- | --- |
  | T1..T3 | zeros (legacy test rows) |
  | W1..W4 wheel FR/FL/BR/BL | driver temperature (0.1 °C), torque current (0.01 A), actual speed (0.1 rpm) |
  | L1..L4 link FR/FL/BR/BL | temperature 0, current 0 (the Eyou driver exposes neither), crank angle (0.1 deg) |

- **MotorState** every 100 ms on `rammp::axis(PEAR_*).state`
  (`rammp/pear_wheel_fr/motor_state` … `rammp/pear_link_bl/motor_state`): the
  motor layer's board → MIB message, published by the MIB itself for its CAN
  motors. `state` ARMED (power stage on) / DISARMED / FAULT (`fault_code`
  DRV_FAULT, the drive's error bits in `drv_status`), `mode` VELOCITY (wheels) /
  POSITION (linkages) while ARMED, COAST otherwise; `position` rad, `velocity`
  rad/s (wheels only; the linkage velocity is not measured), `iq` = the wheel's
  torque current, `temps[0]` = temperature, `cmd_age_ms` since the last
  set-point / move, `uptime_ms`. **`MotorCommand` is not subscribed**: the CAN
  motors are not PACE RACER boards, the MIB commands them itself.

### Safety

- Boots e-stopped; only a DriveCommand ENABLE (or the CLI) starts driving.
- **Joystick watchdog**: no XYTwist for 500 ms while driving streams zero
  velocity; none for 2 s e-stops and reports `JOYSTICK LOST` in MibStatus until
  the HMI sends ENABLE again. It is timed from the HMI's ENABLE as well as from
  each XYTwist, so a stream that never starts is caught too. (A bench `enable`
  from the CLI is not watched until a joystick stream appears.)
- Every wheel command comes from the single control loop, so an e-stop can never
  be overtaken by an in-flight drive command; a stop first drains the CAN TX
  queue (`CanBus::flush_pending`) so no older set-point follows the zeros.
- Wheel speeds are scaled together so none exceeds `kMaxWheelRpm`.
- **Linkage interlock**: a linkage move is refused while the drive controller is
  in DRIVE (the spec lets the HMI command seat rows only while `IDLE`; the CLI
  follows the same rule), and while any linkage motor is not initialized / not
  enabled / faulted. Moves are queued and enacted by the linkage task, never from
  the RTPS callback; a queued move is dropped if driving started meanwhile.
  The interlock holds in the other direction too: the drive cannot be enabled
  while a batch is being sent or a corner has not reported target reached. A
  move that does not finish in time, or a batch a later corner refused, is
  halted (power stage off, confirmed, then on again); a halt the motor does
  not confirm keeps the drive locked out (`ERROR`, "halt unconfirmed") and is
  retried until it does.
- A wheel or linkage DS402 fault is shown in MibStatus (`ERROR`, motor named) and
  in that motor's MotorState (`FAULT`); `reset_fault` on the console clears it.

## Geometry placeholders to fill (hw_config.hpp, all `TODO(hardware)`)

- **CAN pins**: `kCanTxGpio` 17 / `kCanRxGpio` 16, the mapping every project in
  this repo ends up with on this kit (STEM_V3's constants are named the other way
  round but passed swapped). Symptom of a wrong pair: "CAN bus started" and every
  SDO timing out.
- **Wheel → node mapping** `kWheelNodeIdsByPlatformWheel` = the holodeck's
  `{2, 4, 3, 1}` (the PEAR ids were chosen to match the holodeck's). Verify:
  `enable`, `speed 0.2 0 0` rolls straight forward, `speed 0 0.2 0` straight
  right, `speed 0 0 3` turns counter-clockwise; otherwise fix the table or
  `kWheelDirections` first.
- **Base geometry** `kHalfLengthM` 0.406, `kHalfWidthM` 0.267, `kWheelDiameterM`
  0.21 (holodeck values; the linkage's fore/aft swing of the contact point is
  ignored - fixed footprint, a follow-up).
- **Wheel ramps** `kWheelAccelerationRps2` / `kWheelDecelerationRps2` 2 rev/s²,
  `kWheelMaxSpeedRpm` 0 (drive's own).
- **Linkage 4-bar** `kLinkageCrankLengthM` 0.15, `kLinkageLevelAngleDeg` 0,
  `kLinkageAngleMinDeg` -30, `kLinkageAngleMaxDeg` 60 (the seat-axis rows L1..L4
  allow -30..60 deg; keep them covering the range), the motion profile
  `kLinkageProfile` 60 deg/s, 20 deg/s². The model assumes a parallelogram
  4-bar: height = crank · sin(angle − level); see `linkage_geometry.hpp`.
- **Zero**: with every linkage at its level position run `zero all` then
  `save_zero`; the zero is kept in NVS per node id and loaded at boot (a missing
  zero is a boot warning and the motor works on its absolute encoder).

## The rammp-rtps spec extension

This firmware is coded against an extension of the rammp-rtps tables (new seat
axes L1..L4, diagnostics rows W1..W4 / L1..L4, motor axes PEAR_*) that is shipped
here as `spec/rammp-rtps-pear.patch` until the spec PR lands and the submodule
gitlink is bumped. For a local build apply it to the submodule working copy:

```sh
git submodule update --init                      # external/rammp-rtps
git -C external/rammp-rtps apply ../../pear/spec/rammp-rtps-pear.patch
```

The `seatState` fields it appends shift every later `MibStatus` field, so the
**HMI and the MIB must be flashed from the same spec revision** (pace-hmi-fw
needs the matching `seat_field()` cases in `main/hmi_rtps_spec.hpp`). The
prototype-fw PR cannot merge before the spec PR.

## Wiring

| signal | GPIO |
| --- | --- |
| TWAI TX → transceiver TXD | 17 |
| TWAI RX ← transceiver RXD | 16 |

3.3 V CAN transceiver on a 120 Ω-terminated bus at 1 Mbit/s shared by all eight
nodes; the transceiver's standby / enable pin must be in its active state.

## Network

The HMI is a DHCP client. `idf.py menuconfig` → **PEAR MIB Configuration**:

- **DHCP server** (default): this board serves 192.168.4.1/24, for a direct cable
  to the HMI (the chair, no router).
- **DHCP client**: for a bench LAN with a router, so the PC tools from
  pace-hmi-fw (`scripts/rtps_mcb_gui.py`, ...) share the subnet with both boards
  and can read the eight MotorState topics.

RTPS discovery is multicast on the local segment; nothing needs to know the
other side's address. The RTPS link is started from a task once the interface
has an address; the CLI is available before that.

## Build & flash

```sh
git submodule update --init && git -C external/rammp-rtps apply ../../pear/spec/rammp-rtps-pear.patch
cd pear
idf.py set-target esp32p4
idf.py -p <PORT> flash monitor
```

Host test of the geometry (no ESP-IDF):

```sh
cd pear/test && c++ -std=c++20 -I../main linkage_geometry_test.cpp && ./a.out
```

## CLI (UART console)

| command | effect |
| --- | --- |
| `enable` | DRIVE (clears e-stop; setpoint zeroed; linkage moves refused while driving) |
| `stop` | e-stop: zero velocity streamed to every wheel |
| `disable` | wheel drives powered off (DS402 disable); `enable` re-enables them |
| `speed <vx> <vy> <w>` | local velocity setpoint: m/s forward, m/s **right**, RPM counter-clockwise |
| `profile <low\|normal\|high>` | the limits of that drive profile |
| `link <fr\|fl\|br\|bl\|all> <deg>` | move a corner linkage to a crank angle |
| `pose <elev_mm> <fb_deg> <lat_deg>` | move all four to a base pose (front up +, left up +) |
| `zero <fr\|fl\|br\|bl\|all>` | software zero = current position (`save_zero` to persist) |
| `save_zero` / `load_zero` | the linkage zeros in NVS |
| `reset_fault <corner\|all>` | clear the DS402 fault of a corner's wheel + linkage |
| `status` | link, drive, wheel and linkage state |
| `diag` | the diagnostics rows as the HMI shows them |

## Bench test without the chair

1. Flash, connect the HMI (or a PC running pace-hmi-fw's peer tools) on the same
   segment; `status` shows `hmi: alive` once XYTwist streams.
2. On the HMI, push the stick up and hold: DriveCommand ENABLE; MibStatus goes
   `ENABLED`, the drive screen opens, the stick drives the wheels.
3. DEBUG ACTUATORS (PIN 1234) with driving disabled: the L1..L4 rows jog the
   linkages by 1 deg; M1..M3 tilt / raise the whole base.
4. DIAGNOSTICS shows W1..W4 / L1..L4; a PC subscriber sees the eight
   `rammp/pear_*/motor_state` topics at 10 Hz.
5. Unplug the Ethernet cable while driving: within 500 ms the base holds zero,
   after 2 s it e-stops and MibStatus (once the link is back) shows
   `JOYSTICK LOST`.

## Follow-ups

- A shared CANopen base for `EyouMotor` and `Ds402VelocityMotor` (they duplicate
  the SDO helpers).
- Footprint correction for the linkage angle (the wheel contact point moves
  along the crank's arc).
- Linkage temperature / current in the L rows if the Eyou drive exposes them.
