# holo_deck

ESP-IDF firmware for the Holo Deck prototype: a holonomic platform driven by
four Reflex RMD-X6-S2 motor actuators (CAN ids 1-4), controlled from an
**M5Stack Tab5** (ESP32-P4) with a touch GUI and a 3-axis analog joystick.

## Hardware & wiring

All pins are defined (with the availability reasoning) in
[`main/hw_config.hpp`](main/hw_config.hpp); the table below is a summary.

### CAN bus (M5-Bus header)

An external CAN transceiver (3.3 V logic, e.g. SN65HVD230) connects the Tab5
to the motor bus (1 Mbit/s):

| Signal | Tab5 pin | GPIO |
|---|---|---|
| TWAI TX -> transceiver TXD | M5-Bus "PC_TX" | GPIO6 |
| TWAI RX <- transceiver RXD | M5-Bus "PC_RX" | GPIO7 |
| Transceiver supply | M5-Bus 3V3 (or 5V, per module) + GND | - |

### 3-axis joystick + button (M5-Bus header)

The joystick potentiometers **must be supplied from 3.3 V** (not 5 V) so the
wiper voltages stay inside the ADC range.

| Signal | Tab5 pin | GPIO | ADC |
|---|---|---|---|
| X (left/right) wiper | M5-Bus pin 2 "G16" | GPIO16 | ADC1_CH0 |
| Y (forward/back) wiper | M5-Bus pin 4 "G17" | GPIO17 | ADC1_CH1 |
| Twist (rotation) wiper | M5-Bus pin 10 "G52" | GPIO52 | ADC2_CH3 |
| Pushbutton (to GND, active-low) | M5-Bus pin 22 "G48" | GPIO48 | - |
| Supply | M5-Bus 3V3 + GND | - | - |

Why these pins: ESP32-P4 ADC1 only covers GPIO16-23, and on the Tab5 GPIO22/23
are used by the display and GPIO20/21 are wired to the on-board RS-485
transceiver - GPIO16-19 are the only ADC1-capable header pins. The twist axis
sits on ADC2 (GPIO49-54), of which GPIO52 is broken out on the M5-Bus.
Per-axis calibration (center/min/max mV, inversion) and the 5% deadzones are
constants in `hw_config.hpp`. Because all three axes are **spring-return**, the
firmware **auto-captures each axis's center at boot** (`kJoystickAutoCenter`)
so the deadzones apply around the true resting point regardless of pot
tolerance - keep the stick centered at power-on. An axis whose boot reading is
implausibly far from the nominal center falls back to the fixed center with a
warning.

The joystick button toggles ENABLE / E-STOP (debounced, same shared state as
the GUI button and the CLI commands).

## Control model

A `HoloDeckController` owns the command state and a 50 Hz control loop:
setpoint -> `HoloDeckPlatform::calculate_wheel_speeds(vx, vy, w)` -> scale all
four wheels together so none exceeds the per-wheel RPM limit (`kMaxWheelRpm`,
45 RPM output) -> CAN velocity commands. **Every motor command is issued from
this one loop** - the mode setters never touch the bus - so a STOP can never be
overtaken by an in-flight drive command from another thread.

Frame convention: `vx` forward (m/s), `vy` left (m/s), `w` counter-clockwise.
Rotation is stored in chassis RPM but shown in deg/s in the GUI. The rotation
speed is bounded by the wheel limit: a pure spin at w RPM drives the wheels at
~4.6*w RPM, so the 45 RPM cap allows ~9.7 RPM (~58 deg/s) - raise `kMaxWheelRpm`
for a faster base (the RMD-X6-S2 has ample headroom).

**Modes**

* **DRIVE** - the active source drives the platform.
* **STOPPED** (e-stop) - the loop actively commands zero velocity every tick
  (motors decelerate and hold at 0), overriding all sources. Triggered by the
  GUI STOP button, the joystick button, or CLI `estop`. Always wins.
* **DISABLED** - the motors are halted at the **control level**
  (`MotorActuator::stop()`): the velocity loop stops commanding them entirely,
  rather than holding them at zero. Triggered by the GUI DISABLE button or CLI
  `disable`; `enable` returns to DRIVE. The firmware boots STOPPED.

**Sources & arbitration** (DRIVE mode)

* **GUI/CLI setpoint** - the touch controls and the `velocity` CLI command
  write the same setpoint.
* **Physical joystick** - any deflection beyond its deadzone immediately takes
  over. Once centered for 500 ms, control falls back to the GUI setpoint, which
  is **zeroed on the handover** so the platform never jumps.
* On (re-)enable the joystick must **re-center first** before it can take over,
  so a stick held deflected at enable time cannot make the platform lurch.

Runtime-adjustable limits: max translation speed (m/s) and max rotation rate
(deg/s), via GUI sliders or the `limits` CLI command; both clamp every source.

## GUI (touch)

The layout is built with flex containers sized from the runtime display
resolution, so it fills the screen without overlap whether the panel is
presented portrait or landscape.

* **Top bar**: active source / mode (GUI / JOYSTICK / STOPPED / MOTORS
  DISABLED, color-coded), a DISABLE button, and a large ENABLE / STOP button.
* **Left**: the commanded velocity as a vector on a circle (full radius = max
  speed), numeric vx / vy / |v| / heading / rotation (deg/s) readouts, the
  current limits, and four motor tiles (id, commanded + measured RPM,
  temperature, OK / STALE / NO DATA with colored borders).
* **Right**: a circular drag-pad (virtual joystick, spring-return) for
  translation, a rotation slider (left = CCW), and sliders for the max-speed /
  max-rotation limits.

The drag-pad and rotation slider only *act* while the GUI is the active
source; when the physical joystick takes over they keep moving as read-only
indicators of the live command.

## CLI

Over the USB serial console (`idf.py monitor`):

| Command | Description |
|---|---|
| `velocity <vx> <vy> <w>` | Set the GUI/CLI setpoint (m/s, m/s, RPM), clamped to the limits |
| `platform_speed <vx> <vy> <w>` | Alias of `velocity` (kept for compatibility) |
| `enable` | Enter DRIVE mode (clears STOP/DISABLE; setpoint zeroed) |
| `estop` | E-stop: command zero velocity to all motors (they hold at 0) |
| `disable` | Halt the motors at the control level (not held at zero); `enable` to resume |
| `limits <max_mps> <max_rpm>` | Set the translation / rotation limits |
| `status` | Print the full controller + motor state |
| `set_speed <id> <rpm>` | Direct single-motor command (debug; e-stop first, the control loop overrides it otherwise) |

## Building & flashing

Requires ESP-IDF (>= 5.3) with the component manager enabled; the espp
components (`m5stack-tab5`, `cli`, `joystick`, `adc`, `math`, `timer`) come
from the IDF component registry, and `motor_actuator` / `holo_deck_platform`
from this repo's `components/` directory.

```console
idf.py set-target esp32p4
idf.py build
idf.py -p PORT flash monitor
```
