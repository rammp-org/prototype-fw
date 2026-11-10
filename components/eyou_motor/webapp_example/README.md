# Eyou Motor Web Console Example

Turns an ESP32-P4 with a CAN transceiver into a **WebUSB / Web Serial
test-stand console** for ONE Eyou harmonic actuator: the
[Eyou motor console web app](../web/eyou_console.html) (`web/eyou_console.html`,
a single offline HTML file) connects over native USB and can

- **watch** the drive live: angle, speed, torque and current as big readouts
  and two strip charts (angle + speed, torque + current, with the torque limit
  drawn as guide lines), the DS402 state, the error code, target reached,
- **limit** the torque: the max-torque object (0x6072) the drive clamps its
  torque demand to, in per-thousand of the rated torque (shown in Nm once the
  rated torque is read from the drive). Every move runs under it, so a
  commanded angle the mechanics cannot reach stalls at the limit instead of
  pushing through,
- **move**: absolute moves to a target angle (clamped to position limits),
  relative jogs (buttons, or the arrow keys with the step field focused), the
  profile velocity / acceleration / deceleration and the position limits,
- **control the drive**: enable / disable / reset fault / halt,
- **set the zero**: the current position becomes 0 deg, or becomes a given
  angle, then save it on the device (NVS, per node id, loaded at boot) or load
  it back.

The example runs the `EyouMotor` CANopen / DS402 driver **on the device** and
exposes a small high-level protocol (`main/eyou_protocol.hpp`, dispatcher
**module id 0x20**, protocol id `eyou.motor`) through `EyouService`
(`main/eyou_service.hpp`), so the web app needs no knowledge of CAN. Both the
vendor (WebUSB) and CDC (Web Serial) interfaces carry the same protocol; the
system console / logs go to **UART0** (set in `sdkconfig.defaults`), which on
the ESP32-P4-ETH is the Type-C port (a CH343 USB-to-UART bridge: flashing and
the monitor, not a USB connection to the chip).

**Which connector is the console's USB?** On the ESP32-P4-ETH the ESP32-P4's
USB 2.0 high-speed OTG signals are on the **4-pin MX1.25 "USB" connector**, not
on the Type-C. Wire D+ / D- / GND of that connector to a USB cable to the PC
(VBUS is not needed while the board is powered from the Type-C) and keep the
default *USB port* = high-speed. Dev kits that put the USB-Serial-JTAG pad
pair (GPIO 24 / 25) on a USB-C (Espressif's, Waveshare's WIFI6 kit) use the
*full-speed port on the USB-C pads* choice instead.

Like every espp USB example it also serves the standard **system** (module 7:
device info, reboot, reboot into the bootloader), **monitor** (8: tasks,
heap, CPU), **OTA** (0: firmware update over USB, with rollback) and **core
dump** (4: read / clear the panic core dump) services on both transports, so
the hosted espp [Device Hub](https://esp-cpp.github.io/espp/apps/dispatcher_hub.html)
and its system / OTA / core-dump consoles work against it, and the hub lists
the Eyou module (it links to `eyou_console.html`, which must be served next to
the hub to be opened from it; opening the local file directly works too).

## Wiring & configuration

| signal | default | menuconfig (`Eyou Console Configuration`) |
| --- | --- | --- |
| CAN TX (TWAI TX -> transceiver TXD) | GPIO 17 | `EXAMPLE_CAN_TX_GPIO` |
| CAN RX (TWAI RX <- transceiver RXD) | GPIO 16 | `EXAMPLE_CAN_RX_GPIO` |
| CAN bitrate | 1 Mbit/s | `EXAMPLE_CAN_BITRATE` |
| Eyou CANopen node id | 1 | `EXAMPLE_NODE_ID` |
| poll period (live readings) | 50 ms | `EXAMPLE_POLL_PERIOD_MS` |
| maximum torque limit the console may set | 500 ‰ of rated | `EXAMPLE_MAX_TORQUE_PERMILLE` |
| torque limit written at boot | = the maximum | `EXAMPLE_BOOT_TORQUE_PERMILLE` |
| profile velocity / accel / decel at boot | 30 deg/s, 20 / 20 deg/s² | `EXAMPLE_PROFILE_*` |
| load the saved zero at boot | yes | `EXAMPLE_LOAD_ZERO_AT_BOOT` |
| USB port | USB 2.0 high-speed (the P4-ETH's 4-pin "USB" connector) | `EXAMPLE_USB_PORT` |

The defaults are the Waveshare ESP32-P4-ETH as wired for the other projects
in this repository (same pins as `pear/`). The bus needs a 120 Ω termination
at each end, and the drive must be set to the configured bitrate and node id.

**Safety.** The drive is NOT enabled at boot: the boot torque limit is
written first (before anything can move), then the profile and the saved
zero, and the drive stays disabled until *Enable* is pressed in the console.
The console changes the limit at run time (the slider applies on release) but
never above the menuconfig **maximum**: the device refuses a higher request,
reports the maximum in every status, and the console sizes its slider to it.
*Halt* stops a move with the profile deceleration while staying enabled.
A drive that is powered after the device (or that drops off the bus) is
commissioned again the moment it answers.

## Build & flash

```bash
idf.py set-target esp32p4
idf.py build flash monitor   # the console / logs are on UART0
```

The espp components (`espp/usb_device`, `espp/dispatcher`, ... ≥ 1.4.2) and
`espressif/esp_tinyusb` come from the component registry
(`main/idf_component.yml`); `eyou_motor` and `can_bus` are the sibling
components of this repository. The 16 MB flash partition table is the dual-OTA
+ core dump layout of `pear/` (`partitions.csv`).

Then, with the 4-pin USB connector cabled to the PC, open
`web/eyou_console.html` in a Chromium browser (file:// works; WebUSB needs a
secure origin, which file:// is) and *Connect (WebUSB)*, or *Connect (Web
Serial)* on the CDC port. If nothing enumerates, the UART0 log at boot says
which port TinyUSB took (`USB-OTG 1.1 on full-speed PHY ...` means the
full-speed choice is active: `idf.py menuconfig` → *Eyou Console
Configuration* → *USB port*; an existing `sdkconfig` keeps the old choice).

## Protocol (`eyou.motor` v1, module 0x20)

All fields little-endian; floats are IEEE-754 binary32. Requests (0x1X) are
answered with OK `[request u8]`, ERROR `[request u8][code u32][utf8 message]`
or, for GetStatus, a STATUS.

| type | request | payload |
| --- | --- | --- |
| 0x10 | GetStatus | - |
| 0x11 | SetStream | `[enable u8][period_ms u16]` STATUS on this transport at the period (0 = device default) |
| 0x12 | SetTorqueLimit | `[permille u16]` max torque 0x6072; refused above the device's `torque_limit_max` |
| 0x13 | MoveTo | `[angle_deg f32][immediate u8]` |
| 0x14 | MoveBy | `[delta_deg f32][immediate u8]` |
| 0x15 | Enable | - |
| 0x16 | Disable | - |
| 0x17 | ResetFault | - |
| 0x18 | Halt | - |
| 0x19 | SetZero | `[mode u8][angle_deg f32]` mode 0: current position = 0; 1: current position = angle |
| 0x1A | SaveZero | - |
| 0x1B | LoadZero | - |
| 0x1C | SetProfile | `[velocity f32][acceleration f32][deceleration f32]` deg/s, deg/s² |
| 0x1D | SetPositionLimits | `[min_deg f32][max_deg f32]` |

STATUS (0x90, 72 B): `[flags u8][node u8][statusword u16][error_code u16][mode i8][res u8]
[angle f32][velocity f32][torque ‰ i16][current ‰ i16][torque_limit ‰ u16][torque_limit_max ‰ u16]
[target f32][rated_torque_mNm u32][rated_current_mA u32][zero_offset_pulses i32]
[pos_min f32][pos_max f32][profile_vel f32][profile_acc f32][profile_dec f32]
[timestamp_ms u32][sequence u32][stream_period_ms u16][res u16]`; flags: bit 0
enabled, 1 faulted, 2 zero set, 3 stream on, 4 target reached, 5 CAN
communication ok. Torque / current are per-thousand of the rated values
(`rated_*` give the scale); `target` is NaN until a move was commanded.

## Files

| file | what |
| --- | --- |
| `main/eyou_protocol.hpp` | the wire protocol (standard-library only) |
| `main/eyou_service.hpp` | the dispatcher module: commands -> `EyouMotor`, STATUS streaming |
| `main/eyou_webapp_example.cpp` | CAN + motor + USB + the poll loop + the standard services |
| `web/eyou_console.html` | the browser console (shared espp connection / discovery / hub code + the motor UI) |
