# STEM web console

`stem_console.html` is the browser front end for the STEM firmware's USB
interface: it draws the 5-bar linkage from the firmware's tracked pose, lets you
drag the end effector (M3) to move it, sends direct pose / pair commands and
shows motor telemetry. It is a single self-contained page (no build step, no
CDN) in the same style as the [espp web apps](https://esp-cpp.github.io/espp/apps/),
and speaks the STEM module (7) of the firmware's espp dispatcher — see
`../main/stem_protocol.hpp` for the wire protocol.

Firmware update and crash-dump inspection are **not** part of this page: the
firmware runs the standard espp OTA (module 0) and core-dump (module 4)
protocols on the same USB streams, so the stock espp consoles handle those.

## Run

Chrome / Edge only (WebUSB and Web Serial). Serve the directory over
`localhost` (WebUSB requires a secure context; `file://` also works for a
single page):

```console
cd STEM/web
./fetch_espp_consoles.sh            # once: hub + OTA + core-dump consoles from esp-cpp.github.io
# or: ./fetch_espp_consoles.sh ~/esp-cpp/espp   (copy from a local espp checkout)
python3 -m http.server 8000
```

Then open <http://localhost:8000/stem_console.html> — or
<http://localhost:8000/dispatcher_hub.html>, which discovers the device's
modules and links each console. The fetched espp consoles are git-ignored.

Two transports carry the same protocol, pick either:

- **Connect (WebUSB)** — the vendor interface (VID `0x1209`, PID `0x0d3a`,
  product "STEM Linkage"). Chrome shows the device's landing page notification
  on plug-in.
- **Connect (Web Serial)** — the CDC-ACM port of the same composite device.
  This is a *frame* stream, not the console: the CLI stays on the board's
  USB-Serial-JTAG / UART0 console.

Each console opens its own connection, so disconnect one before connecting
another to the same transport (or use WebUSB for one and Web Serial for the
other).

## What the page shows

- **Linkage view** (analysis frame, mm: origin at M1, +x toward M2, +y up):
  the solid linkage is the firmware's *tracked* pose — the forward kinematics
  of the pair positions it has commanded since the last zero. The RMD status
  frame only carries the motor-side (pre-gearbox) encoder, so there is no
  measured output-shaft angle; the motor velocities are the "still moving"
  indicator. The dashed box is the documented design travel with the five edge
  poses marked.
- **Target** (translucent): drag anywhere in the drawing. The page runs the
  same IK and pair-limit checks as the firmware and turns the ghost red when a
  point is unreachable or a pair would leave its limits; such targets are not
  sent (and the firmware rejects them anyway instead of clamping). "Move when
  the drag is released" sends one `MOVE_TO` on release; "live" sends while
  dragging at 4 Hz.
- **Pose command**: `x_rel` / `y_rel` are millimetres relative to the IK
  reference pose (the firmware / CLI convention, `home` = 0, 0); the readouts
  also give the plate-frame coordinates from `Geometry.md`.
- **Actuator pairs**: tracked position per pair, direct `Set`, per-pair
  `Zero` / `Zero-align`, `Zero all`, `Zero-align all` and `Release brakes`.
  - `Zero` marks the current position as 0 without moving. `Zero-align` /
    `Zero-align all` (confirmation required) first drive each motor to its
    encoder zero — **the motors move** — then zero the pair: the once-per-power-up
    calibration step, to be done after reaching the calibration pose. Same as the
    CLI's `zero_align`.
  - **Seat tilt**: an offset in seat-pair degrees that the firmware adds to the
    IK's level seat angle on *every* move, so the seat keeps its tilt as the
    linkage moves (a plain `Set` on the seat pair would be undone by the next
    move). `Apply tilt` re-commands the seat immediately; the device's current
    value is shown next to it and the drag preview's limit check includes it.
    Same as the CLI's `tilt <deg>`.
- **Motor telemetry**: per-motor status streamed by the firmware at the chosen
  period (device-side streaming; the page does not poll).

The page works offline too: without a device it draws the reference pose from
the geometry in `Geometry.md` and the drag / IK preview still runs.
