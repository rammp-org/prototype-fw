# RAMMP MIB bench (`rtps_mib_gui.py`)

A PC-side stand-in for the joystick / HMI, so the PEAR MIB firmware
(ESP32-P4-ETH) can be watched and driven over RTPS before a physical joystick
exists. It is the mirror image of `pace-hmi-fw/scripts/rtps_mcb_gui.py`, which
plays the MIB for the real joystick: this tool subscribes to everything the MIB
publishes and publishes everything the joystick would.

Stdlib only (tkinter for the window). Python 3.10+.

## Running

```sh
python3 tools/rtps_mib_gui/rtps_mib_gui.py --peer <mib-ip>
python3 tools/rtps_mib_gui/rtps_mib_gui.py --peer <mib-ip> --advertised-address <this-pc-ip>
python3 tools/rtps_mib_gui/rtps_mib_gui.py --headless --peer <mib-ip>    # over ssh
python3 tools/rtps_mib_gui/rtps_mib_gui.py --self-test                   # no network
python3 tools/rtps_mib_gui/rtps_mib_gui.py --list-interfaces
```

Flags: `--peer`, `--advertised-address`, `--domain-id` (0), `--participant-id`
(16; the HMI's bench tools use 10-15, the discovery probe uses 19),
`--peer-participant-ids` (`0-3`), `--multicast-interface`, `--multicast-group`,
`--xy-hz` (30), `--verbose`, `--trace-packets`. The peer and adapter are
remembered (`~/.config/rammp-rtps/config.json`, keys `mib_peer` /
`mib_advertised_address`; the HMI tools keep theirs under other keys in the
same file), so after the first run `--peer` can be left off. With a peer
prefilled the window connects on open.

The spec is read from the C++ headers at start, so the tool needs the
rammp-rtps submodule: `git submodule update --init` in the prototype-fw repo if
it says it cannot find them. `python3 tools/rtps_mib_gui/rammp_spec.py` prints
everything it parsed.

### Which adapter

From the HMI tool's docstring, which is still the advice:

> Multi-homed PCs (VirtualBox, Tailscale, WSL, VPNs) are the usual reason
> nothing arrives: RTPS discovery goes out of exactly one interface, and the
> automatic pick is often a virtual one. `--list-interfaces` shows the
> candidates and `--advertised-address` forces the choice;
> `--multicast-interface` does the same for the multicast join if it differs.
>
> `--peer` is for a board that is routed rather than on-link - over Tailscale,
> a VPN or another subnet - since none of those carry the multicast RTPS
> discovery needs. Given the peer's address the tool unicasts its SPDP/SEDP
> straight at it.

In the window the "Via" dropdown defaults to Auto, which asks the kernel which
source address a packet to the MIB would leave from - the adapter that routes
to the board, not the one that reaches the internet. Detect tries the saved
address, `espressif` / `espressif.local`, then listens for multicast; Scan
sweeps a subnet with discovery probes.

## Python with Tk

The GUI needs a Python with `tkinter` **8.6 or newer**. Homebrew's `python3`
ships without it (`ModuleNotFoundError: No module named '_tkinter'`):
`brew install python-tk@3.14` (match your Homebrew Python version, `python3
--version`). Apple's interpreter (`/usr/bin/python3`, Xcode's Python 3.9) has
tkinter, but its Tk 8.5 opens a window that stays blank on current macOS, so
the tool refuses it. `--headless` and `--self-test` work with any Python 3.10+.

## Panels and what they are on the wire

| Panel | Wire |
| --- | --- |
| Connection | SPDP/SEDP: one participant (`rammp-mib-bench`), 3 best-effort writers, 2 + N best-effort readers |
| MibStatus | reader on `rammp/mib/status` (`rammp/msg/MibStatus`): link state (waiting / connected / LOST after 2 s), `systemState` (coloured), `activeProfile`, `speed` (m/s and mph), `seq` + gap count + rate, `epoch_s` + `utc_offset_min` as the MIB's wall clock, `status_text`, `error_message`, `error_footer`, every `currentSeatState` field labelled from the seat-axis table |
| Drive | writer `rammp/joystick/drive_command` (`DriveCommand{request, profile}`) for Enable / Disable / profile radio; writer `rammp/joystick/xy_twist` (`XYTwist{x, y, twist, buttons}`) streamed at `--xy-hz` while the MIB reports ENABLED or "always stream" is ticked; E-STOP = one zero XYTwist, then DISABLE |
| Actuators | writer `rammp/joystick/seat_command` (`SeatCommand{axis, target}`), target absolute in whole units; `-`/`+` step one table `step` from the value the MIB last reported (min if unknown), clamped, like the HMI's `seat_step`; Set sends a typed target; Zero sends 0 where the range includes it. Greyed unless the MIB is IDLE (it refuses otherwise) - "force" overrides for bench use |
| Diagnostics | reader on `rammp/mcb/diagnostics` (`Diagnostics{seq, items[]{values[]}}`): one row per diag-table item, readings formatted with the row's decimals and unit, age, rate; red when stale (> 2 s) |
| Motors | one reader per `RAMMP_AXIS_TABLE` row on `rammp/<segment>/motor_state` (`rammp/msg/MotorState`), all subscribed: board state, mode, fault, position (rad and deg), velocity (rad/s and rpm), torque, iq, vbus, temps, cmd age, uptime, age/rate. Grey until seen, red when stale (> 1 s), orange on FAULT |
| Log | `rtps_host`'s log lines; "verbose" shows the discovery chatter |

Conventions (from the spec comments): x + = right, y + = forward, twist + =
clockwise; axes are -1..+1 within the unit circle. The stream stops with one
zero sample when driving ends and on disconnect.

Timing the HMI uses (hard-coded in `rammp_spec.py`, since the HMI's own header
is not in the shared submodule): MibStatus every 500 ms, link lost after
2000 ms; Diagnostics every 500 ms, stale after 2000 ms.

## Keyboard

Click the joystick pad to give it focus.

| Key | Action |
| --- | --- |
| W / S | forward / back (y) |
| A / D | left / right (x) |
| Q / E | twist anticlockwise / clockwise |
| space | centre everything |
| Esc | E-STOP (anywhere in the window) |

A held key ramps its axis to full in 0.4 s; releasing springs it back unless
"hold" is ticked. A key with no release event and no OS key-repeat for 2.5 s is
treated as released. The mouse works too: drag on the pad, the twist slider
below it; both spring back on release unless "hold" is ticked.

## Headless mode

`--headless` prints one line per received message to stdout (`status ...`,
`diag ...`, `motor <segment> ...`), rate-limited to 2 Hz per topic, and reads
commands from stdin:

```
enable | disable          DriveCommand ENABLE / DISABLE with the current profile
profile low|normal|high   set the profile and send a DriveCommand with it
xy <x> <y> <twist>        hold the stick there (-1..+1)
button on|off             the JOYSTICK button bit
stream on|off             stream XYTwist even while the MIB is not ENABLED
seat <axis> <target>      axis by id, short (L1) or NAME; target in whole units,
                          or up / down (one table step) / zero
estop                     zero XYTwist, then DISABLE
status                    print the latest MibStatus, Diagnostics and MotorStates
verbose on|off            show the harness's discovery chatter
help, quit
```

## Where the rows come from

Topics, type names, enums (`MibSystemState`, `DriveProfile`, `DriveRequest`,
`Buttons`, `SeatAxis`, `DiagId`, `AxisId`, `RequestedState`, `ControlMode`,
`BoardState`, `FaultCode`), the three X-macro tables
(`RAMMP_SEAT_AXIS_TABLE`, `RAMMP_DIAG_TABLE`, `RAMMP_AXIS_TABLE`), the derived
per-axis topics, `RAMMP_MOTOR_API_VERSION` and every message struct are parsed
by `rammp_spec.py` from
`external/rammp-rtps/components/rammp_rtps_messages/include/messages/{joystick,mib,motor}_message.hpp`
at import. A row added to a table or a field appended to a struct shows up in
the window without touching the tool. The PEAR rows (seat axes `L1..L4`,
diagnostics `W1..W4`/`L1..L4`, motor axes `pear_*`) appear once
`pear/spec/rammp-rtps-pear.patch` is applied to the submodule; the tool works
with or without them.

Encoding is XCDR1 as espp/cdr writes it: 4-byte header `00 01 00 00`,
little-endian, each primitive aligned to its size counted from the first
payload byte, `std::string` = u32 length including NUL + bytes + NUL,
`std::vector` = u32 count + elements, `std::array` = elements only, nested
structs inline. `--self-test` checks every message against hand-built byte
vectors and runs the vendored `rtps_host` self test.

## Files

| File | Origin |
| --- | --- |
| `rtps_mib_gui.py` | this tool: window, `MibBench` harness subclass, headless mode, self test |
| `rammp_spec.py` | this tool: header parser + XCDR1 codec (same spirit as `pace-hmi-fw/scripts/rammp_rtps.py`, which cannot be used as-is because it needs the HMI's `main/hmi_rtps_spec.hpp`) |
| `rtps_host.py` | vendored verbatim from `pace-hmi-fw/scripts/rtps_host.py` @ d8efcbc; keep in sync |
| `rtps_net.py` | vendored from `pace-hmi-fw/scripts/rtps_net.py` @ d8efcbc with two local changes (imports `rammp_spec`; the discovery probe carries the joystick's endpoints); keep in sync |
