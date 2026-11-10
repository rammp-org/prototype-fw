#!/usr/bin/env python3
"""RAMMP MIB bench: plays the joystick / HMI side of the RTPS link from a PC.

The PEAR MIB firmware (ESP32-P4-ETH) talks to a joystick HMI over RTPS: it
publishes MibStatus, Diagnostics and one MotorState per motor axis, and acts on
the joystick's XYTwist stream, DriveCommand and SeatCommand. This window is that
joystick, so the MIB can be watched and driven before a physical joystick
exists. It is the mirror image of pace-hmi-fw's rtps_mcb_gui.py, which plays
the MIB for the real joystick.

Usage:
  python3 rtps_mib_gui.py --peer 10.0.0.133
  python3 rtps_mib_gui.py --peer 10.0.0.133 --advertised-address 100.92.133.114
  python3 rtps_mib_gui.py --headless --peer 10.0.0.133     # stdout + stdin, over ssh
  python3 rtps_mib_gui.py --self-test                      # no network

Both settings are editable in the window; the flags only prefill them. The
"Via" dropdown lists this PC's IPv4 addresses, which matters on a machine with
VirtualBox/Tailscale/VPN adapters: RTPS leaves by exactly one interface, and the
automatic pick is often a virtual one. --peer is what reaches a MIB that is
routed rather than on-link (Tailscale, a VPN, another subnet), since none of
those carry the multicast RTPS discovery needs.

Keyboard, while the joystick pad has focus (click it):
  W / S   forward / back        A / D   left / right
  Q / E   twist anticlockwise / clockwise        space   centre everything
  Esc     E-STOP (anywhere in the window)

Topics, type names, enums, tables and the wire layouts all come from
``rammp_spec.py``, which parses the shared rammp-rtps headers - the same ones
the firmware builds against.

The window is Qt (PySide6, or PyQt6 if that is what is installed); the RTPS
side stays stdlib only, so --headless and --self-test need no third-party
package. Participant id 16 (the HMI's tools use 10-15, the probe uses 19).
"""
from __future__ import annotations

import argparse
import os
import struct
import sys
import time
from typing import Any, Dict, List, Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rammp_spec as spec  # noqa: E402  (path setup must run first)
import rtps_host  # noqa: E402
import rtps_net  # noqa: E402
from mib_bench import *  # noqa: E402,F401,F403  (the model: harness, Stick, formatters, constants)
from mib_bench import MibBench, Stick, build_harness_args  # noqa: E402  (explicit for readers)

# ---------------------------------------------------------------- headless

HEADLESS_HELP = """commands:
  enable | disable          DriveCommand ENABLE / DISABLE with the current profile
  profile low|normal|high   set the profile and send a DriveCommand with it
  xy <x> <y> <twist>        hold the stick there (-1..+1; + = right, forward, clockwise)
  button on|off             the JOYSTICK button bit
  stream on|off             stream XYTwist even while the MIB is not ENABLED
  seat <axis> <target>      SeatCommand: axis by id, short (L1) or NAME; target in whole units,
                            or up / down (one table step) / zero
  estop                     zero XYTwist, then DISABLE
  status                    print the latest MibStatus, Diagnostics and MotorStates
  verbose on|off            show the harness's discovery chatter
  help                      this text
  quit"""


def _find_seat_axis(text: str) -> Optional[spec.SeatAxis]:
    key = text.strip().upper()
    for axis in spec.SEAT_AXES:
        if key in (str(axis.id), axis.short.upper(), axis.name.upper()):
            return axis
    return None


def _find_profile(text: str) -> Optional[int]:
    key = text.strip().upper()
    for value, name in spec.DRIVE_PROFILE_NAMES.items():
        if key in (name, str(value)):
            return value
    return None


def headless_command(harness: MibBench, line: str, say: Callable[[str], None] = print) -> bool:
    """Act on one stdin line; returns False to quit."""
    words = line.strip().split()
    if not words:
        return True
    command, args = words[0].lower(), words[1:]
    requests = spec.ENUM_VALUES["DriveRequest"]
    try:
        if command in ("q", "quit", "exit"):
            return False
        if command in ("h", "help", "?"):
            say(HEADLESS_HELP)
        elif command == "enable":
            harness.send_drive(requests["ENABLE"])
        elif command == "disable":
            harness.send_drive(requests["DISABLE"])
        elif command == "profile" and args:
            profile = _find_profile(args[0])
            if profile is None:
                say(f"  unknown profile '{args[0]}' - "
                    f"{', '.join(n.lower() for n in spec.DRIVE_PROFILE_NAMES.values())}")
            else:
                request = requests["ENABLE"] if harness.mib_enabled() else requests["DISABLE"]
                harness.send_drive(request, profile)
        elif command == "xy" and len(args) >= 3:
            harness.stick.hold = True
            harness.stick.set_pad(float(args[0]), float(args[1]))
            harness.stick.set_twist(float(args[2]))
            x, y, twist, _b = harness.stick.sample()
            say(f"  stick x={x:+.2f} y={y:+.2f} twist={twist:+.2f} "
                f"({'streaming' if harness.streaming_wanted() else 'not streaming: MIB not ENABLED, or `stream on`'})")
        elif command == "button" and args:
            harness.stick.button = args[0].lower() in ("on", "1", "true", "pressed")
        elif command == "stream" and args:
            harness.always_stream = args[0].lower() in ("on", "1", "true")
            say(f"  always stream: {harness.always_stream}")
        elif command == "seat" and len(args) >= 2:
            axis = _find_seat_axis(args[0])
            if axis is None:
                say("  unknown axis - " + ", ".join(f"{a.id}/{a.short}/{a.name}"
                                                     for a in spec.SEAT_AXES))
            elif args[1].lower() in ("up", "+"):
                harness.seat_step(axis.id, +1)
            elif args[1].lower() in ("down", "-"):
                harness.seat_step(axis.id, -1)
            elif args[1].lower() == "zero":
                harness.send_seat(axis.id, 0.0)
            else:
                harness.send_seat(axis.id, axis.to_units(axis.parse(args[1])))
        elif command == "estop":
            harness.estop()
        elif command == "status":
            snap = harness.snapshot()
            if snap["status"] is None:
                say("  no MibStatus yet")
            else:
                say(f"  status {describe_status(snap['status'], snap['status_gaps'])} "
                    f"age={snap['now'] - snap['status_at']:.1f}s rate={snap['status_rate']:.1f}Hz")
            if snap["diag"] is not None:
                say(f"  diag {describe_diagnostics(snap['diag'], snap['diag_gaps'])} "
                    f"age={snap['now'] - snap['diag_at']:.1f}s")
            for axis in spec.AXES:
                state, at, gaps, _rate = snap["motors"][axis.id]
                if state is not None:
                    say(f"  motor {describe_motor(axis, state, gaps)} age={snap['now'] - at:.1f}s")
        elif command == "verbose" and args:
            harness.verbose = args[0].lower() in ("on", "1", "true")
        else:
            say(f"  unknown command '{line.strip()}' - 'help' for the list")
    except ValueError as exc:
        say(f"  bad number: {exc}")
    return True


def _print_events(harness: MibBench, stop: threading.Event) -> None:
    """One stdout line per message, at most 2 Hz per topic (per axis for motors)."""
    last_print: Dict[str, float] = {}
    while not stop.is_set():
        try:
            kind, payload, axis = harness.events.get(timeout=0.5)  # type: ignore[union-attr]
        except queue.Empty:
            continue
        key = kind if axis is None else f"{kind}:{axis.id}"
        now = time.monotonic()
        if now - last_print.get(key, 0.0) < 0.5:
            continue
        last_print[key] = now
        if kind == "status":
            print(f"status {describe_status(payload, harness.status_gaps)}", flush=True)
        elif kind == "diag":
            print(f"diag {describe_diagnostics(payload, harness.diag_gaps)}", flush=True)
        elif kind == "motor":
            print(f"motor {describe_motor(axis, payload, harness.motors[axis.id].gaps)}",
                  flush=True)


def resolve_peer(cli: argparse.Namespace) -> Tuple[Optional[str], Optional[str]]:
    """Which MIB to talk to and from which adapter: the flags, then what worked last
    time, then passive discovery."""
    config = rtps_net.load_config()
    peer = (cli.peer[0] if cli.peer else None) or config.get(CONFIG_PEER)
    if peer is None:
        peer = rtps_net.discover_board(None, progress=print)
    advertised = cli.advertised_address or (rtps_net.source_address_for(peer) if peer else None) \
        or config.get(CONFIG_ADDRESS)
    return peer, advertised


def run_headless(cli: argparse.Namespace) -> int:
    peer, advertised = resolve_peer(cli)
    if peer is None:
        print("Could not find the MIB. Pass --peer <ip>, or run the GUI and use Scan.")
        return 1
    args = build_harness_args(cli, peer, advertised)
    print(f"MIB {peer} via {args.advertised_address}; spec from {spec.HEADER_DIR}")

    harness_box: List[MibBench] = []

    def sink(line: str) -> None:
        verbose = harness_box[0].verbose if harness_box else bool(cli.verbose)
        if verbose or not line.startswith(QUIET_PREFIXES):
            print(line, flush=True)

    # rtps_host.log prints every line itself and then calls the sink. Over ssh
    # the discovery chatter buries the data lines, so shadow that module's
    # `print` with a no-op and let the sink decide what reaches stdout.
    rtps_host.LOG_SINK = sink
    rtps_host.print = lambda *_args, **_kwargs: None
    try:
        try:
            harness = MibBench(args, cli.xy_hz)
        except OSError as exc:
            print(f"connect failed: {exc}")
            return 1
        harness.verbose = bool(cli.verbose)
        harness.events = queue.Queue(maxsize=4096)
        harness_box.append(harness)
        stop = threading.Event()
        network = threading.Thread(target=harness.run, daemon=True, name="rtps")
        network.start()
        printer = threading.Thread(target=_print_events, args=(harness, stop), daemon=True)
        printer.start()
        rtps_net.save_config({CONFIG_PEER: peer, CONFIG_ADDRESS: args.advertised_address})
        print(HEADLESS_HELP)
        try:
            while True:
                try:
                    line = input("mib> ")
                except (EOFError, KeyboardInterrupt):
                    break
                if not headless_command(harness, line):
                    break
        finally:
            stop.set()
            harness.stop()
            network.join(timeout=2.0)
    finally:
        rtps_host.LOG_SINK = None
        del rtps_host.print  # back to the builtin
    return 0


# --------------------------------------------------------------- self test


def run_self_test() -> int:
    """Encode/decode every message against hand-built byte vectors; no network."""
    failures: List[str] = []

    def check(name: str, condition: bool) -> None:
        print(f"  [{'PASS' if condition else 'FAIL'}] {name}")
        if not condition:
            failures.append(name)

    hdr = b"\x00\x01\x00\x00"
    print("spec:")
    check("seat axes parsed (>= 4, ids contiguous)", len(spec.SEAT_AXES) >= 4)
    check("diagnostics parsed (>= 3)", len(spec.DIAGNOSTICS) >= 3)
    check("motor axes parsed (>= 4)", len(spec.AXES) >= 4)
    check("MibSystemState IDLE == 1", spec.ENUM_VALUES["MibSystemState"].get("IDLE") == 1)
    check("DriveRequest ENABLE == 1", spec.ENUM_VALUES["DriveRequest"].get("ENABLE") == 1)
    check("DriveProfile HIGH == 2", spec.ENUM_VALUES["DriveProfile"].get("HIGH") == 2)
    check("Buttons JOYSTICK == 1", spec.ENUM_VALUES["Buttons"].get("JOYSTICK") == 1)
    check("AxisId UNASSIGNED == 255", spec.ENUM_VALUES["AxisId"].get("UNASSIGNED") == 255)
    check("AxisId DRIVE_LEFT == 2", spec.ENUM_VALUES["AxisId"].get("DRIVE_LEFT") == 2)
    check("BoardState FAULT == 4", spec.ENUM_VALUES["BoardState"].get("FAULT") == 4)
    check("FaultCode SAMPLER == 8", spec.ENUM_VALUES["FaultCode"].get("SAMPLER") == 8)
    check("SeatAxis enum follows the table",
          spec.ENUM_VALUES["SeatAxis"] == {a.name: a.id for a in spec.SEAT_AXES})
    check("DiagId enum follows the table",
          spec.ENUM_VALUES["DiagId"] == {d.name: d.id for d in spec.DIAGNOSTICS})
    check("motor_state topic derived", spec.motor_state_topic("drive_left")
          == "rammp/drive_left/motor_state")
    check("motor_command topic derived", spec.motor_command_topic("drive_left")
          == "rammp/mib/motor_command/drive_left")
    check("axis rows carry both topics", all(
        a.state_topic == f"rammp/{a.segment}/motor_state"
        and a.command_topic == f"rammp/mib/motor_command/{a.segment}" for a in spec.AXES))
    check("RAMMP_MOTOR_API_VERSION parsed", isinstance(spec.NUMBERS.get("MOTOR_API_VERSION"), int))
    check("topic strings", spec.TOPIC_MIB_STATUS == "rammp/mib/status"
          and spec.TYPE_MOTOR_STATE == "rammp/msg/MotorState")
    seat_fields = [f for f, _k in spec.layout("seatState")]
    check("seatState has one float per seat-axis row (same order)",
          len(seat_fields) == len(spec.SEAT_AXES)
          and all(k == "f" for _f, k in spec.layout("seatState")))
    first = spec.SEAT_AXES[0]
    check("SeatAxis.format/parse round-trip", first.parse(first.format(126)) == 126
          and first.to_raw(first.to_units(126)) == 126
          and first.clamp(first.parse("1e9")) == first.max_value)

    # -- XYTwist: three floats then a uint32, 16 bytes, no padding.
    print("XYTwist:")
    xy = hdr + struct.pack("<fffI", 0.25, -0.5, 1.0, 1)
    check("encode", spec.pack_xy_twist(0.25, -0.5, 1.0, 1) == xy)
    check("decode", spec.unpack_xy_twist(xy) == {"x": 0.25, "y": -0.5, "twist": 1.0, "buttons": 1})

    # -- DriveCommand: two bytes.
    print("DriveCommand:")
    drive = hdr + b"\x01\x02"
    check("encode", spec.pack_drive_command(1, 2) == drive)
    check("decode", spec.unpack_drive_command(drive) == {"request": 1, "profile": 2})

    # -- SeatCommand: uint8 axis, 3 pad bytes, float at offset 4.
    print("SeatCommand:")
    seat = hdr + b"\x02\x00\x00\x00" + struct.pack("<f", 250.0)
    check("encode", spec.pack_seat_command(2, 250.0) == seat)
    check("decode", spec.unpack_seat_command(seat) == {"axis": 2, "target": 250.0})

    # -- MibStatus, as rammp_rtps.py's MSG_MIB_STATUS lays it out, with the
    # seatState floats inline. Offsets after the header, N seat floats:
    #   0 systemState u8, 1 activeProfile u8, 2..3 pad, 4 .. 4+4N floats,
    #   then error_message (u32 len 9 + "No error\0"), pad to 8, int64 epoch_s,
    #   float speed, int16 utc_offset_min, u8 seq, 1 pad, status_text, error_footer.
    # N=8 (PEAR rows): floats end at 36, string ends at 49, int64 at 56..64,
    # speed 64, offset 68, seq 70, pad 71, status_text 72..84, footer 84..89.
    # N=4: 20, 33, 40..48, 48, 52, 54, 55, 56..68, 68..73.
    print("MibStatus:")
    seat_count = len(spec.SEAT_AXES)
    seat_values = ([12.5, -3.0, 250.0, 50.0, 1.5, -2.5, 0.0, 30.0] + [7.0] * seat_count)[:seat_count]
    epoch = 1_760_000_000
    # Hand-derived for the two known tables (4 rows: 33 -> 40, 8 rows: 49 -> 56). Any
    # other row count falls back to the arithmetic, which is no longer independent.
    int64_pad = {4: 7, 8: 7}.get(seat_count, -(17 + 4 * seat_count) % 8)
    status = hdr + b"".join((
        b"\x02\x01\x00\x00",
        b"".join(struct.pack("<f", v) for v in seat_values),
        struct.pack("<I", 9) + b"No error\x00",
        b"\x00" * int64_pad,
        struct.pack("<q", epoch),
        struct.pack("<f", 1.25),
        struct.pack("<h", -300),
        b"\x07",
        b"\x00",
        struct.pack("<I", 8) + b"DRIVING\x00",
        struct.pack("<I", 1) + b"\x00",
    ))
    expected_len = {8: 93, 4: 77}.get(seat_count, 54 + 4 * seat_count + int64_pad)
    check(f"vector is {expected_len} bytes for {seat_count} seat floats",
          len(status) == expected_len)
    encoded = spec.pack_mib_status(2, 1, seat_values, seq=7, speed=1.25, status_text="DRIVING",
                                   error_message="No error", epoch_s=epoch, utc_offset_min=-300)
    check("encode", encoded == status)
    decoded = spec.unpack_mib_status(status)
    check("decode", decoded is not None and decoded["systemState"] == 2
          and decoded["activeProfile"] == 1 and decoded["error_message"] == "No error"
          and decoded["epoch_s"] == epoch and decoded["speed"] == 1.25
          and decoded["utc_offset_min"] == -300 and decoded["seq"] == 7
          and decoded["status_text"] == "DRIVING" and decoded["error_footer"] == "")
    check("seat values in table order", decoded is not None
          and spec.seat_values(decoded) == seat_values)
    check("truncated sample decodes to None", spec.unpack_mib_status(status[:-3]) is None)
    check("big-endian header rejected", spec.unpack_mib_status(b"\x00\x00" + status[2:]) is None)
    check("describe_status", decoded is not None and "state=ENABLED" in describe_status(decoded))
    check("format_clock uses the MIB's offset", decoded is not None
          and format_clock(decoded).endswith("(UTC-05:00)"))

    # -- Diagnostics: u8 seq, 3 pad, u32 item count, then per item u32 count + int32s:
    # 4 (header) + 8 + 16 per row.
    print("Diagnostics:")
    for rows in (3, 11):
        values = [[234 + i, 150 + i, -450 + i] for i in range(rows)]
        diag = hdr + b"\x05\x00\x00\x00" + struct.pack("<I", rows) + b"".join(
            struct.pack("<I", 3) + struct.pack("<iii", *row) for row in values)
        check(f"{rows} rows: vector is {12 + rows * 16} bytes", len(diag) == 12 + rows * 16)
        check(f"{rows} rows: encode", spec.pack_diagnostics(values, 5) == diag)
        check(f"{rows} rows: decode", spec.unpack_diagnostics(diag) == (5, values))
    check("empty readings", spec.unpack_diagnostics(spec.pack_diagnostics([[], [1]], 0))
          == (0, [[], [1]]))
    check("DiagItem.format", spec.DIAGNOSTICS[0].format(2345, 1) == "23.45")

    # -- MotorState, by hand from the struct (offsets after the header):
    #   0..9 ten u8s, 10 cmd_age_ms u16, 12 drv_status u16, 14..15 pad,
    #   16 uptime_ms u32, 20 fw_version u32, 24..64 ten floats, 64..80 temps[4]. 80 bytes.
    print("MotorState:")
    motor_values = {
        "api_version": 1, "axis_id": 4, "seq": 200, "last_cmd_seq": 199, "state": 1, "mode": 2,
        "fault_code": 0, "clear_fault_ack": 3, "vbus_measured": 1, "flags": 0,
        "cmd_age_ms": 15, "drv_status": 0x0102, "uptime_ms": 123456789,
        "fw_version": (1 << 24) | (2 << 16) | 3,
        "position": 3.5, "velocity": -1.25, "torque_est": 0.75, "iq": 2.5, "id": -0.5,
        "vbus": 48.0, "ibus_est": 0.125, "torque_limit_eff": 4.0, "vel_limit_eff": 20.0,
        "accel_limit_eff": 0.0, "temps": [30.0, 31.0, 32.5, 33.0],
    }
    motor = hdr + b"".join((
        bytes([1, 4, 200, 199, 1, 2, 0, 3, 1, 0]),
        struct.pack("<H", 15), struct.pack("<H", 0x0102), b"\x00\x00",
        struct.pack("<I", 123456789), struct.pack("<I", 0x01020003),
        struct.pack("<ffffffffff", 3.5, -1.25, 0.75, 2.5, -0.5, 48.0, 0.125, 4.0, 20.0, 0.0),
        struct.pack("<ffff", 30.0, 31.0, 32.5, 33.0),
    ))
    check("vector is 80 bytes after the header", len(motor) == 84)
    check("encode", spec.pack_motor_state(motor_values) == motor)
    check("decode", spec.unpack_motor_state(motor) == motor_values)
    check("truncated sample decodes to None", spec.unpack_motor_state(motor[:-1]) is None)
    check("layout is 80 bytes with zero values", len(spec.pack_motor_state({})) == 84)
    # MotorCommand, the other direction (28 bytes: four u8 then six floats).
    command = hdr + bytes([9, 1, 2, 0]) + struct.pack("<ffffff", 0.0, 1.2, 0.0, 0.0, 0.0, 0.0)
    check("MotorCommand encode (28 bytes)", spec.encode("MotorCommand", {
        "seq": 9, "requested_state": 1, "mode": 2, "velocity": 1.2}) == command)
    check("describe_motor", "state=ARMED" in describe_motor(spec.AXES[0], motor_values))

    # -- The stick model.
    print("stick:")
    stick = Stick()
    stick.set_pad(3.0, 4.0)
    check("pad clamps to the unit circle", abs(math.hypot(stick.x, stick.y) - 1.0) < 1e-9
          and abs(stick.x - 0.6) < 1e-9 and abs(stick.y - 0.8) < 1e-9)
    stick.set_twist(-7.0)
    stick.button = True
    check("sample", stick.sample() == (0.6, 0.8, -1.0, 1))
    stick.centre()
    check("centre", stick.sample() == (0.0, 0.0, 0.0, 1))

    # -- A harness on loopback: endpoints, and routing by addressed reader id.
    print("harness (loopback, no peer traffic):")
    try:
        cli = argparse.Namespace(node_name=NODE_NAME, domain_id=0, participant_id=37,
                                 multicast_interface=None, multicast_group="239.255.0.1",
                                 trace_packets=False, peer_participant_ids=[0])
        bench = MibBench(build_harness_args(cli, "127.0.0.1", "127.0.0.1"), xy_hz=30.0)
    except OSError as exc:
        print(f"  [SKIP] could not open loopback sockets: {exc}")
        bench = None
    if bench is not None:
        try:
            check("writers: XYTwist, DriveCommand, SeatCommand",
                  [w.topic_name for w in bench.local_writers] == [
                      spec.TOPIC_JOYSTICK_XY_TWIST, spec.TOPIC_JOYSTICK_DRIVE_COMMAND,
                      spec.TOPIC_JOYSTICK_SEAT_COMMAND])
            check("writer type names", [w.type_name for w in bench.local_writers] == [
                spec.TYPE_XY_TWIST, spec.TYPE_DRIVE_COMMAND, spec.TYPE_SEAT_COMMAND])
            check("readers: MibStatus, Diagnostics, every axis",
                  [r.topic_name for r in bench.local_readers] == [
                      spec.TOPIC_MIB_STATUS, spec.TOPIC_MCB_DIAGNOSTICS]
                  + [a.state_topic for a in spec.AXES])
            check("reader type names", [r.type_name for r in bench.local_readers] == [
                spec.TYPE_MIB_STATUS, spec.TYPE_DIAGNOSTICS]
                + [spec.TYPE_MOTOR_STATE] * len(spec.AXES))
            check("everything best-effort", not any(
                e.reliable for e in bench.local_writers + bench.local_readers))
            check("entity indexes unique", len({w.entity_index for w in bench.local_writers})
                  == len(bench.local_writers)
                  and len({r.entity_index for r in bench.local_readers}) == len(bench.local_readers))
            prefix = rtps_host.make_guid_prefix("mib", 0, 1)
            writer_id = rtps_host.entity_id_for_index(0, rtps_host.USER_WRITER_NO_KEY_KIND)

            def addressed(reader_index: int, cdr: bytes) -> bytes:
                reader_id = rtps_host.entity_id_for_index(reader_index,
                                                          rtps_host.USER_READER_NO_KEY_KIND)
                return rtps_host.build_rtps_message(prefix, reader_id, writer_id, 1, cdr)

            bench.handle_user_packet(addressed(0, status), "127.0.0.1", 7411)
            check("MibStatus routed by reader id", bench.status is not None
                  and bench.status["seq"] == 7 and bench.status_count == 1)
            bench.handle_user_packet(addressed(0, spec.pack_mib_status(
                2, 1, seat_values, seq=10, error_message="No error")), "127.0.0.1", 7411)
            check("seq gap counted (7 -> 10 = 2 missing)", bench.status_gaps == 2)
            check("seat_raw reads the table units", bench.seat_raw(0) == 125)
            bench.handle_user_packet(addressed(1, diag), "127.0.0.1", 7411)
            check("Diagnostics routed", bench.diagnostics is not None
                  and len(bench.diagnostics["items"]) == 11)
            last_axis = spec.AXES[-1]
            bench.handle_user_packet(addressed(2 + last_axis.id, motor), "127.0.0.1", 7411)
            check("MotorState routed to its axis", bench.motors[last_axis.id].state is not None
                  and bench.motors[last_axis.id].state["axis_id"] == 4)
            check("mib_enabled follows the status", bench.mib_enabled())
            check("streaming wanted while ENABLED", bench.streaming_wanted())
            # a step is NOT clamped to the row (the MIB's own limits decide; the row
            # is the HMI's jog range): from the row's max it goes one step past it
            check("seat_step steps up past the row's max (unclamped)",
                  bench.seat_step(2, +1)
                  == spec.SEAT_AXES[2].to_units(spec.SEAT_AXES[2].max_value
                                                + spec.SEAT_AXES[2].step))
            check("seat_step steps down", bench.seat_step(0, -1)
                  == spec.SEAT_AXES[0].to_units(125 - spec.SEAT_AXES[0].step))
            check("headless 'seat L1 12.3' (or by id)", headless_command(
                bench, f"seat {spec.SEAT_AXES[-1].short} 12.3", say=lambda _m: None)
                and bench.last_seat_command == (spec.SEAT_AXES[-1].id, 12.3))
            check("headless 'xy' holds the stick", headless_command(
                bench, "xy 0.5 -0.25 0.1", say=lambda _m: None)
                and bench.stick.sample() == (0.5, -0.25, 0.1, 0))
            check("headless 'profile high'", headless_command(bench, "profile high",
                                                              say=lambda _m: None)
                  and bench.profile == 2)
            check("headless quit", headless_command(bench, "quit") is False)
        finally:
            bench.close()

    print()
    print("rtps_host self test:")
    host_result = rtps_host.run_self_test()
    check("rtps_host.run_self_test() passed", host_result == 0)

    if failures:
        print(f"\nSELF-TEST FAILED ({len(failures)} check(s)): {', '.join(failures)}")
        return 1
    print("\nSELF-TEST PASSED")
    return 0


# ------------------------------------------------------------------- main


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Bench joystick for the RAMMP MIB over RTPS: a window (default), a "
                    "stdin/stdout mode (--headless), or the codec self test (--self-test).")
    parser.add_argument("--peer", action="append", default=None, metavar="HOST",
                        help="The MIB's hostname or IP: prefills the window, and seeds "
                             "discovery by unicast for a routed link (Tailscale, VPN, another "
                             "subnet) where multicast does not reach")
    parser.add_argument("--advertised-address", default=None,
                        help="Local IPv4 address to advertise (prefills 'Via'); set it when "
                             "the automatic pick lands on a VirtualBox/Tailscale/WSL adapter")
    parser.add_argument("--node-name", default=NODE_NAME, help="Local participant name")
    parser.add_argument("--domain-id", type=int, default=0, help="RTPS domain id (the MIB's)")
    parser.add_argument("--participant-id", type=int, default=DEFAULT_PARTICIPANT_ID,
                        help=f"Local participant id (default {DEFAULT_PARTICIPANT_ID}; the "
                             "HMI's bench tools use 10-15)")
    parser.add_argument("--peer-participant-ids", type=rtps_host.parse_participant_id_range,
                        default="0-3", metavar="IDS",
                        help="Participant ids to try on the peer, as '0-3' or '0,1,2'")
    parser.add_argument("--multicast-interface", default=None,
                        help="IPv4 interface for the multicast join/send, if it differs")
    parser.add_argument("--multicast-group", default="239.255.0.1",
                        help="RTPS metatraffic multicast group")
    parser.add_argument("--xy-hz", type=float, default=DEFAULT_XY_HZ,
                        help=f"XYTwist samples per second while streaming (default {DEFAULT_XY_HZ:g})")
    parser.add_argument("--headless", action="store_true",
                        help="No window: print MibStatus/Diagnostics/MotorState lines, read "
                             "commands from stdin")
    parser.add_argument("--self-test", action="store_true",
                        help="Check the codec against hand-built byte vectors and exit")
    parser.add_argument("--list-interfaces", action="store_true",
                        help="Print this host's IPv4 adapters and exit")
    parser.add_argument("--verbose", action="store_true",
                        help="Show the harness's discovery chatter in the log")
    parser.add_argument("--trace-packets", action="store_true",
                        help="Log every received UDP packet and its RTPS submessage headers")
    cli = parser.parse_args()

    if cli.self_test:
        return run_self_test()
    if cli.list_interfaces:
        print("network adapters (pass one as --advertised-address):")
        for adapter in rtps_net.list_adapters():
            print(f"  {adapter.label()}")
        return 0
    if cli.xy_hz <= 0:
        parser.error("--xy-hz must be > 0")
    if cli.headless:
        return run_headless(cli)
    try:
        import mib_window_qt
    except ImportError as exc:
        print(f"the window needs PySide6 (or PyQt6): {exc}\n"
              "  python3 -m venv ~/.venvs/rammp && ~/.venvs/rammp/bin/pip install PySide6\n"
              "  ~/.venvs/rammp/bin/python tools/rtps_mib_gui/rtps_mib_gui.py ...\n"
              "or use --headless (no third-party package needed)", file=sys.stderr)
        return 2
    return mib_window_qt.run(cli)


if __name__ == "__main__":
    sys.exit(main())
