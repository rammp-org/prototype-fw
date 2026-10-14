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

tkinter only, no third-party dependency, matching the stdlib-only RTPS side.
Participant id 16 (the HMI's tools use 10-15, the probe uses 19).
"""

from __future__ import annotations

import argparse
import datetime
import math
import os
import queue
import struct
import sys
import threading
import time
from collections import deque
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rammp_spec as spec  # noqa: E402  (path setup must run first)
import rtps_host  # noqa: E402
import rtps_net  # noqa: E402

# tkinter is imported by _import_tk(), from the GUI entry point only: --self-test,
# --headless and `python3 -m py_compile` must work on a Python built without Tk.
# MibPanel refers to these names at call time, never at import time.
tk: Any = None
ttk: Any = None
simpledialog: Any = None


def _import_tk() -> bool:
    """Bind the tkinter names this module's GUI uses; False if Tk is unavailable."""
    global tk, ttk, simpledialog
    try:
        import tkinter as tkinter_module
        from tkinter import simpledialog as simpledialog_module, ttk as ttk_module
    except ImportError:
        return False
    tk, ttk, simpledialog = tkinter_module, ttk_module, simpledialog_module
    return True

NODE_NAME = "rammp-mib-bench"
#: distinct from the HMI's bench tools (10-15) and rtps_net's probe (19)
DEFAULT_PARTICIPANT_ID = 16
#: XYTwist samples per second while streaming
DEFAULT_XY_HZ = 30.0

LOG_MAX_LINES = 600
#: GUI refresh of the status/diagnostics/motor panels
TICK_MS = 100
#: virtual joystick animation and keyboard integration
JOY_TICK_MS = 33
#: a held key moves its axis 0 -> 1 in 1/KEY_RATE seconds
KEY_RATE = 2.5
#: a released axis springs back 1 -> 0 in 1/SPRING_RATE seconds
SPRING_RATE = 6.0
#: a key with no OS key-repeat and no release event is dropped after this long
KEY_STUCK_S = 2.5
#: a MotorState row with nothing for this long reads as stale
MOTOR_STALE_S = spec.MOTOR_STATE_TIMEOUT_MS / 1000.0

#: "Via" entry meaning "work it out from whichever adapter reaches the peer".
AUTO_ADAPTER = "Auto (follow the route to the MIB)"
#: config keys; the file is shared with the HMI tools, which store their own board under "peer"
CONFIG_PEER = "mib_peer"
CONFIG_ADDRESS = "mib_advertised_address"

#: log lines the "verbose" toggle hides: discovery chatter the harness repeats every period
QUIET_PREFIXES = ("[spdp]", "[sedp]", "[raw:", "[multicast] joined", "[publish] no discovered",
                  "[peer] seeding")

STATE_COLOURS = {
    "INITIALIZING": "#b06a00", "IDLE": "#1a6dd4", "ENABLED": "#1e8a2e", "ERROR": "#c62828",
}


# ------------------------------------------------------------------ model


class RateMeter:
    """Messages per second over the last few arrivals; 0 once they stop."""

    def __init__(self, window: int = 16) -> None:
        self.times: Deque[float] = deque(maxlen=window)

    def mark(self, now: float) -> None:
        self.times.append(now)

    def rate(self, now: float, timeout_s: float) -> float:
        if len(self.times) < 2 or now - self.times[-1] > timeout_s:
            return 0.0
        span = self.times[-1] - self.times[0]
        return (len(self.times) - 1) / span if span > 0 else 0.0


class Stick:
    """The virtual joystick: x/y within the unit circle, twist -1..+1, one button.

    Conventions from the spec: x + = right, y + = forward, twist + = clockwise.
    Written by the GUI/stdin thread, read by the stream thread; the fields are
    plain floats so a torn read is at worst one sample off."""

    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.twist = 0.0
        self.button = False
        #: keep the deflection after the mouse/key is released
        self.hold = False

    @staticmethod
    def clamp_circle(x: float, y: float) -> Tuple[float, float]:
        radius = math.hypot(x, y)
        if radius > 1.0:
            return x / radius, y / radius
        return x, y

    def set_pad(self, x: float, y: float) -> None:
        self.x, self.y = self.clamp_circle(x, y)

    def set_twist(self, twist: float) -> None:
        self.twist = max(-1.0, min(1.0, twist))

    def centre(self) -> None:
        self.x = self.y = self.twist = 0.0

    def sample(self) -> Tuple[float, float, float, int]:
        buttons = spec.ENUM_VALUES.get("Buttons", {}).get("JOYSTICK", 1) if self.button else 0
        return (self.x, self.y, self.twist, buttons)


class MotorRecord:
    """The last MotorState seen on one axis's topic, and when."""

    def __init__(self) -> None:
        self.state: Optional[Dict[str, Any]] = None
        self.at = 0.0
        self.count = 0
        self.gaps = 0
        self._seq: Optional[int] = None
        self.meter = RateMeter()

    def take(self, state: Dict[str, Any], now: float) -> None:
        if self._seq is not None:
            self.gaps += (state["seq"] - self._seq - 1) & 0xFF
        self._seq = state["seq"]
        self.state = state
        self.at = now
        self.count += 1
        self.meter.mark(now)


# ---------------------------------------------------------------- harness


class MibBench(rtps_host.RtpsHostHarness):
    """Plays the joystick: streams XYTwist, sends Drive/SeatCommand, takes the MIB's topics."""

    def __init__(self, args: argparse.Namespace, xy_hz: float = DEFAULT_XY_HZ) -> None:
        super().__init__(args)
        self.xy_hz = max(1.0, float(xy_hz))
        # Writer 0 (XYTwist) comes from --publish-topic; the two command writers
        # are appended the way rtps_mcb_sim.py appends its extra endpoints: the
        # harness builds every SEDP announcement and target list from these lists.
        self._xy_writer = self.local_writers[0]
        self._drive_writer = rtps_host.WriterConfig(
            topic_name=spec.TOPIC_JOYSTICK_DRIVE_COMMAND, type_name=spec.TYPE_DRIVE_COMMAND,
            reliable=False, entity_index=len(self.local_writers))
        self.local_writers.append(self._drive_writer)
        self._seat_writer = rtps_host.WriterConfig(
            topic_name=spec.TOPIC_JOYSTICK_SEAT_COMMAND, type_name=spec.TYPE_SEAT_COMMAND,
            reliable=False, entity_index=len(self.local_writers))
        self.local_writers.append(self._seat_writer)
        # Reader 0 (MibStatus) comes from --subscribe-topic; Diagnostics and every
        # axis's MotorState topic get their own type names here.
        self.local_readers.append(rtps_host.ReaderConfig(
            topic_name=spec.TOPIC_MCB_DIAGNOSTICS, type_name=spec.TYPE_DIAGNOSTICS,
            reliable=False, entity_index=len(self.local_readers)))
        self._axis_by_topic: Dict[str, spec.Axis] = {}
        for axis in spec.AXES:
            self._axis_by_topic[axis.state_topic] = axis
            self.local_readers.append(rtps_host.ReaderConfig(
                topic_name=axis.state_topic, type_name=spec.TYPE_MOTOR_STATE,
                reliable=False, entity_index=len(self.local_readers)))

        self.lock = threading.Lock()
        self.stick = Stick()
        #: stream XYTwist even while the MIB is not ENABLED
        self.always_stream = False
        #: headless mode: let the harness's discovery chatter through to stdout
        self.verbose = False
        self.profile = spec.ENUM_VALUES["DriveProfile"].get("NORMAL", 1)
        #: what we last asked for on DriveCommand (None = nothing yet)
        self.last_drive_request: Optional[int] = None
        self.last_seat_command: Optional[Tuple[int, float]] = None

        self.status: Optional[Dict[str, Any]] = None
        self.status_at = 0.0
        self.status_count = 0
        self.status_gaps = 0
        self._status_seq: Optional[int] = None
        self.status_meter = RateMeter()
        self.diagnostics: Optional[Dict[str, Any]] = None
        self.diag_at = 0.0
        self.diag_count = 0
        self.diag_gaps = 0
        self._diag_seq: Optional[int] = None
        self.diag_meter = RateMeter()
        self.motors: Dict[int, MotorRecord] = {axis.id: MotorRecord() for axis in spec.AXES}
        #: ("status" | "diag" | "motor", payload dict, axis or None); headless sets it
        self.events: Optional["queue.Queue[tuple]"] = None

        self.xy_sent = 0
        self.xy_targets = -1
        self._was_streaming = False
        self._stream_stop = threading.Event()
        self._stream_thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------ sending

    def _build_user_targets(self, writer: rtps_host.WriterConfig) -> List[Tuple[str, int]]:
        # The stream thread and the GUI call this while the network thread
        # fills the discovery dicts; iterating one mid-insert raises. Retry.
        for _attempt in range(3):
            try:
                return super()._build_user_targets(writer)
            except RuntimeError:
                continue
        return []

    def _send_on(self, writer: rtps_host.WriterConfig, cdr_payload: bytes) -> int:
        """Publish one sample on `writer`; returns how many targets it went to."""
        payload = self.build_data_message(writer, cdr_payload)
        targets = self._build_user_targets(writer)
        for destination in targets:
            self.send_user_datagram(payload, destination)
        return len(targets)

    def publish_stick(self, sample: Optional[Tuple[float, float, float, int]] = None) -> int:
        """One XYTwist: the current stick, or `sample`."""
        x, y, twist, buttons = sample if sample is not None else self.stick.sample()
        sent_to = self._send_on(self._xy_writer, spec.pack_xy_twist(x, y, twist, buttons))
        self.xy_sent += 1
        if sent_to != self.xy_targets:
            self.xy_targets = sent_to
            rtps_host.log(f"[xy] streaming '{self._xy_writer.topic_name}' to {sent_to} target(s)"
                          if sent_to else f"[xy] no target for '{self._xy_writer.topic_name}' "
                          "yet - waiting for the MIB's discovery")
        return sent_to

    def publish_now(self) -> None:
        """The harness's periodic hook (unused: publish_interval is 0; the stream thread
        keeps its own clock). Kept so a caller's publish_now() sends a stick sample."""
        self.publish_stick()

    def send_drive(self, request: int, profile: Optional[int] = None) -> int:
        """One DriveCommand. The MIB decides: the status panel says whether it took."""
        if profile is not None:
            self.profile = profile
        self.last_drive_request = request
        sent_to = self._send_on(self._drive_writer, spec.pack_drive_command(request, self.profile))
        rtps_host.log(f"[drive] sent {spec.enum_name('DriveRequest', request)} "
                      f"profile={spec.enum_name('DriveProfile', self.profile)} "
                      f"to {sent_to} target(s)")
        return sent_to

    def send_seat(self, axis_id: int, target_units: float) -> int:
        """One SeatCommand: put `axis_id` at `target_units` (whole units, as the HMI sends)."""
        self.last_seat_command = (axis_id, target_units)
        sent_to = self._send_on(self._seat_writer, spec.pack_seat_command(axis_id, target_units))
        axis = spec.SEAT_AXES[axis_id] if 0 <= axis_id < len(spec.SEAT_AXES) else None
        name = f"{axis.short} {axis.label}" if axis else f"axis #{axis_id}"
        shown = format_seat(axis, target_units) if axis else f"{target_units:g}"
        rtps_host.log(f"[seat] sent {name} -> {shown} to {sent_to} target(s)")
        return sent_to

    def seat_step(self, axis_id: int, direction: int) -> Optional[float]:
        """One step from where the MIB last said the axis is (min if unknown), like the
        HMI's seat_step: clamped, absolute, whole units. Returns the target sent."""
        if not 0 <= axis_id < len(spec.SEAT_AXES):
            return None
        axis = spec.SEAT_AXES[axis_id]
        current = self.seat_raw(axis_id)
        start = axis.min_value if current is None else current
        target_raw = axis.clamp(start + direction * axis.step)
        target = axis.to_units(target_raw)
        self.send_seat(axis_id, target)
        return target

    def seat_raw(self, axis_id: int) -> Optional[int]:
        """The axis's last reported value in raw table units, or None."""
        with self.lock:
            status = self.status
        if status is None:
            return None
        values = spec.seat_values(status)
        if axis_id >= len(values):
            return None
        return spec.SEAT_AXES[axis_id].to_raw(values[axis_id])

    def estop(self) -> None:
        """Zero the stick, send a zero sample at once, then DriveCommand DISABLE."""
        self.stick.centre()
        self.stick.hold = False
        try:
            self.publish_stick((0.0, 0.0, 0.0, 0))
        except OSError as exc:
            rtps_host.log(f"[estop] zero sample failed: {exc}")
        self.send_drive(spec.ENUM_VALUES["DriveRequest"]["DISABLE"])
        rtps_host.log("[estop] E-STOP: zero XYTwist + DISABLE sent")

    # ------------------------------------------------------------- state

    def system_state(self) -> Optional[int]:
        with self.lock:
            return None if self.status is None else int(self.status["systemState"])

    def link_age(self, now: Optional[float] = None) -> Optional[float]:
        """Seconds since the last MibStatus, or None if none ever arrived."""
        now = time.monotonic() if now is None else now
        with self.lock:
            return None if self.status is None else now - self.status_at

    def mib_enabled(self) -> bool:
        age = self.link_age()
        return (age is not None and age <= spec.MIB_STATUS_TIMEOUT_MS / 1000.0
                and self.system_state() == spec.ENUM_VALUES["MibSystemState"].get("ENABLED"))

    def streaming_wanted(self) -> bool:
        return self.always_stream or self.mib_enabled()

    def snapshot(self) -> Dict[str, Any]:
        """Everything the GUI draws, copied out under the lock in one go."""
        now = time.monotonic()
        with self.lock:
            return {
                "now": now,
                "status": self.status, "status_at": self.status_at,
                "status_count": self.status_count, "status_gaps": self.status_gaps,
                "status_rate": self.status_meter.rate(now, spec.MIB_STATUS_TIMEOUT_MS / 1000.0),
                "diag": self.diagnostics, "diag_at": self.diag_at, "diag_count": self.diag_count,
                "diag_gaps": self.diag_gaps,
                "diag_rate": self.diag_meter.rate(now, spec.DIAG_TIMEOUT_MS / 1000.0),
                "motors": {axis_id: (record.state, record.at, record.gaps,
                                     record.meter.rate(now, MOTOR_STALE_S))
                           for axis_id, record in self.motors.items()},
            }

    # ---------------------------------------------------------- receiving

    def handle_user_packet(self, packet: bytes, sender_ip: str, sender_port: int) -> None:
        """Decode MibStatus, Diagnostics and MotorState by the topic the sample is on."""
        now = time.monotonic()
        for guid_prefix, writer_id, payload, reader_id in rtps_host.parse_rtps_data_messages(
                packet):
            topic = self.topic_for_sample(guid_prefix, writer_id, reader_id)
            if topic == spec.TOPIC_MIB_STATUS:
                status = spec.unpack_mib_status(payload)
                if status is None:
                    self._bad_sample(topic, payload, sender_ip)
                    continue
                with self.lock:
                    if self._status_seq is not None:
                        self.status_gaps += (status["seq"] - self._status_seq - 1) & 0xFF
                    self._status_seq = status["seq"]
                    if self.status is None:
                        rtps_host.log(f"[status] first MibStatus from {sender_ip}: "
                                      f"{describe_status(status)}")
                    self.status = status
                    self.status_at = now
                    self.status_count += 1
                    self.status_meter.mark(now)
                self._event(("status", status, None))
            elif topic == spec.TOPIC_MCB_DIAGNOSTICS:
                diag = spec.decode("Diagnostics", payload)
                if diag is None:
                    self._bad_sample(topic, payload, sender_ip)
                    continue
                with self.lock:
                    if self._diag_seq is not None:
                        self.diag_gaps += (diag["seq"] - self._diag_seq - 1) & 0xFF
                    self._diag_seq = diag["seq"]
                    self.diagnostics = diag
                    self.diag_at = now
                    self.diag_count += 1
                    self.diag_meter.mark(now)
                self._event(("diag", diag, None))
            elif topic in self._axis_by_topic:
                axis = self._axis_by_topic[topic]
                state = spec.unpack_motor_state(payload)
                if state is None:
                    self._bad_sample(topic, payload, sender_ip)
                    continue
                with self.lock:
                    record = self.motors[axis.id]
                    if record.state is None:
                        rtps_host.log(f"[motor] first MotorState for {axis.label} "
                                      f"({topic}) from {sender_ip}")
                    record.take(state, now)
                self._event(("motor", state, axis))
            elif topic is None:
                self._unknown_sample(guid_prefix, writer_id, reader_id, payload, sender_ip,
                                     sender_port)

    def _event(self, event: tuple) -> None:
        if self.events is None:
            return
        try:
            self.events.put_nowait(event)
        except queue.Full:
            pass  # the printer is behind; the newest state wins, as on the wire

    _last_bad_log = 0.0

    def _bad_sample(self, topic: str, payload: bytes, sender_ip: str) -> None:
        now = time.monotonic()
        if now - self._last_bad_log > 2.0:
            self._last_bad_log = now
            rtps_host.log(f"[data] {len(payload)}-byte sample on '{topic}' from {sender_ip} "
                          f"does not decode (header {payload[:4].hex()}) - spec mismatch?")

    def _unknown_sample(self, guid_prefix: bytes, writer_id: bytes, reader_id: bytes,
                        payload: bytes, sender_ip: str, sender_port: int) -> None:
        now = time.monotonic()
        if now - self.last_unknown_writer_log > 2.0:
            self.last_unknown_writer_log = now
            rtps_host.log(
                f"[data] {len(payload)}-byte sample from undiscovered writer "
                f"{rtps_host.guid_to_string(guid_prefix + writer_id)} at {sender_ip}:{sender_port}"
                f" (reader {reader_id.hex()}); cannot route")

    # ------------------------------------------------------------ threads

    def run(self) -> None:
        self._stream_thread = threading.Thread(target=self._stream_loop, daemon=True,
                                               name="xy-stream")
        self._stream_thread.start()
        super().run()  # closes the sockets via close() when stopped

    def _stream_loop(self) -> None:
        """XYTwist at xy_hz on its own clock: the harness's select loop wakes at most every
        200 ms when the link is quiet, which is nowhere near 30 Hz."""
        period = 1.0 / self.xy_hz
        next_at = time.monotonic()
        while not self._stream_stop.is_set():
            wanted = self.streaming_wanted()
            try:
                if wanted:
                    self.publish_stick()
                elif self._was_streaming:
                    # The drive just went away (DISABLE took, link lost): leave the
                    # MIB holding a zero rather than the last deflection.
                    self.publish_stick((0.0, 0.0, 0.0, 0))
            except OSError:
                pass
            self._was_streaming = wanted
            next_at += period
            delay = next_at - time.monotonic()
            if delay <= 0:
                next_at = time.monotonic()
                continue
            self._stream_stop.wait(delay)

    def close(self) -> None:
        """Stop the stream, leave the MIB a zero sample, then close the sockets."""
        self._stream_stop.set()
        if self._stream_thread is not None and self._stream_thread is not threading.current_thread():
            self._stream_thread.join(timeout=1.0)
        try:
            self.publish_stick((0.0, 0.0, 0.0, 0))
        except OSError:
            pass
        super().close()


def build_harness_args(cli: argparse.Namespace, peer: Optional[str],
                       advertised: Optional[str]) -> argparse.Namespace:
    """The argparse.Namespace RtpsHostHarness expects, with the joystick's endpoints.

    Falls back to the route to the peer, not the route to the internet:
    guess_local_ipv4() names the adapter that reaches 8.8.8.8, which on a bench
    network is usually the wrong one."""
    advertised = (advertised or (rtps_net.source_address_for(peer) if peer else None)
                  or rtps_host.guess_local_ipv4())
    return argparse.Namespace(
        node_name=cli.node_name,
        domain_id=cli.domain_id,
        participant_id=cli.participant_id,
        bind_address=advertised,
        advertised_address=advertised,
        multicast_interface=cli.multicast_interface,
        multicast_group=cli.multicast_group,
        enclave="/",
        subscribe_topic=[spec.TOPIC_MIB_STATUS],
        subscribe_type_name=spec.TYPE_MIB_STATUS,
        publish_topic=spec.TOPIC_JOYSTICK_XY_TWIST,
        publish_value=0,  # unused: publish_now() is overridden
        publish_interval=0.0,  # the stream thread keeps its own clock
        echo_received=False,
        reliable=False,
        type_name=spec.TYPE_XY_TWIST,
        announce_period=1.0,
        duration=0.0,
        trace_packets=cli.trace_packets,
        peer=[peer] if peer else None,
        peer_participant_ids=cli.peer_participant_ids,
        quiet=False,
    )


# ------------------------------------------------------------- formatting


def format_seat(axis: spec.SeatAxis, units: Optional[float]) -> str:
    """'12.5 deg' for a wire value in whole units, '--' for none."""
    if units is None:
        return "--"
    return f"{axis.format(axis.to_raw(units))} {axis.unit}"


def format_clock(status: Dict[str, Any]) -> str:
    """The MIB's wall clock: epoch_s shifted by its own utc_offset_min (what the HMI
    shows), with the offset, or 'unknown' when the MIB has no time."""
    epoch = int(status.get("epoch_s", 0))
    if epoch == 0:
        return "unknown (epoch 0)"
    offset_min = int(status.get("utc_offset_min", 0))
    try:
        zone = datetime.timezone(datetime.timedelta(minutes=offset_min))
        when = datetime.datetime.fromtimestamp(epoch, tz=zone)
    except (OverflowError, OSError, ValueError):
        return f"epoch {epoch} (out of range)"
    sign = "+" if offset_min >= 0 else "-"
    return f"{when:%Y-%m-%d %H:%M:%S} (UTC{sign}{abs(offset_min) // 60:02d}:{abs(offset_min) % 60:02d})"


def describe_status(status: Dict[str, Any], gaps: int = 0) -> str:
    """One line per MibStatus, for the log and headless mode."""
    seat = ", ".join(f"{axis.short}={format_seat(axis, value)}"
                     for axis, value in zip(spec.SEAT_AXES, spec.seat_values(status)))
    speed = float(status["speed"])
    text = (f"state={spec.enum_name('MibSystemState', status['systemState'])} "
            f"profile={spec.enum_name('DriveProfile', status['activeProfile'])} "
            f"speed={speed:.2f}m/s({speed * spec.MPH_PER_MPS:.1f}mph) "
            f"seq={status['seq']} gaps={gaps} seat=[{seat}]")
    if status.get("status_text"):
        text += f" text='{status['status_text']}'"
    if status.get("error_message"):
        text += f" error='{status['error_message']}'"
    if status.get("error_footer"):
        text += f" footer='{status['error_footer']}'"
    return text


def format_diag_item(item: spec.DiagItem, values: List[int]) -> List[str]:
    """Up to three readings as '23.4 Temp [C]'; '' for an unused or missing one."""
    out = []
    for field, unit in enumerate(item.units):
        if not unit or field >= len(values):
            out.append("")
        else:
            out.append(f"{item.format(values[field], field)} {unit}")
    return out


def describe_diagnostics(diag: Dict[str, Any], gaps: int = 0) -> str:
    items = diag["items"]
    parts = []
    for item in spec.DIAGNOSTICS:
        values = items[item.id]["values"] if item.id < len(items) else []
        readings = [r for r in format_diag_item(item, values) if r]
        parts.append(f"{item.short}: " + (", ".join(readings) if readings else "--"))
    extra = f" (+{len(items) - len(spec.DIAGNOSTICS)} rows the table lacks)" \
        if len(items) > len(spec.DIAGNOSTICS) else ""
    return f"seq={diag['seq']} gaps={gaps} " + " | ".join(parts) + extra


def describe_motor(axis: spec.Axis, state: Dict[str, Any], gaps: int = 0) -> str:
    temps = "/".join(f"{t:.0f}" for t in state["temps"])
    velocity = float(state["velocity"])
    position = float(state["position"])
    return (f"{axis.segment} api={state['api_version']} "
            f"state={spec.enum_name('BoardState', state['state'])} "
            f"mode={spec.enum_name('ControlMode', state['mode'])} "
            f"fault={spec.enum_name('FaultCode', state['fault_code'])} "
            f"pos={position:.3f}rad({math.degrees(position):.1f}deg) "
            f"vel={velocity:.3f}rad/s({velocity * spec.RPM_PER_RAD_S:.1f}rpm) "
            f"torque={state['torque_est']:.3f}Nm iq={state['iq']:.2f}A vbus={state['vbus']:.1f}V "
            f"temps={temps}C cmd_age={state['cmd_age_ms']}ms uptime={state['uptime_ms'] / 1000:.0f}s "
            f"seq={state['seq']} gaps={gaps}")


def format_age(age: Optional[float]) -> str:
    return "--" if age is None else (f"{age:.1f}s" if age < 100 else f"{age:.0f}s")


# ------------------------------------------------------------------ GUI


class MibPanel:
    """The window, plus the harness thread it starts and stops."""

    def __init__(self, root: "tk.Tk", cli: argparse.Namespace) -> None:
        self.root = root
        self.cli = cli
        self.harness: Optional[MibBench] = None
        self.thread: Optional[threading.Thread] = None

        # The harness runs on its own thread and tkinter is not thread-safe, so
        # log lines cross over through a queue that _tick drains on the UI
        # thread. rtps_host.log calls this from the network thread.
        self.log_queue: "queue.Queue[str]" = queue.Queue()
        rtps_host.LOG_SINK = self.log_queue.put
        # Worker threads hand results back here; only _tick touches widgets.
        self.result_queue: "queue.Queue[tuple]" = queue.Queue()
        self.adapters: list = []
        self.config = rtps_net.load_config()

        # Virtual joystick bookkeeping (UI thread only).
        self._keys: Dict[str, float] = {}
        self._key_release_jobs: Dict[str, str] = {}
        self._dragging = False
        self._twist_dragging = False
        self._seat_controls_enabled: Optional[bool] = None

        root.title("RAMMP MIB bench")
        self._fix_treeview_tags()
        self._build_connection()
        self._build_status()
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill="both", expand=False, padx=8, pady=(4, 0))
        drive_tab = ttk.Frame(self.notebook)
        seat_tab = ttk.Frame(self.notebook)
        diag_tab = ttk.Frame(self.notebook)
        motor_tab = ttk.Frame(self.notebook)
        self.notebook.add(drive_tab, text="Drive")
        self.notebook.add(seat_tab, text="Actuators")
        self.notebook.add(diag_tab, text="Diagnostics")
        self.notebook.add(motor_tab, text="Motors")
        self._build_drive(drive_tab)
        self._build_actuators(seat_tab)
        self._build_diagnostics(diag_tab)
        self._build_motors(motor_tab)
        self._build_log()

        root.bind_all("<Escape>", lambda _event: self._estop())
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        root.after(TICK_MS, self._tick)
        root.after(JOY_TICK_MS, self._joy_tick)
        if self.peer_var.get().strip():
            # A remembered or given MIB: connect straight away, the common case.
            root.after(300, self._connect)

    # ------------------------------------------------------------ widgets

    def _fix_treeview_tags(self) -> None:
        """Tk 8.6.9 ignores Treeview tag colours unless the style map is rebuilt."""
        style = ttk.Style()

        def fixed(option: str):
            return [item for item in style.map("Treeview", query_opt=option)
                    if item[:2] != ("!disabled", "!selected")]

        try:
            style.map("Treeview", foreground=fixed("foreground"), background=fixed("background"))
        except tk.TclError:
            pass

    def _build_connection(self) -> None:
        frame = ttk.LabelFrame(self.root, text="Connection", padding=8)
        frame.pack(fill="x", padx=8, pady=(8, 4))

        ttk.Label(frame, text="MIB").grid(row=0, column=0, sticky="w")
        self.peer_var = tk.StringVar(
            value=(self.cli.peer[0] if self.cli.peer else self.config.get(CONFIG_PEER, "")))
        ttk.Entry(frame, textvariable=self.peer_var, width=18).grid(row=0, column=1, padx=(4, 4))
        self.detect_button = ttk.Button(frame, text="Detect", command=self._on_detect)
        self.detect_button.grid(row=0, column=2)
        self.scan_button = ttk.Button(frame, text="Scan...", command=self._on_scan)
        self.scan_button.grid(row=0, column=3, padx=(4, 12))

        ttk.Label(frame, text="Via").grid(row=1, column=0, sticky="w", pady=(6, 0))
        self.address_var = tk.StringVar(
            value=self.cli.advertised_address or self.config.get(CONFIG_ADDRESS) or AUTO_ADAPTER)
        self.address_combo = ttk.Combobox(frame, textvariable=self.address_var, width=46)
        self.address_combo.grid(row=1, column=1, columnspan=3, sticky="w", padx=(4, 12),
                                pady=(6, 0))
        self._refresh_adapters()

        self.connect_button = ttk.Button(frame, text="Connect", command=self._toggle_connection)
        self.connect_button.grid(row=0, column=4)
        self.connection_label = ttk.Label(frame, text="not connected")
        self.connection_label.grid(row=1, column=4, columnspan=2, sticky="w", pady=(6, 0))
        ttk.Label(
            frame,
            text=f"domain {self.cli.domain_id}, participant {self.cli.participant_id}, "
                 f"node '{self.cli.node_name}'. Via defaults to Auto, which picks the adapter "
                 "that actually routes to the MIB - not the one that reaches the internet.",
            foreground="#666666", wraplength=640, justify="left",
        ).grid(row=2, column=0, columnspan=6, sticky="w", pady=(6, 0))

    def _build_status(self) -> None:
        frame = ttk.LabelFrame(self.root, text="MibStatus (from the MIB)", padding=8)
        frame.pack(fill="x", padx=8, pady=4)
        self.status_vars: Dict[str, "tk.StringVar"] = {}
        self.status_labels: Dict[str, "ttk.Label"] = {}
        cells = (
            ("Link", "link"), ("State", "state"), ("Profile", "profile"),
            ("Speed", "speed"), ("Seq / gaps", "seq"), ("Rate", "rate"),
            ("MIB clock", "clock"), ("status_text", "status_text"), ("error_message", "error"),
        )
        for index, (caption, key) in enumerate(cells):
            row, column = divmod(index, 3)
            ttk.Label(frame, text=caption, foreground="#666666").grid(
                row=row, column=column * 2, sticky="w", padx=(0, 4), pady=1)
            var = tk.StringVar(value="--")
            label = ttk.Label(frame, textvariable=var)
            label.grid(row=row, column=column * 2 + 1, sticky="w", padx=(0, 16), pady=1)
            self.status_vars[key] = var
            self.status_labels[key] = label
        self.status_vars["link"].set("not connected")
        ttk.Label(frame, text="error_footer", foreground="#666666").grid(
            row=3, column=0, sticky="w", padx=(0, 4))
        self.status_vars["footer"] = tk.StringVar(value="--")
        ttk.Label(frame, textvariable=self.status_vars["footer"]).grid(
            row=3, column=1, columnspan=5, sticky="w")
        ttk.Label(frame, text="seat", foreground="#666666").grid(
            row=4, column=0, sticky="nw", padx=(0, 4), pady=(4, 0))
        self.seat_summary_var = tk.StringVar(value="--")
        ttk.Label(frame, textvariable=self.seat_summary_var, wraplength=620,
                  justify="left").grid(row=4, column=1, columnspan=5, sticky="w", pady=(4, 0))

    def _build_drive(self, parent: "tk.Widget") -> None:
        left = ttk.Frame(parent, padding=8)
        left.pack(side="left", fill="y")
        right = ttk.Frame(parent, padding=8)
        right.pack(side="left", fill="both", expand=True)

        profile = ttk.LabelFrame(left, text="Profile (DriveCommand.profile)", padding=6)
        profile.pack(fill="x")
        self.profile_var = tk.IntVar(value=spec.ENUM_VALUES["DriveProfile"].get("NORMAL", 1))
        for value in sorted(spec.DRIVE_PROFILE_NAMES):
            ttk.Radiobutton(profile, text=spec.DRIVE_PROFILE_NAMES[value], value=value,
                            variable=self.profile_var, command=self._on_profile).pack(
                side="left", padx=(0, 8))

        drive = ttk.LabelFrame(left, text="Drive (DriveCommand.request)", padding=6)
        drive.pack(fill="x", pady=(6, 0))
        ttk.Button(drive, text="Enable", width=10, command=lambda: self._drive(
            spec.ENUM_VALUES["DriveRequest"]["ENABLE"])).grid(row=0, column=0, padx=(0, 6))
        ttk.Button(drive, text="Disable", width=10, command=lambda: self._drive(
            spec.ENUM_VALUES["DriveRequest"]["DISABLE"])).grid(row=0, column=1)
        self.drive_label = ttk.Label(drive, text="nothing sent yet", foreground="#666666")
        self.drive_label.grid(row=1, column=0, columnspan=2, sticky="w", pady=(4, 0))

        # A canvas rather than tk.Button: neither Aqua nor the Windows "vista"
        # theme honours a button's background, and E-STOP has to be red.
        self.estop = tk.Canvas(left, width=220, height=64, highlightthickness=0)
        self.estop.pack(pady=(10, 0))
        self.estop_rect = self.estop.create_rectangle(2, 2, 218, 62, fill="#c62828",
                                                      outline="#7f0000", width=2)
        self.estop.create_text(110, 32, text="E-STOP", fill="white",
                               font=("TkDefaultFont", 20, "bold"))
        self.estop.bind("<Button-1>", lambda _e: self.estop.itemconfigure(
            self.estop_rect, fill="#8e0000"))
        self.estop.bind("<ButtonRelease-1>", self._on_estop_release)
        ttk.Label(left, text="zero XYTwist, then DISABLE (also: Esc)",
                  foreground="#666666").pack()

        options = ttk.LabelFrame(left, text="Stream (XYTwist)", padding=6)
        options.pack(fill="x", pady=(10, 0))
        self.always_stream_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(options, text="always stream (not only while ENABLED)",
                        variable=self.always_stream_var, command=self._apply_options).pack(
            anchor="w")
        self.hold_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(options, text="hold (no spring return)", variable=self.hold_var,
                        command=self._apply_options).pack(anchor="w")
        self.button_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(options, text="JOYSTICK button pressed (buttons bit 0)",
                        variable=self.button_var, command=self._apply_options).pack(anchor="w")
        self.stream_label = ttk.Label(options, text=f"{self.cli.xy_hz:g} Hz: idle",
                                      foreground="#666666")
        self.stream_label.pack(anchor="w", pady=(4, 0))

        pad = ttk.LabelFrame(right, text="Virtual joystick (click the pad for keyboard: "
                                         "W/S A/D Q/E, space = centre)", padding=6)
        pad.pack(fill="both", expand=True)
        self.PAD = 240
        self.PAD_R = 100
        self.pad = tk.Canvas(pad, width=self.PAD, height=self.PAD, bg="#f4f4f4",
                             highlightthickness=2, highlightbackground="#bbbbbb",
                             highlightcolor="#1a6dd4", takefocus=1, cursor="crosshair")
        self.pad.grid(row=0, column=0, rowspan=4, padx=(0, 12))
        centre = self.PAD / 2
        self.pad.create_oval(centre - self.PAD_R, centre - self.PAD_R, centre + self.PAD_R,
                             centre + self.PAD_R, outline="#888888", width=2)
        self.pad.create_line(centre - self.PAD_R, centre, centre + self.PAD_R, centre,
                             fill="#cccccc")
        self.pad.create_line(centre, centre - self.PAD_R, centre, centre + self.PAD_R,
                             fill="#cccccc")
        self.pad.create_text(centre, 10, text="forward (+y)", fill="#888888",
                             font=("TkDefaultFont", 9))
        self.pad.create_text(self.PAD - 4, centre - 10, text="+x", fill="#888888", anchor="e",
                             font=("TkDefaultFont", 9))
        self.twist_marker = self.pad.create_line(centre, centre, centre, centre - self.PAD_R,
                                                 fill="#e0a000", width=3)
        self.knob = self.pad.create_oval(centre - 12, centre - 12, centre + 12, centre + 12,
                                         fill="#1a6dd4", outline="#0d4a96", width=2)
        self.pad.bind("<Button-1>", self._on_pad_press)
        self.pad.bind("<B1-Motion>", self._on_pad_motion)
        self.pad.bind("<ButtonRelease-1>", self._on_pad_release)
        self.pad.bind("<KeyPress>", self._on_key_press)
        self.pad.bind("<KeyRelease>", self._on_key_release)
        self.pad.bind("<FocusOut>", lambda _e: self._clear_keys())

        ttk.Label(pad, text="twist (- anticlockwise, + clockwise)").grid(
            row=0, column=1, sticky="sw")
        self.twist_scale = ttk.Scale(pad, from_=-1.0, to=1.0, orient="horizontal", length=200)
        self.twist_scale.grid(row=1, column=1, sticky="w")
        self.twist_scale.bind("<Button-1>", self._on_twist_press)
        self.twist_scale.bind("<B1-Motion>", self._on_twist_motion)
        self.twist_scale.bind("<ButtonRelease-1>", self._on_twist_release)
        self.live_var = tk.StringVar(value="x +0.00  y +0.00  twist +0.00")
        ttk.Label(pad, textvariable=self.live_var, font=("TkFixedFont", 11)).grid(
            row=2, column=1, sticky="nw", pady=(8, 0))
        ttk.Button(pad, text="Centre", command=self._centre_stick).grid(
            row=3, column=1, sticky="nw", pady=(6, 0))

    def _build_actuators(self, parent: "tk.Widget") -> None:
        frame = ttk.LabelFrame(parent, text="Seat axes (SeatCommand -> the MIB; value from "
                                            "MibStatus.currentSeatState)", padding=8)
        frame.pack(fill="x", padx=8, pady=4)
        for column, heading in enumerate(("", "Axis", "Value", "", "", "", "Target", "", "",
                                          "Range")):
            ttk.Label(frame, text=heading, foreground="#666666").grid(
                row=0, column=column, sticky="w", padx=4, pady=(0, 4))
        self.seat_value_vars: List["tk.StringVar"] = []
        self.seat_target_vars: List["tk.StringVar"] = []
        self.seat_controls: List["tk.Widget"] = []
        for index, axis in enumerate(spec.SEAT_AXES):
            row = index + 1
            ttk.Label(frame, text=axis.short, width=4).grid(row=row, column=0, sticky="w", padx=4)
            ttk.Label(frame, text=axis.label, width=12).grid(row=row, column=1, sticky="w", padx=4)
            value_var = tk.StringVar(value="--")
            ttk.Label(frame, textvariable=value_var, width=12, anchor="e").grid(
                row=row, column=2, sticky="e", padx=4)
            self.seat_value_vars.append(value_var)
            minus = ttk.Button(frame, text="-", width=3,
                               command=lambda i=index: self._seat_step(i, -1))
            minus.grid(row=row, column=3, padx=2, pady=1)
            plus = ttk.Button(frame, text="+", width=3,
                              command=lambda i=index: self._seat_step(i, +1))
            plus.grid(row=row, column=4, padx=2, pady=1)
            ttk.Label(frame, text=f"step {axis.format(axis.step)}", foreground="#666666").grid(
                row=row, column=5, sticky="w", padx=(2, 8))
            target_var = tk.StringVar(value="")
            entry = ttk.Entry(frame, textvariable=target_var, width=8, justify="right")
            entry.grid(row=row, column=6, padx=2)
            entry.bind("<Return>", lambda _event, i=index: self._seat_set(i))
            self.seat_target_vars.append(target_var)
            set_button = ttk.Button(frame, text="Set", width=4,
                                    command=lambda i=index: self._seat_set(i))
            set_button.grid(row=row, column=7, padx=2)
            controls = [minus, plus, entry, set_button]
            if axis.min_value <= 0 <= axis.max_value:
                zero = ttk.Button(frame, text="Zero", width=5,
                                  command=lambda i=index: self._seat_zero(i))
                zero.grid(row=row, column=8, padx=2)
                controls.append(zero)
            ttk.Label(frame, text=f"{axis.format(axis.min_value)} .. "
                                  f"{axis.format(axis.max_value)} {axis.unit}",
                      foreground="#666666").grid(row=row, column=9, sticky="w", padx=(8, 4))
            self.seat_controls.extend(controls)

        footer = ttk.Frame(frame)
        footer.grid(row=len(spec.SEAT_AXES) + 1, column=0, columnspan=10, sticky="w",
                    pady=(8, 0))
        self.force_seat_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(footer, text="force (send even when the MIB is not IDLE)",
                        variable=self.force_seat_var).pack(side="left")
        self.seat_note = ttk.Label(footer, text="", foreground="#666666")
        self.seat_note.pack(side="left", padx=(16, 0))
        ttk.Label(
            frame,
            text="Targets are absolute, in whole units, like the HMI's seat_step: - / + send "
                 "one step from the value the MIB last reported (min if unknown), clamped to the "
                 "range. The MIB refuses seat commands unless its state is IDLE, so the "
                 "controls grey out otherwise.",
            foreground="#666666", wraplength=640, justify="left",
        ).grid(row=len(spec.SEAT_AXES) + 2, column=0, columnspan=10, sticky="w", pady=(6, 0))

    def _build_diagnostics(self, parent: "tk.Widget") -> None:
        frame = ttk.LabelFrame(parent, text="Diagnostics (from the MIB, one row per "
                                            "RAMMP_DIAG_TABLE item)", padding=8)
        frame.pack(fill="both", expand=True, padx=8, pady=4)
        columns = ("short", "label", "r1", "r2", "r3", "age", "rate")
        self.diag_tree = ttk.Treeview(frame, columns=columns, show="headings",
                                      height=max(4, len(spec.DIAGNOSTICS)))
        widths = {"short": 50, "label": 130, "r1": 130, "r2": 130, "r3": 130, "age": 60,
                  "rate": 70}
        for column in columns:
            self.diag_tree.heading(column, text=column)
            self.diag_tree.column(column, width=widths[column], anchor="w", stretch=False)
        for item in spec.DIAGNOSTICS:
            self.diag_tree.insert("", "end", iid=str(item.id),
                                  values=(item.short, item.label, "", "", "", "--", "--"),
                                  tags=("unseen",))
        self.diag_tree.tag_configure("unseen", foreground="#999999")
        self.diag_tree.tag_configure("stale", foreground="#c62828")
        self.diag_tree.tag_configure("live", foreground="#000000")
        self.diag_tree.pack(fill="both", expand=True)
        self.diag_summary = ttk.Label(frame, text="no Diagnostics yet", foreground="#666666")
        self.diag_summary.pack(anchor="w", pady=(6, 0))

    def _build_motors(self, parent: "tk.Widget") -> None:
        frame = ttk.LabelFrame(parent, text="MotorState (one topic per RAMMP_AXIS_TABLE axis, "
                                            "all subscribed)", padding=8)
        frame.pack(fill="both", expand=True, padx=8, pady=4)
        columns = ("label", "api", "state", "mode", "fault", "pos", "vel", "torque", "iq",
                   "vbus", "temps", "cmd_age", "uptime", "age")
        headings = {"label": "axis", "api": "api", "state": "state", "mode": "mode",
                    "fault": "fault", "pos": "position rad (deg)", "vel": "velocity rad/s (rpm)",
                    "torque": "torque Nm", "iq": "iq A", "vbus": "vbus V", "temps": "temps C",
                    "cmd_age": "cmd age", "uptime": "uptime", "age": "age"}
        widths = {"label": 110, "api": 36, "state": 95, "mode": 75, "fault": 95, "pos": 130,
                  "vel": 130, "torque": 70, "iq": 60, "vbus": 60, "temps": 110, "cmd_age": 60,
                  "uptime": 65, "age": 55}
        container = ttk.Frame(frame)
        container.pack(fill="both", expand=True)
        self.motor_tree = ttk.Treeview(container, columns=columns, show="headings",
                                       height=max(4, len(spec.AXES)))
        for column in columns:
            self.motor_tree.heading(column, text=headings[column])
            self.motor_tree.column(column, width=widths[column], anchor="w", stretch=False)
        for axis in spec.AXES:
            self.motor_tree.insert("", "end", iid=str(axis.id),
                                   values=(axis.label,) + ("--",) * (len(columns) - 1),
                                   tags=("unseen",))
        self.motor_tree.tag_configure("unseen", foreground="#999999")
        self.motor_tree.tag_configure("stale", foreground="#c62828")
        self.motor_tree.tag_configure("fault", foreground="#b06a00")
        self.motor_tree.tag_configure("live", foreground="#000000")
        scroll = ttk.Scrollbar(container, orient="horizontal", command=self.motor_tree.xview)
        self.motor_tree.configure(xscrollcommand=scroll.set)
        self.motor_tree.pack(fill="both", expand=True)
        scroll.pack(fill="x")
        ttk.Label(frame, text="Rows never seen stay grey; a row with nothing for "
                              f"{MOTOR_STALE_S:g} s turns red; FAULT is orange.",
                  foreground="#666666").pack(anchor="w", pady=(6, 0))

    def _build_log(self) -> None:
        frame = ttk.LabelFrame(self.root, text="Log", padding=4)
        frame.pack(fill="both", expand=True, padx=8, pady=(4, 8))
        bar = ttk.Frame(frame)
        bar.pack(fill="x")
        self.verbose_var = tk.BooleanVar(value=bool(self.cli.verbose))
        ttk.Checkbutton(bar, text="verbose (discovery chatter)", variable=self.verbose_var).pack(
            side="left")
        ttk.Button(bar, text="Clear", command=self._clear_log).pack(side="right")
        body = ttk.Frame(frame)
        body.pack(fill="both", expand=True)
        self.log_text = tk.Text(body, height=10, width=100, wrap="none", state="disabled")
        self.log_text.pack(side="left", fill="both", expand=True)
        scroll = ttk.Scrollbar(body, command=self.log_text.yview)
        scroll.pack(side="right", fill="y")
        self.log_text.configure(yscrollcommand=scroll.set)

    # ------------------------------------------------------------ connection

    def _refresh_adapters(self) -> None:
        """Populate the Via dropdown with named adapters, Auto first."""
        self.adapters = rtps_net.list_adapters()
        values = [AUTO_ADAPTER]
        values += [a.label() for a in self.adapters if a.is_up]
        values += [a.label() for a in self.adapters if not a.is_up]
        self.address_combo["values"] = values

    def _selected_address(self) -> Optional[str]:
        """The advertised address to use, or None for Auto."""
        chosen = self.address_var.get().strip()
        if not chosen or chosen == AUTO_ADAPTER:
            return None
        for adapter in self.adapters:
            if chosen == adapter.label():
                return adapter.ip
        return chosen  # a hand-typed address

    def _on_detect(self) -> None:
        """Find the MIB on a worker thread; never blocks the window."""
        if self.harness is not None:
            self._append_log("[gui] disconnect before detecting")
            return
        saved = self.peer_var.get().strip() or self.config.get(CONFIG_PEER)
        self.detect_button.configure(state="disabled")
        self.connection_label.configure(text="looking for the MIB...")

        def work() -> None:
            found = rtps_net.discover_board(saved, progress=self.log_queue.put)
            self.result_queue.put(("detect", found))

        threading.Thread(target=work, daemon=True).start()

    def _on_scan(self) -> None:
        """Sweep a subnet, after showing which one."""
        if self.harness is not None:
            self._append_log("[gui] disconnect before scanning")
            return
        default = rtps_net.subnet_guess(self.peer_var.get().strip()
                                        or self.config.get(CONFIG_PEER))
        subnet = simpledialog.askstring("Scan for the MIB",
                                        "Subnet to sweep with RTPS discovery probes:",
                                        initialvalue=default, parent=self.root)
        if not subnet:
            return
        self.scan_button.configure(state="disabled")
        self.connection_label.configure(text=f"scanning {subnet}...")

        def work() -> None:
            try:
                found = rtps_net.scan_subnet(subnet, progress=self.log_queue.put)
            except ValueError as exc:
                self.log_queue.put(f"[gui] {subnet} is not a valid subnet: {exc}")
                found = []
            self.result_queue.put(("scan", found[0] if found else None))

        threading.Thread(target=work, daemon=True).start()

    def _discovery_finished(self, kind: str, found: Optional[str]) -> None:
        self.detect_button.configure(state="normal")
        self.scan_button.configure(state="normal")
        if found:
            self.peer_var.set(found)
            adapter = rtps_net.adapter_for(found)
            via = adapter.label() if adapter else "unknown"
            self.connection_label.configure(text=f"found {found} via {via}")
            self._append_log(f"[gui] MIB at {found}, reachable via {via}")
            return
        hint = "try Scan" if kind == "detect" else "check the subnet, and that the MIB is on"
        self.connection_label.configure(text=f"MIB not found - {hint}")

    def _toggle_connection(self) -> None:
        if self.harness is None:
            self._connect()
        else:
            self._disconnect()

    def _connect(self) -> None:
        if self.harness is not None:
            return
        peer = self.peer_var.get().strip() or None
        args = build_harness_args(self.cli, peer, self._selected_address())
        try:
            self.harness = MibBench(args, self.cli.xy_hz)
        except OSError as exc:
            # a bad address or a port already taken by another script
            self._append_log(f"[gui] connect failed: {exc}")
            self.connection_label.configure(text="connect failed")
            self.harness = None
            return
        self._apply_options()
        self.harness.profile = self.profile_var.get()
        self.thread = threading.Thread(target=self.harness.run, daemon=True)
        self.thread.start()
        self.connect_button.configure(text="Disconnect")
        self.connection_label.configure(
            text=f"participant up as {args.advertised_address}" + (f" -> {peer}" if peer else ""))
        self.status_vars["link"].set("waiting for MibStatus")
        rtps_net.save_config({CONFIG_PEER: peer, CONFIG_ADDRESS: args.advertised_address})

    def _disconnect(self) -> None:
        if self.harness is not None:
            self.harness.stop()  # run() returns, sends a zero sample, closes the sockets
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        self.harness = None
        self.thread = None
        self._clear_keys()
        self.connect_button.configure(text="Connect")
        self.connection_label.configure(text="not connected")
        self.status_vars["link"].set("not connected")
        self.stream_label.configure(text=f"{self.cli.xy_hz:g} Hz: idle")

    # ------------------------------------------------------------- drive

    def _apply_options(self) -> None:
        if self.harness is None:
            return
        self.harness.always_stream = self.always_stream_var.get()
        self.harness.stick.hold = self.hold_var.get()
        self.harness.stick.button = self.button_var.get()
        if not self.harness.stick.hold and not self._dragging and not self._keys:
            self.harness.stick.set_pad(0.0, 0.0)

    def _on_profile(self) -> None:
        """The HMI sends DriveCommand on change; carry the current request with it."""
        if self.harness is None:
            return
        self.harness.profile = self.profile_var.get()
        request = (spec.ENUM_VALUES["DriveRequest"]["ENABLE"] if self.harness.mib_enabled()
                   else spec.ENUM_VALUES["DriveRequest"]["DISABLE"])
        self.harness.send_drive(request)
        self._show_drive_request()

    def _drive(self, request: int) -> None:
        if self.harness is None:
            self._append_log("[gui] not connected")
            return
        self.harness.send_drive(request, self.profile_var.get())
        self._show_drive_request()

    def _show_drive_request(self) -> None:
        if self.harness is None or self.harness.last_drive_request is None:
            return
        self.drive_label.configure(
            text=f"last sent: {spec.enum_name('DriveRequest', self.harness.last_drive_request)} "
                 f"{spec.enum_name('DriveProfile', self.harness.profile)}")

    def _on_estop_release(self, _event: Any) -> None:
        self.estop.itemconfigure(self.estop_rect, fill="#c62828")
        self._estop()

    def _estop(self) -> None:
        self._clear_keys()
        self.hold_var.set(False)
        self.twist_scale.set(0.0)
        if self.harness is None:
            self._append_log("[gui] E-STOP: not connected, nothing to send")
            return
        self.harness.estop()
        self._show_drive_request()

    # ---------------------------------------------------------- joystick

    def _stick(self) -> Optional[Stick]:
        return None if self.harness is None else self.harness.stick

    def _centre_stick(self) -> None:
        self._clear_keys()
        self.twist_scale.set(0.0)
        stick = self._stick()
        if stick is not None:
            stick.centre()

    def _pad_to_axes(self, px: float, py: float) -> Tuple[float, float]:
        centre = self.PAD / 2
        return (px - centre) / self.PAD_R, (centre - py) / self.PAD_R

    def _on_pad_press(self, event: Any) -> None:
        self.pad.focus_set()
        self._dragging = True
        self._on_pad_motion(event)

    def _on_pad_motion(self, event: Any) -> None:
        stick = self._stick()
        if stick is None or not self._dragging:
            return
        stick.set_pad(*self._pad_to_axes(event.x, event.y))

    def _on_pad_release(self, _event: Any) -> None:
        self._dragging = False
        stick = self._stick()
        if stick is not None and not stick.hold:
            stick.set_pad(0.0, 0.0)

    def _on_twist_press(self, _event: Any) -> None:
        self._twist_dragging = True

    def _on_twist_motion(self, _event: Any) -> None:
        stick = self._stick()
        if stick is not None and self._twist_dragging:
            stick.set_twist(float(self.twist_scale.get()))

    def _on_twist_release(self, _event: Any) -> None:
        # The click lands before ttk moves the slider; read it after the event.
        self.root.after(1, self._twist_released)

    def _twist_released(self) -> None:
        self._twist_dragging = False
        stick = self._stick()
        if stick is None:
            return
        stick.set_twist(float(self.twist_scale.get()))
        if not stick.hold:
            stick.set_twist(0.0)
            self.twist_scale.set(0.0)

    KEY_AXES = {"w": ("y", +1), "s": ("y", -1), "d": ("x", +1), "a": ("x", -1),
                "e": ("twist", +1), "q": ("twist", -1)}

    def _on_key_press(self, event: Any) -> None:
        key = (event.keysym or "").lower()
        if key == "space":
            self._centre_stick()
            return
        if key not in self.KEY_AXES:
            return
        job = self._key_release_jobs.pop(key, None)
        if job is not None:
            self.root.after_cancel(job)
        self._keys[key] = time.monotonic()

    def _on_key_release(self, event: Any) -> None:
        key = (event.keysym or "").lower()
        if key not in self.KEY_AXES:
            return
        # X11 autorepeat delivers release/press pairs a few ms apart; drop the key
        # only if no press follows at once.
        job = self._key_release_jobs.pop(key, None)
        if job is not None:
            self.root.after_cancel(job)
        self._key_release_jobs[key] = self.root.after(60, self._drop_key, key)

    def _drop_key(self, key: str) -> None:
        self._key_release_jobs.pop(key, None)
        self._keys.pop(key, None)

    def _clear_keys(self) -> None:
        for job in self._key_release_jobs.values():
            self.root.after_cancel(job)
        self._key_release_jobs.clear()
        self._keys.clear()

    def _joy_tick(self) -> None:
        """Integrate the held keys, spring released axes back, and redraw the pad."""
        now = time.monotonic()
        for key, pressed_at in list(self._keys.items()):
            if now - pressed_at > KEY_STUCK_S:
                self._keys.pop(key, None)  # no repeat, no release: treat as let go
        stick = self._stick()
        if stick is not None:
            targets = {"x": 0, "y": 0, "twist": 0}
            for key in self._keys:
                axis, direction = self.KEY_AXES[key]
                targets[axis] += direction
            x, y, twist = stick.x, stick.y, stick.twist
            if not self._dragging:
                x = self._approach(x, targets["x"], stick.hold)
                y = self._approach(y, targets["y"], stick.hold)
                stick.set_pad(x, y)
            if not self._twist_dragging:
                twist = self._approach(twist, targets["twist"], stick.hold)
                stick.set_twist(twist)
                if abs(float(self.twist_scale.get()) - stick.twist) > 1e-3:
                    self.twist_scale.set(stick.twist)
            x, y, twist = stick.x, stick.y, stick.twist
        else:
            x = y = twist = 0.0
        self._draw_stick(x, y, twist)
        self.root.after(JOY_TICK_MS, self._joy_tick)

    @staticmethod
    def _approach(value: float, target: int, hold: bool) -> float:
        dt = JOY_TICK_MS / 1000.0
        if target:
            step = KEY_RATE * dt
            return max(-1.0, min(1.0, value + target * step))
        if hold:
            return value
        step = SPRING_RATE * dt
        if abs(value) <= step:
            return 0.0
        return value - math.copysign(step, value)

    def _draw_stick(self, x: float, y: float, twist: float) -> None:
        centre = self.PAD / 2
        kx, ky = centre + x * self.PAD_R, centre - y * self.PAD_R
        self.pad.coords(self.knob, kx - 12, ky - 12, kx + 12, ky + 12)
        angle = math.radians(90.0 - twist * 90.0)  # + twist = clockwise from the top
        self.pad.coords(self.twist_marker, centre, centre,
                        centre + self.PAD_R * math.cos(angle),
                        centre - self.PAD_R * math.sin(angle))
        self.live_var.set(f"x {x:+.2f}  y {y:+.2f}  twist {twist:+.2f}")

    # ---------------------------------------------------------- actuators

    def _seat_allowed(self) -> bool:
        if self.harness is None:
            return False
        if self.force_seat_var.get():
            return True
        age = self.harness.link_age()
        return (age is not None and age <= spec.MIB_STATUS_TIMEOUT_MS / 1000.0
                and self.harness.system_state() == spec.ENUM_VALUES["MibSystemState"].get("IDLE"))

    def _seat_step(self, index: int, direction: int) -> None:
        if not self._seat_allowed():
            self._append_log("[gui] seat command blocked: the MIB is not IDLE (tick force)")
            return
        self.harness.seat_step(index, direction)

    def _seat_set(self, index: int) -> None:
        if not self._seat_allowed():
            self._append_log("[gui] seat command blocked: the MIB is not IDLE (tick force)")
            return
        axis = spec.SEAT_AXES[index]
        try:
            raw = axis.parse(self.seat_target_vars[index].get())
        except ValueError:
            self._append_log(f"[gui] '{self.seat_target_vars[index].get()}' is not a number")
            return
        self.harness.send_seat(index, axis.to_units(raw))
        self.root.focus_set()

    def _seat_zero(self, index: int) -> None:
        if not self._seat_allowed():
            self._append_log("[gui] seat command blocked: the MIB is not IDLE (tick force)")
            return
        self.harness.send_seat(index, 0.0)

    # ----------------------------------------------------------------- tick

    def _append_log(self, message: str) -> None:
        self.log_text.configure(state="normal")
        self.log_text.insert("end", message + "\n")
        line_count = int(self.log_text.index("end-1c").split(".")[0])
        if line_count > LOG_MAX_LINES:
            self.log_text.delete("1.0", f"{line_count - LOG_MAX_LINES}.0")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _clear_log(self) -> None:
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    def _tick(self) -> None:
        verbose = self.verbose_var.get()
        while True:
            try:
                line = self.log_queue.get_nowait()
            except queue.Empty:
                break
            if verbose or not line.startswith(QUIET_PREFIXES):
                self._append_log(line)
        while True:
            try:
                kind, payload = self.result_queue.get_nowait()
            except queue.Empty:
                break
            if kind in ("detect", "scan"):
                self._discovery_finished(kind, payload)
        if self.harness is not None:
            snapshot = self.harness.snapshot()
            self._refresh_status(snapshot)
            self._refresh_seat(snapshot)
            self._refresh_diagnostics(snapshot)
            self._refresh_motors(snapshot)
            self._refresh_stream()
        self.root.after(TICK_MS, self._tick)

    def _refresh_status(self, snap: Dict[str, Any]) -> None:
        status = snap["status"]
        if status is None:
            self.status_vars["link"].set("waiting for MibStatus")
            self.status_labels["link"].configure(foreground="#b06a00")
            return
        age = snap["now"] - snap["status_at"]
        lost = age > spec.MIB_STATUS_TIMEOUT_MS / 1000.0
        self.status_vars["link"].set(f"LOST ({age:.0f}s silent)" if lost
                                     else f"connected ({age * 1000:.0f} ms ago)")
        self.status_labels["link"].configure(foreground="#c62828" if lost else "#1e8a2e")
        state = spec.enum_name("MibSystemState", status["systemState"])
        self.status_vars["state"].set(f"{state} ({status['systemState']})")
        self.status_labels["state"].configure(
            foreground="#999999" if lost else STATE_COLOURS.get(state, "#6a1b9a"))
        self.status_vars["profile"].set(spec.enum_name("DriveProfile", status["activeProfile"]))
        speed = float(status["speed"])
        self.status_vars["speed"].set(f"{speed:.2f} m/s  ({speed * spec.MPH_PER_MPS:.1f} mph)")
        self.status_vars["seq"].set(f"{status['seq']} / {snap['status_gaps']} gaps "
                                    f"({snap['status_count']} rx)")
        self.status_vars["rate"].set(f"{snap['status_rate']:.1f} Hz")
        self.status_vars["clock"].set(format_clock(status))
        self.status_vars["status_text"].set(status.get("status_text") or "(empty)")
        self.status_vars["error"].set(status.get("error_message") or "(empty)")
        self.status_labels["error"].configure(
            foreground="#c62828" if state == "ERROR" else "#000000")
        self.status_vars["footer"].set(status.get("error_footer") or "(empty)")
        values = spec.seat_values(status)
        self.seat_summary_var.set("   ".join(
            f"{axis.label}: {format_seat(axis, values[i] if i < len(values) else None)}"
            for i, axis in enumerate(spec.SEAT_AXES)))

    def _refresh_seat(self, snap: Dict[str, Any]) -> None:
        status = snap["status"]
        values = spec.seat_values(status) if status is not None else []
        for index, axis in enumerate(spec.SEAT_AXES):
            text = format_seat(axis, values[index] if index < len(values) else None)
            if self.seat_value_vars[index].get() != text:
                self.seat_value_vars[index].set(text)
        allowed = self._seat_allowed()
        if allowed != self._seat_controls_enabled:
            self._seat_controls_enabled = allowed
            for widget in self.seat_controls:
                widget.configure(state="normal" if allowed else "disabled")
            self.seat_note.configure(
                text="" if allowed else "controls greyed: the MIB is not IDLE (or no status)")

    def _refresh_diagnostics(self, snap: Dict[str, Any]) -> None:
        diag = snap["diag"]
        if diag is None:
            return
        age = snap["now"] - snap["diag_at"]
        stale = age > spec.DIAG_TIMEOUT_MS / 1000.0
        tag = "stale" if stale else "live"
        items = diag["items"]
        for item in spec.DIAGNOSTICS:
            values = items[item.id]["values"] if item.id < len(items) else []
            readings = format_diag_item(item, values)
            self.diag_tree.item(str(item.id), values=(
                item.short, item.label, *readings, format_age(age), f"{snap['diag_rate']:.1f} Hz"),
                tags=(tag,))
        extra = (f"; {len(items)} rows on the wire, table has {len(spec.DIAGNOSTICS)}"
                 if len(items) != len(spec.DIAGNOSTICS) else "")
        self.diag_summary.configure(
            text=f"seq {diag['seq']}, {snap['diag_gaps']} gaps, {snap['diag_count']} rx, "
                 f"{snap['diag_rate']:.1f} Hz" + (" - STALE" if stale else "") + extra,
            foreground="#c62828" if stale else "#666666")

    def _refresh_motors(self, snap: Dict[str, Any]) -> None:
        for axis in spec.AXES:
            state, at, _gaps, rate = snap["motors"][axis.id]
            if state is None:
                continue
            age = snap["now"] - at
            stale = age > MOTOR_STALE_S
            board = spec.enum_name("BoardState", state["state"])
            tag = "stale" if stale else ("fault" if board == "FAULT" else "live")
            position = float(state["position"])
            velocity = float(state["velocity"])
            self.motor_tree.item(str(axis.id), values=(
                axis.label, state["api_version"], board,
                spec.enum_name("ControlMode", state["mode"]),
                spec.enum_name("FaultCode", state["fault_code"]),
                f"{position:.3f} ({math.degrees(position):.1f})",
                f"{velocity:.3f} ({velocity * spec.RPM_PER_RAD_S:.1f})",
                f"{state['torque_est']:.3f}", f"{state['iq']:.2f}", f"{state['vbus']:.1f}",
                " ".join(f"{t:.0f}" for t in state["temps"]),
                f"{state['cmd_age_ms']} ms", f"{state['uptime_ms'] / 1000:.0f} s",
                f"{format_age(age)} {rate:.0f}Hz"), tags=(tag,))

    def _refresh_stream(self) -> None:
        harness = self.harness
        if harness is None:
            return
        if harness.streaming_wanted():
            why = "always" if harness.always_stream else "MIB ENABLED"
            text = (f"{harness.xy_hz:g} Hz: streaming ({why}), {harness.xy_sent} sent to "
                    f"{max(0, harness.xy_targets)} target(s)")
        else:
            text = f"{harness.xy_hz:g} Hz: idle (MIB not ENABLED), {harness.xy_sent} sent"
        self.stream_label.configure(text=text)
        self._show_drive_request()

    def _on_close(self) -> None:
        self._disconnect()
        rtps_host.LOG_SINK = None
        self.root.destroy()


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
          and first.parse("1e9") == first.max_value)

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
            check("seat_step clamps at max",
                  bench.seat_step(2, +1) == spec.SEAT_AXES[2].to_units(spec.SEAT_AXES[2].max_value))
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
    if not _import_tk():
        print("tkinter is not available in this Python; use --headless, or a Python built "
              "with Tk (e.g. `brew install python-tk`)", file=sys.stderr)
        return 2
    root = tk.Tk()
    MibPanel(root, cli)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
