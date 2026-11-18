"""RAMMP MIB bench model: the RTPS harness that plays the joystick / HMI side,
plus the formatting helpers the window and the headless mode share.

No UI in here: `rtps_mib_gui.py` is the entry point (window / --headless /
--self-test) and `mib_window_qt.py` the Qt window.
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

    #: a seat step is computed from the MIB's reported value; older than this
    #: (two MibStatus periods) it is not a safe base, so the step is refused
    SEAT_STEP_MAX_AGE_S = 2 * spec.MIB_STATUS_PERIOD_MS / 1000.0

    def seat_step(self, axis_id: int, direction: int) -> Optional[float]:
        """One step from where the MIB last said the axis is, like the HMI's
        seat_step: clamped, absolute, whole units. Returns the target sent, or
        None when there is no fresh value to step from (the HMI starts from the
        table's minimum then; this tool refuses - that can be a large move)."""
        if not 0 <= axis_id < len(spec.SEAT_AXES):
            return None
        axis = spec.SEAT_AXES[axis_id]
        current = self.seat_raw(axis_id)
        age = self.link_age()
        if current is None or age is None or age > self.SEAT_STEP_MAX_AGE_S:
            rtps_host.log(f"[seat] {axis.short}: no fresh value to step from "
                          f"(last MibStatus {'never' if age is None else f'{age:.1f} s ago'})")
            return None
        # not clamped to the spec row: the row is the HMI's jog range, the
        # MIB's own per-corner limits (which this tool cannot read) decide;
        # the MIB clamps or refuses, this tool just must not step from stale data
        target_raw = current + direction * axis.step
        target = axis.to_units(target_raw)
        self.send_seat(axis_id, target)
        return target

    #: the largest jog the tool sends (the MIB refuses more than its own limit, 1 deg)
    SEAT_JOG_MAX_UNITS = 1.0

    def seat_jog(self, axis_id: int, delta_units: float) -> Optional[float]:
        """A small move relative to the MIB's fresh reported value, NOT clamped to
        the table's range: the recovery move for a linkage that sits outside
        its range (the MIB refuses every other command for it and itself
        accepts a jog only within its jog limit of the current position).
        Returns the target sent, or None if refused here."""
        if not 0 <= axis_id < len(spec.SEAT_AXES):
            return None
        axis = spec.SEAT_AXES[axis_id]
        if not abs(delta_units) <= self.SEAT_JOG_MAX_UNITS or delta_units == 0:
            rtps_host.log(f"[seat] {axis.short}: a jog is at most "
                          f"{self.SEAT_JOG_MAX_UNITS:g} {axis.unit}")
            return None
        current = self.seat_raw(axis_id)
        age = self.link_age()
        if current is None or age is None or age > self.SEAT_STEP_MAX_AGE_S:
            rtps_host.log(f"[seat] {axis.short}: no fresh value to jog from "
                          f"(last MibStatus {'never' if age is None else f'{age:.1f} s ago'})")
            return None
        target = axis.to_units(current) + delta_units
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
