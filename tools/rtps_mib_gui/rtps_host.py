#!/usr/bin/env python3
# VENDORED: copied verbatim from pace-hmi-fw/scripts/rtps_host.py @ d8efcbc; keep in sync.
# Do not edit here - fix it in pace-hmi-fw and copy it back.
"""Simple host-side RTPS test harness for the ESPP RTPS component.

This script speaks the ESPP RTPS discovery wire format plus the standard
CDR-over-RTPS user-data path used by ``RtpsParticipant``. It is useful for:

1. discovering an embedded ESPP RTPS participant from a PC/host,
2. inspecting SPDP/SEDP announcements, and
3. sending or receiving ``std_msgs/msg/UInt32``-style test samples. User samples
   are standard RTPS ``DATA`` submessages whose serializedPayload is the raw
   CDR-encapsulated sample; the topic is identified by the writer GUID resolved
   through SEDP discovery (no ESPP-specific payload framing).

Run ``python rtps_host.py --self-test`` to validate the wire-format encoders and
decoders against the firmware's expectations without any network I/O.

It uses only the Python standard library, so it does not require Python
bindings or a rebuilt host ``lib/`` tree.
"""

from __future__ import annotations

import argparse
import hashlib
import ipaddress
import select
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Set, Tuple


RTPS_MAGIC = b"RTPS"
PL_CDR_LE = b"\x00\x03\x00\x00"

PORT_BASE = 7400
DOMAIN_GAIN = 250
PARTICIPANT_GAIN = 2
METATRAFFIC_MULTICAST_OFFSET = 0
METATRAFFIC_UNICAST_OFFSET = 10
USER_MULTICAST_OFFSET = 1
USER_UNICAST_OFFSET = 11

DATA_SUBMESSAGE_KIND = 0x15
DATA_SUBMESSAGE_FLAGS = 0x01 | 0x04
DATA_SUBMESSAGE_OCTETS_TO_INLINE_QOS = 16
HEARTBEAT_SUBMESSAGE_KIND = 0x07
ACKNACK_SUBMESSAGE_KIND = 0x06
ACKNACK_SUBMESSAGE_FLAGS = 0x03  # EndiannessFlag (LE) | FinalFlag

RTPS_QOS_RELIABILITY_BEST_EFFORT = 1
RTPS_QOS_RELIABILITY_RELIABLE = 2

KIND_UDP_V4 = 1
VENDOR_ID = b"\xca\xfe"

ENTITY_ID_UNKNOWN = b"\x00\x00\x00\x00"
PARTICIPANT_ENTITY_ID = b"\x00\x00\x01\xc1"
SPDP_WRITER_ENTITY_ID = b"\x00\x01\x00\xc2"
SPDP_READER_ENTITY_ID = b"\x00\x01\x00\xc7"
SEDP_PUBLICATIONS_WRITER_ENTITY_ID = b"\x00\x00\x03\xc2"
SEDP_PUBLICATIONS_READER_ENTITY_ID = b"\x00\x00\x03\xc7"
SEDP_SUBSCRIPTIONS_WRITER_ENTITY_ID = b"\x00\x00\x04\xc2"
SEDP_SUBSCRIPTIONS_READER_ENTITY_ID = b"\x00\x00\x04\xc7"
#: The 'no particular reader' entity id. A writer that matched us through
#: SEDP replaces it with our reader's own id, which topic_for_sample uses.
ENTITYID_UNKNOWN = bytes(4)
USER_WRITER_NO_KEY_KIND = 0x03
USER_READER_NO_KEY_KIND = 0x04

BUILTIN_ENDPOINT_SET = (
    (1 << 0)
    | (1 << 1)
    | (1 << 2)
    | (1 << 3)
    | (1 << 4)
    | (1 << 5)
    | (1 << 10)
    | (1 << 11)
)

PID_SENTINEL = 0x0001
PID_PARTICIPANT_LEASE_DURATION = 0x0002
PID_TOPIC_NAME = 0x0005
PID_TYPE_NAME = 0x0007
PID_DOMAIN_ID = 0x000F
PID_PROTOCOL_VERSION = 0x0015
PID_VENDORID = 0x0016
PID_RELIABILITY = 0x001A
PID_LIVELINESS = 0x001B
PID_DURABILITY = 0x001D
PID_USER_DATA = 0x002C
PID_MULTICAST_LOCATOR = 0x0030
PID_UNICAST_LOCATOR = 0x002F
PID_DEFAULT_UNICAST_LOCATOR = 0x0031
PID_METATRAFFIC_UNICAST_LOCATOR = 0x0032
PID_METATRAFFIC_MULTICAST_LOCATOR = 0x0033
PID_HISTORY = 0x0040
PID_EXPECTS_INLINE_QOS = 0x0043
PID_DEFAULT_MULTICAST_LOCATOR = 0x0048
PID_PARTICIPANT_GUID = 0x0050
PID_BUILTIN_ENDPOINT_SET = 0x0058
PID_ENDPOINT_GUID = 0x005A
PID_TYPE_MAX_SIZE_SERIALIZED = 0x0060
PID_ENTITY_NAME = 0x0062
PID_KEY_HASH = 0x0070

DEFAULT_LEASE_DURATION_SECONDS = 20
DEFAULT_LEASE_DURATION_NANOSECONDS = 0
DEFAULT_MAX_BLOCKING_SECONDS = 0
DEFAULT_MAX_BLOCKING_NANOSECONDS = 100_000_000
DEFAULT_TOPIC_PREFIX = "espp/rtps_example"
DEFAULT_REQUEST_TOPIC = f"{DEFAULT_TOPIC_PREFIX}/request"
DEFAULT_RESPONSE_TOPIC = f"{DEFAULT_TOPIC_PREFIX}/response"


@dataclass
class PortMapping:
    metatraffic_multicast: int
    metatraffic_unicast: int
    user_multicast: int
    user_unicast: int


@dataclass
class ParticipantProxy:
    participant_guid: bytes
    guid_prefix: bytes
    name: str
    enclave: str
    address: str
    ports: PortMapping
    builtin_endpoints: int


@dataclass
class EndpointProxy:
    guid: bytes
    participant_guid: bytes
    topic_name: str
    type_name: str
    reliability: str
    is_reader: bool
    expects_inline_qos: bool
    unicast_address: str
    unicast_port: int
    multicast_locators: List[Tuple[str, int]]


@dataclass
class WriterConfig:
    topic_name: str
    type_name: str
    reliable: bool
    entity_index: int


@dataclass
class ReaderConfig:
    topic_name: str
    type_name: str
    reliable: bool
    entity_index: int


#: Optional callable receiving every log line as well as stdout. Set by a GUI
#: that wants the harness output in a widget; called from the network thread, so
#: whatever is assigned here must be safe to call from there (a Queue.put is).
LOG_SINK = None


def log(message: str) -> None:
    if LOG_SINK is not None:
        try:
            LOG_SINK(message)
        except Exception:  # a broken sink must not take the network thread down
            pass
    print(message, flush=True)


def hex_string(value: bytes) -> str:
    return value.hex()


def guid_to_string(guid: bytes) -> str:
    return hex_string(guid[:12]) + ":" + hex_string(guid[12:])


def entity_id_for_index(entity_index: int, kind: int) -> bytes:
    return bytes((0x00, 0x00, 0x10 + entity_index, kind))


def reliability_to_name(reliable: bool) -> str:
    return "reliable" if reliable else "best-effort"


def ntp_fraction_from_nanoseconds(nanoseconds: int) -> int:
    # RTPS Duration_t/Time_t use NTP fraction units of 1/2^32 s, not nanoseconds.
    return (nanoseconds << 32) // 1_000_000_000


def padded_parameter_length(length: int) -> int:
    # RTPS PL_CDR requires each parameterLength to be a multiple of 4.
    return (length + 3) & ~3


def compute_port_mapping(domain_id: int, participant_id: int) -> PortMapping:
    base = PORT_BASE + DOMAIN_GAIN * domain_id
    participant_offset = PARTICIPANT_GAIN * participant_id
    return PortMapping(
        metatraffic_multicast=base + METATRAFFIC_MULTICAST_OFFSET,
        metatraffic_unicast=base + METATRAFFIC_UNICAST_OFFSET + participant_offset,
        user_multicast=base + USER_MULTICAST_OFFSET,
        user_unicast=base + USER_UNICAST_OFFSET + participant_offset,
    )


def parse_participant_id_range(text: str) -> List[int]:
    """Parse '0-3' or '0,1,2' into a list of participant ids."""
    ids: List[int] = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part[1:]:
            first, _, last = part.partition("-")
            ids.extend(range(int(first), int(last) + 1))
        else:
            ids.append(int(part))
    if not ids:
        raise ValueError(f"no participant ids in '{text}'")
    return ids


def guess_local_ipv4() -> str:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("8.8.8.8", 80))
        return probe.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        probe.close()


def make_guid_prefix(node_name: str, domain_id: int, participant_id: int) -> bytes:
    digest = hashlib.sha256(node_name.encode("utf-8")).digest()
    return bytes(
        (
            participant_id & 0xFF,
            (participant_id >> 8) & 0xFF,
            domain_id & 0xFF,
            (domain_id >> 8) & 0xFF,
        )
    ) + digest[:8]


def make_guid(prefix: bytes, entity_id: bytes) -> bytes:
    return prefix + entity_id


def align4(buffer: bytearray) -> None:
    while len(buffer) % 4 != 0:
        buffer.append(0)


def append_parameter_header(buffer: bytearray, pid: int, length: int) -> None:
    buffer.extend(struct.pack("<HH", pid, length))


def append_parameter_guid(buffer: bytearray, pid: int, guid: bytes) -> None:
    append_parameter_header(buffer, pid, 16)
    buffer.extend(guid)


def append_parameter_protocol_version(buffer: bytearray) -> None:
    append_parameter_header(buffer, PID_PROTOCOL_VERSION, 4)
    buffer.extend((2, 3, 0, 0))


def append_parameter_vendor_id(buffer: bytearray) -> None:
    append_parameter_header(buffer, PID_VENDORID, 4)
    buffer.extend(VENDOR_ID)
    buffer.extend((0, 0))


def append_parameter_u32(buffer: bytearray, pid: int, value: int) -> None:
    append_parameter_header(buffer, pid, 4)
    buffer.extend(struct.pack("<I", value))


def append_parameter_bool(buffer: bytearray, pid: int, value: bool) -> None:
    append_parameter_header(buffer, pid, 4)
    buffer.extend((1 if value else 0, 0, 0, 0))


def append_parameter_duration(buffer: bytearray, pid: int, seconds: int, nanoseconds: int) -> None:
    append_parameter_header(buffer, pid, 8)
    buffer.extend(struct.pack("<iI", seconds, ntp_fraction_from_nanoseconds(nanoseconds)))


def locator_bytes(ip_address: str, port: int) -> bytes:
    # Locator_t.kind/.port are little-endian in PL_CDR_LE; only the 16-byte address is raw bytes.
    locator = bytearray(24)
    struct.pack_into("<I", locator, 0, KIND_UDP_V4)
    struct.pack_into("<I", locator, 4, port)
    locator[20:24] = socket.inet_aton(ip_address)
    return bytes(locator)


def append_parameter_locator(buffer: bytearray, pid: int, ip_address: str, port: int) -> None:
    append_parameter_header(buffer, pid, 24)
    buffer.extend(locator_bytes(ip_address, port))


def append_parameter_string_cdr(buffer: bytearray, pid: int, text: str) -> None:
    encoded = text.encode("utf-8")
    # parameterLength must be a multiple of 4 and includes the trailing CDR padding.
    append_parameter_header(buffer, pid, padded_parameter_length(4 + len(encoded) + 1))
    buffer.extend(struct.pack("<I", len(encoded) + 1))
    buffer.extend(encoded)
    buffer.append(0)
    align4(buffer)


def append_parameter_octet_sequence(buffer: bytearray, pid: int, payload: bytes) -> None:
    append_parameter_header(buffer, pid, padded_parameter_length(4 + len(payload)))
    buffer.extend(struct.pack("<I", len(payload)))
    buffer.extend(payload)
    align4(buffer)


def append_parameter_reliability(buffer: bytearray, reliable: bool) -> None:
    append_parameter_header(buffer, PID_RELIABILITY, 12)
    kind = RTPS_QOS_RELIABILITY_RELIABLE if reliable else RTPS_QOS_RELIABILITY_BEST_EFFORT
    buffer.extend(struct.pack("<I", kind))
    buffer.extend(
        struct.pack(
            "<iI",
            DEFAULT_MAX_BLOCKING_SECONDS,
            ntp_fraction_from_nanoseconds(DEFAULT_MAX_BLOCKING_NANOSECONDS),
        )
    )


def append_parameter_durability(buffer: bytearray) -> None:
    append_parameter_header(buffer, PID_DURABILITY, 4)
    buffer.extend(struct.pack("<I", 0))


def append_parameter_liveliness(buffer: bytearray) -> None:
    append_parameter_header(buffer, PID_LIVELINESS, 12)
    buffer.extend(struct.pack("<I", 0))
    buffer.extend(
        struct.pack(
            "<iI",
            DEFAULT_LEASE_DURATION_SECONDS,
            ntp_fraction_from_nanoseconds(DEFAULT_LEASE_DURATION_NANOSECONDS),
        )
    )


def append_parameter_history(buffer: bytearray) -> None:
    append_parameter_header(buffer, PID_HISTORY, 8)
    buffer.extend(struct.pack("<II", 0, 1))


def append_parameter_key_hash(buffer: bytearray, guid: bytes) -> None:
    append_parameter_header(buffer, PID_KEY_HASH, 16)
    buffer.extend(guid)


def append_parameter_sentinel(buffer: bytearray) -> None:
    append_parameter_header(buffer, PID_SENTINEL, 0)


def build_parameter_list_payload(parameter_buffer: bytearray) -> bytes:
    return PL_CDR_LE + bytes(parameter_buffer)


def build_data_submessage(reader_id: bytes, writer_id: bytes, sequence_number: int, payload: bytes) -> bytes:
    high = sequence_number >> 32
    low = sequence_number & 0xFFFFFFFF
    submessage_payload = bytearray()
    submessage_payload.extend(struct.pack("<HH", 0, DATA_SUBMESSAGE_OCTETS_TO_INLINE_QOS))
    submessage_payload.extend(reader_id)
    submessage_payload.extend(writer_id)
    submessage_payload.extend(struct.pack("<iI", high, low))
    submessage_payload.extend(payload)
    align4(submessage_payload)
    return (
        struct.pack("<BBH", DATA_SUBMESSAGE_KIND, DATA_SUBMESSAGE_FLAGS, len(submessage_payload))
        + submessage_payload
    )


def build_rtps_message(guid_prefix: bytes, reader_id: bytes, writer_id: bytes, sequence_number: int, payload: bytes) -> bytes:
    header = RTPS_MAGIC + bytes((2, 3)) + VENDOR_ID + guid_prefix
    return header + build_data_submessage(reader_id, writer_id, sequence_number, payload)


def parse_parameter_list(payload: bytes) -> List[tuple[int, bytes]]:
    if len(payload) < 4 or payload[:4] != PL_CDR_LE:
        return []
    parameters: List[tuple[int, bytes]] = []
    offset = 4
    while offset + 4 <= len(payload):
        pid, length = struct.unpack_from("<HH", payload, offset)
        offset += 4
        if pid == PID_SENTINEL:
            break
        if offset + length > len(payload):
            return []
        value = payload[offset : offset + length]
        parameters.append((pid, value))
        offset += length
        offset += (4 - (length % 4)) & 0x3
    return parameters


def find_parameter(parameters: Iterable[tuple[int, bytes]], pid: int) -> Optional[bytes]:
    for candidate_pid, candidate_value in parameters:
        if candidate_pid == pid:
            return candidate_value
    return None


def find_parameters(parameters: Iterable[tuple[int, bytes]], pid: int) -> List[bytes]:
    return [candidate_value for candidate_pid, candidate_value in parameters if candidate_pid == pid]


def parse_guid(value: Optional[bytes]) -> Optional[bytes]:
    if value is None or len(value) != 16:
        return None
    return value


def parse_u32_le(value: Optional[bytes]) -> Optional[int]:
    if value is None or len(value) < 4:
        return None
    return struct.unpack_from("<I", value, 0)[0]


def parse_bool(value: Optional[bytes]) -> Optional[bool]:
    if value is None or not value:
        return None
    return value[0] != 0


def parse_cdr_string(value: Optional[bytes]) -> Optional[str]:
    if value is None or len(value) < 4:
        return None
    length = struct.unpack_from("<I", value, 0)[0]
    if length == 0 or 4 + length > len(value):
        return None
    raw = value[4 : 4 + length]
    if raw.endswith(b"\x00"):
        raw = raw[:-1]
    return raw.decode("utf-8", errors="replace")


def parse_octet_sequence(value: Optional[bytes]) -> Optional[bytes]:
    if value is None or len(value) < 4:
        return None
    length = struct.unpack_from("<I", value, 0)[0]
    if 4 + length > len(value):
        return None
    return value[4 : 4 + length]


def parse_locator(value: Optional[bytes]) -> tuple[str, int]:
    if value is None or len(value) != 24:
        return ("0.0.0.0", 0)
    kind = struct.unpack_from("<I", value, 0)[0]
    if kind != KIND_UDP_V4:
        return ("0.0.0.0", 0)
    port = struct.unpack_from("<I", value, 4)[0]
    ip_address = socket.inet_ntoa(value[20:24])
    return (ip_address, port)


def parse_reliability(value: Optional[bytes]) -> str:
    kind = parse_u32_le(value)
    return "reliable" if kind == RTPS_QOS_RELIABILITY_RELIABLE else "best-effort"


def extract_enclave(value: Optional[bytes]) -> str:
    if not value:
        return "/"
    text = value.decode("utf-8", errors="replace")
    marker = "enclave="
    start = text.find(marker)
    if start < 0:
        return "/"
    start += len(marker)
    end = text.find(";", start)
    if end < 0:
        end = len(text)
    return text[start:end] or "/"


def serialize_uint32_cdr(value: int) -> bytes:
    return b"\x00\x01\x00\x00" + struct.pack("<I", value & 0xFFFFFFFF)


def deserialize_uint32_cdr(payload: bytes) -> Optional[int]:
    if len(payload) < 8 or payload[:2] != b"\x00\x01":
        return None
    return struct.unpack_from("<I", payload, 4)[0]


class RtpsHostHarness:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.ports = compute_port_mapping(args.domain_id, args.participant_id)
        self.guid_prefix = make_guid_prefix(args.node_name, args.domain_id, args.participant_id)
        self.participant_guid = make_guid(self.guid_prefix, PARTICIPANT_ENTITY_ID)
        self.sequence_numbers: Dict[bytes, int] = {}
        self.discovered_participants: Dict[bytes, ParticipantProxy] = {}
        self.discovered_writers: Dict[bytes, EndpointProxy] = {}
        self.discovered_readers: Dict[bytes, EndpointProxy] = {}
        # How many times we have announced our endpoints to each peer. Unlike
        # SPDP (a liveliness refresh that must keep repeating), SEDP only needs
        # to arrive once. embeddedRTPS adds a ReaderProxy per announcement with
        # no dedupe by GUID (Writer::addNewMatchedReader -> m_proxies.add), so
        # repeating forever exhausts its pool: "MemoryPool RESSOURCE LIMIT
        # EXCEEDED". Send a few times to cover packet loss, then stop.
        self.sedp_announce_counts: Dict[bytes, int] = {}
        # same cap, but keyed by (address, port) for peers seeded by hand — they
        # are never discovered via SPDP, so they have no participant GUID
        self.peer_sedp_counts: Dict[Tuple[str, int], int] = {}
        # Each endpoint's SEDP announcement, built once and then resent
        # verbatim; see _sedp_announcement for why the bytes must not change.
        self._sedp_messages: Dict[Tuple[str, int], bytes] = {}
        self.joined_user_multicast_groups: Set[str] = set()

        # Peers to reach without multicast discovery (see seed_peer_discovery).
        self.peer_addresses = self._resolve_peers()
        self.peer_participant_ids = list(getattr(args, "peer_participant_ids", None) or [])

        self.local_writers = [
            WriterConfig(
                topic_name=args.publish_topic,
                type_name=args.type_name,
                reliable=args.reliable,
                entity_index=0,
            )
        ] if args.publish_topic else []
        subscribe_type_name = getattr(args, "subscribe_type_name", None) or args.type_name
        self.local_readers = [
            ReaderConfig(
                topic_name=topic_name,
                type_name=subscribe_type_name,
                reliable=False,
                entity_index=index,
            )
            for index, topic_name in enumerate(args.subscribe_topic)
        ]

        self.metatraffic_multicast_sock = self._create_metatraffic_multicast_socket()
        self.metatraffic_unicast_sock = self._create_bound_udp_socket(self.ports.metatraffic_unicast)
        self.user_unicast_sock = self._create_bound_udp_socket(self.ports.user_unicast)
        self.user_multicast_sock = self._create_user_multicast_socket()
        self._configure_multicast_sender(self.metatraffic_unicast_sock)
        self._configure_multicast_sender(self.user_unicast_sock)

        # Lets a caller end run() without a KeyboardInterrupt or a --duration,
        # which is what a GUI's Disconnect needs.
        self._stop_event = threading.Event()
        self.next_discovery_send = 0.0
        self.next_publish_send = 0.0
        self.last_no_participant_log = 0.0
        self.last_unknown_writer_log = 0.0

    @staticmethod
    def _ignore_icmp_port_unreachable(sock: socket.socket) -> None:
        """Stop Windows turning an ICMP port-unreachable into a fatal socket error.

        On Windows a UDP sendto that draws an ICMP port-unreachable makes the
        *next* recvfrom on that socket raise ConnectionResetError (WinError
        10054), and the socket stays poisoned. RTPS sprays discovery at ports
        that may not be open yet — a peer that has not created a participant, or
        a --peer participant-id sweep where only one id is real — so this is
        normal traffic, not an error. SIO_UDP_CONNRESET(False) suppresses it.
        No-op everywhere else, where UDP already ignores ICMP.
        """
        if not hasattr(socket, "SIO_UDP_CONNRESET"):
            return
        try:
            sock.ioctl(socket.SIO_UDP_CONNRESET, False)
        except OSError as exc:
            log(f"[socket] could not disable UDP connreset reporting: {exc}")

    def _create_bound_udp_socket(self, port: int) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self._ignore_icmp_port_unreachable(sock)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, "SO_REUSEPORT"):
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except OSError:
                # Some platforms expose SO_REUSEPORT but reject setting it; this is
                # a best-effort optimization and is not required for correctness.
                pass
        sock.bind((self.args.bind_address, port))
        sock.setblocking(False)
        return sock

    def _create_metatraffic_multicast_socket(self) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self._ignore_icmp_port_unreachable(sock)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, "SO_REUSEPORT"):
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except OSError:
                # Some platforms expose SO_REUSEPORT but reject setting it; this is
                # a best-effort optimization and is not required for correctness.
                pass
        try:
            sock.bind((self.args.multicast_group, self.ports.metatraffic_multicast))
        except OSError:
            # Not all platforms allow binding directly to the multicast group
            # address, so fall back to the selected local interface address.
            sock.bind((self.args.bind_address, self.ports.metatraffic_multicast))
        interface_ip = self.args.multicast_interface or self.args.advertised_address
        membership = socket.inet_aton(self.args.multicast_group) + socket.inet_aton(interface_ip)
        try:
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
        except OSError as exc:
            # A point-to-point overlay interface (Tailscale, most VPNs) carries
            # no multicast and rejects the join. That is fatal for ordinary
            # discovery but harmless when --peer is seeding it by unicast, so
            # only refuse when there is no peer to fall back on.
            if not getattr(self.args, "peer", None):
                raise
            log(f"[multicast] join on {interface_ip} failed ({exc}); relying on --peer instead")
        sock.setblocking(False)
        return sock

    def _create_user_multicast_socket(self) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self._ignore_icmp_port_unreachable(sock)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, "SO_REUSEPORT"):
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except OSError:
                # Some platforms expose SO_REUSEPORT but reject setting it; this is
                # a best-effort optimization and is not required for correctness.
                pass
        # Bind to INADDR_ANY, not a unicast address: multicast datagrams are addressed to the group
        # (e.g. 239.255.0.11), so a socket bound to a specific unicast interface address will not
        # receive them on Linux (and unreliably on macOS). Delivery is decided by the joined
        # group(s) + port. This socket may join several user-data groups, so binding to a single
        # group address is not an option.
        sock.bind(("", self.ports.user_multicast))
        sock.setblocking(False)
        return sock

    def _join_user_multicast_group(self, group: str) -> None:
        if group in self.joined_user_multicast_groups:
            return
        interface_ip = self.args.multicast_interface or self.args.advertised_address
        membership = socket.inet_aton(group) + socket.inet_aton(interface_ip)
        self.user_multicast_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
        self.joined_user_multicast_groups.add(group)
        log(f"[multicast] joined user-data group {group}:{self.ports.user_multicast}")

    def _configure_multicast_sender(self, sock: socket.socket) -> None:
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        interface_ip = self.args.multicast_interface or self.args.advertised_address
        try:
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(interface_ip))
        except OSError as exc:
            # See _create_metatraffic_multicast_socket: no multicast on this
            # interface is survivable only because --peer does not need it.
            if not getattr(self.args, "peer", None):
                raise
            log(f"[multicast] sender setup on {interface_ip} failed ({exc}); --peer only")

    def _next_sequence(self, writer_entity_id: bytes) -> int:
        value = self.sequence_numbers.get(writer_entity_id, 1)
        self.sequence_numbers[writer_entity_id] = value + 1
        return value

    def _local_writer_guid(self, entity_index: int) -> bytes:
        return make_guid(self.guid_prefix, entity_id_for_index(entity_index, USER_WRITER_NO_KEY_KIND))

    def _local_reader_guid(self, entity_index: int) -> bytes:
        return make_guid(self.guid_prefix, entity_id_for_index(entity_index, USER_READER_NO_KEY_KIND))

    def build_spdp_announce_message(self) -> bytes:
        parameters = bytearray()
        append_parameter_protocol_version(parameters)
        append_parameter_vendor_id(parameters)
        append_parameter_u32(parameters, PID_DOMAIN_ID, self.args.domain_id)
        append_parameter_guid(parameters, PID_PARTICIPANT_GUID, self.participant_guid)
        append_parameter_locator(
            parameters,
            PID_METATRAFFIC_MULTICAST_LOCATOR,
            self.args.multicast_group,
            self.ports.metatraffic_multicast,
        )
        append_parameter_locator(
            parameters,
            PID_METATRAFFIC_UNICAST_LOCATOR,
            self.args.advertised_address,
            self.ports.metatraffic_unicast,
        )
        append_parameter_locator(
            parameters,
            PID_DEFAULT_UNICAST_LOCATOR,
            self.args.advertised_address,
            self.ports.user_unicast,
        )
        append_parameter_locator(
            parameters,
            PID_DEFAULT_MULTICAST_LOCATOR,
            self.args.multicast_group,
            self.ports.user_multicast,
        )
        append_parameter_duration(
            parameters,
            PID_PARTICIPANT_LEASE_DURATION,
            DEFAULT_LEASE_DURATION_SECONDS,
            DEFAULT_LEASE_DURATION_NANOSECONDS,
        )
        append_parameter_u32(parameters, PID_BUILTIN_ENDPOINT_SET, BUILTIN_ENDPOINT_SET)
        append_parameter_octet_sequence(
            parameters,
            PID_USER_DATA,
            f"enclave={self.args.enclave};".encode("utf-8"),
        )
        append_parameter_string_cdr(parameters, PID_ENTITY_NAME, self.args.node_name)
        append_parameter_sentinel(parameters)
        payload = build_parameter_list_payload(parameters)
        # Address the SPDP DATA to the builtin SPDP reader rather than
        # ENTITYID_UNKNOWN. embeddedRTPS (espp/rtps >= 1.2.0) routes an UNKNOWN
        # readerId via getReaderByWriterId(), which only finds a reader that
        # already has a proxy for this exact remote writer — impossible on
        # first contact, so the announcement is dropped and the peer never
        # learns we exist. An explicit reader id dispatches directly.
        return build_rtps_message(
            self.guid_prefix,
            SPDP_READER_ENTITY_ID,
            SPDP_WRITER_ENTITY_ID,
            self._next_sequence(SPDP_WRITER_ENTITY_ID),
            payload,
        )

    def build_sedp_publication_message(self, writer: WriterConfig) -> bytes:
        guid = self._local_writer_guid(writer.entity_index)
        parameters = bytearray()
        append_parameter_guid(parameters, PID_ENDPOINT_GUID, guid)
        append_parameter_locator(parameters, PID_UNICAST_LOCATOR, self.args.advertised_address, self.ports.user_unicast)
        append_parameter_guid(parameters, PID_PARTICIPANT_GUID, self.participant_guid)
        append_parameter_string_cdr(parameters, PID_TOPIC_NAME, writer.topic_name)
        append_parameter_string_cdr(parameters, PID_TYPE_NAME, writer.type_name)
        append_parameter_key_hash(parameters, guid)
        append_parameter_u32(parameters, PID_TYPE_MAX_SIZE_SERIALIZED, 8)
        append_parameter_protocol_version(parameters)
        append_parameter_vendor_id(parameters)
        append_parameter_durability(parameters)
        append_parameter_liveliness(parameters)
        append_parameter_reliability(parameters, writer.reliable)
        append_parameter_history(parameters)
        append_parameter_sentinel(parameters)
        payload = build_parameter_list_payload(parameters)
        return build_rtps_message(
            self.guid_prefix,
            SEDP_PUBLICATIONS_READER_ENTITY_ID,
            SEDP_PUBLICATIONS_WRITER_ENTITY_ID,
            self._next_sequence(SEDP_PUBLICATIONS_WRITER_ENTITY_ID),
            payload,
        )

    def build_sedp_subscription_message(self, reader: ReaderConfig) -> bytes:
        guid = self._local_reader_guid(reader.entity_index)
        parameters = bytearray()
        append_parameter_guid(parameters, PID_ENDPOINT_GUID, guid)
        append_parameter_locator(parameters, PID_UNICAST_LOCATOR, self.args.advertised_address, self.ports.user_unicast)
        append_parameter_bool(parameters, PID_EXPECTS_INLINE_QOS, False)
        append_parameter_guid(parameters, PID_PARTICIPANT_GUID, self.participant_guid)
        append_parameter_string_cdr(parameters, PID_TOPIC_NAME, reader.topic_name)
        append_parameter_string_cdr(parameters, PID_TYPE_NAME, reader.type_name)
        append_parameter_key_hash(parameters, guid)
        append_parameter_protocol_version(parameters)
        append_parameter_vendor_id(parameters)
        append_parameter_durability(parameters)
        append_parameter_liveliness(parameters)
        append_parameter_reliability(parameters, reader.reliable)
        append_parameter_history(parameters)
        append_parameter_sentinel(parameters)
        payload = build_parameter_list_payload(parameters)
        return build_rtps_message(
            self.guid_prefix,
            SEDP_SUBSCRIPTIONS_READER_ENTITY_ID,
            SEDP_SUBSCRIPTIONS_WRITER_ENTITY_ID,
            self._next_sequence(SEDP_SUBSCRIPTIONS_WRITER_ENTITY_ID),
            payload,
        )

    def _sedp_announcement(self, endpoint) -> bytes:
        """This endpoint's SEDP message, built once and resent verbatim.

        The peer's builtin SEDP reader is reliable, so it only accepts sequence
        numbers in order. Rebuilding the message for every repeat gave each
        copy a fresh number, so one announcement lost from a burst stalled every
        later one for good - no repeat ever carried the number it was waiting
        for. Seen on the bench over Tailscale: of seven announcements the HMI
        took the first four, and the readers behind the lost fifth never
        matched. Resending the same bytes lets a repeat fill exactly the hole,
        and a peer that already has it drops the duplicate.
        """
        is_writer = isinstance(endpoint, WriterConfig)
        key = ("pub" if is_writer else "sub", endpoint.entity_index)
        message = self._sedp_messages.get(key)
        if message is None:
            message = (self.build_sedp_publication_message(endpoint) if is_writer
                       else self.build_sedp_subscription_message(endpoint))
            self._sedp_messages[key] = message
        return message

    def _sedp_announcements(self) -> List[bytes]:
        """Every local endpoint's announcement, writers then readers, in a fixed order."""
        return ([self._sedp_announcement(writer) for writer in self.local_writers]
                + [self._sedp_announcement(reader) for reader in self.local_readers])

    def build_data_message(self, writer: WriterConfig, cdr_payload: bytes) -> bytes:
        # Standard RTPS: the DATA serializedPayload is exactly the CDR-encapsulated sample.
        writer_entity_id = entity_id_for_index(writer.entity_index, USER_WRITER_NO_KEY_KIND)
        return build_rtps_message(
            self.guid_prefix,
            ENTITY_ID_UNKNOWN,
            writer_entity_id,
            self._next_sequence(writer_entity_id),
            cdr_payload,
        )

    def _send_metatraffic(self, payload: bytes, target: Tuple[str, int]) -> bool:
        """sendto on the metatraffic socket, tolerating a closed peer port."""
        try:
            self.metatraffic_unicast_sock.sendto(payload, target)
            return True
        except OSError:
            return False

    def _resolve_peers(self) -> List[str]:
        """Resolve --peer hostnames/IPs to addresses, skipping what won't resolve."""
        resolved: List[str] = []
        for peer in getattr(self.args, "peer", None) or []:
            try:
                address = socket.gethostbyname(peer)
            except socket.gaierror as exc:
                log(f"[peer] could not resolve '{peer}': {exc}")
                continue
            if address not in resolved:
                resolved.append(address)
                # A subnet scan seeds hundreds of peers at once; one line each
                # would bury whatever the scan was run to find.
                if not getattr(self.args, "quiet", False):
                    log(f"[peer] seeding discovery at {peer}"
                        + (f" ({address})" if address != peer else ""))
        return resolved

    def seed_peer_discovery(self) -> None:
        """Unicast SPDP+SEDP straight at a known peer, for links with no multicast.

        Normal RTPS discovery needs multicast, which an L3 overlay (Tailscale,
        most VPNs) or a routed subnet will not carry. Given the peer's address we
        can skip it: its metatraffic unicast port is a pure function of the
        domain and participant id, so announce to each candidate id directly.

        embeddedRTPS registers the sender on our SPDP, then answers with SEDP
        unicast to the metatraffic locator our announcement advertised — so the
        rest of discovery follows without a single multicast packet. Our own SEDP
        goes with it, because the peer's reader will not accept our DATA until it
        has matched our writer.
        """
        if not self.peer_addresses:
            return
        spdp = self.build_spdp_announce_message()
        for address in self.peer_addresses:
            for participant_id in self.peer_participant_ids:
                port = compute_port_mapping(self.args.domain_id, participant_id).metatraffic_unicast
                target = (address, port)
                # SPDP repeats forever: it is a liveliness refresh, and the peer
                # dedupes by GUID (findRemoteParticipant -> refresh, not add).
                if not self._send_metatraffic(spdp, target):
                    continue  # nothing listening on this id; try the next
                # SEDP does not: embeddedRTPS adds a proxy per announcement it
                # accepts with no dedupe, so repeating forever could exhaust its
                # pool. (The copies are verbatim, so a peer that already took
                # one drops the repeat; the cap is belt and braces.)
                sent = self.peer_sedp_counts.get(target, 0)
                if sent >= self.SEDP_ANNOUNCE_REPEATS:
                    continue
                self.peer_sedp_counts[target] = sent + 1
                for message in self._sedp_announcements():
                    self._send_metatraffic(message, target)
                    time.sleep(self.SEDP_SEND_GAP_S)

    def send_spdp_announce_now(self) -> None:
        payload = self.build_spdp_announce_message()
        try:
            self.metatraffic_unicast_sock.sendto(
                payload,
                (self.args.multicast_group, self.ports.metatraffic_multicast),
            )
        except OSError as exc:
            # An interface with no multicast route (Tailscale, most VPNs); the
            # unicast seeding below is what reaches the peer in that case.
            if not self.peer_addresses:
                raise
            if not getattr(self, "_warned_multicast_send", False):
                self._warned_multicast_send = True
                log(f"[multicast] SPDP send failed ({exc}); seeding --peer by unicast only")
        # embeddedRTPS (espp/rtps >= 1.2.0) drops any SEDP message whose sender
        # it has not already registered via SPDP, so a peer that never receives
        # our multicast announcement will silently ignore our endpoints. Mirror
        # the announcement as unicast to every participant we know about — the
        # same way send_sedp_announcements_to() already reaches them.
        for participant in list(self.discovered_participants.values()):
            if not participant.address or participant.ports.metatraffic_unicast == 0:
                continue
            self.metatraffic_unicast_sock.sendto(
                payload,
                (participant.address, participant.ports.metatraffic_unicast),
            )

    # Rounds of SEDP per peer. Each round can repair one hole left by the ones
    # before it (see _sedp_announcement), so this is how many losses discovery
    # survives.
    SEDP_ANNOUNCE_REPEATS = 5
    # Gap between the announcements in a round. Sent back to back, a round is
    # a burst the HMI's W5500 does not always take whole.
    SEDP_SEND_GAP_S = 0.005

    def send_sedp_announcements_to(self, participant: ParticipantProxy) -> None:
        target = (participant.address, participant.ports.metatraffic_unicast)
        if participant.ports.metatraffic_unicast == 0 or not participant.address:
            return
        sent = self.sedp_announce_counts.get(participant.participant_guid, 0)
        if sent >= self.SEDP_ANNOUNCE_REPEATS:
            return
        self.sedp_announce_counts[participant.participant_guid] = sent + 1
        for message in self._sedp_announcements():
            self.metatraffic_unicast_sock.sendto(message, target)
            time.sleep(self.SEDP_SEND_GAP_S)

    def send_discovery_now(self) -> None:
        self.send_spdp_announce_now()
        self.seed_peer_discovery()
        for participant in list(self.discovered_participants.values()):
            self.send_sedp_announcements_to(participant)

    def publish_now(self) -> None:
        if not self.local_writers:
            return
        if not self._publish_value(self.local_writers[0], self.args.publish_value):
            now = time.monotonic()
            if now - self.last_no_participant_log > 2.0:
                log(
                    f"[publish] no discovered participants yet for topic '{self.local_writers[0].topic_name}', "
                    "waiting for SPDP"
                )
                self.last_no_participant_log = now
        else:
            log(
                f"[publish] sent {self.args.publish_value} on '{self.local_writers[0].topic_name}' "
                f"using {len(self._build_user_targets(self.local_writers[0]))} discovered target(s)"
            )

    def _build_user_targets(self, writer: WriterConfig) -> List[Tuple[str, int]]:
        targets: List[Tuple[str, int]] = []
        for reader in self.discovered_readers.values():
            if reader.topic_name != writer.topic_name:
                continue
            # A seeded peer is, by definition, somewhere multicast does not
            # reach, so its advertised multicast locator is a dead end — take
            # the unicast one even when both are offered.
            if reader.multicast_locators and not self.peer_addresses:
                for multicast_address, multicast_port in reader.multicast_locators:
                    target = (multicast_address, multicast_port)
                    if target not in targets:
                        targets.append(target)
                continue
            if reader.unicast_port > 0 and reader.unicast_address:
                target = (reader.unicast_address, reader.unicast_port)
                if target not in targets:
                    targets.append(target)
        if targets:
            return targets
        for participant in self.discovered_participants.values():
            target = (participant.address, participant.ports.user_unicast)
            if participant.address and participant.ports.user_unicast > 0 and target not in targets:
                targets.append(target)
        if targets:
            return targets
        # Last resort for a seeded peer whose SEDP never came back (a one-way
        # link, e.g. a NATing subnet router). Its user unicast port is the same
        # pure function of domain + participant id, so publish blind: if our
        # SEDP reached it, its reader is listening there and matched our writer.
        for address in self.peer_addresses:
            for participant_id in self.peer_participant_ids:
                port = compute_port_mapping(self.args.domain_id, participant_id).user_unicast
                target = (address, port)
                if target not in targets:
                    targets.append(target)
        return targets

    def send_user_datagram(self, payload: bytes, target: Tuple[str, int]) -> bool:
        """sendto on the user socket, tolerating a peer port that is not open."""
        try:
            self.user_unicast_sock.sendto(payload, target)
            return True
        except OSError as exc:
            log(f"[publish] send to {target[0]}:{target[1]} failed: {exc}")
            return False

    def _publish_value(self, writer: WriterConfig, value: int, target: Optional[tuple[str, int]] = None) -> bool:
        payload = self.build_data_message(writer, serialize_uint32_cdr(value))
        if target is not None:
            return self.send_user_datagram(payload, target)
        targets = self._build_user_targets(writer)
        if not targets:
            return False
        for destination in targets:
            self.send_user_datagram(payload, destination)
        return True

    # Reader entity to answer with, per builtin writer we may hear a HEARTBEAT from.
    BUILTIN_HEARTBEAT_READERS = {
        SEDP_PUBLICATIONS_WRITER_ENTITY_ID: SEDP_PUBLICATIONS_READER_ENTITY_ID,
        SEDP_SUBSCRIPTIONS_WRITER_ENTITY_ID: SEDP_SUBSCRIPTIONS_READER_ENTITY_ID,
    }

    def respond_to_heartbeats(self, packet: bytes, sender_ip: str) -> None:
        """Answer a reliable builtin writer's HEARTBEAT with an ACKNACK.

        embeddedRTPS publishes SEDP over reliable stateful writers: it announces
        what it holds via HEARTBEAT and only sends the DATA once a reader ACKNACKs
        for it. Without this the peer's endpoints are never learned, so its samples
        arrive but cannot be routed to a topic.
        """
        for guid_prefix, _reader_id, writer_id, first_sn, last_sn, count in parse_heartbeats(packet):
            local_reader = self.BUILTIN_HEARTBEAT_READERS.get(writer_id)
            if local_reader is None or last_sn < first_sn:
                continue
            port = next(
                (
                    p.ports.metatraffic_unicast
                    for p in self.discovered_participants.values()
                    if p.guid_prefix == guid_prefix
                ),
                0,
            )
            if not port:
                continue
            acknack = build_acknack_message(
                self.guid_prefix, local_reader, writer_id, first_sn, last_sn - first_sn + 1, count
            )
            self.metatraffic_unicast_sock.sendto(acknack, (sender_ip, port))

    def handle_metatraffic_packet(self, packet: bytes, sender_ip: str) -> None:
        self.respond_to_heartbeats(packet, sender_ip)
        for _guid_prefix, writer_id, serialized_payload, _reader_id in parse_rtps_data_messages(
            packet
        ):
            parameters = parse_parameter_list(serialized_payload)
            if not parameters:
                continue

            if writer_id == SPDP_WRITER_ENTITY_ID:
                self._handle_spdp(parameters, sender_ip)
            elif writer_id == SEDP_PUBLICATIONS_WRITER_ENTITY_ID:
                self._handle_sedp(parameters, sender_ip, is_reader=False)
            elif writer_id == SEDP_SUBSCRIPTIONS_WRITER_ENTITY_ID:
                self._handle_sedp(parameters, sender_ip, is_reader=True)

    def _handle_spdp(self, parameters: List[tuple[int, bytes]], sender_ip: str) -> None:
        participant_guid = parse_guid(find_parameter(parameters, PID_PARTICIPANT_GUID))
        if participant_guid is None or participant_guid[:12] == self.guid_prefix:
            return

        meta_ip, meta_uc_port = parse_locator(find_parameter(parameters, PID_METATRAFFIC_UNICAST_LOCATOR))
        _, meta_mc_port = parse_locator(find_parameter(parameters, PID_METATRAFFIC_MULTICAST_LOCATOR))
        user_ip, user_uc_port = parse_locator(find_parameter(parameters, PID_DEFAULT_UNICAST_LOCATOR))
        _, user_mc_port = parse_locator(find_parameter(parameters, PID_DEFAULT_MULTICAST_LOCATOR))

        participant = ParticipantProxy(
            participant_guid=participant_guid,
            guid_prefix=participant_guid[:12],
            name=parse_cdr_string(find_parameter(parameters, PID_ENTITY_NAME)) or "",
            enclave=extract_enclave(parse_octet_sequence(find_parameter(parameters, PID_USER_DATA))),
            address=user_ip if user_ip != "0.0.0.0" else sender_ip,
            ports=PortMapping(
                metatraffic_multicast=meta_mc_port,
                metatraffic_unicast=meta_uc_port,
                user_multicast=user_mc_port,
                user_unicast=user_uc_port,
            ),
            builtin_endpoints=parse_u32_le(find_parameter(parameters, PID_BUILTIN_ENDPOINT_SET)) or 0,
        )

        is_new = participant_guid not in self.discovered_participants
        self.discovered_participants[participant_guid] = participant
        label = participant.name or hex_string(participant.guid_prefix)
        log(
            f"[spdp] participant '{label}' at {participant.address} "
            f"(meta={participant.ports.metatraffic_unicast}, user={participant.ports.user_unicast}, "
            f"enclave={participant.enclave})"
        )
        if is_new:
            self.send_sedp_announcements_to(participant)

    def _handle_sedp(self, parameters: List[tuple[int, bytes]], sender_ip: str, is_reader: bool) -> None:
        endpoint_guid = parse_guid(find_parameter(parameters, PID_ENDPOINT_GUID))
        if endpoint_guid is None or endpoint_guid[:12] == self.guid_prefix:
            return

        participant_guid = parse_guid(find_parameter(parameters, PID_PARTICIPANT_GUID))
        if participant_guid is None:
            participant_guid = endpoint_guid[:12] + PARTICIPANT_ENTITY_ID

        endpoint_ip, endpoint_port = parse_locator(find_parameter(parameters, PID_UNICAST_LOCATOR))
        multicast_locators = [
            parse_locator(value)
            for value in find_parameters(parameters, PID_MULTICAST_LOCATOR)
        ]
        multicast_locators = [
            (multicast_address, multicast_port)
            for multicast_address, multicast_port in multicast_locators
            if multicast_address != "0.0.0.0" and multicast_port > 0
        ]
        endpoint = EndpointProxy(
            guid=endpoint_guid,
            participant_guid=participant_guid,
            topic_name=parse_cdr_string(find_parameter(parameters, PID_TOPIC_NAME)) or "",
            type_name=parse_cdr_string(find_parameter(parameters, PID_TYPE_NAME)) or "",
            reliability=parse_reliability(find_parameter(parameters, PID_RELIABILITY)),
            is_reader=is_reader,
            expects_inline_qos=parse_bool(find_parameter(parameters, PID_EXPECTS_INLINE_QOS)) or False,
            unicast_address=endpoint_ip if endpoint_ip != "0.0.0.0" else sender_ip,
            unicast_port=endpoint_port,
            multicast_locators=multicast_locators,
        )
        endpoint_map = self.discovered_readers if is_reader else self.discovered_writers
        is_new = endpoint_guid not in endpoint_map
        endpoint_map[endpoint_guid] = endpoint
        if not is_reader:
            subscribed_topics = {reader.topic_name for reader in self.local_readers}
            if endpoint.topic_name in subscribed_topics:
                for multicast_address, multicast_port in endpoint.multicast_locators:
                    if multicast_port == self.ports.user_multicast:
                        self._join_user_multicast_group(multicast_address)
        if is_new:
            kind = "reader" if is_reader else "writer"
            log(
                f"[sedp] {kind} topic='{endpoint.topic_name}' type='{endpoint.type_name}' "
                f"reliability={endpoint.reliability} participant={guid_to_string(endpoint.participant_guid)}"
            )

    def topic_for_sample(self, guid_prefix: bytes, writer_id: bytes,
                         reader_id: bytes) -> Optional[str]:
        """Which topic a DATA submessage belongs to, or None if it can't be placed.

        The normal route is SEDP: the writer GUID is looked up in the endpoints
        the peer announced. A peer seeded by --peer never announces them to us,
        though — embeddedRTPS sends SEDP only to participants it discovered
        itself, and a unicast SPDP does not put us in that set, so
        discovered_writers stays empty and every sample would be dropped as
        "undiscovered".

        The sample still carries the reader entity id the writer addressed it
        to, which it learned from OUR subscription announcement. That names one
        of our own readers, so it places the sample just as definitively.
        """
        writer = self.discovered_writers.get(guid_prefix + writer_id)
        if writer is not None:
            return writer.topic_name
        if reader_id and reader_id != ENTITYID_UNKNOWN:
            for reader in self.local_readers:
                if entity_id_for_index(reader.entity_index, USER_READER_NO_KEY_KIND) == reader_id:
                    return reader.topic_name
        return None

    def handle_user_packet(self, packet: bytes, sender_ip: str, sender_port: int) -> None:
        subscribed_topics = {reader.topic_name for reader in self.local_readers}
        for guid_prefix, writer_id, serialized_payload, reader_id in parse_rtps_data_messages(
            packet
        ):
            topic_name = self.topic_for_sample(guid_prefix, writer_id, reader_id)
            if topic_name is None:
                # Neither SEDP nor the addressed reader id could place this
                # sample. Surface it (rate-limited) rather than dropping silently.
                now = time.monotonic()
                if now - self.last_unknown_writer_log > 2.0:
                    log(
                        f"[data] received {len(serialized_payload)}-byte sample from UNDISCOVERED "
                        f"writer {guid_to_string(guid_prefix + writer_id)} at "
                        f"{sender_ip}:{sender_port}, addressed to reader {reader_id.hex()}; cannot "
                        f"route (discovered_writers={len(self.discovered_writers)})"
                    )
                    self.last_unknown_writer_log = now
                continue
            if topic_name not in subscribed_topics:
                continue
            writer = self.discovered_writers.get(guid_prefix + writer_id)
            maybe_value = deserialize_uint32_cdr(serialized_payload)
            if maybe_value is None:
                continue
            # writer is None when the sample was routed by reader id alone, so
            # its QoS is not something we ever learned
            reliability = writer.reliability if writer is not None else "unknown"
            log(
                f"[data] topic='{topic_name}' value={maybe_value} reliability={reliability} "
                f"from {sender_ip}:{sender_port} writer={hex_string(writer_id)}"
            )
            if self.args.echo_received and self.local_writers:
                out_writer = self.local_writers[0]
                if self._publish_value(out_writer, maybe_value):
                    log(f"[echo] responded with value={maybe_value} on '{out_writer.topic_name}'")
                else:
                    self._publish_value(out_writer, maybe_value, (sender_ip, sender_port))
                    log(
                        f"[echo] responded with value={maybe_value} on '{out_writer.topic_name}' "
                        f"to {sender_ip}:{sender_port}"
                    )

    def run(self) -> None:
        start_time = time.monotonic()
        self.next_discovery_send = start_time
        self.next_publish_send = start_time + self.args.publish_interval

        # Probe and scan participants are created and torn down constantly;
        # their banners would drown the log they exist to populate.
        if not getattr(self.args, "quiet", False):
            log(
                "Starting RTPS host harness\n"
                f"  node: {self.args.node_name}\n"
                f"  advertised address: {self.args.advertised_address}\n"
                f"  domain/participant: {self.args.domain_id}/{self.args.participant_id}\n"
                f"  ports: meta_mc={self.ports.metatraffic_multicast}, "
                f"meta_uc={self.ports.metatraffic_unicast}, "
                f"user_mc={self.ports.user_multicast}, user_uc={self.ports.user_unicast}"
            )
            if self.local_readers:
                log("  readers: " + ", ".join(r.topic_name for r in self.local_readers))
            if self.local_writers:
                writer = self.local_writers[0]
                writer_mode = "echo responder" if self.args.echo_received else "periodic publisher"
                interval_text = (
                    f", publish value={self.args.publish_value} "
                    f"every {self.args.publish_interval:.2f}s"
                    if self.args.publish_interval > 0
                    else ""
                )
                log(f"  writer: {writer.topic_name} "
                    f"({reliability_to_name(writer.reliable)}, {writer_mode}{interval_text})")

        try:
            while not self._stop_event.is_set():
                now = time.monotonic()
                if now >= self.next_discovery_send:
                    self.send_discovery_now()
                    self.next_discovery_send = now + self.args.announce_period
                if self.local_writers and self.args.publish_interval > 0 and now >= self.next_publish_send:
                    self.publish_now()
                    self.next_publish_send = now + self.args.publish_interval
                if self.args.duration > 0 and now - start_time >= self.args.duration:
                    break

                readable, _, _ = select.select(
                    [
                        self.metatraffic_multicast_sock,
                        self.metatraffic_unicast_sock,
                        self.user_unicast_sock,
                        self.user_multicast_sock,
                    ],
                    [],
                    [],
                    0.2,
                )
                for sock in readable:
                    try:
                        packet, sender = sock.recvfrom(4096)
                    except ConnectionResetError:
                        # A previous send drew an ICMP port-unreachable. The
                        # ioctl above normally suppresses this; if it did not
                        # take, dropping the read is still better than killing
                        # discovery over a closed port on the peer.
                        continue
                    sender_ip, sender_port = sender[0], sender[1]
                    is_user = sock is self.user_unicast_sock or sock is self.user_multicast_sock
                    if getattr(self.args, "trace_packets", False):
                        log(
                            f"[raw:{'user' if is_user else 'meta'}] {len(packet)}B from "
                            f"{sender_ip}:{sender_port} :: {describe_submessages(packet)}"
                        )
                    if is_user:
                        self.handle_user_packet(packet, sender_ip, sender_port)
                    else:
                        self.handle_metatraffic_packet(packet, sender_ip)
        except KeyboardInterrupt:
            log("Stopping RTPS host harness")
        finally:
            self.close()

    def stop(self) -> None:
        """Ask run() to return. Safe from any thread; run() closes the sockets."""
        self._stop_event.set()

    def close(self) -> None:
        for sock in (
            self.metatraffic_multicast_sock,
            self.metatraffic_unicast_sock,
            self.user_unicast_sock,
            self.user_multicast_sock,
        ):
            try:
                sock.close()
            except OSError as exc:
                log(f"[close] ignoring socket close failure for {sock!r}: {exc}")


def describe_submessages(packet: bytes) -> str:
    """Compact dump of an RTPS message's submessage headers, for --trace-packets.

    Surfaces the two things parse_rtps_data_messages() silently drops a DATA on:
    the InlineQos flag being set, and octetsToInlineQos differing from 16. Also
    flags octetsToNextHeader==0 ("rest of message"), which the parser mishandles.
    """
    if len(packet) < 20 or not packet.startswith(RTPS_MAGIC):
        return f"not-RTPS ({len(packet)}B)"
    parts = []
    offset = 20
    while offset + 4 <= len(packet):
        kind = packet[offset]
        flags = packet[offset + 1]
        length = struct.unpack_from("<H", packet, offset + 2)[0]
        body = packet[offset + 4 : offset + 4 + length] if length else packet[offset + 4 :]
        detail = ""
        if kind == DATA_SUBMESSAGE_KIND:
            detail = f" D={bool(flags & 0x04)} Q={bool(flags & 0x02)}"
            if len(body) >= 4:
                detail += f" octetsToInlineQos={struct.unpack_from('<H', body, 2)[0]}"
        parts.append(f"kind=0x{kind:02x} flags=0x{flags:02x} len={length}{detail}")
        if length == 0:
            parts.append("<- len=0 means 'rest of message'; parser cannot walk past this")
            break
        offset += 4 + length
    return " | ".join(parts)


def parse_heartbeats(packet: bytes) -> List[tuple[bytes, bytes, bytes, int, int, int]]:
    """Return (guid_prefix, reader_id, writer_id, first_sn, last_sn, count) per HEARTBEAT."""
    if len(packet) < 20 or not packet.startswith(RTPS_MAGIC):
        return []
    guid_prefix = packet[8:20]
    offset = 20
    beats: List[tuple[bytes, bytes, bytes, int, int, int]] = []
    while offset + 4 <= len(packet):
        kind = packet[offset]
        length = struct.unpack_from("<H", packet, offset + 2)[0]
        offset += 4
        if length == 0 or offset + length > len(packet):
            break
        body = packet[offset : offset + length]
        offset += length
        if kind != HEARTBEAT_SUBMESSAGE_KIND or len(body) < 28:
            continue
        reader_id = body[0:4]
        writer_id = body[4:8]
        first_high, first_low = struct.unpack_from("<iI", body, 8)
        last_high, last_low = struct.unpack_from("<iI", body, 16)
        count = struct.unpack_from("<I", body, 24)[0]
        beats.append(
            (
                guid_prefix,
                reader_id,
                writer_id,
                (first_high << 32) | first_low,
                (last_high << 32) | last_low,
                count,
            )
        )
    return beats


def build_acknack_message(
    guid_prefix: bytes, reader_id: bytes, writer_id: bytes, base_sn: int, num_bits: int, count: int
) -> bytes:
    """ACKNACK requesting `num_bits` sequence numbers starting at `base_sn`.

    The readerSNState bitmap marks every requested sequence as missing, which is
    how a reliable writer is told to (re)send them.
    """
    num_bits = max(0, min(num_bits, 256))
    body = bytearray()
    body.extend(reader_id)
    body.extend(writer_id)
    body.extend(struct.pack("<iI", base_sn >> 32, base_sn & 0xFFFFFFFF))
    body.extend(struct.pack("<I", num_bits))
    for word in range((num_bits + 31) // 32):
        remaining = num_bits - word * 32
        bits = 0xFFFFFFFF if remaining >= 32 else (0xFFFFFFFF << (32 - remaining)) & 0xFFFFFFFF
        body.extend(struct.pack("<I", bits))
    body.extend(struct.pack("<I", count))
    header = RTPS_MAGIC + bytes((2, 3)) + VENDOR_ID + guid_prefix
    return header + struct.pack(
        "<BBH", ACKNACK_SUBMESSAGE_KIND, ACKNACK_SUBMESSAGE_FLAGS, len(body)
    ) + bytes(body)


def parse_rtps_data_messages(packet: bytes) -> List[tuple[bytes, bytes, bytes, bytes]]:
    """Return (guid_prefix, writer_id, serialized_payload, reader_id) per DATA submessage.

    reader_id is the entity the writer addressed the sample to. It is often
    ENTITYID_UNKNOWN, but a writer that matched us through SEDP fills it in with
    our own reader's entity id — which is the only way to route a sample when
    the writer itself was never discovered (see topic_for_sample).
    """
    if len(packet) < 20 or not packet.startswith(RTPS_MAGIC):
        return []
    guid_prefix = packet[8:20]
    offset = 20
    messages: List[tuple[bytes, bytes, bytes, bytes]] = []
    while offset + 4 <= len(packet):
        kind = packet[offset]
        flags = packet[offset + 1]
        length = struct.unpack_from("<H", packet, offset + 2)[0]
        offset += 4
        if offset + length > len(packet):
            break
        payload = packet[offset : offset + length]
        offset += length
        if kind != DATA_SUBMESSAGE_KIND or (flags & 0x04) == 0:
            continue
        if len(payload) < 20:
            continue
        extra_flags, octets_to_inline_qos = struct.unpack_from("<HH", payload, 0)
        del extra_flags
        if (flags & 0x02) != 0 or octets_to_inline_qos != DATA_SUBMESSAGE_OCTETS_TO_INLINE_QOS:
            continue
        reader_id = payload[4:8]
        writer_id = payload[8:12]
        serialized_payload = payload[20:]
        messages.append((guid_prefix, writer_id, serialized_payload, reader_id))
    return messages


def run_self_test() -> int:
    """Validate the wire-format encoders/decoders against firmware expectations (no network I/O)."""
    failures: List[str] = []

    def check(name: str, condition: bool) -> None:
        log(f"  [{'PASS' if condition else 'FAIL'}] {name}")
        if not condition:
            failures.append(name)

    # Locators: kind/port are little-endian, address is raw network-order bytes; round-trips.
    loc = locator_bytes("192.168.1.5", 7411)
    check("locator kind is little-endian", loc[:4] == struct.pack("<I", KIND_UDP_V4))
    check("locator port is little-endian", loc[4:8] == struct.pack("<I", 7411))
    check("locator round-trips", parse_locator(loc) == ("192.168.1.5", 7411))

    # Durations are encoded as NTP fraction (1/2^32 s), not nanoseconds.
    expected_fraction = (DEFAULT_MAX_BLOCKING_NANOSECONDS << 32) // 1_000_000_000
    rel = bytearray()
    append_parameter_reliability(rel, reliable=False)
    fraction = struct.unpack_from("<I", rel, 4 + 4 + 4)[0]  # header(4) + kind(4) + max_blocking_sec(4)
    check("reliability max_blocking is NTP fraction", fraction == expected_fraction)
    check("100ms fraction ~= 0.1 * 2^32", abs(fraction - (1 << 32) // 10) <= 1)

    # Every emitted parameterLength is a multiple of 4 (PL_CDR requirement).
    params = bytearray()
    append_parameter_string_cdr(params, PID_TOPIC_NAME, "rt/chatter")  # body 4+10+1=15 -> 16
    append_parameter_string_cdr(params, PID_TYPE_NAME, "std_msgs::msg::dds_::UInt32_")
    append_parameter_octet_sequence(params, PID_USER_DATA, b"enclave=/;")  # body 4+10=14 -> 16
    offset = 0
    aligned = True
    while offset + 4 <= len(params):
        pid, length = struct.unpack_from("<HH", params, offset)
        offset += 4
        if pid == PID_SENTINEL:
            break
        if length % 4 != 0:
            aligned = False
        offset += length
    check("all parameterLengths are multiples of 4", aligned)

    # PL_CDR round-trips the strings through the standard parser.
    parsed_params = parse_parameter_list(build_parameter_list_payload(params))
    check("topic name round-trips", parse_cdr_string(find_parameter(parsed_params, PID_TOPIC_NAME)) == "rt/chatter")

    # Standard CDR-over-RTPS DATA: payload is exactly the CDR sample, routed by writer GUID.
    prefix = make_guid_prefix("selftest", 0, 10)
    writer_id = entity_id_for_index(0, USER_WRITER_NO_KEY_KIND)
    cdr = serialize_uint32_cdr(0xDEADBEEF)
    message = build_rtps_message(prefix, ENTITY_ID_UNKNOWN, writer_id, 1, cdr)
    parsed = parse_rtps_data_messages(message)
    check("one DATA submessage parsed", len(parsed) == 1)
    if parsed:
        got_prefix, got_writer_id, payload, _reader_id = parsed[0]
        check("guid_prefix recovered", got_prefix == prefix)
        check("writer_id recovered", got_writer_id == writer_id)
        check("serializedPayload is raw CDR (no framing)", payload == cdr)
        check("uint32 round-trips", deserialize_uint32_cdr(payload) == 0xDEADBEEF)

    if failures:
        log(f"SELF-TEST FAILED ({len(failures)} check(s)): {', '.join(failures)}")
        return 1
    log("SELF-TEST PASSED")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Discover an ESPP RTPS participant from a PC/host and optionally "
            "exchange temporary UInt32 user-data samples."
        )
    )
    parser.add_argument("--node-name", default="python_rtps_host", help="Local participant name")
    parser.add_argument("--domain-id", type=int, default=0, help="RTPS domain id")
    parser.add_argument("--participant-id", type=int, default=10, help="Local participant id")
    parser.add_argument(
        "--bind-address",
        default=None,
        help="Local bind address (defaults to the advertised address rather than all interfaces)",
    )
    parser.add_argument(
        "--advertised-address",
        default=None,
        help="IPv4 address to advertise to peers (defaults to best-effort local IPv4)",
    )
    parser.add_argument(
        "--multicast-interface",
        default=None,
        help="IPv4 interface to use for multicast join/send (defaults to advertised address)",
    )
    parser.add_argument("--multicast-group", default="239.255.0.1", help="RTPS metatraffic multicast group")
    parser.add_argument(
        "--peer", action="append", default=None, metavar="HOST",
        help="Hostname or IP of a peer to reach without multicast discovery (repeatable). "
             "Use this when the peer is routed rather than on-link — over Tailscale, a VPN or "
             "another subnet — since none of those carry the multicast RTPS discovery needs. "
             "The ESP32 is usually 'espressif'.")
    parser.add_argument(
        "--peer-participant-ids", type=parse_participant_id_range, default="0-3", metavar="IDS",
        help="Participant ids to try on each --peer, as '0-3' or '0,1,2' (default 0-3). The peer's "
             "metatraffic port is derived from these, so the sweep covers not knowing its id.")
    parser.add_argument("--trace-packets", action="store_true",
                        help="Log every received UDP packet and its RTPS submessage headers")
    parser.add_argument("--enclave", default="/", help="Enclave string advertised in SPDP user data")
    parser.add_argument(
        "--subscribe-topic",
        action="append",
        default=None,
        help=f"Topic name to advertise as a local reader (repeatable). Defaults to {DEFAULT_REQUEST_TOPIC}.",
    )
    parser.add_argument(
        "--publish-topic",
        default=None,
        help=f"Topic name to publish as a local writer. Defaults to {DEFAULT_RESPONSE_TOPIC}.",
    )
    parser.add_argument("--publish-value", type=int, default=42, help="UInt32 value to publish")
    parser.add_argument(
        "--publish-interval",
        type=float,
        default=0.0,
        help="Seconds between periodic publish attempts when --publish-topic is set (0 disables periodic publishing)",
    )
    parser.set_defaults(echo_received=True)
    parser.add_argument(
        "--echo-received",
        dest="echo_received",
        action="store_true",
        help="Echo received subscribed-topic values back on the publish topic (enabled by default)",
    )
    parser.add_argument(
        "--no-echo-received",
        dest="echo_received",
        action="store_false",
        help="Disable request/response echo behavior and only use periodic publishing",
    )
    parser.add_argument("--reliable", action="store_true", help="Mark the local writer as reliable")
    parser.add_argument("--type-name", default="std_msgs/msg/UInt32", help="Advertised type name")
    parser.add_argument(
        "--announce-period",
        type=float,
        default=1.0,
        help="Seconds between periodic SPDP/SEDP discovery announcements",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Stop after this many seconds (0 = run until Ctrl+C)",
    )
    args = parser.parse_args()

    if args.subscribe_topic is None:
        args.subscribe_topic = [DEFAULT_REQUEST_TOPIC]
    if args.publish_topic is None:
        args.publish_topic = DEFAULT_RESPONSE_TOPIC
    if args.advertised_address is None:
        args.advertised_address = guess_local_ipv4()
    if args.bind_address is None:
        args.bind_address = args.advertised_address
    try:
        ipaddress.IPv4Address(args.bind_address)
        ipaddress.IPv4Address(args.advertised_address)
        ipaddress.IPv4Address(args.multicast_interface or args.advertised_address)
        ipaddress.IPv4Address(args.multicast_group)
    except ipaddress.AddressValueError as exc:
        parser.error(str(exc))
    if args.publish_interval < 0:
        parser.error("--publish-interval must be >= 0")
    if args.announce_period <= 0:
        parser.error("--announce-period must be > 0")
    return args


def main() -> int:
    # Handle --self-test before full argument parsing so it needs no network/address configuration.
    if "--self-test" in sys.argv:
        return run_self_test()
    args = parse_args()
    harness = RtpsHostHarness(args)
    harness.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
