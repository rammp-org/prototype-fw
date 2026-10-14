#!/usr/bin/env python3
# VENDORED: copied from pace-hmi-fw/scripts/rtps_net.py @ d8efcbc; keep in sync.
# Local changes: `import rammp_spec as spec` instead of rammp_rtps, and the
# throwaway probe participant in _listener_args carries the HMI's endpoints (a
# MibStatus reader, an XYTwist writer) rather than the MCB's, because this tool
# plays the joystick side and it is the MIB that has to answer the probe. The
# probe/scan participant id (19) and the config file are shared with the HMI
# tools; this tool stores its settings under its own keys (mib_*).
"""Network discovery and remembered settings for the RTPS test tools.

Three jobs, all stdlib:

  * name this PC's network adapters, so "which of these six addresses is the
    real LAN" is answerable by looking rather than guessing;
  * find the board, and from it work out which adapter actually reaches it;
  * remember what worked, so the next run needs no flags.

The adapter question is the one that has cost the most time on this project. A
multi-homed machine (Tailscale, VirtualBox, VMware, a VPN) has no single
"local IP", and the usual trick of routing to 8.8.8.8 answers a question nobody
asked — it names the adapter that reaches the *internet*, which on a bench
network is rarely the one that reaches the board. source_address_for() asks the
kernel the right question instead: which source address would a packet to THIS
destination leave from.
"""

from __future__ import annotations

import ipaddress
import json
import os
import socket
import sys
import threading
import time
from typing import Callable, Iterable, List, NamedTuple, Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rammp_spec as spec  # noqa: E402  (path setup must run first)
import rtps_host  # noqa: E402

#: Participant id used by the throwaway probe/scan participants. Kept clear of
#: the tools' own ids (10-15) so a probe can run while one of them is live.
PROBE_PARTICIPANT_ID = 19

#: Hostnames worth trying before anything more expensive. The ESP32 usually
#: registers itself with the router under this name.
DEFAULT_HOSTNAMES = ("espressif", "espressif.local")

ProgressFn = Optional[Callable[[str], None]]


class Adapter(NamedTuple):
    """One IPv4 address, with the adapter it belongs to."""

    name: str  #: friendly name, e.g. "Wi-Fi"; "" when it could not be read
    ip: str
    is_up: bool

    def label(self) -> str:
        # plain hyphen, not an em dash: this label is printed to consoles whose
        # code page mangles anything outside cp1252
        base = f"{self.name} - {self.ip}" if self.name else self.ip
        return base if self.is_up else f"{base} (down)"


# ---------------------------------------------------------------- adapters


def _list_adapters_windows() -> List[Adapter]:
    """Adapter names via GetAdaptersAddresses.

    ctypes rather than a powershell subprocess: no 200 ms startup, and no
    dependence on the locale of ipconfig's headings. The structures below only
    need to be correct as far as OperStatus; everything after it is ignored.
    """
    import ctypes
    from ctypes import wintypes

    AF_INET = 2
    GAA_FLAG_SKIP_ANYCAST = 0x0002
    GAA_FLAG_SKIP_MULTICAST = 0x0004
    GAA_FLAG_SKIP_DNS_SERVER = 0x0008
    ERROR_BUFFER_OVERFLOW = 111
    IF_TYPE_SOFTWARE_LOOPBACK = 24
    IF_OPER_STATUS_UP = 1

    class SockaddrIn(ctypes.Structure):
        _fields_ = [
            ("sin_family", wintypes.USHORT),
            ("sin_port", wintypes.USHORT),
            ("sin_addr", ctypes.c_ubyte * 4),
            ("sin_zero", ctypes.c_char * 8),
        ]

    class SocketAddress(ctypes.Structure):
        _fields_ = [
            ("lpSockaddr", ctypes.POINTER(SockaddrIn)),
            ("iSockaddrLength", ctypes.c_int),
        ]

    class UnicastAddress(ctypes.Structure):
        pass

    # The real struct starts with a union of ULONGLONG against {ULONG; DWORD},
    # which is the same eight bytes either way, so spelling out the two 32-bit
    # members keeps everything after it correctly aligned.
    UnicastAddress._fields_ = [
        ("Length", wintypes.ULONG),
        ("Flags", wintypes.DWORD),
        ("Next", ctypes.POINTER(UnicastAddress)),
        ("Address", SocketAddress),
    ]

    class AdapterAddresses(ctypes.Structure):
        pass

    AdapterAddresses._fields_ = [
        ("Length", wintypes.ULONG),
        ("IfIndex", wintypes.DWORD),
        ("Next", ctypes.POINTER(AdapterAddresses)),
        ("AdapterName", ctypes.c_char_p),
        ("FirstUnicastAddress", ctypes.POINTER(UnicastAddress)),
        ("FirstAnycastAddress", ctypes.c_void_p),
        ("FirstMulticastAddress", ctypes.c_void_p),
        ("FirstDnsServerAddress", ctypes.c_void_p),
        ("DnsSuffix", ctypes.c_wchar_p),
        ("Description", ctypes.c_wchar_p),
        ("FriendlyName", ctypes.c_wchar_p),
        ("PhysicalAddress", ctypes.c_ubyte * 8),
        ("PhysicalAddressLength", wintypes.ULONG),
        ("Flags", wintypes.ULONG),
        ("Mtu", wintypes.ULONG),
        ("IfType", wintypes.ULONG),
        ("OperStatus", wintypes.ULONG),
    ]

    get_adapters = ctypes.windll.iphlpapi.GetAdaptersAddresses
    flags = GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST | GAA_FLAG_SKIP_DNS_SERVER

    size = wintypes.ULONG(15 * 1024)
    for _attempt in range(3):
        buffer = ctypes.create_string_buffer(size.value)
        result = get_adapters(AF_INET, flags, None,
                              ctypes.cast(buffer, ctypes.POINTER(AdapterAddresses)),
                              ctypes.byref(size))
        if result != ERROR_BUFFER_OVERFLOW:
            break
    if result != 0:
        raise OSError(f"GetAdaptersAddresses failed with {result}")

    adapters: List[Adapter] = []
    node = ctypes.cast(buffer, ctypes.POINTER(AdapterAddresses))
    while node:
        entry = node.contents
        if entry.IfType != IF_TYPE_SOFTWARE_LOOPBACK:
            address = entry.FirstUnicastAddress
            while address:
                sockaddr = address.contents.Address.lpSockaddr
                if sockaddr and sockaddr.contents.sin_family == AF_INET:
                    ip = ".".join(str(b) for b in sockaddr.contents.sin_addr)
                    adapters.append(Adapter(entry.FriendlyName or "", ip,
                                            entry.OperStatus == IF_OPER_STATUS_UP))
                address = address.contents.Next
        node = entry.Next
    return adapters


def list_adapters(only_up: bool = False) -> List[Adapter]:
    """This host's IPv4 addresses, named where the platform allows it.

    Never raises: a platform without the Win32 call, or a struct that does not
    match, degrades to bare addresses rather than taking the tool down.
    """
    adapters: List[Adapter] = []
    if sys.platform == "win32":
        try:
            adapters = _list_adapters_windows()
        except Exception:  # noqa: BLE001 - any failure here is non-fatal
            adapters = []
    if not adapters:
        try:
            for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
                ip = info[4][0]
                if all(a.ip != ip for a in adapters):
                    adapters.append(Adapter("", ip, True))
        except socket.gaierror:
            pass
    if only_up:
        adapters = [a for a in adapters if a.is_up]
    return adapters


def source_address_for(destination: str) -> Optional[str]:
    """Which local address a packet to `destination` would leave from.

    Connecting a UDP socket sends nothing; it just asks the kernel to apply the
    routing table and bind a source. That makes this the authoritative answer to
    "which adapter reaches the board", including across a VPN or an overlay
    where the answer differs from the default route.
    """
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # port is arbitrary — connect() on UDP only consults the routing table
        probe.connect((destination, 7400))
        return probe.getsockname()[0]
    except OSError:
        return None
    finally:
        probe.close()


def adapter_for(destination: str) -> Optional[Adapter]:
    """The named adapter that reaches `destination`, if it can be named."""
    ip = source_address_for(destination)
    if ip is None:
        return None
    for adapter in list_adapters():
        if adapter.ip == ip:
            return adapter
    return Adapter("", ip, True)


# ----------------------------------------------------------------- probing


class _Listener(rtps_host.RtpsHostHarness):
    """Throwaway participant that records which peers answer it.

    Carries the same endpoints as the real tool, because that is what makes a
    peer talk back: embeddedRTPS answers our SEDP with heartbeats addressed to
    our locator, and starts streaming its status topic once it has matched.
    Bare SPDP with no endpoints would often draw nothing over a routed link.
    """

    def __init__(self, args, targets: Iterable[str]) -> None:
        super().__init__(args)
        self._targets = set(targets)
        self.responders: List[str] = []
        self.first_response = threading.Event()

    def _note(self, sender_ip: str) -> None:
        # Our own multicast announcement loops back, so only addresses we
        # actually probed count as a find.
        if sender_ip in self._targets and sender_ip not in self.responders:
            self.responders.append(sender_ip)
            self.first_response.set()

    def handle_user_packet(self, packet: bytes, sender_ip: str, sender_port: int) -> None:
        self._note(sender_ip)

    def handle_metatraffic_packet(self, packet: bytes, sender_ip: str) -> None:
        self._note(sender_ip)
        super().handle_metatraffic_packet(packet, sender_ip)


def _listener_args(targets: List[str], advertised: Optional[str],
                   participant_id: int) -> "object":
    import argparse

    address = advertised or (source_address_for(targets[0]) if targets else None) \
        or rtps_host.guess_local_ipv4()
    return argparse.Namespace(
        node_name="rammp_probe",
        domain_id=0,
        participant_id=participant_id,
        bind_address=address,
        advertised_address=address,
        multicast_interface=None,
        multicast_group="239.255.0.1",
        enclave="/",
        # The joystick's endpoints: the MIB streams MibStatus at a matched reader.
        subscribe_topic=[spec.TOPIC_MIB_STATUS],
        subscribe_type_name=spec.TYPE_MIB_STATUS,
        publish_topic=spec.TOPIC_JOYSTICK_XY_TWIST,
        publish_value=0,
        publish_interval=0.0,  # probing only; never publish a stick sample
        echo_received=False,
        reliable=False,
        type_name=spec.TYPE_XY_TWIST,
        announce_period=0.5,
        duration=0.0,
        trace_packets=False,
        quiet=True,
        peer=list(targets),
        peer_participant_ids=[0, 1, 2, 3],
    )


def _run_listener(targets: List[str], timeout: float, advertised: Optional[str],
                  stop_on_first: bool) -> List[str]:
    if not targets:
        return []
    listener = _Listener(_listener_args(targets, advertised, PROBE_PARTICIPANT_ID), targets)
    thread = threading.Thread(target=listener.run, daemon=True)
    thread.start()
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            if stop_on_first and listener.first_response.wait(0.1):
                break
            time.sleep(0.05)
    finally:
        listener.stop()
        thread.join(timeout=2.0)
    return list(listener.responders)


def probe_board(ip: str, timeout: float = 3.0, advertised: Optional[str] = None) -> bool:
    """Is something at `ip` speaking our RTPS domain?"""
    try:
        ipaddress.IPv4Address(ip)
    except ValueError:
        try:
            ip = socket.gethostbyname(ip)
        except socket.gaierror:
            return False
    return bool(_run_listener([ip], timeout, advertised, stop_on_first=True))


def scan_subnet(cidr: str, timeout: float = 6.0, progress: ProgressFn = None) -> List[str]:
    """Sweep a subnet for anything answering RTPS.

    Used when passive discovery finds nothing, which is the routed case: the
    board is reachable but its multicast never gets here. Every host in the
    range is seeded with unicast SPDP at once, and whichever answers is it.
    """
    network = ipaddress.IPv4Network(cidr, strict=False)
    hosts = [str(h) for h in network.hosts()]
    if progress:
        progress(f"scanning {cidr} ({len(hosts)} addresses, up to {timeout:.0f}s)")
    found = _run_listener(hosts, timeout, None, stop_on_first=True)
    if progress:
        progress(f"scan found {found}" if found else "scan found nothing")
    return found


def discover_board(saved_ip: Optional[str] = None, progress: ProgressFn = None) -> Optional[str]:
    """Find the board without scanning: saved address, hostname, then multicast.

    Ordered cheapest first and each candidate is confirmed by an actual
    exchange, so a stale saved address does not get handed back as if it were
    live. Returns None when only a subnet scan would find it.
    """

    def say(message: str) -> None:
        if progress:
            progress(message)

    if saved_ip:
        say(f"trying saved address {saved_ip}")
        if probe_board(saved_ip, timeout=2.5):
            say(f"board found at {saved_ip}")
            return saved_ip
        say(f"no answer from {saved_ip}")

    for hostname in DEFAULT_HOSTNAMES:
        resolved = resolve_hostname(hostname)
        if resolved is None:
            continue
        say(f"'{hostname}' resolves to {resolved}")
        if probe_board(resolved, timeout=2.5):
            say(f"board found at {resolved}")
            return resolved

    say("listening for the board's RTPS announcement")
    for participant in _passive_listen(3.0):
        say(f"heard participant at {participant}")
        return participant
    say("nothing found passively - a subnet scan is the next step")
    return None


def resolve_hostname(hostname: str, timeout: float = 1.5) -> Optional[str]:
    """Resolve a name, giving up after `timeout` rather than blocking on DNS.

    socket.gethostbyname takes no timeout and on Windows can sit for five
    seconds per name when no resolver answers — which is the normal case for a
    bench hostname. Resolving on a daemon thread and abandoning it keeps a cold
    start responsive; the stranded thread dies with the process.
    """
    result: List[str] = []

    def resolve() -> None:
        try:
            result.append(socket.gethostbyname(hostname))
        except socket.gaierror:
            pass

    worker = threading.Thread(target=resolve, daemon=True)
    worker.start()
    worker.join(timeout)
    return result[0] if result else None


def _passive_listen(timeout: float) -> List[str]:
    """Addresses of any participants announcing themselves by multicast."""
    import argparse

    address = rtps_host.guess_local_ipv4()
    args = argparse.Namespace(
        node_name="rammp_listen", domain_id=0, participant_id=PROBE_PARTICIPANT_ID,
        bind_address=address, advertised_address=address, multicast_interface=None,
        multicast_group="239.255.0.1", enclave="/", subscribe_topic=[],
        subscribe_type_name=None, publish_topic=None, publish_value=0,
        publish_interval=0.0, echo_received=False, reliable=False,
        type_name=spec.TYPE_MIB_STATUS, announce_period=0.5, duration=0.0,
        trace_packets=False, quiet=True, peer=None, peer_participant_ids=[],
    )
    try:
        harness = rtps_host.RtpsHostHarness(args)
    except OSError:
        return []
    thread = threading.Thread(target=harness.run, daemon=True)
    thread.start()
    time.sleep(timeout)
    addresses = [p.address for p in harness.discovered_participants.values()
                 if p.address and p.address != address]
    harness.stop()
    thread.join(timeout=2.0)
    return addresses


def subnet_guess(ip: Optional[str]) -> str:
    """A sensible /24 to offer for a scan, given the last known board address."""
    if ip:
        try:
            return str(ipaddress.IPv4Network(f"{ip}/24", strict=False))
        except ValueError:
            pass
    local = rtps_host.guess_local_ipv4()
    try:
        return str(ipaddress.IPv4Network(f"{local}/24", strict=False))
    except ValueError:
        return "192.168.1.0/24"


# ------------------------------------------------------------------ config


def config_path() -> str:
    """Per-user config location, outside the repo so it survives checkouts."""
    if sys.platform == "win32":
        root = os.environ.get("APPDATA") or os.path.expanduser("~")
    else:
        root = os.environ.get("XDG_CONFIG_HOME") or os.path.join(os.path.expanduser("~"),
                                                                 ".config")
    return os.path.join(root, "rammp-rtps", "config.json")


def load_config() -> dict:
    """Remembered settings, or an empty dict. Never raises."""
    try:
        with open(config_path(), encoding="utf-8") as handle:
            data = json.load(handle)
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def save_config(values: dict) -> bool:
    """Merge `values` into the stored config. Returns False if it could not.

    Written to a temporary file and moved into place, so an interrupted write
    leaves the previous config intact rather than a truncated one.
    """
    path = config_path()
    merged = load_config()
    merged.update({k: v for k, v in values.items() if v is not None})
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        temporary = path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as handle:
            json.dump(merged, handle, indent=2, sort_keys=True)
        os.replace(temporary, path)
        return True
    except OSError:
        return False


if __name__ == "__main__":
    print(f"config: {config_path()}")
    stored = load_config()
    print(f"stored: {stored or '(none)'}\n")
    print("adapters:")
    for adapter in list_adapters():
        print(f"  {adapter.label()}")
    target = sys.argv[1] if len(sys.argv) > 1 else stored.get("peer")
    if target:
        print(f"\nroute to {target}: {source_address_for(target)}")
        found = adapter_for(target)
        print(f"adapter: {found.label() if found else '(unknown)'}")
