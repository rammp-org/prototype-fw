#!/usr/bin/env python3
"""Python view of the RAMMP RTPS wire spec, read from the shared C++ headers.

The rammp-rtps submodule's ``messages/{joystick,mib,motor}_message.hpp`` hold the
topics, type names, enums, tables and message structs every RAMMP device shares.
This module parses those three headers at import time and mirrors the message
structs, encoded the way espp/cdr encodes them (XCDR1). Nothing here is typed in
by hand, so a row added to a table or a field appended to a struct shows up in
the bench tool without touching this file.

    import rammp_spec as spec
    spec.TOPIC_MIB_STATUS          # 'rammp/mib/status'
    spec.MIB_SYSTEM_STATE_IDLE     # 1
    spec.SEAT_AXES[2].label        # 'Elevation'
    spec.AXES[2].state_topic       # 'rammp/drive_left/motor_state'
    spec.decode("MibStatus", payload)   # {'systemState': 1, ...} or None
    spec.pack_xy_twist(0.0, 0.5, 0.0, 0)

Names lose the RAMMP_ prefix or become UPPER_SNAKE: ``RAMMP_TOPIC_MIB_STATUS``
gives TOPIC_MIB_STATUS, ``MibSystemState::IDLE`` gives MIB_SYSTEM_STATE_IDLE.

The HMI's own timing (what pace-hmi-fw adds in main/hmi_rtps_spec.hpp) is not in
the submodule, so the values the HMI uses are hard-coded at the bottom.

The same spirit as pace-hmi-fw/scripts/rammp_rtps.py, whose encode/decode is known
to interoperate with espp/cdr; this module derives the layouts from the structs
instead of spelling them out.
"""

from __future__ import annotations

import os
import re
import struct
from typing import Any, Dict, List, Mapping, NamedTuple, Optional, Sequence, Tuple, Union

# ----------------------------------------------------------------- locating

HEADER_DIR_RELATIVE = os.path.join(
    "external", "rammp-rtps", "components", "rammp_rtps_messages", "include", "messages")
HEADER_NAMES = ("joystick_message.hpp", "mib_message.hpp", "motor_message.hpp")


def find_header_dir(start: Optional[str] = None) -> str:
    """Locate the spec headers by walking up from this file to the repo root."""
    directory = os.path.dirname(os.path.abspath(start or __file__))
    while True:
        candidate = os.path.join(directory, HEADER_DIR_RELATIVE)
        if all(os.path.isfile(os.path.join(candidate, name)) for name in HEADER_NAMES):
            return candidate
        parent = os.path.dirname(directory)
        if parent == directory:
            raise FileNotFoundError(
                f"could not find {HEADER_DIR_RELATIVE}/{{{','.join(HEADER_NAMES)}}} above "
                f"{os.path.dirname(os.path.abspath(start or __file__))} - the shared spec is a git "
                "submodule: run `git submodule update --init` in the prototype-fw repo")
        directory = parent


#: the directory holding the three shared headers
HEADER_DIR = find_header_dir()
#: the headers this module was built from, in parse order
HEADER_PATHS: List[str] = [os.path.join(HEADER_DIR, name) for name in HEADER_NAMES]


def _read_header(path: str) -> str:
    with open(path, encoding="utf-8") as handle:
        return handle.read().split("#if 0", 1)[0]  # anything #if 0'd out is not the spec


_RAW_TEXT = "\n".join(_read_header(path) for path in HEADER_PATHS)

_COMMENT_RE = re.compile(r"/\*.*?\*/|//[^\n]*", re.S)
#: the headers with every comment removed (the tables are cut out of the raw text
#: first, because their rows rely on the trailing backslashes comments also carry)
_TEXT = _COMMENT_RE.sub("", _RAW_TEXT)

# -------------------------------------------------------------- strings

# #define RAMMP_TOPIC_MIB_STATUS "rammp/mib/status" / #define RAMMP_TYPE_MIB_STATUS "..."
_DEFINE_RE = re.compile(r'^\s*#define\s+RAMMP_((?:TOPIC|TYPE)_[A-Z0-9_]+)\s+"([^"]*)"', re.M)
# #define RAMMP_TOPIC_MIB_MOTOR_COMMAND(segment) "rammp/mib/motor_command/" segment
_TOPIC_MACRO_RE = re.compile(r"^\s*#define\s+RAMMP_(TOPIC_[A-Z0-9_]+)\((\w+)\)\s+(.+?)\s*$", re.M)
# #define RAMMP_MOTOR_API_VERSION 1
_INT_DEFINE_RE = re.compile(r"^\s*#define\s+RAMMP_([A-Z0-9_]+)\s+(0[xX][0-9a-fA-F]+|\d+)\s*$", re.M)

#: TOPIC_* (topic names) and TYPE_* (DDS type names)
STRINGS: Dict[str, str] = dict(_DEFINE_RE.findall(_TEXT))
#: RAMMP_MOTOR_API_VERSION and any other plain integer #define
NUMBERS: Dict[str, int] = {name: int(value, 0) for name, value in _INT_DEFINE_RE.findall(_TEXT)}


class TopicPattern(NamedTuple):
    """A function-like topic macro: the literal pieces around its one argument."""

    name: str
    prefix: str
    suffix: str

    def __call__(self, segment: str) -> str:
        return f"{self.prefix}{segment}{self.suffix}"


def _topic_pattern(name: str, parameter: str, body: str) -> TopicPattern:
    """'"rammp/" segment "/motor_state"' -> TopicPattern('rammp/', '/motor_state')."""
    pieces = re.findall(r'"([^"]*)"|(\w+)', body)
    prefix, suffix, seen = "", "", False
    for literal, token in pieces:
        if token == parameter:
            seen = True
        elif token:
            raise RuntimeError(f"RAMMP_{name}: unexpected token {token!r} in {body!r}")
        elif seen:
            suffix += literal
        else:
            prefix += literal
    if not seen:
        raise RuntimeError(f"RAMMP_{name}: {parameter!r} not used in {body!r}")
    return TopicPattern(name, prefix, suffix)


#: TOPIC_MIB_MOTOR_COMMAND / TOPIC_MOTOR_STATE: callables taking the axis segment
TOPIC_PATTERNS: Dict[str, TopicPattern] = {
    name: _topic_pattern(name, parameter, body)
    for name, parameter, body in _TOPIC_MACRO_RE.findall(_TEXT)
}

if not STRINGS:
    raise RuntimeError(f"{HEADER_DIR}: no RAMMP_TOPIC_/RAMMP_TYPE_ strings parsed")

globals().update(STRINGS)
globals().update(TOPIC_PATTERNS)
globals().update(NUMBERS)


def motor_command_topic(segment: str) -> str:
    """RAMMP_TOPIC_MIB_MOTOR_COMMAND(segment): MIB -> board."""
    return TOPIC_PATTERNS["TOPIC_MIB_MOTOR_COMMAND"](segment)


def motor_state_topic(segment: str) -> str:
    """RAMMP_TOPIC_MOTOR_STATE(segment): board -> MIB."""
    return TOPIC_PATTERNS["TOPIC_MOTOR_STATE"](segment)


# ---------------------------------------------------------------- tables


def _macro_block(name: str) -> str:
    """The body of `#define name(X)` with its continuation lines, comments removed."""
    lines = _RAW_TEXT.splitlines()
    for index, line in enumerate(lines):
        if re.match(rf"\s*#define\s+{re.escape(name)}\s*\(", line):
            block = [line]
            while block[-1].rstrip().endswith("\\") and index + len(block) < len(lines):
                block.append(lines[index + len(block)])
            return _COMMENT_RE.sub("", "\n".join(block))
    return ""


class SeatAxis(NamedTuple):
    """One row of RAMMP_SEAT_AXIS_TABLE: raw integer values, `decimals` for display."""

    id: int
    name: str
    short: str
    label: str
    min_value: int
    max_value: int
    step: int
    decimals: int
    unit: str

    def format(self, raw: int) -> str:
        """Raw units as the HMI's row draws them, e.g. 126 -> '12.6'."""
        return f"{raw / (10 ** self.decimals):.{self.decimals}f}"

    def parse(self, text: str) -> int:
        """Inverse of format(): '12.6' -> 126, rounded. NOT clamped to the row: the
        row is the HMI's jog range, the MIB's own limits decide (clamp() is there
        for callers that want the row)."""
        return int(round(float(text) * (10 ** self.decimals)))

    def clamp(self, raw: int) -> int:
        return max(self.min_value, min(self.max_value, raw))

    def to_units(self, raw: int) -> float:
        """Raw table units -> the whole units the wire carries (250 -> 25.0 for 1 decimal)."""
        return raw / (10 ** self.decimals)

    def to_raw(self, units: float) -> int:
        """Whole wire units -> raw table units, rounded (25.04 -> 250 for 1 decimal)."""
        return int(round(units * (10 ** self.decimals)))


# X(0, ELEVATION, "M1", "Elevation", 0, 2500, 50, 1, "mm")
_SEAT_AXIS_RE = re.compile(
    r"""X\(\s*(\d+)\s*,\s*([A-Z0-9_]+)\s*,\s*"([^"]*)"\s*,\s*"([^"]*)"\s*,"""
    r"""\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(\d+)\s*,\s*"([^"]*)"\s*\)""")

#: every seat axis in the shared X-macro table, in wire-id order
SEAT_AXES: List[SeatAxis] = [
    SeatAxis(int(i), name, short, label, int(lo), int(hi), int(step), int(dec), unit)
    for i, name, short, label, lo, hi, step, dec, unit
    in _SEAT_AXIS_RE.findall(_macro_block("RAMMP_SEAT_AXIS_TABLE"))
]
if not SEAT_AXES:
    raise RuntimeError(f"{HEADER_DIR}: RAMMP_SEAT_AXIS_TABLE parsed to nothing")
if [a.id for a in SEAT_AXES] != list(range(len(SEAT_AXES))):
    raise RuntimeError(f"{HEADER_DIR}: seat axis ids must be 0..N-1 in table order")


class DiagItem(NamedTuple):
    """One row of RAMMP_DIAG_TABLE: up to three readings, each with its unit."""

    id: int
    name: str
    short: str
    label: str
    units: Tuple[str, ...]  # three unit labels; "" = reading unused
    decimals: Tuple[int, ...]  # three decimal counts, display only

    def format(self, raw: int, field: int) -> str:
        """A raw reading as the HMI draws it, e.g. 2345 with 2 decimals -> '23.45'."""
        places = self.decimals[field]
        return f"{raw / (10 ** places):.{places}f}"


# D(0, TEST_1, "T1", "Test actuator 1", "Temp [C]", 1, "Current [A]", 2, "Pos [deg]", 1)
_DIAG_RE = re.compile(
    r"""D\(\s*(\d+)\s*,\s*([A-Z0-9_]+)\s*,\s*"([^"]*)"\s*,\s*"([^"]*)"\s*,"""
    r"""\s*"([^"]*)"\s*,\s*(\d+)\s*,\s*"([^"]*)"\s*,\s*(\d+)\s*,\s*"([^"]*)"\s*,\s*(\d+)\s*\)""")

#: every item in the shared diagnostics table, in wire order
DIAGNOSTICS: List[DiagItem] = [
    DiagItem(int(i), name, short, label, (u1, u2, u3), (int(d1), int(d2), int(d3)))
    for i, name, short, label, u1, d1, u2, d2, u3, d3
    in _DIAG_RE.findall(_macro_block("RAMMP_DIAG_TABLE"))
]
if not DIAGNOSTICS:
    raise RuntimeError(f"{HEADER_DIR}: RAMMP_DIAG_TABLE parsed to nothing")
if [d.id for d in DIAGNOSTICS] != list(range(len(DIAGNOSTICS))):
    raise RuntimeError(f"{HEADER_DIR}: diagnostics ids must be 0..N-1 in table order")


class Axis(NamedTuple):
    """One row of RAMMP_AXIS_TABLE (a PACE RACER board, or a CAN motor the MIB fronts)."""

    id: int
    name: str
    segment: str
    label: str
    command_topic: str  #: RAMMP_TOPIC_MIB_MOTOR_COMMAND(segment), MIB -> board
    state_topic: str  #: RAMMP_TOPIC_MOTOR_STATE(segment), board -> MIB


# X(2, DRIVE_LEFT, "drive_left", "Drive L")
_AXIS_RE = re.compile(r"""X\(\s*(\d+)\s*,\s*([A-Z0-9_]+)\s*,\s*"([^"]*)"\s*,\s*"([^"]*)"\s*\)""")

#: every motor axis in the shared table, in id order, with its two topics
AXES: List[Axis] = [
    Axis(int(i), name, segment, label, motor_command_topic(segment), motor_state_topic(segment))
    for i, name, segment, label in _AXIS_RE.findall(_macro_block("RAMMP_AXIS_TABLE"))
]
if not AXES:
    raise RuntimeError(f"{HEADER_DIR}: RAMMP_AXIS_TABLE parsed to nothing")
if [a.id for a in AXES] != list(range(len(AXES))):
    raise RuntimeError(f"{HEADER_DIR}: axis ids must be 0..N-1 in table order")

#: table name (as in RAMMP_<name>_TABLE) -> (NAME, id) rows, for the X-macro enums
_TABLE_IDS: Dict[str, List[Tuple[str, int]]] = {
    "SEAT_AXIS": [(a.name, a.id) for a in SEAT_AXES],
    "DIAG": [(d.name, d.id) for d in DIAGNOSTICS],
    "AXIS": [(a.name, a.id) for a in AXES],
}

# ----------------------------------------------------------------- enums

# enum class MibSystemState : uint8_t { INITIALIZING, IDLE = 1, ... };
_ENUM_RE = re.compile(r"enum class (\w+)\s*:\s*(\w+)\s*\{(.*?)\}\s*;", re.S)
_MEMBER_RE = re.compile(r"^([A-Za-z_][A-Za-z0-9_]*)(?:\s*=\s*(0[xX][0-9a-fA-F]+|\d+))?$")
_TABLE_CALL_RE = re.compile(r"^RAMMP_(\w+)_TABLE\(\w+\)$")


def _enumerators(body: str) -> List[Tuple[str, int]]:
    """(NAME, value) for one enum body, counted as C++ counts: a bare enumerator is
    the previous one + 1. An enum built from an X-macro table takes its members
    from the table rows (the ids are explicit there)."""
    body = "\n".join(line for line in body.splitlines() if not line.strip().startswith("#"))
    members: List[Tuple[str, int]] = []
    value = 0
    for segment in body.split(","):
        segment = segment.strip()
        if not segment:
            continue
        table = _TABLE_CALL_RE.match(segment)
        if table:
            rows = _TABLE_IDS.get(table.group(1))
            if rows is None:
                raise RuntimeError(f"enum uses unknown table RAMMP_{table.group(1)}_TABLE")
            members.extend(rows)
            value = rows[-1][1] + 1 if rows else value
            continue
        member = _MEMBER_RE.match(segment)
        if not member:
            continue
        if member.group(2) is not None:
            value = int(member.group(2), 0)
        members.append((member.group(1), value))
        value += 1
    return members


def _snake(name: str) -> str:
    """SystemState -> SYSTEM_STATE, XYTwist -> XY_TWIST, AxisId -> AXIS_ID."""
    return re.sub(r"(?<=[a-z0-9])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])", "_", name).upper()


#: enum name -> its underlying C++ type ('uint8_t'), which is its wire width
ENUM_UNDERLYING: Dict[str, str] = {}
#: enum name -> {MEMBER: value}
ENUM_VALUES: Dict[str, Dict[str, int]] = {}
#: enum name -> {value: MEMBER}
ENUM_NAMES: Dict[str, Dict[int, str]] = {}
#: every enumerator flattened, as ENUM_MEMBER (MIB_SYSTEM_STATE_IDLE)
ENUMS: Dict[str, int] = {}

for _enum, _underlying, _body in _ENUM_RE.findall(_TEXT):
    ENUM_UNDERLYING[_enum] = _underlying
    _members = _enumerators(_body)
    ENUM_VALUES[_enum] = dict(_members)
    ENUM_NAMES[_enum] = {value: name for name, value in _members}
    for _name, _value in _members:
        ENUMS[f"{_snake(_enum)}_{_name}"] = _value

if not ENUMS:
    raise RuntimeError(f"{HEADER_DIR}: no enums parsed - has the C++ style changed?")

globals().update(ENUMS)


def enum_name(enum: str, value: int) -> str:
    """'IDLE' for MibSystemState 1; '?<n>' for a value the header does not name."""
    return ENUM_NAMES.get(enum, {}).get(value, f"?{value}")


#: {0: 'INITIALIZING', 1: 'IDLE', 2: 'ENABLED', 3: 'ERROR'} - MIB::MibSystemState
MIB_SYSTEM_STATE_NAMES: Dict[int, str] = ENUM_NAMES.get("MibSystemState", {})
#: {0: 'LOW', 1: 'NORMAL', 2: 'HIGH'} - MIB::DriveProfile
DRIVE_PROFILE_NAMES: Dict[int, str] = ENUM_NAMES.get("DriveProfile", {})
#: {0: 'DISABLE', 1: 'ENABLE'} - rammp::DriveRequest
DRIVE_REQUEST_NAMES: Dict[int, str] = ENUM_NAMES.get("DriveRequest", {})
#: rammp::BoardState / ControlMode / FaultCode / RequestedState (motor_message.hpp)
BOARD_STATE_NAMES: Dict[int, str] = ENUM_NAMES.get("BoardState", {})
CONTROL_MODE_NAMES: Dict[int, str] = ENUM_NAMES.get("ControlMode", {})
FAULT_CODE_NAMES: Dict[int, str] = ENUM_NAMES.get("FaultCode", {})
REQUESTED_STATE_NAMES: Dict[int, str] = ENUM_NAMES.get("RequestedState", {})
#: {0x1: 'JOYSTICK'} - rammp::Buttons, a bit set (NONE left out)
BUTTON_NAMES: Dict[int, str] = {
    value: name for value, name in ENUM_NAMES.get("Buttons", {}).items() if value}

# --------------------------------------------------------------- structs
#
# A layout is a list of (field name, kind). A kind is a struct format char ('B',
# 'f', 'q', ...), "str", ("seq", element kind), ("arr", count, element kind) or
# a nested layout (a struct, written inline).

Kind = Union[str, tuple, list]
Layout = List[Tuple[str, Kind]]

_PRIMITIVES: Dict[str, str] = {
    "uint8_t": "B", "int8_t": "b", "uint16_t": "H", "int16_t": "h",
    "uint32_t": "I", "int32_t": "i", "uint64_t": "Q", "int64_t": "q",
    "float": "f", "double": "d", "bool": "?", "char": "b",
}
_MEMBER_DECL_RE = re.compile(r"^([\w:]+(?:\s*<[^<>]*>)?)\s+(\w+)\s*(?:\{[^{}]*\})?$")
_VECTOR_RE = re.compile(r"^std::vector<(.+)>$")
_ARRAY_RE = re.compile(r"^std::array<(.+),(\d+)>$")


def _struct_bodies(text: str) -> Dict[str, str]:
    """{name: body} for every `struct Name { ... }`, by brace matching (member
    initialisers like `float x{0.0f};` contain `};`, so a regex will not do)."""
    bodies: Dict[str, str] = {}
    for match in re.finditer(r"\bstruct\s+(\w+)\s*\{", text):
        depth, index = 1, match.end()
        while depth and index < len(text):
            depth += (text[index] == "{") - (text[index] == "}")
            index += 1
        if depth == 0:
            bodies[match.group(1)] = text[match.end():index - 1]
    return bodies


_STRUCT_BODIES: Dict[str, str] = _struct_bodies(_TEXT)
_LAYOUTS: Dict[str, Layout] = {}


def _kind(type_text: str, owner: str) -> Kind:
    text = re.sub(r"\s+", "", type_text)
    if text in _PRIMITIVES:
        return _PRIMITIVES[text]
    if text == "std::string":
        return "str"
    vector = _VECTOR_RE.match(text)
    if vector:
        return ("seq", _kind(vector.group(1), owner))
    array = _ARRAY_RE.match(text)
    if array:
        return ("arr", int(array.group(2)), _kind(array.group(1), owner))
    base = text.rsplit("::", 1)[-1]
    if base in ENUM_UNDERLYING:
        return _PRIMITIVES[ENUM_UNDERLYING[base]]
    if base in _STRUCT_BODIES:
        return layout(base)
    raise ValueError(f"struct {owner}: no wire encoding for member type {type_text!r}")


def layout(name: str) -> Layout:
    """The wire layout of `struct name`, fields in declaration order."""
    cached = _LAYOUTS.get(name)
    if cached is not None:
        return cached
    body = _STRUCT_BODIES.get(name)
    if body is None:
        raise KeyError(f"no struct {name} in {HEADER_DIR}")
    fields: Layout = []
    for declaration in body.split(";"):
        declaration = " ".join(declaration.split())
        if not declaration:
            continue
        member = _MEMBER_DECL_RE.match(declaration)
        if not member:
            raise ValueError(f"struct {name}: cannot read member {declaration!r}")
        fields.append((member.group(2), _kind(member.group(1), name)))
    if not fields:
        raise ValueError(f"struct {name} has no members")
    _LAYOUTS[name] = fields
    return fields


#: the structs that travel on a topic; parsed now so a header change fails loudly
MESSAGES: Tuple[str, ...] = (
    "MibStatus", "XYTwist", "DriveCommand", "SeatCommand", "Diagnostics",
    "MotorCommand", "MotorState",
)
for _message in MESSAGES:
    layout(_message)


def flat_layout(name: str) -> Layout:
    """`layout` with nested structs inlined (seatState's floats appear as MibStatus
    fields), the way rammp_rtps.py spells MSG_MIB_STATUS."""
    flat: Layout = []
    for field, kind in layout(name):
        if isinstance(kind, list):
            flat.extend(kind)
        else:
            flat.append((field, kind))
    return flat


def defaults(kind: Kind) -> Any:
    """A zero value of `kind` (a dict for a struct), e.g. defaults(layout('XYTwist'))."""
    if isinstance(kind, list):
        return {field: defaults(field_kind) for field, field_kind in kind}
    if isinstance(kind, tuple):
        if kind[0] == "arr":
            return [defaults(kind[2]) for _ in range(kind[1])]
        return []
    if kind == "str":
        return ""
    return False if kind == "?" else (0.0 if kind in "fd" else 0)


# ------------------------------------------------------------- CDR (XCDR1)
# What espp/cdr puts on the wire for the spec's C++ structs: header 00 01 00 00,
# little-endian, each primitive aligned to its size counted from the first
# payload byte (after the header), string = uint32 length (NUL included) + bytes
# + NUL, sequence = uint32 count + elements, std::array = elements only, a nested
# struct = its fields inline.

#: 4-byte CDR encapsulation header: little-endian classic CDR (xcdr1)
CDR_LE_HEADER = b"\x00\x01\x00\x00"


def _write(out: bytearray, kind: Kind, value: Any) -> None:
    if isinstance(kind, list):  # a struct: value is a mapping (missing fields = zero)
        for field, field_kind in kind:
            _write(out, field_kind, value.get(field, defaults(field_kind)))
    elif isinstance(kind, tuple):
        if kind[0] == "arr":
            if len(value) != kind[1]:
                raise ValueError(f"std::array expects {kind[1]} elements, got {len(value)}")
            for element in value:
                _write(out, kind[2], element)
        else:  # ("seq", element)
            _write(out, "I", len(value))
            for element in value:
                _write(out, kind[1], element)
    elif kind == "str":
        data = str(value).encode("utf-8")  # XCDR1 std::string carries UTF-8 as-is
        _write(out, "I", len(data) + 1)
        out.extend(data + b"\0")
    else:
        size = struct.calcsize(kind)
        out.extend(b"\0" * (-len(out) % size))
        out.extend(struct.pack("<" + kind, value))


def _read(buf: bytes, pos: int, kind: Kind) -> Tuple[Any, int]:
    """(value, position after it)."""
    if isinstance(kind, list):
        values: Dict[str, Any] = {}
        for field, field_kind in kind:
            values[field], pos = _read(buf, pos, field_kind)
        return values, pos
    if isinstance(kind, tuple):
        if kind[0] == "arr":
            count, element = kind[1], kind[2]
        else:
            count, pos = _read(buf, pos, "I")
            element = kind[1]
            if count > len(buf) - pos:
                raise ValueError("sequence runs past the end")
        items = []
        for _ in range(count):
            item, pos = _read(buf, pos, element)
            items.append(item)
        return items, pos
    if kind == "str":
        length, pos = _read(buf, pos, "I")
        if pos + length > len(buf):
            raise ValueError("string runs past the end")
        text = buf[pos:pos + length].split(b"\0", 1)[0].decode("utf-8", "replace")
        return text, pos + length
    size = struct.calcsize(kind)
    pos += -pos % size
    if pos + size > len(buf):
        raise ValueError("field runs past the end")
    return struct.unpack_from("<" + kind, buf, pos)[0], pos + size


def encode(message: str, values: Mapping[str, Any]) -> bytes:
    """One `struct message` sample: CDR_LE_HEADER + its XCDR1 payload. `values` is a
    mapping by field name; a nested struct is a mapping too; missing fields are zero."""
    out = bytearray()
    _write(out, layout(message), values)
    return CDR_LE_HEADER + bytes(out)


def decode(message: str, payload: bytes) -> Optional[Dict[str, Any]]:
    """The sample's fields by name (nested structs as dicts), or None if it does not
    decode as a `struct message`."""
    if len(payload) < len(CDR_LE_HEADER) or payload[:2] != CDR_LE_HEADER[:2]:
        return None
    try:
        return _read(payload[len(CDR_LE_HEADER):], 0, layout(message))[0]
    except (struct.error, ValueError):
        return None


# ------------------------------------------------------- typed helpers


def pack_xy_twist(x: float, y: float, twist: float, buttons: int = 0) -> bytes:
    """A rammp::XYTwist (joystick -> MIB): axes -1..+1, + = right / forward / clockwise."""
    return encode("XYTwist", {"x": float(x), "y": float(y), "twist": float(twist),
                              "buttons": buttons & 0xFFFFFFFF})


def unpack_xy_twist(payload: bytes) -> Optional[Dict[str, Any]]:
    return decode("XYTwist", payload)


def pack_drive_command(request: int, profile: int) -> bytes:
    """A rammp::DriveCommand (joystick -> MIB): DriveRequest + MIB::DriveProfile."""
    return encode("DriveCommand", {"request": request & 0xFF, "profile": profile & 0xFF})


def unpack_drive_command(payload: bytes) -> Optional[Dict[str, Any]]:
    return decode("DriveCommand", payload)


def pack_seat_command(axis: int, target: float) -> bytes:
    """A rammp::SeatCommand (joystick -> MIB): put `axis` at `target`, in whole units."""
    return encode("SeatCommand", {"axis": axis & 0xFF, "target": float(target)})


def unpack_seat_command(payload: bytes) -> Optional[Dict[str, Any]]:
    return decode("SeatCommand", payload)


def unpack_mib_status(payload: bytes) -> Optional[Dict[str, Any]]:
    """A MIB::MibStatus as a dict; currentSeatState is a dict in seat-table order."""
    return decode("MibStatus", payload)


def seat_values(status: Mapping[str, Any]) -> List[float]:
    """The seatState floats of a decoded MibStatus in RAMMP_SEAT_AXIS_TABLE order (the
    table's rows are that struct's fields in that order, so index = axis id)."""
    seat = status.get("currentSeatState") or {}
    return [float(v) for v in seat.values()]


def pack_mib_status(system_state: int, profile: int = 0, seat: Sequence[float] = (),
                    seq: int = 0, speed: float = 0.0, status_text: str = "",
                    error_message: str = "", error_footer: str = "", epoch_s: int = 0,
                    utc_offset_min: int = 0) -> bytes:
    """A MIB::MibStatus, for tests of the tools (the bench never sends one)."""
    seat_fields = [field for field, _kind in layout("seatState")]
    seat_dict = {field: float(value) for field, value in zip(seat_fields, seat)}
    return encode("MibStatus", {
        "systemState": system_state & 0xFF, "activeProfile": profile & 0xFF,
        "currentSeatState": seat_dict, "error_message": error_message,
        "epoch_s": epoch_s, "speed": float(speed), "utc_offset_min": utc_offset_min,
        "seq": seq & 0xFF, "status_text": status_text, "error_footer": error_footer,
    })


def pack_diagnostics(values: Sequence[Sequence[int]], seq: int = 0) -> bytes:
    """A rammp::Diagnostics; `values` = one list of raw readings per table item."""
    return encode("Diagnostics", {
        "seq": seq & 0xFF, "items": [{"values": [int(v) for v in row]} for row in values]})


def unpack_diagnostics(payload: bytes) -> Optional[Tuple[int, List[List[int]]]]:
    """(seq, readings per item), or None."""
    message = decode("Diagnostics", payload)
    return None if message is None else (message["seq"], [i["values"] for i in message["items"]])


def unpack_motor_state(payload: bytes) -> Optional[Dict[str, Any]]:
    return decode("MotorState", payload)


def pack_motor_state(values: Mapping[str, Any]) -> bytes:
    """A rammp::MotorState, for tests of the tools (the bench never sends one)."""
    return encode("MotorState", values)


# ---------------------------------------------------- HMI-side constants
# From pace-hmi-fw/main/hmi_rtps_spec.hpp, which is the HMI's own and not in the
# shared submodule. These are what the real joystick does, so the bench does the
# same.

#: the MIB sends MibStatus this often
MIB_STATUS_PERIOD_MS = 500
#: no MibStatus for this long = link lost
MIB_STATUS_TIMEOUT_MS = 2000
#: the MIB sends Diagnostics this often
DIAG_PERIOD_MS = 500
#: no Diagnostics for this long = stale, red
DIAG_TIMEOUT_MS = 2000
#: the HMI shows up to len-1 chars of status_text / error_message / error_footer
MCB_TEXT_LEN = 16
ERROR_TEXT_LEN = 64
ERROR_FOOTER_LEN = 32
#: a MotorState topic with nothing for this long is shown stale (boards publish at 20 Hz)
MOTOR_STATE_TIMEOUT_MS = 1000

#: mph per metre per second: MibStatus.speed is m/s, the HMI's dial says mph
MPH_PER_MPS = 2.236936
#: rpm per radian per second
RPM_PER_RAD_S = 60.0 / (2.0 * 3.141592653589793)


def describe() -> str:
    """Everything parsed, one block of text (what `python3 rammp_spec.py` prints)."""
    lines = ["spec headers:"] + [f"  {path}" for path in HEADER_PATHS]
    lines.append("\ntopics and types:")
    lines += [f"  {key:<28} {STRINGS[key]}" for key in sorted(STRINGS)]
    lines += [f"  {name:<28} {p.prefix!r} + segment + {p.suffix!r}"
              for name, p in sorted(TOPIC_PATTERNS.items())]
    lines.append("\nnumbers:")
    lines += [f"  {key:<28} {NUMBERS[key]}" for key in sorted(NUMBERS)]
    lines.append("\nenums:")
    for enum in ENUM_VALUES:
        members = ", ".join(f"{n}={v}" for n, v in ENUM_VALUES[enum].items())
        lines.append(f"  {enum} : {ENUM_UNDERLYING[enum]} {{ {members} }}")
    lines.append("\nseat axes:")
    for axis in SEAT_AXES:
        lines.append(f"  {axis.id:>2} {axis.short:<4} {axis.label:<14} "
                     f"{axis.format(axis.min_value):>7} .. {axis.format(axis.max_value):<7} "
                     f"step {axis.format(axis.step)} {axis.unit}")
    lines.append("\ndiagnostics:")
    for item in DIAGNOSTICS:
        units = ", ".join(f"{u} ({d} dp)" for u, d in zip(item.units, item.decimals) if u)
        lines.append(f"  {item.id:>2} {item.short:<4} {item.label:<16} {units}")
    lines.append("\nmotor axes:")
    for axis in AXES:
        lines.append(f"  {axis.id:>2} {axis.label:<16} {axis.state_topic}")
    lines.append("\nmessage layouts (XCDR1, after the 4-byte header):")
    for message in MESSAGES:
        fields = ", ".join(f"{field}:{_describe_kind(kind)}" for field, kind in layout(message))
        lines.append(f"  {message}: {fields}")
    return "\n".join(lines)


def _describe_kind(kind: Kind) -> str:
    if isinstance(kind, list):
        return "{" + ",".join(f"{f}:{_describe_kind(k)}" for f, k in kind) + "}"
    if isinstance(kind, tuple):
        if kind[0] == "arr":
            return f"{_describe_kind(kind[2])}[{kind[1]}]"
        return f"seq<{_describe_kind(kind[1])}>"
    return kind


if __name__ == "__main__":
    print(describe())
