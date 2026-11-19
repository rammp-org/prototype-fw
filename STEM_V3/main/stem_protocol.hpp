#pragma once

// STEM linkage USB protocol -- message ids + payload helpers layered on the espp
// `stream_frame` codec (magic "OT" + flags u8 + module u8 + type u8 + len u32 +
// payload + CRC-32, all little-endian; see stream_frame.hpp for the framing
// spec). Host-testable: no ESP-IDF dependencies.
//
// The STEM protocol occupies dispatcher MODULE 7. Firmware update and crash-dump
// inspection are NOT part of it: the firmware runs the standard espp OTA protocol
// on module 0 and the coredump service on module 4 (routed by the same
// espp::Dispatcher) so the stock ota / coredump / Device Hub web consoles keep
// working. The `type` byte carries the message id; request types (host->device)
// clear the frame reply flag and reply/telemetry types (high bit set,
// device->host) set it -- build() derives the flag from the type.
//
//   0x10..0x2F  host -> device  commands
//   0x81..0x8F  device -> host  generic replies (OK / ERROR)
//   0x90..0xAF  device -> host  STEM replies + telemetry
//
// Every generic reply starts with the request type it answers (u8) so the host
// can attribute OK / ERROR to a command without correlation ids (the protocol
// is one-command-in-flight per transport). Units: millimeters in the analysis
// frame relative to the IK reference pose ("x_rel"/"y_rel"), degrees. The `rpm`
// fields are accepted for wire compatibility but ignored: EyouMotor moves at a
// pre-configured profile velocity (see EyouMotor::configure_profile_position),
// not a per-call speed.

#include <algorithm>
#include <bit>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

#include "stream_frame.hpp"

namespace stem_proto {

namespace stream = espp::stream_frame;

/// Dispatcher module id owned by the STEM protocol (the frame `module` byte).
static constexpr uint8_t kModule = 7;

/// Protocol version reported in the INFO reply. v2 (2026-11): SET_TORQUE_LIMIT, RESET_FAULT,
/// the torque limit + ceiling in INFO, torque / velocity per motor and a CAN
/// controller block appended to STATE / TELEMETRY (a v1 host that stops reading
/// after the motor records still decodes everything it knew).
/// v3 (2026-11): output torque sensor block (0x27BD, -F joints) appended to
/// STATE / TELEMETRY after the CAN block (a v2 host stops before it).
/// v4 (2026-11): NUDGE / SET_MOTOR_ZERO + NUDGE_RESULT (pair balance bring-up tool).
static constexpr uint8_t kProtocolVersion = 4;

/// Actuator pair indices used in payloads.
enum class Pair : uint8_t { M1_Seat = 0, M2_Seat = 1, M3_Seat = 2 };
static constexpr uint8_t kPairCount = 3;
/// Wildcard pair index accepted by ZERO.
static constexpr uint8_t kAllPairs = 0xFF;

static constexpr uint8_t kMotorCount = 6;

/// Bounds applied to SET_STREAMING periods (ms).
static constexpr uint16_t kMinStreamPeriodMs = 50;
static constexpr uint16_t kMaxStreamPeriodMs = 5000;
static constexpr uint16_t kDefaultStreamPeriodMs = 200;

enum class Msg : uint8_t {
  // --- Commands (host -> device) --------------------------------------------
  GetInfo = 0x10,      ///< no payload -> Info
  GetState = 0x11,     ///< no payload -> State
  MoveTo = 0x12,       ///< f32 x_rel, f32 y_rel, f32 rpm (0 = default) -> Moved | Error
  SetPair = 0x13,      ///< u8 pair, f32 degrees, f32 rpm (0 = default) -> Ok(0) | Error
  Home = 0x14,         ///< f32 rpm (0 = default) -> Moved | Error
  Stop = 0x15,         ///< no payload -> Ok(0) | Error
  Release = 0x16,      ///< no payload -> Ok(0) | Error  (release all brakes)
  Zero = 0x17,         ///< u8 pair (kAllPairs = all), u8 align -> Ok(0)
  SetStreaming = 0x18, ///< u8 enable, u16 period_ms -> Ok(period_ms)
  SolveIk = 0x19,      ///< f32 x_rel, f32 y_rel -> IkResult (no motion)
  SetSeatTilt =
      0x1A, ///< f32 tilt deg (seat-pair offset on the IK level angle), f32 rpm -> Ok(0) | Error
  SetTorqueLimit = 0x1B, ///< u16 permille of rated torque, every motor (<= the INFO ceiling) ->
  ResetFault = 0x1C, ///< no payload: clear the DS402 fault of every faulted motor and re-enable its
                     ///< pair -> Ok(0) | Error
  SetMeasure = 0x1D, ///< u8 enable, u8 drive mode while measuring (13 CSF / 1 PP; optional, 13)
                     ///< -> Ok(mode) | Error. Measure mode: drives disabled, brakes hold the
                     ///< pose, the output torque sensors (0x27BD) read the load; moves refused.
  Engage = 0x1E, ///< no payload: stop, drives off so the brakes hold; moves refused until Release
                 ///< -> Ok(0) | Error
  // Pair balance (temporary bring-up tool, see StemController::nudge):
  Nudge = 0x1F, ///< u8 motor id, f32 step deg (|step| <= 0.1; 0 = measure only), u8 flags
                ///< (bit0 clear a latched red flag, bit1 reset the step totals) -> NudgeResult | Error
  SetMotorZero = 0x20, ///< u8 motor id: re-zero it to mirror its pair partner, save to NVS
                       ///< -> Ok(f32 bits: how far its reading moved, deg) | Error
                     ///< Ok(permille) | Error
  // --- Generic replies (device -> host) --------------------------------------
  Ok = 0x81,    ///< u8 request type, u32 value
  Error = 0x82, ///< u8 request type, u32 code (std::errc), utf8 message
  // --- STEM replies / telemetry ----------------------------------------------
  Info = 0x90,      ///< see Info layout below
  State = 0x91,     ///< see State layout below (answer to GetState)
  Moved = 0x92,     ///< f32 x_rel, f32 y_rel, f32 left_deg, f32 right_deg, f32 seat_deg
  IkResult = 0x93,  ///< see IkResult layout below
  Telemetry = 0x94, ///< same layout as State (periodic while streaming)
  NudgeResult = 0x95, ///< u8 status (0 ok, 1 refused, 2 red flag - step undone), u8 motor id,
                      ///< f32 step deg, f32 motor's step total deg, u8 motor count; per motor:
                      ///< u8 id, f32 sensor Nm before, f32 after, f32 position deg before, f32 after;
                      ///< then a utf8 message (rest of the payload)
};

// Info payload:
//   u8  protocol version
//   str project, str version, str build (date time), str idf version
//   u8  pair count; per pair: u8 index, str name, u8 primary motor id,
//       u8 secondary motor id, f32 min deg, f32 max deg
//   f32 x 13  GeometryConfig fields in declaration order (m1_x .. crank_length_m2_to_j1)
//   f32 x 5   IkReference fields (x, y, m1_angle_deg, m2_angle_deg, m3_angle_deg)
//   f32 default move rpm, f32 default set rpm, f32 max rpm
//   u16 stream period ms, u8 streaming (on this transport)
//   v2: u16 torque limit permille (what every motor's 0x6072 is set to),
//       u16 torque limit ceiling permille (menuconfig; SET_TORQUE_LIMIT above it is refused),
//       u32 rated torque mNm (0x6076 of the first motor that answered; 0 = not
//           read yet) - the scale of every per-mille torque figure, so a host
//           shows and sets the limit in Nm
//
// State / Telemetry payload:
//   u32 uptime ms
//   u8  flags (see flags:: below)
//   f32 target x_rel, f32 target y_rel      (last accepted MOVE/HOME; 0 if !kTargetValid)
//   f32 pose x_rel, f32 pose y_rel          (FK of the tracked crank angles; 0 if !kPoseValid)
//   f32 M1_Seat deg, f32 M2_Seat deg, f32 M3_Seat deg (tracked pair positions, primary motor)
//   f32 seat tilt deg                       (offset applied on top of the IK seat angle)
//   u8  motor count; per motor: u8 id, u8 ok, u16 statusword, u16 error_code,
//       f32 tracked position deg (read live over CANopen, object 0x6064),
//       v2: i16 torque permille of rated (0x6077), f32 velocity deg/s (0x606C)
//   v2 CAN controller block (CanBus::get_status, the quickest wiring / bitrate
//   check): u8 can state (see can_state:: below), u16 TEC, u16 REC,
//       u32 bus errors since start, u32 bus-off count since start,
//       u32 stuff+form+bit errors (signal integrity), u32 ack errors (nobody heard us)
//   v3 torque sensor block, one record per motor in the same order as the motor
//   records: u8 sensor ok, f32 output torque Nm (0x27BD; 0 when !ok - a non -F
//   joint, or a read that failed)
//
// IkResult payload:
//   f32 x_rel, f32 y_rel (echo), u8 reachable, u8 within limits
//   if reachable: f32 M1_Seat deg, f32 M2_Seat deg, f32 M3_Seat deg,
//                 f32 x 10 joints J1, J2, J3, J5, M3 (absolute analysis frame)

namespace flags {
static constexpr uint8_t kStreaming = 1 << 0;   ///< telemetry streaming on this transport
static constexpr uint8_t kTargetValid = 1 << 1; ///< a MOVE/HOME target has been accepted
static constexpr uint8_t kPoseValid = 1 << 2;   ///< FK of the tracked angles succeeded
static constexpr uint8_t kMotorsOk = 1 << 3;    ///< every motor answered its status poll
static constexpr uint8_t kCanOk = 1 << 4;       ///< the CAN bus started
static constexpr uint8_t kCanBusOff = 1
                                      << 5; ///< v2: the controller left the bus (see the CAN block)
static constexpr uint8_t kMeasuring = 1 << 6; ///< measure mode: brakes hold, sensors read
static constexpr uint8_t kBrakesEngaged = 1 << 7; ///< Engage: drives off until Release
} // namespace flags

/// v2: the CAN controller's error state (TWAI), as the CAN block's first byte.
namespace can_state {
static constexpr uint8_t kNotStarted = 0;
static constexpr uint8_t kErrorActive = 1;  ///< normal
static constexpr uint8_t kErrorWarning = 2; ///< TEC or REC above 96
static constexpr uint8_t kErrorPassive = 3; ///< above 127: this node no longer flags errors
static constexpr uint8_t kBusOff = 4; ///< left the bus; recovery is automatic, or CLI can_recover
static constexpr uint8_t kUnknown = 0xFF;
} // namespace can_state

// ---------------------------------------------------------------------------
// Little-endian payload append / read helpers
// ---------------------------------------------------------------------------

using stream::get_u32;
using stream::put_u16;
using stream::put_u32;

inline void put_i16(std::vector<uint8_t> &out, int16_t value) {
  put_u16(out, static_cast<uint16_t>(value));
}

inline void put_i32(std::vector<uint8_t> &out, int32_t value) {
  put_u32(out, static_cast<uint32_t>(value));
}

inline void put_f32(std::vector<uint8_t> &out, float value) {
  put_u32(out, std::bit_cast<uint32_t>(value));
}

/// Append a u8-length-prefixed UTF-8 string (truncated to 255 bytes).
inline void put_str(std::vector<uint8_t> &out, std::string_view str) {
  const size_t count = std::min<size_t>(str.size(), 0xFF);
  out.push_back(static_cast<uint8_t>(count));
  out.insert(out.end(), str.begin(), str.begin() + count);
}

inline std::optional<uint8_t> get_u8_at(std::span<const uint8_t> bytes, size_t offset) {
  if (bytes.size() < offset + 1)
    return std::nullopt;
  return bytes[offset];
}

inline std::optional<uint16_t> get_u16_at(std::span<const uint8_t> bytes, size_t offset) {
  if (bytes.size() < offset + 2)
    return std::nullopt;
  return static_cast<uint16_t>(bytes[offset]) | (static_cast<uint16_t>(bytes[offset + 1]) << 8);
}

inline std::optional<float> get_f32_at(std::span<const uint8_t> bytes, size_t offset) {
  if (bytes.size() < offset + 4)
    return std::nullopt;
  return std::bit_cast<float>(get_u32(bytes.subspan(offset)));
}

/// Build a frame for any STEM-protocol message type (module 7; the reply flag is
/// set for reply/telemetry types, whose ids have the high bit set).
inline std::vector<uint8_t> build(Msg type, std::span<const uint8_t> payload = {}) {
  const bool reply = (static_cast<uint8_t>(type) & 0x80) != 0;
  return stream::build_frame(reply, kModule, static_cast<uint8_t>(type), payload);
}

/// OK reply attributed to `request`.
inline std::vector<uint8_t> make_ok(Msg request, uint32_t value) {
  std::vector<uint8_t> payload;
  payload.push_back(static_cast<uint8_t>(request));
  put_u32(payload, value);
  return build(Msg::Ok, payload);
}

/// ERROR reply attributed to `request`.
inline std::vector<uint8_t> make_error(Msg request, uint32_t code, std::string_view message) {
  std::vector<uint8_t> payload;
  payload.push_back(static_cast<uint8_t>(request));
  put_u32(payload, code);
  payload.insert(payload.end(), message.begin(), message.end());
  return build(Msg::Error, payload);
}

} // namespace stem_proto
