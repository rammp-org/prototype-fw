#pragma once

// Wire protocol of the Eyou motor console: a small command set that lets a host
// (web/eyou_console.html, or any other client) drive ONE Eyou harmonic actuator
// on the device's CAN bus from a browser: read its live position / velocity /
// torque / current, set the torque limit the drive clamps to, command absolute
// and relative moves, enable / disable / fault-reset / halt the drive, and set,
// save and load its software zero.
//
// Framed with the espp stream_frame v2 codec and routed by an espp::Dispatcher
// on MODULE ID 0x20 (kModuleId). Host->device requests use type 0x1X; the
// device->host replies / events use 0x9X (their high bit is the frame reply
// flag). All multi-byte fields are little-endian; floats are IEEE-754 binary32.
//
// Standard-library only (host-testable); EyouService (eyou_service.hpp)
// implements the device side.

#include <bit>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace eyou_protocol {

/// Default dispatcher module id (a routing key; EyouService::Config::module can
/// move it, the console finds it through discovery by kProtocol).
static constexpr uint8_t kModuleId = 0x20;
/// Protocol identifier + version advertised through discovery.
static constexpr const char *kProtocol = "eyou.motor";
static constexpr uint16_t kProtocolVersion = 1;

/// Host -> device requests (0x1X).
enum class Request : uint8_t {
  GetStatus = 0x10,         ///< one STATUS snapshot (no payload)
  SetStream = 0x11,         ///< [enable u8][period_ms u16]: stream STATUS on this transport
  SetTorqueLimit = 0x12,    ///< [permille u16]: max torque (0x6072), per-thousand of rated
  MoveTo = 0x13,            ///< [angle_deg f32][immediate u8]: absolute move (profile position)
  MoveBy = 0x14,            ///< [delta_deg f32][immediate u8]: relative move
  Enable = 0x15,            ///< DS402 -> Operation enabled (no payload)
  Disable = 0x16,           ///< DS402 -> Ready to switch on, power stage off (no payload)
  ResetFault = 0x17,        ///< clear a DS402 fault (no payload)
  Halt = 0x18,              ///< stop the move in progress, stay enabled (no payload)
  SetZero = 0x19,           ///< [mode u8][angle_deg f32]: 0 = the current position becomes 0,
                            ///< 1 = the current position becomes angle_deg (volatile)
  SaveZero = 0x1A,          ///< persist the software zero to NVS (no payload)
  LoadZero = 0x1B,          ///< load the software zero from NVS (no payload)
  SetProfile = 0x1C,        ///< [velocity f32][acceleration f32][deceleration f32] deg/s, deg/s^2
  SetPositionLimits = 0x1D, ///< [min_deg f32][max_deg f32]: MoveTo targets are clamped to these
};

/// Device -> host replies / events (0x9X => reply flag set).
enum class Reply : uint8_t {
  Status = 0x90, ///< status snapshot (see Status)
  Ok = 0x91,     ///< ack for a request: [request_type u8]
  Error = 0x92,  ///< failure: [request_type u8][code u32][utf8 message]
};

constexpr bool is_reply(uint8_t type) { return (type & 0x80) != 0; }

// ---- little-endian helpers -----------------------------------------------------

inline void put_u16(std::vector<uint8_t> &out, uint16_t v) {
  out.push_back(static_cast<uint8_t>(v));
  out.push_back(static_cast<uint8_t>(v >> 8));
}
inline void put_u32(std::vector<uint8_t> &out, uint32_t v) {
  out.push_back(static_cast<uint8_t>(v));
  out.push_back(static_cast<uint8_t>(v >> 8));
  out.push_back(static_cast<uint8_t>(v >> 16));
  out.push_back(static_cast<uint8_t>(v >> 24));
}
inline void put_i16(std::vector<uint8_t> &out, int16_t v) {
  put_u16(out, static_cast<uint16_t>(v));
}
inline void put_i32(std::vector<uint8_t> &out, int32_t v) {
  put_u32(out, static_cast<uint32_t>(v));
}
inline void put_f32(std::vector<uint8_t> &out, float v) {
  put_u32(out, std::bit_cast<uint32_t>(v));
}
inline uint16_t get_u16(std::span<const uint8_t> p, size_t off) {
  return static_cast<uint16_t>(static_cast<uint16_t>(p[off]) |
                               (static_cast<uint16_t>(p[off + 1]) << 8));
}
inline uint32_t get_u32(std::span<const uint8_t> p, size_t off) {
  return static_cast<uint32_t>(p[off]) | (static_cast<uint32_t>(p[off + 1]) << 8) |
         (static_cast<uint32_t>(p[off + 2]) << 16) | (static_cast<uint32_t>(p[off + 3]) << 24);
}
inline int16_t get_i16(std::span<const uint8_t> p, size_t off) {
  return static_cast<int16_t>(get_u16(p, off));
}
inline int32_t get_i32(std::span<const uint8_t> p, size_t off) {
  return static_cast<int32_t>(get_u32(p, off));
}
inline float get_f32(std::span<const uint8_t> p, size_t off) {
  return std::bit_cast<float>(get_u32(p, off));
}

// ---- request payloads -------------------------------------------------------------

/// SetStream: [enable u8][period_ms u16] (3 B); period 0 = device default.
struct SetStream {
  bool enabled{false};
  uint16_t period_ms{0};
  static constexpr size_t kSize = 3;
  static std::optional<SetStream> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    return SetStream{p[0] != 0, get_u16(p, 1)};
  }
  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out{static_cast<uint8_t>(enabled ? 1 : 0)};
    put_u16(out, period_ms);
    return out;
  }
};

/// SetTorqueLimit: [permille u16] (2 B).
struct SetTorqueLimit {
  uint16_t permille{0};
  static constexpr size_t kSize = 2;
  static std::optional<SetTorqueLimit> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    return SetTorqueLimit{get_u16(p, 0)};
  }
  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out;
    put_u16(out, permille);
    return out;
  }
};

/// MoveTo / MoveBy: [degrees f32][immediate u8] (5 B).
struct Move {
  float degrees{0.0f};
  bool immediate{true};
  static constexpr size_t kSize = 5;
  static std::optional<Move> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    return Move{get_f32(p, 0), p[4] != 0};
  }
  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out;
    put_f32(out, degrees);
    out.push_back(immediate ? 1 : 0);
    return out;
  }
};

/// SetZero: [mode u8][angle_deg f32] (5 B); mode 0 ignores the angle.
struct SetZero {
  uint8_t mode{0};
  float angle_degrees{0.0f};
  static constexpr uint8_t kModeHere = 0;    ///< the current position becomes 0
  static constexpr uint8_t kModeAtAngle = 1; ///< the current position becomes angle_degrees
  static constexpr size_t kSize = 5;
  static std::optional<SetZero> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    return SetZero{p[0], get_f32(p, 1)};
  }
  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out{mode};
    put_f32(out, angle_degrees);
    return out;
  }
};

/// SetProfile: [velocity f32][acceleration f32][deceleration f32] (12 B).
struct SetProfile {
  float velocity_dps{0.0f};
  float acceleration_dps2{0.0f};
  float deceleration_dps2{0.0f};
  static constexpr size_t kSize = 12;
  static std::optional<SetProfile> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    return SetProfile{get_f32(p, 0), get_f32(p, 4), get_f32(p, 8)};
  }
  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out;
    put_f32(out, velocity_dps);
    put_f32(out, acceleration_dps2);
    put_f32(out, deceleration_dps2);
    return out;
  }
};

/// SetPositionLimits: [min_deg f32][max_deg f32] (8 B).
struct SetPositionLimits {
  float min_degrees{0.0f};
  float max_degrees{0.0f};
  static constexpr size_t kSize = 8;
  static std::optional<SetPositionLimits> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    return SetPositionLimits{get_f32(p, 0), get_f32(p, 4)};
  }
  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out;
    put_f32(out, min_degrees);
    put_f32(out, max_degrees);
    return out;
  }
};

// ---- reply payloads ----------------------------------------------------------------

/// STATUS payload (72 B):
///   [flags u8][node_id u8][statusword u16][error_code u16][mode i8][reserved u8]
///   [angle_deg f32][velocity_dps f32][torque_permille i16][current_permille i16]
///   [torque_limit_permille u16][torque_limit_max_permille u16]
///   [target_deg f32][rated_torque_mnm u32][rated_current_ma u32][zero_offset_pulses i32]
///   [pos_min_deg f32][pos_max_deg f32][profile_velocity f32][profile_accel f32][profile_decel f32]
///   [timestamp_ms u32][sequence u32][stream_period_ms u16][reserved u16]
/// The live fields are the latest poll of the drive; torque / current are
/// per-thousand of the rated torque / current (rated_* give the scale).
struct Status {
  bool enabled{false};        ///< DS402 Operation enabled
  bool faulted{false};        ///< DS402 Fault (or the drive did not answer)
  bool zero_set{false};       ///< a software zero is active (angles are relative to it)
  bool stream_enabled{false}; ///< STATUS is streamed on this transport
  bool target_reached{false}; ///< statusword bit 10
  bool comm_ok{false};        ///< the last poll of the drive succeeded
  uint8_t node_id{1};
  uint16_t statusword{0};
  uint16_t error_code{0}; ///< object 0x603F
  int8_t mode{0};         ///< modes of operation display (0x6061); 1 = profile position
  float angle_degrees{0.0f};
  float velocity_dps{0.0f};
  int16_t torque_permille{0};
  int16_t current_permille{0};
  uint16_t torque_limit_permille{0};
  uint16_t torque_limit_max_permille{0}; ///< ceiling SetTorqueLimit accepts (device config)
  float target_degrees{0.0f};            ///< the last MoveTo target (NaN until one was commanded)
  uint32_t rated_torque_mnm{0};
  uint32_t rated_current_ma{0};
  int32_t zero_offset_pulses{0};
  float pos_min_degrees{0.0f}; ///< +-inf when unset
  float pos_max_degrees{0.0f};
  float profile_velocity_dps{0.0f};
  float profile_accel_dps2{0.0f};
  float profile_decel_dps2{0.0f};
  uint32_t timestamp_ms{0};
  uint32_t sequence{0};
  uint16_t stream_period_ms{0};

  static constexpr uint8_t kFlagEnabled = 0x01;
  static constexpr uint8_t kFlagFaulted = 0x02;
  static constexpr uint8_t kFlagZeroSet = 0x04;
  static constexpr uint8_t kFlagStreamEnabled = 0x08;
  static constexpr uint8_t kFlagTargetReached = 0x10;
  static constexpr uint8_t kFlagCommOk = 0x20;
  static constexpr size_t kSize = 72;

  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out;
    out.reserve(kSize);
    uint8_t flags = 0;
    if (enabled)
      flags |= kFlagEnabled;
    if (faulted)
      flags |= kFlagFaulted;
    if (zero_set)
      flags |= kFlagZeroSet;
    if (stream_enabled)
      flags |= kFlagStreamEnabled;
    if (target_reached)
      flags |= kFlagTargetReached;
    if (comm_ok)
      flags |= kFlagCommOk;
    out.push_back(flags);
    out.push_back(node_id);
    put_u16(out, statusword);
    put_u16(out, error_code);
    out.push_back(static_cast<uint8_t>(mode));
    out.push_back(0);
    put_f32(out, angle_degrees);
    put_f32(out, velocity_dps);
    put_i16(out, torque_permille);
    put_i16(out, current_permille);
    put_u16(out, torque_limit_permille);
    put_u16(out, torque_limit_max_permille);
    put_f32(out, target_degrees);
    put_u32(out, rated_torque_mnm);
    put_u32(out, rated_current_ma);
    put_i32(out, zero_offset_pulses);
    put_f32(out, pos_min_degrees);
    put_f32(out, pos_max_degrees);
    put_f32(out, profile_velocity_dps);
    put_f32(out, profile_accel_dps2);
    put_f32(out, profile_decel_dps2);
    put_u32(out, timestamp_ms);
    put_u32(out, sequence);
    put_u16(out, stream_period_ms);
    put_u16(out, 0);
    return out;
  }

  static std::optional<Status> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    Status s;
    const uint8_t flags = p[0];
    s.enabled = (flags & kFlagEnabled) != 0;
    s.faulted = (flags & kFlagFaulted) != 0;
    s.zero_set = (flags & kFlagZeroSet) != 0;
    s.stream_enabled = (flags & kFlagStreamEnabled) != 0;
    s.target_reached = (flags & kFlagTargetReached) != 0;
    s.comm_ok = (flags & kFlagCommOk) != 0;
    s.node_id = p[1];
    s.statusword = get_u16(p, 2);
    s.error_code = get_u16(p, 4);
    s.mode = static_cast<int8_t>(p[6]);
    s.angle_degrees = get_f32(p, 8);
    s.velocity_dps = get_f32(p, 12);
    s.torque_permille = get_i16(p, 16);
    s.current_permille = get_i16(p, 18);
    s.torque_limit_permille = get_u16(p, 20);
    s.torque_limit_max_permille = get_u16(p, 22);
    s.target_degrees = get_f32(p, 24);
    s.rated_torque_mnm = get_u32(p, 28);
    s.rated_current_ma = get_u32(p, 32);
    s.zero_offset_pulses = get_i32(p, 36);
    s.pos_min_degrees = get_f32(p, 40);
    s.pos_max_degrees = get_f32(p, 44);
    s.profile_velocity_dps = get_f32(p, 48);
    s.profile_accel_dps2 = get_f32(p, 52);
    s.profile_decel_dps2 = get_f32(p, 56);
    s.timestamp_ms = get_u32(p, 60);
    s.sequence = get_u32(p, 64);
    s.stream_period_ms = get_u16(p, 68);
    return s;
  }
};

/// OK payload: [request_type u8].
inline std::vector<uint8_t> make_ok_payload(uint8_t request_type) { return {request_type}; }

/// ERROR payload: [request_type u8][code u32][utf8 message].
inline std::vector<uint8_t> make_error_payload(uint8_t request_type, uint32_t code,
                                               std::string_view message) {
  std::vector<uint8_t> out{request_type};
  put_u32(out, code);
  out.insert(out.end(), message.begin(), message.end());
  return out;
}

} // namespace eyou_protocol
