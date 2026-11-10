#pragma once

// Wire protocol of the PEAR linkage console: configure and drive ONE 5-bar
// linkage (two Eyou harmonic actuators on CAN, one per crank) from a browser:
// the geometry (link lengths, assembly), each motor's node id / direction /
// limits / crank angle at zero / torque limit, the profile; end-effector moves
// (inverse kinematics on the device), direct crank moves, enable / disable /
// fault reset / halt, the motors' software zeros; and a streamed STATUS with
// the live pose (forward kinematics of the measured crank angles) and every
// motor's readings.
//
// Framed with the espp stream_frame v2 codec and routed by an espp::Dispatcher
// on MODULE ID 0x21 (kModuleId). Host->device requests use types 0x1X / 0x2X;
// the device->host replies / events use 0x9X (their high bit is the frame
// reply flag). All multi-byte fields are little-endian; floats are IEEE-754
// binary32; angles in degrees, lengths / positions in millimetres. The
// linkage frame is the kinematics' (five_bar.hpp): M1 at the origin, +x toward
// M2, crank angles CCW from +x.
//
// Standard-library only (host-testable); LinkageService implements the device
// side. The SETTINGS payload doubles as the device's NVS record.

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "five_bar.hpp"

namespace linkage_protocol {

static constexpr uint8_t kModuleId = 0x21;
static constexpr const char *kProtocol = "pear.linkage";
static constexpr uint16_t kProtocolVersion = 1;
static constexpr uint8_t kMotorCount = 2; ///< crank 1 (M1) and crank 2 (M2)
static constexpr uint8_t kAllMotors = 0xFF;

/// Host -> device requests.
enum class Request : uint8_t {
  GetInfo = 0x10,       ///< -> Info
  GetStatus = 0x11,     ///< -> Status
  SetStream = 0x12,     ///< [enable u8][period_ms u16]: stream Status on this transport
  GetSettings = 0x13,   ///< -> Settings
  SetSettings = 0x14,   ///< [Settings]: apply + persist (node ids, limits, geometry, ...)
  ResetSettings = 0x15, ///< back to the firmware defaults (applied + persisted)
  MoveTo = 0x16,        ///< [x f32][y f32]: end effector to (x, y) (ik on the device)
  MoveBy = 0x17,        ///< [dx f32][dy f32]: relative to the current pose
  Home = 0x18,          ///< both cranks to the configured home angles
  SetMotor = 0x19,      ///< [motor u8][degrees f32]: one crank's motor angle (direct)
  Enable = 0x1B,        ///< [motor u8 | 0xFF]
  Disable = 0x1C,       ///< [motor u8 | 0xFF]
  ResetFault = 0x1D,    ///< [motor u8 | 0xFF]
  Halt = 0x1E,          ///< [motor u8 | 0xFF]
  ZeroMotor = 0x1F,     ///< [motor u8][mode u8][value f32]: set the motor's software zero so that
                        ///< NOW it reads 0 (mode 0), `value` motor degrees (mode 1) or so that the
                        ///< crank is at `value` degrees in the linkage frame (mode 2)
  SaveZero = 0x20,      ///< [motor u8 | 0xFF]: persist the software zero(s) on the device
  LoadZero = 0x21,      ///< [motor u8 | 0xFF]
  SolveIk = 0x22,       ///< [x f32][y f32] -> IkResult (no motion)
};

/// Device -> host replies / events (0x9X => reply flag set).
enum class Reply : uint8_t {
  Info = 0x90,
  Status = 0x91,
  Settings = 0x92,
  IkResult = 0x93,
  Ok = 0x94,    ///< [request u8]
  Error = 0x95, ///< [request u8][code u32][utf8 message]
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
inline void put_f32(std::vector<uint8_t> &out, float v) {
  put_u32(out, std::bit_cast<uint32_t>(v));
}
inline void put_str(std::vector<uint8_t> &out, std::string_view s) {
  const size_t n = s.size() > 255 ? 255 : s.size();
  out.push_back(static_cast<uint8_t>(n));
  out.insert(out.end(), s.begin(), s.begin() + n);
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
inline float get_f32(std::span<const uint8_t> p, size_t off) {
  return std::bit_cast<float>(get_u32(p, off));
}

// ---- settings -------------------------------------------------------------------

/// One crank's motor.
struct MotorSettings {
  uint8_t node_id{0};    ///< CANopen node id; 0 = no motor (the crank is not driven)
  int8_t direction{1};   ///< +1: positive motor angle = CCW crank; -1: CW
  float min_deg{-90.0f}; ///< motor angle limits (motor degrees, relative to its zero)
  float max_deg{90.0f};
  float theta_at_zero_deg{-90.0f};     ///< the crank's angle (linkage frame) when the motor reads 0
                                       ///< (default: the crank hanging straight down reads 0)
  uint16_t torque_limit_permille{300}; ///< max torque 0x6072, per-thousand of rated
  static constexpr float kLimitToleranceDeg = 1e-3f; ///< float round-off at a limit

  /// crank angle (linkage frame) <-> motor angle
  float motor_to_theta(float motor_deg) const {
    return fivebar::normalize_angle_deg(theta_at_zero_deg + direction * motor_deg);
  }
  float theta_to_motor(float theta_deg) const {
    return direction * fivebar::normalize_angle_deg(theta_deg - theta_at_zero_deg);
  }
  bool within_limits(float motor_deg) const {
    return motor_deg >= min_deg - kLimitToleranceDeg && motor_deg <= max_deg + kLimitToleranceDeg;
  }
};

/// Everything the console configures; the SETTINGS payload and the NVS record.
///   [version u8]
///   geometry: [m2_x f32][m2_y f32][crank1 f32][crank2 f32][bar f32][coupler f32]
///             [ee_extension f32][ee_offset f32][assembly u8][coupler_side u8]
///   [home_theta1 f32][home_theta2 f32]
///   per motor (2): [node_id u8][direction i8][reserved u16][min f32][max f32]
///                  [theta_at_zero f32][torque_limit u16][reserved u16]
///   [profile_velocity f32][profile_accel f32][profile_decel f32]
struct Settings {
  static constexpr uint8_t kVersion = 1;
  static constexpr size_t kSize = 1 + 34 + 8 + 2 * 20 + 12; // 95

  fivebar::Geometry geometry{};
  /// The Home pose and the ik's branch reference without a measured pose
  /// (default: both cranks hanging straight down, the wheel below the base).
  float home_theta1_deg{-90.0f};
  float home_theta2_deg{-90.0f};
  std::array<MotorSettings, kMotorCount> motors{};
  float profile_velocity_dps{30.0f};
  float profile_accel_dps2{20.0f};
  float profile_decel_dps2{20.0f};

  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out;
    out.reserve(kSize);
    out.push_back(kVersion);
    const auto &g = geometry;
    put_f32(out, g.m2_x);
    put_f32(out, g.m2_y);
    put_f32(out, g.crank1);
    put_f32(out, g.crank2);
    put_f32(out, g.bar);
    put_f32(out, g.coupler);
    put_f32(out, g.ee_extension);
    put_f32(out, g.ee_offset);
    out.push_back(static_cast<uint8_t>(g.assembly));
    out.push_back(static_cast<uint8_t>(g.coupler_side));
    put_f32(out, home_theta1_deg);
    put_f32(out, home_theta2_deg);
    for (const auto &m : motors) {
      out.push_back(m.node_id);
      out.push_back(static_cast<uint8_t>(m.direction));
      put_u16(out, 0);
      put_f32(out, m.min_deg);
      put_f32(out, m.max_deg);
      put_f32(out, m.theta_at_zero_deg);
      put_u16(out, m.torque_limit_permille);
      put_u16(out, 0);
    }
    put_f32(out, profile_velocity_dps);
    put_f32(out, profile_accel_dps2);
    put_f32(out, profile_decel_dps2);
    return out;
  }

  static std::optional<Settings> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize || p[0] != kVersion)
      return std::nullopt;
    Settings s;
    size_t i = 1;
    auto f32 = [&]() {
      const float v = get_f32(p, i);
      i += 4;
      return v;
    };
    auto &g = s.geometry;
    g.m2_x = f32();
    g.m2_y = f32();
    g.crank1 = f32();
    g.crank2 = f32();
    g.bar = f32();
    g.coupler = f32();
    g.ee_extension = f32();
    g.ee_offset = f32();
    g.assembly = static_cast<fivebar::Assembly>(p[i++]);
    g.coupler_side = static_cast<fivebar::CouplerSide>(p[i++]);
    s.home_theta1_deg = f32();
    s.home_theta2_deg = f32();
    for (auto &m : s.motors) {
      m.node_id = p[i++];
      m.direction = static_cast<int8_t>(p[i++]);
      i += 2;
      m.min_deg = f32();
      m.max_deg = f32();
      m.theta_at_zero_deg = f32();
      m.torque_limit_permille = get_u16(p, i);
      i += 4;
    }
    s.profile_velocity_dps = f32();
    s.profile_accel_dps2 = f32();
    s.profile_decel_dps2 = f32();
    return s;
  }

  /// Sanity of the values (the device refuses settings that fail; `why` says
  /// what). `max_torque_permille` is the device's ceiling for the limits.
  bool validate(uint16_t max_torque_permille, std::string &why) const {
    if (!fivebar::is_valid(geometry)) {
      why = "geometry: every length must be positive and the loop closable";
      return false;
    }
    if (static_cast<uint8_t>(geometry.assembly) > 2) {
      why = "geometry: assembly must be 0 (any), 1 (crossed) or 2 (open)";
      return false;
    }
    if (static_cast<uint8_t>(geometry.coupler_side) > 1) {
      why = "geometry: coupler side must be 0 (ccw) or 1 (cw)";
      return false;
    }
    if (!std::isfinite(home_theta1_deg) || !std::isfinite(home_theta2_deg)) {
      why = "home angles must be finite";
      return false;
    }
    for (size_t k = 0; k < motors.size(); ++k) {
      const auto &m = motors[k];
      const std::string who = "motor " + std::to_string(k + 1) + ": ";
      if (m.node_id > 127) {
        why = who + "node id must be 0 (none) or 1..127";
        return false;
      }
      if (m.direction != 1 && m.direction != -1) {
        why = who + "direction must be +1 or -1";
        return false;
      }
      if (!std::isfinite(m.min_deg) || !std::isfinite(m.max_deg) || m.min_deg > m.max_deg) {
        why = who + "limits must be finite with min <= max";
        return false;
      }
      if (!std::isfinite(m.theta_at_zero_deg)) {
        why = who + "crank angle at zero must be finite";
        return false;
      }
      if (m.torque_limit_permille > max_torque_permille) {
        why = who + "torque limit " + std::to_string(m.torque_limit_permille) +
              " exceeds the device's maximum of " + std::to_string(max_torque_permille);
        return false;
      }
    }
    if (!(profile_velocity_dps > 0.0f) || !(profile_accel_dps2 > 0.0f) ||
        !(profile_decel_dps2 > 0.0f)) {
      why = "profile velocity / acceleration / deceleration must be positive";
      return false;
    }
    if (motors[0].node_id != 0 && motors[0].node_id == motors[1].node_id) {
      why = "the two motors cannot share a node id";
      return false;
    }
    return true;
  }
};

// ---- request payloads -----------------------------------------------------------

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

/// MoveTo / MoveBy / SolveIk: [x f32][y f32] (8 B).
struct Point2 {
  float x{0.0f};
  float y{0.0f};
  static constexpr size_t kSize = 8;
  static std::optional<Point2> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    return Point2{get_f32(p, 0), get_f32(p, 4)};
  }
  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out;
    put_f32(out, x);
    put_f32(out, y);
    return out;
  }
};

/// SetMotor: [motor u8][degrees f32] (5 B).
struct SetMotor {
  uint8_t motor{0};
  float degrees{0.0f};
  static constexpr size_t kSize = 5;
  static std::optional<SetMotor> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    return SetMotor{p[0], get_f32(p, 1)};
  }
  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out{motor};
    put_f32(out, degrees);
    return out;
  }
};

/// Enable / Disable / ResetFault / Halt / SaveZero / LoadZero: [motor u8] (1 B), 0xFF = all.
struct MotorSelect {
  uint8_t motor{kAllMotors};
  static constexpr size_t kSize = 1;
  static std::optional<MotorSelect> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    return MotorSelect{p[0]};
  }
  std::vector<uint8_t> serialize() const { return {motor}; }
};

/// ZeroMotor: [motor u8][mode u8][value f32] (6 B).
struct ZeroMotor {
  static constexpr uint8_t kModeHere = 0;       ///< the motor reads 0 here
  static constexpr uint8_t kModeMotorAngle = 1; ///< the motor reads `value` here (motor deg)
  static constexpr uint8_t kModeCrankAngle =
      2; ///< the crank is at `value` deg (linkage frame) here
  uint8_t motor{0};
  uint8_t mode{kModeHere};
  float value{0.0f};
  static constexpr size_t kSize = 6;
  static std::optional<ZeroMotor> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    return ZeroMotor{p[0], p[1], get_f32(p, 2)};
  }
  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out{motor, mode};
    put_f32(out, value);
    return out;
  }
};

// ---- reply payloads --------------------------------------------------------------

/// INFO: [protocol_version u8][motor_count u8][max_torque_permille u16][poll_period_ms u16]
///       [str project][str version][str build]
struct Info {
  uint8_t protocol_version{kProtocolVersion};
  uint8_t motor_count{kMotorCount};
  uint16_t max_torque_permille{0};
  uint16_t poll_period_ms{0};
  std::string project;
  std::string version;
  std::string build;
  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out{protocol_version, motor_count};
    put_u16(out, max_torque_permille);
    put_u16(out, poll_period_ms);
    put_str(out, project);
    put_str(out, version);
    put_str(out, build);
    return out;
  }
};

/// One motor's live readings inside STATUS (24 B):
///   [node u8][flags u8][statusword u16][error_code u16][reserved u16]
///   [angle f32 (motor deg)][velocity f32 (deg/s)][torque i16][current i16]
///   [torque_limit u16][reserved u16]
struct MotorStatus {
  static constexpr uint8_t kFlagCommOk = 0x01;
  static constexpr uint8_t kFlagEnabled = 0x02;
  static constexpr uint8_t kFlagFaulted = 0x04;
  static constexpr uint8_t kFlagTargetReached = 0x08;
  static constexpr uint8_t kFlagZeroSet = 0x10;
  static constexpr uint8_t kFlagPresent = 0x20; ///< node id != 0
  static constexpr size_t kSize = 24;
  uint8_t node_id{0};
  bool present{false};
  bool comm_ok{false};
  bool enabled{false};
  bool faulted{false};
  bool target_reached{false};
  bool zero_set{false};
  uint16_t statusword{0};
  uint16_t error_code{0};
  float angle_deg{0.0f}; ///< motor angle (NaN until read)
  float velocity_dps{0.0f};
  int16_t torque_permille{0};
  int16_t current_permille{0};
  uint16_t torque_limit_permille{0};
};

/// STATUS (87 B):
///   [flags u8][sequence u32][timestamp_ms u32][stream_period_ms u16]
///   [x f32][y f32][theta1 f32][theta2 f32][bar_angle f32][target_x f32][target_y f32]
///   then kMotorCount x MotorStatus.
/// x / y / thetas are NaN while the pose is not valid (a motor missing or no
/// fk solution); target_* are NaN until a move was accepted.
struct Status {
  static constexpr uint8_t kFlagStream = 0x01;
  static constexpr uint8_t kFlagCanOk = 0x02;
  static constexpr uint8_t kFlagPoseValid = 0x04;
  static constexpr uint8_t kFlagTargetValid = 0x08;
  static constexpr uint8_t kFlagCommOk = 0x10;  ///< every present motor answered
  static constexpr uint8_t kFlagEnabled = 0x20; ///< every present motor Operation enabled
  static constexpr uint8_t kFlagFaulted = 0x40; ///< any motor faulted (or not answering)
  static constexpr uint8_t kFlagMoving = 0x80;  ///< some enabled motor has not reached its target
  static constexpr size_t kSize = 11 + 28 + kMotorCount * MotorStatus::kSize;

  bool stream_enabled{false};
  bool can_ok{false};
  bool pose_valid{false};
  bool target_valid{false};
  bool comm_ok{false};
  bool enabled{false};
  bool faulted{false};
  bool moving{false};
  uint32_t sequence{0};
  uint32_t timestamp_ms{0};
  uint16_t stream_period_ms{0};
  float x{0.0f};
  float y{0.0f};
  float theta1_deg{0.0f};
  float theta2_deg{0.0f};
  float bar_angle_deg{0.0f};
  float target_x{0.0f};
  float target_y{0.0f};
  std::array<MotorStatus, kMotorCount> motors{};

  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out;
    out.reserve(kSize);
    uint8_t flags = 0;
    if (stream_enabled)
      flags |= kFlagStream;
    if (can_ok)
      flags |= kFlagCanOk;
    if (pose_valid)
      flags |= kFlagPoseValid;
    if (target_valid)
      flags |= kFlagTargetValid;
    if (comm_ok)
      flags |= kFlagCommOk;
    if (enabled)
      flags |= kFlagEnabled;
    if (faulted)
      flags |= kFlagFaulted;
    if (moving)
      flags |= kFlagMoving;
    out.push_back(flags);
    put_u32(out, sequence);
    put_u32(out, timestamp_ms);
    put_u16(out, stream_period_ms);
    put_f32(out, x);
    put_f32(out, y);
    put_f32(out, theta1_deg);
    put_f32(out, theta2_deg);
    put_f32(out, bar_angle_deg);
    put_f32(out, target_x);
    put_f32(out, target_y);
    for (const auto &m : motors) {
      uint8_t mf = 0;
      if (m.comm_ok)
        mf |= MotorStatus::kFlagCommOk;
      if (m.enabled)
        mf |= MotorStatus::kFlagEnabled;
      if (m.faulted)
        mf |= MotorStatus::kFlagFaulted;
      if (m.target_reached)
        mf |= MotorStatus::kFlagTargetReached;
      if (m.zero_set)
        mf |= MotorStatus::kFlagZeroSet;
      if (m.present)
        mf |= MotorStatus::kFlagPresent;
      out.push_back(m.node_id);
      out.push_back(mf);
      put_u16(out, m.statusword);
      put_u16(out, m.error_code);
      put_u16(out, 0);
      put_f32(out, m.angle_deg);
      put_f32(out, m.velocity_dps);
      put_i16(out, m.torque_permille);
      put_i16(out, m.current_permille);
      put_u16(out, m.torque_limit_permille);
      put_u16(out, 0);
    }
    return out;
  }

  static std::optional<Status> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    Status s;
    const uint8_t flags = p[0];
    s.stream_enabled = (flags & kFlagStream) != 0;
    s.can_ok = (flags & kFlagCanOk) != 0;
    s.pose_valid = (flags & kFlagPoseValid) != 0;
    s.target_valid = (flags & kFlagTargetValid) != 0;
    s.comm_ok = (flags & kFlagCommOk) != 0;
    s.enabled = (flags & kFlagEnabled) != 0;
    s.faulted = (flags & kFlagFaulted) != 0;
    s.moving = (flags & kFlagMoving) != 0;
    s.sequence = get_u32(p, 1);
    s.timestamp_ms = get_u32(p, 5);
    s.stream_period_ms = get_u16(p, 9);
    s.x = get_f32(p, 11);
    s.y = get_f32(p, 15);
    s.theta1_deg = get_f32(p, 19);
    s.theta2_deg = get_f32(p, 23);
    s.bar_angle_deg = get_f32(p, 27);
    s.target_x = get_f32(p, 31);
    s.target_y = get_f32(p, 35);
    size_t i = 39;
    for (auto &m : s.motors) {
      m.node_id = p[i];
      const uint8_t mf = p[i + 1];
      m.comm_ok = (mf & MotorStatus::kFlagCommOk) != 0;
      m.enabled = (mf & MotorStatus::kFlagEnabled) != 0;
      m.faulted = (mf & MotorStatus::kFlagFaulted) != 0;
      m.target_reached = (mf & MotorStatus::kFlagTargetReached) != 0;
      m.zero_set = (mf & MotorStatus::kFlagZeroSet) != 0;
      m.present = (mf & MotorStatus::kFlagPresent) != 0;
      m.statusword = get_u16(p, i + 2);
      m.error_code = get_u16(p, i + 4);
      m.angle_deg = get_f32(p, i + 8);
      m.velocity_dps = get_f32(p, i + 12);
      m.torque_permille = get_i16(p, i + 16);
      m.current_permille = get_i16(p, i + 18);
      m.torque_limit_permille = get_u16(p, i + 20);
      i += MotorStatus::kSize;
    }
    return s;
  }
};

/// IK_RESULT (58 B): [x f32][y f32][reachable u8][within_limits u8]
///   [theta1 f32][theta2 f32][motor1 f32][motor2 f32]
///   [j1 x f32][j1 y][j2 x][j2 y][j3 x][j3 y][m3 x][m3 y]
/// Everything after `within_limits` is zero when not reachable.
struct IkResult {
  static constexpr size_t kSize = 58;
  float x{0.0f};
  float y{0.0f};
  bool reachable{false};
  bool within_limits{false};
  float theta1_deg{0.0f};
  float theta2_deg{0.0f};
  std::array<float, kMotorCount> motor_deg{};
  fivebar::Pose pose{};

  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out;
    out.reserve(kSize);
    put_f32(out, x);
    put_f32(out, y);
    out.push_back(reachable ? 1 : 0);
    out.push_back(within_limits ? 1 : 0);
    put_f32(out, theta1_deg);
    put_f32(out, theta2_deg);
    put_f32(out, motor_deg[0]);
    put_f32(out, motor_deg[1]);
    for (const auto &pt : {pose.j1, pose.j2, pose.j3, pose.m3}) {
      put_f32(out, pt[0]);
      put_f32(out, pt[1]);
    }
    return out;
  }

  static std::optional<IkResult> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    IkResult r;
    r.x = get_f32(p, 0);
    r.y = get_f32(p, 4);
    r.reachable = p[8] != 0;
    r.within_limits = p[9] != 0;
    r.theta1_deg = get_f32(p, 10);
    r.theta2_deg = get_f32(p, 14);
    r.motor_deg[0] = get_f32(p, 18);
    r.motor_deg[1] = get_f32(p, 22);
    size_t i = 26;
    for (auto *pt : {&r.pose.j1, &r.pose.j2, &r.pose.j3, &r.pose.m3}) {
      (*pt)[0] = get_f32(p, i);
      (*pt)[1] = get_f32(p, i + 4);
      i += 8;
    }
    r.pose.theta1_deg = r.theta1_deg;
    r.pose.theta2_deg = r.theta2_deg;
    return r;
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

} // namespace linkage_protocol
