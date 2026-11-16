#pragma once

// The one place that turns end-effector targets into actuator-pair commands.
// Shared by the console CLI and the USB (dispatcher) module, so both see the
// same limits, the same IK reference convention and the same tracked state.
// All public methods are serialized by an internal mutex (the CLI thread and
// the USB RX worker call in concurrently).

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <span>
#include <string_view>

#include "base_component.hpp"
#include "can_bus.hpp"
#include "eyou_motor.hpp"
#include "ik_5bar.hpp"
#include "paired_eyou_actuator.hpp"

namespace stem {

enum class Pair : uint8_t { Left = 0, Right = 1, Seat = 2 };
inline constexpr size_t kPairCount = 3;
inline constexpr size_t kMotorCount = 6;

const char *to_string(Pair pair);
std::optional<Pair> pair_from_name(std::string_view name);
std::optional<Pair> pair_from_index(uint8_t index);

struct MotorTelemetry {
  uint8_t id{0};
  bool ok{false}; ///< the status poll was answered
  uint16_t statusword{0};
  uint16_t error_code{0};
  float tracked_deg{0.0f};    ///< actual position read over CANopen (object 0x6064)
  int16_t torque_permille{0}; ///< actual torque (0x6077), per-thousand of rated
  float velocity_deg_s{0.0f}; ///< actual velocity (0x606C)
};

/// The CAN controller as CanBus::get_status() reports it (TWAI error state and
/// counters): the quickest way to tell a wiring / bitrate problem from a motor
/// that is off. Zeroed when the bus never started.
struct CanTelemetry {
  bool started{false};
  bool bus_off{false};
  const char *state{"not started"};
  uint16_t tx_error_count{0};
  uint16_t rx_error_count{0};
  uint32_t bus_error_count{0};
  uint32_t bus_off_count{0};
  uint32_t signal_errors{0}; ///< stuff + form + bit errors: termination, ground, bitrate
  uint32_t ack_errors{0};    ///< frames nobody acknowledged: bus open / nodes off
};

// Snapshot of what the controller knows. Unlike the RMD-based STEM, the Eyou
// motors report their actual output-shaft position over CANopen (object
// 0x6064, relative to the software zero): pair_deg is MEASURED and the pose is
// the forward kinematics of those measured positions; "still moving" is the
// DS402 statusword (target reached), not a velocity.
struct State {
  bool can_ok{false};
  bool target_valid{false};
  float target_x{0.0f};
  float target_y{0.0f};
  bool pose_valid{false};
  float pose_x{0.0f};
  float pose_y{0.0f};
  std::array<float, kPairCount> pair_deg{}; ///< primary motor of each pair, Pair order
  float seat_tilt_deg{0.0f};                ///< tilt offset applied on top of the IK seat angle
  std::array<MotorTelemetry, kMotorCount> motors{};
  bool motors_ok{false};
  uint16_t torque_limit_permille{0}; ///< the limit last written to every motor
  uint32_t rated_torque_mnm{0};      ///< 0x6076 of the first motor that answered (0: unknown)
  CanTelemetry can{};
};

// A solved end-effector target expressed as actuator-pair positions.
struct PoseCommand {
  float x_rel{0.0f};
  float y_rel{0.0f};
  std::array<float, kPairCount> pair_deg{}; ///< Pair order
  bool within_limits{false};
  IkSolution solution{};
};

class StemController : public espp::BaseComponent {
public:
  struct Config {
    PairedEyouActuator &left;
    PairedEyouActuator &right;
    PairedEyouActuator &seat;
    std::span<EyouMotor> motors; ///< every motor, for telemetry (ids reported as-is)
    bool can_ok{true};           ///< whether the CAN bus came up (reported in State)
    float min_deg{-60.0f};       ///< pair position limits (applied to every pair)
    float max_deg{60.0f};
    float mirror_tolerance_deg{1.0f}; ///< max |primary + secondary| for a pair to be usable
    // NOTE: EyouMotor moves at a pre-configured profile velocity (see
    // EyouMotor::configure_profile_position), not a per-call speed, so these are
    // kept only so the wire protocol / CLI keep the same rpm parameter shape;
    // they no longer affect motion.
    float default_move_rpm{2.0f}; ///< MOVE / HOME speed when the caller passes 0
    float default_set_rpm{5.0f};  ///< SET pair speed when the caller passes 0
    float max_rpm{10.0f};         ///< requested speeds are clamped to (0, max_rpm]
    GeometryConfig geometry{};
    IkReference reference{kFullDownReference};
    /// Torque limit (0x6072, per-thousand of rated) written to every motor by
    /// set_torque_limit(); a request above max_torque_permille is refused, so
    /// the person who flashed the firmware bounds what a browser may ask for.
    uint16_t max_torque_permille{1000};
    /// Optional: the CAN bus, for the controller state in State::can. Null:
    /// the block reports "not started".
    CanBus *can_bus{nullptr};
    /// Optional: run before a STOP is enacted. A set-point stuck in the TWAI
    /// queue in front of the stop (nobody acknowledged it) would otherwise
    /// delay the stop until the driver gave up; CanBus::flush_pending() drops it.
    std::function<void()> flush_pending_commands{nullptr};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::INFO};
  };

  enum class Result : uint8_t {
    Ok,
    Unreachable,
    OutOfLimits,
    ActuatorFailed,
    InvalidArgument,
    ZeroMismatch
  };
  static const char *to_string(Result result);

  explicit StemController(const Config &config);

  const Config &config() const { return config_; }

  /// Solve the IK for a target (relative to the reference) and map it to pair
  /// positions. No motion. Returns false if the point is unreachable; check
  /// `out.within_limits` for the position-limit test.
  bool solve(float x_rel, float y_rel, PoseCommand &out) const;

  /// Move the end effector. Unlike the raw actuator API this REJECTS targets
  /// whose pair angles fall outside the limits instead of silently clamping
  /// them (a clamped pose is not the pose that was asked for).
  Result move_to(float x_rel, float y_rel, float rpm, PoseCommand *out = nullptr);
  Result home(float rpm, PoseCommand *out = nullptr);

  /// Drive one pair to an absolute position (clamped by the pair itself).
  Result set_pair(Pair pair, float degrees, float rpm);

  /// Seat tilt: an offset (pair degrees, positive = positive seat-pair
  /// rotation) added to the IK's level seat angle on every move, so the seat
  /// keeps the chosen tilt as the linkage moves. Re-commands the seat pair
  /// immediately; rejected if the resulting seat angle leaves the limits.
  Result set_seat_tilt(float tilt_deg, float rpm);
  float seat_tilt() const { return seat_tilt_deg_.load(); }

  /// Stop every pair where it is: halt (profile deceleration, drives stay
  /// enabled and HOLD the linkage), and disable the pairs whose halt was not
  /// confirmed, so a pair that did not answer coasts rather than keeps moving.
  /// The old behaviour disabled everything: the linkage then sagged under its
  /// own weight (found on the single-actuator stand - the drive has a halt).
  bool stop();
  bool release_brakes();
  /// Clear the DS402 Fault on every motor that reports one (controlword fault
  /// reset) and bring its pair back to Operation Enabled. A fault that comes
  /// straight back (its cause still present - an overload, a limit) leaves the
  /// motor faulted and returns false. Also run by release_brakes()' enable.
  bool reset_fault();
  /// Write the torque limit (0x6072, per-thousand of rated) to every motor.
  /// Refused above Config::max_torque_permille (Result::OutOfLimits); a motor
  /// that did not confirm makes it ActuatorFailed (the others keep the new
  /// value, State reports the one asked for).
  Result set_torque_limit(uint16_t torque_permille);
  /// The same in newton-metres, through the rated torque read from the motors
  /// (InvalidArgument when it is not known yet: nothing to scale by).
  Result set_torque_limit_nm(float torque_nm);
  uint16_t torque_limit() const { return torque_limit_permille_.load(); }
  /// Rated torque (0x6076, mNm) of the first motor that answered; 0 until one
  /// did. Read at the first set_torque_limit() / snapshot() and kept.
  uint32_t rated_torque_mnm() const { return rated_torque_mnm_.load(); }
  /// per-mille of rated -> Nm with the known rated torque (0 when unknown).
  float permille_to_nm(uint16_t permille) const {
    return static_cast<float>(permille) * static_cast<float>(rated_torque_mnm_.load()) / 1.0e6f;
  }
  /// Re-zero one pair (or all when `pair` is empty). Clears the tracked target.
  bool zero(std::optional<Pair> pair, bool align);
  /// Persist every pair's software zero to NVS.
  bool save_zero();
  /// Read every motor's saved software zero without changing active offsets.
  bool read_zero(std::array<std::array<int32_t, 2>, kPairCount> &zero_offset_pulses);
  /// Load every pair's software zero from NVS, then run check_pairs(). Clears the
  /// tracked target. Returns the NVS load result; see has_zero_mismatch().
  bool load_zero();
  /// Check every pair is mirrored within mirror_tolerance_deg. A mismatched pair
  /// rejects all move commands until a new zero removes the mismatch.
  bool check_pairs();
  bool has_zero_mismatch() const;

  State snapshot(bool read_motors);

private:
  // stop() with mutex_ already held (command_pose's halt on a partial pose).
  bool stop_unlocked();
  PairedEyouActuator &actuator(Pair pair);
  const PairedEyouActuator &actuator(Pair pair) const;
  float effective_rpm(float rpm, float fallback) const;
  Result command_pose(const PoseCommand &pose, float rpm);
  // Map an IK solution (angles relative to the reference) to the sign
  // convention the pairs are commanded in (seat tilt included).
  std::array<float, kPairCount> pose_to_pair_deg(const IkSolution &solution) const;
  bool check_pair_unlocked(Pair pair);
  bool any_zero_mismatch_unlocked() const;

  Config config_;
  mutable std::mutex mutex_;
  std::array<bool, kPairCount> zero_mismatch_{};
  std::atomic<uint16_t> torque_limit_permille_{0}; // atomic: read by the lock-free torque_limit()
  std::atomic<uint32_t> rated_torque_mnm_{0};      // atomic: read by the lock-free getters
  // Read the rated torque once, from the first motor that answers (mutex_ held).
  void ensure_rated_torque_unlocked();
  std::atomic<float> seat_tilt_deg_{0.0f}; // atomic: read by the lock-free solve()
  bool target_valid_{false};
  float target_x_{0.0f};
  float target_y_{0.0f};
};

} // namespace stem
