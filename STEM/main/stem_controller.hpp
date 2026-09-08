#pragma once

// The one place that turns end-effector targets into actuator-pair commands.
// Shared by the console CLI and the USB (dispatcher) module, so both see the
// same limits, the same IK reference convention and the same tracked state.
// All public methods are serialized by an internal mutex (the CLI thread and
// the USB RX worker call in concurrently).

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <span>
#include <string_view>

#include "base_component.hpp"
#include "ik_5bar.hpp"
#include "motor_actuator.hpp"
#include "paired_actuator.hpp"

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
  int8_t temperature_c{0};
  int16_t torque_raw{0};
  float velocity_rpm{0.0f};
  float tracked_deg{0.0f}; ///< firmware-tracked (commanded) position
};

// Snapshot of what the controller knows. NOTE: the RMD status frame only carries
// the motor-side single-turn encoder (pre-gearbox), so there is no measured
// output-shaft angle; the pose here is the FK of the *tracked* pair positions
// (what has been commanded since the last zero), with the motor velocities as
// the "still moving" indicator.
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
    PairedActuator &left;
    PairedActuator &right;
    PairedActuator &seat;
    std::span<MotorActuator> motors; ///< every motor, for telemetry (ids reported as-is)
    bool can_ok{true};               ///< whether the CAN bus came up (reported in State)
    float min_deg{-60.0f};           ///< pair position limits (applied to every pair)
    float max_deg{60.0f};
    float default_move_rpm{2.0f}; ///< MOVE / HOME speed when the caller passes 0
    float default_set_rpm{5.0f};  ///< SET pair speed when the caller passes 0
    float max_rpm{10.0f};         ///< requested speeds are clamped to (0, max_rpm]
    GeometryConfig geometry{};
    IkReference reference{kFullDownReference};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::INFO};
  };

  enum class Result : uint8_t { Ok, Unreachable, OutOfLimits, ActuatorFailed, InvalidArgument };
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

  bool stop();
  bool release_brakes();
  /// Re-zero one pair (or all when `pair` is empty). Clears the tracked target.
  bool zero(std::optional<Pair> pair, bool align);

  State snapshot(bool read_motors);

private:
  PairedActuator &actuator(Pair pair);
  const PairedActuator &actuator(Pair pair) const;
  float effective_rpm(float rpm, float fallback) const;
  Result command_pose(const PoseCommand &pose, float rpm);
  // Map an IK solution (angles relative to the reference) to the sign
  // convention the pairs are commanded in (seat tilt included).
  std::array<float, kPairCount> pose_to_pair_deg(const IkSolution &solution) const;

  Config config_;
  mutable std::mutex mutex_;
  std::atomic<float> seat_tilt_deg_{0.0f}; // atomic: read by the lock-free solve()
  bool target_valid_{false};
  float target_x_{0.0f};
  float target_y_{0.0f};
};

} // namespace stem
