#include "stem_controller.hpp"

#include <algorithm>
#include <cmath>

namespace stem {

const char *to_string(Pair pair) {
  switch (pair) {
  case Pair::Left:
    return "left";
  case Pair::Right:
    return "right";
  case Pair::Seat:
    return "seat";
  }
  return "?";
}

std::optional<Pair> pair_from_name(std::string_view name) {
  if (name == "left")
    return Pair::Left;
  if (name == "right")
    return Pair::Right;
  if (name == "seat")
    return Pair::Seat;
  return std::nullopt;
}

std::optional<Pair> pair_from_index(uint8_t index) {
  if (index >= kPairCount)
    return std::nullopt;
  return static_cast<Pair>(index);
}

const char *StemController::to_string(Result result) {
  switch (result) {
  case Result::Ok:
    return "ok";
  case Result::Unreachable:
    return "unreachable";
  case Result::OutOfLimits:
    return "outside position limits";
  case Result::ActuatorFailed:
    return "actuator command failed";
  case Result::InvalidArgument:
    return "invalid argument";
  }
  return "?";
}

StemController::StemController(const Config &config)
    : espp::BaseComponent("StemController", config.log_level), config_(config) {
  for (uint8_t i = 0; i < kPairCount; ++i) {
    actuator(static_cast<Pair>(i)).set_position_limits(config_.min_deg, config_.max_deg);
  }
}

PairedActuator &StemController::actuator(Pair pair) {
  switch (pair) {
  case Pair::Left:
    return config_.left;
  case Pair::Right:
    return config_.right;
  case Pair::Seat:
    return config_.seat;
  }
  return config_.left;
}

const PairedActuator &StemController::actuator(Pair pair) const {
  return const_cast<StemController *>(this)->actuator(pair);
}

float StemController::effective_rpm(float rpm, float fallback) const {
  if (!std::isfinite(rpm) || rpm <= 0.0f) {
    rpm = fallback;
  }
  return std::clamp(rpm, 0.1f, config_.max_rpm);
}

std::array<float, kPairCount> StemController::pose_to_pair_deg(const IkSolution &solution) const {
  // The pairs are mounted so that positive pair rotation is negative crank
  // rotation (this is the sign convention the original CLI `move` used). The
  // IK's seat angle keeps the seat level; the tilt offset rides on top of it.
  return {-solution.theta1_deg, -solution.theta2_deg,
          -solution.m3_angle_deg + seat_tilt_deg_.load()};
}

StemController::Result StemController::set_seat_tilt(float tilt_deg, float rpm) {
  if (!std::isfinite(tilt_deg)) {
    return Result::InvalidArgument;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  // New seat angle: the IK level angle of the current target plus the tilt, or
  // -- with no target -- the tracked seat angle shifted by the tilt change.
  const float previous_tilt = seat_tilt_deg_.load();
  float seat_deg = 0.0f;
  if (target_valid_) {
    IkSolution solution{};
    if (!solve_ik_for_m3_reference(target_x_, target_y_, config_.reference, solution,
                                   config_.geometry)) {
      return Result::Unreachable; // cannot happen for an accepted target
    }
    seat_deg = -solution.m3_angle_deg + tilt_deg;
  } else {
    seat_deg = config_.seat.get_position()[0] - previous_tilt + tilt_deg;
  }
  if (seat_deg < config_.min_deg || seat_deg > config_.max_deg) {
    logger_.warn("seat tilt {:.1f} deg would put the seat at {:.1f} deg, outside [{}, {}]",
                 tilt_deg, seat_deg, config_.min_deg, config_.max_deg);
    return Result::OutOfLimits;
  }
  const float speed = effective_rpm(rpm, config_.default_set_rpm);
  if (!config_.seat.set_position(seat_deg, speed)) {
    return Result::ActuatorFailed;
  }
  seat_tilt_deg_.store(tilt_deg);
  logger_.info("seat tilt {:.2f} deg -> seat pair {:.2f} deg @ {:.1f} rpm", tilt_deg, seat_deg,
               speed);
  return Result::Ok;
}

bool StemController::solve(float x_rel, float y_rel, PoseCommand &out) const {
  if (!std::isfinite(x_rel) || !std::isfinite(y_rel)) {
    return false;
  }
  IkSolution solution{};
  if (!solve_ik_for_m3_reference(x_rel, y_rel, config_.reference, solution, config_.geometry)) {
    return false;
  }
  out.x_rel = x_rel;
  out.y_rel = y_rel;
  out.solution = solution;
  out.pair_deg = pose_to_pair_deg(solution);
  out.within_limits = std::all_of(out.pair_deg.begin(), out.pair_deg.end(), [&](float deg) {
    return deg >= config_.min_deg && deg <= config_.max_deg;
  });
  return true;
}

StemController::Result StemController::command_pose(const PoseCommand &pose, float rpm) {
  const float speed = effective_rpm(rpm, config_.default_move_rpm);
  bool all_ok = true;
  for (uint8_t i = 0; i < kPairCount; ++i) {
    const auto pair = static_cast<Pair>(i);
    if (!actuator(pair).set_position(pose.pair_deg[i], speed)) {
      logger_.error("{} pair rejected position {:.2f} deg", stem::to_string(pair),
                    pose.pair_deg[i]);
      all_ok = false;
    }
  }
  logger_.info("move x={:.1f} y={:.1f} -> left={:.2f} right={:.2f} seat={:.2f} deg @ {:.1f} rpm{}",
               pose.x_rel, pose.y_rel, pose.pair_deg[0], pose.pair_deg[1], pose.pair_deg[2], speed,
               all_ok ? "" : " (PARTIAL FAILURE)");
  if (!all_ok) {
    // The pairs no longer agree with any single target.
    target_valid_ = false;
    return Result::ActuatorFailed;
  }
  target_valid_ = true;
  target_x_ = pose.x_rel;
  target_y_ = pose.y_rel;
  return Result::Ok;
}

StemController::Result StemController::move_to(float x_rel, float y_rel, float rpm,
                                               PoseCommand *out) {
  if (!std::isfinite(x_rel) || !std::isfinite(y_rel)) {
    return Result::InvalidArgument;
  }
  PoseCommand pose{};
  if (!solve(x_rel, y_rel, pose)) {
    logger_.warn("move x={:.1f} y={:.1f}: unreachable", x_rel, y_rel);
    return Result::Unreachable;
  }
  if (out) {
    *out = pose;
  }
  if (!pose.within_limits) {
    logger_.warn("move x={:.1f} y={:.1f}: pair angles ({:.1f}, {:.1f}, {:.1f}) outside [{}, {}]",
                 x_rel, y_rel, pose.pair_deg[0], pose.pair_deg[1], pose.pair_deg[2],
                 config_.min_deg, config_.max_deg);
    return Result::OutOfLimits;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  return command_pose(pose, rpm);
}

StemController::Result StemController::home(float rpm, PoseCommand *out) {
  return move_to(0.0f, 0.0f, rpm, out);
}

StemController::Result StemController::set_pair(Pair pair, float degrees, float rpm) {
  if (!std::isfinite(degrees)) {
    return Result::InvalidArgument;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const float speed = effective_rpm(rpm, config_.default_set_rpm);
  const bool ok = actuator(pair).set_position(degrees, speed);
  // A single pair moved on its own: the end effector is no longer at a target.
  target_valid_ = false;
  logger_.info("set {} -> {:.2f} deg @ {:.1f} rpm: {}", stem::to_string(pair), degrees, speed,
               ok ? "ok" : "FAILED");
  return ok ? Result::Ok : Result::ActuatorFailed;
}

bool StemController::stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  // NOTE: stopping does not update the tracked positions (they record what was
  // commanded, and the motors have no absolute output-shaft feedback), so a
  // pair stopped mid-move must be re-zeroed before its tracked position can be
  // trusted again.
  const bool right_ok = config_.right.stop();
  const bool left_ok = config_.left.stop();
  const bool seat_ok = config_.seat.stop();
  logger_.warn("STOP: right={} left={} seat={}", right_ok, left_ok, seat_ok);
  return right_ok && left_ok && seat_ok;
}

bool StemController::release_brakes() {
  std::lock_guard<std::mutex> lock(mutex_);
  const bool right_ok = config_.right.release_brake();
  const bool left_ok = config_.left.release_brake();
  const bool seat_ok = config_.seat.release_brake();
  logger_.info("brakes released: right={} left={} seat={}", right_ok, left_ok, seat_ok);
  return right_ok && left_ok && seat_ok;
}

bool StemController::zero(std::optional<Pair> pair, bool align) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (pair.has_value()) {
    actuator(*pair).zero_position(align);
    logger_.info("{} pair zeroed{}", stem::to_string(*pair), align ? " (aligned)" : "");
  } else {
    for (uint8_t i = 0; i < kPairCount; ++i) {
      actuator(static_cast<Pair>(i)).zero_position(align);
    }
    logger_.info("all pairs zeroed{}", align ? " (aligned)" : "");
  }
  target_valid_ = false;
  return true;
}

State StemController::snapshot(bool read_motors) {
  std::lock_guard<std::mutex> lock(mutex_);
  State state{};
  state.can_ok = config_.can_ok;
  state.target_valid = target_valid_;
  state.target_x = target_x_;
  state.target_y = target_y_;
  for (uint8_t i = 0; i < kPairCount; ++i) {
    state.pair_deg[i] = actuator(static_cast<Pair>(i)).get_position()[0];
  }
  state.seat_tilt_deg = seat_tilt_deg_.load();
  // Tracked pair positions -> crank angles (inverse of pose_to_pair_deg) -> pose
  IkSolution fk{};
  if (solve_fk_reference(-state.pair_deg[static_cast<uint8_t>(Pair::Left)],
                         -state.pair_deg[static_cast<uint8_t>(Pair::Right)], config_.reference, fk,
                         config_.geometry)) {
    state.pose_valid = true;
    state.pose_x = fk.m3[0] - config_.reference.x;
    state.pose_y = fk.m3[1] - config_.reference.y;
  }
  state.motors_ok = read_motors;
  const size_t count = std::min(config_.motors.size(), kMotorCount);
  for (size_t i = 0; i < count; ++i) {
    auto &motor = config_.motors[i];
    auto &telemetry = state.motors[i];
    telemetry.id = motor.get_motor_id();
    telemetry.tracked_deg = motor.get_position();
    if (read_motors) {
      MotorActuator::Status status{};
      telemetry.ok = motor.read_status(status, 50);
      if (telemetry.ok) {
        telemetry.temperature_c = static_cast<int8_t>(std::clamp(status.temperature_c, -128, 127));
        telemetry.torque_raw = status.torque_raw;
        telemetry.velocity_rpm = status.velocity_rpm;
      }
      state.motors_ok = state.motors_ok && telemetry.ok;
    }
  }
  return state;
}

} // namespace stem
