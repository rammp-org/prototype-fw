#include "linkage_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string_view>

namespace {
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr uint32_t kReadTimeoutMs = 50;
constexpr uint32_t kSlowEveryTicks = 20; ///< error code / torque limit re-read cadence
} // namespace

const char *LinkageController::to_string(Result result) {
  switch (result) {
  case Result::Ok:
    return "ok";
  case Result::Unreachable:
    return "unreachable";
  case Result::OutOfLimits:
    return "outside the motor limits";
  case Result::NoMotor:
    return "a crank has no motor configured (node id 0)";
  case Result::NotEnabled:
    return "a motor is not enabled (enable the drives first)";
  case Result::MotorFailed:
    return "the motor command failed (see the device log)";
  case Result::NoPose:
    return "no measured pose to move relative to";
  case Result::InvalidArgument:
    return "invalid argument";
  }
  return "?";
}

LinkageController::LinkageController(const Config &config)
    : espp::BaseComponent("LinkageController", config.log_level)
    , config_(config) {}

LinkageController::Settings LinkageController::settings() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return settings_;
}

bool LinkageController::apply_settings(const Settings &settings, std::string &why) {
  if (!settings.validate(config_.max_torque_permille, why))
    return false;
  std::lock_guard<std::mutex> lock(mutex_);
  const Settings previous = settings_;
  settings_ = settings;
  target_.reset();
  for (uint8_t i = 0; i < kMotorCount; ++i) {
    auto &motor = *config_.motors[i];
    const auto &m = settings_.motors[i];
    if (m.node_id != previous.motors[i].node_id) {
      // the crank's motor changed: follow it (a node id of 0 keeps the old
      // binding but the crank counts as undriven)
      if (m.node_id != 0 && !motor.set_node_id(m.node_id)) {
        why = "motor " + std::to_string(i + 1) + ": could not switch to node " +
              std::to_string(m.node_id);
        settings_ = previous;
        return false;
      }
      commissioned_[i] = false;
      measured_theta_.reset();
    }
    // limits live in the component, no bus traffic
    motor.set_position_limits(m.min_deg, m.max_deg);
    if (!present_unlocked(i)) {
      commissioned_[i] = false;
      continue;
    }
    // the drive's objects: now if it answers, else when it appears
    commissioned_[i] = config_.can_ok && commission_unlocked(i);
    settings_dirty_[i] = true;
  }
  logger_.info("settings applied: motors {} / {}, cranks {} / {} mm, bar {} / coupler {} mm",
               settings_.motors[0].node_id, settings_.motors[1].node_id, settings_.geometry.crank1,
               settings_.geometry.crank2, settings_.geometry.bar, settings_.geometry.coupler);
  return true;
}

bool LinkageController::commission_unlocked(uint8_t i) {
  auto &motor = *config_.motors[i];
  const auto &m = settings_.motors[i];
  // the torque limit FIRST: nothing moves harder than this
  if (!motor.set_max_torque(m.torque_limit_permille))
    return false;
  // NMT start + profile position mode + target = actual: a fresh drive (mode
  // 0) would otherwise refuse the enable sequence
  if (!motor.prepare_profile_position())
    logger_.warn("motor {} (node {}): could not select profile position mode (enable may fail)",
                 i + 1, m.node_id);
  motor.set_profile_position_config(
      {.velocity_degrees_per_second = settings_.profile_velocity_dps,
       .acceleration_degrees_per_second_squared = settings_.profile_accel_dps2,
       .deceleration_degrees_per_second_squared = settings_.profile_decel_dps2});
  motor.set_position_limits(m.min_deg, m.max_deg);
  if (!motor.load_zero())
    logger_.warn("motor {} (node {}): no saved software zero; its angle is the encoder's "
                 "absolute position until one is set",
                 i + 1, m.node_id);
  logger_.info("motor {} (node {}) commissioned: torque limit {} permille, limits [{}, {}] deg",
               i + 1, m.node_id, m.torque_limit_permille, m.min_deg, m.max_deg);
  return true;
}

void LinkageController::poll(Status &status) {
  std::lock_guard<std::mutex> lock(mutex_);
  ++tick_;
  status.can_ok = config_.can_ok;
  if (config_.can_bus != nullptr && config_.can_ok) {
    const auto c = config_.can_bus->get_status();
    const std::string_view state{c.state != nullptr ? c.state : ""};
    using Can = linkage_protocol::CanStatus;
    status.can.state = c.bus_off                                         ? Can::kBusOff
                       : state.find("passive") != std::string_view::npos ? Can::kErrorPassive
                       : state.find("warning") != std::string_view::npos ? Can::kErrorWarning
                       : state.find("active") != std::string_view::npos  ? Can::kErrorActive
                                                                         : Can::kUnknown;
    status.can.tx_error_count = c.tx_error_count;
    status.can.rx_error_count = c.rx_error_count;
    status.can.bus_error_count = c.bus_error_count;
    status.can.bus_off_count = c.bus_off_count;
    status.can.signal_errors = c.stuff_errors + c.form_errors + c.bit_errors;
    status.can.ack_errors = c.ack_errors;
  }
  bool all_comm = true, all_enabled = true, any_fault = false, any_moving = false;
  std::array<float, 2> theta{};
  bool have_theta = true;
  int present_count = 0;
  for (uint8_t i = 0; i < kMotorCount; ++i) {
    auto &ms = status.motors[i];
    auto &motor = *config_.motors[i];
    const auto &m = settings_.motors[i];
    ms.node_id = m.node_id;
    ms.present = present_unlocked(i);
    ms.torque_limit_permille = m.torque_limit_permille;
    ms.zero_set = motor.zero_offset_pulses().has_value();
    if (!ms.present || !config_.can_ok) {
      ms.comm_ok = ms.enabled = ms.faulted = ms.target_reached = false;
      ms.angle_deg = kNaN;
      ms.velocity_dps = 0.0f;
      ms.torque_permille = ms.current_permille = 0;
      have_theta = false;
      continue;
    }
    ++present_count;
    float angle = 0.0f, velocity = 0.0f;
    int16_t torque = 0, current = 0;
    uint16_t statusword = 0;
    const bool ok =
        motor.get_position(angle, kReadTimeoutMs) && motor.get_velocity(velocity, kReadTimeoutMs) &&
        motor.get_torque(torque, kReadTimeoutMs) && motor.get_current(current, kReadTimeoutMs) &&
        motor.get_statusword(statusword, kReadTimeoutMs);
    ms.comm_ok = ok;
    if (!ok) {
      ms.enabled = false;
      ms.faulted = true; // fail-safe: no answer shows as a fault
      ms.target_reached = false;
      ms.angle_deg = kNaN;
      all_comm = false;
      all_enabled = false;
      any_fault = true;
      have_theta = false;
      commissioned_[i] = false; // commission again when it is back
      continue;
    }
    if (!commissioned_[i]) {
      // the drive just appeared (powered after boot, node changed, came back)
      commissioned_[i] = commission_unlocked(i);
      settings_dirty_[i] = true;
      rated_torque_cnm_[i] = 0; // another drive may be on this node now
    }
    if (rated_torque_cnm_[i] == 0) {
      // the scale of the per-mille torque figures: read once per drive
      uint32_t rated_mnm = 0;
      if (motor.get_rated_torque(rated_mnm, kReadTimeoutMs) && rated_mnm != 0) {
        rated_torque_cnm_[i] = static_cast<uint16_t>(std::min<uint32_t>(rated_mnm / 10, 0xFFFF));
      }
    }
    ms.rated_torque_cnm = rated_torque_cnm_[i];
    ms.angle_deg = angle;
    ms.velocity_dps = velocity;
    ms.torque_permille = torque;
    ms.current_permille = current;
    ms.statusword = statusword;
    ms.enabled = (statusword & 0x006F) == 0x0027;
    ms.faulted = (statusword & 0x0008) != 0;
    ms.target_reached = (statusword & 0x0400) != 0;
    if ((tick_ % kSlowEveryTicks) == 0 || settings_dirty_[i]) {
      settings_dirty_[i] = false;
      uint16_t error_code = 0;
      if (motor.get_error_code(error_code, kReadTimeoutMs))
        ms.error_code = error_code;
      uint16_t limit = 0;
      if (motor.get_max_torque(limit, kReadTimeoutMs))
        ms.torque_limit_permille = limit;
    }
    all_enabled = all_enabled && ms.enabled;
    any_fault = any_fault || ms.faulted;
    any_moving = any_moving || (ms.enabled && !ms.target_reached);
    theta[i] = m.motor_to_theta(angle);
  }
  status.comm_ok = present_count > 0 && all_comm;
  status.enabled = present_count > 0 && all_enabled;
  status.faulted = any_fault;
  status.moving = any_moving;
  status.theta1_deg = status.theta2_deg = status.bar_angle_deg = status.x = status.y = kNaN;
  status.pose_valid = false;
  if (have_theta && present_count == kMotorCount) {
    fivebar::Pose pose{};
    status.theta1_deg = theta[0];
    status.theta2_deg = theta[1];
    if (fivebar::solve_fk(theta[0], theta[1], settings_.geometry, pose)) {
      status.pose_valid = true;
      status.x = pose.m3[0];
      status.y = pose.m3[1];
      status.bar_angle_deg = pose.bar_angle_deg;
      measured_theta_ = theta;
    } else {
      measured_theta_.reset();
    }
  } else {
    measured_theta_.reset();
  }
  status.target_valid = target_.has_value();
  status.target_x = target_ ? (*target_)[0] : kNaN;
  status.target_y = target_ ? (*target_)[1] : kNaN;
}

std::array<float, 2> LinkageController::reference_unlocked() const {
  if (measured_theta_)
    return *measured_theta_;
  return {settings_.home_theta1_deg, settings_.home_theta2_deg};
}

bool LinkageController::solve_unlocked(float x, float y, IkResult &out) const {
  out = IkResult{};
  out.x = x;
  out.y = y;
  if (!std::isfinite(x) || !std::isfinite(y))
    return false;
  const auto ref = reference_unlocked();
  fivebar::Pose pose{};
  if (!fivebar::solve_ik(x, y, settings_.geometry, ref[0], ref[1], pose))
    return false;
  out.reachable = true;
  out.pose = pose;
  out.theta1_deg = pose.theta1_deg;
  out.theta2_deg = pose.theta2_deg;
  out.motor_deg = {settings_.motors[0].theta_to_motor(pose.theta1_deg),
                   settings_.motors[1].theta_to_motor(pose.theta2_deg)};
  out.within_limits = settings_.motors[0].within_limits(out.motor_deg[0]) &&
                      settings_.motors[1].within_limits(out.motor_deg[1]);
  return true;
}

bool LinkageController::solve(float x, float y, IkResult &out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return solve_unlocked(x, y, out);
}

LinkageController::Result
LinkageController::command_unlocked(const std::array<float, kMotorCount> &motor_deg,
                                    const char *what) {
  for (uint8_t i = 0; i < kMotorCount; ++i) {
    if (!present_unlocked(i))
      return Result::NoMotor;
    if (!commissioned_[i])
      return Result::NotEnabled; // not answering (or not yet commissioned)
    uint16_t statusword = 0;
    if (!config_.motors[i]->get_statusword(statusword, kReadTimeoutMs) ||
        (statusword & 0x006F) != 0x0027)
      return Result::NotEnabled;
  }
  for (uint8_t i = 0; i < kMotorCount; ++i) {
    if (!config_.motors[i]->move_absolute(motor_deg[i], true)) {
      logger_.error("{}: motor {} refused {:.2f} deg", what, i + 1, motor_deg[i]);
      if (i == 1) {
        // the first crank is already on its way: stop it rather than twist
        // the linkage toward half a pose
        config_.motors[0]->halt();
      }
      return Result::MotorFailed;
    }
  }
  logger_.info("{}: motors -> {:.2f} / {:.2f} deg", what, motor_deg[0], motor_deg[1]);
  return Result::Ok;
}

LinkageController::Result LinkageController::move_to(float x, float y) {
  if (!std::isfinite(x) || !std::isfinite(y))
    return Result::InvalidArgument;
  std::lock_guard<std::mutex> lock(mutex_);
  IkResult ik{};
  if (!solve_unlocked(x, y, ik)) {
    logger_.warn("move to ({:.1f}, {:.1f}): unreachable", x, y);
    return Result::Unreachable;
  }
  if (!ik.within_limits) {
    logger_.warn("move to ({:.1f}, {:.1f}): motors {:.1f} / {:.1f} deg outside the limits", x, y,
                 ik.motor_deg[0], ik.motor_deg[1]);
    return Result::OutOfLimits;
  }
  const auto result = command_unlocked(ik.motor_deg, "move to");
  if (result == Result::Ok)
    target_ = fivebar::Point{x, y};
  return result;
}

LinkageController::Result LinkageController::move_by(float dx, float dy) {
  if (!std::isfinite(dx) || !std::isfinite(dy))
    return Result::InvalidArgument;
  fivebar::Point from{};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // relative to the target when one is pending, else to the measured pose
    if (target_) {
      from = *target_;
    } else if (measured_theta_) {
      fivebar::Pose pose{};
      if (!fivebar::solve_fk((*measured_theta_)[0], (*measured_theta_)[1], settings_.geometry,
                             pose))
        return Result::NoPose;
      from = pose.m3;
    } else {
      return Result::NoPose;
    }
  }
  return move_to(from[0] + dx, from[1] + dy);
}

LinkageController::Result LinkageController::home() {
  std::lock_guard<std::mutex> lock(mutex_);
  const std::array<float, kMotorCount> motor_deg = {
      settings_.motors[0].theta_to_motor(settings_.home_theta1_deg),
      settings_.motors[1].theta_to_motor(settings_.home_theta2_deg)};
  if (!settings_.motors[0].within_limits(motor_deg[0]) ||
      !settings_.motors[1].within_limits(motor_deg[1]))
    return Result::OutOfLimits;
  const auto result = command_unlocked(motor_deg, "home");
  if (result == Result::Ok) {
    fivebar::Pose pose{};
    if (fivebar::solve_fk(settings_.home_theta1_deg, settings_.home_theta2_deg, settings_.geometry,
                          pose))
      target_ = pose.m3;
    else
      target_.reset();
  }
  return result;
}

LinkageController::Result LinkageController::set_motor(uint8_t motor, float degrees) {
  if (motor >= kMotorCount || !std::isfinite(degrees))
    return Result::InvalidArgument;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!present_unlocked(motor))
    return Result::NoMotor;
  if (!settings_.motors[motor].within_limits(degrees))
    return Result::OutOfLimits;
  uint16_t statusword = 0;
  if (!config_.motors[motor]->get_statusword(statusword, kReadTimeoutMs) ||
      (statusword & 0x006F) != 0x0027)
    return Result::NotEnabled;
  target_.reset(); // one crank on its own: the end effector is not at a target
  if (!config_.motors[motor]->move_absolute(degrees, true))
    return Result::MotorFailed;
  logger_.info("motor {} -> {:.2f} deg", motor + 1, degrees);
  return Result::Ok;
}

template <typename Fn>
bool LinkageController::for_each_motor(uint8_t motor, const char *what, Fn &&fn) {
  if (motor != kAllMotors && motor >= kMotorCount)
    return false;
  std::lock_guard<std::mutex> lock(mutex_);
  bool ok = true;
  bool any = false;
  for (uint8_t i = 0; i < kMotorCount; ++i) {
    if (motor != kAllMotors && motor != i)
      continue;
    if (!present_unlocked(i))
      continue;
    any = true;
    const bool this_ok = fn(i, *config_.motors[i]);
    logger_.info("{} motor {} (node {}): {}", what, i + 1, settings_.motors[i].node_id,
                 this_ok ? "ok" : "FAILED");
    ok = ok && this_ok;
  }
  return any && ok;
}

bool LinkageController::enable(uint8_t motor) {
  return for_each_motor(motor, "enable", [](uint8_t, EyouMotor &m) { return m.enable_drive(); });
}

void LinkageController::flush_pending_unlocked() {
  // A set-point nobody acknowledged, stuck in the TWAI queue, would delay the
  // halt / disable frames behind it until the driver gave up on it.
  if (config_.can_bus != nullptr && config_.can_ok) {
    config_.can_bus->flush_pending();
  }
}

bool LinkageController::recover_can() {
  if (config_.can_bus == nullptr || !config_.can_ok) {
    return false;
  }
  return config_.can_bus->recover();
}

EyouMotor::NodeIdResult LinkageController::set_drive_node_id(uint8_t motor, uint8_t new_id) {
  if (motor >= kMotorCount || new_id < 1 || new_id > 127) {
    return EyouMotor::NodeIdResult::Failed;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!present_unlocked(motor)) {
    logger_.error("re-address: no motor configured on crank {}", motor + 1);
    return EyouMotor::NodeIdResult::Failed;
  }
  auto &m = *config_.motors[motor];
  const uint8_t old_id = m.node_id();
  const auto result = m.set_drive_node_id(new_id);
  switch (result) {
  case EyouMotor::NodeIdResult::Applied:
    // the motor object follows the drive; so do the settings (the caller
    // persists them), and the drive is commissioned again as a newcomer
    settings_.motors[motor].node_id = new_id;
    commissioned_[motor] = false;
    rated_torque_cnm_[motor] = 0;
    target_.reset();
    logger_.warn("crank {}: drive re-addressed {} -> {} and followed", motor + 1, old_id, new_id);
    break;
  case EyouMotor::NodeIdResult::SavedNeedsPowerCycle:
    logger_.warn("crank {}: new node id {} saved in the drive; it keeps {} until power-cycled",
                 motor + 1, new_id, old_id);
    break;
  case EyouMotor::NodeIdResult::Failed:
    logger_.error("crank {}: re-addressing {} -> {} failed, nothing changed", motor + 1, old_id,
                  new_id);
    break;
  }
  return result;
}

bool LinkageController::disable(uint8_t motor) {
  flush_pending_unlocked();
  const bool ok =
      for_each_motor(motor, "disable", [](uint8_t, EyouMotor &m) { return m.disable_drive(); });
  std::lock_guard<std::mutex> lock(mutex_);
  target_.reset();
  return ok;
}

bool LinkageController::reset_fault(uint8_t motor) {
  return for_each_motor(motor, "reset fault",
                        [](uint8_t, EyouMotor &m) { return m.reset_fault(); });
}

bool LinkageController::halt(uint8_t motor) {
  flush_pending_unlocked();
  const bool ok = for_each_motor(motor, "halt", [](uint8_t, EyouMotor &m) { return m.halt(); });
  std::lock_guard<std::mutex> lock(mutex_);
  target_.reset();
  return ok;
}

LinkageController::Result LinkageController::zero_motor(uint8_t motor, uint8_t mode, float value) {
  if (motor >= kMotorCount || mode > 2 || !std::isfinite(value))
    return Result::InvalidArgument;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!present_unlocked(motor))
    return Result::NoMotor;
  auto &m = *config_.motors[motor];
  bool ok = false;
  switch (mode) {
  case linkage_protocol::ZeroMotor::kModeHere:
    ok = m.zero();
    break;
  case linkage_protocol::ZeroMotor::kModeMotorAngle:
    ok = m.zero_at(value);
    break;
  case linkage_protocol::ZeroMotor::kModeCrankAngle: {
    const float motor_deg = settings_.motors[motor].theta_to_motor(value);
    logger_.info("motor {}: the crank is at {:.2f} deg -> the motor reads {:.2f} deg here",
                 motor + 1, value, motor_deg);
    ok = m.zero_at(motor_deg);
    break;
  }
  default:
    return Result::InvalidArgument;
  }
  target_.reset();
  measured_theta_.reset();
  return ok ? Result::Ok : Result::MotorFailed;
}

bool LinkageController::save_zero(uint8_t motor) {
  return for_each_motor(motor, "save zero", [](uint8_t, EyouMotor &m) { return m.save_zero(); });
}

bool LinkageController::load_zero(uint8_t motor) {
  const bool ok =
      for_each_motor(motor, "load zero", [](uint8_t, EyouMotor &m) { return m.load_zero(); });
  std::lock_guard<std::mutex> lock(mutex_);
  target_.reset();
  measured_theta_.reset();
  return ok;
}
