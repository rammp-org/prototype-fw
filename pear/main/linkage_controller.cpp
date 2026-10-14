#include "linkage_controller.hpp"

#include <algorithm>
#include <cmath>

namespace {
constexpr uint16_t kDs402StateMask = 0x006F;
constexpr uint16_t kDs402OperationEnabled = 0x0027;
constexpr uint16_t kDs402FaultBit = 0x0008;
} // namespace

LinkageController::LinkageController(const Config &config)
    : espp::BaseComponent("LinkageController", config.log_level)
    , motors_(config.motors)
    , geometry_(config.geometry)
    , profile_(config.profile)
    , drive_enabled_(config.drive_enabled)
    , status_stale_timeout_(config.status_stale_timeout)
    , status_read_timeout_ms_(config.status_read_timeout_ms)
    , status_timer_({.name = "link_status",
                     .period = config.status_poll_period,
                     .callback = [this]() { return status_step(); },
                     .auto_start = true,
                     .stack_size_bytes = 6 * 1024,
                     .priority = 5}) {
  for (const Corner corner : pear::kCorners) {
    motor_status_[pear::index(corner)].id = motors_[pear::index(corner)].node_id();
    // the request clamp is the intersection of the table row and the geometry
    min_deg_[corner] = std::max(config.table_min_deg[corner], geometry_.min_angle_deg);
    max_deg_[corner] = std::min(config.table_max_deg[corner], geometry_.max_angle_deg);
    if (min_deg_[corner] > max_deg_[corner]) {
      logger_.error("Corner {}: the seat-axis table range [{}, {}] and the geometry range [{}, "
                    "{}] do not overlap; the corner cannot be commanded",
                    pear::corner_name(corner), config.table_min_deg[corner],
                    config.table_max_deg[corner], geometry_.min_angle_deg, geometry_.max_angle_deg);
    }
  }
}

LinkageController::~LinkageController() { status_timer_.cancel(); }

bool LinkageController::initialize() {
  bool ok = true;
  for (const Corner corner : pear::kCorners) {
    auto &motor = motors_[pear::index(corner)];
    const char *name = pear::corner_name(corner);
    if (!motor.configure_profile_position(profile_)) {
      logger_.error("Linkage {} (node {}): profile-position configuration failed", name,
                    motor.node_id());
      ok = false;
      continue;
    }
    if (!motor.enable_drive()) {
      logger_.error("Linkage {} (node {}): enable failed", name, motor.node_id());
      ok = false;
      continue;
    }
    const bool zero_loaded = motor.load_zero();
    if (!zero_loaded) {
      logger_.warn("Linkage {} (node {}): no saved zero in NVS - positions are relative to the "
                   "absolute encoder until `zero` + `save_zero`",
                   name, motor.node_id());
    }
    // the motor's own clamp is the geometry range: a target can never leave it
    // whatever the table says
    motor.set_position_limits(geometry_.min_angle_deg, geometry_.max_angle_deg);
    std::lock_guard<std::mutex> lock(mutex_);
    motor_status_[pear::index(corner)].zero_loaded = zero_loaded;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  initialized_ = ok;
  return ok;
}

float LinkageController::min_angle_deg(Corner corner) const { return min_deg_[corner]; }
float LinkageController::max_angle_deg(Corner corner) const { return max_deg_[corner]; }

const char *LinkageController::refusal_reason_locked() const {
  if (drive_enabled_ && drive_enabled_())
    return "the base is driving (disable driving first)";
  if (!initialized_)
    return "the linkage motors are not initialized";
  // a motor whose status has not been read recently may have disappeared:
  // its last operation_enabled is no evidence, so treat it as unhealthy
  const auto now = std::chrono::steady_clock::now();
  size_t index = 0;
  const auto unhealthy =
      std::find_if(motor_status_.begin(), motor_status_.end(), [&](const MotorStatus &m) {
        const bool stale = !m.valid || (now - motor_status_time_[index++]) > status_stale_timeout_;
        return stale || m.faulted || !m.operation_enabled;
      });
  if (unhealthy == motor_status_.end())
    return nullptr;
  // static strings per corner, so the reason can be returned without a buffer
  static constexpr std::array<const char *, pear::kCornerCount> kNotReported = {
      "linkage FR has not reported yet", "linkage FL has not reported yet",
      "linkage BR has not reported yet", "linkage BL has not reported yet"};
  static constexpr std::array<const char *, pear::kCornerCount> kFaulted = {
      "linkage FR is faulted", "linkage FL is faulted", "linkage BR is faulted",
      "linkage BL is faulted"};
  static constexpr std::array<const char *, pear::kCornerCount> kNotEnabled = {
      "linkage FR is not enabled", "linkage FL is not enabled", "linkage BR is not enabled",
      "linkage BL is not enabled"};
  static constexpr std::array<const char *, pear::kCornerCount> kStale = {
      "linkage FR status is stale (no reply)", "linkage FL status is stale (no reply)",
      "linkage BR status is stale (no reply)", "linkage BL status is stale (no reply)"};
  index = static_cast<size_t>(unhealthy - motor_status_.begin());
  if (!unhealthy->valid)
    return kNotReported[index];
  if ((now - motor_status_time_[index]) > status_stale_timeout_)
    return kStale[index];
  if (unhealthy->faulted)
    return kFaulted[index];
  return kNotEnabled[index];
}

bool LinkageController::clamp_target(Corner corner, float &angle_deg) {
  if (!std::isfinite(angle_deg))
    return false;
  if (min_deg_[corner] > max_deg_[corner])
    return false;
  const float clamped = std::clamp(angle_deg, min_deg_[corner], max_deg_[corner]);
  if (clamped != angle_deg) {
    logger_.warn_rate_limited("Linkage {} target {:.1f} deg clamped to {:.1f} deg",
                              pear::corner_name(corner), angle_deg, clamped);
  }
  angle_deg = clamped;
  return true;
}

void LinkageController::note_command_sent_locked() {
  last_command_ = std::chrono::steady_clock::now();
  have_command_ = true;
}

bool LinkageController::request_corner_angle(Corner corner, float angle_deg) {
  if (!clamp_target(corner, angle_deg)) {
    logger_.error_rate_limited("Linkage {}: rejecting target {} deg", pear::corner_name(corner),
                               angle_deg);
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (const char *reason = refusal_reason_locked()) {
    logger_.warn_rate_limited("Linkage {} -> {:.1f} deg refused: {}", pear::corner_name(corner),
                              angle_deg, reason);
    return false;
  }
  pending_target_deg_[corner] = angle_deg;
  pending_[pear::index(corner)] = true;
  logger_.info("Linkage {} -> {:.1f} deg", pear::corner_name(corner), angle_deg);
  return true;
}

bool LinkageController::request_pose(const Pose &pose) {
  if (!pear::geometry::is_finite(pose)) {
    logger_.error_rate_limited("Rejecting a non-finite pose request");
    return false;
  }
  Corners targets = pear::geometry::corner_angles_from_pose(geometry_, pose);
  const bool all_clamped =
      std::all_of(pear::kCorners.begin(), pear::kCorners.end(),
                  [&](Corner corner) { return clamp_target(corner, targets[corner]); });
  if (!all_clamped)
    return false;
  std::lock_guard<std::mutex> lock(mutex_);
  if (const char *reason = refusal_reason_locked()) {
    logger_.warn_rate_limited("Pose (elev {:.3f} m, fb {:.1f} deg, lat {:.1f} deg) refused: {}",
                              pose.elevation_m, pose.fb_tilt_deg, pose.lateral_tilt_deg, reason);
    return false;
  }
  for (const Corner corner : pear::kCorners) {
    pending_target_deg_[corner] = targets[corner];
    pending_[pear::index(corner)] = true;
  }
  logger_.info("Pose elev {:.3f} m fb {:.1f} deg lat {:.1f} deg -> FR {:.1f} FL {:.1f} BR {:.1f} "
               "BL {:.1f} deg",
               pose.elevation_m, pose.fb_tilt_deg, pose.lateral_tilt_deg, targets[Corner::FR],
               targets[Corner::FL], targets[Corner::BR], targets[Corner::BL]);
  return true;
}

bool LinkageController::zero(std::optional<Corner> corner) {
  bool ok = true;
  for (const Corner c : pear::kCorners) {
    if (corner.has_value() && *corner != c)
      continue;
    const bool zeroed = motors_[pear::index(c)].zero();
    if (!zeroed)
      logger_.error("Linkage {}: zero failed", pear::corner_name(c));
    std::lock_guard<std::mutex> lock(mutex_);
    if (zeroed)
      motor_status_[pear::index(c)].zero_loaded = true;
    ok = zeroed && ok;
  }
  return ok;
}

bool LinkageController::save_zero() {
  bool ok = true;
  for (const Corner c : pear::kCorners) {
    ok = motors_[pear::index(c)].save_zero() && ok;
  }
  return ok;
}

bool LinkageController::load_zero() {
  bool ok = true;
  for (const Corner c : pear::kCorners) {
    const bool loaded = motors_[pear::index(c)].load_zero();
    std::lock_guard<std::mutex> lock(mutex_);
    if (loaded)
      motor_status_[pear::index(c)].zero_loaded = true;
    ok = loaded && ok;
  }
  return ok;
}

LinkageController::State LinkageController::state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  State state;
  state.motors = motor_status_;
  state.initialized = initialized_;
  for (const Corner corner : pear::kCorners) {
    const auto &m = motor_status_[pear::index(corner)];
    state.angles_deg[corner] = m.position_degrees;
    state.any_faulted = state.any_faulted || (m.valid && m.faulted);
    state.motors[pear::index(corner)].target_pending = pending_[pear::index(corner)];
  }
  state.pose = pear::geometry::pose_from_corner_angles(geometry_, state.angles_deg);
  state.enabled = initialized_ && refusal_reason_locked() == nullptr;
  if (have_command_) {
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_command_);
    state.command_age_ms = static_cast<uint16_t>(std::min<int64_t>(age.count(), 65535));
  }
  return state;
}

bool LinkageController::status_step() {
  // 1. enact the queued moves (all corners of a pose in one tick), outside the
  //    lock: move_absolute is a confirmed transaction with retries
  std::array<bool, pear::kCornerCount> to_move{};
  Corners targets{};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    to_move = pending_;
    targets = pending_target_deg_;
    pending_.fill(false);
    // a STOP / DRIVE that landed after the request was accepted: drop the move
    if (drive_enabled_ && drive_enabled_() &&
        std::any_of(to_move.begin(), to_move.end(), [](bool b) { return b; })) {
      logger_.warn("Queued linkage move dropped: the base started driving");
      to_move.fill(false);
    }
  }
  for (const Corner corner : pear::kCorners) {
    if (!to_move[pear::index(corner)])
      continue;
    auto &motor = motors_[pear::index(corner)];
    const bool moved = motor.move_absolute(targets[corner]);
    std::lock_guard<std::mutex> lock(mutex_);
    if (moved) {
      motor_status_[pear::index(corner)].target_degrees = targets[corner];
      note_command_sent_locked();
    } else {
      logger_.error("Linkage {}: move to {:.1f} deg failed", pear::corner_name(corner),
                    targets[corner]);
    }
  }

  // 2. poll one motor (round-robin)
  const size_t index = status_poll_index_;
  status_poll_index_ = (status_poll_index_ + 1) % motors_.size();
  auto &motor = motors_[index];
  uint16_t statusword = 0;
  float position = 0.0f;
  const bool status_ok = motor.get_statusword(statusword, status_read_timeout_ms_);
  const bool position_ok = status_ok && motor.get_position(position, status_read_timeout_ms_);
  const auto now = std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> lock(mutex_);
  auto &motor_status = motor_status_[index];
  if (status_ok) {
    const bool faulted = (statusword & kDs402FaultBit) != 0;
    if (faulted && !motor_status.faulted)
      logger_.error("Linkage {} (node {}) is FAULTED: statusword 0x{:04X} ({})",
                    pear::corner_name(static_cast<Corner>(index)), motor_status.id, statusword,
                    EyouMotor::ds402_state_name(statusword));
    motor_status.statusword = statusword;
    motor_status.faulted = faulted;
    motor_status.operation_enabled = (statusword & kDs402StateMask) == kDs402OperationEnabled;
    motor_status.valid = true;
    motor_status_time_[index] = now;
  }
  if (position_ok)
    motor_status.position_degrees = position;
  // stale until the first successful read, and again whenever the last
  // successful read is too old
  motor_status.stale =
      !motor_status.valid || (now - motor_status_time_[index]) > status_stale_timeout_;
  return false; // keep the timer running
}
