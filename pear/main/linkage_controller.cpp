#include "linkage_controller.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace {
constexpr uint16_t kDs402StateMask = 0x006F;
constexpr uint16_t kDs402OperationEnabled = 0x0027;
constexpr uint16_t kDs402FaultBit = 0x0008;
constexpr uint16_t kDs402TargetReached = 0x0400; // profile position: target reached
} // namespace

LinkageController::LinkageController(const Config &config)
    : espp::BaseComponent("LinkageController", config.log_level)
    , motors_(config.motors)
    , geometry_(config.geometry)
    , profile_(config.profile)
    , jog_max_deg_(config.jog_max_deg)
    , range_tolerance_deg_(config.range_tolerance_deg)
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
    // The hardware (hw_config: the corner's range in the geometry) is the
    // request clamp, the same range the pose solver uses per corner. The
    // seat-axis table row is what the HMI shows and jogs within: it must cover
    // the corner's range, or the HMI cannot reach part of it (it does not
    // widen the clamp either way). An intersection of the two used to be the
    // clamp, which silently pinned every target when the ranges barely
    // overlapped.
    min_deg_[corner] = geometry_.min_angle_deg[corner];
    max_deg_[corner] = geometry_.max_angle_deg[corner];
    direction_[corner] = config.direction[corner] < 0.0f ? -1.0f : 1.0f;
    if (config.table_min_deg[corner] > min_deg_[corner] ||
        config.table_max_deg[corner] < max_deg_[corner]) {
      logger_.warn("Corner {}: the seat-axis table row allows [{}, {}] deg but the corner's range "
                   "is [{}, {}] deg: the HMI's L{} jog cannot reach the whole range - update "
                   "RAMMP_SEAT_AXIS_TABLE (pear/spec/rammp-rtps-pear.patch)",
                   pear::corner_name(corner), config.table_min_deg[corner],
                   config.table_max_deg[corner], min_deg_[corner], max_deg_[corner],
                   pear::index(corner) + 1);
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
    // the motor's own clamp is the corner's range, in the motor's frame (a
    // mirrored corner swaps the ends): a target can never leave it whatever
    // the table says. A jog past the range is clamped by the motor, which is
    // the intent: a jog moves a corner back INTO its range.
    const float end_a = to_motor_deg(corner, min_deg_[corner]);
    const float end_b = to_motor_deg(corner, max_deg_[corner]);
    motor.set_position_limits(std::min(end_a, end_b), std::max(end_a, end_b));
    std::lock_guard<std::mutex> lock(mutex_);
    motor_status_[pear::index(corner)].zero_loaded = zero_loaded;
    if (zero_loaded) {
      // the status timer already runs: a position it cached (or is sampling)
      // in the pre-zero frame is dropped, like after zero() / load_zero()
      motor_status_[pear::index(corner)].valid = false;
      frame_generation_.fetch_add(1);
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  initialized_ = ok;
  init_finished_ = true;
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

bool LinkageController::clamp_target_locked(Corner corner, float &angle_deg) {
  if (!std::isfinite(angle_deg))
    return false;
  if (min_deg_[corner] > max_deg_[corner])
    return false;
  const float clamped = std::clamp(angle_deg, min_deg_[corner], max_deg_[corner]);
  if (clamped != angle_deg) {
    logger_.warn_rate_limited("Linkage {} target {:.1f} deg clamped to {:.1f} deg (range [{:.1f}, "
                              "{:.1f}])",
                              pear::corner_name(corner), angle_deg, clamped, min_deg_[corner],
                              max_deg_[corner]);
  }
  angle_deg = clamped;
  return true;
}

bool LinkageController::out_of_range_locked(Corner corner) const {
  const auto &status = motor_status_[pear::index(corner)];
  if (!status.valid || status.stale || !std::isfinite(status.position_degrees))
    return false; // unknown; the stale / not-read refusals cover this
  return status.position_degrees < min_deg_[corner] - range_tolerance_deg_ ||
         status.position_degrees > max_deg_[corner] + range_tolerance_deg_;
}

bool LinkageController::queue_corner_locked(Corner corner, float angle_deg, const char *what) {
  if (const char *reason = refusal_reason_locked()) {
    logger_.warn_rate_limited("Linkage {} {} -> {:.1f} deg refused: {}", pear::corner_name(corner),
                              what, angle_deg, reason);
    return false;
  }
  pending_target_deg_[corner] = angle_deg;
  pending_[pear::index(corner)] = true;
  desired_pose_.reset(); // the corners no longer describe one requested pose
  ++request_generation_;
  logger_.info("Linkage {} {} -> {:.1f} deg", pear::corner_name(corner), what, angle_deg);
  return true;
}

bool LinkageController::request_jog(Corner corner, float delta_deg) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::isfinite(delta_deg) || std::abs(delta_deg) > jog_max_deg_ || delta_deg == 0.0f) {
    logger_.error_rate_limited("Linkage {}: a jog is at most +/-{:.1f} deg (got {})",
                               pear::corner_name(corner), jog_max_deg_, delta_deg);
    return false;
  }
  const auto &status = motor_status_[pear::index(corner)];
  if (!status.valid || status.stale) {
    logger_.error_rate_limited("Linkage {}: no fresh position to jog from",
                               pear::corner_name(corner));
    return false;
  }
  // relative to the MEASURED angle, not clamped: this is how a corner outside
  // its range is brought back (the motor's own clamp still bounds it)
  return queue_corner_locked(corner, status.position_degrees + delta_deg, "jog");
}

void LinkageController::note_command_sent_locked() {
  last_command_ = std::chrono::steady_clock::now();
  have_command_ = true;
}

bool LinkageController::request_corner_angle(Corner corner, float angle_deg) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::isfinite(angle_deg)) {
    logger_.error_rate_limited("Linkage {}: rejecting target {} deg", pear::corner_name(corner),
                               angle_deg);
    return false;
  }
  if (out_of_range_locked(corner)) {
    // Outside the range nothing is clamped: a clamp would snap the corner to
    // a limit, many degrees at once (that broke a linkage). Only a jog-sized
    // move relative to the measured angle is accepted (the HMI's one-step jog
    // from the value it was shown, when that value is current); anything
    // else is an error, the corner is reported as such in MibStatus.
    const auto &status = motor_status_[pear::index(corner)];
    const float delta = angle_deg - status.position_degrees;
    if (std::abs(delta) > jog_max_deg_) {
      logger_.error_rate_limited(
          "Linkage {} is OUTSIDE its range [{:.1f}, {:.1f}] deg at {:.1f} deg: {:.1f} deg "
          "refused; only jogs within +/-{:.1f} deg of the current position are accepted "
          "(`jog {} <deg>`)",
          pear::corner_name(corner), min_deg_[corner], max_deg_[corner], status.position_degrees,
          angle_deg, jog_max_deg_, pear::corner_name(corner));
      return false;
    }
    return queue_corner_locked(corner, angle_deg, "jog (out of range)");
  }
  if (!clamp_target_locked(corner, angle_deg)) {
    logger_.error_rate_limited("Linkage {}: rejecting target {} deg", pear::corner_name(corner),
                               angle_deg);
    return false;
  }
  return queue_corner_locked(corner, angle_deg, "");
}

bool LinkageController::request_pose_component(PoseComponent component, float value) {
  if (!std::isfinite(value)) {
    logger_.error_rate_limited("Rejecting a non-finite pose component");
    return false;
  }
  // base pose, modification and commit in ONE critical section: the CLI and
  // the RTPS callbacks call this concurrently, and an interleaved command must
  // not be overwritten from a stale base
  std::lock_guard<std::mutex> lock(mutex_);
  Pose pose = desired_pose_ ? *desired_pose_ : measured_pose_locked();
  switch (component) {
  case PoseComponent::ELEVATION:
    pose.elevation_m = value;
    break;
  case PoseComponent::FB_TILT:
    pose.fb_tilt_deg = value;
    break;
  case PoseComponent::LATERAL_TILT:
    pose.lateral_tilt_deg = value;
    break;
  }
  return commit_pose_locked(pose);
}

LinkageController::PosePlan LinkageController::plan_pose(const Pose &pose) const {
  PosePlan plan;
  plan.requested = pose;
  if (!pear::geometry::is_finite(pose)) {
    plan.refusal = "the pose is not finite";
    return plan;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  plan.measured = measured_pose_locked();
  const Corners raw = pear::geometry::corner_angles_from_pose(geometry_, pose);
  Corners targets{};
  for (const Corner corner : pear::kCorners) {
    auto &c = plan.corners[pear::index(corner)];
    const auto &status = motor_status_[pear::index(corner)];
    c.measured_valid = status.valid && !status.stale;
    c.measured_deg = status.position_degrees;
    c.measured_height_m = pear::geometry::height_from_angle(geometry_, corner, c.measured_deg);
    c.raw_target_deg = raw[corner];
    // corner_angles_from_pose already clamps to the corner's range; say so
    const float unclamped =
        geometry_.level_angle_deg[corner] +
        pear::geometry::rad_to_deg(
            std::asin(std::clamp(pear::geometry::corner_heights_from_pose(geometry_, pose)[corner] /
                                     std::max(geometry_.crank_length_m, 1e-6f),
                                 -1.0f, 1.0f)));
    c.raw_target_deg = unclamped;
    c.target_deg = std::clamp(unclamped, min_deg_[corner], max_deg_[corner]);
    c.clamped = c.target_deg != unclamped;
    c.target_height_m = pear::geometry::height_from_angle(geometry_, corner, c.target_deg);
    c.delta_deg = c.target_deg - c.measured_deg;
    c.out_of_range = out_of_range_locked(corner);
    targets[corner] = c.target_deg;
  }
  plan.resulting = pear::geometry::pose_from_corner_angles(geometry_, targets);
  for (const Corner corner : pear::kCorners) {
    if (plan.corners[pear::index(corner)].out_of_range) {
      plan.refusal = "a corner is outside its range (jog it back first)";
      return plan;
    }
  }
  plan.refusal = refusal_reason_locked();
  return plan;
}

bool LinkageController::request_pose(const Pose &pose) {
  if (!pear::geometry::is_finite(pose)) {
    logger_.error_rate_limited("Rejecting a non-finite pose request");
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  return commit_pose_locked(pose);
}

LinkageController::Pose LinkageController::measured_pose_locked() const {
  Corners angles{};
  for (const Corner corner : pear::kCorners)
    angles[corner] = motor_status_[pear::index(corner)].position_degrees;
  return pear::geometry::pose_from_corner_angles(geometry_, angles);
}

bool LinkageController::commit_pose_locked(const Pose &pose) {
  for (const Corner corner : pear::kCorners) {
    if (out_of_range_locked(corner)) {
      logger_.error_rate_limited(
          "Pose refused: linkage {} is OUTSIDE its range [{:.1f}, {:.1f}] deg at {:.1f} deg; "
          "jog it back first (`jog {} <deg>`)",
          pear::corner_name(corner), min_deg_[corner], max_deg_[corner],
          motor_status_[pear::index(corner)].position_degrees, pear::corner_name(corner));
      return false;
    }
  }
  Corners targets = pear::geometry::corner_angles_from_pose(geometry_, pose);
  const bool all_clamped =
      std::all_of(pear::kCorners.begin(), pear::kCorners.end(),
                  [&](Corner corner) { return clamp_target_locked(corner, targets[corner]); });
  if (!all_clamped)
    return false;
  if (const char *reason = refusal_reason_locked()) {
    logger_.warn_rate_limited("Pose (elev {:.3f} m, fb {:.1f} deg, lat {:.1f} deg) refused: {}",
                              pose.elevation_m, pose.fb_tilt_deg, pose.lateral_tilt_deg, reason);
    return false;
  }
  for (const Corner corner : pear::kCorners) {
    pending_target_deg_[corner] = targets[corner];
    pending_[pear::index(corner)] = true;
  }
  desired_pose_ = pose;
  ++request_generation_;
  logger_.info("Pose elev {:.3f} m fb {:.1f} deg lat {:.1f} deg -> FR {:.1f} FL {:.1f} BR {:.1f} "
               "BL {:.1f} deg",
               pose.elevation_m, pose.fb_tilt_deg, pose.lateral_tilt_deg, targets[Corner::FR],
               targets[Corner::FL], targets[Corner::BR], targets[Corner::BL]);
  return true;
}

void LinkageController::invalidate_targets_locked() {
  pending_.fill(false);
  desired_pose_.reset();
  ++request_generation_;
}

void LinkageController::forget_batch_pose_locked(uint32_t generation, const char *why) {
  // the pose this batch was meant to reach was never reached: a later single
  // component command must start from the MEASURED pose, not from it. A
  // request queued since owns the current generation and keeps its pose.
  if (request_generation_ != generation || !desired_pose_)
    return;
  logger_.info("Desired pose forgotten ({})", why);
  desired_pose_.reset();
}

bool LinkageController::zero(std::optional<Corner> corner) {
  // no move may execute while the coordinate system changes, and whatever was
  // queued was expressed in the old one
  std::lock_guard<std::mutex> moves(move_mutex_);
  if (moving_mask_.load() != 0) {
    // a software zero only shifts the offset: an in-flight profile move would
    // continue toward its old absolute target in the new coordinate system
    logger_.error("Zero refused: a linkage is still moving (wait for target reached)");
    return false;
  }
  // mutex_ is held for the whole rebase: no request is accepted (or pose
  // measured) against positions of the old frame, and the cached positions are
  // invalidated so requests stay refused until the next poll reads the new one
  std::lock_guard<std::mutex> lock(mutex_);
  invalidate_targets_locked();
  bool ok = true;
  for (const Corner c : pear::kCorners) {
    if (corner.has_value() && *corner != c)
      continue;
    const bool zeroed = motors_[pear::index(c)].zero();
    if (!zeroed)
      logger_.error("Linkage {}: zero failed", pear::corner_name(c));
    if (zeroed) {
      motor_status_[pear::index(c)].zero_loaded = true;
      motor_status_[pear::index(c)].valid = false;
    }
    ok = zeroed && ok;
  }
  frame_generation_.fetch_add(1);
  return ok;
}

bool LinkageController::save_zero() {
  // serialized with zero() / load_zero() so the persisted set of offsets is
  // never a mix of before and after one of those
  std::lock_guard<std::mutex> moves(move_mutex_);
  bool ok = true;
  for (const Corner c : pear::kCorners) {
    ok = motors_[pear::index(c)].save_zero() && ok;
  }
  return ok;
}

bool LinkageController::load_zero() {
  std::lock_guard<std::mutex> moves(move_mutex_);
  if (moving_mask_.load() != 0) { // see zero()
    logger_.error("load_zero refused: a linkage is still moving (wait for target reached)");
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_); // see zero()
  invalidate_targets_locked();
  bool ok = true;
  for (const Corner c : pear::kCorners) {
    const bool loaded = motors_[pear::index(c)].load_zero();
    if (loaded) {
      motor_status_[pear::index(c)].zero_loaded = true;
      motor_status_[pear::index(c)].valid = false;
    }
    ok = loaded && ok;
  }
  frame_generation_.fetch_add(1);
  return ok;
}

LinkageController::State LinkageController::state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  State state;
  state.motors = motor_status_;
  state.initialized = initialized_;
  state.min_deg = min_deg_;
  state.max_deg = max_deg_;
  for (const Corner corner : pear::kCorners)
    if (motor_status_[pear::index(corner)].out_of_range)
      state.out_of_range_mask |= static_cast<uint8_t>(1u << pear::index(corner));
  state.halt_unconfirmed_mask = halt_unconfirmed_mask_.load();
  for (const Corner corner : pear::kCorners) {
    const auto &m = motor_status_[pear::index(corner)];
    state.angles_deg[corner] = m.position_degrees;
    state.any_faulted = state.any_faulted || (m.valid && m.faulted);
    state.motors[pear::index(corner)].target_pending = pending_[pear::index(corner)];
  }
  state.pose = measured_pose_locked();
  state.enabled = initialized_ && refusal_reason_locked() == nullptr;
  if (have_command_) {
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_command_);
    state.command_age_ms = static_cast<uint16_t>(std::min<int64_t>(age.count(), 65535));
  }
  return state;
}

bool LinkageController::halt_corner(Corner corner, const char *why) {
  const uint8_t bit = static_cast<uint8_t>(1u << pear::index(corner));
  auto &motor = motors_[pear::index(corner)];
  const bool first_attempt = (halt_unconfirmed_mask_.load() & bit) == 0;
  if (first_attempt)
    logger_.warn("Linkage {}: halting its move ({})", pear::corner_name(corner), why);
  if (!motor.disable_drive()) {
    // the move may still be running: keep the interlock and retry; shouted
    // once, then quiet until it confirms
    if (first_attempt) {
      logger_.error("Linkage {}: halt NOT confirmed; drive stays locked out until it is",
                    pear::corner_name(corner));
      halt_unconfirmed_mask_.fetch_or(bit);
    }
    return false;
  }
  if (halt_unconfirmed_mask_.fetch_and(static_cast<uint8_t>(~bit)) & bit)
    logger_.info("Linkage {}: halt confirmed", pear::corner_name(corner));
  // the power stage is off, so the motion is over whatever the re-enable
  // does; a motor left disabled is refused by the interlock (not enabled)
  if (!motor.enable_drive())
    logger_.error("Linkage {}: re-enable after the halt failed", pear::corner_name(corner));
  moving_mask_.fetch_and(static_cast<uint8_t>(~bit));
  return true;
}

bool LinkageController::status_step() {
  // 1. enact the queued moves (all corners of a pose in one tick), outside the
  //    lock: move_absolute is a confirmed transaction with retries
  {
    std::array<bool, pear::kCornerCount> to_move{};
    Corners targets{};
    std::lock_guard<std::mutex> moves(move_mutex_); // excludes zero() / load_zero()
    // 0. a move that did not report target reached in time is HALTED before the
    //    drive interlock is released: the accepted profile move would otherwise
    //    keep running under a base that may start driving. Power stage off
    //    (the Eyou's halt), then on again so the motor is ready for the next
    //    request (re-enabling does not restart the consumed set-point).
    if (const uint8_t mask = moving_mask_.load(); mask != 0) {
      // each corner times out on its own deadline; a corner whose halt did not
      // confirm keeps its bit (and the drive interlock) and is retried on
      // every step, not only after the timeout
      uint8_t to_halt = mask & halt_unconfirmed_mask_.load();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now = std::chrono::steady_clock::now();
        for (const Corner corner : pear::kCorners) {
          const uint8_t bit = static_cast<uint8_t>(1u << pear::index(corner));
          if ((mask & bit) != 0 && now - motion_started_[pear::index(corner)] > kMoveTimeout)
            to_halt = static_cast<uint8_t>(to_halt | bit);
        }
      }
      for (const Corner corner : pear::kCorners) {
        if ((to_halt & (1u << pear::index(corner))) == 0)
          continue;
        halt_corner(corner, "no target reached in time");
      }
    }
    uint32_t batch_generation = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      to_move = pending_;
      targets = pending_target_deg_;
      pending_.fill(false);
      batch_generation = request_generation_;
    }
    const bool have_work = std::any_of(to_move.begin(), to_move.end(), [](bool b) { return b; });
    // Only a batch with work marks itself moving (an idle tick must not make
    // DriveController::enable() refuse), and it does so BEFORE looking at the
    // drive (enable() does the mirror image), so base and linkage motion can
    // never overlap. The mark is cleared when this block ends.
    struct EnactingGuard {
      std::atomic<bool> &flag;
      ~EnactingGuard() { flag.store(false); }
    };
    std::optional<EnactingGuard> enacting_guard;
    if (have_work) {
      enacting_.store(true);
      enacting_guard.emplace(enacting_);
      // a DRIVE that landed after the request was accepted: drop the batch
      if (drive_enabled_ && drive_enabled_()) {
        logger_.warn("Queued linkage move dropped: the base started driving");
        to_move.fill(false);
        std::lock_guard<std::mutex> lock(mutex_);
        forget_batch_pose_locked(batch_generation, "batch dropped");
      }
    }
    // A batch is all-or-nothing for the platform: when a later corner cannot
    // take its move, the corners accepted earlier are halted again (power
    // stage off, then back on so they are ready; confirmed transactions) so
    // the base is never twisted against the failed corner. The interlock
    // bits of the halted corners are cleared only after their halts confirm.
    uint8_t accepted = 0;
    const auto halt_accepted = [&](const char *why) {
      for (const Corner done : pear::kCorners) {
        if ((accepted & (1u << pear::index(done))) == 0)
          continue;
        // an unconfirmed halt keeps the bit; the timeout path retries it
        halt_corner(done, why);
      }
      accepted = 0;
      std::lock_guard<std::mutex> lock(mutex_);
      forget_batch_pose_locked(batch_generation, "batch abandoned");
    };
    for (const Corner corner : pear::kCorners) {
      if (!to_move[pear::index(corner)])
        continue;
      // re-validated per corner: a refusal (driving, fault, stale) between two
      // confirmed moves stops the rest of the batch
      {
        std::unique_lock<std::mutex> lock(mutex_);
        if (const char *reason = refusal_reason_locked()) {
          logger_.warn("Linkage batch stopped before {}: {}", pear::corner_name(corner), reason);
          lock.unlock();
          halt_accepted(reason);
          break;
        }
      }
      auto &motor = motors_[pear::index(corner)];
      const bool moved = motor.move_absolute(to_motor_deg(corner, targets[corner]));
      const uint8_t bit = static_cast<uint8_t>(1u << pear::index(corner));
      if (!moved) {
        // the platform must not be twisted against a corner that did not take
        // its move: the rest of the batch is abandoned (and re-requested whole).
        // A failed move_absolute may still have been ACCEPTED by the drive
        // (the set-point went out, only the acknowledgement timed out), so this
        // corner counts as possibly moving and is halted with the rest.
        logger_.error("Linkage {}: move to {:.1f} deg failed; batch abandoned",
                      pear::corner_name(corner), targets[corner]);
        moving_mask_.fetch_or(bit);
        accepted = static_cast<uint8_t>(accepted | bit);
        halt_accepted("a later corner failed");
        break;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      motor_status_[pear::index(corner)].target_degrees = targets[corner];
      note_command_sent_locked();
      // physical motion starts now; the poll clears the bit on target reached
      moving_mask_.fetch_or(bit);
      accepted = static_cast<uint8_t>(accepted | bit);
      motion_started_[pear::index(corner)] = std::chrono::steady_clock::now();
    }
  }

  // 2. poll one motor (round-robin); not before initialize() is through with
  //    the motors (its NMT resets leave a node silent for ~100 ms)
  if (!init_finished_.load())
    return false;
  const size_t index = status_poll_index_;
  status_poll_index_ = (status_poll_index_ + 1) % motors_.size();
  auto &motor = motors_[index];
  uint16_t statusword = 0;
  float position = 0.0f;
  const uint32_t frame = frame_generation_.load();
  const bool status_ok = motor.get_statusword(statusword, status_read_timeout_ms_);
  const bool position_ok = status_ok && motor.get_position(position, status_read_timeout_ms_);
  const auto now = std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> lock(mutex_);
  if (frame != frame_generation_.load())
    return false; // sampled in a coordinate frame a zero / load_zero just replaced
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
    // a commanded corner is in motion until the drive reports target reached
    // (or the motion cannot continue: fault / power stage off)
    const uint8_t bit = static_cast<uint8_t>(1u << index);
    const bool settled =
        (statusword & kDs402TargetReached) != 0 || faulted || !motor_status.operation_enabled;
    if ((moving_mask_.load() & bit) != 0 && settled) {
      if (moving_mask_.fetch_and(static_cast<uint8_t>(~bit)) == bit)
        logger_.info("Linkage move complete");
    }
  }
  // fresh only after a COMPLETE sample: a statusword that reads while the
  // position object keeps failing must not keep an old position looking current
  const Corner corner = static_cast<Corner>(index);
  if (position_ok) {
    motor_status.position_degrees = from_motor_deg(corner, position);
    motor_status.valid = true;
    motor_status_time_[index] = now;
  }
  // stale until the first successful read, and again whenever the last
  // successful read is too old
  motor_status.stale =
      !motor_status.valid || (now - motor_status_time_[index]) > status_stale_timeout_;
  const bool out = out_of_range_locked(corner);
  if (out && !motor_status.out_of_range)
    logger_.error("Linkage {} is OUTSIDE its range [{:.1f}, {:.1f}] deg at {:.1f} deg: moves "
                  "refused except `jog {} <deg>` (+/-{:.1f} deg max)",
                  pear::corner_name(corner), min_deg_[corner], max_deg_[corner],
                  motor_status.position_degrees, pear::corner_name(corner), jog_max_deg_);
  else if (!out && motor_status.out_of_range)
    logger_.info("Linkage {} is back inside its range", pear::corner_name(corner));
  motor_status.out_of_range = out;
  return false; // keep the timer running
}
