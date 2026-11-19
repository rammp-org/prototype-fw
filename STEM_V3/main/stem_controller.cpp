#include "stem_controller.hpp"

#include <algorithm>
#include <cctype>
#include <string>
#include <cmath>
#include <cstdio>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace stem {

const char *to_string(Pair pair) {
  switch (pair) {
  case Pair::M1_Seat:
    return "M1_Seat";
  case Pair::M2_Seat:
    return "M2_Seat";
  case Pair::M3_Seat:
    return "M3_Seat";
  }
  return "?";
}

std::optional<Pair> pair_from_name(std::string_view name) {
  // M1_Seat / M2_Seat / M3_Seat, any case; "m1" / "m2" / "m3" for short
  std::string lower(name);
  for (auto &c : lower) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  if (lower == "m1_seat" || lower == "m1")
    return Pair::M1_Seat;
  if (lower == "m2_seat" || lower == "m2")
    return Pair::M2_Seat;
  if (lower == "m3_seat" || lower == "m3")
    return Pair::M3_Seat;
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
  case Result::ZeroMismatch:
    return "pair zero mismatch (re-zero required)";
  case Result::Measuring:
    return "measure mode is on (exit it first)";
  case Result::BrakesEngaged:
    return "the brakes are engaged (release them first)";
  }
  return "?";
}

StemController::StemController(const Config &config)
    : espp::BaseComponent("StemController", config.log_level)
    , config_(config) {
  for (uint8_t i = 0; i < kPairCount; ++i) {
    actuator(static_cast<Pair>(i)).set_position_limits(config_.min_deg[i], config_.max_deg[i]);
  }
}

PairedEyouActuator &StemController::actuator(Pair pair) {
  switch (pair) {
  case Pair::M1_Seat:
    return config_.m1_seat;
  case Pair::M2_Seat:
    return config_.m2_seat;
  case Pair::M3_Seat:
    return config_.m3_seat;
  }
  return config_.m1_seat;
}

const PairedEyouActuator &StemController::actuator(Pair pair) const {
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
  if (measuring_) {
    return Result::Measuring;
  }
  if (brakes_engaged_) {
    return Result::BrakesEngaged;
  }
  if (zero_mismatch_[static_cast<uint8_t>(Pair::M3_Seat)] && !check_pair_unlocked(Pair::M3_Seat)) {
    logger_.error("seat tilt rejected: seat pair zero mismatch");
    return Result::ZeroMismatch;
  }
  // New seat angle: the IK level angle of the current target plus the tilt, or
  // -- with no target -- the tracked seat angle shifted by the tilt change.
  const float previous_tilt = seat_tilt_deg_.load();
  if (run_) {
    if (!config_.operable.tilt_contains(tilt_deg)) {
      return Result::OutOfLimits;
    }
    seat_tilt_deg_.store(tilt_deg);
    const Result replanned = start_move_unlocked(target_x_, target_y_, run_->speed_deg_s);
    if (replanned != Result::Ok) {
      seat_tilt_deg_.store(previous_tilt);
    }
    return replanned;
  }
  float seat_deg = 0.0f;
  if (target_valid_) {
    IkSolution solution{};
    if (!solve_ik_for_m3_reference(target_x_, target_y_, config_.reference, solution,
                                   config_.geometry)) {
      return Result::Unreachable; // cannot happen for an accepted target
    }
    seat_deg = -solution.m3_angle_deg + tilt_deg;
  } else {
    std::array<float, 2> seat_position{};
    if (!config_.m3_seat.get_position(seat_position)) {
      // without the live seat angle the tilt change has no base: a failed
      // read must not turn a small adjustment into a move to `tilt_deg`
      logger_.error("seat tilt rejected: could not read the seat pair position");
      return Result::ActuatorFailed;
    }
    seat_deg = seat_position[0] - previous_tilt + tilt_deg;
  }
  constexpr auto kSeat = static_cast<uint8_t>(Pair::M3_Seat);
  if (!config_.operable.tilt_contains(tilt_deg)) {
    logger_.warn("seat tilt {:.1f} deg outside the operable range [{}, {}] deg from level",
                 tilt_deg, config_.operable.tilt_min_deg, config_.operable.tilt_max_deg);
    return Result::OutOfLimits;
  }
  if (seat_deg < config_.min_deg[kSeat] || seat_deg > config_.max_deg[kSeat]) {
    logger_.warn("seat tilt {:.1f} deg would put the seat at {:.1f} deg, outside [{}, {}]",
                 tilt_deg, seat_deg, config_.min_deg[kSeat], config_.max_deg[kSeat]);
    return Result::OutOfLimits;
  }
  (void)rpm; // EyouMotor moves at its pre-configured profile velocity, not a per-call speed.
  if (!config_.m3_seat.set_position(seat_deg)) {
    return Result::ActuatorFailed;
  }
  seat_tilt_deg_.store(tilt_deg);
  logger_.info("seat tilt {:.2f} deg -> seat pair {:.2f} deg", tilt_deg, seat_deg);
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
  bool in_pair_limits = true;
  for (size_t i = 0; i < kPairCount; ++i) {
    in_pair_limits = in_pair_limits && out.pair_deg[i] >= config_.min_deg[i] &&
                     out.pair_deg[i] <= config_.max_deg[i];
  }
  out.within_limits =
      in_pair_limits && config_.operable.contains(out.pair_deg, seat_tilt_deg_.load());
  return true;
}

StemController::Result StemController::move_to(float x_rel, float y_rel, float rpm,
                                               PoseCommand *out) {
  if (!std::isfinite(x_rel) || !std::isfinite(y_rel)) {
    return Result::InvalidArgument;
  }
  // Solve, limit-check and command under ONE lock: solve() folds in the seat
  // tilt, and set_seat_tilt() (which takes this mutex) must not change it
  // between the solve and the command.
  std::lock_guard<std::mutex> lock(mutex_);
  if (measuring_) {
    return Result::Measuring;
  }
  if (brakes_engaged_) {
    return Result::BrakesEngaged;
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
    logger_.warn("move x={:.1f} y={:.1f}: pair angles ({:.1f}, {:.1f}, {:.1f}) outside the "
                 "operable range (enc101 {:.1f}, enc103 {:.1f})",
                 x_rel, y_rel, pose.pair_deg[0], pose.pair_deg[1], pose.pair_deg[2],
                 -pose.pair_deg[0], -pose.pair_deg[1]);
    return Result::OutOfLimits;
  }
  if (any_zero_mismatch_unlocked() && recheck_mismatch_unlocked()) {
    logger_.error("move x={:.1f} y={:.1f} rejected: pair zero mismatch", x_rel, y_rel);
    return Result::ZeroMismatch;
  }
  const float speed = std::min(effective_rpm(rpm, config_.default_move_rpm) * 6.0f,
                               config_.max_move_speed_deg_s);
  return start_move_unlocked(x_rel, y_rel, speed);
}

bool StemController::moving() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return run_.has_value();
}

float StemController::tilt_of(const LevelTrajectory::Pairs &pairs) const {
  IkSolution fk{};
  if (!solve_fk_reference(-pairs[0], -pairs[1], config_.reference, fk, config_.geometry)) {
    return NAN;
  }
  return pairs[2] + fk.m3_angle_deg; // inverse of pose_to_pair_deg's seat
}

void StemController::restore_profile_unlocked() {
  for (uint8_t i = 0; i < kPairCount; ++i) {
    actuator(static_cast<Pair>(i)).set_profile_position_config(config_.profile);
  }
}

StemController::Result StemController::start_move_unlocked(float x_rel, float y_rel,
                                                           float speed_deg_s) {
  // From where the running move is (a re-plan keeps its speed), else from the
  // measured pairs.
  LevelTrajectory::Pairs start{};
  float v0 = 0.0f;
  if (run_) {
    start = run_->plan.at(run_->t);
    v0 = run_->rate * run_->plan.speed_at(run_->t);
  } else {
    for (uint8_t i = 0; i < kPairCount; ++i) {
      std::array<float, 2> position{};
      if (!actuator(static_cast<Pair>(i)).get_position(position, 100)) {
        logger_.error("move rejected: could not read the {} pair position",
                      stem::to_string(static_cast<Pair>(i)));
        return Result::ActuatorFailed;
      }
      start[i] = position[0];
    }
  }
  LevelTrajectoryLimits limits{};
  limits.speed_deg_s = speed_deg_s;
  limits.accel_deg_s2 = config_.move_accel_deg_s2;
  LevelTrajectory plan;
  const auto planned = plan.plan(start, x_rel, y_rel, seat_tilt_deg_.load(), config_.reference,
                                 config_.geometry, config_.operable, config_.min_deg,
                                 config_.max_deg, limits, v0);
  switch (planned) {
  case LevelTrajectory::Result::Ok:
    break;
  case LevelTrajectory::Result::StartUnknown:
    logger_.error("move rejected: the measured pairs ({:.1f}, {:.1f}) do not match a linkage pose",
                  start[0], start[1]);
    return Result::Unreachable;
  case LevelTrajectory::Result::Unreachable:
    logger_.warn("move x={:.1f} y={:.1f}: the straight path is unreachable at ({:.1f}, {:.1f})",
                 x_rel, y_rel, plan.fail_x, plan.fail_y);
    return Result::Unreachable;
  case LevelTrajectory::Result::OutOfLimits:
    logger_.warn("move x={:.1f} y={:.1f}: no path inside the operable range (the straight line "
                 "leaves it at ({:.1f}, {:.1f}) and no route around the edge was found)",
                 x_rel, y_rel, plan.fail_x, plan.fail_y);
    return Result::OutOfLimits;
  }
  const auto now = Clock::now();
  if (!run_) {
    // Profile velocity mode for the move: service() streams each pair's speed
    // (the plan's speed plus a position correction); the drives ramp smoothly
    // between speeds with kVelocityAccel. Target velocity starts at 0.
    for (uint8_t i = 0; i < kPairCount; ++i) {
      auto &pair = actuator(static_cast<Pair>(i));
      if (!pair.start_velocity_mode(kVelocityAccel)) {
        logger_.error("move rejected: the {} pair did not enter velocity mode",
                      stem::to_string(static_cast<Pair>(i)));
        run_.emplace(); // so stop_unlocked() returns every pair to a position hold
        stop_unlocked();
        return Result::ActuatorFailed;
      }
    }
  }
  run_.emplace();
  run_->plan = std::move(plan);
  run_->speed_deg_s = speed_deg_s;
  run_->last = now;
  run_->stalled_since = now;
  target_valid_ = true;
  target_x_ = x_rel;
  target_y_ = y_rel;
  const auto &end = run_->plan.end();
  logger_.info("level move x={:.1f} y={:.1f} -> M1_Seat={:.2f} M2_Seat={:.2f} M3_Seat={:.2f} deg, "
               "{:.1f} s at {:.1f} deg/s{}",
               x_rel, y_rel, end[0], end[1], end[2], run_->plan.duration(), speed_deg_s,
               run_->plan.routed() ? ", routed around the operable range's edge" : "");
  for (const auto &corner : run_->plan.route_corners()) {
    logger_.info("  via ({:.1f}, {:.1f})", corner.x, corner.y);
  }
  return Result::Ok;
}

void StemController::service() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!run_) {
    return;
  }
  auto &r = *run_;
  const auto now = Clock::now();
  const float dt = std::clamp(std::chrono::duration<float>(now - r.last).count(), 0.0f, 0.5f);
  r.last = now;

  LevelTrajectory::Pairs measured{};
  bool read_ok = true;
  for (uint8_t i = 0; i < kPairCount && read_ok; ++i) {
    std::array<float, 2> position{};
    read_ok = actuator(static_cast<Pair>(i)).get_position(position, 50);
    measured[i] = position[0];
  }
  if (!read_ok) {
    if (++r.read_failures >= 3) {
      logger_.error("level move STOPPED: position reads failed");
      stop_unlocked();
    }
    return;
  }
  r.read_failures = 0;

  // Level guard: the measured seat tilt against the plan's at this time. Lag
  // along the path does not count (the plan is level everywhere); a seat pair
  // that falls behind the others does.
  const auto planned = r.plan.at(r.t);
  const float deviation = tilt_of(measured) - tilt_of(planned);
  float lag = 0.0f;
  for (size_t i = 0; i < kPairCount; ++i) {
    lag = std::max(lag, std::fabs(measured[i] - planned[i]));
  }
  const float lag_limit = 2.0f + 0.5f * r.speed_deg_s;
  if (!std::isfinite(deviation) || std::fabs(deviation) > config_.level_abort_deg) {
    logger_.error("level move STOPPED: seat {:.1f} deg off level (limit {:.1f})", deviation,
                  config_.level_abort_deg);
    stop_unlocked();
    return;
  }
  // Slow the whole path instead of pausing: the plan's clock runs at `rate`
  // (0..1), so every pair slows together along the same level path. Full speed
  // while the seat is within half of level_hold_deg (and no pair lags), zero at
  // level_hold_deg; the rate itself changes at most kRateSlew per second, so
  // the drives decelerate and re-accelerate smoothly.
  constexpr float kRateSlew = 1.5f;
  const float hold = config_.level_hold_deg;
  const float level_rate = std::clamp((hold - std::fabs(deviation)) / (0.5f * hold), 0.0f, 1.0f);
  const float lag_rate = std::clamp((lag_limit - lag) / (0.5f * lag_limit), 0.0f, 1.0f);
  const float wanted = std::min(level_rate, lag_rate);
  r.rate += std::clamp(wanted - r.rate, -kRateSlew * dt, kRateSlew * dt);
  if (!r.slowed && r.rate < 0.5f) {
    r.slowed = true;
    logger_.warn("level move slowing: seat {:.2f} deg off level, max lag {:.1f} deg", deviation, lag);
  } else if (r.slowed && r.rate > 0.95f) {
    r.slowed = false;
    logger_.info("level move back to full speed");
  }
  if (r.rate > 0.05f) {
    r.stalled_since = now;
  } else if (now - r.stalled_since > std::chrono::milliseconds(config_.hold_timeout_ms)) {
    logger_.error("level move STOPPED: no progress for {} ms (seat {:.2f} deg off level, lag "
                  "{:.1f} deg) - a pair stalled? (torque limit)",
                  config_.hold_timeout_ms, deviation, lag);
    stop_unlocked();
    return;
  }

  // hard backstop: a pair beyond its position limits (velocity mode has no
  // drive-side target to stop at)
  for (size_t i = 0; i < kPairCount; ++i) {
    if (measured[i] < config_.min_deg[i] - 1.0f || measured[i] > config_.max_deg[i] + 1.0f) {
      logger_.error("level move STOPPED: the {} pair is at {:.1f} deg, outside [{}, {}]",
                    stem::to_string(static_cast<Pair>(i)), measured[i], config_.min_deg[i],
                    config_.max_deg[i]);
      stop_unlocked();
      return;
    }
  }

  r.t += dt * r.rate;
  const float duration = r.plan.duration();
  const auto &end = r.plan.end();
  if (r.t >= duration) {
    float remaining = 0.0f;
    for (size_t i = 0; i < kPairCount; ++i) {
      remaining = std::max(remaining, std::fabs(measured[i] - end[i]));
    }
    if (!r.at_end) {
      r.at_end = true;
      r.end_since = now;
    }
    if (r.settling) {
      // speeds were zeroed last tick: the drives have ramped down. Switch to
      // profile position holding exactly where each pair stopped - no final
      // correction move (that was the "click"): the pull below already brought
      // every pair within kEndTolerance of the target.
      if (now - r.settle_since < std::chrono::milliseconds(150)) {
        return;
      }
      run_.reset();
      bool ok = true;
      for (uint8_t i = 0; i < kPairCount; ++i) {
        ok = actuator(static_cast<Pair>(i)).hold_position() && ok;
      }
      restore_profile_unlocked();
      logger_.info("level move done ({:.2f} deg from the target){}", remaining,
                   ok ? "" : " - a pair did not take the final hold, check");
      return;
    }
    constexpr float kEndTolerance = 0.1f;
    if (remaining < kEndTolerance || now - r.end_since > std::chrono::milliseconds(2500)) {
      if (remaining >= kEndTolerance) {
        logger_.warn("level move: {:.2f} deg from the target after the settle time", remaining);
      }
      for (uint8_t i = 0; i < kPairCount; ++i) {
        actuator(static_cast<Pair>(i)).set_velocity(0.0f);
      }
      r.settling = true;
      r.settle_since = now;
      return;
    }
  }
  // Velocity command per pair: the plan's speed at (now + the command's
  // latency), scaled by the slow-down rate, plus a proportional pull toward the
  // plan's position. Both come from the same level path, so the seat stays level
  // while every pair tracks it.
  constexpr float kLatency = 0.05f;
  constexpr float kPositionGain = 2.0f; // 1/s: a 1 deg lag adds 2 deg/s
  constexpr float kProbe = 0.05f;
  const auto p0 = r.plan.at(r.t + kLatency * r.rate), p1 = r.plan.at(r.t + (kLatency + kProbe) * r.rate);
  const float v_max = 1.5f * r.speed_deg_s + 5.0f;
  std::array<float, kPairCount> velocity{};
  for (size_t i = 0; i < kPairCount; ++i) {
    const float feed = r.rate > 1e-3f ? (p1[i] - p0[i]) / kProbe : 0.0f; // already scaled by rate
    const float target = r.at_end ? end[i] : planned[i];
    velocity[i] = std::clamp(feed + kPositionGain * (target - measured[i]), -v_max, v_max);
  }
  // seat first: of the three, it is the one that keeps the seat level
  constexpr Pair order[] = {Pair::M3_Seat, Pair::M1_Seat, Pair::M2_Seat};
  for (const Pair pair : order) {
    const auto i = static_cast<uint8_t>(pair);
    if (!actuator(pair).set_velocity(velocity[i])) {
      logger_.error("level move STOPPED: the {} pair did not take its speed", stem::to_string(pair));
      stop_unlocked();
      return;
    }
  }
}

StemController::Result StemController::home(float rpm, PoseCommand *out) {
  return move_to(0.0f, 0.0f, rpm, out);
}

StemController::Result StemController::set_pair(Pair pair, float degrees, float rpm) {
  if (!std::isfinite(degrees)) {
    return Result::InvalidArgument;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (measuring_) {
    return Result::Measuring;
  }
  if (brakes_engaged_) {
    return Result::BrakesEngaged;
  }
  if (zero_mismatch_[static_cast<uint8_t>(pair)] && !check_pair_unlocked(pair)) {
    logger_.error("set {} rejected: pair zero mismatch", stem::to_string(pair));
    return Result::ZeroMismatch;
  }
  if (run_) {
    stop_unlocked(); // a lone pair inside a level move would tilt the seat
  }
  (void)rpm; // EyouMotor moves at its pre-configured profile velocity, not a per-call speed.
  const bool ok = actuator(pair).set_position(degrees);
  // A single pair moved on its own: the end effector is no longer at a target.
  target_valid_ = false;
  logger_.info("set {} -> {:.2f} deg: {}", stem::to_string(pair), degrees, ok ? "ok" : "FAILED");
  return ok ? Result::Ok : Result::ActuatorFailed;
}

bool StemController::stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  return stop_unlocked();
}

bool StemController::stop_unlocked() {
  const bool had_run = run_.has_value();
  run_.reset(); // service() stops streaming speeds
  if (had_run) {
    // velocity mode: zero every target speed first (releasing the halt below
    // would otherwise resume the last one)
    for (uint8_t i = 0; i < kPairCount; ++i) {
      actuator(static_cast<Pair>(i)).set_velocity(0.0f);
    }
  }
  // A set-point nobody acknowledged, stuck in the TWAI queue, would delay the
  // halt frames behind it: drop it first (CanBus::flush_pending, see Config).
  if (config_.flush_pending_commands) {
    config_.flush_pending_commands();
  }
  // Every drive starts decelerating NOW, at the stop deceleration: halting the
  // motors one by one (each waited until at rest) let the other motors - even
  // a pair's own partner on the same shaft - keep driving for seconds.
  for (auto &motor : config_.motors) {
    motor.request_halt(kStopDecel);
  }
  // Halt: the drives decelerate with the profile deceleration and HOLD, still
  // enabled (EyouMotor::halt raises the controlword's halt bit, waits for rest,
  // and makes the reached position the set-point). A pair whose halt was not
  // confirmed is disabled instead: coasting beats a pair that may still be
  // moving towards a target the others abandoned.
  PairedEyouActuator *pairs[] = {&config_.m2_seat, &config_.m1_seat, &config_.m3_seat};
  const char *names[] = {"M2_Seat", "M1_Seat", "M3_Seat"};
  bool all_ok = true;
  for (size_t i = 0; i < 3; ++i) {
    if (pairs[i]->halt()) {
      continue;
    }
    const bool disabled = pairs[i]->disable_drive();
    logger_.error("STOP: {} pair did not confirm the halt, drive {}", names[i],
                  disabled ? "disabled (coasting)" : "NOT disabled either");
    all_ok = false;
  }
  if (had_run) {
    // back to profile position, holding where each pair stopped
    for (uint8_t i = 0; i < kPairCount; ++i) {
      all_ok = actuator(static_cast<Pair>(i)).hold_position() && all_ok;
    }
  }
  restore_profile_unlocked(); // the stop deceleration / streaming profile -> the normal one
  logger_.warn("STOP: {}", all_ok ? "every pair halted and holding" : "see the errors above");
  // the move this interrupted never reached its target: a later seat tilt must
  // offset the MEASURED seat angle (the Eyou branch of set_seat_tilt), not the
  // stale final pose of that move
  target_valid_ = false;
  return all_ok;
}

void StemController::ensure_rated_torque_unlocked() {
  if (rated_torque_mnm_.load() != 0) {
    return;
  }
  for (auto &motor : config_.motors) {
    uint32_t rated = 0;
    if (motor.get_rated_torque(rated, 50) && rated != 0) {
      rated_torque_mnm_.store(rated);
      logger_.info("rated torque {} mNm (motor {}): torque limits can be given in Nm", rated,
                   motor.node_id());
      return;
    }
  }
}

StemController::Result StemController::set_torque_limit_nm(float torque_nm) {
  if (!std::isfinite(torque_nm) || torque_nm < 0.0f) {
    return Result::InvalidArgument;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ensure_rated_torque_unlocked();
  }
  const uint32_t rated = rated_torque_mnm_.load();
  if (rated == 0) {
    logger_.error("torque limit {:.2f} Nm: the rated torque is not known yet (no motor answered "
                  "0x6076), give it in per-mille of rated instead",
                  torque_nm);
    return Result::InvalidArgument;
  }
  const float permille = torque_nm * 1.0e6f / static_cast<float>(rated);
  if (permille > 65535.0f) {
    return Result::OutOfLimits;
  }
  return set_torque_limit(static_cast<uint16_t>(std::lround(permille)));
}

StemController::Result StemController::set_torque_limit(uint16_t torque_permille) {
  if (torque_permille > config_.max_torque_permille) {
    logger_.warn("torque limit {} permille refused: the device's ceiling is {} (menuconfig)",
                 torque_permille, config_.max_torque_permille);
    return Result::OutOfLimits;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  ensure_rated_torque_unlocked();
  // Raising the limit after the linkage was forced off its set-point (a pair
  // stalled at the low limit) would let every drive close that error at once,
  // at the new limit and with no ramp. So first re-aim each drive where it is
  // (no jump), raise the limit, then go back to the last pose target with a
  // normal seat-level move.
  bool return_to_target = false;
  const bool raising = torque_permille > torque_limit_permille_.load();
  if (raising && !run_ && !measuring_ && !brakes_engaged_) {
    float deviation = 0.0f;
    bool read_ok = true;
    PoseCommand pose{};
    const bool have_target = target_valid_ && solve(target_x_, target_y_, pose);
    for (uint8_t i = 0; i < kPairCount && read_ok; ++i) {
      std::array<float, 2> position{};
      read_ok = actuator(static_cast<Pair>(i)).get_position(position, 100);
      if (read_ok && have_target) {
        deviation = std::max(deviation, std::fabs(position[0] - pose.pair_deg[i]));
      }
    }
    if (read_ok && (!have_target || deviation > 0.3f)) {
      for (uint8_t i = 0; i < kPairCount; ++i) {
        actuator(static_cast<Pair>(i)).hold_position();
      }
      return_to_target = have_target;
      if (have_target) {
        logger_.info("torque limit raised: drives re-aimed where they are ({:.2f} deg off the "
                     "target)",
                     deviation);
      } else {
        logger_.info("torque limit raised: drives re-aimed where they are (no pose target)");
      }
    }
  }
  bool ok = true;
  for (uint8_t i = 0; i < kPairCount; ++i) {
    if (!actuator(static_cast<Pair>(i)).set_max_torque(torque_permille)) {
      logger_.error("{} pair did not take the torque limit", stem::to_string(static_cast<Pair>(i)));
      ok = false;
    }
  }
  torque_limit_permille_.store(torque_permille);
  const uint32_t rated = rated_torque_mnm_.load();
  if (rated != 0) {
    logger_.info("torque limit {} permille of rated = {:.2f} Nm on every motor: {}",
                 torque_permille, permille_to_nm(torque_permille), ok ? "ok" : "FAILED on some");
  } else {
    logger_.info("torque limit {} permille of rated on every motor: {}", torque_permille,
                 ok ? "ok" : "FAILED on some");
  }
  if (ok && return_to_target && !any_zero_mismatch_unlocked()) {
    const float speed = std::min(config_.default_move_rpm * 6.0f, config_.max_move_speed_deg_s);
    if (start_move_unlocked(target_x_, target_y_, speed) == Result::Ok) {
      logger_.info("returning to the target x={:.1f} y={:.1f} with a level move", target_x_,
                   target_y_);
    }
  }
  return ok ? Result::Ok : Result::ActuatorFailed;
}

bool StemController::release_brakes() {
  if (measuring_) {
    const bool ok = set_measure_mode(false) == Result::Ok; // re-enables, in mode 1
    if (ok) {
      brakes_engaged_.store(false);
    }
    return ok;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  // EyouMotor has no separate brake relay: enabling the drives releases the
  // brakes, and the drives then HOLD the pose (target = where each one is).
  const bool m2_ok = config_.m2_seat.enable_drive();
  const bool m1_ok = config_.m1_seat.enable_drive();
  const bool m3_ok = config_.m3_seat.enable_drive();
  const bool ok = m1_ok && m2_ok && m3_ok;
  if (ok) {
    brakes_engaged_.store(false);
    recheck_mismatch_unlocked(); // a boot with the drives unpowered left a stale flag
  }
  logger_.info("brakes released (drives enabled, holding): M1_Seat={} M2_Seat={} M3_Seat={}",
               m1_ok, m2_ok, m3_ok);
  return ok;
}

bool StemController::engage_brakes() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!measuring_) {
    stop_unlocked(); // decelerate to rest first (any move), then switch off
  }
  if (measuring_) {
    for (auto &motor : config_.motors) {
      motor.set_operating_mode(1); // drives stay off; mode 1 for the next release
    }
    measuring_.store(false);
  }
  bool ok = true;
  for (uint8_t i = 0; i < kPairCount; ++i) {
    ok = actuator(static_cast<Pair>(i)).disable_drive() && ok;
  }
  brakes_engaged_.store(true);
  target_valid_ = false;
  logger_.warn("brakes ENGAGED: drives off{} - moves are refused until the brakes are released",
               ok ? "" : " (a drive did not confirm, check the motor table)");
  return ok;
}

StemController::Result StemController::set_measure_mode(bool on, int8_t drive_mode) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (on == measuring_.load()) {
    return Result::Ok;
  }
  auto read_pairs = [this](std::array<float, kPairCount> &out) {
    for (uint8_t i = 0; i < kPairCount; ++i) {
      std::array<float, 2> position{};
      if (!actuator(static_cast<Pair>(i)).get_position(position, 50)) {
        return false;
      }
      out[i] = position[0];
    }
    return true;
  };
  auto enable_all = [this]() {
    bool ok = true;
    for (auto &motor : config_.motors) {
      ok = motor.set_operating_mode(1) && ok; // enable_drive() verifies mode 1 too
    }
    for (uint8_t i = 0; i < kPairCount; ++i) {
      ok = actuator(static_cast<Pair>(i)).enable_drive() && ok; // holds where it is
    }
    return ok;
  };

  if (!on) {
    const bool ok = enable_all();
    measuring_.store(false);
    if (ok) {
      brakes_engaged_.store(false);
    }
    target_valid_ = false;
    logger_.info("measure mode OFF: drives in profile position, enabled: {}", ok ? "ok" : "FAILED");
    return ok ? Result::Ok : Result::ActuatorFailed;
  }

  if (drive_mode != 13 && drive_mode != 1) {
    return Result::InvalidArgument;
  }
  if (run_) {
    stop_unlocked();
  }
  std::array<float, kPairCount> before{};
  if (!read_pairs(before)) {
    logger_.error("measure mode refused: could not read the pair positions");
    return Result::ActuatorFailed;
  }
  // Drives off: the brakes engage and hold the pose.
  for (uint8_t i = 0; i < kPairCount; ++i) {
    actuator(static_cast<Pair>(i)).disable_drive();
  }
  // Watch for 0.5 s: a pair that moves is not held by its brake -> re-enable.
  for (int k = 0; k < 10; ++k) {
    vTaskDelay(pdMS_TO_TICKS(50));
    std::array<float, kPairCount> now{};
    if (!read_pairs(now)) {
      continue;
    }
    for (uint8_t i = 0; i < kPairCount; ++i) {
      if (std::fabs(now[i] - before[i]) > 0.5f) {
        enable_all();
        logger_.error("measure mode ABORTED: the {} pair moved {:.2f} deg with the drives off "
                      "(brake not holding?) - drives re-enabled",
                      stem::to_string(static_cast<Pair>(i)), now[i] - before[i]);
        return Result::ActuatorFailed;
      }
    }
  }
  // Mode while measuring (still disabled: nothing is commanded).
  bool ok = true;
  for (auto &motor : config_.motors) {
    int8_t mode = 0;
    const bool set_ok = motor.set_operating_mode(drive_mode);
    const bool read_ok = motor.get_operating_mode(mode);
    logger_.info("measure: motor {} mode {} -> reads back {}", motor.node_id(), drive_mode,
                 read_ok ? static_cast<int>(mode) : -1);
    ok = ok && set_ok;
  }
  {
    std::lock_guard<std::mutex> telemetry_lock(telemetry_mutex_);
    sensor_skip_.fill(0); // read every sensor from the first poll on
  }
  target_valid_ = false;
  measuring_.store(true);
  logger_.warn("measure mode ON: drives off, brakes holding, drive mode {}{}", drive_mode,
               ok ? "" : " (some mode writes failed)");
  return Result::Ok;
}

bool StemController::reset_fault() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (run_) {
    stop_unlocked();
  }
  bool ok = true;
  size_t faulted = 0;
  for (uint8_t i = 0; i < kPairCount; ++i) {
    auto &pair = actuator(static_cast<Pair>(i));
    if (!pair.is_faulted()) {
      continue;
    }
    ++faulted;
    // reset, then re-enable (a reset lands in Switch On Disabled, holding nothing)
    const bool reset_ok = pair.reset_fault();
    const bool enable_ok = reset_ok && pair.enable_drive();
    logger_.warn("{} pair fault reset: {}{}", stem::to_string(static_cast<Pair>(i)),
                 reset_ok ? "cleared" : "NOT cleared (its cause is still there?)",
                 reset_ok ? (enable_ok ? ", drive enabled" : ", drive NOT re-enabled") : "");
    ok = ok && reset_ok && enable_ok;
  }
  if (faulted == 0) {
    logger_.info("fault reset: no pair reports a fault");
  }
  // a fault interrupted whatever move was in progress
  target_valid_ = false;
  return ok;
}

bool StemController::zero(std::optional<Pair> pair, bool align) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (measuring_) {
    logger_.warn("zero refused: measure mode is on");
    return false;
  }
  if (run_) {
    stop_unlocked();
  }
  // EyouMotor::zero() always reads the absolute CANopen encoder, so there is no
  // separate alignment maneuver; `align` is kept only for CLI/protocol compatibility.
  (void)align;
  bool ok = true;
  if (pair.has_value()) {
    ok = actuator(*pair).zero();
    logger_.info("{} pair zeroed: {}", stem::to_string(*pair), ok ? "ok" : "FAILED");
    check_pair_unlocked(*pair);
  } else {
    for (uint8_t i = 0; i < kPairCount; ++i) {
      ok = actuator(static_cast<Pair>(i)).zero() && ok;
    }
    logger_.info("all pairs zeroed: {}", ok ? "ok" : "FAILED");
    for (uint8_t i = 0; i < kPairCount; ++i) {
      check_pair_unlocked(static_cast<Pair>(i));
    }
  }
  target_valid_ = false;
  return ok;
}

bool StemController::save_zero() {
  std::lock_guard<std::mutex> lock(mutex_);
  bool ok = true;
  for (uint8_t i = 0; i < kPairCount; ++i) {
    ok = actuator(static_cast<Pair>(i)).save_zero() && ok;
  }
  logger_.info("all pairs' zero saved to NVS: {}", ok ? "ok" : "FAILED");
  return ok;
}

bool StemController::read_zero(std::array<std::array<int32_t, 2>, kPairCount> &zero_offset_pulses) {
  std::lock_guard<std::mutex> lock(mutex_);
  bool ok = true;
  for (uint8_t i = 0; i < kPairCount; ++i) {
    ok = actuator(static_cast<Pair>(i)).read_zero(zero_offset_pulses[i]) && ok;
  }
  return ok;
}

bool StemController::load_zero() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (run_) {
    stop_unlocked();
  }
  bool ok = true;
  for (uint8_t i = 0; i < kPairCount; ++i) {
    ok = actuator(static_cast<Pair>(i)).load_zero() && ok;
  }
  logger_.info("all pairs' zero loaded from NVS: {}", ok ? "ok" : "FAILED");
  target_valid_ = false;
  for (uint8_t i = 0; i < kPairCount; ++i) {
    check_pair_unlocked(static_cast<Pair>(i));
  }
  return ok;
}

bool StemController::check_pairs() {
  std::lock_guard<std::mutex> lock(mutex_);
  bool ok = true;
  for (uint8_t i = 0; i < kPairCount; ++i) {
    ok = check_pair_unlocked(static_cast<Pair>(i)) && ok;
  }
  return ok;
}

bool StemController::has_zero_mismatch() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return any_zero_mismatch_unlocked();
}

bool StemController::check_pair_unlocked(Pair pair) {
  float mismatch_degrees = 0.0f;
  const bool ok = actuator(pair).check_mirrored(config_.mirror_tolerance_deg, mismatch_degrees);
  zero_mismatch_[static_cast<uint8_t>(pair)] = !ok;
  if (!ok) {
    logger_.error("{} pair zero mismatch: moves are blocked until the pair is re-zeroed",
                  stem::to_string(pair));
  }
  return ok;
}

bool StemController::recheck_mismatch_unlocked() {
  for (uint8_t i = 0; i < kPairCount; ++i) {
    if (zero_mismatch_[i] && check_pair_unlocked(static_cast<Pair>(i))) {
      logger_.info("{} pair zero mismatch cleared on re-check (drives answering now)",
                   stem::to_string(static_cast<Pair>(i)));
    }
  }
  return any_zero_mismatch_unlocked();
}

bool StemController::any_zero_mismatch_unlocked() const {
  return std::any_of(zero_mismatch_.begin(), zero_mismatch_.end(), [](bool m) { return m; });
}

State StemController::snapshot(bool read_motors) {
  // Only the controller's own fields are read under mutex_: the CAN reads below
  // take each motor's transaction lock alone, so a pose move's service() ticks
  // (velocity mode: a late speed update is a longer run at the last speed) are
  // never queued behind a whole telemetry poll.
  State state{};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state.can_ok = config_.can_ok;
    state.target_valid = target_valid_;
    state.target_x = target_x_;
    state.target_y = target_y_;
  }
  std::lock_guard<std::mutex> telemetry_lock(telemetry_mutex_); // sensor_skip_, one poll at a time
  bool all_pairs_read = true;
  for (uint8_t i = 0; i < kPairCount; ++i) {
    std::array<float, 2> position{};
    if (actuator(static_cast<Pair>(i)).get_position(position, 100)) {
      state.pair_deg[i] = position[0];
    } else {
      all_pairs_read = false; // the entry stays 0: no pose may be derived from it
    }
  }
  state.seat_tilt_deg = seat_tilt_deg_.load();
  // Measured pair positions -> crank angles (inverse of pose_to_pair_deg) -> pose;
  // only when every read succeeded (a zeroed entry would draw a false pose)
  IkSolution fk{};
  if (all_pairs_read && solve_fk_reference(-state.pair_deg[static_cast<uint8_t>(Pair::M1_Seat)],
                                           -state.pair_deg[static_cast<uint8_t>(Pair::M2_Seat)],
                                           config_.reference, fk, config_.geometry)) {
    state.pose_valid = true;
    state.pose_x = fk.m3[0] - config_.reference.x;
    state.pose_y = fk.m3[1] - config_.reference.y;
  }
  state.motors_ok = read_motors;
  const size_t count = std::min(config_.motors.size(), kMotorCount);
  for (size_t i = 0; i < count; ++i) {
    auto &motor = config_.motors[i];
    auto &telemetry = state.motors[i];
    telemetry.id = motor.node_id();
    if (read_motors) {
      float position_degrees = 0.0f;
      const bool position_ok = motor.get_position(position_degrees, 50);
      const bool statusword_ok = motor.get_statusword(telemetry.statusword, 50);
      const bool error_code_ok = motor.get_error_code(telemetry.error_code, 50);
      // torque and velocity are for the eye (how hard the linkage pushes, is it
      // moving): a failed read zeroes them without marking the motor bad
      if (!motor.get_torque(telemetry.torque_permille, 50)) {
        telemetry.torque_permille = 0;
      }
      if (!motor.get_velocity(telemetry.velocity_deg_s, 50)) {
        telemetry.velocity_deg_s = 0.0f;
      }
      // output torque sensor (-F joints): measured at the flange, so unlike
      // 0x6077 it does not include the gear friction
      telemetry.sensor_ok = false;
      if (sensor_skip_[i] > 0) {
        --sensor_skip_[i];
      } else if (motor.get_sensor_torque(telemetry.sensor_torque_nm, 50)) {
        telemetry.sensor_ok = true;
      } else {
        sensor_skip_[i] = kSensorRetryPolls;
      }
      if (!telemetry.sensor_ok) {
        telemetry.sensor_torque_nm = 0.0f;
      }
      telemetry.tracked_deg = position_degrees;
      telemetry.ok = position_ok && statusword_ok && error_code_ok;
      state.motors_ok = state.motors_ok && telemetry.ok;
    }
  }
  state.torque_limit_permille = torque_limit_permille_.load();
  if (read_motors && config_.can_ok) {
    ensure_rated_torque_unlocked(); // a motor that was off at boot may answer now
  }
  state.rated_torque_mnm = rated_torque_mnm_.load();
  state.measuring = measuring_.load();
  state.brakes_engaged = brakes_engaged_.load();
  if (config_.can_bus != nullptr && config_.can_ok) {
    const auto s = config_.can_bus->get_status();
    state.can.started = true;
    state.can.bus_off = s.bus_off;
    state.can.state = s.state;
    state.can.tx_error_count = s.tx_error_count;
    state.can.rx_error_count = s.rx_error_count;
    state.can.bus_error_count = s.bus_error_count;
    state.can.bus_off_count = s.bus_off_count;
    state.can.signal_errors = s.stuff_errors + s.form_errors + s.bit_errors;
    state.can.ack_errors = s.ack_errors;
  }
  return state;
}

// ======================= Pair balance (temporary bring-up tool) =======================

EyouMotor *StemController::motor_by_id(uint8_t id, size_t *index) {
  for (size_t i = 0; i < config_.motors.size() && i < kMotorCount; ++i) {
    if (config_.motors[i].node_id() == id) {
      if (index != nullptr) {
        *index = i;
      }
      return &config_.motors[i];
    }
  }
  return nullptr;
}

bool StemController::pair_of_motor(uint8_t id, Pair &pair, bool &is_secondary) const {
  for (uint8_t i = 0; i < kPairCount; ++i) {
    const auto &a = actuator(static_cast<Pair>(i));
    if (a.primary_id() == id || a.secondary_id() == id) {
      pair = static_cast<Pair>(i);
      is_secondary = a.secondary_id() == id;
      return true;
    }
  }
  return false;
}

// Average kSamples reads of every motor's output torque sensor and position: the
// sensors read ~0.1 Nm noise sample to sample, the steps change ~0.3-3 Nm.
bool StemController::read_balance(std::array<float, kMotorCount> &sensor,
                                  std::array<float, kMotorCount> &position) {
  constexpr int kSamples = 8;
  const size_t count = std::min(config_.motors.size(), kMotorCount);
  sensor.fill(0.0f);
  position.fill(0.0f);
  for (int k = 0; k < kSamples; ++k) {
    for (size_t i = 0; i < count; ++i) {
      float torque = 0.0f, degrees = 0.0f;
      if (!config_.motors[i].get_sensor_torque(torque, 50) ||
          !config_.motors[i].get_position(degrees, 50)) {
        logger_.error("balance: motor {} sensor / position read failed",
                      config_.motors[i].node_id());
        return false;
      }
      sensor[i] += torque / kSamples;
      position[i] += degrees / kSamples;
    }
    vTaskDelay(pdMS_TO_TICKS(15));
  }
  return true;
}

bool StemController::wait_target_reached(EyouMotor &motor, uint32_t timeout_ms) {
  for (uint32_t waited = 0; waited <= timeout_ms; waited += 20) {
    uint16_t statusword = 0;
    if (motor.get_statusword(statusword, 50) && (statusword & 0x0400) != 0) {
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  return false;
}

StemController::NudgeReport StemController::nudge(uint8_t motor_id, float step_deg,
                                                  uint8_t flags) {
  // Red-flag thresholds (Nm / deg). Sensor noise after averaging is ~0.05 Nm.
  constexpr float kFightRiseNm = 0.4f;   // the pair's fight grew: wrong direction / overshoot
  constexpr float kLoadShiftNm = 1.5f;   // the pair's load (sum) changed: something moved
  constexpr float kOtherMotorNm = 1.5f;  // a motor outside the pair changed torque
  constexpr float kOtherMoveDeg = 0.05f; // any motor other than the stepped one moved
  constexpr float kMaxSensorNm = 70.0f;  // 84 Nm rated: never let a step approach it
  constexpr uint32_t kSettleMs = 300;    // torque settle after the target is reached

  std::lock_guard<std::mutex> lock(mutex_);
  NudgeReport report{};
  report.motor_id = motor_id;
  report.step_deg = step_deg;
  const size_t count = std::min(config_.motors.size(), kMotorCount);
  for (size_t i = 0; i < count; ++i) {
    report.ids[i] = config_.motors[i].node_id();
  }
  auto refuse = [&](const std::string &why) {
    report.status = NudgeReport::Status::Refused;
    report.message = why;
    logger_.warn("nudge motor {} by {:.3f} deg refused: {}", motor_id, step_deg, why);
    return report;
  };

  size_t index = 0;
  EyouMotor *motor = motor_by_id(motor_id, &index);
  Pair pair{};
  bool is_secondary = false;
  if (motor == nullptr || !pair_of_motor(motor_id, pair, is_secondary)) {
    return refuse("unknown motor id");
  }
  if ((flags & kNudgeResetSession) != 0) {
    nudge_total_deg_.fill(0.0f);
  }
  if ((flags & kNudgeClearRedFlag) != 0 && nudge_red_flag_) {
    nudge_red_flag_ = false;
    logger_.warn("nudge red flag cleared by the user");
  }
  report.session_deg = nudge_total_deg_[index];
  if (nudge_red_flag_) {
    return refuse("a red flag is latched - review it, then Clear red flag");
  }
  if (!std::isfinite(step_deg) || std::fabs(step_deg) > kNudgeMaxStepDeg + 1e-6f) {
    return refuse("step must be within +/-" + std::to_string(kNudgeMaxStepDeg) + " deg");
  }
  if (std::fabs(nudge_total_deg_[index] + step_deg) > kNudgeMaxTotalDeg + 1e-6f) {
    return refuse("motor's total steps would exceed " + std::to_string(kNudgeMaxTotalDeg) +
                  " deg (zero it, or reset the session)");
  }
  if (run_) {
    return refuse("a move is running");
  }
  if (measuring_) {
    return refuse("measure mode is on");
  }
  if (brakes_engaged_) {
    return refuse("brakes engaged - release them first (the drives must hold)");
  }
  for (size_t i = 0; i < count; ++i) {
    uint16_t sw = 0;
    const uint8_t id = config_.motors[i].node_id();
    if (!config_.motors[i].get_statusword(sw, 50)) {
      return refuse("motor " + std::to_string(id) + " did not answer");
    }
    if ((sw & 0x0008) != 0) {
      return refuse("motor " + std::to_string(id) + " is faulted");
    }
    if ((sw & 0x006F) != 0x0027) {
      return refuse("motor " + std::to_string(id) + " is not enabled (holding)");
    }
    if ((sw & 0x0400) == 0) {
      return refuse("motor " + std::to_string(id) + " has not reached its target (still moving)");
    }
  }

  if (!read_balance(report.sensor_before, report.position_before)) {
    return refuse("a torque sensor / position read failed");
  }
  report.sensor_after = report.sensor_before;
  report.position_after = report.position_before;
  if (step_deg == 0.0f) {
    report.status = NudgeReport::Status::Ok;
    report.message = "measured";
    return report;
  }

  float target = 0.0f;
  if (!motor->get_target_position(target, 100)) {
    return refuse("could not read the motor's target");
  }
  // From here on a failure undoes the step and latches the red flag.
  auto red_flag = [&](const std::string &why) {
    const bool undone = motor->move_absolute(target) && wait_target_reached(*motor, 1500);
    vTaskDelay(pdMS_TO_TICKS(kSettleMs));
    nudge_red_flag_ = true;
    report.status = NudgeReport::Status::RedFlag;
    report.message = why + (undone ? " - step undone" : " - UNDO FAILED, check the drives");
    logger_.error("nudge RED FLAG, motor {} step {:+.3f} deg: {}", motor_id, step_deg,
                  report.message);
    for (size_t i = 0; i < count; ++i) {
      logger_.error("  motor {}: sensor {:+7.2f} -> {:+7.2f} Nm, position {:+9.4f} -> {:+9.4f} deg",
                    report.ids[i], report.sensor_before[i], report.sensor_after[i],
                    report.position_before[i], report.position_after[i]);
    }
    return report;
  };

  if (!motor->move_absolute(target + step_deg)) {
    return red_flag("the drive did not accept the step");
  }
  const bool reached = wait_target_reached(*motor, 1500);
  vTaskDelay(pdMS_TO_TICKS(kSettleMs));
  if (!read_balance(report.sensor_after, report.position_after)) {
    return red_flag("a torque sensor / position read failed after the step");
  }
  if (!reached) {
    return red_flag("the motor did not reach the stepped target (blocked?)");
  }

  // ---- red-flag checks ----
  char text[160];
  for (size_t i = 0; i < count; ++i) {
    if (std::fabs(report.sensor_after[i]) > kMaxSensorNm) {
      snprintf(text, sizeof(text), "motor %u output torque %.1f Nm is above %.0f Nm",
               report.ids[i], report.sensor_after[i], kMaxSensorNm);
      return red_flag(text);
    }
  }
  for (uint8_t p = 0; p < kPairCount; ++p) {
    const auto &a = actuator(static_cast<Pair>(p));
    size_t ip = 0, is = 0;
    if (motor_by_id(a.primary_id(), &ip) == nullptr || motor_by_id(a.secondary_id(), &is) == nullptr) {
      continue;
    }
    // mirrored drives: the pair's load is primary - secondary, the fight is their mean
    const float load_before = report.sensor_before[ip] - report.sensor_before[is];
    const float load_after = report.sensor_after[ip] - report.sensor_after[is];
    const float fight_before = 0.5f * (report.sensor_before[ip] + report.sensor_before[is]);
    const float fight_after = 0.5f * (report.sensor_after[ip] + report.sensor_after[is]);
    const char *name = stem::to_string(static_cast<Pair>(p));
    if (static_cast<Pair>(p) == pair) {
      if (std::fabs(fight_after) > std::fabs(fight_before) + kFightRiseNm) {
        snprintf(text, sizeof(text),
                 "%s fight grew %.2f -> %.2f Nm (wrong direction, or the step overshot zero)", name,
                 fight_before, fight_after);
        return red_flag(text);
      }
      if (std::fabs(load_after - load_before) > kLoadShiftNm) {
        snprintf(text, sizeof(text), "%s load changed %.2f -> %.2f Nm (the linkage moved?)", name,
                 load_before, load_after);
        return red_flag(text);
      }
    } else {
      for (const size_t i : {ip, is}) {
        if (std::fabs(report.sensor_after[i] - report.sensor_before[i]) > kOtherMotorNm) {
          snprintf(text, sizeof(text), "motor %u (%s, not stepped) torque changed %.2f -> %.2f Nm",
                   report.ids[i], name, report.sensor_before[i], report.sensor_after[i]);
          return red_flag(text);
        }
      }
    }
  }
  for (size_t i = 0; i < count; ++i) {
    if (i != index && std::fabs(report.position_after[i] - report.position_before[i]) > kOtherMoveDeg) {
      snprintf(text, sizeof(text), "motor %u (not stepped) moved %.3f deg", report.ids[i],
               report.position_after[i] - report.position_before[i]);
      return red_flag(text);
    }
  }

  nudge_total_deg_[index] += step_deg;
  report.session_deg = nudge_total_deg_[index];
  report.status = NudgeReport::Status::Ok;
  report.message = "ok";
  logger_.info("nudge motor {} {:+.3f} deg (total {:+.3f}): ok", motor_id, step_deg,
               report.session_deg);
  return report;
}

StemController::Result StemController::set_motor_zero(uint8_t motor_id, float &zero_change_deg,
                                                      std::string &message) {
  std::lock_guard<std::mutex> lock(mutex_);
  zero_change_deg = 0.0f;
  size_t index = 0;
  EyouMotor *motor = motor_by_id(motor_id, &index);
  Pair pair{};
  bool is_secondary = false;
  if (motor == nullptr || !pair_of_motor(motor_id, pair, is_secondary)) {
    message = "unknown motor id";
    return Result::InvalidArgument;
  }
  if (run_ || measuring_ || brakes_engaged_ || nudge_red_flag_) {
    message = run_ ? "a move is running" : measuring_ ? "measure mode is on"
              : brakes_engaged_ ? "brakes engaged" : "a nudge red flag is latched";
    return Result::InvalidArgument;
  }
  const auto &a = actuator(pair);
  EyouMotor *partner = motor_by_id(is_secondary ? a.primary_id() : a.secondary_id());
  float partner_deg = 0.0f, own_deg = 0.0f;
  if (partner == nullptr || !partner->get_position(partner_deg, 100) ||
      !motor->get_position(own_deg, 100)) {
    message = "position read failed";
    return Result::ActuatorFailed;
  }
  // mirrored pair: this motor should read the negative of its partner
  zero_change_deg = -partner_deg - own_deg;
  if (std::fabs(zero_change_deg) > kNudgeMaxTotalDeg + 0.1f) {
    message = "the new zero would move this motor's reading by " + std::to_string(zero_change_deg) +
              " deg - more than the balance tool's range, refused";
    return Result::OutOfLimits;
  }
  if (!motor->zero_at(-partner_deg) || !motor->save_zero() || !motor->hold_position()) {
    message = "zero / NVS save / hold failed (see the device log)";
    return Result::ActuatorFailed;
  }
  nudge_total_deg_[index] = 0.0f;
  check_pair_unlocked(pair);
  message = "ok";
  logger_.warn("motor {} re-zeroed to mirror motor {}: its reading moved {:+.4f} deg (saved to NVS)",
               motor_id, partner->node_id(), zero_change_deg);
  return Result::Ok;
}

} // namespace stem
