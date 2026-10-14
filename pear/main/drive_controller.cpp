#include "drive_controller.hpp"

#include <string>

#include <algorithm>
#include <cmath>
#include <numeric>

DriveController::DriveController(const Config &config)
    : espp::BaseComponent("DriveController", config.log_level)
    , platform_(config.platform)
    , motors_(config.motors)
    , max_wheel_rpm_(clamp_positive(config.max_wheel_rpm))
    , joystick_release_timeout_(config.joystick_release_timeout)
    , status_stale_timeout_(config.status_stale_timeout)
    , rearm_retry_period_(config.rearm_retry_period)
    , status_read_timeout_ms_(config.status_read_timeout_ms)
    , flush_pending_commands_(config.flush_pending_commands)
    , armed_(config.motors_armed)
    , max_speed_mps_(clamp_positive(config.max_speed_mps))
    , max_rotation_rpm_(clamp_positive(config.max_rotation_rpm))
    , twist_rotation_scale_(clamp_positive(config.twist_rotation_scale))
    , control_timer_({.name = "drive_ctrl",
                      .period = config.control_period,
                      .callback = [this]() { return control_step(); },
                      .auto_start = true,
                      .stack_size_bytes = 6 * 1024,
                      .priority = 10})
    , status_timer_({.name = "drive_status",
                     .period = config.status_poll_period,
                     .callback = [this]() { return status_step(); },
                     .auto_start = true,
                     .stack_size_bytes = 6 * 1024,
                     .priority = 5}) {
  for (size_t i = 0; i < motor_status_.size(); ++i) {
    motor_status_[i].id = motors_[i].node_id();
    // a drive that never answers becomes stale one timeout after boot
    motor_status_time_[i] = std::chrono::steady_clock::now();
  }
  // resolve the platform wheel -> node id table to motors_ indices once
  for (size_t wheel = 0; wheel < motor_index_by_wheel_.size(); ++wheel) {
    const size_t node_id = config.motor_ids_by_wheel[wheel];
    const auto found = std::find_if(motor_status_.begin(), motor_status_.end(),
                                    [node_id](const MotorStatus &m) { return m.id == node_id; });
    if (found == motor_status_.end()) {
      logger_.error("motor_ids_by_wheel[{}] = {} is not one of the wheel drives; using wheel {}",
                    wheel, node_id, wheel);
      motor_index_by_wheel_[wheel] = wheel;
    } else {
      motor_index_by_wheel_[wheel] = static_cast<size_t>(found - motor_status_.begin());
    }
  }
}

DriveController::~DriveController() {
  // Stop the loop first so nothing else commands the motors, then zero them.
  control_timer_.cancel();
  status_timer_.cancel();
  send_zeros();
}

float DriveController::clamp_positive(float value) { return std::max(value, 0.0f); }

void DriveController::set_gui_velocity(float vx_mps, float vy_mps, float w_rpm) {
  if (!std::isfinite(vx_mps) || !std::isfinite(vy_mps) || !std::isfinite(w_rpm)) {
    logger_.error("Rejecting a non-finite velocity setpoint");
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  // clamp the translation vector magnitude (preserving direction) and the
  // rotation rate to the current limits
  const float magnitude = std::sqrt(vx_mps * vx_mps + vy_mps * vy_mps);
  if (magnitude > max_speed_mps_ && magnitude > 0.0f) {
    const float scale = max_speed_mps_ / magnitude;
    vx_mps *= scale;
    vy_mps *= scale;
  }
  gui_vx_mps_ = vx_mps;
  gui_vy_mps_ = vy_mps;
  gui_w_rpm_ = std::clamp(w_rpm, -max_rotation_rpm_, max_rotation_rpm_);
}

void DriveController::set_joystick_input(float forward, float left, float ccw) {
  std::lock_guard<std::mutex> lock(mutex_);
  joystick_forward_ = std::clamp(forward, -1.0f, 1.0f);
  joystick_left_ = std::clamp(left, -1.0f, 1.0f);
  joystick_ccw_ = std::clamp(ccw, -1.0f, 1.0f);
  ++joystick_sample_seq_;
  if (joystick_forward_ != 0.0f || joystick_left_ != 0.0f || joystick_ccw_ != 0.0f) {
    last_joystick_activity_ = std::chrono::steady_clock::now();
  }
}

void DriveController::hold_joystick() {
  std::lock_guard<std::mutex> lock(mutex_);
  // not a sample: joystick_sample_seq_ is untouched so the re-center guard
  // (recenter_after_seq_) still waits for a real centered reading
  joystick_forward_ = 0.0f;
  joystick_left_ = 0.0f;
  joystick_ccw_ = 0.0f;
}

void DriveController::set_linkage_moving(std::function<bool()> fn) {
  std::lock_guard<std::mutex> lock(mutex_);
  linkage_moving_ = std::move(fn);
}

bool DriveController::enable() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode_ == Mode::DRIVE) {
    return true;
  }
  const Mode previous = mode_;
  // Commit the intent FIRST, then look at the other side: the linkage batch
  // does the mirror image (marks itself moving, then checks us), so whichever
  // interleaving happens at least one side yields and motion never overlaps.
  mode_ = Mode::DRIVE;
  if (linkage_moving_ && linkage_moving_()) {
    mode_ = previous;
    logger_.warn("DRIVE enable refused: a linkage move is executing");
    return false;
  }
  // Zero the setpoint and require the joystick to re-center before it can take
  // over, so nothing jumps or lurches on (re-)enable.
  gui_vx_mps_ = 0.0f;
  gui_vy_mps_ = 0.0f;
  gui_w_rpm_ = 0.0f;
  // The guard clears only on a sample that arrives AFTER this enable: the
  // cached (possibly stale-zero) reading must not clear it.
  require_joystick_recenter_ = true;
  recenter_after_seq_ = joystick_sample_seq_;
  source_ = Source::GUI;
  next_arm_attempt_ = std::chrono::steady_clock::now(); // re-arm at once if needed
  // The control loop resumes commanding velocities from here (no motor sends
  // are done from this setter - see the class docs).
  logger_.info("DRIVE enabled (joystick must re-center before it takes over)");
  return true;
}

void DriveController::stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  // E-stop always wins: the control loop enacts zero velocity on its next tick
  // (and every tick while STOPPED), so an in-flight drive command cannot leave
  // the motors driving. No motor send is done from this setter.
  mode_ = Mode::STOPPED;
  source_ = Source::STOPPED;
  vx_mps_ = 0.0f;
  vy_mps_ = 0.0f;
  w_rpm_ = 0.0f;
  flush_pending_ = true; // queued drive frames must not run after this stop
  logger_.info("E-STOP: commanding zero velocity");
}

void DriveController::disable_motors() {
  std::lock_guard<std::mutex> lock(mutex_);
  mode_ = Mode::DISABLED;
  source_ = Source::STOPPED;
  vx_mps_ = 0.0f;
  vy_mps_ = 0.0f;
  w_rpm_ = 0.0f;
  disable_sent_ = false; // the control loop issues the one-shot motor stop
  flush_pending_ = true; // queued drive frames must not run after the stop
  logger_.info("Wheel drives DISABLED (power stage off after the stop)");
}

void DriveController::flush_transport_if_requested() {
  bool flush = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    flush = flush_pending_;
    flush_pending_ = false;
  }
  if (flush && flush_pending_commands_) {
    flush_pending_commands_();
  }
}

void DriveController::note_command_sent() {
  std::lock_guard<std::mutex> lock(mutex_);
  last_command_ = std::chrono::steady_clock::now();
  have_command_ = true;
}

bool DriveController::is_enabled() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_ == Mode::DRIVE;
}

void DriveController::set_max_speed(float max_speed_mps) {
  std::lock_guard<std::mutex> lock(mutex_);
  max_speed_mps_ = clamp_positive(max_speed_mps);
}

void DriveController::set_max_rotation(float max_rotation_rpm) {
  std::lock_guard<std::mutex> lock(mutex_);
  max_rotation_rpm_ = clamp_positive(max_rotation_rpm);
}

DriveController::State DriveController::state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  State state;
  state.mode = mode_;
  state.enabled = mode_ == Mode::DRIVE;
  state.armed = armed_;
  state.source = source_;
  state.vx_mps = vx_mps_;
  state.vy_mps = vy_mps_;
  state.w_rpm = w_rpm_;
  state.gui_vx_mps = gui_vx_mps_;
  state.gui_vy_mps = gui_vy_mps_;
  state.gui_w_rpm = gui_w_rpm_;
  state.max_speed_mps = max_speed_mps_;
  state.max_rotation_rpm = max_rotation_rpm_;
  state.twist_rotation_scale = twist_rotation_scale_;
  state.max_wheel_rpm = max_wheel_rpm_;
  state.send_failing = send_failing_;
  state.lost_wheel_id = lost_wheel_id_;
  state.motors = motor_status_;
  if (have_command_) {
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_command_);
    state.command_age_ms = static_cast<uint16_t>(std::min<int64_t>(age.count(), 65535));
  }
  return state;
}

bool DriveController::send_zeros() {
  bool ok = true;
  for (auto &motor : motors_) {
    ok = motor.stop() && ok;
  }
  note_command_sent();
  return ok;
}

bool DriveController::send_disable() {
  // Acknowledged per drive (the SDO confirmation), not just "the frame left":
  // DISABLED is a one-shot that stops retrying once this returns true, so it
  // must mean every drive really took the zero and powered its stage off.
  bool ok = true;
  for (auto &motor : motors_) {
    ok = motor.stop_acknowledged(kStopAckTimeoutMs) && ok;
  }
  for (auto &motor : motors_) {
    ok = motor.disable_drive() && ok;
  }
  note_command_sent();
  std::lock_guard<std::mutex> lock(mutex_);
  // whatever subset of drives took the disable, the next DRIVE must re-enable
  // them all together (arm_motors()) before any set-point goes out
  armed_ = false;
  return ok;
}

bool DriveController::arm_motors() {
  bool ok = true;
  for (auto &motor : motors_) {
    if (!motor.enable_drive()) {
      logger_.error("Wheel drive {} failed to enable", motor.node_id());
      ok = false;
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  armed_ = ok;
  next_arm_attempt_ = std::chrono::steady_clock::now() + rearm_retry_period_;
  return ok;
}

bool DriveController::control_step() {
  float vx = 0.0f;
  float vy = 0.0f;
  float w = 0.0f;
  Mode current_mode;
  bool disable_now = false;
  bool arm_now = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_mode = mode_;

    // STOPPED / DISABLED are enacted from this loop (never from the setters),
    // so a mode change can never be overtaken by an in-flight drive command.
    // The actual CAN sends happen below, outside the lock.
    if (current_mode == Mode::STOPPED) {
      // fall through to the outside-the-lock send
    } else if (current_mode == Mode::DISABLED) {
      // retried every tick until every motor has acknowledged its stop
      disable_now = !disable_sent_;
    } else {
      // DRIVE: source arbitration. Any joystick deflection takes over - UNLESS
      // we are still waiting for the joystick to re-center after enable (so a
      // stick that was held at enable time cannot immediately drive). Once the
      // joystick reads centered, it is cleared to take over again.
      const auto now = std::chrono::steady_clock::now();
      arm_now = !armed_ && now >= next_arm_attempt_;
      const bool joystick_deflected =
          joystick_forward_ != 0.0f || joystick_left_ != 0.0f || joystick_ccw_ != 0.0f;
      if (require_joystick_recenter_) {
        if (!joystick_deflected && joystick_sample_seq_ != recenter_after_seq_) {
          require_joystick_recenter_ = false; // a post-enable centered sample: cleared
        }
      } else if (joystick_deflected) {
        source_ = Source::JOYSTICK;
      } else if (source_ == Source::JOYSTICK &&
                 (now - last_joystick_activity_) > joystick_release_timeout_) {
        // handover back to the GUI: zero its setpoint so nothing jumps
        gui_vx_mps_ = 0.0f;
        gui_vy_mps_ = 0.0f;
        gui_w_rpm_ = 0.0f;
        source_ = Source::GUI;
        logger_.info("Joystick released; GUI setpoint zeroed and active");
      }

      if (source_ == Source::JOYSTICK && !require_joystick_recenter_) {
        vx = joystick_forward_ * max_speed_mps_;
        // the joystick input is left-positive; the controller (and the platform)
        // frame is right-positive, so convert here
        vy = -joystick_left_ * max_speed_mps_;
        // The twist scale deliberately lets the commanded rate exceed the
        // nominal rotation limit (the per-wheel max_wheel_rpm scaling below
        // still bounds the motors): the platform's rotation authority is much
        // weaker than translation, so the twist axis needs its own gain.
        w = joystick_ccw_ * max_rotation_rpm_ * twist_rotation_scale_;
      } else {
        vx = gui_vx_mps_;
        vy = gui_vy_mps_;
        w = gui_w_rpm_;
      }
      vx_mps_ = vx;
      vy_mps_ = vy;
      w_rpm_ = w;
    } // end DRIVE
  }

  // Enact STOPPED / DISABLED with the CAN sends outside the lock. If a STOP
  // arrived during a DRIVE tick's compute below, the next tick lands here and
  // stops the motors - they can be driving for at most one control period,
  // never indefinitely.
  if (current_mode == Mode::STOPPED) {
    flush_transport_if_requested(); // first tick after the stop: drop queued set-points
    // The zero set-point is the stop, streamed EVERY tick: the L2DB drives let
    // the shaft go after their communication-loss timeout if the stream
    // stops, so a held e-stop keeps feeding zeros. A drive that does not take
    // it is a fault the operator must see (and a recovered one clears a
    // latched DRIVE fault).
    const bool all_zeroed = send_zeros();
    std::lock_guard<std::mutex> lock(mutex_);
    if (!all_zeroed && !send_failing_)
      logger_.error("Failed to send zero velocity to one or more drives; retrying");
    send_failing_ = !all_zeroed;
    return false; // keep the timer running
  }
  if (current_mode == Mode::DISABLED) {
    if (disable_now) {
      flush_transport_if_requested(); // first tick after the disable: drop queued set-points
      const bool all_stopped = send_disable();
      std::lock_guard<std::mutex> lock(mutex_);
      if (mode_ == Mode::DISABLED) {
        disable_sent_ = all_stopped; // a failed stop is retried next tick
      }
      if (!all_stopped && !send_failing_)
        logger_.error("Failed to stop / disable one or more drives; retrying");
      send_failing_ = !all_stopped;
    }
    return false; // keep the timer running
  }

  // DRIVE: the drives must be enabled (power stage on) before a set-point can
  // move them; after a DISABLED (or a failed boot enable) that happens here,
  // retried with a back-off so a missing drive does not hog the loop.
  if (arm_now) {
    const bool armed = arm_motors();
    std::lock_guard<std::mutex> lock(mutex_);
    if (!armed && !send_failing_)
      logger_.error("Wheel drives not enabled; retrying every {} ms", rearm_retry_period_.count());
    send_failing_ = !armed;
    if (!armed)
      return false;
    // arming took several acknowledged SDOs outside the lock: a STOP / DISABLE
    // that landed meanwhile wins, the next tick enacts it (no set-point now)
    if (mode_ != Mode::DRIVE)
      return false;
  } else {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!armed_)
      return false; // waiting for the next re-arm attempt
  }

  // A wheel that stopped reporting is a drive fault: its set-points are
  // unconfirmed (any node ACKs the frame), so it may already have released
  // after its communication-loss timeout while the other three keep driving.
  // STOP the platform (zeros from now on), drop the armed state and latch the
  // node id for MibStatus / MotorState until it reports again.
  bool wheel_fault = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const size_t stale = stale_wheel_locked(std::chrono::steady_clock::now());
    // a drive reporting an alarm (status_step() records it) has stopped
    // contributing just like a silent one: the other three must not keep driving
    const auto faulted = std::find_if(motor_status_.begin(), motor_status_.end(),
                                      [](const MotorStatus &m) { return m.valid && m.faulted; });
    if (stale != std::string::npos) {
      lost_wheel_id_ = motor_status_[stale].id;
      logger_.error("Wheel drive {} stopped reporting; platform STOPPED", lost_wheel_id_);
    } else if (faulted != motor_status_.end()) {
      logger_.error("Wheel drive {} reports an alarm (error 0x{:04X}); platform STOPPED "
                    "(`reset_fault`, then enable again)",
                    faulted->id, faulted->error_code);
    }
    if (stale != std::string::npos || faulted != motor_status_.end()) {
      mode_ = Mode::STOPPED;
      armed_ = false;
      flush_pending_ = true;
      send_failing_ = true;
      wheel_fault = true;
    }
    wheel_fault =
        wheel_fault || lost_wheel_id_ != 0; // read under the lock: status_step() clears it
  }
  if (wheel_fault) {
    flush_transport_if_requested();
    send_zeros();
    return false; // STOPPED from the next tick on
  }

  // DRIVE: compute + send outside the lock; the drives have their own locking
  const auto wheel_speeds = platform_.calculate_wheel_speeds(vx, vy, w);
  const std::array<float, 4> wheel_rpms = {
      wheel_speeds.top_left_rpm,
      wheel_speeds.top_right_rpm,
      wheel_speeds.bottom_left_rpm,
      wheel_speeds.bottom_right_rpm,
  };
  const float maximum_rpm = std::transform_reduce(
      wheel_rpms.begin(), wheel_rpms.end(), 0.0f, [](float a, float b) { return std::max(a, b); },
      [](float rpm) { return std::abs(rpm); });
  const float scale = maximum_rpm > max_wheel_rpm_ ? max_wheel_rpm_ / maximum_rpm : 1.0f;

  std::array<float, pear::kCornerCount> commanded_rpms{};
  bool success = true;
  for (size_t wheel_index = 0; wheel_index < wheel_rpms.size(); ++wheel_index) {
    const size_t motor_index = motor_index_by_wheel_[wheel_index];
    const float rpm = wheel_rpms[wheel_index] * scale;
    commanded_rpms[motor_index] = rpm;
    success = motors_[motor_index].send_velocity_rpm(rpm) && success;
  }
  note_command_sent();

  Mode mode_after = Mode::DRIVE;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    mode_after = mode_;
    for (size_t i = 0; i < motor_status_.size(); ++i) {
      motor_status_[i].commanded_rpm = commanded_rpms[i];
    }
    // log send failures only on the transition, not at 50 Hz
    if (!success && !send_failing_) {
      logger_.error("Failed to send velocity command(s) to one or more drives");
    }
    send_failing_ = !success;
  }
  // A stop() / disable_motors() that landed while the frames above were on the
  // wire is enacted NOW rather than on the next tick: the drive frames just
  // queued are drained from the transport and overridden within the same
  // control period.
  if (mode_after == Mode::STOPPED) {
    flush_transport_if_requested();
    send_zeros();
  } else if (mode_after == Mode::DISABLED) {
    flush_transport_if_requested();
    send_disable();
  }
  return false; // keep the timer running
}

bool DriveController::status_step() {
  const size_t index = status_poll_index_;
  status_poll_index_ = (status_poll_index_ + 1) % motors_.size();

  Ds402VelocityMotor::Status status{};
  const bool ok = motors_[index].read_status(status, status_read_timeout_ms_);
  const auto now = std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> lock(mutex_);
  auto &motor_status = motor_status_[index];
  if (ok) {
    if (status.faulted && !motor_status.faulted)
      logger_.error("Wheel drive {} reports a fault (statusword 0x{:04X}, error 0x{:04X})",
                    motor_status.id, status.statusword, status.error_code);
    motor_status.valid = true;
    motor_status.velocity_rpm = status.velocity_rpm;
    motor_status.temperature_c = status.temperature_c;
    motor_status.current_a = status.current_a;
    motor_status.angle_degrees = status.position_degrees;
    motor_status.statusword = status.statusword;
    motor_status.error_code = status.error_code;
    motor_status.faulted = status.faulted;
    motor_status_time_[index] = now;
    if (lost_wheel_id_ == motor_status.id) {
      logger_.info("Wheel drive {} reports again; enable driving to resume", motor_status.id);
      lost_wheel_id_ = 0;
    }
  }
  // a motor is stale until its first successful read, and again whenever the
  // last successful read is too old
  motor_status.stale =
      !motor_status.valid || (now - motor_status_time_[index]) > status_stale_timeout_;
  return false; // keep the timer running
}

size_t DriveController::stale_wheel_locked(std::chrono::steady_clock::time_point now) const {
  for (size_t i = 0; i < motor_status_time_.size(); ++i) {
    if (now - motor_status_time_[i] > status_stale_timeout_)
      return i;
  }
  return std::string::npos;
}
