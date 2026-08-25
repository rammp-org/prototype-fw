#include "holo_deck_controller.hpp"

#include <algorithm>
#include <cmath>

HoloDeckController::HoloDeckController(const Config &config)
    : espp::BaseComponent("HoloDeckController", config.log_level)
    , platform_(config.platform)
    , motors_(config.motors)
    , motor_ids_by_wheel_(config.motor_ids_by_wheel)
    , max_wheel_rpm_(clamp_positive(config.max_wheel_rpm))
    , joystick_release_timeout_(config.joystick_release_timeout)
    , status_stale_timeout_(config.status_stale_timeout)
    , max_speed_mps_(clamp_positive(config.max_speed_mps))
    , max_rotation_rpm_(clamp_positive(config.max_rotation_rpm))
    , control_timer_({.name = "holo_ctrl",
                      .period = config.control_period,
                      .callback = [this]() { return control_step(); },
                      .auto_start = true,
                      .stack_size_bytes = 6 * 1024,
                      .priority = 10})
    , status_timer_({.name = "holo_status",
                     .period = config.status_poll_period,
                     .callback = [this]() { return status_step(); },
                     .auto_start = true,
                     .stack_size_bytes = 6 * 1024,
                     .priority = 5}) {
  for (size_t i = 0; i < motor_status_.size(); ++i) {
    motor_status_[i].id = motors_[i].get_motor_id();
  }
}

HoloDeckController::~HoloDeckController() {
  control_timer_.cancel();
  status_timer_.cancel();
  std::lock_guard<std::mutex> lock(mutex_);
  if (!stop_sent_) {
    send_zeros_unlocked();
    stop_sent_ = true;
  }
}

float HoloDeckController::clamp_positive(float value) { return std::max(value, 0.0f); }

void HoloDeckController::set_gui_velocity(float vx_mps, float vy_mps, float w_rpm) {
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

void HoloDeckController::set_gui_translation_normalized(float forward, float left) {
  std::lock_guard<std::mutex> lock(mutex_);
  forward = std::clamp(forward, -1.0f, 1.0f);
  left = std::clamp(left, -1.0f, 1.0f);
  float vx = forward * max_speed_mps_;
  float vy = left * max_speed_mps_;
  const float magnitude = std::sqrt(vx * vx + vy * vy);
  if (magnitude > max_speed_mps_ && magnitude > 0.0f) {
    const float scale = max_speed_mps_ / magnitude;
    vx *= scale;
    vy *= scale;
  }
  gui_vx_mps_ = vx;
  gui_vy_mps_ = vy;
}

void HoloDeckController::set_gui_rotation_normalized(float ccw) {
  std::lock_guard<std::mutex> lock(mutex_);
  ccw = std::clamp(ccw, -1.0f, 1.0f);
  gui_w_rpm_ = ccw * max_rotation_rpm_;
}

void HoloDeckController::set_joystick_input(float forward, float left, float ccw) {
  std::lock_guard<std::mutex> lock(mutex_);
  joystick_forward_ = std::clamp(forward, -1.0f, 1.0f);
  joystick_left_ = std::clamp(left, -1.0f, 1.0f);
  joystick_ccw_ = std::clamp(ccw, -1.0f, 1.0f);
  if (joystick_forward_ != 0.0f || joystick_left_ != 0.0f || joystick_ccw_ != 0.0f) {
    last_joystick_activity_ = std::chrono::steady_clock::now();
  }
}

void HoloDeckController::set_enabled(bool enabled) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (enabled == enabled_) {
    return;
  }
  enabled_ = enabled;
  if (enabled_) {
    // zero the setpoint so the platform never jumps on re-enable
    gui_vx_mps_ = 0.0f;
    gui_vy_mps_ = 0.0f;
    gui_w_rpm_ = 0.0f;
    source_ = Source::GUI;
    logger_.info("Enabled");
  } else {
    // e-stop: command zeros immediately (once); the control loop will not
    // command the motors again until re-enabled
    source_ = Source::STOPPED;
    vx_mps_ = 0.0f;
    vy_mps_ = 0.0f;
    w_rpm_ = 0.0f;
    send_zeros_unlocked();
    stop_sent_ = true;
    logger_.info("E-STOP: motors zeroed and disabled");
  }
}

bool HoloDeckController::is_enabled() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return enabled_;
}

void HoloDeckController::set_max_speed(float max_speed_mps) {
  std::lock_guard<std::mutex> lock(mutex_);
  max_speed_mps_ = clamp_positive(max_speed_mps);
}

void HoloDeckController::set_max_rotation(float max_rotation_rpm) {
  std::lock_guard<std::mutex> lock(mutex_);
  max_rotation_rpm_ = clamp_positive(max_rotation_rpm);
}

HoloDeckController::State HoloDeckController::state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  State state;
  state.enabled = enabled_;
  state.source = source_;
  state.vx_mps = vx_mps_;
  state.vy_mps = vy_mps_;
  state.w_rpm = w_rpm_;
  state.gui_vx_mps = gui_vx_mps_;
  state.gui_vy_mps = gui_vy_mps_;
  state.gui_w_rpm = gui_w_rpm_;
  state.max_speed_mps = max_speed_mps_;
  state.max_rotation_rpm = max_rotation_rpm_;
  state.max_wheel_rpm = max_wheel_rpm_;
  state.motors = motor_status_;
  return state;
}

void HoloDeckController::send_zeros_unlocked() {
  for (auto &motor : motors_) {
    if (!motor.send_velocity(0.0f)) {
      logger_.error("Failed to send zero velocity to motor {}", motor.get_motor_id());
    }
  }
}

bool HoloDeckController::control_step() {
  float vx = 0.0f;
  float vy = 0.0f;
  float w = 0.0f;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_) {
      if (!stop_sent_) {
        send_zeros_unlocked();
        stop_sent_ = true;
      }
      return false; // keep the timer running
    }
    stop_sent_ = false;

    // source arbitration: any joystick deflection takes over; once the
    // joystick has been centered for longer than the release timeout, fall
    // back to the (zeroed) GUI setpoint
    const auto now = std::chrono::steady_clock::now();
    const bool joystick_deflected =
        joystick_forward_ != 0.0f || joystick_left_ != 0.0f || joystick_ccw_ != 0.0f;
    if (joystick_deflected) {
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

    if (source_ == Source::JOYSTICK) {
      vx = joystick_forward_ * max_speed_mps_;
      vy = joystick_left_ * max_speed_mps_;
      w = joystick_ccw_ * max_rotation_rpm_;
    } else {
      vx = gui_vx_mps_;
      vy = gui_vy_mps_;
      w = gui_w_rpm_;
    }
    vx_mps_ = vx;
    vy_mps_ = vy;
    w_rpm_ = w;
  }

  // compute + send outside the lock; the CAN bus has its own locking
  const auto wheel_speeds = platform_.calculate_wheel_speeds(vx, vy, w);
  const std::array<float, 4> wheel_rpms = {
      wheel_speeds.top_left_rpm,
      wheel_speeds.top_right_rpm,
      wheel_speeds.bottom_left_rpm,
      wheel_speeds.bottom_right_rpm,
  };
  float maximum_rpm = 0.0f;
  for (float rpm : wheel_rpms) {
    maximum_rpm = std::max(maximum_rpm, std::abs(rpm));
  }
  const float scale = maximum_rpm > max_wheel_rpm_ ? max_wheel_rpm_ / maximum_rpm : 1.0f;

  std::array<float, 4> commanded_rpms{};
  bool success = true;
  for (size_t wheel_index = 0; wheel_index < wheel_rpms.size(); ++wheel_index) {
    const size_t motor_index = motor_ids_by_wheel_[wheel_index] - 1;
    const float rpm = wheel_rpms[wheel_index] * scale;
    commanded_rpms[motor_index] = rpm;
    success = motors_[motor_index].send_velocity(rpm) && success;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < motor_status_.size(); ++i) {
      motor_status_[i].commanded_rpm = commanded_rpms[i];
    }
    // log send failures only on the transition, not at 50 Hz
    if (!success && !send_failing_) {
      logger_.error("Failed to send velocity command(s) to one or more motors");
    }
    send_failing_ = !success;
  }
  return false; // keep the timer running
}

bool HoloDeckController::status_step() {
  const size_t index = status_poll_index_;
  status_poll_index_ = (status_poll_index_ + 1) % motors_.size();

  MotorActuator::Status status{};
  const bool ok = motors_[index].read_status(status, 50);
  const auto now = std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> lock(mutex_);
  auto &motor_status = motor_status_[index];
  if (ok) {
    motor_status.valid = true;
    motor_status.velocity_rpm = status.velocity_rpm;
    motor_status.temperature_c = status.temperature_c;
    motor_status_time_[index] = now;
  }
  // a motor is stale until its first successful read, and again whenever the
  // last successful read is too old
  motor_status.stale =
      !motor_status.valid || (now - motor_status_time_[index]) > status_stale_timeout_;
  return false; // keep the timer running
}
