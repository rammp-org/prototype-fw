#include "rtps_link.hpp"

#include <algorithm>
#include <cmath>

#include "format.hpp"

#include "hw_config.hpp"

namespace {
const char *profile_name(MIB::DriveProfile p) {
  switch (p) {
  case MIB::DriveProfile::LOW:
    return "LOW";
  case MIB::DriveProfile::HIGH:
    return "HIGH";
  default:
    return "NORMAL";
  }
}

const char *state_name(MIB::MibSystemState s) {
  switch (s) {
  case MIB::MibSystemState::IDLE:
    return "IDLE";
  case MIB::MibSystemState::ENABLED:
    return "ENABLED";
  case MIB::MibSystemState::ERROR:
    return "ERROR";
  default:
    return "INITIALIZING";
  }
}

/// A float in whole units to the table's raw integer (10^decimals per unit).
int32_t to_raw(float value, uint8_t decimals) {
  float scale = 1.0f;
  for (uint8_t i = 0; i < decimals; ++i)
    scale *= 10.0f;
  return static_cast<int32_t>(std::lround(value * scale));
}
} // namespace

RtpsLink::RtpsLink(const Config &config)
    : espp::BaseComponent("RtpsLink", config.log_level)
    , config_(config)
    , controller_(config.controller) {}

RtpsLink::~RtpsLink() {
  // stop the timers before the endpoints they use go away; the endpoints
  // before the participant they are registered on
  watchdog_timer_.reset();
  publish_timer_.reset();
  xy_twist_sub_.reset();
  drive_sub_.reset();
  seat_sub_.reset();
  status_pub_.reset();
  diag_pub_.reset();
  participant_.reset();
}

bool RtpsLink::start() {
  participant_ = std::make_unique<espp::RtpsParticipant>(espp::RtpsParticipant::Config{
      .interface_address = config_.interface_address,
      .on_publisher_matched =
          [this]() {
            if (!peer_matched_.exchange(true))
              logger_.info("HMI matched (reader)");
          },
      .on_subscriber_matched =
          [this]() {
            if (!peer_matched_.exchange(true))
              logger_.info("HMI matched (writer)");
          },
      .log_level = espp::Logger::Verbosity::WARN,
  });
  if (!participant_->start()) {
    logger_.error("Failed to start the RTPS participant on {}", config_.interface_address);
    participant_.reset();
    return false;
  }

  // publishers: the chair's state, resent every publish period (best effort,
  // no durability, so a joystick that reboots catches up on the next tick)
  status_pub_ = std::make_unique<espp::Publisher<MIB::MibStatus>>(
      *participant_, espp::Publisher<MIB::MibStatus>::Config{.topic = MIB::kMibStatus.name,
                                                             .type_name = MIB::kMibStatus.type});
  diag_pub_ = std::make_unique<espp::Publisher<rammp::Diagnostics>>(
      *participant_,
      espp::Publisher<rammp::Diagnostics>::Config{.topic = rammp::kMcbDiagnostics.name,
                                                  .type_name = rammp::kMcbDiagnostics.type});
  // subscribers: the joystick's requests
  xy_twist_sub_ = std::make_unique<espp::Subscriber<rammp::XYTwist>>(
      *participant_, espp::Subscriber<rammp::XYTwist>::Config{
                         .topic = rammp::kJoystickXYTwist.name,
                         .type_name = rammp::kJoystickXYTwist.type,
                         .on_message = [this](const rammp::XYTwist &m) { on_xy_twist(m); }});
  drive_sub_ = std::make_unique<espp::Subscriber<rammp::DriveCommand>>(
      *participant_,
      espp::Subscriber<rammp::DriveCommand>::Config{
          .topic = rammp::kJoystickDriveCommand.name,
          .type_name = rammp::kJoystickDriveCommand.type,
          .on_message = [this](const rammp::DriveCommand &m) { on_drive_command(m); }});
  seat_sub_ = std::make_unique<espp::Subscriber<rammp::SeatCommand>>(
      *participant_,
      espp::Subscriber<rammp::SeatCommand>::Config{
          .topic = rammp::kJoystickSeatCommand.name,
          .type_name = rammp::kJoystickSeatCommand.type,
          .on_message = [this](const rammp::SeatCommand &m) { on_seat_command(m); }});
  if (!status_pub_->is_valid() || !diag_pub_->is_valid() || !xy_twist_sub_->is_valid() ||
      !drive_sub_->is_valid() || !seat_sub_->is_valid()) {
    logger_.error("Failed to create the RTPS endpoints (raise CONFIG_RTPS_LIMIT_NUM_* ?)");
    return false;
  }

  publish_timer_ = std::make_unique<espp::Timer>(espp::Timer::Config{
      .name = "mib_status_pub",
      .period = config_.publish_period,
      .callback = [this]() { return publish_step(); },
      .log_level = espp::Logger::Verbosity::WARN,
  });
  watchdog_timer_ = std::make_unique<espp::Timer>(espp::Timer::Config{
      .name = "joystick_wdt",
      .period = std::chrono::milliseconds(50),
      .callback = [this]() { return watchdog_step(); },
      .log_level = espp::Logger::Verbosity::WARN,
  });
  logger_.info("RTPS up on {}: sub {} / {} / {}, pub {} / {}", config_.interface_address,
               rammp::kJoystickXYTwist.name, rammp::kJoystickDriveCommand.name,
               rammp::kJoystickSeatCommand.name, MIB::kMibStatus.name, rammp::kMcbDiagnostics.name);
  return true;
}

void RtpsLink::set_ready(bool ready) { ready_.store(ready); }

void RtpsLink::set_error(const std::string &message, const std::string &footer) {
  std::lock_guard<std::mutex> lock(mutex_);
  error_message_ = message;
  error_footer_ = footer;
}

MIB::DriveProfile RtpsLink::active_profile() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return profile_;
}

bool RtpsLink::hmi_alive() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return have_message_ &&
         (std::chrono::steady_clock::now() - last_message_) <= config_.joystick_lost_timeout;
}

std::optional<std::chrono::milliseconds> RtpsLink::joystick_age() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!have_xy_twist_)
    return std::nullopt;
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                               last_xy_twist_);
}

void RtpsLink::apply_profile(MIB::DriveProfile profile) {
  const hw_config::ProfileLimits *limits = &hw_config::kProfileNormal;
  switch (profile) {
  case MIB::DriveProfile::LOW:
    limits = &hw_config::kProfileLow;
    break;
  case MIB::DriveProfile::HIGH:
    limits = &hw_config::kProfileHigh;
    break;
  default:
    break;
  }
  controller_.set_max_speed(limits->max_speed_mps);
  controller_.set_max_rotation(limits->max_rotation_rpm);
  std::lock_guard<std::mutex> lock(mutex_);
  profile_ = profile;
}

void RtpsLink::on_xy_twist(const rammp::XYTwist &msg) {
  // Network input: std::clamp passes NaN through, and a NaN would become an
  // active joystick command all the way to the motor frames. Drop such a
  // sample entirely, so it feeds neither the controller nor the watchdog (a
  // publisher that only sends garbage is then treated as a lost stream).
  if (!std::isfinite(msg.x) || !std::isfinite(msg.y) || !std::isfinite(msg.twist)) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (rejected_twist_++ == 0)
      logger_.error("XYTwist with non-finite values rejected (x={} y={} twist={})", msg.x, msg.y,
                    msg.twist);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_xy_twist_ = std::chrono::steady_clock::now();
    have_xy_twist_ = true;
    watchdog_fed_ = last_xy_twist_;
    if (!watch_joystick_) {
      watch_joystick_ = true;
      ++watch_generation_;
    }
    last_message_ = last_xy_twist_;
    have_message_ = true;
    joystick_held_ = false;
  }
  // HMI: x + = right, y + = forward, twist + = clockwise (all in [-1, 1],
  // deadzoned). Controller: forward, left, counter-clockwise.
  const float forward = std::clamp(msg.y, -1.0f, 1.0f);
  const float left = -std::clamp(msg.x, -1.0f, 1.0f);
  const float ccw = -std::clamp(msg.twist, -1.0f, 1.0f);
  controller_.set_joystick_input(forward, left, ccw);
}

void RtpsLink::on_drive_command(const rammp::DriveCommand &msg) {
  apply_profile(msg.profile);
  const bool enable = msg.request == rammp::DriveRequest::ENABLE;
  logger_.info("DriveCommand: {} (profile {})", enable ? "ENABLE" : "DISABLE",
               profile_name(msg.profile));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_message_ = std::chrono::steady_clock::now();
    have_message_ = true;
    joystick_lost_ = false;
    status_text_.clear();
  }
  if (!enable) {
    controller_.stop();
    return;
  }
  if (!ready_.load()) {
    logger_.warn("ENABLE refused: the platform is not ready");
    return;
  }
  {
    // Arm the watchdog from the ENABLE itself, so the hold / e-stop thresholds
    // apply even if the XYTwist stream never starts. Armed BEFORE the
    // controller is enabled (no stale timestamp can trip it), and with a new
    // generation, so a watchdog tick that saw the controller still disabled
    // cannot disarm this ENABLE afterwards.
    std::lock_guard<std::mutex> lock(mutex_);
    watchdog_fed_ = std::chrono::steady_clock::now();
    joystick_held_ = false;
    watch_joystick_ = true;
    ++watch_generation_;
  }
  controller_.enable(); // requires the joystick to re-center before it takes over
}

void RtpsLink::on_seat_command(const rammp::SeatCommand &msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  last_message_ = std::chrono::steady_clock::now();
  have_message_ = true;
  if (!seat_warned_) {
    seat_warned_ = true;
    logger_.warn("SeatCommand (axis {} -> {}) ignored: the holonomic platform has no seat",
                 static_cast<int>(msg.axis), msg.target);
  }
}

bool RtpsLink::watchdog_step() {
  uint32_t generation = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    generation = watch_generation_;
  }
  if (!controller_.is_enabled()) {
    std::lock_guard<std::mutex> lock(mutex_);
    // an ENABLE that armed the watchdog between the two checks above owns a
    // newer generation: leave it armed
    if (watch_generation_ == generation)
      watch_joystick_ = false; // re-armed by the next HMI ENABLE or XYTwist
    return false;
  }
  // timed from the last XYTwist or the HMI's ENABLE, whichever is later: a
  // stream that never starts after ENABLE is held and then e-stopped like a
  // stalled one
  std::chrono::milliseconds age{0};
  bool held = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!watch_joystick_)
      return false; // a bench `enable` from the CLI, no joystick expected
    held = joystick_held_;
    age = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                watchdog_fed_);
  }
  if (age > config_.joystick_lost_timeout) {
    logger_.error("Joystick stream lost ({} ms): e-stop", age.count());
    controller_.stop();
    std::lock_guard<std::mutex> lock(mutex_);
    joystick_lost_ = true;
    status_text_ = "JOYSTICK LOST";
  } else if (age > config_.joystick_hold_timeout && !held) {
    logger_.warn("Joystick stream paused ({} ms): holding zero", age.count());
    // not set_joystick_input(0, 0, 0): a synthetic zero must not count as the
    // centered sample the post-enable guard waits for
    controller_.hold_joystick();
    std::lock_guard<std::mutex> lock(mutex_);
    joystick_held_ = true;
  }
  return false; // keep the timer running
}

MIB::MibStatus RtpsLink::build_status() {
  const auto state = controller_.state();
  MIB::MibStatus status;
  std::lock_guard<std::mutex> lock(mutex_);
  // a configured error (CAN failed at boot) or a live motor-command failure
  // is reported before readiness, so a broken bus never hides as INITIALIZING
  std::string error = error_message_;
  std::string footer = error_footer_;
  if (error.empty() && state.send_failing) {
    error = "Motor command failed";
    footer = "A wheel motor is not acknowledging CAN commands";
  }
  if (!error.empty()) {
    status.systemState = MIB::MibSystemState::ERROR;
  } else if (!ready_.load()) {
    status.systemState = MIB::MibSystemState::INITIALIZING;
  } else if (state.enabled) {
    status.systemState = MIB::MibSystemState::ENABLED;
  } else {
    status.systemState = MIB::MibSystemState::IDLE;
  }
  status.activeProfile = profile_;
  status.currentSeatState = {}; // no seat on the holonomic platform
  status.error_message = error.empty() ? "No error" : error;
  status.error_footer = footer;
  status.epoch_s = 0; // no clock on this board
  status.speed = std::sqrt(state.vx_mps * state.vx_mps + state.vy_mps * state.vy_mps);
  status.utc_offset_min = 0;
  status.seq = status_seq_++;
  status.status_text = status_text_;
  return status;
}

rammp::Diagnostics RtpsLink::build_diagnostics() {
  const auto state = controller_.state();
  rammp::Diagnostics diag;
  std::lock_guard<std::mutex> lock(mutex_);
  diag.seq = diag_seq_++;
  // one item per RAMMP_DIAG_TABLE row, from the motors in id order; readings are
  // raw integers in 10^-decimals of the row's unit
  for (size_t row = 0; row < rammp::kDiagCount; ++row) {
    rammp::DiagItem item;
    const auto &spec = rammp::kDiagItems[row];
    if (row < state.motors.size() && state.motors[row].valid) {
      const auto &m = state.motors[row];
      item.values = {
          to_raw(static_cast<float>(m.temperature_c), spec.decimals[0]), // Temp [C]
          to_raw(static_cast<float>(m.torque_raw) * hw_config::kTorqueRawToAmps,
                 spec.decimals[1]),                  // Current [A]
          to_raw(m.angle_degrees, spec.decimals[2]), // Pos [deg]
      };
    } else {
      item.values = {0, 0, 0};
    }
    diag.items.push_back(std::move(item));
  }
  return diag;
}

bool RtpsLink::publish_step() {
  const auto status = build_status();
  if (!status_pub_->publish(status))
    logger_.debug("MibStatus publish dropped");
  if (!diag_pub_->publish(build_diagnostics()))
    logger_.debug("Diagnostics publish dropped");
  logger_.debug("MibStatus: {} profile {} speed {:.2f} m/s seq {}", state_name(status.systemState),
                profile_name(status.activeProfile), status.speed, status.seq);
  return false; // keep the timer running
}
