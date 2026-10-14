#include "rtps_link.hpp"

#include <algorithm>
#include <cmath>

#include "esp_timer.h"
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

constexpr float kTwoPi = 2.0f * pear::geometry::kPi;
float rpm_to_rad_s(float rpm) { return rpm * kTwoPi / 60.0f; }

uint32_t uptime_ms() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }

/// The seat-axis row of a PEAR_LINK_* axis -> its corner, or nullopt.
std::optional<pear::Corner> link_axis_corner(rammp::SeatAxis axis) {
  switch (axis) {
  case rammp::SeatAxis::PEAR_LINK_FR:
    return pear::Corner::FR;
  case rammp::SeatAxis::PEAR_LINK_FL:
    return pear::Corner::FL;
  case rammp::SeatAxis::PEAR_LINK_BR:
    return pear::Corner::BR;
  case rammp::SeatAxis::PEAR_LINK_BL:
    return pear::Corner::BL;
  default:
    return std::nullopt;
  }
}

/// The diagnostics row of a wheel / linkage corner.
constexpr size_t wheel_diag_row(size_t corner) {
  return static_cast<size_t>(rammp::DiagId::PEAR_WHEEL_FR) + corner;
}
constexpr size_t link_diag_row(size_t corner) {
  return static_cast<size_t>(rammp::DiagId::PEAR_LINK_FR) + corner;
}
} // namespace

RtpsLink::RtpsLink(const Config &config)
    : espp::BaseComponent("RtpsLink", config.log_level)
    , config_(config)
    , drive_(config.drive)
    , linkage_(config.linkage) {}

RtpsLink::~RtpsLink() {
  // stop the timers before the endpoints they use go away; the endpoints
  // before the participant they are registered on
  watchdog_timer_.reset();
  motor_state_timer_.reset();
  publish_timer_.reset();
  xy_twist_sub_.reset();
  drive_sub_.reset();
  seat_sub_.reset();
  status_pub_.reset();
  diag_pub_.reset();
  for (auto &pub : motor_pubs_)
    pub.reset();
  participant_.reset();
}

bool RtpsLink::start(const std::string &interface_address) {
  interface_address_ = interface_address;
  participant_ = std::make_unique<espp::RtpsParticipant>(espp::RtpsParticipant::Config{
      .interface_address = interface_address_,
      .on_publisher_matched =
          [this]() {
            if (!peer_matched_.exchange(true))
              logger_.info("Peer matched (reader)");
          },
      .on_subscriber_matched =
          [this]() {
            if (!peer_matched_.exchange(true))
              logger_.info("Peer matched (writer)");
          },
      .log_level = espp::Logger::Verbosity::WARN,
  });
  if (!participant_->start()) {
    logger_.error("Failed to start the RTPS participant on {}", interface_address_);
    participant_.reset();
    return false;
  }

  // publishers: the base's state, resent every period (best effort, no
  // durability, so a joystick that reboots catches up on the next tick)
  status_pub_ = std::make_unique<espp::Publisher<MIB::MibStatus>>(
      *participant_, espp::Publisher<MIB::MibStatus>::Config{.topic = MIB::kMibStatus.name,
                                                             .type_name = MIB::kMibStatus.type});
  diag_pub_ = std::make_unique<espp::Publisher<rammp::Diagnostics>>(
      *participant_,
      espp::Publisher<rammp::Diagnostics>::Config{.topic = rammp::kMcbDiagnostics.name,
                                                  .type_name = rammp::kMcbDiagnostics.type});
  bool motor_pubs_ok = true;
  for (size_t i = 0; i < kMotorAxisCount; ++i) {
    const auto &axis = rammp::axis(kMotorAxes[i]);
    motor_pubs_[i] = std::make_unique<espp::Publisher<rammp::MotorState>>(
        *participant_, espp::Publisher<rammp::MotorState>::Config{.topic = axis.state.name,
                                                                  .type_name = axis.state.type});
    motor_pubs_ok = motor_pubs_[i]->is_valid() && motor_pubs_ok;
  }
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
  if (!status_pub_->is_valid() || !diag_pub_->is_valid() || !motor_pubs_ok ||
      !xy_twist_sub_->is_valid() || !drive_sub_->is_valid() || !seat_sub_->is_valid()) {
    logger_.error("Failed to create the RTPS endpoints (raise CONFIG_RTPS_LIMIT_NUM_* ?)");
    return false;
  }

  publish_timer_ = std::make_unique<espp::Timer>(espp::Timer::Config{
      .name = "mib_status_pub",
      .period = config_.publish_period,
      .callback = [this]() { return publish_step(); },
      .log_level = espp::Logger::Verbosity::WARN,
  });
  motor_state_timer_ = std::make_unique<espp::Timer>(espp::Timer::Config{
      .name = "motor_state_pub",
      .period = config_.motor_state_period,
      .callback = [this]() { return motor_state_step(); },
      .log_level = espp::Logger::Verbosity::WARN,
  });
  watchdog_timer_ = std::make_unique<espp::Timer>(espp::Timer::Config{
      .name = "joystick_wdt",
      .period = std::chrono::milliseconds(50),
      .callback = [this]() { return watchdog_step(); },
      .log_level = espp::Logger::Verbosity::WARN,
  });
  logger_.info("RTPS up on {}: sub {} / {} / {}, pub {} / {} + {} motor_state topics",
               interface_address_, rammp::kJoystickXYTwist.name, rammp::kJoystickDriveCommand.name,
               rammp::kJoystickSeatCommand.name, MIB::kMibStatus.name, rammp::kMcbDiagnostics.name,
               kMotorAxisCount);
  return true;
}

void RtpsLink::set_ready(bool ready) { ready_.store(ready); }

void RtpsLink::set_error(const std::string &message, const std::string &footer) {
  std::lock_guard<std::mutex> lock(mutex_);
  error_message_ = message;
  error_footer_ = footer;
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
  drive_.set_max_speed(limits->max_speed_mps);
  drive_.set_max_rotation(limits->max_rotation_rpm);
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
  drive_.set_joystick_input(forward, left, ccw);
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
    // DriveRequest::DISABLE means "stop driving on the stick" (the spec's
    // words), not "power the drives off": like holo_deck_mib, it e-stops into
    // STOPPED, where the drives hold at zero in closed loop. Cutting the power
    // stages (DS402 disable, the CLI's `disable`) would let the shafts go and
    // a base on a slope roll; that remains a deliberate console action.
    drive_.stop();
    return;
  }
  if (!ready_.load()) {
    logger_.warn("ENABLE refused: the base is not ready");
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
  if (!drive_.enable()) // requires the joystick to re-center before it takes over
    logger_.warn("ENABLE refused: a linkage move is executing (the HMI sees no ENABLED)");
}

void RtpsLink::on_seat_command(const rammp::SeatCommand &msg) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_message_ = std::chrono::steady_clock::now();
    have_message_ = true;
  }
  if (!std::isfinite(msg.target)) {
    logger_.error_rate_limited("SeatCommand axis {} with a non-finite target rejected",
                               static_cast<int>(msg.axis));
    return;
  }
  // a single corner row
  if (const auto corner = link_axis_corner(msg.axis)) {
    linkage_.request_corner_angle(*corner, msg.target);
    return;
  }
  // a pose row: replace that component of the controller's DESIRED pose
  // (table units: deg, mm) so successive single-axis requests accumulate
  using PoseComponent = LinkageController::PoseComponent;
  switch (msg.axis) {
  case rammp::SeatAxis::FRONT_BACK_TILT:
    linkage_.request_pose_component(PoseComponent::FB_TILT, msg.target);
    return;
  case rammp::SeatAxis::LATERAL_TILT:
    linkage_.request_pose_component(PoseComponent::LATERAL_TILT, msg.target);
    return;
  case rammp::SeatAxis::ELEVATION:
    linkage_.request_pose_component(PoseComponent::ELEVATION, msg.target * 0.001f);
    return;
  case rammp::SeatAxis::TRANSLATION: {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!translation_warned_) {
      translation_warned_ = true;
      logger_.warn("SeatCommand TRANSLATION ({}) ignored: the base has no translation axis",
                   msg.target);
    }
    return;
  }
  default:
    logger_.warn_rate_limited("SeatCommand for unknown axis {} ignored",
                              static_cast<int>(msg.axis));
    return;
  }
}

bool RtpsLink::watchdog_step() {
  uint32_t generation = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    generation = watch_generation_;
  }
  if (!drive_.is_enabled()) {
    std::lock_guard<std::mutex> lock(mutex_);
    // an ENABLE that armed the watchdog between the two checks above owns a
    // newer generation: leave it armed (cppcheck cannot see the lock gap)
    // cppcheck-suppress knownConditionTrueFalse
    if (watch_generation_ == generation)
      watch_joystick_ = false; // re-armed by the next HMI ENABLE or XYTwist
    return false;
  }
  // timed from the last XYTwist or the HMI's ENABLE, whichever is later: a
  // stream that never starts after ENABLE is held and then e-stopped like a
  // stalled one
  std::chrono::milliseconds age{0};
  bool held = false;
  std::chrono::steady_clock::time_point fed{};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!watch_joystick_)
      return false; // a bench `enable` from the CLI, no joystick expected
    held = joystick_held_;
    fed = watchdog_fed_;
    age = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                fed);
  }
  // The transition is taken under the link mutex and only if nothing fed the
  // watchdog since the measurement: a sample or ENABLE that landed meanwhile
  // owns a newer feed / generation and wins, and one that lands after this
  // block finds the drive already stopped (it must ENABLE again), so a live
  // stream is never overtaken by a stale "lost" verdict.
  if (age > config_.joystick_lost_timeout) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (watch_generation_ != generation || watchdog_fed_ != fed)
      return false;
    logger_.error("Joystick stream lost ({} ms): e-stop", age.count());
    drive_.stop();
    joystick_lost_ = true;
    status_text_ = "JOYSTICK LOST";
  } else if (age > config_.joystick_hold_timeout && !held) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (watch_generation_ != generation || watchdog_fed_ != fed)
      return false;
    logger_.warn("Joystick stream paused ({} ms): holding zero", age.count());
    // not set_joystick_input(0, 0, 0): a synthetic zero must not count as the
    // centered sample the post-enable guard waits for
    drive_.hold_joystick();
    joystick_held_ = true;
  }
  return false; // keep the timer running
}

MIB::MibStatus RtpsLink::build_status() {
  const auto drive = drive_.state();
  const auto linkage = linkage_.state();
  MIB::MibStatus status;
  std::lock_guard<std::mutex> lock(mutex_);
  // a configured error (CAN failed at boot), a live motor-command failure or a
  // faulted motor is reported before readiness, so a broken bus never hides
  // as INITIALIZING
  std::string error = error_message_;
  std::string footer = error_footer_;
  if (error.empty() && drive.lost_wheel_id != 0) {
    error = fmt::format("Wheel drive {} lost", drive.lost_wheel_id);
    footer = "It stopped reporting on CAN; check it, then enable driving again";
  }
  if (error.empty() && drive.send_failing) {
    error = "Wheel command failed";
    footer = "A wheel drive is not taking CAN commands";
  }
  if (error.empty()) {
    for (const pear::Corner corner : pear::kCorners) {
      const auto &wheel = drive.motors[pear::index(corner)];
      if (wheel.valid && wheel.faulted) {
        error = fmt::format("Wheel {} fault", pear::corner_name(corner));
        footer = fmt::format("Drive {} error 0x{:04X}; `reset_fault` on the console", wheel.id,
                             wheel.error_code);
        break;
      }
      const auto &link = linkage.motors[pear::index(corner)];
      if (link.valid && link.faulted) {
        error = fmt::format("Linkage {} fault", pear::corner_name(corner));
        footer = fmt::format("Motor {}: {}", link.id, EyouMotor::ds402_state_name(link.statusword));
        break;
      }
      if (link.valid && link.stale) { // reported once, then went silent
        error = fmt::format("Linkage {} lost", pear::corner_name(corner));
        footer = fmt::format("Motor {} stopped reporting on CAN; check it", link.id);
        break;
      }
    }
  }
  if (!error.empty()) {
    status.systemState = MIB::MibSystemState::ERROR;
  } else if (!ready_.load()) {
    status.systemState = MIB::MibSystemState::INITIALIZING;
  } else if (drive.enabled) {
    status.systemState = MIB::MibSystemState::ENABLED;
  } else {
    status.systemState = MIB::MibSystemState::IDLE;
  }
  status.activeProfile = profile_;
  // the measured pose in the table's whole units (deg / mm) + the corner angles
  status.currentSeatState = {
      .front_back_tilt = linkage.pose.fb_tilt_deg,
      .lateral_tilt = linkage.pose.lateral_tilt_deg,
      .elevation = linkage.pose.elevation_m * 1000.0f,
      .translation = 0.0f,
      .pear_link_fr = linkage.angles_deg[pear::Corner::FR],
      .pear_link_fl = linkage.angles_deg[pear::Corner::FL],
      .pear_link_br = linkage.angles_deg[pear::Corner::BR],
      .pear_link_bl = linkage.angles_deg[pear::Corner::BL],
  };
  status.error_message = error.empty() ? "No error" : error;
  status.error_footer = footer;
  status.epoch_s = 0; // no clock on this board
  status.speed = std::sqrt(drive.vx_mps * drive.vx_mps + drive.vy_mps * drive.vy_mps);
  status.utc_offset_min = 0;
  status.seq = status_seq_++;
  status.status_text = status_text_;
  return status;
}

rammp::Diagnostics RtpsLink::build_diagnostics() {
  const auto drive = drive_.state();
  const auto linkage = linkage_.state();
  rammp::Diagnostics diag;
  std::lock_guard<std::mutex> lock(mutex_);
  diag.seq = diag_seq_++;
  // one item per RAMMP_DIAG_TABLE row; readings are raw integers in
  // 10^-decimals of the row's unit. TEST rows (and a motor never read) are zeros.
  for (size_t row = 0; row < rammp::kDiagCount; ++row) {
    rammp::DiagItem item;
    const auto &spec = rammp::kDiagItems[row];
    item.values = {0, 0, 0};
    for (size_t corner = 0; corner < pear::kCornerCount; ++corner) {
      if (row == wheel_diag_row(corner) && drive.motors[corner].valid) {
        const auto &m = drive.motors[corner];
        item.values = {
            to_raw(m.temperature_c, spec.decimals[0]), // Temp [C]
            to_raw(m.current_a, spec.decimals[1]),     // Current [A]
            to_raw(m.velocity_rpm, spec.decimals[2]),  // Vel [rpm]
        };
      } else if (row == link_diag_row(corner) && linkage.motors[corner].valid) {
        const auto &m = linkage.motors[corner];
        item.values = {
            0,                                            // Temp [C]: not exposed by the driver
            0,                                            // Current [A]: not exposed by the driver
            to_raw(m.position_degrees, spec.decimals[2]), // Pos [deg]
        };
      }
    }
    diag.items.push_back(std::move(item));
  }
  return diag;
}

rammp::MotorState RtpsLink::build_wheel_state(size_t corner, const DriveController::State &drive) {
  const auto &m = drive.motors[corner];
  rammp::MotorState state{};
  state.api_version = RAMMP_MOTOR_API_VERSION;
  state.axis_id = kMotorAxes[corner];
  state.seq = motor_seq_[corner]++;
  if (m.stale) {
    state.state = rammp::BoardState::FAULT; // no status within the stale timeout
    state.fault_code = rammp::FaultCode::WATCHDOG;
  } else if (m.valid && m.faulted) {
    state.state = rammp::BoardState::FAULT;
    state.fault_code = rammp::FaultCode::DRV_FAULT;
    state.drv_status = m.error_code;
  } else if (drive.armed) {
    state.state = rammp::BoardState::ARMED;
  } else {
    state.state = rammp::BoardState::DISARMED;
  }
  state.mode = state.state == rammp::BoardState::ARMED ? rammp::ControlMode::VELOCITY
                                                       : rammp::ControlMode::COAST;
  state.cmd_age_ms = drive.command_age_ms;
  state.uptime_ms = uptime_ms();
  state.position = pear::geometry::deg_to_rad(m.angle_degrees);
  state.velocity = rpm_to_rad_s(m.velocity_rpm);
  state.iq = m.current_a;
  state.vel_limit_eff = rpm_to_rad_s(drive.max_wheel_rpm);
  state.temps[0] = m.temperature_c;
  return state;
}

rammp::MotorState RtpsLink::build_linkage_state(size_t corner,
                                                const LinkageController::State &linkage) {
  const auto &m = linkage.motors[corner];
  rammp::MotorState state{};
  state.api_version = RAMMP_MOTOR_API_VERSION;
  state.axis_id = kMotorAxes[pear::kCornerCount + corner];
  state.seq = motor_seq_[pear::kCornerCount + corner]++;
  if (m.stale) {
    // no status within the stale timeout: the controller refuses to move it,
    // and the cached DS402 state is no evidence of anything
    state.state = rammp::BoardState::FAULT;
    state.fault_code = rammp::FaultCode::WATCHDOG;
  } else if (m.valid && m.faulted) {
    state.state = rammp::BoardState::FAULT;
    state.fault_code = rammp::FaultCode::DRV_FAULT;
    state.drv_status = m.statusword;
  } else if (m.valid && m.operation_enabled) {
    state.state = rammp::BoardState::ARMED;
  } else {
    state.state = rammp::BoardState::DISARMED;
  }
  state.mode = state.state == rammp::BoardState::ARMED ? rammp::ControlMode::POSITION
                                                       : rammp::ControlMode::COAST;
  state.cmd_age_ms = linkage.command_age_ms;
  state.uptime_ms = uptime_ms();
  state.position = pear::geometry::deg_to_rad(m.position_degrees);
  state.velocity = 0.0f; // not measured (the profile moves are slow; the poll is 1 Hz)
  return state;
}

bool RtpsLink::motor_state_step() {
  const auto drive = drive_.state();
  const auto linkage = linkage_.state();
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t corner = 0; corner < pear::kCornerCount; ++corner) {
    if (!motor_pubs_[corner]->publish(build_wheel_state(corner, drive)))
      logger_.debug("MotorState publish dropped (wheel {})", corner);
    if (!motor_pubs_[pear::kCornerCount + corner]->publish(build_linkage_state(corner, linkage)))
      logger_.debug("MotorState publish dropped (linkage {})", corner);
  }
  return false; // keep the timer running
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
