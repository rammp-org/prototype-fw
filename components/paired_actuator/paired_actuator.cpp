#include "paired_actuator.hpp"

PairedActuator::PairedActuator(MotorActuator &primary, MotorActuator &secondary)
    : espp::BaseComponent("PairedActuator", espp::Logger::Verbosity::INFO), primary_(primary),
      secondary_(secondary) {
  set_log_tag("PairedActuator-" + std::to_string(primary_.get_motor_id()) + "-" +
              std::to_string(secondary_.get_motor_id()));
  set_position_limits();
}

bool PairedActuator::read_status(Status &status, uint32_t timeout_ms) {
  const bool primary_ok = primary_.read_status(status.motors[0], timeout_ms);
  const bool secondary_ok = secondary_.read_status(status.motors[1], timeout_ms);
  return primary_ok && secondary_ok;
}

void PairedActuator::zero_position(bool need_align) {
  primary_.zero_position(need_align);
  secondary_.zero_position(need_align);
}

std::array<float, 2> PairedActuator::get_position() const {
  return {primary_.get_position(), secondary_.get_position()};
}

bool PairedActuator::set_position_limits(float minimum_degrees, float maximum_degrees) {
  if (minimum_degrees > maximum_degrees) {
    logger_.error("Position minimum {} exceeds maximum {}", minimum_degrees, maximum_degrees);
    return false;
  }
  const bool primary_ok = primary_.set_position_limits(minimum_degrees, maximum_degrees);
  const bool secondary_ok = secondary_.set_position_limits(-maximum_degrees, -minimum_degrees);
  return primary_ok && secondary_ok;
}

bool PairedActuator::set_position(float position_degrees, float max_speed_rpm) {
  const bool primary_ok = primary_.set_position(position_degrees, max_speed_rpm);
  const bool secondary_ok = secondary_.set_position(-position_degrees, max_speed_rpm);
  return primary_ok && secondary_ok;
}

bool PairedActuator::stop() {
  const bool primary_ok = primary_.stop();
  const bool secondary_ok = secondary_.stop();
  return primary_ok && secondary_ok;
}

bool PairedActuator::release_brake() {
  const bool primary_ok = primary_.release_brake();
  const bool secondary_ok = secondary_.release_brake();
  return primary_ok && secondary_ok;
}

bool PairedActuator::lock_brake() {
  const bool primary_ok = primary_.lock_brake();
  const bool secondary_ok = secondary_.lock_brake();
  return primary_ok && secondary_ok;
}
