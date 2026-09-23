#include "paired_eyou_actuator.hpp"

PairedEyouActuator::PairedEyouActuator(EyouMotor &primary, EyouMotor &secondary)
    : espp::BaseComponent("PairedEyouActuator", espp::Logger::Verbosity::INFO), primary_(primary),
      secondary_(secondary) {
  set_log_tag("PairedEyouActuator-" + std::to_string(primary_.node_id()) + "-" +
              std::to_string(secondary_.node_id()));
}

bool PairedEyouActuator::configure_profile_position(const EyouMotor::ProfilePositionConfig &config) {
  const bool primary_ok = primary_.configure_profile_position(config);
  const bool secondary_ok = secondary_.configure_profile_position(config);
  return primary_ok && secondary_ok;
}

bool PairedEyouActuator::enable_drive(uint32_t timeout_ms) {
  const bool primary_ok = primary_.enable_drive(timeout_ms);
  const bool secondary_ok = secondary_.enable_drive(timeout_ms);
  return primary_ok && secondary_ok;
}

bool PairedEyouActuator::disable_drive(uint32_t timeout_ms) {
  const bool primary_ok = primary_.disable_drive(timeout_ms);
  const bool secondary_ok = secondary_.disable_drive(timeout_ms);
  return primary_ok && secondary_ok;
}

bool PairedEyouActuator::reset_fault(uint32_t timeout_ms) {
  const bool primary_ok = primary_.reset_fault(timeout_ms);
  const bool secondary_ok = secondary_.reset_fault(timeout_ms);
  return primary_ok && secondary_ok;
}

bool PairedEyouActuator::is_faulted(uint32_t timeout_ms) {
  // Evaluate both (no short-circuit) so a fault on either motor is always detected.
  const bool primary_faulted = primary_.is_faulted(timeout_ms);
  const bool secondary_faulted = secondary_.is_faulted(timeout_ms);
  return primary_faulted || secondary_faulted;
}

bool PairedEyouActuator::zero(uint32_t timeout_ms) {
  const bool primary_ok = primary_.zero(timeout_ms);
  const bool secondary_ok = secondary_.zero(timeout_ms);
  return primary_ok && secondary_ok;
}

bool PairedEyouActuator::save_zero() {
  const bool primary_ok = primary_.save_zero();
  const bool secondary_ok = secondary_.save_zero();
  return primary_ok && secondary_ok;
}

bool PairedEyouActuator::load_zero() {
  const bool primary_ok = primary_.load_zero();
  const bool secondary_ok = secondary_.load_zero();
  return primary_ok && secondary_ok;
}

bool PairedEyouActuator::get_position(std::array<float, 2> &position_degrees, uint32_t timeout_ms) {
  const bool primary_ok = primary_.get_position(position_degrees[0], timeout_ms);
  const bool secondary_ok = secondary_.get_position(position_degrees[1], timeout_ms);
  return primary_ok && secondary_ok;
}

bool PairedEyouActuator::get_status(Status &status, uint32_t timeout_ms) {
  const bool primary_ok = primary_.get_statusword(status.statusword[0], timeout_ms) &&
                          primary_.get_error_code(status.error_code[0], timeout_ms);
  const bool secondary_ok = secondary_.get_statusword(status.statusword[1], timeout_ms) &&
                            secondary_.get_error_code(status.error_code[1], timeout_ms);
  return primary_ok && secondary_ok;
}

bool PairedEyouActuator::set_position_limits(float minimum_degrees, float maximum_degrees) {
  if (minimum_degrees > maximum_degrees) {
    logger_.error("Position minimum {} exceeds maximum {}", minimum_degrees, maximum_degrees);
    return false;
  }
  const bool primary_ok = primary_.set_position_limits(minimum_degrees, maximum_degrees);
  const bool secondary_ok = secondary_.set_position_limits(-maximum_degrees, -minimum_degrees);
  return primary_ok && secondary_ok;
}

bool PairedEyouActuator::set_position(float position_degrees, bool immediate) {
  const bool primary_ok = primary_.move_absolute(position_degrees, immediate);
  const bool secondary_ok = secondary_.move_absolute(-position_degrees, immediate);
  if (!primary_ok || !secondary_ok) {
    logger_.error("set_position failed (primary={}, secondary={}); disabling both drives",
                  primary_ok, secondary_ok);
    if (!disable_drive()) {
      logger_.error("Failed to disable both drives after a move failure");
    }
    return false;
  }
  return true;
}

bool PairedEyouActuator::move_incremental(float increment_degrees, bool immediate) {
  const bool primary_ok = primary_.move_incremental(increment_degrees, immediate);
  const bool secondary_ok = secondary_.move_incremental(-increment_degrees, immediate);
  if (!primary_ok || !secondary_ok) {
    logger_.error("move_incremental failed (primary={}, secondary={}); disabling both drives",
                  primary_ok, secondary_ok);
    if (!disable_drive()) {
      logger_.error("Failed to disable both drives after a move failure");
    }
    return false;
  }
  return true;
}
