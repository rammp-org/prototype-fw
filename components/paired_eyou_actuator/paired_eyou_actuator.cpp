#include "paired_eyou_actuator.hpp"

#include <cmath>

PairedEyouActuator::PairedEyouActuator(EyouMotor &primary, EyouMotor &secondary)
    : espp::BaseComponent("PairedEyouActuator", espp::Logger::Verbosity::INFO)
    , primary_(primary)
    , secondary_(secondary) {
  set_log_tag("PairedEyouActuator-" + std::to_string(primary_.node_id()) + "-" +
              std::to_string(secondary_.node_id()));
}

bool PairedEyouActuator::configure_profile_position(
    const EyouMotor::ProfilePositionConfig &config) {
  const bool primary_ok = primary_.configure_profile_position(config);
  const bool secondary_ok = secondary_.configure_profile_position(config);
  return primary_ok && secondary_ok;
}

bool PairedEyouActuator::enable_drive(uint32_t timeout_ms) {
  const bool primary_ok = primary_.enable_drive(timeout_ms);
  const bool secondary_ok = secondary_.enable_drive(timeout_ms);
  if (primary_ok && secondary_ok) {
    return true;
  }
  // a mechanically coupled pair is never left half energized: BOTH sides are
  // disabled again (a side whose enable "failed" may only have lost the
  // acknowledgement and be energized all the same), so false means both off
  logger_.error("Enable failed ({}); disabling both motors of the pair",
                !primary_ok && !secondary_ok ? "both" : (primary_ok ? "secondary" : "primary"));
  const bool primary_off = primary_.disable_drive(timeout_ms);
  const bool secondary_off = secondary_.disable_drive(timeout_ms);
  if (!primary_off || !secondary_off) {
    logger_.error("Disable after the failed enable not confirmed ({}); check the pair",
                  !primary_off && !secondary_off ? "both"
                                                 : (primary_off ? "secondary" : "primary"));
  }
  return false;
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

bool PairedEyouActuator::read_zero(std::array<int32_t, 2> &zero_offset_pulses) {
  const bool primary_ok = primary_.read_saved_zero(zero_offset_pulses[0]);
  const bool secondary_ok = secondary_.read_saved_zero(zero_offset_pulses[1]);
  return primary_ok && secondary_ok;
}

bool PairedEyouActuator::load_zero() {
  const bool primary_ok = primary_.load_zero();
  const bool secondary_ok = secondary_.load_zero();
  return primary_ok && secondary_ok;
}

bool PairedEyouActuator::check_mirrored(float tolerance_degrees, float &mismatch_degrees,
                                        uint32_t timeout_ms) {
  std::array<float, 2> position{};
  if (!get_position(position, timeout_ms)) {
    logger_.error("Mirror check failed: could not read both positions");
    return false;
  }
  mismatch_degrees = position[0] + position[1];
  if (std::fabs(mismatch_degrees) > tolerance_degrees) {
    logger_.error("Mirror check failed: primary={:.3f} deg, secondary={:.3f} deg, "
                  "mismatch={:.3f} deg (tolerance {:.3f} deg)",
                  position[0], position[1], mismatch_degrees, tolerance_degrees);
    return false;
  }
  logger_.info("Mirror check ok: primary={:.3f} deg, secondary={:.3f} deg, mismatch={:.3f} deg",
               position[0], position[1], mismatch_degrees);
  return true;
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
  // Read the pair, then command absolute targets: EyouMotor::move_absolute
  // clamps to the configured limits, EyouMotor::move_incremental does not.
  std::array<float, 2> position{};
  if (!get_position(position)) {
    logger_.error("move_incremental: could not read the pair's position; no move");
    return false;
  }
  const bool primary_ok = primary_.move_absolute(position[0] + increment_degrees, immediate);
  const bool secondary_ok = secondary_.move_absolute(position[1] - increment_degrees, immediate);
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
