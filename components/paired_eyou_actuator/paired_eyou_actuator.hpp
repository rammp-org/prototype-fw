#pragma once

#include <array>
#include <cstdint>

#include "base_component.hpp"
#include "eyou_motor.hpp"

// Drives two Eyou motors as a mechanically paired pair (e.g. a mirrored linkage):
// the secondary always receives the negated command of the primary.
class PairedEyouActuator : public espp::BaseComponent {
public:
  struct Status {
    std::array<uint16_t, 2> statusword{};
    std::array<uint16_t, 2> error_code{};
  };

  PairedEyouActuator(EyouMotor &primary, EyouMotor &secondary);

  PairedEyouActuator(const PairedEyouActuator &) = delete;
  PairedEyouActuator &operator=(const PairedEyouActuator &) = delete;

  bool configure_profile_position(const EyouMotor::ProfilePositionConfig &config);
  bool enable_drive(uint32_t timeout_ms = 1000);
  bool disable_drive(uint32_t timeout_ms = 1000);
  bool reset_fault(uint32_t timeout_ms = 1000);
  // True if either motor reports the DS402 Fault state (or its status can't be read).
  bool is_faulted(uint32_t timeout_ms = 100);
  bool zero(uint32_t timeout_ms = 1000);
  // Persist both motors' software zero offsets to NVS, keyed by node id.
  bool save_zero();
  // Load both motors' software zero offsets from NVS, keyed by node id.
  bool load_zero();
  // Restrict set_position/move_incremental targets to [minimum_degrees, maximum_degrees] on the
  // primary; the secondary is limited to the negated range.
  bool set_position_limits(float minimum_degrees, float maximum_degrees);
  // Index 0 is the primary actuator; index 1 is the secondary actuator (negated command).
  bool get_position(std::array<float, 2> &position_degrees, uint32_t timeout_ms = 100);
  bool get_status(Status &status, uint32_t timeout_ms = 100);
  bool set_position(float position_degrees, bool immediate = true);
  bool move_incremental(float increment_degrees, bool immediate = true);

  uint8_t primary_id() const { return primary_.node_id(); }
  uint8_t secondary_id() const { return secondary_.node_id(); }

private:
  EyouMotor &primary_;
  EyouMotor &secondary_;
};
