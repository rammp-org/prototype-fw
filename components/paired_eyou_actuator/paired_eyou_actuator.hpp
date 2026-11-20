#pragma once

#include <array>
#include <cstdint>
#include <string_view>

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

  /// `tag` names the pair in its log lines; it must outlive the object (the
  /// logger keeps a view of it): a string literal such as
  /// "PairedEyouActuator-left" is the intended use.
  PairedEyouActuator(EyouMotor &primary, EyouMotor &secondary,
                     std::string_view tag = "PairedEyouActuator");

  PairedEyouActuator(const PairedEyouActuator &) = delete;
  PairedEyouActuator &operator=(const PairedEyouActuator &) = delete;

  bool configure_profile_position(const EyouMotor::ProfilePositionConfig &config, bool enable = true);
  bool enable_drive(uint32_t timeout_ms = 1000);
  bool disable_drive(uint32_t timeout_ms = 1000);
  // Stop the move in progress on both motors with the profile deceleration and
  // hold there, drives still enabled (EyouMotor::halt). Both are asked even if
  // the first fails, so a pair never ends with one side stopped and the other
  // still moving; false if either did not confirm.
  bool halt(uint32_t timeout_ms = 2000);
  // The drives' Max torque (0x6072), per-thousand of the rated torque, on both
  // motors: a move cannot push harder than this in any mode.
  bool set_max_torque(uint16_t torque_permille, uint32_t timeout_ms = 100);
  bool reset_fault(uint32_t timeout_ms = 1000);
  // True if either motor reports the DS402 Fault state (or its status can't be read).
  bool is_faulted(uint32_t timeout_ms = 100);
  bool zero(uint32_t timeout_ms = 1000);
  // Persist both motors' software zero offsets to NVS, keyed by node id.
  bool save_zero();
  // Read both saved software zero offsets without changing either active offset.
  bool read_zero(std::array<int32_t, 2> &zero_offset_pulses);
  // Load both motors' software zero offsets from NVS, keyed by node id.
  bool load_zero();
  // Verify the pair is mirrored: |primary + secondary| <= tolerance_degrees. A failed position
  // read also returns false. `mismatch_degrees` receives primary + secondary.
  bool check_mirrored(float tolerance_degrees, float &mismatch_degrees, uint32_t timeout_ms = 100);
  // Restrict set_position/move_incremental targets to [minimum_degrees, maximum_degrees] on the
  // primary; the secondary is limited to the negated range (move_incremental reads the pair and
  // commands limit-checked absolute targets).
  bool set_position_limits(float minimum_degrees, float maximum_degrees);
  // Index 0 is the primary actuator; index 1 is the secondary actuator (negated command).
  bool get_position(std::array<float, 2> &position_degrees, uint32_t timeout_ms = 100);
  bool get_status(Status &status, uint32_t timeout_ms = 100);
  bool set_position(float position_degrees, bool immediate = true);
  bool move_incremental(float increment_degrees, bool immediate = true);
  // Position streaming (EyouMotor::stream_setpoint on both, the secondary
  // mirrored). ready = both drives can take a set-point now (send to both or
  // neither, so the pair never splits).
  bool setpoint_ready(bool &ready);
  bool stream_setpoint(float position_degrees, float velocity_degrees_per_second);
  // Profile velocity / acceleration / deceleration on both motors (no mode change).
  bool set_profile_position_config(const EyouMotor::ProfilePositionConfig &config);
  // Hold where each motor is (EyouMotor::hold_position on both).
  bool hold_position();

  uint8_t primary_id() const { return primary_.node_id(); }
  uint8_t secondary_id() const { return secondary_.node_id(); }

private:
  EyouMotor &primary_;
  EyouMotor &secondary_;
};
