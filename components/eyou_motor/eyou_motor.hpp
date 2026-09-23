#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <optional>
#include <span>

#include "base_component.hpp"
#include "can_bus.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

using EyouCanFrame = CanFrame;

class EyouMotor : public espp::BaseComponent {
public:
  struct ProfilePositionConfig {
    float velocity_degrees_per_second;
    float acceleration_degrees_per_second_squared;
    float deceleration_degrees_per_second_squared;
  };

  static constexpr float kPulsesPerOutputTurn = static_cast<float>((1U << 19) * 100U);
  static constexpr float kDegreesPerOutputTurn = 360.0f;

  EyouMotor(CanBus &bus, uint8_t node_id = 1);
  ~EyouMotor();

  EyouMotor(const EyouMotor &) = delete;
  EyouMotor &operator=(const EyouMotor &) = delete;

  // Send a raw CAN frame on the bus (bypasses the CANopen/DS402 layer).
  bool send_raw(uint32_t can_id, std::span<const uint8_t> data, bool extended = false,
                bool rtr = false) const;

  // Reset the CANopen node and configure it for Profile Position mode.
  bool configure_profile_position(const ProfilePositionConfig &config);
  // Read back the drive's current Profile Position velocity/accel/decel (objects 0x6081/0x6083/0x6084).
  bool get_profile_position_config(ProfilePositionConfig &config, uint32_t timeout_ms = 100);
  // Reset the software zero reference to the current actual position; set_position
  // (move_absolute) is relative to this reference.
  bool zero(uint32_t timeout_ms = 1000);
  // Move to an absolute position (degrees), relative to the software zero if set.
  // Retries up to 3 times if a send/acknowledge step fails.
  bool move_absolute(float target_degrees, bool immediate = true);
  // Move by a relative position increment (degrees) from the current position.
  bool move_incremental(float increment_degrees, bool immediate = true);
  // Drive the DS402 state machine to Operation Enabled.
  bool enable_drive(uint32_t timeout_ms = 1000);
  // Drive the DS402 state machine to Ready to Switch On (power stage off).
  bool disable_drive(uint32_t timeout_ms = 1000);
  // Clear a DS402 fault via a controlword rising edge.
  bool reset_fault(uint32_t timeout_ms = 1000);
  // Read the actual position (degrees), relative to the software zero if set.
  bool get_position(float &position_degrees, uint32_t timeout_ms = 100);
  // Read the DS402 statusword (object 0x6041).
  bool get_statusword(uint16_t &statusword, uint32_t timeout_ms = 100);
  // Read the active error code (object 0x603F).
  bool get_error_code(uint16_t &error_code, uint32_t timeout_ms = 100);
  // Read the statusword and report whether the drive is in the DS402 Fault state.
  // A communication failure is treated as faulted (fail-safe for monitoring loops).
  bool is_faulted(uint32_t timeout_ms = 100);
  // Read the modes-of-operation display (object 0x6061).
  bool get_operating_mode(int8_t &mode, uint32_t timeout_ms = 100);
  // Persist the current object dictionary (profile velocity/accel/decel, mode, etc.) to
  // non-volatile memory via CiA 301 object 0x1010 (signature "save"). Sub-index 3 covers the
  // DS402 application objects (0x6000-0x9FFF); sub-index 1 saves everything. Optional and
  // vendor-specific: call once during commissioning, not on every boot (finite EEPROM endurance).
  bool save_parameters(uint8_t sub_index = 3, uint32_t timeout_ms = 5000);
  // Decode a DS402 statusword's state bits into a human-readable name.
  static const char *ds402_state_name(uint16_t statusword);

private:
  bool send_sdo_u8(uint16_t index, uint8_t value, uint8_t subindex = 0, uint32_t timeout_ms = 100);
  bool send_sdo_u16(uint16_t index, uint16_t value, uint8_t subindex = 0, uint32_t timeout_ms = 100);
  bool send_sdo_u32(uint16_t index, uint32_t value, uint8_t subindex = 0, uint32_t timeout_ms = 100);
  bool send_sdo_download(uint16_t index, uint8_t subindex, uint8_t command, uint32_t value,
                        uint32_t timeout_ms = 100);
  bool reset_communication_and_wait_for_bootup(uint32_t timeout_ms);
  bool ensure_operation_enabled(uint32_t timeout_ms);
  bool ensure_profile_position_mode(uint32_t timeout_ms);
  // Same check as is_faulted(), but assumes transaction_mutex_ is already held.
  bool is_faulted_unlocked(uint32_t timeout_ms);
  bool wait_for_ds402_state(uint16_t expected_state, uint32_t timeout_ms);
  bool wait_for_operation_enabled(uint32_t timeout_ms);
  bool request_sdo_u32(uint16_t index, uint8_t expected_response_command, uint32_t &value,
                       uint32_t timeout_ms);
  bool trigger_profile_position(bool relative, bool immediate);
  bool degrees_to_pulses(float degrees, int32_t &pulses) const;
  bool degrees_to_profile_units(float degrees, uint32_t &pulses) const;
  static float pulses_to_degrees(int32_t pulses);
  static float profile_units_to_degrees(uint32_t pulses);

  CanBus &bus_;
  uint8_t node_id_{1};
  QueueHandle_t receive_queue_{nullptr};
  // Unset means get_position/move_absolute operate on the absolute encoder position.
  std::optional<int32_t> zero_offset_pulses_;
  mutable std::mutex transaction_mutex_;
};
