#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <span>

#include "base_component.hpp"
#include "canopen_bus.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

using EyouCanFrame = CanopenFrame;

class EyouMotor : public espp::BaseComponent {
public:
  struct ProfilePositionConfig {
    float velocity_degrees_per_second;
    float acceleration_degrees_per_second_squared;
    float deceleration_degrees_per_second_squared;
  };

  static constexpr float kPulsesPerOutputTurn = static_cast<float>((1U << 19) * 100U);
  static constexpr float kDegreesPerOutputTurn = 360.0f;

  EyouMotor(CanopenBus &bus, uint8_t node_id = 1);
  ~EyouMotor();

  EyouMotor(const EyouMotor &) = delete;
  EyouMotor &operator=(const EyouMotor &) = delete;

  bool send_raw(uint32_t can_id, std::span<const uint8_t> data, bool extended = false,
                bool rtr = false) const;

  bool configure_profile_position(const ProfilePositionConfig &config);
  bool move_absolute(float target_degrees, bool immediate = false);
  bool move_incremental(float increment_degrees, bool immediate = true);
  bool enable_drive(uint32_t timeout_ms = 1000);
  bool disable_drive(uint32_t timeout_ms = 1000);
  bool reset_fault(uint32_t timeout_ms = 1000);
  bool get_position(float &position_degrees, uint32_t timeout_ms = 100);
  bool get_statusword(uint16_t &statusword, uint32_t timeout_ms = 100);
  bool get_error_code(uint16_t &error_code, uint32_t timeout_ms = 100);
  bool get_operating_mode(int8_t &mode, uint32_t timeout_ms = 100);
  static const char *ds402_state_name(uint16_t statusword);

private:
  bool send_sdo_u8(uint16_t index, uint8_t value);
  bool send_sdo_u16(uint16_t index, uint16_t value);
  bool send_sdo_u32(uint16_t index, uint32_t value);
  bool send_sdo_download(uint16_t index, uint8_t command, uint32_t value);
  bool reset_communication_and_wait_for_bootup(uint32_t timeout_ms);
  bool ensure_operation_enabled(uint32_t timeout_ms);
  bool wait_for_ds402_state(uint16_t expected_state, uint32_t timeout_ms);
  bool wait_for_operation_enabled(uint32_t timeout_ms);
  bool request_sdo_u32(uint16_t index, uint8_t expected_response_command, uint32_t &value,
                       uint32_t timeout_ms);
  bool trigger_profile_position(bool relative, bool immediate);
  bool degrees_to_pulses(float degrees, int32_t &pulses) const;
  bool degrees_to_profile_units(float degrees, uint32_t &pulses) const;
  static float pulses_to_degrees(int32_t pulses);

  CanopenBus &bus_;
  uint8_t node_id_{1};
  QueueHandle_t receive_queue_{nullptr};
  mutable std::mutex transaction_mutex_;
};
