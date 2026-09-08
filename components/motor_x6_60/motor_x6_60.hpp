#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <limits>

#include "base_component.hpp"
#include "motor_actuator.hpp"

class MotorX660 : public espp::BaseComponent {
public:
  using CommunicationFunction =
      std::function<bool(const MotorPacket &command, MotorPacket &response,
                         uint32_t timeout_ms)>;

  struct Status {
    std::array<uint8_t, 8> data{};
    int temperature_c{0};
    int16_t torque_raw{0};
    float velocity_rpm{0.0f};
    float angle_degrees{0.0f};
  };

  explicit MotorX660(CommunicationFunction communication, uint8_t motor_id,
                     float gear_ratio = 6.0f);

  MotorX660(const MotorX660 &) = delete;
  MotorX660 &operator=(const MotorX660 &) = delete;

  bool is_connected(uint32_t timeout_ms = 100);
  bool read_status(Status &status, uint32_t timeout_ms = 100);
  bool read_multi_turn_encoder(int32_t &encoder_value, uint32_t timeout_ms = 100);
  bool read_temperature(int &temperature_c, uint32_t timeout_ms = 100);
  bool read_angle(float &angle_degrees, uint32_t timeout_ms = 100);
  bool read_velocity(float &velocity_rpm, uint32_t timeout_ms = 100);
  bool read_torque(int16_t &torque_raw, uint32_t timeout_ms = 100);

  bool send_torque(int16_t torque_raw);
  bool send_velocity(float velocity_rpm);
  bool set_position_limits(float minimum_degrees, float maximum_degrees);
  bool set_position(float position_degrees, float max_speed_rpm);
  bool set_absolute_position(float position_degrees, float max_speed_rpm);
  bool send_incremental_position(float delta_degrees, float max_speed_rpm);
  bool stop();
  bool disable();
  bool hold();
  bool reset_system();
  bool release_brake();
  bool lock_brake();

  void zero_position() { virtual_position_degrees_ = 0.0f; }
  float get_position() const { return virtual_position_degrees_; }
  uint8_t get_motor_id() const { return motor_id_; }
  float get_gear_ratio() const { return gear_ratio_; }

private:
  static constexpr size_t kPacketLength = 8;

  bool send_command(const std::array<uint8_t, kPacketLength> &command);
  bool request(uint8_t command_code, MotorPacket &response, uint32_t timeout_ms);
  static int16_t read_i16(const std::array<uint8_t, 8> &data, size_t index);
  static int32_t read_i32(const std::array<uint8_t, 8> &data, size_t index);
  static void set_u16(std::array<uint8_t, 8> &data, size_t index, uint16_t value);
  static void set_i16(std::array<uint8_t, 8> &data, size_t index, int16_t value);
  static void set_i32(std::array<uint8_t, 8> &data, size_t index, int32_t value);

  CommunicationFunction communication_;
  uint8_t motor_id_;
  float gear_ratio_;
  float virtual_position_degrees_{0.0f};
  float minimum_position_degrees_{-std::numeric_limits<float>::infinity()};
  float maximum_position_degrees_{std::numeric_limits<float>::infinity()};
};