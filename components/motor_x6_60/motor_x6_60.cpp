#include "motor_x6_60.hpp"

#include <algorithm>
#include <cmath>
#include <climits>
#include <utility>

MotorX660::MotorX660(CommunicationFunction communication, uint8_t motor_id, float gear_ratio)
    : espp::BaseComponent("MotorX660", espp::Logger::Verbosity::INFO),
      communication_(std::move(communication)), motor_id_(motor_id), gear_ratio_(gear_ratio) {
  set_log_tag("MotorX660-" + std::to_string(motor_id_));
}

bool MotorX660::send_command(const std::array<uint8_t, kPacketLength> &command) {
  MotorPacket packet{};
  packet.id = motor_id_;
  packet.length = kPacketLength;
  packet.data = command;
  MotorPacket response{};
  return communication_ && communication_(packet, response, 0);
}

bool MotorX660::request(uint8_t command_code, MotorPacket &response, uint32_t timeout_ms) {
  if (!communication_) return false;
  MotorPacket command{};
  command.id = motor_id_;
  command.length = kPacketLength;
  command.data[0] = command_code;
  return communication_(command, response, timeout_ms);
}

bool MotorX660::read_status(Status &status, uint32_t timeout_ms) {
  MotorPacket response{};
  if (!request(0x9C, response, timeout_ms)) return false;
  status.data = response.data;
  status.temperature_c = static_cast<int8_t>(response.data[1]);
  status.torque_raw = read_i16(response.data, 2);
  status.velocity_rpm = static_cast<float>(read_i16(response.data, 4)) * 60.0f /
                        (100.0f * gear_ratio_);
  status.angle_degrees = static_cast<float>(read_i16(response.data, 6));
  return true;
}

bool MotorX660::is_connected(uint32_t timeout_ms) {
  Status status{};
  return read_status(status, timeout_ms);
}

bool MotorX660::read_multi_turn_encoder(int32_t &encoder_value, uint32_t timeout_ms) {
  MotorPacket response{};
  if (!request(0x60, response, timeout_ms)) return false;
  encoder_value = read_i32(response.data, 4);
  return true;
}

bool MotorX660::read_temperature(int &temperature_c, uint32_t timeout_ms) {
  Status status{};
  if (!read_status(status, timeout_ms)) return false;
  temperature_c = status.temperature_c;
  return true;
}

bool MotorX660::read_angle(float &angle_degrees, uint32_t timeout_ms) {
  Status status{};
  if (!read_status(status, timeout_ms)) return false;
  angle_degrees = status.angle_degrees;
  return true;
}

bool MotorX660::read_velocity(float &velocity_rpm, uint32_t timeout_ms) {
  Status status{};
  if (!read_status(status, timeout_ms)) return false;
  velocity_rpm = status.velocity_rpm;
  return true;
}

bool MotorX660::read_torque(int16_t &torque_raw, uint32_t timeout_ms) {
  Status status{};
  if (!read_status(status, timeout_ms)) return false;
  torque_raw = status.torque_raw;
  return true;
}

bool MotorX660::send_torque(int16_t torque_raw) {
  std::array<uint8_t, kPacketLength> command{};
  command[0] = 0xA1;
  set_i16(command, 4, torque_raw);
  return send_command(command);
}

bool MotorX660::send_velocity(float velocity_rpm) {
  const float velocity_raw = velocity_rpm * gear_ratio_ * 100.0f;
  if (velocity_raw < static_cast<float>(INT32_MIN) ||
      velocity_raw > static_cast<float>(INT32_MAX)) return false;
  std::array<uint8_t, kPacketLength> command{};
  command[0] = 0xA2;
  set_i32(command, 4, static_cast<int32_t>(std::lround(velocity_raw)));
  return send_command(command);
}

bool MotorX660::set_position_limits(float minimum_degrees, float maximum_degrees) {
  if (minimum_degrees > maximum_degrees) return false;
  minimum_position_degrees_ = minimum_degrees;
  maximum_position_degrees_ = maximum_degrees;
  return true;
}

bool MotorX660::set_position(float position_degrees, float max_speed_rpm) {
  const float limited_position =
      std::clamp(position_degrees, minimum_position_degrees_, maximum_position_degrees_);
  if (!send_incremental_position(limited_position - virtual_position_degrees_, max_speed_rpm)) {
    return false;
  }
  virtual_position_degrees_ = limited_position;
  return true;
}

bool MotorX660::set_absolute_position(float position_degrees, float max_speed_rpm) {
  const float limited_position =
      std::clamp(position_degrees, minimum_position_degrees_, maximum_position_degrees_);
  const float speed_raw = max_speed_rpm * gear_ratio_ * 6.0f;
  const float position_raw = limited_position * 100.0f;
  if (speed_raw < 0.0f || speed_raw > static_cast<float>(UINT16_MAX) ||
      position_raw < static_cast<float>(INT32_MIN) ||
      position_raw > static_cast<float>(INT32_MAX)) {
    return false;
  }

  std::array<uint8_t, kPacketLength> command{};
  command[0] = 0xA4;
  set_u16(command, 2, static_cast<uint16_t>(std::lround(speed_raw)));
  set_i32(command, 4, static_cast<int32_t>(std::lround(position_raw)));
  if (!send_command(command)) return false;
  virtual_position_degrees_ = limited_position;
  return true;
}

bool MotorX660::send_incremental_position(float delta_degrees, float max_speed_rpm) {
  const float angle_raw = delta_degrees * 100.0f;
  const float speed_raw = max_speed_rpm * 6.0f;
  if (angle_raw < static_cast<float>(INT32_MIN) || angle_raw > static_cast<float>(INT32_MAX) ||
      speed_raw < 0.0f || speed_raw > 65535.0f) return false;
  std::array<uint8_t, kPacketLength> command{};
  command[0] = 0xA8;
  set_u16(command, 2, static_cast<uint16_t>(std::lround(speed_raw)));
  set_i32(command, 4, static_cast<int32_t>(std::lround(angle_raw)));
  return send_command(command);
}

bool MotorX660::stop() {
  std::array<uint8_t, kPacketLength> command{};
  command[0] = 0x81;
  return send_command(command);
}

bool MotorX660::disable() {
  std::array<uint8_t, kPacketLength> command{};
  command[0] = 0x80;
  return send_command(command);
}

bool MotorX660::hold() { return send_velocity(0.0f); }

bool MotorX660::reset_system() {
  std::array<uint8_t, kPacketLength> command{};
  command[0] = 0x76;
  return send_command(command);
}

bool MotorX660::release_brake() {
  std::array<uint8_t, kPacketLength> command{};
  command[0] = 0x77;
  return send_command(command);
}

bool MotorX660::lock_brake() {
  std::array<uint8_t, kPacketLength> command{};
  command[0] = 0x78;
  return send_command(command);
}

int16_t MotorX660::read_i16(const std::array<uint8_t, 8> &data, size_t index) {
  return static_cast<int16_t>(static_cast<uint16_t>(data[index]) |
                              (static_cast<uint16_t>(data[index + 1]) << 8));
}

int32_t MotorX660::read_i32(const std::array<uint8_t, 8> &data, size_t index) {
  return static_cast<int32_t>(static_cast<uint32_t>(data[index]) |
                              (static_cast<uint32_t>(data[index + 1]) << 8) |
                              (static_cast<uint32_t>(data[index + 2]) << 16) |
                              (static_cast<uint32_t>(data[index + 3]) << 24));
}

void MotorX660::set_u16(std::array<uint8_t, 8> &data, size_t index, uint16_t value) {
  data[index] = static_cast<uint8_t>(value & 0xFF);
  data[index + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void MotorX660::set_i16(std::array<uint8_t, 8> &data, size_t index, int16_t value) {
  data[index] = static_cast<uint8_t>(value & 0xFF);
  data[index + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void MotorX660::set_i32(std::array<uint8_t, 8> &data, size_t index, int32_t value) {
  data[index] = static_cast<uint8_t>(value & 0xFF);
  data[index + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  data[index + 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  data[index + 3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}