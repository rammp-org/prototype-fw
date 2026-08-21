#include "motor_actuator.hpp"

#include <algorithm>
#include <cmath>

namespace {
constexpr gpio_num_t kUnusedGpio = GPIO_NUM_NC;
}

MotorActuator::MotorActuator(espp::Logger &logger, gpio_num_t rx_gpio, gpio_num_t tx_gpio)
    : logger_(logger), rx_gpio_(rx_gpio), tx_gpio_(tx_gpio) {}

MotorActuator::~MotorActuator() {
  if (node_ != nullptr) {
    twai_node_disable(node_);
    twai_node_delete(node_);
  }
  if (receive_queue_ != nullptr) {
    vQueueDelete(receive_queue_);
  }
}

bool MotorActuator::on_receive(twai_node_handle_t handle, const twai_rx_done_event_data_t *,
                               void *context) {
  auto *actuator = static_cast<MotorActuator *>(context);
  CanPacket packet{};
  twai_frame_t frame{};
  frame.buffer = packet.data.data();
  frame.buffer_len = packet.data.size();
  if (twai_node_receive_from_isr(handle, &frame) != ESP_OK) {
    return false;
  }
  packet.id = frame.header.id;
  packet.length = twaifd_dlc2len(frame.header.dlc);
  packet.extended = frame.header.ide;
  BaseType_t higher_priority_task_woken = pdFALSE;
  xQueueSendFromISR(actuator->receive_queue_, &packet, &higher_priority_task_woken);
  return higher_priority_task_woken == pdTRUE;
}

bool MotorActuator::start(uint32_t bitrate) {
  if (node_ != nullptr) {
    return true;
  }
  receive_queue_ = xQueueCreate(8, sizeof(CanPacket));
  if (receive_queue_ == nullptr) {
    logger_.error("Failed to create motor CAN receive queue");
    return false;
  }

  twai_onchip_node_config_t config{};
  config.io_cfg.rx = rx_gpio_;
  config.io_cfg.tx = tx_gpio_;
  config.io_cfg.quanta_clk_out = kUnusedGpio;
  config.io_cfg.bus_off_indicator = kUnusedGpio;
  config.bit_timing.bitrate = bitrate;
  config.tx_queue_depth = 3;

  esp_err_t result = twai_new_node_onchip(&config, &node_);
  if (result != ESP_OK) {
    logger_.error("Failed to create motor CAN node: {}", esp_err_to_name(result));
    return false;
  }

  twai_event_callbacks_t callbacks{};
  callbacks.on_rx_done = on_receive;
  result = twai_node_register_event_callbacks(node_, &callbacks, this);
  if (result == ESP_OK) {
    result = twai_node_enable(node_);
  }
  if (result != ESP_OK) {
    logger_.error("Failed to start motor CAN node: {}", esp_err_to_name(result));
    twai_node_delete(node_);
    node_ = nullptr;
    return false;
  }

  logger_.info("Motor CAN started: RX GPIO {}, TX GPIO {}, {} bit/s", static_cast<int>(rx_gpio_),
               static_cast<int>(tx_gpio_), bitrate);
  return true;
}

bool MotorActuator::send_command(const std::array<uint8_t, packet_length_> &command) {
  if (node_ == nullptr) {
    logger_.error("Motor CAN is not started");
    return false;
  }
  twai_frame_t frame{};
  frame.header.id = motor_can_id_;
  frame.header.dlc = packet_length_;
  frame.buffer = const_cast<uint8_t *>(command.data());
  frame.buffer_len = command.size();
  esp_err_t result = twai_node_transmit(node_, &frame, 3);
  if (result == ESP_OK) {
    result = twai_node_transmit_wait_all_done(node_, 10);
  }
  if (result != ESP_OK) {
    logger_.error("Motor CAN transmit failed: {}", esp_err_to_name(result));
    return false;
  }
  return true;
}

bool MotorActuator::request(uint8_t command_code, CanPacket &response, uint32_t timeout_ms) {
  while (xQueueReceive(receive_queue_, &response, 0) == pdTRUE) {
  }
  std::array<uint8_t, packet_length_> command{};
  command[0] = command_code;
  if (!send_command(command)) {
    return false;
  }

  const TickType_t start_tick = xTaskGetTickCount();
  const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
  while (xTaskGetTickCount() - start_tick < timeout_ticks) {
    const TickType_t elapsed = xTaskGetTickCount() - start_tick;
    if (xQueueReceive(receive_queue_, &response, timeout_ticks - elapsed) != pdTRUE) {
      break;
    }
    const bool expected_reply_id = response.id == motor_reply_can_id_ || response.id == motor_can_id_;
    if (expected_reply_id && !response.extended &&
      response.length == packet_length_ &&
        response.data[0] == command_code) {
      return true;
    }
  }
  return false;
}

bool MotorActuator::read_status(Status &status, uint32_t timeout_ms) {
  CanPacket response{};
  if (!request(0x9C, response, timeout_ms)) {
    logger_.warn("No motor status response received");
    return false;
  }
  status.data = response.data;
  status.temperature_c = response.data[1];
  status.torque_raw = read_i16(response.data, 2);
  status.velocity_rpm = static_cast<float>(read_i16(response.data, 4)) / (gear_ratio_ * 6.0f);
  status.angle_degrees = static_cast<float>(read_i16(response.data, 6)) / gear_ratio_;
  return true;
}

bool MotorActuator::is_connected(uint32_t timeout_ms) {
  Status status{};
  return read_status(status, timeout_ms);
}

bool MotorActuator::read_temperature(int &temperature_c, uint32_t timeout_ms) {
  Status status{};
  if (!read_status(status, timeout_ms)) return false;
  temperature_c = status.temperature_c;
  return true;
}

bool MotorActuator::read_angle(float &angle_degrees, uint32_t timeout_ms) {
  Status status{};
  if (!read_status(status, timeout_ms)) return false;
  angle_degrees = status.angle_degrees;
  return true;
}

bool MotorActuator::read_velocity(float &velocity_rpm, uint32_t timeout_ms) {
  Status status{};
  if (!read_status(status, timeout_ms)) return false;
  velocity_rpm = status.velocity_rpm;
  return true;
}

bool MotorActuator::read_torque(int16_t &torque_raw, uint32_t timeout_ms) {
  Status status{};
  if (!read_status(status, timeout_ms)) return false;
  torque_raw = status.torque_raw;
  return true;
}

bool MotorActuator::send_torque(int16_t torque_raw) {
  std::array<uint8_t, packet_length_> command{};
  command[0] = 0xA1;
  set_i16(command, 4, torque_raw);
  return send_command(command);
}

bool MotorActuator::send_velocity(float velocity_rpm) {
  const float motor_velocity_raw = velocity_rpm * gear_ratio_ * 6.0f * 100.0f;
  if (motor_velocity_raw < static_cast<float>(INT32_MIN) ||
      motor_velocity_raw > static_cast<float>(INT32_MAX)) {
    logger_.error("Requested velocity is out of range");
    return false;
  }
  std::array<uint8_t, packet_length_> command{};
  command[0] = 0xA2;
  set_i32(command, 4, static_cast<int32_t>(std::lround(motor_velocity_raw)));
  return send_command(command);
}

void MotorActuator::zero_position() {
  virtual_position_degrees_ = 0.0f;
  logger_.info("Virtual motor position set to zero");
}

bool MotorActuator::set_position(float position_degrees, float max_speed_rpm) {
  const float delta_degrees = position_degrees - virtual_position_degrees_;
  if (!send_incremental_position(delta_degrees, max_speed_rpm)) {
    return false;
  }
  virtual_position_degrees_ = position_degrees;
  return true;
}

bool MotorActuator::send_incremental_position(float delta_degrees, float max_speed_rpm) {
  const float motor_delta_centidegrees = delta_degrees * gear_ratio_ * 100.0f;
  const float motor_speed_dps = max_speed_rpm * gear_ratio_ * 6.0f;
  if (motor_delta_centidegrees < static_cast<float>(INT32_MIN) ||
      motor_delta_centidegrees > static_cast<float>(INT32_MAX) || motor_speed_dps < 0.0f ||
      motor_speed_dps > static_cast<float>(UINT16_MAX)) {
    logger_.error("Requested incremental position or speed is out of range");
    return false;
  }
  std::array<uint8_t, packet_length_> command{};
  command[0] = 0xA8;
  const uint16_t speed_limit_dps = static_cast<uint16_t>(std::lround(motor_speed_dps));
  command[2] = static_cast<uint8_t>(speed_limit_dps & 0xFF);
  command[3] = static_cast<uint8_t>((speed_limit_dps >> 8) & 0xFF);
  set_i32(command, 4, static_cast<int32_t>(std::lround(motor_delta_centidegrees)));
  return send_command(command);
}

bool MotorActuator::stop() {
  std::array<uint8_t, packet_length_> command{};
  command[0] = 0x81;
  return send_command(command);
}

bool MotorActuator::disable() { return send_torque(0); }
bool MotorActuator::hold() { return send_velocity(0.0f); }

bool MotorActuator::release_brake() {
  std::array<uint8_t, packet_length_> command{};
  command[0] = 0x77;
  return send_command(command);
}

bool MotorActuator::lock_brake() {
  std::array<uint8_t, packet_length_> command{};
  command[0] = 0x78;
  return send_command(command);
}

bool MotorActuator::write_default_pid_rom() {
  std::array<uint8_t, packet_length_> command{0x32, 0, 0x64, 0x64, 0x32, 0x28, 0x32, 0x32};
  return send_command(command);
}

bool MotorActuator::write_default_pid_ram() {
  std::array<uint8_t, packet_length_> command{0x31, 0, 0x64, 0x64, 0x32, 0x28, 0x32, 0x32};
  return send_command(command);
}

bool MotorActuator::write_global_pid_gain(float gain) {
  gain = std::clamp(gain, 0.0f, 1.0f);
  std::array<uint8_t, packet_length_> command{
      0x31, 0, static_cast<uint8_t>(std::lround(gain * 100.0f)),
      static_cast<uint8_t>(std::lround(gain * 100.0f)),
      static_cast<uint8_t>(std::lround(gain * 50.0f)),
      static_cast<uint8_t>(std::lround(gain * 40.0f)),
      static_cast<uint8_t>(std::lround(gain * 50.0f)),
      static_cast<uint8_t>(std::lround(gain * 50.0f)),
  };
  return send_command(command);
}

int16_t MotorActuator::read_i16(const std::array<uint8_t, 8> &data, size_t index) {
  return static_cast<int16_t>(static_cast<uint16_t>(data[index]) |
                              (static_cast<uint16_t>(data[index + 1]) << 8));
}

int32_t MotorActuator::read_i32(const std::array<uint8_t, 8> &data, size_t index) {
  return static_cast<int32_t>(static_cast<uint32_t>(data[index]) |
                              (static_cast<uint32_t>(data[index + 1]) << 8) |
                              (static_cast<uint32_t>(data[index + 2]) << 16) |
                              (static_cast<uint32_t>(data[index + 3]) << 24));
}

void MotorActuator::set_i16(std::array<uint8_t, 8> &data, size_t index, int16_t value) {
  data[index] = static_cast<uint8_t>(value & 0xFF);
  data[index + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void MotorActuator::set_i32(std::array<uint8_t, 8> &data, size_t index, int32_t value) {
  data[index] = static_cast<uint8_t>(value & 0xFF);
  data[index + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  data[index + 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  data[index + 3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}
