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
  status.velocity_raw = read_i16(response.data, 4);
  status.angle_raw = read_i16(response.data, 6);
  return true;
}

bool MotorActuator::read_motor_model(std::array<char, 8> &model, uint32_t timeout_ms) {
  CanPacket response{};
  if (!request(0xB5, response, timeout_ms)) {
    logger_.warn("No motor model response received");
    return false;
  }
  for (size_t index = 0; index < 7; ++index) {
    model[index] = static_cast<char>(response.data[index + 1]);
  }
  model[7] = '\0';
  return true;
}

bool MotorActuator::read_software_version_date(uint32_t &version_date, uint32_t timeout_ms) {
  CanPacket response{};
  if (!request(0xB2, response, timeout_ms)) {
    logger_.warn("No software version response received");
    return false;
  }
  version_date = static_cast<uint32_t>(read_i32(response.data, 4));
  return true;
}

// The current motor firmware returns zero for the 0x92/0x60 multi-turn reads;
// use the working 0x9C status angle for position feedback instead.
bool MotorActuator::read_multi_turn_position(int32_t &position_raw, uint32_t timeout_ms) {
  CanPacket response{};
  // 0x92 is the multi-turn absolute angle used by the original actuator code
  // and provides the 0.01-degree units required by the 0xA4 command.
  if (request(0x92, response, timeout_ms)) {
    position_raw = read_i32(response.data, 4);
    return true;
  }

  // Fall back to the encoder-pulse command for firmware that supports 0x60
  // but not the multi-turn angle command.
  if (request(0x60, response, timeout_ms)) {
    position_raw = read_i32(response.data, 4);
    return true;
  }

  logger_.warn("No multi-turn encoder or angle response received");
  return false;
}

bool MotorActuator::read_multi_turn_raw_position(int32_t &position_raw, uint32_t timeout_ms) {
  CanPacket response{};
  if (!request(0x61, response, timeout_ms)) {
    logger_.warn("No 0x61 multi-turn raw position response received");
    return false;
  }
  position_raw = read_i32(response.data, 4);
  return true;
}

bool MotorActuator::read_multi_turn_zero_offset(int32_t &offset_raw, uint32_t timeout_ms) {
  CanPacket response{};
  if (!request(0x62, response, timeout_ms)) {
    logger_.warn("No 0x62 multi-turn zero offset response received");
    return false;
  }
  offset_raw = read_i32(response.data, 4);
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

bool MotorActuator::read_angle(int32_t &angle_raw, uint32_t timeout_ms) {
  Status status{};
  if (!read_status(status, timeout_ms)) return false;
  angle_raw = status.angle_raw;
  return true;
}

bool MotorActuator::read_velocity(int16_t &velocity_raw, uint32_t timeout_ms) {
  Status status{};
  if (!read_status(status, timeout_ms)) return false;
  velocity_raw = status.velocity_raw;
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

bool MotorActuator::send_velocity(int32_t velocity_raw) {
  std::array<uint8_t, packet_length_> command{};
  command[0] = 0xA2;
  set_i32(command, 4, velocity_raw);
  return send_command(command);
}

bool MotorActuator::zero_position(uint32_t timeout_ms) {
  CanPacket response{};
  if (!request(0x64, response, timeout_ms)) {
    logger_.warn("Motor did not acknowledge position zero command");
    return false;
  }
  logger_.info("Motor current position saved as zero");
  return true;
}

bool MotorActuator::write_multi_turn_zero_offset(int32_t offset_raw, uint32_t timeout_ms) {
  std::array<uint8_t, packet_length_> command{};
  command[0] = 0x63;
  set_i32(command, 4, offset_raw);
  CanPacket response{};
  while (xQueueReceive(receive_queue_, &response, 0) == pdTRUE) {
  }
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
    if ((response.id == motor_reply_can_id_ || response.id == motor_can_id_) &&
        !response.extended && response.length == packet_length_ && response.data[0] == 0x63) {
      return true;
    }
  }
  logger_.warn("No 0x63 write zero offset response received");
  return false;
}

bool MotorActuator::write_current_position_as_zero(uint32_t timeout_ms) {
  CanPacket response{};
  if (!request(0x64, response, timeout_ms)) {
    logger_.warn("No 0x64 write current position as zero response received");
    return false;
  }
  return true;
}

bool MotorActuator::set_position(int32_t position_centidegrees, uint16_t max_speed_dps) {
  std::array<uint8_t, packet_length_> command{};
  command[0] = 0xA4;
  set_i16(command, 2, static_cast<int16_t>(max_speed_dps));
  set_i32(command, 4, position_centidegrees);
  return send_command(command);
}

bool MotorActuator::send_incremental_position(int32_t delta_centidegrees,
                                               uint16_t max_speed_dps) {
  std::array<uint8_t, packet_length_> command{};
  command[0] = 0xA8;
  set_i16(command, 2, static_cast<int16_t>(max_speed_dps));
  set_i32(command, 4, delta_centidegrees);
  return send_command(command);
}

bool MotorActuator::stop() {
  std::array<uint8_t, packet_length_> command{};
  command[0] = 0x81;
  return send_command(command);
}

bool MotorActuator::disable() { return send_torque(0); }
bool MotorActuator::hold() { return send_velocity(0); }

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
