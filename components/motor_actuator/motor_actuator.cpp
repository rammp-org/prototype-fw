#include "motor_actuator.hpp"

#include <algorithm>
#include <cmath>

namespace {
constexpr gpio_num_t kUnusedGpio = GPIO_NUM_NC;
constexpr size_t kPacketLength = 8;
}

MotorCanBus::MotorCanBus(gpio_num_t rx_gpio, gpio_num_t tx_gpio)
    : espp::BaseComponent("MotorCanBus", espp::Logger::Verbosity::INFO), rx_gpio_(rx_gpio),
      tx_gpio_(tx_gpio) {}

MotorCanBus::~MotorCanBus() {
  if (node_ != nullptr) {
    twai_node_disable(node_);
    twai_node_delete(node_);
  }
  if (receive_queue_ != nullptr) {
    vQueueDelete(receive_queue_);
  }
}

bool MotorCanBus::on_receive(twai_node_handle_t handle, const twai_rx_done_event_data_t *,
                             void *context) {
  auto *bus = static_cast<MotorCanBus *>(context);
  MotorPacket packet{};
  twai_frame_t frame{};
  frame.buffer = packet.data.data();
  frame.buffer_len = packet.data.size();
  if (twai_node_receive_from_isr(handle, &frame) != ESP_OK) {
    return false;
  }
  packet.id = frame.header.id;
  packet.length = twaifd_dlc2len(frame.header.dlc);
  BaseType_t higher_priority_task_woken = pdFALSE;
  xQueueSendFromISR(bus->receive_queue_, &packet, &higher_priority_task_woken);
  return higher_priority_task_woken == pdTRUE;
}

bool MotorCanBus::start(uint32_t bitrate) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  if (node_ != nullptr) {
    return true;
  }
  receive_queue_ = xQueueCreate(32, sizeof(MotorPacket));
  if (receive_queue_ == nullptr) {
    logger_.error("Failed to create CAN receive queue");
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

  logger_.info("CAN bus started: RX GPIO {}, TX GPIO {}, {} bit/s", static_cast<int>(rx_gpio_),
               static_cast<int>(tx_gpio_), bitrate);
  return true;
}

bool MotorCanBus::send(const MotorPacket &command) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  // Fire-and-forget: enqueue non-blocking, drop if the TX queue is full.
  return send_unlocked(command, /*wait_for_queue_space=*/false);
}

bool MotorCanBus::send_unlocked(const MotorPacket &command, bool wait_for_queue_space) {
  if (node_ == nullptr) {
    logger_.error("Motor CAN is not started");
    return false;
  }
  if (command.id < 1 || command.id > 32 || command.length > kPacketLength) {
    logger_.error("Invalid CAN motor packet: id={}, length={}", command.id, command.length);
    return false;
  }
  // Copy into a persistent ring slot: the on-chip TWAI TX queue is zero-copy and
  // reads this frame/buffer asynchronously from the ISR, so it must not live on
  // the stack. Only advance the ring after a successful enqueue, so a slot is
  // never reused while the driver may still reference it.
  TxSlot &slot = tx_ring_[tx_ring_index_];
  slot.payload = command.data;
  slot.frame = twai_frame_t{};
  slot.frame.header.id = command.data[0] == 0x79 ? 0x300u : 0x140u + command.id;
  slot.frame.header.dlc = command.length;
  slot.frame.buffer = slot.payload.data();
  slot.frame.buffer_len = command.length;

  // Never wait on wire completion: with no bus ACK (e.g. no motor attached) that
  // would stall for the full timeout on every send. A brief enqueue wait is
  // allowed for the request path so a poll is not dropped under transient queue
  // pressure; fire-and-forget uses a non-blocking enqueue. The request/response
  // path gets its reply via the RX queue, so it needs no TX-completion wait.
  const uint32_t enqueue_timeout_ms = wait_for_queue_space ? 3 : 0;
  esp_err_t result = twai_node_transmit(node_, &slot.frame, enqueue_timeout_ms);
  if (result == ESP_OK) {
    tx_ring_index_ = (tx_ring_index_ + 1) % kTxRingSize;
    return true;
  }
  if (wait_for_queue_space) {
    logger_.warn("Motor CAN transmit failed: {}", esp_err_to_name(result));
  } else {
    // Expected and self-correcting when nothing is draining the bus; keep it off
    // the error path so it does not spam a real-time control loop.
    logger_.debug("Motor CAN frame dropped (non-blocking): {}", esp_err_to_name(result));
  }
  return false;
}

bool MotorCanBus::request(const MotorPacket &command, MotorPacket &response,
                          uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  if (receive_queue_ == nullptr) {
    logger_.error("Motor CAN is not started");
    return false;
  }
  while (xQueueReceive(receive_queue_, &response, 0) == pdTRUE) {
  }
  if (!send_unlocked(command)) {
    return false;
  }

  const TickType_t start_tick = xTaskGetTickCount();
  const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
  while (xTaskGetTickCount() - start_tick < timeout_ticks) {
    const TickType_t elapsed = xTaskGetTickCount() - start_tick;
    if (xQueueReceive(receive_queue_, &response, timeout_ticks - elapsed) != pdTRUE) {
      break;
    }
    logger_.info("CAN RX id=0x{:x} cmd=0x{:02x} length={}", response.id, response.data[0],
           response.length);
    const uint32_t expected_reply_id_value = command.data[0] == 0x79
                           ? 0x300u
                           : 0x240u + command.id;
    const bool expected_reply_id = response.id == expected_reply_id_value ||
                     response.id == (command.data[0] == 0x79
                               ? 0x300u
                               : 0x140u + command.id);
    if (expected_reply_id && response.length == kPacketLength &&
      response.data[0] == command.data[0]) {
      response.id = command.id;
      return true;
    }
  }
  return false;
}

MotorActuator::MotorActuator(CommunicationFunction communication, uint8_t motor_id)
    : espp::BaseComponent("MotorActuator", espp::Logger::Verbosity::INFO),
      communication_(std::move(communication)), motor_id_(motor_id) {
  set_log_tag("MotorActuator-" + std::to_string(motor_id_));
}

bool MotorActuator::send_command(const std::array<uint8_t, packet_length_> &command) {
  MotorPacket packet{};
  packet.id = motor_id_;
  packet.length = packet_length_;
  packet.data = command;
  MotorPacket response{};
  return communication_ && communication_(packet, response, 0);
}

bool MotorActuator::request(uint8_t command_code, MotorPacket &response,
                            uint32_t timeout_ms) {
  std::array<uint8_t, packet_length_> command{};
  command[0] = command_code;
  MotorPacket packet{};
  packet.id = motor_id_;
  packet.length = packet_length_;
  packet.data = command;
  return request(packet, response, timeout_ms);
}

bool MotorActuator::request(const MotorPacket &command, MotorPacket &response,
                            uint32_t timeout_ms) {
  if (!communication_ || !communication_(command, response, timeout_ms)) {
    return false;
  }
  return true;
}

bool MotorActuator::read_status(Status &status, uint32_t timeout_ms) {
  MotorPacket response{};
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

bool MotorActuator::read_motor_id(uint8_t &motor_id, uint32_t timeout_ms) {
  MotorPacket command{};
  command.id = motor_id_;
  command.length = packet_length_;
  command.data[0] = 0x79;
  command.data[2] = 1;
  MotorPacket response{};
  if (!request(command, response, timeout_ms)) {
    logger_.warn("No motor ID response received");
    return false;
  }
  const uint16_t can_id = static_cast<uint16_t>(response.data[6]) |
                          (static_cast<uint16_t>(response.data[7]) << 8);
  if (can_id < 0x141 || can_id > 0x160) {
    logger_.warn("Invalid motor CAN ID in response: 0x{:x}", can_id);
    return false;
  }
  motor_id = static_cast<uint8_t>(can_id - 0x140);
  return true;
}

bool MotorActuator::write_motor_id(uint8_t motor_id, uint32_t timeout_ms) {
  if (motor_id < 1 || motor_id > 32) {
    logger_.error("Motor ID must be between 1 and 32");
    return false;
  }
  std::array<uint8_t, packet_length_> command{};
  command[0] = 0x79;
  command[2] = 0;
  command[7] = motor_id;
  MotorPacket packet{};
  packet.id = motor_id_;
  packet.length = packet_length_;
  packet.data = command;
  MotorPacket response{};
  if (!communication_ || !communication_(packet, response, timeout_ms)) {
    logger_.warn("No motor ID write response received");
    return false;
  }
  motor_id_ = motor_id;
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
  logger_.info("Sending incremental position command: delta={} deg, speed={} RPM",
               delta_degrees, max_speed_rpm);
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
