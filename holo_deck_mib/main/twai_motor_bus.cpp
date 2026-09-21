#include "twai_motor_bus.hpp"

#include <chrono>

namespace {
constexpr size_t kPacketLength = 8;
constexpr uint8_t kMultiMotorCommand = 0x79; // RMD multi-motor command byte
constexpr uint32_t kMultiMotorId = 0x300;
constexpr uint32_t kCommandBase = 0x140;
constexpr uint32_t kReplyBase = 0x240;
} // namespace

TwaiMotorBus::TwaiMotorBus(const Config &config)
    : espp::BaseComponent("TwaiMotorBus", config.log_level)
    , config_(config)
    , twai_({
          .tx_gpio = config.tx_gpio,
          .rx_gpio = config.rx_gpio,
          .baudrate = config.bitrate,
          .mode = espp::Twai::Mode::NORMAL,
          .tx_queue_depth = 4, // one frame in flight per transaction; single-shot transmit
          .on_receive = [this](const espp::Twai::Message &m) { on_receive(m); },
          .auto_start = true,
          .task_config = {.name = "motor_can_rx", .stack_size_bytes = 4096, .priority = 12},
          .log_level = config.log_level,
      }) {}

TwaiMotorBus::~TwaiMotorBus() = default;

bool TwaiMotorBus::start(std::error_code &ec) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  if (!twai_.initialize(ec)) {
    logger_.error("TWAI initialization failed: {}", ec.message());
    return false;
  }
  logger_.info("CAN bus up: TX GPIO {}, RX GPIO {}, {} bit/s", static_cast<int>(config_.tx_gpio),
               static_cast<int>(config_.rx_gpio), config_.bitrate);
  return true;
}

uint32_t TwaiMotorBus::tx_id_for(const MotorPacket &command) {
  return command.data[0] == kMultiMotorCommand ? kMultiMotorId : kCommandBase + command.id;
}

bool TwaiMotorBus::reply_matches(const MotorPacket &command, const espp::Twai::Message &m) {
  if (m.dlc != kPacketLength || m.data[0] != command.data[0])
    return false;
  if (command.data[0] == kMultiMotorCommand)
    return m.id == kMultiMotorId;
  // the RMD answers from 0x240+id; some firmware echoes on the command id
  return m.id == kReplyBase + command.id || m.id == kCommandBase + command.id;
}

bool TwaiMotorBus::transmit_locked(const MotorPacket &command) {
  if (command.id < 1 || command.id > 32 || command.length > kPacketLength) {
    logger_.error("Invalid motor packet: id={} length={}", command.id, command.length);
    return false;
  }
  espp::Twai::Message m;
  m.id = tx_id_for(command);
  m.extended = false;
  m.rtr = false;
  m.dlc = static_cast<uint8_t>(command.length);
  m.data = command.data;
  std::error_code ec;
  if (!twai_.transmit(m, ec, config_.tx_timeout_ms)) {
    tx_errors_.fetch_add(1);
    logger_.debug("transmit to 0x{:03x} failed: {}", m.id, ec.message());
    return false;
  }
  return true;
}

void TwaiMotorBus::on_receive(const espp::Twai::Message &m) {
  // Twai receive task context: hand a matching reply to the waiter, count the rest
  std::lock_guard<std::mutex> lock(reply_mutex_);
  if (waiting_ && pending_command_ && !reply_ && reply_matches(*pending_command_, m)) {
    MotorPacket packet{};
    packet.id = pending_command_->id; // logical motor id, as the actuator expects
    packet.length = m.dlc;
    packet.data = m.data;
    reply_ = packet;
    reply_cv_.notify_one();
    return;
  }
  unexpected_rx_.fetch_add(1);
}

bool TwaiMotorBus::send(const MotorPacket &command) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  return transmit_locked(command);
}

bool TwaiMotorBus::request(const MotorPacket &command, MotorPacket &response, uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  {
    std::lock_guard<std::mutex> rlock(reply_mutex_);
    pending_command_ = command;
    reply_.reset();
    waiting_ = true;
  }
  bool ok = transmit_locked(command);
  if (ok) {
    std::unique_lock<std::mutex> rlock(reply_mutex_);
    ok = reply_cv_.wait_for(rlock, std::chrono::milliseconds(timeout_ms),
                            [this] { return reply_.has_value(); });
    if (ok)
      response = *reply_;
  }
  std::lock_guard<std::mutex> rlock(reply_mutex_);
  waiting_ = false;
  pending_command_.reset();
  reply_.reset();
  return ok;
}

MotorActuator::CommunicationFunction TwaiMotorBus::communication_function() {
  return [this](const MotorPacket &command, MotorPacket &response, uint32_t timeout_ms) {
    if (timeout_ms == 0)
      return send(command);
    return request(command, response, timeout_ms);
  };
}
