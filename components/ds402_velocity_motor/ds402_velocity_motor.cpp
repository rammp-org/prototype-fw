#include "ds402_velocity_motor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "freertos/task.h"

namespace {
constexpr uint16_t kControlwordIndex = 0x6040;
constexpr uint16_t kStatuswordIndex = 0x6041;
constexpr uint16_t kModesOfOperationIndex = 0x6060;
constexpr uint16_t kModesOfOperationDisplayIndex = 0x6061;
constexpr uint16_t kMaxSpeedRpmIndex = 0x6080;
constexpr uint16_t kProfileAccelerationIndex = 0x6083;
constexpr uint16_t kProfileDecelerationIndex = 0x6084;
constexpr uint16_t kTargetVelocityIndex = 0x60FF;
// L2DB controlword commands (manual 7.2: 0x06 disable, 0x0F enable, 0x86 clear error)
constexpr uint16_t kControlwordDisable = 0x0006;
constexpr uint16_t kControlwordEnable = 0x000F;
constexpr uint16_t kControlwordClearError = 0x0086;
constexpr uint16_t kAlarmBit = 0x0008;
constexpr int8_t kSpeedModeWithRamps = 3;
constexpr int8_t kSpeedModeWithoutRamps = -3;
constexpr uint8_t kSdoDownloadU8 = 0x2F;
constexpr uint8_t kSdoDownloadU16 = 0x2B;
constexpr uint8_t kSdoDownloadU32 = 0x23;
constexpr uint8_t kSdoDownloadOk = 0x60;
constexpr uint8_t kSdoAbort = 0x80;
constexpr uint8_t kSdoUpload = 0x40;
constexpr uint32_t kDefaultSdoTimeoutMs = 100;

bool sdo_matches(const CanFrame &frame, uint32_t reply_id, uint16_t index, uint8_t subindex) {
  return frame.id == reply_id && frame.dlc == 8 && frame.data[1] == static_cast<uint8_t>(index) &&
         frame.data[2] == static_cast<uint8_t>(index >> 8) && frame.data[3] == subindex;
}

/// Sign-extend an expedited upload payload by the size its response code declares.
int32_t signed_payload(uint8_t response_command, uint32_t raw) {
  switch (response_command) {
  case 0x4F:
    return static_cast<int8_t>(raw);
  case 0x4B:
    return static_cast<int16_t>(raw);
  default:
    return static_cast<int32_t>(raw);
  }
}
} // namespace

Ds402VelocityMotor::Ds402VelocityMotor(CanBus &bus, uint8_t node_id, const Config &config)
    : espp::BaseComponent("Ds402VelocityMotor", config.log_level)
    , bus_(bus)
    , node_id_(node_id)
    , config_(config) {
  set_log_tag("Ds402Motor-" + std::to_string(node_id_));
  receive_queue_ = xQueueCreate(16, sizeof(CanFrame));
  if (receive_queue_ == nullptr || !bus_.register_receiver(0x580 + node_id_, receive_queue_) ||
      !bus_.register_receiver(0x700 + node_id_, receive_queue_)) {
    logger_.error("Failed to register CANopen receivers for node {}", node_id_);
  }
}

Ds402VelocityMotor::~Ds402VelocityMotor() {
  if (receive_queue_ != nullptr) {
    bus_.unregister_receiver(receive_queue_); // the RX ISR must not see a deleted queue
    vQueueDelete(receive_queue_);
  }
}

const char *Ds402VelocityMotor::state_name(uint16_t statusword) {
  // The L2DB statusword is not the DS402 state word: 0x0000 = normal,
  // 0x0008 = alarm (its PDO form adds bit0 ready, bits1-2 enabled, bit5 e-stop).
  if (statusword & kAlarmBit) {
    return "Alarm";
  }
  if (statusword & 0x0020) {
    return "Emergency stop";
  }
  if (statusword & 0x0006) {
    return "Enabled";
  }
  return "Normal";
}

// ---------------------------------------------------------------------------
// Raw / SDO plumbing (transaction_mutex_ held by the callers)
// ---------------------------------------------------------------------------

bool Ds402VelocityMotor::send_raw(uint32_t can_id, std::span<const uint8_t> data,
                                  bool wait_for_queue_space) {
  if (can_id > 0x7FF || data.size() > 8) {
    logger_.error("Invalid CAN frame: id=0x{:x} length={}", can_id, data.size());
    return false;
  }
  CanFrame frame{};
  frame.id = can_id;
  frame.dlc = static_cast<uint8_t>(data.size());
  std::copy(data.begin(), data.end(), frame.data.begin());
  return bus_.send(frame, wait_for_queue_space);
}

bool Ds402VelocityMotor::send_sdo_download_unconfirmed(uint16_t index, uint8_t subindex,
                                                       uint8_t command, uint32_t value) {
  const std::array<uint8_t, 8> data{command,
                                    static_cast<uint8_t>(index),
                                    static_cast<uint8_t>(index >> 8),
                                    subindex,
                                    static_cast<uint8_t>(value),
                                    static_cast<uint8_t>(value >> 8),
                                    static_cast<uint8_t>(value >> 16),
                                    static_cast<uint8_t>(value >> 24)};
  // A brief enqueue wait (no wire wait): a set-point must not be silently
  // dropped under transient queue pressure, but the loop must never block on
  // the drive's reply.
  return send_raw(0x600 + node_id_, data, /*wait_for_queue_space=*/true);
}

bool Ds402VelocityMotor::send_sdo_download(uint16_t index, uint8_t subindex, uint8_t command,
                                           uint32_t value, uint32_t timeout_ms) {
  if (receive_queue_ == nullptr) {
    logger_.error("Cannot write SDO: CAN receive queue is not started");
    return false;
  }
  CanFrame response{};
  while (xQueueReceive(receive_queue_, &response, 0) == pdTRUE) {
  }
  if (!send_sdo_download_unconfirmed(index, subindex, command, value)) {
    return false;
  }
  const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
  const TickType_t start_tick = xTaskGetTickCount();
  while (xTaskGetTickCount() - start_tick < timeout_ticks) {
    const TickType_t elapsed = xTaskGetTickCount() - start_tick;
    if (xQueueReceive(receive_queue_, &response, timeout_ticks - elapsed) != pdTRUE) {
      break;
    }
    if (!sdo_matches(response, 0x580 + node_id_, index, subindex)) {
      continue; // a late confirmation of an earlier unconfirmed write, or a boot-up
    }
    if (response.data[0] == kSdoAbort) {
      logger_.error("SDO write 0x{:04X}:{:02X} aborted: 0x{:02X}{:02X}{:02X}{:02X}", index,
                    subindex, response.data[7], response.data[6], response.data[5],
                    response.data[4]);
      return false;
    }
    if (response.data[0] != kSdoDownloadOk) {
      logger_.error("SDO write 0x{:04X}:{:02X} returned unsupported response 0x{:02X}", index,
                    subindex, response.data[0]);
      return false;
    }
    return true;
  }
  logger_.error("Timed out writing SDO 0x{:04X}:{:02X}", index, subindex);
  return false;
}

bool Ds402VelocityMotor::request_sdo_u32(uint16_t index, uint8_t subindex, uint32_t &value,
                                         uint32_t timeout_ms) {
  if (receive_queue_ == nullptr) {
    logger_.error("Cannot read SDO: CAN receive queue is not started");
    return false;
  }
  CanFrame response{};
  while (xQueueReceive(receive_queue_, &response, 0) == pdTRUE) {
  }
  const std::array<uint8_t, 8> request{kSdoUpload,
                                       static_cast<uint8_t>(index),
                                       static_cast<uint8_t>(index >> 8),
                                       subindex,
                                       0x00,
                                       0x00,
                                       0x00,
                                       0x00};
  if (!send_raw(0x600 + node_id_, request, /*wait_for_queue_space=*/true)) {
    return false;
  }
  const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
  const TickType_t start_tick = xTaskGetTickCount();
  while (xTaskGetTickCount() - start_tick < timeout_ticks) {
    const TickType_t elapsed = xTaskGetTickCount() - start_tick;
    if (xQueueReceive(receive_queue_, &response, timeout_ticks - elapsed) != pdTRUE) {
      break;
    }
    if (!sdo_matches(response, 0x580 + node_id_, index, subindex)) {
      continue;
    }
    if (response.data[0] == kSdoAbort) {
      logger_.error("SDO read 0x{:04X}:{:02X} aborted: 0x{:02X}{:02X}{:02X}{:02X}", index, subindex,
                    response.data[7], response.data[6], response.data[5], response.data[4]);
      return false;
    }
    // expedited upload responses: 0x4F (1 byte), 0x4B (2), 0x47 (3), 0x43 (4);
    // the payload is sign-extended by the caller from the declared size, so
    // return it with the response command in the top bits cleared
    if ((response.data[0] & 0xF3) != 0x43) {
      logger_.error("SDO read 0x{:04X}:{:02X} returned unsupported response 0x{:02X}", index,
                    subindex, response.data[0]);
      return false;
    }
    const uint32_t raw = static_cast<uint32_t>(response.data[4]) |
                         static_cast<uint32_t>(response.data[5]) << 8 |
                         static_cast<uint32_t>(response.data[6]) << 16 |
                         static_cast<uint32_t>(response.data[7]) << 24;
    value = static_cast<uint32_t>(signed_payload(response.data[0], raw));
    return true;
  }
  logger_.warn("Timed out reading SDO 0x{:04X}:{:02X}", index, subindex);
  return false;
}

bool Ds402VelocityMotor::read_signed(const ObjectSpec &object, float &value, uint32_t timeout_ms) {
  if (object.index == 0) {
    return false;
  }
  uint32_t raw = 0;
  if (!request_sdo_u32(object.index, object.subindex, raw, timeout_ms)) {
    return false;
  }
  value = static_cast<float>(static_cast<int32_t>(raw)) * object.scale;
  return true;
}

bool Ds402VelocityMotor::reset_communication_and_wait_for_bootup(uint32_t timeout_ms) {
  CanFrame frame{};
  while (xQueueReceive(receive_queue_, &frame, 0) == pdTRUE) {
  }
  const std::array<uint8_t, 2> nmt_reset_communication{0x82, node_id_};
  if (!send_raw(0x000, nmt_reset_communication, true)) {
    return false;
  }
  const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
  const TickType_t start_tick = xTaskGetTickCount();
  while (xTaskGetTickCount() - start_tick < timeout_ticks) {
    const TickType_t elapsed = xTaskGetTickCount() - start_tick;
    if (xQueueReceive(receive_queue_, &frame, timeout_ticks - elapsed) != pdTRUE) {
      break;
    }
    if (frame.id == 0x700 + node_id_ && frame.dlc >= 1 && frame.data[0] == 0x00) {
      logger_.info("CANopen node {} booted after communication reset", node_id_);
      return true;
    }
  }
  logger_.error("Timed out waiting for CANopen node {} boot-up", node_id_);
  return false;
}

// ---------------------------------------------------------------------------
// Enable / disable / faults
// ---------------------------------------------------------------------------

bool Ds402VelocityMotor::wait_for_alarm_clear(uint32_t timeout_ms, uint16_t &statusword) {
  const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
  const TickType_t start_tick = xTaskGetTickCount();
  do {
    uint32_t raw = 0;
    if (!request_sdo_u32(kStatuswordIndex, 0, raw, kDefaultSdoTimeoutMs)) {
      return false;
    }
    statusword = static_cast<uint16_t>(raw);
    if ((statusword & kAlarmBit) == 0) {
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  } while (xTaskGetTickCount() - start_tick < timeout_ticks);
  return false;
}

bool Ds402VelocityMotor::enable_drive(uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  if (config_.reset_communication_on_enable &&
      !reset_communication_and_wait_for_bootup(timeout_ms)) {
    return false;
  }
  // NMT start: harmless when the node is already operational, needed for PDOs
  const std::array<uint8_t, 2> nmt_start{0x01, node_id_};
  if (!send_raw(0x000, nmt_start, true)) {
    return false;
  }
  const int8_t mode = config_.use_acc_dec ? kSpeedModeWithRamps : kSpeedModeWithoutRamps;
  if (!send_sdo_download(kModesOfOperationIndex, 0, kSdoDownloadU8, static_cast<uint8_t>(mode),
                         kDefaultSdoTimeoutMs)) {
    return false;
  }
  uint32_t raw = 0;
  if (request_sdo_u32(kModesOfOperationDisplayIndex, 0, raw, kDefaultSdoTimeoutMs) &&
      static_cast<int8_t>(raw) != mode) {
    logger_.warn("Drive reports operation mode {} after requesting {}",
                 static_cast<int>(static_cast<int8_t>(raw)), static_cast<int>(mode));
  }
  if (config_.use_acc_dec) {
    const auto ramp = [&](float rps2) {
      return static_cast<uint32_t>(std::lround(rps2 * config_.acceleration_counts_per_rps2));
    };
    if (config_.acceleration_rps2 > 0.0f &&
        !send_sdo_download(kProfileAccelerationIndex, 0, kSdoDownloadU32,
                           ramp(config_.acceleration_rps2), kDefaultSdoTimeoutMs)) {
      return false;
    }
    if (config_.deceleration_rps2 > 0.0f &&
        !send_sdo_download(kProfileDecelerationIndex, 0, kSdoDownloadU32,
                           ramp(config_.deceleration_rps2), kDefaultSdoTimeoutMs)) {
      return false;
    }
  }
  if (config_.max_speed_rpm != 0 &&
      !send_sdo_download(kMaxSpeedRpmIndex, 0, kSdoDownloadU16, config_.max_speed_rpm,
                         kDefaultSdoTimeoutMs)) {
    return false;
  }
  // a zero set-point before the power stage comes on, so nothing lurches
  if (!write_target_velocity(0, /*confirmed=*/true, kDefaultSdoTimeoutMs)) {
    return false;
  }
  if (!send_sdo_download(kControlwordIndex, 0, kSdoDownloadU16, kControlwordEnable,
                         kDefaultSdoTimeoutMs)) {
    return false;
  }
  uint16_t statusword = 0;
  if (!wait_for_alarm_clear(timeout_ms, statusword)) {
    logger_.error("Drive reports an alarm after enable (statusword 0x{:04X})", statusword);
    return false;
  }
  logger_.info("Drive enabled in speed mode {} (statusword 0x{:04X}: {})", static_cast<int>(mode),
               statusword, state_name(statusword));
  return true;
}

bool Ds402VelocityMotor::disable_drive(uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  return send_sdo_download(kControlwordIndex, 0, kSdoDownloadU16, kControlwordDisable,
                           std::max<uint32_t>(timeout_ms, kDefaultSdoTimeoutMs));
}

bool Ds402VelocityMotor::reset_fault(uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  if (!send_sdo_download(kControlwordIndex, 0, kSdoDownloadU16, kControlwordClearError,
                         kDefaultSdoTimeoutMs)) {
    return false;
  }
  uint16_t statusword = 0;
  if (!wait_for_alarm_clear(timeout_ms, statusword)) {
    logger_.error("Drive alarm did not clear (statusword 0x{:04X})", statusword);
    return false;
  }
  logger_.info("Drive alarm cleared");
  return true;
}

// ---------------------------------------------------------------------------
// Velocity set-points
// ---------------------------------------------------------------------------

int32_t Ds402VelocityMotor::rpm_to_counts(float rpm) const {
  const float counts = rpm * config_.velocity_counts_per_rpm;
  constexpr float kLimit = static_cast<float>(std::numeric_limits<int32_t>::max());
  return static_cast<int32_t>(std::lround(std::clamp(counts, -kLimit, kLimit)));
}

bool Ds402VelocityMotor::write_target_velocity(int32_t counts, bool confirmed,
                                               uint32_t timeout_ms) {
  const uint32_t value = static_cast<uint32_t>(counts);
  if (confirmed) {
    return send_sdo_download(kTargetVelocityIndex, 0, kSdoDownloadU32, value, timeout_ms);
  }
  return send_sdo_download_unconfirmed(kTargetVelocityIndex, 0, kSdoDownloadU32, value);
}

bool Ds402VelocityMotor::send_velocity_rpm(float rpm) {
  if (!std::isfinite(rpm)) {
    logger_.error("Rejecting a non-finite velocity set-point");
    return false;
  }
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  return write_target_velocity(rpm_to_counts(rpm), /*confirmed=*/false, 0);
}

bool Ds402VelocityMotor::stop() {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  return write_target_velocity(0, /*confirmed=*/false, 0);
}

bool Ds402VelocityMotor::stop_acknowledged(uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  return write_target_velocity(0, /*confirmed=*/true, timeout_ms);
}

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------

bool Ds402VelocityMotor::read_status(Status &status, uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  uint32_t raw = 0;
  if (!request_sdo_u32(kStatuswordIndex, 0, raw, timeout_ms)) {
    return false;
  }
  status.statusword = static_cast<uint16_t>(raw);
  status.faulted = (status.statusword & kAlarmBit) != 0;
  float value = 0.0f;
  if (read_signed(config_.velocity_actual, value, timeout_ms)) {
    status.velocity_rpm = value;
  }
  if (config_.position_counts_per_rev > 0.0f &&
      read_signed(config_.position_actual, value, timeout_ms)) {
    status.position_degrees = value / config_.position_counts_per_rev * 360.0f;
  }
  if (read_signed(config_.current_actual, value, timeout_ms)) {
    status.current_a = value;
  }
  if (read_signed(config_.temperature, value, timeout_ms)) {
    status.temperature_c = value;
  }
  if (status.faulted && config_.error_code.index != 0 &&
      request_sdo_u32(config_.error_code.index, config_.error_code.subindex, raw, timeout_ms)) {
    status.error_code = static_cast<uint16_t>(raw);
  } else if (!status.faulted) {
    status.error_code = 0;
  }
  return true;
}
