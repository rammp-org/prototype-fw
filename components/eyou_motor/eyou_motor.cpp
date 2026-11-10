#include "eyou_motor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "freertos/task.h"

#include "esp_log.h"
#include "nvs.hpp"

namespace {
constexpr char kTag[] = "EyouMotor";
constexpr char kNvsNamespace[] = "eyou_motor";
constexpr uint16_t kControlwordIndex = 0x6040;
constexpr uint16_t kStatuswordIndex = 0x6041;
constexpr uint16_t kErrorCodeIndex = 0x603F;
constexpr uint16_t kModesOfOperationIndex = 0x6060;
constexpr uint16_t kModesOfOperationDisplayIndex = 0x6061;
constexpr uint16_t kPositionActualIndex = 0x6064;
constexpr uint16_t kTargetPositionIndex = 0x607A;
constexpr uint16_t kProfileVelocityIndex = 0x6081;
constexpr uint16_t kProfileAccelerationIndex = 0x6083;
constexpr uint16_t kProfileDecelerationIndex = 0x6084;
constexpr uint16_t kVelocityActualIndex = 0x606C;
constexpr uint16_t kMaxTorqueIndex = 0x6072;
constexpr uint16_t kRatedCurrentIndex = 0x6075;
constexpr uint16_t kRatedTorqueIndex = 0x6076;
constexpr uint16_t kTorqueActualIndex = 0x6077;
constexpr uint16_t kCurrentActualIndex = 0x6078;
constexpr uint16_t kControlwordHalt = 0x0100;
constexpr uint16_t kControlwordShutdown = 0x0006;
constexpr uint16_t kControlwordSwitchOn = 0x0007;
constexpr uint16_t kControlwordEnableOperation = 0x000F;
constexpr uint16_t kControlwordFaultReset = 0x0080;
constexpr uint16_t kControlwordNewSetpoint = 0x0010;
constexpr uint16_t kControlwordImmediate = 0x0020;
constexpr uint16_t kControlwordRelative = 0x0040;
constexpr uint16_t kStoreParametersIndex = 0x1010;
constexpr uint32_t kStoreParametersSignature = 0x65766173; // ASCII "save", little-endian
constexpr uint16_t kNodeIdIndex = 0x26A0;                  // vendor: the drive's own node id (USINT)
} // namespace

EyouMotor::EyouMotor(CanBus &bus, uint8_t node_id)
    : espp::BaseComponent("EyouMotor", espp::Logger::Verbosity::INFO)
    , bus_(bus)
    , node_id_(node_id) {
  receive_queue_ = xQueueCreate(16, sizeof(EyouCanFrame));
  if (receive_queue_ == nullptr || !bus_.register_receiver(0x580 + node_id_, receive_queue_) ||
      !bus_.register_receiver(0x700 + node_id_, receive_queue_)) {
    logger_.error("Failed to register CANopen receivers for node {}", node_id_);
  }
}

EyouMotor::~EyouMotor() {
  if (receive_queue_ != nullptr) {
    bus_.unregister_receiver(receive_queue_); // the RX ISR must not see a deleted queue
    vQueueDelete(receive_queue_);
  }
}

bool EyouMotor::send_sdo_u8(uint16_t index, uint8_t value, uint8_t subindex, uint32_t timeout_ms) {
  return send_sdo_download(index, subindex, 0x2F, value, timeout_ms);
}

bool EyouMotor::send_sdo_u16(uint16_t index, uint16_t value, uint8_t subindex,
                             uint32_t timeout_ms) {
  return send_sdo_download(index, subindex, 0x2B, value, timeout_ms);
}

bool EyouMotor::send_sdo_u32(uint16_t index, uint32_t value, uint8_t subindex,
                             uint32_t timeout_ms) {
  return send_sdo_download(index, subindex, 0x23, value, timeout_ms);
}

bool EyouMotor::send_sdo_download(uint16_t index, uint8_t subindex, uint8_t command, uint32_t value,
                                  uint32_t timeout_ms) {
  if (receive_queue_ == nullptr) {
    logger_.error("Cannot write SDO: CAN receive queue is not started");
    return false;
  }

  EyouCanFrame response{};
  while (xQueueReceive(receive_queue_, &response, 0) == pdTRUE) {
  }
  const std::array<uint8_t, 8> data{command,
                                    static_cast<uint8_t>(index),
                                    static_cast<uint8_t>(index >> 8),
                                    subindex,
                                    static_cast<uint8_t>(value),
                                    static_cast<uint8_t>(value >> 8),
                                    static_cast<uint8_t>(value >> 16),
                                    static_cast<uint8_t>(value >> 24)};
  if (!send_raw(0x600 + node_id_, data)) {
    return false;
  }
  // The queue also carries the node's boot-up / heartbeat frames (0x700 + id)
  // and a late reply of an earlier transaction: wait for THIS write's reply
  // within one overall deadline, skipping anything else (as the upload path does).
  const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
  const TickType_t start_tick = xTaskGetTickCount();
  bool matched = false;
  while (xTaskGetTickCount() - start_tick < timeout_ticks) {
    const TickType_t elapsed = xTaskGetTickCount() - start_tick;
    if (xQueueReceive(receive_queue_, &response, timeout_ticks - elapsed) != pdTRUE) {
      break;
    }
    if (response.id == 0x580 + node_id_ && response.dlc == 8 &&
        response.data[1] == static_cast<uint8_t>(index) &&
        response.data[2] == static_cast<uint8_t>(index >> 8) && response.data[3] == subindex) {
      matched = true;
      break;
    }
  }
  if (!matched) {
    logger_.error("Timed out writing SDO 0x{:04X}:{:02X}", index, subindex);
    return false;
  }
  if (response.data[0] == 0x80) {
    logger_.error(
        "SDO write 0x{:04X}:{:02X} aborted: 0x{:02X}{:02X}{:02X}{:02X}; response={:02X} {:02X} "
        "{:02X} {:02X} {:02X} {:02X} {:02X} {:02X}",
        index, subindex, response.data[7], response.data[6], response.data[5], response.data[4],
        response.data[0], response.data[1], response.data[2], response.data[3], response.data[4],
        response.data[5], response.data[6], response.data[7]);
    return false;
  }
  if (response.data[0] != 0x60) {
    logger_.error("SDO write 0x{:04X}:{:02X} returned unsupported response 0x{:02X}", index,
                  subindex, response.data[0]);
    return false;
  }
  return true;
}

bool EyouMotor::reset_communication_and_wait_for_bootup(uint32_t timeout_ms) {
  if (receive_queue_ == nullptr) {
    logger_.error("Cannot reset CANopen communication: CAN receive queue is not started");
    return false;
  }

  EyouCanFrame frame{};
  while (xQueueReceive(receive_queue_, &frame, 0) == pdTRUE) {
  }
  const std::array<uint8_t, 2> nmt_stop{0x02, node_id_};
  const std::array<uint8_t, 2> nmt_reset_communication{0x82, node_id_};
  if (!send_raw(0x000, nmt_stop) || !send_raw(0x000, nmt_reset_communication)) {
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

bool EyouMotor::wait_for_operation_enabled(uint32_t timeout_ms) {
  return wait_for_ds402_state(0x0027, timeout_ms);
}

bool EyouMotor::wait_for_ds402_state(uint16_t expected_state, uint32_t timeout_ms) {
  const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
  const TickType_t start_tick = xTaskGetTickCount();
  uint16_t statusword = 0;
  while (xTaskGetTickCount() - start_tick < timeout_ticks) {
    uint32_t raw_statusword = 0;
    if (!request_sdo_u32(kStatuswordIndex, 0x4B, raw_statusword, 100)) {
      return false;
    }
    statusword = static_cast<uint16_t>(raw_statusword);
    if ((statusword & 0x006F) == expected_state) {
      logger_.info("Drive reached DS402 state 0x{:04X} (statusword=0x{:04X})", expected_state,
                   statusword);
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  logger_.error("Timed out waiting for DS402 state 0x{:04X} (last statusword=0x{:04X})",
                expected_state, statusword);
  return false;
}

bool EyouMotor::ensure_operation_enabled(uint32_t timeout_ms) {
  uint32_t raw_statusword = 0;
  if (!request_sdo_u32(kStatuswordIndex, 0x4B, raw_statusword, timeout_ms)) {
    return false;
  }
  const uint16_t state = static_cast<uint16_t>(raw_statusword) & 0x006F;
  if (state == 0x0027) {
    return true;
  }

  if (state == 0x0023) {
    logger_.info("Drive is Switched On; enabling operation before position command");
    return send_sdo_u16(kControlwordIndex, kControlwordEnableOperation) &&
           wait_for_operation_enabled(timeout_ms);
  }
  if (state == 0x0021) {
    logger_.info("Drive is Ready to Switch On; completing DS402 enable sequence");
    return send_sdo_u16(kControlwordIndex, kControlwordSwitchOn) &&
           wait_for_ds402_state(0x0023, timeout_ms) &&
           send_sdo_u16(kControlwordIndex, kControlwordEnableOperation) &&
           wait_for_operation_enabled(timeout_ms);
  }

  logger_.warn("Drive state 0x{:04X}; applying DS402 enable sequence", state);
  return send_sdo_u16(kControlwordIndex, kControlwordShutdown) &&
         wait_for_ds402_state(0x0021, timeout_ms) &&
         send_sdo_u16(kControlwordIndex, kControlwordSwitchOn) &&
         wait_for_ds402_state(0x0023, timeout_ms) &&
         send_sdo_u16(kControlwordIndex, kControlwordEnableOperation) &&
         wait_for_operation_enabled(timeout_ms);
}

bool EyouMotor::ensure_profile_position_mode(uint32_t timeout_ms) {
  uint32_t raw_mode = 0;
  if (!request_sdo_u32(kModesOfOperationDisplayIndex, 0x4F, raw_mode, 100)) {
    logger_.error("Failed to read modes-of-operation display");
    return false;
  }
  if (static_cast<int8_t>(raw_mode) == 1) {
    return true;
  }

  logger_.info("Drive is in mode {}, not Profile Position; requesting mode 1",
               static_cast<int>(static_cast<int8_t>(raw_mode)));
  if (!send_sdo_u8(kModesOfOperationIndex, 1)) {
    return false;
  }

  const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
  const TickType_t start_tick = xTaskGetTickCount();
  while (xTaskGetTickCount() - start_tick < timeout_ticks) {
    if (!request_sdo_u32(kModesOfOperationDisplayIndex, 0x4F, raw_mode, 100)) {
      return false;
    }
    if (static_cast<int8_t>(raw_mode) == 1) {
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  logger_.error("Timed out switching to Profile Position mode (last mode={})",
                static_cast<int>(static_cast<int8_t>(raw_mode)));
  return false;
}

const char *EyouMotor::ds402_state_name(uint16_t statusword) {
  switch (statusword & 0x006F) {
  case 0x0000:
    return "Not ready to switch on";
  case 0x0040:
    return "Switch on disabled";
  case 0x0021:
    return "Ready to switch on";
  case 0x0023:
    return "Switched on";
  case 0x0027:
    return "Operation enabled";
  case 0x0007:
    return "Quick stop active";
  case 0x000F:
    return "Fault reaction active";
  case 0x0008:
    return "Fault";
  default:
    return "Unknown";
  }
}

bool EyouMotor::enable_drive(uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  return ensure_operation_enabled(timeout_ms);
}

bool EyouMotor::disable_drive(uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  return send_sdo_u16(kControlwordIndex, kControlwordShutdown) &&
         wait_for_ds402_state(0x0021, timeout_ms);
}

bool EyouMotor::reset_fault(uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  if (!send_sdo_u16(kControlwordIndex, 0x0000) ||
      !send_sdo_u16(kControlwordIndex, kControlwordFaultReset)) {
    return false;
  }

  const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
  const TickType_t start_tick = xTaskGetTickCount();
  while (xTaskGetTickCount() - start_tick < timeout_ticks) {
    uint32_t raw_statusword = 0;
    if (!request_sdo_u32(kStatuswordIndex, 0x4B, raw_statusword, 100)) {
      return false;
    }
    const uint16_t statusword = static_cast<uint16_t>(raw_statusword);
    if ((statusword & 0x0008) == 0) {
      logger_.info("Drive fault cleared (statusword=0x{:04X})", statusword);
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  logger_.error("Timed out clearing drive fault");
  return false;
}

bool EyouMotor::configure_profile_position(const ProfilePositionConfig &config) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  uint32_t velocity_pulses_per_second = 0;
  uint32_t acceleration_pulses_per_second_squared = 0;
  uint32_t deceleration_pulses_per_second_squared = 0;
  if (!degrees_to_profile_units(config.velocity_degrees_per_second, velocity_pulses_per_second) ||
      !degrees_to_profile_units(config.acceleration_degrees_per_second_squared,
                                acceleration_pulses_per_second_squared) ||
      !degrees_to_profile_units(config.deceleration_degrees_per_second_squared,
                                deceleration_pulses_per_second_squared)) {
    return false;
  }
  if (!reset_communication_and_wait_for_bootup(2000)) {
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(100));
  uint32_t raw_position = 0;
  if (!request_sdo_u32(kPositionActualIndex, 0x43, raw_position, 100)) {
    logger_.error("Failed to read actual position before entering profile-position mode");
    return false;
  }

  const std::array<uint8_t, 2> nmt_start{0x01, node_id_};
  if (!send_raw(0x000, nmt_start) || !ensure_operation_enabled(1000) ||
      !ensure_profile_position_mode(1000)) {
    return false;
  }

  return send_sdo_u32(kTargetPositionIndex, raw_position) &&
         send_sdo_u32(kProfileVelocityIndex, velocity_pulses_per_second) &&
         send_sdo_u32(kProfileAccelerationIndex, acceleration_pulses_per_second_squared) &&
         send_sdo_u32(kProfileDecelerationIndex, deceleration_pulses_per_second_squared);
}

bool EyouMotor::get_profile_position_config(ProfilePositionConfig &config, uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  uint32_t velocity_pulses_per_second = 0;
  uint32_t acceleration_pulses_per_second_squared = 0;
  uint32_t deceleration_pulses_per_second_squared = 0;
  if (!request_sdo_u32(kProfileVelocityIndex, 0x43, velocity_pulses_per_second, timeout_ms) ||
      !request_sdo_u32(kProfileAccelerationIndex, 0x43, acceleration_pulses_per_second_squared,
                       timeout_ms) ||
      !request_sdo_u32(kProfileDecelerationIndex, 0x43, deceleration_pulses_per_second_squared,
                       timeout_ms)) {
    return false;
  }
  config.velocity_degrees_per_second = profile_units_to_degrees(velocity_pulses_per_second);
  config.acceleration_degrees_per_second_squared =
      profile_units_to_degrees(acceleration_pulses_per_second_squared);
  config.deceleration_degrees_per_second_squared =
      profile_units_to_degrees(deceleration_pulses_per_second_squared);
  return true;
}

bool EyouMotor::trigger_profile_position(bool relative, bool immediate) {
  uint16_t controlword = kControlwordEnableOperation;
  if (immediate) {
    controlword |= kControlwordImmediate;
  }
  if (relative) {
    controlword |= kControlwordRelative;
  }
  if (!send_sdo_u16(kControlwordIndex, controlword) ||
      !send_sdo_u16(kControlwordIndex, controlword | kControlwordNewSetpoint)) {
    return false;
  }

  for (int attempt = 0; attempt < 10; ++attempt) {
    uint32_t raw_statusword = 0;
    if (!request_sdo_u32(kStatuswordIndex, 0x4B, raw_statusword, 100)) {
      return false;
    }
    if ((raw_statusword & 0x1000) != 0) {
      return send_sdo_u16(kControlwordIndex, controlword);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  logger_.error("Profile-position setpoint was not acknowledged");
  return false;
}

bool EyouMotor::degrees_to_pulses(float degrees, int32_t &pulses) const {
  const double raw_pulses =
      static_cast<double>(degrees) * kPulsesPerOutputTurn / kDegreesPerOutputTurn;
  if (!std::isfinite(raw_pulses) || raw_pulses < std::numeric_limits<int32_t>::min() ||
      raw_pulses > std::numeric_limits<int32_t>::max()) {
    logger_.error("Position {} degrees is outside the signed 32-bit encoder range", degrees);
    return false;
  }
  pulses = static_cast<int32_t>(std::lround(raw_pulses));
  return true;
}

bool EyouMotor::degrees_to_profile_units(float degrees, uint32_t &pulses) const {
  const double raw_pulses =
      static_cast<double>(degrees) * kPulsesPerOutputTurn / kDegreesPerOutputTurn;
  if (!std::isfinite(raw_pulses) || raw_pulses < 0.0 ||
      raw_pulses > std::numeric_limits<uint32_t>::max()) {
    logger_.error("Profile quantity {} degrees is outside the unsigned 32-bit pulse range",
                  degrees);
    return false;
  }
  pulses = static_cast<uint32_t>(std::llround(raw_pulses));
  return true;
}

float EyouMotor::pulses_to_degrees(int32_t pulses) {
  return static_cast<float>(pulses) * kDegreesPerOutputTurn / kPulsesPerOutputTurn;
}

float EyouMotor::profile_units_to_degrees(uint32_t pulses) {
  // 0x6081 / 0x6083 / 0x6084 are unsigned: converted as such, so a readback in
  // the upper half of the range is not reported negative
  return static_cast<float>(pulses) * kDegreesPerOutputTurn / kPulsesPerOutputTurn;
}

bool EyouMotor::zero(uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  uint32_t raw_position = 0;
  if (!request_sdo_u32(kPositionActualIndex, 0x43, raw_position, timeout_ms)) {
    logger_.error("Failed to read actual position while setting software zero");
    return false;
  }
  zero_offset_pulses_ = static_cast<int32_t>(raw_position);
  logger_.info("Software zero position set (actual position {} pulses)", zero_offset_pulses_);
  return true;
}

bool EyouMotor::save_zero() {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  if (!zero_offset_pulses_.has_value()) {
    logger_.warn("No software zero set; nothing to save for node {}", node_id_);
    return false;
  }
  std::error_code ec;
  espp::Nvs nvs;
  nvs.init(ec);
  if (ec) {
    logger_.error("Failed to init NVS: {}", ec.message());
    return false;
  }
  nvs.set_var(kNvsNamespace, zero_nvs_key(), *zero_offset_pulses_, ec);
  if (ec) {
    logger_.error("Failed to save software zero to NVS: {}", ec.message());
    return false;
  }
  logger_.info("Saved software zero ({} pulses) to NVS for node {}", *zero_offset_pulses_,
               node_id_);
  return true;
}

bool EyouMotor::read_saved_zero(int32_t &zero_offset_pulses) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  std::error_code ec;
  espp::Nvs nvs;
  nvs.init(ec);
  if (ec) {
    logger_.error("Failed to init NVS: {}", ec.message());
    return false;
  }
  int32_t saved_zero_offset_pulses = 0;
  nvs.get_var(kNvsNamespace, zero_nvs_key(), saved_zero_offset_pulses, ec);
  if (ec) {
    logger_.warn("No saved software zero found in NVS for node {}: {}", node_id_, ec.message());
    return false;
  }
  zero_offset_pulses = saved_zero_offset_pulses;
  return true;
}

bool EyouMotor::load_zero() {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  std::error_code ec;
  espp::Nvs nvs;
  nvs.init(ec);
  if (ec) {
    logger_.error("Failed to init NVS: {}", ec.message());
    return false;
  }
  int32_t zero_offset_pulses = 0;
  nvs.get_var(kNvsNamespace, zero_nvs_key(), zero_offset_pulses, ec);
  if (ec) {
    logger_.warn("No saved software zero found in NVS for node {}: {}", node_id_, ec.message());
    return false;
  }
  zero_offset_pulses_ = zero_offset_pulses;
  logger_.info("Loaded software zero ({} pulses) from NVS for node {}", zero_offset_pulses,
               node_id_);
  return true;
}

std::string EyouMotor::zero_nvs_key() const { return "zero_" + std::to_string(node_id_); }

bool EyouMotor::set_position_limits(float minimum_degrees, float maximum_degrees) {
  if (!std::isfinite(minimum_degrees) || !std::isfinite(maximum_degrees)) {
    // a NaN would pass the ordering check and silently disable the clamp
    logger_.error("Position limits must be finite (got {} / {})", minimum_degrees, maximum_degrees);
    return false;
  }
  if (minimum_degrees > maximum_degrees) {
    logger_.error("Position minimum {} exceeds maximum {}", minimum_degrees, maximum_degrees);
    return false;
  }
  minimum_position_degrees_ = minimum_degrees;
  maximum_position_degrees_ = maximum_degrees;
  logger_.info("Position limits set to [{}, {}] degrees", minimum_degrees, maximum_degrees);
  return true;
}

bool EyouMotor::move_absolute(float target_degrees, bool immediate) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  const float limited_target_degrees =
      std::clamp(target_degrees, minimum_position_degrees_, maximum_position_degrees_);
  if (limited_target_degrees != target_degrees) {
    logger_.warn("Position {} degrees clamped to {} degrees", target_degrees,
                 limited_target_degrees);
  }
  int32_t target_pulses = 0;
  if (!degrees_to_pulses(limited_target_degrees, target_pulses)) {
    return false;
  }
  const int64_t absolute_pulses =
      static_cast<int64_t>(target_pulses) + zero_offset_pulses_.value_or(0);
  if (absolute_pulses < std::numeric_limits<int32_t>::min() ||
      absolute_pulses > std::numeric_limits<int32_t>::max()) {
    logger_.error("Position {} degrees is outside the signed 32-bit encoder range relative to "
                  "the software zero",
                  target_degrees);
    return false;
  }

  constexpr int kMaxAttempts = 3;
  for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
    if (is_faulted_unlocked(100)) {
      logger_.error("Drive is faulted; aborting move_absolute");
      return false;
    }
    if (ensure_operation_enabled(1000) && ensure_profile_position_mode(1000) &&
        send_sdo_u32(kTargetPositionIndex,
                     static_cast<uint32_t>(static_cast<int32_t>(absolute_pulses))) &&
        trigger_profile_position(false, immediate)) {
      return true;
    }
    logger_.warn("move_absolute attempt {}/{} failed", attempt, kMaxAttempts);
  }
  return false;
}

bool EyouMotor::move_incremental(float increment_degrees, bool immediate) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  int32_t increment_pulses = 0;
  if (!degrees_to_pulses(increment_degrees, increment_pulses)) {
    return false;
  }
  if (is_faulted_unlocked(100)) {
    logger_.error("Drive is faulted; aborting move_incremental");
    return false;
  }
  return ensure_operation_enabled(1000) && ensure_profile_position_mode(1000) &&
         send_sdo_u32(kTargetPositionIndex, static_cast<uint32_t>(increment_pulses)) &&
         trigger_profile_position(true, immediate);
}

bool EyouMotor::request_sdo_u32(uint16_t index, uint8_t expected_response_command, uint32_t &value,
                                uint32_t timeout_ms) {
  if (receive_queue_ == nullptr) {
    logger_.error("Cannot read SDO: CAN receive queue is not started");
    return false;
  }

  EyouCanFrame response{};
  while (xQueueReceive(receive_queue_, &response, 0) == pdTRUE) {
  }
  const std::array<uint8_t, 8> request{
      0x40, static_cast<uint8_t>(index), static_cast<uint8_t>(index >> 8), 0x00, 0x00, 0x00, 0x00,
      0x00};
  if (!send_raw(0x600 + node_id_, request)) {
    return false;
  }

  const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
  const TickType_t start_tick = xTaskGetTickCount();
  while (xTaskGetTickCount() - start_tick < timeout_ticks) {
    const TickType_t elapsed = xTaskGetTickCount() - start_tick;
    if (xQueueReceive(receive_queue_, &response, timeout_ticks - elapsed) != pdTRUE) {
      break;
    }
    logger_.debug("CAN RX id=0x{:03X}, dlc={}, data={:02X} {:02X} {:02X} {:02X} {:02X} {:02X} "
                  "{:02X} {:02X}",
                  response.id, response.dlc, response.data[0], response.data[1], response.data[2],
                  response.data[3], response.data[4], response.data[5], response.data[6],
                  response.data[7]);
    if (response.id != 0x580 + node_id_ || response.dlc != 8 ||
        response.data[1] != static_cast<uint8_t>(index) ||
        response.data[2] != static_cast<uint8_t>(index >> 8) || response.data[3] != 0x00) {
      continue;
    }
    if (response.data[0] == 0x80) {
      logger_.error("SDO read 0x{:04X} aborted: 0x{:02X}{:02X}{:02X}{:02X}", index,
                    response.data[7], response.data[6], response.data[5], response.data[4]);
      return false;
    }
    if (response.data[0] != expected_response_command) {
      logger_.error("SDO read 0x{:04X} returned unsupported response 0x{:02X}", index,
                    response.data[0]);
      return false;
    }
    const uint32_t raw_value = static_cast<uint32_t>(response.data[4]) |
                               static_cast<uint32_t>(response.data[5]) << 8 |
                               static_cast<uint32_t>(response.data[6]) << 16 |
                               static_cast<uint32_t>(response.data[7]) << 24;
    value = raw_value;
    return true;
  }
  logger_.error("Timed out reading SDO 0x{:04X}", index);
  return false;
}

bool EyouMotor::get_position(float &position_degrees, uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  constexpr uint8_t kMaxAttempts = 3;
  for (uint8_t attempt = 0; attempt < kMaxAttempts; ++attempt) {
    uint32_t raw_position = 0;
    if (!request_sdo_u32(kPositionActualIndex, 0x43, raw_position, timeout_ms)) {
      continue;
    }
    const int32_t relative_pulses =
        static_cast<int32_t>(raw_position) - zero_offset_pulses_.value_or(0);
    position_degrees = pulses_to_degrees(relative_pulses);
    return true;
  }
  return false;
}

bool EyouMotor::get_statusword(uint16_t &statusword, uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  uint32_t raw_statusword = 0;
  if (!request_sdo_u32(kStatuswordIndex, 0x4B, raw_statusword, timeout_ms)) {
    return false;
  }
  statusword = static_cast<uint16_t>(raw_statusword);
  return true;
}

bool EyouMotor::get_error_code(uint16_t &error_code, uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  uint32_t raw_error_code = 0;
  if (!request_sdo_u32(kErrorCodeIndex, 0x4B, raw_error_code, timeout_ms)) {
    return false;
  }
  error_code = static_cast<uint16_t>(raw_error_code);
  return true;
}

bool EyouMotor::is_faulted(uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  return is_faulted_unlocked(timeout_ms);
}

bool EyouMotor::is_faulted_unlocked(uint32_t timeout_ms) {
  uint32_t raw_statusword = 0;
  if (!request_sdo_u32(kStatuswordIndex, 0x4B, raw_statusword, timeout_ms)) {
    logger_.warn("Could not read statusword; treating drive as faulted");
    return true;
  }
  return (raw_statusword & 0x0008) != 0;
}

bool EyouMotor::get_operating_mode(int8_t &mode, uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  uint32_t raw_mode = 0;
  if (!request_sdo_u32(kModesOfOperationDisplayIndex, 0x4F, raw_mode, timeout_ms)) {
    return false;
  }
  mode = static_cast<int8_t>(raw_mode);
  return true;
}

bool EyouMotor::save_parameters(uint8_t sub_index, uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  if (!send_sdo_u32(kStoreParametersIndex, kStoreParametersSignature, sub_index, timeout_ms)) {
    logger_.error("Failed to store parameters at 0x1010:{:02X} (drive may not support saving, "
                  "or refused with 'cannot store'/'local control')",
                  sub_index);
    return false;
  }
  logger_.info("Stored parameters to non-volatile memory (0x1010:{:02X})", sub_index);
  return true;
}

bool EyouMotor::send_raw(uint32_t can_id, std::span<const uint8_t> data, bool extended,
                         bool rtr) const {
  const uint32_t max_id = extended ? 0x1FFFFFFF : 0x7FF;
  if (can_id > max_id || data.size() > 8) {
    ESP_LOGE(kTag, "Invalid raw CAN frame: id=0x%lx, extended=%d, length=%u",
             static_cast<unsigned long>(can_id), extended, static_cast<unsigned>(data.size()));
    return false;
  }

  EyouCanFrame frame{};
  frame.id = can_id;
  frame.extended = extended;
  frame.rtr = rtr;
  frame.dlc = static_cast<uint8_t>(data.size());
  std::copy(data.begin(), data.end(), frame.data.begin());
  return bus_.send(frame);
}

// --- live readings, the torque limit, zero-at, profile writes, halt ---------------

bool EyouMotor::request_sdo_expedited(uint16_t index, uint8_t subindex, uint32_t &value,
                                      uint8_t &size, uint32_t timeout_ms) {
  if (receive_queue_ == nullptr) {
    logger_.error("Cannot read SDO: CAN receive queue is not started");
    return false;
  }
  EyouCanFrame response{};
  while (xQueueReceive(receive_queue_, &response, 0) == pdTRUE) {
  }
  const std::array<uint8_t, 8> request{0x40,
                                       static_cast<uint8_t>(index),
                                       static_cast<uint8_t>(index >> 8),
                                       subindex,
                                       0x00,
                                       0x00,
                                       0x00,
                                       0x00};
  if (!send_raw(0x600 + node_id_, request)) {
    return false;
  }
  const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
  const TickType_t start_tick = xTaskGetTickCount();
  while (xTaskGetTickCount() - start_tick < timeout_ticks) {
    const TickType_t elapsed = xTaskGetTickCount() - start_tick;
    if (xQueueReceive(receive_queue_, &response, timeout_ticks - elapsed) != pdTRUE) {
      break;
    }
    if (response.id != 0x580 + node_id_ || response.dlc != 8 ||
        response.data[1] != static_cast<uint8_t>(index) ||
        response.data[2] != static_cast<uint8_t>(index >> 8) || response.data[3] != subindex) {
      continue;
    }
    if (response.data[0] == 0x80) {
      logger_.error("SDO read 0x{:04X}.{} aborted: 0x{:02X}{:02X}{:02X}{:02X}", index, subindex,
                    response.data[7], response.data[6], response.data[5], response.data[4]);
      return false;
    }
    // expedited upload response: 0x4F / 0x4B / 0x47 / 0x43 = 1 / 2 / 3 / 4 bytes
    switch (response.data[0]) {
    case 0x4F:
      size = 1;
      break;
    case 0x4B:
      size = 2;
      break;
    case 0x47:
      size = 3;
      break;
    case 0x43:
      size = 4;
      break;
    default:
      logger_.error("SDO read 0x{:04X}.{} returned unsupported response 0x{:02X}", index, subindex,
                    response.data[0]);
      return false;
    }
    value = static_cast<uint32_t>(response.data[4]) | static_cast<uint32_t>(response.data[5]) << 8 |
            static_cast<uint32_t>(response.data[6]) << 16 |
            static_cast<uint32_t>(response.data[7]) << 24;
    return true;
  }
  logger_.error("Timed out reading SDO 0x{:04X}.{}", index, subindex);
  return false;
}

bool EyouMotor::request_sdo_signed(uint16_t index, uint8_t subindex, int32_t &value,
                                   uint32_t timeout_ms) {
  uint32_t raw = 0;
  uint8_t size = 0;
  if (!request_sdo_expedited(index, subindex, raw, size, timeout_ms)) {
    return false;
  }
  switch (size) {
  case 1:
    value = static_cast<int8_t>(raw & 0xFF);
    break;
  case 2:
    value = static_cast<int16_t>(raw & 0xFFFF);
    break;
  default:
    value = static_cast<int32_t>(raw);
    break;
  }
  return true;
}

bool EyouMotor::get_velocity(float &velocity_degrees_per_second, uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  int32_t pulses_per_second = 0;
  if (!request_sdo_signed(kVelocityActualIndex, 0, pulses_per_second, timeout_ms)) {
    return false;
  }
  velocity_degrees_per_second = pulses_to_degrees(pulses_per_second);
  return true;
}

bool EyouMotor::get_torque(int16_t &torque_permille, uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  int32_t value = 0;
  if (!request_sdo_signed(kTorqueActualIndex, 0, value, timeout_ms)) {
    return false;
  }
  torque_permille = static_cast<int16_t>(std::clamp<int32_t>(value, INT16_MIN, INT16_MAX));
  return true;
}

bool EyouMotor::get_current(int16_t &current_permille, uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  int32_t value = 0;
  if (!request_sdo_signed(kCurrentActualIndex, 0, value, timeout_ms)) {
    return false;
  }
  current_permille = static_cast<int16_t>(std::clamp<int32_t>(value, INT16_MIN, INT16_MAX));
  return true;
}

bool EyouMotor::set_max_torque(uint16_t torque_permille, uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  if (!send_sdo_u16(kMaxTorqueIndex, torque_permille, 0, timeout_ms)) {
    logger_.error("Failed to set the max torque to {} permille", torque_permille);
    return false;
  }
  logger_.info("Max torque set to {} permille of rated", torque_permille);
  return true;
}

bool EyouMotor::get_max_torque(uint16_t &torque_permille, uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  uint32_t raw = 0;
  uint8_t size = 0;
  if (!request_sdo_expedited(kMaxTorqueIndex, 0, raw, size, timeout_ms)) {
    return false;
  }
  torque_permille = static_cast<uint16_t>(raw & 0xFFFF);
  return true;
}

bool EyouMotor::get_rated_torque(uint32_t &rated_torque_mnm, uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  uint8_t size = 0;
  return request_sdo_expedited(kRatedTorqueIndex, 0, rated_torque_mnm, size, timeout_ms);
}

bool EyouMotor::get_rated_current(uint32_t &rated_current_ma, uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  uint8_t size = 0;
  return request_sdo_expedited(kRatedCurrentIndex, 0, rated_current_ma, size, timeout_ms);
}

bool EyouMotor::zero_at(float current_angle_degrees, uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  int32_t angle_pulses = 0;
  if (!degrees_to_pulses(current_angle_degrees, angle_pulses)) {
    return false;
  }
  uint32_t raw_position = 0;
  if (!request_sdo_u32(kPositionActualIndex, 0x43, raw_position, timeout_ms)) {
    logger_.error("Failed to read actual position while setting software zero");
    return false;
  }
  // offset such that (raw - offset) == angle_pulses
  const int64_t offset =
      static_cast<int64_t>(static_cast<int32_t>(raw_position)) - static_cast<int64_t>(angle_pulses);
  if (offset < std::numeric_limits<int32_t>::min() ||
      offset > std::numeric_limits<int32_t>::max()) {
    logger_.error("Zero at {} degrees puts the offset outside the 32-bit encoder range",
                  current_angle_degrees);
    return false;
  }
  zero_offset_pulses_ = static_cast<int32_t>(offset);
  logger_.info("Software zero set: actual position {} pulses now reads {} degrees (offset {})",
               static_cast<int32_t>(raw_position), current_angle_degrees, *zero_offset_pulses_);
  return true;
}

bool EyouMotor::set_profile_position_config(const ProfilePositionConfig &config,
                                            uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  uint32_t velocity = 0, acceleration = 0, deceleration = 0;
  if (!degrees_to_profile_units(config.velocity_degrees_per_second, velocity) ||
      !degrees_to_profile_units(config.acceleration_degrees_per_second_squared, acceleration) ||
      !degrees_to_profile_units(config.deceleration_degrees_per_second_squared, deceleration)) {
    return false;
  }
  if (!send_sdo_u32(kProfileVelocityIndex, velocity, 0, timeout_ms) ||
      !send_sdo_u32(kProfileAccelerationIndex, acceleration, 0, timeout_ms) ||
      !send_sdo_u32(kProfileDecelerationIndex, deceleration, 0, timeout_ms)) {
    logger_.error("Failed to write the profile velocity / acceleration / deceleration");
    return false;
  }
  logger_.info("Profile set: {} deg/s, {} / {} deg/s^2", config.velocity_degrees_per_second,
               config.acceleration_degrees_per_second_squared,
               config.deceleration_degrees_per_second_squared);
  return true;
}

bool EyouMotor::halt(uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  // halt bit set, operation stays enabled: the drive decelerates
  if (!send_sdo_u16(kControlwordIndex, kControlwordEnableOperation | kControlwordHalt)) {
    return false;
  }
  // wait until it has come to rest (or give up waiting, the halt is in effect)
  const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
  const TickType_t start_tick = xTaskGetTickCount();
  while (xTaskGetTickCount() - start_tick < timeout_ticks) {
    int32_t pulses_per_second = 0;
    if (request_sdo_signed(kVelocityActualIndex, 0, pulses_per_second, 100) &&
        std::abs(pulses_to_degrees(pulses_per_second)) < 0.5f) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  // the position reached becomes the set-point, so releasing the halt bit does
  // not resume the interrupted move
  uint32_t raw_position = 0;
  const bool have_position = request_sdo_u32(kPositionActualIndex, 0x43, raw_position, 100);
  if (have_position) {
    send_sdo_u32(kTargetPositionIndex, raw_position);
    // new set-point (absolute, immediate) with the halt bit still raised
    send_sdo_u16(kControlwordIndex, kControlwordEnableOperation | kControlwordHalt |
                                        kControlwordNewSetpoint | kControlwordImmediate);
  }
  const bool released = send_sdo_u16(kControlwordIndex, kControlwordEnableOperation);
  logger_.info("Halted{}", have_position ? "" : " (position not re-targeted)");
  return have_position && released;
}

std::optional<int32_t> EyouMotor::zero_offset_pulses() const {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  return zero_offset_pulses_;
}

float EyouMotor::minimum_position_degrees() const {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  return minimum_position_degrees_;
}

float EyouMotor::maximum_position_degrees() const {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  return maximum_position_degrees_;
}

bool EyouMotor::rebind_receivers_unlocked(uint8_t new_node_id) {
  if (receive_queue_ == nullptr) {
    logger_.error("Cannot change the node id: CAN receive queue is not started");
    return false;
  }
  const uint8_t old_node_id = node_id_;
  bus_.unregister_receiver(receive_queue_);
  xQueueReset(receive_queue_); // replies of the old node are not ours any more
  node_id_ = new_node_id;
  if (!bus_.register_receiver(0x580 + node_id_, receive_queue_) ||
      !bus_.register_receiver(0x700 + node_id_, receive_queue_)) {
    logger_.error("Failed to register CANopen receivers for node {} (receiver table full?); "
                  "staying on node {}",
                  node_id_, old_node_id);
    bus_.unregister_receiver(receive_queue_);
    node_id_ = old_node_id;
    bus_.register_receiver(0x580 + node_id_, receive_queue_);
    bus_.register_receiver(0x700 + node_id_, receive_queue_);
    return false;
  }
  return true;
}

bool EyouMotor::set_node_id(uint8_t new_node_id) {
  if (new_node_id < 1 || new_node_id > 127) {
    logger_.error("Invalid CANopen node id {} (1..127)", new_node_id);
    return false;
  }
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  if (new_node_id == node_id_)
    return true;
  if (!rebind_receivers_unlocked(new_node_id))
    return false;
  zero_offset_pulses_.reset(); // the software zero belongs to the previous node
  logger_.info("Now talking to CANopen node {} (software zero cleared: load_zero() restores the "
               "one saved for it)",
               node_id_);
  return true;
}

bool EyouMotor::set_drive_node_id(uint8_t new_node_id, uint32_t timeout_ms) {
  if (new_node_id < 1 || new_node_id > 127) {
    logger_.error("Invalid CANopen node id {} (1..127)", new_node_id);
    return false;
  }
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  const uint8_t old_node_id = node_id_;
  if (new_node_id == old_node_id) {
    logger_.info("The drive already is node {}", old_node_id);
    return true;
  }
  // 1. the new id into the drive's Node-ID object (takes effect after a restart)
  if (!send_sdo_u8(kNodeIdIndex, new_node_id)) {
    logger_.error("Failed to write node id {} to 0x26A0 on node {}", new_node_id, old_node_id);
    return false;
  }
  // 2. save it: 0x1010:01 reads 1 while saving and 2 once done (about 3 s)
  if (!send_sdo_u32(kStoreParametersIndex, kStoreParametersSignature, 1, 1000)) {
    logger_.error("Node {}: the drive refused the parameter save (0x1010:01); its node id is "
                  "unchanged until saved",
                  old_node_id);
    return false;
  }
  const TickType_t start_tick = xTaskGetTickCount();
  bool saved = false;
  while (xTaskGetTickCount() - start_tick < pdMS_TO_TICKS(timeout_ms)) {
    vTaskDelay(pdMS_TO_TICKS(250));
    uint32_t value = 0;
    uint8_t size = 0;
    if (request_sdo_expedited(kStoreParametersIndex, 1, value, size, 200) && value == 2) {
      saved = true;
      break;
    }
  }
  if (!saved) {
    logger_.error("Node {}: the drive did not confirm the parameter save within {} ms. If it "
                  "stops answering after a power cycle, it did save: select node {}",
                  old_node_id, timeout_ms, new_node_id);
    return false;
  }
  logger_.info("Node id {} written and saved on node {}; restarting the drive", new_node_id,
               old_node_id);
  // 3. restart it so the new id takes effect, and follow it
  EyouCanFrame frame{};
  while (xQueueReceive(receive_queue_, &frame, 0) == pdTRUE) {
  }
  const std::array<uint8_t, 2> nmt_reset_node{0x81, old_node_id};
  send_raw(0x000, nmt_reset_node);
  if (!rebind_receivers_unlocked(new_node_id))
    return false;
  zero_offset_pulses_.reset();
  bool booted = false;
  const TickType_t boot_timeout = pdMS_TO_TICKS(3000);
  const TickType_t boot_start = xTaskGetTickCount();
  while (xTaskGetTickCount() - boot_start < boot_timeout) {
    const TickType_t elapsed = xTaskGetTickCount() - boot_start;
    if (xQueueReceive(receive_queue_, &frame, boot_timeout - elapsed) != pdTRUE)
      break;
    if (frame.id == 0x700u + node_id_ && frame.dlc >= 1 && frame.data[0] == 0x00) {
      booted = true;
      break;
    }
  }
  if (booted)
    logger_.info("The drive is now node {} (booted with its new id)", node_id_);
  else
    logger_.warn("Node id {} saved, but no boot-up on the new id after the NMT reset: power-cycle "
                 "the drive (this object already talks to node {})",
                 node_id_, node_id_);
  return true;
}
