#include "eyou_motor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "freertos/task.h"

#include "esp_log.h"

namespace {
constexpr char kTag[] = "EyouMotor";
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
constexpr uint16_t kControlwordShutdown = 0x0006;
constexpr uint16_t kControlwordSwitchOn = 0x0007;
constexpr uint16_t kControlwordEnableOperation = 0x000F;
constexpr uint16_t kControlwordFaultReset = 0x0080;
constexpr uint16_t kControlwordNewSetpoint = 0x0010;
constexpr uint16_t kControlwordImmediate = 0x0020;
constexpr uint16_t kControlwordRelative = 0x0040;
}

EyouMotor::EyouMotor(CanopenBus &bus, uint8_t node_id)
		: espp::BaseComponent("EyouMotor", espp::Logger::Verbosity::INFO), bus_(bus), node_id_(node_id) {
	receive_queue_ = xQueueCreate(16, sizeof(EyouCanFrame));
	if (receive_queue_ == nullptr || !bus_.register_receiver(0x580 + node_id_, receive_queue_) ||
			!bus_.register_receiver(0x700 + node_id_, receive_queue_)) {
		logger_.error("Failed to register CANopen receivers for node {}", node_id_);
	}
}

EyouMotor::~EyouMotor() {
	if (receive_queue_ != nullptr) {
		vQueueDelete(receive_queue_);
	}
}

bool EyouMotor::send_sdo_u8(uint16_t index, uint8_t value) {
	return send_sdo_download(index, 0x2F, value);
}

bool EyouMotor::send_sdo_u16(uint16_t index, uint16_t value) {
	return send_sdo_download(index, 0x2B, value);
}

bool EyouMotor::send_sdo_u32(uint16_t index, uint32_t value) {
	return send_sdo_download(index, 0x23, value);
}

bool EyouMotor::send_sdo_download(uint16_t index, uint8_t command, uint32_t value) {
	if (receive_queue_ == nullptr) {
		logger_.error("Cannot write SDO: CAN receive queue is not started");
		return false;
	}

	EyouCanFrame response{};
	while (xQueueReceive(receive_queue_, &response, 0) == pdTRUE) {
	}
	const std::array<uint8_t, 8> data{command, static_cast<uint8_t>(index),
															static_cast<uint8_t>(index >> 8), 0x00, static_cast<uint8_t>(value),
															static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value >> 16),
															static_cast<uint8_t>(value >> 24)};
	if (!send_raw(0x600 + node_id_, data)) {
		return false;
	}
	if (xQueueReceive(receive_queue_, &response, pdMS_TO_TICKS(100)) != pdTRUE) {
		logger_.error("Timed out writing SDO 0x{:04X}", index);
		return false;
	}
	if (response.id != 0x580 + node_id_ || response.dlc != 8 ||
				response.data[1] != static_cast<uint8_t>(index) ||
				response.data[2] != static_cast<uint8_t>(index >> 8) || response.data[3] != 0x00) {
		logger_.error("Unexpected SDO response while writing 0x{:04X}", index);
		return false;
	}
	if (response.data[0] == 0x80) {
		logger_.error(
				"SDO write 0x{:04X} aborted: 0x{:02X}{:02X}{:02X}{:02X}; response={:02X} {:02X} "
				"{:02X} {:02X} {:02X} {:02X} {:02X} {:02X}",
				index, response.data[7], response.data[6], response.data[5], response.data[4],
				response.data[0], response.data[1], response.data[2], response.data[3],
				response.data[4], response.data[5], response.data[6], response.data[7]);
		return false;
	}
	if (response.data[0] != 0x60) {
		logger_.error("SDO write 0x{:04X} returned unsupported response 0x{:02X}", index,
							response.data[0]);
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
	const int32_t current_position = static_cast<int32_t>(raw_position);

	const std::array<uint8_t, 2> nmt_start{0x01, node_id_};
	if (!send_raw(0x000, nmt_start) || !ensure_operation_enabled(1000) ||
			!send_sdo_u8(kModesOfOperationIndex, 1)) {
		return false;
	}

	int8_t mode = 0;
	for (int attempt = 0; attempt < 20; ++attempt) {
		uint32_t raw_mode = 0;
		if (!request_sdo_u32(kModesOfOperationDisplayIndex, 0x4F, raw_mode, 100)) {
			logger_.error("Failed to verify Profile Position mode");
			return false;
		}
		mode = static_cast<int8_t>(raw_mode);
		if (mode == 1) {
			logger_.info("Mode display after PP request: {}", static_cast<int>(mode));
			break;
		}
		vTaskDelay(pdMS_TO_TICKS(50));
	}
	if (mode != 1) {
		logger_.error("Profile Position mode requested 1, but drive reports {} after 1 second",
						static_cast<int>(mode));
		return false;
	}

	return send_sdo_u32(kTargetPositionIndex, static_cast<uint32_t>(current_position)) &&
			 send_sdo_u32(kProfileVelocityIndex, velocity_pulses_per_second) &&
			 send_sdo_u32(kProfileAccelerationIndex, acceleration_pulses_per_second_squared) &&
			 send_sdo_u32(kProfileDecelerationIndex, deceleration_pulses_per_second_squared);
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
	const double raw_pulses = static_cast<double>(degrees) * kPulsesPerOutputTurn /
									 kDegreesPerOutputTurn;
	if (!std::isfinite(raw_pulses) || raw_pulses < std::numeric_limits<int32_t>::min() ||
			raw_pulses > std::numeric_limits<int32_t>::max()) {
		logger_.error("Position {} degrees is outside the signed 32-bit encoder range", degrees);
		return false;
	}
	pulses = static_cast<int32_t>(std::lround(raw_pulses));
	return true;
}

bool EyouMotor::degrees_to_profile_units(float degrees, uint32_t &pulses) const {
	const double raw_pulses = static_cast<double>(degrees) * kPulsesPerOutputTurn /
									 kDegreesPerOutputTurn;
	if (!std::isfinite(raw_pulses) || raw_pulses < 0.0 ||
			raw_pulses > std::numeric_limits<uint32_t>::max()) {
		logger_.error("Profile quantity {} degrees is outside the unsigned 32-bit pulse range", degrees);
		return false;
	}
	pulses = static_cast<uint32_t>(std::llround(raw_pulses));
	return true;
}

float EyouMotor::pulses_to_degrees(int32_t pulses) {
	return static_cast<float>(pulses) * kDegreesPerOutputTurn / kPulsesPerOutputTurn;
}

bool EyouMotor::move_absolute(float target_degrees, bool immediate) {
	std::lock_guard<std::mutex> lock(transaction_mutex_);
	int32_t target_pulses = 0;
	if (!degrees_to_pulses(target_degrees, target_pulses)) {
		return false;
	}
	return ensure_operation_enabled(1000) &&
			 send_sdo_u32(kTargetPositionIndex, static_cast<uint32_t>(target_pulses)) &&
			 trigger_profile_position(false, immediate);
}

bool EyouMotor::move_incremental(float increment_degrees, bool immediate) {
	std::lock_guard<std::mutex> lock(transaction_mutex_);
	int32_t increment_pulses = 0;
	if (!degrees_to_pulses(increment_degrees, increment_pulses)) {
		return false;
	}
	return ensure_operation_enabled(1000) &&
			 send_sdo_u32(kTargetPositionIndex, static_cast<uint32_t>(increment_pulses)) &&
			 trigger_profile_position(true, immediate);
}

bool EyouMotor::request_sdo_u32(uint16_t index, uint8_t expected_response_command,
								uint32_t &value, uint32_t timeout_ms) {
	if (receive_queue_ == nullptr) {
		logger_.error("Cannot read SDO: CAN receive queue is not started");
		return false;
	}

	EyouCanFrame response{};
	while (xQueueReceive(receive_queue_, &response, 0) == pdTRUE) {
	}
	const std::array<uint8_t, 8> request{0x40, static_cast<uint8_t>(index),
																static_cast<uint8_t>(index >> 8), 0x00, 0x00, 0x00, 0x00, 0x00};
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
		logger_.info("CAN RX id=0x{:03X}, dlc={}, data={:02X} {:02X} {:02X} {:02X} {:02X} {:02X} "
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
	uint32_t raw_position = 0;
	if (!request_sdo_u32(kPositionActualIndex, 0x43, raw_position, timeout_ms)) {
		return false;
	}
	position_degrees = pulses_to_degrees(static_cast<int32_t>(raw_position));
	return true;
}

bool EyouMotor::get_statusword(uint16_t &statusword, uint32_t timeout_ms) {
	std::lock_guard<std::mutex> lock(transaction_mutex_);
	uint32_t raw_statusword = 0;
	if (!request_sdo_u32(kControlwordIndex + 1, 0x4B, raw_statusword, timeout_ms)) {
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

bool EyouMotor::get_operating_mode(int8_t &mode, uint32_t timeout_ms) {
	std::lock_guard<std::mutex> lock(transaction_mutex_);
	uint32_t raw_mode = 0;
	if (!request_sdo_u32(kModesOfOperationDisplayIndex, 0x4F, raw_mode, timeout_ms)) {
		return false;
	}
	mode = static_cast<int8_t>(raw_mode);
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
