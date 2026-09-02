#include <algorithm>
#include <span>
#include <thread>

#include "mcp266_controller.hpp"

using namespace std::chrono_literals;

namespace mib {

Mcp266Controller::Mcp266Controller(const Config &config)
    : BaseComponent("Mcp266Controller", config.log_level)
    , config_(config) {}

Mcp266Controller::~Mcp266Controller() {
  if (serial_rx_queue_) {
    vQueueDelete(serial_rx_queue_);
    serial_rx_queue_ = nullptr;
  }
}

bool Mcp266Controller::initialize(std::error_code &ec) {
  ec.clear();
  logger_.info("Initializing Mcp266Controller on TWAI (TX GPIO {}, RX GPIO {}, Baud {})",
               static_cast<int>(config_.twai_tx_gpio), static_cast<int>(config_.twai_rx_gpio),
               config_.baudrate);

  if (!init_twai(ec)) {
    logger_.error("Failed to initialize TWAI transport: {}", ec.message());
    return false;
  }

  if (config_.mode == Mode::CANOPEN) {
    if (!init_canopen(ec)) {
      logger_.error("Failed to initialize CANopen mode: {}", ec.message());
      return false;
    }
  } else {
    if (!init_packet_serial(ec)) {
      logger_.error("Failed to initialize Packet Serial mode: {}", ec.message());
      return false;
    }
  }

  logger_.info("Mcp266Controller initialized successfully");
  return true;
}

bool Mcp266Controller::init_twai(std::error_code &ec) {
  twai_ = std::make_unique<espp::Twai>(espp::Twai::Config{
      .tx_gpio = config_.twai_tx_gpio,
      .rx_gpio = config_.twai_rx_gpio,
      .baudrate = config_.baudrate,
      .mode = espp::Twai::Mode::NORMAL,
      .tx_queue_depth = 10,
      .on_receive =
          [this](const espp::Twai::Message &msg) {
            if (config_.mode == Mode::CANOPEN) {
              if (canopen_client_) {
                canopen_client_->process_frame(espp::CanopenClient::CanFrame{
                    .id = msg.id,
                    .extended = msg.extended,
                    .rtr = msg.rtr,
                    .dlc = msg.dlc,
                    .data = msg.data,
                });
              }
            } else if (config_.mode == Mode::PACKET_SERIAL && serial_rx_queue_) {
              for (uint8_t i = 0; i < msg.dlc; ++i) {
                xQueueSend(serial_rx_queue_, &msg.data[i], 0);
              }
            }
          },
      .log_level = config_.log_level,
  });

  return twai_->initialize(ec);
}

bool Mcp266Controller::init_canopen(std::error_code &ec) {
  logger_.info("Configuring CANopen mode for Node ID {}", config_.node_id);

  canopen_client_ = std::make_unique<espp::CanopenClient>(espp::CanopenClient::Config{
      .node_id = config_.node_id,
      .send =
          [this](const espp::CanopenClient::CanFrame &frame) {
            espp::Twai::Message msg{
                .id = frame.id,
                .extended = frame.extended,
                .rtr = frame.rtr,
                .dlc = frame.dlc,
                .data = frame.data,
            };
            std::error_code tx_ec;
            return twai_->transmit(msg, tx_ec);
          },
      .sdo_timeout = 500ms,
      .on_heartbeat = nullptr,
  });

  drive_m1_ = std::make_unique<espp::Ds402Drive>(*canopen_client_,
                                                 espp::Ds402Drive::Config{
                                                     .state_timeout = 1s,
                                                     .poll_period = 25ms,
                                                     .log_level = config_.log_level,
                                                 });

  // Send NMT start to the node
  if (!canopen_client_->nmt_start(ec)) {
    logger_.warn("Failed to send NMT start to Node ID {}: {}", config_.node_id, ec.message());
  }

  if (!reset_faults(ec)) {
    logger_.warn("Fault reset did not complete: {}", ec.message());
  }

  return true;
}

bool Mcp266Controller::init_packet_serial(std::error_code &ec) {
  logger_.info("Configuring Packet Serial over TWAI (Address 0x{:02X})",
               config_.packet_serial_address);

  serial_rx_queue_ = xQueueCreate(256, sizeof(uint8_t));
  if (!serial_rx_queue_) {
    logger_.error("Failed to create serial receive queue");
    ec = std::make_error_code(std::errc::not_enough_memory);
    return false;
  }

  espp::Basicmicro::Config bm_config{
      .address = config_.packet_serial_address,
      .write =
          [this](std::span<const uint8_t> data) -> bool {
            size_t offset = 0;
            while (offset < data.size()) {
              espp::Twai::Message msg{};
              msg.id = config_.packet_serial_address;
              msg.extended = false;
              msg.rtr = false;
              const size_t len = std::min<size_t>(8, data.size() - offset);
              msg.dlc = static_cast<uint8_t>(len);
              std::copy_n(data.begin() + offset, len, msg.data.begin());

              std::error_code tx_ec;
              if (!twai_->transmit(msg, tx_ec)) {
                return false;
              }
              offset += len;
            }
            return true;
          },
      .read =
          [this](std::span<uint8_t> data, std::chrono::milliseconds timeout) -> size_t {
            size_t bytes_read = 0;
            const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout.count());
            const TickType_t start_ticks = xTaskGetTickCount();

            while (bytes_read < data.size()) {
              TickType_t elapsed = xTaskGetTickCount() - start_ticks;
              if (elapsed >= timeout_ticks) {
                break;
              }
              uint8_t byte = 0;
              if (xQueueReceive(serial_rx_queue_, &byte, timeout_ticks - elapsed) == pdTRUE) {
                data[bytes_read++] = byte;
              } else {
                break;
              }
            }
            return bytes_read;
          },
      .timeout = 200ms,
      .log_level = config_.log_level,
  };

  basicmicro_ = std::make_unique<espp::Basicmicro>(bm_config);
  return true;
}

// --- Mode Control ---

bool Mcp266Controller::set_mode_m1(int8_t mode, std::error_code &ec) {
  ec.clear();
  if (config_.mode == Mode::CANOPEN && canopen_client_) {
    return canopen_client_->write_i8(0x6060, 0, mode, ec);
  }
  return false;
}

int8_t Mcp266Controller::get_mode_m1(std::error_code &ec) {
  ec.clear();
  if (config_.mode == Mode::CANOPEN && canopen_client_) {
    return canopen_client_->read_i8(0x6061, 0, ec);
  }
  return 0;
}

bool Mcp266Controller::set_mode_m2(int8_t mode, std::error_code &ec) {
  ec.clear();
  if (config_.mode == Mode::CANOPEN && canopen_client_) {
    return canopen_client_->write_i8(0x6860, 0, mode, ec);
  }
  return false;
}

int8_t Mcp266Controller::get_mode_m2(std::error_code &ec) {
  ec.clear();
  if (config_.mode == Mode::CANOPEN && canopen_client_) {
    return canopen_client_->read_i8(0x6861, 0, ec);
  }
  return 0;
}

// --- CiA 402 state machine ---

bool Mcp266Controller::reset_axis_fault(const AxisObjects &axis, std::error_code &ec) {
  const uint16_t sw = canopen_client_->read_u16(axis.statusword, 0, ec);
  if (ec) {
    logger_.error("{}: statusword read before fault reset failed: {}", axis.name, ec.message());
    return false;
  }
  if ((sw & 0x004F) != 0x0008) {
    logger_.info("{}: no fault to reset (statusword 0x{:04X})", axis.name, sw);
    return true;
  }

  logger_.warn("{}: resetting fault (statusword 0x{:04X})", axis.name, sw);
  if (!canopen_client_->write_u16(axis.controlword, 0, 0x0080, ec)) {
    return false;
  }
  std::this_thread::sleep_for(50ms);
  if (!canopen_client_->write_u16(axis.controlword, 0, 0x0000, ec)) {
    return false;
  }

  constexpr int kMaxPolls = 20;
  for (int i = 0; i < kMaxPolls; ++i) {
    std::this_thread::sleep_for(25ms);
    const uint16_t reset_sw = canopen_client_->read_u16(axis.statusword, 0, ec);
    if (ec) {
      return false;
    }
    if ((reset_sw & 0x004F) != 0x0008) {
      logger_.info("{}: fault reset (statusword 0x{:04X})", axis.name, reset_sw);
      return true;
    }
  }
  logger_.error("{}: fault remains active after reset", axis.name);
  ec = std::make_error_code(std::errc::protocol_error);
  return false;
}

bool Mcp266Controller::reset_faults(std::error_code &ec) {
  ec.clear();
  if (config_.mode != Mode::CANOPEN || !canopen_client_) {
    return true;
  }
  if (!reset_axis_fault(kAxisM1, ec)) {
    return false;
  }
  return reset_axis_fault(kAxisM2, ec);
}

void Mcp266Controller::log_statusword(const AxisObjects &axis, const char *step,
                                      uint16_t statusword) const {
  logger_.info(
      "{} {}: 0x{:04X} [ready={}, switched_on={}, operation_enabled={}, fault={}, "
      "voltage_enabled={}, quick_stop={}, switch_on_disabled={}, warning={}, remote={}, "
      "target_reached={}, internal_limit={}, setpoint_ack={}, following_error={}]",
      axis.name, step, statusword, (statusword & 0x0001) != 0, (statusword & 0x0002) != 0,
      (statusword & 0x0004) != 0, (statusword & 0x0008) != 0, (statusword & 0x0010) != 0,
      (statusword & 0x0020) != 0, (statusword & 0x0040) != 0, (statusword & 0x0080) != 0,
      (statusword & 0x0200) != 0, (statusword & 0x0400) != 0, (statusword & 0x0800) != 0,
      (statusword & 0x1000) != 0, (statusword & 0x2000) != 0);
}

bool Mcp266Controller::wait_for_statusword(const AxisObjects &axis, uint16_t mask,
                                           uint16_t expected, const char *step,
                                           std::error_code &ec) {
  constexpr int kMaxPolls = 20;
  constexpr auto kPollPeriod = 25ms;
  uint16_t sw = 0;
  for (int i = 0; i < kMaxPolls; ++i) {
    std::this_thread::sleep_for(kPollPeriod);
    sw = canopen_client_->read_u16(axis.statusword, 0, ec);
    if (ec) {
      logger_.error("{} {}: statusword read failed: {}", axis.name, step, ec.message());
      return false;
    }
    if ((sw & 0x004F) == 0x0008) {
      logger_.error("{} {}: drive faulted (statusword 0x{:04X})", axis.name, step, sw);
      ec = std::make_error_code(std::errc::protocol_error);
      return false;
    }
    if ((sw & mask) == expected) {
      logger_.info("{} {}: reached (statusword 0x{:04X})", axis.name, step, sw);
      log_statusword(axis, step, sw);
      return true;
    }
  }
  logger_.error("{} {}: timed out, statusword stuck at 0x{:04X}", axis.name, step, sw);
  ec = std::make_error_code(std::errc::timed_out);
  return false;
}

bool Mcp266Controller::enable_axis(const AxisObjects &axis, int8_t mode, bool verify_mode,
                                   std::error_code &ec) {
  ec.clear();
  if (!canopen_client_->write_i8(axis.mode, 0, mode, ec)) {
    logger_.error("{}: failed to set mode {}: {}", axis.name, mode, ec.message());
    return false;
  }
  std::this_thread::sleep_for(25ms);
  const int8_t display = canopen_client_->read_i8(axis.mode_display, 0, ec);
  if (ec) {
    logger_.error("{}: mode display read failed: {}", axis.name, ec.message());
    return false;
  }
  if (verify_mode && display != mode) {
    logger_.error("{}: mode {} rejected, drive reports {}", axis.name, mode, display);
    ec = std::make_error_code(std::errc::not_supported);
    return false;
  }
  logger_.info("{}: requested mode {}, display {}", axis.name, mode, display);

  if (!reset_axis_fault(axis, ec)) {
    return false;
  }

  // The CiA 402 state lives in statusword bits 0-3, 5 and 6 (mask 0x6F); bit 4
  // (voltage enabled) merely reflects main power and must not be matched.
  // Shutdown -> Ready to switch on (statusword xxxx xxxx x01x 0001)
  if (!canopen_client_->write_u16(axis.controlword, 0, 0x0006, ec) ||
      !wait_for_statusword(axis, 0x006F, 0x0021, "Ready to switch on", ec)) {
    return false;
  }
  // Switch On -> Switched on (statusword xxxx xxxx x01x 0011)
  if (!canopen_client_->write_u16(axis.controlword, 0, 0x0007, ec) ||
      !wait_for_statusword(axis, 0x006F, 0x0023, "Switched on", ec)) {
    return false;
  }
  // Enable Operation -> Operation enabled (statusword xxxx xxxx x01x 0111)
  if (!canopen_client_->write_u16(axis.controlword, 0, 0x000F, ec) ||
      !wait_for_statusword(axis, 0x006F, 0x0027, "Operation enabled", ec)) {
    return false;
  }
  return true;
}

bool Mcp266Controller::enable_m1(int8_t mode, bool verify_mode, std::error_code &ec) {
  ec.clear();
  if (!canopen_client_->write_i8(kAxisM1.mode, 0, mode, ec)) {
    logger_.error("M1: failed to set mode {}: {}", mode, ec.message());
    return false;
  }
  std::this_thread::sleep_for(25ms);
  const int8_t display = canopen_client_->read_i8(kAxisM1.mode_display, 0, ec);
  if (ec) {
    logger_.error("M1: mode display read failed: {}", ec.message());
    return false;
  }
  if (display != mode) {
    if (verify_mode) {
      logger_.error("M1: mode {} rejected, drive reports {}", mode, display);
      ec = std::make_error_code(std::errc::not_supported);
      return false;
    }
    logger_.warn("M1: requested mode {}, drive reports {}; continuing", mode, display);
  }

  const auto state = drive_m1_->get_state(ec);
  if (ec) {
    logger_.error("M1: statusword read failed: {}", ec.message());
    return false;
  }
  if (state == espp::Ds402Drive::State::Fault ||
      state == espp::Ds402Drive::State::FaultReactionActive) {
    logger_.warn("M1: drive is in fault; attempting fault reset");
    if (!drive_m1_->fault_reset(ec)) {
      logger_.error("M1: fault reset failed: {}", ec.message());
      return false;
    }
  }
  if (!drive_m1_->enable_operation(ec)) {
    logger_.error("M1: enable operation failed: {}", ec.message());
    return false;
  }
  return true;
}

// --- Motor Duty Control ---

bool Mcp266Controller::drive_m1_duty(int16_t duty, std::error_code &ec) {
  ec.clear();
  if (config_.mode == Mode::PACKET_SERIAL && basicmicro_) {
    return basicmicro_->drive_m1_duty(duty, ec);
  }
  if (config_.mode == Mode::CANOPEN && canopen_client_) {
    // Duty mode (-1) is manufacturer-specific and may not be echoed in the
    // mode display, so don't fail hard on a mismatch.
    if (duty != 0 && !enable_m1(-1, false, ec)) {
      return false;
    }
    return canopen_client_->write_i32(kAxisM1.target, 0, static_cast<int32_t>(duty), ec);
  }
  return false;
}

bool Mcp266Controller::drive_m2_duty(int16_t duty, std::error_code &ec) {
  ec.clear();
  if (config_.mode == Mode::PACKET_SERIAL && basicmicro_) {
    return basicmicro_->drive_m2_duty(duty, ec);
  }
  if (config_.mode == Mode::CANOPEN && canopen_client_) {
    if (duty != 0 && !enable_axis(kAxisM2, -1, false, ec)) {
      return false;
    }
    return canopen_client_->write_i32(kAxisM2.target, 0, static_cast<int32_t>(duty), ec);
  }
  return false;
}

bool Mcp266Controller::drive_duty(int16_t duty_m1, int16_t duty_m2, std::error_code &ec) {
  ec.clear();
  if (config_.mode == Mode::PACKET_SERIAL && basicmicro_) {
    return basicmicro_->drive_duty(duty_m1, duty_m2, ec);
  }
  return drive_m1_duty(duty_m1, ec) && drive_m2_duty(duty_m2, ec);
}

// --- Speed Control ---

bool Mcp266Controller::drive_m1_speed(int32_t qpps, std::error_code &ec) {
  ec.clear();
  if (config_.mode == Mode::PACKET_SERIAL && basicmicro_) {
    return basicmicro_->drive_m1_speed(qpps, ec);
  }
  if (config_.mode == Mode::CANOPEN && canopen_client_) {
    if (qpps != 0 &&
        !enable_m1(static_cast<int8_t>(espp::Ds402Drive::OperatingMode::ProfileVelocity), true,
                   ec)) {
      return false;
    }
    return drive_m1_->set_target_velocity(qpps, ec);
  }
  return false;
}

bool Mcp266Controller::drive_m2_speed(int32_t qpps, std::error_code &ec) {
  ec.clear();
  if (config_.mode == Mode::PACKET_SERIAL && basicmicro_) {
    return basicmicro_->drive_m2_speed(qpps, ec);
  }
  if (config_.mode == Mode::CANOPEN && canopen_client_) {
    if (qpps != 0 && !enable_axis(kAxisM2, 3, true, ec)) {
      return false;
    }
    return canopen_client_->write_i32(kAxisM2.target, 0, qpps, ec);
  }
  return false;
}

bool Mcp266Controller::set_m1_position_limits(int32_t minimum_position, int32_t maximum_position,
                                               std::error_code &ec) {
  ec.clear();
  if (minimum_position > maximum_position) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  if (config_.mode != Mode::CANOPEN || !canopen_client_) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }
  return canopen_client_->write_i32(0x607B, 1, minimum_position, ec) &&
         canopen_client_->write_i32(0x607B, 2, maximum_position, ec);
}

bool Mcp266Controller::get_m1_position_limits(int32_t &minimum_position, int32_t &maximum_position,
                                               std::error_code &ec) {
  ec.clear();
  if (config_.mode != Mode::CANOPEN || !canopen_client_) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }
  minimum_position = canopen_client_->read_i32(0x607B, 1, ec);
  if (ec) {
    return false;
  }
  maximum_position = canopen_client_->read_i32(0x607B, 2, ec);
  return !ec;
}

bool Mcp266Controller::move_m1_to_position(int32_t target_position, uint32_t profile_velocity,
                                            uint32_t profile_acceleration,
                                            uint32_t profile_deceleration, std::error_code &ec) {
  ec.clear();
  if (config_.mode != Mode::CANOPEN || !canopen_client_) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }
  if (!enable_m1(static_cast<int8_t>(espp::Ds402Drive::OperatingMode::ProfilePosition), false,
                 ec)) {
    return false;
  }
  if (!(drive_m1_->set_profile_acceleration(profile_acceleration, ec) &&
        drive_m1_->set_profile_deceleration(profile_deceleration, ec) &&
        drive_m1_->set_profile_velocity(profile_velocity, ec))) {
    return false;
  }
  // Writes 0x607A and performs the full new-set-point handshake: raise
  // controlword bit 4 (with bit 5, change set immediately), wait for the
  // set-point acknowledge (statusword bit 12), then release bit 4.
  return drive_m1_->set_target_position(target_position, ec);
}

bool Mcp266Controller::drive_speed(int32_t qpps_m1, int32_t qpps_m2, std::error_code &ec) {
  ec.clear();
  if (config_.mode == Mode::PACKET_SERIAL && basicmicro_) {
    return basicmicro_->drive_speed(qpps_m1, qpps_m2, ec);
  }
  return drive_m1_speed(qpps_m1, ec) && drive_m2_speed(qpps_m2, ec);
}

bool Mcp266Controller::stop_motors(std::error_code &ec) {
  ec.clear();
  return drive_speed(0, 0, ec);
}

// --- Encoder Readback ---

bool Mcp266Controller::read_encoder_m1(int32_t &count, uint8_t &status, std::error_code &ec) {
  ec.clear();
  if (config_.mode == Mode::PACKET_SERIAL && basicmicro_) {
    uint32_t u_count = 0;
    bool ok = basicmicro_->read_encoder_m1(u_count, status, ec);
    count = static_cast<int32_t>(u_count);
    return ok;
  }
  if (config_.mode == Mode::CANOPEN && canopen_client_) {
    count = canopen_client_->read_i32(0x6064, 0, ec);
    status = ec ? 0 : 1;
    return !ec;
  }
  return false;
}

bool Mcp266Controller::read_encoder_m2(int32_t &count, uint8_t &status, std::error_code &ec) {
  ec.clear();
  if (config_.mode == Mode::PACKET_SERIAL && basicmicro_) {
    uint32_t u_count = 0;
    bool ok = basicmicro_->read_encoder_m2(u_count, status, ec);
    count = static_cast<int32_t>(u_count);
    return ok;
  }
  if (config_.mode == Mode::CANOPEN && canopen_client_) {
    count = canopen_client_->read_i32(0x6864, 0, ec);
    status = ec ? 0 : 1;
    return !ec;
  }
  return false;
}

bool Mcp266Controller::read_encoders(int32_t &count_m1, int32_t &count_m2, std::error_code &ec) {
  ec.clear();
  if (config_.mode == Mode::PACKET_SERIAL && basicmicro_) {
    uint32_t u1 = 0, u2 = 0;
    bool ok = basicmicro_->read_encoders(u1, u2, ec);
    count_m1 = static_cast<int32_t>(u1);
    count_m2 = static_cast<int32_t>(u2);
    return ok;
  }
  uint8_t st1 = 0, st2 = 0;
  return read_encoder_m1(count_m1, st1, ec) && read_encoder_m2(count_m2, st2, ec);
}

bool Mcp266Controller::read_position_demand_m1(int32_t &demand, std::error_code &ec) {
  ec.clear();
  if (config_.mode != Mode::CANOPEN || !canopen_client_) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }
  demand = canopen_client_->read_i32(0x6062, 0, ec);
  return !ec;
}

bool Mcp266Controller::read_velocity_demand_m1(int32_t &demand, std::error_code &ec) {
  ec.clear();
  if (config_.mode != Mode::CANOPEN || !canopen_client_) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }
  demand = canopen_client_->read_i32(0x606B, 0, ec);
  return !ec;
}

bool Mcp266Controller::read_speed_m1(int32_t &qpps, uint8_t &status, std::error_code &ec) {
  ec.clear();
  if (config_.mode == Mode::PACKET_SERIAL && basicmicro_) {
    return basicmicro_->read_encoder_speed_m1(qpps, status, ec);
  }
  if (config_.mode == Mode::CANOPEN && canopen_client_) {
    qpps = canopen_client_->read_i32(0x606C, 0, ec);
    status = ec ? 0 : 1;
    return !ec;
  }
  return false;
}

bool Mcp266Controller::read_speed_m2(int32_t &qpps, uint8_t &status, std::error_code &ec) {
  ec.clear();
  if (config_.mode == Mode::PACKET_SERIAL && basicmicro_) {
    return basicmicro_->read_encoder_speed_m2(qpps, status, ec);
  }
  if (config_.mode == Mode::CANOPEN && canopen_client_) {
    qpps = canopen_client_->read_i32(0x686C, 0, ec);
    status = ec ? 0 : 1;
    return !ec;
  }
  return false;
}

// --- Telemetry & Diagnostics ---

bool Mcp266Controller::read_device_info(std::string &device_name, uint32_t &device_type,
                                         std::error_code &ec) {
  ec.clear();
  if (config_.mode == Mode::CANOPEN && canopen_client_) {
    device_type = canopen_client_->read_u32(0x1000, 0, ec);
    if (ec) {
      return false;
    }
    device_name = canopen_client_->read_string(0x1008, 0, ec);
    return !ec;
  }
  if (config_.mode == Mode::PACKET_SERIAL && basicmicro_) {
    std::string ver;
    bool ok = basicmicro_->read_firmware_version(ver, ec);
    device_name = ver;
    device_type = 0;
    return ok;
  }
  return false;
}

bool Mcp266Controller::read_object_dictionary(std::string &eds, std::error_code &ec) {
  ec.clear();
  if (config_.mode != Mode::CANOPEN || !canopen_client_) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }
  eds = canopen_client_->read_string(0x1021, 0, ec);
  return !ec;
}

size_t Mcp266Controller::scan_object_dictionary(uint16_t first_index, uint16_t last_index) {
  if (config_.mode != Mode::CANOPEN || !canopen_client_) {
    return 0;
  }
  size_t found = 0;
  // Probing for existence produces an SDO abort for every missing object;
  // mute the client's per-abort error logging for the duration of the scan.
  canopen_client_->set_log_level(espp::Logger::Verbosity::NONE);
  for (uint32_t index = first_index; index <= last_index; ++index) {
    std::error_code ec;
    std::array<uint8_t, 4> data{};
    const size_t len =
        canopen_client_->sdo_upload(static_cast<uint16_t>(index), 0, data, ec);
    if (!ec) {
      const uint32_t value = espp::detail::canopen::get_le(data.data(), len);
      logger_.info("OD 0x{:04X}:00 = 0x{:08X} ({} bytes)", index, value, len);
      ++found;
    } else if (ec == std::errc::protocol_error &&
               canopen_client_->last_abort_code() != 0x06020000) {
      // Any abort other than "object does not exist" (e.g. write-only object,
      // subindex error, segmented/string data) still proves the object exists.
      logger_.info("OD 0x{:04X}:00 exists (abort 0x{:08X})", index,
                   canopen_client_->last_abort_code());
      ++found;
    } else if (ec == std::errc::timed_out) {
      logger_.warn("OD scan aborted at 0x{:04X}: node stopped responding", index);
      break;
    }
    if ((index & 0x03FF) == 0x03FF) {
      logger_.info("OD scan progress: through 0x{:04X}, {} objects so far", index, found);
    }
  }
  canopen_client_->set_log_level(config_.log_level);
  return found;
}

size_t Mcp266Controller::dump_object_subindices(uint16_t first_index, uint16_t last_index) {
  if (config_.mode != Mode::CANOPEN || !canopen_client_) {
    return 0;
  }
  size_t dumped = 0;
  canopen_client_->set_log_level(espp::Logger::Verbosity::NONE);
  for (uint32_t idx = first_index; idx <= last_index; ++idx) {
    const uint16_t index = static_cast<uint16_t>(idx);
    std::error_code ec;
    std::array<uint8_t, 4> data{};
    const size_t len = canopen_client_->sdo_upload(index, 0, data, ec);
    const uint32_t abort = ec ? canopen_client_->last_abort_code() : 0;
    if (ec && (ec != std::errc::protocol_error || abort == 0x06020000)) {
      continue; // not implemented
    }
    // Records/arrays report their subindex count in subindex 0; a write-only
    // subindex 0 aborts, so fall back to probing a fixed number.
    uint8_t count = 8;
    std::string line;
    if (!ec) {
      const uint32_t sub0 = espp::detail::canopen::get_le(data.data(), len);
      line += fmt::format(" [0]=0x{:X}({}B)", sub0, len);
      if (len == 1 && sub0 > 0 && sub0 <= 32) {
        count = static_cast<uint8_t>(sub0);
      }
    } else {
      line += fmt::format(" [0]=<abort 0x{:08X}>", abort);
    }
    for (uint8_t sub = 1; sub <= count; ++sub) {
      std::error_code sub_ec;
      std::array<uint8_t, 4> sub_data{};
      const size_t sub_len = canopen_client_->sdo_upload(index, sub, sub_data, sub_ec);
      if (!sub_ec) {
        line += fmt::format(" [{}]=0x{:X}({}B)", sub,
                            espp::detail::canopen::get_le(sub_data.data(), sub_len), sub_len);
      } else if (canopen_client_->last_abort_code() == 0x06090011) {
        break; // subindex does not exist: past the end of the record
      } else {
        line += fmt::format(" [{}]=<abort 0x{:08X}>", sub, canopen_client_->last_abort_code());
      }
    }
    logger_.info("OD 0x{:04X}:{}", index, line);
    ++dumped;
  }
  canopen_client_->set_log_level(config_.log_level);
  return dumped;
}

bool Mcp266Controller::drive_m1_speed_manufacturer(int32_t qpps, std::error_code &ec) {
  ec.clear();
  if (config_.mode != Mode::CANOPEN || !canopen_client_) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }
  // Manufacturer velocity mode; the mode display echo is not standardized for
  // negative modes, so don't fail hard on a mismatch.
  if (qpps != 0 && !enable_m1(-2, false, ec)) {
    return false;
  }
  return drive_m1_->set_target_velocity(qpps, ec);
}

bool Mcp266Controller::drive_m1_duty_via_torque(int16_t per_mille, std::error_code &ec) {
  ec.clear();
  if (config_.mode != Mode::CANOPEN || !canopen_client_) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }
  if (per_mille != 0 && !enable_m1(-1, false, ec)) {
    return false;
  }
  return canopen_client_->write_i16(0x6071, 0, per_mille, ec);
}

bool Mcp266Controller::configure_m1_position_range(int32_t min_pos, int32_t max_pos,
                                                   std::error_code &ec) {
  ec.clear();
  if (config_.mode != Mode::CANOPEN || !canopen_client_) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }
  constexpr uint16_t kReadbackIndex = 0x203F; // readable position PID M1 mirror
  // Preserve the currently-configured gains; only the [min, max] clamp (subs
  // 6 and 7) needs to change for position targets to survive.
  std::array<int32_t, 7> values{};
  for (uint8_t sub = 1; sub <= 7; ++sub) {
    values[sub - 1] = canopen_client_->read_i32(kReadbackIndex, sub, ec);
    if (ec) {
      logger_.error("position PID readback 0x{:04X}:{} failed: {}", kReadbackIndex, sub,
                    ec.message());
      return false;
    }
  }
  values[5] = min_pos;
  values[6] = max_pos;

  for (const uint16_t setter : {uint16_t{0x203D}, uint16_t{0x203E}}) {
    bool wrote = true;
    for (uint8_t sub = 1; sub <= 7; ++sub) {
      if (!canopen_client_->write_i32(setter, sub, values[sub - 1], ec)) {
        logger_.warn("position PID write 0x{:04X}:{} rejected: {}", setter, sub, ec.message());
        wrote = false;
        break;
      }
    }
    if (!wrote) {
      continue;
    }
    const int32_t got_min = canopen_client_->read_i32(kReadbackIndex, 6, ec);
    const int32_t got_max = canopen_client_->read_i32(kReadbackIndex, 7, ec);
    if (!ec && got_min == min_pos && got_max == max_pos) {
      logger_.info("M1 position range set to [{}, {}] via 0x{:04X}", min_pos, max_pos, setter);
      return true;
    }
    logger_.warn("0x{:04X} did not change the M1 position range (read [{}, {}])", setter,
                 got_min, got_max);
  }
  logger_.error("failed to configure the M1 position range via 0x203D/0x203E");
  ec = std::make_error_code(std::errc::protocol_error);
  return false;
}

size_t Mcp266Controller::send_test_rpdos(int8_t mode, int32_t target_velocity,
                                         int32_t target_position, int16_t duty_per_mille,
                                         std::error_code &ec) {
  ec.clear();
  if (config_.mode != Mode::CANOPEN || !canopen_client_) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return 0;
  }
  // Value to place in a mapped object slot; unknown objects are zero-filled.
  const auto value_for = [&](uint16_t object) -> uint32_t {
    switch (object & 0xF7FF) { // fold the M2 axis (+0x800) onto M1 objects
    case 0x6040:
      return 0x000F; // controlword: enable operation
    case 0x6060:
      return static_cast<uint32_t>(static_cast<uint8_t>(mode));
    case 0x60FF:
    case 0x6042:
      return static_cast<uint32_t>(target_velocity);
    case 0x607A:
      return static_cast<uint32_t>(target_position);
    case 0x6071:
      return static_cast<uint32_t>(static_cast<uint16_t>(duty_per_mille));
    default:
      return 0;
    }
  };

  size_t sent = 0;
  for (uint16_t rpdo = 0; rpdo < 4; ++rpdo) {
    std::error_code pdo_ec;
    const uint32_t cob = canopen_client_->read_u32(0x1400 + rpdo, 1, pdo_ec);
    if (pdo_ec || (cob & 0x80000000u)) {
      continue; // RPDO missing or disabled
    }
    const uint8_t entries = canopen_client_->read_u8(0x1600 + rpdo, 0, pdo_ec);
    if (pdo_ec) {
      continue;
    }
    std::array<uint8_t, 8> payload{};
    size_t bit_offset = 0;
    std::string desc;
    for (uint8_t sub = 1; sub <= entries && bit_offset < 64; ++sub) {
      const uint32_t entry = canopen_client_->read_u32(0x1600 + rpdo, sub, pdo_ec);
      if (pdo_ec) {
        break;
      }
      const uint16_t object = static_cast<uint16_t>(entry >> 16);
      const uint8_t bits = static_cast<uint8_t>(entry & 0xFF);
      const uint32_t value = value_for(object);
      for (uint8_t b = 0; b + 8 <= bits && bit_offset < 64; b += 8) {
        payload[bit_offset / 8] = static_cast<uint8_t>(value >> b);
        bit_offset += 8;
      }
      desc += fmt::format(" 0x{:04X}:{:02X}/{}={:X}", object, (entry >> 8) & 0xFF, bits, value);
    }
    const size_t dlc = (bit_offset + 7) / 8;
    if (dlc == 0) {
      continue;
    }
    if (!canopen_client_->send_rpdo(cob & 0x7FF, std::span(payload.data(), dlc), ec)) {
      return sent;
    }
    logger_.info("RPDO{} sent on COB 0x{:03X} ({} bytes):{}", rpdo + 1, cob & 0x7FF, dlc, desc);
    ++sent;
  }
  // Cover synchronous transmission types: latch the just-sent PDO data.
  canopen_client_->send_sync(ec);
  return sent;
}

bool Mcp266Controller::read_main_battery_voltage(float &volts, std::error_code &ec) {
  ec.clear();
  if (config_.mode == Mode::PACKET_SERIAL && basicmicro_) {
    return basicmicro_->read_main_battery_voltage(volts, ec);
  }
  // Default placeholder if CANopen custom voltage SDO is not used
  volts = 0.0f;
  return true;
}

bool Mcp266Controller::read_temperature(float &temp_c, std::error_code &ec) {
  ec.clear();
  if (config_.mode == Mode::PACKET_SERIAL && basicmicro_) {
    return basicmicro_->read_temperature(temp_c, ec);
  }
  temp_c = 0.0f;
  return true;
}

bool Mcp266Controller::read_status(uint32_t &status_mask, std::error_code &ec) {
  ec.clear();
  if (config_.mode == Mode::PACKET_SERIAL && basicmicro_) {
    return basicmicro_->read_status(status_mask, ec);
  }
  if (config_.mode == Mode::CANOPEN && canopen_client_) {
    uint16_t sw1 = canopen_client_->read_u16(0x6041, 0, ec);
    if (ec) {
      return false;
    }
    status_mask = static_cast<uint32_t>(sw1);
    return true;
  }
  status_mask = 0;
  return true;
}

} // namespace mib
