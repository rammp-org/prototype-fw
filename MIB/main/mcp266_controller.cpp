#include <array>
#include <span>
#include <thread>

#include "mcp266_controller.hpp"

using namespace std::chrono_literals;

namespace {
/// The MCP mirrors its packet-serial command set into the manufacturer region
/// of the CANopen object dictionary at index 0x2000 + command number.
/// Confirmed empirically on MCP266 firmware: cmd 61 "Set M1 position PID" =
/// 0x203D, cmd 55 "Read M1 velocity PID" = 0x2037, cmd 200 "E-Stop Reset" =
/// 0x20C8, cmd 24 "Read Main Battery" = 0x2018, cmd 65/67 "Buffered position
/// move" = 0x2041/0x2043 (5/9 subindices matching their parameter counts).
constexpr uint16_t mcp_command_object(uint8_t command) {
  return static_cast<uint16_t>(0x2000 + command);
}
constexpr uint16_t kCmdDriveM1Duty = mcp_command_object(32);
constexpr uint16_t kCmdDriveM2Duty = mcp_command_object(33);
constexpr uint16_t kCmdDriveM1Speed = mcp_command_object(35);
constexpr uint16_t kCmdDriveM2Speed = mcp_command_object(36);
constexpr uint16_t kCmdReadMainBattery = mcp_command_object(24);
constexpr uint16_t kCmdReadTemperature = mcp_command_object(82);
constexpr uint16_t kEStopReset = mcp_command_object(200);   // cmd 200
constexpr uint16_t kEStopLockState = mcp_command_object(202); // cmd 202 (0x20CA)
constexpr uint16_t kM1PositionPidSet = 0x203D;              // cmd 61, write-only
constexpr uint16_t kM1PositionPidRead = 0x203F;             // read-back mirror
} // namespace

namespace mib {

Mcp266Controller::Mcp266Controller(const Config &config)
    : BaseComponent("Mcp266Controller", config.log_level)
    , config_(config) {}

bool Mcp266Controller::initialize(std::error_code &ec) {
  ec.clear();
  logger_.info("Initializing Mcp266Controller on TWAI (TX GPIO {}, RX GPIO {}, Baud {})",
               static_cast<int>(config_.twai_tx_gpio), static_cast<int>(config_.twai_rx_gpio),
               config_.baudrate);

  if (!init_twai(ec)) {
    logger_.error("Failed to initialize TWAI transport: {}", ec.message());
    return false;
  }
  if (!init_canopen(ec)) {
    logger_.error("Failed to initialize CANopen mode: {}", ec.message());
    return false;
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
            if (canopen_client_) {
              canopen_client_->process_frame(espp::CanopenClient::CanFrame{
                  .id = msg.id,
                  .extended = msg.extended,
                  .rtr = msg.rtr,
                  .dlc = msg.dlc,
                  .data = msg.data,
              });
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

// --- Mode Control ---

bool Mcp266Controller::set_mode_m1(int8_t mode, std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    return false;
  }
  if (canopen_client_->write_i8(kAxisM1.mode, 0, mode, ec)) {
    m1_mode_ = mode;
    return true;
  }
  return false;
}

int8_t Mcp266Controller::get_mode_m1(std::error_code &ec) {
  ec.clear();
  return canopen_client_ ? canopen_client_->read_i8(kAxisM1.mode_display, 0, ec) : 0;
}

bool Mcp266Controller::set_mode_m2(int8_t mode, std::error_code &ec) {
  ec.clear();
  return canopen_client_ && canopen_client_->write_i8(kAxisM2.mode, 0, mode, ec);
}

int8_t Mcp266Controller::get_mode_m2(std::error_code &ec) {
  ec.clear();
  return canopen_client_ ? canopen_client_->read_i8(kAxisM2.mode_display, 0, ec) : 0;
}

// --- CiA 402 state machine ---

bool Mcp266Controller::reset_axis_fault(const AxisObjects &axis, std::error_code &ec) {
  const uint16_t sw = canopen_client_->read_u16(axis.statusword, 0, ec);
  if (ec) {
    logger_.error("{}: statusword read before fault reset failed: {}", axis.name, ec.message());
    return false;
  }
  if ((sw & 0x004F) != 0x0008) {
    return true; // not in Fault
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
  if (!canopen_client_) {
    return true;
  }
  if (!reset_axis_fault(kAxisM1, ec)) {
    return false;
  }
  return reset_axis_fault(kAxisM2, ec);
}

bool Mcp266Controller::enable_m1(int8_t mode, bool verify_mode, std::error_code &ec) {
  ec.clear();
  if (!canopen_client_->write_i8(kAxisM1.mode, 0, mode, ec)) {
    logger_.error("M1: failed to set mode {}: {}", mode, ec.message());
    return false;
  }
  m1_mode_ = mode;
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

// --- Motor Duty Control (manufacturer command mirror) ---

bool Mcp266Controller::drive_m1_duty(int16_t duty, std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    return false;
  }
  // The power stage is only live in Operation Enabled; profile velocity mode
  // (3) reaches it without the position loop holding a target. Enable there,
  // then issue the mirrored duty command (packet-serial cmd 32).
  if (duty != 0 &&
      !enable_m1(static_cast<int8_t>(espp::Ds402Drive::OperatingMode::ProfileVelocity), false,
                 ec)) {
    return false;
  }
  return canopen_client_->write_i16(kCmdDriveM1Duty, 0, duty, ec);
}

bool Mcp266Controller::drive_m2_duty(int16_t duty, std::error_code &ec) {
  ec.clear();
  return canopen_client_ && canopen_client_->write_i16(kCmdDriveM2Duty, 0, duty, ec);
}

bool Mcp266Controller::drive_duty(int16_t duty_m1, int16_t duty_m2, std::error_code &ec) {
  ec.clear();
  return drive_m1_duty(duty_m1, ec) && drive_m2_duty(duty_m2, ec);
}

// --- Speed Control (manufacturer command mirror) ---

bool Mcp266Controller::drive_m1_speed(int32_t qpps, std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    return false;
  }
  // The mirrored speed command only turns the motor when the power stage is
  // live, i.e. Operation Enabled. Profile velocity mode (3) reaches Operation
  // Enabled without the position loop holding a target (unlike position mode,
  // which servos to its last setpoint and overrides the raw command). Setting
  // the mode to -1 does NOT idle the drive -- it drops it to Switch On
  // Disabled (power stage off), so never use that to "release" a hold.
  // With the drive enabled here, issue the mirrored packet-serial command 35
  // ("Drive M1 With Signed Speed"); the standard 0x60FF target is inert on
  // this firmware. Requires a tuned velocity PID.
  if (qpps != 0 &&
      !enable_m1(static_cast<int8_t>(espp::Ds402Drive::OperatingMode::ProfileVelocity), false,
                 ec)) {
    return false;
  }
  return canopen_client_->write_i32(kCmdDriveM1Speed, 0, qpps, ec);
}

bool Mcp266Controller::drive_m2_speed(int32_t qpps, std::error_code &ec) {
  ec.clear();
  return canopen_client_ && canopen_client_->write_i32(kCmdDriveM2Speed, 0, qpps, ec);
}

bool Mcp266Controller::drive_speed(int32_t qpps_m1, int32_t qpps_m2, std::error_code &ec) {
  ec.clear();
  return drive_m1_speed(qpps_m1, ec) && drive_m2_speed(qpps_m2, ec);
}

bool Mcp266Controller::stop_motors(std::error_code &ec) {
  ec.clear();
  return drive_speed(0, 0, ec);
}

// --- Position Control (CiA 402 profile position mode) ---

bool Mcp266Controller::set_m1_position_limits(int32_t minimum_position, int32_t maximum_position,
                                              std::error_code &ec) {
  ec.clear();
  if (minimum_position > maximum_position) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  if (!canopen_client_) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }
  return canopen_client_->write_i32(0x607B, 1, minimum_position, ec) &&
         canopen_client_->write_i32(0x607B, 2, maximum_position, ec);
}

bool Mcp266Controller::get_m1_position_limits(int32_t &minimum_position, int32_t &maximum_position,
                                              std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
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
  if (!canopen_client_) {
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

bool Mcp266Controller::read_position_pid_m1_raw(std::array<int32_t, 7> &values,
                                                std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }
  for (uint8_t sub = 1; sub <= 7; ++sub) {
    values[sub - 1] = canopen_client_->read_i32(kM1PositionPidRead, sub, ec);
    if (ec) {
      return false;
    }
  }
  return true;
}

bool Mcp266Controller::write_position_pid_m1_raw(const std::array<int32_t, 7> &values,
                                                 std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }
  for (uint8_t sub = 1; sub <= 7; ++sub) {
    if (!canopen_client_->write_i32(kM1PositionPidSet, sub, values[sub - 1], ec)) {
      logger_.error("position PID write 0x{:04X}:{} rejected: {}", kM1PositionPidSet, sub,
                    ec.message());
      return false;
    }
  }
  std::array<int32_t, 7> readback{};
  if (!read_position_pid_m1_raw(readback, ec)) {
    return false;
  }
  if (readback != values) {
    logger_.error("position PID readback mismatch after write");
    ec = std::make_error_code(std::errc::protocol_error);
    return false;
  }
  return true;
}

bool Mcp266Controller::configure_m1_position_loop(int32_t min_pos, int32_t max_pos,
                                                  std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }
  std::array<int32_t, 7> pid{};
  if (!read_position_pid_m1_raw(pid, ec)) {
    logger_.error("could not read M1 position PID: {}", ec.message());
    return false;
  }
  // The readback (0x203F) and the setter (0x203D) use DIFFERENT field orders:
  // the readback reports [P, I, D, MaxI, Deadzone, MinPos, MaxPos] while the
  // setter takes the packet-serial write order [D, P, I, ...]. So the P gain
  // we read in sub 1 must be written into the setter's sub 2, and the min/max
  // clamp (readback subs 6/7) maps to the same setter subs. Build the setter
  // record explicitly rather than writing the readback array straight back
  // (which would drop P into the D slot and zero P).
  const int32_t p_gain = pid[0] != 0 ? pid[0] : 0x3C83; // readback sub 1 = P
  const int32_t i_gain = pid[1];                        // readback sub 2 = I
  const int32_t d_gain = pid[2];                        // readback sub 3 = D
  const int32_t max_i = pid[3];
  const int32_t deadzone = pid[4];
  const std::array<int32_t, 7> setter{d_gain, p_gain, i_gain, max_i, deadzone, min_pos, max_pos};
  for (uint8_t sub = 1; sub <= 7; ++sub) {
    if (!canopen_client_->write_i32(kM1PositionPidSet, sub, setter[sub - 1], ec)) {
      logger_.error("position PID write 0x{:04X}:{} rejected: {}", kM1PositionPidSet, sub,
                    ec.message());
      return false;
    }
  }
  // Verify via the readback's own field order (min/max are subs 6/7 there too).
  const int32_t got_min = canopen_client_->read_i32(kM1PositionPidRead, 6, ec);
  const int32_t got_max = canopen_client_->read_i32(kM1PositionPidRead, 7, ec);
  if (ec || got_min != min_pos || got_max != max_pos) {
    logger_.error("M1 position clamp did not take (read [{}, {}], wanted [{}, {}])", got_min,
                  got_max, min_pos, max_pos);
    ec = std::make_error_code(std::errc::protocol_error);
    return false;
  }
  logger_.info("M1 position loop configured: P={} clamp=[{}, {}]", p_gain, min_pos, max_pos);
  return true;
}

bool Mcp266Controller::try_estop_reset(std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }
  // Packet-serial command 200 (E-Stop Reset) takes no payload, so the SDO
  // scalar size is unknown; probe the common widths.
  bool reset_ok = canopen_client_->write_u8(kEStopReset, 0, 1, ec);
  if (!reset_ok) {
    reset_ok = canopen_client_->write_u32(kEStopReset, 0, 1, ec);
  }
  std::error_code lock_ec;
  const uint8_t lock = canopen_client_->read_u8(kEStopLockState, 0, lock_ec);
  logger_.info("E-stop reset {}; lock state = {}", reset_ok ? "accepted" : "rejected",
               lock_ec ? -1 : static_cast<int>(lock));
  return reset_ok;
}

bool Mcp266Controller::map_rpdo1_for_position(std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }
  const uint32_t cob = canopen_client_->read_u32(0x1400, 1, ec);
  if (ec) {
    return false;
  }
  // Standard remap sequence: disable the PDO, clear the mapping count, write
  // the entries ((index << 16) | (sub << 8) | bit length), restore the count,
  // re-enable the PDO.
  if (!canopen_client_->write_u32(0x1400, 1, cob | 0x80000000u, ec) ||
      !canopen_client_->write_u8(0x1600, 0, 0, ec) ||
      !canopen_client_->write_u32(0x1600, 1, 0x60400010u, ec) ||
      !canopen_client_->write_u32(0x1600, 2, 0x607A0020u, ec) ||
      !canopen_client_->write_u8(0x1600, 0, 2, ec) ||
      !canopen_client_->write_u32(0x1400, 1, cob & ~0x80000000u, ec)) {
    logger_.warn("RPDO1 remap rejected: {} (abort 0x{:08X})", ec.message(),
                 canopen_client_->last_abort_code());
    return false;
  }
  rpdo1_cob_ = cob & 0x7FFu;
  logger_.info("RPDO1 mapped to controlword + target position on COB 0x{:03X}", rpdo1_cob_);
  return true;
}

bool Mcp266Controller::send_position_rpdo(uint16_t controlword, int32_t target,
                                          std::error_code &ec) {
  ec.clear();
  if (!canopen_client_ || rpdo1_cob_ == 0) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }
  const std::array<uint8_t, 6> payload{
      static_cast<uint8_t>(controlword),  static_cast<uint8_t>(controlword >> 8),
      static_cast<uint8_t>(target),       static_cast<uint8_t>(target >> 8),
      static_cast<uint8_t>(target >> 16), static_cast<uint8_t>(target >> 24)};
  if (!canopen_client_->send_rpdo(rpdo1_cob_, payload, ec)) {
    return false;
  }
  return canopen_client_->send_sync(ec);
}

// --- Encoder Readback ---

bool Mcp266Controller::read_encoder_m1(int32_t &count, uint8_t &status, std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    return false;
  }
  count = canopen_client_->read_i32(0x6064, 0, ec);
  status = ec ? 0 : 1;
  return !ec;
}

bool Mcp266Controller::read_encoder_m2(int32_t &count, uint8_t &status, std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    return false;
  }
  count = canopen_client_->read_i32(0x6864, 0, ec);
  status = ec ? 0 : 1;
  return !ec;
}

bool Mcp266Controller::read_encoders(int32_t &count_m1, int32_t &count_m2, std::error_code &ec) {
  ec.clear();
  uint8_t st1 = 0, st2 = 0;
  return read_encoder_m1(count_m1, st1, ec) && read_encoder_m2(count_m2, st2, ec);
}

bool Mcp266Controller::read_position_demand_m1(int32_t &demand, std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }
  demand = canopen_client_->read_i32(0x6062, 0, ec);
  return !ec;
}

bool Mcp266Controller::read_velocity_demand_m1(int32_t &demand, std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }
  demand = canopen_client_->read_i32(0x606B, 0, ec);
  return !ec;
}

bool Mcp266Controller::read_speed_m1(int32_t &qpps, uint8_t &status, std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    return false;
  }
  qpps = canopen_client_->read_i32(0x606C, 0, ec);
  status = ec ? 0 : 1;
  return !ec;
}

bool Mcp266Controller::read_speed_m2(int32_t &qpps, uint8_t &status, std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    return false;
  }
  qpps = canopen_client_->read_i32(0x686C, 0, ec);
  status = ec ? 0 : 1;
  return !ec;
}

// --- Telemetry & Diagnostics ---

bool Mcp266Controller::read_device_info(std::string &device_name, uint32_t &device_type,
                                        std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    return false;
  }
  device_type = canopen_client_->read_u32(0x1000, 0, ec);
  if (ec) {
    return false;
  }
  device_name = canopen_client_->read_string(0x1008, 0, ec);
  return !ec;
}

bool Mcp266Controller::read_object_dictionary(std::string &eds, std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }
  eds = canopen_client_->read_string(0x1021, 0, ec);
  return !ec;
}

size_t Mcp266Controller::scan_object_dictionary(uint16_t first_index, uint16_t last_index) {
  if (!canopen_client_) {
    return 0;
  }
  size_t found = 0;
  // Probing for existence produces an SDO abort for every missing object;
  // mute the client's per-abort error logging for the duration of the scan.
  canopen_client_->set_log_level(espp::Logger::Verbosity::NONE);
  for (uint32_t index = first_index; index <= last_index; ++index) {
    std::error_code ec;
    std::array<uint8_t, 4> data{};
    const size_t len = canopen_client_->sdo_upload(static_cast<uint16_t>(index), 0, data, ec);
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
  if (!canopen_client_) {
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

bool Mcp266Controller::read_main_battery_voltage(float &volts, std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    volts = 0.0f;
    return false;
  }
  // Packet-serial command 24 mirrored into the object dictionary (tenths of a volt).
  const uint16_t tenths = canopen_client_->read_u16(kCmdReadMainBattery, 0, ec);
  volts = static_cast<float>(tenths) / 10.0f;
  return !ec;
}

bool Mcp266Controller::read_temperature(float &temp_c, std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    temp_c = 0.0f;
    return false;
  }
  // Packet-serial command 82 mirrored into the object dictionary (tenths of a degree C).
  const uint16_t tenths = canopen_client_->read_u16(kCmdReadTemperature, 0, ec);
  temp_c = static_cast<float>(tenths) / 10.0f;
  return !ec;
}

bool Mcp266Controller::read_status(uint32_t &status_mask, std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    status_mask = 0;
    return false;
  }
  const uint16_t sw1 = canopen_client_->read_u16(0x6041, 0, ec);
  if (ec) {
    return false;
  }
  status_mask = static_cast<uint32_t>(sw1);
  return true;
}

} // namespace mib
