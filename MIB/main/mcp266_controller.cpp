#include <array>
#include <thread>

#include "mcp266_controller.hpp"

using namespace std::chrono_literals;

namespace {
// The MCP mirrors its packet-serial command set into the manufacturer region
// of the CANopen object dictionary at index 0x2000 + command number (verified
// on MCP266 firmware: cmd 61 set M1 position PID = 0x203D, cmd 55 read M1
// velocity PID = 0x2037, cmd 200 e-stop reset = 0x20C8, cmd 24 main battery =
// 0x2018). Position-PID and drive commands follow the same mapping per axis.
constexpr uint16_t mcp_command_object(uint8_t command) {
  return static_cast<uint16_t>(0x2000 + command);
}
constexpr uint16_t kEStopReset = mcp_command_object(200);
constexpr uint16_t kReadMainBattery = mcp_command_object(24);
constexpr uint16_t kReadTemperature = mcp_command_object(82);
} // namespace

namespace mib {

Mcp266Controller::Mcp266Controller(const Config &config)
    : BaseComponent("Mcp266Controller", config.log_level)
    , config_(config) {
  // M1 objects at the standard indices; M2 mirrors them at +0x800 (CiA 402)
  // and its manufacturer commands are the M2 variants (cmd n+1).
  m1_ = {nullptr, 0x000, mcp_command_object(61), mcp_command_object(63), mcp_command_object(32),
         mcp_command_object(35), "M1"};
  m2_ = {nullptr, 0x800, mcp_command_object(62), mcp_command_object(64), mcp_command_object(33),
         mcp_command_object(36), "M2"};
}

bool Mcp266Controller::initialize(std::error_code &ec) {
  ec.clear();
  logger_.info("Initializing on TWAI (TX GPIO {}, RX GPIO {}, Baud {}, Node {})",
               static_cast<int>(config_.twai_tx_gpio), static_cast<int>(config_.twai_rx_gpio),
               config_.baudrate, config_.node_id);
  if (!init_twai(ec)) {
    logger_.error("Failed to initialize TWAI transport: {}", ec.message());
    return false;
  }
  if (!init_canopen(ec)) {
    logger_.error("Failed to initialize CANopen: {}", ec.message());
    return false;
  }
  logger_.info("Mcp266Controller initialized");
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

  const espp::Ds402Drive::Config m1_cfg{
      .state_timeout = 1s, .poll_period = 25ms, .object_offset = m1_.object_offset,
      .log_level = config_.log_level};
  const espp::Ds402Drive::Config m2_cfg{
      .state_timeout = 1s, .poll_period = 25ms, .object_offset = m2_.object_offset,
      .log_level = config_.log_level};
  m1_.drive = std::make_unique<espp::Ds402Drive>(*canopen_client_, m1_cfg);
  m2_.drive = std::make_unique<espp::Ds402Drive>(*canopen_client_, m2_cfg);

  if (!canopen_client_->nmt_start(ec)) {
    logger_.warn("NMT start failed: {}", ec.message());
  }
  if (!reset_faults(ec)) {
    logger_.warn("Fault reset did not complete: {}", ec.message());
  }
  return true;
}

bool Mcp266Controller::reset_faults(std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    return true;
  }
  for (AxisState *a : {&m1_, &m2_}) {
    std::error_code state_ec;
    const auto state = a->drive->get_state(state_ec);
    if (!state_ec &&
        (state == espp::Ds402Drive::State::Fault ||
         state == espp::Ds402Drive::State::FaultReactionActive)) {
      logger_.warn("{}: clearing fault", a->name);
      if (!a->drive->fault_reset(ec)) {
        return false;
      }
    }
  }
  return true;
}

bool Mcp266Controller::reset_estop(std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }
  // Command 200 takes no payload, so the SDO scalar width is unknown; try the
  // common ones.
  bool ok = canopen_client_->write_u8(kEStopReset, 0, 1, ec);
  if (!ok) {
    ok = canopen_client_->write_u32(kEStopReset, 0, 1, ec);
  }
  logger_.info("E-stop reset {}", ok ? "accepted" : "rejected");
  return ok;
}

bool Mcp266Controller::enable(Axis axis, espp::Ds402Drive::OperatingMode mode,
                              std::error_code &ec) {
  ec.clear();
  AxisState &a = axis_state(axis);
  // Write the mode of operation directly (0x6060 + offset). The MCP does not
  // echo the requested mode in 0x6061, so Ds402Drive::set_mode() -- which
  // verifies the display -- would time out; write it and proceed.
  const uint16_t mode_obj = static_cast<uint16_t>(0x6060 + a.object_offset);
  if (!canopen_client_->write_i8(mode_obj, 0, static_cast<int8_t>(mode), ec)) {
    logger_.error("{}: failed to set mode: {}", a.name, ec.message());
    return false;
  }
  std::this_thread::sleep_for(25ms);
  const auto state = a.drive->get_state(ec);
  if (ec) {
    return false;
  }
  if (state == espp::Ds402Drive::State::Fault ||
      state == espp::Ds402Drive::State::FaultReactionActive) {
    if (!a.drive->fault_reset(ec)) {
      logger_.error("{}: fault reset failed: {}", a.name, ec.message());
      return false;
    }
  }
  return a.drive->enable_operation(ec);
}

bool Mcp266Controller::configure_position_loop(Axis axis, int32_t min_pos, int32_t max_pos,
                                               std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }
  AxisState &a = axis_state(axis);
  // The readback (cmd 63/64) reports [P, I, D, MaxI, Deadzone, MinPos, MaxPos]
  // but the setter (cmd 61/62) takes the packet-serial write order
  // [D, P, I, ...]. Read the current record, then build the setter record in
  // its own order: P must go to setter sub 2 (not sub 1), else it lands in the
  // D slot and the loop runs with P = 0 (no output). The factory MinPos/MaxPos
  // of [0, 0] clamps every target to zero, so widen it here.
  std::array<int32_t, 7> read{};
  for (uint8_t sub = 1; sub <= 7; ++sub) {
    read[sub - 1] = canopen_client_->read_i32(a.position_pid_read, sub, ec);
    if (ec) {
      logger_.error("{}: position PID read 0x{:04X}:{} failed: {}", a.name, a.position_pid_read,
                    sub, ec.message());
      return false;
    }
  }
  const int32_t p_gain = read[0] != 0 ? read[0] : 0x3C83; // readback sub 1 = P
  const int32_t i_gain = read[1];                         // readback sub 2 = I
  const int32_t d_gain = read[2];                         // readback sub 3 = D
  const int32_t max_i = read[3];
  const int32_t deadzone = read[4];
  const std::array<int32_t, 7> setter{d_gain, p_gain, i_gain, max_i, deadzone, min_pos, max_pos};
  for (uint8_t sub = 1; sub <= 7; ++sub) {
    if (!canopen_client_->write_i32(a.position_pid_set, sub, setter[sub - 1], ec)) {
      logger_.error("{}: position PID write 0x{:04X}:{} rejected: {}", a.name, a.position_pid_set,
                    sub, ec.message());
      return false;
    }
  }
  // Verify via the readback's field order (min/max are subs 6/7 there too).
  const int32_t got_min = canopen_client_->read_i32(a.position_pid_read, 6, ec);
  const int32_t got_max = canopen_client_->read_i32(a.position_pid_read, 7, ec);
  if (ec || got_min != min_pos || got_max != max_pos) {
    logger_.error("{}: position clamp did not take (read [{}, {}], wanted [{}, {}])", a.name,
                  got_min, got_max, min_pos, max_pos);
    ec = std::make_error_code(std::errc::protocol_error);
    return false;
  }
  logger_.info("{}: position loop configured (P={}, clamp=[{}, {}])", a.name, p_gain, min_pos,
               max_pos);
  return true;
}

bool Mcp266Controller::set_position_limits(Axis axis, int32_t min_pos, int32_t max_pos,
                                           std::error_code &ec) {
  ec.clear();
  if (!canopen_client_ || min_pos > max_pos) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  const uint16_t obj = static_cast<uint16_t>(0x607D + axis_state(axis).object_offset);
  return canopen_client_->write_i32(obj, 1, min_pos, ec) &&
         canopen_client_->write_i32(obj, 2, max_pos, ec);
}

bool Mcp266Controller::move_to_position(Axis axis, int32_t target_position,
                                        uint32_t profile_velocity, uint32_t profile_acceleration,
                                        uint32_t profile_deceleration, std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }
  AxisState &a = axis_state(axis);
  if (!enable(axis, espp::Ds402Drive::OperatingMode::ProfilePosition, ec)) {
    return false;
  }
  if (!(a.drive->set_profile_acceleration(profile_acceleration, ec) &&
        a.drive->set_profile_deceleration(profile_deceleration, ec) &&
        a.drive->set_profile_velocity(profile_velocity, ec))) {
    return false;
  }
  // Writes the target and runs the new-set-point handshake (controlword bit 4
  // with change-set-immediately, wait for statusword bit 12, release bit 4).
  return a.drive->set_target_position(target_position, ec);
}

bool Mcp266Controller::drive_speed(Axis axis, int32_t qpps, std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    return false;
  }
  AxisState &a = axis_state(axis);
  if (qpps != 0 && !enable(axis, espp::Ds402Drive::OperatingMode::ProfileVelocity, ec)) {
    return false;
  }
  return canopen_client_->write_i32(a.cmd_speed, 0, qpps, ec);
}

bool Mcp266Controller::drive_duty(Axis axis, int16_t duty, std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    return false;
  }
  AxisState &a = axis_state(axis);
  if (duty != 0 && !enable(axis, espp::Ds402Drive::OperatingMode::ProfileVelocity, ec)) {
    return false;
  }
  return canopen_client_->write_i16(a.cmd_duty, 0, duty, ec);
}

bool Mcp266Controller::read_encoder(Axis axis, int32_t &count, std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    return false;
  }
  count = axis_state(axis).drive->get_position_actual(ec);
  return !ec;
}

bool Mcp266Controller::read_speed(Axis axis, int32_t &qpps, std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    return false;
  }
  qpps = axis_state(axis).drive->get_velocity_actual(ec);
  return !ec;
}

bool Mcp266Controller::read_statusword(Axis axis, uint16_t &statusword, std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    return false;
  }
  statusword = axis_state(axis).drive->get_statusword(ec);
  return !ec;
}

bool Mcp266Controller::read_main_battery_voltage(float &volts, std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    volts = 0.0f;
    return false;
  }
  volts = static_cast<float>(canopen_client_->read_u16(kReadMainBattery, 0, ec)) / 10.0f;
  return !ec;
}

bool Mcp266Controller::read_temperature(float &temp_c, std::error_code &ec) {
  ec.clear();
  if (!canopen_client_) {
    temp_c = 0.0f;
    return false;
  }
  temp_c = static_cast<float>(canopen_client_->read_u16(kReadTemperature, 0, ec)) / 10.0f;
  return !ec;
}

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

size_t Mcp266Controller::scan_object_dictionary(uint16_t first_index, uint16_t last_index) {
  if (!canopen_client_) {
    return 0;
  }
  size_t found = 0;
  // Probing missing objects yields an SDO abort each; mute per-abort logging.
  canopen_client_->set_log_level(espp::Logger::Verbosity::NONE);
  for (uint32_t index = first_index; index <= last_index; ++index) {
    std::error_code ec;
    std::array<uint8_t, 4> data{};
    const size_t len = canopen_client_->sdo_upload(static_cast<uint16_t>(index), 0, data, ec);
    if (!ec) {
      logger_.info("OD 0x{:04X}:00 = 0x{:08X} ({} bytes)", index,
                   espp::detail::canopen::get_le(data.data(), len), len);
      ++found;
    } else if (ec == std::errc::protocol_error &&
               canopen_client_->last_abort_code() != 0x06020000) {
      // Any abort other than "object does not exist" still proves existence.
      logger_.info("OD 0x{:04X}:00 exists (abort 0x{:08X})", index,
                   canopen_client_->last_abort_code());
      ++found;
    } else if (ec == std::errc::timed_out) {
      logger_.warn("OD scan aborted at 0x{:04X}: node stopped responding", index);
      break;
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
    // Records report their subindex count in subindex 0; a write-only sub 0
    // aborts, so fall back to a fixed probe count.
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

} // namespace mib
