#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <system_error>

#include <driver/gpio.h>

#include "canopen_client.hpp"
#include "ds402.hpp"
#include "logger.hpp"
#include "twai.hpp"

namespace mib {

/// \brief Controller for a Basicmicro MCP266 (RoboClaw family) dual-channel
///        motor driver over TWAI (CAN 2.0) using CANopen.
/// \details Position control uses the standard CiA 402 profile position mode
///          (0x607A + new-set-point handshake), which this firmware genuinely
///          implements. Velocity and duty control use the manufacturer
///          command mirror: the MCP maps its packet-serial command set into
///          the object dictionary at index 0x2000 + command number (the
///          standard profile-velocity mode is accepted but inert). The
///          position loop's min/max clamp and P gain must be configured
///          (see configure_m1_position_loop()) -- the factory record clamps
///          every position target to [0, 0].
class Mcp266Controller : public espp::BaseComponent {
public:
  struct Config {
    gpio_num_t twai_tx_gpio{GPIO_NUM_16};
    gpio_num_t twai_rx_gpio{GPIO_NUM_17};
    uint32_t baudrate{1000000};
    uint8_t node_id{1}; ///< CANopen Node ID (1-127) of the MCP266.
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::INFO};
  };

  explicit Mcp266Controller(const Config &config);
  ~Mcp266Controller() = default;

  Mcp266Controller(const Mcp266Controller &) = delete;
  Mcp266Controller &operator=(const Mcp266Controller &) = delete;

  /// \brief Initialize the TWAI peripheral and the CANopen client, NMT-start
  ///        the node, and clear any latched CiA 402 faults.
  bool initialize(std::error_code &ec);

  /// \brief Clear any latched CiA 402 faults on both MCP266 motor axes.
  bool reset_faults(std::error_code &ec);

  // --- Mode Control ---
  bool set_mode_m1(int8_t mode, std::error_code &ec);
  int8_t get_mode_m1(std::error_code &ec);
  bool set_mode_m2(int8_t mode, std::error_code &ec);
  int8_t get_mode_m2(std::error_code &ec);

  // --- Motor Duty Control (manufacturer command mirror, cmd 32/33) ---
  bool drive_m1_duty(int16_t duty, std::error_code &ec);
  bool drive_m2_duty(int16_t duty, std::error_code &ec);
  bool drive_duty(int16_t duty_m1, int16_t duty_m2, std::error_code &ec);

  // --- Closed-Loop Speed Control (manufacturer command mirror, cmd 35/36) ---
  bool drive_m1_speed(int32_t qpps, std::error_code &ec);
  bool drive_m2_speed(int32_t qpps, std::error_code &ec);
  bool drive_speed(int32_t qpps_m1, int32_t qpps_m2, std::error_code &ec);

  // --- Position Control (CiA 402 profile position mode) ---
  bool set_m1_position_limits(int32_t minimum_position, int32_t maximum_position,
                              std::error_code &ec);
  bool get_m1_position_limits(int32_t &minimum_position, int32_t &maximum_position,
                              std::error_code &ec);
  bool move_m1_to_position(int32_t target_position, uint32_t profile_velocity,
                           uint32_t profile_acceleration, uint32_t profile_deceleration,
                           std::error_code &ec);

  /// \brief Configure the M1 position loop for CANopen use: widen the min/max
  ///        position clamp (factory [0, 0] forces every target to zero) and
  ///        ensure the P gain is non-zero (the record's leading field is the
  ///        D gain on this firmware, so a factory record can have P = 0).
  ///        Written via the mirrored setter (0x203D) and verified via the
  ///        readback (0x203F). Call once after initialize().
  bool configure_m1_position_loop(int32_t min_pos, int32_t max_pos, std::error_code &ec);

  /// \brief Read the raw 7-value M1 position PID record (0x203F subs 1-7).
  bool read_position_pid_m1_raw(std::array<int32_t, 7> &values, std::error_code &ec);
  /// \brief Write the raw 7-value M1 position PID record via 0x203D and
  ///        verify it via the 0x203F readback.
  bool write_position_pid_m1_raw(const std::array<int32_t, 7> &values, std::error_code &ec);

  /// \brief Attempt an E-stop reset (packet-serial command 200 mirrored at
  ///        write-only 0x20C8) and log the E-stop lock state (0x20CA).
  ///        Harmless when no e-stop is latched.
  bool try_estop_reset(std::error_code &ec);

  /// \brief Remap RPDO1 to [controlword (16 bit), target position (32 bit)]
  ///        using the standard disable -> clear -> map -> enable sequence.
  ///        The MCP's default PDO mappings are empty.
  bool map_rpdo1_for_position(std::error_code &ec);
  /// \brief Send one RPDO1 frame with the given controlword and target
  ///        position (requires map_rpdo1_for_position), followed by a SYNC.
  bool send_position_rpdo(uint16_t controlword, int32_t target, std::error_code &ec);

  // --- Stop Motors ---
  bool stop_motors(std::error_code &ec);

  // --- Encoder Readback ---
  bool read_encoder_m1(int32_t &count, uint8_t &status, std::error_code &ec);
  bool read_encoder_m2(int32_t &count, uint8_t &status, std::error_code &ec);
  bool read_encoders(int32_t &count_m1, int32_t &count_m2, std::error_code &ec);

  bool read_speed_m1(int32_t &qpps, uint8_t &status, std::error_code &ec);
  bool read_speed_m2(int32_t &qpps, uint8_t &status, std::error_code &ec);

  /// \brief Read the M1 position demand value (0x6062): the profile
  ///        generator's instantaneous position command.
  bool read_position_demand_m1(int32_t &demand, std::error_code &ec);
  /// \brief Read the M1 velocity demand value (0x606B).
  bool read_velocity_demand_m1(int32_t &demand, std::error_code &ec);

  // --- Telemetry & Diagnostics ---
  bool read_device_info(std::string &device_name, uint32_t &device_type, std::error_code &ec);
  /// \brief Read the optional CiA 306 EDS object (0x1021:00). The MCP266 does
  ///        not implement it; kept for other CANopen nodes.
  bool read_object_dictionary(std::string &eds, std::error_code &ec);
  /// \brief Main battery voltage via the mirrored packet-serial command 24.
  bool read_main_battery_voltage(float &volts, std::error_code &ec);
  /// \brief Board temperature via the mirrored packet-serial command 82.
  bool read_temperature(float &temp_c, std::error_code &ec);
  /// \brief Read the M1 CiA 402 statusword (0x6041).
  bool read_status(uint32_t &status_mask, std::error_code &ec);

  /// \brief Probe every object index in [first_index, last_index] (subindex 0)
  ///        via SDO upload and log the ones the node implements.
  /// \return The number of objects found.
  size_t scan_object_dictionary(uint16_t first_index, uint16_t last_index);
  /// \brief For every implemented object in [first_index, last_index], read
  ///        and log its subindex values.
  /// \return The number of objects dumped.
  size_t dump_object_subindices(uint16_t first_index, uint16_t last_index);

private:
  bool init_twai(std::error_code &ec);
  bool init_canopen(std::error_code &ec);

  /// Object dictionary base for one axis: M1 uses 0x60xx, M2 uses 0x68xx.
  struct AxisObjects {
    uint16_t controlword;
    uint16_t statusword;
    uint16_t mode;
    uint16_t mode_display;
    uint16_t target;
    const char *name;
  };

  static constexpr AxisObjects kAxisM1{0x6040, 0x6041, 0x6060, 0x6061, 0x60FF, "M1"};
  static constexpr AxisObjects kAxisM2{0x6840, 0x6841, 0x6860, 0x6861, 0x68FF, "M2"};

  /// Select the M1 mode (optionally verified against 0x6061), clear any
  /// latched fault, and walk the drive to Operation Enabled.
  bool enable_m1(int8_t mode, bool verify_mode, std::error_code &ec);
  bool reset_axis_fault(const AxisObjects &axis, std::error_code &ec);

  Config config_;
  std::unique_ptr<espp::Twai> twai_;
  std::unique_ptr<espp::CanopenClient> canopen_client_;
  std::unique_ptr<espp::Ds402Drive> drive_m1_;

  // Last mode of operation commanded to M1 (0x6060); tracked so speed/duty
  // commands can release an active position hold without an extra SDO read.
  int8_t m1_mode_{0};

  // COB-ID of RPDO1 once map_rpdo1_for_position() succeeds (0 = unmapped).
  uint32_t rpdo1_cob_{0};
};

} // namespace mib
