#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <system_error>

#include <driver/gpio.h>

#include "base_component.hpp"
#include "canopen_client.hpp"
#include "ds402.hpp"
#include "logger.hpp"
#include "twai.hpp"

namespace mib {

/// \brief Dual-channel controller for a Basicmicro MCP266 (RoboClaw family)
///        motor driver over TWAI (CAN 2.0) using CANopen.
/// \details Both motor channels (M1, M2) are driven symmetrically. Position
///          control uses the standard CiA 402 profile position mode (0x607A +
///          new-set-point handshake), addressing M2 through the +0x800 object
///          offset via espp::Ds402Drive's multi-axis support. The MCP's
///          control-loop parameters are not standard CiA 402 objects; the MCP
///          mirrors its packet-serial command set into the manufacturer region
///          at index 0x2000 + command number, which this class uses to
///          configure the position PID (commands 61-64) and read telemetry
///          (battery 24, temperature 82).
///
///          Position control is validated and reliable. The manufacturer
///          speed/duty command mirror (drive_speed/drive_duty) is implemented
///          but does NOT produce motion on the tested MCP266 firmware (the
///          command is accepted but the velocity generator stays idle); use
///          position mode for motion. See the project README.
class Mcp266Controller : public espp::BaseComponent {
public:
  /// \brief Motor channel selector.
  enum class Axis { M1, M2 };

  struct Config {
    gpio_num_t twai_tx_gpio{GPIO_NUM_17};
    gpio_num_t twai_rx_gpio{GPIO_NUM_16};
    uint32_t baudrate{1000000};
    uint8_t node_id{1}; ///< CANopen Node ID (1-127) of the MCP266.
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::INFO};
  };

  explicit Mcp266Controller(const Config &config);
  ~Mcp266Controller() = default;

  Mcp266Controller(const Mcp266Controller &) = delete;
  Mcp266Controller &operator=(const Mcp266Controller &) = delete;

  /// \brief Initialize the TWAI peripheral and CANopen client, NMT-start the
  ///        node, and clear any latched CiA 402 faults on both axes.
  bool initialize(std::error_code &ec);

  /// \brief Clear any latched CiA 402 faults on both axes.
  bool reset_faults(std::error_code &ec);
  /// \brief Attempt an E-stop reset (mirrored packet-serial command 200 at
  ///        0x20C8). Harmless when nothing is latched.
  bool reset_estop(std::error_code &ec);

  // --- Position control (CiA 402 profile position mode) ---

  /// \brief Configure an axis's position loop for CANopen use: widen the
  ///        min/max position clamp (factory [0, 0] forces every target to
  ///        zero) and ensure a non-zero position P gain. Written via the
  ///        mirrored position-PID setter and verified via the readback. The
  ///        MCP reverts to EEPROM on power-up, so call once per boot before
  ///        commanding position moves.
  bool configure_position_loop(Axis axis, int32_t min_pos, int32_t max_pos, std::error_code &ec);
  /// \brief Set the CiA 402 software position limits (0x607D:1/:2) for an axis.
  bool set_position_limits(Axis axis, int32_t min_pos, int32_t max_pos, std::error_code &ec);
  /// \brief Command a profile-position move: enable the axis, set the motion
  ///        profile, and issue the target with the new-set-point handshake.
  bool move_to_position(Axis axis, int32_t target_position, uint32_t profile_velocity,
                        uint32_t profile_acceleration, uint32_t profile_deceleration,
                        std::error_code &ec);

  // --- Manufacturer speed / duty command mirror ---
  // NOTE: accepted by the drive but inert on the tested MCP266 firmware; kept
  // for completeness and in case a firmware update activates them.

  /// \brief Closed-loop speed via the mirrored packet-serial command (35/36).
  bool drive_speed(Axis axis, int32_t qpps, std::error_code &ec);
  /// \brief Open-loop duty via the mirrored packet-serial command (32/33).
  bool drive_duty(Axis axis, int16_t duty, std::error_code &ec);

  // --- Feedback ---
  bool read_encoder(Axis axis, int32_t &count, std::error_code &ec);
  bool read_speed(Axis axis, int32_t &qpps, std::error_code &ec);
  /// \brief Read the axis CiA 402 statusword (0x6041 / 0x6841).
  bool read_statusword(Axis axis, uint16_t &statusword, std::error_code &ec);

  // --- Device telemetry ---
  bool read_main_battery_voltage(float &volts, std::error_code &ec);
  bool read_temperature(float &temp_c, std::error_code &ec);
  bool read_device_info(std::string &device_name, uint32_t &device_type, std::error_code &ec);

  // --- Diagnostics (for mapping a new device; not needed in normal use) ---
  /// \brief Probe [first_index, last_index] (subindex 0) via SDO upload and
  ///        log every implemented object. \return count found.
  size_t scan_object_dictionary(uint16_t first_index, uint16_t last_index);
  /// \brief Read and log every subindex of each implemented object in the
  ///        range. \return count of objects dumped.
  size_t dump_object_subindices(uint16_t first_index, uint16_t last_index);

private:
  /// Per-axis object addresses and its Ds402Drive helper.
  struct AxisState {
    std::unique_ptr<espp::Ds402Drive> drive; ///< CiA 402 helper (offset 0 / 0x800).
    uint16_t object_offset;                   ///< 0 for M1, 0x800 for M2.
    uint16_t position_pid_set;                ///< Mirrored cmd 61/62 (write-only).
    uint16_t position_pid_read;               ///< Mirrored cmd 63/64.
    uint16_t cmd_duty;                        ///< Mirrored cmd 32/33.
    uint16_t cmd_speed;                       ///< Mirrored cmd 35/36.
    const char *name;
  };

  bool init_twai(std::error_code &ec);
  bool init_canopen(std::error_code &ec);
  AxisState &axis_state(Axis axis) { return axis == Axis::M1 ? m1_ : m2_; }
  /// Write the axis mode of operation (not verified -- the MCP does not echo
  /// the requested mode), clear any fault, and walk to Operation Enabled.
  bool enable(Axis axis, espp::Ds402Drive::OperatingMode mode, std::error_code &ec);

  Config config_;
  std::unique_ptr<espp::Twai> twai_;
  std::unique_ptr<espp::CanopenClient> canopen_client_;
  AxisState m1_{};
  AxisState m2_{};
};

} // namespace mib
