#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <span>
#include <system_error>

#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "basicmicro.hpp"
#include "canopen_client.hpp"
#include "ds402.hpp"
#include "logger.hpp"
#include "twai.hpp"

namespace mib {

/// \brief Controller class for Basicmicro MCP266 (RoboClaw) dual-channel motor driver over TWAI (CAN bus).
class Mcp266Controller : public espp::BaseComponent {
public:
  enum class Mode {
    CANOPEN,       ///< CANopen (CiA 301 / CiA 402 profile)
    PACKET_SERIAL  ///< Packet Serial protocol framed over CAN/TWAI
  };

  struct Config {
    gpio_num_t twai_tx_gpio{GPIO_NUM_16};
    gpio_num_t twai_rx_gpio{GPIO_NUM_17};
    uint32_t baudrate{1000000};
    Mode mode{Mode::CANOPEN};
    uint8_t node_id{1};                  ///< CANopen Node ID (1-127) for the MCP266 device
    uint8_t packet_serial_address{0x80}; ///< Packet serial address (0x80 - 0x87)
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::INFO};
  };

  explicit Mcp266Controller(const Config &config);
  ~Mcp266Controller();

  Mcp266Controller(const Mcp266Controller &) = delete;
  Mcp266Controller &operator=(const Mcp266Controller &) = delete;

  /// \brief Initialize TWAI peripheral and setup MCP266 communication.
  bool initialize(std::error_code &ec);

  /// \brief Clear any latched CiA 402 faults on both MCP266 motor axes.
  bool reset_faults(std::error_code &ec);

  // --- Mode Control ---
  bool set_mode_m1(int8_t mode, std::error_code &ec);
  int8_t get_mode_m1(std::error_code &ec);
  bool set_mode_m2(int8_t mode, std::error_code &ec);
  int8_t get_mode_m2(std::error_code &ec);

  // --- Motor Duty Control ---
  bool drive_m1_duty(int16_t duty, std::error_code &ec);
  bool drive_m2_duty(int16_t duty, std::error_code &ec);
  bool drive_duty(int16_t duty_m1, int16_t duty_m2, std::error_code &ec);

  // --- Closed-Loop Speed / Velocity Control ---
  bool drive_m1_speed(int32_t qpps, std::error_code &ec);
  bool drive_m2_speed(int32_t qpps, std::error_code &ec);
  bool drive_speed(int32_t qpps_m1, int32_t qpps_m2, std::error_code &ec);
  bool set_m1_position_limits(int32_t minimum_position, int32_t maximum_position,
                              std::error_code &ec);
  bool get_m1_position_limits(int32_t &minimum_position, int32_t &maximum_position,
                              std::error_code &ec);
  bool move_m1_to_position(int32_t target_position, uint32_t profile_velocity,
                           uint32_t profile_acceleration, uint32_t profile_deceleration,
                           std::error_code &ec);

  // --- Stop Motors ---
  bool stop_motors(std::error_code &ec);

  // --- Encoder Readback ---
  bool read_encoder_m1(int32_t &count, uint8_t &status, std::error_code &ec);
  bool read_encoder_m2(int32_t &count, uint8_t &status, std::error_code &ec);
  bool read_encoders(int32_t &count_m1, int32_t &count_m2, std::error_code &ec);

  bool read_speed_m1(int32_t &qpps, uint8_t &status, std::error_code &ec);
  bool read_speed_m2(int32_t &qpps, uint8_t &status, std::error_code &ec);

  /// \brief Read the M1 position demand value (0x6062): the profile
  ///        generator's instantaneous position command. CANopen mode only.
  bool read_position_demand_m1(int32_t &demand, std::error_code &ec);
  /// \brief Read the M1 velocity demand value (0x606B). CANopen mode only.
  bool read_velocity_demand_m1(int32_t &demand, std::error_code &ec);

  // --- Telemetry & Diagnostics ---
  bool read_device_info(std::string &device_name, uint32_t &device_type, std::error_code &ec);
  /// \brief Read the optional CiA 306 EDS object (0x1021:00) from the CANopen node.
  bool read_object_dictionary(std::string &eds, std::error_code &ec);
  /// \brief Probe every object index in [first_index, last_index] (subindex 0)
  ///        via SDO upload and log the ones the node implements. Used to
  ///        discover manufacturer-specific objects on devices (like the MCP266)
  ///        that publish no EDS. CANopen mode only.
  /// \return The number of objects found.
  size_t scan_object_dictionary(uint16_t first_index, uint16_t last_index);
  /// \brief For every implemented object in [first_index, last_index], read
  ///        and log its subindex values (records/arrays report their subindex
  ///        count in subindex 0). Used to map undocumented manufacturer
  ///        objects. CANopen mode only.
  /// \return The number of objects dumped.
  size_t dump_object_subindices(uint16_t first_index, uint16_t last_index);
  /// \brief Drive M1 in the Basicmicro manufacturer velocity mode (-2)
  ///        instead of standard profile velocity mode. CANopen mode only.
  bool drive_m1_speed_manufacturer(int32_t qpps, std::error_code &ec);
  bool read_main_battery_voltage(float &volts, std::error_code &ec);
  bool read_temperature(float &temp_c, std::error_code &ec);
  bool read_status(uint32_t &status_mask, std::error_code &ec);

private:
  bool init_twai(std::error_code &ec);
  bool init_canopen(std::error_code &ec);
  bool init_packet_serial(std::error_code &ec);

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

  /// Select the mode then walk Shutdown -> Switch On -> Enable Operation,
  /// verifying the statusword after each step.
  bool enable_axis(const AxisObjects &axis, int8_t mode, bool verify_mode,
                   std::error_code &ec);
  /// M1 enable path via the espp::Ds402Drive helper: select the mode
  /// (optionally verified against 0x6061), clear any latched fault, and walk
  /// the drive to Operation Enabled.
  bool enable_m1(int8_t mode, bool verify_mode, std::error_code &ec);
  bool wait_for_statusword(const AxisObjects &axis, uint16_t mask, uint16_t expected,
                           const char *step, std::error_code &ec);
  void log_statusword(const AxisObjects &axis, const char *step, uint16_t statusword) const;
  bool reset_axis_fault(const AxisObjects &axis, std::error_code &ec);

  Config config_;
  std::unique_ptr<espp::Twai> twai_;

  // CANopen Mode
  std::unique_ptr<espp::CanopenClient> canopen_client_;
  std::unique_ptr<espp::Ds402Drive> drive_m1_;

  // Packet Serial Mode
  std::unique_ptr<espp::Basicmicro> basicmicro_;
  QueueHandle_t serial_rx_queue_{nullptr};
};

} // namespace mib
