#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <system_error>

#include "basicmicro_ext.hpp"
#include "driver/gpio.h"
#include "driver/uart.h"

namespace mib {

class Mcp266UartController {
public:
  struct Config {
    uart_port_t uart_port{UART_NUM_1};
    gpio_num_t tx_gpio{GPIO_NUM_4}; // Connect to MCP266 S1 (RX)
    gpio_num_t rx_gpio{GPIO_NUM_5}; // Connect to MCP266 S2 (TX)
    int baudrate{38400};
    uint8_t address{128};
  };

  explicit Mcp266UartController(const Config &config);
  ~Mcp266UartController();

  Mcp266UartController(const Mcp266UartController &) = delete;
  Mcp266UartController &operator=(const Mcp266UartController &) = delete;

  bool initialize(std::error_code &ec);
  bool read_firmware_version(std::string &version, std::error_code &ec);
  bool read_status(uint32_t &status, std::error_code &ec);
  bool read_encoder_m1(uint32_t &count, uint8_t &status, std::error_code &ec);
  bool read_encoder_m2(uint32_t &count, uint8_t &status, std::error_code &ec);
  bool drive_m1_duty(int16_t duty, std::error_code &ec);
  bool drive_m2_duty(int16_t duty, std::error_code &ec);
  bool stop(std::error_code &ec);

  // --- Control-loop configuration (not reachable via standard DS402) ---
  bool read_velocity_pid_m1(float &p, float &i, float &d, uint32_t &qpps, std::error_code &ec);
  bool set_velocity_pid_m1(float p, float i, float d, uint32_t qpps, std::error_code &ec);
  bool read_position_pid_m1(float &p, float &i, float &d, uint32_t &max_i, uint32_t &deadzone,
                            int32_t &min_pos, int32_t &max_pos, std::error_code &ec);
  bool set_position_pid_m1(float p, float i, float d, uint32_t max_i, uint32_t deadzone,
                           int32_t min_pos, int32_t max_pos, std::error_code &ec);
  /// \brief Persist the active settings to the MCP's EEPROM (command 94).
  bool write_settings_to_eeprom(std::error_code &ec);

private:
  Config config_;
  std::unique_ptr<BasicmicroExt> driver_;
  bool uart_installed_{false};
};

} // namespace mib