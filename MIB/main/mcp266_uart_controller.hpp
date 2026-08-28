#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <system_error>

#include "basicmicro.hpp"
#include "driver/gpio.h"
#include "driver/uart.h"

namespace mib {

class Mcp266UartController {
public:
  struct Config {
    uart_port_t uart_port{UART_NUM_1};
    gpio_num_t tx_gpio{GPIO_NUM_4}; // Connect to MCP266 S1 (RX)
    gpio_num_t rx_gpio{GPIO_NUM_5}; // Connect to MCP266 S2 (TX)
    int baudrate{115200};
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

private:
  Config config_;
  std::unique_ptr<espp::Basicmicro> driver_;
  bool uart_installed_{false};
};

} // namespace mib