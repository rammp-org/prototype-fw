#include <chrono>
#include <span>

#include "mcp266_uart_controller.hpp"

using namespace std::chrono_literals;

namespace mib {

Mcp266UartController::Mcp266UartController(const Config &config)
    : config_(config) {}

Mcp266UartController::~Mcp266UartController() {
  if (uart_installed_) {
    uart_driver_delete(config_.uart_port);
  }
}

bool Mcp266UartController::initialize(std::error_code &ec) {
  ec.clear();
  uart_config_t uart_config{};
  uart_config.baud_rate = config_.baudrate;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uart_config.rx_flow_ctrl_thresh = 0;
  uart_config.source_clk = UART_SCLK_DEFAULT;

  esp_err_t err = uart_driver_install(config_.uart_port, 256, 0, 0, nullptr, 0);
  if (err != ESP_OK) {
    ec = std::error_code(err, std::generic_category());
    return false;
  }
  uart_installed_ = true;
  if ((err = uart_param_config(config_.uart_port, &uart_config)) != ESP_OK ||
      (err = uart_set_pin(config_.uart_port, config_.tx_gpio, config_.rx_gpio, UART_PIN_NO_CHANGE,
                          UART_PIN_NO_CHANGE)) != ESP_OK) {
    ec = std::error_code(err, std::generic_category());
    return false;
  }

  driver_ = std::make_unique<BasicmicroExt>(espp::Basicmicro::Config{
      .address = config_.address,
      .write = [this](std::span<const uint8_t> data) {
        return uart_write_bytes(config_.uart_port, reinterpret_cast<const char *>(data.data()),
                                data.size()) == static_cast<int>(data.size());
      },
      .read = [this](std::span<uint8_t> data, std::chrono::milliseconds timeout) -> size_t {
        const int received = uart_read_bytes(config_.uart_port, data.data(), data.size(),
                                             pdMS_TO_TICKS(timeout.count()));
        return received < 0 ? 0 : static_cast<size_t>(received);
      },
      .timeout = 50ms,
  });
  return true;
}

bool Mcp266UartController::read_firmware_version(std::string &version, std::error_code &ec) {
  return driver_ && driver_->read_firmware_version(version, ec);
}

bool Mcp266UartController::read_status(uint32_t &status, std::error_code &ec) {
  return driver_ && driver_->read_status(status, ec);
}

bool Mcp266UartController::read_encoder_m1(uint32_t &count, uint8_t &status,
                                            std::error_code &ec) {
  return driver_ && driver_->read_encoder_m1(count, status, ec);
}

bool Mcp266UartController::read_encoder_m2(uint32_t &count, uint8_t &status,
                                            std::error_code &ec) {
  return driver_ && driver_->read_encoder_m2(count, status, ec);
}

bool Mcp266UartController::drive_m1_duty(int16_t duty, std::error_code &ec) {
  return driver_ && driver_->drive_m1_duty(duty, ec);
}

bool Mcp266UartController::drive_m2_duty(int16_t duty, std::error_code &ec) {
  return driver_ && driver_->drive_m2_duty(duty, ec);
}

bool Mcp266UartController::stop(std::error_code &ec) {
  return driver_ && driver_->drive_duty(0, 0, ec);
}

bool Mcp266UartController::read_velocity_pid_m1(float &p, float &i, float &d, uint32_t &qpps,
                                                std::error_code &ec) {
  return driver_ && driver_->read_velocity_pid_m1(p, i, d, qpps, ec);
}

bool Mcp266UartController::set_velocity_pid_m1(float p, float i, float d, uint32_t qpps,
                                               std::error_code &ec) {
  return driver_ && driver_->set_velocity_pid_m1(p, i, d, qpps, ec);
}

bool Mcp266UartController::read_position_pid_m1(float &p, float &i, float &d, uint32_t &max_i,
                                                uint32_t &deadzone, int32_t &min_pos,
                                                int32_t &max_pos, std::error_code &ec) {
  return driver_ &&
         driver_->read_position_pid_m1(p, i, d, max_i, deadzone, min_pos, max_pos, ec);
}

bool Mcp266UartController::set_position_pid_m1(float p, float i, float d, uint32_t max_i,
                                               uint32_t deadzone, int32_t min_pos,
                                               int32_t max_pos, std::error_code &ec) {
  return driver_ && driver_->set_position_pid_m1(p, i, d, max_i, deadzone, min_pos, max_pos, ec);
}

bool Mcp266UartController::write_settings_to_eeprom(std::error_code &ec) {
  return driver_ && driver_->write_settings_to_eeprom(ec);
}

} // namespace mib