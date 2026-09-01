#include <chrono>
#include <thread>

#include "esp32-p4-eth.hpp"
#include "logger.hpp"
#include "mcp266_controller.hpp"
#include "mcp266_uart_controller.hpp"

using namespace std::chrono_literals;

#ifndef MIB_USE_UART_TESTING
#define MIB_USE_UART_TESTING 0
#endif

extern "C" void app_main(void) {
  // Board hardware abstraction
  auto &board = espp::Esp32P4Eth::get();
  espp::Logger logger({.tag = "MIB", .level = espp::Logger::Verbosity::INFO});

  logger.info("==============================================");
  logger.info("  MIB Project - ESP32-P4 + MCL CAN + MCP266");
  logger.info("==============================================");

  // Initialize Ethernet
  if (!board.initialize_ethernet()) {
    logger.error("Failed to initialize Ethernet on ESP32-P4-ETH");
  } else {
    logger.info("Ethernet initialized successfully");
  }

#if MIB_USE_UART_TESTING
  mib::Mcp266UartController roboclaw({});
  std::error_code ec;

  logger.info("Initializing MCP266 Controller via UART Packet Serial...");
  if (!roboclaw.initialize(ec)) {
    logger.error("Failed to initialize MCP266 UART Controller: {}", ec.message());
  } else {
    logger.info("MCP266 UART controller initialized (TX=5 -> S1, RX=6 <- S2, 115200 baud)");
    std::string firmware_version;
    if (roboclaw.read_firmware_version(firmware_version, ec)) {
      logger.info("MCP266 firmware: '{}'", firmware_version);
    } else {
      logger.warn("Failed to read MCP266 firmware version: {}", ec.message());
    }

    uint32_t status = 0;
    if (roboclaw.read_status(status, ec)) {
      logger.info("MCP266 Packet Serial status: 0x{:08X}", status);
    } else {
      logger.warn("Failed to read MCP266 Packet Serial status: {}", ec.message());
    }

    // Read encoder.
    uint32_t encoder_value = 0;
    uint8_t encoder_status = 0;
    if (roboclaw.read_encoder_m1(encoder_value, encoder_status, ec)) {
      logger.info("MCP266 encoder value: {} (status=0x{:02X})", encoder_value, encoder_status);
    } else {
      logger.warn("Failed to read MCP266 encoder value: {}", ec.message());
    }
    // Drive at approximately 50 percent duty.
    constexpr int16_t kFiftyPercentDuty = 16384;
    if (roboclaw.drive_m1_duty(kFiftyPercentDuty, ec)) {
      logger.info("MCP266 drive duty set to 50%");
    } else {
      logger.warn("Failed to set MCP266 drive duty: {}", ec.message());
    }
  }
#else
  mib::Mcp266Controller roboclaw({
      .twai_tx_gpio = GPIO_NUM_17,
      .twai_rx_gpio = GPIO_NUM_16,
      .baudrate = 1000000,
      .mode = mib::Mcp266Controller::Mode::CANOPEN,
      .node_id = 10,
      .log_level = espp::Logger::Verbosity::INFO,
  });
  std::error_code ec;

  logger.info("Initializing MCP266 Controller via CANopen (TWAI)...");
  if (!roboclaw.initialize(ec)) {
    logger.error("Failed to initialize MCP266 CANopen Controller: {}", ec.message());
  } else {
    logger.info("MCP266 CANopen controller initialized");
    std::string eds;
    if (roboclaw.read_object_dictionary(eds, ec)) {
      logger.info("MCP266 EDS (0x1021:00):\n{}", eds);
    } else {
      logger.warn("MCP266 does not expose an EDS at 0x1021:00: {}", ec.message());
    }

    constexpr int32_t kMotor1MinimumPosition = -20'000;
    constexpr int32_t kMotor1MaximumPosition = 20'000;
    constexpr int32_t kMotor1TargetPosition = 10'000;
    constexpr uint32_t kMotor1ProfileVelocity = 500;
    constexpr uint32_t kMotor1ProfileAcceleration = 500;
    constexpr uint32_t kMotor1ProfileDeceleration = 500;
    constexpr int kPositionPolls = 5;
    logger.info("Setting M1 position limits: [{}, {}]", kMotor1MinimumPosition,
                kMotor1MaximumPosition);
    if (!roboclaw.set_m1_position_limits(kMotor1MinimumPosition, kMotor1MaximumPosition, ec)) {
      logger.error("Failed to set M1 position limits: {}", ec.message());
    } else {
      int32_t configured_minimum = 0;
      int32_t configured_maximum = 0;
      if (!roboclaw.get_m1_position_limits(configured_minimum, configured_maximum, ec)) {
        logger.error("Failed to read M1 position limits: {}", ec.message());
      } else if (configured_minimum != kMotor1MinimumPosition ||
                 configured_maximum != kMotor1MaximumPosition) {
        logger.error("M1 position limit mismatch: expected [{}, {}], got [{}, {}]",
                     kMotor1MinimumPosition, kMotor1MaximumPosition, configured_minimum,
                     configured_maximum);
      } else if (roboclaw.move_m1_to_position(kMotor1TargetPosition, kMotor1ProfileVelocity,
                       kMotor1ProfileAcceleration,
                       kMotor1ProfileDeceleration, ec)) {
        logger.info("Testing Motor 1 position command: target={}, velocity={}, acceleration={}, "
            "deceleration={}",
            kMotor1TargetPosition, kMotor1ProfileVelocity, kMotor1ProfileAcceleration,
            kMotor1ProfileDeceleration);
        for (int poll = 1; poll <= kPositionPolls; ++poll) {
          std::this_thread::sleep_for(1s);
          uint32_t status = 0;
          int32_t position = 0;
          uint8_t encoder_status = 0;
          const bool status_ok = roboclaw.read_status(status, ec);
          const bool position_ok = roboclaw.read_encoder_m1(position, encoder_status, ec);
          if (status_ok && position_ok) {
            logger.info("Position poll {}/{}: statusword=0x{:04X}, M1 position={}", poll,
                        kPositionPolls, status, position);
          } else {
            logger.warn("Position poll {}/{} failed: {}", poll, kPositionPolls, ec.message());
          }
        }
    }
      else {
        logger.warn("Motor 1 position command rejected: {}", ec.message());
      }
    }
  }
#endif

  // Background status loop
  bool last_eth_status = false;
  bool have_eth_status = false;

  while (true) {
    const bool eth_connected = board.is_ethernet_connected();
    if (!have_eth_status || eth_connected != last_eth_status) {
      logger.info("Ethernet status changed: connected={}", eth_connected);
      if (eth_connected) {
        auto ip = board.ethernet_ip();
        logger.info("Ethernet IP: " IPSTR, IP2STR(&ip));
      }
      last_eth_status = eth_connected;
      have_eth_status = true;
    }

#if MIB_USE_UART_TESTING
    uint32_t status = 0;
    uint32_t encoder_value = 0;
    uint8_t encoder_status = 0;
    const bool status_ok = roboclaw.read_status(status, ec);
    const bool encoder_ok = roboclaw.read_encoder_m1(encoder_value, encoder_status, ec);
    if (status_ok && encoder_ok) {
      logger.info("MCP266 UART poll: status=0x{:08X}, M1 encoder={} (status=0x{:02X})", status,
                  encoder_value, encoder_status);
    } else {
      logger.warn("MCP266 UART poll failed: {}", ec.message());
    }
#else
    uint32_t status = 0;
    int32_t encoder_value = 0;
    uint8_t encoder_status = 0;
    const bool status_ok = roboclaw.read_status(status, ec);
    const bool encoder_ok = roboclaw.read_encoder_m1(encoder_value, encoder_status, ec);
    if (status_ok && encoder_ok) {
      logger.info("MCP266 CANopen poll: statusword=0x{:04X}, M1 encoder={} (status=0x{:02X})",
                  status, encoder_value, encoder_status);
    } else {
      logger.warn("MCP266 CANopen poll failed: {}", ec.message());
    }
#endif

    std::this_thread::sleep_for(2000ms);
  }
}
