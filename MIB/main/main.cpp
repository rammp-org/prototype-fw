#include <chrono>
#include <thread>

#include "esp32-p4-eth.hpp"
#include "logger.hpp"
#include "mcp266_controller.hpp"

using namespace std::chrono_literals;

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

  mib::Mcp266Controller roboclaw({
      .twai_tx_gpio = GPIO_NUM_17,
      .twai_rx_gpio = GPIO_NUM_16,
      .baudrate = 1000000,
      .mode = mib::Mcp266Controller::Mode::CANOPEN,
      .node_id = 1,
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
  }

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

    std::this_thread::sleep_for(2000ms);
  }
}
