#include <chrono>
#include <thread>

#include "esp32-p4-eth.hpp"
#include "logger.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {

  auto &board = espp::Esp32P4Eth::get();

  espp::Logger logger({.tag = "five_bar_v1", .level = espp::Logger::Verbosity::INFO});

  logger.info("Bootup");

  if (!board.initialize_ethernet()) {
    logger.error("Failed to initialize Ethernet");
  } else {
    logger.info("Ethernet initialized");
  }

  while (true) {
    logger.info("Ethernet connected: {}", board.is_ethernet_connected());
    std::this_thread::sleep_for(5s);
  }
}