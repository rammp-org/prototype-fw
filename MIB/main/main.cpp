#include <array>
#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>

#include "esp32-p4-eth.hpp"
#include "logger.hpp"
#include "mcp266_controller.hpp"

using namespace std::chrono_literals;

// Probe the MCP's SDO object dictionary at startup to discover which objects
// it implements (it publishes no EDS). Takes ~1 minute; enable with
// -DMIB_SCAN_OD=1 when mapping a device.
#ifndef MIB_SCAN_OD
#define MIB_SCAN_OD 0
#endif

using Axis = mib::Mcp266Controller::Axis;

extern "C" void app_main(void) {
  auto &board = espp::Esp32P4Eth::get();
  espp::Logger logger({.tag = "MIB", .level = espp::Logger::Verbosity::INFO});

  logger.info("==============================================");
  logger.info("  MIB Project - ESP32-P4 + MCL CAN + MCP266");
  logger.info("==============================================");

  if (!board.initialize_ethernet()) {
    logger.error("Failed to initialize Ethernet on ESP32-P4-ETH");
  } else {
    logger.info("Ethernet initialized successfully");
  }

  // Continuous ping-pong demo state, driven from the status loop below.
  constexpr int32_t kPingPongTarget = 10'000;
  bool pingpong_active = false;
  int32_t pingpong_target = kPingPongTarget;

  mib::Mcp266Controller roboclaw({
      .twai_tx_gpio = GPIO_NUM_17,
      .twai_rx_gpio = GPIO_NUM_16,
      .baudrate = 1000000,
      .node_id = 10,
      .log_level = espp::Logger::Verbosity::INFO,
  });
  std::error_code ec;

  logger.info("Initializing MCP266 Controller via CANopen (TWAI)...");
  if (!roboclaw.initialize(ec)) {
    logger.error("Failed to initialize MCP266 CANopen Controller: {}", ec.message());
  } else {
    logger.info("MCP266 CANopen controller initialized");

#if MIB_SCAN_OD
    logger.info("Scanning MCP object dictionary via SDO (0x1000-0x6FFF, takes ~1 minute)...");
    size_t od_found = roboclaw.scan_object_dictionary(0x1000, 0x1FFF);
    od_found += roboclaw.scan_object_dictionary(0x2000, 0x5FFF);
    od_found += roboclaw.scan_object_dictionary(0x6000, 0x6FFF);
    logger.info("OD scan complete: {} objects implemented", od_found);
    roboclaw.dump_object_subindices(0x2000, 0x20FF);
#endif

    // --- One-time per-boot MCP configuration ---
    // Clear any latched e-stop (no-op if nothing is latched).
    roboclaw.reset_estop(ec);
    // Configure the M1 position loop: widen the min/max clamp (factory [0, 0]
    // forces every target to zero) and ensure a non-zero position P gain.
    // The MCP reverts to EEPROM on power-up, so this runs every boot.
    if (!roboclaw.configure_position_loop(Axis::M1, -2'000'000'000, 2'000'000'000, ec)) {
      logger.warn("Could not configure the M1 position loop: {}", ec.message());
    }
    if (!roboclaw.set_position_limits(Axis::M1, -20'000, 20'000, ec)) {
      logger.warn("Failed to set M1 software position limits: {}", ec.message());
    }

    float volts = 0.0f, temp_c = 0.0f;
    if (roboclaw.read_main_battery_voltage(volts, ec)) {
      logger.info("Main battery: {:.1f} V", volts);
    }
    if (roboclaw.read_temperature(temp_c, ec)) {
      logger.info("Board temperature: {:.1f} C", temp_c);
    }

    constexpr uint32_t kProfileVelocity = 500;
    constexpr uint32_t kProfileAccel = 500;
    constexpr uint32_t kProfileDecel = 500;
    constexpr int32_t kPositionTolerance = 100; // encoder counts

    auto read_position = [&]() -> int32_t {
      int32_t position = 0;
      std::error_code read_ec;
      roboclaw.read_encoder(Axis::M1, position, read_ec);
      return position;
    };
    auto command_position = [&](int32_t target) {
      return roboclaw.move_to_position(Axis::M1, target, kProfileVelocity, kProfileAccel,
                                       kProfileDecel, ec);
    };
    // Wait for the motor to arrive at a target, logging progress once a second.
    auto wait_until_reached = [&](int32_t target, std::chrono::seconds timeout) -> bool {
      const auto start = std::chrono::steady_clock::now();
      const auto deadline = start + timeout;
      int iteration = 0;
      while (std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(250ms);
        const int32_t position = read_position();
        if (std::abs(position - target) <= kPositionTolerance) {
          const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now() - start);
          logger.info("  reached {} in {} ms", target, elapsed.count());
          return true;
        }
        if ((++iteration % 4) == 0) {
          int32_t speed = 0;
          std::error_code speed_ec;
          roboclaw.read_speed(Axis::M1, speed, speed_ec);
          logger.info("  moving: position={} speed={} counts/s (target {})", position, speed,
                      target);
        }
      }
      logger.warn("  did NOT reach {} within {} s (position={})", target, timeout.count(),
                  read_position());
      return false;
    };

    // --- Position setpoint sequence ---
    logger.info("=== Position setpoint sequence ===");
    constexpr std::array<int32_t, 4> kPositionSequence{10'000, -10'000, 5'000, 0};
    for (size_t i = 0; i < kPositionSequence.size(); ++i) {
      const int32_t target = kPositionSequence[i];
      logger.info("Position setpoint {}/{}: {}", i + 1, kPositionSequence.size(), target);
      if (!command_position(target)) {
        logger.warn("Position command rejected: {}", ec.message());
        continue;
      }
      wait_until_reached(target, 30s);
    }

    // --- Continuous position ping-pong, driven from the status loop below ---
    logger.info("=== Continuous position ping-pong between +/-{} ===", kPingPongTarget);
    if (command_position(kPingPongTarget)) {
      pingpong_active = true;
    } else {
      logger.warn("Ping-pong start command rejected: {}", ec.message());
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

    uint16_t statusword = 0;
    int32_t encoder_value = 0;
    const bool status_ok = roboclaw.read_statusword(Axis::M1, statusword, ec);
    const bool encoder_ok = roboclaw.read_encoder(Axis::M1, encoder_value, ec);
    if (status_ok && encoder_ok) {
      logger.info("MCP266 CANopen poll: statusword=0x{:04X}, M1 encoder={}, target={}", statusword,
                  encoder_value, pingpong_active ? std::to_string(pingpong_target) : "idle");
      // Bounce between +/-kPingPongTarget so motion stays observable.
      if (pingpong_active && std::abs(encoder_value - pingpong_target) <= 100) {
        pingpong_target = -pingpong_target;
        logger.info("Ping-pong: new target {}", pingpong_target);
        if (!roboclaw.move_to_position(Axis::M1, pingpong_target, 500, 500, 500, ec)) {
          logger.warn("Ping-pong position command rejected: {}", ec.message());
        }
      }
    } else {
      logger.warn("MCP266 CANopen poll failed: {}", ec.message());
    }

    std::this_thread::sleep_for(2000ms);
  }
}
