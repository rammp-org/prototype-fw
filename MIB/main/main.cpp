#include <array>
#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>

#include "esp32-p4-eth.hpp"
#include "logger.hpp"

#include "canopen_client.hpp"
#include "mcp266.hpp"
#include "twai.hpp"

using namespace std::chrono_literals;

// Probe the MCP's SDO object dictionary at startup to discover which objects
// it implements (it publishes no EDS). Takes ~1 minute; enable with
// -DMIB_SCAN_OD=1 when mapping a device.
#ifndef MIB_SCAN_OD
#define MIB_SCAN_OD 0
#endif

using Axis = espp::Mcp266::Axis;

namespace {

constexpr int32_t kPingPongTarget = 10'000;

/// Bare status loop used when the motor controller could not be brought up:
/// keep reporting Ethernet link state so the board is still observable.
[[noreturn]] void run_ethernet_only(espp::Esp32P4Eth &board, espp::Logger &logger) {
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

/// Configure M1, run a profile-position setpoint sequence, then a continuous
/// ping-pong interleaved with Ethernet status reporting. The controller must
/// already be started.
[[noreturn]] void run_motor_demo(espp::Mcp266 &mc, espp::Esp32P4Eth &board,
                                 espp::Logger &logger) {
  std::error_code ec;

#if MIB_SCAN_OD
  logger.info("Scanning MCP object dictionary via SDO (0x1000-0x6FFF, takes ~1 minute)...");
  size_t od_found = mc.scan_object_dictionary(0x1000, 0x1FFF);
  od_found += mc.scan_object_dictionary(0x2000, 0x5FFF);
  od_found += mc.scan_object_dictionary(0x6000, 0x6FFF);
  logger.info("OD scan complete: {} objects implemented", od_found);
  mc.dump_object_subindices(0x2000, 0x20FF);
#endif

  // --- One-time per-boot MCP configuration ---
  mc.reset_estop(ec); // clear any latched e-stop (no-op if nothing is latched)
  // Widen the M1 position clamp (factory [0, 0] forces every target to zero)
  // and seed a P gain if the drive has none. The MCP reverts to EEPROM on
  // power-up, so this runs every boot.
  if (!mc.configure_position_loop(Axis::M1, -2'000'000'000, 2'000'000'000, ec)) {
    logger.warn("Could not configure the M1 position loop: {}", ec.message());
  }
  if (!mc.set_position_limits(Axis::M1, -20'000, 20'000, ec)) {
    logger.warn("Failed to set M1 software position limits: {}", ec.message());
  }

  float volts = 0.0f, temp_c = 0.0f;
  if (mc.read_main_battery_voltage(volts, ec)) {
    logger.info("Main battery: {:.1f} V", volts);
  }
  if (mc.read_temperature(temp_c, ec)) {
    logger.info("Board temperature: {:.1f} C", temp_c);
  }

  constexpr uint32_t kProfileVelocity = 500;
  constexpr uint32_t kProfileAccel = 500;
  constexpr uint32_t kProfileDecel = 500;
  constexpr int32_t kPositionTolerance = 100; // encoder counts

  auto read_position = [&]() -> int32_t {
    int32_t position = 0;
    std::error_code read_ec;
    mc.read_encoder(Axis::M1, position, read_ec);
    return position;
  };
  auto command_position = [&](int32_t target) {
    return mc.move_to_position(Axis::M1, target, kProfileVelocity, kProfileAccel, kProfileDecel,
                               ec);
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
        mc.read_speed(Axis::M1, speed, speed_ec);
        logger.info("  moving: position={} speed={} counts/s (target {})", position, speed, target);
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

  // --- Continuous position ping-pong, interleaved with Ethernet status ---
  logger.info("=== Continuous position ping-pong between +/-{} ===", kPingPongTarget);
  bool pingpong_active = command_position(kPingPongTarget);
  if (!pingpong_active) {
    logger.warn("Ping-pong start command rejected: {}", ec.message());
  }
  int32_t pingpong_target = kPingPongTarget;

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
    const bool status_ok = mc.read_statusword(Axis::M1, statusword, ec);
    const bool encoder_ok = mc.read_encoder(Axis::M1, encoder_value, ec);
    if (status_ok && encoder_ok) {
      logger.info("MCP266 CANopen poll: statusword=0x{:04X}, M1 encoder={}, target={}", statusword,
                  encoder_value, pingpong_active ? std::to_string(pingpong_target) : "idle");
      if (pingpong_active && std::abs(encoder_value - pingpong_target) <= 100) {
        pingpong_target = -pingpong_target;
        logger.info("Ping-pong: new target {}", pingpong_target);
        if (!mc.move_to_position(Axis::M1, pingpong_target, 500, 500, 500, ec)) {
          logger.warn("Ping-pong position command rejected: {}", ec.message());
        }
      }
    } else {
      logger.warn("MCP266 CANopen poll failed: {}", ec.message());
    }

    std::this_thread::sleep_for(2000ms);
  }
}

} // namespace

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

  std::error_code ec;

  // The MCP266 controller (espp/mcp266) is transport-agnostic, so MIB owns the
  // TWAI + CANopen client and drives espp::Mcp266 on top. These are
  // function-local statics so the TWAI receive task and the client's send
  // lambda keep valid references for the life of the program.
  logger.info("Initializing MCP266 over CANopen (TWAI, node 10)...");
  static espp::CanopenClient *client_ptr = nullptr;
  static espp::Twai twai({
      .tx_gpio = 17,
      .rx_gpio = 16,
      .baudrate = 1000000,
      .mode = espp::Twai::Mode::NORMAL,
      .tx_queue_depth = 10,
      .on_receive =
          [](const espp::Twai::Message &msg) {
            if (client_ptr) {
              client_ptr->process_frame(espp::CanopenClient::CanFrame{
                  .id = msg.id,
                  .extended = msg.extended,
                  .rtr = msg.rtr,
                  .dlc = msg.dlc,
                  .data = msg.data,
              });
            }
          },
      .log_level = espp::Logger::Verbosity::WARN,
  });
  static espp::CanopenClient client({
      .node_id = 10,
      .send =
          [](const espp::CanopenClient::CanFrame &frame) {
            espp::Twai::Message msg{
                .id = frame.id,
                .extended = frame.extended,
                .rtr = frame.rtr,
                .dlc = frame.dlc,
                .data = frame.data,
            };
            std::error_code tx_ec;
            return twai.transmit(msg, tx_ec);
          },
      .sdo_timeout = 500ms,
      .log_level = espp::Logger::Verbosity::WARN,
  });
  client_ptr = &client;
  static espp::Mcp266 mcp(client, {.log_level = espp::Logger::Verbosity::INFO});

  if (!twai.initialize(ec)) {
    logger.error("Failed to initialize TWAI: {}", ec.message());
    run_ethernet_only(board, logger);
  }
  if (!mcp.start(ec)) {
    logger.error("Failed to start MCP266: {} -- is the node on the bus?", ec.message());
    run_ethernet_only(board, logger);
  }
  logger.info("MCP266 CANopen controller started");
  run_motor_demo(mcp, board, logger);
}
