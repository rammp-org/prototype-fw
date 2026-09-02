#include <array>
#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>

#include "esp32-p4-eth.hpp"
#include "logger.hpp"
#include "mcp266_controller.hpp"
#include "mcp266_uart_controller.hpp"

using namespace std::chrono_literals;

#ifndef MIB_USE_UART_TESTING
#define MIB_USE_UART_TESTING 0
#endif

// Probe the MCP's SDO object dictionary at startup to discover which standard
// and manufacturer-specific objects it implements (it publishes no EDS).
// Takes about a minute; enable with -DMIB_SCAN_OD=1 when mapping a device.
#ifndef MIB_SCAN_OD
#define MIB_SCAN_OD 0
#endif

// Write the starter control-loop gains below to the MCP over UART (and save
// them to EEPROM) before testing. Off by default: review the constants first.
#ifndef MIB_WRITE_PID_CONFIG
#define MIB_WRITE_PID_CONFIG 0
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
  // The MCP's control-loop parameters (velocity / position PID) are not
  // reachable through standard DS402 objects, so inspect -- and optionally
  // write -- them over the Packet Serial UART link before the CAN tests. The
  // factory-default position PID constants are all zero, in which case
  // position commands are accepted but never produce motion.
  {
    mib::Mcp266UartController uart({});
    std::error_code uart_ec;
    if (!uart.initialize(uart_ec)) {
      logger.warn("UART link unavailable, skipping PID check: {}", uart_ec.message());
    } else {
      float vel_p = 0, vel_i = 0, vel_d = 0;
      uint32_t vel_qpps = 0;
      if (uart.read_velocity_pid_m1(vel_p, vel_i, vel_d, vel_qpps, uart_ec)) {
        logger.info("M1 velocity PID: P={:.3f} I={:.3f} D={:.3f} QPPS={}", vel_p, vel_i, vel_d,
                    vel_qpps);
      } else {
        logger.warn("Failed to read M1 velocity PID over UART: {}", uart_ec.message());
      }
      float pos_p = 0, pos_i = 0, pos_d = 0;
      uint32_t pos_max_i = 0, pos_deadzone = 0;
      int32_t pos_min = 0, pos_max = 0;
      if (uart.read_position_pid_m1(pos_p, pos_i, pos_d, pos_max_i, pos_deadzone, pos_min,
                                    pos_max, uart_ec)) {
        logger.info(
            "M1 position PID: P={:.3f} I={:.3f} D={:.3f} MaxI={} Deadzone={} range=[{}, {}]",
            pos_p, pos_i, pos_d, pos_max_i, pos_deadzone, pos_min, pos_max);
      } else {
        logger.warn("Failed to read M1 position PID over UART: {}", uart_ec.message());
      }
#if MIB_WRITE_PID_CONFIG
      // Starter gains: velocity loop uses the factory defaults from the MCP
      // manual; position loop uses conservative Basicmicro-style values.
      // Tune for the actual motor / encoder before relying on them.
      constexpr float kVelocityP = 1.0f;
      constexpr float kVelocityI = 0.5f;
      constexpr float kVelocityD = 0.25f;
      constexpr uint32_t kVelocityQpps = 44000;
      constexpr float kPositionP = 2000.0f;
      constexpr float kPositionI = 0.0f;
      constexpr float kPositionD = 4000.0f;
      constexpr uint32_t kPositionMaxI = 0;
      constexpr uint32_t kPositionDeadzone = 10;
      constexpr int32_t kPositionMin = -2'000'000'000;
      constexpr int32_t kPositionMax = 2'000'000'000;
      if (!uart.set_velocity_pid_m1(kVelocityP, kVelocityI, kVelocityD, kVelocityQpps,
                                    uart_ec)) {
        logger.error("Failed to write M1 velocity PID: {}", uart_ec.message());
      } else if (!uart.set_position_pid_m1(kPositionP, kPositionI, kPositionD, kPositionMaxI,
                                           kPositionDeadzone, kPositionMin, kPositionMax,
                                           uart_ec)) {
        logger.error("Failed to write M1 position PID: {}", uart_ec.message());
      } else if (!uart.write_settings_to_eeprom(uart_ec)) {
        logger.error("Failed to save MCP settings to EEPROM: {}", uart_ec.message());
      } else {
        logger.info("M1 velocity + position PID written and saved to EEPROM");
      }
#endif
    }
  }

  // Continuous ping-pong demo state, driven from the status loop at the
  // bottom once the setpoint sequences complete.
  constexpr int32_t kPingPongTarget = 10'000;
  bool pingpong_active = false;
  int32_t pingpong_target = kPingPongTarget;

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

#if MIB_SCAN_OD
    logger.info("Scanning MCP object dictionary via SDO (0x1000-0x6FFF, takes ~1 minute)...");
    size_t od_found = roboclaw.scan_object_dictionary(0x1000, 0x1FFF);
    od_found += roboclaw.scan_object_dictionary(0x2000, 0x5FFF);
    od_found += roboclaw.scan_object_dictionary(0x6000, 0x6FFF);
    logger.info("OD scan complete: {} objects implemented", od_found);
    // The manufacturer-specific region almost certainly holds the Basicmicro
    // settings registers (PID gains etc.); dump every subindex to map it.
    logger.info("Dumping manufacturer-specific object subindices (0x2000-0x20FF)...");
    roboclaw.dump_object_subindices(0x2000, 0x20FF);
    // The default PDO mappings reveal which objects the motion engine
    // actually consumes.
    logger.info("Dumping PDO communication and mapping parameters...");
    roboclaw.dump_object_subindices(0x1400, 0x140F);
    roboclaw.dump_object_subindices(0x1600, 0x160F);
    roboclaw.dump_object_subindices(0x1800, 0x180F);
    roboclaw.dump_object_subindices(0x1A00, 0x1A0F);
#endif

    // Clear any latched e-stop / safety lockout before testing (packet-serial
    // command 200 mirrored at write-only 0x20C8; a no-op when nothing is
    // latched).
    roboclaw.try_estop_reset(ec);

    // The MCP clamps position targets to the position PID's [min, max]
    // range, which is [0, 0] from the factory -- widen it over CAN first.
    if (!roboclaw.configure_m1_position_range(-2'000'000'000, 2'000'000'000, ec)) {
      logger.warn("Could not configure the M1 position range over CAN: {}", ec.message());
    }

    // Position moves need the position loop configured on the MCP (PID gains
    // and the min/max clamp) on top of the velocity PID.
    constexpr int32_t kMotor1MinimumPosition = -20'000;
    constexpr int32_t kMotor1MaximumPosition = 20'000;
    constexpr uint32_t kMotor1ProfileVelocity = 500;
    constexpr uint32_t kMotor1ProfileAcceleration = 500;
    constexpr uint32_t kMotor1ProfileDeceleration = 500;
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
      } else {
        constexpr int32_t kPositionTolerance = 100; // encoder counts
        auto read_position = [&]() -> int32_t {
          int32_t position = 0;
          uint8_t encoder_status = 0;
          std::error_code read_ec;
          roboclaw.read_encoder_m1(position, encoder_status, read_ec);
          return position;
        };
        auto command_position = [&](int32_t target) {
          return roboclaw.move_m1_to_position(target, kMotor1ProfileVelocity,
                                              kMotor1ProfileAcceleration,
                                              kMotor1ProfileDeceleration, ec);
        };
        // Wait for the motor to arrive at a target, logging progress once a
        // second. Returns true once |position - target| is within tolerance.
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
              uint8_t speed_status = 0;
              std::error_code speed_ec;
              roboclaw.read_speed_m1(speed, speed_status, speed_ec);
              logger.info("  moving: position={} speed={} counts/s (target {})", position, speed,
                          target);
            }
          }
          logger.warn("  did NOT reach {} within {} s (position={})", target, timeout.count(),
                      read_position());
          return false;
        };

        logger.info("=== Position setpoint sequence ===");
        constexpr std::array<int32_t, 4> kPositionSequence{10'000, -10'000, 5'000, 0};
        bool gain_fix_attempted = false;
        for (size_t i = 0; i < kPositionSequence.size(); ++i) {
          const int32_t target = kPositionSequence[i];
          logger.info("Position setpoint {}/{}: {}", i + 1, kPositionSequence.size(), target);
          if (!command_position(target)) {
            logger.warn("Position command rejected: {}", ec.message());
            continue;
          }
          if (wait_until_reached(target, 30s)) {
            continue;
          }
          // No movement at all can mean the position P gain is zero: the
          // packet-serial position-PID WRITE order is D,P,I while the READ
          // order is P,I,D, so a record reading [gain, 0, 0, ...] may
          // actually be P=0. Duplicating the gain into sub 2 is safe either
          // way (in read-order it is an I term neutralized by MaxI=0).
          if (!gain_fix_attempted) {
            gain_fix_attempted = true;
            std::array<int32_t, 7> pid{};
            if (roboclaw.read_position_pid_m1_raw(pid, ec) && pid[1] == 0) {
              pid[1] = pid[0] != 0 ? pid[0] : 0x3C83;
              logger.info("Duplicating position gain into sub 2 and retrying");
              if (roboclaw.write_position_pid_m1_raw(pid, ec) && command_position(target)) {
                wait_until_reached(target, 30s);
              }
            }
          }
        }

        // Closed-loop velocity via the mirrored packet-serial speed command
        // (0x2023 = cmd 35); the standard profile-velocity mode is accepted
        // but inert on this firmware.
        logger.info("=== Velocity setpoint sequence ===");
        constexpr std::array<int32_t, 4> kVelocitySequence{400, -400, 800, 0};
        for (size_t i = 0; i < kVelocitySequence.size(); ++i) {
          const int32_t target = kVelocitySequence[i];
          logger.info("Velocity setpoint {}/{}: {} counts/s", i + 1, kVelocitySequence.size(),
                      target);
          if (!roboclaw.drive_m1_speed(target, ec)) {
            logger.warn("Velocity command rejected: {}", ec.message());
            continue;
          }
          for (int poll = 0; poll < 3; ++poll) {
            std::this_thread::sleep_for(1s);
            int32_t speed = 0;
            uint8_t speed_status = 0;
            std::error_code speed_ec;
            roboclaw.read_speed_m1(speed, speed_status, speed_ec);
            logger.info("  velocity: target={} actual={} counts/s, position={}", target, speed,
                        read_position());
          }
        }
        roboclaw.drive_m1_speed(0, ec);

        // Hand off to the continuous ping-pong in the status loop below.
        logger.info("=== Continuous position ping-pong between +/-{} ===", kPingPongTarget);
        if (command_position(kPingPongTarget)) {
          pingpong_active = true;
        } else {
          logger.warn("Ping-pong start command rejected: {}", ec.message());
        }
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
      logger.info("MCP266 CANopen poll: statusword=0x{:04X}, M1 encoder={}, target={}", status,
                  encoder_value, pingpong_active ? std::to_string(pingpong_target) : "idle");
      // Bounce between +/-kPingPongTarget so motion stays observable.
      if (pingpong_active && std::abs(encoder_value - pingpong_target) <= 100) {
        pingpong_target = -pingpong_target;
        logger.info("Ping-pong: new target {}", pingpong_target);
        if (!roboclaw.move_m1_to_position(pingpong_target, 500, 500, 500, ec)) {
          logger.warn("Ping-pong position command rejected: {}", ec.message());
        }
      }
    } else {
      logger.warn("MCP266 CANopen poll failed: {}", ec.message());
    }
#endif

    std::this_thread::sleep_for(2000ms);
  }
}
