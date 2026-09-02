#include <chrono>
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
// Takes about a minute; disable with -DMIB_SCAN_OD=0 once mapped.
#ifndef MIB_SCAN_OD
#define MIB_SCAN_OD 1
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

    // Test 0: open-loop duty (manufacturer mode -1). Bypasses every control
    // loop, so any motion here proves the CAN -> power stage path end to end.
    constexpr int16_t kTestDuty = 8192; // 25%
    logger.info("Testing Motor 1 duty command: {} (25%)", kTestDuty);
    if (!roboclaw.drive_m1_duty(kTestDuty, ec)) {
      logger.warn("Motor 1 duty command rejected: {}", ec.message());
    } else {
      std::this_thread::sleep_for(2s);
      int32_t position = 0;
      int32_t speed = 0;
      uint8_t encoder_status = 0;
      uint8_t speed_status = 0;
      if (roboclaw.read_encoder_m1(position, encoder_status, ec) &&
          roboclaw.read_speed_m1(speed, speed_status, ec)) {
        logger.info("After 2s at 25% duty: position={}, speed={} counts/s", position, speed);
      }
      if (!roboclaw.drive_m1_duty(0, ec)) {
        logger.warn("Failed to stop Motor 1 after duty test: {}", ec.message());
      }
    }

    // Test 0b: duty via the target torque object (0x6071, per-mille). In the
    // manufacturer duty mode the target may be sampled from 0x6071 rather
    // than 0x60FF.
    logger.info("Testing Motor 1 duty via 0x6071: 250 per-mille (25%)");
    if (!roboclaw.drive_m1_duty_via_torque(250, ec)) {
      logger.warn("Motor 1 duty-via-torque command rejected: {}", ec.message());
    } else {
      std::this_thread::sleep_for(2s);
      int32_t position = 0;
      int32_t speed = 0;
      uint8_t encoder_status = 0;
      uint8_t speed_status = 0;
      if (roboclaw.read_encoder_m1(position, encoder_status, ec) &&
          roboclaw.read_speed_m1(speed, speed_status, ec)) {
        logger.info("After 2s at 25% via 0x6071: position={}, speed={} counts/s", position,
                    speed);
      }
      if (!roboclaw.drive_m1_duty_via_torque(0, ec)) {
        logger.warn("Failed to stop Motor 1 after duty-via-torque test: {}", ec.message());
      }
    }

    // Test 1: closed-loop velocity. This exercises only the velocity PID
    // (auto-tuned in Motion Studio), so it isolates the CANopen control path
    // from the position-loop configuration.
    constexpr int32_t kTestVelocity = 500;
    constexpr int kVelocityPolls = 3;
    logger.info("Testing Motor 1 velocity command: target={} counts/s", kTestVelocity);
    if (!roboclaw.drive_m1_speed(kTestVelocity, ec)) {
      logger.warn("Motor 1 velocity command rejected: {}", ec.message());
    } else {
      for (int poll = 1; poll <= kVelocityPolls; ++poll) {
        std::this_thread::sleep_for(1s);
        uint32_t status = 0;
        int32_t position = 0;
        int32_t speed = 0;
        uint8_t encoder_status = 0;
        uint8_t speed_status = 0;
        const bool status_ok = roboclaw.read_status(status, ec);
        const bool position_ok = roboclaw.read_encoder_m1(position, encoder_status, ec);
        const bool speed_ok = roboclaw.read_speed_m1(speed, speed_status, ec);
        if (status_ok && position_ok && speed_ok) {
          std::error_code demand_ec;
          int32_t velocity_demand = 0;
          const bool demand_ok = roboclaw.read_velocity_demand_m1(velocity_demand, demand_ec);
          logger.info(
              "Velocity poll {}/{}: statusword=0x{:04X}, position={}, speed={} counts/s, "
              "velocity demand={}",
              poll, kVelocityPolls, status, position, speed,
              demand_ok ? std::to_string(velocity_demand) : "n/a");
        } else {
          logger.warn("Velocity poll {}/{} failed: {}", poll, kVelocityPolls, ec.message());
        }
      }
      if (!roboclaw.drive_m1_speed(0, ec)) {
        logger.warn("Failed to stop Motor 1 after velocity test: {}", ec.message());
      }
    }

    // Test 1b: manufacturer velocity mode (-2). The MCP advertises only
    // manufacturer modes in supported-drive-modes (0x6502 = 0x700), so the
    // standard profile velocity mode may be a no-op even though it is echoed.
    logger.info("Testing Motor 1 manufacturer velocity mode (-2): target={} counts/s",
                kTestVelocity);
    if (!roboclaw.drive_m1_speed_manufacturer(kTestVelocity, ec)) {
      logger.warn("Motor 1 manufacturer velocity command rejected: {}", ec.message());
    } else {
      for (int poll = 1; poll <= kVelocityPolls; ++poll) {
        std::this_thread::sleep_for(1s);
        int32_t position = 0;
        int32_t speed = 0;
        uint8_t encoder_status = 0;
        uint8_t speed_status = 0;
        if (roboclaw.read_encoder_m1(position, encoder_status, ec) &&
            roboclaw.read_speed_m1(speed, speed_status, ec)) {
          std::error_code demand_ec;
          int32_t velocity_demand = 0;
          const bool demand_ok = roboclaw.read_velocity_demand_m1(velocity_demand, demand_ec);
          logger.info("Mfr velocity poll {}/{}: position={}, speed={} counts/s, demand={}", poll,
                      kVelocityPolls, position, speed,
                      demand_ok ? std::to_string(velocity_demand) : "n/a");
        } else {
          logger.warn("Mfr velocity poll {}/{} failed: {}", poll, kVelocityPolls, ec.message());
        }
      }
      if (!roboclaw.drive_m1_speed_manufacturer(0, ec)) {
        logger.warn("Failed to stop Motor 1 after manufacturer velocity test: {}", ec.message());
      }
    }

    // Test 1c: deliver the command via RPDO instead of SDO. Some
    // implementations only sample motion targets from PDO traffic; SDO writes
    // land in the dictionary but never reach the motion engine.
    logger.info("Testing command delivery via RPDO (mode 3, target={} counts/s)", kTestVelocity);
    if (!roboclaw.drive_m1_speed(kTestVelocity, ec)) {
      logger.warn("SDO enable before RPDO test failed: {}", ec.message());
    }
    if (const size_t sent = roboclaw.send_test_rpdos(3, kTestVelocity, 0, 0, ec); sent == 0) {
      logger.warn("No RPDOs sent: {}", ec.message());
    } else {
      for (int poll = 1; poll <= kVelocityPolls; ++poll) {
        std::this_thread::sleep_for(1s);
        int32_t position = 0;
        int32_t speed = 0;
        uint8_t encoder_status = 0;
        uint8_t speed_status = 0;
        if (roboclaw.read_encoder_m1(position, encoder_status, ec) &&
            roboclaw.read_speed_m1(speed, speed_status, ec)) {
          std::error_code demand_ec;
          int32_t velocity_demand = 0;
          const bool demand_ok = roboclaw.read_velocity_demand_m1(velocity_demand, demand_ec);
          logger.info("RPDO velocity poll {}/{}: position={}, speed={} counts/s, demand={}", poll,
                      kVelocityPolls, position, speed,
                      demand_ok ? std::to_string(velocity_demand) : "n/a");
        }
      }
      roboclaw.send_test_rpdos(3, 0, 0, 0, ec);
      roboclaw.drive_m1_speed(0, ec);
    }

    // The MCP clamps position targets to the position PID's [min, max]
    // range, which is [0, 0] from the factory -- widen it over CAN first.
    if (!roboclaw.configure_m1_position_range(-2'000'000'000, 2'000'000'000, ec)) {
      logger.warn("Could not configure the M1 position range over CAN: {}", ec.message());
    }

    // Test 2: profile position move. On top of the velocity PID this needs the
    // position loop configured on the MCP (position PID gains, max speed).
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
            // The demand value is the profile generator's instantaneous
            // command: if it ramps toward the target while the actual position
            // stays put, the profile is running but the position loop is not
            // driving the motor (PID gains / max speed unset on the MCP).
            std::error_code demand_ec;
            int32_t position_demand = 0;
            const bool demand_ok = roboclaw.read_position_demand_m1(position_demand, demand_ec);
            logger.info("Position poll {}/{}: statusword=0x{:04X}, M1 position={}, demand={}",
                        poll, kPositionPolls, status, position,
                        demand_ok ? std::to_string(position_demand) : "n/a");
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
