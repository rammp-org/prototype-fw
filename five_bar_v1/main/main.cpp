#include <chrono>
#include <thread>

#include "esp32-p4-eth.hpp"
#include "logger.hpp"
#include "motor_actuator.hpp"

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

  MotorActuator actuator(logger, GPIO_NUM_16, GPIO_NUM_17);
  if (actuator.start()) {
    std::array<char, 8> motor_model{};
    if (actuator.read_motor_model(motor_model)) {
      logger.info("Motor model: {}", motor_model.data());
    }
    uint32_t version_date = 0;
    if (actuator.read_software_version_date(version_date)) {
      logger.info("Motor software version date: {}", version_date);
    }
    int32_t raw_position = 0;
    if (actuator.read_multi_turn_raw_position(raw_position)) {
      logger.info("0x61 raw multi-turn position: {}", raw_position);
    }
    int32_t zero_offset = 0;
    if (actuator.read_multi_turn_zero_offset(zero_offset)) {
      logger.info("0x62 multi-turn zero offset: {}", zero_offset);
    }
    logger.info("0x63 and 0x64 write commands are available but not run automatically");
    MotorActuator::Status status{};
    if (actuator.read_status(status)) {
      logger.info("Motor status: temperature={} C torque_raw={} velocity_raw={} angle_raw={}",
                  status.temperature_c, status.torque_raw, status.velocity_raw,
                  status.angle_raw);
        constexpr int32_t kIncrementalPositionCentidegrees = 36000;
        constexpr uint16_t kPositionSpeedDps = 1000;
        constexpr int kTestDurationSeconds = 5;
      logger.warn("Releasing motor brake");
        if (actuator.release_brake() &&
          actuator.send_incremental_position(kIncrementalPositionCentidegrees,
                                              kPositionSpeedDps)) {
        logger.warn("Moving motor incrementally by {} degrees at {} dps",
                    kIncrementalPositionCentidegrees / 100.0f, kPositionSpeedDps);
        for (int second = 1; second <= kTestDurationSeconds; ++second) {
          std::this_thread::sleep_for(1s);
          MotorActuator::Status progress_status{};
          if (actuator.read_status(progress_status)) {
            logger.info("Motor status after {}s: angle={} deg velocity={} dps", second,
                        progress_status.angle_raw, progress_status.velocity_raw);
          } else {
            logger.warn("Motor status read failed after {}s", second);
          }
          actuator.send_incremental_position(kIncrementalPositionCentidegrees,
                                              kPositionSpeedDps);
        }
        if (actuator.stop()) {
          logger.info("Motor stopped after {} seconds", kTestDurationSeconds);
        } else {
          logger.error("Failed to stop motor after test");
        }

        MotorActuator::Status final_status{};
        if (actuator.read_status(final_status)) {
          logger.info("Motor position after incremental move: {} deg", final_status.angle_raw);
        } else {
          logger.warn("Failed to read motor position after the move");
        }
      }
    }
  }

  bool have_ethernet_status = false;
  bool last_ethernet_status = false;
  while (true) {
    const bool ethernet_status = board.is_ethernet_connected();
    if (!have_ethernet_status || ethernet_status != last_ethernet_status) {
      logger.info("Ethernet connected: {}", ethernet_status);
      last_ethernet_status = ethernet_status;
      have_ethernet_status = true;
    }
    std::this_thread::sleep_for(2000ms);
  }
}
