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
    MotorActuator::Status status{};
    if (actuator.read_status(status)) {
      logger.info("Motor status: temperature={} C torque_raw={} velocity={} RPM angle={} deg",
                  status.temperature_c, status.torque_raw, status.velocity_rpm,
                  status.angle_degrees);
        constexpr float kTargetPositionDegrees = 360.0f;
        constexpr float kPositionSpeedRpm = 1000.0f / (36.0f * 6.0f);
        constexpr int kTestDurationSeconds = 5;
      logger.warn("Releasing motor brake");
        actuator.zero_position();
        if (actuator.release_brake() &&
          actuator.set_position(kTargetPositionDegrees, kPositionSpeedRpm)) {
        logger.warn("Moving output shaft to virtual position {} degrees at {} RPM",
                    kTargetPositionDegrees, kPositionSpeedRpm);
        for (int second = 1; second <= kTestDurationSeconds; ++second) {
          std::this_thread::sleep_for(1s);
          MotorActuator::Status progress_status{};
          if (actuator.read_status(progress_status)) {
            logger.info("Motor status after {}s: output angle={} deg velocity={} RPM", second,
                        progress_status.angle_degrees, progress_status.velocity_rpm);
          } else {
            logger.warn("Motor status read failed after {}s", second);
          }
        }
        if (actuator.stop()) {
          logger.info("Motor stopped after {} seconds", kTestDurationSeconds);
        } else {
          logger.error("Failed to stop motor after test");
        }

        MotorActuator::Status final_status{};
        if (actuator.read_status(final_status)) {
          logger.info("Output position after incremental move: {} deg", final_status.angle_degrees);
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
