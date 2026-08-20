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
      logger.info("Motor status: temperature={} C torque_raw={} velocity_raw={} angle_raw={}",
                  status.temperature_c, status.torque_raw, status.velocity_raw,
                  status.angle_raw);
      const int32_t position_before = status.angle_raw;

      constexpr int32_t kTestVelocityRaw = 100000;
      constexpr int kTestDurationSeconds = 5;
      logger.warn("Releasing motor brake");
      if (actuator.release_brake() && actuator.send_velocity(kTestVelocityRaw)) {
        logger.warn("Starting five-second motor test at velocity_raw={}", kTestVelocityRaw);
        for (int second = 1; second <= kTestDurationSeconds; ++second) {
          std::this_thread::sleep_for(1s);
          int16_t velocity_raw = 0;
          if (actuator.read_velocity(velocity_raw)) {
            logger.info("Motor velocity after {}s: {} raw", second, velocity_raw);
          } else {
            logger.warn("Motor velocity read failed after {}s", second);
          }
        }
        if (actuator.hold()) {
          logger.info("Motor stopped after {} seconds", kTestDurationSeconds);
        } else {
          logger.error("Failed to stop motor after test");
        }

        MotorActuator::Status final_status{};
        if (actuator.read_status(final_status)) {
          const int32_t position_delta = final_status.angle_raw - position_before;
          logger.info("Motor position: before={} after={} delta={} raw", position_before,
                      final_status.angle_raw, position_delta);
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
