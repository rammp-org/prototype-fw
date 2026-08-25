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

  MotorCanBus can_bus(GPIO_NUM_16, GPIO_NUM_17);
  if (can_bus.start()) {
    MotorActuator::CommunicationFunction communicate =
        [&can_bus](const MotorPacket &command, MotorPacket &response, uint32_t timeout_ms) {
          if (timeout_ms == 0) return can_bus.send(command);
          return can_bus.request(command, response, timeout_ms);
        };
    MotorActuator actuators[] = {
        MotorActuator(communicate, 1),
        // MotorActuator(communicate, 2),
        // MotorActuator(communicate, 3),
        // MotorActuator(communicate, 4),
        // MotorActuator(communicate, 5),
        // MotorActuator(communicate, 6),
        // MotorActuator(communicate, 7),
        // MotorActuator(communicate, 8),
        // MotorActuator(communicate, 9),
        // MotorActuator(communicate, 10),
        // MotorActuator(communicate, 11),
        // MotorActuator(communicate, 12),
        // MotorActuator(communicate, 13),
        // MotorActuator(communicate, 14),
        // MotorActuator(communicate, 15),
        // MotorActuator(communicate, 16),
        // MotorActuator(communicate, 17),
        // MotorActuator(communicate, 18),
        // MotorActuator(communicate, 19),
        // MotorActuator(communicate, 20),
        // MotorActuator(communicate, 21),
        // MotorActuator(communicate, 22),
        // MotorActuator(communicate, 23),
        // MotorActuator(communicate, 24),
        // MotorActuator(communicate, 25),
        // MotorActuator(communicate, 26),
        // MotorActuator(communicate, 27),
        // MotorActuator(communicate, 28),
        // MotorActuator(communicate, 29),
        // MotorActuator(communicate, 30),
        // MotorActuator(communicate, 31),
        // MotorActuator(communicate, 32),
    };
    logger.info("Twenty motor actuators configured with IDs 1-32");

    // read all motor statuses see which one response. 
    for (auto &actuator : actuators) {
      MotorActuator::Status status{};
      if (actuator.read_status(status)) {
        logger.info("Motor ID {} status: temperature={} C torque_raw={} velocity={} RPM angle={} deg",
                    actuator.get_motor_id(), status.temperature_c, status.torque_raw,
                    status.velocity_rpm, status.angle_degrees);
      } else {
        logger.warn("Motor ID {} status read failed", actuator.get_motor_id());
      }
    }

    MotorActuator &actuator = actuators[0];
    uint8_t reported_motor_id = 0;
    if (actuator.read_motor_id(reported_motor_id)) {
      logger.info("Motor ID read test: motor reports logical ID {}", reported_motor_id);
    } else {
      logger.warn("Motor ID read test failed");
    }
    MotorActuator::Status status{};
    if (actuator.read_status(status)) {
      logger.info("Motor status: temperature={} C torque_raw={} velocity={} RPM angle={} deg",
                  status.temperature_c, status.torque_raw, status.velocity_rpm,
                  status.angle_degrees);
        constexpr float kTargetPositionDegrees = -180.0f;
        constexpr float kPositionSpeedRpm = 1000.0f / (36.0f * 6.0f);
        constexpr int kTestDurationSeconds = 25;
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
