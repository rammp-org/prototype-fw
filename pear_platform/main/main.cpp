#include <array>
#include <chrono>
#include <thread>

#include "esp32-p4-eth.hpp"
#include "logger.hpp"
#include "motor_actuator.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  auto &board = espp::Esp32P4Eth::get();
  espp::Logger logger({.tag = "pear_platform", .level = espp::Logger::Verbosity::INFO});

  logger.info("Bootup");
  if (!board.initialize_ethernet()) {
    logger.error("Failed to initialize Ethernet");
  }

  MotorCanBus can_bus(GPIO_NUM_16, GPIO_NUM_17);
  if (!can_bus.start()) {
    logger.error("Failed to start motor CAN bus");
    return;
  }

  MotorActuator::CommunicationFunction communicate =
      [&can_bus](const MotorPacket &command, MotorPacket &response, uint32_t timeout_ms) {
        if (timeout_ms == 0) return can_bus.send(command);
        return can_bus.request(command, response, timeout_ms);
      };

  std::array<MotorActuator, 4> motors = {
      MotorActuator(communicate, 1),
      MotorActuator(communicate, 2),
      MotorActuator(communicate, 3),
      MotorActuator(communicate, 4),
  };
  logger.info("Four motor actuators configured with IDs 1-4");

  for (auto &motor : motors) {
    MotorActuator::Status status{};
    if (motor.read_status(status)) {
      logger.info("Motor ID {} status: temperature={} C torque_raw={} velocity={} RPM angle={} deg",
                  motor.get_motor_id(), status.temperature_c, status.torque_raw,
                  status.velocity_rpm, status.angle_degrees);
    } else {
      logger.warn("Motor ID {} status read failed", motor.get_motor_id());
    }
  }

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
