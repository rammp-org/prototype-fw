#include <chrono>
#include <memory>
#include <thread>

#include "cli.hpp"
#include "logger.hpp"
#include "motor_actuator.hpp"

using namespace std::chrono_literals;

namespace {
constexpr gpio_num_t kCanRxGpio = GPIO_NUM_16;
constexpr gpio_num_t kCanTxGpio = GPIO_NUM_17;
constexpr uint8_t kMotorId = 1;
}

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "MotorActuatorExample", .level = espp::Logger::Verbosity::INFO});

  MotorCanBus can_bus(kCanRxGpio, kCanTxGpio);
  if (!can_bus.start()) {
    logger.error("Failed to start CAN bus");
    return;
  }

  MotorActuator::CommunicationFunction communicate =
      [&can_bus](const MotorPacket &command, MotorPacket &response, uint32_t timeout_ms) {
        return timeout_ms == 0 ? can_bus.send(command)
                               : can_bus.request(command, response, timeout_ms);
      };
  MotorActuator actuator(communicate, kMotorId);

  actuator.set_position_limits(-180.0f, 180.0f);
  actuator.zero_position();

  auto root_menu = std::make_unique<cli::Menu>("motor_actuator_example");
  root_menu->Insert(
      "set",
      [&actuator](std::ostream &out, float degrees, float rpm) {
        if (actuator.set_position(degrees, rpm)) {
          out << "Moving to virtual position " << degrees << " deg at " << rpm << " RPM.\n";
        } else {
          out << "Position command failed.\n";
        }
      },
      "Move to a virtual position: set <degrees> <rpm>");
  root_menu->Insert(
      "get",
      [&actuator](std::ostream &out) {
        out << "Virtual position: " << actuator.get_position() << " deg\n";
      },
      "Get the commanded virtual position: get");
  root_menu->Insert(
      "zero",
      [&actuator](std::ostream &out) {
        actuator.zero_position();
        out << "Current physical position is now virtual zero.\n";
      },
      "Set the current physical position as virtual zero: zero");
  root_menu->Insert(
      "limits",
      [&actuator](std::ostream &out, float minimum, float maximum) {
        if (actuator.set_position_limits(minimum, maximum)) {
          out << "Position limits set to [" << minimum << ", " << maximum << "] deg.\n";
        } else {
          out << "Invalid position limits.\n";
        }
      },
      "Set virtual position limits: limits <minimum_degrees> <maximum_degrees>");
  root_menu->Insert(
      "release",
      [&actuator](std::ostream &out) {
        out << (actuator.release_brake() ? "Brake released.\n" : "Failed to release brake.\n");
      },
      "Release the motor brake: release");
  root_menu->Insert(
      "stop",
      [&actuator](std::ostream &out) {
        out << (actuator.stop() ? "Motor stopped.\n" : "Failed to stop motor.\n");
      },
      "Stop the motor: stop");
  root_menu->Insert(
      "status",
      [&actuator](std::ostream &out) {
        MotorActuator::Status status{};
        if (actuator.read_status(status)) {
          out << "Temperature=" << status.temperature_c << " C, velocity=" << status.velocity_rpm
              << " RPM, angle=" << status.angle_degrees << " deg\n";
        } else {
          out << "Status read failed.\n";
        }
      },
      "Read motor status: status");

  static auto cli = std::make_unique<cli::Cli>(std::move(root_menu));
  std::thread([cli_ptr = cli.get()] {
    espp::Cli input(*cli_ptr);
    input.Start();
  }).detach();

  logger.info("CLI ready: set, get, zero, limits, release, stop, status");
  while (true) {
    // for (int sample = 1; sample <= 10; ++sample) {
    //   MotorActuator::Status status{};
    //   actuator.read_status(status);
    // //   if (actuator.read_status(status)) {
    // //     logger.info("Status {}/10: temperature={} C, velocity={} RPM, angle={} deg", sample,
    // //                 status.temperature_c, status.velocity_rpm, status.angle_degrees);
    // //   } else {
    // //     logger.warn("Status {}/10 read failed", sample);
    // //   }
    // }
    std::this_thread::sleep_for(1s);
  }
}