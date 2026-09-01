#include <chrono>
#include <memory>
#include <thread>

#include "cli.hpp"
#include "logger.hpp"
#include "motor_actuator.hpp"
#include "paired_actuator.hpp"

using namespace std::chrono_literals;

namespace {
constexpr gpio_num_t kCanRxGpio = GPIO_NUM_16;
constexpr gpio_num_t kCanTxGpio = GPIO_NUM_17;
}

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "PairedActuatorExample", .level = espp::Logger::Verbosity::INFO});

  MotorCanBus can_bus(kCanRxGpio, kCanTxGpio);
  if (!can_bus.start()) {
    logger.error("Failed to start CAN bus");
    return;
  }

  MotorActuator::CommunicationFunction communicate =
      [&can_bus](const MotorPacket &command, MotorPacket &response, uint32_t timeout_ms) {
        if (timeout_ms == 0) {
          return can_bus.send(command);
        }
        return can_bus.request(command, response, timeout_ms);
      };

  MotorActuator primary(communicate, 1);
  MotorActuator secondary(communicate, 2);
  PairedActuator pair(primary, secondary);

  PairedActuator::Status status{};
  if (pair.read_status(status)) {
    logger.info("Pair IDs 1-2: primary={} RPM secondary={} RPM",
                status.motors[0].velocity_rpm, status.motors[1].velocity_rpm);
  } else {
    logger.warn("Pair IDs 1-2 status read failed");
  }

  pair.zero_position();
  pair.set_position_limits(-45.0f, 45.0f);
  const auto initial_positions = pair.get_position();
  logger.info("Pair 1 virtual positions: primary={} deg secondary={} deg", initial_positions[0],
              initial_positions[1]);

  auto root_menu = std::make_unique<cli::Menu>("paired_actuator_example");
  root_menu->Insert(
      "get",
      [&pair, &logger](std::ostream &out) {
        const auto positions = pair.get_position();
        out << "Primary: " << positions[0] << " deg, Secondary: " << positions[1] << " deg\n";
      },
      "Get the current virtual positions: get");
  root_menu->Insert(
      "status",
      [&pair](std::ostream &out) {
        PairedActuator::Status status{};
        if (!pair.read_status(status)) {
          out << "Failed to read actuator status.\n";
          return;
        }

        for (size_t index = 0; index < status.motors.size(); index++) {
          const auto &motor = status.motors[index];
          out << (index == 0 ? "Primary" : "Secondary") << ": temperature="
              << motor.temperature_c << " C, torque=" << motor.torque_raw
              << ", velocity=" << motor.velocity_rpm << " RPM, angle="
              << motor.angle_degrees << " deg\n";
        }
      },
      "Get the status of all actuators: status");
  root_menu->Insert(
      "set",
      [&pair](std::ostream &out, float degrees, float rpm) {
        if (pair.set_position(degrees, rpm)) {
          out << "Position set to " << degrees << " degrees at " << rpm << " RPM.\n";
        } else {
          out << "Failed to set position.\n";
        }
      },
      "Set the virtual position: set <degrees> <rpm>");
  root_menu->Insert(
      "stop",
      [&pair](std::ostream &out) {
        if (pair.stop()) {
          out << "Motors stopped.\n";
        } else {
          out << "Failed to stop motors.\n";
        }
      },
      "Stop both motors: stop");
  root_menu->Insert(
      "release",
      [&pair](std::ostream &out) {
        if (pair.release_brake()) {
          out << "Motor brakes released.\n";
        } else {
          out << "Failed to release motor brakes.\n";
        }
      },
      "Release both motor brakes: release");
  root_menu->Insert(
      "zero",
      [&pair](std::ostream &out) {
        pair.zero_position();
        out << "Virtual position zeroed.\n";
      },
      "Calibrate to current position as zero: zero");
  root_menu->Insert(
      "limit",
      [&pair](std::ostream &out, float min_deg, float max_deg) {
        if (pair.set_position_limits(min_deg, max_deg)) {
          out << "Position limits set to [" << min_deg << ", " << max_deg << "] degrees.\n";
        } else {
          out << "Failed to set position limits.\n";
        }
      },
      "Set position limits: limit <min_degrees> <max_degrees>");

  static auto cli = std::make_unique<cli::Cli>(std::move(root_menu));
  std::thread([cli_ptr = cli.get()] {
    espp::Cli input(*cli_ptr);
    input.Start();
  }).detach();

  logger.info("Paired Actuator CLI ready: get, status, set, stop, release, zero, limit");

  while (true) {
    std::this_thread::sleep_for(1000ms);
  }
}
