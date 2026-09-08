#include <chrono>
#include <memory>
#include <thread>

#include "cli.hpp"
#include "logger.hpp"
#include "motor_x6_60.hpp"

using namespace std::chrono_literals;

namespace {
constexpr gpio_num_t kCanRxGpio = GPIO_NUM_16;
constexpr gpio_num_t kCanTxGpio = GPIO_NUM_17;
constexpr uint8_t kMotorId = 1;
constexpr float kGearRatio = 6.0f;
}

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "MotorX660Example", .level = espp::Logger::Verbosity::INFO});

  MotorCanBus can_bus(kCanRxGpio, kCanTxGpio);
  if (!can_bus.start()) {
    logger.error("Failed to start CAN bus");
    return;
  }

  MotorX660::CommunicationFunction communicate =
      [&can_bus](const MotorPacket &command, MotorPacket &response, uint32_t timeout_ms) {
        return timeout_ms == 0 ? can_bus.send(command)
                               : can_bus.request(command, response, timeout_ms);
      };
  MotorX660 actuator(communicate, kMotorId, kGearRatio);
  actuator.set_position_limits(-1800.0f, 1800.0f);

  auto root_menu = std::make_unique<cli::Menu>("motor_x6_60_example");
  root_menu->Insert(
      "set",
      [&actuator](std::ostream &out, float degrees, float rpm) {
        out << (actuator.set_position(degrees, rpm) ? "Position command sent.\n"
                                                     : "Position command failed.\n");
      },
      "Move to a virtual position: set <degrees> <rpm>");
  root_menu->Insert(
      "set_abs",
      [&actuator](std::ostream &out, float degrees, float rpm) {
        out << (actuator.set_absolute_position(degrees, rpm)
                    ? "Absolute position command sent.\n"
                    : "Absolute position command failed.\n");
      },
      "Move to an absolute position: set_abs <degrees> <rpm>");
  root_menu->Insert(
      "get",
      [&actuator](std::ostream &out) {
        out << "Virtual position: " << actuator.get_position() << " deg\n";
      },
      "Read the virtual position: get");
  root_menu->Insert(
      "zero",
      [&actuator](std::ostream &out) {
        actuator.zero_position();
        out << "Virtual position set to zero.\n";
      },
      "Set the current command reference to zero: zero");
  root_menu->Insert(
      "status",
      [&actuator](std::ostream &out) {
        MotorX660::Status status{};
        if (actuator.read_status(status)) {
          out << "Temperature=" << status.temperature_c << " C, velocity="
              << status.velocity_rpm << " RPM, angle=" << status.angle_degrees << " deg\n";
        } else {
          out << "Status read failed.\n";
        }
      },
      "Read motor status: status");
  root_menu->Insert(
      "encoder",
      [&actuator](std::ostream &out) {
        int32_t encoder_value = 0;
        if (actuator.read_multi_turn_encoder(encoder_value)) {
          out << "Multi-turn encoder: " << encoder_value << " counts\n";
        } else {
          out << "Multi-turn encoder read failed.\n";
        }
      },
      "Read the multi-turn encoder count: encoder");
  root_menu->Insert(
      "stop",
      [&actuator](std::ostream &out) {
        out << (actuator.stop() ? "Motor stopped.\n" : "Stop command failed.\n");
      },
      "Stop the motor: stop");
  root_menu->Insert(
      "disable",
      [&actuator](std::ostream &out) {
        out << (actuator.disable() ? "Motor disabled.\n" : "Disable command failed.\n");
      },
      "Disable motor output: disable");
  root_menu->Insert(
      "release",
      [&actuator](std::ostream &out) {
        out << (actuator.release_brake() ? "Brake released.\n" : "Brake release failed.\n");
      },
      "Release the motor brake: release");
  root_menu->Insert(
      "lock",
      [&actuator](std::ostream &out) {
        out << (actuator.lock_brake() ? "Brake locked.\n" : "Brake lock failed.\n");
      },
      "Lock the motor brake: lock");

  static auto cli = std::make_unique<cli::Cli>(std::move(root_menu));
  std::thread([cli_ptr = cli.get()] {
    espp::Cli input(*cli_ptr);
    input.Start();
  }).detach();

  logger.info("MotorX660 CLI ready: set, get, zero, status, encoder, stop, disable, release, lock");
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}