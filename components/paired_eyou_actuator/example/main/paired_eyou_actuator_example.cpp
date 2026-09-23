#include "paired_eyou_actuator.hpp"

#include <chrono>
#include <memory>
#include <thread>

#include "can_bus.hpp"
#include "cli.hpp"
#include "logger.hpp"

using namespace std::chrono_literals;

namespace {
constexpr gpio_num_t kCanRxGpio = GPIO_NUM_16;
constexpr gpio_num_t kCanTxGpio = GPIO_NUM_17;
constexpr uint8_t kPrimaryNodeId = 1;
constexpr uint8_t kSecondaryNodeId = 2;
}

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "PairedEyouActuatorExample", .level = espp::Logger::Verbosity::INFO});

  CanBus bus(kCanRxGpio, kCanTxGpio);
  EyouMotor primary(bus, kPrimaryNodeId);
  EyouMotor secondary(bus, kSecondaryNodeId);
  if (!bus.start()) {
    logger.error("Failed to start CAN bus");
    return;
  }

  PairedEyouActuator pair(primary, secondary);

  const EyouMotor::ProfilePositionConfig profile{
      .velocity_degrees_per_second = 60.0f,
      .acceleration_degrees_per_second_squared = 60.0f,
      .deceleration_degrees_per_second_squared = 60.0f,
  };
  if (!pair.configure_profile_position(profile)) {
    logger.error("Failed to configure profile-position mode");
    return;
  }

  auto root_menu = std::make_unique<cli::Menu>("paired_eyou_actuator_example");
  root_menu->Insert(
      "get_position",
      [&pair](std::ostream &out) {
        std::array<float, 2> position{};
        if (pair.get_position(position)) {
          out << "primary=" << position[0] << " deg, secondary=" << position[1] << " deg\n";
        } else {
          out << "Failed to read position.\n";
        }
      },
      "Read both actuator positions: get_position");
  root_menu->Insert(
      "zero",
      [&pair](std::ostream &out) {
        out << (pair.zero() ? "Software zero position set.\n" : "Failed to set software zero.\n");
      },
      "Set the software zero reference for both actuators: zero");
  root_menu->Insert(
      "enable",
      [&pair](std::ostream &out) {
        out << (pair.enable_drive() ? "Drives enabled.\n" : "Failed to enable drives.\n");
      },
      "Enable both DS402 drives: enable");
  root_menu->Insert(
      "disable",
      [&pair](std::ostream &out) {
        out << (pair.disable_drive() ? "Drives disabled.\n" : "Failed to disable drives.\n");
      },
      "Disable both DS402 drives: disable");
  root_menu->Insert(
      "fault_reset",
      [&pair](std::ostream &out) {
        out << (pair.reset_fault() ? "Faults reset.\n" : "Failed to reset faults.\n");
      },
      "Reset a DS402 fault on both actuators: fault_reset");
  root_menu->Insert(
      "set_position",
      [&pair](std::ostream &out, float position) {
        out << (pair.set_position(position) ? "Absolute position command sent.\n"
                                            : "Absolute position command failed.\n");
      },
      "Move the pair to a mirrored absolute position: set_position <degrees>");
  root_menu->Insert(
      "move_incremental",
      [&pair](std::ostream &out, float increment) {
        out << (pair.move_incremental(increment) ? "Incremental position command sent.\n"
                                                  : "Incremental position command failed.\n");
      },
      "Move the pair by a mirrored increment: move_incremental <degrees>");
  root_menu->Insert(
      "set_limits",
      [&pair](std::ostream &out, float minimum_degrees, float maximum_degrees) {
        out << (pair.set_position_limits(minimum_degrees, maximum_degrees)
                    ? "Position limits set.\n"
                    : "Failed to set position limits.\n");
      },
      "Set the primary's position limits (secondary is limited to the negated range): "
      "set_limits <minimum_degrees> <maximum_degrees>");

  static auto cli = std::make_unique<cli::Cli>(std::move(root_menu));
  std::thread([cli_ptr = cli.get()] {
    espp::Cli input(*cli_ptr);
    input.Start();
  }).detach();

  logger.info("CLI ready: get_position, zero, enable, disable, fault_reset, "
              "set_position <degrees>, move_incremental <degrees>, "
              "set_limits <minimum_degrees> <maximum_degrees>");
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
