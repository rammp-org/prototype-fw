#include "eyou_motor.hpp"
#include "canopen_bus.hpp"

#include <chrono>
#include <memory>
#include <thread>

#include "cli.hpp"
#include "logger.hpp"

using namespace std::chrono_literals;

namespace {
constexpr gpio_num_t kCanRxGpio = GPIO_NUM_16;
constexpr gpio_num_t kCanTxGpio = GPIO_NUM_17;
constexpr uint8_t kNodeId = 105;
}

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "EyouMotorExample", .level = espp::Logger::Verbosity::INFO});
  CanopenBus bus(kCanRxGpio, kCanTxGpio);
  EyouMotor motor(bus, kNodeId);
  if (!bus.start()) {
    logger.error("Failed to start CAN bus");
    return;
  }

  if (!motor.configure_profile_position({
      .velocity_degrees_per_second = 60.0f,
      .acceleration_degrees_per_second_squared = 60.0f,
      .deceleration_degrees_per_second_squared = 60.0f,
      })) {
        logger.error("Failed to configure profile-position mode");
    return;
  }

      auto root_menu = std::make_unique<cli::Menu>("eyou_motor_example");
      root_menu->Insert(
          "get_position",
          [&motor](std::ostream &out) {
            float position = 0.0f;
            if (motor.get_position(position)) {
              out << "Output position: " << position << " degrees\n";
            } else {
              out << "Failed to read output position.\n";
            }
          },
          "Read output position: get_position");
      root_menu->Insert(
          "status",
          [&motor](std::ostream &out) {
            uint16_t statusword = 0;
            int8_t mode = 0;
            if (!motor.get_statusword(statusword) || !motor.get_operating_mode(mode)) {
              out << "Failed to read drive status.\n";
              return;
            }
            out << "Mode=" << static_cast<int>(mode) << ", statusword=0x" << std::hex
                << statusword << std::dec << " (" << EyouMotor::ds402_state_name(statusword)
                << "), operation_enabled="
                << (((statusword & 0x006F) == 0x0027) ? "yes" : "no")
                << ", fault=" << ((statusword & 0x0008) ? "yes" : "no")
                << ", internal_limit=" << ((statusword & 0x0800) ? "yes" : "no") << "\n";
          },
          "Read PP mode and DS402 status: status");
      root_menu->Insert(
          "enable",
          [&motor](std::ostream &out) {
            out << (motor.enable_drive() ? "Drive enabled.\n" : "Failed to enable drive.\n");
          },
          "Enable the DS402 drive: enable");
      root_menu->Insert(
          "disable",
          [&motor](std::ostream &out) {
            out << (motor.disable_drive() ? "Drive disabled.\n" : "Failed to disable drive.\n");
          },
          "Disable the DS402 drive: disable");
      root_menu->Insert(
          "fault_reset",
          [&motor](std::ostream &out) {
            out << (motor.reset_fault() ? "Drive fault reset.\n" : "Failed to reset drive fault.\n");
          },
          "Reset a DS402 drive fault: fault_reset");
      root_menu->Insert(
          "fault",
          [&motor](std::ostream &out) {
            uint16_t statusword = 0;
            uint16_t error_code = 0;
            if (!motor.get_statusword(statusword) || !motor.get_error_code(error_code)) {
              out << "Failed to read drive fault information.\n";
              return;
            }
            out << "Statusword=0x" << std::hex << statusword << ", error_code=0x" << error_code
                << std::dec << "\n";
          },
          "Read DS402 status and active error code: fault");
      root_menu->Insert(
          "set_position",
          [&motor](std::ostream &out, float position) {
            out << (motor.move_absolute(position) ? "Absolute position command sent.\n"
                                                  : "Absolute position command failed.\n");
          },
          "Move to an absolute output position: set_position <degrees>");
      root_menu->Insert(
          "move_incremental",
          [&motor](std::ostream &out, float increment) {
            out << (motor.move_incremental(increment) ? "Incremental position command sent.\n"
                                                      : "Incremental position command failed.\n");
          },
          "Move by an output increment: move_incremental <degrees>");

      static auto cli = std::make_unique<cli::Cli>(std::move(root_menu));
      std::thread([cli_ptr = cli.get()] {
        espp::Cli input(*cli_ptr);
        input.Start();
      }).detach();

      logger.info("CLI ready: status, enable, disable, fault, fault_reset, get_position, "
          "set_position <degrees>, move_incremental <degrees>");
  while (true) {
        std::this_thread::sleep_for(1s);
  }
}
