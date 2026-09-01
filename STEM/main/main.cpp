#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "cli.hpp"
#include "esp32-p4-eth.hpp"
#include "logger.hpp"
#include "motor_actuator.hpp"
#include "paired_actuator.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  auto &board = espp::Esp32P4Eth::get();
  espp::Logger logger({.tag = "STEM", .level = espp::Logger::Verbosity::INFO});

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
        MotorActuator(communicate, 2),
        MotorActuator(communicate, 3),
        MotorActuator(communicate, 4),
        MotorActuator(communicate, 5),
        MotorActuator(communicate, 6),
    };
    logger.info("Six motor actuators configured with IDs 1-6");
    PairedActuator right_actuator_pair(actuators[0], actuators[1]);
    PairedActuator left_actuator_pair(actuators[2], actuators[3]);
    PairedActuator seat_actuator_pair(actuators[4], actuators[5]);
    logger.info("Three reversed actuator pairs configured: 1-2, 3-4, 5-6");
    seat_actuator_pair.zero_position();
    right_actuator_pair.zero_position();
    left_actuator_pair.zero_position();

    seat_actuator_pair.set_position_limits(-60.0f, 60.0f);
    right_actuator_pair.set_position_limits(0.0f,60.0f);
    left_actuator_pair.set_position_limits(-60.0f, 0.0f);

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


    auto root_menu = std::make_unique<cli::Menu>("STEM");
    root_menu->Insert(
        "set",
        [&right_actuator_pair, &left_actuator_pair,
         &seat_actuator_pair](std::ostream &out, const std::string &pair_name, float degrees) {
          constexpr float rpm = 5.0f;
          bool set_ok = false;
          if (pair_name == "right") {
            set_ok = right_actuator_pair.set_position(degrees, rpm);
          } else if (pair_name == "left") {
            set_ok = left_actuator_pair.set_position(degrees, rpm);
          } else if (pair_name == "seat") {
            set_ok = seat_actuator_pair.set_position(degrees, rpm);
          } else {
            out << "Unknown pair '" << pair_name << "'. Use right, left, or seat.\n";
            return;
          }

          if (set_ok) {
            out << pair_name << " position set to " << degrees << " deg at " << rpm << " RPM.\n";
          } else {
            out << "Failed to set " << pair_name << " position.\n";
          }
        },
        "Set one pair position at 5 RPM: set <right|left|seat> <degrees>");
    root_menu->Insert(
        "get",
        [&actuators](std::ostream &out) {
          for (const auto &actuator : actuators) {
            out << "Motor " << static_cast<int>(actuator.get_motor_id())
                << ": position=" << actuator.get_position() << " deg\n";
          }
        },
        "Get virtual positions for all motors: get");
    root_menu->Insert(
        "status",
        [&actuators](std::ostream &out) {
          for (auto &actuator : actuators) {
            MotorActuator::Status status{};
            if (actuator.read_status(status)) {
              out << "Motor " << static_cast<int>(actuator.get_motor_id())
                  << ": temperature=" << status.temperature_c << " C, torque=" << status.torque_raw
                  << ", velocity=" << status.velocity_rpm << " RPM, angle="
                  << status.angle_degrees << " deg\n";
            } else {
              out << "Motor " << static_cast<int>(actuator.get_motor_id())
                  << ": status read failed\n";
            }
          }
        },
        "Get status for all motors: status");
    root_menu->Insert(
        "release",
        [&right_actuator_pair, &left_actuator_pair, &seat_actuator_pair](std::ostream &out) {
          const bool right_ok = right_actuator_pair.release_brake();
          const bool left_ok = left_actuator_pair.release_brake();
          const bool seat_ok = seat_actuator_pair.release_brake();
          if (right_ok && left_ok && seat_ok) {
            out << "All motor brakes released.\n";
          } else {
            out << "Failed to release brakes for:";
            if (!right_ok) out << " right";
            if (!left_ok) out << " left";
            if (!seat_ok) out << " seat";
            out << ".\n";
          }
        },
        "Release brakes for all actuator pairs: release");
    root_menu->Insert(
        "zero",
        [&right_actuator_pair, &left_actuator_pair,
         &seat_actuator_pair](std::ostream &out, const std::string &pair_name) {
          if (pair_name == "right") {
            right_actuator_pair.zero_position();
          } else if (pair_name == "left") {
            left_actuator_pair.zero_position();
          } else if (pair_name == "seat") {
            seat_actuator_pair.zero_position();
          } else {
            out << "Unknown pair '" << pair_name << "'. Use right, left, or seat.\n";
            return;
          }
          out << pair_name << " actuator pair zeroed.\n";
        },
        "Set a pair's current position as virtual zero: zero <right|left|seat>");
    static auto cli = std::make_unique<cli::Cli>(std::move(root_menu));
    std::thread([cli_ptr = cli.get()] {
      espp::Cli input(*cli_ptr);
      input.Start();
    }).detach();
    logger.info("Motor CLI ready: set <right|left|seat> <degrees>, get, status, release, zero <right|left|seat>");

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
}
