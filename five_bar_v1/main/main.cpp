#include <chrono>
#include <memory>
#include <thread>

#include "cli.hpp"
#include "esp32-p4-eth.hpp"
#include "logger.hpp"
#include "motor_actuator.hpp"
#include "paired_actuator.hpp"

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
        MotorActuator(communicate, 2),
        MotorActuator(communicate, 3),
        MotorActuator(communicate, 4),
        MotorActuator(communicate, 5),
        MotorActuator(communicate, 6),
    };
    logger.info("Six motor actuators configured with IDs 1-6");
    PairedActuator actuator_pairs[] = {
      PairedActuator(actuators[0], actuators[1]),
      PairedActuator(actuators[2], actuators[3]),
      PairedActuator(actuators[4], actuators[5]),
    };
    logger.info("Three reversed actuator pairs configured: 1-2, 3-4, 5-6");

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
    PairedActuator &actuator_pair = actuator_pairs[0];
    uint8_t reported_motor_id = 0;
    if (actuator.read_motor_id(reported_motor_id)) {
      logger.info("Motor ID read test: motor reports logical ID {}", reported_motor_id);
    } else {
      logger.warn("Motor ID read test failed");
    }
    auto root_menu = std::make_unique<cli::Menu>("five_bar_v1");
    root_menu->Insert(
        "calibration",
        [&actuator_pair](std::ostream &out) {
          actuator_pair.zero_position();
          out << "Virtual paired actuator position calibrated to 0 degrees.\n";
        },
        "Set the current position as virtual zero: calibration");
    root_menu->Insert(
        "set_position",
        [&actuator_pair](std::ostream &out, float degrees, float rpm) {
          if (actuator_pair.set_position(degrees, rpm)) {
            out << "Paired target position set to " << degrees << " degrees at " << rpm << " RPM.\n";
          } else {
            out << "Failed to set motor position.\n";
          }
        },
        "Move to a virtual position: set_position <degrees> <rpm>");
    static auto cli = std::make_unique<cli::Cli>(std::move(root_menu));
    std::thread([cli_ptr = cli.get()] {
      espp::Cli input(*cli_ptr);
      input.Start();
    }).detach();
    logger.info("Motor CLI ready: calibration, set_position <degrees> <rpm>");

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
