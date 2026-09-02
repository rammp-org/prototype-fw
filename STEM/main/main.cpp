#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "cli.hpp"
#include "esp32-p4-eth.hpp"
#include "ik_5bar.hpp"
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
    PairedActuator right_actuator_pair(actuators[3], actuators[2]);
    PairedActuator left_actuator_pair(actuators[1], actuators[0]);
    PairedActuator seat_actuator_pair(actuators[5], actuators[4]);
    logger.info("Three reversed actuator pairs configured: 1-2, 3-4, 5-6");
    seat_actuator_pair.zero_position(false);
    right_actuator_pair.zero_position(false);
    left_actuator_pair.zero_position(false);

    seat_actuator_pair.set_position_limits(-60.0f, 60.0f);
    right_actuator_pair.set_position_limits(-60.0f,60.0f);
    left_actuator_pair.set_position_limits(-60.0f, 60.0f);

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
        "ik",
        [](std::ostream &out, float x_rel, float y_rel) {
          stem::GeometryConfig config{};
          stem::IkSolution solution{};
          if (!stem::solve_ik_for_m3(x_rel, y_rel, solution, config)) {
            out << "IK solution failed for x_rel=" << x_rel << ", y_rel=" << y_rel << "\n";
            return;
          }
          out << "reference=(x=170, y=230, m1_angle=-0.16, m2_angle=143.74, m3_angle=121.23)\n"
              << "M3=(" << solution.m3[0] << ", " << solution.m3[1] << ")\n"
              << "J1=(" << solution.j1[0] << ", " << solution.j1[1] << ")\n"
              << "J2=(" << solution.j2[0] << ", " << solution.j2[1] << ")\n"
              << "J3=(" << solution.j3[0] << ", " << solution.j3[1] << ")\n"
              << "theta1_deg=" << solution.theta1_deg << "\n"
              << "theta2_deg=" << solution.theta2_deg << "\n"
              << "phi1_deg=" << solution.phi1_deg << "\n"
              << "phi2_deg=" << solution.phi2_deg << "\n"
              << "plate_angle_deg=" << solution.plate_angle_deg << "\n"
              << "m3_angle_deg=" << solution.m3_angle_deg << "\n";
        },
        "Solve the 5-bar IK at the raw M3 point: ik <x_rel> <y_rel>");
    root_menu->Insert(
        "ik_ref",
        [](std::ostream &out, float x_rel, float y_rel) {
          stem::GeometryConfig config{};
          stem::IkSolution solution{};
          const stem::IkReference reference = stem::kFullDownReference;
          if (!stem::solve_ik_for_m3_reference(x_rel, y_rel, reference, solution, config)) {
            out << "IK solution failed for x_rel=" << x_rel << ", y_rel=" << y_rel << "\n";
            return;
          }
          out << "reference=(x=170, y=230, m1_angle=-0.16, m2_angle=143.74, m3_angle=121.23)\n"
              << "M3=(" << solution.m3[0] << ", " << solution.m3[1] << ")\n"
              << "J1=(" << solution.j1[0] << ", " << solution.j1[1] << ")\n"
              << "J2=(" << solution.j2[0] << ", " << solution.j2[1] << ")\n"
              << "J3=(" << solution.j3[0] << ", " << solution.j3[1] << ")\n"
              << "theta1_deg=" << solution.theta1_deg << "\n"
              << "theta2_deg=" << solution.theta2_deg << "\n"
              << "phi1_deg=" << solution.phi1_deg << "\n"
              << "phi2_deg=" << solution.phi2_deg << "\n"
              << "plate_angle_deg=" << solution.plate_angle_deg << "\n"
              << "m3_angle_deg=" << solution.m3_angle_deg << "\n";
        },
        "Solve the 5-bar IK relative to the fixed full-down calibration reference: ik_ref <x_rel> <y_rel>");
    auto move_command = [&right_actuator_pair, &left_actuator_pair,
               &seat_actuator_pair](std::ostream &out, float x_rel, float y_rel) {
          constexpr float rpm = 2.0f;
          stem::GeometryConfig config{};
          stem::IkSolution solution{};
          const stem::IkReference reference = stem::kFullDownReference;
          if (!stem::solve_ik_for_m3_reference(x_rel, y_rel, reference, solution, config)) {
            out << "Move failed: IK solution failed for x_rel=" << x_rel << ", y_rel=" << y_rel << "\n";
            return;
          }

          const float left_deg = solution.theta1_deg;
          const float right_deg = solution.theta2_deg;
          const float seat_deg = solution.m3_angle_deg;

          const bool left_ok = left_actuator_pair.set_position(-left_deg, rpm);
          const bool right_ok = right_actuator_pair.set_position(-right_deg, rpm);
          const bool seat_ok = seat_actuator_pair.set_position(-seat_deg, rpm);

          out << "move x=" << x_rel << ", y=" << y_rel << " -> left=" << left_deg
              << " deg, right=" << right_deg << " deg, seat=" << seat_deg << " deg\n";

          if (!left_ok || !right_ok || !seat_ok) {
            out << "Move command issue: ";
            if (!left_ok) out << "left ";
            if (!right_ok) out << "right ";
            if (!seat_ok) out << "seat ";
            out << "set failed.\n";
          }
          };
    root_menu->Insert(
          "move", move_command,
        "Solve IK relative to the fixed reference and move the left/right/seat actuator pairs: move <x_rel> <y_rel>");
    root_menu->Insert(
          "home",
          [move_command](std::ostream &out) { move_command(out, 0.0f, 0.0f); },
          "Move to the home position: home");
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
            right_actuator_pair.zero_position(false);
          } else if (pair_name == "left") {
            left_actuator_pair.zero_position(false);
          } else if (pair_name == "seat") {
            seat_actuator_pair.zero_position(false);
          } else {
            out << "Unknown pair '" << pair_name << "'. Use right, left, or seat.\n";
            return;
          }
          out << pair_name << " actuator pair zeroed without alignment.\n";
        },
        "Set a pair's current position as virtual zero without alignment: zero <right|left|seat>");
    root_menu->Insert(
        "zero_align",
        [&right_actuator_pair, &left_actuator_pair,
         &seat_actuator_pair](std::ostream &out, const std::string &pair_name) {
          if (pair_name == "right") {
            right_actuator_pair.zero_position(true);
          } else if (pair_name == "left") {
            left_actuator_pair.zero_position(true);
          } else if (pair_name == "seat") {
            seat_actuator_pair.zero_position(true);
          } else {
            out << "Unknown pair '" << pair_name << "'. Use right, left, or seat.\n";
            return;
          }
          out << pair_name << " actuator pair zeroed after alignment.\n";
        },
        "Set a pair's current position as virtual zero after alignment: zero_align <right|left|seat>");
    static auto cli = std::make_unique<cli::Cli>(std::move(root_menu));
    std::thread([cli_ptr = cli.get()] {
      espp::Cli input(*cli_ptr);
      input.Start();
    }).detach();
    logger.info("Motor CLI ready: set <right|left|seat> <degrees>, move <x_rel> <y_rel>, home, get, status, release, zero <right|left|seat>, zero_align <right|left|seat>");

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
