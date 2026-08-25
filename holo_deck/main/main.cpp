#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "cli.hpp"
#include "esp32-p4-eth.hpp"
#include "holo_deck_platform.hpp"
#include "logger.hpp"
#include "motor_actuator.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  auto &board = espp::Esp32P4Eth::get();
  espp::Logger logger({.tag = "holo_deck", .level = espp::Logger::Verbosity::INFO});

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

  constexpr std::array<size_t, 4> kMotorIdsByWheel = {
      2, // top left
      4, // top right
      3, // bottom left
      1, // bottom right
  };

  constexpr HoloDeckPlatform::Configuration kPlatformConfiguration = {
      .wheel_diameter_m = 0.21f,
      .half_length_m = 0.406f,
      .half_width_m = 0.267f,
      .wheel_angles_degrees = {30.0f, 150.0f, -30.0f, -150.0f},
      .motor_directions = {1.0f, 1.0f, 1.0f, 1.0f},
  };
  HoloDeckPlatform platform(kPlatformConfiguration);

  auto root_menu = std::make_unique<cli::Menu>("holo_deck");
  root_menu->Insert(
      "set_speed",
      [&motors](std::ostream &out, int motor_id, float rpm) {
        if (motor_id < 1 || motor_id > static_cast<int>(motors.size())) {
          out << "Motor ID must be between 1 and " << motors.size() << ".\n";
          return;
        }
        if (motors[motor_id - 1].send_velocity(rpm)) {
          out << "Motor " << motor_id << " speed set to " << rpm << " RPM.\n";
        } else {
          out << "Failed to set motor " << motor_id << " speed.\n";
        }
      },
      "Set a motor speed: set_speed <motor_id> <rpm>");
  root_menu->Insert(
      "platform_speed",
      [&motors, &platform, &kMotorIdsByWheel](std::ostream &out, float x_mps, float y_mps,
                           float w_rpm) {
        const auto wheel_speeds = platform.calculate_wheel_speeds(x_mps, y_mps, w_rpm);
        const std::array<float, 4> motor_rpms = {
            wheel_speeds.top_left_rpm,
            wheel_speeds.top_right_rpm,
            wheel_speeds.bottom_left_rpm,
            wheel_speeds.bottom_right_rpm,
        };
        float maximum_rpm = 0.0f;
        for (float rpm : motor_rpms) {
          maximum_rpm = std::max(maximum_rpm, std::abs(rpm));
        }
        const float speed_scale = maximum_rpm > 30.0f ? 30.0f / maximum_rpm : 1.0f;
        bool success = true;
        for (size_t wheel_index = 0; wheel_index < motor_rpms.size(); ++wheel_index) {
          const size_t motor_index = kMotorIdsByWheel[wheel_index] - 1;
          success = motors[motor_index].send_velocity(motor_rpms[wheel_index] * speed_scale) &&
                    success;
        }
        out << "Motor RPM: top_left=" << motor_rpms[0] * speed_scale
            << " top_right=" << motor_rpms[1] * speed_scale
            << " bottom_left=" << motor_rpms[2] * speed_scale
            << " bottom_right=" << motor_rpms[3] * speed_scale << "\n";
        if (speed_scale < 1.0f) {
          out << "Motor speeds scaled to the 30 RPM limit.\n";
        }
        if (!success) {
          out << "Failed to apply one or more motor speeds.\n";
        }
      },
      "Set platform motion: platform_speed <x_mps> <y_mps> <w_rpm>");
  cli::Cli cli(std::move(root_menu));
  std::thread([&cli] {
    espp::Cli input(cli);
    input.Start();
  }).detach();

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
