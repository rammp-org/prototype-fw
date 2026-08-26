#include <array>
#include <chrono>
#include <memory>
#include <thread>

#include "cli.hpp"
#include "logger.hpp"
#include "m5stack-tab5.hpp"

#include "gui.hpp"
#include "holo_deck_controller.hpp"
#include "holo_deck_platform.hpp"
#include "hw_config.hpp"
#include "joystick_input.hpp"
#include "motor_actuator.hpp"

using namespace std::chrono_literals;

namespace {
const char *source_name(HoloDeckController::Source source) {
  switch (source) {
  case HoloDeckController::Source::GUI:
    return "GUI";
  case HoloDeckController::Source::JOYSTICK:
    return "JOYSTICK";
  default:
    return "STOPPED";
  }
}
} // namespace

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "holo_deck", .level = espp::Logger::Verbosity::INFO});
  logger.info("Bootup");

  // Board (M5Stack Tab5): IO expanders, LCD, LVGL display, touch
  auto &tab5 = espp::M5StackTab5::get();
  if (!tab5.initialize_io_expanders()) {
    logger.error("Failed to initialize IO expanders");
    return;
  }
  if (!tab5.initialize_lcd()) {
    logger.error("Failed to initialize LCD");
    return;
  }
  // full-frame LVGL draw buffer (in PSRAM) so full-screen redraws flush in a
  // single pass
  const size_t pixel_buffer_size = tab5.display_width() * tab5.display_height();
  if (!tab5.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display");
    return;
  }
  if (!tab5.initialize_touch()) {
    logger.error("Failed to initialize touch");
    return;
  }
  tab5.brightness(75.0f);

  // Motors (four Reflex RMD-X6-S2 on the CAN bus, see hw_config.hpp)
  MotorCanBus can_bus(hw_config::kCanRxGpio, hw_config::kCanTxGpio);
  if (!can_bus.start()) {
    logger.error("Failed to start motor CAN bus");
    return;
  }

  MotorActuator::CommunicationFunction communicate =
      [&can_bus](const MotorPacket &command, MotorPacket &response, uint32_t timeout_ms) {
        if (timeout_ms == 0)
          return can_bus.send(command);
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

  // Controller: owns the command state and the 50 Hz control loop. It boots
  // e-stopped; enable via the GUI, the joystick button, or the CLI.
  HoloDeckController controller({
      .platform = platform,
      .motors = motors,
      .motor_ids_by_wheel = kMotorIdsByWheel,
      .max_wheel_rpm = hw_config::kMaxWheelRpm,
      .max_speed_mps = hw_config::kDefaultMaxSpeedMps,
      .max_rotation_rpm = hw_config::kDefaultMaxRotationRpm,
      .control_period = hw_config::kControlPeriod,
      .status_poll_period = hw_config::kStatusPollPeriod,
      .joystick_release_timeout = hw_config::kJoystickReleaseTimeout,
      .status_stale_timeout = hw_config::kMotorStatusStaleTimeout,
      .status_read_timeout_ms = hw_config::kMotorStatusReadTimeoutMs,
      .log_level = espp::Logger::Verbosity::INFO,
  });

  // Physical joystick: axes feed the controller's arbitration; the button
  // toggles the shared enable / e-stop state.
  JoystickInput joystick_input({
      .callback = [&controller](float forward, float left,
                                float ccw) { controller.set_joystick_input(forward, left, ccw); },
      .button_callback =
          [&controller, &logger]() {
            const bool enable = !controller.is_enabled();
            logger.info("Joystick button: {}", enable ? "ENABLE" : "E-STOP");
            controller.set_enabled(enable);
          },
  });

  // GUI: touch controls write to the controller; a 10 Hz refresh below reads
  // the controller state back into the widgets.
  Gui gui({});
  // translation and rotation are commanded independently: the drag-pad
  // leaves the rotation setpoint alone and vice versa
  gui.set_translation_callback([&controller](float forward, float left) {
    controller.set_gui_translation_normalized(forward, left);
  });
  gui.set_rotation_callback(
      [&controller](float ccw) { controller.set_gui_rotation_normalized(ccw); });
  gui.set_enable_callback([&controller](bool enable) { controller.set_enabled(enable); });
  gui.set_disable_callback([&controller]() { controller.disable_motors(); });
  gui.set_max_speed_callback([&controller](float mps) { controller.set_max_speed(mps); });
  gui.set_max_rotation_callback([&controller](float rpm) { controller.set_max_rotation(rpm); });

  // CLI
  auto root_menu = std::make_unique<cli::Menu>("holo_deck");
  root_menu->Insert(
      "set_speed",
      [&motors, &controller](std::ostream &out, int motor_id, float rpm) {
        if (motor_id < 1 || motor_id > static_cast<int>(motors.size())) {
          out << "Motor ID must be between 1 and " << motors.size() << ".\n";
          return;
        }
        if (controller.is_enabled()) {
          out << "Controller is enabled; the control loop will override this within one "
                 "cycle. Use `estop` first for direct motor control.\n";
        }
        if (motors[motor_id - 1].send_velocity(rpm)) {
          out << "Motor " << motor_id << " speed set to " << rpm << " RPM.\n";
        } else {
          out << "Failed to set motor " << motor_id << " speed.\n";
        }
      },
      "Set a single motor speed directly (debug; e-stop first): set_speed <motor_id> <rpm>");
  auto set_velocity = [&controller](std::ostream &out, float x_mps, float y_mps, float w_rpm) {
    controller.set_gui_velocity(x_mps, y_mps, w_rpm);
    const auto state = controller.state();
    out << "Setpoint: vx=" << state.gui_vx_mps << " m/s vy=" << state.gui_vy_mps
        << " m/s w=" << state.gui_w_rpm << " RPM (clamped to limits).\n";
    if (!state.enabled) {
      out << "Controller is E-STOPPED; `enable` to apply.\n";
    } else if (state.source == HoloDeckController::Source::JOYSTICK) {
      out << "Physical joystick is active; the setpoint applies when it is released.\n";
    }
  };
  root_menu->Insert(
      "velocity", set_velocity,
      "Set the platform velocity setpoint (GUI/CLI source): velocity <x_mps> <y_mps> <w_rpm>");
  root_menu->Insert("platform_speed", set_velocity,
                    "Alias of `velocity` (kept for compatibility): platform_speed <x_mps> "
                    "<y_mps> <w_rpm>");
  root_menu->Insert(
      "enable",
      [&controller](std::ostream &out) {
        controller.set_enabled(true);
        out << "Enabled (setpoint zeroed).\n";
      },
      "Enable the platform (clears e-stop; setpoint starts at zero)");
  root_menu->Insert(
      "estop",
      [&controller](std::ostream &out) {
        controller.stop();
        out << "E-STOP: commanding zero velocity (motors hold at 0).\n";
      },
      "E-stop: command zero velocity to all motors (they hold at 0)");
  root_menu->Insert(
      "disable",
      [&controller](std::ostream &out) {
        controller.disable_motors();
        out << "Motors DISABLED at the control level. Use `enable` to drive again.\n";
      },
      "Disable the motors at the control level (not held at zero by the loop)");
  root_menu->Insert(
      "limits",
      [&controller](std::ostream &out, float max_speed_mps, float max_rotation_rpm) {
        controller.set_max_speed(max_speed_mps);
        controller.set_max_rotation(max_rotation_rpm);
        const auto state = controller.state();
        out << "Limits: " << state.max_speed_mps << " m/s, " << state.max_rotation_rpm << " RPM.\n";
      },
      "Set the max translation speed / rotation rate: limits <max_speed_mps> <max_rot_rpm>");
  root_menu->Insert(
      "status",
      [&controller](std::ostream &out) {
        const auto state = controller.state();
        const char *mode_name = state.mode == HoloDeckController::Mode::DRIVE     ? "DRIVE"
                                : state.mode == HoloDeckController::Mode::STOPPED ? "STOPPED"
                                                                                 : "DISABLED";
        out << "mode:    " << mode_name << "\nsource:  " << source_name(state.source)
            << "\ncommand: vx=" << state.vx_mps
            << " m/s vy=" << state.vy_mps << " m/s w=" << state.w_rpm
            << " RPM\nsetpoint (GUI/CLI): vx=" << state.gui_vx_mps << " m/s vy=" << state.gui_vy_mps
            << " m/s w=" << state.gui_w_rpm << " RPM\nlimits:  " << state.max_speed_mps << " m/s, "
            << state.max_rotation_rpm << " RPM, wheel " << state.max_wheel_rpm << " RPM\n";
        for (const auto &motor : state.motors) {
          out << "motor " << static_cast<int>(motor.id) << ": cmd=" << motor.commanded_rpm
              << " RPM";
          if (motor.valid) {
            out << " meas=" << motor.velocity_rpm << " RPM temp=" << motor.temperature_c << " C "
                << (motor.stale ? "[STALE]" : "[OK]");
          } else {
            out << " [NO DATA]";
          }
          out << "\n";
        }
      },
      "Print the controller state");
  cli::Cli cli(std::move(root_menu));
  std::thread([&cli] {
    espp::Cli input(cli);
    input.Start();
  }).detach();

  logger.info("Ready; platform is E-STOPPED until enabled (GUI, joystick button, or CLI)");

  // refresh the GUI from the controller state at 10 Hz
  while (true) {
    gui.update_state(controller.state());
    std::this_thread::sleep_for(100ms);
  }
}
