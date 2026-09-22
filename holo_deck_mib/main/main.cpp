// Holonomic-drive MIB on the Waveshare ESP32-P4-ETH kit.
//
// The joystick HMI (pace-hmi-fw) publishes its stick (XYTwist) and its drive
// requests (DriveCommand) over Ethernet with the shared RTPS messages
// (external/rammp-rtps); this firmware drives the four RMD-X6-S2 wheel motors
// over CAN from them and publishes the chair's state back (MibStatus,
// Diagnostics). See hw_config.hpp for the pins, limits and timing, and
// README.md for the network setup.

#include <array>
#include <chrono>
#include <memory>
#include <thread>

#include "cli.hpp"
#include "esp32-p4-eth.hpp"
#include "logger.hpp"

#include "holo_deck_controller.hpp"
#include "holo_deck_platform.hpp"
#include "hw_config.hpp"
#include "motor_actuator.hpp"
#include "rtps_link.hpp"
#include "twai_motor_bus.hpp"

using namespace std::chrono_literals;

namespace {
const char *source_name(HoloDeckController::Source source) {
  switch (source) {
  case HoloDeckController::Source::GUI:
    return "CLI";
  case HoloDeckController::Source::JOYSTICK:
    return "JOYSTICK";
  default:
    return "STOPPED";
  }
}

const char *mode_name(HoloDeckController::Mode mode) {
  switch (mode) {
  case HoloDeckController::Mode::DRIVE:
    return "DRIVE";
  case HoloDeckController::Mode::STOPPED:
    return "STOPPED";
  default:
    return "DISABLED";
  }
}

std::string ip_to_string(const esp_ip4_addr_t &ip) {
  return fmt::format("{}.{}.{}.{}", esp_ip4_addr1_16(&ip), esp_ip4_addr2_16(&ip),
                     esp_ip4_addr3_16(&ip), esp_ip4_addr4_16(&ip));
}
} // namespace

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "holo_deck_mib", .level = espp::Logger::Verbosity::INFO});
  logger.info("Bootup: holonomic-drive MIB (ESP32-P4-ETH)");

  // --- Ethernet ---------------------------------------------------------------
  auto &board = espp::Esp32P4Eth::get();
  espp::Esp32P4Eth::EthernetConfig eth_config{
      .on_link_up = [&]() { logger.info("Ethernet link up"); },
      .on_link_down = [&]() { logger.warn("Ethernet link down"); },
  };
#if CONFIG_HOLO_DECK_MIB_DHCP_SERVER
  // the HMI is a DHCP client: on a direct cable (no router) this board hands
  // it an address
  eth_config.mode = espp::Esp32P4Eth::DhcpMode::SERVER;
  eth_config.server_config.on_client_assigned = [&](esp_ip4_addr_t ip, std::array<uint8_t, 6>) {
    logger.info("DHCP lease handed out: {}", ip_to_string(ip));
  };
#else
  eth_config.mode = espp::Esp32P4Eth::DhcpMode::CLIENT;
#endif
  if (!board.initialize_ethernet(eth_config)) {
    logger.error("Ethernet initialization failed");
    return;
  }

  // --- CAN bus + motors -------------------------------------------------------
  TwaiMotorBus can_bus({
      .tx_gpio = hw_config::kCanTxGpio,
      .rx_gpio = hw_config::kCanRxGpio,
      .bitrate = hw_config::kCanBitrateBps,
      .log_level = espp::Logger::Verbosity::INFO,
  });
  std::error_code can_ec;
  const bool can_ok = can_bus.start(can_ec);
  if (!can_ok)
    logger.error("CAN bus failed to start: {} (driving is unavailable)", can_ec.message());

  const auto communicate = can_bus.communication_function();
  std::array<MotorActuator, 4> motors = {
      MotorActuator(communicate, 1),
      MotorActuator(communicate, 2),
      MotorActuator(communicate, 3),
      MotorActuator(communicate, 4),
  };

  // wheel -> motor id map and platform geometry, unchanged from the Tab5 firmware
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

  // --- Controller: owns the command state and the 50 Hz control loop ----------
  // It boots e-stopped; the HMI's DriveCommand (or the CLI) enables it.
  HoloDeckController controller({
      .platform = platform,
      .motors = motors,
      .motor_ids_by_wheel = kMotorIdsByWheel,
      .max_wheel_rpm = hw_config::kMaxWheelRpm,
      .max_speed_mps = hw_config::kProfileNormal.max_speed_mps,
      .max_rotation_rpm = hw_config::kProfileNormal.max_rotation_rpm,
      .twist_rotation_scale = hw_config::kTwistRotationScale,
      .control_period = hw_config::kControlPeriod,
      .status_poll_period = hw_config::kStatusPollPeriod,
      .joystick_release_timeout = hw_config::kJoystickReleaseTimeout,
      .status_stale_timeout = hw_config::kMotorStatusStaleTimeout,
      .status_read_timeout_ms = hw_config::kMotorStatusReadTimeoutMs,
      .log_level = espp::Logger::Verbosity::INFO,
  });

  // --- Wait for an IP, then bring up the RTPS link -----------------------------
  logger.info("Waiting for the Ethernet link + IP...");
  while (!board.is_ethernet_connected()) {
    std::this_thread::sleep_for(100ms);
  }
  const std::string interface_address = ip_to_string(board.ethernet_ip());
  logger.info("Ethernet up: {}", interface_address);

  RtpsLink link({
      .interface_address = interface_address,
      .controller = controller,
      .publish_period = hw_config::kStatusPublishPeriod,
      .joystick_hold_timeout = hw_config::kJoystickHoldTimeout,
      .joystick_lost_timeout = hw_config::kJoystickLostTimeout,
      .log_level = espp::Logger::Verbosity::INFO,
  });
  if (!link.start()) {
    logger.error("RTPS link failed to start");
    return;
  }
  if (can_ok) {
    link.set_ready(true);
  } else {
    link.set_error("CAN bus unavailable", "Check the transceiver wiring and restart");
  }

  // --- CLI (UART console): bench control without the HMI ------------------------
  auto root_menu = std::make_unique<cli::Menu>("holo_deck_mib");
  root_menu->Insert(
      "velocity",
      [&controller](std::ostream &out, float x_mps, float y_mps, float w_rpm) {
        controller.set_gui_velocity(x_mps, y_mps, w_rpm);
        const auto state = controller.state();
        out << "Setpoint: vx=" << state.gui_vx_mps << " m/s vy=" << state.gui_vy_mps
            << " m/s w=" << state.gui_w_rpm << " RPM (clamped to limits).\n";
        if (!state.enabled)
          out << "Controller is not in DRIVE; `enable` to apply.\n";
        else if (state.source == HoloDeckController::Source::JOYSTICK)
          out << "The HMI joystick is active; the setpoint applies when it is released.\n";
      },
      "Set the local velocity setpoint: velocity <x_mps> <y_mps> <w_rpm>");
  root_menu->Insert(
      "enable",
      [&controller](std::ostream &out) {
        controller.enable();
        out << "DRIVE (setpoint zeroed).\n";
      },
      "Enable driving (clears e-stop; setpoint starts at zero)");
  root_menu->Insert(
      "estop",
      [&controller](std::ostream &out) {
        controller.stop();
        out << "E-STOP: commanding zero velocity (motors hold at 0).\n";
      },
      "E-stop: command zero velocity to all motors");
  root_menu->Insert(
      "disable",
      [&controller](std::ostream &out) {
        controller.disable_motors();
        out << "Motors DISABLED at the control level. `enable` to drive again.\n";
      },
      "Disable the motors at the control level");
  root_menu->Insert(
      "limits",
      [&controller](std::ostream &out, float max_speed_mps, float max_rotation_rpm) {
        controller.set_max_speed(max_speed_mps);
        controller.set_max_rotation(max_rotation_rpm);
        const auto state = controller.state();
        out << "Limits: " << state.max_speed_mps << " m/s, " << state.max_rotation_rpm
            << " RPM (the next DriveCommand resets them to its profile).\n";
      },
      "Set the max translation speed / rotation rate: limits <max_speed_mps> <max_rot_rpm>");
  root_menu->Insert(
      "status",
      [&controller, &link, &can_bus, &board](std::ostream &out) {
        const auto state = controller.state();
        out << "ethernet: "
            << (board.is_ethernet_connected() ? ip_to_string(board.ethernet_ip())
                                              : std::string("down"))
            << "\nhmi:      "
            << (link.hmi_alive()      ? "alive"
                : link.peer_matched() ? "matched, silent"
                                      : "not found");
        if (const auto age = link.joystick_age())
          out << ", last XYTwist " << age->count() << " ms ago";
        out << "\nmode:     " << mode_name(state.mode)
            << "\nsource:   " << source_name(state.source) << "\ncommand:  vx=" << state.vx_mps
            << " m/s vy=" << state.vy_mps << " m/s w=" << state.w_rpm
            << " RPM\nlimits:   " << state.max_speed_mps << " m/s, " << state.max_rotation_rpm
            << " RPM, wheel " << state.max_wheel_rpm << " RPM\ncan:      tx errors "
            << can_bus.tx_error_count() << ", unexpected rx " << can_bus.unexpected_rx_count()
            << "\n";
        for (const auto &motor : state.motors) {
          out << "motor " << static_cast<int>(motor.id) << ": cmd=" << motor.commanded_rpm
              << " RPM";
          if (motor.valid)
            out << " meas=" << motor.velocity_rpm << " RPM temp=" << motor.temperature_c
                << " C angle=" << motor.angle_degrees << " deg "
                << (motor.stale ? "[STALE]" : "[OK]");
          else
            out << " [NO DATA]";
          out << "\n";
        }
      },
      "Print the link, controller and motor state");
  cli::Cli cli(std::move(root_menu));
  std::thread([&cli] {
    espp::Cli input(cli);
    input.Start();
  }).detach();

  logger.info("Ready ({}); waiting for the HMI's DriveCommand ENABLE", can_ok ? "IDLE" : "ERROR");

  // Everything runs in the controller / link timers; the main task only reports
  // link changes.
  bool last_matched = false;
  while (true) {
    const bool alive = link.hmi_alive();
    if (alive != last_matched) {
      logger.info("HMI {}", alive ? "connected" : "gone (no messages)");
      last_matched = alive;
    }
    std::this_thread::sleep_for(1s);
  }
}
