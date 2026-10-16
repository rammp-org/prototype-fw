// PEAR prototype base MIB on the Waveshare ESP32-P4-ETH kit.
//
// The joystick HMI (pace-hmi-fw) publishes its stick (XYTwist), its drive
// requests (DriveCommand) and its seat / debug-actuator requests (SeatCommand)
// over Ethernet with the shared RTPS messages (external/rammp-rtps); this
// firmware drives the four hub-wheel drives and the four corner-linkage motors
// over ONE CAN bus from them and publishes the base's state back (MibStatus,
// Diagnostics, one MotorState per motor). See hw_config.hpp for the pins,
// ids, geometry, limits and timing, and README.md for the network setup.

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>

#include "cli.hpp"
#include "esp32-p4-eth.hpp"
#include "logger.hpp"
#include "nvs.hpp"
#include "task.hpp"

#include "can_bus.hpp"
#include "drive_controller.hpp"
#include "ds402_velocity_motor.hpp"
#include "eyou_motor.hpp"
#include "holo_deck_platform.hpp"
#include "hw_config.hpp"
#include "linkage_controller.hpp"
#include "rtps_link.hpp"

using namespace std::chrono_literals;

namespace {
const char *source_name(DriveController::Source source) {
  switch (source) {
  case DriveController::Source::GUI:
    return "CLI";
  case DriveController::Source::JOYSTICK:
    return "JOYSTICK";
  default:
    return "STOPPED";
  }
}

const char *mode_name(DriveController::Mode mode) {
  switch (mode) {
  case DriveController::Mode::DRIVE:
    return "DRIVE";
  case DriveController::Mode::STOPPED:
    return "STOPPED";
  default:
    return "DISABLED";
  }
}

std::string ip_to_string(const esp_ip4_addr_t &ip) {
  return fmt::format("{}.{}.{}.{}", esp_ip4_addr1_16(&ip), esp_ip4_addr2_16(&ip),
                     esp_ip4_addr3_16(&ip), esp_ip4_addr4_16(&ip));
}

/// "fr" / "fl" / "br" / "bl" -> the corner; "all" -> nullopt; anything else -> false.
bool parse_corner(std::string_view text, std::optional<pear::Corner> &corner) {
  if (text == "all") {
    corner = std::nullopt;
    return true;
  }
  for (const pear::Corner c : pear::kCorners) {
    std::string name = pear::corner_name(c);
    std::transform(name.begin(), name.end(), name.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    if (text == name) {
      corner = c;
      return true;
    }
  }
  return false;
}

/// The seat-axis table range (whole units) of a PEAR_LINK_* row.
pear::geometry::Corners table_limits(bool maximum) {
  pear::geometry::Corners limits{};
  for (const pear::Corner corner : pear::kCorners) {
    const auto &row =
        rammp::kSeatAxes[static_cast<size_t>(rammp::SeatAxis::PEAR_LINK_FR) + pear::index(corner)];
    float scale = 1.0f;
    for (uint8_t i = 0; i < row.decimals; ++i)
      scale *= 10.0f;
    limits[corner] = static_cast<float>(maximum ? row.max_value : row.min_value) / scale;
  }
  return limits;
}

void print_drive_status(std::ostream &out, const DriveController::State &state) {
  out << "drive:    " << mode_name(state.mode) << (state.armed ? " (drives armed)" : "")
      << (state.send_failing ? " [SEND FAILING]" : "")
      << "\nsource:   " << source_name(state.source) << "\ncommand:  vx=" << state.vx_mps
      << " m/s vy=" << state.vy_mps << " m/s w=" << state.w_rpm
      << " RPM\nlimits:   " << state.max_speed_mps << " m/s, " << state.max_rotation_rpm
      << " RPM, wheel " << state.max_wheel_rpm << " RPM\n";
  for (const pear::Corner corner : pear::kCorners) {
    const auto &motor = state.motors[pear::index(corner)];
    out << "wheel " << pear::corner_name(corner) << " (node " << static_cast<int>(motor.id)
        << "): cmd=" << motor.commanded_rpm << " RPM";
    if (motor.valid)
      out << " meas=" << motor.velocity_rpm << " RPM temp=" << motor.temperature_c
          << " C cur=" << motor.current_a << " A pos=" << motor.angle_degrees << " deg"
          << (motor.faulted ? " [FAULT]" : "") << (motor.stale ? " [STALE]" : " [OK]");
    else
      out << " [NO DATA]";
    out << "\n";
  }
}

void print_linkage_status(std::ostream &out, const LinkageController::State &state) {
  out << "linkage:  " << (state.initialized ? "initialized" : "NOT initialized")
      << (state.enabled ? ", ready" : ", not ready") << (state.any_faulted ? " [FAULT]" : "")
      << "\npose:     elev=" << state.pose.elevation_m * 1000.0f
      << " mm fb=" << state.pose.fb_tilt_deg << " deg lat=" << state.pose.lateral_tilt_deg
      << " deg\n";
  for (const pear::Corner corner : pear::kCorners) {
    const auto &motor = state.motors[pear::index(corner)];
    out << "link " << pear::corner_name(corner) << " (node " << static_cast<int>(motor.id) << "): ";
    if (motor.valid)
      out << "pos=" << motor.position_degrees << " deg target=" << motor.target_degrees
          << " deg range=[" << state.min_deg[corner] << ", " << state.max_deg[corner] << "] "
          << EyouMotor::ds402_state_name(motor.statusword)
          << (motor.zero_loaded ? "" : " [NO ZERO]") << (motor.target_pending ? " [MOVING]" : "")
          << (motor.faulted ? " [FAULT]" : "")
          << (motor.out_of_range ? " [OUT OF RANGE: jog only]" : "")
          << (motor.stale ? " [STALE]" : " [OK]");
    else
      out << "[NO DATA]" << (motor.zero_loaded ? "" : " [NO ZERO]");
    out << "\n";
  }
}
} // namespace

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "pear", .level = espp::Logger::Verbosity::INFO});
  logger.info("Bootup: PEAR prototype base MIB (ESP32-P4-ETH)");

  // --- NVS (the linkage motors keep their software zero there) ---------------
  {
    std::error_code ec;
    espp::Nvs nvs;
    nvs.init(ec);
    if (ec)
      logger.error("NVS init failed: {} (linkage zeros cannot be loaded / saved)", ec.message());
  }

  // --- Ethernet ---------------------------------------------------------------
  auto &board = espp::Esp32P4Eth::get();
  espp::Esp32P4Eth::EthernetConfig eth_config{
      .on_link_up = [&]() { logger.info("Ethernet link up"); },
      .on_link_down = [&]() { logger.warn("Ethernet link down"); },
  };
#if CONFIG_PEAR_DHCP_SERVER
  // the HMI is a DHCP client: on a direct cable (no router) this board hands
  // it an address
  eth_config.mode = espp::Esp32P4Eth::DhcpMode::SERVER;
  eth_config.server_config.on_client_assigned = [&](esp_ip4_addr_t ip, std::array<uint8_t, 6>) {
    logger.info("DHCP lease handed out: {}", ip_to_string(ip));
  };
#else
  eth_config.mode = espp::Esp32P4Eth::DhcpMode::CLIENT;
#endif
  const bool eth_ok = board.initialize_ethernet(eth_config);
  if (!eth_ok)
    logger.error("Ethernet initialization failed (the HMI link is unavailable; CLI still runs)");

  // --- CAN bus + the eight CANopen nodes --------------------------------------
  // CanBus(rx_gpio, tx_gpio): the constants are named by TWAI function, see
  // hw_config.hpp for the mapping to verify on the kit.
  CanBus can_bus(hw_config::kCanRxGpio, hw_config::kCanTxGpio);
  // Every motor registers its CAN receivers in its constructor, which must
  // happen before CanBus::start() (registration is locked once the bus runs).
  std::array<EyouMotor, pear::kCornerCount> linkage_motors{{
      EyouMotor(can_bus, hw_config::kLinkageNodeIds[0]),
      EyouMotor(can_bus, hw_config::kLinkageNodeIds[1]),
      EyouMotor(can_bus, hw_config::kLinkageNodeIds[2]),
      EyouMotor(can_bus, hw_config::kLinkageNodeIds[3]),
  }};
  std::array<Ds402VelocityMotor, pear::kCornerCount> wheel_motors{{
      Ds402VelocityMotor(can_bus, hw_config::kWheelNodeIds[0], hw_config::kWheelDriveConfig),
      Ds402VelocityMotor(can_bus, hw_config::kWheelNodeIds[1], hw_config::kWheelDriveConfig),
      Ds402VelocityMotor(can_bus, hw_config::kWheelNodeIds[2], hw_config::kWheelDriveConfig),
      Ds402VelocityMotor(can_bus, hw_config::kWheelNodeIds[3], hw_config::kWheelDriveConfig),
  }};
  const bool can_ok = can_bus.start(hw_config::kCanBitrateBps);
  if (!can_ok)
    logger.error("CAN bus failed to start (driving and the linkages are unavailable)");

  // --- Bring-up: wheel drives (velocity mode, zero set-point, enabled) ---------
  bool wheels_armed = false;
  if (can_ok) {
    wheels_armed = true;
    for (auto &motor : wheel_motors) {
      if (!motor.enable_drive()) {
        logger.error("Wheel drive {} failed to enable (retried from the control loop on enable)",
                     motor.node_id());
        wheels_armed = false;
      }
    }
    for (auto &motor : wheel_motors) {
      Ds402VelocityMotor::Status status{};
      if (motor.read_status(status, 50))
        logger.info("Wheel drive {}: {} (statusword 0x{:04X}, error 0x{:04X}, {:.1f} C)",
                    motor.node_id(), Ds402VelocityMotor::state_name(status.statusword),
                    status.statusword, status.error_code, status.temperature_c);
      else
        logger.warn("Wheel drive {}: status read failed", motor.node_id());
    }
  }

  // --- Controllers ------------------------------------------------------------
  HoloDeckPlatform platform(hw_config::kPlatformConfiguration);

  // The drive controller boots e-stopped; the HMI's DriveCommand (or the CLI)
  // enables it.
  DriveController drive({
      .platform = platform,
      .motors = wheel_motors,
      .motor_ids_by_wheel = hw_config::kWheelNodeIdsByPlatformWheel,
      .motors_armed = wheels_armed,
      .max_wheel_rpm = hw_config::kMaxWheelRpm,
      .max_speed_mps = hw_config::kProfileNormal.max_speed_mps,
      .max_rotation_rpm = hw_config::kProfileNormal.max_rotation_rpm,
      .twist_rotation_scale = hw_config::kTwistRotationScale,
      .control_period = hw_config::kControlPeriod,
      .status_poll_period = hw_config::kStatusPollPeriod,
      .joystick_release_timeout = hw_config::kJoystickReleaseTimeout,
      .status_stale_timeout = hw_config::kMotorStatusStaleTimeout,
      .rearm_retry_period = hw_config::kWheelRearmRetryPeriod,
      .status_read_timeout_ms = hw_config::kMotorStatusReadTimeoutMs,
      // the CanBus queues frames: drain the queued set-points before a stop's
      // zeros so nothing older executes after them
      .flush_pending_commands = [&can_bus]() { can_bus.flush_pending(); },
      .log_level = espp::Logger::Verbosity::INFO,
  });

  LinkageController linkage({
      .motors = linkage_motors,
      .geometry = hw_config::kLinkageGeometry,
      .profile = hw_config::kLinkageProfile,
      .direction = hw_config::kLinkageDirection,
      .jog_max_deg = hw_config::kLinkageJogMaxDeg,
      .range_tolerance_deg = hw_config::kLinkageRangeToleranceDeg,
      .table_min_deg = table_limits(false),
      .table_max_deg = table_limits(true),
      .drive_enabled = [&drive]() { return drive.is_enabled(); },
      .status_poll_period = hw_config::kLinkageStatusPollPeriod,
      .status_stale_timeout = hw_config::kLinkageStatusStaleTimeout,
      .status_read_timeout_ms = hw_config::kLinkageStatusReadTimeoutMs,
      .log_level = espp::Logger::Verbosity::INFO,
  });
  drive.set_linkage_moving([&linkage]() { return linkage.moving(); });
  bool linkage_ok = false;
  if (can_ok) {
    // configure_profile_position -> enable_drive -> load_zero -> limits, per motor
    linkage_ok = linkage.initialize();
    if (!linkage_ok)
      logger.error("One or more linkage motors failed to initialize (see above)");
  }

  // --- RTPS link, brought up from a task once the interface has an IP ---------
  RtpsLink link({
      .drive = drive,
      .linkage = linkage,
      .publish_period = hw_config::kStatusPublishPeriod,
      .motor_state_period = hw_config::kMotorStatePublishPeriod,
      .joystick_hold_timeout = hw_config::kJoystickHoldTimeout,
      .joystick_lost_timeout = hw_config::kJoystickLostTimeout,
      .log_level = espp::Logger::Verbosity::INFO,
  });
  std::mutex link_mutex;
  bool link_started = false;
  espp::Task rtps_task(
      {.callback = [&](std::mutex &m, std::condition_variable &cv) -> bool {
         if (!eth_ok)
           return true; // nothing to wait for
         if (!board.is_ethernet_connected()) {
           std::unique_lock<std::mutex> lock(m);
           cv.wait_for(lock, 100ms);
           return false;
         }
         const std::string interface_address = ip_to_string(board.ethernet_ip());
         logger.info("Ethernet up: {}; starting the RTPS link", interface_address);
         if (!link.start(interface_address)) {
           logger.error("RTPS link failed to start");
           return true;
         }
         if (can_ok && linkage_ok) {
           link.set_ready(true);
         } else if (!can_ok) {
           link.set_error("CAN bus unavailable", "Check the transceiver wiring and restart");
         } else {
           // driving is still possible; the linkages are refused until fixed
           link.set_ready(true);
           link.set_error("Linkage motor init failed", "See the console; `status` for details");
         }
         std::lock_guard<std::mutex> lock(link_mutex);
         link_started = true;
         return true; // done
       },
       .task_config = {.name = "pear_rtps", .stack_size_bytes = 8192}});
  rtps_task.start();

  // --- CLI (UART console): bench control without the HMI ------------------------
  auto root_menu = std::make_unique<cli::Menu>("pear");
  root_menu->Insert(
      "enable",
      [&drive](std::ostream &out) {
        if (!drive.enable()) {
          out << "ENABLE refused: a linkage move is executing; try again\n";
          return;
        }
        out << "DRIVE (setpoint zeroed; linkage moves are refused while driving).\n";
      },
      "Enable driving (clears e-stop; setpoint starts at zero)");
  root_menu->Insert(
      "stop",
      [&drive](std::ostream &out) {
        drive.stop();
        out << "E-STOP: streaming zero velocity (drives hold at 0).\n";
      },
      "E-stop: command zero velocity to all wheel drives");
  root_menu->Insert(
      "disable",
      [&drive](std::ostream &out) {
        drive.disable_motors();
        out << "Wheel drives DISABLED (power stage off). `enable` to drive again.\n";
      },
      "Disable the wheel drives at the DS402 level");
  root_menu->Insert(
      "speed",
      [&drive](std::ostream &out, float vx_mps, float vy_mps, float w_rpm) {
        drive.set_gui_velocity(vx_mps, vy_mps, w_rpm);
        const auto state = drive.state();
        out << "Setpoint: vx=" << state.gui_vx_mps << " m/s vy=" << state.gui_vy_mps
            << " m/s (+ = right) w=" << state.gui_w_rpm << " RPM (clamped to the profile's limits; "
            << "the kinematics see w x " << state.twist_rotation_scale
            << ", hw_config::kTwistRotationScale).\n";
        if (!state.enabled)
          out << "Not in DRIVE; `enable` to apply.\n";
        else if (state.source == DriveController::Source::JOYSTICK)
          out << "The HMI joystick is active; the setpoint applies when it is released.\n";
      },
      "Set the local velocity setpoint: speed <vx_mps> <vy_mps (+ = right)> <w_rpm (+ = ccw)>");
  root_menu->Insert(
      "profile",
      [&drive](std::ostream &out, const std::string &name) {
        const hw_config::ProfileLimits *limits = nullptr;
        if (name == "low")
          limits = &hw_config::kProfileLow;
        else if (name == "normal")
          limits = &hw_config::kProfileNormal;
        else if (name == "high")
          limits = &hw_config::kProfileHigh;
        if (limits == nullptr) {
          out << "Unknown profile '" << name << "': low | normal | high\n";
          return;
        }
        drive.set_max_speed(limits->max_speed_mps);
        drive.set_max_rotation(limits->max_rotation_rpm);
        out << "Limits: " << limits->max_speed_mps << " m/s, " << limits->max_rotation_rpm
            << " RPM (the next DriveCommand resets them to its profile).\n";
      },
      "Select the drive profile's limits: profile <low|normal|high>");
  root_menu->Insert(
      "link",
      [&linkage](std::ostream &out, const std::string &which, float angle_deg) {
        std::optional<pear::Corner> corner;
        if (!parse_corner(which, corner)) {
          out << "Unknown corner '" << which << "': fr | fl | br | bl | all\n";
          return;
        }
        bool ok = true;
        for (const pear::Corner c : pear::kCorners) {
          if (corner.has_value() && *corner != c)
            continue;
          ok = linkage.request_corner_angle(c, angle_deg) && ok;
        }
        out << (ok ? "Move queued"
                   : "REFUSED (see the log; a corner outside its range only "
                     "takes `jog`)")
            << ": " << which << " -> " << angle_deg << " deg (clamped to the corner's range";
        if (corner.has_value())
          out << " [" << linkage.min_angle_deg(*corner) << ", " << linkage.max_angle_deg(*corner)
              << "]";
        out << ").\n";
      },
      "Move a corner linkage to an absolute crank angle: link <fr|fl|br|bl|all> <deg>");
  root_menu->Insert(
      "jog",
      [&linkage](std::ostream &out, const std::string &which, float delta_deg) {
        std::optional<pear::Corner> corner;
        if (!parse_corner(which, corner) || !corner.has_value()) {
          out << "One corner: jog <fr|fl|br|bl> <deg> (+/-" << hw_config::kLinkageJogMaxDeg
              << " deg max)\n";
          return;
        }
        out << (linkage.request_jog(*corner, delta_deg) ? "Jog queued" : "REFUSED (see the log)")
            << ": " << which << " " << (delta_deg >= 0 ? "+" : "") << delta_deg
            << " deg from its measured angle (not clamped to the range; the motor's own limit "
               "still applies).\n";
      },
      "Jog one corner relative to its measured angle, the only move a corner outside its range "
      "takes: jog <fr|fl|br|bl> <deg> (+/-1 max)");
  root_menu->Insert(
      "pose",
      [&linkage](std::ostream &out, float elevation_mm, float fb_deg, float lat_deg) {
        const bool ok = linkage.request_pose({.elevation_m = elevation_mm * 0.001f,
                                              .fb_tilt_deg = fb_deg,
                                              .lateral_tilt_deg = lat_deg});
        out << (ok ? "Pose queued" : "Refused (see the log)") << ": elev " << elevation_mm
            << " mm, fb " << fb_deg << " deg (front up +), lat " << lat_deg
            << " deg (left up +); corners are clamped individually.\n";
      },
      "Move the four linkages to a base pose: pose <elev_mm> <fb_deg> <lat_deg>");
  root_menu->Insert(
      "zero",
      [&linkage](std::ostream &out, const std::string &which) {
        std::optional<pear::Corner> corner;
        if (!parse_corner(which, corner)) {
          out << "Unknown corner '" << which << "': fr | fl | br | bl | all\n";
          return;
        }
        out << (linkage.zero(corner) ? "Software zero set to the current position"
                                     : "Zero failed for one or more motors")
            << " (" << which << "); `save_zero` to persist.\n";
      },
      "Set a linkage's software zero to its current position: zero <fr|fl|br|bl|all>");
  root_menu->Insert(
      "save_zero",
      [&linkage](std::ostream &out) {
        out << (linkage.save_zero() ? "All linkage zeros saved to NVS.\n"
                                    : "Saving failed for one or more motors (zeroed?).\n");
      },
      "Persist every linkage's software zero to NVS");
  root_menu->Insert(
      "load_zero",
      [&linkage](std::ostream &out) {
        out << (linkage.load_zero() ? "All linkage zeros loaded from NVS.\n"
                                    : "No saved zero for one or more motors.\n");
      },
      "Load every linkage's software zero from NVS");
  root_menu->Insert(
      "reset_fault",
      [&wheel_motors, &linkage_motors](std::ostream &out, const std::string &which) {
        std::optional<pear::Corner> corner;
        if (!parse_corner(which, corner)) {
          out << "Unknown corner '" << which << "': fr | fl | br | bl | all\n";
          return;
        }
        for (const pear::Corner c : pear::kCorners) {
          if (corner.has_value() && *corner != c)
            continue;
          out << pear::corner_name(c) << ": wheel "
              << (wheel_motors[pear::index(c)].reset_fault() ? "ok" : "FAILED") << ", linkage "
              << (linkage_motors[pear::index(c)].reset_fault() ? "ok" : "FAILED") << "\n";
        }
      },
      "Clear the DS402 fault of a corner's wheel + linkage: reset_fault <fr|fl|br|bl|all>");
  root_menu->Insert(
      "status",
      [&drive, &linkage, &link, &board, &link_mutex, &link_started](std::ostream &out) {
        bool started = false;
        {
          std::lock_guard<std::mutex> lock(link_mutex);
          started = link_started;
        }
        out << "ethernet: "
            << (board.is_ethernet_connected() ? ip_to_string(board.ethernet_ip())
                                              : std::string("down"))
            << "\nrtps:     " << (started ? "up" : "not started") << "\nhmi:      ";
        if (started) {
          out << (link.hmi_alive()      ? "alive"
                  : link.peer_matched() ? "matched, silent"
                                        : "not found");
          if (const auto age = link.joystick_age())
            out << ", last XYTwist " << age->count() << " ms ago";
        } else {
          out << "-";
        }
        out << "\n";
        print_drive_status(out, drive.state());
        print_linkage_status(out, linkage.state());
      },
      "Print the link, drive, wheel and linkage state");
  root_menu->Insert(
      "can",
      [&can_bus](std::ostream &out) {
        const auto s = can_bus.get_status();
        out << "CAN controller: " << s.state << "\n  TEC " << s.tx_error_count << ", REC "
            << s.rx_error_count << ", bus errors since start " << s.bus_error_count
            << ", TX queue free " << s.tx_queue_free << "\n  by type: stuff " << s.stuff_errors
            << ", form " << s.form_errors << ", bit " << s.bit_errors << ", ack " << s.ack_errors
            << ", arbitration lost " << s.arbitration_lost << " (normal, retransmitted)\n  bus-off "
            << s.bus_off_count << " time(s) since start (recovered automatically)\n";
        if (s.bus_off) {
          out << "  the controller left the bus: fix the wiring / bitrate, then `can_recover`\n";
        } else if (s.stuff_errors + s.form_errors + s.bit_errors > 0) {
          out << "  corrupted frames: signal integrity - 120 ohm termination at BOTH ends only "
                 "(about 60 ohm across H/L, bus unpowered), every node's isolated CAN ground "
                 "(L2DB CN5 pin 3 isoGND) tied to the bus ground, short stubs, or a node at "
                 "another bitrate (L2DB SW2-2 ON = 1 Mbit/s, read at power-up)\n";
        } else if (s.ack_errors > 0) {
          out << "  frames we sent were not acknowledged by anyone: bus open / all nodes off at "
                 "that moment\n";
        } else {
          out << "  no bus errors: a node that does not answer is off, unpowered, at another id "
                 "or not on this bus\n";
        }
      },
      "Print the CAN controller state and error counters (bitrate / wiring check)");
  root_menu->Insert(
      "can_recover",
      [&can_bus](std::ostream &out) {
        out << (can_bus.recover() ? "bus-off recovery started; `can` shows when it is back\n"
                                  : "recovery not started (see the log)\n");
      },
      "Start the CAN bus-off recovery");
  root_menu->Insert(
      "diag",
      [&drive, &linkage](std::ostream &out) {
        // the diagnostics rows as the HMI shows them
        const auto d = drive.state();
        const auto l = linkage.state();
        for (const pear::Corner corner : pear::kCorners) {
          const auto &w = d.motors[pear::index(corner)];
          const auto &m = l.motors[pear::index(corner)];
          out << "W" << pear::index(corner) + 1 << " wheel " << pear::corner_name(corner)
              << ": temp " << w.temperature_c << " C, current " << w.current_a << " A, vel "
              << w.velocity_rpm << " rpm, sw 0x" << std::hex << w.statusword << " err 0x"
              << w.error_code << std::dec << ", sdo " << w.reply_latency_us / 1000.0f << " ms (max "
              << w.max_reply_latency_us / 1000.0f << ")" << (w.valid ? "" : " [NO DATA]")
              << (w.stale ? " [STALE]" : "") << "\n";
          out << "L" << pear::index(corner) + 1 << " link " << pear::corner_name(corner) << ": pos "
              << m.position_degrees << " deg, sw 0x" << std::hex << m.statusword << std::dec << " ("
              << EyouMotor::ds402_state_name(m.statusword) << ")" << (m.valid ? "" : " [NO DATA]")
              << (m.stale ? " [STALE]" : "") << "\n";
        }
        out << "pose: elev " << l.pose.elevation_m * 1000.0f << " mm, fb " << l.pose.fb_tilt_deg
            << " deg, lat " << l.pose.lateral_tilt_deg << " deg\n";
      },
      "Print the diagnostics rows (W1..W4 wheels, L1..L4 linkages) and the pose");
  cli::Cli cli(std::move(root_menu));
  std::thread([&cli] {
    espp::Cli input(cli);
    input.Start();
  }).detach();

  logger.info("Ready ({}); boots e-stopped, waiting for the HMI's DriveCommand ENABLE",
              can_ok ? (linkage_ok ? "IDLE" : "IDLE, linkages unavailable") : "ERROR");

  // Everything runs in the controllers' / link's timers; the main task only
  // reports link changes.
  bool last_alive = false;
  while (true) {
    bool started = false;
    {
      std::lock_guard<std::mutex> lock(link_mutex);
      started = link_started;
    }
    if (started) {
      const bool alive = link.hmi_alive();
      if (alive != last_alive) {
        logger.info("HMI {}", alive ? "connected" : "gone (no messages)");
        last_alive = alive;
      }
    }
    std::this_thread::sleep_for(1s);
  }
}
