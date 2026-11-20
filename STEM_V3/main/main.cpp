// STEM_V3 firmware: drives the 5-bar seat linkage (three reversed Eyou
// CANopen/DS402 actuator pairs over CAN) and exposes it over native USB as an
// espp "dispatcher" device. The USB vendor (WebUSB) and CDC (Web Serial) streams each carry
// stream_frame traffic routed by module id:
//
//   module 0 -> OTA        (standard espp ota_stream protocol -> ota_console)
//   module 4 -> core dump  (espp::CoreDumpService               -> coredump_console)
//   module 7 -> STEM       (this project's protocol             -> web/stem_console.html)
//
// plus dispatcher capability discovery (module 0xFF) so the espp Device Hub
// lists all three. The interactive CLI stays on the console (USB-Serial-JTAG /
// UART0), which on the ESP32-P4 has its own PHY and coexists with TinyUSB on
// the USB-OTG HS port.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <atomic>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "can_bus.hpp"
#include "cli.hpp"
#include "coredump.hpp"
#include "coredump_service.hpp"
#include "detail/ota_stream_protocol.hpp"
#include "dispatcher.hpp"
#include "esp32-p4-eth.hpp"
#include "eyou_motor.hpp"
#include "ik_5bar.hpp"
#include "logger.hpp"
#include "messages.hpp"
#include "motor_actuator.hpp"
#include "ota.hpp"
#include "paired_eyou_actuator.hpp"
#include "rtps_participant.hpp"
#include "rtps_pubsub.hpp"
#include "stem_controller.hpp"
#include "stem_module.hpp"
#include "stream_frame.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "task.hpp"
#include "usb_device.hpp"

using namespace std::chrono_literals;

namespace {

// pid.codes VID 0x1209 (espp default) with a PID distinct from the espp examples
// (0x0d32 ota, 0x0d34 haptics, 0x0d36 coredump); the web consoles filter by VID.
constexpr uint16_t kUsbPid = 0x0d3a;
// Named by TWAI function, the way CanBus(rx_gpio, tx_gpio) takes them: RX from
// the transceiver's RXD on GPIO 16, TX to its TXD on GPIO 17 (the wiring every
// project in this repository uses on the P4-ETH). The previous names had them
// the other way round and relied on the call swapping them back.
constexpr gpio_num_t kCanRxGpio = GPIO_NUM_16;
constexpr gpio_num_t kCanTxGpio = GPIO_NUM_17;

// Torque limit (0x6072, per-thousand of the rated torque): written to every
// motor before the drives are enabled, raised later from the console / CLI up
// to the menuconfig ceiling (see Kconfig.projbuild).
constexpr uint16_t kMaxTorquePermille = CONFIG_STEM_MAX_TORQUE_PERMILLE;
constexpr uint16_t kBootTorquePermille =
    std::min<uint16_t>(CONFIG_STEM_BOOT_TORQUE_PERMILLE, kMaxTorquePermille);

// Base/seat swivel: two MotorActuator (RMD-style) motors sharing the same CAN
// bus as the Eyou linkage motors. MotorActuator's own gear_ratio_ (36:1) is
// internal to that actuator; on top of it an external 100:12 gear ratio drives
// the swivel, so the actuator output shaft turns 100 deg for every 12 deg of
// swivel (motor rotates 100 deg -> swivel rotates 12 deg).
constexpr uint8_t kBaseSwivelMotorId = 5;
constexpr uint8_t kSeatSwivelMotorId = 6;
constexpr float kSwivelGearRatio = 100.0f / 12.0f;
constexpr float kSwivelDefaultSpeedRpm = 10.0f;

constexpr const char *kSeatXyTwistTopic = "rammp/joystick/seat/xy_twist";
constexpr const char *kSwivelXyTwistTopic = "rammp/joystick/swivel/xy_twist";
constexpr float kSwivelJoystickDeadband = 0.2f;

// Joystick-driven swivel state; directions are -1 (backward), 0 (stopped), +1 (forward).
struct SwivelJoystick {
  MotorActuator &base;
  MotorActuator &seat;
  int8_t base_direction{0};
  int8_t seat_direction{0};
};

using Transport = stem::StemModule::Transport;
using Frame = espp::stream_frame::Frame;
using send_fn = stem::StemModule::send_fn;
namespace otap = espp::detail::ota_stream;

// A well-behaved host keeps one frame in flight per stream; cap the RX queue so a
// misbehaving one cannot exhaust RAM while the worker blocks in flash operations.
constexpr size_t kMaxQueuedRxBytes = 8 * espp::stream_frame::kMaxFrameSize;

// If telemetry sends keep failing for this long the host stopped draining the
// endpoint (tab closed / frozen): pause the stream instead of flooding a full
// FIFO and burying the next connect's replies.
constexpr auto kTelemetryStallTimeout = 2s;
// CANopen heartbeat from this board (COB 0x700 + id, state 5 = operational).
// Every actuator is a heartbeat consumer of it (0x1016): if the board stops
// talking (crash, reset, cable) for kHeartbeatTimeoutMs the drives raise a
// fault and stop (pose moves stream position set-points; each drive would
// otherwise finish its last one and hold). The EYOU drives ignore it (tested 2026-11-20).
constexpr uint8_t kHeartbeatNodeId = 0x7F;
constexpr auto kHeartbeatPeriod = 50ms;
constexpr uint16_t kHeartbeatTimeoutMs = 300;
std::atomic<bool> heartbeat_enabled{true}; // CLI `heartbeat 0` pauses it (watchdog test)

void print_solution(std::ostream &out, const stem::IkSolution &s) {
  out << "M3=(" << s.m3[0] << ", " << s.m3[1] << ")\n"
      << "J1=(" << s.j1[0] << ", " << s.j1[1] << ")\n"
      << "J2=(" << s.j2[0] << ", " << s.j2[1] << ")\n"
      << "J3=(" << s.j3[0] << ", " << s.j3[1] << ")\n"
      << "theta1_deg=" << s.theta1_deg << "\n"
      << "theta2_deg=" << s.theta2_deg << "\n"
      << "plate_angle_deg=" << s.plate_angle_deg << "\n"
      << "m3_angle_deg=" << s.m3_angle_deg << "\n";
}

// TODO: drive the seat from the joystick.
void handle_seat_xy_twist(espp::Logger &logger, const rammp::XYTwist &sample) {
  logger.info("{}: x={:.3f} y={:.3f} twist={:.3f} buttons=0x{:X}", kSeatXyTwistTopic, sample.x,
              sample.y, sample.twist, static_cast<uint32_t>(sample.buttons));
}

int8_t joystick_direction(float value) {
  if (value > kSwivelJoystickDeadband) {
    return 1;
  }
  if (value < -kSwivelJoystickDeadband) {
    return -1;
  }
  return 0;
}

// Commands only on a direction change: the joystick streams continuously.
void drive_swivel(espp::Logger &logger, const char *name, MotorActuator &motor, float input,
                  int8_t &current_direction) {
  const int8_t direction = joystick_direction(input);
  if (direction == current_direction) {
    return;
  }
  const bool ok =
      direction == 0 ? motor.stop() : motor.send_velocity(direction * kSwivelDefaultSpeedRpm);
  if (!ok) {
    logger.warn("{} swivel command failed", name);
    return; // retried on the next joystick sample
  }
  current_direction = direction;
  logger.info("{} swivel {}", name,
              direction > 0 ? "forward" : (direction < 0 ? "backward" : "stop"));
}

// x drives the base swivel, y drives the seat swivel.
void handle_swivel_xy_twist(espp::Logger &logger, SwivelJoystick &swivel,
                            const rammp::XYTwist &sample) {
  drive_swivel(logger, "base", swivel.base, sample.x, swivel.base_direction);
  drive_swivel(logger, "seat", swivel.seat, sample.y, swivel.seat_direction);
}

// Console CLI: the same operations the USB module exposes, on top of the same
// controller (so limits / rejections / tracked state agree).
std::unique_ptr<cli::Menu> make_cli_menu(stem::StemController &controller,
                                         std::span<EyouMotor> actuators, MotorActuator &base_swivel,
                                         MotorActuator &seat_swivel, CanBus &can_bus) {
  auto menu = std::make_unique<cli::Menu>("STEM");
  menu->Insert(
      "set",
      [&controller](std::ostream &out, const std::string &pair_name, float degrees) {
        const auto pair = stem::pair_from_name(pair_name);
        if (!pair) {
          out << "Unknown pair '" << pair_name << "'. Use M1_Seat, M2_Seat or M3_Seat (or m1 / m2 / m3).\n";
          return;
        }
        const auto result = controller.set_pair(*pair, degrees, 0.0f);
        out << pair_name << " -> " << degrees << " deg: " << stem::StemController::to_string(result)
            << "\n";
      },
      "Set one pair position at the default set speed: set <M1_Seat|M2_Seat|M3_Seat> <degrees>");
  menu->Insert(
      "ik",
      [](std::ostream &out, float x, float y) {
        stem::IkSolution solution{};
        if (!stem::solve_ik_for_m3(x, y, solution)) {
          out << "IK solution failed for x=" << x << ", y=" << y << "\n";
          return;
        }
        print_solution(out, solution);
      },
      "Solve the 5-bar IK at an absolute M3 point (analysis frame): ik <x> <y>");
  menu->Insert(
      "ik_ref",
      [&controller](std::ostream &out, float x_rel, float y_rel) {
        stem::PoseCommand pose{};
        if (!controller.solve(x_rel, y_rel, pose)) {
          out << "IK solution failed for x_rel=" << x_rel << ", y_rel=" << y_rel << "\n";
          return;
        }
        print_solution(out, pose.solution);
        out << "pairs: M1_Seat=" << pose.pair_deg[0] << " M2_Seat=" << pose.pair_deg[1]
            << " M3_Seat=" << pose.pair_deg[2] << " deg"
            << (pose.within_limits ? "" : " (OUTSIDE LIMITS)") << "\n";
      },
      "Solve the IK relative to the calibration reference (no motion): ik_ref <x_rel> <y_rel>");
  menu->Insert(
      "move",
      [&controller](std::ostream &out, float x_rel, float y_rel) {
        stem::PoseCommand pose{};
        const auto result = controller.move_to(x_rel, y_rel, 0.0f, &pose);
        out << "move x=" << x_rel << ", y=" << y_rel << ": "
            << stem::StemController::to_string(result);
        if (result != stem::StemController::Result::Unreachable) {
          out << " (M1_Seat=" << pose.pair_deg[0] << " M2_Seat=" << pose.pair_deg[1]
              << " M3_Seat=" << pose.pair_deg[2] << " deg)";
        }
        out << "\n";
      },
      "Solve IK relative to the reference and move all pairs: move <x_rel> <y_rel>");
  menu->Insert(
      "home",
      [&controller](std::ostream &out) {
        out << "home: " << stem::StemController::to_string(controller.home(0.0f)) << "\n";
      },
      "Move to the home position (reference pose): home");
  // Diagnostics: read objects from the drives (read only - nothing moves).
  auto print_sdo = [](std::ostream &out, EyouMotor &motor, uint16_t index, uint8_t sub,
                      const char *label) {
    uint32_t raw = 0;
    uint8_t size = 0;
    char line[160];
    if (!motor.read_sdo(index, sub, raw, size)) {
      std::snprintf(line, sizeof(line), "  %3d  0x%04X:%02X %-26s  no answer / aborted (code in the log)\n",
                    static_cast<int>(motor.node_id()), static_cast<unsigned>(index),
                    static_cast<unsigned>(sub), label);
    } else {
      float as_float = 0.0f;
      std::memcpy(&as_float, &raw, sizeof(as_float));
      const int32_t as_int = size == 1 ? static_cast<int8_t>(raw)
                             : size == 2 ? static_cast<int16_t>(raw)
                                         : static_cast<int32_t>(raw);
      std::snprintf(line, sizeof(line), "  %3d  0x%04X:%02X %-26s  raw 0x%08lX (%d B)  int %ld  real %g\n",
                    static_cast<int>(motor.node_id()), static_cast<unsigned>(index),
                    static_cast<unsigned>(sub), label, static_cast<unsigned long>(raw),
                    static_cast<int>(size),
                    static_cast<long>(as_int), size == 4 ? static_cast<double>(as_float) : 0.0);
    }
    out << line;
  };
  menu->Insert(
      "sdo_read",
      [actuators, print_sdo](std::ostream &out, int node, const std::string &index, int sub) {
        const auto idx = static_cast<uint16_t>(std::strtoul(index.c_str(), nullptr, 0));
        for (auto &motor : actuators) {
          if (node == 0 || motor.node_id() == node) {
            print_sdo(out, motor, idx, static_cast<uint8_t>(sub), "");
          }
        }
      },
      "Read an object from a drive (0 = all six): sdo_read <node> <index, e.g. 0x27BD> <sub>");
  menu->Insert(
      "sensor_info",
      [actuators, print_sdo](std::ostream &out) {
        struct Item {
          uint16_t index;
          uint8_t sub;
          const char *label;
        };
        constexpr Item items[] = {
            {0x1018, 2, "product code"},          {0x1018, 3, "revision"},
            {0x1018, 4, "serial number"},         {0x603F, 0, "error code (0x7512 = sensor)"},
            {0x2087, 0, "rated torque (int = mNm)"}, {0x2452, 0, "torque offset (tare)"},
            {0x2453, 0, "torque filter Hz"},      {0x27BD, 0, "torque value (int = mNm)"},
            {0x27BC, 0, "torque reference"},   {0x6061, 0, "mode display"},
        };
        for (auto &motor : actuators) {
          out << "motor " << static_cast<int>(motor.node_id()) << ":\n";
          for (const auto &item : items) {
            print_sdo(out, motor, item.index, item.sub, item.label);
          }
        }
      },
      "Torque-sensor diagnostics on all six drives (read only): sensor_info");
  menu->Insert(
      "heartbeat",
      [](std::ostream &out, int on) {
        heartbeat_enabled.store(on != 0);
        out << "heartbeat " << (on ? "ON" : "PAUSED (drives should fault within 300 ms)") << "\n";
      },
      "Pause / resume this board's CANopen heartbeat (watchdog test): heartbeat <0|1>");
  menu->Insert(
      "save_params",
      [actuators, &controller](std::ostream &out) {
        // 0x1010:01 "save" = every drive parameter to the drive's own EEPROM (~3 s each,
        // never power off meanwhile). Brakes must be engaged so nothing can move.
        if (!controller.brakes_engaged()) {
          out << "refused: engage the brakes first (console: Engage brakes & stop)\n";
          return;
        }
        for (auto &motor : actuators) {
          out << "  " << int(motor.node_id()) << ": saving..." << std::flush;
          if (!motor.save_parameters(1, 1000)) {
            out << " FAILED (write refused, see the log)\n";
            continue;
          }
          uint32_t raw = 0;
          uint8_t size = 0;
          bool done = false;
          for (int waited = 0; waited < 6000 && !done; waited += 250) {
            vTaskDelay(pdMS_TO_TICKS(250));
            done = motor.read_sdo(0x1010, 1, raw, size, 200) && raw == 2;
          }
          out << (done ? " saved\n" : " NOT confirmed (0x1010:01 never read 2)\n");
        }
        out << "done. Power-cycle the drives, then repeat the heartbeat test.\n";
      },
      "Save every drive parameter to the drives' EEPROM (brakes engaged; ~3 s per drive): "
      "save_params");
  menu->Insert(
      "measure",
      [&controller](std::ostream &out, int on, int mode) {
        out << "measure " << (on ? "on" : "off") << ": "
            << stem::StemController::to_string(
                   controller.set_measure_mode(on != 0, static_cast<int8_t>(mode)))
            << "\n";
      },
      "Measure mode (drives off, brakes hold, torque sensors read): measure <1|0> <mode 13|1>");
  menu->Insert(
      "tilt",
      [&controller](std::ostream &out, float tilt_deg) {
        out << "seat tilt " << tilt_deg
            << " deg: " << stem::StemController::to_string(controller.set_seat_tilt(tilt_deg, 0.0f))
            << "\n";
      },
      "Set the seat tilt offset (deg on top of the IK level angle), kept across moves: tilt <deg>");
  menu->Insert(
      "stop",
      [&controller](std::ostream &out) {
        out << (controller.stop() ? "All pairs stopped.\n" : "Stop failed for some pair.\n");
      },
      "Stop all actuator pairs: stop");
  menu->Insert(
      "get",
      [&controller](std::ostream &out) {
        const auto state = controller.snapshot(false);
        for (uint8_t i = 0; i < stem::kPairCount; ++i) {
          out << stem::to_string(static_cast<stem::Pair>(i)) << ": " << state.pair_deg[i]
              << " deg\n";
        }
        out << "seat tilt: " << state.seat_tilt_deg << " deg\n";
        out << "pose: ";
        if (state.pose_valid)
          out << "x_rel=" << state.pose_x << " y_rel=" << state.pose_y;
        else
          out << "(invalid)";
        out << "; target: ";
        if (state.target_valid)
          out << "x_rel=" << state.target_x << " y_rel=" << state.target_y;
        else
          out << "(none)";
        out << "\n";
      },
      "Get tracked pair positions and the resulting pose: get");
  menu->Insert(
      "status",
      [actuators](std::ostream &out) {
        for (auto &actuator : actuators) {
          uint16_t statusword = 0;
          float position_degrees = 0.0f;
          if (actuator.get_statusword(statusword) && actuator.get_position(position_degrees)) {
            out << "Motor " << static_cast<int>(actuator.node_id())
                << ": state=" << EyouMotor::ds402_state_name(statusword)
                << ", position=" << position_degrees << " deg";
            int16_t torque = 0;
            float velocity = 0.0f;
            if (actuator.get_torque(torque) && actuator.get_velocity(velocity)) {
              out << ", torque=" << torque << " permille, velocity=" << velocity << " deg/s";
            }
            out << "\n";
          } else {
            out << "Motor " << static_cast<int>(actuator.node_id()) << ": status read failed\n";
          }
        }
      },
      "Get status for all motors: status");
  auto report_torque_limit = [&controller](std::ostream &out, stem::StemController::Result result) {
    out << stem::StemController::to_string(result);
    if (result == stem::StemController::Result::OutOfLimits) {
      out << " (ceiling " << controller.config().max_torque_permille << " permille";
      if (controller.rated_torque_mnm() != 0) {
        out << " = " << controller.permille_to_nm(controller.config().max_torque_permille) << " Nm";
      }
      out << ", menuconfig)";
    }
    if (result == stem::StemController::Result::Ok) {
      out << ": every motor at " << controller.torque_limit() << " permille of rated";
      if (controller.rated_torque_mnm() != 0) {
        out << " = " << controller.permille_to_nm(controller.torque_limit()) << " Nm";
      }
    }
    out << "\n";
  };
  menu->Insert(
      "torque_limit",
      [&controller, report_torque_limit](std::ostream &out, float newton_metres) {
        out << "torque limit " << newton_metres << " Nm: ";
        report_torque_limit(out, controller.set_torque_limit_nm(newton_metres));
        if (controller.rated_torque_mnm() == 0) {
          out << "  (the rated torque could not be read from any motor: torque_limit_permille "
                 "<n> sets it relative to the rated torque)\n";
        }
      },
      "Set every motor's torque limit in newton-metres, up to the menuconfig ceiling: "
      "torque_limit <Nm>");
  menu->Insert(
      "torque_limit_permille",
      [&controller, report_torque_limit](std::ostream &out, int permille) {
        if (permille < 0 || permille > 0xFFFF) {
          out << "torque limit must be 0..65535 permille of the rated torque\n";
          return;
        }
        out << "torque limit " << permille << " permille: ";
        report_torque_limit(out, controller.set_torque_limit(static_cast<uint16_t>(permille)));
      },
      "The same, per-thousand of the rated torque (1000 = rated): torque_limit_permille <n>");
  menu->Insert(
      "set_node_id",
      [actuators](std::ostream &out, int old_id, int new_id) {
        if (old_id < 1 || old_id > 127 || new_id < 1 || new_id > 127) {
          out << "node ids are 1..127\n";
          return;
        }
        for (auto &actuator : actuators) {
          if (actuator.node_id() != static_cast<uint8_t>(old_id)) {
            continue;
          }
          out << "re-addressing the drive at node " << old_id << " to " << new_id
              << " (writes both Node-ID objects, saves, resets communication; ~5 s)...\n";
          switch (actuator.set_drive_node_id(static_cast<uint8_t>(new_id))) {
          case EyouMotor::NodeIdResult::Applied:
            out << "applied: the drive answers on " << new_id
                << " and this motor object follows it; its software zero is per node - "
                   "load_zero / zero again\n";
            return;
          case EyouMotor::NodeIdResult::SavedNeedsPowerCycle:
            out << "saved: the drive keeps " << old_id
                << " until it is power-cycled; this object stays on the old id\n";
            return;
          case EyouMotor::NodeIdResult::Failed:
            out << "FAILED: nothing changed (see the log)\n";
            return;
          }
        }
        out << "no motor object talks to node " << old_id << " (the pairs use 101-106)\n";
      },
      "Re-address an Eyou drive (replacement actuator): set_node_id <current id> <new id>");
  menu->Insert(
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
                 "tied to the bus ground, short stubs, or a node at another bitrate (the Eyou "
                 "drives run 1 Mbit/s)\n";
        } else if (s.ack_errors > 0) {
          out << "  frames we sent were not acknowledged by anyone: bus open / all nodes off at "
                 "that moment\n";
        } else {
          out << "  no bus errors: a node that does not answer is off, unpowered, at another id "
                 "or not on this bus\n";
        }
      },
      "Print the CAN controller state and error counters (bitrate / wiring check)");
  menu->Insert(
      "can_recover",
      [&can_bus](std::ostream &out) {
        out << (can_bus.recover() ? "bus-off recovery started; `can` shows when it is back\n"
                                  : "recovery not started (see the log)\n");
      },
      "Start the CAN bus-off recovery");
  menu->Insert(
      "release",
      [&controller](std::ostream &out) {
        out << (controller.release_brakes() ? "All motor brakes released.\n"
                                            : "Failed to release some brakes.\n");
      },
      "Release brakes for all actuator pairs: release");
  menu->Insert(
      "engage",
      [&controller](std::ostream &out) {
        out << (controller.engage_brakes() ? "Brakes engaged: drives off, moves refused until release.\n"
                                           : "Engage: a drive did not confirm (see the log).\n");
      },
      "Stop and engage the brakes (drives off) until release: engage");
  menu->Insert(
      "reset_fault",
      [&controller](std::ostream &out) {
        out << (controller.reset_fault()
                    ? "Faults cleared, drives enabled (see `status`).\n"
                    : "A fault did not clear or a drive did not re-enable: its cause is still "
                      "there (overload, a hard stop, the limit) - see `status` for the error "
                      "code, fix the cause, try again.\n");
      },
      "Clear the DS402 fault of every faulted motor and re-enable its pair: reset_fault");
  menu->Insert(
      "zero",
      [&controller](std::ostream &out, const std::string &pair_name) {
        const auto pair = stem::pair_from_name(pair_name);
        if (!pair && pair_name != "all") {
          out << "Unknown pair '" << pair_name << "'. Use M1_Seat, M2_Seat, M3_Seat (m1 / m2 / m3), or all.\n";
          return;
        }
        controller.zero(pair, false);
        out << pair_name << " zeroed without alignment.\n";
      },
      "Set a pair's (or all pairs') current position as zero without alignment: zero "
      "<M1_Seat|M2_Seat|M3_Seat|all>");
  menu->Insert(
      "zero_align",
      [&controller](std::ostream &out, const std::string &pair_name) {
        const auto pair = stem::pair_from_name(pair_name);
        if (!pair && pair_name != "all") {
          out << "Unknown pair '" << pair_name << "'. Use M1_Seat, M2_Seat, M3_Seat (m1 / m2 / m3), or all.\n";
          return;
        }
        controller.zero(pair, true);
        out << pair_name << " zeroed after alignment.\n";
      },
      "Set a pair's (or all pairs') current position as zero after alignment: zero_align "
      "<M1_Seat|M2_Seat|M3_Seat|all>");
  menu->Insert(
      "save_zero",
      [&controller](std::ostream &out) {
        out << (controller.save_zero() ? "All pairs' zero saved to NVS.\n"
                                       : "Zero save failed for one or more pairs.\n");
      },
      "Persist every pair's software zero to NVS: save_zero");
  menu->Insert(
      "read_zero",
      [&controller](std::ostream &out) {
        std::array<std::array<int32_t, 2>, stem::kPairCount> zero_pulses{};
        if (!controller.read_zero(zero_pulses)) {
          out << "Failed to read one or more saved software zeros from NVS.\n";
          return;
        }
        const auto &config = controller.config();
        const PairedEyouActuator *pairs[] = {&config.m1_seat, &config.m2_seat, &config.m3_seat};
        for (uint8_t i = 0; i < stem::kPairCount; ++i) {
          out << stem::to_string(static_cast<stem::Pair>(i)) << " (nodes "
              << static_cast<int>(pairs[i]->primary_id()) << ", "
              << static_cast<int>(pairs[i]->secondary_id()) << "): " << zero_pulses[i][0] << ", "
              << zero_pulses[i][1] << " pulses\n";
        }
      },
      "Read every motor's saved software zero without loading it: read_zero");
  menu->Insert(
      "load_zero",
      [&controller](std::ostream &out) {
        out << (controller.load_zero() ? "All pairs' zero loaded from NVS.\n"
                                       : "Zero load failed for one or more pairs.\n");
        if (controller.has_zero_mismatch()) {
          out << "ERROR: pair zero mismatch detected; moves are blocked until re-zeroed.\n";
        }
      },
      "Load every pair's software zero from NVS: load_zero");
  menu->Insert(
      "check_zero",
      [&controller](std::ostream &out) {
        out << (controller.check_pairs()
                    ? "All pairs are mirrored within tolerance.\n"
                    : "ERROR: pair zero mismatch detected; moves are blocked until re-zeroed.\n");
      },
      "Check every pair's positions are mirrored (primary ~= -secondary): check_zero");
  menu->Insert(
      "set_base",
      [&base_swivel](std::ostream &out, float swivel_degrees) {
        const float actuator_degrees = swivel_degrees * kSwivelGearRatio;
        out << (base_swivel.set_position(actuator_degrees, kSwivelDefaultSpeedRpm)
                    ? "Base swivel angle command sent.\n"
                    : "Base swivel angle command failed.\n");
      },
      "Set the base swivel angle, accounting for the 1:7 external gear: set_base <degrees>");
  menu->Insert(
      "set_seat_swivel",
      [&seat_swivel](std::ostream &out, float swivel_degrees) {
        const float actuator_degrees = swivel_degrees * kSwivelGearRatio;
        out << (seat_swivel.set_position(actuator_degrees, kSwivelDefaultSpeedRpm)
                    ? "Seat swivel angle command sent.\n"
                    : "Seat swivel angle command failed.\n");
      },
      "Set the seat swivel angle, accounting for the 1:7 external gear: set_seat_swivel <degrees>");
  menu->Insert(
      "get_base",
      [&base_swivel](std::ostream &out) {
        out << "Base swivel angle: " << base_swivel.get_position() / kSwivelGearRatio << " deg\n";
      },
      "Read the tracked base swivel angle: get_base");
  menu->Insert(
      "get_seat_swivel",
      [&seat_swivel](std::ostream &out) {
        out << "Seat swivel angle: " << seat_swivel.get_position() / kSwivelGearRatio << " deg\n";
      },
      "Read the tracked seat swivel angle: get_seat_swivel");
  return menu;
}

} // namespace

extern "C" void app_main(void) {
  auto &board = espp::Esp32P4Eth::get();
  espp::Logger logger({.tag = "STEM", .level = espp::Logger::Verbosity::INFO});
  logger.info("Bootup");

  // --------------------------------------------------------------------------
  // Previous-crash report (flash core dump) and OTA state
  // --------------------------------------------------------------------------
  espp::CoreDump core_dump;
  const std::string crash_report = core_dump.format_report();
  if (!crash_report.empty()) {
    logger.error("Previous abnormal reset:\n{}", crash_report);
  }

  espp::Ota ota({.reject_same_version = false, .log_level = espp::Logger::Verbosity::INFO});
  const auto running = ota.running_app_description();
  logger.info("Running '{}' version '{}' (built {} {}, IDF {}) from partition '{}'",
              running.project_name, running.version, running.date, running.time,
              running.idf_version, ota.running_partition_label());

  std::mutex ethernet_ip_mutex;
  std::string ethernet_ip;
  espp::Esp32P4Eth::EthernetConfig ethernet_config{};
  ethernet_config.mode = espp::Esp32P4Eth::DhcpMode::CLIENT;
  ethernet_config.on_got_ip = [&](esp_ip4_addr_t ip) {
    std::lock_guard<std::mutex> lock(ethernet_ip_mutex);
    ethernet_ip =
        std::to_string(esp_ip4_addr1_16(&ip)) + "." + std::to_string(esp_ip4_addr2_16(&ip)) + "." +
        std::to_string(esp_ip4_addr3_16(&ip)) + "." + std::to_string(esp_ip4_addr4_16(&ip));
  };
  if (!board.initialize_ethernet(ethernet_config)) {
    logger.error("Failed to initialize Ethernet");
  } else {
    logger.info("Ethernet initialized as DHCP client");
  }

  // --------------------------------------------------------------------------
  // CAN bus, actuators, pairs, controller
  // --------------------------------------------------------------------------
  CanBus can_bus(kCanRxGpio, kCanTxGpio);
  // EyouMotor registers its CAN receivers in its constructor, which must happen
  // before CanBus::start() (registration is locked once the bus is running).
  std::array<EyouMotor, stem::kMotorCount> actuators{{
      EyouMotor(can_bus, 101),
      EyouMotor(can_bus, 102),
      EyouMotor(can_bus, 103),
      EyouMotor(can_bus, 104),
      EyouMotor(can_bus, 105),
      EyouMotor(can_bus, 106),
  }};
  // MotorCanBus registers its own receivers inside start(), so construct it and
  // the two swivel motors before starting the bus, and let its start() bring up
  // the shared CanBus (do not call can_bus.start() separately).
  MotorCanBus swivel_can_bus(can_bus);
  MotorActuator::CommunicationFunction swivel_communicate =
      [&swivel_can_bus](const MotorPacket &command, MotorPacket &response, uint32_t timeout_ms) {
        if (timeout_ms == 0)
          return swivel_can_bus.send(command);
        return swivel_can_bus.request(command, response, timeout_ms);
      };
  MotorActuator base_swivel(swivel_communicate, kBaseSwivelMotorId);
  MotorActuator seat_swivel(swivel_communicate, kSeatSwivelMotorId);
  const bool can_ok = swivel_can_bus.start();
  if (!can_ok) {
    // Keep going: USB / OTA / core-dump access must not depend on the motors.
    logger.error("CAN bus failed to start; motor commands will fail until reboot");
  }
  // Reversed pairs (front / back plane of each axis): 101-102 M1_Seat, 103-104 M2_Seat, 105-106 M3_Seat
  // The tags are literals: the logger keeps a view of its tag, and a tag set
  // after construction would need the mutable-tag logger option (RAM).
  PairedEyouActuator m2_seat_pair(actuators[3], actuators[2], "PairedEyouActuator-M2_Seat");
  PairedEyouActuator m1_seat_pair(actuators[1], actuators[0], "PairedEyouActuator-M1_Seat");
  PairedEyouActuator m3_seat_pair(actuators[5], actuators[4], "PairedEyouActuator-M3_Seat");
  logger.info("Six motor actuators (IDs 101-106) in three reversed pairs: M1_Seat 102-101, "
              "M2_Seat 104-103, M3_Seat 106-105");

  const EyouMotor::ProfilePositionConfig profile{
      .velocity_degrees_per_second = 60.0f,
      .acceleration_degrees_per_second_squared = 20.0f,
      .deceleration_degrees_per_second_squared = 20.0f,
  };

  stem::StemController controller({
      .m1_seat = m1_seat_pair,
      .m2_seat = m2_seat_pair,
      .m3_seat = m3_seat_pair,
      .motors = actuators,
      .can_ok = can_ok,
      .max_torque_permille = kMaxTorquePermille,
      .can_bus = &can_bus,
      // a STOP must not queue behind a set-point nobody acknowledged
      .flush_pending_commands = [&can_bus]() { can_bus.flush_pending(); },
      .profile = profile, // restored after every seat-level pose move
      .log_level = espp::Logger::Verbosity::INFO,
  });

  if (can_ok) {
    // Mode before the controlword: a fresh Eyou drive refuses the DS402 enable
    // sequence until modes of operation is set (enable_drive() also prepares
    // it, idempotently), and the torque limit before anything can move.
    // enable = false: the drives stay OFF (brakes engaged) through boot; only
    // the console's "Release brakes" switches them on.
    if (!m1_seat_pair.configure_profile_position(profile, false) ||
        !m2_seat_pair.configure_profile_position(profile, false) ||
        !m3_seat_pair.configure_profile_position(profile, false)) {
      logger.error("Failed to configure profile-position mode on one or more pairs");
    }
    if (controller.set_torque_limit(kBootTorquePermille) != stem::StemController::Result::Ok) {
      logger.error("Failed to write the boot torque limit ({} permille) to one or more motors",
                   kBootTorquePermille);
    } else if (controller.rated_torque_mnm() != 0) {
      logger.info("Torque limit {} permille of rated = {:.2f} Nm on every motor (ceiling {} = "
                  "{:.2f} Nm, menuconfig; rated {} mNm)",
                  kBootTorquePermille, controller.permille_to_nm(kBootTorquePermille),
                  kMaxTorquePermille, controller.permille_to_nm(kMaxTorquePermille),
                  controller.rated_torque_mnm());
    } else {
      logger.info("Torque limit {} permille of rated on every motor (ceiling {}, menuconfig); "
                  "the rated torque was not read, Nm figures come once a motor answers",
                  kBootTorquePermille, kMaxTorquePermille);
    }
    for (auto &actuator : actuators) {
      if (!actuator.set_heartbeat_consumer(kHeartbeatNodeId, kHeartbeatTimeoutMs)) {
        logger.warn("Motor {}: heartbeat watchdog not configured", actuator.node_id());
      }
    }
    // Boot with the brakes ENGAGED: only the user releases them (console
    // "Release brakes"). Releasing enables the drives, which also clears any
    // fault left by the watchdog.
    controller.start_engaged();
    controller.zero(std::nullopt, false);
    if (!controller.load_zero()) {
      logger.warn("Failed to load zero for one or more pairs, please load zero manually");
    }
    if (controller.has_zero_mismatch()) {
      logger.error("Pair zero mismatch after load_zero: moves are blocked until re-zeroed");
    }
    for (auto &actuator : actuators) {
      uint16_t statusword = 0;
      if (actuator.get_statusword(statusword)) {
        logger.info("Motor {} state: {}", actuator.node_id(),
                    EyouMotor::ds402_state_name(statusword));
      } else {
        logger.warn("Motor {} status read failed", actuator.node_id());
      }
    }
  }

  // --------------------------------------------------------------------------
  // Pose moves: MOVE / HOME start a seat-level trajectory; this task streams its
  // set-points and runs the level guard (StemController::service).
  // --------------------------------------------------------------------------
  espp::Task heartbeat_task(
      {.callback = [&](std::mutex &m, std::condition_variable &cv) -> bool {
         // Only while the bus is healthy: with the drives unpowered nothing ACKs,
         // and stacking heartbeats on an unacknowledged bus kept the TX queue
         // full (no SDO got through until a reset). Resumes by itself.
         const auto bus = can_bus.get_status();
         if (heartbeat_enabled.load() && !bus.bus_off && bus.tx_error_count < 96) {
           CanFrame frame{};
           frame.id = 0x700 + kHeartbeatNodeId;
           frame.dlc = 1;
           frame.data[0] = 0x05; // operational
           can_bus.send(frame, false); // never block: a full queue just skips one beat
         }
         std::unique_lock<std::mutex> lock(m);
         cv.wait_for(lock, kHeartbeatPeriod);
         return false;
       },
       .task_config = {.name = "can_heartbeat", .stack_size_bytes = 3072, .priority = 10}});
  if (can_ok) {
    heartbeat_task.start();
    logger.info("Heartbeat {} ms as node {} (drives fault after {} ms without it)",
                std::chrono::duration_cast<std::chrono::milliseconds>(kHeartbeatPeriod).count(),
                kHeartbeatNodeId, kHeartbeatTimeoutMs);
  }

  espp::Task motion_task(
      {.callback = [&](std::mutex &m, std::condition_variable &cv) -> bool {
         // TEMP(tick-timing): measure how long service() takes and the real
         // tick period during pose moves, logged once a second. Remove after the
         // 20 Hz streaming decision (search "TEMP(tick-timing)").
         static auto tt_last_start = std::chrono::steady_clock::now();
         static auto tt_last_log = tt_last_start;
         static float tt_sum_ms = 0, tt_max_ms = 0, tt_period_sum_ms = 0, tt_period_max_ms = 0;
         static int tt_n = 0;
         const auto tt_start = std::chrono::steady_clock::now();
         const bool tt_moving = controller.moving();
         controller.service();
         if (tt_moving) {
           using ms = std::chrono::duration<float, std::milli>;
           const float busy = ms(std::chrono::steady_clock::now() - tt_start).count();
           const float period = ms(tt_start - tt_last_start).count();
           tt_sum_ms += busy;
           tt_max_ms = std::max(tt_max_ms, busy);
           tt_period_sum_ms += period;
           tt_period_max_ms = std::max(tt_period_max_ms, period);
           if (++tt_n > 0 && tt_start - tt_last_log >= 1s) {
             logger.info("TEMP tick: service avg {:.1f} max {:.1f} ms | period avg {:.1f} max {:.1f} ms ({} ticks)",
                         tt_sum_ms / tt_n, tt_max_ms, tt_period_sum_ms / tt_n, tt_period_max_ms, tt_n);
             tt_sum_ms = tt_max_ms = tt_period_sum_ms = tt_period_max_ms = 0;
             tt_n = 0;
             tt_last_log = tt_start;
           }
         } else {
           tt_n = 0;
           tt_sum_ms = tt_max_ms = tt_period_sum_ms = tt_period_max_ms = 0;
           tt_last_log = tt_start;
         }
         tt_last_start = tt_start;
         // END TEMP(tick-timing)
         std::unique_lock<std::mutex> lock(m);
         cv.wait_for(lock, 100ms);
         return false; // don't stop the task
       },
       .task_config = {.name = "stem_motion", .stack_size_bytes = 6144}});
  if (can_ok) {
    motion_task.start();
  }

  // --------------------------------------------------------------------------
  // RTPS: on_got_ip runs in the event loop, so it is brought up from this task.
  // Started after the motors exist because the subscribers command them.
  // --------------------------------------------------------------------------
  SwivelJoystick swivel_joystick{.base = base_swivel, .seat = seat_swivel};
  std::unique_ptr<espp::RtpsParticipant> rtps_participant;
  std::unique_ptr<espp::Subscriber<rammp::XYTwist>> seat_xy_twist_subscriber;
  std::unique_ptr<espp::Subscriber<rammp::XYTwist>> swivel_xy_twist_subscriber;
  espp::Task rtps_task(
      {.callback = [&](std::mutex &m, std::condition_variable &cv) -> bool {
         std::string ip;
         {
           std::lock_guard<std::mutex> lock(ethernet_ip_mutex);
           ip = ethernet_ip;
         }
         if (ip.empty()) {
           std::unique_lock<std::mutex> lock(m);
           cv.wait_for(lock, 100ms);
           return false;
         }
         logger.info("Ethernet got IP {}; starting RTPS", ip);
         auto participant = std::make_unique<espp::RtpsParticipant>(espp::RtpsParticipant::Config{
             .interface_address = ip,
             .log_level = espp::Logger::Verbosity::INFO,
         });
         if (!participant->start()) {
           logger.error("Failed to start RTPS on {}", ip);
           return true;
         }
         rtps_participant = std::move(participant);
         logger.info("RTPS participant started on {}", ip);
         seat_xy_twist_subscriber = std::make_unique<espp::Subscriber<rammp::XYTwist>>(
             *rtps_participant,
             espp::Subscriber<rammp::XYTwist>::Config{
                 .topic = kSeatXyTwistTopic,
                 .type_name = RAMMP_TYPE_XY_TWIST,
                 .reliability = espp::RtpsParticipant::Reliability::BEST_EFFORT,
                 .on_message = std::bind_front(&handle_seat_xy_twist, std::ref(logger))});
         if (!seat_xy_twist_subscriber->is_valid()) {
           logger.error("Failed to create RTPS subscriber for {}", kSeatXyTwistTopic);
         }
         swivel_xy_twist_subscriber = std::make_unique<espp::Subscriber<rammp::XYTwist>>(
             *rtps_participant,
             espp::Subscriber<rammp::XYTwist>::Config{
                 .topic = kSwivelXyTwistTopic,
                 .type_name = RAMMP_TYPE_XY_TWIST,
                 .reliability = espp::RtpsParticipant::Reliability::BEST_EFFORT,
                 .on_message = std::bind_front(&handle_swivel_xy_twist, std::ref(logger),
                                               std::ref(swivel_joystick))});
         if (!swivel_xy_twist_subscriber->is_valid()) {
           logger.error("Failed to create RTPS subscriber for {}", kSwivelXyTwistTopic);
         }
         return true;
       },
       .task_config = {.name = "stem_rtps", .stack_size_bytes = 8192}});
  rtps_task.start();

  // --------------------------------------------------------------------------
  // USB composite device: vendor / WebUSB + CDC, each an independent frame stream
  // --------------------------------------------------------------------------
  espp::UsbDevice::Config usb_cfg;
  usb_cfg.pid = kUsbPid;
  usb_cfg.manufacturer = "rammp";
  usb_cfg.product = "STEM Linkage";
  usb_cfg.log_level = espp::Logger::Verbosity::INFO;
  espp::UsbDevice::CdcFunction cdc;
  cdc.interface_name = "STEM Linkage (serial frames)";
  usb_cfg.cdc = cdc;
  espp::UsbDevice::VendorFunction vendor;
  vendor.interface_name = "STEM Linkage (WebUSB)";
  vendor.webusb = true; // BOS / WebUSB / MS OS 2.0 descriptors
  // Chrome offers this on plug-in; the hosted hub discovers the modules and
  // links the stock OTA / core-dump consoles (see web/README.md for the STEM
  // console, which is served locally next to a copy of the hub).
  vendor.landing_page_url = "esp-cpp.github.io/espp/apps/dispatcher_hub.html";
  usb_cfg.vendor = vendor;
  espp::UsbDevice usb(usb_cfg);

  // One TX mutex per stream: replies (RX worker), telemetry and discovery all write.
  std::mutex vendor_tx_mutex, cdc_tx_mutex;
  auto send_vendor = [&](std::span<const uint8_t> frame) -> bool {
    if (frame.empty())
      return true;
    std::lock_guard<std::mutex> lock(vendor_tx_mutex);
    std::error_code ec;
    if (!usb.write_vendor(frame, ec)) {
      logger.warn_rate_limited("USB vendor TX dropped {}-byte frame: {}", frame.size(),
                               ec.message());
      return false;
    }
    return true;
  };
  auto send_cdc = [&](std::span<const uint8_t> frame) -> bool {
    if (frame.empty())
      return true;
    std::lock_guard<std::mutex> lock(cdc_tx_mutex);
    std::error_code ec;
    if (!usb.write_cdc(frame, ec)) {
      logger.warn_rate_limited("USB CDC TX dropped {}-byte frame: {}", frame.size(), ec.message());
      return false;
    }
    return true;
  };
  auto sender = [&](Transport transport) -> send_fn {
    return transport == Transport::Vendor ? send_fn(send_vendor) : send_fn(send_cdc);
  };

  // RX bytes arrive in the TinyUSB task context: queue them (tagged by stream)
  // and dispatch from the worker below (OTA flash erase / core-dump erase can
  // block for a long time and must not stall the USB stack).
  std::mutex rx_mutex;
  std::condition_variable rx_cv;
  std::deque<std::pair<Transport, std::vector<uint8_t>>> rx_queue;
  size_t rx_queued_bytes = 0;
  std::array<bool, stem::StemModule::kTransportCount> rx_dropped{};
  auto enqueue_rx = [&](Transport transport, std::span<const uint8_t> data) {
    {
      std::lock_guard<std::mutex> lock(rx_mutex);
      if (rx_queued_bytes + data.size() > kMaxQueuedRxBytes) {
        rx_dropped[static_cast<size_t>(transport)] = true; // worker resyncs that stream
        return;
      }
      rx_queue.emplace_back(transport, std::vector<uint8_t>(data.begin(), data.end()));
      rx_queued_bytes += data.size();
    }
    rx_cv.notify_one();
  };
  usb.set_vendor_receive_callback(
      [&](std::span<const uint8_t> data) { enqueue_rx(Transport::Vendor, data); });
  usb.set_cdc_receive_callback(
      [&](std::span<const uint8_t> data) { enqueue_rx(Transport::Cdc, data); });

  std::error_code usb_ec;
  const bool usb_ok = usb.initialize(usb_ec);
  if (!usb_ok) {
    logger.error("USB device failed to initialize: {} (web console / OTA / core dump "
                 "unavailable; CLI and motors still run)",
                 usb_ec.message());
  }

  // --------------------------------------------------------------------------
  // Protocol modules on each stream's dispatcher
  // --------------------------------------------------------------------------
  espp::Dispatcher vendor_dispatcher, cdc_dispatcher;
  espp::CoreDumpService vendor_coredump(
      core_dump, {.send = [&](std::span<const uint8_t> f) { send_vendor(f); },
                  .log_level = espp::Logger::Verbosity::INFO});
  espp::CoreDumpService cdc_coredump(core_dump,
                                     {.send = [&](std::span<const uint8_t> f) { send_cdc(f); },
                                      .log_level = espp::Logger::Verbosity::INFO});
  stem::StemModule stem_module({
      .controller = controller,
      .app = {.project = running.project_name,
              .version = running.version,
              .build = running.date + " " + running.time,
              .idf = running.idf_version},
      .log_level = espp::Logger::Verbosity::INFO,
  });
  std::atomic<bool> restart_pending{false};

  // --- OTA (module 0): the standard espp ota_stream handling, replying on the
  //     stream the request came in on, so a plain ota_console can update us.
  auto handle_ota_frame = [&](const Frame &frame, const send_fn &send) {
    auto ota_error = [&](const std::error_code &err, const std::string &ctx) {
      send(otap::make_error(static_cast<uint32_t>(err.value()), ctx + ": " + err.message()));
    };
    std::error_code ec;
    switch (static_cast<otap::MessageType>(frame.type)) {
    case otap::MessageType::Begin: {
      const auto image_size = otap::parse_u32_payload(frame);
      if (!image_size.has_value()) {
        ota_error(std::make_error_code(std::errc::invalid_argument), "malformed BEGIN");
        break;
      }
      if (ota.begin(*image_size, ec))
        send(otap::make_ok(0));
      else
        ota_error(ec, "OTA begin failed");
      break;
    }
    case otap::MessageType::Data:
      if (!ota.session_active()) {
        ota_error(std::make_error_code(std::errc::operation_not_permitted),
                  "no update session (send BEGIN first)");
        break;
      }
      if (ota.write(frame.payload, ec))
        send(otap::make_ok(static_cast<uint32_t>(ota.bytes_written())));
      else
        ota_error(ec, "OTA write failed"); // write() aborted the session on failure
      break;
    case otap::MessageType::Finish: {
      if (!ota.session_active()) {
        ota_error(std::make_error_code(std::errc::operation_not_permitted),
                  "no update session (send BEGIN first)");
        break;
      }
      const auto written = static_cast<uint32_t>(ota.bytes_written());
      if (ota.finish(ec)) {
        send(otap::make_ok(written));
        restart_pending = true; // reply first; the worker restarts shortly
      } else {
        ota_error(ec, "OTA finish (validate/activate) failed");
      }
      break;
    }
    case otap::MessageType::Abort: {
      if (!ota.session_active()) {
        ota_error(std::make_error_code(std::errc::operation_not_permitted),
                  "no update session to abort");
        break;
      }
      const auto written = static_cast<uint32_t>(ota.bytes_written());
      if (ota.abort(ec))
        send(otap::make_ok(written));
      else
        ota_error(ec, "OTA abort failed");
      break;
    }
    default:
      ota_error(std::make_error_code(std::errc::not_supported), "unknown OTA message");
      break;
    }
  };

  // Register the three modules on a stream's dispatcher. Every handler gates on
  // !is_reply() so an echoed reply can never re-enter it.
  auto register_modules = [&](espp::Dispatcher &dispatcher, Transport transport,
                              espp::CoreDumpService &coredump_service) {
    const send_fn send = sender(transport);
    dispatcher.register_module(otap::kModule,
                               [&, send](const Frame &frame) {
                                 if (!frame.is_reply())
                                   handle_ota_frame(frame, send);
                               },
                               {.name = "OTA",
                                .app = "ota_console.html",
                                .description = "Firmware update over USB",
                                .protocol = "espp.ota",
                                .protocol_version = 1});
    dispatcher.register_module(espp::CoreDumpService::kModule,
                               [&coredump_service](const Frame &frame) {
                                 if (!frame.is_reply())
                                   coredump_service.handle_frame(frame.type, frame.payload);
                               },
                               {.name = "Core Dump",
                                .app = "coredump_console.html",
                                .description = "Inspect the last crash core dump",
                                .protocol = "espp.coredump",
                                .protocol_version = 1});
    dispatcher.register_module(
        stem_proto::kModule,
        [&, send, transport](const Frame &frame) {
          if (!frame.is_reply())
            stem_module.handle_frame(frame, transport, send);
        },
        stem::StemModule::module_info());
    dispatcher.set_device_info(usb_cfg.product, running.version);
    dispatcher.serve_discovery([send](std::span<const uint8_t> f) { send(f); });
  };
  register_modules(vendor_dispatcher, Transport::Vendor, vendor_coredump);
  register_modules(cdc_dispatcher, Transport::Cdc, cdc_coredump);

  auto dispatcher_for = [&](Transport transport) -> espp::Dispatcher & {
    return transport == Transport::Vendor ? vendor_dispatcher : cdc_dispatcher;
  };

  espp::Task rx_task(
      {.callback = [&](std::mutex &, std::condition_variable &) -> bool {
         std::deque<std::pair<Transport, std::vector<uint8_t>>> chunks;
         std::array<bool, stem::StemModule::kTransportCount> dropped{};
         {
           std::unique_lock<std::mutex> lock(rx_mutex);
           rx_cv.wait_for(lock, 100ms, [&] { return !rx_queue.empty(); });
           std::swap(chunks, rx_queue);
           rx_queued_bytes = 0;
           dropped = rx_dropped;
           rx_dropped = {};
         }
         for (size_t i = 0; i < dropped.size(); ++i) {
           if (!dropped[i])
             continue;
           // Bytes were lost on this stream: any in-flight frame / OTA image is
           // unusable. Resync the parser and tell the host so it does not wait
           // for its own timeout.
           const auto transport = static_cast<Transport>(i);
           dispatcher_for(transport).reset();
           if (ota.session_active()) {
             std::error_code abort_ec;
             ota.abort(abort_ec);
             sender(transport)(otap::make_error(
                 static_cast<uint32_t>(std::make_error_code(std::errc::no_buffer_space).value()),
                 "RX overflow: frames dropped, update aborted"));
           } else {
             sender(transport)(stem_proto::make_error(
                 stem_proto::Msg::GetState,
                 static_cast<uint32_t>(std::make_error_code(std::errc::no_buffer_space).value()),
                 "RX overflow: frames dropped -- wait for replies between commands"));
           }
         }
         for (const auto &[transport, bytes] : chunks) {
           dispatcher_for(transport).feed(bytes);
         }
         if (restart_pending) {
           std::this_thread::sleep_for(750ms); // let the final OK reach the host
           ota.restart();
         }
         return false; // don't stop the task
       },
       .task_config = {.name = "stem_usb_rx", .stack_size_bytes = 8192}});

  // --------------------------------------------------------------------------
  // Telemetry streaming (per stream: its own device-side period and deadline)
  // --------------------------------------------------------------------------
  using TelemetryClock = std::chrono::steady_clock;
  std::array<TelemetryClock::time_point, stem::StemModule::kTransportCount> telemetry_stall_start{};
  std::array<TelemetryClock::time_point, stem::StemModule::kTransportCount> telemetry_next_send{};
  constexpr auto kTelemetryIdlePoll = 100ms; // how soon a newly enabled stream is noticed
  espp::Task telemetry_task(
      {.callback = [&](std::mutex &m, std::condition_variable &cv) -> bool {
         const auto now = TelemetryClock::now();
         auto wake = now + kTelemetryIdlePoll;
         // one snapshot per tick, and only when some stream is due: the status
         // reads it costs are blocking CAN transactions
         std::optional<decltype(controller.snapshot(true))> state;
         // Pause while an OTA transfer runs so the bulk pipes carry only the
         // flow-controlled OTA replies.
         const bool paused = ota.session_active();
         for (size_t i = 0; i < stem::StemModule::kTransportCount; ++i) {
           const auto transport = static_cast<Transport>(i);
           if (!stem_module.streaming(transport))
             continue;
           const char *name = transport == Transport::Vendor ? "WebUSB" : "CDC";
           const bool connected =
               transport == Transport::Vendor ? usb.is_vendor_connected() : usb.is_cdc_connected();
           auto &stall_start = telemetry_stall_start[i];
           if (!connected) {
             // An abrupt disconnect ends the stream with its session: no more
             // snapshots for nobody, and a reconnecting host starts clean
             // (its own SET_STREAMING) instead of inheriting this one.
             stem_module.set_streaming(transport, false);
             stall_start = {};
             if (transport == Transport::Vendor)
               usb.vendor_write_clear();
             else
               usb.cdc_write_clear();
             logger.info("Telemetry stopped on {}: the host disconnected", name);
             continue;
           }
           const auto period = std::chrono::milliseconds(stem_module.stream_period_ms(transport));
           auto &next_send = telemetry_next_send[i];
           if (next_send == TelemetryClock::time_point{} || next_send > now + period)
             next_send = now; // (re)started, or the period was shortened
           if (now < next_send) {
             wake = std::min(wake, next_send);
             continue;
           }
           next_send = now + period;
           wake = std::min(wake, next_send);
           if (paused)
             continue;
           if (!state)
             state = controller.snapshot(true);
           const auto frame =
               stem_module.build_state_frame(stem_proto::Msg::Telemetry, *state, transport);
           if (sender(transport)(frame)) {
             stall_start = {}; // queued OK -> the host is draining
           } else if (stall_start == TelemetryClock::time_point{}) {
             stall_start = now; // first drop -> start the stall clock
           } else if (now - stall_start > kTelemetryStallTimeout) {
             // The host abandoned the stream (a tab close does not unmount, so
             // the FIFO is not cleared for us): stop, and drop the backlog so a
             // reconnecting host reads a clean stream.
             stem_module.set_streaming(transport, false);
             stall_start = {};
             if (transport == Transport::Vendor)
               usb.vendor_write_clear();
             else
               usb.cdc_write_clear();
             logger.warn("Telemetry auto-paused on {}: the host stopped draining the endpoint",
                         name);
           }
         }
         {
           std::unique_lock<std::mutex> lock(m);
           cv.wait_until(lock, wake);
         }
         return false; // don't stop the task
       },
       // fmt-based rate-limited TX warnings need a few KB of stack
       .task_config = {.name = "stem_telemetry", .stack_size_bytes = 8192}});

  if (usb_ok) {
    rx_task.start();
    telemetry_task.start();
    logger.info("USB ready: WebUSB vendor + CDC frame streams (modules: OTA 0, core dump 4, "
                "STEM 7); open web/stem_console.html or https://{}",
                vendor.landing_page_url);
  }

  // With CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE an image booted right after an
  // OTA update is PENDING_VERIFY: CAN + USB brought up is this firmware's
  // health check, so mark it valid then, or the bootloader rolls back on the
  // next reset. An image that broke either subsystem is left PENDING (the
  // console still works for a look) and is rolled back by that next reset.
  if (ota.is_pending_verify()) {
    logger.warn("This image is PENDING VERIFY (first boot after an OTA update)");
    std::error_code ota_ec;
    if (!can_ok || !usb_ok) {
      logger.error("Self-check FAILED (CAN {}, USB {}): image left PENDING; the next reset "
                   "rolls back to the previous image",
                   can_ok ? "ok" : "FAILED", usb_ok ? "ok" : "FAILED");
    } else if (ota.mark_app_valid(ota_ec)) {
      logger.info("Self-check passed -> image marked VALID; rollback cancelled");
    } else {
      logger.error("Marking app valid failed ({}) -> rolling back", ota_ec.message());
      ota.mark_app_invalid_and_rollback(ota_ec); // reboots into the old image
    }
  }

  // --------------------------------------------------------------------------
  // Console CLI
  // --------------------------------------------------------------------------
  static auto cli = std::make_unique<cli::Cli>(
      make_cli_menu(controller, actuators, base_swivel, seat_swivel, can_bus));
  std::thread([cli_ptr = cli.get()] {
    espp::Cli input(*cli_ptr);
    input.Start();
  }).detach();
  logger.info("CLI ready: set <pair> <deg>, move <x> <y>, home, tilt <deg>, stop, get, status, "
              "release, reset_fault, zero <pair|all>, zero_align <pair|all>, save_zero, read_zero, "
              "load_zero, check_zero, ik <x> <y>, ik_ref <x> <y>, set_base <deg>, "
              "set_seat_swivel <deg>, get_base, get_seat_swivel, torque_limit <Nm>, "
              "torque_limit_permille <n>, "
              "set_node_id <old> <new>, can, can_recover");

  bool have_ethernet_status = false;
  bool last_ethernet_status = false;
  bool cdc_was_connected = false;
  while (true) {
    const bool ethernet_status = board.is_ethernet_connected();
    if (!have_ethernet_status || ethernet_status != last_ethernet_status) {
      logger.info("Ethernet connected: {}", ethernet_status);
      last_ethernet_status = ethernet_status;
      have_ethernet_status = true;
    }
    // A terminal attaching to the CDC port missed the boot output; re-log the
    // previous-crash summary once per connection.
    const bool cdc_connected = usb_ok && usb.is_cdc_connected();
    if (cdc_connected && !cdc_was_connected && !crash_report.empty()) {
      logger.error("Previous abnormal reset:\n{}", crash_report);
    }
    cdc_was_connected = cdc_connected;
    std::this_thread::sleep_for(1s);
  }
}
