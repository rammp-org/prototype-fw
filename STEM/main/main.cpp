// STEM firmware: drives the 5-bar seat linkage (three reversed RMD actuator
// pairs over CAN) and exposes it over native USB as an espp "dispatcher"
// device. The USB vendor (WebUSB) and CDC (Web Serial) streams each carry
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

#include <array>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "cli.hpp"
#include "coredump.hpp"
#include "coredump_service.hpp"
#include "detail/ota_stream_protocol.hpp"
#include "dispatcher.hpp"
#include "esp32-p4-eth.hpp"
#include "ik_5bar.hpp"
#include "logger.hpp"
#include "motor_actuator.hpp"
#include "ota.hpp"
#include "paired_actuator.hpp"
#include "stem_controller.hpp"
#include "stem_module.hpp"
#include "stream_frame.hpp"
#include "task.hpp"
#include "usb_device.hpp"

using namespace std::chrono_literals;

namespace {

// pid.codes VID 0x1209 (espp default) with a PID distinct from the espp examples
// (0x0d32 ota, 0x0d34 haptics, 0x0d36 coredump); the web consoles filter by VID.
constexpr uint16_t kUsbPid = 0x0d3a;
constexpr gpio_num_t kCanTxGpio = GPIO_NUM_16;
constexpr gpio_num_t kCanRxGpio = GPIO_NUM_17;

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

// Console CLI: the same operations the USB module exposes, on top of the same
// controller (so limits / rejections / tracked state agree).
std::unique_ptr<cli::Menu> make_cli_menu(stem::StemController &controller,
                                         std::span<MotorActuator> actuators) {
  auto menu = std::make_unique<cli::Menu>("STEM");
  menu->Insert(
      "set",
      [&controller](std::ostream &out, const std::string &pair_name, float degrees) {
        const auto pair = stem::pair_from_name(pair_name);
        if (!pair) {
          out << "Unknown pair '" << pair_name << "'. Use right, left, or seat.\n";
          return;
        }
        const auto result = controller.set_pair(*pair, degrees, 0.0f);
        out << pair_name << " -> " << degrees << " deg: " << stem::StemController::to_string(result)
            << "\n";
      },
      "Set one pair position at the default set speed: set <right|left|seat> <degrees>");
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
        out << "pairs: left=" << pose.pair_deg[0] << " right=" << pose.pair_deg[1]
            << " seat=" << pose.pair_deg[2] << " deg" << (pose.within_limits ? "" : " (OUTSIDE LIMITS)")
            << "\n";
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
          out << " (left=" << pose.pair_deg[0] << " right=" << pose.pair_deg[1]
              << " seat=" << pose.pair_deg[2] << " deg)";
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
          MotorActuator::Status status{};
          if (actuator.read_status(status)) {
            out << "Motor " << static_cast<int>(actuator.get_motor_id())
                << ": temperature=" << status.temperature_c << " C, torque=" << status.torque_raw
                << ", velocity=" << status.velocity_rpm << " RPM, tracked=" << actuator.get_position()
                << " deg\n";
          } else {
            out << "Motor " << static_cast<int>(actuator.get_motor_id())
                << ": status read failed\n";
          }
        }
      },
      "Get status for all motors: status");
  menu->Insert(
      "release",
      [&controller](std::ostream &out) {
        out << (controller.release_brakes() ? "All motor brakes released.\n"
                                            : "Failed to release some brakes.\n");
      },
      "Release brakes for all actuator pairs: release");
  menu->Insert(
      "zero",
      [&controller](std::ostream &out, const std::string &pair_name) {
        const auto pair = stem::pair_from_name(pair_name);
        if (!pair && pair_name != "all") {
          out << "Unknown pair '" << pair_name << "'. Use right, left, seat, or all.\n";
          return;
        }
        controller.zero(pair, false);
        out << pair_name << " zeroed without alignment.\n";
      },
      "Set a pair's (or all pairs') current position as zero without alignment: zero <right|left|seat|all>");
  menu->Insert(
      "zero_align",
      [&controller](std::ostream &out, const std::string &pair_name) {
        const auto pair = stem::pair_from_name(pair_name);
        if (!pair && pair_name != "all") {
          out << "Unknown pair '" << pair_name << "'. Use right, left, seat, or all.\n";
          return;
        }
        controller.zero(pair, true);
        out << pair_name << " zeroed after alignment.\n";
      },
      "Set a pair's (or all pairs') current position as zero after alignment: zero_align <right|left|seat|all>");
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

  if (!board.initialize_ethernet()) {
    logger.error("Failed to initialize Ethernet");
  } else {
    logger.info("Ethernet initialized");
  }

  // --------------------------------------------------------------------------
  // CAN bus, actuators, pairs, controller
  // --------------------------------------------------------------------------
  MotorCanBus can_bus(kCanTxGpio, kCanRxGpio);
  const bool can_ok = can_bus.start();
  if (!can_ok) {
    // Keep going: USB / OTA / core-dump access must not depend on the motors.
    logger.error("CAN bus failed to start; motor commands will fail until reboot");
  }
  MotorActuator::CommunicationFunction communicate =
      [&can_bus](const MotorPacket &command, MotorPacket &response, uint32_t timeout_ms) {
        if (timeout_ms == 0)
          return can_bus.send(command);
        return can_bus.request(command, response, timeout_ms);
      };
  std::array<MotorActuator, stem::kMotorCount> actuators{{
      MotorActuator(communicate, 1),
      MotorActuator(communicate, 2),
      MotorActuator(communicate, 3),
      MotorActuator(communicate, 4),
      MotorActuator(communicate, 5),
      MotorActuator(communicate, 6),
  }};
  // Reversed pairs (front / back plane of each axis): 1-2 left, 3-4 right, 5-6 seat
  PairedActuator right_pair(actuators[3], actuators[2]);
  PairedActuator left_pair(actuators[1], actuators[0]);
  PairedActuator seat_pair(actuators[5], actuators[4]);
  logger.info("Six motor actuators (IDs 1-6) in three reversed pairs: left 2-1, right 4-3, seat 6-5");

  stem::StemController controller({
      .left = left_pair,
      .right = right_pair,
      .seat = seat_pair,
      .motors = actuators,
      .can_ok = can_ok,
      .log_level = espp::Logger::Verbosity::INFO,
  });

  if (can_ok) {
    controller.zero(std::nullopt, false);
    for (auto &actuator : actuators) {
      MotorActuator::Status status{};
      if (actuator.read_status(status)) {
        logger.info("Motor {} status: temperature={} C torque_raw={} velocity={} RPM",
                    actuator.get_motor_id(), status.temperature_c, status.torque_raw,
                    status.velocity_rpm);
      } else {
        logger.warn("Motor {} status read failed", actuator.get_motor_id());
      }
    }
  }

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
  espp::CoreDumpService cdc_coredump(
      core_dump, {.send = [&](std::span<const uint8_t> f) { send_cdc(f); },
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
    dispatcher.register_module(
        otap::kModule,
        [&, send](const Frame &frame) {
          if (!frame.is_reply())
            handle_ota_frame(frame, send);
        },
        {.name = "OTA", .app = "ota_console.html", .description = "Firmware update over USB"});
    dispatcher.register_module(
        espp::CoreDumpService::kModule,
        [&coredump_service](const Frame &frame) {
          if (!frame.is_reply())
            coredump_service.handle_frame(frame.type, frame.payload);
        },
        {.name = "Core Dump",
         .app = "coredump_console.html",
         .description = "Inspect the last crash core dump"});
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
  // Telemetry streaming (per stream, device-side period)
  // --------------------------------------------------------------------------
  std::array<std::chrono::steady_clock::time_point, stem::StemModule::kTransportCount>
      telemetry_stall_start{};
  espp::Task telemetry_task(
      {.callback = [&](std::mutex &m, std::condition_variable &cv) -> bool {
         const auto start = std::chrono::steady_clock::now();
         // Pause while an OTA transfer runs so the bulk pipes carry only the
         // flow-controlled OTA replies.
         if (stem_module.any_streaming() && !ota.session_active()) {
           const auto state = controller.snapshot(true);
           for (size_t i = 0; i < stem::StemModule::kTransportCount; ++i) {
             const auto transport = static_cast<Transport>(i);
             const bool connected = transport == Transport::Vendor ? usb.is_vendor_connected()
                                                                   : usb.is_cdc_connected();
             if (!stem_module.streaming(transport) || !connected)
               continue;
             const auto frame =
                 stem_module.build_state_frame(stem_proto::Msg::Telemetry, state, transport);
             auto &stall_start = telemetry_stall_start[i];
             if (sender(transport)(frame)) {
               stall_start = {}; // queued OK -> the host is draining
             } else if (stall_start == std::chrono::steady_clock::time_point{}) {
               stall_start = start; // first drop -> start the stall clock
             } else if (start - stall_start > kTelemetryStallTimeout) {
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
                           transport == Transport::Vendor ? "WebUSB" : "CDC");
             }
           }
         }
         {
           std::unique_lock<std::mutex> lock(m);
           cv.wait_until(lock, start + std::chrono::milliseconds(stem_module.stream_period_ms()));
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
  // OTA update is PENDING_VERIFY: getting this far (CAN + USB brought up) is
  // this firmware's health check, so mark it valid or the bootloader rolls
  // back on the next reset.
  if (ota.is_pending_verify()) {
    logger.warn("This image is PENDING VERIFY (first boot after an OTA update)");
    std::error_code ota_ec;
    if (ota.mark_app_valid(ota_ec)) {
      logger.info("Self-check passed -> image marked VALID; rollback cancelled");
    } else {
      logger.error("Marking app valid failed ({}) -> rolling back", ota_ec.message());
      ota.mark_app_invalid_and_rollback(ota_ec); // reboots into the old image
    }
  }

  // --------------------------------------------------------------------------
  // Console CLI
  // --------------------------------------------------------------------------
  static auto cli = std::make_unique<cli::Cli>(make_cli_menu(controller, actuators));
  std::thread([cli_ptr = cli.get()] {
    espp::Cli input(*cli_ptr);
    input.Start();
  }).detach();
  logger.info("CLI ready: set <pair> <deg>, move <x> <y>, home, stop, get, status, release, "
              "zero <pair|all>, zero_align <pair|all>, ik <x> <y>, ik_ref <x> <y>");

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
