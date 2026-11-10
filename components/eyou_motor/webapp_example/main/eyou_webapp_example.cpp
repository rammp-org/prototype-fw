// Eyou motor test-stand console: USB <-> one Eyou harmonic actuator on CAN.
//
// Runs on an ESP32-P4 with a CAN transceiver on the TWAI pins (the Waveshare
// ESP32-P4-ETH as used by the other projects in this repo: TX 17 / RX 16) and
// exposes ONE Eyou actuator (CANopen DS402, profile position mode) to a browser
// over USB: the console web app (../web/eyou_console.html) connects on the
// vendor (WebUSB) or CDC (Web Serial) interface, sets the torque limit the drive
// clamps to, commands absolute / relative moves, enables / disables / resets /
// halts the drive, sets, saves and loads the software zero, and plots the live
// angle, speed, torque and current. All CAN work happens here, inside
// EyouService (protocol: eyou_protocol.hpp, dispatcher module 0x20); the web app
// only sends requests and decodes the streamed STATUS snapshots.
//
// Like every espp USB example it also serves the standard system (7), monitor
// (8), OTA (0) and core-dump (4) services on both transports, so the hosted
// Device Hub, system / OTA / coredump consoles work against it. The system
// console and logs go to UART0.
//
// Safety: the drive is NOT enabled at boot, and the torque limit from
// menuconfig is written before anything can move; every move the console
// commands runs under whatever limit is set at that moment.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <span>
#include <string>
#include <thread>

#include "can_bus.hpp"
#include "coredump.hpp"
#include "coredump_service.hpp"
#include "dispatcher_worker.hpp"
#include "eyou_motor.hpp"
#include "eyou_service.hpp"
#include "logger.hpp"
#include "monitor_service.hpp"
#include "ota.hpp"
#include "ota_service.hpp"
#include "system_service.hpp"
#include "task.hpp"
#include "usb_device.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Eyou Console", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting the Eyou motor test-stand console");

  // --- Standard USB services' engines -----------------------------------------
  espp::CoreDump core_dump({.log_level = espp::Logger::Verbosity::INFO});
  const std::string crash_report = core_dump.format_report();
  if (crash_report.empty())
    logger.info("Clean boot history (reset reason: {})",
                espp::CoreDump::reset_reason_name(espp::CoreDump::reset_reason()));
  else
    logger.error("Previous abnormal reset:\n{}", crash_report);
  espp::Ota ota({.reject_same_version = false, .log_level = espp::Logger::Verbosity::INFO});
  if (ota.is_pending_verify())
    logger.warn("This image is PENDING VERIFY (first boot after an OTA update): it rolls back on "
                "the next reset unless the host confirms it (MARK_VALID from the OTA console)");

  // --- CAN + the motor -------------------------------------------------------
  // CanBus(rx_gpio, tx_gpio); the motor registers its CANopen receivers in its
  // constructor, which must happen before the bus starts.
  static constexpr gpio_num_t can_rx = static_cast<gpio_num_t>(CONFIG_EXAMPLE_CAN_RX_GPIO);
  static constexpr gpio_num_t can_tx = static_cast<gpio_num_t>(CONFIG_EXAMPLE_CAN_TX_GPIO);
  static constexpr uint8_t node_id = CONFIG_EXAMPLE_NODE_ID;
  CanBus bus(can_rx, can_tx);
  EyouMotor motor(bus, node_id);
  const bool can_ok = bus.start(CONFIG_EXAMPLE_CAN_BITRATE);
  if (!can_ok)
    logger.error("CAN bus did not start (TX {} / RX {} at {} bit/s): check the transceiver wiring",
                 (int)can_tx, (int)can_rx, CONFIG_EXAMPLE_CAN_BITRATE);
  else
    logger.info("CAN bus up (TX {} / RX {} at {} bit/s), Eyou node {}", (int)can_tx, (int)can_rx,
                CONFIG_EXAMPLE_CAN_BITRATE, node_id);

  // Commissioning at boot: the torque limit first (nothing moves harder than
  // this), then the profile, then the saved zero. The drive stays DISABLED
  // until the console enables it. Failures are not fatal: the drive may be off
  // or unplugged, the console shows "no communication" and every call is
  // retried when it is back.
  bool drive_seen = false;
  if (can_ok) {
    drive_seen = motor.set_max_torque(CONFIG_EXAMPLE_DEFAULT_MAX_TORQUE_PERMILLE);
    if (drive_seen) {
      motor.set_profile_position_config(
          {.velocity_degrees_per_second = static_cast<float>(CONFIG_EXAMPLE_PROFILE_VELOCITY_DPS),
           .acceleration_degrees_per_second_squared =
               static_cast<float>(CONFIG_EXAMPLE_PROFILE_ACCEL_DPS2),
           .deceleration_degrees_per_second_squared =
               static_cast<float>(CONFIG_EXAMPLE_PROFILE_DECEL_DPS2)});
#if CONFIG_EXAMPLE_LOAD_ZERO_AT_BOOT
      if (motor.load_zero())
        logger.info("Software zero loaded from NVS");
      else
        logger.warn("No saved software zero for node {}: angles are the encoder's absolute "
                    "position until one is set",
                    node_id);
#endif
    } else {
      logger.warn("The Eyou drive (node {}) did not answer: power / wiring / bitrate? The "
                  "console keeps trying.",
                  node_id);
    }
  }

  // --- USB: vendor (WebUSB) + CDC (Web Serial), both carry the protocol ------
  espp::UsbDevice::Config usb_cfg;
  usb_cfg.manufacturer = "rammp";
  usb_cfg.product = "Eyou Motor Console";
#if CONFIG_EXAMPLE_USB_PORT_FS_USBC
  usb_cfg.port = 0;   // the USB-OTG 1.1 full-speed controller ...
  usb_cfg.fs_phy = 0; // ... on the USB-C pads (GPIO 24 / 25) of the P4 dev kits
#elif CONFIG_EXAMPLE_USB_PORT_HIGH_SPEED
  usb_cfg.port = 1;
#endif
  usb_cfg.log_level = espp::Logger::Verbosity::WARN;
  espp::UsbDevice::VendorFunction vendor;
  vendor.interface_name = "Eyou Motor Console (WebUSB)";
  vendor.webusb = true;
  usb_cfg.vendor = vendor;
  espp::UsbDevice::CdcFunction cdc;
  cdc.interface_name = "Eyou Motor Console (CDC)";
  usb_cfg.cdc = cdc;
  espp::UsbDevice usb(usb_cfg);

  // One sender mutex per transport: UsbDevice's write_*() do not serialize
  // writers and each transport is fed from two tasks (the dispatcher worker's
  // replies and the poll task's stream).
  std::mutex tx_mutex;
  auto vendor_send = [&](std::span<const uint8_t> f) {
    std::lock_guard<std::mutex> lock(tx_mutex);
    if (!usb.write_vendor(f))
      logger.warn_rate_limited("dropped a {}-byte vendor frame (TX backpressure)", f.size());
  };
  auto cdc_send = [&](std::span<const uint8_t> f) {
    std::lock_guard<std::mutex> lock(tx_mutex);
    if (!usb.write_cdc(f))
      logger.warn_rate_limited("dropped a {}-byte CDC frame (TX backpressure)", f.size());
  };

  // The Eyou service, one per transport, sharing the motor and the target.
  EyouService::Shared shared;
  EyouService vendor_eyou(motor, shared,
                          {.send = vendor_send, .log_level = espp::Logger::Verbosity::INFO});
  EyouService cdc_eyou(motor, shared,
                       {.send = cdc_send, .log_level = espp::Logger::Verbosity::INFO});

  // The standard services (system 7 / monitor 8 / OTA 0 / core dump 4).
  auto reboot_request = [&](espp::SystemService::RebootKind kind) {
    logger.warn("Host requested a {}; allowing it",
                kind == espp::SystemService::RebootKind::Bootloader ? "reboot into the bootloader"
                                                                    : "reboot");
    return true;
  };
  espp::SystemService vendor_system({.send = vendor_send,
                                     .on_reboot_request = reboot_request,
                                     .log_level = espp::Logger::Verbosity::INFO});
  espp::SystemService cdc_system({.send = cdc_send,
                                  .on_reboot_request = reboot_request,
                                  .log_level = espp::Logger::Verbosity::INFO});
  espp::MonitorService vendor_monitor(
      {.send = vendor_send,
       .task_config = {.name = "monitor_v", .stack_size_bytes = 6 * 1024},
       .log_level = espp::Logger::Verbosity::INFO});
  espp::MonitorService cdc_monitor(
      {.send = cdc_send,
       .task_config = {.name = "monitor_c", .stack_size_bytes = 6 * 1024},
       .log_level = espp::Logger::Verbosity::INFO});
  espp::OtaService vendor_ota(ota,
                              {.send = vendor_send, .log_level = espp::Logger::Verbosity::INFO});
  espp::OtaService cdc_ota(ota, {.send = cdc_send, .log_level = espp::Logger::Verbosity::INFO});
  espp::CoreDumpService vendor_coredump(
      core_dump, {.send = vendor_send, .log_level = espp::Logger::Verbosity::INFO});
  espp::CoreDumpService cdc_coredump(
      core_dump, {.send = cdc_send, .log_level = espp::Logger::Verbosity::INFO});

  espp::DispatcherWorker vendor_link(
      {.send = vendor_send,
       .on_overflow = [&]() { vendor_ota.on_rx_overflow(); },
       .task_config = {.name = "eyou_vendor", .stack_size_bytes = 16384}});
  espp::DispatcherWorker cdc_link({.send = cdc_send,
                                   .on_overflow = [&]() { cdc_ota.on_rx_overflow(); },
                                   .task_config = {.name = "eyou_cdc", .stack_size_bytes = 16384}});
  vendor_link.register_module(vendor_eyou); // module 0x20 + its discovery metadata
  cdc_link.register_module(cdc_eyou);
  vendor_link.register_module(vendor_system);
  vendor_link.register_module(vendor_monitor);
  vendor_link.register_module(vendor_ota);
  vendor_link.register_module(vendor_coredump);
  cdc_link.register_module(cdc_system);
  cdc_link.register_module(cdc_monitor);
  cdc_link.register_module(cdc_ota);
  cdc_link.register_module(cdc_coredump);
  vendor_link.serve_discovery(usb_cfg.product);
  cdc_link.serve_discovery(usb_cfg.product);
  usb.set_vendor_receive_callback([&](std::span<const uint8_t> data) { vendor_link.push(data); });
  usb.set_cdc_receive_callback([&](std::span<const uint8_t> data) { cdc_link.push(data); });

  // --- the poll loop: one STATUS snapshot per period, fed to both services ---
  // Every period: position, velocity, torque, current, statusword (five SDO
  // reads, a few milliseconds on a 1 Mbit/s bus). About once a second, and
  // right after a command changed them: the torque limit, the error code, the
  // profile; the rated torque / current once they are read successfully.
  eyou_protocol::Status status;
  status.node_id = node_id;
  status.target_degrees = std::numeric_limits<float>::quiet_NaN();
  const auto t0 = std::chrono::steady_clock::now();
  uint32_t tick = 0;
  bool have_rated = false;
  auto last_report = std::chrono::steady_clock::now();
  static constexpr uint32_t kSlowEvery =
      std::max<uint32_t>(1, 1000 / std::max<int>(1, CONFIG_EXAMPLE_POLL_PERIOD_MS));
  espp::Task poll_task({
      .callback = [&](std::mutex &m, std::condition_variable &cv) -> bool {
        const auto start = std::chrono::steady_clock::now();
        if (can_ok) {
          float angle = 0.0f, velocity = 0.0f;
          int16_t torque = 0, current = 0;
          uint16_t statusword = 0;
          const bool ok = motor.get_position(angle, 50) && motor.get_velocity(velocity, 50) &&
                          motor.get_torque(torque, 50) && motor.get_current(current, 50) &&
                          motor.get_statusword(statusword, 50);
          status.comm_ok = ok;
          if (ok) {
            status.angle_degrees = angle;
            status.velocity_dps = velocity;
            status.torque_permille = torque;
            status.current_permille = current;
            status.statusword = statusword;
            status.enabled = (statusword & 0x006F) == 0x0027;
            status.faulted = (statusword & 0x0008) != 0;
            status.target_reached = (statusword & 0x0400) != 0;
            if (!drive_seen) {
              // the drive just appeared (powered on / plugged in after boot):
              // commission it the same way the boot did
              drive_seen = true;
              motor.set_max_torque(CONFIG_EXAMPLE_DEFAULT_MAX_TORQUE_PERMILLE);
              shared.settings_dirty.store(true);
            }
          } else {
            status.faulted = true; // fail-safe: no answer is shown as a fault
          }
          const bool slow = (tick % kSlowEvery) == 0 || shared.settings_dirty.exchange(false);
          if (ok && slow) {
            uint16_t limit = 0;
            if (motor.get_max_torque(limit, 50))
              status.torque_limit_permille = limit;
            uint16_t error_code = 0;
            if (motor.get_error_code(error_code, 50))
              status.error_code = error_code;
            int8_t mode = 0;
            if (motor.get_operating_mode(mode, 50))
              status.mode = mode;
            EyouMotor::ProfilePositionConfig profile{};
            if (motor.get_profile_position_config(profile, 50)) {
              status.profile_velocity_dps = profile.velocity_degrees_per_second;
              status.profile_accel_dps2 = profile.acceleration_degrees_per_second_squared;
              status.profile_decel_dps2 = profile.deceleration_degrees_per_second_squared;
            }
            if (!have_rated) {
              uint32_t rated_torque = 0, rated_current = 0;
              if (motor.get_rated_torque(rated_torque, 50) &&
                  motor.get_rated_current(rated_current, 50)) {
                status.rated_torque_mnm = rated_torque;
                status.rated_current_ma = rated_current;
                have_rated = true;
                logger.info("Rated torque {} mNm, rated current {} mA", rated_torque,
                            rated_current);
              }
            }
          }
        }
        // the component-side settings cost nothing to read
        const auto zero = motor.zero_offset_pulses();
        status.zero_set = zero.has_value();
        status.zero_offset_pulses = zero.value_or(0);
        status.pos_min_degrees = motor.minimum_position_degrees();
        status.pos_max_degrees = motor.maximum_position_degrees();
        status.timestamp_ms = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(start - t0).count());
        status.sequence = ++tick;
        vendor_eyou.on_status(status);
        cdc_eyou.on_status(status);

        const auto now = std::chrono::steady_clock::now();
        if (now - last_report >= 10s) {
          logger.info("{}: angle {:.2f} deg, {:.1f} deg/s, torque {} permille, {} ({}); stream "
                      "vendor {} / cdc {}",
                      status.comm_ok ? "drive" : "drive NOT answering", status.angle_degrees,
                      status.velocity_dps, status.torque_permille,
                      EyouMotor::ds402_state_name(status.statusword),
                      status.enabled ? "enabled" : "disabled",
                      vendor_eyou.stream_enabled() ? "on" : "off",
                      cdc_eyou.stream_enabled() ? "on" : "off");
          last_report = now;
        }
        // keep the period, whatever the five reads took (up to 250 ms when the
        // drive is away: each read times out at 50 ms)
        std::unique_lock<std::mutex> lock(m);
        const auto period = std::chrono::milliseconds(CONFIG_EXAMPLE_POLL_PERIOD_MS);
        const auto elapsed = std::chrono::steady_clock::now() - start;
        cv.wait_for(lock, elapsed < period ? period - elapsed : 1ms);
        return false; // keep polling
      },
      .task_config = {.name = "eyou poll", .stack_size_bytes = 8 * 1024},
      .log_level = espp::Logger::Verbosity::WARN,
  });
  poll_task.start();

  std::error_code usb_ec;
  if (!usb.initialize(usb_ec))
    logger.error("Failed to initialize USB device: {} - no host transport available",
                 usb_ec.message());
  else
    logger.info("Eyou motor console ready. Connect the web app over WebUSB / Web Serial.");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
