// PEAR linkage console: USB <-> one 5-bar linkage (two Eyou actuators on CAN).
//
// Runs on the Waveshare ESP32-P4-ETH (CAN transceiver on TX 17 / RX 16; the
// chip's USB 2.0 HS signals on the 4-pin MX1.25 "USB" connector; the Type-C is
// the CH343 UART console) and exposes ONE 5-bar linkage -- two Eyou harmonic
// actuators (CANopen DS402, profile position), one per crank -- to a browser:
// the console web app (../web/linkage_console.html) configures the geometry
// (link lengths, assembly), each motor (node id, direction, limits, crank angle
// at zero, torque limit) and the profile, moves the end effector (inverse
// kinematics on the device), drives the cranks directly, enables / disables /
// resets / halts the drives, sets and saves the motors' software zeros, and
// draws the live pose (forward kinematics of the measured crank angles). The
// settings persist in NVS (the SETTINGS payload is the record).
//
// Like every espp USB example it also serves the standard system (7), monitor
// (8), OTA (0) and core-dump (4) services on both transports.
//
// Safety: the drives are NOT enabled at boot; each motor's torque limit is
// written before anything can move; moves whose motor angles fall outside the
// configured limits are refused, not clamped.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include "nvs.h"
#include "nvs_flash.h"

#include "can_bus.hpp"
#include "coredump.hpp"
#include "coredump_service.hpp"
#include "dispatcher_worker.hpp"
#include "eyou_motor.hpp"
#include "linkage_controller.hpp"
#include "linkage_service.hpp"
#include "logger.hpp"
#include "monitor_service.hpp"
#include "nvs.hpp"
#include "ota.hpp"
#include "ota_service.hpp"
#include "system_service.hpp"
#include "task.hpp"
#include "usb_device.hpp"

using namespace std::chrono_literals;

namespace {

// pid.codes VID 0x1209 (espp default) with a PID distinct from the espp
// examples and the other projects here (STEM 0x0d3a); consoles filter by VID.
constexpr uint16_t kUsbPid = 0x0d3c;
constexpr char kNvsNamespace[] = "pear_linkage";
constexpr char kNvsSettingsKey[] = "settings";

using Settings = linkage_protocol::Settings;

/// The firmware defaults: the Kconfig node ids, otherwise the protocol's.
Settings default_settings() {
  Settings s;
  s.motors[0].node_id = CONFIG_LINKAGE_MOTOR1_NODE_ID;
  s.motors[1].node_id = CONFIG_LINKAGE_MOTOR2_NODE_ID;
  for (auto &m : s.motors)
    m.torque_limit_permille = std::min<uint16_t>(CONFIG_LINKAGE_DEFAULT_TORQUE_PERMILLE,
                                                 CONFIG_LINKAGE_MAX_TORQUE_PERMILLE);
  return s;
}

bool load_settings_blob(espp::Logger &logger, Settings &settings) {
  nvs_handle_t handle = 0;
  if (nvs_open(kNvsNamespace, NVS_READONLY, &handle) != ESP_OK)
    return false;
  size_t size = 0;
  bool ok = false;
  if (nvs_get_blob(handle, kNvsSettingsKey, nullptr, &size) == ESP_OK && size > 0) {
    std::vector<uint8_t> blob(size);
    if (nvs_get_blob(handle, kNvsSettingsKey, blob.data(), &size) == ESP_OK) {
      const auto parsed = Settings::parse(blob);
      if (parsed) {
        settings = *parsed;
        ok = true;
      } else {
        logger.warn("Saved settings ({} bytes) are of another version: using the defaults", size);
      }
    }
  }
  nvs_close(handle);
  return ok;
}

bool save_settings_blob(espp::Logger &logger, const Settings &settings) {
  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    logger.error("NVS open failed: {}", esp_err_to_name(err));
    return false;
  }
  const auto blob = settings.serialize();
  err = nvs_set_blob(handle, kNvsSettingsKey, blob.data(), blob.size());
  if (err == ESP_OK)
    err = nvs_commit(handle);
  nvs_close(handle);
  if (err != ESP_OK) {
    logger.error("Saving the settings failed: {}", esp_err_to_name(err));
    return false;
  }
  logger.info("Settings saved ({} bytes)", blob.size());
  return true;
}

} // namespace

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "PEAR Linkage", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting the PEAR linkage console");

  // --- Standard USB services' engines -----------------------------------------
  espp::CoreDump core_dump({.log_level = espp::Logger::Verbosity::INFO});
  const std::string crash_report = core_dump.format_report();
  if (crash_report.empty())
    logger.info("Clean boot history (reset reason: {})",
                espp::CoreDump::reset_reason_name(espp::CoreDump::reset_reason()));
  else
    logger.error("Previous abnormal reset:\n{}", crash_report);
  espp::Ota ota({.reject_same_version = false, .log_level = espp::Logger::Verbosity::INFO});
  const auto running = ota.running_app_description();
  if (ota.is_pending_verify())
    logger.warn("This image is PENDING VERIFY (first boot after an OTA update): it rolls back on "
                "the next reset unless the host confirms it (MARK_VALID from the OTA console)");

  // --- Settings (NVS) ---------------------------------------------------------
  {
    std::error_code ec;
    espp::Nvs nvs; // nvs_flash_init (idempotent); EyouMotor's zeros use it too
    nvs.init(ec);
    if (ec)
      logger.error("NVS init failed: {}", ec.message());
  }
  const Settings defaults = default_settings();
  Settings settings = defaults;
  if (load_settings_blob(logger, settings))
    logger.info("Settings loaded from NVS: motors {} / {}", settings.motors[0].node_id,
                settings.motors[1].node_id);
  else
    logger.info("No saved settings: using the defaults (motors {} / {})",
                defaults.motors[0].node_id, defaults.motors[1].node_id);
  {
    std::string why;
    if (!settings.validate(CONFIG_LINKAGE_MAX_TORQUE_PERMILLE, why)) {
      logger.error("Saved settings are invalid ({}): using the defaults", why);
      settings = defaults;
    }
  }

  // --- CAN + the two motors ---------------------------------------------------
  static constexpr gpio_num_t can_rx = static_cast<gpio_num_t>(CONFIG_LINKAGE_CAN_RX_GPIO);
  static constexpr gpio_num_t can_tx = static_cast<gpio_num_t>(CONFIG_LINKAGE_CAN_TX_GPIO);
  CanBus bus(can_rx, can_tx); // CanBus(rx_gpio, tx_gpio)
  // node id 0 (no motor) still needs an object: it binds to node 0 (unused)
  EyouMotor motor1(bus, settings.motors[0].node_id);
  EyouMotor motor2(bus, settings.motors[1].node_id);
  const bool can_ok = bus.start(CONFIG_LINKAGE_CAN_BITRATE);
  if (!can_ok)
    logger.error("CAN bus did not start (TX {} / RX {} at {} bit/s): check the transceiver wiring",
                 (int)can_tx, (int)can_rx, CONFIG_LINKAGE_CAN_BITRATE);
  else
    logger.info("CAN bus up (TX {} / RX {} at {} bit/s)", (int)can_tx, (int)can_rx,
                CONFIG_LINKAGE_CAN_BITRATE);

  LinkageController controller({.motors = {&motor1, &motor2},
                                .max_torque_permille = CONFIG_LINKAGE_MAX_TORQUE_PERMILLE,
                                .can_ok = can_ok,
                                .log_level = espp::Logger::Verbosity::INFO});
  {
    // commissions the motors that answer (torque limit first, profile, limits,
    // saved zero); the drives stay DISABLED until the console enables them
    std::string why;
    if (!controller.apply_settings(settings, why))
      logger.error("Could not apply the settings: {}", why);
  }

  // --- USB: vendor (WebUSB) + CDC (Web Serial), both carry the protocol ------
  espp::UsbDevice::Config usb_cfg;
  usb_cfg.pid = kUsbPid;
  usb_cfg.manufacturer = "rammp";
  usb_cfg.product = "PEAR Linkage Console";
#if CONFIG_LINKAGE_USB_PORT_HIGH_SPEED
  usb_cfg.port = 1; // USB 2.0 HS: the ESP32-P4-ETH's 4-pin MX1.25 "USB" connector
#elif CONFIG_LINKAGE_USB_PORT_FS_USBC
  usb_cfg.port = 0;   // the USB-OTG 1.1 full-speed controller ...
  usb_cfg.fs_phy = 0; // ... on the USB-C pads (GPIO 24 / 25) of other P4 dev kits
#endif
  usb_cfg.log_level = espp::Logger::Verbosity::WARN;
  espp::UsbDevice::VendorFunction vendor;
  vendor.interface_name = "PEAR Linkage Console (WebUSB)";
  vendor.webusb = true;
  vendor.landing_page_url = "esp-cpp.github.io/espp/apps/dispatcher_hub.html";
  usb_cfg.vendor = vendor;
  espp::UsbDevice::CdcFunction cdc;
  cdc.interface_name = "PEAR Linkage Console (CDC)";
  usb_cfg.cdc = cdc;
  espp::UsbDevice usb(usb_cfg);

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

  // The linkage service, one per transport, sharing the controller.
  auto on_settings_changed = [&](const Settings &s) { save_settings_blob(logger, s); };
  const linkage_protocol::Info info{.max_torque_permille = CONFIG_LINKAGE_MAX_TORQUE_PERMILLE,
                                    .poll_period_ms = CONFIG_LINKAGE_POLL_PERIOD_MS,
                                    .project = running.project_name,
                                    .version = running.version,
                                    .build = running.date + " " + running.time};
  LinkageService vendor_linkage(controller, {.send = vendor_send,
                                             .info = info,
                                             .defaults = defaults,
                                             .on_settings_changed = on_settings_changed,
                                             .log_level = espp::Logger::Verbosity::INFO});
  LinkageService cdc_linkage(controller, {.send = cdc_send,
                                          .info = info,
                                          .defaults = defaults,
                                          .on_settings_changed = on_settings_changed,
                                          .log_level = espp::Logger::Verbosity::INFO});

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
       .task_config = {.name = "link_vendor", .stack_size_bytes = 16384}});
  espp::DispatcherWorker cdc_link({.send = cdc_send,
                                   .on_overflow = [&]() { cdc_ota.on_rx_overflow(); },
                                   .task_config = {.name = "link_cdc", .stack_size_bytes = 16384}});
  vendor_link.register_module(vendor_linkage); // module 0x21 + its discovery metadata
  cdc_link.register_module(cdc_linkage);
  vendor_link.register_module(vendor_system);
  vendor_link.register_module(vendor_monitor);
  vendor_link.register_module(vendor_ota);
  vendor_link.register_module(vendor_coredump);
  cdc_link.register_module(cdc_system);
  cdc_link.register_module(cdc_monitor);
  cdc_link.register_module(cdc_ota);
  cdc_link.register_module(cdc_coredump);
  vendor_link.serve_discovery(usb_cfg.product, running.version);
  cdc_link.serve_discovery(usb_cfg.product, running.version);
  usb.set_vendor_receive_callback([&](std::span<const uint8_t> data) { vendor_link.push(data); });
  usb.set_cdc_receive_callback([&](std::span<const uint8_t> data) { cdc_link.push(data); });

  // --- the poll loop: one STATUS snapshot per period, fed to both services ---
  linkage_protocol::Status status;
  const auto t0 = std::chrono::steady_clock::now();
  uint32_t seq = 0;
  auto last_report = std::chrono::steady_clock::now();
  espp::Task poll_task({
      .callback = [&](std::mutex &m, std::condition_variable &cv) -> bool {
        const auto start = std::chrono::steady_clock::now();
        controller.poll(status);
        status.timestamp_ms = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(start - t0).count());
        status.sequence = ++seq;
        vendor_linkage.on_status(status);
        cdc_linkage.on_status(status);
        const auto now = std::chrono::steady_clock::now();
        if (now - last_report >= 10s) {
          logger.info("pose {} ({:.1f}, {:.1f}) mm, cranks {:.1f} / {:.1f} deg; motors {} / {}; "
                      "stream vendor {} / cdc {}",
                      status.pose_valid ? "valid" : "INVALID", status.x, status.y,
                      status.theta1_deg, status.theta2_deg,
                      status.motors[0].comm_ok ? (status.motors[0].enabled ? "enabled" : "idle")
                                               : "NO ANSWER",
                      status.motors[1].comm_ok ? (status.motors[1].enabled ? "enabled" : "idle")
                                               : "NO ANSWER",
                      vendor_linkage.stream_enabled() ? "on" : "off",
                      cdc_linkage.stream_enabled() ? "on" : "off");
          last_report = now;
        }
        // keep the period whatever the reads took (two motors away = up to
        // 500 ms of timeouts: the period then stretches, which is fine)
        std::unique_lock<std::mutex> lock(m);
        const auto period = std::chrono::milliseconds(CONFIG_LINKAGE_POLL_PERIOD_MS);
        const auto elapsed = std::chrono::steady_clock::now() - start;
        cv.wait_for(lock, elapsed < period ? period - elapsed : 1ms);
        return false;
      },
      .task_config = {.name = "linkage poll", .stack_size_bytes = 8 * 1024},
      .log_level = espp::Logger::Verbosity::WARN,
  });
  poll_task.start();

  std::error_code usb_ec;
  if (!usb.initialize(usb_ec))
    logger.error("Failed to initialize USB device: {} - no host transport available",
                 usb_ec.message());
  else
    logger.info("PEAR linkage console ready. Connect the web app over WebUSB / Web Serial.");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
