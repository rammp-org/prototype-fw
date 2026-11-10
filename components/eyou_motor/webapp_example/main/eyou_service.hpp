#pragma once

// EyouService: the Eyou motor console protocol (eyou_protocol.hpp) as a
// dispatcher service around one EyouMotor on the device's CAN bus. Register it
// on an espp::Dispatcher / DispatcherWorker and the console web app can drive
// the motor and watch it live.
//
//   EyouMotor motor(bus, node_id);
//   EyouService::Shared shared;      // what every transport's instance shows alike
//   EyouService service(motor, shared, {.send = send});
//   link.register_module(service);
//   // the application's poll task, every period:
//   service.on_status(status);
//
// The service does NOT poll the drive itself: the application owns the one
// poll loop (one motor, however many transports serve it) and hands each
// snapshot to every service instance through on_status(), which forwards it on
// the transport whose host asked for the stream, at that host's period, and
// keeps it for GET_STATUS. Commands run the EyouMotor calls (CANopen SDO
// transactions, tens of milliseconds) on the dispatcher worker that feeds
// handle(); EyouMotor serializes its own transactions, so the poll loop and a
// command interleave safely at transaction granularity.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "base_component.hpp"
#include "dispatcher.hpp"
#include "eyou_motor.hpp"
#include "eyou_protocol.hpp"
#include "stream_frame.hpp"

class EyouService : public espp::BaseComponent {
public:
  using Request = eyou_protocol::Request;
  using Reply = eyou_protocol::Reply;
  using Status = eyou_protocol::Status;
  using send_fn = std::function<void(std::span<const uint8_t> frame)>;

  /// State every transport's service instance shows alike, owned by the
  /// application: the last commanded target and a flag telling the poll loop
  /// to read the profile / limit objects back after a command changed them.
  struct Shared {
    std::atomic<float> target_degrees{std::numeric_limits<float>::quiet_NaN()};
    std::atomic<bool> settings_dirty{true};
  };

  struct Config {
    /// Transmits an encoded frame (required). Called from the task that feeds
    /// handle() (request replies) AND from the task that calls on_status()
    /// (the stream), so it must be thread-safe.
    send_fn send{nullptr};
    /// Dispatcher module id to answer on (a routing key; the console finds it
    /// through discovery by eyou_protocol::kProtocol).
    uint8_t module{eyou_protocol::kModuleId};
    /// Stream period when the host asks for 0.
    uint16_t default_stream_period_ms{100};
    /// Ceiling for SetTorqueLimit, per-thousand of the rated torque: a request
    /// above it is refused (the device's configuration bounds what a host may
    /// ask for). Reported in STATUS so the console can size its control.
    uint16_t max_torque_permille{1000};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN};
  };

  EyouService(EyouMotor &motor, Shared &shared, const Config &config)
      : espp::BaseComponent("EyouService", config.log_level)
      , motor_(motor)
      , shared_(shared)
      , config_(config)
      , stream_period_ms_(config.default_stream_period_ms ? config.default_stream_period_ms : 100) {
  }

  EyouService(const EyouService &) = delete;
  EyouService &operator=(const EyouService &) = delete;

  uint8_t module_id() const { return config_.module; }

  espp::Dispatcher::ModuleInfo module_info() const {
    return {.name = "Eyou motor",
            .app = "eyou_console.html",
            .description = "Drive one Eyou harmonic actuator over CAN: torque limit, moves, zero, "
                           "live readings",
            .protocol = eyou_protocol::kProtocol,
            .protocol_version = eyou_protocol::kProtocolVersion};
  }

  /// Dispatcher entry point.
  void handle(const espp::stream_frame::Frame &frame) {
    if (frame.module != module_id() || frame.is_reply())
      return;
    handle_request(frame.type, frame.payload);
  }

  /// Hand the service the snapshot the poll loop just built (every period,
  /// whether or not a host streams). Must be called WITHOUT the motor's
  /// transactions in flight on this task (it only transmits).
  void on_status(const Status &status) {
    const auto now = std::chrono::steady_clock::now();
    std::vector<uint8_t> payload;
    {
      std::lock_guard<std::mutex> lock(latest_mutex_);
      latest_ = status;
      have_latest_ = true;
      if (!stream_enabled_.load())
        return;
      const auto period = std::chrono::milliseconds(stream_period_ms_.load());
      if (last_sent_.time_since_epoch().count() != 0 && now - last_sent_ < period)
        return;
      last_sent_ = now;
      payload = decorate(latest_).serialize();
    }
    send(Reply::Status, payload);
  }

  bool stream_enabled() const { return stream_enabled_.load(); }

protected:
  /// The snapshot with this transport's stream fields filled in.
  Status decorate(Status s) const {
    s.stream_enabled = stream_enabled_.load();
    s.stream_period_ms = stream_period_ms_.load();
    s.target_degrees = shared_.target_degrees.load();
    s.torque_limit_max_permille = config_.max_torque_permille;
    return s;
  }

  void send_status() {
    std::vector<uint8_t> payload;
    {
      std::lock_guard<std::mutex> lock(latest_mutex_);
      if (have_latest_)
        payload = decorate(latest_).serialize();
    }
    if (payload.empty())
      send_error(static_cast<uint8_t>(Request::GetStatus), std::errc::no_message_available,
                 "no poll of the drive yet");
    else
      send(Reply::Status, payload);
  }

  void handle_request(uint8_t type, std::span<const uint8_t> payload) {
    namespace proto = eyou_protocol;
    switch (static_cast<Request>(type)) {
    case Request::GetStatus:
      send_status();
      return;
    case Request::SetStream: {
      const auto req = proto::SetStream::parse(payload);
      if (!req) {
        send_error(type, std::errc::invalid_argument, "short payload");
        return;
      }
      stream_period_ms_.store(req->period_ms ? req->period_ms : config_.default_stream_period_ms);
      stream_enabled_.store(req->enabled);
      logger_.info("status stream {} (every {} ms)", req->enabled ? "enabled" : "disabled",
                   stream_period_ms_.load());
      send_ok(type);
      return;
    }
    default:
      break;
    }

    // Everything else is a motor call (CANopen transactions, blocking for tens
    // of milliseconds; fine on the dispatcher worker).
    bool ok = false;
    std::string context;
    auto malformed = [&](const char *what) { send_error(type, std::errc::invalid_argument, what); };
    switch (static_cast<Request>(type)) {
    case Request::SetTorqueLimit: {
      const auto req = proto::SetTorqueLimit::parse(payload);
      if (!req)
        return malformed("short payload");
      if (req->permille > config_.max_torque_permille) {
        logger_.warn("refusing torque limit {} permille: the device's maximum is {}", req->permille,
                     config_.max_torque_permille);
        send_error(type, std::errc::result_out_of_range,
                   "torque limit " + std::to_string(req->permille) +
                       " permille exceeds the device's maximum of " +
                       std::to_string(config_.max_torque_permille) + " permille");
        return;
      }
      ok = motor_.set_max_torque(req->permille);
      context = "set torque limit";
      shared_.settings_dirty.store(true);
      break;
    }
    case Request::MoveTo: {
      const auto req = proto::Move::parse(payload);
      if (!req)
        return malformed("short payload");
      if (!std::isfinite(req->degrees))
        return malformed("target must be finite");
      ok = motor_.move_absolute(req->degrees, req->immediate);
      if (ok)
        shared_.target_degrees.store(std::clamp(req->degrees, motor_.minimum_position_degrees(),
                                                motor_.maximum_position_degrees()));
      context = "move to";
      break;
    }
    case Request::MoveBy: {
      const auto req = proto::Move::parse(payload);
      if (!req)
        return malformed("short payload");
      if (!std::isfinite(req->degrees))
        return malformed("increment must be finite");
      ok = motor_.move_incremental(req->degrees, req->immediate);
      if (ok) {
        // the target is where the move ends: the last target plus the step
        // when one is known, else the position plus the step
        float base = shared_.target_degrees.load();
        if (!std::isfinite(base)) {
          float position = 0.0f;
          base = motor_.get_position(position) ? position : 0.0f;
        }
        shared_.target_degrees.store(base + req->degrees);
      }
      context = "move by";
      break;
    }
    case Request::Enable:
      ok = motor_.enable_drive();
      context = "enable";
      break;
    case Request::Disable:
      ok = motor_.disable_drive();
      context = "disable";
      break;
    case Request::ResetFault:
      ok = motor_.reset_fault();
      context = "reset fault";
      break;
    case Request::Halt:
      ok = motor_.halt();
      if (ok)
        shared_.target_degrees.store(std::numeric_limits<float>::quiet_NaN());
      context = "halt";
      break;
    case Request::SetZero: {
      const auto req = proto::SetZero::parse(payload);
      if (!req)
        return malformed("short payload");
      if (req->mode == proto::SetZero::kModeHere) {
        ok = motor_.zero();
      } else if (req->mode == proto::SetZero::kModeAtAngle) {
        if (!std::isfinite(req->angle_degrees))
          return malformed("angle must be finite");
        ok = motor_.zero_at(req->angle_degrees);
      } else {
        return malformed("zero mode must be 0 (here) or 1 (at angle)");
      }
      // the target was relative to the old zero
      shared_.target_degrees.store(std::numeric_limits<float>::quiet_NaN());
      context = "set zero";
      break;
    }
    case Request::SaveZero:
      ok = motor_.save_zero();
      context = "save zero";
      break;
    case Request::LoadZero:
      ok = motor_.load_zero();
      shared_.target_degrees.store(std::numeric_limits<float>::quiet_NaN());
      context = "load zero";
      break;
    case Request::SetProfile: {
      const auto req = proto::SetProfile::parse(payload);
      if (!req)
        return malformed("short payload");
      if (!(req->velocity_dps > 0.0f) || !(req->acceleration_dps2 > 0.0f) ||
          !(req->deceleration_dps2 > 0.0f))
        return malformed("velocity, acceleration and deceleration must be positive");
      ok = motor_.set_profile_position_config(
          {.velocity_degrees_per_second = req->velocity_dps,
           .acceleration_degrees_per_second_squared = req->acceleration_dps2,
           .deceleration_degrees_per_second_squared = req->deceleration_dps2});
      shared_.settings_dirty.store(true);
      context = "set profile";
      break;
    }
    case Request::SetPositionLimits: {
      const auto req = proto::SetPositionLimits::parse(payload);
      if (!req)
        return malformed("short payload");
      ok = motor_.set_position_limits(req->min_degrees, req->max_degrees);
      shared_.settings_dirty.store(true);
      context = "set position limits";
      break;
    }
    default:
      send_error(type, std::errc::not_supported, "unknown Eyou message");
      return;
    }
    if (ok)
      send_ok(type);
    else
      send_error(type, std::errc::io_error, context + " failed (see the device log)");
  }

  void send_ok(uint8_t request_type) {
    send(Reply::Ok, eyou_protocol::make_ok_payload(request_type));
  }

  void send_error(uint8_t request_type, std::errc errc, std::string_view context) {
    const auto ec = std::make_error_code(errc);
    logger_.warn("{}: {}", context, ec.message());
    send(Reply::Error, eyou_protocol::make_error_payload(
                           request_type, static_cast<uint32_t>(ec.value()), std::string(context)));
  }

  void send(Reply type, std::span<const uint8_t> payload) {
    if (!config_.send) {
      logger_.warn("no send function configured; dropping a reply");
      return;
    }
    const auto t = static_cast<uint8_t>(type);
    config_.send(
        espp::stream_frame::build_frame(eyou_protocol::is_reply(t), module_id(), t, payload));
  }

  EyouMotor &motor_;
  Shared &shared_;
  Config config_;
  std::atomic<bool> stream_enabled_{false};
  std::atomic<uint16_t> stream_period_ms_;
  std::mutex latest_mutex_; ///< guards latest_ / have_latest_ / last_sent_
  Status latest_{};
  bool have_latest_{false};
  std::chrono::steady_clock::time_point last_sent_{};
};

static_assert(espp::DispatcherModuleConcept<EyouService>);
