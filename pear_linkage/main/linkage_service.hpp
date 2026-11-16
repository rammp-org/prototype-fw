#pragma once

// LinkageService: the PEAR linkage protocol (linkage_protocol.hpp) as a
// dispatcher module around the LinkageController. One instance per USB
// transport (vendor / CDC); the application's poll task hands every status
// snapshot to each instance through on_status(), which streams it on the
// transport whose host asked, at that host's period, and keeps it for
// GET_STATUS. Commands run on the dispatcher worker that feeds handle().

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "base_component.hpp"
#include "dispatcher.hpp"
#include "linkage_controller.hpp"
#include "linkage_protocol.hpp"
#include "stream_frame.hpp"

class LinkageService : public espp::BaseComponent {
public:
  using Request = linkage_protocol::Request;
  using Reply = linkage_protocol::Reply;
  using Status = linkage_protocol::Status;
  using Settings = linkage_protocol::Settings;
  using send_fn = std::function<void(std::span<const uint8_t> frame)>;

  struct Config {
    /// Transmits an encoded frame (required; called from the dispatcher
    /// worker and from the poll task, so thread-safe).
    send_fn send{nullptr};
    uint8_t module{linkage_protocol::kModuleId};
    uint16_t default_stream_period_ms{100};
    /// What GET_INFO reports.
    linkage_protocol::Info info{};
    /// The firmware defaults RESET_SETTINGS goes back to.
    Settings defaults{};
    /// Called after new settings were applied (SET_SETTINGS / RESET_SETTINGS):
    /// the application persists them.
    std::function<void(const Settings &)> on_settings_changed{nullptr};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN};
  };

  LinkageService(LinkageController &controller, const Config &config)
      : espp::BaseComponent("LinkageService", config.log_level)
      , controller_(controller)
      , config_(config)
      , stream_period_ms_(config.default_stream_period_ms ? config.default_stream_period_ms : 100) {
  }

  LinkageService(const LinkageService &) = delete;
  LinkageService &operator=(const LinkageService &) = delete;

  uint8_t module_id() const { return config_.module; }

  espp::Dispatcher::ModuleInfo module_info() const {
    return {.name = "PEAR linkage",
            .app = "linkage_console.html",
            .description = "Configure and drive one 5-bar linkage (two Eyou actuators on CAN): "
                           "geometry, motor limits, end-effector moves, live pose",
            .protocol = linkage_protocol::kProtocol,
            .protocol_version = linkage_protocol::kProtocolVersion};
  }

  void handle(const espp::stream_frame::Frame &frame) {
    if (frame.module != module_id() || frame.is_reply())
      return;
    handle_request(frame.type, frame.payload);
  }

  /// The poll task's snapshot (every period, streamed or not).
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
  Status decorate(Status s) const {
    s.stream_enabled = stream_enabled_.load();
    s.stream_period_ms = stream_period_ms_.load();
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
                 "no poll of the motors yet");
    else
      send(Reply::Status, payload);
  }

  void handle_request(uint8_t type, std::span<const uint8_t> payload) {
    namespace proto = linkage_protocol;
    using Result = LinkageController::Result;
    auto malformed = [&](const char *what) { send_error(type, std::errc::invalid_argument, what); };
    auto finish = [&](Result result, const std::string &what) {
      if (result == Result::Ok) {
        send_ok(type);
        return;
      }
      const auto errc =
          result == Result::Unreachable || result == Result::OutOfLimits
              ? std::errc::result_out_of_range
          : result == Result::InvalidArgument ? std::errc::invalid_argument
          : result == Result::NotEnabled || result == Result::NoMotor || result == Result::NoPose
              ? std::errc::operation_not_permitted
              : std::errc::io_error;
      send_error(type, errc, what + ": " + LinkageController::to_string(result));
    };
    auto finish_bool = [&](bool ok, const std::string &what) {
      if (ok)
        send_ok(type);
      else
        send_error(type, std::errc::io_error, what + " failed (see the device log)");
    };
    auto motor_select = [&](std::optional<uint8_t> &motor) -> bool {
      const auto req = proto::MotorSelect::parse(payload);
      if (!req) {
        malformed("short payload");
        return false;
      }
      if (req->motor != proto::kAllMotors && req->motor >= proto::kMotorCount) {
        malformed("motor must be 0, 1 or 0xFF (all)");
        return false;
      }
      motor = req->motor;
      return true;
    };

    switch (static_cast<Request>(type)) {
    case Request::GetInfo:
      send(Reply::Info, config_.info.serialize());
      return;
    case Request::GetStatus:
      send_status();
      return;
    case Request::SetStream: {
      const auto req = proto::SetStream::parse(payload);
      if (!req)
        return malformed("short payload");
      stream_period_ms_.store(req->period_ms ? req->period_ms : config_.default_stream_period_ms);
      stream_enabled_.store(req->enabled);
      logger_.info("status stream {} (every {} ms)", req->enabled ? "enabled" : "disabled",
                   stream_period_ms_.load());
      send_ok(type);
      return;
    }
    case Request::GetSettings:
      send(Reply::Settings, controller_.settings().serialize());
      return;
    case Request::SetSettings:
    case Request::ResetSettings: {
      Settings settings = config_.defaults;
      if (static_cast<Request>(type) == Request::SetSettings) {
        const auto parsed = Settings::parse(payload);
        if (!parsed)
          return malformed("settings payload is short or of an unknown version");
        settings = *parsed;
      }
      std::string why;
      if (!controller_.apply_settings(settings, why)) {
        send_error(type, std::errc::invalid_argument, "settings refused: " + why);
        return;
      }
      if (config_.on_settings_changed)
        config_.on_settings_changed(settings);
      send_ok(type);
      // the host wants to see what is active now
      send(Reply::Settings, controller_.settings().serialize());
      return;
    }
    case Request::MoveTo: {
      const auto req = proto::Point2::parse(payload);
      if (!req)
        return malformed("short payload");
      return finish(controller_.move_to(req->x, req->y), "move to");
    }
    case Request::MoveBy: {
      const auto req = proto::Point2::parse(payload);
      if (!req)
        return malformed("short payload");
      return finish(controller_.move_by(req->x, req->y), "move by");
    }
    case Request::Home:
      return finish(controller_.home(), "home");
    case Request::SetMotor: {
      const auto req = proto::SetMotor::parse(payload);
      if (!req)
        return malformed("short payload");
      return finish(controller_.set_motor(req->motor, req->degrees), "set motor");
    }
    case Request::Enable: {
      std::optional<uint8_t> motor;
      if (motor_select(motor))
        finish_bool(controller_.enable(*motor), "enable");
      return;
    }
    case Request::Disable: {
      std::optional<uint8_t> motor;
      if (motor_select(motor))
        finish_bool(controller_.disable(*motor), "disable");
      return;
    }
    case Request::ResetFault: {
      std::optional<uint8_t> motor;
      if (motor_select(motor))
        finish_bool(controller_.reset_fault(*motor), "reset fault");
      return;
    }
    case Request::Halt: {
      std::optional<uint8_t> motor;
      if (motor_select(motor))
        finish_bool(controller_.halt(*motor), "halt");
      return;
    }
    case Request::ZeroMotor: {
      const auto req = proto::ZeroMotor::parse(payload);
      if (!req)
        return malformed("short payload");
      return finish(controller_.zero_motor(req->motor, req->mode, req->value), "zero");
    }
    case Request::SaveZero: {
      std::optional<uint8_t> motor;
      if (motor_select(motor))
        finish_bool(controller_.save_zero(*motor), "save zero");
      return;
    }
    case Request::LoadZero: {
      std::optional<uint8_t> motor;
      if (motor_select(motor))
        finish_bool(controller_.load_zero(*motor), "load zero");
      return;
    }
    case Request::SolveIk: {
      const auto req = proto::Point2::parse(payload);
      if (!req)
        return malformed("short payload");
      proto::IkResult result{};
      controller_.solve(req->x, req->y, result);
      send(Reply::IkResult, result.serialize());
      return;
    }
    case Request::RecoverCan:
      finish_bool(controller_.recover_can(), "CAN bus-off recovery");
      return;
    case Request::SetDriveNodeId: {
      if (payload.size() < 2)
        return malformed("needs [motor u8][new node id u8]");
      const uint8_t motor = payload[0], new_id = payload[1];
      if (motor >= proto::kMotorCount)
        return malformed("motor must be 0 or 1");
      if (new_id < 1 || new_id > 127)
        return malformed("node id must be 1..127");
      switch (controller_.set_drive_node_id(motor, new_id)) {
      case EyouMotor::NodeIdResult::Applied:
        // the settings' node id followed the drive: persist them like SET_SETTINGS
        if (config_.on_settings_changed)
          config_.on_settings_changed(controller_.settings());
        send_ok(type);
        return;
      case EyouMotor::NodeIdResult::SavedNeedsPowerCycle:
        send_error(type, std::errc::operation_in_progress,
                   "saved in the drive: it keeps its old node id until power-cycled; set the "
                   "settings' node id to the new one after that");
        return;
      case EyouMotor::NodeIdResult::Failed:
        send_error(type, std::errc::io_error,
                   "re-addressing failed, nothing changed (see the device log)");
        return;
      }
      return;
    }
    default:
      send_error(type, std::errc::not_supported, "unknown linkage message");
      return;
    }
  }

  void send_ok(uint8_t request_type) {
    send(Reply::Ok, linkage_protocol::make_ok_payload(request_type));
  }

  void send_error(uint8_t request_type, std::errc errc, std::string_view context) {
    const auto ec = std::make_error_code(errc);
    logger_.warn("{}: {}", context, ec.message());
    send(Reply::Error, linkage_protocol::make_error_payload(
                           request_type, static_cast<uint32_t>(ec.value()), std::string(context)));
  }

  void send(Reply type, std::span<const uint8_t> payload) {
    if (!config_.send) {
      logger_.warn("no send function configured; dropping a reply");
      return;
    }
    const auto t = static_cast<uint8_t>(type);
    config_.send(
        espp::stream_frame::build_frame(linkage_protocol::is_reply(t), module_id(), t, payload));
  }

  LinkageController &controller_;
  Config config_;
  std::atomic<bool> stream_enabled_{false};
  std::atomic<uint16_t> stream_period_ms_;
  std::mutex latest_mutex_;
  Status latest_{};
  bool have_latest_{false};
  std::chrono::steady_clock::time_point last_sent_{};
};

static_assert(espp::DispatcherModuleConcept<LinkageService>);
