#include "stem_module.hpp"

#include <algorithm>
#include <cmath>
#include <system_error>

#include "esp_timer.h"

namespace stem {

namespace proto = stem_proto;
using Msg = proto::Msg;
using Result = StemController::Result;

namespace {

uint32_t uptime_ms() {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

void put_point(std::vector<uint8_t> &out, const std::array<float, 2> &point) {
  proto::put_f32(out, point[0]);
  proto::put_f32(out, point[1]);
}

} // namespace

StemModule::StemModule(const Config &config)
    : espp::BaseComponent("StemModule", config.log_level), config_(config) {}

espp::Dispatcher::ModuleInfo StemModule::module_info() {
  return {.name = "STEM Linkage",
          .app = "stem_console.html",
          .description = "5-bar seat linkage: live pose, drag-to-move, pair control, telemetry"};
}

bool StemModule::streaming(Transport transport) const {
  return streaming_[static_cast<size_t>(transport)].load();
}

void StemModule::set_streaming(Transport transport, bool enable) {
  streaming_[static_cast<size_t>(transport)].store(enable);
}

bool StemModule::any_streaming() const {
  for (const auto &flag : streaming_) {
    if (flag.load())
      return true;
  }
  return false;
}

std::vector<uint8_t> StemModule::error_for(Msg request, Result result,
                                           const std::string &context) {
  std::errc errc = std::errc::invalid_argument;
  switch (result) {
  case Result::Unreachable:
    errc = std::errc::argument_out_of_domain;
    break;
  case Result::OutOfLimits:
    errc = std::errc::result_out_of_range;
    break;
  case Result::ActuatorFailed:
    errc = std::errc::io_error;
    break;
  case Result::InvalidArgument:
  case Result::Ok:
    break;
  }
  const auto ec = std::make_error_code(errc);
  return proto::make_error(request, static_cast<uint32_t>(ec.value()),
                           context + ": " + StemController::to_string(result));
}

std::vector<uint8_t> StemModule::build_info(Transport transport) const {
  const auto &cfg = config_.controller.config();
  std::vector<uint8_t> payload;
  payload.push_back(proto::kProtocolVersion);
  proto::put_str(payload, config_.app.project);
  proto::put_str(payload, config_.app.version);
  proto::put_str(payload, config_.app.build);
  proto::put_str(payload, config_.app.idf);

  payload.push_back(proto::kPairCount);
  const PairedActuator *pairs[] = {&cfg.left, &cfg.right, &cfg.seat};
  for (uint8_t i = 0; i < proto::kPairCount; ++i) {
    payload.push_back(i);
    proto::put_str(payload, to_string(static_cast<Pair>(i)));
    payload.push_back(pairs[i]->primary_id());
    payload.push_back(pairs[i]->secondary_id());
    proto::put_f32(payload, cfg.min_deg);
    proto::put_f32(payload, cfg.max_deg);
  }

  const auto &g = cfg.geometry;
  for (const float value :
       {g.m1_x, g.m1_y, g.m2_x, g.m2_y, g.j4_x, g.j4_y, g.bar_length_m3_to_j2,
        g.bar_length_m3_to_j3, g.bar_length_m3_to_j5, g.bar_length_j1_to_j3,
        g.bar_length_j5_perp_offset, g.crank_length_m1_to_j2, g.crank_length_m2_to_j1}) {
    proto::put_f32(payload, value);
  }
  const auto &r = cfg.reference;
  for (const float value : {r.x, r.y, r.m1_angle_deg, r.m2_angle_deg, r.m3_angle_deg}) {
    proto::put_f32(payload, value);
  }
  proto::put_f32(payload, cfg.default_move_rpm);
  proto::put_f32(payload, cfg.default_set_rpm);
  proto::put_f32(payload, cfg.max_rpm);
  proto::put_u16(payload, period_ms_.load());
  payload.push_back(streaming(transport) ? 1 : 0);
  return proto::build(Msg::Info, payload);
}

std::vector<uint8_t> StemModule::build_state_frame(Msg type, const State &state,
                                                   Transport transport) const {
  std::vector<uint8_t> payload;
  proto::put_u32(payload, uptime_ms());
  uint8_t flags = 0;
  if (streaming(transport))
    flags |= proto::flags::kStreaming;
  if (state.target_valid)
    flags |= proto::flags::kTargetValid;
  if (state.pose_valid)
    flags |= proto::flags::kPoseValid;
  if (state.motors_ok)
    flags |= proto::flags::kMotorsOk;
  if (state.can_ok)
    flags |= proto::flags::kCanOk;
  payload.push_back(flags);
  proto::put_f32(payload, state.target_x);
  proto::put_f32(payload, state.target_y);
  proto::put_f32(payload, state.pose_x);
  proto::put_f32(payload, state.pose_y);
  for (const float deg : state.pair_deg) {
    proto::put_f32(payload, deg);
  }
  payload.push_back(static_cast<uint8_t>(state.motors.size()));
  for (const auto &motor : state.motors) {
    payload.push_back(motor.id);
    payload.push_back(motor.ok ? 1 : 0);
    payload.push_back(static_cast<uint8_t>(motor.temperature_c));
    proto::put_i16(payload, motor.torque_raw);
    proto::put_f32(payload, motor.velocity_rpm);
    proto::put_f32(payload, motor.tracked_deg);
  }
  return proto::build(type, payload);
}

std::vector<uint8_t> StemModule::build_moved(const PoseCommand &pose) const {
  std::vector<uint8_t> payload;
  proto::put_f32(payload, pose.x_rel);
  proto::put_f32(payload, pose.y_rel);
  for (const float deg : pose.pair_deg) {
    proto::put_f32(payload, deg);
  }
  return proto::build(Msg::Moved, payload);
}

std::vector<uint8_t> StemModule::build_ik_result(float x_rel, float y_rel, bool reachable,
                                                 const PoseCommand &pose) const {
  std::vector<uint8_t> payload;
  proto::put_f32(payload, x_rel);
  proto::put_f32(payload, y_rel);
  payload.push_back(reachable ? 1 : 0);
  payload.push_back(reachable && pose.within_limits ? 1 : 0);
  if (reachable) {
    for (const float deg : pose.pair_deg) {
      proto::put_f32(payload, deg);
    }
    put_point(payload, pose.solution.j1);
    put_point(payload, pose.solution.j2);
    put_point(payload, pose.solution.j3);
    put_point(payload, pose.solution.j5);
    put_point(payload, pose.solution.m3);
  }
  return proto::build(Msg::IkResult, payload);
}

void StemModule::handle_frame(const espp::stream_frame::Frame &frame, Transport transport,
                              const send_fn &send) {
  auto &controller = config_.controller;
  const auto request = static_cast<Msg>(frame.type);
  const std::span<const uint8_t> payload = frame.payload;
  auto bad_args = [&](const char *expected) {
    send(proto::make_error(request,
                           static_cast<uint32_t>(
                               std::make_error_code(std::errc::invalid_argument).value()),
                           expected));
  };
  auto reply_result = [&](Result result, const char *context) {
    if (result == Result::Ok)
      send(proto::make_ok(request, 0));
    else
      send(error_for(request, result, context));
  };

  switch (request) {
  case Msg::GetInfo:
    send(build_info(transport));
    break;

  case Msg::GetState:
    send(build_state_frame(Msg::State, controller.snapshot(true), transport));
    break;

  case Msg::MoveTo: {
    const auto x = proto::get_f32_at(payload, 0);
    const auto y = proto::get_f32_at(payload, 4);
    const auto rpm = proto::get_f32_at(payload, 8);
    if (payload.size() != 12 || !x || !y || !rpm) {
      bad_args("MOVE_TO needs f32 x_rel, f32 y_rel, f32 rpm");
      break;
    }
    PoseCommand pose{};
    const Result result = controller.move_to(*x, *y, *rpm, &pose);
    if (result == Result::Ok)
      send(build_moved(pose));
    else
      send(error_for(request, result, "move rejected"));
    break;
  }

  case Msg::Home: {
    float rpm = 0.0f;
    if (!payload.empty()) {
      const auto value = proto::get_f32_at(payload, 0);
      if (payload.size() != 4 || !value) {
        bad_args("HOME takes an optional f32 rpm");
        break;
      }
      rpm = *value;
    }
    PoseCommand pose{};
    const Result result = controller.home(rpm, &pose);
    if (result == Result::Ok)
      send(build_moved(pose));
    else
      send(error_for(request, result, "home rejected"));
    break;
  }

  case Msg::SetPair: {
    const auto index = proto::get_u8_at(payload, 0);
    const auto degrees = proto::get_f32_at(payload, 1);
    const auto rpm = proto::get_f32_at(payload, 5);
    const auto pair = index ? pair_from_index(*index) : std::nullopt;
    if (payload.size() != 9 || !pair || !degrees || !rpm) {
      bad_args("SET_PAIR needs u8 pair (0 left, 1 right, 2 seat), f32 degrees, f32 rpm");
      break;
    }
    reply_result(controller.set_pair(*pair, *degrees, *rpm), "set pair rejected");
    break;
  }

  case Msg::Stop:
    reply_result(controller.stop() ? Result::Ok : Result::ActuatorFailed, "stop");
    break;

  case Msg::Release:
    reply_result(controller.release_brakes() ? Result::Ok : Result::ActuatorFailed, "release");
    break;

  case Msg::Zero: {
    const auto index = proto::get_u8_at(payload, 0);
    const auto align = proto::get_u8_at(payload, 1);
    if (payload.size() != 2 || !index || !align ||
        (*index != proto::kAllPairs && !pair_from_index(*index))) {
      bad_args("ZERO needs u8 pair (0xFF = all), u8 align");
      break;
    }
    const std::optional<Pair> pair =
        *index == proto::kAllPairs ? std::nullopt : pair_from_index(*index);
    reply_result(controller.zero(pair, *align != 0) ? Result::Ok : Result::ActuatorFailed, "zero");
    break;
  }

  case Msg::SetStreaming: {
    const auto enable = proto::get_u8_at(payload, 0);
    const auto period = proto::get_u16_at(payload, 1);
    if (payload.size() != 3 || !enable || !period) {
      bad_args("SET_STREAMING needs u8 enable, u16 period_ms");
      break;
    }
    const uint16_t period_ms =
        std::clamp<uint16_t>(*period == 0 ? proto::kDefaultStreamPeriodMs : *period,
                             proto::kMinStreamPeriodMs, proto::kMaxStreamPeriodMs);
    period_ms_.store(period_ms);
    set_streaming(transport, *enable != 0);
    logger_.info("telemetry streaming {} on {} (period {} ms)", *enable ? "on" : "off",
                 transport == Transport::Vendor ? "WebUSB" : "CDC", period_ms);
    send(proto::make_ok(request, period_ms));
    break;
  }

  case Msg::SolveIk: {
    const auto x = proto::get_f32_at(payload, 0);
    const auto y = proto::get_f32_at(payload, 4);
    if (payload.size() != 8 || !x || !y) {
      bad_args("SOLVE_IK needs f32 x_rel, f32 y_rel");
      break;
    }
    PoseCommand pose{};
    const bool reachable = controller.solve(*x, *y, pose);
    send(build_ik_result(*x, *y, reachable, pose));
    break;
  }

  default:
    send(proto::make_error(
        request, static_cast<uint32_t>(std::make_error_code(std::errc::not_supported).value()),
        "unknown STEM message type"));
    break;
  }
}

} // namespace stem
