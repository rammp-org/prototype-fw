#pragma once

// The STEM dispatcher module (module 7): decodes stem_proto request frames,
// drives the StemController and encodes the replies / telemetry. Transport
// agnostic -- the caller hands in a `send` for whichever USB stream (vendor /
// CDC) the request arrived on, and streaming is tracked per stream.

#include <array>
#include <atomic>
#include <functional>
#include <span>
#include <string>
#include <vector>

#include "base_component.hpp"
#include "dispatcher.hpp"
#include "stem_controller.hpp"
#include "stem_protocol.hpp"

namespace stem {

class StemModule : public espp::BaseComponent {
public:
  using send_fn = std::function<bool(std::span<const uint8_t>)>;

  enum class Transport : uint8_t { Vendor = 0, Cdc = 1 };
  static constexpr size_t kTransportCount = 2;

  struct AppInfo {
    std::string project;
    std::string version;
    std::string build;
    std::string idf;
  };

  struct Config {
    StemController &controller;
    AppInfo app{};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::INFO};
  };

  explicit StemModule(const Config &config);

  /// Metadata advertised to the Device Hub via dispatcher discovery.
  static espp::Dispatcher::ModuleInfo module_info();

  /// Handle one host->device request that arrived on `transport`; every reply
  /// is written through `send`. Runs in the caller's (USB RX worker) context.
  void handle_frame(const espp::stream_frame::Frame &frame, Transport transport,
                    const send_fn &send);

  bool streaming(Transport transport) const;
  void set_streaming(Transport transport, bool enable);
  bool any_streaming() const;
  uint16_t stream_period_ms() const { return period_ms_.load(); }

  /// Encode a State (reply) or Telemetry (periodic) frame from a snapshot.
  std::vector<uint8_t> build_state_frame(stem_proto::Msg type, const State &state,
                                         Transport transport) const;

private:
  std::vector<uint8_t> build_info(Transport transport) const;
  std::vector<uint8_t> build_moved(const PoseCommand &pose) const;
  std::vector<uint8_t> build_ik_result(float x_rel, float y_rel, bool reachable,
                                       const PoseCommand &pose) const;
  static std::vector<uint8_t> error_for(stem_proto::Msg request, StemController::Result result,
                                        const std::string &context);

  Config config_;
  std::array<std::atomic<bool>, kTransportCount> streaming_{};
  std::atomic<uint16_t> period_ms_{stem_proto::kDefaultStreamPeriodMs};
};

} // namespace stem
