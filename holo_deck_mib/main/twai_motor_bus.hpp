#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>

#include "base_component.hpp"
#include "twai.hpp"

#include "motor_actuator.hpp"

/// RMD-X6-S2 motor transport on espp::Twai.
///
/// Implements the MotorActuator::CommunicationFunction contract: send a
/// command frame to a motor and, for a request, wait for the matching reply.
/// The RMD protocol addresses motor N at CAN id 0x140+N and answers from
/// 0x240+N (the multi-motor broadcast 0x300 answers on 0x300); a reply carries
/// the command byte of the request in data[0].
///
/// One transaction at a time: the control loop's set-points and the status
/// poll share the bus through one mutex, so a poll can never interleave with a
/// set-point frame and a stray reply can never be matched to the wrong request.
class TwaiMotorBus : public espp::BaseComponent {
public:
  struct Config {
    gpio_num_t tx_gpio;          ///< TWAI TX -> transceiver TXD.
    gpio_num_t rx_gpio;          ///< TWAI RX <- transceiver RXD.
    uint32_t bitrate{1'000'000}; ///< Bus bit rate.
    /// Bound on one frame's transmit completion. Only reached when the controller
    /// is wedged: on a healthy 1 Mbit/s bus a frame completes in ~0.15 ms, and a
    /// missing motor fails the single-shot attempt just as fast (no ACK), so a
    /// 50 Hz tick's four set-points stay well inside the period either way.
    int tx_timeout_ms{2};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN};
  };

  explicit TwaiMotorBus(const Config &config);
  ~TwaiMotorBus();

  TwaiMotorBus(const TwaiMotorBus &) = delete;
  TwaiMotorBus &operator=(const TwaiMotorBus &) = delete;

  /// Bring up the TWAI node. \return true if the node is on the bus.
  bool start(std::error_code &ec);

  /// Fire-and-forget: transmit one command frame.
  bool send(const MotorPacket &command);

  /// Transmit one command frame and wait up to timeout_ms for the motor's reply.
  bool request(const MotorPacket &command, MotorPacket &response, uint32_t timeout_ms);

  /// The MotorActuator communication function: timeout_ms == 0 means send().
  MotorActuator::CommunicationFunction communication_function();

  /// Frames received that no request was waiting for (stale replies, other
  /// nodes); for diagnostics.
  uint32_t unexpected_rx_count() const { return unexpected_rx_.load(); }
  /// Transmits that failed (no ACK on the bus, bit error, ...).
  uint32_t tx_error_count() const { return tx_errors_.load(); }

protected:
  static uint32_t tx_id_for(const MotorPacket &command);
  static bool reply_matches(const MotorPacket &command, const espp::Twai::Message &m);
  bool transmit_locked(const MotorPacket &command);
  void on_receive(const espp::Twai::Message &m);

  Config config_;
  espp::Twai twai_;

  std::mutex transaction_mutex_; ///< one transaction (send, or send + reply) at a time

  // reply hand-off from the Twai receive task to the requester
  std::mutex reply_mutex_;
  std::condition_variable reply_cv_;
  bool waiting_{false};
  std::optional<MotorPacket> pending_command_{}; ///< what the waiter sent
  std::optional<MotorPacket> reply_{};           ///< the matching reply, once seen

  std::atomic<uint32_t> unexpected_rx_{0};
  std::atomic<uint32_t> tx_errors_{0};
};
