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
    /// After a request times out, how long its reply may still turn up. A
    /// reply carries only the motor id and the command byte, so one arriving
    /// once the NEXT request to that motor / command is armed would be taken
    /// for its answer (a repeated 0x81 stop, typically); such a request waits
    /// out this window first, and a reply matching the timed-out request that
    /// arrives inside it is discarded. An RMD answers well inside 1 ms.
    uint32_t stale_reply_grace_ms{5};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN};
  };

  explicit TwaiMotorBus(const Config &config);
  ~TwaiMotorBus();

  TwaiMotorBus(const TwaiMotorBus &) = delete;
  TwaiMotorBus &operator=(const TwaiMotorBus &) = delete;

  /// Bring up the TWAI node. \return true if the node is on the bus.
  bool start(std::error_code &ec);

  /// Transmit one command frame and wait for it to complete on the bus (up to
  /// Config::tx_timeout_ms): true means the frame was acknowledged by at least
  /// one node, not that the addressed motor received it. For a per-motor
  /// confirmation use request() (the RMD echoes every command it takes).
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
  /// Late replies to requests that had already timed out, discarded.
  uint32_t stale_reply_count() const { return stale_rx_.load(); }

protected:
  static uint32_t tx_id_for(const MotorPacket &command);
  static bool reply_matches(const MotorPacket &command, const espp::Twai::Message &m);
  /// Whether a reply to \p a would also pass as a reply to \p b (same bus id
  /// and command byte).
  static bool same_reply_key(const MotorPacket &a, const MotorPacket &b);
  bool transmit_locked(const MotorPacket &command);
  void on_receive(const espp::Twai::Message &m);

  Config config_;

  std::mutex transaction_mutex_; ///< one transaction (send, or send + reply) at a time

  // reply hand-off from the Twai receive task to the requester
  std::mutex reply_mutex_;
  std::condition_variable reply_cv_;
  bool waiting_{false};
  std::optional<MotorPacket> pending_command_{}; ///< what the waiter sent
  std::optional<MotorPacket> reply_{};           ///< the matching reply, once seen
  /// The last request that timed out, while its reply could still turn up
  /// (until quarantine_until_); see Config::stale_reply_grace_ms.
  std::optional<MotorPacket> quarantined_command_{};
  std::chrono::steady_clock::time_point quarantine_until_{};

  std::atomic<uint32_t> unexpected_rx_{0};
  std::atomic<uint32_t> tx_errors_{0};
  std::atomic<uint32_t> stale_rx_{0};

  /// Declared LAST on purpose: members are destroyed in reverse order, so the
  /// transport (and its receive task, which calls on_receive() and touches the
  /// reply state above) is torn down before that state goes away.
  espp::Twai twai_;
};
