#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "base_component.hpp"
#include "rtps_participant.hpp"
#include "rtps_pubsub.hpp"
#include "timer.hpp"

#include "messages.hpp" // rammp-rtps: XYTwist / DriveCommand / SeatCommand / MibStatus / Diagnostics

#include "holo_deck_controller.hpp"

/// The MIB side of the RAMMP RTPS link (the joystick HMI is pace-hmi-fw).
///
/// Subscribes to the joystick's topics and drives the HoloDeckController from
/// them; publishes the chair's state back on MibStatus + Diagnostics at the
/// period the HMI expects. All message callbacks run in the RTPS worker
/// context and only call the controller's thread-safe setters.
///
/// - XYTwist (~30 Hz while driving): x + = right, y + = forward, twist + =
///   clockwise, already deadzoned on the HMI (exactly 0 = centered). Mapped to
///   the controller's (forward, left, ccw). A watchdog feeds zeros when the
///   stream stops (hold timeout) and e-stops when it stays silent (lost timeout).
/// - DriveCommand: ENABLE / DISABLE driving, with the drive profile that sets
///   the speed / rotation limits. The HMI shows driving only once MibStatus says
///   ENABLED, so the controller's mode is the single source of truth.
/// - SeatCommand: the holonomic platform has no seat; logged and ignored.
/// - MibStatus: systemState (INITIALIZING until the CAN bus is up, ENABLED while
///   driving, IDLE otherwise, ERROR when motor commands fail), activeProfile,
///   seat zeros, speed = |commanded translation|, seq, status/error text.
/// - Diagnostics: one item per RAMMP_DIAG_TABLE row from the motors' status
///   polls (temperature, torque raw, output angle).
class RtpsLink : public espp::BaseComponent {
public:
  struct Config {
    std::string interface_address;                   ///< IPv4 of the Ethernet interface.
    HoloDeckController &controller;                  ///< The platform controller to drive.
    std::chrono::milliseconds publish_period;        ///< MibStatus / Diagnostics period.
    std::chrono::milliseconds joystick_hold_timeout; ///< No XYTwist -> zero velocity.
    std::chrono::milliseconds joystick_lost_timeout; ///< No XYTwist -> e-stop.
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN};
  };

  explicit RtpsLink(const Config &config);
  ~RtpsLink();

  RtpsLink(const RtpsLink &) = delete;
  RtpsLink &operator=(const RtpsLink &) = delete;

  /// Start the participant, endpoints and timers. \return true on success.
  bool start();

  /// Mark the chair ready (leaves INITIALIZING) or not.
  void set_ready(bool ready);

  /// Set the error text reported in MibStatus (empty = no error).
  void set_error(const std::string &message, const std::string &footer = {});

  /// The drive profile the HMI last asked for.
  MIB::DriveProfile active_profile() const;
  /// True once discovery matched a remote reader / writer (never cleared: the
  /// participant reports matches only). Use hmi_alive() for the live link.
  bool peer_matched() const { return peer_matched_.load(); }
  /// True while the HMI is heard from: a message on any of its topics within
  /// the joystick-lost timeout (it streams XYTwist continuously once matched).
  bool hmi_alive() const;
  /// Age of the last XYTwist, or nullopt if none was ever received.
  std::optional<std::chrono::milliseconds> joystick_age() const;

protected:
  void on_xy_twist(const rammp::XYTwist &msg);
  void on_drive_command(const rammp::DriveCommand &msg);
  void on_seat_command(const rammp::SeatCommand &msg);
  void apply_profile(MIB::DriveProfile profile);
  bool publish_step();
  bool watchdog_step();
  MIB::MibStatus build_status();
  rammp::Diagnostics build_diagnostics();

  Config config_;
  HoloDeckController &controller_;

  std::unique_ptr<espp::RtpsParticipant> participant_;
  std::unique_ptr<espp::Publisher<MIB::MibStatus>> status_pub_;
  std::unique_ptr<espp::Publisher<rammp::Diagnostics>> diag_pub_;
  std::unique_ptr<espp::Subscriber<rammp::XYTwist>> xy_twist_sub_;
  std::unique_ptr<espp::Subscriber<rammp::DriveCommand>> drive_sub_;
  std::unique_ptr<espp::Subscriber<rammp::SeatCommand>> seat_sub_;
  std::unique_ptr<espp::Timer> publish_timer_;
  std::unique_ptr<espp::Timer> watchdog_timer_;

  std::atomic<bool> peer_matched_{false};
  std::atomic<bool> ready_{false};

  mutable std::mutex mutex_;
  MIB::DriveProfile profile_{MIB::DriveProfile::NORMAL};
  std::string error_message_;
  std::string error_footer_;
  std::string status_text_;
  uint8_t status_seq_{0};
  uint8_t diag_seq_{0};
  std::chrono::steady_clock::time_point last_xy_twist_{};
  bool have_xy_twist_{false};
  /// What the watchdog times from: the last XYTwist, or the ENABLE request if
  /// none has arrived since - a publisher that never starts is still "lost".
  std::chrono::steady_clock::time_point watchdog_fed_{};
  /// The watchdog is armed: DRIVE was requested by the HMI, or XYTwist is
  /// streaming. A CLI `enable` on the bench (no joystick) is not watched until
  /// the stream starts; cleared whenever the controller leaves DRIVE.
  bool watch_joystick_{false};
  std::chrono::steady_clock::time_point last_message_{}; ///< any HMI topic
  bool have_message_{false};
  bool joystick_held_{false}; ///< zeros fed because the stream paused
  bool joystick_lost_{false}; ///< e-stopped because the stream stopped
  bool seat_warned_{false};
};
