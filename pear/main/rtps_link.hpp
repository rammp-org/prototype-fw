#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "base_component.hpp"
#include "rtps_participant.hpp"
#include "rtps_pubsub.hpp"
#include "timer.hpp"

#include "messages.hpp" // rammp-rtps: XYTwist / DriveCommand / SeatCommand / MibStatus / Diagnostics / MotorState

#include "drive_controller.hpp"
#include "linkage_controller.hpp"

/// The MIB side of the RAMMP RTPS link for the PEAR base (the joystick HMI is
/// pace-hmi-fw). holo_deck_mib's link plus the linkages and the motor layer.
///
/// Subscribes to the joystick's topics and drives the controllers from them;
/// publishes the base's state back at the periods the HMI expects. All message
/// callbacks run in the RTPS worker context and only call the controllers'
/// thread-safe setters (a linkage request is queued, never enacted there).
///
/// - XYTwist (~30 Hz while driving): x + = right, y + = forward, twist + =
///   clockwise, already deadzoned on the HMI (exactly 0 = centered). Mapped to
///   the drive controller's (forward, left, ccw). A watchdog feeds zeros when
///   the stream stops (hold timeout) and e-stops when it stays silent (lost
///   timeout).
/// - DriveCommand: ENABLE / DISABLE driving, with the drive profile that sets
///   the speed / rotation limits. The HMI shows driving only once MibStatus says
///   ENABLED, so the drive controller's mode is the single source of truth.
/// - SeatCommand{axis, target}: absolute, in the axis' whole unit. The pose
///   rows FRONT_BACK_TILT / LATERAL_TILT (deg) / ELEVATION (mm) replace that
///   component of the measured pose and request the pose; the PEAR_LINK_*
///   rows request one corner's crank angle (deg); TRANSLATION is logged once
///   and ignored (the base has no translation axis). The linkage controller
///   clamps and applies the interlocks (only while not driving).
/// - MibStatus (500 ms): systemState (INITIALIZING until the CAN bus is up,
///   ENABLED while driving, IDLE otherwise, ERROR when a motor command fails or
///   a motor is faulted - named in error_message), activeProfile, seatState
///   (the measured pose in table units: deg / mm, translation 0, and the four
///   corner angles), speed = |commanded translation| m/s, seq, status text.
/// - Diagnostics (500 ms): one item per RAMMP_DIAG_TABLE row: TEST rows zeros,
///   PEAR_WHEEL_* temperature / current / velocity, PEAR_LINK_* temperature /
///   current (0: the Eyou driver exposes neither) / position.
/// - MotorState (100 ms) on rammp::axis(PEAR_*).state for all eight motors
///   (the motor layer's board -> MIB message, published by the MIB itself for
///   its CAN motors): state ARMED / DISARMED / FAULT, mode VELOCITY (wheels) /
///   POSITION (linkages) while ARMED else COAST, position rad, velocity rad/s
///   (wheels; the linkage velocity is not measured: 0), current in iq,
///   temperature in temps[0]. MotorCommand is NOT subscribed: the CAN motors
///   are not PACE RACER boards.
class RtpsLink : public espp::BaseComponent {
public:
  struct Config {
    DriveController &drive;                                ///< The wheel controller.
    LinkageController &linkage;                            ///< The corner-linkage controller.
    std::chrono::milliseconds publish_period{500};         ///< MibStatus / Diagnostics period.
    std::chrono::milliseconds motor_state_period{100};     ///< MotorState period (all axes).
    std::chrono::milliseconds joystick_hold_timeout{500};  ///< No XYTwist -> zero velocity.
    std::chrono::milliseconds joystick_lost_timeout{2000}; ///< No XYTwist -> e-stop.
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN};
  };

  /// The eight PEAR motor axes of the RTPS axis table, wheels then linkages,
  /// each in pear::Corner order.
  static constexpr size_t kMotorAxisCount = 2 * pear::kCornerCount;
  static constexpr std::array<rammp::AxisId, kMotorAxisCount> kMotorAxes = {
      rammp::AxisId::PEAR_WHEEL_FR, rammp::AxisId::PEAR_WHEEL_FL, rammp::AxisId::PEAR_WHEEL_BR,
      rammp::AxisId::PEAR_WHEEL_BL, rammp::AxisId::PEAR_LINK_FR,  rammp::AxisId::PEAR_LINK_FL,
      rammp::AxisId::PEAR_LINK_BR,  rammp::AxisId::PEAR_LINK_BL,
  };

  explicit RtpsLink(const Config &config);
  ~RtpsLink();

  RtpsLink(const RtpsLink &) = delete;
  RtpsLink &operator=(const RtpsLink &) = delete;

  /// Start the participant (on the interface with that IPv4 address), the
  /// endpoints and the timers. \return true on success.
  bool start(const std::string &interface_address);

  /// Mark the base ready (leaves INITIALIZING) or not.
  void set_ready(bool ready);

  /// Set the error text reported in MibStatus (empty = no error).
  void set_error(const std::string &message, const std::string &footer = {});

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
  bool motor_state_step();
  bool watchdog_step();
  MIB::MibStatus build_status();
  rammp::Diagnostics build_diagnostics();
  rammp::MotorState build_wheel_state(size_t corner, const DriveController::State &drive);
  rammp::MotorState build_linkage_state(size_t corner, const LinkageController::State &linkage);

  Config config_;
  DriveController &drive_;
  LinkageController &linkage_;
  std::string interface_address_; ///< set by start()

  std::unique_ptr<espp::RtpsParticipant> participant_;
  std::unique_ptr<espp::Publisher<MIB::MibStatus>> status_pub_;
  std::unique_ptr<espp::Publisher<rammp::Diagnostics>> diag_pub_;
  std::array<std::unique_ptr<espp::Publisher<rammp::MotorState>>, kMotorAxisCount> motor_pubs_;
  std::unique_ptr<espp::Subscriber<rammp::XYTwist>> xy_twist_sub_;
  std::unique_ptr<espp::Subscriber<rammp::DriveCommand>> drive_sub_;
  std::unique_ptr<espp::Subscriber<rammp::SeatCommand>> seat_sub_;
  std::unique_ptr<espp::Timer> publish_timer_;
  std::unique_ptr<espp::Timer> motor_state_timer_;
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
  std::array<uint8_t, kMotorAxisCount> motor_seq_{};
  std::chrono::steady_clock::time_point last_xy_twist_{};
  bool have_xy_twist_{false};
  /// What the watchdog times from: the last XYTwist, or the ENABLE request if
  /// none has arrived since - a publisher that never starts is still "lost".
  std::chrono::steady_clock::time_point watchdog_fed_{};
  /// The watchdog is armed: DRIVE was requested by the HMI, or XYTwist is
  /// streaming. A CLI `enable` on the bench (no joystick) is not watched until
  /// the stream starts; cleared whenever the controller leaves DRIVE.
  bool watch_joystick_{false};
  /// Bumped every time the watchdog is armed, so a watchdog tick that saw the
  /// controller disabled before an ENABLE slipped in does not disarm the new one.
  uint32_t watch_generation_{0};
  uint32_t rejected_twist_{0}; ///< XYTwist samples dropped for non-finite values
  std::chrono::steady_clock::time_point last_message_{}; ///< any HMI topic
  bool have_message_{false};
  bool joystick_held_{false};      ///< zeros fed because the stream paused
  bool joystick_lost_{false};      ///< e-stopped because the stream stopped
  bool translation_warned_{false}; ///< the TRANSLATION row was requested once
};
