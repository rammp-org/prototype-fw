#pragma once

#include <array>
#include <chrono>
#include <functional>
#include <mutex>

#include "base_component.hpp"
#include "timer.hpp"

#include "ds402_velocity_motor.hpp"
#include "holo_deck_platform.hpp"
#include "linkage_geometry.hpp"

/// The DriveController owns the platform command state and the fixed-rate
/// control loop for the four hub-wheel drives (holo_deck_mib's
/// HoloDeckController on Ds402VelocityMotor):
///
/// * It keeps a velocity setpoint (vx m/s forward, vy m/s RIGHT, w RPM
///   counter-clockwise) per control source (GUI/CLI vs. the physical
///   joystick), an enable/e-stop flag, and runtime-adjustable translation /
///   rotation limits.
/// * A 50 Hz espp::Timer reads the active source's setpoint, computes the
///   four wheel speeds via HoloDeckPlatform, scales them so no wheel exceeds
///   the configured wheel-RPM limit, and streams the velocity set-points over
///   the CAN bus (non-blocking SDO downloads).
/// * A slower timer polls one drive's status per tick (round-robin) for the
///   CLI / diagnostics / MotorState.
///
/// Modes: DRIVE streams the active setpoint; STOPPED (e-stop) streams zero
/// EVERY tick - the L2DB drives drop the shaft after their communication-loss
/// timeout if the set-point stream stops, so a stop keeps streaming zeros and
/// the drives hold; DISABLED sends one acknowledged zero and then powers the
/// stage off (Ds402VelocityMotor::disable_drive), after which the next DRIVE
/// re-enables the drives from the control loop before the first set-point.
///
/// Source arbitration: any joystick deflection beyond the deadzone makes the
/// joystick the active source. Once the joystick has been centered for longer
/// than the release timeout, control falls back to the GUI setpoint, which is
/// zeroed on the handover so the platform never jumps. STOP always wins.
///
/// IMPORTANT: every motor command is issued from the single control loop, so a
/// STOP can never be overtaken by an in-flight drive command from another
/// thread. The setters only change the mode; the next control tick (<= one
/// control period) enacts it.
///
/// All public setters and state() are thread-safe.
class DriveController : public espp::BaseComponent {
public:
  /// The controller's operating mode.
  enum class Mode {
    DRIVE,    ///< The active source drives the platform (normal operation).
    STOPPED,  ///< E-stop: the loop streams zero velocity (drives hold at 0 in
              ///< closed loop). Overrides all sources.
    DISABLED, ///< Drives powered off at the DS402 level (shaft loose), NOT held
              ///< at zero. Re-enable to drive.
  };

  /// The currently-active control source (meaningful only in DRIVE mode).
  enum class Source {
    STOPPED,  ///< E-stopped / disabled; motors are not being driven.
    GUI,      ///< The local (CLI) setpoint drives the platform.
    JOYSTICK, ///< The remote joystick (RTPS XYTwist) drives the platform.
  };

  /// Last known status of one wheel drive (pear::Corner order).
  struct MotorStatus {
    uint8_t id{0};                    ///< CAN node id.
    float velocity_rpm{0.0f};         ///< Last reported output-shaft velocity.
    float temperature_c{0.0f};        ///< Last reported driver temperature.
    float current_a{0.0f};            ///< Last reported torque current.
    float angle_degrees{0.0f};        ///< Last reported output-shaft position.
    uint16_t statusword{0};           ///< Last reported statusword.
    uint16_t error_code{0};           ///< Last reported error bits (0x2601).
    float commanded_rpm{0.0f};        ///< Last commanded wheel speed.
    bool valid{false};                ///< True once a status read has succeeded.
    bool stale{true};                 ///< True if not successfully read recently.
    bool faulted{false};              ///< Statusword alarm bit (from the last read).
    uint32_t reply_latency_us{0};     ///< SDO round trip of the last status read.
    uint32_t max_reply_latency_us{0}; ///< Longest SDO round trip seen.
  };

  /// Snapshot of the full controller state, for the GUI / CLI.
  struct State {
    Mode mode{Mode::STOPPED}; ///< Operating mode.
    bool enabled{false};      ///< Convenience: mode == DRIVE.
    bool armed{false};        ///< The drives' power stages are on (enable_drive done).
    Source source{Source::STOPPED};
    float vx_mps{0.0f};               ///< Active commanded forward velocity.
    float vy_mps{0.0f};               ///< Active commanded velocity, + = RIGHT (platform frame).
    float w_rpm{0.0f};                ///< Active commanded CCW rotation rate.
    float gui_vx_mps{0.0f};           ///< GUI/CLI setpoint (pre-arbitration).
    float gui_vy_mps{0.0f};           ///< GUI/CLI setpoint (pre-arbitration), + = right.
    float gui_w_rpm{0.0f};            ///< GUI/CLI setpoint (pre-arbitration).
    float max_speed_mps{0.0f};        ///< Current translation limit.
    float max_rotation_rpm{0.0f};     ///< Current rotation-rate limit.
    float twist_rotation_scale{1.0f}; ///< Joystick-twist rotation scale.
    float max_wheel_rpm{0.0f};        ///< Per-wheel scaling limit.
    std::array<MotorStatus, pear::kCornerCount> motors; ///< pear::Corner order.
    /// True while the last control tick failed to deliver a velocity (or stop)
    /// command to at least one motor: a CAN / motor fault the operator must see.
    bool send_failing{false};
    /// Node id of a wheel drive whose status went stale while driving (its
    /// velocity downloads are unconfirmed, so a silent wheel can let go while
    /// the others keep driving): the loop STOPPED the platform and keeps this
    /// latched until that drive reports again. 0 = none.
    uint8_t lost_wheel_id{0};
    /// Age of the last set-point / stop frame the loop sent (65535 = none yet).
    uint16_t command_age_ms{65535};
  };

  /// Configuration for the controller.
  struct Config {
    HoloDeckPlatform &platform; ///< Wheel-speed kinematics.
    /// The wheel drives in pear::Corner order (FR, FL, BR, BL).
    // cppcheck-suppress uninitMemberVarNoCtor
    std::array<Ds402VelocityMotor, pear::kCornerCount> &motors;
    /// CAN node id for each wheel, in HoloDeckPlatform wheel order (TL, TR, BL,
    /// BR); each must be one of the motors' ids.
    std::array<size_t, 4> motor_ids_by_wheel{};
    /// True if main already ran enable_drive() on every drive: the first DRIVE
    /// tick then streams at once instead of re-enabling first.
    bool motors_armed{false};
    float max_wheel_rpm{0.0f};                               ///< Per-wheel scaling limit.
    float max_speed_mps{0.0f};                               ///< Initial translation limit.
    float max_rotation_rpm{0.0f};                            ///< Initial rotation-rate limit.
    float twist_rotation_scale{1.0f};                        ///< Initial joystick-twist scale.
    std::chrono::milliseconds control_period{20};            ///< Control loop period.
    std::chrono::milliseconds status_poll_period{250};       ///< Status poll period.
    std::chrono::milliseconds joystick_release_timeout{500}; ///< Centered time
                                                             ///< before GUI regains
                                                             ///< control.
    std::chrono::milliseconds status_stale_timeout{3000};    ///< Age before a motor
                                                             ///< status is stale.
    std::chrono::milliseconds rearm_retry_period{1000};      ///< Between failed
                                                             ///< re-enable attempts.
    uint32_t status_read_timeout_ms{20};                     ///< Per-status-read reply
                                                             ///< timeout; bounds how long
                                                             ///< a drive's mutex is held
                                                             ///< by a status poll.
    /// Transport hook: make sure no set-point the transport still has queued
    /// can execute after the stop frames that follow (drain them within a
    /// bound so the stop frames go out on an empty queue). Called on each
    /// transition into STOPPED / DISABLED, right before the stop frames.
    std::function<void()> flush_pending_commands{nullptr};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN};
  };

  explicit DriveController(const Config &config);
  ~DriveController();

  DriveController(const DriveController &) = delete;
  DriveController &operator=(const DriveController &) = delete;

  /// Set the GUI/CLI velocity setpoint in real units, in the PLATFORM frame:
  /// vx forward, vy RIGHT, w counter-clockwise (the normalized setters below
  /// take a left-positive input and convert). The translation vector is
  /// clamped to the max-speed limit and w to the max-rotation limit.
  void set_gui_velocity(float vx_mps, float vy_mps, float w_rpm);

  /// Feed the latest physical-joystick reading (normalized [-1, 1], already
  /// deadzoned: exactly 0 means centered). Any non-zero value makes the
  /// joystick the active source.
  void set_joystick_input(float forward, float left, float ccw);
  /// Zero the joystick input WITHOUT counting it as a joystick sample: for a
  /// watchdog whose stream has paused. Unlike set_joystick_input(0, 0, 0) it
  /// cannot satisfy the post-enable re-center guard, so a stream that resumes
  /// with the stick already deflected still has to center first.
  void hold_joystick();

  /// Enter DRIVE mode. Zeroes the GUI setpoint and requires the physical
  /// joystick to be re-centered before it can take over, so the platform never
  /// jumps or lurches on (re-)enable - even if a source was deflected.
  /// Enter DRIVE. Refused (false) while a linkage batch is executing: the
  /// interlock is two-sided (see set_linkage_moving), so base and linkage
  /// motion never overlap. The joystick must re-center before it takes over.
  bool enable();
  /// Install the linkage side of the interlock (LinkageController::moving).
  void set_linkage_moving(std::function<bool()> fn);
  /// E-STOP: enter STOPPED mode. The control loop streams zero velocity to
  /// every drive (they decelerate and hold at 0), overriding all sources.
  /// Always wins.
  void stop();
  /// Disable the drives at the DS402 level: one acknowledged zero, then the
  /// power stage off (shaft loose). Call enable() to resume driving (the loop
  /// re-enables the drives first).
  void disable_motors();
  /// True in DRIVE mode.
  bool is_enabled() const;

  /// Set the maximum translation speed, m/s (clamped to be positive).
  void set_max_speed(float max_speed_mps);
  /// Set the maximum chassis rotation rate, RPM (clamped to be positive).
  void set_max_rotation(float max_rotation_rpm);

  /// Get a snapshot of the current controller state. Thread-safe.
  State state() const;

protected:
  bool control_step();
  bool status_step();
  /// Stream zero velocity to every drive (STOPPED). The drives have their own
  /// locking. Does not log: the caller reports a failure on the transition
  /// (send_failing_), not every tick. \return true if every frame was queued.
  bool send_zeros();
  /// Halt every drive at the DS402 level (DISABLED): an acknowledged zero
  /// set-point, then disable_drive(). \return true only if every drive took it.
  bool send_disable();
  /// (Re-)enable every drive (after a DISABLED, or if main could not at boot).
  /// \return true only if every drive is enabled.
  bool arm_motors();
  /// How long a drive gets to acknowledge its stop. Only paid in DISABLED (and
  /// once on the transition into it), where the loop has nothing else to do.
  static constexpr uint32_t kStopAckTimeoutMs = 20;
  /// Run Config::flush_pending_commands once if a stop() / disable_motors()
  /// asked for it (before that transition's stop frames go out).
  void flush_transport_if_requested();
  void note_command_sent();
  static float clamp_positive(float value);

  HoloDeckPlatform &platform_;
  std::array<Ds402VelocityMotor, pear::kCornerCount> &motors_;
  std::array<size_t, 4> motor_index_by_wheel_{}; ///< motors_ index per platform wheel
  float max_wheel_rpm_;
  std::chrono::milliseconds joystick_release_timeout_;
  std::chrono::milliseconds status_stale_timeout_;
  std::chrono::milliseconds rearm_retry_period_;
  uint32_t status_read_timeout_ms_;
  std::function<void()> flush_pending_commands_; ///< Config::flush_pending_commands

  mutable std::mutex mutex_;
  Mode mode_{Mode::STOPPED};
  bool armed_{false};                             ///< the drives' power stages are on
  uint8_t lost_wheel_id_{0};                      ///< State::lost_wheel_id
  std::function<bool()> linkage_moving_{nullptr}; ///< the linkage batch guard
  /// Index of a wheel whose last successful status read is older than the
  /// stale timeout (a never-polled wheel counts from boot), or npos.
  size_t stale_wheel_locked(std::chrono::steady_clock::time_point now) const;
  bool disable_sent_{false};  ///< the one-shot motor stop for DISABLED was sent
  bool flush_pending_{false}; ///< a STOP / DISABLE landed: flush the transport before enacting it
  bool require_joystick_recenter_{false}; ///< ignore the joystick until it re-centers
  uint32_t joystick_sample_seq_{0};       ///< +1 per set_joystick_input()
  uint32_t recenter_after_seq_{0};        ///< the guard clears only on a LATER sample
  Source source_{Source::STOPPED};
  float gui_vx_mps_{0.0f};
  float gui_vy_mps_{0.0f};
  float gui_w_rpm_{0.0f};
  float joystick_forward_{0.0f};
  float joystick_left_{0.0f};
  float joystick_ccw_{0.0f};
  std::chrono::steady_clock::time_point last_joystick_activity_{};
  std::chrono::steady_clock::time_point next_arm_attempt_{};
  std::chrono::steady_clock::time_point last_command_{};
  bool have_command_{false};
  float max_speed_mps_;
  float max_rotation_rpm_;
  float twist_rotation_scale_{1.0f};
  // active (post-arbitration) command, written by the control loop
  float vx_mps_{0.0f};
  float vy_mps_{0.0f};
  float w_rpm_{0.0f};
  std::array<MotorStatus, pear::kCornerCount> motor_status_{};
  std::array<std::chrono::steady_clock::time_point, pear::kCornerCount> motor_status_time_{};
  bool send_failing_{false};

  size_t status_poll_index_{0};

  espp::Timer control_timer_;
  espp::Timer status_timer_;
};
