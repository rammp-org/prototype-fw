#pragma once

#include <array>
#include <chrono>
#include <mutex>

#include "base_component.hpp"
#include "timer.hpp"

#include "holo_deck_platform.hpp"
#include "motor_actuator.hpp"

/// The HoloDeckController owns the platform command state and the fixed-rate
/// control loop:
///
/// * It keeps a velocity setpoint (vx m/s forward, vy m/s left, w RPM
///   counter-clockwise) per control source (GUI/CLI vs. the physical
///   joystick), an enable/e-stop flag, and runtime-adjustable translation /
///   rotation limits.
/// * A 50 Hz espp::Timer reads the active source's setpoint, computes the
///   four wheel speeds via HoloDeckPlatform, scales them so no wheel exceeds
///   the configured wheel-RPM limit, and sends the velocity commands over the
///   CAN bus. On the enabled -> disabled transition it sends zeros exactly
///   once, then stops commanding the motors.
/// * A slower timer polls one motor's status per tick (round-robin) for the
///   GUI / CLI status display.
///
/// Source arbitration: any joystick deflection beyond the deadzone makes the
/// joystick the active source. Once the joystick has been centered for longer
/// than the release timeout, control falls back to the GUI setpoint, which is
/// zeroed on the handover so the platform never jumps. STOP always wins.
///
/// IMPORTANT: every motor command is issued from the single control loop, so a
/// STOP can never be overtaken by an in-flight drive command from another
/// thread (the bug where hitting STOP with the joystick deflected left the
/// motors driving). The setters only change the mode; the next control tick
/// (<= one control period) enacts it.
///
/// All public setters and state() are thread-safe.
class HoloDeckController : public espp::BaseComponent {
public:
  /// The controller's operating mode.
  enum class Mode {
    DRIVE,    ///< The active source drives the platform (normal operation).
    STOPPED,  ///< E-stop: the loop actively commands zero velocity (motors
              ///< hold at 0 in closed-loop). Overrides all sources.
    DISABLED, ///< Motors stopped at the CONTROL level (MotorActuator::stop()),
              ///< NOT held at zero by the velocity loop. Re-enable to drive.
  };

  /// The currently-active control source (meaningful only in DRIVE mode).
  enum class Source {
    STOPPED,  ///< E-stopped / disabled; motors are not being driven.
    GUI,      ///< The touch GUI / CLI setpoint drives the platform.
    JOYSTICK, ///< The physical joystick drives the platform.
  };

  /// Last known status of one motor.
  struct MotorStatus {
    uint8_t id{0};             ///< Motor CAN id.
    float velocity_rpm{0.0f};  ///< Last reported velocity.
    int temperature_c{0};      ///< Last reported temperature.
    float commanded_rpm{0.0f}; ///< Last commanded wheel speed.
    bool valid{false};         ///< True once a status read has succeeded.
    bool stale{true};          ///< True if not successfully read recently.
  };

  /// Snapshot of the full controller state, for the GUI / CLI.
  struct State {
    Mode mode{Mode::STOPPED};          ///< Operating mode.
    bool enabled{false};               ///< Convenience: mode == DRIVE.
    Source source{Source::STOPPED};
    float vx_mps{0.0f};                ///< Active commanded forward velocity.
    float vy_mps{0.0f};                ///< Active commanded leftward velocity.
    float w_rpm{0.0f};                 ///< Active commanded CCW rotation rate.
    float gui_vx_mps{0.0f};            ///< GUI/CLI setpoint (pre-arbitration).
    float gui_vy_mps{0.0f};            ///< GUI/CLI setpoint (pre-arbitration).
    float gui_w_rpm{0.0f};             ///< GUI/CLI setpoint (pre-arbitration).
    float max_speed_mps{0.0f};         ///< Current translation limit.
    float max_rotation_rpm{0.0f};      ///< Current rotation-rate limit.
    float twist_rotation_scale{1.0f};  ///< Joystick-twist rotation scale.
    float max_wheel_rpm{0.0f};         ///< Per-wheel scaling limit.
    std::array<MotorStatus, 4> motors; ///< Indexed by motor id - 1.
  };

  /// Configuration for the controller.
  struct Config {
    HoloDeckPlatform &platform;                         ///< Wheel-speed kinematics.
    std::array<MotorActuator, 4> &motors;               ///< Motors, indexed by id - 1.
    std::array<size_t, 4> motor_ids_by_wheel;           ///< Motor id for each wheel, in
                                                        ///< HoloDeckPlatform wheel order
                                                        ///< (TL, TR, BL, BR).
    float max_wheel_rpm;                                ///< Per-wheel scaling limit.
    float max_speed_mps;                                ///< Initial translation limit.
    float max_rotation_rpm;                             ///< Initial rotation-rate limit.
    float twist_rotation_scale{1.0f};                   ///< Initial joystick-twist scale.
    std::chrono::milliseconds control_period;           ///< Control loop period.
    std::chrono::milliseconds status_poll_period;       ///< Status poll period.
    std::chrono::milliseconds joystick_release_timeout; ///< Centered time
                                                        ///< before GUI regains
                                                        ///< control.
    std::chrono::milliseconds status_stale_timeout;     ///< Age before a motor
                                                        ///< status is stale.
    uint32_t status_read_timeout_ms;                    ///< Per-status-read reply
                                                        ///< timeout; bounds how long
                                                        ///< the shared CAN mutex is
                                                        ///< held by a status poll.
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN};
  };

  explicit HoloDeckController(const Config &config);
  ~HoloDeckController();

  HoloDeckController(const HoloDeckController &) = delete;
  HoloDeckController &operator=(const HoloDeckController &) = delete;

  /// Set the GUI/CLI velocity setpoint in real units. The translation vector
  /// is clamped to the max-speed limit and w to the max-rotation limit.
  void set_gui_velocity(float vx_mps, float vy_mps, float w_rpm);

  /// Set only the translation part of the GUI setpoint from normalized
  /// [-1, 1] inputs (multiplied by the max-speed limit); the rotation part is
  /// left unchanged.
  void set_gui_translation_normalized(float forward, float left);

  /// Set only the rotation part of the GUI setpoint from a normalized
  /// [-1, 1] input (multiplied by the max-rotation limit); the translation
  /// part is left unchanged.
  void set_gui_rotation_normalized(float ccw);

  /// Feed the latest physical-joystick reading (normalized [-1, 1], already
  /// deadzoned: exactly 0 means centered). Any non-zero value makes the
  /// joystick the active source.
  void set_joystick_input(float forward, float left, float ccw);

  /// Enter DRIVE mode. Zeroes the GUI setpoint and requires the physical
  /// joystick to be re-centered before it can take over, so the platform never
  /// jumps or lurches on (re-)enable - even if a source was deflected.
  void enable();
  /// E-STOP: enter STOPPED mode. The control loop actively commands zero
  /// velocity to every motor (they decelerate and hold at 0), overriding all
  /// sources. Always wins.
  void stop();
  /// Disable the motors at the CONTROL level (MotorActuator::stop()): the
  /// velocity loop stops commanding them entirely and the motors are halted by
  /// the motor controllers themselves, not held at zero by the loop. Call
  /// enable() to resume driving.
  void disable_motors();
  /// Compatibility shim: true -> enable(), false -> stop() (e-stop).
  void set_enabled(bool enabled);
  /// True in DRIVE mode.
  bool is_enabled() const;
  /// The current operating mode.
  Mode mode() const;

  /// Set the maximum translation speed, m/s (clamped to be positive).
  void set_max_speed(float max_speed_mps);
  /// Set the maximum chassis rotation rate, RPM (clamped to be positive).
  void set_max_rotation(float max_rotation_rpm);
  /// Set the scale factor applied to the rotation rate commanded from the
  /// JOYSTICK TWIST axis (twist * scale * max-rotation). Values > 1 let the
  /// twist command exceed the nominal rotation limit to compensate the
  /// platform's weak rotation authority; wheel commands remain bounded by
  /// max_wheel_rpm regardless (clamped to be positive).
  void set_twist_rotation_scale(float scale);

  /// Get a snapshot of the current controller state. Thread-safe.
  State state() const;

protected:
  bool control_step();
  bool status_step();
  /// Send zero velocity to every motor (STOPPED). Safe to call without holding
  /// mutex_ - the CAN transport has its own locking.
  void send_zeros();
  /// Halt every motor at the control level via MotorActuator::stop() (DISABLED).
  void send_disable();
  static float clamp_positive(float value);

  HoloDeckPlatform &platform_;
  std::array<MotorActuator, 4> &motors_;
  std::array<size_t, 4> motor_ids_by_wheel_;
  float max_wheel_rpm_;
  std::chrono::milliseconds joystick_release_timeout_;
  std::chrono::milliseconds status_stale_timeout_;
  uint32_t status_read_timeout_ms_;

  mutable std::mutex mutex_;
  Mode mode_{Mode::STOPPED};
  bool disable_sent_{false};             ///< the one-shot motor stop for DISABLED was sent
  bool require_joystick_recenter_{false}; ///< ignore the joystick until it re-centers
  Source source_{Source::STOPPED};
  float gui_vx_mps_{0.0f};
  float gui_vy_mps_{0.0f};
  float gui_w_rpm_{0.0f};
  float joystick_forward_{0.0f};
  float joystick_left_{0.0f};
  float joystick_ccw_{0.0f};
  std::chrono::steady_clock::time_point last_joystick_activity_{};
  float max_speed_mps_;
  float max_rotation_rpm_;
  float twist_rotation_scale_{1.0f};
  // active (post-arbitration) command, written by the control loop
  float vx_mps_{0.0f};
  float vy_mps_{0.0f};
  float w_rpm_{0.0f};
  std::array<MotorStatus, 4> motor_status_{};
  std::array<std::chrono::steady_clock::time_point, 4> motor_status_time_{};
  bool send_failing_{false};

  size_t status_poll_index_{0};

  espp::Timer control_timer_;
  espp::Timer status_timer_;
};
