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
/// zeroed on the handover so the platform never jumps. E-stop always wins.
///
/// All public setters and state() are thread-safe.
class HoloDeckController : public espp::BaseComponent {
public:
  /// The currently-active control source.
  enum class Source {
    STOPPED,  ///< E-stopped / disabled; motors are not being commanded.
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
    bool enabled{false};
    Source source{Source::STOPPED};
    float vx_mps{0.0f};                ///< Active commanded forward velocity.
    float vy_mps{0.0f};                ///< Active commanded leftward velocity.
    float w_rpm{0.0f};                 ///< Active commanded CCW rotation rate.
    float gui_vx_mps{0.0f};            ///< GUI/CLI setpoint (pre-arbitration).
    float gui_vy_mps{0.0f};            ///< GUI/CLI setpoint (pre-arbitration).
    float gui_w_rpm{0.0f};             ///< GUI/CLI setpoint (pre-arbitration).
    float max_speed_mps{0.0f};         ///< Current translation limit.
    float max_rotation_rpm{0.0f};      ///< Current rotation-rate limit.
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
    std::chrono::milliseconds control_period;           ///< Control loop period.
    std::chrono::milliseconds status_poll_period;       ///< Status poll period.
    std::chrono::milliseconds joystick_release_timeout; ///< Centered time
                                                        ///< before GUI regains
                                                        ///< control.
    std::chrono::milliseconds status_stale_timeout;     ///< Age before a motor
                                                        ///< status is stale.
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

  /// Enable the platform, or e-stop it (false). Enabling zeroes the GUI
  /// setpoint so the platform never jumps on re-enable. Disabling sends a
  /// single zero-velocity command to every motor.
  void set_enabled(bool enabled);
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
  void send_zeros_unlocked();
  static float clamp_positive(float value);

  HoloDeckPlatform &platform_;
  std::array<MotorActuator, 4> &motors_;
  std::array<size_t, 4> motor_ids_by_wheel_;
  float max_wheel_rpm_;
  std::chrono::milliseconds joystick_release_timeout_;
  std::chrono::milliseconds status_stale_timeout_;

  mutable std::mutex mutex_;
  bool enabled_{false};
  bool stop_sent_{false};
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
