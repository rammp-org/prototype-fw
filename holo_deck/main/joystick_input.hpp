#pragma once

#include <functional>

#include "base_component.hpp"
#include "joystick.hpp"
#include "oneshot_adc.hpp"
#include "range_mapper.hpp"
#include "timer.hpp"

#include "hw_config.hpp"

/// Samples the 3-axis analog joystick and its pushbutton (see hw_config.hpp
/// for the wiring) at a fixed rate and reports events via callbacks:
///
/// * X/Y (ADC1) go through an espp::Joystick configured as Type::CIRCULAR
///   with a circular center deadzone, so a centered stick reports exactly
///   (0, 0).
/// * The twist axis (ADC2 - a separate ADC unit, hence the second
///   OneshotAdc) goes through an espp::FloatRangeMapper with its own
///   deadband, so an untwisted knob reports exactly 0.
/// * The pushbutton is polled each sample and debounced (the level must be
///   stable for hw_config::kJoystickButtonDebounce); the button callback
///   fires once per accepted press.
///
/// The axis callback receives (forward, left, ccw), each in [-1, 1] using
/// the platform frame (x forward, y left, rotation counter-clockwise
/// positive).
class JoystickInput : public espp::BaseComponent {
public:
  /// Callback with the normalized (already deadzoned) joystick values.
  using callback_t = std::function<void(float forward, float left, float ccw)>;
  /// Callback fired once per debounced button press.
  using button_callback_t = std::function<void()>;

  /// Configuration for the JoystickInput.
  struct Config {
    callback_t callback;               ///< Called with each new sample. Required.
    button_callback_t button_callback; ///< Called on each debounced press. Optional.
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN};
  };

  explicit JoystickInput(const Config &config);

  JoystickInput(const JoystickInput &) = delete;
  JoystickInput &operator=(const JoystickInput &) = delete;

protected:
  bool update();
  void update_button();
  /// Boot-time auto-centering: average each spring-return axis at rest and,
  /// if plausible, adopt it as that axis's center so the deadzones are applied
  /// around the true resting point (see hw_config::kJoystickAutoCenter). Falls
  /// back to the nominal centers for any axis whose reading is implausible or
  /// cannot be read. Called once from the ctor before the sampling timer runs.
  void calibrate_center();
  /// (Re)apply the axis calibration with the given per-axis center voltages,
  /// deriving the twist deadband from kJoystickTwistDeadzoneFraction.
  void apply_calibration(float x_center_mv, float y_center_mv, float twist_center_mv);

  callback_t callback_;
  button_callback_t button_callback_;
  espp::OneshotAdc xy_adc_;
  espp::OneshotAdc twist_adc_;
  espp::AdcConfig x_channel_;
  espp::AdcConfig y_channel_;
  espp::AdcConfig twist_channel_;
  espp::Joystick joystick_;
  espp::FloatRangeMapper twist_mapper_;
  bool button_configured_{false};
  bool button_pressed_{false};      // debounced state
  bool button_candidate_{false};    // last raw state that differed
  size_t button_stable_samples_{0}; // consecutive samples of the candidate
  espp::Timer timer_;
};
