#include "joystick_input.hpp"

#include <algorithm>

namespace {
espp::FloatRangeMapper::Config axis_calibration(float center_mv, float min_mv, float max_mv,
                                                float deadband_mv, bool inverted) {
  return {
      .center = center_mv,
      .center_deadband = deadband_mv,
      .minimum = min_mv,
      .maximum = max_mv,
      .invert_output = inverted,
  };
}

// Rotate a stick vector CLOCKWISE by a multiple of 90 degrees to compensate for
// the joystick's physical mounting orientation. In the platform frame (x right,
// y forward) a 90-degree clockwise turn maps (x, y) -> (y, -x), so +y (forward)
// rotates onto +x (right). Applied to the already-deadzoned/normalized stick
// output; the circular deadzone is radius-based and thus rotation-invariant.
void rotate_stick_cw(float &x, float &y, int cw_deg) {
  const int quadrant = ((cw_deg / 90) % 4 + 4) % 4;
  const float ox = x;
  const float oy = y;
  switch (quadrant) {
  case 1: // 90 CW
    x = oy;
    y = -ox;
    break;
  case 2: // 180
    x = -ox;
    y = -oy;
    break;
  case 3: // 270 CW
    x = -oy;
    y = ox;
    break;
  default: // 0
    break;
  }
}
} // namespace

JoystickInput::JoystickInput(const Config &config)
    : espp::BaseComponent("JoystickInput", config.log_level)
    , callback_(config.callback)
    , button_callback_(config.button_callback)
    , xy_adc_({.unit = hw_config::kJoystickXYAdcUnit,
               .channels = {{.unit = hw_config::kJoystickXYAdcUnit,
                             .channel = hw_config::kJoystickXAdcChannel,
                             .attenuation = hw_config::kJoystickAdcAttenuation},
                            {.unit = hw_config::kJoystickXYAdcUnit,
                             .channel = hw_config::kJoystickYAdcChannel,
                             .attenuation = hw_config::kJoystickAdcAttenuation}},
               .log_level = config.log_level})
    , twist_adc_({.unit = hw_config::kJoystickTwistAdcUnit,
                  .channels = {{.unit = hw_config::kJoystickTwistAdcUnit,
                                .channel = hw_config::kJoystickTwistAdcChannel,
                                .attenuation = hw_config::kJoystickAdcAttenuation}},
                  .log_level = config.log_level})
    , x_channel_({.unit = hw_config::kJoystickXYAdcUnit,
                  .channel = hw_config::kJoystickXAdcChannel,
                  .attenuation = hw_config::kJoystickAdcAttenuation})
    , y_channel_({.unit = hw_config::kJoystickXYAdcUnit,
                  .channel = hw_config::kJoystickYAdcChannel,
                  .attenuation = hw_config::kJoystickAdcAttenuation})
    , twist_channel_({.unit = hw_config::kJoystickTwistAdcUnit,
                      .channel = hw_config::kJoystickTwistAdcChannel,
                      .attenuation = hw_config::kJoystickAdcAttenuation})
    // for a CIRCULAR joystick the per-axis deadbands are 0 and the deadzone
    // is applied on the vector, via center_deadzone_radius / range_deadzone
    , joystick_({.x_calibration = axis_calibration(
                     hw_config::kJoystickXCenterMv, hw_config::kJoystickXMinMv,
                     hw_config::kJoystickXMaxMv, 0.0f, hw_config::kJoystickXInverted),
                 .y_calibration = axis_calibration(
                     hw_config::kJoystickYCenterMv, hw_config::kJoystickYMinMv,
                     hw_config::kJoystickYMaxMv, 0.0f, hw_config::kJoystickYInverted),
                 .type = espp::Joystick::Type::CIRCULAR,
                 .center_deadzone_radius = hw_config::kJoystickCenterDeadzoneRadius,
                 .range_deadzone = hw_config::kJoystickRangeDeadzone,
                 .log_level = config.log_level})
    // placeholder mapper; calibrate_center() -> apply_calibration() sets the
    // real center + deadband (from kJoystickTwistDeadzoneFraction) before the
    // sampling timer starts
    , twist_mapper_(axis_calibration(hw_config::kJoystickTwistCenterMv,
                                     hw_config::kJoystickTwistMinMv, hw_config::kJoystickTwistMaxMv,
                                     0.0f, hw_config::kJoystickTwistInverted))
    // auto_start is FALSE: the sampling timer is started at the end of the ctor
    // body, AFTER boot-time auto-centering, so update() never runs against an
    // un-centered calibration.
    , timer_({.name = "joystick",
              .period = hw_config::kJoystickPeriod,
              .callback = [this]() { return update(); },
              .auto_start = false,
              .stack_size_bytes = 6 * 1024,
              .priority = 8}) {
  // the pushbutton: input with the internal pull-up, active-low (see
  // hw_config.hpp)
  const gpio_config_t button_config = {
      .pin_bit_mask = 1ULL << hw_config::kJoystickButtonGpio,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
      // input hysteresis helps reject noise on the (debounced) button line
      .hys_ctrl_mode = GPIO_HYS_SOFT_ENABLE,
  };
  const auto err = gpio_config(&button_config);
  if (err != ESP_OK) {
    logger_.error("Failed to configure joystick button GPIO {}: {}",
                  static_cast<int>(hw_config::kJoystickButtonGpio), esp_err_to_name(err));
  } else {
    button_configured_ = true;
  }

  // Capture the spring-return resting center of each axis, then start sampling.
  calibrate_center();
  timer_.start();
}

void JoystickInput::apply_calibration(float x_center_mv, float y_center_mv, float twist_center_mv) {
  // X/Y: per-axis deadband 0 - the circular deadzone is applied on the vector
  // by the Joystick (radius/range preserved here).
  joystick_.set_calibration(
      axis_calibration(x_center_mv, hw_config::kJoystickXMinMv, hw_config::kJoystickXMaxMv, 0.0f,
                       hw_config::kJoystickXInverted),
      axis_calibration(y_center_mv, hw_config::kJoystickYMinMv, hw_config::kJoystickYMaxMv, 0.0f,
                       hw_config::kJoystickYInverted),
      hw_config::kJoystickCenterDeadzoneRadius, hw_config::kJoystickRangeDeadzone);
  // Twist: deadband is a fraction of the (possibly asymmetric) half-span
  // around the captured center.
  const float twist_half_span = std::min(twist_center_mv - hw_config::kJoystickTwistMinMv,
                                         hw_config::kJoystickTwistMaxMv - twist_center_mv);
  const float twist_deadband =
      std::max(0.0f, twist_half_span) * hw_config::kJoystickTwistDeadzoneFraction;
  twist_mapper_ = espp::FloatRangeMapper(
      axis_calibration(twist_center_mv, hw_config::kJoystickTwistMinMv,
                       hw_config::kJoystickTwistMaxMv, twist_deadband,
                       hw_config::kJoystickTwistInverted));
}

void JoystickInput::calibrate_center() {
  // Nominal (configured) centers are the fallback.
  float x_center = hw_config::kJoystickXCenterMv;
  float y_center = hw_config::kJoystickYCenterMv;
  float twist_center = hw_config::kJoystickTwistCenterMv;

  if (hw_config::kJoystickAutoCenter) {
    double x_sum = 0.0, y_sum = 0.0, t_sum = 0.0;
    size_t n = 0;
    for (size_t i = 0; i < hw_config::kJoystickAutoCenterSamples; ++i) {
      const auto x = xy_adc_.read_mv(x_channel_);
      const auto y = xy_adc_.read_mv(y_channel_);
      const auto t = twist_adc_.read_mv(twist_channel_);
      if (x.has_value() && y.has_value() && t.has_value()) {
        x_sum += x.value();
        y_sum += y.value();
        t_sum += t.value();
        ++n;
      }
    }
    if (n == 0) {
      logger_.warn("Auto-center: no valid ADC reads; using nominal centers");
    } else {
      // Adopt each measured center only if it is plausibly near mid-scale
      // (guards against a stick that was NOT centered at boot).
      const auto adopt = [this](float measured, float nominal, const char *axis) {
        if (std::abs(measured - nominal) <= hw_config::kJoystickAutoCenterMaxDeviationMv) {
          return measured;
        }
        logger_.warn("Auto-center: {} axis center {:.0f} mV implausible (nominal {:.0f}); "
                     "keeping nominal - was the stick centered at boot?",
                     axis, measured, nominal);
        return nominal;
      };
      x_center = adopt(static_cast<float>(x_sum / n), hw_config::kJoystickXCenterMv, "X");
      y_center = adopt(static_cast<float>(y_sum / n), hw_config::kJoystickYCenterMv, "Y");
      twist_center =
          adopt(static_cast<float>(t_sum / n), hw_config::kJoystickTwistCenterMv, "twist");
      logger_.info("Auto-centered: x={:.0f} y={:.0f} twist={:.0f} mV", x_center, y_center,
                   twist_center);
    }
  }
  apply_calibration(x_center, y_center, twist_center);
}

bool JoystickInput::update() {
  update_button();

  const auto x_mv = xy_adc_.read_mv(x_channel_);
  const auto y_mv = xy_adc_.read_mv(y_channel_);
  const auto twist_mv = twist_adc_.read_mv(twist_channel_);
  if (!x_mv.has_value() || !y_mv.has_value() || !twist_mv.has_value()) {
    logger_.warn("Failed to read joystick ADC channels");
    return false; // keep the timer running
  }
  logger_.debug("raw mV: x={} y={} twist={}", x_mv.value(), y_mv.value(), twist_mv.value());

  joystick_.update(static_cast<float>(x_mv.value()), static_cast<float>(y_mv.value()));
  const float twist =
      std::clamp(twist_mapper_.map(static_cast<float>(twist_mv.value())), -1.0f, 1.0f);

  // Compensate for the stick's mounting orientation, then map to the platform
  // frame: joystick +x is right -> left is -x; joystick +y is forward; twist
  // counter-clockwise positive.
  float sx = joystick_.x();
  float sy = joystick_.y();
  rotate_stick_cw(sx, sy, hw_config::kJoystickMountingRotationCwDeg);
  const float forward = sy;
  const float left = -sx;
  if (callback_) {
    callback_(forward, left, twist);
  }
  return false; // keep the timer running
}

void JoystickInput::update_button() {
  if (!button_configured_) {
    return;
  }
  const bool raw_pressed =
      gpio_get_level(hw_config::kJoystickButtonGpio) == hw_config::kJoystickButtonActiveLevel;
  if (raw_pressed == button_pressed_) {
    // stable in the accepted state; restart any pending debounce
    button_stable_samples_ = 0;
    return;
  }
  if (raw_pressed != button_candidate_) {
    button_candidate_ = raw_pressed;
    button_stable_samples_ = 0;
  }
  button_stable_samples_++;
  // accept the new state once it has been stable for the debounce time
  static constexpr size_t kSamplesNeeded = std::max<size_t>(
      1, hw_config::kJoystickButtonDebounce.count() / hw_config::kJoystickPeriod.count());
  if (button_stable_samples_ < kSamplesNeeded) {
    return;
  }
  button_pressed_ = raw_pressed;
  button_stable_samples_ = 0;
  if (button_pressed_) {
    logger_.info("Joystick button pressed");
    if (button_callback_) {
      button_callback_();
    }
  }
}
