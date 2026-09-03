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
    , twist_mapper_(axis_calibration(hw_config::kJoystickTwistCenterMv,
                                     hw_config::kJoystickTwistMinMv, hw_config::kJoystickTwistMaxMv,
                                     hw_config::kJoystickTwistDeadbandMv,
                                     hw_config::kJoystickTwistInverted))
    , timer_({.name = "joystick",
              .period = hw_config::kJoystickPeriod,
              .callback = [this]() { return update(); },
              .auto_start = true,
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

  // map to the platform frame: joystick +x is right -> left is -x;
  // joystick +y is forward; twist counter-clockwise positive
  const float forward = joystick_.y();
  const float left = -joystick_.x();
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
