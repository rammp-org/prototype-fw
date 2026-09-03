#include "holo_deck_platform.hpp"

#include <cmath>

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadiansPerDegree = kPi / 180.0f;
constexpr float kRadPerSecPerRpm = 2.0f * kPi / 60.0f;
constexpr float kWheelCircumferenceFactor = kPi;
}

HoloDeckPlatform::HoloDeckPlatform(Configuration configuration)
    : configuration_(configuration) {}

HoloDeckPlatform::WheelSpeeds HoloDeckPlatform::calculate_wheel_speeds(float x_mps,
                                                                         float y_mps,
                                                                         float w_rpm) const {
  if (configuration_.wheel_diameter_m <= 0.0f) {
    return {};
  }

  constexpr std::array<float, 4> x_positions = {1.0f, 1.0f, -1.0f, -1.0f};
  constexpr std::array<float, 4> y_positions = {1.0f, -1.0f, 1.0f, -1.0f};
  const float angular_velocity_rad_per_second = w_rpm * kRadPerSecPerRpm;
  const float wheel_circumference_m = kWheelCircumferenceFactor * configuration_.wheel_diameter_m;
  std::array<float, 4> wheel_rpm{};

  for (size_t index = 0; index < wheel_rpm.size(); ++index) {
    const float wheel_x_m = x_positions[index] * configuration_.half_length_m;
    const float wheel_y_m = y_positions[index] * configuration_.half_width_m;
    const float angle_rad = configuration_.wheel_angles_degrees[index] * kRadiansPerDegree;
    const float drive_x = std::cos(angle_rad);
    const float drive_y = std::sin(angle_rad);
    const float rotational_x_mps = -angular_velocity_rad_per_second * wheel_y_m;
    const float rotational_y_mps = angular_velocity_rad_per_second * wheel_x_m;
    const float wheel_speed_mps = drive_x * (x_mps + rotational_x_mps) +
                                  drive_y * (y_mps + rotational_y_mps);
    wheel_rpm[index] = configuration_.motor_directions[index] *
                        wheel_speed_mps * 60.0f / wheel_circumference_m;
  }

  return {
      .top_left_rpm = wheel_rpm[0],
      .top_right_rpm = wheel_rpm[1],
      .bottom_left_rpm = wheel_rpm[2],
      .bottom_right_rpm = wheel_rpm[3],
  };
}