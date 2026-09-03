#pragma once

#include <array>

class HoloDeckPlatform {
public:
  struct WheelSpeeds {
    float top_left_rpm{0.0f};
    float top_right_rpm{0.0f};
    float bottom_left_rpm{0.0f};
    float bottom_right_rpm{0.0f};
  };

  struct Configuration {
    float wheel_diameter_m{0.0f};
    float half_length_m{0.0f};
    float half_width_m{0.0f};
    std::array<float, 4> wheel_angles_degrees{};
    std::array<float, 4> motor_directions{1.0f, 1.0f, 1.0f, 1.0f};
  };

  explicit HoloDeckPlatform(Configuration configuration);

  // x and y are linear speeds in m/s. w is the chassis rotation speed in RPM.
  WheelSpeeds calculate_wheel_speeds(float x_mps, float y_mps, float w_rpm) const;

private:
  Configuration configuration_;
};