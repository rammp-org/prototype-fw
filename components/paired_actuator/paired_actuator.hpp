#pragma once

#include <array>
#include <cstdint>

#include "base_component.hpp"
#include "motor_actuator.hpp"

class PairedActuator : public espp::BaseComponent {
public:
  struct Status {
    std::array<MotorActuator::Status, 2> motors{};
  };

  PairedActuator(MotorActuator &primary, MotorActuator &secondary);

  PairedActuator(const PairedActuator &) = delete;
  PairedActuator &operator=(const PairedActuator &) = delete;

  bool read_status(Status &status, uint32_t timeout_ms = 100);
  void zero_position();
  // Index 0 is the primary actuator; index 1 is the secondary actuator.
  std::array<float, 2> get_position() const;
  bool set_position_limits(float minimum_degrees = -45.0f, float maximum_degrees = 45.0f);
  bool set_position(float position_degrees, float max_speed_rpm);
  bool stop();
  bool release_brake();
  bool lock_brake();

private:
  MotorActuator &primary_;
  MotorActuator &secondary_;
};
