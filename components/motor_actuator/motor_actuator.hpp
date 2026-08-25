#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <utility>

#include "base_component.hpp"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "logger.hpp"

struct MotorPacket {
  uint32_t id{0}; // Logical motor ID; each transport maps it to its bus address.
  uint16_t length{0};
  std::array<uint8_t, 8> data{};
};

class MotorCanBus : public espp::BaseComponent {
public:
  MotorCanBus(gpio_num_t rx_gpio, gpio_num_t tx_gpio);
  ~MotorCanBus();

  MotorCanBus(const MotorCanBus &) = delete;
  MotorCanBus &operator=(const MotorCanBus &) = delete;

  bool start(uint32_t bitrate = 1'000'000);
  bool send(const MotorPacket &command);
  bool request(const MotorPacket &command, MotorPacket &response, uint32_t timeout_ms = 100);

private:
  static bool on_receive(twai_node_handle_t handle, const twai_rx_done_event_data_t *event,
                         void *context);

  gpio_num_t rx_gpio_;
  gpio_num_t tx_gpio_;
  twai_node_handle_t node_{nullptr};
  QueueHandle_t receive_queue_{nullptr};
};

class MotorActuator : public espp::BaseComponent {
public:
  using CommunicationFunction =
      std::function<bool(const MotorPacket &command, MotorPacket &response,
               uint32_t timeout_ms)>;

  struct Status {
    std::array<uint8_t, 8> data{};
    int temperature_c{0};
    int16_t torque_raw{0};
    float velocity_rpm{0.0f};
    float angle_degrees{0.0f};
  };

  MotorActuator(CommunicationFunction communication, uint8_t motor_id);

  MotorActuator(const MotorActuator &) = delete;
  MotorActuator &operator=(const MotorActuator &) = delete;

  bool is_connected(uint32_t timeout_ms = 100);

  bool read_status(Status &status, uint32_t timeout_ms = 100);
  bool read_motor_id(uint8_t &motor_id, uint32_t timeout_ms = 100);
  bool write_motor_id(uint8_t motor_id, uint32_t timeout_ms = 100);
  bool read_temperature(int &temperature_c, uint32_t timeout_ms = 100);
  bool read_angle(float &angle_degrees, uint32_t timeout_ms = 100);
  bool read_velocity(float &velocity_rpm, uint32_t timeout_ms = 100);
  bool read_torque(int16_t &torque_raw, uint32_t timeout_ms = 100);

  bool send_torque(int16_t torque_raw);
  bool send_velocity(float velocity_rpm);
  // Reset the software-only position reference; this does not write motor ROM.
  void zero_position();
  bool set_position(float position_degrees, float max_speed_rpm);
  bool send_incremental_position(float delta_degrees, float max_speed_rpm);
  bool stop();
  bool disable();
  bool hold();
  bool release_brake();
  bool lock_brake();
  bool write_default_pid_rom();
  bool write_default_pid_ram();
  bool write_global_pid_gain(float gain);
  uint8_t get_motor_id() const { return motor_id_; }

private:
  static constexpr size_t packet_length_ = 8;
  static constexpr float gear_ratio_ = 36.0f;

  bool send_command(const std::array<uint8_t, packet_length_> &command);
  bool request(uint8_t command_code, MotorPacket &response, uint32_t timeout_ms);
  bool request(const MotorPacket &command, MotorPacket &response, uint32_t timeout_ms);
  static int16_t read_i16(const std::array<uint8_t, 8> &data, size_t index);
  static int32_t read_i32(const std::array<uint8_t, 8> &data, size_t index);
  static void set_i16(std::array<uint8_t, 8> &data, size_t index, int16_t value);
  static void set_i32(std::array<uint8_t, 8> &data, size_t index, int32_t value);

  CommunicationFunction communication_;
  uint8_t motor_id_;
  float virtual_position_degrees_{0.0f};
};
