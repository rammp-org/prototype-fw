#pragma once

#include <array>
#include <cstdint>

#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "logger.hpp"

class MotorActuator {
public:
  struct Status {
    std::array<uint8_t, 8> data{};
    int temperature_c{0};
    int16_t torque_raw{0};
    int16_t velocity_raw{0};
    int32_t angle_raw{0};
  };

  MotorActuator(espp::Logger &logger, gpio_num_t rx_gpio, gpio_num_t tx_gpio);
  ~MotorActuator();

  MotorActuator(const MotorActuator &) = delete;
  MotorActuator &operator=(const MotorActuator &) = delete;

  bool start(uint32_t bitrate = 1'000'000);
  bool is_connected(uint32_t timeout_ms = 100);

  bool read_status(Status &status, uint32_t timeout_ms = 100);
  // Unsupported by the current motor firmware: command 0xB5 gets no response.
  bool read_motor_model(std::array<char, 8> &model, uint32_t timeout_ms = 100);
  // Unsupported by the current motor firmware: command 0xB2 gets no response.
  bool read_software_version_date(uint32_t &version_date, uint32_t timeout_ms = 100);
  bool read_multi_turn_position(int32_t &position_raw, uint32_t timeout_ms = 100);
  // Unsupported by the current motor firmware: command 0x61 gets no response.
  bool read_multi_turn_raw_position(int32_t &position_raw, uint32_t timeout_ms = 100);
  // Unsupported by the current motor firmware: command 0x62 gets no response.
  bool read_multi_turn_zero_offset(int32_t &offset_raw, uint32_t timeout_ms = 100);
  bool read_temperature(int &temperature_c, uint32_t timeout_ms = 100);
  bool read_angle(int32_t &angle_raw, uint32_t timeout_ms = 100);
  bool read_velocity(int16_t &velocity_raw, uint32_t timeout_ms = 100);
  bool read_torque(int16_t &torque_raw, uint32_t timeout_ms = 100);

  bool send_torque(int16_t torque_raw);
  bool send_velocity(int32_t velocity_raw);
  bool zero_position(uint32_t timeout_ms = 100);
  bool write_multi_turn_zero_offset(int32_t offset_raw, uint32_t timeout_ms = 100);
  bool write_current_position_as_zero(uint32_t timeout_ms = 100);
  bool set_position(int32_t position_centidegrees, uint16_t max_speed_dps);
  bool send_incremental_position(int32_t delta_centidegrees, uint16_t max_speed_dps);
  bool stop();
  bool disable();
  bool hold();
  bool release_brake();
  bool lock_brake();
  bool write_default_pid_rom();
  bool write_default_pid_ram();
  bool write_global_pid_gain(float gain);

private:
  struct CanPacket {
    uint32_t id{0};
    uint16_t length{0};
    bool extended{false};
    std::array<uint8_t, 8> data{};
  };

  static constexpr uint32_t motor_can_id_ = 0x141;
  static constexpr uint32_t motor_reply_can_id_ = 0x241;
  static constexpr size_t packet_length_ = 8;

  static bool on_receive(twai_node_handle_t handle, const twai_rx_done_event_data_t *event,
                         void *context);
  bool send_command(const std::array<uint8_t, packet_length_> &command);
  bool request(uint8_t command_code, CanPacket &response, uint32_t timeout_ms);
  static int16_t read_i16(const std::array<uint8_t, 8> &data, size_t index);
  static int32_t read_i32(const std::array<uint8_t, 8> &data, size_t index);
  static void set_i16(std::array<uint8_t, 8> &data, size_t index, int16_t value);
  static void set_i32(std::array<uint8_t, 8> &data, size_t index, int32_t value);

  espp::Logger &logger_;
  gpio_num_t rx_gpio_;
  gpio_num_t tx_gpio_;
  twai_node_handle_t node_{nullptr};
  QueueHandle_t receive_queue_{nullptr};
};
