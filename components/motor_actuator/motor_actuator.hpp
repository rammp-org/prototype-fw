#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <utility>

#include "base_component.hpp"
#include "can_bus.hpp"
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
  /// \brief Attach the MotorActuator protocol adapter to a shared CAN interface.
  /// \details Construct all protocol adapters before starting \p bus so they can
  ///          register their independent receive queues.
  explicit MotorCanBus(CanBus &bus);
  ~MotorCanBus();

  MotorCanBus(const MotorCanBus &) = delete;
  MotorCanBus &operator=(const MotorCanBus &) = delete;

  bool start(uint32_t bitrate = 1'000'000);
  bool send(const MotorPacket &command);
  bool request(const MotorPacket &command, MotorPacket &response, uint32_t timeout_ms = 100);
  // Drain every frame still queued in the driver (bounded: single-shot
  // transmit empties the 4-frame backlog within ~1 ms), for a mode transition
  // that must not let an older set-point execute after the stop frames that
  // follow: those then go out on an empty queue. Returns false if the queue
  // did not drain in time (a wedged controller).
  bool flush_pending();

private:
  bool register_receivers();
  // Enqueue one frame with transaction_mutex_ already held. Never waits on wire
  // completion: CanBus::send() copies the frame into its persistent TX ring
  // (the on-chip TWAI TX queue is zero-copy) and never blocks a real-time
  // control loop on an ACK that may never arrive. wait_for_queue_space=true
  // (request path) briefly waits for queue space so a poll is not dropped;
  // false (fire-and-forget set-points) enqueues non-blocking and drops the
  // stale frame if the queue is full - the next control tick supersedes it.
  bool send_unlocked(const MotorPacket &command, bool wait_for_queue_space = true);

  std::unique_ptr<CanBus> owned_bus_;
  CanBus *bus_{nullptr};
  QueueHandle_t receive_queue_{nullptr};
  std::mutex transaction_mutex_;
  bool receivers_registered_{false};
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
  // If need_align is true, also send an incremental move to align to the current
  // physical angle before zeroing the virtual position.
  void zero_position(bool need_align = false);
  float get_position() const { return virtual_position_degrees_; }
  bool set_position_limits(float minimum_degrees, float maximum_degrees);
  bool set_position(float position_degrees, float max_speed_rpm);
  bool send_incremental_position(float delta_degrees, float max_speed_rpm);
  bool stop();
  /// Stop the motor (0x81) and wait for the motor's own acknowledgement: the
  /// RMD echoes the command frame from its reply id. Unlike stop(), which only
  /// reports that the frame left (queued, or completed on the bus, depending on
  /// the transport), this returns true only when THIS motor answered, so a
  /// control-level disable can retry until every motor has really stopped.
  bool stop_acknowledged(uint32_t timeout_ms = 20);
  bool disable();
  bool hold();
  bool reset_system();
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
  float minimum_position_degrees_{-std::numeric_limits<float>::infinity()};
  float maximum_position_degrees_{std::numeric_limits<float>::infinity()};
};
