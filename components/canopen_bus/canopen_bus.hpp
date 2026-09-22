#pragma once

#include <array>
#include <cstdint>
#include <mutex>

#include "base_component.hpp"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

struct CanopenFrame {
  uint32_t id{0};
  bool extended{false};
  bool rtr{false};
  uint8_t dlc{0};
  std::array<uint8_t, 8> data{};
};

class CanopenBus : public espp::BaseComponent {
public:
  CanopenBus(gpio_num_t rx_gpio, gpio_num_t tx_gpio);
  ~CanopenBus();

  CanopenBus(const CanopenBus &) = delete;
  CanopenBus &operator=(const CanopenBus &) = delete;

  bool start(uint32_t bitrate = 1'000'000);
  bool send(const CanopenFrame &frame) const;
  bool register_receiver(uint32_t can_id, QueueHandle_t queue);

private:
  struct Receiver {
    uint32_t can_id{0};
    QueueHandle_t queue{nullptr};
  };

  static bool on_receive(twai_node_handle_t handle, const twai_rx_done_event_data_t *event,
                         void *context);

  gpio_num_t rx_gpio_;
  gpio_num_t tx_gpio_;
  twai_node_handle_t node_{nullptr};
  std::array<Receiver, 16> receivers_{};
  mutable std::mutex transmit_mutex_;
};