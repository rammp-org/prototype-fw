#include "can_bus.hpp"

#include <algorithm>

CanBus::CanBus(gpio_num_t rx_gpio, gpio_num_t tx_gpio)
    : espp::BaseComponent("CanBus", espp::Logger::Verbosity::INFO), rx_gpio_(rx_gpio),
      tx_gpio_(tx_gpio) {}

CanBus::~CanBus() {
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (node_ != nullptr) {
    twai_node_disable(node_);
    twai_node_delete(node_);
  }
}

bool CanBus::register_receiver(uint32_t can_id, QueueHandle_t queue) {
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (node_ != nullptr || queue == nullptr) {
    return false;
  }
  for (auto &receiver : receivers_) {
    if (receiver.queue == nullptr) {
      receiver = {.can_id = can_id, .queue = queue};
      return true;
    }
  }
  return false;
}

bool CanBus::on_receive(twai_node_handle_t handle, const twai_rx_done_event_data_t *,
                         void *context) {
  auto *bus = static_cast<CanBus *>(context);
  CanFrame frame{};
  twai_frame_t twai_frame{};
  twai_frame.buffer = frame.data.data();
  twai_frame.buffer_len = frame.data.size();
  if (twai_node_receive_from_isr(handle, &twai_frame) != ESP_OK) {
    return false;
  }
  frame.id = twai_frame.header.id;
  frame.extended = twai_frame.header.ide;
  frame.rtr = twai_frame.header.rtr;
  frame.dlc = static_cast<uint8_t>(
      std::min<size_t>(twaifd_dlc2len(twai_frame.header.dlc), frame.data.size()));

  BaseType_t task_woken = pdFALSE;
  for (const auto &receiver : bus->receivers_) {
    if (receiver.queue != nullptr && receiver.can_id == frame.id) {
      xQueueSendFromISR(receiver.queue, &frame, &task_woken);
    }
  }
  return task_woken == pdTRUE;
}

bool CanBus::start(uint32_t bitrate) {
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (node_ != nullptr) {
    return true;
  }

  twai_onchip_node_config_t config{};
  config.io_cfg.rx = rx_gpio_;
  config.io_cfg.tx = tx_gpio_;
  config.io_cfg.quanta_clk_out = GPIO_NUM_NC;
  config.io_cfg.bus_off_indicator = GPIO_NUM_NC;
  config.bit_timing.bitrate = bitrate;
  config.tx_queue_depth = 8;

  esp_err_t error = twai_new_node_onchip(&config, &node_);
  if (error == ESP_OK) {
    twai_event_callbacks_t callbacks{};
    callbacks.on_rx_done = on_receive;
    error = twai_node_register_event_callbacks(node_, &callbacks, this);
  }
  if (error == ESP_OK) {
    error = twai_node_enable(node_);
  }
  if (error != ESP_OK) {
    logger_.error("Failed to start CAN bus: {}", esp_err_to_name(error));
    if (node_ != nullptr) {
      twai_node_delete(node_);
      node_ = nullptr;
    }
    return false;
  }

  logger_.info("CAN bus started: RX GPIO {}, TX GPIO {}, {} bit/s", static_cast<int>(rx_gpio_),
               static_cast<int>(tx_gpio_), bitrate);
  return true;
}

bool CanBus::send(const CanFrame &message) const {
  std::lock_guard<std::mutex> lock(transmit_mutex_);
  if (node_ == nullptr || message.dlc > message.data.size()) {
    return false;
  }

  twai_frame_t frame{};
  frame.header.id = message.id;
  frame.header.ide = message.extended;
  frame.header.rtr = message.rtr;
  frame.header.fdf = 0;
  frame.header.dlc = message.dlc;
  frame.buffer = const_cast<uint8_t *>(message.data.data());
  frame.buffer_len = message.dlc;
  const esp_err_t error = twai_node_transmit(node_, &frame, 3);
  return error == ESP_OK && twai_node_transmit_wait_all_done(node_, 10) == ESP_OK;
}