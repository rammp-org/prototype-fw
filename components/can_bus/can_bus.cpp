#include "can_bus.hpp"

#include <algorithm>

#include "esp_timer.h"

CanBus::CanBus(gpio_num_t rx_gpio, gpio_num_t tx_gpio)
    : espp::BaseComponent("CanBus", espp::Logger::Verbosity::INFO)
    , rx_gpio_(rx_gpio)
    , tx_gpio_(tx_gpio) {}

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
  portENTER_CRITICAL(&receivers_lock_);
  const auto free_slot = std::find_if(receivers_.begin(), receivers_.end(),
                                      [](const Receiver &r) { return r.queue == nullptr; });
  const bool found = free_slot != receivers_.end();
  if (found) {
    *free_slot = {.can_id = can_id, .queue = queue};
  }
  portEXIT_CRITICAL(&receivers_lock_);
  return found;
}

void CanBus::unregister_receiver(QueueHandle_t queue) {
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  // the critical section excludes the RX ISR on every core: when it is
  // released, no ISR can still be fanning a frame out to this queue
  portENTER_CRITICAL(&receivers_lock_);
  std::replace_if(
      receivers_.begin(), receivers_.end(), [queue](const Receiver &r) { return r.queue == queue; },
      Receiver{});
  portEXIT_CRITICAL(&receivers_lock_);
}

bool CanBus::on_receive(twai_node_handle_t handle, const twai_rx_done_event_data_t *event,
                        void *context) {
  (void)event;
  const auto *bus = static_cast<const CanBus *>(context);
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
  portENTER_CRITICAL_ISR(&bus->receivers_lock_);
  for (const auto &receiver : bus->receivers_) {
    if (receiver.queue != nullptr && receiver.can_id == frame.id) {
      xQueueSendFromISR(receiver.queue, &frame, &task_woken);
    }
  }
  portEXIT_CRITICAL_ISR(&bus->receivers_lock_);
  return task_woken == pdTRUE;
}

bool CanBus::on_error(twai_node_handle_t handle, const twai_error_event_data_t *event,
                      void *context) {
  (void)handle;
  auto *bus = static_cast<CanBus *>(context);
  const auto flags = event->err_flags;
  if (flags.arb_lost)
    bus->arbitration_lost_.fetch_add(1, std::memory_order_relaxed);
  if (flags.bit_err)
    bus->bit_errors_.fetch_add(1, std::memory_order_relaxed);
  if (flags.form_err)
    bus->form_errors_.fetch_add(1, std::memory_order_relaxed);
  if (flags.stuff_err)
    bus->stuff_errors_.fetch_add(1, std::memory_order_relaxed);
  if (flags.ack_err)
    bus->ack_errors_.fetch_add(1, std::memory_order_relaxed);
  return false;
}

bool CanBus::create_node_locked() {
  twai_onchip_node_config_t config{};
  config.io_cfg.rx = rx_gpio_;
  config.io_cfg.tx = tx_gpio_;
  config.io_cfg.quanta_clk_out = GPIO_NUM_NC;
  config.io_cfg.bus_off_indicator = GPIO_NUM_NC;
  config.bit_timing.bitrate = bitrate_;
  config.tx_queue_depth = kTxQueueDepth;
  config.fail_retry_cnt = kTxRetryForever;

  esp_err_t error = twai_new_node_onchip(&config, &node_);
  if (error == ESP_OK) {
    twai_event_callbacks_t callbacks{};
    callbacks.on_rx_done = on_receive;
    callbacks.on_error = on_error;
    callbacks.on_state_change = on_state_change;
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
  return true;
}

bool CanBus::on_state_change(twai_node_handle_t handle, const twai_state_change_event_data_t *event,
                             void *context) {
  (void)handle;
  auto *bus = static_cast<CanBus *>(context);
  const bool off = event->new_sta == TWAI_ERROR_BUS_OFF;
  if (off && event->old_sta != TWAI_ERROR_BUS_OFF)
    bus->bus_off_count_.fetch_add(1, std::memory_order_relaxed);
  bus->bus_off_.store(off, std::memory_order_relaxed);
  return false;
}

bool CanBus::handle_bus_off_locked() {
  const int64_t now = esp_timer_get_time();
  if (now - last_bus_off_log_us_ > 2'000'000) {
    last_bus_off_log_us_ = now;
    logger_.warn("CAN controller is BUS OFF ({} time(s) since start): recovering; frames are "
                 "dropped until it is back on the bus",
                 bus_off_count_.load(std::memory_order_relaxed));
  }
  if (now - last_recover_us_ > 100'000) {
    last_recover_us_ = now;
    const esp_err_t error = twai_node_recover(node_);
    if (error != ESP_OK && error != ESP_ERR_INVALID_STATE) // INVALID_STATE: already back
      logger_.error("CAN bus-off recovery failed: {}", esp_err_to_name(error));
  }
  return false;
}

bool CanBus::start(uint32_t bitrate) {
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (node_ != nullptr) {
    return true;
  }
  bitrate_ = bitrate;
  if (!create_node_locked()) {
    return false;
  }
  logger_.info("CAN bus started: RX GPIO {}, TX GPIO {}, {} bit/s", static_cast<int>(rx_gpio_),
               static_cast<int>(tx_gpio_), bitrate);
  return true;
}

bool CanBus::abort_pending() {
  std::lock_guard<std::mutex> lock(transmit_mutex_); // no send() while the node is gone
  return abort_pending_locked();
}

bool CanBus::abort_pending_locked() {
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (node_ == nullptr) {
    return false;
  }
  // ESP_ERR_INVALID_STATE = already stopped (bus-off), which is what delete needs
  esp_err_t error = twai_node_disable(node_);
  if (error != ESP_OK && error != ESP_ERR_INVALID_STATE) {
    logger_.error("Could not disable the CAN node to drop its pending frames: {}",
                  esp_err_to_name(error));
    return false;
  }
  error = twai_node_delete(node_);
  if (error != ESP_OK) {
    logger_.error("Could not delete the CAN node to drop its pending frames: {}",
                  esp_err_to_name(error));
    twai_node_enable(node_); // keep what we have
    return false;
  }
  node_ = nullptr;
  bus_off_.store(false, std::memory_order_relaxed);
  logger_.warn("CAN TX queue dropped: a frame nobody acknowledged was blocking it");
  return create_node_locked();
}

bool CanBus::send(const CanFrame &frame, bool wait_for_queue_space) {
  std::lock_guard<std::mutex> lock(transmit_mutex_);
  if (node_ == nullptr || frame.dlc > frame.data.size()) {
    return false;
  }
  if (bus_off_.load(std::memory_order_relaxed)) {
    return handle_bus_off_locked(); // no point queueing: the driver refuses while bus-off
  }

  // Copy into a persistent ring slot: the on-chip TWAI TX queue is zero-copy and
  // reads this frame/buffer asynchronously from the ISR, so it must not live on
  // the stack. Only advance the ring after a successful enqueue, so a slot is
  // never reused while the driver may still reference it.
  TxSlot &slot = tx_ring_[tx_ring_index_];
  slot.payload = frame.data;
  slot.frame = twai_frame_t{};
  slot.frame.header.id = frame.id;
  slot.frame.header.ide = frame.extended;
  slot.frame.header.rtr = frame.rtr;
  slot.frame.header.fdf = 0;
  slot.frame.header.dlc = frame.dlc;
  slot.frame.buffer = slot.payload.data();
  slot.frame.buffer_len = frame.dlc;

  // Never wait on wire completion (twai_node_transmit_wait_all_done): with no
  // bus ACK that would stall for the full timeout on every send. The request /
  // response protocols get their reply through the RX queue instead.
  const uint32_t enqueue_timeout_ms = wait_for_queue_space ? 3 : 0;
  const esp_err_t error = twai_node_transmit(node_, &slot.frame, enqueue_timeout_ms);
  if (error == ESP_OK) {
    tx_ring_index_ = (tx_ring_index_ + 1) % kTxRingSize;
    return true;
  }
  if (wait_for_queue_space) {
    logger_.warn("CAN transmit failed: {}", esp_err_to_name(error));
  } else {
    // Expected and self-correcting when nothing is draining the bus; keep it off
    // the error path so it does not spam a real-time control loop.
    logger_.debug("CAN frame dropped (non-blocking): {}", esp_err_to_name(error));
  }
  return false;
}

CanBus::Status CanBus::get_status() {
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  Status status{};
  if (node_ == nullptr) {
    return status;
  }
  twai_node_status_t node_status{};
  twai_node_record_t record{};
  if (twai_node_get_info(node_, &node_status, &record) != ESP_OK) {
    status.state = "unknown (get_info failed)";
    return status;
  }
  switch (node_status.state) {
  case TWAI_ERROR_ACTIVE:
    status.state = "error active (healthy)";
    break;
  case TWAI_ERROR_WARNING:
    status.state = "error warning (TEC/REC >= 96)";
    break;
  case TWAI_ERROR_PASSIVE:
    status.state = "error passive (TEC/REC >= 128)";
    break;
  case TWAI_ERROR_BUS_OFF:
    status.state = "BUS OFF";
    status.bus_off = true;
    break;
  default:
    status.state = "unknown";
    break;
  }
  status.tx_error_count = node_status.tx_error_count;
  status.rx_error_count = node_status.rx_error_count;
  status.bus_error_count = record.bus_err_num;
  status.tx_queue_free = node_status.tx_queue_remaining;
  status.arbitration_lost = arbitration_lost_.load(std::memory_order_relaxed);
  status.bit_errors = bit_errors_.load(std::memory_order_relaxed);
  status.form_errors = form_errors_.load(std::memory_order_relaxed);
  status.stuff_errors = stuff_errors_.load(std::memory_order_relaxed);
  status.ack_errors = ack_errors_.load(std::memory_order_relaxed);
  status.bus_off_count = bus_off_count_.load(std::memory_order_relaxed);
  return status;
}

bool CanBus::recover() {
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (node_ == nullptr) {
    return false;
  }
  const esp_err_t error = twai_node_recover(node_);
  if (error != ESP_OK) {
    logger_.error("CAN bus-off recovery failed: {}", esp_err_to_name(error));
    return false;
  }
  return true;
}

bool CanBus::flush_pending() {
  std::lock_guard<std::mutex> lock(transmit_mutex_);
  if (node_ == nullptr) {
    return false;
  }
  // On a live bus the backlog (kTxQueueDepth queued + 1 in flight) is gone
  // within a couple of ms at 1 Mbit/s, so the caller's stop frames then find
  // an empty queue, go out next, and nothing older follows. If it does NOT
  // drain, a frame nobody acknowledges is being retried forever in front
  // (kTxRetryForever; motor power off): drop the whole queue so the stop
  // frames are not queued behind a stale set-point that would execute the
  // moment the bus comes back.
  static constexpr int kDrainTimeoutMs = 10;
  const esp_err_t error = twai_node_transmit_wait_all_done(node_, kDrainTimeoutMs);
  if (error == ESP_OK) {
    return true;
  }
  logger_.warn("CAN TX queue did not drain within {} ms ({}); dropping it", kDrainTimeoutMs,
               esp_err_to_name(error));
  return abort_pending_locked();
}
