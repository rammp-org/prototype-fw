#pragma once

#include <array>
#include <cstdint>
#include <mutex>

#include "base_component.hpp"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

struct CanFrame {
  uint32_t id{0};
  bool extended{false};
  bool rtr{false};
  uint8_t dlc{0};
  std::array<uint8_t, 8> data{};
};

/// One on-chip TWAI node shared by every protocol on the bus (CANopen/DS402
/// drives, RMD motors, ...). Each protocol registers the CAN ids it wants on its
/// own FreeRTOS queue BEFORE start(); the receive ISR fans every matching frame
/// out to all registered queues.
class CanBus : public espp::BaseComponent {
public:
  CanBus(gpio_num_t rx_gpio, gpio_num_t tx_gpio);
  ~CanBus();

  CanBus(const CanBus &) = delete;
  CanBus &operator=(const CanBus &) = delete;

  bool start(uint32_t bitrate = 1'000'000);
  /// Enqueue one frame for transmission. Never waits on wire completion: the
  /// on-chip TWAI TX queue is zero-copy (it stores a pointer to the frame and
  /// reads the buffer from the ISR when the frame actually goes out), so the
  /// frame is copied into a persistent ring slot (see tx_ring_) rather than a
  /// stack frame, and a caller is never blocked on an ACK that may never arrive
  /// (e.g. no motor attached). wait_for_queue_space=true (request/response
  /// paths, which get their reply through the RX queue) briefly waits for queue
  /// space so a poll is not dropped under transient pressure; false
  /// (fire-and-forget set-points from a real-time loop) enqueues non-blocking
  /// and drops the frame when the queue is full - the next tick supersedes it.
  bool send(const CanFrame &frame, bool wait_for_queue_space = true);
  /// Wait (bounded, a few ms at 1 Mbit/s) for every queued frame to leave: a
  /// mode transition that must not let an older set-point execute after the
  /// stop frames that follow calls this first, so those go out on an empty
  /// queue. Returns false if the queue did not drain in time (a wedged
  /// controller).
  bool flush_pending();
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
  std::array<Receiver, 64> receivers_{};
  std::mutex transmit_mutex_;
  std::mutex lifecycle_mutex_;

  /// Driver TX queue depth; the ring below must exceed it plus the one frame in
  /// flight so a slot is never reused while the driver may still reference it.
  static constexpr size_t kTxQueueDepth = 8;
  // Persistent, driver-referenced TX frames: send() only advances the ring
  // after a successful enqueue. All access is serialized by transmit_mutex_.
  static constexpr size_t kTxRingSize = 16;
  struct TxSlot {
    twai_frame_t frame{};
    std::array<uint8_t, 8> payload{};
  };
  std::array<TxSlot, kTxRingSize> tx_ring_{};
  size_t tx_ring_index_{0};
};
