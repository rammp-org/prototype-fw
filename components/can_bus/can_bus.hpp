#pragma once

#include <array>
#include <atomic>
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

  /// Controller error state and counters (twai_node_get_info()): the quickest
  /// way to tell a wiring / bitrate problem (REC and bus errors climb while
  /// nothing answers) from a silent node (counters stay at zero).
  struct Status {
    const char *state{"not started"}; ///< error active / warning / passive / bus off
    uint16_t tx_error_count{0};       ///< TEC
    uint16_t rx_error_count{0};       ///< REC
    uint32_t bus_error_count{0};      ///< bus errors seen since start()
    uint32_t tx_queue_free{0};        ///< free slots in the driver's TX queue
    bool bus_off{false};              ///< the controller left the bus; see recover()
    /// Errors by type since start() (the driver's error callback). Stuff /
    /// form / bit / CRC-class errors are signal integrity (termination, a
    /// floating isolated ground, a bad stub); ACK errors mean nobody heard a
    /// frame we sent; arbitration losses are normal and retried.
    uint32_t arbitration_lost{0};
    uint32_t bit_errors{0};
    uint32_t form_errors{0};
    uint32_t stuff_errors{0};
    uint32_t ack_errors{0};
  };

  bool start(uint32_t bitrate = 1'000'000);
  Status get_status();
  /// Start the bus-off recovery sequence (128 x 11 recessive bits); the node
  /// is back once get_status() reports error active again.
  bool recover();
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
  /// Drop every registration of `queue`, so the RX ISR stops writing to it.
  /// Call it BEFORE deleting the queue; the slot's pointer is cleared with a
  /// single store, so a frame the ISR is fanning out at that instant still
  /// lands on the live queue and nothing later touches the deleted one.
  void unregister_receiver(QueueHandle_t queue);

private:
  struct Receiver {
    uint32_t can_id{0};
    QueueHandle_t queue{nullptr};
  };

  static bool on_receive(twai_node_handle_t handle, const twai_rx_done_event_data_t *event,
                         void *context);
  static bool on_error(twai_node_handle_t handle, const twai_error_event_data_t *event,
                       void *context);

  gpio_num_t rx_gpio_;
  gpio_num_t tx_gpio_;
  twai_node_handle_t node_{nullptr};
  std::array<Receiver, 64> receivers_{};
  /// Guards receivers_ against the RX ISR (on either core): registration and
  /// deregistration take it, and the ISR fans a frame out while holding it, so
  /// once unregister_receiver() returns no ISR can still hold the queue handle.
  mutable portMUX_TYPE receivers_lock_ = portMUX_INITIALIZER_UNLOCKED;
  std::mutex transmit_mutex_;
  std::mutex lifecycle_mutex_;
  /// error counters by type, written from the driver's error ISR
  std::atomic<uint32_t> arbitration_lost_{0};
  std::atomic<uint32_t> bit_errors_{0};
  std::atomic<uint32_t> form_errors_{0};
  std::atomic<uint32_t> stuff_errors_{0};
  std::atomic<uint32_t> ack_errors_{0};

  /// Driver TX queue depth; the ring below must exceed it plus the one frame in
  /// flight so a slot is never reused while the driver may still reference it.
  static constexpr size_t kTxQueueDepth = 8;
  /// Hardware retransmissions after a lost arbitration or a bus error. The bus
  /// is multi-master in practice (every CANopen node answers on its own, with
  /// ids that beat ours), so single-shot (0) silently dropped any request that
  /// started together with a reply - seen as sporadic SDO timeouts. Bounded,
  /// so a frame nothing ACKs is given up within about a millisecond instead
  /// of wedging the queue the way infinite retries (-1) would.
  static constexpr int8_t kTxRetryLimit = 8;
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
