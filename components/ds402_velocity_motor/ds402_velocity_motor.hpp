#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <span>

#include "base_component.hpp"
#include "can_bus.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/// A simple CANopen velocity drive: the AMPS L2DB servo hub driver (e.g. the
/// L2DB4830-CAFC on the PEAR wheels) and drives with the same reduced DS402
/// surface. Everything goes over expedited SDO (0x600 + id / 0x580 + id) on a
/// CanBus shared with other CANopen nodes; no PDO mapping is assumed.
///
/// What "simple" means (L2DB manual, 7.2 Speed Mode + Appendix 1): modes of
/// operation 0x6060 = 3 (speed mode with acc/dec) or -3 (without); controlword
/// 0x6040 is a plain command - 0x0F enable, 0x06 disable (loose shaft), 0x86
/// clear error - instead of the DS402 shutdown / switch-on / enable sequence;
/// statusword 0x6041 carries only the alarm bit (0x0008); the set-point is
/// "Target Velocity_DEC" 0x60FF:00 (int32, DEC = rpm * 512 * resolution / 1875).
///
/// The set-point path is NON-BLOCKING: send_velocity_rpm() / stop() enqueue the
/// SDO download and return without waiting for the drive's confirmation, so a
/// real-time control loop never stalls on a missing reply (the confirmation is
/// drained at the next transaction). Enable / disable / fault reset / status
/// reads are confirmed transactions.
///
/// Construct every instance BEFORE CanBus::start(): the receivers (SDO reply
/// 0x580 + id, boot-up 0x700 + id) are registered here.
class Ds402VelocityMotor : public espp::BaseComponent {
public:
  /// A readable object the driver polls for read_status(); index 0 = none.
  struct ObjectSpec {
    uint16_t index{0};
    uint8_t subindex{0};
    float scale{1.0f}; ///< value = raw (signed) * scale
  };

  /// Defaults are the L2DB4830 with its default feedback resolution (4096).
  struct Config {
    /// 0x60FF units per OUTPUT-SHAFT RPM: 512 * resolution / 1875.
    float velocity_counts_per_rpm{512.0f * 4096.0f / 1875.0f};
    /// 0x6063 counts per output-shaft revolution (the feedback resolution);
    /// 0 = position not reported.
    float position_counts_per_rev{4096.0f};
    /// Speed mode with (3) or without (-3) the drive's acc/dec ramps.
    bool use_acc_dec{true};
    /// 0x6083 / 0x6084 ramps in rev/s^2 (DEC = rps/s * 256 * resolution / 15625);
    /// 0 = leave the drive's own setting.
    float acceleration_rps2{0.0f};
    float deceleration_rps2{0.0f};
    /// 0x6083 / 0x6084 units per rev/s^2.
    float acceleration_counts_per_rps2{256.0f * 4096.0f / 15625.0f};
    /// 0x6080 "Max. speed_rpm" (uint16); 0 = leave the drive's own setting.
    uint16_t max_speed_rpm{0};
    /// Objects read by read_status(). L2DB: actual speed 0x60F9:19 in 0.001 rpm
    /// (the smoother of the two), actual position 0x6063:00, actual current Iq
    /// 0x6078:00 (DEC = A * 1.414 * 2048 / I_MAX, I_MAX = 33 A on the 4830),
    /// driver temperature 0x60F7:0B in °C, error code bits 0x2601:00.
    ObjectSpec velocity_actual{0x60F9, 0x19, 0.001f};
    ObjectSpec position_actual{0x6063, 0x00, 1.0f};
    ObjectSpec current_actual{0x6078, 0x00, 33.0f / (1.414f * 2048.0f)};
    ObjectSpec temperature{0x60F7, 0x0B, 1.0f};
    ObjectSpec error_code{0x2601, 0x00, 1.0f};
    /// NMT reset-communication + boot-up wait before enabling (drives that
    /// send a boot-up message and need the reset to leave a stale state).
    bool reset_communication_on_enable{false};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::INFO};
  };

  struct Status {
    uint16_t statusword{0};
    uint16_t error_code{0}; ///< the drive's error bit mask (0 = none)
    float velocity_rpm{0.0f};
    float position_degrees{0.0f};
    float temperature_c{0.0f}; ///< 0 when the drive has no temperature object
    float current_a{0.0f};     ///< 0 when the drive has no current object
    bool faulted{false};       ///< statusword alarm bit
  };

  Ds402VelocityMotor(CanBus &bus, uint8_t node_id, const Config &config);
  ~Ds402VelocityMotor();

  Ds402VelocityMotor(const Ds402VelocityMotor &) = delete;
  Ds402VelocityMotor &operator=(const Ds402VelocityMotor &) = delete;

  uint8_t node_id() const { return node_id_; }

  /// NMT start, speed mode, ramps / max speed, a zero set-point, then
  /// controlword enable; confirmed, and the alarm bit must be clear after.
  bool enable_drive(uint32_t timeout_ms = 1000);
  /// Controlword disable (power stage off, shaft loose; confirmed).
  bool disable_drive(uint32_t timeout_ms = 1000);
  /// Controlword clear-error, then wait for the alarm bit to clear (confirmed).
  bool reset_fault(uint32_t timeout_ms = 1000);

  /// Stream a velocity set-point (output-shaft RPM). Non-blocking.
  bool send_velocity_rpm(float rpm);
  /// Velocity 0 (non-blocking) - the drive decelerates and holds at zero.
  bool stop();
  /// Velocity 0 and wait for THIS drive's SDO confirmation, so a control-level
  /// stop can retry until every motor really took it.
  bool stop_acknowledged(uint32_t timeout_ms = 20);

  /// Poll the statusword and the configured measurement objects. Returns
  /// false if the statusword read failed (the other fields keep their last
  /// value then).
  bool read_status(Status &status, uint32_t timeout_ms = 20);

  static const char *state_name(uint16_t statusword);

private:
  bool send_raw(uint32_t can_id, std::span<const uint8_t> data, bool wait_for_queue_space);
  bool send_sdo_download(uint16_t index, uint8_t subindex, uint8_t command, uint32_t value,
                         uint32_t timeout_ms);
  bool send_sdo_download_unconfirmed(uint16_t index, uint8_t subindex, uint8_t command,
                                     uint32_t value);
  bool request_sdo_u32(uint16_t index, uint8_t subindex, uint32_t &value, uint32_t timeout_ms);
  bool read_signed(const ObjectSpec &object, float &value, uint32_t timeout_ms);
  bool wait_for_alarm_clear(uint32_t timeout_ms, uint16_t &statusword);
  bool reset_communication_and_wait_for_bootup(uint32_t timeout_ms);
  bool write_target_velocity(int32_t counts, bool confirmed, uint32_t timeout_ms);
  int32_t rpm_to_counts(float rpm) const;

  CanBus &bus_;
  uint8_t node_id_{1};
  Config config_;
  QueueHandle_t receive_queue_{nullptr};
  std::mutex transaction_mutex_;
};
