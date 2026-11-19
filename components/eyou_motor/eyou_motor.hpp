#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <span>
#include <string>

#include "base_component.hpp"
#include "can_bus.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

using EyouCanFrame = CanFrame;

class EyouMotor : public espp::BaseComponent {
public:
  struct ProfilePositionConfig {
    float velocity_degrees_per_second;
    float acceleration_degrees_per_second_squared;
    float deceleration_degrees_per_second_squared;
  };

  static constexpr float kPulsesPerOutputTurn = static_cast<float>((1U << 19) * 100U);
  static constexpr float kDegreesPerOutputTurn = 360.0f;

  EyouMotor(CanBus &bus, uint8_t node_id = 1);
  ~EyouMotor();

  EyouMotor(const EyouMotor &) = delete;
  EyouMotor &operator=(const EyouMotor &) = delete;

  uint8_t node_id() const { return node_id_; }

  // Send a raw CAN frame on the bus (bypasses the CANopen/DS402 layer).
  bool send_raw(uint32_t can_id, std::span<const uint8_t> data, bool extended = false,
                bool rtr = false) const;

  // Reset the CANopen node and configure it for Profile Position mode.
  bool configure_profile_position(const ProfilePositionConfig &config);
  // Read back the drive's current Profile Position velocity/accel/decel (objects
  // 0x6081/0x6083/0x6084).
  bool get_profile_position_config(ProfilePositionConfig &config, uint32_t timeout_ms = 100);
  // Reset the software zero reference to the current actual position; set_position
  // (move_absolute) is relative to this reference.
  bool zero(uint32_t timeout_ms = 1000);
  // Persist the current software zero offset to NVS, keyed by node id, so it survives reboots.
  bool save_zero();
  // Read the persisted software zero offset without changing the active offset.
  bool read_saved_zero(int32_t &zero_offset_pulses);
  // Load a previously saved software zero offset from NVS, keyed by node id.
  bool load_zero();
  // Restrict move_absolute targets to [minimum_degrees, maximum_degrees].
  bool set_position_limits(float minimum_degrees, float maximum_degrees);
  // Move to an absolute position (degrees), relative to the software zero if set. The target is
  // clamped to the configured position limits (see set_position_limits).
  // Retries up to 3 times if a send/acknowledge step fails.
  bool move_absolute(float target_degrees, bool immediate = true);
  // Move by a relative position increment (degrees) from the current position.
  bool move_incremental(float increment_degrees, bool immediate = true);
  // Trajectory streaming: write the profile velocity (0x6081) and a new target,
  // and apply it immediately (the drive blends from its current motion; no
  // stop). Clamped to the position limits like move_absolute(), but without its
  // enable / mode / fault reads, so it costs ~5 SDOs. The drive must already be
  // enabled in profile position mode.
  bool retarget(float target_degrees, float velocity_degrees_per_second);
  // Drive the DS402 state machine to Operation Enabled.
  // Enable the drive (DS402 sequence to Operation Enabled, profile position
  // mode prepared first). A drive in Fault gets its fault reset on the way.
  bool enable_drive(uint32_t timeout_ms = 1000);
  // Drive the DS402 state machine to Ready to Switch On (power stage off).
  bool disable_drive(uint32_t timeout_ms = 1000);
  // Clear a DS402 fault via a controlword rising edge.
  bool reset_fault(uint32_t timeout_ms = 1000);
  // Read the actual position (degrees), relative to the software zero if set.
  bool get_position(float &position_degrees, uint32_t timeout_ms = 100);
  // Read the commanded profile-position target (0x607A, degrees), relative to the software zero.
  bool get_target_position(float &target_degrees, uint32_t timeout_ms = 100);
  // Read the DS402 statusword (object 0x6041).
  bool get_statusword(uint16_t &statusword, uint32_t timeout_ms = 100);
  // Read the active error code (object 0x603F).
  bool get_error_code(uint16_t &error_code, uint32_t timeout_ms = 100);
  // Read the statusword and report whether the drive is in the DS402 Fault state.
  // A communication failure is treated as faulted (fail-safe for monitoring loops).
  bool is_faulted(uint32_t timeout_ms = 100);
  // Read the modes-of-operation display (object 0x6061).
  bool get_operating_mode(int8_t &mode, uint32_t timeout_ms = 100);
  // Write modes of operation (0x6060): 1 profile position, 13 CSF (force
  // control, -F joints). No enable / verify: read get_operating_mode() back.
  bool set_operating_mode(int8_t mode, uint32_t timeout_ms = 100);
  // Diagnostics: read any object (expedited SDO, up to 4 bytes). `raw` holds
  // the little-endian bytes, `size` how many the drive sent. An abort is logged
  // with its code.
  bool read_sdo(uint16_t index, uint8_t subindex, uint32_t &raw, uint8_t &size,
                uint32_t timeout_ms = 100);
  // Profile velocity mode (3) for streamed moves: target velocity 0, the
  // given acceleration / deceleration (0x6083 / 0x6084), mode 3 (verified),
  // enabled. The drive then ramps smoothly to every set_target_velocity().
  bool start_velocity_mode(float acceleration_degrees_per_second_squared,
                           uint32_t timeout_ms = 1000);
  // Target velocity (0x60FF), deg/s at the output, signed. Mode 3 only.
  bool set_target_velocity(float velocity_degrees_per_second);
  // Back to profile position holding where the drive is: velocity 0, target
  // position = actual, mode 1 (verified), enabled, halt bit clear.
  bool hold_position(uint32_t timeout_ms = 1000);
  // Heartbeat consumer (0x1016:01): the drive watches `producer_node_id`'s
  // heartbeat (COB 0x700 + id) and raises a fault when none arrives within
  // timeout_ms. 0 ms disables it. Monitoring starts at the first heartbeat.
  bool set_heartbeat_consumer(uint8_t producer_node_id, uint16_t timeout_ms);
  // Persist the current object dictionary (profile velocity/accel/decel, mode, etc.) to
  // non-volatile memory via CiA 301 object 0x1010 (signature "save"). Sub-index 3 covers the
  // DS402 application objects (0x6000-0x9FFF); sub-index 1 saves everything. Optional and
  // vendor-specific: call once during commissioning, not on every boot (finite EEPROM endurance).
  bool save_parameters(uint8_t sub_index = 3, uint32_t timeout_ms = 5000);
  // Decode a DS402 statusword's state bits into a human-readable name.
  static const char *ds402_state_name(uint16_t statusword);

  // --- live readings and the torque limit (the test-stand console uses these) ---
  // Actual velocity (object 0x606C, pulses/s) in degrees per second.
  bool get_velocity(float &velocity_degrees_per_second, uint32_t timeout_ms = 100);
  // Actual torque (0x6077) and actual current (0x6078), in per-thousand of the
  // rated torque / current (see get_rated_torque / get_rated_current).
  bool get_torque(int16_t &torque_permille, uint32_t timeout_ms = 100);
  bool get_current(int16_t &current_permille, uint32_t timeout_ms = 100);
  // Output torque SENSOR (object 0x27BD, Nm, float), on force-controlled (-F)
  // joints only. Unlike get_torque() (0x6077, derived from motor current, so it
  // includes the gear friction), this is measured at the output flange.
  // Returns false if the drive has no such object (non -F joint) or does not answer.
  bool get_sensor_torque(float &torque_nm, uint32_t timeout_ms = 100);
  // Max torque (0x6072): the drive clamps its torque demand to this, in
  // per-thousand of the rated torque (1000 = rated). Applies to every mode, so a
  // profile position move cannot push harder than this.
  bool set_max_torque(uint16_t torque_permille, uint32_t timeout_ms = 100);
  bool get_max_torque(uint16_t &torque_permille, uint32_t timeout_ms = 100);
  // Rated torque (0x6076, mNm) and rated current (0x6075, mA): the scale of the
  // per-thousand values above.
  bool get_rated_torque(uint32_t &rated_torque_mnm, uint32_t timeout_ms = 100);
  bool get_rated_current(uint32_t &rated_current_ma, uint32_t timeout_ms = 100);
  // Set the software zero so that the CURRENT actual position reads as
  // `current_angle_degrees` (zero() is the special case 0). Volatile until save_zero().
  bool zero_at(float current_angle_degrees, uint32_t timeout_ms = 1000);
  // Write the Profile Position velocity / acceleration / deceleration (0x6081 /
  // 0x6083 / 0x6084) only: no node reset, no mode change (configure_profile_position
  // does both). Takes effect from the next set-point.
  bool set_profile_position_config(const ProfilePositionConfig &config, uint32_t timeout_ms = 100);
  // Stop the move in progress: raise the controlword's halt bit (the drive
  // decelerates with the profile deceleration), wait for it to come to rest,
  // then make the position reached the new set-point and release the halt bit,
  // so the move does not resume. The drive stays enabled.
  bool halt(uint32_t timeout_ms = 2000);
  // Only the first half of halt(): the stopping deceleration (0x6084) and the
  // halt bit, no waiting - so a STOP can start every drive decelerating at
  // once before waiting on any of them.
  bool request_halt(float deceleration_degrees_per_second_squared);

  // --- which node this object talks to ---
  // Talk to another CANopen node id from now on (the receivers move with it;
  // the bus may be running). The software zero is per node and is cleared:
  // load_zero() for the new node's saved zero. 1..127.
  bool set_node_id(uint8_t new_node_id);
  enum class NodeIdResult : uint8_t {
    Applied,              ///< the drive answers on the new id; this object follows it
    SavedNeedsPowerCycle, ///< written and saved, but the drive still runs its old id until
                          ///< power-cycled; this object STAYS on the old id
    Failed,               ///< nothing changed (the log says what failed)
  };
  // Re-address the DRIVE: write `new_node_id` to BOTH Node-ID objects (the
  // vendor 0x26A0, saved, and the CANopen 0x100B, which the manual says takes
  // effect after an NMT reset communication + start), save the parameters
  // (0x1010:01, about 3 s, confirmed by reading it back as 2), reset
  // communication + start the node, and look for it on the new id (boot-up,
  // then a statusword read). Applied -> this object follows it; else it keeps
  // talking to the old id.
  NodeIdResult set_drive_node_id(uint8_t new_node_id, uint32_t timeout_ms = 8000);

  // Get a drive ready for profile position moves without a node reset: NMT
  // start, modes of operation = 1 (profile position, verified on 0x6061) and
  // the target position = the actual position, so enabling does not jump. A
  // fresh drive (mode 0) refuses the DS402 enable sequence otherwise; the
  // manual's own example sets the mode before the controlword. Idempotent:
  // enable_drive() runs it first.
  bool prepare_profile_position(uint32_t timeout_ms = 1000);

  // The active software zero (encoder pulses), if one was set / loaded.
  std::optional<int32_t> zero_offset_pulses() const;
  // The position limits move_absolute() clamps to (infinite when unset).
  float minimum_position_degrees() const;
  float maximum_position_degrees() const;

private:
  bool send_sdo_u8(uint16_t index, uint8_t value, uint8_t subindex = 0, uint32_t timeout_ms = 100);
  bool send_sdo_u16(uint16_t index, uint16_t value, uint8_t subindex = 0,
                    uint32_t timeout_ms = 100);
  bool send_sdo_u32(uint16_t index, uint32_t value, uint8_t subindex = 0,
                    uint32_t timeout_ms = 100);
  bool send_sdo_download(uint16_t index, uint8_t subindex, uint8_t command, uint32_t value,
                         uint32_t timeout_ms = 100);
  bool reset_communication_and_wait_for_bootup(uint32_t timeout_ms);
  bool ensure_operation_enabled(uint32_t timeout_ms);
  bool ensure_profile_position_mode(uint32_t timeout_ms);
  bool wait_for_mode_unlocked(int8_t mode, uint32_t timeout_ms);
  // Same check as is_faulted(), but assumes transaction_mutex_ is already held.
  bool is_faulted_unlocked(uint32_t timeout_ms);
  bool reset_fault_unlocked(uint32_t timeout_ms);
  bool wait_for_ds402_state(uint16_t expected_state, uint32_t timeout_ms);
  bool wait_for_operation_enabled(uint32_t timeout_ms);
  bool request_sdo_u32(uint16_t index, uint8_t expected_response_command, uint32_t &value,
                       uint32_t timeout_ms);
  // Expedited SDO upload of any size: `value` holds the raw little-endian bytes,
  // `size` (1..4) what the drive sent (from the response command byte).
  bool request_sdo_expedited(uint16_t index, uint8_t subindex, uint32_t &value, uint8_t &size,
                             uint32_t timeout_ms);
  // ... sign-extended to the size the drive sent (INT8 / INT16 / INT32 objects).
  bool request_sdo_signed(uint16_t index, uint8_t subindex, int32_t &value, uint32_t timeout_ms);
  bool trigger_profile_position(bool relative, bool immediate);
  // Move the receive queue's registrations to 0x580 / 0x700 + new_node_id and
  // adopt it (transaction_mutex_ held). Keeps the old id if the bus refuses.
  bool rebind_receivers_unlocked(uint8_t new_node_id);
  // prepare_profile_position() with transaction_mutex_ held.
  bool prepare_profile_position_unlocked(uint32_t timeout_ms);
  bool degrees_to_pulses(float degrees, int32_t &pulses) const;
  bool degrees_to_profile_units(float degrees, uint32_t &pulses) const;
  static float pulses_to_degrees(int32_t pulses);
  static float profile_units_to_degrees(uint32_t pulses);
  std::string zero_nvs_key() const;

  CanBus &bus_;
  uint8_t node_id_{1};
  QueueHandle_t receive_queue_{nullptr};
  // Unset means get_position/move_absolute operate on the absolute encoder position.
  std::optional<int32_t> zero_offset_pulses_;
  float minimum_position_degrees_{-std::numeric_limits<float>::infinity()};
  float maximum_position_degrees_{std::numeric_limits<float>::infinity()};
  mutable std::mutex transaction_mutex_;
};
