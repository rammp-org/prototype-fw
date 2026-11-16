#pragma once

// LinkageController: ONE 5-bar linkage = two EyouMotor cranks + the
// kinematics + the settings. The one place that turns end-effector targets
// into motor commands (settings, limits, the ik reference, the tracked
// target), shared by the USB service instances. Public methods are serialized
// by a mutex; the motor transactions themselves are serialized by EyouMotor.

#include <array>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include "base_component.hpp"
#include "can_bus.hpp"
#include "eyou_motor.hpp"
#include "five_bar.hpp"
#include "linkage_protocol.hpp"

class LinkageController : public espp::BaseComponent {
public:
  using Settings = linkage_protocol::Settings;
  using Status = linkage_protocol::Status;
  using IkResult = linkage_protocol::IkResult;
  static constexpr uint8_t kMotorCount = linkage_protocol::kMotorCount;
  static constexpr uint8_t kAllMotors = linkage_protocol::kAllMotors;

  struct Config {
    std::array<EyouMotor *, kMotorCount> motors; ///< crank 1 (M1), crank 2 (M2)
    uint16_t max_torque_permille{500};           ///< ceiling for the settings' torque limits
    /// The CAN bus the motors are on (optional): its error state and counters
    /// go into Status::can, a halt / disable flushes a set-point stuck in its
    /// TX queue first (CanBus::flush_pending), recover_can() restarts it.
    CanBus *can_bus{nullptr};
    bool can_ok{true};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::INFO};
  };

  enum class Result : uint8_t {
    Ok,
    Unreachable,
    OutOfLimits,
    NoMotor,     ///< a crank has no motor configured (node id 0)
    NotEnabled,  ///< a motor is not Operation enabled (or not answering)
    MotorFailed, ///< the CANopen command failed
    NoPose,      ///< a relative move needs a measured pose
    InvalidArgument,
  };
  static const char *to_string(Result result);

  explicit LinkageController(const Config &config);

  /// A copy of the active settings.
  Settings settings() const;
  /// Validate and apply new settings: node ids (the motors follow), limits,
  /// torque limits, profile, the saved zeros of the (new) nodes. The motors
  /// that answer are commissioned now, the others when they appear. False
  /// with `why` when the settings are refused (nothing changes).
  bool apply_settings(const Settings &settings, std::string &why);

  /// Read every motor and build the status snapshot (the poll task's tick).
  /// Commissions a motor that just started answering.
  void poll(Status &status);

  /// Inverse kinematics of (x, y) with the active settings and the latest
  /// measured pose (or home) as the branch reference; no motion.
  bool solve(float x, float y, IkResult &out) const;

  Result move_to(float x, float y);
  Result move_by(float dx, float dy);
  Result home();
  /// One crank's motor to an absolute motor angle (clears the target).
  Result set_motor(uint8_t motor, float degrees);

  /// 0xFF = every configured motor. True when every addressed motor succeeded.
  bool enable(uint8_t motor);
  bool disable(uint8_t motor);
  bool reset_fault(uint8_t motor);
  bool halt(uint8_t motor);
  /// Start the CAN bus-off recovery (false without a bus or if not started).
  bool recover_can();
  /// Re-address the DRIVE on crank `motor` (0 / 1) to `new_id`
  /// (EyouMotor::set_drive_node_id: both Node-ID objects, saved, communication
  /// reset). Applied: the settings' node id follows it (persist them through
  /// settings()); SavedNeedsPowerCycle: the drive keeps its old id until
  /// power-cycled, the settings are unchanged; Failed: nothing changed.
  EyouMotor::NodeIdResult set_drive_node_id(uint8_t motor, uint8_t new_id);

  /// Set a motor's software zero: mode 0 = reads 0 here, 1 = reads `value`
  /// (motor deg) here, 2 = the crank is at `value` deg (linkage frame) here.
  Result zero_motor(uint8_t motor, uint8_t mode, float value);
  bool save_zero(uint8_t motor);
  bool load_zero(uint8_t motor);

private:
  bool present_unlocked(uint8_t i) const { return settings_.motors[i].node_id != 0; }
  /// The ik branch reference: the measured crank angles when the pose is
  /// valid, else the home angles (mutex_ held).
  std::array<float, 2> reference_unlocked() const;
  bool solve_unlocked(float x, float y, IkResult &out) const;
  /// Both cranks to motor angles (mutex_ held; limits already checked).
  Result command_unlocked(const std::array<float, kMotorCount> &motor_deg, const char *what);
  /// torque limit, profile, limits and saved zero of one motor (mutex_ held).
  bool commission_unlocked(uint8_t i);
  void flush_pending_unlocked();
  template <typename Fn> bool for_each_motor(uint8_t motor, const char *what, Fn &&fn);

  Config config_;
  mutable std::mutex mutex_;
  Settings settings_{};
  std::array<bool, kMotorCount> commissioned_{};
  std::array<uint16_t, kMotorCount>
      rated_torque_cnm_{}; ///< 0x6076 in 0.01 Nm per motor, 0 until read (re-read on commission)
  std::array<bool, kMotorCount> settings_dirty_{};     ///< re-read the slow objects
  std::optional<std::array<float, 2>> measured_theta_; ///< last valid fk input
  std::optional<fivebar::Point> target_;
  uint32_t tick_{0};
};
