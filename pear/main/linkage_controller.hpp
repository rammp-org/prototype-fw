#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <optional>

#include "base_component.hpp"
#include "timer.hpp"

#include "eyou_motor.hpp"
#include "linkage_geometry.hpp"

/// The LinkageController owns the four corner 4-bar linkage motors (Eyou,
/// CANopen DS402 profile position, software zero kept in NVS).
///
/// Targets are per-corner crank angles (deg, the HMI's L1..L4 debug rows and
/// the CLI's `link`) or a base pose (elevation / front-back tilt / lateral
/// tilt: the HMI's M1..M3 rows and the CLI's `pose`), turned into corner
/// angles with pear::geometry. Every request is clamped to the seat-axis
/// table's range for that corner AND to the geometry's angle range, then
/// queued; the controller's own task enacts it (EyouMotor::move_absolute, a
/// confirmed SDO transaction) so an RTPS callback never blocks on the bus.
///
/// Interlocks (a refused request returns false and is logged, rate-limited):
///   * the drive controller must not be in DRIVE (Config::drive_enabled; the
///     spec only lets the HMI command seat rows while the MIB is IDLE, and the
///     CLI gets the same rule),
///   * every linkage motor must be enabled and un-faulted (a pose moves all
///     four; a single corner still needs the whole set healthy, so the base
///     cannot be twisted against a dead corner).
///
/// A status task polls one motor per tick (round-robin: position, statusword,
/// fault) and keeps State; a faulted motor is reported with its corner so the
/// RTPS link can raise MibStatus ERROR naming it.
///
/// All public methods are thread-safe. zero() / save_zero() / load_zero() are
/// confirmed transactions (they block the caller for the SDO / NVS time).
class LinkageController : public espp::BaseComponent {
public:
  using Corner = pear::Corner;
  using Pose = pear::geometry::Pose;
  using Corners = pear::geometry::Corners;

  /// Last known status of one linkage motor.
  struct MotorStatus {
    uint8_t id{0};                 ///< CAN node id.
    float position_degrees{0.0f};  ///< Last read position (relative to the zero).
    float target_degrees{0.0f};    ///< Last commanded target.
    uint16_t statusword{0};        ///< Last read DS402 statusword.
    bool valid{false};             ///< True once a status read has succeeded.
    bool stale{true};              ///< True if not successfully read recently.
    bool faulted{false};           ///< DS402 fault bit (or a failed read).
    bool operation_enabled{false}; ///< DS402 Operation Enabled.
    bool zero_loaded{false};       ///< A software zero is active (NVS or `zero`).
    bool target_pending{false};    ///< A move is queued for the task.
  };

  /// Snapshot of the controller state.
  struct State {
    Corners angles_deg{}; ///< Measured corner angles (last reads).
    Pose pose{};          ///< The pose those angles describe (elevation in m).
    std::array<MotorStatus, pear::kCornerCount> motors{};
    bool enabled{false}; ///< Every motor enabled, un-faulted and read at least once.
    bool any_faulted{false};
    bool initialized{false}; ///< initialize() succeeded for every motor.
    /// Age of the last move command (65535 = none yet).
    uint16_t command_age_ms{65535};
  };

  struct Config {
    /// The linkage motors in pear::Corner order (FR, FL, BR, BL).
    // cppcheck-suppress uninitMemberVarNoCtor
    std::array<EyouMotor, pear::kCornerCount> &motors;
    pear::geometry::Geometry geometry{};        ///< 4-bar model + angle range.
    EyouMotor::ProfilePositionConfig profile{}; ///< Motion profile for every motor.
    /// Min / max angle (deg) the RTPS seat-axis row of each corner allows; the
    /// request clamp is the intersection with the geometry's range.
    Corners table_min_deg{};
    Corners table_max_deg{};
    /// True while the base is driving (DRIVE); linkage moves are refused then.
    std::function<bool()> drive_enabled{nullptr};
    std::chrono::milliseconds status_poll_period{250};    ///< Round-robin poll period.
    std::chrono::milliseconds status_stale_timeout{3000}; ///< Age before a status is stale.
    uint32_t status_read_timeout_ms{50};                  ///< Per-read reply timeout.
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN};
  };

  explicit LinkageController(const Config &config);
  ~LinkageController();

  LinkageController(const LinkageController &) = delete;
  LinkageController &operator=(const LinkageController &) = delete;

  /// Bring every motor up (STEM_V3's sequence): configure_profile_position ->
  /// enable_drive -> load_zero (a missing saved zero is a warning: the motor
  /// then works on its absolute encoder until `zero` / `load_zero`) ->
  /// set_position_limits. Blocking (SDO transactions). \return true if every
  /// motor is configured and enabled.
  bool initialize();

  /// Queue a move of one corner to an absolute crank angle (deg). Clamped;
  /// refused (false) while driving or with a motor not enabled / faulted.
  bool request_corner_angle(Corner corner, float angle_deg);
  /// Queue a move of all four corners to the angles a base pose needs
  /// (elevation m, tilts deg). Each corner is clamped on its own, so a pose
  /// outside the reach is approximated. Same refusals as above.
  bool request_pose(const Pose &pose);
  /// One component of the pose the HMI's seat rows adjust.
  enum class PoseComponent { ELEVATION, FB_TILT, LATERAL_TILT };
  /// Replace one component of the DESIRED pose (the last requested pose, or
  /// the measured pose when none was requested since a corner moved on its
  /// own) and request the result: successive single-axis requests therefore
  /// accumulate instead of each one rebuilding the pose from stale
  /// measurements and discarding the others.
  bool request_pose_component(PoseComponent component, float value);

  /// Set the software zero of one corner (or all) to the motor's current
  /// position (confirmed; the active offset only - see save_zero()).
  bool zero(std::optional<Corner> corner = std::nullopt);
  /// Persist every motor's software zero to NVS.
  bool save_zero();
  /// Load every motor's software zero from NVS.
  bool load_zero();

  /// The clamp range of a corner (table range intersected with the geometry).
  float min_angle_deg(Corner corner) const;
  float max_angle_deg(Corner corner) const;

  /// Snapshot of the current state. Thread-safe.
  State state() const;
  /// True while a batch of queued moves is being sent to the motors OR a
  /// commanded corner has not yet reported DS402 target reached (profile
  /// position bit 10): the drive side of the interlock (DriveController::enable
  /// refuses then). Lock-free: enable() calls it with its own mutex held.
  bool moving() const { return enacting_.load() || moving_mask_.load() != 0; }

protected:
  bool status_step();
  /// Why a request would be refused right now (nullptr = allowed). mutex_ held.
  const char *refusal_reason_locked() const;
  /// Clamp a requested angle to the corner's range (non-finite -> refused).
  bool clamp_target(Corner corner, float &angle_deg);
  /// Drop queued corner targets and the desired pose (mutex_ held).
  void invalidate_targets_locked();
  /// Clamp, check the interlocks and queue every corner of `pose` (mutex_ held).
  bool commit_pose_locked(const Pose &pose);
  /// The pose the measured corner angles describe (mutex_ held).
  Pose measured_pose_locked() const;
  void note_command_sent_locked();

  std::array<EyouMotor, pear::kCornerCount> &motors_;
  pear::geometry::Geometry geometry_;
  EyouMotor::ProfilePositionConfig profile_;
  Corners min_deg_{};
  Corners max_deg_{};
  std::function<bool()> drive_enabled_;
  std::chrono::milliseconds status_stale_timeout_;
  uint32_t status_read_timeout_ms_;

  mutable std::mutex mutex_;
  /// Serializes move execution (status_step's enact phase) with a change of
  /// the software zero (zero / load_zero): a target copied before the zero
  /// moved must not execute in the new coordinate system. Locked BEFORE mutex_.
  std::mutex move_mutex_;
  std::atomic<bool> enacting_{false}; ///< moving(): a batch is being sent
  /// Corners commanded and not yet reporting DS402 target reached (bit =
  /// corner index): the physical-motion half of moving(). Set under mutex_
  /// when a move is sent, cleared by the poll on target reached / fault /
  /// not enabled, or by kMoveTimeout.
  std::atomic<uint8_t> moving_mask_{0};
  std::chrono::steady_clock::time_point motion_started_{};
  static constexpr std::chrono::seconds kMoveTimeout{15};
  std::array<MotorStatus, pear::kCornerCount> motor_status_{};
  std::array<std::chrono::steady_clock::time_point, pear::kCornerCount> motor_status_time_{};
  Corners pending_target_deg_{};
  std::array<bool, pear::kCornerCount> pending_{};
  std::optional<Pose> desired_pose_{}; ///< last requested pose; reset by a corner move
  bool initialized_{false};
  std::chrono::steady_clock::time_point last_command_{};
  bool have_command_{false};
  size_t status_poll_index_{0};

  espp::Timer status_timer_;
};
