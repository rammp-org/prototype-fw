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
    /// The measured angle is outside [min, max] (+ tolerance): the corner
    /// refuses every move except a jog; MibStatus reports an ERROR.
    bool out_of_range{false};
  };

  /// Snapshot of the controller state.
  struct State {
    Corners angles_deg{}; ///< Measured corner angles (last reads).
    Pose pose{};          ///< The pose those angles describe (elevation in m).
    std::array<MotorStatus, pear::kCornerCount> motors{};
    Corners min_deg{}; ///< Per-corner range (the request clamp), deg.
    Corners max_deg{};
    bool enabled{false}; ///< Every motor enabled, un-faulted and read at least once.
    bool any_faulted{false};
    uint8_t out_of_range_mask{0}; ///< Corners outside their range (bit = corner index).
    bool initialized{false};      ///< initialize() succeeded for every motor.
    /// Corners whose halt (power stage off after a timed-out or abandoned
    /// move) has not been confirmed by the motor (bit = corner index): the
    /// move may still be running, the drive interlock stays asserted and the
    /// halt is retried every status step until it confirms.
    uint8_t halt_unconfirmed_mask{0};
    /// Age of the last move command (65535 = none yet).
    uint16_t command_age_ms{65535};
  };

  struct Config {
    /// The linkage motors in pear::Corner order (FR, FL, BR, BL).
    // cppcheck-suppress uninitMemberVarNoCtor
    std::array<EyouMotor, pear::kCornerCount> &motors;
    pear::geometry::Geometry geometry{};        ///< 4-bar model + angle range.
    EyouMotor::ProfilePositionConfig profile{}; ///< Motion profile for every motor.
    /// Per-corner request clamp (deg, "positive = up"), within the geometry's.
    Corners min_deg{};
    Corners max_deg{};
    /// +1 / -1 per corner: the motor's positive direction raises / lowers the
    /// module. Every angle read from or sent to a motor goes through it.
    Corners direction{};
    /// Largest jog (deg) for a corner outside its range; see request_jog().
    float jog_max_deg{1.0f};
    /// How far past a limit still counts as inside it (deg).
    float range_tolerance_deg{0.1f};
    /// Min / max angle (deg) the RTPS seat-axis row of each corner allows; the
    /// HMI's display / jog range; the request clamp is the geometry's range,
    /// and a row that does not cover it is warned about at construction.
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
  /// A relative move of at most Config::jog_max_deg from the corner's measured
  /// angle, NOT clamped to the range: the only move accepted for a corner
  /// that sits outside its range (to bring it back), usable in range as well.
  /// Needs a fresh position; otherwise refused like any other move.
  bool request_jog(Corner corner, float delta_deg);
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
  /// mutex_ held.
  bool clamp_target_locked(Corner corner, float &angle_deg);
  /// The corner's measured angle is outside its range (mutex_ held).
  bool out_of_range_locked(Corner corner) const;
  /// Queue a target for one corner (mutex_ held): the shared tail of
  /// request_corner_angle / request_jog.
  bool queue_corner_locked(Corner corner, float angle_deg, const char *what);
  /// Angle conversions through the corner's mounting direction.
  float to_motor_deg(Corner corner, float logical_deg) const {
    return direction_[corner] * logical_deg;
  }
  float from_motor_deg(Corner corner, float motor_deg) const {
    return direction_[corner] * motor_deg;
  }
  /// Drop queued corner targets and the desired pose (mutex_ held).
  void invalidate_targets_locked();
  /// Clamp, check the interlocks and queue every corner of `pose` (mutex_ held).
  bool commit_pose_locked(const Pose &pose);
  /// The pose the measured corner angles describe (mutex_ held).
  Pose measured_pose_locked() const;
  void note_command_sent_locked();
  /// Halt a commanded move: power stage off (the Eyou's halt), then on again so
  /// the motor is ready for the next request. The corner's moving_mask_ bit is
  /// cleared ONLY when the disable was confirmed; otherwise it stays latched
  /// (the interlock holds) and the halt is retried on the next status step.
  /// Returns true when the halt was confirmed.
  bool halt_corner(Corner corner, const char *why);

  std::array<EyouMotor, pear::kCornerCount> &motors_;
  pear::geometry::Geometry geometry_;
  EyouMotor::ProfilePositionConfig profile_;
  Corners min_deg_{};
  Corners max_deg_{};
  Corners direction_{};
  float jog_max_deg_{1.0f};
  float range_tolerance_deg_{0.1f};
  std::function<bool()> drive_enabled_;
  std::chrono::milliseconds status_stale_timeout_;
  uint32_t status_read_timeout_ms_;

  mutable std::mutex mutex_;
  /// Serializes move execution (status_step's enact phase) with a change of
  /// the software zero (zero / load_zero): a target copied before the zero
  /// moved must not execute in the new coordinate system. Locked BEFORE mutex_.
  /// A rebase additionally holds mutex_ for its whole duration (no request is
  /// accepted against old-frame positions), bumps frame_generation_ (a poll
  /// that sampled the motor in the old frame discards its sample) and marks
  /// the cached positions invalid until the next successful poll.
  std::mutex move_mutex_;
  std::atomic<uint32_t> frame_generation_{0};
  /// initialize() has run (whatever its result): the status poll starts only
  /// then, so it never hits a motor while it is being reset / configured
  /// (those reads timed out and looked like bus trouble).
  std::atomic<bool> init_finished_{false};
  std::atomic<bool> enacting_{false}; ///< moving(): a batch is being sent
  /// Corners commanded and not yet reporting DS402 target reached (bit =
  /// corner index): the physical-motion half of moving(). Set under mutex_
  /// when a move is sent, cleared by the poll on target reached / fault /
  /// not enabled, or by kMoveTimeout.
  std::atomic<uint8_t> moving_mask_{0};
  /// Corners whose halt did not confirm yet (subset of moving_mask_); the
  /// failure is logged once per corner, retries are quiet.
  std::atomic<uint8_t> halt_unconfirmed_mask_{0};
  /// Per corner: when its current move was accepted (each moving bit times
  /// out on its own; a later command to another corner must not postpone it).
  std::array<std::chrono::steady_clock::time_point, pear::kCornerCount> motion_started_{};
  static constexpr std::chrono::seconds kMoveTimeout{15};
  std::array<MotorStatus, pear::kCornerCount> motor_status_{};
  std::array<std::chrono::steady_clock::time_point, pear::kCornerCount> motor_status_time_{};
  Corners pending_target_deg_{};
  std::array<bool, pear::kCornerCount> pending_{};
  std::optional<Pose> desired_pose_{}; ///< last requested pose; reset by a corner move
  /// Bumped (mutex_ held) whenever a request is queued. A batch remembers the
  /// generation it was copied from, so when it is dropped or abandoned it can
  /// clear the desired pose it was meant to reach WITHOUT erasing a newer
  /// request queued meanwhile.
  uint32_t request_generation_{0};
  /// Drop the desired pose of the batch of `generation` (mutex_ held) unless
  /// a newer request has been queued since.
  void forget_batch_pose_locked(uint32_t generation, const char *why);
  bool initialized_{false};
  std::chrono::steady_clock::time_point last_command_{};
  bool have_command_{false};
  size_t status_poll_index_{0};

  espp::Timer status_timer_;
};
