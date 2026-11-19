#pragma once

// The one place that turns end-effector targets into actuator-pair commands.
// Shared by the console CLI and the USB (dispatcher) module, so both see the
// same limits, the same IK reference convention and the same tracked state.
// All public methods are serialized by an internal mutex (the CLI thread and
// the USB RX worker call in concurrently).

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>

#include "base_component.hpp"
#include "can_bus.hpp"
#include "eyou_motor.hpp"
#include "ik_5bar.hpp"
#include "level_trajectory.hpp"
#include "operable_range.hpp"
#include "paired_eyou_actuator.hpp"

namespace stem {

enum class Pair : uint8_t { M1_Seat = 0, M2_Seat = 1, M3_Seat = 2 };
inline constexpr size_t kPairCount = 3;
inline constexpr size_t kMotorCount = 6;

const char *to_string(Pair pair);
std::optional<Pair> pair_from_name(std::string_view name);
std::optional<Pair> pair_from_index(uint8_t index);

struct MotorTelemetry {
  uint8_t id{0};
  bool ok{false}; ///< the status poll was answered
  uint16_t statusword{0};
  uint16_t error_code{0};
  float tracked_deg{0.0f};    ///< actual position read over CANopen (object 0x6064)
  int16_t torque_permille{0}; ///< actual torque (0x6077), per-thousand of rated
  bool sensor_ok{false};      ///< the output torque sensor (0x27BD) answered this poll
  float sensor_torque_nm{0.0f}; ///< output torque sensor (0x27BD, -F joints), Nm at the flange
  float velocity_deg_s{0.0f}; ///< actual velocity (0x606C)
};

/// The CAN controller as CanBus::get_status() reports it (TWAI error state and
/// counters): the quickest way to tell a wiring / bitrate problem from a motor
/// that is off. Zeroed when the bus never started.
struct CanTelemetry {
  bool started{false};
  bool bus_off{false};
  const char *state{"not started"};
  uint16_t tx_error_count{0};
  uint16_t rx_error_count{0};
  uint32_t bus_error_count{0};
  uint32_t bus_off_count{0};
  uint32_t signal_errors{0}; ///< stuff + form + bit errors: termination, ground, bitrate
  uint32_t ack_errors{0};    ///< frames nobody acknowledged: bus open / nodes off
};

// Snapshot of what the controller knows. Unlike the RMD-based STEM, the Eyou
// motors report their actual output-shaft position over CANopen (object
// 0x6064, relative to the software zero): pair_deg is MEASURED and the pose is
// the forward kinematics of those measured positions; "still moving" is the
// DS402 statusword (target reached), not a velocity.
struct State {
  bool can_ok{false};
  bool target_valid{false};
  float target_x{0.0f};
  float target_y{0.0f};
  bool pose_valid{false};
  float pose_x{0.0f};
  float pose_y{0.0f};
  std::array<float, kPairCount> pair_deg{}; ///< primary motor of each pair, Pair order
  float seat_tilt_deg{0.0f};                ///< tilt offset applied on top of the IK seat angle
  std::array<MotorTelemetry, kMotorCount> motors{};
  bool motors_ok{false};
  uint16_t torque_limit_permille{0}; ///< the limit last written to every motor
  uint32_t rated_torque_mnm{0};      ///< 0x6076 of the first motor that answered (0: unknown)
  bool measuring{false};             ///< measure mode (drives off, brakes hold, sensors read)
  bool brakes_engaged{false};        ///< engage_brakes(): drives off, nothing moves until released
  CanTelemetry can{};
};

// A solved end-effector target expressed as actuator-pair positions.
struct PoseCommand {
  float x_rel{0.0f};
  float y_rel{0.0f};
  std::array<float, kPairCount> pair_deg{}; ///< Pair order
  bool within_limits{false};
  IkSolution solution{};
};

class StemController : public espp::BaseComponent {
public:
  struct Config {
    PairedEyouActuator &m1_seat; ///< base motor M1 (motors 102 primary / 101)
    PairedEyouActuator &m2_seat; ///< base motor M2 (104 / 103)
    PairedEyouActuator &m3_seat; ///< seat tilt M3 (106 / 105)
    std::span<EyouMotor> motors; ///< every motor, for telemetry (ids reported as-is)
    bool can_ok{true};           ///< whether the CAN bus came up (reported in State)
    /// Per-pair hard position limits (primary motor, Pair order), written to the
    /// drives as their position clamp: the bounding box of the operable range
    /// plus ~5 deg (M1_Seat = -enc101 in [6, 90], M2_Seat = -enc103 in [-47, 12], M3_Seat
    /// = level +/- tilt over the whole range). Set pair moves are clamped to these.
    std::array<float, kPairCount> min_deg{-95.0f, -17.0f, -55.0f};
    std::array<float, kPairCount> max_deg{-1.0f, 52.0f, 155.0f};
    /// Pose moves (MOVE / HOME / tilt) must also land inside the operable range
    /// from the v3.1 reach study (operable_range.hpp).
    OperableRange operable{};
    float mirror_tolerance_deg{1.0f}; ///< max |primary + secondary| for a pair to be usable
    // Pose moves (MOVE / HOME) run as seat-level trajectories at the requested
    // rpm (pair output; 1 rpm = 6 deg/s on the fastest pair). Set pair moves still
    // use the drive profile below.
    float default_move_rpm{2.0f}; ///< MOVE / HOME speed when the caller passes 0
    float default_set_rpm{5.0f};  ///< SET pair speed when the caller passes 0
    float max_rpm{10.0f};         ///< requested speeds are clamped to (0, max_rpm]
    GeometryConfig geometry{};
    IkReference reference{kFullDownReference};
    /// Torque limit (0x6072, per-thousand of rated) written to every motor by
    /// set_torque_limit(); a request above max_torque_permille is refused, so
    /// the person who flashed the firmware bounds what a browser may ask for.
    uint16_t max_torque_permille{1000};
    /// Optional: the CAN bus, for the controller state in State::can. Null:
    /// the block reports "not started".
    CanBus *can_bus{nullptr};
    /// Optional: run before a STOP is enacted. A set-point stuck in the TWAI
    /// queue in front of the stop (nobody acknowledged it) would otherwise
    /// delay the stop until the driver gave up; CanBus::flush_pending() drops it.
    std::function<void()> flush_pending_commands{nullptr};
    /// The drive profile outside pose moves (Set pair, seat tilt, STOP), as main
    /// writes it at boot; restored when a pose move ends.
    EyouMotor::ProfilePositionConfig profile{60.0f, 20.0f, 20.0f};
    /// Pose moves: acceleration of the fastest pair along the path (deg/s^2).
    float move_accel_deg_s2{20.0f};
    /// Pose moves: speed cap of the fastest pair (deg/s), whatever rpm asks for.
    float max_move_speed_deg_s{18.0f};
    /// Seat-level guard during a pose move: the whole move slows as the
    /// measured seat drifts off the plan (full speed within level_hold_deg / 2,
    /// stopped at level_hold_deg, all pairs together), STOPs beyond
    /// level_abort_deg, or after hold_timeout_ms without progress.
    float level_hold_deg{1.0f};
    float level_abort_deg{4.0f};
    uint32_t hold_timeout_ms{3000};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::INFO};
  };

  enum class Result : uint8_t {
    Ok,
    Unreachable,
    OutOfLimits,
    ActuatorFailed,
    InvalidArgument,
    ZeroMismatch,
    Measuring,    ///< refused: measure mode is on (exit it first)
    BrakesEngaged ///< refused: the brakes are engaged (release them first)
  };
  static const char *to_string(Result result);

  explicit StemController(const Config &config);

  const Config &config() const { return config_; }

  /// Solve the IK for a target (relative to the reference) and map it to pair
  /// positions. No motion. Returns false if the point is unreachable; check
  /// `out.within_limits` for the position-limit test.
  bool solve(float x_rel, float y_rel, PoseCommand &out) const;

  /// Move the end effector. Unlike the raw actuator API this REJECTS targets
  /// whose pair angles fall outside the limits instead of silently clamping
  /// them (a clamped pose is not the pose that was asked for).
  /// Starts a seat-level trajectory (level_trajectory.hpp) and returns; the
  /// move runs in service(). A new move while one runs re-plans from where the
  /// running one is, at its speed (live drag).
  Result move_to(float x_rel, float y_rel, float rpm, PoseCommand *out = nullptr);
  /// Advance the running pose move: call every ~100 ms from a task. Streams
  /// each pair's next set-point, holds the path while the seat is off level
  /// or a pair lags, stops on the level guard, and ends the move.
  void service();
  /// A pose move is running.
  bool moving() const;
  Result home(float rpm, PoseCommand *out = nullptr);

  /// Drive one pair to an absolute position (clamped by the pair itself).
  Result set_pair(Pair pair, float degrees, float rpm);

  /// Seat tilt: an offset (pair degrees, positive = positive seat-pair
  /// rotation) added to the IK's level seat angle on every move, so the seat
  /// keeps the chosen tilt as the linkage moves. Re-commands the seat pair
  /// immediately; rejected if the resulting seat angle leaves the limits.
  Result set_seat_tilt(float tilt_deg, float rpm);
  float seat_tilt() const { return seat_tilt_deg_.load(); }

  /// Stop every pair where it is: halt (profile deceleration, drives stay
  /// enabled and HOLD the linkage), and disable the pairs whose halt was not
  /// confirmed, so a pair that did not answer coasts rather than keeps moving.
  /// The old behaviour disabled everything: the linkage then sagged under its
  /// own weight (found on the single-actuator stand - the drive has a halt).
  bool stop();
  bool release_brakes();
  /// The opposite of release_brakes(): stop any move, leave measure mode, and
  /// switch every drive off so the brakes hold. Moves, pair sets and tilt are
  /// refused (Result::BrakesEngaged) until release_brakes().
  bool engage_brakes();
  bool brakes_engaged() const { return brakes_engaged_.load(); }
  /// Measure mode, for reading the output torque sensors under load: stop,
  /// disable every drive so the brakes hold the pose (aborted, drives
  /// re-enabled, if any pair moves more than 0.5 deg - a brake that does not
  /// hold), then put every drive in `drive_mode` (13 CSF by default: 0x27BD may
  /// only update with the torque loop selected). Moves, pair sets, tilt and
  /// zero are refused while on. Off: mode 1 again and the drives re-enabled
  /// holding where they are (no jump).
  Result set_measure_mode(bool on, int8_t drive_mode = 13);
  bool measuring() const { return measuring_.load(); }
  /// Clear the DS402 Fault on every motor that reports one (controlword fault
  /// reset) and bring its pair back to Operation Enabled. A fault that comes
  /// straight back (its cause still present - an overload, a limit) leaves the
  /// motor faulted and returns false. Also run by release_brakes()' enable.
  bool reset_fault();
  /// Write the torque limit (0x6072, per-thousand of rated) to every motor.
  /// Refused above Config::max_torque_permille (Result::OutOfLimits); a motor
  /// that did not confirm makes it ActuatorFailed (the others keep the new
  /// value, State reports the one asked for).
  Result set_torque_limit(uint16_t torque_permille);
  /// The same in newton-metres, through the rated torque read from the motors
  /// (InvalidArgument when it is not known yet: nothing to scale by).
  Result set_torque_limit_nm(float torque_nm);
  uint16_t torque_limit() const { return torque_limit_permille_.load(); }
  /// Rated torque (0x6076, mNm) of the first motor that answered; 0 until one
  /// did. Read at the first set_torque_limit() / snapshot() and kept.
  uint32_t rated_torque_mnm() const { return rated_torque_mnm_.load(); }
  /// per-mille of rated -> Nm with the known rated torque (0 when unknown).
  float permille_to_nm(uint16_t permille) const {
    return static_cast<float>(permille) * static_cast<float>(rated_torque_mnm_.load()) / 1.0e6f;
  }
  /// Re-zero one pair (or all when `pair` is empty). Clears the tracked target.
  bool zero(std::optional<Pair> pair, bool align);
  /// Persist every pair's software zero to NVS.
  bool save_zero();
  /// Read every motor's saved software zero without changing active offsets.
  bool read_zero(std::array<std::array<int32_t, 2>, kPairCount> &zero_offset_pulses);
  /// Load every pair's software zero from NVS, then run check_pairs(). Clears the
  /// tracked target. Returns the NVS load result; see has_zero_mismatch().
  bool load_zero();
  /// Check every pair is mirrored within mirror_tolerance_deg. A mismatched pair
  /// rejects all move commands until a new zero removes the mismatch.
  bool check_pairs();
  bool has_zero_mismatch() const;

  State snapshot(bool read_motors);

  // ---- Pair balance (temporary bring-up tool, console "Data tour" panel) ----
  // Step ONE motor's profile-position target by a small angle, with every other
  // motor holding, and compare the output torque sensors before / after. Used to
  // find the angle where a pair's two motors stop fighting each other, then
  // set_motor_zero() re-zeroes that motor there. Red flags (the step made the
  // pair fight harder, shifted load, or disturbed another motor) undo the step
  // and latch: further steps are refused until cleared.
  struct NudgeReport {
    enum class Status : uint8_t { Ok = 0, Refused = 1, RedFlag = 2 };
    Status status{Status::Refused};
    uint8_t motor_id{0};
    float step_deg{0.0f};    ///< the step requested (0 = measure only)
    float session_deg{0.0f}; ///< this motor's accumulated steps since its last zero / reset
    std::array<uint8_t, kMotorCount> ids{};
    std::array<float, kMotorCount> sensor_before{}, sensor_after{};     ///< 0x27BD, Nm (averaged)
    std::array<float, kMotorCount> position_before{}, position_after{}; ///< deg (averaged)
    std::string message;
  };
  static constexpr uint8_t kNudgeClearRedFlag = 1 << 0;
  static constexpr uint8_t kNudgeResetSession = 1 << 1;
  static constexpr float kNudgeMaxStepDeg = 0.1f;   ///< per step
  static constexpr float kNudgeMaxTotalDeg = 1.0f;  ///< per motor, since its last zero / reset
  NudgeReport nudge(uint8_t motor_id, float step_deg, uint8_t flags);
  // Re-zero one motor so it reads exactly the mirror of its pair partner (where
  // the pair stopped fighting), save that zero to NVS, and hold there.
  Result set_motor_zero(uint8_t motor_id, float &zero_change_deg, std::string &message);

private:
  // stop() with mutex_ already held (pose-move aborts, commands that interrupt a move).
  bool stop_unlocked();
  PairedEyouActuator &actuator(Pair pair);
  const PairedEyouActuator &actuator(Pair pair) const;
  float effective_rpm(float rpm, float fallback) const;
  // Map an IK solution (angles relative to the reference) to the sign
  // convention the pairs are commanded in (seat tilt included).
  std::array<float, kPairCount> pose_to_pair_deg(const IkSolution &solution) const;
  bool check_pair_unlocked(Pair pair);
  // Re-check every flagged pair; true if any mismatch remains (mutex_ held).
  bool recheck_mismatch_unlocked();
  // Seat-level pose moves (mutex_ held).
  using Clock = std::chrono::steady_clock;
  struct Run {
    LevelTrajectory plan;
    float speed_deg_s{0.0f};
    float t{0.0f}; ///< plan time (s): advances with real time, frozen on hold
    Clock::time_point last{}, stalled_since{}, end_since{};
    float rate{1.0f}; ///< plan clock rate (0..1), lowered while the seat drifts
    bool slowed{false};
    bool at_end{false};
    bool settling{false}; ///< end reached, speeds zeroed: the next tick switches to the hold
    Clock::time_point settle_since{};
    uint8_t read_failures{0};
  };
  Result start_move_unlocked(float x_rel, float y_rel, float speed_deg_s);
  static constexpr float kVelocityAccel = 200.0f; ///< drive ramp between streamed speeds, deg/s^2
  static constexpr float kStopDecel = 120.0f;     ///< STOP / engage deceleration, deg/s^2
  void restore_profile_unlocked();
  // seat tilt from level of a pair triple; NAN when the cranks have no FK
  float tilt_of(const LevelTrajectory::Pairs &pairs) const;
  bool any_zero_mismatch_unlocked() const;

  Config config_;
  mutable std::mutex mutex_;
  std::array<bool, kPairCount> zero_mismatch_{};
  std::atomic<uint16_t> torque_limit_permille_{0}; // atomic: read by the lock-free torque_limit()
  std::atomic<uint32_t> rated_torque_mnm_{0};      // atomic: read by the lock-free getters
  // Output torque sensor polling (0x27BD): a motor whose read fails (a non -F
  // joint aborts the SDO) is retried only every kSensorRetryPolls snapshots, so it
  // neither floods the log nor slows telemetry. telemetry_mutex_ held.
  static constexpr uint8_t kSensorRetryPolls = 25;
  std::array<uint8_t, kMotorCount> sensor_skip_{};
  std::mutex telemetry_mutex_; // snapshot(): sensor_skip_ and one poll at a time
  // Read the rated torque once, from the first motor that answers (mutex_ held).
  void ensure_rated_torque_unlocked();
  std::atomic<float> seat_tilt_deg_{0.0f}; // atomic: read by the lock-free solve()
  bool target_valid_{false};
  std::optional<Run> run_;
  std::atomic<bool> measuring_{false};
  std::atomic<bool> brakes_engaged_{false};
  float target_x_{0.0f};
  float target_y_{0.0f};
  // pair balance (mutex_ held)
  EyouMotor *motor_by_id(uint8_t id, size_t *index = nullptr);
  bool pair_of_motor(uint8_t id, Pair &pair, bool &is_secondary) const;
  bool read_balance(std::array<float, kMotorCount> &sensor, std::array<float, kMotorCount> &position);
  bool wait_target_reached(EyouMotor &motor, uint32_t timeout_ms);
  std::array<float, kMotorCount> nudge_total_deg_{};
  bool nudge_red_flag_{false};
};

} // namespace stem
