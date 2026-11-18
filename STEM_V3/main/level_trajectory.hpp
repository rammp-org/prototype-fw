#pragma once

// Seat-level pose moves.
//
// A pose move used to send each pair its final angle at once. The pairs then
// travelled at the same drive profile speed over different distances, so they
// arrived at different times and the seat tilted in between (up to ~15 deg on a
// long move). This planner instead builds the move as a dense path of pair
// angles that keeps the seat at its commanded tilt at EVERY point:
//
//   1. level first: if the seat is off its commanded tilt at the start, move
//      the seat pair alone until it is (left / right hold);
//   2. then a straight line of the seat axis M3 from where it is to the target,
//      sampled so no pair moves more than max_step_deg between samples, each
//      sample solved with the IK (seat angle = level + tilt) and checked
//      against the pair limits and the operable range.
//
// The path is time-scaled with one trapezoidal profile on the fastest-moving
// pair, so every pair moves together and arrives together; at(t) gives the
// three pair angles at time t. Pure math: no drives, host-tested
// (test/level_trajectory_host_test.cpp).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "ik_5bar.hpp"
#include "operable_range.hpp"

namespace stem {

struct LevelTrajectoryLimits {
  float speed_deg_s{12.0f};    ///< fastest pair's cruise speed
  float accel_deg_s2{20.0f};   ///< fastest pair's acceleration / deceleration
  float max_step_deg{0.25f};   ///< max pair change between path samples
  float cart_step_mm{2.0f};    ///< initial M3 sampling of the straight line
  float start_match_deg{1.5f}; ///< IK at the start vs the measured left / right
};

class LevelTrajectory {
public:
  using Pairs = std::array<float, 3>; ///< left, right, seat (pair degrees)

  enum class Result : uint8_t { Ok, StartUnknown, Unreachable, OutOfLimits };

  /// Plan from `start` (pair degrees) to the target M3 (relative to the IK
  /// reference) at seat tilt `tilt_deg`. `initial_speed_deg_s` lets a re-plan
  /// (live drag, tilt change) continue at the speed it was moving. On failure
  /// `fail_x / fail_y` hold the first rejected path point (reference frame).
  Result plan(const Pairs &start, float x_rel, float y_rel, float tilt_deg,
              const IkReference &reference, const GeometryConfig &geometry,
              const OperableRange &operable, const std::array<float, 3> &min_deg,
              const std::array<float, 3> &max_deg, const LevelTrajectoryLimits &limits,
              float initial_speed_deg_s = 0.0f) {
    q_.clear();
    u_.clear();
    q_.push_back(start);
    fail_x = x_rel;
    fail_y = y_rel;

    // where the seat axis is now
    IkSolution fk{};
    if (!solve_fk_reference(-start[0], -start[1], reference, fk, geometry)) {
      return Result::StartUnknown;
    }
    const float x0 = fk.m3[0] - reference.x, y0 = fk.m3[1] - reference.y;
    // an operable-range check only makes sense from inside it: a start left
    // outside (by a Set pair) may still move back in
    const bool start_inside = operable.base_contains(start[0] * -1.0f, start[1] * -1.0f);

    // 1. level first: seat pair alone to (level + tilt) at the start pose
    const float level_seat = -fk.m3_angle_deg + tilt_deg;
    append_line(start, {start[0], start[1], level_seat}, limits.max_step_deg);

    // 2. straight M3 line, every sample on the IK's level seat
    const float length = std::hypot(x_rel - x0, y_rel - y0);
    const int n = std::max(1, static_cast<int>(std::ceil(length / limits.cart_step_mm)));
    Pairs previous = q_.back();
    float previous_s = 0.0f;
    for (int k = 1; k <= n; ++k) {
      const float s = static_cast<float>(k) / static_cast<float>(n);
      Pairs pairs{};
      const Result r = solve_point(x0, y0, x_rel, y_rel, s, tilt_deg, reference, geometry,
                                   operable, min_deg, max_deg, start_inside, pairs);
      if (r != Result::Ok) {
        return r;
      }
      if (k == 1 && (std::fabs(pairs[0] - start[0]) > limits.start_match_deg + max_abs_step(previous, pairs) ||
                     std::fabs(pairs[1] - start[1]) > limits.start_match_deg + max_abs_step(previous, pairs))) {
        return Result::StartUnknown; // the IK branch disagrees with the measured cranks
      }
      // refine where a 2 mm step moves a pair more than max_step_deg
      const int sub = std::max(1, static_cast<int>(std::ceil(max_abs_step(previous, pairs) /
                                                             limits.max_step_deg)));
      for (int j = 1; j < sub; ++j) {
        const float sj = previous_s + (s - previous_s) * static_cast<float>(j) / static_cast<float>(sub);
        Pairs mid{};
        const Result rm = solve_point(x0, y0, x_rel, y_rel, sj, tilt_deg, reference, geometry,
                                      operable, min_deg, max_deg, start_inside, mid);
        if (rm != Result::Ok) {
          return rm;
        }
        q_.push_back(mid);
      }
      q_.push_back(pairs);
      previous = pairs;
      previous_s = s;
    }
    end_ = q_.back();

    // path length in "fastest pair degrees"
    u_.resize(q_.size());
    u_[0] = 0.0f;
    for (size_t i = 1; i < q_.size(); ++i) {
      u_[i] = u_[i - 1] + max_abs_step(q_[i - 1], q_[i]);
    }
    scale_time(u_.back(), limits.speed_deg_s, limits.accel_deg_s2, initial_speed_deg_s);
    return Result::Ok;
  }

  float duration() const { return t_total_; }
  const Pairs &end() const { return end_; }
  size_t samples() const { return q_.size(); }

  /// Path speed (fastest pair, deg/s) at time t.
  float speed_at(float t) const {
    if (t <= 0.0f) {
      return v0_;
    }
    if (t < t1_) {
      return v0_ + (vp_ > v0_ ? a_ : -a_) * t;
    }
    if (t < t2_) {
      return vp_;
    }
    if (t < t_total_) {
      return vp_ - a_ * (t - t2_);
    }
    return 0.0f;
  }

  /// The three pair angles at time t (clamped to the path's ends).
  Pairs at(float t) const {
    if (q_.empty()) {
      return {};
    }
    const float u = u_at(t);
    const auto it = std::upper_bound(u_.begin(), u_.end(), u);
    if (it == u_.end()) {
      return q_.back();
    }
    const size_t i = static_cast<size_t>(it - u_.begin());
    if (i == 0) {
      return q_.front();
    }
    const float du = u_[i] - u_[i - 1];
    const float f = du > 1e-9f ? (u - u_[i - 1]) / du : 1.0f;
    Pairs out{};
    for (size_t k = 0; k < 3; ++k) {
      out[k] = q_[i - 1][k] + (q_[i][k] - q_[i - 1][k]) * f;
    }
    return out;
  }

  float fail_x{0.0f}, fail_y{0.0f};

private:
  static float max_abs_step(const Pairs &a, const Pairs &b) {
    return std::max({std::fabs(b[0] - a[0]), std::fabs(b[1] - a[1]), std::fabs(b[2] - a[2])});
  }

  void append_line(const Pairs &from, const Pairs &to, float max_step) {
    const int n = static_cast<int>(std::ceil(max_abs_step(from, to) / max_step));
    for (int k = 1; k <= n; ++k) {
      const float f = static_cast<float>(k) / static_cast<float>(n);
      q_.push_back({from[0] + (to[0] - from[0]) * f, from[1] + (to[1] - from[1]) * f,
                    from[2] + (to[2] - from[2]) * f});
    }
  }

  Result solve_point(float x0, float y0, float x1, float y1, float s, float tilt_deg,
                     const IkReference &reference, const GeometryConfig &geometry,
                     const OperableRange &operable, const std::array<float, 3> &min_deg,
                     const std::array<float, 3> &max_deg, bool check_operable, Pairs &pairs) {
    const float x = x0 + (x1 - x0) * s, y = y0 + (y1 - y0) * s;
    IkSolution sol{};
    if (!solve_ik_for_m3_reference(x, y, reference, sol, geometry)) {
      fail_x = x;
      fail_y = y;
      return Result::Unreachable;
    }
    pairs = {-sol.theta1_deg, -sol.theta2_deg, -sol.m3_angle_deg + tilt_deg};
    bool ok = true;
    for (size_t i = 0; i < 3; ++i) {
      ok = ok && pairs[i] >= min_deg[i] && pairs[i] <= max_deg[i];
    }
    if (check_operable) {
      ok = ok && operable.contains(pairs, tilt_deg);
    }
    if (!ok) {
      fail_x = x;
      fail_y = y;
      return Result::OutOfLimits;
    }
    return Result::Ok;
  }

  // Trapezoid on u: from v0 to the peak at a, cruise, down to 0 at a.
  void scale_time(float total, float v, float a, float v0) {
    a_ = std::max(a, 1e-3f);
    v = std::max(v, 1e-3f);
    total_ = std::max(total, 0.0f);
    // cannot stop in time from v0: start slower (the drives absorb the step)
    v0_ = std::clamp(v0, 0.0f, std::min(v, std::sqrt(2.0f * a_ * total_)));
    // peak reachable: accelerate v0 -> vp, decelerate vp -> 0 within total
    const float vp_max = std::sqrt(a_ * total_ + 0.5f * v0_ * v0_);
    vp_ = std::max(std::min(v, vp_max), v0_);
    const float d_acc = (vp_ * vp_ - v0_ * v0_) / (2.0f * a_);
    const float d_dec = vp_ * vp_ / (2.0f * a_);
    const float d_cruise = std::max(total_ - d_acc - d_dec, 0.0f);
    t1_ = (vp_ - v0_) / a_;
    t2_ = t1_ + d_cruise / vp_;
    t_total_ = t2_ + vp_ / a_;
    u1_ = d_acc;
    u2_ = d_acc + d_cruise;
  }

  float u_at(float t) const {
    if (t <= 0.0f) {
      return 0.0f;
    }
    if (t < t1_) {
      return v0_ * t + 0.5f * a_ * t * t;
    }
    if (t < t2_) {
      return u1_ + vp_ * (t - t1_);
    }
    if (t < t_total_) {
      const float d = t - t2_;
      return std::min(u2_ + vp_ * d - 0.5f * a_ * d * d, total_);
    }
    return total_;
  }

  std::vector<Pairs> q_;
  std::vector<float> u_;
  Pairs end_{};
  float total_{0.0f}, a_{1.0f}, v0_{0.0f}, vp_{0.0f};
  float t1_{0.0f}, t2_{0.0f}, t_total_{0.0f}, u1_{0.0f}, u2_{0.0f};
};

} // namespace stem
