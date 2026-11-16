#include "five_bar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fivebar {
namespace {

constexpr float kDegreesPerRad = 57.2957795130823208768f;

inline float degrees(float radians) { return radians * kDegreesPerRad; }
inline float radians(float deg) { return deg / kDegreesPerRad; }

/// z of (b - a) x (c - a): > 0 when c is on the CCW (left) side of a->b.
inline float cross2d(const Point &a, const Point &b, const Point &c) {
  return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
}

/// Proper intersection of the segments a-b and c-d (shared endpoints and
/// touching count as intersecting).
bool segments_intersect(const Point &a, const Point &b, const Point &c, const Point &d) {
  const float d1 = cross2d(c, d, a), d2 = cross2d(c, d, b);
  const float d3 = cross2d(a, b, c), d4 = cross2d(a, b, d);
  if (((d1 > 0.0f && d2 < 0.0f) || (d1 < 0.0f && d2 > 0.0f)) &&
      ((d3 > 0.0f && d4 < 0.0f) || (d3 < 0.0f && d4 > 0.0f)))
    return true;
  auto on_segment = [](const Point &p, const Point &q, const Point &r) {
    return std::min(p[0], q[0]) - 1e-6f <= r[0] && r[0] <= std::max(p[0], q[0]) + 1e-6f &&
           std::min(p[1], q[1]) - 1e-6f <= r[1] && r[1] <= std::max(p[1], q[1]) + 1e-6f;
  };
  if (std::fabs(d1) < 1e-6f && on_segment(c, d, a))
    return true;
  if (std::fabs(d2) < 1e-6f && on_segment(c, d, b))
    return true;
  if (std::fabs(d3) < 1e-6f && on_segment(a, b, c))
    return true;
  if (std::fabs(d4) < 1e-6f && on_segment(a, b, d))
    return true;
  return false;
}

} // namespace

bool loop_is_open(const Geometry &g, const Point &m1, const Point &m2, const Point &j1,
                  const Point &j2, const Point &j3) {
  // The loop M1 -> J2 -> J3 -> J1 -> M2 -> M1 is a simple polygon: no two
  // non-adjacent links touch. Testing the cranks alone let through poses where
  // the bar or the coupler folded back across a crank, or the loop turned
  // inside out, with the cranks still apart.
  if (segments_intersect(m1, j2, m2, j1)) // crank 1 - crank 2
    return false;
  if (segments_intersect(m1, j2, j3, j1)) // crank 1 - coupler
    return false;
  if (segments_intersect(j2, j3, m2, j1)) // bar - crank 2
    return false;
  if (segments_intersect(j2, j3, m2, m1)) // bar - ground (between the motor mounts)
    return false;
  if (segments_intersect(j3, j1, m2, m1)) // coupler - ground
    return false;
  // The end effector's arm past J3 (bar extension / offset) is part of the
  // bar: it must not reach back across a crank or the ground either.
  if (g.ee_extension != 0.0f || g.ee_offset != 0.0f) {
    const float bx = j3[0] - j2[0], by = j3[1] - j2[1];
    const float b = std::hypot(bx, by);
    if (b > 1e-9f) {
      const float ux = bx / b, uy = by / b;
      const Point m3 = {j2[0] + (g.bar + g.ee_extension) * ux - g.ee_offset * uy,
                        j2[1] + (g.bar + g.ee_extension) * uy + g.ee_offset * ux};
      if (segments_intersect(j3, m3, m1, j2) || segments_intersect(j3, m3, m2, j1) ||
          segments_intersect(j3, m3, m2, m1))
        return false;
    }
  }
  return true;
}

namespace {

/// The configured crank assembly, for the loop points (J3 closes the loop).
bool assembly_ok(const Geometry &g, const Point &m1, const Point &m2, const Point &j1,
                 const Point &j2, const Point &j3) {
  switch (g.assembly) {
  case Assembly::Crossed:
    // the STEM pin invariant (Geometry.md section 6): a sign condition, not a
    // segment test, because the crossing point legitimately slides past the
    // motor axes at the travel extremes
    return cross2d(m1, j2, j1) > 0.0f && cross2d(m2, j1, j2) < 0.0f;
  case Assembly::Open:
    return loop_is_open(g, m1, m2, j1, j2, j3);
  case Assembly::Any:
  default:
    return true;
  }
}

/// The configured fold of the bar/coupler dyad.
bool coupler_side_ok(const Geometry &g, const Point &j1, const Point &j2, const Point &j3) {
  const float c = cross2d(j1, j2, j3);
  return g.coupler_side == CouplerSide::Ccw ? c > 0.0f : c < 0.0f;
}

/// Fill a pose from the four loop points (M3 derives from the bar).
bool complete(const Geometry &g, const Point &m1, const Point &m2, const Point &j1, const Point &j2,
              const Point &j3, Pose &pose) {
  const float bx = j3[0] - j2[0], by = j3[1] - j2[1];
  const float b = std::hypot(bx, by);
  if (b < 1e-9f)
    return false;
  const float ux = bx / b, uy = by / b; // J2 -> J3
  const float nx = -uy, ny = ux;        // 90 deg CCW of it
  const float along = g.bar + g.ee_extension;
  pose.j1 = j1;
  pose.j2 = j2;
  pose.j3 = j3;
  pose.m3 = {j2[0] + along * ux + g.ee_offset * nx, j2[1] + along * uy + g.ee_offset * ny};
  pose.theta1_deg = degrees(std::atan2(j2[1] - m1[1], j2[0] - m1[0]));
  pose.theta2_deg = degrees(std::atan2(j1[1] - m2[1], j1[0] - m2[0]));
  pose.bar_angle_deg = degrees(std::atan2(uy, ux));
  return true;
}

} // namespace

float normalize_angle_deg(float angle_deg) {
  float angle = std::fmod(angle_deg, 360.0f);
  if (angle > 180.0f)
    angle -= 360.0f;
  if (angle <= -180.0f)
    angle += 360.0f;
  return angle;
}

bool is_valid(const Geometry &g) {
  if (!(g.crank1 > 0.0f) || !(g.crank2 > 0.0f) || !(g.bar > 0.0f) || !(g.coupler > 0.0f))
    return false;
  if (!std::isfinite(g.m2_x) || !std::isfinite(g.m2_y) || !std::isfinite(g.ee_extension) ||
      !std::isfinite(g.ee_offset))
    return false;
  const float base = std::hypot(g.m2_x, g.m2_y);
  if (base < 1e-6f)
    return false;
  // the two dyads must be able to reach a common point somewhere
  return base <= (g.crank1 + g.bar) + (g.crank2 + g.coupler);
}

bool solve_fk(float theta1_deg, float theta2_deg, const Geometry &g, Pose &pose) {
  if (!is_valid(g) || !std::isfinite(theta1_deg) || !std::isfinite(theta2_deg))
    return false;
  const Point m1 = {0.0f, 0.0f};
  const Point m2 = {g.m2_x, g.m2_y};
  const float t1 = radians(theta1_deg), t2 = radians(theta2_deg);
  const Point j2 = {m1[0] + g.crank1 * std::cos(t1), m1[1] + g.crank1 * std::sin(t1)};
  const Point j1 = {m2[0] + g.crank2 * std::cos(t2), m2[1] + g.crank2 * std::sin(t2)};
  // J3 closes the loop: |J3 - J1| = coupler and |J3 - J2| = bar
  const float dx = j2[0] - j1[0], dy = j2[1] - j1[1];
  const float d = std::hypot(dx, dy);
  if (d < 1e-9f || d > g.coupler + g.bar + 1e-3f || d < std::fabs(g.coupler - g.bar) - 1e-3f)
    return false;
  const float a = (d * d + g.coupler * g.coupler - g.bar * g.bar) / (2.0f * d);
  const float h = std::sqrt(std::max(g.coupler * g.coupler - a * a, 0.0f));
  const float vx = dx / d, vy = dy / d;
  // the two circle intersections mirror across J1->J2; the machine's fold picks one
  const float side = g.coupler_side == CouplerSide::Ccw ? 1.0f : -1.0f;
  const Point j3 = {j1[0] + a * vx - side * h * vy, j1[1] + a * vy + side * h * vx};
  if (h < 1e-6f)
    return false; // the dyad is folded flat onto J1-J2 (singular): no defined side
  if (!assembly_ok(g, m1, m2, j1, j2, j3))
    return false;
  return complete(g, m1, m2, j1, j2, j3, pose);
}

bool solve_ik(float x, float y, const Geometry &g, float reference_theta1_deg,
              float reference_theta2_deg, Pose &pose) {
  if (!is_valid(g) || !std::isfinite(x) || !std::isfinite(y))
    return false;
  const Point m1 = {0.0f, 0.0f};
  const Point m2 = {g.m2_x, g.m2_y};
  // M3 sits on the bar's rigid body: R from J2 at angle gamma from the bar
  const float along = g.bar + g.ee_extension;
  const float R = std::hypot(along, g.ee_offset);
  const float gamma = std::atan2(g.ee_offset, along);
  if (R < 1e-9f)
    return false;
  const float r = std::hypot(x, y);
  // |M3 + R v - M1| = crank1 with M1 at the origin gives the direction v of
  // M3 -> J2: p.v = (crank1^2 - R^2 - r^2) / (2 R)
  float cos_beta = 0.0f;
  float alpha = 0.0f;
  if (r < 1e-9f) {
    // M3 at M1: any direction with R == crank1 works; take +x (then the
    // reference picks among the J1 folds)
    if (std::fabs(R - g.crank1) > 1e-3f)
      return false;
    cos_beta = 1.0f;
  } else {
    const float dot = (g.crank1 * g.crank1 - R * R - r * r) / (2.0f * R);
    if (std::fabs(dot / r) > 1.0f + 1e-5f)
      return false; // out of reach of the crank1 + bar dyad
    cos_beta = std::clamp(dot / r, -1.0f, 1.0f);
    alpha = std::atan2(y, x);
  }
  const float beta = std::acos(cos_beta);
  const std::array<float, 2> v_angles = {alpha + beta, alpha - beta};

  Pose best{};
  float best_score = std::numeric_limits<float>::infinity();
  for (const float va : v_angles) {
    const Point j2 = {x + R * std::cos(va), y + R * std::sin(va)};
    // the bar direction u: M3 - J2 = R * rot(gamma) u  ->  u = rot(-gamma) * (-(v))
    const float u_angle = va + static_cast<float>(M_PI) - gamma;
    const Point j3 = {j2[0] + g.bar * std::cos(u_angle), j2[1] + g.bar * std::sin(u_angle)};
    // J1: the crank2 circle around M2 meets the coupler circle around J3
    const float dx = j3[0] - m2[0], dy = j3[1] - m2[1];
    const float d = std::hypot(dx, dy);
    if (d < 1e-9f)
      continue;
    const float a = (d * d + g.crank2 * g.crank2 - g.coupler * g.coupler) / (2.0f * d);
    const float h_sq = g.crank2 * g.crank2 - a * a;
    if (h_sq < -1e-2f)
      continue; // this bar direction cannot be closed by crank2 + coupler
    const float h = std::sqrt(std::max(h_sq, 0.0f));
    const float vx = dx / d, vy = dy / d;
    for (const float s : {1.0f, -1.0f}) {
      const Point j1 = {m2[0] + a * vx - s * h * vy, m2[1] + a * vy + s * h * vx};
      if (!assembly_ok(g, m1, m2, j1, j2, j3) || !coupler_side_ok(g, j1, j2, j3))
        continue;
      Pose candidate{};
      if (!complete(g, m1, m2, j1, j2, j3, candidate))
        continue;
      const float score =
          std::fabs(normalize_angle_deg(candidate.theta1_deg - reference_theta1_deg)) +
          std::fabs(normalize_angle_deg(candidate.theta2_deg - reference_theta2_deg));
      if (score < best_score) {
        best = candidate;
        best_score = score;
      }
    }
  }
  if (!std::isfinite(best_score))
    return false;
  pose = best;
  return true;
}

} // namespace fivebar
