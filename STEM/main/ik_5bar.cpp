#include "ik_5bar.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace stem {
namespace {

using Point = std::array<float, 2>;

constexpr float kDegreesPerRad = 57.2957795130823208768f;

inline float degrees(float radians) {
  return radians * kDegreesPerRad;
}

inline float radians(float degrees) {
  return degrees / kDegreesPerRad;
}

inline float cross2d(const Point &a, const Point &b, const Point &c) {
  return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
}

// The physical pins at J1 / J2 keep links 1 and 2 crossed; this is the sign
// invariant from Geometry.md section 6 (NOT a segment-intersection test).
inline bool is_crossed_branch(const Point &m1, const Point &m2, const Point &j1, const Point &j2) {
  return cross2d(m1, j2, j1) > 0.0f && cross2d(m2, j1, j2) < 0.0f;
}

// The coupler / seat-bar sub-loop (J1-J3-J2) also has two assembly modes that
// the pin invariant cannot tell apart: J3 (and with it the whole seat bar) on
// the CCW side of J1->J2, or mirrored underneath it. The machine is built in
// the CCW mode at every documented pose (the mirrored mode would put the seat
// bar through the base), and it cannot flip without disassembly, so both
// solvers reject the mirrored mode.
inline bool is_bar_side_ok(const Point &j1, const Point &j2, const Point &j3) {
  return cross2d(j1, j2, j3) > 0.0f;
}

// Fill in everything derived from the four loop points (J5, the crank angles
// and the bar/plate angle). Shared by the IK and FK solvers so both report the
// same fields. Returns false only for a degenerate (zero-length) bar.
bool complete_solution(const Point &m1, const Point &m2, const Point &m3, const Point &j1,
                       const Point &j2, const Point &j3, const GeometryConfig &config,
                       IkSolution &solution) {
  const Point bar_dir = {j2[0] - m3[0], j2[1] - m3[1]};
  const float bar_mag = std::hypot(bar_dir[0], bar_dir[1]);
  if (bar_mag < 1e-9f) {
    return false;
  }
  const float ux = bar_dir[0] / bar_mag;
  const float uy = bar_dir[1] / bar_mag;
  // J5 is offset 90 deg CCW from the M3->J2 direction
  const float nx = -uy;
  const float ny = ux;

  solution.m3 = m3;
  solution.j1 = j1;
  solution.j2 = j2;
  solution.j3 = j3;
  solution.j5 = {
      m3[0] + config.bar_length_m3_to_j5 * ux + config.bar_length_j5_perp_offset * nx,
      m3[1] + config.bar_length_m3_to_j5 * uy + config.bar_length_j5_perp_offset * ny,
  };
  solution.theta1_deg = degrees(std::atan2(j2[1] - m1[1], j2[0] - m1[0]));
  solution.theta2_deg = degrees(std::atan2(j1[1] - m2[1], j1[0] - m2[0]));
  solution.phi1_deg = solution.theta1_deg;
  solution.phi2_deg = solution.theta2_deg;
  solution.plate_angle_deg = degrees(std::atan2(m3[1] - j2[1], m3[0] - j2[0]));
  solution.m3_angle_deg = solution.plate_angle_deg;
  solution.x_ref = 0.0f;
  solution.y_ref = 0.0f;
  solution.m1_angle_ref_deg = 0.0f;
  solution.m2_angle_ref_deg = 0.0f;
  solution.m3_angle_ref_deg = 0.0f;
  return true;
}

// Re-express an absolute solution relative to the reference pose.
IkSolution apply_reference(const IkSolution &raw, const IkReference &reference) {
  IkSolution transformed = raw;
  transformed.x_ref = reference.x;
  transformed.y_ref = reference.y;
  transformed.m1_angle_ref_deg = reference.m1_angle_deg;
  transformed.m2_angle_ref_deg = reference.m2_angle_deg;
  transformed.m3_angle_ref_deg = reference.m3_angle_deg;
  transformed.theta1_deg = normalize_angle_deg(raw.theta1_deg - reference.m1_angle_deg);
  transformed.theta2_deg = normalize_angle_deg(raw.theta2_deg - reference.m2_angle_deg);
  transformed.phi1_deg = normalize_angle_deg(raw.phi1_deg - reference.m1_angle_deg);
  transformed.phi2_deg = normalize_angle_deg(raw.phi2_deg - reference.m2_angle_deg);
  transformed.plate_angle_deg = normalize_angle_deg(raw.plate_angle_deg - reference.m3_angle_deg);
  transformed.m3_angle_deg = normalize_angle_deg(raw.m3_angle_deg - reference.m3_angle_deg);
  return transformed;
}

} // namespace

float normalize_angle_deg(float angle_deg) {
  float angle = std::fmod(angle_deg, 360.0f);
  if (angle >= 180.0f) {
    angle -= 360.0f;
  }
  if (angle < -180.0f) {
    angle += 360.0f;
  }
  return angle;
}

bool solve_ik_for_m3(float x_rel, float y_rel, IkSolution &solution, const GeometryConfig &config,
                    const IkReference &branch_reference) {
  const Point m1 = {config.m1_x, config.m1_y};
  const Point m2 = {config.m2_x, config.m2_y};

  const Point m3 = {x_rel, y_rel};
  const float r = std::hypot(m3[0], m3[1]);
  if (r < 1e-9f) {
    return false;
  }

  const float l1 = config.crank_length_m1_to_j2;
  const float l2 = config.crank_length_m2_to_j1;
  const float l3 = config.bar_length_j1_to_j3;
  const float l4 = config.bar_length_m3_to_j2;
  const float l4_j3 = config.bar_length_m3_to_j3;

  // |M3 + L4*u - M1| = L1 with M1 at the origin gives the bar direction u:
  // p.u = (L1^2 - L4^2 - |p|^2) / (2*L4), u = (cos(alpha +/- beta), sin(...)).
  const float dot_p_u = (l1 * l1 - l4 * l4 - r * r) / (2.0f * l4);
  // |cos| > 1 means no bar direction puts J2 on the M1 crank circle: the point
  // is out of reach. Only absorb float round-off, never a real overshoot, or
  // the "solution" would violate the link lengths.
  if (std::fabs(dot_p_u / r) > 1.0f + 1e-5f) {
    return false;
  }
  const float cos_beta = std::clamp(dot_p_u / r, -1.0f, 1.0f);
  const float alpha = std::atan2(m3[1], m3[0]);
  const float beta = std::acos(cos_beta);

  const std::array<Point, 2> candidate_dirs = {{
      {std::cos(alpha + beta), std::sin(alpha + beta)},
      {std::cos(alpha - beta), std::sin(alpha - beta)},
  }};

  IkSolution best{};
  float best_score = std::numeric_limits<float>::infinity();

  for (const auto &dir : candidate_dirs) {
    const Point j2 = {m3[0] + l4 * dir[0], m3[1] + l4 * dir[1]};
    const Point j3 = {m3[0] + l4_j3 * dir[0], m3[1] + l4_j3 * dir[1]};

    // J1 is where the M2 crank circle meets the coupler circle around J3
    const float dx = j3[0] - m2[0];
    const float dy = j3[1] - m2[1];
    const float d = std::hypot(dx, dy);
    if (d < 1e-9f) {
      continue;
    }
    const float a = (d * d + l2 * l2 - l3 * l3) / (2.0f * d);
    const float h_sq = l2 * l2 - a * a;
    // The crank and coupler circles do not meet: this bar direction cannot be
    // closed. Only absorb round-off, otherwise J1 would not satisfy |J1-J3|=L3.
    if (h_sq < -1e-2f) {
      continue;
    }
    const float h = std::sqrt(std::max(h_sq, 0.0f));
    const float vx = dx / d;
    const float vy = dy / d;
    const float px = -vy;
    const float py = vx;

    const std::array<Point, 2> candidate_j1 = {{
        {m2[0] + a * vx + h * px, m2[1] + a * vy + h * py},
        {m2[0] + a * vx - h * px, m2[1] + a * vy - h * py},
    }};

    for (const auto &j1 : candidate_j1) {
      if (!is_crossed_branch(m1, m2, j1, j2) || !is_bar_side_ok(j1, j2, j3)) {
        continue;
      }
      IkSolution candidate{};
      if (!complete_solution(m1, m2, m3, j1, j2, j3, config, candidate)) {
        continue;
      }
      // More than one assembly can pass the pin invariant (at full-down a
      // second one exists with the M2 crank swung forward, theta2 ~ 19 deg
      // instead of ~178 deg). Prefer the branch nearest the reference pose's
      // crank angles: the machine never leaves that assembly.
      const float score =
          std::fabs(normalize_angle_deg(candidate.theta1_deg - branch_reference.m1_angle_deg)) +
          std::fabs(normalize_angle_deg(candidate.theta2_deg - branch_reference.m2_angle_deg));
      if (score < best_score) {
        best = candidate;
        best_score = score;
      }
    }
  }

  if (best_score == std::numeric_limits<float>::infinity()) {
    return false;
  }
  solution = best;
  return true;
}

bool solve_ik_for_m3_reference(float x_rel, float y_rel, const IkReference &reference,
                              IkSolution &solution, const GeometryConfig &config) {
  IkSolution raw_solution{};
  if (!solve_ik_for_m3(x_rel + reference.x, y_rel + reference.y, raw_solution, config,
                       reference)) {
    return false;
  }
  solution = apply_reference(raw_solution, reference);
  return true;
}

bool solve_fk(float theta1_deg, float theta2_deg, IkSolution &solution,
              const GeometryConfig &config) {
  const Point m1 = {config.m1_x, config.m1_y};
  const Point m2 = {config.m2_x, config.m2_y};

  const float l1 = config.crank_length_m1_to_j2;
  const float l2 = config.crank_length_m2_to_j1;
  const float l3 = config.bar_length_j1_to_j3;
  const float l4 = config.bar_length_m3_to_j2;
  // J3 sits on the rigid bar between M3 and J2, so its distance from J2 is fixed
  const float l_j2_j3 = l4 - config.bar_length_m3_to_j3;
  if (l_j2_j3 <= 0.0f) {
    return false;
  }

  const float t1 = radians(theta1_deg);
  const float t2 = radians(theta2_deg);
  const Point j2 = {m1[0] + l1 * std::cos(t1), m1[1] + l1 * std::sin(t1)};
  const Point j1 = {m2[0] + l2 * std::cos(t2), m2[1] + l2 * std::sin(t2)};
  if (!is_crossed_branch(m1, m2, j1, j2)) {
    return false;
  }

  // J3 closes the loop: |J3 - J1| = L3 (coupler) and |J3 - J2| = L4 - L4_J3.
  const float dx = j2[0] - j1[0];
  const float dy = j2[1] - j1[1];
  const float d = std::hypot(dx, dy);
  if (d < 1e-9f || d > l3 + l_j2_j3 || d < std::fabs(l3 - l_j2_j3)) {
    return false; // the coupler cannot span J1 -> J2
  }
  const float a = (d * d + l3 * l3 - l_j2_j3 * l_j2_j3) / (2.0f * d);
  const float h = std::sqrt(std::max(l3 * l3 - a * a, 0.0f));
  const float vx = dx / d;
  const float vy = dy / d;

  // The two circle intersections mirror across the J1->J2 line; only the CCW
  // one is the as-built assembly mode (see is_bar_side_ok), which makes the
  // FK unique. The host test's ik -> fk sweep verifies this is the exact
  // inverse of the IK over the whole reachable workspace.
  const Point j3 = {j1[0] + a * vx - h * vy, j1[1] + a * vy + h * vx};
  if (!is_bar_side_ok(j1, j2, j3)) {
    return false; // h == 0: the coupler is folded onto the bar (singular)
  }

  // M3 is on the J2 -> J3 line, L4 from J2
  const float bx = j3[0] - j2[0];
  const float by = j3[1] - j2[1];
  const float b = std::hypot(bx, by);
  if (b < 1e-9f) {
    return false;
  }
  const Point m3 = {j2[0] + l4 * bx / b, j2[1] + l4 * by / b};

  IkSolution candidate{};
  if (!complete_solution(m1, m2, m3, j1, j2, j3, config, candidate)) {
    return false;
  }
  solution = candidate;
  return true;
}

bool solve_fk_reference(float theta1_rel_deg, float theta2_rel_deg, const IkReference &reference,
                        IkSolution &solution, const GeometryConfig &config) {
  IkSolution raw_solution{};
  if (!solve_fk(theta1_rel_deg + reference.m1_angle_deg, theta2_rel_deg + reference.m2_angle_deg,
                raw_solution, config)) {
    return false;
  }
  solution = apply_reference(raw_solution, reference);
  return true;
}

} // namespace stem
