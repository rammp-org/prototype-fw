#include "ik_5bar.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace stem {
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegreesPerRad = 57.2957795130823208768f;

inline float degrees(float radians) {
  return radians * kDegreesPerRad;
}

inline float radians(float degrees) {
  return degrees / kDegreesPerRad;
}

inline float cross2d(const std::array<float, 2> &a,
                    const std::array<float, 2> &b,
                    const std::array<float, 2> &c) {
  return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
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

bool solve_ik_for_m3(float x_rel, float y_rel, IkSolution &solution, const GeometryConfig &config) {
  const std::array<float, 2> m1 = {config.m1_x, config.m1_y};
  const std::array<float, 2> m2 = {config.m2_x, config.m2_y};

  const std::array<float, 2> m3 = {x_rel, y_rel};
  const float r = std::hypot(m3[0], m3[1]);
  if (r < 1e-9f) {
    return false;
  }

  const float l1 = config.crank_length_m1_to_j2;
  const float l2 = config.crank_length_m2_to_j1;
  const float l3 = config.bar_length_j1_to_j3;
  const float l4 = config.bar_length_m3_to_j2;
  const float l4_j3 = config.bar_length_m3_to_j3;
  const float l4_j5 = config.bar_length_m3_to_j5;
  const float j5_perp_offset = config.bar_length_j5_perp_offset;

  const float dot_p_u = (l1 * l1 - l4 * l4 - r * r) / (2.0f * l4);
  const float cos_beta = std::clamp(dot_p_u / r, -1.0f, 1.0f);
  const float alpha = std::atan2(m3[1], m3[0]);
  const float beta = std::acos(cos_beta);

  const std::array<std::array<float, 2>, 2> candidate_dirs = {{
      {std::cos(alpha + beta), std::sin(alpha + beta)},
      {std::cos(alpha - beta), std::sin(alpha - beta)},
  }};

  IkSolution best{};
  float best_score = std::numeric_limits<float>::infinity();

  for (const auto &dir : candidate_dirs) {
    const std::array<float, 2> j2 = {m3[0] + l4 * dir[0], m3[1] + l4 * dir[1]};
    const std::array<float, 2> j3 = {m3[0] + l4_j3 * dir[0], m3[1] + l4_j3 * dir[1]};

    const float dx = j3[0] - m2[0];
    const float dy = j3[1] - m2[1];
    const float d = std::hypot(dx, dy);
    if (d < 1e-9f) {
      continue;
    }

    const float a = (d * d + l2 * l2 - l3 * l3) / (2.0f * d);
    const float h = std::sqrt(std::max(l2 * l2 - a * a, 0.0f));

    const float vx = dx / d;
    const float vy = dy / d;
    const float px = -vy;
    const float py = vx;

    const std::array<std::array<float, 2>, 2> candidate_j1 = {{
        {m2[0] + a * vx + h * px, m2[1] + a * vy + h * py},
        {m2[0] + a * vx - h * px, m2[1] + a * vy - h * py},
    }};

    for (const auto &j1 : candidate_j1) {
      if (cross2d(m1, j2, j1) <= 0.0f || cross2d(m2, j1, j2) >= 0.0f) {
        continue;
      }

      const float theta1 = std::atan2(j2[1] - m1[1], j2[0] - m1[0]);
      const float theta2 = std::atan2(j1[1] - m2[1], j1[0] - m2[0]);

      const float theta1_deg = degrees(theta1);
      const float theta2_deg = degrees(theta2);
      const float phi1_deg = theta1_deg;
      const float phi2_deg = theta2_deg;
      const float score = std::fabs(theta1_deg) + std::fabs(theta2_deg);

      const std::array<float, 2> bar_dir = {j2[0] - m3[0], j2[1] - m3[1]};
      const float bar_mag = std::hypot(bar_dir[0], bar_dir[1]);
      if (bar_mag < 1e-9f) {
        continue;
      }

      const float ux = bar_dir[0] / bar_mag;
      const float uy = bar_dir[1] / bar_mag;
      const float nx = -uy;
      const float ny = ux;
      const std::array<float, 2> j5 = {
          m3[0] + l4_j5 * ux + j5_perp_offset * nx,
          m3[1] + l4_j5 * uy + j5_perp_offset * ny,
      };

      const std::array<float, 2> plate_vec = {m3[0] - j2[0], m3[1] - j2[1]};
      const float plate_angle_abs = std::atan2(plate_vec[1], plate_vec[0]);
      const float plate_angle_deg = degrees(plate_angle_abs);

      IkSolution candidate{};
      candidate.m3 = m3;
      candidate.j1 = j1;
      candidate.j2 = j2;
      candidate.j3 = j3;
      candidate.j5 = j5;
      candidate.theta1_deg = theta1_deg;
      candidate.theta2_deg = theta2_deg;
      candidate.phi1_deg = phi1_deg;
      candidate.phi2_deg = phi2_deg;
      candidate.plate_angle_deg = plate_angle_deg;
      candidate.m3_angle_deg = plate_angle_deg;
      candidate.x_ref = 0.0f;
      candidate.y_ref = 0.0f;
      candidate.m1_angle_ref_deg = 0.0f;
      candidate.m2_angle_ref_deg = 0.0f;
      candidate.m3_angle_ref_deg = 0.0f;

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
                              IkSolution &solution,
                              const GeometryConfig &config) {
  const float x_analysis = x_rel + reference.x;
  const float y_analysis = y_rel + reference.y;

  IkSolution raw_solution{};
  if (!solve_ik_for_m3(x_analysis, y_analysis, raw_solution, config)) {
    return false;
  }

  IkSolution transformed = raw_solution;
  transformed.x_ref = reference.x;
  transformed.y_ref = reference.y;
  transformed.m1_angle_ref_deg = reference.m1_angle_deg;
  transformed.m2_angle_ref_deg = reference.m2_angle_deg;
  transformed.m3_angle_ref_deg = reference.m3_angle_deg;

  transformed.theta1_deg = normalize_angle_deg(raw_solution.theta1_deg - reference.m1_angle_deg);
  transformed.theta2_deg = normalize_angle_deg(raw_solution.theta2_deg - reference.m2_angle_deg);
  transformed.phi1_deg = normalize_angle_deg(raw_solution.phi1_deg - reference.m1_angle_deg);
  transformed.phi2_deg = normalize_angle_deg(raw_solution.phi2_deg - reference.m2_angle_deg);
  transformed.plate_angle_deg = normalize_angle_deg(raw_solution.plate_angle_deg - reference.m3_angle_deg);
  transformed.m3_angle_deg = normalize_angle_deg(raw_solution.m3_angle_deg - reference.m3_angle_deg);

  solution = transformed;
  return true;
}

} // namespace stem
